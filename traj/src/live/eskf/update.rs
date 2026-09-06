//! Schmidt measurement updates and innovation solves.

use super::{
    ConsiderMeasurementCross, Eskf, EskfError, GapNavCrossCovariance, MAX_MEASUREMENT_DIM,
    MeasurementConsiderJacobian, MeasurementMatrix, MeasurementNavJacobian,
    MeasurementSampleJacobian, MeasurementVector, NavMeasurementCross, RtsUpdateCapture,
    SampleMeasurementCross, covariance::condition_navigation_covariance, gnss::UpdateDecision,
    matrix::multiply_nav_gap,
};
use crate::live::{
    preintegration::{BIAS_DIM, ImuSampleCovariance},
    state::{ATT, NAV_DIM, NavMatrix, NavVector},
};

impl Eskf {
    pub(super) fn linear_update(
        &mut self,
        measurement: LinearMeasurement,
        soft_gate: f32,
        hard_gate: f32,
        maximum_inflation: f32,
        sample_covariance: Option<&ImuSampleCovariance>,
        sample_cross: Option<&mut GapNavCrossCovariance>,
        capture: Option<&mut RtsUpdateCapture>,
    ) -> Result<UpdateDecision, EskfError> {
        if measurement.dimension == 0 || measurement.dimension > MAX_MEASUREMENT_DIM {
            return Err(EskfError::InvalidMeasurement);
        }

        let (u_nav, u_consider, u_sample, mut innovation) =
            self.innovation_terms(&measurement, sample_covariance, sample_cross.as_deref());
        let initial_cholesky = cholesky_active(&innovation, measurement.dimension)
            .ok_or(EskfError::InvalidInnovationCovariance)?;
        let solved_residual = solve_cholesky(
            &initial_cholesky,
            &measurement.residual,
            measurement.dimension,
        );
        let nis = active_dot(
            &measurement.residual,
            &solved_residual,
            measurement.dimension,
        );
        if !nis.is_finite() || nis < 0.0 {
            return Err(EskfError::NumericalFailure);
        }
        if nis > hard_gate {
            return Ok(UpdateDecision::RejectedInnovation { nis });
        }

        let inflation = if nis > soft_gate {
            (nis / soft_gate).min(maximum_inflation)
        } else {
            1.0
        };
        if inflation > 1.0 {
            for row in 0..measurement.dimension {
                for column in 0..measurement.dimension {
                    innovation[(row, column)] +=
                        measurement.noise[(row, column)] * (inflation - 1.0);
                }
            }
        }
        let cholesky = cholesky_active(&innovation, measurement.dimension)
            .ok_or(EskfError::InvalidInnovationCovariance)?;
        let mut kalman_gain = NavMeasurementCross::zeros();
        for nav_row in 0..NAV_DIM {
            let rhs = MeasurementVector::from_fn(|index, _| u_nav[(nav_row, index)]);
            let solution = solve_cholesky(&cholesky, &rhs, measurement.dimension);
            for column in 0..measurement.dimension {
                kalman_gain[(nav_row, column)] = solution[column];
            }
        }

        let mut correction = NavVector::zeros();
        for row in 0..NAV_DIM {
            for column in 0..measurement.dimension {
                correction[row] += kalman_gain[(row, column)] * measurement.residual[column];
            }
        }
        let mut candidate_state = self.state;
        let attitude_reset = candidate_state
            .inject(&correction)
            .map_err(EskfError::State)?;

        // Joseph-equivalent stabilized block update:
        // P+ = P - K U' - U K' + K S K', U = P H'.
        let mut candidate_covariance = self.covariance;
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                let mut k_u_t = 0.0;
                let mut u_k_t = 0.0;
                let mut k_s_k_t = 0.0;
                for a in 0..measurement.dimension {
                    k_u_t += kalman_gain[(row, a)] * u_nav[(column, a)];
                    u_k_t += u_nav[(row, a)] * kalman_gain[(column, a)];
                    for b in 0..measurement.dimension {
                        k_s_k_t +=
                            kalman_gain[(row, a)] * innovation[(a, b)] * kalman_gain[(column, b)];
                    }
                }
                candidate_covariance[(row, column)] += -k_u_t - u_k_t + k_s_k_t;
            }
        }
        let mut candidate_cross = self.nav_consider_covariance;
        for row in 0..NAV_DIM {
            for shared in 0..self.active_consider {
                let mut reduction = 0.0;
                for measurement_column in 0..measurement.dimension {
                    reduction += kalman_gain[(row, measurement_column)]
                        * u_consider[(shared, measurement_column)];
                }
                candidate_cross[(row, shared)] -= reduction;
            }
        }
        let mut candidate_gap_cross = self.gap_nav_cross_covariance;
        if self.gap_origin.is_some() {
            for row in 0..NAV_DIM {
                for latent in 0..BIAS_DIM {
                    let mut reduction = 0.0;
                    for measurement_column in 0..measurement.dimension {
                        let mut measurement_gap_cross = 0.0;
                        for nav in 0..NAV_DIM {
                            measurement_gap_cross += measurement.h_nav[(measurement_column, nav)]
                                * self.gap_nav_cross_covariance[(nav, latent)];
                        }
                        reduction += kalman_gain[(row, measurement_column)] * measurement_gap_cross;
                    }
                    candidate_gap_cross[(row, latent)] -= reduction;
                }
            }
        }

        let mut candidate_sample_cross = sample_cross.as_deref().copied();
        if let Some(sample_cross) = candidate_sample_cross.as_mut() {
            for row in 0..NAV_DIM {
                for latent in 0..BIAS_DIM {
                    let mut reduction = 0.0;
                    for measurement_column in 0..measurement.dimension {
                        reduction += kalman_gain[(row, measurement_column)]
                            * u_sample[(latent, measurement_column)];
                    }
                    sample_cross[(row, latent)] -= reduction;
                }
            }
        }

        let mut reset = NavMatrix::identity();
        reset
            .fixed_view_mut::<3, 3>(ATT, ATT)
            .copy_from(&attitude_reset);
        candidate_covariance = reset * candidate_covariance * reset.transpose();
        candidate_cross = reset * candidate_cross;
        candidate_gap_cross = reset * candidate_gap_cross;
        if let Some(sample_cross) = candidate_sample_cross.as_mut() {
            let prior = *sample_cross;
            multiply_nav_gap(&reset, &prior, sample_cross);
            if sample_cross.iter().any(|value| !value.is_finite()) {
                return Err(EskfError::NumericalFailure);
            }
        }

        if candidate_cross
            .iter()
            .chain(candidate_gap_cross.iter())
            .any(|value| !value.is_finite())
        {
            return Err(EskfError::NumericalFailure);
        }
        let (repairs, normalized_repair) =
            condition_navigation_covariance(&mut candidate_covariance, &self.covariance_policy)?;
        if let Some(capture) = capture {
            capture.apply_update(&measurement, &kalman_gain, &attitude_reset)?;
        }

        // Every fallible operation is complete. Commit the filter and its
        // caller-owned held-sample cross together, including the attitude
        // reset, so a rejected or failed update cannot consume correlation.
        self.state = candidate_state;
        self.covariance = candidate_covariance;
        self.nav_consider_covariance = candidate_cross;
        self.gap_nav_cross_covariance = candidate_gap_cross;
        self.covariance_repairs = self.covariance_repairs.saturating_add(repairs);
        self.total_normalized_repair += normalized_repair;
        if let (Some(target), Some(candidate)) = (sample_cross, candidate_sample_cross) {
            *target = candidate;
        }

        if inflation > 1.0 {
            Ok(UpdateDecision::Downweighted { nis, inflation })
        } else {
            Ok(UpdateDecision::Fused { nis })
        }
    }

    pub(super) fn innovation_terms(
        &self,
        measurement: &LinearMeasurement,
        sample_covariance: Option<&ImuSampleCovariance>,
        sample_cross: Option<&GapNavCrossCovariance>,
    ) -> (
        NavMeasurementCross,
        ConsiderMeasurementCross,
        SampleMeasurementCross,
        MeasurementMatrix,
    ) {
        let mut u_nav = NavMeasurementCross::zeros();
        let mut u_consider = ConsiderMeasurementCross::zeros();
        let mut u_sample = SampleMeasurementCross::zeros();
        for nav in 0..NAV_DIM {
            for measurement_column in 0..measurement.dimension {
                for other_nav in 0..NAV_DIM {
                    u_nav[(nav, measurement_column)] += self.covariance[(nav, other_nav)]
                        * measurement.h_nav[(measurement_column, other_nav)];
                }
                for shared in 0..self.active_consider {
                    u_nav[(nav, measurement_column)] += self.nav_consider_covariance[(nav, shared)]
                        * measurement.h_consider[(measurement_column, shared)];
                }
                if let Some(cross) = sample_cross {
                    for latent in 0..BIAS_DIM {
                        u_nav[(nav, measurement_column)] += cross[(nav, latent)]
                            * measurement.h_sample[(measurement_column, latent)];
                    }
                }
            }
        }
        for shared in 0..self.active_consider {
            for measurement_column in 0..measurement.dimension {
                for nav in 0..NAV_DIM {
                    u_consider[(shared, measurement_column)] += self.nav_consider_covariance
                        [(nav, shared)]
                        * measurement.h_nav[(measurement_column, nav)];
                }
                for other_shared in 0..self.active_consider {
                    u_consider[(shared, measurement_column)] += self.consider_covariance
                        [(shared, other_shared)]
                        * measurement.h_consider[(measurement_column, other_shared)];
                }
            }
        }
        // Fixed shared parameters and held sample errors have independent
        // priors and both use zero Schmidt gain. Their mutual covariance
        // therefore stays zero even when each becomes correlated with x.
        if let (Some(covariance), Some(cross)) = (sample_covariance, sample_cross) {
            for latent in 0..BIAS_DIM {
                for measurement_column in 0..measurement.dimension {
                    for nav in 0..NAV_DIM {
                        u_sample[(latent, measurement_column)] +=
                            cross[(nav, latent)] * measurement.h_nav[(measurement_column, nav)];
                    }
                    for other_latent in 0..BIAS_DIM {
                        u_sample[(latent, measurement_column)] += covariance
                            [(latent, other_latent)]
                            * measurement.h_sample[(measurement_column, other_latent)];
                    }
                }
            }
        }
        let mut innovation = measurement.noise;
        for row in 0..measurement.dimension {
            for column in 0..measurement.dimension {
                for nav in 0..NAV_DIM {
                    innovation[(row, column)] +=
                        measurement.h_nav[(row, nav)] * u_nav[(nav, column)];
                }
                for shared in 0..self.active_consider {
                    innovation[(row, column)] +=
                        measurement.h_consider[(row, shared)] * u_consider[(shared, column)];
                }
                for latent in 0..BIAS_DIM {
                    innovation[(row, column)] +=
                        measurement.h_sample[(row, latent)] * u_sample[(latent, column)];
                }
            }
        }
        for row in 0..measurement.dimension {
            for column in 0..row {
                let symmetric = 0.5 * (innovation[(row, column)] + innovation[(column, row)]);
                innovation[(row, column)] = symmetric;
                innovation[(column, row)] = symmetric;
            }
        }
        (u_nav, u_consider, u_sample, innovation)
    }
}

#[derive(Clone, Copy)]
pub(super) struct LinearMeasurement {
    pub(super) dimension: usize,
    pub(super) residual: MeasurementVector,
    pub(super) h_nav: MeasurementNavJacobian,
    pub(super) h_consider: MeasurementConsiderJacobian,
    pub(super) h_sample: MeasurementSampleJacobian,
    pub(super) noise: MeasurementMatrix,
}

impl LinearMeasurement {
    pub(super) fn zeros() -> Self {
        Self {
            dimension: 0,
            residual: MeasurementVector::zeros(),
            h_nav: MeasurementNavJacobian::zeros(),
            h_consider: MeasurementConsiderJacobian::zeros(),
            h_sample: MeasurementSampleJacobian::zeros(),
            noise: MeasurementMatrix::zeros(),
        }
    }
}

pub(super) fn cholesky_active(
    matrix: &MeasurementMatrix,
    dimension: usize,
) -> Option<MeasurementMatrix> {
    let mut lower = MeasurementMatrix::zeros();
    for row in 0..dimension {
        for column in 0..=row {
            let mut value = 0.5 * (matrix[(row, column)] + matrix[(column, row)]);
            for k in 0..column {
                value -= lower[(row, k)] * lower[(column, k)];
            }
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return None;
                }
                lower[(row, column)] = crate::scalar_math::sqrt(value);
            } else {
                lower[(row, column)] = value / lower[(column, column)];
            }
        }
    }
    Some(lower)
}

fn solve_cholesky(
    lower: &MeasurementMatrix,
    rhs: &MeasurementVector,
    dimension: usize,
) -> MeasurementVector {
    let mut intermediate = MeasurementVector::zeros();
    for row in 0..dimension {
        let mut value = rhs[row];
        for column in 0..row {
            value -= lower[(row, column)] * intermediate[column];
        }
        intermediate[row] = value / lower[(row, row)];
    }
    let mut result = MeasurementVector::zeros();
    for row in (0..dimension).rev() {
        let mut value = intermediate[row];
        for column in (row + 1)..dimension {
            value -= lower[(column, row)] * result[column];
        }
        result[row] = value / lower[(row, row)];
    }
    result
}

fn active_dot(left: &MeasurementVector, right: &MeasurementVector, dimension: usize) -> f32 {
    let mut result = 0.0;
    for index in 0..dimension {
        result += left[index] * right[index];
    }
    result
}
