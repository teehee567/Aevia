//! Nominal and covariance propagation through preintegrated IMU batches.

#[cfg(test)]
use super::ProcessNoise;
use super::{
    Eskf, EskfError, EskfPropagationScratch, GapNavCrossCovariance, MAX_CONSIDER,
    NavConsiderCovariance, PreintegrationNavJacobian,
    discretization::{
        add_bias_random_walk_discrete_covariance, continuous_state_matrix_into,
        state_transition_into,
    },
    matrix::{
        add_symmetric_cross_product, multiply_gamma_consider, multiply_nav, multiply_nav_consider,
        multiply_nav_gap, multiply_nav_right_transpose, multiply_preintegration_gap,
        symmetrize_nav,
    },
};
use crate::{
    live::{
        preintegration::{
            BIAS_DIM, GapDerivativeCovariance, ImuSampleCovariance, PREINT_DIM, PreintegratedBatch,
        },
        smoothing::{CONSIDER_START, GAP_START, SAMPLE_START},
        state::{ATT, MechanizationContext, NAV_DIM, NavMatrix, NavState, POS, VEL, so3_exp},
    },
    time::SessionTime,
};
use nalgebra::Matrix3;

impl Eskf {
    #[cfg(test)]
    #[inline(never)]
    pub(crate) fn propagate(
        &mut self,
        batch: &PreintegratedBatch,
        context: &MechanizationContext,
        scratch: &mut EskfPropagationScratch,
    ) -> Result<(), EskfError> {
        self.propagate_with_imu_sample(batch, context, None, scratch)
    }

    /// Propagates while retaining cross covariance with an interval-average
    /// IMU sample that was already active at the batch start. The returned
    /// sample candidate lives in caller-owned scratch and is committed by the
    /// core only after this entire filter transaction succeeds.
    #[inline(never)]
    pub(crate) fn propagate_with_imu_sample(
        &mut self,
        batch: &PreintegratedBatch,
        context: &MechanizationContext,
        prior_sample_cross: Option<&GapNavCrossCovariance>,
        scratch: &mut EskfPropagationScratch,
    ) -> Result<(), EskfError> {
        if batch.start != self.state.time {
            return Err(EskfError::TimeMismatch);
        }
        validate_process_consider_block(batch, self.active_consider)?;
        validate_imu_sample_transition(batch, prior_sample_cross)?;
        let corrected = batch
            .corrected(self.state.accel_bias_b, self.state.gyro_bias_b)
            .map_err(EskfError::Preintegration)?;
        let dt = corrected
            .duration_seconds()
            .map_err(EskfError::Preintegration)?;
        if dt > 0.050 {
            return Err(EskfError::PropagationIntervalTooLong);
        }
        validate_gap_transition(self.gap_origin, &self.gap_derivative_covariance, &corrected)?;

        let old_state = self.state;
        let force_rotation = propagate_nominal(&mut self.state, &corrected, context)?;
        continuous_state_matrix_into(&old_state, &corrected, context, &mut scratch.continuous);
        state_transition_into(
            &scratch.continuous,
            dt,
            &mut scratch.transition,
            &mut scratch.nav_b,
            &mut scratch.nav_c,
        )?;
        preintegration_nav_mapping_into(&force_rotation, &mut scratch.mapping);
        process_consider_sensitivity(
            &corrected,
            &force_rotation,
            dt,
            self.active_consider,
            &mut scratch.gamma,
        )?;
        // Retain the actual mean-error transition before the bias-noise
        // discretizer reuses `scratch.transition` for its quadrature powers.
        scratch.rts_transition.nav.fill(0.0);
        scratch.rts_transition.retain_sample = false;
        scratch.rts_transition.retain_gap = false;
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                scratch.rts_transition.nav[(row, column)] = scratch.transition[(row, column)];
            }
            for shared in 0..self.active_consider {
                scratch.rts_transition.nav[(row, CONSIDER_START + shared)] =
                    scratch.gamma[(row, shared)];
            }
        }

        multiply_nav(&scratch.transition, &self.covariance, &mut scratch.nav_b);
        multiply_nav_right_transpose(&scratch.nav_b, &scratch.transition, &mut scratch.nav_a);
        add_mapped_preintegration_covariance(&corrected, &scratch.mapping, &mut scratch.nav_a)?;
        multiply_nav_consider(
            &scratch.transition,
            &self.nav_consider_covariance,
            &mut scratch.cross_a,
        );
        self.nav_consider_covariance = scratch.cross_a;
        multiply_gamma_consider(
            &scratch.gamma,
            &self.consider_covariance,
            self.active_consider,
            &mut scratch.cross_a,
        );
        multiply_nav_gap(
            &scratch.transition,
            &self.gap_nav_cross_covariance,
            &mut scratch.gap_a,
        );

        let mut next_gap_origin = None;
        let mut next_gap_derivative_covariance = GapDerivativeCovariance::zeros();
        if let Some(gap) = corrected.gap {
            multiply_preintegration_gap(&scratch.mapping, &gap.jacobian, &mut scratch.gap_b);
            if self.gap_origin == Some(gap.origin) {
                for row in 0..NAV_DIM {
                    for latent in 0..BIAS_DIM {
                        scratch.rts_transition.nav[(row, GAP_START + latent)] =
                            scratch.gap_b[(row, latent)];
                    }
                }
                scratch.rts_transition.retain_gap = gap.active_at_end;
            }
            // `batch.covariance` already contains L C L'. When this is a
            // continuation, add the two cross terms with the retained P_xz.
            add_symmetric_cross_product(&mut scratch.nav_a, &scratch.gap_a, &scratch.gap_b);
            for row in 0..NAV_DIM {
                for latent in 0..BIAS_DIM {
                    let mut value = scratch.gap_a[(row, latent)];
                    for source in 0..BIAS_DIM {
                        value += scratch.gap_b[(row, source)]
                            * gap.derivative_covariance[(source, latent)];
                    }
                    self.gap_nav_cross_covariance[(row, latent)] = value;
                }
            }
            if gap.active_at_end {
                next_gap_origin = Some(gap.origin);
                next_gap_derivative_covariance = gap.derivative_covariance;
            }
        }
        // With fixed uncertain parameters c and x+ = Phi*x + Gamma*c + w:
        // Pxx+ = Phi Pxx Phi' + Phi Pxc Gamma' + Gamma Pcx Phi'
        //        + Gamma Pcc Gamma' + Q,
        // Pxc+ = Phi Pxc + Gamma Pcc. The consider mean and Pcc stay fixed.
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                for shared in 0..self.active_consider {
                    scratch.nav_a[(row, column)] += self.nav_consider_covariance[(row, shared)]
                        * scratch.gamma[(column, shared)]
                        + scratch.gamma[(row, shared)]
                            * self.nav_consider_covariance[(column, shared)]
                        + scratch.cross_a[(row, shared)] * scratch.gamma[(column, shared)];
                }
            }
        }
        self.covariance = scratch.nav_a;
        for row in 0..NAV_DIM {
            for shared in 0..MAX_CONSIDER {
                self.nav_consider_covariance[(row, shared)] += scratch.cross_a[(row, shared)];
            }
        }

        // `corrected.covariance` already contains every B S B' sample
        // marginal. If the leading sample was active before this batch, its
        // state cross covariance contributes the two missing Phi*C/B cross
        // terms. Preserve the next active sample cross in the first six cold
        // scratch columns until all remaining fallible propagation work has
        // succeeded.
        scratch.cross_a.fill(0.0);
        if let Some(leading) = corrected.leading_sample {
            let prior = prior_sample_cross.ok_or(EskfError::ImuSampleLatentMismatch)?;
            multiply_nav_gap(&scratch.transition, prior, &mut scratch.gap_a);
            multiply_preintegration_gap(&scratch.mapping, &leading.jacobian, &mut scratch.gap_b);
            for row in 0..NAV_DIM {
                for latent in 0..BIAS_DIM {
                    scratch.rts_transition.nav[(row, SAMPLE_START + latent)] =
                        scratch.gap_b[(row, latent)];
                }
            }
            scratch.rts_transition.retain_sample = leading.active_at_end;
            add_symmetric_cross_product(&mut self.covariance, &scratch.gap_a, &scratch.gap_b);
            if leading.active_at_end {
                store_sample_cross_candidate(
                    &scratch.gap_a,
                    &scratch.gap_b,
                    &leading.covariance,
                    &mut scratch.cross_a,
                );
            }
        }
        if let Some(trailing) = corrected.trailing_sample {
            if corrected
                .leading_sample
                .is_some_and(|leading| leading.active_at_end)
            {
                return Err(EskfError::ImuSampleLatentMismatch);
            }
            multiply_preintegration_gap(&scratch.mapping, &trailing.jacobian, &mut scratch.gap_b);
            store_new_sample_cross_candidate(
                &scratch.gap_b,
                &trailing.covariance,
                &mut scratch.cross_a,
            );
        }
        if !self
            .covariance
            .iter()
            .chain(scratch.cross_a.iter())
            .all(|value| value.is_finite())
        {
            return Err(EskfError::NumericalFailure);
        }
        self.gap_origin = next_gap_origin;
        self.gap_derivative_covariance = next_gap_derivative_covariance;
        if self.gap_origin.is_none() {
            self.gap_nav_cross_covariance.fill(0.0);
        }
        add_bias_random_walk_discrete_covariance(
            &scratch.continuous,
            dt,
            self.process_noise,
            &mut self.covariance,
            &mut scratch.transition,
            &mut scratch.nav_a,
            &mut scratch.nav_b,
            &mut scratch.nav_c,
        )?;
        self.condition_covariance()?;
        for row in 0..NAV_DIM {
            for latent in 0..BIAS_DIM {
                scratch.gap_a[(row, latent)] = scratch.cross_a[(row, latent)];
            }
        }
        Ok(())
    }
}

/// Propagates only the nominal state. The low-latency predictor deliberately
/// shares this exact implementation but never presents covariance-grade output.
pub(crate) fn propagate_nominal(
    state: &mut NavState,
    corrected: &PreintegratedBatch,
    context: &MechanizationContext,
) -> Result<Matrix3<f32>, EskfError> {
    if corrected.start != state.time {
        return Err(EskfError::TimeMismatch);
    }
    let dt = corrected
        .duration_seconds()
        .map_err(EskfError::Preintegration)?;
    if dt > 0.050 {
        return Err(EskfError::PropagationIntervalTooLong);
    }
    let old_state = *state;
    let old_rotation = old_state
        .orientation_n_from_b
        .to_rotation_matrix()
        .into_inner();
    let earth_delta = so3_exp(context.earth_rate_n * -dt);
    let earth_half = so3_exp(context.earth_rate_n * (-0.5 * dt));
    let force_rotation = earth_half.to_rotation_matrix().into_inner() * old_rotation;
    let specific_delta_v_n = force_rotation * corrected.delta_velocity_b0;
    let specific_delta_p_n = force_rotation * corrected.delta_position_b0;

    let gravity_start = context.gravity_at(&old_state.position_n);
    let specific_acceleration = specific_delta_v_n / dt;
    let acceleration_start = specific_acceleration + gravity_start
        - 2.0 * context.earth_rate_n.cross(&old_state.velocity_n);
    let velocity_mid = old_state.velocity_n + acceleration_start * (0.5 * dt);
    let position_mid = old_state.position_n + old_state.velocity_n * (0.5 * dt);
    let external_mid =
        context.gravity_at(&position_mid) - 2.0 * context.earth_rate_n.cross(&velocity_mid);

    state.position_n = old_state.position_n
        + old_state.velocity_n * dt
        + specific_delta_p_n
        + external_mid * (0.5 * dt * dt);
    state.velocity_n = old_state.velocity_n + specific_delta_v_n + external_mid * dt;
    state.orientation_n_from_b =
        earth_delta * old_state.orientation_n_from_b * corrected.delta_rotation;
    state.orientation_n_from_b.renormalize();
    state.time = corrected.end;
    if !state.is_finite() {
        return Err(EskfError::NumericalFailure);
    }
    Ok(force_rotation)
}

fn validate_process_consider_block(
    batch: &PreintegratedBatch,
    active_consider: usize,
) -> Result<(), EskfError> {
    if active_consider > MAX_CONSIDER {
        return Err(EskfError::InvalidConsiderBlock);
    }
    let has_sensitivity = batch
        .mean_specific_force_consider_jacobian
        .iter()
        .chain(batch.mean_angular_rate_consider_jacobian.iter())
        .any(|value| *value != 0.0);
    if !batch
        .mean_specific_force_consider_jacobian
        .iter()
        .chain(batch.mean_angular_rate_consider_jacobian.iter())
        .all(|value| value.is_finite())
    {
        return Err(EskfError::InvalidConsiderBlock);
    }
    match batch.calibration_consider_start {
        Some(start) if usize::from(start).saturating_add(3) <= active_consider => Ok(()),
        Some(_) => Err(EskfError::InvalidConsiderBlock),
        None if has_sensitivity => Err(EskfError::InvalidConsiderBlock),
        None => Ok(()),
    }
}

fn process_consider_sensitivity(
    batch: &PreintegratedBatch,
    force_rotation: &Matrix3<f32>,
    dt: f32,
    active_consider: usize,
    result: &mut NavConsiderCovariance,
) -> Result<(), EskfError> {
    validate_process_consider_block(batch, active_consider)?;
    result.fill(0.0);
    let Some(start) = batch.calibration_consider_start.map(usize::from) else {
        return Ok(());
    };
    let force_sensitivity = force_rotation * batch.mean_specific_force_consider_jacobian;
    result
        .fixed_view_mut::<3, 3>(POS, start)
        .copy_from(&(force_sensitivity * (0.5 * dt * dt)));
    result
        .fixed_view_mut::<3, 3>(VEL, start)
        .copy_from(&(force_sensitivity * dt));
    result
        .fixed_view_mut::<3, 3>(ATT, start)
        .copy_from(&(batch.mean_angular_rate_consider_jacobian * dt));
    Ok(())
}

pub(super) fn preintegration_nav_mapping_into(
    force_rotation: &Matrix3<f32>,
    mapping: &mut PreintegrationNavJacobian,
) {
    mapping.fill(0.0);
    mapping
        .fixed_view_mut::<3, 3>(ATT, 0)
        .copy_from(&Matrix3::identity());
    mapping
        .fixed_view_mut::<3, 3>(VEL, 3)
        .copy_from(force_rotation);
    mapping
        .fixed_view_mut::<3, 3>(POS, 6)
        .copy_from(force_rotation);
}

fn validate_gap_transition(
    active_origin: Option<SessionTime>,
    active_covariance: &GapDerivativeCovariance,
    batch: &PreintegratedBatch,
) -> Result<(), EskfError> {
    match (active_origin, batch.gap) {
        (Some(origin), Some(gap))
            if origin == gap.origin && *active_covariance == gap.derivative_covariance => {}
        (None, Some(gap)) if gap.origin >= batch.start => {}
        (None, None) => {}
        _ => return Err(EskfError::GapLatentMismatch),
    }
    if batch.gap.is_some_and(|gap| {
        !gap.jacobian
            .iter()
            .chain(gap.derivative_covariance.iter())
            .all(|value| value.is_finite())
    }) {
        return Err(EskfError::GapLatentMismatch);
    }
    Ok(())
}

fn validate_imu_sample_transition(
    batch: &PreintegratedBatch,
    prior_sample_cross: Option<&GapNavCrossCovariance>,
) -> Result<(), EskfError> {
    if batch.leading_sample.is_some() && prior_sample_cross.is_none() {
        return Err(EskfError::ImuSampleLatentMismatch);
    }
    if batch
        .leading_sample
        .iter()
        .chain(batch.trailing_sample.iter())
        .flat_map(|sample| sample.covariance.iter().chain(sample.jacobian.iter()))
        .any(|value| !value.is_finite())
        || prior_sample_cross.is_some_and(|cross| cross.iter().any(|value| !value.is_finite()))
    {
        return Err(EskfError::ImuSampleLatentMismatch);
    }
    Ok(())
}

fn store_sample_cross_candidate(
    propagated_prior: &GapNavCrossCovariance,
    mapped_sample: &GapNavCrossCovariance,
    sample_covariance: &ImuSampleCovariance,
    candidate: &mut NavConsiderCovariance,
) {
    for row in 0..NAV_DIM {
        for latent in 0..BIAS_DIM {
            let mut value = propagated_prior[(row, latent)];
            for source in 0..BIAS_DIM {
                value += mapped_sample[(row, source)] * sample_covariance[(source, latent)];
            }
            candidate[(row, latent)] = value;
        }
    }
}

fn store_new_sample_cross_candidate(
    mapped_sample: &GapNavCrossCovariance,
    sample_covariance: &ImuSampleCovariance,
    candidate: &mut NavConsiderCovariance,
) {
    for row in 0..NAV_DIM {
        for latent in 0..BIAS_DIM {
            let mut value = 0.0;
            for source in 0..BIAS_DIM {
                value += mapped_sample[(row, source)] * sample_covariance[(source, latent)];
            }
            candidate[(row, latent)] = value;
        }
    }
}

#[inline(never)]
fn add_mapped_preintegration_covariance(
    batch: &PreintegratedBatch,
    mapping: &PreintegrationNavJacobian,
    target: &mut NavMatrix,
) -> Result<(), EskfError> {
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            for left in 0..PREINT_DIM {
                for right in 0..PREINT_DIM {
                    value += mapping[(row, left)]
                        * batch.covariance[(left, right)]
                        * mapping[(column, right)];
                }
            }
            target[(row, column)] += value;
        }
    }
    symmetrize_nav(target);
    if target.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(EskfError::NumericalFailure)
    }
}

#[cfg(test)]
pub(super) fn map_preintegration_covariance(
    batch: &PreintegratedBatch,
    mapping: &PreintegrationNavJacobian,
    continuous: &NavMatrix,
    dt: f32,
    process_noise: ProcessNoise,
) -> Result<NavMatrix, EskfError> {
    let mut result = NavMatrix::zeros();
    add_mapped_preintegration_covariance(batch, mapping, &mut result)?;
    let mut transition = NavMatrix::zeros();
    let mut covariance = NavMatrix::zeros();
    let mut term = NavMatrix::zeros();
    let mut temporary = NavMatrix::zeros();
    add_bias_random_walk_discrete_covariance(
        continuous,
        dt,
        process_noise,
        &mut result,
        &mut transition,
        &mut covariance,
        &mut term,
        &mut temporary,
    )?;
    Ok(result)
}
