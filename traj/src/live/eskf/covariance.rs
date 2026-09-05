//! Navigation covariance conditioning and PSD checks.

use super::{Eskf, EskfError};
use crate::live::{
    preintegration::covariance_density_is_valid,
    state::{NAV_DIM, NavMatrix},
};
use nalgebra::{Matrix3, SMatrix};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CovariancePolicy {
    /// Characteristic units used before a Cholesky PSD test.
    pub(crate) state_scales: [f32; NAV_DIM],
    pub(crate) minimum_variance: [f32; NAV_DIM],
    pub(crate) repair_initial: f32,
    pub(crate) repair_growth: f32,
    pub(crate) maximum_total_repair: f32,
    pub(crate) maximum_repair_attempts: u8,
}

impl CovariancePolicy {
    #[cfg(test)]
    pub(crate) const fn conservative_candidate() -> Self {
        Self {
            state_scales: [
                10.0, 10.0, 10.0, // position
                10.0, 10.0, 10.0, // velocity
                1.0, 1.0, 1.0, // attitude
                1.0, 1.0, 1.0, // accelerometer bias
                0.1, 0.1, 0.1, // gyro bias
            ],
            minimum_variance: [
                1.0e-10, 1.0e-10, 1.0e-10, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12,
                1.0e-14, 1.0e-14, 1.0e-14, 1.0e-16, 1.0e-16, 1.0e-16,
            ],
            repair_initial: 1.0e-8,
            repair_growth: 10.0,
            maximum_total_repair: 1.0e-4,
            maximum_repair_attempts: 2,
        }
    }

    pub(super) fn is_valid(&self) -> bool {
        self.state_scales
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
            && self
                .minimum_variance
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0)
            && self.repair_initial.is_finite()
            && self.repair_initial > 0.0
            && self.repair_growth.is_finite()
            && self.repair_growth >= 1.0
            && self.maximum_total_repair.is_finite()
            && self.maximum_total_repair >= self.repair_initial
            && self.maximum_repair_attempts <= 4
    }
}

impl Eskf {
    pub(super) fn condition_covariance(&mut self) -> Result<(), EskfError> {
        if !self.covariance.iter().all(|value| value.is_finite())
            || !self
                .nav_consider_covariance
                .iter()
                .all(|value| value.is_finite())
            || !self
                .gap_derivative_covariance
                .iter()
                .chain(self.gap_nav_cross_covariance.iter())
                .all(|value| value.is_finite())
        {
            return Err(EskfError::NumericalFailure);
        }
        let (repairs, normalized_repair) =
            condition_navigation_covariance(&mut self.covariance, &self.covariance_policy)?;
        self.covariance_repairs = self.covariance_repairs.saturating_add(repairs);
        self.total_normalized_repair += normalized_repair;
        Ok(())
    }
}

/// Conditions only candidate storage and reports repair accounting to commit
/// after success. Update failures never modify live covariance or counters.
pub(super) fn condition_navigation_covariance(
    covariance: &mut NavMatrix,
    policy: &CovariancePolicy,
) -> Result<(u32, f32), EskfError> {
    if covariance.iter().any(|value| !value.is_finite()) {
        return Err(EskfError::NumericalFailure);
    }
    for row in 0..NAV_DIM {
        for column in 0..row {
            let symmetric = 0.5 * covariance[(row, column)] + 0.5 * covariance[(column, row)];
            covariance[(row, column)] = symmetric;
            covariance[(column, row)] = symmetric;
        }
        covariance[(row, row)] = covariance[(row, row)].max(policy.minimum_variance[row]);
    }
    if normalized_cholesky_succeeds(covariance, &policy.state_scales) {
        return Ok((0, 0.0));
    }

    let mut repair = policy.repair_initial;
    let mut accumulated = 0.0;
    for attempt in 0..policy.maximum_repair_attempts {
        if accumulated + repair > policy.maximum_total_repair {
            break;
        }
        for index in 0..NAV_DIM {
            let scale = policy.state_scales[index];
            covariance[(index, index)] += repair * scale * scale;
        }
        accumulated += repair;
        if normalized_cholesky_succeeds(covariance, &policy.state_scales) {
            return Ok((u32::from(attempt) + 1, accumulated));
        }
        repair *= policy.repair_growth;
    }
    Err(EskfError::CovarianceNotPositiveSemidefinite)
}

pub(super) fn matrix3_is_psd(matrix: &Matrix3<f32>) -> bool {
    // Measurement covariances are allowed to be singular; only the innovation
    // after state uncertainty is added must be strictly positive definite.
    // The normalized principal-minor check is scale-aware and therefore does
    // not introduce a unit-scale diagonal floor that could hide indefiniteness.
    covariance_density_is_valid(matrix)
}

pub(super) fn active_principal_block_is_psd<const D: usize>(
    matrix: &SMatrix<f32, D, D>,
    dimension: usize,
) -> bool {
    if dimension == 0 {
        return true;
    }
    if dimension > D {
        return false;
    }
    let mut scale = 0.0_f32;
    for row in 0..dimension {
        for column in 0..dimension {
            let value = matrix[(row, column)];
            if !value.is_finite() {
                return false;
            }
            scale = scale.max(value.abs());
        }
    }
    if scale == 0.0 {
        return true;
    }
    let dimension_scale = dimension as f32;
    let symmetry_tolerance = 128.0 * f32::EPSILON * dimension_scale;
    for row in 0..dimension {
        for column in 0..row {
            if ((matrix[(row, column)] - matrix[(column, row)]) / scale).abs() > symmetry_tolerance
            {
                return false;
            }
        }
    }

    // Semidefinite-safe normalized Cholesky. A vanishing pivot is legal only
    // when the residual entries in that row/column also vanish at the same
    // scale. Innovation solves use a separate strict-PD decomposition.
    let psd_tolerance = 512.0 * f32::EPSILON * dimension_scale;
    let mut lower = SMatrix::<f32, D, D>::zeros();
    for row in 0..dimension {
        for column in 0..=row {
            let mut value = 0.5 * (matrix[(row, column)] / scale + matrix[(column, row)] / scale);
            for k in 0..column {
                value -= lower[(row, k)] * lower[(column, k)];
            }
            if row == column {
                if value < -psd_tolerance || !value.is_finite() {
                    return false;
                }
                lower[(row, column)] = if value > psd_tolerance {
                    crate::scalar_math::sqrt(value)
                } else {
                    0.0
                };
            } else if lower[(column, column)] > crate::scalar_math::sqrt(psd_tolerance) {
                lower[(row, column)] = value / lower[(column, column)];
            } else if value.abs() > psd_tolerance {
                return false;
            }
        }
    }
    true
}

fn normalized_cholesky_succeeds(covariance: &NavMatrix, scales: &[f32; NAV_DIM]) -> bool {
    let mut lower = NavMatrix::zeros();
    for row in 0..NAV_DIM {
        for column in 0..=row {
            let mut value = 0.5 * (covariance[(row, column)] + covariance[(column, row)])
                / (scales[row] * scales[column]);
            for k in 0..column {
                value -= lower[(row, k)] * lower[(column, k)];
            }
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return false;
                }
                lower[(row, column)] = crate::scalar_math::sqrt(value);
            } else {
                lower[(row, column)] = value / lower[(column, column)];
            }
        }
    }
    true
}
