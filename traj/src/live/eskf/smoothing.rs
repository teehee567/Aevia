//! Forward error transforms retained for the bounded RTS backward pass.

use super::{EskfError, MAX_CONSIDER, NavMeasurementCross, update::LinearMeasurement};
use crate::live::{
    preintegration::BIAS_DIM,
    smoothing::{AUG_DIM, CONSIDER_START, SAMPLE_START},
    state::{ATT, NAV_DIM, NavMatrix},
};
use nalgebra::{ArrayStorage, Matrix3, SMatrix};

type NavAugmentedTransform = SMatrix<f32, NAV_DIM, AUG_DIM>;

/// Cumulative transform from a node's predicted augmented error to its
/// filtered navigation error. Nuisance means retain implicit identity rows.
/// Both matrices belong in caller-owned PSRAM, including transaction staging.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RtsUpdateCapture {
    pub(crate) nav_transform: NavAugmentedTransform,
    pub(crate) attitude_reset: Matrix3<f32>,
    candidate: NavAugmentedTransform,
}

impl RtsUpdateCapture {
    pub(crate) const fn new() -> Self {
        let mut identity = [[0.0; NAV_DIM]; AUG_DIM];
        let mut index = 0;
        while index < NAV_DIM {
            identity[index][index] = 1.0;
            index += 1;
        }
        Self {
            nav_transform: NavAugmentedTransform::from_array_storage(ArrayStorage(identity)),
            attitude_reset: Matrix3::from_array_storage(ArrayStorage([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ])),
            candidate: NavAugmentedTransform::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; AUG_DIM],
            )),
        }
    }

    pub(crate) fn reset(&mut self) {
        self.nav_transform.fill(0.0);
        for index in 0..NAV_DIM {
            self.nav_transform[(index, index)] = 1.0;
        }
        self.candidate.fill(0.0);
        self.attitude_reset = Matrix3::identity();
    }

    pub(crate) fn copy_from(&mut self, source: &Self) {
        self.nav_transform.copy_from(&source.nav_transform);
        self.candidate.copy_from(&source.candidate);
        self.attitude_reset.copy_from(&source.attitude_reset);
    }

    /// Stage Anew = Jnav Aold Jaug' for an orthogonal local-frame change.
    /// `target` is caller-owned transaction scratch and may change on error.
    pub(crate) fn reanchor_into(
        &self,
        target: &mut Self,
        jacobian: &NavMatrix,
    ) -> Result<(), EskfError> {
        for row in 0..NAV_DIM {
            for column in 0..AUG_DIM {
                target.candidate[(row, column)] = (0..NAV_DIM)
                    .map(|source| jacobian[(row, source)] * self.nav_transform[(source, column)])
                    .sum();
            }
        }
        for row in 0..NAV_DIM {
            for column in 0..AUG_DIM {
                target.nav_transform[(row, column)] = if column < NAV_DIM {
                    (0..NAV_DIM)
                        .map(|source| target.candidate[(row, source)] * jacobian[(column, source)])
                        .sum()
                } else {
                    target.candidate[(row, column)]
                };
            }
        }
        if target
            .candidate
            .iter()
            .chain(target.nav_transform.iter())
            .any(|value| !value.is_finite())
        {
            return Err(EskfError::NumericalFailure);
        }
        // Right-multiplicative attitude errors stay in body coordinates.
        target.attitude_reset.copy_from(&self.attitude_reset);
        Ok(())
    }

    pub(super) fn apply_update(
        &mut self,
        measurement: &LinearMeasurement,
        gain: &NavMeasurementCross,
        attitude_reset: &Matrix3<f32>,
    ) -> Result<(), EskfError> {
        // A+ = reset * [(I - K Hx) A - K Hnuisance]. Compute one
        // augmented column at a time without a matrix-valued stack temporary.
        for column in 0..AUG_DIM {
            let mut projected = [0.0; super::MAX_MEASUREMENT_DIM];
            for (measurement_row, value) in
                projected.iter_mut().enumerate().take(measurement.dimension)
            {
                for nav in 0..NAV_DIM {
                    *value += measurement.h_nav[(measurement_row, nav)]
                        * self.nav_transform[(nav, column)];
                }
                if (CONSIDER_START..CONSIDER_START + MAX_CONSIDER).contains(&column) {
                    *value += measurement.h_consider[(measurement_row, column - CONSIDER_START)];
                } else if (SAMPLE_START..SAMPLE_START + BIAS_DIM).contains(&column) {
                    *value += measurement.h_sample[(measurement_row, column - SAMPLE_START)];
                }
            }
            for row in 0..NAV_DIM {
                let mut value = self.nav_transform[(row, column)];
                for measurement_row in 0..measurement.dimension {
                    value -= gain[(row, measurement_row)] * projected[measurement_row];
                }
                self.candidate[(row, column)] = value;
            }
            let old_attitude = [
                self.candidate[(ATT, column)],
                self.candidate[(ATT + 1, column)],
                self.candidate[(ATT + 2, column)],
            ];
            for row in 0..3 {
                self.candidate[(ATT + row, column)] = (0..3)
                    .map(|source| attitude_reset[(row, source)] * old_attitude[source])
                    .sum();
            }
        }
        let cumulative_reset = attitude_reset * self.attitude_reset;
        if self
            .candidate
            .iter()
            .chain(cumulative_reset.iter())
            .any(|value| !value.is_finite())
        {
            return Err(EskfError::NumericalFailure);
        }
        self.nav_transform.copy_from(&self.candidate);
        self.attitude_reset = cumulative_reset;
        Ok(())
    }
}
