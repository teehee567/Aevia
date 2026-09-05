//! Scalar metric quantities and elementary vector/time operations.

use super::{definition::SpeedQuantity, report::MetricError};
use crate::{time::SessionTime, trajectory::ScalarKinematics};
use nalgebra::ComplexField;

pub(super) fn all_finite(values: &[f64]) -> bool {
    values.iter().all(|value| value.is_finite())
}

pub(super) fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0].mul_add(right[0], left[1].mul_add(right[1], left[2] * right[2]))
}

pub(super) fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1].mul_add(right[2], -left[2] * right[1]),
        left[2].mul_add(right[0], -left[0] * right[2]),
        left[0].mul_add(right[1], -left[1] * right[0]),
    ]
}

pub(super) fn sub(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

pub(super) fn add(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

pub(super) fn scale(vector: [f64; 3], scalar: f64) -> [f64; 3] {
    [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

pub(super) fn norm(vector: [f64; 3]) -> f64 {
    dot(vector, vector).sqrt()
}

pub(super) fn normalize(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = norm(vector);
    (length.is_finite() && length > f64::EPSILON).then(|| scale(vector, length.recip()))
}

pub(super) fn scalar_speed(state: &ScalarKinematics, quantity: SpeedQuantity) -> Option<f64> {
    match quantity {
        SpeedQuantity::InstantaneousHorizontal => Some(state.horizontal_speed_mps),
        SpeedQuantity::Spatial3d => Some(norm(state.velocity_ecef_mps)),
        SpeedQuantity::BodyLongitudinalSigned => state.body_longitudinal_speed_mps,
        SpeedQuantity::BodyLongitudinalMagnitude => state.body_longitudinal_speed_mps.map(f64::abs),
    }
}

pub(super) fn seconds_between(start: SessionTime, end: SessionTime) -> Result<f64, MetricError> {
    end.checked_duration_since(start)
        .map(|duration| duration.as_seconds_f64())
        .ok_or(MetricError::NumericalFailure)
}
