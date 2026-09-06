//! Vector arithmetic, tangent frames, and outward scalar bounds.

use crate::error::QueryError;
use crate::frame::ReferenceEllipsoid;
use crate::math::Vector3;
use crate::metric::MetricError;
#[cfg(not(any(feature = "offline", test)))]
use nalgebra::ComplexField;

pub(super) fn midpoint(lower: f64, upper: f64) -> f64 {
    lower + (upper - lower) * 0.5
}

pub(super) fn upper_add(left: f64, right: f64) -> f64 {
    debug_assert!(left >= 0.0 && right >= 0.0);
    let value = left + right;
    if value.is_finite() {
        value.next_up()
    } else {
        f64::INFINITY
    }
}

pub(super) fn upper_mul(left: f64, right: f64) -> f64 {
    debug_assert!(left >= 0.0 && right >= 0.0);
    let value = left * right;
    if value.is_finite() {
        value.next_up()
    } else {
        f64::INFINITY
    }
}

pub(super) fn norm_upper(value: [f64; 3]) -> f64 {
    let square = upper_add(
        upper_mul(value[0].abs(), value[0].abs()),
        upper_add(
            upper_mul(value[1].abs(), value[1].abs()),
            upper_mul(value[2].abs(), value[2].abs()),
        ),
    );
    square.sqrt().next_up()
}

pub(super) fn dot_abs_sum(left: [f64; 3], right: [f64; 3]) -> f64 {
    upper_add(
        upper_mul(left[0].abs(), right[0].abs()),
        upper_add(
            upper_mul(left[1].abs(), right[1].abs()),
            upper_mul(left[2].abs(), right[2].abs()),
        ),
    )
}

pub(super) fn roundoff_guard(scale: f64) -> f64 {
    if !scale.is_finite() {
        return f64::INFINITY;
    }
    // Numerical guard for singular point-speed conversions. Root exclusion
    // and existence use the separate complete interval expression graph.
    upper_mul(512.0 * f64::EPSILON, upper_add(1.0, scale.abs()))
}

#[derive(Clone, Copy)]
pub(super) struct TangentBasis {
    pub(super) east: Option<[f64; 3]>,
    pub(super) north: Option<[f64; 3]>,
    pub(super) up: [f64; 3],
}

pub(super) fn tangent_basis(
    position_ecef: [f64; 3],
    ellipsoid: ReferenceEllipsoid,
) -> Result<TangentBasis, MetricError> {
    if !position_ecef.iter().all(|value| value.is_finite()) {
        return Err(MetricError::NumericalFailure);
    }
    let [x, y, z] = position_ecef;
    let horizontal = x.hypot(y);
    if horizontal == 0.0 && z == 0.0 {
        return Err(MetricError::FrameMismatch);
    }
    let a = ellipsoid.semi_major_axis_m();
    let flattening = ellipsoid.inverse_flattening().recip();
    let b = a * (1.0 - flattening);
    let e2 = flattening * (2.0 - flattening);
    let ep2 = (a * a - b * b) / (b * b);
    let longitude = crate::scalar_math::atan2(y, x);
    let theta = crate::scalar_math::atan2(z * a, horizontal * b);
    let (sin_theta, cos_theta) = crate::scalar_math::sin_cos(theta);
    let latitude = crate::scalar_math::atan2(
        z + ep2 * b * sin_theta.powi(3),
        horizontal - e2 * a * cos_theta.powi(3),
    );
    let (sin_latitude, cos_latitude) = crate::scalar_math::sin_cos(latitude);
    let (sin_longitude, cos_longitude) = crate::scalar_math::sin_cos(longitude);
    let up = [
        cos_latitude * cos_longitude,
        cos_latitude * sin_longitude,
        sin_latitude,
    ];
    if horizontal <= a * 1.0e-12 {
        return Ok(TangentBasis {
            east: None,
            north: None,
            up,
        });
    }
    let east = [-sin_longitude, cos_longitude, 0.0];
    let north = cross(up, east);
    Ok(TangentBasis {
        east: Some(east),
        north: Some(north),
        up,
    })
}

pub(super) fn vector(components: [f64; 3]) -> Result<Vector3, QueryError> {
    Vector3::from_components(components).map_err(|_| QueryError::TrajectoryInvalid)
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

pub(super) fn add(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

pub(super) fn sub(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

pub(super) fn scale(vector: [f64; 3], scalar: f64) -> [f64; 3] {
    [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

pub(super) fn norm(vector: [f64; 3]) -> f64 {
    dot(vector, vector).sqrt()
}

pub(super) fn query_to_metric(error: QueryError) -> MetricError {
    match error {
        QueryError::ReferencePointUnavailable => MetricError::ReferencePointUnavailable,
        QueryError::FrameUnavailable => MetricError::FrameMismatch,
        QueryError::ObservabilityUnavailable => MetricError::Unobservable,
        QueryError::OutsideAvailableSpan { .. } => MetricError::OutsideTrajectory,
        QueryError::InvalidRequest => MetricError::InvalidDefinition,
        QueryError::BackingStoreFailure | QueryError::TrajectoryInvalid => {
            MetricError::NumericalFailure
        }
    }
}

pub(super) fn metric_to_query(error: MetricError) -> QueryError {
    match error {
        MetricError::ReferencePointUnavailable => QueryError::ReferencePointUnavailable,
        MetricError::FrameMismatch => QueryError::FrameUnavailable,
        MetricError::Unobservable => QueryError::ObservabilityUnavailable,
        MetricError::OutsideTrajectory | MetricError::EmptyTrajectory => QueryError::InvalidRequest,
        MetricError::InvalidDefinition => QueryError::InvalidRequest,
        MetricError::AmbiguousRoot
        | MetricError::NumericalFailure
        | MetricError::EvaluationBudgetExceeded
        | MetricError::CapacityExceeded
        | MetricError::Unsupported => QueryError::TrajectoryInvalid,
    }
}
