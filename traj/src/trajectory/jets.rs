//! Point speed values and derivatives for metric slope and uncertainty queries.

use super::dense::{DenseSegment, PointJet};
use super::math::{
    add, cross, dot, dot_abs_sum, norm_upper, roundoff_guard, scale, sub, upper_add, upper_mul,
    vector,
};
use super::roots::ScalarJet;
use crate::frame::ReferenceEllipsoid;
use crate::metric::{MetricError, SpeedQuantity};
#[cfg(not(any(feature = "offline", test)))]
use nalgebra::ComplexField;

pub(super) fn speed_value_jet(
    segment: &DenseSegment,
    lever_body_m: [f64; 3],
    ellipsoid: ReferenceEllipsoid,
    quantity: SpeedQuantity,
    parameter: f64,
) -> Result<ScalarJet, MetricError> {
    let point = segment.point_jet(parameter, lever_body_m)?;
    match quantity {
        SpeedQuantity::InstantaneousHorizontal => {
            scalar_sqrt_jet(horizontal_speed_squared_jet(segment, point, ellipsoid)?)
        }
        SpeedQuantity::Spatial3d => scalar_sqrt_jet(spatial_speed_squared_jet(segment, point)),
        SpeedQuantity::BodyLongitudinalSigned => body_longitudinal_jet(segment, point, parameter),
        SpeedQuantity::BodyLongitudinalMagnitude => {
            let mut body = body_longitudinal_jet(segment, point, parameter)?;
            if body.value.abs() <= body.value_roundoff {
                return Err(MetricError::AmbiguousRoot);
            }
            let sign = if body.value > 0.0 { 1.0 } else { -1.0 };
            body.value = body.value.abs();
            body.derivative *= sign;
            body.second_derivative *= sign;
            Ok(body)
        }
    }
}

pub(super) fn spatial_speed_squared_jet(segment: &DenseSegment, point: PointJet) -> ScalarJet {
    let inverse_duration = segment.duration_seconds.recip();
    let velocity = scale(point.first, inverse_duration);
    let first = scale(point.second, inverse_duration);
    let second = scale(point.third, inverse_duration);
    let value_scale = dot_abs_sum(velocity, velocity);
    let derivative_scale = 2.0 * dot_abs_sum(velocity, first);
    let second_scale = 2.0 * (dot_abs_sum(first, first) + dot_abs_sum(velocity, second));
    ScalarJet {
        value: dot(velocity, velocity),
        derivative: 2.0 * dot(velocity, first),
        second_derivative: 2.0 * (dot(first, first) + dot(velocity, second)),
        value_roundoff: roundoff_guard(value_scale),
        derivative_roundoff: roundoff_guard(derivative_scale),
        second_derivative_roundoff: roundoff_guard(second_scale),
    }
}

pub(super) fn body_longitudinal_jet(
    segment: &DenseSegment,
    point: PointJet,
    parameter: f64,
) -> Result<ScalarJet, MetricError> {
    let inverse_duration = segment.duration_seconds.recip();
    let velocity = scale(point.first, inverse_duration);
    let first = scale(point.second, inverse_duration);
    let second = scale(point.third, inverse_duration);
    let inverse = point.orientation.inverse();
    let body_velocity = inverse
        .rotate_vector(vector(velocity).map_err(|_| MetricError::NumericalFailure)?)
        .components();
    let body_first = inverse
        .rotate_vector(vector(first).map_err(|_| MetricError::NumericalFailure)?)
        .components();
    let body_second = inverse
        .rotate_vector(vector(second).map_err(|_| MetricError::NumericalFailure)?)
        .components();
    let (_, omega, omega_derivative, _) = segment
        .derived_orientation_bridge()
        .map_err(|_| MetricError::NumericalFailure)?
        .kinematics(
            segment.start.orientation_ecef_from_body.quaternion(),
            parameter,
        )
        .map_err(|_| MetricError::NumericalFailure)?;
    let first_rotating = sub(body_first, cross(omega, body_velocity));
    let second_rotating = add(
        sub(
            sub(body_second, scale(cross(omega, body_first), 2.0)),
            cross(omega_derivative, body_velocity),
        ),
        cross(omega, cross(omega, body_velocity)),
    );
    Ok(ScalarJet {
        value: body_velocity[0],
        derivative: first_rotating[0],
        second_derivative: second_rotating[0],
        value_roundoff: roundoff_guard(norm_upper(velocity)),
        derivative_roundoff: roundoff_guard(upper_add(
            norm_upper(first),
            upper_mul(norm_upper(omega), norm_upper(velocity)),
        )),
        second_derivative_roundoff: roundoff_guard(upper_add(
            norm_upper(second),
            upper_add(
                upper_add(
                    upper_mul(2.0, upper_mul(norm_upper(omega), norm_upper(first))),
                    upper_mul(norm_upper(omega_derivative), norm_upper(velocity)),
                ),
                upper_mul(
                    upper_mul(norm_upper(omega), norm_upper(omega)),
                    norm_upper(velocity),
                ),
            ),
        )),
    })
}

pub(super) fn horizontal_speed_squared_jet(
    segment: &DenseSegment,
    point: PointJet,
    ellipsoid: ReferenceEllipsoid,
) -> Result<ScalarJet, MetricError> {
    let inverse_duration = segment.duration_seconds.recip();
    let normal = ellipsoid_normal_jet(point, ellipsoid)?;
    let velocity = [
        Jet2::new(
            point.first[0] * inverse_duration,
            point.second[0] * inverse_duration,
            point.third[0] * inverse_duration,
        ),
        Jet2::new(
            point.first[1] * inverse_duration,
            point.second[1] * inverse_duration,
            point.third[1] * inverse_duration,
        ),
        Jet2::new(
            point.first[2] * inverse_duration,
            point.second[2] * inverse_duration,
            point.third[2] * inverse_duration,
        ),
    ];
    let mut speed_squared = Jet2::constant(0.0);
    let mut normal_speed = Jet2::constant(0.0);
    for axis in 0..3 {
        speed_squared = speed_squared.add(velocity[axis].mul(velocity[axis]));
        normal_speed = normal_speed.add(normal[axis].mul(velocity[axis]));
    }
    let horizontal = speed_squared.sub(normal_speed.mul(normal_speed));
    let speed_scale = velocity.iter().map(|entry| entry.value.abs()).sum::<f64>();
    let first_scale = velocity.iter().map(|entry| entry.first.abs()).sum::<f64>();
    let second_scale = velocity.iter().map(|entry| entry.second.abs()).sum::<f64>();
    Ok(ScalarJet {
        value: horizontal.value,
        derivative: horizontal.first,
        second_derivative: horizontal.second,
        value_roundoff: roundoff_guard(upper_mul(speed_scale, speed_scale)),
        derivative_roundoff: roundoff_guard(upper_mul(4.0, upper_mul(speed_scale, first_scale))),
        second_derivative_roundoff: roundoff_guard(upper_mul(
            8.0,
            upper_add(
                upper_mul(first_scale, first_scale),
                upper_mul(speed_scale, second_scale),
            ),
        )),
    })
}

pub(super) fn scalar_sqrt_jet(source: ScalarJet) -> Result<ScalarJet, MetricError> {
    if !source.value.is_finite() || source.value <= source.value_roundoff {
        return Err(MetricError::AmbiguousRoot);
    }
    let value = source.value.sqrt();
    let derivative = source.derivative / (2.0 * value);
    let second_derivative = source.second_derivative / (2.0 * value)
        - source.derivative * source.derivative / (4.0 * value * value * value);
    Ok(ScalarJet {
        value,
        derivative,
        second_derivative,
        value_roundoff: roundoff_guard(value),
        derivative_roundoff: roundoff_guard(derivative),
        second_derivative_roundoff: roundoff_guard(second_derivative),
    })
}

#[derive(Clone, Copy)]
pub(super) struct Jet2 {
    pub(super) value: f64,
    pub(super) first: f64,
    pub(super) second: f64,
}

impl Jet2 {
    pub(super) const fn new(value: f64, first: f64, second: f64) -> Self {
        Self {
            value,
            first,
            second,
        }
    }

    pub(super) const fn constant(value: f64) -> Self {
        Self::new(value, 0.0, 0.0)
    }

    pub(super) fn add(self, other: Self) -> Self {
        Self::new(
            self.value + other.value,
            self.first + other.first,
            self.second + other.second,
        )
    }

    pub(super) fn sub(self, other: Self) -> Self {
        Self::new(
            self.value - other.value,
            self.first - other.first,
            self.second - other.second,
        )
    }

    pub(super) fn mul(self, other: Self) -> Self {
        Self::new(
            self.value * other.value,
            self.first * other.value + self.value * other.first,
            self.second * other.value + 2.0 * self.first * other.first + self.value * other.second,
        )
    }

    pub(super) fn scale(self, scalar: f64) -> Self {
        Self::new(
            self.value * scalar,
            self.first * scalar,
            self.second * scalar,
        )
    }

    pub(super) fn sqrt(self) -> Result<Self, MetricError> {
        if !self.value.is_finite() || self.value <= 0.0 {
            return Err(MetricError::AmbiguousRoot);
        }
        let root = self.value.sqrt();
        Ok(Self::new(
            root,
            self.first / (2.0 * root),
            self.second / (2.0 * root) - self.first * self.first / (4.0 * root * root * root),
        ))
    }

    pub(super) fn sin_cos(self) -> (Self, Self) {
        let (sin, cos) = crate::scalar_math::sin_cos(self.value);
        (
            Self::new(
                sin,
                cos * self.first,
                -sin * self.first * self.first + cos * self.second,
            ),
            Self::new(
                cos,
                -sin * self.first,
                -cos * self.first * self.first - sin * self.second,
            ),
        )
    }

    pub(super) fn atan2(y: Self, x: Self) -> Result<Self, MetricError> {
        let denominator = x.value * x.value + y.value * y.value;
        if !denominator.is_finite() || denominator <= f64::MIN_POSITIVE {
            return Err(MetricError::AmbiguousRoot);
        }
        let numerator = x.value * y.first - y.value * x.first;
        let numerator_derivative = x.value * y.second - y.value * x.second;
        let denominator_derivative = 2.0 * (x.value * x.first + y.value * y.first);
        Ok(Self::new(
            crate::scalar_math::atan2(y.value, x.value),
            numerator / denominator,
            (numerator_derivative * denominator - numerator * denominator_derivative)
                / (denominator * denominator),
        ))
    }
}

pub(super) fn ellipsoid_normal_jet(
    point: PointJet,
    ellipsoid: ReferenceEllipsoid,
) -> Result<[Jet2; 3], MetricError> {
    let x = Jet2::new(point.position[0], point.first[0], point.second[0]);
    let y = Jet2::new(point.position[1], point.first[1], point.second[1]);
    let z = Jet2::new(point.position[2], point.first[2], point.second[2]);
    let horizontal = x.mul(x).add(y.mul(y)).sqrt()?;
    let longitude = Jet2::atan2(y, x)?;
    let a = ellipsoid.semi_major_axis_m();
    let flattening = ellipsoid.inverse_flattening().recip();
    let b = a * (1.0 - flattening);
    let e2 = flattening * (2.0 - flattening);
    let ep2 = (a * a - b * b) / (b * b);
    let theta = Jet2::atan2(z.scale(a), horizontal.scale(b))?;
    let (sin_theta, cos_theta) = theta.sin_cos();
    let latitude = Jet2::atan2(
        z.add(sin_theta.mul(sin_theta).mul(sin_theta).scale(ep2 * b)),
        horizontal.sub(cos_theta.mul(cos_theta).mul(cos_theta).scale(e2 * a)),
    )?;
    let (sin_latitude, cos_latitude) = latitude.sin_cos();
    let (sin_longitude, cos_longitude) = longitude.sin_cos();
    let up = [
        cos_latitude.mul(cos_longitude),
        cos_latitude.mul(sin_longitude),
        sin_latitude,
    ];
    if up.iter().any(|entry| {
        !entry.value.is_finite() || !entry.first.is_finite() || !entry.second.is_finite()
    }) {
        return Err(MetricError::NumericalFailure);
    }
    Ok(up)
}
