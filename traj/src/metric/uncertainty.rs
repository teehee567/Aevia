//! Implicit-event sensitivities and covariance projection.

#[cfg(any(test, feature = "offline"))]
use super::definition::DistancePlan;
use super::{
    definition::{FiniteGate, SpeedQuantity},
    geometry::{add, dot, norm, scale, sub},
    report::GateCrossingReport,
};
#[cfg(any(test, feature = "offline"))]
use crate::time::TimeSpan;
use crate::{
    frame::ReferenceEllipsoid,
    ids::{GateId, ReferencePointId},
    quality::{CovarianceConditioning, FieldValue, UnavailableReason},
    time::SessionTime,
    trajectory::{ScalarKinematics, Trajectory},
    uncertainty::KinematicCovariance,
};
use nalgebra::ComplexField;

/// Below this absolute time derivative the implicit-function linearization is
/// numerically singular for the units used by the supported event functions.
pub(super) const MIN_EVENT_DERIVATIVE: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct StateSensitivity {
    pub(crate) position: [f64; 3],
    pub(crate) velocity: [f64; 3],
    pub(crate) attitude: [f64; 3],
}

impl StateSensitivity {
    pub(super) const ZERO: Self = Self {
        position: [0.0; 3],
        velocity: [0.0; 3],
        attitude: [0.0; 3],
    };

    fn scaled(self, scale: f64) -> Self {
        Self {
            position: super::geometry::scale(self.position, scale),
            velocity: super::geometry::scale(self.velocity, scale),
            attitude: super::geometry::scale(self.attitude, scale),
        }
    }

    fn add(self, other: Self) -> Self {
        Self {
            position: add(self.position, other.position),
            velocity: add(self.velocity, other.velocity),
            attitude: add(self.attitude, other.attitude),
        }
    }
}

/// First-order coordinates of one implicit event. The gate-survey component
/// is a displacement along the gate normal. A provider that retains a full
/// host joint state may use the location and gradients to recover event-pair
/// covariance in one sequential pass; the default trajectory provider has
/// marginals only and therefore fails that operation closed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EventTimeSensitivity {
    pub(crate) segment_index: usize,
    pub(crate) parameter: f64,
    pub(crate) time: SessionTime,
    pub(crate) reference_point: ReferencePointId,
    pub(crate) state: StateSensitivity,
    pub(crate) gate: Option<GateId>,
    pub(crate) gate_survey_coefficient_s_per_m: f64,
    pub(crate) gate_survey_variance_m2: Option<f64>,
}

pub(crate) trait MetricUncertaintyProvider {
    fn kinematic_covariance_at(
        &mut self,
        trajectory: &Trajectory,
        segment_index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<KinematicCovariance, UnavailableReason>;

    fn event_time_cross_covariance_s2(
        &mut self,
        _trajectory: &Trajectory,
        _first: &EventTimeSensitivity,
        _second: &EventTimeSensitivity,
    ) -> FieldValue<f64> {
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    }

    /// Projects one complete implicit-event sensitivity through the retained
    /// joint model. Host smoothers override this operation so an interior
    /// event can use both dense-segment endpoints and their shared-parameter
    /// cross terms. The current public gate model does not declare survey
    /// independence or expose a stable shared-parameter identity, so the
    /// marginal-only adapter accepts only an exact (zero-variance) survey.
    fn event_time_variance_s2(
        &mut self,
        trajectory: &Trajectory,
        event: &EventTimeSensitivity,
    ) -> FieldValue<f64> {
        let covariance = match self.kinematic_covariance_at(
            trajectory,
            event.segment_index,
            event.parameter,
            event.reference_point,
        ) {
            Ok(value) => value,
            Err(reason) => return FieldValue::Unavailable(reason),
        };
        let state_variance = match projected_state_variance(covariance, event.state) {
            Ok(value) => value,
            Err(reason) => return FieldValue::Unavailable(reason),
        };
        let survey_variance = match (event.gate, event.gate_survey_variance_m2) {
            (Some(_), Some(0.0)) => 0.0,
            (Some(_), Some(_)) => {
                return FieldValue::Unavailable(UnavailableReason::MissingCorrelation);
            }
            (Some(_), None) => {
                return FieldValue::Unavailable(UnavailableReason::MissingUncertainty);
            }
            (None, _) => 0.0,
        };
        FieldValue::Available(state_variance + survey_variance)
    }

    #[cfg(any(test, feature = "offline"))]
    fn integrated_distance_one_sigma_m(
        &mut self,
        _trajectory: &Trajectory,
        _plan: DistancePlan,
        _span: TimeSpan,
    ) -> FieldValue<f64> {
        // An integral needs an augmented accumulator and its state/parameter
        // cross covariance. Summing pointwise marginals is never a fallback.
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    }
}

/// Shared survey contribution for two events on the same physical gate. A
/// full provider adds this to its retained trajectory/shared-parameter cross
/// terms; returning this term alone as the complete event covariance would be
/// incorrect.
#[cfg(test)]
pub(crate) fn shared_gate_survey_time_covariance_s2(
    first: &EventTimeSensitivity,
    second: &EventTimeSensitivity,
) -> Option<f64> {
    let gate = first.gate?;
    if second.gate != Some(gate) {
        return None;
    }
    let first_variance = first.gate_survey_variance_m2?;
    let second_variance = second.gate_survey_variance_m2?;
    if first_variance != second_variance {
        return None;
    }
    Some(
        first.gate_survey_coefficient_s_per_m
            * second.gate_survey_coefficient_s_per_m
            * first_variance,
    )
}

#[derive(Default)]
pub(super) struct TrajectoryMarginalUncertainty;

impl MetricUncertaintyProvider for TrajectoryMarginalUncertainty {
    fn kinematic_covariance_at(
        &mut self,
        trajectory: &Trajectory,
        segment_index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<KinematicCovariance, UnavailableReason> {
        let estimate = trajectory
            .metric_estimate_at_parameter(segment_index, parameter, reference_point)
            .map_err(|_| UnavailableReason::MissingUncertainty)?;
        if estimate.quality.covariance == CovarianceConditioning::Unavailable {
            // Interior Hermite states and nonzero lever arms currently lack a
            // justified joint covariance transform. The numeric envelope in
            // the query result is diagnostic only.
            return Err(UnavailableReason::MissingCorrelation);
        }
        Ok(estimate.covariance)
    }
}

pub(super) fn gate_event_sensitivity(
    segment_index: usize,
    parameter: f64,
    state: &ScalarKinematics,
    gate: &FiniteGate,
    reference_point: ReferencePointId,
) -> Result<EventTimeSensitivity, UnavailableReason> {
    let derivative = dot(state.velocity_ecef_mps, gate.normal_ecef);
    if !derivative.is_finite() || derivative.abs() <= MIN_EVENT_DERIVATIVE {
        return Err(UnavailableReason::IllConditioned);
    }
    Ok(EventTimeSensitivity {
        segment_index,
        parameter,
        time: state.time,
        reference_point,
        state: StateSensitivity {
            position: scale(gate.normal_ecef, -derivative.recip()),
            ..StateSensitivity::ZERO
        },
        gate: Some(gate.id),
        gate_survey_coefficient_s_per_m: derivative.recip(),
        gate_survey_variance_m2: gate.survey_variance_normal_m2,
    })
}

pub(super) fn speed_event_sensitivity(
    segment_index: usize,
    parameter: f64,
    state: &ScalarKinematics,
    quantity: SpeedQuantity,
    slope_mps2: f64,
    reference_point: ReferencePointId,
    ellipsoid: ReferenceEllipsoid,
    orientation_ecef_from_body: crate::frame::OrientationEcefFromBody,
) -> Result<EventTimeSensitivity, UnavailableReason> {
    if !slope_mps2.is_finite() || slope_mps2.abs() <= MIN_EVENT_DERIVATIVE {
        return Err(UnavailableReason::IllConditioned);
    }
    let gradient = speed_state_sensitivity(state, quantity, ellipsoid, orientation_ecef_from_body)?;
    Ok(EventTimeSensitivity {
        segment_index,
        parameter,
        time: state.time,
        reference_point,
        state: gradient.scaled(-slope_mps2.recip()),
        gate: None,
        gate_survey_coefficient_s_per_m: 0.0,
        gate_survey_variance_m2: None,
    })
}

pub(super) fn event_time_one_sigma(
    trajectory: &Trajectory,
    provider: &mut dyn MetricUncertaintyProvider,
    sensitivity: Result<EventTimeSensitivity, UnavailableReason>,
) -> FieldValue<f64> {
    let sensitivity = match sensitivity {
        Ok(value) => value,
        Err(reason) => return FieldValue::Unavailable(reason),
    };
    match provider.event_time_variance_s2(trajectory, &sensitivity) {
        FieldValue::Available(variance) => sigma_from_variance(variance),
        FieldValue::Unavailable(reason) => FieldValue::Unavailable(reason),
    }
}

pub(super) fn gate_crossing_speed_one_sigma(
    trajectory: &Trajectory,
    provider: &mut dyn MetricUncertaintyProvider,
    event: Result<EventTimeSensitivity, UnavailableReason>,
    state: &ScalarKinematics,
    quantity: SpeedQuantity,
    speed_slope_mps2: f64,
) -> FieldValue<f64> {
    let event = match event {
        Ok(value) => value,
        Err(reason) => return FieldValue::Unavailable(reason),
    };
    let estimate = match trajectory.metric_estimate_at_parameter(
        event.segment_index,
        event.parameter,
        event.reference_point,
    ) {
        Ok(value) => value,
        Err(_) => return FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
    };
    let fixed_time = match speed_state_sensitivity(
        state,
        quantity,
        trajectory.frame().ellipsoid(),
        estimate.orientation_ecef_from_body,
    ) {
        Ok(value) => value,
        Err(reason) => return FieldValue::Unavailable(reason),
    };
    let total = fixed_time.add(event.state.scaled(speed_slope_mps2));
    let covariance = match provider.kinematic_covariance_at(
        trajectory,
        event.segment_index,
        event.parameter,
        event.reference_point,
    ) {
        Ok(value) => value,
        Err(reason) => return FieldValue::Unavailable(reason),
    };
    let state_variance = match projected_state_variance(covariance, total) {
        Ok(value) => value,
        Err(reason) => return FieldValue::Unavailable(reason),
    };
    let survey_variance = match event.gate_survey_variance_m2 {
        Some(0.0) => 0.0,
        Some(_) => return FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
        None => return FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
    };
    sigma_from_variance(state_variance + survey_variance)
}

pub(super) fn speed_target_uncertainty(
    trajectory: &Trajectory,
    provider: &mut dyn MetricUncertaintyProvider,
    reference_point: ReferencePointId,
    terminal_speed: Option<(SpeedQuantity, f64)>,
    slope: Option<f64>,
    location: Option<(usize, f64)>,
) -> (FieldValue<f64>, Option<(SpeedQuantity, FieldValue<f64>)>) {
    let (Some((quantity, _)), Some(slope), Some((segment_index, parameter))) =
        (terminal_speed, slope, location)
    else {
        // A distance-root event needs the augmented path accumulator and its
        // covariance with the terminal state; a timestamp alone is not enough.
        return (
            FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
            terminal_speed.map(|(quantity, _)| {
                (
                    quantity,
                    FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
                )
            }),
        );
    };
    let state = match trajectory.scalar_kinematics_at_parameter(
        segment_index,
        parameter,
        reference_point,
    ) {
        Ok(value) => value,
        Err(_) => {
            return (
                FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
                Some((
                    quantity,
                    FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
                )),
            );
        }
    };
    let estimate =
        match trajectory.metric_estimate_at_parameter(segment_index, parameter, reference_point) {
            Ok(value) => value,
            Err(_) => {
                return (
                    FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
                    Some((
                        quantity,
                        FieldValue::Unavailable(UnavailableReason::MissingUncertainty),
                    )),
                );
            }
        };
    let event = speed_event_sensitivity(
        segment_index,
        parameter,
        &state,
        quantity,
        slope,
        reference_point,
        trajectory.frame().ellipsoid(),
        estimate.orientation_ecef_from_body,
    );
    let time_sigma = event_time_one_sigma(trajectory, provider, event);

    let speed_sigma = match speed_state_sensitivity(
        &state,
        quantity,
        trajectory.frame().ellipsoid(),
        estimate.orientation_ecef_from_body,
    ) {
        Ok(sensitivity) => match provider.kinematic_covariance_at(
            trajectory,
            segment_index,
            parameter,
            reference_point,
        ) {
            Ok(covariance) => projected_state_variance(covariance, sensitivity)
                .map_or_else(FieldValue::Unavailable, sigma_from_variance),
            Err(reason) => FieldValue::Unavailable(reason),
        },
        Err(reason) => FieldValue::Unavailable(reason),
    };
    (time_sigma, Some((quantity, speed_sigma)))
}

pub(super) fn lap_elapsed_one_sigma(
    trajectory: &Trajectory,
    provider: &mut dyn MetricUncertaintyProvider,
    start: &GateCrossingReport,
    start_sensitivity: Option<&EventTimeSensitivity>,
    end: &GateCrossingReport,
    end_sensitivity: Option<&EventTimeSensitivity>,
) -> FieldValue<f64> {
    let start_sigma = match start.time_one_sigma_s {
        FieldValue::Available(value) => value,
        FieldValue::Unavailable(reason) => return FieldValue::Unavailable(reason),
    };
    let end_sigma = match end.time_one_sigma_s {
        FieldValue::Available(value) => value,
        FieldValue::Unavailable(reason) => return FieldValue::Unavailable(reason),
    };
    let (Some(start_sensitivity), Some(end_sensitivity)) = (start_sensitivity, end_sensitivity)
    else {
        return FieldValue::Unavailable(UnavailableReason::MissingCorrelation);
    };
    let cross = match provider.event_time_cross_covariance_s2(
        trajectory,
        start_sensitivity,
        end_sensitivity,
    ) {
        FieldValue::Available(value) => value,
        FieldValue::Unavailable(reason) => return FieldValue::Unavailable(reason),
    };
    // This is the only allowed duration propagation. In particular there is
    // no fallback to sqrt(var_start + var_end).
    sigma_from_variance(start_sigma.powi(2) + end_sigma.powi(2) - 2.0 * cross)
}

fn projected_state_variance(
    covariance: KinematicCovariance,
    sensitivity: StateSensitivity,
) -> Result<f64, UnavailableReason> {
    let position = quadratic(sensitivity.position, covariance.position());
    let velocity = quadratic(sensitivity.velocity, covariance.velocity());
    let attitude = quadratic(sensitivity.attitude, covariance.attitude_error());
    let mut variance = position + velocity + attitude;
    let mut scale = position.abs() + velocity.abs() + attitude.abs();
    if vector_has_effect(sensitivity.position) && vector_has_effect(sensitivity.velocity) {
        let Some(cross_covariance) = covariance.position_velocity() else {
            return Err(UnavailableReason::MissingCorrelation);
        };
        let cross = 2.0
            * bilinear(
                sensitivity.position,
                cross_covariance.to_matrix(),
                sensitivity.velocity,
            );
        variance += cross;
        scale += cross.abs();
    }
    // KinematicCovariance intentionally exposes no position/attitude or
    // velocity/attitude cross blocks. A nonzero attitude projection therefore
    // cannot be combined with another nonzero marginal as if independent.
    if attitude > 0.0 && (position > 0.0 || velocity > 0.0) {
        return Err(UnavailableReason::MissingCorrelation);
    }
    if !variance.is_finite() {
        return Err(UnavailableReason::MissingUncertainty);
    }
    let tolerance = 256.0 * f64::EPSILON * scale;
    if variance < -tolerance {
        return Err(UnavailableReason::MissingUncertainty);
    }
    Ok(variance.max(0.0))
}

fn sigma_from_variance(variance: f64) -> FieldValue<f64> {
    if variance.is_finite() && variance >= 0.0 {
        FieldValue::Available(variance.sqrt())
    } else {
        FieldValue::Unavailable(UnavailableReason::MissingUncertainty)
    }
}

fn quadratic(vector: [f64; 3], covariance: crate::uncertainty::Covariance3) -> f64 {
    bilinear(vector, covariance.to_matrix(), vector)
}

fn bilinear(left: [f64; 3], matrix: [[f64; 3]; 3], right: [f64; 3]) -> f64 {
    let projected = [
        dot(matrix[0], right),
        dot(matrix[1], right),
        dot(matrix[2], right),
    ];
    dot(left, projected)
}

fn vector_has_effect(vector: [f64; 3]) -> bool {
    vector.iter().any(|value| *value != 0.0)
}

pub(super) fn speed_state_sensitivity(
    state: &ScalarKinematics,
    quantity: SpeedQuantity,
    ellipsoid: ReferenceEllipsoid,
    orientation_ecef_from_body: crate::frame::OrientationEcefFromBody,
) -> Result<StateSensitivity, UnavailableReason> {
    match quantity {
        SpeedQuantity::Spatial3d => {
            let speed = norm(state.velocity_ecef_mps);
            if !speed.is_finite() || speed <= MIN_EVENT_DERIVATIVE {
                return Err(UnavailableReason::IllConditioned);
            }
            Ok(StateSensitivity {
                velocity: scale(state.velocity_ecef_mps, speed.recip()),
                ..StateSensitivity::ZERO
            })
        }
        SpeedQuantity::InstantaneousHorizontal => {
            let (up, up_jacobian) = ellipsoid_up_with_jacobian(state.position_ecef_m, ellipsoid)?;
            let vertical = dot(up, state.velocity_ecef_mps);
            let horizontal = sub(state.velocity_ecef_mps, scale(up, vertical));
            let speed = norm(horizontal);
            if !speed.is_finite() || speed <= MIN_EVENT_DERIVATIVE {
                return Err(UnavailableReason::IllConditioned);
            }
            let velocity_gradient = scale(horizontal, speed.recip());
            let mut position_gradient = [0.0; 3];
            for axis in 0..3 {
                let up_derivative = [
                    up_jacobian[0][axis],
                    up_jacobian[1][axis],
                    up_jacobian[2][axis],
                ];
                // Since the horizontal unit vector is orthogonal to up, the
                // derivative of -up(up·v) reduces to this single term.
                position_gradient[axis] = -vertical * dot(velocity_gradient, up_derivative);
            }
            Ok(StateSensitivity {
                position: position_gradient,
                velocity: velocity_gradient,
                attitude: [0.0; 3],
            })
        }
        SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude => {
            let inverse = orientation_ecef_from_body.quaternion().inverse();
            let body_velocity = inverse
                .rotate_vector(
                    crate::math::Vector3::from_components(state.velocity_ecef_mps)
                        .map_err(|_| UnavailableReason::Unobservable)?,
                )
                .components();
            let body_x_ecef = orientation_ecef_from_body
                .quaternion()
                .rotate_vector(
                    crate::math::Vector3::from_components([1.0, 0.0, 0.0])
                        .map_err(|_| UnavailableReason::Unobservable)?,
                )
                .components();
            let sign = if matches!(quantity, SpeedQuantity::BodyLongitudinalMagnitude) {
                let longitudinal = body_velocity[0];
                if longitudinal.abs() <= MIN_EVENT_DERIVATIVE {
                    return Err(UnavailableReason::IllConditioned);
                }
                longitudinal.signum()
            } else {
                1.0
            };
            Ok(StateSensitivity {
                position: [0.0; 3],
                velocity: scale(body_x_ecef, sign),
                // Right-multiplicative attitude error: R' = R Exp(dtheta).
                attitude: scale([0.0, -body_velocity[2], body_velocity[1]], sign),
            })
        }
    }
}

/// Geodetic ellipsoid normal and its analytic ECEF-position Jacobian. This is
/// the differential of the same Bowring map used by trajectory queries.
pub(super) fn ellipsoid_up_with_jacobian(
    position: [f64; 3],
    ellipsoid: ReferenceEllipsoid,
) -> Result<([f64; 3], [[f64; 3]; 3]), UnavailableReason> {
    let [x, y, z] = position;
    if !position.iter().all(|value| value.is_finite()) {
        return Err(UnavailableReason::FrameUnresolved);
    }
    let horizontal = x.hypot(y);
    if horizontal <= ellipsoid.semi_major_axis_m() * 1.0e-12 {
        return Err(UnavailableReason::FrameUnresolved);
    }
    let a = ellipsoid.semi_major_axis_m();
    let flattening = ellipsoid.inverse_flattening().recip();
    let b = a * (1.0 - flattening);
    let e2 = flattening * (2.0 - flattening);
    let ep2 = (a * a - b * b) / (b * b);
    let longitude = crate::scalar_math::atan2(y, x);
    let theta = crate::scalar_math::atan2(z * a, horizontal * b);
    let (sin_theta, cos_theta) = crate::scalar_math::sin_cos(theta);
    let latitude_numerator = z + ep2 * b * sin_theta.powi(3);
    let latitude_denominator = horizontal - e2 * a * cos_theta.powi(3);
    let latitude = crate::scalar_math::atan2(latitude_numerator, latitude_denominator);
    let (sin_latitude, cos_latitude) = crate::scalar_math::sin_cos(latitude);
    let (sin_longitude, cos_longitude) = crate::scalar_math::sin_cos(longitude);
    let up = [
        cos_latitude * cos_longitude,
        cos_latitude * sin_longitude,
        sin_latitude,
    ];

    let horizontal2 = horizontal * horizontal;
    let theta_denominator = (horizontal * b).powi(2) + (z * a).powi(2);
    let latitude_denominator_squared = latitude_denominator.powi(2) + latitude_numerator.powi(2);
    if theta_denominator <= f64::MIN_POSITIVE || latitude_denominator_squared <= f64::MIN_POSITIVE {
        return Err(UnavailableReason::FrameUnresolved);
    }
    let mut jacobian = [[0.0; 3]; 3];
    for axis in 0..3 {
        let d_horizontal = match axis {
            0 => x / horizontal,
            1 => y / horizontal,
            _ => 0.0,
        };
        let d_longitude = match axis {
            0 => -y / horizontal2,
            1 => x / horizontal2,
            _ => 0.0,
        };
        let d_z = if axis == 2 { 1.0 } else { 0.0 };
        let d_theta =
            ((horizontal * b) * (a * d_z) - (z * a) * (b * d_horizontal)) / theta_denominator;
        let d_latitude_numerator = d_z + 3.0 * ep2 * b * sin_theta.powi(2) * cos_theta * d_theta;
        let d_latitude_denominator =
            d_horizontal + 3.0 * e2 * a * cos_theta.powi(2) * sin_theta * d_theta;
        let d_latitude = (latitude_denominator * d_latitude_numerator
            - latitude_numerator * d_latitude_denominator)
            / latitude_denominator_squared;
        jacobian[0][axis] =
            -sin_latitude * cos_longitude * d_latitude - cos_latitude * sin_longitude * d_longitude;
        jacobian[1][axis] =
            -sin_latitude * sin_longitude * d_latitude + cos_latitude * cos_longitude * d_longitude;
        jacobian[2][axis] = cos_latitude * d_latitude;
    }
    if up
        .iter()
        .chain(jacobian.iter().flatten())
        .all(|value| value.is_finite())
    {
        Ok((up, jacobian))
    } else {
        Err(UnavailableReason::FrameUnresolved)
    }
}
