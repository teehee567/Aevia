//! Dense segment interpolation and rigid-point kinematics.

use super::TrajectoryKnot;
#[cfg(feature = "offline")]
use super::bridge::{
    BRIDGE_ATTITUDE, BRIDGE_ENDPOINT_DIMENSION, BRIDGE_KINEMATIC_DIMENSION, BRIDGE_POSITION,
    BRIDGE_VELOCITY, DenseBridgeInput, DenseBridgeLinearization, DenseConditionalBridge,
    dense_kinematic_covariance,
};
use super::math::{add, cross, dot, norm, query_to_metric, scale, vector};
use super::quality::{conservative_covariance, conservative_observability, conservative_quality};
use crate::error::{QueryError, ValidationError};
use crate::math::{UnitQuaternion, Vector3};
use crate::metric::MetricError;
use crate::quality::{CovarianceConditioning, EstimateQuality, ObservabilityReport, Validity};
use crate::time::SessionTime;
use crate::uncertainty::KinematicCovariance;
#[cfg(not(any(feature = "offline", test)))]
use nalgebra::ComplexField;
#[cfg(feature = "offline")]
use nalgebra::DMatrix;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct OrientationBridge {
    pub(super) integrated_rotation_body: [f64; 3],
    pub(super) endpoint_correction_body: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct DenseSegment {
    pub(super) start: TrajectoryKnot,
    pub(super) end: TrajectoryKnot,
    pub(super) duration_seconds: f64,
    // Ascending coefficients in normalized segment parameter s in [0, 1].
    pub(super) position_coefficient: [[f64; 4]; 3],
    pub(super) relative_rotation_body: [f64; 3],
    #[cfg(feature = "offline")]
    pub(super) orientation_bridge: Option<OrientationBridge>,
}

impl DenseSegment {
    pub(super) fn new(start: TrajectoryKnot, end: TrajectoryKnot) -> Result<Self, ValidationError> {
        let delta = end
            .time
            .checked_duration_since(start.time)
            .ok_or(ValidationError::TimeOverflow)?;
        let duration_seconds = delta.as_seconds_f64();
        if !duration_seconds.is_finite() || duration_seconds <= 0.0 {
            return Err(ValidationError::InvalidTimeSpan);
        }
        if matches!(start.quality.validity, Validity::Invalid)
            || matches!(end.quality.validity, Validity::Invalid)
        {
            return Err(ValidationError::IncompatibleDefinition);
        }

        let p0 = start.position_ecef.components();
        let p1 = end.position_ecef.components();
        let v0 = start.velocity_ecef.components();
        let v1 = end.velocity_ecef.components();
        let mut coefficient = [[0.0; 4]; 3];
        for axis in 0..3 {
            // Hermite polynomial expressed as c0 + c1*s + c2*s^2 + c3*s^3.
            coefficient[axis][0] = p0[axis];
            coefficient[axis][1] = duration_seconds * v0[axis];
            coefficient[axis][2] = -3.0 * p0[axis] - 2.0 * duration_seconds * v0[axis]
                + 3.0 * p1[axis]
                - duration_seconds * v1[axis];
            coefficient[axis][3] = 2.0 * p0[axis] + duration_seconds * v0[axis] - 2.0 * p1[axis]
                + duration_seconds * v1[axis];
        }
        let relative = start
            .orientation_ecef_from_body
            .quaternion()
            .inverse()
            .multiply(end.orientation_ecef_from_body.quaternion())
            .rotation_vector()
            .components();
        if !relative.iter().all(|value| value.is_finite()) {
            return Err(ValidationError::InvalidRotation);
        }
        Ok(Self {
            start,
            end,
            duration_seconds,
            position_coefficient: coefficient,
            relative_rotation_body: relative,
            #[cfg(feature = "offline")]
            orientation_bridge: None,
        })
    }

    pub(super) fn new_imu_conditioned(
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        integrated_rotation_body: [f64; 3],
    ) -> Result<Self, ValidationError> {
        if integrated_rotation_body
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(ValidationError::NonFinite);
        }
        let mut segment = Self::new(start, end)?;
        segment.relative_rotation_body = integrated_rotation_body;
        // Validate the implied endpoint correction now so an invalid segment
        // cannot enter the rolling trajectory and fail only at query time.
        segment.derived_orientation_bridge()?;
        Ok(segment)
    }

    pub(super) fn derived_orientation_bridge(&self) -> Result<OrientationBridge, ValidationError> {
        #[cfg(feature = "offline")]
        if let Some(bridge) = self.orientation_bridge {
            return Ok(bridge);
        }
        let integrated = UnitQuaternion::from_rotation_vector(Vector3::from_components(
            self.relative_rotation_body,
        )?)?;
        let predicted_end = self
            .start
            .orientation_ecef_from_body
            .quaternion()
            .multiply(integrated);
        let correction = predicted_end
            .inverse()
            .multiply(self.end.orientation_ecef_from_body.quaternion())
            .rotation_vector()
            .components();
        if correction.iter().any(|value| !value.is_finite())
            || norm(correction) >= core::f64::consts::PI - 1.0e-8
        {
            return Err(ValidationError::InvalidRotation);
        }
        Ok(OrientationBridge {
            integrated_rotation_body: self.relative_rotation_body,
            endpoint_correction_body: correction,
        })
    }

    #[cfg(feature = "offline")]
    pub(super) fn new_conditional(
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        input: &DenseBridgeInput,
    ) -> Result<(Self, DenseConditionalBridge), ValidationError> {
        let mut segment = Self::new(start, end)?;
        if input
            .coupled
            .as_ref()
            .is_some_and(|model| model.duration_seconds != segment.duration_seconds)
        {
            return Err(ValidationError::InvalidTimeSpan);
        }
        if !input
            .reintegrated_position_ecef_m
            .iter()
            .chain(input.reintegrated_velocity_ecef_mps.iter())
            .chain(input.integrated_rotation_body.iter())
            .all(|value| value.is_finite())
        {
            return Err(ValidationError::NonFinite);
        }
        if !input.covariance_available {
            // A coupled inertial process cannot be encoded as independent
            // position/velocity and attitude bridge noises. Preserve the
            // endpoint-conditioned kinematics without certifying that model.
            return Ok((
                Self::new_imu_conditioned(start, end, input.integrated_rotation_body)?,
                DenseConditionalBridge::new(input)?,
            ));
        }

        // A cubic nominal matches the reintegrated position and velocity
        // independently, including rotating-force motion. Condition its error
        // on both optimized endpoints with the integrated-Wiener bridge.
        // Their sum is the endpoint Hermite curve.
        let p0 = start.position_ecef.components();
        let v0 = start.velocity_ecef.components();
        let p1 = end.position_ecef.components();
        let v1 = end.velocity_ecef.components();
        let duration = segment.duration_seconds;
        for axis in 0..3 {
            let nominal_position_delta =
                input.reintegrated_position_ecef_m[axis] - p0[axis] - duration * v0[axis];
            let nominal_velocity_delta = input.reintegrated_velocity_ecef_mps[axis] - v0[axis];
            let nominal_c2 = 3.0 * nominal_position_delta - duration * nominal_velocity_delta;
            let nominal_c3 = -2.0 * nominal_position_delta + duration * nominal_velocity_delta;
            let position_error = p1[axis] - input.reintegrated_position_ecef_m[axis];
            let velocity_error = v1[axis] - input.reintegrated_velocity_ecef_mps[axis];
            segment.position_coefficient[axis] = [
                p0[axis],
                duration * v0[axis],
                nominal_c2 + 3.0 * position_error - duration * velocity_error,
                nominal_c3 - 2.0 * position_error + duration * velocity_error,
            ];
        }

        let integrated = UnitQuaternion::from_rotation_vector(Vector3::from_components(
            input.integrated_rotation_body,
        )?)?;
        let predicted_end = start
            .orientation_ecef_from_body
            .quaternion()
            .multiply(integrated);
        let correction = predicted_end
            .inverse()
            .multiply(end.orientation_ecef_from_body.quaternion())
            .rotation_vector()
            .components();
        if correction.iter().any(|value| !value.is_finite())
            || norm(correction) >= core::f64::consts::PI - 1.0e-8
        {
            return Err(ValidationError::InvalidRotation);
        }
        segment.orientation_bridge = Some(OrientationBridge {
            integrated_rotation_body: input.integrated_rotation_body,
            endpoint_correction_body: correction,
        });
        let bridge = DenseConditionalBridge::new(input)?;
        Ok((segment, bridge))
    }

    pub(super) fn contains(&self, time: SessionTime, include_end: bool) -> bool {
        time >= self.start.time && (time < self.end.time || (include_end && time == self.end.time))
    }

    pub(super) fn parameter_at(&self, time: SessionTime) -> Result<f64, QueryError> {
        let elapsed = time
            .checked_duration_since(self.start.time)
            .ok_or(QueryError::TrajectoryInvalid)?
            .as_seconds_f64();
        let parameter = elapsed / self.duration_seconds;
        if !parameter.is_finite() || !(-f64::EPSILON..=1.0 + f64::EPSILON).contains(&parameter) {
            return Err(QueryError::InvalidRequest);
        }
        Ok(parameter.clamp(0.0, 1.0))
    }

    pub(super) fn time_at(&self, parameter: f64) -> Result<SessionTime, MetricError> {
        if !parameter.is_finite() || !(0.0..=1.0).contains(&parameter) {
            return Err(MetricError::OutsideTrajectory);
        }
        let duration_ns = self
            .end
            .time
            .as_ns()
            .saturating_sub(self.start.time.as_ns());
        let offset = (parameter * duration_ns as f64).round();
        if !offset.is_finite() || offset < i64::MIN as f64 || offset > i64::MAX as f64 {
            return Err(MetricError::NumericalFailure);
        }
        self.start
            .time
            .as_ns()
            .checked_add(offset as i64)
            .map(SessionTime::from_ns)
            .ok_or(MetricError::NumericalFailure)
    }

    pub(super) fn base_kinematics(&self, parameter: f64) -> Result<BaseKinematics, QueryError> {
        if !parameter.is_finite() || !(0.0..=1.0).contains(&parameter) {
            return Err(QueryError::InvalidRequest);
        }
        let s2 = parameter * parameter;
        let mut position = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut acceleration = [0.0; 3];
        for axis in 0..3 {
            let c = self.position_coefficient[axis];
            position[axis] = c[0] + parameter * (c[1] + parameter * (c[2] + parameter * c[3]));
            velocity[axis] =
                (c[1] + 2.0 * c[2] * parameter + 3.0 * c[3] * s2) / self.duration_seconds;
            acceleration[axis] =
                (2.0 * c[2] + 6.0 * c[3] * parameter) / self.duration_seconds.powi(2);
        }

        let q0 = self.start.orientation_ecef_from_body.quaternion();
        let bridge = self
            .derived_orientation_bridge()
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        let (orientation, normalized_rate, normalized_acceleration, _) = bridge
            .kinematics(q0, parameter)
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        let angular_rate = scale(normalized_rate, self.duration_seconds.recip());
        let angular_acceleration = scale(
            normalized_acceleration,
            self.duration_seconds.powi(2).recip(),
        );
        let orientation = if parameter == 0.0 {
            q0
        } else if parameter == 1.0 {
            self.end.orientation_ecef_from_body.quaternion()
        } else {
            orientation
        };
        let f0 = self.start.specific_force_body.components();
        let f1 = self.end.specific_force_body.components();
        let specific_force = add(scale(f0, 1.0 - parameter), scale(f1, parameter));

        Ok(BaseKinematics {
            position,
            velocity,
            acceleration,
            orientation,
            angular_rate_body: angular_rate,
            angular_acceleration_body: angular_acceleration,
            specific_force_body: specific_force,
        })
    }

    pub(super) fn quality_at(
        &self,
        parameter: f64,
        conditional_covariance: bool,
    ) -> (EstimateQuality, ObservabilityReport) {
        if parameter == 0.0 {
            return (self.start.quality, self.start.observability);
        }
        if parameter == 1.0 {
            return (self.end.quality, self.end.observability);
        }
        let mut quality = conservative_quality(self.start.quality, self.end.quality);
        if !conditional_covariance {
            // A Hermite segment constructed from endpoint marginals alone has
            // no justified joint covariance interpolation. Return a
            // conservative numeric envelope for diagnostics but label it
            // unavailable.
            quality.covariance = CovarianceConditioning::Unavailable;
        }
        (
            quality,
            conservative_observability(self.start.observability, self.end.observability),
        )
    }

    pub(super) fn covariance_at(
        &self,
        parameter: f64,
        #[cfg(feature = "offline")] bridge: Option<&DenseConditionalBridge>,
    ) -> Result<KinematicCovariance, QueryError> {
        if parameter == 0.0 {
            return Ok(self.start.covariance);
        }
        if parameter == 1.0 {
            return Ok(self.end.covariance);
        }
        #[cfg(feature = "offline")]
        if let Some(bridge) = bridge {
            let linearization = self.bridge_endpoint_jacobians(parameter)?;
            let mut interpolation =
                DMatrix::zeros(BRIDGE_KINEMATIC_DIMENSION, BRIDGE_ENDPOINT_DIMENSION);
            interpolation
                .view_mut(
                    (0, 0),
                    (BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION),
                )
                .copy_from(&linearization.start_jacobian);
            interpolation
                .view_mut(
                    (0, BRIDGE_KINEMATIC_DIMENSION),
                    (BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION),
                )
                .copy_from(&linearization.end_jacobian);
            let covariance = &interpolation * bridge.endpoint_joint() * interpolation.transpose()
                + bridge.conditional_process_cross(self.duration_seconds, parameter, parameter)?;
            return dense_kinematic_covariance(&covariance);
        }
        conservative_covariance(self.start.covariance, self.end.covariance)
            .map_err(|_| QueryError::TrajectoryInvalid)
    }

    #[cfg(feature = "offline")]
    pub(super) fn bridge_endpoint_jacobians(
        &self,
        parameter: f64,
    ) -> Result<DenseBridgeLinearization, QueryError> {
        if self.orientation_bridge.is_none()
            || !parameter.is_finite()
            || !(0.0..=1.0).contains(&parameter)
        {
            return Err(QueryError::TrajectoryInvalid);
        }
        let mut start = DMatrix::zeros(BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION);
        let mut end = DMatrix::zeros(BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION);
        if parameter == 0.0 {
            start.fill_diagonal(1.0);
            return Ok(DenseBridgeLinearization {
                start_jacobian: start,
                end_jacobian: end,
            });
        }
        if parameter == 1.0 {
            end.fill_diagonal(1.0);
            return Ok(DenseBridgeLinearization {
                start_jacobian: start,
                end_jacobian: end,
            });
        }
        let s2 = parameter * parameter;
        let s3 = s2 * parameter;
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = s3 - 2.0 * s2 + parameter;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = s3 - s2;
        let dv_p0 = (6.0 * s2 - 6.0 * parameter) / self.duration_seconds;
        let dv_v0 = 3.0 * s2 - 4.0 * parameter + 1.0;
        let dv_p1 = (-6.0 * s2 + 6.0 * parameter) / self.duration_seconds;
        let dv_v1 = 3.0 * s2 - 2.0 * parameter;
        for axis in 0..3 {
            start[(BRIDGE_POSITION + axis, BRIDGE_POSITION + axis)] = h00;
            start[(BRIDGE_POSITION + axis, BRIDGE_VELOCITY + axis)] = self.duration_seconds * h10;
            start[(BRIDGE_VELOCITY + axis, BRIDGE_POSITION + axis)] = dv_p0;
            start[(BRIDGE_VELOCITY + axis, BRIDGE_VELOCITY + axis)] = dv_v0;
            end[(BRIDGE_POSITION + axis, BRIDGE_POSITION + axis)] = h01;
            end[(BRIDGE_POSITION + axis, BRIDGE_VELOCITY + axis)] = self.duration_seconds * h11;
            end[(BRIDGE_VELOCITY + axis, BRIDGE_POSITION + axis)] = dv_p1;
            end[(BRIDGE_VELOCITY + axis, BRIDGE_VELOCITY + axis)] = dv_v1;
        }
        let (attitude_start, attitude_end) = self.orientation_endpoint_jacobians(parameter)?;
        for row in 0..3 {
            for column in 0..3 {
                start[(BRIDGE_ATTITUDE + row, BRIDGE_ATTITUDE + column)] =
                    attitude_start[(row, column)];
                end[(BRIDGE_ATTITUDE + row, BRIDGE_ATTITUDE + column)] =
                    attitude_end[(row, column)];
            }
        }
        if !start
            .iter()
            .chain(end.iter())
            .all(|value| value.is_finite())
        {
            return Err(QueryError::TrajectoryInvalid);
        }
        Ok(DenseBridgeLinearization {
            start_jacobian: start,
            end_jacobian: end,
        })
    }

    #[cfg(feature = "offline")]
    pub(super) fn orientation_endpoint_jacobians(
        &self,
        parameter: f64,
    ) -> Result<(DMatrix<f64>, DMatrix<f64>), QueryError> {
        let nominal = self.bridge_orientation_for_endpoints(
            self.start.orientation_ecef_from_body.quaternion(),
            self.end.orientation_ecef_from_body.quaternion(),
            parameter,
        )?;
        let epsilon = 1.0e-6;
        let mut start_jacobian = DMatrix::zeros(3, 3);
        let mut end_jacobian = DMatrix::zeros(3, 3);
        for axis in 0..3 {
            let mut delta = [0.0; 3];
            delta[axis] = epsilon;
            let positive = UnitQuaternion::from_rotation_vector(
                Vector3::from_components(delta).map_err(|_| QueryError::TrajectoryInvalid)?,
            )
            .map_err(|_| QueryError::TrajectoryInvalid)?;
            delta[axis] = -epsilon;
            let negative = UnitQuaternion::from_rotation_vector(
                Vector3::from_components(delta).map_err(|_| QueryError::TrajectoryInvalid)?,
            )
            .map_err(|_| QueryError::TrajectoryInvalid)?;
            for (is_start, target) in [(true, &mut start_jacobian), (false, &mut end_jacobian)] {
                let start_positive = if is_start {
                    self.start
                        .orientation_ecef_from_body
                        .quaternion()
                        .multiply(positive)
                } else {
                    self.start.orientation_ecef_from_body.quaternion()
                };
                let start_negative = if is_start {
                    self.start
                        .orientation_ecef_from_body
                        .quaternion()
                        .multiply(negative)
                } else {
                    self.start.orientation_ecef_from_body.quaternion()
                };
                let end_positive = if is_start {
                    self.end.orientation_ecef_from_body.quaternion()
                } else {
                    self.end
                        .orientation_ecef_from_body
                        .quaternion()
                        .multiply(positive)
                };
                let end_negative = if is_start {
                    self.end.orientation_ecef_from_body.quaternion()
                } else {
                    self.end
                        .orientation_ecef_from_body
                        .quaternion()
                        .multiply(negative)
                };
                let positive_output =
                    self.bridge_orientation_for_endpoints(start_positive, end_positive, parameter)?;
                let negative_output =
                    self.bridge_orientation_for_endpoints(start_negative, end_negative, parameter)?;
                let positive_error = nominal
                    .inverse()
                    .multiply(positive_output)
                    .rotation_vector()
                    .components();
                let negative_error = nominal
                    .inverse()
                    .multiply(negative_output)
                    .rotation_vector()
                    .components();
                for row in 0..3 {
                    target[(row, axis)] =
                        (positive_error[row] - negative_error[row]) / (2.0 * epsilon);
                }
            }
        }
        Ok((start_jacobian, end_jacobian))
    }

    #[cfg(feature = "offline")]
    pub(super) fn bridge_orientation_for_endpoints(
        &self,
        start: UnitQuaternion,
        end: UnitQuaternion,
        parameter: f64,
    ) -> Result<UnitQuaternion, QueryError> {
        let bridge = self
            .orientation_bridge
            .ok_or(QueryError::TrajectoryInvalid)?;
        let integrated_full = UnitQuaternion::from_rotation_vector(
            Vector3::from_components(bridge.integrated_rotation_body)
                .map_err(|_| QueryError::TrajectoryInvalid)?,
        )
        .map_err(|_| QueryError::TrajectoryInvalid)?;
        let integrated_partial = UnitQuaternion::from_rotation_vector(
            Vector3::from_components(scale(bridge.integrated_rotation_body, parameter))
                .map_err(|_| QueryError::TrajectoryInvalid)?,
        )
        .map_err(|_| QueryError::TrajectoryInvalid)?;
        let correction = start
            .multiply(integrated_full)
            .inverse()
            .multiply(end)
            .rotation_vector()
            .components();
        let correction_partial = UnitQuaternion::from_rotation_vector(
            Vector3::from_components(scale(correction, parameter))
                .map_err(|_| QueryError::TrajectoryInvalid)?,
        )
        .map_err(|_| QueryError::TrajectoryInvalid)?;
        Ok(start
            .multiply(integrated_partial)
            .multiply(correction_partial))
    }

    pub(super) fn plane_polynomial(&self, center: [f64; 3], normal: [f64; 3]) -> [f64; 5] {
        let mut result = [-dot(center, normal), 0.0, 0.0, 0.0, 0.0];
        for axis in 0..3 {
            for power in 0..4 {
                result[power] += normal[axis] * self.position_coefficient[axis][power];
            }
        }
        result
    }

    pub(super) fn spatial_speed_squared_polynomial(&self, target_mps: f64) -> [f64; 5] {
        let mut result = [0.0; 5];
        result[0] = -target_mps * target_mps;
        for axis in 0..3 {
            let c = self.position_coefficient[axis];
            let velocity = [
                c[1] / self.duration_seconds,
                2.0 * c[2] / self.duration_seconds,
                3.0 * c[3] / self.duration_seconds,
            ];
            for left in 0..3 {
                for right in 0..3 {
                    result[left + right] += velocity[left] * velocity[right];
                }
            }
        }
        result
    }

    /// Exact derivatives of the selected rigid point with respect to the
    /// normalized Hermite parameter. Rotation is `R(0) exp(s [phi]x)`, so the
    /// kth lever derivative is `R(s) [phi]x^k r`.
    pub(super) fn point_jet(
        &self,
        parameter: f64,
        lever_body_m: [f64; 3],
    ) -> Result<PointJet, MetricError> {
        if !parameter.is_finite() || !(0.0..=1.0).contains(&parameter) {
            return Err(MetricError::InvalidDefinition);
        }
        let base = self.base_kinematics(parameter).map_err(query_to_metric)?;
        let mut position = [0.0; 3];
        let mut first = [0.0; 3];
        let mut second = [0.0; 3];
        let mut third = [0.0; 3];
        for axis in 0..3 {
            let [c0, c1, c2, c3] = self.position_coefficient[axis];
            position[axis] = c0 + parameter * (c1 + parameter * (c2 + parameter * c3));
            first[axis] = c1 + parameter * (2.0 * c2 + parameter * 3.0 * c3);
            second[axis] = 2.0 * c2 + parameter * 6.0 * c3;
            third[axis] = 6.0 * c3;
        }

        let bridge = self
            .derived_orientation_bridge()
            .map_err(|_| MetricError::NumericalFailure)?;
        let (_, omega, omega_derivative, omega_second_derivative) = bridge
            .kinematics(
                self.start.orientation_ecef_from_body.quaternion(),
                parameter,
            )
            .map_err(|_| MetricError::NumericalFailure)?;
        let first_body = cross(omega, lever_body_m);
        let second_body = add(
            cross(omega, first_body),
            cross(omega_derivative, lever_body_m),
        );
        let third_body = add(
            cross(omega, second_body),
            add(
                cross(omega_derivative, first_body),
                add(
                    cross(omega, cross(omega_derivative, lever_body_m)),
                    cross(omega_second_derivative, lever_body_m),
                ),
            ),
        );
        let rotated = [
            base.orientation
                .rotate_vector(vector(lever_body_m).map_err(|_| MetricError::NumericalFailure)?)
                .components(),
            base.orientation
                .rotate_vector(vector(first_body).map_err(|_| MetricError::NumericalFailure)?)
                .components(),
            base.orientation
                .rotate_vector(vector(second_body).map_err(|_| MetricError::NumericalFailure)?)
                .components(),
            base.orientation
                .rotate_vector(vector(third_body).map_err(|_| MetricError::NumericalFailure)?)
                .components(),
        ];

        Ok(PointJet {
            position: add(position, rotated[0]),
            first: add(first, rotated[1]),
            second: add(second, rotated[2]),
            third: add(third, rotated[3]),
            orientation: base.orientation,
        })
    }
}

impl OrientationBridge {
    pub(super) fn kinematics(
        self,
        start: UnitQuaternion,
        parameter: f64,
    ) -> Result<(UnitQuaternion, [f64; 3], [f64; 3], [f64; 3]), ValidationError> {
        if !parameter.is_finite() || !(0.0..=1.0).contains(&parameter) {
            return Err(ValidationError::InvalidTimeSpan);
        }
        let integrated = UnitQuaternion::from_rotation_vector(Vector3::from_components(scale(
            self.integrated_rotation_body,
            parameter,
        ))?)?;
        let correction = UnitQuaternion::from_rotation_vector(Vector3::from_components(scale(
            self.endpoint_correction_body,
            parameter,
        ))?)?;
        let orientation = start.multiply(integrated).multiply(correction);
        let rotated_integrated = correction
            .inverse()
            .rotate_vector(Vector3::from_components(self.integrated_rotation_body)?)
            .components();
        let rate = add(rotated_integrated, self.endpoint_correction_body);
        let rate_derivative = scale(
            cross(self.endpoint_correction_body, rotated_integrated),
            -1.0,
        );
        let rate_second_derivative = cross(
            self.endpoint_correction_body,
            cross(self.endpoint_correction_body, rotated_integrated),
        );
        Ok((orientation, rate, rate_derivative, rate_second_derivative))
    }
}

#[derive(Clone, Copy)]
pub(super) struct BaseKinematics {
    pub(super) position: [f64; 3],
    pub(super) velocity: [f64; 3],
    pub(super) acceleration: [f64; 3],
    pub(super) orientation: UnitQuaternion,
    pub(super) angular_rate_body: [f64; 3],
    pub(super) angular_acceleration_body: [f64; 3],
    pub(super) specific_force_body: [f64; 3],
}

#[derive(Clone, Copy)]
pub(super) struct PointJet {
    pub(super) position: [f64; 3],
    pub(super) first: [f64; 3],
    pub(super) second: [f64; 3],
    pub(super) third: [f64; 3],
    pub(super) orientation: UnitQuaternion,
}
