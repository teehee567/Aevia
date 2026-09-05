//! Public trajectory state and scalar queries.

use super::math::{add, cross, dot, metric_to_query, tangent_basis, vector};
use super::{KinematicEstimate, Trajectory};
use crate::error::QueryError;
use crate::frame::{
    BodyAngularAcceleration, BodyAngularRate, BodyVector, EcefAcceleration, EcefPosition,
    EcefVelocity, OrientationEcefFromBody, OutputFrame,
};
use crate::ids::ReferencePointId;
#[cfg(any(test, feature = "offline"))]
use crate::metric::{MetricError, MetricPlan, MetricResults};
use crate::quality::{
    CovarianceConditioning, FieldValue, HeadingObservability, HeadingSource, UnavailableReason,
};
use crate::time::{SessionTime, TimeSpan};
#[cfg(not(any(feature = "offline", test)))]
use nalgebra::ComplexField;

impl Trajectory {
    /// Evaluates the path at an exact session time and named rigid point.
    pub fn state_at(
        &self,
        time: SessionTime,
        reference_point: ReferencePointId,
    ) -> Result<KinematicEstimate, QueryError> {
        let (segment_index, parameter) = self.locate(time)?;
        self.evaluate(segment_index, parameter, reference_point)
    }

    /// Explicit instantaneous horizontal speed in the ellipsoidal tangent
    /// plane at the selected point.
    pub fn horizontal_speed_at(
        &self,
        time: SessionTime,
        reference_point: ReferencePointId,
    ) -> Result<f64, QueryError> {
        self.scalar_kinematics_at(time, reference_point)
            .map(|state| state.horizontal_speed_mps)
            .map_err(metric_to_query)
    }

    /// Explicit signed vertical speed along the instantaneous ellipsoid normal.
    pub fn vertical_speed_at(
        &self,
        time: SessionTime,
        reference_point: ReferencePointId,
    ) -> Result<f64, QueryError> {
        self.scalar_kinematics_at(time, reference_point)
            .map(|state| state.vertical_speed_mps)
            .map_err(metric_to_query)
    }

    /// Course over ground in radians clockwise from geodetic north. Near a
    /// pole or below observability/SNR gates it is explicitly unavailable.
    pub fn course_over_ground_at(
        &self,
        time: SessionTime,
        reference_point: ReferencePointId,
    ) -> Result<FieldValue<f64>, QueryError> {
        let estimate = self.state_at(time, reference_point)?;
        if !estimate.observability.course_available {
            return Ok(FieldValue::Unavailable(
                UnavailableReason::InsufficientSignalToNoise,
            ));
        }
        let position = estimate.position.components();
        let velocity = estimate.velocity.components();
        let basis = tangent_basis(position, self.frame.ellipsoid()).map_err(metric_to_query)?;
        let (Some(east), Some(north)) = (basis.east, basis.north) else {
            return Ok(FieldValue::Unavailable(UnavailableReason::FrameUnresolved));
        };
        let east_speed = dot(velocity, east);
        let north_speed = dot(velocity, north);
        if east_speed.hypot(north_speed) <= f64::EPSILON {
            return Ok(FieldValue::Unavailable(
                UnavailableReason::InsufficientSignalToNoise,
            ));
        }
        Ok(FieldValue::Available(crate::scalar_math::atan2(
            east_speed,
            north_speed,
        )))
    }

    /// Evaluates a shared semantic metric plan through the engine-owned dense
    /// cursor rather than display samples.
    #[cfg(any(test, feature = "offline"))]
    pub fn measure(&self, plan: &MetricPlan) -> Result<MetricResults, MetricError> {
        plan.evaluate(self)
    }

    pub(super) fn locate(&self, time: SessionTime) -> Result<(usize, f64), QueryError> {
        let span = self.span();
        if span.is_none_or(|available| !available.contains(time)) {
            return Err(QueryError::OutsideAvailableSpan {
                requested: time,
                earliest: span.map(TimeSpan::start),
                latest: span.map(TimeSpan::end),
            });
        }
        let count = self.segment_count();
        let mut lower = 0_usize;
        let mut upper = count;
        while lower < upper {
            let index = lower + (upper - lower) / 2;
            let lease = self.segment_lease(index)?;
            let segment = lease.segment();
            if time < segment.start.time {
                upper = index;
            } else if time > segment.end.time || (time == segment.end.time && index + 1 < count) {
                lower = index + 1;
            } else if segment.contains(time, index + 1 == count) {
                return Ok((index, segment.parameter_at(time)?));
            } else {
                break;
            }
        }
        Err(QueryError::OutsideAvailableSpan {
            requested: time,
            earliest: span.map(TimeSpan::start),
            latest: span.map(TimeSpan::end),
        })
    }

    pub(super) fn evaluate(
        &self,
        segment_index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<KinematicEstimate, QueryError> {
        let lease = self.segment_lease(segment_index)?;
        let segment = lease.segment();
        #[cfg(feature = "offline")]
        let conditional_bridge = lease.conditional_bridge();
        let reference = self.reference_point_for_query(reference_point)?;
        let mut base = segment.base_kinematics(parameter)?;
        let lever_body = reference.imu_to_point().components_m();
        let lever_ecef = base
            .orientation
            .rotate_vector(vector(lever_body)?)
            .components();
        let rotational_velocity_body = cross(base.angular_rate_body, lever_body);
        let rotational_velocity_ecef = base
            .orientation
            .rotate_vector(vector(rotational_velocity_body)?)
            .components();
        let rotational_acceleration_body = add(
            cross(base.angular_acceleration_body, lever_body),
            cross(
                base.angular_rate_body,
                cross(base.angular_rate_body, lever_body),
            ),
        );
        let rotational_acceleration_ecef = base
            .orientation
            .rotate_vector(vector(rotational_acceleration_body)?)
            .components();
        base.position = add(base.position, lever_ecef);
        base.velocity = add(base.velocity, rotational_velocity_ecef);
        base.acceleration = add(base.acceleration, rotational_acceleration_ecef);

        let (mut quality, mut observability) = segment.quality_at(parameter, {
            #[cfg(feature = "offline")]
            {
                conditional_bridge.is_some()
            }
            #[cfg(not(feature = "offline"))]
            {
                false
            }
        });
        if !self.attachment.permits_body_axis_quantities() {
            // A supplied or observable device yaw does not establish a
            // vehicle/person forward axis. Preserve frame-independent course
            // and vector kinematics, but withdraw every body-axis claim.
            observability.heading_source = HeadingSource::None;
            observability.heading = HeadingObservability::Unobservable;
            observability.heading_variance_rad2 = None;
            observability.body_axis_quantities_available = false;
        }
        if lever_body != [0.0; 3] {
            // The dense segment currently retains no state/attitude/lever-arm
            // cross block, so a transformed numeric covariance would be
            // incomplete even when the lever itself is surveyed exactly.
            quality.covariance = CovarianceConditioning::Unavailable;
        }
        let time = segment
            .time_at(parameter)
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        let angular_acceleration =
            BodyAngularAcceleration::from_components(base.angular_acceleration_body)
                .map_err(|_| QueryError::TrajectoryInvalid)?;
        let angular_acceleration = if observability.angular_acceleration_available {
            FieldValue::Available(angular_acceleration)
        } else {
            FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
        };
        let kinematic_acceleration = EcefAcceleration::from_components(base.acceleration)
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        let kinematic_acceleration =
            if lever_body == [0.0; 3] || observability.angular_acceleration_available {
                FieldValue::Available(kinematic_acceleration)
            } else {
                FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
            };
        Ok(KinematicEstimate {
            time,
            reference_point,
            frame: OutputFrame::Ecef(self.frame.id()),
            position: EcefPosition::from_components(base.position)
                .map_err(|_| QueryError::TrajectoryInvalid)?,
            velocity: EcefVelocity::from_components(base.velocity)
                .map_err(|_| QueryError::TrajectoryInvalid)?,
            orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(base.orientation),
            angular_rate_body_relative_ecef: BodyAngularRate::from_components(
                base.angular_rate_body,
            )
            .map_err(|_| QueryError::TrajectoryInvalid)?,
            angular_acceleration_body_relative_ecef: angular_acceleration,
            kinematic_acceleration,
            specific_force_body: BodyVector::from_components(base.specific_force_body)
                .map_err(|_| QueryError::TrajectoryInvalid)?,
            covariance: segment.covariance_at(
                parameter,
                #[cfg(feature = "offline")]
                conditional_bridge,
            )?,
            quality,
            observability,
            revision: self.revision,
        })
    }
}
