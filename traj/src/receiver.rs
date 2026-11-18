//! Receiver-only measurements when inertial preparation is unavailable.
//!
//! This deliberately does not return a `Trajectory` or an attitude estimate.
//! It interpolates co-timed receiver position/velocity fixes with the existing
//! Hermite model and evaluates the existing metric definitions. All results
//! are degraded and statistical uncertainty is unavailable: receiver marginal
//! accuracy alone does not describe interpolation error or temporal correlation.
//! Invalid fixes, clock resets and acquisition gaps must split the input runs.
//! Callers must also check position/velocity consistency before admission:
//! differentiating a Hermite path through noisy positions can exceed both
//! endpoint speeds. This wrapper supplies interpolation, not a GNSS noise filter
//! or a guarantee that receiver evidence supports a speed-threshold crossing.

use crate::{
    config::AttachmentModel,
    error::ValidationError,
    frame::{
        BodyLeverArm, BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody,
        ReferencePoint, ReferencePointKind, TerrestrialFrame,
    },
    ids::{ReferencePointId, SharedParameterId, TrajectoryRevision},
    math::UnitQuaternion,
    metric::{MetricError, MetricPlan, MetricResults},
    quality::{
        CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
        HeadingSource, Integrity, ObservabilityReport, TimingQuality, Validity,
    },
    time::{DurationNs, SessionTime},
    trajectory::{Trajectory, TrajectoryKnot},
    uncertainty::{Covariance3, KinematicCovariance, MeasurementUncertainty},
};

/// A valid receiver fix at one common effective position/velocity epoch.
#[derive(Clone, Copy, Debug)]
pub struct ReceiverFix {
    pub time: SessionTime,
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    pub timing: TimingQuality,
}

/// A contiguous receiver-only path. It cannot expose body/IMU quantities.
pub struct ReceiverTrack {
    path: Trajectory,
}

impl ReceiverTrack {
    /// Builds one continuous run; rejects repeated epochs and gaps instead of
    /// interpolating across them. Callers retain invalid records separately.
    pub fn from_fixes(
        frame: TerrestrialFrame,
        point: ReferencePointId,
        fixes: &[ReceiverFix],
        maximum_gap: DurationNs,
    ) -> Result<Self, ValidationError> {
        if fixes.len() < 2 || maximum_gap == DurationNs::ZERO {
            return Err(ValidationError::InvalidTimeSpan);
        }
        let mut path = Trajectory::new(frame, TrajectoryRevision::new(1));
        path.set_attachment_model(AttachmentModel::DeviceTrajectoryOnly)?;
        path.add_reference_point(ReferencePoint::new(
            point,
            ReferencePointKind::GnssAntennaPhaseCenter,
            BodyLeverArm::new(0.0, 0.0, 0.0)?,
            SharedParameterId::new(1),
            // Exact identity offset from the receiver point to itself.
            MeasurementUncertainty::Provided(Covariance3::ZERO),
        ))?;
        for pair in fixes.windows(2) {
            let dt = pair[1]
                .time
                .checked_duration_since(pair[0].time)
                .ok_or(ValidationError::TimeOverflow)?
                .as_ns();
            if dt <= 0 || dt as u64 > maximum_gap.as_ns() {
                return Err(ValidationError::InvalidTimeSpan);
            }
            path.push_hermite_segment(knot(pair[0])?, knot(pair[1])?)?;
        }
        Ok(Self { path })
    }

    /// Uses the same lap, drag and distance algorithms as fused trajectories.
    /// The private device-only reference point excludes body-relative metrics.
    pub fn measure(&self, plan: &MetricPlan) -> Result<MetricResults, MetricError> {
        plan.evaluate(&self.path)
    }

    /// Receiver-point horizontal speed at a measured event. This is not a
    /// trap-window average and has the same GNSS-only limitations as metrics.
    pub fn horizontal_speed_at(
        &self,
        time: SessionTime,
        point: ReferencePointId,
    ) -> Result<f64, crate::QueryError> {
        self.path.horizontal_speed_at(time, point)
    }

    /// Explicit host work allowance for a longer receiver recording.
    pub fn measure_with_limits(
        &self,
        plan: &MetricPlan,
        limits: crate::metric::MetricEvaluationLimits,
    ) -> Result<MetricResults, MetricError> {
        plan.evaluate_with_limits(&self.path, limits)
    }
}

fn knot(fix: ReceiverFix) -> Result<TrajectoryKnot, ValidationError> {
    if matches!(fix.timing, TimingQuality::Discontinuous) {
        return Err(ValidationError::InvalidTimeSpan);
    }
    // The dense representation requires finite storage for attitude and
    // covariance. These placeholders cannot escape this wrapper as estimates;
    // covariance=Unavailable disables all statistical uncertainty publication.
    let placeholder = Covariance3::diagonal(1.0, 1.0, 1.0)?;
    Ok(TrajectoryKnot {
        time: fix.time,
        position_ecef: fix.position,
        velocity_ecef: fix.velocity,
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0)?,
        covariance: KinematicCovariance::new(placeholder, placeholder, None, placeholder)?,
        quality: EstimateQuality {
            stage: EstimateStage::Finalized,
            validity: Validity::Degraded,
            gnss: GnssState::Healthy,
            timing: fix.timing,
            integrity: Integrity::Unavailable,
            covariance: CovarianceConditioning::Unavailable,
            imu_gap: false,
            degraded_input: true,
        },
        observability: ObservabilityReport {
            heading_source: HeadingSource::None,
            heading: HeadingObservability::Unobservable,
            heading_variance_rad2: None,
            course_available: true,
            body_axis_quantities_available: false,
            angular_acceleration_available: false,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        frame::{CoordinateEpoch, ReferenceEllipsoid, TerrestrialRealization, Wgs84Realization},
        ids::{FrameId, MetricDefinitionId},
        metric::{DistancePlan, DistanceQuantity, MetricDefinition, MetricResultValue},
        quality::FieldValue,
    };

    fn frame() -> TerrestrialFrame {
        TerrestrialFrame::new(
            FrameId::new(1),
            TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
            CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
            ReferenceEllipsoid::WGS84,
        )
    }
    fn fixes() -> [ReceiverFix; 2] {
        [0, 1].map(|i| ReceiverFix {
            time: SessionTime::from_ns(i * 1_000_000_000),
            position: EcefPosition::new(6_378_137.0, i as f64 * 10.0, 0.0).unwrap(),
            velocity: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
            timing: TimingQuality::ArrivalOnly,
        })
    }
    #[test]
    fn receiver_distance_is_degraded_without_statistical_precision() {
        let path = ReceiverTrack::from_fixes(
            frame(),
            ReferencePointId::new(1),
            &fixes(),
            DurationNs::from_ns(1_000_000_000),
        )
        .unwrap();
        let mut plan = MetricPlan::new(1);
        plan.push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(1),
            reference_point: ReferencePointId::new(1),
            quantity: DistanceQuantity::HorizontalPath,
            absolute_tolerance_m: 0.001,
            relative_tolerance: 1e-6,
        }))
        .unwrap();
        let result = path.measure(&plan).unwrap();
        let MetricResultValue::Distance(report) = result.as_slice()[0].value else {
            panic!("distance missing")
        };
        assert!((report.metres - 10.0).abs() < 0.001);
        assert_eq!(report.validity, Validity::Degraded);
        assert!(matches!(
            report.uncertainty_one_sigma_m,
            FieldValue::Unavailable(_)
        ));
    }
    #[test]
    fn rejects_gaps_and_repeated_epochs() {
        assert!(
            ReceiverTrack::from_fixes(
                frame(),
                ReferencePointId::new(1),
                &fixes(),
                DurationNs::from_ns(500_000_000)
            )
            .is_err()
        );
        let mut repeated = fixes();
        repeated[1].time = repeated[0].time;
        assert!(
            ReceiverTrack::from_fixes(
                frame(),
                ReferencePointId::new(1),
                &repeated,
                DurationNs::from_ns(1_000_000_000)
            )
            .is_err()
        );
    }
}
