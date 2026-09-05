//! Shared metric test fixtures.

use super::super::{
    definition::{CrossingDirection, FiniteGate},
    numerical::MetricEvaluationLimits,
};
use crate::{
    config::AttachmentModel,
    frame::{
        BodyLeverArm, BodyVector, CoordinateEpoch, EcefPosition, EcefVelocity,
        OrientationEcefFromBody, ReferenceEllipsoid, ReferencePoint, ReferencePointKind,
        TerrestrialFrame, TerrestrialRealization, Wgs84Realization,
    },
    ids::{FrameId, GateId, ReferencePointId, SharedParameterId, TrajectoryRevision},
    math::UnitQuaternion,
    quality::{
        CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
        HeadingSource, Integrity, ObservabilityReport, TimingQuality, Validity,
    },
    time::{DurationNs, SessionTime},
    trajectory::{Trajectory, TrajectoryKnot},
    uncertainty::{Covariance3, KinematicCovariance, MeasurementUncertainty},
};

pub(super) fn limits() -> MetricEvaluationLimits {
    MetricEvaluationLimits {
        absolute_root_tolerance_s: 1.0e-12,
        value_tolerance: 1.0e-12,
        absolute_integration_tolerance: 1.0e-12,
        relative_integration_tolerance: 1.0e-12,
        maximum_root_evaluations: 1_024,
        maximum_quadrature_evaluations: 32_768,
    }
}

pub(super) fn with_large_test_stack(test: fn()) {
    std::thread::Builder::new()
        .name("metric-numerical-test".into())
        .stack_size(16 * 1_024 * 1_024)
        .spawn(test)
        .unwrap()
        .join()
        .unwrap();
}

pub(super) fn test_quality() -> EstimateQuality {
    EstimateQuality {
        stage: EstimateStage::Finalized,
        validity: Validity::Nominal,
        gnss: GnssState::Fixed,
        timing: TimingQuality::PpsCorrelated,
        integrity: Integrity::Monitored,
        covariance: CovarianceConditioning::UnconditionalModel,
        imu_gap: false,
        degraded_input: false,
    }
}

pub(super) fn test_observability() -> ObservabilityReport {
    ObservabilityReport {
        heading_source: HeadingSource::Supplied,
        heading: HeadingObservability::Supplied,
        heading_variance_rad2: Some(0.01),
        course_available: true,
        body_axis_quantities_available: true,
        angular_acceleration_available: true,
    }
}

pub(super) fn test_covariance() -> KinematicCovariance {
    KinematicCovariance::new(
        Covariance3::diagonal(0.01, 0.01, 0.01).unwrap(),
        Covariance3::diagonal(0.01, 0.01, 0.01).unwrap(),
        None,
        Covariance3::diagonal(0.001, 0.001, 0.001).unwrap(),
    )
    .unwrap()
}

pub(super) fn test_trajectory_with_attachment(
    attachment: AttachmentModel,
    point_kind: ReferencePointKind,
) -> Trajectory {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(1));
    trajectory.set_attachment_model(attachment).unwrap();
    trajectory
        .add_reference_point(ReferencePoint::new(
            ReferencePointId::new(1),
            point_kind,
            BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
            SharedParameterId::new(1),
            MeasurementUncertainty::Provided(Covariance3::ZERO),
        ))
        .unwrap();
    trajectory
}

pub(super) fn test_trajectory() -> Trajectory {
    test_trajectory_with_attachment(
        AttachmentModel::RigidBody,
        ReferencePointKind::ImuSensingCenter,
    )
}

pub(super) fn eastbound_knot(time_ns: i64, east_m: f64, speed_mps: f64) -> TrajectoryKnot {
    TrajectoryKnot {
        time: SessionTime::from_ns(time_ns),
        position_ecef: EcefPosition::new(6_378_137.0, east_m, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, speed_mps, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: test_covariance(),
        quality: test_quality(),
        observability: test_observability(),
    }
}

pub(super) fn eastbound_trajectory_between(
    start_ns: i64,
    end_ns: i64,
    start_east_m: f64,
    end_east_m: f64,
    start_speed: f64,
    end_speed: f64,
) -> Trajectory {
    let mut trajectory = test_trajectory();
    let start = eastbound_knot(start_ns, start_east_m, start_speed);
    let end = eastbound_knot(end_ns, end_east_m, end_speed);
    trajectory.push_hermite_segment(start, end).unwrap();
    trajectory
}

pub(super) fn eastbound_trajectory(
    start_speed: f64,
    end_speed: f64,
    distance_m: f64,
) -> Trajectory {
    eastbound_trajectory_between(0, 1_000_000_000, 0.0, distance_m, start_speed, end_speed)
}

pub(super) fn gap_bridged_eastbound_trajectory() -> Trajectory {
    let mut trajectory = test_trajectory();
    // Non-constant endpoint speeds avoid an intentionally ambiguous
    // all-times-equal peak while preserving a simple monotonic path.
    let mut start = eastbound_knot(0, 0.0, 9.0);
    let mut end = eastbound_knot(1_000_000_000, 10.0, 11.0);
    for knot in [&mut start, &mut end] {
        knot.quality.validity = Validity::Degraded;
        knot.quality.imu_gap = true;
    }
    trajectory.push_hermite_segment(start, end).unwrap();
    trajectory
}

pub(super) fn eastbound_trajectory_with_internal_gap() -> Trajectory {
    let mut trajectory = test_trajectory();
    trajectory
        .push_hermite_segment(
            eastbound_knot(0, 0.0, 9.0),
            eastbound_knot(1_000_000_000, 10.0, 10.0),
        )
        .unwrap();
    let mut gap_start = eastbound_knot(1_000_000_000, 10.0, 10.0);
    let mut gap_end = eastbound_knot(2_000_000_000, 20.0, 11.0);
    for knot in [&mut gap_start, &mut gap_end] {
        knot.quality.validity = Validity::Degraded;
        knot.quality.imu_gap = true;
    }
    trajectory.push_hermite_segment(gap_start, gap_end).unwrap();
    trajectory
        .push_hermite_segment(
            eastbound_knot(2_000_000_000, 20.0, 11.0),
            eastbound_knot(3_000_000_000, 30.0, 12.0),
        )
        .unwrap();
    trajectory
}

pub(super) fn east_gate(id: u32, east_m: f64, rearm_distance_m: f64) -> FiniteGate {
    FiniteGate::new(
        GateId::new(id),
        FrameId::new(1),
        [6_378_137.0, east_m, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        20.0,
        20.0,
        CrossingDirection::Either,
        0.1,
        rearm_distance_m,
        DurationNs::ZERO,
        None,
    )
    .unwrap()
}
