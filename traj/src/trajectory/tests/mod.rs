use super::*;
use crate::{
    config::AttachmentModel,
    frame::{
        BodyLeverArm, CoordinateEpoch, ReferencePointKind, TerrestrialRealization, Wgs84Realization,
    },
    ids::{FrameId, SharedParameterId},
    uncertainty::MeasurementUncertainty,
};
#[cfg(feature = "offline")]
use crate::{
    ids::MetricDefinitionId,
    metric::{DistancePlan, DistanceQuantity, MetricDefinition},
};

#[cfg(feature = "offline")]
use super::bridge::*;
use super::dense::*;
use super::math::*;
use super::roots::*;
use crate::error::{QueryError, ValidationError};
use crate::frame::{
    BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody, ReferenceEllipsoid,
    ReferencePoint, TerrestrialFrame,
};
use crate::ids::{ReferencePointId, TrajectoryRevision};
use crate::math::{UnitQuaternion, Vector3};
use crate::metric::MetricError;
#[cfg(feature = "offline")]
use crate::metric::MetricPlan;
#[cfg(feature = "offline")]
use crate::offline::{FIXED_RECORD_HEADER_BYTES, FixedRecordStoreKind};
use crate::quality::{
    CovarianceConditioning, EstimateQuality, EstimateStage, FieldValue, GnssState,
    HeadingObservability, HeadingSource, Integrity, ObservabilityReport, TimingQuality,
    UnavailableReason, Validity,
};
use crate::time::SessionTime;
use crate::uncertainty::{Covariance3, KinematicCovariance};
#[cfg(feature = "offline")]
use nalgebra::DMatrix;
#[cfg(feature = "offline")]
use std::boxed::Box;

#[cfg(feature = "offline")]
mod coupled_storage;
mod kinematics;
#[cfg(feature = "offline")]
mod offline;
mod roots;
mod storage;

fn quality() -> EstimateQuality {
    EstimateQuality {
        stage: EstimateStage::Finalized,
        validity: Validity::Nominal,
        gnss: GnssState::Healthy,
        timing: TimingQuality::PpsCorrelated,
        integrity: Integrity::Monitored,
        covariance: CovarianceConditioning::UnconditionalModel,
        imu_gap: false,
        degraded_input: false,
    }
}

fn observability() -> ObservabilityReport {
    ObservabilityReport {
        heading_source: HeadingSource::Supplied,
        heading: HeadingObservability::Supplied,
        heading_variance_rad2: Some(0.01),
        course_available: true,
        body_axis_quantities_available: true,
        angular_acceleration_available: true,
    }
}

fn covariance() -> KinematicCovariance {
    KinematicCovariance::new(
        Covariance3::diagonal(1.0, 1.0, 1.0).unwrap(),
        Covariance3::diagonal(0.1, 0.1, 0.1).unwrap(),
        None,
        Covariance3::diagonal(0.01, 0.01, 0.01).unwrap(),
    )
    .unwrap()
}

fn reference(id: u32, lever: [f64; 3]) -> ReferencePoint {
    ReferencePoint::new(
        ReferencePointId::new(id),
        ReferencePointKind::RigidBodyPoint,
        BodyLeverArm::new(lever[0], lever[1], lever[2]).unwrap(),
        SharedParameterId::new(id),
        MeasurementUncertainty::Provided(Covariance3::ZERO),
    )
}

fn trajectory() -> Trajectory {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(1));
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(1, [0.0; 3]))
        .unwrap();
    trajectory
        .add_reference_point(reference(2, [1.0, 0.0, 0.0]))
        .unwrap();
    let start = TrajectoryKnot {
        time: SessionTime::from_ns(0),
        position_ecef: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: covariance(),
        quality: quality(),
        observability: observability(),
    };
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 10.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        ..start
    };
    trajectory.push_hermite_segment(start, end).unwrap();
    trajectory
}

#[cfg(feature = "offline")]
fn offline_conditional_fixture(
    kind: FixedRecordStoreKind,
) -> (Trajectory, TrajectoryKnot, TrajectoryKnot) {
    offline_bridge_fixture(kind, true)
}

#[cfg(feature = "offline")]
fn offline_bridge_fixture(
    kind: FixedRecordStoreKind,
    covariance_available: bool,
) -> (Trajectory, TrajectoryKnot, TrajectoryKnot) {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let start = TrajectoryKnot {
        time: SessionTime::ZERO,
        position_ecef: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: covariance(),
        quality: quality(),
        observability: observability(),
    };
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 10.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        ..start
    };
    let mut joint = Box::new([[0.0; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION]);
    for axis in 0..BRIDGE_KINEMATIC_DIMENSION {
        let variance = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01][axis];
        joint[axis][axis] = variance;
        joint[axis + BRIDGE_KINEMATIC_DIMENSION][axis + BRIDGE_KINEMATIC_DIMENSION] = variance;
        joint[axis][axis + BRIDGE_KINEMATIC_DIMENSION] = 0.5 * variance;
        joint[axis + BRIDGE_KINEMATIC_DIMENSION][axis] = 0.5 * variance;
    }
    let input = DenseBridgeInput {
        coupled: None,
        covariance_available,
        endpoint_joint_covariance: joint,
        acceleration_spectral_density_ecef: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
        attitude_spectral_density_body: [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]],
        acceleration_interval_average_covariance_ecef: [[0.0; 3]; 3],
        angular_rate_interval_average_covariance_body: [[0.0; 3]; 3],
        reintegrated_position_ecef_m: [6_378_137.0, 10.0, 0.0],
        // The fallback also accepts rotating-force reintegration whose
        // endpoint velocity is not implied by constant acceleration.
        reintegrated_velocity_ecef_mps: if covariance_available {
            [0.0, 10.0, 0.0]
        } else {
            [1.0, 12.0, -0.5]
        },
        integrated_rotation_body: [0.01, 0.02, 0.03],
    };
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(11));
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(1, [0.0; 3]))
        .unwrap();
    trajectory.prepare_offline_storage(1, kind).unwrap();
    trajectory
        .push_offline_conditional_bridge_segment(start, end, input, (7, 8))
        .unwrap();
    trajectory.finish_offline_storage().unwrap();
    (trajectory, start, end)
}

fn quadratic_oracle(
    first_root: f64,
    second_root: f64,
) -> impl Fn(f64, f64) -> Result<ScalarEnclosure, MetricError> {
    move |lower, upper| {
        let x = midpoint(lower, upper);
        taylor_enclosure(
            ScalarJet {
                value: (x - first_root) * (x - second_root),
                derivative: 2.0 * x - first_root - second_root,
                second_derivative: 2.0,
                value_roundoff: 0.0,
                derivative_roundoff: 0.0,
                second_derivative_roundoff: 0.0,
            },
            2.0,
            lower,
            upper,
        )
    }
}
