//! Physical attachment, installation, and reference-point validation.

use crate::error::ValidationError;
use crate::frame::{BodyLeverArm, ReferencePoint, ReferencePointKind, SensorToBodyRotation};
use crate::ids::{CalibrationRevision, ContentDigestV1, DynamicsProfileId, SharedParameterId};
use crate::uncertainty::{Covariance3, MeasurementUncertainty};

/// Maximum named physical points in one installation.
pub const MAX_REFERENCE_POINTS: usize = 32;

/// Whether fitted calibration means may change during processing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CalibrationPolicy {
    /// Keep supplied means fixed while propagating their joint uncertainty.
    Fixed,
    /// Estimate selected parameters under supplied priors; advanced graph only.
    RefineWithPriors,
}

/// Physical attachment semantics, independent of estimator dynamics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AttachmentModel {
    /// Sensor, antenna, and named output points form one rigid body.
    RigidBody,
    /// Only the instrument/antenna trajectory is physically claimed.
    DeviceTrajectoryOnly,
}

impl AttachmentModel {
    /// Returns whether this attachment may make an output claim at the named
    /// physical point. Device-only installations retain their sensor and
    /// antenna package trajectory, but never promote an arbitrary rigid point
    /// to a vehicle, athlete, or centre-of-mass trajectory.
    #[must_use]
    pub const fn permits_reference_point(self, kind: ReferencePointKind) -> bool {
        match self {
            Self::RigidBody => true,
            Self::DeviceTrajectoryOnly => !matches!(kind, ReferencePointKind::RigidBodyPoint),
        }
    }

    /// Returns whether forward/lateral/body-heading quantities may be claimed.
    #[must_use]
    pub const fn permits_body_axis_quantities(self) -> bool {
        matches!(self, Self::RigidBody)
    }
}

/// A rotation mean tied to one correlated shared parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotationParameter {
    /// Stable parameter identity.
    pub parameter_id: SharedParameterId,
    /// Mean sensor-to-body rotation.
    pub mean: SensorToBodyRotation,
    /// Small-angle covariance or configured uncertainty model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
}

/// A lever-arm mean tied to one correlated shared parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LeverArmParameter {
    /// Stable parameter identity.
    pub parameter_id: SharedParameterId,
    /// Mean body-frame lever arm in metres.
    pub mean: BodyLeverArm,
    /// Lever-arm covariance or configured uncertainty model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
}

/// Immutable physical installation truth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Installation<'a> {
    /// Coordinate frame carried by the prepared IMU vectors.
    pub imu_sensor_frame: crate::ids::FrameId,
    /// Rotation from the IMU measurement frame into forward-left-up body axes.
    pub body_from_imu: RotationParameter,
    /// IMU sensing-centre to GNSS antenna phase-centre lever arm.
    pub imu_to_gnss_antenna: LeverArmParameter,
    /// Named reference points; borrowed and allocation-free.
    pub reference_points: &'a [ReferencePoint],
    /// Physical attachment claim.
    pub attachment: AttachmentModel,
    /// Dynamics profile jointly qualified with this attachment.
    pub dynamics_profile: DynamicsProfileId,
    /// Residual installation-calibration revision.
    pub calibration_revision: CalibrationRevision,
    /// Canonical installation-definition digest.
    pub digest: ContentDigestV1,
}

impl Installation<'_> {
    /// Validates bounded uniqueness and required IMU/antenna points.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.imu_sensor_frame.get() == 0
            || self.body_from_imu.parameter_id.get() == 0
            || self.imu_to_gnss_antenna.parameter_id.get() == 0
            || self.dynamics_profile.get() == 0
            || self.calibration_revision.get() == 0
            || self.digest.is_zero()
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if self.reference_points.is_empty() || self.reference_points.len() > MAX_REFERENCE_POINTS {
            return Err(ValidationError::CapacityExceeded);
        }
        let mut imu_count = 0_u8;
        let mut antenna_count = 0_u8;
        let mut antenna_matches_declared_lever_arm = false;
        for (index, point) in self.reference_points.iter().enumerate() {
            if point.id().get() == 0
                || point.parameter_id().get() == 0
                || self
                    .reference_points
                    .iter()
                    .skip(index + 1)
                    .any(|other| other.id() == point.id())
            {
                return Err(ValidationError::InvalidReferencePoint);
            }
            match point.kind() {
                ReferencePointKind::ImuSensingCenter => {
                    imu_count = imu_count.saturating_add(1);
                    if point.imu_to_point().components_m() != [0.0; 3] {
                        return Err(ValidationError::InvalidReferencePoint);
                    }
                }
                ReferencePointKind::GnssAntennaPhaseCenter => {
                    antenna_count = antenna_count.saturating_add(1);
                    antenna_matches_declared_lever_arm |= point.parameter_id()
                        == self.imu_to_gnss_antenna.parameter_id
                        && point.imu_to_point() == self.imu_to_gnss_antenna.mean;
                }
                ReferencePointKind::RigidBodyPoint | ReferencePointKind::InstrumentPackage => {}
            }
        }
        if imu_count != 1 || antenna_count == 0 || !antenna_matches_declared_lever_arm {
            return Err(ValidationError::InvalidReferencePoint);
        }
        Ok(self)
    }
}
