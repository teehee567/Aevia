//! Coordinate-frame, physical-reference-point, and framed-vector semantics.

use crate::{
    error::ValidationError,
    ids::{ContentDigestV1, CoordinateOperationId, FrameId, ReferencePointId, SharedParameterId},
    math::{NonNegativeF64, UnitQuaternion, Vector3},
    uncertainty::{Covariance3, MeasurementUncertainty},
};

macro_rules! framed_vector {
    ($name:ident, $documentation:literal) => {
        #[doc = $documentation]
        #[derive(Clone, Copy, Debug, PartialEq)]
        #[repr(transparent)]
        pub struct $name(Vector3);

        impl $name {
            /// Validates and constructs the framed vector.
            pub fn new(x: f64, y: f64, z: f64) -> Result<Self, ValidationError> {
                Ok(Self(Vector3::new(x, y, z)?))
            }

            /// Validates and constructs the framed vector from ordered
            /// components.
            pub fn from_components(components: [f64; 3]) -> Result<Self, ValidationError> {
                Ok(Self(Vector3::from_components(components)?))
            }

            /// Constructs the framed value from an already validated vector.
            #[must_use]
            pub const fn from_vector(vector: Vector3) -> Self {
                Self(vector)
            }

            /// Returns the ordered components.
            #[must_use]
            pub const fn components(self) -> [f64; 3] {
                self.0.components()
            }

            /// Returns the underlying validated mathematical vector.
            #[must_use]
            pub const fn as_vector(self) -> Vector3 {
                self.0
            }
        }
    };
}

framed_vector!(
    EcefPosition,
    "Position in metres in a named Earth-centred, Earth-fixed terrestrial frame."
);
framed_vector!(
    EcefVelocity,
    "Velocity in metres per second in a named Earth-centred, Earth-fixed frame."
);
framed_vector!(
    EcefAcceleration,
    "Kinematic acceleration in metres per second squared in an ECEF frame."
);
framed_vector!(
    SensorSpecificForce,
    "Calibrated specific force in metres per second squared in the sensor frame."
);
framed_vector!(
    SensorAngularRate,
    "Body-relative-inertial angular rate in radians per second in the sensor frame."
);
framed_vector!(
    BodyVector,
    "A vector expressed in the right-handed forward-left-up body frame."
);
framed_vector!(
    LocalEnuVector,
    "A vector expressed in a fixed-anchor right-handed east-north-up frame."
);
framed_vector!(
    BodyAngularRate,
    "Body-relative-Earth angular rate in radians per second in body coordinates."
);
framed_vector!(
    BodyAngularAcceleration,
    "Derivative of body-relative-Earth angular rate in body coordinates."
);

/// Rotation from the canonical forward-left-up body frame into ECEF.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct OrientationEcefFromBody(UnitQuaternion);

impl OrientationEcefFromBody {
    /// Constructs an orientation from a validated unit quaternion.
    #[must_use]
    pub const fn from_quaternion(quaternion: UnitQuaternion) -> Self {
        Self(quaternion)
    }

    /// Returns the underlying scalar-first unit quaternion.
    #[must_use]
    pub const fn quaternion(self) -> UnitQuaternion {
        self.0
    }

    /// Rotates a body-frame vector into ECEF coordinates.
    #[must_use]
    pub fn rotate_body_vector(self, vector: BodyVector) -> Vector3 {
        self.0.rotate_vector(vector.as_vector())
    }
}

/// Rotation from one physical sensor's axes into the canonical body frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct SensorToBodyRotation(UnitQuaternion);

impl SensorToBodyRotation {
    /// Constructs the transform from a validated unit quaternion.
    #[must_use]
    pub const fn from_quaternion(quaternion: UnitQuaternion) -> Self {
        Self(quaternion)
    }

    /// Returns the underlying scalar-first unit quaternion.
    #[must_use]
    pub const fn quaternion(self) -> UnitQuaternion {
        self.0
    }

    /// Rotates a vector from sensor axes into canonical body axes.
    #[must_use]
    pub fn rotate_sensor_vector(self, vector: Vector3) -> BodyVector {
        BodyVector::from_vector(self.0.rotate_vector(vector))
    }
}

/// A surveyed displacement in body coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct BodyLeverArm(BodyVector);

impl BodyLeverArm {
    /// Validates a body-frame lever arm in metres.
    pub fn new(x_m: f64, y_m: f64, z_m: f64) -> Result<Self, ValidationError> {
        Ok(Self(BodyVector::new(x_m, y_m, z_m)?))
    }

    /// Constructs a lever arm from a validated body vector.
    #[must_use]
    pub const fn from_body_vector(vector: BodyVector) -> Self {
        Self(vector)
    }

    /// Returns body-frame metre components.
    #[must_use]
    pub const fn components_m(self) -> [f64; 3] {
        self.0.components()
    }

    /// Returns the lever arm as a body-frame vector.
    #[must_use]
    pub const fn as_body_vector(self) -> BodyVector {
        self.0
    }
}

/// A specific WGS-84 terrestrial-frame realization.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Wgs84Realization {
    /// GPS week 730 realization.
    G730,
    /// GPS week 873 realization.
    G873,
    /// GPS week 1150 realization.
    G1150,
    /// GPS week 1674 realization.
    G1674,
    /// GPS week 1762 realization.
    G1762,
    /// GPS week 2139 realization.
    G2139,
    /// GPS week 2296 realization.
    G2296,
    /// A future realization identified by its GPS-week tag.
    Other { gps_week_tag: u16 },
}

/// Identity of a terrestrial datum/frame realization.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TerrestrialRealization {
    /// A named International Terrestrial Reference Frame realization.
    Itrf { realization_year: u16 },
    /// A named WGS-84 realization.
    Wgs84(Wgs84Realization),
    /// The metre-level WGS-84 ensemble, which is not sufficient for surveyed
    /// centimetre comparisons.
    Wgs84Ensemble,
    /// An extensible authority/code pair, such as an EPSG datum code.
    AuthorityCode { authority: u16, code: u32 },
}

impl TerrestrialRealization {
    /// Returns whether the realization is specific enough to participate in
    /// timing-grade surveyed geometry after a valid coordinate operation.
    #[must_use]
    pub const fn is_specific(self) -> bool {
        !matches!(self, Self::Wgs84Ensemble)
    }
}

/// Coordinate epoch represented as a finite decimal year.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct CoordinateEpoch(f64);

impl CoordinateEpoch {
    /// Validates a positive finite decimal year.
    pub fn from_decimal_year(value: f64) -> Result<Self, ValidationError> {
        if !value.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        if value <= 0.0 {
            return Err(ValidationError::InvalidFrame);
        }
        Ok(Self(value))
    }

    /// Returns the decimal-year representation.
    #[must_use]
    pub const fn decimal_year(self) -> f64 {
        self.0
    }
}

/// Reference ellipsoid used for local-up and geodetic tangent semantics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceEllipsoid {
    semi_major_axis_m: f64,
    inverse_flattening: f64,
}

impl ReferenceEllipsoid {
    /// WGS-84 ellipsoid parameters.
    pub const WGS84: Self = Self {
        semi_major_axis_m: 6_378_137.0,
        inverse_flattening: 298.257_223_563,
    };

    /// Validates a rotational ellipsoid.
    pub fn new(semi_major_axis_m: f64, inverse_flattening: f64) -> Result<Self, ValidationError> {
        if !semi_major_axis_m.is_finite() || !inverse_flattening.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        if semi_major_axis_m <= 0.0 || inverse_flattening <= 1.0 {
            return Err(ValidationError::InvalidFrame);
        }
        Ok(Self {
            semi_major_axis_m,
            inverse_flattening,
        })
    }

    /// Returns the semi-major axis in metres.
    #[must_use]
    pub const fn semi_major_axis_m(self) -> f64 {
        self.semi_major_axis_m
    }

    /// Returns inverse flattening.
    #[must_use]
    pub const fn inverse_flattening(self) -> f64 {
        self.inverse_flattening
    }
}

/// Complete terrestrial-frame identity required by navigation observations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerrestrialFrame {
    id: FrameId,
    realization: TerrestrialRealization,
    coordinate_epoch: CoordinateEpoch,
    ellipsoid: ReferenceEllipsoid,
}

impl TerrestrialFrame {
    /// Constructs a named terrestrial frame and coordinate epoch.
    #[must_use]
    pub const fn new(
        id: FrameId,
        realization: TerrestrialRealization,
        coordinate_epoch: CoordinateEpoch,
        ellipsoid: ReferenceEllipsoid,
    ) -> Self {
        Self {
            id,
            realization,
            coordinate_epoch,
            ellipsoid,
        }
    }

    /// Returns the stable frame ID.
    #[must_use]
    pub const fn id(self) -> FrameId {
        self.id
    }

    /// Returns the terrestrial realization.
    #[must_use]
    pub const fn realization(self) -> TerrestrialRealization {
        self.realization
    }

    /// Returns the coordinate epoch.
    #[must_use]
    pub const fn coordinate_epoch(self) -> CoordinateEpoch {
        self.coordinate_epoch
    }

    /// Returns the reference ellipsoid.
    #[must_use]
    pub const fn ellipsoid(self) -> ReferenceEllipsoid {
        self.ellipsoid
    }

    /// Returns whether the frame is specific enough for timing-grade surveyed
    /// geometry.
    #[must_use]
    pub const fn supports_surveyed_geometry(self) -> bool {
        self.realization.is_specific()
    }
}

/// Physical coordinate frame requested by a trajectory query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OutputFrame {
    /// ECEF coordinates in the named frame.
    Ecef(FrameId),
    /// Fixed-anchor ENU coordinates in the named local frame.
    LocalEnu(FrameId),
    /// Canonical right-handed forward-left-up body axes.
    Body,
    /// Original sensor axes.
    Sensor(FrameId),
}

/// Mathematical class of a recorded coordinate operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinateOperationKind {
    /// No coordinate change; source and target definitions are identical.
    Identity,
    /// Static Helmert-style transformation.
    Helmert,
    /// Time-dependent transformation evaluated at the coordinate epoch.
    TimeDependent,
    /// Topocentric or projected conversion.
    Projection,
    /// Extensible versioned operation kind.
    Other(u16),
}

/// Compact, auditable metadata for a coordinate operation performed upstream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateOperation {
    id: CoordinateOperationId,
    source: FrameId,
    target: FrameId,
    kind: CoordinateOperationKind,
    pipeline_digest: ContentDigestV1,
    grid_digest: Option<ContentDigestV1>,
    one_sigma_accuracy_m: Option<NonNegativeF64>,
    ballpark: bool,
}

impl CoordinateOperation {
    /// Validates and constructs operation metadata.
    pub fn new(
        id: CoordinateOperationId,
        source: FrameId,
        target: FrameId,
        kind: CoordinateOperationKind,
        pipeline_digest: ContentDigestV1,
        grid_digest: Option<ContentDigestV1>,
        one_sigma_accuracy_m: Option<NonNegativeF64>,
        ballpark: bool,
    ) -> Result<Self, ValidationError> {
        let identity = matches!(kind, CoordinateOperationKind::Identity);
        if id.get() == 0
            || source.get() == 0
            || target.get() == 0
            || pipeline_digest.is_zero()
            || grid_digest.is_some_and(ContentDigestV1::is_zero)
            || identity != (source == target)
        {
            return Err(ValidationError::InvalidFrame);
        }
        Ok(Self {
            id,
            source,
            target,
            kind,
            pipeline_digest,
            grid_digest,
            one_sigma_accuracy_m,
            ballpark,
        })
    }

    /// Returns the operation ID.
    #[must_use]
    pub const fn id(self) -> CoordinateOperationId {
        self.id
    }

    /// Returns the source frame ID.
    #[must_use]
    pub const fn source(self) -> FrameId {
        self.source
    }

    /// Returns the target frame ID.
    #[must_use]
    pub const fn target(self) -> FrameId {
        self.target
    }

    /// Returns the operation kind.
    #[must_use]
    pub const fn kind(self) -> CoordinateOperationKind {
        self.kind
    }

    /// Returns the canonical pipeline digest.
    #[must_use]
    pub const fn pipeline_digest(self) -> ContentDigestV1 {
        self.pipeline_digest
    }

    /// Returns the optional digest covering transformation grids.
    #[must_use]
    pub const fn grid_digest(self) -> Option<ContentDigestV1> {
        self.grid_digest
    }

    /// Returns the declared one-sigma spatial accuracy in metres.
    #[must_use]
    pub const fn one_sigma_accuracy_m(self) -> Option<NonNegativeF64> {
        self.one_sigma_accuracy_m
    }

    /// Returns whether this is an approximate or ballpark transformation.
    #[must_use]
    pub const fn is_ballpark(self) -> bool {
        self.ballpark
    }

    /// Returns whether this operation may support a requested surveyed-geometry
    /// accuracy.
    #[must_use]
    pub fn supports_surveyed_accuracy(self, maximum_one_sigma_m: f64) -> bool {
        if self.ballpark || !maximum_one_sigma_m.is_finite() || maximum_one_sigma_m < 0.0 {
            return false;
        }
        self.one_sigma_accuracy_m
            .is_some_and(|accuracy| accuracy.get() <= maximum_one_sigma_m)
    }
}

/// Physical meaning of a named rigid-body reference point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReferencePointKind {
    /// IMU sensing centre, the navigation-state origin.
    ImuSensingCenter,
    /// GNSS antenna phase centre.
    GnssAntennaPhaseCenter,
    /// Another surveyed rigid point on the device or vehicle.
    RigidBodyPoint,
    /// Instrument package point whose relationship to a human centre of mass
    /// is not claimed.
    InstrumentPackage,
}

/// A named point rigidly offset from the IMU sensing centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferencePoint {
    id: ReferencePointId,
    kind: ReferencePointKind,
    imu_to_point: BodyLeverArm,
    parameter_id: SharedParameterId,
    uncertainty: MeasurementUncertainty<Covariance3>,
}

impl ReferencePoint {
    /// Constructs a reference-point definition.
    #[must_use]
    pub const fn new(
        id: ReferencePointId,
        kind: ReferencePointKind,
        imu_to_point: BodyLeverArm,
        parameter_id: SharedParameterId,
        uncertainty: MeasurementUncertainty<Covariance3>,
    ) -> Self {
        Self {
            id,
            kind,
            imu_to_point,
            parameter_id,
            uncertainty,
        }
    }

    /// Returns the stable point ID.
    #[must_use]
    pub const fn id(self) -> ReferencePointId {
        self.id
    }

    /// Returns the physical point kind.
    #[must_use]
    pub const fn kind(self) -> ReferencePointKind {
        self.kind
    }

    /// Returns the body-frame offset from the IMU centre.
    #[must_use]
    pub const fn imu_to_point(self) -> BodyLeverArm {
        self.imu_to_point
    }

    /// Returns the shared-parameter identity for the surveyed offset.
    #[must_use]
    pub const fn parameter_id(self) -> SharedParameterId {
        self.parameter_id
    }

    /// Returns the supplied or modeled lever-arm uncertainty.
    #[must_use]
    pub const fn uncertainty(self) -> MeasurementUncertainty<Covariance3> {
        self.uncertainty
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::UncertaintyModelId;

    #[test]
    fn framed_vectors_reject_non_finite_components() {
        assert_eq!(
            EcefPosition::new(1.0, f64::NAN, 3.0),
            Err(ValidationError::NonFinite)
        );
        assert_eq!(
            EcefVelocity::from_components([1.0, 2.0, 3.0])
                .unwrap()
                .components(),
            [1.0, 2.0, 3.0]
        );
    }

    #[test]
    fn ensemble_is_explicitly_not_survey_grade() {
        let epoch = CoordinateEpoch::from_decimal_year(2026.5).unwrap();
        let frame = TerrestrialFrame::new(
            FrameId::new(1),
            TerrestrialRealization::Wgs84Ensemble,
            epoch,
            ReferenceEllipsoid::WGS84,
        );
        assert!(!frame.supports_surveyed_geometry());
    }

    #[test]
    fn coordinate_operation_enforces_identity_semantics() {
        let result = CoordinateOperation::new(
            CoordinateOperationId::new(1),
            FrameId::new(1),
            FrameId::new(2),
            CoordinateOperationKind::Identity,
            ContentDigestV1::from_bytes([0; 32]),
            None,
            None,
            false,
        );
        assert_eq!(result, Err(ValidationError::InvalidFrame));
    }

    #[test]
    fn coordinate_operation_rejects_placeholder_identity() {
        let digest = ContentDigestV1::from_bytes([1; 32]);
        for result in [
            CoordinateOperation::new(
                CoordinateOperationId::new(0),
                FrameId::new(1),
                FrameId::new(1),
                CoordinateOperationKind::Identity,
                digest,
                None,
                None,
                false,
            ),
            CoordinateOperation::new(
                CoordinateOperationId::new(1),
                FrameId::new(0),
                FrameId::new(0),
                CoordinateOperationKind::Identity,
                digest,
                None,
                None,
                false,
            ),
            CoordinateOperation::new(
                CoordinateOperationId::new(1),
                FrameId::new(1),
                FrameId::new(1),
                CoordinateOperationKind::Identity,
                ContentDigestV1::from_bytes([0; 32]),
                None,
                None,
                false,
            ),
        ] {
            assert_eq!(result, Err(ValidationError::InvalidFrame));
        }
    }

    #[test]
    fn ballpark_operation_never_supports_surveyed_accuracy() {
        let operation = CoordinateOperation::new(
            CoordinateOperationId::new(1),
            FrameId::new(1),
            FrameId::new(2),
            CoordinateOperationKind::Helmert,
            ContentDigestV1::from_bytes([1; 32]),
            Some(ContentDigestV1::from_bytes([2; 32])),
            Some(NonNegativeF64::new(0.001).unwrap()),
            true,
        )
        .unwrap();
        assert!(!operation.supports_surveyed_accuracy(0.01));
    }

    #[test]
    fn reference_point_retains_explicit_modeled_uncertainty() {
        let point = ReferencePoint::new(
            ReferencePointId::new(7),
            ReferencePointKind::InstrumentPackage,
            BodyLeverArm::new(0.1, 0.0, 0.0).unwrap(),
            SharedParameterId::new(9),
            MeasurementUncertainty::Modeled(UncertaintyModelId::new(3)),
        );
        assert_eq!(
            point.uncertainty(),
            MeasurementUncertainty::Modeled(UncertaintyModelId::new(3))
        );
    }
}
