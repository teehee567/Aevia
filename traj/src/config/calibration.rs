//! Shared calibration parameters and measurement uncertainty models.

use crate::error::ValidationError;
use crate::ids::{
    CalibrationRevision, ContentDigestV1, InputProfileId, SharedParameterId, UncertaintyModelId,
};
use crate::math::{FiniteF64, NonNegativeF64, Vector3};
use crate::time::TimeSpan;
use crate::uncertainty::{
    Covariance3, MAX_SHARED_PARAMETER_DIMENSION, SharedParameterCovariance, Variance,
};

/// Maximum uncertainty-model definitions in one embedded configuration.
pub const MAX_UNCERTAINTY_MODELS: usize = 64;

/// Semantic kind and unit of one shared uncertain parameter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SharedParameterKind {
    /// Fitted clock offset in nanoseconds.
    ClockOffsetNs,
    /// Fitted fractional clock drift.
    ClockDrift,
    /// Body-frame lever-arm coordinate in metres.
    LeverArmMetres,
    /// Boresight/small-angle coordinate in radians.
    BoresightRadians,
    /// Dimensionless sensor scale coefficient.
    Scale,
    /// Sensor-axis misalignment in radians.
    MisalignmentRadians,
    /// Gyro g-sensitivity coefficient.
    GyroGSensitivity,
    /// Filter/group delay in nanoseconds.
    DelayNs,
    /// Survey or coordinate-transform parameter in metres.
    SurveyMetres,
    /// Extensible profile-defined parameter kind.
    Other(u16),
}

/// Fixed supplied mean for a scalar or three-coordinate shared parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SharedParameterMean {
    /// One scalar coordinate.
    Scalar(FiniteF64),
    /// Three ordered coordinates sharing one stable semantic identity.
    Vector3(Vector3),
}

impl SharedParameterMean {
    /// Returns the number of scalar covariance coordinates occupied by the
    /// mean.
    #[must_use]
    pub const fn dimension(self) -> usize {
        match self {
            Self::Scalar(_) => 1,
            Self::Vector3(_) => 3,
        }
    }
}

/// Mean and validity of one member of a joint shared-parameter block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SharedParameterDefinition {
    /// Stable identity used by measurements and geometry.
    pub id: SharedParameterId,
    /// Semantic kind/unit.
    pub kind: SharedParameterKind,
    /// Fixed supplied mean.
    pub mean: SharedParameterMean,
    /// Session span over which the mean/covariance is valid.
    pub validity: TimeSpan,
}

/// Required treatment for repeated use of shared uncertain parameters.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SharedUncertaintyTreatment {
    /// Carry fixed-mean Schmidt/consider covariance and cross-covariance.
    SchmidtConsider,
    /// Apply a qualified conservative sequence-level bound.
    QualifiedSequenceBound {
        qualification_digest: ContentDigestV1,
    },
}

/// Ordered means and joint covariance for shared uncertain parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SharedParameterSet<'a> {
    /// Definitions in the exact covariance ordering.
    pub definitions: &'a [SharedParameterDefinition],
    /// Joint covariance in the same ordering.
    pub covariance: SharedParameterCovariance<'a>,
    /// Repeated-observation uncertainty treatment.
    pub treatment: SharedUncertaintyTreatment,
}

impl SharedParameterSet<'_> {
    /// Validates dimension and stable-ID uniqueness.
    pub fn validate(self) -> Result<Self, ValidationError> {
        let scalar_dimension = self
            .definitions
            .iter()
            .try_fold(0_usize, |total, definition| {
                total.checked_add(definition.mean.dimension())
            })
            .ok_or(ValidationError::CapacityExceeded)?;
        if scalar_dimension != self.covariance.dimension()
            || scalar_dimension > MAX_SHARED_PARAMETER_DIMENSION
        {
            return Err(ValidationError::InvalidCovariance);
        }
        if matches!(
            self.treatment,
            SharedUncertaintyTreatment::QualifiedSequenceBound {
                qualification_digest
            } if qualification_digest.is_zero()
        ) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        for (index, definition) in self.definitions.iter().enumerate() {
            if definition.id.get() == 0
                || self
                    .definitions
                    .iter()
                    .skip(index + 1)
                    .any(|other| other.id == definition.id)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        Ok(self)
    }

    /// Returns whether the set contains a stable parameter identity.
    #[must_use]
    pub fn contains(self, id: SharedParameterId) -> bool {
        self.definitions
            .iter()
            .any(|definition| definition.id == id)
    }

    /// Returns a definition by stable identity.
    #[must_use]
    pub fn definition(self, id: SharedParameterId) -> Option<SharedParameterDefinition> {
        self.definitions
            .iter()
            .copied()
            .find(|definition| definition.id == id)
    }

    /// Returns the scalar covariance dimension occupied by all definitions.
    #[must_use]
    pub fn scalar_dimension(self) -> usize {
        self.definitions
            .iter()
            .map(|definition| definition.mean.dimension())
            .sum()
    }
}

/// Immutable residual system-calibration bundle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibrationBundle<'a> {
    /// Revision selected by the installation.
    pub revision: CalibrationRevision,
    /// Prepared-input profile to which the residual calibration applies.
    pub input_profile: InputProfileId,
    /// Correlated fitted parameters and covariance.
    pub shared_parameters: SharedParameterSet<'a>,
    /// Canonical calibration content digest.
    pub digest: ContentDigestV1,
}

impl CalibrationBundle<'_> {
    /// Validates identity and shared calibration uncertainty.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.revision.get() == 0 || self.input_profile.get() == 0 || self.digest.is_zero() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.shared_parameters.validate()?;
        Ok(self)
    }
}

/// Configured uncertainty model available by stable ID.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UncertaintyModelKind {
    /// Constant three-axis covariance.
    ConstantCovariance3(Covariance3),
    /// Constant scalar variance.
    ConstantVariance(Variance),
    /// Sequence-level conservative bound qualified as one correlated process.
    SequenceBound {
        /// One-sigma bound in the parameter's declared unit.
        one_sigma: NonNegativeF64,
        /// Qualification evidence for span-wise coverage.
        qualification_digest: ContentDigestV1,
    },
}

/// Immutable configured uncertainty-model definition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UncertaintyModelDefinition {
    /// Stable model identity used by observations.
    pub id: UncertaintyModelId,
    /// Concrete model behavior.
    pub kind: UncertaintyModelKind,
    /// Canonical model-content digest.
    pub digest: ContentDigestV1,
}
