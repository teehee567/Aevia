//! Immutable installation, calibration, profile, resource, and processing specs.

use crate::error::ValidationError;
use crate::frame::{CoordinateOperation, TerrestrialFrame};
use crate::ids::ContentDigestV1;
use crate::uncertainty::{Covariance3, MeasurementUncertainty};

mod calibration;
mod installation;
mod live;
mod numeric;
mod processing;
mod profiles;
mod qualification;
mod resources;

pub use calibration::{
    CalibrationBundle, MAX_UNCERTAINTY_MODELS, SharedParameterDefinition, SharedParameterKind,
    SharedParameterMean, SharedParameterSet, SharedUncertaintyTreatment,
    UncertaintyModelDefinition, UncertaintyModelKind,
};
pub use installation::{
    AttachmentModel, CalibrationPolicy, Installation, LeverArmParameter, MAX_REFERENCE_POINTS,
    RotationParameter,
};
pub use live::{
    ClockSharedCrossCovariance, InitialClockConsiderPrior, InitialHeading, LiveSpec,
    MAX_INITIAL_CLOCK_SHARED_DIMENSION,
};
pub use numeric::{
    ENCLOSURE_NATIVE_F64_ROOT_BACKEND_ID, ENCLOSURE_NATIVE_F64_ROOT_BACKEND_REVISION, FmaPolicy,
    LiveRootEnclosureQualificationV1, NATIVE_F64_TAYLOR_ROOT_BACKEND_ID,
    NATIVE_F64_TAYLOR_ROOT_BACKEND_REVISION, NumericProfileSpec, ScalarPolicy,
};
pub use processing::{
    CapturedReplayComparison, MAX_PROCESSING_PREFERENCE_LEVELS, ProcessingLevel, ProcessingPolicy,
    ProcessingPreference, ProcessingResultSpec, ProcessingSpec, RunControl,
    processing_span_contains,
};
pub use profiles::{
    CovarianceRepairPolicy, DynamicsProfileSpec, EmbeddedLiveTuning, GnssCorrelationPolicy,
    GnssFusionSpec, HeadingObservabilitySpec, InputProfileSpec, NavigationProfileSpec,
    ProcessNoiseSpec, StationaryClassifierSpec,
};
pub use qualification::{QualificationReportV1, QualificationSpecV1, QualificationStatus};
pub use resources::{LiveResourceLimits, OfflineResourceLimits};

/// Maximum recorded coordinate operations examined by embedded preflight.
pub const MAX_COORDINATE_OPERATIONS: usize = 32;

/// Complete immutable semantic configuration shared by all processing levels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineConfig<'a> {
    /// Physical installation and reference points.
    pub installation: Installation<'a>,
    /// Serial-bound residual calibration.
    pub calibration: CalibrationBundle<'a>,
    /// Prepared-input profile and engine capacity limits.
    pub input_profile: InputProfileSpec,
    /// Dynamics/measurement profile.
    pub dynamics_profile: DynamicsProfileSpec,
    /// Navigation math, delay, and repair profile.
    pub navigation_profile: NavigationProfileSpec,
    /// Scalar/toolchain behavior.
    pub numeric_profile: NumericProfileSpec,
    /// Session processing terrestrial frame.
    pub processing_frame: TerrestrialFrame,
    /// Recorded coordinate operations accepted by embedded inputs.
    pub coordinate_operations: &'a [CoordinateOperation],
    /// All configured uncertainty models referenced by observations.
    pub uncertainty_models: &'a [UncertaintyModelDefinition],
    /// Production qualification state for this combination.
    pub qualification: QualificationStatus<'a>,
    /// Canonical complete configuration digest.
    pub digest: ContentDigestV1,
}

impl EngineConfig<'_> {
    /// Validates cross-profile identity, capacities, and uncertainty references.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.digest.is_zero() || self.processing_frame.id().get() == 0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.installation.validate()?;
        self.calibration.validate()?;
        self.input_profile.validate()?;
        self.dynamics_profile.validate()?;
        self.navigation_profile.validate()?;
        self.numeric_profile.validate()?;
        self.qualification.validate_for_configuration(self.digest)?;
        self.qualification
            .validate_numeric_attestations(self.numeric_profile)?;
        if self.installation.calibration_revision != self.calibration.revision
            || self.installation.dynamics_profile != self.dynamics_profile.id
            || self.installation.attachment != self.dynamics_profile.attachment
            || self.calibration.input_profile != self.input_profile.id
            || usize::from(self.navigation_profile.consider_dimension)
                != self
                    .calibration
                    .shared_parameters
                    .scalar_dimension()
                    .saturating_add(2)
            || self.uncertainty_models.len() > MAX_UNCERTAINTY_MODELS
            || self.coordinate_operations.is_empty()
            || self.coordinate_operations.len() > MAX_COORDINATE_OPERATIONS
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        for (index, model) in self.uncertainty_models.iter().enumerate() {
            if model.id.get() == 0
                || model.digest.is_zero()
                || matches!(
                    model.kind,
                    UncertaintyModelKind::SequenceBound {
                        one_sigma,
                        qualification_digest
                    } if one_sigma.get() == 0.0 || qualification_digest.is_zero()
                )
                || self
                    .uncertainty_models
                    .iter()
                    .skip(index + 1)
                    .any(|other| other.id == model.id)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        for (index, operation) in self.coordinate_operations.iter().enumerate() {
            if self
                .coordinate_operations
                .iter()
                .skip(index + 1)
                .any(|other| other.id() == operation.id())
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
            if operation.target() != self.processing_frame.id()
                && operation.source() != self.processing_frame.id()
            {
                return Err(ValidationError::InvalidFrame);
            }
        }
        for required in [
            self.installation.body_from_imu.parameter_id,
            self.installation.imu_to_gnss_antenna.parameter_id,
        ] {
            if self
                .calibration
                .shared_parameters
                .definition(required)
                .is_none_or(|definition| definition.mean.dimension() != 3)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        for point in self.installation.reference_points {
            if self
                .calibration
                .shared_parameters
                .definition(point.parameter_id())
                .is_none_or(|definition| definition.mean.dimension() != 3)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
            self.require_uncertainty_model(point.uncertainty())?;
        }
        self.require_uncertainty_model(self.installation.body_from_imu.uncertainty)?;
        self.require_uncertainty_model(self.installation.imu_to_gnss_antenna.uncertainty)?;
        Ok(self)
    }

    /// Returns whether preflight may select this configuration for production.
    #[must_use]
    pub const fn is_qualified(self) -> bool {
        self.qualification.is_qualified()
    }

    /// Returns the exact measured live non-polynomial root attestation, if
    /// this qualified configuration carries one that matches its numeric
    /// profile and target report.
    pub(crate) fn live_root_enclosure_qualification(
        self,
    ) -> Option<LiveRootEnclosureQualificationV1> {
        self.qualification.live_root_enclosure(self.numeric_profile)
    }

    fn require_uncertainty_model(
        self,
        uncertainty: MeasurementUncertainty<Covariance3>,
    ) -> Result<(), ValidationError> {
        if let MeasurementUncertainty::Modeled(required) = uncertainty {
            if !self
                .uncertainty_models
                .iter()
                .any(|model| model.id == required)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
