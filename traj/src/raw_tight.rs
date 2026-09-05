//! Private boundary and safety state machines for raw tightly coupled RTK/INS.
//!
//! The Rust code here performs evidence, resource, lineage, ambiguity-use, and
//! conditional-fix validation.  It does not contain a hidden approximation of
//! a tightly coupled solver.  A raw-tight run can start only if a separately
//! reviewed backend registration is linked into this module.  No registration
//! is linked in this release, so the compiled capability remains fail-closed.

#![allow(
    dead_code,
    reason = "dormant private boundary until qualified backend linkage exists"
)]

use crate::ids::{BackendVersionId, ContentDigestV1, QualificationSpecId};

mod ambiguity;
mod fix;
mod phase_ledger;
mod preflight;

#[allow(
    unused_imports,
    reason = "preserve the dormant raw-tight boundary paths"
)]
pub(crate) use preflight::{
    PreparedRawTightBackend, RawTightCandidateDiagnostic, RawTightEvidenceChecks,
    RawTightEvidenceRequirement, RawTightPassLimits, RawTightPreflightError,
    RawTightPreflightFailure, RawTightPreflightRequest, RawTightProblemShape,
    RawTightResourceEstimate, RawTightResourceKind, preflight,
};

#[allow(
    unused_imports,
    reason = "preserve the dormant raw-tight boundary paths"
)]
pub(crate) use ambiguity::{
    AmbiguityArcError, AmbiguityArcId, AmbiguityArcState, AmbiguityArcTransition,
    AmbiguityContinuityEvent, AmbiguityFixState, AmbiguitySignalKey, ArcTerminationReason,
    ContinuityIndicator, CycleSlipEvidence, ReferenceCovarianceAction,
};

#[allow(
    unused_imports,
    reason = "preserve the dormant raw-tight boundary paths"
)]
pub(crate) use phase_ledger::{PhaseContribution, PhaseSampleKey, PhaseUseError, PhaseUseLedger};

#[allow(
    unused_imports,
    reason = "preserve the dormant raw-tight boundary paths"
)]
pub(crate) use fix::{
    AmbiguityFixDecision, AmbiguityFixEvidence, AmbiguityFixThresholds,
    ConditionalUncertaintyBasis, FixAssessmentError, FixRejectionReason, assess_ambiguity_fix,
};

#[cfg(test)]
mod tests;

pub(crate) const RAW_TIGHT_ADAPTER_ABI_REVISION: u32 = 1;

/// Sparse graph implementation hidden by the raw-tight adapter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RawGraphBackendKind {
    IndependentNative,
    GtsamSystem,
    GtsamVendored,
}

/// Backend implementation/build identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightImplementationIdentity {
    adapter_abi_revision: u32,
    algorithm_revision: u32,
    implementation_digest: ContentDigestV1,
    native_build_digest: ContentDigestV1,
    graph_backend: RawGraphBackendKind,
    /// Mandatory for either GTSAM mode and forbidden for an independent
    /// implementation. It is the reviewed GTSAM source digest, not a version
    /// string.
    gtsam_source_digest: Option<ContentDigestV1>,
}

/// One auditable invariant implemented by the registered backend.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
enum RawTightSafetyProperty {
    PerConstellationAndFrequencyModels = 0,
    ReceiverClockAndHardwareBiasStates = 1,
    SatellitePropagationAndSagnac = 2,
    AntennaPhaseAndWindupCorrections = 3,
    OrbitClockIssueValidityAndAgePolicy = 4,
    ElevationAndCn0Weighting = 5,
    FullDoubleDifferenceCovariance = 6,
    DopplerClockDriftState = 7,
    ReferenceSwitchCovarianceReparameterization = 8,
    PhysicalAmbiguityArcContinuity = 9,
    PhaseTdcpJointCovarianceOrExclusion = 10,
    PvtInitializationOnly = 11,
    IntegerResolutionOutsideFloatGraph = 12,
    RatioSuccessResidualTemporalValidation = 13,
    PartialFixSupported = 14,
    ImmediateFixResetOnIntegrityLoss = 15,
    SatelliteExclusionAndReinstatement = 16,
    ConditionalFixUncertaintyLabeling = 17,
}

/// Compact, exact set of raw-tight safety claims.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub(crate) struct RawTightSafetyClaims(u32);

impl RawTightSafetyClaims {
    const ALL: Self = Self((1_u32 << 18) - 1);

    const fn all_required(self) -> bool {
        self.0 == Self::ALL.0
    }

    #[cfg(test)]
    const fn without(self, property: RawTightSafetyProperty) -> Self {
        Self(self.0 & !(1_u32 << property as u8))
    }
}

/// Frozen raw-tight qualification evidence. Counts are intentionally explicit:
/// a digest without exercised slip/reference/fix cases is insufficient.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightQualificationReceipt {
    specification: QualificationSpecId,
    specification_digest: ContentDigestV1,
    report_digest: ContentDigestV1,
    cycle_slip_cases: u32,
    reference_switch_cases: u32,
    rejected_incorrect_fix_cases: u32,
    covariance_coverage_sessions: u16,
    passed: bool,
}

/// Explicit backend registration. This is metadata only; native graph and FFI
/// types never cross into the engine.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightBackendRegistration {
    backend_version: BackendVersionId,
    identity: RawTightImplementationIdentity,
    qualification: RawTightQualificationReceipt,
    safety: RawTightSafetyClaims,
    maximum_signal_samples: u64,
    maximum_simultaneous_ambiguity_arcs: u32,
    maximum_integer_candidates: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RawTightRegistrationError {
    InvalidBackendVersion,
    AdapterAbiMismatch { expected: u32, registered: u32 },
    InvalidAlgorithmRevision,
    MissingImplementationIdentity,
    MissingGtsamSourceDigest,
    UnexpectedGtsamSourceDigest,
    GraphBackendNotCompiled,
    QualificationFailed,
    IncompleteQualificationEvidence,
    MissingSafetyInvariant,
    InvalidCapacity,
}

impl RawTightBackendRegistration {
    fn validate(self) -> Result<Self, RawTightRegistrationError> {
        if self.backend_version.get() == 0 {
            return Err(RawTightRegistrationError::InvalidBackendVersion);
        }
        if self.identity.adapter_abi_revision != RAW_TIGHT_ADAPTER_ABI_REVISION {
            return Err(RawTightRegistrationError::AdapterAbiMismatch {
                expected: RAW_TIGHT_ADAPTER_ABI_REVISION,
                registered: self.identity.adapter_abi_revision,
            });
        }
        if self.identity.algorithm_revision == 0 {
            return Err(RawTightRegistrationError::InvalidAlgorithmRevision);
        }
        if digest_is_zero(self.identity.implementation_digest)
            || digest_is_zero(self.identity.native_build_digest)
        {
            return Err(RawTightRegistrationError::MissingImplementationIdentity);
        }
        match self.identity.graph_backend {
            RawGraphBackendKind::IndependentNative => {
                if self.identity.gtsam_source_digest.is_some() {
                    return Err(RawTightRegistrationError::UnexpectedGtsamSourceDigest);
                }
            }
            RawGraphBackendKind::GtsamSystem | RawGraphBackendKind::GtsamVendored => {
                if self.identity.gtsam_source_digest.is_none_or(digest_is_zero) {
                    return Err(RawTightRegistrationError::MissingGtsamSourceDigest);
                }
            }
        }
        if (self.identity.graph_backend == RawGraphBackendKind::GtsamSystem
            && !cfg!(feature = "gtsam-system"))
            || (self.identity.graph_backend == RawGraphBackendKind::GtsamVendored
                && !cfg!(feature = "gtsam-vendored"))
        {
            return Err(RawTightRegistrationError::GraphBackendNotCompiled);
        }
        if !self.qualification.passed {
            return Err(RawTightRegistrationError::QualificationFailed);
        }
        if self.qualification.specification.get() == 0
            || digest_is_zero(self.qualification.specification_digest)
            || digest_is_zero(self.qualification.report_digest)
            || self.qualification.cycle_slip_cases == 0
            || self.qualification.reference_switch_cases == 0
            || self.qualification.rejected_incorrect_fix_cases == 0
            || self.qualification.covariance_coverage_sessions == 0
        {
            return Err(RawTightRegistrationError::IncompleteQualificationEvidence);
        }
        if !self.safety.all_required() {
            return Err(RawTightRegistrationError::MissingSafetyInvariant);
        }
        if self.maximum_signal_samples == 0
            || self.maximum_simultaneous_ambiguity_arcs == 0
            || self.maximum_integer_candidates == 0
        {
            return Err(RawTightRegistrationError::InvalidCapacity);
        }
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ConditionalFixKind {
    Partial,
    Full,
}

/// A fixed result is always explicitly conditional; there is no unconditional
/// fixed constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ConditionalAmbiguityFix {
    kind: ConditionalFixKind,
    fixed_ambiguity_count: u16,
    available_ambiguity_count: u16,
    hypothesis_digest: ContentDigestV1,
    uncertainty_basis: ConditionalUncertaintyBasis,
}

const COMPILED_BACKEND_REGISTRATION: Option<RawTightBackendRegistration> = None;

fn compiled_backend_registration() -> Option<&'static RawTightBackendRegistration> {
    COMPILED_BACKEND_REGISTRATION.as_ref()
}

fn digest_is_zero(digest: ContentDigestV1) -> bool {
    digest.as_bytes().iter().all(|byte| *byte == 0)
}
