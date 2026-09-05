//! Evidence, lineage, and qualification preflight for raw-tight backend execution.

use super::{
    RawTightBackendRegistration, RawTightRegistrationError, compiled_backend_registration,
    digest_is_zero,
};
use crate::config::{OfflineResourceLimits, ProcessingLevel};
use crate::error::{PrepareError, ProcessError};
use crate::ids::{BackendVersionId, ContentDigestV1, SourceId};
use crate::provenance::{
    Capabilities, Capability, EvidenceClass, EvidenceLineage, EvidenceLineageKind, EvidenceUse,
    ProcessingAttemptOutcome,
};
use crate::time::TimeSpan;

mod resources;
use resources::{enforce_limits, estimate_resources};

/// Counts produced by a checksum-valid semantic scan before any graph is
/// allocated. `base_or_correction_epochs` includes aligned base epochs or
/// complete equivalent correction messages selected by the recording profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightProblemShape {
    pub imu_samples: u64,
    pub rover_epochs: u64,
    pub base_or_correction_epochs: u64,
    pub raw_signal_samples: u64,
    pub pseudorange_samples: u64,
    pub ambiguity_carrier_phase_samples: u64,
    pub tdcp_samples: u64,
    pub doppler_samples: u64,
    pub ephemeris_records: u64,
    pub proposed_keyframes: u64,
    pub receiver_clock_nodes: u64,
    pub troposphere_nodes: u64,
    pub inter_system_bias_coordinates: u16,
    pub maximum_simultaneous_ambiguity_arcs: u32,
    pub requested_output_epochs: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum RawTightEvidenceRequirement {
    RoverBaseEpochAlignment = 0,
    BaseCoordinateFrameAndEpoch = 1,
    SignalFrequencyMetadata = 2,
    EphemerisIssueAndValidity = 3,
    CorrectionAge = 4,
    TrackingContinuity = 5,
    DifferencedCovarianceInputs = 6,
}

/// Semantic checks that require looking inside evidence records rather than
/// only at manifest capability bits.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(transparent)]
pub(crate) struct RawTightEvidenceChecks(u16);

impl RawTightEvidenceChecks {
    pub(crate) const NONE: Self = Self(0);
    pub(crate) const ALL: Self = Self((1_u16 << 7) - 1);

    pub(crate) const fn with(self, requirement: RawTightEvidenceRequirement) -> Self {
        Self(self.0 | (1_u16 << requirement as u8))
    }

    pub(crate) const fn contains(self, requirement: RawTightEvidenceRequirement) -> bool {
        self.0 & (1_u16 << requirement as u8) != 0
    }

    #[cfg(test)]
    pub(super) const fn without(self, requirement: RawTightEvidenceRequirement) -> Self {
        Self(self.0 & !(1_u16 << requirement as u8))
    }
}

impl RawTightEvidenceChecks {
    fn first_missing(self) -> Option<RawTightEvidenceRequirement> {
        [
            RawTightEvidenceRequirement::RoverBaseEpochAlignment,
            RawTightEvidenceRequirement::BaseCoordinateFrameAndEpoch,
            RawTightEvidenceRequirement::SignalFrequencyMetadata,
            RawTightEvidenceRequirement::EphemerisIssueAndValidity,
            RawTightEvidenceRequirement::CorrectionAge,
            RawTightEvidenceRequirement::TrackingContinuity,
            RawTightEvidenceRequirement::DifferencedCovarianceInputs,
        ]
        .into_iter()
        .find(|requirement| !self.contains(*requirement))
    }
}

/// Full preflight request. Raw-tight must be explicitly required; it cannot be
/// reached merely because it appears attractive in a fallback order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RawTightPreflightRequest<'a> {
    pub span: TimeSpan,
    pub capabilities: Capabilities,
    pub evidence_lineage: EvidenceLineage<'a>,
    pub evidence_restartable: bool,
    pub explicitly_required: bool,
    pub profile_qualified: bool,
    pub evidence_checks: RawTightEvidenceChecks,
    pub shape: RawTightProblemShape,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightPassLimits {
    pub reverse_initialization_passes: u8,
    pub robust_reclassification_passes: u8,
    pub float_solver_iterations: u16,
    pub integer_validation_passes: u8,
    pub maximum_integer_candidates: u32,
}

/// Conservative resource estimate produced before graph construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightResourceEstimate {
    pub estimation_model_revision: u32,
    pub proposed_keyframes: u64,
    pub state_coordinate_count: u64,
    pub factor_count: u64,
    pub estimated_sparse_nonzeros: u64,
    pub estimated_peak_memory_bytes: u64,
    pub estimated_temporary_storage_bytes: u64,
    pub estimated_output_bytes: u64,
    pub progress_work_units: u64,
    pub minimum_worker_count: u16,
    pub pass_limits: RawTightPassLimits,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RawTightResourceKind {
    PeakMemory,
    TemporaryStorage,
    OutputStorage,
    WorkerCount,
    ElapsedWork,
    BackendSignalCapacity,
    BackendAmbiguityCapacity,
    BackendIntegerCandidateCapacity,
}

/// Precise fail-closed preflight result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RawTightPreflightError {
    ExplicitSelectionRequired,
    InvalidProblemShape,
    EstimateOverflow,
    MissingEvidence(Capability),
    MissingLineage(EvidenceClass),
    EvidenceCheckFailed(RawTightEvidenceRequirement),
    ConflictingEvidenceLineage,
    PvtDoubleUse {
        source: SourceId,
    },
    EvidenceNotRestartable,
    ProfileUnqualified,
    BackendNotRegistered,
    InvalidRegistration(RawTightRegistrationError),
    InvalidResourceLimits,
    InsufficientResource {
        resource: RawTightResourceKind,
        required: u64,
        available: u64,
    },
}

impl RawTightPreflightError {
    pub(crate) const fn attempt_outcome(self) -> ProcessingAttemptOutcome {
        match self {
            Self::MissingEvidence(_)
            | Self::MissingLineage(_)
            | Self::EvidenceCheckFailed(_)
            | Self::ConflictingEvidenceLineage
            | Self::PvtDoubleUse { .. }
            | Self::EvidenceNotRestartable => ProcessingAttemptOutcome::EvidenceUnavailable,
            Self::ExplicitSelectionRequired
            | Self::ProfileUnqualified
            | Self::InvalidRegistration(_) => ProcessingAttemptOutcome::NotQualified,
            Self::BackendNotRegistered => ProcessingAttemptOutcome::NotCompiled,
            Self::InsufficientResource { .. } => ProcessingAttemptOutcome::InsufficientResources,
            Self::InvalidProblemShape | Self::EstimateOverflow | Self::InvalidResourceLimits => {
                ProcessingAttemptOutcome::Failed
            }
        }
    }

    pub(crate) const fn prepare_error(self) -> PrepareError {
        match self {
            Self::MissingEvidence(_)
            | Self::MissingLineage(_)
            | Self::EvidenceCheckFailed(_)
            | Self::ConflictingEvidenceLineage
            | Self::PvtDoubleUse { .. }
            | Self::EvidenceNotRestartable => PrepareError::EvidenceUnavailable,
            Self::ExplicitSelectionRequired
            | Self::ProfileUnqualified
            | Self::InvalidRegistration(_) => PrepareError::UnqualifiedProfile,
            Self::BackendNotRegistered => PrepareError::CapabilityUnavailable,
            Self::InvalidProblemShape
            | Self::EstimateOverflow
            | Self::InvalidResourceLimits
            | Self::InsufficientResource { .. } => PrepareError::InsufficientResources,
        }
    }

    pub(crate) const fn process_error(self) -> ProcessError {
        match self {
            Self::MissingEvidence(_)
            | Self::MissingLineage(_)
            | Self::EvidenceCheckFailed(_)
            | Self::EvidenceNotRestartable => ProcessError::IncompleteEvidence,
            Self::ConflictingEvidenceLineage | Self::PvtDoubleUse { .. } => {
                ProcessError::EvidenceLineageConflict
            }
            Self::BackendNotRegistered => ProcessError::CapabilityUnavailable,
            Self::InvalidProblemShape
            | Self::ExplicitSelectionRequired
            | Self::ProfileUnqualified
            | Self::InvalidRegistration(_) => ProcessError::AdvancedCapabilityFailure,
            Self::EstimateOverflow
            | Self::InvalidResourceLimits
            | Self::InsufficientResource { .. } => ProcessError::ResourceLimit,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightPreflightFailure {
    pub error: RawTightPreflightError,
    pub estimate: Option<RawTightResourceEstimate>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RawTightCandidateDiagnostic {
    pub level: ProcessingLevel,
    /// `None` means ready for native invocation, not that a result succeeded.
    pub final_outcome: Option<ProcessingAttemptOutcome>,
    pub error: Option<RawTightPreflightError>,
    pub estimate: Option<RawTightResourceEstimate>,
    pub backend_version: Option<BackendVersionId>,
}

impl RawTightPreflightFailure {
    pub(crate) const fn diagnostic(self) -> RawTightCandidateDiagnostic {
        RawTightCandidateDiagnostic {
            level: ProcessingLevel::RawTight,
            final_outcome: Some(self.error.attempt_outcome()),
            error: Some(self.error),
            estimate: self.estimate,
            backend_version: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PreparedRawTightBackend {
    registration: RawTightBackendRegistration,
    estimate: RawTightResourceEstimate,
}

impl PreparedRawTightBackend {
    pub(crate) const fn estimate(self) -> RawTightResourceEstimate {
        self.estimate
    }

    pub(crate) const fn backend_version(self) -> BackendVersionId {
        self.registration.backend_version
    }

    pub(crate) const fn implementation_digest(self) -> ContentDigestV1 {
        self.registration.identity.implementation_digest
    }

    pub(crate) const fn diagnostic(self) -> RawTightCandidateDiagnostic {
        RawTightCandidateDiagnostic {
            level: ProcessingLevel::RawTight,
            final_outcome: None,
            error: None,
            estimate: Some(self.estimate),
            backend_version: Some(self.registration.backend_version),
        }
    }
}

pub(crate) fn preflight(
    request: RawTightPreflightRequest<'_>,
    limits: OfflineResourceLimits,
) -> Result<PreparedRawTightBackend, RawTightPreflightFailure> {
    preflight_with_registration(request, limits, compiled_backend_registration())
}

pub(super) fn preflight_with_registration(
    request: RawTightPreflightRequest<'_>,
    limits: OfflineResourceLimits,
    registration: Option<&RawTightBackendRegistration>,
) -> Result<PreparedRawTightBackend, RawTightPreflightFailure> {
    let estimate = estimate_resources(request.shape).map_err(|error| RawTightPreflightFailure {
        error,
        estimate: None,
    })?;
    let failure = |error| RawTightPreflightFailure {
        error,
        estimate: Some(estimate),
    };

    if !request.explicitly_required {
        return Err(failure(RawTightPreflightError::ExplicitSelectionRequired));
    }
    for capability in [
        Capability::NormalizedImu,
        Capability::RawRoverGnss,
        Capability::BaseOrCorrection,
        Capability::Ephemerides,
        Capability::Timing,
        Capability::Configuration,
        Capability::CompleteEnd,
    ] {
        if !request.capabilities.contains(capability) {
            return Err(failure(RawTightPreflightError::MissingEvidence(capability)));
        }
    }
    if request
        .evidence_lineage
        .validate_for_level(ProcessingLevel::RawTight)
        .is_err()
    {
        return Err(failure(RawTightPreflightError::ConflictingEvidenceLineage));
    }
    reject_pvt_double_use(request.evidence_lineage, request.span).map_err(failure)?;
    for class in [
        EvidenceClass::Imu,
        EvidenceClass::RawGnss,
        EvidenceClass::BaseOrCorrection,
        EvidenceClass::Ephemeris,
        EvidenceClass::Timing,
        EvidenceClass::Control,
    ] {
        if !lineage_covers_span(request.evidence_lineage, request.span, class) {
            return Err(failure(RawTightPreflightError::MissingLineage(class)));
        }
    }
    if let Some(requirement) = request.evidence_checks.first_missing() {
        return Err(failure(RawTightPreflightError::EvidenceCheckFailed(
            requirement,
        )));
    }
    if !request.evidence_restartable {
        return Err(failure(RawTightPreflightError::EvidenceNotRestartable));
    }
    if !request.profile_qualified {
        return Err(failure(RawTightPreflightError::ProfileUnqualified));
    }
    let Some(registration) = registration.copied() else {
        return Err(failure(RawTightPreflightError::BackendNotRegistered));
    };
    let registration = registration
        .validate()
        .map_err(|error| failure(RawTightPreflightError::InvalidRegistration(error)))?;
    limits
        .validate()
        .map_err(|_| failure(RawTightPreflightError::InvalidResourceLimits))?;
    enforce_limits(estimate, request.shape, registration, limits).map_err(failure)?;

    Ok(PreparedRawTightBackend {
        registration,
        estimate,
    })
}

fn reject_pvt_double_use(
    lineage: EvidenceLineage<'_>,
    requested: TimeSpan,
) -> Result<(), RawTightPreflightError> {
    // EvidenceSelection does not currently carry an explicit derivation edge
    // from receiver PVT to its raw epochs. Conservatively reject every fused
    // PVT selection overlapping this raw-tight request. Initialization-only or
    // diagnostic PVT remains available and is not counted as independent.
    if let Some(selection) = lineage.selections().iter().find(|selection| {
        selection.class == EvidenceClass::GnssSolution
            && selection.usage == EvidenceUse::Fusion
            && spans_overlap(selection.span, requested)
    }) {
        Err(RawTightPreflightError::PvtDoubleUse {
            source: selection.source,
        })
    } else {
        Ok(())
    }
}

fn lineage_covers_span(
    lineage: EvidenceLineage<'_>,
    requested: TimeSpan,
    class: EvidenceClass,
) -> bool {
    let mut cursor = requested.start().as_ns();
    let requested_end = requested.end().as_ns();
    loop {
        let mut furthest = None;
        for selection in lineage.selections().iter().copied() {
            if selection.class != class
                || selection.usage != EvidenceUse::Fusion
                || !lineage_kind_allowed(class, selection.lineage)
                || digest_is_zero(selection.digest)
                || selection.span.start().as_ns() > cursor
                || selection.span.end().as_ns() < cursor
            {
                continue;
            }
            furthest = Some(
                furthest.map_or(selection.span.end().as_ns(), |current: i64| {
                    current.max(selection.span.end().as_ns())
                }),
            );
        }
        let Some(covered_end) = furthest else {
            return false;
        };
        if covered_end >= requested_end {
            return true;
        }
        let Some(next) = covered_end.checked_add(1) else {
            return false;
        };
        cursor = next;
    }
}

const fn lineage_kind_allowed(class: EvidenceClass, kind: EvidenceLineageKind) -> bool {
    match class {
        EvidenceClass::RawGnss => matches!(kind, EvidenceLineageKind::Raw),
        EvidenceClass::BaseOrCorrection | EvidenceClass::Ephemeris => {
            matches!(
                kind,
                EvidenceLineageKind::Raw | EvidenceLineageKind::External
            )
        }
        EvidenceClass::Imu | EvidenceClass::Timing | EvidenceClass::Control => matches!(
            kind,
            EvidenceLineageKind::Captured
                | EvidenceLineageKind::Recomputed
                | EvidenceLineageKind::External
        ),
        EvidenceClass::GnssSolution | EvidenceClass::ReplacementTrajectory => false,
    }
}

#[allow(
    clippy::too_many_lines,
    reason = "keep the checked revisioned estimate formula auditable in one place"
)]
fn spans_overlap(first: TimeSpan, second: TimeSpan) -> bool {
    first.start() <= second.end() && second.start() <= first.end()
}
