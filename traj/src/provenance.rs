//! Capability, evidence-lineage, and immutable processing provenance contracts.

use core::cmp::Ordering;

use sha2::{Digest, Sha256};

use crate::{
    config::{ProcessingLevel, ProcessingPolicy},
    error::{ProcessError, ValidationError},
    ids::{
        BackendVersionId, CalibrationRevision, ContentDigestV1, NormalizationRevision,
        ResultRevisionId, SessionId, SourceId,
    },
    time::{SessionTime, TimeSpan},
};

/// Maximum evidence selections accepted in one allocation-free semantic view.
pub const MAX_EVIDENCE_SELECTIONS: usize = 256;
/// Maximum fallback attempts representable by the fixed processing levels.
pub const MAX_PROCESSING_ATTEMPTS: usize = 5;
/// Maximum immutable parents attached to one result revision.
pub const MAX_PARENT_REVISIONS: usize = 64;
/// Maximum external input digests directly attached to one result revision.
pub const MAX_EXTERNAL_INPUTS: usize = 128;

/// One independently selectable processing or evidence capability.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(u8)]
pub enum Capability {
    /// Bounded on-device navigation and live metrics.
    EmbeddedLive = 0,
    /// Exact replay under the transcript's named comparison contract. The v1
    /// contract is same-build bit-exact and makes no cross-target claim.
    CapturedReplay = 1,
    /// Recomputed normalized navigation evidence.
    RecomputedNavigation = 2,
    /// Offline solution-level fixed-interval smoothing.
    OfflineSmooth = 3,
    /// Optional workstation factor-graph processing.
    AdvancedGraph = 4,
    /// Optional raw tightly coupled GNSS/INS processing.
    RawTight = 5,
    /// Complete normalized high-rate IMU evidence.
    NormalizedImu = 6,
    /// Complete native IMU evidence suitable for recomputation.
    NativeImu = 7,
    /// Receiver-solution position/vector-velocity evidence.
    GnssSolution = 8,
    /// Raw rover GNSS observables.
    RawRoverGnss = 9,
    /// Raw base observations or equivalent received corrections.
    BaseOrCorrection = 10,
    /// Required broadcast/precise ephemerides.
    Ephemerides = 11,
    /// Complete timing/clock-model evidence.
    Timing = 12,
    /// Complete configuration/calibration/control provenance.
    Configuration = 13,
    /// Complete end-of-session record exists.
    CompleteEnd = 14,
    /// Offline ski/activity full-session evaluation.
    FullOfflineMetrics = 15,
}

/// Fixed bit set of declared or derived capabilities.
#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
#[repr(transparent)]
pub struct Capabilities(u64);

impl Capabilities {
    /// Empty capability set.
    pub const NONE: Self = Self(0);

    /// Constructs a set containing one capability.
    #[must_use]
    pub const fn one(capability: Capability) -> Self {
        Self(1_u64 << capability as u8)
    }

    /// Adds a capability and returns the updated value.
    #[must_use]
    pub const fn with(self, capability: Capability) -> Self {
        Self(self.0 | (1_u64 << capability as u8))
    }

    /// Returns whether the capability is present.
    #[must_use]
    pub const fn contains(self, capability: Capability) -> bool {
        self.0 & (1_u64 << capability as u8) != 0
    }

    /// Returns whether every capability in `required` is present.
    #[must_use]
    pub const fn contains_all(self, required: Self) -> bool {
        self.0 & required.0 == required.0
    }

    /// Returns the stable bit representation used by canonical codecs.
    #[must_use]
    pub const fn bits(self) -> u64 {
        self.0
    }

    /// Validates and constructs a bit set, rejecting unknown future bits.
    pub const fn from_bits(bits: u64) -> Result<Self, ValidationError> {
        const KNOWN: u64 = (1_u64 << 16) - 1;
        if bits & !KNOWN == 0 {
            Ok(Self(bits))
        } else {
            Err(ValidationError::IncompatibleDefinition)
        }
    }
}

/// Semantic evidence class used to enforce one selected lineage per span.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum EvidenceClass {
    /// Normalized calibrated IMU evidence.
    Imu,
    /// Receiver position/vector-velocity solutions.
    GnssSolution,
    /// Raw GNSS code/carrier/Doppler epochs.
    RawGnss,
    /// Ephemerides and orbit/clock products.
    Ephemeris,
    /// Base observations or correction content.
    BaseOrCorrection,
    /// Clock fits, PPS captures, and discontinuities.
    Timing,
    /// Configuration, calibration, installation, and user controls.
    Control,
    /// Replacement trajectory such as an imported PPK solution.
    ReplacementTrajectory,
}

/// Origin of the normalized semantic evidence selected for a source/span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceLineageKind {
    /// Exact normalized observations consumed on-device.
    Captured,
    /// Normalization rebuilt from verified native records.
    Recomputed,
    /// Native raw evidence consumed directly by raw-tight processing.
    Raw,
    /// Immutable external replacement or augmentation sidecar.
    External,
}

/// How selected evidence is allowed to influence a run.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceUse {
    /// Independent evidence entering the estimator objective/update.
    Fusion,
    /// Dependent evidence used only to initialize a state.
    InitializationOnly,
    /// Evidence retained only for diagnostics/comparison.
    DiagnosticOnly,
}

/// One immutable evidence-lineage selection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EvidenceSelection {
    /// Evidence source identity.
    pub source: SourceId,
    /// Semantic evidence class.
    pub class: EvidenceClass,
    /// Checksum-valid selected span.
    pub span: TimeSpan,
    /// Captured, recomputed, raw, or external lineage.
    pub lineage: EvidenceLineageKind,
    /// Normalization revision when the class is normalized evidence.
    pub normalization_revision: Option<NormalizationRevision>,
    /// Canonical digest of the selected semantic evidence.
    pub digest: ContentDigestV1,
    /// Permitted estimator use.
    pub usage: EvidenceUse,
}

impl EvidenceSelection {
    /// Validates lineage/revision consistency.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        let normalized = matches!(
            self.lineage,
            EvidenceLineageKind::Captured | EvidenceLineageKind::Recomputed
        );
        let zero_normalization_revision = match self.normalization_revision {
            Some(revision) => revision.get() == 0,
            None => false,
        };
        if self.source.get() == 0
            || self.digest.is_zero()
            || zero_normalization_revision
            || normalized != self.normalization_revision.is_some()
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Allocation-free borrowed set of evidence selections.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EvidenceLineage<'a> {
    selections: &'a [EvidenceSelection],
}

impl<'a> EvidenceLineage<'a> {
    /// Validates uniqueness and overlap rules for all selected evidence.
    pub fn new(selections: &'a [EvidenceSelection]) -> Result<Self, ValidationError> {
        if selections.is_empty() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if selections.len() > MAX_EVIDENCE_SELECTIONS {
            return Err(ValidationError::CapacityExceeded);
        }
        for (index, selection) in selections.iter().copied().enumerate() {
            selection.validate()?;
            for other in selections.iter().copied().skip(index + 1) {
                let replacement_conflict = (selection.class
                    == EvidenceClass::ReplacementTrajectory
                    || other.class == EvidenceClass::ReplacementTrajectory)
                    && spans_overlap(selection.span, other.span)
                    && selection.usage == EvidenceUse::Fusion
                    && other.usage == EvidenceUse::Fusion;
                if selection.source == other.source
                    && selection.class == other.class
                    && spans_overlap(selection.span, other.span)
                    && selection.usage == EvidenceUse::Fusion
                    && other.usage == EvidenceUse::Fusion
                    || replacement_conflict
                {
                    return Err(ValidationError::IncompatibleDefinition);
                }
            }
        }
        Ok(Self { selections })
    }

    /// Returns all validated selections.
    #[must_use]
    pub const fn selections(self) -> &'a [EvidenceSelection] {
        self.selections
    }

    /// Returns the canonical v1 identity of this exact ordered selection set.
    ///
    /// The list must be in strict canonical order. This makes semantically
    /// identical selection sets have one identity and lets evidence sources,
    /// processing preflight, and result sinks bind the same provenance
    /// without allocating or depending on a log codec.
    pub fn canonical_digest_v1(self) -> Result<ContentDigestV1, ValidationError> {
        for pair in self.selections.windows(2) {
            if evidence_selection_cmp(pair[0], pair[1]) != Ordering::Less {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        let count =
            u16::try_from(self.selections.len()).map_err(|_| ValidationError::CapacityExceeded)?;
        let mut hash = Sha256::new();
        hash.update(b"aevia.evidence-selection-set.v1\0");
        hash.update(count.to_le_bytes());
        for selection in self.selections {
            hash.update(selection.source.get().to_le_bytes());
            hash.update([evidence_class_identity(selection.class)]);
            hash.update(selection.span.start().as_ns().to_le_bytes());
            hash.update(selection.span.end().as_ns().to_le_bytes());
            hash.update([evidence_lineage_identity(selection.lineage)]);
            hash.update(
                selection
                    .normalization_revision
                    .map_or(0, NormalizationRevision::get)
                    .to_le_bytes(),
            );
            // Algorithm tag 1 is SHA-256 throughout the v1 provenance
            // contracts.
            hash.update([1]);
            hash.update(selection.digest.as_bytes());
            hash.update([evidence_use_identity(selection.usage)]);
        }
        Ok(ContentDigestV1::from_bytes(hash.finalize().into()))
    }

    /// Checks processing-level-specific double-use rules.
    pub fn validate_for_level(self, level: ProcessingLevel) -> Result<(), ProcessError> {
        if !matches!(level, ProcessingLevel::RawTight) {
            return Ok(());
        }
        for raw in self.selections.iter().filter(|item| {
            item.class == EvidenceClass::RawGnss && item.usage == EvidenceUse::Fusion
        }) {
            let duplicates_raw_information = self.selections.iter().any(|solution| {
                solution.source == raw.source
                    && solution.class == EvidenceClass::GnssSolution
                    && solution.usage == EvidenceUse::Fusion
                    && spans_overlap(solution.span, raw.span)
            });
            if duplicates_raw_information {
                return Err(ProcessError::EvidenceLineageConflict);
            }
        }
        Ok(())
    }
}

fn evidence_selection_cmp(first: EvidenceSelection, second: EvidenceSelection) -> Ordering {
    first
        .source
        .get()
        .cmp(&second.source.get())
        .then_with(|| {
            evidence_class_identity(first.class).cmp(&evidence_class_identity(second.class))
        })
        .then_with(|| first.span.start().as_ns().cmp(&second.span.start().as_ns()))
        .then_with(|| first.span.end().as_ns().cmp(&second.span.end().as_ns()))
        .then_with(|| {
            evidence_lineage_identity(first.lineage).cmp(&evidence_lineage_identity(second.lineage))
        })
        .then_with(|| {
            first
                .normalization_revision
                .map_or(0, NormalizationRevision::get)
                .cmp(
                    &second
                        .normalization_revision
                        .map_or(0, NormalizationRevision::get),
                )
        })
        .then_with(|| first.digest.as_bytes().cmp(second.digest.as_bytes()))
        .then_with(|| evidence_use_identity(first.usage).cmp(&evidence_use_identity(second.usage)))
}

const fn evidence_class_identity(value: EvidenceClass) -> u8 {
    match value {
        EvidenceClass::Imu => 0,
        EvidenceClass::GnssSolution => 1,
        EvidenceClass::RawGnss => 2,
        EvidenceClass::Ephemeris => 3,
        EvidenceClass::BaseOrCorrection => 4,
        EvidenceClass::Timing => 5,
        EvidenceClass::Control => 6,
        EvidenceClass::ReplacementTrajectory => 7,
    }
}

const fn evidence_lineage_identity(value: EvidenceLineageKind) -> u8 {
    match value {
        EvidenceLineageKind::Captured => 0,
        EvidenceLineageKind::Recomputed => 1,
        EvidenceLineageKind::Raw => 2,
        EvidenceLineageKind::External => 3,
    }
}

const fn evidence_use_identity(value: EvidenceUse) -> u8 {
    match value {
        EvidenceUse::Fusion => 0,
        EvidenceUse::InitializationOnly => 1,
        EvidenceUse::DiagnosticOnly => 2,
    }
}

/// Actual capability/completeness derived for one checksum-valid span.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpanCapabilities {
    /// Contiguous checksum-valid span.
    pub span: TimeSpan,
    /// Capabilities derived from records actually present.
    pub capabilities: Capabilities,
    /// Last canonical record sequence included in this span.
    pub terminal_record_sequence: u64,
    /// Whether a valid explicit end record closes the span.
    pub has_valid_end: bool,
}

/// Outcome of one candidate selected by a processing policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcessingAttemptOutcome {
    /// Candidate completed and produced the returned result revision.
    Succeeded,
    /// Candidate was not compiled for this build.
    NotCompiled,
    /// Platform could not run the candidate.
    PlatformUnsupported,
    /// Requested span lacked required evidence.
    EvidenceUnavailable,
    /// Installation, calibration, or qualification did not permit it.
    NotQualified,
    /// Preflight resource limits rejected it.
    InsufficientResources,
    /// Candidate began but failed without publishing partial state.
    Failed,
}

/// Recorded selection/rejection result for one processing candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProcessingAttempt {
    /// Attempted processing level.
    pub level: ProcessingLevel,
    /// Attempt order, starting at zero.
    pub ordinal: u8,
    /// Diagnosed outcome.
    pub outcome: ProcessingAttemptOutcome,
}

/// Concrete implementation identity hidden behind the engine interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BackendProvenance {
    /// Actual processing level.
    pub level: ProcessingLevel,
    /// Engine/backend implementation revision.
    pub version: BackendVersionId,
    /// Reviewed native-source digest, when a native backend was involved.
    pub native_source_digest: Option<ContentDigestV1>,
}

/// Immutable provenance carried by one trajectory/result revision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResultProvenance<'a> {
    /// Unique output revision.
    pub result_revision: ResultRevisionId,
    /// Source session identity.
    pub source_session: SessionId,
    /// Checksum-valid processed span.
    pub source_span: TimeSpan,
    /// Canonical source logical-record digest.
    pub source_digest: ContentDigestV1,
    /// Selected normalization digest.
    pub normalization_digest: ContentDigestV1,
    /// Immutable engine-configuration digest.
    pub configuration_digest: ContentDigestV1,
    /// Installation-definition digest.
    pub installation_digest: ContentDigestV1,
    /// Calibration bundle revision.
    pub calibration_revision: CalibrationRevision,
    /// Calibration bundle digest.
    pub calibration_digest: ContentDigestV1,
    /// Uncertainty-model-set digest.
    pub uncertainty_digest: ContentDigestV1,
    /// Metric-plan digest.
    pub metric_plan_digest: ContentDigestV1,
    /// Requested selection policy.
    pub requested_policy: ProcessingPolicy,
    /// Concrete backend used for this result.
    pub actual_backend: BackendProvenance,
    /// Every considered candidate in order.
    pub attempts: &'a [ProcessingAttempt],
    /// Immutable parent result revisions.
    pub parents: &'a [ResultRevisionId],
    /// External base/correction/ephemeris inputs.
    pub external_inputs: &'a [ContentDigestV1],
    /// Capabilities actually present in this result.
    pub capabilities: Capabilities,
}

impl ResultProvenance<'_> {
    /// Validates attempt order, unique parents, and successful-backend
    /// consistency without allocating.
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.result_revision.get() == 0
            || self.source_session.is_zero()
            || self.source_digest.is_zero()
            || self.normalization_digest.is_zero()
            || self.configuration_digest.is_zero()
            || self.installation_digest.is_zero()
            || self.calibration_revision.get() == 0
            || self.calibration_digest.is_zero()
            || self.uncertainty_digest.is_zero()
            || self.metric_plan_digest.is_zero()
            || self.actual_backend.version.get() == 0
            || self.attempts.is_empty()
            || self.attempts.len() > MAX_PROCESSING_ATTEMPTS
            || self.parents.len() > MAX_PARENT_REVISIONS
            || self.external_inputs.len() > MAX_EXTERNAL_INPUTS
            || !self
                .capabilities
                .contains(capability_for_level(self.actual_backend.level))
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        let mut success_count = 0_u8;
        for (index, attempt) in self.attempts.iter().enumerate() {
            if usize::from(attempt.ordinal) != index
                || self
                    .requested_policy
                    .levels()
                    .get(index)
                    .is_none_or(|level| *level != attempt.level)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
            if attempt.outcome == ProcessingAttemptOutcome::Succeeded {
                success_count = success_count.saturating_add(1);
                if attempt.level != self.actual_backend.level {
                    return Err(ValidationError::IncompatibleDefinition);
                }
            }
        }
        if success_count != 1 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        for (index, parent) in self.parents.iter().enumerate() {
            if parent.get() == 0
                || *parent == self.result_revision
                || self.parents[index + 1..].contains(parent)
            {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        for (index, digest) in self.external_inputs.iter().enumerate() {
            if digest.is_zero() || self.external_inputs[index + 1..].contains(digest) {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        Ok(())
    }
}

const fn capability_for_level(level: ProcessingLevel) -> Capability {
    match level {
        ProcessingLevel::EmbeddedLive => Capability::EmbeddedLive,
        ProcessingLevel::CapturedReplay => Capability::CapturedReplay,
        ProcessingLevel::OfflineSmooth => Capability::OfflineSmooth,
        ProcessingLevel::AdvancedGraph => Capability::AdvancedGraph,
        ProcessingLevel::RawTight => Capability::RawTight,
    }
}

fn spans_overlap(first: TimeSpan, second: TimeSpan) -> bool {
    first.start() <= second.end() && second.start() <= first.end()
}

/// Returns whether `time` is covered by the selected span.
#[must_use]
pub const fn selection_covers(selection: EvidenceSelection, time: SessionTime) -> bool {
    selection.span.contains(time)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::QualificationSpecId;

    fn selection(class: EvidenceClass, lineage: EvidenceLineageKind) -> EvidenceSelection {
        EvidenceSelection {
            source: SourceId::new(1),
            class,
            span: TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(10)).unwrap(),
            lineage,
            normalization_revision: match lineage {
                EvidenceLineageKind::Captured | EvidenceLineageKind::Recomputed => {
                    Some(NormalizationRevision::new(1))
                }
                EvidenceLineageKind::Raw | EvidenceLineageKind::External => None,
            },
            digest: ContentDigestV1::from_bytes([1; 32]),
            usage: EvidenceUse::Fusion,
        }
    }

    #[test]
    fn capabilities_reject_unknown_bits() {
        assert_eq!(
            Capabilities::from_bits(1_u64 << 63),
            Err(ValidationError::IncompatibleDefinition)
        );
        let set = Capabilities::NONE.with(Capability::EmbeddedLive);
        assert!(set.contains(Capability::EmbeddedLive));
    }
    #[test]
    fn evidence_lineage_rejects_empty_and_placeholder_identity() {
        assert_eq!(
            EvidenceLineage::new(&[]),
            Err(ValidationError::IncompatibleDefinition)
        );

        let mut invalid = selection(EvidenceClass::Imu, EvidenceLineageKind::Captured);
        invalid.source = SourceId::new(0);
        assert_eq!(
            EvidenceLineage::new(&[invalid]),
            Err(ValidationError::IncompatibleDefinition)
        );

        invalid = selection(EvidenceClass::Imu, EvidenceLineageKind::Captured);
        invalid.digest = ContentDigestV1::from_bytes([0; 32]);
        assert_eq!(
            EvidenceLineage::new(&[invalid]),
            Err(ValidationError::IncompatibleDefinition)
        );

        invalid = selection(EvidenceClass::Imu, EvidenceLineageKind::Captured);
        invalid.normalization_revision = Some(NormalizationRevision::new(0));
        assert_eq!(
            EvidenceLineage::new(&[invalid]),
            Err(ValidationError::IncompatibleDefinition)
        );
    }

    #[test]
    fn overlapping_competing_lineages_are_rejected() {
        let captured = selection(EvidenceClass::Imu, EvidenceLineageKind::Captured);
        let recomputed = selection(EvidenceClass::Imu, EvidenceLineageKind::Recomputed);
        assert_eq!(
            EvidenceLineage::new(&[captured, recomputed]),
            Err(ValidationError::IncompatibleDefinition)
        );
    }

    #[test]
    fn raw_tight_rejects_fused_pvt_from_the_same_evidence_source() {
        let raw = selection(EvidenceClass::RawGnss, EvidenceLineageKind::Raw);
        let solution = selection(EvidenceClass::GnssSolution, EvidenceLineageKind::Captured);
        let selections = [raw, solution];
        let lineage = EvidenceLineage::new(&selections).unwrap();
        assert_eq!(
            lineage.validate_for_level(ProcessingLevel::RawTight),
            Err(ProcessError::EvidenceLineageConflict)
        );
    }

    #[test]
    fn pvt_may_be_retained_for_raw_tight_initialization_only() {
        let raw = selection(EvidenceClass::RawGnss, EvidenceLineageKind::Raw);
        let mut solution = selection(EvidenceClass::GnssSolution, EvidenceLineageKind::Captured);
        solution.usage = EvidenceUse::InitializationOnly;
        let selections = [raw, solution];
        let lineage = EvidenceLineage::new(&selections).unwrap();
        assert_eq!(
            lineage.validate_for_level(ProcessingLevel::RawTight),
            Ok(())
        );
    }

    #[test]
    fn result_requires_one_success_matching_actual_backend() {
        let attempts = [ProcessingAttempt {
            level: ProcessingLevel::OfflineSmooth,
            ordinal: 0,
            outcome: ProcessingAttemptOutcome::Succeeded,
        }];
        let result = ResultProvenance {
            result_revision: ResultRevisionId::new(1),
            source_session: SessionId::from_bytes([1; 16]),
            source_span: TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(1)).unwrap(),
            source_digest: ContentDigestV1::from_bytes([1; 32]),
            normalization_digest: ContentDigestV1::from_bytes([2; 32]),
            configuration_digest: ContentDigestV1::from_bytes([3; 32]),
            installation_digest: ContentDigestV1::from_bytes([4; 32]),
            calibration_revision: CalibrationRevision::new(1),
            calibration_digest: ContentDigestV1::from_bytes([5; 32]),
            uncertainty_digest: ContentDigestV1::from_bytes([6; 32]),
            metric_plan_digest: ContentDigestV1::from_bytes([7; 32]),
            requested_policy: ProcessingPolicy::require(ProcessingLevel::OfflineSmooth),
            actual_backend: BackendProvenance {
                level: ProcessingLevel::OfflineSmooth,
                version: BackendVersionId::new(1),
                native_source_digest: None,
            },
            attempts: &attempts,
            parents: &[],
            external_inputs: &[],
            capabilities: Capabilities::one(Capability::OfflineSmooth),
        };
        assert_eq!(result.validate(), Ok(()));
        let _ = QualificationSpecId::new(1);
    }
}
