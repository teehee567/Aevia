//! Borrowed host-evidence and transactional result-publication ports.
//!
//! These ports deliberately describe semantic records rather than artifact
//! bytes or filesystem handles.  An artifact reader and an in-memory fixture
//! are therefore interchangeable adapters at this seam, while framing,
//! decoding, and storage remain outside the estimator.

use crate::{
    config::{
        CapturedReplayComparison, InitialClockConsiderPrior, InitialHeading, LiveResourceLimits,
    },
    error::{ProcessError, ValidationError},
    frame::{BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody},
    ids::{ClockModelId, ClockSegmentId, ContentDigestV1, SessionId, TrajectoryRevision},
    metric::{LiveMetricLimits, MetricResults},
    observation::{LiveObservation, WorkQuota},
    provenance::{Capabilities, ResultProvenance, SpanCapabilities},
    quality::{DiagnosticCounts, EstimateQuality, ObservabilityReport},
    time::{SessionTime, TimeSpan},
    uncertainty::KinematicCovariance,
};
use core::{
    fmt::{self, Write as _},
    num::NonZeroU64,
};
use sha2::{Digest, Sha256};

/// Hard protection against a hostile stream declaring millions of clock
/// segments and thereby forcing an unbounded nuisance-parameter matrix.
pub const MAX_OFFLINE_CLOCK_MODELS: usize = 256;

/// Version of the fixed, bounded captured-live-call contract carrying an
/// explicit initial clock-segment identity.
pub const CAPTURED_REPLAY_CONTRACT_V2: u16 = 2;
/// Canonical navigation-restart recipe understood by captured replay.
pub const CAPTURED_REINITIALIZATION_SCHEMA_V2: u16 = 2;
const CAPTURED_LIVE_CALL_SCHEMA_V1: u16 = 1;

/// Complete construction data bound to one captured live-call transcript.
///
/// Version two supports replay from session start or from a schema-v2
/// navigation reinitialization carrying the same complete construction data.
/// Opaque private-state seeds still fail closed until their state schema is
/// implemented by the engine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapturedReplayContract {
    pub version: u16,
    /// V2 retains the [`CapturedReplayComparison::SameBuildBitExactV1`]
    /// comparison algorithm.
    pub comparison: CapturedReplayComparison,
    pub transcript_digest: ContentDigestV1,
    pub configuration_digest: ContentDigestV1,
    pub navigation_profile_digest: ContentDigestV1,
    pub metric_plan_digest: ContentDigestV1,
    /// Exact call count (also the allocation/preflight bound).
    pub maximum_call_count: u64,
    /// Exact sum of every non-zero call's corrected-frontier credits. This
    /// bounds replayed frontier work, not total CPU work in the fixed-capacity
    /// phases surrounding each call.
    pub maximum_total_work_units: u64,
    pub metric_limits: LiveMetricLimits,
    pub resources: LiveResourceLimits,
    pub initial_heading: Option<InitialHeading>,
    pub initial_clock_prior: InitialClockConsiderPrior,
}

impl CapturedReplayContract {
    pub(crate) fn validate(self) -> Result<Self, ValidationError> {
        if self.version != CAPTURED_REPLAY_CONTRACT_V2
            || self.comparison != CapturedReplayComparison::SameBuildBitExactV1
            || self.transcript_digest.is_zero()
            || self.configuration_digest.is_zero()
            || self.navigation_profile_digest.is_zero()
            || self.metric_plan_digest.is_zero()
            || self.maximum_call_count == 0
            || self.maximum_total_work_units == 0
            || self.maximum_total_work_units < self.maximum_call_count
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.resources.validate_v2_mini()?;
        self.initial_clock_prior.validate()?;
        Ok(self)
    }
}

/// Exact recorded call to [`crate::LiveSession::step`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CapturedLiveStepCall {
    pub call_index: u64,
    /// Linked observation record, or `None` for an empty drain call.
    pub observation_record_sequence: Option<u64>,
    pub work: WorkQuota,
    /// Same-build bit-exact digest; this is not a cross-target tolerance test.
    pub expected_bit_exact_update_digest: ContentDigestV1,
}

/// Exact recorded call to [`crate::LiveSession::finish`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CapturedLiveFinishCall {
    pub call_index: u64,
    pub work: WorkQuota,
    pub expected_complete: bool,
    /// Same-build bit-exact digest; this is not a cross-target tolerance test.
    pub expected_bit_exact_update_digest: ContentDigestV1,
    /// Required exactly on the completing call.
    pub expected_summary_digest: Option<ContentDigestV1>,
}

/// Incremental canonical identity of a captured-live call transcript.
///
/// This is an engine-owned semantic contract. Artifact adapters decode their
/// wire records into [`CapturedLiveStepCall`] and [`CapturedLiveFinishCall`]
/// before crossing the evidence port; the estimator never depends on a log
/// codec or its record types.
#[derive(Clone, Debug)]
pub(crate) struct CapturedTranscriptDigestV1 {
    hash: Sha256,
    next_call: u64,
}

impl CapturedTranscriptDigestV1 {
    pub(crate) fn new() -> Self {
        let mut hash = Sha256::new();
        hash.update(b"aevia.captured-live-transcript.v1\0");
        Self { hash, next_call: 0 }
    }

    pub(crate) fn observe_step(
        &mut self,
        call: CapturedLiveStepCall,
    ) -> Result<(), ValidationError> {
        if call.call_index != self.next_call || call.expected_bit_exact_update_digest.is_zero() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        let payload_bytes = 2_u32
            + 8
            + 1
            + if call.observation_record_sequence.is_some() {
                8
            } else {
                0
            }
            + 4
            + 33;
        self.hash.update(26_u16.to_le_bytes());
        self.hash.update(payload_bytes.to_le_bytes());
        self.hash.update(CAPTURED_LIVE_CALL_SCHEMA_V1.to_le_bytes());
        self.hash.update(call.call_index.to_le_bytes());
        self.hash
            .update([u8::from(call.observation_record_sequence.is_some())]);
        if let Some(sequence) = call.observation_record_sequence {
            self.hash.update(sequence.to_le_bytes());
        }
        self.hash.update(call.work.units().to_le_bytes());
        hash_digest(&mut self.hash, call.expected_bit_exact_update_digest);
        self.next_call = self
            .next_call
            .checked_add(1)
            .ok_or(ValidationError::CapacityExceeded)?;
        Ok(())
    }

    pub(crate) fn observe_finish(
        &mut self,
        call: CapturedLiveFinishCall,
    ) -> Result<(), ValidationError> {
        if call.call_index != self.next_call
            || call.expected_bit_exact_update_digest.is_zero()
            || call.expected_complete != call.expected_summary_digest.is_some()
            || call
                .expected_summary_digest
                .is_some_and(ContentDigestV1::is_zero)
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        let payload_bytes = 2_u32
            + 8
            + 4
            + 1
            + 33
            + 1
            + if call.expected_summary_digest.is_some() {
                33
            } else {
                0
            };
        self.hash.update(27_u16.to_le_bytes());
        self.hash.update(payload_bytes.to_le_bytes());
        self.hash.update(CAPTURED_LIVE_CALL_SCHEMA_V1.to_le_bytes());
        self.hash.update(call.call_index.to_le_bytes());
        self.hash.update(call.work.units().to_le_bytes());
        self.hash.update([u8::from(call.expected_complete)]);
        hash_digest(&mut self.hash, call.expected_bit_exact_update_digest);
        self.hash
            .update([u8::from(call.expected_summary_digest.is_some())]);
        if let Some(digest) = call.expected_summary_digest {
            hash_digest(&mut self.hash, digest);
        }
        self.next_call = self
            .next_call
            .checked_add(1)
            .ok_or(ValidationError::CapacityExceeded)?;
        Ok(())
    }

    pub(crate) fn finalize(self) -> ContentDigestV1 {
        ContentDigestV1::from_bytes(self.hash.finalize().into())
    }
}

fn hash_digest(hash: &mut Sha256, digest: ContentDigestV1) {
    // Algorithm tag 1 is SHA-256 in every v1 canonical artifact schema.
    hash.update([1]);
    hash.update(digest.as_bytes());
}

/// Independently preflightable description of one semantic evidence stream.
///
/// The runner receives a second copy from [`EvidenceSource::manifest`] and
/// requires exact equality before and after every restart. Offline numerical
/// passes additionally seal the complete semantic event stream, so unchanged
/// manifest metadata cannot hide a value mutation between restarts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EvidenceManifest {
    /// Immutable source session.
    pub session_id: SessionId,
    /// Digest of the canonical logical record stream, independent of chunking.
    pub source_logical_digest: ContentDigestV1,
    /// Digest of the selected captured or recomputed normalization.
    pub normalization_digest: ContentDigestV1,
    /// Digest of the complete engine configuration named by the evidence.
    pub configuration_digest: ContentDigestV1,
    /// Checksum-valid span and capabilities derived from records actually seen.
    pub span_capabilities: SpanCapabilities,
    /// Convenience copy of the whole-source capability set.  It must contain
    /// the span-local set above; it is not trusted as a substitute for it.
    pub capabilities: Capabilities,
    /// Whether immutable evidence can be restarted for fallback/IEKS passes.
    pub restartable: bool,
    /// Conservative semantic-event count, when known without scanning.
    pub estimated_event_count: Option<u64>,
    /// Full typed live construction contract, when exact replay is available.
    pub captured_replay: Option<CapturedReplayContract>,
}

impl EvidenceManifest {
    /// Validates internal completeness and capability consistency.
    ///
    /// # Errors
    ///
    /// Returns [`ValidationError::IncompatibleDefinition`] when the declared
    /// whole-source capabilities do not cover the checksum-derived span-local
    /// capabilities, or when an explicitly known event bound is empty.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.session_id.is_zero()
            || self.source_logical_digest.is_zero()
            || self.normalization_digest.is_zero()
            || self.configuration_digest.is_zero()
            || !self.span_capabilities.has_valid_end
            || !self
                .capabilities
                .contains_all(self.span_capabilities.capabilities)
            || matches!(self.estimated_event_count, Some(0))
            || self
                .captured_replay
                .is_some_and(|contract| contract.validate().is_err())
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Cause and semantic consequence of a checksum-valid evidence gap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceGapReason {
    DroppedRecords,
    QueueOverrun,
    StorageFailure,
    SensorReset,
    UnknownRequiredEvidence,
}

/// A gap never permits interpolation.  A later explicit reinitialization can
/// start another disconnected trajectory span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvidenceGap {
    pub span: TimeSpan,
    pub reason: EvidenceGapReason,
}

/// Why the estimator must discard its dynamic state and wait for a new
/// absolute initialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReinitializationReason {
    ExplicitRecord,
    CommonImuFault,
    ClockDiscontinuity,
    NumericalIntegrity,
    PostGapRecovery,
}

/// Fully typed construction recipe for a captured span that begins at an
/// explicit navigation reinitialization.
///
/// The canonical artifact payload for
/// [`CAPTURED_REINITIALIZATION_SCHEMA_V2`] must decode to every field below;
/// a digest or schema tag alone is not a restorable state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapturedReinitializationInputV2 {
    pub navigation_profile_digest: ContentDigestV1,
    pub metric_plan_digest: ContentDigestV1,
    pub resources: LiveResourceLimits,
    pub initial_heading: Option<InitialHeading>,
    pub initial_clock_prior: InitialClockConsiderPrior,
}

impl CapturedReinitializationInputV2 {
    fn validate_for_contract(self, contract: CapturedReplayContract) -> Result<(), ProcessError> {
        if self.navigation_profile_digest != contract.navigation_profile_digest
            || self.metric_plan_digest != contract.metric_plan_digest
            || self.resources != contract.resources
            || self.initial_heading != contract.initial_heading
            || self.initial_clock_prior != contract.initial_clock_prior
        {
            return Err(ProcessError::IncompleteEvidence);
        }
        self.resources
            .validate_v2_mini()
            .map_err(|_| ProcessError::InvalidEvidence)?;
        self.initial_clock_prior
            .validate()
            .map_err(|_| ProcessError::InvalidEvidence)?;
        Ok(())
    }
}

/// Semantic reinitialization marker.  It intentionally contains no public
/// solver vector or covariance representation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReinitializationEvidence {
    pub at: SessionTime,
    pub reason: ReinitializationReason,
    /// Monotonic navigation generation declared by the artifact.
    pub generation: u32,
    /// Versioned canonical restart-recipe schema.
    pub input_schema: u16,
    /// Identity of the exact canonical restart-recipe bytes validated by the
    /// source adapter.
    pub input_digest: ContentDigestV1,
    /// Effective immutable engine configuration after the restart.
    pub configuration_digest: ContentDigestV1,
    /// Decoded complete construction recipe. The source adapter must verify
    /// that its canonical bytes hash to `input_digest` before emitting it.
    pub input: CapturedReinitializationInputV2,
}

impl ReinitializationEvidence {
    pub(crate) fn validate_for_contract(
        self,
        contract: CapturedReplayContract,
    ) -> Result<(), ProcessError> {
        if self.generation == 0
            || self.input_schema != CAPTURED_REINITIALIZATION_SCHEMA_V2
            || self.input_digest.is_zero()
            || self.configuration_digest != contract.configuration_digest
        {
            return Err(ProcessError::IncompleteEvidence);
        }
        self.input.validate_for_contract(contract)
    }
}

/// Immutable effective-value link for a recorded control change.
///
/// The artifact adapter validates that `next_digest` identifies the embedded
/// canonical definition bytes. Captured replay accepts a change only at a
/// span boundary, before any live call, and only when it resolves to the
/// configuration named by that span's replay contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControlChangeEvidence {
    pub at: SessionTime,
    pub generation: u32,
    pub previous_digest: ContentDigestV1,
    pub next_digest: ContentDigestV1,
}

/// Digest-bound metadata for a complete private live-state replay seed.
///
/// State bytes remain owned and validated by the artifact adapter. The core
/// rejects every unimplemented `state_schema`; carrying the complete identity
/// here prevents an opaque marker from accidentally granting equivalence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReplaySeedEvidence {
    pub at: SessionTime,
    pub frontier: SessionTime,
    pub navigation_watermark: SessionTime,
    pub metric_watermark: Option<SessionTime>,
    pub configuration_generation: u32,
    pub anchor_generation: u32,
    pub clock_segment: ClockSegmentId,
    pub run_namespace: u64,
    pub next_event_allocation: u64,
    pub profile_digest: ContentDigestV1,
    pub configuration_digest: ContentDigestV1,
    pub metric_state_digest: ContentDigestV1,
    pub state_schema: u16,
    pub state_digest: ContentDigestV1,
}

impl ReplaySeedEvidence {
    pub(crate) fn validate_identity(self) -> Result<(), ProcessError> {
        if self.state_schema == 0
            || self.state_digest.is_zero()
            || self.profile_digest.is_zero()
            || self.configuration_digest.is_zero()
            || self.metric_state_digest.is_zero()
            || self.frontier > self.at
            || self.navigation_watermark > self.frontier
            || self.metric_watermark.is_some_and(|time| time > self.at)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        Ok(())
    }
}

/// Fixed fitted mean and covariance for one immutable affine clock segment.
///
/// `cross_covariance_with_prior` is a row-major `2 x prior_dimension` block,
/// where the prior ordering is the calibration block followed by clock models
/// already declared in the stream.  This represents declared correlations
/// without exposing an estimator matrix type.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockModelEvidence<'a> {
    pub model: ClockModelId,
    pub segment: ClockSegmentId,
    pub validity: TimeSpan,
    pub reference_time: SessionTime,
    pub offset_ns: f64,
    pub fractional_drift: f64,
    /// `(offset variance ns^2, offset/drift covariance ns, drift variance)`.
    pub covariance_upper: [f64; 3],
    pub cross_covariance_with_prior: &'a [f64],
}

impl ClockModelEvidence<'_> {
    pub(crate) fn validate(self, prior_dimension: usize) -> Result<(), ProcessError> {
        let expected = prior_dimension
            .checked_mul(2)
            .ok_or(ProcessError::ResourceLimit)?;
        if self.cross_covariance_with_prior.len() != expected
            || !self.offset_ns.is_finite()
            || !self.fractional_drift.is_finite()
            || !self.validity.contains(self.reference_time)
            || !self.covariance_upper.iter().all(|value| value.is_finite())
            || !self
                .cross_covariance_with_prior
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(ProcessError::InvalidEvidence);
        }
        let [offset_variance, cross, drift_variance] = self.covariance_upper;
        if !crate::time::covariance_2x2_is_psd(offset_variance, cross, drift_variance) {
            return Err(ProcessError::InvalidEvidence);
        }
        Ok(())
    }
}

/// Complete, checksum-validated termination of a semantic source.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvidenceEnd {
    pub span: TimeSpan,
    pub terminal_record_sequence: u64,
    pub source_logical_digest: ContentDigestV1,
}

/// One borrowed semantic record emitted by an evidence adapter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EvidenceEvent<'a> {
    /// Integrity-bound artifact record that has no estimator-facing semantic
    /// payload. Adapters emit this marker instead of dropping manifests,
    /// native evidence, summaries, and unknown optional records, preserving
    /// canonical record-sequence continuity and restart sealing without
    /// exposing log-codec types through this port.
    Opaque {
        record_sequence: u64,
        semantic_digest: ContentDigestV1,
    },
    ReplayContract {
        record_sequence: u64,
        contract: CapturedReplayContract,
    },
    Observation {
        record_sequence: u64,
        observation: &'a LiveObservation,
    },
    ClockModel {
        record_sequence: u64,
        model: ClockModelEvidence<'a>,
    },
    Gap {
        record_sequence: u64,
        gap: EvidenceGap,
    },
    Reinitialize {
        record_sequence: u64,
        evidence: ReinitializationEvidence,
    },
    /// Immutable control provenance. V1 accepts this only before the selected
    /// span's replay contract and requires the digest chain to resolve to the
    /// contract's complete effective configuration; it is never applied to a
    /// running estimator.
    ControlChange {
        record_sequence: u64,
        change: ControlChangeEvidence,
    },
    /// An opaque private-state seed. Its identity can be validated, but V1 has
    /// no private state-import schema and therefore rejects it fail-closed.
    ReplaySeed {
        record_sequence: u64,
        seed: ReplaySeedEvidence,
    },
    /// Private-state checkpoint identity used only for comparison. It cannot
    /// establish a replay origin in V1.
    LiveCheckpoint {
        record_sequence: u64,
        state_digest: ContentDigestV1,
    },
    LiveStepCall {
        record_sequence: u64,
        call: CapturedLiveStepCall,
    },
    LiveFinishCall {
        record_sequence: u64,
        call: CapturedLiveFinishCall,
    },
    End {
        record_sequence: u64,
        end: EvidenceEnd,
    },
}

impl EvidenceEvent<'_> {
    /// Returns canonical artifact-record order.
    #[must_use]
    pub const fn record_sequence(self) -> u64 {
        match self {
            Self::Opaque {
                record_sequence, ..
            }
            | Self::ReplayContract {
                record_sequence, ..
            }
            | Self::Observation {
                record_sequence, ..
            }
            | Self::ClockModel {
                record_sequence, ..
            }
            | Self::Gap {
                record_sequence, ..
            }
            | Self::Reinitialize {
                record_sequence, ..
            }
            | Self::ControlChange {
                record_sequence, ..
            }
            | Self::ReplaySeed {
                record_sequence, ..
            }
            | Self::LiveCheckpoint {
                record_sequence, ..
            }
            | Self::LiveStepCall {
                record_sequence, ..
            }
            | Self::LiveFinishCall {
                record_sequence, ..
            }
            | Self::End {
                record_sequence, ..
            } => record_sequence,
        }
    }
}

/// Same-build semantic seal proving that every source restart produced the
/// exact event stream scanned during preflight.
///
/// This seal is separate from the persisted canonical logical digest. The
/// artifact adapter verifies that identity; the engine uses this
/// allocation-free formatter sink to detect mutation between numerical
/// passes.
pub(crate) struct SemanticStreamSeal {
    hash: Sha256,
}

impl SemanticStreamSeal {
    pub(crate) fn new() -> Self {
        let mut hash = Sha256::new();
        hash.update(b"aevia.semantic-evidence-restart-seal.v1\0");
        Self { hash }
    }

    pub(crate) fn observe(&mut self, event: EvidenceEvent<'_>) -> Result<(), ProcessError> {
        struct HashWriter<'a>(&'a mut Sha256);

        impl fmt::Write for HashWriter<'_> {
            fn write_str(&mut self, value: &str) -> fmt::Result {
                self.0.update(value.as_bytes());
                Ok(())
            }
        }

        let mut writer = HashWriter(&mut self.hash);
        writer
            .write_fmt(format_args!("{event:?}\n"))
            .map_err(|_| ProcessError::InvalidEvidence)
    }

    pub(crate) fn finish(self) -> ContentDigestV1 {
        ContentDigestV1::from_bytes(self.hash.finalize().into())
    }
}

/// Restartable borrowed semantic input.  Returned events may borrow decoder
/// storage only until the next method call on the source.
pub trait EvidenceSource {
    fn manifest(&self) -> EvidenceManifest;
    /// Restarts at the first semantic record of the same immutable evidence.
    ///
    /// # Errors
    ///
    /// Returns a source or integrity error when the adapter cannot recreate
    /// the preflighted stream.
    fn restart(&mut self) -> Result<(), ProcessError>;
    /// Borrows the next semantic record, or `None` after the complete stream.
    ///
    /// # Errors
    ///
    /// Returns a source, decoding, or integrity error. The runner still
    /// independently validates ordering and the terminal record.
    fn next(&mut self) -> Result<Option<EvidenceEvent<'_>>, ProcessError>;
}

/// Zero-copy in-memory evidence adapter used by fixtures and small host jobs.
pub struct SliceEvidenceSource<'a> {
    manifest: EvidenceManifest,
    events: &'a [EvidenceEvent<'a>],
    cursor: usize,
}

impl<'a> SliceEvidenceSource<'a> {
    /// Constructs an immutable restartable source.  Full stream validation is
    /// deliberately performed by the engine runner, not trusted here.
    #[must_use]
    pub const fn new(manifest: EvidenceManifest, events: &'a [EvidenceEvent<'a>]) -> Self {
        Self {
            manifest,
            events,
            cursor: 0,
        }
    }
}

impl EvidenceSource for SliceEvidenceSource<'_> {
    fn manifest(&self) -> EvidenceManifest {
        self.manifest
    }

    fn restart(&mut self) -> Result<(), ProcessError> {
        self.cursor = 0;
        Ok(())
    }

    fn next(&mut self) -> Result<Option<EvidenceEvent<'_>>, ProcessError> {
        let event = self.events.get(self.cursor).copied();
        self.cursor = self.cursor.saturating_add(usize::from(event.is_some()));
        Ok(event)
    }
}

/// Immutable identity supplied before any result records are written.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResultDescriptor<'a> {
    /// Complete immutable result provenance. The sink must serialize/copy the
    /// borrowed lists during `begin`; they remain owned by the prepared run.
    pub provenance: ResultProvenance<'a>,
    /// Trajectory revision used by every following state/metric record.
    pub trajectory_revision: TrajectoryRevision,
}

/// Solver-independent semantic state published to a result artifact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothedStateRecord {
    pub time: SessionTime,
    pub position_ecef: EcefPosition,
    pub velocity_ecef: EcefVelocity,
    pub orientation_ecef_from_body: OrientationEcefFromBody,
    pub specific_force_body: BodyVector,
    pub covariance: KinematicCovariance,
    pub quality: EstimateQuality,
    pub observability: ObservabilityReport,
}

/// End-of-result integrity summary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResultEnd {
    pub state_count: u64,
    pub objective: f64,
    pub attempted_ieks_passes: u8,
    pub accepted_ieks_passes: u8,
    pub diagnostics: DiagnosticCounts,
}

/// Borrowed semantic output written within one result transaction.
#[derive(Clone, Copy, Debug)]
pub enum ResultRecord<'a> {
    State(&'a SmoothedStateRecord),
    Metrics(&'a MetricResults),
    End(ResultEnd),
}

/// Solver-independent upper bounds for one complete result transaction.
///
/// A result always contains one descriptor (including all borrowed
/// provenance lists), one framed metric record, and one end record. State and
/// metric-result counts vary by run and are bounded here. These are semantic
/// counts, not byte-size estimates; only a concrete sink knows its encoding,
/// framing, and staging representation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResultRecordBounds {
    maximum_state_records: u64,
    maximum_metric_results: u64,
}

impl ResultRecordBounds {
    /// Creates a complete logical transaction bound.
    ///
    /// # Errors
    ///
    /// Returns [`ProcessError::ResourceLimit`] for an empty state capacity or
    /// when the complete logical record/result count cannot be represented.
    pub fn new(
        maximum_state_records: u64,
        maximum_metric_results: u64,
    ) -> Result<Self, ProcessError> {
        if maximum_state_records == 0
            || maximum_state_records
                .checked_add(maximum_metric_results)
                .and_then(|count| count.checked_add(3))
                .is_none()
        {
            return Err(ProcessError::ResourceLimit);
        }
        Ok(Self {
            maximum_state_records,
            maximum_metric_results,
        })
    }

    /// The single immutable descriptor/provenance record.
    #[must_use]
    pub const fn descriptor_records(self) -> u8 {
        1
    }

    #[must_use]
    pub const fn maximum_state_records(self) -> u64 {
        self.maximum_state_records
    }

    /// The single framed metric record, including an empty result set.
    #[must_use]
    pub const fn metric_frames(self) -> u8 {
        1
    }

    #[must_use]
    pub const fn maximum_metric_results(self) -> u64 {
        self.maximum_metric_results
    }

    /// The single mandatory transaction-integrity end record.
    #[must_use]
    pub const fn end_records(self) -> u8 {
        1
    }
}

/// Complete input to a result adapter's byte-accurate staging preflight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResultSinkPreflight<'a> {
    descriptor: ResultDescriptor<'a>,
    records: ResultRecordBounds,
    output_byte_ceiling: NonZeroU64,
}

impl<'a> ResultSinkPreflight<'a> {
    fn new(
        descriptor: ResultDescriptor<'a>,
        records: ResultRecordBounds,
        output_byte_ceiling: u64,
    ) -> Result<Self, ProcessError> {
        let output_byte_ceiling =
            NonZeroU64::new(output_byte_ceiling).ok_or(ProcessError::ResourceLimit)?;
        Ok(Self {
            descriptor,
            records,
            output_byte_ceiling,
        })
    }

    #[must_use]
    pub const fn descriptor(self) -> ResultDescriptor<'a> {
        self.descriptor
    }

    #[must_use]
    pub const fn records(self) -> ResultRecordBounds {
        self.records
    }

    #[must_use]
    pub const fn output_byte_ceiling(self) -> u64 {
        self.output_byte_ceiling.get()
    }

    /// Attests the adapter's exact maximum encoded and framed transaction
    /// size after it has also established sufficient unpublished staging
    /// capacity.
    ///
    /// # Errors
    ///
    /// Returns [`ProcessError::ResourceLimit`] for a zero byte result or when
    /// the complete staged transaction exceeds the caller's hard ceiling.
    /// Adapters must likewise return `ResourceLimit` before calling this
    /// method if checked size arithmetic overflows or their staging capacity
    /// is smaller than the computed transaction size.
    pub fn attest(
        self,
        exact_transaction_bytes: u64,
    ) -> Result<ResultSinkAttestation<'a>, ProcessError> {
        let exact_transaction_bytes =
            NonZeroU64::new(exact_transaction_bytes).ok_or(ProcessError::ResourceLimit)?;
        if exact_transaction_bytes > self.output_byte_ceiling {
            return Err(ProcessError::ResourceLimit);
        }
        Ok(ResultSinkAttestation {
            exact_transaction_bytes,
            request: self,
        })
    }
}

/// Byte-accurate capacity attestation returned by a concrete result adapter.
///
/// This value can only be constructed from the matching
/// [`ResultSinkPreflight`], so zero-byte and over-ceiling attestations cannot
/// reach `begin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResultSinkAttestation<'a> {
    exact_transaction_bytes: NonZeroU64,
    request: ResultSinkPreflight<'a>,
}

impl ResultSinkAttestation<'_> {
    #[must_use]
    pub const fn exact_transaction_bytes(self) -> u64 {
        self.exact_transaction_bytes.get()
    }

    fn matches(self, request: ResultSinkPreflight<'_>) -> bool {
        self.request == request
    }
}

/// Transactional output publication.  `commit` is the only operation that
/// makes a candidate visible; `abort` must be idempotent.
pub trait ResultSink {
    /// Computes the exact maximum bytes needed by this adapter's encoding and
    /// framing for the declared complete transaction and establishes that its
    /// unpublished staging capacity can hold them.
    ///
    /// The calculation must include the descriptor and every borrowed
    /// provenance list, every bounded state record, one metric frame with up
    /// to the bounded number of results, one end record, and all adapter
    /// framing/staging overhead. It must use checked arithmetic. A missing or
    /// zero bound, arithmetic overflow, insufficient staging capacity, or a
    /// result above `request.output_byte_ceiling()` must fail with
    /// [`ProcessError::ResourceLimit`]. `begin` is not called after failure.
    ///
    /// If capacity becomes unavailable after a successful attestation, the
    /// adapter must return `ResourceLimit` from `begin`, `write`, or `commit`
    /// before publishing anything. The runner then calls [`Self::abort`].
    ///
    /// # Errors
    ///
    /// Returns `ResourceLimit` when the complete encoded/framed/staged
    /// transaction cannot be bounded or guaranteed within the request.
    fn preflight<'a>(
        &mut self,
        request: ResultSinkPreflight<'a>,
    ) -> Result<ResultSinkAttestation<'a>, ProcessError>;

    /// Starts an unpublished candidate transaction.
    ///
    /// # Errors
    ///
    /// Returns a sink error if staging cannot begin.
    fn begin(&mut self, descriptor: ResultDescriptor<'_>) -> Result<(), ProcessError>;
    /// Stages one ordered semantic output record.
    ///
    /// # Errors
    ///
    /// Returns a sink error if the record cannot be staged.
    fn write(&mut self, record: ResultRecord<'_>) -> Result<(), ProcessError>;
    /// Atomically publishes the complete staged candidate.
    ///
    /// # Errors
    ///
    /// Returns a sink error if publication fails; the runner will abort the
    /// transaction during unwinding.
    fn commit(&mut self) -> Result<(), ProcessError>;
    fn abort(&mut self);
}

/// Holds a sink-specific capacity attestation without starting publication.
/// A failed numerical solve drops this guard and releases the unpublished
/// reservation before `begin` can be called.
pub(crate) struct SinkPreflightReservation<'sink, 'descriptor, S: ResultSink> {
    sink: Option<&'sink mut S>,
    request: ResultSinkPreflight<'descriptor>,
}

impl<'sink, 'descriptor, S: ResultSink> SinkPreflightReservation<'sink, 'descriptor, S> {
    pub(crate) fn preflight(
        sink: &'sink mut S,
        descriptor: ResultDescriptor<'descriptor>,
        records: ResultRecordBounds,
        output_byte_ceiling: u64,
    ) -> Result<Self, ProcessError> {
        let request = ResultSinkPreflight::new(descriptor, records, output_byte_ceiling)?;
        let attestation = match sink.preflight(request) {
            Ok(attestation) => attestation,
            Err(error) => {
                // Preflight may have reserved adapter-private staging. Abort
                // is idempotent and must release any such partial reservation.
                sink.abort();
                return Err(error);
            }
        };
        if !attestation.matches(request) {
            sink.abort();
            return Err(ProcessError::ResourceLimit);
        }
        Ok(Self {
            sink: Some(sink),
            request,
        })
    }

    pub(crate) fn begin(mut self) -> Result<SinkTransaction<'sink, S>, ProcessError> {
        let request = self.request;
        let sink = self.sink.take().ok_or(ProcessError::SinkFailure)?;
        if let Err(error) = sink.begin(request.descriptor) {
            // `abort` is required to be idempotent, so calling it even when an
            // adapter failed part-way through begin is the safest contract.
            sink.abort();
            return Err(error);
        }
        Ok(SinkTransaction {
            sink,
            active: true,
            records: request.records,
            state_records: 0,
            phase: TransactionPhase::States,
        })
    }
}

impl<S: ResultSink> Drop for SinkPreflightReservation<'_, '_, S> {
    fn drop(&mut self) {
        if let Some(sink) = self.sink.take() {
            sink.abort();
        }
    }
}

pub(crate) struct SinkTransaction<'a, S: ResultSink> {
    sink: &'a mut S,
    active: bool,
    records: ResultRecordBounds,
    state_records: u64,
    phase: TransactionPhase,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TransactionPhase {
    States,
    Metrics,
    End,
}

impl<'a, S: ResultSink> SinkTransaction<'a, S> {
    pub(crate) fn begin(
        sink: &'a mut S,
        descriptor: ResultDescriptor<'_>,
        records: ResultRecordBounds,
        output_byte_ceiling: u64,
    ) -> Result<Self, ProcessError> {
        SinkPreflightReservation::preflight(sink, descriptor, records, output_byte_ceiling)?.begin()
    }

    pub(crate) fn write(&mut self, record: ResultRecord<'_>) -> Result<(), ProcessError> {
        match record {
            ResultRecord::State(_) => {
                if self.phase != TransactionPhase::States {
                    return Err(ProcessError::SinkFailure);
                }
                if self.state_records >= self.records.maximum_state_records {
                    return Err(ProcessError::ResourceLimit);
                }
                self.sink.write(record)?;
                self.state_records = self
                    .state_records
                    .checked_add(1)
                    .ok_or(ProcessError::ResourceLimit)?;
            }
            ResultRecord::Metrics(results) => {
                if self.phase != TransactionPhase::States {
                    return Err(ProcessError::SinkFailure);
                }
                let result_count =
                    u64::try_from(results.len()).map_err(|_| ProcessError::ResourceLimit)?;
                if result_count > self.records.maximum_metric_results {
                    return Err(ProcessError::ResourceLimit);
                }
                self.sink.write(record)?;
                self.phase = TransactionPhase::Metrics;
            }
            ResultRecord::End(end) => {
                if self.phase != TransactionPhase::Metrics || end.state_count != self.state_records
                {
                    return Err(ProcessError::SinkFailure);
                }
                self.sink.write(record)?;
                self.phase = TransactionPhase::End;
            }
        }
        Ok(())
    }

    pub(crate) fn commit(mut self) -> Result<(), ProcessError> {
        if self.phase != TransactionPhase::End {
            return Err(ProcessError::SinkFailure);
        }
        match self.sink.commit() {
            Ok(()) => {
                self.active = false;
                Ok(())
            }
            Err(error) => Err(error),
        }
    }
}

impl<S: ResultSink> Drop for SinkTransaction<'_, S> {
    fn drop(&mut self) {
        if self.active {
            self.sink.abort();
            self.active = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::{ProcessingLevel, ProcessingPolicy},
        ids::{BackendVersionId, ResultRevisionId, TrajectoryRevision},
        provenance::{
            BackendProvenance, Capabilities, Capability, ProcessingAttempt,
            ProcessingAttemptOutcome,
        },
    };

    fn descriptor() -> ResultDescriptor<'static> {
        static ATTEMPTS: [ProcessingAttempt; 1] = [ProcessingAttempt {
            level: ProcessingLevel::OfflineSmooth,
            ordinal: 0,
            outcome: ProcessingAttemptOutcome::Succeeded,
        }];
        ResultDescriptor {
            provenance: ResultProvenance {
                result_revision: ResultRevisionId::new(1),
                source_session: SessionId::from_bytes([1; 16]),
                source_span: TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(1))
                    .unwrap(),
                source_digest: ContentDigestV1::from_bytes([2; 32]),
                normalization_digest: ContentDigestV1::from_bytes([3; 32]),
                configuration_digest: ContentDigestV1::from_bytes([4; 32]),
                installation_digest: ContentDigestV1::from_bytes([5; 32]),
                calibration_revision: crate::ids::CalibrationRevision::new(1),
                calibration_digest: ContentDigestV1::from_bytes([6; 32]),
                uncertainty_digest: ContentDigestV1::from_bytes([7; 32]),
                metric_plan_digest: ContentDigestV1::from_bytes([8; 32]),
                requested_policy: ProcessingPolicy::require(ProcessingLevel::OfflineSmooth),
                actual_backend: BackendProvenance {
                    level: ProcessingLevel::OfflineSmooth,
                    version: BackendVersionId::new(1),
                    native_source_digest: None,
                },
                attempts: &ATTEMPTS,
                parents: &[],
                external_inputs: &[],
                capabilities: Capabilities::one(Capability::OfflineSmooth),
            },
            trajectory_revision: TrajectoryRevision::new(1),
        }
    }

    struct RecordingSink {
        capacity_bytes: u64,
        fail_begin: bool,
        fail_write: bool,
        stale_attestation: bool,
        resource_limit_on_write: Option<u32>,
        preflights: u32,
        began: u32,
        writes: u32,
        commits: u32,
        aborts: u32,
        staged_writes: u32,
        visible_writes: u32,
        reserved: bool,
        active: bool,
        preflight_backend: Option<ProcessingLevel>,
        attested_bytes: u64,
    }

    impl Default for RecordingSink {
        fn default() -> Self {
            Self {
                capacity_bytes: u64::MAX,
                fail_begin: false,
                fail_write: false,
                stale_attestation: false,
                resource_limit_on_write: None,
                preflights: 0,
                began: 0,
                writes: 0,
                commits: 0,
                aborts: 0,
                staged_writes: 0,
                visible_writes: 0,
                reserved: false,
                active: false,
                preflight_backend: None,
                attested_bytes: 0,
            }
        }
    }

    fn mock_transaction_bytes(request: ResultSinkPreflight<'_>) -> Result<u64, ProcessError> {
        // This adapter's deliberately tiny test encoding uses eight-byte
        // frames and four bytes for each variable provenance/metric entry.
        // It is sink-specific; production adapters must price their own bytes.
        let descriptor = request.descriptor();
        let provenance_entries = descriptor
            .provenance
            .attempts
            .len()
            .checked_add(descriptor.provenance.parents.len())
            .and_then(|count| count.checked_add(descriptor.provenance.external_inputs.len()))
            .and_then(|count| u64::try_from(count).ok())
            .ok_or(ProcessError::ResourceLimit)?;
        let records = request.records();
        u64::from(records.descriptor_records())
            .checked_mul(8)
            .and_then(|bytes| {
                provenance_entries
                    .checked_mul(4)
                    .and_then(|entries| bytes.checked_add(entries))
            })
            .and_then(|bytes| {
                records
                    .maximum_state_records()
                    .checked_mul(8)
                    .and_then(|states| bytes.checked_add(states))
            })
            .and_then(|bytes| bytes.checked_add(u64::from(records.metric_frames()) * 8))
            .and_then(|bytes| {
                records
                    .maximum_metric_results()
                    .checked_mul(4)
                    .and_then(|metrics| bytes.checked_add(metrics))
            })
            .and_then(|bytes| bytes.checked_add(u64::from(records.end_records()) * 8))
            .ok_or(ProcessError::ResourceLimit)
    }

    impl ResultSink for RecordingSink {
        fn preflight<'a>(
            &mut self,
            request: ResultSinkPreflight<'a>,
        ) -> Result<ResultSinkAttestation<'a>, ProcessError> {
            self.preflights += 1;
            self.preflight_backend = Some(request.descriptor().provenance.actual_backend.level);
            let exact_bytes = mock_transaction_bytes(request)?;
            if exact_bytes > self.capacity_bytes {
                return Err(ProcessError::ResourceLimit);
            }
            let attestation = if self.stale_attestation {
                let descriptor = request.descriptor();
                let stale_descriptor = ResultDescriptor {
                    provenance: ResultProvenance {
                        source_digest: ContentDigestV1::from_bytes([99; 32]),
                        ..descriptor.provenance
                    },
                    ..descriptor
                };
                ResultSinkPreflight::new(
                    stale_descriptor,
                    request.records(),
                    request.output_byte_ceiling(),
                )?
                .attest(exact_bytes)?
            } else {
                request.attest(exact_bytes)?
            };
            self.attested_bytes = attestation.exact_transaction_bytes();
            self.reserved = true;
            Ok(attestation)
        }

        fn begin(&mut self, _: ResultDescriptor<'_>) -> Result<(), ProcessError> {
            self.began += 1;
            if self.fail_begin {
                Err(ProcessError::SinkFailure)
            } else {
                assert!(self.reserved);
                self.active = true;
                Ok(())
            }
        }

        fn write(&mut self, _: ResultRecord<'_>) -> Result<(), ProcessError> {
            self.writes += 1;
            assert!(self.active);
            if self.resource_limit_on_write == Some(self.writes) {
                return Err(ProcessError::ResourceLimit);
            }
            if self.fail_write {
                Err(ProcessError::SinkFailure)
            } else {
                self.staged_writes += 1;
                Ok(())
            }
        }

        fn commit(&mut self) -> Result<(), ProcessError> {
            assert!(self.active);
            self.commits += 1;
            self.visible_writes = self.staged_writes;
            self.staged_writes = 0;
            self.active = false;
            self.reserved = false;
            Ok(())
        }

        fn abort(&mut self) {
            if self.reserved || self.active {
                self.aborts += 1;
            }
            self.staged_writes = 0;
            self.active = false;
            self.reserved = false;
        }
    }

    fn bounds() -> ResultRecordBounds {
        ResultRecordBounds::new(2, 0).unwrap()
    }

    const OUTPUT_BYTE_CEILING: u64 = 1_024;

    #[test]
    fn failed_begin_is_aborted_even_when_adapter_partially_started() {
        let mut sink = RecordingSink {
            fail_begin: true,
            ..RecordingSink::default()
        };
        assert!(matches!(
            SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING),
            Err(ProcessError::SinkFailure)
        ));
        assert_eq!(sink.preflights, 1);
        assert_eq!(sink.aborts, 1);
    }

    #[test]
    fn any_uncommitted_transaction_aborts_on_drop() {
        let mut sink = RecordingSink::default();
        {
            let transaction =
                SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING)
                    .unwrap();
            drop(transaction);
        }
        assert_eq!(sink.aborts, 1);
        assert_eq!(sink.commits, 0);
    }

    #[test]
    fn unused_preflight_reservation_aborts_before_begin() {
        let mut sink = RecordingSink::default();
        {
            let reservation = SinkPreflightReservation::preflight(
                &mut sink,
                descriptor(),
                bounds(),
                OUTPUT_BYTE_CEILING,
            )
            .unwrap();
            drop(reservation);
        }
        assert_eq!(sink.preflights, 1);
        assert_eq!(sink.began, 0);
        assert_eq!(sink.aborts, 1);
        assert_eq!(sink.visible_writes, 0);
    }

    #[test]
    fn successful_commit_never_aborts() {
        let mut sink = RecordingSink::default();
        let metrics = MetricResults::new();
        let mut transaction =
            SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING).unwrap();
        transaction.write(ResultRecord::Metrics(&metrics)).unwrap();
        transaction
            .write(ResultRecord::End(ResultEnd {
                state_count: 0,
                objective: 0.0,
                attempted_ieks_passes: 0,
                accepted_ieks_passes: 0,
                diagnostics: DiagnosticCounts::default(),
            }))
            .unwrap();
        transaction.commit().unwrap();
        assert_eq!(sink.preflights, 1);
        assert_eq!(sink.preflight_backend, Some(ProcessingLevel::OfflineSmooth));
        assert!(sink.attested_bytes > 0);
        assert_eq!(sink.commits, 1);
        assert_eq!(sink.aborts, 0);
        assert_eq!(sink.visible_writes, 2);
    }

    #[test]
    fn zero_overflow_and_over_ceiling_attestations_fail_closed() {
        assert_eq!(
            ResultRecordBounds::new(0, 0),
            Err(ProcessError::ResourceLimit)
        );
        assert_eq!(
            ResultRecordBounds::new(u64::MAX, 0),
            Err(ProcessError::ResourceLimit)
        );
        let request = ResultSinkPreflight::new(descriptor(), bounds(), 64).unwrap();
        assert_eq!(request.attest(0), Err(ProcessError::ResourceLimit));
        assert_eq!(request.attest(65), Err(ProcessError::ResourceLimit));
    }

    #[test]
    fn insufficient_sink_capacity_is_rejected_before_begin() {
        let mut sink = RecordingSink {
            capacity_bytes: 1,
            ..RecordingSink::default()
        };
        assert!(matches!(
            SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING),
            Err(ProcessError::ResourceLimit)
        ));
        assert_eq!(sink.preflights, 1);
        assert_eq!(sink.began, 0);
        assert_eq!(sink.commits, 0);
        assert_eq!(sink.visible_writes, 0);
    }

    #[test]
    fn stale_descriptor_attestation_is_rejected_before_begin() {
        let mut sink = RecordingSink {
            stale_attestation: true,
            ..RecordingSink::default()
        };
        assert!(matches!(
            SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING),
            Err(ProcessError::ResourceLimit)
        ));
        assert_eq!(sink.preflights, 1);
        assert_eq!(sink.began, 0);
        assert_eq!(sink.commits, 0);
        assert_eq!(sink.visible_writes, 0);
    }

    #[test]
    fn capacity_loss_after_begin_aborts_without_visible_records() {
        let mut sink = RecordingSink {
            resource_limit_on_write: Some(1),
            ..RecordingSink::default()
        };
        let metrics = MetricResults::new();
        {
            let mut transaction =
                SinkTransaction::begin(&mut sink, descriptor(), bounds(), OUTPUT_BYTE_CEILING)
                    .unwrap();
            assert_eq!(
                transaction.write(ResultRecord::Metrics(&metrics)),
                Err(ProcessError::ResourceLimit)
            );
        }
        assert_eq!(sink.began, 1);
        assert_eq!(sink.commits, 0);
        assert_eq!(sink.aborts, 1);
        assert_eq!(sink.staged_writes, 0);
        assert_eq!(sink.visible_writes, 0);
    }

    #[test]
    fn manifest_rejects_capability_claim_not_present_whole_source() {
        let span = TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(1)).unwrap();
        let manifest = EvidenceManifest {
            session_id: SessionId::from_bytes([1; 16]),
            source_logical_digest: ContentDigestV1::from_bytes([2; 32]),
            normalization_digest: ContentDigestV1::from_bytes([3; 32]),
            configuration_digest: ContentDigestV1::from_bytes([4; 32]),
            span_capabilities: SpanCapabilities {
                span,
                capabilities: Capabilities::one(Capability::Timing),
                terminal_record_sequence: 1,
                has_valid_end: true,
            },
            capabilities: Capabilities::NONE,
            restartable: true,
            estimated_event_count: Some(1),
            captured_replay: None,
        };
        assert_eq!(
            manifest.validate(),
            Err(ValidationError::IncompatibleDefinition)
        );

        let complete = Capabilities::one(Capability::Timing);
        let valid = EvidenceManifest {
            capabilities: complete,
            span_capabilities: SpanCapabilities {
                capabilities: complete,
                ..manifest.span_capabilities
            },
            ..manifest
        };
        assert_eq!(valid.validate(), Ok(valid));
        for invalid in [
            EvidenceManifest {
                source_logical_digest: ContentDigestV1::from_bytes([0; 32]),
                ..valid
            },
            EvidenceManifest {
                normalization_digest: ContentDigestV1::from_bytes([0; 32]),
                ..valid
            },
            EvidenceManifest {
                configuration_digest: ContentDigestV1::from_bytes([0; 32]),
                ..valid
            },
            EvidenceManifest {
                span_capabilities: SpanCapabilities {
                    has_valid_end: false,
                    ..valid.span_capabilities
                },
                ..valid
            },
        ] {
            assert_eq!(
                invalid.validate(),
                Err(ValidationError::IncompatibleDefinition)
            );
        }
    }

    #[test]
    fn semantic_restart_seal_changes_with_event_content() {
        let span = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(1)).unwrap();
        let event = EvidenceEvent::End {
            record_sequence: 7,
            end: EvidenceEnd {
                span,
                terminal_record_sequence: 7,
                source_logical_digest: ContentDigestV1::from_bytes([1; 32]),
            },
        };
        let mut first = SemanticStreamSeal::new();
        first.observe(event).unwrap();
        let first = first.finish();

        let mut identical = SemanticStreamSeal::new();
        identical.observe(event).unwrap();
        assert_eq!(first, identical.finish());

        let mut changed = SemanticStreamSeal::new();
        changed
            .observe(EvidenceEvent::End {
                record_sequence: 8,
                end: EvidenceEnd {
                    span,
                    terminal_record_sequence: 8,
                    source_logical_digest: ContentDigestV1::from_bytes([1; 32]),
                },
            })
            .unwrap();
        assert_ne!(first, changed.finish());
    }

    #[test]
    fn clock_model_evidence_covariance_is_scale_independent() {
        let evidence = ClockModelEvidence {
            model: ClockModelId::new(1),
            segment: ClockSegmentId::new(1),
            validity: TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(1)).unwrap(),
            reference_time: SessionTime::from_ns(0),
            offset_ns: 0.0,
            fractional_drift: 0.0,
            covariance_upper: [1.0e-150, 1.0e-150, 1.0e-150],
            cross_covariance_with_prior: &[],
        };
        assert_eq!(evidence.validate(0), Ok(()));

        let indefinite = ClockModelEvidence {
            covariance_upper: [1.0e-150, 2.0e-150, 1.0e-150],
            ..evidence
        };
        assert_eq!(indefinite.validate(0), Err(ProcessError::InvalidEvidence));

        let smallest = f64::from_bits(1);
        let underflow_indefinite = ClockModelEvidence {
            covariance_upper: [smallest, smallest * 2.0, smallest],
            ..evidence
        };
        assert_eq!(
            underflow_indefinite.validate(0),
            Err(ProcessError::InvalidEvidence)
        );

        let extreme_psd = ClockModelEvidence {
            covariance_upper: [f64::MAX, f64::MAX, f64::MAX],
            ..evidence
        };
        assert_eq!(extreme_psd.validate(0), Ok(()));
    }
}
