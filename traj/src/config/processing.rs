//! Processing level selection, result requests, and host run control.

use super::{CalibrationPolicy, EngineConfig};
use crate::error::ValidationError;
use crate::ids::{ContentDigestV1, ResultRevisionId, TrajectoryRevision};
use crate::metric::MetricPlan;
use crate::provenance::{EvidenceLineage, MAX_EXTERNAL_INPUTS, MAX_PARENT_REVISIONS};
use crate::time::{SessionTime, TimeSpan};

/// Maximum processing candidates representable without allocation.
pub const MAX_PROCESSING_PREFERENCE_LEVELS: usize = 5;

/// Runtime estimator/refinement implementation selected behind one interface.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ProcessingLevel {
    /// Bounded live receiver-solution ESKF on V2 Mini.
    EmbeddedLive,
    /// Exact replay from captured normalized observations under an explicit
    /// comparison contract. Version one is same-build bit-exact only.
    CapturedReplay,
    /// Offline `f64` solution-level fixed-interval smoothing.
    OfflineSmooth,
    /// Optional workstation GTSAM graph smoothing.
    AdvancedGraph,
    /// Optional workstation raw tightly coupled RTK/INS.
    RawTight,
}

/// Numeric comparison contract attached to a captured live transcript.
///
/// A future cross-target mode must carry a typed per-field tolerance table and
/// preserve exact discrete dispositions, identities, and topology. It must be
/// added as a new variant and semantic encoding; unknown modes fail closed.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CapturedReplayComparison {
    /// Exact IEEE-bit identity using the same qualified engine/toolchain build.
    SameBuildBitExactV1,
}

impl ProcessingLevel {
    /// Returns whether this level runs as an offline batch workflow.
    #[must_use]
    pub const fn is_offline(self) -> bool {
        !matches!(self, Self::EmbeddedLive)
    }
}

/// Ordered, duplicate-free processing preference list.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProcessingPreference {
    levels: [ProcessingLevel; MAX_PROCESSING_PREFERENCE_LEVELS],
    length: u8,
}

impl ProcessingPreference {
    /// Validates and copies an ordered preference list.
    pub fn new(levels: &[ProcessingLevel]) -> Result<Self, ValidationError> {
        if levels.is_empty() || levels.len() > MAX_PROCESSING_PREFERENCE_LEVELS {
            return Err(ValidationError::CapacityExceeded);
        }
        let mut result = Self {
            levels: [ProcessingLevel::OfflineSmooth; MAX_PROCESSING_PREFERENCE_LEVELS],
            length: levels.len() as u8,
        };
        for (index, level) in levels.iter().copied().enumerate() {
            if !level.is_offline() || levels[..index].contains(&level) {
                return Err(ValidationError::IncompatibleDefinition);
            }
            result.levels[index] = level;
        }
        Ok(result)
    }

    /// Returns ordered candidate levels.
    #[must_use]
    pub fn levels(&self) -> &[ProcessingLevel] {
        &self.levels[..usize::from(self.length)]
    }
}

/// Required level or ordered fallback policy for host processing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcessingPolicy {
    /// Fail unless this exact level completes.
    Require(ProcessingLevel),
    /// Try qualified candidates in order, restarting from immutable evidence.
    BestQualified { preference: ProcessingPreference },
}

impl ProcessingPolicy {
    /// Constructs a required-level policy.
    #[must_use]
    pub const fn require(level: ProcessingLevel) -> Self {
        Self::Require(level)
    }

    /// Constructs and validates an ordered best-qualified policy.
    pub fn best_qualified(levels: &[ProcessingLevel]) -> Result<Self, ValidationError> {
        let preference = ProcessingPreference::new(levels)?;
        // Raw-tight remains explicitly required until a future qualification
        // revision changes this semantic rule.
        if preference.levels().contains(&ProcessingLevel::RawTight) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self::BestQualified { preference })
    }

    /// Returns the ordered levels considered by this policy.
    #[must_use]
    pub fn levels(&self) -> &[ProcessingLevel] {
        match self {
            Self::Require(level) => core::slice::from_ref(level),
            Self::BestQualified { preference } => preference.levels(),
        }
    }
}

/// Complete semantic request for captured replay or host refinement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProcessingResultSpec<'a> {
    /// Immutable identity of the result sidecar being created.
    pub result_revision: ResultRevisionId,
    /// Revision assigned to the returned trajectory.
    pub trajectory_revision: TrajectoryRevision,
    /// Canonical digest of the complete uncertainty-model set.
    pub uncertainty_digest: ContentDigestV1,
    /// Canonical digest of the full metric plan.
    pub metric_plan_digest: ContentDigestV1,
    /// Immutable result revisions used as parents.
    pub parents: &'a [ResultRevisionId],
    /// External base/correction/ephemeris inputs selected for this run.
    pub external_inputs: &'a [ContentDigestV1],
}

impl ProcessingResultSpec<'_> {
    fn validate(self) -> Result<Self, ValidationError> {
        if self.result_revision.get() == 0
            || self.trajectory_revision.get() == 0
            || self.uncertainty_digest.is_zero()
            || self.metric_plan_digest.is_zero()
            || self.parents.len() > MAX_PARENT_REVISIONS
            || self.external_inputs.len() > MAX_EXTERNAL_INPUTS
            || self.external_inputs.iter().any(|digest| digest.is_zero())
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        for (index, parent) in self.parents.iter().enumerate() {
            if parent.get() == 0 || self.parents[index + 1..].contains(parent) {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        Ok(self)
    }
}

/// Complete semantic request for captured replay or host refinement.
#[derive(Clone, Debug, PartialEq)]
pub struct ProcessingSpec<'a> {
    /// Immutable estimator/installation configuration.
    pub engine: EngineConfig<'a>,
    /// Requested checksum-valid session span.
    pub span: TimeSpan,
    /// Required or ordered fallback processing policy.
    pub policy: ProcessingPolicy,
    /// Captured/recomputed/raw evidence selected exactly once per source/span.
    pub evidence_lineage: EvidenceLineage<'a>,
    /// Fixed or advanced solve-for calibration policy.
    pub calibration_policy: CalibrationPolicy,
    /// Full semantic metric definitions to evaluate on the returned trajectory.
    pub metrics: MetricPlan,
    /// Complete immutable identity/provenance inputs for the result sidecar.
    pub result: ProcessingResultSpec<'a>,
}

impl ProcessingSpec<'_> {
    /// Validates host-level semantics before resource/evidence preflight.
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.engine.validate()?;
        self.result.validate()?;
        if self.policy.levels().iter().any(|level| !level.is_offline()) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if self.calibration_policy == CalibrationPolicy::RefineWithPriors
            && self.policy.levels().iter().any(|level| {
                matches!(
                    level,
                    ProcessingLevel::CapturedReplay | ProcessingLevel::OfflineSmooth
                )
            })
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(())
    }
}

/// Progress/cancellation behavior supplied to a host run without entering the
/// immutable semantic processing spec.
#[derive(Clone, Copy)]
pub struct RunControl<'a> {
    /// Called at deterministic processing work boundaries; returning `false`
    /// requests cancellation.
    pub continue_running: &'a dyn Fn(u64) -> bool,
    /// Called with monotonically increasing completed and total work units.
    pub progress: &'a dyn Fn(u64, u64),
}

impl core::fmt::Debug for RunControl<'_> {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str("RunControl { .. }")
    }
}

/// Returns whether an observation epoch lies inside a processing request.
#[must_use]
pub const fn processing_span_contains(spec: &ProcessingSpec<'_>, time: SessionTime) -> bool {
    spec.span.contains(time)
}
