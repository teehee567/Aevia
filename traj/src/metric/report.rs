//! Typed metric reports, result storage, and live mutation values.

#[cfg(not(feature = "offline"))]
use super::MAX_METRIC_RESULTS;
use super::{
    MAX_METRIC_MUTATIONS_PER_STEP,
    definition::{DistanceQuantity, SkiState, SpeedQuantity},
};
use crate::{
    ids::{GateId, LiveResultId, MetricDefinitionId, ReferencePointId, TargetId},
    quality::{EstimateStage, FieldValue, Validity},
    time::{DurationNs, SessionTime, TimeSpan},
};
use core::fmt;
use heapless::Vec as FixedVec;

/// Distance integral and its numerical error estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceReport {
    pub definition: MetricDefinitionId,
    pub quantity: DistanceQuantity,
    pub reference_point: ReferencePointId,
    pub span: TimeSpan,
    pub metres: f64,
    pub numerical_error_m: f64,
    pub uncertainty_one_sigma_m: FieldValue<f64>,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// One accepted finite-gate crossing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateCrossingReport {
    pub definition: MetricDefinitionId,
    pub gate: GateId,
    pub time: SessionTime,
    pub time_one_sigma_s: FieldValue<f64>,
    pub normal_speed_mps: f64,
    pub crossing_speed: Option<(SpeedQuantity, FieldValue<f64>)>,
    /// One-sigma uncertainty of the named crossing speed, evaluated with the
    /// gate event's implicit-time sensitivity rather than as a fixed-time
    /// velocity marginal.
    pub crossing_speed_one_sigma_mps: Option<(SpeedQuantity, FieldValue<f64>)>,
    pub reference_point: ReferencePointId,
    pub occurrence: u32,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// A completed ordered lap or sector interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LapReport {
    pub definition: MetricDefinitionId,
    pub lap_index: u32,
    pub start_gate: GateId,
    pub end_gate: GateId,
    pub start: SessionTime,
    pub end: SessionTime,
    pub elapsed_seconds: f64,
    pub elapsed_one_sigma_s: FieldValue<f64>,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// A reached drag speed or distance target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DragTargetReport {
    pub definition: MetricDefinitionId,
    pub target: TargetId,
    pub launch_time: SessionTime,
    pub event_time: SessionTime,
    /// One-sigma uncertainty of the target event epoch itself.
    pub event_time_one_sigma_s: FieldValue<f64>,
    pub elapsed_seconds: f64,
    /// One-sigma uncertainty of `event_time - launch_time`. This requires the
    /// retained launch/event covariance; two marginal variances are never
    /// combined as though independent.
    pub elapsed_one_sigma_s: FieldValue<f64>,
    pub rollout_adjusted_seconds: FieldValue<f64>,
    pub terminal_speed: Option<(SpeedQuantity, f64)>,
    pub terminal_speed_one_sigma_mps: Option<(SpeedQuantity, FieldValue<f64>)>,
    pub terminal_speed_slope_mps2: FieldValue<f64>,
    pub reference_point: ReferencePointId,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// Whole-span activity totals.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActivityReport {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub span: TimeSpan,
    pub elapsed_seconds: f64,
    pub moving_seconds: f64,
    pub horizontal_distance_m: FieldValue<f64>,
    pub spatial_distance_m: FieldValue<f64>,
    pub ascent_m: FieldValue<f64>,
    pub descent_m: FieldValue<f64>,
    pub peak_speed: SpeedQuantity,
    pub peak_speed_mps: f64,
    pub peak_window: DurationNs,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// Time at which an ordered horizontal-path activity split was reached.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActivitySplitReport {
    pub definition: MetricDefinitionId,
    pub split_index: u16,
    pub horizontal_distance_m: f64,
    pub time: SessionTime,
    pub elapsed_seconds: f64,
    pub reference_point: ReferencePointId,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// One contiguous host ski-state interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkiSegmentReport {
    pub definition: MetricDefinitionId,
    pub state: SkiState,
    pub start: SessionTime,
    pub end: SessionTime,
    pub confidence: f64,
    pub reference_point: ReferencePointId,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// Summary counts for a ski session.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkiReport {
    pub definition: MetricDefinitionId,
    pub downhill_segments: u32,
    pub lift_segments: u32,
    pub ascent_segments: u32,
    pub downhill_seconds: f64,
    pub reference_point: ReferencePointId,
    pub stage: EstimateStage,
    pub validity: Validity,
}

/// One typed result. The ID is stable across later mutation/finalization.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MetricResultValue {
    Distance(DistanceReport),
    GateCrossing(GateCrossingReport),
    Lap(LapReport),
    DragTarget(DragTargetReport),
    Activity(ActivityReport),
    ActivitySplit(ActivitySplitReport),
    SkiSegment(SkiSegmentReport),
    Ski(SkiReport),
    /// This definition could not produce a trustworthy value, while the
    /// trajectory and unrelated definitions remained usable.
    Unavailable(MetricDefinitionDiagnostic),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricResult {
    pub id: LiveResultId,
    pub revision: u64,
    pub value: MetricResultValue,
}

/// Revision operation emitted by bounded live evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MetricMutation {
    Upsert {
        id: LiveResultId,
        revision: u64,
        value: MetricResultValue,
    },
    Withdraw {
        id: LiveResultId,
        revision: u64,
        reason: WithdrawalReason,
    },
    Finalize {
        id: LiveResultId,
        revision: u64,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WithdrawalReason {
    RetrospectiveRuleChanged,
    TrajectoryReinitialized,
    QualityInvalidated,
    OutputSuperseded,
}

/// Bounded mutations and the watermark governing their immutability.
#[derive(Clone, Debug, PartialEq)]
pub struct LiveMetricUpdate {
    pub(super) navigation_watermark: SessionTime,
    pub(super) metric_watermark: Option<SessionTime>,
    pub(super) mutations: FixedVec<MetricMutation, MAX_METRIC_MUTATIONS_PER_STEP>,
}

impl LiveMetricUpdate {
    /// Empty caller-owned output storage for [`super::LiveMetricTracker::update_into`].
    #[must_use]
    pub(crate) const fn empty() -> Self {
        Self {
            navigation_watermark: SessionTime::ZERO,
            metric_watermark: None,
            mutations: FixedVec::new(),
        }
    }

    pub(super) fn replace(
        &mut self,
        navigation_watermark: SessionTime,
        metric_watermark: Option<SessionTime>,
        mutations: &[MetricMutation],
    ) -> Result<(), MetricError> {
        if mutations.len() > MAX_METRIC_MUTATIONS_PER_STEP {
            return Err(MetricError::CapacityExceeded);
        }
        self.mutations.clear();
        for mutation in mutations {
            self.mutations
                .push(*mutation)
                .map_err(|_| MetricError::CapacityExceeded)?;
        }
        self.navigation_watermark = navigation_watermark;
        self.metric_watermark = metric_watermark;
        Ok(())
    }

    #[must_use]
    pub const fn navigation_watermark(&self) -> SessionTime {
        self.navigation_watermark
    }

    #[must_use]
    pub const fn metric_watermark(&self) -> Option<SessionTime> {
        self.metric_watermark
    }

    #[must_use]
    pub fn mutations(&self) -> &[MetricMutation] {
        self.mutations.as_slice()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum ResultKey {
    Distance(MetricDefinitionId),
    Gate(MetricDefinitionId, GateId, u32),
    Lap(MetricDefinitionId, u32),
    Drag(MetricDefinitionId, TargetId),
    Activity(MetricDefinitionId),
    ActivitySplit(MetricDefinitionId, u16),
    SkiSegment(MetricDefinitionId, SkiState, SessionTime),
    Ski(MetricDefinitionId),
    Unavailable(MetricDefinitionId),
}

impl MetricResultValue {
    pub(super) fn key(self) -> ResultKey {
        match self {
            Self::Distance(report) => ResultKey::Distance(report.definition),
            Self::GateCrossing(report) => {
                ResultKey::Gate(report.definition, report.gate, report.occurrence)
            }
            Self::Lap(report) => ResultKey::Lap(report.definition, report.lap_index),
            Self::DragTarget(report) => ResultKey::Drag(report.definition, report.target),
            Self::Activity(report) => ResultKey::Activity(report.definition),
            Self::ActivitySplit(report) => {
                ResultKey::ActivitySplit(report.definition, report.split_index)
            }
            Self::SkiSegment(report) => {
                ResultKey::SkiSegment(report.definition, report.state, report.start)
            }
            Self::Ski(report) => ResultKey::Ski(report.definition),
            Self::Unavailable(report) => ResultKey::Unavailable(report.definition),
        }
    }

    pub(super) fn event_time(self) -> Option<SessionTime> {
        match self {
            Self::GateCrossing(report) => Some(report.time),
            Self::Lap(report) => Some(report.end),
            Self::DragTarget(report) => Some(report.event_time),
            Self::ActivitySplit(report) => Some(report.time),
            Self::SkiSegment(report) => Some(report.end),
            Self::Distance(_) | Self::Activity(_) | Self::Ski(_) | Self::Unavailable(_) => None,
        }
    }

    pub(super) fn with_stage(mut self, stage: EstimateStage) -> Self {
        match &mut self {
            Self::Distance(report) => report.stage = stage,
            Self::GateCrossing(report) => report.stage = stage,
            Self::Lap(report) => report.stage = stage,
            Self::DragTarget(report) => report.stage = stage,
            Self::Activity(report) => report.stage = stage,
            Self::ActivitySplit(report) => report.stage = stage,
            Self::SkiSegment(report) => report.stage = stage,
            Self::Ski(report) => report.stage = stage,
            Self::Unavailable(report) => report.stage = stage,
        }
        self
    }

    pub(super) fn stage(self) -> EstimateStage {
        match self {
            Self::Distance(report) => report.stage,
            Self::GateCrossing(report) => report.stage,
            Self::Lap(report) => report.stage,
            Self::DragTarget(report) => report.stage,
            Self::Activity(report) => report.stage,
            Self::ActivitySplit(report) => report.stage,
            Self::SkiSegment(report) => report.stage,
            Self::Ski(report) => report.stage,
            Self::Unavailable(report) => report.stage,
        }
    }
}

/// Ordered result collection. Base builds use fixed storage; offline builds
/// may retain a full-session result set in an allocated vector. A freshly
/// constructed offline collection is an unbounded host convenience; replay
/// and other resource-qualified paths must call `try_prepare_bounded` before
/// evaluation.
#[derive(Clone, Debug, Default)]
pub struct MetricResults {
    #[cfg(feature = "offline")]
    pub(super) values: std::vec::Vec<MetricResult>,
    #[cfg(not(feature = "offline"))]
    pub(super) values: FixedVec<MetricResult, MAX_METRIC_RESULTS>,
    #[cfg(feature = "offline")]
    pub(super) maximum_len: Option<usize>,
}

impl MetricResults {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            values: new_result_storage(),
            #[cfg(feature = "offline")]
            maximum_len: None,
        }
    }

    /// Clears and fallibly reserves storage for at most `maximum_len` results.
    ///
    /// Once prepared, [`Self::push`] rejects the first result beyond that
    /// logical bound before `Vec::push` could attempt another allocation.
    #[cfg(feature = "offline")]
    pub(crate) fn try_prepare_bounded(&mut self, maximum_len: usize) -> Result<(), MetricError> {
        self.clear();
        // Fail closed if reservation itself fails. Keeping a zero logical
        // bound prevents accidental reuse through an infallible growth path.
        self.maximum_len = Some(0);
        self.values
            .try_reserve_exact(maximum_len)
            .map_err(|_| MetricError::CapacityExceeded)?;
        self.maximum_len = Some(maximum_len);
        Ok(())
    }

    #[must_use]
    pub fn as_slice(&self) -> &[MetricResult] {
        self.values.as_slice()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Definition-local unavailable/diagnostic outcomes. These are ordinary
    /// typed results with stable IDs and do not erase unrelated values.
    pub fn diagnostics(&self) -> impl Iterator<Item = MetricDefinitionDiagnostic> + '_ {
        self.values.iter().filter_map(|result| match result.value {
            MetricResultValue::Unavailable(diagnostic) => Some(diagnostic),
            _ => None,
        })
    }

    pub(super) fn clear(&mut self) {
        self.values.clear();
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn truncate_values(&mut self, len: usize) {
        self.values.truncate(len);
    }

    pub(super) fn push(&mut self, result: MetricResult) -> Result<(), MetricError> {
        #[cfg(feature = "offline")]
        {
            if self
                .maximum_len
                .is_some_and(|maximum_len| self.values.len() >= maximum_len)
            {
                return Err(MetricError::CapacityExceeded);
            }
            self.values.push(result);
            Ok(())
        }
        #[cfg(not(feature = "offline"))]
        {
            self.values
                .push(result)
                .map_err(|_| MetricError::CapacityExceeded)
        }
    }
}

impl PartialEq for MetricResults {
    fn eq(&self, other: &Self) -> bool {
        self.values == other.values
    }
}

#[cfg(feature = "offline")]
const fn new_result_storage() -> std::vec::Vec<MetricResult> {
    std::vec::Vec::new()
}

#[cfg(not(feature = "offline"))]
const fn new_result_storage() -> FixedVec<MetricResult, MAX_METRIC_RESULTS> {
    FixedVec::new()
}

/// Metric validation, numerical, or observability failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MetricError {
    InvalidDefinition,
    EmptyTrajectory,
    ReferencePointUnavailable,
    FrameMismatch,
    OutsideTrajectory,
    Unobservable,
    AmbiguousRoot,
    NumericalFailure,
    EvaluationBudgetExceeded,
    CapacityExceeded,
    Unsupported,
}

/// A non-fatal outcome scoped to one metric definition.
///
/// These reasons describe a definition that could not produce a trustworthy
/// value from an otherwise usable trajectory. Resource exhaustion, corrupt
/// backing data, and internal numerical failure remain fatal [`MetricError`]s.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MetricDefinitionDiagnosticReason {
    InvalidDefinition,
    ReferencePointUnavailable,
    FrameMismatch,
    Unobservable,
    Ambiguous,
    UnsupportedAtProcessingLevel,
    AttachmentModelUnavailable,
}

impl MetricDefinitionDiagnosticReason {
    #[cfg(any(test, feature = "offline"))]
    pub(super) const fn from_error(error: MetricError) -> Option<Self> {
        match error {
            MetricError::InvalidDefinition => Some(Self::InvalidDefinition),
            MetricError::ReferencePointUnavailable => Some(Self::ReferencePointUnavailable),
            MetricError::FrameMismatch => Some(Self::FrameMismatch),
            MetricError::Unobservable => Some(Self::Unobservable),
            MetricError::AmbiguousRoot => Some(Self::Ambiguous),
            MetricError::Unsupported => Some(Self::UnsupportedAtProcessingLevel),
            MetricError::EmptyTrajectory
            | MetricError::OutsideTrajectory
            | MetricError::NumericalFailure
            | MetricError::EvaluationBudgetExceeded
            | MetricError::CapacityExceeded => None,
        }
    }
}

/// Typed host-evaluation diagnostic for one immutable definition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MetricDefinitionDiagnostic {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub reason: MetricDefinitionDiagnosticReason,
    pub stage: EstimateStage,
    pub validity: Validity,
}

impl fmt::Display for MetricError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

#[cfg(feature = "offline")]
impl std::error::Error for MetricError {}
