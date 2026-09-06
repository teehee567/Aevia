//! Explicit metric definitions and evaluation over the continuous trajectory.
//!
//! There is intentionally no unqualified `speed` or `distance` in this
//! module.  Every definition selects a physical reference point and the exact
//! kinematic quantity it consumes.

mod activity;
mod definition;
mod distance;
mod evaluation;
mod events;
mod geometry;
mod identity;
mod live_activity;
mod live_drag;
mod live_lap;
mod live_state;
mod live_tracker;
mod numerical;
mod plan;
mod quality;
mod report;
#[cfg(feature = "offline")]
mod ski;
mod uncertainty;

#[cfg(test)]
mod numerical_tests;

pub use definition::{
    ActivityPlan, CrossingDirection, DistancePlan, DistanceQuantity, DragPlan, DragTarget,
    FiniteGate, GateSurveyUncertainty, LapPlan, LaunchRule, MetricDefinition, Rollout, SkiHmmModel,
    SkiPlan, SkiState, SpeedQuantity, TargetDirection,
};
pub(crate) use identity::encode_metric_value_identity_v1;
pub(crate) use live_state::LiveMetricScratch;
pub use live_tracker::LiveMetricTracker;
pub use numerical::MetricEvaluationLimits;
pub(crate) use numerical::{NumericalWorkBudget, isolate_polynomial_coefficients_with_budget};
pub use plan::{LiveMetricLimits, LiveMetricPlan, MetricPlan};
pub use report::{
    ActivityReport, ActivitySplitReport, DistanceReport, DragTargetReport, GateCrossingReport,
    LapReport, LiveMetricUpdate, MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason,
    MetricError, MetricMutation, MetricResult, MetricResultValue, MetricResults, SkiReport,
    SkiSegmentReport, WithdrawalReason,
};
#[cfg(test)]
pub(crate) use uncertainty::shared_gate_survey_time_covariance_s2;
#[cfg(any(test, feature = "offline"))]
pub(crate) use uncertainty::{EventTimeSensitivity, MetricUncertaintyProvider, StateSensitivity};

/// Maximum definitions in one immutable metric plan.
pub const MAX_METRIC_DEFINITIONS: usize = 64;

/// Maximum ordered gates in one lap plan.
pub const MAX_LAP_GATES: usize = 16;

/// Maximum drag targets in one drag plan.
pub const MAX_DRAG_TARGETS: usize = 24;

/// Maximum activity distance splits in one activity plan.
pub const MAX_ACTIVITY_SPLITS: usize = 32;

/// Maximum results retained by an allocator-free evaluation.
pub const MAX_METRIC_RESULTS: usize = 192;

/// Hard maximum mutations returned by one live engine step.
pub const MAX_METRIC_MUTATIONS_PER_STEP: usize = 16;
