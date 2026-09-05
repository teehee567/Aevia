//! Persistent definition dispatch and caller-owned live scratch storage.

use super::{
    MAX_METRIC_DEFINITIONS, MAX_METRIC_MUTATIONS_PER_STEP, MAX_METRIC_RESULTS,
    definition::MetricDefinition,
    live_activity::LiveActivityState,
    live_drag::LiveDragState,
    live_lap::LiveLapState,
    plan::LiveMetricPlan,
    report::{MetricMutation, MetricResults},
};
use crate::error::ValidationError;
use heapless::Vec as FixedVec;

#[derive(Clone, Debug, PartialEq)]
pub(super) enum LiveDefinitionState {
    Stateless,
    Activity(LiveActivityState),
    Lap(LiveLapState),
    Drag(LiveDragState),
}

/// Caller-owned temporary storage for one bounded live metric update.
///
/// Keeping candidates and speculative definition state outside the call frame
/// lets the ESP32-S31 place them in its cold workspace while the persistent
/// tracker remains unchanged until all output-capacity checks succeed.
pub(crate) struct LiveMetricScratch {
    pub(super) candidates: MetricResults,
    pub(super) activity_state: LiveActivityState,
    pub(super) lap_state: LiveLapState,
    pub(super) drag_state: LiveDragState,
    pub(super) preflight_seen: [bool; MAX_METRIC_RESULTS],
    pub(super) seen: [bool; MAX_METRIC_RESULTS],
    pub(super) mutations: FixedVec<MetricMutation, MAX_METRIC_MUTATIONS_PER_STEP>,
}

impl LiveMetricScratch {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            candidates: MetricResults::new(),
            activity_state: LiveActivityState::new(),
            lap_state: LiveLapState::new(),
            drag_state: LiveDragState::new(),
            preflight_seen: [false; MAX_METRIC_RESULTS],
            seen: [false; MAX_METRIC_RESULTS],
            mutations: FixedVec::new(),
        }
    }

    /// Clears transient ledgers without constructing a candidate collection
    /// by value. Each persistent definition is copied into its one matching
    /// scratch slot immediately before evaluation.
    pub(crate) fn configure(&mut self, plan: &LiveMetricPlan) -> Result<(), ValidationError> {
        if plan.plan.definitions.len() > MAX_METRIC_DEFINITIONS {
            return Err(ValidationError::CapacityExceeded);
        }
        self.candidates.clear();
        self.mutations.clear();
        self.preflight_seen.fill(false);
        self.seen.fill(false);
        Ok(())
    }
}

pub(super) fn empty_live_definition_state(definition: &MetricDefinition) -> LiveDefinitionState {
    match definition {
        MetricDefinition::Lap(_) => LiveDefinitionState::Lap(LiveLapState::new()),
        MetricDefinition::Drag(_) => LiveDefinitionState::Drag(LiveDragState::new()),
        MetricDefinition::Activity(_) => LiveDefinitionState::Activity(LiveActivityState::new()),
        MetricDefinition::Distance(_) | MetricDefinition::Ski(_) => LiveDefinitionState::Stateless,
    }
}
