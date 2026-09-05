//! Transactional live evaluation and stable result publication.

use super::{
    MAX_METRIC_DEFINITIONS, MAX_METRIC_MUTATIONS_PER_STEP, MAX_METRIC_RESULTS,
    definition::MetricDefinition,
    distance::live_distance_report,
    evaluation::Evaluator,
    live_activity::{advance_live_activity, copy_live_activity_state},
    live_drag::{advance_live_drag, copy_live_drag_state, live_drag_report},
    live_lap::{advance_live_lap, copy_live_lap_state},
    live_state::{LiveDefinitionState, LiveMetricScratch, empty_live_definition_state},
    numerical::{NumericalWork, NumericalWorkBudget, live_metric_evaluation_limits},
    plan::{LiveMetricPlan, live_active_candidate_bound},
    report::{
        LiveMetricUpdate, MetricError, MetricMutation, MetricResult, MetricResultValue, ResultKey,
        WithdrawalReason,
    },
    uncertainty::TrajectoryMarginalUncertainty,
};
use crate::{
    error::ValidationError,
    ids::LiveResultId,
    quality::EstimateStage,
    time::{SessionTime, SignedDurationNs},
    trajectory::Trajectory,
};
use heapless::Vec as FixedVec;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct TrackedResult {
    pub(super) key: ResultKey,
    pub(super) id: LiveResultId,
    pub(super) revision: u64,
    pub(super) value: MetricResultValue,
    /// Whether the metric-specific future-support rule has been satisfied.
    /// Watermark age alone cannot finalize a pending braking target.
    pub(super) finalization_ready: bool,
    pub(super) active: bool,
}

/// Fixed-capacity tracker that incrementally consumes corrected trajectory
/// suffixes and converts them into stable upsert/finalize/withdraw mutations.
/// It retains tombstones so an ID is never reused.
#[derive(Debug, PartialEq)]
pub struct LiveMetricTracker {
    pub(super) plan: LiveMetricPlan,
    pub(super) next_allocation: u64,
    pub(super) entries: FixedVec<TrackedResult, MAX_METRIC_RESULTS>,
    pub(super) definition_states: FixedVec<LiveDefinitionState, MAX_METRIC_DEFINITIONS>,
    /// Latest dense-trajectory endpoint incorporated into cumulative live
    /// totals. Keeping the cursor here prevents a rolling trajectory from
    /// making session distance/activity totals move backwards.
    pub(super) last_consumed_end: Option<SessionTime>,
    pub(super) terminal_endpoint_consumed: bool,
    /// While set, provisional results from the ended trajectory span are
    /// withdrawn in bounded batches before a new span may be evaluated.
    pub(super) pending_withdrawal_at: Option<SessionTime>,
    pub(super) pending_withdrawal_reason: Option<WithdrawalReason>,
}

impl LiveMetricTracker {
    /// Empty tracker storage suitable for static/PSRAM construction.
    #[must_use]
    pub(crate) const fn unconfigured() -> Self {
        Self {
            plan: LiveMetricPlan::placeholder(),
            next_allocation: 0,
            entries: FixedVec::new(),
            definition_states: FixedVec::new(),
            last_consumed_end: None,
            terminal_endpoint_consumed: false,
            pending_withdrawal_at: None,
            pending_withdrawal_reason: None,
        }
    }

    /// Resets this storage and copies one compiled plan directly into its
    /// fixed ledgers. No complete tracker value is returned or staged on the
    /// caller's stack.
    pub(crate) fn configure(&mut self, plan: &LiveMetricPlan) -> Result<(), ValidationError> {
        self.plan.plan.run_namespace = plan.plan.run_namespace;
        self.plan.plan.definitions.clear();
        for definition in &plan.plan.definitions {
            self.plan
                .plan
                .definitions
                .push(definition.clone())
                .map_err(|_| ValidationError::CapacityExceeded)?;
        }
        self.plan.maximum_lookahead = plan.maximum_lookahead;
        self.plan.limits = plan.limits;

        self.definition_states.clear();
        for definition in &plan.plan.definitions {
            self.definition_states
                .push(empty_live_definition_state(definition))
                .map_err(|_| ValidationError::CapacityExceeded)?;
        }
        self.next_allocation = 0;
        self.entries.clear();
        self.last_consumed_end = None;
        self.terminal_endpoint_consumed = false;
        self.pending_withdrawal_at = None;
        self.pending_withdrawal_reason = None;
        Ok(())
    }

    /// Ends the current metric span and schedules bounded withdrawals for
    /// every still-provisional result. Finalized results remain immutable;
    /// their local tombstones are discarded only after all withdrawals have
    /// been published. `next_allocation` deliberately remains monotonic so a
    /// result ID is never reused across navigation generations.
    pub(crate) fn begin_trajectory_reinitialization(&mut self, at: SessionTime) {
        self.pending_withdrawal_at = Some(
            self.pending_withdrawal_at
                .map_or(at, |pending| pending.max(at)),
        );
        self.pending_withdrawal_reason = Some(WithdrawalReason::TrajectoryReinitialized);
        for (state, definition) in self
            .definition_states
            .iter_mut()
            .zip(self.plan.plan.definitions.iter())
        {
            *state = empty_live_definition_state(definition);
        }
        self.last_consumed_end = None;
        self.terminal_endpoint_consumed = false;
    }

    #[must_use]
    pub(crate) const fn has_pending_withdrawals(&self) -> bool {
        self.pending_withdrawal_at.is_some()
    }

    /// Invalidates provisional metric state after a bounded evaluator failure
    /// without discarding otherwise valid navigation.
    pub(crate) fn begin_quality_invalidation(&mut self, at: SessionTime) {
        self.pending_withdrawal_at = Some(
            self.pending_withdrawal_at
                .map_or(at, |pending| pending.max(at)),
        );
        self.pending_withdrawal_reason = Some(WithdrawalReason::QualityInvalidated);
        for (state, definition) in self
            .definition_states
            .iter_mut()
            .zip(self.plan.plan.definitions.iter())
        {
            *state = empty_live_definition_state(definition);
        }
        self.last_consumed_end = None;
        self.terminal_endpoint_consumed = false;
    }

    /// Publishes at most the compiled mutation bound for one step. New-span
    /// evaluation is intentionally blocked until this drain completes.
    pub(crate) fn drain_pending_withdrawals_into(
        &mut self,
        output: &mut LiveMetricUpdate,
    ) -> Result<(), MetricError> {
        let at = self
            .pending_withdrawal_at
            .ok_or(MetricError::InvalidDefinition)?;
        let reason = self
            .pending_withdrawal_reason
            .ok_or(MetricError::InvalidDefinition)?;
        output.mutations.clear();
        let limit =
            usize::from(self.plan.limits.max_mutations_per_step).min(MAX_METRIC_MUTATIONS_PER_STEP);
        for tracked in self.entries.iter_mut() {
            if output.mutations.len() >= limit {
                break;
            }
            if tracked.active && tracked.value.stage() != EstimateStage::Finalized {
                tracked.active = false;
                tracked.revision = tracked.revision.saturating_add(1);
                output
                    .mutations
                    .push(MetricMutation::Withdraw {
                        id: tracked.id,
                        revision: tracked.revision,
                        reason,
                    })
                    .map_err(|_| MetricError::CapacityExceeded)?;
            }
        }
        output.navigation_watermark = at;
        output.metric_watermark = None;
        if !self
            .entries
            .iter()
            .any(|tracked| tracked.active && tracked.value.stage() != EstimateStage::Finalized)
        {
            self.entries.clear();
            self.pending_withdrawal_at = None;
            self.pending_withdrawal_reason = None;
        }
        Ok(())
    }

    pub(super) fn evaluate_live_candidates_into(
        &self,
        trajectory: &Trajectory,
        end_of_input: bool,
        scratch: &mut LiveMetricScratch,
    ) -> Result<NumericalWork, MetricError> {
        let active_candidate_bound = live_active_candidate_bound(&self.plan.plan)
            .map_err(|_| MetricError::InvalidDefinition)?;
        if active_candidate_bound > usize::from(self.plan.limits.max_active_candidates) {
            return Err(MetricError::CapacityExceeded);
        }
        let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let scan_start = self
            .last_consumed_end
            .map_or(span.start(), |previous| previous.max(span.start()));
        let scan_available = self
            .last_consumed_end
            .is_none_or(|previous| span.end() > previous)
            || (end_of_input && !self.terminal_endpoint_consumed);
        let include_terminal_endpoint = end_of_input && !self.terminal_endpoint_consumed;
        let limits = live_metric_evaluation_limits(self.plan.limits);
        let mut numerical_budget = NumericalWorkBudget::from_limits(limits);
        let mut commit_work = NumericalWork::default();
        scratch.candidates.clear();
        let mut uncertainty = TrajectoryMarginalUncertainty;
        let mut evaluator = Evaluator::new(
            self.plan.plan.run_namespace,
            trajectory,
            &mut uncertainty,
            &mut scratch.candidates,
        );

        for (index, definition) in self.plan.plan.definitions.iter().enumerate() {
            match definition {
                MetricDefinition::Distance(plan) => {
                    let previous = self.entries.iter().find_map(|entry| match entry.value {
                        MetricResultValue::Distance(report)
                            if entry.active && report.definition == plan.definition =>
                        {
                            Some(report)
                        }
                        _ => None,
                    });
                    let report = live_distance_report(
                        trajectory,
                        *plan,
                        previous,
                        scan_start,
                        span.end(),
                        scan_available,
                        limits,
                        &mut numerical_budget,
                    )?;
                    evaluator.emit(MetricResultValue::Distance(report))?;
                }
                MetricDefinition::Activity(plan) => {
                    let previous = self.entries.iter().find_map(|entry| match entry.value {
                        MetricResultValue::Activity(report)
                            if entry.active && report.definition == plan.definition =>
                        {
                            Some(report)
                        }
                        _ => None,
                    });
                    let LiveDefinitionState::Activity(source) =
                        self.definition_states
                            .get(index)
                            .ok_or(MetricError::InvalidDefinition)?
                    else {
                        return Err(MetricError::InvalidDefinition);
                    };
                    copy_live_activity_state(&mut scratch.activity_state, source);
                    let trajectory = evaluator.trajectory;
                    let before = numerical_budget;
                    advance_live_activity(
                        trajectory,
                        plan,
                        previous,
                        &mut scratch.activity_state,
                        scan_start,
                        span.end(),
                        scan_available,
                        include_terminal_endpoint,
                        limits,
                        &mut numerical_budget,
                        &mut |value| evaluator.emit(value),
                    )?;
                    commit_work.checked_add_assign(before.work_since(numerical_budget)?)?;
                }
                MetricDefinition::Lap(plan) if scan_available => {
                    let LiveDefinitionState::Lap(source) = self
                        .definition_states
                        .get(index)
                        .ok_or(MetricError::InvalidDefinition)?
                    else {
                        return Err(MetricError::InvalidDefinition);
                    };
                    copy_live_lap_state(&mut scratch.lap_state, source);
                    let trajectory = evaluator.trajectory;
                    let before = numerical_budget;
                    advance_live_lap(
                        trajectory,
                        plan,
                        &mut scratch.lap_state,
                        scan_start,
                        span.end(),
                        include_terminal_endpoint,
                        limits,
                        &mut numerical_budget,
                        &mut |value| evaluator.emit(value),
                    )?;
                    commit_work.checked_add_assign(before.work_since(numerical_budget)?)?;
                }
                MetricDefinition::Drag(plan) if scan_available => {
                    let LiveDefinitionState::Drag(source) = self
                        .definition_states
                        .get(index)
                        .ok_or(MetricError::InvalidDefinition)?
                    else {
                        return Err(MetricError::InvalidDefinition);
                    };
                    copy_live_drag_state(&mut scratch.drag_state, source);
                    let trajectory = evaluator.trajectory;
                    let before = numerical_budget;
                    advance_live_drag(
                        trajectory,
                        plan,
                        &mut scratch.drag_state,
                        scan_start,
                        span.end(),
                        include_terminal_endpoint,
                        limits,
                        &mut numerical_budget,
                        &mut |value| evaluator.emit(value),
                    )?;
                    commit_work.checked_add_assign(before.work_since(numerical_budget)?)?;
                }
                MetricDefinition::Drag(plan) => {
                    let LiveDefinitionState::Drag(source) = self
                        .definition_states
                        .get(index)
                        .ok_or(MetricError::InvalidDefinition)?
                    else {
                        return Err(MetricError::InvalidDefinition);
                    };
                    for (target_index, target) in source.target_report.iter().enumerate() {
                        if let Some(target) = target {
                            evaluator.emit(MetricResultValue::DragTarget(live_drag_report(
                                plan,
                                source,
                                target_index,
                                *target,
                            )?))?;
                        }
                    }
                }
                MetricDefinition::Lap(_) => {}
                MetricDefinition::Ski(_) => return Err(MetricError::Unsupported),
            }
        }
        // State commit repeats the same pure stateful traversals. Reserve
        // their measured oracle work now, while persistent state is untouched.
        numerical_budget.ensure_available(commit_work)?;
        Ok(commit_work)
    }

    /// Commits the exact state transition preflighted by candidate evaluation.
    /// The numerical routines are pure, so this in-place second pass cannot
    /// introduce a new disposition for identical inputs.
    pub(super) fn commit_definition_states(
        &mut self,
        trajectory: &Trajectory,
        end_of_input: bool,
        budget: &mut NumericalWorkBudget,
    ) -> Result<(), MetricError> {
        let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let scan_start = self
            .last_consumed_end
            .map_or(span.start(), |previous| previous.max(span.start()));
        let scan_available = self
            .last_consumed_end
            .is_none_or(|previous| span.end() > previous)
            || (end_of_input && !self.terminal_endpoint_consumed);
        if !scan_available {
            return Ok(());
        }
        let include_terminal_endpoint = end_of_input && !self.terminal_endpoint_consumed;
        let limits = live_metric_evaluation_limits(self.plan.limits);
        for (definition, state) in self
            .plan
            .plan
            .definitions
            .iter()
            .zip(self.definition_states.iter_mut())
        {
            match (definition, state) {
                (MetricDefinition::Lap(plan), LiveDefinitionState::Lap(state)) => {
                    advance_live_lap(
                        trajectory,
                        plan,
                        state,
                        scan_start,
                        span.end(),
                        include_terminal_endpoint,
                        limits,
                        budget,
                        &mut |_| Ok(()),
                    )?;
                }
                (MetricDefinition::Drag(plan), LiveDefinitionState::Drag(state)) => {
                    advance_live_drag(
                        trajectory,
                        plan,
                        state,
                        scan_start,
                        span.end(),
                        include_terminal_endpoint,
                        limits,
                        budget,
                        &mut |_| Ok(()),
                    )?;
                }
                (MetricDefinition::Activity(plan), LiveDefinitionState::Activity(state)) => {
                    let previous = self.entries.iter().find_map(|entry| match entry.value {
                        MetricResultValue::Activity(report)
                            if entry.active && report.definition == plan.definition =>
                        {
                            Some(report)
                        }
                        _ => None,
                    });
                    advance_live_activity(
                        trajectory,
                        plan,
                        previous,
                        state,
                        scan_start,
                        span.end(),
                        scan_available,
                        include_terminal_endpoint,
                        limits,
                        budget,
                        &mut |_| Ok(()),
                    )?;
                }
                (MetricDefinition::Distance(_), LiveDefinitionState::Stateless) => {}
                _ => return Err(MetricError::InvalidDefinition),
            }
        }
        Ok(())
    }

    /// Consumes the previously unseen suffix of the available rolling
    /// trajectory and produces a transactional bounded mutation set. State is
    /// unchanged on overflow.
    pub(crate) fn update_into(
        &mut self,
        trajectory: &Trajectory,
        navigation_watermark: SessionTime,
        end_of_input: bool,
        scratch: &mut LiveMetricScratch,
        output: &mut LiveMetricUpdate,
    ) -> Result<(), MetricError> {
        let commit_work = self.evaluate_live_candidates_into(trajectory, end_of_input, scratch)?;
        let results = &scratch.candidates;
        let trajectory_span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let lookahead_ns = i64::try_from(self.plan.maximum_lookahead.as_ns())
            .map_err(|_| MetricError::InvalidDefinition)?;
        // S_m is the trajectory frontier actually consumed by this evaluator.
        // It cannot be advanced by a caller supplying a navigation watermark
        // beyond the available dense trajectory.
        let consumed_through = navigation_watermark.min(trajectory_span.end());
        let metric_watermark = Some(
            consumed_through
                .checked_add(SignedDurationNs::from_ns(-lookahead_ns))
                .ok_or(MetricError::NumericalFailure)?,
        );

        // Preflight the complete ledger mutation before touching persistent
        // state. This avoids cloning the large tombstone ledger onto the MCU
        // stack while preserving the transactional overflow contract.
        scratch.preflight_seen.fill(false);
        let mut new_entries = 0_usize;
        let mut required_mutations = 0_usize;
        for (candidate_index, candidate) in results.as_slice().iter().enumerate() {
            let key = candidate.value.key();
            if results.as_slice()[..candidate_index]
                .iter()
                .any(|previous| previous.value.key() == key)
            {
                return Err(MetricError::InvalidDefinition);
            }
            let finalization_ready = candidate.value.stage() == EstimateStage::Finalized;
            let should_finalize = finalization_ready
                && match candidate.value.event_time() {
                    Some(event) => metric_watermark.is_some_and(|watermark| event <= watermark),
                    None => end_of_input,
                };
            let value = candidate.value.with_stage(if should_finalize {
                EstimateStage::Finalized
            } else {
                EstimateStage::Provisional
            });
            if let Some((index, tracked)) = self
                .entries
                .iter()
                .enumerate()
                .find(|(_, tracked)| tracked.key == key)
            {
                scratch.preflight_seen[index] = true;
                if !tracked.active {
                    required_mutations += 1 + usize::from(should_finalize);
                } else if tracked.value.stage() != EstimateStage::Finalized {
                    required_mutations += usize::from(tracked.value != value);
                    required_mutations += usize::from(should_finalize);
                }
            } else {
                new_entries += 1;
                required_mutations += 1 + usize::from(should_finalize);
            }
        }
        for (index, tracked) in self.entries.iter().enumerate() {
            if tracked.active
                && !scratch.preflight_seen[index]
                && tracked.value.stage() != EstimateStage::Finalized
            {
                if tracked.value.event_time().is_some() {
                    if tracked.finalization_ready
                        && tracked.value.event_time().is_some_and(|event| {
                            metric_watermark.is_some_and(|watermark| event <= watermark)
                        })
                    {
                        // Match the ordinary candidate transition: publish
                        // the finalized value, then its finalization marker.
                        required_mutations += 2;
                    } else if !tracked.finalization_ready {
                        required_mutations += 1;
                    }
                } else {
                    required_mutations += 1;
                }
            }
        }
        if self.entries.len().saturating_add(new_entries) > MAX_METRIC_RESULTS
            || self.entries.len().saturating_add(new_entries)
                > usize::from(self.plan.limits.max_results)
            || required_mutations > usize::from(self.plan.limits.max_mutations_per_step)
            || required_mutations > MAX_METRIC_MUTATIONS_PER_STEP
        {
            return Err(MetricError::CapacityExceeded);
        }
        self.next_allocation
            .checked_add(u64::try_from(new_entries).map_err(|_| MetricError::CapacityExceeded)?)
            .ok_or(MetricError::CapacityExceeded)?;

        // Candidate evaluation reserved this exact deterministic second-pass
        // work before any persistent state was touched.
        let mut commit_budget = NumericalWorkBudget::from_work(commit_work);
        self.commit_definition_states(trajectory, end_of_input, &mut commit_budget)?;
        debug_assert_eq!(commit_budget, NumericalWorkBudget::new(0, 0));

        scratch.seen.fill(false);
        scratch.mutations.clear();

        for candidate in results.as_slice() {
            let key = candidate.value.key();
            // Declaring end-of-input advances the navigation frontier, but it
            // cannot manufacture the future support a metric needs.  Results
            // in the terminal look-ahead tail therefore remain provisional.
            let finalization_ready = candidate.value.stage() == EstimateStage::Finalized;
            let should_finalize = finalization_ready
                && match candidate.value.event_time() {
                    Some(event) => metric_watermark.is_some_and(|watermark| event <= watermark),
                    None => end_of_input,
                };
            let value = candidate.value.with_stage(if should_finalize {
                EstimateStage::Finalized
            } else {
                EstimateStage::Provisional
            });
            if let Some((index, tracked)) = self
                .entries
                .iter_mut()
                .enumerate()
                .find(|(_, tracked)| tracked.key == key)
            {
                scratch.seen[index] = true;
                if tracked.value.stage() == EstimateStage::Finalized {
                    continue;
                }
                if !tracked.active {
                    tracked.active = true;
                    tracked.revision = tracked.revision.saturating_add(1);
                    tracked.value = value;
                    tracked.finalization_ready = finalization_ready;
                    push_mutation(
                        &mut scratch.mutations,
                        MetricMutation::Upsert {
                            id: tracked.id,
                            revision: tracked.revision,
                            value,
                        },
                        self.plan.limits.max_mutations_per_step,
                    )?;
                } else if tracked.value != value {
                    tracked.revision = tracked.revision.saturating_add(1);
                    tracked.value = value;
                    tracked.finalization_ready = finalization_ready;
                    push_mutation(
                        &mut scratch.mutations,
                        MetricMutation::Upsert {
                            id: tracked.id,
                            revision: tracked.revision,
                            value,
                        },
                        self.plan.limits.max_mutations_per_step,
                    )?;
                }
                tracked.finalization_ready = finalization_ready;
                if should_finalize {
                    tracked.revision = tracked.revision.saturating_add(1);
                    push_mutation(
                        &mut scratch.mutations,
                        MetricMutation::Finalize {
                            id: tracked.id,
                            revision: tracked.revision,
                        },
                        self.plan.limits.max_mutations_per_step,
                    )?;
                }
            } else {
                let id = LiveResultId::new(self.plan.plan.run_namespace, self.next_allocation);
                self.next_allocation = self
                    .next_allocation
                    .checked_add(1)
                    .ok_or(MetricError::CapacityExceeded)?;
                let tracked = TrackedResult {
                    key,
                    id,
                    revision: 0,
                    value,
                    finalization_ready,
                    active: true,
                };
                self.entries
                    .push(tracked)
                    .map_err(|_| MetricError::CapacityExceeded)?;
                scratch.seen[self.entries.len() - 1] = true;
                push_mutation(
                    &mut scratch.mutations,
                    MetricMutation::Upsert {
                        id,
                        revision: 0,
                        value,
                    },
                    self.plan.limits.max_mutations_per_step,
                )?;
                if should_finalize {
                    let inserted_index = self.entries.len() - 1;
                    self.entries[inserted_index].revision = 1;
                    push_mutation(
                        &mut scratch.mutations,
                        MetricMutation::Finalize { id, revision: 1 },
                        self.plan.limits.max_mutations_per_step,
                    )?;
                }
            }
        }

        for (index, tracked) in self.entries.iter_mut().enumerate() {
            // Once finalized, a result is immutable. It must not disappear
            // merely because its source segment rolled out of live storage.
            if tracked.active
                && !scratch.seen[index]
                && tracked.value.stage() != EstimateStage::Finalized
            {
                if tracked.value.event_time().is_some() {
                    let can_finalize = tracked.finalization_ready
                        && tracked.value.event_time().is_some_and(|event| {
                            metric_watermark.is_some_and(|watermark| event <= watermark)
                        });
                    if can_finalize {
                        tracked.revision = tracked.revision.saturating_add(1);
                        tracked.value = tracked.value.with_stage(EstimateStage::Finalized);
                        push_mutation(
                            &mut scratch.mutations,
                            MetricMutation::Upsert {
                                id: tracked.id,
                                revision: tracked.revision,
                                value: tracked.value,
                            },
                            self.plan.limits.max_mutations_per_step,
                        )?;
                        tracked.revision = tracked.revision.saturating_add(1);
                        push_mutation(
                            &mut scratch.mutations,
                            MetricMutation::Finalize {
                                id: tracked.id,
                                revision: tracked.revision,
                            },
                            self.plan.limits.max_mutations_per_step,
                        )?;
                    } else if !tracked.finalization_ready {
                        tracked.active = false;
                        tracked.revision = tracked.revision.saturating_add(1);
                        push_mutation(
                            &mut scratch.mutations,
                            MetricMutation::Withdraw {
                                id: tracked.id,
                                revision: tracked.revision,
                                reason: WithdrawalReason::RetrospectiveRuleChanged,
                            },
                            self.plan.limits.max_mutations_per_step,
                        )?;
                    }
                    continue;
                }
                tracked.active = false;
                tracked.revision = tracked.revision.saturating_add(1);
                push_mutation(
                    &mut scratch.mutations,
                    MetricMutation::Withdraw {
                        id: tracked.id,
                        revision: tracked.revision,
                        reason: WithdrawalReason::RetrospectiveRuleChanged,
                    },
                    self.plan.limits.max_mutations_per_step,
                )?;
            }
        }

        self.last_consumed_end = Some(
            self.last_consumed_end
                .map_or(trajectory_span.end(), |previous| {
                    previous.max(trajectory_span.end())
                }),
        );
        self.terminal_endpoint_consumed |= end_of_input;

        output.replace(
            navigation_watermark,
            metric_watermark,
            scratch.mutations.as_slice(),
        )?;
        Ok(())
    }

    /// By-value compatibility adapter retained only for unit tests. Firmware
    /// uses [`Self::update_into`] with workspace-owned storage.
    #[cfg(test)]
    pub fn update(
        &mut self,
        trajectory: &Trajectory,
        navigation_watermark: SessionTime,
        end_of_input: bool,
    ) -> Result<LiveMetricUpdate, MetricError> {
        let mut scratch = LiveMetricScratch::new();
        scratch
            .configure(&self.plan)
            .map_err(|_| MetricError::CapacityExceeded)?;
        let mut output = LiveMetricUpdate::empty();
        self.update_into(
            trajectory,
            navigation_watermark,
            end_of_input,
            &mut scratch,
            &mut output,
        )?;
        Ok(output)
    }

    #[must_use]
    pub fn active_results(&self) -> impl Iterator<Item = MetricResult> + '_ {
        self.entries
            .iter()
            .filter(|entry| entry.active)
            .map(|entry| MetricResult {
                id: entry.id,
                revision: entry.revision,
                value: entry.value,
            })
    }

    /// Number of active results whose complete future support is behind the
    /// metric watermark. Provisional terminal-tail values are deliberately
    /// excluded.
    #[must_use]
    pub fn finalized_result_count(&self) -> usize {
        self.entries
            .iter()
            .filter(|entry| entry.active && entry.value.stage() == EstimateStage::Finalized)
            .count()
    }
}

fn push_mutation(
    mutations: &mut FixedVec<MetricMutation, MAX_METRIC_MUTATIONS_PER_STEP>,
    mutation: MetricMutation,
    configured_limit: u8,
) -> Result<(), MetricError> {
    if mutations.len() >= usize::from(configured_limit) {
        return Err(MetricError::CapacityExceeded);
    }
    mutations
        .push(mutation)
        .map_err(|_| MetricError::CapacityExceeded)
}
