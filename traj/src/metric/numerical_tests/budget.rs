//! Metric budget regression tests.

use super::super::{
    definition::{
        CrossingDirection, DistancePlan, DistanceQuantity, DragPlan, DragTarget, FiniteGate,
        LapPlan, LaunchRule, MetricDefinition,
    },
    live_state::LiveMetricScratch,
    live_tracker::LiveMetricTracker,
    plan::{LiveMetricLimits, MetricPlan},
    report::{LiveMetricUpdate, MetricError},
};
use super::support::{eastbound_trajectory, eastbound_trajectory_between, with_large_test_stack};
use crate::{
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId, TargetId},
    time::{DurationNs, SessionTime},
};

#[test]
fn live_quadrature_budget_exhaustion_is_transactional() {
    with_large_test_stack(live_quadrature_budget_exhaustion_is_transactional_inner);
}

fn live_quadrature_budget_exhaustion_is_transactional_inner() {
    let first = std::boxed::Box::new(eastbound_trajectory_between(
        0,
        1_000_000_000,
        0.0,
        10.0,
        10.0,
        10.0,
    ));
    let second = std::boxed::Box::new(eastbound_trajectory_between(
        1_000_000_000,
        2_000_000_000,
        10.0,
        20.0,
        10.0,
        10.0,
    ));
    let mut plan = MetricPlan::new(96);
    for definition in [1, 2] {
        plan.push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(definition),
            quantity: DistanceQuantity::HorizontalPath,
            reference_point: ReferencePointId::new(1),
            absolute_tolerance_m: 0.01,
            relative_tolerance: 1.0e-6,
        }))
        .unwrap();
    }
    // V1 derives one 15-point quadrature panel from each configured root
    // credit. Two credits are exactly enough for the two straight-line
    // distance definitions in the successful first update.
    let limits = LiveMetricLimits {
        max_root_evaluations: 2,
        ..LiveMetricLimits::default()
    };
    let compiled = plan.compile_live(limits).unwrap();
    let mut tracker = LiveMetricTracker::unconfigured();
    tracker.configure(&compiled).unwrap();
    let mut scratch = LiveMetricScratch::new();
    scratch.configure(&compiled).unwrap();
    let mut output = LiveMetricUpdate::empty();

    tracker
        .update_into(
            &first,
            SessionTime::from_ns(1_000_000_000),
            false,
            &mut scratch,
            &mut output,
        )
        .unwrap();
    assert_eq!(tracker.entries.len(), 2);
    let output_before = output.clone();
    let entries_before = tracker.entries.clone();
    let states_before = tracker.definition_states.clone();
    let allocation_before = tracker.next_allocation;
    let cursor_before = tracker.last_consumed_end;

    // One panel now permits the first distance candidate but forces the
    // second to fail before ledger/state commit.
    tracker.plan.limits.max_root_evaluations = 1;
    assert_eq!(
        tracker.update_into(
            &second,
            SessionTime::from_ns(2_000_000_000),
            false,
            &mut scratch,
            &mut output,
        ),
        Err(MetricError::EvaluationBudgetExceeded)
    );
    assert_eq!(output, output_before);
    assert_eq!(tracker.entries, entries_before);
    assert_eq!(tracker.definition_states, states_before);
    assert_eq!(tracker.next_allocation, allocation_before);
    assert_eq!(tracker.last_consumed_end, cursor_before);
}

#[test]
fn live_stateful_quadrature_commit_reservation_is_transactional() {
    with_large_test_stack(live_stateful_quadrature_commit_reservation_is_transactional_inner);
}

fn live_stateful_quadrature_commit_reservation_is_transactional_inner() {
    let trajectory = std::boxed::Box::new(eastbound_trajectory(10.0, 10.0, 10.0));
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(4),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(8),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 1_000.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(98);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    // Candidate traversal consumes two 15-point panels and mutates its
    // scratch drag state. Three panel credits let that speculative pass
    // finish, but cannot reserve the same two panels for state commit.
    let limits = LiveMetricLimits {
        max_root_evaluations: 3,
        ..LiveMetricLimits::default()
    };
    let compiled = plan.compile_live(limits).unwrap();
    let mut tracker = LiveMetricTracker::unconfigured();
    tracker.configure(&compiled).unwrap();
    let mut scratch = LiveMetricScratch::new();
    scratch.configure(&compiled).unwrap();
    let mut output = LiveMetricUpdate::empty();
    let states_before = tracker.definition_states.clone();

    assert_eq!(
        tracker.update_into(
            &trajectory,
            SessionTime::from_ns(1_000_000_000),
            false,
            &mut scratch,
            &mut output,
        ),
        Err(MetricError::EvaluationBudgetExceeded)
    );
    assert_eq!(tracker.definition_states, states_before);
    assert!(tracker.entries.is_empty());
    assert_eq!(tracker.next_allocation, 0);
    assert_eq!(tracker.last_consumed_end, None);
    assert_eq!(output, LiveMetricUpdate::empty());
}

#[test]
fn live_polynomial_root_budget_exhaustion_is_transactional() {
    with_large_test_stack(live_polynomial_root_budget_exhaustion_is_transactional_inner);
}

fn live_polynomial_root_budget_exhaustion_is_transactional_inner() {
    let before_gate = std::boxed::Box::new(eastbound_trajectory_between(
        0,
        1_000_000_000,
        0.0,
        4.0,
        4.0,
        4.0,
    ));
    let through_gate = std::boxed::Box::new(eastbound_trajectory_between(
        1_000_000_000,
        2_000_000_000,
        4.0,
        10.0,
        6.0,
        6.0,
    ));
    let gate = FiniteGate::new(
        GateId::new(1),
        FrameId::new(1),
        [6_378_137.0, 5.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        20.0,
        20.0,
        CrossingDirection::Either,
        0.1,
        1.0,
        DurationNs::ZERO,
        None,
    )
    .unwrap();
    let mut lap = LapPlan::new(MetricDefinitionId::new(3), ReferencePointId::new(1), None);
    lap.push_gate(gate).unwrap();
    lap.set_maximum_occurrences_per_gate(1).unwrap();
    let mut plan = MetricPlan::new(97);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let compiled = plan.compile_live(LiveMetricLimits::default()).unwrap();
    let mut tracker = LiveMetricTracker::unconfigured();
    tracker.configure(&compiled).unwrap();
    let mut scratch = LiveMetricScratch::new();
    scratch.configure(&compiled).unwrap();
    let mut output = LiveMetricUpdate::empty();

    tracker
        .update_into(
            &before_gate,
            SessionTime::from_ns(1_000_000_000),
            false,
            &mut scratch,
            &mut output,
        )
        .unwrap();
    let output_before = output.clone();
    let entries_before = tracker.entries.clone();
    let states_before = tracker.definition_states.clone();
    let allocation_before = tracker.next_allocation;
    let cursor_before = tracker.last_consumed_end;

    // A cubic gate equation needs more than one polynomial evaluation.
    // Exhaustion must remain speculative even though the suffix crosses
    // the gate and would otherwise advance lap state.
    tracker.plan.limits.max_root_evaluations = 1;
    assert_eq!(
        tracker.update_into(
            &through_gate,
            SessionTime::from_ns(2_000_000_000),
            false,
            &mut scratch,
            &mut output,
        ),
        Err(MetricError::EvaluationBudgetExceeded)
    );
    assert_eq!(output, output_before);
    assert_eq!(tracker.entries, entries_before);
    assert_eq!(tracker.definition_states, states_before);
    assert_eq!(tracker.next_allocation, allocation_before);
    assert_eq!(tracker.last_consumed_end, cursor_before);
}
