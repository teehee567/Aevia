//! Metric tracker regression tests.

use super::super::{
    MAX_METRIC_MUTATIONS_PER_STEP,
    definition::{
        DistancePlan, DistanceQuantity, DragPlan, DragTarget, LapPlan, LaunchRule,
        MetricDefinition, SpeedQuantity, TargetDirection,
    },
    live_tracker::TrackedResult,
    plan::{LiveMetricLimits, MetricPlan},
    report::{
        ActivitySplitReport, LiveMetricUpdate, MetricError, MetricMutation, MetricResultValue,
        WithdrawalReason,
    },
};
use super::support::{
    east_gate, eastbound_trajectory, eastbound_trajectory_between,
    gap_bridged_eastbound_trajectory, with_large_test_stack,
};
use crate::{
    ids::{LiveResultId, MetricDefinitionId, ReferencePointId, TargetId},
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::SessionTime,
};

#[test]
fn live_update_replace_rejects_oversize_without_mutation() {
    let prior = MetricMutation::Withdraw {
        id: LiveResultId::new(1, 2),
        revision: 3,
        reason: WithdrawalReason::OutputSuperseded,
    };
    let mut output = LiveMetricUpdate::empty();
    output
        .replace(
            SessionTime::from_ns(7),
            Some(SessionTime::from_ns(5)),
            &[prior],
        )
        .unwrap();
    let before = output.clone();
    let oversized = [prior; MAX_METRIC_MUTATIONS_PER_STEP + 1];
    assert_eq!(
        output.replace(SessionTime::from_ns(9), None, &oversized),
        Err(MetricError::CapacityExceeded)
    );
    assert_eq!(output, before);
}

#[test]
fn live_runtime_rechecks_the_active_candidate_contract_transactionally() {
    with_large_test_stack(
        live_runtime_rechecks_the_active_candidate_contract_transactionally_inner,
    );
}

fn live_runtime_rechecks_the_active_candidate_contract_transactionally_inner() {
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(1),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    for (id, metres) in [(1, 1.0), (2, 2.0)] {
        drag.push_target(DragTarget::Distance {
            id: TargetId::new(id),
            quantity: DistanceQuantity::HorizontalPath,
            metres,
        })
        .unwrap();
    }
    let mut plan = MetricPlan::new(10);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let compiled = plan
        .compile_live(LiveMetricLimits {
            max_active_candidates: 2,
            ..LiveMetricLimits::default()
        })
        .unwrap();
    let mut tracker = compiled.start();
    // Simulate corrupted/internal configuration after preflight. The
    // runtime guard must reject it before any cursor or ledger mutation.
    tracker.plan.limits.max_active_candidates = 1;
    let trajectory = eastbound_trajectory(1.0, 1.0, 1.0);

    assert_eq!(
        tracker.update(&trajectory, SessionTime::from_ns(1_000_000_000), false),
        Err(MetricError::CapacityExceeded)
    );
    assert!(tracker.entries.is_empty());
    assert_eq!(tracker.last_consumed_end, None);
}

#[test]
fn live_metrics_never_publish_gap_bridges_as_nominal() {
    with_large_test_stack(live_metrics_never_publish_gap_bridges_as_nominal_inner);
}

fn live_metrics_never_publish_gap_bridges_as_nominal_inner() {
    let trajectory = gap_bridged_eastbound_trajectory();
    let watermark = SessionTime::from_ns(1_000_000_000);

    let mut distance_plan = MetricPlan::new(112);
    distance_plan
        .push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(1),
            quantity: DistanceQuantity::HorizontalPath,
            reference_point: ReferencePointId::new(1),
            absolute_tolerance_m: 0.01,
            relative_tolerance: 1.0e-6,
        }))
        .unwrap();
    let mut distance_tracker = distance_plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();
    let distance_update = distance_tracker
        .update(&trajectory, watermark, false)
        .unwrap();
    assert!(distance_update.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Upsert {
            value: MetricResultValue::Distance(report),
            ..
        } if report.validity != Validity::Nominal
    )));

    let mut lap = LapPlan::new(MetricDefinitionId::new(3), ReferencePointId::new(1), None);
    lap.push_gate(east_gate(1, 5.0, 1.0)).unwrap();
    lap.set_maximum_occurrences_per_gate(1).unwrap();
    let mut lap_plan = MetricPlan::new(114);
    lap_plan.push(MetricDefinition::Lap(lap)).unwrap();
    let mut lap_tracker = lap_plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();
    let lap_update = lap_tracker.update(&trajectory, watermark, false).unwrap();
    assert!(lap_update.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Upsert {
            value: MetricResultValue::GateCrossing(report),
            ..
        } if report.validity != Validity::Nominal
            && report.time_one_sigma_s
                == FieldValue::Unavailable(UnavailableReason::IllConditioned)
    )));

    let mut drag = DragPlan::new(
        MetricDefinitionId::new(4),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 5.0,
    })
    .unwrap();
    let mut drag_plan = MetricPlan::new(115);
    drag_plan.push(MetricDefinition::Drag(drag)).unwrap();
    let mut drag_tracker = drag_plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();
    let drag_update = drag_tracker.update(&trajectory, watermark, false).unwrap();
    assert!(drag_update.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Upsert {
            value: MetricResultValue::DragTarget(report),
            ..
        } if report.validity != Validity::Nominal
            && report.event_time_one_sigma_s
                == FieldValue::Unavailable(UnavailableReason::IllConditioned)
    )));
}

#[test]
fn reinitialization_withdrawals_drain_in_bounded_batches_without_id_reuse() {
    with_large_test_stack(
        reinitialization_withdrawals_drain_in_bounded_batches_without_id_reuse_inner,
    );
}

fn reinitialization_withdrawals_drain_in_bounded_batches_without_id_reuse_inner() {
    let mut plan = MetricPlan::new(113);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 0.01,
        relative_tolerance: 1.0e-6,
    }))
    .unwrap();
    let compiled = plan.compile_live(LiveMetricLimits::default()).unwrap();
    let mut tracker = compiled.start();
    for allocation in 0_u16..20 {
        let value = MetricResultValue::ActivitySplit(ActivitySplitReport {
            definition: MetricDefinitionId::new(90),
            split_index: allocation,
            horizontal_distance_m: f64::from(allocation) + 1.0,
            time: SessionTime::from_ns(i64::from(allocation)),
            elapsed_seconds: f64::from(allocation),
            reference_point: ReferencePointId::new(1),
            stage: EstimateStage::Provisional,
            validity: Validity::Nominal,
        });
        tracker
            .entries
            .push(TrackedResult {
                key: value.key(),
                id: LiveResultId::new(113, u64::from(allocation)),
                revision: 0,
                value,
                finalization_ready: false,
                active: true,
            })
            .unwrap();
    }
    tracker.next_allocation = 20;
    let reset_at = SessionTime::from_ns(50);
    tracker.begin_trajectory_reinitialization(reset_at);
    let mut output = LiveMetricUpdate::empty();
    tracker.drain_pending_withdrawals_into(&mut output).unwrap();
    assert_eq!(output.mutations().len(), MAX_METRIC_MUTATIONS_PER_STEP);
    assert!(tracker.has_pending_withdrawals());
    assert!(output.mutations().iter().all(|mutation| matches!(
        mutation,
        MetricMutation::Withdraw {
            reason: WithdrawalReason::TrajectoryReinitialized,
            ..
        }
    )));
    tracker.drain_pending_withdrawals_into(&mut output).unwrap();
    assert_eq!(output.mutations().len(), 4);
    assert!(!tracker.has_pending_withdrawals());
    assert!(tracker.entries.is_empty());
    assert_eq!(tracker.next_allocation, 20);

    let trajectory = eastbound_trajectory(1.0, 1.0, 1.0);
    let update = tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    assert!(update.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Upsert { id, .. } if *id == LiveResultId::new(113, 20)
    )));
    tracker.begin_quality_invalidation(SessionTime::from_ns(1_000_000_001));
    tracker.drain_pending_withdrawals_into(&mut output).unwrap();
    assert!(output.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Withdraw {
            id,
            reason: WithdrawalReason::QualityInvalidated,
            ..
        } if *id == LiveResultId::new(113, 20)
    )));
}

#[test]
fn live_tracker_keeps_id_and_finalizes_once() {
    with_large_test_stack(live_tracker_keeps_id_and_finalizes_once_inner);
}

fn live_tracker_keeps_id_and_finalizes_once_inner() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let mut plan = MetricPlan::new(88);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-7,
        relative_tolerance: 1.0e-9,
    }))
    .unwrap();
    let compiled = plan.compile_live(LiveMetricLimits::default()).unwrap();
    let mut tracker = compiled.start();
    let first = tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    assert_eq!(first.mutations().len(), 1);
    let id = match first.mutations()[0] {
        MetricMutation::Upsert { id, .. } => id,
        _ => panic!("first mutation was not an upsert"),
    };
    assert_eq!(id, LiveResultId::new(88, 0));
    assert!(
        tracker
            .update(&trajectory, SessionTime::from_ns(1_000_000_000), false)
            .unwrap()
            .mutations()
            .is_empty()
    );
    let final_update = tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), true)
        .unwrap();
    assert!(final_update
        .mutations()
        .iter()
        .any(|mutation| matches!(mutation, MetricMutation::Finalize { id: finalized, .. } if *finalized == id)));
    assert!(
        tracker
            .update(&trajectory, SessionTime::from_ns(1_000_000_000), true)
            .unwrap()
            .mutations()
            .is_empty()
    );
}

#[test]
fn terminal_event_waits_for_consumed_lookahead_and_stays_immutable() {
    with_large_test_stack(terminal_event_waits_for_consumed_lookahead_and_stays_immutable_inner);
}

fn terminal_event_waits_for_consumed_lookahead_and_stays_immutable_inner() {
    let short = std::boxed::Box::new(eastbound_trajectory(0.0, 20.0, 10.0));
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 18.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(91);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let compiled = plan.compile_live(LiveMetricLimits::default()).unwrap();
    let mut terminal_tracker = compiled.start();

    // Supplying a frontier past the dense path cannot manufacture the
    // missing 500 ms of retrospective support at the terminal tail.
    let tail = terminal_tracker
        .update(&short, SessionTime::from_ns(2_000_000_000), true)
        .unwrap();
    assert_eq!(
        tail.metric_watermark(),
        Some(SessionTime::from_ns(500_000_000))
    );
    assert_eq!(terminal_tracker.finalized_result_count(), 0);
    assert!(
        !tail
            .mutations()
            .iter()
            .any(|mutation| matches!(mutation, MetricMutation::Finalize { .. }))
    );
    match tail.mutations()[0] {
        MetricMutation::Upsert { id, value, .. } => {
            assert_eq!(value.stage(), EstimateStage::Provisional);
            assert_eq!(id, LiveResultId::new(91, 0));
        }
        _ => panic!("first event mutation was not an upsert"),
    }

    // A separate still-open session demonstrates that support can close
    // after the source segment itself rolls out.
    let mut tracker = compiled.start();
    let initial = tracker
        .update(&short, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    let id = match initial.mutations()[0] {
        MetricMutation::Upsert { id, .. } => id,
        _ => panic!("first event mutation was not an upsert"),
    };

    // The event's source segment may roll out before its lookahead closes.
    // Advancing with only the following segment must still finalize the
    // retained candidate at the same stable ID.
    let complete = std::boxed::Box::new(eastbound_trajectory_between(
        1_000_000_000,
        2_000_000_000,
        10.0,
        30.0,
        20.0,
        20.0,
    ));
    let finalized = tracker
        .update(&complete, SessionTime::from_ns(2_000_000_000), false)
        .unwrap();
    assert_eq!(
        finalized.metric_watermark(),
        Some(SessionTime::from_ns(1_500_000_000))
    );
    assert!(finalized.mutations().iter().any(
        |mutation| matches!(mutation, MetricMutation::Finalize { id: finalized, .. } if *finalized == id)
    ));
    assert_eq!(tracker.finalized_result_count(), 1);

    // Once finalized, losing the source interval from rolling storage
    // neither withdraws nor revises the result.
    let rolled = std::boxed::Box::new(eastbound_trajectory_between(
        2_000_000_000,
        3_000_000_000,
        30.0,
        50.0,
        20.0,
        20.0,
    ));
    let after_roll = tracker
        .update(&rolled, SessionTime::from_ns(3_000_000_000), false)
        .unwrap();
    assert!(after_roll.mutations().is_empty());
    let retained = tracker.active_results().next().unwrap();
    assert_eq!(retained.id, id);
    assert_eq!(retained.value.stage(), EstimateStage::Finalized);
}
