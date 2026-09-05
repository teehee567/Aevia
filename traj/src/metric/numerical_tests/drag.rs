//! Metric drag regression tests.

use super::super::{
    definition::{
        DistanceQuantity, DragPlan, DragTarget, LaunchRule, MetricDefinition, Rollout,
        SpeedQuantity, TargetDirection,
    },
    live_state::LiveMetricScratch,
    live_tracker::LiveMetricTracker,
    plan::{LiveMetricLimits, MetricPlan},
    report::{
        DragTargetReport, LiveMetricUpdate, MetricError, MetricMutation, MetricResultValue,
        WithdrawalReason,
    },
};
use super::support::{eastbound_trajectory, eastbound_trajectory_between, with_large_test_stack};
use crate::{
    ids::{MetricDefinitionId, ReferencePointId, TargetId},
    quality::{EstimateStage, FieldValue, UnavailableReason},
    time::{DurationNs, SessionTime},
};
use heapless::Vec as FixedVec;

#[test]
fn descending_drag_terminal_tail_stays_provisional() {
    let trajectory = eastbound_trajectory_between(0, 1_000_000_000, 0.0, 7.0 / 8.0, 1.5, 0.25);
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 0.5,
        direction: TargetDirection::Descending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(103);
    plan.push(MetricDefinition::Drag(drag)).unwrap();

    let report = plan
        .evaluate(&trajectory)
        .unwrap()
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::DragTarget(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert!((report.event_time.as_ns() - 800_000_000).abs() <= 4);
    assert_eq!(report.stage, EstimateStage::Provisional);
}

#[test]
fn live_descending_drag_rebound_withdraws_reopens_and_confirms() {
    with_large_test_stack(live_descending_drag_rebound_withdraws_reopens_and_confirms_inner);
}

fn live_descending_drag_rebound_withdraws_reopens_and_confirms_inner() {
    let braking = eastbound_trajectory_between(0, 1_000_000_000, 0.0, 7.0 / 8.0, 1.5, 0.25);
    let rebound_and_brake_again = eastbound_trajectory_between(
        1_000_000_000,
        2_000_000_000,
        7.0 / 8.0,
        43.0 / 24.0,
        0.25,
        0.25,
    );
    let dwell_completion = eastbound_trajectory_between(
        2_000_000_000,
        2_500_000_000,
        43.0 / 24.0,
        23.0 / 12.0,
        0.25,
        0.25,
    );
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(4),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(2),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 0.5,
        direction: TargetDirection::Descending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(104);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let mut tracker = plan
        .compile_live(LiveMetricLimits {
            max_root_evaluations: 1_024,
            ..LiveMetricLimits::default()
        })
        .unwrap()
        .start();

    let first = tracker
        .update(&braking, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    let id = match first.mutations()[0] {
        MetricMutation::Upsert { id, value, .. } => {
            assert_eq!(value.stage(), EstimateStage::Provisional);
            id
        }
        _ => panic!("first braking target mutation was not an upsert"),
    };

    let rebound = tracker
        .update(
            &rebound_and_brake_again,
            SessionTime::from_ns(2_000_000_000),
            false,
        )
        .unwrap();
    assert!(rebound.mutations().iter().any(|mutation| matches!(
        mutation,
        MetricMutation::Withdraw {
            id: withdrawn,
            reason: WithdrawalReason::RetrospectiveRuleChanged,
            ..
        } if *withdrawn == id
    )));
    assert!(
        !rebound
            .mutations()
            .iter()
            .any(|mutation| matches!(mutation, MetricMutation::Upsert { .. }))
    );

    let reopened = tracker
        .update(
            &rebound_and_brake_again,
            SessionTime::from_ns(2_000_000_000),
            false,
        )
        .unwrap();
    let reopened_report = reopened
        .mutations()
        .iter()
        .find_map(|mutation| match mutation {
            MetricMutation::Upsert {
                id: reopened_id,
                value: MetricResultValue::DragTarget(report),
                ..
            } if *reopened_id == id => Some(*report),
            _ => None,
        })
        .unwrap();
    assert_eq!(reopened_report.stage, EstimateStage::Provisional);
    assert!((reopened_report.event_time.as_ns() - 1_933_012_702).abs() <= 16);

    let completed = tracker
        .update(
            &dwell_completion,
            SessionTime::from_ns(2_500_000_000),
            false,
        )
        .unwrap();
    assert!(completed.mutations().iter().any(
        |mutation| matches!(mutation, MetricMutation::Finalize { id: finalized, .. } if *finalized == id)
    ));
    let retained = tracker
        .active_results()
        .find(|result| result.id == id)
        .unwrap();
    assert_eq!(retained.value.stage(), EstimateStage::Finalized);
}

#[test]
fn live_drag_retains_launch_and_distance_progress_across_rollout() {
    with_large_test_stack(live_drag_retains_launch_and_distance_progress_across_rollout_inner);
}

fn live_drag_retains_launch_and_distance_progress_across_rollout_inner() {
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
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(7),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 15.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(94);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let mut tracker = plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();

    let before_target = tracker
        .update(&first, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    assert!(before_target.mutations().is_empty());
    let reached = tracker
        .update(&second, SessionTime::from_ns(2_000_000_000), false)
        .unwrap();
    let report = reached
        .mutations()
        .iter()
        .find_map(|mutation| match mutation {
            MetricMutation::Upsert {
                value: MetricResultValue::DragTarget(report),
                ..
            } => Some(*report),
            _ => None,
        })
        .expect("distance target must survive launch-window rollout");
    assert_eq!(report.launch_time, SessionTime::ZERO);
    assert!((report.elapsed_seconds - 1.5).abs() < 1.0e-8);
    assert!((report.event_time.as_ns() - 1_500_000_000).abs() <= 4);
}

#[test]
fn live_mutation_overflow_does_not_advance_drag_state() {
    with_large_test_stack(live_mutation_overflow_does_not_advance_drag_state_inner);
}

fn live_mutation_overflow_does_not_advance_drag_state_inner() {
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
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.stop_dwell = DurationNs::ZERO;
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(7),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 15.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(95);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let compiled = plan.compile_live(LiveMetricLimits::default()).unwrap();
    let mut tracker = LiveMetricTracker::unconfigured();
    tracker.configure(&compiled).unwrap();
    // Compile-time validation prevents this setting through the public
    // API. Corrupt it internally to exercise the transactional runtime
    // defense independently.
    tracker.plan.limits.max_mutations_per_step = 1;
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
    assert_eq!(
        output.navigation_watermark(),
        SessionTime::from_ns(1_000_000_000)
    );
    assert!(output.mutations().is_empty());
    let before_overflow = tracker.definition_states[0].clone();
    assert_eq!(
        tracker.update_into(
            &second,
            SessionTime::from_ns(2_000_000_000),
            false,
            &mut scratch,
            &mut output,
        ),
        Err(MetricError::CapacityExceeded)
    );
    // Failed speculative work leaves both persistent state and the last
    // caller-owned successful update intact.
    assert_eq!(
        output.navigation_watermark(),
        SessionTime::from_ns(1_000_000_000)
    );
    assert!(output.mutations().is_empty());
    assert_eq!(tracker.definition_states[0], before_overflow);
    assert_eq!(
        tracker.last_consumed_end,
        Some(SessionTime::from_ns(1_000_000_000))
    );
    assert!(tracker.active_results().next().is_none());

    // Retrying the identical suffix with enough output capacity must
    // produce the original event, proving the failed dry run committed no
    // phase or distance progress.
    tracker.plan.limits.max_mutations_per_step = 2;
    tracker
        .update_into(
            &second,
            SessionTime::from_ns(2_000_000_000),
            false,
            &mut scratch,
            &mut output,
        )
        .unwrap();
    assert_eq!(output.mutations().len(), 2);
    let event_time = output
        .mutations()
        .iter()
        .find_map(|mutation| match mutation {
            MetricMutation::Upsert {
                value: MetricResultValue::DragTarget(report),
                ..
            } => Some(report.event_time),
            _ => None,
        })
        .unwrap();
    assert!((event_time.as_ns() - 1_500_000_000).abs() <= 4);
}

#[test]
fn drag_speed_and_distance_targets_use_continuous_roots() {
    // The Hermite path is p(t) = 10 t^2 and v(t) = 20 t. Speed 10 m/s
    // occurs at 0.5 s, while integrated distance 5 m occurs at sqrt(0.5)
    // seconds. The two independently exercise continuous root finding.
    let trajectory = eastbound_trajectory(0.0, 20.0, 10.0);
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 10.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(2),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 5.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(46);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    let targets: FixedVec<DragTargetReport, 2> = results
        .as_slice()
        .iter()
        .filter_map(|result| match result.value {
            MetricResultValue::DragTarget(report) => Some(report),
            _ => None,
        })
        .collect();
    assert_eq!(targets.len(), 2);
    assert!((targets[0].event_time.as_ns() - 500_000_000).abs() <= 4);
    assert!((targets[1].event_time.as_ns() - 707_106_781).abs() <= 4);
}

#[test]
fn rollout_after_a_target_is_typed_unavailable_for_offline_and_live() {
    with_large_test_stack(rollout_after_a_target_is_typed_unavailable_for_offline_and_live_inner);
}

fn rollout_after_a_target_is_typed_unavailable_for_offline_and_live_inner() {
    // p(t) = 10 t^2: the 10 m/s target occurs at 0.5 s, while the
    // eight-metre rollout is not reached until sqrt(0.8) s.
    let trajectory = eastbound_trajectory(0.0, 20.0, 10.0);
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(4),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.rollout = Rollout::Distance {
        quantity: DistanceQuantity::HorizontalPath,
        metres: 8.0,
    };
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 10.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(47);
    plan.push(MetricDefinition::Drag(drag)).unwrap();

    let offline_report = plan
        .evaluate(&trajectory)
        .unwrap()
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::DragTarget(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert_eq!(offline_report.elapsed_seconds, 0.5);
    assert_eq!(
        offline_report.rollout_adjusted_seconds,
        FieldValue::Unavailable(UnavailableReason::OutsideQualifiedRange)
    );

    let mut tracker = plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();
    let update = tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), true)
        .unwrap();
    let live_report = update
        .mutations()
        .iter()
        .find_map(|mutation| match mutation {
            MetricMutation::Upsert {
                value: MetricResultValue::DragTarget(report),
                ..
            } => Some(*report),
            _ => None,
        })
        .unwrap();
    assert_eq!(
        live_report.rollout_adjusted_seconds,
        FieldValue::Unavailable(UnavailableReason::OutsideQualifiedRange)
    );
}
