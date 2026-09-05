//! Metric lap regression tests.

use super::super::{
    definition::{CrossingDirection, FiniteGate, LapPlan, MetricDefinition, SpeedQuantity},
    plan::{LiveMetricLimits, MetricPlan},
    report::{
        GateCrossingReport, MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason,
        MetricResultValue,
    },
};
use super::support::{
    east_gate, eastbound_knot, eastbound_trajectory, eastbound_trajectory_between, test_trajectory,
    with_large_test_stack,
};
use crate::{
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId},
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{DurationNs, SessionTime},
};
use heapless::Vec as FixedVec;

#[test]
fn ordered_lap_gates_are_recomputed_within_one_dense_segment() {
    let trajectory = eastbound_trajectory_between(0, 1_000_000_000, 0.0, 20.0, 20.0, 20.0);
    let mut lap = LapPlan::new(MetricDefinitionId::new(1), ReferencePointId::new(1), None);
    lap.push_gate(east_gate(1, 5.0, 1.0)).unwrap();
    lap.push_gate(east_gate(2, 15.0, 1.0)).unwrap();
    let mut plan = MetricPlan::new(101);
    plan.push(MetricDefinition::Lap(lap)).unwrap();

    let reports: FixedVec<GateCrossingReport, 2> = plan
        .evaluate(&trajectory)
        .unwrap()
        .as_slice()
        .iter()
        .filter_map(|result| match result.value {
            MetricResultValue::GateCrossing(report) => Some(report),
            _ => None,
        })
        .collect();
    assert_eq!(reports.len(), 2);
    assert_eq!(reports[0].gate, GateId::new(1));
    assert_eq!(reports[1].gate, GateId::new(2));
    assert!((reports[0].time.as_ns() - 250_000_000).abs() <= 4);
    assert!((reports[1].time.as_ns() - 750_000_000).abs() <= 4);
}

#[test]
fn live_lap_rearm_accepts_an_interior_tangent_excursion() {
    with_large_test_stack(live_lap_rearm_accepts_an_interior_tangent_excursion_inner);
}

fn live_lap_rearm_accepts_an_interior_tangent_excursion_inner() {
    // p(s) = -0.5 + 8s - 8s^2. Both endpoints remain inside the
    // 1.5-m rearm band; only the interior tangent proves rearming.
    let trajectory = eastbound_trajectory_between(0, 1_000_000_000, -0.5, -0.5, 8.0, -8.0);
    let mut lap = LapPlan::new(MetricDefinitionId::new(2), ReferencePointId::new(1), None);
    lap.push_gate(east_gate(1, 0.0, 1.5)).unwrap();
    lap.set_maximum_occurrences_per_gate(2).unwrap();
    let mut plan = MetricPlan::new(102);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let limits = LiveMetricLimits {
        max_root_evaluations: 1_024,
        ..LiveMetricLimits::default()
    };
    let mut tracker = plan.compile_live(limits).unwrap().start();
    tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();

    let gates: FixedVec<GateCrossingReport, 2> = tracker
        .active_results()
        .filter_map(|result| match result.value {
            MetricResultValue::GateCrossing(report) => Some(report),
            _ => None,
        })
        .collect();
    assert_eq!(gates.len(), 2);
    assert_eq!(gates[0].occurrence, 0);
    assert_eq!(gates[1].occurrence, 1);
    assert!((gates[0].time.as_ns() - 66_987_298).abs() <= 16);
    assert!((gates[1].time.as_ns() - 933_012_702).abs() <= 16);
    assert!(tracker.active_results().any(|result| {
        matches!(result.value, MetricResultValue::Lap(report) if report.lap_index == 0)
    }));
}

#[test]
fn live_lap_retains_ordered_gate_phase_across_rollout() {
    with_large_test_stack(live_lap_retains_ordered_gate_phase_across_rollout_inner);
}

fn live_lap_retains_ordered_gate_phase_across_rollout_inner() {
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
    let returning = std::boxed::Box::new(eastbound_trajectory_between(
        2_000_000_000,
        3_000_000_000,
        20.0,
        0.0,
        -20.0,
        -20.0,
    ));
    let gate = |id, east| {
        FiniteGate::new(
            GateId::new(id),
            FrameId::new(1),
            [6_378_137.0, east, 0.0],
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
        .unwrap()
    };
    let mut lap = LapPlan::new(MetricDefinitionId::new(5), ReferencePointId::new(1), None);
    lap.push_gate(gate(1, 5.0)).unwrap();
    lap.push_gate(gate(2, 15.0)).unwrap();
    lap.set_maximum_occurrences_per_gate(2).unwrap();
    let mut plan = MetricPlan::new(95);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let mut tracker = plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();

    tracker
        .update(&first, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    tracker
        .update(&second, SessionTime::from_ns(2_000_000_000), false)
        .unwrap();
    tracker
        .update(&returning, SessionTime::from_ns(3_000_000_000), false)
        .unwrap();

    let lap = tracker
        .active_results()
        .find_map(|result| match result.value {
            MetricResultValue::Lap(report) => Some(report),
            _ => None,
        })
        .expect("ordered phase must produce a completed lap after rollout");
    assert_eq!(lap.lap_index, 0);
    assert!((lap.start.as_ns() - 500_000_000).abs() <= 4);
    assert!((lap.end.as_ns() - 2_750_000_000).abs() <= 4);
}

#[test]
fn live_lap_retains_gap_quality_across_rolling_eviction() {
    with_large_test_stack(live_lap_retains_gap_quality_across_rolling_eviction_inner);
}

fn live_lap_retains_gap_quality_across_rolling_eviction_inner() {
    let first = std::boxed::Box::new(eastbound_trajectory_between(
        0,
        1_000_000_000,
        0.0,
        10.0,
        10.0,
        10.0,
    ));
    let mut degraded = test_trajectory();
    let mut degraded_start = eastbound_knot(1_000_000_000, 10.0, 10.0);
    let mut degraded_end = eastbound_knot(2_000_000_000, 20.0, 10.0);
    for knot in [&mut degraded_start, &mut degraded_end] {
        knot.quality.validity = Validity::Degraded;
        knot.quality.imu_gap = true;
    }
    degraded
        .push_hermite_segment(degraded_start, degraded_end)
        .unwrap();
    let returning = std::boxed::Box::new(eastbound_trajectory_between(
        2_000_000_000,
        3_000_000_000,
        20.0,
        0.0,
        -20.0,
        -20.0,
    ));
    let gate = |id, east| {
        FiniteGate::new(
            GateId::new(id),
            FrameId::new(1),
            [6_378_137.0, east, 0.0],
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
        .unwrap()
    };
    let mut lap = LapPlan::new(MetricDefinitionId::new(15), ReferencePointId::new(1), None);
    lap.push_gate(gate(1, 5.0)).unwrap();
    lap.push_gate(gate(2, 15.0)).unwrap();
    lap.set_maximum_occurrences_per_gate(2).unwrap();
    let mut plan = MetricPlan::new(195);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let mut tracker = plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();

    tracker
        .update(&first, SessionTime::from_ns(1_000_000_000), false)
        .unwrap();
    tracker
        .update(&degraded, SessionTime::from_ns(2_000_000_000), false)
        .unwrap();
    tracker
        .update(&returning, SessionTime::from_ns(3_000_000_000), false)
        .unwrap();

    let report = tracker
        .active_results()
        .find_map(|result| match result.value {
            MetricResultValue::Lap(report) => Some(report),
            _ => None,
        })
        .expect("the completed lap must survive rolling eviction");
    assert_eq!(report.validity, Validity::Degraded);
    assert_eq!(
        report.elapsed_one_sigma_s,
        FieldValue::Unavailable(UnavailableReason::IllConditioned)
    );
}

#[test]
fn finite_gate_crossing_is_solved_inside_dense_interval() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let gate = FiniteGate::new(
        GateId::new(1),
        FrameId::new(1),
        [6_378_137.0, 5.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        20.0,
        20.0,
        CrossingDirection::NegativeToPositive,
        1.0,
        1.0,
        DurationNs::ZERO,
        Some(0.01),
    )
    .unwrap();
    let mut lap = LapPlan::new(
        MetricDefinitionId::new(2),
        ReferencePointId::new(1),
        Some(SpeedQuantity::InstantaneousHorizontal),
    );
    lap.push_gate(gate).unwrap();
    let mut plan = MetricPlan::new(45);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    let crossing = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::GateCrossing(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert!((crossing.time.as_ns() - 500_000_000).abs() <= 2);
    assert!((crossing.normal_speed_mps - 10.0).abs() < 1.0e-9);
}

#[test]
fn finite_gate_frame_must_match_trajectory_frame() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let gate = FiniteGate::new(
        GateId::new(1),
        FrameId::new(2),
        [6_378_137.0, 5.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        20.0,
        20.0,
        CrossingDirection::NegativeToPositive,
        1.0,
        1.0,
        DurationNs::ZERO,
        None,
    )
    .unwrap();
    let mut lap = LapPlan::new(MetricDefinitionId::new(2), ReferencePointId::new(1), None);
    lap.push_gate(gate).unwrap();
    let mut plan = MetricPlan::new(47);
    plan.push(MetricDefinition::Lap(lap)).unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    assert!(results.diagnostics().eq([MetricDefinitionDiagnostic {
        definition: MetricDefinitionId::new(2),
        reference_point: ReferencePointId::new(1),
        reason: MetricDefinitionDiagnosticReason::FrameMismatch,
        stage: EstimateStage::Finalized,
        validity: Validity::Invalid,
    }]));
}
