//! Metric offline regression tests.

#[cfg(feature = "offline")]
use super::super::report::{MetricError, MetricResults};
use super::super::{
    definition::{
        ActivityPlan, DistancePlan, DistanceQuantity, DragPlan, DragTarget, LaunchRule,
        MetricDefinition, SkiHmmModel, SkiPlan,
    },
    plan::MetricPlan,
    report::{MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason, MetricResultValue},
};
use super::support::{
    eastbound_knot, eastbound_trajectory, eastbound_trajectory_with_internal_gap, test_trajectory,
};
use crate::{
    frame::{EcefPosition, EcefVelocity},
    ids::{MetricDefinitionId, ReferencePointId, TargetId},
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{DurationNs, SessionTime},
};

#[cfg(feature = "offline")]
#[test]
fn bounded_offline_results_reject_excess_before_vec_growth() {
    let mut plan = MetricPlan::new(41);
    for definition in [7, 8] {
        plan.push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(definition),
            quantity: DistanceQuantity::HorizontalPath,
            reference_point: ReferencePointId::new(1),
            absolute_tolerance_m: 0.01,
            relative_tolerance: 1.0e-6,
        }))
        .unwrap();
    }
    let trajectory = eastbound_trajectory(1.0, 1.0, 1.0);
    let mut results = MetricResults::new();
    results.try_prepare_bounded(1).unwrap();
    let reserved_capacity = results.values.capacity();

    assert_eq!(
        plan.evaluate_into(&trajectory, &mut results),
        Err(MetricError::CapacityExceeded)
    );
    assert!(results.is_empty());
    assert_eq!(results.values.capacity(), reserved_capacity);

    results.try_prepare_bounded(2).unwrap();
    plan.evaluate_into(&trajectory, &mut results).unwrap();
    assert_eq!(results.len(), 2);
}

#[test]
fn offline_distance_validity_covers_the_exact_integrated_span() {
    let trajectory = eastbound_trajectory_with_internal_gap();
    let mut plan = MetricPlan::new(144);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    let MetricResultValue::Distance(report) = results.as_slice()[0].value else {
        panic!("wrong result kind");
    };
    assert_eq!(report.span, trajectory.span().unwrap());
    assert_eq!(report.validity, Validity::Degraded);
}

#[test]
fn offline_activity_and_split_validity_cover_their_exact_spans() {
    let trajectory = eastbound_trajectory_with_internal_gap();
    let mut activity = ActivityPlan::new(MetricDefinitionId::new(2), ReferencePointId::new(1));
    activity.push_split(25.0).unwrap();
    let mut plan = MetricPlan::new(145);
    plan.push(MetricDefinition::Activity(activity)).unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    let activity = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::Activity(report) => Some(report),
            _ => None,
        })
        .unwrap();
    let split = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::ActivitySplit(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert_eq!(activity.validity, Validity::Degraded);
    assert_eq!(split.validity, Validity::Degraded);
}

#[test]
fn offline_drag_gap_span_degrades_result_and_event_timing() {
    let trajectory = eastbound_trajectory_with_internal_gap();
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 25.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(146);
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
    assert_eq!(report.validity, Validity::Degraded);
    assert_eq!(
        report.event_time_one_sigma_s,
        FieldValue::Unavailable(UnavailableReason::IllConditioned)
    );
    assert_eq!(
        report.elapsed_one_sigma_s,
        FieldValue::Unavailable(UnavailableReason::IllConditioned)
    );
}

#[cfg(feature = "offline")]
#[test]
fn offline_ski_results_cover_each_segment_and_summary_span() {
    let trajectory = eastbound_trajectory_with_internal_gap();
    let model = SkiHmmModel {
        initial_log_probability: [0.0; 5],
        transition_log_probability: [[0.0; 5]; 5],
        emission_bias: [0.0; 5],
        emission_weight: [[0.0; 3]; 5],
    };
    let mut plan = MetricPlan::new(147);
    plan.push(MetricDefinition::Ski(SkiPlan {
        definition: MetricDefinitionId::new(4),
        reference_point: ReferencePointId::new(1),
        sample_period: DurationNs::from_ns(1_000_000_000),
        minimum_segment_duration: DurationNs::from_ns(1_000_000_000),
        model,
    }))
    .unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    let segment = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::SkiSegment(report) => Some(report),
            _ => None,
        })
        .unwrap();
    let summary = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::Ski(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert_eq!(segment.start, SessionTime::ZERO);
    assert_eq!(segment.end, SessionTime::from_ns(3_000_000_000));
    assert_eq!(segment.validity, Validity::Degraded);
    assert_eq!(summary.validity, Validity::Degraded);
}

#[test]
fn ambiguous_definition_does_not_clear_or_block_unrelated_offline_results() {
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(0.0, 0.0, 0.0).unwrap();
    let mut end = start.clone();
    end.time = SessionTime::from_ns(1_000_000_000);
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut plan = MetricPlan::new(312);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(40),
        quantity: DistanceQuantity::BodyLongitudinalAbsolute,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(41),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    assert_eq!(results.len(), 2);
    let distance_result = results
        .as_slice()
        .iter()
        .find(|result| matches!(result.value, MetricResultValue::Distance(_)))
        .expect("the independent spatial-distance result must survive");
    let MetricResultValue::Distance(distance) = distance_result.value else {
        unreachable!();
    };
    assert_eq!(distance.definition, MetricDefinitionId::new(41));
    assert_eq!(distance.metres, 0.0);
    assert_eq!(distance_result.id.allocation(), 1);
    assert!(results.diagnostics().eq([MetricDefinitionDiagnostic {
        definition: MetricDefinitionId::new(40),
        reference_point: ReferencePointId::new(1),
        reason: MetricDefinitionDiagnosticReason::Ambiguous,
        stage: EstimateStage::Finalized,
        validity: Validity::Invalid,
    }]));
}

#[cfg(not(feature = "offline"))]
#[test]
fn unsupported_definition_does_not_clear_or_block_unrelated_offline_results() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let mut plan = MetricPlan::new(313);
    plan.push(MetricDefinition::Ski(SkiPlan {
        definition: MetricDefinitionId::new(42),
        reference_point: ReferencePointId::new(1),
        sample_period: DurationNs::from_ns(100_000_000),
        minimum_segment_duration: DurationNs::from_ns(100_000_000),
        model: SkiHmmModel {
            initial_log_probability: [0.0; 5],
            transition_log_probability: [[0.0; 5]; 5],
            emission_bias: [0.0; 5],
            emission_weight: [[0.0; 3]; 5],
        },
    }))
    .unwrap();
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(43),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    assert_eq!(results.len(), 2);
    assert!(results.as_slice().iter().any(
        |result| matches!(result.value, MetricResultValue::Distance(report) if report.definition == MetricDefinitionId::new(43))
    ));
    assert!(results.diagnostics().eq([MetricDefinitionDiagnostic {
        definition: MetricDefinitionId::new(42),
        reference_point: ReferencePointId::new(1),
        reason: MetricDefinitionDiagnosticReason::UnsupportedAtProcessingLevel,
        stage: EstimateStage::Finalized,
        validity: Validity::Invalid,
    }]));
}
