//! Metric activity regression tests.

use super::super::{
    definition::{ActivityPlan, MetricDefinition},
    plan::{LiveMetricLimits, MetricPlan},
    report::MetricResultValue,
};
use super::support::{eastbound_trajectory, with_large_test_stack};
use crate::{
    ids::{MetricDefinitionId, ReferencePointId},
    quality::{FieldValue, UnavailableReason},
    time::SessionTime,
};

#[test]
fn live_activity_publishes_supported_totals_and_marks_vertical_totals_unavailable() {
    with_large_test_stack(
        live_activity_publishes_supported_totals_and_marks_vertical_totals_unavailable_inner,
    );
}

fn live_activity_publishes_supported_totals_and_marks_vertical_totals_unavailable_inner() {
    let trajectory = eastbound_trajectory(9.0, 11.0, 10.0);
    let mut activity = ActivityPlan::new(MetricDefinitionId::new(1), ReferencePointId::new(1));
    activity.push_split(5.0).unwrap();
    let mut plan = MetricPlan::new(208);
    plan.push(MetricDefinition::Activity(activity)).unwrap();
    let mut tracker = plan
        .compile_live(LiveMetricLimits::default())
        .unwrap()
        .start();
    tracker
        .update(&trajectory, SessionTime::from_ns(1_000_000_000), true)
        .unwrap();

    let report = tracker
        .active_results()
        .find_map(|result| match result.value {
            MetricResultValue::Activity(report) => Some(report),
            _ => None,
        })
        .unwrap();
    assert_eq!(report.elapsed_seconds, 1.0);
    assert!(report.moving_seconds > 0.99);
    assert!(matches!(
        report.horizontal_distance_m,
        FieldValue::Available(_)
    ));
    assert!(matches!(
        report.spatial_distance_m,
        FieldValue::Available(_)
    ));
    assert_eq!(
        report.ascent_m,
        FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
    );
    assert_eq!(
        report.descent_m,
        FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
    );
    assert!(
        tracker
            .active_results()
            .any(|result| { matches!(result.value, MetricResultValue::ActivitySplit(_)) })
    );
}
