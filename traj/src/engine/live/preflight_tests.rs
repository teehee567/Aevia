//! Regression tests for live root preflight tests.

use super::live_definition_needs_non_polynomial_roots;
use crate::{
    ids::{MetricDefinitionId, ReferencePointId, TargetId},
    metric::{
        ActivityPlan, DistancePlan, DistanceQuantity, DragPlan, DragTarget, LapPlan, LaunchRule,
        MetricDefinition, SpeedQuantity, TargetDirection,
    },
    time::SessionTime,
};

#[test]
fn classifier_keeps_only_origin_spatial_roots_on_the_polynomial_path() {
    let reference = ReferencePointId::new(1);
    let spatial_distance = MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: reference,
        absolute_tolerance_m: 1.0e-4,
        relative_tolerance: 1.0e-6,
    });
    assert!(!live_definition_needs_non_polynomial_roots(
        &spatial_distance,
        false
    ));
    assert!(live_definition_needs_non_polynomial_roots(
        &spatial_distance,
        true
    ));

    let horizontal_distance = MetricDefinition::Distance(DistancePlan {
        quantity: DistanceQuantity::HorizontalPath,
        ..match spatial_distance {
            MetricDefinition::Distance(plan) => plan,
            _ => unreachable!(),
        }
    });
    assert!(live_definition_needs_non_polynomial_roots(
        &horizontal_distance,
        false
    ));

    let lap = MetricDefinition::Lap(LapPlan::new(MetricDefinitionId::new(2), reference, None));
    assert!(!live_definition_needs_non_polynomial_roots(&lap, false));
    let horizontal_lap = MetricDefinition::Lap(LapPlan::new(
        MetricDefinitionId::new(3),
        reference,
        Some(SpeedQuantity::InstantaneousHorizontal),
    ));
    assert!(live_definition_needs_non_polynomial_roots(
        &horizontal_lap,
        false
    ));

    let mut drag = DragPlan::new(
        MetricDefinitionId::new(4),
        reference,
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 10.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    assert!(!live_definition_needs_non_polynomial_roots(
        &MetricDefinition::Drag(drag.clone()),
        false
    ));
    let mut distance_drag = DragPlan::new(
        MetricDefinitionId::new(6),
        reference,
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    distance_drag
        .push_target(DragTarget::Distance {
            id: TargetId::new(3),
            quantity: DistanceQuantity::Spatial3d,
            metres: 100.0,
        })
        .unwrap();
    assert!(live_definition_needs_non_polynomial_roots(
        &MetricDefinition::Drag(distance_drag),
        false
    ));
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(2),
        quantity: SpeedQuantity::BodyLongitudinalSigned,
        metres_per_second: 11.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    assert!(live_definition_needs_non_polynomial_roots(
        &MetricDefinition::Drag(drag),
        false
    ));

    let mut activity = ActivityPlan::new(MetricDefinitionId::new(5), reference);
    activity.include_horizontal_distance = false;
    activity.moving_speed = SpeedQuantity::Spatial3d;
    activity.peak_speed = SpeedQuantity::Spatial3d;
    assert!(!live_definition_needs_non_polynomial_roots(
        &MetricDefinition::Activity(activity.clone()),
        false
    ));
    activity.moving_speed = SpeedQuantity::InstantaneousHorizontal;
    assert!(live_definition_needs_non_polynomial_roots(
        &MetricDefinition::Activity(activity),
        false
    ));
}
