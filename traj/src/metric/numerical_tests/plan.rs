//! Metric plan regression tests.

use super::super::{
    definition::{
        ActivityPlan, CrossingDirection, DistancePlan, DistanceQuantity, DragPlan, DragTarget,
        FiniteGate, LapPlan, LaunchRule, MetricDefinition, Rollout, SkiHmmModel, SkiPlan,
        SpeedQuantity, TargetDirection,
    },
    geometry::{dot, norm},
    plan::{LiveMetricLimits, LiveMetricPlan, MetricPlan},
    report::{MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason, MetricResultValue},
};
use super::support::{eastbound_knot, test_trajectory_with_attachment};
use crate::{
    config::AttachmentModel,
    error::ValidationError,
    frame::{BodyLeverArm, ReferencePoint, ReferencePointKind},
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId, SharedParameterId, TargetId},
    quality::{EstimateStage, Validity},
    time::{DurationNs, SessionTime},
    uncertainty::{Covariance3, MeasurementUncertainty},
};

#[test]
fn device_attachment_keeps_antenna_metric_and_withdraws_body_and_rigid_point_claims() {
    let mut trajectory = test_trajectory_with_attachment(
        AttachmentModel::DeviceTrajectoryOnly,
        ReferencePointKind::GnssAntennaPhaseCenter,
    );
    trajectory
        .add_reference_point(ReferencePoint::new(
            ReferencePointId::new(2),
            ReferencePointKind::RigidBodyPoint,
            BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
            SharedParameterId::new(2),
            MeasurementUncertainty::Provided(Covariance3::ZERO),
        ))
        .unwrap();
    trajectory
        .push_hermite_segment(
            eastbound_knot(0, 0.0, 10.0),
            eastbound_knot(1_000_000_000, 10.0, 10.0),
        )
        .unwrap();

    let mut plan = MetricPlan::new(100);
    for (definition, quantity, reference_point) in [
        (
            1,
            DistanceQuantity::BodyLongitudinalSigned,
            ReferencePointId::new(1),
        ),
        (2, DistanceQuantity::Spatial3d, ReferencePointId::new(2)),
        (3, DistanceQuantity::Spatial3d, ReferencePointId::new(1)),
    ] {
        plan.push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(definition),
            quantity,
            reference_point,
            absolute_tolerance_m: 1.0e-8,
            relative_tolerance: 1.0e-10,
        }))
        .unwrap();
    }

    let results = plan.evaluate(&trajectory).unwrap();
    let antenna = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::Distance(report) => Some(report),
            _ => None,
        })
        .expect("antenna spatial distance must remain available");
    assert_eq!(results.len(), 3);
    assert_eq!(antenna.definition, MetricDefinitionId::new(3));
    assert!((antenna.metres - 10.0).abs() < 1.0e-8);
    assert!(results.diagnostics().eq([
        MetricDefinitionDiagnostic {
            definition: MetricDefinitionId::new(1),
            reference_point: ReferencePointId::new(1),
            reason: MetricDefinitionDiagnosticReason::AttachmentModelUnavailable,
            stage: EstimateStage::Finalized,
            validity: Validity::Invalid,
        },
        MetricDefinitionDiagnostic {
            definition: MetricDefinitionId::new(2),
            reference_point: ReferencePointId::new(2),
            reason: MetricDefinitionDiagnosticReason::AttachmentModelUnavailable,
            stage: EstimateStage::Finalized,
            validity: Validity::Invalid,
        },
    ]));
}

#[test]
fn attachment_classifier_finds_body_axis_use_in_every_nested_metric_field() {
    let point = ReferencePointId::new(1);
    let body_speed = SpeedQuantity::BodyLongitudinalMagnitude;
    let body_distance = DistanceQuantity::BodyLongitudinalAbsolute;

    let distance = MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(11),
        quantity: body_distance,
        reference_point: point,
        absolute_tolerance_m: 0.01,
        relative_tolerance: 1.0e-6,
    });
    let lap = MetricDefinition::Lap(LapPlan::new(
        MetricDefinitionId::new(12),
        point,
        Some(body_speed),
    ));
    let mut launch_drag = DragPlan::new(
        MetricDefinitionId::new(13),
        point,
        LaunchRule::SpeedThreshold {
            quantity: body_speed,
            threshold_mps: 1.0,
            dwell: DurationNs::ZERO,
        },
    );
    launch_drag
        .push_target(DragTarget::Distance {
            id: TargetId::new(1),
            quantity: DistanceQuantity::Spatial3d,
            metres: 1.0,
        })
        .unwrap();
    let mut rollout_drag = DragPlan::new(
        MetricDefinitionId::new(14),
        point,
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    rollout_drag.rollout = Rollout::Distance {
        quantity: body_distance,
        metres: 0.1,
    };
    rollout_drag
        .push_target(DragTarget::Distance {
            id: TargetId::new(2),
            quantity: DistanceQuantity::Spatial3d,
            metres: 1.0,
        })
        .unwrap();
    let mut target_drag = DragPlan::new(
        MetricDefinitionId::new(15),
        point,
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    target_drag
        .push_target(DragTarget::Speed {
            id: TargetId::new(3),
            quantity: body_speed,
            metres_per_second: 1.0,
            direction: TargetDirection::Ascending,
        })
        .unwrap();
    let mut activity = ActivityPlan::new(MetricDefinitionId::new(16), point);
    activity.moving_speed = body_speed;

    for definition in [
        distance,
        lap,
        MetricDefinition::Drag(launch_drag),
        MetricDefinition::Drag(rollout_drag),
        MetricDefinition::Drag(target_drag),
        MetricDefinition::Activity(activity),
    ] {
        assert!(definition.requires_body_axis_quantities());
        assert!(!definition.is_permitted_by_attachment(
            AttachmentModel::DeviceTrajectoryOnly,
            ReferencePointKind::GnssAntennaPhaseCenter,
        ));
    }
}

#[test]
fn nonzero_activity_peak_window_is_rejected() {
    let mut activity = ActivityPlan::new(MetricDefinitionId::new(5), ReferencePointId::new(1));
    activity.peak_window = DurationNs::from_ns(1);
    let mut plan = MetricPlan::new(105);
    assert_eq!(
        plan.push(MetricDefinition::Activity(activity)),
        Err(ValidationError::InvalidMetricDefinition)
    );
    assert!(
        plan.push(MetricDefinition::Activity(ActivityPlan::new(
            MetricDefinitionId::new(5),
            ReferencePointId::new(1),
        )))
        .is_ok()
    );
}

#[test]
fn descending_drag_target_above_stop_threshold_is_rejected() {
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(6),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(3),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 1.0,
        direction: TargetDirection::Descending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(106);
    assert_eq!(
        plan.push(MetricDefinition::Drag(drag)),
        Err(ValidationError::InvalidMetricDefinition)
    );
}

#[test]
fn gate_constructor_orthonormalizes_basis() {
    let gate = FiniteGate::new(
        GateId::new(1),
        FrameId::new(2),
        [1.0, 2.0, 3.0],
        [2.0, 0.0, 0.0],
        [1.0, 3.0, 0.0],
        10.0,
        4.0,
        CrossingDirection::Either,
        0.1,
        1.0,
        DurationNs::ZERO,
        Some(0.01),
    )
    .unwrap();
    assert!((dot(gate.normal_ecef, gate.width_axis_ecef)).abs() < 1.0e-14);
    assert!((dot(gate.normal_ecef, gate.height_axis_ecef)).abs() < 1.0e-14);
    assert!((norm(gate.width_axis_ecef) - 1.0).abs() < 1.0e-14);
}

#[test]
fn live_compile_rejects_offline_ski_model() {
    let model = SkiHmmModel {
        initial_log_probability: [0.0; 5],
        transition_log_probability: [[0.0; 5]; 5],
        emission_bias: [0.0; 5],
        emission_weight: [[0.0; 3]; 5],
    };
    let mut plan = MetricPlan::new(7);
    plan.push(MetricDefinition::Ski(SkiPlan {
        definition: MetricDefinitionId::new(1),
        reference_point: ReferencePointId::new(1),
        sample_period: DurationNs::from_ns(100_000_000),
        minimum_segment_duration: DurationNs::from_ns(100_000_000),
        model,
    }))
    .unwrap();
    assert_eq!(
        plan.compile_live(LiveMetricLimits::default()),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn live_compile_accepts_supported_activity_outputs() {
    let mut plan = MetricPlan::new(8);
    plan.push(MetricDefinition::Activity(ActivityPlan::new(
        MetricDefinitionId::new(1),
        ReferencePointId::new(1),
    )))
    .unwrap();

    assert!(plan.compile_live(LiveMetricLimits::default()).is_ok());
}

#[test]
fn live_compile_reserves_the_worst_case_mutations_for_each_result_key() {
    let mut plan = MetricPlan::new(9);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 0.01,
        relative_tolerance: 1.0e-6,
    }))
    .unwrap();

    assert_eq!(
        plan.compile_live(LiveMetricLimits {
            max_mutations_per_step: 1,
            ..LiveMetricLimits::default()
        }),
        Err(ValidationError::CapacityExceeded)
    );
    assert!(
        plan.compile_live(LiveMetricLimits {
            max_mutations_per_step: 2,
            ..LiveMetricLimits::default()
        })
        .is_ok()
    );
}

#[test]
fn live_compile_enforces_the_active_candidate_contract() {
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

    assert_eq!(
        plan.compile_live(LiveMetricLimits {
            max_active_candidates: 1,
            ..LiveMetricLimits::default()
        }),
        Err(ValidationError::CapacityExceeded)
    );
    assert!(
        plan.compile_live(LiveMetricLimits {
            max_active_candidates: 2,
            ..LiveMetricLimits::default()
        })
        .is_ok()
    );
}

#[test]
fn live_compile_rejects_unrepresentable_or_unretained_lookahead() {
    let drag_plan = |launch_dwell, stop_dwell| {
        let mut drag = DragPlan::new(
            MetricDefinitionId::new(1),
            ReferencePointId::new(1),
            LaunchRule::SpeedThreshold {
                quantity: SpeedQuantity::Spatial3d,
                threshold_mps: 1.0,
                dwell: launch_dwell,
            },
        );
        drag.stop_dwell = stop_dwell;
        drag.push_target(DragTarget::Distance {
            id: TargetId::new(1),
            quantity: DistanceQuantity::HorizontalPath,
            metres: 1.0,
        })
        .unwrap();
        let mut plan = MetricPlan::new(11);
        plan.push(MetricDefinition::Drag(drag)).unwrap();
        plan
    };

    assert_eq!(
        drag_plan(DurationNs::from_ns(i64::MAX as u64 + 1), DurationNs::ZERO,)
            .compile_live(LiveMetricLimits::default()),
        Err(ValidationError::TimeOutOfRange)
    );
    assert_eq!(
        drag_plan(
            DurationNs::ZERO,
            DurationNs::from_ns(crate::live::MAX_HISTORY_HORIZON_NS + 1),
        )
        .compile_live(LiveMetricLimits::default()),
        Err(ValidationError::CapacityExceeded)
    );
}

#[test]
fn live_compile_rejects_signed_longitudinal_distance_target() {
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(31),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::BodyLongitudinalSigned,
        metres: 1.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(305);
    plan.push(MetricDefinition::Drag(drag)).unwrap();

    assert_eq!(
        plan.compile_live(LiveMetricLimits::default()),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn live_compile_rejects_signed_longitudinal_rollout() {
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(37),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.rollout = Rollout::Distance {
        quantity: DistanceQuantity::BodyLongitudinalSigned,
        metres: 0.3048,
    };
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        metres: 100.0,
    })
    .unwrap();
    let mut plan = MetricPlan::new(311);
    plan.push(MetricDefinition::Drag(drag)).unwrap();

    assert_eq!(
        plan.compile_live(LiveMetricLimits::default()),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn live_compile_into_reuses_storage_and_resets_it_on_failure() {
    let mut plan = MetricPlan::new(37);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(5),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(2),
        absolute_tolerance_m: 0.01,
        relative_tolerance: 1.0e-6,
    }))
    .unwrap();
    let mut output = LiveMetricPlan::placeholder();

    plan.compile_live_into(LiveMetricLimits::default(), &mut output)
        .unwrap();
    assert_eq!(output.plan().run_namespace(), 37);
    assert_eq!(output.plan().definitions(), plan.definitions());

    let insufficient = LiveMetricLimits {
        max_results: 0,
        ..LiveMetricLimits::default()
    };
    assert_eq!(
        plan.compile_live_into(insufficient, &mut output),
        Err(ValidationError::CapacityExceeded)
    );
    assert_eq!(output.plan().run_namespace(), 0);
    assert!(output.plan().definitions().is_empty());
    assert_eq!(output.maximum_lookahead(), DurationNs::ZERO);
    assert_eq!(output.limits(), LiveMetricPlan::PLACEHOLDER_LIMITS);
}

#[test]
fn live_compile_preflights_lap_occurrence_tombstones() {
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
    let mut lap = LapPlan::new(MetricDefinitionId::new(4), ReferencePointId::new(1), None);
    lap.push_gate(gate).unwrap();
    let mut plan = MetricPlan::new(7);
    plan.push(MetricDefinition::Lap(lap.clone())).unwrap();
    let limits = LiveMetricLimits {
        max_results: 31,
        ..LiveMetricLimits::default()
    };
    // Sixteen possible crossings and sixteen lap identities must all fit,
    // even though only a small subset may be active at one instant.
    assert_eq!(
        plan.compile_live(limits),
        Err(ValidationError::CapacityExceeded)
    );

    assert_eq!(
        lap.set_maximum_occurrences_per_gate(0),
        Err(ValidationError::InvalidMetricDefinition)
    );
    // Four crossings plus four lap identities also fit the conservative
    // two-mutations-per-key publication bound (8 keys, 16 mutations).
    lap.set_maximum_occurrences_per_gate(4).unwrap();
    let mut bounded = MetricPlan::new(7);
    bounded.push(MetricDefinition::Lap(lap)).unwrap();
    assert!(bounded.compile_live(limits).is_ok());
}
