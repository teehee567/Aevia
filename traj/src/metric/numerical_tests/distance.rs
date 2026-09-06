//! Metric distance regression tests.

use super::super::{
    definition::{
        ActivityPlan, DistancePlan, DistanceQuantity, DragPlan, DragTarget, LaunchRule,
        MetricDefinition, SpeedQuantity,
    },
    plan::{LiveMetricLimits, MetricPlan},
    report::{MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason, MetricResultValue},
};
use super::support::{
    eastbound_knot, eastbound_trajectory, eastbound_trajectory_between, test_trajectory,
    with_large_test_stack,
};
use crate::{
    frame::{EcefPosition, EcefVelocity},
    ids::{LiveResultId, MetricDefinitionId, ReferencePointId, TargetId},
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::SessionTime,
};

fn push_curved_arc(trajectory: &mut crate::trajectory::Trajectory, index: i64, pieces: i64) {
    let direction = if index % 2 == 0 { 1.0 } else { -1.0 };
    let knot = |piece: i64| {
        let u = piece as f64 / pieces as f64;
        let mut knot = eastbound_knot(
            index * 1_000_000_000 + piece * 1_000_000_000 / pieces,
            0.0,
            0.0,
        );
        knot.position_ecef = EcefPosition::new(
            6_378_137.0 + index as f64 + u,
            5.0 * direction * (u * u - u),
            0.0,
        )
        .unwrap();
        knot.velocity_ecef = EcefVelocity::new(1.0, direction * (10.0 * u - 5.0), 0.0).unwrap();
        knot
    };
    for piece in 0..pieces {
        trajectory
            .push_hermite_segment(knot(piece), knot(piece + 1))
            .unwrap();
    }
}

#[test]
fn distance_tolerance_applies_to_the_whole_multisegment_measurement() {
    for pieces in [1, 4] {
        let mut trajectory = test_trajectory();
        for index in 0..16 {
            push_curved_arc(&mut trajectory, index, pieces);
        }
        let tolerance = 1.0e-5;
        let mut plan = MetricPlan::new(310);
        plan.push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(36),
            quantity: DistanceQuantity::Spatial3d,
            reference_point: ReferencePointId::new(1),
            absolute_tolerance_m: tolerance,
            relative_tolerance: 0.0,
        }))
        .unwrap();
        let results = plan.evaluate(&trajectory).unwrap();
        let MetricResultValue::Distance(report) = results.as_slice()[0].value else {
            panic!("distance expected")
        };
        assert!(
            report.numerical_error_m <= tolerance,
            "reported error {} exceeds whole-distance tolerance {}",
            report.numerical_error_m,
            tolerance
        );
        let expected = 16.0 * (5.0 * 26.0_f64.sqrt() + 5.0_f64.asinh()) / 10.0;
        assert!((report.metres - expected).abs() <= tolerance);
    }
}

#[test]
fn cumulative_live_distance_consumes_one_absolute_error_allowance() {
    use super::super::{
        distance::live_distance_report,
        numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    };
    let plan = DistancePlan {
        definition: MetricDefinitionId::new(37),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-5,
        relative_tolerance: 0.0,
    };
    let mut previous = None;
    for index in 0..16 {
        let mut trajectory = test_trajectory();
        push_curved_arc(&mut trajectory, index, 1);
        let limits = MetricEvaluationLimits::default();
        let report = live_distance_report(
            &trajectory,
            plan,
            previous,
            SessionTime::from_ns(index * 1_000_000_000),
            SessionTime::from_ns((index + 1) * 1_000_000_000),
            true,
            limits,
            &mut NumericalWorkBudget::from_limits(limits),
        )
        .unwrap();
        assert!(report.numerical_error_m <= plan.absolute_tolerance_m);
        previous = Some(report);
    }
    let expected = 16.0 * (5.0 * 26.0_f64.sqrt() + 5.0_f64.asinh()) / 10.0;
    assert!((previous.unwrap().metres - expected).abs() <= plan.absolute_tolerance_m);
}

#[test]
fn signed_distance_relative_allowance_uses_the_net_measurement() {
    use crate::{
        frame::OrientationEcefFromBody,
        math::{UnitQuaternion, Vector3},
    };
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(1_000.0, 0.0, 0.0).unwrap();
    let mut middle = start;
    middle.time = SessionTime::from_ns(1_000_000_000);
    middle.position_ecef = EcefPosition::new(6_379_137.0, 0.0, 0.0).unwrap();
    middle.orientation_ecef_from_body = OrientationEcefFromBody::from_quaternion(
        UnitQuaternion::from_rotation_vector(Vector3::new(0.0, 0.0, 8.0).unwrap()).unwrap(),
    );
    trajectory
        .push_rolling_imu_segment(start, middle, [0.0, 0.0, 8.0])
        .unwrap();
    // The signed body distance of the first rotating segment is
    // 1000*sin(8)/8. The second continuous segment cancels that distance.
    let displacement = -1_000.0 * 8.0_f64.sin() / (8.0 * 8.0_f64.cos());
    let mut end = middle;
    end.time = SessionTime::from_ns(2_000_000_000);
    end.position_ecef = EcefPosition::new(6_379_137.0 + displacement, 0.0, 0.0).unwrap();
    end.velocity_ecef = EcefVelocity::new(2.0 * displacement - 1_000.0, 0.0, 0.0).unwrap();
    trajectory.push_hermite_segment(middle, end).unwrap();
    let mut plan = MetricPlan::new(311);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(38),
        quantity: DistanceQuantity::BodyLongitudinalSigned,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-7,
        relative_tolerance: 0.1,
    }))
    .unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    let MetricResultValue::Distance(report) = results.as_slice()[0].value else {
        panic!("distance expected")
    };
    assert!(report.numerical_error_m <= 1.0e-7_f64.max(0.1 * report.metres.abs()));
    assert!(report.metres.abs() <= 1.0e-7);
}

#[test]
fn continuous_distance_is_independent_of_export_sampling() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let mut plan = MetricPlan::new(44);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    assert_eq!(results.len(), 1);
    let MetricResultValue::Distance(report) = results.as_slice()[0].value else {
        panic!("wrong result kind");
    };
    assert!((report.metres - 10.0).abs() < 1.0e-5);
    assert_eq!(
        report.uncertainty_one_sigma_m,
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    );
    assert_eq!(results.as_slice()[0].id, LiveResultId::new(44, 0));
}

#[test]
fn signed_longitudinal_target_finds_an_interior_tangent() {
    // Over one second, identity attitude makes body-x equal ECEF-x. The
    // Hermite path is p(u) = 2 a u - u^2, whose signed displacement only
    // touches the target a^2 at u=a before reversing. Choosing `a` halfway
    // between the legacy 64-cell scan points makes endpoint-sign scanning
    // miss the event deterministically.
    let a = 0.5 / 64.0;
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(2.0 * a, 0.0, 0.0).unwrap();
    let mut end = eastbound_knot(1_000_000_000, 0.0, 0.0);
    end.position_ecef = EcefPosition::new(6_378_137.0 + 2.0 * a - 1.0, 0.0, 0.0).unwrap();
    end.velocity_ecef = EcefVelocity::new(2.0 * a - 2.0, 0.0, 0.0).unwrap();
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut drag = DragPlan::new(
        MetricDefinitionId::new(30),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::BodyLongitudinalSigned,
        metres: a * a,
    })
    .unwrap();
    let mut plan = MetricPlan::new(304);
    plan.push(MetricDefinition::Drag(drag)).unwrap();

    let results = plan.evaluate(&trajectory).unwrap();
    let target = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::DragTarget(report) => Some(report),
            _ => None,
        })
        .expect("the tangent signed-displacement target must be isolated");
    assert!((target.event_time.as_ns() - 7_812_500).abs() <= 2);
}

#[test]
fn absolute_longitudinal_distance_splits_a_narrow_sign_reversal() {
    // v(u)=k(u-a)(u-b) is negative only in a narrow interval between two
    // Gauss-Kronrod abscissae. Integrating abs(v) without first isolating
    // the two zeros samples only the positive polynomial and reports a
    // spuriously tiny quadrature error.
    let (a, b, k) = (0.24_f64, 0.26_f64, 100.0_f64);
    let signed_displacement = k * (1.0 / 3.0 - (a + b) / 2.0 + a * b);
    let reversal_area = k * (b - a).powi(3) / 6.0;
    let expected_absolute_distance = signed_displacement + 2.0 * reversal_area;
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(k * a * b, 0.0, 0.0).unwrap();
    let mut end = eastbound_knot(1_000_000_000, 0.0, 0.0);
    end.position_ecef = EcefPosition::new(6_378_137.0 + signed_displacement, 0.0, 0.0).unwrap();
    end.velocity_ecef = EcefVelocity::new(k * (1.0 - a) * (1.0 - b), 0.0, 0.0).unwrap();
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut plan = MetricPlan::new(306);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(32),
        quantity: DistanceQuantity::BodyLongitudinalAbsolute,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-10,
        relative_tolerance: 1.0e-12,
    }))
    .unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    let MetricResultValue::Distance(report) = results.as_slice()[0].value else {
        panic!("wrong result kind");
    };
    assert!((report.metres - expected_absolute_distance).abs() < 1.0e-8);
}

#[test]
fn absolute_longitudinal_target_uses_sign_split_cumulative_distance() {
    let (a, b, k) = (0.2495_f64, 0.2505_f64, 1_000_000.0_f64);
    let signed_at = |u: f64| k * (u.powi(3) / 3.0 - (a + b) * u.powi(2) / 2.0 + a * b * u);
    let signed_displacement = signed_at(1.0);
    let half_reversal_area = k * (b - a).powi(3) / 12.0;
    let target_metres = signed_at(a) + half_reversal_area;
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(k * a * b, 0.0, 0.0).unwrap();
    let mut end = eastbound_knot(1_000_000_000, 0.0, 0.0);
    end.position_ecef = EcefPosition::new(6_378_137.0 + signed_displacement, 0.0, 0.0).unwrap();
    end.velocity_ecef = EcefVelocity::new(k * (1.0 - a) * (1.0 - b), 0.0, 0.0).unwrap();
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut drag = DragPlan::new(
        MetricDefinitionId::new(33),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Distance {
        id: TargetId::new(1),
        quantity: DistanceQuantity::BodyLongitudinalAbsolute,
        metres: target_metres,
    })
    .unwrap();
    let mut plan = MetricPlan::new(307);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let target = plan
        .evaluate(&trajectory)
        .unwrap()
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::DragTarget(report) => Some(report),
            _ => None,
        })
        .expect("absolute-distance target must be found");
    assert!((target.event_time.as_ns() - 250_000_000).abs() <= 10);
}

#[test]
fn absolute_longitudinal_distance_reports_ambiguous_for_unresolved_zero_span() {
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(0.0, 0.0, 0.0).unwrap();
    let mut end = start.clone();
    end.time = SessionTime::from_ns(1_000_000_000);
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut plan = MetricPlan::new(308);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(34),
        quantity: DistanceQuantity::BodyLongitudinalAbsolute,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-10,
    }))
    .unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    assert_eq!(results.len(), 1);
    assert!(results.diagnostics().eq([MetricDefinitionDiagnostic {
        definition: MetricDefinitionId::new(34),
        reference_point: ReferencePointId::new(1),
        reason: MetricDefinitionDiagnosticReason::Ambiguous,
        stage: EstimateStage::Finalized,
        validity: Validity::Invalid,
    }]));
}

#[test]
fn signed_longitudinal_peak_remains_negative_for_an_all_reverse_span() {
    // p(t)=-5t+t^2, so body-longitudinal speed rises from -5 to -3 m/s.
    // The maximum signed value is therefore the terminal -3 m/s, not zero.
    let mut trajectory = test_trajectory();
    let mut start = eastbound_knot(0, 0.0, 0.0);
    start.position_ecef = EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap();
    start.velocity_ecef = EcefVelocity::new(-5.0, 0.0, 0.0).unwrap();
    let mut end = eastbound_knot(1_000_000_000, 0.0, 0.0);
    end.position_ecef = EcefPosition::new(6_378_133.0, 0.0, 0.0).unwrap();
    end.velocity_ecef = EcefVelocity::new(-3.0, 0.0, 0.0).unwrap();
    trajectory.push_hermite_segment(start, end).unwrap();

    let mut activity = ActivityPlan::new(MetricDefinitionId::new(35), ReferencePointId::new(1));
    activity.include_horizontal_distance = false;
    activity.include_spatial_distance = false;
    activity.moving_speed = SpeedQuantity::BodyLongitudinalMagnitude;
    activity.peak_speed = SpeedQuantity::BodyLongitudinalSigned;
    let mut plan = MetricPlan::new(309);
    plan.push(MetricDefinition::Activity(activity)).unwrap();
    let results = plan.evaluate(&trajectory).unwrap();
    let report = results
        .as_slice()
        .iter()
        .find_map(|result| match result.value {
            MetricResultValue::Activity(report) => Some(report),
            _ => None,
        })
        .expect("activity report must be present");
    assert!((report.peak_speed_mps + 3.0).abs() < 1.0e-9);
}

#[test]
fn cumulative_live_distance_does_not_shrink_with_the_rolling_window() {
    with_large_test_stack(cumulative_live_distance_does_not_shrink_with_the_rolling_window_inner);
}

fn cumulative_live_distance_does_not_shrink_with_the_rolling_window_inner() {
    let first = std::boxed::Box::new(eastbound_trajectory_between(
        0,
        1_000_000_000,
        0.0,
        10.0,
        9.0,
        11.0,
    ));
    let second = std::boxed::Box::new(eastbound_trajectory_between(
        1_000_000_000,
        2_000_000_000,
        10.0,
        20.0,
        9.0,
        11.0,
    ));
    let mut plan = MetricPlan::new(92);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-7,
        relative_tolerance: 1.0e-9,
    }))
    .unwrap();
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

    let mut distance = None;
    for result in tracker.active_results() {
        if let MetricResultValue::Distance(report) = result.value {
            distance = Some(report);
        }
    }
    let distance = distance.unwrap();
    assert!((distance.metres - 20.0).abs() < 1.0e-5);
    assert_eq!(distance.span.start(), SessionTime::ZERO);
    assert_eq!(distance.span.end(), SessionTime::from_ns(2_000_000_000));
}
