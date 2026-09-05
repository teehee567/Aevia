//! Metric uncertainty regression tests.

#[cfg(test)]
use super::super::shared_gate_survey_time_covariance_s2;
use super::super::{
    EventTimeSensitivity, MetricUncertaintyProvider, StateSensitivity,
    definition::{
        CrossingDirection, DragPlan, DragTarget, FiniteGate, LapPlan, LaunchRule, MetricDefinition,
        SpeedQuantity, TargetDirection,
    },
    geometry::{dot, norm, scale, sub},
    plan::MetricPlan,
    report::{GateCrossingReport, MetricResultValue},
    uncertainty::{
        TrajectoryMarginalUncertainty, ellipsoid_up_with_jacobian, lap_elapsed_one_sigma,
        speed_event_sensitivity, speed_state_sensitivity,
    },
};
use super::support::{eastbound_trajectory, test_covariance};
use crate::{
    frame::{OrientationEcefFromBody, ReferenceEllipsoid},
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId, TargetId},
    math::UnitQuaternion,
    quality::{EstimateQuality, EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{DurationNs, SessionTime},
    trajectory::{ScalarKinematics, Trajectory},
    uncertainty::KinematicCovariance,
};

#[test]
fn endpoint_gate_survey_without_declared_state_correlation_fails_closed() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let gate = FiniteGate::new(
        GateId::new(1),
        FrameId::new(1),
        [6_378_137.0, 10.0, 0.0],
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
        Some(SpeedQuantity::Spatial3d),
    );
    lap.push_gate(gate).unwrap();
    let mut plan = MetricPlan::new(145);
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
    assert_eq!(
        crossing.time_one_sigma_s,
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    );
    assert_eq!(
        crossing.crossing_speed_one_sigma_mps,
        Some((
            SpeedQuantity::Spatial3d,
            FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
        ))
    );
}

#[test]
fn endpoint_speed_event_uses_speed_gradient_and_slope() {
    // v(t)=20t, so a 0.1 m/s speed sigma becomes a 5 ms event-time sigma
    // at the 20 m/s endpoint through dt=-dq/(dq/dt).
    let trajectory = eastbound_trajectory(0.0, 20.0, 10.0);
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(3),
        ReferencePointId::new(1),
        LaunchRule::ExternalTimestamp(SessionTime::ZERO),
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 20.0,
        direction: TargetDirection::Ascending,
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
    assert_eq!(report.event_time, SessionTime::from_ns(1_000_000_000));
    assert_eq!(report.event_time_one_sigma_s, FieldValue::Available(0.005));
    assert_eq!(
        report.terminal_speed_one_sigma_mps,
        Some((SpeedQuantity::Spatial3d, FieldValue::Available(0.1)))
    );
    assert_eq!(
        report.elapsed_one_sigma_s,
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    );
}

#[test]
fn near_zero_implicit_derivative_is_ill_conditioned() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let state = trajectory
        .scalar_kinematics_at_parameter(0, 1.0, ReferencePointId::new(1))
        .unwrap();
    let estimate = trajectory
        .metric_estimate_at_parameter(0, 1.0, ReferencePointId::new(1))
        .unwrap();
    assert_eq!(
        speed_event_sensitivity(
            0,
            1.0,
            &state,
            SpeedQuantity::Spatial3d,
            1.0e-12,
            ReferencePointId::new(1),
            trajectory.frame().ellipsoid(),
            estimate.orientation_ecef_from_body,
        ),
        Err(UnavailableReason::IllConditioned)
    );
}

#[test]
fn horizontal_speed_analytic_gradient_matches_central_difference() {
    let ellipsoid = ReferenceEllipsoid::WGS84;
    let position = [4_200_000.0, 1_900_000.0, 4_400_000.0];
    let velocity = [31.0, -7.0, 4.0];
    let horizontal_speed = |position: [f64; 3], velocity: [f64; 3]| {
        let (up, _) = ellipsoid_up_with_jacobian(position, ellipsoid).unwrap();
        let horizontal = sub(velocity, scale(up, dot(up, velocity)));
        norm(horizontal)
    };
    let state = ScalarKinematics {
        time: SessionTime::ZERO,
        position_ecef_m: position,
        velocity_ecef_mps: velocity,
        acceleration_ecef_mps2: Some([0.0; 3]),
        horizontal_speed_mps: horizontal_speed(position, velocity),
        vertical_speed_mps: 0.0,
        body_longitudinal_speed_mps: None,
        quality: EstimateQuality::INVALID,
    };
    let gradient = speed_state_sensitivity(
        &state,
        SpeedQuantity::InstantaneousHorizontal,
        ellipsoid,
        OrientationEcefFromBody::from_quaternion(UnitQuaternion::IDENTITY),
    )
    .unwrap();
    for axis in 0..3 {
        let position_step = 0.1;
        let mut before = position;
        let mut after = position;
        before[axis] -= position_step;
        after[axis] += position_step;
        let numerical = (horizontal_speed(after, velocity) - horizontal_speed(before, velocity))
            / (2.0 * position_step);
        assert!((gradient.position[axis] - numerical).abs() < 1.0e-10);

        let velocity_step = 1.0e-5;
        let mut before = velocity;
        let mut after = velocity;
        before[axis] -= velocity_step;
        after[axis] += velocity_step;
        let numerical = (horizontal_speed(position, after) - horizontal_speed(position, before))
            / (2.0 * velocity_step);
        assert!((gradient.velocity[axis] - numerical).abs() < 1.0e-9);
    }
}

#[test]
fn reused_gate_survey_covariance_is_not_counted_as_independent() {
    #[derive(Clone, Copy)]
    struct SharedGateProvider;

    impl MetricUncertaintyProvider for SharedGateProvider {
        fn kinematic_covariance_at(
            &mut self,
            _trajectory: &Trajectory,
            _segment_index: usize,
            _parameter: f64,
            _reference_point: ReferencePointId,
        ) -> Result<KinematicCovariance, UnavailableReason> {
            Ok(test_covariance())
        }

        fn event_time_cross_covariance_s2(
            &mut self,
            _trajectory: &Trajectory,
            first: &EventTimeSensitivity,
            second: &EventTimeSensitivity,
        ) -> FieldValue<f64> {
            shared_gate_survey_time_covariance_s2(first, second).map_or_else(
                || FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
                FieldValue::Available,
            )
        }
    }

    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let sensitivity = EventTimeSensitivity {
        segment_index: 0,
        parameter: 1.0,
        time: SessionTime::from_ns(1_000_000_000),
        reference_point: ReferencePointId::new(1),
        state: StateSensitivity::ZERO,
        gate: Some(GateId::new(7)),
        gate_survey_coefficient_s_per_m: 0.1,
        gate_survey_variance_m2: Some(0.04),
    };
    let start = GateCrossingReport {
        definition: MetricDefinitionId::new(1),
        gate: GateId::new(7),
        time: SessionTime::ZERO,
        time_one_sigma_s: FieldValue::Available(0.03),
        normal_speed_mps: 10.0,
        crossing_speed: None,
        crossing_speed_one_sigma_mps: None,
        reference_point: ReferencePointId::new(1),
        occurrence: 0,
        stage: EstimateStage::Finalized,
        validity: Validity::Nominal,
    };
    let end = GateCrossingReport {
        time: SessionTime::from_ns(1_000_000_000),
        occurrence: 1,
        ..start
    };
    let mut provider = SharedGateProvider;
    let FieldValue::Available(sigma) = lap_elapsed_one_sigma(
        &trajectory,
        &mut provider,
        &start,
        Some(&sensitivity),
        &end,
        Some(&sensitivity),
    ) else {
        panic!("test provider supplies the complete pair covariance");
    };
    let shared = 0.1 * 0.1 * 0.04;
    assert!((sigma - (0.03_f64.powi(2) * 2.0 - 2.0 * shared).sqrt()).abs() < 1.0e-12);
    assert!(sigma < (2.0_f64 * 0.03_f64.powi(2)).sqrt());

    let mut marginal_only = TrajectoryMarginalUncertainty;
    assert_eq!(
        lap_elapsed_one_sigma(
            &trajectory,
            &mut marginal_only,
            &start,
            Some(&sensitivity),
            &end,
            Some(&sensitivity),
        ),
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    );
}
