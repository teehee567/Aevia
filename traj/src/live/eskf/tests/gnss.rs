//! Gnss regression coverage.

use super::super::{EskfError, covariance::matrix3_is_psd, gnss::UpdateDecision};
use super::support::{context, filter, gate, position_observation};
use crate::{
    live::state::{MechanizationContext, POS, skew, so3_exp},
    time::SessionTime,
};
use nalgebra::{Matrix3, Vector3};

#[test]
fn position_update_moves_state_and_reduces_variance() {
    let mut filter = filter();
    let before = filter.covariance[(POS, POS)];
    let result = filter
        .update_gnss(
            &position_observation(SessionTime::ZERO, Vector3::new(0.2, 0.0, 0.0)),
            &context(),
            gate(),
        )
        .unwrap();
    assert!(matches!(
        result.position,
        Some(UpdateDecision::Fused { .. })
    ));
    assert!(filter.state.position_n.x > 0.1);
    assert!(filter.covariance[(POS, POS)] < before);
    assert!((filter.covariance - filter.covariance.transpose()).norm() < 1.0e-5);
}

#[test]
fn lever_arm_velocity_timing_fails_closed_without_angular_acceleration() {
    let mut filter = filter();
    let before = filter;
    let mut observation = position_observation(SessionTime::ZERO, Vector3::zeros());
    observation.position_n = None;
    observation.velocity_n = Some(Vector3::zeros());
    observation.imu_to_antenna_b = Vector3::new(0.4, -0.2, 0.1);
    observation.velocity_independent_timing_sigma_s = 0.001;
    assert_eq!(
        filter.update_gnss(&observation, &context(), gate()),
        Err(EskfError::MissingAngularAccelerationForTiming)
    );
    assert_eq!(filter, before);
}

#[test]
fn measurement_covariance_psd_check_accepts_tiny_semidefinite_but_not_indefinite() {
    let tiny_rank_one = Matrix3::new(
        1.0e-30, 1.0e-30, 0.0, //
        1.0e-30, 1.0e-30, 0.0, //
        0.0, 0.0, 0.0,
    );
    let tiny_indefinite = Matrix3::new(
        1.0e-30, 2.0e-30, 0.0, //
        2.0e-30, 1.0e-30, 0.0, //
        0.0, 0.0, 0.0,
    );
    assert!(matrix3_is_psd(&tiny_rank_one));
    assert!(!matrix3_is_psd(&tiny_indefinite));
}

#[test]
fn hard_outlier_does_not_mutate_filter() {
    let mut filter = filter();
    let before = filter;
    let result = filter
        .update_gnss(
            &position_observation(SessionTime::ZERO, Vector3::new(1_000.0, 0.0, 0.0)),
            &context(),
            gate(),
        )
        .unwrap();
    assert!(matches!(
        result.position,
        Some(UpdateDecision::RejectedInnovation { .. })
    ));
    assert_eq!(filter, before);
}

#[test]
fn unhealthy_receiver_is_rejected_without_numerical_use() {
    let mut filter = filter();
    let before = filter;
    let mut observation = position_observation(SessionTime::ZERO, Vector3::zeros());
    observation.receiver_healthy = false;
    let result = filter
        .update_gnss(&observation, &context(), gate())
        .unwrap();
    assert_eq!(result.position, Some(UpdateDecision::RejectedHealth));
    assert_eq!(filter, before);
}

#[test]
fn lever_arm_position_jacobian_matches_central_difference() {
    let filter = filter();
    let lever = Vector3::new(0.4, -0.2, 0.1);
    let rotation = filter
        .state
        .orientation_n_from_b
        .to_rotation_matrix()
        .into_inner();
    let analytic = -rotation * skew(&lever);
    let epsilon = 1.0e-3;
    for axis in 0..3 {
        let mut delta = Vector3::zeros();
        delta[axis] = epsilon;
        let plus = (filter.state.orientation_n_from_b * so3_exp(delta)).transform_vector(&lever);
        let minus = (filter.state.orientation_n_from_b * so3_exp(-delta)).transform_vector(&lever);
        let numerical = (plus - minus) / (2.0 * epsilon);
        assert!((analytic.column(axis) - numerical).norm() < 2.0e-4);
    }
}

#[test]
fn consider_mean_never_changes_but_cross_covariance_updates() {
    let mut filter = filter();
    filter.active_consider = 1;
    filter.consider_covariance[(0, 0)] = 1.0;
    let mut observation = position_observation(SessionTime::ZERO, Vector3::new(0.1, 0.0, 0.0));
    observation.shared_jacobians.position[(0, 0)] = 1.0;
    filter
        .update_gnss(&observation, &context(), gate())
        .unwrap();
    assert_eq!(filter.consider_covariance[(0, 0)], 1.0);
    assert!(filter.nav_consider_covariance[(POS, 0)].abs() > 0.0);
}

#[test]
fn position_clock_lever_and_independent_timing_terms_are_analytic() {
    let mut filter = filter();
    filter.state.time = SessionTime::from_ns(10_000_000_000);
    filter.state.velocity_n = Vector3::new(3.0, -2.0, 1.0);
    filter.state.orientation_n_from_b = so3_exp(Vector3::new(0.2, -0.1, 0.3));
    let rotation = filter
        .state
        .orientation_n_from_b
        .to_rotation_matrix()
        .into_inner();
    let lever = Vector3::new(0.5, -0.2, 0.1);
    let omega = Vector3::new(0.0, 0.0, 2.0);
    let antenna_velocity = filter.state.velocity_n + rotation * omega.cross(&lever);
    let mut observation = position_observation(
        filter.state.time,
        filter.state.position_n + rotation * lever,
    );
    observation.imu_to_antenna_b = lever;
    observation.omega_ib_b = omega;
    observation.clock_consider_start = Some(0);
    observation.clock_reference_time = SessionTime::from_ns(6_000_000_000);
    observation.lever_arm_consider_start = Some(2);
    observation.position_independent_timing_sigma_s = 0.2;

    let linearization = filter
        .gnss_linearization(
            &observation,
            &MechanizationContext::new(Vector3::zeros(), Vector3::zeros(), Matrix3::zeros())
                .unwrap(),
            observation.position_n,
            None,
            None,
        )
        .unwrap();

    assert!(linearization.residual.fixed_rows::<3>(0).norm() < 1.0e-6);
    assert!((linearization.h_consider.fixed_view::<3, 1>(0, 0) - antenna_velocity).norm() < 1.0e-6);
    assert!(
        (linearization.h_consider.fixed_view::<3, 1>(0, 1) - antenna_velocity * 4.0).norm()
            < 2.0e-6
    );
    assert!((linearization.h_consider.fixed_view::<3, 3>(0, 2) - rotation).norm() < 1.0e-6);
    let expected_noise =
        observation.position_covariance_n + antenna_velocity * antenna_velocity.transpose() * 0.04;
    assert!((linearization.noise.fixed_view::<3, 3>(0, 0) - expected_noise).norm() < 1.0e-6);

    // The lever block is the derivative with respect to the fixed body
    // lever coordinates, independently of the attitude-state derivative.
    let epsilon = 1.0e-3;
    for axis in 0..3 {
        let mut delta = Vector3::zeros();
        delta[axis] = epsilon;
        let plus = filter.state.position_n + rotation * (lever + delta);
        let minus = filter.state.position_n + rotation * (lever - delta);
        let numerical = (plus - minus) / (2.0 * epsilon);
        assert!(
            (linearization.h_consider.fixed_view::<3, 1>(0, 2 + axis) - numerical).norm() < 3.0e-5
        );
    }
}

#[test]
fn velocity_uses_measurement_epoch_force_rate_and_timing_uncertainty() {
    let mut filter = filter();
    filter.state.time = SessionTime::from_ns(3_000_000_000);
    filter.state.velocity_n = Vector3::new(1.0, 2.0, 3.0);
    let lever = Vector3::new(0.5, 0.0, 0.0);
    let omega = Vector3::new(0.0, 0.0, 2.0);
    let angular_acceleration = Vector3::new(0.0, 2.0, 0.0);
    let angular_acceleration_covariance = Matrix3::from_diagonal(&Vector3::new(0.01, 0.04, 0.09));
    let specific_force = Vector3::new(4.0, 5.0, 16.0);
    let test_context = MechanizationContext::new(
        Vector3::zeros(),
        Vector3::new(0.0, 0.0, -10.0),
        Matrix3::zeros(),
    )
    .unwrap();
    let expected_acceleration = Vector3::new(2.0, 5.0, 5.0);
    let expected_velocity = filter.state.velocity_n + omega.cross(&lever);
    let mut observation = position_observation(filter.state.time, Vector3::zeros());
    observation.position_n = None;
    observation.velocity_n = Some(expected_velocity);
    observation.velocity_covariance_n = Matrix3::identity() * 0.25;
    observation.imu_to_antenna_b = lever;
    observation.omega_ib_b = omega;
    observation.specific_force_b = specific_force;
    observation.angular_acceleration_eb_b = Some(angular_acceleration);
    observation.angular_acceleration_covariance_b = angular_acceleration_covariance;
    observation.clock_consider_start = Some(0);
    observation.clock_reference_time = SessionTime::from_ns(1_000_000_000);
    observation.lever_arm_consider_start = Some(2);
    observation.velocity_independent_timing_sigma_s = 0.1;

    let linearization = filter
        .gnss_linearization(
            &observation,
            &test_context,
            None,
            observation.velocity_n,
            None,
        )
        .unwrap();

    assert!(linearization.residual.fixed_rows::<3>(0).norm() < 1.0e-6);
    assert!(
        (linearization.h_consider.fixed_view::<3, 1>(0, 0) - expected_acceleration).norm() < 1.0e-6
    );
    assert!(
        (linearization.h_consider.fixed_view::<3, 1>(0, 1) - expected_acceleration * 2.0).norm()
            < 1.0e-6
    );
    assert!((linearization.h_consider.fixed_view::<3, 3>(0, 2) - skew(&omega)).norm() < 1.0e-6);
    let tangential_jacobian = -skew(&lever);
    let tangential_covariance =
        tangential_jacobian * angular_acceleration_covariance * tangential_jacobian.transpose();
    let expected_noise = observation.velocity_covariance_n
        + (expected_acceleration * expected_acceleration.transpose() + tangential_covariance)
            * 0.01;
    assert!((linearization.noise.fixed_view::<3, 3>(0, 0) - expected_noise).norm() < 1.0e-6);
    let epsilon = 1.0e-2;
    for axis in 0..3 {
        let mut delta = Vector3::zeros();
        delta[axis] = epsilon;
        let plus = filter.state.velocity_n + omega.cross(&(lever + delta));
        let minus = filter.state.velocity_n + omega.cross(&(lever - delta));
        let numerical = (plus - minus) / (2.0 * epsilon);
        assert!(
            (linearization.h_consider.fixed_view::<3, 1>(0, 2 + axis) - numerical).norm() < 3.0e-5
        );
    }
}

#[test]
fn gnss_consider_block_bounds_are_transactional() {
    let mut filter = filter();
    filter.active_consider = 2;
    let before = filter;
    let mut observation = position_observation(SessionTime::ZERO, Vector3::zeros());
    observation.clock_consider_start = Some(1);

    assert_eq!(
        filter.update_gnss(&observation, &context(), gate()),
        Err(EskfError::InvalidConsiderBlock)
    );
    assert_eq!(filter, before);
}
