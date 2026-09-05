//! Consider regression coverage.

#[cfg(test)]
use super::super::discretization::{continuous_state_matrix, state_transition};
use super::super::{
    ConsiderCovariance, EskfError, MAX_CONSIDER, NavConsiderCovariance,
    clock::{
        clock_innovation_is_psd, independent_clock_consider_covariance_into,
        transition_consider_covariance_into,
    },
    covariance::active_principal_block_is_psd,
};
use super::support::{context, filter, propagate_test, stationary_batch};
use crate::live::state::{ATT, MechanizationContext, POS, VEL};
use nalgebra::{Matrix3, Vector3};

#[test]
fn process_consider_sensitivity_propagates_the_complete_joint_covariance() {
    let mut with_consider = filter();
    with_consider.active_consider = 3;
    with_consider.consider_covariance[(0, 0)] = 4.0;
    with_consider.consider_covariance[(1, 1)] = 9.0;
    with_consider.consider_covariance[(2, 2)] = 16.0;
    with_consider.nav_consider_covariance[(POS, 0)] = 0.02;
    with_consider.nav_consider_covariance[(VEL + 1, 1)] = -0.03;
    with_consider.nav_consider_covariance[(ATT + 2, 2)] = 0.01;
    let mut without_consider = with_consider;

    let mut batch = stationary_batch();
    batch.calibration_consider_start = Some(0);
    batch.mean_specific_force_consider_jacobian = Matrix3::new(
        10.0, 2.0, 0.0, //
        -1.0, 4.0, 0.0, //
        0.0, 0.0, -3.0,
    );
    batch.mean_angular_rate_consider_jacobian = Matrix3::new(
        0.5, 0.0, 0.0, //
        0.0, -0.25, 0.0, //
        0.0, 0.0, 0.75,
    );
    let mut baseline_batch = batch;
    baseline_batch.calibration_consider_start = None;
    baseline_batch.mean_specific_force_consider_jacobian = Matrix3::zeros();
    baseline_batch.mean_angular_rate_consider_jacobian = Matrix3::zeros();
    let zero_context =
        MechanizationContext::new(Vector3::zeros(), Vector3::zeros(), Matrix3::zeros()).unwrap();
    let unchanged_consider_covariance = with_consider.consider_covariance;
    let initial_cross = with_consider.nav_consider_covariance;
    let dt = batch.duration_seconds().unwrap();
    let transition = state_transition(
        &continuous_state_matrix(&with_consider.state, &batch, &zero_context),
        dt,
    )
    .unwrap();

    propagate_test(&mut with_consider, &batch, &zero_context).unwrap();
    propagate_test(&mut without_consider, &baseline_batch, &zero_context).unwrap();

    let mut expected_gamma = NavConsiderCovariance::zeros();
    expected_gamma
        .fixed_view_mut::<3, 3>(POS, 0)
        .copy_from(&(batch.mean_specific_force_consider_jacobian * (0.5 * dt * dt)));
    expected_gamma
        .fixed_view_mut::<3, 3>(VEL, 0)
        .copy_from(&(batch.mean_specific_force_consider_jacobian * dt));
    expected_gamma
        .fixed_view_mut::<3, 3>(ATT, 0)
        .copy_from(&(batch.mean_angular_rate_consider_jacobian * dt));
    let propagated_cross = transition * initial_cross;
    let gamma_consider_covariance = expected_gamma * unchanged_consider_covariance;
    let expected_cross = propagated_cross + gamma_consider_covariance;
    let expected_added_covariance = propagated_cross * expected_gamma.transpose()
        + expected_gamma * propagated_cross.transpose()
        + gamma_consider_covariance * expected_gamma.transpose();

    assert_eq!(with_consider.state, without_consider.state);
    assert_eq!(
        with_consider.consider_covariance,
        unchanged_consider_covariance
    );
    assert!((with_consider.nav_consider_covariance - expected_cross).norm() < 1.0e-6);
    assert!(
        (with_consider.covariance - without_consider.covariance - expected_added_covariance).norm()
            < 2.0e-5
    );
}

#[test]
fn process_consider_block_bounds_are_checked_before_state_mutation() {
    let mut filter = filter();
    filter.active_consider = 3;
    let before = filter;
    let mut batch = stationary_batch();
    batch.calibration_consider_start = Some(1);
    batch.mean_specific_force_consider_jacobian = Matrix3::identity();

    assert_eq!(
        propagate_test(&mut filter, &batch, &context()),
        Err(EskfError::InvalidConsiderBlock)
    );
    assert_eq!(filter, before);
}

#[test]
fn clock_bridge_transforms_pxc_and_pcc_without_changing_navigation_marginal() {
    let mut filter = filter();
    filter.active_consider = 4;
    filter.consider_covariance[(0, 0)] = 4.0;
    filter.consider_covariance[(1, 1)] = 9.0;
    filter.consider_covariance[(2, 2)] = 16.0;
    filter.consider_covariance[(3, 3)] = 25.0;
    filter.nav_consider_covariance[(POS, 0)] = 2.0;
    filter.nav_consider_covariance[(POS, 1)] = 3.0;
    filter.nav_consider_covariance[(POS, 2)] = 4.0;
    filter.nav_consider_covariance[(POS, 3)] = 5.0;

    let mut mapping = [[0.0; MAX_CONSIDER]; 2];
    mapping[0][0] = 1.0;
    mapping[0][1] = 0.25;
    mapping[0][2] = 0.5;
    mapping[1][1] = 1.0;
    mapping[1][3] = -0.25;
    let innovation = [1.0, 0.1, 2.0];
    let before = filter;

    let mut transform = ConsiderCovariance::identity();
    for column in 0..4 {
        transform[(0, column)] = mapping[0][column];
        transform[(1, column)] = mapping[1][column];
    }
    let mut noise = ConsiderCovariance::zeros();
    noise[(0, 0)] = innovation[0];
    noise[(0, 1)] = innovation[1];
    noise[(1, 0)] = innovation[1];
    noise[(1, 1)] = innovation[2];
    let expected_consider = transform * before.consider_covariance * transform.transpose() + noise;
    let expected_cross = before.nav_consider_covariance * transform.transpose();
    let mut staged_seed = ConsiderCovariance::repeat(-7.0);
    transition_consider_covariance_into(
        &before.consider_covariance,
        4,
        &mapping,
        innovation,
        &mut staged_seed,
    )
    .unwrap();

    filter
        .transition_clock_consider(4, &mapping, innovation)
        .unwrap();

    assert_eq!(filter.covariance, before.covariance);
    assert!((filter.consider_covariance - expected_consider).norm() < 1.0e-5);
    assert_eq!(filter.consider_covariance, staged_seed);
    assert!((filter.nav_consider_covariance - expected_cross).norm() < 1.0e-6);
}

#[test]
fn invalid_clock_bridge_is_transactional() {
    let mut filter = filter();
    filter.active_consider = 4;
    for coordinate in 0..4 {
        filter.consider_covariance[(coordinate, coordinate)] = 1.0;
    }
    let before = filter;
    let seed_before = filter.consider_covariance;
    let mut seed_candidate = ConsiderCovariance::repeat(-7.0);
    let candidate_before = seed_candidate;
    let mut mapping = [[0.0; MAX_CONSIDER]; 2];
    mapping[0][0] = 1.0;
    mapping[1][1] = 1.0;
    mapping[0][4] = 1.0;

    assert_eq!(
        filter.transition_clock_consider(4, &mapping, [0.0; 3]),
        Err(EskfError::InvalidConsiderBlock)
    );
    assert_eq!(filter, before);
    assert_eq!(
        transition_consider_covariance_into(
            &seed_before,
            4,
            &mapping,
            [0.0; 3],
            &mut seed_candidate,
        ),
        Err(EskfError::InvalidConsiderBlock)
    );
    assert_eq!(seed_before, filter.consider_covariance);
    assert_eq!(seed_candidate, candidate_before);
}

#[test]
fn independent_clock_prior_preserves_calibration_block_but_removes_declared_crosses() {
    let mut previous = ConsiderCovariance::zeros();
    previous[(0, 0)] = 4.0;
    previous[(1, 1)] = 9.0;
    previous[(2, 2)] = 16.0;
    previous[(3, 3)] = 25.0;
    previous[(0, 2)] = 0.5;
    previous[(2, 0)] = 0.5;
    previous[(1, 3)] = -0.75;
    previous[(3, 1)] = -0.75;

    let mut next = ConsiderCovariance::repeat(-7.0);
    independent_clock_consider_covariance_into(&previous, 4, [1.0, 0.2, 2.0], &mut next).unwrap();
    assert_eq!(next[(0, 0)], 1.0);
    assert_eq!(next[(0, 1)], 0.2);
    assert_eq!(next[(1, 1)], 2.0);
    assert_eq!(next[(0, 2)], 0.0);
    assert_eq!(next[(1, 3)], 0.0);
    assert_eq!(
        next.fixed_view::<2, 2>(2, 2),
        previous.fixed_view::<2, 2>(2, 2)
    );
}

#[test]
fn clock_psd_check_is_robust_for_extreme_finite_scales() {
    let largest = f32::MAX;
    let smallest = f32::from_bits(1);
    let boundary = crate::scalar_math::sqrt(largest) * crate::scalar_math::sqrt(smallest);

    assert!(clock_innovation_is_psd([largest, boundary, smallest]));
    assert!(!clock_innovation_is_psd([
        largest,
        boundary * 2.0,
        smallest,
    ]));
    assert!(clock_innovation_is_psd([smallest, smallest, smallest]));
    assert!(clock_innovation_is_psd([0.0, 0.0, largest]));
    assert!(!clock_innovation_is_psd([0.0, smallest, largest]));
}

#[test]
fn consider_psd_check_rejects_scale_relative_asymmetry() {
    let tiny = f32::MIN_POSITIVE;
    let mut covariance = ConsiderCovariance::zeros();
    covariance[(0, 0)] = tiny;
    covariance[(0, 1)] = tiny;
    covariance[(1, 0)] = tiny;
    covariance[(1, 1)] = tiny;
    assert!(active_principal_block_is_psd(&covariance, 2));

    covariance[(1, 0)] = 0.5 * tiny;
    assert!(!active_principal_block_is_psd(&covariance, 2));

    covariance[(1, 0)] = 2.0 * tiny;
    covariance[(0, 1)] = 2.0 * tiny;
    assert!(!active_principal_block_is_psd(&covariance, 2));
}
