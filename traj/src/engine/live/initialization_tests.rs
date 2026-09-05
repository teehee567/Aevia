//! Regression tests for initialization conversion tests.

use super::*;
use nalgebra::UnitQuaternion as NaUnitQuaternion;

#[test]
fn initial_antenna_to_imu_transform_updates_mean_and_complete_covariance() {
    use crate::live::state::{ATT, GYRO_BIAS, NAV_DIM, NavMatrix, NavState, POS, VEL};
    use nalgebra::SMatrix;

    let mut state = NavState {
        time: SessionTime::ZERO,
        position_n: NaVector3::new(10.0, 20.0, 30.0),
        velocity_n: NaVector3::new(4.0, 5.0, 6.0),
        orientation_n_from_b: NaUnitQuaternion::identity(),
        accel_bias_b: NaVector3::zeros(),
        gyro_bias_b: NaVector3::new(0.01, -0.02, 0.03),
    };
    let mut covariance = NavMatrix::zeros();
    for index in 0..NAV_DIM {
        covariance[(index, index)] = 0.1 * (index + 1) as f32;
    }
    covariance[(POS, ATT + 1)] = 0.03;
    covariance[(ATT + 1, POS)] = 0.03;
    let original_covariance = covariance;
    let omega_ib_b = NaVector3::new(0.2, -0.1, 0.5);
    let earth_rate_n = NaVector3::new(0.0, 4.0e-5, 5.0e-5);
    let lever_b = NaVector3::new(1.0, -0.25, 0.5);
    let gyro_sample_covariance = Matrix3::from_diagonal(&NaVector3::new(0.01, 0.02, 0.03));

    let omega_eb_b = transform_initial_antenna_state_to_imu(
        &mut state,
        &mut covariance,
        omega_ib_b,
        gyro_sample_covariance,
        earth_rate_n,
        lever_b,
    )
    .unwrap();

    let rotation_n_from_b = Matrix3::<f32>::identity();
    let expected_omega_eb_b = omega_ib_b - state.gyro_bias_b - earth_rate_n;
    assert!((omega_eb_b - expected_omega_eb_b).norm() < 1.0e-7);
    assert!((state.position_n - NaVector3::new(9.0, 20.25, 29.5)).norm() < 1.0e-6);
    assert!(
        (state.velocity_n - (NaVector3::new(4.0, 5.0, 6.0) - expected_omega_eb_b.cross(&lever_b)))
            .norm()
            < 1.0e-6
    );

    let angular_rate_minus_bias = omega_ib_b - state.gyro_bias_b;
    let earth_rate_b = earth_rate_n;
    let position_attitude = rotation_n_from_b * crate::live::state::skew(&lever_b);
    let velocity_attitude = rotation_n_from_b
        * (crate::live::state::skew(&angular_rate_minus_bias.cross(&lever_b))
            - crate::live::state::skew(&earth_rate_b) * crate::live::state::skew(&lever_b));
    let velocity_gyro_bias = -rotation_n_from_b * crate::live::state::skew(&lever_b);
    let mut jacobian = NavMatrix::identity();
    jacobian
        .fixed_view_mut::<3, 3>(POS, ATT)
        .copy_from(&position_attitude);
    jacobian
        .fixed_view_mut::<3, 3>(VEL, ATT)
        .copy_from(&velocity_attitude);
    jacobian
        .fixed_view_mut::<3, 3>(VEL, GYRO_BIAS)
        .copy_from(&velocity_gyro_bias);
    let mut expected = jacobian * original_covariance * jacobian.transpose();
    let gyro_to_velocity = rotation_n_from_b * crate::live::state::skew(&lever_b);
    let expected_velocity_covariance = expected.fixed_view::<3, 3>(VEL, VEL).into_owned()
        + gyro_to_velocity * gyro_sample_covariance * gyro_to_velocity.transpose();
    expected
        .fixed_view_mut::<3, 3>(VEL, VEL)
        .copy_from(&expected_velocity_covariance);
    expected = (expected + expected.transpose()) * 0.5;
    let error: SMatrix<f32, NAV_DIM, NAV_DIM> = covariance - expected;
    assert!(error.norm() < 1.0e-5, "covariance error: {}", error.norm());
    assert!(covariance.fixed_view::<3, 3>(POS, ATT).norm() > 0.0);
    assert!(covariance.fixed_view::<3, 3>(VEL, GYRO_BIAS).norm() > 0.0);
}

#[test]
fn position_extrapolation_retains_the_derived_position_velocity_cross_block() {
    let position_covariance = Matrix3::identity() * 10.0;
    let velocity_covariance = Matrix3::from_diagonal(&NaVector3::new(1.0, 2.0, 3.0));
    let initial_cross = Matrix3::new(
        0.5, 0.1, 0.0, //
        0.0, 0.4, 0.2, //
        0.1, 0.0, 0.3,
    );
    let evidence = InitializationFixEvidence {
        position_epoch: SessionTime::ZERO,
        velocity_epoch: SessionTime::ZERO,
        position_n: NaVector3::new(1.0, 2.0, 3.0),
        velocity_n: NaVector3::new(4.0, 5.0, 6.0),
        position_covariance_n: position_covariance,
        velocity_covariance_n: velocity_covariance,
        position_velocity_cross_n: Some(initial_cross),
        position_independent_timing_sigma_s: 0.0,
    };

    let fix = initialization_fix_at(evidence, SessionTime::from_ns(2_000_000_000), 3_000_000_000)
        .unwrap()
        .unwrap();

    assert_eq!(fix.position_n, NaVector3::new(9.0, 12.0, 15.0));
    assert_eq!(
        fix.position_velocity_cross_n,
        initial_cross + velocity_covariance * 2.0
    );
    assert_eq!(
        fix.position_covariance_n,
        position_covariance
            + velocity_covariance * 4.0
            + (initial_cross + initial_cross.transpose()) * 2.0
    );
}
