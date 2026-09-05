//! Propagation regression coverage.

use super::super::ProcessNoise;
#[cfg(test)]
use super::super::{
    discretization::{
        bias_random_walk_discrete_covariance, continuous_state_matrix, preintegration_nav_mapping,
    },
    propagation::map_preintegration_covariance,
};
use super::support::{context, filter, propagate_test, stationary_batch, zero_context};
use crate::{
    live::state::{
        ACC_BIAS, ATT, GYRO_BIAS, MechanizationContext, NAV_DIM, NavMatrix, NavState, POS, VEL,
    },
    time::SessionTime,
};
use nalgebra::{Matrix3, SMatrix, Vector3};

type NavMatrix64 = SMatrix<f64, NAV_DIM, NAV_DIM>;

fn bias_random_walk_oracle(
    continuous: &NavMatrix,
    dt: f32,
    process_noise: ProcessNoise,
) -> NavMatrix64 {
    // Independent f64 RK4 integration of dQ/dt = FQ + QF' + Qc. A
    // thousand uniform steps puts its O(h^4) global error well below the
    // f32 production discretizer for the 50 ms validated interval.
    let continuous = continuous.cast::<f64>();
    let mut density = NavMatrix64::zeros();
    density
        .fixed_view_mut::<3, 3>(ACC_BIAS, ACC_BIAS)
        .copy_from(
            &process_noise
                .accel_bias_random_walk_covariance_density
                .cast::<f64>(),
        );
    density
        .fixed_view_mut::<3, 3>(GYRO_BIAS, GYRO_BIAS)
        .copy_from(
            &process_noise
                .gyro_bias_random_walk_covariance_density
                .cast::<f64>(),
        );
    let derivative =
        |value: &NavMatrix64| continuous * value + value * continuous.transpose() + density;
    const STEPS: usize = 1_024;
    let step = f64::from(dt) / STEPS as f64;
    let mut covariance = NavMatrix64::zeros();
    for _ in 0..STEPS {
        let k1 = derivative(&covariance);
        let k2 = derivative(&(covariance + k1 * (0.5 * step)));
        let k3 = derivative(&(covariance + k2 * (0.5 * step)));
        let k4 = derivative(&(covariance + k3 * step));
        covariance += (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (step / 6.0);
    }
    (covariance + covariance.transpose()) * 0.5
}

fn assert_matches_bias_oracle(continuous: &NavMatrix, dt: f32, process_noise: ProcessNoise) {
    let production = bias_random_walk_discrete_covariance(continuous, dt, process_noise).unwrap();
    let oracle = bias_random_walk_oracle(continuous, dt, process_noise);
    let error = (production.cast::<f64>() - oracle).norm();
    let reference = oracle.norm().max(1.0e-30);
    assert!(
        error / reference < 2.0e-5,
        "relative Qd error {} (absolute {error}, reference {reference})",
        error / reference
    );
}

#[test]
fn stationary_specific_force_cancels_gravity() {
    let mut filter = filter();
    propagate_test(&mut filter, &stationary_batch(), &context()).unwrap();
    assert!(filter.state.velocity_n.norm() < 2.0e-5);
    assert!(filter.state.position_n.norm() < 2.0e-7);
}

#[test]
fn off_axis_bias_random_walk_density_reaches_mapped_process_covariance() {
    let batch = stationary_batch();
    let dt = batch.duration_seconds().unwrap();
    let accel_bias = Matrix3::new(
        4.0e-8, 1.0e-8, 0.0, //
        1.0e-8, 3.0e-8, 0.5e-8, //
        0.0, 0.5e-8, 2.0e-8,
    );
    let gyro_bias = Matrix3::new(
        4.0e-10, -1.0e-10, 0.0, //
        -1.0e-10, 3.0e-10, 0.5e-10, //
        0.0, 0.5e-10, 2.0e-10,
    );
    let covariance = map_preintegration_covariance(
        &batch,
        &preintegration_nav_mapping(&Matrix3::identity()),
        &continuous_state_matrix(&filter().state, &batch, &context()),
        dt,
        ProcessNoise {
            accel_bias_random_walk_covariance_density: accel_bias,
            gyro_bias_random_walk_covariance_density: gyro_bias,
        },
    )
    .unwrap();

    assert!((covariance[(ACC_BIAS, ACC_BIAS + 1)] - accel_bias[(0, 1)] * dt).abs() < 1.0e-14);
    assert!((covariance[(GYRO_BIAS, GYRO_BIAS + 1)] - gyro_bias[(0, 1)] * dt).abs() < 1.0e-16);
    assert_ne!(covariance[(ACC_BIAS, ACC_BIAS + 1)], 0.0);
    assert_ne!(covariance[(GYRO_BIAS, GYRO_BIAS + 1)], 0.0);
    assert_ne!(covariance[(VEL, ACC_BIAS + 1)], 0.0);
    assert_ne!(covariance[(ATT, GYRO_BIAS + 1)], 0.0);
}

#[test]
fn bias_random_walk_qd_matches_static_analytic_and_f64_oracle() {
    let mut batch = stationary_batch();
    batch.mean_omega_ib_b = Vector3::zeros();
    batch.mean_specific_force_b = Vector3::zeros();
    let state = NavState::stationary(SessionTime::ZERO);
    let continuous = continuous_state_matrix(&state, &batch, &zero_context());
    let accel_density = Matrix3::new(
        2.0, 0.3, 0.0, //
        0.3, 1.5, 0.2, //
        0.0, 0.2, 1.0,
    );
    let gyro_density = Matrix3::new(
        0.8, -0.1, 0.0, //
        -0.1, 0.6, 0.05, //
        0.0, 0.05, 0.4,
    );
    let process_noise = ProcessNoise {
        accel_bias_random_walk_covariance_density: accel_density,
        gyro_bias_random_walk_covariance_density: gyro_density,
    };
    let dt = 0.010_f32;
    let covariance = bias_random_walk_discrete_covariance(&continuous, dt, process_noise).unwrap();

    let qa = accel_density[(0, 0)];
    let qg = gyro_density[(0, 0)];
    let analytic = [
        ((ACC_BIAS, ACC_BIAS), qa * dt),
        ((VEL, ACC_BIAS), -qa * dt.powi(2) / 2.0),
        ((POS, ACC_BIAS), -qa * dt.powi(3) / 6.0),
        ((VEL, VEL), qa * dt.powi(3) / 3.0),
        ((POS, VEL), qa * dt.powi(4) / 8.0),
        ((POS, POS), qa * dt.powi(5) / 20.0),
        ((GYRO_BIAS, GYRO_BIAS), qg * dt),
        ((ATT, GYRO_BIAS), -qg * dt.powi(2) / 2.0),
        ((ATT, ATT), qg * dt.powi(3) / 3.0),
    ];
    for ((row, column), expected) in analytic {
        let tolerance = expected.abs() * 3.0e-5 + 1.0e-14;
        assert!(
            (covariance[(row, column)] - expected).abs() <= tolerance,
            "Qd({row},{column}) was {}, expected {expected}",
            covariance[(row, column)]
        );
    }
    assert_matches_bias_oracle(&continuous, dt, process_noise);
}

#[test]
fn bias_random_walk_qd_matches_f64_oracle_while_rotating() {
    let mut batch = stationary_batch();
    batch.mean_omega_ib_b = Vector3::new(8.0, -3.0, 5.0);
    batch.mean_specific_force_b = Vector3::new(15.0, -7.0, 23.0);
    let mut state = NavState::stationary(SessionTime::ZERO);
    state.orientation_n_from_b =
        nalgebra::UnitQuaternion::from_scaled_axis(Vector3::new(0.4, -0.2, 0.3));
    let rotating_context = MechanizationContext::new(
        Vector3::new(0.0, 5.0e-5, 4.0e-5),
        Vector3::new(0.0, 0.0, -9.8),
        Matrix3::new(
            1.0e-6, 2.0e-7, 0.0, //
            2.0e-7, -0.5e-6, 1.0e-7, //
            0.0, 1.0e-7, -0.5e-6,
        ),
    )
    .unwrap();
    let continuous = continuous_state_matrix(&state, &batch, &rotating_context);
    let process_noise = ProcessNoise {
        accel_bias_random_walk_covariance_density: Matrix3::new(
            2.0, 0.4, -0.1, //
            0.4, 1.5, 0.2, //
            -0.1, 0.2, 1.0,
        ),
        gyro_bias_random_walk_covariance_density: Matrix3::new(
            0.8, -0.15, 0.05, //
            -0.15, 0.7, 0.1, //
            0.05, 0.1, 0.6,
        ),
    };
    let covariance =
        bias_random_walk_discrete_covariance(&continuous, 0.05, process_noise).unwrap();
    assert_ne!(covariance[(VEL, GYRO_BIAS)], 0.0);
    assert_ne!(covariance[(POS, GYRO_BIAS)], 0.0);
    assert_ne!(covariance[(ATT, GYRO_BIAS + 1)], 0.0);
    assert_matches_bias_oracle(&continuous, 0.05, process_noise);
}
