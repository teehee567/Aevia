//! Sample regression coverage.

use super::super::{
    Eskf, EskfError, EskfPropagationScratch, GapNavCrossCovariance, ProcessNoise,
    gnss::{GnssObservation, UpdateDecision},
    update::LinearMeasurement,
};
use super::support::{filter, gate, position_observation, zero_context};
use crate::{
    live::{
        preintegration::{
            BIAS_DIM, ImuInterval, ImuNoise, ImuSampleCovariance, PreintegratedBatch, Preintegrator,
        },
        state::{ACC_BIAS, ATT, GYRO_BIAS, NAV_DIM, POS, VEL, skew, so3_exp},
    },
    time::SessionTime,
};
use nalgebra::{Matrix3, SMatrix, Vector3};

const AUGMENTED_DIM: usize = NAV_DIM + 2 + BIAS_DIM;

const SAMPLE_START: usize = NAV_DIM + 2;

type AugmentedCovariance = SMatrix<f64, AUGMENTED_DIM, AUGMENTED_DIM>;

type AugmentedMeasurementJacobian = SMatrix<f64, 6, AUGMENTED_DIM>;

fn augmented_covariance(
    filter: &Eskf,
    sample_covariance: &ImuSampleCovariance,
    sample_cross: &GapNavCrossCovariance,
) -> AugmentedCovariance {
    let mut joint = AugmentedCovariance::zeros();
    joint
        .fixed_view_mut::<NAV_DIM, NAV_DIM>(0, 0)
        .copy_from(&filter.covariance.cast::<f64>());
    let clock_cross = filter
        .nav_consider_covariance
        .fixed_columns::<2>(0)
        .into_owned()
        .cast::<f64>();
    joint
        .fixed_view_mut::<NAV_DIM, 2>(0, NAV_DIM)
        .copy_from(&clock_cross);
    joint
        .fixed_view_mut::<2, NAV_DIM>(NAV_DIM, 0)
        .copy_from(&clock_cross.transpose());
    joint.fixed_view_mut::<2, 2>(NAV_DIM, NAV_DIM).copy_from(
        &filter
            .consider_covariance
            .fixed_view::<2, 2>(0, 0)
            .into_owned()
            .cast::<f64>(),
    );
    joint
        .fixed_view_mut::<NAV_DIM, BIAS_DIM>(0, SAMPLE_START)
        .copy_from(&sample_cross.cast::<f64>());
    joint
        .fixed_view_mut::<BIAS_DIM, NAV_DIM>(SAMPLE_START, 0)
        .copy_from(&sample_cross.transpose().cast::<f64>());
    joint
        .fixed_view_mut::<BIAS_DIM, BIAS_DIM>(SAMPLE_START, SAMPLE_START)
        .copy_from(&sample_covariance.cast::<f64>());
    joint
}

fn augmented_jacobian(measurement: &LinearMeasurement) -> AugmentedMeasurementJacobian {
    let mut jacobian = AugmentedMeasurementJacobian::zeros();
    jacobian
        .fixed_view_mut::<6, NAV_DIM>(0, 0)
        .copy_from(&measurement.h_nav.cast::<f64>());
    jacobian.fixed_view_mut::<6, 2>(0, NAV_DIM).copy_from(
        &measurement
            .h_consider
            .fixed_columns::<2>(0)
            .into_owned()
            .cast::<f64>(),
    );
    jacobian
        .fixed_view_mut::<6, BIAS_DIM>(0, SAMPLE_START)
        .copy_from(&measurement.h_sample.cast::<f64>());
    jacobian
}

fn sample_filter(lever: Vector3<f32>) -> (Eskf, ImuSampleCovariance, GapNavCrossCovariance) {
    let mut filter = filter();
    filter.process_noise = ProcessNoise {
        accel_bias_random_walk_covariance_density: Matrix3::zeros(),
        gyro_bias_random_walk_covariance_density: Matrix3::zeros(),
    };
    filter.state.velocity_n = Vector3::new(1.3, -2.0, 0.4);
    filter.active_consider = 2;
    filter.consider_covariance[(0, 0)] = 0.04;
    filter.consider_covariance[(0, 1)] = 0.003;
    filter.consider_covariance[(1, 0)] = 0.003;
    filter.consider_covariance[(1, 1)] = 0.001;
    let mut clock_loading = SMatrix::<f32, NAV_DIM, 2>::zeros();
    clock_loading[(POS, 0)] = 0.1;
    clock_loading[(VEL + 1, 1)] = -0.2;
    let clock_covariance = filter
        .consider_covariance
        .fixed_view::<2, 2>(0, 0)
        .into_owned();
    filter
        .nav_consider_covariance
        .fixed_columns_mut::<2>(0)
        .copy_from(&(clock_loading * clock_covariance));
    filter.covariance += clock_loading * clock_covariance * clock_loading.transpose();
    let mut sample_covariance = ImuSampleCovariance::identity() * 0.05;
    sample_covariance[(0, 1)] = 0.003;
    sample_covariance[(1, 0)] = 0.003;
    sample_covariance
        .fixed_view_mut::<3, 3>(3, 3)
        .scale_mut(0.4);
    let mut sample_loading = GapNavCrossCovariance::zeros();
    // The antenna-to-IMU initialization uses true-minus-nominal state
    // error and observed-minus-true sample error, giving this minus sign.
    sample_loading
        .fixed_view_mut::<3, 3>(VEL, 3)
        .copy_from(&(-skew(&lever)));
    sample_loading[(POS, 0)] = 0.03;
    sample_loading[(ATT + 1, 4)] = -0.02;
    let cross = sample_loading * sample_covariance;
    filter.covariance += cross * sample_loading.transpose();
    (filter, sample_covariance, cross)
}

fn joint_sample_observation(filter: &Eskf, lever: Vector3<f32>) -> GnssObservation {
    let mut observation = position_observation(filter.state.time, filter.state.position_n + lever);
    observation.velocity_n = Some(filter.state.velocity_n);
    observation.position_velocity_cross_n = Some(Matrix3::zeros());
    observation.imu_to_antenna_b = lever;
    observation.velocity_covariance_n = Matrix3::identity() * 0.03;
    observation.clock_consider_start = Some(0);
    observation.angular_acceleration_eb_b = Some(Vector3::zeros());
    observation
}

#[test]
fn held_sample_gnss_matches_augmented_schmidt_oracle_with_clock_and_reset() {
    let lever = Vector3::new(0.4, -0.2, 0.1);
    let (mut filter, sample_covariance, mut sample_cross) = sample_filter(lever);
    filter.state.time = SessionTime::from_ns(2_000_000_000);
    filter.state.orientation_n_from_b = so3_exp(Vector3::new(0.2, -0.3, 0.1));
    let rotation = filter
        .state
        .orientation_n_from_b
        .to_rotation_matrix()
        .into_inner();
    let mut observation = joint_sample_observation(&filter, lever);
    observation.omega_ib_b = Vector3::new(0.3, -0.2, 0.5);
    observation.specific_force_b = Vector3::new(3.0, 1.0, 0.5);
    observation.angular_acceleration_eb_b = Some(Vector3::new(0.3, -0.1, 0.2));
    observation.angular_acceleration_covariance_b = Matrix3::identity() * 0.01;
    observation.velocity_independent_timing_sigma_s = 0.002;
    observation.position_n =
        Some(filter.state.position_n + rotation * lever + Vector3::new(0.03, -0.01, 0.015));
    observation.velocity_n = Some(
        filter.state.velocity_n
            + rotation * observation.omega_ib_b.cross(&lever)
            + Vector3::new(0.06, -0.02, 0.04),
    );
    let measurement = filter
        .gnss_linearization(
            &observation,
            &zero_context(),
            observation.position_n,
            observation.velocity_n,
            observation.position_velocity_cross_n,
        )
        .unwrap();
    let prior = augmented_covariance(&filter, &sample_covariance, &sample_cross);
    let jacobian = augmented_jacobian(&measurement);
    let noise = measurement.noise.cast::<f64>();
    let innovation = jacobian * prior * jacobian.transpose() + noise;
    let full_cross = prior * jacobian.transpose();
    let (u_nav, u_consider, u_sample, actual_innovation) =
        filter.innovation_terms(&measurement, Some(&sample_covariance), Some(&sample_cross));
    assert!((actual_innovation.cast::<f64>() - innovation).norm() < 2.0e-6);
    assert!((u_nav.cast::<f64>() - full_cross.fixed_rows::<NAV_DIM>(0)).norm() < 2.0e-6);
    assert!(
        (u_consider.fixed_rows::<2>(0).into_owned().cast::<f64>()
            - full_cross.fixed_rows::<2>(NAV_DIM))
        .norm()
            < 2.0e-7
    );
    assert!(
        (u_sample.cast::<f64>() - full_cross.fixed_rows::<BIAS_DIM>(SAMPLE_START)).norm() < 2.0e-7
    );
    let mut gain = SMatrix::<f64, AUGMENTED_DIM, 6>::zeros();
    gain.fixed_rows_mut::<NAV_DIM>(0)
        .copy_from(&(full_cross.fixed_rows::<NAV_DIM>(0) * innovation.try_inverse().unwrap()));
    let correction = gain.fixed_rows::<NAV_DIM>(0) * measurement.residual.cast::<f64>();
    let mut expected_state = filter.state;
    let attitude_reset = expected_state.inject(&correction.cast::<f32>()).unwrap();
    let mut reset = AugmentedCovariance::identity();
    reset
        .fixed_view_mut::<3, 3>(ATT, ATT)
        .copy_from(&attitude_reset.cast::<f64>());
    let residual_map = AugmentedCovariance::identity() - gain * jacobian;
    let expected = reset
        * (residual_map * prior * residual_map.transpose() + gain * noise * gain.transpose())
        * reset.transpose();
    let result = filter
        .update_gnss_with_imu_sample(
            &observation,
            &zero_context(),
            gate(),
            Some(&sample_covariance),
            Some(&mut sample_cross),
        )
        .unwrap();
    assert!(matches!(result.joint, Some(UpdateDecision::Fused { .. })));
    assert!(
        (augmented_covariance(&filter, &sample_covariance, &sample_cross) - expected).norm()
            < 3.0e-6
    );
    assert!((filter.state.position_n - expected_state.position_n).norm() < 2.0e-7);
    assert!((filter.state.velocity_n - expected_state.velocity_n).norm() < 2.0e-7);
    assert!(
        (filter.state.orientation_n_from_b.inverse() * expected_state.orientation_n_from_b).angle()
            < 2.0e-7
    );

    // Differentiate the observation model independently of its Jacobian.
    for axis in 0..3 {
        let mut error = Vector3::zeros();
        error[axis] = 0.001;
        let plus = rotation * (observation.omega_ib_b - error).cross(&lever);
        let minus = rotation * (observation.omega_ib_b + error).cross(&lever);
        let numerical = (plus - minus) / 0.002;
        assert!((measurement.h_sample.fixed_view::<3, 1>(3, 3 + axis) - numerical).norm() < 3.0e-5);
    }
}

fn sample_batch(
    start_ns: i64,
    end_ns: i64,
    covariance: &ImuSampleCovariance,
) -> PreintegratedBatch {
    use crate::live::preintegration::CompactCovariance3;
    let mut preintegrator = Preintegrator::new(
        SessionTime::from_ns(start_ns),
        Vector3::zeros(),
        Vector3::zeros(),
        1.0,
    )
    .unwrap();
    preintegrator
        .push_piece(
            ImuInterval {
                start: SessionTime::from_ns(start_ns),
                end: SessionTime::from_ns(end_ns),
                omega_ib_b: Vector3::zeros(),
                specific_force_b: Vector3::zeros(),
                degraded_input: false,
                gap_elapsed_ns_plus_one: 0,
                body_from_sensor: nalgebra::UnitQuaternion::identity(),
                accel_sample_covariance: CompactCovariance3::from_matrix(
                    covariance.fixed_view::<3, 3>(0, 0).into_owned(),
                )
                .unwrap(),
                gyro_sample_covariance: CompactCovariance3::from_matrix(
                    covariance.fixed_view::<3, 3>(3, 3).into_owned(),
                )
                .unwrap(),
                calibration_consider_start: None,
            },
            ImuNoise {
                accel_covariance_density: Matrix3::zeros(),
                gyro_covariance_density: Matrix3::zeros(),
            },
            true,
            true,
        )
        .unwrap();
    preintegrator.batch().unwrap()
}

fn propagate_sample(
    filter: &mut Eskf,
    cross: &mut GapNavCrossCovariance,
    sample_covariance: &ImuSampleCovariance,
    end_ns: i64,
) {
    let batch = sample_batch(filter.state.time.as_ns(), end_ns, sample_covariance);
    let mut scratch = std::boxed::Box::new(EskfPropagationScratch::new());
    filter
        .propagate_with_imu_sample(&batch, &zero_context(), Some(cross), &mut scratch)
        .unwrap();
    scratch.commit_sample_candidate_into(cross);
}

fn stationary_augmented_transition(dt: f64) -> AugmentedCovariance {
    let mut transition = AugmentedCovariance::identity();
    for axis in 0..3 {
        transition[(POS + axis, VEL + axis)] = dt;
        transition[(POS + axis, ACC_BIAS + axis)] = -0.5 * dt * dt;
        transition[(VEL + axis, ACC_BIAS + axis)] = -dt;
        transition[(ATT + axis, GYRO_BIAS + axis)] = -dt;
        transition[(POS + axis, SAMPLE_START + axis)] = -0.5 * dt * dt;
        transition[(VEL + axis, SAMPLE_START + axis)] = -dt;
        transition[(ATT + axis, SAMPLE_START + 3 + axis)] = -dt;
    }
    transition
}

#[test]
fn held_sample_gnss_and_propagation_are_invariant_to_scheduler_cuts() {
    let lever = Vector3::new(0.4, -0.2, 0.1);
    let (initial, sample_covariance, initial_cross) = sample_filter(lever);
    let mut unsplit = initial;
    let mut split = initial;
    let mut unsplit_cross = initial_cross;
    let mut split_cross = initial_cross;
    let mut oracle = augmented_covariance(&initial, &sample_covariance, &initial_cross);
    let before_measurement = stationary_augmented_transition(0.004);
    oracle = before_measurement * oracle * before_measurement.transpose();
    propagate_sample(
        &mut unsplit,
        &mut unsplit_cross,
        &sample_covariance,
        4_000_000,
    );
    propagate_sample(&mut split, &mut split_cross, &sample_covariance, 1_000_000);
    propagate_sample(&mut split, &mut split_cross, &sample_covariance, 3_000_000);
    propagate_sample(&mut split, &mut split_cross, &sample_covariance, 4_000_000);
    assert!(
        (augmented_covariance(&unsplit, &sample_covariance, &unsplit_cross) - oracle).norm()
            < 2.0e-6
    );
    assert!(
        (augmented_covariance(&split, &sample_covariance, &split_cross) - oracle).norm() < 2.0e-6
    );
    let observation = joint_sample_observation(&unsplit, lever);
    let measurement = unsplit
        .gnss_linearization(
            &observation,
            &zero_context(),
            observation.position_n,
            observation.velocity_n,
            observation.position_velocity_cross_n,
        )
        .unwrap();
    let jacobian = augmented_jacobian(&measurement);
    let noise = measurement.noise.cast::<f64>();
    let innovation = jacobian * oracle * jacobian.transpose() + noise;
    let mut gain = SMatrix::<f64, AUGMENTED_DIM, 6>::zeros();
    gain.fixed_rows_mut::<NAV_DIM>(0).copy_from(
        &(oracle.fixed_rows::<NAV_DIM>(0)
            * jacobian.transpose()
            * innovation.try_inverse().unwrap()),
    );
    let residual_map = AugmentedCovariance::identity() - gain * jacobian;
    oracle = residual_map * oracle * residual_map.transpose() + gain * noise * gain.transpose();
    for (filter, cross) in [
        (&mut unsplit, &mut unsplit_cross),
        (&mut split, &mut split_cross),
    ] {
        let result = filter
            .update_gnss_with_imu_sample(
                &observation,
                &zero_context(),
                gate(),
                Some(&sample_covariance),
                Some(cross),
            )
            .unwrap();
        assert!(matches!(result.joint, Some(UpdateDecision::Fused { .. })));
    }
    let after_measurement = stationary_augmented_transition(0.006);
    oracle = after_measurement * oracle * after_measurement.transpose();
    propagate_sample(
        &mut unsplit,
        &mut unsplit_cross,
        &sample_covariance,
        10_000_000,
    );
    propagate_sample(&mut split, &mut split_cross, &sample_covariance, 7_000_000);
    propagate_sample(&mut split, &mut split_cross, &sample_covariance, 10_000_000);
    assert!(
        (augmented_covariance(&unsplit, &sample_covariance, &unsplit_cross) - oracle).norm()
            < 3.0e-6
    );
    assert!(
        (augmented_covariance(&split, &sample_covariance, &split_cross) - oracle).norm() < 3.0e-6
    );
    assert!((split.state.position_n - unsplit.state.position_n).norm() < 2.0e-7);
    assert!((split.covariance - unsplit.covariance).norm() < 2.0e-6);
}

#[test]
fn held_sample_cross_and_filter_survive_rejection_and_late_covariance_failure() {
    let lever = Vector3::new(0.4, -0.2, 0.1);
    let (mut filter, sample_covariance, mut sample_cross) = sample_filter(lever);
    let before = filter;
    let prior_cross = sample_cross;
    let mut observation = joint_sample_observation(&filter, lever);
    observation.position_n = Some(Vector3::repeat(1_000.0));
    let result = filter
        .update_gnss_with_imu_sample(
            &observation,
            &zero_context(),
            gate(),
            Some(&sample_covariance),
            Some(&mut sample_cross),
        )
        .unwrap();
    assert!(matches!(
        result.joint,
        Some(UpdateDecision::RejectedInnovation { .. })
    ));
    assert_eq!(filter, before);
    assert_eq!(sample_cross, prior_cross);

    // Deliberately corrupt an unobserved prior block. The innovation and
    // injection succeed, but final covariance conditioning must fail.
    filter.covariance[(POS, POS + 1)] = 100.0;
    filter.covariance[(POS + 1, POS)] = 100.0;
    let before_failure = filter;
    observation.position_n = None;
    observation.position_velocity_cross_n = None;
    observation.velocity_n = Some(filter.state.velocity_n + Vector3::repeat(0.1));
    assert_eq!(
        filter.update_gnss_with_imu_sample(
            &observation,
            &zero_context(),
            gate(),
            Some(&sample_covariance),
            Some(&mut sample_cross)
        ),
        Err(EskfError::CovarianceNotPositiveSemidefinite)
    );
    assert_eq!(filter, before_failure);
    assert_eq!(sample_cross, prior_cross);
}
