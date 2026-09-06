//! Forward RTS capture checked against scalar and augmented error models.

use super::super::{
    EskfError, EskfPropagationScratch, GapNavCrossCovariance, RtsUpdateCapture,
    gnss::UpdateDecision, update::LinearMeasurement,
};
use super::{
    gap::gap_batch,
    sample::sample_batch,
    support::{filter, gate, position_observation, stationary_batch, zero_context},
};
use crate::{
    live::{
        preintegration::{GapModel, ImuSampleCovariance},
        smoothing::{AUG_DIM, CONSIDER_START, GAP_START, SAMPLE_START},
        state::{ACC_BIAS, ATT, GYRO_BIAS, NAV_DIM, POS, VEL},
    },
    time::SessionTime,
};
use nalgebra::{Matrix3, Vector3};

#[test]
fn sequential_schmidt_updates_capture_shared_sample_and_gap_crosses() {
    let mut filter = filter();
    filter.active_consider = 1;
    filter.consider_covariance[(0, 0)] = 9.0;
    filter.nav_consider_covariance[(POS, 0)] = 1.0;
    filter.gap_origin = Some(SessionTime::ZERO);
    filter.gap_derivative_covariance[(0, 0)] = 2.0;
    filter.gap_nav_cross_covariance[(POS, 0)] = 0.15;
    let mut sample = ImuSampleCovariance::zeros();
    sample[(0, 0)] = 0.25;
    let mut sample_cross = GapNavCrossCovariance::zeros();
    sample_cross[(POS, 0)] = 0.2;
    let mut capture = std::boxed::Box::new(RtsUpdateCapture::new());
    let mut expected = [0.0_f32; AUG_DIM];
    expected[POS] = 1.0;
    for _ in 0..2 {
        let variance = filter.covariance[(POS, POS)];
        let consider_cross = filter.nav_consider_covariance[(POS, 0)];
        let held_cross = sample_cross[(POS, 0)];
        let u = variance + 0.5 * consider_cross + 2.0 * held_cross;
        let innovation = variance + consider_cross + 4.0 * held_cross + 2.25 + 1.0 + 1.0;
        let gain = u / innovation;
        for value in &mut expected {
            *value *= 1.0 - gain;
        }
        expected[CONSIDER_START] -= gain * 0.5;
        expected[SAMPLE_START] -= gain * 2.0;

        let mut measurement = LinearMeasurement::zeros();
        measurement.dimension = 1;
        measurement.h_nav[(0, POS)] = 1.0;
        measurement.h_consider[(0, 0)] = 0.5;
        measurement.h_sample[(0, 0)] = 2.0;
        measurement.noise[(0, 0)] = 1.0;
        measurement.residual[0] = 0.1;
        assert!(matches!(
            filter
                .linear_update(
                    measurement,
                    10.0,
                    100.0,
                    1.0,
                    Some(&sample),
                    Some(&mut sample_cross),
                    Some(&mut capture)
                )
                .unwrap(),
            UpdateDecision::Fused { .. }
        ));
    }
    for (column, expected) in expected.iter().enumerate() {
        assert!((capture.nav_transform[(POS, column)] - expected).abs() < 1.0e-6);
    }
    // P(predicted augmented error, filtered navigation error) = Ppred A'.
    let predicted_nav_cross =
        4.0 * expected[POS] + expected[CONSIDER_START] + 0.2 * expected[SAMPLE_START];
    let predicted_consider_cross = expected[POS] + 9.0 * expected[CONSIDER_START];
    let predicted_sample_cross = 0.2 * expected[POS] + 0.25 * expected[SAMPLE_START];
    let predicted_gap_cross = 0.15 * expected[POS];
    let captured_nav_cross = 4.0 * capture.nav_transform[(POS, POS)]
        + capture.nav_transform[(POS, CONSIDER_START)]
        + 0.2 * capture.nav_transform[(POS, SAMPLE_START)];
    assert!((captured_nav_cross - predicted_nav_cross).abs() < 1.0e-6);
    assert!((filter.nav_consider_covariance[(POS, 0)] - predicted_consider_cross).abs() < 1.0e-6);
    assert!((sample_cross[(POS, 0)] - predicted_sample_cross).abs() < 1.0e-6);
    assert!((filter.gap_nav_cross_covariance[(POS, 0)] - predicted_gap_cross).abs() < 1.0e-6);
    assert_eq!(capture.nav_transform[(POS, GAP_START)], 0.0);
}

#[test]
fn rejected_and_failed_gnss_leave_rts_capture_unchanged() {
    let mut filter = filter();
    let mut capture = std::boxed::Box::new(RtsUpdateCapture::new());
    let before = capture.nav_transform;
    let rejected = position_observation(SessionTime::ZERO, Vector3::repeat(1000.0));
    let outcome = filter
        .update_gnss_with_imu_sample_and_smoothing(
            &rejected,
            &zero_context(),
            gate(),
            None,
            None,
            Some(&mut capture),
        )
        .unwrap();
    assert!(matches!(
        outcome.position,
        Some(UpdateDecision::RejectedInnovation { .. })
    ));
    assert_eq!(capture.nav_transform, before);

    filter.covariance[(ATT, ATT + 1)] = 100.0;
    filter.covariance[(ATT + 1, ATT)] = 100.0;
    let observation = position_observation(SessionTime::ZERO, Vector3::repeat(0.01));
    assert_eq!(
        filter.update_gnss_with_imu_sample_and_smoothing(
            &observation,
            &zero_context(),
            gate(),
            None,
            None,
            Some(&mut capture)
        ),
        Err(EskfError::CovarianceNotPositiveSemidefinite)
    );
    assert_eq!(capture.nav_transform, before);
}

#[test]
fn cumulative_attitude_reset_follows_each_sequential_injection() {
    use crate::live::state::{NavVector, right_jacobian, so3_log};
    let mut filter = filter();
    let mut capture = std::boxed::Box::new(RtsUpdateCapture::new());
    let original_orientation = filter.state.orientation_n_from_b;
    let mut expected_reset = Matrix3::identity();
    for (axis, residual) in [(ATT, 0.15), (ATT + 1, 0.18)] {
        let innovation = filter.covariance[(axis, axis)] + 1.0;
        let correction =
            NavVector::from_fn(|row, _| filter.covariance[(row, axis)] * residual / innovation);
        let mut expected_state = filter.state;
        expected_reset = expected_state.inject(&correction).unwrap() * expected_reset;
        let mut measurement = LinearMeasurement::zeros();
        measurement.dimension = 1;
        measurement.h_nav[(0, axis)] = 1.0;
        measurement.noise[(0, 0)] = 1.0;
        measurement.residual[0] = residual;
        filter
            .linear_update(
                measurement,
                10.0,
                100.0,
                1.0,
                None,
                None,
                Some(&mut capture),
            )
            .unwrap();
    }
    assert!((capture.attitude_reset - expected_reset).norm() < 1.0e-7);
    let single_reset = right_jacobian(so3_log(
        &(original_orientation.inverse() * filter.state.orientation_n_from_b),
    ));
    assert!((capture.attitude_reset - single_reset).norm() > 1.0e-4);
}

#[test]
fn rts_propagation_capture_keeps_mean_transition_and_shared_sensitivity() {
    let mut filter = filter();
    filter.active_consider = 3;
    let mut batch = stationary_batch();
    batch.calibration_consider_start = Some(0);
    batch.mean_specific_force_consider_jacobian = Matrix3::identity() * 2.0;
    batch.mean_angular_rate_consider_jacobian = Matrix3::identity() * 3.0;
    let mut scratch = std::boxed::Box::new(EskfPropagationScratch::new());
    filter
        .propagate(&batch, &zero_context(), &mut scratch)
        .unwrap();
    let transition = &scratch.rts_transition;
    let dt = 0.005_f32;
    for axis in 0..3 {
        assert!((transition.nav[(POS + axis, VEL + axis)] - dt).abs() < 1.0e-7);
        assert!((transition.nav[(ATT + axis, GYRO_BIAS + axis)] + dt).abs() < 1.0e-7);
        assert!((transition.nav[(VEL + axis, ACC_BIAS + axis)] + dt).abs() < 1.0e-7);
        assert!((transition.nav[(VEL + axis, CONSIDER_START + axis)] - 2.0 * dt).abs() < 1.0e-7);
        assert!((transition.nav[(ATT + axis, CONSIDER_START + axis)] - 3.0 * dt).abs() < 1.0e-7);
    }
    assert!(!transition.retain_sample && !transition.retain_gap);
    assert_eq!(transition.nav.fixed_columns::<12>(SAMPLE_START).norm(), 0.0);
    for diagonal in 0..NAV_DIM {
        assert!((transition.nav[(diagonal, diagonal)] - 1.0).abs() < 1.0e-5);
    }
}

#[test]
fn rts_transition_only_retains_latents_that_existed_at_the_left_endpoint() {
    let mut filter = filter();
    let sample = ImuSampleCovariance::identity() * 0.1;
    let mut scratch = std::boxed::Box::new(EskfPropagationScratch::new());
    let batch = sample_batch(0, 4_000_000, &sample);
    filter
        .propagate_with_imu_sample(
            &batch,
            &zero_context(),
            Some(&GapNavCrossCovariance::zeros()),
            &mut scratch,
        )
        .unwrap();
    assert!(scratch.rts_transition.retain_sample);
    assert!((scratch.rts_transition.nav[(VEL, SAMPLE_START)] + 0.004).abs() < 1.0e-7);
    assert!((scratch.rts_transition.nav[(ATT, SAMPLE_START + 3)] + 0.004).abs() < 1.0e-7);

    let mut closing_sample = sample_batch(4_000_000, 5_000_000, &sample);
    closing_sample
        .leading_sample
        .as_mut()
        .unwrap()
        .active_at_end = false;
    let mut prior_sample_cross = GapNavCrossCovariance::zeros();
    scratch.commit_sample_candidate_into(&mut prior_sample_cross);
    filter
        .propagate_with_imu_sample(
            &closing_sample,
            &zero_context(),
            Some(&prior_sample_cross),
            &mut scratch,
        )
        .unwrap();
    assert!(!scratch.rts_transition.retain_sample);
    assert!((scratch.rts_transition.nav[(VEL, SAMPLE_START)] + 0.001).abs() < 1.0e-7);

    let model = GapModel {
        maximum_gap_ns: 10_000_000,
        angular_acceleration_one_sigma: Vector3::repeat(4.0),
        jerk_one_sigma: Vector3::repeat(20.0),
    };
    let first = gap_batch(5_000_000, 6_000_000, 0, true, model);
    filter
        .propagate(&first, &zero_context(), &mut scratch)
        .unwrap();
    assert!(!scratch.rts_transition.retain_gap);
    assert_eq!(
        scratch
            .rts_transition
            .nav
            .fixed_columns::<6>(GAP_START)
            .norm(),
        0.0
    );
    let continuing = gap_batch(6_000_000, 8_000_000, 1_000_000, true, model);
    filter
        .propagate_with_imu_sample(
            &continuing,
            &zero_context(),
            Some(&GapNavCrossCovariance::zeros()),
            &mut scratch,
        )
        .unwrap();
    assert!(scratch.rts_transition.retain_gap);
    assert!(
        scratch
            .rts_transition
            .nav
            .fixed_columns::<6>(GAP_START)
            .norm()
            > 0.0
    );
    let closing = gap_batch(8_000_000, 10_000_000, 3_000_000, false, model);
    filter
        .propagate_with_imu_sample(
            &closing,
            &zero_context(),
            Some(&GapNavCrossCovariance::zeros()),
            &mut scratch,
        )
        .unwrap();
    assert!(!scratch.rts_transition.retain_gap);
    assert!(
        scratch
            .rts_transition
            .nav
            .fixed_columns::<6>(GAP_START)
            .norm()
            > 0.0
    );
}
