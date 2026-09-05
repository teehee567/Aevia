//! Gap regression coverage.

use super::super::{
    ConsiderCovariance, Eskf, GapNavCrossCovariance, NavConsiderCovariance, ProcessNoise,
    covariance::CovariancePolicy,
};
use super::support::{gate, position_observation, propagate_test, zero_context};
use crate::{
    live::{
        preintegration::{GapModel, ImuInterval, ImuNoise, PreintegratedBatch, Preintegrator},
        state::{NavMatrix, NavState, POS},
    },
    time::SessionTime,
};
use nalgebra::{Matrix3, Vector3};

fn gap_batch(
    start_ns: i64,
    end_ns: i64,
    elapsed_ns: u32,
    continues: bool,
    model: GapModel,
) -> PreintegratedBatch {
    let mut preintegrator = Preintegrator::new(
        SessionTime::from_ns(start_ns),
        Vector3::zeros(),
        Vector3::zeros(),
        1.0,
    )
    .unwrap();
    preintegrator
        .push_gap(
            ImuInterval {
                start: SessionTime::from_ns(start_ns),
                end: SessionTime::from_ns(end_ns),
                omega_ib_b: Vector3::zeros(),
                specific_force_b: Vector3::zeros(),
                degraded_input: false,
                gap_elapsed_ns_plus_one: elapsed_ns + 1,
                body_from_sensor: nalgebra::UnitQuaternion::identity(),
                accel_sample_covariance: crate::live::preintegration::CompactCovariance3::ZERO,
                gyro_sample_covariance: crate::live::preintegration::CompactCovariance3::ZERO,
                calibration_consider_start: None,
            },
            ImuNoise {
                accel_covariance_density: Matrix3::zeros(),
                gyro_covariance_density: Matrix3::zeros(),
            },
            model,
            continues,
        )
        .unwrap();
    preintegrator.batch().unwrap()
}

fn gap_filter() -> Eskf {
    Eskf::new(
        NavState::stationary(SessionTime::ZERO),
        NavMatrix::identity() * 1.0e-3,
        NavConsiderCovariance::zeros(),
        ConsiderCovariance::zeros(),
        0,
        ProcessNoise {
            accel_bias_random_walk_covariance_density: Matrix3::zeros(),
            gyro_bias_random_walk_covariance_density: Matrix3::zeros(),
        },
        CovariancePolicy::conservative_candidate(),
    )
    .unwrap()
}

#[test]
fn held_derivative_latent_makes_split_and_unsplit_gap_propagation_equivalent() {
    let model = GapModel {
        maximum_gap_ns: 10_000_000,
        angular_acceleration_one_sigma: Vector3::new(4.0, 5.0, 6.0),
        jerk_one_sigma: Vector3::new(20.0, 30.0, 40.0),
    };
    let mut unsplit = gap_filter();
    let mut split = gap_filter();
    propagate_test(
        &mut unsplit,
        &gap_batch(0, 10_000_000, 0, false, model),
        &zero_context(),
    )
    .unwrap();
    propagate_test(
        &mut split,
        &gap_batch(0, 4_000_000, 0, true, model),
        &zero_context(),
    )
    .unwrap();
    assert_eq!(split.gap_origin, Some(SessionTime::ZERO));
    assert!(split.gap_nav_cross_covariance.norm() > 0.0);
    propagate_test(
        &mut split,
        &gap_batch(4_000_000, 10_000_000, 4_000_000, false, model),
        &zero_context(),
    )
    .unwrap();

    assert_eq!(split.gap_origin, None);
    assert_eq!(
        split.gap_nav_cross_covariance,
        GapNavCrossCovariance::zeros()
    );
    assert!((split.state.position_n - unsplit.state.position_n).norm() < 1.0e-9);
    assert!((split.state.velocity_n - unsplit.state.velocity_n).norm() < 1.0e-9);
    assert!((split.covariance - unsplit.covariance).norm() < 2.0e-8);
}

#[test]
fn gnss_cut_conditions_the_retained_gap_cross_covariance() {
    let model = GapModel {
        maximum_gap_ns: 10_000_000,
        angular_acceleration_one_sigma: Vector3::repeat(10.0),
        jerk_one_sigma: Vector3::repeat(1_000.0),
    };
    let mut filter = gap_filter();
    propagate_test(
        &mut filter,
        &gap_batch(0, 5_000_000, 0, true, model),
        &zero_context(),
    )
    .unwrap();
    let prior_cross = filter.gap_nav_cross_covariance;
    let noise = Matrix3::identity() * 0.01;
    let innovation = filter.covariance.fixed_view::<3, 3>(POS, POS).into_owned() + noise;
    let gain =
        filter.covariance.fixed_columns::<3>(POS).into_owned() * innovation.try_inverse().unwrap();
    let expected = prior_cross - gain * prior_cross.fixed_rows::<3>(POS).into_owned();
    let mut observation = position_observation(filter.state.time, filter.state.position_n);
    observation.position_covariance_n = noise;
    filter
        .update_gnss(&observation, &zero_context(), gate())
        .unwrap();
    assert!((filter.gap_nav_cross_covariance - expected).norm() < 2.0e-7);
    assert!((filter.gap_nav_cross_covariance - prior_cross).norm() > 0.0);

    propagate_test(
        &mut filter,
        &gap_batch(5_000_000, 10_000_000, 5_000_000, false, model),
        &zero_context(),
    )
    .unwrap();
    assert_eq!(filter.gap_origin, None);
}
