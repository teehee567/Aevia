//! Compile semantic profiles into private initializer and live-core configuration.

use super::conversion::{
    array_f32_nonnegative, covariance_density, earth_rate_n, finite_f32, vector_f32_finite,
    vector_f32_nonnegative,
};
use crate::config::EngineConfig;
use crate::error::{PrepareError, StepError, ValidationError};
use crate::live::{
    AlignmentConfig, ConsiderCovariance, EcefAnchor, GapModel, ImuNoise, LiveCoreConfig,
    MAX_CONSIDER, MechanizationContext, NisGate, PredictorConfig, ProcessNoise, StationaryConfig,
};
use nalgebra::{Matrix3, Vector3 as NaVector3};

pub(super) fn make_initializer(
    engine: &EngineConfig<'_>,
) -> Result<crate::live::Initializer, PrepareError> {
    let tuning = engine.navigation_profile.embedded_tuning;
    let stationary = engine.dynamics_profile.stationary;
    crate::live::Initializer::new(AlignmentConfig {
        stationary: StationaryConfig {
            gravity_magnitude: tuning.gravity_magnitude_mps2.get() as f32,
            gyro_score_variance: tuning.stationary_gyro_score_variance.get() as f32,
            force_norm_score_variance: tuning.stationary_force_norm_score_variance.get() as f32,
            probability_stays_stationary: stationary.probability_stays_stationary.get() as f32,
            probability_motion_becomes_stationary: stationary.probability_motion_to_stationary.get()
                as f32,
            enter_probability: stationary.enter_probability.get() as f32,
            exit_probability: stationary.exit_probability.get() as f32,
            minimum_window_samples: stationary.minimum_window_samples,
        },
        minimum_coarse_samples: tuning.minimum_coarse_alignment_samples,
        minimum_gyrocompass_samples: tuning.minimum_gyrocompass_samples,
        gyrocompassing_qualified: tuning.gyrocompassing_qualified,
        minimum_earth_rate_cross_gravity: tuning.minimum_earth_rate_cross_gravity.get() as f32,
        maximum_force_variance: tuning.maximum_static_force_variance.get() as f32,
        maximum_gyro_variance: tuning.maximum_static_gyro_variance.get() as f32,
        minimum_dynamic_yaw_information: engine
            .dynamics_profile
            .heading
            .minimum_yaw_information
            .get() as f32,
        maximum_dynamic_yaw_variance: engine
            .dynamics_profile
            .heading
            .maximum_yaw_variance_rad2
            .get() as f32,
        roll_pitch_variance: tuning.roll_pitch_variance_rad2.get() as f32,
        unobservable_yaw_variance: tuning.unobservable_yaw_variance_rad2.get() as f32,
        accel_bias_prior: vector_f32_finite(tuning.accelerometer_bias_prior_mps2),
        gyro_bias_prior: vector_f32_finite(tuning.gyroscope_bias_prior_rad_s),
        accel_bias_variance: vector_f32_nonnegative(tuning.accelerometer_bias_variance),
        gyro_bias_variance: vector_f32_nonnegative(tuning.gyroscope_bias_variance),
    })
    .map_err(|_| PrepareError::IncompatibleProfile)
}

pub(super) fn make_live_core_config(
    engine: &EngineConfig<'_>,
    anchor: &EcefAnchor,
    imu_noise: ImuNoise,
) -> Result<LiveCoreConfig, StepError> {
    let navigation = engine.navigation_profile;
    let tuning = navigation.embedded_tuning;
    let cadence = i64::from(navigation.navigation_cadence_hz);
    let navigation_period_ns = 1_000_000_000_i64
        .checked_div(cadence)
        .ok_or(StepError::WorkspaceContract)?;
    let earth_rate_n = earth_rate_n(anchor)?;
    let mut gravity_gradient = Matrix3::zeros();
    gravity_gradient[(2, 2)] = -(tuning.gravity_vertical_gradient_s2.get() as f32);
    let mechanization = MechanizationContext::new(
        earth_rate_n,
        NaVector3::new(0.0, 0.0, -(tuning.gravity_magnitude_mps2.get() as f32)),
        gravity_gradient,
    )
    .map_err(|_| StepError::EstimatorFailure)?;
    let process = engine.dynamics_profile.process_noise;
    let time_constant = navigation.predictor_time_constant.as_seconds_f64() as f32;
    Ok(LiveCoreConfig {
        fusion_delay_ns: i64::try_from(navigation.fusion_delay.as_ns())
            .map_err(|_| StepError::WorkspaceContract)?,
        navigation_period_ns,
        bias_correction_validity_norm: tuning.bias_correction_validity_norm.get() as f32,
        mechanization,
        imu_noise,
        process_noise: ProcessNoise {
            accel_bias_random_walk_covariance_density: covariance_density(
                process.accelerometer_bias,
            )?,
            gyro_bias_random_walk_covariance_density: covariance_density(process.gyroscope_bias)?,
        },
        covariance_policy: crate::live::CovariancePolicy {
            state_scales: array_f32_nonnegative(tuning.covariance_state_scales),
            minimum_variance: array_f32_nonnegative(tuning.covariance_minimum_variances),
            repair_initial: tuning.covariance_repair_initial.get() as f32,
            repair_growth: tuning.covariance_repair_growth.get() as f32,
            maximum_total_repair: navigation
                .covariance_repair
                .maximum_total_regularization
                .get() as f32,
            maximum_repair_attempts: navigation.covariance_repair.maximum_attempts,
        },
        nis_gate: NisGate {
            soft_3d: engine.dynamics_profile.gnss.robust_weight_threshold.get() as f32,
            hard_3d: engine.dynamics_profile.gnss.nis_rejection_threshold.get() as f32,
            soft_6d: engine.dynamics_profile.gnss.robust_weight_threshold.get() as f32,
            hard_6d: engine.dynamics_profile.gnss.nis_rejection_threshold.get() as f32,
            maximum_covariance_inflation: engine
                .dynamics_profile
                .gnss
                .maximum_covariance_inflation
                .get() as f32,
        },
        predictor: PredictorConfig {
            position_time_constant_s: time_constant,
            velocity_time_constant_s: time_constant,
            attitude_time_constant_s: time_constant,
            position_reset_threshold_m: navigation.predictor_reset_position_m.get() as f32,
            velocity_reset_threshold_mps: tuning.predictor_reset_velocity_mps.get() as f32,
            attitude_reset_threshold_rad: tuning.predictor_reset_attitude_rad.get() as f32,
        },
        gap: GapModel {
            maximum_gap_ns: i64::try_from(navigation.maximum_bridgeable_imu_gap.as_ns())
                .map_err(|_| StepError::WorkspaceContract)?,
            angular_acceleration_one_sigma: vector_f32_nonnegative(
                tuning.gap_angular_acceleration_one_sigma_rad_s2,
            ),
            jerk_one_sigma: vector_f32_nonnegative(tuning.gap_jerk_one_sigma_mps3),
        },
    })
}

pub(super) fn make_consider_covariance(
    engine: &EngineConfig<'_>,
    clock: crate::config::InitialClockConsiderPrior,
) -> Result<ConsiderCovariance, StepError> {
    let active = usize::from(engine.navigation_profile.consider_dimension);
    if active > MAX_CONSIDER {
        return Err(StepError::WorkspaceContract);
    }
    let mut covariance = ConsiderCovariance::zeros();
    covariance[(0, 0)] = finite_f32(clock.offset_variance_s2.get())?;
    covariance[(1, 1)] = finite_f32(clock.drift_variance.get())?;
    let cross = finite_f32(clock.offset_drift_covariance_s.get())?;
    covariance[(0, 1)] = cross;
    covariance[(1, 0)] = cross;
    let clock_shared = clock.cross_covariance_with_shared.values();
    if clock.cross_covariance_with_shared.shared_dimension()
        != engine.calibration.shared_parameters.covariance.dimension()
    {
        return Err(StepError::InvalidObservation(
            ValidationError::InvalidCovariance,
        ));
    }
    for shared_coordinate in 0..clock.cross_covariance_with_shared.shared_dimension() {
        let offset_cross = finite_f32(clock_shared[0][shared_coordinate])?;
        let drift_cross = finite_f32(clock_shared[1][shared_coordinate])?;
        covariance[(0, shared_coordinate + 2)] = offset_cross;
        covariance[(shared_coordinate + 2, 0)] = offset_cross;
        covariance[(1, shared_coordinate + 2)] = drift_cross;
        covariance[(shared_coordinate + 2, 1)] = drift_cross;
    }
    let shared = engine.calibration.shared_parameters.covariance;
    let mut packed = 0;
    for row in 0..shared.dimension() {
        for column in row..shared.dimension() {
            let value = finite_f32(shared.upper_triangle()[packed])?;
            covariance[(row + 2, column + 2)] = value;
            covariance[(column + 2, row + 2)] = value;
            packed += 1;
        }
    }
    Ok(covariance)
}
