//! Support regression coverage.

use super::super::{
    ConsiderCovariance, Eskf, EskfError, EskfPropagationScratch, GapNavCrossCovariance,
    NavConsiderCovariance, ProcessNoise,
    covariance::CovariancePolicy,
    gnss::{GnssObservation, NisGate, SharedMeasurementJacobians},
};
use crate::{
    live::{
        preintegration::{
            ImuInterval, ImuNoise, ImuSampleCovariance, PreintegratedBatch, Preintegrator,
        },
        state::{MechanizationContext, NavMatrix, NavState, POS},
    },
    quality::{GnssState, TimingQuality},
    time::SessionTime,
};
use nalgebra::{Matrix3, Vector3};

pub(super) fn zero_context() -> MechanizationContext {
    MechanizationContext::new(Vector3::zeros(), Vector3::zeros(), Matrix3::zeros()).unwrap()
}

pub(super) fn context() -> MechanizationContext {
    MechanizationContext::new(
        Vector3::new(0.0, 5.0e-5, 5.0e-5),
        Vector3::new(0.0, 0.0, -9.806_65),
        Matrix3::zeros(),
    )
    .unwrap()
}

pub(super) fn filter() -> Eskf {
    let mut covariance = NavMatrix::identity();
    covariance.fixed_view_mut::<3, 3>(POS, POS).scale_mut(4.0);
    Eskf::new(
        NavState::stationary(SessionTime::ZERO),
        covariance,
        NavConsiderCovariance::zeros(),
        ConsiderCovariance::zeros(),
        0,
        ProcessNoise {
            accel_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-8,
            gyro_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-10,
        },
        CovariancePolicy::conservative_candidate(),
    )
    .unwrap()
}

pub(super) fn propagate_test(
    filter: &mut Eskf,
    batch: &PreintegratedBatch,
    context: &MechanizationContext,
) -> Result<(), EskfError> {
    let mut scratch = std::boxed::Box::new(EskfPropagationScratch::new());
    if let Some(leading) = batch.leading_sample {
        // These older gap-only fixtures deliberately have no sample
        // noise. Their continuing sample still has an explicit identity.
        assert_eq!(leading.covariance, ImuSampleCovariance::zeros());
        filter.propagate_with_imu_sample(
            batch,
            context,
            Some(&GapNavCrossCovariance::zeros()),
            &mut scratch,
        )
    } else {
        filter.propagate(batch, context, &mut scratch)
    }
}

pub(super) fn stationary_batch() -> PreintegratedBatch {
    let mut preintegrator =
        Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
    let noise = ImuNoise {
        accel_covariance_density: Matrix3::identity() * 1.0e-6,
        gyro_covariance_density: Matrix3::identity() * 1.0e-8,
    };
    preintegrator
        .push(
            ImuInterval {
                start: SessionTime::ZERO,
                end: SessionTime::from_ns(5_000_000),
                omega_ib_b: context().earth_rate_n,
                specific_force_b: Vector3::new(0.0, 0.0, 9.806_65),
                degraded_input: false,
                gap_elapsed_ns_plus_one: 0,
                body_from_sensor: nalgebra::UnitQuaternion::identity(),
                accel_sample_covariance: crate::live::preintegration::CompactCovariance3::ZERO,
                gyro_sample_covariance: crate::live::preintegration::CompactCovariance3::ZERO,
                calibration_consider_start: None,
            },
            noise,
        )
        .unwrap();
    preintegrator.batch().unwrap()
}

pub(super) fn gate() -> NisGate {
    NisGate {
        soft_3d: 7.815,
        hard_3d: 25.0,
        soft_6d: 12.592,
        hard_6d: 35.0,
        maximum_covariance_inflation: 20.0,
    }
}

pub(super) fn position_observation(time: SessionTime, position: Vector3<f32>) -> GnssObservation {
    GnssObservation {
        time,
        position_n: Some(position),
        velocity_n: None,
        position_covariance_n: Matrix3::identity() * 0.01,
        velocity_covariance_n: Matrix3::identity(),
        position_velocity_cross_n: None,
        imu_to_antenna_b: Vector3::zeros(),
        omega_ib_b: Vector3::zeros(),
        specific_force_b: Vector3::zeros(),
        angular_acceleration_eb_b: None,
        angular_acceleration_covariance_b: Matrix3::zeros(),
        clock_consider_start: None,
        clock_reference_time: SessionTime::ZERO,
        lever_arm_consider_start: None,
        position_independent_timing_sigma_s: 0.0,
        velocity_independent_timing_sigma_s: 0.0,
        shared_jacobians: SharedMeasurementJacobians::default(),
        receiver_healthy: true,
        quality_state: GnssState::Healthy,
        quality_timing: TimingQuality::PpsCorrelated,
    }
}
