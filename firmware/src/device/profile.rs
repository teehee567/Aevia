//! Explicitly unqualified V2 Mini bench profile. Factory scale factors are
//! usable for bring-up; installation, noise and timing priors still need capture.

use aevia_trajectory::{
    config::*,
    frame::*,
    ids::*,
    math::{FiniteF64, NonNegativeF64, Probability, UnitQuaternion, Vector3},
    time::{DurationNs, SessionTime, TimeSpan},
    uncertainty::{Covariance3, MeasurementUncertainty, SharedParameterCovariance},
};
use core::mem::MaybeUninit;

pub(super) const FRAME: TerrestrialFrame = TerrestrialFrame::new(
    FrameId::new(10),
    // BESTNAV names WGS84, without identifying a particular realization.
    TerrestrialRealization::Wgs84Ensemble,
    match CoordinateEpoch::from_decimal_year(2026.0) {
        Ok(value) => value,
        Err(_) => panic!("invalid fixed coordinate epoch"),
    },
    ReferenceEllipsoid::WGS84,
);

static mut SHARED: MaybeUninit<[SharedParameterDefinition; 2]> = MaybeUninit::uninit();
static mut POINTS: MaybeUninit<[ReferencePoint; 2]> = MaybeUninit::uninit();
static mut OPERATIONS: MaybeUninit<[CoordinateOperation; 1]> = MaybeUninit::uninit();

// Six shared parameters: 0.1 rad boresight and 0.1 m unknown antenna offset.
static SHARED_COVARIANCE: [f64; 21] = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.01, 0.0, 0.01,
];

fn finite(value: f64) -> FiniteF64 {
    FiniteF64::new(value).unwrap()
}
fn nonnegative(value: f64) -> NonNegativeF64 {
    NonNegativeF64::new(value).unwrap()
}
fn covariance(value: f64) -> Covariance3 {
    Covariance3::diagonal(value, value, value).unwrap()
}

// Development configuration identities only, never qualification attestations.
fn digest(component: u8) -> ContentDigestV1 {
    let mut bytes = *b"AEVIA-UNQUALIFIED-DEVICE-V2-----";
    bytes[31] = component;
    ContentDigestV1::from_bytes(bytes)
}

pub(super) unsafe fn engine() -> EngineConfig<'static> {
    let validity = TimeSpan::new(
        SessionTime::from_ns(i64::MIN),
        SessionTime::from_ns(i64::MAX),
    )
    .unwrap();
    let boresight = SharedParameterId::new(1);
    let lever = SharedParameterId::new(2);
    let zero_lever = BodyLeverArm::new(0.0, 0.0, 0.0).unwrap();
    let prior = MeasurementUncertainty::Provided(covariance(0.01));
    let definitions = unsafe {
        (&mut *core::ptr::addr_of_mut!(SHARED)).write([
            SharedParameterDefinition {
                id: boresight,
                kind: SharedParameterKind::BoresightRadians,
                mean: SharedParameterMean::Vector3(Vector3::ZERO),
                validity,
            },
            SharedParameterDefinition {
                id: lever,
                kind: SharedParameterKind::LeverArmMetres,
                mean: SharedParameterMean::Vector3(Vector3::ZERO),
                validity,
            },
        ])
    };
    let points = unsafe {
        (&mut *core::ptr::addr_of_mut!(POINTS)).write([
            ReferencePoint::new(
                ReferencePointId::new(1),
                ReferencePointKind::ImuSensingCenter,
                zero_lever,
                boresight,
                prior,
            ),
            ReferencePoint::new(
                ReferencePointId::new(2),
                ReferencePointKind::GnssAntennaPhaseCenter,
                zero_lever,
                lever,
                prior,
            ),
        ])
    };
    let operations = unsafe {
        (&mut *core::ptr::addr_of_mut!(OPERATIONS)).write([CoordinateOperation::new(
            CoordinateOperationId::new(1),
            FRAME.id(),
            FRAME.id(),
            CoordinateOperationKind::Identity,
            digest(13),
            None,
            Some(nonnegative(0.0)),
            false,
        )
        .unwrap()])
    };
    EngineConfig {
        installation: Installation {
            imu_sensor_frame: FrameId::new(20),
            body_from_imu: RotationParameter {
                parameter_id: boresight,
                mean: SensorToBodyRotation::from_quaternion(UnitQuaternion::IDENTITY),
                uncertainty: prior,
            },
            imu_to_gnss_antenna: LeverArmParameter {
                parameter_id: lever,
                mean: zero_lever,
                uncertainty: prior,
            },
            reference_points: points,
            attachment: AttachmentModel::RigidBody,
            dynamics_profile: DynamicsProfileId::new(1),
            calibration_revision: CalibrationRevision::new(1),
            digest: digest(2),
        },
        calibration: CalibrationBundle {
            revision: CalibrationRevision::new(1),
            input_profile: InputProfileId::new(1),
            shared_parameters: SharedParameterSet {
                definitions,
                covariance: SharedParameterCovariance::new(6, &SHARED_COVARIANCE).unwrap(),
                treatment: SharedUncertaintyTreatment::SchmidtConsider,
            },
            digest: digest(3),
        },
        input_profile: InputProfileSpec {
            id: InputProfileId::new(1),
            imu_rate_hz_range: (nonnegative(100.0), nonnegative(1750.0)),
            maximum_imu_samples_per_second: 1750,
            maximum_position_updates_per_second: 50,
            maximum_velocity_updates_per_second: 50,
            maximum_raw_signals_per_epoch: 0,
            digest: digest(7),
        },
        dynamics_profile: DynamicsProfileSpec {
            id: DynamicsProfileId::new(1),
            attachment: AttachmentModel::RigidBody,
            process_noise: ProcessNoiseSpec {
                accelerometer: covariance(0.0001),
                gyroscope: covariance(1.0e-6),
                accelerometer_bias: covariance(1.0e-6),
                gyroscope_bias: covariance(1.0e-10),
            },
            stationary: StationaryClassifierSpec {
                probability_stays_stationary: Probability::new(0.995).unwrap(),
                probability_motion_to_stationary: Probability::new(0.01).unwrap(),
                enter_probability: Probability::new(0.95).unwrap(),
                exit_probability: Probability::new(0.2).unwrap(),
                minimum_window_samples: 100,
                zupt_covariance: covariance(0.01),
                zupt_nis_threshold: nonnegative(16.0),
            },
            heading: HeadingObservabilitySpec {
                minimum_yaw_information: nonnegative(10.0),
                maximum_yaw_variance_rad2: nonnegative(0.25),
                minimum_course_snr: nonnegative(3.0),
                maximum_course_variance_rad2: nonnegative(0.25),
                dwell: DurationNs::from_ns(500_000_000),
            },
            gnss: GnssFusionSpec {
                position_covariance_floor: covariance(0.0004),
                velocity_covariance_floor: covariance(0.0009),
                nis_rejection_threshold: nonnegative(30.0),
                robust_weight_threshold: nonnegative(12.0),
                maximum_covariance_inflation: nonnegative(10.0),
                maximum_correction_age: DurationNs::from_ns(2_000_000_000),
                // All 50Hz updates enter the estimator; inflate their covariance
                // provisionally because inter-epoch correlation is unmeasured.
                correlation: GnssCorrelationPolicy::SequenceInflation {
                    multiplier: nonnegative(5.0),
                },
            },
            permits_non_holonomic_constraint: false,
            digest: digest(8),
        },
        navigation_profile: NavigationProfileSpec {
            revision: 1,
            navigation_cadence_hz: 200,
            fusion_delay: DurationNs::from_ns(100_000_000),
            // Keep the high-rate acquisition path independent of display cadence.
            smoothing_lag: DurationNs::from_ns(20_000_000),
            history_guard: DurationNs::from_ns(50_000_000),
            maximum_bridgeable_imu_gap: DurationNs::from_ns(10_000_000),
            reanchor_distance_m: nonnegative(1_000.0),
            reanchor_hysteresis_m: nonnegative(100.0),
            consider_dimension: 8,
            predictor_time_constant: DurationNs::from_ns(100_000_000),
            predictor_reset_position_m: nonnegative(10.0),
            covariance_repair: CovarianceRepairPolicy {
                maximum_attempts: 3,
                maximum_total_regularization: nonnegative(1.0e-4),
            },
            embedded_tuning: EmbeddedLiveTuning {
                gravity_magnitude_mps2: nonnegative(9.80665),
                gravity_vertical_gradient_s2: finite(-3.086e-6),
                stationary_gyro_score_variance: nonnegative(0.001),
                stationary_force_norm_score_variance: nonnegative(0.04),
                minimum_coarse_alignment_samples: 100,
                minimum_gyrocompass_samples: 1_000,
                gyrocompassing_qualified: false,
                minimum_earth_rate_cross_gravity: nonnegative(1.0e-8),
                maximum_static_force_variance: nonnegative(0.25),
                maximum_static_gyro_variance: nonnegative(0.001),
                roll_pitch_variance_rad2: nonnegative(0.01),
                unobservable_yaw_variance_rad2: nonnegative(
                    core::f64::consts::PI * core::f64::consts::PI,
                ),
                accelerometer_bias_prior_mps2: [finite(0.0); 3],
                gyroscope_bias_prior_rad_s: [finite(0.0); 3],
                accelerometer_bias_variance: [nonnegative(0.01); 3],
                gyroscope_bias_variance: [nonnegative(1.0e-4); 3],
                gap_jerk_one_sigma_mps3: [nonnegative(10.0); 3],
                gap_angular_acceleration_one_sigma_rad_s2: [nonnegative(2.0); 3],
                bias_correction_validity_norm: nonnegative(1.0),
                predictor_reset_velocity_mps: nonnegative(5.0),
                predictor_reset_attitude_rad: nonnegative(0.5),
                covariance_state_scales: [nonnegative(1.0); 15],
                covariance_minimum_variances: [nonnegative(1.0e-10); 15],
                covariance_repair_initial: nonnegative(1.0e-8),
                covariance_repair_growth: nonnegative(10.0),
            },
            digest: digest(9),
        },
        numeric_profile: NumericProfileSpec {
            revision: 1,
            scalar_policy: ScalarPolicy::EmbeddedMixedF32F64,
            fma_policy: FmaPolicy::Disabled,
            minimum_rust_version: (1, 87, 0),
            fpmath_source_digest: digest(10),
            toolchain_digest: digest(11),
            digest: digest(12),
        },
        processing_frame: FRAME,
        coordinate_operations: operations,
        uncertainty_models: &[],
        qualification: QualificationStatus::Unqualified,
        digest: digest(1),
    }
}
