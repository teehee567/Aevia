//! Disposable, explicitly unqualified on-device trajectory experiment.
//!
//! Uses the real EmbeddedLive estimator. The factory-scaled IMU has no
//! board-specific residual calibration, the antenna lever arm is unsurveyed,
//! and time is registered at acquisition rather than locked to GNSS PPS.
//! These limitations are represented by development status and broad priors.

use core::{
    mem::{MaybeUninit, align_of, size_of},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use aevia_trajectory::{
    TrajectoryEngine,
    config::*,
    engine::{LivePhase, LiveSession},
    error::{PrepareError, StepError},
    frame::*,
    ids::*,
    math::{FiniteF64, NonNegativeF64, Probability, UnitQuaternion, Vector3},
    metric::{LiveMetricLimits, LiveMetricPlan, MetricPlan},
    observation::*,
    quality::DiagnosticCounts,
    time::{
        DurationNs, ObservationTime, SampleSupport, SessionTime, SignedDurationNs, TimeSpan,
        TimingBasis,
    },
    uncertainty::{Covariance3, MeasurementUncertainty, SharedParameterCovariance, Variance},
    workspace::{LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion},
};

use crate::gps::BestNav;

const FRAME: TerrestrialFrame = TerrestrialFrame::new(
    FrameId::new(10),
    // BESTNAV names WGS84, without identifying a particular realization.
    TerrestrialRealization::Wgs84Ensemble,
    match CoordinateEpoch::from_decimal_year(2026.0) {
        Ok(value) => value,
        Err(_) => panic!("invalid fixed coordinate epoch"),
    },
    ReferenceEllipsoid::WGS84,
);

// ROM templates copy directly into PSRAM; neither multi-megabyte object is
// ever materialized on a task stack. Large immutable metric configuration is
// also kept in PSRAM, despite the empty metric plan used for the speed display.
static PSRAM_TEMPLATE: LivePsramWorkspace = LivePsramWorkspace::new(FRAME);
static METRIC_TEMPLATE: LiveMetricPlan = LiveMetricPlan::placeholder();
static EMPTY_METRICS: MetricPlan = MetricPlan::new(1);
static STARTED: AtomicBool = AtomicBool::new(false);
/// Startup progress for the USB task: 1 checks, 2 cold copy, 3 metric copy,
/// 4 metric compile, 5 profile, 6 preflight, 7 session start, 8 ready.
pub static START_STAGE: AtomicU32 = AtomicU32::new(0);
static mut INTERNAL: LiveInternalWorkspace = LiveInternalWorkspace::new();
static mut SHARED: MaybeUninit<[SharedParameterDefinition; 2]> = MaybeUninit::uninit();
static mut POINTS: MaybeUninit<[ReferencePoint; 2]> = MaybeUninit::uninit();
static mut OPERATIONS: MaybeUninit<[CoordinateOperation; 1]> = MaybeUninit::uninit();

// Six shared parameters: 0.1 rad boresight and 0.1 m unknown antenna offset.
static SHARED_COVARIANCE: [f64; 21] = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.01, 0.0, 0.01,
];

#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    /// Latest corrected trajectory speed at the IMU centre, about 100 ms delayed.
    pub speed_mps: Option<f64>,
    pub epoch_us: Option<u64>,
    pub phase: LivePhase,
    pub diagnostics: DiagnosticCounts,
    pub last_input: Option<InputDisposition>,
    pub last_fusion: Option<InputDisposition>,
}

pub struct Poc {
    session: LiveSession<'static, 'static>,
    imu_sequence: u64,
    gnss_sequence: u64,
    last_gnss_time_ms: Option<u64>,
    snapshot: Snapshot,
}

pub const fn internal_workspace_bytes() -> usize {
    size_of::<LiveInternalWorkspace>()
}

pub const fn psram_required_bytes() -> usize {
    size_of::<LivePsramWorkspace>() + size_of::<LiveMetricPlan>() + 32
}

/// Initializes the one development session, copying its cold storage to PSRAM.
///
/// # Safety
/// `psram_ptr..psram_ptr+psram_len` must be initialized, writable PSRAM and
/// reserved exclusively for this session for the remainder of the program.
/// Call only after PSRAM cache initialization, on one estimator task/core.
pub unsafe fn start(psram_ptr: *mut u8, psram_len: usize) -> Result<Poc, PrepareError> {
    START_STAGE.store(1, Ordering::Relaxed);
    let address = psram_ptr as usize;
    let aligned = address
        .checked_add(15)
        .ok_or(PrepareError::InsufficientResources)?
        & !15;
    let cold_bytes = size_of::<LivePsramWorkspace>();
    let metric_address = aligned
        .checked_add(cold_bytes)
        .and_then(|v| v.checked_add(align_of::<LiveMetricPlan>() - 1))
        .map(|v| v & !(align_of::<LiveMetricPlan>() - 1))
        .ok_or(PrepareError::InsufficientResources)?;
    let end = metric_address
        .checked_add(size_of::<LiveMetricPlan>())
        .ok_or(PrepareError::InsufficientResources)?;
    if psram_ptr.is_null() || end.checked_sub(address).is_none_or(|n| n > psram_len) {
        return Err(PrepareError::InsufficientResources);
    }
    if STARTED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(PrepareError::IncompatibleProfile);
    }
    let cold = aligned as *mut LivePsramWorkspace;
    let metrics = metric_address as *mut LiveMetricPlan;
    START_STAGE.store(2, Ordering::Relaxed);
    // SAFETY: unique checked PSRAM ranges; templates have valid constructors
    // and own no allocations or references to mutable storage.
    unsafe {
        core::ptr::copy_nonoverlapping(&PSRAM_TEMPLATE, cold, 1);
        START_STAGE.store(3, Ordering::Relaxed);
        core::ptr::copy_nonoverlapping(&METRIC_TEMPLATE, metrics, 1);
    }
    let metrics = unsafe { &mut *metrics };
    START_STAGE.store(4, Ordering::Relaxed);
    EMPTY_METRICS
        .compile_live_into(LiveMetricLimits::default(), metrics)
        .map_err(PrepareError::InvalidDefinition)?;
    START_STAGE.store(5, Ordering::Relaxed);
    let engine = unsafe { development_engine() };
    START_STAGE.store(6, Ordering::Relaxed);
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: SessionId::from_bytes(*b"AEVIA-SPEED-POC1"),
        engine,
        metrics,
        resources: LiveResourceLimits::V2_MINI_RTS,
        // Speed does not require a surveyed body heading. The initializer can
        // align roll/pitch while explicitly retaining unobservable yaw.
        initial_heading: None,
        initial_clock_prior: InitialClockConsiderPrior {
            model: ClockModelId::new(1),
            segment: ClockSegmentId::new(1),
            reference_time: SessionTime::ZERO,
            offset_variance_s2: variance(0.020 * 0.020),
            drift_variance: variance(100.0e-6 * 100.0e-6),
            offset_drift_covariance_s: finite(0.0),
            cross_covariance_with_shared: ClockSharedCrossCovariance::independent(6).unwrap(),
        },
    })
    .preflight_development()?;
    let workspace = unsafe {
        LiveWorkspace::bind(
            &mut *core::ptr::addr_of_mut!(INTERNAL),
            MemoryRegion::InternalSram,
            &mut *cold,
            MemoryRegion::Psram,
        )
    };
    START_STAGE.store(7, Ordering::Relaxed);
    let mut session = plan.start(workspace)?;
    // Explicit bench approximation: integrate at nominal acquisition epochs
    // while retaining nonzero recorded timing uncertainty and degraded status.
    session
        .use_development_imu_timing_approximation()
        .map_err(|_| PrepareError::IncompatibleProfile)?;
    START_STAGE.store(8, Ordering::Relaxed);
    Ok(Poc {
        session,
        imu_sequence: 0,
        gnss_sequence: 0,
        last_gnss_time_ms: None,
        snapshot: Snapshot {
            speed_mps: None,
            epoch_us: None,
            phase: LivePhase::Initializing,
            diagnostics: DiagnosticCounts::default(),
            last_input: None,
            last_fusion: None,
        },
    })
}

impl Poc {
    /// Both vectors must be factory-scaled SI averages over the same interval.
    pub fn push_imu(
        &mut self,
        end_us: u64,
        duration_us: u32,
        acceleration_mps2: [f64; 3],
        angular_rate_rps: [f64; 3],
        saturated: bool,
    ) -> Result<(), StepError> {
        self.imu_sequence = self.imu_sequence.saturating_add(1);
        let mut time = arrival_time(end_us, 100_000)?;
        time.support = SampleSupport::IntervalAverage {
            duration: DurationNs::from_ns(u64::from(duration_us) * 1_000),
        };
        let axes = AxisStatus::new([true; 3], [saturated; 3]);
        let observation = ImuObservation::new(
            ObservationId::new(SourceId::new(1), self.imu_sequence),
            FrameId::new(20),
            InputProfileId::new(1),
            TimedAngularRate {
                value: SensorAngularRate::from_components(angular_rate_rps)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(covariance(0.001 * 0.001)),
                axes,
            },
            TimedSpecificForce {
                value: SensorSpecificForce::from_components(acceleration_mps2)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(covariance(0.03 * 0.03)),
                axes,
            },
            // A factory-scaled sensor with no board-specific calibration is a
            // usable development input, never a qualified ImuStatus::Valid.
            ImuStatus::Degraded,
        )
        .map_err(StepError::InvalidObservation)?;
        self.step(LiveObservation::Imu(observation))
    }

    pub fn push_gnss(&mut self, received_us: u64, nav: &BestNav<'_>) -> Result<(), StepError> {
        let receiver_ms = u64::from(nav.gps_week) * 604_800_000 + u64::from(nav.gps_tow_ms);
        if self
            .last_gnss_time_ms
            .is_some_and(|last| receiver_ms <= last)
        {
            return Ok(());
        }
        let latitude = f64::from(nav.latitude_e7) * 1.0e-7 * core::f64::consts::PI / 180.0;
        let longitude = f64::from(nav.longitude_e7) * 1.0e-7 * core::f64::consts::PI / 180.0;
        let (slat, clat) = (libm::sin(latitude), libm::cos(latitude));
        let (slon, clon) = (libm::sin(longitude), libm::cos(longitude));
        let basis = [
            [-slon, -slat * clon, clat * clon],
            [clon, -slat * slon, clat * slon],
            [0.0, clat, slat],
        ];
        let height = (f64::from(nav.height_mm) + f64::from(nav.undulation_mm)) * 0.001;
        let eccentricity_sq = 6.694_379_990_141_316_5e-3;
        let radius = 6_378_137.0 / libm::sqrt(1.0 - eccentricity_sq * slat * slat);
        let position = EcefPosition::new(
            (radius + height) * clat * clon,
            (radius + height) * clat * slon,
            (radius * (1.0 - eccentricity_sq) + height) * slat,
        )
        .map_err(StepError::InvalidObservation)?;
        let course = f64::from(nav.track_hundredths_deg) * 0.01 * core::f64::consts::PI / 180.0;
        let speed = f64::from(nav.horizontal_speed_mm_s) * 0.001;
        let velocity_enu = [
            speed * libm::sin(course),
            speed * libm::cos(course),
            f64::from(nav.vertical_speed_mm_s) * 0.001,
        ];
        let velocity = EcefVelocity::from_components(rotate(basis, velocity_enu))
            .map_err(StepError::InvalidObservation)?;
        let time = arrival_time(received_us, 20_000_000)?;
        let velocity_time = ObservationTime {
            // BESTNAV velocity latency applies to its velocity epoch.
            correction: SignedDurationNs::from_ns(-i64::from(nav.latency_ms) * 1_000_000),
            ..time
        };
        self.gnss_sequence = self.gnss_sequence.saturating_add(1);
        let solution = GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), self.gnss_sequence),
            ReferencePointId::new(2),
            Some(GnssPosition {
                value: position,
                time,
                frame: FRAME.id(),
                valid: true,
                uncertainty: MeasurementUncertainty::Provided(rotated_covariance(
                    basis,
                    [
                        f64::from(nav.longitude_sigma_mm).max(20.0) * 0.001,
                        f64::from(nav.latitude_sigma_mm).max(20.0) * 0.001,
                        f64::from(nav.height_sigma_mm).max(50.0) * 0.001,
                    ],
                )?),
            }),
            Some(GnssVelocity {
                value: velocity,
                time: velocity_time,
                frame: FRAME.id(),
                valid: true,
                uncertainty: MeasurementUncertainty::Provided(rotated_covariance(
                    basis,
                    [
                        f64::from(nav.horizontal_speed_sigma_mm_s).max(30.0) * 0.001,
                        f64::from(nav.horizontal_speed_sigma_mm_s).max(30.0) * 0.001,
                        f64::from(nav.vertical_speed_sigma_mm_s).max(50.0) * 0.001,
                    ],
                )?),
            }),
            None,
            GnssDiagnostics {
                correction_age: None,
                solution_age: None,
                // POC receiver health is narrowly CRC + SOL_COMPUTED in both
                // fields, enforced by parse_bestnava. It asserts no RTK lock,
                // correction source, antenna survey or qualification campaign.
                health: Some(TimedDiagnostic {
                    value: ReceiverHealth::Healthy,
                    time: velocity_time,
                    age: DurationNs::ZERO,
                }),
            },
        )
        .map_err(StepError::InvalidObservation)?;
        self.step(LiveObservation::GnssSolution(solution))?;
        self.last_gnss_time_ms = Some(receiver_ms);
        Ok(())
    }

    pub fn snapshot(&self) -> Snapshot {
        let mut snapshot = self.snapshot;
        let trajectory = self.session.trajectory();
        let time = trajectory.span().map(TimeSpan::end);
        snapshot.speed_mps = time.and_then(|time| {
            trajectory
                .horizontal_speed_at(time, ReferencePointId::new(1))
                .ok()
        });
        snapshot.epoch_us = time.and_then(|time| u64::try_from(time.as_ns() / 1_000).ok());
        snapshot
    }

    fn step(&mut self, observation: LiveObservation) -> Result<(), StepError> {
        let update = self.session.step(LiveStep {
            observation: Some(&observation),
            work: WorkQuota::new(128).unwrap(),
        })?;
        self.snapshot.phase = update.phase;
        self.snapshot.diagnostics = update.diagnostics;
        self.snapshot.last_input = update.input.map(|(_, disposition)| disposition);
        if let Some(fusion) = update.fusion {
            self.snapshot.last_fusion = Some(fusion.disposition);
        }
        Ok(())
    }
}

fn arrival_time(micros: u64, sigma_ns: u64) -> Result<ObservationTime, StepError> {
    let nanos = micros
        .checked_mul(1_000)
        .and_then(|v| i64::try_from(v).ok())
        .ok_or(StepError::InvalidObservation(
            aevia_trajectory::ValidationError::TimeOverflow,
        ))?;
    Ok(ObservationTime {
        registered_at: SessionTime::from_ns(nanos),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::from_ns(sigma_ns),
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis: TimingBasis::ArrivalOnly,
    })
}

fn rotate(rotation: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    rotation.map(|row| row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2])
}

fn rotated_covariance(rotation: [[f64; 3]; 3], sigma: [f64; 3]) -> Result<Covariance3, StepError> {
    let entry = |row: usize, col: usize| {
        (0..3)
            .map(|k| rotation[row][k] * rotation[col][k] * sigma[k] * sigma[k])
            .sum()
    };
    Covariance3::from_upper_triangle([
        entry(0, 0),
        entry(0, 1),
        entry(0, 2),
        entry(1, 1),
        entry(1, 2),
        entry(2, 2),
    ])
    .map_err(StepError::InvalidObservation)
}

fn finite(value: f64) -> FiniteF64 {
    FiniteF64::new(value).unwrap()
}
fn nonnegative(value: f64) -> NonNegativeF64 {
    NonNegativeF64::new(value).unwrap()
}
fn variance(value: f64) -> Variance {
    Variance::new(value).unwrap()
}
fn covariance(value: f64) -> Covariance3 {
    Covariance3::diagonal(value, value, value).unwrap()
}

// Development configuration identities only, never qualification attestations.
fn digest(component: u8) -> ContentDigestV1 {
    let mut bytes = *b"AEVIA-UNQUALIFIED-SPEED-POC-V1--";
    bytes[31] = component;
    ContentDigestV1::from_bytes(bytes)
}

unsafe fn development_engine() -> EngineConfig<'static> {
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
            imu_rate_hz_range: (nonnegative(100.0), nonnegative(400.0)),
            maximum_imu_samples_per_second: 400,
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
            // The console queries the latest forward-filter trajectory speed.
            // Keep the backward RTS pass off for this first throughput test.
            smoothing_lag: DurationNs::ZERO,
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
