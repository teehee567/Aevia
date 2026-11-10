//! Single-owner trajectory worker, pinned to the second CPU by runtime.
//! Sensor queues and latest-value snapshots isolate acquisition from UI/USB.

use super::{
    interface::{self, EngineError, Phase, Snapshot},
    profile,
    sensors::{self, GnssSample, ImuSample},
    timing,
};
use aevia_trajectory::{
    LiveSession, PrepareError, StepError, TrajectoryEngine,
    config::{ClockSharedCrossCovariance, InitialClockConsiderPrior, LiveResourceLimits, LiveSpec},
    engine::LivePhase,
    frame::{SensorAngularRate, SensorSpecificForce},
    ids::{
        ClockModelId, ClockSegmentId, FrameId, InputProfileId, ObservationId, ReferencePointId,
        SessionId, SourceId,
    },
    math::FiniteF64,
    metric::{LiveMetricLimits, LiveMetricPlan, MetricPlan},
    observation::*,
    time::{
        DurationNs, ObservationTime, SampleSupport, SessionTime, SignedDurationNs, TimingBasis,
    },
    uncertainty::{Covariance3, MeasurementUncertainty, Variance},
    workspace::{LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion},
};
use core::{
    mem::{align_of, size_of},
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_time::Instant;
use esp_hal::time::Duration;
use esp_rtos::CurrentThreadHandle;

// Immutable ROM templates avoid constructing multi-megabyte values on a stack.
static PSRAM_TEMPLATE: LivePsramWorkspace = LivePsramWorkspace::new(profile::FRAME);
static METRIC_TEMPLATE: LiveMetricPlan = LiveMetricPlan::placeholder();
static METRICS: MetricPlan = MetricPlan::new(1);
static STARTED: AtomicBool = AtomicBool::new(false);
static mut INTERNAL: LiveInternalWorkspace = LiveInternalWorkspace::new();

pub const fn psram_required_bytes() -> usize {
    size_of::<LivePsramWorkspace>() + size_of::<LiveMetricPlan>() + 32
}

struct Processor {
    session: LiveSession<'static, 'static>,
    imu_sequence: u64,
    gnss_sequence: u64,
    have_initialization_seed: bool,
    imu_rate_covariance: Covariance3,
    imu_acceleration_covariance: Covariance3,
    last_gnss_epoch_ms: Option<(u32, u64)>,
    snapshot: Snapshot,
    rate_epoch_us: u64,
    rate_imu_samples: u32,
    rate_gnss_samples: u32,
}

impl Processor {
    /// The reserved PSRAM range must remain exclusive and alive forever.
    unsafe fn start(ptr: *mut u8, len: usize) -> Result<Self, PrepareError> {
        let address = ptr as usize;
        let cold_address = address
            .checked_add(15)
            .ok_or(PrepareError::InsufficientResources)?
            & !15;
        let metric_address = cold_address
            .checked_add(size_of::<LivePsramWorkspace>())
            .and_then(|v| v.checked_add(align_of::<LiveMetricPlan>() - 1))
            .map(|v| v & !(align_of::<LiveMetricPlan>() - 1))
            .ok_or(PrepareError::InsufficientResources)?;
        let end = metric_address
            .checked_add(size_of::<LiveMetricPlan>())
            .ok_or(PrepareError::InsufficientResources)?;
        if ptr.is_null() || end.checked_sub(address).is_none_or(|n| n > len) {
            return Err(PrepareError::InsufficientResources);
        }
        if STARTED.swap(true, Ordering::AcqRel) {
            return Err(PrepareError::IncompatibleProfile);
        }
        let cold = cold_address as *mut LivePsramWorkspace;
        let metrics = metric_address as *mut LiveMetricPlan;
        crate::runtime::set_startup_stage(6);
        CurrentThreadHandle::get().delay(Duration::from_millis(50));
        // SAFETY: checked non-overlapping aligned ranges, exclusively owned by
        // this worker. Both templates are valid and contain no owned allocation.
        unsafe {
            core::ptr::copy_nonoverlapping(&PSRAM_TEMPLATE, cold, 1);
            core::ptr::copy_nonoverlapping(&METRIC_TEMPLATE, metrics, 1);
        }
        crate::runtime::set_startup_stage(7);
        CurrentThreadHandle::get().delay(Duration::from_millis(50));
        let metrics = unsafe { &mut *metrics };
        METRICS
            .compile_live_into(LiveMetricLimits::default(), metrics)
            .map_err(PrepareError::InvalidDefinition)?;
        crate::runtime::set_startup_stage(8);
        CurrentThreadHandle::get().delay(Duration::from_millis(50));
        let plan = TrajectoryEngine::live(LiveSpec {
            session_id: SessionId::from_bytes(*b"AEVIA-DEVICE-001"),
            engine: unsafe { profile::engine() },
            metrics,
            resources: LiveResourceLimits::V2_MINI_RTS,
            initial_heading: None,
            initial_clock_prior: InitialClockConsiderPrior {
                model: ClockModelId::new(1),
                segment: ClockSegmentId::new(1),
                reference_time: SessionTime::ZERO,
                // Shared capture clock: initialization cross-stamp margin and
                // unqualified crystal-rate prior. Per-sensor delay is separate.
                offset_variance_s2: Variance::new(5.0e-6 * 5.0e-6).unwrap(),
                drift_variance: Variance::new(500.0e-6 * 500.0e-6).unwrap(),
                offset_drift_covariance_s: FiniteF64::new(0.0).unwrap(),
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
        crate::runtime::set_startup_stage(9);
        CurrentThreadHandle::get().delay(Duration::from_millis(50));
        let mut session = plan.start(workspace)?;
        // Hardware capture removes wakeup jitter. LPF group-delay calibration
        // and fractional-delay resampling remain unqualified; preserve their
        // nonzero timing uncertainty rather than claiming perfect support.
        session
            .use_development_imu_timing_approximation()
            .map_err(|_| PrepareError::IncompatibleProfile)?;
        crate::runtime::set_startup_stage(10);
        Ok(Self {
            session,
            imu_sequence: 0,
            gnss_sequence: 0,
            have_initialization_seed: false,
            // These immutable interval-average uncertainties belong to the
            // profile. Validate once, rather than repeating software-f64
            // covariance construction for every high-rate IMU sample.
            imu_rate_covariance: diagonal([profile::IMU_RATE_ONE_SIGMA_RPS; 3])
                .map_err(PrepareError::InvalidDefinition)?,
            imu_acceleration_covariance: diagonal([profile::IMU_ACCELERATION_ONE_SIGMA_MPS2; 3])
                .map_err(PrepareError::InvalidDefinition)?,
            last_gnss_epoch_ms: None,
            snapshot: Snapshot::default(),
            rate_epoch_us: 0,
            rate_imu_samples: 0,
            rate_gnss_samples: 0,
        })
    }

    fn imu(&mut self, sample: ImuSample) -> Result<(), StepError> {
        self.imu_sequence += 1;
        self.snapshot.imu_received += 1;
        self.snapshot.last_imu_us = sample.captured_ns / 1000;
        self.snapshot.acceleration_mps2 = sample.acceleration_mps2.map(|x| x as f32);
        self.snapshot.angular_rate_rps = sample.angular_rate_rps.map(|x| x as f32);
        // Inertial data alone cannot initialize terrestrial navigation. Drain
        // and report every raw sample while waiting for an accepted position/
        // velocity seed; only subsequent real intervals enter the initializer.
        // This does not synthesize samples or claim estimator acceptance.
        if !self.have_initialization_seed {
            return Ok(());
        }
        let time = ObservationTime {
            support: SampleSupport::IntervalAverage {
                duration: DurationNs::from_ns(u64::from(sample.duration_ns)),
            },
            // IntervalAverage is end-stamped. Do not also subtract half the
            // averaging window: the engine already interprets its full support.
            correction: SignedDurationNs::from_ns(
                i64::try_from(sample.end_ns).map_err(|_| StepError::ClockDiscontinuity)?
                    - i64::try_from(sample.captured_ns)
                        .map_err(|_| StepError::ClockDiscontinuity)?,
            ),
            ..observation_time(
                sample.captured_ns,
                super::imu::DELAY_UNCERTAINTY_NS,
                TimingBasis::SensorCounterAnchored,
            )?
        };
        let axes = AxisStatus::new([true; 3], [sample.saturated; 3]);
        let observation = ImuObservation::new(
            ObservationId::new(SourceId::new(1), self.imu_sequence),
            FrameId::new(20),
            InputProfileId::new(1),
            TimedAngularRate {
                value: SensorAngularRate::from_components(sample.angular_rate_rps)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(self.imu_rate_covariance),
                axes,
            },
            TimedSpecificForce {
                value: SensorSpecificForce::from_components(sample.acceleration_mps2)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(self.imu_acceleration_covariance),
                axes,
            },
            ImuStatus::Degraded,
        )
        .map_err(StepError::InvalidObservation)?;
        self.step(Some(&LiveObservation::Imu(observation)))
    }

    fn gnss(&mut self, sample: GnssSample) -> Result<(), StepError> {
        self.snapshot.gnss_received += 1;
        self.snapshot.last_gnss_us = sample.received_us;
        self.snapshot.satellites = sample.satellites;
        if self.last_gnss_epoch_ms.is_some_and(|(generation, last)| {
            generation == sample.clock_generation && sample.epoch_ms <= last
        }) {
            return Ok(());
        }
        self.last_gnss_epoch_ms = Some((sample.clock_generation, sample.epoch_ms));
        self.gnss_sequence += 1;
        let observation =
            super::gnss::observation::observation(sample, self.gnss_sequence, profile::FRAME.id())?;
        self.step(Some(&LiveObservation::GnssSolution(observation)))
    }

    fn step(&mut self, observation: Option<&LiveObservation>) -> Result<(), StepError> {
        // Before an initialization seed there is no live frontier/metric work.
        // Raw acquisition may remain active for a long antenna-free cold start.
        if observation.is_none() && !self.have_initialization_seed {
            return Ok(());
        }
        let update = self.session.step(LiveStep {
            observation,
            work: WorkQuota::new(128).unwrap(),
        })?;
        if let (Some(LiveObservation::GnssSolution(solution)), Some((_, disposition))) =
            (observation, update.input)
            && super::gnss::observation::initialization_seed_accepted(*solution, disposition)
        {
            self.have_initialization_seed = true;
        }
        Ok(())
    }

    fn record_error(&mut self, error: StepError) {
        self.snapshot.processing_errors = self.snapshot.processing_errors.saturating_add(1);
        self.snapshot.error_kind = Some(step_error(error));
    }

    fn publish(&mut self, now: u64) {
        let diagnostics = self.session.diagnostics();
        self.snapshot.updated_us = now;
        self.snapshot.imu_samples = sensors::IMU_SAMPLES.load(Ordering::Relaxed);
        self.snapshot.gnss_samples = sensors::GNSS_SAMPLES.load(Ordering::Relaxed);
        self.snapshot.imu_errors = sensors::IMU_ERRORS.load(Ordering::Relaxed);
        self.snapshot.gnss_errors = sensors::GNSS_ERRORS.load(Ordering::Relaxed);
        self.snapshot.gnss_uart_errors = sensors::GNSS_UART_ERRORS.load(Ordering::Relaxed);
        self.snapshot.gnss_parse_errors = sensors::GNSS_PARSE_ERRORS.load(Ordering::Relaxed);
        let elapsed = now.saturating_sub(self.rate_epoch_us);
        if elapsed >= 1_000_000 {
            self.snapshot.imu_rate_hz = self
                .snapshot
                .imu_samples
                .wrapping_sub(self.rate_imu_samples) as f32
                * 1_000_000.0
                / elapsed as f32;
            self.snapshot.gnss_rate_hz =
                self.snapshot
                    .gnss_samples
                    .wrapping_sub(self.rate_gnss_samples) as f32
                    * 1_000_000.0
                    / elapsed as f32;
            self.rate_epoch_us = now;
            self.rate_imu_samples = self.snapshot.imu_samples;
            self.rate_gnss_samples = self.snapshot.gnss_samples;
        }
        self.snapshot.phase = match self.session.phase() {
            LivePhase::Initializing => Phase::Initializing,
            LivePhase::Navigating => Phase::Navigating,
            _ => Phase::Degraded,
        };
        let imu_stale = now.saturating_sub(self.snapshot.last_imu_us) > 100_000;
        let gnss_stale = now.saturating_sub(self.snapshot.last_gnss_us) > 1_000_000;
        if imu_stale || gnss_stale || !timing::status().pps_locked {
            self.snapshot.phase = Phase::Degraded;
        }
        self.snapshot.imu_accepted = diagnostics.imu_epochs_accepted;
        self.snapshot.gnss_fused = diagnostics.gnss_updates_fused;
        self.snapshot.input_rejected =
            diagnostics.imu_epochs_rejected + diagnostics.gnss_updates_rejected;
        self.snapshot.too_late = diagnostics.observations_too_late;
        let trajectory = self.session.trajectory();
        let epoch = trajectory.span().map(|span| span.end());
        // Clear unavailable fields each publication; never present old state as current.
        self.snapshot.epoch_us = epoch.and_then(|time| u64::try_from(time.as_ns() / 1000).ok());
        self.snapshot.horizontal_speed_mps = epoch
            .and_then(|time| {
                trajectory
                    .horizontal_speed_at(time, ReferencePointId::new(1))
                    .ok()
            })
            .map(|x| x as f32);
        self.snapshot.vertical_speed_mps = epoch
            .and_then(|time| {
                trajectory
                    .vertical_speed_at(time, ReferencePointId::new(1))
                    .ok()
            })
            .map(|x| x as f32);
        let state = epoch.and_then(|time| trajectory.state_at(time, ReferencePointId::new(1)).ok());
        self.snapshot.position_ecef_m = state.map(|value| value.position.components());
        self.snapshot.velocity_ecef_mps =
            state.map(|value| value.velocity.components().map(|x| x as f32));
        if imu_stale {
            self.snapshot.horizontal_speed_mps = None;
            self.snapshot.vertical_speed_mps = None;
            self.snapshot.position_ecef_m = None;
            self.snapshot.velocity_ecef_mps = None;
        }
        // Apply the same epoch check at publication and at transport reads.
        // A live worker can retain an old trajectory after rejected inputs.
        interface::publish(self.snapshot.at(now));
    }
}

/// Run on a dedicated core. `psram_address..+psram_len` is reserved exclusively
/// by runtime before launching this worker; no allocator may reuse this region.
///
/// # Safety
/// The address must identify initialized PSRAM that lives for the whole program.
pub unsafe fn run(psram_address: usize, psram_len: usize) -> ! {
    let mut processor = match unsafe { Processor::start(psram_address as *mut u8, psram_len) } {
        Ok(value) => value,
        Err(error) => {
            interface::publish(Snapshot {
                phase: Phase::Failed,
                updated_us: Instant::now().as_micros(),
                processing_errors: 1,
                error_kind: Some(prepare_error(error)),
                ..Snapshot::default()
            });
            loop {
                CurrentThreadHandle::get().delay(Duration::from_millis(1000));
            }
        }
    };
    super::supervision::worker_heartbeat();
    let mut published_us = 0;
    loop {
        // Separate queues and one member from each per turn prevent either
        // source from starving the other. Work never holds the snapshot lock.
        let started = Instant::now().as_micros();
        let mut active = false;
        if let Ok(sample) = sensors::GNSS.try_receive() {
            active = true;
            if let Err(error) = processor.gnss(sample) {
                processor.record_error(error);
            }
        }
        if let Ok(sample) = sensors::IMU.try_receive() {
            active = true;
            if let Err(error) = processor.imu(sample) {
                processor.record_error(error);
            }
        }
        let now = Instant::now().as_micros();
        if now.saturating_sub(published_us) >= profile::SNAPSHOT_INTERVAL_US {
            processor.publish(now);
            published_us = now;
        }
        if !active && let Err(error) = processor.step(None) {
            processor.record_error(error);
        }
        // Include pending estimator work and telemetry queries, excluding sleep.
        processor.snapshot.maximum_step_us = processor
            .snapshot
            .maximum_step_us
            .max((Instant::now().as_micros() - started).min(u64::from(u32::MAX)) as u32);
        super::supervision::worker_heartbeat();
        if !active {
            // At most 2 kHz idle wakeups bound shared RTOS timer/critical-section
            // traffic. 500 us remains below the earliest DEC4 sample period;
            // hardware capture, rather than this consumer poll, timestamps it.
            CurrentThreadHandle::get().delay(Duration::from_micros(500));
        }
    }
}

fn observation_time(
    nanos: u64,
    sigma_ns: u64,
    basis: TimingBasis,
) -> Result<ObservationTime, StepError> {
    let nanos = i64::try_from(nanos)
        .ok()
        .ok_or(StepError::InvalidObservation(
            aevia_trajectory::ValidationError::TimeOverflow,
        ))?;
    Ok(ObservationTime {
        registered_at: SessionTime::from_ns(nanos),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::from_ns(sigma_ns),
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis,
    })
}

fn diagonal(sigma: [f64; 3]) -> Result<Covariance3, aevia_trajectory::ValidationError> {
    Covariance3::diagonal(
        sigma[0] * sigma[0],
        sigma[1] * sigma[1],
        sigma[2] * sigma[2],
    )
}

fn prepare_error(error: PrepareError) -> EngineError {
    match error {
        PrepareError::InvalidDefinition(aevia_trajectory::ValidationError::CapacityExceeded)
        | PrepareError::InsufficientResources => EngineError::Resources,
        PrepareError::InvalidDefinition(_) => EngineError::Definition,
        PrepareError::IncompatibleProfile | PrepareError::PlatformUnsupported => {
            EngineError::Profile
        }
        PrepareError::FrameUnresolved => EngineError::Frame,
        PrepareError::CalibrationUnavailable => EngineError::Calibration,
        PrepareError::CapabilityUnavailable => EngineError::Capability,
        PrepareError::InvalidWorkspaceAlignment => EngineError::Workspace,
        PrepareError::UnqualifiedProfile => EngineError::Qualification,
        PrepareError::EvidenceUnavailable | PrepareError::ReplayContractUnavailable => {
            EngineError::Evidence
        }
        PrepareError::NotRestartable => EngineError::Lifecycle,
    }
}

fn step_error(error: StepError) -> EngineError {
    match error {
        StepError::DuplicateObservation { .. } | StepError::NonMonotonicSequence { .. } => {
            EngineError::Sequence
        }
        StepError::InvalidObservation(aevia_trajectory::ValidationError::CapacityExceeded)
        | StepError::OutputCapacityExceeded => EngineError::OutputCapacity,
        StepError::InvalidObservation(_) => EngineError::Observation,
        StepError::FrameMismatch => EngineError::Frame,
        StepError::ClockDiscontinuity => EngineError::Clock,
        StepError::WorkspaceContract => EngineError::Workspace,
        StepError::EstimatorFailure => EngineError::Estimator,
        StepError::MetricFailure => EngineError::Metric,
        StepError::AlreadyFinishing | StepError::Finished => EngineError::Lifecycle,
    }
}
