//! Bounded Traj live navigation. CPU1 owns all floating-point estimator state.
mod definitions;
mod initialization;
mod measurements;
use measurements::prepare_gnss;
pub mod memory;
mod profile;
pub mod settings;

use aevia_trajectory::{
    LiveSession, PrepareError, StepError, TrajectoryEngine,
    config::{ClockSharedCrossCovariance, InitialClockConsiderPrior, LiveResourceLimits, LiveSpec},
    engine::LivePhase,
    frame::{EcefPosition, EcefVelocity, SensorAngularRate, SensorSpecificForce},
    ids::*,
    math::FiniteF64,
    metric::*,
    observation::*,
    quality::{EstimateStage, Validity},
    time::{
        DurationNs, ObservationTime, SampleSupport, SessionTime, SignedDurationNs, TimingBasis,
    },
    uncertainty::{Covariance3, MeasurementUncertainty, Variance},
    workspace::{LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion},
};
use core::{
    cell::RefCell,
    mem::{align_of, size_of},
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use heapless::Deque;
use initialization::InitializationGate;
use serde::Serialize;

#[derive(Clone, Copy, Serialize)]
pub struct DragResult {
    pub target: u32,
    pub seconds: f64,
    pub stage: &'static str,
    pub validity: &'static str,
}
#[derive(Clone, Copy, Serialize)]
pub struct Status {
    pub phase: &'static str,
    pub qualified: bool,
    pub metrics: &'static str,
    pub timing: &'static str,
    pub error: Option<&'static str>,
    pub updated_us: u64,
    pub epoch_us: Option<u64>,
    pub speed_mps: Option<f64>,
    pub position_ecef_m: Option<[f64; 3]>,
    pub imu_accepted: u64,
    pub gnss_fused: u64,
    pub rejected: u64,
    pub queue_dropped: u32,
    pub maximum_step_us: u32,
    pub errors: u32,
    pub drag: [Option<DragResult>; 2],
}
const INITIAL: Status = Status {
    phase: "starting",
    qualified: false,
    metrics: "unavailable-unqualified-root-profile",
    timing: "unqualified",
    error: None,
    updated_us: 0,
    epoch_us: None,
    speed_mps: None,
    position_ecef_m: None,
    imu_accepted: 0,
    gnss_fused: 0,
    rejected: 0,
    queue_dropped: 0,
    maximum_step_us: 0,
    errors: 0,
    drag: [None; 2],
};
struct Shared {
    imu: Deque<crate::imu::ImuSample, 256>,
    gnss: Deque<(crate::gnss::OwnedBestNav, u64), 32>,
    status: Status,
}
static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> = Mutex::new(RefCell::new(Shared {
    imu: Deque::new(),
    gnss: Deque::new(),
    status: INITIAL,
}));
pub fn snapshot() -> Status {
    SHARED.lock(|cell| {
        let mut s = cell.borrow().status;
        let now = embassy_time::Instant::now().as_micros();
        if s.epoch_us
            .is_some_and(|epoch| now.saturating_sub(epoch) > 1_000_000)
        {
            s.speed_mps = None;
            s.position_ecef_m = None;
            s.phase = "stale";
        }
        s
    })
}
pub fn capture_imu(sample: crate::imu::ImuSample) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.imu.push_back(sample).is_err() {
            s.status.queue_dropped = s.status.queue_dropped.saturating_add(1);
        }
    });
}
pub fn capture_gnss(sample: crate::gnss::OwnedBestNav, received_us: u64) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.gnss.push_back((sample, received_us)).is_err() {
            s.status.queue_dropped = s.status.queue_dropped.saturating_add(1);
        }
    });
}
fn update(f: impl FnOnce(&mut Status)) {
    SHARED.lock(|s| f(&mut s.borrow_mut().status));
}

static PSRAM_TEMPLATE: LivePsramWorkspace = LivePsramWorkspace::new(profile::FRAME);
static METRIC_TEMPLATE: LiveMetricPlan = LiveMetricPlan::placeholder();
static DEFINITIONS_TEMPLATE: MetricPlan = MetricPlan::new(1);
static mut INTERNAL: LiveInternalWorkspace = LiveInternalWorkspace::new();
static mut PROCESSOR: Option<Processor> = None;
static READY: AtomicBool = AtomicBool::new(false);
static RESET: AtomicBool = AtomicBool::new(false);
/// Rearm live metrics at the start of each independently recorded session.
pub fn request_reset() {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu.clear();
        s.gnss.clear();
        s.status = INITIAL;
        s.status.phase = "reset-requested";
    });
    RESET.store(true, Ordering::Release);
}
struct Processor {
    session: LiveSession<'static, 'static>,
    imu_seq: u64,
    gnss_seq: u64,
    last_gps: Option<u64>,
    // Chosen at the first valid fix; never silently change latency models.
    gnss_clock: Option<Option<u32>>,
    gate: InitializationGate,
    gyro_cov: Covariance3,
    accel_cov: Covariance3,
    last_publish: u64,
    last_imu: u64,
    result_ids: [Option<LiveResultId>; 2],
}

/// Called once on CPU1. Memory bring-up tests the full fitted range, reserves a
/// disjoint tail for CPU0 preview, then gives this processor only its own arena.
pub fn initialize(psram: esp_hal::psram::Psram) {
    if esp_hal::system::Cpu::current() != esp_hal::system::Cpu::AppCpu {
        update(|s| {
            s.phase = "failed";
            s.error = Some("wrong-cpu");
        });
        return;
    }
    match memory::bring_up(psram) {
        Ok(len) => {
            READY.store(true, Ordering::Release);
            match unsafe { Processor::start(memory::BASE, len) } {
                Ok(p) => {
                    unsafe {
                        *core::ptr::addr_of_mut!(PROCESSOR) = Some(p);
                    };
                    update(|s| s.phase = "waiting-for-fix");
                }
                Err(e) => update(|s| {
                    s.phase = "failed";
                    s.error = Some(prepare_error(e));
                }),
            }
        }
        Err(_) => update(|s| {
            s.phase = "failed";
            s.error = Some("psram-validation");
        }),
    }
}

/// Bounded service point called by the sole CPU1 storage owner between I/O.
pub fn tick() {
    if cfg!(feature = "diagnostic-no-navigation") {
        return;
    }
    if READY.load(Ordering::Acquire) && RESET.swap(false, Ordering::AcqRel) {
        // This CPU owns every workspace reference. Drop the old handle before
        // reconstructing the cold state from immutable templates.
        unsafe {
            *core::ptr::addr_of_mut!(PROCESSOR) = None;
        }
        match unsafe { Processor::start(memory::BASE, memory::NAVIGATION_BYTES) } {
            Ok(p) => {
                unsafe {
                    *core::ptr::addr_of_mut!(PROCESSOR) = Some(p);
                };
                update(|s| s.phase = "waiting-for-fix");
            }
            Err(e) => update(|s| {
                s.phase = "failed";
                s.error = Some(prepare_error(e));
            }),
        }
    }
    let Some(p) = (unsafe { &mut *core::ptr::addr_of_mut!(PROCESSOR) }).as_mut() else {
        return;
    };
    let started = embassy_time::Instant::now().as_micros();
    let (gnss, imu) = SHARED.lock(|cell| {
        let mut s = cell.borrow_mut();
        (s.gnss.pop_front(), s.imu.pop_front())
    });
    let mut active = false;
    if let Some((g, t)) = gnss {
        active = true;
        if let Err(e) = p.gnss(g.as_nav(), t) {
            p.error(e);
        }
    }
    if let Some(i) = imu {
        active = true;
        if let Err(e) = p.imu(i) {
            p.error(e);
        }
    }
    if !active
        && !matches!(p.session.phase(), LivePhase::Initializing)
        && let Err(e) = p.step(None)
    {
        p.error(e);
    }
    let now = embassy_time::Instant::now().as_micros();
    if now.saturating_sub(p.last_publish) >= profile::SNAPSHOT_INTERVAL_US {
        p.publish(now);
        p.last_publish = now;
    }
    update(|s| {
        s.maximum_step_us = s.maximum_step_us.max(
            (embassy_time::Instant::now().as_micros() - started).min(u64::from(u32::MAX)) as u32,
        )
    });
}

impl Processor {
    unsafe fn start(address: usize, len: usize) -> Result<Self, PrepareError> {
        let cold_address = (address + align_of::<LivePsramWorkspace>() - 1)
            & !(align_of::<LivePsramWorkspace>() - 1);
        let metric_address =
            (cold_address + size_of::<LivePsramWorkspace>() + align_of::<LiveMetricPlan>() - 1)
                & !(align_of::<LiveMetricPlan>() - 1);
        let definitions_address =
            (metric_address + size_of::<LiveMetricPlan>() + align_of::<MetricPlan>() - 1)
                & !(align_of::<MetricPlan>() - 1);
        if definitions_address + size_of::<MetricPlan>() > address + len {
            return Err(PrepareError::InsufficientResources);
        }
        let cold = cold_address as *mut LivePsramWorkspace;
        let metrics = metric_address as *mut LiveMetricPlan;
        let definitions = definitions_address as *mut MetricPlan;
        unsafe {
            core::ptr::copy_nonoverlapping(&PSRAM_TEMPLATE, cold, 1);
            core::ptr::copy_nonoverlapping(&METRIC_TEMPLATE, metrics, 1);
            core::ptr::copy_nonoverlapping(&DEFINITIONS_TEMPLATE, definitions, 1);
        }
        let definitions = unsafe { &mut *definitions };
        definitions::populate(definitions).map_err(PrepareError::InvalidDefinition)?;
        let metrics = unsafe { &mut *metrics };
        definitions
            .compile_live_into(LiveMetricLimits::default(), metrics)
            .map_err(PrepareError::InvalidDefinition)?;
        let builder = TrajectoryEngine::live(LiveSpec {
            session_id: SessionId::from_bytes(*b"AEVIA-DEVICE-001"),
            engine: unsafe { profile::engine() },
            metrics,
            resources: LiveResourceLimits::V2_MINI_RTS,
            initial_heading: None,
            initial_clock_prior: InitialClockConsiderPrior {
                model: ClockModelId::new(1),
                segment: ClockSegmentId::new(1),
                reference_time: SessionTime::ZERO,
                offset_variance_s2: Variance::new(0.025 * 0.025).unwrap(),
                drift_variance: Variance::new(500e-6 * 500e-6).unwrap(),
                offset_drift_covariance_s: FiniteF64::new(0.0).unwrap(),
                cross_covariance_with_shared: ClockSharedCrossCovariance::independent(6).unwrap(),
            },
        });
        #[cfg(feature = "development-navigation")]
        let plan = builder.preflight_development()?;
        #[cfg(not(feature = "development-navigation"))]
        let plan = builder.preflight()?;
        let workspace = unsafe {
            LiveWorkspace::bind(
                &mut *core::ptr::addr_of_mut!(INTERNAL),
                MemoryRegion::InternalSram,
                &mut *cold,
                MemoryRegion::Psram,
            )
        };
        let session = plan.start(workspace)?;
        #[cfg(feature = "development-navigation")]
        let mut session = session;
        #[cfg(feature = "development-navigation")]
        session
            .use_development_imu_timing_approximation()
            .map_err(|_| PrepareError::IncompatibleProfile)?;
        Ok(Self {
            session,
            imu_seq: 0,
            gnss_seq: 0,
            last_gps: None,
            gnss_clock: None,
            gate: InitializationGate::default(),
            gyro_cov: diagonal(profile::IMU_RATE_ONE_SIGMA_RPS).unwrap(),
            accel_cov: diagonal(profile::IMU_ACCELERATION_ONE_SIGMA_MPS2).unwrap(),
            last_publish: 0,
            last_imu: 0,
            result_ids: [None; 2],
        })
    }
    fn imu(&mut self, sample: crate::imu::ImuSample) -> Result<(), StepError> {
        self.imu_seq += 1;
        self.last_imu = sample.monotonic_us;
        let end = sample.effective_end_ns;
        let support = end.saturating_sub(u64::from(sample.interval_ns));
        let initializing = matches!(self.session.phase(), LivePhase::Initializing);
        if !self
            .gate
            .permits_imu(initializing, support, profile::MAX_GNSS_AGE_NS)
        {
            return Ok(());
        }
        let time = ObservationTime {
            support: SampleSupport::IntervalAverage {
                duration: DurationNs::from_ns(u64::from(sample.interval_ns)),
            },
            correction: SignedDurationNs::from_ns(
                i64::try_from(end).map_err(|_| StepError::ClockDiscontinuity)?
                    - i64::try_from(sample.captured_ns)
                        .map_err(|_| StepError::ClockDiscontinuity)?,
            ),
            ..time(
                sample.captured_ns,
                sample.timing_uncertainty_ns,
                TimingBasis::SensorCounterAnchored,
            )?
        };
        let axes = AxisStatus::new([true; 3], [false; 3]);
        let obs = ImuObservation::new(
            ObservationId::new(SourceId::new(1), self.imu_seq),
            FrameId::new(20),
            InputProfileId::new(1),
            TimedAngularRate {
                value: SensorAngularRate::from_components(sample.aligned_angular_rate_rps)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(self.gyro_cov),
                axes,
            },
            TimedSpecificForce {
                value: SensorSpecificForce::from_components(sample.acceleration_mps2)
                    .map_err(StepError::InvalidObservation)?,
                time,
                uncertainty: MeasurementUncertainty::Provided(self.accel_cov),
                axes,
            },
            ImuStatus::Degraded,
        )
        .map_err(StepError::InvalidObservation)?;
        let result = self.step(Some(&LiveObservation::Imu(obs)))?;
        self.gate
            .observe_imu_result(initializing, support, profile::MAX_GNSS_AGE_NS, result);
        Ok(())
    }
    fn gnss(&mut self, nav: crate::gnss::BestNav<'_>, received_us: u64) -> Result<(), StepError> {
        let epoch = u64::from(nav.gps_week) * 604_800_000 + u64::from(nav.gps_tow_ms);
        if self.last_gps.is_some_and(|last| epoch <= last) {
            return Ok(());
        }
        self.last_gps = Some(epoch);
        self.gnss_seq += 1;
        let received_ns = received_us.saturating_mul(1000);
        let mapped = crate::timing::map_epoch(nav.gps_week, nav.gps_tow_ms, received_ns);
        let selected = *self.gnss_clock.get_or_insert(mapped.map(|e| e.generation));
        let mapped = if let Some(generation) = selected {
            let Some(epoch) = mapped.filter(|epoch| epoch.generation == generation) else {
                update(|s| s.timing = "pps-unavailable-rearm-required");
                return Ok(());
            };
            Some(epoch)
        } else {
            None
        };
        let (epoch_ns, sigma, basis) = if let Some(epoch) = mapped {
            (
                epoch.time_ns,
                epoch.uncertainty_ns,
                TimingBasis::PpsCorrelated,
            )
        } else {
            (received_ns, 25_000_000, TimingBasis::ArrivalOnly)
        };
        update(|s| {
            s.timing = if mapped.is_some() {
                "pps-with-uncalibrated-sensor-delay"
            } else {
                "arrival-only-development"
            }
        });
        let position_time = time(epoch_ns, sigma, basis)?;
        let velocity_time = time(
            epoch_ns.saturating_sub(u64::from(nav.latency_ms) * 1_000_000),
            sigma,
            basis,
        )?;
        let (position, velocity, position_cov, velocity_cov) =
            prepare_gnss(nav).map_err(StepError::InvalidObservation)?;
        let obs = GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), self.gnss_seq),
            ReferencePointId::new(2),
            Some(GnssPosition {
                value: EcefPosition::from_components(position)
                    .map_err(StepError::InvalidObservation)?,
                time: position_time,
                frame: profile::FRAME.id(),
                valid: true,
                uncertainty: MeasurementUncertainty::Provided(position_cov),
            }),
            Some(GnssVelocity {
                value: EcefVelocity::from_components(velocity)
                    .map_err(StepError::InvalidObservation)?,
                time: velocity_time,
                frame: profile::FRAME.id(),
                valid: true,
                uncertainty: MeasurementUncertainty::Provided(velocity_cov),
            }),
            None,
            GnssDiagnostics {
                correction_age: None,
                solution_age: None,
                health: Some(TimedDiagnostic {
                    value: ReceiverHealth::Healthy,
                    time: velocity_time,
                    age: DurationNs::ZERO,
                }),
            },
        )
        .map_err(StepError::InvalidObservation)?;
        let result = self.step(Some(&LiveObservation::GnssSolution(obs)))?;
        if matches!(
            result,
            Some(
                InputDisposition::InitializationOnly
                    | InputDisposition::QueuedForFusion
                    | InputDisposition::Fused
                    | InputDisposition::Downweighted
            )
        ) {
            self.gate.accepted_seed(
                epoch_ns,
                epoch_ns.saturating_sub(u64::from(nav.latency_ms) * 1_000_000),
            );
        }
        Ok(())
    }
    fn step(
        &mut self,
        obs: Option<&LiveObservation>,
    ) -> Result<Option<InputDisposition>, StepError> {
        let update = self.session.step(LiveStep {
            observation: obs,
            work: WorkQuota::new(64).unwrap(),
        })?;
        for mutation in update.mutations {
            match *mutation {
                MetricMutation::Upsert {
                    id,
                    value: MetricResultValue::DragTarget(report),
                    ..
                } => {
                    let index = if report.target == TargetId::new(1) {
                        0
                    } else {
                        1
                    };
                    self.result_ids[index] = Some(id);
                    let stage = stage(report.stage);
                    let validity = match report.validity {
                        Validity::Nominal => "unqualified",
                        Validity::Degraded => "degraded",
                        Validity::Invalid => "invalid",
                    };
                    update_status_drag(
                        index,
                        Some(DragResult {
                            target: index as u32 + 1,
                            seconds: report.elapsed_seconds,
                            stage,
                            validity,
                        }),
                    );
                }
                MetricMutation::Withdraw { id, .. } => {
                    for (i, present) in self.result_ids.iter_mut().enumerate() {
                        if *present == Some(id) {
                            *present = None;
                            update_status_drag(i, None);
                        }
                    }
                }
                MetricMutation::Finalize { id, .. } => {
                    for (i, present) in self.result_ids.iter().enumerate() {
                        if *present == Some(id) {
                            update_status_finalize(i);
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(update.input.map(|(_, d)| d))
    }
    fn error(&mut self, error: StepError) {
        update(|s| {
            s.errors = s.errors.saturating_add(1);
            s.error = Some(match error {
                StepError::MetricFailure => "metric",
                StepError::ClockDiscontinuity => "clock",
                StepError::EstimatorFailure => "estimator",
                StepError::OutputCapacityExceeded => "capacity",
                _ => "observation",
            });
        });
    }
    fn publish(&self, now: u64) {
        let d = self.session.diagnostics();
        let trajectory = self.session.trajectory();
        let epoch = trajectory.span().map(|s| s.end());
        let fresh = now.saturating_sub(self.last_imu) < 100_000;
        let speed = if fresh {
            epoch.and_then(|t| {
                trajectory
                    .horizontal_speed_at(t, ReferencePointId::new(1))
                    .ok()
            })
        } else {
            None
        };
        let position = if fresh {
            epoch
                .and_then(|t| trajectory.state_at(t, ReferencePointId::new(1)).ok())
                .map(|s| s.position.components())
        } else {
            None
        };
        update(|s| {
            s.updated_us = now;
            s.epoch_us = epoch.and_then(|t| u64::try_from(t.as_ns() / 1000).ok());
            s.speed_mps = speed;
            s.position_ecef_m = position;
            s.imu_accepted = d.imu_epochs_accepted;
            s.gnss_fused = d.gnss_updates_fused;
            s.rejected = d.imu_epochs_rejected + d.gnss_updates_rejected;
            s.phase = match self.session.phase() {
                LivePhase::Initializing => "waiting-for-fix",
                LivePhase::Navigating => "development-navigation",
                _ => "degraded",
            };
        });
    }
}
fn update_status_drag(index: usize, value: Option<DragResult>) {
    update(|s| s.drag[index] = value);
}
fn update_status_finalize(index: usize) {
    update(|s| {
        if let Some(r) = &mut s.drag[index] {
            r.stage = "finalized";
        }
    });
}
fn stage(value: EstimateStage) -> &'static str {
    match value {
        EstimateStage::Predicted => "predicted",
        EstimateStage::Provisional => "provisional",
        EstimateStage::Finalized => "finalized",
    }
}
fn diagonal(sigma: f64) -> Result<Covariance3, aevia_trajectory::ValidationError> {
    Covariance3::diagonal(sigma * sigma, sigma * sigma, sigma * sigma)
}
fn time(ns: u64, sigma: u64, basis: TimingBasis) -> Result<ObservationTime, StepError> {
    Ok(ObservationTime {
        registered_at: SessionTime::from_ns(
            i64::try_from(ns).map_err(|_| StepError::ClockDiscontinuity)?,
        ),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::from_ns(sigma),
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis,
    })
}
fn prepare_error(e: PrepareError) -> &'static str {
    match e {
        PrepareError::UnqualifiedProfile => "production-profile-unqualified",
        PrepareError::InsufficientResources => "workspace-resources",
        PrepareError::CapabilityUnavailable => "capability",
        _ => "profile-preflight",
    }
}
