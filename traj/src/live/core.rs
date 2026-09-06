//! Cohesive orchestration for the embedded delayed-filter path.
//!
//! The public engine converts semantic observations into these private values.
//! This layer deliberately owns no allocator, device, clock, or storage API.

#[cfg(test)]
use core::mem::{align_of, size_of};

use core::ops::{Deref, DerefMut};

use crate::{
    live::{
        DENSE_HISTORY_CAPACITY, IMU_HISTORY_CAPACITY, MAX_CONSIDER, MAX_HISTORY_HORIZON_NS,
        MEASUREMENT_QUEUE_CAPACITY,
        dense_history::{
            DenseCovariance, DenseEndpoint, DenseHistory, DenseHistoryError, DenseSegment,
        },
        eskf::{
            ConsiderCovariance, CovariancePolicy, Eskf, EskfError, EskfPropagationScratch,
            GapNavCrossCovariance, GnssObservation, GnssUpdateOutcome, NavConsiderCovariance,
            NisGate, ProcessNoise,
        },
        initializer::InitializationResult,
        predictor::{OutputPredictor, PredictorConfig, PredictorError, PredictorTrackingError},
        preintegration::{
            CompactCovariance3, GapModel, ImuInterval, ImuNoise, MAX_BATCH_SAMPLES,
            PreintegrationError, Preintegrator,
        },
        reanchor::ReanchorError,
        scheduler::{EnqueueDisposition, FixedRing, FrontierScheduler, QueueError, SchedulerError},
        state::MechanizationContext,
    },
    quality::{GnssState, TimingQuality},
    time::SessionTime,
};

use nalgebra::{ArrayStorage, Matrix3, Vector3};

mod frontier;
mod ingestion;
mod lifecycle;
mod output;
mod propagation;
mod smoothing;

#[cfg(test)]
mod tests;

/// The fastest supported navigation cadence is 400 Hz.  A slower cadence is
/// allowed down to 100 Hz; larger batches need a separately qualified profile.
pub(crate) const MIN_NAVIGATION_PERIOD_NS: i64 = 2_500_000;
pub(crate) const MAX_NAVIGATION_PERIOD_NS: i64 = 10_000_000;
/// One normalized IMU observation (excluding an explicit bridge) is bounded so
/// ingest work and the number of crossed navigation epochs remain finite.
pub(crate) const MAX_IMU_INTERVAL_NS: i64 = 10_000_000;
pub(crate) const MAX_PREDICTOR_SEGMENTS_PER_INGEST: usize = 10;

pub(crate) const IMU_SLICE_CREDITS: u16 = 1;
pub(crate) const FILTER_PROPAGATION_CREDITS: u16 = 8;
pub(crate) const GNSS_UPDATE_CREDITS: u16 = 12;
pub(crate) const SEGMENT_FINALIZATION_CREDITS: u16 = 1;
pub(crate) const FRONTIER_COMMIT_CREDITS: u16 = 1;

const MAX_PROPAGATION_SLICES: usize = MAX_BATCH_SAMPLES as usize;
const MIN_PROPAGATION_CREDITS: u16 = IMU_SLICE_CREDITS + FILTER_PROPAGATION_CREDITS;

/// Large, caller-owned buffers. Firmware may place this object in aligned
/// PSRAM; [`LiveCore`] itself contains the hot covariance and queue state.
#[derive(Debug, PartialEq)]
pub(crate) struct LiveCoreHistory {
    imu: FixedRing<ImuInterval, IMU_HISTORY_CAPACITY>,
    corrected: DenseHistory<DENSE_HISTORY_CAPACITY>,
    predicted: DenseHistory<DENSE_HISTORY_CAPACITY>,
    propagation_scratch: EskfPropagationScratch,
    /// Cross covariance with the one interval-average IMU sample whose
    /// support owns the corrected frontier. Kept in PSRAM because it is cold
    /// except at propagation and GNSS-update boundaries.
    active_imu_sample: Option<CorrectedImuSample>,
    active_imu_sample_nav_cross: GapNavCrossCovariance,
    predictor_staging: [Option<DenseSegment>; MAX_PREDICTOR_SEGMENTS_PER_INGEST],
    smoothing: super::rts_window::RtsWindow,
    smoothing_update: crate::live::eskf::RtsUpdateCapture,
    smoothing_update_transaction: crate::live::eskf::RtsUpdateCapture,
    corrected_quality:
        FixedRing<(Option<GnssQualityUpdate>, Option<GnssQualityUpdate>), DENSE_HISTORY_CAPACITY>,
    current_quality: Option<GnssQualityUpdate>,
    endpoint_quality: Option<GnssQualityUpdate>,
    published_frontier: Option<SessionTime>,
}

impl LiveCoreHistory {
    pub(crate) const fn new() -> Self {
        Self {
            imu: FixedRing::new(),
            corrected: DenseHistory::new(),
            predicted: DenseHistory::new(),
            propagation_scratch: EskfPropagationScratch::new(),
            active_imu_sample: None,
            active_imu_sample_nav_cross: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; super::state::NAV_DIM]; super::preintegration::BIAS_DIM],
            )),
            predictor_staging: [None; MAX_PREDICTOR_SEGMENTS_PER_INGEST],
            smoothing: super::rts_window::RtsWindow::new(),
            smoothing_update: crate::live::eskf::RtsUpdateCapture::new(),
            smoothing_update_transaction: crate::live::eskf::RtsUpdateCapture::new(),
            corrected_quality: FixedRing::new(),
            current_quality: None,
            endpoint_quality: None,
            published_frontier: None,
        }
    }

    pub(crate) const fn is_empty(&self) -> bool {
        self.imu.len() == 0
            && self.corrected.len() == 0
            && self.predicted.len() == 0
            && self.active_imu_sample.is_none()
            && self.smoothing.is_empty()
    }

    #[cfg(test)]
    pub(crate) const fn raw_imu_len(&self) -> usize {
        self.imu.len()
    }

    /// Releases a workspace for a later session without constructing or
    /// moving another large history value on the task stack.
    pub(crate) fn clear(&mut self) {
        while self.imu.pop_front().is_some() {}
        while self.corrected.pop_oldest().is_some() {}
        while self.predicted.pop_oldest().is_some() {}
        self.active_imu_sample = None;
        self.active_imu_sample_nav_cross.fill(0.0);
        self.predictor_staging.fill(None);
        self.smoothing.clear();
        self.smoothing_update.reset();
        self.smoothing_update_transaction.reset();
        while self.corrected_quality.pop_front().is_some() {}
        self.current_quality = None;
        self.endpoint_quality = None;
        self.published_frontier = None;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LiveCoreConfig {
    pub(crate) fusion_delay_ns: i64,
    pub(crate) smoothing_lag_ns: i64,
    pub(crate) navigation_period_ns: i64,
    pub(crate) bias_correction_validity_norm: f32,
    pub(crate) mechanization: MechanizationContext,
    pub(crate) imu_noise: ImuNoise,
    pub(crate) process_noise: ProcessNoise,
    pub(crate) covariance_policy: CovariancePolicy,
    pub(crate) nis_gate: NisGate,
    pub(crate) predictor: PredictorConfig,
    pub(crate) gap: GapModel,
}

impl LiveCoreConfig {
    pub(crate) fn validate(&self) -> Result<(), LiveCoreError> {
        if self.navigation_period_ns < MIN_NAVIGATION_PERIOD_NS
            || self.navigation_period_ns > MAX_NAVIGATION_PERIOD_NS
            || self.fusion_delay_ns < self.navigation_period_ns
            || self.fusion_delay_ns > MAX_HISTORY_HORIZON_NS as i64
            || !(0..=super::rts_window::MAX_SMOOTHING_LAG_NS).contains(&self.smoothing_lag_ns)
            || !self.bias_correction_validity_norm.is_finite()
            || self.bias_correction_validity_norm <= 0.0
            || !self.imu_noise.is_valid()
            || !self.process_noise.is_valid()
        {
            return Err(LiveCoreError::InvalidConfiguration);
        }
        FrontierScheduler::<GnssObservation, MEASUREMENT_QUEUE_CAPACITY>::validate_fusion_delay(
            self.fusion_delay_ns,
        )
        .map_err(LiveCoreError::Scheduler)?;
        self.nis_gate.validate().map_err(LiveCoreError::Eskf)?;
        self.predictor
            .validate()
            .map_err(LiveCoreError::Predictor)?;
        self.gap.validate().map_err(LiveCoreError::Preintegration)?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LiveCoreSeed<'a> {
    pub(crate) initialization: &'a InitializationResult,
    pub(crate) nav_consider_covariance: &'a NavConsiderCovariance,
    pub(crate) consider_covariance: &'a ConsiderCovariance,
    pub(crate) active_consider: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum LiveCoreInput {
    Imu(ImuInterval),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum IngestDisposition {
    ImuAccepted {
        stored_intervals: u8,
        predictor_segments: u8,
        gap_bridged: bool,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DrainBlock {
    QuotaExhausted,
    AwaitingDelayedFrontier,
    CorrectedHistoryFull,
    Finished,
}

/// One time-indexed GNSS quality transition whose measurement was actually
/// accepted by the ESKF. This deliberately excludes queueing, lateness, and
/// health/innovation rejection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GnssQualityUpdate {
    pub(crate) epoch: SessionTime,
    pub(crate) state: GnssState,
    pub(crate) timing: TimingQuality,
    pub(crate) downweighted: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DrainReport {
    pub(crate) imu_slices: u16,
    pub(crate) filter_propagations: u16,
    pub(crate) gnss_updates: u16,
    pub(crate) gnss_fused: u16,
    pub(crate) gnss_rejected: u16,
    pub(crate) gnss_downweighted: u16,
    pub(crate) finalized_segments: u16,
    pub(crate) frontier_commits: u16,
    pub(crate) smoothing_steps: u16,
    gnss_quality_updates: [Option<GnssQualityUpdate>; MEASUREMENT_QUEUE_CAPACITY],
    gnss_quality_update_count: usize,
    pub(crate) last_gnss_outcome: Option<GnssUpdateOutcome>,
    pub(crate) last_gnss_key: Option<super::scheduler::OrderKey>,
    pub(crate) blocked_on: DrainBlock,
}

impl DrainReport {
    pub(crate) const fn new() -> Self {
        Self {
            imu_slices: 0,
            filter_propagations: 0,
            gnss_updates: 0,
            gnss_fused: 0,
            gnss_rejected: 0,
            gnss_downweighted: 0,
            finalized_segments: 0,
            frontier_commits: 0,
            smoothing_steps: 0,
            gnss_quality_updates: [None; MEASUREMENT_QUEUE_CAPACITY],
            gnss_quality_update_count: 0,
            last_gnss_outcome: None,
            last_gnss_key: None,
            blocked_on: DrainBlock::AwaitingDelayedFrontier,
        }
    }

    pub(crate) fn gnss_quality_updates(&self) -> impl Iterator<Item = GnssQualityUpdate> + '_ {
        self.gnss_quality_updates
            .iter()
            .take(self.gnss_quality_update_count)
            .copied()
            .flatten()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct FinishReport {
    pub(crate) terminal_time: SessionTime,
    pub(crate) predictor_segment_flushed: bool,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct LiveCoreSizes {
    pub(crate) core_bytes: usize,
    pub(crate) core_alignment: usize,
    pub(crate) history_bytes: usize,
    pub(crate) history_alignment: usize,
    pub(crate) queued_measurement_bytes: usize,
    pub(crate) imu_history_bytes: usize,
    pub(crate) dense_history_bytes_each: usize,
}

#[cfg(test)]
impl LiveCoreSizes {
    pub(crate) const fn compiled() -> Self {
        Self {
            core_bytes: size_of::<LiveCoreState>(),
            core_alignment: align_of::<LiveCoreState>(),
            history_bytes: size_of::<LiveCoreHistory>(),
            history_alignment: align_of::<LiveCoreHistory>(),
            queued_measurement_bytes: size_of::<
                FrontierScheduler<GnssObservation, MEASUREMENT_QUEUE_CAPACITY>,
            >(),
            imu_history_bytes: size_of::<FixedRing<ImuInterval, IMU_HISTORY_CAPACITY>>(),
            dense_history_bytes_each: size_of::<DenseHistory<DENSE_HISTORY_CAPACITY>>(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LiveCoreStatus {
    pub(crate) corrected_frontier: Option<SessionTime>,
    pub(crate) published_frontier: Option<SessionTime>,
    pub(crate) corrected_state_time: SessionTime,
    pub(crate) present_input_time: Option<SessionTime>,
    pub(crate) queued_measurements: usize,
    pub(crate) retained_imu_intervals: usize,
    pub(crate) retained_corrected_segments: usize,
    pub(crate) retained_predictor_segments: usize,
    pub(crate) finishing: bool,
    pub(crate) drained: bool,
    pub(crate) predictor_tracking: PredictorTrackingError,
    pub(crate) predictor_tracking_degraded: bool,
    pub(crate) predictor_gap: bool,
    pub(crate) predictor_degraded_input: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PendingCorrectedSegment {
    start: DenseEndpoint,
    integrated_attitude_delta: Vector3<f32>,
    end_specific_force_b: Vector3<f32>,
    degraded: bool,
    degraded_input: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CorrectedEpochKinematics {
    sample: CorrectedImuSample,
    angular_acceleration_eb_b: Option<Vector3<f32>>,
    angular_acceleration_covariance_b: Matrix3<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CorrectedImuSample {
    support_start: SessionTime,
    support_end: SessionTime,
    omega_ib_b: Vector3<f32>,
    specific_force_b: Vector3<f32>,
    accel_sample_covariance_b: CompactCovariance3,
    gyro_sample_covariance_b: CompactCovariance3,
    gap: bool,
}

impl CorrectedImuSample {
    fn same_latent_as(self, previous: Self) -> bool {
        self.omega_ib_b == previous.omega_ib_b
            && self.specific_force_b == previous.specific_force_b
            && self.accel_sample_covariance_b == previous.accel_sample_covariance_b
            && self.gyro_sample_covariance_b == previous.gyro_sample_covariance_b
            && ((self.support_start == previous.support_start
                && self.support_end == previous.support_end
                && self.gap == previous.gap)
                || (self.gap && self.support_start == previous.support_end))
    }
}

#[derive(Clone, Copy)]
struct PropagationPlan {
    piece_count: usize,
    ring_entries: usize,
    remainder: Option<ImuInterval>,
    last_piece: ImuInterval,
    last_sample: CorrectedImuSample,
    previous_sample: Option<CorrectedImuSample>,
}

/// Hot persistent state placed by the caller in internal SRAM. It contains no
/// self-reference, so a tiny [`LiveCore`] facade can borrow this state and the
/// independently placed PSRAM history for just one engine call.
#[cfg_attr(test, derive(Debug, PartialEq))]
pub(crate) struct LiveCoreState {
    scheduler: FrontierScheduler<GnssObservation, MEASUREMENT_QUEUE_CAPACITY>,
    filter: Eskf,
    /// Persistent transaction candidate. Keeping the second ESKF beside the
    /// committed state prevents a ~7 KiB filter copy from living on the S31
    /// task stack across nested propagation/update calls.
    transaction_filter: Eskf,
    predictor: OutputPredictor,
    predictor_preintegrator: Preintegrator,
    imu_noise: ImuNoise,
    context: MechanizationContext,
    nis_gate: NisGate,
    gap_model: GapModel,
    navigation_period_ns: i64,
    bias_correction_validity_norm: f32,
    // Fits the existing alignment slot beside the f32 tuning value.
    smoothing_lag_ns: i32,
    next_corrected_deadline: SessionTime,
    next_predictor_deadline: SessionTime,
    corrected_pending_interval: Option<ImuInterval>,
    pending_corrected_segment: Option<PendingCorrectedSegment>,
    previous_corrected_kinematics: Option<CorrectedImuSample>,
    corrected_epoch_kinematics: Option<CorrectedImuSample>,
    corrected_endpoint: DenseEndpoint,
    predictor_endpoint: DenseEndpoint,
    last_ingested_interval: Option<ImuInterval>,
    unreconciled_predictor_gap_end: Option<SessionTime>,
    next_corrected_segment_id: u64,
    next_predictor_segment_id: u64,
    active: bool,
}

impl LiveCoreState {
    /// Valid inactive state intended to live in statically placed caller-owned
    /// storage for the lifetime of the workspace.
    pub(crate) const fn placeholder() -> Self {
        Self {
            scheduler: FrontierScheduler::placeholder(),
            filter: Eskf::placeholder(),
            transaction_filter: Eskf::placeholder(),
            predictor: OutputPredictor::placeholder(),
            predictor_preintegrator: Preintegrator::placeholder(),
            imu_noise: ImuNoise {
                accel_covariance_density: Matrix3::from_array_storage(ArrayStorage([[0.0; 3]; 3])),
                gyro_covariance_density: Matrix3::from_array_storage(ArrayStorage([[0.0; 3]; 3])),
            },
            context: MechanizationContext::placeholder(),
            nis_gate: NisGate {
                soft_3d: 0.0,
                hard_3d: 0.0,
                soft_6d: 0.0,
                hard_6d: 0.0,
                maximum_covariance_inflation: 0.0,
            },
            gap_model: GapModel {
                maximum_gap_ns: 0,
                angular_acceleration_one_sigma: Vector3::new(0.0, 0.0, 0.0),
                jerk_one_sigma: Vector3::new(0.0, 0.0, 0.0),
            },
            navigation_period_ns: 0,
            bias_correction_validity_norm: 0.0,
            smoothing_lag_ns: 0,
            next_corrected_deadline: SessionTime::ZERO,
            next_predictor_deadline: SessionTime::ZERO,
            corrected_pending_interval: None,
            pending_corrected_segment: None,
            previous_corrected_kinematics: None,
            corrected_epoch_kinematics: None,
            corrected_endpoint: DenseEndpoint::placeholder(),
            predictor_endpoint: DenseEndpoint::placeholder(),
            last_ingested_interval: None,
            unreconciled_predictor_gap_end: None,
            next_corrected_segment_id: 0,
            next_predictor_segment_id: 0,
            active: false,
        }
    }

    /// Returns whether initialization completed successfully.
    pub(crate) const fn is_active(&self) -> bool {
        self.active
    }

    /// Restores a reusable inactive state without constructing or moving a
    /// second `LiveCoreState` on the task stack.
    pub(crate) fn reset(&mut self) {
        self.active = false;
        self.scheduler.reset();
        self.filter.reset();
        self.transaction_filter.reset();
        self.predictor.reset();
        self.predictor_preintegrator.reset();
        self.imu_noise.accel_covariance_density.fill(0.0);
        self.imu_noise.gyro_covariance_density.fill(0.0);
        self.context.reset();
        self.nis_gate.soft_3d = 0.0;
        self.nis_gate.hard_3d = 0.0;
        self.nis_gate.soft_6d = 0.0;
        self.nis_gate.hard_6d = 0.0;
        self.nis_gate.maximum_covariance_inflation = 0.0;
        self.gap_model.maximum_gap_ns = 0;
        self.gap_model.angular_acceleration_one_sigma.fill(0.0);
        self.gap_model.jerk_one_sigma.fill(0.0);
        self.navigation_period_ns = 0;
        self.bias_correction_validity_norm = 0.0;
        self.smoothing_lag_ns = 0;
        self.next_corrected_deadline = SessionTime::ZERO;
        self.next_predictor_deadline = SessionTime::ZERO;
        self.corrected_pending_interval = None;
        self.pending_corrected_segment = None;
        self.previous_corrected_kinematics = None;
        self.corrected_epoch_kinematics = None;
        self.corrected_endpoint = DenseEndpoint::placeholder();
        self.predictor_endpoint = DenseEndpoint::placeholder();
        self.last_ingested_interval = None;
        self.unreconciled_predictor_gap_end = None;
        self.next_corrected_segment_id = 0;
        self.next_predictor_segment_id = 0;
    }

    /// Initializes a pre-existing placeholder in place. The history is a
    /// read-only precondition and is never modified. On every error, `self`
    /// is restored to its inactive reset state and `history` is unchanged.
    pub(crate) fn initialize(
        &mut self,
        config: &LiveCoreConfig,
        seed: &LiveCoreSeed<'_>,
        history: &LiveCoreHistory,
    ) -> Result<(), LiveCoreError> {
        self.reset();
        let result = self.initialize_inner(config, seed, history);
        if result.is_err() {
            self.reset();
        }
        result
    }

    fn initialize_inner(
        &mut self,
        config: &LiveCoreConfig,
        seed: &LiveCoreSeed<'_>,
        history: &LiveCoreHistory,
    ) -> Result<(), LiveCoreError> {
        config.validate()?;
        if seed.active_consider > MAX_CONSIDER || !history.is_empty() {
            return Err(LiveCoreError::HistoryNotEmptyOrInvalidSeed);
        }
        let first_deadline =
            checked_time_add(seed.initialization.state.time, config.navigation_period_ns)?;

        self.scheduler
            .initialize(config.fusion_delay_ns)
            .map_err(LiveCoreError::Scheduler)?;
        self.filter
            .initialize(
                seed.initialization.state,
                &seed.initialization.covariance,
                seed.nav_consider_covariance,
                seed.consider_covariance,
                seed.active_consider,
                config.process_noise,
                config.covariance_policy,
            )
            .map_err(LiveCoreError::Eskf)?;
        self.predictor
            .initialize(seed.initialization.state, config.predictor)
            .map_err(LiveCoreError::Predictor)?;
        self.predictor_preintegrator
            .initialize(
                seed.initialization.state.time,
                seed.initialization.state.accel_bias_b,
                seed.initialization.state.gyro_bias_b,
                config.bias_correction_validity_norm,
            )
            .map_err(LiveCoreError::Preintegration)?;

        self.imu_noise = config.imu_noise;
        self.context = config.mechanization;
        self.nis_gate = config.nis_gate;
        self.gap_model = config.gap;
        self.navigation_period_ns = config.navigation_period_ns;
        self.bias_correction_validity_norm = config.bias_correction_validity_norm;
        self.smoothing_lag_ns = config.smoothing_lag_ns as i32;
        self.next_corrected_deadline = first_deadline;
        self.next_predictor_deadline = first_deadline;
        let endpoint = DenseEndpoint {
            state: seed.initialization.state,
            specific_force_b: Vector3::zeros(),
            covariance: DenseCovariance::from_navigation(&seed.initialization.covariance),
        };
        self.corrected_endpoint = endpoint;
        self.predictor_endpoint = endpoint;
        self.next_corrected_segment_id = 1;
        self.next_predictor_segment_id = 1;
        self.active = true;
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn new(
        config: LiveCoreConfig,
        seed: LiveCoreSeed<'_>,
        history: &LiveCoreHistory,
    ) -> Result<Self, LiveCoreError> {
        let mut result = Self::placeholder();
        result.initialize(&config, &seed, history)?;
        Ok(result)
    }
}

/// Small facade over separately placed internal-SRAM state and PSRAM history.
/// It is recreated for each public engine operation and never moves either
/// persistent region by value.
pub(crate) struct LiveCore<'a> {
    state: &'a mut LiveCoreState,
    history: &'a mut LiveCoreHistory,
}

impl Deref for LiveCore<'_> {
    type Target = LiveCoreState;

    fn deref(&self) -> &Self::Target {
        self.state
    }
}

impl DerefMut for LiveCore<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.state
    }
}

fn checked_time_add(time: SessionTime, nanoseconds: i64) -> Result<SessionTime, LiveCoreError> {
    time.as_ns()
        .checked_add(nanoseconds)
        .map(SessionTime::from_ns)
        .ok_or(LiveCoreError::TimeOverflow)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum LiveCoreError {
    InvalidConfiguration,
    HistoryNotEmptyOrInvalidSeed,
    InputClosed,
    TimeOverflow,
    SegmentIdOverflow,
    /// Private control-flow stop caught by `propagate_to`; never returned from
    /// the live-core API as a numerical or contract failure.
    PlanningQuotaExhausted,
    MeasurementTimeMismatch,
    ImuOverlapOrRegression,
    MissingInitialImuSupport,
    ImuIntervalTooLong,
    MissingImuSupport,
    RawImuHistoryFull,
    PredictorHistoryFull,
    PredictorHistoryUnavailable,
    PredictorWorkBoundExceeded,
    MeasurementQueueRejected(EnqueueDisposition),
    ClockTransitionRequiresReinitialization,
    InternalInvariant,
    Preintegration(PreintegrationError),
    Eskf(EskfError),
    Predictor(PredictorError),
    Scheduler(SchedulerError),
    DenseHistory(DenseHistoryError),
    Queue(QueueError),
    Reanchor(ReanchorError),
    Smoothing(super::smoothing::SmoothingError),
    SmoothingHistoryFull,
}
