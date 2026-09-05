//! Live construction, session ownership, and bounded public call lifecycle.

use self::configure::{make_consider_covariance, make_initializer, make_live_core_config};
use self::conversion::{covariance_density, map_core_step_error, matrix_from_array};
use self::preflight::{compile_live_consider_layout, validate_live_metric_bindings};
use self::quality::fusion_outcome;
use self::transaction::{LiveIngestCheckpoint, begin_source_sequence, rollback_source_sequence};
use super::{LiveFinishUpdate, LivePhase, LivePlatform, LiveSummary, LiveUpdate};
use crate::config::{EngineConfig, GnssCorrelationPolicy, LiveSpec, ScalarPolicy};
use crate::error::{PrepareError, StepError, ValidationError};
use crate::ids::{ClockModelId, ClockSegmentId, TrajectoryRevision};
use crate::live::{
    CapacityRequest, DrainReport, EcefAnchor, GnssQualityUpdate, ImuNoise, InitializationPhase,
    LiveCore, ReanchorMonitor, ReanchorPolicy, RequiredCapacities,
};
use crate::metric::LiveMetricUpdate;
use crate::observation::{
    ClockTransitionObservation, InputDisposition, LiveObservation, LiveStep, WorkQuota,
};
use crate::quality::{DiagnosticCounts, GnssState, HeadingSource, Integrity, TimingQuality};
use crate::time::{SessionTime, TimeSpan};
use crate::trajectory::{MAX_REFERENCE_POINTS as MAX_TRAJECTORY_POINTS, Trajectory};
use crate::workspace::{LiveWorkspace, WorkspaceRequirements};
use nalgebra::{Matrix3, Vector3 as NaVector3};

mod clock;
mod configure;
mod conversion;
mod frontier;
mod gnss;
mod imu;
mod initialization;
mod preflight;
mod quality;
mod transaction;

#[cfg(all(test, feature = "offline"))]
mod tests;

const V2_MINI_STACK_CONTRACT_BYTES: usize = 32 * 1_024;

/// Stateless live builder. Preflight is the only transition to a startable
/// plan, so stateful work cannot begin with a partially checked definition.
#[derive(Clone, Debug)]
pub struct LiveBuilder<'a> {
    spec: LiveSpec<'a>,
}

/// Fully checked live construction plan.
#[derive(Clone, Debug)]
pub struct LivePlan<'a> {
    spec: LiveSpec<'a>,
    requirements: WorkspaceRequirements,
    capacities: RequiredCapacities,
    consider_layout: LiveConsiderLayout,
    imu_noise: ImuNoise,
    body_from_imu: Matrix3<f32>,
}

/// Scalar-coordinate layout of the live Schmidt block. Clock offset/drift
/// always occupy columns 0 and 1; calibration definitions follow their
/// canonical covariance ordering.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LiveConsiderLayout {
    imu_boresight_start: u8,
    antenna_lever_start: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct InitializationFixEvidence {
    position_epoch: SessionTime,
    velocity_epoch: SessionTime,
    position_n: NaVector3<f32>,
    velocity_n: NaVector3<f32>,
    position_covariance_n: Matrix3<f32>,
    velocity_covariance_n: Matrix3<f32>,
    position_velocity_cross_n: Option<Matrix3<f32>>,
    position_independent_timing_sigma_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PendingClockTransition {
    observation: ClockTransitionObservation,
    preserve_navigation: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct GnssQualityEvidence {
    epoch: SessionTime,
    state: GnssState,
    timing: TimingQuality,
    downweighted: bool,
}

/// Small borrowed handle over caller-owned live workspaces.
pub struct LiveSession<'config, 'workspace> {
    engine: EngineConfig<'config>,
    internal: &'workspace mut crate::workspace::LiveInternalWorkspace,
    psram: &'workspace mut crate::workspace::LivePsramWorkspace,
    anchor: Option<EcefAnchor>,
    reanchor_monitor: ReanchorMonitor,
    latest_initialization_fix: Option<InitializationFixEvidence>,
    current_clock_model: Option<ClockModelId>,
    current_clock_segment: ClockSegmentId,
    last_clock_transition_time: Option<SessionTime>,
    pending_clock_transition: Option<PendingClockTransition>,
    clock_reference_time: SessionTime,
    clock_uncertainty_valid: bool,
    consider_layout: LiveConsiderLayout,
    imu_noise: ImuNoise,
    body_from_imu: Matrix3<f32>,
    initial_heading: Option<crate::config::InitialHeading>,
    last_accepted_imu_end: Option<SessionTime>,
    heading_source: HeadingSource,
    heading_variance_rad2: Option<f64>,
    gnss_state: GnssState,
    last_gnss_evidence: Option<GnssQualityEvidence>,
    timing_quality: TimingQuality,
    integrity: Integrity,
    predictor_tracking_degraded: bool,
    predictor_gap: bool,
    predictor_degraded_input: bool,
    diagnostics: DiagnosticCounts,
    terminal_time: Option<SessionTime>,
    finishing: bool,
    finished: bool,
}

#[derive(Clone, Copy)]
struct PublicCoreStatus {
    navigation_watermark: Option<SessionTime>,
}

#[derive(Clone, Copy)]
struct DrainedWork {
    report: DrainReport,
    remaining: u32,
    corrected_interval: Option<TimeSpan>,
    reanchor_generation: Option<u32>,
}

impl<'a> LiveBuilder<'a> {
    pub fn preflight(self) -> Result<LivePlan<'a>, PrepareError> {
        self.spec
            .validate()
            .map_err(PrepareError::InvalidDefinition)?;
        self.spec
            .metrics
            .plan()
            .validate_attachment_bindings(
                self.spec.engine.installation.attachment,
                self.spec.engine.installation.reference_points,
            )
            .map_err(PrepareError::InvalidDefinition)?;
        if !self.spec.engine.is_qualified() {
            return Err(PrepareError::UnqualifiedProfile);
        }
        if self.spec.engine.numeric_profile.scalar_policy != ScalarPolicy::EmbeddedMixedF32F64 {
            return Err(PrepareError::IncompatibleProfile);
        }
        if matches!(
            self.spec.engine.dynamics_profile.gnss.correlation,
            GnssCorrelationPolicy::GaussMarkov { .. }
        ) {
            return Err(PrepareError::IncompatibleProfile);
        }
        if self
            .spec
            .engine
            .dynamics_profile
            .permits_non_holonomic_constraint
        {
            return Err(PrepareError::IncompatibleProfile);
        }
        let consider_layout = compile_live_consider_layout(&self.spec.engine)?;
        validate_live_metric_bindings(
            &self.spec.engine,
            &self.spec.metrics,
            self.spec.initial_heading.is_some(),
        )?;
        // Compile private live math before a stateful session exists. Only the
        // actual ECEF origin remains deferred until a healthy first fix.
        make_initializer(&self.spec.engine)?;
        let preflight_anchor = EcefAnchor::from_origin(
            0,
            nalgebra::Vector3::new(6_378_137.0, 0.0, 0.0),
            self.spec.engine.processing_frame.ellipsoid(),
        )
        .map_err(|_| PrepareError::IncompatibleProfile)?;
        // These values are immutable for the entire session. Compile them
        // once so the S31 IMU path does not recast covariance matrices or
        // reconstruct the installation quaternion at sensor rate.
        let process = self.spec.engine.dynamics_profile.process_noise;
        let imu_noise = ImuNoise {
            accel_covariance_density: covariance_density(process.accelerometer)
                .map_err(|_| PrepareError::IncompatibleProfile)?,
            gyro_covariance_density: covariance_density(process.gyroscope)
                .map_err(|_| PrepareError::IncompatibleProfile)?,
        };
        let core_config = make_live_core_config(&self.spec.engine, &preflight_anchor, imu_noise)
            .map_err(|_| PrepareError::IncompatibleProfile)?;
        core_config
            .validate()
            .map_err(|_| PrepareError::IncompatibleProfile)?;
        let body_from_imu = matrix_from_array(
            self.spec
                .engine
                .installation
                .body_from_imu
                .mean
                .quaternion()
                .rotation_matrix(),
        )
        .map_err(|_| PrepareError::IncompatibleProfile)?;
        make_consider_covariance(&self.spec.engine, self.spec.initial_clock_prior)
            .map_err(|_| PrepareError::IncompatibleProfile)?;
        if self.spec.engine.installation.reference_points.len() > MAX_TRAJECTORY_POINTS {
            return Err(PrepareError::InvalidDefinition(
                ValidationError::CapacityExceeded,
            ));
        }

        let navigation = self.spec.engine.navigation_profile;
        let horizon = navigation
            .fusion_delay
            .as_ns()
            .checked_add(navigation.history_guard.as_ns())
            .ok_or(PrepareError::InvalidDefinition(
                ValidationError::TimeOverflow,
            ))?;
        let capacities = CapacityRequest {
            imu_rate_hz: u32::from(
                self.spec
                    .engine
                    .input_profile
                    .maximum_imu_samples_per_second,
            ),
            position_rate_hz: u32::from(
                self.spec
                    .engine
                    .input_profile
                    .maximum_position_updates_per_second,
            ),
            velocity_rate_hz: u32::from(
                self.spec
                    .engine
                    .input_profile
                    .maximum_velocity_updates_per_second,
            ),
            navigation_rate_hz: u32::from(navigation.navigation_cadence_hz),
            fusion_and_guard_ns: horizon,
            transition_reserve: 16,
        }
        .preflight()
        .map_err(|_| PrepareError::InsufficientResources)?;
        let requirements =
            WorkspaceRequirements::compiled(V2_MINI_STACK_CONTRACT_BYTES, navigation.digest);
        let resources = self.spec.resources;
        if requirements
            .internal_sram_bytes()
            .checked_add(requirements.maximum_stack_bytes())
            .is_none_or(|bytes| bytes > resources.internal_sram_bytes)
            || requirements.psram_bytes() > resources.psram_bytes
            || requirements.maximum_stack_bytes() > resources.stack_bytes
        {
            return Err(PrepareError::InsufficientResources);
        }
        Ok(LivePlan {
            spec: self.spec,
            requirements,
            capacities,
            consider_layout,
            imu_noise,
            body_from_imu,
        })
    }
}

impl<'a> LiveBuilder<'a> {
    pub(super) fn new(spec: LiveSpec<'a>) -> Self {
        Self { spec }
    }
}

impl LivePlan<'_> {
    #[must_use]
    pub const fn platform(&self) -> LivePlatform {
        LivePlatform::Esp32S31Wroom3N16R16V
    }

    #[must_use]
    pub const fn requirements(&self) -> WorkspaceRequirements {
        self.requirements
    }

    #[must_use]
    pub const fn required_imu_capacity(&self) -> usize {
        self.capacities.imu
    }

    #[must_use]
    pub const fn required_measurement_capacity(&self) -> usize {
        self.capacities.measurements
    }

    #[must_use]
    pub const fn required_dense_segment_capacity(&self) -> usize {
        self.capacities.dense_segments
    }
}

impl<'config> LivePlan<'config> {
    pub fn start<'workspace>(
        self,
        workspace: LiveWorkspace<'workspace>,
    ) -> Result<LiveSession<'config, 'workspace>, PrepareError> {
        if !workspace.validate(self.requirements) {
            return Err(PrepareError::InvalidWorkspaceAlignment);
        }

        let LiveWorkspace {
            internal, psram, ..
        } = workspace;
        internal.clear();
        internal.consider_seed_covariance =
            make_consider_covariance(&self.spec.engine, self.spec.initial_clock_prior)
                .map_err(|_| PrepareError::IncompatibleProfile)?;
        psram.clear(
            self.spec.engine.processing_frame,
            TrajectoryRevision::new(0),
        );
        psram
            .trajectory
            .set_attachment_model(self.spec.engine.installation.attachment)
            .map_err(PrepareError::InvalidDefinition)?;
        psram
            .trajectory
            .set_root_evaluation_budget(u32::from(self.spec.metrics.limits().max_root_evaluations))
            .map_err(PrepareError::InvalidDefinition)?;
        for point in self.spec.engine.installation.reference_points {
            psram
                .trajectory
                .add_reference_point(*point)
                .map_err(PrepareError::InvalidDefinition)?;
        }

        let mut initializer = make_initializer(&self.spec.engine)?;
        if let Some(heading) = self.spec.initial_heading {
            initializer
                .provide_heading(heading.radians() as f32, heading.variance.get() as f32)
                .map_err(|_| PrepareError::IncompatibleProfile)?;
        }
        internal.initializer = Some(initializer);
        psram
            .metric_tracker
            .configure(&self.spec.metrics)
            .map_err(PrepareError::InvalidDefinition)?;
        psram
            .metric_scratch
            .configure(&self.spec.metrics)
            .map_err(PrepareError::InvalidDefinition)?;

        Ok(LiveSession {
            engine: self.spec.engine,
            internal,
            psram,
            anchor: None,
            reanchor_monitor: ReanchorMonitor::new(ReanchorPolicy {
                trigger_distance_m: self
                    .spec
                    .engine
                    .navigation_profile
                    .reanchor_distance_m
                    .get() as f32,
                rearm_distance_m: self
                    .spec
                    .engine
                    .navigation_profile
                    .reanchor_hysteresis_m
                    .get() as f32,
            })
            .map_err(|_| PrepareError::IncompatibleProfile)?,
            latest_initialization_fix: None,
            current_clock_model: Some(self.spec.initial_clock_prior.model),
            current_clock_segment: self.spec.initial_clock_prior.segment,
            last_clock_transition_time: None,
            pending_clock_transition: None,
            clock_reference_time: self.spec.initial_clock_prior.reference_time,
            clock_uncertainty_valid: true,
            consider_layout: self.consider_layout,
            imu_noise: self.imu_noise,
            body_from_imu: self.body_from_imu,
            initial_heading: self.spec.initial_heading,
            last_accepted_imu_end: None,
            heading_source: HeadingSource::None,
            heading_variance_rad2: None,
            gnss_state: GnssState::Absent,
            last_gnss_evidence: None,
            timing_quality: TimingQuality::ArrivalOnly,
            integrity: Integrity::Unavailable,
            predictor_tracking_degraded: false,
            predictor_gap: false,
            predictor_degraded_input: false,
            diagnostics: DiagnosticCounts::default(),
            terminal_time: None,
            finishing: false,
            finished: false,
        })
    }
}

impl From<GnssQualityUpdate> for GnssQualityEvidence {
    fn from(value: GnssQualityUpdate) -> Self {
        Self {
            epoch: value.epoch,
            state: value.state,
            timing: value.timing,
            downweighted: value.downweighted,
        }
    }
}

impl DrainedWork {
    fn empty(remaining: u32) -> Self {
        Self {
            report: DrainReport::new(),
            remaining,
            corrected_interval: None,
            reanchor_generation: None,
        }
    }
}

impl LiveSession<'_, '_> {
    #[must_use]
    pub fn trajectory(&self) -> &Trajectory {
        &self.psram.trajectory
    }

    #[must_use]
    pub const fn diagnostics(&self) -> DiagnosticCounts {
        self.diagnostics
    }

    #[must_use]
    pub fn phase(&self) -> LivePhase {
        if self.finished {
            LivePhase::Finished
        } else if self.finishing {
            LivePhase::Finishing
        } else if self.internal.core.is_active() {
            let gnss_outage = self
                .last_accepted_imu_end
                .is_some_and(|time| self.gnss_evidence_is_stale_at(time));
            if self.predictor_tracking_degraded
                || self.predictor_gap
                || self.predictor_degraded_input
                || gnss_outage
            {
                LivePhase::Degraded
            } else {
                LivePhase::Navigating
            }
        } else if matches!(
            self.internal.initializer.as_ref().map(|value| value.phase),
            Some(InitializationPhase::Invalid)
        ) {
            LivePhase::Degraded
        } else {
            LivePhase::Initializing
        }
    }

    pub fn step<'update>(
        &'update mut self,
        step: LiveStep<'_>,
    ) -> Result<LiveUpdate<'update>, StepError> {
        if self.finished {
            return Err(StepError::Finished);
        }
        if self.finishing {
            return Err(StepError::AlreadyFinishing);
        }

        let mut input = None;
        if let Some(observation) = step.observation {
            let id = observation.id();
            // Commit the tiny source-sequence journal first. It is rolled back
            // together with scalar ingest state if semantic validation fails;
            // after a successful ingest, all estimator/runtime failures are
            // handled operationally and cannot turn the accepted call into a
            // transactional StepError.
            let sequence_undo = begin_source_sequence(&mut self.internal.sequences, id)?;
            let checkpoint = LiveIngestCheckpoint::capture(self);
            let disposition = match self.ingest_observation(*observation) {
                Ok(disposition) => disposition,
                Err(error) => {
                    checkpoint.restore(self);
                    rollback_source_sequence(&mut self.internal.sequences, sequence_undo);
                    return Err(error);
                }
            };
            input = Some((id, disposition));
        }

        let drained = match self.drain_work(step.work) {
            Ok(drained) => drained,
            Err(_) => {
                // Numerical/integrity failures after an accepted observation
                // are operational. Re-enter initialization and report the
                // state transition instead of returning StepError after
                // partially committed bounded work.
                self.invalidate_navigation();
                DrainedWork::empty(0)
            }
        };
        self.update_diagnostics(&drained.report);
        let mut status = match self.core_status() {
            Ok(status) => status,
            Err(_) => {
                self.invalidate_navigation();
                PublicCoreStatus {
                    navigation_watermark: None,
                }
            }
        };
        if self
            .refresh_metrics(status.navigation_watermark, false)
            .is_err()
        {
            self.invalidate_navigation();
            status.navigation_watermark = None;
        }
        let present = match self.present_projection() {
            Ok(present) => present,
            Err(_) => {
                self.invalidate_navigation();
                status.navigation_watermark = None;
                None
            }
        };
        let fusion = drained
            .report
            .last_gnss_key
            .zip(drained.report.last_gnss_outcome)
            .map(|(key, outcome)| fusion_outcome(key, outcome));
        let metric_watermark = self
            .internal
            .last_metric_update
            .as_ref()
            .and_then(LiveMetricUpdate::metric_watermark);
        let mutations = self
            .internal
            .last_metric_update
            .as_ref()
            .map_or(&[][..], LiveMetricUpdate::mutations);
        Ok(LiveUpdate {
            input,
            fusion,
            corrected_interval: drained.corrected_interval,
            reanchor_generation: drained.reanchor_generation,
            navigation_watermark: status.navigation_watermark,
            metric_watermark,
            present,
            mutations,
            diagnostics: self.diagnostics,
            phase: self.phase(),
            work_remaining: drained.remaining,
        })
    }

    /// Irrevocably closes input, then spends at most `work` frontier credits
    /// advancing to the final trusted IMU epoch. Fixed-capacity finish, metric,
    /// and projection phases are outside that credit count. Call repeatedly
    /// until `complete` is true.
    pub fn finish<'update>(
        &'update mut self,
        work: WorkQuota,
        summary: &mut LiveSummary,
    ) -> Result<LiveFinishUpdate<'update>, StepError> {
        if !self.finishing && !self.finished {
            if self.internal.core.is_active() {
                let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
                let report = core.finish().map_err(map_core_step_error)?;
                self.terminal_time = Some(report.terminal_time);
                self.finishing = true;
            } else {
                self.finished = true;
            }
        }

        let drained = if self.finished {
            DrainedWork::empty(work.units())
        } else {
            self.drain_work(work)?
        };
        self.update_diagnostics(&drained.report);
        let status = self.core_status()?;
        let complete = if self.finished || !self.internal.core.is_active() {
            true
        } else {
            LiveCore::attach(&mut self.internal.core, &mut self.psram.history)
                .status()
                .map(|status| status.drained)
                .map_err(map_core_step_error)?
        };
        if complete {
            self.finished = true;
        }
        self.refresh_metrics(status.navigation_watermark, complete)?;
        if complete {
            let finalized_metric_results = self.psram.metric_tracker.finalized_result_count();
            *summary = LiveSummary {
                terminal_time: self.terminal_time.or(status.navigation_watermark),
                retained_trajectory_span: self.psram.trajectory.span(),
                diagnostics: self.diagnostics,
                finalized_metric_results: u16::try_from(finalized_metric_results)
                    .unwrap_or(u16::MAX),
            };
        }
        let present = self.present_projection()?;
        let fusion = drained
            .report
            .last_gnss_key
            .zip(drained.report.last_gnss_outcome)
            .map(|(key, outcome)| fusion_outcome(key, outcome));
        let metric_watermark = self
            .internal
            .last_metric_update
            .as_ref()
            .and_then(LiveMetricUpdate::metric_watermark);
        let mutations = self
            .internal
            .last_metric_update
            .as_ref()
            .map_or(&[][..], LiveMetricUpdate::mutations);
        let update = LiveUpdate {
            input: None,
            fusion,
            corrected_interval: drained.corrected_interval,
            reanchor_generation: drained.reanchor_generation,
            navigation_watermark: status.navigation_watermark,
            metric_watermark,
            present,
            mutations,
            diagnostics: self.diagnostics,
            phase: self.phase(),
            work_remaining: drained.remaining,
        };
        Ok(LiveFinishUpdate { complete, update })
    }

    #[inline(never)]
    fn ingest_observation(
        &mut self,
        observation: LiveObservation,
    ) -> Result<InputDisposition, StepError> {
        match observation {
            LiveObservation::Imu(value) => self.ingest_imu(value),
            LiveObservation::GnssSolution(value) => self.ingest_gnss(value),
            LiveObservation::ClockTransition(value) => self.ingest_clock_transition(&value),
        }
    }
}
