//! Forward-filter lifecycle and the state retained between evidence tasks.

use crate::{
    config::{EngineConfig, GnssCorrelationPolicy},
    error::ProcessError,
    ids::ClockModelId,
    observation::{GnssSolutionObservation, ImuObservation, InputDisposition},
    offline::store::{StateStore, StoredCovariance, StoredIntegrationImu, StoredNominal},
    quality::{DiagnosticCounts, GnssState, TimingQuality},
    time::{DurationNs, SessionTime},
};

use nalgebra::{DMatrix, Matrix3, Vector3};

use super::{
    catalog::ConsiderCatalog,
    evidence::{QueuedTask, TimedTask},
    inertial::{
        build_held_imu, ensure_imu_support_is_contiguous, qualified_imu_support,
        rejected_imu_breaks_continuity,
    },
    initialization::initialization_pair_time,
    math::{COLORED_ERROR_DIMENSION, NAVIGATION_DIMENSION},
    measurement::{GnssField, should_reject_solution},
};

#[derive(Clone)]
pub(super) struct HeldImu {
    pub(super) start: SessionTime,
    pub(super) time: SessionTime,
    pub(super) angular_rate_body: [f64; 3],
    pub(super) specific_force_body: [f64; 3],
    pub(super) accelerometer_covariance: Matrix3<f64>,
    pub(super) gyroscope_covariance: Matrix3<f64>,
    pub(super) degraded_input: bool,
}

#[derive(Clone, Copy)]
pub(super) struct AngularAccelerationEstimate {
    pub(super) mean_body: Vector3<f64>,
    pub(super) covariance_body: Matrix3<f64>,
}

#[derive(Clone, Copy)]
pub(super) struct VelocityTimingModel {
    pub(super) antenna_acceleration_ecef: Vector3<f64>,
    pub(super) timing_sensitivity_covariance_ecef: Matrix3<f64>,
    pub(super) angular_rate_prediction_covariance_ecef: Matrix3<f64>,
}

#[derive(Clone)]
pub(super) struct ActiveImuSample {
    pub(super) start: SessionTime,
    pub(super) end: SessionTime,
    pub(super) covariance_body: DMatrix<f64>,
    pub(super) state_cross: DMatrix<f64>,
    pub(super) stored_interior_cut: bool,
}

impl ActiveImuSample {
    pub(super) fn record_stored_propagation(
        &mut self,
        end: SessionTime,
    ) -> Result<(), ProcessError> {
        // The current RTS record contains navigation/consider state only.
        // A held sample shared by two stored edges would make that sequence
        // non-Markov. Refuse it until the smoother retains the sample latent;
        // adjacent cross covariances alone do not recover the joint model.
        if self.stored_interior_cut && self.covariance_body.amax() > 0.0 {
            return Err(ProcessError::CapabilityUnavailable);
        }
        self.stored_interior_cut = end < self.end;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct QualifiedImuSupport {
    pub(super) start: SessionTime,
    pub(super) end: SessionTime,
    pub(super) duration: DurationNs,
    pub(super) clock_model: ClockModelId,
}

#[derive(Default)]
pub(super) struct PendingInitialization {
    pub(super) position: Option<GnssSolutionObservation>,
    pub(super) velocity: Option<GnssSolutionObservation>,
}

#[derive(Clone, Copy)]
pub(super) struct InitializationPair {
    pub(super) position: GnssSolutionObservation,
    pub(super) velocity: GnssSolutionObservation,
    pub(super) time: SessionTime,
}

impl PendingInitialization {
    pub(super) fn observe(
        &mut self,
        solution: GnssSolutionObservation,
        field: GnssField,
        time: SessionTime,
    ) -> Result<Option<InitializationPair>, ProcessError> {
        let field_time_matches =
            match field {
                GnssField::Position => solution
                    .position()
                    .is_some_and(|position| position.time.effective_time().ok() == Some(time)),
                GnssField::Velocity => solution
                    .velocity()
                    .is_some_and(|velocity| velocity.time.effective_time().ok() == Some(time)),
                GnssField::Joint => solution.position().zip(solution.velocity()).is_some_and(
                    |(position, velocity)| {
                        position.time.effective_time().ok() == Some(time)
                            && velocity.time.effective_time().ok() == Some(time)
                    },
                ),
            };
        if !field_time_matches {
            return Err(ProcessError::InvalidEvidence);
        }
        match field {
            GnssField::Position => self.position = Some(solution),
            GnssField::Velocity => self.velocity = Some(solution),
            GnssField::Joint => {
                self.position = Some(solution);
                self.velocity = Some(solution);
            }
        }
        let Some(position) = self.position else {
            return Ok(None);
        };
        let Some(velocity) = self.velocity else {
            return Ok(None);
        };
        Ok(
            initialization_pair_time(position, velocity)?.map(|time| InitializationPair {
                position,
                velocity,
                time,
            }),
        )
    }

    pub(super) fn clear(&mut self) {
        self.position = None;
        self.velocity = None;
    }
}

pub(super) struct OfflineFilter<'a> {
    pub(super) config: &'a EngineConfig<'a>,
    pub(super) catalog: &'a ConsiderCatalog,
    pub(super) state_dimension: usize,
    pub(super) colored_error: bool,
    pub(super) nominal: Option<StoredNominal>,
    pub(super) covariance: Option<StoredCovariance>,
    /// Previous-pass smoothed state at the current epoch.  Present only during
    /// an IEKS relinearization pass; the estimable covariance remains in the
    /// tangent basis of `nominal`, not this guide.
    pub(super) guide_nominal: Option<StoredNominal>,
    pub(super) relinearized: bool,
    pub(super) damping: f64,
    /// Most recently completed qualified interval. It is retained only to
    /// form a support-aligned derivative with `held_imu`; it is never used to
    /// extrapolate navigation state.
    pub(super) previous_imu: Option<HeldImu>,
    pub(super) held_imu: Option<HeldImu>,
    pub(super) active_imu_sample: Option<ActiveImuSample>,
    pub(super) pending_initialization: PendingInitialization,
    pub(super) last_processed_time: Option<SessionTime>,
    pub(super) gap_until: Option<SessionTime>,
    pub(super) requires_reinitialization: bool,
    pub(super) last_stored_time: Option<SessionTime>,
    pub(super) last_stored_covariance: Option<StoredCovariance>,
    pub(super) transition_accumulator: DMatrix<f64>,
    pub(super) consider_transition_accumulator: DMatrix<f64>,
    pub(super) process_accumulator: DMatrix<f64>,
    pub(super) sample_influence_accumulator: DMatrix<f64>,
    pub(super) last_stored_sample_cross: DMatrix<f64>,
    pub(super) integration_imu: Option<StoredIntegrationImu>,
    pub(super) connected: bool,
    pub(super) gnss_state: GnssState,
    pub(super) timing_quality: TimingQuality,
    pub(super) diagnostics: DiagnosticCounts,
    pub(super) objective: f64,
}

impl<'a> OfflineFilter<'a> {
    pub(super) fn new(
        config: &'a EngineConfig<'a>,
        catalog: &'a ConsiderCatalog,
        relinearized: bool,
        damping: f64,
    ) -> Result<Self, ProcessError> {
        if !damping.is_finite() || !(0.0..=1.0).contains(&damping) || damping == 0.0 {
            return Err(ProcessError::InvalidEvidence);
        }
        let colored_error = matches!(
            config.dynamics_profile.gnss.correlation,
            GnssCorrelationPolicy::GaussMarkov { .. }
        );
        let state_dimension = NAVIGATION_DIMENSION
            + if colored_error {
                COLORED_ERROR_DIMENSION
            } else {
                0
            };
        let consider_dimension = catalog.covariance.nrows();
        Ok(Self {
            config,
            catalog,
            state_dimension,
            colored_error,
            nominal: None,
            covariance: None,
            guide_nominal: None,
            relinearized,
            damping,
            previous_imu: None,
            held_imu: None,
            active_imu_sample: None,
            pending_initialization: PendingInitialization::default(),
            last_processed_time: None,
            gap_until: None,
            requires_reinitialization: false,
            last_stored_time: None,
            last_stored_covariance: None,
            transition_accumulator: DMatrix::identity(state_dimension, state_dimension),
            consider_transition_accumulator: DMatrix::zeros(state_dimension, consider_dimension),
            process_accumulator: DMatrix::zeros(state_dimension, state_dimension),
            sample_influence_accumulator: DMatrix::zeros(state_dimension, 6),
            last_stored_sample_cross: DMatrix::zeros(state_dimension, 6),
            integration_imu: None,
            connected: false,
            gnss_state: GnssState::Absent,
            timing_quality: TimingQuality::Modeled,
            diagnostics: DiagnosticCounts::default(),
            objective: 0.0,
        })
    }

    pub(super) fn process(
        &mut self,
        queued: QueuedTask,
        store: &mut dyn StateStore,
        guide: Option<StoredNominal>,
    ) -> Result<(), ProcessError> {
        if self
            .last_processed_time
            .is_some_and(|last| queued.time < last)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        match queued.task {
            TimedTask::Gap(gap) => {
                if gap.span.start() != queued.time || self.requires_reinitialization {
                    return Err(ProcessError::InvalidEvidence);
                }
                self.break_continuity();
                self.gap_until = Some(gap.span.end());
                self.requires_reinitialization = true;
                self.last_processed_time = Some(queued.time);
                return Ok(());
            }
            TimedTask::Reinitialize(evidence) => {
                if evidence.at != queued.time
                    || self.gap_until.is_some_and(|end| evidence.at < end)
                    || !self.catalog.clocks.iter().any(|clock| {
                        clock.model == evidence.input.initial_clock_prior.model
                            && clock.segment == evidence.input.initial_clock_prior.segment
                            && clock.reference_time
                                == evidence.input.initial_clock_prior.reference_time
                            && clock.validity.contains(evidence.at)
                    })
                {
                    return Err(ProcessError::InvalidEvidence);
                }
                self.break_continuity();
                self.gap_until = None;
                self.requires_reinitialization = false;
                self.diagnostics.reinitializations =
                    self.diagnostics.reinitializations.saturating_add(1);
                self.last_processed_time = Some(queued.time);
                return Ok(());
            }
            TimedTask::Imu(_)
            | TimedTask::GnssPosition(_)
            | TimedTask::GnssVelocity(_)
            | TimedTask::GnssJoint(_)
            | TimedTask::ClockTransition(_) => {
                if self.gap_until.is_some_and(|end| queued.time < end) {
                    return Err(ProcessError::InvalidEvidence);
                }
                if self.requires_reinitialization {
                    return Err(ProcessError::IncompleteEvidence);
                }
            }
        }
        if guide
            .as_ref()
            .is_some_and(|value| value.time != queued.time)
        {
            return Err(ProcessError::StorageCorrupt);
        }
        match queued.task {
            TimedTask::Imu(observation) => {
                self.process_imu(observation, queued.time, store, guide.as_ref())?;
            }
            TimedTask::GnssPosition(solution) => {
                self.process_gnss(
                    solution,
                    GnssField::Position,
                    queued.time,
                    store,
                    guide.as_ref(),
                )?;
            }
            TimedTask::GnssVelocity(solution) => {
                self.process_gnss(
                    solution,
                    GnssField::Velocity,
                    queued.time,
                    store,
                    guide.as_ref(),
                )?;
            }
            TimedTask::GnssJoint(solution) => {
                self.process_gnss(
                    solution,
                    GnssField::Joint,
                    queued.time,
                    store,
                    guide.as_ref(),
                )?;
            }
            TimedTask::ClockTransition(transition) => {
                let next = transition
                    .next_model
                    .map(|model| self.catalog.clock(model, queued.time));
                if matches!(next, Some(None))
                    || next.flatten().is_some_and(|model| {
                        model.segment != transition.next_segment
                            || transition.previous_model == transition.next_model
                    })
                    || transition.previous_model.is_some_and(|model| {
                        !self.catalog.clocks.iter().any(|clock| clock.model == model)
                    })
                {
                    return Err(ProcessError::IncompleteEvidence);
                }
                self.propagate_to(queued.time, guide.as_ref())?;
                self.timing_quality = TimingQuality::Discontinuous;
                self.diagnostics.clock_discontinuities =
                    self.diagnostics.clock_discontinuities.saturating_add(1);
                self.store_current(None, None, store)?;
                self.timing_quality = TimingQuality::Modeled;
            }
            TimedTask::Gap(_) | TimedTask::Reinitialize(_) => {
                return Err(ProcessError::InvalidEvidence);
            }
        }
        self.last_processed_time = Some(queued.time);
        Ok(())
    }

    pub(super) fn break_continuity(&mut self) {
        self.nominal = None;
        self.covariance = None;
        self.guide_nominal = None;
        self.previous_imu = None;
        self.held_imu = None;
        self.active_imu_sample = None;
        self.pending_initialization.clear();
        self.last_stored_time = None;
        self.last_stored_covariance = None;
        self.connected = false;
        self.gnss_state = GnssState::Absent;
        self.timing_quality = TimingQuality::Discontinuous;
        self.transition_accumulator.fill(0.0);
        self.transition_accumulator.fill_diagonal(1.0);
        self.consider_transition_accumulator.fill(0.0);
        self.process_accumulator.fill(0.0);
        self.sample_influence_accumulator.fill(0.0);
        self.last_stored_sample_cross.fill(0.0);
        self.integration_imu = None;
    }

    pub(super) fn process_imu(
        &mut self,
        observation: ImuObservation,
        time: SessionTime,
        store: &mut dyn StateStore,
        guide: Option<&StoredNominal>,
    ) -> Result<(), ProcessError> {
        let support = qualified_imu_support(observation)?;
        if support.start != time {
            return Err(ProcessError::InvalidEvidence);
        }
        let held = build_held_imu(self.config, observation, time)?;
        let mut completed_previous = None;
        if let Some(previous) = self.held_imu.clone() {
            if let Some(next) = held.as_ref() {
                ensure_imu_support_is_contiguous(&previous, next)?;
                completed_previous = Some(previous.clone());
            } else {
                if support.start < previous.time {
                    return Err(ProcessError::InvalidEvidence);
                }
                if support.start > previous.time {
                    // A selected interval is missing between two qualified IMU
                    // supports. The live engine can bridge an explicitly modeled
                    // synthetic interval; the offline smoother must fail closed.
                    return Err(ProcessError::IncompleteEvidence);
                }
            }
            if let Some(state) = self.nominal.as_ref() {
                if state.time < previous.start || state.time > previous.time {
                    return Err(ProcessError::InvalidEvidence);
                }
                if state.time < previous.time {
                    self.propagate_with_imu(previous.time, &previous, guide)?;
                    // Preserve every complete IMU support boundary in the
                    // smoothing store. Measurements within the interval may
                    // already have introduced additional exact-time states.
                    self.store_current(None, None, store)?;
                }
            }
        }
        if held.is_none() {
            self.diagnostics.imu_epochs_rejected =
                self.diagnostics.imu_epochs_rejected.saturating_add(1);
            self.previous_imu = None;
            self.held_imu = None;
            if rejected_imu_breaks_continuity(observation) {
                self.break_continuity();
            }
            return Ok(());
        }
        let Some(held) = held else {
            return Ok(());
        };
        self.diagnostics.imu_epochs_accepted =
            self.diagnostics.imu_epochs_accepted.saturating_add(1);
        self.previous_imu = completed_previous;
        self.held_imu = Some(held.clone());
        if self.nominal.is_some() {
            let state_time = self
                .nominal
                .as_ref()
                .ok_or(ProcessError::InvalidEvidence)?
                .time;
            if state_time != held.start {
                return Err(ProcessError::InvalidEvidence);
            }
            self.install_active_imu_sample(&held, None)?;
        }
        Ok(())
    }

    pub(super) fn process_gnss(
        &mut self,
        solution: GnssSolutionObservation,
        field: GnssField,
        time: SessionTime,
        store: &mut dyn StateStore,
        guide: Option<&StoredNominal>,
    ) -> Result<(), ProcessError> {
        if should_reject_solution(solution, field, self.config) {
            self.diagnostics.gnss_updates_rejected =
                self.diagnostics.gnss_updates_rejected.saturating_add(1);
            return Ok(());
        }
        if let GnssCorrelationPolicy::FixedDecimation { accept_every } =
            self.config.dynamics_profile.gnss.correlation
        {
            if !solution
                .id()
                .sequence
                .is_multiple_of(u64::from(accept_every.get()))
            {
                return Ok(());
            }
        }
        if self.nominal.is_none() {
            let Some(pair) = self.pending_initialization.observe(solution, field, time)? else {
                return Ok(());
            };
            if pair.time != time {
                return Err(ProcessError::InvalidEvidence);
            }
            self.initialize(
                pair.position,
                pair.velocity,
                pair.time,
                store,
                guide.cloned(),
            )?;
            self.pending_initialization.clear();
            self.diagnostics.gnss_updates_fused =
                self.diagnostics.gnss_updates_fused.saturating_add(1);
            return Ok(());
        }
        self.propagate_to(time, guide)?;
        let predicted = self.nominal.clone().ok_or(ProcessError::InvalidEvidence)?;
        let predicted_covariance = self
            .covariance
            .clone()
            .ok_or(ProcessError::InvalidEvidence)?;
        let outcome = self.update_gnss(solution, field, time, guide)?;
        self.objective += outcome.objective;
        match outcome.disposition {
            InputDisposition::Fused => {
                self.diagnostics.gnss_updates_fused =
                    self.diagnostics.gnss_updates_fused.saturating_add(1);
            }
            InputDisposition::Downweighted => {
                self.diagnostics.gnss_updates_downweighted =
                    self.diagnostics.gnss_updates_downweighted.saturating_add(1);
            }
            _ => {
                self.diagnostics.gnss_updates_rejected =
                    self.diagnostics.gnss_updates_rejected.saturating_add(1);
            }
        }
        self.store_current(
            Some((predicted, predicted_covariance)),
            Some((outcome.disposition, outcome.objective, outcome.reset_basis)),
            store,
        )?;
        Ok(())
    }
}
