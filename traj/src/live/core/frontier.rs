//! Quota-driven corrected-frontier draining, GNSS fusion, and segment finalization.

use crate::{
    live::{
        MEASUREMENT_QUEUE_CAPACITY,
        dense_history::{DenseCovariance, DenseEndpoint, DenseSegment},
        eskf::{GnssObservation, GnssUpdateOutcome, UpdateDecision},
        preintegration::imu_sample_covariance,
        scheduler::{SchedulerError, WorkQuota},
        state::NavState,
    },
    time::SessionTime,
};

use super::{
    DrainBlock, DrainReport, FRONTIER_COMMIT_CREDITS, GNSS_UPDATE_CREDITS, GnssQualityUpdate,
    LiveCore, LiveCoreError, PendingCorrectedSegment, SEGMENT_FINALIZATION_CREDITS,
};

impl<'a> LiveCore<'a> {
    #[inline(never)]
    pub(crate) fn drain(&mut self, quota: &mut WorkQuota) -> Result<DrainReport, LiveCoreError> {
        self.drain_through(quota, None)
    }

    /// Drains corrected work without crossing an optional exact control
    /// boundary. When enough trusted IMU support exists, the boundary is an
    /// additional propagation stop even when it falls between navigation
    /// cadence deadlines.
    #[inline(never)]
    pub(crate) fn drain_through(
        &mut self,
        quota: &mut WorkQuota,
        boundary: Option<SessionTime>,
    ) -> Result<DrainReport, LiveCoreError> {
        let mut report = DrainReport::new();
        loop {
            let scheduler_target = match self.scheduler.target() {
                Ok(target) => target,
                Err(SchedulerError::NoTrustedImu) => {
                    report.blocked_on = DrainBlock::AwaitingDelayedFrontier;
                    return Ok(report);
                }
                Err(error) => return Err(LiveCoreError::Scheduler(error)),
            };
            let boundary_reachable = boundary.is_some_and(|at| at <= scheduler_target);
            let target = boundary.map_or(scheduler_target, |at| at.min(scheduler_target));

            if let Some(pending) = self.pending_corrected_segment {
                if self.next_measurement_time()? == Some(self.filter.state.time) {
                    if !quota.take(GNSS_UPDATE_CREDITS) {
                        report.blocked_on = DrainBlock::QuotaExhausted;
                        return Ok(report);
                    }
                    self.fuse_next_measurement(&mut report)?;
                    continue;
                }
                if self.history.corrected.available() == 0 {
                    report.blocked_on = DrainBlock::CorrectedHistoryFull;
                    return Ok(report);
                }
                if !quota.take(SEGMENT_FINALIZATION_CREDITS) {
                    report.blocked_on = DrainBlock::QuotaExhausted;
                    return Ok(report);
                }
                self.finalize_pending_segment(pending, &mut report)?;
                continue;
            }

            if let Some(measurement_time) = self.next_measurement_time()? {
                if measurement_time < self.filter.state.time {
                    return Err(LiveCoreError::InternalInvariant);
                }
                if measurement_time == self.filter.state.time {
                    if !quota.take(GNSS_UPDATE_CREDITS) {
                        report.blocked_on = DrainBlock::QuotaExhausted;
                        return Ok(report);
                    }
                    self.fuse_next_measurement(&mut report)?;
                    continue;
                }
            }

            let flush_smoothing = (boundary_reachable && self.filter.state.time == target)
                || (self.scheduler.is_finishing() && self.filter.state.time == scheduler_target);
            if self.drain_smoothing(flush_smoothing, quota, &mut report)? {
                return Ok(report);
            }

            if self.scheduler.processed_frontier().is_none() && self.filter.state.time <= target {
                if !quota.take(FRONTIER_COMMIT_CREDITS) {
                    report.blocked_on = DrainBlock::QuotaExhausted;
                    return Ok(report);
                }
                self.corrected_endpoint.state = self.filter.state;
                self.corrected_endpoint.covariance =
                    DenseCovariance::from_navigation(&self.filter.covariance);
                let frontier = self.filter.state.time;
                self.scheduler
                    .commit_frontier(frontier)
                    .map_err(LiveCoreError::Scheduler)?;
                self.history.published_frontier.get_or_insert(frontier);
                report.frontier_commits = report.frontier_commits.saturating_add(1);
                continue;
            }

            let Some(stop) = self.next_propagation_stop(target, boundary_reachable)? else {
                report.blocked_on = if self.is_drained_internal()? {
                    DrainBlock::Finished
                } else {
                    DrainBlock::AwaitingDelayedFrontier
                };
                return Ok(report);
            };
            if !self.propagate_to(stop, quota, &mut report)? {
                return Ok(report);
            }
        }
    }

    pub(super) fn next_measurement_time(&self) -> Result<Option<SessionTime>, LiveCoreError> {
        self.scheduler
            .next_measurement()
            .map(|entry| entry.map(|scheduled| scheduled.key.time))
            .map_err(LiveCoreError::Scheduler)
    }

    #[inline(never)]
    pub(super) fn fuse_next_measurement(
        &mut self,
        report: &mut DrainReport,
    ) -> Result<(), LiveCoreError> {
        let scheduled = *self
            .scheduler
            .next_measurement()
            .map_err(LiveCoreError::Scheduler)?
            .ok_or(LiveCoreError::InternalInvariant)?;
        if scheduled.key.time != self.filter.state.time {
            return Err(LiveCoreError::InternalInvariant);
        }
        let kinematics = self.corrected_kinematics_at(self.filter.state.time)?;
        let mut observation = scheduled.value;
        observation.omega_ib_b = kinematics.sample.omega_ib_b;
        observation.specific_force_b = kinematics.sample.specific_force_b;
        observation.angular_acceleration_eb_b = kinematics.angular_acceleration_eb_b;
        observation.angular_acceleration_covariance_b =
            kinematics.angular_acceleration_covariance_b;
        if self
            .history
            .active_imu_sample
            .is_some_and(|previous| kinematics.sample.same_latent_as(previous))
        {
            self.history
                .propagation_scratch
                .stage_sample_cross(&self.history.active_imu_sample_nav_cross);
        } else {
            // At a shared boundary the new, right-owned sample has not
            // contributed to the state yet. A bridge still owns the held
            // previous sample and retains its cross covariance above.
            self.history
                .propagation_scratch
                .sample_candidate_mut()
                .fill(0.0);
        }
        let sample_covariance = imu_sample_covariance(
            kinematics.sample.accel_sample_covariance_b,
            kinematics.sample.gyro_sample_covariance_b,
        );
        self.transaction_filter = self.filter;
        if self.smoothing_lag_ns != 0 {
            self.history
                .smoothing_update_transaction
                .copy_from(&self.history.smoothing_update);
        }
        let context = self.context;
        let nis_gate = self.nis_gate;
        let velocity_needs_missing_alpha = observation.receiver_healthy
            && observation.velocity_n.is_some()
            && observation.imu_to_antenna_b.norm_squared() > 0.0
            && (observation.velocity_independent_timing_sigma_s > 0.0
                || observation.clock_consider_start.is_some())
            && observation.angular_acceleration_eb_b.is_none();
        let outcome =
            if velocity_needs_missing_alpha && observation.position_velocity_cross_n.is_some() {
                GnssUpdateOutcome {
                    position: None,
                    velocity: None,
                    joint: Some(UpdateDecision::RejectedInsufficientKinematics),
                }
            } else if velocity_needs_missing_alpha {
                observation.velocity_n = None;
                observation.position_velocity_cross_n = None;
                let mut outcome = if observation.position_n.is_some() {
                    self.state
                        .transaction_filter
                        .update_gnss_with_imu_sample_and_smoothing(
                            &observation,
                            &context,
                            nis_gate,
                            Some(&sample_covariance),
                            Some(self.history.propagation_scratch.sample_candidate_mut()),
                            (self.state.smoothing_lag_ns != 0)
                                .then_some(&mut self.history.smoothing_update_transaction),
                        )
                        .map_err(LiveCoreError::Eskf)?
                } else {
                    GnssUpdateOutcome {
                        position: None,
                        velocity: None,
                        joint: None,
                    }
                };
                outcome.velocity = Some(UpdateDecision::RejectedInsufficientKinematics);
                outcome
            } else {
                self.state
                    .transaction_filter
                    .update_gnss_with_imu_sample_and_smoothing(
                        &observation,
                        &context,
                        nis_gate,
                        Some(&sample_covariance),
                        Some(self.history.propagation_scratch.sample_candidate_mut()),
                        (self.state.smoothing_lag_ns != 0)
                            .then_some(&mut self.history.smoothing_update_transaction),
                    )
                    .map_err(LiveCoreError::Eskf)?
            };
        let quality_update = accepted_gnss_quality_update(&observation, outcome);
        if quality_update.is_some()
            && report.gnss_quality_update_count == MEASUREMENT_QUEUE_CAPACITY
        {
            return Err(LiveCoreError::InternalInvariant);
        }
        let popped = self
            .scheduler
            .pop_next_measurement()
            .map_err(LiveCoreError::Scheduler)?
            .ok_or(LiveCoreError::InternalInvariant)?;
        if popped.key != scheduled.key {
            return Err(LiveCoreError::InternalInvariant);
        }
        self.filter = self.transaction_filter;
        if self.smoothing_lag_ns != 0 {
            self.history
                .smoothing_update
                .copy_from(&self.history.smoothing_update_transaction);
        }
        self.history
            .propagation_scratch
            .commit_sample_candidate_into(&mut self.history.active_imu_sample_nav_cross);
        self.history.active_imu_sample = Some(kinematics.sample);
        report.gnss_updates = report.gnss_updates.saturating_add(1);
        for decision in [outcome.position, outcome.velocity, outcome.joint]
            .into_iter()
            .flatten()
        {
            match decision {
                UpdateDecision::Fused { .. } => {
                    report.gnss_fused = report.gnss_fused.saturating_add(1);
                }
                UpdateDecision::Downweighted { .. } => {
                    report.gnss_downweighted = report.gnss_downweighted.saturating_add(1);
                }
                UpdateDecision::RejectedInnovation { .. } | UpdateDecision::RejectedHealth => {
                    report.gnss_rejected = report.gnss_rejected.saturating_add(1);
                }
                UpdateDecision::RejectedInsufficientKinematics => {
                    report.gnss_rejected = report.gnss_rejected.saturating_add(1);
                }
            }
        }
        if let Some(update) = quality_update {
            self.history.current_quality = Some(update);
            if self.pending_corrected_segment.is_none() {
                self.history.endpoint_quality = Some(update);
            }
            self.history.smoothing.observation_accepted();
            let index = report.gnss_quality_update_count;
            report.gnss_quality_updates[index] = Some(update);
            report.gnss_quality_update_count += 1;
        }
        report.last_gnss_outcome = Some(outcome);
        report.last_gnss_key = Some(scheduled.key);
        Ok(())
    }

    pub(super) fn finalize_pending_segment(
        &mut self,
        pending: PendingCorrectedSegment,
        report: &mut DrainReport,
    ) -> Result<(), LiveCoreError> {
        let end = DenseEndpoint {
            state: self.filter.state,
            specific_force_b: pending.end_specific_force_b,
            covariance: DenseCovariance::from_navigation(&self.filter.covariance),
        };
        let segment = DenseSegment::new_imu_conditioned(
            self.next_corrected_segment_id,
            pending.start,
            end,
            pending.integrated_attitude_delta,
            pending.degraded,
            pending.degraded_input,
        )
        .map_err(LiveCoreError::DenseHistory)?;
        let predicted = self
            .history
            .predicted
            .state_at(self.filter.state.time)
            .map_err(|_| LiveCoreError::PredictorHistoryUnavailable)?;
        let predicted_state = NavState {
            time: predicted.time,
            position_n: predicted.position_n,
            velocity_n: predicted.velocity_n,
            orientation_n_from_b: predicted.orientation_n_from_b,
            accel_bias_b: self.predictor.state.accel_bias_b,
            gyro_bias_b: self.predictor.state.gyro_bias_b,
        };
        let mut staged_predictor = self.predictor;
        staged_predictor
            .correct_from_frontier(&self.filter.state, &predicted_state)
            .map_err(LiveCoreError::Predictor)?;

        self.retain_or_publish_segment(segment)?;
        let frontier = self.filter.state.time;
        self.scheduler
            .commit_frontier(frontier)
            .map_err(LiveCoreError::Scheduler)?;
        if self
            .unreconciled_predictor_gap_end
            .is_some_and(|gap_end| frontier >= gap_end)
        {
            self.unreconciled_predictor_gap_end = None;
        }
        self.history
            .predicted
            .discard_ending_at_or_before(self.filter.state.time);
        self.predictor = staged_predictor;
        self.corrected_endpoint = end;
        self.pending_corrected_segment = None;
        self.next_corrected_segment_id = self
            .next_corrected_segment_id
            .checked_add(1)
            .ok_or(LiveCoreError::SegmentIdOverflow)?;
        if self.smoothing_lag_ns == 0 {
            report.finalized_segments = report.finalized_segments.saturating_add(1);
        }
        report.frontier_commits = report.frontier_commits.saturating_add(1);
        Ok(())
    }

    pub(super) fn is_drained_internal(&self) -> Result<bool, LiveCoreError> {
        if !self.scheduler.is_finishing() || self.pending_corrected_segment.is_some() {
            return Ok(false);
        }
        let target = self.scheduler.target().map_err(LiveCoreError::Scheduler)?;
        Ok(self.filter.state.time == target
            && self.next_measurement_time()?.is_none()
            && !self.history.smoothing.has_tail())
    }
}

pub(super) fn accepted_gnss_quality_update(
    observation: &GnssObservation,
    outcome: GnssUpdateOutcome,
) -> Option<GnssQualityUpdate> {
    let mut accepted = false;
    let mut downweighted = false;
    for decision in [outcome.position, outcome.velocity, outcome.joint]
        .into_iter()
        .flatten()
    {
        match decision {
            UpdateDecision::Fused { .. } => accepted = true,
            UpdateDecision::Downweighted { .. } => {
                accepted = true;
                downweighted = true;
            }
            UpdateDecision::RejectedInnovation { .. }
            | UpdateDecision::RejectedHealth
            | UpdateDecision::RejectedInsufficientKinematics => {}
        }
    }
    accepted.then_some(GnssQualityUpdate {
        epoch: observation.time,
        state: observation.quality_state,
        timing: observation.quality_timing,
        downweighted,
    })
}
