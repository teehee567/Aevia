//! Advance the corrected frontier, reanchor, and transfer trajectory and metric updates.

use super::conversion::{map_core_step_error, trajectory_knot};
use super::quality::corrected_observability;
use super::{DrainedWork, LiveSession, PublicCoreStatus};
use crate::error::StepError;
use crate::live::{DrainReport, EcefAnchor, LiveCore, WorkQuota as CoreWorkQuota};
use crate::metric::MetricError;
use crate::observation::WorkQuota;
use crate::quality::EstimateStage;
use crate::time::{SessionTime, TimeSpan};

impl LiveSession<'_, '_> {
    #[inline(never)]
    pub(super) fn drain_work(&mut self, work: WorkQuota) -> Result<DrainedWork, StepError> {
        let mut quota = CoreWorkQuota::new(work.credits());
        if !self.internal.core.is_active() {
            return Ok(DrainedWork::empty(work.units()));
        }

        let clock_boundary = self
            .pending_clock_transition
            .map(|pending| pending.observation.at);
        let report = {
            let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            match clock_boundary {
                Some(boundary) => core.drain_through(&mut quota, Some(boundary)),
                None => core.drain(&mut quota),
            }
            .map_err(map_core_step_error)?
        };
        let clock_boundary_reached = clock_boundary.is_some_and(|at| {
            let core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            core.status().is_ok_and(|status| {
                status.corrected_frontier == Some(at) && status.corrected_state_time == at
            })
        });
        // A corrected-history capacity stop is resumed by the caller's next
        // bounded step; one public call never starts another core drain chunk.
        let corrected_interval = self.transfer_corrected_segments(&report)?;
        // Publish every old-segment endpoint through the exact boundary
        // before a fail-closed independent/unavailable transition discards
        // navigation state. No segment can then bridge across the reset.
        if clock_boundary_reached {
            if let Some(pending) = self.pending_clock_transition {
                let _ =
                    self.commit_clock_transition(pending.observation, pending.preserve_navigation);
            }
        }
        let reanchor_generation = if corrected_interval.is_some() {
            self.maybe_reanchor()?
        } else {
            None
        };
        Ok(DrainedWork {
            report,
            remaining: u32::from(quota.remaining()),
            corrected_interval,
            reanchor_generation,
        })
    }

    #[inline(never)]
    fn maybe_reanchor(&mut self) -> Result<Option<u32>, StepError> {
        let Some(old_anchor) = self.anchor else {
            return Ok(None);
        };
        if !self.internal.core.is_active() {
            return Ok(None);
        }
        let should_reanchor = {
            let core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            let status = core.status().map_err(map_core_step_error)?;
            status.corrected_frontier == Some(core.corrected_state().time)
                && self
                    .reanchor_monitor
                    .observe(core.corrected_state().position_n)
        };
        if !should_reanchor {
            return Ok(None);
        }
        let origin_ecef = {
            let core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            old_anchor.position_to_ecef(core.corrected_state().position_n)
        };
        let generation = old_anchor
            .generation
            .checked_add(1)
            .ok_or(StepError::EstimatorFailure)?;
        let new_anchor = EcefAnchor::from_origin(
            generation,
            origin_ecef,
            self.engine.processing_frame.ellipsoid(),
        )
        .map_err(|_| StepError::EstimatorFailure)?;
        {
            let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            core.reanchor(&old_anchor, &new_anchor)
                .map_err(map_core_step_error)?;
        }
        self.anchor = Some(new_anchor);
        Ok(Some(generation))
    }

    #[inline(never)]
    fn transfer_corrected_segments(
        &mut self,
        report: &DrainReport,
    ) -> Result<Option<TimeSpan>, StepError> {
        if !self.internal.core.is_active() {
            return Ok(None);
        }
        let anchor = self.anchor.ok_or(StepError::WorkspaceContract)?;
        let mut evidence = self.last_gnss_evidence;
        let mut quality_updates = report.gnss_quality_updates().peekable();
        let mut first = None;
        let mut last = None;
        loop {
            let segment = {
                let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
                core.pop_corrected_segment()
            };
            let Some(segment) = segment else {
                break;
            };
            let integrated_attitude_delta = segment.integrated_attitude_delta();
            while quality_updates
                .peek()
                .is_some_and(|update| update.epoch <= segment.start.state.time)
            {
                evidence = quality_updates.next().map(Into::into);
            }
            let start_observability =
                corrected_observability(segment.start, &anchor, &self.engine, self.heading_source)?;
            let start_quality = self.quality_at(
                EstimateStage::Finalized,
                segment.degraded,
                segment.start.state.time,
                evidence,
                segment.degraded_input,
            );
            while quality_updates
                .peek()
                .is_some_and(|update| update.epoch <= segment.end.state.time)
            {
                evidence = quality_updates.next().map(Into::into);
            }
            let end_observability =
                corrected_observability(segment.end, &anchor, &self.engine, self.heading_source)?;
            let end_quality = self.quality_at(
                EstimateStage::Finalized,
                segment.degraded,
                segment.end.state.time,
                evidence,
                segment.degraded_input,
            );
            let start = trajectory_knot(
                segment.start,
                segment.degraded,
                segment.degraded_input,
                &anchor,
                start_quality,
                start_observability,
            )?;
            let end = trajectory_knot(
                segment.end,
                segment.degraded,
                segment.degraded_input,
                &anchor,
                end_quality,
                end_observability,
            )?;
            self.psram
                .trajectory
                .push_rolling_imu_segment(
                    start,
                    end,
                    [
                        integrated_attitude_delta.x as f64,
                        integrated_attitude_delta.y as f64,
                        integrated_attitude_delta.z as f64,
                    ],
                )
                .map_err(|_| StepError::OutputCapacityExceeded)?;
            first.get_or_insert(start.time);
            last = Some(end.time);
        }
        for update in quality_updates {
            evidence = Some(update.into());
        }
        self.commit_gnss_evidence(evidence);
        first
            .zip(last)
            .map(|(start, end)| TimeSpan::new(start, end).map_err(StepError::InvalidObservation))
            .transpose()
    }

    pub(super) fn core_status(&mut self) -> Result<PublicCoreStatus, StepError> {
        if !self.internal.core.is_active() {
            self.predictor_tracking_degraded = false;
            self.predictor_gap = false;
            self.predictor_degraded_input = false;
            return Ok(PublicCoreStatus {
                navigation_watermark: None,
            });
        }
        let (status, covariance_repairs) = {
            let core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            (
                core.status().map_err(map_core_step_error)?,
                core.covariance_repairs(),
            )
        };
        self.diagnostics.covariance_repairs = covariance_repairs;
        self.predictor_tracking_degraded = status.predictor_tracking_degraded;
        self.predictor_gap = status.predictor_gap;
        self.predictor_degraded_input = status.predictor_degraded_input;
        Ok(PublicCoreStatus {
            navigation_watermark: status.corrected_frontier,
        })
    }

    #[inline(never)]
    pub(super) fn refresh_metrics(
        &mut self,
        navigation_watermark: Option<SessionTime>,
        end_of_input: bool,
    ) -> Result<(), StepError> {
        self.internal.last_metric_update = None;
        if self.psram.metric_tracker.has_pending_withdrawals() {
            let output = self
                .internal
                .last_metric_update
                .get_or_insert_with(crate::metric::LiveMetricUpdate::empty);
            self.psram
                .metric_tracker
                .drain_pending_withdrawals_into(output)
                .map_err(|_| StepError::OutputCapacityExceeded)?;
            return Ok(());
        }
        let (Some(watermark), Some(_)) = (navigation_watermark, self.psram.trajectory.span())
        else {
            return Ok(());
        };
        let output = self
            .internal
            .last_metric_update
            .get_or_insert_with(crate::metric::LiveMetricUpdate::empty);
        match self.psram.metric_tracker.update_into(
            &self.psram.trajectory,
            watermark,
            end_of_input,
            &mut self.psram.metric_scratch,
            output,
        ) {
            Ok(()) => {}
            Err(MetricError::CapacityExceeded | MetricError::EvaluationBudgetExceeded) => {
                self.internal.last_metric_update = None;
                self.psram
                    .metric_tracker
                    .begin_quality_invalidation(watermark);
                self.diagnostics.output_overflows =
                    self.diagnostics.output_overflows.saturating_add(1);
            }
            Err(MetricError::AmbiguousRoot) => {
                self.internal.last_metric_update = None;
                self.psram
                    .metric_tracker
                    .begin_quality_invalidation(watermark);
                self.diagnostics.metric_ambiguities =
                    self.diagnostics.metric_ambiguities.saturating_add(1);
            }
            Err(_) => {
                self.internal.last_metric_update = None;
                self.psram
                    .metric_tracker
                    .begin_quality_invalidation(watermark);
                // The compiled live plan cannot turn a navigation commit into
                // a transactional StepError. Preserve navigation and expose a
                // metric-integrity diagnostic for the unavailable update.
                self.diagnostics.metric_ambiguities =
                    self.diagnostics.metric_ambiguities.saturating_add(1);
            }
        }
        Ok(())
    }
}
