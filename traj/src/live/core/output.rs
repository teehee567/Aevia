//! Finishing the live stream and querying corrected and present output.

#[cfg(test)]
use super::LiveCoreSizes;
#[cfg(test)]
use crate::time::SessionTime;

use crate::live::{
    dense_history::{DenseEndpoint, DenseSegment},
    preintegration::Preintegrator,
    scheduler::SchedulerError,
    state::{NavState, so3_log},
};

use super::{FinishReport, LiveCore, LiveCoreError, LiveCoreStatus};

impl<'a> LiveCore<'a> {
    #[cfg(test)]
    pub(crate) const fn sizes() -> LiveCoreSizes {
        LiveCoreSizes::compiled()
    }

    pub(crate) fn finish(&mut self) -> Result<FinishReport, LiveCoreError> {
        if self.scheduler.is_finishing() {
            return Err(LiveCoreError::InputClosed);
        }
        let terminal_time = self
            .scheduler
            .latest_trusted_imu()
            .ok_or(LiveCoreError::Scheduler(SchedulerError::NoTrustedImu))?;
        let mut staged_predictor = self.predictor;
        let mut staged_endpoint = self.predictor_endpoint;
        let mut staged_segment = None;
        let mut staged_next_id = self.next_predictor_segment_id;
        if !self.predictor_preintegrator.is_empty() {
            let batch = self
                .predictor_preintegrator
                .batch()
                .map_err(LiveCoreError::Preintegration)?;
            staged_predictor
                .propagate(&batch, &self.context)
                .map_err(LiveCoreError::Predictor)?;
            let end = DenseEndpoint {
                state: staged_predictor.state,
                specific_force_b: batch.mean_specific_force_b,
                covariance: staged_endpoint.covariance,
            };
            let integrated_attitude_delta = so3_log(
                &(staged_endpoint.state.orientation_n_from_b.inverse()
                    * end.state.orientation_n_from_b),
            );
            staged_segment = Some(
                DenseSegment::new_imu_conditioned(
                    staged_next_id,
                    staged_endpoint,
                    end,
                    integrated_attitude_delta,
                    batch.degraded,
                    batch.degraded_input,
                )
                .map_err(LiveCoreError::DenseHistory)?,
            );
            staged_next_id = staged_next_id
                .checked_add(1)
                .ok_or(LiveCoreError::SegmentIdOverflow)?;
            staged_endpoint = end;
            if self.history.predicted.available() == 0 {
                return Err(LiveCoreError::PredictorHistoryFull);
            }
        }

        self.scheduler.finish().map_err(LiveCoreError::Scheduler)?;
        if let Some(segment) = staged_segment {
            self.history
                .predicted
                .push(segment)
                .map_err(LiveCoreError::DenseHistory)?;
            self.predictor = staged_predictor;
            self.predictor_endpoint = staged_endpoint;
            self.next_predictor_segment_id = staged_next_id;
            self.predictor_preintegrator = Preintegrator::new(
                terminal_time,
                self.predictor.state.accel_bias_b,
                self.predictor.state.gyro_bias_b,
                self.bias_correction_validity_norm,
            )
            .map_err(LiveCoreError::Preintegration)?;
        }
        Ok(FinishReport {
            terminal_time,
            predictor_segment_flushed: staged_segment.is_some(),
        })
    }

    pub(crate) fn present_state(&self) -> Result<NavState, LiveCoreError> {
        if self.predictor_preintegrator.is_empty() {
            return Ok(self.predictor.state);
        }
        let mut staged = self.predictor;
        let batch = self
            .predictor_preintegrator
            .batch()
            .map_err(LiveCoreError::Preintegration)?;
        staged
            .propagate(&batch, &self.context)
            .map_err(LiveCoreError::Predictor)?;
        Ok(staged.state)
    }

    #[cfg(test)]
    pub(crate) fn corrected_dense_state_at(
        &self,
        time: SessionTime,
    ) -> Result<crate::live::dense_history::DenseState, LiveCoreError> {
        self.history
            .corrected
            .state_at(time)
            .map_err(LiveCoreError::DenseHistory)
    }

    #[cfg(test)]
    pub(crate) fn predictor_dense_state_at(
        &self,
        time: SessionTime,
    ) -> Result<crate::live::dense_history::DenseState, LiveCoreError> {
        self.history
            .predicted
            .state_at(time)
            .map_err(LiveCoreError::DenseHistory)
    }

    pub(crate) fn pop_corrected_segment(&mut self) -> Option<DenseSegment> {
        self.history.corrected.pop_oldest()
    }

    pub(crate) fn status(&self) -> Result<LiveCoreStatus, LiveCoreError> {
        Ok(LiveCoreStatus {
            corrected_frontier: self.scheduler.processed_frontier(),
            corrected_state_time: self.filter.state.time,
            present_input_time: self.scheduler.latest_trusted_imu(),
            queued_measurements: self.scheduler.queued_measurements(),
            retained_imu_intervals: self.history.imu.len()
                + usize::from(self.corrected_pending_interval.is_some()),
            retained_corrected_segments: self.history.corrected.len(),
            retained_predictor_segments: self.history.predicted.len(),
            finishing: self.scheduler.is_finishing(),
            drained: self.is_drained_internal()?,
            predictor_tracking: self.predictor.tracking_error,
            predictor_tracking_degraded: self.predictor.tracking_degraded(),
            predictor_gap: self.unreconciled_predictor_gap_end.is_some(),
            predictor_degraded_input: self.predictor.degraded_input,
        })
    }
}
