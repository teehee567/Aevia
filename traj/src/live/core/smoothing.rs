//! Correlation snapshots and bounded publication of the live RTS window.

use crate::live::{
    dense_history::DenseSegment, preintegration::imu_sample_covariance,
    rts_window::RTS_STEP_CREDITS, scheduler::WorkQuota,
};

use super::{DrainBlock, DrainReport, GnssQualityUpdate, LiveCore, LiveCoreError};

impl LiveCore<'_> {
    pub(crate) fn reanchor_ready(&self) -> bool {
        !self.history.smoothing.is_busy()
    }

    pub(super) fn seed_smoothing(&mut self) -> Result<(), LiveCoreError> {
        if self.smoothing_lag_ns == 0 || !self.history.smoothing.is_empty() {
            return Ok(());
        }
        self.align_smoothing_sample()?;
        let sample = self
            .history
            .active_imu_sample
            .ok_or(LiveCoreError::MissingImuSupport)?;
        let covariance = imu_sample_covariance(
            sample.accel_sample_covariance_b,
            sample.gyro_sample_covariance_b,
        );
        self.history.smoothing.seed(
            &self.state.filter,
            &covariance,
            &self.history.active_imu_sample_nav_cross,
            self.state.corrected_endpoint,
            self.history.endpoint_quality,
        );
        self.history.smoothing_update.reset();
        Ok(())
    }

    /// Use the right-owned sample at a measurement epoch even when no GNSS
    /// arrives there. Its initially zero cross makes the next propagation's
    /// sample transition explicit without adding an observation.
    fn align_smoothing_sample(&mut self) -> Result<bool, LiveCoreError> {
        let selected = self.corrected_kinematics_at(self.filter.state.time)?.sample;
        let changed = !self
            .history
            .active_imu_sample
            .is_some_and(|old| selected.same_latent_as(old));
        if changed {
            self.history.active_imu_sample = Some(selected);
            self.history.active_imu_sample_nav_cross.fill(0.0);
        }
        Ok(changed)
    }

    pub(super) fn capture_smoothing_prediction(&mut self) -> Result<(), LiveCoreError> {
        if self.smoothing_lag_ns == 0 {
            return Ok(());
        }
        let sample = self
            .history
            .active_imu_sample
            .ok_or(LiveCoreError::MissingImuSupport)?;
        let covariance = imu_sample_covariance(
            sample.accel_sample_covariance_b,
            sample.gyro_sample_covariance_b,
        );
        self.history.smoothing.push_prediction(
            &self.state.filter,
            &covariance,
            &self.history.active_imu_sample_nav_cross,
            &self.history.propagation_scratch.rts_transition,
            self.history.current_quality,
        );
        if self.align_smoothing_sample()? {
            let selected = self
                .history
                .active_imu_sample
                .ok_or(LiveCoreError::MissingImuSupport)?;
            self.history
                .smoothing
                .replace_predicted_sample(&imu_sample_covariance(
                    selected.accel_sample_covariance_b,
                    selected.gyro_sample_covariance_b,
                ));
        }
        self.history.smoothing_update.reset();
        Ok(())
    }

    pub(super) fn retain_or_publish_segment(
        &mut self,
        segment: DenseSegment,
    ) -> Result<(), LiveCoreError> {
        if self.smoothing_lag_ns == 0 {
            self.publish_segment(
                segment,
                self.history.endpoint_quality,
                self.history.current_quality,
            )?;
        } else {
            let sample = self
                .history
                .active_imu_sample
                .ok_or(LiveCoreError::MissingImuSupport)?;
            let covariance = imu_sample_covariance(
                sample.accel_sample_covariance_b,
                sample.gyro_sample_covariance_b,
            );
            self.history.smoothing.finish_node(
                &self.state.filter,
                &covariance,
                &self.history.active_imu_sample_nav_cross,
                &self.history.smoothing_update,
                segment,
                self.history.current_quality,
            );
        }
        self.history.endpoint_quality = self.history.current_quality;
        Ok(())
    }

    pub(super) fn publish_segment(
        &mut self,
        segment: DenseSegment,
        start_quality: Option<GnssQualityUpdate>,
        end_quality: Option<GnssQualityUpdate>,
    ) -> Result<(), LiveCoreError> {
        self.history
            .corrected
            .push(segment)
            .map_err(LiveCoreError::DenseHistory)?;
        self.history
            .corrected_quality
            .push_back((start_quality, end_quality))
            .map_err(LiveCoreError::Queue)?;
        self.history.published_frontier = Some(segment.end_time());
        Ok(())
    }

    /// Returns true when the caller must yield. Forward propagation is held
    /// while a backward pass is in progress, so quota chunking cannot alter it.
    pub(super) fn drain_smoothing(
        &mut self,
        flush: bool,
        quota: &mut WorkQuota,
        report: &mut DrainReport,
    ) -> Result<bool, LiveCoreError> {
        if self.smoothing_lag_ns == 0 {
            return Ok(false);
        }
        loop {
            if !self.history.smoothing.ready(
                self.filter.state.time,
                i64::from(self.smoothing_lag_ns),
                flush,
            ) {
                return Ok(false);
            }
            if self.history.corrected.available() == 0 {
                report.blocked_on = DrainBlock::CorrectedHistoryFull;
                return Ok(true);
            }
            if !quota.take(RTS_STEP_CREDITS) {
                report.blocked_on = DrainBlock::QuotaExhausted;
                return Ok(true);
            }
            report.smoothing_steps = report.smoothing_steps.saturating_add(1);
            if !self
                .history
                .smoothing
                .smooth_one()
                .map_err(LiveCoreError::Smoothing)?
            {
                continue;
            }
            let (segment, start, end) = self
                .history
                .smoothing
                .publish_one()
                .map_err(LiveCoreError::DenseHistory)?;
            self.publish_segment(segment, start, end)?;
            report.finalized_segments = report.finalized_segments.saturating_add(1);
        }
    }
}
