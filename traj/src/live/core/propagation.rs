//! Planning and committing inertial propagation with support-aligned kinematics.

use crate::{
    live::{
        eskf::EskfError,
        preintegration::{
            PreintegratedBatch, PreintegrationError, Preintegrator, imu_sample_covariance,
        },
        scheduler::WorkQuota,
        state::{NavState, so3_log},
    },
    time::SessionTime,
};

use nalgebra::{Matrix3, Vector3};

use super::{
    CorrectedEpochKinematics, CorrectedImuSample, DrainBlock, DrainReport,
    FILTER_PROPAGATION_CREDITS, IMU_SLICE_CREDITS, LiveCore, LiveCoreError, MAX_PROPAGATION_SLICES,
    MIN_PROPAGATION_CREDITS, PendingCorrectedSegment, PropagationPlan, checked_time_add,
    ingestion::split_interval,
};

impl<'a> LiveCore<'a> {
    /// Keeps the propagation plan, preintegrated batch, and transactional ESKF
    /// copy out of the controller's frame. This boundary is part of the S31
    /// stack contract: the controller is live while either GNSS fusion or
    /// propagation runs, so merging these locals would make unrelated branch
    /// maxima additive.
    #[inline(never)]
    pub(super) fn propagate_to(
        &mut self,
        stop: SessionTime,
        quota: &mut WorkQuota,
        report: &mut DrainReport,
    ) -> Result<bool, LiveCoreError> {
        self.seed_smoothing()?;
        if self.smoothing_lag_ns != 0 && self.history.smoothing.is_full() {
            return Err(LiveCoreError::SmoothingHistoryFull);
        }
        let (plan, batch) = match self.plan_propagation(stop, quota) {
            Ok(planned) => planned,
            Err(LiveCoreError::PlanningQuotaExhausted) => {
                report.blocked_on = DrainBlock::QuotaExhausted;
                return Ok(false);
            }
            Err(error) => return Err(error),
        };

        let piece_count = plan.piece_count;
        if !quota.take(FILTER_PROPAGATION_CREDITS) {
            return Err(LiveCoreError::InternalInvariant);
        }
        self.state.transaction_filter = self.state.filter;
        let context = self.state.context;
        let prior_sample_cross = if let Some(leading) = batch.leading_sample {
            let sample = self
                .history
                .active_imu_sample
                .ok_or(LiveCoreError::Eskf(EskfError::ImuSampleLatentMismatch))?;
            if leading.covariance
                != imu_sample_covariance(
                    sample.accel_sample_covariance_b,
                    sample.gyro_sample_covariance_b,
                )
            {
                return Err(LiveCoreError::Eskf(EskfError::ImuSampleLatentMismatch));
            }
            Some(&self.history.active_imu_sample_nav_cross)
        } else {
            None
        };
        self.state
            .transaction_filter
            .propagate_with_imu_sample(
                &batch,
                &context,
                prior_sample_cross,
                &mut self.history.propagation_scratch,
            )
            .map_err(LiveCoreError::Eskf)?;
        self.commit_propagation(plan, batch, stop)?;
        self.history
            .propagation_scratch
            .commit_sample_candidate_into(&mut self.history.active_imu_sample_nav_cross);
        self.history.active_imu_sample = Some(plan.last_sample);
        self.capture_smoothing_prediction()?;
        report.imu_slices = report
            .imu_slices
            .saturating_add(u16::try_from(piece_count).unwrap_or(u16::MAX));
        report.filter_propagations = report.filter_propagations.saturating_add(1);
        Ok(true)
    }

    pub(crate) const fn corrected_state(&self) -> &NavState {
        &self.state.filter.state
    }

    pub(crate) const fn covariance_repairs(&self) -> u32 {
        self.state.filter.covariance_repairs
    }

    pub(super) fn next_propagation_stop(
        &self,
        target: SessionTime,
        exact_target: bool,
    ) -> Result<Option<SessionTime>, LiveCoreError> {
        if target <= self.filter.state.time {
            return Ok(None);
        }
        let mut stop = self.next_corrected_deadline;
        if let Some(measurement_time) = self.next_measurement_time()? {
            if measurement_time < stop {
                stop = measurement_time;
            }
        }
        if (self.scheduler.is_finishing() || exact_target) && target < stop {
            stop = target;
        }
        if stop <= self.filter.state.time {
            return Err(LiveCoreError::InternalInvariant);
        }
        if stop > target {
            Ok(None)
        } else {
            Ok(Some(stop))
        }
    }

    pub(super) fn plan_propagation(
        &self,
        stop: SessionTime,
        quota: &mut WorkQuota,
    ) -> Result<(PropagationPlan, PreintegratedBatch), LiveCoreError> {
        if quota.remaining() < MIN_PROPAGATION_CREDITS {
            return Err(LiveCoreError::PlanningQuotaExhausted);
        }
        let mut piece_count = 0;
        let mut ring_entries = 0;
        let mut cursor = self.filter.state.time;
        let mut pending = self.corrected_pending_interval;
        let mut remainder = None;
        let mut last_piece = None;
        let mut current_sample = self.history.active_imu_sample;
        let mut previous_sample = self
            .corrected_epoch_kinematics
            .filter(|previous| {
                current_sample.is_some_and(|current| {
                    previous.support_end == current.support_start
                        && !current.same_latent_as(*previous)
                })
            })
            .or(self.previous_corrected_kinematics);
        let mut preintegrator = Preintegrator::new(
            self.filter.state.time,
            self.filter.state.accel_bias_b,
            self.filter.state.gyro_bias_b,
            self.bias_correction_validity_norm,
        )
        .map_err(LiveCoreError::Preintegration)?;

        while cursor < stop {
            if piece_count == MAX_PROPAGATION_SLICES {
                return Err(LiveCoreError::Preintegration(
                    PreintegrationError::BatchCapacity,
                ));
            }
            // Keep the propagation credit reserved while planning. Each slice
            // is charged before its evidence is inspected and integrated, so
            // an incomplete local plan consumes only the planning work already
            // performed and cannot mutate retained evidence or filter state.
            if quota.remaining() < MIN_PROPAGATION_CREDITS {
                return Err(LiveCoreError::PlanningQuotaExhausted);
            }
            if !quota.take(IMU_SLICE_CREDITS) {
                return Err(LiveCoreError::InternalInvariant);
            }
            let (source, from_ring) = if let Some(interval) = pending.take() {
                (interval, false)
            } else {
                let interval = *self
                    .history
                    .imu
                    .get(ring_entries)
                    .ok_or(LiveCoreError::MissingImuSupport)?;
                ring_entries += 1;
                (interval, true)
            };
            if source.start != cursor {
                return Err(LiveCoreError::MissingImuSupport);
            }
            let source_continues_current = current_sample.is_some_and(|kinematics| {
                let payload_matches = kinematics.omega_ib_b == source.omega_ib_b
                    && kinematics.specific_force_b == source.specific_force_b
                    && kinematics.accel_sample_covariance_b == source.accel_sample_covariance
                    && kinematics.gyro_sample_covariance_b == source.gyro_sample_covariance;
                payload_matches
                    && ((!from_ring
                        && kinematics.support_end == source.end
                        && kinematics.gap == source.is_gap())
                        || (from_ring && source.is_gap() && kinematics.support_end == source.start)
                        || (from_ring
                            && kinematics.support_start == source.start
                            && kinematics.support_end == source.end
                            && kinematics.gap == source.is_gap()))
            });
            let source_sample_start = current_sample
                .filter(|_| source_continues_current)
                .map_or(source.start, |kinematics| kinematics.support_start);
            let cut = if source.end < stop { source.end } else { stop };
            let (piece, tail) = split_interval(source, cut)?;
            let continues_previous_sample = source_continues_current;
            let sample_active_after_piece = cut == stop;
            let next_sample = CorrectedImuSample {
                support_start: source_sample_start,
                support_end: source.end,
                omega_ib_b: source.omega_ib_b,
                specific_force_b: source.specific_force_b,
                accel_sample_covariance_b: source.accel_sample_covariance,
                gyro_sample_covariance_b: source.gyro_sample_covariance,
                gap: source.is_gap(),
            };
            if !source_continues_current {
                previous_sample = current_sample;
            }
            current_sample = Some(next_sample);
            if piece.is_gap() {
                preintegrator
                    .push_gap_piece(
                        piece,
                        self.imu_noise,
                        self.gap_model,
                        continues_previous_sample,
                        tail.is_some(),
                        sample_active_after_piece,
                    )
                    .map_err(LiveCoreError::Preintegration)?;
            } else {
                preintegrator
                    .push_piece(
                        piece,
                        self.imu_noise,
                        continues_previous_sample,
                        sample_active_after_piece,
                    )
                    .map_err(LiveCoreError::Preintegration)?;
            }
            last_piece = Some(piece);
            piece_count += 1;
            cursor = cut;
            if let Some(tail) = tail {
                remainder = Some(tail);
                if !from_ring {
                    pending = None;
                }
            }
        }
        let plan = PropagationPlan {
            piece_count,
            ring_entries,
            remainder,
            last_piece: last_piece.ok_or(LiveCoreError::MissingImuSupport)?,
            last_sample: current_sample.ok_or(LiveCoreError::MissingImuSupport)?,
            previous_sample,
        };
        let batch = preintegrator
            .batch()
            .map_err(LiveCoreError::Preintegration)?;
        Ok((plan, batch))
    }

    pub(super) fn commit_propagation(
        &mut self,
        plan: PropagationPlan,
        batch: PreintegratedBatch,
        stop: SessionTime,
    ) -> Result<(), LiveCoreError> {
        for _ in 0..plan.ring_entries {
            self.history
                .imu
                .pop_front()
                .ok_or(LiveCoreError::InternalInvariant)?;
        }
        self.corrected_pending_interval = plan.remainder;
        self.filter = self.transaction_filter;
        self.previous_corrected_kinematics = plan.previous_sample;
        self.corrected_epoch_kinematics = Some(plan.last_sample);
        let integrated_attitude_delta = so3_log(
            &(self.corrected_endpoint.state.orientation_n_from_b.inverse()
                * self.filter.state.orientation_n_from_b),
        );
        self.pending_corrected_segment = Some(PendingCorrectedSegment {
            start: self.corrected_endpoint,
            integrated_attitude_delta,
            end_specific_force_b: plan.last_piece.specific_force_b,
            degraded: batch.degraded,
            degraded_input: batch.degraded_input,
        });
        if stop == self.next_corrected_deadline {
            self.next_corrected_deadline =
                checked_time_add(self.next_corrected_deadline, self.navigation_period_ns)?;
        }
        Ok(())
    }

    pub(super) fn corrected_kinematics_at(
        &self,
        time: SessionTime,
    ) -> Result<CorrectedEpochKinematics, LiveCoreError> {
        // Qualified supports are half-open for measurement-epoch ownership:
        // at a shared boundary the interval starting there supplies the
        // instantaneous rate/force, while propagation up to the boundary has
        // already consumed the interval ending there. This matches offline
        // canonical tie ordering and dense-segment boundary ownership.
        let mut right_owned = None;
        for offset in 0..self.history.imu.len() {
            let interval = self
                .history
                .imu
                .get(offset)
                .ok_or(LiveCoreError::InternalInvariant)?;
            if interval.start == time {
                right_owned = Some(CorrectedImuSample {
                    support_start: interval.start,
                    support_end: interval.end,
                    omega_ib_b: interval.omega_ib_b,
                    specific_force_b: interval.specific_force_b,
                    accel_sample_covariance_b: interval.accel_sample_covariance,
                    gyro_sample_covariance_b: interval.gyro_sample_covariance,
                    gap: interval.is_gap(),
                });
                break;
            }
        }
        let base = if right_owned.is_some() {
            right_owned
        } else if let Some(kinematics) = self.corrected_epoch_kinematics {
            if kinematics.support_start <= time && time <= kinematics.support_end {
                Some(kinematics)
            } else {
                None
            }
        } else {
            None
        };
        let base = if base.is_some() {
            base
        } else if let Some(interval) = self.corrected_pending_interval {
            if interval.start <= time && time <= interval.end {
                Some(CorrectedImuSample {
                    support_start: interval.start,
                    support_end: interval.end,
                    omega_ib_b: interval.omega_ib_b,
                    specific_force_b: interval.specific_force_b,
                    accel_sample_covariance_b: interval.accel_sample_covariance,
                    gyro_sample_covariance_b: interval.gyro_sample_covariance,
                    gap: interval.is_gap(),
                })
            } else {
                None
            }
        } else {
            None
        };
        let base = if let Some(base) = base {
            base
        } else {
            let mut found = None;
            for offset in 0..self.history.imu.len() {
                let interval = self
                    .history
                    .imu
                    .get(offset)
                    .ok_or(LiveCoreError::InternalInvariant)?;
                if interval.start <= time && time <= interval.end {
                    found = Some(CorrectedImuSample {
                        support_start: interval.start,
                        support_end: interval.end,
                        omega_ib_b: interval.omega_ib_b,
                        specific_force_b: interval.specific_force_b,
                        accel_sample_covariance_b: interval.accel_sample_covariance,
                        gyro_sample_covariance_b: interval.gyro_sample_covariance,
                        gap: interval.is_gap(),
                    });
                    break;
                }
            }
            found.ok_or(LiveCoreError::MissingImuSupport)?
        };
        let (angular_acceleration_eb_b, angular_acceleration_covariance_b) = self
            .angular_acceleration_from_previous_sample(base)?
            .map_or((None, Matrix3::zeros()), |(mean, covariance)| {
                (Some(mean), covariance)
            });
        Ok(CorrectedEpochKinematics {
            sample: base,
            angular_acceleration_eb_b,
            angular_acceleration_covariance_b,
        })
    }

    pub(super) fn angular_acceleration_from_previous_sample(
        &self,
        current: CorrectedImuSample,
    ) -> Result<Option<(Vector3<f32>, Matrix3<f32>)>, LiveCoreError> {
        let previous = self
            .corrected_epoch_kinematics
            .filter(|candidate| candidate.support_end == current.support_start)
            .or(self.previous_corrected_kinematics);
        let Some(previous) = previous else {
            return Ok(None);
        };
        if current.gap || previous.gap || previous.support_end != current.support_start {
            return Ok(None);
        }
        let current_ns = current
            .support_end
            .as_ns()
            .checked_sub(current.support_start.as_ns())
            .ok_or(LiveCoreError::TimeOverflow)?;
        let previous_ns = previous
            .support_end
            .as_ns()
            .checked_sub(previous.support_start.as_ns())
            .ok_or(LiveCoreError::TimeOverflow)?;
        if current_ns <= 0 || previous_ns <= 0 {
            return Err(LiveCoreError::InternalInvariant);
        }
        let current_duration = current_ns as f32 * 1.0e-9;
        let previous_duration = previous_ns as f32 * 1.0e-9;
        let centre_separation = 0.5 * (current_duration + previous_duration);
        if !centre_separation.is_finite() || centre_separation <= 0.0 {
            return Err(LiveCoreError::TimeOverflow);
        }
        let rotation = self
            .filter
            .state
            .orientation_n_from_b
            .to_rotation_matrix()
            .into_inner();
        let earth_rate_body = rotation.transpose() * self.context.earth_rate_n;
        let omega_eb_body = current.omega_ib_b - self.filter.state.gyro_bias_b - earth_rate_body;
        let raw_derivative = (current.omega_ib_b - previous.omega_ib_b) / centre_separation;
        let alpha = raw_derivative + omega_eb_body.cross(&earth_rate_body);
        let current_rate_covariance = current.gyro_sample_covariance_b.to_matrix()
            + self.imu_noise.gyro_covariance_density / current_duration;
        let previous_rate_covariance = previous.gyro_sample_covariance_b.to_matrix()
            + self.imu_noise.gyro_covariance_density / previous_duration;
        let covariance = (current_rate_covariance + previous_rate_covariance)
            / (centre_separation * centre_separation);
        let covariance = (covariance + covariance.transpose()) * 0.5;
        if !alpha.iter().all(|value| value.is_finite())
            || !crate::live::preintegration::covariance_density_is_valid(&covariance)
        {
            return Err(LiveCoreError::InternalInvariant);
        }
        Ok(Some((alpha, covariance)))
    }
}
