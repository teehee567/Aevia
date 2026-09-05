//! Bounded IMU and GNSS ingestion and staging of predictor intervals.

use crate::{
    live::{
        dense_history::{DenseEndpoint, DenseSegment},
        eskf::GnssObservation,
        predictor::OutputPredictor,
        preintegration::{GapModel, ImuInterval, ImuNoise, Preintegrator},
        scheduler::{EnqueueDisposition, Scheduled},
        state::{MechanizationContext, so3_log},
    },
    time::SessionTime,
};

use super::{
    IngestDisposition, LiveCore, LiveCoreError, LiveCoreInput, MAX_IMU_INTERVAL_NS,
    MAX_PREDICTOR_SEGMENTS_PER_INGEST, checked_time_add,
};

impl<'a> LiveCore<'a> {
    pub(crate) fn ingest(
        &mut self,
        input: LiveCoreInput,
    ) -> Result<IngestDisposition, LiveCoreError> {
        match input {
            LiveCoreInput::Imu(interval) => self.ingest_imu(interval),
        }
    }

    #[cfg(test)]
    pub(crate) fn ingest_gnss(
        &mut self,
        observation: Scheduled<GnssObservation>,
    ) -> Result<EnqueueDisposition, LiveCoreError> {
        if self.scheduler.is_finishing() {
            return Err(LiveCoreError::InputClosed);
        }
        if observation.key.time != observation.value.time {
            return Err(LiveCoreError::MeasurementTimeMismatch);
        }
        observation.value.validate().map_err(LiveCoreError::Eskf)?;
        if observation.key.time < self.filter.state.time {
            return Ok(EnqueueDisposition::TooLateForLive);
        }
        Ok(self.scheduler.enqueue(observation))
    }

    /// Atomically enqueues the independently timed position and velocity
    /// members of one public receiver observation. A capacity/duplicate fault
    /// leaves the chronological queue unchanged.
    pub(crate) fn ingest_gnss_pair(
        &mut self,
        mut observations: [Option<Scheduled<GnssObservation>>; 2],
    ) -> Result<[Option<EnqueueDisposition>; 2], LiveCoreError> {
        if self.scheduler.is_finishing() {
            return Err(LiveCoreError::InputClosed);
        }
        for observation in observations.iter().flatten() {
            if observation.key.time != observation.value.time {
                return Err(LiveCoreError::MeasurementTimeMismatch);
            }
            observation.value.validate().map_err(LiveCoreError::Eskf)?;
        }
        let mut dispositions = [None; 2];
        for (index, candidate) in observations.iter_mut().enumerate() {
            if candidate.is_some_and(|observation| observation.key.time < self.filter.state.time) {
                dispositions[index] = Some(EnqueueDisposition::TooLateForLive);
                *candidate = None;
            }
        }
        let accepted = self
            .scheduler
            .enqueue_pair_atomic(&observations)
            .map_err(LiveCoreError::MeasurementQueueRejected)?;
        for index in 0..2 {
            if accepted[index].is_some() {
                dispositions[index] = accepted[index];
            }
        }
        Ok(dispositions)
    }

    pub(crate) fn ingest_imu(
        &mut self,
        interval: ImuInterval,
    ) -> Result<IngestDisposition, LiveCoreError> {
        if self.scheduler.is_finishing() {
            return Err(LiveCoreError::InputClosed);
        }
        interval.validate().map_err(LiveCoreError::Preintegration)?;
        let duration_ns = interval
            .end
            .as_ns()
            .checked_sub(interval.start.as_ns())
            .ok_or(LiveCoreError::TimeOverflow)?;
        if duration_ns > MAX_IMU_INTERVAL_NS {
            return Err(LiveCoreError::ImuIntervalTooLong);
        }

        let expected_start = self
            .last_ingested_interval
            .map_or(self.predictor_preintegrator.end(), |previous| previous.end);
        if interval.start < expected_start {
            return Err(LiveCoreError::ImuOverlapOrRegression);
        }
        if self.last_ingested_interval.is_none() && interval.start != expected_start {
            return Err(LiveCoreError::MissingInitialImuSupport);
        }

        let bridge = if interval.start > expected_start {
            let previous = self
                .last_ingested_interval
                .ok_or(LiveCoreError::MissingInitialImuSupport)?;
            Some(
                ImuInterval::bridge_after(previous, interval.start, self.gap_model)
                    .map_err(LiveCoreError::Preintegration)?,
            )
        } else {
            None
        };
        let raw_needed = 1 + usize::from(bridge.is_some());
        if self.history.imu.available() < raw_needed {
            return Err(LiveCoreError::RawImuHistoryFull);
        }

        let mut staged_predictor = self.predictor;
        let mut staged_preintegrator = self.predictor_preintegrator;
        let mut staged_endpoint = self.predictor_endpoint;
        let mut staged_corrected_endpoint = self.corrected_endpoint;
        if self.last_ingested_interval.is_none() {
            staged_endpoint.specific_force_b = interval.specific_force_b;
            staged_corrected_endpoint.specific_force_b = interval.specific_force_b;
        }
        let mut staged_deadline = self.next_predictor_deadline;
        let mut staged_next_id = self.next_predictor_segment_id;
        let context = self.context;
        let imu_noise = self.imu_noise;
        let gap_model = self.gap_model;
        let navigation_period_ns = self.navigation_period_ns;
        let bias_correction_validity_norm = self.bias_correction_validity_norm;
        self.history.predictor_staging.fill(None);
        let mut segment_count = 0;
        if let Some(gap) = bridge {
            stage_predictor_interval(
                gap,
                &context,
                imu_noise,
                gap_model,
                navigation_period_ns,
                bias_correction_validity_norm,
                &mut staged_predictor,
                &mut staged_preintegrator,
                &mut staged_endpoint,
                &mut staged_deadline,
                &mut staged_next_id,
                &mut self.history.predictor_staging,
                &mut segment_count,
            )?;
        }
        stage_predictor_interval(
            interval,
            &context,
            imu_noise,
            gap_model,
            navigation_period_ns,
            bias_correction_validity_norm,
            &mut staged_predictor,
            &mut staged_preintegrator,
            &mut staged_endpoint,
            &mut staged_deadline,
            &mut staged_next_id,
            &mut self.history.predictor_staging,
            &mut segment_count,
        )?;
        if self.history.predicted.available() < segment_count {
            return Err(LiveCoreError::PredictorHistoryFull);
        }

        // Every remaining operation has been prevalidated and capacity-checked;
        // the accepted observation becomes visible as one transaction.
        self.scheduler
            .observe_trusted_imu(interval.end)
            .map_err(LiveCoreError::Scheduler)?;
        if let Some(gap) = bridge {
            self.history
                .imu
                .push_back(gap)
                .map_err(LiveCoreError::Queue)?;
        }
        self.history
            .imu
            .push_back(interval)
            .map_err(LiveCoreError::Queue)?;
        for index in 0..segment_count {
            let segment =
                self.history.predictor_staging[index].ok_or(LiveCoreError::InternalInvariant)?;
            self.history
                .predicted
                .push(segment)
                .map_err(LiveCoreError::DenseHistory)?;
        }
        self.predictor = staged_predictor;
        self.predictor_preintegrator = staged_preintegrator;
        self.predictor_endpoint = staged_endpoint;
        self.corrected_endpoint = staged_corrected_endpoint;
        self.next_predictor_deadline = staged_deadline;
        self.next_predictor_segment_id = staged_next_id;
        self.last_ingested_interval = Some(interval);
        if let Some(gap) = bridge {
            self.unreconciled_predictor_gap_end = Some(
                self.unreconciled_predictor_gap_end
                    .map_or(gap.end, |previous| previous.max(gap.end)),
            );
        }

        Ok(IngestDisposition::ImuAccepted {
            stored_intervals: raw_needed as u8,
            predictor_segments: segment_count as u8,
            gap_bridged: bridge.is_some(),
        })
    }
}

pub(super) fn stage_predictor_interval(
    interval: ImuInterval,
    context: &MechanizationContext,
    imu_noise: ImuNoise,
    gap_model: GapModel,
    navigation_period_ns: i64,
    bias_correction_validity_norm: f32,
    predictor: &mut OutputPredictor,
    preintegrator: &mut Preintegrator,
    endpoint: &mut DenseEndpoint,
    next_deadline: &mut SessionTime,
    next_segment_id: &mut u64,
    segments: &mut [Option<DenseSegment>; MAX_PREDICTOR_SEGMENTS_PER_INGEST],
    segment_count: &mut usize,
) -> Result<(), LiveCoreError> {
    let mut remaining = interval;
    loop {
        let cut = if *next_deadline < remaining.end {
            *next_deadline
        } else {
            remaining.end
        };
        let (piece, tail) = split_interval(remaining, cut)?;
        if piece.is_gap() {
            preintegrator
                .push_gap(piece, imu_noise, gap_model, tail.is_some())
                .map_err(LiveCoreError::Preintegration)?;
        } else {
            preintegrator
                .push(piece, imu_noise)
                .map_err(LiveCoreError::Preintegration)?;
        }
        if cut == *next_deadline {
            if *segment_count == MAX_PREDICTOR_SEGMENTS_PER_INGEST {
                return Err(LiveCoreError::PredictorWorkBoundExceeded);
            }
            let batch = preintegrator
                .batch()
                .map_err(LiveCoreError::Preintegration)?;
            predictor
                .propagate(&batch, context)
                .map_err(LiveCoreError::Predictor)?;
            let end = DenseEndpoint {
                state: predictor.state,
                specific_force_b: piece.specific_force_b,
                covariance: endpoint.covariance,
            };
            let integrated_attitude_delta = so3_log(
                &(endpoint.state.orientation_n_from_b.inverse() * end.state.orientation_n_from_b),
            );
            segments[*segment_count] = Some(
                DenseSegment::new_imu_conditioned(
                    *next_segment_id,
                    *endpoint,
                    end,
                    integrated_attitude_delta,
                    batch.degraded,
                    batch.degraded_input,
                )
                .map_err(LiveCoreError::DenseHistory)?,
            );
            *segment_count += 1;
            *next_segment_id = next_segment_id
                .checked_add(1)
                .ok_or(LiveCoreError::SegmentIdOverflow)?;
            *endpoint = end;
            *preintegrator = Preintegrator::new(
                cut,
                predictor.state.accel_bias_b,
                predictor.state.gyro_bias_b,
                bias_correction_validity_norm,
            )
            .map_err(LiveCoreError::Preintegration)?;
            *next_deadline = checked_time_add(*next_deadline, navigation_period_ns)?;
        }
        if let Some(tail) = tail {
            remaining = tail;
        } else {
            return Ok(());
        }
    }
}

pub(super) fn split_interval(
    interval: ImuInterval,
    cut: SessionTime,
) -> Result<(ImuInterval, Option<ImuInterval>), LiveCoreError> {
    if cut <= interval.start || cut > interval.end {
        return Err(LiveCoreError::InternalInvariant);
    }
    if cut == interval.end {
        return Ok((interval, None));
    }
    let mut prefix = interval;
    prefix.end = cut;
    let mut suffix = interval;
    suffix.start = cut;
    if interval.is_gap() {
        let prefix_ns = cut
            .as_ns()
            .checked_sub(interval.start.as_ns())
            .and_then(|value| u32::try_from(value).ok())
            .ok_or(LiveCoreError::TimeOverflow)?;
        suffix.gap_elapsed_ns_plus_one = interval
            .gap_elapsed_ns_plus_one
            .checked_add(prefix_ns)
            .ok_or(LiveCoreError::TimeOverflow)?;
    }
    Ok((prefix, Some(suffix)))
}
