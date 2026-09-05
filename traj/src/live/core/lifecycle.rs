//! Attaching live state, seeding sample correlations, and transactional frame transitions.

use crate::{
    live::{
        eskf::{EskfError, GnssObservation},
        preintegration::ImuInterval,
        reanchor::{EcefAnchor, ReanchorError, ReanchorTransform},
    },
    time::SessionTime,
};

use nalgebra::Vector3;

use super::{CorrectedImuSample, LiveCore, LiveCoreError, LiveCoreHistory, LiveCoreState};

impl<'a> LiveCore<'a> {
    pub(crate) fn attach(state: &'a mut LiveCoreState, history: &'a mut LiveCoreHistory) -> Self {
        Self { state, history }
    }

    /// Retains the sample error used to transform the initial antenna
    /// velocity to the IMU. Navigation errors are true minus nominal, while
    /// sample errors are observed minus true, hence the negative cross sign.
    pub(crate) fn seed_initial_imu_sample(
        &mut self,
        interval: ImuInterval,
        lever_b: Vector3<f32>,
    ) -> Result<(), LiveCoreError> {
        if interval.start != self.filter.state.time || self.history.active_imu_sample.is_some() {
            return Err(LiveCoreError::InternalInvariant);
        }
        interval.validate().map_err(LiveCoreError::Preintegration)?;
        let cross = -self
            .filter
            .state
            .orientation_n_from_b
            .to_rotation_matrix()
            .into_inner()
            * crate::live::state::skew(&lever_b)
            * interval.gyro_sample_covariance.to_matrix();
        if cross.iter().any(|value| !value.is_finite()) {
            return Err(LiveCoreError::Eskf(EskfError::NumericalFailure));
        }
        let sample = CorrectedImuSample {
            support_start: interval.start,
            support_end: interval.end,
            omega_ib_b: interval.omega_ib_b,
            specific_force_b: interval.specific_force_b,
            accel_sample_covariance_b: interval.accel_sample_covariance,
            gyro_sample_covariance_b: interval.gyro_sample_covariance,
            gap: interval.is_gap(),
        };
        self.history.active_imu_sample_nav_cross.fill(0.0);
        self.history
            .active_imu_sample_nav_cross
            .fixed_view_mut::<3, 3>(crate::live::state::VEL, 3)
            .copy_from(&cross);
        self.history.active_imu_sample = Some(sample);
        self.corrected_epoch_kinematics = Some(sample);
        Ok(())
    }

    /// Transfers the running joint clock uncertainty only when no delayed
    /// measurement still refers to the previous segment. A caller receiving
    /// `ClockTransitionRequiresReinitialization` must discard navigation;
    /// relabeling those queued Jacobians as the new clock would be inexact.
    #[inline(never)]
    pub(crate) fn transition_clock_consider(
        &mut self,
        active_consider: usize,
        next_clock_from_previous: &[[f32; crate::live::MAX_CONSIDER]; 2],
        innovation_covariance_upper: [f32; 3],
    ) -> Result<(), LiveCoreError> {
        if self.scheduler.queued_measurements() != 0 {
            return Err(LiveCoreError::ClockTransitionRequiresReinitialization);
        }
        let state = &mut *self.state;
        state.transaction_filter = state.filter;
        state
            .filter
            .transition_clock_consider_into(
                &mut state.transaction_filter,
                active_consider,
                next_clock_from_previous,
                innovation_covariance_upper,
            )
            .map_err(LiveCoreError::Eskf)?;
        state.filter = state.transaction_filter;
        Ok(())
    }

    /// Returns whether all retained delayed measurements precede an exact
    /// future clock boundary. Measurements at or after the boundary are
    /// expressed in the next segment and must never be interpreted against
    /// the current consider block.
    pub(crate) fn can_schedule_clock_transition_at(&self, at: SessionTime) -> bool {
        if self.filter.state.time > at {
            return false;
        }
        let mut safe = true;
        let result = self.scheduler.try_for_each_measurement(|measurement| {
            if measurement.key.time >= at {
                safe = false;
            }
            Ok::<(), ()>(())
        });
        result.is_ok() && safe
    }

    /// Atomically changes the fixed local navigation anchor. Every retained
    /// object expressed in navigation coordinates is validated under the new
    /// frame before any persistent state is changed. Body-frame IMU evidence,
    /// time/order state, counters, and fixed-mean parameters are invariant.
    #[inline(never)]
    pub(crate) fn reanchor(
        &mut self,
        old_anchor: &EcefAnchor,
        new_anchor: &EcefAnchor,
    ) -> Result<(), LiveCoreError> {
        let transform =
            ReanchorTransform::between(old_anchor, new_anchor).map_err(LiveCoreError::Reanchor)?;

        // Stage the hot values and validate the large caller-owned histories
        // in place without copying them onto the MCU task stack.
        {
            let state = &mut *self.state;
            transform
                .map_filter_into(&state.filter, &mut state.transaction_filter)
                .map_err(LiveCoreError::Reanchor)?;
        }
        self.predictor
            .validate_reanchor(&transform)
            .map_err(LiveCoreError::Reanchor)?;
        let staged_context = transform
            .map_context(self.context)
            .map_err(LiveCoreError::Reanchor)?;
        let staged_corrected_endpoint = self
            .corrected_endpoint
            .mapped_reanchor(&transform)
            .map_err(LiveCoreError::Reanchor)?;
        let staged_predictor_endpoint = self
            .predictor_endpoint
            .mapped_reanchor(&transform)
            .map_err(LiveCoreError::Reanchor)?;
        let staged_pending_start = self
            .pending_corrected_segment
            .map(|pending| pending.start.mapped_reanchor(&transform))
            .transpose()
            .map_err(LiveCoreError::Reanchor)?;

        self.history
            .corrected
            .validate_reanchor(&transform)
            .map_err(LiveCoreError::DenseHistory)?;
        self.history
            .predicted
            .validate_reanchor(&transform)
            .map_err(LiveCoreError::DenseHistory)?;
        self.scheduler.try_for_each_measurement(|scheduled| {
            mapped_gnss(scheduled.value, &transform)
                .validate()
                .map_err(LiveCoreError::Eskf)
        })?;

        // Raw retained IMU quantities and partial preintegrations are all in
        // body coordinates. Validate that support before committing, but do
        // not rotate it.
        for offset in 0..self.history.imu.len() {
            self.history
                .imu
                .get(offset)
                .ok_or(LiveCoreError::InternalInvariant)?
                .validate()
                .map_err(LiveCoreError::Preintegration)?;
        }
        if let Some(interval) = self.corrected_pending_interval {
            interval.validate().map_err(LiveCoreError::Preintegration)?;
        }
        if let Some(interval) = self.last_ingested_interval {
            interval.validate().map_err(LiveCoreError::Preintegration)?;
        }

        let candidate = self.history.propagation_scratch.sample_candidate_mut();
        candidate
            .copy_from(&(transform.covariance_jacobian * self.history.active_imu_sample_nav_cross));
        if candidate.iter().any(|value| !value.is_finite()) {
            return Err(LiveCoreError::Reanchor(ReanchorError::NonFinite));
        }

        // Nothing below this line is fallible: this is the atomic commit.
        self.history
            .propagation_scratch
            .commit_sample_candidate_into(&mut self.history.active_imu_sample_nav_cross);
        self.filter = self.transaction_filter;
        self.predictor.apply_reanchor(&transform);
        self.context = staged_context;
        self.corrected_endpoint = staged_corrected_endpoint;
        self.predictor_endpoint = staged_predictor_endpoint;
        if let (Some(pending), Some(start)) = (
            self.pending_corrected_segment.as_mut(),
            staged_pending_start,
        ) {
            pending.start = start;
        }
        self.history.corrected.apply_reanchor(&transform);
        self.history.predicted.apply_reanchor(&transform);
        self.scheduler.for_each_measurement_mut(|scheduled| {
            scheduled.value = mapped_gnss(scheduled.value, &transform);
        });
        Ok(())
    }
}

pub(super) fn mapped_gnss(
    observation: GnssObservation,
    transform: &ReanchorTransform,
) -> GnssObservation {
    let mut result = observation;
    result.position_n = observation
        .position_n
        .map(|position| transform.map_position(position));
    result.velocity_n = observation
        .velocity_n
        .map(|velocity| transform.map_vector(velocity));
    result.position_covariance_n = transform.map_covariance(observation.position_covariance_n);
    result.velocity_covariance_n = transform.map_covariance(observation.velocity_covariance_n);
    result.position_velocity_cross_n = observation
        .position_velocity_cross_n
        .map(|cross| transform.new_n_from_old_n * cross * transform.new_n_from_old_n.transpose());
    result.shared_jacobians.position =
        transform.new_n_from_old_n * observation.shared_jacobians.position;
    result.shared_jacobians.velocity =
        transform.new_n_from_old_n * observation.shared_jacobians.velocity;
    result
}
