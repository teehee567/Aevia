//! Navigation initialization and restart, including antenna-to-IMU state conversion.

use super::configure::{make_initializer, make_live_core_config};
use super::conversion::{earth_rate_n, map_core_step_error, vector_f32};
use super::quality::heading_source;
use super::{InitializationFixEvidence, LiveSession};
use crate::error::{StepError, ValidationError};
use crate::live::{
    GnssInitializationFix, ImuInterval, InitializationPhase, LiveCore, LiveCoreSeed,
    NavConsiderCovariance,
};
use crate::observation::InputDisposition;
use crate::quality::{GnssState, Integrity};
use crate::time::{SessionTime, TimeSpan};
use nalgebra::{Matrix3, SMatrix, Vector3 as NaVector3};

#[cfg(test)]
#[path = "initialization_tests.rs"]
mod tests;

impl LiveSession<'_, '_> {
    pub(super) fn invalidate_navigation(&mut self) {
        let boundary = self
            .last_accepted_imu_end
            .or_else(|| self.psram.trajectory.span().map(TimeSpan::end))
            .unwrap_or(SessionTime::ZERO);
        self.invalidate_navigation_at(boundary);
    }

    pub(super) fn invalidate_navigation_at(&mut self, boundary: SessionTime) {
        self.psram
            .metric_tracker
            .begin_trajectory_reinitialization(boundary);
        self.psram
            .trajectory
            .clear_segments_preserving_reference_points();
        self.internal.core.reset();
        self.psram.history.clear();
        let mut initializer = make_initializer(&self.engine).ok();
        if let (Some(initializer), Some(heading)) = (initializer.as_mut(), self.initial_heading) {
            if initializer
                .provide_heading(heading.radians() as f32, heading.variance.get() as f32)
                .is_err()
            {
                // This was preflighted before the session started. If persistent
                // memory corruption nevertheless invalidates it, keep navigation
                // unavailable rather than returning an error after state changed.
                initializer.phase = InitializationPhase::Invalid;
            }
        }
        self.internal.initializer = initializer;
        self.last_accepted_imu_end = None;
        self.last_gnss_evidence = None;
        self.gnss_state = GnssState::Absent;
        self.integrity = Integrity::Unavailable;
        self.predictor_tracking_degraded = false;
        self.predictor_gap = false;
        self.predictor_degraded_input = false;
        self.diagnostics.reinitializations = self.diagnostics.reinitializations.saturating_add(1);
    }

    pub(super) fn ingest_initialization_imu(
        &mut self,
        interval: ImuInterval,
    ) -> Result<InputDisposition, StepError> {
        let mut initializer = *self
            .internal
            .initializer
            .as_ref()
            .ok_or(StepError::WorkspaceContract)?;
        if let Some(evidence) = self.latest_initialization_fix {
            if let Some(fix) = initialization_fix_at(
                evidence,
                interval.start,
                self.engine
                    .dynamics_profile
                    .gnss
                    .maximum_correction_age
                    .as_ns(),
            )? {
                initializer
                    .observe_gnss(fix)
                    .map_err(|_| StepError::EstimatorFailure)?;
            }
        }
        initializer
            .observe_imu(
                interval.start,
                self.engine
                    .dynamics_profile
                    .gnss
                    .maximum_correction_age
                    .as_ns(),
                interval.omega_ib_b,
                interval.specific_force_b,
            )
            .map_err(|_| StepError::EstimatorFailure)?;
        let Some(anchor) = self.anchor else {
            self.internal.initializer = Some(initializer);
            return Ok(InputDisposition::InitializationOnly);
        };
        let earth_rate = earth_rate_n(&anchor)?;
        let Some(mut result) = initializer
            .try_initialize(earth_rate)
            .map_err(|_| StepError::EstimatorFailure)?
        else {
            self.internal.initializer = Some(initializer);
            return Ok(InputDisposition::InitializationOnly);
        };

        let lever = vector_f32(
            self.engine
                .installation
                .imu_to_gnss_antenna
                .mean
                .components_m(),
        )?;
        let rotation_n_from_b = result
            .state
            .orientation_n_from_b
            .to_rotation_matrix()
            .into_inner();
        let omega_eb_b = transform_initial_antenna_state_to_imu(
            &mut result.state,
            &mut result.covariance,
            interval.omega_ib_b,
            interval.gyro_sample_covariance.to_matrix(),
            earth_rate,
            lever,
        )?;

        let consider_covariance = &self.internal.consider_seed_covariance;
        let mut state_consider_sensitivity = NavConsiderCovariance::zeros();
        let lever_start = usize::from(self.consider_layout.antenna_lever_start);
        state_consider_sensitivity
            .fixed_view_mut::<3, 3>(crate::live::state::POS, lever_start)
            .copy_from(&(-rotation_n_from_b));
        state_consider_sensitivity
            .fixed_view_mut::<3, 3>(crate::live::state::VEL, lever_start)
            .copy_from(&(-rotation_n_from_b * crate::live::state::skew(&omega_eb_b)));
        let clock_dt_s = result
            .state
            .time
            .checked_duration_since(self.clock_reference_time)
            .map_or_else(
                || {
                    self.clock_reference_time
                        .checked_duration_since(result.state.time)
                        .map(|duration| -(duration.as_seconds_f64() as f32))
                },
                |duration| Some(duration.as_seconds_f64() as f32),
            )
            .ok_or(StepError::InvalidObservation(ValidationError::TimeOverflow))?;
        state_consider_sensitivity
            .fixed_view_mut::<3, 1>(crate::live::state::POS, 0)
            .copy_from(&result.state.velocity_n);
        state_consider_sensitivity
            .fixed_view_mut::<3, 1>(crate::live::state::POS, 1)
            .copy_from(&(result.state.velocity_n * clock_dt_s));
        // Form P_xc once. Besides avoiding duplicate work, keeping the large
        // fixed consider covariance borrowed prevents a 4 KiB copy in the
        // S31 initialization frame.
        let nav_consider_covariance = &state_consider_sensitivity * consider_covariance;
        result.covariance += &nav_consider_covariance * state_consider_sensitivity.transpose();
        let initialized_heading_source = heading_source(result.heading_source);
        let nav_up_b = rotation_n_from_b.transpose() * NaVector3::z();
        let attitude_covariance = result
            .covariance
            .fixed_view::<3, 3>(crate::live::state::ATT, crate::live::state::ATT);
        let initialized_heading_variance_rad2 =
            nav_up_b.dot(&(attitude_covariance * nav_up_b)) as f64;
        if !initialized_heading_variance_rad2.is_finite() || initialized_heading_variance_rad2 < 0.0
        {
            return Err(StepError::EstimatorFailure);
        }
        let config = make_live_core_config(&self.engine, &anchor, self.imu_noise)?;
        let seed = LiveCoreSeed {
            initialization: &result,
            nav_consider_covariance: &nav_consider_covariance,
            consider_covariance,
            active_consider: usize::from(self.engine.navigation_profile.consider_dimension),
        };
        self.internal
            .core
            .initialize(&config, &seed, &self.psram.history)
            .map_err(map_core_step_error)?;
        let initial_ingest = {
            let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
            core.ingest_imu(interval)
                .and_then(|_| core.seed_initial_imu_sample(interval, lever))
        };
        if let Err(error) = initial_ingest {
            self.internal.core.reset();
            self.psram.history.clear();
            return Err(map_core_step_error(error));
        }
        self.heading_source = initialized_heading_source;
        self.heading_variance_rad2 = Some(initialized_heading_variance_rad2);
        self.internal.initializer = None;
        Ok(InputDisposition::Fused)
    }
}

fn initialization_fix_at(
    evidence: InitializationFixEvidence,
    time: SessionTime,
    maximum_age_ns: u64,
) -> Result<Option<GnssInitializationFix>, StepError> {
    let age_ns = time
        .as_ns()
        .checked_sub(evidence.position_epoch.as_ns())
        .ok_or(StepError::InvalidObservation(ValidationError::TimeOverflow))?;
    let velocity_age_ns = time
        .as_ns()
        .checked_sub(evidence.velocity_epoch.as_ns())
        .ok_or(StepError::InvalidObservation(ValidationError::TimeOverflow))?;
    if age_ns < 0
        || velocity_age_ns < 0
        || age_ns as u64 > maximum_age_ns
        || velocity_age_ns as u64 > maximum_age_ns
    {
        return Ok(None);
    }
    let dt = age_ns as f32 * 1.0e-9;
    let position_n = evidence.position_n + evidence.velocity_n * dt;
    let initial_position_velocity_cross = evidence
        .position_velocity_cross_n
        .unwrap_or_else(Matrix3::zeros);
    let mut position_covariance =
        evidence.position_covariance_n + evidence.velocity_covariance_n * (dt * dt);
    position_covariance +=
        (initial_position_velocity_cross + initial_position_velocity_cross.transpose()) * dt;
    let position_velocity_cross_n =
        initial_position_velocity_cross + evidence.velocity_covariance_n * dt;
    let timing_direction = evidence.velocity_n * evidence.position_independent_timing_sigma_s;
    position_covariance += timing_direction * timing_direction.transpose();
    let zero_velocity_nis = cholesky_nis(evidence.velocity_n, evidence.velocity_covariance_n);
    Ok(Some(GnssInitializationFix {
        time,
        evidence_oldest_time: core::cmp::min(evidence.position_epoch, evidence.velocity_epoch),
        position_n,
        velocity_n: evidence.velocity_n,
        position_covariance_n: position_covariance,
        velocity_covariance_n: evidence.velocity_covariance_n,
        position_velocity_cross_n,
        zero_velocity_nis,
    }))
}

/// Converts an initialization expressed at the GNSS antenna to the IMU
/// sensing centre. The covariance uses the complete first-order congruence
/// for position/velocity lever-arm coupling under the right-multiplicative
/// attitude convention, including gyro-bias and observed-rate uncertainty.
fn transform_initial_antenna_state_to_imu(
    state: &mut crate::live::NavState,
    covariance: &mut crate::live::state::NavMatrix,
    omega_ib_b: NaVector3<f32>,
    gyro_sample_covariance: Matrix3<f32>,
    earth_rate_n: NaVector3<f32>,
    lever_b: NaVector3<f32>,
) -> Result<NaVector3<f32>, StepError> {
    use crate::live::state::{ATT, GYRO_BIAS, NAV_DIM, POS, VEL, skew};

    let rotation_n_from_b = state.orientation_n_from_b.to_rotation_matrix().into_inner();
    let earth_rate_b = rotation_n_from_b.transpose() * earth_rate_n;
    let angular_rate_minus_bias = omega_ib_b - state.gyro_bias_b;
    let omega_eb_b = angular_rate_minus_bias - earth_rate_b;
    let next_position_n = state.position_n - rotation_n_from_b * lever_b;
    let next_velocity_n = state.velocity_n - rotation_n_from_b * omega_eb_b.cross(&lever_b);

    let position_attitude = rotation_n_from_b * skew(&lever_b);
    let velocity_attitude = rotation_n_from_b
        * (skew(&angular_rate_minus_bias.cross(&lever_b)) - skew(&earth_rate_b) * skew(&lever_b));
    let velocity_gyro_bias = -rotation_n_from_b * skew(&lever_b);

    // Only the first six state rows change. Forming their rectangular
    // transform keeps this initialization frame smaller than materializing a
    // second full 15-by-15 Jacobian on the MCU stack.
    let mut transformed_coordinate_rows = SMatrix::<f32, 6, NAV_DIM>::zeros();
    transformed_coordinate_rows
        .fixed_view_mut::<3, 3>(POS, POS)
        .copy_from(&Matrix3::identity());
    transformed_coordinate_rows
        .fixed_view_mut::<3, 3>(VEL, VEL)
        .copy_from(&Matrix3::identity());
    transformed_coordinate_rows
        .fixed_view_mut::<3, 3>(POS, ATT)
        .copy_from(&position_attitude);
    transformed_coordinate_rows
        .fixed_view_mut::<3, 3>(VEL, ATT)
        .copy_from(&velocity_attitude);
    transformed_coordinate_rows
        .fixed_view_mut::<3, 3>(VEL, GYRO_BIAS)
        .copy_from(&velocity_gyro_bias);

    let transformed_rows = transformed_coordinate_rows * *covariance;
    let mut transformed_front = transformed_rows * transformed_coordinate_rows.transpose();
    let gyro_to_velocity = rotation_n_from_b * skew(&lever_b);
    let velocity_covariance = transformed_front.fixed_view::<3, 3>(VEL, VEL).into_owned()
        + gyro_to_velocity * gyro_sample_covariance * gyro_to_velocity.transpose();
    transformed_front
        .fixed_view_mut::<3, 3>(VEL, VEL)
        .copy_from(&velocity_covariance);
    transformed_front = (transformed_front + transformed_front.transpose()) * 0.5;

    if !next_position_n
        .iter()
        .chain(next_velocity_n.iter())
        .chain(omega_eb_b.iter())
        .chain(transformed_rows.iter())
        .chain(transformed_front.iter())
        .all(|value| value.is_finite())
    {
        return Err(StepError::EstimatorFailure);
    }
    for row in 0..6 {
        for column in 0..6 {
            covariance[(row, column)] = transformed_front[(row, column)];
        }
        for column in 6..NAV_DIM {
            let value = transformed_rows[(row, column)];
            covariance[(row, column)] = value;
            covariance[(column, row)] = value;
        }
    }
    state.position_n = next_position_n;
    state.velocity_n = next_velocity_n;
    Ok(omega_eb_b)
}

fn cholesky_nis(value: NaVector3<f32>, covariance: Matrix3<f32>) -> Option<f32> {
    covariance
        .cholesky()
        .map(|factor| value.dot(&factor.solve(&value)))
        .filter(|value| value.is_finite() && *value >= 0.0)
}
