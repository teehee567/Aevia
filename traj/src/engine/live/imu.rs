//! IMU support validation, measurement preparation, and operational ingest failures.

use super::LiveSession;
use super::conversion::{covariance_density, map_core_step_error, vector_f32};
use super::gnss::timing_quality;
use crate::config::UncertaintyModelKind;
use crate::error::{StepError, ValidationError};
use crate::live::{CompactCovariance3, ImuInterval, LiveCore, LiveCoreError, LiveCoreInput};
use crate::observation::{ImuIntegrationEligibility, ImuObservation, InputDisposition};
use crate::time::{SampleSupport, SessionTime};
use crate::uncertainty::{Covariance3, MeasurementUncertainty};
use nalgebra::{Rotation3, UnitQuaternion as NaUnitQuaternion};

impl LiveSession<'_, '_> {
    #[inline(never)]
    pub(super) fn ingest_imu(
        &mut self,
        observation: ImuObservation,
    ) -> Result<InputDisposition, StepError> {
        if observation.profile() != self.engine.input_profile.id
            || observation.measurement_frame() != self.engine.installation.imu_sensor_frame
        {
            return Err(StepError::FrameMismatch);
        }
        let rate_time = observation.angular_rate().time;
        let force_time = observation.specific_force().time;
        if rate_time.clock_model != force_time.clock_model {
            return Err(StepError::ClockDiscontinuity);
        }
        let rate_effective_time = rate_time
            .effective_time()
            .map_err(StepError::InvalidObservation)?;
        let eligibility = observation.integration_eligibility();
        if observation.breaks_continuity() {
            self.validate_point_clock_model(rate_time.clock_model, rate_effective_time)?;
            if self.internal.core.is_active() {
                self.invalidate_navigation();
            }
            self.diagnostics.imu_epochs_rejected =
                self.diagnostics.imu_epochs_rejected.saturating_add(1);
            return Ok(
                if matches!(eligibility, ImuIntegrationEligibility::RejectInitialization) {
                    InputDisposition::InitializationOnly
                } else {
                    InputDisposition::RetainedForOffline
                },
            );
        }
        if matches!(
            eligibility,
            ImuIntegrationEligibility::RejectUnavailable
                | ImuIntegrationEligibility::RejectAngularRate
                | ImuIntegrationEligibility::RejectSpecificForce
        ) {
            self.validate_point_clock_model(rate_time.clock_model, rate_effective_time)?;
            // An incomplete vector is a missing IMU epoch, not proof that the
            // previously qualified navigation state is invalid. Preserve the
            // last accepted support boundary so the next complete interval
            // either receives the bounded synthetic bridge or triggers the
            // declared overlong-gap reinitialization below.
            self.current_clock_model
                .get_or_insert(rate_time.clock_model);
            self.diagnostics.imu_epochs_rejected =
                self.diagnostics.imu_epochs_rejected.saturating_add(1);
            return Ok(InputDisposition::RetainedForOffline);
        }
        let (interval, eligibility) = self.prepared_imu_interval(observation)?;
        let Some(interval) = interval else {
            self.validate_point_clock_model(rate_time.clock_model, rate_effective_time)?;
            self.current_clock_model
                .get_or_insert(rate_time.clock_model);
            self.diagnostics.imu_epochs_rejected =
                self.diagnostics.imu_epochs_rejected.saturating_add(1);
            return Ok(eligibility);
        };
        self.prepare_imu_clock_boundary(rate_time.clock_model, interval)?;
        if !self.clock_uncertainty_valid {
            self.diagnostics.imu_epochs_rejected =
                self.diagnostics.imu_epochs_rejected.saturating_add(1);
            return Ok(InputDisposition::RetainedForOffline);
        }

        let mut restarted_for_gap = false;
        if let Some(previous_end) = self.last_accepted_imu_end {
            let gap_ns = interval
                .start
                .as_ns()
                .checked_sub(previous_end.as_ns())
                .ok_or(StepError::InvalidObservation(ValidationError::TimeOverflow))?;
            if gap_ns
                > i64::try_from(
                    self.engine
                        .navigation_profile
                        .maximum_bridgeable_imu_gap
                        .as_ns(),
                )
                .map_err(|_| StepError::WorkspaceContract)?
                && self.internal.core.is_active()
            {
                self.invalidate_navigation();
                restarted_for_gap = true;
            }
        }

        let disposition = if self.internal.core.is_active() {
            let ingest = {
                let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
                core.ingest(LiveCoreInput::Imu(interval))
            };
            match ingest {
                Ok(_) => InputDisposition::Fused,
                Err(error) if is_operational_imu_ingest_failure(error) => {
                    // The semantic observation and source sequence are valid,
                    // but the live runtime could not retain or propagate it
                    // within its fixed contract. Make the state loss explicit
                    // without rolling acquisition history back as though the
                    // observation had never arrived.
                    self.invalidate_navigation();
                    self.current_clock_model
                        .get_or_insert(rate_time.clock_model);
                    self.diagnostics.imu_epochs_rejected =
                        self.diagnostics.imu_epochs_rejected.saturating_add(1);
                    return Ok(InputDisposition::RetainedForOffline);
                }
                Err(error) => return Err(map_core_step_error(error)),
            }
        } else {
            match self.ingest_initialization_imu(interval) {
                Ok(disposition) => disposition,
                Err(_) if restarted_for_gap => {
                    // The overlong gap already committed an operational
                    // navigation reset. A numerical failure while seeding the
                    // replacement initializer cannot be surfaced as a
                    // transactional contract error for this accepted input.
                    self.diagnostics.imu_epochs_rejected =
                        self.diagnostics.imu_epochs_rejected.saturating_add(1);
                    self.psram.history.clear();
                    self.current_clock_model
                        .get_or_insert(rate_time.clock_model);
                    return Ok(InputDisposition::RetainedForOffline);
                }
                Err(error) => return Err(error),
            }
        };
        self.current_clock_model
            .get_or_insert(rate_time.clock_model);
        self.last_accepted_imu_end = Some(interval.end);
        if self.clock_uncertainty_valid {
            self.timing_quality = timing_quality(rate_time);
        }
        self.diagnostics.imu_epochs_accepted =
            self.diagnostics.imu_epochs_accepted.saturating_add(1);
        Ok(disposition)
    }

    pub(super) fn prepared_imu_interval(
        &self,
        observation: ImuObservation,
    ) -> Result<(Option<ImuInterval>, InputDisposition), StepError> {
        match observation.integration_eligibility() {
            ImuIntegrationEligibility::Complete => {}
            ImuIntegrationEligibility::RejectInitialization => {
                return Ok((None, InputDisposition::InitializationOnly));
            }
            ImuIntegrationEligibility::RejectDiscontinuity
            | ImuIntegrationEligibility::RejectUnavailable
            | ImuIntegrationEligibility::RejectAngularRate
            | ImuIntegrationEligibility::RejectSpecificForce => {
                return Ok((None, InputDisposition::RetainedForOffline));
            }
        }
        let angular = observation.angular_rate();
        let force = observation.specific_force();
        if angular.time.independent_one_sigma.as_ns() != 0
            || force.time.independent_one_sigma.as_ns() != 0
        {
            // Applying interval averages at an uncertain support epoch needs
            // neighboring temporal derivatives/resampling. This implementation
            // cannot turn timestamp jitter into independent value noise. Keep
            // the evidence for a future processing path that supports it.
            return Ok((None, InputDisposition::RetainedForOffline));
        }
        let end = angular
            .time
            .effective_time()
            .map_err(StepError::InvalidObservation)?;
        if force
            .time
            .effective_time()
            .map_err(StepError::InvalidObservation)?
            != end
            || force.time.clock_model != angular.time.clock_model
        {
            return Err(StepError::InvalidObservation(
                ValidationError::InvalidTimeSpan,
            ));
        }
        let duration = match (angular.time.support, force.time.support) {
            (
                SampleSupport::IntervalAverage { duration: left },
                SampleSupport::IntervalAverage { duration: right },
            ) if left == right && left.as_ns() > 0 => left,
            _ => {
                return Err(StepError::InvalidObservation(
                    ValidationError::IncompatibleDefinition,
                ));
            }
        };
        let duration_i64 = i64::try_from(duration.as_ns())
            .map_err(|_| StepError::InvalidObservation(ValidationError::TimeOutOfRange))?;
        let start = end
            .as_ns()
            .checked_sub(duration_i64)
            .map(SessionTime::from_ns)
            .ok_or(StepError::InvalidObservation(ValidationError::TimeOverflow))?;

        let force_sensor = force.value.components();
        let force_sample_covariance =
            covariance_density(self.resolve_imu_covariance(force.uncertainty)?)?;
        let rate_sensor = angular.value.components();
        let body_from_sensor = self.body_from_imu;
        let rate_sensor_vector = vector_f32(rate_sensor)?;
        let force_sensor_vector = vector_f32(force_sensor)?;
        let omega = body_from_sensor * rate_sensor_vector;
        let specific_force = body_from_sensor * force_sensor_vector;
        let gyro_sample_covariance =
            covariance_density(self.resolve_imu_covariance(angular.uncertainty)?)?;
        let accel_sample_covariance_body =
            body_from_sensor * force_sample_covariance * body_from_sensor.transpose();
        let accel_sample_covariance = CompactCovariance3::from_matrix(
            (accel_sample_covariance_body + accel_sample_covariance_body.transpose()) * 0.5,
        )
        .map_err(|_| StepError::InvalidObservation(ValidationError::InvalidCovariance))?;
        let gyro_sample_covariance_body =
            body_from_sensor * gyro_sample_covariance * body_from_sensor.transpose();
        let gyro_sample_covariance = CompactCovariance3::from_matrix(
            (gyro_sample_covariance_body + gyro_sample_covariance_body.transpose()) * 0.5,
        )
        .map_err(|_| StepError::InvalidObservation(ValidationError::InvalidCovariance))?;
        let body_from_sensor_quaternion = NaUnitQuaternion::from_rotation_matrix(
            &Rotation3::from_matrix_unchecked(body_from_sensor),
        );
        let calibration_definition = self
            .engine
            .calibration
            .shared_parameters
            .definition(self.engine.installation.body_from_imu.parameter_id)
            .ok_or(StepError::WorkspaceContract)?;
        if !calibration_definition.validity.contains(start)
            || !calibration_definition.validity.contains(end)
        {
            return Err(StepError::InvalidObservation(
                ValidationError::IncompatibleDefinition,
            ));
        }
        Ok((
            Some(ImuInterval {
                start,
                end,
                omega_ib_b: omega,
                specific_force_b: specific_force,
                degraded_input: observation.is_degraded(),
                gap_elapsed_ns_plus_one: 0,
                body_from_sensor: body_from_sensor_quaternion,
                accel_sample_covariance,
                gyro_sample_covariance,
                calibration_consider_start: Some(self.consider_layout.imu_boresight_start),
            }),
            InputDisposition::Fused,
        ))
    }

    fn resolve_imu_covariance(
        &self,
        uncertainty: MeasurementUncertainty<Covariance3>,
    ) -> Result<Covariance3, StepError> {
        match uncertainty {
            MeasurementUncertainty::Provided(value) => Ok(value),
            MeasurementUncertainty::Modeled(id) => self
                .engine
                .uncertainty_models
                .iter()
                .find(|model| model.id == id)
                .and_then(|model| match model.kind {
                    UncertaintyModelKind::ConstantCovariance3(value) => Some(value),
                    UncertaintyModelKind::ConstantVariance(_)
                    | UncertaintyModelKind::SequenceBound { .. } => None,
                })
                .ok_or(StepError::InvalidObservation(
                    ValidationError::InvalidCovariance,
                )),
        }
    }
}

fn is_operational_imu_ingest_failure(error: LiveCoreError) -> bool {
    !matches!(
        error,
        LiveCoreError::InputClosed
            | LiveCoreError::MeasurementTimeMismatch
            | LiveCoreError::ImuOverlapOrRegression
            | LiveCoreError::MissingInitialImuSupport
            | LiveCoreError::ImuIntervalTooLong
    )
}
