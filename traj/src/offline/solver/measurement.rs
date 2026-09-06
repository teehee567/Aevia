//! GNSS updates, receiver health, and lever-arm and timing sensitivities.

use crate::{
    config::{EngineConfig, GnssCorrelationPolicy, SharedParameterKind, UncertaintyModelKind},
    error::ProcessError,
    frame::ReferencePointKind,
    observation::{GnssSolutionObservation, InputDisposition, ReceiverHealth},
    offline::store::StoredNominal,
    quality::{GnssState, TimingQuality},
    time::{ObservationTime, SessionTime, TimingBasis},
    uncertainty::{Covariance3, MeasurementUncertainty},
};

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use super::{
    catalog::OwnedClockModel,
    estimation::{matrix_is_psd, schmidt_update_affine_with_sample},
    filter::{
        ActiveImuSample, AngularAccelerationEstimate, HeldImu, OfflineFilter, VelocityTimingModel,
    },
    inertial::{ensure_imu_support_is_contiguous, predicted_acceleration},
    initialization::antenna_velocity_attitude_jacobian,
    math::{
        ATTITUDE, EARTH_RATE_RAD_S, GYROSCOPE_BIAS, NAVIGATION_DIMENSION, POSITION, VELOCITY,
        dvector3, matrix3_from_array, set_dynamic_matrix3, set_identity3, set_matrix3,
        set_rect_matrix3, set_vector3, skew, symmetric3, vector3,
    },
};

impl<'a> OfflineFilter<'a> {
    pub(super) fn update_gnss(
        &mut self,
        solution: GnssSolutionObservation,
        field: GnssField,
        time: SessionTime,
        guide: Option<&StoredNominal>,
    ) -> Result<MeasurementOutcome, ProcessError> {
        if should_reject_solution(solution, field, self.config) {
            return Ok(MeasurementOutcome::rejected(self.state_dimension));
        }
        let current_nominal = self.nominal.clone().ok_or(ProcessError::InvalidEvidence)?;
        let mut linearization_nominal = if self.relinearized {
            let guide = guide
                .or(self.guide_nominal.as_ref())
                .ok_or(ProcessError::NumericalNonConvergence)?;
            if guide.time != time {
                return Err(ProcessError::NumericalNonConvergence);
            }
            guide.clone()
        } else {
            current_nominal
        };
        if let Some(imu) = &self.held_imu {
            super::inertial::refresh_inertial_kinematics(&mut linearization_nominal, imu)?;
        }
        let nominal = &linearization_nominal;
        let lever = self
            .config
            .installation
            .imu_to_gnss_antenna
            .mean
            .components_m();
        let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
        let lever_ecef = rotation * vector3(lever);
        let omega_body = vector3(nominal.angular_rate_body);
        let lever_body = vector3(lever);
        let earth_rate_body = rotation.transpose() * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
        let omega_cross_lever = omega_body.cross(&lever_body);
        let rotational_velocity = rotation * omega_cross_lever;
        let antenna_position = vector3(nominal.position_ecef) + lever_ecef;
        let antenna_velocity = vector3(nominal.velocity_ecef) + rotational_velocity;
        let velocity_attitude_jacobian =
            antenna_velocity_attitude_jacobian(&rotation, omega_body, earth_rate_body, lever_body);

        let (rows, mut residual, h_state, h_consider, h_sample, mut noise) = match field {
            GnssField::Position => {
                let position = solution.position().ok_or(ProcessError::InvalidEvidence)?;
                ensure_frame(position.frame, self.config)?;
                let clock = self.clock_for(position.time, time)?;
                let predicted = antenna_position
                    + if self.colored_error {
                        vector3(nominal.colored_gnss_error)
                    } else {
                        Vector3::zeros()
                    };
                let residual = vector3(position.value.components()) - predicted;
                let mut h_state = DMatrix::zeros(3, self.state_dimension);
                set_identity3(&mut h_state, 0, POSITION);
                set_matrix3(
                    &mut h_state,
                    0,
                    ATTITUDE,
                    &(-rotation * skew(&vector3(lever))),
                );
                if self.colored_error {
                    set_identity3(&mut h_state, 0, NAVIGATION_DIMENSION);
                }
                let mut h_consider = DMatrix::zeros(3, self.catalog.covariance.nrows());
                let h_sample = DMatrix::zeros(3, 6);
                self.add_lever_sensitivity(&mut h_consider, 0, &rotation);
                add_clock_sensitivity(&mut h_consider, 0, clock, time, antenna_velocity)?;
                self.add_delay_sensitivity(&mut h_consider, 0, time, antenna_velocity);
                let noise = measurement_noise(
                    self.config,
                    position.uncertainty,
                    self.config.dynamics_profile.gnss.position_covariance_floor,
                    position.time,
                    antenna_velocity,
                )?;
                (3, dvector3(residual), h_state, h_consider, h_sample, noise)
            }
            GnssField::Velocity => {
                let velocity = solution.velocity().ok_or(ProcessError::InvalidEvidence)?;
                ensure_frame(velocity.frame, self.config)?;
                let clock = self.clock_for(velocity.time, time)?;
                let timing_model =
                    self.velocity_timing_model(nominal, velocity.time, time, clock)?;
                let acceleration = timing_model.antenna_acceleration_ecef;
                let residual = vector3(velocity.value.components()) - antenna_velocity;
                let mut h_state = DMatrix::zeros(3, self.state_dimension);
                set_identity3(&mut h_state, 0, VELOCITY);
                set_matrix3(&mut h_state, 0, ATTITUDE, &velocity_attitude_jacobian);
                set_matrix3(
                    &mut h_state,
                    0,
                    GYROSCOPE_BIAS,
                    &(rotation * skew(&vector3(lever))),
                );
                let mut h_consider = DMatrix::zeros(3, self.catalog.covariance.nrows());
                let mut h_sample = DMatrix::zeros(3, 6);
                set_rect_matrix3(&mut h_sample, 0, 3, &(rotation * skew(&lever_body)));
                self.add_velocity_lever_sensitivity(
                    &mut h_consider,
                    0,
                    &rotation,
                    nominal.angular_rate_body,
                );
                add_clock_sensitivity(&mut h_consider, 0, clock, time, acceleration)?;
                self.add_delay_sensitivity(&mut h_consider, 0, time, acceleration);
                let mut noise = measurement_noise(
                    self.config,
                    velocity.uncertainty,
                    self.config.dynamics_profile.gnss.velocity_covariance_floor,
                    velocity.time,
                    acceleration,
                )?;
                let extra = timing_model.timing_sensitivity_covariance_ecef
                    + timing_model.angular_rate_prediction_covariance_ecef;
                for row in 0..3 {
                    for column in 0..3 {
                        noise[(row, column)] += extra[(row, column)];
                    }
                }
                (3, dvector3(residual), h_state, h_consider, h_sample, noise)
            }
            GnssField::Joint => {
                let position = solution.position().ok_or(ProcessError::InvalidEvidence)?;
                let velocity = solution.velocity().ok_or(ProcessError::InvalidEvidence)?;
                ensure_frame(position.frame, self.config)?;
                ensure_frame(velocity.frame, self.config)?;
                let position_clock = self.clock_for(position.time, time)?;
                let velocity_clock = self.clock_for(velocity.time, time)?;
                let timing_model =
                    self.velocity_timing_model(nominal, velocity.time, time, velocity_clock)?;
                let acceleration = timing_model.antenna_acceleration_ecef;
                let predicted_position = antenna_position
                    + if self.colored_error {
                        vector3(nominal.colored_gnss_error)
                    } else {
                        Vector3::zeros()
                    };
                let position_residual = vector3(position.value.components()) - predicted_position;
                let velocity_residual = vector3(velocity.value.components()) - antenna_velocity;
                let mut residual = DVector::zeros(6);
                set_vector3(&mut residual, 0, position_residual);
                set_vector3(&mut residual, 3, velocity_residual);
                let mut h_state = DMatrix::zeros(6, self.state_dimension);
                set_identity3(&mut h_state, 0, POSITION);
                set_identity3(&mut h_state, 3, VELOCITY);
                set_matrix3(
                    &mut h_state,
                    0,
                    ATTITUDE,
                    &(-rotation * skew(&vector3(lever))),
                );
                set_matrix3(&mut h_state, 3, ATTITUDE, &velocity_attitude_jacobian);
                set_matrix3(
                    &mut h_state,
                    3,
                    GYROSCOPE_BIAS,
                    &(rotation * skew(&vector3(lever))),
                );
                if self.colored_error {
                    set_identity3(&mut h_state, 0, NAVIGATION_DIMENSION);
                }
                let mut h_consider = DMatrix::zeros(6, self.catalog.covariance.nrows());
                let mut h_sample = DMatrix::zeros(6, 6);
                set_rect_matrix3(&mut h_sample, 3, 3, &(rotation * skew(&lever_body)));
                self.add_lever_sensitivity(&mut h_consider, 0, &rotation);
                self.add_velocity_lever_sensitivity(
                    &mut h_consider,
                    3,
                    &rotation,
                    nominal.angular_rate_body,
                );
                add_clock_sensitivity(&mut h_consider, 0, position_clock, time, antenna_velocity)?;
                add_clock_sensitivity(&mut h_consider, 3, velocity_clock, time, acceleration)?;
                self.add_delay_sensitivity(&mut h_consider, 0, time, antenna_velocity);
                self.add_delay_sensitivity(&mut h_consider, 3, time, acceleration);
                let position_noise = measurement_noise(
                    self.config,
                    position.uncertainty,
                    self.config.dynamics_profile.gnss.position_covariance_floor,
                    position.time,
                    antenna_velocity,
                )?;
                let velocity_noise = measurement_noise(
                    self.config,
                    velocity.uncertainty,
                    self.config.dynamics_profile.gnss.velocity_covariance_floor,
                    velocity.time,
                    acceleration,
                )?;
                let mut noise = DMatrix::zeros(6, 6);
                set_dynamic_matrix3(&mut noise, 0, 0, &position_noise);
                set_dynamic_matrix3(&mut noise, 3, 3, &velocity_noise);
                let extra = timing_model.timing_sensitivity_covariance_ecef
                    + timing_model.angular_rate_prediction_covariance_ecef;
                for row in 0..3 {
                    for column in 0..3 {
                        noise[(3 + row, 3 + column)] += extra[(row, column)];
                    }
                }
                let cross = solution
                    .position_velocity_cross_covariance()
                    .ok_or(ProcessError::InvalidEvidence)?;
                let cross = matrix3_from_array(cross.to_matrix());
                set_matrix3(&mut noise, 0, 3, &cross);
                set_matrix3(&mut noise, 3, 0, &cross.transpose());
                (6, residual, h_state, h_consider, h_sample, noise)
            }
        };
        if rows != residual.len()
            || h_state.nrows() != rows
            || h_consider.nrows() != rows
            || h_sample.shape() != (rows, 6)
            || noise.shape() != (rows, rows)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        apply_correlation_policy(self.config.dynamics_profile.gnss.correlation, &mut noise);
        let covariance = self
            .covariance
            .as_mut()
            .ok_or(ProcessError::InvalidEvidence)?;
        let result = schmidt_update_affine_with_sample(
            self.nominal.as_mut().ok_or(ProcessError::InvalidEvidence)?,
            covariance,
            self.active_imu_sample
                .as_mut()
                .ok_or(ProcessError::IncompleteEvidence)?,
            &h_state,
            &h_consider,
            &h_sample,
            &self.catalog.covariance,
            &mut residual,
            &noise,
            self.config
                .dynamics_profile
                .gnss
                .robust_weight_threshold
                .get(),
            self.config
                .dynamics_profile
                .gnss
                .nis_rejection_threshold
                .get(),
            self.config
                .dynamics_profile
                .gnss
                .maximum_covariance_inflation
                .get(),
            self.config
                .navigation_profile
                .covariance_repair
                .maximum_attempts,
            self.config
                .navigation_profile
                .covariance_repair
                .maximum_total_regularization
                .get(),
            &mut self.diagnostics.covariance_repairs,
            self.relinearized.then_some(nominal),
            self.damping,
        )?;
        if let (Some(nominal), Some(imu)) = (self.nominal.as_mut(), self.held_imu.as_ref()) {
            super::inertial::refresh_inertial_kinematics(nominal, imu)?;
        }
        if matches!(
            result.disposition,
            InputDisposition::Fused | InputDisposition::Downweighted
        ) {
            self.gnss_state = gnss_state(receiver_is_healthy(solution, field, self.config));
            self.timing_quality = match field {
                GnssField::Position | GnssField::Joint => timing_quality(
                    solution
                        .position()
                        .ok_or(ProcessError::InvalidEvidence)?
                        .time
                        .basis,
                ),
                GnssField::Velocity => timing_quality(
                    solution
                        .velocity()
                        .ok_or(ProcessError::InvalidEvidence)?
                        .time
                        .basis,
                ),
            };
        }
        Ok(result)
    }

    pub(super) fn clock_for(
        &self,
        timing: ObservationTime,
        effective_time: SessionTime,
    ) -> Result<&OwnedClockModel, ProcessError> {
        self.catalog
            .clock(timing.clock_model, effective_time)
            .ok_or(ProcessError::IncompleteEvidence)
    }

    pub(super) fn angular_acceleration_estimate(
        &self,
        nominal: &StoredNominal,
    ) -> Result<Option<AngularAccelerationEstimate>, ProcessError> {
        let (Some(previous), Some(current)) = (self.previous_imu.as_ref(), self.held_imu.as_ref())
        else {
            return Ok(None);
        };
        ensure_imu_support_is_contiguous(previous, current)?;
        if nominal.time < current.start || nominal.time > current.time {
            return Err(ProcessError::InvalidEvidence);
        }
        let previous_duration = current
            .start
            .checked_duration_since(previous.start)
            .ok_or(ProcessError::InvalidEvidence)?
            .as_seconds_f64();
        let current_duration = current
            .time
            .checked_duration_since(current.start)
            .ok_or(ProcessError::InvalidEvidence)?
            .as_seconds_f64();
        // Contiguous supports make `current.start - previous.start` exactly
        // the previous interval duration. Interval-average rates belong at
        // their support centres, so their representative-epoch separation is
        // half the sum of the two durations.
        let centre_separation = 0.5 * (previous_duration + current_duration);
        if !previous_duration.is_finite()
            || previous_duration <= 0.0
            || !current_duration.is_finite()
            || current_duration <= 0.0
            || !centre_separation.is_finite()
            || centre_separation <= 0.0
        {
            return Err(ProcessError::InvalidEvidence);
        }

        let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
        let earth_rate_body = rotation.transpose() * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
        let omega_eb_body = vector3(current.angular_rate_body)
            - vector3(nominal.gyroscope_bias_body)
            - earth_rate_body;
        let raw_rate_derivative = (vector3(current.angular_rate_body)
            - vector3(previous.angular_rate_body))
            / centre_separation;
        // alpha_eb^b is the derivative of the Earth-relative rate components
        // expressed in body coordinates, not merely a raw gyro difference.
        let mean_body = raw_rate_derivative + omega_eb_body.cross(&earth_rate_body);

        let density = matrix3_from_array(
            self.config
                .dynamics_profile
                .process_noise
                .gyroscope
                .to_matrix(),
        );
        let previous_rate_covariance = previous.gyroscope_covariance + density / previous_duration;
        let current_rate_covariance = current.gyroscope_covariance + density / current_duration;
        let covariance_body = symmetric3(
            (previous_rate_covariance + current_rate_covariance) / centre_separation.powi(2),
        );
        let covariance_dynamic = DMatrix::from_column_slice(3, 3, covariance_body.as_slice());
        if !mean_body.iter().all(|value| value.is_finite()) || !matrix_is_psd(&covariance_dynamic) {
            return Err(ProcessError::NumericalNonConvergence);
        }
        Ok(Some(AngularAccelerationEstimate {
            mean_body,
            covariance_body,
        }))
    }

    pub(super) fn install_active_imu_sample(
        &mut self,
        imu: &HeldImu,
        state_cross: Option<DMatrix<f64>>,
    ) -> Result<(), ProcessError> {
        let mut covariance_body = DMatrix::zeros(6, 6);
        set_matrix3(&mut covariance_body, 0, 0, &imu.accelerometer_covariance);
        set_matrix3(&mut covariance_body, 3, 3, &imu.gyroscope_covariance);
        if !matrix_is_psd(&covariance_body) {
            return Err(ProcessError::InvalidEvidence);
        }
        let state_cross = state_cross.unwrap_or_else(|| DMatrix::zeros(self.state_dimension, 6));
        if state_cross.shape() != (self.state_dimension, 6)
            || !state_cross.iter().all(|value| value.is_finite())
        {
            return Err(ProcessError::InvalidEvidence);
        }
        if let Some(nominal) = self.nominal.as_mut() {
            nominal.imu_sample_error_body = [0.0; 6];
        }
        self.active_imu_sample = Some(ActiveImuSample {
            start: imu.start,
            end: imu.time,
            covariance_body,
            state_cross: state_cross.clone(),
            consider_cross: DMatrix::zeros(6, self.catalog.covariance.nrows()),
        });
        if self
            .nominal
            .as_ref()
            .is_some_and(|nominal| self.last_stored_time == Some(nominal.time))
        {
            self.sample_influence_accumulator.fill(0.0);
            self.last_stored_sample_cross = state_cross;
        }
        Ok(())
    }

    pub(super) fn shared_timing_variance(
        &self,
        clock: &OwnedClockModel,
        time: SessionTime,
    ) -> Result<f64, ProcessError> {
        let elapsed = time
            .checked_duration_since(clock.reference_time)
            .ok_or(ProcessError::InvalidEvidence)?
            .as_seconds_f64();
        let mut weights = DVector::zeros(self.catalog.covariance.nrows());
        weights[clock.offset_index] = 1.0e-9;
        weights[clock.offset_index + 1] = elapsed;
        for parameter in self.catalog.parameters.iter().filter(|parameter| {
            parameter.kind == SharedParameterKind::DelayNs && parameter.validity.contains(time)
        }) {
            for coordinate in 0..parameter.dimension {
                weights[parameter.start + coordinate] += 1.0e-9;
            }
        }
        let variance = (weights.transpose() * &self.catalog.covariance * &weights)[(0, 0)];
        if !variance.is_finite() {
            return Err(ProcessError::NumericalNonConvergence);
        }
        let scale = self.catalog.covariance.norm().max(1.0) * weights.norm_squared().max(1.0);
        if variance < -128.0 * f64::EPSILON * scale {
            return Err(ProcessError::InvalidEvidence);
        }
        Ok(variance.max(0.0))
    }

    pub(super) fn velocity_timing_model(
        &self,
        nominal: &StoredNominal,
        timing: ObservationTime,
        time: SessionTime,
        clock: &OwnedClockModel,
    ) -> Result<VelocityTimingModel, ProcessError> {
        let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
        let lever = vector3(
            self.config
                .installation
                .imu_to_gnss_antenna
                .mean
                .components_m(),
        );
        let omega_eb_body = vector3(nominal.angular_rate_body);
        let independent_variance = timing.independent_one_sigma.as_seconds_f64().powi(2);
        let shared_variance = self.shared_timing_variance(clock, time)?;
        let total_timing_variance = independent_variance + shared_variance;
        let needs_alpha = lever.norm_squared() > 0.0 && total_timing_variance > 0.0;
        let alpha = if needs_alpha {
            self.angular_acceleration_estimate(nominal)?
                .ok_or(ProcessError::IncompleteEvidence)?
        } else {
            AngularAccelerationEstimate {
                mean_body: Vector3::zeros(),
                covariance_body: Matrix3::zeros(),
            }
        };
        let tangential_map = -rotation * skew(&lever);
        let tangential_covariance =
            symmetric3(tangential_map * alpha.covariance_body * tangential_map.transpose());
        let antenna_acceleration_ecef = predicted_acceleration(nominal, self.config)?
            + rotation
                * (alpha.mean_body.cross(&lever)
                    + omega_eb_body.cross(&omega_eb_body.cross(&lever)));

        let current = self
            .held_imu
            .as_ref()
            .ok_or(ProcessError::IncompleteEvidence)?;
        if nominal.time < current.start || nominal.time > current.time {
            return Err(ProcessError::IncompleteEvidence);
        }
        let duration = current
            .time
            .checked_duration_since(current.start)
            .ok_or(ProcessError::InvalidEvidence)?
            .as_seconds_f64();
        if !duration.is_finite() || duration <= 0.0 {
            return Err(ProcessError::InvalidEvidence);
        }
        let gyro_density = matrix3_from_array(
            self.config
                .dynamics_profile
                .process_noise
                .gyroscope
                .to_matrix(),
        );
        // The interval-average observation covariance is represented by the
        // persistent sample latent and its direct measurement Jacobian. Only
        // the independent continuous profile component belongs in `R` here.
        let rate_covariance = gyro_density / duration;
        let rate_to_velocity = rotation * skew(&lever);
        let angular_rate_prediction_covariance_ecef =
            symmetric3(rate_to_velocity * rate_covariance * rate_to_velocity.transpose());
        if !antenna_acceleration_ecef
            .iter()
            .chain(tangential_covariance.iter())
            .chain(angular_rate_prediction_covariance_ecef.iter())
            .all(|value| value.is_finite())
        {
            return Err(ProcessError::NumericalNonConvergence);
        }
        Ok(VelocityTimingModel {
            antenna_acceleration_ecef,
            timing_sensitivity_covariance_ecef: tangential_covariance * total_timing_variance,
            angular_rate_prediction_covariance_ecef,
        })
    }

    pub(super) fn add_lever_sensitivity(
        &self,
        target: &mut DMatrix<f64>,
        row: usize,
        rotation: &Matrix3<f64>,
    ) {
        if let Some(parameter) = self
            .catalog
            .parameter(self.config.installation.imu_to_gnss_antenna.parameter_id)
        {
            let dimension = parameter.dimension.min(3);
            for axis in 0..3 {
                for coordinate in 0..dimension {
                    target[(row + axis, parameter.start + coordinate)] +=
                        rotation[(axis, coordinate)];
                }
            }
        }
    }

    pub(super) fn add_velocity_lever_sensitivity(
        &self,
        target: &mut DMatrix<f64>,
        row: usize,
        rotation: &Matrix3<f64>,
        angular_rate_body: [f64; 3],
    ) {
        if let Some(parameter) = self
            .catalog
            .parameter(self.config.installation.imu_to_gnss_antenna.parameter_id)
        {
            let derivative = rotation * skew(&vector3(angular_rate_body));
            let dimension = parameter.dimension.min(3);
            for axis in 0..3 {
                for coordinate in 0..dimension {
                    target[(row + axis, parameter.start + coordinate)] +=
                        derivative[(axis, coordinate)];
                }
            }
        }
    }

    pub(super) fn add_delay_sensitivity(
        &self,
        target: &mut DMatrix<f64>,
        row: usize,
        time: SessionTime,
        temporal_sensitivity: Vector3<f64>,
    ) {
        for parameter in self.catalog.parameters.iter().filter(|parameter| {
            parameter.kind == SharedParameterKind::DelayNs && parameter.validity.contains(time)
        }) {
            for coordinate in 0..parameter.dimension {
                for axis in 0..3 {
                    target[(row + axis, parameter.start + coordinate)] +=
                        temporal_sensitivity[axis] * 1.0e-9;
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(super) enum GnssField {
    Position,
    Velocity,
    Joint,
}

pub(super) struct MeasurementOutcome {
    pub(super) disposition: InputDisposition,
    pub(super) objective: f64,
    pub(super) reset_basis: DMatrix<f64>,
}

impl MeasurementOutcome {
    pub(super) fn rejected(state_dimension: usize) -> Self {
        Self {
            disposition: InputDisposition::StatisticallyRejected,
            objective: 0.0,
            reset_basis: DMatrix::identity(state_dimension, state_dimension),
        }
    }
}

pub(super) fn resolve_covariance(
    config: &EngineConfig<'_>,
    uncertainty: MeasurementUncertainty<Covariance3>,
) -> Result<Matrix3<f64>, ProcessError> {
    match uncertainty {
        MeasurementUncertainty::Provided(value) => Ok(matrix3_from_array(value.to_matrix())),
        MeasurementUncertainty::Modeled(id) => {
            let model = config
                .uncertainty_models
                .iter()
                .find(|model| model.id == id)
                .ok_or(ProcessError::IncompleteEvidence)?;
            match model.kind {
                UncertaintyModelKind::ConstantCovariance3(value) => {
                    Ok(matrix3_from_array(value.to_matrix()))
                }
                UncertaintyModelKind::ConstantVariance(value) => {
                    Ok(Matrix3::identity() * value.get())
                }
                // A sequence-level bound is correlated across observations;
                // treating it as fresh per-sample noise would create false
                // information. It needs an explicit shared-parameter mapping
                // or a dedicated qualified sequence correction, neither of
                // which is represented by this observation uncertainty port.
                UncertaintyModelKind::SequenceBound { .. } => {
                    Err(ProcessError::CapabilityUnavailable)
                }
            }
        }
    }
}

pub(super) fn measurement_covariance(
    config: &EngineConfig<'_>,
    uncertainty: MeasurementUncertainty<Covariance3>,
    floor: Covariance3,
) -> Result<Matrix3<f64>, ProcessError> {
    let supplied = resolve_covariance(config, uncertainty)?;
    let floor = matrix3_from_array(floor.to_matrix());
    // Adding the qualified covariance floor is conservative and avoids
    // pretending receiver-reported formal precision supersedes the empirical
    // error model.
    Ok(supplied + floor)
}

pub(super) fn measurement_noise(
    config: &EngineConfig<'_>,
    uncertainty: MeasurementUncertainty<Covariance3>,
    floor: Covariance3,
    timing: ObservationTime,
    temporal_sensitivity: Vector3<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    let mut covariance = measurement_covariance(config, uncertainty, floor)?;
    let sigma_seconds = timing.independent_one_sigma.as_seconds_f64();
    covariance +=
        temporal_sensitivity * temporal_sensitivity.transpose() * (sigma_seconds * sigma_seconds);
    Ok(DMatrix::from_column_slice(3, 3, covariance.as_slice()))
}

pub(super) fn apply_correlation_policy(
    policy: GnssCorrelationPolicy,
    covariance: &mut DMatrix<f64>,
) {
    if let GnssCorrelationPolicy::SequenceInflation { multiplier } = policy {
        *covariance *= multiplier.get();
    }
}

pub(super) fn should_reject_solution(
    solution: GnssSolutionObservation,
    field: GnssField,
    config: &EngineConfig<'_>,
) -> bool {
    let position_invalid = solution.position().is_none_or(|value| !value.valid);
    let velocity_invalid = solution.velocity().is_none_or(|value| !value.valid);
    let invalid_measurement = match field {
        GnssField::Position => position_invalid,
        GnssField::Velocity => velocity_invalid,
        GnssField::Joint => position_invalid || velocity_invalid,
    };
    invalid_measurement || !receiver_is_healthy(solution, field, config)
}

pub(super) fn validate_antenna(
    solution: GnssSolutionObservation,
    config: &EngineConfig<'_>,
) -> Result<(), ProcessError> {
    let is_declared_phase_center = config.installation.reference_points.iter().any(|point| {
        point.id() == solution.antenna_reference_point()
            && point.kind() == ReferencePointKind::GnssAntennaPhaseCenter
            && point.parameter_id() == config.installation.imu_to_gnss_antenna.parameter_id
    });
    if is_declared_phase_center {
        Ok(())
    } else {
        Err(ProcessError::InvalidEvidence)
    }
}

pub(super) fn ensure_frame(
    frame: crate::ids::FrameId,
    config: &EngineConfig<'_>,
) -> Result<(), ProcessError> {
    // Coordinate-operation records establish provenance but do not carry an
    // executable transform at this seam. Accepting a different frame while
    // using its coordinates unchanged would be a silent frame error.
    if frame == config.processing_frame.id() {
        Ok(())
    } else {
        Err(ProcessError::InvalidEvidence)
    }
}

pub(super) fn add_clock_sensitivity(
    target: &mut DMatrix<f64>,
    row: usize,
    clock: &OwnedClockModel,
    time: SessionTime,
    temporal_sensitivity: Vector3<f64>,
) -> Result<(), ProcessError> {
    let elapsed = time
        .checked_duration_since(clock.reference_time)
        .ok_or(ProcessError::InvalidEvidence)?
        .as_seconds_f64();
    for axis in 0..3 {
        target[(row + axis, clock.offset_index)] += temporal_sensitivity[axis] * 1.0e-9;
        target[(row + axis, clock.offset_index + 1)] += temporal_sensitivity[axis] * elapsed;
    }
    Ok(())
}

pub(super) fn receiver_is_healthy(
    solution: GnssSolutionObservation,
    field: GnssField,
    config: &EngineConfig<'_>,
) -> bool {
    let maximum_age = config.dynamics_profile.gnss.maximum_correction_age.as_ns();
    match field {
        GnssField::Position => solution.position().is_some_and(|position| {
            receiver_diagnostics_are_healthy(solution, position.time, maximum_age)
        }),
        GnssField::Velocity => solution.velocity().is_some_and(|velocity| {
            receiver_diagnostics_are_healthy(solution, velocity.time, maximum_age)
        }),
        GnssField::Joint => {
            solution.position().is_some_and(|position| {
                receiver_diagnostics_are_healthy(solution, position.time, maximum_age)
            }) && solution.velocity().is_some_and(|velocity| {
                receiver_diagnostics_are_healthy(solution, velocity.time, maximum_age)
            })
        }
    }
}

pub(super) fn receiver_diagnostics_are_healthy(
    solution: GnssSolutionObservation,
    measurement_time: ObservationTime,
    maximum_age: u64,
) -> bool {
    let diagnostics = solution.diagnostics();
    let health = diagnostics.health.is_some_and(|value| {
        value.value == ReceiverHealth::Healthy
            && diagnostic_information_age_at(value, measurement_time)
                .is_some_and(|age| age <= maximum_age)
    });
    let correction_fresh = diagnostics.correction_age.is_none_or(|value| {
        diagnostic_information_age_at(value, measurement_time)
            .and_then(|information_age| value.value.as_ns().checked_add(information_age))
            .is_some_and(|age| age <= maximum_age)
    });
    let solution_fresh = diagnostics.solution_age.is_none_or(|value| {
        diagnostic_information_age_at(value, measurement_time)
            .and_then(|information_age| value.value.as_ns().checked_add(information_age))
            .is_some_and(|age| age <= maximum_age)
    });
    health && correction_fresh && solution_fresh
}

pub(super) fn diagnostic_information_age_at<T: Copy>(
    diagnostic: crate::observation::TimedDiagnostic<T>,
    measurement_time: ObservationTime,
) -> Option<u64> {
    if diagnostic.time.clock_model != measurement_time.clock_model {
        return None;
    }
    let diagnostic_epoch = diagnostic.time.effective_time().ok()?;
    let measurement_epoch = measurement_time.effective_time().ok()?;
    let elapsed_ns = measurement_epoch
        .as_ns()
        .checked_sub(diagnostic_epoch.as_ns())?;
    let elapsed_ns = u64::try_from(elapsed_ns).ok()?;
    diagnostic
        .age
        .as_ns()
        .checked_add(elapsed_ns)?
        .checked_add(diagnostic.time.independent_one_sigma.as_ns())
        .and_then(|age| age.checked_add(measurement_time.independent_one_sigma.as_ns()))
}

pub(super) fn gnss_state(healthy: bool) -> GnssState {
    if healthy {
        GnssState::Healthy
    } else {
        GnssState::Suspect
    }
}

pub(super) fn timing_quality(basis: TimingBasis) -> TimingQuality {
    match basis {
        TimingBasis::PpsCorrelated | TimingBasis::SensorCounterAnchored => {
            TimingQuality::PpsCorrelated
        }
        TimingBasis::ModeledLatency => TimingQuality::Modeled,
        TimingBasis::ArrivalOnly => TimingQuality::ArrivalOnly,
    }
}

pub(super) fn combined_timing_quality(left: TimingBasis, right: TimingBasis) -> TimingQuality {
    match (timing_quality(left), timing_quality(right)) {
        (TimingQuality::ArrivalOnly, _) | (_, TimingQuality::ArrivalOnly) => {
            TimingQuality::ArrivalOnly
        }
        (TimingQuality::Modeled, _) | (_, TimingQuality::Modeled) => TimingQuality::Modeled,
        _ => TimingQuality::PpsCorrelated,
    }
}
