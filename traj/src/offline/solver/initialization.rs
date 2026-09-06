//! GNSS initialization and antenna-to-IMU state and covariance transforms.

use crate::{
    config::GnssCorrelationPolicy,
    error::ProcessError,
    frame::ReferenceEllipsoid,
    math::UnitQuaternion,
    observation::{GnssSolutionObservation, InputDisposition},
    offline::store::{StateStore, StoredCovariance, StoredNominal},
    time::SessionTime,
};

use nalgebra::{DMatrix, Matrix3, UnitQuaternion as NaUnitQuaternion, Vector3};

use super::{
    estimation::matrix_is_psd,
    filter::{HeldImu, OfflineFilter},
    inertial::{add_parameter_effect, initial_gyro_sample_cross},
    math::{
        ACCELEROMETER_BIAS, ATTITUDE, COLORED_ERROR_DIMENSION, EARTH_RATE_RAD_S, GYROSCOPE_BIAS,
        NAVIGATION_DIMENSION, POSITION, VELOCITY, array_matrix3, array3, matrix3_from_array,
        set_matrix3, set_rect_matrix3, skew, symmetric, vector3,
    },
    measurement::{
        GnssField, add_clock_sensitivity, apply_correlation_policy, combined_timing_quality,
        ensure_frame, gnss_state, measurement_covariance, receiver_is_healthy,
    },
};

impl<'a> OfflineFilter<'a> {
    pub(super) fn initialize(
        &mut self,
        position_solution: GnssSolutionObservation,
        velocity_solution: GnssSolutionObservation,
        time: SessionTime,
        store: &mut dyn StateStore,
        initial_guess: Option<StoredNominal>,
    ) -> Result<(), ProcessError> {
        if initialization_pair_time(position_solution, velocity_solution)? != Some(time) {
            return Err(ProcessError::InvalidEvidence);
        }
        let position = position_solution
            .position()
            .ok_or(ProcessError::InvalidEvidence)?;
        if !position.valid {
            return Err(ProcessError::InvalidEvidence);
        }
        ensure_frame(position.frame, self.config)?;
        let velocity = velocity_solution
            .velocity()
            .ok_or(ProcessError::InvalidEvidence)?;
        if !velocity.valid {
            return Err(ProcessError::InvalidEvidence);
        }
        ensure_frame(velocity.frame, self.config)?;
        let velocity_components = velocity.value.components();
        let position_time = position
            .time
            .effective_time()
            .map_err(|_| ProcessError::InvalidEvidence)?;
        let velocity_time = velocity
            .time
            .effective_time()
            .map_err(|_| ProcessError::InvalidEvidence)?;
        let position_age = time
            .checked_duration_since(position_time)
            .ok_or(ProcessError::InvalidEvidence)?;
        let velocity_age = time
            .checked_duration_since(velocity_time)
            .ok_or(ProcessError::InvalidEvidence)?;
        let maximum_pair_age = self.config.dynamics_profile.gnss.maximum_correction_age;
        let position_age_ns =
            u64::try_from(position_age.as_ns()).map_err(|_| ProcessError::InvalidEvidence)?;
        let velocity_age_ns =
            u64::try_from(velocity_age.as_ns()).map_err(|_| ProcessError::InvalidEvidence)?;
        if position_age_ns > maximum_pair_age.as_ns() || velocity_age_ns > maximum_pair_age.as_ns()
        {
            return Err(ProcessError::IncompleteEvidence);
        }
        let position_to_initialization_seconds = position_age.as_seconds_f64();
        let position_components = array3(
            vector3(position.value.components())
                + vector3(velocity_components) * position_to_initialization_seconds,
        );
        let orientation = orientation_from_position_velocity(
            position_components,
            velocity_components,
            self.config.processing_frame.ellipsoid(),
        )?;
        let initial_imu = self
            .held_imu
            .clone()
            .ok_or(ProcessError::IncompleteEvidence)?;
        // The queued interval-average is available from its support start and
        // is the same piecewise-constant input used by live preintegration.
        // Never initialize outside that qualified support.
        if time < initial_imu.start || time > initial_imu.time {
            return Err(ProcessError::IncompleteEvidence);
        }
        let tuning = self.config.navigation_profile.embedded_tuning;
        let accelerometer_bias = tuning
            .accelerometer_bias_prior_mps2
            .map(crate::math::FiniteF64::get);
        let gyroscope_bias = tuning
            .gyroscope_bias_prior_rad_s
            .map(crate::math::FiniteF64::get);
        let lever = self
            .config
            .installation
            .imu_to_gnss_antenna
            .mean
            .components_m();
        let baseline_nominal = initialized_nominal(
            time,
            position_components,
            velocity_components,
            orientation,
            &initial_imu,
            lever,
            accelerometer_bias,
            gyroscope_bias,
        );
        let nominal = match initial_guess {
            Some(guess) if guess.time == time => {
                let rotation =
                    matrix3_from_array(guess.orientation_ecef_from_body.rotation_matrix());
                let earth_rate_body =
                    rotation.transpose() * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
                let angular_rate_body = vector3(initial_imu.angular_rate_body)
                    - vector3(guess.gyroscope_bias_body)
                    - earth_rate_body;
                StoredNominal {
                    specific_force_body: initial_imu.specific_force_body,
                    angular_rate_body: array3(angular_rate_body),
                    ..guess
                }
            }
            _ => baseline_nominal,
        };
        let mut state_covariance = DMatrix::zeros(self.state_dimension, self.state_dimension);
        let position_covariance = measurement_covariance(
            self.config,
            position.uncertainty,
            self.config.dynamics_profile.gnss.position_covariance_floor,
        )?;
        let mut velocity_covariance = measurement_covariance(
            self.config,
            velocity.uncertainty,
            self.config.dynamics_profile.gnss.velocity_covariance_floor,
        )?;
        let position_velocity_cross = if position_solution.id() == velocity_solution.id() {
            if position_solution != velocity_solution {
                return Err(ProcessError::InvalidEvidence);
            }
            position_solution
                .position_velocity_cross_covariance()
                .map(|value| matrix3_from_array(value.to_matrix()))
        } else {
            None
        };
        let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
        let lever_vector = vector3(lever);
        let velocity_clock = self.clock_for(velocity.time, velocity_time)?;
        let velocity_timing_model =
            self.velocity_timing_model(&nominal, velocity.time, velocity_time, velocity_clock)?;
        velocity_covariance += velocity_timing_model.timing_sensitivity_covariance_ecef;
        let velocity_timing_sigma_s = velocity.time.independent_one_sigma.as_seconds_f64();
        let velocity_temporal_sensitivity = velocity_timing_model.antenna_acceleration_ecef;
        set_initial_gnss_covariance(
            &mut state_covariance,
            &InitialGnssCovariance {
                position: position_covariance,
                velocity: velocity_covariance,
                position_velocity: position_velocity_cross,
                position_timing_sigma_s: position.time.independent_one_sigma.as_seconds_f64(),
                position_temporal_sensitivity: vector3(velocity_components),
                velocity_timing_sigma_s,
                velocity_temporal_sensitivity,
                position_to_initialization_seconds,
            },
            self.config.dynamics_profile.gnss.correlation,
        )?;
        state_covariance[(ATTITUDE, ATTITUDE)] = tuning.roll_pitch_variance_rad2.get();
        state_covariance[(ATTITUDE + 1, ATTITUDE + 1)] = tuning.roll_pitch_variance_rad2.get();
        state_covariance[(ATTITUDE + 2, ATTITUDE + 2)] =
            tuning.unobservable_yaw_variance_rad2.get();
        for axis in 0..3 {
            state_covariance[(ACCELEROMETER_BIAS + axis, ACCELEROMETER_BIAS + axis)] =
                tuning.accelerometer_bias_variance[axis].get();
            state_covariance[(GYROSCOPE_BIAS + axis, GYROSCOPE_BIAS + axis)] =
                tuning.gyroscope_bias_variance[axis].get();
        }
        if self.colored_error {
            let GnssCorrelationPolicy::GaussMarkov {
                driving_variance, ..
            } = self.config.dynamics_profile.gnss.correlation
            else {
                return Err(ProcessError::InvalidEvidence);
            };
            add_initial_colored_gnss_error_covariance(
                &mut state_covariance,
                driving_variance.get(),
            )?;
        }
        let initial_rate_duration = initial_imu
            .time
            .checked_duration_since(initial_imu.start)
            .ok_or(ProcessError::InvalidEvidence)?
            .as_seconds_f64();
        if !initial_rate_duration.is_finite() || initial_rate_duration <= 0.0 {
            return Err(ProcessError::InvalidEvidence);
        }
        let initial_rate_covariance = initial_imu.gyroscope_covariance
            + matrix3_from_array(
                self.config
                    .dynamics_profile
                    .process_noise
                    .gyroscope
                    .to_matrix(),
            ) / initial_rate_duration;
        let antenna_to_imu = initial_antenna_to_imu_error_jacobian(
            self.state_dimension,
            &rotation,
            vector3(initial_imu.angular_rate_body),
            vector3(nominal.gyroscope_bias_body),
            lever_vector,
        )?;
        state_covariance = transform_initial_antenna_covariance_to_imu(
            state_covariance,
            &rotation,
            vector3(initial_imu.angular_rate_body),
            vector3(nominal.gyroscope_bias_body),
            initial_rate_covariance,
            lever_vector,
        )?;
        let mut antenna_consider_mapping =
            DMatrix::zeros(self.state_dimension, self.catalog.covariance.nrows());
        let position_clock = self.clock_for(position.time, position_time)?;
        let position_temporal_sensitivity = vector3(velocity_components);
        add_clock_sensitivity(
            &mut antenna_consider_mapping,
            POSITION,
            position_clock,
            position_time,
            position_temporal_sensitivity,
        )?;
        add_clock_sensitivity(
            &mut antenna_consider_mapping,
            VELOCITY,
            velocity_clock,
            velocity_time,
            velocity_temporal_sensitivity,
        )?;
        self.add_delay_sensitivity(
            &mut antenna_consider_mapping,
            POSITION,
            position_time,
            position_temporal_sensitivity,
        );
        self.add_delay_sensitivity(
            &mut antenna_consider_mapping,
            VELOCITY,
            velocity_time,
            velocity_temporal_sensitivity,
        );
        if position_to_initialization_seconds != 0.0 {
            add_clock_sensitivity(
                &mut antenna_consider_mapping,
                POSITION,
                velocity_clock,
                velocity_time,
                velocity_temporal_sensitivity * position_to_initialization_seconds,
            )?;
            self.add_delay_sensitivity(
                &mut antenna_consider_mapping,
                POSITION,
                velocity_time,
                velocity_temporal_sensitivity * position_to_initialization_seconds,
            );
        }
        let mut initial_consider_mapping = &antenna_to_imu * antenna_consider_mapping;
        if let Some(parameter) = self
            .catalog
            .parameter(self.config.installation.imu_to_gnss_antenna.parameter_id)
        {
            add_parameter_effect(
                &mut initial_consider_mapping,
                &parameter,
                POSITION,
                &(-rotation),
            );
            add_parameter_effect(
                &mut initial_consider_mapping,
                &parameter,
                VELOCITY,
                &(-rotation * skew(&vector3(nominal.angular_rate_body))),
            );
        }
        state_covariance += &initial_consider_mapping
            * &self.catalog.covariance
            * initial_consider_mapping.transpose();
        let state_consider = &initial_consider_mapping * &self.catalog.covariance;
        let covariance = StoredCovariance {
            state: symmetric(state_covariance),
            state_consider,
        };
        self.guide_nominal = self.relinearized.then(|| nominal.clone());
        self.nominal = Some(nominal);
        self.covariance = Some(covariance);
        let mut initial_sample_cross = DMatrix::zeros(self.state_dimension, 6);
        let velocity_gyro_cross =
            initial_gyro_sample_cross(rotation, lever_vector, initial_imu.gyroscope_covariance);
        set_rect_matrix3(&mut initial_sample_cross, VELOCITY, 3, &velocity_gyro_cross);
        self.install_active_imu_sample(&initial_imu, Some(initial_sample_cross))?;
        self.connected = false;
        self.gnss_state = gnss_state(
            receiver_is_healthy(position_solution, GnssField::Position, self.config)
                && receiver_is_healthy(velocity_solution, GnssField::Velocity, self.config),
        );
        self.timing_quality = combined_timing_quality(position.time.basis, velocity.time.basis);
        self.transition_accumulator.fill(0.0);
        self.transition_accumulator.fill_diagonal(1.0);
        self.consider_transition_accumulator.fill(0.0);
        self.process_accumulator.fill(0.0);
        self.sample_influence_accumulator.fill(0.0);
        self.last_stored_sample_cross.fill(0.0);
        self.integration_imu = None;
        self.store_current(
            None,
            Some((
                InputDisposition::InitializationOnly,
                0.0,
                DMatrix::identity(self.state_dimension, self.state_dimension),
            )),
            store,
        )?;
        self.connected = true;
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(super) struct InitialGnssCovariance {
    pub(super) position: Matrix3<f64>,
    pub(super) velocity: Matrix3<f64>,
    pub(super) position_velocity: Option<Matrix3<f64>>,
    pub(super) position_timing_sigma_s: f64,
    pub(super) position_temporal_sensitivity: Vector3<f64>,
    pub(super) velocity_timing_sigma_s: f64,
    pub(super) velocity_temporal_sensitivity: Vector3<f64>,
    /// Constant-velocity transport from the position field's effective epoch
    /// to the common initialization epoch.
    pub(super) position_to_initialization_seconds: f64,
}

/// Installs the joint antenna position/velocity covariance used by the
/// initializer. Sample-specific position and velocity timing errors are
/// independent here; shared clock uncertainty remains in the consider block.
pub(super) fn set_initial_gnss_covariance(
    target: &mut DMatrix<f64>,
    inputs: &InitialGnssCovariance,
    correlation: GnssCorrelationPolicy,
) -> Result<(), ProcessError> {
    if target.nrows() != target.ncols()
        || target.nrows() < VELOCITY + 3
        || !inputs.position_timing_sigma_s.is_finite()
        || inputs.position_timing_sigma_s < 0.0
        || !inputs.velocity_timing_sigma_s.is_finite()
        || inputs.velocity_timing_sigma_s < 0.0
        || !inputs.position_to_initialization_seconds.is_finite()
        || inputs.position_to_initialization_seconds < 0.0
        || !inputs
            .position
            .iter()
            .chain(inputs.velocity.iter())
            .chain(inputs.position_temporal_sensitivity.iter())
            .chain(inputs.velocity_temporal_sensitivity.iter())
            .all(|value| value.is_finite())
        || inputs
            .position_velocity
            .is_some_and(|cross| !cross.iter().all(|value| value.is_finite()))
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let mut joint = DMatrix::zeros(6, 6);
    set_matrix3(&mut joint, POSITION, POSITION, &inputs.position);
    set_matrix3(&mut joint, VELOCITY, VELOCITY, &inputs.velocity);
    if let Some(cross) = inputs.position_velocity {
        set_matrix3(&mut joint, POSITION, VELOCITY, &cross);
        set_matrix3(&mut joint, VELOCITY, POSITION, &cross.transpose());
    }
    apply_correlation_policy(correlation, &mut joint);
    let position = matrix3_from_array(array_matrix3(&joint, POSITION, POSITION))
        + inputs.position_temporal_sensitivity
            * inputs.position_temporal_sensitivity.transpose()
            * inputs.position_timing_sigma_s.powi(2);
    let velocity = matrix3_from_array(array_matrix3(&joint, VELOCITY, VELOCITY))
        + inputs.velocity_temporal_sensitivity
            * inputs.velocity_temporal_sensitivity.transpose()
            * inputs.velocity_timing_sigma_s.powi(2);
    set_matrix3(&mut joint, POSITION, POSITION, &position);
    set_matrix3(&mut joint, VELOCITY, VELOCITY, &velocity);
    if inputs.position_to_initialization_seconds != 0.0 {
        let mut epoch_transform = DMatrix::identity(6, 6);
        for axis in 0..3 {
            epoch_transform[(POSITION + axis, VELOCITY + axis)] =
                inputs.position_to_initialization_seconds;
        }
        joint = &epoch_transform * joint * epoch_transform.transpose();
    }
    if !matrix_is_psd(&joint) {
        return Err(ProcessError::InvalidEvidence);
    }
    for row in 0..6 {
        for column in 0..6 {
            target[(row, column)] = joint[(row, column)];
        }
    }
    Ok(())
}

pub(super) fn add_initial_colored_gnss_error_covariance(
    target: &mut DMatrix<f64>,
    stationary_variance: f64,
) -> Result<(), ProcessError> {
    if target.nrows() != target.ncols()
        || target.nrows() < NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION
        || !stationary_variance.is_finite()
        || stationary_variance < 0.0
    {
        return Err(ProcessError::InvalidEvidence);
    }
    // The initialized position estimate is the GNSS observation while the
    // colored receiver error starts at its zero-mean prior. For z = p + e + ν,
    // the corresponding joint error is δp = ν - e, hence
    // Ppp += C, Pee = C and Ppe = -C. Initializing the two blocks
    // independently would make the first measurement appear overconfident.
    for axis in 0..3 {
        target[(POSITION + axis, POSITION + axis)] += stationary_variance;
        target[(NAVIGATION_DIMENSION + axis, NAVIGATION_DIMENSION + axis)] = stationary_variance;
        target[(POSITION + axis, NAVIGATION_DIMENSION + axis)] = -stationary_variance;
        target[(NAVIGATION_DIMENSION + axis, POSITION + axis)] = -stationary_variance;
    }
    Ok(())
}

pub(super) fn initialization_pair_time(
    position_solution: GnssSolutionObservation,
    velocity_solution: GnssSolutionObservation,
) -> Result<Option<SessionTime>, ProcessError> {
    let Some(position) = position_solution.position() else {
        return Ok(None);
    };
    let Some(velocity) = velocity_solution.velocity() else {
        return Ok(None);
    };
    let position_time = position
        .time
        .effective_time()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    let velocity_time = velocity
        .time
        .effective_time()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    let compatible = position.valid
        && velocity.valid
        && position.time.clock_model == velocity.time.clock_model
        && position.frame == velocity.frame
        && position_solution.id().source == velocity_solution.id().source
        && position_solution.antenna_reference_point()
            == velocity_solution.antenna_reference_point();
    Ok(compatible.then_some(position_time.max(velocity_time)))
}

pub(super) fn orientation_from_position_velocity(
    position: [f64; 3],
    velocity: [f64; 3],
    ellipsoid: ReferenceEllipsoid,
) -> Result<UnitQuaternion, ProcessError> {
    let (_, up) = geodetic_north_up(vector3(position), ellipsoid)?;
    let velocity = vector3(velocity);
    let horizontal_velocity = velocity - up * velocity.dot(&up);
    let forward = horizontal_velocity
        .try_normalize(1.0e-4)
        .ok_or(ProcessError::IncompleteEvidence)?;
    let left = up
        .cross(&forward)
        .try_normalize(1.0e-12)
        .ok_or(ProcessError::InvalidEvidence)?;
    let forward = left.cross(&up);
    let rotation = Matrix3::from_columns(&[forward, left, up]);
    let quaternion = NaUnitQuaternion::from_rotation_matrix(
        &nalgebra::Rotation3::from_matrix_unchecked(rotation),
    );
    let raw = quaternion.quaternion();
    UnitQuaternion::from_wxyz([raw.w, raw.i, raw.j, raw.k])
        .map_err(|_| ProcessError::InvalidEvidence)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn initialized_nominal(
    time: SessionTime,
    antenna_position: [f64; 3],
    antenna_velocity: [f64; 3],
    orientation_ecef_from_body: UnitQuaternion,
    imu: &HeldImu,
    lever_body: [f64; 3],
    accelerometer_bias_body: [f64; 3],
    gyroscope_bias_body: [f64; 3],
) -> StoredNominal {
    let rotation = matrix3_from_array(orientation_ecef_from_body.rotation_matrix());
    let earth_rate_body = rotation.transpose() * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let angular_rate_body =
        vector3(imu.angular_rate_body) - vector3(gyroscope_bias_body) - earth_rate_body;
    let lever = vector3(lever_body);
    StoredNominal {
        time,
        position_ecef: array3(vector3(antenna_position) - rotation * lever),
        velocity_ecef: array3(
            vector3(antenna_velocity) - rotation * angular_rate_body.cross(&lever),
        ),
        orientation_ecef_from_body,
        accelerometer_bias_body,
        gyroscope_bias_body,
        colored_gnss_error: [0.0; 3],
        imu_sample_error_body: [0.0; 6],
        specific_force_body: imu.specific_force_body,
        angular_rate_body: array3(angular_rate_body),
    }
}

/// Right-multiplicative attitude sensitivity of
/// `R_e_b (omega_eb_b x lever_b)`.  `omega_eb_b` itself changes when the
/// attitude error changes because the Earth rate is represented in body
/// coordinates; omitting that term is observable even for a body stationary
/// in ECEF.
pub(super) fn antenna_velocity_attitude_jacobian(
    rotation_ecef_from_body: &Matrix3<f64>,
    omega_eb_body: Vector3<f64>,
    earth_rate_body: Vector3<f64>,
    lever_body: Vector3<f64>,
) -> Matrix3<f64> {
    -rotation_ecef_from_body * skew(&omega_eb_body.cross(&lever_body))
        + rotation_ecef_from_body * skew(&lever_body) * skew(&earth_rate_body)
}

pub(super) fn initial_antenna_to_imu_error_jacobian(
    state_dimension: usize,
    rotation_ecef_from_body: &Matrix3<f64>,
    angular_rate_ib_body: Vector3<f64>,
    gyroscope_bias_body: Vector3<f64>,
    lever_body: Vector3<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    if state_dimension < NAVIGATION_DIMENSION
        || !rotation_ecef_from_body
            .iter()
            .chain(angular_rate_ib_body.iter())
            .chain(gyroscope_bias_body.iter())
            .chain(lever_body.iter())
            .all(|value| value.is_finite())
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let earth_rate_ecef = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let earth_rate_body = rotation_ecef_from_body.transpose() * earth_rate_ecef;
    let angular_rate_minus_bias = angular_rate_ib_body - gyroscope_bias_body;
    let omega_eb_body = angular_rate_minus_bias - earth_rate_body;
    let position_attitude = rotation_ecef_from_body * skew(&lever_body);
    let velocity_attitude = -antenna_velocity_attitude_jacobian(
        rotation_ecef_from_body,
        omega_eb_body,
        earth_rate_body,
        lever_body,
    );
    let velocity_gyroscope_bias = -rotation_ecef_from_body * skew(&lever_body);
    let mut jacobian = DMatrix::identity(state_dimension, state_dimension);
    set_matrix3(&mut jacobian, POSITION, ATTITUDE, &position_attitude);
    set_matrix3(&mut jacobian, VELOCITY, ATTITUDE, &velocity_attitude);
    set_matrix3(
        &mut jacobian,
        VELOCITY,
        GYROSCOPE_BIAS,
        &velocity_gyroscope_bias,
    );
    if jacobian.iter().all(|value| value.is_finite()) {
        Ok(jacobian)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

/// Maps the complete antenna-centred state covariance to the IMU sensing
/// centre under the right-multiplicative attitude-error convention. The gyro
/// observation is not a state coordinate, so its sample covariance enters as
/// an independent velocity-noise term after the state congruence.
pub(super) fn transform_initial_antenna_covariance_to_imu(
    antenna_covariance: DMatrix<f64>,
    rotation_ecef_from_body: &Matrix3<f64>,
    angular_rate_ib_body: Vector3<f64>,
    gyroscope_bias_body: Vector3<f64>,
    gyro_sample_covariance: Matrix3<f64>,
    lever_body: Vector3<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    if antenna_covariance.nrows() != antenna_covariance.ncols()
        || antenna_covariance.nrows() < NAVIGATION_DIMENSION
        || !antenna_covariance
            .iter()
            .chain(gyro_sample_covariance.iter())
            .all(|value| value.is_finite())
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let jacobian = initial_antenna_to_imu_error_jacobian(
        antenna_covariance.nrows(),
        rotation_ecef_from_body,
        angular_rate_ib_body,
        gyroscope_bias_body,
        lever_body,
    )?;
    let mut transformed = &jacobian * antenna_covariance * jacobian.transpose();
    let gyro_to_velocity = rotation_ecef_from_body * skew(&lever_body);
    let velocity_covariance = matrix3_from_array(array_matrix3(&transformed, VELOCITY, VELOCITY))
        + gyro_to_velocity * gyro_sample_covariance * gyro_to_velocity.transpose();
    set_matrix3(&mut transformed, VELOCITY, VELOCITY, &velocity_covariance);
    let transformed = symmetric(transformed);
    if transformed.iter().all(|value| value.is_finite()) && matrix_is_psd(&transformed) {
        Ok(transformed)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn geodetic_north_up(
    position: Vector3<f64>,
    ellipsoid: ReferenceEllipsoid,
) -> Result<(Vector3<f64>, Vector3<f64>), ProcessError> {
    if !position.iter().all(|value| value.is_finite()) {
        return Err(ProcessError::InvalidEvidence);
    }
    let horizontal = position.x.hypot(position.y);
    if horizontal == 0.0 && position.z == 0.0 {
        return Err(ProcessError::InvalidEvidence);
    }
    let semi_major = ellipsoid.semi_major_axis_m();
    let flattening = ellipsoid.inverse_flattening().recip();
    let semi_minor = semi_major * (1.0 - flattening);
    let eccentricity_squared = flattening * (2.0 - flattening);
    let second_eccentricity_squared =
        (semi_major * semi_major - semi_minor * semi_minor) / (semi_minor * semi_minor);
    let longitude = position.y.atan2(position.x);
    let theta = (position.z * semi_major).atan2(horizontal * semi_minor);
    let (sin_theta, cos_theta) = theta.sin_cos();
    let latitude = (position.z + second_eccentricity_squared * semi_minor * sin_theta.powi(3))
        .atan2(horizontal - eccentricity_squared * semi_major * cos_theta.powi(3));
    let (sin_latitude, cos_latitude) = latitude.sin_cos();
    let (sin_longitude, cos_longitude) = longitude.sin_cos();
    let north = Vector3::new(
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    );
    let up = Vector3::new(
        cos_latitude * cos_longitude,
        cos_latitude * sin_longitude,
        sin_latitude,
    );
    if north.iter().chain(up.iter()).all(|value| value.is_finite()) {
        Ok((north, up))
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}
