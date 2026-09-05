//! Qualified IMU support, ECEF inertial dynamics, and process discretization.

use crate::{
    config::{EngineConfig, GnssCorrelationPolicy, SharedParameterKind},
    error::ProcessError,
    math::{UnitQuaternion, Vector3 as SemanticVector3},
    observation::{ImuIntegrationEligibility, ImuObservation},
    offline::store::StoredNominal,
    time::{SampleSupport, SessionTime},
};

use nalgebra::{DMatrix, Matrix3, Vector3};

use super::{
    catalog::{ConsiderCatalog, ParameterCoordinate},
    filter::{HeldImu, QualifiedImuSupport},
    math::{
        ACCELEROMETER_BIAS, ATTITUDE, EARTH_J2, EARTH_MU_M3_S2, EARTH_RATE_RAD_S, GYROSCOPE_BIAS,
        NAVIGATION_DIMENSION, POSITION, VELOCITY, array3, matrix3_from_array, set_identity3,
        set_matrix3, set_rect_matrix3, skew, symmetric, vector3,
    },
    measurement::resolve_covariance,
};

pub(super) struct PropagationModel {
    pub(super) nominal: StoredNominal,
    pub(super) transition: DMatrix<f64>,
    pub(super) consider_transition: DMatrix<f64>,
    pub(super) process_covariance: DMatrix<f64>,
    pub(super) sample_influence: DMatrix<f64>,
}

pub(super) fn refresh_inertial_kinematics(
    nominal: &mut StoredNominal,
    imu: &HeldImu,
) -> Result<(), ProcessError> {
    let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
    let earth_rate_body = rotation.transpose() * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    nominal.specific_force_body = imu.specific_force_body;
    nominal.angular_rate_body = array3(
        vector3(imu.angular_rate_body) - vector3(nominal.gyroscope_bias_body) - earth_rate_body,
    );
    if nominal.is_finite() {
        Ok(())
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

/// Integrates a frozen continuous model, including a constant-input map and
/// all continuous-noise cross terms. Scaling bounds the exponential and
/// Lyapunov series before exact interval doubling.
pub(super) fn discretize_inertial_model(
    continuous: &DMatrix<f64>,
    noise_density: &DMatrix<f64>,
    dt: f64,
) -> Result<(DMatrix<f64>, DMatrix<f64>, DMatrix<f64>), ProcessError> {
    if continuous.nrows() != continuous.ncols()
        || continuous.nrows() < NAVIGATION_DIMENSION
        || noise_density.shape() != continuous.shape()
        || !dt.is_finite()
        || dt <= 0.0
        || !continuous.iter().all(|value| value.is_finite())
        || !noise_density.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let norm = continuous.row_iter().fold(0.0_f64, |largest, row| {
        largest.max(row.iter().map(|value| value.abs()).sum())
    });
    let mut local_dt = dt;
    let mut squarings = 0;
    while norm * local_dt > 0.25 {
        if squarings == 20 {
            return Err(ProcessError::NumericalNonConvergence);
        }
        local_dt *= 0.5;
        squarings += 1;
    }
    let identity = DMatrix::identity(continuous.nrows(), continuous.ncols());
    let mut transition = identity.clone();
    let mut input_integral = &identity * local_dt;
    let mut transition_term = identity;
    let mut process_covariance = noise_density * local_dt;
    let mut noise_term = process_covariance.clone();
    for order in 1..=14 {
        transition_term = (&transition_term * continuous) * (local_dt / f64::from(order));
        transition += &transition_term;
        input_integral += &transition_term * (local_dt / f64::from(order + 1));
        noise_term = (continuous * &noise_term + &noise_term * continuous.transpose())
            * (local_dt / f64::from(order + 1));
        process_covariance += &noise_term;
    }
    for _ in 0..squarings {
        process_covariance =
            &transition * &process_covariance * transition.transpose() + process_covariance;
        input_integral = &transition * &input_integral + input_integral;
        transition = &transition * &transition;
    }
    if !transition
        .iter()
        .chain(input_integral.iter())
        .chain(process_covariance.iter())
        .all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    Ok((transition, input_integral, symmetric(process_covariance)))
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(super) fn inertial_process_covariance(
    state_dimension: usize,
    accelerometer_density_ecef: Matrix3<f64>,
    accelerometer_sample_ecef: Matrix3<f64>,
    gyroscope_density_body: Matrix3<f64>,
    gyroscope_sample_body: Matrix3<f64>,
    accelerometer_bias_density_body: Matrix3<f64>,
    gyroscope_bias_density_body: Matrix3<f64>,
    dt: f64,
) -> Result<DMatrix<f64>, ProcessError> {
    if state_dimension < NAVIGATION_DIMENSION
        || !dt.is_finite()
        || dt <= 0.0
        || !accelerometer_density_ecef
            .iter()
            .chain(accelerometer_sample_ecef.iter())
            .chain(gyroscope_density_body.iter())
            .chain(gyroscope_sample_body.iter())
            .chain(accelerometer_bias_density_body.iter())
            .chain(gyroscope_bias_density_body.iter())
            .all(|value| value.is_finite())
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let mut process_covariance = DMatrix::zeros(state_dimension, state_dimension);
    // Profile process noise is a continuous covariance density. In contrast,
    // uncertainty attached to an interval-average IMU observation is the
    // covariance of one held sample. Preserve those units when mapping both
    // sources into the discrete state increment.
    let velocity_noise = accelerometer_density_ecef * dt + accelerometer_sample_ecef * dt.powi(2);
    let position_noise = accelerometer_density_ecef * (dt.powi(3) / 3.0)
        + accelerometer_sample_ecef * (dt.powi(4) / 4.0);
    let position_velocity_noise = accelerometer_density_ecef * (0.5 * dt.powi(2))
        + accelerometer_sample_ecef * (0.5 * dt.powi(3));
    set_matrix3(&mut process_covariance, VELOCITY, VELOCITY, &velocity_noise);
    set_matrix3(&mut process_covariance, POSITION, POSITION, &position_noise);
    set_matrix3(
        &mut process_covariance,
        POSITION,
        VELOCITY,
        &position_velocity_noise,
    );
    set_matrix3(
        &mut process_covariance,
        VELOCITY,
        POSITION,
        &position_velocity_noise.transpose(),
    );
    set_matrix3(
        &mut process_covariance,
        ATTITUDE,
        ATTITUDE,
        &(gyroscope_density_body * dt + gyroscope_sample_body * dt.powi(2)),
    );
    set_matrix3(
        &mut process_covariance,
        ACCELEROMETER_BIAS,
        ACCELEROMETER_BIAS,
        &(accelerometer_bias_density_body * dt),
    );
    set_matrix3(
        &mut process_covariance,
        GYROSCOPE_BIAS,
        GYROSCOPE_BIAS,
        &(gyroscope_bias_density_body * dt),
    );
    Ok(process_covariance)
}

/// Integrates the piecewise-constant body measurements over their declared
/// support. Earth rotation acts on the left and inertial body rotation on the
/// right; replacing these by one body-rate exponential is not equivalent.
pub(super) fn integrate_held_imu(
    current: &StoredNominal,
    specific_force: Vector3<f64>,
    measured_rate: Vector3<f64>,
    dt: f64,
    ellipsoid_axis: f64,
) -> Result<(Vector3<f64>, Vector3<f64>, UnitQuaternion), ProcessError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(ProcessError::InvalidEvidence);
    }
    let earth_rate = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let mut local_dt = dt;
    let mut subdivisions = 1_u16;
    // The RK4 force quadrature remains accurate for rapid rotation and long
    // interval-average supports without an unbounded integration loop.
    while local_dt > 0.01 || measured_rate.norm() * local_dt > 0.05 {
        if subdivisions == 1_024 {
            return Err(ProcessError::NumericalNonConvergence);
        }
        local_dt *= 0.5;
        subdivisions *= 2;
    }
    let rotation_increment = |rate: Vector3<f64>, duration: f64| {
        UnitQuaternion::from_rotation_vector(
            SemanticVector3::from_components(array3(rate * duration))
                .map_err(|_| ProcessError::NumericalNonConvergence)?,
        )
        .map_err(|_| ProcessError::NumericalNonConvergence)
    };
    let earth_half = rotation_increment(-earth_rate, 0.5 * local_dt)?;
    let earth_full = rotation_increment(-earth_rate, local_dt)?;
    let body_half = rotation_increment(measured_rate, 0.5 * local_dt)?;
    let body_full = rotation_increment(measured_rate, local_dt)?;
    let mut position = vector3(current.position_ecef);
    let mut velocity = vector3(current.velocity_ecef);
    let mut orientation = current.orientation_ecef_from_body;
    let acceleration = |position, velocity, rotation: UnitQuaternion| {
        Ok::<_, ProcessError>(
            matrix3_from_array(rotation.rotation_matrix()) * specific_force
                + normal_gravity_ecef(position, ellipsoid_axis)?
                - 2.0 * earth_rate.cross(&velocity),
        )
    };
    for _ in 0..subdivisions {
        let midpoint = earth_half.multiply(orientation).multiply(body_half);
        let endpoint = earth_full.multiply(orientation).multiply(body_full);
        let k1_position = velocity;
        let k1_velocity = acceleration(position, velocity, orientation)?;
        let k2_position = velocity + k1_velocity * (0.5 * local_dt);
        let k2_velocity = acceleration(
            position + k1_position * (0.5 * local_dt),
            k2_position,
            midpoint,
        )?;
        let k3_position = velocity + k2_velocity * (0.5 * local_dt);
        let k3_velocity = acceleration(
            position + k2_position * (0.5 * local_dt),
            k3_position,
            midpoint,
        )?;
        let k4_position = velocity + k3_velocity * local_dt;
        let k4_velocity = acceleration(position + k3_position * local_dt, k4_position, endpoint)?;
        position +=
            (k1_position + 2.0 * k2_position + 2.0 * k3_position + k4_position) * (local_dt / 6.0);
        velocity +=
            (k1_velocity + 2.0 * k2_velocity + 2.0 * k3_velocity + k4_velocity) * (local_dt / 6.0);
        orientation = endpoint;
    }
    Ok((position, velocity, orientation))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn propagation_model(
    config: &EngineConfig<'_>,
    catalog: &ConsiderCatalog,
    current: &StoredNominal,
    imu: &HeldImu,
    dt: f64,
    state_dimension: usize,
    colored_error: bool,
) -> Result<PropagationModel, ProcessError> {
    let position = vector3(current.position_ecef);
    let velocity = vector3(current.velocity_ecef);
    let accelerometer_bias = vector3(current.accelerometer_bias_body);
    let gyroscope_bias = vector3(current.gyroscope_bias_body);
    let specific_force = vector3(imu.specific_force_body) - accelerometer_bias;
    let measured_rate = vector3(imu.angular_rate_body) - gyroscope_bias;
    let earth_rate_ecef = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let initial_rotation = matrix3_from_array(current.orientation_ecef_from_body.rotation_matrix());
    let earth_rate_body = initial_rotation.transpose() * earth_rate_ecef;
    let body_relative_ecef_rate = measured_rate - earth_rate_body;
    let ellipsoid_axis = config.processing_frame.ellipsoid().semi_major_axis_m();
    let (new_position, new_velocity, new_orientation) =
        integrate_held_imu(current, specific_force, measured_rate, dt, ellipsoid_axis)?;
    let midpoint_earth = UnitQuaternion::from_rotation_vector(
        SemanticVector3::from_components(array3(-earth_rate_ecef * (0.5 * dt)))
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
    )
    .map_err(|_| ProcessError::NumericalNonConvergence)?;
    let midpoint_body = UnitQuaternion::from_rotation_vector(
        SemanticVector3::from_components(array3(measured_rate * (0.5 * dt)))
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
    )
    .map_err(|_| ProcessError::NumericalNonConvergence)?;
    let midpoint_orientation = midpoint_earth
        .multiply(current.orientation_ecef_from_body)
        .multiply(midpoint_body);
    let rotation = matrix3_from_array(midpoint_orientation.rotation_matrix());

    let mut continuous = DMatrix::zeros(state_dimension, state_dimension);
    set_identity3(&mut continuous, POSITION, VELOCITY);
    set_matrix3(
        &mut continuous,
        VELOCITY,
        VELOCITY,
        &(-2.0 * skew(&earth_rate_ecef)),
    );
    set_matrix3(
        &mut continuous,
        VELOCITY,
        POSITION,
        &normal_gravity_gradient_ecef(position + velocity * (0.5 * dt), ellipsoid_axis)?,
    );
    set_matrix3(
        &mut continuous,
        VELOCITY,
        ATTITUDE,
        &(-rotation * skew(&specific_force)),
    );
    set_matrix3(&mut continuous, VELOCITY, ACCELEROMETER_BIAS, &(-rotation));
    set_matrix3(
        &mut continuous,
        ATTITUDE,
        ATTITUDE,
        &(-skew(&measured_rate)),
    );
    set_matrix3(
        &mut continuous,
        ATTITUDE,
        GYROSCOPE_BIAS,
        &(-Matrix3::identity()),
    );
    let configured_accelerometer = matrix3_from_array(
        config
            .dynamics_profile
            .process_noise
            .accelerometer
            .to_matrix(),
    );
    let configured_gyroscope =
        matrix3_from_array(config.dynamics_profile.process_noise.gyroscope.to_matrix());
    let accelerometer_density = rotation * configured_accelerometer * rotation.transpose();
    let gyroscope_density = configured_gyroscope;
    let mut noise_density = DMatrix::zeros(state_dimension, state_dimension);
    set_matrix3(
        &mut noise_density,
        VELOCITY,
        VELOCITY,
        &accelerometer_density,
    );
    set_matrix3(&mut noise_density, ATTITUDE, ATTITUDE, &gyroscope_density);
    set_matrix3(
        &mut noise_density,
        ACCELEROMETER_BIAS,
        ACCELEROMETER_BIAS,
        &matrix3_from_array(
            config
                .dynamics_profile
                .process_noise
                .accelerometer_bias
                .to_matrix(),
        ),
    );
    set_matrix3(
        &mut noise_density,
        GYROSCOPE_BIAS,
        GYROSCOPE_BIAS,
        &matrix3_from_array(
            config
                .dynamics_profile
                .process_noise
                .gyroscope_bias
                .to_matrix(),
        ),
    );
    let mut sample_rate_mapping = DMatrix::zeros(state_dimension, 6);
    set_rect_matrix3(&mut sample_rate_mapping, VELOCITY, 0, &(-rotation));
    set_rect_matrix3(
        &mut sample_rate_mapping,
        ATTITUDE,
        3,
        &(-Matrix3::identity()),
    );

    let mut colored = current.colored_gnss_error;
    if colored_error {
        let GnssCorrelationPolicy::GaussMarkov {
            correlation_time,
            driving_variance,
        } = config.dynamics_profile.gnss.correlation
        else {
            return Err(ProcessError::InvalidEvidence);
        };
        let tau = correlation_time.as_seconds_f64();
        let alpha = (-dt / tau).exp();
        for axis in 0..3 {
            continuous[(NAVIGATION_DIMENSION + axis, NAVIGATION_DIMENSION + axis)] = -1.0 / tau;
            noise_density[(NAVIGATION_DIMENSION + axis, NAVIGATION_DIMENSION + axis)] =
                2.0 * driving_variance.get() / tau;
            colored[axis] *= alpha;
        }
    }
    let (transition, input_integral, process_covariance) =
        discretize_inertial_model(&continuous, &noise_density, dt)?;
    let sample_influence = &input_integral * sample_rate_mapping;

    let body_from_sensor = matrix3_from_array(
        config
            .installation
            .body_from_imu
            .mean
            .quaternion()
            .rotation_matrix(),
    );
    let (boresight_force, boresight_rate) = boresight_dynamics_jacobians(
        rotation,
        body_from_sensor,
        vector3(imu.specific_force_body),
        vector3(imu.angular_rate_body),
    );
    let mut consider_rate_mapping = DMatrix::zeros(state_dimension, catalog.covariance.nrows());
    for parameter in &catalog.parameters {
        match parameter.kind {
            SharedParameterKind::BoresightRadians | SharedParameterKind::MisalignmentRadians => {
                add_parameter_effect(
                    &mut consider_rate_mapping,
                    parameter,
                    VELOCITY,
                    &boresight_force,
                );
                add_parameter_effect(
                    &mut consider_rate_mapping,
                    parameter,
                    ATTITUDE,
                    &boresight_rate,
                );
            }
            SharedParameterKind::Scale
            | SharedParameterKind::GyroGSensitivity
            | SharedParameterKind::Other(_) => {
                return Err(ProcessError::CapabilityUnavailable);
            }
            SharedParameterKind::ClockOffsetNs
            | SharedParameterKind::ClockDrift
            | SharedParameterKind::LeverArmMetres
            | SharedParameterKind::DelayNs
            | SharedParameterKind::SurveyMetres => {}
        }
    }
    let consider_transition = input_integral * consider_rate_mapping;

    let nominal = StoredNominal {
        time: current.time,
        position_ecef: array3(new_position),
        velocity_ecef: array3(new_velocity),
        orientation_ecef_from_body: new_orientation,
        accelerometer_bias_body: current.accelerometer_bias_body,
        gyroscope_bias_body: current.gyroscope_bias_body,
        colored_gnss_error: colored,
        specific_force_body: imu.specific_force_body,
        angular_rate_body: array3(body_relative_ecef_rate),
    };
    if !nominal.is_finite() {
        return Err(ProcessError::NumericalNonConvergence);
    }
    Ok(PropagationModel {
        nominal,
        transition,
        consider_transition,
        process_covariance: symmetric(process_covariance),
        sample_influence,
    })
}

/// Installation rotations have a right sensor-frame perturbation. Both
/// vector Jacobians therefore map the same sensor-angle error through the
/// fixed body-from-sensor rotation, including the measured gyro magnitude.
pub(super) fn boresight_dynamics_jacobians(
    ecef_from_body: Matrix3<f64>,
    body_from_sensor: Matrix3<f64>,
    measured_force_body: Vector3<f64>,
    measured_rate_body: Vector3<f64>,
) -> (Matrix3<f64>, Matrix3<f64>) {
    (
        -ecef_from_body * skew(&measured_force_body) * body_from_sensor,
        -skew(&measured_rate_body) * body_from_sensor,
    )
}

pub(super) fn initial_gyro_sample_cross(
    ecef_from_body: Matrix3<f64>,
    lever_body: Vector3<f64>,
    sample_covariance: Matrix3<f64>,
) -> Matrix3<f64> {
    // Measurement error is observed minus true; navigation error is true
    // minus nominal. Antenna-to-IMU velocity initialization has opposite sign
    // to the direct sample Jacobian used by subsequent GNSS updates.
    -ecef_from_body * skew(&lever_body) * sample_covariance
}

pub(super) fn add_parameter_effect(
    target: &mut DMatrix<f64>,
    parameter: &ParameterCoordinate,
    row: usize,
    effect: &Matrix3<f64>,
) {
    let dimension = parameter.dimension.min(3);
    for output in 0..3 {
        for coordinate in 0..dimension {
            target[(row + output, parameter.start + coordinate)] += effect[(output, coordinate)];
        }
    }
}

pub(super) fn build_held_imu(
    config: &EngineConfig<'_>,
    observation: ImuObservation,
    time: SessionTime,
) -> Result<Option<HeldImu>, ProcessError> {
    let support = qualified_imu_support(observation)?;
    if support.start != time {
        return Err(ProcessError::InvalidEvidence);
    }
    if observation.integration_eligibility() != ImuIntegrationEligibility::Complete {
        return Ok(None);
    }
    if observation
        .angular_rate()
        .time
        .independent_one_sigma
        .as_ns()
        != 0
        || observation
            .specific_force()
            .time
            .independent_one_sigma
            .as_ns()
            != 0
    {
        // Independent uncertainty in the support boundaries is not captured by
        // the shared clock consider state or by interval-average value noise.
        // A temporal sensitivity model is required before this can be fused.
        return Err(ProcessError::CapabilityUnavailable);
    }
    let specific_force_sensor = observation.specific_force().value.components();
    let accelerometer_covariance =
        resolve_covariance(config, observation.specific_force().uncertainty)?;
    let rotation = matrix3_from_array(
        config
            .installation
            .body_from_imu
            .mean
            .quaternion()
            .rotation_matrix(),
    );
    let angular_rate_body = rotation * vector3(observation.angular_rate().value.components());
    let specific_force_body = rotation * vector3(specific_force_sensor);
    let gyroscope_covariance = resolve_covariance(config, observation.angular_rate().uncertainty)?;
    Ok(Some(HeldImu {
        start: support.start,
        time: support.end,
        angular_rate_body: array3(angular_rate_body),
        specific_force_body: array3(specific_force_body),
        accelerometer_covariance: rotation * accelerometer_covariance * rotation.transpose(),
        gyroscope_covariance: rotation * gyroscope_covariance * rotation.transpose(),
        degraded_input: observation.is_degraded(),
    }))
}

pub(super) fn rejected_imu_breaks_continuity(observation: ImuObservation) -> bool {
    observation.breaks_continuity()
}

pub(super) fn ensure_imu_support_is_contiguous(
    previous: &HeldImu,
    next: &HeldImu,
) -> Result<(), ProcessError> {
    if next.start < previous.time {
        return Err(ProcessError::InvalidEvidence);
    }
    if next.start > previous.time {
        // The live core represents this as a distinct synthetic interval and
        // inflates it with qualified jerk/angular-acceleration bounds.
        // Offline HeldImu does not retain that interval in the smoother state
        // yet, so averaging across the hole would add false support.
        return Err(ProcessError::IncompleteEvidence);
    }
    Ok(())
}

pub(super) fn qualified_imu_support(
    observation: ImuObservation,
) -> Result<QualifiedImuSupport, ProcessError> {
    let angular = observation.angular_rate().time;
    let force = observation.specific_force().time;
    let end = angular
        .effective_time()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    if force
        .effective_time()
        .map_err(|_| ProcessError::InvalidEvidence)?
        != end
        || force.clock_model != angular.clock_model
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let duration = match (angular.support, force.support) {
        (
            SampleSupport::IntervalAverage { duration: angular },
            SampleSupport::IntervalAverage { duration: force },
        ) if angular == force && angular.as_ns() > 0 => angular,
        _ => return Err(ProcessError::InvalidEvidence),
    };
    let duration_ns = i64::try_from(duration.as_ns()).map_err(|_| ProcessError::InvalidEvidence)?;
    let start = end
        .as_ns()
        .checked_sub(duration_ns)
        .map(SessionTime::from_ns)
        .ok_or(ProcessError::InvalidEvidence)?;
    Ok(QualifiedImuSupport {
        start,
        end,
        duration,
        clock_model: angular.clock_model,
    })
}

pub(super) fn predicted_acceleration(
    nominal: &StoredNominal,
    config: &EngineConfig<'_>,
) -> Result<Vector3<f64>, ProcessError> {
    let rotation = matrix3_from_array(nominal.orientation_ecef_from_body.rotation_matrix());
    let force = rotation
        * (vector3(nominal.specific_force_body) - vector3(nominal.accelerometer_bias_body));
    let position = vector3(nominal.position_ecef);
    let velocity = vector3(nominal.velocity_ecef);
    let gravity = normal_gravity_ecef(
        position,
        config.processing_frame.ellipsoid().semi_major_axis_m(),
    )?;
    Ok(force + gravity - 2.0 * Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S).cross(&velocity))
}

/// WGS-84/J2 apparent gravity in ECEF, including the centrifugal term.
pub(super) fn normal_gravity_ecef(
    position: Vector3<f64>,
    semi_major_axis_m: f64,
) -> Result<Vector3<f64>, ProcessError> {
    let radius = position.norm();
    if !radius.is_finite()
        || !(6.0e6..=7.0e6).contains(&radius)
        || !semi_major_axis_m.is_finite()
        || semi_major_axis_m <= 0.0
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let radius_squared = radius * radius;
    let normalized_z_squared = position.z * position.z / radius_squared;
    let oblateness = 1.5 * EARTH_J2 * (semi_major_axis_m * semi_major_axis_m / radius_squared);
    let common = -EARTH_MU_M3_S2 / (radius_squared * radius);
    let gravitational = Vector3::new(
        common * position.x * (1.0 - oblateness * (5.0 * normalized_z_squared - 1.0)),
        common * position.y * (1.0 - oblateness * (5.0 * normalized_z_squared - 1.0)),
        common * position.z * (1.0 - oblateness * (5.0 * normalized_z_squared - 3.0)),
    );
    let earth = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    // Centrifugal acceleration is included here, so the mechanization does
    // not add it a second time.
    let apparent = gravitational - earth.cross(&earth.cross(&position));
    if apparent.iter().all(|value| value.is_finite()) {
        Ok(apparent)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

/// Position Jacobian of the exact gravity function used above.  A symmetric
/// metre-scale central difference is deterministic in `f64` and retains the
/// J2 and centrifugal derivatives without maintaining a second, subtly
/// different closed-form model.
pub(super) fn normal_gravity_gradient_ecef(
    position: Vector3<f64>,
    semi_major_axis_m: f64,
) -> Result<Matrix3<f64>, ProcessError> {
    const STEP_M: f64 = 1.0;
    let mut gradient = Matrix3::zeros();
    for axis in 0..3 {
        let mut delta = Vector3::zeros();
        delta[axis] = STEP_M;
        let plus = normal_gravity_ecef(position + delta, semi_major_axis_m)?;
        let minus = normal_gravity_ecef(position - delta, semi_major_axis_m)?;
        gradient.set_column(axis, &((plus - minus) / (2.0 * STEP_M)));
    }
    if gradient.iter().all(|value| value.is_finite()) {
        Ok(gradient)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}
