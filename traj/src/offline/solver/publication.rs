//! Trajectory construction, dense bridges, and semantic state publication.

use crate::{
    config::{EngineConfig, ProcessingSpec},
    error::ProcessError,
    frame::{BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody},
    ids::TrajectoryRevision,
    offline::{
        ports::{ResultRecord, ResultSink, SinkTransaction, SmoothedStateRecord},
        store::{FixedRecordStoreKind, StateStore, StoredCovariance, StoredNominal, StoredStep},
    },
    quality::{
        CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
        HeadingSource, Integrity, ObservabilityReport, TimingQuality, Validity,
    },
    trajectory::{CoupledDenseBridge, DenseBridgeInput, Trajectory, TrajectoryKnot},
};

use nalgebra::{DMatrix, Matrix3};

use std::boxed::Box;

use super::{
    catalog::ConsiderCatalog,
    estimation::{injection_reset, matrix_is_psd},
    filter::HeldImu,
    inertial::propagation_model,
    initialization::geodetic_north_up,
    math::{
        ATTITUDE, NAVIGATION_DIMENSION, POSITION, VELOCITY, array_matrix3, array3,
        kinematic_covariance, matrix3_from_array, matrix3_to_array, symmetric, symmetric3, vector3,
    },
    smoothing::{augmented_covariance, boxminus, joint_covariance},
};

pub(super) fn build_trajectory(
    spec: &ProcessingSpec<'_>,
    catalog: &ConsiderCatalog,
    store: &mut dyn StateStore,
    revision: TrajectoryRevision,
    maximum_segments: u64,
    backing_kind: FixedRecordStoreKind,
) -> Result<Trajectory, ProcessError> {
    let mut trajectory = Trajectory::new(spec.engine.processing_frame, revision);
    trajectory
        .set_attachment_model(spec.engine.installation.attachment)
        .map_err(|_| ProcessError::InvalidEvidence)?;
    trajectory.prepare_offline_storage_with_covariance(
        maximum_segments,
        backing_kind,
        store.dimensions().0 + store.dimensions().1 + 6,
    )?;
    for point in spec.engine.installation.reference_points {
        trajectory
            .add_reference_point(*point)
            .map_err(|_| ProcessError::ResourceLimit)?;
    }
    let mut previous: Option<(u64, StoredStep, TrajectoryKnot)> = None;
    for index in 0..store.len() {
        let step = store.get(index).map_err(ProcessError::from)?;
        let smoothed = step.smoothed.as_ref().ok_or(ProcessError::StorageCorrupt)?;
        let covariance = step
            .smoothed_covariance
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        if !spec.span.contains(smoothed.time) {
            previous = None;
            continue;
        }
        let knot = trajectory_knot(smoothed, covariance, &step, &spec.engine)?;
        if let Some((previous_index, previous_step, previous_knot)) = previous.take() {
            if step.connected_from_previous && smoothed.time > previous_step.filtered.time {
                let previous_smoothed = previous_step
                    .smoothed
                    .as_ref()
                    .ok_or(ProcessError::StorageCorrupt)?;
                let bridge_input = dense_bridge_input(
                    previous_smoothed,
                    &previous_step,
                    smoothed,
                    &step,
                    &spec.engine,
                    catalog,
                    store.dimensions().0,
                )?;
                trajectory.push_offline_conditional_bridge_segment(
                    previous_knot,
                    knot,
                    bridge_input,
                    (previous_index, index),
                )?;
            }
        }
        previous = Some((index, step, knot));
    }
    if trajectory.span().is_none() {
        return Err(ProcessError::IncompleteEvidence);
    }
    trajectory.finish_offline_storage()?;
    Ok(trajectory)
}

pub(super) fn dense_bridge_input(
    start: &StoredNominal,
    start_step: &StoredStep,
    end: &StoredNominal,
    end_step: &StoredStep,
    config: &EngineConfig<'_>,
    catalog: &ConsiderCatalog,
    state_dimension: usize,
) -> Result<DenseBridgeInput, ProcessError> {
    let imu = end_step
        .integration_imu
        .as_ref()
        .ok_or(ProcessError::IncompleteEvidence)?;
    if imu.start != start.time || imu.end != end.time || state_dimension < NAVIGATION_DIMENSION {
        return Err(ProcessError::StorageCorrupt);
    }
    let duration = end
        .time
        .checked_duration_since(start.time)
        .ok_or(ProcessError::StorageCorrupt)?
        .as_seconds_f64();
    if !duration.is_finite() || duration <= 0.0 {
        return Err(ProcessError::StorageCorrupt);
    }
    let held = HeldImu {
        start: imu.start,
        time: imu.end,
        angular_rate_body: imu.angular_rate_body,
        specific_force_body: imu.specific_force_body,
        // These terms affect Q only. The mean reintegration below uses the
        // immutable vectors, while the bridge covariance uses the exact Q
        // retained by the accepted smoother pass.
        accelerometer_covariance: Matrix3::zeros(),
        gyroscope_covariance: Matrix3::zeros(),
        degraded_input: end_step.degraded_input,
    };
    let mut reintegrated = propagation_model(
        config,
        catalog,
        start,
        &held,
        duration,
        state_dimension,
        state_dimension > NAVIGATION_DIMENSION,
    )?
    .nominal;
    reintegrated.time = end.time;
    let integrated_rotation_body = start
        .orientation_ecef_from_body
        .inverse()
        .multiply(reintegrated.orientation_ecef_from_body)
        .rotation_vector()
        .components();

    let start_covariance = start_step
        .smoothed_covariance
        .as_ref()
        .ok_or(ProcessError::StorageCorrupt)?;
    let end_covariance = end_step
        .smoothed_covariance
        .as_ref()
        .ok_or(ProcessError::StorageCorrupt)?;
    let start_augmented = start_step.smoothed_sample.as_ref().map_or_else(
        || augmented_covariance(start_covariance, &catalog.covariance),
        |sample| joint_covariance(start_covariance, sample, &catalog.covariance),
    );
    let end_augmented = end_step.smoothed_sample.as_ref().map_or_else(
        || augmented_covariance(end_covariance, &catalog.covariance),
        |sample| joint_covariance(end_covariance, sample, &catalog.covariance),
    );
    let backward_gain = start_step
        .smoothed_backward_gain
        .as_ref()
        .ok_or(ProcessError::StorageCorrupt)?;
    if backward_gain.shape() != start_augmented.shape()
        || end_augmented.shape() != start_augmented.shape()
    {
        return Err(ProcessError::StorageCorrupt);
    }
    let cross = backward_gain * &end_augmented;
    let kinematic_indices = [
        POSITION,
        POSITION + 1,
        POSITION + 2,
        VELOCITY,
        VELOCITY + 1,
        VELOCITY + 2,
        ATTITUDE,
        ATTITUDE + 1,
        ATTITUDE + 2,
    ];
    let mut endpoint_joint_covariance = Box::new([[0.0; 18]; 18]);
    for (row, &state_row) in kinematic_indices.iter().enumerate() {
        for (column, &state_column) in kinematic_indices.iter().enumerate() {
            endpoint_joint_covariance[row][column] = start_augmented[(state_row, state_column)];
            endpoint_joint_covariance[row + 9][column + 9] =
                end_augmented[(state_row, state_column)];
            endpoint_joint_covariance[row][column + 9] = cross[(state_row, state_column)];
            endpoint_joint_covariance[column + 9][row] = cross[(state_row, state_column)];
        }
    }

    if let Some(dynamics) = &end_step.dynamics {
        let n = state_dimension;
        let m = catalog.covariance.nrows();
        let d = n + m + 6;
        if start_augmented.nrows() != d {
            return Err(ProcessError::StorageCorrupt);
        }
        let mut endpoint_joint = DMatrix::zeros(2 * d, 2 * d);
        endpoint_joint
            .view_mut((0, 0), (d, d))
            .copy_from(&start_augmented);
        endpoint_joint
            .view_mut((d, d), (d, d))
            .copy_from(&end_augmented);
        endpoint_joint.view_mut((0, d), (d, d)).copy_from(&cross);
        endpoint_joint
            .view_mut((d, 0), (d, d))
            .copy_from(&cross.transpose());
        let start_residual = boxminus(start, &dynamics.reference_start, n)?;
        let end_residual = boxminus(end, &dynamics.reference_end, n)?;
        let start_reset = injection_reset(&start_residual)?;
        let end_reset = injection_reset(&end_residual)?;
        let mut start_to_reference = DMatrix::identity(d, d);
        let mut end_to_reference = DMatrix::identity(d, d);
        start_to_reference.view_mut((0, 0), (n, n)).copy_from(
            &start_reset
                .lu()
                .solve(&DMatrix::identity(n, n))
                .ok_or(ProcessError::NumericalNonConvergence)?,
        );
        end_to_reference.view_mut((0, 0), (n, n)).copy_from(
            &end_reset
                .lu()
                .solve(&DMatrix::identity(n, n))
                .ok_or(ProcessError::NumericalNonConvergence)?,
        );
        let mut continuous = DMatrix::zeros(d, d);
        continuous
            .view_mut((0, 0), (n, n))
            .copy_from(&dynamics.continuous);
        continuous
            .view_mut((0, n), (n, m))
            .copy_from(&dynamics.consider_rate_mapping);
        continuous
            .view_mut((0, n + m), (n, 6))
            .copy_from(&dynamics.sample_rate_mapping);
        let mut noise_density = DMatrix::zeros(d, d);
        noise_density
            .view_mut((0, 0), (n, n))
            .copy_from(&dynamics.noise_density);
        let mut rate_mapping = DMatrix::zeros(3, d);
        let earth_body = matrix3_from_array(
            dynamics
                .reference_start
                .orientation_ecef_from_body
                .rotation_matrix(),
        )
        .transpose()
            * nalgebra::Vector3::new(0.0, 0.0, super::math::EARTH_RATE_RAD_S);
        rate_mapping
            .view_mut((0, ATTITUDE), (3, 3))
            .copy_from(&(-super::math::skew(&earth_body)));
        for axis in 0..3 {
            rate_mapping[(axis, super::math::GYROSCOPE_BIAS + axis)] = -1.0;
            rate_mapping[(axis, n + m + 3 + axis)] = -1.0;
        }
        rate_mapping
            .view_mut((0, n), (3, m))
            .copy_from(&dynamics.consider_rate_mapping.rows(ATTITUDE, 3));
        let mut parameter_ids = std::vec![0;d];
        for parameter in &catalog.parameters {
            if parameter.kind == crate::config::SharedParameterKind::LeverArmMetres
                && parameter.dimension == 3
            {
                parameter_ids[n + parameter.start] = u64::from(parameter.id.get());
            }
        }
        let model = CoupledDenseBridge {
            state_dimension: n,
            continuous,
            noise_density,
            endpoint_joint: symmetric(endpoint_joint),
            start_to_reference,
            end_to_reference,
            rate_mapping,
            parameter_ids,
            duration_seconds: duration,
            cache: Default::default(),
            reference_start_orientation: dynamics
                .reference_start
                .orientation_ecef_from_body
                .components_wxyz(),
            reference_body_rate: core::array::from_fn(|axis| {
                imu.angular_rate_body[axis]
                    - dynamics.reference_start.gyroscope_bias_body[axis]
                    - dynamics.reference_start.imu_sample_error_body[3 + axis]
            }),
            gyro_density: config.dynamics_profile.process_noise.gyroscope.to_matrix(),
        };
        model
            .validate()
            .map_err(|_| ProcessError::NumericalNonConvergence)?;
        return Ok(DenseBridgeInput {
            coupled: Some(Box::new(model)),
            covariance_available: true,
            endpoint_joint_covariance,
            acceleration_spectral_density_ecef: [[0.0; 3]; 3],
            attitude_spectral_density_body: [[0.0; 3]; 3],
            acceleration_interval_average_covariance_ecef: [[0.0; 3]; 3],
            angular_rate_interval_average_covariance_body: [[0.0; 3]; 3],
            reintegrated_position_ecef_m: reintegrated.position_ecef,
            reintegrated_velocity_ecef_mps: reintegrated.velocity_ecef,
            integrated_rotation_body,
        });
    }

    let end_residual = boxminus(end, &end_step.predicted, state_dimension)?;
    let end_reset = injection_reset(&end_residual)?;
    let process = symmetric(&end_reset * &end_step.process_covariance * end_reset.transpose());
    let start_rotation = matrix3_from_array(start.orientation_ecef_from_body.rotation_matrix());
    let acceleration_density = symmetric3(
        start_rotation
            * matrix3_from_array(
                config
                    .dynamics_profile
                    .process_noise
                    .accelerometer
                    .to_matrix(),
            )
            * start_rotation.transpose(),
    );
    let attitude_reset = matrix3_from_array(array_matrix3(&end_reset, ATTITUDE, ATTITUDE));
    let attitude_density = symmetric3(
        attitude_reset
            * matrix3_from_array(config.dynamics_profile.process_noise.gyroscope.to_matrix())
            * attitude_reset.transpose(),
    );
    let velocity_process = matrix3_from_array(array_matrix3(&process, VELOCITY, VELOCITY));
    let attitude_process = matrix3_from_array(array_matrix3(&process, ATTITUDE, ATTITUDE));
    let acceleration_sample =
        symmetric3((velocity_process - acceleration_density * duration) / (duration * duration));
    let angular_rate_sample =
        symmetric3((attitude_process - attitude_density * duration) / (duration * duration));
    let mut acceleration_spectral_density_ecef = matrix3_to_array(acceleration_density);
    let mut attitude_spectral_density_body = matrix3_to_array(attitude_density);
    let mut acceleration_interval_average_covariance_ecef = matrix3_to_array(acceleration_sample);
    let mut angular_rate_interval_average_covariance_body = matrix3_to_array(angular_rate_sample);
    let covariance_available = match validate_bridge_process_model(
        &process,
        duration,
        &acceleration_spectral_density_ecef,
        &attitude_spectral_density_body,
        &acceleration_interval_average_covariance_ecef,
        &angular_rate_interval_average_covariance_body,
    ) {
        Ok(()) => true,
        Err(ProcessError::NumericalNonConvergence) => {
            // Full inertial dynamics and a retained sample's state cross
            // cannot generally be represented by independent acceleration
            // and gyro bridge noise. Preserve the refined curve and endpoint
            // marginals, but do not invent an interior covariance model.
            acceleration_spectral_density_ecef = [[0.0; 3]; 3];
            attitude_spectral_density_body = [[0.0; 3]; 3];
            acceleration_interval_average_covariance_ecef = [[0.0; 3]; 3];
            angular_rate_interval_average_covariance_body = [[0.0; 3]; 3];
            false
        }
        Err(error) => return Err(error),
    };

    Ok(DenseBridgeInput {
        coupled: None,
        covariance_available,
        endpoint_joint_covariance,
        acceleration_spectral_density_ecef,
        attitude_spectral_density_body,
        acceleration_interval_average_covariance_ecef,
        angular_rate_interval_average_covariance_body,
        reintegrated_position_ecef_m: reintegrated.position_ecef,
        reintegrated_velocity_ecef_mps: reintegrated.velocity_ecef,
        integrated_rotation_body,
    })
}

pub(super) fn validate_bridge_process_model(
    process: &DMatrix<f64>,
    duration: f64,
    acceleration_density: &[[f64; 3]; 3],
    attitude_density: &[[f64; 3]; 3],
    acceleration_sample: &[[f64; 3]; 3],
    angular_rate_sample: &[[f64; 3]; 3],
) -> Result<(), ProcessError> {
    if process.nrows() < NAVIGATION_DIMENSION
        || process.ncols() != process.nrows()
        || !duration.is_finite()
        || duration <= 0.0
        || !process.iter().all(|value| value.is_finite())
        || !acceleration_density
            .iter()
            .flatten()
            .chain(attitude_density.iter().flatten())
            .chain(acceleration_sample.iter().flatten())
            .chain(angular_rate_sample.iter().flatten())
            .all(|value| value.is_finite())
    {
        return Err(ProcessError::StorageCorrupt);
    }
    const BLOCK_RELATIVE_TOLERANCE: f64 = 64.0 * f32::EPSILON as f64;
    let acceleration = matrix3_from_array(*acceleration_density);
    let attitude = matrix3_from_array(*attitude_density);
    let acceleration_sample = matrix3_from_array(*acceleration_sample);
    let angular_rate_sample = matrix3_from_array(*angular_rate_sample);
    let sample_psd =
        |matrix: Matrix3<f64>| matrix_is_psd(&DMatrix::from_column_slice(3, 3, matrix.as_slice()));
    if !sample_psd(acceleration)
        || !sample_psd(attitude)
        || !sample_psd(acceleration_sample)
        || !sample_psd(angular_rate_sample)
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let expected_blocks = [
        (
            POSITION,
            POSITION,
            acceleration * (duration.powi(3) / 3.0)
                + acceleration_sample * (duration.powi(4) / 4.0),
        ),
        (
            POSITION,
            VELOCITY,
            acceleration * (0.5 * duration.powi(2))
                + acceleration_sample * (0.5 * duration.powi(3)),
        ),
        (
            VELOCITY,
            POSITION,
            acceleration * (0.5 * duration.powi(2))
                + acceleration_sample * (0.5 * duration.powi(3)),
        ),
        (
            VELOCITY,
            VELOCITY,
            acceleration * duration + acceleration_sample * duration.powi(2),
        ),
        (
            ATTITUDE,
            ATTITUDE,
            attitude * duration + angular_rate_sample * duration.powi(2),
        ),
    ];
    for (row, column, expected) in expected_blocks {
        let mut block_scale = expected.amax();
        for local_row in 0..3 {
            for local_column in 0..3 {
                block_scale =
                    block_scale.max(process[(row + local_row, column + local_column)].abs());
            }
        }
        let tolerance = BLOCK_RELATIVE_TOLERANCE * block_scale.max(f64::MIN_POSITIVE);
        for local_row in 0..3 {
            for local_column in 0..3 {
                let actual = process[(row + local_row, column + local_column)];
                if (actual - expected[(local_row, local_column)]).abs() > tolerance {
                    return Err(ProcessError::NumericalNonConvergence);
                }
            }
        }
    }
    // The current dense prior carries integrated acceleration and gyro noise.
    // If the accepted filter later adds a direct cross-process term between
    // these kinematic blocks, this constructor must be extended rather than
    // silently dropping it.
    for row in 0..9 {
        for column in 0..9 {
            let retained = (row < 6 && column < 6) || (row >= ATTITUDE && column >= ATTITUDE);
            if !retained {
                let value = process[(row, column)];
                if value.abs() > BLOCK_RELATIVE_TOLERANCE * process.amax().max(f64::MIN_POSITIVE) {
                    return Err(ProcessError::CapabilityUnavailable);
                }
            }
        }
    }
    Ok(())
}

pub(super) fn publish_states<S: ResultSink>(
    spec: &ProcessingSpec<'_>,
    store: &mut dyn StateStore,
    transaction: &mut SinkTransaction<'_, S>,
) -> Result<u64, ProcessError> {
    let mut count = 0_u64;
    for index in 0..store.len() {
        let step = store.get(index).map_err(ProcessError::from)?;
        let smoothed = step.smoothed.as_ref().ok_or(ProcessError::StorageCorrupt)?;
        if !spec.span.contains(smoothed.time) {
            continue;
        }
        let covariance = step
            .smoothed_covariance
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        let knot = trajectory_knot(smoothed, covariance, &step, &spec.engine)?;
        let record = SmoothedStateRecord {
            time: knot.time,
            position_ecef: knot.position_ecef,
            velocity_ecef: knot.velocity_ecef,
            orientation_ecef_from_body: knot.orientation_ecef_from_body,
            specific_force_body: knot.specific_force_body,
            covariance: knot.covariance,
            quality: knot.quality,
            observability: knot.observability,
        };
        transaction.write(ResultRecord::State(&record))?;
        count = count.checked_add(1).ok_or(ProcessError::ResourceLimit)?;
    }
    Ok(count)
}

pub(super) fn trajectory_knot(
    state: &StoredNominal,
    covariance: &StoredCovariance,
    step: &StoredStep,
    config: &EngineConfig<'_>,
) -> Result<TrajectoryKnot, ProcessError> {
    let kinematic_covariance = kinematic_covariance(&covariance.state)?;
    let (_, up_ecef) = geodetic_north_up(
        vector3(state.position_ecef),
        config.processing_frame.ellipsoid(),
    )?;
    let rotation = matrix3_from_array(state.orientation_ecef_from_body.rotation_matrix());
    let local_up_body = rotation.transpose() * up_ecef;
    let attitude_covariance = covariance
        .state
        .view((ATTITUDE, ATTITUDE), (3, 3))
        .into_owned();
    let heading_variance =
        (local_up_body.transpose() * attitude_covariance * local_up_body)[(0, 0)];
    let velocity = vector3(state.velocity_ecef);
    let horizontal_velocity = velocity - up_ecef * velocity.dot(&up_ecef);
    let speed = horizontal_velocity.norm();
    let velocity_covariance = covariance
        .state
        .view((VELOCITY, VELOCITY), (3, 3))
        .into_owned();
    let horizontal_projector = Matrix3::identity() - up_ecef * up_ecef.transpose();
    let horizontal_covariance =
        horizontal_projector * velocity_covariance * horizontal_projector.transpose();
    let maximum_horizontal_variance = horizontal_covariance
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .copied()
        .fold(0.0_f64, f64::max);
    let course_variance =
        horizontal_velocity
            .try_normalize(1.0e-12)
            .map_or(f64::INFINITY, |course_direction| {
                let lateral = up_ecef.cross(&course_direction);
                (lateral.transpose() * horizontal_covariance * lateral)[(0, 0)]
                    / speed.powi(2).max(f64::MIN_POSITIVE)
            });
    let course_snr = speed / maximum_horizontal_variance.max(f64::MIN_POSITIVE).sqrt();
    let course_available = course_snr >= config.dynamics_profile.heading.minimum_course_snr.get()
        && course_variance
            <= config
                .dynamics_profile
                .heading
                .maximum_course_variance_rad2
                .get();
    Ok(TrajectoryKnot {
        time: state.time,
        position_ecef: EcefPosition::from_components(state.position_ecef)
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
        velocity_ecef: EcefVelocity::from_components(state.velocity_ecef)
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            state.orientation_ecef_from_body,
        ),
        specific_force_body: BodyVector::from_components(array3(
            vector3(state.specific_force_body) - vector3(state.accelerometer_bias_body),
        ))
        .map_err(|_| ProcessError::NumericalNonConvergence)?,
        covariance: kinematic_covariance,
        quality: EstimateQuality {
            stage: EstimateStage::Finalized,
            validity: if step.connected_from_previous && !step.degraded_input {
                Validity::Nominal
            } else {
                Validity::Degraded
            },
            gnss: step.gnss_state,
            timing: step.timing_quality,
            integrity: if !matches!(step.gnss_state, GnssState::Absent | GnssState::Suspect)
                && !matches!(
                    step.timing_quality,
                    TimingQuality::ArrivalOnly | TimingQuality::Discontinuous
                ) {
                Integrity::Monitored
            } else {
                Integrity::Unavailable
            },
            covariance: CovarianceConditioning::ConditionalOnSelection,
            imu_gap: !step.connected_from_previous,
            degraded_input: step.degraded_input,
        },
        observability: offline_observability(heading_variance, course_available),
    })
}

pub(super) fn offline_observability(
    heading_variance_rad2: f64,
    course_available: bool,
) -> ObservabilityReport {
    // Course is only a numerical yaw seed.  The offline smoother has no
    // typed supplied-heading or qualified dynamic-alignment state, so a small
    // posterior attitude variance alone cannot promote it into body-heading
    // evidence.
    ObservabilityReport {
        heading_source: HeadingSource::None,
        heading: HeadingObservability::Unobservable,
        heading_variance_rad2: heading_variance_rad2
            .is_finite()
            .then_some(heading_variance_rad2),
        course_available,
        body_axis_quantities_available: false,
        angular_acceleration_available: false,
    }
}
