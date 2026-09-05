use super::*;

#[test]
fn initialization_pairs_asynchronous_position_and_velocity_at_the_later_epoch() {
    let mut pending = PendingInitialization::default();
    let position_only = gnss_solution(
        1,
        Some(point_time(10)),
        None,
        Some(ReceiverHealth::Healthy),
        None,
        None,
    );
    assert!(
        pending
            .observe(position_only, GnssField::Position, SessionTime::from_ns(10))
            .unwrap()
            .is_none()
    );

    let asynchronous_velocity = gnss_solution(
        2,
        None,
        Some(point_time(11)),
        Some(ReceiverHealth::Healthy),
        None,
        None,
    );
    let pair = pending
        .observe(
            asynchronous_velocity,
            GnssField::Velocity,
            SessionTime::from_ns(11),
        )
        .unwrap()
        .unwrap();
    assert_eq!(pair.time, SessionTime::from_ns(11));
    assert_eq!(
        pair.position
            .position()
            .unwrap()
            .time
            .effective_time()
            .unwrap(),
        SessionTime::from_ns(10)
    );
    assert_eq!(
        pair.velocity
            .velocity()
            .unwrap()
            .time
            .effective_time()
            .unwrap(),
        SessionTime::from_ns(11)
    );

    let mut incompatible = PendingInitialization::default();
    let mut other_clock = point_time(20);
    other_clock.clock_model = ClockModelId::new(2);
    let position = gnss_solution(
        4,
        Some(point_time(20)),
        None,
        Some(ReceiverHealth::Healthy),
        None,
        None,
    );
    let velocity = gnss_solution(
        5,
        None,
        Some(other_clock),
        Some(ReceiverHealth::Healthy),
        None,
        None,
    );
    assert!(
        incompatible
            .observe(position, GnssField::Position, SessionTime::from_ns(20))
            .unwrap()
            .is_none()
    );
    assert!(
        incompatible
            .observe(velocity, GnssField::Velocity, SessionTime::from_ns(20))
            .unwrap()
            .is_none()
    );
}

#[test]
fn zero_speed_cannot_fabricate_a_north_heading_seed() {
    assert_eq!(
        orientation_from_position_velocity(
            [6_378_137.0, 0.0, 0.0],
            [0.0; 3],
            ReferenceEllipsoid::WGS84,
        ),
        Err(ProcessError::IncompleteEvidence)
    );
    assert!(
        orientation_from_position_velocity(
            [6_378_137.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            ReferenceEllipsoid::WGS84,
        )
        .is_ok()
    );
}

#[test]
fn initial_antenna_to_imu_jacobian_matches_central_difference() {
    let orientation = NaUnitQuaternion::from_scaled_axis(Vector3::new(0.3, -0.2, 0.4));
    let rotation = orientation.to_rotation_matrix().into_inner();
    let angular_rate_ib_body = Vector3::new(0.21, -0.13, 0.47);
    let gyroscope_bias_body = Vector3::new(0.01, -0.02, 0.03);
    let lever_body = Vector3::new(0.8, -0.35, 0.42);
    let analytic = initial_antenna_to_imu_error_jacobian(
        NAVIGATION_DIMENSION,
        &rotation,
        angular_rate_ib_body,
        gyroscope_bias_body,
        lever_body,
    )
    .unwrap();
    let numerical = numerical_initial_antenna_to_imu_jacobian(
        orientation,
        angular_rate_ib_body,
        gyroscope_bias_body,
        lever_body,
    );
    let error = (&analytic - numerical).norm();
    assert!(error < 2.0e-8, "initialization Jacobian error: {error}");
}

#[test]
fn initial_gnss_covariance_preserves_cross_and_independent_timing_terms() {
    let position = Matrix3::new(
        4.0, 0.2, -0.1, //
        0.2, 5.0, 0.3, //
        -0.1, 0.3, 6.0,
    );
    let velocity = Matrix3::new(
        0.7, -0.02, 0.03, //
        -0.02, 0.8, 0.04, //
        0.03, 0.04, 0.9,
    );
    let cross = Matrix3::new(
        0.10, -0.02, 0.03, //
        0.04, 0.08, -0.01, //
        -0.03, 0.02, 0.06,
    );
    let position_timing = Vector3::new(7.0, -2.0, 1.0);
    let velocity_timing = Vector3::new(-0.4, 1.2, 0.7);
    let multiplier = 2.5;
    let inputs = InitialGnssCovariance {
        position,
        velocity,
        position_velocity: Some(cross),
        position_timing_sigma_s: 0.02,
        position_temporal_sensitivity: position_timing,
        velocity_timing_sigma_s: 0.03,
        velocity_temporal_sensitivity: velocity_timing,
        position_to_initialization_seconds: 0.0,
    };
    let mut state_covariance = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    set_initial_gnss_covariance(
        &mut state_covariance,
        &inputs,
        GnssCorrelationPolicy::SequenceInflation {
            multiplier: NonNegativeF64::new(multiplier).unwrap(),
        },
    )
    .unwrap();

    let expected_position = position * multiplier
        + position_timing * position_timing.transpose() * inputs.position_timing_sigma_s.powi(2);
    let expected_velocity = velocity * multiplier
        + velocity_timing * velocity_timing.transpose() * inputs.velocity_timing_sigma_s.powi(2);
    let actual_position = matrix3_from_array(array_matrix3(&state_covariance, POSITION, POSITION));
    let actual_velocity = matrix3_from_array(array_matrix3(&state_covariance, VELOCITY, VELOCITY));
    let actual_cross = matrix3_from_array(array_matrix3(&state_covariance, POSITION, VELOCITY));
    let actual_cross_transpose =
        matrix3_from_array(array_matrix3(&state_covariance, VELOCITY, POSITION));
    assert!((actual_position - expected_position).norm() < 1.0e-12);
    assert!((actual_velocity - expected_velocity).norm() < 1.0e-12);
    assert!((actual_cross - cross * multiplier).norm() < 1.0e-12);
    assert!((actual_cross_transpose - cross.transpose() * multiplier).norm() < 1.0e-12);
}

#[test]
fn gauss_markov_initialization_preserves_the_measurement_noise_marginal() {
    let measurement_variance = 0.4;
    let colored_variance = 1.7;
    let dimension = NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION;
    let mut covariance = DMatrix::zeros(dimension, dimension);
    for axis in 0..3 {
        covariance[(POSITION + axis, POSITION + axis)] = measurement_variance;
    }
    add_initial_colored_gnss_error_covariance(&mut covariance, colored_variance).unwrap();

    let position = matrix3_from_array(array_matrix3(&covariance, POSITION, POSITION));
    let colored = matrix3_from_array(array_matrix3(
        &covariance,
        NAVIGATION_DIMENSION,
        NAVIGATION_DIMENSION,
    ));
    let cross = matrix3_from_array(array_matrix3(&covariance, POSITION, NAVIGATION_DIMENSION));
    assert_eq!(
        position,
        Matrix3::identity() * (measurement_variance + colored_variance)
    );
    assert_eq!(colored, Matrix3::identity() * colored_variance);
    assert_eq!(cross, Matrix3::identity() * -colored_variance);
    let measurement = position + colored + cross + cross.transpose();
    assert!((measurement - Matrix3::identity() * measurement_variance).norm() < 1.0e-15);
    assert!(matrix_is_psd(&covariance));
}

#[test]
fn initial_covariance_transform_retains_every_induced_block_and_gyro_noise() {
    let orientation = NaUnitQuaternion::from_scaled_axis(Vector3::new(0.3, -0.2, 0.4));
    let rotation = orientation.to_rotation_matrix().into_inner();
    let angular_rate_ib_body = Vector3::new(0.21, -0.13, 0.47);
    let gyroscope_bias_body = Vector3::new(0.01, -0.02, 0.03);
    let lever_body = Vector3::new(0.8, -0.35, 0.42);
    let gyro_sample_covariance = Matrix3::from_diagonal(&Vector3::new(0.012, 0.023, 0.034));
    let mut antenna_covariance = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    let mut diagonal_variance = 0.1;
    for index in 0..NAVIGATION_DIMENSION {
        antenna_covariance[(index, index)] = diagonal_variance;
        diagonal_variance += 0.1;
    }
    let position_velocity_cross = Matrix3::new(
        0.03, -0.01, 0.02, //
        0.01, 0.04, -0.02, //
        -0.03, 0.02, 0.05,
    );
    set_matrix3(
        &mut antenna_covariance,
        POSITION,
        VELOCITY,
        &position_velocity_cross,
    );
    set_matrix3(
        &mut antenna_covariance,
        VELOCITY,
        POSITION,
        &position_velocity_cross.transpose(),
    );

    let transformed = transform_initial_antenna_covariance_to_imu(
        antenna_covariance.clone(),
        &rotation,
        angular_rate_ib_body,
        gyroscope_bias_body,
        gyro_sample_covariance,
        lever_body,
    )
    .unwrap();
    let numerical = numerical_initial_antenna_to_imu_jacobian(
        orientation,
        angular_rate_ib_body,
        gyroscope_bias_body,
        lever_body,
    );
    let mut expected = &numerical * antenna_covariance * numerical.transpose();
    let gyro_to_velocity = rotation * skew(&lever_body);
    let velocity_sample_noise =
        gyro_to_velocity * gyro_sample_covariance * gyro_to_velocity.transpose();
    let expected_velocity =
        matrix3_from_array(array_matrix3(&expected, VELOCITY, VELOCITY)) + velocity_sample_noise;
    set_matrix3(&mut expected, VELOCITY, VELOCITY, &expected_velocity);
    let error = (&transformed - symmetric(expected)).norm();
    assert!(error < 5.0e-8, "initial covariance error: {error}");
    assert!(matrix3_from_array(array_matrix3(&transformed, POSITION, ATTITUDE)).norm() > 0.0);
    assert!(matrix3_from_array(array_matrix3(&transformed, VELOCITY, ATTITUDE)).norm() > 0.0);
    assert!(matrix3_from_array(array_matrix3(&transformed, VELOCITY, GYROSCOPE_BIAS)).norm() > 0.0);
    assert!(
        matrix3_from_array(array_matrix3(&transformed, POSITION, VELOCITY)).norm()
            > position_velocity_cross.norm()
    );
    assert!(velocity_sample_noise.norm() > 0.0);
}

#[test]
fn course_seed_never_claims_body_heading_observability() {
    let report = offline_observability(1.0e-12, true);
    assert_eq!(report.heading_source, HeadingSource::None);
    assert_eq!(report.heading, HeadingObservability::Unobservable);
    assert!(!report.body_axis_quantities_available);
    assert!(report.course_available);
    assert_eq!(report.heading_variance_rad2, Some(1.0e-12));
}

#[test]
fn gnss_health_requires_explicit_healthy_and_fresh_diagnostics() {
    const MAX_AGE: u64 = 10;
    let solution = |health, correction_age, solution_age| {
        gnss_solution(
            1,
            Some(point_time(10)),
            Some(point_time(10)),
            health,
            correction_age,
            solution_age,
        )
    };
    assert!(receiver_diagnostics_are_healthy(
        solution(Some(ReceiverHealth::Healthy), Some((5, 5)), Some(10)),
        point_time(10),
        MAX_AGE,
    ));
    for health in [
        None,
        Some(ReceiverHealth::Unknown),
        Some(ReceiverHealth::Suspect),
        Some(ReceiverHealth::Fault),
    ] {
        assert!(!receiver_diagnostics_are_healthy(
            solution(health, None, None),
            point_time(10),
            MAX_AGE,
        ));
    }
    assert!(!receiver_diagnostics_are_healthy(
        solution(Some(ReceiverHealth::Healthy), Some((11, 0)), None),
        point_time(10),
        MAX_AGE,
    ));
    assert!(!receiver_diagnostics_are_healthy(
        solution(Some(ReceiverHealth::Healthy), Some((0, 11)), None),
        point_time(10),
        MAX_AGE,
    ));
    assert!(!receiver_diagnostics_are_healthy(
        solution(Some(ReceiverHealth::Healthy), None, Some(11)),
        point_time(10),
        MAX_AGE,
    ));
}

#[test]
fn gnss_health_is_aligned_to_the_selected_field_epoch_and_clock() {
    const MAX_AGE: u64 = 10;
    let base = gnss_solution(
        1,
        Some(point_time(10)),
        Some(point_time(20)),
        None,
        None,
        None,
    );
    let at = |time, age| TimedDiagnostic {
        value: ReceiverHealth::Healthy,
        time: point_time(time),
        age: DurationNs::from_ns(age),
    };
    let position_fresh = with_health_diagnostic(base, at(0, 0));
    assert!(receiver_diagnostics_are_healthy(
        position_fresh,
        point_time(10),
        MAX_AGE,
    ));
    assert!(!receiver_diagnostics_are_healthy(
        position_fresh,
        point_time(20),
        MAX_AGE,
    ));

    let future = with_health_diagnostic(base, at(11, 0));
    assert!(!receiver_diagnostics_are_healthy(
        future,
        point_time(10),
        MAX_AGE,
    ));
    let mut different_clock = point_time(0);
    different_clock.clock_model = ClockModelId::new(2);
    let different_clock = with_health_diagnostic(
        base,
        TimedDiagnostic {
            value: ReceiverHealth::Healthy,
            time: different_clock,
            age: DurationNs::ZERO,
        },
    );
    assert!(!receiver_diagnostics_are_healthy(
        different_clock,
        point_time(10),
        MAX_AGE,
    ));

    let mut diagnostic_time = point_time(5);
    diagnostic_time.independent_one_sigma = DurationNs::from_ns(2);
    let mut measurement_time = point_time(10);
    measurement_time.independent_one_sigma = DurationNs::from_ns(3);
    let exact_boundary = with_health_diagnostic(
        base,
        TimedDiagnostic {
            value: ReceiverHealth::Healthy,
            time: diagnostic_time,
            age: DurationNs::ZERO,
        },
    );
    assert!(receiver_diagnostics_are_healthy(
        exact_boundary,
        measurement_time,
        MAX_AGE,
    ));
    let over_boundary = with_health_diagnostic(
        base,
        TimedDiagnostic {
            value: ReceiverHealth::Healthy,
            time: diagnostic_time,
            age: DurationNs::from_ns(1),
        },
    );
    assert!(!receiver_diagnostics_are_healthy(
        over_boundary,
        measurement_time,
        MAX_AGE,
    ));
}
