use super::*;

#[test]
fn state_transition_contains_the_position_velocity_term_exactly_once() {
    let dt = 0.125;
    let mut continuous = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    set_identity3(&mut continuous, POSITION, VELOCITY);
    let velocity_dynamics = Matrix3::new(
        0.0, 0.2, -0.1, //
        -0.2, 0.0, 0.3, //
        0.1, -0.3, 0.0,
    );
    set_matrix3(&mut continuous, VELOCITY, VELOCITY, &velocity_dynamics);

    let (transition, _, _) = discretize_inertial_model(
        &continuous,
        &DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
        dt,
    )
    .unwrap();
    let actual = matrix3_from_array(array_matrix3(&transition, POSITION, VELOCITY));
    let rate = 0.14_f64.sqrt();
    let expected = Matrix3::identity() * dt
        + velocity_dynamics * ((1.0 - (rate * dt).cos()) / rate.powi(2))
        + velocity_dynamics * velocity_dynamics * ((rate * dt - (rate * dt).sin()) / rate.powi(3));
    assert!((actual - expected).norm() < 1.0e-15);
    assert!((actual - Matrix3::identity() * (2.0 * dt)).norm() > 0.1);
}

#[test]
fn continuous_bias_noise_reaches_position_velocity_and_all_cross_terms() {
    let dt = 0.4_f64;
    let density = 2.5;
    let mut continuous = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    continuous[(POSITION, VELOCITY)] = 1.0;
    continuous[(VELOCITY, ACCELEROMETER_BIAS)] = -1.0;
    let mut noise = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    noise[(ACCELEROMETER_BIAS, ACCELEROMETER_BIAS)] = density;
    let (transition, input_integral, covariance) =
        discretize_inertial_model(&continuous, &noise, dt).unwrap();
    for (row, column, expected) in [
        (POSITION, POSITION, density * dt.powi(5) / 20.0),
        (POSITION, VELOCITY, density * dt.powi(4) / 8.0),
        (POSITION, ACCELEROMETER_BIAS, -density * dt.powi(3) / 6.0),
        (VELOCITY, VELOCITY, density * dt.powi(3) / 3.0),
        (VELOCITY, ACCELEROMETER_BIAS, -density * dt.powi(2) / 2.0),
        (ACCELEROMETER_BIAS, ACCELEROMETER_BIAS, density * dt),
    ] {
        assert!((covariance[(row, column)] - expected).abs() < 2.0e-15);
        assert!((covariance[(column, row)] - expected).abs() < 2.0e-15);
    }
    assert!((transition[(POSITION, ACCELEROMETER_BIAS)] + 0.5 * dt * dt).abs() < 1.0e-15);
    assert!((input_integral[(POSITION, ACCELEROMETER_BIAS)] + dt.powi(3) / 6.0).abs() < 1.0e-15);
}

#[test]
fn split_discretization_preserves_rotation_process_noise_and_held_sample_covariance() {
    let mut continuous = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    set_identity3(&mut continuous, POSITION, VELOCITY);
    set_matrix3(
        &mut continuous,
        VELOCITY,
        ATTITUDE,
        &(-skew(&Vector3::new(1.0, 3.0, 9.0))),
    );
    set_matrix3(
        &mut continuous,
        ATTITUDE,
        ATTITUDE,
        &(-skew(&Vector3::new(4.0, -2.0, 7.0))),
    );
    set_matrix3(
        &mut continuous,
        ATTITUDE,
        GYROSCOPE_BIAS,
        &(-Matrix3::identity()),
    );
    let density = DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION) * 0.02;
    let (full, full_input, full_q) = discretize_inertial_model(&continuous, &density, 0.4).unwrap();
    let (first, first_input, first_q) =
        discretize_inertial_model(&continuous, &density, 0.13).unwrap();
    let (second, second_input, second_q) =
        discretize_inertial_model(&continuous, &density, 0.27).unwrap();
    assert!((&full - &second * &first).norm() < 2.0e-13);
    assert!((&full_input - (&second * &first_input + &second_input)).norm() < 2.0e-13);
    assert!((&full_q - (&second * first_q * second.transpose() + second_q)).norm() < 2.0e-13);

    let mut sample_map = DMatrix::zeros(NAVIGATION_DIMENSION, 6);
    set_rect_matrix3(&mut sample_map, VELOCITY, 0, &(-Matrix3::identity()));
    set_rect_matrix3(&mut sample_map, ATTITUDE, 3, &(-Matrix3::identity()));
    let full_sample = full_input * &sample_map;
    let first_sample = first_input * &sample_map;
    let second_sample = second_input * sample_map;
    let sample_covariance = DMatrix::identity(6, 6) * 0.3;
    let first_covariance = &first_sample * &sample_covariance * first_sample.transpose();
    let first_cross = &first_sample * &sample_covariance;
    let split_covariance = &second * first_covariance * second.transpose()
        + &second * &first_cross * second_sample.transpose()
        + &second_sample * first_cross.transpose() * second.transpose()
        + &second_sample * &sample_covariance * second_sample.transpose();
    let full_covariance = &full_sample * sample_covariance * full_sample.transpose();
    assert!((split_covariance - full_covariance).norm() < 2.0e-13);
    assert!(full_sample.view((VELOCITY, 3), (3, 3)).norm() > 0.1);
}

#[test]
fn rotating_held_force_uses_its_whole_support_and_exact_earth_rotation() {
    let mut initial = nominal(0, 0.0);
    initial.position_ecef = [0.0, 0.0, 6_356_752.314_245];
    initial.velocity_ecef = [0.0; 3];
    initial.orientation_ecef_from_body = UnitQuaternion::IDENTITY;
    let dt = 0.01;
    let body_rate = 30.0;
    let force = 10.0;
    let (position, velocity, orientation) = integrate_held_imu(
        &initial,
        Vector3::new(force, 0.0, 0.0),
        Vector3::new(0.0, 0.0, body_rate),
        dt,
        6_378_137.0,
    )
    .unwrap();
    let rate_sum = body_rate + EARTH_RATE_RAD_S;
    let integrated_force = Vector3::new(
        force * (rate_sum * dt).sin() / rate_sum,
        force * (1.0 - (rate_sum * dt).cos()) / rate_sum,
        0.0,
    );
    let coriolis_rotation =
        NaUnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, -2.0 * EARTH_RATE_RAD_S * dt));
    let expected_velocity = coriolis_rotation * integrated_force;
    assert!((velocity.x - expected_velocity.x).abs() < 2.0e-10);
    assert!((velocity.y - expected_velocity.y).abs() < 2.0e-10);
    assert!(position.y > 4.9e-5);
    let expected_rotation = (body_rate - EARTH_RATE_RAD_S) * dt;
    assert!((orientation.rotation_vector().components()[2] - expected_rotation).abs() < 1.0e-14);
}

#[test]
fn held_sample_and_clock_update_matches_augmented_schmidt_oracle() {
    const CONSIDER: usize = 2;
    const AUGMENTED: usize = NAVIGATION_DIMENSION + CONSIDER + 6;
    let mut factor = DMatrix::identity(AUGMENTED, AUGMENTED);
    let lever_mapping = skew(&Vector3::new(0.7, -0.2, 0.4));
    set_rect_matrix3(
        &mut factor,
        VELOCITY,
        NAVIGATION_DIMENSION + CONSIDER + 3,
        &lever_mapping,
    );
    factor[(POSITION, NAVIGATION_DIMENSION)] = 0.3;
    factor[(VELOCITY + 1, NAVIGATION_DIMENSION + 1)] = -0.2;
    let prior = &factor * factor.transpose();
    let mut measurement = DMatrix::zeros(6, AUGMENTED);
    set_identity3(&mut measurement, 0, POSITION);
    set_identity3(&mut measurement, 3, VELOCITY);
    set_rect_matrix3(&mut measurement, 0, ATTITUDE, &(-lever_mapping));
    set_rect_matrix3(
        &mut measurement,
        3,
        NAVIGATION_DIMENSION + CONSIDER + 3,
        &lever_mapping,
    );
    measurement[(0, NAVIGATION_DIMENSION)] = 0.25;
    measurement[(4, NAVIGATION_DIMENSION + 1)] = -0.15;
    let noise = DMatrix::identity(6, 6) * 0.05;
    let innovation = &measurement * &prior * measurement.transpose() + &noise;
    let cross = &prior * measurement.transpose();
    let mut gain = &cross * innovation.try_inverse().unwrap();
    gain.rows_mut(NAVIGATION_DIMENSION, CONSIDER + 6).fill(0.0);
    let transform = DMatrix::identity(AUGMENTED, AUGMENTED) - &gain * &measurement;
    let expected = &transform * &prior * transform.transpose() + &gain * &noise * gain.transpose();
    let mut state = nominal(0, 0.0);
    let mut covariance = StoredCovariance {
        state: prior
            .view((0, 0), (NAVIGATION_DIMENSION, NAVIGATION_DIMENSION))
            .into_owned(),
        state_consider: prior
            .view((0, NAVIGATION_DIMENSION), (NAVIGATION_DIMENSION, CONSIDER))
            .into_owned(),
    };
    let mut sample = ActiveImuSample {
        start: SessionTime::from_ns(0),
        end: SessionTime::from_ns(1_000_000),
        covariance_body: prior
            .view(
                (
                    NAVIGATION_DIMENSION + CONSIDER,
                    NAVIGATION_DIMENSION + CONSIDER,
                ),
                (6, 6),
            )
            .into_owned(),
        state_cross: prior
            .view(
                (0, NAVIGATION_DIMENSION + CONSIDER),
                (NAVIGATION_DIMENSION, 6),
            )
            .into_owned(),
        stored_interior_cut: false,
    };
    let h_state = measurement.columns(0, NAVIGATION_DIMENSION).into_owned();
    let h_consider = measurement
        .columns(NAVIGATION_DIMENSION, CONSIDER)
        .into_owned();
    let h_sample = measurement
        .columns(NAVIGATION_DIMENSION + CONSIDER, 6)
        .into_owned();
    let consider_covariance = prior
        .view(
            (NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
            (CONSIDER, CONSIDER),
        )
        .into_owned();
    let outcome = schmidt_update_affine_with_sample(
        &mut state,
        &mut covariance,
        &mut sample,
        &h_state,
        &h_consider,
        &h_sample,
        &consider_covariance,
        &mut DVector::zeros(6),
        &noise,
        10.0,
        100.0,
        10.0,
        2,
        1.0e-8,
        &mut 0,
        None,
        1.0,
    )
    .unwrap();
    assert_eq!(outcome.disposition, InputDisposition::Fused);
    assert!(
        (&covariance.state - expected.view((0, 0), (NAVIGATION_DIMENSION, NAVIGATION_DIMENSION)))
            .norm()
            < 2.0e-14
    );
    assert!(
        (&covariance.state_consider
            - expected.view((0, NAVIGATION_DIMENSION), (NAVIGATION_DIMENSION, CONSIDER)))
        .norm()
            < 2.0e-14
    );
    assert!(
        (&sample.state_cross
            - expected.view(
                (0, NAVIGATION_DIMENSION + CONSIDER),
                (NAVIGATION_DIMENSION, 6)
            ))
        .norm()
            < 2.0e-14
    );
    let covariance_before_rejection = covariance.clone();
    let sample_before_rejection = sample.clone();
    let rejected = schmidt_update_affine_with_sample(
        &mut state,
        &mut covariance,
        &mut sample,
        &h_state,
        &h_consider,
        &h_sample,
        &consider_covariance,
        &mut DVector::from_element(6, 1.0e5),
        &noise,
        10.0,
        100.0,
        10.0,
        2,
        1.0e-8,
        &mut 0,
        None,
        1.0,
    )
    .unwrap();
    assert_eq!(
        rejected.disposition,
        InputDisposition::StatisticallyRejected
    );
    assert_eq!(covariance.state, covariance_before_rejection.state);
    assert_eq!(
        covariance.state_consider,
        covariance_before_rejection.state_consider
    );
    assert_eq!(sample.state_cross, sample_before_rejection.state_cross);
    assert_eq!(
        sample.covariance_body,
        sample_before_rejection.covariance_body
    );
}

#[test]
fn boresight_force_and_rate_jacobians_match_sensor_frame_perturbations() {
    let installation = NaUnitQuaternion::from_scaled_axis(Vector3::new(0.3, -0.2, 0.5));
    let attitude = NaUnitQuaternion::from_scaled_axis(Vector3::new(-0.4, 0.6, 0.2));
    let force_sensor = Vector3::new(1.0, -3.0, 8.0);
    let rate_sensor = Vector3::new(0.4, -0.1, 0.7);
    let (force_jacobian, rate_jacobian) = boresight_dynamics_jacobians(
        attitude.to_rotation_matrix().into_inner(),
        installation.to_rotation_matrix().into_inner(),
        installation * force_sensor,
        installation * rate_sensor,
    );
    let step = 1.0e-6;
    for axis in 0..3 {
        let mut perturbation = Vector3::zeros();
        perturbation[axis] = step;
        let plus = installation * NaUnitQuaternion::from_scaled_axis(perturbation);
        let minus = installation * NaUnitQuaternion::from_scaled_axis(-perturbation);
        let numerical_force =
            attitude * (plus * force_sensor - minus * force_sensor) / (2.0 * step);
        let numerical_rate = (plus * rate_sensor - minus * rate_sensor) / (2.0 * step);
        assert!((force_jacobian.column(axis) - numerical_force).norm() < 2.0e-9);
        assert!((rate_jacobian.column(axis) - numerical_rate).norm() < 2.0e-10);
    }
    let (_, zero_rate_jacobian) = boresight_dynamics_jacobians(
        attitude.to_rotation_matrix().into_inner(),
        installation.to_rotation_matrix().into_inner(),
        installation * force_sensor,
        Vector3::zeros(),
    );
    assert_eq!(zero_rate_jacobian, Matrix3::zeros());
}

#[test]
fn initial_velocity_sample_cross_cancels_the_same_sample_in_antenna_measurement() {
    let rotation = NaUnitQuaternion::from_scaled_axis(Vector3::new(0.2, -0.3, 0.4))
        .to_rotation_matrix()
        .into_inner();
    let lever = Vector3::new(0.4, -0.7, 0.1);
    let covariance = Matrix3::from_diagonal(&Vector3::new(0.1, 0.2, 0.3));
    let cross = initial_gyro_sample_cross(rotation, lever, covariance);
    let direct_sample_jacobian = rotation * skew(&lever);
    assert!((cross + direct_sample_jacobian * covariance).norm() < 1.0e-15);
    assert!(cross.norm() > 0.1);
}

#[test]
fn smoother_rejects_a_nonzero_sample_shared_by_multiple_stored_edges() {
    // x0=a, x1=a+s, x2=a+2s with independent unit-variance a,s.
    // Observing x2 exactly gives Var(x0|x2)=1-1/5=0.8. Marginal
    // adjacent RTS would incorrectly return 1+(0.2-2)/4=0.55.
    let correct = 1.0_f64 - 1.0 / 5.0;
    let adjacent_only = 1.0_f64 + (0.2 - 2.0) / 4.0;
    assert!((correct - adjacent_only - 0.25).abs() < 1.0e-15);
    let sample = |variance| ActiveImuSample {
        start: SessionTime::from_ns(0),
        end: SessionTime::from_ns(10),
        covariance_body: DMatrix::identity(6, 6) * variance,
        state_cross: DMatrix::zeros(NAVIGATION_DIMENSION, 6),
        stored_interior_cut: false,
    };
    let mut split = sample(1.0);
    assert_eq!(
        split.record_stored_propagation(SessionTime::from_ns(5)),
        Ok(())
    );
    assert_eq!(
        split.record_stored_propagation(SessionTime::from_ns(10)),
        Err(ProcessError::CapabilityUnavailable)
    );
    let mut aligned = sample(1.0);
    assert_eq!(
        aligned.record_stored_propagation(SessionTime::from_ns(10)),
        Ok(())
    );
    let mut deterministic = sample(0.0);
    assert_eq!(
        deterministic.record_stored_propagation(SessionTime::from_ns(5)),
        Ok(())
    );
    assert_eq!(
        deterministic.record_stored_propagation(SessionTime::from_ns(10)),
        Ok(())
    );
}

#[test]
fn interval_average_imu_covariance_has_discrete_sample_units() {
    let acceleration_sample = Matrix3::from_diagonal(&Vector3::new(2.0, 3.0, 5.0));
    let angular_rate_sample = Matrix3::from_diagonal(&Vector3::new(7.0, 11.0, 13.0));
    let dt = 0.02;
    let process = inertial_process_covariance(
        NAVIGATION_DIMENSION,
        Matrix3::zeros(),
        acceleration_sample,
        Matrix3::zeros(),
        angular_rate_sample,
        Matrix3::zeros(),
        Matrix3::zeros(),
        dt,
    )
    .unwrap();
    let block = |row, column| matrix3_from_array(array_matrix3(&process, row, column));
    assert!((block(VELOCITY, VELOCITY) - acceleration_sample * dt.powi(2)).norm() < 1.0e-15);
    assert!(
        (block(POSITION, VELOCITY) - acceleration_sample * (0.5 * dt.powi(3))).norm() < 1.0e-15
    );
    assert!(
        (block(POSITION, POSITION) - acceleration_sample * (0.25 * dt.powi(4))).norm() < 1.0e-15
    );
    assert!((block(ATTITUDE, ATTITUDE) - angular_rate_sample * dt.powi(2)).norm() < 1.0e-15);
}

#[test]
fn continuous_imu_density_and_sample_covariance_remain_additive() {
    let acceleration_density = Matrix3::from_diagonal(&Vector3::new(0.2, 0.3, 0.5));
    let acceleration_sample = Matrix3::from_diagonal(&Vector3::new(2.0, 3.0, 5.0));
    let gyroscope_density = Matrix3::from_diagonal(&Vector3::new(0.7, 1.1, 1.3));
    let gyroscope_sample = Matrix3::from_diagonal(&Vector3::new(7.0, 11.0, 13.0));
    let dt = 0.04;
    let process = inertial_process_covariance(
        NAVIGATION_DIMENSION,
        acceleration_density,
        acceleration_sample,
        gyroscope_density,
        gyroscope_sample,
        Matrix3::identity() * 0.17,
        Matrix3::identity() * 0.19,
        dt,
    )
    .unwrap();
    let block = |row, column| matrix3_from_array(array_matrix3(&process, row, column));
    assert!(
        (block(VELOCITY, VELOCITY)
            - (acceleration_density * dt + acceleration_sample * dt.powi(2)))
        .norm()
            < 1.0e-15
    );
    assert!(
        (block(POSITION, POSITION)
            - (acceleration_density * (dt.powi(3) / 3.0)
                + acceleration_sample * (dt.powi(4) / 4.0)))
            .norm()
            < 1.0e-15
    );
    assert!(
        (block(ATTITUDE, ATTITUDE) - (gyroscope_density * dt + gyroscope_sample * dt.powi(2)))
            .norm()
            < 1.0e-15
    );
    assert!(
        (block(ACCELEROMETER_BIAS, ACCELEROMETER_BIAS) - Matrix3::identity() * (0.17 * dt)).norm()
            < 1.0e-15
    );
    assert!(
        (block(GYROSCOPE_BIAS, GYROSCOPE_BIAS) - Matrix3::identity() * (0.19 * dt)).norm()
            < 1.0e-15
    );
}

#[test]
fn imu_rate_and_force_must_share_epoch_clock_and_support() {
    let good = imu_observation(interval_time(10, 5), interval_time(10, 5), false);
    assert_eq!(
        qualified_imu_support(good).unwrap(),
        QualifiedImuSupport {
            start: SessionTime::from_ns(5),
            end: SessionTime::from_ns(10),
            duration: DurationNs::from_ns(5),
            clock_model: ClockModelId::new(1),
        }
    );
}

#[test]
fn reset_imu_is_rejected_and_breaks_continuity() {
    let reset = imu_observation(interval_time(10, 5), interval_time(10, 5), true);
    assert!(rejected_imu_breaks_continuity(reset));
    assert!(matches!(
        reset.integration_eligibility(),
        ImuIntegrationEligibility::RejectDiscontinuity
    ));
}

#[test]
fn offline_imu_gap_fails_closed_without_a_synthetic_interval() {
    let held = |start, end| HeldImu {
        start: SessionTime::from_ns(start),
        time: SessionTime::from_ns(end),
        angular_rate_body: [0.0; 3],
        specific_force_body: [0.0; 3],
        accelerometer_covariance: Matrix3::identity(),
        gyroscope_covariance: Matrix3::identity(),
        degraded_input: false,
    };
    assert_eq!(
        ensure_imu_support_is_contiguous(&held(0, 5), &held(5, 10)),
        Ok(())
    );
    assert_eq!(
        ensure_imu_support_is_contiguous(&held(0, 5), &held(6, 11)),
        Err(ProcessError::IncompleteEvidence)
    );
    assert_eq!(
        ensure_imu_support_is_contiguous(&held(0, 5), &held(4, 9)),
        Err(ProcessError::InvalidEvidence)
    );
}
