use super::*;

#[test]
fn sixty_minute_high_rate_offline_preflight_is_bounded_and_accounts_for_both_files() {
    let maximum_segments = 60_u64 * 60 * 1_475;
    let maximum_records = maximum_segments + 1;
    let consider = DMatrix::zeros(0, 0);
    let state =
        state_store_resource_bounds(NAVIGATION_DIMENSION, &consider, maximum_records).unwrap();
    let trajectory = Trajectory::offline_storage_bounds(maximum_segments).unwrap();
    let temporary_storage_bytes = state
        .seekable_temporary_bytes
        .checked_add(trajectory.seekable_temporary_bytes)
        .unwrap();
    assert_eq!(state.record_bytes, 15_109);
    assert_eq!(state.seekable_temporary_bytes, 80_228_805_173);
    assert_eq!(trajectory.record_bytes, 3_241);
    assert_eq!(trajectory.seekable_temporary_bytes, 17_209_710_064);
    assert_eq!(temporary_storage_bytes, 97_438_515_237);
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 256 * 1_024 * 1_024,
        temporary_storage_bytes,
        output_bytes: 1,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    assert_eq!(
        choose_offline_storage_plan(state, trajectory, limits),
        Ok(OfflineStoragePlan {
            state: StoreKind::SeekableTemporary,
            trajectory: FixedRecordStoreKind::SeekableTemporary,
        })
    );
    assert!(state.memory_peak_bytes > limits.peak_memory_bytes);
    assert!(trajectory.memory_peak_bytes > limits.peak_memory_bytes);
    assert_eq!(
        choose_offline_storage_plan(
            state,
            trajectory,
            OfflineResourceLimits {
                temporary_storage_bytes: temporary_storage_bytes - 1,
                ..limits
            },
        ),
        Err(ProcessError::StorageExhausted)
    );
}

#[test]
fn dense_bridge_process_validation_uses_block_scale_for_cancellation_residue() {
    let duration = 0.005_f64;
    let acceleration_density = [[1.0e-4, 0.0, 0.0], [0.0, 2.0e-4, 0.0], [0.0, 0.0, 3.0e-4]];
    let attitude_density = [[4.0e-5, 0.0, 0.0], [0.0, 5.0e-5, 0.0], [0.0, 0.0, 6.0e-5]];
    let zero_sample = [[0.0; 3]; 3];
    let acceleration = matrix3_from_array(acceleration_density);
    let attitude = matrix3_from_array(attitude_density);
    let mut process = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    let blocks = [
        (POSITION, POSITION, acceleration * (duration.powi(3) / 3.0)),
        (POSITION, VELOCITY, acceleration * (0.5 * duration.powi(2))),
        (VELOCITY, POSITION, acceleration * (0.5 * duration.powi(2))),
        (VELOCITY, VELOCITY, acceleration * duration),
        (ATTITUDE, ATTITUDE, attitude * duration),
    ];
    for (row, column, block) in blocks {
        process.view_mut((row, column), (3, 3)).copy_from(&block);
    }

    // An off-diagonal that is negligible at the covariance block scale
    // can arise from cancellation through an f32-qualified input path.
    process[(POSITION, VELOCITY + 1)] = 1.0e-20;
    assert_eq!(
        validate_bridge_process_model(
            &process,
            duration,
            &acceleration_density,
            &attitude_density,
            &zero_sample,
            &zero_sample,
        ),
        Ok(())
    );

    process[(POSITION, VELOCITY + 1)] = 1.0e-6;
    assert_eq!(
        validate_bridge_process_model(
            &process,
            duration,
            &acceleration_density,
            &attitude_density,
            &zero_sample,
            &zero_sample,
        ),
        Err(ProcessError::NumericalNonConvergence)
    );
}

#[test]
fn offline_event_projection_uses_both_endpoints_and_shared_consider() {
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let span = TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(1_000_000_000)).unwrap();
    let consider = DMatrix::identity(3, 3) * 0.25;
    let catalog = ConsiderCatalog {
        parameters: vec![ParameterCoordinate {
            id: SharedParameterId::new(9),
            kind: SharedParameterKind::LeverArmMetres,
            validity: span,
            start: 0,
            dimension: 3,
        }],
        clocks: Vec::new(),
        covariance: consider.clone(),
    };
    let mut planned = plan_store(NAVIGATION_DIMENSION, &consider, 2, limits).unwrap();
    let covariance = StoredCovariance {
        state: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION) * 4.0,
        state_consider: DMatrix::zeros(NAVIGATION_DIMENSION, 3),
    };
    let mut first = step(0, 0.0, 4.0, 4.0);
    first.predicted_covariance = covariance.clone();
    first.filtered_covariance = covariance.clone();
    first.smoothed_covariance = Some(covariance.clone());
    first.smoothed = Some(nominal(0, 0.0));
    first.consider_transition = DMatrix::zeros(NAVIGATION_DIMENSION, 3);
    let mut backward = DMatrix::identity(NAVIGATION_DIMENSION + 3, NAVIGATION_DIMENSION + 3);
    for index in 0..NAVIGATION_DIMENSION {
        backward[(index, index)] = 0.5;
    }
    first.smoothed_backward_gain = Some(backward);
    let mut second = step(1_000_000_000, 0.0, 4.0, 4.0);
    second.predicted_covariance = covariance.clone();
    second.filtered_covariance = covariance.clone();
    second.smoothed_covariance = Some(covariance);
    second.smoothed = Some(nominal(1_000_000_000, 0.0));
    second.consider_transition = DMatrix::zeros(NAVIGATION_DIMENSION, 3);
    planned.store.push(&first).unwrap();
    planned.store.push(&second).unwrap();

    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let point = ReferencePoint::new(
        ReferencePointId::new(1),
        ReferencePointKind::RigidBodyPoint,
        BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
        SharedParameterId::new(9),
        MeasurementUncertainty::Provided(Covariance3::diagonal(0.25, 0.25, 0.25).unwrap()),
    );
    let quality = EstimateQuality {
        stage: EstimateStage::Finalized,
        validity: Validity::Nominal,
        gnss: GnssState::Fixed,
        timing: TimingQuality::PpsCorrelated,
        integrity: Integrity::Monitored,
        covariance: CovarianceConditioning::UnconditionalModel,
        imu_gap: false,
        degraded_input: false,
    };
    let observability = ObservabilityReport {
        heading_source: HeadingSource::Supplied,
        heading: HeadingObservability::Supplied,
        heading_variance_rad2: Some(4.0),
        course_available: true,
        body_axis_quantities_available: true,
        angular_acceleration_available: true,
    };
    let semantic_covariance = KinematicCovariance::new(
        Covariance3::diagonal(4.0, 4.0, 4.0).unwrap(),
        Covariance3::diagonal(4.0, 4.0, 4.0).unwrap(),
        None,
        Covariance3::diagonal(4.0, 4.0, 4.0).unwrap(),
    )
    .unwrap();
    let knot = |time| TrajectoryKnot {
        time,
        position_ecef: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 0.0, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: semantic_covariance,
        quality,
        observability,
    };
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(1));
    trajectory
        .set_attachment_model(crate::config::AttachmentModel::RigidBody)
        .unwrap();
    trajectory.add_reference_point(point).unwrap();
    trajectory
        .prepare_offline_storage(1, FixedRecordStoreKind::Memory)
        .unwrap();
    let mut endpoint_joint_covariance = Box::new([[0.0; 18]; 18]);
    for coordinate in 0..18 {
        endpoint_joint_covariance[coordinate][coordinate] = 4.0;
    }
    trajectory
        .push_offline_conditional_bridge_segment(
            knot(SessionTime::from_ns(0)),
            knot(SessionTime::from_ns(1_000_000_000)),
            DenseBridgeInput {
                covariance_available: true,
                endpoint_joint_covariance,
                acceleration_spectral_density_ecef: [
                    [1.0e-12, 0.0, 0.0],
                    [0.0, 1.0e-12, 0.0],
                    [0.0, 0.0, 1.0e-12],
                ],
                attitude_spectral_density_body: [
                    [1.0e-12, 0.0, 0.0],
                    [0.0, 1.0e-12, 0.0],
                    [0.0, 0.0, 1.0e-12],
                ],
                acceleration_interval_average_covariance_ecef: [[0.0; 3]; 3],
                angular_rate_interval_average_covariance_body: [[0.0; 3]; 3],
                reintegrated_position_ecef_m: [6_378_137.0, 0.0, 0.0],
                reintegrated_velocity_ecef_mps: [0.0; 3],
                integrated_rotation_body: [0.0; 3],
            },
            (0, 1),
        )
        .unwrap();
    trajectory.finish_offline_storage().unwrap();
    let reference_points = [point];
    let mut provider = OfflineMetricUncertainty::new(
        planned.store.as_mut(),
        &reference_points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    let event = |parameter: f64, time: i64| EventTimeSensitivity {
        segment_index: 0,
        parameter,
        time: SessionTime::from_ns(time),
        reference_point: point.id(),
        state: StateSensitivity {
            position: [1.0, 0.0, 0.0],
            velocity: [0.0; 3],
            attitude: [0.0; 3],
        },
        gate: Some(GateId::new(1)),
        gate_survey_coefficient_s_per_m: 1.0,
        gate_survey_variance_m2: Some(0.0),
    };
    let midpoint = event(0.5, 500_000_000);
    let FieldValue::Available(midpoint_variance) =
        provider.event_time_variance_s2(&trajectory, &midpoint)
    else {
        panic!("midpoint variance unavailable");
    };
    assert!((midpoint_variance - 3.3125).abs() < 1.0e-12);
    let start = event(0.0, 0);
    let end = event(1.0, 1_000_000_000);
    let FieldValue::Available(start_variance) =
        provider.event_time_variance_s2(&trajectory, &start)
    else {
        panic!("start variance unavailable");
    };
    let FieldValue::Available(end_variance) = provider.event_time_variance_s2(&trajectory, &end)
    else {
        panic!("end variance unavailable");
    };
    let FieldValue::Available(cross) =
        provider.event_time_cross_covariance_s2(&trajectory, &start, &end)
    else {
        panic!("event cross covariance unavailable");
    };
    assert!((start_variance - 4.25).abs() < 1.0e-12);
    assert!((end_variance - 4.25).abs() < 1.0e-12);
    assert!((cross - 2.25).abs() < 1.0e-12);
    assert!((start_variance + end_variance - 2.0 * cross - 4.0).abs() < 1.0e-12);

    let uncertain_survey = EventTimeSensitivity {
        gate_survey_variance_m2: Some(0.01),
        ..midpoint
    };
    assert_eq!(
        provider.event_time_variance_s2(&trajectory, &uncertain_survey),
        FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
    );
}
