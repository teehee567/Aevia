use super::*;

const DIMENSION: usize = 21;

fn model() -> crate::trajectory::CoupledDenseBridge {
    let mut continuous = DMatrix::zeros(DIMENSION, DIMENSION);
    let mut noise_density = DMatrix::zeros(DIMENSION, DIMENSION);
    let mut rate_mapping = DMatrix::zeros(3, DIMENSION);
    for axis in 0..3 {
        continuous[(axis, axis + 3)] = 1.0;
        continuous[(axis + 3, axis + 9)] = -1.0;
        continuous[(axis + 3, axis + 15)] = 1.0;
        continuous[(axis + 6, axis + 12)] = -1.0;
        continuous[(axis + 6, axis + 18)] = -1.0;
        noise_density[(axis + 3, axis + 3)] = 0.1;
        noise_density[(axis + 6, axis + 6)] = 0.01;
        rate_mapping[(axis, axis + 12)] = -1.0;
        rate_mapping[(axis, axis + 18)] = -1.0;
    }
    let mut joint = DMatrix::identity(2 * DIMENSION, 2 * DIMENSION);
    for axis in 0..DIMENSION {
        joint[(axis, axis + DIMENSION)] = 0.25;
        joint[(axis + DIMENSION, axis)] = 0.25;
    }
    crate::trajectory::CoupledDenseBridge {
        duration_seconds: 1.0,
        state_dimension: 15,
        continuous,
        noise_density,
        endpoint_joint: joint,
        start_to_reference: DMatrix::identity(DIMENSION, DIMENSION),
        end_to_reference: DMatrix::identity(DIMENSION, DIMENSION),
        reference_start_orientation: [1.0, 0.0, 0.0, 0.0],
        reference_body_rate: [0.01, 0.02, 0.03],
        rate_mapping,
        gyro_density: [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]],
        parameter_ids: std::vec![0; DIMENSION],
        cache: Default::default(),
    }
}

fn fixture(kind: FixedRecordStoreKind) -> Trajectory {
    let (legacy, start, end) = offline_conditional_fixture(FixedRecordStoreKind::Memory);
    let lease = legacy.segment_lease(0).unwrap();
    let original = lease.conditional_bridge().unwrap();
    let mut trajectory = Trajectory::new(legacy.frame(), legacy.revision());
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(1, [0.0; 3]))
        .unwrap();
    trajectory
        .add_reference_point(reference(2, [0.4, 0.2, 0.1]))
        .unwrap();
    trajectory
        .prepare_offline_storage_with_covariance(1, kind, DIMENSION)
        .unwrap();
    trajectory
        .push_offline_conditional_bridge_segment(
            start,
            end,
            DenseBridgeInput {
                coupled: Some(Box::new(model())),
                covariance_available: true,
                endpoint_joint_covariance: original.endpoint_joint_covariance.clone(),
                acceleration_spectral_density_ecef: original.acceleration_spectral_density_ecef,
                attitude_spectral_density_body: original.attitude_spectral_density_body,
                acceleration_interval_average_covariance_ecef: original
                    .acceleration_interval_average_covariance_ecef,
                angular_rate_interval_average_covariance_body: original
                    .angular_rate_interval_average_covariance_body,
                reintegrated_position_ecef_m: end.position_ecef.components(),
                reintegrated_velocity_ecef_mps: end.velocity_ecef.components(),
                integrated_rotation_body: [0.01, 0.02, 0.03],
            },
            (7, 8),
        )
        .unwrap();
    trajectory.finish_offline_storage().unwrap();
    trajectory
}

#[test]
fn coupled_storage_memory_and_seekable_preserve_queries_and_joint_sensitivities() {
    let memory = fixture(FixedRecordStoreKind::Memory);
    let seekable = fixture(FixedRecordStoreKind::SeekableTemporary);
    for point in [ReferencePointId::new(1), ReferencePointId::new(2)] {
        for time in [0, 250_000_000, 500_000_000, 1_000_000_000] {
            let time = SessionTime::from_ns(time);
            assert_eq!(
                memory.state_at(time, point).unwrap(),
                seekable.state_at(time, point).unwrap()
            );
        }
        let first = memory.dense_point_linearization(0, 0.25, point).unwrap();
        let second = seekable.dense_point_linearization(0, 0.25, point).unwrap();
        assert_eq!(first.start_jacobian, second.start_jacobian);
        assert_eq!(first.end_jacobian, second.end_jacobian);
        assert_eq!(first.start_jacobian.shape(), (9, DIMENSION));
        assert_eq!(
            memory
                .dense_point_process_cross(0, 0.25, 0.75, point, ReferencePointId::new(2))
                .unwrap(),
            seekable
                .dense_point_process_cross(0, 0.25, 0.75, point, ReferencePointId::new(2))
                .unwrap(),
        );
    }
    assert_eq!(seekable.offline_segment_store_indices(0).unwrap(), (7, 8));
    let path = seekable.offline_backing_path_for_test().unwrap();
    let bounds = Trajectory::offline_storage_bounds_with_covariance(1, DIMENSION).unwrap();
    assert_eq!(bounds.record_bytes, 32_289);
    assert_eq!(
        std::fs::metadata(path).unwrap().len(),
        bounds.seekable_temporary_bytes
    );
    // This byte belongs to the added coupled payload, beyond every legacy
    // segment field. Its corruption must be detected before querying it.
    seekable
        .corrupt_offline_record_for_test(0, bounds.record_bytes - 8)
        .unwrap();
    assert!(matches!(
        seekable.state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1)),
        Err(QueryError::BackingStoreFailure)
    ));
}

#[test]
fn coupled_storage_bounds_include_full_covariance_records_and_constant_cache() {
    let small = Trajectory::offline_storage_bounds_with_covariance(1, DIMENSION).unwrap();
    let large = Trajectory::offline_storage_bounds_with_covariance(5_310_000, DIMENSION).unwrap();
    assert_eq!(small.seekable_peak_bytes, large.seekable_peak_bytes);
    assert_eq!(
        large.memory_peak_bytes - large.seekable_peak_bytes,
        5_310_000 * 32_289
    );
    assert_eq!(
        large.seekable_temporary_bytes,
        FIXED_RECORD_HEADER_BYTES + 5_310_000 * 32_289
    );
    assert!(Trajectory::offline_storage_bounds_with_covariance(1, usize::MAX).is_err());
}

#[test]
fn coupled_storage_rejects_malformed_covariance_shapes_and_noise() {
    let valid = model();
    assert_eq!(valid.validate(), Ok(()));
    for mutate in [
        |model: &mut crate::trajectory::CoupledDenseBridge| {
            model.continuous = DMatrix::zeros(DIMENSION, DIMENSION - 1)
        },
        |model: &mut crate::trajectory::CoupledDenseBridge| {
            model.rate_mapping = DMatrix::zeros(2, DIMENSION)
        },
        |model: &mut crate::trajectory::CoupledDenseBridge| model.noise_density[(0, 0)] = -1.0,
        |model: &mut crate::trajectory::CoupledDenseBridge| model.endpoint_joint[(0, 1)] = 0.5,
        |model: &mut crate::trajectory::CoupledDenseBridge| {
            let _ = model.parameter_ids.pop();
        },
        |model: &mut crate::trajectory::CoupledDenseBridge| model.duration_seconds = 0.0,
        |model: &mut crate::trajectory::CoupledDenseBridge| model.state_dimension = usize::MAX,
    ] {
        let mut invalid = valid.clone();
        mutate(&mut invalid);
        assert!(invalid.validate().is_err());
    }
}

#[test]
fn coupled_storage_rejects_nonfinite_gyro_density_and_nonunit_reference_rotation() {
    let mut invalid = model();
    invalid.gyro_density[0][0] = f64::NAN;
    assert!(
        invalid.validate().is_err(),
        "gyro covariance must be finite"
    );
    let mut invalid = model();
    invalid.gyro_density[0][0] = 0.02;
    assert!(
        invalid.validate().is_err(),
        "gyro-integral covariance must agree with its attitude-noise cross block"
    );
    let mut invalid = model();
    invalid.reference_start_orientation = [2.0, 0.0, 0.0, 0.0];
    assert!(
        invalid.validate().is_err(),
        "reference quaternion must be unit length"
    );
}
