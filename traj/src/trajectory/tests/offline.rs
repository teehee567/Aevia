use super::*;

#[cfg(feature = "offline")]
#[test]
fn conditional_bridge_uses_joint_endpoints_and_vanishes_at_boundaries() {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(7));
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(1, [0.0; 3]))
        .unwrap();
    let start = TrajectoryKnot {
        time: SessionTime::ZERO,
        position_ecef: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: covariance(),
        quality: quality(),
        observability: observability(),
    };
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 10.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 10.0, 0.0).unwrap(),
        ..start
    };
    let endpoint_variances = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01];
    let mut joint = Box::new([[0.0; 18]; 18]);
    for axis in 0..9 {
        joint[axis][axis] = endpoint_variances[axis];
        joint[axis + 9][axis + 9] = endpoint_variances[axis];
        joint[axis][axis + 9] = 0.5 * endpoint_variances[axis];
        joint[axis + 9][axis] = 0.5 * endpoint_variances[axis];
    }
    trajectory
        .push_conditional_bridge_segment(
            start,
            end,
            DenseBridgeInput {
                covariance_available: true,
                endpoint_joint_covariance: joint,
                acceleration_spectral_density_ecef: [
                    [0.1, 0.0, 0.0],
                    [0.0, 0.1, 0.0],
                    [0.0, 0.0, 0.1],
                ],
                attitude_spectral_density_body: [
                    [0.01, 0.0, 0.0],
                    [0.0, 0.01, 0.0],
                    [0.0, 0.0, 0.01],
                ],
                acceleration_interval_average_covariance_ecef: [[0.0; 3]; 3],
                angular_rate_interval_average_covariance_body: [[0.0; 3]; 3],
                reintegrated_position_ecef_m: [6_378_137.0, 10.0, 0.0],
                reintegrated_velocity_ecef_mps: [0.0, 10.0, 0.0],
                integrated_rotation_body: [0.01, 0.02, 0.03],
            },
        )
        .unwrap();
    let first_process = trajectory
        .dense_bridge_process_cross_covariance(0, 0.0, 0.5)
        .unwrap();
    let last_process = trajectory
        .dense_bridge_process_cross_covariance(0, 0.5, 1.0)
        .unwrap();
    assert_eq!(first_process, DMatrix::zeros(9, 9));
    assert_eq!(last_process, DMatrix::zeros(9, 9));
    let middle = trajectory
        .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1))
        .unwrap();
    assert_ne!(
        middle.quality.covariance,
        CovarianceConditioning::Unavailable
    );
    assert!(middle.covariance.position().variance(0).unwrap() > 0.0);
    assert_ne!(middle.covariance.position().variance(0).unwrap(), 1.0);
    let terminal = trajectory
        .state_at(
            SessionTime::from_ns(1_000_000_000),
            ReferencePointId::new(1),
        )
        .unwrap();
    assert_eq!(terminal.position, end.position_ecef);
    assert_eq!(terminal.velocity, end.velocity_ecef);
    assert_eq!(
        terminal.orientation_ecef_from_body,
        end.orientation_ecef_from_body
    );
}

#[cfg(feature = "offline")]
#[test]
fn offline_backing_query_and_metric_match_resident_dense_trajectory() {
    let (backed, start, end) = offline_conditional_fixture(FixedRecordStoreKind::Memory);
    let mut resident = Trajectory::new(backed.frame(), backed.revision());
    resident
        .set_attachment_model(backed.attachment_model())
        .unwrap();
    resident
        .add_reference_point(reference(1, [0.0; 3]))
        .unwrap();
    let lease = backed.segment_lease(0).unwrap();
    let bridge = lease.conditional_bridge().unwrap();
    resident.segments.push(*lease.segment());
    resident
        .conditional_bridges
        .push(Some(Box::new(bridge.clone())));

    for time in [start.time, SessionTime::from_ns(500_000_000), end.time] {
        assert_eq!(
            backed.state_at(time, ReferencePointId::new(1)).unwrap(),
            resident.state_at(time, ReferencePointId::new(1)).unwrap()
        );
    }
    let mut plan = MetricPlan::new(7);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::Spatial3d,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-8,
        relative_tolerance: 1.0e-8,
    }))
    .unwrap();
    assert_eq!(
        backed.measure(&plan).unwrap(),
        resident.measure(&plan).unwrap()
    );
    assert_eq!(backed.offline_segment_store_indices(0).unwrap(), (7, 8));
}

#[cfg(feature = "offline")]
#[test]
fn offline_backing_corruption_is_an_explicit_query_failure() {
    let (trajectory, _, _) = offline_conditional_fixture(FixedRecordStoreKind::SeekableTemporary);
    trajectory.corrupt_offline_record_for_test(0, 17).unwrap();
    assert!(matches!(
        trajectory.state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1)),
        Err(QueryError::BackingStoreFailure)
    ));
}

#[cfg(feature = "offline")]
#[test]
fn offline_seekable_backing_is_removed_with_the_last_trajectory_clone() {
    let (trajectory, _, _) = offline_conditional_fixture(FixedRecordStoreKind::SeekableTemporary);
    let path = trajectory.offline_backing_path_for_test().unwrap();
    let clone = trajectory.clone();
    drop(trajectory);
    assert!(path.exists());
    drop(clone);
    assert!(!path.exists());
}

#[cfg(feature = "offline")]
#[test]
fn sixty_minute_high_rate_trajectory_has_constant_resident_cache_bound() {
    let segments = 60_u64 * 60 * 1_475;
    let bounds = Trajectory::offline_storage_bounds(segments).unwrap();
    assert_eq!(
        bounds.memory_peak_bytes - bounds.seekable_peak_bytes,
        bounds.record_bytes * segments
    );
    assert_eq!(
        bounds.seekable_temporary_bytes,
        FIXED_RECORD_HEADER_BYTES + bounds.record_bytes * segments
    );
    assert!(bounds.seekable_peak_bytes < 128 * 1_024);
    assert!(bounds.memory_peak_bytes > 15 * 1_024 * 1_024 * 1_024);
}
