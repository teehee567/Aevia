use super::*;

#[test]
fn hermite_segment_preserves_endpoints_and_derivatives() {
    let trajectory = trajectory();
    let start = trajectory
        .state_at(SessionTime::from_ns(0), ReferencePointId::new(1))
        .unwrap();
    let middle = trajectory
        .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1))
        .unwrap();
    let end = trajectory
        .state_at(
            SessionTime::from_ns(1_000_000_000),
            ReferencePointId::new(1),
        )
        .unwrap();
    assert_eq!(start.position.components(), [6_378_137.0, 0.0, 0.0]);
    assert!((middle.position.components()[1] - 5.0).abs() < 1.0e-10);
    assert_eq!(end.position.components(), [6_378_137.0, 10.0, 0.0]);
    assert!((middle.velocity.components()[1] - 10.0).abs() < 1.0e-10);
    let FieldValue::Available(acceleration) = middle.kinematic_acceleration else {
        panic!("origin translational acceleration must be available");
    };
    assert!(norm(acceleration.components()) < 1.0e-10);
}

#[test]
fn device_attachment_masks_body_claims_but_keeps_package_kinematics() {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(4));
    assert_eq!(
        trajectory.attachment_model(),
        AttachmentModel::DeviceTrajectoryOnly
    );
    for (id, kind, lever) in [
        (1, ReferencePointKind::ImuSensingCenter, [0.0; 3]),
        (
            2,
            ReferencePointKind::GnssAntennaPhaseCenter,
            [0.2, 0.0, 0.0],
        ),
        (3, ReferencePointKind::RigidBodyPoint, [0.5, 0.0, 0.0]),
        (4, ReferencePointKind::InstrumentPackage, [0.1, 0.0, 0.0]),
    ] {
        trajectory
            .add_reference_point(ReferencePoint::new(
                ReferencePointId::new(id),
                kind,
                BodyLeverArm::new(lever[0], lever[1], lever[2]).unwrap(),
                SharedParameterId::new(id),
                MeasurementUncertainty::Provided(Covariance3::ZERO),
            ))
            .unwrap();
    }
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
        // Deliberately supplied: attachment semantics must still win.
        observability: observability(),
    };
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 10.0, 0.0).unwrap(),
        ..start
    };
    trajectory.push_hermite_segment(start, end).unwrap();

    for id in [1, 2, 4] {
        let estimate = trajectory
            .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(id))
            .unwrap();
        assert_eq!(estimate.observability.heading_source, HeadingSource::None);
        assert_eq!(
            estimate.observability.heading,
            HeadingObservability::Unobservable
        );
        assert_eq!(estimate.observability.heading_variance_rad2, None);
        assert!(!estimate.observability.body_axis_quantities_available);
    }
    assert_eq!(
        trajectory.state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(3)),
        Err(QueryError::ReferencePointUnavailable)
    );
    assert!(
        trajectory
            .scalar_kinematics_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1))
            .unwrap()
            .body_longitudinal_speed_mps
            .is_none()
    );
    assert_eq!(
        trajectory.set_attachment_model(AttachmentModel::RigidBody),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn rolling_imu_attitude_uses_nominal_and_coherent_endpoint_correction() {
    let source = trajectory();
    let start = source.segments.first().unwrap().start;
    let integrated = [0.0, 0.0, 0.4];
    let correction = [0.2, 0.0, 0.0];
    let nominal =
        UnitQuaternion::from_rotation_vector(Vector3::from_components(integrated).unwrap())
            .unwrap();
    let endpoint_correction =
        UnitQuaternion::from_rotation_vector(Vector3::from_components(correction).unwrap())
            .unwrap();
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            start
                .orientation_ecef_from_body
                .quaternion()
                .multiply(nominal)
                .multiply(endpoint_correction),
        ),
        ..start
    };
    let segment = DenseSegment::new_imu_conditioned(start, end, integrated).unwrap();

    let before = segment.base_kinematics(0.499).unwrap();
    let middle = segment.base_kinematics(0.5).unwrap();
    let after = segment.base_kinematics(0.501).unwrap();
    let numerical_rate = scale(
        before
            .orientation
            .inverse()
            .multiply(after.orientation)
            .rotation_vector()
            .components(),
        500.0,
    );
    let numerical_acceleration = scale(
        sub(after.angular_rate_body, before.angular_rate_body),
        500.0,
    );
    assert!(norm(sub(middle.angular_rate_body, numerical_rate)) < 2.0e-6);
    assert!(
        norm(sub(
            middle.angular_acceleration_body,
            numerical_acceleration,
        )) < 2.0e-6
    );

    let endpoint_delta = start
        .orientation_ecef_from_body
        .quaternion()
        .inverse()
        .multiply(end.orientation_ecef_from_body.quaternion())
        .rotation_vector()
        .components();
    let direct_midpoint = start.orientation_ecef_from_body.quaternion().multiply(
        UnitQuaternion::from_rotation_vector(
            Vector3::from_components(scale(endpoint_delta, 0.5)).unwrap(),
        )
        .unwrap(),
    );
    assert!(
        norm(
            direct_midpoint
                .inverse()
                .multiply(middle.orientation)
                .rotation_vector()
                .components(),
        ) > 1.0e-3
    );
}

#[cfg(feature = "offline")]
#[test]
fn unsupported_dense_covariance_preserves_stored_means_and_endpoint_uncertainty() {
    for kind in [
        FixedRecordStoreKind::Memory,
        FixedRecordStoreKind::SeekableTemporary,
    ] {
        let (trajectory, start, end) = offline_bridge_fixture(kind, false);
        let point = ReferencePointId::new(1);
        let middle = trajectory
            .state_at(SessionTime::from_ns(500_000_000), point)
            .unwrap();
        assert_eq!(
            middle.quality.covariance,
            CovarianceConditioning::Unavailable
        );
        assert!((middle.position.components()[1] - 5.0).abs() < 1.0e-10);
        assert!((middle.velocity.components()[1] - 10.0).abs() < 1.0e-10);
        for knot in [start, end] {
            let estimate = trajectory.state_at(knot.time, point).unwrap();
            assert_eq!(estimate.position, knot.position_ecef);
            assert_eq!(estimate.velocity, knot.velocity_ecef);
            assert_eq!(
                estimate.orientation_ecef_from_body,
                knot.orientation_ecef_from_body
            );
            assert_eq!(estimate.quality.covariance, knot.quality.covariance);
            assert_eq!(estimate.covariance.position(), knot.covariance.position());
            assert_eq!(estimate.covariance.velocity(), knot.covariance.velocity());
            assert_eq!(
                estimate.covariance.attitude_error(),
                knot.covariance.attitude_error()
            );
        }
        assert!(matches!(
            trajectory.dense_bridge_linearization_at_parameter(0, 0.5),
            Err(QueryError::TrajectoryInvalid),
        ));
        assert!(matches!(
            trajectory.dense_bridge_process_cross_covariance(0, 0.25, 0.75),
            Err(QueryError::TrajectoryInvalid),
        ));
    }
}

#[test]
fn ellipsoid_horizontal_and_vertical_are_instantaneous() {
    let trajectory = trajectory();
    let time = SessionTime::from_ns(500_000_000);
    let horizontal = trajectory
        .horizontal_speed_at(time, ReferencePointId::new(1))
        .unwrap();
    let vertical = trajectory
        .vertical_speed_at(time, ReferencePointId::new(1))
        .unwrap();
    assert!((horizontal - 10.0).abs() < 2.0e-5);
    assert!(vertical.abs() < 2.0e-5);
    let course = trajectory
        .course_over_ground_at(time, ReferencePointId::new(1))
        .unwrap();
    assert!(
        matches!(course, FieldValue::Available(value) if (value - core::f64::consts::FRAC_PI_2).abs() < 1.0e-6)
    );
}

#[test]
fn offset_reference_point_uses_rotational_kinematics() {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(2));
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(2, [1.0, 0.0, 0.0]))
        .unwrap();
    let start = TrajectoryKnot {
        time: SessionTime::from_ns(0),
        position_ecef: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
        velocity_ecef: EcefVelocity::new(0.0, 0.0, 0.0).unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::IDENTITY,
        ),
        specific_force_body: BodyVector::new(0.0, 0.0, 0.0).unwrap(),
        covariance: covariance(),
        quality: quality(),
        observability: observability(),
    };
    let rotation = UnitQuaternion::from_rotation_vector(
        Vector3::new(0.0, 0.0, core::f64::consts::FRAC_PI_2).unwrap(),
    )
    .unwrap();
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(rotation),
        ..start
    };
    trajectory.push_hermite_segment(start, end).unwrap();
    let middle = trajectory
        .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(2))
        .unwrap();
    let velocity = middle.velocity.components();
    let expected = core::f64::consts::FRAC_PI_2 / 2.0_f64.sqrt();
    assert!((velocity[0] + expected).abs() < 1.0e-9);
    assert!((velocity[1] - expected).abs() < 1.0e-9);
    let FieldValue::Available(acceleration) = middle.kinematic_acceleration else {
        panic!("qualified offset acceleration must be available");
    };
    assert!(acceleration.components()[0] < 0.0);
}

#[test]
fn unavailable_angular_acceleration_is_never_published_as_numeric_zero() {
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let mut trajectory = Trajectory::new(frame, TrajectoryRevision::new(3));
    trajectory
        .set_attachment_model(AttachmentModel::RigidBody)
        .unwrap();
    trajectory
        .add_reference_point(reference(1, [0.0, 0.0, 0.0]))
        .unwrap();
    trajectory
        .add_reference_point(reference(2, [1.0, 0.0, 0.0]))
        .unwrap();
    let mut unavailable = observability();
    unavailable.angular_acceleration_available = false;
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
        observability: unavailable,
    };
    let end = TrajectoryKnot {
        time: SessionTime::from_ns(1_000_000_000),
        position_ecef: EcefPosition::new(6_378_137.0, 10.0, 0.0).unwrap(),
        ..start
    };
    trajectory.push_hermite_segment(start, end).unwrap();

    let origin = trajectory
        .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(1))
        .unwrap();
    assert_eq!(
        origin.angular_acceleration_body_relative_ecef,
        FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
    );
    assert!(matches!(
        origin.kinematic_acceleration,
        FieldValue::Available(_)
    ));
    let offset = trajectory
        .state_at(SessionTime::from_ns(500_000_000), ReferencePointId::new(2))
        .unwrap();
    assert_eq!(
        offset.angular_acceleration_body_relative_ecef,
        FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
    );
    assert_eq!(
        offset.kinematic_acceleration,
        FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
    );
}
