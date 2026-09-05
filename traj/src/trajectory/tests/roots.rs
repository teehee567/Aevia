use super::*;

#[test]
fn offset_gate_isolator_finds_narrow_rotational_pair() {
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
        Vector3::new(0.0, 0.0, core::f64::consts::PI).unwrap(),
    )
    .unwrap();
    trajectory
        .push_hermite_segment(
            start,
            TrajectoryKnot {
                time: SessionTime::from_ns(1_000_000_000),
                orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(rotation),
                ..start
            },
        )
        .unwrap();

    // The crossings are about 9e-5 of a segment apart, both inside one
    // cell of the removed 64-cell sampler.
    let target = 1.0 - 1.0e-8;
    let roots = trajectory
        .gate_roots(
            0,
            ReferencePointId::new(2),
            [0.0, target, 0.0],
            [0.0, 1.0, 0.0],
            1.0e-9,
            1.0e-12,
        )
        .unwrap();
    let first = crate::scalar_math::asin(target) / core::f64::consts::PI;
    assert_eq!(roots.len(), 2);
    assert!((roots[0] - first).abs() <= 1.0e-8);
    assert!((roots[1] - (1.0 - first)).abs() <= 1.0e-8);
}

#[test]
fn live_root_budget_is_fixed_and_bounded_by_the_implementation_ceiling() {
    let mut trajectory = trajectory();
    assert_eq!(
        trajectory.set_root_evaluation_budget(0),
        Err(ValidationError::CapacityExceeded)
    );
    assert_eq!(
        trajectory.set_root_evaluation_budget(MAX_ROOT_ISOLATION_EVALUATIONS + 1),
        Err(ValidationError::CapacityExceeded)
    );
    assert_eq!(trajectory.set_root_evaluation_budget(37), Ok(()));
    assert_eq!(trajectory.root_evaluation_budget, 37);
}

#[test]
fn interval_isolator_finds_two_narrow_off_grid_roots() {
    // Both roots lie in one cell of the removed 64-sample scan, and none
    // is a cell endpoint or midpoint.
    let first = 0.500_001_3;
    let second = 0.500_011_7;
    let roots = isolate_enclosed_roots(
        0.0,
        1.0,
        1.0e-9,
        1.0e-14,
        EndpointOwnership {
            lower: true,
            upper: true,
        },
        MAX_ROOT_ISOLATION_EVALUATIONS,
        quadratic_oracle(first, second),
    )
    .unwrap();
    assert_eq!(roots.len(), 2);
    assert!((roots[0] - first).abs() <= 1.0e-9);
    assert!((roots[1] - second).abs() <= 1.0e-9);
}

#[test]
fn interval_isolator_recovers_off_grid_even_multiplicity_contact() {
    let contact = 0.314_159_265_35;
    let roots = isolate_enclosed_roots(
        0.0,
        1.0,
        1.0e-10,
        1.0e-18,
        EndpointOwnership {
            lower: true,
            upper: true,
        },
        MAX_ROOT_ISOLATION_EVALUATIONS,
        quadratic_oracle(contact, contact),
    )
    .unwrap();
    assert_eq!(roots.len(), 1);
    assert!((roots[0] - contact).abs() <= 1.0e-9);
}

#[test]
fn interval_isolator_budget_exhaustion_is_not_false_no_root() {
    let unresolved = |_lower: f64, _upper: f64| {
        Ok(ScalarEnclosure {
            value_estimate: 1.0,
            derivative_estimate: 0.0,
            value: OutwardInterval::new(-1.0, 1.0)?,
            derivative: OutwardInterval::new(-1.0, 1.0)?,
        })
    };
    assert_eq!(
        isolate_enclosed_roots(
            0.0,
            1.0,
            1.0e-12,
            0.0,
            EndpointOwnership {
                lower: true,
                upper: true,
            },
            8,
            unresolved,
        ),
        Err(MetricError::EvaluationBudgetExceeded)
    );
}

#[test]
fn shared_endpoint_is_owned_only_by_right_hand_interval() {
    let oracle = |lower: f64, upper: f64| {
        let x = midpoint(lower, upper);
        taylor_enclosure(
            ScalarJet {
                value: x - 0.5,
                derivative: 1.0,
                second_derivative: 0.0,
                value_roundoff: 0.0,
                derivative_roundoff: 0.0,
                second_derivative_roundoff: 0.0,
            },
            0.0,
            lower,
            upper,
        )
    };
    let left = isolate_enclosed_roots(
        0.0,
        0.5,
        1.0e-12,
        0.0,
        EndpointOwnership {
            lower: true,
            upper: false,
        },
        128,
        oracle,
    )
    .unwrap();
    let right = isolate_enclosed_roots(
        0.5,
        1.0,
        1.0e-12,
        0.0,
        EndpointOwnership {
            lower: true,
            upper: true,
        },
        128,
        oracle,
    )
    .unwrap();
    assert!(left.is_empty());
    assert_eq!(right.as_slice(), &[0.5]);
}
