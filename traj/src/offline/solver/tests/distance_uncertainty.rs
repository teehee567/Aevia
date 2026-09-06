use super::*;
use crate::{
    ids::MetricDefinitionId,
    metric::{DistancePlan, DistanceQuantity},
};

fn distance_fixture() -> (
    Trajectory,
    crate::offline::store::PlannedStore,
    ConsiderCatalog,
    [ReferencePoint; 1],
) {
    distance_fixture_with_lever(None)
}

fn distance_fixture_with_lever(
    lever: Option<[f64; 3]>,
) -> (
    Trajectory,
    crate::offline::store::PlannedStore,
    ConsiderCatalog,
    [ReferencePoint; 1],
) {
    distance_fixture_with_motion(lever, None)
}

fn distance_fixture_with_motion(
    lever: Option<[f64; 3]>,
    reversal: Option<f64>,
) -> (
    Trajectory,
    crate::offline::store::PlannedStore,
    ConsiderCatalog,
    [ReferencePoint; 1],
) {
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let m = if lever.is_some() { 3 } else { 1 };
    let d = NAVIGATION_DIMENSION + m + 6;
    let mut consider = DMatrix::zeros(m, m);
    if lever.is_some() {
        consider[(0, 0)] = 0.04;
        consider[(1, 1)] = 0.09;
        consider[(2, 2)] = 0.16;
    }
    let span = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(2_000_000_000)).unwrap();
    let parameters = if lever.is_some() {
        vec![ParameterCoordinate {
            id: SharedParameterId::new(9),
            kind: SharedParameterKind::LeverArmMetres,
            validity: span,
            start: 0,
            dimension: 3,
        }]
    } else {
        Vec::new()
    };
    let catalog = ConsiderCatalog {
        parameters,
        clocks: Vec::new(),
        covariance: consider.clone(),
    };
    let mut store = plan_store(NAVIGATION_DIMENSION, &consider, 3, limits).unwrap();
    for index in 0..3 {
        let time = index * 1_000_000_000;
        let mut current = step(time, 0.0, 4.0, 4.0);
        current.filtered_covariance.state_consider = DMatrix::zeros(NAVIGATION_DIMENSION, m);
        if lever.is_some() || reversal.is_some() {
            current.filtered_covariance.state.fill(0.0);
        }
        if reversal.is_some() {
            let time = index as f64;
            current.filtered_covariance.state[(POSITION, POSITION)] = time * time;
            current.filtered_covariance.state[(POSITION, VELOCITY)] = time;
            current.filtered_covariance.state[(VELOCITY, POSITION)] = time;
            current.filtered_covariance.state[(VELOCITY, VELOCITY)] = 1.0;
        }
        current.predicted_covariance = current.filtered_covariance.clone();
        current.consider_transition = DMatrix::zeros(NAVIGATION_DIMENSION, m);
        current.predicted_sample = StoredImuSample::zeros(NAVIGATION_DIMENSION, m);
        current.filtered_sample = StoredImuSample::zeros(NAVIGATION_DIMENSION, m);
        current.smoothed_covariance = Some(current.filtered_covariance.clone());
        current.smoothed_sample = Some(StoredImuSample::zeros(NAVIGATION_DIMENSION, m));
        current.smoothed = Some(current.filtered.clone());
        if index != 2 {
            current.smoothed_backward_gain = Some(if reversal.is_some() {
                let time = index as f64;
                let mut gain = DMatrix::zeros(d, d);
                let denominator = 1.0 + (time + 1.0).powi(2);
                for (row, a) in [(POSITION, time), (VELOCITY, 1.0)] {
                    for (column, b) in [(POSITION, time + 1.0), (VELOCITY, 1.0)] {
                        gain[(row, column)] = a * b / denominator;
                    }
                }
                gain
            } else {
                DMatrix::identity(d, d) * if lever.is_some() { 1.0 } else { 0.5 }
            });
        }
        store.store.push(&current).unwrap();
    }
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let point = ReferencePoint::new(
        ReferencePointId::new(1),
        if lever.is_some() {
            ReferencePointKind::RigidBodyPoint
        } else {
            ReferencePointKind::ImuSensingCenter
        },
        BodyLeverArm::new(
            lever.unwrap_or([0.0; 3])[0],
            lever.unwrap_or([0.0; 3])[1],
            lever.unwrap_or([0.0; 3])[2],
        )
        .unwrap(),
        SharedParameterId::new(9),
        MeasurementUncertainty::Provided(if lever.is_some() {
            Covariance3::diagonal(0.04, 0.09, 0.16).unwrap()
        } else {
            Covariance3::ZERO
        }),
    );
    let quality = EstimateQuality {
        stage: EstimateStage::Finalized,
        validity: Validity::Nominal,
        gnss: GnssState::Healthy,
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
    let speed = if lever.is_some() { 0.0 } else { 10.0 };
    let rotation_rate = if lever.is_some() { 0.6 } else { 0.0 };
    let knot = |index: i64| TrajectoryKnot {
        time: SessionTime::from_ns(index * 1_000_000_000),
        position_ecef: EcefPosition::new(
            6_378_137.0
                + reversal.map_or(index as f64 * speed, |zero| {
                    0.5 * (index as f64).powi(2) - zero * index as f64
                }),
            0.0,
            0.0,
        )
        .unwrap(),
        velocity_ecef: EcefVelocity::new(
            reversal.map_or(speed, |zero| index as f64 - zero),
            0.0,
            0.0,
        )
        .unwrap(),
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
            UnitQuaternion::from_wxyz([
                (index as f64 * rotation_rate * 0.5).cos(),
                0.0,
                0.0,
                (index as f64 * rotation_rate * 0.5).sin(),
            ])
            .unwrap(),
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
        .prepare_offline_storage_with_covariance(
            2,
            FixedRecordStoreKind::Memory,
            if lever.is_some() || reversal.is_some() {
                d
            } else {
                0
            },
        )
        .unwrap();
    for index in 0..2 {
        let mut endpoint_joint_covariance = Box::new([[0.0; 18]; 18]);
        for coordinate in 0..18 {
            endpoint_joint_covariance[coordinate][coordinate] = 4.0;
        }
        let coupled = (lever.is_some() || reversal.is_some()).then(|| {
            let mut joint = DMatrix::zeros(2 * d, 2 * d);
            for left in 0..2 {
                for right in 0..2 {
                    joint
                        .view_mut((left * d + 15, right * d + 15), (m, m))
                        .copy_from(&consider);
                }
            }
            if reversal.is_some() {
                for left in 0..2 {
                    for right in 0..2 {
                        let left_time = (index + left as i64) as f64;
                        let right_time = (index + right as i64) as f64;
                        for (row, a) in [(POSITION, left_time), (VELOCITY, 1.0)] {
                            for (column, b) in [(POSITION, right_time), (VELOCITY, 1.0)] {
                                joint[(left * d + row, right * d + column)] = a * b;
                            }
                        }
                    }
                }
            }
            let mut continuous = DMatrix::zeros(d, d);
            if reversal.is_some() {
                continuous[(POSITION, VELOCITY)] = 1.0;
            }
            let mut parameter_ids = vec![0; d];
            parameter_ids[15] = 9;
            crate::trajectory::CoupledDenseBridge {
                duration_seconds: 1.0,
                state_dimension: NAVIGATION_DIMENSION,
                continuous,
                noise_density: DMatrix::zeros(d, d),
                endpoint_joint: joint,
                start_to_reference: DMatrix::identity(d, d),
                end_to_reference: DMatrix::identity(d, d),
                reference_start_orientation: knot(index)
                    .orientation_ecef_from_body
                    .quaternion()
                    .components_wxyz(),
                reference_body_rate: [0.0, 0.0, rotation_rate],
                rate_mapping: DMatrix::zeros(3, d),
                gyro_density: [[0.0; 3]; 3],
                parameter_ids,
                cache: Default::default(),
            }
        });
        trajectory
            .push_offline_conditional_bridge_segment(
                knot(index),
                knot(index + 1),
                DenseBridgeInput {
                    coupled: coupled.map(Box::new),
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
                    reintegrated_position_ecef_m: knot(index + 1).position_ecef.components(),
                    reintegrated_velocity_ecef_mps: knot(index + 1).velocity_ecef.components(),
                    integrated_rotation_body: [0.0, 0.0, rotation_rate],
                },
                (index as u64, index as u64 + 1),
            )
            .unwrap();
    }
    trajectory.finish_offline_storage().unwrap();
    (trajectory, store, catalog, [point])
}

fn plan(quantity: DistanceQuantity) -> DistancePlan {
    DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-6,
        relative_tolerance: 1.0e-6,
    }
}

#[test]
fn partial_distance_uncertainty_matches_correlated_endpoint_displacement() {
    let (trajectory, mut store, catalog, points) = distance_fixture();
    let mut provider = OfflineMetricUncertainty::new(
        store.store.as_mut(),
        &points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    let partial = TimeSpan::new(
        SessionTime::from_ns(250_000_000),
        SessionTime::from_ns(750_000_000),
    )
    .unwrap();
    let variance = provider
        .integrated_distance_variance_m2(&trajectory, plan(DistanceQuantity::Spatial3d), partial)
        .unwrap();
    // Hermite endpoint displacement weights are ±0.6875 on position and
    // -0.09375 on each endpoint velocity; each marginal is 4 and cross is 2.
    assert!((variance - 1.996_093_75).abs() < 1.0e-10, "{variance}");
}

#[test]
fn full_distance_uses_cross_covariance_across_multiple_segments() {
    let (trajectory, mut store, catalog, points) = distance_fixture();
    let mut provider = OfflineMetricUncertainty::new(
        store.store.as_mut(),
        &points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    for quantity in [
        DistanceQuantity::Spatial3d,
        DistanceQuantity::BodyLongitudinalSigned,
        DistanceQuantity::BodyLongitudinalAbsolute,
    ] {
        let variance = provider
            .integrated_distance_variance_m2(
                &trajectory,
                plan(quantity),
                trajectory.span().unwrap(),
            )
            .unwrap();
        // 4 + 4 - 2 Cov(p0,p2), with Cov(p0,p2)=0.5²*4.
        assert!((variance - 6.0).abs() < 1.0e-10, "{quantity:?}: {variance}");
    }
}

#[test]
fn rotating_offset_distance_retains_shared_lever_uncertainty_and_matches_perturbations() {
    let lever = [1.0, 2.0, 0.5];
    let (trajectory, mut store, catalog, points) = distance_fixture_with_lever(Some(lever));
    let mut provider = OfflineMetricUncertainty::new(
        store.store.as_mut(),
        &points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    let span = TimeSpan::new(
        SessionTime::from_ns(250_000_000),
        SessionTime::from_ns(1_750_000_000),
    )
    .unwrap();
    let variance = provider
        .integrated_distance_variance_m2(&trajectory, plan(DistanceQuantity::Spatial3d), span)
        .unwrap();
    let mut numeric_gradient = Vector3::zeros();
    for axis in 0..3 {
        let length = |delta: f64| {
            let mut shifted = lever;
            shifted[axis] += delta;
            let (trajectory, _, _, _) = distance_fixture_with_lever(Some(shifted));
            // Constant angular rate makes the offset speed constant. Use the
            // actual dense point velocity, independently of its covariance Jacobian.
            trajectory
                .scalar_kinematics_at_parameter(0, 0.5, ReferencePointId::new(1))
                .unwrap()
                .velocity_ecef_mps
                .into_iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt()
                * 1.5
        };
        numeric_gradient[axis] = (length(1.0e-5) - length(-1.0e-5)) / 2.0e-5;
    }
    let expected = (numeric_gradient.transpose() * &catalog.covariance * numeric_gradient)[(0, 0)];
    assert!(
        (variance - expected).abs() < 1.0e-9,
        "{variance} versus {expected}"
    );
    assert!((variance - 0.0648).abs() < 1.0e-10);
    for quantity in [
        DistanceQuantity::BodyLongitudinalSigned,
        DistanceQuantity::BodyLongitudinalAbsolute,
    ] {
        let variance = provider
            .integrated_distance_variance_m2(&trajectory, plan(quantity), span)
            .unwrap();
        // The body-forward speed of this rotating lever is -omega * lever_y.
        assert!(
            (variance - 0.0729).abs() < 1.0e-10,
            "{quantity:?}: {variance}"
        );
    }
}

#[test]
fn body_distance_gradient_matches_right_tangent_perturbations_and_marks_absolute_zero() {
    use crate::offline::solver::distance_uncertainty::distance_speed_sensitivity;
    let (trajectory, _, _, _) = distance_fixture();
    let mut estimate = trajectory
        .metric_estimate_at_parameter(0, 0.5, ReferencePointId::new(1))
        .unwrap();
    estimate.velocity = EcefVelocity::new(-2.0, 3.0, 4.0).unwrap();
    for quantity in [
        DistanceQuantity::BodyLongitudinalSigned,
        DistanceQuantity::BodyLongitudinalAbsolute,
    ] {
        let analytic =
            distance_speed_sensitivity(&estimate, quantity, ReferenceEllipsoid::WGS84).unwrap();
        for axis in 0..3 {
            let value = |delta: f64| {
                let mut tangent = Vector3::zeros();
                tangent[axis] = delta;
                let rotation = NaUnitQuaternion::from_scaled_axis(tangent);
                let signed = rotation
                    .inverse_transform_vector(&Vector3::new(-2.0, 3.0, 4.0))
                    .x;
                if quantity == DistanceQuantity::BodyLongitudinalAbsolute {
                    signed.abs()
                } else {
                    signed
                }
            };
            let numerical = (value(1.0e-6) - value(-1.0e-6)) / 2.0e-6;
            assert!((analytic[ATTITUDE + axis] - numerical).abs() < 1.0e-8);
        }
    }
    estimate.velocity = EcefVelocity::new(0.0, 3.0, 4.0).unwrap();
    assert!(
        distance_speed_sensitivity(
            &estimate,
            DistanceQuantity::BodyLongitudinalSigned,
            ReferenceEllipsoid::WGS84
        )
        .is_ok()
    );
    assert_eq!(
        distance_speed_sensitivity(
            &estimate,
            DistanceQuantity::BodyLongitudinalAbsolute,
            ReferenceEllipsoid::WGS84
        ),
        Err(UnavailableReason::IllConditioned)
    );
}

#[test]
fn reversal_distance_variance_does_not_cancel_a_shared_velocity_bias_between_quadrature_nodes() {
    let (trajectory, mut store, catalog, points) = distance_fixture_with_motion(None, Some(0.45));
    let mut provider = OfflineMetricUncertainty::new(
        store.store.as_mut(),
        &points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    for (lower, upper) in [(0.0, 1.0), (0.1, 0.93)] {
        let span = TimeSpan::new(
            SessionTime::from_ns((lower * 1.0e9) as i64),
            SessionTime::from_ns((upper * 1.0e9) as i64),
        )
        .unwrap();
        let expected = (upper + lower - 0.9_f64).powi(2);
        for quantity in [
            DistanceQuantity::BodyLongitudinalAbsolute,
            DistanceQuantity::Spatial3d,
        ] {
            let actual = provider
                .integrated_distance_variance_m2(&trajectory, plan(quantity), span)
                .unwrap();
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "{quantity:?}, {lower}..{upper}: variance {actual}, expected {expected}"
            );
        }
    }
}
