use super::*;

#[test]
fn exact_nonzero_reference_offset_needs_no_uncertain_lever_coordinate() {
    let catalog = ConsiderCatalog {
        parameters: Vec::new(),
        clocks: Vec::new(),
        covariance: DMatrix::zeros(0, 0),
    };
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1024 * 1024,
        temporary_storage_bytes: 0,
        output_bytes: 1024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let mut planned = plan_store(NAVIGATION_DIMENSION, &catalog.covariance, 2, limits).unwrap();
    let point = ReferencePoint::new(
        ReferencePointId::new(17),
        ReferencePointKind::RigidBodyPoint,
        BodyLeverArm::new(1.2, -0.3, 0.8).unwrap(),
        SharedParameterId::new(91),
        MeasurementUncertainty::Provided(Covariance3::diagonal(0.0, 0.0, 0.0).unwrap()),
    );
    let event = EventTimeSensitivity {
        segment_index: 0,
        parameter: 0.5,
        time: SessionTime::from_ns(5),
        reference_point: point.id(),
        state: StateSensitivity {
            position: [1.0, 0.0, 0.0],
            velocity: [0.0; 3],
            attitude: [0.0; 3],
        },
        gate: None,
        gate_survey_coefficient_s_per_m: 0.0,
        gate_survey_uncertainty: crate::metric::GateSurveyUncertainty::Exact,
    };
    let points = [point];
    let provider = OfflineMetricUncertainty::new(
        planned.store.as_mut(),
        &points,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    let (resolved, coordinate) = provider.reference_point(&event).unwrap();
    assert_eq!(resolved.imu_to_point(), point.imu_to_point());
    assert!(coordinate.is_none());
    let uncertain = [ReferencePoint::new(
        point.id(),
        point.kind(),
        point.imu_to_point(),
        point.parameter_id(),
        MeasurementUncertainty::Provided(Covariance3::diagonal(0.01, 0.02, 0.03).unwrap()),
    )];
    let provider = OfflineMetricUncertainty::new(
        planned.store.as_mut(),
        &uncertain,
        &catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    assert!(matches!(
        provider.reference_point(&event),
        Err(UnavailableReason::MissingCorrelation)
    ));

    // A declared shared coordinate remains authoritative even when the
    // reference carries a zero standalone marginal.
    let mut bound_catalog = ConsiderCatalog {
        parameters: vec![ParameterCoordinate {
            id: point.parameter_id(),
            kind: SharedParameterKind::LeverArmMetres,
            validity: TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(10)).unwrap(),
            start: 0,
            dimension: 3,
        }],
        clocks: Vec::new(),
        covariance: DMatrix::identity(3, 3),
    };
    let provider = OfflineMetricUncertainty::new(
        planned.store.as_mut(),
        &points,
        &bound_catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    assert!(provider.reference_point(&event).unwrap().1.is_some());
    drop(provider);
    bound_catalog.parameters[0].kind = SharedParameterKind::SurveyMetres;
    let provider = OfflineMetricUncertainty::new(
        planned.store.as_mut(),
        &points,
        &bound_catalog,
        SharedUncertaintyTreatment::SchmidtConsider,
    );
    assert!(matches!(
        provider.reference_point(&event),
        Err(UnavailableReason::MissingCorrelation)
    ));
}
