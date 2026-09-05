//! Live preflight.

use super::*;

#[test]
fn live_preflight_rejects_unimplemented_nhc_profile_claim() {
    let spec = processing_spec();
    let metrics = spec
        .metrics
        .compile_live(LiveMetricLimits::default())
        .unwrap();
    let mut engine = spec.engine;
    engine.dynamics_profile.permits_non_holonomic_constraint = true;
    assert!(matches!(
        TrajectoryEngine::live(LiveSpec {
            session_id: SessionId::from_bytes([3; 16]),
            engine,
            metrics: &metrics,
            resources: LiveResourceLimits::V2_MINI_INITIAL,
            initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
            initial_clock_prior: initial_clock_prior(),
        })
        .preflight(),
        Err(PrepareError::IncompatibleProfile)
    ));
}

#[test]
fn live_preflight_rejects_body_axis_metric_without_supplied_heading() {
    let spec = processing_spec();
    let mut plan = MetricPlan::new(1);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(9),
        quantity: DistanceQuantity::BodyLongitudinalAbsolute,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 1.0e-4,
        relative_tolerance: 1.0e-6,
    }))
    .unwrap();
    let metrics = plan.compile_live(LiveMetricLimits::default()).unwrap();
    assert!(matches!(
        TrajectoryEngine::live(LiveSpec {
            session_id: SessionId::from_bytes([4; 16]),
            engine: spec.engine,
            metrics: &metrics,
            resources: LiveResourceLimits::V2_MINI_INITIAL,
            initial_heading: None,
            initial_clock_prior: initial_clock_prior(),
        })
        .preflight(),
        Err(PrepareError::CapabilityUnavailable)
    ));
}

#[test]
fn live_preflight_rejects_offset_acceleration_change_point() {
    let spec = processing_spec();
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(10),
        ReferencePointId::new(2),
        LaunchRule::AccelerationChangePoint {
            minimum_acceleration_mps2: 1.0,
            dwell: DurationNs::from_ns(10_000_000),
        },
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::Spatial3d,
        metres_per_second: 10.0,
        direction: TargetDirection::Ascending,
    })
    .unwrap();
    let mut plan = MetricPlan::new(1);
    plan.push(MetricDefinition::Drag(drag)).unwrap();
    let metrics = plan.compile_live(LiveMetricLimits::default()).unwrap();
    assert!(matches!(
        TrajectoryEngine::live(LiveSpec {
            session_id: SessionId::from_bytes([5; 16]),
            engine: spec.engine,
            metrics: &metrics,
            resources: LiveResourceLimits::V2_MINI_INITIAL,
            initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
            initial_clock_prior: initial_clock_prior(),
        })
        .preflight(),
        Err(PrepareError::CapabilityUnavailable)
    ));
}

#[test]
fn zero_variance_positive_course_keeps_infinite_snr_observable() {
    let validity = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(1)).unwrap();
    let engine = qualified_engine(validity);
    let anchor = EcefAnchor::from_origin(
        0,
        NaVector3::new(6_378_137.0, 0.0, 0.0),
        engine.processing_frame.ellipsoid(),
    )
    .unwrap();
    let mut state = crate::live::NavState::stationary(SessionTime::ZERO);
    state.velocity_n = NaVector3::new(10.0, 0.0, 0.0);
    let report = corrected_observability(
        DenseEndpoint {
            state,
            specific_force_b: NaVector3::zeros(),
            covariance: DenseCovariance::placeholder(),
        },
        &anchor,
        &engine,
        HeadingSource::Supplied,
    )
    .unwrap();
    assert!(report.course_available);
}
