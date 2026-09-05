//! Session fixture.

use super::*;

pub(super) fn offline_limits() -> OfflineResourceLimits {
    OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 64 * 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    }
}

pub(super) fn continue_running(_: u64) -> bool {
    true
}

pub(super) fn report_progress(_: u64, _: u64) {}

pub(super) fn run_control() -> RunControl<'static> {
    RunControl {
        continue_running: &continue_running,
        progress: &report_progress,
    }
}

pub(super) fn with_large_stack(test: fn()) {
    std::thread::Builder::new()
        .name("captured-replay-test".into())
        .stack_size(32 * 1_024 * 1_024)
        .spawn(test)
        .unwrap()
        .join()
        .unwrap();
}

pub(super) fn with_navigating_live_session(test: impl FnOnce(&mut LiveSession<'_, '_>)) {
    with_navigating_live_session_and_gnss_age(DurationNs::from_ns(1_000_000_000), test);
}

pub(super) fn with_navigating_live_session_and_gnss_age(
    maximum_gnss_age: DurationNs,
    test: impl FnOnce(&mut LiveSession<'_, '_>),
) {
    with_navigating_live_session_contract(maximum_gnss_age, REPLAY_END_NS + 100_000_000, test);
}

pub(super) fn with_navigating_live_session_contract(
    maximum_gnss_age: DurationNs,
    validity_end_ns: i64,
    test: impl FnOnce(&mut LiveSession<'_, '_>),
) {
    let spec = processing_spec();
    let live_validity =
        TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(validity_end_ns)).unwrap();
    let mut engine = qualified_engine(live_validity);
    engine.dynamics_profile.gnss.maximum_correction_age = maximum_gnss_age;
    let metric_limits = LiveMetricLimits::default();
    let live_metrics = spec.metrics.compile_live(metric_limits).unwrap();
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: SessionId::from_bytes([2; 16]),
        engine,
        metrics: &live_metrics,
        resources: LiveResourceLimits::V2_MINI_INITIAL,
        initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
        initial_clock_prior: initial_clock_prior(),
    })
    .preflight()
    .unwrap();
    let mut internal = std::boxed::Box::new(LiveInternalWorkspace::new());
    let mut psram = std::boxed::Box::new(LivePsramWorkspace::new(engine.processing_frame));
    let workspace = LiveWorkspace::bind(
        &mut internal,
        MemoryRegion::InternalSram,
        &mut psram,
        MemoryRegion::Psram,
    );
    let mut session = plan.start(workspace).unwrap();
    let work = WorkQuota::new(128).unwrap();
    for observation in replay_observations() {
        session
            .step(LiveStep {
                observation: Some(&observation),
                work,
            })
            .unwrap();
    }
    assert_eq!(session.phase(), LivePhase::Navigating);
    test(&mut session);
}

pub(super) fn clock_transition(
    sequence: u64,
    next_model: ClockModelId,
    next_segment: ClockSegmentId,
    uncertainty: ClockTransitionUncertainty,
) -> LiveObservation {
    clock_transition_at(
        sequence,
        SessionTime::from_ns(REPLAY_END_NS),
        Some(ClockModelId::new(1)),
        Some(next_model),
        next_segment,
        uncertainty,
    )
}

pub(super) fn clock_transition_at(
    sequence: u64,
    at: SessionTime,
    previous_model: Option<ClockModelId>,
    next_model: Option<ClockModelId>,
    next_segment: ClockSegmentId,
    uncertainty: ClockTransitionUncertainty,
) -> LiveObservation {
    LiveObservation::ClockTransition(ClockTransitionObservation {
        id: ObservationId::new(SourceId::new(3), sequence),
        at,
        previous_model,
        next_model,
        next_segment,
        reason: ClockDiscontinuityReason::Reconfigured,
        uncertainty,
    })
}

pub(super) fn affine_bridge(mapping_scale: f32) -> ClockAffineBridge {
    affine_bridge_at(mapping_scale, SessionTime::from_ns(REPLAY_END_NS))
}

pub(super) fn affine_bridge_at(
    mapping_scale: f32,
    next_reference_time: SessionTime,
) -> ClockAffineBridge {
    let mut mapping = [[0.0; MAX_CONSIDER]; 2];
    mapping[0][0] = mapping_scale;
    mapping[1][1] = 1.0;
    ClockAffineBridge::new(8, next_reference_time, mapping, [0.25, 0.0, 0.5]).unwrap()
}

pub(super) fn fuse_nominal_position_at_25ms(session: &mut LiveSession<'_, '_>) {
    let observation = position_update(
        2,
        25_000_000,
        0.0,
        RtkState::Fixed,
        healthy_at(25_000_000),
        None,
    );
    {
        let update = session
            .step(LiveStep {
                observation: Some(&observation),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();
        assert_eq!(
            update.input,
            Some((observation.id(), InputDisposition::QueuedForFusion))
        );
        assert!(update.fusion.is_none());
    }
    assert!(session.last_gnss_evidence.is_none());

    let imu = stationary_imu(7, 35_000_000);
    let update = session
        .step(LiveStep {
            observation: Some(&imu),
            work: WorkQuota::new(128).unwrap(),
        })
        .unwrap();
    assert!(matches!(
        update.fusion,
        Some(FusionOutcome {
            disposition: InputDisposition::Fused | InputDisposition::Downweighted,
            ..
        })
    ));
    drop(update);
    assert_eq!(
        session
            .last_gnss_evidence
            .map(|value| (value.epoch, value.state)),
        Some((SessionTime::from_ns(25_000_000), GnssState::Fixed))
    );
}
