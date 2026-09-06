//! Moving starts use receiver position/velocity without claiming body heading.

use super::*;

fn moving_solution(sequence: u64, epoch_ns: i64, velocity_valid: bool) -> LiveObservation {
    let LiveObservation::GnssSolution(original) = initialization_fix() else {
        unreachable!();
    };
    let time = point_time(epoch_ns);
    let mut position = original.position().unwrap();
    position.time = time;
    position.value = EcefPosition::new(6_378_137.0, epoch_ns as f64 * 1.0e-9 * 8.0, 0.0).unwrap();
    let mut velocity = original.velocity().unwrap();
    velocity.time = time;
    velocity.value = EcefVelocity::new(0.0, 8.0, 0.0).unwrap();
    velocity.valid = velocity_valid;
    LiveObservation::GnssSolution(
        GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), sequence),
            original.antenna_reference_point(),
            Some(position),
            Some(velocity),
            None,
            GnssDiagnostics {
                health: Some(healthy_at(epoch_ns)),
                ..original.diagnostics()
            },
        )
        .unwrap(),
    )
}

fn with_unheaded_session(smoothing_lag: DurationNs, test: impl FnOnce(&mut LiveSession<'_, '_>)) {
    let spec = processing_spec();
    let live_validity =
        TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(1_000_000_000)).unwrap();
    let mut engine = qualified_engine(live_validity);
    engine.navigation_profile.smoothing_lag = smoothing_lag;
    engine.navigation_profile.revision += 1;
    engine.navigation_profile.digest = digest(91);
    engine.digest = digest(92);
    let QualificationStatus::Qualified {
        specification,
        report,
    } = engine.qualification
    else {
        unreachable!();
    };
    let mut report = *report;
    report.configuration_digest = engine.digest;
    report.report_digest = digest(93);
    engine.qualification = QualificationStatus::Qualified {
        specification,
        report: std::boxed::Box::leak(std::boxed::Box::new(report)),
    };
    let metrics = spec
        .metrics
        .compile_live(LiveMetricLimits::default())
        .unwrap();
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: SessionId::from_bytes([2; 16]),
        engine,
        metrics: &metrics,
        resources: LiveResourceLimits::V2_MINI_RTS,
        initial_heading: None,
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
    test(&mut session);
}

#[test]
fn live_starts_in_motion_on_second_valid_epoch_without_supplied_heading() {
    with_large_stack(|| {
        for horizontal_force in [0.0, 3.0] {
            with_unheaded_session(DurationNs::ZERO, |session| {
                let imu = |sequence, end| {
                    stationary_imu_with_specific_force(
                        sequence,
                        end,
                        [horizontal_force, 0.0, 9.806_65],
                    )
                };
                // A duplicate receiver epoch and a solution without valid vector
                // velocity cannot satisfy the two-solution fallback.
                for observation in [
                    moving_solution(1, 5_000_000, true),
                    imu(1, 10_000_000),
                    moving_solution(2, 5_000_000, true),
                    imu(2, 15_000_000),
                    moving_solution(3, 15_000_000, false),
                    imu(3, 20_000_000),
                    moving_solution(4, 20_000_000, true),
                ] {
                    session
                        .step(LiveStep {
                            observation: Some(&observation),
                            work: WorkQuota::new(128).unwrap(),
                        })
                        .unwrap();
                    assert_ne!(session.phase(), LivePhase::Navigating);
                }
                let observation = imu(4, 25_000_000);
                let update = session
                    .step(LiveStep {
                        observation: Some(&observation),
                        work: WorkQuota::new(128).unwrap(),
                    })
                    .unwrap();
                assert_eq!(
                    update.input,
                    Some((observation.id(), InputDisposition::Fused))
                );
                assert!(update.present.is_some());
                drop(update);
                assert_eq!(session.phase(), LivePhase::Navigating);
                assert_eq!(session.heading_source, HeadingSource::None);
                let core = LiveCore::attach(&mut session.internal.core, &mut session.psram.history);
                let state = core.present_state().unwrap();
                assert!((state.velocity_n.x - 8.0).abs() < 0.1);
                assert!(state.velocity_n.iter().all(|value| value.is_finite()));
            });
        }
    });
}

#[test]
fn moving_start_with_rts_fuses_later_fixes_and_flushes_finite_trajectory() {
    use crate::engine::LiveSummary;
    with_large_stack(|| {
        with_unheaded_session(DurationNs::from_ns(100_000_000), |session| {
            let work = WorkQuota::new(WorkQuota::MAX_UNITS).unwrap();
            let first = moving_solution(1, 5_000_000, true);
            session
                .step(LiveStep {
                    observation: Some(&first),
                    work,
                })
                .unwrap();
            for sample in 2..=50 {
                let end_ns = sample * 5_000_000;
                let start_ns = end_ns - 5_000_000;
                let fix_sequence = match start_ns {
                    10_000_000 => Some(2),
                    50_000_000 => Some(3),
                    150_000_000 => Some(4),
                    _ => None,
                };
                if let Some(sequence) = fix_sequence {
                    let observation = moving_solution(sequence, start_ns, true);
                    session
                        .step(LiveStep {
                            observation: Some(&observation),
                            work,
                        })
                        .unwrap();
                }
                let observation = stationary_imu(sample as u64, end_ns);
                let update = session
                    .step(LiveStep {
                        observation: Some(&observation),
                        work,
                    })
                    .unwrap();
                if end_ns >= 15_000_000 {
                    assert!(update.present.is_some());
                    assert_ne!(update.phase, LivePhase::Initializing);
                }
            }
            assert_eq!(session.heading_source, HeadingSource::None);
            assert_eq!(session.diagnostics.reinitializations, 0);
            assert!(session.diagnostics.gnss_updates_fused >= 2);
            let span = session.trajectory().span().unwrap();
            assert_eq!(span.start(), SessionTime::from_ns(10_000_000));
            assert!(span.end() > span.start());
            assert!(span.end() <= SessionTime::from_ns(150_000_000));

            let mut summary = LiveSummary::default();
            let mut complete = false;
            for _ in 0..128 {
                let finish = session
                    .finish(WorkQuota::new(128).unwrap(), &mut summary)
                    .unwrap();
                assert!(finish.update.present.is_some());
                if finish.complete {
                    complete = true;
                    break;
                }
            }
            assert!(complete, "moving RTS tail must flush in bounded calls");
            let terminal = SessionTime::from_ns(250_000_000);
            assert_eq!(summary.terminal_time, Some(terminal));
            assert_eq!(summary.retained_trajectory_span.unwrap().end(), terminal);
            assert_eq!(summary.diagnostics.reinitializations, 0);
            for epoch in [25_000_000, 100_000_000, 200_000_000, 250_000_000] {
                let point = session
                    .trajectory()
                    .state_at(SessionTime::from_ns(epoch), ReferencePointId::new(1))
                    .unwrap();
                assert!(
                    point
                        .position
                        .components()
                        .iter()
                        .all(|value| value.is_finite())
                );
                assert!(
                    point
                        .velocity
                        .components()
                        .iter()
                        .all(|value| value.is_finite())
                );
            }
            assert_eq!(session.phase(), LivePhase::Finished);
        });
    });
}
