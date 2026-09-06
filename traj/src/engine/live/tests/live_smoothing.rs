//! Public live publication, quality, and control boundaries with RTS enabled.

use super::*;
use crate::engine::LiveSummary;
use crate::quality::EstimateStage;

const LAG_NS: i64 = 100_000_000;
const IMU_PERIOD_NS: i64 = 5_000_000;

fn with_smoothed_session(test: impl FnOnce(&mut LiveSession<'_, '_>)) {
    with_smoothed_session_at_reanchor_distance(100.0, test);
}

fn with_smoothed_session_at_reanchor_distance(
    reanchor_distance_m: f64,
    test: impl FnOnce(&mut LiveSession<'_, '_>),
) {
    let spec = processing_spec();
    let validity = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(1_000_000_000)).unwrap();
    let mut engine = qualified_engine(validity);
    engine.navigation_profile.smoothing_lag = DurationNs::from_ns(LAG_NS as u64);
    engine.navigation_profile.revision += 1;
    engine.navigation_profile.digest = digest(81);
    engine.navigation_profile.reanchor_distance_m = nonnegative(reanchor_distance_m);
    engine.navigation_profile.reanchor_hysteresis_m = nonnegative(reanchor_distance_m * 0.1);
    engine.digest = digest(82);
    let QualificationStatus::Qualified {
        specification,
        report,
    } = engine.qualification
    else {
        unreachable!();
    };
    let mut report = *report;
    report.configuration_digest = engine.digest;
    report.report_digest = digest(83);
    engine.qualification = QualificationStatus::Qualified {
        specification,
        report: std::boxed::Box::leak(std::boxed::Box::new(report)),
    };
    let metrics = spec
        .metrics
        .compile_live(LiveMetricLimits::default())
        .unwrap();
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: SessionId::from_bytes([81; 16]),
        engine,
        metrics: &metrics,
        resources: LiveResourceLimits::V2_MINI_RTS,
        initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
        initial_clock_prior: initial_clock_prior(),
    })
    .preflight()
    .unwrap();
    let mut internal = std::boxed::Box::new(LiveInternalWorkspace::new());
    let mut psram = std::boxed::Box::new(LivePsramWorkspace::new(engine.processing_frame));
    let mut session = plan
        .start(LiveWorkspace::bind(
            &mut internal,
            MemoryRegion::InternalSram,
            &mut psram,
            MemoryRegion::Psram,
        ))
        .unwrap();
    for observation in replay_observations() {
        session
            .step(LiveStep {
                observation: Some(&observation),
                work: ample_work(),
            })
            .unwrap();
    }
    assert_eq!(session.phase(), LivePhase::Navigating);
    assert!(session.trajectory().span().is_none());
    test(&mut session);
}

fn ample_work() -> WorkQuota {
    WorkQuota::new(WorkQuota::MAX_UNITS).unwrap()
}

fn advance_stationary(session: &mut LiveSession<'_, '_>, start_ns: i64, end_ns: i64) {
    for time_ns in (start_ns..=end_ns).step_by(IMU_PERIOD_NS as usize) {
        let imu = stationary_imu((time_ns / IMU_PERIOD_NS) as u64, time_ns);
        let update = session
            .step(LiveStep {
                observation: Some(&imu),
                work: ample_work(),
            })
            .unwrap();
        assert!(matches!(
            update.phase,
            LivePhase::Navigating | LivePhase::Degraded
        ));
    }
}

#[test]
fn present_keeps_advancing_while_trajectory_and_metrics_wait_for_lag() {
    with_large_stack(|| {
        with_smoothed_session(|session| {
            let mut previous_watermark = None;
            for time_ns in (35_000_000..=200_000_000).step_by(IMU_PERIOD_NS as usize) {
                let imu = stationary_imu((time_ns / IMU_PERIOD_NS) as u64, time_ns);
                let update = session
                    .step(LiveStep {
                        observation: Some(&imu),
                        work: ample_work(),
                    })
                    .unwrap();
                let present = update.present.unwrap();
                assert_eq!(present.time, SessionTime::from_ns(time_ns));
                assert_eq!(present.quality.stage, EstimateStage::Predicted);
                if let Some(interval) = update.corrected_interval {
                    assert!(interval.end().as_ns() <= time_ns - 10_000_000 - LAG_NS);
                    assert_eq!(update.navigation_watermark, Some(interval.end()));
                }
                if let Some(watermark) = update.navigation_watermark {
                    assert!(previous_watermark.is_none_or(|previous| watermark >= previous));
                    previous_watermark = Some(watermark);
                }
                if let Some(watermark) = update.metric_watermark {
                    assert!(watermark <= update.navigation_watermark.unwrap());
                }
            }
            assert_eq!(previous_watermark, Some(SessionTime::from_ns(90_000_000)));
            assert_eq!(
                session.trajectory().span().unwrap().end(),
                SessionTime::from_ns(90_000_000)
            );
            let core = LiveCore::attach(&mut session.internal.core, &mut session.psram.history);
            assert_eq!(
                core.status().unwrap().corrected_frontier,
                Some(SessionTime::from_ns(190_000_000))
            );
        })
    });
}

#[test]
fn smoothed_historical_quality_keeps_its_original_gnss_epoch() {
    with_large_stack(|| {
        with_smoothed_session(|session| {
            let first = position_update(2, 25_000_000, 0.0, healthy_at(25_000_000), None);
            session
                .step(LiveStep {
                    observation: Some(&first),
                    work: ample_work(),
                })
                .unwrap();
            advance_stationary(session, 35_000_000, 40_000_000);
            let LiveObservation::GnssSolution(second) =
                position_update(3, 35_000_000, 0.0, healthy_at(35_000_000), None)
            else {
                unreachable!();
            };
            let mut position = second.position().unwrap();
            position.time.basis = TimingBasis::ModeledLatency;
            let second = LiveObservation::GnssSolution(
                GnssSolutionObservation::new(
                    second.id(),
                    second.antenna_reference_point(),
                    Some(position),
                    None,
                    None,
                    second.diagnostics(),
                )
                .unwrap(),
            );
            session
                .step(LiveStep {
                    observation: Some(&second),
                    work: ample_work(),
                })
                .unwrap();
            advance_stationary(session, 45_000_000, 150_000_000);
            assert_eq!(
                session.last_gnss_evidence.unwrap().epoch,
                SessionTime::from_ns(35_000_000)
            );
            let reference = ReferencePointId::new(1);
            let before = session
                .trajectory()
                .state_at(SessionTime::from_ns(20_000_000), reference)
                .unwrap();
            let between = session
                .trajectory()
                .state_at(SessionTime::from_ns(27_500_000), reference)
                .unwrap();
            let after = session
                .trajectory()
                .state_at(SessionTime::from_ns(35_000_000), reference)
                .unwrap();
            assert_eq!(before.quality.gnss, GnssState::Absent);
            assert_eq!(between.quality.gnss, GnssState::Healthy);
            assert_eq!(between.quality.timing, TimingQuality::PpsCorrelated);
            assert_eq!(after.quality.gnss, GnssState::Healthy);
            assert_eq!(after.quality.timing, TimingQuality::Modeled);
            assert_eq!(between.quality.stage, EstimateStage::Finalized);
        })
    });
}

#[test]
fn affine_clock_transition_flushes_smoothed_tail_before_changing_segment() {
    with_large_stack(|| {
        with_smoothed_session(|session| {
            let fix = position_update(2, 25_000_000, 0.0, healthy_at(25_000_000), None);
            session
                .step(LiveStep {
                    observation: Some(&fix),
                    work: ample_work(),
                })
                .unwrap();
            let boundary = SessionTime::from_ns(40_000_000);
            let transition = clock_transition_at(
                1,
                boundary,
                Some(ClockModelId::new(1)),
                Some(ClockModelId::new(2)),
                ClockSegmentId::new(2),
                ClockTransitionUncertainty::AffineBridge(affine_bridge_at(1.0, boundary)),
            );
            session
                .step(LiveStep {
                    observation: Some(&transition),
                    work: ample_work(),
                })
                .unwrap();
            advance_stationary(session, 35_000_000, 40_000_000);
            for time_ns in [45_000_000, 50_000_000] {
                let imu = stationary_imu_for_model(
                    (time_ns / IMU_PERIOD_NS) as u64,
                    time_ns,
                    ClockModelId::new(2),
                );
                session
                    .step(LiveStep {
                        observation: Some(&imu),
                        work: WorkQuota::new(128).unwrap(),
                    })
                    .unwrap();
            }
            assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
            assert!(session.pending_clock_transition.is_some());
            for _ in 0..32 {
                let update = session
                    .step(LiveStep {
                        observation: None,
                        work: WorkQuota::new(128).unwrap(),
                    })
                    .unwrap();
                drop(update);
                if session.pending_clock_transition.is_none() {
                    break;
                }
                assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
            }
            assert_eq!(session.current_clock_segment, ClockSegmentId::new(2));
            assert!(session.pending_clock_transition.is_none());
            assert_eq!(session.trajectory().span().unwrap().end(), boundary);
            let core = LiveCore::attach(&mut session.internal.core, &mut session.psram.history);
            assert_eq!(core.status().unwrap().published_frontier, Some(boundary));
            assert_eq!(core.status().unwrap().corrected_frontier, Some(boundary));
        })
    });
}

#[test]
fn bounded_finish_flushes_every_smoothed_interval_and_finalizes_metrics() {
    with_large_stack(|| {
        with_smoothed_session(|session| {
            let fix = position_update(2, 25_000_000, 0.0, healthy_at(25_000_000), None);
            session
                .step(LiveStep {
                    observation: Some(&fix),
                    work: ample_work(),
                })
                .unwrap();
            advance_stationary(session, 35_000_000, 60_000_000);
            assert!(session.trajectory().span().is_none());
            let terminal = SessionTime::from_ns(60_000_000);
            let mut summary = LiveSummary::default();
            let first = session
                .finish(WorkQuota::new(1).unwrap(), &mut summary)
                .unwrap();
            assert!(!first.complete);
            assert!(first.update.corrected_interval.is_none());
            let mut previous = first.update.navigation_watermark;
            let mut complete = false;
            let mut final_metric_watermark = None;
            let mut saw_mutation = false;
            for _ in 0..64 {
                let finish = session
                    .finish(WorkQuota::new(128).unwrap(), &mut summary)
                    .unwrap();
                if let Some(watermark) = finish.update.navigation_watermark {
                    assert!(previous.is_none_or(|value| watermark >= value));
                    assert!(watermark <= terminal);
                    previous = Some(watermark);
                }
                if let Some(watermark) = finish.update.metric_watermark {
                    assert!(watermark <= finish.update.navigation_watermark.unwrap());
                    final_metric_watermark = Some(watermark);
                }
                saw_mutation |= !finish.update.mutations.is_empty();
                if finish.complete {
                    complete = true;
                    break;
                }
            }
            assert!(complete);
            assert!(saw_mutation);
            assert_eq!(previous, Some(terminal));
            assert_eq!(final_metric_watermark, Some(terminal));
            assert_eq!(summary.terminal_time, Some(terminal));
            assert_eq!(summary.retained_trajectory_span.unwrap().end(), terminal);
            assert!(summary.finalized_metric_results > 0);
            assert_eq!(session.phase(), LivePhase::Finished);
        })
    });
}

#[test]
fn reanchoring_a_retained_window_preserves_published_physical_states() {
    with_large_stack(|| {
        let mut reference = None;
        let mut reanchored = None;
        for (distance, result) in [(100.0, &mut reference), (0.05, &mut reanchored)] {
            with_smoothed_session_at_reanchor_distance(distance, |session| {
                let first = position_update(2, 25_000_000, 0.0, healthy_at(25_000_000), None);
                session
                    .step(LiveStep {
                        observation: Some(&first),
                        work: ample_work(),
                    })
                    .unwrap();
                advance_stationary(session, 35_000_000, 150_000_000);
                let before_second_fix = session.anchor.unwrap().generation;
                assert_eq!(before_second_fix > 0, distance < 1.0);
                let second = position_update(3, 145_000_000, 0.001, healthy_at(145_000_000), None);
                session
                    .step(LiveStep {
                        observation: Some(&second),
                        work: ample_work(),
                    })
                    .unwrap();
                advance_stationary(session, 155_000_000, 260_000_000);
                assert_eq!(session.phase(), LivePhase::Navigating);
                *result = Some(
                    session
                        .trajectory()
                        .state_at(SessionTime::from_ns(150_000_000), ReferencePointId::new(1))
                        .unwrap(),
                );
            });
        }
        let reference = reference.unwrap();
        let reanchored = reanchored.unwrap();
        for (expected, actual) in reference
            .position
            .components()
            .into_iter()
            .zip(reanchored.position.components())
        {
            assert!(
                (actual - expected).abs() < 1.0e-4,
                "position {actual} versus {expected}"
            );
        }
        for (expected, actual) in reference
            .velocity
            .components()
            .into_iter()
            .zip(reanchored.velocity.components())
        {
            assert!(
                (actual - expected).abs() < 1.0e-4,
                "velocity {actual} versus {expected}"
            );
        }
        assert_eq!(reference.quality, reanchored.quality);
    });
}
