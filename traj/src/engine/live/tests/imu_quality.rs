//! Imu quality.

use super::*;

#[test]
fn prepared_degraded_imu_reaches_live_present_and_finalized_quality() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            let degraded = imu_with_status(stationary_imu(7, 35_000_000), ImuStatus::Degraded);
            let update = session
                .step(LiveStep {
                    observation: Some(&degraded),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert_eq!(update.input, Some((degraded.id(), InputDisposition::Fused)));
            let present = update.present.unwrap();
            assert!(present.quality.degraded_input);
            assert!(!present.quality.imu_gap);
            assert_eq!(present.quality.validity, Validity::Degraded);
            assert_eq!(update.phase, LivePhase::Degraded);
            drop(update);
            for sequence in 8..=10 {
                let imu = stationary_imu(sequence, i64::try_from(sequence).unwrap() * 5_000_000);
                session
                    .step(LiveStep {
                        observation: Some(&imu),
                        work: WorkQuota::new(128).unwrap(),
                    })
                    .unwrap();
            }
            let estimate = session
                .trajectory()
                .state_at(SessionTime::from_ns(32_500_000), ReferencePointId::new(1))
                .unwrap();
            assert!(estimate.quality.degraded_input);
            assert!(!estimate.quality.imu_gap);
            assert_eq!(estimate.quality.validity, Validity::Degraded);
        });
    });
}

#[test]
fn prepared_unavailable_imu_uses_gap_policy_and_discontinuity_reinitializes() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            let rejected = session.diagnostics.imu_epochs_rejected;
            let unavailable =
                imu_with_status(stationary_imu(7, 35_000_000), ImuStatus::Unavailable);
            let update = session
                .step(LiveStep {
                    observation: Some(&unavailable),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert_eq!(
                update.input,
                Some((unavailable.id(), InputDisposition::RetainedForOffline))
            );
            drop(update);
            assert!(session.internal.core.is_active());
            assert_eq!(session.diagnostics.imu_epochs_rejected, rejected + 1);
            let next = stationary_imu(8, 40_000_000);
            let update = session
                .step(LiveStep {
                    observation: Some(&next),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert!(update.present.unwrap().quality.imu_gap);
            drop(update);
            let broken = imu_with_status(stationary_imu(9, 45_000_000), ImuStatus::Discontinuity);
            let update = session
                .step(LiveStep {
                    observation: Some(&broken),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert_eq!(update.phase, LivePhase::Initializing);
            assert!(update.present.is_none());
            drop(update);
            assert!(!session.internal.core.is_active());
        });
    });
}

#[test]
fn offline_rejects_unmodeled_independent_imu_timestamp_jitter() {
    with_large_stack(|| {
        let (mut spec, manifest, events) = replay_fixture(false);
        spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
        let LiveObservation::Imu(imu) = stationary_imu(6, REPLAY_END_NS) else {
            unreachable!()
        };
        // Independent gyro timing error cannot disappear when the nominal
        // rate and force supports happen to coincide.
        let mut angular = imu.angular_rate();
        angular.time.independent_one_sigma = DurationNs::from_ns(1);
        let jittered = LiveObservation::Imu(
            ImuObservation::new(
                imu.id(),
                imu.measurement_frame(),
                imu.profile(),
                angular,
                imu.specific_force(),
                imu.status(),
            )
            .unwrap(),
        );
        let mut changed = events.to_vec();
        let EvidenceEvent::Observation { observation, .. } = &mut changed[14] else {
            unreachable!()
        };
        *observation = &jittered;
        let prepared = TrajectoryEngine::process(spec)
            .preflight(manifest, offline_limits())
            .unwrap();
        let mut source = SliceEvidenceSource::new(manifest, &changed);
        let mut sink = RecordingSink::default();
        assert!(matches!(
            prepared.run(&mut source, &mut sink, run_control()),
            Err(ProcessError::CapabilityUnavailable)
        ));
        assert_eq!(sink.commits, 0);
    });
}

#[test]
fn prepared_degraded_imu_reaches_offline_trajectory_quality_without_changing_math() {
    with_large_stack(|| {
        let (mut spec, manifest, events) = replay_fixture(false);
        spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
        let changed_observations: std::vec::Vec<_> = events
            .iter()
            .filter_map(|event| {
                if let EvidenceEvent::Observation { observation, .. } = event {
                    if matches!(observation, LiveObservation::Imu(_)) {
                        return Some(imu_with_status(**observation, ImuStatus::Degraded));
                    }
                }
                None
            })
            .collect();
        let mut prepared_inputs = changed_observations.iter();
        let mut changed_events = events.to_vec();
        for event in &mut changed_events {
            if let EvidenceEvent::Observation { observation, .. } = event {
                if matches!(observation, LiveObservation::Imu(_)) {
                    *observation = prepared_inputs.next().unwrap();
                }
            }
        }
        let prepared = TrajectoryEngine::process(spec)
            .preflight(manifest, offline_limits())
            .unwrap();
        let mut source = SliceEvidenceSource::new(manifest, &events);
        let mut sink = RecordingSink::default();
        let baseline = prepared.run(&mut source, &mut sink, run_control()).unwrap();
        let mut source = SliceEvidenceSource::new(manifest, &changed_events);
        let mut sink = RecordingSink::default();
        let changed = prepared.run(&mut source, &mut sink, run_control()).unwrap();
        let time = SessionTime::from_ns(20_000_000);
        let reference = ReferencePointId::new(1);
        let original = baseline.trajectory.state_at(time, reference).unwrap();
        let degraded = changed.trajectory.state_at(time, reference).unwrap();
        assert_eq!(original.position, degraded.position);
        assert_eq!(original.velocity, degraded.velocity);
        assert_eq!(original.covariance, degraded.covariance);
        assert!(!original.quality.degraded_input);
        assert!(degraded.quality.degraded_input);
        assert_eq!(degraded.quality.validity, Validity::Degraded);
    });
}

#[test]
fn predictor_gap_degrades_present_quality_and_live_phase() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            let after_gap = stationary_imu(7, 40_000_000);
            let update = session
                .step(LiveStep {
                    observation: Some(&after_gap),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            let present = update.present.unwrap();
            assert!(present.quality.imu_gap);
            assert_eq!(present.quality.validity, Validity::Degraded);
            assert_eq!(update.phase, LivePhase::Degraded);
        });
    });
}

#[test]
fn active_imu_capacity_exhaustion_is_an_operational_reinitialization() {
    with_large_stack(|| {
        with_navigating_live_session_contract(
            DurationNs::from_ns(1_000_000_000),
            10_000_000_000,
            |session| {
                let reinitializations_before = session.diagnostics.reinitializations;
                let mut capacity_candidate = None;
                // Saturate the active core directly so unrelated bounded
                // drain/projection work cannot trigger a different
                // operational reset before the public-ingest assertion.
                let end_sequence =
                    7_u64 + u64::try_from(crate::live::IMU_HISTORY_CAPACITY).unwrap() + 16;
                for sequence in 7_u64..end_sequence {
                    let offset = i64::try_from(sequence - 6).unwrap() * 5_000_000;
                    let observation = stationary_imu(sequence, REPLAY_END_NS + offset);
                    let LiveObservation::Imu(raw_imu) = observation else {
                        unreachable!()
                    };
                    let (Some(interval), _) = session.prepared_imu_interval(raw_imu).unwrap()
                    else {
                        panic!("stationary fixture must form a prepared interval")
                    };
                    let result = {
                        let mut core = LiveCore::attach(
                            &mut session.internal.core,
                            &mut session.psram.history,
                        );
                        core.ingest(LiveCoreInput::Imu(interval))
                    };
                    match result {
                        Ok(_) => session.last_accepted_imu_end = Some(interval.end),
                        Err(
                            LiveCoreError::RawImuHistoryFull | LiveCoreError::PredictorHistoryFull,
                        ) => {
                            capacity_candidate = Some((sequence, observation));
                            break;
                        }
                        Err(error) => panic!("unexpected core fill failure: {error:?}"),
                    }
                }
                let (sequence, observation) =
                    capacity_candidate.expect("fixed history must exhaust");
                let update = session
                    .step(LiveStep {
                        observation: Some(&observation),
                        work: WorkQuota::new(1).unwrap(),
                    })
                    .unwrap();
                assert_eq!(
                    update.input,
                    Some((observation.id(), InputDisposition::RetainedForOffline))
                );
                assert_eq!(update.phase, LivePhase::Initializing);
                assert_eq!(
                    session.diagnostics.reinitializations,
                    reinitializations_before + 1
                );
                assert!(
                    session.internal.sequences.iter().any(|entry| {
                        entry.source == SourceId::new(1) && entry.latest == sequence
                    })
                );
            },
        );
    });
}
