//! Invalid GNSS fields must never contribute to navigation.

use super::*;

fn prepared_solution(
    original: GnssSolutionObservation,
    position_valid: bool,
    velocity_valid: bool,
    omit_invalid: bool,
) -> LiveObservation {
    let position = original.position().map(|mut field| {
        field.valid = position_valid;
        if !position_valid {
            field.value = EcefPosition::new(1.0e9, -1.0e9, 1.0e9).unwrap();
        }
        field
    });
    let velocity = original.velocity().map(|mut field| {
        field.valid = velocity_valid;
        if !velocity_valid {
            field.value = EcefVelocity::new(1.0e6, -1.0e6, 1.0e6).unwrap();
        }
        field
    });
    LiveObservation::GnssSolution(
        GnssSolutionObservation::new(
            original.id(),
            original.antenna_reference_point(),
            position.filter(|field| !omit_invalid || field.valid),
            velocity.filter(|field| !omit_invalid || field.valid),
            None,
            original.diagnostics(),
        )
        .unwrap(),
    )
}

#[test]
fn live_invalid_fields_match_omitted_fields_in_state_covariance_and_fusion() {
    with_large_stack(|| {
        for (position_valid, velocity_valid) in [(false, true), (true, false)] {
            let mut baseline = None;
            for omit_invalid in [true, false] {
                with_navigating_live_session(|session| {
                    let LiveObservation::GnssSolution(original) =
                        asynchronous_solution_update(2, 25_000_000, 25_000_000)
                    else {
                        unreachable!();
                    };
                    let observation =
                        prepared_solution(original, position_valid, velocity_valid, omit_invalid);
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
                    let imu = stationary_imu(7, 35_000_000);
                    let update = session
                        .step(LiveStep {
                            observation: Some(&imu),
                            work: WorkQuota::new(128).unwrap(),
                        })
                        .unwrap();
                    let fusion = update.fusion.unwrap();
                    assert!(matches!(
                        fusion.disposition,
                        InputDisposition::Fused | InputDisposition::Downweighted
                    ));
                    let present = update.present.unwrap();
                    let corrected = session
                        .trajectory()
                        .state_at(SessionTime::from_ns(25_000_000), ReferencePointId::new(1))
                        .unwrap();
                    let result = (present, corrected, fusion, session.diagnostics);
                    if let Some(expected) = baseline {
                        assert_eq!(result, expected);
                    } else {
                        baseline = Some(result);
                    }
                });
            }
        }
    });
}

#[test]
fn live_all_invalid_fields_are_retained_without_fusion_or_quality_changes() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            let LiveObservation::GnssSolution(original) =
                asynchronous_solution_update(2, 25_000_000, 25_000_000)
            else {
                unreachable!();
            };
            let observation = prepared_solution(original, false, false, false);
            let before = session.present_projection().unwrap();
            let fused = session.diagnostics.gnss_updates_fused;
            let rejected = session.diagnostics.gnss_updates_rejected;
            let evidence = session.last_gnss_evidence;
            let update = session
                .step(LiveStep {
                    observation: Some(&observation),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert_eq!(
                update.input,
                Some((observation.id(), InputDisposition::RetainedForOffline))
            );
            assert!(update.fusion.is_none());
            assert_eq!(update.present, before);
            assert_eq!(update.diagnostics.gnss_updates_fused, fused);
            assert_eq!(update.diagnostics.gnss_updates_rejected, rejected + 1);
            assert_eq!(session.last_gnss_evidence, evidence);
        });
    });
}

#[test]
fn offline_invalid_fields_match_omitted_fields_in_smoothed_state_and_covariance() {
    with_large_stack(|| {
        let (mut spec, manifest, events) = replay_fixture(false);
        spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
        let prepared = TrajectoryEngine::process(spec)
            .preflight(manifest, offline_limits())
            .unwrap();
        for (position_valid, velocity_valid) in [(false, true), (true, false), (false, false)] {
            let mut baseline = None;
            for omit_invalid in [true, false] {
                let LiveObservation::GnssSolution(original) =
                    asynchronous_solution_update(2, 15_000_000, 15_000_000)
                else {
                    unreachable!();
                };
                let observation =
                    prepared_solution(original, position_valid, velocity_valid, false);
                let mut changed = events.to_vec();
                let omitted;
                if omit_invalid && (position_valid || velocity_valid) {
                    omitted = prepared_solution(original, position_valid, velocity_valid, true);
                    changed[9] = EvidenceEvent::Observation {
                        record_sequence: 9,
                        observation: &omitted,
                    };
                } else if !omit_invalid {
                    changed[9] = EvidenceEvent::Observation {
                        record_sequence: 9,
                        observation: &observation,
                    };
                }
                let mut source = SliceEvidenceSource::new(manifest, &changed);
                let mut sink = RecordingSink::default();
                let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();
                assert_eq!(sink.commits, 1);
                let states = [5_000_000, 15_000_000, 25_000_000].map(|time| {
                    result
                        .trajectory
                        .state_at(SessionTime::from_ns(time), ReferencePointId::new(1))
                        .unwrap()
                });
                if let Some(expected) = baseline {
                    assert_eq!(states, expected);
                } else {
                    baseline = Some(states);
                }
            }
        }
    });
}
