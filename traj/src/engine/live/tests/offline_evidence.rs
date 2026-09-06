//! Offline evidence.

use super::*;

#[test]
fn offline_restart_mutation_aborts_before_publication() {
    with_large_stack(offline_restart_mutation_aborts_before_publication_case);
}

fn offline_restart_mutation_aborts_before_publication_case() {
    let (mut spec, manifest, events) = replay_fixture(false);
    spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
    let baseline = events.to_vec();
    let mut changed = baseline.clone();
    let altered = std::boxed::Box::leak(std::boxed::Box::new(stationary_imu_with_specific_force(
        1,
        5_000_000,
        [0.01, -0.02, 9.8],
    )));
    let EvidenceEvent::Observation { observation, .. } = &mut changed[4] else {
        unreachable!();
    };
    *observation = altered;
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = MutatingRestartEvidenceSource {
        manifest,
        baseline,
        changed,
        restart_count: 0,
        index: 0,
    };
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::InvalidEvidence)
    ));
    assert_eq!(sink.preflights, 1);
    assert_eq!(sink.begins, 0);
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.aborts, 1);
}

#[test]
fn offline_requested_span_excludes_later_manifest_observations() {
    with_large_stack(offline_requested_span_excludes_later_manifest_observations_case);
}

fn offline_requested_span_excludes_later_manifest_observations_case() {
    let (mut spec, manifest, events) = replay_fixture(false);
    spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
    spec.span = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(25_000_000)).unwrap();

    let mut changed = events.to_vec();
    let altered = std::boxed::Box::leak(std::boxed::Box::new(stationary_imu_with_specific_force(
        6,
        REPLAY_END_NS,
        [0.5, -0.25, 9.0],
    )));
    let EvidenceEvent::Observation { observation, .. } = &mut changed[14] else {
        unreachable!();
    };
    *observation = altered;

    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut baseline_source = SliceEvidenceSource::new(manifest, &events);
    let mut baseline_sink = RecordingSink::default();
    let baseline = prepared
        .run(&mut baseline_source, &mut baseline_sink, run_control())
        .unwrap();
    let mut changed_source = SliceEvidenceSource::new(manifest, &changed);
    let mut changed_sink = RecordingSink::default();
    let changed = prepared
        .run(&mut changed_source, &mut changed_sink, run_control())
        .unwrap();

    assert_eq!(baseline.summary.state_count, changed.summary.state_count);
    assert_eq!(
        baseline.summary.objective.to_bits(),
        changed.summary.objective.to_bits()
    );
    assert_eq!(baseline.summary.diagnostics, changed.summary.diagnostics);
    assert_eq!(baseline.trajectory.span(), changed.trajectory.span());
    assert_eq!(
        baseline.trajectory.segment_count(),
        changed.trajectory.segment_count()
    );
}

#[test]
fn offline_smooths_noisy_imu_across_asynchronous_measurement_cuts() {
    with_large_stack(offline_smooths_noisy_imu_across_asynchronous_measurement_cuts_case);
}

fn offline_smooths_noisy_imu_across_asynchronous_measurement_cuts_case() {
    let (mut spec, manifest, events) = replay_fixture(false);
    spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
    let mut changed = events.to_vec();
    let asynchronous = std::boxed::Box::leak(std::boxed::Box::new(asynchronous_solution_update(
        2, 12_000_000, 13_000_000,
    )));
    // Offline smoothing does not consume captured live-call records. Use
    // one such semantic slot to exercise a second GNSS observation while
    // retaining the fixture's exact record sequence and terminal bound.
    changed[9] = EvidenceEvent::Observation {
        record_sequence: 9,
        observation: asynchronous,
    };

    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &changed);
    let mut sink = RecordingSink::default();
    let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();
    assert!(result.summary.state_count >= 5);
    assert_eq!(sink.commits, 1);
    assert!(sink.states >= 5);
}

#[test]
fn offline_post_gap_span_requires_reinitialization_before_new_evidence() {
    with_large_stack(offline_post_gap_span_requires_reinitialization_before_new_evidence_case);
}

#[test]
fn offline_accepts_a_bound_post_gap_reinitialization() {
    with_large_stack(offline_accepts_a_bound_post_gap_reinitialization_case);
}

fn offline_accepts_a_bound_post_gap_reinitialization_case() {
    let (mut spec, manifest, events) = post_gap_reinitialization_fixture();
    spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();
    assert!(result.summary.state_count >= 2);
    assert_eq!(result.summary.diagnostics.reinitializations, 1);
    assert_eq!(sink.commits, 1);
}

fn offline_post_gap_span_requires_reinitialization_before_new_evidence_case() {
    let (mut spec, manifest, mut events) = post_gap_reinitialization_fixture();
    spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
    let EvidenceEvent::Reinitialize { evidence, .. } = &mut events[1] else {
        unreachable!();
    };
    evidence.at = SessionTime::from_ns(6_000_000);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::IncompleteEvidence)
    ));
    assert_eq!(sink.commits, 0);
}

#[test]
fn offline_rank_deficient_clock_prior_preserves_publication() {
    with_large_stack(|| {
        let (mut spec, manifest, events) = replay_fixture(false);
        spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
        let prepared = TrajectoryEngine::process(spec)
            .preflight(manifest, offline_limits())
            .unwrap();
        let mut baseline_source = SliceEvidenceSource::new(manifest, &events);
        let mut baseline_sink = RecordingSink::default();
        let baseline = prepared.run(&mut baseline_source, &mut baseline_sink, run_control());
        assert!(baseline.is_ok());
        let mut changed = events.to_vec();
        let EvidenceEvent::ClockModel { model, .. } = &mut changed[1] else {
            unreachable!()
        };
        model.covariance_upper = [1.0, 0.0, 0.0];
        model.validate(6).unwrap();
        let mut source = SliceEvidenceSource::new(manifest, &changed);
        let mut sink = RecordingSink::default();
        let result = prepared.run(&mut source, &mut sink, run_control());
        assert!(
            result.is_ok(),
            "a valid PSD clock prior should preserve smoothed trajectory publication"
        );
    });
}
