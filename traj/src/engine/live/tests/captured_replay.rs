//! Captured replay.

use super::*;

#[test]
fn prepared_captured_replay_runs_live_core_and_commits_complete_result() {
    with_large_stack(prepared_captured_replay_success_case);
}

#[test]
fn process_preflight_rejects_a_manifest_for_a_different_selection_set() {
    let (spec, mut manifest, _) = replay_fixture(false);
    manifest.normalization_digest = digest(51);
    assert!(matches!(
        TrajectoryEngine::process(spec).preflight(manifest, offline_limits()),
        Err(PrepareError::EvidenceUnavailable)
    ));
}

fn prepared_captured_replay_success_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    assert_eq!(prepared.selected_level(), ProcessingLevel::CapturedReplay);
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();
    let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();

    assert_eq!(sink.preflights, 1);
    assert!(sink.attested_bytes > 0);
    assert_eq!(sink.begins, 1);
    assert_eq!(sink.commits, 1);
    assert_eq!(sink.aborts, 0);
    assert_eq!(sink.backend, Some(ProcessingLevel::CapturedReplay));
    assert_eq!(sink.attempts.len(), 1);
    assert_eq!(sink.attempts[0].level, ProcessingLevel::CapturedReplay);
    assert_eq!(
        sink.attempts[0].outcome,
        ProcessingAttemptOutcome::Succeeded
    );
    assert!(result.trajectory.segment_count() > 0);
    assert!(result.summary.state_count >= 2);
    assert_eq!(sink.states, result.summary.state_count);
    assert_eq!(sink.metrics, 1);
    assert_eq!(sink.metric_results, 1);
    assert_eq!(sink.end.unwrap().state_count, result.summary.state_count);
}

#[test]
fn captured_replay_accepts_a_complete_post_gap_reinitialization_span() {
    with_large_stack(captured_replay_accepts_a_complete_post_gap_reinitialization_span_case);
}

fn captured_replay_accepts_a_complete_post_gap_reinitialization_span_case() {
    let (spec, manifest, events) = post_gap_reinitialization_fixture();
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();

    assert_eq!(sink.backend, Some(ProcessingLevel::CapturedReplay));
    assert_eq!(sink.commits, 1);
    assert_eq!(sink.aborts, 0);
    assert_eq!(sink.states, result.summary.state_count);
}

#[test]
fn captured_replay_rejects_reinitialization_before_its_gap() {
    with_large_stack(captured_replay_rejects_reinitialization_before_its_gap_case);
}

fn captured_replay_rejects_reinitialization_before_its_gap_case() {
    let (spec, manifest, mut events) = post_gap_reinitialization_fixture();
    let EvidenceEvent::Gap { gap, .. } = events[0] else {
        unreachable!();
    };
    let EvidenceEvent::Reinitialize { evidence, .. } = events[1] else {
        unreachable!();
    };
    events[0] = EvidenceEvent::Reinitialize {
        record_sequence: 0,
        evidence,
    };
    events[1] = EvidenceEvent::Gap {
        record_sequence: 1,
        gap,
    };
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
    assert_eq!(sink.aborts, 1);
}

#[test]
fn captured_replay_rejects_an_unimplemented_private_state_seed() {
    with_large_stack(captured_replay_rejects_an_unimplemented_private_state_seed_case);
}

fn captured_replay_rejects_an_unimplemented_private_state_seed_case() {
    let (spec, manifest, mut events) = post_gap_reinitialization_fixture();
    events[1] = EvidenceEvent::ReplaySeed {
        record_sequence: 1,
        seed: crate::offline::ReplaySeedEvidence {
            at: spec.span.start(),
            frontier: spec.span.start(),
            navigation_watermark: spec.span.start(),
            metric_watermark: Some(spec.span.start()),
            configuration_generation: 2,
            anchor_generation: 1,
            clock_segment: ClockSegmentId::new(1),
            run_namespace: 1,
            next_event_allocation: 1,
            profile_digest: spec.engine.navigation_profile.digest,
            configuration_digest: spec.engine.digest,
            metric_state_digest: digest(63),
            state_schema: 1,
            state_digest: digest(64),
        },
    };
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
    assert_eq!(sink.aborts, 1);
}

#[test]
fn captured_replay_rejects_an_incomplete_post_gap_reinitialization() {
    with_large_stack(captured_replay_rejects_an_incomplete_post_gap_reinitialization_case);
}

fn captured_replay_rejects_an_incomplete_post_gap_reinitialization_case() {
    let (spec, manifest, mut events) = post_gap_reinitialization_fixture();
    let EvidenceEvent::Reinitialize { evidence, .. } = &mut events[1] else {
        unreachable!();
    };
    evidence.input.metric_plan_digest = digest(65);
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
    assert_eq!(sink.aborts, 1);
}

#[test]
fn captured_replay_rejects_control_provenance_that_does_not_resolve() {
    with_large_stack(captured_replay_rejects_control_provenance_that_does_not_resolve_case);
}

fn captured_replay_rejects_control_provenance_that_does_not_resolve_case() {
    let (spec, manifest, mut events) = post_gap_reinitialization_fixture();
    let EvidenceEvent::ControlChange { change, .. } = &mut events[2] else {
        unreachable!();
    };
    change.next_digest = digest(66);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::InvalidEvidence)
    ));
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.aborts, 1);
}

#[test]
fn captured_replay_never_bridges_a_gap_into_the_selected_span() {
    with_large_stack(captured_replay_never_bridges_a_gap_into_the_selected_span_case);
}

fn captured_replay_never_bridges_a_gap_into_the_selected_span_case() {
    let (spec, manifest, mut events) = post_gap_reinitialization_fixture();
    let EvidenceEvent::Gap { gap, .. } = &mut events[0] else {
        unreachable!();
    };
    gap.span = TimeSpan::new(SessionTime::from_ns(-1), SessionTime::from_ns(1)).unwrap();
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
    assert_eq!(sink.aborts, 1);
}
