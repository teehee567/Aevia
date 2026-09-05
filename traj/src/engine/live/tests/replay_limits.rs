//! Replay limits.

use super::*;

#[test]
fn tampered_live_update_identity_aborts_staged_result() {
    with_large_stack(tampered_live_update_identity_case);
}

fn tampered_live_update_identity_case() {
    let (spec, manifest, events) = replay_fixture(true);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ReplayMismatch)
    ));
    assert_eq!(sink.begins, 1);
    assert_eq!(sink.preflights, 1);
    assert_eq!(sink.begun_backends, [ProcessingLevel::CapturedReplay]);
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.aborts, 1);
    assert!(sink.write_attempts >= 2);
    assert_eq!(sink.states, 0);
    assert_eq!(sink.metrics, 0);
    assert_eq!(sink.metric_results, 0);
    assert!(sink.end.is_none());
}

#[test]
fn captured_replay_rejects_tiny_exact_segment_budget() {
    with_large_stack(captured_replay_tiny_segment_budget_case);
}

fn captured_replay_tiny_segment_budget_case() {
    let (spec, manifest, _) = replay_fixture(false);
    let mut limits = offline_limits();
    limits.peak_memory_bytes = 1;
    assert!(matches!(
        TrajectoryEngine::process(spec).preflight(manifest, limits),
        Err(PrepareError::InsufficientResources)
    ));
}

#[test]
fn captured_replay_rejects_sink_bytes_before_begin() {
    with_large_stack(captured_replay_rejects_sink_bytes_case);
}

fn captured_replay_rejects_sink_bytes_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let mut limits = offline_limits();
    limits.output_bytes = 1;
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, limits)
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ResourceLimit)
    ));
    assert_eq!(sink.preflights, 1);
    assert_eq!(sink.begins, 0);
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.states, 0);
    assert_eq!(sink.metrics, 0);
    assert!(sink.end.is_none());
}

#[test]
fn captured_replay_late_capacity_loss_aborts_all_staged_output() {
    with_large_stack(captured_replay_late_capacity_loss_case);
}

fn captured_replay_late_capacity_loss_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink {
        resource_limit_on_write: Some(2),
        ..RecordingSink::default()
    };

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ResourceLimit)
    ));
    assert_eq!(sink.preflights, 1);
    assert_eq!(sink.begins, 1);
    assert_eq!(sink.write_attempts, 2);
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.aborts, 1);
    assert_eq!(sink.states, 0);
    assert_eq!(sink.metrics, 0);
    assert!(sink.end.is_none());
    assert_eq!(sink.staged_states, 0);
    assert_eq!(sink.staged_metrics, 0);
    assert!(sink.staged_end.is_none());
}
