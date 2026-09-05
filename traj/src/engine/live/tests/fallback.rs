//! Fallback.

use super::*;

#[test]
fn best_qualified_does_not_retry_a_sink_preflight_failure() {
    with_large_stack(best_qualified_does_not_retry_a_sink_preflight_failure_case);
}

fn best_qualified_does_not_retry_a_sink_preflight_failure_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let spec = with_best_qualified(
        spec,
        &[
            ProcessingLevel::CapturedReplay,
            ProcessingLevel::OfflineSmooth,
        ],
    );
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    assert_eq!(prepared.selected_level(), ProcessingLevel::CapturedReplay);
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink {
        resource_limit_on_preflight: Some(1),
        ..RecordingSink::default()
    };

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ResourceLimit)
    ));
    assert_eq!(sink.preflight_backends, [ProcessingLevel::CapturedReplay]);
    assert!(sink.begun_backends.is_empty());
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.states, 0);
}

#[test]
fn best_qualified_does_not_retry_a_source_failure() {
    with_large_stack(best_qualified_does_not_retry_a_source_failure_case);
}

fn best_qualified_does_not_retry_a_source_failure_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let spec = with_best_qualified(
        spec,
        &[
            ProcessingLevel::CapturedReplay,
            ProcessingLevel::OfflineSmooth,
        ],
    );
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = FailingEvidenceSource {
        inner: SliceEvidenceSource::new(manifest, &events),
        fail_next: true,
    };
    let mut sink = RecordingSink::default();

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ResourceLimit)
    ));
    assert_eq!(sink.preflight_backends, [ProcessingLevel::CapturedReplay]);
    assert_eq!(sink.begun_backends, [ProcessingLevel::CapturedReplay]);
    assert_eq!(sink.aborts, 1);
    assert_eq!(sink.commits, 0);
}

#[test]
fn best_qualified_aborts_partial_candidate_then_restarts_next_backend() {
    with_large_stack(best_qualified_aborts_partial_candidate_then_restarts_next_backend_case);
}

fn best_qualified_aborts_partial_candidate_then_restarts_next_backend_case() {
    let (spec, manifest, events) = replay_fixture(true);
    let spec = with_best_qualified(
        spec,
        &[
            ProcessingLevel::AdvancedGraph,
            ProcessingLevel::CapturedReplay,
            ProcessingLevel::OfflineSmooth,
        ],
    );
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    assert_eq!(prepared.selected_level(), ProcessingLevel::CapturedReplay);
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink::default();

    let result = prepared.run(&mut source, &mut sink, run_control()).unwrap();

    assert_eq!(
        sink.begun_backends,
        [
            ProcessingLevel::CapturedReplay,
            ProcessingLevel::OfflineSmooth
        ]
    );
    assert_eq!(sink.aborts, 1);
    assert_eq!(sink.commits, 1);
    assert_eq!(sink.backend, Some(ProcessingLevel::OfflineSmooth));
    assert_eq!(sink.attempts.len(), 3);
    assert_eq!(sink.attempts[0].level, ProcessingLevel::AdvancedGraph);
    assert_eq!(sink.attempts[0].ordinal, 0);
    assert_eq!(
        sink.attempts[0].outcome,
        ProcessingAttemptOutcome::NotCompiled
    );
    assert_eq!(sink.attempts[1].level, ProcessingLevel::CapturedReplay);
    assert_eq!(sink.attempts[1].ordinal, 1);
    assert_eq!(sink.attempts[1].outcome, ProcessingAttemptOutcome::Failed);
    assert_eq!(sink.attempts[2].level, ProcessingLevel::OfflineSmooth);
    assert_eq!(sink.attempts[2].ordinal, 2);
    assert_eq!(
        sink.attempts[2].outcome,
        ProcessingAttemptOutcome::Succeeded
    );
    assert_eq!(sink.states, result.summary.state_count);
    assert_eq!(sink.metrics, 1);
    assert!(sink.end.is_some());
}

#[test]
fn best_qualified_does_not_retry_a_sink_write_failure() {
    with_large_stack(best_qualified_does_not_retry_a_sink_write_failure_case);
}

fn best_qualified_does_not_retry_a_sink_write_failure_case() {
    let (spec, manifest, events) = replay_fixture(false);
    let spec = with_best_qualified(
        spec,
        &[
            ProcessingLevel::CapturedReplay,
            ProcessingLevel::OfflineSmooth,
        ],
    );
    let prepared = TrajectoryEngine::process(spec)
        .preflight(manifest, offline_limits())
        .unwrap();
    let mut source = SliceEvidenceSource::new(manifest, &events);
    let mut sink = RecordingSink {
        resource_limit_on_every_write: true,
        ..RecordingSink::default()
    };

    assert!(matches!(
        prepared.run(&mut source, &mut sink, run_control()),
        Err(ProcessError::ResourceLimit)
    ));
    assert_eq!(sink.begun_backends, [ProcessingLevel::CapturedReplay]);
    assert_eq!(sink.commits, 0);
    assert_eq!(sink.aborts, 1);
    assert_eq!(sink.states, 0);
    assert_eq!(sink.metrics, 0);
    assert_eq!(sink.metric_results, 0);
    assert!(sink.end.is_none());
    assert!(!sink.active);
    assert!(!sink.reserved);
}
