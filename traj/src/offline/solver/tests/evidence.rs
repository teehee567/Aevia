use super::*;

#[test]
fn requested_span_selects_complete_imu_support_and_canonical_boundary_order() {
    let observation = imu_observation(interval_time(10, 5), interval_time(10, 5), false);
    let (tasks, count) = tasks_for_observation(LiveObservation::Imu(observation)).unwrap();
    assert_eq!(count, 1);
    let task = tasks[0].unwrap();
    assert!(
        task_is_selected(
            task,
            TimeSpan::new(SessionTime::from_ns(5), SessionTime::from_ns(10)).unwrap()
        )
        .unwrap()
    );
    assert!(
        !task_is_selected(
            task,
            TimeSpan::new(SessionTime::from_ns(6), SessionTime::from_ns(10)).unwrap()
        )
        .unwrap()
    );
    assert!(TASK_CLASS_GAP < TASK_CLASS_REINITIALIZE);
    assert!(TASK_CLASS_REINITIALIZE < TASK_CLASS_CLOCK_TRANSITION);
    assert!(TASK_CLASS_CLOCK_TRANSITION < TASK_CLASS_IMU);
    assert!(TASK_CLASS_IMU < TASK_CLASS_GNSS_POSITION);
    assert!(TASK_CLASS_GNSS_POSITION < TASK_CLASS_GNSS_VELOCITY);
}

#[test]
fn stream_validator_requires_exact_terminal_sequence_span_and_digest() {
    let span = TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(10)).unwrap();
    let digest = ContentDigestV1::from_bytes([7; 32]);
    let capabilities = Capabilities::one(Capability::CompleteEnd);
    let manifest = EvidenceManifest {
        session_id: SessionId::from_bytes([1; 16]),
        source_logical_digest: digest,
        normalization_digest: ContentDigestV1::from_bytes([2; 32]),
        configuration_digest: ContentDigestV1::from_bytes([3; 32]),
        span_capabilities: SpanCapabilities {
            span,
            capabilities,
            terminal_record_sequence: 4,
            has_valid_end: true,
        },
        capabilities,
        restartable: true,
        estimated_event_count: Some(1),
        captured_replay: None,
    };
    let mut validator = StreamValidator::new(manifest);
    let wrong = EvidenceEvent::End {
        record_sequence: 0,
        end: EvidenceEnd {
            span,
            terminal_record_sequence: 0,
            source_logical_digest: digest,
        },
    };
    assert_eq!(
        validator.observe(wrong),
        Err(ProcessError::IncompleteEvidence)
    );
}

#[test]
fn stream_validator_rejects_declared_event_bound_before_dispatch() {
    let span = TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(10)).unwrap();
    let digest = ContentDigestV1::from_bytes([7; 32]);
    let capabilities = Capabilities::one(Capability::CompleteEnd);
    let manifest = EvidenceManifest {
        session_id: SessionId::from_bytes([1; 16]),
        source_logical_digest: digest,
        normalization_digest: ContentDigestV1::from_bytes([2; 32]),
        configuration_digest: ContentDigestV1::from_bytes([3; 32]),
        span_capabilities: SpanCapabilities {
            span,
            capabilities,
            terminal_record_sequence: 1,
            has_valid_end: true,
        },
        capabilities,
        restartable: true,
        estimated_event_count: Some(1),
        captured_replay: None,
    };
    let mut validator = StreamValidator::new(manifest);
    let change = ControlChangeEvidence {
        at: SessionTime::ZERO,
        generation: 1,
        previous_digest: ContentDigestV1::from_bytes([4; 32]),
        next_digest: ContentDigestV1::from_bytes([5; 32]),
    };
    assert_eq!(
        validator.observe(EvidenceEvent::ControlChange {
            record_sequence: 0,
            change,
        }),
        Ok(())
    );
    assert_eq!(
        validator.observe(EvidenceEvent::ControlChange {
            record_sequence: 1,
            change,
        }),
        Err(ProcessError::InvalidEvidence)
    );
}

#[test]
fn restart_integrity_dominates_a_deferred_numerical_failure() {
    let expected = ContentDigestV1::from_bytes([1; 32]);
    let changed = ContentDigestV1::from_bytes([2; 32]);
    assert_eq!(
        verify_restarted_stream(
            5,
            expected,
            5,
            changed,
            Some(ProcessError::NumericalNonConvergence),
        ),
        Err(ProcessError::InvalidEvidence)
    );
    assert_eq!(
        verify_restarted_stream(
            5,
            expected,
            5,
            expected,
            Some(ProcessError::NumericalNonConvergence),
        ),
        Err(ProcessError::NumericalNonConvergence)
    );
}
