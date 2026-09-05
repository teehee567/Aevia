//! Replay fixture.

use super::*;

pub(super) fn replay_observations() -> [LiveObservation; 7] {
    [
        initialization_fix(),
        stationary_imu(1, 5_000_000),
        stationary_imu(2, 10_000_000),
        stationary_imu(3, 15_000_000),
        stationary_imu(4, 20_000_000),
        stationary_imu(5, 25_000_000),
        stationary_imu(6, REPLAY_END_NS),
    ]
}

pub(super) fn recorded_calls(
    spec: &ProcessingSpec<'_>,
    observations: &[LiveObservation; 7],
    tamper_late_step_digest: bool,
) -> (
    CapturedReplayContract,
    [CapturedLiveStepCall; 8],
    CapturedLiveFinishCall,
) {
    let metric_limits = LiveMetricLimits::default();
    let resources = LiveResourceLimits::V2_MINI_INITIAL;
    let live_metrics = spec.metrics.compile_live(metric_limits).unwrap();
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: SessionId::from_bytes([1; 16]),
        engine: spec.engine,
        metrics: &live_metrics,
        resources,
        initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
        initial_clock_prior: initial_clock_prior(),
    })
    .preflight()
    .unwrap();
    let mut internal = std::boxed::Box::new(LiveInternalWorkspace::new());
    let mut psram = std::boxed::Box::new(LivePsramWorkspace::new(spec.engine.processing_frame));
    let workspace = LiveWorkspace::bind(
        &mut internal,
        MemoryRegion::InternalSram,
        &mut psram,
        MemoryRegion::Psram,
    );
    let mut session = plan.start(workspace).unwrap();
    let work = WorkQuota::new(128).unwrap();
    let observation_record_sequences = [2_u64, 4, 6, 8, 10, 12, 14];
    let mut steps = Vec::with_capacity(8);
    for (call_index, (observation, record_sequence)) in observations
        .iter()
        .zip(observation_record_sequences)
        .enumerate()
    {
        let update = session
            .step(LiveStep {
                observation: Some(observation),
                work,
            })
            .unwrap();
        steps.push(CapturedLiveStepCall {
            call_index: u64::try_from(call_index).unwrap(),
            observation_record_sequence: Some(record_sequence),
            work,
            expected_bit_exact_update_digest: captured_update_digest_v1(&update).unwrap(),
        });
    }

    let late_update = session
        .step(LiveStep {
            observation: None,
            work,
        })
        .unwrap();
    let mut late_digest = captured_update_digest_v1(&late_update).unwrap();
    if tamper_late_step_digest {
        let mut bytes = *late_digest.as_bytes();
        bytes[0] ^= 0xff;
        late_digest = ContentDigestV1::from_bytes(bytes);
    }
    steps.push(CapturedLiveStepCall {
        call_index: 7,
        observation_record_sequence: None,
        work,
        expected_bit_exact_update_digest: late_digest,
    });

    let mut summary = LiveSummary::default();
    let (complete, finish_digest) = {
        let finish = session.finish(work, &mut summary).unwrap();
        (
            finish.complete,
            captured_update_digest_v1(&finish.update).unwrap(),
        )
    };
    assert!(complete);
    let summary_digest = captured_summary_digest_v1(summary);
    let finish = CapturedLiveFinishCall {
        call_index: 8,
        work,
        expected_complete: true,
        expected_bit_exact_update_digest: finish_digest,
        expected_summary_digest: Some(summary_digest),
    };
    let mut transcript = CapturedTranscriptDigestV1::new();
    for step in &steps {
        transcript.observe_step(*step).unwrap();
    }
    transcript.observe_finish(finish).unwrap();
    (
        CapturedReplayContract {
            version: crate::offline::CAPTURED_REPLAY_CONTRACT_V2,
            comparison: crate::config::CapturedReplayComparison::SameBuildBitExactV1,
            transcript_digest: transcript.finalize(),
            configuration_digest: spec.engine.digest,
            navigation_profile_digest: spec.engine.navigation_profile.digest,
            metric_plan_digest: spec.result.metric_plan_digest,
            maximum_call_count: 9,
            maximum_total_work_units: 9 * u64::from(work.units()),
            metric_limits,
            resources,
            initial_heading: Some(InitialHeading::new(0.0, variance(1.0)).unwrap()),
            initial_clock_prior: initial_clock_prior(),
        },
        steps.try_into().unwrap(),
        finish,
    )
}

pub(super) fn replay_fixture(
    tamper_late_step_digest: bool,
) -> (
    ProcessingSpec<'static>,
    EvidenceManifest,
    [EvidenceEvent<'static>; 19],
) {
    let spec = processing_spec();
    let observations = std::boxed::Box::leak(std::boxed::Box::new(replay_observations()));
    let (contract, steps, finish) = recorded_calls(&spec, observations, tamper_late_step_digest);
    let source_digest = digest(50);
    let capabilities = Capabilities::NONE
        .with(Capability::CapturedReplay)
        .with(Capability::OfflineSmooth)
        .with(Capability::NormalizedImu)
        .with(Capability::GnssSolution)
        .with(Capability::Timing)
        .with(Capability::Configuration)
        .with(Capability::CompleteEnd);
    let manifest = EvidenceManifest {
        session_id: SessionId::from_bytes([1; 16]),
        source_logical_digest: source_digest,
        normalization_digest: spec.evidence_lineage.canonical_digest_v1().unwrap(),
        configuration_digest: spec.engine.digest,
        span_capabilities: SpanCapabilities {
            span: spec.span,
            capabilities,
            terminal_record_sequence: 18,
            has_valid_end: true,
        },
        capabilities,
        restartable: true,
        estimated_event_count: Some(19),
        captured_replay: Some(contract),
    };
    static CLOCK_CROSS_COVARIANCE: [f64; 12] = [0.0; 12];
    let clock = ClockModelEvidence {
        model: ClockModelId::new(1),
        segment: ClockSegmentId::new(1),
        validity: spec.span,
        reference_time: SessionTime::ZERO,
        offset_ns: 0.0,
        fractional_drift: 0.0,
        covariance_upper: [1.0, 0.0, 1.0],
        cross_covariance_with_prior: &CLOCK_CROSS_COVARIANCE,
    };
    let events = [
        EvidenceEvent::ReplayContract {
            record_sequence: 0,
            contract,
        },
        EvidenceEvent::ClockModel {
            record_sequence: 1,
            model: clock,
        },
        EvidenceEvent::Observation {
            record_sequence: 2,
            observation: &observations[0],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 3,
            call: steps[0],
        },
        EvidenceEvent::Observation {
            record_sequence: 4,
            observation: &observations[1],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 5,
            call: steps[1],
        },
        EvidenceEvent::Observation {
            record_sequence: 6,
            observation: &observations[2],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 7,
            call: steps[2],
        },
        EvidenceEvent::Observation {
            record_sequence: 8,
            observation: &observations[3],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 9,
            call: steps[3],
        },
        EvidenceEvent::Observation {
            record_sequence: 10,
            observation: &observations[4],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 11,
            call: steps[4],
        },
        EvidenceEvent::Observation {
            record_sequence: 12,
            observation: &observations[5],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 13,
            call: steps[5],
        },
        EvidenceEvent::Observation {
            record_sequence: 14,
            observation: &observations[6],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 15,
            call: steps[6],
        },
        EvidenceEvent::LiveStepCall {
            record_sequence: 16,
            call: steps[7],
        },
        EvidenceEvent::LiveFinishCall {
            record_sequence: 17,
            call: finish,
        },
        EvidenceEvent::End {
            record_sequence: 18,
            end: EvidenceEnd {
                span: spec.span,
                terminal_record_sequence: 18,
                source_logical_digest: source_digest,
            },
        },
    ];
    (spec, manifest, events)
}

pub(super) fn post_gap_reinitialization_fixture() -> (
    ProcessingSpec<'static>,
    EvidenceManifest,
    Vec<EvidenceEvent<'static>>,
) {
    let (spec, mut manifest, base) = replay_fixture(false);
    let mut contract = manifest.captured_replay.unwrap();
    let mut shifted = Vec::with_capacity(base.len() + 3);
    shifted.push(EvidenceEvent::Gap {
        record_sequence: 0,
        gap: EvidenceGap {
            span: TimeSpan::new(SessionTime::from_ns(-1), spec.span.start()).unwrap(),
            reason: EvidenceGapReason::StorageFailure,
        },
    });
    shifted.push(EvidenceEvent::Reinitialize {
        record_sequence: 1,
        evidence: ReinitializationEvidence {
            at: spec.span.start(),
            reason: ReinitializationReason::PostGapRecovery,
            generation: 2,
            input_schema: CAPTURED_REINITIALIZATION_SCHEMA_V2,
            input_digest: digest(61),
            configuration_digest: contract.configuration_digest,
            input: CapturedReinitializationInputV2 {
                navigation_profile_digest: contract.navigation_profile_digest,
                metric_plan_digest: contract.metric_plan_digest,
                resources: contract.resources,
                initial_heading: contract.initial_heading,
                initial_clock_prior: contract.initial_clock_prior,
            },
        },
    });
    shifted.push(EvidenceEvent::ControlChange {
        record_sequence: 2,
        change: ControlChangeEvidence {
            at: spec.span.start(),
            generation: 2,
            previous_digest: digest(62),
            next_digest: contract.configuration_digest,
        },
    });

    let mut transcript = CapturedTranscriptDigestV1::new();
    for event in base {
        let event = match event {
            EvidenceEvent::ReplayContract { contract, .. } => EvidenceEvent::ReplayContract {
                record_sequence: 3,
                contract,
            },
            EvidenceEvent::ClockModel {
                record_sequence,
                model,
            } => EvidenceEvent::ClockModel {
                record_sequence: record_sequence + 3,
                model,
            },
            EvidenceEvent::Observation {
                record_sequence,
                observation,
            } => EvidenceEvent::Observation {
                record_sequence: record_sequence + 3,
                observation,
            },
            EvidenceEvent::LiveStepCall {
                record_sequence,
                mut call,
            } => {
                call.observation_record_sequence = call
                    .observation_record_sequence
                    .map(|sequence| sequence + 3);
                transcript.observe_step(call).unwrap();
                EvidenceEvent::LiveStepCall {
                    record_sequence: record_sequence + 3,
                    call,
                }
            }
            EvidenceEvent::LiveFinishCall {
                record_sequence,
                call,
            } => {
                transcript.observe_finish(call).unwrap();
                EvidenceEvent::LiveFinishCall {
                    record_sequence: record_sequence + 3,
                    call,
                }
            }
            EvidenceEvent::End {
                record_sequence,
                mut end,
            } => {
                end.terminal_record_sequence += 3;
                EvidenceEvent::End {
                    record_sequence: record_sequence + 3,
                    end,
                }
            }
            _ => unreachable!(),
        };
        shifted.push(event);
    }
    contract.transcript_digest = transcript.finalize();
    if let EvidenceEvent::ReplayContract {
        contract: streamed, ..
    } = &mut shifted[3]
    {
        *streamed = contract;
    }
    manifest.captured_replay = Some(contract);
    manifest.estimated_event_count = Some(22);
    manifest.span_capabilities.terminal_record_sequence = 21;
    (spec, manifest, shifted)
}
