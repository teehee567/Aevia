//! Exact captured live-call replay and transactional publication of the resulting trajectory.

use super::selection::captured_replay_preflight;
use crate::config::{LiveSpec, OfflineResourceLimits, ProcessingSpec, RunControl};
use crate::engine::digest::{captured_summary_digest_v1, captured_update_digest_v1};
use crate::engine::{LiveSummary, TrajectoryEngine};
use crate::error::{PrepareError, ProcessError};
use crate::ids::ContentDigestV1;
use crate::metric::MetricError;
use crate::observation::{LiveObservation, LiveStep};
use crate::offline::{
    CapturedLiveFinishCall, CapturedLiveStepCall, CapturedTranscriptDigestV1, EvidenceEvent,
    ResultDescriptor, ResultEnd, ResultRecord, ResultRecordBounds, SinkTransaction,
};
use crate::provenance::ResultProvenance;
use crate::trajectory::Trajectory;
use crate::workspace::{LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion};

#[cfg(feature = "offline")]
pub(super) fn run_captured_replay<
    S: crate::offline::EvidenceSource,
    K: crate::offline::ResultSink,
>(
    spec: &ProcessingSpec<'_>,
    manifest: crate::offline::EvidenceManifest,
    limits: OfflineResourceLimits,
    provenance: ResultProvenance<'_>,
    source: &mut S,
    sink: &mut K,
    control: RunControl<'_>,
) -> Result<crate::offline::OfflineRun, ProcessError> {
    let contract = manifest
        .captured_replay
        .ok_or(ProcessError::IncompleteEvidence)?;
    let live_metrics =
        captured_replay_preflight(spec, manifest, limits).map_err(|error| match error {
            PrepareError::InsufficientResources => ProcessError::ResourceLimit,
            PrepareError::ReplayContractUnavailable | PrepareError::EvidenceUnavailable => {
                ProcessError::IncompleteEvidence
            }
            _ => ProcessError::InvalidEvidence,
        })?;
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: manifest.session_id,
        engine: spec.engine.clone(),
        metrics: &live_metrics,
        resources: contract.resources,
        initial_heading: contract.initial_heading,
        initial_clock_prior: contract.initial_clock_prior,
    })
    .preflight()
    .map_err(|error| match error {
        PrepareError::InsufficientResources => ProcessError::ResourceLimit,
        _ => ProcessError::InvalidEvidence,
    })?;
    provenance
        .validate()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    let descriptor = ResultDescriptor {
        provenance,
        trajectory_revision: spec.result.trajectory_revision,
    };
    let state_record_bound = contract
        .maximum_total_work_units
        .checked_add(1)
        .ok_or(ProcessError::ResourceLimit)?;
    let record_bounds = ResultRecordBounds::new(
        state_record_bound,
        u64::from(contract.metric_limits.max_results),
    )?;
    // Keep publication staged for the entire replay. Any transcript, update,
    // summary, or metric failure below drops this guard and aborts the
    // candidate before a result revision can become visible.
    let mut transaction =
        SinkTransaction::begin(sink, descriptor, record_bounds, limits.output_bytes)?;

    // Host allocation is explicit, bounded by the preflighted compiled object
    // sizes, and uses no unsafe placement or native solver workspace.
    let maximum_segments = usize::try_from(contract.maximum_total_work_units)
        .map_err(|_| ProcessError::ResourceLimit)?;
    let rolling_segments =
        maximum_segments.min(crate::trajectory::MAX_EMBEDDED_TRAJECTORY_SEGMENTS);
    let mut internal = std::boxed::Box::new(LiveInternalWorkspace::new());
    let mut psram = std::boxed::Box::new(LivePsramWorkspace::new(spec.engine.processing_frame));
    psram
        .trajectory
        .try_reserve_segments_exact(rolling_segments)
        .map_err(|_| ProcessError::ResourceLimit)?;
    let mut trajectory = Trajectory::new(
        spec.engine.processing_frame,
        spec.result.trajectory_revision,
    );
    trajectory
        .try_reserve_segments_exact(maximum_segments)
        .map_err(|_| ProcessError::ResourceLimit)?;
    let workspace = LiveWorkspace::bind(
        &mut internal,
        MemoryRegion::InternalSram,
        &mut psram,
        MemoryRegion::Psram,
    );
    let mut session = plan.start(workspace).map_err(|error| match error {
        PrepareError::InsufficientResources | PrepareError::InvalidWorkspaceAlignment => {
            ProcessError::ResourceLimit
        }
        _ => ProcessError::InvalidEvidence,
    })?;
    for point in spec.engine.installation.reference_points {
        trajectory
            .add_reference_point(*point)
            .map_err(|_| ProcessError::InvalidEvidence)?;
    }

    let mut pending_observation: Option<(u64, LiveObservation)> = None;
    let mut contract_seen = false;
    let mut next_call = 0_u64;
    let mut spent_work = 0_u64;
    let mut finishing = false;
    let mut complete = false;
    let mut summary = LiveSummary::default();
    let mut transcript = CapturedTranscriptDigestV1::new();
    let mut state_count = 0_u64;
    let mut leading_gap: Option<crate::offline::EvidenceGap> = None;
    let mut restart: Option<crate::offline::ReinitializationEvidence> = None;
    let mut control_head: Option<ContentDigestV1> = None;
    let mut control_generation: Option<u32> = None;

    crate::offline::drive_captured_replay(spec, manifest, source, control, |event| match event {
        EvidenceEvent::Opaque { .. } => Ok(()),
        EvidenceEvent::ReplayContract {
            contract: streamed, ..
        } => {
            if contract_seen
                || next_call != 0
                || pending_observation.is_some()
                || streamed != contract
            {
                return Err(ProcessError::InvalidEvidence);
            }
            if let Some(gap) = leading_gap {
                if gap.span.end() > spec.span.start() {
                    return Err(ProcessError::IncompleteEvidence);
                }
                restart
                    .ok_or(ProcessError::IncompleteEvidence)?
                    .validate_for_contract(streamed)?;
            } else if let Some(restart) = restart {
                restart.validate_for_contract(streamed)?;
            }
            if control_head.is_some_and(|digest| digest != streamed.configuration_digest) {
                return Err(ProcessError::InvalidEvidence);
            }
            contract_seen = true;
            Ok(())
        }
        EvidenceEvent::Observation {
            record_sequence,
            observation,
        } => {
            if !contract_seen || finishing || pending_observation.is_some() {
                return Err(ProcessError::IncompleteEvidence);
            }
            pending_observation = Some((record_sequence, *observation));
            Ok(())
        }
        EvidenceEvent::LiveStepCall { call, .. } => {
            validate_replay_step_call(call, next_call, contract.maximum_call_count)?;
            if !contract_seen || finishing {
                return Err(ProcessError::InvalidEvidence);
            }
            let observation = match call.observation_record_sequence {
                Some(sequence) => {
                    let (record_sequence, observation) = pending_observation
                        .take()
                        .ok_or(ProcessError::IncompleteEvidence)?;
                    if sequence != record_sequence {
                        return Err(ProcessError::InvalidEvidence);
                    }
                    Some(observation)
                }
                None => None,
            };
            transcript
                .observe_step(call)
                .map_err(|_| ProcessError::InvalidEvidence)?;
            spent_work = spent_work
                .checked_add(u64::from(call.work.units()))
                .ok_or(ProcessError::ResourceLimit)?;
            if spent_work > contract.maximum_total_work_units {
                return Err(ProcessError::ResourceLimit);
            }
            let update = session
                .step(LiveStep {
                    observation: observation.as_ref(),
                    work: call.work,
                })
                .map_err(|_| ProcessError::ReplayMismatch)?;
            let actual = captured_update_digest_v1(&update)?;
            let committed = update.corrected_interval;
            verify_replay_identity(call.expected_bit_exact_update_digest, actual)?;
            drop(update);
            trajectory
                .append_unseen_segments_from(session.trajectory(), committed)
                .map_err(|_| ProcessError::ResourceLimit)?;
            stage_new_captured_states(&trajectory, &mut state_count, &mut transaction)?;
            next_call = next_call
                .checked_add(1)
                .ok_or(ProcessError::ResourceLimit)?;
            Ok(())
        }
        EvidenceEvent::LiveFinishCall { call, .. } => {
            validate_replay_finish_call(call, next_call, contract.maximum_call_count)?;
            if !contract_seen || complete || pending_observation.is_some() {
                return Err(ProcessError::IncompleteEvidence);
            }
            finishing = true;
            transcript
                .observe_finish(call)
                .map_err(|_| ProcessError::InvalidEvidence)?;
            spent_work = spent_work
                .checked_add(u64::from(call.work.units()))
                .ok_or(ProcessError::ResourceLimit)?;
            if spent_work > contract.maximum_total_work_units {
                return Err(ProcessError::ResourceLimit);
            }
            let update = session
                .finish(call.work, &mut summary)
                .map_err(|_| ProcessError::ReplayMismatch)?;
            let actual_complete = update.complete;
            let actual_update = captured_update_digest_v1(&update.update)?;
            let committed = update.update.corrected_interval;
            drop(update);
            if actual_complete != call.expected_complete {
                return Err(ProcessError::ReplayMismatch);
            }
            verify_replay_identity(call.expected_bit_exact_update_digest, actual_update)?;
            if actual_complete {
                let expected_summary = call
                    .expected_summary_digest
                    .ok_or(ProcessError::InvalidEvidence)?;
                verify_replay_identity(expected_summary, captured_summary_digest_v1(summary))?;
                complete = true;
            } else if call.expected_summary_digest.is_some() {
                return Err(ProcessError::InvalidEvidence);
            }
            trajectory
                .append_unseen_segments_from(session.trajectory(), committed)
                .map_err(|_| ProcessError::ResourceLimit)?;
            stage_new_captured_states(&trajectory, &mut state_count, &mut transaction)?;
            next_call = next_call
                .checked_add(1)
                .ok_or(ProcessError::ResourceLimit)?;
            Ok(())
        }
        EvidenceEvent::Gap { gap, .. } => {
            // A selected captured span may be preceded by the explicit gap
            // that cut it from the previous span. A gap after replay starts,
            // or one overlapping the selected span, is never bridged.
            if contract_seen
                || pending_observation.is_some()
                || restart.is_some()
                || leading_gap.replace(gap).is_some()
                || gap.span.end() > spec.span.start()
            {
                return Err(ProcessError::IncompleteEvidence);
            }
            Ok(())
        }
        EvidenceEvent::Reinitialize { evidence, .. } => {
            if contract_seen
                || pending_observation.is_some()
                || restart.replace(evidence).is_some()
                || evidence.at != spec.span.start()
            {
                return Err(ProcessError::IncompleteEvidence);
            }
            Ok(())
        }
        EvidenceEvent::ControlChange { change, .. } => {
            // A control change at the selected span boundary is provenance,
            // not an instruction to mutate an already-running candidate. The
            // following contract must name its exact effective digest.
            if contract_seen
                || pending_observation.is_some()
                || change.generation == 0
                || change.previous_digest.is_zero()
                || change.next_digest.is_zero()
                || change.at > spec.span.start()
                || control_generation.is_some_and(|generation| change.generation <= generation)
                || control_head.is_some_and(|head| head != change.previous_digest)
            {
                return Err(ProcessError::InvalidEvidence);
            }
            control_head = Some(change.next_digest);
            control_generation = Some(change.generation);
            Ok(())
        }
        EvidenceEvent::ReplaySeed { seed, .. } => {
            seed.validate_identity()?;
            // The artifact carries the complete, digest-bound opaque state,
            // but no private live-state schema is implemented by this engine
            // revision. Never reinterpret it as a fresh initialization.
            Err(ProcessError::IncompleteEvidence)
        }
        EvidenceEvent::LiveCheckpoint { .. } => {
            // Checkpoints are comparison evidence, not replay origins.
            Ok(())
        }
        EvidenceEvent::ClockModel { .. } => Ok(()),
        EvidenceEvent::End { .. } => validate_replay_end(
            contract_seen,
            complete,
            pending_observation.is_some(),
            next_call,
            spent_work,
            contract.maximum_call_count,
            contract.maximum_total_work_units,
        ),
    })?;
    if transcript.finalize() != contract.transcript_digest {
        return Err(ProcessError::ReplayMismatch);
    }
    drop(session);

    let mut metrics = crate::metric::MetricResults::new();
    metrics
        .try_prepare_bounded(usize::from(contract.metric_limits.max_results))
        .map_err(|_| ProcessError::ResourceLimit)?;
    spec.metrics
        .evaluate_into(&trajectory, &mut metrics)
        .map_err(|error| match error {
            MetricError::CapacityExceeded => ProcessError::ResourceLimit,
            _ => ProcessError::NumericalNonConvergence,
        })?;
    transaction.write(ResultRecord::Metrics(&metrics))?;
    transaction.write(ResultRecord::End(ResultEnd {
        state_count,
        objective: 0.0,
        attempted_ieks_passes: 0,
        accepted_ieks_passes: 0,
        diagnostics: summary.diagnostics,
    }))?;
    transaction.commit()?;

    Ok(crate::offline::OfflineRun {
        trajectory,
        summary: crate::offline::OfflineRunSummary {
            state_count,
            objective: 0.0,
            attempted_ieks_passes: 0,
            accepted_ieks_passes: 0,
            diagnostics: summary.diagnostics,
            used_seekable_store: false,
            state_store_record_bytes: 0,
        },
    })
}

#[cfg(feature = "offline")]
fn stage_new_captured_states<S: crate::offline::ResultSink>(
    trajectory: &Trajectory,
    state_count: &mut u64,
    transaction: &mut SinkTransaction<'_, S>,
) -> Result<(), ProcessError> {
    let previous = *state_count;
    let total = trajectory.try_for_each_knot_from(previous, |knot| {
        let state = crate::offline::SmoothedStateRecord {
            time: knot.time,
            position_ecef: knot.position_ecef,
            velocity_ecef: knot.velocity_ecef,
            orientation_ecef_from_body: knot.orientation_ecef_from_body,
            specific_force_body: knot.specific_force_body,
            covariance: knot.covariance,
            quality: knot.quality,
            observability: knot.observability,
        };
        transaction.write(ResultRecord::State(&state))
    })?;
    if total < previous {
        return Err(ProcessError::InvalidEvidence);
    }
    *state_count = total;
    Ok(())
}

#[cfg(feature = "offline")]
pub(super) fn validate_replay_step_call(
    call: CapturedLiveStepCall,
    next_call: u64,
    maximum_call_count: u64,
) -> Result<(), ProcessError> {
    if call.call_index != next_call
        || call.call_index >= maximum_call_count
        || call.expected_bit_exact_update_digest.is_zero()
    {
        Err(ProcessError::InvalidEvidence)
    } else {
        Ok(())
    }
}

#[cfg(feature = "offline")]
pub(super) fn verify_replay_identity(
    expected: ContentDigestV1,
    actual: ContentDigestV1,
) -> Result<(), ProcessError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ProcessError::ReplayMismatch)
    }
}

#[cfg(feature = "offline")]
pub(super) fn validate_replay_end(
    contract_seen: bool,
    complete: bool,
    pending_observation: bool,
    call_count: u64,
    work_units: u64,
    expected_call_count: u64,
    expected_work_units: u64,
) -> Result<(), ProcessError> {
    if !contract_seen
        || !complete
        || pending_observation
        || call_count != expected_call_count
        || work_units != expected_work_units
    {
        Err(ProcessError::IncompleteEvidence)
    } else {
        Ok(())
    }
}

#[cfg(feature = "offline")]
pub(super) fn validate_replay_finish_call(
    call: CapturedLiveFinishCall,
    next_call: u64,
    maximum_call_count: u64,
) -> Result<(), ProcessError> {
    if call.call_index != next_call
        || call.call_index >= maximum_call_count
        || call.expected_bit_exact_update_digest.is_zero()
        || call.expected_complete != call.expected_summary_digest.is_some()
        || call
            .expected_summary_digest
            .is_some_and(ContentDigestV1::is_zero)
    {
        Err(ProcessError::InvalidEvidence)
    } else {
        Ok(())
    }
}
