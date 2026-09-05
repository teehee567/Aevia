//! Backend qualification, evidence lineage, resource bounds, and fallback classification.

use crate::config::{LiveSpec, OfflineResourceLimits, ProcessingLevel, ProcessingSpec};
use crate::engine::TrajectoryEngine;
use crate::error::{PrepareError, ProcessError, ValidationError};
use crate::provenance::{
    Capabilities, Capability, EvidenceClass, EvidenceLineageKind, EvidenceUse,
    MAX_PROCESSING_ATTEMPTS, ProcessingAttempt, ProcessingAttemptOutcome,
};
use crate::time::TimeSpan;
use crate::trajectory::Trajectory;

#[cfg(feature = "offline")]
pub(super) fn select_offline_level(
    spec: &ProcessingSpec<'_>,
    manifest: crate::offline::EvidenceManifest,
    limits: OfflineResourceLimits,
) -> Result<
    (
        ProcessingLevel,
        heapless::Vec<ProcessingAttempt, MAX_PROCESSING_ATTEMPTS>,
    ),
    PrepareError,
> {
    let _ = limits;
    let mut preferred_failure = None;
    let mut attempts = heapless::Vec::<ProcessingAttempt, MAX_PROCESSING_ATTEMPTS>::new();
    for (ordinal, level) in spec.policy.levels().iter().copied().enumerate() {
        let lineage_available = solution_level_lineage_available(
            spec.evidence_lineage,
            spec.span,
            level == ProcessingLevel::CapturedReplay,
        );
        let candidate = spec
            .evidence_lineage
            .validate_for_level(level)
            .map_err(|_| PrepareError::EvidenceUnavailable)
            .and_then(|_| {
                offline_level_preflight(level, spec, manifest, limits, lineage_available)
            });
        match candidate {
            Ok(()) => {
                attempts
                    .push(ProcessingAttempt {
                        level,
                        ordinal: ordinal as u8,
                        outcome: ProcessingAttemptOutcome::Succeeded,
                    })
                    .map_err(|_| PrepareError::InsufficientResources)?;
                return Ok((level, attempts));
            }
            Err(error) => {
                attempts
                    .push(ProcessingAttempt {
                        level,
                        ordinal: ordinal as u8,
                        outcome: attempt_outcome_for_prepare_error(error),
                    })
                    .map_err(|_| PrepareError::InsufficientResources)?;
                if matches!(spec.policy, crate::config::ProcessingPolicy::Require(_)) {
                    return Err(error);
                }
                // Ordered fallback reports the highest-preference rejection if
                // no later candidate is usable.
                preferred_failure.get_or_insert(error);
            }
        }
    }
    Err(preferred_failure.unwrap_or(PrepareError::CapabilityUnavailable))
}

#[cfg(feature = "offline")]
pub(super) fn select_next_offline_level(
    spec: &ProcessingSpec<'_>,
    manifest: crate::offline::EvidenceManifest,
    limits: OfflineResourceLimits,
    attempts: &mut heapless::Vec<ProcessingAttempt, MAX_PROCESSING_ATTEMPTS>,
) -> Result<Option<ProcessingLevel>, PrepareError> {
    for (ordinal, level) in spec
        .policy
        .levels()
        .iter()
        .copied()
        .enumerate()
        .skip(attempts.len())
    {
        let lineage_available = solution_level_lineage_available(
            spec.evidence_lineage,
            spec.span,
            level == ProcessingLevel::CapturedReplay,
        );
        let candidate = spec
            .evidence_lineage
            .validate_for_level(level)
            .map_err(|_| PrepareError::EvidenceUnavailable)
            .and_then(|_| {
                offline_level_preflight(level, spec, manifest, limits, lineage_available)
            });
        let outcome = match candidate {
            Ok(()) => ProcessingAttemptOutcome::Succeeded,
            Err(error) => attempt_outcome_for_prepare_error(error),
        };
        attempts
            .push(ProcessingAttempt {
                level,
                ordinal: ordinal as u8,
                outcome,
            })
            .map_err(|_| PrepareError::InsufficientResources)?;
        if candidate.is_ok() {
            return Ok(Some(level));
        }
    }
    Ok(None)
}

#[cfg(feature = "offline")]
const fn attempt_outcome_for_prepare_error(error: PrepareError) -> ProcessingAttemptOutcome {
    match error {
        PrepareError::CapabilityUnavailable => ProcessingAttemptOutcome::NotCompiled,
        PrepareError::PlatformUnsupported => ProcessingAttemptOutcome::PlatformUnsupported,
        PrepareError::EvidenceUnavailable
        | PrepareError::ReplayContractUnavailable
        | PrepareError::NotRestartable => ProcessingAttemptOutcome::EvidenceUnavailable,
        PrepareError::UnqualifiedProfile => ProcessingAttemptOutcome::NotQualified,
        PrepareError::InsufficientResources
        | PrepareError::InvalidWorkspaceAlignment
        | PrepareError::InvalidDefinition(ValidationError::CapacityExceeded) => {
            ProcessingAttemptOutcome::InsufficientResources
        }
        PrepareError::InvalidDefinition(_)
        | PrepareError::IncompatibleProfile
        | PrepareError::FrameUnresolved
        | PrepareError::CalibrationUnavailable => ProcessingAttemptOutcome::Failed,
    }
}

#[cfg(feature = "offline")]
pub(super) const fn attempt_outcome_for_process_error(
    error: ProcessError,
) -> ProcessingAttemptOutcome {
    match error {
        ProcessError::IncompleteEvidence | ProcessError::EvidenceLineageConflict => {
            ProcessingAttemptOutcome::EvidenceUnavailable
        }
        ProcessError::CapabilityUnavailable => ProcessingAttemptOutcome::NotCompiled,
        ProcessError::StorageExhausted | ProcessError::ResourceLimit => {
            ProcessingAttemptOutcome::InsufficientResources
        }
        ProcessError::InvalidEvidence
        | ProcessError::SourceFailure
        | ProcessError::SinkFailure
        | ProcessError::StorageCorrupt
        | ProcessError::Cancelled
        | ProcessError::NumericalNonConvergence
        | ProcessError::AdvancedCapabilityFailure
        | ProcessError::ReplayMismatch => ProcessingAttemptOutcome::Failed,
    }
}

#[cfg(feature = "offline")]
pub(super) const fn runtime_failure_allows_fallback(error: ProcessError) -> bool {
    matches!(
        error,
        ProcessError::IncompleteEvidence
            | ProcessError::EvidenceLineageConflict
            | ProcessError::StorageExhausted
            | ProcessError::ResourceLimit
            | ProcessError::StorageCorrupt
            | ProcessError::NumericalNonConvergence
            | ProcessError::AdvancedCapabilityFailure
            | ProcessError::ReplayMismatch
            | ProcessError::CapabilityUnavailable
    )
}

#[cfg(feature = "offline")]
pub(super) const fn process_error_for_prepare_error(error: PrepareError) -> ProcessError {
    match error {
        PrepareError::EvidenceUnavailable
        | PrepareError::ReplayContractUnavailable
        | PrepareError::NotRestartable => ProcessError::IncompleteEvidence,
        PrepareError::CapabilityUnavailable | PrepareError::PlatformUnsupported => {
            ProcessError::CapabilityUnavailable
        }
        PrepareError::InsufficientResources
        | PrepareError::InvalidWorkspaceAlignment
        | PrepareError::InvalidDefinition(ValidationError::CapacityExceeded) => {
            ProcessError::ResourceLimit
        }
        PrepareError::InvalidDefinition(_)
        | PrepareError::IncompatibleProfile
        | PrepareError::FrameUnresolved
        | PrepareError::CalibrationUnavailable
        | PrepareError::UnqualifiedProfile => ProcessError::AdvancedCapabilityFailure,
    }
}

#[cfg(feature = "offline")]
fn offline_level_preflight(
    level: ProcessingLevel,
    spec: &ProcessingSpec<'_>,
    manifest: crate::offline::EvidenceManifest,
    limits: OfflineResourceLimits,
    lineage_available: bool,
) -> Result<(), PrepareError> {
    let capabilities = manifest.span_capabilities.capabilities;
    offline_level_evidence_preflight(
        level,
        capabilities,
        lineage_available,
        manifest.captured_replay.is_some(),
    )?;
    if level == ProcessingLevel::CapturedReplay {
        captured_replay_preflight(spec, manifest, limits).map(|_| ())
    } else {
        Ok(())
    }
}

#[cfg(feature = "offline")]
pub(super) fn offline_level_evidence_preflight(
    level: ProcessingLevel,
    capabilities: Capabilities,
    lineage_available: bool,
    replay_contract_available: bool,
) -> Result<(), PrepareError> {
    let common = Capabilities::NONE
        .with(Capability::NormalizedImu)
        .with(Capability::GnssSolution)
        .with(Capability::Timing)
        .with(Capability::Configuration)
        .with(Capability::CompleteEnd);
    match level {
        ProcessingLevel::CapturedReplay => {
            if !lineage_available
                || !capabilities.contains_all(common.with(Capability::CapturedReplay))
            {
                return Err(PrepareError::EvidenceUnavailable);
            }
            if replay_contract_available {
                Ok(())
            } else {
                Err(PrepareError::ReplayContractUnavailable)
            }
        }
        ProcessingLevel::OfflineSmooth => {
            if lineage_available
                && capabilities.contains_all(common.with(Capability::OfflineSmooth))
            {
                Ok(())
            } else {
                Err(PrepareError::EvidenceUnavailable)
            }
        }
        ProcessingLevel::AdvancedGraph | ProcessingLevel::RawTight => {
            Err(PrepareError::CapabilityUnavailable)
        }
        ProcessingLevel::EmbeddedLive => Err(PrepareError::PlatformUnsupported),
    }
}

#[cfg(feature = "offline")]
pub(super) fn captured_replay_preflight<'a>(
    spec: &ProcessingSpec<'a>,
    manifest: crate::offline::EvidenceManifest,
    limits: OfflineResourceLimits,
) -> Result<crate::metric::LiveMetricPlan, PrepareError> {
    let contract = manifest
        .captured_replay
        .ok_or(PrepareError::ReplayContractUnavailable)?
        .validate()
        .map_err(PrepareError::InvalidDefinition)?;
    let event_count = manifest
        .estimated_event_count
        .ok_or(PrepareError::ReplayContractUnavailable)?;
    let total_control_work = event_count
        .checked_add(contract.maximum_total_work_units)
        .ok_or(PrepareError::InsufficientResources)?;
    if spec.span != manifest.span_capabilities.span
        || contract.configuration_digest != manifest.configuration_digest
        || contract.configuration_digest != spec.engine.digest
        || contract.navigation_profile_digest != spec.engine.navigation_profile.digest
        || contract.metric_plan_digest != spec.result.metric_plan_digest
        || contract.maximum_call_count > event_count
    {
        return Err(PrepareError::ReplayContractUnavailable);
    }
    if limits
        .elapsed_work_limit
        .is_some_and(|limit| total_control_work > limit)
    {
        return Err(PrepareError::InsufficientResources);
    }
    let metrics = spec
        .metrics
        .compile_live(contract.metric_limits)
        .map_err(PrepareError::InvalidDefinition)?;
    let plan = TrajectoryEngine::live(LiveSpec {
        session_id: manifest.session_id,
        engine: spec.engine.clone(),
        metrics: &metrics,
        resources: contract.resources,
        initial_heading: contract.initial_heading,
        initial_clock_prior: contract.initial_clock_prior,
    })
    .preflight()?;
    let required = plan.requirements();
    let resident = u64::try_from(required.internal_sram_bytes())
        .ok()
        .and_then(|bytes| bytes.checked_add(u64::try_from(required.psram_bytes()).ok()?))
        .ok_or(PrepareError::InsufficientResources)?;
    let maximum_segments = usize::try_from(contract.maximum_total_work_units)
        .map_err(|_| PrepareError::InsufficientResources)?;
    let rolling_segments =
        maximum_segments.min(crate::trajectory::MAX_EMBEDDED_TRAJECTORY_SEGMENTS);
    let total_segment_capacity = maximum_segments
        .checked_add(rolling_segments)
        .ok_or(PrepareError::InsufficientResources)?;
    let segment_bound = u64::try_from(Trajectory::dense_segment_size_bytes())
        .ok()
        .and_then(|bytes| bytes.checked_mul(u64::try_from(total_segment_capacity).ok()?))
        .ok_or(PrepareError::InsufficientResources)?;
    let accumulator = u64::try_from(core::mem::size_of::<Trajectory>())
        .map_err(|_| PrepareError::InsufficientResources)?;
    let metric_bound = u64::try_from(core::mem::size_of::<crate::metric::MetricResult>())
        .ok()
        .and_then(|bytes| bytes.checked_mul(u64::from(contract.metric_limits.max_results)))
        .ok_or(PrepareError::InsufficientResources)?;
    let peak_bound = resident
        .checked_add(accumulator)
        .and_then(|bytes| bytes.checked_add(segment_bound))
        .and_then(|bytes| bytes.checked_add(metric_bound))
        .ok_or(PrepareError::InsufficientResources)?;
    if peak_bound > limits.peak_memory_bytes {
        return Err(PrepareError::InsufficientResources);
    }
    Ok(metrics)
}

#[cfg(feature = "offline")]
pub(super) fn solution_level_lineage_available(
    lineage: crate::provenance::EvidenceLineage<'_>,
    span: TimeSpan,
    captured_only: bool,
) -> bool {
    // A solution-level run needs an explicit normalized fusion lineage for
    // every semantic class that can affect its state. Capability bits alone
    // are not evidence selection. Exact captured replay additionally forbids
    // recomputed normalization.
    [
        EvidenceClass::Imu,
        EvidenceClass::GnssSolution,
        EvidenceClass::Timing,
        EvidenceClass::Control,
    ]
    .into_iter()
    .all(|class| {
        lineage.selections().iter().any(|selection| {
            selection.class == class
                && matches!(
                    selection.lineage,
                    EvidenceLineageKind::Captured | EvidenceLineageKind::Recomputed
                )
                && (!captured_only || selection.lineage == EvidenceLineageKind::Captured)
                && selection.usage == EvidenceUse::Fusion
                && selection.span.contains(span.start())
                && selection.span.contains(span.end())
        })
    })
}
