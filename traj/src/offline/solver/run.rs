//! Offline resource planning, refinement passes, and transactional publication.

use crate::{
    config::{GnssCorrelationPolicy, OfflineResourceLimits, ProcessingSpec},
    error::ProcessError,
    offline::{
        ports::{
            EvidenceManifest, EvidenceSource, ResultDescriptor, ResultEnd, ResultRecord,
            ResultRecordBounds, ResultSink, SinkPreflightReservation,
        },
        store::{
            FixedRecordStoreKind, StateStoreResourceBounds, StoreKind, plan_store_kind,
            state_store_resource_bounds,
        },
    },
    provenance::{Capability, ResultProvenance},
    trajectory::{OfflineTrajectoryStorageBounds, Trajectory},
};

use super::{
    OfflineRun, OfflineRunSummary,
    evidence::{ensure_source_manifest, metric_result_upper_bound, scan_source},
    forward::{WorkTracker, candidate_integrity_not_worse, run_forward},
    math::{COLORED_ERROR_DIMENSION, NAVIGATION_DIMENSION},
    metric_uncertainty::OfflineMetricUncertainty,
    publication::{build_trajectory, publish_states},
    smoothing::{maximum_smoothed_difference, smooth_store},
};

pub(super) const MAX_IEKS_PASSES: u8 = 3;

pub(super) const IEKS_OBJECTIVE_RELATIVE_TOLERANCE: f64 = 1.0e-8;

pub(super) const IEKS_STEP_TOLERANCE: f64 = 1.0e-7;

pub(super) const IEKS_MINIMUM_DAMPING: f64 = 0.5;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct OfflineStoragePlan {
    pub(super) state: StoreKind,
    pub(super) trajectory: FixedRecordStoreKind,
}

pub(super) fn choose_offline_storage_plan(
    state: StateStoreResourceBounds,
    trajectory: OfflineTrajectoryStorageBounds,
    limits: OfflineResourceLimits,
) -> Result<OfflineStoragePlan, ProcessError> {
    let candidates = [
        OfflineStoragePlan {
            state: StoreKind::Memory,
            trajectory: FixedRecordStoreKind::Memory,
        },
        OfflineStoragePlan {
            state: StoreKind::Memory,
            trajectory: FixedRecordStoreKind::SeekableTemporary,
        },
        OfflineStoragePlan {
            state: StoreKind::SeekableTemporary,
            trajectory: FixedRecordStoreKind::Memory,
        },
        OfflineStoragePlan {
            state: StoreKind::SeekableTemporary,
            trajectory: FixedRecordStoreKind::SeekableTemporary,
        },
    ];
    for candidate in candidates {
        let state_peak = match candidate.state {
            StoreKind::Memory => state.memory_peak_bytes,
            StoreKind::SeekableTemporary => state.seekable_peak_bytes,
        };
        let state_temporary = match candidate.state {
            StoreKind::Memory => 0,
            StoreKind::SeekableTemporary => state.seekable_temporary_bytes,
        };
        let trajectory_peak = match candidate.trajectory {
            FixedRecordStoreKind::Memory => trajectory.memory_peak_bytes,
            FixedRecordStoreKind::SeekableTemporary => trajectory.seekable_peak_bytes,
        };
        let trajectory_temporary = match candidate.trajectory {
            FixedRecordStoreKind::Memory => 0,
            FixedRecordStoreKind::SeekableTemporary => trajectory.seekable_temporary_bytes,
        };
        let peak_fits = state_peak
            .checked_add(trajectory_peak)
            .is_some_and(|bytes| bytes <= limits.peak_memory_bytes);
        let temporary_fits = state_temporary
            .checked_add(trajectory_temporary)
            .is_some_and(|bytes| bytes <= limits.temporary_storage_bytes);
        if peak_fits && temporary_fits {
            return Ok(candidate);
        }
    }
    Err(ProcessError::StorageExhausted)
}

/// Runs the offline Rust fixed-interval refinement and publishes its semantic
/// sidecar transactionally.
pub(crate) fn run_offline<S: EvidenceSource, K: ResultSink>(
    spec: &ProcessingSpec<'_>,
    expected_manifest: EvidenceManifest,
    limits: OfflineResourceLimits,
    provenance: ResultProvenance<'_>,
    source: &mut S,
    sink: &mut K,
    control: crate::config::RunControl<'_>,
) -> Result<OfflineRun, ProcessError> {
    spec.validate().map_err(|_| ProcessError::InvalidEvidence)?;
    provenance
        .validate()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    if provenance.source_session != expected_manifest.session_id
        || provenance.source_span != spec.span
        || provenance.source_digest != expected_manifest.source_logical_digest
        || provenance.normalization_digest != expected_manifest.normalization_digest
        || provenance.configuration_digest != spec.engine.digest
        || provenance.installation_digest != spec.engine.installation.digest
        || provenance.calibration_revision != spec.engine.calibration.revision
        || provenance.calibration_digest != spec.engine.calibration.digest
        || provenance.uncertainty_digest != spec.result.uncertainty_digest
        || provenance.metric_plan_digest != spec.result.metric_plan_digest
        || provenance.requested_policy != spec.policy
        || provenance.actual_backend.level != crate::config::ProcessingLevel::OfflineSmooth
    {
        return Err(ProcessError::InvalidEvidence);
    }
    limits.validate().map_err(|_| ProcessError::ResourceLimit)?;
    ensure_source_manifest(spec, expected_manifest, source.manifest())?;
    let required = crate::provenance::Capabilities::NONE
        .with(Capability::OfflineSmooth)
        .with(Capability::NormalizedImu)
        .with(Capability::GnssSolution)
        .with(Capability::Timing)
        .with(Capability::Configuration)
        .with(Capability::CompleteEnd);
    if !expected_manifest
        .span_capabilities
        .capabilities
        .contains_all(required)
    {
        return Err(ProcessError::IncompleteEvidence);
    }

    let scan = scan_source(spec, expected_manifest, source, limits, control)?;
    let revision = spec.result.trajectory_revision;
    let descriptor = ResultDescriptor {
        provenance,
        trajectory_revision: revision,
    };
    let metric_result_bound = metric_result_upper_bound(spec)?;
    let record_bounds = ResultRecordBounds::new(scan.maximum_records, metric_result_bound)?;
    // Sink framing and staging are adapter-specific, so the first sound host
    // output preflight point is immediately after the evidence scan has
    // established the state bound. Hold only the unpublished reservation
    // across numerical work; `begin` still occurs at final publication.
    let sink_reservation =
        SinkPreflightReservation::preflight(sink, descriptor, record_bounds, limits.output_bytes)?;
    let state_dimension = NAVIGATION_DIMENSION
        + if matches!(
            spec.engine.dynamics_profile.gnss.correlation,
            GnssCorrelationPolicy::GaussMarkov { .. }
        ) {
            COLORED_ERROR_DIMENSION
        } else {
            0
        };
    let maximum_segments = scan
        .maximum_records
        .checked_sub(1)
        .filter(|value| *value > 0)
        .ok_or(ProcessError::IncompleteEvidence)?;
    let state_store_bounds = state_store_resource_bounds(
        state_dimension,
        &scan.catalog.covariance,
        scan.maximum_records,
    )?;
    let trajectory_store_bounds = Trajectory::offline_storage_bounds(maximum_segments)
        .map_err(|_| ProcessError::ResourceLimit)?;
    let storage_plan =
        choose_offline_storage_plan(state_store_bounds, trajectory_store_bounds, limits)?;
    let total_work = scan
        .semantic_events
        .checked_mul(u64::from(MAX_IEKS_PASSES) + 1)
        .and_then(|value| {
            scan.maximum_records
                .checked_mul(u64::from(MAX_IEKS_PASSES) * 2)
                .and_then(|records| value.checked_add(records))
        })
        .ok_or(ProcessError::ResourceLimit)?;
    let mut work = WorkTracker::new(
        control,
        limits.elapsed_work_limit,
        total_work,
        scan.semantic_events,
    );
    let mut planned = plan_store_kind(
        state_dimension,
        &scan.catalog.covariance,
        scan.maximum_records,
        limits,
        storage_plan.state,
    )?;
    if planned.store.consider_covariance() != &scan.catalog.covariance {
        return Err(ProcessError::StorageCorrupt);
    }
    let mut outcome = run_forward(
        spec,
        expected_manifest,
        &scan,
        source,
        planned.store.as_mut(),
        None,
        1.0,
        &mut work,
    )?;
    let initial_smoothing = smooth_store(
        planned.store.as_mut(),
        &scan.catalog,
        spec.engine
            .navigation_profile
            .covariance_repair
            .maximum_attempts,
        spec.engine
            .navigation_profile
            .covariance_repair
            .maximum_total_regularization
            .get(),
        &mut work,
    )?;
    outcome.diagnostics.covariance_repairs = outcome
        .diagnostics
        .covariance_repairs
        .saturating_add(initial_smoothing.covariance_repairs);
    let mut maximum_step = initial_smoothing.maximum_step;
    let mut attempted_passes = 1_u8;
    let mut accepted_ieks_passes = 0_u8;
    let mut damping = 1.0_f64;

    // A second store preserves the last valid solution until a relinearized
    // candidate has demonstrably reduced the declared objective.  We only
    // enter the outer loop when both stores fit the caller's complete resource
    // ceiling; otherwise the valid one-pass RTS result remains the result.
    let double_store_fits = match planned.kind {
        StoreKind::Memory => state_store_bounds
            .memory_peak_bytes
            .checked_mul(2)
            .is_some_and(|value| value <= limits.peak_memory_bytes),
        StoreKind::SeekableTemporary => {
            state_store_bounds
                .seekable_peak_bytes
                .checked_mul(2)
                .is_some_and(|value| value <= limits.peak_memory_bytes)
                && state_store_bounds
                    .seekable_temporary_bytes
                    .checked_mul(2)
                    .is_some_and(|value| value <= limits.temporary_storage_bytes)
        }
    };
    while attempted_passes < MAX_IEKS_PASSES
        && maximum_step > IEKS_STEP_TOLERANCE
        && double_store_fits
    {
        let mut candidate = plan_store_kind(
            state_dimension,
            &scan.catalog.covariance,
            scan.maximum_records,
            limits,
            storage_plan.state,
        )?;
        attempted_passes = attempted_passes.saturating_add(1);
        let candidate_result = (|| {
            let mut candidate_outcome = run_forward(
                spec,
                expected_manifest,
                &scan,
                source,
                candidate.store.as_mut(),
                Some(planned.store.as_mut()),
                damping,
                &mut work,
            )?;
            let smoothing = smooth_store(
                candidate.store.as_mut(),
                &scan.catalog,
                spec.engine
                    .navigation_profile
                    .covariance_repair
                    .maximum_attempts,
                spec.engine
                    .navigation_profile
                    .covariance_repair
                    .maximum_total_regularization
                    .get(),
                &mut work,
            )?;
            candidate_outcome.diagnostics.covariance_repairs = candidate_outcome
                .diagnostics
                .covariance_repairs
                .saturating_add(smoothing.covariance_repairs);
            let candidate_step =
                maximum_smoothed_difference(candidate.store.as_mut(), planned.store.as_mut())?;
            Ok::<_, ProcessError>((candidate_outcome, candidate_step))
        })();
        let (candidate_outcome, candidate_step) = match candidate_result {
            Ok(value) => value,
            Err(ProcessError::NumericalNonConvergence) if damping > IEKS_MINIMUM_DAMPING => {
                damping *= 0.5;
                continue;
            }
            Err(ProcessError::NumericalNonConvergence) => break,
            Err(error) => return Err(error),
        };
        let required_decrease =
            IEKS_OBJECTIVE_RELATIVE_TOLERANCE * outcome.objective.abs().max(1.0);
        if candidate_outcome.objective + required_decrease < outcome.objective
            && candidate_integrity_not_worse(&candidate_outcome, &outcome)
            && candidate_step.is_finite()
        {
            planned = candidate;
            outcome = candidate_outcome;
            maximum_step = candidate_step;
            accepted_ieks_passes = accepted_ieks_passes.saturating_add(1);
            damping = 1.0;
        } else if damping > IEKS_MINIMUM_DAMPING {
            damping *= 0.5;
        } else {
            break;
        }
    }
    planned.store.finish().map_err(ProcessError::from)?;

    let trajectory = build_trajectory(
        spec,
        &scan.catalog,
        planned.store.as_mut(),
        revision,
        maximum_segments,
        storage_plan.trajectory,
    )?;
    let metrics = {
        let mut uncertainty = OfflineMetricUncertainty::new(
            planned.store.as_mut(),
            spec.engine.installation.reference_points,
            &scan.catalog,
            spec.engine.calibration.shared_parameters.treatment,
        );
        spec.metrics
            .evaluate_with_uncertainty(&trajectory, &mut uncertainty)
            .map_err(|_| ProcessError::NumericalNonConvergence)?
    };
    let mut transaction = sink_reservation.begin()?;
    let state_count = publish_states(spec, planned.store.as_mut(), &mut transaction)?;
    transaction.write(ResultRecord::Metrics(&metrics))?;
    transaction.write(ResultRecord::End(ResultEnd {
        state_count,
        objective: outcome.objective,
        attempted_ieks_passes: attempted_passes,
        accepted_ieks_passes,
        diagnostics: outcome.diagnostics,
    }))?;
    transaction.commit()?;

    Ok(OfflineRun {
        trajectory,
        summary: OfflineRunSummary {
            state_count,
            objective: outcome.objective,
            attempted_ieks_passes: attempted_passes,
            accepted_ieks_passes,
            diagnostics: outcome.diagnostics,
            used_seekable_store: matches!(planned.kind, StoreKind::SeekableTemporary),
            state_store_record_bytes: planned.record_bytes,
        },
    })
}
