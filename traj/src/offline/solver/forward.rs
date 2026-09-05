//! Restartable forward passes, work accounting, and previous-pass guides.

use crate::{
    config::ProcessingSpec,
    error::ProcessError,
    offline::{
        ports::{
            ClockModelEvidence, EvidenceEvent, EvidenceManifest, EvidenceSource, SemanticStreamSeal,
        },
        store::{StateStore, StoredNominal},
    },
    quality::DiagnosticCounts,
    time::SessionTime,
};

use std::{cmp::Reverse, collections::BinaryHeap};

use super::{
    catalog::OwnedClockModel,
    evidence::{
        MAX_REORDER_TASKS, QueuedTask, ScanSummary, StreamValidator, TASK_CLASS_GAP,
        TASK_CLASS_REINITIALIZE, TimedTask, ensure_source_manifest, gap_is_valid_for_manifest,
        observation_within_span, spans_intersect, task_frontier_time, task_is_selected,
        tasks_for_observation, validate_observation, validate_reinitialization,
        validate_selected_task_lineage,
    },
    filter::OfflineFilter,
};

pub(super) struct WorkTracker<'a> {
    pub(super) control: crate::config::RunControl<'a>,
    pub(super) limit: Option<u64>,
    pub(super) completed: u64,
    pub(super) total: u64,
}

impl<'a> WorkTracker<'a> {
    pub(super) fn new(
        control: crate::config::RunControl<'a>,
        limit: Option<u64>,
        total: u64,
        completed: u64,
    ) -> Self {
        Self {
            control,
            limit,
            completed,
            total: total.max(1),
        }
    }

    pub(super) fn advance(&mut self, units: u64) -> Result<(), ProcessError> {
        self.completed = self
            .completed
            .checked_add(units)
            .ok_or(ProcessError::ResourceLimit)?;
        if self.limit.is_some_and(|limit| self.completed > limit) {
            return Err(ProcessError::ResourceLimit);
        }
        if !(self.control.continue_running)(self.completed) {
            return Err(ProcessError::Cancelled);
        }
        (self.control.progress)(self.completed.min(self.total), self.total);
        Ok(())
    }
}

pub(super) struct ForwardOutcome {
    pub(super) objective: f64,
    pub(super) diagnostics: DiagnosticCounts,
}

pub(super) fn candidate_integrity_not_worse(
    candidate: &ForwardOutcome,
    accepted: &ForwardOutcome,
) -> bool {
    candidate.objective.is_finite()
        && candidate.diagnostics.imu_epochs_accepted == accepted.diagnostics.imu_epochs_accepted
        && candidate.diagnostics.imu_epochs_rejected == accepted.diagnostics.imu_epochs_rejected
        && candidate.diagnostics.clock_discontinuities == accepted.diagnostics.clock_discontinuities
        && candidate.diagnostics.reinitializations == accepted.diagnostics.reinitializations
        && candidate.diagnostics.covariance_repairs <= accepted.diagnostics.covariance_repairs
        && candidate.diagnostics.observations_too_late == accepted.diagnostics.observations_too_late
        && candidate.diagnostics.metric_ambiguities == accepted.diagnostics.metric_ambiguities
        && candidate.diagnostics.output_overflows == accepted.diagnostics.output_overflows
}

pub(super) fn run_forward<S: EvidenceSource>(
    spec: &ProcessingSpec<'_>,
    expected: EvidenceManifest,
    scan: &ScanSummary,
    source: &mut S,
    store: &mut dyn StateStore,
    mut guide: Option<&mut dyn StateStore>,
    damping: f64,
    work: &mut WorkTracker<'_>,
) -> Result<ForwardOutcome, ProcessError> {
    ensure_source_manifest(spec, expected, source.manifest())?;
    source.restart()?;
    ensure_source_manifest(spec, expected, source.manifest())?;
    let mut validator = StreamValidator::new(expected);
    let mut semantic_seal = SemanticStreamSeal::new();
    let mut filter = OfflineFilter::new(&spec.engine, &scan.catalog, guide.is_some(), damping)?;
    let mut queue = BinaryHeap::<Reverse<QueuedTask>>::new();
    queue
        .try_reserve(MAX_REORDER_TASKS)
        .map_err(|_| ProcessError::ResourceLimit)?;
    let mut maximum_seen_time: Option<SessionTime> = None;
    let mut clock_ordinal = 0_usize;
    let mut deferred_numerical_error: Option<ProcessError> = None;

    while let Some(event) = source.next()? {
        semantic_seal.observe(event)?;
        validator.observe(event)?;
        work.advance(1)?;
        match event {
            EvidenceEvent::Observation { observation, .. } => {
                validate_observation(*observation, &spec.engine)?;
                if !observation_within_span(*observation, expected.span_capabilities.span)? {
                    return Err(ProcessError::InvalidEvidence);
                }
                let (tasks, count) = tasks_for_observation(*observation)?;
                for task in tasks.into_iter().take(count).flatten() {
                    if !task_is_selected(task, spec.span)? {
                        continue;
                    }
                    validate_selected_task_lineage(spec, task)?;
                    if deferred_numerical_error.is_none() {
                        let frontier = task_frontier_time(task)?;
                        maximum_seen_time =
                            Some(maximum_seen_time.map_or(frontier, |time| time.max(frontier)));
                        push_task(&mut queue, task)?;
                    }
                }
            }
            EvidenceEvent::ClockModel { model, .. } => {
                let expected_model = scan
                    .catalog
                    .clocks
                    .get(clock_ordinal)
                    .ok_or(ProcessError::InvalidEvidence)?;
                if !clock_matches(expected_model, model) {
                    return Err(ProcessError::InvalidEvidence);
                }
                clock_ordinal += 1;
            }
            EvidenceEvent::Gap {
                record_sequence,
                gap,
            } => {
                if !gap_is_valid_for_manifest(gap.span, expected.span_capabilities.span) {
                    return Err(ProcessError::InvalidEvidence);
                }
                if !spans_intersect(gap.span, spec.span) {
                    continue;
                }
                let task = QueuedTask {
                    time: gap.span.start(),
                    class: TASK_CLASS_GAP,
                    source: 0,
                    observation_sequence: record_sequence,
                    sub_sequence: 0,
                    task: TimedTask::Gap(gap),
                };
                if deferred_numerical_error.is_none() {
                    maximum_seen_time =
                        Some(maximum_seen_time.map_or(task.time, |time| time.max(task.time)));
                    push_task(&mut queue, task)?;
                }
            }
            EvidenceEvent::Reinitialize {
                record_sequence,
                evidence,
            } => {
                validate_reinitialization(spec, evidence)?;
                if !expected.span_capabilities.span.contains(evidence.at) {
                    return Err(ProcessError::InvalidEvidence);
                }
                if !spec.span.contains(evidence.at) {
                    continue;
                }
                let task = QueuedTask {
                    time: evidence.at,
                    class: TASK_CLASS_REINITIALIZE,
                    source: 0,
                    observation_sequence: record_sequence,
                    sub_sequence: 0,
                    task: TimedTask::Reinitialize(evidence),
                };
                if deferred_numerical_error.is_none() {
                    maximum_seen_time =
                        Some(maximum_seen_time.map_or(task.time, |time| time.max(task.time)));
                    push_task(&mut queue, task)?;
                }
            }
            EvidenceEvent::Opaque { .. }
            | EvidenceEvent::ReplayContract { .. }
            | EvidenceEvent::LiveStepCall { .. }
            | EvidenceEvent::LiveFinishCall { .. }
            | EvidenceEvent::ReplaySeed { .. }
            | EvidenceEvent::LiveCheckpoint { .. } => {}
            EvidenceEvent::End { .. } => {}
            EvidenceEvent::ControlChange { .. } => {}
        }
        if deferred_numerical_error.is_none() {
            if let Some(maximum) = maximum_seen_time {
                let delay = i64::try_from(spec.engine.navigation_profile.fusion_delay.as_ns())
                    .map_err(|_| ProcessError::ResourceLimit)?;
                let threshold = maximum
                    .as_ns()
                    .checked_sub(delay)
                    .map(SessionTime::from_ns)
                    .ok_or(ProcessError::InvalidEvidence)?;
                if let Err(error) = drain_queue(
                    &mut queue,
                    Some(threshold),
                    &mut filter,
                    store,
                    &mut guide,
                    work,
                ) {
                    if matches!(error, ProcessError::NumericalNonConvergence) {
                        deferred_numerical_error = Some(error);
                        queue.clear();
                    } else {
                        return Err(error);
                    }
                }
            }
        }
    }
    let semantic_events = validator.finish(None)?;
    verify_restarted_stream(
        scan.semantic_events,
        scan.semantic_seal,
        semantic_events,
        semantic_seal.finish(),
        deferred_numerical_error,
    )?;
    if clock_ordinal != scan.catalog.clocks.len() {
        return Err(ProcessError::InvalidEvidence);
    }
    drain_queue(&mut queue, None, &mut filter, store, &mut guide, work)?;
    if let Some(terminal) = filter.held_imu.as_ref().map(|imu| imu.time) {
        let guide_state = guide
            .as_deref_mut()
            .map(|state_store| smoothed_state_at(state_store, terminal))
            .transpose()?
            .flatten();
        if filter.flush_held_interval(store, guide_state.as_ref())? {
            work.advance(1)?;
        }
    }
    if filter
        .nominal
        .as_ref()
        .is_some_and(|state| Some(state.time) != filter.last_stored_time)
    {
        // Retain an explicit terminal guide epoch even when the final IMU lies
        // inside one navigation-cadence interval.  Subsequent IEKS passes must
        // never extrapolate past a complete connected session tail merely due
        // to export decimation.
        filter.store_current(None, None, store)?;
        work.advance(1)?;
    }
    if store.len() < 2 || filter.nominal.is_none() {
        return Err(ProcessError::IncompleteEvidence);
    }
    Ok(ForwardOutcome {
        objective: filter.objective,
        diagnostics: filter.diagnostics,
    })
}

pub(super) fn verify_restarted_stream(
    expected_events: u64,
    expected_seal: crate::ids::ContentDigestV1,
    actual_events: u64,
    actual_seal: crate::ids::ContentDigestV1,
    deferred_numerical_error: Option<ProcessError>,
) -> Result<(), ProcessError> {
    // Evidence integrity dominates numerical failure. In particular, an IEKS
    // pass may treat non-convergence as a rejected candidate, but it must
    // never mistake a changed restart stream for ordinary non-convergence.
    if actual_events != expected_events || actual_seal != expected_seal {
        return Err(ProcessError::InvalidEvidence);
    }
    if let Some(error) = deferred_numerical_error {
        return Err(error);
    }
    Ok(())
}

pub(super) fn push_task(
    queue: &mut BinaryHeap<Reverse<QueuedTask>>,
    task: QueuedTask,
) -> Result<(), ProcessError> {
    if queue.len() >= MAX_REORDER_TASKS {
        return Err(ProcessError::ResourceLimit);
    }
    queue.push(Reverse(task));
    Ok(())
}

pub(super) fn drain_queue(
    queue: &mut BinaryHeap<Reverse<QueuedTask>>,
    through: Option<SessionTime>,
    filter: &mut OfflineFilter<'_>,
    store: &mut dyn StateStore,
    guide: &mut Option<&mut dyn StateStore>,
    work: &mut WorkTracker<'_>,
) -> Result<(), ProcessError> {
    loop {
        let should_pop = queue
            .peek()
            .is_some_and(|Reverse(task)| through.is_none_or(|limit| task.time <= limit));
        if !should_pop {
            return Ok(());
        }
        let Some(Reverse(task)) = queue.pop() else {
            return Ok(());
        };
        let guide_state = guide
            .as_deref_mut()
            .map(|state_store| smoothed_state_at(state_store, task.time))
            .transpose()?
            .flatten();
        filter.process(task, store, guide_state)?;
        work.advance(1)?;
    }
}

pub(super) fn smoothed_state_at(
    store: &mut dyn StateStore,
    time: SessionTime,
) -> Result<Option<StoredNominal>, ProcessError> {
    let mut lower = 0_u64;
    let mut upper = store.len();
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        let step = store.get(middle).map_err(ProcessError::from)?;
        if step.filtered.time <= time {
            lower = middle.saturating_add(1);
        } else {
            upper = middle;
        }
    }
    if lower > 0 {
        let previous = store.get(lower - 1).map_err(ProcessError::from)?;
        if previous.filtered.time == time {
            return previous
                .smoothed
                .ok_or(ProcessError::StorageCorrupt)
                .map(Some);
        }
        if lower < store.len() {
            let next = store.get(lower).map_err(ProcessError::from)?;
            if next.connected_from_previous {
                let previous = previous
                    .smoothed
                    .as_ref()
                    .ok_or(ProcessError::StorageCorrupt)?;
                let next = next.smoothed.as_ref().ok_or(ProcessError::StorageCorrupt)?;
                return interpolate_nominal(previous, next, time).map(Some);
            }
        }
    }
    Ok(None)
}

pub(super) fn interpolate_nominal(
    start: &StoredNominal,
    end: &StoredNominal,
    time: SessionTime,
) -> Result<StoredNominal, ProcessError> {
    let duration = end
        .time
        .checked_duration_since(start.time)
        .ok_or(ProcessError::StorageCorrupt)?
        .as_seconds_f64();
    let elapsed = time
        .checked_duration_since(start.time)
        .ok_or(ProcessError::StorageCorrupt)?
        .as_seconds_f64();
    if !duration.is_finite()
        || duration <= 0.0
        || !elapsed.is_finite()
        || !(0.0..=duration).contains(&elapsed)
    {
        return Err(ProcessError::StorageCorrupt);
    }
    let fraction = elapsed / duration;
    let interpolate3 = |left: [f64; 3], right: [f64; 3]| {
        [
            left[0] + fraction * (right[0] - left[0]),
            left[1] + fraction * (right[1] - left[1]),
            left[2] + fraction * (right[2] - left[2]),
        ]
    };
    let nominal = StoredNominal {
        time,
        position_ecef: interpolate3(start.position_ecef, end.position_ecef),
        velocity_ecef: interpolate3(start.velocity_ecef, end.velocity_ecef),
        orientation_ecef_from_body: start
            .orientation_ecef_from_body
            .slerp(end.orientation_ecef_from_body, fraction)
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
        accelerometer_bias_body: interpolate3(
            start.accelerometer_bias_body,
            end.accelerometer_bias_body,
        ),
        gyroscope_bias_body: interpolate3(start.gyroscope_bias_body, end.gyroscope_bias_body),
        colored_gnss_error: interpolate3(start.colored_gnss_error, end.colored_gnss_error),
        specific_force_body: interpolate3(start.specific_force_body, end.specific_force_body),
        angular_rate_body: interpolate3(start.angular_rate_body, end.angular_rate_body),
    };
    if nominal.is_finite() {
        Ok(nominal)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn clock_matches(expected: &OwnedClockModel, actual: ClockModelEvidence<'_>) -> bool {
    expected.model == actual.model
        && expected.segment == actual.segment
        && expected.validity == actual.validity
        && expected.reference_time == actual.reference_time
        && expected.offset_ns.to_bits() == actual.offset_ns.to_bits()
        && expected.fractional_drift.to_bits() == actual.fractional_drift.to_bits()
        && expected
            .covariance_upper
            .iter()
            .zip(actual.covariance_upper)
            .all(|(left, right)| left.to_bits() == right.to_bits())
        && expected.cross_covariance_with_prior.len() == actual.cross_covariance_with_prior.len()
        && expected
            .cross_covariance_with_prior
            .iter()
            .zip(actual.cross_covariance_with_prior)
            .all(|(left, right)| left.to_bits() == right.to_bits())
}
