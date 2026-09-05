//! Evidence validation, canonical task ordering, and captured transcript replay.

use crate::{
    config::{EngineConfig, OfflineResourceLimits, ProcessingSpec},
    error::ProcessError,
    ids::SourceId,
    metric::MetricDefinition,
    observation::{GnssSolutionObservation, ImuObservation, LiveObservation},
    offline::ports::{
        CAPTURED_REINITIALIZATION_SCHEMA_V2, EvidenceEnd, EvidenceEvent, EvidenceManifest,
        EvidenceSource, SemanticStreamSeal,
    },
    provenance::{
        Capability, EvidenceClass, EvidenceLineageKind, EvidenceUse, MAX_EVIDENCE_SELECTIONS,
    },
    time::{SessionTime, TimeSpan},
};

use std::{cmp::Ordering, vec::Vec};

use super::{
    catalog::ConsiderCatalog, estimation::matrix_is_psd, inertial::qualified_imu_support,
    measurement::validate_antenna,
};

pub(super) const MAX_REORDER_TASKS: usize = 2_048;

pub(super) const TASK_CLASS_GAP: u8 = 0;

pub(super) const TASK_CLASS_REINITIALIZE: u8 = 1;

pub(super) const TASK_CLASS_CLOCK_TRANSITION: u8 = 2;

pub(super) const TASK_CLASS_IMU: u8 = 3;

pub(super) const TASK_CLASS_GNSS_POSITION: u8 = 4;

pub(super) const TASK_CLASS_GNSS_VELOCITY: u8 = 5;

pub(super) struct ScanSummary {
    pub(super) catalog: ConsiderCatalog,
    pub(super) semantic_events: u64,
    pub(super) maximum_records: u64,
    pub(super) semantic_seal: crate::ids::ContentDigestV1,
}

pub(super) struct StreamValidator {
    pub(super) expected: EvidenceManifest,
    pub(super) last_sequence: Option<u64>,
    pub(super) saw_end: bool,
    pub(super) semantic_events: u64,
    pub(super) observation_sequences: Vec<(u32, u64)>,
    pub(super) control_head: Option<crate::ids::ContentDigestV1>,
    pub(super) control_generation: Option<u32>,
    pub(super) reinitialization_generation: Option<u32>,
}

impl StreamValidator {
    pub(super) fn new(expected: EvidenceManifest) -> Self {
        Self {
            expected,
            last_sequence: None,
            saw_end: false,
            semantic_events: 0,
            observation_sequences: Vec::new(),
            control_head: None,
            control_generation: None,
            reinitialization_generation: None,
        }
    }

    pub(super) fn observe(&mut self, event: EvidenceEvent<'_>) -> Result<(), ProcessError> {
        if self.saw_end {
            return Err(ProcessError::InvalidEvidence);
        }
        if matches!(
            event,
            EvidenceEvent::Opaque {
                semantic_digest,
                ..
            } if semantic_digest.is_zero()
        ) {
            return Err(ProcessError::InvalidEvidence);
        }
        let sequence = event.record_sequence();
        if self.last_sequence.is_none() && sequence != 0 {
            return Err(ProcessError::InvalidEvidence);
        }
        if let Some(previous) = self.last_sequence {
            let expected = previous
                .checked_add(1)
                .ok_or(ProcessError::InvalidEvidence)?;
            if sequence != expected {
                return Err(ProcessError::InvalidEvidence);
            }
        }
        self.last_sequence = Some(sequence);
        if let EvidenceEvent::Observation { observation, .. } = event {
            let id = observation.id();
            if id.source.get() == 0 {
                return Err(ProcessError::InvalidEvidence);
            }
            if let Some((_, previous)) = self
                .observation_sequences
                .iter_mut()
                .find(|(source, _)| *source == id.source.get())
            {
                if id.sequence <= *previous {
                    return Err(ProcessError::InvalidEvidence);
                }
                *previous = id.sequence;
            } else {
                if self.observation_sequences.len() >= MAX_EVIDENCE_SELECTIONS {
                    return Err(ProcessError::ResourceLimit);
                }
                self.observation_sequences
                    .try_reserve(1)
                    .map_err(|_| ProcessError::ResourceLimit)?;
                self.observation_sequences
                    .push((id.source.get(), id.sequence));
            }
        }
        if let EvidenceEvent::ControlChange { change, .. } = event {
            if change.generation == 0
                || change.previous_digest.is_zero()
                || change.next_digest.is_zero()
                || change.at > self.expected.span_capabilities.span.start()
                || self
                    .control_generation
                    .is_some_and(|generation| change.generation <= generation)
                || self
                    .control_head
                    .is_some_and(|head| head != change.previous_digest)
            {
                return Err(ProcessError::InvalidEvidence);
            }
            self.control_head = Some(change.next_digest);
            self.control_generation = Some(change.generation);
        }
        if let EvidenceEvent::Reinitialize { evidence, .. } = event {
            if evidence.generation == 0
                || evidence.input_schema != CAPTURED_REINITIALIZATION_SCHEMA_V2
                || evidence.input_digest.is_zero()
                || evidence.configuration_digest.is_zero()
                || self
                    .reinitialization_generation
                    .is_some_and(|generation| evidence.generation <= generation)
            {
                return Err(ProcessError::InvalidEvidence);
            }
            self.reinitialization_generation = Some(evidence.generation);
        }
        self.semantic_events = self
            .semantic_events
            .checked_add(1)
            .ok_or(ProcessError::ResourceLimit)?;
        if self
            .expected
            .estimated_event_count
            .is_some_and(|upper_bound| self.semantic_events > upper_bound)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        if let EvidenceEvent::End {
            record_sequence,
            end,
        } = event
        {
            self.validate_end(record_sequence, end)?;
            self.saw_end = true;
        }
        Ok(())
    }

    pub(super) fn validate_end(
        &self,
        record_sequence: u64,
        end: EvidenceEnd,
    ) -> Result<(), ProcessError> {
        let span = self.expected.span_capabilities;
        if record_sequence != end.terminal_record_sequence
            || record_sequence != span.terminal_record_sequence
            || end.span != span.span
            || end.source_logical_digest != self.expected.source_logical_digest
            || !span.has_valid_end
        {
            return Err(ProcessError::IncompleteEvidence);
        }
        Ok(())
    }

    pub(super) fn finish(self, trailing: Option<EvidenceEvent<'_>>) -> Result<u64, ProcessError> {
        if trailing.is_some() || !self.saw_end {
            return Err(ProcessError::IncompleteEvidence);
        }
        if self
            .control_head
            .is_some_and(|digest| digest != self.expected.configuration_digest)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        if self
            .expected
            .estimated_event_count
            .is_some_and(|upper_bound| self.semantic_events > upper_bound)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        Ok(self.semantic_events)
    }
}

pub(super) fn scan_source<S: EvidenceSource>(
    spec: &ProcessingSpec<'_>,
    expected: EvidenceManifest,
    source: &mut S,
    limits: OfflineResourceLimits,
    control: crate::config::RunControl<'_>,
) -> Result<ScanSummary, ProcessError> {
    ensure_source_manifest(spec, expected, source.manifest())?;
    source.restart()?;
    ensure_source_manifest(spec, expected, source.manifest())?;
    let mut validator = StreamValidator::new(expected);
    let mut semantic_seal = SemanticStreamSeal::new();
    let mut catalog = ConsiderCatalog::from_config(&spec.engine)?;
    if !catalog.covers_span(spec.span) {
        return Err(ProcessError::IncompleteEvidence);
    }
    let mut selected_records = 0_u64;
    while let Some(event) = source.next()? {
        semantic_seal.observe(event)?;
        validator.observe(event)?;
        if limits
            .elapsed_work_limit
            .is_some_and(|limit| validator.semantic_events > limit)
        {
            return Err(ProcessError::ResourceLimit);
        }
        if !(control.continue_running)(validator.semantic_events) {
            return Err(ProcessError::Cancelled);
        }
        match event {
            EvidenceEvent::Observation { observation, .. } => {
                validate_observation(*observation, &spec.engine)?;
                if !observation_within_span(*observation, expected.span_capabilities.span)? {
                    return Err(ProcessError::InvalidEvidence);
                }
                let (tasks, count) = tasks_for_observation(*observation)?;
                for task in tasks.into_iter().take(count).flatten() {
                    if task_is_selected(task, spec.span)? {
                        validate_selected_task_lineage(spec, task)?;
                        // A joint GNSS update may retain both an initialization
                        // and an updated state at the same epoch. Counting it
                        // as two is a conservative store preflight bound.
                        let records = if matches!(task.task, TimedTask::GnssJoint(_)) {
                            2
                        } else {
                            1
                        };
                        selected_records = selected_records
                            .checked_add(records)
                            .ok_or(ProcessError::ResourceLimit)?;
                    }
                }
            }
            EvidenceEvent::ClockModel { model, .. } => catalog.push_clock(model)?,
            EvidenceEvent::Gap { gap, .. } => {
                if !gap_is_valid_for_manifest(gap.span, expected.span_capabilities.span) {
                    return Err(ProcessError::InvalidEvidence);
                }
                if spans_intersect(gap.span, spec.span) {
                    selected_records = selected_records
                        .checked_add(1)
                        .ok_or(ProcessError::ResourceLimit)?;
                }
            }
            EvidenceEvent::Reinitialize { evidence, .. } => {
                if !expected.span_capabilities.span.contains(evidence.at) {
                    return Err(ProcessError::InvalidEvidence);
                }
                validate_reinitialization(spec, evidence)?;
                if spec.span.contains(evidence.at) {
                    selected_records = selected_records
                        .checked_add(1)
                        .ok_or(ProcessError::ResourceLimit)?;
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
    }
    let semantic_events = validator.finish(None)?;
    if !matrix_is_psd(&catalog.covariance) {
        return Err(ProcessError::InvalidEvidence);
    }
    // Every timed observation must resolve to one declared immutable clock
    // model.  This is checked during the forward pass after reordering, but an
    // empty catalog is already a decisive preflight failure.
    if catalog.clocks.is_empty() {
        return Err(ProcessError::IncompleteEvidence);
    }
    let maximum_records = selected_records
        .checked_add(16)
        .ok_or(ProcessError::ResourceLimit)?;
    Ok(ScanSummary {
        catalog,
        semantic_events,
        maximum_records,
        semantic_seal: semantic_seal.finish(),
    })
}

pub(super) fn ensure_source_manifest(
    spec: &ProcessingSpec<'_>,
    expected: EvidenceManifest,
    actual: EvidenceManifest,
) -> Result<(), ProcessError> {
    expected
        .validate()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    actual
        .validate()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    if !expected.restartable {
        return Err(ProcessError::CapabilityUnavailable);
    }
    if expected != actual
        || expected.configuration_digest != spec.engine.digest
        || !expected.span_capabilities.span.contains(spec.span.start())
        || !expected.span_capabilities.span.contains(spec.span.end())
        || !expected
            .span_capabilities
            .capabilities
            .contains(Capability::CompleteEnd)
    {
        return Err(ProcessError::InvalidEvidence);
    }
    Ok(())
}

pub(super) fn metric_result_upper_bound(spec: &ProcessingSpec<'_>) -> Result<u64, ProcessError> {
    let duration_ns = spec
        .span
        .end()
        .checked_duration_since(spec.span.start())
        .ok_or(ProcessError::InvalidEvidence)?
        .as_ns();
    let duration_ns = u64::try_from(duration_ns).map_err(|_| ProcessError::ResourceLimit)?;
    let mut bound = 0_u64;
    for definition in spec.metrics.definitions() {
        let definition_bound = match definition {
            MetricDefinition::Distance(_) => 1,
            MetricDefinition::Lap(plan) => {
                let occurrences = u64::from(plan.maximum_occurrences_per_gate());
                u64::try_from(plan.gates().len())
                    .ok()
                    .and_then(|gates| gates.checked_mul(occurrences))
                    .and_then(|crossings| crossings.checked_add(occurrences))
                    .ok_or(ProcessError::ResourceLimit)?
            }
            MetricDefinition::Drag(plan) => {
                u64::try_from(plan.targets().len()).map_err(|_| ProcessError::ResourceLimit)?
            }
            MetricDefinition::Activity(plan) => u64::try_from(plan.splits_m().len())
                .ok()
                .and_then(|splits| splits.checked_add(1))
                .ok_or(ProcessError::ResourceLimit)?,
            MetricDefinition::Ski(plan) => {
                let period_ns = plan.sample_period.as_ns();
                if period_ns == 0 {
                    return Err(ProcessError::InvalidEvidence);
                }
                duration_ns
                    .checked_div(period_ns)
                    .and_then(|samples| samples.checked_add(1))
                    // At most one segment can end at each sampled instant,
                    // followed by the mandatory ski summary.
                    .and_then(|segments| segments.checked_add(1))
                    .ok_or(ProcessError::ResourceLimit)?
            }
        };
        bound = bound
            .checked_add(definition_bound)
            .ok_or(ProcessError::ResourceLimit)?;
    }
    Ok(bound)
}

pub(super) fn validate_observation(
    observation: LiveObservation,
    config: &EngineConfig<'_>,
) -> Result<(), ProcessError> {
    match observation {
        LiveObservation::Imu(value) => {
            if value.profile() != config.input_profile.id
                || value.measurement_frame() != config.installation.imu_sensor_frame
            {
                return Err(ProcessError::InvalidEvidence);
            }
            value
                .angular_rate()
                .validate()
                .and_then(|_| value.specific_force().validate())
                .map_err(|_| ProcessError::InvalidEvidence)?;
            qualified_imu_support(value)?;
        }
        LiveObservation::GnssSolution(value) => {
            validate_antenna(value, config)?;
            value
                .diagnostics()
                .validate()
                .map_err(|_| ProcessError::InvalidEvidence)?;
            if let Some(position) = value.position() {
                position
                    .validate()
                    .map_err(|_| ProcessError::InvalidEvidence)?;
            }
            if let Some(velocity) = value.velocity() {
                velocity
                    .validate()
                    .map_err(|_| ProcessError::InvalidEvidence)?;
            }
        }
        LiveObservation::ClockTransition(value) => {
            value
                .validate()
                .map_err(|_| ProcessError::InvalidEvidence)?;
        }
    }
    Ok(())
}

pub(super) fn observation_within_span(
    observation: LiveObservation,
    span: TimeSpan,
) -> Result<bool, ProcessError> {
    let within = match observation {
        LiveObservation::Imu(value) => {
            let support = qualified_imu_support(value)?;
            span.contains(support.start) && span.contains(support.end)
        }
        LiveObservation::GnssSolution(value) => {
            let position = value
                .position()
                .map(|field| field.time.effective_time())
                .transpose()
                .map_err(|_| ProcessError::InvalidEvidence)?;
            let velocity = value
                .velocity()
                .map(|field| field.time.effective_time())
                .transpose()
                .map_err(|_| ProcessError::InvalidEvidence)?;
            position.is_none_or(|time| span.contains(time))
                && velocity.is_none_or(|time| span.contains(time))
        }
        LiveObservation::ClockTransition(value) => span.contains(value.at),
    };
    Ok(within)
}

#[derive(Clone, Copy)]
pub(super) enum TimedTask {
    Imu(ImuObservation),
    GnssPosition(GnssSolutionObservation),
    GnssVelocity(GnssSolutionObservation),
    GnssJoint(GnssSolutionObservation),
    ClockTransition(crate::observation::ClockTransitionObservation),
    Gap(crate::offline::ports::EvidenceGap),
    Reinitialize(crate::offline::ports::ReinitializationEvidence),
}

#[derive(Clone, Copy)]
pub(super) struct QueuedTask {
    pub(super) time: SessionTime,
    pub(super) class: u8,
    pub(super) source: u32,
    pub(super) observation_sequence: u64,
    pub(super) sub_sequence: u8,
    pub(super) task: TimedTask,
}

impl PartialEq for QueuedTask {
    fn eq(&self, other: &Self) -> bool {
        self.order_tuple() == other.order_tuple()
    }
}

impl Eq for QueuedTask {}

impl PartialOrd for QueuedTask {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueuedTask {
    fn cmp(&self, other: &Self) -> Ordering {
        self.order_tuple().cmp(&other.order_tuple())
    }
}

impl QueuedTask {
    pub(super) fn order_tuple(self) -> (SessionTime, u8, u32, u64, u8) {
        (
            self.time,
            self.class,
            self.source,
            self.observation_sequence,
            self.sub_sequence,
        )
    }
}

pub(super) fn tasks_for_observation(
    observation: LiveObservation,
) -> Result<([Option<QueuedTask>; 2], usize), ProcessError> {
    let id = observation.id();
    let source = id.source.get();
    let sequence = id.sequence;
    let mut tasks = [None, None];
    let count = match observation {
        LiveObservation::Imu(value) => {
            // Queue an interval when its support begins. This makes the
            // interval-average evidence available before independently timed
            // GNSS fields inside the interval are dispatched. The filter
            // closes the preceding interval transactionally before installing
            // this one and flushes the final interval at end of input.
            let time = qualified_imu_support(value)?.start;
            tasks[0] = Some(QueuedTask {
                time,
                class: TASK_CLASS_IMU,
                source,
                observation_sequence: sequence,
                sub_sequence: 0,
                task: TimedTask::Imu(value),
            });
            1
        }
        LiveObservation::GnssSolution(value) => match (value.position(), value.velocity()) {
            (Some(position), Some(velocity)) => {
                let position_time = position
                    .time
                    .effective_time()
                    .map_err(|_| ProcessError::InvalidEvidence)?;
                let velocity_time = velocity
                    .time
                    .effective_time()
                    .map_err(|_| ProcessError::InvalidEvidence)?;
                if position_time != velocity_time
                    && value.position_velocity_cross_covariance().is_some()
                {
                    // A supplied cross block couples two distinct epochs. It
                    // cannot be consumed by independent scalar-time updates
                    // without a correlated delayed-measurement model.
                    return Err(ProcessError::CapabilityUnavailable);
                }
                if position_time == velocity_time
                    && value.position_velocity_cross_covariance().is_some()
                {
                    tasks[0] = Some(QueuedTask {
                        time: position_time,
                        class: TASK_CLASS_GNSS_POSITION,
                        source,
                        observation_sequence: sequence,
                        sub_sequence: 0,
                        task: TimedTask::GnssJoint(value),
                    });
                    1
                } else {
                    tasks[0] = Some(QueuedTask {
                        time: position_time,
                        class: TASK_CLASS_GNSS_POSITION,
                        source,
                        observation_sequence: sequence,
                        sub_sequence: 0,
                        task: TimedTask::GnssPosition(value),
                    });
                    tasks[1] = Some(QueuedTask {
                        time: velocity_time,
                        class: TASK_CLASS_GNSS_VELOCITY,
                        source,
                        observation_sequence: sequence,
                        sub_sequence: 1,
                        task: TimedTask::GnssVelocity(value),
                    });
                    2
                }
            }
            (Some(position), None) => {
                tasks[0] = Some(QueuedTask {
                    time: position
                        .time
                        .effective_time()
                        .map_err(|_| ProcessError::InvalidEvidence)?,
                    class: TASK_CLASS_GNSS_POSITION,
                    source,
                    observation_sequence: sequence,
                    sub_sequence: 0,
                    task: TimedTask::GnssPosition(value),
                });
                1
            }
            (None, Some(velocity)) => {
                tasks[0] = Some(QueuedTask {
                    time: velocity
                        .time
                        .effective_time()
                        .map_err(|_| ProcessError::InvalidEvidence)?,
                    class: TASK_CLASS_GNSS_VELOCITY,
                    source,
                    observation_sequence: sequence,
                    sub_sequence: 0,
                    task: TimedTask::GnssVelocity(value),
                });
                1
            }
            (None, None) => return Err(ProcessError::InvalidEvidence),
        },
        LiveObservation::ClockTransition(value) => {
            tasks[0] = Some(QueuedTask {
                time: value.at,
                class: TASK_CLASS_CLOCK_TRANSITION,
                source,
                observation_sequence: sequence,
                sub_sequence: 0,
                task: TimedTask::ClockTransition(value),
            });
            1
        }
    };
    Ok((tasks, count))
}

pub(super) fn span_contains_span(outer: TimeSpan, inner: TimeSpan) -> bool {
    outer.contains(inner.start()) && outer.contains(inner.end())
}

pub(super) fn spans_intersect(left: TimeSpan, right: TimeSpan) -> bool {
    left.start() <= right.end() && right.start() <= left.end()
}

pub(super) fn gap_is_valid_for_manifest(gap: TimeSpan, manifest: TimeSpan) -> bool {
    gap.end() <= manifest.start() || span_contains_span(manifest, gap)
}

pub(super) fn task_support(task: QueuedTask) -> Result<(SessionTime, SessionTime), ProcessError> {
    match task.task {
        TimedTask::Imu(observation) => {
            let support = qualified_imu_support(observation)?;
            Ok((support.start, support.end))
        }
        TimedTask::GnssPosition(_)
        | TimedTask::GnssVelocity(_)
        | TimedTask::GnssJoint(_)
        | TimedTask::ClockTransition(_)
        | TimedTask::Gap(_)
        | TimedTask::Reinitialize(_) => Ok((task.time, task.time)),
    }
}

pub(super) fn task_frontier_time(task: QueuedTask) -> Result<SessionTime, ProcessError> {
    match task.task {
        TimedTask::Imu(observation) => Ok(qualified_imu_support(observation)?.end),
        TimedTask::GnssPosition(_)
        | TimedTask::GnssVelocity(_)
        | TimedTask::GnssJoint(_)
        | TimedTask::ClockTransition(_)
        | TimedTask::Gap(_)
        | TimedTask::Reinitialize(_) => Ok(task.time),
    }
}

pub(super) fn task_is_selected(task: QueuedTask, span: TimeSpan) -> Result<bool, ProcessError> {
    let (start, end) = task_support(task)?;
    Ok(span.contains(start) && span.contains(end))
}

pub(super) fn validate_selected_task_lineage(
    spec: &ProcessingSpec<'_>,
    task: QueuedTask,
) -> Result<(), ProcessError> {
    let class = match task.task {
        TimedTask::Imu(_) => EvidenceClass::Imu,
        TimedTask::GnssPosition(_) | TimedTask::GnssVelocity(_) | TimedTask::GnssJoint(_) => {
            EvidenceClass::GnssSolution
        }
        TimedTask::ClockTransition(_) => EvidenceClass::Timing,
        TimedTask::Gap(_) | TimedTask::Reinitialize(_) => return Ok(()),
    };
    let source = SourceId::new(task.source);
    let (start, end) = task_support(task)?;
    let selected = spec.evidence_lineage.selections().iter().any(|selection| {
        selection.source == source
            && selection.class == class
            && selection.usage == EvidenceUse::Fusion
            && matches!(
                selection.lineage,
                EvidenceLineageKind::Captured | EvidenceLineageKind::Recomputed
            )
            && selection.span.contains(start)
            && selection.span.contains(end)
    });
    if selected {
        Ok(())
    } else {
        Err(ProcessError::IncompleteEvidence)
    }
}

pub(super) fn validate_reinitialization(
    spec: &ProcessingSpec<'_>,
    evidence: crate::offline::ports::ReinitializationEvidence,
) -> Result<(), ProcessError> {
    if evidence.generation == 0
        || evidence.input_schema != CAPTURED_REINITIALIZATION_SCHEMA_V2
        || evidence.input_digest.is_zero()
        || evidence.input.navigation_profile_digest.is_zero()
        || evidence.input.metric_plan_digest.is_zero()
    {
        return Err(ProcessError::InvalidEvidence);
    }
    if evidence.configuration_digest != spec.engine.digest
        || evidence.input.navigation_profile_digest != spec.engine.navigation_profile.digest
    {
        return Err(ProcessError::IncompleteEvidence);
    }
    evidence
        .input
        .resources
        .validate_v2_mini()
        .map_err(|_| ProcessError::InvalidEvidence)?;
    evidence
        .input
        .initial_clock_prior
        .validate_with_shared(spec.engine.calibration.shared_parameters.covariance)
        .map_err(|_| ProcessError::InvalidEvidence)?;
    Ok(())
}

/// Validates and dispatches a typed captured transcript in canonical record
/// order. The engine consumer drives the same [`crate::LiveSession`] core used
/// on-device; this validator never substitutes the offline f64 smoother.
pub(crate) fn drive_captured_replay<S, F>(
    spec: &ProcessingSpec<'_>,
    expected_manifest: EvidenceManifest,
    source: &mut S,
    control: crate::config::RunControl<'_>,
    mut consume: F,
) -> Result<u64, ProcessError>
where
    S: EvidenceSource,
    F: for<'event> FnMut(EvidenceEvent<'event>) -> Result<(), ProcessError>,
{
    ensure_source_manifest(spec, expected_manifest, source.manifest())?;
    if !expected_manifest
        .span_capabilities
        .capabilities
        .contains(Capability::CapturedReplay)
    {
        return Err(ProcessError::IncompleteEvidence);
    }
    let contract = expected_manifest
        .captured_replay
        .ok_or(ProcessError::IncompleteEvidence)?;
    let expected_event_count = expected_manifest
        .estimated_event_count
        .ok_or(ProcessError::IncompleteEvidence)?;
    let expected_control_work = expected_event_count
        .checked_add(contract.maximum_total_work_units)
        .ok_or(ProcessError::ResourceLimit)?;
    source.restart()?;
    ensure_source_manifest(spec, expected_manifest, source.manifest())?;
    let mut validator = StreamValidator::new(expected_manifest);
    let mut completed_control_work = 0_u64;
    let mut captured_call_work = 0_u64;
    while let Some(event) = source.next()? {
        validator.observe(event)?;
        if let EvidenceEvent::Observation { observation, .. } = event {
            validate_observation(*observation, &spec.engine)?;
            if !observation_within_span(*observation, expected_manifest.span_capabilities.span)? {
                return Err(ProcessError::InvalidEvidence);
            }
        }
        let call_work = match event {
            EvidenceEvent::LiveStepCall { call, .. } => u64::from(call.work.units()),
            EvidenceEvent::LiveFinishCall { call, .. } => u64::from(call.work.units()),
            _ => 0,
        };
        captured_call_work = captured_call_work
            .checked_add(call_work)
            .ok_or(ProcessError::ResourceLimit)?;
        if captured_call_work > contract.maximum_total_work_units {
            return Err(ProcessError::ResourceLimit);
        }
        completed_control_work = completed_control_work
            .checked_add(1)
            .and_then(|work| work.checked_add(call_work))
            .ok_or(ProcessError::ResourceLimit)?;
        if completed_control_work > expected_control_work {
            return Err(ProcessError::ResourceLimit);
        }
        if !(control.continue_running)(completed_control_work) {
            return Err(ProcessError::Cancelled);
        }
        (control.progress)(completed_control_work, expected_control_work);
        consume(event)?;
    }
    let event_count = validator.finish(None)?;
    if event_count != expected_event_count
        || captured_call_work != contract.maximum_total_work_units
        || completed_control_work != expected_control_work
    {
        return Err(ProcessError::IncompleteEvidence);
    }
    Ok(event_count)
}
