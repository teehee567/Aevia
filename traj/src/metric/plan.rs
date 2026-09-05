//! Metric-plan validation, binding, and live resource preflight.

#[cfg(test)]
use super::live_tracker::LiveMetricTracker;
use super::{
    MAX_METRIC_DEFINITIONS, MAX_METRIC_MUTATIONS_PER_STEP, MAX_METRIC_RESULTS,
    definition::{
        DistanceQuantity, DragTarget, LaunchRule, MetricDefinition, Rollout, SkiHmmModel,
        SpeedQuantity, TargetDirection,
    },
    geometry::all_finite,
};
#[cfg(any(test, feature = "offline"))]
use super::{
    evaluation::Evaluator,
    report::{
        MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason, MetricError, MetricResults,
    },
    uncertainty::{MetricUncertaintyProvider, TrajectoryMarginalUncertainty},
};
use crate::{
    config::AttachmentModel, error::ValidationError, frame::ReferencePoint, time::DurationNs,
};
#[cfg(any(test, feature = "offline"))]
use crate::{
    quality::{EstimateStage, Validity},
    trajectory::Trajectory,
};
use heapless::Vec as FixedVec;

/// Immutable semantic measurement plan.
#[derive(Clone, Debug, PartialEq)]
pub struct MetricPlan {
    pub(super) run_namespace: u64,
    pub(super) definitions: FixedVec<MetricDefinition, MAX_METRIC_DEFINITIONS>,
}

impl MetricPlan {
    #[must_use]
    pub const fn new(run_namespace: u64) -> Self {
        Self {
            run_namespace,
            definitions: FixedVec::new(),
        }
    }

    #[must_use]
    pub const fn run_namespace(&self) -> u64 {
        self.run_namespace
    }

    pub fn push(&mut self, definition: MetricDefinition) -> Result<(), ValidationError> {
        validate_definition(&definition)?;
        if self
            .definitions
            .iter()
            .any(|present| present.id() == definition.id())
        {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.definitions
            .push(definition)
            .map_err(|_| ValidationError::CapacityExceeded)
    }

    #[must_use]
    pub fn definitions(&self) -> &[MetricDefinition] {
        self.definitions.as_slice()
    }

    /// Validates all definition/reference bindings against an installation's
    /// physical attachment claim. Live construction uses this fail-closed
    /// form; host evaluation reports the same decision per definition.
    pub fn validate_attachment_bindings(
        &self,
        attachment: AttachmentModel,
        reference_points: &[ReferencePoint],
    ) -> Result<(), ValidationError> {
        for definition in &self.definitions {
            let point = reference_points
                .iter()
                .find(|point| point.id() == definition.reference_point())
                .ok_or(ValidationError::InvalidReferencePoint)?;
            if !definition.is_permitted_by_attachment(attachment, point.kind()) {
                return Err(ValidationError::IncompatibleDefinition);
            }
        }
        Ok(())
    }

    /// Compiles this plan into caller-owned storage for bounded live
    /// execution.
    ///
    /// The destination is reset before validation and remains an inactive
    /// placeholder if validation or capacity checking fails. This is the
    /// firmware construction path: no complete [`LiveMetricPlan`] is returned
    /// through the call frame.
    pub fn compile_live_into(
        &self,
        limits: LiveMetricLimits,
        output: &mut LiveMetricPlan,
    ) -> Result<(), ValidationError> {
        output.reset();
        if limits.max_mutations_per_step == 0
            || usize::from(limits.max_mutations_per_step) > MAX_METRIC_MUTATIONS_PER_STEP
            || limits.max_active_candidates == 0
            || limits.max_root_evaluations == 0
        {
            return Err(ValidationError::InvalidMetricDefinition);
        }

        let mut gate_count = 0usize;
        let mut target_count = 0usize;
        let mut maximum_lookahead = DurationNs::ZERO;
        let mut result_bound = 0_usize;
        let active_candidate_bound = live_active_candidate_bound(self)?;
        for definition in &self.definitions {
            validate_definition(definition)?;
            match definition {
                MetricDefinition::Lap(plan) => {
                    gate_count += plan.gates.len();
                    let occurrences = usize::from(plan.maximum_occurrences_per_gate);
                    result_bound = result_bound
                        .checked_add(
                            plan.gates
                                .len()
                                .checked_mul(occurrences)
                                .and_then(|crossings| crossings.checked_add(occurrences))
                                .ok_or(ValidationError::CapacityExceeded)?,
                        )
                        .ok_or(ValidationError::CapacityExceeded)?;
                }
                MetricDefinition::Drag(plan) => {
                    if matches!(
                        plan.rollout,
                        Rollout::Distance {
                            quantity: DistanceQuantity::BodyLongitudinalSigned,
                            ..
                        }
                    ) || plan.targets.iter().any(|target| {
                        matches!(
                            target,
                            DragTarget::Distance {
                                quantity: DistanceQuantity::BodyLongitudinalSigned,
                                ..
                            }
                        )
                    }) {
                        // Signed displacement can reverse. Live plans do not
                        // currently carry the bounded monotonicity proof needed
                        // to make one ordered target bracket sound.
                        return Err(ValidationError::IncompatibleDefinition);
                    }
                    target_count += plan.targets.len();
                    result_bound = result_bound
                        .checked_add(plan.targets.len())
                        .ok_or(ValidationError::CapacityExceeded)?;
                    maximum_lookahead = max_duration(
                        maximum_lookahead,
                        max_duration(launch_dwell(plan.launch), plan.stop_dwell),
                    );
                }
                MetricDefinition::Activity(plan) => {
                    // Elapsed/moving time, path totals, ordered splits, and
                    // instantaneous peak speed are supported live. Vertical
                    // ascent/descent remain explicitly unavailable because
                    // they require the host-only final activity classifier.
                    result_bound = result_bound
                        .checked_add(
                            1usize
                                .checked_add(plan.splits_m.len())
                                .ok_or(ValidationError::CapacityExceeded)?,
                        )
                        .ok_or(ValidationError::CapacityExceeded)?;
                }
                MetricDefinition::Distance(_) => {
                    result_bound = result_bound
                        .checked_add(1)
                        .ok_or(ValidationError::CapacityExceeded)?;
                }
                MetricDefinition::Ski(_) => {
                    return Err(ValidationError::IncompatibleDefinition);
                }
            }
        }
        if maximum_lookahead.as_ns() > i64::MAX as u64 {
            return Err(ValidationError::TimeOutOfRange);
        }
        if maximum_lookahead.as_ns() > crate::live::MAX_HISTORY_HORIZON_NS {
            return Err(ValidationError::CapacityExceeded);
        }
        let mutation_bound = result_bound
            .checked_mul(2)
            .ok_or(ValidationError::CapacityExceeded)?;
        if gate_count > limits.max_gates as usize
            || target_count > limits.max_targets as usize
            || result_bound > limits.max_results as usize
            || result_bound > MAX_METRIC_RESULTS
            || active_candidate_bound > usize::from(limits.max_active_candidates)
            || mutation_bound > usize::from(limits.max_mutations_per_step)
            || mutation_bound > MAX_METRIC_MUTATIONS_PER_STEP
        {
            return Err(ValidationError::CapacityExceeded);
        }

        output.plan.run_namespace = self.run_namespace;
        for definition in &self.definitions {
            if output.plan.definitions.push(definition.clone()).is_err() {
                output.reset();
                return Err(ValidationError::CapacityExceeded);
            }
        }
        output.maximum_lookahead = maximum_lookahead;
        output.limits = limits;
        Ok(())
    }

    /// Host/test convenience adapter over [`Self::compile_live_into`].
    /// Firmware should place a [`LiveMetricPlan::placeholder`] in its chosen
    /// memory region and compile into it directly.
    #[cfg(any(test, feature = "offline"))]
    pub fn compile_live(
        &self,
        limits: LiveMetricLimits,
    ) -> Result<LiveMetricPlan, ValidationError> {
        let mut output = LiveMetricPlan::placeholder();
        self.compile_live_into(limits, &mut output)?;
        Ok(output)
    }

    /// Host convenience evaluation against the engine-owned continuous path.
    ///
    /// In a offline build this adapter permits ordinary `Vec` growth. Code
    /// enforcing a preflighted memory envelope must instead prepare bounded
    /// [`MetricResults`] and use the internal bounded evaluator.
    #[cfg(any(test, feature = "offline"))]
    pub fn evaluate(&self, trajectory: &Trajectory) -> Result<MetricResults, MetricError> {
        let mut results = MetricResults::new();
        self.evaluate_into(trajectory, &mut results)?;
        Ok(results)
    }

    /// Evaluates every definition into caller-owned result storage.
    ///
    /// Offline replay can first call [`MetricResults::try_prepare_bounded`]
    /// so every subsequent emission is both allocation-free and checked
    /// against the preflighted result count.
    #[cfg(any(test, feature = "offline"))]
    pub(crate) fn evaluate_into(
        &self,
        trajectory: &Trajectory,
        results: &mut MetricResults,
    ) -> Result<(), MetricError> {
        let mut uncertainty = TrajectoryMarginalUncertainty;
        self.evaluate_with_uncertainty_into(trajectory, &mut uncertainty, results)
    }

    /// Private execution seam for a host smoother that still owns its
    /// state/consider and adjacent cross-covariance store. The provider is
    /// consumed while metrics scan forward, before that joint store is
    /// destroyed; callers cannot inject covariance through the public API.
    #[cfg(feature = "offline")]
    pub(crate) fn evaluate_with_uncertainty(
        &self,
        trajectory: &Trajectory,
        uncertainty: &mut dyn MetricUncertaintyProvider,
    ) -> Result<MetricResults, MetricError> {
        let mut results = MetricResults::new();
        self.evaluate_with_uncertainty_into(trajectory, uncertainty, &mut results)?;
        Ok(results)
    }

    #[cfg(any(test, feature = "offline"))]
    fn evaluate_with_uncertainty_into(
        &self,
        trajectory: &Trajectory,
        uncertainty: &mut dyn MetricUncertaintyProvider,
        results: &mut MetricResults,
    ) -> Result<(), MetricError> {
        results.clear();
        let evaluation = (|| {
            let mut evaluator =
                Evaluator::new(self.run_namespace, trajectory, uncertainty, results);
            for definition in &self.definitions {
                if validate_definition(definition).is_err() {
                    evaluator.record_diagnostic(MetricDefinitionDiagnostic {
                        definition: definition.id(),
                        reference_point: definition.reference_point(),
                        reason: MetricDefinitionDiagnosticReason::InvalidDefinition,
                        stage: EstimateStage::Finalized,
                        validity: Validity::Invalid,
                    })?;
                    continue;
                }
                if let Some(reason) = definition_binding_diagnostic(trajectory, definition) {
                    evaluator.record_diagnostic(MetricDefinitionDiagnostic {
                        definition: definition.id(),
                        reference_point: definition.reference_point(),
                        reason,
                        stage: EstimateStage::Finalized,
                        validity: Validity::Invalid,
                    })?;
                    continue;
                }
                let checkpoint = evaluator.checkpoint();
                let definition_result = match definition {
                    MetricDefinition::Distance(plan) => evaluator.distance(*plan),
                    MetricDefinition::Lap(plan) => evaluator.lap(plan),
                    MetricDefinition::Drag(plan) => evaluator.drag(plan),
                    MetricDefinition::Activity(plan) => evaluator.activity(plan),
                    MetricDefinition::Ski(plan) => evaluator.ski(*plan),
                };
                if let Err(error) = definition_result {
                    let Some(reason) = MetricDefinitionDiagnosticReason::from_error(error) else {
                        return Err(error);
                    };
                    // A definition can emit several values before discovering
                    // an ambiguous later event. Keep its publication atomic,
                    // but do not discard values already produced by unrelated
                    // definitions.
                    evaluator.rollback(checkpoint);
                    evaluator.record_diagnostic(MetricDefinitionDiagnostic {
                        definition: definition.id(),
                        reference_point: definition.reference_point(),
                        reason,
                        stage: EstimateStage::Finalized,
                        validity: Validity::Invalid,
                    })?;
                }
            }
            Ok(())
        })();
        if evaluation.is_err() {
            results.clear();
        }
        evaluation
    }
}

#[cfg(any(test, feature = "offline"))]
fn definition_binding_diagnostic(
    trajectory: &Trajectory,
    definition: &MetricDefinition,
) -> Option<MetricDefinitionDiagnosticReason> {
    let kind = match trajectory.configured_reference_point_kind(definition.reference_point()) {
        Ok(kind) => kind,
        Err(MetricError::ReferencePointUnavailable) => {
            return Some(MetricDefinitionDiagnosticReason::ReferencePointUnavailable);
        }
        Err(_) => return Some(MetricDefinitionDiagnosticReason::InvalidDefinition),
    };
    let attachment = trajectory.attachment_model();
    if !definition.is_permitted_by_attachment(attachment, kind) {
        return Some(MetricDefinitionDiagnosticReason::AttachmentModelUnavailable);
    }
    None
}

impl Default for MetricPlan {
    fn default() -> Self {
        Self::new(0)
    }
}

/// Fixed resource envelope used when compiling a live plan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LiveMetricLimits {
    pub max_gates: u16,
    pub max_targets: u16,
    pub max_results: u16,
    pub max_active_candidates: u8,
    pub max_mutations_per_step: u8,
    /// Total scalar numerical/root-oracle evaluations allowed by one live
    /// update, including its transactional state-commit replay. In the V1
    /// schema, each credit also grants one 15-point Gauss-Kronrod panel to
    /// the update's separate quadrature ledger.
    pub max_root_evaluations: u16,
}

impl Default for LiveMetricLimits {
    fn default() -> Self {
        Self {
            max_gates: 64,
            max_targets: 64,
            max_results: 64,
            max_active_candidates: 4,
            max_mutations_per_step: 16,
            max_root_evaluations: 256,
        }
    }
}

/// Validated bounded form consumed by the embedded evaluator.
#[derive(Clone, Debug, PartialEq)]
pub struct LiveMetricPlan {
    pub(super) plan: MetricPlan,
    pub(super) maximum_lookahead: DurationNs,
    pub(super) limits: LiveMetricLimits,
}

impl LiveMetricPlan {
    pub(super) const PLACEHOLDER_LIMITS: LiveMetricLimits = LiveMetricLimits {
        max_gates: 0,
        max_targets: 0,
        max_results: 0,
        max_active_candidates: 0,
        max_mutations_per_step: 0,
        max_root_evaluations: 0,
    };

    /// Valid inactive storage for caller-owned/static placement.
    ///
    /// Compile a metric plan into this value with
    /// [`MetricPlan::compile_live_into`] before starting a live session.
    #[must_use]
    pub const fn placeholder() -> Self {
        Self {
            plan: MetricPlan::new(0),
            maximum_lookahead: DurationNs::ZERO,
            limits: Self::PLACEHOLDER_LIMITS,
        }
    }

    pub(super) fn reset(&mut self) {
        self.plan.run_namespace = 0;
        self.plan.definitions.clear();
        self.maximum_lookahead = DurationNs::ZERO;
        self.limits = Self::PLACEHOLDER_LIMITS;
    }

    #[must_use]
    pub fn plan(&self) -> &MetricPlan {
        &self.plan
    }

    #[must_use]
    pub const fn maximum_lookahead(&self) -> DurationNs {
        self.maximum_lookahead
    }

    #[must_use]
    pub const fn limits(&self) -> LiveMetricLimits {
        self.limits
    }

    /// Starts a bounded mutation tracker for this compiled plan.
    #[must_use]
    #[cfg(test)]
    pub fn start(&self) -> LiveMetricTracker {
        let mut tracker = LiveMetricTracker::unconfigured();
        tracker
            .configure(self)
            .expect("compiled metric plan fits the fixed live tracker");
        tracker
    }
}

fn validate_definition(definition: &MetricDefinition) -> Result<(), ValidationError> {
    match definition {
        MetricDefinition::Distance(plan) => {
            if !plan.absolute_tolerance_m.is_finite()
                || !plan.relative_tolerance.is_finite()
                || plan.absolute_tolerance_m <= 0.0
                || plan.relative_tolerance < 0.0
            {
                return Err(ValidationError::InvalidMetricDefinition);
            }
        }
        MetricDefinition::Lap(plan) => {
            if plan.gates.is_empty() || plan.maximum_occurrences_per_gate == 0 {
                return Err(ValidationError::InvalidMetricDefinition);
            }
        }
        MetricDefinition::Drag(plan) => {
            if plan.targets.is_empty()
                || !plan.stop_threshold_mps.is_finite()
                || plan.stop_threshold_mps < 0.0
            {
                return Err(ValidationError::InvalidMetricDefinition);
            }
            validate_launch(plan.launch)?;
            match plan.rollout {
                Rollout::None => {}
                Rollout::Distance { metres, .. } if metres.is_finite() && metres >= 0.0 => {}
                Rollout::Distance { .. } => {
                    return Err(ValidationError::InvalidMetricDefinition);
                }
            }
            for target in &plan.targets {
                match *target {
                    DragTarget::Speed {
                        quantity,
                        metres_per_second,
                        direction,
                        ..
                    } if metres_per_second.is_finite()
                        && metres_per_second >= 0.0
                        && (!matches!(direction, TargetDirection::Descending)
                            || (metres_per_second <= plan.stop_threshold_mps
                                && !matches!(quantity, SpeedQuantity::BodyLongitudinalSigned))) => {
                    }
                    DragTarget::Distance { metres, .. } if metres.is_finite() && metres >= 0.0 => {}
                    _ => return Err(ValidationError::InvalidMetricDefinition),
                }
            }
        }
        MetricDefinition::Activity(plan) => {
            if !plan.moving_threshold_mps.is_finite()
                || plan.moving_threshold_mps < 0.0
                || plan.peak_window != DurationNs::ZERO
            {
                return Err(ValidationError::InvalidMetricDefinition);
            }
        }
        MetricDefinition::Ski(plan) => {
            if plan.sample_period == DurationNs::ZERO
                || !model_finite(&plan.model)
                || plan.minimum_segment_duration < plan.sample_period
            {
                return Err(ValidationError::InvalidMetricDefinition);
            }
        }
    }
    Ok(())
}

fn validate_launch(launch: LaunchRule) -> Result<(), ValidationError> {
    match launch {
        LaunchRule::FirstSustainedMotion { threshold_mps, .. }
        | LaunchRule::SpeedThreshold { threshold_mps, .. }
            if threshold_mps.is_finite() && threshold_mps >= 0.0 =>
        {
            Ok(())
        }
        LaunchRule::AccelerationChangePoint {
            minimum_acceleration_mps2,
            ..
        } if minimum_acceleration_mps2.is_finite() && minimum_acceleration_mps2 >= 0.0 => Ok(()),
        LaunchRule::ExternalTimestamp(_) => Ok(()),
        _ => Err(ValidationError::InvalidMetricDefinition),
    }
}

fn model_finite(model: &SkiHmmModel) -> bool {
    all_finite(&model.initial_log_probability)
        && model
            .transition_log_probability
            .iter()
            .all(|row| all_finite(row))
        && all_finite(&model.emission_bias)
        && model.emission_weight.iter().all(|row| all_finite(row))
}

pub(super) const fn launch_dwell(rule: LaunchRule) -> DurationNs {
    match rule {
        LaunchRule::FirstSustainedMotion { dwell, .. }
        | LaunchRule::SpeedThreshold { dwell, .. }
        | LaunchRule::AccelerationChangePoint { dwell, .. } => dwell,
        LaunchRule::ExternalTimestamp(_) => DurationNs::ZERO,
    }
}

pub(super) fn live_active_candidate_bound(plan: &MetricPlan) -> Result<usize, ValidationError> {
    let mut bound = 0_usize;
    for definition in &plan.definitions {
        let additional = match definition {
            MetricDefinition::Distance(_) => 0,
            // An ordered lap retains only its next expected gate candidate.
            MetricDefinition::Lap(_) => 1,
            // Every drag target may remain provisional at the same time.
            MetricDefinition::Drag(plan) => plan.targets.len(),
            MetricDefinition::Activity(_) => 0,
            // Final reclassification is host-only.
            MetricDefinition::Ski(_) => {
                return Err(ValidationError::IncompatibleDefinition);
            }
        };
        bound = bound
            .checked_add(additional)
            .ok_or(ValidationError::CapacityExceeded)?;
    }
    Ok(bound)
}

const fn max_duration(left: DurationNs, right: DurationNs) -> DurationNs {
    if left.as_ns() >= right.as_ns() {
        left
    } else {
        right
    }
}
