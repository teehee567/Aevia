//! Whole-trajectory metric evaluation and result emission.

#[cfg(any(test, feature = "offline"))]
use super::{
    activity::activity_extrema,
    definition::{
        ActivityPlan, DistancePlan, DistanceQuantity, DragPlan, DragTarget, LapPlan, Rollout,
        SkiPlan, TargetDirection,
    },
    distance::{find_distance_target, integrate_trajectory_distance},
    events::{find_launch, find_speed_target},
    geometry::seconds_between,
    live_drag::{StopDwellStatus, advance_stop_dwell, rollout_adjusted_seconds},
    live_lap::{LiveLapState, advance_lap},
    numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    quality::metric_validity,
    report::{
        ActivityReport, ActivitySplitReport, DistanceReport, DragTargetReport,
        MetricDefinitionDiagnostic,
    },
    uncertainty::speed_target_uncertainty,
};
#[cfg(feature = "offline")]
use super::{definition::SkiState, report::SkiReport, ski::ski_viterbi};
use super::{
    report::{MetricError, MetricResult, MetricResultValue, MetricResults},
    uncertainty::MetricUncertaintyProvider,
};
use crate::{ids::LiveResultId, trajectory::Trajectory};
#[cfg(any(test, feature = "offline"))]
use crate::{
    quality::{EstimateStage, FieldValue, UnavailableReason},
    time::SignedDurationNs,
};

pub(super) struct Evaluator<'a, 'results> {
    run_namespace: u64,
    next_result: u64,
    pub(super) trajectory: &'a Trajectory,
    #[cfg(any(test, feature = "offline"))]
    limits: MetricEvaluationLimits,
    #[cfg(any(test, feature = "offline"))]
    numerical_budget: NumericalWorkBudget,
    results: &'results mut MetricResults,
    #[cfg(any(test, feature = "offline"))]
    uncertainty: &'a mut dyn MetricUncertaintyProvider,
}

#[cfg(any(test, feature = "offline"))]
#[derive(Clone, Copy)]
pub(super) struct EvaluatorCheckpoint {
    next_result: u64,
    result_len: usize,
}

impl<'a, 'results> Evaluator<'a, 'results> {
    pub(super) fn new(
        run_namespace: u64,
        trajectory: &'a Trajectory,
        uncertainty: &'a mut dyn MetricUncertaintyProvider,
        results: &'results mut MetricResults,
    ) -> Self {
        let _ = &uncertainty;
        #[cfg(any(test, feature = "offline"))]
        let limits = MetricEvaluationLimits::default();
        Self {
            run_namespace,
            next_result: 0,
            trajectory,
            #[cfg(any(test, feature = "offline"))]
            limits,
            #[cfg(any(test, feature = "offline"))]
            numerical_budget: NumericalWorkBudget::from_limits(limits),
            results,
            #[cfg(any(test, feature = "offline"))]
            uncertainty,
        }
    }

    pub(super) fn emit(&mut self, value: MetricResultValue) -> Result<(), MetricError> {
        let id = LiveResultId::new(self.run_namespace, self.next_result);
        self.next_result = self
            .next_result
            .checked_add(1)
            .ok_or(MetricError::CapacityExceeded)?;
        self.results.push(MetricResult {
            id,
            revision: 0,
            value,
        })
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn checkpoint(&self) -> EvaluatorCheckpoint {
        EvaluatorCheckpoint {
            next_result: self.next_result,
            result_len: self.results.len(),
        }
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn rollback(&mut self, checkpoint: EvaluatorCheckpoint) {
        self.next_result = checkpoint.next_result;
        self.results.truncate_values(checkpoint.result_len);
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn record_diagnostic(
        &mut self,
        diagnostic: MetricDefinitionDiagnostic,
    ) -> Result<(), MetricError> {
        self.emit(MetricResultValue::Unavailable(diagnostic))
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn distance(&mut self, plan: DistancePlan) -> Result<(), MetricError> {
        let span = self.trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let integral = integrate_trajectory_distance(
            self.trajectory,
            plan.reference_point,
            plan.quantity,
            span.start(),
            span.end(),
            MetricEvaluationLimits {
                absolute_integration_tolerance: plan.absolute_tolerance_m,
                relative_integration_tolerance: plan.relative_tolerance,
                ..self.limits
            },
            &mut self.numerical_budget,
        )?;
        let uncertainty_one_sigma_m =
            self.uncertainty
                .integrated_distance_one_sigma_m(self.trajectory, plan, span);
        let validity = metric_validity(
            self.trajectory
                .conservative_quality_over_span(span.start(), span.end())?,
        );
        self.emit(MetricResultValue::Distance(DistanceReport {
            definition: plan.definition,
            quantity: plan.quantity,
            reference_point: plan.reference_point,
            span,
            metres: integral.value,
            numerical_error_m: integral.absolute_error,
            uncertainty_one_sigma_m,
            stage: EstimateStage::Finalized,
            validity,
        }))
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn lap(&mut self, plan: &LapPlan) -> Result<(), MetricError> {
        let span = self.trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let mut state = LiveLapState::new();
        let trajectory = self.trajectory;
        let run_namespace = self.run_namespace;
        let next_result = &mut self.next_result;
        let results = &mut *self.results;
        advance_lap(
            trajectory,
            plan,
            &mut state,
            span.start(),
            span.end(),
            true,
            self.limits,
            &mut self.numerical_budget,
            &mut *self.uncertainty,
            &mut |value| {
                let id = LiveResultId::new(run_namespace, *next_result);
                *next_result = next_result
                    .checked_add(1)
                    .ok_or(MetricError::CapacityExceeded)?;
                results.push(MetricResult {
                    id,
                    revision: 0,
                    value,
                })
            },
        )
    }
    #[cfg(any(test, feature = "offline"))]
    pub(super) fn drag(&mut self, plan: &DragPlan) -> Result<(), MetricError> {
        let span = self.trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let Some(launch) = find_launch(
            self.trajectory,
            plan.reference_point,
            plan.launch,
            self.limits,
            &mut self.numerical_budget,
        )?
        else {
            // A launch that has not occurred (or has rolled out of a live
            // trajectory) means this definition currently has no candidates;
            // it is not a failure of unrelated metric evaluation.
            return Ok(());
        };
        let rollout_time = match plan.rollout {
            Rollout::None => Some(launch),
            Rollout::Distance { quantity, metres } => find_distance_target(
                self.trajectory,
                plan.reference_point,
                quantity,
                launch,
                metres,
                self.limits,
                &mut self.numerical_budget,
            )?,
        };

        for target in plan.targets.iter().copied() {
            let mut search_after = launch;
            loop {
                let event = match target {
                    DragTarget::Speed {
                        quantity,
                        metres_per_second,
                        direction,
                        ..
                    } => find_speed_target(
                        self.trajectory,
                        plan.reference_point,
                        quantity,
                        search_after,
                        metres_per_second,
                        direction,
                        self.limits,
                        &mut self.numerical_budget,
                    )?,
                    DragTarget::Distance {
                        quantity, metres, ..
                    } => find_distance_target(
                        self.trajectory,
                        plan.reference_point,
                        quantity,
                        launch,
                        metres,
                        self.limits,
                        &mut self.numerical_budget,
                    )?
                    .map(|time| (time, None, None, None)),
                };
                let Some((event_time, terminal_speed, slope, event_location)) = event else {
                    break;
                };
                let stage = if let DragTarget::Speed {
                    quantity,
                    direction: TargetDirection::Descending,
                    ..
                } = target
                {
                    match advance_stop_dwell(
                        self.trajectory,
                        plan,
                        quantity,
                        event_time,
                        event_time,
                        span.end(),
                        self.limits,
                        &mut self.numerical_budget,
                    )? {
                        StopDwellStatus::Pending(_) => EstimateStage::Provisional,
                        StopDwellStatus::Confirmed => EstimateStage::Finalized,
                        StopDwellStatus::Rebounded(rebound) => {
                            search_after = rebound
                                .checked_add(SignedDurationNs::from_ns(1))
                                .ok_or(MetricError::NumericalFailure)?;
                            if search_after > span.end() {
                                break;
                            }
                            continue;
                        }
                    }
                } else {
                    EstimateStage::Finalized
                };
                let elapsed_seconds = seconds_between(launch, event_time)?;
                let rollout_adjusted_seconds = rollout_adjusted_seconds(rollout_time, event_time);
                let (event_time_one_sigma_s, terminal_speed_one_sigma_mps) =
                    speed_target_uncertainty(
                        self.trajectory,
                        self.uncertainty,
                        plan.reference_point,
                        terminal_speed,
                        slope,
                        event_location,
                    );
                let event_span_quality = self
                    .trajectory
                    .conservative_quality_over_span(launch, event_time)?;
                let gap_affected = event_span_quality.imu_gap;
                let event_time_one_sigma_s = if gap_affected {
                    FieldValue::Unavailable(UnavailableReason::IllConditioned)
                } else {
                    event_time_one_sigma_s
                };
                let elapsed_one_sigma_s = if gap_affected {
                    FieldValue::Unavailable(UnavailableReason::IllConditioned)
                } else {
                    FieldValue::Unavailable(UnavailableReason::MissingCorrelation)
                };
                self.emit(MetricResultValue::DragTarget(DragTargetReport {
                    definition: plan.definition,
                    target: target.id(),
                    launch_time: launch,
                    event_time,
                    event_time_one_sigma_s,
                    elapsed_seconds,
                    elapsed_one_sigma_s,
                    rollout_adjusted_seconds,
                    terminal_speed,
                    terminal_speed_one_sigma_mps,
                    terminal_speed_slope_mps2: slope
                        .map(FieldValue::Available)
                        .unwrap_or(FieldValue::Unavailable(UnavailableReason::IllConditioned)),
                    reference_point: plan.reference_point,
                    stage,
                    validity: metric_validity(event_span_quality),
                }))?;
                break;
            }
        }
        Ok(())
    }

    #[cfg(any(test, feature = "offline"))]
    pub(super) fn activity(&mut self, plan: &ActivityPlan) -> Result<(), MetricError> {
        let span = self.trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let elapsed_seconds = seconds_between(span.start(), span.end())?;
        let validity = metric_validity(
            self.trajectory
                .conservative_quality_over_span(span.start(), span.end())?,
        );
        let horizontal = if plan.include_horizontal_distance {
            FieldValue::Available(
                integrate_trajectory_distance(
                    self.trajectory,
                    plan.reference_point,
                    DistanceQuantity::HorizontalPath,
                    span.start(),
                    span.end(),
                    self.limits,
                    &mut self.numerical_budget,
                )?
                .value,
            )
        } else {
            FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
        };
        let spatial = if plan.include_spatial_distance {
            FieldValue::Available(
                integrate_trajectory_distance(
                    self.trajectory,
                    plan.reference_point,
                    DistanceQuantity::Spatial3d,
                    span.start(),
                    span.end(),
                    self.limits,
                    &mut self.numerical_budget,
                )?
                .value,
            )
        } else {
            FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
        };
        let (moving_seconds, peak_speed, ascent, descent) = activity_extrema(
            self.trajectory,
            plan.reference_point,
            plan.moving_speed,
            plan.moving_threshold_mps,
            plan.peak_speed,
            span.start(),
            span.end(),
            true,
            self.limits,
            &mut self.numerical_budget,
        )?;
        self.emit(MetricResultValue::Activity(ActivityReport {
            definition: plan.definition,
            reference_point: plan.reference_point,
            span,
            elapsed_seconds,
            moving_seconds,
            horizontal_distance_m: horizontal,
            spatial_distance_m: spatial,
            ascent_m: FieldValue::Available(ascent),
            descent_m: FieldValue::Available(descent),
            peak_speed: plan.peak_speed,
            peak_speed_mps: peak_speed,
            peak_window: plan.peak_window,
            stage: EstimateStage::Finalized,
            validity,
        }))?;
        for (index, split) in plan.splits_m.iter().enumerate() {
            let Some(time) = find_distance_target(
                self.trajectory,
                plan.reference_point,
                DistanceQuantity::HorizontalPath,
                span.start(),
                *split,
                self.limits,
                &mut self.numerical_budget,
            )?
            else {
                continue;
            };
            let split_validity = metric_validity(
                self.trajectory
                    .conservative_quality_over_span(span.start(), time)?,
            );
            self.emit(MetricResultValue::ActivitySplit(ActivitySplitReport {
                definition: plan.definition,
                split_index: u16::try_from(index).map_err(|_| MetricError::CapacityExceeded)?,
                horizontal_distance_m: *split,
                time,
                elapsed_seconds: seconds_between(span.start(), time)?,
                reference_point: plan.reference_point,
                stage: EstimateStage::Finalized,
                validity: split_validity,
            }))?;
        }
        Ok(())
    }

    #[cfg(feature = "offline")]
    pub(super) fn ski(&mut self, plan: SkiPlan) -> Result<(), MetricError> {
        let segments = ski_viterbi(self.trajectory, plan)?;
        let span = self.trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
        let summary_validity = metric_validity(
            self.trajectory
                .conservative_quality_over_span(span.start(), span.end())?,
        );
        let mut summary = SkiReport {
            definition: plan.definition,
            downhill_segments: 0,
            lift_segments: 0,
            ascent_segments: 0,
            downhill_seconds: 0.0,
            reference_point: plan.reference_point,
            stage: EstimateStage::Finalized,
            validity: summary_validity,
        };
        for segment in segments {
            match segment.state {
                SkiState::Downhill => {
                    summary.downhill_segments += 1;
                    summary.downhill_seconds += seconds_between(segment.start, segment.end)?;
                }
                SkiState::Lift => summary.lift_segments += 1,
                SkiState::Ascent => summary.ascent_segments += 1,
                SkiState::Stationary | SkiState::Other => {}
            }
            self.emit(MetricResultValue::SkiSegment(segment))?;
        }
        self.emit(MetricResultValue::Ski(summary))
    }

    #[cfg(all(test, not(feature = "offline")))]
    pub(super) fn ski(&mut self, _plan: SkiPlan) -> Result<(), MetricError> {
        Err(MetricError::Unsupported)
    }
}
