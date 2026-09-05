//! Incremental activity totals and ordered distance splits.

use super::{
    MAX_ACTIVITY_SPLITS,
    activity::activity_extrema,
    definition::{ActivityPlan, DistanceQuantity},
    distance::{find_distance_target, integrate_trajectory_distance},
    geometry::seconds_between,
    numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    quality::{metric_validity, worse_metric_validity},
    report::{ActivityReport, ActivitySplitReport, MetricError, MetricResultValue},
};
use crate::{
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{SessionTime, TimeSpan},
    trajectory::Trajectory,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct LiveActivityState {
    pub(super) horizontal_distance_m: f64,
    pub(super) split_emitted: [bool; MAX_ACTIVITY_SPLITS],
}

impl LiveActivityState {
    pub(super) const fn new() -> Self {
        Self {
            horizontal_distance_m: 0.0,
            split_emitted: [false; MAX_ACTIVITY_SPLITS],
        }
    }
}

pub(super) fn copy_live_activity_state(target: &mut LiveActivityState, source: &LiveActivityState) {
    target.horizontal_distance_m = source.horizontal_distance_m;
    target.split_emitted.copy_from_slice(&source.split_emitted);
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_live_activity(
    trajectory: &Trajectory,
    plan: &ActivityPlan,
    previous: Option<ActivityReport>,
    state: &mut LiveActivityState,
    scan_start: SessionTime,
    scan_end: SessionTime,
    scan_available: bool,
    include_terminal_endpoint: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
    emit: &mut impl FnMut(MetricResultValue) -> Result<(), MetricError>,
) -> Result<(), MetricError> {
    if !scan_available {
        let report = previous.ok_or(MetricError::EmptyTrajectory)?;
        return emit(MetricResultValue::Activity(report));
    }
    let trajectory_span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    let has_duration = scan_end > scan_start;
    let before_horizontal = state.horizontal_distance_m;
    let horizontal_delta =
        if has_duration && (plan.include_horizontal_distance || !plan.splits_m.is_empty()) {
            integrate_trajectory_distance(
                trajectory,
                plan.reference_point,
                DistanceQuantity::HorizontalPath,
                scan_start,
                scan_end,
                limits,
                budget,
            )?
            .value
        } else {
            0.0
        };
    state.horizontal_distance_m += horizontal_delta;
    let spatial_delta = if has_duration && plan.include_spatial_distance {
        integrate_trajectory_distance(
            trajectory,
            plan.reference_point,
            DistanceQuantity::Spatial3d,
            scan_start,
            scan_end,
            limits,
            budget,
        )?
        .value
    } else {
        0.0
    };
    let (moving_delta, peak_delta, _, _) = if has_duration {
        activity_extrema(
            trajectory,
            plan.reference_point,
            plan.moving_speed,
            plan.moving_threshold_mps,
            plan.peak_speed,
            scan_start,
            scan_end,
            false,
            limits,
            budget,
        )?
    } else {
        (0.0, 0.0, 0.0, 0.0)
    };
    let start = previous.map_or(trajectory_span.start(), |report| report.span.start());
    let suffix_validity = trajectory
        .conservative_quality_over_span(scan_start, scan_end)
        .map(metric_validity)
        .unwrap_or(Validity::Invalid);
    let validity = previous.map_or(suffix_validity, |report| {
        worse_metric_validity(report.validity, suffix_validity)
    });
    let report = ActivityReport {
        definition: plan.definition,
        reference_point: plan.reference_point,
        span: TimeSpan::new(start, scan_end).map_err(|_| MetricError::NumericalFailure)?,
        elapsed_seconds: seconds_between(start, scan_end)?,
        moving_seconds: previous.map_or(0.0, |report| report.moving_seconds) + moving_delta,
        horizontal_distance_m: if plan.include_horizontal_distance {
            FieldValue::Available(
                previous
                    .and_then(|report| match report.horizontal_distance_m {
                        FieldValue::Available(value) => Some(value),
                        FieldValue::Unavailable(_) => None,
                    })
                    .unwrap_or(0.0)
                    + horizontal_delta,
            )
        } else {
            FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
        },
        spatial_distance_m: if plan.include_spatial_distance {
            FieldValue::Available(
                previous
                    .and_then(|report| match report.spatial_distance_m {
                        FieldValue::Available(value) => Some(value),
                        FieldValue::Unavailable(_) => None,
                    })
                    .unwrap_or(0.0)
                    + spatial_delta,
            )
        } else {
            FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel)
        },
        ascent_m: FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel),
        descent_m: FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel),
        peak_speed: plan.peak_speed,
        peak_speed_mps: previous.map_or(peak_delta, |report| report.peak_speed_mps.max(peak_delta)),
        peak_window: plan.peak_window,
        stage: EstimateStage::Finalized,
        validity,
    };
    emit(MetricResultValue::Activity(report))?;

    for (index, split_metres) in plan.splits_m.iter().copied().enumerate() {
        if state.split_emitted[index]
            || split_metres < before_horizontal
            || split_metres > state.horizontal_distance_m
        {
            continue;
        }
        let remaining = (split_metres - before_horizontal).max(0.0);
        let Some(time) = find_distance_target(
            trajectory,
            plan.reference_point,
            DistanceQuantity::HorizontalPath,
            scan_start,
            remaining,
            limits,
            budget,
        )?
        else {
            continue;
        };
        if time > scan_end || (time == scan_end && !include_terminal_endpoint) {
            continue;
        }
        let split_validity = trajectory
            .scalar_kinematics_at(time, plan.reference_point)
            .map(|sample| metric_validity(sample.quality))
            .unwrap_or(Validity::Invalid);
        emit(MetricResultValue::ActivitySplit(ActivitySplitReport {
            definition: plan.definition,
            split_index: u16::try_from(index).map_err(|_| MetricError::CapacityExceeded)?,
            horizontal_distance_m: split_metres,
            time,
            elapsed_seconds: seconds_between(start, time)?,
            reference_point: plan.reference_point,
            stage: EstimateStage::Finalized,
            validity: worse_metric_validity(validity, split_validity),
        }))?;
        state.split_emitted[index] = true;
    }
    Ok(())
}
