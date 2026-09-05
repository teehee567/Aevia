//! Incremental launch, rollout, target, and stop-dwell state.

use super::{
    MAX_DRAG_TARGETS,
    definition::{DragPlan, DragTarget, Rollout, SpeedQuantity, TargetDirection},
    distance::{find_distance_target, integrate_trajectory_distance},
    events::find_launch,
    geometry::{scalar_speed, seconds_between},
    numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    quality::{metric_validity, retained_span_quality, worse_metric_validity},
    report::{DragTargetReport, MetricError, MetricResultValue},
    uncertainty::{MIN_EVENT_DERIVATIVE, TrajectoryMarginalUncertainty, speed_target_uncertainty},
};
use crate::{
    ids::ReferencePointId,
    quality::{EstimateQuality, EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{DurationNs, SessionTime, SignedDurationNs},
    trajectory::Trajectory,
};

#[derive(Clone, Debug, PartialEq)]
pub(super) struct LiveDragState {
    pub(super) launch: Option<SessionTime>,
    pub(super) rollout_time: Option<SessionTime>,
    pub(super) rollout_distance_m: f64,
    pub(super) target_distance_m: [f64; MAX_DRAG_TARGETS],
    pub(super) target_search_after: [Option<SessionTime>; MAX_DRAG_TARGETS],
    // Persist only irreducible event facts. Definition/reference/stage,
    // elapsed/rollout durations, and terminal-speed sigma are reconstructed
    // when emitting. Storing full public reports here multiplies their size by
    // every target and every fixed definition slot in the PSRAM workspace.
    pub(super) target_report: [Option<LiveDragTargetState>; MAX_DRAG_TARGETS],
    /// Quality accumulated from launch through the previously consumed
    /// trajectory suffix, before this update's scan interval.
    pub(super) accumulated_validity: Validity,
    pub(super) accumulated_gap: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct LiveDragTargetState {
    event_time: SessionTime,
    event_time_one_sigma_s: FieldValue<f64>,
    terminal_speed: Option<(SpeedQuantity, f64)>,
    terminal_speed_slope_mps2: FieldValue<f64>,
    stop_checked_through: SessionTime,
    stop_confirmed: bool,
    validity: Validity,
    gap_affected: bool,
}

impl LiveDragState {
    pub(super) const fn new() -> Self {
        Self {
            launch: None,
            rollout_time: None,
            rollout_distance_m: 0.0,
            target_distance_m: [0.0; MAX_DRAG_TARGETS],
            target_search_after: [None; MAX_DRAG_TARGETS],
            target_report: [None; MAX_DRAG_TARGETS],
            accumulated_validity: Validity::Nominal,
            accumulated_gap: false,
        }
    }
}

pub(super) fn copy_live_drag_state(target: &mut LiveDragState, source: &LiveDragState) {
    target.launch = source.launch;
    target.rollout_time = source.rollout_time;
    target.rollout_distance_m = source.rollout_distance_m;
    target
        .target_distance_m
        .copy_from_slice(&source.target_distance_m);
    target
        .target_search_after
        .copy_from_slice(&source.target_search_after);
    target.target_report.copy_from_slice(&source.target_report);
    target.accumulated_validity = source.accumulated_validity;
    target.accumulated_gap = source.accumulated_gap;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum StopDwellStatus {
    Pending(SessionTime),
    Confirmed,
    Rebounded(SessionTime),
}

fn stop_dwell_deadline(
    event_time: SessionTime,
    dwell: DurationNs,
) -> Result<SessionTime, MetricError> {
    let dwell_ns = i64::try_from(dwell.as_ns()).map_err(|_| MetricError::InvalidDefinition)?;
    event_time
        .checked_add(SignedDurationNs::from_ns(dwell_ns))
        .ok_or(MetricError::NumericalFailure)
}

#[allow(clippy::too_many_arguments)]
fn find_stop_rebound(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: SpeedQuantity,
    threshold_mps: f64,
    start: SessionTime,
    end: SessionTime,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SessionTime>, MetricError> {
    let start_sample = trajectory.scalar_kinematics_at(start, reference_point)?;
    let start_speed = scalar_speed(&start_sample, quantity).ok_or(MetricError::Unobservable)?;
    if start_speed > threshold_mps + limits.value_tolerance {
        return Err(MetricError::AmbiguousRoot);
    }

    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) = trajectory.segment_parameter_overlap(segment_index, start, end)
        else {
            continue;
        };
        let roots = trajectory.speed_roots_with_budget(
            segment_index,
            reference_point,
            quantity,
            threshold_mps,
            lower,
            upper,
            limits.absolute_root_tolerance_s,
            limits.value_tolerance,
            budget,
        )?;
        for parameter in roots {
            let sample = trajectory.scalar_kinematics_at_parameter(
                segment_index,
                parameter,
                reference_point,
            )?;
            if sample.time < start || sample.time > end {
                continue;
            }
            let slope = trajectory.speed_slope_at_parameter(
                segment_index,
                parameter,
                reference_point,
                quantity,
            )?;
            if slope.abs() <= MIN_EVENT_DERIVATIVE {
                // Do not infer whether an even/degenerate contact crossed the
                // stop boundary from endpoint samples.
                return Err(MetricError::AmbiguousRoot);
            }
            if slope > 0.0 {
                return Ok(Some(sample.time));
            }
        }
    }
    let end_sample = trajectory.scalar_kinematics_at(end, reference_point)?;
    let end_speed = scalar_speed(&end_sample, quantity).ok_or(MetricError::Unobservable)?;
    if end_speed > threshold_mps + limits.value_tolerance {
        // A continuous above-threshold endpoint without a resolved ascending
        // root makes the dwell topology unknowable.
        return Err(MetricError::AmbiguousRoot);
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_stop_dwell(
    trajectory: &Trajectory,
    plan: &DragPlan,
    quantity: SpeedQuantity,
    event_time: SessionTime,
    checked_through: SessionTime,
    scan_end: SessionTime,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<StopDwellStatus, MetricError> {
    let deadline = stop_dwell_deadline(event_time, plan.stop_dwell)?;
    if checked_through >= deadline {
        return Ok(StopDwellStatus::Confirmed);
    }
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    if checked_through < span.start() {
        // The below-threshold support rolled out before it was certified.
        return Err(MetricError::AmbiguousRoot);
    }
    let search_end = scan_end.min(deadline);
    if search_end < checked_through {
        return Ok(StopDwellStatus::Pending(checked_through));
    }
    let rebound = find_stop_rebound(
        trajectory,
        plan.reference_point,
        quantity,
        plan.stop_threshold_mps,
        checked_through,
        search_end,
        limits,
        budget,
    )?;
    if let Some(rebound) = rebound.filter(|rebound| *rebound < deadline) {
        return Ok(StopDwellStatus::Rebounded(rebound));
    }
    if scan_end >= deadline {
        Ok(StopDwellStatus::Confirmed)
    } else {
        Ok(StopDwellStatus::Pending(scan_end))
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_live_drag(
    trajectory: &Trajectory,
    plan: &DragPlan,
    state: &mut LiveDragState,
    scan_start: SessionTime,
    scan_end: SessionTime,
    include_terminal_endpoint: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
    emit: &mut impl FnMut(MetricResultValue) -> Result<(), MetricError>,
) -> Result<(), MetricError> {
    let mut uncertainty = TrajectoryMarginalUncertainty;
    if scan_end < scan_start {
        return Ok(());
    }
    let had_launch = state.launch.is_some();
    if state.launch.is_none() {
        state.launch = find_launch(
            trajectory,
            plan.reference_point,
            plan.launch,
            limits,
            budget,
        )?;
    }
    let Some(launch) = state.launch else {
        return Ok(());
    };
    let scan_from = if had_launch {
        scan_start.max(launch)
    } else {
        launch.max(
            trajectory
                .span()
                .ok_or(MetricError::EmptyTrajectory)?
                .start(),
        )
    };
    if scan_end < scan_from {
        return Ok(());
    }
    let validity_before_scan = state.accumulated_validity;
    let gap_before_scan = state.accumulated_gap;

    match plan.rollout {
        Rollout::None => state.rollout_time = Some(launch),
        Rollout::Distance { quantity, metres } if state.rollout_time.is_none() => {
            let remaining = (metres - state.rollout_distance_m).max(0.0);
            if let Some(time) = find_distance_target(
                trajectory,
                plan.reference_point,
                quantity,
                scan_from,
                remaining,
                limits,
                budget,
            )? {
                if time < scan_end || (include_terminal_endpoint && time == scan_end) {
                    state.rollout_time = Some(time);
                }
            }
            state.rollout_distance_m += integrate_trajectory_distance(
                trajectory,
                plan.reference_point,
                quantity,
                scan_from,
                scan_end,
                limits,
                budget,
            )?
            .value;
        }
        Rollout::Distance { .. } => {}
    }

    for (index, target) in plan.targets.iter().copied().enumerate() {
        let mut reopened_this_update = false;
        if let Some(mut present) = state.target_report[index] {
            let quality_checked_from = present.stop_checked_through;
            if let DragTarget::Speed {
                quantity,
                direction: TargetDirection::Descending,
                ..
            } = target
            {
                if !present.stop_confirmed {
                    match advance_stop_dwell(
                        trajectory,
                        plan,
                        quantity,
                        present.event_time,
                        present.stop_checked_through,
                        scan_end,
                        limits,
                        budget,
                    )? {
                        StopDwellStatus::Pending(checked_through) => {
                            present.stop_checked_through = checked_through;
                            state.target_report[index] = Some(present);
                        }
                        StopDwellStatus::Confirmed => {
                            present.stop_checked_through =
                                stop_dwell_deadline(present.event_time, plan.stop_dwell)?;
                            present.stop_confirmed = true;
                            state.target_report[index] = Some(present);
                        }
                        StopDwellStatus::Rebounded(rebound) => {
                            state.target_report[index] = None;
                            state.target_search_after[index] = rebound
                                .checked_add(SignedDurationNs::from_ns(1))
                                .ok_or(MetricError::NumericalFailure)
                                .map(Some)?;
                            reopened_this_update = true;
                        }
                    }
                }
            }
            if let Some(mut updated) = state.target_report[index] {
                let dwell_quality = retained_span_quality(
                    trajectory,
                    quality_checked_from,
                    updated.stop_checked_through,
                )
                .unwrap_or(EstimateQuality::INVALID);
                updated.validity =
                    worse_metric_validity(updated.validity, metric_validity(dwell_quality));
                updated.gap_affected |=
                    dwell_quality.imu_gap || matches!(dwell_quality.validity, Validity::Invalid);
                if updated.gap_affected {
                    updated.event_time_one_sigma_s =
                        FieldValue::Unavailable(UnavailableReason::IllConditioned);
                    updated.terminal_speed_slope_mps2 =
                        FieldValue::Unavailable(UnavailableReason::IllConditioned);
                }
                state.target_report[index] = Some(updated);
            }
            if let Some(present) = state.target_report[index] {
                emit(MetricResultValue::DragTarget(live_drag_report(
                    plan, state, index, present,
                )?))?;
                continue;
            }
        }

        let mut search_from = state.target_search_after[index]
            .unwrap_or(scan_from)
            .max(scan_from);
        loop {
            let event = match target {
                DragTarget::Speed {
                    quantity,
                    metres_per_second,
                    direction,
                    ..
                } => find_live_speed_target(
                    trajectory,
                    plan.reference_point,
                    quantity,
                    metres_per_second,
                    direction,
                    search_from,
                    scan_end,
                    include_terminal_endpoint,
                    limits,
                    budget,
                )?
                .map(|(time, speed, slope, segment, parameter)| {
                    (
                        time,
                        Some((quantity, speed)),
                        Some(slope),
                        Some((segment, parameter)),
                    )
                }),
                DragTarget::Distance {
                    quantity, metres, ..
                } => {
                    let remaining = (metres - state.target_distance_m[index]).max(0.0);
                    let event = find_distance_target(
                        trajectory,
                        plan.reference_point,
                        quantity,
                        scan_from,
                        remaining,
                        limits,
                        budget,
                    )?
                    .filter(|time| {
                        *time < scan_end || (include_terminal_endpoint && *time == scan_end)
                    });
                    state.target_distance_m[index] += integrate_trajectory_distance(
                        trajectory,
                        plan.reference_point,
                        quantity,
                        scan_from,
                        scan_end,
                        limits,
                        budget,
                    )?
                    .value;
                    event.map(|time| (time, None, None, None))
                }
            };
            let Some((event_time, terminal_speed, slope, event_location)) = event else {
                break;
            };
            let (mut event_time_one_sigma_s, _) = speed_target_uncertainty(
                trajectory,
                &mut uncertainty,
                plan.reference_point,
                terminal_speed,
                slope,
                event_location,
            );
            let event_quality = retained_span_quality(trajectory, scan_from, event_time)
                .unwrap_or(EstimateQuality::INVALID);
            let target_validity =
                worse_metric_validity(validity_before_scan, metric_validity(event_quality));
            let gap_affected = gap_before_scan
                || event_quality.imu_gap
                || matches!(event_quality.validity, Validity::Invalid);
            if gap_affected {
                event_time_one_sigma_s = FieldValue::Unavailable(UnavailableReason::IllConditioned);
            }
            let mut target_state = LiveDragTargetState {
                event_time,
                event_time_one_sigma_s,
                terminal_speed,
                terminal_speed_slope_mps2: if gap_affected {
                    FieldValue::Unavailable(UnavailableReason::IllConditioned)
                } else {
                    slope
                        .map(FieldValue::Available)
                        .unwrap_or(FieldValue::Unavailable(UnavailableReason::IllConditioned))
                },
                stop_checked_through: event_time,
                stop_confirmed: !matches!(
                    target,
                    DragTarget::Speed {
                        direction: TargetDirection::Descending,
                        ..
                    }
                ),
                validity: target_validity,
                gap_affected,
            };

            if let DragTarget::Speed {
                quantity,
                direction: TargetDirection::Descending,
                ..
            } = target
            {
                match advance_stop_dwell(
                    trajectory, plan, quantity, event_time, event_time, scan_end, limits, budget,
                )? {
                    StopDwellStatus::Pending(checked_through) => {
                        target_state.stop_checked_through = checked_through;
                    }
                    StopDwellStatus::Confirmed => {
                        target_state.stop_checked_through =
                            stop_dwell_deadline(event_time, plan.stop_dwell)?;
                        target_state.stop_confirmed = true;
                    }
                    StopDwellStatus::Rebounded(rebound) => {
                        search_from = rebound
                            .checked_add(SignedDurationNs::from_ns(1))
                            .ok_or(MetricError::NumericalFailure)?;
                        state.target_search_after[index] = Some(search_from);
                        if search_from > scan_end {
                            break;
                        }
                        continue;
                    }
                }
            }

            state.target_report[index] = Some(target_state);
            if !reopened_this_update {
                emit(MetricResultValue::DragTarget(live_drag_report(
                    plan,
                    state,
                    index,
                    target_state,
                )?))?;
            }
            break;
        }
    }
    let scanned_quality =
        retained_span_quality(trajectory, scan_from, scan_end).unwrap_or(EstimateQuality::INVALID);
    state.accumulated_validity =
        worse_metric_validity(state.accumulated_validity, metric_validity(scanned_quality));
    state.accumulated_gap |=
        scanned_quality.imu_gap || matches!(scanned_quality.validity, Validity::Invalid);
    Ok(())
}

pub(super) fn live_drag_report(
    plan: &DragPlan,
    state: &LiveDragState,
    target_index: usize,
    target: LiveDragTargetState,
) -> Result<DragTargetReport, MetricError> {
    let launch = state.launch.ok_or(MetricError::InvalidDefinition)?;
    let elapsed_seconds = seconds_between(launch, target.event_time)?;
    let terminal_speed_one_sigma_mps = target.terminal_speed.map(|(quantity, _)| {
        let sigma = match (
            target.event_time_one_sigma_s,
            target.terminal_speed_slope_mps2,
        ) {
            (FieldValue::Available(time_sigma), FieldValue::Available(slope)) => {
                FieldValue::Available(time_sigma * slope.abs())
            }
            (FieldValue::Unavailable(reason), _) | (_, FieldValue::Unavailable(reason)) => {
                FieldValue::Unavailable(reason)
            }
        };
        (quantity, sigma)
    });
    Ok(DragTargetReport {
        definition: plan.definition,
        target: plan
            .targets
            .get(target_index)
            .ok_or(MetricError::InvalidDefinition)?
            .id(),
        launch_time: launch,
        event_time: target.event_time,
        event_time_one_sigma_s: target.event_time_one_sigma_s,
        elapsed_seconds,
        elapsed_one_sigma_s: FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
        rollout_adjusted_seconds: rollout_adjusted_seconds(state.rollout_time, target.event_time),
        terminal_speed: target.terminal_speed,
        terminal_speed_one_sigma_mps,
        terminal_speed_slope_mps2: target.terminal_speed_slope_mps2,
        reference_point: plan.reference_point,
        stage: if target.stop_confirmed {
            EstimateStage::Finalized
        } else {
            EstimateStage::Provisional
        },
        validity: target.validity,
    })
}

pub(super) fn rollout_adjusted_seconds(
    rollout_time: Option<SessionTime>,
    target_time: SessionTime,
) -> FieldValue<f64> {
    let Some(rollout_time) = rollout_time else {
        return FieldValue::Unavailable(UnavailableReason::UnsupportedAtProcessingLevel);
    };
    if rollout_time > target_time {
        return FieldValue::Unavailable(UnavailableReason::OutsideQualifiedRange);
    }
    seconds_between(rollout_time, target_time).map_or(
        FieldValue::Unavailable(UnavailableReason::OutsideQualifiedRange),
        FieldValue::Available,
    )
}

#[allow(clippy::too_many_arguments)]
fn find_live_speed_target(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: SpeedQuantity,
    target_mps: f64,
    direction: TargetDirection,
    start: SessionTime,
    end: SessionTime,
    include_terminal_endpoint: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<(SessionTime, f64, f64, usize, f64)>, MetricError> {
    let last_segment = trajectory.segment_count().saturating_sub(1);
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) = trajectory.segment_parameter_overlap(segment_index, start, end)
        else {
            continue;
        };
        let owns_terminal = include_terminal_endpoint && segment_index == last_segment;
        if upper < lower || (upper == lower && !owns_terminal) {
            continue;
        }
        let roots = trajectory.speed_roots_with_budget(
            segment_index,
            reference_point,
            quantity,
            target_mps,
            lower,
            upper,
            limits.absolute_root_tolerance_s,
            limits.value_tolerance,
            budget,
        )?;
        for parameter in roots {
            if parameter < lower || parameter > upper || (parameter == upper && !owns_terminal) {
                continue;
            }
            let state = trajectory.scalar_kinematics_at_parameter(
                segment_index,
                parameter,
                reference_point,
            )?;
            let speed = scalar_speed(&state, quantity).ok_or(MetricError::Unobservable)?;
            let slope = trajectory.speed_slope_at_parameter(
                segment_index,
                parameter,
                reference_point,
                quantity,
            )?;
            let accepted = match direction {
                TargetDirection::Ascending => slope > 0.0,
                TargetDirection::Descending => slope < 0.0,
                TargetDirection::Either => slope != 0.0,
            };
            if accepted {
                return Ok(Some((state.time, speed, slope, segment_index, parameter)));
            }
        }
    }
    Ok(None)
}
