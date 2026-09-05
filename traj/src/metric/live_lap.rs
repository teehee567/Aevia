//! Ordered gate crossings, rearming, and lap accumulation.

use super::{
    MAX_LAP_GATES,
    definition::{FiniteGate, LapPlan},
    geometry::{add, dot, scalar_speed, scale, seconds_between, sub},
    numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    quality::{metric_validity, worse_metric_validity},
    report::{GateCrossingReport, LapReport, MetricError, MetricResultValue},
    uncertainty::{
        EventTimeSensitivity, MetricUncertaintyProvider, TrajectoryMarginalUncertainty,
        event_time_one_sigma, gate_crossing_speed_one_sigma, gate_event_sensitivity,
        lap_elapsed_one_sigma,
    },
};
use crate::{
    ids::ReferencePointId,
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::SessionTime,
    trajectory::Trajectory,
};

#[derive(Clone, Debug, PartialEq)]
pub(super) struct LiveLapState {
    pub(super) expected_gate: usize,
    pub(super) lap_start: Option<GateCrossingReport>,
    pub(super) lap_start_sensitivity: Option<EventTimeSensitivity>,
    pub(super) accumulated_validity: Validity,
    pub(super) accumulated_gap: bool,
    pub(super) quality_checked_through: Option<SessionTime>,
    pub(super) last_gate_time: [Option<SessionTime>; MAX_LAP_GATES],
    pub(super) gate_occurrence: [u32; MAX_LAP_GATES],
    pub(super) gate_armed: [bool; MAX_LAP_GATES],
    pub(super) gate_rearm_time: [Option<SessionTime>; MAX_LAP_GATES],
    pub(super) gate_rearm_checked_through: [Option<SessionTime>; MAX_LAP_GATES],
    pub(super) lap_index: u32,
}

impl LiveLapState {
    pub(super) const fn new() -> Self {
        Self {
            expected_gate: 0,
            lap_start: None,
            lap_start_sensitivity: None,
            accumulated_validity: Validity::Nominal,
            accumulated_gap: false,
            quality_checked_through: None,
            last_gate_time: [None; MAX_LAP_GATES],
            gate_occurrence: [0; MAX_LAP_GATES],
            gate_armed: [true; MAX_LAP_GATES],
            gate_rearm_time: [None; MAX_LAP_GATES],
            gate_rearm_checked_through: [None; MAX_LAP_GATES],
            lap_index: 0,
        }
    }
}

pub(super) fn copy_live_lap_state(target: &mut LiveLapState, source: &LiveLapState) {
    target.expected_gate = source.expected_gate;
    target.lap_start = source.lap_start;
    target.lap_start_sensitivity = source.lap_start_sensitivity;
    target.accumulated_validity = source.accumulated_validity;
    target.accumulated_gap = source.accumulated_gap;
    target.quality_checked_through = source.quality_checked_through;
    target
        .last_gate_time
        .copy_from_slice(&source.last_gate_time);
    target
        .gate_occurrence
        .copy_from_slice(&source.gate_occurrence);
    target.gate_armed.copy_from_slice(&source.gate_armed);
    target
        .gate_rearm_time
        .copy_from_slice(&source.gate_rearm_time);
    target
        .gate_rearm_checked_through
        .copy_from_slice(&source.gate_rearm_checked_through);
    target.lap_index = source.lap_index;
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_live_lap(
    trajectory: &Trajectory,
    plan: &LapPlan,
    state: &mut LiveLapState,
    scan_start: SessionTime,
    scan_end: SessionTime,
    include_terminal_endpoint: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
    emit: &mut impl FnMut(MetricResultValue) -> Result<(), MetricError>,
) -> Result<(), MetricError> {
    let mut uncertainty = TrajectoryMarginalUncertainty;
    advance_lap(
        trajectory,
        plan,
        state,
        scan_start,
        scan_end,
        include_terminal_endpoint,
        limits,
        budget,
        &mut uncertainty,
        emit,
    )
}

fn find_gate_rearm_time(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    gate: FiniteGate,
    search_start: SessionTime,
    search_end: SessionTime,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SessionTime>, MetricError> {
    if gate.rearm_distance_m == 0.0 {
        return Ok(Some(search_start));
    }
    if search_end < search_start {
        return Ok(None);
    }
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    if search_start < span.start() {
        // The trajectory no longer contains the interval on which rearming
        // had to be proved. An endpoint outside the band is not a substitute
        // for resolving the intervening topology.
        return Err(MetricError::AmbiguousRoot);
    }

    let positive_center = add(
        gate.center_ecef_m,
        scale(gate.normal_ecef, gate.rearm_distance_m),
    );
    let negative_center = sub(
        gate.center_ecef_m,
        scale(gate.normal_ecef, gate.rearm_distance_m),
    );
    let mut earliest = None;
    for segment_index in 0..trajectory.segment_count() {
        if trajectory
            .segment_parameter_overlap(segment_index, search_start, search_end)
            .is_none()
        {
            continue;
        }
        for center in [positive_center, negative_center] {
            let roots = trajectory.gate_roots_with_budget(
                segment_index,
                reference_point,
                center,
                gate.normal_ecef,
                limits.absolute_root_tolerance_s,
                limits.value_tolerance,
                budget,
            )?;
            for parameter in roots {
                let time = trajectory.time_at_parameter(segment_index, parameter)?;
                if time < search_start || time > search_end {
                    continue;
                }
                if earliest.is_none_or(|present| time < present) {
                    earliest = Some(time);
                }
            }
        }
    }
    Ok(earliest)
}

fn accumulate_live_lap_quality(
    trajectory: &Trajectory,
    state: &mut LiveLapState,
    through: SessionTime,
) -> Result<(), MetricError> {
    let Some(start) = state.lap_start else {
        return Ok(());
    };
    let checked_through = state.quality_checked_through.unwrap_or(start.time);
    if through < checked_through {
        return Err(MetricError::InvalidDefinition);
    }
    if through == checked_through {
        return Ok(());
    }
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    if checked_through < span.start() {
        // The live tracker must consume quality before its source segment is
        // evicted. If that invariant is ever broken, do not silently recover
        // a nominal lap from only the retained suffix.
        state.accumulated_validity = Validity::Invalid;
        state.accumulated_gap = true;
    }
    let quality =
        trajectory.conservative_quality_over_span(checked_through.max(span.start()), through)?;
    state.accumulated_validity =
        worse_metric_validity(state.accumulated_validity, metric_validity(quality));
    state.accumulated_gap |= quality.imu_gap;
    state.quality_checked_through = Some(through);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(super) fn advance_lap(
    trajectory: &Trajectory,
    plan: &LapPlan,
    state: &mut LiveLapState,
    scan_start: SessionTime,
    scan_end: SessionTime,
    include_terminal_endpoint: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
    uncertainty: &mut dyn MetricUncertaintyProvider,
    emit: &mut impl FnMut(MetricResultValue) -> Result<(), MetricError>,
) -> Result<(), MetricError> {
    if plan
        .gates
        .iter()
        .any(|gate| gate.frame != trajectory.frame().id())
    {
        return Err(MetricError::FrameMismatch);
    }
    if plan.gates.is_empty() || scan_end < scan_start {
        return Ok(());
    }

    // Prove rearming for every disarmed gate while its trajectory suffix is
    // still resident. A gate may not become expected again until much later.
    for gate_index in 0..plan.gates.len() {
        if state.gate_armed[gate_index] || state.gate_rearm_time[gate_index].is_some() {
            continue;
        }
        let gate = *plan
            .gates
            .get(gate_index)
            .ok_or(MetricError::InvalidDefinition)?;
        let checked_through = state.gate_rearm_checked_through[gate_index]
            .or(state.last_gate_time[gate_index])
            .ok_or(MetricError::InvalidDefinition)?;
        state.gate_rearm_time[gate_index] = find_gate_rearm_time(
            trajectory,
            plan.reference_point,
            gate,
            checked_through,
            scan_end,
            limits,
            budget,
        )?;
        state.gate_rearm_checked_through[gate_index] = Some(scan_end);
    }

    let last_segment = trajectory.segment_count().saturating_sub(1);
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) =
            trajectory.segment_parameter_overlap(segment_index, scan_start, scan_end)
        else {
            continue;
        };
        let owns_terminal = include_terminal_endpoint && segment_index == last_segment;
        if upper < lower || (upper == lower && !owns_terminal) {
            continue;
        }
        let mut cursor = lower;
        loop {
            let gate_index = state.expected_gate;
            let gate = plan
                .gates
                .get(gate_index)
                .ok_or(MetricError::InvalidDefinition)?;
            if !state.gate_armed[gate_index] {
                let cursor_time = trajectory.time_at_parameter(segment_index, cursor)?;
                let Some(rearm_time) = state.gate_rearm_time[gate_index] else {
                    break;
                };
                if rearm_time > trajectory.time_at_parameter(segment_index, upper)? {
                    break;
                }
                if rearm_time > cursor_time {
                    let Some((rearm_parameter, _)) =
                        trajectory.segment_parameter_overlap(segment_index, rearm_time, rearm_time)
                    else {
                        break;
                    };
                    cursor = cursor.max(rearm_parameter);
                }
                state.gate_armed[gate_index] = true;
            }

            let roots = trajectory.gate_roots_with_budget(
                segment_index,
                plan.reference_point,
                gate.center_ecef_m,
                gate.normal_ecef,
                limits.absolute_root_tolerance_s,
                limits.value_tolerance,
                budget,
            )?;
            let mut accepted = None;
            for root in roots {
                if root < cursor || root > upper || (root == upper && !owns_terminal) {
                    continue;
                }
                let sample = trajectory.scalar_kinematics_at_parameter(
                    segment_index,
                    root,
                    plan.reference_point,
                )?;
                let normal_speed = dot(sample.velocity_ecef_mps, gate.normal_ecef);
                if !gate.direction.accepts(normal_speed)
                    || normal_speed.abs() < gate.minimum_normal_speed_mps
                    || !gate.contains(sample.position_ecef_m, limits.value_tolerance)
                {
                    continue;
                }
                if let Some(previous) = state.last_gate_time[gate_index] {
                    let elapsed_ns = sample.time.as_ns().saturating_sub(previous.as_ns());
                    let minimum_interval =
                        i64::try_from(gate.minimum_crossing_interval.as_ns()).unwrap_or(i64::MAX);
                    if elapsed_ns < minimum_interval {
                        cursor = root.next_up();
                        continue;
                    }
                }
                accepted = Some((root, sample, normal_speed));
                break;
            }
            let Some((root, sample, normal_speed)) = accepted else {
                break;
            };
            let occurrence = state.gate_occurrence[gate_index];
            if occurrence >= u32::from(plan.maximum_occurrences_per_gate) {
                return Err(MetricError::CapacityExceeded);
            }
            let crossing_speed = match plan.crossing_speed {
                Some(quantity) => Some((
                    quantity,
                    scalar_speed(&sample, quantity)
                        .map(FieldValue::Available)
                        .unwrap_or(FieldValue::Unavailable(UnavailableReason::Unobservable)),
                )),
                None => None,
            };
            let event_sensitivity =
                gate_event_sensitivity(segment_index, root, &sample, gate, plan.reference_point);
            let gap_affected = sample.quality.imu_gap;
            let time_one_sigma_s = if gap_affected {
                FieldValue::Unavailable(UnavailableReason::IllConditioned)
            } else {
                event_time_one_sigma(trajectory, uncertainty, event_sensitivity)
            };
            let crossing_speed_one_sigma_mps = plan.crossing_speed.map(|quantity| {
                let sigma = if gap_affected {
                    FieldValue::Unavailable(UnavailableReason::IllConditioned)
                } else if scalar_speed(&sample, quantity).is_none() {
                    FieldValue::Unavailable(UnavailableReason::Unobservable)
                } else {
                    match trajectory.speed_slope_at_parameter(
                        segment_index,
                        root,
                        plan.reference_point,
                        quantity,
                    ) {
                        Ok(slope) => gate_crossing_speed_one_sigma(
                            trajectory,
                            uncertainty,
                            event_sensitivity,
                            &sample,
                            quantity,
                            slope,
                        ),
                        Err(_) => FieldValue::Unavailable(UnavailableReason::IllConditioned),
                    }
                };
                (quantity, sigma)
            });
            let crossing = GateCrossingReport {
                definition: plan.definition,
                gate: gate.id,
                time: sample.time,
                time_one_sigma_s,
                normal_speed_mps: normal_speed,
                crossing_speed,
                crossing_speed_one_sigma_mps,
                reference_point: plan.reference_point,
                occurrence,
                stage: EstimateStage::Finalized,
                validity: metric_validity(sample.quality),
            };
            emit(MetricResultValue::GateCrossing(crossing))?;

            accumulate_live_lap_quality(trajectory, state, crossing.time)?;

            if gate_index == 0 {
                if let Some(start) = state.lap_start {
                    if state.lap_index >= u32::from(plan.maximum_occurrences_per_gate) {
                        return Err(MetricError::CapacityExceeded);
                    }
                    let elapsed_one_sigma_s = if state.accumulated_gap {
                        FieldValue::Unavailable(UnavailableReason::IllConditioned)
                    } else {
                        lap_elapsed_one_sigma(
                            trajectory,
                            uncertainty,
                            &start,
                            state.lap_start_sensitivity.as_ref(),
                            &crossing,
                            event_sensitivity.as_ref().ok(),
                        )
                    };
                    emit(MetricResultValue::Lap(LapReport {
                        definition: plan.definition,
                        lap_index: state.lap_index,
                        start_gate: start.gate,
                        end_gate: crossing.gate,
                        start: start.time,
                        end: crossing.time,
                        elapsed_seconds: seconds_between(start.time, crossing.time)?,
                        elapsed_one_sigma_s,
                        stage: EstimateStage::Finalized,
                        validity: state.accumulated_validity,
                    }))?;
                    state.lap_index = state.lap_index.saturating_add(1);
                }
                state.lap_start = Some(crossing);
                state.lap_start_sensitivity = event_sensitivity.ok();
                state.accumulated_validity = crossing.validity;
                state.accumulated_gap = gap_affected;
                state.quality_checked_through = Some(crossing.time);
            }
            state.last_gate_time[gate_index] = Some(crossing.time);
            state.gate_occurrence[gate_index] = occurrence.saturating_add(1);
            state.gate_armed[gate_index] = false;
            state.gate_rearm_checked_through[gate_index] = Some(crossing.time);
            state.gate_rearm_time[gate_index] = find_gate_rearm_time(
                trajectory,
                plan.reference_point,
                *gate,
                crossing.time,
                scan_end,
                limits,
                budget,
            )?;
            state.gate_rearm_checked_through[gate_index] = Some(scan_end);
            state.expected_gate = (gate_index + 1) % plan.gates.len();
            cursor = root.next_up();
            if cursor > upper || (cursor == upper && !owns_terminal) {
                break;
            }
        }
    }
    accumulate_live_lap_quality(trajectory, state, scan_end)?;
    Ok(())
}
