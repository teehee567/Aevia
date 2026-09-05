//! Continuous launch and speed-threshold event search.

use super::{
    definition::{LaunchRule, SpeedQuantity, TargetDirection},
    geometry::{norm, scalar_speed},
    numerical::{MetricEvaluationLimits, NumericalWorkBudget},
    report::MetricError,
};
use crate::{
    ids::ReferencePointId,
    time::{DurationNs, SessionTime, SignedDurationNs},
    trajectory::Trajectory,
};

pub(super) fn find_launch(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    rule: LaunchRule,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SessionTime>, MetricError> {
    match rule {
        LaunchRule::ExternalTimestamp(time) => {
            let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
            Ok(span.contains(time).then_some(time))
        }
        LaunchRule::FirstSustainedMotion {
            threshold_mps,
            dwell,
        } => find_sustained_threshold(
            trajectory,
            reference_point,
            SpeedQuantity::InstantaneousHorizontal,
            threshold_mps,
            dwell,
            limits,
            budget,
        ),
        LaunchRule::SpeedThreshold {
            quantity,
            threshold_mps,
            dwell,
        } => find_sustained_threshold(
            trajectory,
            reference_point,
            quantity,
            threshold_mps,
            dwell,
            limits,
            budget,
        ),
        LaunchRule::AccelerationChangePoint {
            minimum_acceleration_mps2,
            dwell,
        } => {
            let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
            let step_ns = 5_000_000i64;
            let dwell_ns =
                i64::try_from(dwell.as_ns()).map_err(|_| MetricError::InvalidDefinition)?;
            let mut candidate = None;
            let mut time_ns = span.start().as_ns();
            while time_ns <= span.end().as_ns() {
                // Unlike threshold roots, the acceleration launch rule uses
                // an explicit 5 ms scalar scan. Charge the same scalar-work
                // ledger so even an extreme session span cannot evade the
                // compiled live bound (or stall at saturating i64::MAX).
                budget.charge_root_evaluation()?;
                let time = SessionTime::from_ns(time_ns);
                let state = trajectory.scalar_kinematics_at(time, reference_point)?;
                let acceleration = state
                    .acceleration_ecef_mps2
                    .ok_or(MetricError::Unobservable)?;
                if norm(acceleration) >= minimum_acceleration_mps2 {
                    let candidate_start = *candidate.get_or_insert(time);
                    if time_ns.saturating_sub(candidate_start.as_ns()) >= dwell_ns {
                        return Ok(candidate);
                    }
                } else {
                    candidate = None;
                }
                time_ns = time_ns.saturating_add(step_ns);
            }
            Ok(None)
        }
    }
}

fn find_sustained_threshold(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: SpeedQuantity,
    threshold_mps: f64,
    dwell: DurationNs,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SessionTime>, MetricError> {
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    let dwell_ns = i64::try_from(dwell.as_ns()).map_err(|_| MetricError::InvalidDefinition)?;
    let initial = trajectory.scalar_kinematics_at(span.start(), reference_point)?;
    let mut search_after = span.start();
    let mut initial_is_candidate =
        scalar_speed(&initial, quantity).ok_or(MetricError::Unobservable)? >= threshold_mps;

    loop {
        let candidate = if initial_is_candidate {
            initial_is_candidate = false;
            Some(span.start())
        } else {
            find_speed_target(
                trajectory,
                reference_point,
                quantity,
                search_after,
                threshold_mps,
                TargetDirection::Ascending,
                limits,
                budget,
            )?
            .map(|event| event.0)
        };
        let Some(time) = candidate else {
            return Ok(None);
        };
        let end_ns = time
            .as_ns()
            .checked_add(dwell_ns)
            .ok_or(MetricError::NumericalFailure)?;
        if end_ns > span.end().as_ns() {
            return Ok(None);
        }

        let next_drop = find_speed_target(
            trajectory,
            reference_point,
            quantity,
            time,
            threshold_mps,
            TargetDirection::Descending,
            limits,
            budget,
        )?
        .map(|event| event.0);
        if next_drop.is_none_or(|drop| drop.as_ns() >= end_ns) {
            return Ok(Some(time));
        }
        search_after = next_drop
            .and_then(|drop| drop.checked_add(SignedDurationNs::from_ns(1)))
            .ok_or(MetricError::NumericalFailure)?;
        if search_after >= span.end() {
            return Ok(None);
        }
    }
}

type SpeedEvent = (
    SessionTime,
    Option<(SpeedQuantity, f64)>,
    Option<f64>,
    Option<(usize, f64)>,
);

pub(super) fn find_speed_target(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: SpeedQuantity,
    after: SessionTime,
    target_mps: f64,
    direction: TargetDirection,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SpeedEvent>, MetricError> {
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) = trajectory.segment_parameter_overlap(
            segment_index,
            after,
            trajectory.span().ok_or(MetricError::EmptyTrajectory)?.end(),
        ) else {
            continue;
        };
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
        for parameter in roots.as_slice() {
            let state = trajectory.scalar_kinematics_at_parameter(
                segment_index,
                *parameter,
                reference_point,
            )?;
            let speed = scalar_speed(&state, quantity).ok_or(MetricError::Unobservable)?;
            let slope = trajectory.speed_slope_at_parameter(
                segment_index,
                *parameter,
                reference_point,
                quantity,
            )?;
            let accepted = match direction {
                TargetDirection::Ascending => slope > 0.0,
                TargetDirection::Descending => slope < 0.0,
                TargetDirection::Either => slope != 0.0,
            };
            if accepted {
                return Ok(Some((
                    state.time,
                    Some((quantity, speed)),
                    Some(slope),
                    Some((segment_index, *parameter)),
                )));
            }
        }
    }
    Ok(None)
}
