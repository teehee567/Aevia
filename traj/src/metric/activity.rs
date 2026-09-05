//! Continuous moving time, speed extrema, and vertical totals.

use super::{
    definition::SpeedQuantity,
    geometry::scalar_speed,
    numerical::{
        MAX_ROOTS_PER_SEGMENT, MetricEvaluationLimits, NumericalWorkBudget, integrate_with_budget,
    },
    report::MetricError,
};
use crate::{ids::ReferencePointId, time::SessionTime, trajectory::Trajectory};
use heapless::Vec as FixedVec;

pub(super) fn activity_extrema(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    moving_quantity: SpeedQuantity,
    moving_threshold: f64,
    peak_quantity: SpeedQuantity,
    start: SessionTime,
    end: SessionTime,
    include_vertical_totals: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<(f64, f64, f64, f64), MetricError> {
    let mut moving_seconds = 0.0;
    let mut peak_speed: Option<f64> = None;
    let mut ascent = 0.0;
    let mut descent = 0.0;
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) = trajectory.segment_parameter_overlap(segment_index, start, end)
        else {
            continue;
        };
        if upper <= lower {
            continue;
        }
        let duration = trajectory.segment_duration_seconds(segment_index);
        let roots = trajectory.speed_roots_with_budget(
            segment_index,
            reference_point,
            moving_quantity,
            moving_threshold,
            lower,
            upper,
            limits.absolute_root_tolerance_s,
            limits.value_tolerance,
            budget,
        )?;
        let mut points = FixedVec::<f64, { MAX_ROOTS_PER_SEGMENT + 2 }>::new();
        points
            .push(lower)
            .map_err(|_| MetricError::CapacityExceeded)?;
        for root in roots {
            if root > lower && root < upper {
                points
                    .push(root)
                    .map_err(|_| MetricError::CapacityExceeded)?;
            }
        }
        points
            .push(upper)
            .map_err(|_| MetricError::CapacityExceeded)?;
        points.sort_unstable_by(f64::total_cmp);
        for window in points.windows(2) {
            let midpoint = (window[0] + window[1]) * 0.5;
            let state = trajectory.scalar_kinematics_at_parameter(
                segment_index,
                midpoint,
                reference_point,
            )?;
            if scalar_speed(&state, moving_quantity).is_some_and(|speed| speed >= moving_threshold)
            {
                moving_seconds += (window[1] - window[0]) * duration;
            }
        }
        if include_vertical_totals {
            let up = integrate_with_budget(
                |parameter| {
                    trajectory
                        .scalar_kinematics_at_parameter(segment_index, parameter, reference_point)
                        .map_or(f64::NAN, |state| {
                            state.vertical_speed_mps.max(0.0) * duration
                        })
                },
                lower,
                upper,
                limits.absolute_integration_tolerance,
                limits.relative_integration_tolerance,
                budget,
            )?;
            let down = integrate_with_budget(
                |parameter| {
                    trajectory
                        .scalar_kinematics_at_parameter(segment_index, parameter, reference_point)
                        .map_or(f64::NAN, |state| {
                            (-state.vertical_speed_mps).max(0.0) * duration
                        })
                },
                lower,
                upper,
                limits.absolute_integration_tolerance,
                limits.relative_integration_tolerance,
                budget,
            )?;
            ascent += up.value;
            descent += down.value;
        }
        let extrema = trajectory.speed_extrema_parameters_with_budget(
            segment_index,
            reference_point,
            peak_quantity,
            limits.absolute_root_tolerance_s,
            limits.value_tolerance,
            budget,
        )?;
        let mut peak_parameters = FixedVec::<f64, { MAX_ROOTS_PER_SEGMENT + 2 }>::new();
        peak_parameters
            .push(lower)
            .map_err(|_| MetricError::CapacityExceeded)?;
        for parameter in extrema {
            if parameter > lower && parameter < upper {
                peak_parameters
                    .push(parameter)
                    .map_err(|_| MetricError::CapacityExceeded)?;
            }
        }
        peak_parameters
            .push(upper)
            .map_err(|_| MetricError::CapacityExceeded)?;
        for parameter in peak_parameters {
            let state = trajectory.scalar_kinematics_at_parameter(
                segment_index,
                parameter,
                reference_point,
            )?;
            let speed = scalar_speed(&state, peak_quantity).ok_or(MetricError::Unobservable)?;
            peak_speed = Some(peak_speed.map_or(speed, |peak| peak.max(speed)));
        }
    }
    Ok((
        moving_seconds,
        peak_speed.ok_or(MetricError::Unobservable)?,
        ascent,
        descent,
    ))
}
