//! Continuous distance integration and distance-target search.

use super::{
    definition::{DistancePlan, DistanceQuantity, SpeedQuantity},
    geometry::norm,
    numerical::{
        MAX_ROOTS_PER_SEGMENT, MetricEvaluationLimits, NumericalWorkBudget, QuadratureResult,
        brent_with_budget, integrate_with_budget,
    },
    quality::{metric_validity, worse_metric_validity},
    report::{DistanceReport, MetricError},
};
use crate::{
    ids::ReferencePointId,
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::{SessionTime, TimeSpan},
    trajectory::{ScalarKinematics, Trajectory},
};
use heapless::Vec as FixedVec;

pub(super) fn live_distance_report(
    trajectory: &Trajectory,
    plan: DistancePlan,
    previous: Option<DistanceReport>,
    scan_start: SessionTime,
    scan_end: SessionTime,
    scan_available: bool,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<DistanceReport, MetricError> {
    if !scan_available {
        return previous.ok_or(MetricError::EmptyTrajectory);
    }
    let trajectory_span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    let (start, prior_metres, prior_error) = previous
        .map_or((trajectory_span.start(), 0.0, 0.0), |report| {
            (report.span.start(), report.metres, report.numerical_error_m)
        });
    let mut suffix = if scan_end > scan_start || previous.is_none() {
        integrate_trajectory_distance(
            trajectory,
            plan.reference_point,
            plan.quantity,
            scan_start,
            scan_end,
            MetricEvaluationLimits {
                absolute_integration_tolerance: (plan.absolute_tolerance_m - prior_error)
                    .max(f64::MIN_POSITIVE),
                relative_integration_tolerance: plan.relative_tolerance,
                ..limits
            },
            budget,
        )?
    } else {
        QuadratureResult {
            value: 0.0,
            absolute_error: 0.0,
            evaluations: 0,
        }
    };
    let whole_tolerance = |metres: f64| {
        plan.absolute_tolerance_m
            .max(plan.relative_tolerance * metres.abs())
    };
    if prior_error + suffix.absolute_error > whole_tolerance(prior_metres + suffix.value) {
        let remaining = whole_tolerance(prior_metres + suffix.value) - prior_error;
        if remaining <= 0.0 {
            return Err(MetricError::NumericalFailure);
        }
        suffix = integrate_trajectory_distance(
            trajectory,
            plan.reference_point,
            plan.quantity,
            scan_start,
            scan_end,
            MetricEvaluationLimits {
                absolute_integration_tolerance: remaining.next_down(),
                relative_integration_tolerance: 0.0,
                ..limits
            },
            budget,
        )?;
        if prior_error + suffix.absolute_error > whole_tolerance(prior_metres + suffix.value) {
            return Err(MetricError::NumericalFailure);
        }
    }
    let suffix_validity = trajectory
        .conservative_quality_over_span(scan_start, scan_end)
        .map(metric_validity)
        .unwrap_or(Validity::Invalid);
    let validity = previous.map_or(suffix_validity, |report| {
        worse_metric_validity(report.validity, suffix_validity)
    });
    Ok(DistanceReport {
        definition: plan.definition,
        quantity: plan.quantity,
        reference_point: plan.reference_point,
        span: TimeSpan::new(start, scan_end).map_err(|_| MetricError::NumericalFailure)?,
        metres: prior_metres + suffix.value,
        numerical_error_m: prior_error + suffix.absolute_error,
        // Live accumulation currently retains the scalar total but not its
        // augmented covariance/cross-covariance state.
        uncertainty_one_sigma_m: FieldValue::Unavailable(UnavailableReason::MissingCorrelation),
        stage: EstimateStage::Finalized,
        validity,
    })
}

pub(super) fn integrate_trajectory_distance(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: DistanceQuantity,
    start: SessionTime,
    end: SessionTime,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<QuadratureResult, MetricError> {
    if !limits.absolute_integration_tolerance.is_finite()
        || limits.absolute_integration_tolerance <= 0.0
        || !limits.relative_integration_tolerance.is_finite()
        || limits.relative_integration_tolerance < 0.0
    {
        return Err(MetricError::InvalidDefinition);
    }
    let first = integrate_trajectory_distance_pass(
        trajectory,
        reference_point,
        quantity,
        start,
        end,
        limits,
        budget,
    )?;
    if first.absolute_error <= distance_tolerance(first.value, limits) {
        return Ok(first);
    }
    // Local relative allowances can add up too generously when signed
    // portions cancel, or when different cells choose different tolerances.
    // Retry with one absolute allowance for the complete measured result.
    let retry_limits = MetricEvaluationLimits {
        absolute_integration_tolerance: distance_tolerance(first.value, limits).next_down(),
        relative_integration_tolerance: 0.0,
        ..limits
    };
    let result = integrate_trajectory_distance_pass(
        trajectory,
        reference_point,
        quantity,
        start,
        end,
        retry_limits,
        budget,
    )?;
    if result.absolute_error > distance_tolerance(result.value, limits) {
        return Err(MetricError::NumericalFailure);
    }
    Ok(result)
}

#[allow(clippy::too_many_arguments)]
fn integrate_trajectory_distance_pass(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: DistanceQuantity,
    start: SessionTime,
    end: SessionTime,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<QuadratureResult, MetricError> {
    let span_seconds = end
        .checked_duration_since(start)
        .ok_or(MetricError::OutsideTrajectory)?
        .as_seconds_f64();
    let mut total = QuadratureResult {
        value: 0.0,
        absolute_error: 0.0,
        evaluations: 0,
    };
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) = trajectory.segment_parameter_overlap(segment_index, start, end)
        else {
            continue;
        };
        if upper <= lower {
            continue;
        }
        let share =
            (upper - lower) * trajectory.segment_duration_seconds(segment_index) / span_seconds;
        let local = integrate_segment_distance(
            trajectory,
            segment_index,
            reference_point,
            quantity,
            lower,
            upper,
            apportioned_limits(limits, share),
            budget,
        )?;
        total.value += local.value;
        total.absolute_error += local.absolute_error;
        total.evaluations += local.evaluations;
    }
    if !total.value.is_finite() || !total.absolute_error.is_finite() {
        return Err(MetricError::NumericalFailure);
    }
    Ok(total)
}

fn distance_tolerance(value: f64, limits: MetricEvaluationLimits) -> f64 {
    limits
        .absolute_integration_tolerance
        .max(limits.relative_integration_tolerance * value.abs())
}

fn apportioned_limits(limits: MetricEvaluationLimits, fraction: f64) -> MetricEvaluationLimits {
    MetricEvaluationLimits {
        // Downward rounding keeps the sum of each disjoint cell's allocation
        // within the requested whole-span absolute allowance.
        absolute_integration_tolerance: (limits.absolute_integration_tolerance * fraction)
            .next_down()
            .max(f64::MIN_POSITIVE),
        ..limits
    }
}

#[allow(clippy::too_many_arguments)]
fn integrate_segment_distance(
    trajectory: &Trajectory,
    segment_index: usize,
    reference_point: ReferencePointId,
    quantity: DistanceQuantity,
    lower: f64,
    upper: f64,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<QuadratureResult, MetricError> {
    let mut sign_roots = FixedVec::<f64, MAX_ROOTS_PER_SEGMENT>::new();
    if matches!(quantity, DistanceQuantity::BodyLongitudinalAbsolute) {
        sign_roots = isolate_body_longitudinal_sign_roots(
            trajectory,
            segment_index,
            reference_point,
            lower,
            upper,
            limits,
            budget,
        )?;
    }
    integrate_segment_distance_with_sign_roots(
        trajectory,
        segment_index,
        reference_point,
        quantity,
        lower,
        upper,
        sign_roots.as_slice(),
        limits,
        budget,
    )
}

#[allow(clippy::too_many_arguments)]
fn isolate_body_longitudinal_sign_roots(
    trajectory: &Trajectory,
    segment_index: usize,
    reference_point: ReferencePointId,
    lower: f64,
    upper: f64,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<FixedVec<f64, MAX_ROOTS_PER_SEGMENT>, MetricError> {
    trajectory
        .speed_roots_with_budget(
            segment_index,
            reference_point,
            SpeedQuantity::BodyLongitudinalSigned,
            0.0,
            lower,
            upper,
            limits.absolute_root_tolerance_s,
            limits.value_tolerance,
            budget,
        )
        .map_err(|error| match error {
            MetricError::CapacityExceeded | MetricError::EvaluationBudgetExceeded => {
                MetricError::AmbiguousRoot
            }
            other => other,
        })
}

#[allow(clippy::too_many_arguments)]
fn integrate_segment_distance_with_sign_roots(
    trajectory: &Trajectory,
    segment_index: usize,
    reference_point: ReferencePointId,
    quantity: DistanceQuantity,
    lower: f64,
    upper: f64,
    sign_roots: &[f64],
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<QuadratureResult, MetricError> {
    let duration = trajectory.segment_duration_seconds(segment_index);
    let mut total = QuadratureResult {
        value: 0.0,
        absolute_error: 0.0,
        evaluations: 0,
    };
    let mut interval_lower = lower;
    for interval_upper in sign_roots
        .iter()
        .copied()
        .filter(|root| *root > lower && *root < upper)
        .chain(core::iter::once(upper))
    {
        if interval_upper <= interval_lower {
            continue;
        }
        let interval_limits =
            apportioned_limits(limits, (interval_upper - interval_lower) / (upper - lower));
        let local = integrate_with_budget(
            |parameter| {
                trajectory
                    .scalar_kinematics_at_parameter(segment_index, parameter, reference_point)
                    .ok()
                    .and_then(|state| distance_integrand(&state, quantity))
                    .unwrap_or(f64::NAN)
                    * duration
            },
            interval_lower,
            interval_upper,
            interval_limits.absolute_integration_tolerance,
            interval_limits.relative_integration_tolerance,
            budget,
        )?;
        total.value += local.value;
        total.absolute_error += local.absolute_error;
        total.evaluations += local.evaluations;
        interval_lower = interval_upper;
    }
    Ok(total)
}

fn distance_integrand(state: &ScalarKinematics, quantity: DistanceQuantity) -> Option<f64> {
    match quantity {
        DistanceQuantity::HorizontalPath => Some(state.horizontal_speed_mps),
        DistanceQuantity::Spatial3d => Some(norm(state.velocity_ecef_mps)),
        DistanceQuantity::BodyLongitudinalSigned => state.body_longitudinal_speed_mps,
        DistanceQuantity::BodyLongitudinalAbsolute => {
            state.body_longitudinal_speed_mps.map(f64::abs)
        }
    }
}

pub(super) fn find_distance_target(
    trajectory: &Trajectory,
    reference_point: ReferencePointId,
    quantity: DistanceQuantity,
    after: SessionTime,
    target_metres: f64,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<Option<SessionTime>, MetricError> {
    if target_metres == 0.0 {
        return Ok(Some(after));
    }
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    let span_seconds = span
        .end()
        .checked_duration_since(after)
        .ok_or(MetricError::OutsideTrajectory)?
        .as_seconds_f64();
    let limits = MetricEvaluationLimits {
        absolute_integration_tolerance: distance_tolerance(target_metres, limits),
        relative_integration_tolerance: 0.0,
        ..limits
    };
    let mut accumulated = 0.0;
    for segment_index in 0..trajectory.segment_count() {
        let Some((lower, upper)) =
            trajectory.segment_parameter_overlap(segment_index, after, span.end())
        else {
            continue;
        };
        let duration = trajectory.segment_duration_seconds(segment_index);
        if upper <= lower {
            continue;
        }
        let limits = apportioned_limits(limits, (upper - lower) * duration / span_seconds);
        let mut sign_roots = FixedVec::<f64, MAX_ROOTS_PER_SEGMENT>::new();
        if matches!(quantity, DistanceQuantity::BodyLongitudinalAbsolute) {
            sign_roots = isolate_body_longitudinal_sign_roots(
                trajectory,
                segment_index,
                reference_point,
                lower,
                upper,
                limits,
                budget,
            )?;
        }
        let local = integrate_segment_distance_with_sign_roots(
            trajectory,
            segment_index,
            reference_point,
            quantity,
            lower,
            upper,
            sign_roots.as_slice(),
            limits,
            budget,
        )?;
        let monotone = !matches!(quantity, DistanceQuantity::BodyLongitudinalSigned);
        if monotone && accumulated + local.value >= target_metres {
            let parameter = brent_with_budget(
                |candidate, budget| {
                    integrate_segment_distance_with_sign_roots(
                        trajectory,
                        segment_index,
                        reference_point,
                        quantity,
                        lower,
                        candidate,
                        sign_roots.as_slice(),
                        apportioned_limits(limits, (candidate - lower) / (upper - lower)),
                        budget,
                    )
                    .map(|partial| accumulated + partial.value - target_metres)
                },
                lower,
                upper,
                limits.absolute_root_tolerance_s / duration.max(f64::MIN_POSITIVE),
                limits.value_tolerance,
                budget,
            )?;
            return trajectory
                .time_at_parameter(segment_index, parameter)
                .map(Some);
        }
        if !monotone {
            // Signed displacement can reverse inside one dense interval. Its
            // derivative is body-longitudinal speed, so exhaustively isolate
            // every stationary point before testing the resulting monotone
            // intervals. This also exposes an even-multiplicity/tangent target
            // as a zero-valued stationary endpoint.
            let stationary = isolate_body_longitudinal_sign_roots(
                trajectory,
                segment_index,
                reference_point,
                lower,
                upper,
                limits,
                budget,
            )?;
            let mut points = FixedVec::<f64, { MAX_ROOTS_PER_SEGMENT + 2 }>::new();
            points
                .push(lower)
                .map_err(|_| MetricError::CapacityExceeded)?;
            for stationary_point in stationary {
                if stationary_point > lower && stationary_point < upper {
                    points
                        .push(stationary_point)
                        .map_err(|_| MetricError::CapacityExceeded)?;
                }
            }
            points
                .push(upper)
                .map_err(|_| MetricError::CapacityExceeded)?;
            points.sort_unstable_by(f64::total_cmp);

            let mut interval_left = lower;
            let mut value_left = accumulated - target_metres;
            for &interval_right in points.as_slice().iter().skip(1) {
                let interval_limits =
                    apportioned_limits(limits, (interval_right - interval_left) / (upper - lower));
                let partial = integrate_with_budget(
                    |sample| {
                        trajectory
                            .scalar_kinematics_at_parameter(segment_index, sample, reference_point)
                            .ok()
                            .and_then(|state| distance_integrand(&state, quantity))
                            .unwrap_or(f64::NAN)
                            * duration
                    },
                    interval_left,
                    interval_right,
                    interval_limits.absolute_integration_tolerance,
                    interval_limits.relative_integration_tolerance,
                    budget,
                )?;
                let value_right = value_left + partial.value;
                if value_left.abs() <= limits.value_tolerance {
                    return trajectory
                        .time_at_parameter(segment_index, interval_left)
                        .map(Some);
                }
                if value_left.is_sign_positive() != value_right.is_sign_positive() {
                    let parameter = brent_with_budget(
                        |candidate, budget| {
                            integrate_with_budget(
                                |sample| {
                                    trajectory
                                        .scalar_kinematics_at_parameter(
                                            segment_index,
                                            sample,
                                            reference_point,
                                        )
                                        .ok()
                                        .and_then(|state| distance_integrand(&state, quantity))
                                        .unwrap_or(f64::NAN)
                                        * duration
                                },
                                interval_left,
                                candidate,
                                apportioned_limits(
                                    limits,
                                    (candidate - interval_left) / (upper - lower),
                                )
                                .absolute_integration_tolerance,
                                0.0,
                                budget,
                            )
                            .map(|candidate_integral| value_left + candidate_integral.value)
                        },
                        interval_left,
                        interval_right,
                        limits.absolute_root_tolerance_s / duration.max(f64::MIN_POSITIVE),
                        limits.value_tolerance,
                        budget,
                    )?;
                    return trajectory
                        .time_at_parameter(segment_index, parameter)
                        .map(Some);
                }
                if value_right.abs() <= limits.value_tolerance {
                    return trajectory
                        .time_at_parameter(segment_index, interval_right)
                        .map(Some);
                }
                interval_left = interval_right;
                value_left = value_right;
            }
        }
        accumulated += local.value;
    }
    Ok(None)
}
