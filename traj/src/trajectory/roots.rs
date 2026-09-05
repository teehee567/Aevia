//! Budgeted interval root isolation and endpoint ownership.

use super::MAX_SEGMENT_ROOTS;
use super::math::{midpoint, upper_add, upper_mul};
use crate::metric::{MetricError, NumericalWorkBudget};
use heapless::Vec as FixedVec;

/// Maximum live/embedded interval cells retained by the depth-first root
/// isolator. The stack grows with subdivision depth, not with the total number
/// of cells visited.
pub(super) const MAX_ROOT_ISOLATION_STACK: usize = 96;

/// Independent recursion-depth ceiling. This also protects against an
/// unproductive split once adjacent floating-point parameters are reached.
pub(super) const MAX_ROOT_ISOLATION_DEPTH: u8 = 64;

/// Closed, conservatively expanded scalar interval used by the current native
/// `f64` Taylor backend. This is not the plan's qualified `EnclosureV1`; live
/// preflight keeps non-polynomial uses unavailable unless the exact backend is
/// covered by a measured numeric attestation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct OutwardInterval {
    pub(super) lower: f64,
    pub(super) upper: f64,
}

impl OutwardInterval {
    #[cfg(test)]
    pub(super) fn new(lower: f64, upper: f64) -> Result<Self, MetricError> {
        if lower.is_nan() || upper.is_nan() || lower > upper {
            return Err(MetricError::NumericalFailure);
        }
        Ok(Self { lower, upper })
    }

    pub(super) fn around(center: f64, radius: f64) -> Result<Self, MetricError> {
        if !center.is_finite() || !radius.is_finite() || radius < 0.0 {
            return Err(MetricError::NumericalFailure);
        }
        Ok(Self {
            lower: (center - radius).next_down(),
            upper: (center + radius).next_up(),
        })
    }

    pub(super) fn intersects_zero_band(self, tolerance: f64) -> bool {
        self.lower <= tolerance && self.upper >= -tolerance
    }

    pub(super) fn strict_sign_outside(self, tolerance: f64) -> i8 {
        if self.lower > tolerance {
            1
        } else if self.upper < -tolerance {
            -1
        } else {
            0
        }
    }
}

/// Estimate plus conservative enclosures over one parameter cell. Derivatives
/// are with respect to normalized segment parameter `s`, not seconds.
#[derive(Clone, Copy, Debug)]
pub(super) struct ScalarEnclosure {
    pub(super) value_estimate: f64,
    pub(super) derivative_estimate: f64,
    pub(super) value: OutwardInterval,
    pub(super) derivative: OutwardInterval,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct ScalarJet {
    pub(super) value: f64,
    pub(super) derivative: f64,
    pub(super) second_derivative: f64,
    pub(super) value_roundoff: f64,
    pub(super) derivative_roundoff: f64,
    pub(super) second_derivative_roundoff: f64,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct RootCell {
    pub(super) lower: f64,
    pub(super) upper: f64,
    pub(super) depth: u8,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct EndpointOwnership {
    pub(super) lower: bool,
    pub(super) upper: bool,
}

pub(super) fn taylor_enclosure(
    jet: ScalarJet,
    supplied_second_derivative_bound: f64,
    lower: f64,
    upper: f64,
) -> Result<ScalarEnclosure, MetricError> {
    if !lower.is_finite()
        || !upper.is_finite()
        || lower > upper
        || !jet.value.is_finite()
        || !jet.derivative.is_finite()
        || !jet.second_derivative.is_finite()
        || !jet.value_roundoff.is_finite()
        || !jet.derivative_roundoff.is_finite()
        || !jet.second_derivative_roundoff.is_finite()
        || !supplied_second_derivative_bound.is_finite()
        || supplied_second_derivative_bound < 0.0
    {
        return Err(MetricError::NumericalFailure);
    }
    let half_width = (upper - lower) * 0.5;
    let observed_second = upper_add(jet.second_derivative.abs(), jet.second_derivative_roundoff);
    let second_bound = supplied_second_derivative_bound.max(observed_second);
    let derivative_radius = upper_add(jet.derivative_roundoff, upper_mul(second_bound, half_width));
    let linear_radius = upper_mul(
        upper_add(jet.derivative.abs(), jet.derivative_roundoff),
        half_width,
    );
    let quadratic_radius = upper_mul(
        0.5,
        upper_mul(second_bound, upper_mul(half_width, half_width)),
    );
    let value_radius = upper_add(
        jet.value_roundoff,
        upper_add(linear_radius, quadratic_radius),
    );
    Ok(ScalarEnclosure {
        value_estimate: jet.value,
        derivative_estimate: jet.derivative,
        value: OutwardInterval::around(jet.value, value_radius)?,
        derivative: OutwardInterval::around(jet.derivative, derivative_radius)?,
    })
}

/// Bounded interval branch-and-bound for the private native Taylor backend. A
/// cell is discarded only after its expanded value interval excludes the
/// configured zero band. A monotonic cell may be refined once opposite
/// endpoint signs prove a root. Every other unresolved cell is subdivided or
/// fails closed; sampling is never evidence that a root is absent. The
/// interval backend itself requires separate qualification before live
/// non-polynomial use.
#[cfg(test)]
pub(super) fn isolate_enclosed_roots<F>(
    lower: f64,
    upper: f64,
    x_tolerance: f64,
    value_tolerance: f64,
    ownership: EndpointOwnership,
    maximum_evaluations: u32,
    oracle: F,
) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError>
where
    F: Fn(f64, f64) -> Result<ScalarEnclosure, MetricError>,
{
    let mut budget = NumericalWorkBudget::root_only(maximum_evaluations);
    isolate_enclosed_roots_with_budget(
        lower,
        upper,
        x_tolerance,
        value_tolerance,
        ownership,
        &mut budget,
        oracle,
    )
}

pub(super) fn isolate_enclosed_roots_with_budget<F>(
    lower: f64,
    upper: f64,
    x_tolerance: f64,
    value_tolerance: f64,
    ownership: EndpointOwnership,
    budget: &mut NumericalWorkBudget,
    oracle: F,
) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError>
where
    F: Fn(f64, f64) -> Result<ScalarEnclosure, MetricError>,
{
    if !lower.is_finite()
        || !upper.is_finite()
        || lower > upper
        || !x_tolerance.is_finite()
        || x_tolerance <= 0.0
        || !value_tolerance.is_finite()
        || value_tolerance < 0.0
    {
        return Err(MetricError::InvalidDefinition);
    }

    let lower_value = evaluate_root_oracle(&oracle, lower, lower, budget)?;
    let upper_value = if upper == lower {
        lower_value
    } else {
        evaluate_root_oracle(&oracle, upper, upper, budget)?
    };
    let mut roots = FixedVec::new();
    maybe_push_endpoint(
        &mut roots,
        lower,
        lower_value,
        value_tolerance,
        lower,
        upper,
        x_tolerance,
        ownership,
    )?;
    if upper != lower {
        maybe_push_endpoint(
            &mut roots,
            upper,
            upper_value,
            value_tolerance,
            lower,
            upper,
            x_tolerance,
            ownership,
        )?;
    }
    if upper == lower {
        return if roots.is_empty() && lower_value.value.intersects_zero_band(value_tolerance) {
            Err(MetricError::AmbiguousRoot)
        } else {
            Ok(roots)
        };
    }

    let mut stack = FixedVec::<RootCell, MAX_ROOT_ISOLATION_STACK>::new();
    stack
        .push(RootCell {
            lower,
            upper,
            depth: 0,
        })
        .map_err(|_| MetricError::CapacityExceeded)?;

    while let Some(cell) = stack.pop() {
        let enclosure = evaluate_root_oracle(&oracle, cell.lower, cell.upper, budget)?;
        if !enclosure.value.intersects_zero_band(value_tolerance) {
            continue;
        }

        let width = cell.upper - cell.lower;
        let derivative_sign = enclosure.derivative.strict_sign_outside(0.0);
        if derivative_sign != 0 {
            let left = evaluate_root_oracle(&oracle, cell.lower, cell.lower, budget)?;
            let right = evaluate_root_oracle(&oracle, cell.upper, cell.upper, budget)?;
            let left_sign = left.value.strict_sign_outside(0.0);
            let right_sign = right.value.strict_sign_outside(0.0);
            if left_sign * right_sign < 0 {
                let root = refine_enclosed_bracket(
                    &oracle,
                    cell.lower,
                    cell.upper,
                    false,
                    x_tolerance,
                    value_tolerance,
                    budget,
                )?;
                push_owned_root(&mut roots, root, lower, upper, x_tolerance, ownership)?;
                continue;
            }
            if left.value_estimate == 0.0 {
                push_owned_root(&mut roots, cell.lower, lower, upper, x_tolerance, ownership)?;
                continue;
            }
            if right.value_estimate == 0.0 {
                push_owned_root(&mut roots, cell.upper, lower, upper, x_tolerance, ownership)?;
                continue;
            }
            if left_sign != 0 && right_sign != 0 {
                // Under a qualified enclosure contract, a derivative-sign
                // interval establishes monotonicity and equal endpoint signs
                // exclude a root.
                if left_sign == right_sign {
                    continue;
                }
                return Err(MetricError::AmbiguousRoot);
            }
        }

        let midpoint = midpoint(cell.lower, cell.upper);
        let cannot_split = width <= x_tolerance
            || cell.depth >= MAX_ROOT_ISOLATION_DEPTH
            || midpoint <= cell.lower
            || midpoint >= cell.upper;
        if !cannot_split {
            stack
                .push(RootCell {
                    lower: midpoint,
                    upper: cell.upper,
                    depth: cell.depth + 1,
                })
                .map_err(|_| MetricError::CapacityExceeded)?;
            stack
                .push(RootCell {
                    lower: cell.lower,
                    upper: midpoint,
                    depth: cell.depth + 1,
                })
                .map_err(|_| MetricError::CapacityExceeded)?;
            continue;
        }

        let left = evaluate_root_oracle(&oracle, cell.lower, cell.lower, budget)?;
        let center = evaluate_root_oracle(&oracle, midpoint, midpoint, budget)?;
        let right = evaluate_root_oracle(&oracle, cell.upper, cell.upper, budget)?;
        let left_sign = left.value.strict_sign_outside(0.0);
        let right_sign = right.value.strict_sign_outside(0.0);
        if left_sign * right_sign < 0 {
            let root = refine_enclosed_bracket(
                &oracle,
                cell.lower,
                cell.upper,
                false,
                x_tolerance,
                value_tolerance,
                budget,
            )?;
            push_owned_root(&mut roots, root, lower, upper, x_tolerance, ownership)?;
            continue;
        }

        let candidate = [(cell.lower, left), (midpoint, center), (cell.upper, right)]
            .into_iter()
            .min_by(|left, right| {
                left.1
                    .value_estimate
                    .abs()
                    .total_cmp(&right.1.value_estimate.abs())
            })
            .ok_or(MetricError::NumericalFailure)?;
        let exact_point_zero = candidate.1.value_estimate == 0.0;
        let stationary_near_zero = candidate.1.value_estimate.abs() <= value_tolerance
            && candidate.1.derivative.intersects_zero_band(0.0);
        if exact_point_zero || stationary_near_zero {
            push_owned_root(
                &mut roots,
                candidate.0,
                lower,
                upper,
                x_tolerance,
                ownership,
            )?;
            continue;
        }

        let left_derivative_sign = left.derivative.strict_sign_outside(0.0);
        let right_derivative_sign = right.derivative.strict_sign_outside(0.0);
        if left_derivative_sign * right_derivative_sign < 0 {
            let stationary = refine_enclosed_bracket(
                &oracle,
                cell.lower,
                cell.upper,
                true,
                x_tolerance,
                0.0,
                budget,
            )?;
            let contact = evaluate_root_oracle(&oracle, stationary, stationary, budget)?;
            if contact.value_estimate.abs() <= value_tolerance {
                push_owned_root(&mut roots, stationary, lower, upper, x_tolerance, ownership)?;
                continue;
            }
        }

        // The interval extension says zero remains possible, but neither a
        // crossing nor a stationary contact was established at the compiled
        // width/depth limit. Returning no root here would be a false proof.
        return Err(MetricError::AmbiguousRoot);
    }

    roots.sort_unstable_by(f64::total_cmp);
    Ok(roots)
}

pub(super) fn evaluate_root_oracle<F>(
    oracle: &F,
    lower: f64,
    upper: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<ScalarEnclosure, MetricError>
where
    F: Fn(f64, f64) -> Result<ScalarEnclosure, MetricError>,
{
    budget.charge_root_evaluation()?;
    let enclosure = oracle(lower, upper)?;
    if !enclosure.value_estimate.is_finite()
        || !enclosure.derivative_estimate.is_finite()
        || enclosure.value.lower.is_nan()
        || enclosure.value.upper.is_nan()
        || enclosure.derivative.lower.is_nan()
        || enclosure.derivative.upper.is_nan()
        || enclosure.value.lower > enclosure.value.upper
        || enclosure.derivative.lower > enclosure.derivative.upper
        || enclosure.value_estimate < enclosure.value.lower
        || enclosure.value_estimate > enclosure.value.upper
        || enclosure.derivative_estimate < enclosure.derivative.lower
        || enclosure.derivative_estimate > enclosure.derivative.upper
    {
        return Err(MetricError::NumericalFailure);
    }
    Ok(enclosure)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn refine_enclosed_bracket<F>(
    oracle: &F,
    mut lower: f64,
    mut upper: f64,
    derivative: bool,
    x_tolerance: f64,
    value_tolerance: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<f64, MetricError>
where
    F: Fn(f64, f64) -> Result<ScalarEnclosure, MetricError>,
{
    let mut left = evaluate_root_oracle(oracle, lower, lower, budget)?;
    let right = evaluate_root_oracle(oracle, upper, upper, budget)?;
    let interval_of = |sample: ScalarEnclosure| {
        if derivative {
            sample.derivative
        } else {
            sample.value
        }
    };
    let estimate_of = |sample: ScalarEnclosure| {
        if derivative {
            sample.derivative_estimate
        } else {
            sample.value_estimate
        }
    };
    let mut left_sign = interval_of(left).strict_sign_outside(value_tolerance);
    let right_sign = interval_of(right).strict_sign_outside(value_tolerance);
    if !derivative && left.value_estimate.abs() <= value_tolerance {
        return Ok(lower);
    }
    if !derivative && right.value_estimate.abs() <= value_tolerance {
        return Ok(upper);
    }
    if left_sign * right_sign >= 0 {
        return Err(MetricError::AmbiguousRoot);
    }

    while upper - lower > x_tolerance {
        let center_parameter = midpoint(lower, upper);
        if center_parameter <= lower || center_parameter >= upper {
            break;
        }
        let center = evaluate_root_oracle(oracle, center_parameter, center_parameter, budget)?;
        if estimate_of(center).abs() <= value_tolerance {
            return Ok(center_parameter);
        }
        let center_sign = interval_of(center).strict_sign_outside(value_tolerance);
        if center_sign == 0 {
            return Err(MetricError::AmbiguousRoot);
        }
        if left_sign * center_sign < 0 {
            upper = center_parameter;
        } else {
            lower = center_parameter;
            left = center;
            left_sign = center_sign;
        }
    }
    let left_estimate = estimate_of(left).abs();
    let right = evaluate_root_oracle(oracle, upper, upper, budget)?;
    if left_estimate <= estimate_of(right).abs() {
        Ok(lower)
    } else {
        Ok(upper)
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn maybe_push_endpoint(
    roots: &mut FixedVec<f64, MAX_SEGMENT_ROOTS>,
    candidate: f64,
    enclosure: ScalarEnclosure,
    value_tolerance: f64,
    lower: f64,
    upper: f64,
    x_tolerance: f64,
    ownership: EndpointOwnership,
) -> Result<(), MetricError> {
    if enclosure.value_estimate.abs() <= value_tolerance
        && enclosure.value.intersects_zero_band(value_tolerance)
    {
        push_owned_root(roots, candidate, lower, upper, x_tolerance, ownership)?;
    }
    Ok(())
}

pub(super) fn push_owned_root(
    roots: &mut FixedVec<f64, MAX_SEGMENT_ROOTS>,
    candidate: f64,
    lower: f64,
    upper: f64,
    tolerance: f64,
    ownership: EndpointOwnership,
) -> Result<(), MetricError> {
    if (!ownership.lower && candidate == lower) || (!ownership.upper && candidate == upper) {
        return Ok(());
    }
    push_unique_root(roots, candidate.clamp(lower, upper), tolerance)
}

pub(super) fn filter_owned_roots(
    source: FixedVec<f64, MAX_SEGMENT_ROOTS>,
    lower: f64,
    upper: f64,
    tolerance: f64,
    ownership: EndpointOwnership,
) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError> {
    let mut filtered = FixedVec::new();
    for candidate in source {
        push_owned_root(&mut filtered, candidate, lower, upper, tolerance, ownership)?;
    }
    Ok(filtered)
}

pub(super) fn push_unique_root(
    roots: &mut FixedVec<f64, MAX_SEGMENT_ROOTS>,
    candidate: f64,
    tolerance: f64,
) -> Result<(), MetricError> {
    if roots
        .iter()
        .any(|present| (*present - candidate).abs() <= tolerance)
    {
        return Ok(());
    }
    roots
        .push(candidate)
        .map_err(|_| MetricError::CapacityExceeded)
}
