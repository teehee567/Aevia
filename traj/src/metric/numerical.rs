//! Shared work budgets, root isolation, and adaptive quadrature.

use super::{geometry::all_finite, plan::LiveMetricLimits, report::MetricError};
use core::ops::RangeInclusive;
use heapless::Vec as FixedVec;
#[cfg(not(any(test, feature = "offline")))]
use nalgebra::ComplexField;

/// Maximum isolated roots owned by one dense interval and scalar function.
pub(super) const MAX_ROOTS_PER_SEGMENT: usize = 12;

const MAX_QUADRATURE_STACK: usize = 96;

/// Numerical controls for host/full-span evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricEvaluationLimits {
    pub absolute_root_tolerance_s: f64,
    pub value_tolerance: f64,
    pub absolute_integration_tolerance: f64,
    pub relative_integration_tolerance: f64,
    pub maximum_root_evaluations: u32,
    pub maximum_quadrature_evaluations: u32,
}

impl Default for MetricEvaluationLimits {
    fn default() -> Self {
        Self {
            absolute_root_tolerance_s: 1.0e-8,
            value_tolerance: 1.0e-9,
            absolute_integration_tolerance: 1.0e-6,
            relative_integration_tolerance: 1.0e-9,
            maximum_root_evaluations: 2_048,
            maximum_quadrature_evaluations: 16_384,
        }
    }
}

// One adaptive Gauss-Kronrod panel evaluates exactly fifteen abscissae. V1
// live contracts reuse each configured root-evaluation credit as one panel
// credit so quadrature is bounded without changing the serialized
// `LiveMetricLimits` schema.
const LIVE_QUADRATURE_EVALUATIONS_PER_ROOT_CREDIT: u32 = 15;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) struct NumericalWork {
    pub(super) root_evaluations: u32,
    pub(super) quadrature_evaluations: u32,
}

impl NumericalWork {
    pub(super) fn checked_add_assign(&mut self, other: Self) -> Result<(), MetricError> {
        self.root_evaluations = self
            .root_evaluations
            .checked_add(other.root_evaluations)
            .ok_or(MetricError::EvaluationBudgetExceeded)?;
        self.quadrature_evaluations = self
            .quadrature_evaluations
            .checked_add(other.quadrature_evaluations)
            .ok_or(MetricError::EvaluationBudgetExceeded)?;
        Ok(())
    }
}

/// Shared scalar-oracle budget used by all numerical work in one metric pass.
/// Trajectory root isolation receives this same object so analytic polynomial
/// and enclosed non-polynomial paths consume the identical ledger.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct NumericalWorkBudget {
    pub(super) root_evaluations_remaining: u32,
    pub(super) quadrature_evaluations_remaining: u32,
}

impl NumericalWorkBudget {
    pub(super) const fn new(root_evaluations: u32, quadrature_evaluations: u32) -> Self {
        Self {
            root_evaluations_remaining: root_evaluations,
            quadrature_evaluations_remaining: quadrature_evaluations,
        }
    }

    pub(super) const fn from_limits(limits: MetricEvaluationLimits) -> Self {
        Self::new(
            limits.maximum_root_evaluations,
            limits.maximum_quadrature_evaluations,
        )
    }

    #[cfg(test)]
    pub(crate) const fn root_only(root_evaluations: u32) -> Self {
        Self::new(root_evaluations, 0)
    }

    pub(crate) fn cap_root_evaluations(&mut self, maximum: u32) {
        self.root_evaluations_remaining = self.root_evaluations_remaining.min(maximum);
    }

    pub(super) const fn from_work(work: NumericalWork) -> Self {
        Self::new(work.root_evaluations, work.quadrature_evaluations)
    }

    pub(super) fn work_since(self, later: Self) -> Result<NumericalWork, MetricError> {
        Ok(NumericalWork {
            root_evaluations: self
                .root_evaluations_remaining
                .checked_sub(later.root_evaluations_remaining)
                .ok_or(MetricError::NumericalFailure)?,
            quadrature_evaluations: self
                .quadrature_evaluations_remaining
                .checked_sub(later.quadrature_evaluations_remaining)
                .ok_or(MetricError::NumericalFailure)?,
        })
    }

    pub(super) fn ensure_available(self, work: NumericalWork) -> Result<(), MetricError> {
        if self.root_evaluations_remaining < work.root_evaluations
            || self.quadrature_evaluations_remaining < work.quadrature_evaluations
        {
            Err(MetricError::EvaluationBudgetExceeded)
        } else {
            Ok(())
        }
    }

    pub(crate) fn charge_root_evaluation(&mut self) -> Result<(), MetricError> {
        self.root_evaluations_remaining = self
            .root_evaluations_remaining
            .checked_sub(1)
            .ok_or(MetricError::EvaluationBudgetExceeded)?;
        Ok(())
    }

    pub(super) fn charge_quadrature_evaluations(
        &mut self,
        evaluations: u32,
    ) -> Result<(), MetricError> {
        self.quadrature_evaluations_remaining = self
            .quadrature_evaluations_remaining
            .checked_sub(evaluations)
            .ok_or(MetricError::EvaluationBudgetExceeded)?;
        Ok(())
    }
}

pub(super) fn live_metric_evaluation_limits(limits: LiveMetricLimits) -> MetricEvaluationLimits {
    let maximum_root_evaluations = u32::from(limits.max_root_evaluations);
    MetricEvaluationLimits {
        maximum_root_evaluations,
        maximum_quadrature_evaluations: maximum_root_evaluations
            .saturating_mul(LIVE_QUADRATURE_EVALUATIONS_PER_ROOT_CREDIT),
        ..MetricEvaluationLimits::default()
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Root {
    pub(super) parameter: f64,
    residual: f64,
}

#[derive(Clone, Copy)]
pub(super) struct Polynomial {
    pub(super) coefficient: [f64; 5],
    pub(super) degree: usize,
}

impl Polynomial {
    pub(super) fn evaluate(self, x: f64) -> f64 {
        let mut value = self.coefficient[self.degree];
        for index in (0..self.degree).rev() {
            value = value.mul_add(x, self.coefficient[index]);
        }
        value
    }

    pub(super) fn derivative(self) -> Self {
        let mut coefficient = [0.0; 5];
        if self.degree == 0 {
            return Self {
                coefficient,
                degree: 0,
            };
        }
        for index in 1..=self.degree {
            coefficient[index - 1] = self.coefficient[index] * index as f64;
        }
        Self {
            coefficient,
            degree: self.degree - 1,
        }
    }
}

#[cfg(test)]
pub(super) fn polynomial_roots(
    polynomial: Polynomial,
    interval: RangeInclusive<f64>,
    limits: MetricEvaluationLimits,
) -> Result<FixedVec<Root, MAX_ROOTS_PER_SEGMENT>, MetricError> {
    let mut budget = NumericalWorkBudget::from_limits(limits);
    polynomial_roots_with_budget(polynomial, interval, limits, &mut budget)
}

fn polynomial_roots_with_budget(
    polynomial: Polynomial,
    interval: RangeInclusive<f64>,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
) -> Result<FixedVec<Root, MAX_ROOTS_PER_SEGMENT>, MetricError> {
    let mut roots = FixedVec::new();
    isolate_polynomial(
        polynomial,
        *interval.start(),
        *interval.end(),
        limits,
        budget,
        &mut roots,
    )?;
    roots.sort_unstable_by(|left: &Root, right: &Root| left.parameter.total_cmp(&right.parameter));
    let mut index = 1;
    while index < roots.len() {
        if (roots[index].parameter - roots[index - 1].parameter).abs()
            <= limits.absolute_root_tolerance_s
        {
            let keep = if roots[index].residual.abs() < roots[index - 1].residual.abs() {
                roots[index]
            } else {
                roots[index - 1]
            };
            roots[index - 1] = keep;
            roots.remove(index);
        } else {
            index += 1;
        }
    }
    Ok(roots)
}

/// Bounded polynomial root seam used by live trajectory traversal. Every
/// scalar polynomial evaluation consumes the caller's shared root ledger.
#[allow(clippy::too_many_arguments)]
pub(crate) fn isolate_polynomial_coefficients_with_budget(
    coefficient: [f64; 5],
    degree: usize,
    lower: f64,
    upper: f64,
    x_tolerance: f64,
    value_tolerance: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<FixedVec<f64, MAX_ROOTS_PER_SEGMENT>, MetricError> {
    if degree > 4
        || lower > upper
        || !all_finite(&coefficient[..=degree])
        || !x_tolerance.is_finite()
        || !value_tolerance.is_finite()
        || x_tolerance <= 0.0
        || value_tolerance < 0.0
    {
        return Err(MetricError::InvalidDefinition);
    }
    let roots = polynomial_roots_with_budget(
        Polynomial {
            coefficient,
            degree,
        },
        lower..=upper,
        MetricEvaluationLimits {
            absolute_root_tolerance_s: x_tolerance,
            value_tolerance,
            ..MetricEvaluationLimits::default()
        },
        budget,
    )?;
    let mut parameters = FixedVec::new();
    for root in roots {
        parameters
            .push(root.parameter)
            .map_err(|_| MetricError::CapacityExceeded)?;
    }
    Ok(parameters)
}

fn isolate_polynomial(
    polynomial: Polynomial,
    lower: f64,
    upper: f64,
    limits: MetricEvaluationLimits,
    budget: &mut NumericalWorkBudget,
    roots: &mut FixedVec<Root, MAX_ROOTS_PER_SEGMENT>,
) -> Result<(), MetricError> {
    if polynomial.degree == 0 {
        return Ok(());
    }
    if polynomial.degree == 1 {
        let denominator = polynomial.coefficient[1];
        if denominator.abs() <= limits.value_tolerance {
            return Ok(());
        }
        let root = -polynomial.coefficient[0] / denominator;
        if root >= lower - limits.absolute_root_tolerance_s
            && root <= upper + limits.absolute_root_tolerance_s
        {
            let residual = evaluate_polynomial_with_budget(polynomial, root, budget)?;
            push_root(
                roots,
                Root {
                    parameter: root.clamp(lower, upper),
                    residual,
                },
                limits,
            )?;
        }
        return Ok(());
    }

    let derivative = polynomial.derivative();
    let critical = polynomial_roots_with_budget(
        derivative,
        lower..=upper,
        MetricEvaluationLimits {
            maximum_root_evaluations: limits.maximum_root_evaluations.saturating_sub(1),
            ..limits
        },
        budget,
    )?;
    let mut points = FixedVec::<f64, { MAX_ROOTS_PER_SEGMENT + 2 }>::new();
    points
        .push(lower)
        .map_err(|_| MetricError::CapacityExceeded)?;
    for root in &critical {
        if root.parameter > lower && root.parameter < upper {
            points
                .push(root.parameter)
                .map_err(|_| MetricError::CapacityExceeded)?;
        }
    }
    points
        .push(upper)
        .map_err(|_| MetricError::CapacityExceeded)?;
    points.sort_unstable_by(f64::total_cmp);

    for point in &points {
        let value = evaluate_polynomial_with_budget(polynomial, *point, budget)?;
        if value.abs() <= limits.value_tolerance {
            push_root(
                roots,
                Root {
                    parameter: *point,
                    residual: value,
                },
                limits,
            )?;
        }
    }
    for window in points.windows(2) {
        let left = window[0];
        let right = window[1];
        let f_left = evaluate_polynomial_with_budget(polynomial, left, budget)?;
        let f_right = evaluate_polynomial_with_budget(polynomial, right, budget)?;
        if f_left * f_right < 0.0 {
            let parameter = brent_with_budget(
                |x, _| Ok(polynomial.evaluate(x)),
                left,
                right,
                limits.absolute_root_tolerance_s,
                limits.value_tolerance,
                budget,
            )?;
            let residual = evaluate_polynomial_with_budget(polynomial, parameter, budget)?;
            push_root(
                roots,
                Root {
                    parameter,
                    residual,
                },
                limits,
            )?;
        }
    }
    Ok(())
}

fn evaluate_polynomial_with_budget(
    polynomial: Polynomial,
    parameter: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<f64, MetricError> {
    budget.charge_root_evaluation()?;
    Ok(polynomial.evaluate(parameter))
}

fn push_root(
    roots: &mut FixedVec<Root, MAX_ROOTS_PER_SEGMENT>,
    candidate: Root,
    limits: MetricEvaluationLimits,
) -> Result<(), MetricError> {
    if roots.iter().any(|present| {
        (present.parameter - candidate.parameter).abs() <= limits.absolute_root_tolerance_s
    }) {
        return Ok(());
    }
    roots
        .push(candidate)
        .map_err(|_| MetricError::CapacityExceeded)
}

/// Safeguarded Brent-Dekker root refinement. The caller must provide a valid
/// sign-changing bracket or an endpoint root.
#[cfg(test)]
pub(super) fn brent<F>(
    mut function: F,
    a: f64,
    b: f64,
    x_tolerance: f64,
    f_tolerance: f64,
    maximum_evaluations: u32,
) -> Result<f64, MetricError>
where
    F: FnMut(f64) -> f64,
{
    let mut budget = NumericalWorkBudget::root_only(maximum_evaluations);
    brent_with_budget(
        |parameter, _| Ok(function(parameter)),
        a,
        b,
        x_tolerance,
        f_tolerance,
        &mut budget,
    )
}

pub(super) fn brent_with_budget<F>(
    mut function: F,
    mut a: f64,
    mut b: f64,
    x_tolerance: f64,
    f_tolerance: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<f64, MetricError>
where
    F: FnMut(f64, &mut NumericalWorkBudget) -> Result<f64, MetricError>,
{
    budget.charge_root_evaluation()?;
    let mut fa = function(a, budget)?;
    budget.charge_root_evaluation()?;
    let mut fb = function(b, budget)?;
    if !fa.is_finite() || !fb.is_finite() {
        return Err(MetricError::NumericalFailure);
    }
    if fa.abs() <= f_tolerance {
        return Ok(a);
    }
    if fb.abs() <= f_tolerance {
        return Ok(b);
    }
    if fa * fb > 0.0 {
        return Err(MetricError::AmbiguousRoot);
    }
    if fa.abs() < fb.abs() {
        core::mem::swap(&mut a, &mut b);
        core::mem::swap(&mut fa, &mut fb);
    }

    let mut c = a;
    let mut fc = fa;
    let mut d = c;
    let mut bisected_last = true;
    loop {
        let mut s = if fa != fc && fb != fc {
            a * fb * fc / ((fa - fb) * (fa - fc))
                + b * fa * fc / ((fb - fa) * (fb - fc))
                + c * fa * fb / ((fc - fa) * (fc - fb))
        } else {
            b - fb * (b - a) / (fb - fa)
        };

        let lower_guard = (3.0 * a + b) * 0.25;
        let outside = if a < b {
            s <= lower_guard || s >= b
        } else {
            s >= lower_guard || s <= b
        };
        let insufficient = (bisected_last && (s - b).abs() >= (b - c).abs() * 0.5)
            || (!bisected_last && (s - b).abs() >= (c - d).abs() * 0.5)
            || (bisected_last && (b - c).abs() < x_tolerance)
            || (!bisected_last && (c - d).abs() < x_tolerance);
        if outside || insufficient || !s.is_finite() {
            s = (a + b) * 0.5;
            bisected_last = true;
        } else {
            bisected_last = false;
        }

        budget.charge_root_evaluation()?;
        let fs = function(s, budget)?;
        if !fs.is_finite() {
            return Err(MetricError::NumericalFailure);
        }
        d = c;
        c = b;
        fc = fb;
        if fa * fs < 0.0 {
            b = s;
            fb = fs;
        } else {
            a = s;
            fa = fs;
        }
        if fa.abs() < fb.abs() {
            core::mem::swap(&mut a, &mut b);
            core::mem::swap(&mut fa, &mut fb);
        }
        if fb.abs() <= f_tolerance || (b - a).abs() <= x_tolerance {
            return Ok(b);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct QuadratureResult {
    pub(super) value: f64,
    pub(super) absolute_error: f64,
    pub(super) evaluations: u32,
}

#[derive(Clone, Copy)]
struct QuadratureInterval {
    lower: f64,
    upper: f64,
    absolute_tolerance: f64,
}

/// Adaptive embedded Gauss-Kronrod 15/7 integration using a bounded explicit
/// stack rather than recursion.
pub(super) fn integrate<F>(
    function: F,
    lower: f64,
    upper: f64,
    absolute_tolerance: f64,
    relative_tolerance: f64,
    maximum_evaluations: u32,
) -> Result<QuadratureResult, MetricError>
where
    F: Fn(f64) -> f64,
{
    if lower == upper {
        return Ok(QuadratureResult {
            value: 0.0,
            absolute_error: 0.0,
            evaluations: 0,
        });
    }
    if !lower.is_finite()
        || !upper.is_finite()
        || !absolute_tolerance.is_finite()
        || !relative_tolerance.is_finite()
        || absolute_tolerance <= 0.0
        || relative_tolerance < 0.0
    {
        return Err(MetricError::InvalidDefinition);
    }

    let sign = if upper >= lower { 1.0 } else { -1.0 };
    let (lower, upper) = if sign > 0.0 {
        (lower, upper)
    } else {
        (upper, lower)
    };
    let mut stack = FixedVec::<QuadratureInterval, MAX_QUADRATURE_STACK>::new();
    stack
        .push(QuadratureInterval {
            lower,
            upper,
            absolute_tolerance,
        })
        .map_err(|_| MetricError::CapacityExceeded)?;
    let mut value = 0.0;
    let mut error = 0.0;
    let mut evaluations = 0u32;

    while let Some(interval) = stack.pop() {
        if evaluations.saturating_add(15) > maximum_evaluations {
            return Err(MetricError::EvaluationBudgetExceeded);
        }
        let local = gauss_kronrod_15(&function, interval.lower, interval.upper)?;
        evaluations += 15;
        let tolerance = interval
            .absolute_tolerance
            .max(relative_tolerance * local.value.abs());
        if local.absolute_error <= tolerance
            || (interval.upper - interval.lower) <= f64::EPSILON * 32.0
        {
            value += local.value;
            error += local.absolute_error;
        } else {
            let midpoint = (interval.lower + interval.upper) * 0.5;
            let half_tolerance = interval.absolute_tolerance * 0.5;
            stack
                .push(QuadratureInterval {
                    lower: midpoint,
                    upper: interval.upper,
                    absolute_tolerance: half_tolerance,
                })
                .map_err(|_| MetricError::CapacityExceeded)?;
            stack
                .push(QuadratureInterval {
                    lower: interval.lower,
                    upper: midpoint,
                    absolute_tolerance: half_tolerance,
                })
                .map_err(|_| MetricError::CapacityExceeded)?;
        }
    }
    Ok(QuadratureResult {
        value: sign * value,
        absolute_error: error,
        evaluations,
    })
}

pub(super) fn integrate_with_budget<F>(
    function: F,
    lower: f64,
    upper: f64,
    absolute_tolerance: f64,
    relative_tolerance: f64,
    budget: &mut NumericalWorkBudget,
) -> Result<QuadratureResult, MetricError>
where
    F: Fn(f64) -> f64,
{
    let result = integrate(
        function,
        lower,
        upper,
        absolute_tolerance,
        relative_tolerance,
        budget.quadrature_evaluations_remaining,
    )?;
    budget.charge_quadrature_evaluations(result.evaluations)?;
    Ok(result)
}

fn gauss_kronrod_15<F>(
    function: &F,
    lower: f64,
    upper: f64,
) -> Result<QuadratureResult, MetricError>
where
    F: Fn(f64) -> f64,
{
    const ABSCISSA: [f64; 8] = [
        0.991_455_371_120_812_6,
        0.949_107_912_342_758_5,
        0.864_864_423_359_769_1,
        0.741_531_185_599_394_5,
        0.586_087_235_467_691_1,
        0.405_845_151_377_397_2,
        0.207_784_955_007_898_48,
        0.0,
    ];
    const KRONROD_WEIGHT: [f64; 8] = [
        0.022_935_322_010_529_224,
        0.063_092_092_629_978_56,
        0.104_790_010_322_250_19,
        0.140_653_259_715_525_92,
        0.169_004_726_639_267_9,
        0.190_350_578_064_785_42,
        0.204_432_940_075_298_89,
        0.209_482_141_084_727_82,
    ];
    const GAUSS_WEIGHT: [f64; 4] = [
        0.129_484_966_168_869_7,
        0.279_705_391_489_276_64,
        0.381_830_050_505_118_9,
        0.417_959_183_673_469_4,
    ];

    let midpoint = (lower + upper) * 0.5;
    let half_width = (upper - lower) * 0.5;
    let centre = function(midpoint);
    if !centre.is_finite() {
        return Err(MetricError::NumericalFailure);
    }
    let mut kronrod = KRONROD_WEIGHT[7] * centre;
    let mut gauss = GAUSS_WEIGHT[3] * centre;
    for index in 0..7 {
        let offset = half_width * ABSCISSA[index];
        let left = function(midpoint - offset);
        let right = function(midpoint + offset);
        if !left.is_finite() || !right.is_finite() {
            return Err(MetricError::NumericalFailure);
        }
        let pair = left + right;
        kronrod += KRONROD_WEIGHT[index] * pair;
        match index {
            1 => gauss += GAUSS_WEIGHT[0] * pair,
            3 => gauss += GAUSS_WEIGHT[1] * pair,
            5 => gauss += GAUSS_WEIGHT[2] * pair,
            _ => {}
        }
    }
    let value = kronrod * half_width;
    Ok(QuadratureResult {
        value,
        absolute_error: ((kronrod - gauss) * half_width).abs(),
        evaluations: 15,
    })
}
