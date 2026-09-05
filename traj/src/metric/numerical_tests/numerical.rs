//! Metric numerical regression tests.

#[cfg(test)]
use super::super::numerical::{brent, polynomial_roots};
use super::super::{
    definition::LaunchRule,
    events::find_launch,
    numerical::{MetricEvaluationLimits, NumericalWorkBudget, Polynomial, integrate},
    report::MetricError,
};
use super::support::{eastbound_trajectory, limits};
use crate::{ids::ReferencePointId, time::DurationNs};

#[test]
fn polynomial_isolator_finds_tangent_and_crossing_roots() {
    // (x - 0.25)^2 (x - 0.75)
    let polynomial = Polynomial {
        coefficient: [-0.046_875, 0.4375, -1.25, 1.0, 0.0],
        degree: 3,
    };
    let roots = polynomial_roots(polynomial, 0.0..=1.0, limits()).unwrap();
    assert_eq!(roots.len(), 2);
    assert!((roots[0].parameter - 0.25).abs() < 1.0e-8);
    assert!((roots[1].parameter - 0.75).abs() < 1.0e-8);
    assert!(polynomial.derivative().evaluate(roots[0].parameter).abs() < 1.0e-7);
}

#[test]
fn brent_requires_a_bracket_and_converges() {
    let root = brent(|x| x * x - 2.0, 0.0, 2.0, 1.0e-14, 1.0e-14, 100).unwrap();
    assert!((root - 2.0_f64.sqrt()).abs() < 1.0e-12);
    assert_eq!(
        brent(|x| x * x + 1.0, -1.0, 1.0, 1.0e-12, 1.0e-12, 100),
        Err(MetricError::AmbiguousRoot)
    );
}

#[test]
fn adaptive_quadrature_integrates_polynomial_and_reversed_interval() {
    let result = integrate(|x| x.powi(4), 0.0, 2.0, 1.0e-12, 1.0e-12, 1_000).unwrap();
    assert!((result.value - 6.4).abs() < 1.0e-12);
    assert!(result.absolute_error < 1.0e-11);
    let reversed = integrate(|x| x * x, 2.0, 0.0, 1.0e-12, 1.0e-12, 1_000).unwrap();
    assert!((reversed.value + 8.0 / 3.0).abs() < 1.0e-12);
}

#[test]
fn acceleration_launch_scan_consumes_the_live_scalar_budget() {
    let trajectory = eastbound_trajectory(10.0, 10.0, 10.0);
    let mut budget = NumericalWorkBudget::root_only(1);
    assert_eq!(
        find_launch(
            &trajectory,
            ReferencePointId::new(1),
            LaunchRule::AccelerationChangePoint {
                minimum_acceleration_mps2: 0.0,
                dwell: DurationNs::from_ns(10_000_000),
            },
            MetricEvaluationLimits::default(),
            &mut budget,
        ),
        Err(MetricError::EvaluationBudgetExceeded)
    );
}
