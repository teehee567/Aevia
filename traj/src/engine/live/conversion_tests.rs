//! Regression tests for imu covariance conversion tests.

use super::covariance_density;
use crate::uncertainty::Covariance3;

#[test]
fn covariance_density_conversion_preserves_cross_axis_terms() {
    let source = Covariance3::from_matrix([
        [4.0e-6, 1.0e-6, 0.0],
        [1.0e-6, 3.0e-6, -0.5e-6],
        [0.0, -0.5e-6, 2.0e-6],
    ])
    .unwrap();
    let converted = covariance_density(source).unwrap();
    assert_eq!(converted[(0, 1)], 1.0e-6_f32);
    assert_eq!(converted[(1, 0)], 1.0e-6_f32);
    assert_eq!(converted[(1, 2)], -0.5e-6_f32);
    assert_eq!(converted[(2, 1)], -0.5e-6_f32);

    let underflow = Covariance3::diagonal(f64::MIN_POSITIVE, 0.0, 0.0).unwrap();
    assert!(covariance_density(underflow).is_err());
}
