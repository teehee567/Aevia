//! Regression tests for imu covariance conversion tests.

use super::covariance_density;
use crate::uncertainty::Covariance3;

#[test]
fn rotated_receiver_covariance_remains_exactly_symmetric() {
    use crate::frame::ReferenceEllipsoid;
    use crate::live::EcefAnchor;
    use nalgebra::{Matrix3, Vector3};
    let latitude = -35.9011846_f64.to_radians();
    let longitude = 149.1616646_f64.to_radians();
    let (slat, clat) = (libm::sin(latitude), libm::cos(latitude));
    let (slon, clon) = (libm::sin(longitude), libm::cos(longitude));
    let radius = 6_378_137.0 / libm::sqrt(1.0 - 6.694_379_990_141_316_5e-3 * slat * slat);
    let origin = Vector3::new(
        (radius + 720.0) * clat * clon,
        (radius + 720.0) * clat * slon,
        (radius * (1.0 - 6.694_379_990_141_316_5e-3) + 720.0) * slat,
    );
    let anchor = EcefAnchor::from_origin(0, origin, ReferenceEllipsoid::WGS84).unwrap();
    let basis = Matrix3::new(
        -slon,
        -slat * clon,
        clat * clon,
        clon,
        -slat * slon,
        clat * slon,
        0.0,
        clat,
        slat,
    );
    let ecef =
        basis * Matrix3::from_diagonal(&Vector3::new(0.0004, 0.0004, 0.0025)) * basis.transpose();
    let source = Covariance3::from_upper_triangle([
        ecef[(0, 0)],
        ecef[(0, 1)],
        ecef[(0, 2)],
        ecef[(1, 1)],
        ecef[(1, 2)],
        ecef[(2, 2)],
    ])
    .unwrap();
    let inflated = super::scale_covariance(
        super::add_matrix(
            source.to_matrix(),
            Covariance3::diagonal(0.0004, 0.0004, 0.0004)
                .unwrap()
                .to_matrix(),
        ),
        5.0,
    )
    .unwrap();
    let converted = super::rotate_covariance_to_n(&anchor, inflated).unwrap();
    assert_eq!(
        converted,
        converted.transpose(),
        "GNSS covariance must pass strict symmetric measurement validation"
    );
}

#[test]
fn rotated_navigation_covariance_can_be_published_away_from_equator() {
    use crate::{
        frame::ReferenceEllipsoid,
        live::{DenseCovariance, EcefAnchor},
    };
    use nalgebra::{Matrix3, Vector3};
    let anchor = EcefAnchor::from_origin(
        0,
        Vector3::new(-4_433_000.0, 2_640_000.0, -3_720_000.0),
        ReferenceEllipsoid::WGS84,
    )
    .unwrap();
    let covariance = DenseCovariance {
        position: Matrix3::from_diagonal(&Vector3::new(0.001, 0.002, 0.003)),
        velocity: Matrix3::from_diagonal(&Vector3::new(0.004, 0.005, 0.006)),
        position_velocity: Matrix3::zeros(),
        attitude: Matrix3::identity() * 0.01,
    };
    assert!(
        super::kinematic_covariance(covariance, &anchor).is_ok(),
        "valid computed navigation covariance must publish without resetting navigation"
    );
}

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
