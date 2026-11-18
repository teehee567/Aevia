//! Preserve BESTNAV ellipsoid height, ENU velocity and axis-specific covariance.
use aevia_trajectory::uncertainty::Covariance3;

pub fn prepare_gnss(
    n: crate::gnss::BestNav<'_>,
) -> Result<([f64; 3], [f64; 3], Covariance3, Covariance3), aevia_trajectory::ValidationError> {
    let lat = f64::from(n.latitude_e7) / 1e7 * core::f64::consts::PI / 180.0;
    let lon = f64::from(n.longitude_e7) / 1e7 * core::f64::consts::PI / 180.0;
    let (sl, cl, so, co) = (
        libm::sin(lat),
        libm::cos(lat),
        libm::sin(lon),
        libm::cos(lon),
    );
    let h = (f64::from(n.height_mm) + f64::from(n.undulation_mm)) / 1000.0;
    let radius = 6378137.0 / libm::sqrt(1.0 - 0.0066943799901413165 * sl * sl);
    let position = [
        (radius + h) * cl * co,
        (radius + h) * cl * so,
        (radius * (1.0 - 0.0066943799901413165) + h) * sl,
    ];
    let heading = f64::from(n.track_hundredths_deg) / 100.0 * core::f64::consts::PI / 180.0;
    let horizontal = f64::from(n.horizontal_speed_mm_s) / 1000.0;
    let enu = [
        horizontal * libm::sin(heading),
        horizontal * libm::cos(heading),
        f64::from(n.vertical_speed_mm_s) / 1000.0,
    ];
    let rotation = [
        [-so, -sl * co, cl * co],
        [co, -sl * so, cl * so],
        [0.0, cl, sl],
    ];
    let velocity = rotation.map(|row| row[0] * enu[0] + row[1] * enu[1] + row[2] * enu[2]);
    fn rotated(
        r: [[f64; 3]; 3],
        sigma: [f64; 3],
    ) -> Result<Covariance3, aevia_trajectory::ValidationError> {
        let mut c = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in i..3 {
                for k in 0..3 {
                    c[i][j] += r[i][k] * r[j][k] * sigma[k] * sigma[k];
                }
                c[j][i] = c[i][j];
            }
        }
        Covariance3::from_matrix(c)
    }
    let pcov = rotated(
        rotation,
        [
            f64::from(n.longitude_sigma_mm).max(20.0) / 1000.0,
            f64::from(n.latitude_sigma_mm).max(20.0) / 1000.0,
            f64::from(n.height_sigma_mm).max(20.0) / 1000.0,
        ],
    )?;
    let vcov = rotated(
        rotation,
        [
            f64::from(n.horizontal_speed_sigma_mm_s).max(30.0) / 1000.0,
            f64::from(n.horizontal_speed_sigma_mm_s).max(30.0) / 1000.0,
            f64::from(n.vertical_speed_sigma_mm_s).max(30.0) / 1000.0,
        ],
    )?;
    Ok((position, velocity, pcov, vcov))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fix() -> crate::gnss::BestNav<'static> {
        crate::gnss::BestNav {
            gps_week: 2300,
            gps_tow_ms: 1000,
            latitude_e7: 0,
            longitude_e7: 0,
            height_mm: 10_000,
            undulation_mm: 30_000,
            latitude_sigma_mm: 2000,
            longitude_sigma_mm: 1000,
            height_sigma_mm: 3000,
            satellites_tracked: 20,
            satellites_used: 15,
            position_type: b"SINGLE",
            velocity_type: b"DOPPLER_VELOCITY",
            latency_ms: 50,
            horizontal_speed_mm_s: 10_000,
            track_hundredths_deg: 9000,
            vertical_speed_mm_s: 2000,
            vertical_speed_sigma_mm_s: 3000,
            horizontal_speed_sigma_mm_s: 1000,
        }
    }
    #[test]
    fn equator_uses_ellipsoid_height_and_east_north_up_axes() {
        let (position, velocity, pcov, vcov) = prepare_gnss(fix()).unwrap();
        assert!((position[0] - 6_378_177.0).abs() < 1e-8);
        assert_eq!(position[1], 0.0);
        assert_eq!(position[2], 0.0);
        assert!((velocity[0] - 2.0).abs() < 1e-12);
        assert!((velocity[1] - 10.0).abs() < 1e-12);
        assert!(velocity[2].abs() < 1e-12);
        assert_eq!(pcov.upper_triangle(), [9.0, 0.0, 0.0, 1.0, 0.0, 4.0]);
        assert_eq!(vcov.upper_triangle(), [9.0, 0.0, 0.0, 1.0, 0.0, 1.0]);
    }
    #[test]
    fn southern_fix_rotates_anisotropic_covariance_without_losing_trace() {
        let mut nav = fix();
        nav.latitude_e7 = -338_500_000;
        nav.longitude_e7 = 1_512_000_000;
        let (position, velocity, pcov, _) = prepare_gnss(nav).unwrap();
        assert!(position[0] < 0.0 && position[1] > 0.0 && position[2] < 0.0);
        let speed2 = velocity.into_iter().map(|v| v * v).sum::<f64>();
        assert!((speed2 - 104.0).abs() < 1e-10);
        let matrix = pcov.to_matrix();
        assert!((matrix[0][0] + matrix[1][1] + matrix[2][2] - 14.0).abs() < 1e-10);
        assert!(matrix[0][1].abs() > 0.1);
        assert_eq!(matrix[1][0], matrix[0][1]);
    }
    #[test]
    fn receiver_zero_sigmas_do_not_claim_exact_position_or_velocity() {
        let mut nav = fix();
        nav.longitude_sigma_mm = 0;
        nav.latitude_sigma_mm = 0;
        nav.height_sigma_mm = 0;
        nav.horizontal_speed_sigma_mm_s = 0;
        nav.vertical_speed_sigma_mm_s = 0;
        let (_, _, pcov, vcov) = prepare_gnss(nav).unwrap();
        for axis in 0..3 {
            assert!(pcov.variance(axis).unwrap() > 0.0);
            assert!(vcov.variance(axis).unwrap() > 0.0);
        }
    }
}
