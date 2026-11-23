//! Convert a validated receiver measurement to the driving estimator's ECEF inputs.
use aevia_trajectory::engine::driving::DrivingFix;

pub fn prepare_gnss(n: crate::gnss::BestNav<'_>, position_us: u64) -> DrivingFix {
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
    DrivingFix {
        position_us,
        velocity_us: position_us.saturating_sub(u64::from(n.latency_ms) * 1000),
        position_ecef_m: position,
        velocity_ecef_mps: velocity,
        speed_sigma_mps: n.horizontal_speed_sigma_mm_s as f32 * 0.001,
        position_sigma_m: n.latitude_sigma_mm.max(n.longitude_sigma_mm) as f32 * 0.001,
    }
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
        let converted = prepare_gnss(fix(), 1_000_000);
        let position = converted.position_ecef_m;
        let velocity = converted.velocity_ecef_mps;
        assert!((position[0] - 6_378_177.0).abs() < 1e-8);
        assert_eq!(position[1], 0.0);
        assert_eq!(position[2], 0.0);
        assert!((velocity[0] - 2.0).abs() < 1e-12);
        assert!((velocity[1] - 10.0).abs() < 1e-12);
        assert!(velocity[2].abs() < 1e-12);
        assert_eq!(converted.position_us, 1_000_000);
        assert_eq!(converted.velocity_us, 950_000);
        assert_eq!(converted.position_sigma_m, 2.0);
        assert_eq!(converted.speed_sigma_mps, 1.0);
    }
    #[test]
    fn southern_fix_rotates_velocity_without_changing_speed() {
        let mut nav = fix();
        nav.latitude_e7 = -338_500_000;
        nav.longitude_e7 = 1_512_000_000;
        let converted = prepare_gnss(nav, 1_000_000);
        let position = converted.position_ecef_m;
        let velocity = converted.velocity_ecef_mps;
        assert!(position[0] < 0.0 && position[1] > 0.0 && position[2] < 0.0);
        let speed2 = velocity.into_iter().map(|v| v * v).sum::<f64>();
        assert!((speed2 - 104.0).abs() < 1e-10);
    }
}
