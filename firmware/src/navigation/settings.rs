//! Explicit bench assumptions, to replace with measured installation calibration.
//! Body axes are forward, left, up; quaternion is scalar-first sensor-to-body.
pub const BODY_FROM_IMU_WXYZ: [f64; 4] = [1.0, 0.0, 0.0, 0.0];
pub const IMU_TO_ANTENNA_METRES: [f64; 3] = [0.0, 0.0, 0.0];
pub const IMU_TIMING_SIGMA_NS: u64 = 1_200_000;
pub const QUALIFIED: bool = false;

#[derive(serde::Serialize)]
pub struct Settings {
    pub body_from_imu_wxyz: [f64; 4],
    pub imu_to_antenna_metres: [f64; 3],
    pub imu_timing_sigma_ns: u64,
    pub qualified: bool,
}
pub fn snapshot() -> Settings {
    Settings {
        body_from_imu_wxyz: BODY_FROM_IMU_WXYZ,
        imu_to_antenna_metres: IMU_TO_ANTENNA_METRES,
        imu_timing_sigma_ns: IMU_TIMING_SIGMA_NS,
        qualified: QUALIFIED,
    }
}
