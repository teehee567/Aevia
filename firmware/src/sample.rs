//! Sensor telemetry shared by acquisition, USB and the recording codec.
use serde::Serialize;

#[derive(Clone, Copy, Serialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Sample {
    Gnss {
        t_us: u64,
        gps_week: u32,
        gps_tow_ms: u32,
        lat: f64,
        lon: f64,
        alt_m: f64,
        speed_mps: f64,
        heading_deg: f64,
        vertical_mps: f64,
        hacc_m: f64,
        vacc_m: f64,
        sacc_mps: f64,
        latency_ms: u32,
        satellites: u8,
        valid: bool,
        mapped_ns: Option<u64>,
        mapping_uncertainty_ns: Option<u64>,
        mapping_generation: Option<u32>,
    },
    Imu {
        t_us: u64,
        dt_us: u32,
        accel: [f64; 3],
        gyro: [f64; 3],
        valid: bool,
        captured_ns: u64,
        interval_ns: u32,
        effective_end_ns: u64,
        timing_uncertainty_ns: u64,
        aligned_gyro: [f64; 3],
    },
}
