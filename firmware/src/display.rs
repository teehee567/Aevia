//! Postcard payload for the negotiated display_v1 snapshot. The app shares
//! this definition. Field order and types are part of the wire format.
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct DisplayFrame {
    pub uptime_us: u64,
    pub session: u32,
    pub recording: bool,
    pub stopped_at_us: Option<u64>,
    pub battery_percent: Option<u8>,
    pub gnss_count: u32,
    pub rejected_fix_count: u32,
    pub fix: Option<DisplayFix>,
    pub imu: Option<DisplayAcceleration>,
}

#[derive(Serialize, Deserialize)]
pub struct DisplayFix {
    pub t_us: u64,
    pub gps_week: u32,
    pub gps_tow_ms: u32,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f32,
    pub speed_mps: f32,
    pub heading_deg: f32,
    pub vertical_mps: f32,
    pub hacc_m: f32,
    pub latency_ms: u32,
    pub satellites: u8,
    pub valid: bool,
}

#[derive(Serialize, Deserialize)]
pub struct DisplayAcceleration {
    pub t_us: u64,
    pub accel: [f32; 3],
    pub valid: bool,
}
