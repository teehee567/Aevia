//! Shared Postcard schemas for driving snapshots and legacy USB decoding.
//! Field order and types are part of the wire format.
use serde::{Deserialize, Serialize};

pub const MAX_DRIVING_FRAME_BYTES: usize = 208;
pub const BATTERY_TELEMETRY_TAG: &[u8; 4] = b"BAT1";
pub const THERMAL_TELEMETRY_TAG: &[u8; 4] = b"BAT2";
pub const MOTION_SOURCE_TAG: &[u8; 4] = b"BAT3";
pub const IMU_CLIPPING_TAG: &[u8; 4] = b"BAT4";
pub const IMU_CLIPPING_BIT: u8 = 0x80;
pub const MAX_BLE_FRAME_BYTES: usize =
    MAX_DRIVING_FRAME_BYTES + BATTERY_TELEMETRY_TAG.len() + 20 + 3 + 1;

/// Whole Celsius, biased by 55. Values outside the wire range are unavailable.
pub fn pack_temperature(value: Option<i16>) -> u8 {
    value
        .filter(|value| (-55..=199).contains(value))
        .map_or(255, |value| (value + 55) as u8)
}

pub fn unpack_temperature(value: u8) -> Option<i16> {
    (value != 255).then(|| i16::from(value) - 55)
}

/// Optional BLE battery trailer. Enum order is on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryState {
    Unknown,
    Charging,
    Bypass,
    Battery,
    Low,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PowerSource {
    Battery,
    External,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChargeStage {
    NotCharging,
    ConstantCurrent,
    Taper,
    TopOff,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TemperatureZone {
    Normal,
    ColdOrUnpowered,
    Hot,
    Cool,
    Warm,
    Precool,
    Prewarm,
    BiasFault,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct BatteryTelemetry {
    pub status: BatteryState,
    pub percent: Option<u8>,
    pub millivolts: Option<u16>,
    pub rate_percent_per_hour: Option<f32>,
    pub charge_stage: Option<ChargeStage>,
    pub power_source: Option<PowerSource>,
    pub temperature_zone: Option<TemperatureZone>,
}

#[cfg(not(target_os = "none"))]
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

#[cfg(not(target_os = "none"))]
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

#[cfg(not(target_os = "none"))]
#[derive(Serialize, Deserialize)]
pub struct DisplayAcceleration {
    pub t_us: u64,
    pub accel: [f32; 3],
    pub valid: bool,
}

/// Compact driving telemetry v2. The measurement epoch is on the board clock.
#[derive(Clone, Copy, Serialize, Deserialize, Debug, Default)]
pub struct Motion {
    pub t_us: u64,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f32,
    pub speed_mps: f32,
    pub course_deg: Option<f32>,
    pub heading_deg: Option<f32>,
    pub vehicle_g: Option<[f32; 3]>,
    pub degraded: bool,
}

/// Live-only metadata. On-wire values are independent of SD DrivingFrame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum MotionSource {
    Waiting = 0,
    Fused = 1,
    PositionDegraded = 2,
    Inertial = 3,
    Gnss = 4,
    Held = 5,
}

#[cfg(not(target_os = "none"))]
impl MotionSource {
    pub fn from_wire(value: u8) -> Option<Self> {
        Some(match value {
            0 => Self::Waiting,
            1 => Self::Fused,
            2 => Self::PositionDegraded,
            3 => Self::Inertial,
            4 => Self::Gnss,
            5 => Self::Held,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug, Default)]
pub struct Timing {
    pub distance_m: f32,
    pub average_mps: Option<f32>,
    pub peak_mps: Option<f32>,
    pub laps: u32,
    pub lap_start_us: Option<u64>,
    pub last_lap_end_us: Option<u64>,
    pub last_lap_s: Option<f32>,
    pub best_lap_s: Option<f32>,
    pub sector: u8,
    pub last_sector_s: Option<f32>,
    pub launch_us: Option<u64>,
    pub timing_origin_us: Option<u64>,
    pub speed_time_s: Option<f32>,
    pub distance_time_s: Option<f32>,
}
#[derive(Serialize, Deserialize, Debug)]
pub struct DrivingFrame {
    pub uptime_us: u64,
    pub session: u32,
    pub recording: bool,
    pub started_at_us: Option<u64>,
    pub stopped_at_us: Option<u64>,
    pub battery_percent: Option<u8>,
    pub gnss_count: u32,
    pub imu_count: u32,
    pub satellites: Option<u8>,
    pub fix_valid: bool,
    pub mounted: bool,
    pub motion: Option<Motion>,
    pub timing: Timing,
}

/// Integer wire units also allow configuration on the acquisition core.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
pub struct MapGate {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub heading_cdeg: u16,
    pub width_cm: u16,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum Configure {
    Mount {
        confirmed: bool,
    },
    Begin {
        start_centi_kph: u32,
        end_centi_kph: u32,
        distance_mm: u32,
        rollout: bool,
        mounted: bool,
        gates: u8,
    },
    Gate {
        index: u8,
        gate: MapGate,
    },
}

#[cfg(target_os = "none")]
impl Timing {
    pub const EMPTY: Self = Self {
        distance_m: 0.,
        average_mps: None,
        peak_mps: None,
        laps: 0,
        lap_start_us: None,
        last_lap_end_us: None,
        last_lap_s: None,
        best_lap_s: None,
        sector: 0,
        last_sector_s: None,
        launch_us: None,
        timing_origin_us: None,
        speed_time_s: None,
        distance_time_s: None,
    };
}
