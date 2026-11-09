//! Transport-independent application commands and latest-value telemetry.
//!
//! USB, and later radio adapters, share these types. Reading telemetry never
//! locks the estimator or puts backpressure on sensor acquisition.

use core::cell::Cell;
#[cfg(target_arch = "riscv32")]
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use serde::{Deserialize, Serialize};

#[cfg(target_arch = "riscv32")]
use super::sensors::{GNSS_DROPS, IMU_DROPS};

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub enum Phase {
    #[default]
    Starting,
    Initializing,
    Navigating,
    Degraded,
    Failed,
}

/// Last estimator failure, retained across later successful observations.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum EngineError {
    Definition,
    Profile,
    Frame,
    Calibration,
    Capability,
    Resources,
    Workspace,
    Qualification,
    Evidence,
    Lifecycle,
    Sequence,
    Observation,
    Clock,
    Estimator,
    OutputCapacity,
    Metric,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Snapshot {
    pub updated_us: u64,
    pub phase: Phase,
    /// Factory-scaled estimates with nominal filter delay are unqualified.
    pub qualified: bool,
    pub epoch_us: Option<u64>,
    pub horizontal_speed_mps: Option<f32>,
    pub vertical_speed_mps: Option<f32>,
    pub position_ecef_m: Option<[f64; 3]>,
    pub velocity_ecef_mps: Option<[f32; 3]>,
    pub acceleration_mps2: [f32; 3],
    pub angular_rate_rps: [f32; 3],
    pub satellites: u8,
    pub imu_rate_hz: f32,
    pub gnss_rate_hz: f32,
    pub imu_samples: u32,
    pub gnss_samples: u32,
    pub imu_errors: u32,
    pub gnss_errors: u32,
    pub gnss_uart_errors: u32,
    pub gnss_parse_errors: u32,
    pub last_imu_us: u64,
    pub last_gnss_us: u64,
    pub imu_received: u64,
    pub gnss_received: u64,
    pub imu_accepted: u64,
    pub gnss_fused: u64,
    pub input_rejected: u64,
    pub too_late: u64,
    pub processing_errors: u32,
    pub error_kind: Option<EngineError>,
    pub maximum_step_us: u32,
    pub imu_drops: u32,
    pub gnss_drops: u32,
    pub timing: super::timing::TimingStatus,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Indicators {
    pub display_enabled: bool,
    pub led_brightness: u8,
}

impl Default for Indicators {
    fn default() -> Self {
        Self {
            display_enabled: true,
            led_brightness: 32,
        }
    }
}

/// Secondary application commands; loader/update commands live in `commands`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum DeviceCommand {
    Status,
    Latest,
    SetIndicators(Indicators),
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum DeviceResponse {
    Snapshot(Snapshot),
    Indicators(Indicators),
}

static SNAPSHOT: Mutex<CriticalSectionRawMutex, Cell<Option<Snapshot>>> =
    Mutex::new(Cell::new(None));
static INDICATORS: Mutex<CriticalSectionRawMutex, Cell<Indicators>> =
    Mutex::new(Cell::new(Indicators {
        display_enabled: true,
        led_brightness: 32,
    }));

pub fn snapshot() -> Snapshot {
    let value = SNAPSHOT.lock(Cell::get).unwrap_or_default();
    #[cfg(target_arch = "riscv32")]
    let mut value = Snapshot {
        imu_drops: IMU_DROPS.load(Ordering::Relaxed),
        gnss_drops: GNSS_DROPS.load(Ordering::Relaxed),
        timing: super::timing::status(),
        ..value
    };
    #[cfg(target_arch = "riscv32")]
    if embassy_time::Instant::now()
        .as_micros()
        .saturating_sub(value.updated_us)
        > 500_000
    {
        if !matches!(value.phase, Phase::Failed | Phase::Starting) {
            value.phase = Phase::Degraded;
        }
        value.epoch_us = None;
        value.horizontal_speed_mps = None;
        value.vertical_speed_mps = None;
        value.position_ecef_m = None;
        value.velocity_ecef_mps = None;
    }
    value
}

pub fn indicators() -> Indicators {
    INDICATORS.lock(Cell::get)
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn publish(snapshot: Snapshot) {
    SNAPSHOT.lock(|latest| latest.set(Some(snapshot)));
}

pub fn command(command: DeviceCommand) -> DeviceResponse {
    match command {
        DeviceCommand::Status | DeviceCommand::Latest => DeviceResponse::Snapshot(snapshot()),
        DeviceCommand::SetIndicators(value) => {
            INDICATORS.lock(|current| current.set(value));
            DeviceResponse::Indicators(value)
        }
    }
}
