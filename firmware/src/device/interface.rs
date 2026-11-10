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
    /// Last input arrival processed by the worker, including rejected inputs.
    /// Navigation freshness is determined by epoch_us, never this timestamp.
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
            display_enabled: false,
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
    /// Read back the temporary battery-free bench charging policy.
    PowerStatus,
    /// Low-rate fitted peripheral health and read-only evaluation data.
    PeripheralStatus,
    /// Acquisition health independent of estimator initialization or failure.
    AcquisitionStatus,
    /// Receiver configuration and trusted-time diagnostics before PPS lock.
    GnssStatus,
    /// Retained IMU initialization failure details.
    ImuDiagnostics,
    /// Reset reason and checksum-verified panic from the preceding boot.
    ResetStatus,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum DeviceResponse {
    Snapshot(Snapshot),
    Indicators(Indicators),
    PowerStatus(PowerStatus),
    PeripheralStatus(PeripheralStatus),
    AcquisitionStatus(AcquisitionStatus),
    GnssStatus(GnssStatus),
    ImuDiagnostics(ImuDiagnostics),
    ResetStatus(ResetStatus),
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ResetStatus {
    /// ESP32-S31 reset reason numeric value, as reported by the pinned HAL.
    pub reset_reason: Option<u8>,
    pub panic: Option<PanicRecord>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct PanicRecord {
    /// FNV-1a of PanicInfo::location().file(), without changing the file path.
    pub file_hash: u32,
    pub line: u32,
    pub column: u32,
    pub core: u8,
    pub startup_stage: u8,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct GnssStatus {
    pub config_verified: bool,
    pub config_queries: u32,
    pub config_timeouts: u32,
    pub navigation_time_ready: bool,
    pub last_pps_quality: Option<u32>,
    /// Bounded evidence for the last parser failure; no errors are suppressed.
    pub last_parser_error: Option<GnssParserError>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct GnssParserError {
    /// Receiver COM1=1, COM2=2.
    pub port: u8,
    /// BufferTooSmall=1, Oversized=2, Length=3, Sync=4, Checksum=5,
    /// InvalidAscii=6, MessageType=7.
    pub code: u8,
    /// Present when CRC-valid binary framing failed typed-message decoding.
    pub message_id: Option<u16>,
    pub received_us: u64,
    /// Number of valid chronological bytes across the two 32-byte arrays.
    pub context_len: u8,
    pub context: [[u8; 32]; 2],
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ImuDiagnostics {
    pub attempts: u32,
    pub ready: bool,
    pub error_kind: u8,
    pub register: Option<u8>,
    pub expected: Option<u16>,
    pub actual: Option<u16>,
    pub component: Option<u16>,
    pub asic: Option<u16>,
    pub status_registers: Option<[u16; 10]>,
    pub status_frame_faults: u16,
    pub driver_errors: u32,
    pub timing_errors: u32,
    pub gap_errors: u32,
    pub range_errors: u32,
    pub drdy_timeouts: u32,
    pub spi_timeouts: u32,
    pub spi_last_phase: u8,
    pub spi_last_context: u8,
    pub spi_last_elapsed_us: u32,
    pub spi_max_elapsed_us: u32,
    pub spi_last_cmd: u32,
    pub spi_last_clock: u32,
    pub spi_last_source: u32,
    pub timing_late_starts: u32,
    pub timing_late_finishes: u32,
    pub timing_overwritten: u32,
    pub max_start_latency_us: u32,
    pub max_burst_us: u32,
    pub last_start_latency_us: u32,
    pub last_burst_us: u32,
    pub cycle_clock_active: bool,
    pub cycle_clock_hz: u32,
    /// Numeric policy rejections, separate from sensor-reported saturation.
    pub numeric_range_errors: u32,
    /// DRDY capture time of the last numeric policy rejection.
    pub last_numeric_range_at_ns: Option<u64>,
    pub last_numeric_rate_abs_dps: Option<[f64; 3]>,
    pub last_numeric_accel_abs_mps2: Option<[f64; 3]>,
    /// Last sensor-status failure, retained across other failures and success.
    pub last_sensor_status_at_ns: Option<u64>,
    pub last_sensor_status_register: Option<u8>,
    /// SensorStatus discriminant shifted left once, with common_error in bit 0.
    pub last_sensor_status_flags: Option<u8>,
    /// Unmodified unsigned 20-bit sensor payload and four-bit sample counter.
    pub last_sensor_status_payload: Option<u32>,
    pub last_sensor_status_counter: Option<u8>,
}

fn gnss_status() -> GnssStatus {
    #[cfg(target_arch = "riscv32")]
    {
        super::gnss::status()
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        GnssStatus::default()
    }
}

fn imu_diagnostics() -> ImuDiagnostics {
    #[cfg(target_arch = "riscv32")]
    {
        super::imu::diagnostics()
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        ImuDiagnostics::default()
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct AcquisitionStatus {
    pub updated_us: u64,
    pub last_imu_us: Option<u64>,
    pub last_gnss_us: Option<u64>,
    pub imu_samples: u32,
    pub gnss_samples: u32,
    pub imu_errors: u32,
    pub gnss_errors: u32,
    pub gnss_uart_errors: u32,
    pub gnss_parse_errors: u32,
    pub gnss_rate_hz: u32,
    pub imu_drops: u32,
    pub gnss_drops: u32,
    pub imu_queue_depth: u16,
    pub gnss_queue_depth: u16,
    pub imu_queue_high_water: u16,
    pub gnss_queue_high_water: u16,
    /// Worst observed DRDY edge to completed burst, including scheduling delay.
    pub imu_max_read_latency_us: u32,
    pub timing: super::timing::TimingStatus,
}

pub fn acquisition_status() -> AcquisitionStatus {
    #[cfg(target_arch = "riscv32")]
    {
        super::sensors::acquisition_status()
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        AcquisitionStatus::default()
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct PeripheralStatus {
    pub updated_us: u64,
    pub leds_ready: bool,
    pub display_ready: bool,
    pub led_errors: u32,
    pub display_errors: u32,
    pub gauge_errors: u32,
    pub buttons_errors: u32,
    pub battery_voltage_uv: Option<u32>,
    pub battery_soc_percent: Option<f32>,
    /// Debounced bit mask in HAL board button order. No button commands enabled.
    pub buttons_pressed: Option<u8>,
}

static PERIPHERALS: Mutex<CriticalSectionRawMutex, Cell<Option<PeripheralStatus>>> =
    Mutex::new(Cell::new(None));

// Display health has a separate publication clock: an active power-bus owner
// must not keep a stalled renderer's readiness alive (or vice versa).
#[cfg(target_arch = "riscv32")]
static DISPLAY: Mutex<CriticalSectionRawMutex, Cell<(bool, u32, u64)>> =
    Mutex::new(Cell::new((false, 0, 0)));

pub fn peripheral_status() -> PeripheralStatus {
    let value = PERIPHERALS.lock(Cell::get).unwrap_or_default();
    #[cfg(target_arch = "riscv32")]
    {
        let now = embassy_time::Instant::now().as_micros();
        value.with_display(DISPLAY.lock(Cell::get), now)
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        value
    }
}

impl PeripheralStatus {
    #[cfg(any(target_arch = "riscv32", test))]
    fn with_display(self, (ready, errors, updated_us): (bool, u32, u64), now_us: u64) -> Self {
        let mut value = self.at(now_us);
        value.display_ready = ready && now_us.saturating_sub(updated_us) <= 3_000_000;
        value.display_errors = errors;
        value
    }

    #[cfg(any(target_arch = "riscv32", test))]
    fn at(mut self, now_us: u64) -> Self {
        if now_us.saturating_sub(self.updated_us) > 3_000_000 {
            self.leds_ready = false;
            self.display_ready = false;
            self.battery_voltage_uv = None;
            self.battery_soc_percent = None;
            self.buttons_pressed = None;
        }
        self
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn publish_peripherals(status: PeripheralStatus) {
    PERIPHERALS.lock(|current| current.set(Some(status)));
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn publish_display(ready: bool, errors: u32) {
    DISPLAY.lock(|current| current.set((ready, errors, embassy_time::Instant::now().as_micros())));
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct PowerStatus {
    pub part_info: Option<u8>,
    pub control: Option<u8>,
    pub charger_status: Option<u8>,
    pub verified: bool,
    pub errors: u32,
}

static POWER: Mutex<CriticalSectionRawMutex, Cell<(PowerStatus, u64)>> = Mutex::new(Cell::new((
    PowerStatus {
        part_info: None,
        control: None,
        charger_status: None,
        verified: false,
        errors: 0,
    },
    0,
)));

pub fn power_status() -> PowerStatus {
    let (value, _updated_us) = POWER.lock(Cell::get);
    #[cfg(target_arch = "riscv32")]
    {
        value.at(_updated_us, embassy_time::Instant::now().as_micros())
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        value
    }
}

impl PowerStatus {
    #[cfg(any(target_arch = "riscv32", test))]
    fn at(mut self, updated_us: u64, now_us: u64) -> Self {
        if now_us.saturating_sub(updated_us) > 3_000_000 {
            self.verified = false;
            self.part_info = None;
            self.control = None;
            self.charger_status = None;
        }
        self
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) fn publish_power(status: PowerStatus) {
    POWER.lock(|current| current.set((status, embassy_time::Instant::now().as_micros())));
}

static SNAPSHOT: Mutex<CriticalSectionRawMutex, Cell<Option<Snapshot>>> =
    Mutex::new(Cell::new(None));
static INDICATORS: Mutex<CriticalSectionRawMutex, Cell<Indicators>> =
    Mutex::new(Cell::new(Indicators {
        display_enabled: false,
        led_brightness: 32,
    }));

pub fn snapshot() -> Snapshot {
    let value = SNAPSHOT.lock(Cell::get).unwrap_or_default();
    #[cfg(target_arch = "riscv32")]
    let value = Snapshot {
        imu_drops: IMU_DROPS.load(Ordering::Relaxed),
        gnss_drops: GNSS_DROPS.load(Ordering::Relaxed),
        timing: super::timing::status(),
        ..value
    };
    #[cfg(target_arch = "riscv32")]
    {
        value.at(embassy_time::Instant::now().as_micros())
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        value
    }
}

impl Snapshot {
    #[cfg(any(target_arch = "riscv32", test))]
    pub(super) fn at(mut self, now_us: u64) -> Self {
        // The trajectory is intentionally historical. Allow its configured
        // fusion/smoothing delay, one publication interval and 100 ms of
        // processing/cadence jitter, including when read just before a publish.
        // Neither recent arrivals nor repeated publications renew its epoch.
        let maximum_age_us =
            (super::profile::FUSION_DELAY.as_ns() + super::profile::SMOOTHING_LAG.as_ns()) / 1000
                + super::profile::SNAPSHOT_INTERVAL_US
                + 100_000;
        let trajectory_stale = self
            .epoch_us
            .and_then(|epoch| now_us.checked_sub(epoch))
            .is_none_or(|age| age > maximum_age_us);
        let publication_stale = now_us.saturating_sub(self.updated_us) > 500_000;
        if publication_stale || trajectory_stale {
            if publication_stale && !matches!(self.phase, Phase::Failed | Phase::Starting)
                || matches!(self.phase, Phase::Navigating)
            {
                self.phase = Phase::Degraded;
            }
            self.epoch_us = None;
            self.horizontal_speed_mps = None;
            self.vertical_speed_mps = None;
            self.position_ecef_m = None;
            self.velocity_ecef_mps = None;
        }
        self
    }
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
        DeviceCommand::ResetStatus => DeviceResponse::ResetStatus(crate::bootdiagnostics::status()),
        DeviceCommand::PowerStatus => DeviceResponse::PowerStatus(power_status()),
        DeviceCommand::PeripheralStatus => DeviceResponse::PeripheralStatus(peripheral_status()),
        DeviceCommand::AcquisitionStatus => DeviceResponse::AcquisitionStatus(acquisition_status()),
        DeviceCommand::GnssStatus => DeviceResponse::GnssStatus(gnss_status()),
        DeviceCommand::ImuDiagnostics => DeviceResponse::ImuDiagnostics(imu_diagnostics()),
        DeviceCommand::Status | DeviceCommand::Latest => DeviceResponse::Snapshot(snapshot()),
        DeviceCommand::SetIndicators(value) => {
            INDICATORS.lock(|current| current.set(value));
            DeviceResponse::Indicators(value)
        }
    }
}

#[cfg(test)]
mod freshness_tests {
    use super::*;

    fn navigating(updated_us: u64, epoch_us: Option<u64>) -> Snapshot {
        Snapshot {
            phase: Phase::Navigating,
            updated_us,
            epoch_us,
            last_imu_us: updated_us,
            last_gnss_us: updated_us,
            horizontal_speed_mps: Some(10.0),
            vertical_speed_mps: Some(1.0),
            position_ecef_m: Some([1.0; 3]),
            velocity_ecef_mps: Some([2.0; 3]),
            ..Default::default()
        }
    }

    fn assert_unavailable(state: Snapshot) {
        assert!(matches!(state.phase, Phase::Degraded));
        assert!(state.epoch_us.is_none());
        assert!(state.horizontal_speed_mps.is_none());
        assert!(state.vertical_speed_mps.is_none());
        assert!(state.position_ecef_m.is_none());
        assert!(state.velocity_ecef_mps.is_none());
    }

    #[test]
    fn new_publications_and_received_inputs_cannot_refresh_a_frozen_trajectory() {
        // Inputs and publications advance, but rejection or stalled processing
        // leaves the retained trajectory at the same measurement epoch.
        for now in [1_400_000, 2_000_000, 10_000_000] {
            assert_unavailable(navigating(now, Some(1_000_000)).at(now));
        }
    }

    #[test]
    fn expected_fusion_and_smoothing_delay_remains_live() {
        let state = navigating(1_200_000, Some(1_080_000)).at(1_200_000);
        assert!(matches!(state.phase, Phase::Navigating));
        assert_eq!(state.epoch_us, Some(1_080_000));
        assert_eq!(state.horizontal_speed_mps, Some(10.0));
        let before_next_publication = state.at(1_299_999);
        assert!(matches!(before_next_publication.phase, Phase::Navigating));
    }

    #[test]
    fn missing_or_future_trajectory_cannot_be_navigation() {
        for epoch in [None, Some(2_000_001)] {
            assert_unavailable(navigating(2_000_000, epoch).at(2_000_000));
        }
    }

    #[test]
    fn stopped_publication_expires_even_with_a_recent_trajectory() {
        assert_unavailable(navigating(1_000_000, Some(1_500_000)).at(1_500_001));
    }

    #[test]
    fn stalled_peripheral_owner_cannot_report_live_measurements() {
        let state = PeripheralStatus {
            updated_us: 1_000_000,
            leds_ready: true,
            display_ready: true,
            battery_voltage_uv: Some(4_000_000),
            battery_soc_percent: Some(50.0),
            buttons_pressed: Some(3),
            gauge_errors: 7,
            ..Default::default()
        };
        assert!(state.at(3_000_000).leds_ready);
        let stale = state.at(4_000_001);
        assert!(!stale.leds_ready && !stale.display_ready);
        assert!(stale.battery_voltage_uv.is_none());
        assert!(stale.battery_soc_percent.is_none());
        assert!(stale.buttons_pressed.is_none());
        assert_eq!(stale.updated_us, state.updated_us);
        assert_eq!(stale.gauge_errors, 7);
    }

    #[test]
    fn display_and_power_bus_health_expire_independently() {
        let health = PeripheralStatus {
            updated_us: 5_000_000,
            leds_ready: true,
            battery_voltage_uv: Some(4_000_000),
            ..Default::default()
        };
        let stalled_display = health.with_display((true, 7, 1_000_000), 5_000_000);
        assert!(!stalled_display.display_ready);
        assert_eq!(stalled_display.display_errors, 7);
        assert!(stalled_display.leds_ready);
        assert!(stalled_display.battery_voltage_uv.is_some());
        let stalled_bus = health.with_display((true, 9, 9_000_000), 9_000_000);
        assert!(stalled_bus.display_ready);
        assert!(!stalled_bus.leds_ready);
        assert!(stalled_bus.battery_voltage_uv.is_none());
    }

    #[test]
    fn power_policy_verification_expires_without_a_new_readback() {
        let state = PowerStatus {
            verified: true,
            part_info: Some(24),
            control: Some(0x80),
            charger_status: Some(0),
            errors: 5,
        };
        assert!(state.at(1_000_000, 4_000_000).verified);
        let stale = state.at(1_000_000, 4_000_001);
        assert!(!stale.verified);
        assert!(stale.control.is_none() && stale.charger_status.is_none());
        assert_eq!(stale.errors, 5);
    }
}
