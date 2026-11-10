//! Capture runs independently of estimation, display, and transport consumers.
use super::{board::SensorResources, gnss, imu, timing};
use core::{
    cell::Cell,
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};
use embassy_time::Timer;
use esp_hal::gpio::Input;

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    /// Hardware DRDY edge, retained separately from the corrected support.
    pub captured_ns: u64,
    /// End of the aligned gyro/acceleration averaging interval.
    pub end_ns: u64,
    pub duration_ns: u32,
    pub acceleration_mps2: [f64; 3],
    pub angular_rate_rps: [f64; 3],
    pub saturated: bool,
}
pub use super::gnss::observation::GnssSample;

// ~87 ms of nominal 1.475 kHz IMU and 320 ms of GNSS. Producers never wait for
// these queues: overload is counted and remains an observable time gap.
pub static IMU: Channel<CriticalSectionRawMutex, ImuSample, 128> = Channel::new();
pub static GNSS: Channel<CriticalSectionRawMutex, GnssSample, 16> = Channel::new();
pub static IMU_DROPS: AtomicU32 = AtomicU32::new(0);
pub static GNSS_DROPS: AtomicU32 = AtomicU32::new(0);
pub static IMU_SAMPLES: AtomicU32 = AtomicU32::new(0);
pub static GNSS_SAMPLES: AtomicU32 = AtomicU32::new(0);
pub static IMU_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static IMU_TIMING_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static GNSS_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static GNSS_UART_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static GNSS_PARSE_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static GNSS_RATE_HZ: AtomicU32 = AtomicU32::new(0);
pub static IMU_MAX_READ_LATENCY_US: AtomicU32 = AtomicU32::new(0);
static IMU_HIGH_WATER: AtomicU32 = AtomicU32::new(0);
static GNSS_HIGH_WATER: AtomicU32 = AtomicU32::new(0);
static LAST_IMU: Mutex<CriticalSectionRawMutex, Cell<Option<u64>>> = Mutex::new(Cell::new(None));
static LAST_GNSS: Mutex<CriticalSectionRawMutex, Cell<Option<u64>>> = Mutex::new(Cell::new(None));

pub fn record_imu(at_us: u64) {
    IMU_SAMPLES.fetch_add(1, Ordering::Relaxed);
    LAST_IMU.lock(|last| last.set(Some(at_us)));
}

pub fn record_gnss(at_us: u64) {
    GNSS_SAMPLES.fetch_add(1, Ordering::Relaxed);
    LAST_GNSS.lock(|last| last.set(Some(at_us)));
}

pub fn observe_queue_depths() {
    IMU_HIGH_WATER.fetch_max(IMU.len() as u32, Ordering::Relaxed);
    GNSS_HIGH_WATER.fetch_max(GNSS.len() as u32, Ordering::Relaxed);
}

pub fn acquisition_status() -> super::interface::AcquisitionStatus {
    super::interface::AcquisitionStatus {
        updated_us: embassy_time::Instant::now().as_micros(),
        last_imu_us: LAST_IMU.lock(Cell::get),
        last_gnss_us: LAST_GNSS.lock(Cell::get),
        imu_samples: IMU_SAMPLES.load(Ordering::Relaxed),
        gnss_samples: GNSS_SAMPLES.load(Ordering::Relaxed),
        imu_errors: IMU_ERRORS.load(Ordering::Relaxed),
        gnss_errors: GNSS_ERRORS.load(Ordering::Relaxed),
        gnss_uart_errors: GNSS_UART_ERRORS.load(Ordering::Relaxed),
        gnss_parse_errors: GNSS_PARSE_ERRORS.load(Ordering::Relaxed),
        gnss_rate_hz: GNSS_RATE_HZ.load(Ordering::Relaxed),
        imu_drops: IMU_DROPS.load(Ordering::Relaxed),
        gnss_drops: GNSS_DROPS.load(Ordering::Relaxed),
        imu_queue_depth: IMU.len() as u16,
        gnss_queue_depth: GNSS.len() as u16,
        imu_queue_high_water: IMU_HIGH_WATER.load(Ordering::Relaxed) as u16,
        gnss_queue_high_water: GNSS_HIGH_WATER.load(Ordering::Relaxed) as u16,
        imu_max_read_latency_us: IMU_MAX_READ_LATENCY_US.load(Ordering::Relaxed),
        timing: timing::status(),
    }
}

/// All high-rate tasks use bounded polls on the acquisition executor. Slow
/// power-bus operations run on a separate executor and cannot block these.
pub fn start(
    spawner: Spawner,
    acquisition: SendSpawner,
    gnss_acquisition: SendSpawner,
    resources: SensorResources,
) {
    gnss_acquisition.spawn(
        gnss::task(resources.command, resources.data, resources.gnss_reset).expect("GNSS task"),
    );
    imu::start(spawner, acquisition, resources.imu);
    gnss_acquisition.spawn(pps_task(resources.pps).expect("PPS task"));
}

#[embassy_executor::task]
async fn pps_task(_pin: Input<'static>) {
    // The edge is latched in hardware; polling only transfers its value into
    // history. Maintenance validates/extends the counter during sensor outages.
    loop {
        timing::capture::maintain();
        timing::observe_pulse();
        super::supervision::acquisition_heartbeat();
        Timer::after_millis(100).await;
    }
}
