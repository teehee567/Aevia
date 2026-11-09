//! Capture runs independently of estimation, display, and transport consumers.
use super::{gnss, imu::Imu, status, timing};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use esp_hal::{
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::*,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    uart::{Config as UartConfig, RxConfig, Uart},
};

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
#[derive(Clone, Copy, Debug)]
pub struct GnssSample {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    pub position_sigma_m: [f64; 3],
    pub velocity_sigma_mps: [f64; 3],
    pub epoch_ms: u64,
    pub received_us: u64,
    pub position_time_ns: u64,
    pub velocity_time_ns: u64,
    pub timing_uncertainty_ns: u64,
    pub clock_generation: u32,
    pub position_valid: bool,
    pub velocity_valid: bool,
    pub satellites: u8,
}

// ~73 ms of maximum-rate IMU and 320 ms of GNSS. Producers never wait for
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

/// The fitted V2 Mini wiring. Unused GPIOs remain with the runtime.
pub struct Board {
    pub uart1: UART1<'static>,
    pub uart2: UART2<'static>,
    pub spi2: SPI2<'static>,
    pub spi3: SPI3<'static>,
    pub i2c0: I2C0<'static>,
    pub gpio4: GPIO4<'static>,
    pub gpio6: GPIO6<'static>,
    pub gpio7: GPIO7<'static>,
    pub gpio8: GPIO8<'static>,
    pub gpio9: GPIO9<'static>,
    pub gpio10: GPIO10<'static>,
    pub gpio11: GPIO11<'static>,
    pub gpio12: GPIO12<'static>,
    pub gpio13: GPIO13<'static>,
    pub gpio14: GPIO14<'static>,
    pub gpio15: GPIO15<'static>,
    pub gpio16: GPIO16<'static>,
    pub gpio17: GPIO17<'static>,
    pub gpio18: GPIO18<'static>,
    pub gpio19: GPIO19<'static>,
    pub gpio39: GPIO39<'static>,
    pub gpio42: GPIO42<'static>,
    pub gpio44: GPIO44<'static>,
    pub gpio46: GPIO46<'static>,
    pub gpio47: GPIO47<'static>,
    pub gpio48: GPIO48<'static>,
    pub gpio49: GPIO49<'static>,
}

pub fn start(spawner: Spawner, b: Board) {
    let pps = Input::new(b.gpio42, InputConfig::default().with_pull(Pull::Down));
    let ready = Input::new(b.gpio14, InputConfig::default());
    timing::capture::init();
    let uart_config = UartConfig::default()
        .with_baudrate(115_200)
        .with_rx(RxConfig::default().with_fifo_full_threshold(32));
    let command = Uart::new(b.uart1, uart_config.clone())
        .expect("GNSS COM1")
        .with_tx(b.gpio47)
        .with_rx(b.gpio46)
        .into_async();
    let data = Uart::new(b.uart2, uart_config)
        .expect("GNSS COM2")
        .with_tx(b.gpio49)
        .with_rx(b.gpio48)
        .into_async();
    let reset = Input::new(b.gpio44, InputConfig::default().with_pull(Pull::Up));
    let spi = Spi::new(
        b.spi2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(8))
            .with_mode(Mode::_0),
    )
    .expect("IMU SPI")
    .with_sck(b.gpio12)
    .with_mosi(b.gpio11)
    .with_miso(b.gpio13);
    let imu = Imu::new(
        spi,
        Output::new(b.gpio10, Level::High, OutputConfig::default()),
        Output::new(b.gpio9, Level::High, OutputConfig::default()),
        ready,
    );
    let display_spi = Spi::new(
        b.spi3,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(24))
            .with_mode(Mode::_0),
    )
    .expect("display SPI")
    .with_sck(b.gpio16)
    .with_mosi(b.gpio17);
    let display = status::Display::new(
        display_spi,
        Output::new(b.gpio18, Level::High, OutputConfig::default()),
        Output::new(b.gpio19, Level::Low, OutputConfig::default()),
        Output::new(b.gpio15, Level::High, OutputConfig::default()),
        Output::new(b.gpio4, Level::Low, OutputConfig::default()),
    );
    let i2c = I2c::new(
        b.i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("status I2C")
    .with_scl(b.gpio6)
    .with_sda(b.gpio7)
    .into_async();
    let enable = Output::new(b.gpio39, Level::Low, OutputConfig::default());
    // MAX16169 active-low power kill is released. There are no button tasks.
    let power = Output::new(
        b.gpio8,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );
    spawner.spawn(gnss::task(command, data, reset).expect("GNSS task"));
    spawner.spawn(imu_task(imu).expect("IMU task"));
    spawner.spawn(pps_task(pps).expect("PPS task"));
    spawner.spawn(status::task(i2c, enable, display, power).expect("status task"));
}

#[embassy_executor::task]
async fn pps_task(_pin: Input<'static>) {
    // The edge is latched in hardware; polling only transfers its value into
    // history. It also extends the counter during sensor outages.
    loop {
        timing::observe_pulse();
        let _ = timing::capture::now_ns();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn imu_task(mut imu: Imu<'static>) {
    loop {
        if imu.initialize().await.is_err() {
            IMU_ERRORS.fetch_add(1, Ordering::Relaxed);
            Timer::after_secs(1).await;
            continue;
        }
        let mut failures = 0;
        loop {
            match imu.next_sample().await {
                Ok(sample) => {
                    failures = 0;
                    IMU_SAMPLES.fetch_add(1, Ordering::Relaxed);
                    if IMU.try_send(sample).is_err() {
                        IMU_DROPS.fetch_add(1, Ordering::Relaxed);
                    }
                }
                Err(error) => {
                    IMU_ERRORS.fetch_add(1, Ordering::Relaxed);
                    if matches!(error, super::imu::Error::Timing) {
                        IMU_TIMING_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                    failures += 1;
                    if failures >= 4 {
                        break;
                    }
                }
            }
        }
    }
}
