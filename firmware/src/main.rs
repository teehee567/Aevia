#![no_std]
#![no_main]

mod gnss_profile;
#[allow(dead_code)] // Shared with the host runner and parser tests.
mod gps;
mod imu;
mod led;
mod power_diagnostic;
#[cfg(not(feature = "host-poc"))]
mod trajectory_poc;

use core::{
    cell::RefCell,
    fmt::Write as _,
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, RxConfig, Uart},
    usb::otg::{
        Usb,
        embassy_usb_device::{Config as UsbDriverConfig, Driver},
    },
};
#[cfg(not(feature = "host-poc"))]
use gps::OwnedBestNav;
use gps::{LineDecoder, ParseError, parse_bestnava, parse_bestnava_status};

esp_bootloader_esp_idf::esp_app_desc!();

// Host POC: IMU acquisition runs on core 1, isolated from core 0 GNSS parsing
// and USB formatting. The embedded mode uses core 1 for its estimator.
// A full queue drops explicitly instead of stalling the sensor capture.
enum Measurement {
    Imu(imu::ImuSample),
    #[cfg(not(feature = "host-poc"))]
    Gnss {
        received_us: u64,
        fix: OwnedBestNav,
    },
    #[cfg(feature = "host-poc")]
    GnssRaw {
        received_us: u64,
        line: heapless::String<512>,
    },
}
static MEASUREMENTS: Channel<CriticalSectionRawMutex, Measurement, 128> = Channel::new();
#[cfg(feature = "host-poc")]
static IMU_STACK: static_cell::ConstStaticCell<Stack<16384>> =
    static_cell::ConstStaticCell::new(Stack::new());
#[cfg(not(feature = "host-poc"))]
static ESTIMATOR_STACK: static_cell::ConstStaticCell<Stack<65536>> =
    static_cell::ConstStaticCell::new(Stack::new());
#[cfg(not(feature = "host-poc"))]
static SPEED: Mutex<CriticalSectionRawMutex, RefCell<heapless::String<192>>> =
    Mutex::new(RefCell::new(heapless::String::new()));
static FIX_MODE: Mutex<CriticalSectionRawMutex, RefCell<heapless::String<48>>> =
    Mutex::new(RefCell::new(heapless::String::new()));
static IMU_STATE: Mutex<CriticalSectionRawMutex, RefCell<heapless::String<192>>> =
    Mutex::new(RefCell::new(heapless::String::new()));
static IMU_ERROR: Mutex<CriticalSectionRawMutex, RefCell<heapless::String<192>>> =
    Mutex::new(RefCell::new(heapless::String::new()));
static GNSS_STATE: Mutex<CriticalSectionRawMutex, RefCell<heapless::String<80>>> =
    Mutex::new(RefCell::new(heapless::String::new()));
static GNSS_RECORDS: AtomicU32 = AtomicU32::new(0);
static GNSS_VALID: AtomicU32 = AtomicU32::new(0);
static GNSS_REGULAR_EPOCHS: AtomicU32 = AtomicU32::new(0);
static IMU_RECORDS: AtomicU32 = AtomicU32::new(0);
static CAPTURE_ERRORS: AtomicU32 = AtomicU32::new(0);
static QUEUE_DROPS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "host-poc")]
static STREAM_NONCE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "host-poc")]
static STREAM_PENDING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
#[cfg(not(feature = "host-poc"))]
static ENGINE_ERRORS: AtomicU32 = AtomicU32::new(0);
#[cfg(not(feature = "host-poc"))]
static LAST_ESTIMATOR_MS: AtomicU32 = AtomicU32::new(0);
#[cfg(not(feature = "host-poc"))]
static ENGINE_MAX_US: AtomicU32 = AtomicU32::new(0);
#[cfg(not(feature = "host-poc"))]
static PSRAM_MIB: AtomicU32 = AtomicU32::new(0);

macro_rules! state_text {
    ($state:expr, $($arg:tt)*) => {
        $state.lock(|cell| {
            let mut text = cell.borrow_mut();
            text.clear();
            let _ = write!(text, $($arg)*);
        })
    };
}

fn enqueue(measurement: Measurement) {
    if MEASUREMENTS.try_send(measurement).is_err() {
        QUEUE_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg_attr(feature = "host-poc", embassy_executor::task)]
async fn imu_task(mut sensor: imu::Imu<'static>) {
    loop {
        state_text!(IMU_STATE, "initializing");
        if let Err(error) = sensor.initialize().await {
            state_text!(IMU_STATE, "init-failed:{error:?}");
            Timer::after_secs(2).await;
            continue;
        }
        state_text!(IMU_STATE, "ready-id:{:04x}", sensor.component_id);
        let mut consecutive_errors = 0;
        loop {
            match sensor.next_sample().await {
                Ok(sample) => {
                    if consecutive_errors != 0 {
                        state_text!(IMU_STATE, "ready-id:{:04x}", sensor.component_id);
                    }
                    consecutive_errors = 0;
                    IMU_RECORDS.fetch_add(1, Ordering::Relaxed);
                    enqueue(Measurement::Imu(sample));
                }
                Err(error) => {
                    CAPTURE_ERRORS.fetch_add(1, Ordering::Relaxed);
                    state_text!(IMU_STATE, "sample-failed:{error:?}");
                    state_text!(IMU_ERROR, "{error:?}");
                    consecutive_errors += 1;
                    if consecutive_errors >= 10 {
                        break;
                    }
                }
            }
        }
    }
}

#[cfg(not(feature = "host-poc"))]
fn estimator_thread(psram_address: usize, psram_len: usize) {
    state_text!(SPEED, "speed=unavailable phase=starting");
    // PSRAM has been initialized on core 0; this is the sole workspace owner.
    let mut engine = match unsafe { trajectory_poc::start(psram_address as *mut u8, psram_len) } {
        Ok(engine) => engine,
        Err(error) => {
            state_text!(SPEED, "speed=unavailable engine-start={error:?}");
            loop {
                core::hint::spin_loop();
            }
        }
    };
    state_text!(SPEED, "speed=unavailable phase=waiting-for-sensors");
    let mut last_report_us = 0;
    loop {
        let measurement = match MEASUREMENTS.try_receive() {
            Ok(measurement) => measurement,
            Err(_) => {
                // Do not busy-poll a cross-core critical-section lock: that
                // starves capture interrupts while both sensors are starting.
                esp_hal::delay::Delay::new().delay_micros(100);
                continue;
            }
        };
        let step_start = Instant::now().as_micros();
        let result = match measurement {
            Measurement::Imu(sample) => engine.push_imu(
                sample.monotonic_us,
                sample.interval_us,
                sample.acceleration_mps2,
                sample.angular_rate_rps,
                false, // The driver rejects saturated/status-invalid frames.
            ),
            Measurement::Gnss { received_us, fix } => engine.push_gnss(received_us, &fix.as_nav()),
        };
        ENGINE_MAX_US.fetch_max(
            (Instant::now().as_micros() - step_start) as u32,
            Ordering::Relaxed,
        );
        if result.is_err() {
            ENGINE_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        let now = Instant::now();
        if now.as_micros().saturating_sub(last_report_us) >= 100_000 {
            let snapshot = engine.snapshot();
            if let Some(speed) = snapshot.speed_mps {
                state_text!(
                    SPEED,
                    "speed={:.2}km/h phase={:?} fused={} imu-accepted={} rejected={}/{}",
                    speed * 3.6,
                    snapshot.phase,
                    snapshot.diagnostics.gnss_updates_fused,
                    snapshot.diagnostics.imu_epochs_accepted,
                    snapshot.diagnostics.imu_epochs_rejected,
                    snapshot.diagnostics.gnss_updates_rejected
                );
            } else {
                state_text!(
                    SPEED,
                    "speed=unavailable phase={:?} fused={} imu-accepted={} input={:?}",
                    snapshot.phase,
                    snapshot.diagnostics.gnss_updates_fused,
                    snapshot.diagnostics.imu_epochs_accepted,
                    snapshot.last_input
                );
            }
            LAST_ESTIMATOR_MS.store(now.as_millis() as u32, Ordering::Relaxed);
            last_report_us = now.as_micros();
        }
    }
}

#[embassy_executor::task]
async fn power_button_task(mut interrupt_n: Input<'static>, mut kill_n: Output<'static>) {
    // MAX16169AALT debounces SW7 itself (50 ms) and sends a 32 ms INT pulse
    // on each press, including power-on. Let that startup pulse and the CLR
    // blanking interval (2 * tINT, at most 76.8 ms) expire before arming.
    // https://www.analog.com/media/en/technical-documentation/data-sheets/max16169.pdf
    Timer::after_millis(100).await;
    interrupt_n.wait_for_high().await;
    interrupt_n.wait_for_low().await;

    // The current POC has no on-board storage writes to flush. Do not wait
    // for USB: shutdown must also work without a connected host/reader.
    // Keep CLR asserted until the MAX16169 disables the main 3.3 V rail.
    kill_n.set_low();
    core::future::pending::<()>().await;
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let power_kill_n = Output::new(
        peripherals.GPIO8,
        Level::High,
        OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::Up),
    );
    let power_interrupt_n = Input::new(
        peripherals.GPIO38,
        InputConfig::default().with_pull(Pull::Up),
    );
    let led_enable = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    let _lcd_backlight = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let _gnss_reset_n = Input::new(
        peripherals.GPIO44,
        InputConfig::default().with_pull(Pull::Up),
    );
    let mut power_i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("power I2C")
        .with_scl(peripherals.GPIO6)
        .with_sda(peripherals.GPIO7);
    let power_report = power_diagnostic::capture(&mut power_i2c);

    let mut gnss_command_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(1)),
    )
    .expect("GNSS COM1 UART")
    .with_tx(peripherals.GPIO47)
    .with_rx(peripherals.GPIO46)
    .into_async();
    let mut gnss_data_uart = Uart::new(
        peripherals.UART2,
        UartConfig::default()
            .with_baudrate(115_200)
            .with_rx(RxConfig::default().with_fifo_full_threshold(64)),
    )
    .expect("GNSS COM2 UART")
    .with_tx(peripherals.GPIO49)
    .with_rx(peripherals.GPIO48)
    .into_async();

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .expect("IMU SPI")
    .with_sck(peripherals.GPIO12)
    .with_miso(peripherals.GPIO13)
    .with_mosi(peripherals.GPIO11);
    let sensor = imu::Imu::new(
        spi,
        Output::new(peripherals.GPIO10, Level::High, OutputConfig::default()),
        Output::new(peripherals.GPIO9, Level::High, OutputConfig::default()),
        Input::new(peripherals.GPIO14, InputConfig::default()),
    );
    let timer = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer.timer0, peripherals.FROM_CPU_INTR0);
    spawner.spawn(led::task(power_i2c, led_enable).expect("LED task allocation failed"));
    spawner.spawn(
        power_button_task(power_interrupt_n, power_kill_n)
            .expect("power button task allocation failed"),
    );
    #[cfg(feature = "host-poc")]
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        peripherals.FROM_CPU_INTR1,
        IMU_STACK.take(),
        move || {
            static EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::Executor> =
                static_cell::StaticCell::new();
            let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner| {
                spawner.spawn(imu_task(sensor).expect("IMU task allocation failed"));
            });
        },
    );
    #[cfg(not(feature = "host-poc"))]
    {
        let psram = esp_hal::psram::Psram::new(
            peripherals.PSRAM,
            esp_hal::psram::PsramConfig {
                timing: esp_hal::psram::PsramTimingParams::MHZ_125,
                ..Default::default()
            },
        );
        let (psram_ptr, psram_len) = psram.raw_parts();
        PSRAM_MIB.store((psram_len / (1024 * 1024)) as u32, Ordering::Relaxed);
        let psram_address = psram_ptr as usize;
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            peripherals.FROM_CPU_INTR1,
            ESTIMATOR_STACK.take(),
            move || estimator_thread(psram_address, psram_len),
        );
    }

    let usb = Usb::new_hs(peripherals.USB_HS);
    let mut endpoint_buffer = [0_u8; 1024];
    let driver = Driver::new(usb, &mut endpoint_buffer, UsbDriverConfig::default());
    let mut usb_config = embassy_usb::Config::new(0x303A, 0x4001);
    usb_config.max_packet_size_0 = 64;
    usb_config.manufacturer = Some("AEVIA");
    usb_config.product = Some("V2 Mini Trajectory POC");
    usb_config.serial_number = Some("V2MINI0001");
    let mut config_descriptor = [0_u8; 256];
    let mut bos_descriptor = [0_u8; 256];
    let mut control_buffer = [0_u8; 64];
    let mut cdc_state = State::new();
    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buffer,
    );
    let serial = CdcAcmClass::new(&mut builder, &mut cdc_state, 512);
    let (mut sender, mut receiver) = serial.split();
    let mut usb_device = builder.build();

    let gnss_task = async {
        loop {
            led::gnss_reconfiguring();
            let mut decoder = LineDecoder::<1024>::new();
            state_text!(
                GNSS_STATE,
                "configuring-{}Hz",
                if cfg!(feature = "rtk-50hz") { 50 } else { 20 }
            );
            let profile = if cfg!(feature = "rtk-50hz") {
                gnss_profile::configure_50hz(&mut gnss_command_uart, &mut gnss_data_uart).await
            } else {
                gnss_profile::configure_20hz_standalone(&mut gnss_command_uart, &mut gnss_data_uart)
                    .await
            };
            match profile {
                Ok(profile) => state_text!(
                    GNSS_STATE,
                    "{}Hz-sg{}-cmdCOM{}:{}-verify:{}",
                    profile.target_rate_hz,
                    profile.signalgroup,
                    profile.command_port,
                    profile.command_baud,
                    if profile.readback_verified {
                        "config"
                    } else {
                        "stream"
                    }
                ),
                Err(error) => {
                    state_text!(GNSS_STATE, "setup-failed:{error:?}");
                    Timer::after_secs(3).await;
                    continue;
                }
            }
            let mut previous_epoch = None;
            let mut chunk = [0_u8; 512];
            loop {
                match with_timeout(
                    Duration::from_secs(3),
                    embedded_io_async::Read::read(&mut gnss_data_uart, &mut chunk),
                )
                .await
                {
                    Ok(Ok(length)) => {
                        for byte in &chunk[..length] {
                            let Some(line) = decoder.push(*byte) else {
                                continue;
                            };
                            if !line.starts_with(b"#BESTNAVA,") {
                                continue;
                            }
                            let received_us = Instant::now().as_micros();
                            if let Ok(status) = parse_bestnava_status(line) {
                                led::gnss_received();
                                GNSS_RECORDS.fetch_add(1, Ordering::Relaxed);
                                let epoch = u64::from(status.gps_week) * 604_800_000
                                    + u64::from(status.gps_tow_ms);
                                if previous_epoch.is_some_and(|previous| {
                                    epoch
                                        == previous
                                            + if cfg!(feature = "rtk-50hz") { 20 } else { 50 }
                                }) {
                                    GNSS_REGULAR_EPOCHS.fetch_add(1, Ordering::Relaxed);
                                }
                                previous_epoch = Some(epoch);
                                state_text!(
                                    FIX_MODE,
                                    "{}",
                                    core::str::from_utf8(status.position_type).unwrap_or("unknown")
                                );
                            }
                            #[cfg(feature = "host-poc")]
                            if let Ok(text) = core::str::from_utf8(line) {
                                if let Ok(raw) = heapless::String::try_from(text.trim_end()) {
                                    enqueue(Measurement::GnssRaw {
                                        received_us,
                                        line: raw,
                                    });
                                }
                            }
                            match parse_bestnava(line) {
                                Ok(fix) => {
                                    GNSS_VALID.fetch_add(1, Ordering::Relaxed);
                                    #[cfg(not(feature = "host-poc"))]
                                    enqueue(Measurement::Gnss {
                                        received_us,
                                        fix: fix.to_owned(),
                                    });
                                    #[cfg(feature = "host-poc")]
                                    let _ = fix;
                                }
                                Err(ParseError::NoSolution) => {}
                                Err(_) => {
                                    CAPTURE_ERRORS.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }
                    }
                    Ok(Err(_)) => {
                        CAPTURE_ERRORS.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(_) => {
                        state_text!(GNSS_STATE, "no-data-retrying");
                        break;
                    }
                }
            }
        }
    };

    #[cfg(not(feature = "host-poc"))]
    let imu_acquisition = imu_task(sensor);
    #[cfg(feature = "host-poc")]
    let imu_acquisition = core::future::pending::<()>();

    #[cfg(not(feature = "host-poc"))]
    let console_task = async {
        loop {
            sender.wait_connection().await;
            let mut previous = [
                GNSS_RECORDS.load(Ordering::Relaxed),
                GNSS_VALID.load(Ordering::Relaxed),
                IMU_RECORDS.load(Ordering::Relaxed),
                GNSS_REGULAR_EPOCHS.load(Ordering::Relaxed),
            ];
            let mut last_ms = Instant::now().as_millis();
            'reports: loop {
                // A USB reader may attach long after enumeration. Never burst
                // old timer ticks when a previously blocked write completes.
                Timer::after_secs(1).await;
                let now_ms = Instant::now().as_millis();
                let elapsed = (now_ms - last_ms).max(1) as f64 / 1000.0;
                let counts = [
                    GNSS_RECORDS.load(Ordering::Relaxed),
                    GNSS_VALID.load(Ordering::Relaxed),
                    IMU_RECORDS.load(Ordering::Relaxed),
                    GNSS_REGULAR_EPOCHS.load(Ordering::Relaxed),
                ];
                let mut line = heapless::String::<1024>::new();
                let _ = write!(line, "TRAJ t={:.3}s ", now_ms as f64 / 1000.0);
                let last_engine = LAST_ESTIMATOR_MS.load(Ordering::Relaxed);
                if last_engine != 0 && (now_ms as u32).wrapping_sub(last_engine) > 2000 {
                    let _ = line.push_str("speed=unavailable phase=stale");
                } else {
                    SPEED.lock(|cell| {
                        let _ = line.push_str(&cell.borrow());
                    });
                }
                let _ = write!(
                    line,
                    " gnss={:.1}Hz valid={:.1}Hz imu={:.1}Hz regular-epochs={} fix=",
                    counts[0].wrapping_sub(previous[0]) as f64 / elapsed,
                    counts[1].wrapping_sub(previous[1]) as f64 / elapsed,
                    counts[2].wrapping_sub(previous[2]) as f64 / elapsed,
                    counts[3].wrapping_sub(previous[3])
                );
                FIX_MODE.lock(|cell| {
                    let _ = line.push_str(&cell.borrow());
                });
                let _ = line.push_str(" imu-state=");
                IMU_STATE.lock(|cell| {
                    let _ = line.push_str(&cell.borrow());
                });
                let _ = line.push_str(" gnss-state=");
                GNSS_STATE.lock(|cell| {
                    let _ = line.push_str(&cell.borrow());
                });
                let _ = write!(line, " led={}", led::status());
                let _ = write!(
                    line,
                    " capture-errors={} queue-drops={} engine-errors={} step-max={}us start-stage={} psram={}MiB development=unqualified\r\n",
                    CAPTURE_ERRORS.load(Ordering::Relaxed),
                    QUEUE_DROPS.load(Ordering::Relaxed),
                    ENGINE_ERRORS.load(Ordering::Relaxed),
                    ENGINE_MAX_US.swap(0, Ordering::Relaxed),
                    trajectory_poc::START_STAGE.load(Ordering::Relaxed),
                    PSRAM_MIB.load(Ordering::Relaxed)
                );
                previous = counts;
                last_ms = now_ms;
                for packet in line.as_bytes().chunks(512) {
                    if sender.write_packet(packet).await.is_err() {
                        break 'reports;
                    }
                }
            }
        }
    };

    // The host option exercises the identical traj adapter on the connected
    // computer while this image captures the physical sensors continuously.
    #[cfg(feature = "host-poc")]
    let console_task = async {
        loop {
            sender.wait_connection().await;
            let mut power_line = power_report.clone();
            let _ = power_line.push_str("\r\n");
            let _ = sender.write_packet(power_line.as_bytes()).await;
            let mut last_status_us = 0;
            let mut last_power_us = 0;
            'raw: loop {
                if STREAM_PENDING.swap(false, Ordering::AcqRel) {
                    // The marker is sent by the same writer as RAW records.
                    // Everything before it belongs to the preceding session.
                    MEASUREMENTS.clear();
                    let mut marker = heapless::String::<64>::new();
                    let _ = write!(
                        marker,
                        "STREAM READY {:08x} {}\r\n",
                        STREAM_NONCE.load(Ordering::Acquire),
                        Instant::now().as_micros()
                    );
                    if sender.write_packet(marker.as_bytes()).await.is_err() {
                        break 'raw;
                    }
                }
                match with_timeout(Duration::from_millis(100), MEASUREMENTS.receive()).await {
                    Ok(measurement) => {
                        let mut line = heapless::String::<640>::new();
                        match measurement {
                            Measurement::Imu(sample) => {
                                let a = sample.acceleration_mps2;
                                let g = sample.angular_rate_rps;
                                let _ = write!(
                                    line,
                                    "RAW IMU {} {} {:.7} {:.7} {:.7} {:.9} {:.9} {:.9}\r\n",
                                    sample.monotonic_us,
                                    sample.interval_us,
                                    a[0],
                                    a[1],
                                    a[2],
                                    g[0],
                                    g[1],
                                    g[2]
                                );
                            }
                            Measurement::GnssRaw {
                                received_us,
                                line: frame,
                            } => {
                                let _ = write!(line, "RAW GNSS {} {}\r\n", received_us, frame);
                            }
                        }
                        for packet in line.as_bytes().chunks(512) {
                            if sender.write_packet(packet).await.is_err() {
                                break 'raw;
                            }
                        }
                    }
                    Err(_) => {}
                }
                let now = Instant::now().as_micros();
                if now - last_status_us >= 1_000_000 {
                    last_status_us = now;
                    let mut line = heapless::String::<512>::new();
                    let _ = line.push_str("STATUS imu=");
                    IMU_STATE.lock(|state| {
                        let _ = line.push_str(&state.borrow());
                    });
                    let _ = line.push_str(" gnss=");
                    GNSS_STATE.lock(|state| {
                        let _ = line.push_str(&state.borrow());
                    });
                    let _ = write!(line, " led={}", led::status());
                    let _ = line.push_str(" last-imu-error=");
                    IMU_ERROR.lock(|state| {
                        let _ = line.push_str(&state.borrow());
                    });
                    let _ = write!(
                        line,
                        " capture-errors={} queue-drops={}\r\n",
                        CAPTURE_ERRORS.load(Ordering::Relaxed),
                        QUEUE_DROPS.load(Ordering::Relaxed)
                    );
                    if sender.write_packet(line.as_bytes()).await.is_err() {
                        break 'raw;
                    }
                    // Repeat the startup snapshot because USB may enumerate
                    // before a terminal starts reading its first packet.
                    if now - last_power_us >= 10_000_000 {
                        last_power_us = now;
                        if sender.write_packet(power_line.as_bytes()).await.is_err() {
                            break 'raw;
                        }
                    }
                }
            }
        }
    };

    let command_task = async {
        let mut packet = [0_u8; 512];
        let mut matched = 0;
        #[cfg(feature = "host-poc")]
        let mut command_line = heapless::Vec::<u8, 24>::new();
        loop {
            receiver.wait_connection().await;
            loop {
                match receiver.read_packet(&mut packet).await {
                    Ok(length) => {
                        for &byte in &packet[..length] {
                            #[cfg(feature = "host-poc")]
                            if byte == b'\n' {
                                if let Some(token) = command_line.strip_prefix(b"STREAM ") {
                                    if token.len() == 8 {
                                        if let Ok(text) = core::str::from_utf8(token) {
                                            if let Ok(nonce) = u32::from_str_radix(text, 16) {
                                                STREAM_NONCE.store(nonce, Ordering::Release);
                                                STREAM_PENDING.store(true, Ordering::Release);
                                            }
                                        }
                                    }
                                }
                                command_line.clear();
                            } else if byte != b'\r' && command_line.push(byte).is_err() {
                                command_line.clear();
                            }
                            if byte == b"BOOTLOADER"[matched] {
                                matched += 1;
                                if matched == b"BOOTLOADER".len() {
                                    esp_hal::peripherals::LP_SYS::regs()
                                        .sys_ctrl()
                                        .modify(|_, writer| writer.force_download_boot().set_bit());
                                    esp_hal::system::software_reset();
                                }
                            } else {
                                matched = usize::from(byte == b'B');
                            }
                        }
                    }
                    Err(_) => {
                        matched = 0;
                        #[cfg(feature = "host-poc")]
                        command_line.clear();
                        break;
                    }
                }
            }
        }
    };
    join3(
        usb_device.run(),
        join(gnss_task, imu_acquisition),
        join(console_task, command_task),
    )
    .await;
}
