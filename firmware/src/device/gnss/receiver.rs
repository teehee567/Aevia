//! Asynchronous receiver lifecycle and acquisition. N4 framing, typed messages
//! and command encoding come from the HAL; timestamp/fusion policy stays here.
use super::super::{
    sensors::{self, GnssSample},
    timing::{self, PpsLabel},
};
use super::{
    diagnostics::ParserContext,
    navigation::{EpochProgress, EpochState, Navigation},
    transport,
    verification::{Action, PpsVerification},
};
use aevia_device_hal::um980::{
    Baud, BinaryDecoder, Command, DecodeError, Decoder, Frame, LogMessage, Period, Port, PpsStatus,
};
use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Write;
use esp_hal::{
    Async,
    uart::{Config, RxConfig, Uart},
};

const BAUD: Baud = Baud::B460800;
const IDLE_TIMEOUT_MS: u64 = 200;
const NAVIGATION_TIMEOUT_MS: u64 = 3000;

fn pps_status(message: PpsStatus, received_ns: u64) {
    let Some(header_epoch_ms) = message.header.gps_epoch_ms() else {
        return;
    };
    let Some(epoch_ms) = message.gps_epoch_ms() else {
        return;
    };
    timing::observe_pulse();
    timing::CLOCK.lock(|clock| {
        clock.borrow_mut().label(
            PpsLabel {
                epoch_ms,
                header_epoch_ms,
                output_delay_ns: u64::from(message.header.output_delay_ms) * 1_000_000,
                quality: message.quality,
                pulse_error_ns: message.pulse_error_ns,
                config: message.config,
            },
            received_ns,
        )
    });
}

fn sample(nav: Navigation, received_us: u64, received_ns: u64) -> Option<GnssSample> {
    let epochs = timing::CLOCK.lock(|clock| {
        let mut clock = clock.borrow_mut();
        Some((
            clock.epoch(nav.position_gps_ns, received_ns)?,
            clock.epoch(nav.velocity_gps_ns, received_ns)?,
        ))
    });
    let Some((position_epoch, velocity_epoch)) = epochs else {
        timing::UNSYNCHRONIZED.fetch_add(1, Ordering::Relaxed);
        return None;
    };
    Some(GnssSample {
        position_m: nav.position_m,
        velocity_mps: nav.velocity_mps,
        position_sigma_m: nav.position_sigma_m,
        velocity_sigma_mps: nav.velocity_sigma_mps,
        epoch_ms: nav.epoch_ms,
        received_us,
        position_time_ns: position_epoch.time_ns,
        velocity_time_ns: velocity_epoch.time_ns,
        timing_uncertainty_ns: position_epoch
            .uncertainty_ns
            .max(velocity_epoch.uncertainty_ns),
        clock_generation: position_epoch.generation,
        position_valid: nav.position_valid,
        velocity_valid: nav.velocity_valid,
        satellites: nav.satellites,
    })
}

fn baud(uart: &mut Uart<'_, Async>, rate: Baud) -> Result<(), ()> {
    uart.apply_config(
        &Config::default()
            .with_baudrate(rate.bits_per_second())
            .with_rx(
                RxConfig::default()
                    .with_fifo_full_threshold(aevia_device_hal::esp32s31::GNSS_RX_FIFO_THRESHOLD),
            ),
    )
    .map_err(|_| ())
}
async fn write(uart: &mut Uart<'_, Async>, bytes: &[u8]) -> Result<(), ()> {
    with_timeout(Duration::from_millis(500), async {
        uart.write_all(bytes).await.map_err(|_| ())?;
        embedded_io_async::Write::flush(uart).await.map_err(|_| ())
    })
    .await
    .map_err(|_| ())?
}
async fn send(uart: &mut Uart<'_, Async>, command: Command<'_>) -> Result<(), ()> {
    super::super::supervision::gnss_heartbeat();
    let mut bytes = [0; 128];
    let len = command.encode(&mut bytes).map_err(|_| ())?;
    write(uart, &bytes[..len]).await
}
fn drain(uart: &mut Uart<'_, Async>) {
    let mut bytes = [0; 128];
    for _ in 0..16 {
        if !matches!(uart.read_buffered(&mut bytes), Ok(n) if n > 0) {
            break;
        }
    }
}

// The assembled board's proven path is COM1 TX -> receiver -> COM2 RX.
// A CRC-valid VERSIONB on COM2 verifies it without requiring COM1 RX.
async fn probe(command: &mut Uart<'_, Async>, data: &mut Uart<'_, Async>) -> Result<(), ()> {
    for command_baud in [Baud::B115200, BAUD, Baud::B921600] {
        baud(command, command_baud)?;
        for data_baud in [Baud::B115200, BAUD, Baud::B921600] {
            baud(data, data_baud)?;
            drain(data);
            write(command, b"\r\n").await?;
            send(command, Command::Unlog { port: Port::Com2 }).await?;
            send(command, Command::Version { port: Port::Com2 }).await?;
            let found = with_timeout(Duration::from_millis(600), async {
                let mut decoder = BinaryDecoder::<512>::new();
                let mut bytes = [0; 128];
                loop {
                    match data.read_async(&mut bytes).await {
                        Ok(n) if n > 0 => {
                            transport::record_bytes(2, n);
                            for &byte in &bytes[..n] {
                                if matches!(decoder.push(byte), Ok(Some(f)) if f.message_id() == 37)
                                {
                                    return;
                                }
                            }
                            Timer::after_micros(100).await;
                        }
                        // A faulted UART must yield so timeout/watchdog tasks run.
                        Err(error) => {
                            transport::record_rx(2, error, Instant::now().as_micros());
                            Timer::after_millis(1).await;
                        }
                        _ => Timer::after_millis(1).await,
                    }
                }
            })
            .await
            .is_ok();
            if found {
                return Ok(());
            }
        }
    }
    Err(())
}

async fn configure(command: &mut Uart<'_, Async>, data: &mut Uart<'_, Async>) -> Result<(), ()> {
    probe(command, data).await?;
    send(command, Command::Raw("CONFIG SIGNALGROUP 8")).await?;
    Timer::after_secs(3).await; // Group changes restart the UM980.
    probe(command, data).await?;
    for setting in [
        Command::Unlog { port: Port::Com1 },
        Command::Unlog { port: Port::Com2 },
        Command::Raw("MODE ROVER AUTOMOTIVE"),
        Command::Raw("CONFIG PVTALG AUTO"),
        Command::Raw("CONFIG ANTIJAM AUTO"),
        Command::SetBaud {
            port: Port::Com2,
            baud: BAUD,
        },
    ] {
        send(command, setting).await?;
    }
    baud(data, BAUD)?;
    drain(data);
    send(command, Command::PpsGpsRisingOneHz).await?;
    // Readback is retried by reception, without pausing navigation decoding.
    for (message, milliseconds) in [
        (LogMessage::PpsStatusBinary, 1000),
        (LogMessage::BestNavXyzBinary, 20),
    ] {
        send(
            command,
            Command::Log {
                port: Port::Com2,
                message,
                period: Period::from_millis(milliseconds).map_err(|_| ())?,
            },
        )
        .await?;
    }
    Ok(())
}

fn uart_error() {
    sensors::GNSS_UART_ERRORS.fetch_add(1, Ordering::Relaxed);
    sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
}
fn parse_error(
    status: &mut super::GnssStatus,
    context: &ParserContext,
    port: usize,
    error: DecodeError,
    message_id: Option<u16>,
    received_us: u64,
) {
    sensors::GNSS_PARSE_ERRORS.fetch_add(1, Ordering::Relaxed);
    sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
    status.last_parser_error =
        Some(context.failure(port as u8 + 1, error, message_id, received_us));
    super::publish(*status);
}

#[embassy_executor::task]
pub async fn task(
    command: Uart<'static, esp_hal::Blocking>,
    data: Uart<'static, esp_hal::Blocking>,
    mut reset: esp_hal::gpio::Output<'static>,
) {
    // Install async UART handlers only after ownership reaches this executor.
    let (mut command, mut data) = super::super::board::activate_gnss(command, data);
    let mut status = super::GnssStatus::default();
    let mut failures = 0_u8;
    loop {
        super::super::supervision::gnss_heartbeat();
        if failures >= 3 {
            // Configuration/readback failure alone does not reach this path:
            // only lost navigation/transport liveness escalates a hard reset.
            reset.set_low();
            Timer::after_millis(5).await;
            reset.set_high();
            Timer::after_millis(500).await;
            failures = 0;
        }
        status.config_verified = false;
        status.navigation_time_ready = false;
        status.last_pps_quality = None;
        super::publish(status);
        sensors::GNSS_RATE_HZ.store(0, Ordering::Relaxed);
        timing::CLOCK.lock(|clock| clock.borrow_mut().invalidate());
        if configure(&mut command, &mut data).await.is_err() {
            failures = failures.saturating_add(1);
            uart_error();
            Timer::after_secs(3).await;
            continue;
        }
        let mut verification = PpsVerification::default();
        // Mixed decoders exclude binary payloads from CONFIG recognition on
        // both return paths, and preserve normal navigation during retries.
        let mut decoders = [Decoder::<512>::new(), Decoder::<512>::new()];
        let mut contexts = [ParserContext::new(), ParserContext::new()];
        let mut command_bytes = [0; 128];
        let mut data_bytes = [0; 128];
        let mut progress = EpochProgress::default();
        let mut last_navigation = Instant::now();
        let mut rate_start = Instant::now();
        let mut advancing = 0;
        let mut uart_failures = 0;
        let mut command_retry_at = Instant::now();
        let mut prefer_data = false;
        loop {
            super::super::supervision::gnss_heartbeat();
            // An idle deadline also discards plausible but truncated frames.
            // Continuous junk/PPS traffic cannot hide missing navigation.
            if last_navigation.elapsed().as_millis() >= NAVIGATION_TIMEOUT_MS {
                sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                break;
            }
            match verification.poll(Instant::now().as_micros()) {
                Action::Query => {
                    status.config_queries = status.config_queries.saturating_add(1);
                    // Only query COM1; never drive the unverified COM2 TX path.
                    if send(&mut command, Command::QueryConfig).await.is_err() {
                        uart_error();
                        verification.failed(Instant::now().as_micros());
                    }
                }
                Action::TimedOut => {
                    status.config_timeouts = status.config_timeouts.saturating_add(1);
                    sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                }
                Action::None => {}
            }
            if status.config_verified && !verification.verified() {
                timing::CLOCK.lock(|clock| clock.borrow_mut().invalidate());
            }
            status.config_verified = verification.verified();
            super::publish(status);
            let command_read = async {
                Timer::at(command_retry_at).await;
                command.read_async(&mut command_bytes).await
            };
            let data_read = data.read_async(&mut data_bytes);
            // select polls its first future first. read_async is cancellation
            // safe, but resolves immediately for even one buffered byte: the
            // FIFO threshold only controls waking an otherwise empty read.
            // A long CONFIG reply must not keep COM1 continuously preferred
            // while COM2's hardware-only FIFO fills with binary navigation.
            let received = with_timeout(Duration::from_millis(IDLE_TIMEOUT_MS), async {
                if prefer_data {
                    match select(data_read, command_read).await {
                        Either::First(result) => Either::Second(result),
                        Either::Second(result) => Either::First(result),
                    }
                } else {
                    select(command_read, data_read).await
                }
            })
            .await;
            let (port, bytes) = match received {
                Ok(Either::First(Ok(n))) if n > 0 => {
                    prefer_data = true;
                    (0, &command_bytes[..n])
                }
                Ok(Either::Second(Ok(n))) if n > 0 => {
                    prefer_data = false;
                    uart_failures = 0;
                    (1, &data_bytes[..n])
                }
                Ok(Either::First(result)) => {
                    prefer_data = true;
                    // An absent/noisy command return path cannot continually
                    // win select, starve COM2 or restart healthy navigation.
                    if let Err(error) = result {
                        transport::record_rx(1, error, Instant::now().as_micros());
                    }
                    uart_error();
                    decoders[0].reset();
                    command_retry_at = Instant::now() + Duration::from_millis(IDLE_TIMEOUT_MS);
                    Timer::after_micros(100).await;
                    continue;
                }
                Ok(Either::Second(result)) => {
                    prefer_data = false;
                    if let Err(error) = result {
                        transport::record_rx(2, error, Instant::now().as_micros());
                    }
                    uart_error();
                    decoders[1].reset();
                    uart_failures += 1;
                    if uart_failures >= 4 {
                        break;
                    }
                    Timer::after_millis(1).await;
                    continue;
                }
                Err(_) => {
                    for decoder in &mut decoders {
                        decoder.reset();
                    }
                    continue;
                }
            };
            transport::record_bytes(port as u8 + 1, bytes.len());
            let received = Instant::now().as_micros();
            let received_ns = timing::capture::now_ns();
            for &byte in bytes {
                contexts[port].push(byte);
                let frame = match decoders[port].push(byte) {
                    Ok(Some(frame)) => frame,
                    Ok(None) => continue,
                    Err(error) => {
                        parse_error(&mut status, &contexts[port], port, error, None, received);
                        continue;
                    }
                };
                let frame = match frame {
                    Frame::Ascii(line) => {
                        verification.observe(line, received);
                        if status.config_verified && !verification.verified() {
                            timing::CLOCK.lock(|clock| clock.borrow_mut().invalidate());
                        }
                        status.config_verified = verification.verified();
                        super::publish(status);
                        continue;
                    }
                    Frame::Binary(frame) => frame,
                };
                // Navigation counters remain available before verification,
                // but PPS labels and fusion stay gated on the exact readback.
                match frame.message_id() {
                    9000 => match frame.pps_status() {
                        Ok(message) => {
                            status.last_pps_quality = Some(message.quality);
                            super::publish(status);
                            if verification.verified()
                                && let Some(now) = received_ns
                            {
                                pps_status(message, now);
                            }
                        }
                        Err(error) => parse_error(
                            &mut status,
                            &contexts[port],
                            port,
                            error,
                            Some(frame.message_id()),
                            received,
                        ),
                    },
                    240 => {
                        let message = match frame.best_nav_xyz() {
                            Ok(message) => message,
                            Err(error) => {
                                parse_error(
                                    &mut status,
                                    &contexts[port],
                                    port,
                                    error,
                                    Some(frame.message_id()),
                                    received,
                                );
                                continue;
                            }
                        };
                        match progress.observe(message.header) {
                            EpochState::TimeUnavailable => {
                                // Cold acquisition can take longer than the
                                // liveness deadline. Never reset the receiver
                                // merely because trusted GPST is not ready.
                                failures = 0; // Valid cold-start traffic proves liveness.
                                status.navigation_time_ready = false;
                                super::publish(status);
                                last_navigation = Instant::now();
                                continue;
                            }
                            EpochState::Stalled => continue,
                            EpochState::Advancing(_) => {}
                        }
                        failures = 0;
                        status.navigation_time_ready = true;
                        super::publish(status);
                        last_navigation = Instant::now();
                        advancing += 1;
                        sensors::record_gnss(received);
                        let Some(nav) = Navigation::prepare(message) else {
                            continue;
                        };
                        let Some(now) = received_ns else {
                            timing::UNSYNCHRONIZED.fetch_add(1, Ordering::Relaxed);
                            continue;
                        };
                        if let Some(sample) = sample(nav, received, now) {
                            if sensors::GNSS.try_send(sample).is_err() {
                                sensors::GNSS_DROPS.fetch_add(1, Ordering::Relaxed);
                            }
                            sensors::observe_queue_depths();
                        }
                    }
                    _ => {}
                }
            }
            let elapsed = rate_start.elapsed().as_millis();
            if elapsed >= 1000 {
                sensors::GNSS_RATE_HZ.store((advancing * 1000 / elapsed) as u32, Ordering::Relaxed);
                advancing = 0;
                rate_start = Instant::now();
            }
            // Bound each interrupt-executor poll even if UART input remains
            // continuously ready. A real 100us suspension lets lower-priority
            // work run; 1us could expire before this task finished returning.
            // At 460800 baud fewer than five bytes arrive during the pause.
            Timer::after_micros(100).await;
        }
        failures = failures.saturating_add(1);
        timing::CLOCK.lock(|clock| clock.borrow_mut().invalidate());
        sensors::GNSS_RATE_HZ.store(0, Ordering::Relaxed);
        Timer::after_millis(100).await;
    }
}
