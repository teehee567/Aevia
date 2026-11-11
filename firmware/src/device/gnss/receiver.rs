//! Asynchronous receiver lifecycle and acquisition. N4 framing, typed messages
//! and command encoding come from the HAL; timestamp/fusion policy stays here.
use super::super::{
    sensors::{self, GnssSample},
    timing::{self, PpsLabel},
};
use super::{
    diagnostics::ParserContext,
    navigation::{EpochProgress, EpochState, Navigation},
    recording, transport,
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
// The UART has only a 128-byte hardware FIFO. At 460800 baud, the 32-byte
// wake threshold leaves about 2 ms to service it. Drain consecutive available
// reads before suspending, rather than adding a timer after every small read.
// Both limits are checked between reads, keeping this interrupt executor's
// work bounded even under continuous input. Higher-priority IMU work remains
// preemptible throughout; a real 100 us suspension admits lower-priority work.
const RECEIVE_BURST_BYTES: usize = 512;
const RECEIVE_BURST_US: u64 = 500;
// N4 R1.2 §§7.3.2/7.3.16–21. Keep raw observations at 1Hz on the proven
// 460800-baud port. PVT remains 50Hz. BD3EPH is explicitly unsupported in
// that manual and is not silently claimed. Actual CRC-valid log IDs are counted.
const OPTIONAL_LOGS: [&str; 8] = [
    "OBSVMB COM2 1",
    "GPSEPHB COM2 ONCHANGED",
    "GLOEPHB COM2 ONCHANGED",
    "BDSEPHB COM2 ONCHANGED",
    "GALEPHB COM2 ONCHANGED",
    "QZSSEPHB COM2 ONCHANGED",
    "VERSIONB COM2",
    "CONFIG",
];

fn pps_status(message: PpsStatus, received_ns: u64) {
    let Some(header_epoch_ms) = message.header.gps_epoch_ms() else {
        return;
    };
    let Some(epoch_ms) = message.gps_epoch_ms() else {
        return;
    };
    timing::observe_pulse();
    let _ = timing::CLOCK.update(|clock| {
        clock.label(
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
    let epochs = timing::CLOCK.update(|clock| {
        let position_epoch = clock.epoch(nav.position_gps_ns, received_ns)?;
        // The fitted receiver normally reports zero velocity latency. Avoid
        // repeating the software-wide projection
        // when both independently validated fields share the same GPS epoch.
        let velocity_epoch = if nav.velocity_gps_ns == nav.position_gps_ns {
            position_epoch
        } else {
            clock.epoch(nav.velocity_gps_ns, received_ns)?
        };
        Some((position_epoch, velocity_epoch))
    });
    let Ok(Some((position_epoch, velocity_epoch))) = epochs else {
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
async fn write(uart: &mut Uart<'_, Async>, bytes: &[u8], flush: bool) -> Result<(), ()> {
    recording::uart(
        8,
        bytes,
        Instant::now().as_micros(),
        timing::capture::now_ns(),
    );
    let result = with_timeout(Duration::from_millis(500), async {
        uart.write_all(bytes).await.map_err(|_| ())?;
        if flush {
            embedded_io_async::Write::flush(uart)
                .await
                .map_err(|_| ())?;
        }
        Ok(())
    })
    .await
    .map_err(|_| ())
    .and_then(|result| result);
    if result.is_err() {
        recording::fault(1, 3, 0);
    }
    result
}
async fn send(uart: &mut Uart<'_, Async>, command: Command<'_>) -> Result<(), ()> {
    transmit(uart, command, true).await
}
async fn send_queued(uart: &mut Uart<'_, Async>, command: Command<'_>) -> Result<(), ()> {
    // COM1 is commonly 115200 baud: a 26-byte command takes 2.26 ms on the wire,
    // longer than COM2's FIFO service margin. Queue the complete command and
    // resume RX immediately; hardware preserves FIFO ordering. This records
    // an attempted write, never a receiver ACK. Setup still uses flushed sends.
    transmit(uart, command, false).await
}
async fn transmit(uart: &mut Uart<'_, Async>, command: Command<'_>, flush: bool) -> Result<(), ()> {
    super::super::supervision::gnss_heartbeat();
    let mut bytes = [0; 128];
    let len = command.encode(&mut bytes).map_err(|_| ())?;
    write(uart, &bytes[..len], flush).await
}
fn drain(uart: &mut Uart<'_, Async>) {
    let mut bytes = [0; 128];
    for _ in 0..16 {
        match uart.read_buffered(&mut bytes) {
            Ok(n) if n > 0 => recording::uart(
                3,
                &bytes[..n],
                Instant::now().as_micros(),
                timing::capture::now_ns(),
            ),
            _ => break,
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
            write(command, b"\r\n", true).await?;
            send(command, Command::Unlog { port: Port::Com2 }).await?;
            send(command, Command::Version { port: Port::Com2 }).await?;
            let found = with_timeout(Duration::from_millis(600), async {
                let mut decoder = BinaryDecoder::<512>::new();
                let mut bytes = [0; 128];
                loop {
                    match data.read_async(&mut bytes).await {
                        Ok(n) if n > 0 => {
                            recording::uart(
                                3,
                                &bytes[..n],
                                Instant::now().as_micros(),
                                timing::capture::now_ns(),
                            );
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
    // A live-loop command may still be in the TX FIFO when recovery begins.
    // Complete it before probe changes COM1 baud; do not truncate queued data.
    with_timeout(
        Duration::from_millis(500),
        embedded_io_async::Write::flush(command),
    )
    .await
    .map_err(|_| ())?
    .map_err(|_| ())?;
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
    recording::fault(0, 1, 0);
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
    recording::fault(
        port as u8 + 1,
        2,
        match error {
            DecodeError::BufferTooSmall => 1,
            DecodeError::Oversized => 2,
            DecodeError::Length => 3,
            DecodeError::Sync => 4,
            DecodeError::Checksum => 5,
            DecodeError::InvalidAscii => 6,
            DecodeError::MessageType => 7,
        },
    );
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
    // Coverage belongs to the current file generation, not a receiver
    // configuration attempt. Retain it across UART/parser recovery.
    let mut raw_status = recording::RawStatus::default();
    recording::publish(raw_status, timing::capture::now_ns().unwrap_or(0));
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
        super::fix::invalidate();
        status.navigation_time_ready = false;
        status.last_pps_quality = None;
        super::publish(status);
        sensors::GNSS_RATE_HZ.store(0, Ordering::Relaxed);
        let _ = timing::CLOCK.update(timing::ClockSync::invalidate);
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
        for port in 0..2 {
            raw_status.complete_frame(port); // Earlier partial framing was discarded by setup.
        }
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
        let mut optional_index = 0;
        let mut optional_after = Instant::now();
        let mut burst_bytes = 0;
        let mut burst_started = Instant::now();
        loop {
            super::super::supervision::gnss_heartbeat();
            let requested = recording::bootstrap_generation();
            if raw_status.begin_generation(
                requested,
                decoders.each_ref().map(|d| d.has_partial_frame()),
            ) {
                optional_index = 0;
                optional_after = Instant::now();
                recording::publish(raw_status, timing::capture::now_ns().unwrap_or(0));
            }
            // An idle deadline also discards plausible but truncated frames.
            // Continuous junk/PPS traffic cannot hide missing navigation.
            if last_navigation.elapsed().as_millis() >= NAVIGATION_TIMEOUT_MS {
                sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                break;
            }
            let mut status_changed = true;
            match verification.poll(Instant::now().as_micros()) {
                Action::Query => {
                    status.config_queries = status.config_queries.saturating_add(1);
                    // Only query COM1; never drive the unverified COM2 TX path.
                    if send_queued(&mut command, Command::QueryConfig)
                        .await
                        .is_err()
                    {
                        uart_error();
                        verification.failed(Instant::now().as_micros());
                    }
                }
                Action::TimedOut => {
                    status.config_timeouts = status.config_timeouts.saturating_add(1);
                    sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                }
                Action::None => status_changed = false,
            }
            if status.config_verified && !verification.verified() {
                let _ = timing::CLOCK.update(timing::ClockSync::invalidate);
            }
            status_changed |= status.config_verified != verification.verified();
            status.config_verified = verification.verified();
            if status_changed {
                super::publish(status);
            }
            // Install optional raw logs gradually while continuously draining
            // COM2. A burst of ephemerides must not fill the FIFO while setup
            // sends multiple commands without servicing reception.
            if optional_index < OPTIONAL_LOGS.len() && Instant::now() >= optional_after {
                if send_queued(&mut command, Command::Raw(OPTIONAL_LOGS[optional_index]))
                    .await
                    .is_err()
                {
                    raw_status.optional_write_failures =
                        raw_status.optional_write_failures.saturating_add(1);
                }
                optional_index += 1;
                // Cached ephemerides can contain many satellites. Half a
                // second between requests leaves UART bandwidth for PVT/raw
                // observations while the receiver drains the preceding burst.
                optional_after = Instant::now() + Duration::from_millis(500);
            }
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
                    // Prefer COM2 and yield briefly so a noisy command return
                    // path cannot monopolize the executor. A 200 ms exclusion
                    // discarded the 921-byte CONFIG reply (~80 ms at 115200)
                    // after one error by letting the 128-byte FIFO overflow.
                    if let Err(error) = result {
                        transport::record_rx(1, error, Instant::now().as_micros());
                    }
                    uart_error();
                    decoders[0].reset();
                    raw_status.complete_frame(0);
                    command_retry_at = Instant::now() + Duration::from_micros(100);
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
                    raw_status.complete_frame(1);
                    uart_failures += 1;
                    if uart_failures >= 4 {
                        break;
                    }
                    // Yield without spending half the 128-byte FIFO service
                    // margin idle after an error (about 46 bytes arrive per ms).
                    Timer::after_micros(100).await;
                    continue;
                }
                Err(_) => {
                    for (port, decoder) in decoders.iter_mut().enumerate() {
                        decoder.reset();
                        raw_status.complete_frame(port);
                    }
                    continue;
                }
            };
            transport::record_bytes(port as u8 + 1, bytes.len());
            let received = Instant::now().as_micros();
            let received_ns = timing::capture::now_ns();
            recording::uart(if port == 1 { 3 } else { 4 }, bytes, received, received_ns);
            for &byte in bytes {
                contexts[port].push(byte);
                let frame = match decoders[port].push(byte) {
                    Ok(Some(frame)) => frame,
                    Ok(None) => continue,
                    Err(error) => {
                        raw_status.complete_frame(port);
                        parse_error(&mut status, &contexts[port], port, error, None, received);
                        continue;
                    }
                };
                let credit_frame = raw_status.complete_frame(port);
                let frame = match frame {
                    Frame::SkippedBinary { header, .. } => {
                        if credit_frame {
                            raw_status.observe(header, received);
                        }
                        continue;
                    }
                    Frame::Ascii(line) => {
                        verification.observe(line, received);
                        if status.config_verified && !verification.verified() {
                            let _ = timing::CLOCK.update(timing::ClockSync::invalidate);
                        }
                        status.config_verified = verification.verified();
                        super::publish(status);
                        continue;
                    }
                    Frame::Binary(frame) => frame,
                };
                if credit_frame {
                    raw_status.observe(frame.header(), received);
                }
                // Navigation counters remain available before verification,
                // but PPS labels and fusion stay gated on the exact readback.
                match frame.message_id() {
                    9000 => match frame.pps_status() {
                        Ok(message) => {
                            super::fix::pps(
                                message,
                                received,
                                received_ns,
                                timing::capture::pps_ns(),
                            );
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
                        super::fix::navigation(message, received);
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
                        if super::super::configuration::DATA_COLLECTION_ONLY {
                            continue;
                        }
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
                recording::publish(raw_status, received_ns.unwrap_or(0));
                sensors::GNSS_RATE_HZ.store((advancing * 1000 / elapsed) as u32, Ordering::Relaxed);
                advancing = 0;
                rate_start = Instant::now();
            }
            burst_bytes += bytes.len();
            if bytes.len() < usize::from(aevia_device_hal::esp32s31::GNSS_RX_FIFO_THRESHOLD)
                || burst_bytes >= RECEIVE_BURST_BYTES
                || burst_started.elapsed().as_micros() >= RECEIVE_BURST_US
            {
                // Continue immediate fair COM1/COM2 reads within this budget.
                // A short read means the FIFO was nearly drained: coalesce
                // new bytes rather than spinning into one-byte log records.
                // At line rate fewer than five bytes arrive in the suspension;
                // pending input is drained again before another such pause.
                Timer::after_micros(100).await;
                burst_bytes = 0;
                burst_started = Instant::now();
            }
        }
        failures = failures.saturating_add(1);
        let _ = timing::CLOCK.update(timing::ClockSync::invalidate);
        sensors::GNSS_RATE_HZ.store(0, Ordering::Relaxed);
        Timer::after_millis(100).await;
    }
}
