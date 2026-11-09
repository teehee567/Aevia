//! UM980 N4 binary BESTNAVXYZ (ID 240), fixed 24-byte header and CRC32.
//! Command reference: Unicore N4 R1.14, binary header and BESTNAVXYZ tables.
use super::{
    sensors::{self, GnssSample},
    timing::{self, PpsLabel},
};
use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Write;
use esp_hal::{
    Async,
    uart::{Config, RxConfig, Uart},
};

const BAUD: u32 = 460_800;
const SYNC: [u8; 3] = [0xaa, 0x44, 0xb5];

/// Bounded decoder; rejects oversized lengths before buffering a payload.
struct Decoder {
    bytes: [u8; 512],
    len: usize,
    total: usize,
}
impl Decoder {
    const fn new() -> Self {
        Self {
            bytes: [0; 512],
            len: 0,
            total: 0,
        }
    }
    fn push(&mut self, byte: u8) -> Option<&[u8]> {
        if self.len < 3 {
            if byte == SYNC[self.len] {
                self.bytes[self.len] = byte;
                self.len += 1;
            } else {
                self.len = usize::from(byte == SYNC[0]);
            }
            return None;
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        if self.len == 8 {
            self.total = 24 + u16_at(&self.bytes, 6) as usize + 4;
            if self.total > self.bytes.len() {
                sensors::GNSS_PARSE_ERRORS.fetch_add(1, Ordering::Relaxed);
                sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                self.len = 0;
                return None;
            }
        }
        if self.len >= 8 && self.len == self.total {
            let total = self.total;
            self.len = 0;
            if crc32(&self.bytes[..total - 4]) == u32_at(&self.bytes, total - 4) {
                return Some(&self.bytes[..total - 4]);
            }
            sensors::GNSS_PARSE_ERRORS.fetch_add(1, Ordering::Relaxed);
            sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        None
    }
}

fn header_epoch_ms(frame: &[u8]) -> Option<u64> {
    // N4 time reference 0=GPST; FINE=160. Never map BDST/UTC as GPST.
    if frame[8] != 0 || frame[9] != 160 {
        return None;
    }
    timing::gps_ms(u32::from(u16_at(frame, 10)), u32_at(frame, 12))
}

fn pps_status(frame: &[u8], received_ns: u64) {
    if frame.len() != 24 + 60 {
        return;
    }
    let Some(header_epoch_ms) = header_epoch_ms(frame) else {
        return;
    };
    let p = &frame[24..];
    let Some(epoch_ms) = timing::gps_ms(u32_at(p, 4), u32_at(p, 8)) else {
        return;
    };
    timing::observe_pulse();
    timing::CLOCK.lock(|clock| {
        clock.borrow_mut().label(
            PpsLabel {
                epoch_ms,
                header_epoch_ms,
                output_delay_ns: u64::from(u16_at(frame, 22)) * 1_000_000,
                quality: u32_at(p, 0),
                pulse_error_ns: i32::from_le_bytes(p[12..16].try_into().unwrap()),
                config: u32_at(p, 20),
            },
            received_ns,
        )
    });
}

fn sample(frame: &[u8], received_us: u64, received_ns: u64) -> Option<GnssSample> {
    if u16_at(frame, 4) != 240 || frame.len() != 24 + 112 {
        return None;
    }
    let p = &frame[24..];
    let position_m = core::array::from_fn(|i| f64_at(p, 8 + i * 8));
    let velocity_mps = core::array::from_fn(|i| f64_at(p, 52 + i * 8));
    let position_sigma_m = core::array::from_fn(|i| f32_at(p, 32 + i * 4) as f64);
    let velocity_sigma_mps = core::array::from_fn(|i| f32_at(p, 76 + i * 4) as f64);
    let latency = f32_at(p, 92);
    let epoch_ms = header_epoch_ms(frame)?;
    let position_valid = u32_at(p, 0) == 0
        && u32_at(p, 4) != 0
        && position_m.iter().all(|v| v.is_finite())
        && position_sigma_m.iter().all(|v| v.is_finite() && *v > 0.0);
    let velocity_valid = u32_at(p, 44) == 0
        && u32_at(p, 48) != 0
        && velocity_mps.iter().all(|v| v.is_finite())
        && velocity_sigma_mps.iter().all(|v| v.is_finite() && *v > 0.0)
        && latency.is_finite()
        && (0.0..=1.0).contains(&latency);
    let position_gps_ns = epoch_ms.checked_mul(1_000_000)?;
    let latency_ns = if velocity_valid {
        (f64::from(latency) * 1_000_000_000.0) as u64
    } else {
        0
    };
    let epochs = timing::CLOCK.lock(|clock| {
        let mut clock = clock.borrow_mut();
        Some((
            clock.epoch(position_gps_ns, received_ns)?,
            clock.epoch(position_gps_ns.checked_sub(latency_ns)?, received_ns)?,
        ))
    });
    let Some((position_epoch, velocity_epoch)) = epochs else {
        timing::UNSYNCHRONIZED.fetch_add(1, Ordering::Relaxed);
        return None;
    };
    Some(GnssSample {
        position_m,
        velocity_mps,
        position_sigma_m,
        velocity_sigma_mps,
        epoch_ms,
        received_us,
        position_time_ns: position_epoch.time_ns,
        velocity_time_ns: velocity_epoch.time_ns,
        timing_uncertainty_ns: position_epoch
            .uncertainty_ns
            .max(velocity_epoch.uncertainty_ns),
        clock_generation: position_epoch.generation,
        position_valid,
        velocity_valid,
        satellites: p[105],
    })
}

fn baud(uart: &mut Uart<'_, Async>, rate: u32) -> Result<(), ()> {
    uart.apply_config(
        &Config::default()
            .with_baudrate(rate)
            .with_rx(RxConfig::default().with_fifo_full_threshold(32)),
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
fn drain(uart: &mut Uart<'_, Async>) {
    let mut bytes = [0; 128];
    for _ in 0..16 {
        if !matches!(uart.read_buffered(&mut bytes), Ok(n) if n>0) {
            break;
        }
    }
}

// The assembled board's proven path is COM1 TX -> receiver -> COM2 RX.
// A CRC-valid VERSIONB on COM2 verifies it without requiring COM1 RX.
async fn probe(command: &mut Uart<'_, Async>, data: &mut Uart<'_, Async>) -> Result<(), ()> {
    for command_baud in [115_200, BAUD, 921_600] {
        baud(command, command_baud)?;
        for data_baud in [115_200, BAUD, 921_600] {
            baud(data, data_baud)?;
            drain(data);
            write(command, b"\r\nUNLOG COM2\r\nVERSIONB COM2\r\n").await?;
            if with_timeout(Duration::from_millis(600), async {
                let mut decoder = Decoder::new();
                let mut bytes = [0; 128];
                loop {
                    if let Ok(n) = data.read_async(&mut bytes).await {
                        for &byte in &bytes[..n] {
                            if decoder.push(byte).is_some_and(|f| u16_at(f, 4) == 37) {
                                return;
                            }
                        }
                    }
                }
            })
            .await
            .is_ok()
            {
                return Ok(());
            }
        }
    }
    Err(())
}

async fn configure(command: &mut Uart<'_, Async>, data: &mut Uart<'_, Async>) -> Result<(), ()> {
    probe(command, data).await?;
    write(command, b"CONFIG SIGNALGROUP 8\r\n").await?;
    Timer::after_secs(3).await; // Group changes restart the UM980.
    probe(command, data).await?;
    write(command,b"UNLOG COM1\r\nUNLOG COM2\r\nMODE ROVER AUTOMOTIVE\r\nCONFIG PVTALG AUTO\r\nCONFIG ANTIJAM AUTO\r\nCONFIG COM2 460800\r\n").await?;
    baud(data, BAUD)?;
    drain(data);
    // Rising edge at each GPST integer second, 100ms pulse, zero user/RF
    // compensation. Read PPSSTATUS back on the data port to verify the profile.
    write(command, timing::PPS_COMMAND).await?;
    // CONFIG replies on the querying port. Query both wired ports to tolerate
    // the assembled board's unreliable COM1 return path.
    write(command, b"CONFIG\r\n").await?;
    write(data, b"CONFIG\r\n").await?;
    with_timeout(Duration::from_secs(2), verify_pps_config(command, data))
        .await
        .map_err(|_| ())??;
    write(command, b"PPSSTATUSB COM2 1\r\nBESTNAVXYZB COM2 0.02\r\n").await
}

async fn verify_pps_config(
    command: &mut Uart<'_, Async>,
    data: &mut Uart<'_, Async>,
) -> Result<(), ()> {
    let mut command_bytes = [0; 128];
    let mut data_bytes = [0; 128];
    let mut lines = [[0; 96]; 2];
    let mut lengths = [0; 2];
    loop {
        let (port, bytes) = match select(
            command.read_async(&mut command_bytes),
            data.read_async(&mut data_bytes),
        )
        .await
        {
            Either::First(Ok(n)) => (0, &command_bytes[..n]),
            Either::Second(Ok(n)) => (1, &data_bytes[..n]),
            _ => {
                // A disconnected/faulted port must not spin inside the
                // configuration timeout and starve acquisition/watchdog tasks.
                Timer::after_millis(1).await;
                continue;
            }
        };
        for &byte in bytes {
            if byte == b'\n' {
                let len = core::mem::take(&mut lengths[port]);
                if len <= lines[port].len() && timing::valid_pps_config(&lines[port][..len]) {
                    return Ok(());
                }
            } else {
                let len = lengths[port];
                if len < lines[port].len() {
                    lines[port][len] = byte;
                }
                lengths[port] = (len + 1).min(lines[port].len() + 1);
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(
    mut command: Uart<'static, Async>,
    mut data: Uart<'static, Async>,
    _reset: esp_hal::gpio::Input<'static>,
) {
    loop {
        timing::CLOCK.lock(|clock| clock.borrow_mut().invalidate());
        if configure(&mut command, &mut data).await.is_err() {
            sensors::GNSS_UART_ERRORS.fetch_add(1, Ordering::Relaxed);
            sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
            Timer::after_secs(3).await;
            continue;
        }
        let mut decoder = Decoder::new();
        let mut bytes = [0; 256];
        let mut last_epoch = None;
        let mut rate_start = Instant::now();
        let mut advancing = 0;
        loop {
            // Processing and consumers never backpressure this receive loop.
            let n = match with_timeout(Duration::from_secs(3), data.read_async(&mut bytes)).await {
                Ok(Ok(n)) => n,
                Ok(Err(_)) => {
                    sensors::GNSS_UART_ERRORS.fetch_add(1, Ordering::Relaxed);
                    sensors::GNSS_ERRORS.fetch_add(1, Ordering::Relaxed);
                    decoder = Decoder::new();
                    continue;
                }
                Err(_) => {
                    sensors::GNSS_RATE_HZ.store(0, Ordering::Relaxed);
                    break;
                }
            };
            let received = Instant::now().as_micros();
            let Some(received_ns) = timing::capture::now_ns() else {
                continue;
            };
            for &byte in &bytes[..n] {
                let Some(frame) = decoder.push(byte) else {
                    continue;
                };
                if u16_at(frame, 4) == 9000 {
                    pps_status(frame, received_ns);
                    continue;
                }
                if u16_at(frame, 4) != 240 || frame.len() != 24 + 112 {
                    continue;
                }
                let Some(epoch_ms) = header_epoch_ms(frame) else {
                    continue;
                };
                if last_epoch.is_some_and(|last| epoch_ms <= last) {
                    continue;
                }
                last_epoch = Some(epoch_ms);
                advancing += 1;
                sensors::GNSS_SAMPLES.fetch_add(1, Ordering::Relaxed);
                let Some(sample) = sample(frame, received, received_ns) else {
                    continue;
                };
                if sensors::GNSS.try_send(sample).is_err() {
                    sensors::GNSS_DROPS.fetch_add(1, Ordering::Relaxed);
                }
            }
            let elapsed = rate_start.elapsed().as_millis();
            if elapsed >= 1000 {
                sensors::GNSS_RATE_HZ.store((advancing * 1000 / elapsed) as u32, Ordering::Relaxed);
                advancing = 0;
                rate_start = Instant::now();
            }
        }
    }
}

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(b[o..o + 2].try_into().unwrap())
}
fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn f32_at(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn f64_at(b: &[u8], o: usize) -> f64 {
    f64::from_le_bytes(b[o..o + 8].try_into().unwrap())
}
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    crc
}
