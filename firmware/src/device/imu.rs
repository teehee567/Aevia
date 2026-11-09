//! SCH16T-K01 SafeSPI, Murata Doc. 11624 Rev. 6. Factory calibration only.
use super::{sensors::ImuSample, timing::capture};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Input, Output},
    spi::master::Spi,
};

const COUNTER: u8 = 0x13;
// Murata Rev.6 Table 9: LPF3 group delay at 10Hz. These are nominal
// frequency-dependent values, not a per-board delay calibration.
const GYRO_DELAY_NS: u64 = 1_400_000;
const ACCEL_DELAY_NS: u64 = 1_500_000;
pub(super) const DELAY_UNCERTAINTY_NS: u64 = 1_200_000;
// DEC4 earliest update: 16/25.1kHz ~=637us. Finish safely before it.
const READ_DEADLINE_NS: u64 = 600_000;
// LPF3, DYN1, DEC4 on RATE2/ACC2: approximately 1.475 kHz.
const PROFILE: &[(u8, u16)] = &[
    (0x25, 0x00db),
    (0x26, 0x00db),
    (0x27, 0x00db),
    (0x28, 0x12db),
    (0x29, 0x12db),
    (0x2a, 0),
    (0x2b, 0),
    (0x2c, 0),
    (0x2d, 0),
    (0x2e, 0),
    (0x33, 0x202c),
    (0x34, 0x0ffe),
];

#[derive(Clone, Copy, Debug)]
pub enum Error {
    Spi,
    Frame,
    Status,
    Identity,
    Readback,
    Timeout,
    Gap,
    Range,
    Timing,
}

pub struct Imu<'d> {
    spi: Spi<'d, Blocking>,
    cs: Output<'d>,
    reset: Output<'d>,
    ready: Input<'d>,
    pending: Option<u8>,
    strict: bool,
    fault: bool,
    last: Option<([u8; 6], u64, [f64; 3])>,
}

impl<'d> Imu<'d> {
    pub fn new(
        spi: Spi<'d, Blocking>,
        cs: Output<'d>,
        reset: Output<'d>,
        ready: Input<'d>,
    ) -> Self {
        Self {
            spi,
            cs,
            reset,
            ready,
            pending: None,
            strict: false,
            fault: false,
            last: None,
        }
    }

    pub async fn initialize(&mut self) -> Result<(), Error> {
        self.strict = false;
        self.pending = None;
        self.last = None;
        self.cs.set_high();
        self.reset.set_low();
        Timer::after_millis(2).await;
        self.reset.set_high();
        Timer::after_millis(32).await;
        if self.read(0x3c)? != 0x23 || self.read(0x3b)? & 0x0f00 != 0 {
            return Err(Error::Identity);
        }
        self.write(0x37, 0x5a3c)?;
        self.verify(0x37, 0x5a3c)?;
        self.write(0x37, 0)?;
        self.verify(0x37, 0)?;
        for &(address, value) in PROFILE {
            self.write(address, value)?;
        }
        for &(address, value) in PROFILE {
            self.verify(address, value)?;
        }
        self.write(0x35, 1)?;
        self.verify(0x35, 1)?;
        Timer::after_millis(215).await;
        self.status()?; // Acknowledge latched startup faults before EOI.
        self.write(0x35, 3)?;
        self.verify(0x35, 3)?;
        let mut healthy = 0;
        for _ in 0..10 {
            Timer::after_millis(3).await;
            self.exchange(COUNTER, None)?; // Retire the pre-delay response.
            self.fault = false;
            let status = self.status()?;
            healthy = if status && !self.fault {
                healthy + 1
            } else {
                0
            };
            if healthy == 2 {
                break;
            }
        }
        if healthy != 2 {
            return Err(Error::Status);
        }
        self.strict = true;
        for &(address, value) in PROFILE {
            self.verify(address, value)?;
        }
        self.verify(0x35, 3)
    }

    /// Every native epoch is consumed. Missing counters invalidate the sample
    /// instead of integrating a stale value across the missing interval.
    pub async fn next_sample(&mut self) -> Result<ImuSample, Error> {
        loop {
            if with_timeout(Duration::from_millis(100), self.ready.wait_for_high())
                .await
                .is_err()
            {
                self.last = None;
                return Err(Error::Timeout);
            }
            let edge_ns = capture::drdy_ns().ok_or(Error::Timing)?;
            let frames = match self.read_axes() {
                Ok(frames) => frames,
                Err(error) => {
                    self.last = None;
                    return Err(error);
                }
            };
            let finished = capture::now_ns().ok_or(Error::Timing)?;
            // No association is accepted if the read crossed an update or
            // started too late. Drain all axes first so DRDY can rearm.
            if capture::drdy_ns() != Some(edge_ns)
                || finished.saturating_sub(edge_ns) > READ_DEADLINE_NS
            {
                self.last = None;
                return Err(Error::Timing);
            }
            let counter = frames.map(|f| ((f >> 29) & 15) as u8);
            let values = frames.map(|f| ((((f >> 8) as u32 & 0xfffff) << 12) as i32) >> 12);
            if values[..3].iter().any(|v| v.abs() > 300 * 1600)
                || values[3..].iter().any(|v| v.abs() > 80 * 3200)
            {
                self.last = None;
                return Err(Error::Range);
            }
            let gyro: [f64; 3] = core::array::from_fn(|i| {
                values[i] as f64 * (core::f64::consts::PI / (1600.0 * 180.0))
            });
            let previous = self.last.replace((counter, edge_ns, gyro));
            let Some((last_counter, last_ns, last_gyro)) = previous else {
                continue;
            };
            let duration = edge_ns.saturating_sub(last_ns);
            if !(600_000..=800_000).contains(&duration)
                || !last_counter
                    .into_iter()
                    .zip(counter)
                    .all(|(a, b)| (a + 1) & 15 == b)
            {
                // The current valid sample seeds the next consecutive pair.
                return Err(Error::Gap);
            }
            // Gyro's nominal filter delay is 100us shorter. Interpolate its
            // adjacent interval averages backwards onto acceleration's support;
            // assigning the raw vectors one common timestamp would misalign them.
            let fraction = (ACCEL_DELAY_NS - GYRO_DELAY_NS) as f64 / duration as f64;
            return Ok(ImuSample {
                captured_ns: edge_ns,
                end_ns: edge_ns.checked_sub(ACCEL_DELAY_NS).ok_or(Error::Timing)?,
                duration_ns: duration as u32,
                saturated: false,
                acceleration_mps2: core::array::from_fn(|i| values[i + 3] as f64 / 3200.0),
                angular_rate_rps: core::array::from_fn(|i| {
                    gyro[i] + fraction * (last_gyro[i] - gyro[i])
                }),
            });
        }
    }

    fn read_axes(&mut self) -> Result<[u64; 6], Error> {
        let mut frames = [0; 6];
        let mut error = self.exchange(0x0a, None).err();
        // Drain all six decimated axes even after a fault, allowing DRY to rearm.
        for (i, frame) in frames.iter_mut().enumerate() {
            match self.exchange(if i == 5 { COUNTER } else { 0x0b + i as u8 }, None) {
                Ok(Some(value)) if value >> 47 == 1 => *frame = value,
                Ok(_) => {
                    error.get_or_insert(Error::Frame);
                }
                Err(fault) => {
                    error.get_or_insert(fault);
                }
            }
        }
        error.map_or(Ok(frames), Err)
    }

    fn status(&mut self) -> Result<bool, Error> {
        let mut healthy = true;
        for address in 0x14..=0x1d {
            healthy &= self.read(address)? == 0xffff;
        }
        Ok(healthy)
    }
    fn verify(&mut self, address: u8, value: u16) -> Result<(), Error> {
        if self.read(address)? == value {
            Ok(())
        } else {
            Err(Error::Readback)
        }
    }
    fn read(&mut self, address: u8) -> Result<u16, Error> {
        self.exchange(address, None)?;
        let frame = self.exchange(COUNTER, None)?.ok_or(Error::Spi)?;
        if frame >> 47 != 0 {
            return Err(Error::Frame);
        }
        Ok((frame >> 8) as u16)
    }
    fn write(&mut self, address: u8, value: u16) -> Result<(), Error> {
        self.exchange(address, Some(value))?;
        self.exchange(COUNTER, None)?;
        Ok(())
    }
    fn exchange(&mut self, address: u8, value: Option<u16>) -> Result<Option<u64>, Error> {
        let mut frame = (u64::from(address) << 38) | (1 << 35);
        if let Some(value) = value {
            frame |= (1 << 37) | (u64::from(value) << 8);
        }
        frame |= u64::from(crc8(frame));
        let bytes = frame.to_be_bytes();
        let mut wire = [bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]];
        let delay = Delay::new();
        delay.delay_micros(1);
        self.cs.set_low();
        delay.delay_micros(1);
        let result = self.spi.transfer(&mut wire);
        delay.delay_micros(1);
        self.cs.set_high();
        if result.is_err() {
            self.pending = None;
            return Err(Error::Spi);
        }
        let Some(expected) = self.pending.replace(address) else {
            return Ok(None);
        };
        let frame =
            u64::from_be_bytes([0, 0, wire[0], wire[1], wire[2], wire[3], wire[4], wire[5]]);
        if frame == 0
            || frame == 0xffffffffffff
            || frame as u8 != crc8(frame)
            || ((frame >> 37) & 0x3ff) != u64::from(expected)
            || frame & (1 << 35) != 0
        {
            return Err(Error::Frame);
        }
        let fault = frame & ((1 << 36) | (3 << 33)) != 0;
        self.fault |= fault;
        if self.strict && fault {
            return Err(Error::Status);
        }
        Ok(Some(frame))
    }
}

// Murata's augmented reference algorithm shifts all 48 bits, including zeros
// in the CRC byte; this is equivalent to the published polynomial form.
fn crc8(frame: u64) -> u8 {
    let mut crc = 0xff_u8;
    for bit in (0..48).rev() {
        let data = (((frame & 0xffffffffff00) >> bit) & 1) as u8;
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ 0x2f ^ data
        } else {
            (crc << 1) | data
        };
    }
    crc
}
