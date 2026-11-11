//! Murata SCH16T-K01 six-axis IMU and die temperature sensor.
//!
//! Source: `data/datasheets/sch16t-k01-datasheet-full.pdf`, Doc. 11624 Rev. 6,
//! especially §§5.2, 5.4, 6 and 7. Every defined public register is accessible;
//! the driver uses 48-bit SafeSPI for full-resolution data. Configure `SpiDevice`
//! for mode 0 at 0.095–10.5 MHz (up to 25.5 MHz only after enabling high speed).
//! A transaction holds CS active over exactly one six-byte frame. SCK must
//! remain low for the 32 ms NVM interval, so a shared bus needs caller exclusion
//! across reset/startup. The driver adds the required inter-frame timing.
//!
//! `initialize` performs one startup attempt and returns failures for the caller
//! to retry/reset with an application-specific limit. Timestamp capture, DRDY or
//! SYNC GPIO ownership, mounting, system calibration, scheduling and filter-delay
//! correction belong to the application. A register burst is not an atomic epoch;
//! retain the returned counters and use DRDY/SYNC and a host timing deadline.

mod config;
mod protocol;
mod registers;

pub use config::*;
pub use protocol::{FrameError, Response, SensorStatus, SlaveAddress, crc8};
pub use registers::Register;

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};
use protocol::Request;
use registers::STATUS_REGISTERS;

const DEGREES_TO_RADIANS: f64 = core::f64::consts::PI / 180.0;

#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    Frame(FrameError),
    Configuration(ConfigError),
    ReadOnly(Register),
    InvalidRegisterValue(Register),
    /// EOI prevents changing settings until reset.
    ConfigurationLocked,
    Identity {
        component: u16,
        asic: u16,
    },
    Readback {
        register: Register,
        expected: u16,
        actual: u16,
    },
    StartupStatus(Status),
    SensorStatus {
        register: Register,
        status: SensorStatus,
        common_error: bool,
        /// Unmodified 20-bit payload, including when sensor health rejects it.
        payload: u32,
        counter: u8,
    },
    NotInitialized,
    Pipeline,
}

/// Status flags are active-low internally. Reading them clears latched faults
/// after the configured clearance delay (§§5.6 and 7.3).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Status {
    /// STAT_SUM, STAT_SUM_SAT, STAT_COM, STAT_RATE_COM, RATE_X/Y/Z, ACC_X/Y/Z.
    pub registers: [u16; 10],
    /// Bit i marks non-normal/IDS diagnostic metadata in response i.
    pub frame_faults: u16,
}

impl Status {
    pub fn healthy(self) -> bool {
        self.frame_faults == 0 && self.registers.iter().all(|value| *value == 0xffff)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity {
    pub component: u16,
    pub asic: u16,
    /// SN_ID1, SN_ID2, SN_ID3; format described by datasheet Table 83.
    pub serial: [u16; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionOutput {
    Interpolated,
    Decimated,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionSample {
    pub raw_rate: [i32; 3],
    pub raw_acceleration: [i32; 3],
    pub angular_rate_dps: [f64; 3],
    pub angular_rate_rps: [f64; 3],
    pub acceleration_mps2: [f64; 3],
    /// RATE_X/Y/Z then ACC_X/Y/Z. Valid only for decimated output.
    pub counters: [u8; 6],
}

/// Native evidence for all axes, including healthy neighbors of a faulted
/// axis. None identifies a response lost to transport/framing/pipeline error.
/// `result` keeps the normal checked navigation contract unchanged.
pub struct MotionCapture<E> {
    pub frames: [Option<Response>; 6],
    pub result: Result<MotionSample, Error<E>>,
}

/// Completed, CRC/status-checked register reads before floating-point scaling.
/// Capture the host's read-finished timestamp after this is returned, then call
/// `into_scaled`; conversion performs no SPI operations or hardware accesses.
pub struct UnscaledMotionCapture<E> {
    pub frames: [Option<Response>; 6],
    pub result: Result<UnscaledMotionSample, Error<E>>,
}

/// Native words and the verified ranges in effect when they were read.
/// Keeping the scales in this value prevents later reconfiguration from
/// changing the interpretation of an already captured sample.
#[derive(Clone, Copy, Debug)]
pub struct UnscaledMotionSample {
    pub raw_rate: [i32; 3],
    pub raw_acceleration: [i32; 3],
    pub counters: [u8; 6],
    rate_scale: f64,
    acc_scale: f64,
}

impl UnscaledMotionSample {
    pub fn into_scaled(self) -> MotionSample {
        let angular_rate_dps = self.raw_rate.map(|v| f64::from(v) * self.rate_scale);
        MotionSample {
            raw_rate: self.raw_rate,
            raw_acceleration: self.raw_acceleration,
            angular_rate_dps,
            angular_rate_rps: angular_rate_dps.map(|v| v * DEGREES_TO_RADIANS),
            acceleration_mps2: self.raw_acceleration.map(|v| f64::from(v) * self.acc_scale),
            counters: self.counters,
        }
    }
}

impl<E> UnscaledMotionCapture<E> {
    pub fn into_scaled(self) -> MotionCapture<E> {
        MotionCapture {
            frames: self.frames,
            result: self.result.map(UnscaledMotionSample::into_scaled),
        }
    }
}

impl MotionSample {
    /// Per-axis modulo-16 advance; zero means repeated output, one means the
    /// next sample, and 2..15 indicate skipped samples (or a wrapped interval).
    /// Counters cannot detect gaps of sixteen or more complete sample periods.
    pub fn counter_advance(&self, previous: &Self) -> [u8; 6] {
        core::array::from_fn(|i| self.counters[i].wrapping_sub(previous.counters[i]) & 15)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Counters {
    pub rate: [u8; 3],
    pub acceleration: [u8; 3],
    /// 14-bit frequency counter, nominally 1.5104 MHz, wraps at 16384.
    pub frequency: u16,
}

/// Reset pin helper; caller controls the pin and ensures supply stabilization.
/// Hold reset low for >=1 ms, release, then leave SCK low for >=32 ms.
pub fn hardware_reset<P: OutputPin, D: DelayNs>(
    pin: &mut P,
    delay: &mut D,
) -> Result<(), P::Error> {
    pin.set_low()?;
    delay.delay_ms(1);
    pin.set_high()?;
    delay.delay_ms(32);
    Ok(())
}

/// Owns an SPI device and delay provider. No allocation and no I/O in `new`.
pub struct Sch16t<SPI, D> {
    spi: SPI,
    delay: D,
    address: SlaveAddress,
    pending: Option<Request>,
    config: Option<Config>,
    locked: bool,
}

impl<SPI, D> Sch16t<SPI, D> {
    pub const fn new(spi: SPI, delay: D) -> Self {
        Self::with_address(spi, delay, SlaveAddress::Zero)
    }

    pub const fn with_address(spi: SPI, delay: D, address: SlaveAddress) -> Self {
        Self {
            spi,
            delay,
            address,
            pending: None,
            config: None,
            locked: false,
        }
    }

    pub fn release(self) -> (SPI, D) {
        (self.spi, self.delay)
    }

    pub const fn configuration(&self) -> Option<Config> {
        self.config
    }
}

impl<SPI: SpiDevice<u8>, D: DelayNs> Sch16t<SPI, D> {
    fn exchange(&mut self, request: Request) -> Result<Option<Response>, Error<SPI::Error>> {
        let mut wire = request.bytes(self.address);
        // >=one microsecond CS-high between calls, then setup/hold within CS.
        self.delay.delay_us(1);
        if let Err(error) = self.spi.transaction(&mut [
            Operation::DelayNs(1000),
            Operation::TransferInPlace(&mut wire),
            Operation::DelayNs(1000),
        ]) {
            // After a transport failure the device may or may not have accepted
            // the command. Discard the next response to resynchronize safely.
            self.pending = None;
            return Err(Error::Spi(error));
        }
        let previous = self.pending.replace(request);
        previous
            .map(|previous| previous.decode(wire, self.address).map_err(Error::Frame))
            .transpose()
    }

    /// Read any defined public register, preserving health and counter metadata.
    /// Faults in preceding command responses still propagate. Status flags are
    /// not rejected here so diagnostic registers remain readable during faults.
    pub fn read_register(&mut self, register: Register) -> Result<Response, Error<SPI::Error>> {
        self.exchange(Request::read(register))?;
        self.exchange(Request::read(Register::FrequencyCounter))?
            .ok_or(Error::Pipeline)
    }

    /// Pipeline an ordered register list, draining the complete burst even if
    /// an intermediate reply fails. This lets all decimated axes rearm DRDY.
    pub fn read_registers<const N: usize>(
        &mut self,
        registers: [Register; N],
    ) -> Result<[Response; N], Error<SPI::Error>> {
        let (frames, failure) = self.read_registers_capture(registers);
        match failure {
            Some(error) => Err(error),
            None => Ok(frames.map(|frame| frame.unwrap())),
        }
    }

    fn read_registers_capture<const N: usize>(
        &mut self,
        registers: [Register; N],
    ) -> ([Option<Response>; N], Option<Error<SPI::Error>>) {
        let mut result = [None; N];
        if N == 0 {
            return (result, None);
        }
        let mut failure = self.exchange(Request::read(registers[0])).err();
        for i in 0..N {
            let next = if i + 1 < N {
                registers[i + 1]
            } else {
                Register::FrequencyCounter
            };
            match self.exchange(Request::read(next)) {
                Ok(Some(response)) => result[i] = Some(response),
                Ok(None) => {
                    failure.get_or_insert(Error::Pipeline);
                }
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        (result, failure)
    }

    /// Low-level register access for capabilities beyond typed configuration.
    /// Validates access and reserved bits. Any configuration write invalidates
    /// cached conversions until a successful full `initialize`. Reset uses the
    /// required quiet period and can be requested even after EOI is set.
    pub fn write_register(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<SPI::Error>> {
        let Some(mask) = register.writable_mask() else {
            return Err(Error::ReadOnly(register));
        };
        if value & !mask != 0 {
            return Err(Error::InvalidRegisterValue(register));
        }
        if register == Register::Reset {
            if value != 0x0a {
                return Err(Error::InvalidRegisterValue(register));
            }
            return self.software_reset();
        }
        if self.locked && register != Register::SystemTest {
            return Err(Error::ConfigurationLocked);
        }
        // SelfTest reserved bit 1 must retain its reset value.
        if register == Register::SelfTest && value & 2 == 0 {
            return Err(Error::InvalidRegisterValue(register));
        }
        if register != Register::SystemTest {
            self.config = None;
        }
        self.write_unchecked(register, value)?;
        if register == Register::Mode && value & 2 != 0 {
            self.locked = true;
        }
        Ok(())
    }

    fn write_unchecked(&mut self, register: Register, value: u16) -> Result<(), Error<SPI::Error>> {
        self.exchange(Request::write(register, value))?;
        self.exchange(Request::read(Register::FrequencyCounter))?
            .ok_or(Error::Pipeline)?;
        Ok(())
    }

    fn verify(&mut self, register: Register, expected: u16) -> Result<(), Error<SPI::Error>> {
        let actual = self.read_register(register)?.information();
        if actual == expected {
            Ok(())
        } else {
            Err(Error::Readback {
                register,
                expected,
                actual,
            })
        }
    }

    /// Soft reset (§7.4.6): no SPI for 2 ms, then wait for 32 ms NVM startup.
    /// SCK must remain low during NVM startup, including other users of a bus.
    pub fn software_reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.config = None;
        self.locked = false;
        // Existing responses may represent a fault that prompted the reset.
        // Ignore that prior pipeline reply, while preserving new transport errors.
        self.pending = None;
        let result = self.exchange(Request::write(Register::Reset, 0x0a));
        self.pending = None;
        self.delay.delay_ms(2);
        self.delay.delay_ms(32);
        result.map(|_| ())
    }

    pub fn identity(&mut self) -> Result<Identity, Error<SPI::Error>> {
        let response = self.read_registers([
            Register::ComponentId,
            Register::AsicId,
            Register::Serial1,
            Register::Serial2,
            Register::Serial3,
        ])?;
        Ok(Identity {
            component: response[0].information(),
            asic: response[1].information(),
            serial: [
                response[2].information(),
                response[3].information(),
                response[4].information(),
            ],
        })
    }

    /// Read and acknowledge the ten startup/diagnostic status registers.
    pub fn status(&mut self) -> Result<Status, Error<SPI::Error>> {
        let frames = self.read_registers(STATUS_REGISTERS)?;
        Ok(Status {
            registers: frames.map(Response::information),
            frame_faults: frames
                .iter()
                .enumerate()
                .fold(0, |mask, (i, r)| mask | ((!r.healthy() as u16) << i)),
        })
    }

    pub fn counters(&mut self) -> Result<Counters, Error<SPI::Error>> {
        let frames = self.read_registers([
            Register::RateCounter,
            Register::AccCounter,
            Register::FrequencyCounter,
        ])?;
        let split = |value: u16| {
            [
                value as u8 & 15,
                (value >> 4) as u8 & 15,
                (value >> 8) as u8 & 15,
            ]
        };
        Ok(Counters {
            rate: split(frames[0].information()),
            acceleration: split(frames[1].information()),
            frequency: frames[2].information() & 0x3fff,
        })
    }

    /// One complete startup attempt based on Figure 8. The caller should retry
    /// with a bounded policy if needed; failed initialization never marks ready.
    /// Sensor self-test/filter settling precedes EOI, followed by two health
    /// checks and a readback of every configuration register written.
    pub fn initialize(&mut self, config: Config) -> Result<Identity, Error<SPI::Error>> {
        config.validate().map_err(Error::Configuration)?;
        self.software_reset()?;
        let identity = self.identity()?;
        if identity.component != 0x23 || identity.asic & 0x0f00 != 0 {
            return Err(Error::Identity {
                component: identity.component,
                asic: identity.asic,
            });
        }
        for pattern in [0x5a3c, 0] {
            self.write_unchecked(Register::SystemTest, pattern)?;
            self.verify(Register::SystemTest, pattern)?;
        }
        for (register, value) in config.registers() {
            self.write_unchecked(register, value)?;
        }
        self.write_unchecked(Register::Mode, 1)?;
        self.delay.delay_ms(215);
        // Startup intentionally latches faults. Read once without judging flags.
        self.status()?;
        self.write_unchecked(Register::Mode, 3)?;
        self.locked = true;
        let clearance_ms = if config.interface.status_clear_delay == StatusClearDelay::Ms5 {
            5
        } else {
            3
        };
        self.delay.delay_ms(clearance_ms);
        // Figure 8 calls for two post-EOI status reads. The first acknowledges
        // latched flags from the transition out of initialization; it must not
        // abort the sequence before that acknowledgement can clear. Respect
        // FTREE_TDEL before judging the second complete status. A persistent
        // hardware fault reasserts and remains an initialization failure.
        self.status()?;
        self.delay.delay_ms(clearance_ms);
        let status = self.status()?;
        // Readback distinguishes an ignored enable/configuration write from
        // its secondary normal-mode or integrity status flags. Both the
        // configuration and final status must pass before any motion is used.
        for (register, value) in config.registers() {
            self.verify(register, value)?;
        }
        self.verify(Register::Mode, 3)?;
        if !status.healthy() {
            return Err(Error::StartupStatus(status));
        }
        self.config = Some(config);
        Ok(identity)
    }

    fn check_sensor(response: Response) -> Result<Response, Error<SPI::Error>> {
        if response.healthy() {
            Ok(response)
        } else {
            Err(Error::SensorStatus {
                register: response.register,
                status: response.status,
                common_error: response.common_error,
                payload: response.payload,
                counter: response.counter,
            })
        }
    }

    /// Die temperature, not board ambient temperature. TEMP is a signed
    /// 16-bit MSB-aligned payload at 100 LSB/°C (§7.2.3).
    pub fn temperature_celsius(&mut self) -> Result<f32, Error<SPI::Error>> {
        let response = Self::check_sensor(self.read_register(Register::Temperature)?)?;
        response.temperature_celsius().ok_or(Error::Pipeline)
    }

    /// Read all six axes using a pipelined burst. Every frame's diagnostic
    /// status is checked. Converted values use the verified initialized ranges.
    pub fn motion(&mut self, output: MotionOutput) -> Result<MotionSample, Error<SPI::Error>> {
        self.motion_capture(output).result
    }

    /// Read native axis metadata before applying navigation health policy.
    /// No extra SPI transfers are added compared with `motion`.
    pub fn motion_capture(&mut self, output: MotionOutput) -> MotionCapture<SPI::Error> {
        self.motion_capture_unscaled(output).into_scaled()
    }

    /// Complete all six register reads and validate their frame/sensor status.
    /// Floating-point conversion is deferred so the host can timestamp the
    /// actual read completion independently of software arithmetic latency.
    pub fn motion_capture_unscaled(
        &mut self,
        output: MotionOutput,
    ) -> UnscaledMotionCapture<SPI::Error> {
        let Some(config) = self.config else {
            return UnscaledMotionCapture {
                frames: [None; 6],
                result: Err(Error::NotInitialized),
            };
        };
        let (registers, rate_scale, acc_scale) = match output {
            MotionOutput::Interpolated => (
                [
                    Register::RateX1,
                    Register::RateY1,
                    Register::RateZ1,
                    Register::AccX1,
                    Register::AccY1,
                    Register::AccZ1,
                ],
                config.interpolated_rate_range.degrees_per_second_per_lsb(),
                config
                    .interpolated_acc_range
                    .metres_per_second_squared_per_lsb(),
            ),
            MotionOutput::Decimated => (
                [
                    Register::RateX2,
                    Register::RateY2,
                    Register::RateZ2,
                    Register::AccX2,
                    Register::AccY2,
                    Register::AccZ2,
                ],
                config.decimated_rate_range.degrees_per_second_per_lsb(),
                config
                    .decimated_acc_range
                    .metres_per_second_squared_per_lsb(),
            ),
        };
        let (frames, failure) = self.read_registers_capture(registers);
        let result = (|| {
            if let Some(error) = failure {
                return Err(error);
            }
            let valid = frames.map(|frame| frame.unwrap());
            for frame in valid {
                Self::check_sensor(frame)?;
            }
            let raw_rate = core::array::from_fn(|i| valid[i].signed_sensor());
            let raw_acceleration = core::array::from_fn(|i| valid[i + 3].signed_sensor());
            Ok(UnscaledMotionSample {
                raw_rate,
                raw_acceleration,
                counters: valid.map(|r| r.counter),
                rate_scale,
                acc_scale,
            })
        })();
        UnscaledMotionCapture { frames, result }
    }

    /// Auxiliary interpolated ACC3, with its independently verified range.
    pub fn auxiliary_acceleration_mps2(&mut self) -> Result<[f64; 3], Error<SPI::Error>> {
        let config = self.config.ok_or(Error::NotInitialized)?;
        let frames = self.read_registers([Register::AccX3, Register::AccY3, Register::AccZ3])?;
        for frame in frames {
            Self::check_sensor(frame)?;
        }
        Ok(frames.map(|r| {
            f64::from(r.signed_sensor()) * config.aux_acc_range.metres_per_second_squared_per_lsb()
        }))
    }
}

#[cfg(test)]
mod tests;

impl<SPI: SpiDevice<u8>, D: DelayNs> crate::interfaces::TemperatureSensor for Sch16t<SPI, D> {
    type Error = Error<SPI::Error>;

    fn temperature_celsius(&mut self) -> Result<f32, Self::Error> {
        Sch16t::temperature_celsius(self)
    }
}
