//! LP5813 boost / matrix RGB LED driver.
//!
//! Source: `data/datasheets/lp5813.pdf`, TI SNVSC74C (February 2025),
//! sections 4, 7.3–7.5, 8 and 9.2.3.4–9.2.3.5. Command values and detailed
//! field definitions additionally follow TI [SNVU859](https://www.ti.com/lit/ug/snvu859/snvu859.pdf),
//! sections 2.3–2.11. The local PDF's prose calls whole-pattern playback a
//! two-bit field; the register table and detailed guide define the four-bit
//! field used here. Register summary typos are resolved by the detailed guide.
//!
//! This driver never changes the EN GPIO: enable the physical power supply and
//! EN first, then call `initialize`. Configure the shared bus at 100 kHz for
//! the board's charger, although LP5813 alone supports up to 1 MHz. Initialization
//! preserves LED-enable/DC/PWM registers; retained settings can resume when the
//! digital core is enabled. Configuration writes require an update
//! command; PWM/DC/LED-enable writes take effect directly and are not atomic.
//! Register-page bits are part of the seven-bit I2C address, not a second
//! register-pointer byte. Raw block operations are split at page boundaries.

mod config;
pub use config::*;
use embedded_hal::{delay::DelayNs, i2c::I2c};

pub const MAX_I2C_HZ: u32 = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Variant {
    A,
    B,
    C,
    D,
}
impl Variant {
    pub const fn address_base(self) -> u8 {
        match self {
            Self::A => 0x50,
            Self::B => 0x54,
            Self::C => 0x58,
            Self::D => 0x5c,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidRegister,
    ReadOnly,
    InvalidValue,
    Configuration,
    Verification {
        register: u16,
        expected: u8,
        actual: u8,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Channel {
    Direct0,
    Direct1,
    Direct2,
    Direct3,
    A0,
    A1,
    A2,
    B0,
    B1,
    B2,
    C0,
    C1,
    C2,
    D0,
    D1,
    D2,
}
impl Channel {
    pub const fn index(self) -> u8 {
        self as u8
    }
    pub const fn bit(self) -> u16 {
        1 << self.index()
    }
    pub const fn animation_base(self) -> u16 {
        0x80 + 26 * self.index() as u16
    }
}

/// Physical RGB packages. Mapping comes from the V2 Mini PCB pads, not their
/// numeric reference order: D32=A, D33=C, D34=D, D35=B. At every scan,
/// green = subchannel 0, red = 1 and blue = 2 (ROHM MSL0402RGBU1 pinout).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MiniLed {
    D32,
    D33,
    D34,
    D35,
}
impl MiniLed {
    /// (red, green, blue) channel order.
    pub const fn channels(self) -> [Channel; 3] {
        match self {
            Self::D32 => [Channel::A1, Channel::A0, Channel::A2],
            Self::D33 => [Channel::C1, Channel::C0, Channel::C2],
            Self::D34 => [Channel::D1, Channel::D0, Channel::D2],
            Self::D35 => [Channel::B1, Channel::B0, Channel::B2],
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Faults {
    pub configuration_error: bool,
    pub thermal_shutdown: bool,
    /// Bit ordering is the same as `Channel::bit()`.
    pub open_channels: u16,
    pub short_channels: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
    Update,
    Start,
    Stop,
    Pause,
    Continue,
}
impl Command {
    const fn register_value(self) -> (u16, u8) {
        match self {
            Self::Update => (0x10, 0x55),
            Self::Start => (0x11, 0xff),
            Self::Stop => (0x12, 0xaa),
            Self::Pause => (0x13, 0x33),
            Self::Continue => (0x14, 0xcc),
        }
    }
}

pub struct Lp5813<I> {
    bus: I,
    variant: Variant,
}
impl<I: I2c> Lp5813<I> {
    pub const fn new(bus: I, variant: Variant) -> Self {
        Self { bus, variant }
    }
    pub fn release(self) -> I {
        self.bus
    }
    pub const fn variant(&self) -> Variant {
        self.variant
    }

    /// EN must already be high. Allows at least 1 ms for startup, enables the
    /// digital core, applies and reads back all configuration registers.
    /// Does not reset or set LED enable, DC or PWM values.
    pub fn initialize<D: DelayNs>(
        &mut self,
        config: &Config,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>> {
        config.encode().map_err(|_| Error::InvalidValue)?;
        delay.delay_ms(1);
        self.enable_chip(true)?;
        self.configure(config)?;
        self.verify_configuration(config)
    }

    pub fn enable_chip(&mut self, enabled: bool) -> Result<(), Error<I::Error>> {
        self.write_register(0, u8::from(enabled))
    }

    /// Validate the entire configuration before sending the first byte. All
    /// CONFIG registers are written, including manual/autonomous selections.
    pub fn configure(&mut self, config: &Config) -> Result<(), Error<I::Error>> {
        let bytes = config.encode().map_err(|_| Error::InvalidValue)?;
        self.write_registers(1, &bytes)?;
        self.command(Command::Update)?;
        if self.read_register(0x300)? & 1 != 0 {
            return Err(Error::Configuration);
        }
        Ok(())
    }

    pub fn verify_configuration(&mut self, config: &Config) -> Result<(), Error<I::Error>> {
        let expected = config.encode().map_err(|_| Error::InvalidValue)?;
        let mut actual = [0; 13];
        self.read_registers(1, &mut actual)?;
        for (index, (&expected, &actual)) in expected.iter().zip(&actual).enumerate() {
            if expected != actual {
                return Err(Error::Verification {
                    register: index as u16 + 1,
                    expected,
                    actual,
                });
            }
        }
        Ok(())
    }

    pub fn command(&mut self, command: Command) -> Result<(), Error<I::Error>> {
        let (register, value) = command.register_value();
        self.write_register(register, value)
    }

    /// Resets all registers; reinitialize afterward. Never issued implicitly.
    pub fn software_reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        self.write_register(0x23, 0x66)?;
        delay.delay_ms(1);
        Ok(())
    }

    /// Set all logical enable bits; use 0xfff0 for all 12 matrix LEDs.
    pub fn set_enabled(&mut self, channels: u16) -> Result<(), Error<I::Error>> {
        self.write_registers(0x20, &channels.to_le_bytes())
    }
    pub fn set_manual_current(
        &mut self,
        channel: Channel,
        dot_current: u8,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(0x30 + u16::from(channel.index()), dot_current)
    }
    pub fn set_manual_pwm(&mut self, channel: Channel, pwm: u8) -> Result<(), Error<I::Error>> {
        self.write_register(0x40 + u16::from(channel.index()), pwm)
    }
    pub fn set_autonomous_current(
        &mut self,
        channel: Channel,
        dot_current: u8,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(0x50 + u16::from(channel.index()), dot_current)
    }
    pub fn set_manual_currents(&mut self, values: &[u8; 16]) -> Result<(), Error<I::Error>> {
        self.write_registers(0x30, values)
    }
    pub fn set_manual_pwms(&mut self, values: &[u8; 16]) -> Result<(), Error<I::Error>> {
        self.write_registers(0x40, values)
    }
    /// Write only this physical package's PWM values, in RGB order. This does
    /// not set its current or enable bits; partial color updates are possible
    /// on a bus failure. Use `set_manual_pwms` for a full contiguous frame.
    pub fn set_mini_rgb(&mut self, led: MiniLed, rgb: [u8; 3]) -> Result<(), Error<I::Error>> {
        for (channel, value) in led.channels().into_iter().zip(rgb) {
            self.set_manual_pwm(channel, value)?;
        }
        Ok(())
    }
    /// Configure one complete 26-byte pattern; this does not select autonomous
    /// mode or start it. Pause/stop before altering a running pattern.
    pub fn set_animation(
        &mut self,
        channel: Channel,
        pattern: &Animation,
    ) -> Result<(), Error<I::Error>> {
        let bytes = pattern.encode().map_err(|_| Error::InvalidValue)?;
        self.write_registers(channel.animation_base(), &bytes)
    }
    pub fn autonomous_pwm(&mut self, channel: Channel) -> Result<u8, Error<I::Error>> {
        self.read_register(0x305 + u16::from(channel.index()))
    }
    /// Three-bit AEP state code from the datasheet. Pause first for a stable
    /// PWM/state pair. Unknown codes remain observable instead of discarded.
    pub fn animation_state(&mut self, channel: Channel) -> Result<u8, Error<I::Error>> {
        let code = self.read_register(0x315 + u16::from(channel.index()) / 2)?;
        Ok((code >> ((channel.index() % 2) * 3)) & 7)
    }
    pub fn faults(&mut self) -> Result<Faults, Error<I::Error>> {
        let mut flags = [0; 5];
        self.read_registers(0x300, &mut flags)?;
        Ok(Faults {
            configuration_error: flags[0] & 1 != 0,
            thermal_shutdown: flags[0] & 2 != 0,
            open_channels: u16::from_le_bytes([flags[1], flags[2]]),
            short_channels: u16::from_le_bytes([flags[3], flags[4]]),
        })
    }
    pub fn clear_faults(
        &mut self,
        open: bool,
        short: bool,
        thermal: bool,
    ) -> Result<(), Error<I::Error>> {
        self.write_register(
            0x22,
            u8::from(open) | (u8::from(short) << 1) | (u8::from(thermal) << 2),
        )
    }
    pub fn read_register(&mut self, register: u16) -> Result<u8, Error<I::Error>> {
        let mut byte = [0];
        self.read_registers(register, &mut byte)?;
        Ok(byte[0])
    }
    /// Access documented locations only. Read commands return their defined
    /// reset value; flags are not read-to-clear. No I2C general-call is used.
    pub fn read_registers(
        &mut self,
        register: u16,
        bytes: &mut [u8],
    ) -> Result<(), Error<I::Error>> {
        validate_range(register, bytes.len(), false)?;
        let mut offset = 0;
        while offset < bytes.len() {
            let address = register + offset as u16;
            let count = (256 - usize::from(address & 0xff)).min(bytes.len() - offset);
            self.bus
                .write_read(
                    self.variant.address_base() | (address >> 8) as u8,
                    &[address as u8],
                    &mut bytes[offset..offset + count],
                )
                .map_err(Error::Bus)?;
            offset += count;
        }
        Ok(())
    }
    pub fn write_register(&mut self, register: u16, value: u8) -> Result<(), Error<I::Error>> {
        self.write_registers(register, &[value])
    }
    /// Write documented registers, rejecting reserved bits before any I/O.
    /// Configuration writes through this API need `command(Update)` afterward.
    /// Transfers are bounded to 32 data bytes and never cross a page.
    pub fn write_registers(&mut self, register: u16, bytes: &[u8]) -> Result<(), Error<I::Error>> {
        validate_range(register, bytes.len(), true)?;
        for (offset, &value) in bytes.iter().enumerate() {
            if value & !write_mask(register + offset as u16) != 0 {
                return Err(Error::InvalidValue);
            }
        }
        let mut packet = [0; 33];
        let mut offset = 0;
        while offset < bytes.len() {
            let address = register + offset as u16;
            let count = 32
                .min(256 - usize::from(address & 0xff))
                .min(bytes.len() - offset);
            packet[0] = address as u8;
            packet[1..=count].copy_from_slice(&bytes[offset..offset + count]);
            self.bus
                .write(
                    self.variant.address_base() | (address >> 8) as u8,
                    &packet[..count + 1],
                )
                .map_err(Error::Bus)?;
            offset += count;
        }
        Ok(())
    }
}

fn validate_range<E>(register: u16, length: usize, writing: bool) -> Result<(), Error<E>> {
    if length > 0x400 || register >= 0x400 || usize::from(register) + length > 0x400 {
        return Err(Error::InvalidRegister);
    }
    for address in register..register + length as u16 {
        if !matches!(address, 0x000..=0x00d | 0x010..=0x014 | 0x020..=0x023 | 0x030..=0x05f | 0x080..=0x21f | 0x300..=0x31c)
        {
            return Err(Error::InvalidRegister);
        }
        if writing && address >= 0x300 {
            return Err(Error::ReadOnly);
        }
    }
    Ok(())
}
fn write_mask(register: u16) -> u8 {
    match register {
        0x000 => 1,
        0x001 => 0x3f,
        0x00c | 0x022 => 7,
        0x080..=0x21f => match (register - 0x80) % 26 {
            1 => 0x3f,
            9 | 17 | 25 => 3,
            _ => 0xff,
        },
        _ => 0xff,
    }
}

#[cfg(test)]
mod tests;
