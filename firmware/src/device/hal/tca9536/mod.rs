//! TCA9536/A/B/C GPIO expander and V2 Mini's four active-low UI buttons.
//!
//! Source: `data/datasheets/tca9536.pdf`, TI SCPS275A (December 2021),
//! §§8.3–8.6, especially Tables 8-1–8-7. The fitted TCA9536A is at 0x40;
//! 0x41 belongs to the unsuffixed TCA9536. Configuration is explicit and
//! constructors do no I/O. Register sequences need exclusive logical ownership
//! of this device; a bus lock around individual transactions is not sufficient
//! if another task can also change its configuration.
//!
//! P0..P3 connect to SW3, SW6, SW4 and SW5 respectively. P3 is a button input
//! on V2 Mini, so `configure_buttons` keeps its optional interrupt function off.
//! Input reads acknowledge an active expander interrupt (§8.3.3); they are not
//! observationally neutral when P3 is configured as INT on another board.
use embedded_hal::i2c::I2c;

pub const PORT_MASK: u8 = 0x0f;
/// Fast-mode Plus requires the electrical conditions in datasheet §6.7;
/// V2 Mini's shared charger bus must instead use 100 kHz.
pub const MAX_I2C_HZ: u32 = 1_000_000;
const P3_INTERRUPT: u8 = 0x80;
const PULLUPS_DISABLED: u8 = 0x40;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Variant {
    Tca9536,
    Tca9536A,
    Tca9536B,
    Tca9536C,
}
impl Variant {
    pub const fn address(self) -> u8 {
        match self {
            Self::Tca9536 => 0x41,
            Self::Tca9536A => 0x40,
            Self::Tca9536B => 0x43,
            Self::Tca9536C => 0x42,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    Input = 0x00,
    Output = 0x01,
    Polarity = 0x02,
    Configuration = 0x03,
    SpecialFunction = 0x50,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Pin {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
}
impl Pin {
    pub const fn mask(self) -> u8 {
        1 << self as u8
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum P3Function {
    Gpio,
    Interrupt,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SpecialFunction {
    pub p3: P3Function,
    /// When disabled, this also removes P3's pullup in interrupt mode.
    pub pullups_enabled: bool,
}
impl SpecialFunction {
    pub const fn from_raw(raw: u8) -> Self {
        Self {
            p3: if raw & P3_INTERRUPT != 0 {
                P3Function::Interrupt
            } else {
                P3Function::Gpio
            },
            pullups_enabled: raw & PULLUPS_DISABLED == 0,
        }
    }
    pub const fn bits(self) -> u8 {
        (if matches!(self.p3, P3Function::Interrupt) {
            P3_INTERRUPT
        } else {
            0
        }) | if self.pullups_enabled {
            0
        } else {
            PULLUPS_DISABLED
        }
    }
}
impl Default for SpecialFunction {
    fn default() -> Self {
        Self {
            p3: P3Function::Gpio,
            pullups_enabled: true,
        }
    }
}

/// Four-bit masks use P0 as bit0. A set direction bit means input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Configuration {
    pub inputs: u8,
    pub output_high: u8,
    pub inverted_inputs: u8,
    pub special: SpecialFunction,
}
impl Default for Configuration {
    fn default() -> Self {
        Self {
            inputs: PORT_MASK,
            output_high: PORT_MASK,
            inverted_inputs: 0,
            special: SpecialFunction::default(),
        }
    }
}
impl Configuration {
    fn valid(self) -> bool {
        (self.inputs | self.output_high | self.inverted_inputs) & !PORT_MASK == 0
            && (!matches!(self.special.p3, P3Function::Interrupt)
                || self.inputs & Pin::P3.mask() != 0)
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidValue,
    ReadOnly,
    P3IsInterrupt,
    /// Four button inputs require normal polarity and enabled pullups.
    ButtonsNotConfigured,
}

pub struct Tca9536<B> {
    bus: B,
    variant: Variant,
}
impl<B> Tca9536<B> {
    pub const fn new(bus: B, variant: Variant) -> Self {
        Self { bus, variant }
    }
    pub const fn v2_mini(bus: B) -> Self {
        Self::new(bus, Variant::Tca9536A)
    }
    pub const fn address(&self) -> u8 {
        self.variant.address()
    }
    pub fn release(self) -> B {
        self.bus
    }
}
impl<B: I2c> Tca9536<B> {
    /// Returns the complete register, including the documented unused bits.
    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<B::Error>> {
        let mut bytes = [0];
        self.bus
            .write_read(self.address(), &[register as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(bytes[0])
    }

    /// Write meaningful bits only. Values with unused bits set are rejected;
    /// unused output/configuration bits are emitted as their reset value, one.
    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<B::Error>> {
        let wire_value = match register {
            Register::Input => return Err(Error::ReadOnly),
            Register::Output | Register::Configuration => {
                if value & !PORT_MASK != 0 {
                    return Err(Error::InvalidValue);
                }
                value | 0xf0
            }
            Register::Polarity => {
                if value & !PORT_MASK != 0 {
                    return Err(Error::InvalidValue);
                }
                value
            }
            Register::SpecialFunction => {
                if value & !(P3_INTERRUPT | PULLUPS_DISABLED) != 0 {
                    return Err(Error::InvalidValue);
                }
                value
            }
        };
        self.bus
            .write(self.address(), &[register as u8, wire_value])
            .map_err(Error::Bus)
    }

    /// Input levels after programmed polarity inversion. This read also
    /// acknowledges an active interrupt when P3 has its INT function enabled.
    pub fn input_levels(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.read_register(Register::Input)? & PORT_MASK)
    }
    pub fn output_latch(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.read_register(Register::Output)? & PORT_MASK)
    }
    pub fn input_directions(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.read_register(Register::Configuration)? & PORT_MASK)
    }
    pub fn set_input_directions(&mut self, input_mask: u8) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Configuration, input_mask)
    }
    pub fn set_polarity(&mut self, inverted_mask: u8) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Polarity, inverted_mask)
    }
    pub fn special_function(&mut self) -> Result<SpecialFunction, Error<B::Error>> {
        Ok(SpecialFunction::from_raw(
            self.read_register(Register::SpecialFunction)?,
        ))
    }
    pub fn set_special_function(
        &mut self,
        special: SpecialFunction,
    ) -> Result<(), Error<B::Error>> {
        self.write_register(Register::SpecialFunction, special.bits())
    }
    pub fn set_outputs(&mut self, high_mask: u8) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Output, high_mask)
    }
    /// Only masked output latches change; this does not change pin direction.
    /// Latches may intentionally be set while their pins are inputs.
    pub fn update_outputs(&mut self, mask: u8, high_bits: u8) -> Result<(), Error<B::Error>> {
        if mask & !PORT_MASK != 0 || high_bits & !mask != 0 {
            return Err(Error::InvalidValue);
        }
        let current = self.output_latch()?;
        self.set_outputs((current & !mask) | high_bits)
    }
    pub fn set_pin_level(&mut self, pin: Pin, high: bool) -> Result<(), Error<B::Error>> {
        self.update_outputs(pin.mask(), if high { pin.mask() } else { 0 })
    }
    /// Makes selected pins outputs after preloading their output latches.
    /// Existing output pins retain their directions and unselected latch bits.
    /// P3 cannot be selected while its interrupt override is active.
    pub fn configure_outputs(
        &mut self,
        output_mask: u8,
        initial_high: u8,
    ) -> Result<(), Error<B::Error>> {
        if output_mask & !PORT_MASK != 0 || initial_high & !output_mask != 0 {
            return Err(Error::InvalidValue);
        }
        if output_mask & Pin::P3.mask() != 0 && self.special_function()?.p3 == P3Function::Interrupt
        {
            return Err(Error::P3IsInterrupt);
        }
        let inputs = self.input_directions()?;
        self.update_outputs(output_mask, initial_high)?;
        self.set_input_directions(inputs & !output_mask)
    }
    /// Validate the complete configuration first, release GPIO outputs to
    /// inputs, preload latches, then enable the requested output directions.
    /// This deliberately introduces a high-impedance interval while changing
    /// an already configured device. A transport failure stops the sequence.
    pub fn configure(&mut self, configuration: Configuration) -> Result<(), Error<B::Error>> {
        if !configuration.valid() {
            return Err(Error::InvalidValue);
        }
        self.set_input_directions(PORT_MASK)?;
        self.set_outputs(configuration.output_high)?;
        self.set_polarity(configuration.inverted_inputs)?;
        self.set_special_function(configuration.special)?;
        self.set_input_directions(configuration.inputs)
    }
    /// Configure all four PCB button pins, including P3, as normal-polarity
    /// inputs with pullups. Does not perform an initial input read/acknowledge.
    pub fn configure_buttons(&mut self) -> Result<(), Error<B::Error>> {
        self.configure(Configuration::default())
    }
    /// Verify button mode before interpreting active-low levels. This catches
    /// accidental P3 interrupt repurposing or inversion/direction changes.
    pub fn buttons(&mut self) -> Result<Buttons, Error<B::Error>> {
        if self.input_directions()? != PORT_MASK
            || self.read_register(Register::Polarity)? & PORT_MASK != 0
            || self.read_register(Register::SpecialFunction)? & 0xc0 != 0
        {
            return Err(Error::ButtonsNotConfigured);
        }
        Ok(Buttons::from_active_low_port(self.input_levels()?))
    }
    /// Explicit I2C General Call reset (§8.5.2): sends address0 / byte0x06 and
    /// a STOP. This resets **every supporting device on the shared bus**,
    /// regardless of this instance's variant/address. Other owners must
    /// invalidate cached register state. Never use this as an automatic retry.
    pub fn reset_all_general_call_devices(&mut self) -> Result<(), Error<B::Error>> {
        self.bus.write(0x00, &[0x06]).map_err(Error::Bus)
    }
}

/// Bit i is one when the button wired to Pi is pressed.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Buttons(u8);
impl Buttons {
    pub const NONE: Self = Self(0);
    pub const fn from_pressed_mask(mask: u8) -> Option<Self> {
        if mask & !PORT_MASK != 0 {
            None
        } else {
            Some(Self(mask))
        }
    }
    pub const fn from_active_low_port(levels: u8) -> Self {
        Self(!levels & PORT_MASK)
    }
    pub const fn pressed_mask(self) -> u8 {
        self.0
    }
    pub const fn is_pressed(self, pin: Pin) -> bool {
        self.0 & pin.mask() != 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ButtonChanges {
    pub state: Buttons,
    pub pressed: Buttons,
    pub released: Buttons,
}

/// Independent stable-interval debounce for each button. Supply monotonic
/// millisecond timestamps, not wall-clock/GNSS time. A clock rollback restarts
/// every candidate interval while retaining the last debounced state; it cannot
/// manufacture a press/release. Zero interval requests immediate transitions.
/// Call often enough to observe the bounce; unobserved transitions are unknown.
pub struct ButtonDebouncer {
    interval_ms: u32,
    stable: Buttons,
    candidate: Buttons,
    candidate_since_ms: [u64; 4],
    last_ms: Option<u64>,
}
impl ButtonDebouncer {
    pub const fn new(interval_ms: u32, initial: Buttons) -> Self {
        Self {
            interval_ms,
            stable: initial,
            candidate: initial,
            candidate_since_ms: [0; 4],
            last_ms: None,
        }
    }
    pub const fn state(&self) -> Buttons {
        self.stable
    }
    pub fn update(&mut self, now_ms: u64, observed: Buttons) -> ButtonChanges {
        let rollback = self.last_ms.is_some_and(|previous| now_ms < previous);
        if rollback {
            self.candidate = observed;
            self.candidate_since_ms = [now_ms; 4];
        }
        self.last_ms = Some(now_ms);
        let previous = self.stable.0;
        for index in 0..4 {
            let mask = 1 << index;
            if (self.candidate.0 ^ observed.0) & mask != 0 {
                self.candidate.0 = (self.candidate.0 & !mask) | (observed.0 & mask);
                self.candidate_since_ms[index] = now_ms;
            }
            if (self.stable.0 ^ observed.0) & mask != 0
                && now_ms.saturating_sub(self.candidate_since_ms[index])
                    >= u64::from(self.interval_ms)
                && (!rollback || self.interval_ms == 0)
            {
                self.stable.0 = (self.stable.0 & !mask) | (observed.0 & mask);
            }
        }
        ButtonChanges {
            state: self.stable,
            pressed: Buttons(self.stable.0 & !previous),
            released: Buttons(previous & !self.stable.0),
        }
    }
}

#[cfg(test)]
mod tests;
