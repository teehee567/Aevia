//! GPIO-controlled regulators; there is no I2C or software voltage register.
//!
//! Sources: local `data/datasheets/tps63802.pdf`, TI SLVSEU9D §§7,8.3,8.5,
//! 9.3.2–3,9.3.6,9.3.10 and10.2.2.5; `tpsm863253.pdf`, TI SLUSF51C §§4–6 and7.3;
//! and `RPL-3_0.pdf`, RECOM Rev.3/2024 pp.1–3,6. TPSM863253 is the **fixed
//! 3.3V FCCM** member of its family; the adjustable252/257 devices in the same
//! PDF must not be confused with it. All three devices enable with a high
//! EN/CTRL level and expose PG. TI specifies open-drain PG; the RECOM sheet
//! characterizes its PG sink current but does not explicitly name its topology.
//! TPS63802 alone has MODE:
//! low selects automatic PWM/PFM power save, high selects forced PWM.
//!
//! Supply correctly configured, electrically compatible GPIOs. These inert
//! constructors never assert enable, change mode, or configure pullups. TI PG
//! needs an external pullup to a GPIO-compatible voltage; confirm RPL PG bias
//! with the board design. A correctly biased high PG is the chip's
//! instantaneous power-good signal, not a calibrated voltage measurement.
//! A successful output command records only what software requested, not the
//! voltage at the pin or rail. An I/O error invalidates that command history.
//!
//! **V2 Mini wiring:** TPS63802 EN is driven by MAX16169 OUT, MODE is tied to
//! ground, and PG is unconnected. Thus the MCU has no direct regulator EN,
//! MODE or PG GPIO to instantiate here. Use `max16169` for Mini shutdown;
//! these generic controls apply where the relevant pins are actually wired.
//! Other older-BOM regulators without a local datasheet are not assigned
//! guessed control pins, protocols or electrical limits.

use embedded_hal::digital::{InputPin, OutputPin};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Part {
    Tps63802,
    Tpsm863253,
    Rpl3_0,
}

/// Recommended nominal rail-voltage envelopes, not absolute-maximum ratings
/// or tolerance guarantees (fixed3.3V means the nominal output target). Buck
/// dropout, load, thermal limits and the datasheet's operating-area restrictions
/// still apply; these independent ranges do not establish a valid load point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OperatingLimits {
    pub input_min_mv: u16,
    pub input_max_mv: u16,
    pub output_min_mv: u16,
    pub output_max_mv: u16,
    /// TPS63802 needs VIN strictly above this until PG. Its lower input limit
    /// applies only after startup. None means no additional startup limit here.
    pub startup_input_above_mv: Option<u16>,
}

impl Part {
    pub const fn operating_limits(self) -> OperatingLimits {
        match self {
            Self::Tps63802 => OperatingLimits {
                input_min_mv: 1300,
                input_max_mv: 5500,
                output_min_mv: 1800,
                output_max_mv: 5200,
                startup_input_above_mv: Some(1800),
            },
            Self::Tpsm863253 => OperatingLimits {
                input_min_mv: 3000,
                input_max_mv: 17000,
                output_min_mv: 3300,
                output_max_mv: 3300,
                startup_input_above_mv: None,
            },
            Self::Rpl3_0 => OperatingLimits {
                input_min_mv: 4000,
                input_max_mv: 18000,
                output_min_mv: 800,
                output_max_mv: 5200,
                startup_input_above_mv: None,
            },
        }
    }

    /// Nominal external divider reference, not a software-programmable output.
    /// The fixed3.3V TPSM863253 instead connects VOS directly to VOUT.
    pub const fn feedback_reference_mv(self) -> Option<u16> {
        match self {
            Self::Tps63802 => Some(500),
            Self::Tpsm863253 => None,
            Self::Rpl3_0 => Some(800),
        }
    }

    pub const fn has_mode_pin(self) -> bool {
        matches!(self, Self::Tps63802)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedbackError {
    FixedOutput,
    InvalidResistance,
    ResistanceOutsideRecommendation,
    OutputOutsideRange,
}

/// Compute nominal VOUT = VREF * (1 + Rupper/Rlower), in millivolts, for an
/// existing two-resistor divider. Both resistors must be nonzero. This pure
/// calculation does not set voltage or account for tolerances, bias current,
/// compensation/capacitance, dropout or load. A direct-FB connection is outside
/// this two-resistor helper. The nominal result must fit the part's rail range.
/// TPS63802 additionally requires Rlower below100kΩ per §9.3.6; the stricter
/// wording is retained despite §10.2.2.5 saying it must not exceed100kΩ.
pub fn nominal_divider_output_mv(
    part: Part,
    upper_ohms: u32,
    lower_ohms: u32,
) -> Result<f64, FeedbackError> {
    let reference = part
        .feedback_reference_mv()
        .ok_or(FeedbackError::FixedOutput)?;
    if upper_ohms == 0 || lower_ohms == 0 {
        return Err(FeedbackError::InvalidResistance);
    }
    if part == Part::Tps63802 && lower_ohms >= 100_000 {
        return Err(FeedbackError::ResistanceOutsideRecommendation);
    }
    let output = f64::from(reference) * (1.0 + f64::from(upper_ohms) / f64::from(lower_ohms));
    let limits = part.operating_limits();
    if output < f64::from(limits.output_min_mv) || output > f64::from(limits.output_max_mv) {
        return Err(FeedbackError::OutputOutsideRange);
    }
    Ok(output)
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Pin(E),
}

/// The software command last accepted by the GPIO backend. No rail state is
/// inferred. None means no command was issued, or its result is uncertain.
pub struct EnableControl<P> {
    pin: P,
    part: Part,
    commanded: Option<bool>,
}
impl<P> EnableControl<P> {
    pub const fn new(part: Part, pin: P) -> Self {
        Self {
            pin,
            part,
            commanded: None,
        }
    }
    pub const fn part(&self) -> Part {
        self.part
    }
    pub const fn commanded_enabled(&self) -> Option<bool> {
        self.commanded
    }
    pub fn invalidate_command(&mut self) {
        self.commanded = None;
    }
    pub fn release(self) -> P {
        self.pin
    }
}
impl<P: OutputPin> EnableControl<P> {
    pub fn set_enabled(&mut self, enabled: bool) -> Result<(), Error<P::Error>> {
        self.commanded = None;
        if enabled {
            self.pin.set_high()
        } else {
            self.pin.set_low()
        }
        .map_err(Error::Pin)?;
        self.commanded = Some(enabled);
        Ok(())
    }
    pub fn enable(&mut self) -> Result<(), Error<P::Error>> {
        self.set_enabled(true)
    }
    pub fn disable(&mut self) -> Result<(), Error<P::Error>> {
        self.set_enabled(false)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchingMode {
    /// Automatic PWM/PFM, MODE low.
    PowerSave,
    /// Continuous forced PWM, MODE high.
    ForcedPwm,
}

/// TPS63802 MODE pin only. MODE must not float; the caller's GPIO setup must
/// supply a defined level before use. No such pin is available on V2 Mini.
pub struct Tps63802Mode<P> {
    pin: P,
    commanded: Option<SwitchingMode>,
}
impl<P> Tps63802Mode<P> {
    pub const fn new(pin: P) -> Self {
        Self {
            pin,
            commanded: None,
        }
    }
    pub const fn commanded_mode(&self) -> Option<SwitchingMode> {
        self.commanded
    }
    pub fn invalidate_command(&mut self) {
        self.commanded = None;
    }
    pub fn release(self) -> P {
        self.pin
    }
}
impl<P: OutputPin> Tps63802Mode<P> {
    pub fn set_mode(&mut self, mode: SwitchingMode) -> Result<(), Error<P::Error>> {
        self.commanded = None;
        match mode {
            SwitchingMode::PowerSave => self.pin.set_low(),
            SwitchingMode::ForcedPwm => self.pin.set_high(),
        }
        .map_err(Error::Pin)?;
        self.commanded = Some(mode);
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PowerGoodState {
    /// PG input is high; interpretation requires correct supply/input bias.
    Asserted,
    /// PG input is low; no software cause is inferred.
    NotAsserted,
}

/// Independent, live PG input. This never changes or assumes an enable command.
pub struct PowerGoodInput<P> {
    pin: P,
    part: Part,
}
impl<P> PowerGoodInput<P> {
    pub const fn new(part: Part, pin: P) -> Self {
        Self { pin, part }
    }
    pub const fn part(&self) -> Part {
        self.part
    }
    pub fn release(self) -> P {
        self.pin
    }
}
impl<P: InputPin> PowerGoodInput<P> {
    pub fn read(&mut self) -> Result<PowerGoodState, Error<P::Error>> {
        Ok(if self.pin.is_high().map_err(Error::Pin)? {
            PowerGoodState::Asserted
        } else {
            PowerGoodState::NotAsserted
        })
    }
}

#[cfg(test)]
mod tests;
