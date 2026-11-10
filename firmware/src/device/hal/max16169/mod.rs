//! MAX16169 pushbutton power controller: GPIO shutdown and interrupt sensing.
//!
//! Source: local `data/datasheets/max16169.pdf`, 19-101732 Rev. 0 (11/2023),
//! pp. 5–6 (timing tolerance/CLR lockout), pp. 8–11 (pins, Table 1 and operation)
//! and p. 12 (ordering). The Mini fits **MAX16169AALTA+T**, timing option A,
//! active-low PB_IN. CLR and INT are always active low. There is no I2C/SPI
//! interface and the MCU cannot command OUT high through CLR.
//!
//! On Mini, CLR is GPIO8 (PWR_KILL_N), INT is GPIO38 and the latched OUT drives
//! TPS63802 EN directly. Configure CLR as **open drain**, initially released
//! high, and INT as an input with the required pullup. Releasing CLR does not
//! turn power on; a debounced physical pushbutton closure latches OUT high.
//! A long hold with option A can remove MCU power before software observes it.
//!
//! Construction performs no I/O. Shutdown methods record only a successful GPIO
//! drive, never a measured power state. CLR is ignored while OUT is off and
//! during the startup lockout after OUT rises (nominally 2 × tINT). The caller
//! must account for that lockout; a successful GPIO operation does not prove
//! shutdown occurred. No CLR pulse helper is provided: the sheet specifies a
//! propagation delay, but no guaranteed minimum CLR pulse width.
//!
//! INT is an active-low open-drain **one-shot pulse**, not the current button
//! state. A sustained button hold does not hold INT low indefinitely. Pure pulse
//! classification below requires a complete electrically captured low interval;
//! polling/task timestamps can miss edges, and missing pullup power can keep INT
//! low without a valid event. No event automatically requests shutdown.

use embedded_hal::digital::{InputPin, OutputPin};

/// Table 1 factory timing selection; these values cannot be programmed by MCU.
/// Options B/C require factory availability; the ordering table lists only AALTA.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimingOption {
    A,
    B,
    C,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PushbuttonPolarity {
    ActiveLow,
    ActiveHigh,
}

/// Datasheet timing interval, in microseconds. Min/max use the documented ±20%
/// timing accuracy; they describe hardware timing, not a software debounce rule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimingInterval {
    pub minimum_us: u32,
    pub nominal_us: u32,
    pub maximum_us: u32,
}
impl TimingInterval {
    const fn from_nominal(nominal_us: u32) -> Self {
        Self {
            minimum_us: nominal_us * 80 / 100,
            nominal_us,
            maximum_us: nominal_us * 120 / 100,
        }
    }
    pub const fn contains(self, duration_us: u64) -> bool {
        duration_us >= self.minimum_us as u64 && duration_us <= self.maximum_us as u64
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Timing {
    pub debounce: TimingInterval,
    pub long_hold: TimingInterval,
    pub switch_interrupt: TimingInterval,
    pub long_hold_interrupt: TimingInterval,
    /// After OUT rises, CLR transitions are ignored for this interval. Limits
    /// match electrical tCLR lockout: 1.6/2.0/2.4 times nominal tINT.
    pub clear_lockout: TimingInterval,
    pub long_hold_deasserts_output: bool,
}
impl TimingOption {
    pub const fn timing(self) -> Timing {
        let (debounce_us, long_hold_us, long_hold_deasserts_output) = match self {
            Self::A => (50_000, 8_000_000, true),
            Self::B => (2_000_000, 16_000_000, false),
            Self::C => (50_000, 16_000_000, false),
        };
        Timing {
            debounce: TimingInterval::from_nominal(debounce_us),
            long_hold: TimingInterval::from_nominal(long_hold_us),
            switch_interrupt: TimingInterval::from_nominal(32_000),
            long_hold_interrupt: TimingInterval::from_nominal(128_000),
            clear_lockout: TimingInterval::from_nominal(64_000),
            long_hold_deasserts_output,
        }
    }
}

/// Fitted hardware metadata. Only `AALTA` corresponds to an explicit orderable
/// part in the local sheet; other combinations require verified factory parts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Variant {
    pub timing: TimingOption,
    pub pushbutton_polarity: PushbuttonPolarity,
}
impl Variant {
    pub const AALTA: Self = Self {
        timing: TimingOption::A,
        pushbutton_polarity: PushbuttonPolarity::ActiveLow,
    };
    pub const fn timing(self) -> Timing {
        self.timing.timing()
    }

    /// Classify an already captured *complete* INT low duration. Tolerance
    /// windows do not overlap. A result describes the controller's event, not
    /// whether the button remains pressed or the powered rail remains on.
    pub const fn classify_interrupt_us(self, duration_us: u64) -> InterruptPulse {
        let timing = self.timing();
        if timing.switch_interrupt.contains(duration_us) {
            InterruptPulse::DebouncedSwitch
        } else if timing.long_hold_interrupt.contains(duration_us) {
            InterruptPulse::LongHold
        } else {
            InterruptPulse::Unrecognized
        }
    }

    /// Convenience for hardware edge timestamps from one monotonic counter.
    /// A rollover/clock rollback must be resolved by the capture backend first.
    pub fn classify_captured_interval_us(
        self,
        falling_edge_us: u64,
        rising_edge_us: u64,
    ) -> Result<InterruptPulse, TimestampError> {
        let duration = rising_edge_us
            .checked_sub(falling_edge_us)
            .ok_or(TimestampError::NonMonotonic)?;
        Ok(self.classify_interrupt_us(duration))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InterruptPulse {
    DebouncedSwitch,
    LongHold,
    /// Outside both tolerance windows; no power-control action is implied.
    Unrecognized,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimestampError {
    NonMonotonic,
}

/// Last successfully commanded CLR state, rather than OUT/power feedback.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShutdownDrive {
    /// Construction or a failed GPIO operation leaves the electrical state unknown.
    Unknown,
    /// CLR is released (open-drain high); OUT may still be latched off.
    Released,
    /// CLR is driven low. OUT may ignore it during its startup lockout.
    HeldLow,
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<C, I> {
    ClearPin(C),
    InterruptPin(I),
}

pub struct Max16169<C, I> {
    clear_n: C,
    interrupt_n: I,
    variant: Variant,
    drive: ShutdownDrive,
}
impl<C, I> Max16169<C, I> {
    pub const fn new(clear_n: C, interrupt_n: I, variant: Variant) -> Self {
        Self {
            clear_n,
            interrupt_n,
            variant,
            drive: ShutdownDrive::Unknown,
        }
    }
    pub const fn v2_mini(clear_n: C, interrupt_n: I) -> Self {
        Self::new(clear_n, interrupt_n, Variant::AALTA)
    }
    pub const fn variant(&self) -> Variant {
        self.variant
    }
    pub const fn shutdown_drive(&self) -> ShutdownDrive {
        self.drive
    }
    /// Releases ownership without changing either electrical pin state.
    pub fn release(self) -> (C, I) {
        (self.clear_n, self.interrupt_n)
    }
}
impl<C: OutputPin, I: InputPin> Max16169<C, I> {
    /// Release the open-drain CLR pin. This never commands OUT to turn on.
    pub fn release_shutdown(&mut self) -> Result<(), Error<C::Error, I::Error>> {
        self.drive = ShutdownDrive::Unknown;
        self.clear_n.set_high().map_err(Error::ClearPin)?;
        self.drive = ShutdownDrive::Released;
        Ok(())
    }
    /// Hold CLR low deliberately. This can remove the MCU's own power before
    /// return; a return value confirms only the pin call, not the powered rail.
    /// The application must finish storage/communication shutdown beforehand.
    pub fn hold_shutdown(&mut self) -> Result<(), Error<C::Error, I::Error>> {
        self.drive = ShutdownDrive::Unknown;
        self.clear_n.set_low().map_err(Error::ClearPin)?;
        self.drive = ShutdownDrive::HeldLow;
        Ok(())
    }
    /// Observe the electrical INT level; true means active low at this instant.
    /// Do not interpret this one-shot signal as a currently held pushbutton.
    pub fn interrupt_asserted(&mut self) -> Result<bool, Error<C::Error, I::Error>> {
        self.interrupt_n.is_low().map_err(Error::InterruptPin)
    }
}

#[cfg(test)]
mod tests;
