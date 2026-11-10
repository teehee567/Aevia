//! BQ25798 multi-cell charger and power-path HAL for the earlier full-size V2.
//!
//! Source: local `data/datasheets/bq25798.pdf`, TI SLUSDV2C (June 2026),
//! §§7.3.2, 7.3.12, 7.3.14 and complete register map §7.5.1. Multi-byte
//! registers transfer their high byte first (register figures show bits15:8
//! at the first address). This is **not** BQ25622's little-endian protocol.
//! Both parts use address 0x6B and must not be selected interchangeably.
//!
//! `new` does no I/O. Writes first verify PN=3 at REG48. Raw writes validate
//! register access, reserved bits and individual field ranges; callers must
//! satisfy cross-register and electrical constraints. `configure_charge` also
//! checks CELL-dependent VREG ranges and VSYSMIN<VREG before changing hardware.
//! No battery, input-source or charging policy is selected automatically.
//!
//! Watchdog expiration restores settings according to the datasheet. Enforce
//! application policy periodically. ADC readings can be stale/disabled; configure
//! conversion explicitly and check one-shot completion before consuming them.
//! Snapshot reads are contiguous, not a hardware atomic latch. Bus deadlines
//! and concurrent ownership are supplied by the caller.

mod registers;
use embedded_hal::i2c::I2c;
pub use registers::{ByteRegister, WordRegister};

pub const ADDRESS: u8 = 0x6b;
pub const MAX_I2C_HZ: u32 = 400_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity {
    pub part_number: u8,
    pub revision: u8,
    pub raw: u8,
}
impl Identity {
    pub const fn from_raw(raw: u8) -> Self {
        Self {
            part_number: (raw >> 3) & 7,
            revision: raw & 7,
            raw,
        }
    }
}
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    Identity(Identity),
    ReadOnly,
    InvalidValue,
    /// Explicit ship control requires a fitted FET, declared in REG14.SFET_PRESENT.
    MissingShipFet,
    Verification,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Cells {
    One = 0,
    Two = 1,
    Three = 2,
    Four = 3,
}
impl Cells {
    pub const fn voltage_range_mv(self) -> (u16, u16) {
        match self {
            Self::One => (3000, 4990),
            Self::Two => (5000, 9990),
            Self::Three => (10000, 13990),
            Self::Four => (14000, 18800),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Watchdog {
    Disabled = 0,
    Millis500 = 1,
    Seconds1 = 2,
    Seconds2 = 3,
    Seconds20 = 4,
    Seconds40 = 5,
    Seconds80 = 6,
    Seconds160 = 7,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PowerMode {
    Normal = 0,
    Shutdown = 1,
    Ship = 2,
    SystemReset = 3,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AdcResolution {
    Bits15 = 0,
    Bits14 = 1,
    Bits13 = 2,
    /// Silicon supports this but TI does not recommend it.
    Bits12 = 3,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdcConfig {
    pub enabled: bool,
    pub one_shot: bool,
    pub resolution: AdcResolution,
    /// Running average is not available for discharging IBAT (datasheet revision note).
    pub running_average: bool,
    pub initialize_average: bool,
    /// REG2F bits7:1; bit0 reserved. See channel names in §7.5.1.43.
    pub disabled_channels_0: u8,
    /// REG30 bits7:4 = D+, D-, VAC2, VAC1; low nibble reserved.
    pub disabled_channels_1: u8,
}
impl Default for AdcConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            one_shot: true,
            resolution: AdcResolution::Bits15,
            running_average: false,
            initialize_average: false,
            disabled_channels_0: 0,
            disabled_channels_1: 0,
        }
    }
}

/// Limits describe hardware programmability, not safe limits for a given pack.
/// Values must be exactly representable; no silently rounded charge limits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChargeConfig {
    pub cells: Cells,
    pub minimum_system_voltage_mv: u16,
    pub charge_voltage_mv: u16,
    pub charge_current_ma: u16,
    pub input_voltage_mv: u16,
    pub input_current_ma: u16,
    pub precharge_current_ma: u16,
    pub termination_current_ma: u16,
}
impl ChargeConfig {
    pub fn valid(self) -> bool {
        let (low, high) = self.cells.voltage_range_mv();
        (low..=high).contains(&self.charge_voltage_mv)
            && exact(self.charge_voltage_mv, 3000, 18800, 0, 10).is_some()
            && exact(self.minimum_system_voltage_mv, 2500, 16000, 2500, 250).is_some()
            && self.minimum_system_voltage_mv < self.charge_voltage_mv
            && exact(self.charge_current_ma, 50, 5000, 0, 10).is_some()
            && exact(self.input_voltage_mv, 3600, 22000, 0, 100).is_some()
            && exact(self.input_current_ma, 100, 3300, 0, 10).is_some()
            && exact(self.precharge_current_ma, 40, 2000, 0, 40).is_some()
            && exact(self.termination_current_ma, 40, 1000, 0, 40).is_some()
    }
}
fn exact(value: u16, min: u16, max: u16, offset: u16, step: u16) -> Option<u16> {
    if value < min || value > max || !(value - offset).is_multiple_of(step) {
        None
    } else {
        Some((value - offset) / step)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChargeState {
    Idle,
    Trickle,
    Precharge,
    ConstantCurrent,
    ConstantVoltage,
    Reserved,
    TopOff,
    Done,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Status {
    /// REG1B..1F, including raw VBUS source classification and NTC state.
    pub charger: [u8; 5],
    /// REG20..21 current faults, separate from latched read-to-clear flags.
    pub faults: [u8; 2],
}
impl Status {
    pub const fn charge_state(self) -> ChargeState {
        match self.charger[1] >> 5 {
            0 => ChargeState::Idle,
            1 => ChargeState::Trickle,
            2 => ChargeState::Precharge,
            3 => ChargeState::ConstantCurrent,
            4 => ChargeState::ConstantVoltage,
            5 => ChargeState::Reserved,
            6 => ChargeState::TopOff,
            _ => ChargeState::Done,
        }
    }
    pub const fn power_good(self) -> bool {
        self.charger[0] & 8 != 0
    }
    pub const fn watchdog_expired(self) -> bool {
        self.charger[0] & 0x20 != 0
    }
    pub const fn adc_done(self) -> bool {
        self.charger[3] & 0x20 != 0
    }
    pub const fn battery_present(self) -> bool {
        self.charger[2] & 1 != 0
    }
    pub const fn vbus_source_code(self) -> u8 {
        (self.charger[1] >> 1) & 15
    }
    pub const fn has_fault(self) -> bool {
        self.faults[0] != 0 || self.faults[1] & 0xf4 != 0
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InterruptFlags {
    /// REG22..25. Reading acknowledges these latched flags.
    pub charger: [u8; 4],
    pub faults: [u8; 2],
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InterruptMasks {
    pub charger: [u8; 4],
    pub faults: [u8; 2],
}
impl InterruptMasks {
    fn valid(self) -> bool {
        self.charger[1] & !0xd7 == 0
            && self.charger[2] & !0x7f == 0
            && self.charger[3] & !0x1f == 0
            && self.faults[1] & !0xf4 == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurements {
    pub bus_current_ma: i16,
    pub battery_current_ma: i16,
    pub bus_voltage_mv: u16,
    pub ac1_voltage_mv: u16,
    pub ac2_voltage_mv: u16,
    pub battery_voltage_mv: u16,
    pub system_voltage_mv: u16,
    /// Dimensionless TS/REGN ratio. Convert through the fitted resistor network
    /// and thermistor curve, not directly to degrees Celsius.
    pub thermistor_ratio: f64,
    pub die_temperature_millicelsius: i32,
    pub dp_voltage_mv: u16,
    pub dm_voltage_mv: u16,
}
impl Measurements {
    pub fn from_raw(words: [u16; 11]) -> Self {
        Self {
            bus_current_ma: words[0] as i16,
            battery_current_ma: words[1] as i16,
            bus_voltage_mv: words[2],
            ac1_voltage_mv: words[3],
            ac2_voltage_mv: words[4],
            battery_voltage_mv: words[5],
            system_voltage_mv: words[6],
            thermistor_ratio: f64::from(words[7]) / 1024.0,
            die_temperature_millicelsius: i32::from(words[8] as i16) * 500,
            dp_voltage_mv: words[9],
            dm_voltage_mv: words[10],
        }
    }
}

pub struct Bq25798<B> {
    bus: B,
    identified: bool,
}
impl<B> Bq25798<B> {
    pub const fn new(bus: B) -> Self {
        Self {
            bus,
            identified: false,
        }
    }
    pub fn release(self) -> B {
        self.bus
    }
    pub fn invalidate_identity(&mut self) {
        self.identified = false;
    }
}
impl<B: I2c> Bq25798<B> {
    pub fn identity(&mut self) -> Result<Identity, Error<B::Error>> {
        Ok(Identity::from_raw(
            self.read_byte(ByteRegister::PartInformation)?,
        ))
    }
    pub fn verify_identity(&mut self) -> Result<Identity, Error<B::Error>> {
        self.identified = false;
        let found = self.identity()?;
        if found.part_number != 3 || found.raw & 0xc0 != 0 {
            return Err(Error::Identity(found));
        }
        self.identified = true;
        Ok(found)
    }
    fn require_identity(&mut self) -> Result<(), Error<B::Error>> {
        if !self.identified {
            self.verify_identity()?;
        }
        Ok(())
    }
    pub fn read_byte(&mut self, register: ByteRegister) -> Result<u8, Error<B::Error>> {
        let mut bytes = [0];
        self.bus
            .write_read(ADDRESS, &[register.address()], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(bytes[0])
    }
    /// Explicit complete register write. Actions are allowed only here or in
    /// dedicated action methods. Cross-register constraints remain caller-owned.
    pub fn write_byte(&mut self, register: ByteRegister, value: u8) -> Result<(), Error<B::Error>> {
        if register.write_mask() == 0 {
            return Err(Error::ReadOnly);
        }
        if !register.value_valid(value) {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        self.bus
            .write(ADDRESS, &[register.address(), value])
            .map_err(Error::Bus)
    }
    /// RMW excludes all documented one-shot/power-mode actions from readback.
    pub fn update_byte(
        &mut self,
        register: ByteRegister,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        let allowed = register.write_mask() & !register.actions();
        if mask == 0 || mask & !allowed != 0 || value & !mask != 0 {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        let old = self.read_byte(register)?;
        self.write_byte(register, (old & allowed & !mask) | value)
    }
    pub fn read_word(&mut self, register: WordRegister) -> Result<u16, Error<B::Error>> {
        let mut bytes = [0; 2];
        self.bus
            .write_read(ADDRESS, &[register.address()], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(u16::from_be_bytes(bytes))
    }
    /// Word value is the raw register code, not mA/mV. Values are checked
    /// against the public field range; ADC and ICO result registers are read-only.
    pub fn write_word(
        &mut self,
        register: WordRegister,
        value: u16,
    ) -> Result<(), Error<B::Error>> {
        let Some((min, max)) = register.writable_range() else {
            return Err(Error::ReadOnly);
        };
        if !(min..=max).contains(&value) {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        let bytes = value.to_be_bytes();
        self.bus
            .write(ADDRESS, &[register.address(), bytes[0], bytes[1]])
            .map_err(Error::Bus)
    }
    pub fn cells(&mut self) -> Result<Cells, Error<B::Error>> {
        Ok(match self.read_byte(ByteRegister::RechargeControl)? >> 6 {
            0 => Cells::One,
            1 => Cells::Two,
            2 => Cells::Three,
            _ => Cells::Four,
        })
    }
    /// Validate the entire profile first. Disable charging, update CELL (which
    /// may reset VREG/ICHG/VSYSMIN), then program a consistent voltage pair and
    /// all limits. Charging stays disabled after success or a later I/O failure.
    /// Watchdog expiry or another bus owner can still restore/change settings.
    pub fn configure_charge(&mut self, config: ChargeConfig) -> Result<(), Error<B::Error>> {
        if !config.valid() {
            return Err(Error::InvalidValue);
        }
        self.set_charging(false)?;
        self.update_byte(
            ByteRegister::RechargeControl,
            0xc0,
            (config.cells as u8) << 6,
        )?;
        let old_vreg = self
            .read_word(WordRegister::ChargeVoltageLimit)?
            .saturating_mul(10);
        if config.minimum_system_voltage_mv < old_vreg {
            self.set_minimum_system_voltage_mv(config.minimum_system_voltage_mv)?;
            self.set_charge_voltage_mv(config.charge_voltage_mv)?;
        } else {
            self.set_charge_voltage_mv(config.charge_voltage_mv)?;
            self.set_minimum_system_voltage_mv(config.minimum_system_voltage_mv)?;
        }
        self.set_charge_current_ma(config.charge_current_ma)?;
        self.set_input_voltage_mv(config.input_voltage_mv)?;
        self.set_input_current_ma(config.input_current_ma)?;
        self.set_precharge_current_ma(config.precharge_current_ma)?;
        self.set_termination_current_ma(config.termination_current_ma)
    }
    pub fn set_minimum_system_voltage_mv(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 2500, 16000, 2500, 250).ok_or(Error::InvalidValue)?;
        self.require_identity()?;
        if u32::from(value) >= u32::from(self.read_word(WordRegister::ChargeVoltageLimit)?) * 10 {
            return Err(Error::InvalidValue);
        }
        self.write_byte(ByteRegister::MinimalSystemVoltage, code as u8)
    }
    pub fn set_charge_voltage_mv(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 3000, 18800, 0, 10).ok_or(Error::InvalidValue)?;
        self.require_identity()?;
        let (min, max) = self.cells()?.voltage_range_mv();
        if !(min..=max).contains(&value) {
            return Err(Error::InvalidValue);
        }
        let system_mv =
            2500 + u32::from(self.read_byte(ByteRegister::MinimalSystemVoltage)? & 0x3f) * 250;
        if system_mv >= u32::from(value) {
            return Err(Error::InvalidValue);
        }
        self.write_word(WordRegister::ChargeVoltageLimit, code)
    }
    pub fn set_charge_current_ma(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 50, 5000, 0, 10).ok_or(Error::InvalidValue)?;
        self.write_word(WordRegister::ChargeCurrentLimit, code)
    }
    pub fn set_input_voltage_mv(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 3600, 22000, 0, 100).ok_or(Error::InvalidValue)?;
        self.write_byte(ByteRegister::InputVoltageLimit, code as u8)
    }
    pub fn set_input_current_ma(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 100, 3300, 0, 10).ok_or(Error::InvalidValue)?;
        self.write_word(WordRegister::InputCurrentLimit, code)
    }
    pub fn set_precharge_current_ma(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 40, 2000, 0, 40).ok_or(Error::InvalidValue)?;
        self.update_byte(ByteRegister::PrechargeControl, 0x3f, code as u8)
    }
    pub fn set_termination_current_ma(&mut self, value: u16) -> Result<(), Error<B::Error>> {
        let code = exact(value, 40, 1000, 0, 40).ok_or(Error::InvalidValue)?;
        self.update_byte(ByteRegister::TerminationControl, 0x1f, code as u8)
    }
    /// Disable EN_OTG and program both limits; does not re-enable OTG.
    /// Disable autonomous backup separately before changing an active backup
    /// profile, because backup operation can re-engage OTG without this call.
    pub fn set_otg_limits(
        &mut self,
        voltage_mv: u16,
        current_ma: u16,
    ) -> Result<(), Error<B::Error>> {
        let voltage = exact(voltage_mv, 2800, 22000, 2800, 10).ok_or(Error::InvalidValue)?;
        let current = exact(current_ma, 160, 3360, 0, 40).ok_or(Error::InvalidValue)?;
        self.set_otg(false)?;
        self.write_word(WordRegister::OtgVoltage, voltage)?;
        self.update_byte(ByteRegister::OtgCurrent, 0x7f, current as u8)
    }
    pub fn set_charging(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_byte(
            ByteRegister::ChargerControl0,
            0x20,
            if enabled { 0x20 } else { 0 },
        )
    }
    pub fn set_high_impedance(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_byte(
            ByteRegister::ChargerControl0,
            4,
            if enabled { 4 } else { 0 },
        )
    }
    pub fn set_otg(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_byte(
            ByteRegister::ChargerControl3,
            0x40,
            if enabled { 0x40 } else { 0 },
        )
    }
    pub fn set_watchdog(&mut self, watchdog: Watchdog) -> Result<(), Error<B::Error>> {
        self.update_byte(ByteRegister::ChargerControl1, 7, watchdog as u8)
    }
    pub fn kick_watchdog(&mut self) -> Result<(), Error<B::Error>> {
        self.trigger(ByteRegister::ChargerControl1, 8)
    }
    /// REG_RST restores defaults, including charging and watchdog settings.
    pub fn reset_registers(&mut self) -> Result<(), Error<B::Error>> {
        self.trigger(ByteRegister::TerminationControl, 0x40)?;
        self.invalidate_identity();
        Ok(())
    }
    pub fn force_input_detection(&mut self) -> Result<(), Error<B::Error>> {
        self.trigger(ByteRegister::ChargerControl2, 0x80)
    }
    fn trigger(&mut self, register: ByteRegister, action: u8) -> Result<(), Error<B::Error>> {
        self.require_identity()?;
        let old = self.read_byte(register)?;
        self.write_byte(
            register,
            (old & register.write_mask() & !register.actions()) | action,
        )
    }
    /// Ship/shutdown/reset may remove power; only meaningful with the fitted
    /// external ship FET. Shutdown also disables I2C until a wake event.
    pub fn set_power_mode(
        &mut self,
        mode: PowerMode,
        immediate: bool,
    ) -> Result<(), Error<B::Error>> {
        self.require_identity()?;
        if mode != PowerMode::Normal && self.read_byte(ByteRegister::ChargerControl5)? & 0x80 == 0 {
            return Err(Error::MissingShipFet);
        }
        let old = self.read_byte(ByteRegister::ChargerControl2)?;
        self.write_byte(
            ByteRegister::ChargerControl2,
            (old & 0x78) | ((mode as u8) << 1) | u8::from(immediate),
        )
    }
    /// Validate reserved masks before any I/O. Channel data retains its old
    /// result while disabled. Enabling starts conversion, not a completed sample.
    pub fn configure_adc(&mut self, config: AdcConfig) -> Result<(), Error<B::Error>> {
        if config.disabled_channels_0 & 1 != 0 || config.disabled_channels_1 & 15 != 0 {
            return Err(Error::InvalidValue);
        }
        self.write_byte(
            ByteRegister::AdcFunctionDisable0,
            config.disabled_channels_0,
        )?;
        self.write_byte(
            ByteRegister::AdcFunctionDisable1,
            config.disabled_channels_1,
        )?;
        let value = u8::from(config.enabled) << 7
            | u8::from(config.one_shot) << 6
            | (config.resolution as u8) << 4
            | u8::from(config.running_average) << 3
            | u8::from(config.initialize_average) << 2;
        self.write_byte(ByteRegister::AdcControl, value)
    }
    pub fn status(&mut self) -> Result<Status, Error<B::Error>> {
        let mut bytes = [0; 7];
        self.bus
            .write_read(ADDRESS, &[0x1b], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(Status {
            charger: [bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]],
            faults: [bytes[5], bytes[6]],
        })
    }
    /// Read and acknowledge all six interrupt flag registers in one transaction.
    pub fn take_interrupt_flags(&mut self) -> Result<InterruptFlags, Error<B::Error>> {
        let mut bytes = [0; 6];
        self.bus
            .write_read(ADDRESS, &[0x22], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(InterruptFlags {
            charger: [bytes[0], bytes[1], bytes[2], bytes[3]],
            faults: [bytes[4], bytes[5]],
        })
    }
    pub fn configure_interrupt_masks(
        &mut self,
        masks: InterruptMasks,
    ) -> Result<(), Error<B::Error>> {
        if !masks.valid() {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        self.bus
            .write(
                ADDRESS,
                &[
                    0x28,
                    masks.charger[0],
                    masks.charger[1],
                    masks.charger[2],
                    masks.charger[3],
                    masks.faults[0],
                    masks.faults[1],
                ],
            )
            .map_err(Error::Bus)
    }
    pub fn measurements(&mut self) -> Result<Measurements, Error<B::Error>> {
        let mut bytes = [0; 22];
        self.bus
            .write_read(ADDRESS, &[0x31], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(Measurements::from_raw(core::array::from_fn(|i| {
            u16::from_be_bytes([bytes[i * 2], bytes[i * 2 + 1]])
        })))
    }
    pub fn die_temperature_millicelsius(&mut self) -> Result<i32, Error<B::Error>> {
        Ok(i32::from(self.read_word(WordRegister::DieTemperatureAdc)? as i16) * 500)
    }
    pub fn thermistor_ratio(&mut self) -> Result<f64, Error<B::Error>> {
        Ok(f64::from(self.read_word(WordRegister::ThermistorAdc)?) / 1024.0)
    }
}

#[cfg(test)]
mod tests;
