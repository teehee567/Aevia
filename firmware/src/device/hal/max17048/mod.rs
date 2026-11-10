//! MAX17048 (1S) / MAX17049 (2S) ModelGauge fuel gauge.
//!
//! Source: `data/datasheets/MAX17048-MAX17049.pdf`, 19-6171 Rev. 7,
//! pages 8–14 (compensation, sleep, registers and model access). The fixed
//! 7-bit I2C address is 0x36; every register transfer is a big-endian 16-bit
//! word. VCELL is 78.125 µV/cell per count; CRATE is signed 0.208 %/hour,
//! **not a current measurement**. The gauge has no temperature sensor: supply
//! externally measured battery temperature and characterized RCOMP constants.
//!
//! Constructors perform no I/O. Neither quick-start, POR, nor a battery model
//! is applied automatically. Register threshold methods expose the datasheet's
//! stated voltage units, rather than guessing MAX17049 pack threshold scaling.
//! Read/modify/write methods preserve observed alert bits; hardware may assert
//! an alert between the read and write, so service/read back alerts accordingly.

use embedded_hal::i2c::{Error as _, ErrorKind, I2c, NoAcknowledgeSource};

pub const ADDRESS: u8 = 0x36;
pub const MAX_I2C_HZ: u32 = 400_000;
const MODE_QUICK_START: u16 = 0x4000;
const MODE_ENABLE_SLEEP: u16 = 0x2000;
const CONFIG_SLEEP: u16 = 0x0080;
const CONFIG_SOC_CHANGE: u16 = 0x0040;
const CONFIG_ALERT: u16 = 0x0020;
const STATUS_RESET_ALERT: u16 = 0x4000;
const STATUS_FLAGS: u16 = 0x3f00;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Variant {
    Max17048,
    Max17049,
}
impl Variant {
    pub const fn cell_count(self) -> u32 {
        match self {
            Self::Max17048 => 1,
            Self::Max17049 => 2,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidValue,
    ReadOnly,
    /// Both failures are retained if model loading and the mandatory relock fail.
    ModelLoad {
        operation: Option<E>,
        relock: Option<E>,
    },
}

/// Public word registers. TABLE is deliberately accessed through `load_model`
/// so the gauge does not remain accidentally unlocked after a transfer error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    CellVoltage = 0x02,
    StateOfCharge = 0x04,
    Mode = 0x06,
    Version = 0x08,
    Hibernate = 0x0a,
    Configuration = 0x0c,
    VoltageAlert = 0x14,
    ChargeRate = 0x16,
    ResetVoltageAndId = 0x18,
    Status = 0x1a,
    Command = 0xfe,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResetOutcome {
    Acknowledged,
    /// The datasheet specifies no ACK after the final command byte. This only
    /// confirms the backend reported a data NACK; re-probe VERSION after POR.
    ExpectedDataNack,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AlertFlags(u16);
impl AlertFlags {
    pub const SOC_CHANGE: Self = Self(0x2000);
    pub const SOC_LOW: Self = Self(0x1000);
    pub const VOLTAGE_RESET: Self = Self(0x0800);
    pub const VOLTAGE_LOW: Self = Self(0x0400);
    pub const VOLTAGE_HIGH: Self = Self(0x0200);
    pub const POWER_ON_RESET: Self = Self(0x0100);
    pub const ALL: Self = Self(STATUS_FLAGS);
    pub const NONE: Self = Self(0);
    pub const fn from_bits_truncate(bits: u16) -> Self {
        Self(bits & STATUS_FLAGS)
    }
    pub const fn bits(self) -> u16 {
        self.0
    }
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Status {
    pub raw: u16,
}
impl Status {
    pub const fn flags(self) -> AlertFlags {
        AlertFlags::from_bits_truncate(self.raw)
    }
    pub const fn reset_alert_enabled(self) -> bool {
        self.raw & STATUS_RESET_ALERT != 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Configuration {
    pub rcomp: u8,
    pub sleeping: bool,
    pub soc_change_alert: bool,
    pub alert_pending: bool,
    /// Low-SOC threshold in whole percent, 1..=32.
    pub low_soc_percent: u8,
}
impl Configuration {
    pub const fn from_raw(raw: u16) -> Self {
        Self {
            rcomp: (raw >> 8) as u8,
            sleeping: raw & CONFIG_SLEEP != 0,
            soc_change_alert: raw & CONFIG_SOC_CHANGE != 0,
            alert_pending: raw & CONFIG_ALERT != 0,
            low_soc_percent: 32 - (raw & 0x1f) as u8,
        }
    }
}

/// Threshold codes exactly as stored in HIBRT. Enter threshold: 0.208 %/hour
/// per count, exit threshold: 1.25 mV per count (datasheet page 12). All zeros
/// disables hibernate; all ones forces it. These are separate explicit methods.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HibernateThresholds {
    pub rate_code: u8,
    pub voltage_code: u8,
}
impl HibernateThresholds {
    pub const fn raw(self) -> u16 {
        u16::from_be_bytes([self.rate_code, self.voltage_code])
    }
    pub const fn rate_millipercent_per_hour(self) -> u32 {
        self.rate_code as u32 * 208
    }
    pub const fn voltage_microvolts(self) -> u32 {
        self.voltage_code as u32 * 1250
    }
}

/// Characterized battery-model coefficients from the caller, not assumed
/// chemistry constants. Slopes are RCOMP counts per degree Celsius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcompCalibration {
    pub at_20_celsius: f32,
    pub counts_per_celsius_above_20: f32,
    pub counts_per_celsius_below_20: f32,
}
impl RcompCalibration {
    /// Datasheet page 8 piecewise compensation, rounded to nearest count and
    /// saturated to the 8-bit field. Invalid/nonfinite inputs return `None`.
    pub fn at_temperature(self, celsius: f32) -> Option<u8> {
        if !celsius.is_finite()
            || !self.at_20_celsius.is_finite()
            || !(0.0..=255.0).contains(&self.at_20_celsius)
            || !self.counts_per_celsius_above_20.is_finite()
            || !self.counts_per_celsius_below_20.is_finite()
        {
            return None;
        }
        let coefficient = if celsius > 20.0 {
            self.counts_per_celsius_above_20
        } else {
            self.counts_per_celsius_below_20
        };
        let value = self.at_20_celsius + (celsius - 20.0) * coefficient;
        if !value.is_finite() {
            return None;
        }
        Some(libm::roundf(value.clamp(0.0, 255.0)) as u8)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Measurements {
    pub voltage_raw: u16,
    /// Units of 1/256 percent; values are not silently clamped to 100 percent.
    pub state_of_charge_raw: u16,
    pub charge_rate_raw: i16,
    pub variant: Variant,
}
impl Measurements {
    /// Pack voltage, truncated to whole microvolts (error < 1 µV).
    pub const fn voltage_microvolts(self) -> u32 {
        self.voltage_raw as u32 * 625 * self.variant.cell_count() / 8
    }
    pub fn state_of_charge_percent(self) -> f32 {
        f32::from(self.state_of_charge_raw) / 256.0
    }
    pub const fn charge_rate_millipercent_per_hour(self) -> i32 {
        self.charge_rate_raw as i32 * 208
    }
}

pub struct Max17048<B> {
    bus: B,
    variant: Variant,
}
impl<B> Max17048<B> {
    /// Variant is selected from the fitted part. VERSION cannot distinguish a
    /// MAX17048 from MAX17049, and is not a unique device-identity register.
    pub const fn new(bus: B, variant: Variant) -> Self {
        Self { bus, variant }
    }
    pub const fn variant(&self) -> Variant {
        self.variant
    }
    pub fn release(self) -> B {
        self.bus
    }
}
impl<B: I2c> Max17048<B> {
    pub fn read_register(&mut self, register: Register) -> Result<u16, Error<B::Error>> {
        let mut bytes = [0; 2];
        self.bus
            .write_read(ADDRESS, &[register as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(u16::from_be_bytes(bytes))
    }
    /// Explicit raw word write. Reserved bits, RO registers and invalid POR
    /// commands are rejected. CONFIG/STATUS writes can clear latched alerts.
    pub fn write_register(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<B::Error>> {
        match register {
            Register::CellVoltage
            | Register::StateOfCharge
            | Register::Version
            | Register::ChargeRate => {
                return Err(Error::ReadOnly);
            }
            Register::Mode if value & !0x6000 != 0 => return Err(Error::InvalidValue),
            Register::Status if value & !0x7f00 != 0 => return Err(Error::InvalidValue),
            Register::Command if value != 0x5400 => return Err(Error::InvalidValue),
            _ => {}
        }
        self.write_word(register as u8, value).map_err(Error::Bus)
    }
    fn write_word(&mut self, address: u8, value: u16) -> Result<(), B::Error> {
        let bytes = value.to_be_bytes();
        self.bus.write(ADDRESS, &[address, bytes[0], bytes[1]])
    }
    pub fn version(&mut self) -> Result<u16, Error<B::Error>> {
        self.read_register(Register::Version)
    }
    pub fn configuration(&mut self) -> Result<Configuration, Error<B::Error>> {
        Ok(Configuration::from_raw(
            self.read_register(Register::Configuration)?,
        ))
    }
    pub fn status(&mut self) -> Result<Status, Error<B::Error>> {
        Ok(Status {
            raw: self.read_register(Register::Status)?,
        })
    }
    pub fn voltage_microvolts(&mut self) -> Result<u32, Error<B::Error>> {
        let raw = self.read_register(Register::CellVoltage)?;
        Ok(u32::from(raw) * 625 * self.variant.cell_count() / 8)
    }
    pub fn state_of_charge_percent(&mut self) -> Result<f32, Error<B::Error>> {
        Ok(f32::from(self.read_register(Register::StateOfCharge)?) / 256.0)
    }
    pub fn charge_rate_millipercent_per_hour(&mut self) -> Result<i32, Error<B::Error>> {
        Ok(i32::from(self.read_register(Register::ChargeRate)? as i16) * 208)
    }
    /// VCELL and SOC are read contiguously; CRATE is a later read. This is not a
    /// hardware-latched atomic snapshot. Updates run at the gauge's own cadence.
    pub fn measurements(&mut self) -> Result<Measurements, Error<B::Error>> {
        let mut bytes = [0; 4];
        self.bus
            .write_read(ADDRESS, &[Register::CellVoltage as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(Measurements {
            voltage_raw: u16::from_be_bytes([bytes[0], bytes[1]]),
            state_of_charge_raw: u16::from_be_bytes([bytes[2], bytes[3]]),
            charge_rate_raw: self.read_register(Register::ChargeRate)? as i16,
            variant: self.variant,
        })
    }
    fn update_config(&mut self, mask: u16, value: u16) -> Result<(), Error<B::Error>> {
        let old = self.read_register(Register::Configuration)?;
        self.write_register(Register::Configuration, (old & !mask) | (value & mask))
    }
    pub fn set_rcomp(&mut self, rcomp: u8) -> Result<(), Error<B::Error>> {
        self.update_config(0xff00, u16::from(rcomp) << 8)
    }
    pub fn compensate_temperature(
        &mut self,
        calibration: RcompCalibration,
        celsius: f32,
    ) -> Result<u8, Error<B::Error>> {
        let rcomp = calibration
            .at_temperature(celsius)
            .ok_or(Error::InvalidValue)?;
        self.set_rcomp(rcomp)?;
        Ok(rcomp)
    }
    pub fn set_low_soc_alert(&mut self, percent: u8) -> Result<(), Error<B::Error>> {
        if !(1..=32).contains(&percent) {
            return Err(Error::InvalidValue);
        }
        self.update_config(0x001f, u16::from(32 - percent))
    }
    pub fn set_soc_change_alert(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_config(
            CONFIG_SOC_CHANGE,
            if enabled { CONFIG_SOC_CHANGE } else { 0 },
        )
    }
    /// Service causes in STATUS separately, then clear the CONFIG.ALRT latch.
    pub fn clear_alert_latch(&mut self) -> Result<(), Error<B::Error>> {
        self.update_config(CONFIG_ALERT, 0)
    }
    pub fn set_reset_alert(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        let old = self.read_register(Register::Status)? & 0x7f00;
        self.write_register(
            Register::Status,
            (old & !STATUS_RESET_ALERT) | if enabled { STATUS_RESET_ALERT } else { 0 },
        )
    }
    /// Clear only the selected observed causes, preserving EnVR and other
    /// observed flags. Does not clear the separate CONFIG.ALRT output latch.
    pub fn clear_alert_flags(&mut self, flags: AlertFlags) -> Result<(), Error<B::Error>> {
        let old = self.read_register(Register::Status)? & 0x7f00;
        self.write_register(Register::Status, old & !flags.bits())
    }
    /// Uses the register's stated 20 mV units, 0..=5100 mV, exact steps only.
    /// These are register-domain thresholds; VCELL telemetry separately scales
    /// the MAX17049 pack voltage by two. Confirm 2S threshold interpretation for
    /// the selected board before treating these as pack safety limits.
    pub fn set_voltage_alerts_register_mv(
        &mut self,
        minimum: u16,
        maximum: u16,
    ) -> Result<(), Error<B::Error>> {
        if minimum > maximum
            || maximum > 5100
            || !minimum.is_multiple_of(20)
            || !maximum.is_multiple_of(20)
        {
            return Err(Error::InvalidValue);
        }
        self.write_register(
            Register::VoltageAlert,
            u16::from_be_bytes([(minimum / 20) as u8, (maximum / 20) as u8]),
        )
    }
    pub fn set_hibernate_thresholds(
        &mut self,
        thresholds: HibernateThresholds,
    ) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Hibernate, thresholds.raw())
    }
    pub fn disable_hibernate(&mut self) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Hibernate, 0)
    }
    pub fn force_hibernate(&mut self) -> Result<(), Error<B::Error>> {
        self.write_register(Register::Hibernate, 0xffff)
    }
    pub fn hibernating(&mut self) -> Result<bool, Error<B::Error>> {
        Ok(self.read_register(Register::Mode)? & 0x1000 != 0)
    }
    pub fn allow_sleep(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        // Do not replay the quick-start action or the RO hibernate bit.
        self.write_register(Register::Mode, if enabled { MODE_ENABLE_SLEEP } else { 0 })
    }
    /// Halts gauging. Prefer hibernate when charge/discharge can continue.
    /// Wake with `wake`; arbitrary I2C traffic does not wake software sleep.
    pub fn sleep(&mut self) -> Result<(), Error<B::Error>> {
        self.allow_sleep(true)?;
        self.update_config(CONFIG_SLEEP, CONFIG_SLEEP)
    }
    pub fn wake(&mut self) -> Result<(), Error<B::Error>> {
        self.update_config(CONFIG_SLEEP, 0)
    }
    /// Re-estimates SOC from the present voltage. Only issue deliberately when
    /// the battery is relaxed; it is not a routine initialization operation.
    pub fn quick_start(&mut self) -> Result<(), Error<B::Error>> {
        let permission = self.read_register(Register::Mode)? & MODE_ENABLE_SLEEP;
        self.write_register(Register::Mode, permission | MODE_QUICK_START)
    }
    /// VRESET has 40 mV register steps in the documented 2280..=3480 mV
    /// range. The read-only factory ID byte is preserved. For example, 2500 mV
    /// is not exactly representable; choose 2480 or 2520 explicitly.
    pub fn set_reset_voltage_register_mv(
        &mut self,
        millivolts: u16,
        disable_comparator_in_hibernate: bool,
    ) -> Result<(), Error<B::Error>> {
        if !(2280..=3480).contains(&millivolts) || !millivolts.is_multiple_of(40) {
            return Err(Error::InvalidValue);
        }
        let old = self.read_register(Register::ResetVoltageAndId)?;
        let high = ((millivolts / 40) as u8) << 1 | u8::from(disable_comparator_in_hibernate);
        self.write_register(
            Register::ResetVoltageAndId,
            u16::from(high) << 8 | (old & 0xff),
        )
    }
    pub fn factory_id(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.read_register(Register::ResetVoltageAndId)? as u8)
    }
    /// Sends POR without an implicit quick-start/model reload afterward. A
    /// data NACK is expected by the datasheet; address/unknown NACKs remain bus
    /// errors because they cannot establish that the command reached the IC.
    pub fn reset(&mut self) -> Result<ResetOutcome, Error<B::Error>> {
        match self.write_word(Register::Command as u8, 0x5400) {
            Ok(()) => Ok(ResetOutcome::Acknowledged),
            Err(error) if error.kind() == ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data) => {
                Ok(ResetOutcome::ExpectedDataNack)
            }
            Err(error) => Err(Error::Bus(error)),
        }
    }
    /// Write all 64 bytes of a caller-provided characterized model, using the
    /// page-14 0x4A57 unlock at 0x3E. Always attempts to relock with 0x0000,
    /// even if unlock or a table write fails. Gauging stops while unlocked.
    /// A failed relock leaves the device state uncertain; recover the bus and
    /// relock/reconfigure before using SOC. Model characterization and any
    /// model-specific follow-up steps must come from the battery model supplier.
    pub fn load_model(&mut self, table: &[u8; 64]) -> Result<(), Error<B::Error>> {
        let mut failure = self.write_word(0x3e, 0x4a57).err();
        if failure.is_none() {
            for (index, word) in table.as_chunks::<2>().0.iter().enumerate() {
                if let Err(error) = self.write_word(
                    0x40 + (index as u8) * 2,
                    u16::from_be_bytes([word[0], word[1]]),
                ) {
                    failure = Some(error);
                    break;
                }
            }
        }
        let relock = self.write_word(0x3e, 0).err();
        if failure.is_some() || relock.is_some() {
            return Err(Error::ModelLoad {
                operation: failure,
                relock,
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
