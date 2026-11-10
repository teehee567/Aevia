//! BQ25622 / BQ25622E single-cell charger.
//!
//! Primary source: `data/datasheets/bq25622.pdf`, TI SLUSEG2D,
//! sections 8.5–8.6 (little-endian 16-bit registers, Table 8-7).
//! The fitted Mini is **BQ25622E**, a distinct part without OTG. Its identity,
//! register availability and reserved-bit masks additionally follow TI
//! [SLUSFA3C sections 5 and 8.6](https://www.ti.com/lit/ds/symlink/bq25622e.pdf).
//! Use 100 kHz I2C on a bus shared with this charger. Construction does no I/O;
//! all writes first check the selected variant's identity. No charging policy is
//! applied automatically. Register limits describe silicon programmability,
//! not a battery's allowable charging limits (or the 3 A operating rating).
//!
//! ADC results retain their previous values when conversion stops. Start a
//! one-shot and check `Status::adc_done()` before reading a fresh result.
//! Snapshot register reads are contiguous but are not a hardware atomic latch.

use embedded_hal::i2c::I2c;

pub const ADDRESS: u8 = 0x6b;
pub const MAX_I2C_HZ: u32 = 100_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    Bq25622,
    Bq25622E,
}
impl Variant {
    const fn part_number(self) -> u8 {
        match self {
            Self::Bq25622 => 1,
            Self::Bq25622E => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    Identity {
        expected: Variant,
        found: Identity,
    },
    InvalidValue,
    ReadOnly,
    Unsupported,
    /// The IBAT ADC reports 0x8000 when current polarity changes mid-conversion.
    ConversionAborted,
    Verification,
}

/// Named 8-bit registers. Flags at 0x20–0x22 are read-to-clear.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ByteRegister {
    ChargeControl0 = 0x14,
    ChargeTimer = 0x15,
    ChargerControl1 = 0x16,
    ChargerControl2 = 0x17,
    ChargerControl3 = 0x18,
    ChargerControl4 = 0x19,
    NtcControl0 = 0x1a,
    NtcControl1 = 0x1b,
    NtcControl2 = 0x1c,
    ChargerStatus0 = 0x1d,
    ChargerStatus1 = 0x1e,
    FaultStatus = 0x1f,
    ChargerFlag0 = 0x20,
    ChargerFlag1 = 0x21,
    FaultFlag = 0x22,
    ChargerMask0 = 0x23,
    ChargerMask1 = 0x24,
    FaultMask = 0x25,
    AdcControl = 0x26,
    AdcDisable = 0x27,
    PartInformation = 0x38,
}
impl ByteRegister {
    const fn write_mask(self, variant: Variant) -> u8 {
        let e = matches!(variant, Variant::Bq25622E);
        match self {
            Self::ChargeControl0 | Self::ChargerControl1 | Self::NtcControl1 | Self::AdcDisable => {
                0xff
            }
            Self::ChargeTimer => {
                if e {
                    0x8f
                } else {
                    0xff
                }
            }
            Self::ChargerControl2 => 0xfd,
            Self::ChargerControl3 => {
                if e {
                    0x1f
                } else {
                    0x7f
                }
            }
            Self::ChargerControl4 => {
                if e {
                    0xe7
                } else {
                    0xf7
                }
            }
            Self::NtcControl0 => {
                if e {
                    0x8f
                } else {
                    0xff
                }
            }
            Self::NtcControl2 | Self::ChargerMask0 => 0x7f,
            Self::ChargerMask1 => 0x09,
            Self::FaultMask => {
                if e {
                    0xe9
                } else {
                    0xf9
                }
            }
            Self::AdcControl => 0xfc,
            _ => 0,
        }
    }
    /// Bits never replayed by a read/modify/write helper. Raw writes are explicit.
    const fn actions(self) -> u8 {
        match self {
            Self::ChargeTimer => 0x20,     // FORCE_INDET, base part only
            Self::ChargerControl1 => 0x04, // WD_RST
            Self::ChargerControl2 => 0x80, // REG_RST
            Self::ChargerControl3 => 0x03, // BATFET mode commands
            _ => 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WordRegister {
    ChargeCurrent = 0x02,
    ChargeVoltage = 0x04,
    InputCurrent = 0x06,
    InputVoltage = 0x08,
    OtgCurrent = 0x0a,
    OtgVoltage = 0x0c,
    MinimumSystemVoltage = 0x0e,
    PrechargeCurrent = 0x10,
    TerminationCurrent = 0x12,
    BusCurrentAdc = 0x28,
    BatteryCurrentAdc = 0x2a,
    BusVoltageAdc = 0x2c,
    PmidVoltageAdc = 0x2e,
    BatteryVoltageAdc = 0x30,
    SystemVoltageAdc = 0x32,
    ThermistorAdc = 0x34,
    DieTemperatureAdc = 0x36,
}
impl WordRegister {
    /// (minimum, maximum, unit step, register shift) for writable registers.
    const fn field(self) -> Option<(u16, u16, u16, u8)> {
        match self {
            Self::ChargeCurrent => Some((80, 3520, 80, 6)),
            Self::ChargeVoltage => Some((3500, 4800, 10, 3)),
            Self::InputCurrent => Some((100, 3200, 20, 4)),
            Self::InputVoltage => Some((3800, 16800, 40, 5)),
            Self::OtgCurrent => Some((100, 2400, 20, 4)),
            Self::OtgVoltage => Some((3840, 9600, 80, 6)),
            Self::MinimumSystemVoltage => Some((2560, 3840, 80, 6)),
            Self::PrechargeCurrent => Some((20, 620, 20, 4)),
            Self::TerminationCurrent => Some((10, 620, 10, 3)),
            _ => None,
        }
    }
    const fn supported(self, variant: Variant) -> bool {
        !matches!(variant, Variant::Bq25622E)
            || !matches!(self, Self::OtgCurrent | Self::OtgVoltage)
    }
    fn encode(self, value: u16) -> Option<u16> {
        let (min, max, step, shift) = self.field()?;
        if value < min || value > max || !value.is_multiple_of(step) {
            return None;
        }
        Some((value / step) << shift)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Watchdog {
    Disabled = 0,
    Seconds50 = 1,
    Seconds100 = 2,
    Seconds200 = 3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ChargeState {
    Idle = 0,
    ConstantCurrent = 1,
    ConstantVoltage = 2,
    TopOff = 3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TemperatureZone {
    Normal = 0,
    Cold = 1,
    Hot = 2,
    Cool = 3,
    Warm = 4,
    PreCool = 5,
    PreWarm = 6,
    BiasFault = 7,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    Normal = 0,
    Shutdown = 1,
    Ship = 2,
    SystemReset = 3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcResolution {
    Bits12 = 0,
    Bits11 = 1,
    Bits10 = 2,
    Bits9 = 3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcConfig {
    pub enabled: bool,
    pub one_shot: bool,
    pub resolution: AdcResolution,
    pub running_average: bool,
    pub initialize_average: bool,
    /// Bit mask of channels to disable; see REG0x27 (IBUS=bit7, VPMID=bit0).
    pub disabled_channels: u8,
}
impl Default for AdcConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            one_shot: true,
            resolution: AdcResolution::Bits12,
            running_average: false,
            initialize_average: false,
            disabled_channels: 0,
        }
    }
}

/// All values must be exactly representable; errors never silently round upward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChargeLimits {
    pub charge_current_ma: u16,
    pub charge_voltage_mv: u16,
    pub input_current_ma: u16,
    pub input_voltage_mv: u16,
    pub minimum_system_voltage_mv: u16,
    pub precharge_current_ma: u16,
    pub termination_current_ma: u16,
}
impl ChargeLimits {
    fn encoded(self) -> Option<[(WordRegister, u16); 7]> {
        Some([
            (
                WordRegister::ChargeCurrent,
                WordRegister::ChargeCurrent.encode(self.charge_current_ma)?,
            ),
            (
                WordRegister::ChargeVoltage,
                WordRegister::ChargeVoltage.encode(self.charge_voltage_mv)?,
            ),
            (
                WordRegister::InputCurrent,
                WordRegister::InputCurrent.encode(self.input_current_ma)?,
            ),
            (
                WordRegister::InputVoltage,
                WordRegister::InputVoltage.encode(self.input_voltage_mv)?,
            ),
            (
                WordRegister::MinimumSystemVoltage,
                WordRegister::MinimumSystemVoltage.encode(self.minimum_system_voltage_mv)?,
            ),
            (
                WordRegister::PrechargeCurrent,
                WordRegister::PrechargeCurrent.encode(self.precharge_current_ma)?,
            ),
            (
                WordRegister::TerminationCurrent,
                WordRegister::TerminationCurrent.encode(self.termination_current_ma)?,
            ),
        ])
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub status0: u8,
    pub status1: u8,
    pub faults: u8,
}
impl Status {
    pub const fn adc_done(self) -> bool {
        self.status0 & 0x40 != 0
    }
    pub const fn charge_state(self) -> ChargeState {
        match (self.status1 >> 3) & 3 {
            0 => ChargeState::Idle,
            1 => ChargeState::ConstantCurrent,
            2 => ChargeState::ConstantVoltage,
            _ => ChargeState::TopOff,
        }
    }
    pub const fn temperature_zone(self) -> TemperatureZone {
        match self.faults & 7 {
            0 => TemperatureZone::Normal,
            1 => TemperatureZone::Cold,
            2 => TemperatureZone::Hot,
            3 => TemperatureZone::Cool,
            4 => TemperatureZone::Warm,
            5 => TemperatureZone::PreCool,
            6 => TemperatureZone::PreWarm,
            _ => TemperatureZone::BiasFault,
        }
    }
    pub const fn thermal_regulation(self) -> bool {
        self.status0 & 0x20 != 0
    }
    pub const fn input_current_regulation(self) -> bool {
        self.status0 & 0x08 != 0
    }
    pub const fn input_voltage_regulation(self) -> bool {
        self.status0 & 0x04 != 0
    }
    pub const fn watchdog_expired(self) -> bool {
        self.status0 & 1 != 0
    }
    pub const fn safety_timer_expired(self) -> bool {
        self.status0 & 2 != 0
    }
    pub const fn vbus_state_raw(self) -> u8 {
        self.status1 & 7
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InterruptFlags {
    pub charger0: u8,
    pub charger1: u8,
    pub faults: u8,
}

/// Decoded ADC values. TS is a voltage ratio, not a temperature; convert it
/// through the actual board thermistor network. Die temperature is unrelated to
/// battery temperature. `battery_current_ma=None` indicates an aborted result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcMeasurements {
    pub bus_current_ma: i16,
    pub battery_current_ma: Option<i16>,
    pub bus_voltage_uv: u32,
    pub pmid_voltage_uv: u32,
    pub battery_voltage_uv: u32,
    pub system_voltage_uv: u32,
    pub thermistor_ratio_ppm: u32,
    pub die_temperature_millicelsius: i32,
}
impl AdcMeasurements {
    pub fn from_registers(raw: [u16; 8]) -> Self {
        Self {
            bus_current_ma: (raw[0] as i16 >> 1) * 2,
            battery_current_ma: if raw[1] == 0x8000 {
                None
            } else {
                Some((raw[1] as i16 >> 2) * 4)
            },
            bus_voltage_uv: u32::from((raw[2] >> 2) & 0x1fff) * 3970,
            pmid_voltage_uv: u32::from((raw[3] >> 2) & 0x1fff) * 3970,
            battery_voltage_uv: u32::from((raw[4] >> 1) & 0x0fff) * 1990,
            system_voltage_uv: u32::from((raw[5] >> 1) & 0x0fff) * 1990,
            thermistor_ratio_ppm: u32::from(raw[6] & 0x0fff) * 961,
            die_temperature_millicelsius: i32::from(((raw[7] & 0x0fff) << 4) as i16 >> 4) * 500,
        }
    }
}

pub struct Bq25622<B> {
    bus: B,
    variant: Variant,
    identified: bool,
}
impl<B> Bq25622<B> {
    pub const fn new(bus: B, variant: Variant) -> Self {
        Self {
            bus,
            variant,
            identified: false,
        }
    }
    pub fn release(self) -> B {
        self.bus
    }
    pub const fn variant(&self) -> Variant {
        self.variant
    }
    /// Recheck identity after replacing/resetting a bus device.
    pub fn invalidate_identity(&mut self) {
        self.identified = false;
    }
}
impl<B: I2c> Bq25622<B> {
    pub fn identity(&mut self) -> Result<Identity, Error<B::Error>> {
        Ok(Identity::from_raw(
            self.read_byte(ByteRegister::PartInformation)?,
        ))
    }
    pub fn verify_identity(&mut self) -> Result<Identity, Error<B::Error>> {
        self.identified = false;
        let found = self.identity()?;
        if found.part_number != self.variant.part_number() || found.raw & 0xc0 != 0 {
            return Err(Error::Identity {
                expected: self.variant,
                found,
            });
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
        let mut value = [0];
        self.bus
            .write_read(ADDRESS, &[register as u8], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }
    /// Explicit write of a complete register. Reserved/RO bits and reserved
    /// converter settings are rejected. One-shot action bits are allowed here.
    pub fn write_byte(&mut self, register: ByteRegister, value: u8) -> Result<(), Error<B::Error>> {
        let mask = register.write_mask(self.variant);
        if mask == 0 {
            return Err(Error::ReadOnly);
        }
        if value & !mask != 0 {
            return Err(Error::InvalidValue);
        }
        if register == ByteRegister::ChargerControl2
            && ((value >> 4) & 3 == 3 || (value >> 2) & 3 == 2)
        {
            return Err(Error::InvalidValue);
        }
        if register == ByteRegister::ChargerControl4 && (value >> 6) < 2 {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        self.bus
            .write(ADDRESS, &[register as u8, value])
            .map_err(Error::Bus)
    }
    /// Update ordinary fields, suppressing watchdog/reset/mode triggers from
    /// the readback. Use dedicated actions or `write_byte` for trigger fields.
    pub fn update_byte(
        &mut self,
        register: ByteRegister,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        let allowed = register.write_mask(self.variant) & !register.actions();
        if mask == 0 || mask & !allowed != 0 || value & !mask != 0 {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        let old = self.read_byte(register)?;
        let desired =
            ((old & register.write_mask(self.variant)) & !register.actions() & !mask) | value;
        self.write_byte(register, desired)
    }
    pub fn read_word(&mut self, register: WordRegister) -> Result<u16, Error<B::Error>> {
        if !register.supported(self.variant) {
            return Err(Error::Unsupported);
        }
        let mut value = [0; 2];
        self.bus
            .write_read(ADDRESS, &[register as u8], &mut value)
            .map_err(Error::Bus)?;
        Ok(u16::from_le_bytes(value))
    }
    /// Write a raw 16-bit configuration value with reserved bits and the
    /// documented field range checked. ADC registers cannot be written.
    pub fn write_word(
        &mut self,
        register: WordRegister,
        value: u16,
    ) -> Result<(), Error<B::Error>> {
        if !register.supported(self.variant) {
            return Err(Error::Unsupported);
        }
        let Some((min, max, step, shift)) = register.field() else {
            return Err(Error::ReadOnly);
        };
        let units = (value >> shift)
            .checked_mul(step)
            .ok_or(Error::InvalidValue)?;
        if units < min || units > max || ((units / step) << shift) != value {
            return Err(Error::InvalidValue);
        }
        self.require_identity()?;
        let bytes = value.to_le_bytes();
        self.bus
            .write(ADDRESS, &[register as u8, bytes[0], bytes[1]])
            .map_err(Error::Bus)
    }
    /// Set a register in mA or mV (see its name). Off-grid values are rejected.
    pub fn set_limit(&mut self, register: WordRegister, value: u16) -> Result<(), Error<B::Error>> {
        let encoded = register.encode(value).ok_or(Error::InvalidValue)?;
        self.write_word(register, encoded)
    }
    /// Validate every limit before I/O, disable charging, then program limits.
    /// Once disabling succeeds this does not re-enable charging, including when
    /// a later limit write fails. Watchdog expiry or another bus owner can still
    /// change hardware state; enforce policy periodically. Validate board/battery
    /// limits before subsequently calling `set_charging(true)`.
    pub fn configure_limits(&mut self, limits: ChargeLimits) -> Result<(), Error<B::Error>> {
        let values = limits.encoded().ok_or(Error::InvalidValue)?;
        // Q4_FULLON raises the minimum valid charge/precharge/termination limits.
        self.require_identity()?;
        let full_on = self.read_byte(ByteRegister::ChargeControl0)? & 0x40 != 0;
        if full_on
            && (limits.charge_current_ma < 160
                || limits.precharge_current_ma < 80
                || limits.termination_current_ma < 120)
        {
            return Err(Error::InvalidValue);
        }
        self.set_charging(false)?;
        for (reg, value) in values {
            self.write_word(reg, value)?;
        }
        Ok(())
    }
    pub fn set_charging(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_byte(
            ByteRegister::ChargerControl1,
            0x20,
            if enabled { 0x20 } else { 0 },
        )
    }
    pub fn set_high_impedance(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.update_byte(
            ByteRegister::ChargerControl1,
            0x10,
            if enabled { 0x10 } else { 0 },
        )
    }
    pub fn set_watchdog(&mut self, watchdog: Watchdog) -> Result<(), Error<B::Error>> {
        self.update_byte(ByteRegister::ChargerControl1, 3, watchdog as u8)
    }
    pub fn kick_watchdog(&mut self) -> Result<(), Error<B::Error>> {
        self.require_identity()?;
        let old = self.read_byte(ByteRegister::ChargerControl1)?;
        self.write_byte(ByteRegister::ChargerControl1, (old & !0x04) | 0x04)
    }
    /// Disable EN_CHG and WATCHDOG while preserving power-path settings; clear
    /// WD_RST so a read/modify/write never accidentally issues a kick.
    pub fn disable_charging_and_watchdog(&mut self) -> Result<Status, Error<B::Error>> {
        self.update_byte(ByteRegister::ChargerControl1, 0x23, 0)?;
        let control = self.read_byte(ByteRegister::ChargerControl1)?;
        let status = self.status()?;
        if control & 0x23 != 0 || status.charge_state() != ChargeState::Idle {
            return Err(Error::Verification);
        }
        Ok(status)
    }
    pub fn reset_registers(&mut self) -> Result<(), Error<B::Error>> {
        self.require_identity()?;
        let old = self.read_byte(ByteRegister::ChargerControl2)?;
        self.write_byte(ByteRegister::ChargerControl2, (old & 0x7d) | 0x80)
    }
    /// Explicit BATFET action. Shutdown/ship/reset can remove system power.
    pub fn set_power_mode(
        &mut self,
        mode: PowerMode,
        allow_with_vbus: bool,
        long_delay: bool,
    ) -> Result<(), Error<B::Error>> {
        self.require_identity()?;
        let old = self.read_byte(ByteRegister::ChargerControl3)?;
        let value = (old & self.variant_control3_mask() & !0x0f)
            | if allow_with_vbus { 8 } else { 0 }
            | if long_delay { 4 } else { 0 }
            | mode as u8;
        self.write_byte(ByteRegister::ChargerControl3, value)
    }
    fn variant_control3_mask(&self) -> u8 {
        ByteRegister::ChargerControl3.write_mask(self.variant)
    }
    pub fn set_otg(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        if self.variant == Variant::Bq25622E {
            return Err(Error::Unsupported);
        }
        self.update_byte(
            ByteRegister::ChargerControl3,
            0x40,
            if enabled { 0x40 } else { 0 },
        )
    }
    pub fn configure_adc(&mut self, config: AdcConfig) -> Result<(), Error<B::Error>> {
        self.write_byte(ByteRegister::AdcDisable, config.disabled_channels)?;
        let value = if config.enabled { 0x80 } else { 0 }
            | if config.one_shot { 0x40 } else { 0 }
            | (config.resolution as u8) << 4
            | if config.running_average { 8 } else { 0 }
            | if config.initialize_average { 4 } else { 0 };
        self.write_byte(ByteRegister::AdcControl, value)
    }
    pub fn status(&mut self) -> Result<Status, Error<B::Error>> {
        let mut bytes = [0; 3];
        self.bus
            .write_read(ADDRESS, &[ByteRegister::ChargerStatus0 as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(Status {
            status0: bytes[0],
            status1: bytes[1],
            faults: bytes[2],
        })
    }
    /// Reads and clears the three interrupt flag registers. Read all three in
    /// one transaction to service an interrupt, then inspect current `status`.
    pub fn take_interrupt_flags(&mut self) -> Result<InterruptFlags, Error<B::Error>> {
        let mut bytes = [0; 3];
        self.bus
            .write_read(ADDRESS, &[ByteRegister::ChargerFlag0 as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(InterruptFlags {
            charger0: bytes[0],
            charger1: bytes[1],
            faults: bytes[2],
        })
    }
    pub fn measurements(&mut self) -> Result<AdcMeasurements, Error<B::Error>> {
        let mut bytes = [0; 16];
        self.bus
            .write_read(ADDRESS, &[WordRegister::BusCurrentAdc as u8], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(AdcMeasurements::from_registers(core::array::from_fn(|i| {
            u16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]])
        })))
    }
    pub fn battery_current_ma(&mut self) -> Result<i16, Error<B::Error>> {
        let raw = self.read_word(WordRegister::BatteryCurrentAdc)?;
        if raw == 0x8000 {
            return Err(Error::ConversionAborted);
        }
        Ok((raw as i16 >> 2) * 4)
    }
    pub fn die_temperature_millicelsius(&mut self) -> Result<i32, Error<B::Error>> {
        let raw = self.read_word(WordRegister::DieTemperatureAdc)?;
        Ok(i32::from(((raw & 0x0fff) << 4) as i16 >> 4) * 500)
    }
    /// TS/VBIAS in parts per million; valid only while TS bias is active.
    pub fn thermistor_ratio_ppm(&mut self) -> Result<u32, Error<B::Error>> {
        Ok(u32::from(self.read_word(WordRegister::ThermistorAdc)? & 0x0fff) * 961)
    }
}

#[cfg(test)]
mod tests;
