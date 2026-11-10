//! MAX17320 2S–4S ModelGauge m5 fuel gauge and pack protector, I2C version.
//!
//! Source: `data/datasheets/max17320.pdf`, 19-100749 Rev. 12 (July 2025),
//! especially pp. 78–124 (units/registers), 125–139 (memory/protection), and
//! 158–165 (reset/I2C/SBS). Internal 9-bit addresses map to **7-bit** I2C 0x36
//! and 0x0B; the datasheet's 6Ch/16h include the write bit. Words are little endian.
//!
//! `new` performs no I/O and applies no battery model or pack protection limits.
//! `Configuration` describes the fitted pack for host conversions; it does not
//! program nPackCfg/nRSense. Confirm those device settings independently.
//! Ordinary writes identify the part, unlock, perform the write and relock,
//! retaining both the original error and a cleanup error. No method implicitly
//! burns NVM, locks memory permanently, changes a secret, or resets the device.
//!
//! `maintenance` provides raw access to all documented commands/features while
//! keeping the unlock/relock sequence in one scope. Its caller must supply the
//! complete silicon-specific programming algorithm, delays and reserved bits.
//! NVM copying is a separate explicit operation with only seven lifetime block
//! writes (one normally consumed in manufacturing); it never retries implicitly.
//! Authentication block organization is not published here: the datasheet directs
//! users to the manufacturer, so no invented SHA message format is provided.
//!
//! Keep exclusive access to this *device* across a whole multi-transaction call:
//! the two CommStat writes must be consecutive without another register access.
//! The bus backend must impose finite I/O deadlines. Measurements are sequential
//! register observations, not a hardware-latched snapshot. ADC/gauge values can
//! be stale after shutdown/reset; wait for conversion/startup and inspect status.

mod registers;
pub use registers::Register;

use embedded_hal::{delay::DelayNs, i2c::I2c};

pub const PRIMARY_ADDRESS: u8 = 0x36;
pub const SECONDARY_ADDRESS: u8 = 0x0b;
pub const MAX_I2C_HZ: u32 = 400_000;
const FET_OVERRIDE: u16 = 0x0300;
const NV_ERROR: u16 = 4;
const NV_BUSY: u16 = 2;
const ALL_WRITE_PROTECT: u16 = 0x00f9;
const STATUS_CLEARABLE: u16 = 0xf782;

#[derive(Debug, PartialEq, Eq)]
pub enum Failure<E> {
    Bus(E),
    InvalidValue,
    Identity(u16),
    ReadOnly,
    Busy,
    SbsBlockRequired,
    Pec,
    Nonvolatile,
    Timeout,
    Verification {
        register: Register,
        expected: u16,
        actual: u16,
    },
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Device(Failure<E>),
    /// Cleanup could not establish the complete protected device state.
    Relock {
        operation: Option<Failure<E>>,
        error: E,
    },
}
impl<E> From<Failure<E>> for Error<E> {
    fn from(value: Failure<E>) -> Self {
        Self::Device(value)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Configuration {
    cells: u8,
    sense_microohms: u32,
}
impl Configuration {
    /// Explicit fitted series-cell count and sense resistance. No chemistry,
    /// protection voltage/current or calibration value is chosen for the caller.
    pub const fn new(cells: u8, sense_microohms: u32) -> Option<Self> {
        if cells >= 2 && cells <= 4 && sense_microohms != 0 {
            Some(Self {
                cells,
                sense_microohms,
            })
        } else {
            None
        }
    }
    pub const fn cells(self) -> u8 {
        self.cells
    }
    pub const fn sense_microohms(self) -> u32 {
        self.sense_microohms
    }
    /// Table 15: signed current, positive means charging, whole µA truncated
    /// toward zero. Calculation uses i64 to preserve the full raw input range.
    pub const fn current_microamps(self, raw: u16) -> i64 {
        (raw as i16 as i64) * 1_562_500 / self.sense_microohms as i64
    }
    pub const fn capacity_microamp_hours(self, raw: u16) -> u64 {
        raw as u64 * 5_000_000 / self.sense_microohms as u64
    }
    /// Instantaneous/average power, signed µW, Table 15 and p.101.
    pub const fn power_microwatts(self, raw: u16) -> i64 {
        (raw as i16 as i64) * 8_000_000 / self.sense_microohms as i64
    }
}

pub const fn cell_voltage_microvolts(raw: u16) -> u32 {
    raw as u32 * 625 / 8
}
pub const fn pack_voltage_microvolts(raw: u16) -> u32 {
    raw as u32 * 625 / 2
}
pub fn temperature_celsius(raw: u16) -> f32 {
    f32::from(raw as i16) / 256.0
}
pub fn percentage(raw: u16) -> f32 {
    f32::from(raw) / 256.0
}
pub const fn time_milliseconds(raw: u16) -> u32 {
    raw as u32 * 5_625
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommunicationStatus(pub u16);
impl CommunicationStatus {
    pub const fn nonvolatile_busy(self) -> bool {
        self.0 & NV_BUSY != 0
    }
    pub const fn nonvolatile_error(self) -> bool {
        self.0 & NV_ERROR != 0
    }
    pub const fn charge_forced_off(self) -> bool {
        self.0 & 0x100 != 0
    }
    pub const fn discharge_forced_off(self) -> bool {
        self.0 & 0x200 != 0
    }
    pub const fn globally_write_protected(self) -> bool {
        self.0 & 1 != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProtectionFlags(pub u16);
impl ProtectionFlags {
    pub const CHARGE_WATCHDOG: u16 = 0x8000;
    pub const TOO_HOT_CHARGING: u16 = 0x4000;
    pub const FULL: u16 = 0x2000;
    pub const TOO_COLD_CHARGING: u16 = 0x1000;
    pub const OVERVOLTAGE: u16 = 0x0800;
    pub const OVERCURRENT_CHARGING: u16 = 0x0400;
    pub const CAPACITY_OVERFLOW: u16 = 0x0200;
    pub const PREQUALIFICATION_TIMEOUT: u16 = 0x0100;
    pub const IMBALANCE: u16 = 0x0080;
    pub const PERMANENT_FAILURE: u16 = 0x0040;
    pub const DIE_TOO_HOT: u16 = 0x0020;
    pub const TOO_HOT_DISCHARGING: u16 = 0x0010;
    pub const UNDERVOLTAGE: u16 = 0x0008;
    pub const OVERCURRENT_DISCHARGING: u16 = 0x0004;
    pub const RESISTANCE_FAULT: u16 = 0x0002;
    /// On ProtStatus this is Ship; on ProtAlrt it is Leakage Detected.
    pub const SHIP_OR_LEAK: u16 = 0x0001;
    pub const fn contains(self, bits: u16) -> bool {
        self.0 & bits == bits
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub raw: u16,
    pub status2: u16,
    pub protection: ProtectionFlags,
    pub protection_alerts: ProtectionFlags,
    pub battery_status: u16,
}
impl Status {
    pub const fn power_on_reset(self) -> bool {
        self.raw & 2 != 0
    }
    pub const fn hibernating(self) -> bool {
        self.status2 & 2 != 0
    }
    pub const fn protection_alert_pending(self) -> bool {
        self.raw & 0x8000 != 0
    }
    pub const fn permanent_failure(self) -> bool {
        self.protection.contains(ProtectionFlags::PERMANENT_FAILURE)
            || self.battery_status & 0x8000 != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurements {
    /// Cell1..CellN; unused entries are `None`, not fabricated readings.
    pub cell_microvolts: [Option<u32>; 4],
    pub lowest_cell_microvolts: u32,
    pub battery_microvolts: u32,
    pub pack_terminal_microvolts: u32,
    pub current_microamps: i64,
    pub average_current_microamps: i64,
    pub reported_soc_percent: f32,
    pub reported_capacity_microamp_hours: u64,
    pub full_capacity_microamp_hours: u64,
    /// Gauge-selected temperature may combine channels; not necessarily die.
    pub gauge_temperature_celsius: f32,
    pub die_temperature_celsius: f32,
    pub time_to_empty_ms: u32,
    pub time_to_full_ms: u32,
}

pub struct Max17320<B> {
    bus: B,
    configuration: Configuration,
    identified: bool,
}
impl<B> Max17320<B> {
    pub const fn new(bus: B, configuration: Configuration) -> Self {
        Self {
            bus,
            configuration,
            identified: false,
        }
    }
    pub fn release(self) -> B {
        self.bus
    }
    pub const fn configuration(&self) -> Configuration {
        self.configuration
    }
    pub fn invalidate_identity(&mut self) {
        self.identified = false;
    }
}

impl<B: I2c> Max17320<B> {
    fn read_raw(&mut self, register: Register) -> Result<u16, Failure<B::Error>> {
        if register.is_sbs_block() {
            return Err(Failure::SbsBlockRequired);
        }
        let mut bytes = [0; 2];
        self.bus
            .write_read(
                register.bus_address(),
                &[register.command_byte()],
                &mut bytes,
            )
            .map_err(Failure::Bus)?;
        Ok(u16::from_le_bytes(bytes))
    }
    fn write_raw(&mut self, register: Register, value: u16) -> Result<(), B::Error> {
        let bytes = value.to_le_bytes();
        self.bus.write(
            register.bus_address(),
            &[register.command_byte(), bytes[0], bytes[1]],
        )
    }
    pub fn read_register(&mut self, register: Register) -> Result<u16, Error<B::Error>> {
        self.read_raw(register).map_err(Into::into)
    }
    pub fn verify_identity(&mut self) -> Result<u16, Error<B::Error>> {
        self.identified = false;
        let value = self.read_raw(Register::DEVICE_NAME)?;
        if !matches!(value, 0x4209..=0x420b) {
            return Err(Failure::Identity(value).into());
        }
        self.identified = true;
        Ok(value)
    }
    fn require_identity(&mut self) -> Result<(), Error<B::Error>> {
        if !self.identified {
            self.verify_identity()?;
        }
        Ok(())
    }
    /// Unique ROM ID, nROMID3:0, p.121. This does not authenticate a battery.
    pub fn rom_id(&mut self) -> Result<u64, Error<B::Error>> {
        let mut id = 0;
        for i in 0..4 {
            id |= u64::from(self.read_raw(Register::new(0x1bc + i).unwrap())?) << (i * 16);
        }
        Ok(id)
    }
    pub fn communication_status(&mut self) -> Result<CommunicationStatus, Error<B::Error>> {
        Ok(CommunicationStatus(
            self.read_raw(Register::COMMUNICATION_STATUS)?,
        ))
    }
    fn protected<T>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> Result<T, Failure<B::Error>>,
    ) -> Result<T, Error<B::Error>> {
        self.require_identity()?;
        let original = self.read_raw(Register::COMMUNICATION_STATUS)?;
        if original & NV_BUSY != 0 {
            return Err(Failure::Busy.into());
        }
        // Preserve CHGOff, DISOff and an existing NVError during both unlock
        // writes. Writing literal zero would silently release FET overrides.
        let unlocked = original & (FET_OVERRIDE | NV_ERROR);
        let mut result = self
            .write_raw(Register::COMMUNICATION_STATUS, unlocked)
            .map_err(Failure::Bus);
        if result.is_ok() {
            result = self
                .write_raw(Register::COMMUNICATION_STATUS, unlocked)
                .map_err(Failure::Bus);
        }
        let result = result.and_then(|()| operation(self));
        // Both consecutive writes are attempted even after the first failed.
        let (state, state_error) = match self.read_raw(Register::COMMUNICATION_STATUS) {
            Ok(state) => (state, None),
            Err(Failure::Bus(error)) => (unlocked, Some(error)),
            _ => unreachable!(),
        };
        let locked = (state & (FET_OVERRIDE | NV_ERROR)) | ALL_WRITE_PROTECT;
        let first = self.write_raw(Register::COMMUNICATION_STATUS, locked).err();
        let second = self.write_raw(Register::COMMUNICATION_STATUS, locked).err();
        match state_error.or(first).or(second) {
            Some(error) => Err(Error::Relock {
                operation: result.err(),
                error,
            }),
            None => result.map_err(Into::into),
        }
    }
    /// Low-level maintenance scope, including SHA data, history, shadow RAM,
    /// resets and permanent commands. Caller validates every raw address/value,
    /// silicon-revision-specific mask, supply and timing. NVM writes are not
    /// implied by writing shadow RAM; a copy command is required explicitly.
    /// Returning normally (including `Err`) always attempts write-protection
    /// restoration. A panic/abort cannot run cleanup. Never nest maintenance.
    pub fn maintenance<T>(
        &mut self,
        operation: impl FnOnce(&mut Maintenance<'_, B>) -> Result<T, Failure<B::Error>>,
    ) -> Result<T, Error<B::Error>> {
        self.protected(|device| operation(&mut Maintenance { device }))
    }
    /// Explicit raw volatile write. Command/CommStat/permanent-lock and SBS
    /// registers require `maintenance`; writes to shadow RAM use `write_shadow`.
    /// Does not infer reserved field masks: check the exact register definition.
    pub fn write_register(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<B::Error>> {
        if register.address() >= 0x100
            || matches!(register.address(), 0x060 | 0x061 | 0x07f)
            || !volatile_writable(register)
        {
            return Err(Failure::ReadOnly.into());
        }
        self.protected(|device| device.write_raw(register, value).map_err(Failure::Bus))
    }
    /// Validate the complete address list before I/O, then update shadow RAM in
    /// one unlock/relock scope. Excludes factory ROM IDs/reserved locations.
    /// A transport failure can leave a partial shadow image; do not copy it to
    /// NVM until every intended value/checksum has been verified.
    pub fn write_shadow(&mut self, values: &[(Register, u16)]) -> Result<(), Error<B::Error>> {
        if values.iter().any(|(r, _)| !r.shadow_writable()) {
            return Err(Failure::ReadOnly.into());
        }
        if values.is_empty() {
            return Ok(());
        }
        self.protected(|device| {
            for &(register, value) in values {
                device.write_raw(register, value).map_err(Failure::Bus)?;
            }
            Ok(())
        })
    }
    pub fn status(&mut self) -> Result<Status, Error<B::Error>> {
        Ok(Status {
            raw: self.read_raw(Register::STATUS)?,
            status2: self.read_raw(Register::STATUS2)?,
            protection: ProtectionFlags(self.read_raw(Register::PROTECTION_STATUS)?),
            protection_alerts: ProtectionFlags(self.read_raw(Register::PROTECTION_ALERT)?),
            battery_status: self.read_raw(Register::N_BATTERY_STATUS)?,
        })
    }
    /// Clear selected host-clearable status latches. PA requires clearing all
    /// ProtAlrt flags first; use `clear_protection_alerts` for that sequence.
    /// Writes ones to unselected W0C bits so new asynchronous flags survive.
    pub fn clear_status(&mut self, flags: u16) -> Result<(), Error<B::Error>> {
        if flags & !STATUS_CLEARABLE != 0 || flags & 0x8000 != 0 {
            return Err(Failure::InvalidValue.into());
        }
        self.protected(|device| {
            device
                .write_raw(Register::STATUS, !flags)
                .map_err(Failure::Bus)
        })
    }
    /// Takes the observed protection history, then clears ProtAlrt before PA as
    /// required by p.123. A concurrent new fault can be relatched by hardware;
    /// inspect current status after servicing an interrupt.
    pub fn clear_protection_alerts(&mut self) -> Result<ProtectionFlags, Error<B::Error>> {
        self.protected(|device| {
            let observed = ProtectionFlags(device.read_raw(Register::PROTECTION_ALERT)?);
            device
                .write_raw(Register::PROTECTION_ALERT, 0)
                .map_err(Failure::Bus)?;
            device
                .write_raw(Register::STATUS, 0x7fff)
                .map_err(Failure::Bus)?;
            Ok(observed)
        })
    }
    /// Set only stable Config fields; never replay a SHIP command from readback.
    /// Alert thresholds should be initialized before enabling the ALRT output.
    pub fn set_alert_output(&mut self, enabled: bool) -> Result<(), Error<B::Error>> {
        self.protected(|device| {
            let old = device.read_raw(Register::CONFIG)? & 0x7e5d;
            device
                .write_raw(Register::CONFIG, (old & !4) | if enabled { 4 } else { 0 })
                .map_err(Failure::Bus)
        })
    }
    pub fn set_voltage_alerts_mv(
        &mut self,
        minimum: u16,
        maximum: u16,
    ) -> Result<(), Error<B::Error>> {
        if minimum > maximum
            || maximum > 5100
            || !minimum.is_multiple_of(20)
            || !maximum.is_multiple_of(20)
        {
            return Err(Failure::InvalidValue.into());
        }
        self.write_register(
            Register::VOLTAGE_ALERT,
            ((maximum / 20) << 8) | (minimum / 20),
        )
    }
    pub fn set_temperature_alerts_celsius(
        &mut self,
        minimum: i8,
        maximum: i8,
    ) -> Result<(), Error<B::Error>> {
        if minimum > maximum {
            return Err(Failure::InvalidValue.into());
        }
        self.write_register(
            Register::TEMPERATURE_ALERT,
            u16::from_le_bytes([minimum as u8, maximum as u8]),
        )
    }
    pub fn set_soc_alerts_percent(
        &mut self,
        minimum: u8,
        maximum: u8,
    ) -> Result<(), Error<B::Error>> {
        if minimum > maximum {
            return Err(Failure::InvalidValue.into());
        }
        self.write_register(Register::SOC_ALERT, u16::from_le_bytes([minimum, maximum]))
    }
    /// Signed current threshold codes, each 400 µV / fitted Rsense. Raw codes
    /// avoid silently rounding a physical protection/current limit upward.
    pub fn set_current_alert_codes(
        &mut self,
        minimum: i8,
        maximum: i8,
    ) -> Result<(), Error<B::Error>> {
        if minimum > maximum {
            return Err(Failure::InvalidValue.into());
        }
        self.write_register(
            Register::CURRENT_ALERT,
            u16::from_le_bytes([minimum as u8, maximum as u8]),
        )
    }
    pub fn cell_microvolts(&mut self, cell: u8) -> Result<u32, Error<B::Error>> {
        if cell == 0 || cell > self.configuration.cells {
            return Err(Failure::InvalidValue.into());
        }
        Ok(cell_voltage_microvolts(self.read_raw(
            Register::new(0x0d9 - u16::from(cell)).unwrap(),
        )?))
    }
    pub fn die_temperature_celsius(&mut self) -> Result<f32, Error<B::Error>> {
        Ok(temperature_celsius(
            self.read_raw(Register::DIE_TEMPERATURE)?,
        ))
    }
    /// Requires the thermistor channel to have been enabled/configured in the
    /// device. Missing/disabled thermistors are not inferred from the raw value.
    pub fn thermistor_celsius(&mut self, channel: u8) -> Result<f32, Error<B::Error>> {
        if !(1..=4).contains(&channel) {
            return Err(Failure::InvalidValue.into());
        }
        Ok(temperature_celsius(self.read_raw(
            Register::new(0x13b - u16::from(channel)).unwrap(),
        )?))
    }
    pub fn measurements(&mut self) -> Result<Measurements, Error<B::Error>> {
        let mut cells = [None; 4];
        for cell in 1..=self.configuration.cells {
            cells[usize::from(cell - 1)] = Some(self.cell_microvolts(cell)?);
        }
        let cfg = self.configuration;
        Ok(Measurements {
            cell_microvolts: cells,
            lowest_cell_microvolts: cell_voltage_microvolts(self.read_raw(Register::CELL_VOLTAGE)?),
            battery_microvolts: pack_voltage_microvolts(self.read_raw(Register::BATTERY_VOLTAGE)?),
            pack_terminal_microvolts: pack_voltage_microvolts(
                self.read_raw(Register::PACK_TERMINAL_VOLTAGE)?,
            ),
            current_microamps: cfg.current_microamps(self.read_raw(Register::CURRENT)?),
            average_current_microamps: cfg
                .current_microamps(self.read_raw(Register::AVERAGE_CURRENT)?),
            reported_soc_percent: percentage(self.read_raw(Register::REPORTED_SOC)?),
            reported_capacity_microamp_hours: cfg
                .capacity_microamp_hours(self.read_raw(Register::REPORTED_CAPACITY)?),
            full_capacity_microamp_hours: cfg
                .capacity_microamp_hours(self.read_raw(Register::FULL_CAPACITY_REPORTED)?),
            gauge_temperature_celsius: temperature_celsius(self.read_raw(Register::TEMPERATURE)?),
            die_temperature_celsius: self.die_temperature_celsius()?,
            time_to_empty_ms: time_milliseconds(self.read_raw(Register::TIME_TO_EMPTY)?),
            time_to_full_ms: time_milliseconds(self.read_raw(Register::TIME_TO_FULL)?),
        })
    }
    /// Verify host conversion parameters against nPackCfg/nRSense. nRSense is
    /// host metadata in 10 µΩ units (Table 53); it does not calibrate hardware.
    pub fn verify_pack_configuration(&mut self) -> Result<(), Error<B::Error>> {
        let pack = self.read_raw(Register::N_PACK_CONFIG)?;
        let actual_cells = (pack & 3) as u8 + 2;
        let sense = self.read_raw(Register::N_SENSE_RESISTANCE)?;
        if actual_cells != self.configuration.cells {
            return Err(Failure::Verification {
                register: Register::N_PACK_CONFIG,
                expected: u16::from(self.configuration.cells - 2),
                actual: pack & 3,
            }
            .into());
        }
        if u32::from(sense) * 10 != self.configuration.sense_microohms {
            return Err(Failure::Verification {
                register: Register::N_SENSE_RESISTANCE,
                expected: (self.configuration.sense_microohms / 10).min(u32::from(u16::MAX)) as u16,
                actual: sense,
            }
            .into());
        }
        Ok(())
    }
    /// Read a single SBS word including and verifying PEC. Requires SBS enabled
    /// (nNVCfg0.enSBS=1), not the mutually exclusive ADC FIFO mode. Never treats SBS
    /// temperatures/currents as ModelGauge-format values. SBS scaling depends on
    /// nSBSCfg / sBatteryMode and silicon revision; raw words preserve that fact.
    pub fn read_sbs_word_pec(&mut self, register: Register) -> Result<u16, Error<B::Error>> {
        if !register.is_sbs() {
            return Err(Failure::InvalidValue.into());
        }
        if register.is_sbs_block() {
            return Err(Failure::SbsBlockRequired.into());
        }
        let mut bytes = [0; 3];
        self.bus
            .write_read(SECONDARY_ADDRESS, &[register.command_byte()], &mut bytes)
            .map_err(Failure::Bus)?;
        let crc = pec(&[
            SECONDARY_ADDRESS << 1,
            register.command_byte(),
            (SECONDARY_ADDRESS << 1) | 1,
            bytes[0],
            bytes[1],
        ]);
        if crc != bytes[2] {
            return Err(Failure::Pec.into());
        }
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }
    /// Raw SBS block wire transfer. `wire` includes the size byte, payload and
    /// optional PEC; caller supplies the exact expected wire length for its
    /// enabled SBS mode/command (Table117). It is not a sequence of raw words.
    /// No payload is silently truncated or interpreted as a fixed-size word.
    pub fn read_sbs_block_raw(
        &mut self,
        register: Register,
        wire: &mut [u8],
        check_pec: bool,
    ) -> Result<(), Error<B::Error>> {
        if !register.is_sbs_block()
            || wire.len() < 2
            || wire.len() > 34
            || (check_pec && wire.len() < 3)
        {
            return Err(Failure::InvalidValue.into());
        }
        self.bus
            .write_read(SECONDARY_ADDRESS, &[register.command_byte()], wire)
            .map_err(Failure::Bus)?;
        if check_pec {
            let mut crc = pec(&[
                SECONDARY_ADDRESS << 1,
                register.command_byte(),
                (SECONDARY_ADDRESS << 1) | 1,
            ]);
            for &byte in &wire[..wire.len() - 1] {
                crc = pec_byte(crc, byte);
            }
            if crc != wire[wire.len() - 1] {
                return Err(Failure::Pec.into());
            }
        }
        Ok(())
    }
    /// Recall the shadow image, discarding unsaved shadow edits. No persistent
    /// write is consumed. Waits maximum tRECALL=5ms and checks NVBusy/NVError.
    pub fn recall_nonvolatile<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<B::Error>> {
        self.protected(|device| {
            device.clear_nv_error()?;
            device
                .write_raw(Register::COMMAND, 0xe001)
                .map_err(Failure::Bus)?;
            delay.delay_ms(5);
            device.check_nv_complete()
        })
    }
    /// Recall write-history flags and return remaining configuration copies.
    /// The maximum is seven total copies, including factory programming. No
    /// persistent write is performed by querying the history page.
    pub fn remaining_configuration_updates<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<u8, Error<B::Error>> {
        self.protected(|device| {
            device.clear_nv_error()?;
            device
                .write_raw(Register::COMMAND, 0xe29b)
                .map_err(Failure::Bus)?;
            delay.delay_ms(5);
            device.check_nv_complete()?;
            let raw = device.read_raw(Register::new(0x1fd).unwrap())?;
            let used = ((raw as u8) | ((raw >> 8) as u8)).count_ones() as u8;
            Ok(7_u8.saturating_sub(used))
        })
    }
    /// Explicit irreversible NVM copy of the **whole currently verified shadow
    /// image** (including its required protection checksum). Ensure supply and
    /// remaining-write budget per pp.128–130 before calling. This consumes one
    /// limited copy attempt even when verification fails. Waits the datasheet
    /// maximum tBLOCK=7360ms, checks NVBusy/NVError, and relocks without retrying.
    /// It does not perform the subsequent reset: call `reset` when appropriate.
    pub fn copy_shadow_to_nonvolatile<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<B::Error>> {
        self.protected(|device| {
            device.clear_nv_error()?;
            device
                .write_raw(Register::COMMAND, 0xe904)
                .map_err(Failure::Bus)?;
            delay.delay_ms(7_360);
            device.check_nv_complete()
        })
    }
    fn clear_nv_error(&mut self) -> Result<(), Failure<B::Error>> {
        let value = self.read_raw(Register::COMMUNICATION_STATUS)? & FET_OVERRIDE;
        self.write_raw(Register::COMMUNICATION_STATUS, value)
            .map_err(Failure::Bus)
    }
    fn check_nv_complete(&mut self) -> Result<(), Failure<B::Error>> {
        let status = self.read_raw(Register::COMMUNICATION_STATUS)?;
        if status & NV_BUSY != 0 {
            return Err(Failure::Busy);
        }
        if status & NV_ERROR != 0 {
            return Err(Failure::Nonvolatile);
        }
        Ok(())
    }
    /// Explicit hardware+fuel-gauge reset (pp.158–159), or fuel-gauge-only
    /// reset. Polling has a caller-supplied finite millisecond limit. Hardware
    /// reset recalls NVM and may change outputs; gauge-only reset preserves FETs.
    pub fn reset<D: DelayNs>(
        &mut self,
        hardware: bool,
        timeout_ms: u32,
        delay: &mut D,
    ) -> Result<(), Error<B::Error>> {
        if timeout_ms == 0 {
            return Err(Failure::InvalidValue.into());
        }
        self.protected(|device| {
            if hardware {
                device
                    .write_raw(Register::COMMAND, 0x000f)
                    .map_err(Failure::Bus)?;
                delay.delay_ms(10);
                // Full reset restores write protection. Unlock again as specified.
                let unlocked =
                    device.read_raw(Register::COMMUNICATION_STATUS)? & (FET_OVERRIDE | NV_ERROR);
                device
                    .write_raw(Register::COMMUNICATION_STATUS, unlocked)
                    .map_err(Failure::Bus)?;
                device
                    .write_raw(Register::COMMUNICATION_STATUS, unlocked)
                    .map_err(Failure::Bus)?;
            }
            device
                .write_raw(Register::CONFIG2, 0x8000)
                .map_err(Failure::Bus)?;
            for _ in 0..timeout_ms {
                delay.delay_ms(1);
                if device.read_raw(Register::CONFIG2)? & 0x8000 == 0 {
                    return Ok(());
                }
            }
            Err(Failure::Timeout)
        })
    }
}

/// Raw register operations inside an identified, unlocked maintenance scope.
/// No reserved-field or irreversible-command policy is inferred for the caller.
pub struct Maintenance<'a, B> {
    device: &'a mut Max17320<B>,
}
impl<B: I2c> Maintenance<'_, B> {
    pub fn read_register(&mut self, register: Register) -> Result<u16, Failure<B::Error>> {
        self.device.read_raw(register)
    }
    pub fn write_register_raw(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Failure<B::Error>> {
        self.device.write_raw(register, value).map_err(Failure::Bus)
    }
}

fn volatile_writable(register: Register) -> bool {
    matches!(register.address(), 0x000..=0x004 | 0x008..=0x00b | 0x00f | 0x010 |
        0x012..=0x013 | 0x017..=0x018 | 0x01e | 0x022..=0x023 | 0x029 |
        0x032 | 0x035 | 0x038..=0x03a | 0x042 | 0x0a0..=0x0a1 | 0x0ab..=0x0ac |
        0x0af | 0x0b8 | 0x0c0..=0x0cf | 0x0d0)
}

/// SMBus PEC (CRC-8 polynomial x^8+x^2+x+1, initial zero, no final XOR).
pub fn pec(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |crc, &byte| pec_byte(crc, byte))
}
fn pec_byte(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ 0x07
        } else {
            crc << 1
        };
    }
    crc
}

impl<B: I2c> crate::interfaces::TemperatureSensor for Max17320<B> {
    type Error = Error<B::Error>;
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error> {
        self.die_temperature_celsius()
    }
}

#[cfg(test)]
mod tests;
