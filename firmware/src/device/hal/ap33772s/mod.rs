//! AP33772S PD3.1 sink controller for the earlier full-size V2 board.
//!
//! Sources: Diodes [DS46176 Rev. 10 (August 2026)](https://www.diodes.com/datasheet/download/AP33772S.pdf),
//! Tables 2–3, 10–14, 18–19, and the [EVB guide Rev. 2.0 (October 2025)](https://www.diodes.com/assets/Evaluation-Boards/AP33772S-Sink-Controller-EVB-User-Guide.pdf),
//! §§5.1–5.4. No AP33772S sheet exists in the local data folder. The schematic
//! identifies AP33772SDKZ-13-FA02. The unsuffixed AP33772 uses another address
//! and register map; it is not interchangeable with this driver.
//!
//! Address 0x52 is seven-bit; multi-byte command data is little endian, without
//! a byte count or PEC. These addresses select commands, not a byte-addressed
//! memory: consecutive words use distinct commands. STATUS reads clear events;
//! INT is active-high. SYSTEM.CMDVER=1 verifies the command interface, not a
//! unique silicon identity. No unique chip-ID register is documented.
//!
//! Constructors do no I/O. Configuration belongs in the controller's 100 ms
//! startup window after STARTED, with exclusive device ownership. The backend
//! supplies bus timeouts and the application handles that startup deadline.
//! Nothing here changes voltage, output gating, protection, reset or persistent
//! settings automatically. A submitted request is not an accepted contract:
//! associate READY/events and message results with the request and read back
//! VREQ/IREQ. Those registers retain the previous successful contract on failure.
//!
//! Capabilities are the chip's compressed 16-bit summaries, not USB 32-bit PDOs.
//! Typed requests conservatively use lower current bounds and upper minimum-
//! voltage bounds where the summary is lossy; unknown voltage minima are rejected.
//! They never use the special maximum-current/voltage request. A source can
//! change between reads and negotiation, so physical input limits/protection
//! and final negotiation verification remain necessary. Raw command writes
//! intentionally allow documented low-level operations, including maximum
//! requests; their caller owns all electrical and protocol validation.

mod pdo;
use embedded_hal::i2c::I2c;
pub use pdo::{
    Capability, CapabilityKind, CurrentRange, ElectricalLimits, MinimumVoltage, Request,
    RequestError, SubmittedRequest,
};

pub const ADDRESS: u8 = 0x52;
pub const MAX_I2C_HZ: u32 = 400_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Variant {
    Fa01,
    Fa02,
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidRegister,
    InvalidLength,
    ReadOnly,
    WriteOnly,
    InvalidValue,
    Unsupported,
    CommandVersion(u8),
    NotPdConnected,
    Request(RequestError),
}

/// Complete public command set. SourcePdo indices are 1..=13. The FA02 GPIO
/// command and data-role-swap bits additionally follow the newer datasheet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Register {
    Status,
    InterruptEnable,
    OperatingMode,
    Configuration,
    PdConfiguration,
    System,
    Thermistor25,
    Thermistor50,
    Thermistor75,
    Thermistor100,
    OutputVoltage,
    OutputCurrent,
    Temperature,
    RequestedVoltage,
    RequestedCurrent,
    MinimumSelectionVoltage,
    UnderVoltageThreshold,
    OverVoltageThreshold,
    OverCurrentThreshold,
    OverTemperatureThreshold,
    DeratingThreshold,
    SourcePdos,
    SourcePdo(u8),
    PdRequest,
    PdCommand,
    PdResult,
    Gpio,
}
impl Register {
    pub const fn command(self) -> Option<u8> {
        Some(match self {
            Self::Status => 0x01,
            Self::InterruptEnable => 0x02,
            Self::OperatingMode => 0x03,
            Self::Configuration => 0x04,
            Self::PdConfiguration => 0x05,
            Self::System => 0x06,
            Self::Thermistor25 => 0x0c,
            Self::Thermistor50 => 0x0d,
            Self::Thermistor75 => 0x0e,
            Self::Thermistor100 => 0x0f,
            Self::OutputVoltage => 0x11,
            Self::OutputCurrent => 0x12,
            Self::Temperature => 0x13,
            Self::RequestedVoltage => 0x14,
            Self::RequestedCurrent => 0x15,
            Self::MinimumSelectionVoltage => 0x16,
            Self::UnderVoltageThreshold => 0x17,
            Self::OverVoltageThreshold => 0x18,
            Self::OverCurrentThreshold => 0x19,
            Self::OverTemperatureThreshold => 0x1a,
            Self::DeratingThreshold => 0x1b,
            Self::SourcePdos => 0x20,
            Self::SourcePdo(index) => {
                if index >= 1 && index <= 13 {
                    0x20 + index
                } else {
                    return None;
                }
            }
            Self::PdRequest => 0x31,
            Self::PdCommand => 0x32,
            Self::PdResult => 0x33,
            Self::Gpio => 0x52,
        })
    }
    pub const fn length(self) -> usize {
        match self {
            Self::SourcePdos => 26,
            Self::Thermistor25
            | Self::Thermistor50
            | Self::Thermistor75
            | Self::Thermistor100
            | Self::OutputVoltage
            | Self::RequestedVoltage
            | Self::RequestedCurrent
            | Self::SourcePdo(_)
            | Self::PdRequest => 2,
            _ => 1,
        }
    }
    const fn writable(self) -> bool {
        !matches!(
            self,
            Self::Status
                | Self::OperatingMode
                | Self::OutputVoltage
                | Self::OutputCurrent
                | Self::Temperature
                | Self::RequestedVoltage
                | Self::RequestedCurrent
                | Self::SourcePdos
                | Self::SourcePdo(_)
                | Self::PdResult
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Events(pub u8);
impl Events {
    pub const fn started(self) -> bool {
        self.0 & 1 != 0
    }
    pub const fn ready(self) -> bool {
        self.0 & 2 != 0
    }
    pub const fn new_capabilities(self) -> bool {
        self.0 & 4 != 0
    }
    pub const fn under_voltage(self) -> bool {
        self.0 & 8 != 0
    }
    pub const fn over_voltage(self) -> bool {
        self.0 & 16 != 0
    }
    pub const fn over_current(self) -> bool {
        self.0 & 32 != 0
    }
    pub const fn over_temperature(self) -> bool {
        self.0 & 64 != 0
    }
    pub const fn faulted(self) -> bool {
        self.0 & 0x78 != 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OperatingMode(pub u8);
impl OperatingMode {
    pub const fn pd_connected(self) -> bool {
        self.0 & 2 != 0
    }
    pub const fn legacy_connected(self) -> bool {
        self.0 & 1 != 0
    }
    pub const fn cc2_connected(self) -> bool {
        self.0 & 0x80 != 0
    }
    pub const fn derating(self) -> bool {
        self.0 & 0x40 != 0
    }
    /// FA02 only; FA01 does not document bit 5.
    pub const fn data_role_bit(self) -> bool {
        self.0 & 0x20 != 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MessageResult {
    BusyOrNoResponse,
    Success,
    InvalidArgument,
    Rejected,
    NoGoodCrc,
    Reserved(u8),
}
impl MessageResult {
    pub const fn from_raw(raw: u8) -> Self {
        match raw & 7 {
            0 => Self::BusyOrNoResponse,
            1 => Self::Success,
            2 => Self::InvalidArgument,
            3 => Self::Rejected,
            4 => Self::NoGoodCrc,
            n => Self::Reserved(n),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum OutputControl {
    Automatic = 0,
    ForceOff = 1,
    ForceOn = 2,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum UnderVoltage {
    Percent80 = 1,
    Percent75 = 2,
    Percent70 = 3,
}

/// Host-supplied NTC characterization; no default curve is selected.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ThermistorPoints {
    pub ohms_at25: u16,
    pub ohms_at50: u16,
    pub ohms_at75: u16,
    pub ohms_at100: u16,
}
impl ThermistorPoints {
    fn valid(self) -> bool {
        self.ohms_at25 > self.ohms_at50
            && self.ohms_at50 > self.ohms_at75
            && self.ohms_at75 > self.ohms_at100
            && self.ohms_at100 > 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Thresholds {
    pub minimum_selection_mv: u16,
    pub under_voltage: UnderVoltage,
    pub over_voltage_offset_mv: u16,
    /// None selects the controller's automatic 110% of source capability.
    /// Some(mA) programs the reference, whose actual trip point is 110%.
    pub over_current_reference_ma: Option<u16>,
    pub over_temperature_celsius: u8,
    pub derating_celsius: u8,
}
impl Thresholds {
    fn valid(self) -> bool {
        self.minimum_selection_mv <= 28_000
            && self.minimum_selection_mv.is_multiple_of(200)
            && self.over_voltage_offset_mv <= 20_400
            && self.over_voltage_offset_mv.is_multiple_of(80)
            && self
                .over_current_reference_ma
                .is_none_or(|v| v > 0 && v <= 12_750 && v.is_multiple_of(50))
    }
}

/// Sequential observations; no hardware atomic snapshot. TEMP is the chip's
/// temperature bit pattern; negative-temperature coding is not specified by
/// these documents. Interpretation also requires the fitted NTC points. CURRENT uses the fitted hardware current-sense path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Measurements {
    pub output_voltage_mv: u32,
    pub output_current_ma: u16,
    pub temperature_register: u8,
    pub requested_voltage_mv: u32,
    pub requested_current_ma: u32,
}

pub struct Ap33772s<B> {
    bus: B,
    variant: Variant,
    verified: bool,
}
impl<B> Ap33772s<B> {
    pub const fn new(bus: B, variant: Variant) -> Self {
        Self {
            bus,
            variant,
            verified: false,
        }
    }
    pub fn release(self) -> B {
        self.bus
    }
    pub const fn variant(&self) -> Variant {
        self.variant
    }
    pub fn invalidate_interface(&mut self) {
        self.verified = false;
    }
}
impl<B: I2c> Ap33772s<B> {
    /// Verify the documented command protocol version. This is not a unique
    /// silicon identity and cannot distinguish the two fitted firmware options.
    pub fn verify_interface(&mut self) -> Result<u8, Error<B::Error>> {
        self.verified = false;
        let version = (self.read_byte(Register::System)? >> 4) & 3;
        if version != 1 {
            return Err(Error::CommandVersion(version));
        }
        self.verified = true;
        Ok(version)
    }
    fn require_interface(&mut self) -> Result<(), Error<B::Error>> {
        if !self.verified {
            self.verify_interface()?;
        }
        Ok(())
    }
    fn validate_register(&self, register: Register, length: usize) -> Result<u8, Error<B::Error>> {
        let command = register.command().ok_or(Error::InvalidRegister)?;
        if length != register.length() {
            return Err(Error::InvalidLength);
        }
        if register == Register::Gpio && self.variant != Variant::Fa02 {
            return Err(Error::Unsupported);
        }
        Ok(command)
    }
    pub fn read_register(
        &mut self,
        register: Register,
        data: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        let command = self.validate_register(register, data.len())?;
        if matches!(register, Register::PdRequest | Register::PdCommand) {
            return Err(Error::WriteOnly);
        }
        self.bus
            .write_read(ADDRESS, &[command], data)
            .map_err(Error::Bus)
    }
    /// Raw command write, validating width, RO/reserved fields and interface
    /// version. PD requests here bypass capability/electrical-limit checks:
    /// prefer `request` for checked negotiation. No memory auto-increment occurs.
    pub fn write_register(
        &mut self,
        register: Register,
        data: &[u8],
    ) -> Result<(), Error<B::Error>> {
        let command = self.validate_register(register, data.len())?;
        if !register.writable() {
            return Err(Error::ReadOnly);
        }
        let value = data[0];
        let valid = match register {
            Register::InterruptEnable => value & 0x80 == 0,
            Register::Configuration => value & 7 == 0,
            Register::PdConfiguration => {
                value & !if self.variant == Variant::Fa02 { 7 } else { 3 } == 0
            }
            Register::System => value <= 2,
            Register::UnderVoltageThreshold => matches!(value, 1..=3),
            Register::PdCommand => value & !if self.variant == Variant::Fa02 { 3 } else { 1 } == 0,
            Register::Gpio => value & !0xc7 == 0,
            Register::PdRequest => matches!(u16::from_le_bytes([data[0], data[1]]) >> 12, 1..=13),
            _ => true,
        };
        if !valid {
            return Err(Error::InvalidValue);
        }
        self.require_interface()?;
        let mut packet = [0; 3];
        packet[0] = command;
        packet[1..=data.len()].copy_from_slice(data);
        self.bus
            .write(ADDRESS, &packet[..data.len() + 1])
            .map_err(Error::Bus)
    }
    pub fn read_byte(&mut self, register: Register) -> Result<u8, Error<B::Error>> {
        let mut bytes = [0];
        self.read_register(register, &mut bytes)?;
        Ok(bytes[0])
    }
    pub fn read_word(&mut self, register: Register) -> Result<u16, Error<B::Error>> {
        let mut bytes = [0; 2];
        self.read_register(register, &mut bytes)?;
        Ok(u16::from_le_bytes(bytes))
    }
    pub fn write_byte(&mut self, register: Register, value: u8) -> Result<(), Error<B::Error>> {
        self.write_register(register, &[value])
    }
    pub fn write_word(&mut self, register: Register, value: u16) -> Result<(), Error<B::Error>> {
        self.write_register(register, &value.to_le_bytes())
    }
    /// Acknowledge all current interrupt events in one read. Cache the returned
    /// value if multiple consumers need those events; another read clears them.
    pub fn take_events(&mut self) -> Result<Events, Error<B::Error>> {
        self.read_byte(Register::Status).map(Events)
    }
    /// One bit enables the corresponding event interrupt; this is not an
    /// inverted conventional mask. INT is high while an enabled event is pending.
    pub fn set_interrupt_enable(&mut self, events: Events) -> Result<(), Error<B::Error>> {
        self.write_byte(Register::InterruptEnable, events.0)
    }
    pub fn operating_mode(&mut self) -> Result<OperatingMode, Error<B::Error>> {
        self.read_byte(Register::OperatingMode).map(OperatingMode)
    }
    pub fn message_result(&mut self) -> Result<MessageResult, Error<B::Error>> {
        self.read_byte(Register::PdResult)
            .map(MessageResult::from_raw)
    }
    pub fn set_output_control(&mut self, control: OutputControl) -> Result<(), Error<B::Error>> {
        self.write_byte(Register::System, control as u8)
    }
    /// Explicit PD hard reset. It can interrupt power; no automatic request follows.
    pub fn hard_reset(&mut self) -> Result<(), Error<B::Error>> {
        self.write_byte(Register::PdCommand, 1)
    }
    pub fn data_role_swap(&mut self) -> Result<(), Error<B::Error>> {
        if self.variant != Variant::Fa02 {
            return Err(Error::Unsupported);
        }
        self.write_byte(Register::PdCommand, 2)
    }
    /// Protection enables in bit7..3: de-rating, temperature, current,
    /// overvoltage, undervoltage. No defaults are applied implicitly.
    pub fn set_protection_enables(
        &mut self,
        derating: bool,
        temperature: bool,
        current: bool,
        over_voltage: bool,
        under_voltage: bool,
    ) -> Result<(), Error<B::Error>> {
        self.write_byte(
            Register::Configuration,
            (u8::from(derating) << 7)
                | (u8::from(temperature) << 6)
                | (u8::from(current) << 5)
                | (u8::from(over_voltage) << 4)
                | (u8::from(under_voltage) << 3),
        )
    }
    pub fn set_pd_features(
        &mut self,
        epr: bool,
        pps_avs: bool,
        accept_data_role_swap: bool,
    ) -> Result<(), Error<B::Error>> {
        if accept_data_role_swap && self.variant != Variant::Fa02 {
            return Err(Error::Unsupported);
        }
        self.write_byte(
            Register::PdConfiguration,
            u8::from(epr) | (u8::from(pps_avs) << 1) | (u8::from(accept_data_role_swap) << 2),
        )
    }
    pub fn configure_thermistor(
        &mut self,
        points: ThermistorPoints,
    ) -> Result<(), Error<B::Error>> {
        if !points.valid() {
            return Err(Error::InvalidValue);
        }
        for (register, value) in [
            (Register::Thermistor25, points.ohms_at25),
            (Register::Thermistor50, points.ohms_at50),
            (Register::Thermistor75, points.ohms_at75),
            (Register::Thermistor100, points.ohms_at100),
        ] {
            self.write_word(register, value)?;
        }
        Ok(())
    }
    /// Validates the entire threshold set before I/O. Multi-command updates
    /// are not atomic; after a bus error some settings may have changed.
    pub fn configure_thresholds(&mut self, thresholds: Thresholds) -> Result<(), Error<B::Error>> {
        if !thresholds.valid() {
            return Err(Error::InvalidValue);
        }
        for (register, value) in [
            (
                Register::MinimumSelectionVoltage,
                (thresholds.minimum_selection_mv / 200) as u8,
            ),
            (
                Register::UnderVoltageThreshold,
                thresholds.under_voltage as u8,
            ),
            (
                Register::OverVoltageThreshold,
                (thresholds.over_voltage_offset_mv / 80) as u8,
            ),
            (
                Register::OverCurrentThreshold,
                (thresholds.over_current_reference_ma.unwrap_or(0) / 50) as u8,
            ),
            (
                Register::OverTemperatureThreshold,
                thresholds.over_temperature_celsius,
            ),
            (Register::DeratingThreshold, thresholds.derating_celsius),
        ] {
            self.write_byte(register, value)?;
        }
        Ok(())
    }
    pub fn measurements(&mut self) -> Result<Measurements, Error<B::Error>> {
        Ok(Measurements {
            output_voltage_mv: u32::from(self.read_word(Register::OutputVoltage)?) * 80,
            output_current_ma: u16::from(self.read_byte(Register::OutputCurrent)?) * 24,
            temperature_register: self.read_byte(Register::Temperature)?,
            requested_voltage_mv: u32::from(self.read_word(Register::RequestedVoltage)?) * 50,
            requested_current_ma: u32::from(self.read_word(Register::RequestedCurrent)?) * 10,
        })
    }
    pub fn capability(&mut self, index: u8) -> Result<Capability, Error<B::Error>> {
        let raw = self.read_word(Register::SourcePdo(index))?;
        Capability::from_raw(index, raw).map_err(Error::Request)
    }
    /// Exactly 13 little-endian summaries (26 bytes). The guide has a prose typo
    /// saying 28; its byte diagram and the current datasheet both specify 26.
    pub fn capabilities(&mut self) -> Result<[Capability; 13], Error<B::Error>> {
        let mut bytes = [0; 26];
        self.read_register(Register::SourcePdos, &mut bytes)?;
        Ok(core::array::from_fn(|i| {
            Capability::from_valid_index_raw(
                i as u8 + 1,
                u16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]]),
            )
        }))
    }
    /// Re-read connection, selected capability and relevant PD enable bits,
    /// then submit one checked request. No maximum-request escape is generated.
    /// Caller must associate fresh READY/result events and verify VREQ/IREQ.
    pub fn request(
        &mut self,
        request: Request,
        limits: ElectricalLimits,
    ) -> Result<SubmittedRequest, Error<B::Error>> {
        request.validate_limits(limits).map_err(Error::Request)?;
        if !self.operating_mode()?.pd_connected() {
            return Err(Error::NotPdConnected);
        }
        let capability = self.capability(request.index())?;
        let submitted = capability
            .encode_request(request, limits)
            .map_err(Error::Request)?;
        if capability.is_epr() || capability.kind() != CapabilityKind::Fixed {
            let features = self.read_byte(Register::PdConfiguration)?;
            if (capability.is_epr() && features & 1 == 0)
                || (capability.kind() != CapabilityKind::Fixed && features & 2 == 0)
            {
                return Err(Error::Unsupported);
            }
        }
        self.write_word(Register::PdRequest, submitted.word())?;
        Ok(submitted)
    }
}

#[cfg(test)]
mod tests;
