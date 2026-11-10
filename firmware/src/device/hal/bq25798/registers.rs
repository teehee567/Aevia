//! Complete public map: local SLUSDV2C Table 7-12 and sections 7.5.1.1–57.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ByteRegister {
    MinimalSystemVoltage = 0x00,
    InputVoltageLimit = 0x05,
    PrechargeControl = 0x08,
    TerminationControl = 0x09,
    RechargeControl = 0x0a,
    OtgCurrent = 0x0d,
    TimerControl = 0x0e,
    ChargerControl0 = 0x0f,
    ChargerControl1 = 0x10,
    ChargerControl2 = 0x11,
    ChargerControl3 = 0x12,
    ChargerControl4 = 0x13,
    ChargerControl5 = 0x14,
    MpptControl = 0x15,
    TemperatureControl = 0x16,
    NtcControl0 = 0x17,
    NtcControl1 = 0x18,
    ChargerStatus0 = 0x1b,
    ChargerStatus1 = 0x1c,
    ChargerStatus2 = 0x1d,
    ChargerStatus3 = 0x1e,
    ChargerStatus4 = 0x1f,
    FaultStatus0 = 0x20,
    FaultStatus1 = 0x21,
    ChargerFlag0 = 0x22,
    ChargerFlag1 = 0x23,
    ChargerFlag2 = 0x24,
    ChargerFlag3 = 0x25,
    FaultFlag0 = 0x26,
    FaultFlag1 = 0x27,
    ChargerMask0 = 0x28,
    ChargerMask1 = 0x29,
    ChargerMask2 = 0x2a,
    ChargerMask3 = 0x2b,
    FaultMask0 = 0x2c,
    FaultMask1 = 0x2d,
    AdcControl = 0x2e,
    AdcFunctionDisable0 = 0x2f,
    AdcFunctionDisable1 = 0x30,
    DpdmDriver = 0x47,
    PartInformation = 0x48,
}
impl ByteRegister {
    pub const fn address(self) -> u8 {
        self as u8
    }
    pub(crate) const fn write_mask(self) -> u8 {
        match self {
            Self::MinimalSystemVoltage => 0x3f,
            Self::TerminationControl => 0x7f,
            Self::ChargerControl5 => 0xbf,
            Self::NtcControl0 | Self::AdcFunctionDisable0 => 0xfe,
            Self::ChargerMask1 => 0xd7,
            Self::ChargerMask2 => 0x7f,
            Self::ChargerMask3 => 0x1f,
            Self::FaultMask1 => 0xf4,
            Self::AdcControl | Self::DpdmDriver => 0xfc,
            Self::AdcFunctionDisable1 => 0xf0,
            Self::InputVoltageLimit
            | Self::PrechargeControl
            | Self::RechargeControl
            | Self::OtgCurrent
            | Self::TimerControl
            | Self::ChargerControl0
            | Self::ChargerControl1
            | Self::ChargerControl2
            | Self::ChargerControl3
            | Self::ChargerControl4
            | Self::MpptControl
            | Self::TemperatureControl
            | Self::NtcControl1
            | Self::ChargerMask0
            | Self::FaultMask0 => 0xff,
            _ => 0,
        }
    }
    /// Explicit commands that must not be replayed by ordinary RMW.
    pub(crate) const fn actions(self) -> u8 {
        match self {
            Self::TerminationControl => 0x40, // REG_RST
            Self::ChargerControl0 => 0x08,    // FORCE_ICO
            Self::ChargerControl1 => 0x08,    // WD_RST
            Self::ChargerControl2 => 0x86,    // FORCE_INDET and ship/reset commands
            Self::ChargerControl4 => 0x02,    // FORCE_VINDPM_DET
            Self::TemperatureControl => 0x01, // BKUP_ACFET1_ON
            Self::AdcControl => 0x04,         // ADC_AVG_INIT
            _ => 0,
        }
    }
    pub(crate) const fn value_valid(self, value: u8) -> bool {
        if self.write_mask() == 0 || value & !self.write_mask() != 0 {
            return false;
        }
        match self {
            Self::MinimalSystemVoltage => value <= 54,
            Self::InputVoltageLimit => value >= 36 && value <= 220,
            Self::PrechargeControl => value & 0x3f >= 1 && value & 0x3f <= 50,
            Self::TerminationControl => value & 0x1f >= 1 && value & 0x1f <= 25,
            Self::OtgCurrent => value & 0x7f >= 4 && value & 0x7f <= 84,
            Self::DpdmDriver => (value >> 2) & 7 != 7,
            _ => true,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum WordRegister {
    ChargeVoltageLimit = 0x01,
    ChargeCurrentLimit = 0x03,
    InputCurrentLimit = 0x06,
    OtgVoltage = 0x0b,
    IcoCurrentLimit = 0x19,
    BusCurrentAdc = 0x31,
    BatteryCurrentAdc = 0x33,
    BusVoltageAdc = 0x35,
    Ac1VoltageAdc = 0x37,
    Ac2VoltageAdc = 0x39,
    BatteryVoltageAdc = 0x3b,
    SystemVoltageAdc = 0x3d,
    ThermistorAdc = 0x3f,
    DieTemperatureAdc = 0x41,
    DpVoltageAdc = 0x43,
    DmVoltageAdc = 0x45,
}
impl WordRegister {
    pub const fn address(self) -> u8 {
        self as u8
    }
    pub(crate) const fn writable_range(self) -> Option<(u16, u16)> {
        match self {
            Self::ChargeVoltageLimit => Some((300, 1880)),
            Self::ChargeCurrentLimit => Some((5, 500)),
            Self::InputCurrentLimit => Some((10, 330)),
            Self::OtgVoltage => Some((0, 1920)),
            _ => None,
        }
    }
}
