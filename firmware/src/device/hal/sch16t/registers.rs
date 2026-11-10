//! Public register map: Murata SCH16T Doc. 11624 Rev. 6, Table 29.

/// Every defined public address. Reserved addresses cannot be constructed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    RateX1 = 0x01,
    RateY1 = 0x02,
    RateZ1 = 0x03,
    AccX1 = 0x04,
    AccY1 = 0x05,
    AccZ1 = 0x06,
    AccX3 = 0x07,
    AccY3 = 0x08,
    AccZ3 = 0x09,
    RateX2 = 0x0a,
    RateY2 = 0x0b,
    RateZ2 = 0x0c,
    AccX2 = 0x0d,
    AccY2 = 0x0e,
    AccZ2 = 0x0f,
    Temperature = 0x10,
    RateCounter = 0x11,
    AccCounter = 0x12,
    FrequencyCounter = 0x13,
    StatusSummary = 0x14,
    StatusSaturation = 0x15,
    StatusCommon = 0x16,
    StatusRateCommon = 0x17,
    StatusRateX = 0x18,
    StatusRateY = 0x19,
    StatusRateZ = 0x1a,
    StatusAccX = 0x1b,
    StatusAccY = 0x1c,
    StatusAccZ = 0x1d,
    StatusSyncActive = 0x1e,
    StatusInfo = 0x1f,
    FilterRate = 0x25,
    FilterAcc12 = 0x26,
    FilterAcc3 = 0x27,
    ControlRate = 0x28,
    ControlAcc12 = 0x29,
    ControlAcc3 = 0x2a,
    RateSaturation1 = 0x2b,
    RateSaturation2 = 0x2c,
    AccSaturation1 = 0x2d,
    AccSaturation2 = 0x2e,
    UserInterface = 0x33,
    SelfTest = 0x34,
    Mode = 0x35,
    Reset = 0x36,
    SystemTest = 0x37,
    Spare1 = 0x38,
    Spare2 = 0x39,
    Spare3 = 0x3a,
    AsicId = 0x3b,
    ComponentId = 0x3c,
    Serial1 = 0x3d,
    Serial2 = 0x3e,
    Serial3 = 0x3f,
}

impl Register {
    pub const fn address(self) -> u8 {
        self as u8
    }

    pub const fn is_sensor(self) -> bool {
        self.address() >= 1 && self.address() <= 0x10
    }

    /// Defined writable bit width. EOI still locks hardware configuration.
    pub const fn writable_mask(self) -> Option<u16> {
        match self {
            Self::FilterRate | Self::FilterAcc12 | Self::FilterAcc3 => Some(0x01ff),
            Self::ControlRate
            | Self::ControlAcc12
            | Self::RateSaturation1
            | Self::RateSaturation2
            | Self::AccSaturation1
            | Self::AccSaturation2 => Some(0x7fff),
            Self::ControlAcc3 => Some(0x0007),
            Self::UserInterface | Self::SystemTest => Some(0xffff),
            Self::SelfTest => Some(0x0fff),
            Self::Mode => Some(0x0003),
            Self::Reset => Some(0x000f),
            _ => None,
        }
    }
}

pub(crate) const STATUS_REGISTERS: [Register; 10] = [
    Register::StatusSummary,
    Register::StatusSaturation,
    Register::StatusCommon,
    Register::StatusRateCommon,
    Register::StatusRateX,
    Register::StatusRateY,
    Register::StatusRateZ,
    Register::StatusAccX,
    Register::StatusAccY,
    Register::StatusAccZ,
];
