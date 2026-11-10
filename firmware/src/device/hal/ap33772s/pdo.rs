//! AP33772S compressed PDO summaries: official EVB guide §5.4 pp. 20–22.
//! Never reinterpret these 16-bit values as USB's 32-bit source PDO format.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RequestError {
    Index,
    CurrentStep,
    VoltageStep,
    NotDetected,
    CurrentLimit,
    VoltageLimit,
    ElectricalLimit,
    UnknownMinimum,
    WrongCapability,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CapabilityKind {
    Fixed,
    Pps,
    Avs,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MinimumVoltage {
    Exact(u16),
    /// Actual minimum is greater than the lower bound and at most the upper.
    Bounded {
        above_mv: u16,
        at_most_mv: u16,
    },
    Unknown,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurrentRange {
    pub at_least_ma: u16,
    /// None means the device reports 5000 mA or higher.
    pub at_most_ma: Option<u16>,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Capability {
    index: u8,
    raw: u16,
}
impl Capability {
    pub fn from_raw(index: u8, raw: u16) -> Result<Self, RequestError> {
        if !(1..=13).contains(&index) {
            return Err(RequestError::Index);
        }
        Ok(Self { index, raw })
    }
    pub(super) const fn from_valid_index_raw(index: u8, raw: u16) -> Self {
        Self { index, raw }
    }
    pub const fn index(self) -> u8 {
        self.index
    }
    pub const fn raw(self) -> u16 {
        self.raw
    }
    pub const fn detected(self) -> bool {
        self.raw & 0x8000 != 0
    }
    pub const fn is_epr(self) -> bool {
        self.index >= 8
    }
    pub const fn kind(self) -> CapabilityKind {
        if self.raw & 0x4000 == 0 {
            CapabilityKind::Fixed
        } else if self.is_epr() {
            CapabilityKind::Avs
        } else {
            CapabilityKind::Pps
        }
    }
    pub const fn current_range(self) -> CurrentRange {
        let code = (self.raw >> 10) & 15;
        match code {
            0 => CurrentRange {
                at_least_ma: 0,
                at_most_ma: Some(1249),
            },
            14 => CurrentRange {
                at_least_ma: 4500,
                at_most_ma: Some(4999),
            },
            15 => CurrentRange {
                at_least_ma: 5000,
                at_most_ma: None,
            },
            _ => CurrentRange {
                at_least_ma: 1000 + code * 250,
                at_most_ma: Some(1249 + code * 250),
            },
        }
    }
    pub const fn maximum_voltage_mv(self) -> u16 {
        (self.raw & 255) * if self.is_epr() { 200 } else { 100 }
    }
    pub fn minimum_voltage(self) -> MinimumVoltage {
        if self.kind() == CapabilityKind::Fixed {
            return MinimumVoltage::Exact(self.maximum_voltage_mv());
        }
        let code = (self.raw >> 8) & 3;
        match (self.is_epr(), code) {
            (false, 1) => MinimumVoltage::Exact(3300),
            (false, 2) => MinimumVoltage::Bounded {
                above_mv: 3300,
                at_most_mv: 5000,
            },
            (true, 1) => MinimumVoltage::Exact(15000),
            (true, 2) => MinimumVoltage::Bounded {
                above_mv: 15000,
                at_most_mv: 20000,
            },
            _ => MinimumVoltage::Unknown,
        }
    }
    /// Bits9:8 are a peak-current capability code only for fixed PDOs.
    pub fn peak_current_code(self) -> Option<u8> {
        (self.kind() == CapabilityKind::Fixed).then_some(((self.raw >> 8) & 3) as u8)
    }
    pub fn encode_request(
        self,
        request: Request,
        limits: ElectricalLimits,
    ) -> Result<SubmittedRequest, RequestError> {
        request.validate_limits(limits)?;
        if request.index != self.index {
            return Err(RequestError::WrongCapability);
        }
        if !self.detected() {
            return Err(RequestError::NotDetected);
        }
        if request.current_ma > self.current_range().at_least_ma {
            return Err(RequestError::CurrentLimit);
        }
        let voltage = request.voltage_mv;
        if voltage > self.maximum_voltage_mv() || (!self.is_epr() && voltage > 21_000) {
            return Err(RequestError::VoltageLimit);
        }
        let voltage_code = if self.kind() == CapabilityKind::Fixed {
            if voltage != self.maximum_voltage_mv() {
                return Err(RequestError::VoltageLimit);
            }
            0
        } else {
            let minimum = match self.minimum_voltage() {
                MinimumVoltage::Exact(v) => v,
                MinimumVoltage::Bounded { at_most_mv, .. } => at_most_mv,
                MinimumVoltage::Unknown => return Err(RequestError::UnknownMinimum),
            };
            if voltage < minimum {
                return Err(RequestError::VoltageLimit);
            }
            let step = if self.is_epr() { 200 } else { 100 };
            if !voltage.is_multiple_of(step) {
                return Err(RequestError::VoltageStep);
            }
            voltage / step
        };
        let code = current_code(request.current_ma).ok_or(RequestError::CurrentStep)?;
        Ok(SubmittedRequest {
            word: (u16::from(self.index) << 12) | (u16::from(code) << 8) | voltage_code,
            request,
        })
    }
}

/// Explicit electrical input envelope of the fitted board and source path.
/// Power is the requested envelope(V × I), not measured consumption.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ElectricalLimits {
    minimum_mv: u16,
    maximum_mv: u16,
    maximum_ma: u16,
    maximum_mw: u32,
}
impl ElectricalLimits {
    pub const fn new(
        minimum_mv: u16,
        maximum_mv: u16,
        maximum_ma: u16,
        maximum_mw: u32,
    ) -> Option<Self> {
        if minimum_mv < 3300
            || minimum_mv > maximum_mv
            || maximum_mv > 28000
            || maximum_ma == 0
            || maximum_ma > 5000
            || maximum_mw == 0
        {
            return None;
        }
        Some(Self {
            minimum_mv,
            maximum_mv,
            maximum_ma,
            maximum_mw,
        })
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Request {
    index: u8,
    voltage_mv: u16,
    current_ma: u16,
}
impl Request {
    pub fn new(index: u8, voltage_mv: u16, current_ma: u16) -> Result<Self, RequestError> {
        if !(1..=13).contains(&index) {
            return Err(RequestError::Index);
        }
        current_code(current_ma).ok_or(RequestError::CurrentStep)?;
        if !(3300..=28000).contains(&voltage_mv) {
            return Err(RequestError::VoltageLimit);
        }
        Ok(Self {
            index,
            voltage_mv,
            current_ma,
        })
    }
    pub const fn index(self) -> u8 {
        self.index
    }
    pub const fn voltage_mv(self) -> u16 {
        self.voltage_mv
    }
    pub const fn current_ma(self) -> u16 {
        self.current_ma
    }
    pub(super) fn validate_limits(self, limits: ElectricalLimits) -> Result<(), RequestError> {
        if self.voltage_mv < limits.minimum_mv
            || self.voltage_mv > limits.maximum_mv
            || self.current_ma > limits.maximum_ma
            || u64::from(self.voltage_mv) * u64::from(self.current_ma)
                > u64::from(limits.maximum_mw) * 1000
        {
            return Err(RequestError::ElectricalLimit);
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubmittedRequest {
    word: u16,
    request: Request,
}
impl SubmittedRequest {
    pub const fn word(self) -> u16 {
        self.word
    }
    pub const fn requested(self) -> Request {
        self.request
    }
}
fn current_code(ma: u16) -> Option<u8> {
    match ma {
        1000..=4500 if ma.is_multiple_of(250) => Some(((ma - 1000) / 250) as u8),
        5000 => Some(15),
        _ => None,
    }
}
