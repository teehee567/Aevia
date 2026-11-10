//! Typed settings from local datasheet §§7.4.1–7.4.5, Tables 58–78.
use super::Register;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    SaturationFilter,
    ConflictingSyncAndDataReady,
}

/// Codes shared by all filter registers. Cutoffs differ for RATE and ACC;
/// see datasheet Table 9 rather than treating the code as one frequency.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum Filter {
    #[default]
    Lpf0 = 0,
    Lpf1 = 1,
    Lpf2 = 2,
    Lpf3 = 3,
    Lpf4 = 4,
    Lpf5 = 5,
    Bypass = 7,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AxisFilters {
    pub x: Filter,
    pub y: Filter,
    pub z: Filter,
}

impl AxisFilters {
    pub const fn all(filter: Filter) -> Self {
        Self {
            x: filter,
            y: filter,
            z: filter,
        }
    }

    pub const fn bits(self) -> u16 {
        self.x as u16 | (self.y as u16) << 3 | (self.z as u16) << 6
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum RateRange {
    #[default]
    Dyn1 = 1,
    Dyn2 = 2,
    Dyn3 = 3,
    Dyn4 = 4,
}

impl RateRange {
    /// 48-bit frames carry 20-bit output values.
    pub const fn lsb_per_degree_per_second(self) -> f64 {
        match self {
            Self::Dyn1 | Self::Dyn2 => 1600.0,
            Self::Dyn3 => 3200.0,
            Self::Dyn4 => 6400.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum AccRange {
    #[default]
    Dyn1 = 1,
    Dyn2 = 2,
    Dyn3 = 3,
    Dyn4 = 4,
}

impl AccRange {
    pub const fn lsb_per_metre_per_second_squared(self) -> f64 {
        match self {
            Self::Dyn1 => 3200.0,
            Self::Dyn2 => 6400.0,
            Self::Dyn3 => 12800.0,
            Self::Dyn4 => 25600.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum AuxAccRange {
    #[default]
    Dyn0 = 0,
    Dyn1 = 1,
    Dyn2 = 2,
    Dyn3 = 3,
    Dyn4 = 4,
}

impl AuxAccRange {
    pub const fn lsb_per_metre_per_second_squared(self) -> f64 {
        match self {
            Self::Dyn0 => 1600.0,
            Self::Dyn1 => 3200.0,
            Self::Dyn2 => 6400.0,
            Self::Dyn3 => 12800.0,
            Self::Dyn4 => 25600.0,
        }
    }
}

/// DEC1..DEC5; rate = F_PRIM / (2 * reduction factor).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum Decimation {
    #[default]
    Dec1 = 0,
    Dec2 = 1,
    Dec3 = 2,
    Dec4 = 3,
    Dec5 = 4,
}

impl Decimation {
    /// Nominal only: actual F_PRIM ranges from 22.1 to 25.1 kHz.
    pub const fn nominal_output_rate_hz(self) -> f64 {
        23600.0 / (2_u32 << self as u16) as f64
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AxisDecimation {
    pub x: Decimation,
    pub y: Decimation,
    pub z: Decimation,
}

impl AxisDecimation {
    pub const fn all(value: Decimation) -> Self {
        Self {
            x: value,
            y: value,
            z: value,
        }
    }

    pub const fn bits(self) -> u16 {
        self.x as u16 | (self.y as u16) << 3 | (self.z as u16) << 6
    }
}

/// Raw five-bit fault-filter code, not an arbitrary sample count. Zero means
/// immediate detection (minimum one fault). Digital threshold otherwise code*16;
/// the pre-processing RATE analog threshold uses (code >> 1)*4 (minimum one).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SaturationFilter(u8);

impl SaturationFilter {
    pub const fn from_code(code: u8) -> Result<Self, ConfigError> {
        if code <= 31 {
            Ok(Self(code))
        } else {
            Err(ConfigError::SaturationFilter)
        }
    }

    pub const fn code(self) -> u8 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AxisSaturation {
    pub x: SaturationFilter,
    pub y: SaturationFilter,
    pub z: SaturationFilter,
}

impl AxisSaturation {
    pub const fn bits(self) -> u16 {
        self.x.code() as u16 | (self.y.code() as u16) << 5 | (self.z.code() as u16) << 10
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum IoVoltage {
    #[default]
    V2_5Or3_3 = 0,
    V1_8 = 1,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum StatusClearDelay {
    Us78 = 0,
    Us625 = 1,
    #[default]
    Ms2_5 = 2,
    Ms5 = 3,
}

/// Threshold in sensor master-clock periods, not exact host milliseconds.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u16)]
pub enum SyncTimeout {
    #[default]
    Clocks32768 = 0,
    Clocks65536 = 1,
    Clocks131072 = 2,
    Clocks262144 = 3,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct UserInterface {
    pub voltage: IoVoltage,
    pub status_clear_delay: StatusClearDelay,
    pub sync_active_low: bool,
    pub sync_timeout: SyncTimeout,
    pub sync_decimated: bool,
    pub sync_interpolated: bool,
    pub data_ready_active_low: bool,
    pub data_ready_enabled: bool,
    pub weak_spi_pulls: bool,
    /// Enables sensor high-speed mode. The caller still configures bus speed.
    pub high_speed: bool,
}

impl UserInterface {
    pub const fn validate(self) -> Result<(), ConfigError> {
        if self.data_ready_enabled && (self.sync_decimated || self.sync_interpolated) {
            Err(ConfigError::ConflictingSyncAndDataReady)
        } else {
            Ok(())
        }
    }

    pub const fn bits(self) -> u16 {
        (self.voltage as u16) << 14
            | (self.status_clear_delay as u16) << 12
            | (self.sync_active_low as u16) << 11
            | (self.sync_timeout as u16) << 9
            | (self.sync_decimated as u16) << 8
            | (self.sync_interpolated as u16) << 7
            | (self.data_ready_active_low as u16) << 6
            | (self.data_ready_enabled as u16) << 5
            | (self.weak_spi_pulls as u16) << 4
            | 0x0c // Keep both supported slew-rate controls enabled.
            | if self.high_speed { 3 } else { 0 }
    }
}

/// Safety diagnostics are enabled by default. Disabling/masking them needs
/// validation by the application. ACC_STS_REQ remains separate from settings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SelfTests {
    pub continuous_rate: [bool; 3],
    pub continuous_acc_normal: [bool; 3],
    pub continuous_acc_shield: [bool; 3],
    pub startup_acc: bool,
}

impl Default for SelfTests {
    fn default() -> Self {
        Self {
            continuous_rate: [true; 3],
            continuous_acc_normal: [true; 3],
            continuous_acc_shield: [true; 3],
            startup_acc: true,
        }
    }
}

impl SelfTests {
    pub const fn bits(self) -> u16 {
        (self.continuous_rate[0] as u16) << 9
            | (self.continuous_rate[1] as u16) << 10
            | (self.continuous_rate[2] as u16) << 11
            | (self.continuous_acc_normal[0] as u16) << 6
            | (self.continuous_acc_normal[1] as u16) << 7
            | (self.continuous_acc_normal[2] as u16) << 8
            | (self.continuous_acc_shield[0] as u16) << 3
            | (self.continuous_acc_shield[1] as u16) << 4
            | (self.continuous_acc_shield[2] as u16) << 5
            | (self.startup_acc as u16) << 2
            | 2 // Reserved bit 1 retains the documented reset value.
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Config {
    pub rate_filters: AxisFilters,
    pub acc_filters: AxisFilters,
    pub aux_acc_filters: AxisFilters,
    pub interpolated_rate_range: RateRange,
    pub decimated_rate_range: RateRange,
    pub interpolated_acc_range: AccRange,
    pub decimated_acc_range: AccRange,
    pub aux_acc_range: AuxAccRange,
    pub rate_decimation: AxisDecimation,
    pub acc_decimation: AxisDecimation,
    pub rate_pre_saturation: AxisSaturation,
    pub rate_post_saturation: AxisSaturation,
    pub acc_pre_saturation: AxisSaturation,
    pub acc_post_saturation: AxisSaturation,
    pub interface: UserInterface,
    pub self_tests: SelfTests,
}

impl Config {
    pub const fn validate(self) -> Result<(), ConfigError> {
        self.interface.validate()
    }

    pub(crate) const fn registers(self) -> [(Register, u16); 12] {
        [
            (Register::FilterRate, self.rate_filters.bits()),
            (Register::FilterAcc12, self.acc_filters.bits()),
            (Register::FilterAcc3, self.aux_acc_filters.bits()),
            (
                Register::ControlRate,
                (self.interpolated_rate_range as u16) << 12
                    | (self.decimated_rate_range as u16) << 9
                    | self.rate_decimation.bits(),
            ),
            (
                Register::ControlAcc12,
                (self.interpolated_acc_range as u16) << 12
                    | (self.decimated_acc_range as u16) << 9
                    | self.acc_decimation.bits(),
            ),
            (Register::ControlAcc3, self.aux_acc_range as u16),
            (Register::RateSaturation1, self.rate_pre_saturation.bits()),
            (Register::RateSaturation2, self.rate_post_saturation.bits()),
            (Register::AccSaturation1, self.acc_pre_saturation.bits()),
            (Register::AccSaturation2, self.acc_post_saturation.bits()),
            (Register::UserInterface, self.interface.bits()),
            (Register::SelfTest, self.self_tests.bits()),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factory_and_existing_decimated_profile_match_datasheet_bits() {
        let mut c = Config::default();
        assert_eq!(c.interface.bits(), 0x200c);
        assert_eq!(c.self_tests.bits(), 0xffe);
        assert_eq!(c.registers()[3].1, 0x1200);
        c.rate_filters = AxisFilters::all(Filter::Lpf3);
        c.rate_decimation = AxisDecimation::all(Decimation::Dec4);
        c.interface.data_ready_enabled = true;
        assert_eq!(c.rate_filters.bits(), 0xdb);
        assert_eq!(c.registers()[3].1, 0x12db);
        assert_eq!(c.interface.bits(), 0x202c);
        assert_eq!(Decimation::Dec4.nominal_output_rate_hz(), 1475.0);
        assert_eq!(AccRange::Dyn4.lsb_per_metre_per_second_squared(), 25600.0);
    }

    #[test]
    fn reserved_saturation_codes_and_conflicting_pins_are_rejected() {
        assert!(SaturationFilter::from_code(31).is_ok());
        assert_eq!(
            SaturationFilter::from_code(32),
            Err(ConfigError::SaturationFilter)
        );
        let interface = UserInterface {
            data_ready_enabled: true,
            sync_interpolated: true,
            ..UserInterface::default()
        };
        assert_eq!(
            interface.validate(),
            Err(ConfigError::ConflictingSyncAndDataReady)
        );
    }
}
