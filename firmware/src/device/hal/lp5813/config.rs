//! Typed configuration and animation encoders; no hardware access.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InvalidConfig;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MaximumCurrent {
    Milliamp25_5,
    Milliamp51,
}
impl MaximumCurrent {
    pub const fn step_microamps(self) -> u32 {
        match self {
            Self::Milliamp25_5 => 100,
            Self::Milliamp51 => 200,
        }
    }
    pub const fn peak_microamps(self, dot_current: u8) -> u32 {
        self.step_microamps() * dot_current as u32
    }
    /// Exact programmable peak current. TCM average current is lower according
    /// to scan count, blanking and PWM duty; this is not LED average current.
    pub fn dot_current(self, peak_microamps: u32) -> Result<u8, InvalidConfig> {
        let step = self.step_microamps();
        if peak_microamps > self.peak_microamps(255) || !peak_microamps.is_multiple_of(step) {
            return Err(InvalidConfig);
        }
        Ok((peak_microamps / step) as u8)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DriveMode {
    Direct,
    Scan { count: u8 },
    Mixed { count: u8, direct_outputs: u8 },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PwmFrequency {
    Khz24 = 0,
    Khz12 = 1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Phase {
    Forward = 0,
    Middle = 2,
    Backward = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BlankTime {
    Us1 = 0,
    Us1_3 = 1,
    Us1_7 = 2,
    Us2 = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ShortThreshold {
    Percent35 = 0,
    Percent45 = 1,
    Percent55 = 2,
    Percent65 = 3,
}

/// All 13 CONFIG registers. Output enable, current and PWM are independent.
/// `phase` and bitmasks use `Channel` ordering. A config error never emits an
/// I2C transaction. Raw register writes can access alternative documented forms.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub boost_millivolts: u16,
    pub maximum_current: MaximumCurrent,
    pub drive_mode: DriveMode,
    pub pwm_frequency: PwmFrequency,
    /// Output number 0..3 for each scan slot; active slots must be unique.
    pub scan_order: [u8; 4],
    pub autonomous_channels: u16,
    pub exponential_channels: u16,
    pub phase: [Phase; 16],
    pub blank_time: BlankTime,
    /// V2 Mini ties SYNC to GND; keep false on that board.
    pub sync_output: bool,
    /// 0..3 encodes VOUT minus 1.1 / 1.3 / 1.5 / 1.7 volts.
    pub clamp_voltage_code: u8,
    pub clamp_during_pwm_off: bool,
    pub disable_clamp: bool,
    pub disable_open_channel: bool,
    pub shutdown_on_short: bool,
    pub short_threshold: ShortThreshold,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            boost_millivolts: 3000,
            maximum_current: MaximumCurrent::Milliamp25_5,
            drive_mode: DriveMode::Direct,
            pwm_frequency: PwmFrequency::Khz24,
            scan_order: [0, 1, 2, 3],
            autonomous_channels: 0,
            exponential_channels: 0,
            phase: [Phase::Forward; 16],
            blank_time: BlankTime::Us1,
            sync_output: false,
            clamp_voltage_code: 0,
            clamp_during_pwm_off: false,
            disable_clamp: false,
            disable_open_channel: true,
            shutdown_on_short: false,
            short_threshold: ShortThreshold::Percent35,
        }
    }
}
impl Config {
    /// Four-scan matrix setup for the fitted LEDs, without enabling LEDs.
    /// Keeps peak current below the ROHM MSL0402RGBU1 30 mA DC limit even at
    /// DC=255 (25.5 mA). Source: local ROHM Rev.010 absolute ratings.
    pub fn v2_mini() -> Self {
        Self {
            boost_millivolts: 4500,
            drive_mode: DriveMode::Scan { count: 4 },
            short_threshold: ShortThreshold::Percent65,
            ..Self::default()
        }
    }
    pub fn encode(&self) -> Result<[u8; 13], InvalidConfig> {
        if !(3000..=5500).contains(&self.boost_millivolts)
            || !(self.boost_millivolts - 3000).is_multiple_of(100)
            || self.clamp_voltage_code > 3
            || self.scan_order.iter().any(|&v| v > 3)
        {
            return Err(InvalidConfig);
        }
        let (mode, scans, direct) = match self.drive_mode {
            DriveMode::Direct => (0, 0, 0),
            DriveMode::Scan { count } if (1..=4).contains(&count) => (count, count, 0),
            DriveMode::Mixed {
                count,
                direct_outputs,
            } if (1..=3).contains(&count) && direct_outputs != 0 && direct_outputs < 16 => {
                (4 + count, count, direct_outputs)
            }
            _ => return Err(InvalidConfig),
        };
        let mut selected = 0;
        for &output in &self.scan_order[..usize::from(scans)] {
            let bit = 1 << output;
            if selected & bit != 0 || direct & bit != 0 {
                return Err(InvalidConfig);
            }
            selected |= bit;
        }
        let mut bytes = [0; 13];
        bytes[0] = (((self.boost_millivolts - 3000) / 100) as u8 * 2)
            | u8::from(self.maximum_current == MaximumCurrent::Milliamp51);
        bytes[1] = (self.pwm_frequency as u8) << 7 | mode << 4 | direct;
        for (index, &output) in self.scan_order.iter().enumerate() {
            bytes[2] |= output << (index * 2);
        }
        bytes[3..5].copy_from_slice(&self.autonomous_channels.to_le_bytes());
        bytes[5..7].copy_from_slice(&self.exponential_channels.to_le_bytes());
        for (index, phase) in self.phase.iter().enumerate() {
            bytes[7 + index / 4] |= (*phase as u8) << (index % 4 * 2);
        }
        bytes[11] = (u8::from(self.sync_output) << 2) | self.blank_time as u8;
        bytes[12] = self.clamp_voltage_code << 6
            | u8::from(self.clamp_during_pwm_off) << 5
            | u8::from(self.disable_clamp) << 4
            | u8::from(self.disable_open_channel) << 3
            | u8::from(self.shutdown_on_short) << 2
            | self.short_threshold as u8;
        Ok(bytes)
    }
}

/// Typical duration; oscillator tolerances apply. Codes follow datasheet Table 7-4.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AnimationTime {
    Zero,
    Ms90,
    Ms180,
    Ms360,
    Ms540,
    Ms800,
    Ms1070,
    Ms1520,
    Ms2060,
    Ms2500,
    Ms3040,
    Ms4020,
    Ms5010,
    Ms5990,
    Ms7060,
    Ms8050,
}
impl AnimationTime {
    pub const fn milliseconds(self) -> u16 {
        [
            0, 90, 180, 360, 540, 800, 1070, 1520, 2060, 2500, 3040, 4020, 5010, 5990, 7060, 8050,
        ][self as usize]
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum UnitPlayback {
    Skip,
    Once,
    Twice,
    Infinite,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AnimationUnit {
    pub pwm: [u8; 5],
    pub slopes: [AnimationTime; 4],
    pub playback: UnitPlayback,
}
impl Default for AnimationUnit {
    fn default() -> Self {
        Self {
            pwm: [0; 5],
            slopes: [AnimationTime::Zero; 4],
            playback: UnitPlayback::Skip,
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Animation {
    pub pause_start: AnimationTime,
    pub pause_end: AnimationTime,
    /// 1..3 active units. Unused units are still written deterministically.
    pub active_units: u8,
    /// 0=skip, 1..14=that many times, 15=infinite (SNVU859 section 2.11.2).
    pub playback: u8,
    pub units: [AnimationUnit; 3],
}
impl Default for Animation {
    fn default() -> Self {
        Self {
            pause_start: AnimationTime::Zero,
            pause_end: AnimationTime::Zero,
            active_units: 1,
            playback: 0,
            units: [AnimationUnit::default(); 3],
        }
    }
}
impl Animation {
    pub fn encode(&self) -> Result<[u8; 26], InvalidConfig> {
        if !(1..=3).contains(&self.active_units) || self.playback > 15 {
            return Err(InvalidConfig);
        }
        let mut bytes = [0; 26];
        bytes[0] = (self.pause_start as u8) << 4 | self.pause_end as u8;
        bytes[1] = (self.active_units - 1) << 4 | self.playback;
        for (index, unit) in self.units.iter().enumerate() {
            let start = 2 + index * 8;
            bytes[start..start + 5].copy_from_slice(&unit.pwm);
            bytes[start + 5] = (unit.slopes[1] as u8) << 4 | unit.slopes[0] as u8;
            bytes[start + 6] = (unit.slopes[3] as u8) << 4 | unit.slopes[2] as u8;
            bytes[start + 7] = unit.playback as u8;
        }
        Ok(bytes)
    }
}
