//! NTC thermistor conversion and explicit temperature-sensor adapters.
//!
//! Mini TH1 is read through BQ25622(E) TS, not a direct MCU ADC pin. The PCB
//! network is TS_BIAS -- 5.23 kΩ -- TS -- (TH1 || 30.1 kΩ) -- GND. The BOM
//! gives NCU18XH103F60RB while the schematic value is 103KT1608T-1P; therefore
//! no thermistor curve is selected automatically. Supply the fitted part's
//! characterized curve. A beta fit is approximate; use a manufacturer table
//! or characterized Steinhart-Hart coefficients when accuracy requires it.
//!
//! Charger source: local `data/datasheets/bq25622.pdf`, SLUSEG2D §§8.3.5.4,
//! 8.3.7, 8.6.2.28–29 and 8.6.2.36–37. TS is a bias-relative voltage at
//! 0.0961%/LSB (961 ppm), not an absolute temperature. Existing BQ25622E
//! register access is supplied by `bq25622` and its documented E supplement.
//! ADC registers retain old values. Charger adapters require an explicit
//! one-shot start and observe ADC_EN before accepting completion; they never
//! infer freshness from a nonzero result. They own ADC configuration while
//! measuring, select only their channel and do not restore prior ADC settings.
//! They never change charging enable or TS_IGNORE. No constructor does I/O.

use crate::{
    bq25622,
    interfaces::{AnalogInput, TemperatureSensor},
};
use embedded_hal::i2c::I2c;

const ABSOLUTE_ZERO_CELSIUS: f32 = -273.15;
const TS_CHANNEL: u8 = 0x04;
const DIE_CHANNEL: u8 = 0x02;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConversionError {
    InvalidConfiguration,
    InvalidRatio,
    ShortCircuit,
    OpenCircuit,
    OutOfRange,
    InvalidCurve,
}

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    Transport(E),
    Conversion(ConversionError),
    AdcBusy,
    NotStarted,
    Pending,
    AdcUnavailable,
    TsIgnored,
    ChannelDisabled,
    AdcReconfigured,
    BiasFault,
}

/// Upper fixed resistor with an NTC to ground; the optional fixed resistor is
/// in parallel with the NTC. Values are ohms. Fault classification is electrical:
/// readings close to an open circuit can also mean a very cold/out-of-range NTC.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Divider {
    pullup_ohms: f32,
    parallel_ohms: Option<f32>,
}
impl Divider {
    pub fn new(pullup_ohms: f32, parallel_ohms: Option<f32>) -> Result<Self, ConversionError> {
        if !positive(pullup_ohms) || parallel_ohms.is_some_and(|r| !positive(r)) {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self {
            pullup_ohms,
            parallel_ohms,
        })
    }
    pub const fn v2_mini() -> Self {
        Self {
            pullup_ohms: crate::board::power::TS_PULLUP_OHMS as f32,
            parallel_ohms: Some(crate::board::power::TS_PARALLEL_OHMS as f32),
        }
    }
    /// Voltage at TS divided by its active bias reference, 0 < ratio < 1.
    /// Detects exact rail/open limits; finite values outside the curve's
    /// calibrated resistance range are rejected by the curve separately.
    pub fn resistance_ohms(self, ratio: f32) -> Result<f32, ConversionError> {
        if !ratio.is_finite() || !(0.0..=1.0).contains(&ratio) {
            return Err(ConversionError::InvalidRatio);
        }
        if ratio == 0.0 {
            return Err(ConversionError::ShortCircuit);
        }
        if ratio == 1.0 {
            return Err(ConversionError::OpenCircuit);
        }
        let equivalent = self.pullup_ohms * ratio / (1.0 - ratio);
        let resistance = if let Some(parallel) = self.parallel_ohms {
            if ratio >= parallel / (self.pullup_ohms + parallel) {
                return Err(ConversionError::OpenCircuit);
            }
            equivalent * (parallel / (parallel - equivalent))
        } else {
            equivalent
        };
        if !positive(resistance) {
            return Err(ConversionError::OutOfRange);
        }
        Ok(resistance)
    }
    pub fn resistance_from_ratio_ppm(self, ratio_ppm: u32) -> Result<f32, ConversionError> {
        self.resistance_ohms(ratio_ppm as f32 / 1_000_000.0)
    }
    pub fn temperature_from_ratio<C: NtcCurve>(
        self,
        curve: &C,
        ratio: f32,
    ) -> Result<f32, ConversionError> {
        checked_temperature(curve.temperature_celsius(self.resistance_ohms(ratio)?)?)
    }
    pub fn temperature_from_ratio_ppm<C: NtcCurve>(
        self,
        curve: &C,
        ppm: u32,
    ) -> Result<f32, ConversionError> {
        self.temperature_from_ratio(curve, ppm as f32 / 1_000_000.0)
    }
}

/// Characterized temperature conversion. Implementations must reject resistance
/// outside their calibration domain; adapters also reject nonfinite or
/// physically impossible returned temperatures.
pub trait NtcCurve {
    fn temperature_celsius(&self, resistance_ohms: f32) -> Result<f32, ConversionError>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TemperatureRange {
    min_celsius: f32,
    max_celsius: f32,
}
impl TemperatureRange {
    pub fn new(min_celsius: f32, max_celsius: f32) -> Result<Self, ConversionError> {
        checked_temperature(min_celsius).map_err(|_| ConversionError::InvalidConfiguration)?;
        checked_temperature(max_celsius).map_err(|_| ConversionError::InvalidConfiguration)?;
        if min_celsius >= max_celsius {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self {
            min_celsius,
            max_celsius,
        })
    }
    fn check(self, value: f32) -> Result<f32, ConversionError> {
        let value = checked_temperature(value)?;
        if value < self.min_celsius || value > self.max_celsius {
            return Err(ConversionError::OutOfRange);
        }
        Ok(value)
    }
}

/// 1/T = 1/Tref + ln(R/Rref)/beta, temperatures internally in kelvins.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BetaCurve {
    reference_ohms: f32,
    reference_kelvins: f32,
    beta_kelvins: f32,
    range: TemperatureRange,
}
impl BetaCurve {
    /// Explicit approximate preset for the BOM MPN, not the conflicting
    /// schematic value. Murata's [primary part list](https://www.murata.com/-/media/webrenewal/tool/library/common-pdf/static-model/component-list-ntc-2508.ashx?cvid=20250930011345000000&la=en-sg)
    /// specifies R25=10 kΩ ±1%, B25/50=3380 K ±1% (B25/85 reference 3434 K).
    /// The changing beta shows why this is not a calibrated full-range curve.
    /// Caller selects the accepted range and confirms the fitted part and
    /// self-heating; no resistance/temperature tolerance correction is applied.
    pub const fn murata_ncu18xh103f60rb(range: TemperatureRange) -> Self {
        Self {
            reference_ohms: 10_000.0,
            reference_kelvins: 298.15,
            beta_kelvins: 3380.0,
            range,
        }
    }

    pub fn new(
        reference_ohms: f32,
        reference_celsius: f32,
        beta_kelvins: f32,
        range: TemperatureRange,
    ) -> Result<Self, ConversionError> {
        if !positive(reference_ohms)
            || !positive(beta_kelvins)
            || checked_temperature(reference_celsius).is_err()
        {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self {
            reference_ohms,
            reference_kelvins: reference_celsius - ABSOLUTE_ZERO_CELSIUS,
            beta_kelvins,
            range,
        })
    }
}
impl NtcCurve for BetaCurve {
    fn temperature_celsius(&self, resistance_ohms: f32) -> Result<f32, ConversionError> {
        if !positive(resistance_ohms) {
            return Err(ConversionError::OutOfRange);
        }
        let inverse = 1.0 / self.reference_kelvins
            + libm::logf(resistance_ohms / self.reference_ohms) / self.beta_kelvins;
        if !positive(inverse) {
            return Err(ConversionError::InvalidCurve);
        }
        self.range.check(1.0 / inverse + ABSOLUTE_ZERO_CELSIUS)
    }
}

/// 1/T = A + B*ln(R) + C*ln(R)^3, R in ohms and T in kelvins.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteinhartHartCurve {
    a: f32,
    b: f32,
    c: f32,
    min_ohms: f32,
    max_ohms: f32,
    range: TemperatureRange,
}
impl SteinhartHartCurve {
    pub fn new(
        coefficients: [f32; 3],
        resistance_range_ohms: [f32; 2],
        range: TemperatureRange,
    ) -> Result<Self, ConversionError> {
        let [a, b, c] = coefficients;
        let [min_ohms, max_ohms] = resistance_range_ohms;
        if !coefficients.iter().all(|v| v.is_finite())
            || !positive(min_ohms)
            || !positive(max_ohms)
            || min_ohms >= max_ohms
        {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self {
            a,
            b,
            c,
            min_ohms,
            max_ohms,
            range,
        })
    }
}
impl NtcCurve for SteinhartHartCurve {
    fn temperature_celsius(&self, resistance_ohms: f32) -> Result<f32, ConversionError> {
        if !resistance_ohms.is_finite()
            || resistance_ohms < self.min_ohms
            || resistance_ohms > self.max_ohms
        {
            return Err(ConversionError::OutOfRange);
        }
        let log = libm::logf(resistance_ohms);
        let inverse = self.a + self.b * log + self.c * log * log * log;
        if !positive(inverse) {
            return Err(ConversionError::InvalidCurve);
        }
        self.range.check(1.0 / inverse + ABSOLUTE_ZERO_CELSIUS)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub resistance_ohms: f32,
    pub celsius: f32,
}

/// Borrowed NTC table ordered by increasing temperature / decreasing resistance.
/// Interpolation is linear in ln(resistance); no extrapolation is performed.
#[derive(Clone, Copy, Debug)]
pub struct TableCurve<'a> {
    points: &'a [Point],
}
impl<'a> TableCurve<'a> {
    pub fn new(points: &'a [Point]) -> Result<Self, ConversionError> {
        if points.len() < 2
            || points
                .iter()
                .any(|p| !positive(p.resistance_ohms) || checked_temperature(p.celsius).is_err())
            || points.windows(2).any(|p| {
                p[0].resistance_ohms <= p[1].resistance_ohms || p[0].celsius >= p[1].celsius
            })
        {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self { points })
    }
}
impl NtcCurve for TableCurve<'_> {
    fn temperature_celsius(&self, resistance: f32) -> Result<f32, ConversionError> {
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        if !resistance.is_finite()
            || resistance > first.resistance_ohms
            || resistance < last.resistance_ohms
        {
            return Err(ConversionError::OutOfRange);
        }
        for p in self.points {
            if resistance == p.resistance_ohms {
                return Ok(p.celsius);
            }
        }
        for pair in self.points.windows(2) {
            let [cold, hot] = [pair[0], pair[1]];
            if resistance <= cold.resistance_ohms && resistance >= hot.resistance_ohms {
                // f64 ratios also cover the entire positive finite f32 input
                // domain without overflowing or underflowing to zero.
                let fraction = libm::log(f64::from(resistance) / f64::from(cold.resistance_ohms))
                    / libm::log(f64::from(hot.resistance_ohms) / f64::from(cold.resistance_ohms));
                return checked_temperature(
                    (f64::from(cold.celsius) + fraction * f64::from(hot.celsius - cold.celsius))
                        as f32,
                );
            }
        }
        Err(ConversionError::OutOfRange)
    }
}

/// A calibrated MCU ADC channel measuring an independently biased divider.
/// Bias voltage must be measured/known in microvolts; raw ADC counts are not
/// voltages. The adapter's `AnalogInput` controls sampling and conversion errors.
pub struct AdcThermistor<A, C> {
    input: A,
    divider: Divider,
    curve: C,
    bias_microvolts: u32,
}
impl<A, C> AdcThermistor<A, C> {
    pub fn new(
        input: A,
        divider: Divider,
        curve: C,
        bias_microvolts: u32,
    ) -> Result<Self, ConversionError> {
        if bias_microvolts == 0 {
            return Err(ConversionError::InvalidConfiguration);
        }
        Ok(Self {
            input,
            divider,
            curve,
            bias_microvolts,
        })
    }
    pub fn release(self) -> (A, C) {
        (self.input, self.curve)
    }
}
impl<A: AnalogInput, C: NtcCurve> TemperatureSensor for AdcThermistor<A, C> {
    type Error = Error<A::Error>;
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error> {
        let voltage = self.input.read_microvolts().map_err(Error::Transport)?;
        self.divider
            .temperature_from_ratio(&self.curve, voltage as f32 / self.bias_microvolts as f32)
            .map_err(Error::Conversion)
    }
}

/// Owns the charger and its ADC until `release`. Start each explicit one-shot,
/// poll `try_temperature_celsius` or the common trait, then start again for the
/// next sample. `charger_mut` permits maintenance; changing ADC settings there
/// can invalidate a pending conversion and never establishes a fresh sample.
pub struct Bq25622Thermistor<B, C> {
    charger: bq25622::Bq25622<B>,
    divider: Divider,
    curve: C,
    pending: bool,
}
impl<B, C> Bq25622Thermistor<B, C> {
    pub const fn new(charger: bq25622::Bq25622<B>, divider: Divider, curve: C) -> Self {
        Self {
            charger,
            divider,
            curve,
            pending: false,
        }
    }
    pub const fn v2_mini(charger: bq25622::Bq25622<B>, curve: C) -> Self {
        Self::new(charger, Divider::v2_mini(), curve)
    }
    pub fn release(self) -> (bq25622::Bq25622<B>, C) {
        (self.charger, self.curve)
    }
    pub fn charger_mut(&mut self) -> &mut bq25622::Bq25622<B> {
        self.pending = false;
        &mut self.charger
    }
}
impl<B: I2c, C: NtcCurve> Bq25622Thermistor<B, C> {
    pub fn start_one_shot(
        &mut self,
        resolution: bq25622::AdcResolution,
    ) -> Result<(), Error<bq25622::Error<B::Error>>> {
        if self.pending {
            return Err(Error::AdcBusy);
        }
        start_adc(&mut self.charger, TS_CHANNEL, resolution)?;
        self.pending = true;
        Ok(())
    }
    pub fn try_temperature_celsius(
        &mut self,
    ) -> Result<Option<f32>, Error<bq25622::Error<B::Error>>> {
        if !self.pending {
            return Err(Error::NotStarted);
        }
        if !adc_completed(&mut self.charger, TS_CHANNEL)? {
            return Ok(None);
        }
        let raw = self
            .charger
            .read_word(bq25622::WordRegister::ThermistorAdc)
            .map_err(Error::Transport)?;
        if raw > 0x03ff {
            self.pending = false;
            return Err(Error::Conversion(ConversionError::InvalidRatio));
        }
        self.pending = false;
        self.divider
            .temperature_from_ratio_ppm(&self.curve, u32::from(raw) * 961)
            .map(Some)
            .map_err(Error::Conversion)
    }
}
impl<B: I2c, C: NtcCurve> TemperatureSensor for Bq25622Thermistor<B, C> {
    type Error = Error<bq25622::Error<B::Error>>;
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error> {
        self.try_temperature_celsius()?.ok_or(Error::Pending)
    }
}

/// Charger junction temperature is distinct from the external NTC/battery
/// temperature. Uses the same explicit one-shot start/completion contract.
pub struct Bq25622DieTemperature<B> {
    charger: bq25622::Bq25622<B>,
    pending: bool,
}
impl<B> Bq25622DieTemperature<B> {
    pub const fn new(charger: bq25622::Bq25622<B>) -> Self {
        Self {
            charger,
            pending: false,
        }
    }
    pub fn release(self) -> bq25622::Bq25622<B> {
        self.charger
    }
    pub fn charger_mut(&mut self) -> &mut bq25622::Bq25622<B> {
        self.pending = false;
        &mut self.charger
    }
}
impl<B: I2c> Bq25622DieTemperature<B> {
    pub fn start_one_shot(
        &mut self,
        resolution: bq25622::AdcResolution,
    ) -> Result<(), Error<bq25622::Error<B::Error>>> {
        if self.pending {
            return Err(Error::AdcBusy);
        }
        start_adc(&mut self.charger, DIE_CHANNEL, resolution)?;
        self.pending = true;
        Ok(())
    }
    pub fn try_temperature_celsius(
        &mut self,
    ) -> Result<Option<f32>, Error<bq25622::Error<B::Error>>> {
        if !self.pending {
            return Err(Error::NotStarted);
        }
        if !adc_completed(&mut self.charger, DIE_CHANNEL)? {
            return Ok(None);
        }
        let value = self
            .charger
            .die_temperature_millicelsius()
            .map_err(Error::Transport)?;
        self.pending = false;
        // The converter clamps to -40..140 °C; other decoded values indicate
        // an invalid register result, not a legitimate junction measurement.
        if !(-40_000..=140_000).contains(&value) {
            return Err(Error::Conversion(ConversionError::OutOfRange));
        }
        checked_temperature(value as f32 / 1000.0)
            .map(Some)
            .map_err(Error::Conversion)
    }
}
impl<B: I2c> TemperatureSensor for Bq25622DieTemperature<B> {
    type Error = Error<bq25622::Error<B::Error>>;
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error> {
        self.try_temperature_celsius()?.ok_or(Error::Pending)
    }
}

fn start_adc<B: I2c>(
    charger: &mut bq25622::Bq25622<B>,
    channel: u8,
    resolution: bq25622::AdcResolution,
) -> Result<(), Error<bq25622::Error<B::Error>>> {
    use bq25622::ByteRegister as R;
    if channel == TS_CHANNEL
        && charger
            .read_byte(R::NtcControl0)
            .map_err(Error::Transport)?
            & 0x80
            != 0
    {
        return Err(Error::TsIgnored);
    }
    if charger.read_byte(R::AdcControl).map_err(Error::Transport)? & 0x80 != 0 {
        return Err(Error::AdcBusy);
    }
    if charger
        .read_byte(R::ChargerControl1)
        .map_err(Error::Transport)?
        & 0x10
        != 0
    {
        return Err(Error::AdcUnavailable);
    }
    charger
        .configure_adc(bq25622::AdcConfig {
            enabled: true,
            one_shot: true,
            resolution,
            running_average: false,
            initialize_average: true,
            disabled_channels: !channel,
        })
        .map_err(Error::Transport)?;
    let readback = charger.read_byte(R::AdcControl).map_err(Error::Transport)?;
    // Observing enable is necessary: an old DONE/old result must not make a
    // rejected start (e.g. absent supply) appear successful. A conversion that
    // finished before this read is conservatively rejected; restart explicitly.
    if readback & 0xc0 != 0xc0 {
        return Err(Error::AdcUnavailable);
    }
    Ok(())
}
fn adc_completed<B: I2c>(
    charger: &mut bq25622::Bq25622<B>,
    channel: u8,
) -> Result<bool, Error<bq25622::Error<B::Error>>> {
    use bq25622::ByteRegister as R;
    if channel == TS_CHANNEL
        && charger
            .read_byte(R::NtcControl0)
            .map_err(Error::Transport)?
            & 0x80
            != 0
    {
        return Err(Error::TsIgnored);
    }
    if charger.read_byte(R::AdcDisable).map_err(Error::Transport)? & channel != 0 {
        return Err(Error::ChannelDisabled);
    }
    let control = charger.read_byte(R::AdcControl).map_err(Error::Transport)?;
    if control & 0x40 == 0 {
        return Err(Error::AdcReconfigured);
    }
    if control & 0x80 != 0 {
        return Ok(false);
    }
    let status = charger.status().map_err(Error::Transport)?;
    if !status.adc_done() {
        return Err(Error::AdcUnavailable);
    }
    if channel == TS_CHANNEL && status.temperature_zone() == bq25622::TemperatureZone::BiasFault {
        return Err(Error::BiasFault);
    }
    Ok(true)
}
fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}
fn checked_temperature(value: f32) -> Result<f32, ConversionError> {
    if !value.is_finite() || value <= ABSOLUTE_ZERO_CELSIUS {
        return Err(ConversionError::OutOfRange);
    }
    Ok(value)
}

#[cfg(test)]
mod tests;
