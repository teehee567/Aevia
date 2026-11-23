//! BQ25628E/BQ25622E/MAX17048 battery monitoring.
//!
//! Register references:
//! https://www.ti.com/lit/gpn/bq25628e (Rev. C, status at 0x1d..0x1f)
//! https://www.ti.com/lit/gpn/bq25622e (Rev. C, same status fields only)
//! https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf
//! Normal monitoring is read-only. Requested power measurements temporarily
//! enable the ADC. Blue means external power without confirmed active charging; the
//! autonomous power path can still supplement a weak input from the battery.

pub use crate::display::BatteryState as State;
use crate::display::{BatteryTelemetry, ChargeStage, PowerSource, TemperatureZone};

pub const LOW_PERCENT: u16 = 15;
pub const LOW_CLEAR_PERCENT: u16 = 20;
const STALE_MS: u64 = 3_000;
const GAUGE_SETTLE_MS: u64 = 1_000;

pub trait Registers {
    fn read<const N: usize>(
        &mut self,
        address: u8,
        register: u8,
    ) -> impl core::future::Future<Output = Result<[u8; N], &'static str>>;
}

/// Programmed limits before a diagnostic ADC request (TI Rev. C tables 8-7,
/// 8-8, 8-13, 8-16 and 8-23). Temperature and input limits can reduce them.
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct ChargerConfiguration {
    pub charge_current_limit_ma: u16,
    pub termination_current_ma: u16,
    pub termination_enabled: bool,
    pub regulation_voltage_mv: u16,
    pub watchdog_seconds: u16,
    pub mode: &'static str,
}

impl ChargerConfiguration {
    pub fn adc_unavailable_reason(
        self,
        external: bool,
        explicit_sample: bool,
    ) -> Option<&'static str> {
        if !external {
            Some("external-power-unavailable")
        } else if self.watchdog_seconds != 0 && !explicit_sample {
            // Any register write enters host mode. A pending watchdog expiry
            // halves ICHG even if the ADC control byte has already been restored.
            Some("watchdog-enabled")
        } else {
            None
        }
    }
}

pub async fn read_charger_configuration(
    bus: &mut impl Registers,
    address: u8,
    part: u8,
) -> Result<ChargerConfiguration, &'static str> {
    let (current_shift, current_step, term_shift, term_step) = match (address, (part >> 3) & 7) {
        (0x6a, 4) => (5, 40, 2, 5),  // BQ25628E
        (0x6b, 3) => (6, 80, 3, 10), // BQ25622E
        _ => return Err("unexpected-part"),
    };
    let limits = bus.read::<4>(address, 0x02).await?;
    let control = bus.read::<5>(address, 0x12).await?;
    let status = bus.read::<1>(address, 0x1d).await?[0];
    let ichg = u16::from_le_bytes([limits[0], limits[1]]);
    let vreg = u16::from_le_bytes([limits[2], limits[3]]);
    let iterm = u16::from_le_bytes([control[0], control[1]]);
    Ok(ChargerConfiguration {
        charge_current_limit_ma: ((ichg >> current_shift) & 0x3f) * current_step,
        termination_current_ma: ((iterm >> term_shift) & 0x3f) * term_step,
        termination_enabled: control[2] & 4 != 0,
        regulation_voltage_mv: ((vreg >> 3) & 0x1ff) * 10,
        watchdog_seconds: [0, 50, 100, 200][usize::from(control[4] & 3)],
        mode: if status & 1 != 0 { "default" } else { "host" },
    })
}

pub async fn read_charger(
    bus: &mut impl Registers,
    detected: &mut Option<u8>,
) -> Result<(u8, [u8; 3]), &'static str> {
    let result = async {
        let (address, part) = match *detected {
            Some(address) => (address, bus.read::<1>(address, 0x38).await?[0]),
            None => match bus.read::<1>(0x6a, 0x38).await {
                Ok(part) => (0x6a, part[0]),
                Err("nack") => (0x6b, bus.read::<1>(0x6b, 0x38).await?[0]),
                Err(error) => return Err(error),
            },
        };
        let expected_pn = if address == 0x6a { 4 } else { 3 };
        if part >> 3 & 7 != expected_pn {
            return Err("unexpected-part");
        }
        *detected = Some(address);
        // Rev. C tables 8-23..8-25 in BOTH datasheets confirm these status
        // fields match. Other settings/ADC scales are not interchangeable.
        // Stop before read-to-clear flags at 0x20.
        Ok((part, bus.read(address, 0x1d).await?))
    }
    .await;
    if result.is_err() {
        *detected = None;
    }
    result
}

/// BQ25622E and BQ25628E Rev. C use the same ADC formats for these four fields.
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct PowerAdc {
    pub input_mv: u32,
    pub input_ma: i16,
    pub battery_mv: u32,
    pub battery_ma: Option<i16>,
}

impl PowerAdc {
    pub fn decode(input: [u8; 6], battery_voltage: [u8; 2]) -> Result<Self, &'static str> {
        let vbus = u32::from((u16::from_le_bytes([input[4], input[5]]) >> 2) & 0x1fff);
        let input_mv = (vbus * 397 + 50) / 100;
        let input_ma = i16::from_le_bytes([input[0], input[1]]) & !1;
        let battery_current = i16::from_le_bytes([input[2], input[3]]) & !3;
        let vbat = u32::from((u16::from_le_bytes(battery_voltage) >> 1) & 0x0fff);
        if !(3900..=18010).contains(&input_mv) || !(0..=4000).contains(&input_ma) {
            return Err("invalid-input-adc");
        }
        Ok(Self {
            input_mv,
            input_ma,
            battery_mv: (vbat * 199 + 50) / 100,
            // 0x8000 indicates a change of current direction during conversion.
            battery_ma: (battery_current != i16::MIN).then_some(battery_current),
        })
    }
}

pub async fn read_gauge(bus: &mut impl Registers) -> Result<Gauge, &'static str> {
    let version = u16::from_be_bytes(bus.read(0x36, 0x08).await?);
    if version & 0xfff0 != 0x0010 {
        return Err("unexpected-version");
    }
    // MAX17048 word reads must explicitly address each register. A longer
    // read repeats VCELL on the connected gauge instead of advancing to SOC.
    let voltage: [u8; 2] = bus.read(0x36, 0x02).await?;
    let soc: [u8; 2] = bus.read(0x36, 0x04).await?;
    let mut gauge =
        decode_gauge([voltage[0], voltage[1], soc[0], soc[1]]).ok_or("invalid-reading")?;
    // A missing rate must not erase an independently valid SOC or voltage.
    gauge.rate = bus.read(0x36, 0x16).await.ok().map(i16::from_be_bytes);
    Ok(gauge)
}

impl State {
    pub fn label(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Charging => "charging",
            Self::Bypass => "bypass",
            Self::Battery => "battery",
            Self::Low => "low",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Gauge {
    rate: Option<i16>, // Signed MAX17048 CRATE, 0.208% of capacity per hour per LSB.
    millivolts: u16,
    soc: u16, // 1/256 percent, never rounded before threshold comparisons.
}

fn decode_gauge(bytes: [u8; 4]) -> Option<Gauge> {
    let millivolts = ((u32::from(u16::from_be_bytes([bytes[0], bytes[1]])) * 5 + 32) / 64) as u16;
    let soc = u16::from_be_bytes([bytes[2], bytes[3]]);
    // Voltage and SOC validity are independent: an uncalibrated SOC estimate
    // must not hide a usable voltage or the charger's active charge phase.
    if !(2_500..=4_500).contains(&millivolts) {
        return None;
    }
    Some(Gauge {
        millivolts,
        soc,
        rate: None,
    })
}

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub power_source: Option<PowerSource>,
    pub charge_stage: Option<ChargeStage>,
    pub rate_percent_per_hour: Option<f32>,
    pub state: State,
    pub percent: Option<u8>,
    pub millivolts: Option<u16>,
    pub raw_soc: Option<u16>,
    pub charger_error: Option<&'static str>,
    pub gauge_error: Option<&'static str>,
    pub faults: Option<u8>,
    pub charger_status: Option<[u8; 3]>,
    pub part: Option<u8>,
    pub errors: u32,
    sampled_ms: Option<u64>,
}

impl Snapshot {
    pub const fn new() -> Self {
        Self {
            state: State::Unknown,
            power_source: None,
            charge_stage: None,
            rate_percent_per_hour: None,
            percent: None,
            millivolts: None,
            raw_soc: None,
            charger_error: None,
            gauge_error: None,
            faults: None,
            charger_status: None,
            part: None,
            errors: 0,
            sampled_ms: None,
        }
    }

    pub fn temperature_zone(&self) -> Option<TemperatureZone> {
        self.charger_status.map(|status| match status[2] & 7 {
            0 => TemperatureZone::Normal,
            1 => TemperatureZone::ColdOrUnpowered,
            2 => TemperatureZone::Hot,
            3 => TemperatureZone::Cool,
            4 => TemperatureZone::Warm,
            5 => TemperatureZone::Precool,
            6 => TemperatureZone::Prewarm,
            _ => TemperatureZone::BiasFault,
        })
    }

    pub fn telemetry(self) -> BatteryTelemetry {
        BatteryTelemetry {
            status: self.state,
            percent: self.percent,
            millivolts: self.millivolts,
            rate_percent_per_hour: self.rate_percent_per_hour,
            charge_stage: self.charge_stage,
            power_source: self.power_source,
            temperature_zone: self.temperature_zone(),
        }
    }

    pub fn fresh(mut self, now_ms: u64) -> Self {
        if self
            .sampled_ms
            .is_none_or(|ms| now_ms.saturating_sub(ms) >= STALE_MS)
        {
            self.state = State::Unknown;
            self.rate_percent_per_hour = None;
            self.power_source = None;
            self.charge_stage = None;
            self.percent = None;
            self.millivolts = None;
            self.raw_soc = None;
            self.charger_status = None;
            self.faults = None;
            self.charger_error = Some("stale");
            self.gauge_error = Some("stale");
        }
        self
    }
}

pub struct Monitor {
    snapshot: Snapshot,
    gauge_since: Option<u64>,
    previous_voltage: Option<u16>,
    low: bool,
    candidate: State,
    candidate_count: u8,
}

impl Monitor {
    pub const fn new() -> Self {
        Self {
            snapshot: Snapshot::new(),
            gauge_since: None,
            previous_voltage: None,
            low: false,
            candidate: State::Unknown,
            candidate_count: 0,
        }
    }

    pub fn update(
        &mut self,
        now_ms: u64,
        charger: Result<(u8, [u8; 3]), &'static str>,
        gauge: Result<Gauge, &'static str>,
        power_good: bool,
    ) -> Snapshot {
        if self
            .snapshot
            .sampled_ms
            .is_some_and(|ms| now_ms.saturating_sub(ms) >= STALE_MS)
        {
            self.gauge_since = None;
            self.previous_voltage = None;
            self.candidate_count = 0;
        }
        let mut next = Snapshot::new();
        next.sampled_ms = Some(now_ms);
        next.errors = self.snapshot.errors;
        let gauge = match gauge {
            Ok(gauge) => {
                next.raw_soc = Some(gauge.soc);
                // An empty connector can repeatedly power the gauge from BAT
                // detection pulses. Require a settling interval after any
                // failed read or large voltage step; ACK alone is not presence.
                if self
                    .previous_voltage
                    .is_some_and(|mv| mv.abs_diff(gauge.millivolts) > 200)
                {
                    self.gauge_since = None;
                }
                self.previous_voltage = Some(gauge.millivolts);
                let since = *self.gauge_since.get_or_insert(now_ms);
                if now_ms.saturating_sub(since) >= GAUGE_SETTLE_MS {
                    next.millivolts = Some(gauge.millivolts);
                    if gauge.soc <= 110 * 256 {
                        next.rate_percent_per_hour = gauge.rate.map(|rate| f32::from(rate) * 0.208);
                        next.percent = Some((gauge.soc.min(100 * 256) / 256) as u8);
                        if gauge.soc <= LOW_PERCENT * 256 {
                            self.low = true;
                        }
                        if gauge.soc >= LOW_CLEAR_PERCENT * 256 {
                            self.low = false;
                        }
                    } else {
                        next.gauge_error = Some("soc-out-of-range");
                    }
                    Some(gauge)
                } else {
                    next.gauge_error = Some("settling");
                    None
                }
            }
            Err(error) => {
                self.gauge_since = None;
                self.previous_voltage = None;
                next.gauge_error = Some(error);
                next.errors = next.errors.saturating_add(1);
                None
            }
        };
        let desired = match charger {
            Ok((part, status)) => {
                next.part = Some(part);
                next.charger_status = Some(status);
                next.faults = Some(status[2]);
                let source = status[1] & 7;
                let charging = status[1] & 0x18 != 0;
                match (source, power_good) {
                    (0, false) => {
                        next.power_source = Some(PowerSource::Battery);
                        next.charge_stage = Some(ChargeStage::NotCharging);
                        if gauge.is_none() {
                            State::Unknown
                        } else if self.low {
                            State::Low
                        } else {
                            State::Battery
                        }
                    }
                    (4, true) => {
                        next.power_source = Some(PowerSource::External);
                        let ts = status[2] & 7;
                        if status[2] & 0xe8 != 0 || matches!(ts, 1 | 2 | 7) || status[0] & 2 != 0 {
                            next.charger_error = Some("fault");
                            State::Unknown
                        } else {
                            let stage = match (status[1] >> 3) & 3 {
                                0 => ChargeStage::NotCharging,
                                1 => ChargeStage::ConstantCurrent,
                                2 => ChargeStage::Taper,
                                _ => ChargeStage::TopOff,
                            };
                            if !charging || gauge.is_some() {
                                next.charge_stage = Some(stage);
                            }
                            if charging && gauge.is_some() {
                                State::Charging
                            } else if !charging || next.gauge_error == Some("nack") {
                                State::Bypass
                            } else {
                                State::Unknown
                            }
                        }
                    }
                    _ => {
                        next.charger_error = Some("source-mismatch");
                        State::Unknown
                    }
                }
            }
            Err(error) => {
                next.charger_error = Some(error);
                next.errors = next.errors.saturating_add(1);
                State::Unknown
            }
        };
        // Drop misleading colours immediately on errors/source changes. Two
        // consecutive samples confirm the new colour instead of flickering.
        if desired != self.candidate {
            self.candidate = desired;
            self.candidate_count = 1;
        } else {
            self.candidate_count = self.candidate_count.saturating_add(1);
        }
        if self.candidate_count >= 2 {
            next.state = desired;
        }
        self.snapshot = next;
        next
    }
}

impl Default for Monitor {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for Snapshot {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_futures::block_on;
    use std::vec::Vec;

    struct Bus {
        reads: Vec<(u8, u8, usize)>,
    }

    #[test]
    fn power_diagnostics_decode_both_parts_and_only_sample_on_request_with_watchdog() {
        struct ConfigBus {
            limits: [u8; 4],
            control: [u8; 5],
            status: [u8; 1],
        }
        impl Registers for ConfigBus {
            async fn read<const N: usize>(
                &mut self,
                _: u8,
                register: u8,
            ) -> Result<[u8; N], &'static str> {
                let bytes: &[u8] = match register {
                    0x02 => &self.limits,
                    0x12 => &self.control,
                    0x1d => &self.status,
                    _ => panic!("unexpected charger read"),
                };
                Ok(bytes.try_into().unwrap())
            }
        }
        let mut bus = ConfigBus {
            limits: [0x40, 0x00, 0x20, 0x0d],
            control: [0x30, 0x00, 0x04, 0x0c, 0x21],
            status: [0x01],
        };
        for (code, seconds) in [(0, 0), (1, 50), (2, 100), (3, 200)] {
            bus.control[4] = 0x20 | code;
            for (status, mode) in [(0, "host"), (1, "default")] {
                bus.status[0] = status;
                let config = block_on(read_charger_configuration(&mut bus, 0x6b, 0x1a)).unwrap();
                assert_eq!(config.charge_current_limit_ma, 80);
                assert_eq!(config.regulation_voltage_mv, 4200);
                assert_eq!(config.termination_current_ma, 60);
                assert!(config.termination_enabled);
                assert_eq!(config.mode, mode);
                assert_eq!(config.watchdog_seconds, seconds);
                assert_eq!(
                    config.adc_unavailable_reason(true, false),
                    (seconds != 0).then_some("watchdog-enabled")
                );
                assert_eq!(config.adc_unavailable_reason(true, true), None);
                assert_eq!(
                    config.adc_unavailable_reason(false, true),
                    Some("external-power-unavailable")
                );
            }
        }
        bus.limits[..2].copy_from_slice(&0x0100u16.to_le_bytes());
        bus.control[0] = 0x10;
        bus.control[2] = 0;
        let config = block_on(read_charger_configuration(&mut bus, 0x6a, 0x22)).unwrap();
        assert_eq!(config.charge_current_limit_ma, 320);
        assert_eq!(config.termination_current_ma, 20);
        assert!(!config.termination_enabled);
        assert!(block_on(read_charger_configuration(&mut bus, 0x6b, 0x22)).is_err());
    }

    #[test]
    fn charger_adc_keeps_units_sign_and_invalid_readings_distinct() {
        let input = [0x2c, 0x01, 0x90, 0x01, 0xb0, 0x13];
        let battery = [0x34, 0x0f];
        let power = PowerAdc::decode(input, battery).unwrap();
        assert_eq!(power.input_mv, 5002);
        assert_eq!(power.input_ma, 300);
        assert_eq!(power.battery_mv, 3873);
        assert_eq!(power.battery_ma, Some(400));
        let mut input = input;
        input[2..4].copy_from_slice(&0x8000u16.to_le_bytes());
        assert_eq!(PowerAdc::decode(input, battery).unwrap().battery_ma, None);
        assert!(PowerAdc::decode([0; 6], battery).is_err());
    }

    impl Registers for Bus {
        async fn read<const N: usize>(
            &mut self,
            address: u8,
            register: u8,
        ) -> Result<[u8; N], &'static str> {
            self.reads.push((address, register, N));
            let source: &[u8] = match (address, register, N) {
                (0x6a, 0x38, 1) => return Err("nack"),
                (0x6b, 0x38, 1) => &[0x1a],
                (0x6b, 0x1d, 3) => &[1, 0x14, 0],
                (0x36, 0x08, 2) => &[0, 0x12],
                (0x36, 0x02, 2) => &[0xd1, 0xc0],
                // The physical gauge repeats VCELL in a four-byte read.
                (0x36, 0x02, 4) => &[0xd1, 0xc0, 0xd1, 0xc0],
                (0x36, 0x04, 2) => &[80, 128],
                (0x36, 0x16, 2) => &[0xff, 0x9c],
                _ => panic!("unexpected transaction"),
            };
            let mut bytes = [0; N];
            bytes.copy_from_slice(source);
            Ok(bytes)
        }
    }

    #[test]
    fn real_gauge_requires_individually_addressed_words() {
        let mut bus = Bus { reads: Vec::new() };
        let gauge = block_on(read_gauge(&mut bus)).unwrap();
        assert_eq!(gauge.soc, 80 * 256 + 128);
        assert_eq!(gauge.rate, Some(-100));
        assert_eq!(
            bus.reads,
            [(0x36, 8, 2), (0x36, 2, 2), (0x36, 4, 2), (0x36, 0x16, 2)]
        );
    }

    #[test]
    fn alternate_charger_is_identified_and_reused_without_absent_address_probes() {
        let mut bus = Bus { reads: Vec::new() };
        let mut detected = None;
        assert_eq!(
            block_on(read_charger(&mut bus, &mut detected)),
            Ok((0x1a, [1, 0x14, 0]))
        );
        assert_eq!(detected, Some(0x6b));
        bus.reads.clear();
        assert!(block_on(read_charger(&mut bus, &mut detected)).is_ok());
        assert_eq!(bus.reads, [(0x6b, 0x38, 1), (0x6b, 0x1d, 3)]);
    }

    fn sample(monitor: &mut Monitor, ms: u64, status: u8, soc: u16) -> Snapshot {
        monitor.update(
            ms,
            Ok((0x22, [0, status, 0])),
            Ok(Gauge {
                rate: Some(-100),
                millivolts: 3800,
                soc,
            }),
            status & 7 == 4,
        )
    }

    #[test]
    fn charge_phases_and_power_transitions() {
        let mut monitor = Monitor::new();
        for ms in [0, 500, 1000, 1500] {
            sample(&mut monitor, ms, 0, 50 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Battery);
        assert_eq!(monitor.snapshot.power_source, Some(PowerSource::Battery));
        assert_eq!(
            monitor.snapshot.charge_stage,
            Some(ChargeStage::NotCharging)
        );
        assert!((monitor.snapshot.rate_percent_per_hour.unwrap() + 20.8).abs() < 0.0001);
        for (phase, expected) in [
            (0x0c, ChargeStage::ConstantCurrent),
            (0x14, ChargeStage::Taper),
            (0x1c, ChargeStage::TopOff),
        ] {
            sample(&mut monitor, 2000, phase, 50 * 256);
            assert_eq!(
                sample(&mut monitor, 2500, phase, 50 * 256).state,
                State::Charging
            );
            assert_eq!(monitor.snapshot.power_source, Some(PowerSource::External));
            assert_eq!(monitor.snapshot.charge_stage, Some(expected));
        }
        assert_eq!(
            sample(&mut monitor, 3000, 4, 100 * 256).state,
            State::Unknown
        );
        assert_eq!(
            sample(&mut monitor, 3500, 4, 100 * 256).state,
            State::Bypass
        );
        sample(&mut monitor, 4000, 0, 50 * 256);
        assert_eq!(
            sample(&mut monitor, 4500, 0, 50 * 256).state,
            State::Battery
        );
    }

    #[test]
    fn low_hysteresis_uses_fractional_soc_and_charging_takes_priority() {
        let mut monitor = Monitor::new();
        for ms in [0, 500, 1000, 1500] {
            sample(&mut monitor, ms, 0, 15 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Low);
        assert_eq!(
            sample(&mut monitor, 2000, 0, 20 * 256 - 1).state,
            State::Low
        );
        sample(&mut monitor, 2500, 0, 20 * 256);
        assert_eq!(
            sample(&mut monitor, 3000, 0, 20 * 256).state,
            State::Battery
        );
        assert_eq!(
            sample(&mut monitor, 3500, 0, 15 * 256 + 1).state,
            State::Battery
        );
        sample(&mut monitor, 4000, 0x0c, 5 * 256);
        assert_eq!(
            sample(&mut monitor, 4500, 0x0c, 5 * 256).state,
            State::Charging
        );
    }

    #[test]
    fn missing_gauge_on_usb_and_reconnection() {
        let mut monitor = Monitor::new();
        for ms in [0, 500] {
            monitor.update(ms, Ok((0x22, [0, 0x0c, 0])), Err("nack"), true);
        }
        assert_eq!(monitor.snapshot.state, State::Bypass);
        assert_eq!(monitor.snapshot.percent, None);
        assert_eq!(sample(&mut monitor, 1000, 0x0c, 40 * 256).percent, None);
        assert_eq!(sample(&mut monitor, 1500, 0x0c, 40 * 256).percent, None);
        sample(&mut monitor, 2000, 0x0c, 40 * 256);
        assert_eq!(
            sample(&mut monitor, 2500, 0x0c, 40 * 256).state,
            State::Charging
        );
    }

    #[test]
    fn faults_mismatches_and_stale_samples_never_claim_charging() {
        let mut monitor = Monitor::new();
        for ms in [0, 500, 1000, 1500] {
            sample(&mut monitor, ms, 0x0c, 50 * 256);
        }
        let stale = monitor.snapshot.fresh(4500);
        assert_eq!(stale.state, State::Unknown);
        assert_eq!(stale.percent, None);
        assert_eq!(stale.rate_percent_per_hour, None);
        assert_eq!(stale.power_source, None);
        assert_eq!(stale.charge_stage, None);
        assert_eq!(stale.temperature_zone(), None);
        let warm = monitor.update(
            2_000,
            Ok((0x22, [0, 0x14, 4])),
            Ok(Gauge {
                millivolts: 4050,
                soc: 80 * 256,
                rate: Some(12),
            }),
            true,
        );
        assert_eq!(warm.temperature_zone(), Some(TemperatureZone::Warm));
        for fault in [0x80, 0x40, 0x20, 0x08, 1, 2, 7] {
            assert_eq!(
                monitor
                    .update(
                        2000,
                        Ok((0x22, [0, 0x0c, fault])),
                        Ok(Gauge {
                            rate: Some(-100),
                            millivolts: 3800,
                            soc: 50 * 256
                        }),
                        true
                    )
                    .state,
                State::Unknown
            );
        }
        assert_eq!(
            monitor
                .update(2500, Err("timeout"), Err("timeout"), true)
                .state,
            State::Unknown
        );
        assert_eq!(monitor.snapshot.percent, None);
        assert_eq!(
            monitor
                .update(3000, Ok((0x22, [0, 0x0c, 0])), Err("nack"), false)
                .state,
            State::Unknown
        );
        for ms in [3500, 4000, 4500, 5000] {
            sample(&mut monitor, ms, 0x0c, 50 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Charging);
    }

    #[test]
    fn empty_connector_voltage_cycles_restart_settling() {
        let mut monitor = Monitor::new();
        for index in 0..20 {
            let snapshot = monitor.update(
                index * 500,
                Ok((0x22, [0, 0x0c, 0])),
                Ok(Gauge {
                    rate: Some(-100),
                    millivolts: if index % 2 == 0 { 4200 } else { 3800 },
                    soc: 70 * 256,
                }),
                true,
            );
            assert_eq!(snapshot.percent, None);
            assert_ne!(snapshot.state, State::Charging);
        }
    }

    #[test]
    fn gauge_byte_order_range_and_soc_clamping() {
        let gauge = decode_gauge([0xbe, 0x00, 50, 128]).unwrap();
        assert_eq!(gauge.millivolts, 3800);
        assert_eq!(gauge.soc, 50 * 256 + 128);
        assert_eq!(decode_gauge([0xbe, 0x00, 101, 0]).unwrap().soc, 101 * 256);
        assert!(decode_gauge([0, 0, 50, 0]).is_none());
        assert!(decode_gauge([0xff; 4]).is_none());
        assert_eq!(decode_gauge([0xbe, 0x00, 255, 255]).unwrap().soc, u16::MAX);
    }

    #[test]
    fn out_of_range_soc_does_not_hide_confirmed_charging() {
        let mut monitor = Monitor::new();
        for ms in [0, 500, 1000, 1500] {
            sample(&mut monitor, ms, 0x14, 0x777e);
        }
        assert_eq!(monitor.snapshot.state, State::Charging);
        assert_eq!(monitor.snapshot.percent, None);
        assert_eq!(monitor.snapshot.millivolts, Some(3800));
        assert_eq!(monitor.snapshot.gauge_error, Some("soc-out-of-range"));
        assert_eq!(
            sample(&mut monitor, 2000, 0x14, 101 * 256).percent,
            Some(100)
        );
    }

    #[test]
    fn low_read_failure_and_long_poll_gap_require_fresh_evidence() {
        let mut monitor = Monitor::new();
        for ms in [0, 500, 1000, 1500] {
            sample(&mut monitor, ms, 0, 10 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Low);
        let failed = monitor.update(2000, Ok((0x22, [0, 0, 0])), Err("timeout"), false);
        assert_eq!(failed.state, State::Unknown);
        assert_eq!(failed.percent, None);
        for ms in [2500, 3000, 3500, 4000] {
            sample(&mut monitor, ms, 0, 18 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Low);
        let resumed = sample(&mut monitor, 8000, 0, 18 * 256);
        assert_eq!(resumed.state, State::Unknown);
        assert_eq!(resumed.percent, None);
        for ms in [8500, 9000, 9500] {
            sample(&mut monitor, ms, 0, 20 * 256);
        }
        assert_eq!(monitor.snapshot.state, State::Battery);
    }
}
