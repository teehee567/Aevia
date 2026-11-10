use super::*;
use core::{cell::RefCell, convert::Infallible};
use embedded_hal::i2c::{ErrorKind, Operation};
use std::{rc::Rc, vec::Vec};

fn beta() -> BetaCurve {
    // Deliberately synthetic, not a fitted-part specification.
    BetaCurve::new(
        10_000.0,
        25.0,
        3950.0,
        TemperatureRange::new(-40.0, 125.0).unwrap(),
    )
    .unwrap()
}
fn ratio_for_resistance(resistance: f32) -> f32 {
    let parallel = 30_100.0 * resistance / (30_100.0 + resistance);
    parallel / (5230.0 + parallel)
}
fn close(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() < tolerance,
        "{actual} vs {expected}"
    );
}
#[test]
fn mini_divider_inverts_parallel_network_and_reference_temperature() {
    let divider = Divider::v2_mini();
    let ratio = ratio_for_resistance(10_000.0);
    close(divider.resistance_ohms(ratio).unwrap(), 10_000.0, 0.01);
    close(
        divider.temperature_from_ratio(&beta(), ratio).unwrap(),
        25.0,
        0.001,
    );
    let cold_resistance = 10_000.0 * libm::expf(3950.0 * (1.0 / 263.15 - 1.0 / 298.15));
    close(
        divider
            .temperature_from_ratio(&beta(), ratio_for_resistance(cold_resistance))
            .unwrap(),
        -10.0,
        0.001,
    );
}
#[test]
fn invalid_networks_and_fault_readings_are_rejected() {
    assert_eq!(
        Divider::new(0.0, None),
        Err(ConversionError::InvalidConfiguration)
    );
    assert!(Divider::new(f32::NAN, None).is_err());
    assert!(Divider::new(1.0, Some(-2.0)).is_err());
    let d = Divider::v2_mini();
    assert_eq!(d.resistance_ohms(0.0), Err(ConversionError::ShortCircuit));
    assert_eq!(d.resistance_ohms(1.0), Err(ConversionError::OpenCircuit));
    assert_eq!(d.resistance_ohms(0.99), Err(ConversionError::OpenCircuit));
    assert_eq!(
        d.resistance_ohms(f32::NAN),
        Err(ConversionError::InvalidRatio)
    );
    assert_eq!(d.resistance_ohms(-0.1), Err(ConversionError::InvalidRatio));
    assert_eq!(
        d.resistance_from_ratio_ppm(1_000_001),
        Err(ConversionError::InvalidRatio)
    );
    assert!(beta().temperature_celsius(0.0).is_err());
    assert!(beta().temperature_celsius(f32::INFINITY).is_err());
    assert_eq!(
        TemperatureRange::new(-273.15, 1.0),
        Err(ConversionError::InvalidConfiguration)
    );
}
#[test]
fn curves_validate_domains_and_table_order_without_extrapolation() {
    let points = [
        Point {
            resistance_ohms: 100_000.0,
            celsius: -20.0,
        },
        Point {
            resistance_ohms: 10_000.0,
            celsius: 25.0,
        },
        Point {
            resistance_ohms: 1_000.0,
            celsius: 80.0,
        },
    ];
    let table = TableCurve::new(&points).unwrap();
    assert_eq!(table.temperature_celsius(10_000.0), Ok(25.0));
    close(
        table
            .temperature_celsius(libm::sqrtf(10_000.0 * 100_000.0))
            .unwrap(),
        2.5,
        0.001,
    );
    assert_eq!(
        table.temperature_celsius(100_001.0),
        Err(ConversionError::OutOfRange)
    );
    assert!(TableCurve::new(&points[..1]).is_err());
    assert!(TableCurve::new(&[points[1], points[0]]).is_err());
    let range = TemperatureRange::new(-40.0, 125.0).unwrap();
    let curve = SteinhartHartCurve::new(
        [
            1.0 / 298.15 - libm::logf(10_000.0) / 3950.0,
            1.0 / 3950.0,
            0.0,
        ],
        [100.0, 1_000_000.0],
        range,
    )
    .unwrap();
    close(curve.temperature_celsius(10_000.0).unwrap(), 25.0, 0.001);
    assert_eq!(
        curve.temperature_celsius(1.0),
        Err(ConversionError::OutOfRange)
    );
    assert!(SteinhartHartCurve::new([f32::NAN, 0.0, 0.0], [1.0, 2.0], range).is_err());
}
struct Analog(Result<u32, u8>);
impl AnalogInput for Analog {
    type Error = u8;
    fn read_microvolts(&mut self) -> Result<u32, u8> {
        self.0
    }
}
#[test]
fn adc_adapter_uses_calibrated_microvolts_and_preserves_errors() {
    let mut sensor = AdcThermistor::new(
        Analog(Ok(1_650_000)),
        Divider::new(10_000.0, None).unwrap(),
        beta(),
        3_300_000,
    )
    .unwrap();
    close(sensor.temperature_celsius().unwrap(), 25.0, 0.001);
    let mut fault =
        AdcThermistor::new(Analog(Err(7)), Divider::v2_mini(), beta(), 3_300_000).unwrap();
    assert_eq!(fault.temperature_celsius(), Err(Error::Transport(7)));
    assert!(AdcThermistor::new(Analog(Ok(0)), Divider::v2_mini(), beta(), 0).is_err());
}
struct BadCurve;
impl NtcCurve for BadCurve {
    fn temperature_celsius(&self, _: f32) -> Result<f32, ConversionError> {
        Ok(f32::NAN)
    }
}
#[test]
fn custom_curves_cannot_leak_nonfinite_temperatures() {
    assert_eq!(
        Divider::new(10_000.0, None)
            .unwrap()
            .temperature_from_ratio(&BadCurve, 0.5),
        Err(ConversionError::OutOfRange)
    );
}
#[derive(Clone)]
struct Bus(Rc<RefCell<State>>);
struct State {
    regs: [u8; 256],
    transfers: usize,
    fail: bool,
    writes: Vec<Vec<u8>>,
}
impl Bus {
    fn new() -> Self {
        let mut state = State {
            regs: [0; 256],
            transfers: 0,
            fail: false,
            writes: Vec::new(),
        };
        state.regs[0x38] = 3 << 3;
        Self(Rc::new(RefCell::new(state)))
    }
    fn complete(&self, raw: u16, register: usize) {
        let mut state = self.0.borrow_mut();
        state.regs[0x26] &= !0x80;
        state.regs[0x1d] = 0x40;
        state.regs[register..register + 2].copy_from_slice(&raw.to_le_bytes());
    }
}
impl embedded_hal::i2c::ErrorType for Bus {
    type Error = ErrorKind;
}
impl I2c for Bus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
        assert_eq!(address, 0x6b);
        let mut state = self.0.borrow_mut();
        state.transfers += 1;
        if state.fail {
            return Err(ErrorKind::Other);
        }
        match operations {
            [Operation::Write(pointer), Operation::Read(data)] => {
                let offset = pointer[0] as usize;
                data.copy_from_slice(&state.regs[offset..offset + data.len()]);
            }
            [Operation::Write(bytes)] => {
                let offset = bytes[0] as usize;
                state.regs[offset..offset + bytes.len() - 1].copy_from_slice(&bytes[1..]);
                state.writes.push(bytes.to_vec());
            }
            _ => panic!("unexpected I2C transaction"),
        }
        Ok(())
    }
}
#[test]
fn charger_requires_started_conversion_and_never_uses_old_ts_as_fresh() {
    let bus = Bus::new();
    bus.complete(600, 0x34); // Old result and old DONE are not proof of freshness.
    let mut sensor = Bq25622Thermistor::v2_mini(
        bq25622::Bq25622::new(bus.clone(), bq25622::Variant::Bq25622E),
        beta(),
    );
    assert_eq!(bus.0.borrow().transfers, 0);
    assert_eq!(sensor.temperature_celsius(), Err(Error::NotStarted));
    sensor
        .start_one_shot(bq25622::AdcResolution::Bits12)
        .unwrap();
    assert_eq!(sensor.temperature_celsius(), Err(Error::Pending));
    let ppm = ratio_for_resistance(10_000.0) * 1_000_000.0;
    let raw = libm::roundf(ppm / 961.0) as u16;
    bus.complete(raw, 0x34);
    close(sensor.temperature_celsius().unwrap(), 25.0, 0.2);
    assert_eq!(sensor.temperature_celsius(), Err(Error::NotStarted));
    let writes = &bus.0.borrow().writes;
    assert!(writes.iter().all(|w| w[0] == 0x26 || w[0] == 0x27));
    assert!(writes.contains(&std::vec![0x27, 0xfb]));
}
#[test]
fn ts_ignored_channel_disable_and_transport_errors_remain_visible() {
    let bus = Bus::new();
    let mut sensor = Bq25622Thermistor::v2_mini(
        bq25622::Bq25622::new(bus.clone(), bq25622::Variant::Bq25622E),
        beta(),
    );
    bus.0.borrow_mut().regs[0x1a] = 0x80;
    assert_eq!(
        sensor.start_one_shot(bq25622::AdcResolution::Bits12),
        Err(Error::TsIgnored)
    );
    assert!(bus.0.borrow().writes.is_empty());
    bus.0.borrow_mut().regs[0x1a] = 0;
    sensor
        .start_one_shot(bq25622::AdcResolution::Bits12)
        .unwrap();
    bus.0.borrow_mut().regs[0x27] |= 0x04;
    assert_eq!(sensor.temperature_celsius(), Err(Error::ChannelDisabled));
    bus.0.borrow_mut().fail = true;
    assert_eq!(
        sensor.temperature_celsius(),
        Err(Error::Transport(bq25622::Error::Bus(ErrorKind::Other)))
    );
}
#[test]
fn signed_die_temperature_is_separate_from_ntc_curve() {
    let bus = Bus::new();
    let mut sensor = Bq25622DieTemperature::new(bq25622::Bq25622::new(
        bus.clone(),
        bq25622::Variant::Bq25622E,
    ));
    sensor
        .start_one_shot(bq25622::AdcResolution::Bits12)
        .unwrap();
    assert_eq!(sensor.try_temperature_celsius(), Ok(None));
    bus.complete(0xfec, 0x36); // Signed -20 counts * 0.5 °C.
    assert_eq!(sensor.temperature_celsius(), Ok(-10.0));
    assert!(bus.0.borrow().writes.contains(&std::vec![0x27, 0xfd]));
}
// Keep the common trait usable with an infallible analog backend too.
struct InfallibleAnalog;
impl AnalogInput for InfallibleAnalog {
    type Error = Infallible;
    fn read_microvolts(&mut self) -> Result<u32, Infallible> {
        Ok(1)
    }
}
#[test]
fn infallible_transport_still_returns_conversion_errors() {
    let mut sensor = AdcThermistor::new(
        InfallibleAnalog,
        Divider::new(1.0, None).unwrap(),
        beta(),
        1,
    )
    .unwrap();
    assert_eq!(
        sensor.temperature_celsius(),
        Err(Error::Conversion(ConversionError::OpenCircuit))
    );
}

#[test]
fn explicit_murata_preset_matches_primary_r25_and_b25_50() {
    let curve = BetaCurve::murata_ncu18xh103f60rb(TemperatureRange::new(-40.0, 125.0).unwrap());
    close(curve.temperature_celsius(10_000.0).unwrap(), 25.0, 0.001);
    let at50 = 10_000.0 * libm::expf(3380.0 * (1.0 / 323.15 - 1.0 / 298.15));
    close(curve.temperature_celsius(at50).unwrap(), 50.0, 0.001);
}

#[test]
fn die_adc_result_outside_documented_clamp_range_is_rejected() {
    let bus = Bus::new();
    let mut sensor = Bq25622DieTemperature::new(bq25622::Bq25622::new(
        bus.clone(),
        bq25622::Variant::Bq25622E,
    ));
    sensor
        .start_one_shot(bq25622::AdcResolution::Bits12)
        .unwrap();
    bus.complete(0x7ff, 0x36);
    assert_eq!(
        sensor.temperature_celsius(),
        Err(Error::Conversion(ConversionError::OutOfRange))
    );
}
