use super::*;
use core::convert::Infallible;
use embedded_hal::i2c::{ErrorType, Operation};
use std::{vec, vec::Vec};

#[derive(Debug, PartialEq, Eq)]
enum Transfer {
    Read(u8, usize),
    Write(Vec<u8>),
}
struct Bus {
    regs: [u8; 57],
    log: Vec<Transfer>,
}
impl Bus {
    fn new(variant: Variant) -> Self {
        let mut bus = Self {
            regs: [0; 57],
            log: Vec::new(),
        };
        bus.regs[0x38] = (variant.part_number() << 3) | 2;
        bus.regs[0x16] = 0xa1;
        bus
    }
}
impl ErrorType for Bus {
    type Error = Infallible;
}
impl I2c for Bus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(address, ADDRESS);
        match operations {
            [Operation::Write(address), Operation::Read(bytes)] => {
                assert_eq!(address.len(), 1);
                let start = address[0] as usize;
                self.log.push(Transfer::Read(address[0], bytes.len()));
                bytes.copy_from_slice(&self.regs[start..start + bytes.len()]);
            }
            [Operation::Write(bytes)] => {
                self.log.push(Transfer::Write(bytes.to_vec()));
                let start = bytes[0] as usize;
                self.regs[start..start + bytes.len() - 1].copy_from_slice(&bytes[1..]);
                if start == 0x16 {
                    self.regs[start] &= !4;
                }
            }
            _ => panic!("unexpected I2C transaction"),
        }
        Ok(())
    }
}

#[test]
fn identity_mismatch_prevents_writes() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622), Variant::Bq25622E);
    assert!(matches!(
        device.set_charging(false),
        Err(Error::Identity { .. })
    ));
    assert_eq!(device.release().log, vec![Transfer::Read(0x38, 1)]);
}

#[test]
fn word_writes_are_little_endian_and_identity_checked_first() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622E), Variant::Bq25622E);
    device.set_limit(WordRegister::ChargeVoltage, 4200).unwrap();
    device.set_limit(WordRegister::InputVoltage, 4600).unwrap();
    assert_eq!(
        device.read_word(WordRegister::ChargeVoltage).unwrap(),
        0x0d20
    );
    assert_eq!(
        device.release().log,
        vec![
            Transfer::Read(0x38, 1),
            Transfer::Write(vec![0x04, 0x20, 0x0d]),
            Transfer::Write(vec![0x08, 0x60, 0x0e]),
            Transfer::Read(0x04, 2)
        ]
    );
}

#[test]
fn rejects_unsupported_otg_read_only_and_reserved_bits_before_io() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622E), Variant::Bq25622E);
    assert_eq!(device.set_otg(true), Err(Error::Unsupported));
    assert_eq!(
        device.read_word(WordRegister::OtgCurrent),
        Err(Error::Unsupported)
    );
    assert_eq!(
        device.write_byte(ByteRegister::ChargerControl3, 0x40),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        device.write_byte(ByteRegister::NtcControl0, 0x10),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        device.write_byte(ByteRegister::ChargerStatus0, 0),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        device.write_word(WordRegister::BatteryCurrentAdc, 0),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        device.write_byte(ByteRegister::ChargerControl2, 0x30),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        device.write_byte(ByteRegister::ChargerControl2, 0x08),
        Err(Error::InvalidValue)
    );
    assert!(device.release().log.is_empty());
}

#[test]
fn rejected_limits_have_no_bus_effect_and_never_round() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622E), Variant::Bq25622E);
    for (reg, value) in [
        (WordRegister::ChargeCurrent, 79),
        (WordRegister::ChargeCurrent, 81),
        (WordRegister::ChargeCurrent, 3600),
        (WordRegister::ChargeVoltage, 4201),
        (WordRegister::InputCurrent, 101),
        (WordRegister::TerminationCurrent, 630),
    ] {
        assert_eq!(device.set_limit(reg, value), Err(Error::InvalidValue));
    }
    assert_eq!(
        device.write_word(WordRegister::ChargeVoltage, 0x0d21),
        Err(Error::InvalidValue)
    );
    assert!(device.release().log.is_empty());
}

#[test]
fn no_battery_policy_preserves_power_path_and_suppresses_watchdog_trigger() {
    let mut bus = Bus::new(Variant::Bq25622E);
    bus.regs[0x16] = 0xff;
    let mut device = Bq25622::new(bus, Variant::Bq25622E);
    device.disable_charging_and_watchdog().unwrap();
    let bus = device.release();
    assert_eq!(bus.regs[0x16], 0xd8);
    assert!(bus.log.contains(&Transfer::Write(vec![0x16, 0xd8])));
}

#[test]
fn read_modify_write_never_replays_mode_or_reset_commands() {
    let mut bus = Bus::new(Variant::Bq25622E);
    bus.regs[0x18] = 0x0f;
    bus.regs[0x17] = 0xcd;
    let mut device = Bq25622::new(bus, Variant::Bq25622E);
    device
        .update_byte(ByteRegister::ChargerControl3, 0x10, 0x10)
        .unwrap();
    device
        .update_byte(ByteRegister::ChargerControl2, 0x40, 0)
        .unwrap();
    let bus = device.release();
    assert_eq!(bus.regs[0x18], 0x1c);
    assert_eq!(bus.regs[0x17], 0x0d);
}

#[test]
fn adc_decodes_shifted_signed_fields_and_aborted_conversion() {
    let values = AdcMeasurements::from_registers([
        (-100_i16 * 2) as u16,
        (-250_i16 * 4) as u16,
        1000 << 2,
        2000 << 2,
        2100 << 1,
        1800 << 1,
        512,
        0xfb0,
    ]);
    assert_eq!(values.bus_current_ma, -200);
    assert_eq!(values.battery_current_ma, Some(-1000));
    assert_eq!(values.bus_voltage_uv, 3_970_000);
    assert_eq!(values.pmid_voltage_uv, 7_940_000);
    assert_eq!(values.battery_voltage_uv, 4_179_000);
    assert_eq!(values.system_voltage_uv, 3_582_000);
    assert_eq!(values.thermistor_ratio_ppm, 492_032);
    assert_eq!(values.die_temperature_millicelsius, -40_000);
    let values = AdcMeasurements::from_registers([0, 0x8000, 0, 0, 0, 0, 0, 280]);
    assert_eq!(values.battery_current_ma, None);
    assert_eq!(values.die_temperature_millicelsius, 140_000);
}

#[test]
fn variant_e_identity_is_three_and_base_identity_is_one() {
    for variant in [Variant::Bq25622, Variant::Bq25622E] {
        let mut device = Bq25622::new(Bus::new(variant), variant);
        assert_eq!(
            device.verify_identity().unwrap().part_number,
            variant.part_number()
        );
    }
}

#[test]
fn adc_configuration_uses_explicit_resolution_and_channel_mask() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622E), Variant::Bq25622E);
    device
        .configure_adc(AdcConfig {
            enabled: true,
            one_shot: true,
            resolution: AdcResolution::Bits11,
            running_average: true,
            initialize_average: true,
            disabled_channels: 0x80,
        })
        .unwrap();
    let bus = device.release();
    assert_eq!(bus.regs[0x26], 0xdc);
    assert_eq!(bus.regs[0x27], 0x80);
}

#[test]
fn validates_entire_charge_profile_before_any_transfer() {
    let mut device = Bq25622::new(Bus::new(Variant::Bq25622E), Variant::Bq25622E);
    assert_eq!(
        device.configure_limits(ChargeLimits {
            charge_current_ma: 1040,
            charge_voltage_mv: 4200,
            input_current_ma: 500,
            input_voltage_mv: 4600,
            minimum_system_voltage_mv: 3520,
            precharge_current_ma: 100,
            termination_current_ma: 61
        }),
        Err(Error::InvalidValue)
    );
    assert!(device.release().log.is_empty());
}
