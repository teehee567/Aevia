use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
use std::{vec, vec::Vec};

struct Mock {
    regs: [u8; 0x49],
    reads: Vec<(u8, usize)>,
    writes: Vec<Vec<u8>>,
    fail_write: Option<u8>,
}
impl Mock {
    fn new() -> Self {
        let mut regs = [0; 0x49];
        regs[0x48] = 0x19;
        regs[0x09] = 5;
        regs[0x08] = 0xc3;
        regs[0x0a] = 0x40;
        regs[0x0f] = 0x22;
        regs[0x10] = 0x85;
        regs[0] = 18;
        regs[1..3].copy_from_slice(&840u16.to_be_bytes());
        Self {
            regs,
            reads: Vec::new(),
            writes: Vec::new(),
            fail_write: None,
        }
    }
}
impl ErrorType for Mock {
    type Error = ErrorKind;
}
impl I2c for Mock {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(address, ADDRESS);
        match operations {
            [Operation::Write(prefix), Operation::Read(data)] => {
                assert_eq!(prefix.len(), 1);
                let start = prefix[0] as usize;
                self.reads.push((prefix[0], data.len()));
                data.copy_from_slice(&self.regs[start..start + data.len()]);
                for index in start..start + data.len() {
                    if (0x22..=0x27).contains(&index) {
                        self.regs[index] = 0;
                    }
                }
            }
            [Operation::Write(data)] => {
                self.writes.push(data.to_vec());
                if self.fail_write == Some(data[0]) {
                    return Err(ErrorKind::Other);
                }
                let start = data[0] as usize;
                let prior_cells = self.regs[0x0a] & 0xc0;
                self.regs[start..start + data.len() - 1].copy_from_slice(&data[1..]);
                if start == 0x0a && self.regs[0x0a] & 0xc0 != prior_cells {
                    let cells = (self.regs[0x0a] >> 6) as usize;
                    self.regs[0] = [4, 18, 26, 38][cells];
                    self.regs[1..3]
                        .copy_from_slice(&[420u16, 840, 1260, 1680][cells].to_be_bytes());
                    self.regs[3..5].copy_from_slice(&100u16.to_be_bytes());
                }
            }
            _ => panic!("unexpected transaction"),
        }
        Ok(())
    }
}
fn config() -> ChargeConfig {
    ChargeConfig {
        cells: Cells::Two,
        minimum_system_voltage_mv: 6000,
        charge_voltage_mv: 8400,
        charge_current_ma: 2000,
        input_voltage_mv: 4500,
        input_current_ma: 3000,
        precharge_current_ma: 120,
        termination_current_ma: 200,
    }
}

#[test]
fn construction_is_silent_and_identity_prevents_writes() {
    let mut bus = Mock::new();
    bus.regs[0x48] = 0x08;
    let mut device = Bq25798::new(bus);
    assert_eq!(
        device.set_charging(false),
        Err(Error::Identity(Identity::from_raw(0x08)))
    );
    let bus = device.release();
    assert!(bus.writes.is_empty());
    assert_eq!(bus.reads, [(0x48, 1)]);
    let bus = Bq25798::new(Mock::new()).release();
    assert!(bus.reads.is_empty());
}

#[test]
fn all_word_operations_are_big_endian_and_have_correct_start_addresses() {
    let mut device = Bq25798::new(Mock::new());
    device.set_charge_current_ma(2000).unwrap();
    device.set_input_current_ma(3300).unwrap();
    assert_eq!(device.read_word(WordRegister::ChargeCurrentLimit), Ok(200));
    let bus = device.release();
    assert_eq!(bus.writes, [vec![3, 0, 200], vec![6, 1, 74]]);
    assert_eq!(bus.reads, [(0x48, 1), (3, 2)]);
}

#[test]
fn invalid_profiles_and_ranges_do_no_io() {
    let mut device = Bq25798::new(Mock::new());
    let mut bad = config();
    bad.charge_voltage_mv = 10000;
    assert_eq!(device.configure_charge(bad), Err(Error::InvalidValue));
    bad = config();
    bad.minimum_system_voltage_mv = 8500;
    assert_eq!(device.configure_charge(bad), Err(Error::InvalidValue));
    assert_eq!(device.set_charge_current_ma(1001), Err(Error::InvalidValue));
    assert_eq!(device.set_input_current_ma(3310), Err(Error::InvalidValue));
    assert_eq!(device.set_precharge_current_ma(0), Err(Error::InvalidValue));
    assert_eq!(
        device.set_termination_current_ma(39),
        Err(Error::InvalidValue)
    );
    assert_eq!(device.set_otg_limits(5000, 159), Err(Error::InvalidValue));
    assert_eq!(
        device.write_word(WordRegister::IcoCurrentLimit, 5),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        device.write_byte(ByteRegister::FaultMask1, 8),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        device.write_byte(ByteRegister::DpdmDriver, 0x1c),
        Err(Error::InvalidValue)
    );
    let bus = device.release();
    assert!(bus.reads.is_empty());
    assert!(bus.writes.is_empty());
}

#[test]
fn rmw_preserves_fields_without_replaying_actions() {
    let mut bus = Mock::new();
    bus.regs[0x10] = 0x9d;
    bus.regs[0x09] = 0x65;
    bus.regs[0x11] = 0xc7;
    let mut device = Bq25798::new(bus);
    device.set_watchdog(Watchdog::Disabled).unwrap();
    device.set_termination_current_ma(400).unwrap();
    device
        .update_byte(ByteRegister::ChargerControl2, 0x40, 0)
        .unwrap();
    let bus = device.release();
    assert_eq!(
        bus.writes,
        [vec![0x10, 0x90], vec![0x09, 0x2a], vec![0x11, 1]]
    );
}

#[test]
fn profile_accounts_for_cell_reset_and_keeps_charging_disabled_on_failure() {
    let mut bus = Mock::new();
    bus.fail_write = Some(0x06);
    let mut device = Bq25798::new(bus);
    let mut profile = config();
    profile.cells = Cells::Three;
    profile.charge_voltage_mv = 13000;
    profile.minimum_system_voltage_mv = 12500;
    assert_eq!(
        device.configure_charge(profile),
        Err(Error::Bus(ErrorKind::Other))
    );
    let bus = device.release();
    assert_eq!(bus.regs[0x0f] & 0x20, 0);
    assert_eq!(bus.regs[0x0a] >> 6, 2);
    assert_eq!(u16::from_be_bytes([bus.regs[1], bus.regs[2]]), 1300);
    assert_eq!(bus.regs[0], 40); // 2500 + 40*250 = 12500mV
    assert!(
        bus.writes
            .iter()
            .all(|bytes| bytes[0] != 0x0f || bytes[1] & 0x20 == 0)
    );
}

#[test]
fn cell_dependent_voltage_rejects_before_write() {
    let mut device = Bq25798::new(Mock::new());
    assert_eq!(
        device.set_charge_voltage_mv(10000),
        Err(Error::InvalidValue)
    );
    assert!(device.release().writes.is_empty());
    assert_eq!(Cells::One.voltage_range_mv(), (3000, 4990));
    assert_eq!(Cells::Three.voltage_range_mv(), (10000, 13990));
}

#[test]
fn adc_uses_signed_currents_die_temperature_and_ts_ratio() {
    let mut words = [0; 11];
    words[0] = (-500i16) as u16;
    words[1] = 1200;
    words[2] = 12000;
    words[3] = 12100;
    words[4] = 0;
    words[5] = 8300;
    words[6] = 8350;
    words[7] = 512;
    words[8] = (-40i16) as u16;
    words[9] = 600;
    words[10] = 1200;
    let mut bus = Mock::new();
    for (i, value) in words.into_iter().enumerate() {
        bus.regs[0x31 + i * 2..0x33 + i * 2].copy_from_slice(&value.to_be_bytes());
    }
    let mut device = Bq25798::new(bus);
    let got = device.measurements().unwrap();
    assert_eq!(got.bus_current_ma, -500);
    assert_eq!(got.battery_current_ma, 1200);
    assert_eq!(got.bus_voltage_mv, 12000);
    assert_eq!(got.battery_voltage_mv, 8300);
    assert_eq!(got.thermistor_ratio, 0.5);
    assert_eq!(got.die_temperature_millicelsius, -20000);
    assert_eq!(got.dp_voltage_mv, 600);
    assert_eq!(device.release().reads, [(0x31, 22)]);
}

#[test]
fn status_does_not_acknowledge_interrupts_but_flag_reads_do() {
    let mut bus = Mock::new();
    bus.regs[0x1b] = 0x29;
    bus.regs[0x1c] = 0x61;
    bus.regs[0x1d] = 1;
    bus.regs[0x1e] = 0x20;
    bus.regs[0x20] = 0x40;
    bus.regs[0x22..0x28].copy_from_slice(&[1, 2, 3, 4, 5, 0x80]);
    let mut device = Bq25798::new(bus);
    let status = device.status().unwrap();
    assert_eq!(status.charge_state(), ChargeState::ConstantCurrent);
    assert!(
        status.power_good()
            && status.watchdog_expired()
            && status.battery_present()
            && status.adc_done()
            && status.has_fault()
    );
    let flags = device.take_interrupt_flags().unwrap();
    assert_eq!(flags.charger, [1, 2, 3, 4]);
    assert_eq!(flags.faults, [5, 0x80]);
    assert_eq!(device.take_interrupt_flags().unwrap().charger, [0; 4]);
}

#[test]
fn adc_masks_validate_before_io_and_full_settings_are_encoded() {
    let mut device = Bq25798::new(Mock::new());
    assert_eq!(
        device.configure_adc(AdcConfig {
            disabled_channels_0: 1,
            ..AdcConfig::default()
        }),
        Err(Error::InvalidValue)
    );
    assert!(device.bus.reads.is_empty());
    device
        .configure_adc(AdcConfig {
            enabled: true,
            initialize_average: true,
            disabled_channels_1: 0xc0,
            ..AdcConfig::default()
        })
        .unwrap();
    assert_eq!(
        device.release().writes,
        [vec![0x2f, 0], vec![0x30, 0xc0], vec![0x2e, 0xc4]]
    );
}

#[test]
fn otg_limits_disable_first_and_preserve_timer_bit() {
    let mut bus = Mock::new();
    bus.regs[0x0d] = 0xcb;
    bus.regs[0x12] = 0xff;
    let mut device = Bq25798::new(bus);
    device.set_otg_limits(5000, 3040).unwrap();
    assert_eq!(
        device.release().writes,
        [vec![0x12, 0xbf], vec![0x0b, 0, 0xdc], vec![0x0d, 0xcc]]
    );
}

#[test]
fn power_mode_requires_declared_ship_fet_and_drops_detection_trigger() {
    let mut device = Bq25798::new(Mock::new());
    assert_eq!(
        device.set_power_mode(PowerMode::Ship, true),
        Err(Error::MissingShipFet)
    );
    assert!(device.bus.writes.is_empty());
    device.bus.regs[0x14] = 0x80;
    device.bus.regs[0x11] = 0xfc;
    device.set_power_mode(PowerMode::Ship, true).unwrap();
    assert_eq!(device.release().writes, [vec![0x11, 0x7d]]);
}

#[test]
fn interrupt_masks_are_reserved_bit_checked_and_contiguous() {
    let mut device = Bq25798::new(Mock::new());
    let mut masks = InterruptMasks {
        charger: [0xff, 0xd7, 0x7f, 0x1f],
        faults: [0xff, 0xf4],
    };
    masks.charger[1] |= 0x20;
    assert_eq!(
        device.configure_interrupt_masks(masks),
        Err(Error::InvalidValue)
    );
    assert!(device.bus.reads.is_empty());
    masks.charger[1] = 0xd7;
    device.configure_interrupt_masks(masks).unwrap();
    assert_eq!(
        device.release().writes,
        [vec![0x28, 0xff, 0xd7, 0x7f, 0x1f, 0xff, 0xf4]]
    );
}

#[test]
fn typed_voltage_setters_reject_inconsistent_system_voltage() {
    let mut device = Bq25798::new(Mock::new());
    assert_eq!(
        device.set_minimum_system_voltage_mv(8500),
        Err(Error::InvalidValue)
    );
    assert_eq!(device.set_charge_voltage_mv(6000), Err(Error::InvalidValue));
    assert!(device.release().writes.is_empty());
}
