use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
use std::{collections::VecDeque, vec, vec::Vec};

#[derive(Clone, Debug)]
enum Step {
    Read(u8, u8, Vec<u8>),
    Write(u8, Vec<u8>, bool),
    ReadFail(u8, u8),
}
struct Bus {
    steps: VecDeque<Step>,
}
impl Bus {
    fn new(steps: Vec<Step>) -> Self {
        Self {
            steps: steps.into(),
        }
    }
    fn done(&self) {
        assert!(self.steps.is_empty(), "unconsumed steps {:?}", self.steps);
    }
}
impl ErrorType for Bus {
    type Error = ErrorKind;
}
impl I2c for Bus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        match self.steps.pop_front().expect("unexpected I2C access") {
            Step::Read(addr, reg, data) => {
                assert_eq!(address, addr);
                let [Operation::Write(command), Operation::Read(output)] = operations else {
                    panic!("expected repeated START word read")
                };
                assert_eq!(*command, [reg]);
                assert_eq!(output.len(), data.len());
                output.copy_from_slice(&data);
                Ok(())
            }
            Step::ReadFail(addr, reg) => {
                assert_eq!(address, addr);
                let [Operation::Write(command), Operation::Read(_)] = operations else {
                    panic!("read expected")
                };
                assert_eq!(*command, [reg]);
                Err(ErrorKind::Bus)
            }
            Step::Write(addr, data, fail) => {
                assert_eq!(address, addr);
                let [Operation::Write(actual)] = operations else {
                    panic!("write expected")
                };
                assert_eq!(*actual, data);
                if fail {
                    Err(ErrorKind::ArbitrationLoss)
                } else {
                    Ok(())
                }
            }
        }
    }
}
fn read(r: Register, v: u16) -> Step {
    Step::Read(r.bus_address(), r.command_byte(), v.to_le_bytes().to_vec())
}
fn write(r: Register, v: u16) -> Step {
    Step::Write(
        r.bus_address(),
        vec![r.command_byte(), v as u8, (v >> 8) as u8],
        false,
    )
}
fn fail_write(r: Register, v: u16) -> Step {
    Step::Write(
        r.bus_address(),
        vec![r.command_byte(), v as u8, (v >> 8) as u8],
        true,
    )
}
fn cfg() -> Configuration {
    Configuration::new(2, 5000).unwrap()
}
fn prefix(comm: u16) -> Vec<Step> {
    vec![
        read(Register::DEVICE_NAME, 0x420b),
        read(Register::COMMUNICATION_STATUS, comm),
        write(Register::COMMUNICATION_STATUS, comm & 0x0304),
        write(Register::COMMUNICATION_STATUS, comm & 0x0304),
    ]
}
fn suffix(steps: &mut Vec<Step>, comm: u16) {
    steps.extend([
        read(Register::COMMUNICATION_STATUS, comm),
        write(Register::COMMUNICATION_STATUS, (comm & 0x0304) | 0x00f9),
        write(Register::COMMUNICATION_STATUS, (comm & 0x0304) | 0x00f9),
    ]);
}
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}

#[test]
fn address_mapping_and_little_endian_match_communication_examples() {
    let mut gauge = Max17320::new(
        Bus::new(vec![
            read(Register::CURRENT, 0xfffe),
            read(Register::THERMISTOR1, 0xf580),
            read(Register::N_SENSE_RESISTANCE, 500),
        ]),
        cfg(),
    );
    assert_eq!(gauge.read_register(Register::CURRENT), Ok(0xfffe));
    assert_eq!(gauge.thermistor_celsius(1), Ok(-10.5));
    assert_eq!(gauge.read_register(Register::N_SENSE_RESISTANCE), Ok(500));
    gauge.release().done();
    assert!(Register::new(0x200).is_none());
    assert_eq!(Register::N_SENSE_RESISTANCE.bus_address(), 0x0b);
    assert_eq!(Register::CURRENT.bus_address(), 0x36);
}

#[test]
fn standard_units_include_pack_scale_signed_current_and_explicit_rsense() {
    assert_eq!(cell_voltage_microvolts(51200), 4_000_000);
    assert_eq!(pack_voltage_microvolts(25600), 8_000_000);
    assert_eq!(cfg().current_microamps((-3200_i16) as u16), -1_000_000);
    assert_eq!(cfg().capacity_microamp_hours(2000), 2_000_000);
    assert_eq!(cfg().power_microwatts((-10_i16) as u16), -16000);
    assert_eq!(temperature_celsius((-2688_i16) as u16), -10.5);
    assert_eq!(percentage(12800), 50.0);
    assert_eq!(time_milliseconds(640), 3_600_000);
    assert_eq!(
        Configuration::new(4, 2500).unwrap().current_microamps(1600),
        1_000_000
    );
    assert!(Configuration::new(1, 5000).is_none());
    assert!(Configuration::new(3, 0).is_none());
}

#[test]
fn unsupported_identity_and_invalid_profiles_never_unlock() {
    let mut gauge = Max17320::new(Bus::new(vec![read(Register::DEVICE_NAME, 0x4201)]), cfg());
    assert_eq!(
        gauge.set_voltage_alerts_mv(3000, 4200),
        Err(Error::Device(Failure::Identity(0x4201)))
    );
    gauge.release().done();
    let mut gauge = Max17320::new(Bus::new(vec![]), cfg());
    assert_eq!(
        gauge.set_voltage_alerts_mv(3001, 4200),
        Err(Error::Device(Failure::InvalidValue))
    );
    assert_eq!(
        gauge.cell_microvolts(3),
        Err(Error::Device(Failure::InvalidValue))
    );
    assert_eq!(
        gauge.write_shadow(&[(Register::N_CONFIG, 0), (Register::N_ROM_ID0, 0)]),
        Err(Error::Device(Failure::ReadOnly))
    );
    gauge.release().done();
}

#[test]
fn protected_write_uses_consecutive_unlock_relock_and_preserves_fet_override() {
    let mut steps = prefix(0x0304);
    steps.push(write(Register::VOLTAGE_ALERT, 0xd296));
    suffix(&mut steps, 0x0304);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    gauge.set_voltage_alerts_mv(3000, 4200).unwrap();
    gauge.release().done();
}

#[test]
fn failed_operation_still_relocks_and_keeps_both_operation_and_cleanup_errors() {
    let mut steps = prefix(0);
    steps.push(fail_write(Register::SOC_ALERT, 0x640a));
    steps.extend([
        read(Register::COMMUNICATION_STATUS, 0),
        fail_write(Register::COMMUNICATION_STATUS, 0xf9),
        write(Register::COMMUNICATION_STATUS, 0xf9),
    ]);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    assert_eq!(
        gauge.set_soc_alerts_percent(10, 100),
        Err(Error::Relock {
            operation: Some(Failure::Bus(ErrorKind::ArbitrationLoss)),
            error: ErrorKind::ArbitrationLoss
        })
    );
    gauge.release().done();
}

#[test]
fn cleanup_read_failure_uses_original_override_and_still_attempts_both_locks() {
    let mut steps = prefix(0x0200);
    steps.extend([
        write(Register::SOC_ALERT, 0x640a),
        Step::ReadFail(0x36, 0x61),
        write(Register::COMMUNICATION_STATUS, 0x02f9),
        write(Register::COMMUNICATION_STATUS, 0x02f9),
    ]);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    assert_eq!(
        gauge.set_soc_alerts_percent(10, 100),
        Err(Error::Relock {
            operation: None,
            error: ErrorKind::Bus
        })
    );
    gauge.release().done();
}

#[test]
fn maintenance_can_deliberately_change_fet_override_without_cleanup_reverting_it() {
    let mut steps = prefix(0);
    steps.push(write(Register::COMMUNICATION_STATUS, 0x0300));
    suffix(&mut steps, 0x0300);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    gauge
        .maintenance(|port| port.write_register_raw(Register::COMMUNICATION_STATUS, 0x0300))
        .unwrap();
    gauge.release().done();
}

#[test]
fn busy_device_is_never_unlocked() {
    let mut gauge = Max17320::new(
        Bus::new(vec![
            read(Register::DEVICE_NAME, 0x420a),
            read(Register::COMMUNICATION_STATUS, 2),
        ]),
        cfg(),
    );
    assert_eq!(
        gauge.set_soc_alerts_percent(10, 100),
        Err(Error::Device(Failure::Busy))
    );
    gauge.release().done();
}

#[test]
fn protection_history_is_cleared_before_summary_without_read_modify_write_of_status() {
    let mut steps = prefix(0);
    steps.extend([
        read(Register::PROTECTION_ALERT, 0x0808),
        write(Register::PROTECTION_ALERT, 0),
        write(Register::STATUS, 0x7fff),
    ]);
    suffix(&mut steps, 0);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    let flags = gauge.clear_protection_alerts().unwrap();
    assert!(flags.contains(ProtectionFlags::OVERVOLTAGE | ProtectionFlags::UNDERVOLTAGE));
    gauge.release().done();
}

#[test]
fn sbs_word_pec_checks_address_prefix_and_rejects_block_as_word() {
    let value = 0x1234_u16;
    let crc = pec(&[0x16, 0x08, 0x17, 0x34, 0x12]);
    let mut gauge = Max17320::new(
        Bus::new(vec![
            Step::Read(0x0b, 0x08, vec![0x34, 0x12, crc]),
            Step::Read(0x0b, 0x08, vec![0x34, 0x12, crc ^ 1]),
        ]),
        cfg(),
    );
    assert_eq!(
        gauge.read_sbs_word_pec(Register::SBS_TEMPERATURE),
        Ok(value)
    );
    assert_eq!(
        gauge.read_sbs_word_pec(Register::SBS_TEMPERATURE),
        Err(Error::Device(Failure::Pec))
    );
    assert_eq!(
        gauge.read_register(Register::SBS_DEVICE_NAME),
        Err(Error::Device(Failure::SbsBlockRequired))
    );
    gauge.release().done();
    assert_eq!(pec(b"123456789"), 0xf4);
}

#[test]
fn nonvolatile_copy_is_explicit_waits_maximum_time_and_never_retries_error() {
    let mut steps = prefix(0);
    steps.extend([
        read(Register::COMMUNICATION_STATUS, 0),
        write(Register::COMMUNICATION_STATUS, 0),
        write(Register::COMMAND, 0xe904),
        read(Register::COMMUNICATION_STATUS, 4),
    ]);
    suffix(&mut steps, 4);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    let mut delay = Delay(vec![]);
    assert_eq!(
        gauge.copy_shadow_to_nonvolatile(&mut delay),
        Err(Error::Device(Failure::Nonvolatile))
    );
    assert_eq!(delay.0, [4_294_000_000, 3_066_000_000]);
    gauge.release().done();
}

#[test]
fn gauge_reset_is_bounded_and_does_not_repeat_reset_command() {
    let mut steps = prefix(0);
    steps.extend([
        write(Register::CONFIG2, 0x8000),
        read(Register::CONFIG2, 0x8000),
        read(Register::CONFIG2, 0x8000),
    ]);
    suffix(&mut steps, 0);
    let mut gauge = Max17320::new(Bus::new(steps), cfg());
    let mut delay = Delay(vec![]);
    assert_eq!(
        gauge.reset(false, 2, &mut delay),
        Err(Error::Device(Failure::Timeout))
    );
    assert_eq!(delay.0, [1_000_000, 1_000_000]);
    gauge.release().done();
}
