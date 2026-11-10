use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
use std::{collections::VecDeque, vec, vec::Vec};

#[derive(Debug)]
enum Expected {
    Read(u8, Vec<u8>),
    Write(Vec<u8>),
    Fail(u8),
}
struct Mock {
    expected: VecDeque<Expected>,
}
impl Mock {
    fn new(expected: Vec<Expected>) -> Self {
        Self {
            expected: expected.into(),
        }
    }
    fn done(self) {
        assert!(
            self.expected.is_empty(),
            "unused transactions: {:?}",
            self.expected
        );
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
        assert_eq!(address, 0x52);
        let expected = self.expected.pop_front().expect("unexpected transaction");
        match expected {
            Expected::Read(command, bytes) => match operations {
                [Operation::Write(pointer), Operation::Read(output)] => {
                    assert_eq!(*pointer, [command]);
                    assert_eq!(output.len(), bytes.len());
                    output.copy_from_slice(&bytes);
                    Ok(())
                }
                _ => panic!("expected repeated-start command read"),
            },
            Expected::Write(bytes) => match operations {
                [Operation::Write(actual)] => {
                    assert_eq!(*actual, bytes);
                    Ok(())
                }
                _ => panic!("expected command write"),
            },
            Expected::Fail(command) => {
                match &operations[0] {
                    Operation::Write(bytes) => assert_eq!(bytes[0], command),
                    _ => panic!(),
                };
                Err(ErrorKind::ArbitrationLoss)
            }
        }
    }
}
fn read(command: u8, bytes: &[u8]) -> Expected {
    Expected::Read(command, bytes.to_vec())
}
fn write(bytes: &[u8]) -> Expected {
    Expected::Write(bytes.to_vec())
}
fn limits() -> ElectricalLimits {
    ElectricalLimits::new(3300, 28000, 5000, 140000).unwrap()
}

#[test]
fn commands_have_exact_width_and_little_endian_data_without_pec() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(0x06, &[0x10]),
            write(&[0x0c, 0x10, 0x27]),
            read(0x14, &[0x90, 1]),
        ]),
        Variant::Fa02,
    );
    driver.write_word(Register::Thermistor25, 10000).unwrap();
    assert_eq!(driver.read_word(Register::RequestedVoltage).unwrap(), 400);
    driver.release().done();
}

#[test]
fn invalid_register_length_reserved_bits_and_readonly_have_no_io() {
    let mut driver = Ap33772s::new(Mock::new(vec![]), Variant::Fa01);
    assert_eq!(
        driver.read_word(Register::SourcePdo(0)),
        Err(Error::InvalidRegister)
    );
    assert_eq!(
        driver.read_word(Register::OutputCurrent),
        Err(Error::InvalidLength)
    );
    assert_eq!(driver.write_byte(Register::Status, 0), Err(Error::ReadOnly));
    assert_eq!(driver.read_word(Register::PdRequest), Err(Error::WriteOnly));
    assert_eq!(
        driver.write_byte(Register::Configuration, 1),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        driver.write_byte(Register::System, 3),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        driver.write_byte(Register::UnderVoltageThreshold, 0),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        driver.write_byte(Register::Gpio, 0),
        Err(Error::Unsupported)
    );
    assert_eq!(driver.data_role_swap(), Err(Error::Unsupported));
    assert_eq!(
        driver.set_pd_features(true, true, true),
        Err(Error::Unsupported)
    );
    driver.release().done();
}

#[test]
fn interface_version_mismatch_prevents_write_and_bus_error_is_retained() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![read(0x06, &[0x20]), Expected::Fail(0x06)]),
        Variant::Fa02,
    );
    assert_eq!(
        driver.set_output_control(OutputControl::ForceOff),
        Err(Error::CommandVersion(2))
    );
    assert_eq!(
        driver.verify_interface(),
        Err(Error::Bus(ErrorKind::ArbitrationLoss))
    );
    driver.release().done();
}

#[test]
fn event_reads_are_explicitly_acknowledging_and_mode_read_is_separate() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(1, &[0x7f]),
            read(1, &[0]),
            read(3, &[0xe2]),
            read(0x33, &[4]),
        ]),
        Variant::Fa02,
    );
    let flags = driver.take_events().unwrap();
    assert!(flags.started() && flags.ready() && flags.new_capabilities() && flags.faulted());
    assert!(
        flags.under_voltage()
            && flags.over_voltage()
            && flags.over_current()
            && flags.over_temperature()
    );
    assert_eq!(driver.take_events().unwrap(), Events(0));
    let mode = driver.operating_mode().unwrap();
    assert!(mode.pd_connected() && mode.cc2_connected() && mode.derating() && mode.data_role_bit());
    assert!(!mode.legacy_connected());
    assert_eq!(driver.message_result().unwrap(), MessageResult::NoGoodCrc);
    driver.release().done();
}

#[test]
fn capability_summary_is_compressed_and_lossy_ranges_are_preserved() {
    let fixed = Capability::from_raw(4, 0xa0c8).unwrap();
    assert!(fixed.detected());
    assert_eq!(fixed.kind(), CapabilityKind::Fixed);
    assert_eq!(fixed.maximum_voltage_mv(), 20000);
    assert_eq!(fixed.current_range().at_least_ma, 3000);
    let pps = Capability::from_raw(5, 0xe16e).unwrap();
    assert_eq!(pps.kind(), CapabilityKind::Pps);
    assert_eq!(pps.maximum_voltage_mv(), 11000);
    assert_eq!(pps.minimum_voltage(), MinimumVoltage::Exact(3300));
    let lossy = Capability::from_raw(5, 0xe26e).unwrap();
    assert_eq!(
        lossy.minimum_voltage(),
        MinimumVoltage::Bounded {
            above_mv: 3300,
            at_most_mv: 5000
        }
    );
    assert_eq!(
        lossy.encode_request(Request::new(5, 4400, 3000).unwrap(), limits()),
        Err(RequestError::VoltageLimit)
    );
    let unknown = Capability::from_raw(5, 0xe36e).unwrap();
    assert_eq!(
        unknown.encode_request(Request::new(5, 5000, 3000).unwrap(), limits()),
        Err(RequestError::UnknownMinimum)
    );
    let zero_bin = Capability::from_raw(1, 0x8032).unwrap();
    assert_eq!(
        zero_bin.encode_request(Request::new(1, 5000, 1000).unwrap(), limits()),
        Err(RequestError::CurrentLimit)
    );
}

#[test]
fn requests_use_exact_fixed_pps_avs_units_and_avoid_maximum_escape() {
    let fixed = Capability::from_raw(4, 0xa0c8)
        .unwrap()
        .encode_request(Request::new(4, 20000, 3000).unwrap(), limits())
        .unwrap();
    assert_eq!(fixed.word(), 0x4800);
    let pps = Capability::from_raw(5, 0xe16e)
        .unwrap()
        .encode_request(Request::new(5, 11000, 3000).unwrap(), limits())
        .unwrap();
    assert_eq!(pps.word(), 0x586e);
    let avs = Capability::from_raw(8, 0xfd8c)
        .unwrap()
        .encode_request(Request::new(8, 20000, 5000).unwrap(), limits())
        .unwrap();
    assert_eq!(avs.word(), 0x8f64);
    assert_eq!(avs.requested().voltage_mv(), 20000);
    let max_fixed = Capability::from_raw(3, 0xbc96)
        .unwrap()
        .encode_request(Request::new(3, 15000, 5000).unwrap(), limits())
        .unwrap();
    assert_eq!(max_fixed.word(), 0x3f00);
    assert_ne!(max_fixed.word(), 0x3fff);
    assert_eq!(Request::new(1, 5000, 4750), Err(RequestError::CurrentStep));
    assert_eq!(
        Capability::from_raw(8, 0xfd8c)
            .unwrap()
            .encode_request(Request::new(8, 20100, 3000).unwrap(), limits()),
        Err(RequestError::VoltageStep)
    );
}

#[test]
fn capability_block_is_exactly26_bytes_and_individual_commands_differ() {
    let mut bytes = [0; 26];
    bytes[0..2].copy_from_slice(&0x9032u16.to_le_bytes());
    bytes[24..26].copy_from_slice(&0xfd8cu16.to_le_bytes());
    let mut driver = Ap33772s::new(
        Mock::new(vec![read(0x20, &bytes), read(0x2d, &[0x8c, 0xfd])]),
        Variant::Fa02,
    );
    let all = driver.capabilities().unwrap();
    assert_eq!(all[0].index(), 1);
    assert_eq!(all[12].index(), 13);
    assert_eq!(all[12].raw(), 0xfd8c);
    assert!(!all[1].detected());
    assert_eq!(driver.capability(13).unwrap(), all[12]);
    driver.release().done();
}

#[test]
fn typed_request_rechecks_pd_mode_capabilities_features_then_writes() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(3, &[2]),
            read(0x28, &[0x8c, 0xfd]),
            read(5, &[3]),
            read(6, &[0x10]),
            write(&[0x31, 0x64, 0x88]),
        ]),
        Variant::Fa02,
    );
    let submitted = driver
        .request(Request::new(8, 20000, 3000).unwrap(), limits())
        .unwrap();
    assert_eq!(submitted.word(), 0x8864);
    driver.release().done();
}

#[test]
fn unsafe_or_unadvertised_contracts_never_write() {
    let mut driver = Ap33772s::new(Mock::new(vec![]), Variant::Fa02);
    let low = ElectricalLimits::new(5000, 12000, 2000, 24000).unwrap();
    assert_eq!(
        driver.request(Request::new(4, 20000, 3000).unwrap(), low),
        Err(Error::Request(RequestError::ElectricalLimit))
    );
    driver.release().done();
    let mut driver = Ap33772s::new(Mock::new(vec![read(3, &[1])]), Variant::Fa02);
    assert_eq!(
        driver.request(Request::new(4, 20000, 3000).unwrap(), limits()),
        Err(Error::NotPdConnected)
    );
    driver.release().done();
    let mut driver = Ap33772s::new(
        Mock::new(vec![read(3, &[2]), read(0x24, &[0xc8, 0x90])]),
        Variant::Fa02,
    );
    assert_eq!(
        driver.request(Request::new(4, 20000, 3000).unwrap(), limits()),
        Err(Error::Request(RequestError::CurrentLimit))
    );
    driver.release().done();
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(3, &[2]),
            read(0x28, &[0x8c, 0xfd]),
            read(5, &[2]),
        ]),
        Variant::Fa02,
    );
    assert_eq!(
        driver.request(Request::new(8, 20000, 3000).unwrap(), limits()),
        Err(Error::Unsupported)
    );
    driver.release().done();
}

#[test]
fn measurements_use_distinct_command_widths_and_do_not_clear_status() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(0x11, &[250, 0]),
            read(0x12, &[125]),
            read(0x13, &[45]),
            read(0x14, &[0x90, 1]),
            read(0x15, &[0x2c, 1]),
        ]),
        Variant::Fa02,
    );
    assert_eq!(
        driver.measurements().unwrap(),
        Measurements {
            output_voltage_mv: 20000,
            output_current_ma: 3000,
            temperature_register: 45,
            requested_voltage_mv: 20000,
            requested_current_ma: 3000
        }
    );
    driver.release().done();
}

#[test]
fn ntc_and_threshold_sets_are_entirely_validated_before_io() {
    let mut driver = Ap33772s::new(Mock::new(vec![]), Variant::Fa02);
    assert_eq!(
        driver.configure_thermistor(ThermistorPoints {
            ohms_at25: 10000,
            ohms_at50: 4161,
            ohms_at75: 5000,
            ohms_at100: 974
        }),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        driver.configure_thresholds(Thresholds {
            minimum_selection_mv: 5100,
            under_voltage: UnderVoltage::Percent80,
            over_voltage_offset_mv: 2000,
            over_current_reference_ma: None,
            over_temperature_celsius: 120,
            derating_celsius: 100
        }),
        Err(Error::InvalidValue)
    );
    driver.release().done();
}

#[test]
fn fa02_specific_gpio_and_role_controls_preserve_protocol_version_readonly_bits() {
    let mut driver = Ap33772s::new(
        Mock::new(vec![
            read(6, &[0x10]),
            write(&[0x52, 0x83]),
            write(&[5, 7]),
            write(&[0x32, 2]),
            write(&[6, 1]),
        ]),
        Variant::Fa02,
    );
    driver.write_byte(Register::Gpio, 0x83).unwrap();
    driver.set_pd_features(true, true, true).unwrap();
    driver.data_role_swap().unwrap();
    driver.set_output_control(OutputControl::ForceOff).unwrap();
    driver.release().done();
}
