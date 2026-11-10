use super::*;
use embedded_hal::i2c::{self, Operation};
use std::{collections::VecDeque, vec, vec::Vec};

#[derive(Debug)]
struct Step {
    written: Vec<u8>,
    reply: Option<Vec<u8>>,
    error: Option<i2c::ErrorKind>,
}
impl Step {
    fn read(address: u8, reply: &[u8]) -> Self {
        Self {
            written: vec![address],
            reply: Some(reply.to_vec()),
            error: None,
        }
    }
    fn write(bytes: &[u8]) -> Self {
        Self {
            written: bytes.to_vec(),
            reply: None,
            error: None,
        }
    }
    fn fail(bytes: &[u8], error: i2c::ErrorKind) -> Self {
        Self {
            written: bytes.to_vec(),
            reply: None,
            error: Some(error),
        }
    }
}
struct Bus {
    steps: VecDeque<Step>,
}
impl Bus {
    fn new(steps: impl IntoIterator<Item = Step>) -> Self {
        Self {
            steps: steps.into_iter().collect(),
        }
    }
    fn done(self) {
        assert!(
            self.steps.is_empty(),
            "remaining transactions: {:?}",
            self.steps
        );
    }
}
impl i2c::ErrorType for Bus {
    type Error = i2c::ErrorKind;
}
impl I2c for Bus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(address, ADDRESS);
        let step = self.steps.pop_front().expect("unexpected transaction");
        let written = match &operations[0] {
            Operation::Write(bytes) => *bytes,
            _ => panic!("expected write"),
        };
        assert_eq!(written, step.written);
        if let Some(error) = step.error {
            return Err(error);
        }
        if let Some(reply) = step.reply {
            assert_eq!(operations.len(), 2);
            match &mut operations[1] {
                Operation::Read(bytes) => {
                    assert_eq!(bytes.len(), reply.len());
                    bytes.copy_from_slice(&reply);
                }
                _ => panic!("expected read"),
            }
        } else {
            assert_eq!(operations.len(), 1);
        }
        Ok(())
    }
}

#[test]
fn measurements_are_word_big_endian_and_scale_by_variant() {
    for (variant, expected_uv) in [
        (Variant::Max17048, 4_000_000),
        (Variant::Max17049, 8_000_000),
    ] {
        let bus = Bus::new([
            Step::read(0x02, &[0xc8, 0x00, 50, 128]),
            Step::read(0x16, &[0xff, 0xfe]),
        ]);
        let mut gauge = Max17048::new(bus, variant);
        let measurement = gauge.measurements().unwrap();
        assert_eq!(measurement.voltage_microvolts(), expected_uv);
        assert_eq!(measurement.state_of_charge_percent(), 50.5);
        assert_eq!(measurement.charge_rate_millipercent_per_hour(), -416);
        gauge.release().done();
    }
}

#[test]
fn rcomp_preserves_observed_configuration_and_alert() {
    let bus = Bus::new([
        Step::read(0x0c, &[0x97, 0xfd]),
        Step::write(&[0x0c, 0x80, 0xfd]),
        Step::read(0x0c, &[0x80, 0xfd]),
        Step::write(&[0x0c, 0x80, 0xe0]),
        Step::read(0x0c, &[0x80, 0xe0]),
        Step::write(&[0x0c, 0x80, 0xc0]),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    gauge.set_rcomp(128).unwrap();
    gauge.set_low_soc_alert(32).unwrap();
    gauge.clear_alert_latch().unwrap();
    gauge.release().done();
}

#[test]
fn flag_clear_preserves_other_observed_causes_and_reset_enable() {
    let bus = Bus::new([
        Step::read(0x1a, &[0x65, 0xaa]),
        Step::write(&[0x1a, 0x45, 0x00]),
        Step::read(0x1a, &[0x45, 0xaa]),
        Step::write(&[0x1a, 0x05, 0x00]),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    gauge.clear_alert_flags(AlertFlags::SOC_CHANGE).unwrap();
    gauge.set_reset_alert(false).unwrap();
    assert!(
        Status { raw: 0x6500 }
            .flags()
            .contains(AlertFlags::SOC_CHANGE.union(AlertFlags::VOLTAGE_LOW))
    );
    gauge.release().done();
}

#[test]
fn invalid_thresholds_and_nonfinite_compensation_do_no_io() {
    let mut gauge = Max17048::new(Bus::new([]), Variant::Max17048);
    assert_eq!(gauge.set_low_soc_alert(0), Err(Error::InvalidValue));
    assert_eq!(gauge.set_low_soc_alert(33), Err(Error::InvalidValue));
    assert_eq!(
        gauge.set_voltage_alerts_register_mv(4000, 3000),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        gauge.set_voltage_alerts_register_mv(3001, 4400),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        gauge.set_voltage_alerts_register_mv(3000, 5120),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        gauge.set_reset_voltage_register_mv(2500, false),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        gauge.set_reset_voltage_register_mv(2240, false),
        Err(Error::InvalidValue)
    );
    let calibration = RcompCalibration {
        at_20_celsius: 100.0,
        counts_per_celsius_above_20: -1.0,
        counts_per_celsius_below_20: -3.0,
    };
    assert_eq!(
        gauge.compensate_temperature(calibration, f32::NAN),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        gauge.write_register(Register::CellVoltage, 0),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        gauge.write_register(Register::Mode, 0x1000),
        Err(Error::InvalidValue)
    );
    gauge.release().done();
}

#[test]
fn threshold_encodings_and_factory_id_preservation_are_exact() {
    let bus = Bus::new([
        Step::write(&[0x14, 150, 220]),
        Step::read(0x18, &[0x96, 0xa5]),
        Step::write(&[0x18, 0x7d, 0xa5]),
        Step::write(&[0x0a, 0x80, 0x30]),
        Step::write(&[0x0a, 0, 0]),
        Step::write(&[0x0a, 0xff, 0xff]),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    gauge.set_voltage_alerts_register_mv(3000, 4400).unwrap();
    gauge.set_reset_voltage_register_mv(2480, true).unwrap();
    let thresholds = HibernateThresholds {
        rate_code: 128,
        voltage_code: 48,
    };
    assert_eq!(thresholds.rate_millipercent_per_hour(), 26_624);
    assert_eq!(thresholds.voltage_microvolts(), 60_000);
    gauge.set_hibernate_thresholds(thresholds).unwrap();
    gauge.disable_hibernate().unwrap();
    gauge.force_hibernate().unwrap();
    gauge.release().done();
}

#[test]
fn sleep_and_quick_start_never_replay_read_only_or_action_bits() {
    let bus = Bus::new([
        Step::write(&[0x06, 0x20, 0]),
        Step::read(0x0c, &[0x97, 0x3c]),
        Step::write(&[0x0c, 0x97, 0xbc]),
        Step::read(0x0c, &[0x97, 0xbc]),
        Step::write(&[0x0c, 0x97, 0x3c]),
        Step::read(0x06, &[0x70, 0xff]),
        Step::write(&[0x06, 0x60, 0]),
        Step::write(&[0x06, 0, 0]),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    gauge.sleep().unwrap();
    gauge.wake().unwrap();
    gauge.quick_start().unwrap();
    gauge.allow_sleep(false).unwrap();
    gauge.release().done();
}

#[test]
fn model_write_uses_all_32_words_and_relocks() {
    let table: [u8; 64] = core::array::from_fn(|i| i as u8);
    let mut steps = vec![Step::write(&[0x3e, 0x4a, 0x57])];
    for (index, word) in table.as_chunks::<2>().0.iter().enumerate() {
        steps.push(Step::write(&[0x40 + index as u8 * 2, word[0], word[1]]));
    }
    steps.push(Step::write(&[0x3e, 0, 0]));
    let mut gauge = Max17048::new(Bus::new(steps), Variant::Max17048);
    gauge.load_model(&table).unwrap();
    gauge.release().done();
}

#[test]
fn model_write_failure_still_attempts_relock_and_retains_both_errors() {
    let bus = Bus::new([
        Step::write(&[0x3e, 0x4a, 0x57]),
        Step::fail(&[0x40, 0, 0], i2c::ErrorKind::Bus),
        Step::fail(&[0x3e, 0, 0], i2c::ErrorKind::ArbitrationLoss),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    assert_eq!(
        gauge.load_model(&[0; 64]),
        Err(Error::ModelLoad {
            operation: Some(i2c::ErrorKind::Bus),
            relock: Some(i2c::ErrorKind::ArbitrationLoss),
        })
    );
    gauge.release().done();
}

#[test]
fn por_accepts_only_documented_data_nack_and_preserves_address_errors() {
    let bus = Bus::new([
        Step::fail(
            &[0xfe, 0x54, 0],
            i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
        ),
        Step::fail(
            &[0xfe, 0x54, 0],
            i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
        ),
        Step::fail(
            &[0xfe, 0x54, 0],
            i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
        ),
    ]);
    let mut gauge = Max17048::new(bus, Variant::Max17048);
    assert_eq!(gauge.reset(), Ok(ResetOutcome::ExpectedDataNack));
    assert_eq!(
        gauge.reset(),
        Err(Error::Bus(i2c::ErrorKind::NoAcknowledge(
            NoAcknowledgeSource::Address
        )))
    );
    assert_eq!(
        gauge.reset(),
        Err(Error::Bus(i2c::ErrorKind::NoAcknowledge(
            NoAcknowledgeSource::Unknown
        )))
    );
    gauge.release().done();
}

#[test]
fn compensation_is_piecewise_saturated_and_never_assumes_a_battery_model() {
    let calibration = RcompCalibration {
        at_20_celsius: 100.0,
        counts_per_celsius_above_20: -1.0,
        counts_per_celsius_below_20: -3.0,
    };
    assert_eq!(calibration.at_temperature(20.0), Some(100));
    assert_eq!(calibration.at_temperature(30.0), Some(90));
    assert_eq!(calibration.at_temperature(10.0), Some(130));
    assert_eq!(calibration.at_temperature(200.0), Some(0));
    assert_eq!(calibration.at_temperature(-100.0), Some(255));
    assert_eq!(calibration.at_temperature(f32::INFINITY), None);
    assert_eq!(
        RcompCalibration {
            at_20_celsius: -1.0,
            ..calibration
        }
        .at_temperature(20.0),
        None
    );
}
