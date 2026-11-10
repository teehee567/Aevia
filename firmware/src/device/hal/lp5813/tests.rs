use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
use std::{collections::VecDeque, vec, vec::Vec};

#[derive(Debug)]
enum Step {
    Write(u8, Vec<u8>),
    Read(u8, u8, Vec<u8>),
    Fail,
}
struct Bus {
    steps: VecDeque<Step>,
}
impl Drop for Bus {
    fn drop(&mut self) {
        assert!(
            self.steps.is_empty(),
            "unconsumed bus expectations: {:?}",
            self.steps
        );
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
        match self.steps.pop_front().expect("unexpected I2C transaction") {
            Step::Write(expected_address, expected) => {
                assert_eq!(address, expected_address);
                match operations {
                    [Operation::Write(bytes)] => assert_eq!(*bytes, expected),
                    _ => panic!("write transaction expected"),
                }
            }
            Step::Read(expected_address, register, expected) => {
                assert_eq!(address, expected_address);
                match operations {
                    [Operation::Write(pointer), Operation::Read(bytes)] => {
                        assert_eq!(*pointer, [register]);
                        assert_eq!(bytes.len(), expected.len());
                        bytes.copy_from_slice(&expected);
                    }
                    _ => panic!("combined write-read expected"),
                }
            }
            Step::Fail => return Err(ErrorKind::Other),
        }
        Ok(())
    }
}
fn make_device(steps: Vec<Step>) -> Lp5813<Bus> {
    Lp5813::new(
        Bus {
            steps: steps.into(),
        },
        Variant::A,
    )
}
struct Delay(u32);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0 += ns;
    }
}

#[test]
fn pages_are_part_of_i2c_address_and_blocks_split_at_boundary() {
    let mut device = make_device(vec![
        Step::Write(0x50, vec![0xff, 0x12]),
        Step::Write(0x51, vec![0, 0x34, 2, 0]),
        Step::Read(0x50, 0xff, vec![1]),
        Step::Read(0x51, 0, vec![2, 3]),
        Step::Read(0x53, 0, vec![0]),
    ]);
    device.write_registers(0xff, &[0x12, 0x34, 2, 0]).unwrap();
    let mut values = [0; 3];
    device.read_registers(0xff, &mut values).unwrap();
    assert_eq!(values, [1, 2, 3]);
    assert_eq!(device.read_register(0x300).unwrap(), 0);
}

#[test]
fn every_silicon_variant_has_four_distinct_page_addresses() {
    for (variant, base) in [
        (Variant::A, 0x50),
        (Variant::B, 0x54),
        (Variant::C, 0x58),
        (Variant::D, 0x5c),
    ] {
        let bus = Bus {
            steps: vec![Step::Read(base | 3, 0x1c, vec![0x3f])].into(),
        };
        let mut device = Lp5813::new(bus, variant);
        assert_eq!(device.read_register(0x31c).unwrap(), 0x3f);
    }
}

#[test]
fn entire_raw_range_and_reserved_bits_are_checked_before_io() {
    let mut device = make_device(vec![]);
    assert_eq!(
        device.write_registers(0x00d, &[0, 0]),
        Err(Error::InvalidRegister)
    );
    assert_eq!(device.write_register(0x300, 0), Err(Error::ReadOnly));
    assert_eq!(device.read_register(0x400), Err(Error::InvalidRegister));
    assert_eq!(device.write_register(0, 2), Err(Error::InvalidValue));
    assert_eq!(device.write_register(0x81, 0x40), Err(Error::InvalidValue));
    assert_eq!(device.write_register(0x21f, 4), Err(Error::InvalidValue));
}

#[test]
fn commands_and_fault_clear_use_individual_magic_values() {
    let mut device = make_device(vec![
        Step::Write(0x50, vec![0x10, 0x55]),
        Step::Write(0x50, vec![0x11, 0xff]),
        Step::Write(0x50, vec![0x12, 0xaa]),
        Step::Write(0x50, vec![0x13, 0x33]),
        Step::Write(0x50, vec![0x14, 0xcc]),
        Step::Write(0x50, vec![0x22, 5]),
        Step::Write(0x50, vec![0x23, 0x66]),
    ]);
    for command in [
        Command::Update,
        Command::Start,
        Command::Stop,
        Command::Pause,
        Command::Continue,
    ] {
        device.command(command).unwrap();
    }
    device.clear_faults(true, false, true).unwrap();
    let mut delay = Delay(0);
    device.software_reset(&mut delay).unwrap();
    assert_eq!(delay.0, 1_000_000);
}

#[test]
fn typed_config_has_correct_boost_shift_scan_order_and_masks() {
    let mut config = Config::v2_mini();
    config.maximum_current = MaximumCurrent::Milliamp51;
    config.autonomous_channels = Channel::A1.bit() | Channel::D2.bit();
    config.phase[Channel::D2.index() as usize] = Phase::Backward;
    let bytes = config.encode().unwrap();
    assert_eq!(bytes[0], 0x1f); // 4.5 V and 51 mA (datasheet example).
    assert_eq!(bytes[1], 0x40);
    assert_eq!(bytes[2], 0xe4);
    assert_eq!(&bytes[3..5], &[0x20, 0x80]);
    assert_eq!(bytes[10], 0xc0);
    assert_eq!(bytes[12], 0x0b);
    assert_eq!(MaximumCurrent::Milliamp25_5.dot_current(20_000), Ok(200));
    assert_eq!(
        MaximumCurrent::Milliamp25_5.dot_current(30_000),
        Err(InvalidConfig)
    );
    assert_eq!(
        MaximumCurrent::Milliamp51.dot_current(201),
        Err(InvalidConfig)
    );
}

#[test]
fn invalid_typed_config_has_no_delay_or_bus_effects() {
    let mut device = make_device(vec![]);
    let mut config = Config::v2_mini();
    config.scan_order = [0, 0, 2, 3];
    let mut delay = Delay(0);
    assert_eq!(
        device.initialize(&config, &mut delay),
        Err(Error::InvalidValue)
    );
    assert_eq!(delay.0, 0);
    config.scan_order = [0, 1, 2, 3];
    config.boost_millivolts = 3001;
    assert_eq!(device.configure(&config), Err(Error::InvalidValue));
    config.boost_millivolts = 4500;
    config.drive_mode = DriveMode::Mixed {
        count: 2,
        direct_outputs: 1,
    };
    assert_eq!(config.encode(), Err(InvalidConfig));
}

#[test]
fn initialize_applies_latch_checks_status_and_reads_back_without_enabling_leds() {
    let config = Config::v2_mini();
    let bytes = config.encode().unwrap();
    let mut packet = vec![1];
    packet.extend_from_slice(&bytes);
    let mut device = make_device(vec![
        Step::Write(0x50, vec![0, 1]),
        Step::Write(0x50, packet),
        Step::Write(0x50, vec![0x10, 0x55]),
        Step::Read(0x53, 0, vec![0]),
        Step::Read(0x50, 1, bytes.to_vec()),
    ]);
    let mut delay = Delay(0);
    device.initialize(&config, &mut delay).unwrap();
    assert_eq!(delay.0, 1_000_000);
}

#[test]
fn status_faults_and_verification_errors_are_observable() {
    let mut device = make_device(vec![Step::Read(0x53, 0, vec![3, 0xf0, 0xff, 0xa5, 0x50])]);
    assert_eq!(
        device.faults().unwrap(),
        Faults {
            configuration_error: true,
            thermal_shutdown: true,
            open_channels: 0xfff0,
            short_channels: 0x50a5
        }
    );
    let config = Config::default();
    let mut expected = config.encode().unwrap();
    expected[2] = 0;
    let mut device = make_device(vec![Step::Read(0x50, 1, expected.to_vec())]);
    assert_eq!(
        device.verify_configuration(&config),
        Err(Error::Verification {
            register: 3,
            expected: 0xe4,
            actual: 0
        })
    );
}

#[test]
fn animation_encoder_preserves_start_end_slopes_and_playbacks() {
    let mut animation = Animation {
        active_units: 3,
        playback: 15,
        pause_start: AnimationTime::Ms90,
        pause_end: AnimationTime::Ms8050,
        ..Animation::default()
    };
    animation.units[0] = AnimationUnit {
        pwm: [1, 2, 3, 4, 5],
        slopes: [
            AnimationTime::Ms90,
            AnimationTime::Ms180,
            AnimationTime::Ms360,
            AnimationTime::Ms540,
        ],
        playback: UnitPlayback::Infinite,
    };
    let bytes = animation.encode().unwrap();
    assert_eq!(&bytes[..10], &[0x1f, 0x2f, 1, 2, 3, 4, 5, 0x21, 0x43, 3]);
    let mut device = make_device(vec![Step::Write(
        0x52,
        vec![
            0x06, 0x1f, 0x2f, 1, 2, 3, 4, 5, 0x21, 0x43, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0,
        ],
    )]);
    assert_eq!(Channel::D2.animation_base(), 0x206);
    device.set_animation(Channel::D2, &animation).unwrap();
    animation.playback = 16;
    assert_eq!(
        device.set_animation(Channel::D2, &animation),
        Err(Error::InvalidValue)
    );
}

#[test]
fn fitted_rgb_wiring_and_transport_error_are_preserved() {
    let mut device = make_device(vec![
        Step::Write(0x50, vec![0x4b, 10]),
        Step::Write(0x50, vec![0x4a, 20]),
        Step::Write(0x50, vec![0x4c, 30]),
        Step::Fail,
    ]);
    device.set_mini_rgb(MiniLed::D33, [10, 20, 30]).unwrap();
    assert_eq!(
        device.set_enabled(0xffff),
        Err(Error::Bus(ErrorKind::Other))
    );
}
