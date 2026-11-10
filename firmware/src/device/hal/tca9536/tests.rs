use super::*;
use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
use std::{collections::VecDeque, vec, vec::Vec};

#[derive(Debug)]
enum Expected {
    Read(u8, u8, u8),
    Write(u8, Vec<u8>),
    Fail(u8, Vec<u8>),
}
struct Bus(VecDeque<Expected>);
impl Bus {
    fn new(steps: impl IntoIterator<Item = Expected>) -> Self {
        Self(steps.into_iter().collect())
    }
    fn done(&self) {
        assert!(self.0.is_empty(), "unconsumed bus operations: {:?}", self.0);
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
        match self.0.pop_front().expect("unexpected I2C operation") {
            Expected::Read(expected_address, register, value) => {
                assert_eq!(address, expected_address);
                match operations {
                    [Operation::Write(command), Operation::Read(bytes)] => {
                        assert_eq!(*command, [register]);
                        assert_eq!(bytes.len(), 1);
                        bytes[0] = value;
                    }
                    _ => panic!("read must preserve repeated START"),
                }
                Ok(())
            }
            Expected::Write(expected_address, bytes) => {
                assert_eq!(address, expected_address);
                match operations {
                    [Operation::Write(actual)] => assert_eq!(*actual, bytes),
                    _ => panic!("write must be one transaction"),
                }
                Ok(())
            }
            Expected::Fail(expected_address, bytes) => {
                assert_eq!(address, expected_address);
                match operations {
                    [Operation::Write(actual)] => assert_eq!(*actual, bytes),
                    _ => panic!("unexpected transaction"),
                }
                Err(ErrorKind::ArbitrationLoss)
            }
        }
    }
}

fn write(register: u8, value: u8) -> Expected {
    Expected::Write(0x40, vec![register, value])
}
fn read(register: u8, value: u8) -> Expected {
    Expected::Read(0x40, register, value)
}

#[test]
fn all_four_variants_use_their_documented_addresses() {
    for (variant, address) in [
        (Variant::Tca9536, 0x41),
        (Variant::Tca9536A, 0x40),
        (Variant::Tca9536B, 0x43),
        (Variant::Tca9536C, 0x42),
    ] {
        let mut driver = Tca9536::new(Bus::new([Expected::Read(address, 0, 0xf5)]), variant);
        assert_eq!(driver.input_levels().unwrap(), 0x05);
        driver.release().done();
    }
}

#[test]
fn invalid_masks_and_readonly_writes_never_touch_bus() {
    let mut driver = Tca9536::v2_mini(Bus::new([]));
    assert_eq!(driver.set_outputs(0x10), Err(Error::InvalidValue));
    assert_eq!(driver.set_input_directions(0xff), Err(Error::InvalidValue));
    assert_eq!(driver.set_polarity(0x80), Err(Error::InvalidValue));
    assert_eq!(driver.update_outputs(1, 2), Err(Error::InvalidValue));
    assert_eq!(
        driver.write_register(Register::SpecialFunction, 1),
        Err(Error::InvalidValue)
    );
    assert_eq!(
        driver.write_register(Register::Input, 0),
        Err(Error::ReadOnly)
    );
    assert_eq!(
        driver.configure(Configuration {
            inputs: 0,
            special: SpecialFunction {
                p3: P3Function::Interrupt,
                pullups_enabled: true,
            },
            ..Configuration::default()
        }),
        Err(Error::InvalidValue)
    );
    driver.release().done();
}

#[test]
fn configuration_preloads_latches_before_output_enable() {
    let mut driver = Tca9536::v2_mini(Bus::new([
        write(3, 0xff),
        write(1, 0xf1),
        write(2, 4),
        write(0x50, 0),
        write(3, 0xfc),
    ]));
    driver
        .configure(Configuration {
            inputs: 0x0c,
            output_high: 1,
            inverted_inputs: 4,
            special: SpecialFunction::default(),
        })
        .unwrap();
    driver.release().done();
}

#[test]
fn partial_output_configuration_preserves_unselected_latches_and_directions() {
    let mut driver = Tca9536::v2_mini(Bus::new([
        read(3, 0xff),
        read(1, 0xfe),
        write(1, 0xfd),
        write(3, 0xfc),
    ]));
    driver.configure_outputs(3, 1).unwrap();
    driver.release().done();

    let mut driver = Tca9536::v2_mini(Bus::new([read(0x50, 0x80)]));
    assert_eq!(driver.configure_outputs(8, 0), Err(Error::P3IsInterrupt));
    driver.release().done();
}

#[test]
fn special_functions_use_bits_seven_and_six_and_general_call_is_explicit() {
    let mut driver = Tca9536::v2_mini(Bus::new([write(0x50, 0xc0), Expected::Write(0, vec![6])]));
    driver
        .set_special_function(SpecialFunction {
            p3: P3Function::Interrupt,
            pullups_enabled: false,
        })
        .unwrap();
    driver.reset_all_general_call_devices().unwrap();
    driver.release().done();
}

#[test]
fn button_setup_retains_p3_as_gpio_and_maps_active_low_levels() {
    let mut driver = Tca9536::v2_mini(Bus::new([
        write(3, 0xff),
        write(1, 0xff),
        write(2, 0),
        write(0x50, 0),
        write(3, 0xff),
        read(3, 0xff),
        read(2, 0),
        read(0x50, 0),
        read(0, 0xf6),
    ]));
    driver.configure_buttons().unwrap();
    let buttons = driver.buttons().unwrap();
    assert_eq!(buttons.pressed_mask(), 9);
    assert!(buttons.is_pressed(Pin::P0));
    assert!(buttons.is_pressed(Pin::P3));
    assert!(!buttons.is_pressed(Pin::P1));
    driver.release().done();

    let mut driver = Tca9536::v2_mini(Bus::new([read(3, 0xff), read(2, 0), read(0x50, 0x80)]));
    assert_eq!(driver.buttons(), Err(Error::ButtonsNotConfigured));
    driver.release().done();
}

#[test]
fn bus_failure_is_preserved_and_configuration_stops() {
    let mut driver = Tca9536::v2_mini(Bus::new([
        write(3, 0xff),
        Expected::Fail(0x40, vec![1, 0xff]),
    ]));
    assert_eq!(
        driver.configure_buttons(),
        Err(Error::Bus(ErrorKind::ArbitrationLoss))
    );
    driver.release().done();
}

fn pressed(mask: u8) -> Buttons {
    Buttons::from_pressed_mask(mask).unwrap()
}

#[test]
fn debounce_tracks_independent_candidates_and_restarts_on_bounce() {
    let mut filter = ButtonDebouncer::new(10, Buttons::NONE);
    assert_eq!(filter.update(100, pressed(1)).pressed, Buttons::NONE);
    assert_eq!(filter.update(105, pressed(3)).pressed, Buttons::NONE);
    assert_eq!(filter.update(110, pressed(3)).pressed, pressed(1));
    assert_eq!(filter.update(112, pressed(1)).state, pressed(1));
    assert_eq!(filter.update(114, pressed(3)).pressed, Buttons::NONE);
    assert_eq!(filter.update(124, pressed(3)).pressed, pressed(2));
    assert_eq!(filter.update(125, pressed(2)).released, Buttons::NONE);
    assert_eq!(filter.update(135, pressed(2)).released, pressed(1));
    assert_eq!(filter.state(), pressed(2));
}

#[test]
fn rollback_restarts_pending_intervals_without_edges() {
    let mut filter = ButtonDebouncer::new(10, Buttons::NONE);
    filter.update(1000, pressed(8));
    assert_eq!(filter.update(5, pressed(8)).state, Buttons::NONE);
    assert_eq!(filter.update(14, pressed(8)).pressed, Buttons::NONE);
    assert_eq!(filter.update(15, pressed(8)).pressed, pressed(8));
    assert_eq!(filter.update(15, pressed(8)).pressed, Buttons::NONE);
    let mut immediate = ButtonDebouncer::new(0, Buttons::NONE);
    assert_eq!(immediate.update(0, pressed(15)).pressed, pressed(15));
    assert!(Buttons::from_pressed_mask(16).is_none());
}
