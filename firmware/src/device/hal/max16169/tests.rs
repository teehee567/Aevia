use super::*;
use core::cell::RefCell;
use embedded_hal::digital::{Error as _, ErrorKind, ErrorType};
use std::{rc::Rc, vec::Vec};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PinError {
    Output,
    Input,
}
impl embedded_hal::digital::Error for PinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}
#[derive(Default)]
struct State {
    events: Vec<&'static str>,
    fail_output: bool,
    fail_input: bool,
    interrupt_low: bool,
}
struct Clear(Rc<RefCell<State>>);
impl ErrorType for Clear {
    type Error = PinError;
}
impl OutputPin for Clear {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        let mut state = self.0.borrow_mut();
        state.events.push("clear_low");
        if state.fail_output {
            Err(PinError::Output)
        } else {
            Ok(())
        }
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        let mut state = self.0.borrow_mut();
        state.events.push("clear_high");
        if state.fail_output {
            Err(PinError::Output)
        } else {
            Ok(())
        }
    }
}
struct Interrupt(Rc<RefCell<State>>);
impl ErrorType for Interrupt {
    type Error = PinError;
}
impl InputPin for Interrupt {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|low| !low)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        let mut state = self.0.borrow_mut();
        state.events.push("interrupt_read");
        if state.fail_input {
            Err(PinError::Input)
        } else {
            Ok(state.interrupt_low)
        }
    }
}
fn device() -> (Max16169<Clear, Interrupt>, Rc<RefCell<State>>) {
    let state = Rc::new(RefCell::new(State::default()));
    let device = Max16169::v2_mini(Clear(state.clone()), Interrupt(state.clone()));
    (device, state)
}

#[test]
fn construction_and_ownership_release_never_change_power_pins() {
    let (device, state) = device();
    assert_eq!(device.variant(), Variant::AALTA);
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Unknown);
    let _pins = device.release();
    assert!(state.borrow().events.is_empty());
}

#[test]
fn shutdown_control_only_drives_active_low_clear_and_keeps_explicit_state() {
    let (mut device, state) = device();
    device.release_shutdown().unwrap();
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Released);
    device.hold_shutdown().unwrap();
    assert_eq!(device.shutdown_drive(), ShutdownDrive::HeldLow);
    assert_eq!(state.borrow().events, ["clear_high", "clear_low"]);
    device.release_shutdown().unwrap();
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Released);
}

#[test]
fn pin_failures_preserve_origin_and_remove_electrical_state_certainty() {
    let (mut device, state) = device();
    device.release_shutdown().unwrap();
    state.borrow_mut().fail_output = true;
    assert_eq!(
        device.hold_shutdown(),
        Err(Error::ClearPin(PinError::Output))
    );
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Unknown);
    assert_eq!(
        device.release_shutdown(),
        Err(Error::ClearPin(PinError::Output))
    );
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Unknown);
    state.borrow_mut().fail_input = true;
    assert_eq!(
        device.interrupt_asserted(),
        Err(Error::InterruptPin(PinError::Input))
    );
    assert_eq!(PinError::Input.kind(), ErrorKind::Other);
}

#[test]
fn interrupt_sensing_is_active_low_without_any_power_control_side_effect() {
    let (mut device, state) = device();
    assert!(!device.interrupt_asserted().unwrap());
    state.borrow_mut().interrupt_low = true;
    assert!(device.interrupt_asserted().unwrap());
    assert_eq!(state.borrow().events, ["interrupt_read", "interrupt_read"]);
    assert_eq!(device.shutdown_drive(), ShutdownDrive::Unknown);
}

#[test]
fn timing_metadata_matches_factory_options_and_tolerance() {
    let a = TimingOption::A.timing();
    assert_eq!(a.debounce.nominal_us, 50_000);
    assert_eq!(a.long_hold.nominal_us, 8_000_000);
    assert_eq!(
        a.switch_interrupt,
        TimingInterval {
            minimum_us: 25_600,
            nominal_us: 32_000,
            maximum_us: 38_400,
        }
    );
    assert_eq!(
        a.long_hold_interrupt,
        TimingInterval {
            minimum_us: 102_400,
            nominal_us: 128_000,
            maximum_us: 153_600,
        }
    );
    assert_eq!(
        a.clear_lockout,
        TimingInterval {
            minimum_us: 51_200,
            nominal_us: 64_000,
            maximum_us: 76_800,
        }
    );
    assert!(a.long_hold_deasserts_output);
    let b = TimingOption::B.timing();
    let c = TimingOption::C.timing();
    assert_eq!(b.debounce.nominal_us, 2_000_000);
    assert_eq!(c.debounce.nominal_us, 50_000);
    assert_eq!(b.long_hold.nominal_us, 16_000_000);
    assert_eq!(c.long_hold.nominal_us, 16_000_000);
    assert!(!b.long_hold_deasserts_output);
    assert!(!c.long_hold_deasserts_output);
}

#[test]
fn pulse_classification_uses_complete_tolerance_windows_and_rejects_clock_rollback() {
    let variant = Variant::AALTA;
    for width in [25_600, 32_000, 38_400] {
        assert_eq!(
            variant.classify_interrupt_us(width),
            InterruptPulse::DebouncedSwitch
        );
    }
    for width in [102_400, 128_000, 153_600] {
        assert_eq!(
            variant.classify_interrupt_us(width),
            InterruptPulse::LongHold
        );
    }
    for width in [0, 25_599, 38_401, 100_000, 153_601, u64::MAX] {
        assert_eq!(
            variant.classify_interrupt_us(width),
            InterruptPulse::Unrecognized
        );
    }
    assert_eq!(
        variant.classify_captured_interval_us(123, 32_123),
        Ok(InterruptPulse::DebouncedSwitch)
    );
    assert_eq!(
        variant.classify_captured_interval_us(200, 199),
        Err(TimestampError::NonMonotonic)
    );
}
