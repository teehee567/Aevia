use super::*;
use core::cell::RefCell;
use std::{rc::Rc, vec::Vec};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PinFailure;
impl embedded_hal::digital::Error for PinFailure {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}
struct Pin {
    events: Rc<RefCell<Vec<bool>>>,
    high: bool,
    fail: bool,
}
impl Pin {
    fn new(events: Rc<RefCell<Vec<bool>>>) -> Self {
        Self {
            events,
            high: false,
            fail: false,
        }
    }
}
impl embedded_hal::digital::ErrorType for Pin {
    type Error = PinFailure;
}
impl OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.events.borrow_mut().push(false);
        self.high = false;
        if self.fail { Err(PinFailure) } else { Ok(()) }
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.events.borrow_mut().push(true);
        self.high = true;
        if self.fail { Err(PinFailure) } else { Ok(()) }
    }
}
impl InputPin for Pin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        if self.fail {
            Err(PinFailure)
        } else {
            Ok(self.high)
        }
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|high| !high)
    }
}

#[test]
fn constructors_are_inert_and_software_commands_start_unknown() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let enable = EnableControl::new(Part::Rpl3_0, Pin::new(events.clone()));
    let mode = Tps63802Mode::new(Pin::new(events.clone()));
    let pg = PowerGoodInput::new(Part::Rpl3_0, Pin::new(events.clone()));
    assert_eq!(enable.commanded_enabled(), None);
    assert_eq!(mode.commanded_mode(), None);
    assert_eq!(pg.part(), Part::Rpl3_0);
    assert!(events.borrow().is_empty());
}

#[test]
fn every_supported_enable_is_high_and_disable_is_low() {
    for part in [Part::Tps63802, Part::Tpsm863253, Part::Rpl3_0] {
        let events = Rc::new(RefCell::new(Vec::new()));
        let mut enable = EnableControl::new(part, Pin::new(events.clone()));
        enable.enable().unwrap();
        assert_eq!(enable.commanded_enabled(), Some(true));
        enable.disable().unwrap();
        assert_eq!(enable.commanded_enabled(), Some(false));
        assert_eq!(*events.borrow(), [true, false]);
        enable.invalidate_command();
        assert_eq!(enable.commanded_enabled(), None);
    }
}

#[test]
fn uncertain_gpio_failure_discards_previous_command_and_preserves_error() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut enable = EnableControl::new(Part::Tps63802, Pin::new(events.clone()));
    enable.enable().unwrap();
    enable.pin.fail = true;
    assert_eq!(enable.disable(), Err(Error::Pin(PinFailure)));
    assert_eq!(enable.commanded_enabled(), None);
    assert_eq!(*events.borrow(), [true, false]);
}

#[test]
fn mode_low_is_power_save_and_high_is_forced_pwm() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut mode = Tps63802Mode::new(Pin::new(events.clone()));
    mode.set_mode(SwitchingMode::PowerSave).unwrap();
    assert_eq!(mode.commanded_mode(), Some(SwitchingMode::PowerSave));
    mode.set_mode(SwitchingMode::ForcedPwm).unwrap();
    assert_eq!(*events.borrow(), [false, true]);
    mode.pin.fail = true;
    assert_eq!(
        mode.set_mode(SwitchingMode::PowerSave),
        Err(Error::Pin(PinFailure))
    );
    assert_eq!(mode.commanded_mode(), None);
}

#[test]
fn power_good_reads_signal_without_changing_enable() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut pg = PowerGoodInput::new(Part::Tpsm863253, Pin::new(events.clone()));
    assert_eq!(pg.read(), Ok(PowerGoodState::NotAsserted));
    pg.pin.high = true;
    assert_eq!(pg.read(), Ok(PowerGoodState::Asserted));
    pg.pin.fail = true;
    assert_eq!(pg.read(), Err(Error::Pin(PinFailure)));
    assert!(events.borrow().is_empty());
}

#[test]
fn local_recommended_ranges_do_not_confuse_startup_and_fixed_family_variant() {
    assert_eq!(
        Part::Tps63802.operating_limits(),
        OperatingLimits {
            input_min_mv: 1300,
            input_max_mv: 5500,
            output_min_mv: 1800,
            output_max_mv: 5200,
            startup_input_above_mv: Some(1800),
        }
    );
    let fixed = Part::Tpsm863253.operating_limits();
    assert_eq!((fixed.input_min_mv, fixed.input_max_mv), (3000, 17000));
    assert_eq!((fixed.output_min_mv, fixed.output_max_mv), (3300, 3300));
    let rpl = Part::Rpl3_0.operating_limits();
    assert_eq!((rpl.input_min_mv, rpl.input_max_mv), (4000, 18000));
    assert_eq!((rpl.output_min_mv, rpl.output_max_mv), (800, 5200));
    assert!(Part::Tps63802.has_mode_pin());
    assert!(!Part::Tpsm863253.has_mode_pin());
    assert!(!Part::Rpl3_0.has_mode_pin());
}

#[test]
fn divider_calculation_is_nominal_static_and_rejects_invalid_or_fixed_output() {
    // Actual Mini511k/91k network and RECOM's47k/15k3.3V application values.
    assert!(
        (nominal_divider_output_mv(Part::Tps63802, 511_000, 91_000).unwrap() - 3307.6923).abs()
            < 0.001
    );
    assert!(
        (nominal_divider_output_mv(Part::Rpl3_0, 47_000, 15_000).unwrap() - 3306.6667).abs()
            < 0.001
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Tpsm863253, 1000, 1000),
        Err(FeedbackError::FixedOutput)
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Tps63802, 1000, 0),
        Err(FeedbackError::InvalidResistance)
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Tps63802, 0, 1000),
        Err(FeedbackError::InvalidResistance)
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Tps63802, 1000, 1000),
        Err(FeedbackError::OutputOutsideRange)
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Tps63802, 511_000, 100_000),
        Err(FeedbackError::ResistanceOutsideRecommendation)
    );
    assert_eq!(
        nominal_divider_output_mv(Part::Rpl3_0, u32::MAX, 1),
        Err(FeedbackError::OutputOutsideRange)
    );
}
