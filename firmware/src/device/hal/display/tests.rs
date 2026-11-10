use super::*;
use core::cell::RefCell;
use embedded_hal::{digital, spi};
use std::{rc::Rc, vec, vec::Vec};

#[derive(Clone, Debug, Eq, PartialEq)]
enum Event {
    Pin(&'static str, bool),
    Spi(Vec<u8>),
    Delay(u32),
}
#[derive(Default)]
struct State {
    events: Vec<Event>,
    spi_calls: usize,
    fail_spi: Option<usize>,
    fail_pin: Option<&'static str>,
}
struct MockSpi(Rc<RefCell<State>>);
impl spi::ErrorType for MockSpi {
    type Error = spi::ErrorKind;
}
impl SpiDevice for MockSpi {
    fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let mut state = self.0.borrow_mut();
        state.spi_calls += 1;
        let fail = state.fail_spi == Some(state.spi_calls);
        for operation in operations {
            if let spi::Operation::Write(bytes) = operation {
                state.events.push(Event::Spi(bytes.to_vec()));
            } else {
                panic!("unexpected SPI operation");
            }
        }
        if fail {
            Err(spi::ErrorKind::Other)
        } else {
            Ok(())
        }
    }
}
struct Pin(&'static str, Rc<RefCell<State>>);
impl digital::ErrorType for Pin {
    type Error = digital::ErrorKind;
}
impl OutputPin for Pin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set(true)
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set(false)
    }
}
impl Pin {
    fn set(&mut self, high: bool) -> Result<(), digital::ErrorKind> {
        let mut state = self.1.borrow_mut();
        state.events.push(Event::Pin(self.0, high));
        if state.fail_pin == Some(self.0) {
            Err(digital::ErrorKind::Other)
        } else {
            Ok(())
        }
    }
}
struct Delay(Rc<RefCell<State>>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().events.push(Event::Delay(ns));
    }
}
type Panel = St7789<MockSpi, Pin, Pin, Pin, Delay>;
fn panel(config: Config) -> (Panel, Rc<RefCell<State>>) {
    let state = Rc::new(RefCell::new(State::default()));
    let panel = St7789::new(
        MockSpi(state.clone()),
        Pin("dc", state.clone()),
        Pin("reset", state.clone()),
        Pin("backlight", state.clone()),
        Delay(state.clone()),
        config,
    );
    (panel, state)
}
fn initialized() -> (Panel, Rc<RefCell<State>>) {
    let (mut panel, state) = panel(Config::v2_mini());
    panel.initialize().unwrap();
    state.borrow_mut().events.clear();
    state.borrow_mut().spi_calls = 0;
    (panel, state)
}
fn writes(state: &Rc<RefCell<State>>) -> Vec<Vec<u8>> {
    state
        .borrow()
        .events
        .iter()
        .filter_map(|event| {
            if let Event::Spi(data) = event {
                Some(data.clone())
            } else {
                None
            }
        })
        .collect()
}

#[test]
fn constructor_is_inert_and_geometry_validation_does_not_overflow() {
    let (panel, state) = panel(Config::v2_mini());
    assert!(!panel.initialized());
    assert!(state.borrow().events.is_empty());
    assert_eq!(Geometry::new(0, 240, 0, 0), Err(GeometryError::Empty));
    assert_eq!(
        Geometry::new(241, 240, 0, 0),
        Err(GeometryError::OutOfBounds)
    );
    assert_eq!(
        Geometry::new(240, 240, u16::MAX, u16::MAX),
        Err(GeometryError::OutOfBounds)
    );
    assert_eq!(
        Geometry::new(240, 320, 0, 0).unwrap().size(Rotation::Deg90),
        (320, 240)
    );
}

#[test]
fn initialization_keeps_backlight_dark_and_uses_board_sequence() {
    let (mut panel, state) = panel(Config::v2_mini());
    panel.initialize().unwrap();
    assert!(panel.initialized());
    assert_eq!(
        writes(&state),
        vec![
            vec![1],
            vec![0x11],
            vec![0x3a],
            vec![0x55],
            vec![0x36],
            vec![0],
            vec![0x21],
            vec![0x13],
            vec![0x29]
        ]
    );
    let events = &state.borrow().events;
    assert_eq!(
        &events[..3],
        &[
            Event::Pin("backlight", false),
            Event::Pin("reset", false),
            Event::Delay(20_000_000)
        ]
    );
    assert!(!events.contains(&Event::Pin("backlight", true)));
    let delays: Vec<_> = events
        .iter()
        .filter_map(|e| {
            if let Event::Delay(ns) = e {
                Some(*ns)
            } else {
                None
            }
        })
        .collect();
    assert_eq!(
        delays,
        [
            20_000_000,
            120_000_000,
            150_000_000,
            120_000_000,
            10_000_000
        ]
    );
}

#[test]
fn visible_window_uses_row_eighty_and_pixels_are_big_endian() {
    let (mut panel, state) = initialized();
    panel
        .write_rect(Rect::new(1, 2, 2, 1), &[0xf800, 0x07e0])
        .unwrap();
    assert_eq!(
        writes(&state),
        vec![
            vec![0x2a],
            vec![0, 1, 0, 2],
            vec![0x2b],
            vec![0, 82, 0, 82],
            vec![0x2c],
            vec![0xf8, 0, 7, 0xe0]
        ]
    );
    assert_eq!(panel.remaining_pixels(), Some(0));
    let events = &state.borrow().events;
    for (i, event) in events.iter().enumerate() {
        if let Event::Spi(bytes) = event {
            let expected_data = bytes.len() != 1 || bytes[0] == 0xf8;
            assert_eq!(events[i - 1], Event::Pin("dc", expected_data));
        }
    }
}

#[test]
fn rotation_derives_crop_offsets_and_commits_only_successful_state() {
    let geometry = Geometry::new(135, 240, 52, 40).unwrap();
    assert_eq!(geometry.offsets(Rotation::Deg0), (52, 40));
    assert_eq!(geometry.offsets(Rotation::Deg90), (40, 53));
    assert_eq!(geometry.offsets(Rotation::Deg180), (53, 40));
    assert_eq!(geometry.offsets(Rotation::Deg270), (40, 52));
    let (mut panel, state) = initialized();
    panel.set_rotation(Rotation::Deg90).unwrap();
    assert_eq!(writes(&state), vec![vec![0x36], vec![0x60]]);
    state.borrow_mut().events.clear();
    panel.set_pixel(239, 239, 0xffff).unwrap();
    assert_eq!(writes(&state)[1], vec![1, 63, 1, 63]);
    assert_eq!(writes(&state)[3], vec![0, 239, 0, 239]);
    panel.set_color_order(ColorOrder::Bgr).unwrap();
    assert_eq!(writes(&state).last().unwrap(), &[0x68]);
}

#[test]
fn invalid_rectangles_and_pixel_counts_have_no_bus_side_effects() {
    let (mut panel, state) = initialized();
    assert_eq!(
        panel.write_rect(Rect::new(0, 0, 2, 1), &[0]),
        Err(Error::PixelCount)
    );
    assert_eq!(
        panel.fill_rect(Rect::new(u16::MAX, 0, 2, 1), 0),
        Err(Error::Geometry(GeometryError::OutOfBounds))
    );
    assert_eq!(
        panel.fill_rect(Rect::new(0, 0, 0, 1), 0),
        Err(Error::Geometry(GeometryError::Empty))
    );
    assert_eq!(
        panel.write_rect_bytes(Rect::new(0, 0, 1, 1), &[0]),
        Err(Error::PixelCount)
    );
    assert!(state.borrow().events.is_empty());
    panel.begin_window(Rect::new(0, 0, 1, 1)).unwrap();
    state.borrow_mut().events.clear();
    assert_eq!(panel.write_pixels(&[0, 1]), Err(Error::PixelCount));
    assert!(state.borrow().events.is_empty());
    assert_eq!(panel.remaining_pixels(), Some(1));
}

#[test]
fn pixel_streams_and_fill_are_bounded_to_sixty_four_data_bytes() {
    let (mut panel, state) = initialized();
    panel.begin_window(Rect::new(0, 0, 65, 1)).unwrap();
    state.borrow_mut().events.clear();
    panel.write_pixels(&[0x1234; 65]).unwrap();
    let data = writes(&state);
    assert_eq!(data.iter().map(Vec::len).collect::<Vec<_>>(), [64, 64, 2]);
    assert_eq!(&data[0][..4], &[0x12, 0x34, 0x12, 0x34]);
    assert_eq!(panel.remaining_pixels(), Some(0));
    state.borrow_mut().events.clear();
    panel.fill_rect(Rect::new(0, 0, 33, 1), 0xabcd).unwrap();
    let data = writes(&state);
    assert_eq!(data[data.len() - 2].len(), 64);
    assert_eq!(data.last().unwrap(), &[0xab, 0xcd]);
    state.borrow_mut().events.clear();
    panel
        .write_rect_bytes(Rect::new(0, 0, 2, 1), &[1, 2, 3, 4])
        .unwrap();
    assert_eq!(writes(&state).last().unwrap(), &[1, 2, 3, 4]);
}

#[test]
fn transport_failure_invalidates_unknown_state_and_stops_writes() {
    let (mut panel, state) = initialized();
    state.borrow_mut().fail_spi = Some(2);
    assert_eq!(
        panel.set_rotation(Rotation::Deg90),
        Err(Error::Spi(spi::ErrorKind::Other))
    );
    assert_eq!(panel.configuration().rotation, Rotation::Deg0);
    assert!(!panel.initialized());
    assert_eq!(panel.set_pixel(0, 0, 0), Err(Error::NotInitialized));
    assert_eq!(state.borrow().spi_calls, 2);
    state.borrow_mut().fail_spi = None;
    panel.initialize().unwrap();
    assert!(panel.initialized());
    panel.begin_window(Rect::new(0, 0, 65, 1)).unwrap();
    let next = state.borrow().spi_calls + 2;
    state.borrow_mut().fail_spi = Some(next);
    assert_eq!(
        panel.write_pixels(&[0; 65]),
        Err(Error::Spi(spi::ErrorKind::Other))
    );
    assert!(!panel.initialized());
    assert_eq!(panel.remaining_pixels(), None);
}

#[test]
fn control_pin_errors_preserve_which_pin_failed() {
    let (mut panel, state) = panel(Config::v2_mini());
    assert_eq!(panel.set_backlight(true), Err(Error::NotInitialized));
    state.borrow_mut().fail_pin = Some("reset");
    assert_eq!(
        panel.initialize(),
        Err(Error::Reset(digital::ErrorKind::Other))
    );
    assert!(writes(&state).is_empty());
    let (mut panel, state) = initialized();
    state.borrow_mut().fail_pin = Some("dc");
    assert_eq!(
        panel.set_pixel(0, 0, 0),
        Err(Error::DataCommand(digital::ErrorKind::Other))
    );
    assert!(!panel.initialized());
    assert!(writes(&state).is_empty());
    state.borrow_mut().fail_pin = Some("backlight");
    assert_eq!(
        panel.set_backlight(false),
        Err(Error::Backlight(digital::ErrorKind::Other))
    );
}

#[test]
fn sleep_wake_and_tearing_are_explicit_and_backlight_stays_off() {
    let (mut panel, state) = initialized();
    panel.set_backlight(true).unwrap();
    panel.sleep().unwrap();
    assert_eq!(panel.set_pixel(0, 0, 0), Err(Error::Sleeping));
    panel.wake().unwrap();
    panel.enable_tearing(true).unwrap();
    panel.enable_tearing(false).unwrap();
    assert_eq!(
        writes(&state),
        vec![vec![0x10], vec![0x11], vec![0x35], vec![0], vec![0x34]]
    );
    assert_eq!(
        state
            .borrow()
            .events
            .iter()
            .filter(|e| **e == Event::Pin("backlight", true))
            .count(),
        1
    );
    panel.set_display_enabled(false).unwrap();
    assert_eq!(writes(&state).last().unwrap(), &[0x28]);
}

#[test]
fn raw_commands_cancel_windows_and_geometry_changes_invalidate_typed_mode() {
    let (mut panel, _) = initialized();
    panel.begin_window(Rect::new(0, 0, 1, 1)).unwrap();
    panel.raw_command(0xe0, &[0; 14]).unwrap();
    assert!(panel.initialized());
    assert_eq!(panel.write_pixels(&[0]), Err(Error::NoWindow));
    panel.raw_command(0x3a, &[0x66]).unwrap();
    assert!(!panel.initialized());
    assert_eq!(panel.set_pixel(0, 0, 0), Err(Error::NotInitialized));
}
