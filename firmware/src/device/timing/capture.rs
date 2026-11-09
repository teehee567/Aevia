//! S31 MCPWM0 capture adapter; no interrupt/task timestamp is used for an edge.
//!
//! Register sequence follows Espressif's ESP32-S31 mcpwm_ll.h and
//! gpio_sig_map.h. esp-hal 1.2.2 has no S31 MCPWM driver. This module exclusively
//! reserves MCPWM0 and GPIO matrix input slots 95/96; no PWM outputs are enabled.

use super::counter::{Counter, TICK_NS};
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;
use esp32s31::{GPIO, HP_SYS_CLKRST, MCPWM0};

static STARTED: AtomicBool = AtomicBool::new(false);
pub(super) static ERRORS: AtomicU32 = AtomicU32::new(0);
static CLOCK: Mutex<CriticalSectionRawMutex, RefCell<Option<CaptureClock>>> =
    Mutex::new(RefCell::new(None));

struct CaptureClock {
    counter: Counter,
    origin_ns: u64,
}

fn registers() -> &'static esp32s31::mcpwm0::RegisterBlock {
    // SAFETY: this adapter is the sole MCPWM0 owner; accesses are volatile.
    unsafe { &*MCPWM0::ptr() }
}

/// Call once after configuring GPIO42/PPS and GPIO14/DRDY as digital inputs.
pub fn init() {
    assert!(
        !STARTED.swap(true, Ordering::AcqRel),
        "capture already initialized"
    );
    // SAFETY: called once before capture consumers start. Only MCPWM0's clock
    // control and the two exclusively reserved matrix input slots are changed.
    unsafe {
        let clock = &*HP_SYS_CLKRST::ptr();
        clock
            .mcpwm0_ctrl0()
            .modify(|_, w| w.apb_clk_en().set_bit().rst_en().set_bit());
        clock.mcpwm0_ctrl0().modify(|_, w| {
            w.rst_en()
                .clear_bit()
                .clk_src_sel()
                .bits(0)
                .clk_div_num()
                .bits(3)
                .clk_en()
                .set_bit()
        });
        let gpio = &*GPIO::ptr();
        for (signal, pin) in [(95, 42), (96, 14)] {
            gpio.func_in_sel_cfg(signal).write(|w| {
                w.in_sel()
                    .bits(pin)
                    .in_inv_sel()
                    .clear_bit()
                    .sel()
                    .set_bit()
            });
        }
        let registers = registers();
        registers.clk().write(|w| w.en().set_bit());
        registers.int_ena().write(|w| w.bits(0));
        registers.int_clr().write(|w| w.bits(u32::MAX));
        for channel in 0..2 {
            registers.cap_ch_cfg(channel).write(|w| {
                w.cap_en()
                    .set_bit()
                    .cap_mode()
                    .bits(2)
                    .cap_prescale()
                    .bits(0)
            });
        }
        registers.cap_ch_cfg(2).write(|w| w.cap_en().set_bit());
        registers
            .cap_timer_cfg()
            .write(|w| w.cap_timer_en().set_bit());
    }
    CLOCK.lock(|clock| {
        let before = Instant::now().as_micros();
        let raw = soft_capture();
        let after = Instant::now().as_micros();
        let origin_ns = (before + (after - before) / 2) * 1000 - u64::from(raw) * TICK_NS;
        *clock.borrow_mut() = Some(CaptureClock {
            counter: Counter::new(raw, after),
            origin_ns,
        });
    });
}

fn soft_capture() -> u32 {
    let registers = registers();
    registers.cap_ch_cfg(2).modify(|_, w| w.cap_sw().set_bit());
    registers.cap_ch(2).read().bits()
}

/// Extends the counter every <=250ms, including while PPS/DRDY are absent.
/// After an ambiguous outage (>half a wrap), fail closed until reboot.
fn read(channel: Option<usize>) -> Option<u64> {
    CLOCK.lock(|clock| {
        let mut clock = clock.borrow_mut();
        let capture = clock.as_mut()?;
        let mut ticks = capture
            .counter
            .advance(soft_capture(), Instant::now().as_micros());
        if ticks.is_none() {
            ERRORS.fetch_add(1, Ordering::Relaxed);
            *clock = None;
            return None;
        }
        if let Some(channel) = channel {
            let raw = registers().cap_ch(channel).read().bits();
            // A zero latch is the reset value, before the first actual edge.
            if raw == 0 {
                return None;
            }
            // Read soft 'now' again: an edge can occur between the first soft
            // capture and reading its latch. Never interpret that as a wrap.
            let now = soft_capture();
            if capture
                .counter
                .advance(now, Instant::now().as_micros())
                .is_none()
            {
                ERRORS.fetch_add(1, Ordering::Relaxed);
                *clock = None;
                return None;
            }
            ticks = capture.counter.captured(raw);
        }
        ticks?.checked_mul(TICK_NS)?.checked_add(capture.origin_ns)
    })
}

pub fn now_ns() -> Option<u64> {
    read(None)
}
pub fn pps_ns() -> Option<u64> {
    read(Some(0))
}
pub fn drdy_ns() -> Option<u64> {
    read(Some(1))
}
