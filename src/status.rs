use core::cell::Cell;

use embassy_nrf::gpio::{Input, Output};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, Timer};

use crate::led::{LedPattern, Pattern};

#[derive(Clone, Copy, Default)]
pub struct GpsStatus {
    pub last_uart: Option<Instant>,
    pub last_pps: Option<Instant>,
    pub sat_count: u8,
    pub best_cno: u8,
    pub has_fix: bool,
}

static STATUS: Mutex<CriticalSectionRawMutex, Cell<GpsStatus>> =
    Mutex::new(Cell::new(GpsStatus {
        last_uart: None,
        last_pps: None,
        sat_count: 0,
        best_cno: 0,
        has_fix: false,
    }));

pub fn update<F: FnOnce(&mut GpsStatus)>(f: F) {
    STATUS.lock(|c| {
        let mut s = c.get();
        f(&mut s);
        c.set(s);
    });
}

// Red solid - no uart response
// red slow blink - uart alive no sat visible
// red fast blink - found sats but weak signal
// blue blink - strong sat visible no fix
// greed flash - got fix and pps working
// green solid - got fix but no pps
const NO_UART: LedPattern = LedPattern { rgb: (true,  false, false), pattern: Pattern::Solid };
const NO_SATS: LedPattern = LedPattern { rgb: (true,  false, false), pattern: Pattern::Blink { period_ms: 1000, duty_ms: 500 } };
const WEAK_SATS: LedPattern = LedPattern { rgb: (true,  false, false), pattern: Pattern::Blink { period_ms: 250,  duty_ms: 125 } };
const STRONG: LedPattern = LedPattern { rgb: (false, false, true ), pattern: Pattern::Blink { period_ms: 1000, duty_ms: 500 } };
const FIX_PPS: LedPattern = LedPattern { rgb: (false, true,  false), pattern: Pattern::PpsFlash };
const FIX_SOLID: LedPattern = LedPattern { rgb: (false, true,  false), pattern: Pattern::Solid };

fn classify(s: &GpsStatus, now: Instant) -> LedPattern {
    let within = |t: Option<Instant>, d| matches!(t, Some(t) if now.duration_since(t) < d);
    if !within(s.last_uart, Duration::from_secs(2)) { return NO_UART; }
    if s.has_fix {
        return if within(s.last_pps, Duration::from_millis(1500)) { FIX_PPS } else { FIX_SOLID };
    }
    if s.sat_count == 0 { return NO_SATS; }
    if s.best_cno < 30 { return WEAK_SATS; }
    STRONG
}

#[embassy_executor::task]
pub async fn status_led_task(
    mut r: Output<'static>,
    mut g: Output<'static>,
    mut b: Output<'static>,
) {
    loop {
        let now = Instant::now();
        let s = STATUS.lock(|c| c.get());
        let state = classify(&s, now);

        let pps_flash = matches!(s.last_pps, Some(t) if now.duration_since(t) < Duration::from_millis(80));
        let on = state.is_on(now.as_millis() as u32, pps_flash);
        state.apply(&mut r, &mut g, &mut b, on);

        Timer::after(Duration::from_millis(20)).await;
    }
}

#[embassy_executor::task]
pub async fn pps_task(mut pps: Input<'static>) {
    loop {
        pps.wait_for_rising_edge().await;
        update(|s| s.last_pps = Some(Instant::now()));
    }
}
