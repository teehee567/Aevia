//! Conservative GPIO backlight control until the pinned S31 HAL supports PWM.
//! Reported settings are duty ceilings; scheduler delays reduce actual duty.
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{delay::Delay, gpio::Output};

static ALLOWED: AtomicBool = AtomicBool::new(false);
static REQUEST: AtomicU32 = AtomicU32::new(0);
static ACTIVE: AtomicU32 = AtomicU32::new(0);

pub fn allow(allowed: bool) {
    ALLOWED.store(allowed, Ordering::Relaxed);
}
pub fn request(duty: u8) {
    if matches!(duty, 0 | 1 | 5 | 10) {
        REQUEST.store(u32::from(duty), Ordering::Relaxed);
    }
}
pub fn duty() -> u32 {
    ACTIVE.load(Ordering::Relaxed)
}

#[embassy_executor::task]
pub async fn run(mut pin: Output<'static>) {
    let mut previous = 0;
    let mut started = Instant::now();
    loop {
        let requested = REQUEST.load(Ordering::Relaxed);
        if requested != previous {
            previous = requested;
            started = Instant::now();
        }
        let allowed = ALLOWED.load(Ordering::Relaxed);
        let expired = matches!(requested, 1 | 5) && started.elapsed() >= Duration::from_secs(5);
        let duty = if allowed && !expired { requested } else { 0 };
        ACTIVE.store(duty, Ordering::Relaxed);
        if duty != 0 {
            pin.set_high();
            Delay::new().delay_micros(duty * 10);
            pin.set_low();
        }
        Timer::after_micros(1000).await;
    }
}
