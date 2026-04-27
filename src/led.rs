use embassy_nrf::gpio::Output;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;


pub struct Led<P: OutputPin>(P);

impl<P: OutputPin> Led<P> {
    pub fn new(pin: P) -> Self {
        Self(pin)
    }

    pub fn on(&mut self) {
        self.0.set_low();
    }

    pub fn off(&mut self) {
        self.0.set_high();
    }

    pub async fn blink(&mut self, period: Duration) {
        loop {
            self.on();
            Timer::after(period).await;
            self.off();
            Timer::after(period).await;
        }
    }
}

#[embassy_executor::task(pool_size = 3)]
pub async fn blink(pin: Output<'static>, period: Duration) {
    Led::new(pin).blink(period).await;
}

#[derive(Clone, Copy)]
pub enum Pattern {
    Solid,
    Blink { period_ms: u32, duty_ms: u32 },
    PpsFlash,
}

#[derive(Clone, Copy)]
pub struct LedPattern {
    pub rgb: (bool, bool, bool),
    pub pattern: Pattern,
}

impl LedPattern {
    pub fn is_on(&self, now_ms: u32, pps_flash: bool) -> bool {
        match self.pattern {
            Pattern::Solid => true,
            Pattern::Blink { period_ms, duty_ms } => now_ms % period_ms < duty_ms,
            Pattern::PpsFlash => pps_flash,
        }
    }

    pub fn apply(
        &self,
        r: &mut Output<'static>,
        g: &mut Output<'static>,
        b: &mut Output<'static>,
        on: bool,
    ) {
        let (rd, gn, bl) = self.rgb;
        set(r, rd && on);
        set(g, gn && on);
        set(b, bl && on);
    }
}

fn set(p: &mut Output<'static>, on: bool) {
    if on { p.set_low() } else { p.set_high() }
}