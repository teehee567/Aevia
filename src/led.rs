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