//! Sole owner of power I2C: battery monitor, indicator and UI buttons.
use aevia_firmware::battery::*;
use core::cell::Cell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Instant, Timer};
use esp_hal::{
    Async,
    gpio::{Input, Output},
    i2c::master::{Error, I2c},
};

static SNAPSHOT: Mutex<CriticalSectionRawMutex, Cell<Snapshot>> =
    Mutex::new(Cell::new(Snapshot::new()));

pub fn snapshot() -> Snapshot {
    SNAPSHOT.lock(Cell::get).fresh(Instant::now().as_millis())
}

struct Bus(I2c<'static, Async>);

impl Registers for Bus {
    async fn read<const N: usize>(
        &mut self,
        address: u8,
        register: u8,
    ) -> Result<[u8; N], &'static str> {
        let mut bytes = [0; N];
        self.0
            .write_read_async(address, &[register], &mut bytes)
            .await
            .map_err(|error| match error {
                Error::AcknowledgeCheckFailed(_) => "nack",
                Error::Timeout => "timeout",
                Error::ArbitrationLost => "arbitration",
                _ => "i2c-error",
            })?;
        Ok(bytes)
    }
}

#[embassy_executor::task]
pub async fn task(bus: I2c<'static, Async>, enable: Output<'static>, power_good_n: Input<'static>) {
    // This task is the only subsequent bus owner; LED recovery never
    // sleeps through a battery poll or requires a second bus lock.
    let mut bus = Bus(bus);
    let mut monitor = Monitor::new();
    let mut buttons = crate::drivers::tca9536::Buttons::new();
    let mut next_battery_ms = 0;
    let mut detected = None;
    let mut led = crate::drivers::lp5813::Indicator::new(enable);
    loop {
        buttons.poll(&mut bus.0, Instant::now().as_millis()).await;
        if Instant::now().as_millis() < next_battery_ms {
            Timer::after_millis(10).await;
            continue;
        }
        next_battery_ms = Instant::now().as_millis() + 500;
        let before = power_good_n.is_low();
        let mut charger = read_charger(&mut bus, &mut detected).await;
        let gauge = read_gauge(&mut bus).await;
        let power_good = power_good_n.is_low();
        if before != power_good {
            charger = Err("source-changing");
        }
        let snapshot = monitor.update(Instant::now().as_millis(), charger, gauge, power_good);
        SNAPSHOT.lock(|cell| cell.set(snapshot));
        led.update(&mut bus.0, snapshot.state).await;
        Timer::after_millis(10).await;
    }
}
