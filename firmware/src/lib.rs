#![no_std]

mod app;
mod flash;
mod loader;
mod usb;
mod watchdog;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::Timer;
use esp_hal::{
    Config,
    clock::CpuClock,
    peripherals::TIMG0,
    timer::timg::{TimerGroup, Wdt},
};
use flash::Flash;

pub use esp_hal::system::software_reset as restart;

pub async fn run(spawner: Spawner, recovery: bool) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);
    watchdog::disable_boot_watchdogs(&mut peripherals.TIMG0, &mut peripherals.TIMG1);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);

    let timers = TimerGroup::new(peripherals.TIMG0);
    let mut watchdog = timers.wdt;
    watchdog::start(&mut watchdog);
    esp_rtos::start(timers.timer0, peripherals.FROM_CPU_INTR0);
    spawner.spawn(feed(watchdog).unwrap());

    if !recovery {
        spawner.spawn(app::run().unwrap());
    }

    let flash = Flash::new(peripherals.FLASH, peripherals.CPU_CTRL);
    let (mut usb_device, serial) = usb::init(peripherals.USB_HS, recovery);
    select(usb_device.run(), loader::run(serial, flash, recovery)).await;

    usb_device.disable().await;
    Timer::after_millis(100).await;
    restart()
}

#[embassy_executor::task]
async fn feed(mut watchdog: Wdt<TIMG0<'static>>) {
    loop {
        Timer::after_secs(1).await;
        watchdog.feed();
    }
}
