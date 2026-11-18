#![no_std]

mod app;
pub mod battery;
mod ble;
mod board;
pub mod buttons;
pub mod capture;
mod configuration;
mod display;
mod drivers;
mod flash;
pub mod gnss;
pub mod gnss_binary;
pub mod imu;
mod loader;
mod navigation;
pub mod ppk;
mod preview_queue;
mod recording;
mod response;
pub mod sample;
pub mod storage;
mod tasks;
pub mod timing;
pub mod uart_pipe;
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

pub async fn run<const RECOVERY: bool>(spawner: Spawner) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);
    watchdog::disable_boot_watchdogs(&mut peripherals.TIMG0, &mut peripherals.TIMG1);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);
    if !RECOVERY {
        esp_alloc::heap_allocator!(size: 80 * 1024);
    }

    let timers = TimerGroup::new(peripherals.TIMG0);
    let mut watchdog = timers.wdt;
    watchdog::start(&mut watchdog);
    esp_rtos::start(timers.timer0);
    spawner.spawn(feed(watchdog).unwrap());

    if !RECOVERY && !cfg!(feature = "diagnostic-usb-only") {
        spawner.spawn(ble::run(peripherals.BT).unwrap());
        let recording_slot = board::start(
            spawner,
            board::Hardware {
                GPIO20: peripherals.GPIO20,
                GPIO21: peripherals.GPIO21,
                GPIO22: peripherals.GPIO22,
                GPIO23: peripherals.GPIO23,
                GPIO24: peripherals.GPIO24,
                GPIO25: peripherals.GPIO25,
                SDHOST: peripherals.SDHOST,
                GPIO4: peripherals.GPIO4,
                GPIO15: peripherals.GPIO15,
                GPIO16: peripherals.GPIO16,
                GPIO17: peripherals.GPIO17,
                GPIO18: peripherals.GPIO18,
                GPIO19: peripherals.GPIO19,
                SPI3: peripherals.SPI3,
                GPIO8: peripherals.GPIO8,
                GPIO47: peripherals.GPIO47,
                GPIO46: peripherals.GPIO46,
                GPIO49: peripherals.GPIO49,
                GPIO48: peripherals.GPIO48,
                GPIO12: peripherals.GPIO12,
                GPIO13: peripherals.GPIO13,
                GPIO11: peripherals.GPIO11,
                GPIO10: peripherals.GPIO10,
                GPIO9: peripherals.GPIO9,
                GPIO14: peripherals.GPIO14,
                GPIO6: peripherals.GPIO6,
                GPIO7: peripherals.GPIO7,
                GPIO39: peripherals.GPIO39,
                GPIO40: peripherals.GPIO40,
                GPIO42: peripherals.GPIO42,
                UART1: peripherals.UART1,
                UART2: peripherals.UART2,
                SPI2: peripherals.SPI2,
                I2C0: peripherals.I2C0,
                FROM_CPU_INTR2: peripherals.FROM_CPU_INTR2,
                FROM_CPU_INTR3: peripherals.FROM_CPU_INTR3,
                IO_MUX: peripherals.IO_MUX,
            },
        );
        if !cfg!(feature = "diagnostic-sensors-only") {
            // Historical V2 Mini preflight exceeded 32 KiB. Keep validation and
            // estimator calls on the proven 64 KiB CPU1 stack.
            // Const initialization keeps the 64 KiB backing array off CPU0's
            // startup stack. StaticCell::init(value) can construct it on the caller
            // stack before moving it, overflowing before USB is initialized.
            static STORAGE_STACK: static_cell::ConstStaticCell<esp_hal::system::Stack<65536>> =
                static_cell::ConstStaticCell::new(esp_hal::system::Stack::new());
            let psram = if cfg!(feature = "diagnostic-no-navigation") {
                None
            } else {
                Some(esp_hal::psram::Psram::new(
                    peripherals.PSRAM,
                    esp_hal::psram::PsramConfig {
                        timing: esp_hal::psram::PsramTimingParams::MHZ_125,
                        ..Default::default()
                    },
                ))
            };
            // Core 1 executes estimator code in flash while CPU0 retains its cache.
            unsafe { &*esp32s31::CACHE::ptr() }
                .l1_icache_ctrl()
                .modify(|_, w| w.l1_icache_shut_ibus1().clear_bit());
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            esp_rtos::start_second_core(
                peripherals.CPU_CTRL.reborrow(),
                STORAGE_STACK.take(),
                move || {
                    if let Some(psram) = psram {
                        navigation::initialize(psram);
                    }
                    storage::run(recording_slot)
                },
            );
        }
    }

    let flash = Flash::new(peripherals.FLASH, peripherals.CPU_CTRL);
    let (mut usb_device, serial) = usb::init(peripherals.USB_HS, RECOVERY);
    select(usb_device.run(), loader::run::<RECOVERY>(serial, flash)).await;

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
