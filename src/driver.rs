use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{Peri, bind_interrupts, peripherals, usb};
use embassy_usb_logger::ReceiverHandler;

// Adafruit nRF52 bootloader magic: 0x57 = boot into UF2 mode on next reset.
const DFU_MAGIC_UF2_RESET: u32 = 0x57;
const POWER_GPREGRET: *mut u32 = 0x4000_051C as *mut u32;

pub fn reboot_to_bootloader() -> ! {
    unsafe { POWER_GPREGRET.write_volatile(DFU_MAGIC_UF2_RESET) };
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    reboot_to_bootloader();
}

struct RebootHandler;
impl ReceiverHandler for RebootHandler {
    fn new() -> Self { Self }
    async fn handle_data(&self, data: &[u8]) {
        // Magic byte from flash.py: reboot into UF2 bootloader.
        if data.contains(&b'!') {
            reboot_to_bootloader();
        }
    }
}

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

const CLOCK_TASKS_HFCLKSTART: *mut u32 = 0x4000_0000 as *mut u32;
const CLOCK_EVENTS_HFCLKSTARTED: *mut u32 = 0x4000_0100 as *mut u32;

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, HardwareVbusDetect>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, RebootHandler);
}

pub fn init(spawner: &Spawner, usbd: Peri<'static, peripherals::USBD>) {
    // USB needs the external HFCLK.
    unsafe {
        CLOCK_TASKS_HFCLKSTART.write_volatile(1);
        while CLOCK_EVENTS_HFCLKSTARTED.read_volatile() != 1 {}
    }

    let driver = Driver::new(usbd, Irqs, HardwareVbusDetect::new(Irqs));
    spawner.spawn(logger_task(driver).unwrap());
}
