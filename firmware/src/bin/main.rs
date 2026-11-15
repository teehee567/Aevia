#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    esp_hal::system::software_reset()
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    firmware::run(spawner, false).await
}
