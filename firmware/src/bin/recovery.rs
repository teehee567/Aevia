#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    firmware::restart()
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    firmware::run::<true>(spawner).await
}
