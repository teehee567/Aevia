#![no_std]
#![no_main]
#![deny(clippy::mem_forget, clippy::large_stack_frames)]

use embassy_executor::Spawner;
use firmware::{loader, runtime};

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    firmware::bootdiagnostics::record_panic(info);
    loader::boot::enter_recovery()
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    runtime::run(spawner).await
}
