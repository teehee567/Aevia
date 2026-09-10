#![no_std]
#![no_main]

mod app;
mod board;
mod drivers;
mod state;
mod tasks;
mod usb;

use esp_backtrace as _;
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(spawner: embassy_executor::Spawner) {
    app::run(spawner).await;
}
