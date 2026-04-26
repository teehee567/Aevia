#![no_std]
#![no_main]

extern crate alloc;

mod led;

use embassy_executor::Spawner;
use embassy_nrf::{Peri, gpio::{Level, Output, OutputDrive, Pin}};
use embassy_time::{Duration, Timer};
use embedded_alloc::TlsfHeap as Heap;
use panic_halt as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 16 * 1024;

fn disable_xiao_charging<P: Pin>(pin: Peri<'static, P>) {
    let _ = Output::new(pin, Level::High, OutputDrive::Standard);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        embedded_alloc::init!(HEAP, HEAP_SIZE);
    }

    let peripherals = embassy_nrf::init(Default::default());

    disable_xiao_charging(peripherals.P0_13);

    let red = Output::new(peripherals.P0_26, Level::High, OutputDrive::Standard);
    let green = Output::new(peripherals.P0_30, Level::High, OutputDrive::Standard);

    spawner.spawn(led::blink(red).unwrap());
    Timer::after(Duration::from_millis(125)).await;
    spawner.spawn(led::blink(green).unwrap());
}
