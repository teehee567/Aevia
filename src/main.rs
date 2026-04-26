#![no_std]
#![no_main]

extern crate alloc;

mod led;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;
use panic_halt as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 16 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    {
        use core::mem::MaybeUninit;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_nrf::init(Default::default());

    let red = Output::new(p.P0_26, Level::High, OutputDrive::Standard);
    let green = Output::new(p.P0_30, Level::High, OutputDrive::Standard);

    spawner.spawn(led::blink(red).unwrap());
    Timer::after(Duration::from_millis(125)).await;
    spawner.spawn(led::blink(green).unwrap());
}
