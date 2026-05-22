#![no_std]
#![no_main]

extern crate alloc;

mod driver;
mod gnss;
mod gps;
mod led;
mod run;
mod status;

use embassy_executor::Spawner;
use embassy_nrf::{
    gpio::{Input, Level, Output, OutputDrive, Pin, Pull},
    Peri,
};
use embassy_time::{Duration, Timer};
use embedded_alloc::TlsfHeap as Heap;
use log::info;

use gnss::{GnssReceiver, GnssUpdate};
use gps::Gps;
use run::{RunConfig, RunDetector, RunEvent};

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

    driver::init(&spawner, peripherals.USBD);

    disable_xiao_charging(peripherals.P0_13);

    Timer::after(Duration::from_secs(3)).await;
    info!("Boot");

    #[cfg(feature = "um980")]
    let receiver = gnss::Um980Receiver::new();
    #[cfg(not(feature = "um980"))]
    let receiver = gnss::UbloxReceiver::new();

    let uart = gps::uart(
        peripherals.UARTE0,
        peripherals.TIMER1,
        peripherals.PPI_CH3,
        peripherals.PPI_CH4,
        peripherals.PPI_GROUP1,
        peripherals.P1_12,
        peripherals.P1_11,
        receiver.baudrate(),
    );
    let mut gps = Gps::new(uart, receiver);
    let r_led = Output::new(peripherals.P0_26, Level::High, OutputDrive::Standard);
    let g_led = Output::new(peripherals.P0_30, Level::High, OutputDrive::Standard);
    let b_led = Output::new(peripherals.P0_06, Level::High, OutputDrive::Standard);
    spawner.spawn(status::status_led_task(r_led, g_led, b_led).unwrap());

    let pps = Input::new(peripherals.P0_28, Pull::None);
    spawner.spawn(status::pps_task(pps).unwrap());

    gps.configure().await;

    let mut detector = RunDetector::new(RunConfig::default());

    loop {
        gps.poll(|update| match update {
            GnssUpdate::Fix(fix) => {
                status::update(|s| s.has_fix = fix.has_fix);
                let event = detector.update(&fix);
                if fix.has_fix {
                    info!(
                        "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                        fix.lat, fix.lon, fix.alt_m
                    );
                    info!(
                        "Speed: {:.2} km/h Accel: {:.2} m/s^2 Heading: {:.2} degrees",
                        fix.speed_mps * 3.6,
                        detector.accel_mps2,
                        fix.heading_deg
                    );
                }
                match event {
                    Some(RunEvent::Armed) => info!("Armed — waiting for launch"),
                    Some(RunEvent::Launched) => info!("Launch! timing..."),
                    Some(RunEvent::Finished { time_s, distance_m }) => {
                        info!("0-target in {:.3} s over {:.1} m", time_s, distance_m)
                    }
                    Some(RunEvent::Aborted) => info!("Run aborted"),
                    None => {}
                }
            }
            GnssUpdate::Sats { count, best_cno } => {
                status::update(|s| {
                    s.sat_count = count;
                    s.best_cno = best_cno;
                });
            }
        })
        .await;
    }
}
