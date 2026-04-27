#![no_std]
#![no_main]

extern crate alloc;

mod driver;
mod led;
mod gps;

use alloc::vec::Vec;
use chrono::{DateTime, Utc};
use embassy_executor::Spawner;
use embassy_nrf::{Peri, gpio::{Level, Output, OutputDrive, Pin}};
use embassy_time::{Duration, Timer};
use embedded_alloc::TlsfHeap as Heap;
use log::info;

use gps::Gps;
use ublox::{GnssFixType, PositionLLA, UbxPacket, Velocity, nav_sat::NavSat, proto27::PacketRef};

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

    let mut gps = Gps::new(
        peripherals.UARTE0,
        peripherals.TIMER1,
        peripherals.PPI_CH3,
        peripherals.PPI_CH4,
        peripherals.PPI_GROUP1,
        peripherals.P1_12,
        peripherals.P1_11,
    );
    spawner.spawn(led::blink(
        Output::new(peripherals.P0_26, Level::Low, OutputDrive::Standard),
        Duration::from_millis(250),
    ).unwrap());

    gps.configure().await;

    loop {
        gps.poll(|pkt| {
            match pkt {
                UbxPacket::Proto27(packet_ref) => {
                    match &packet_ref {
                        PacketRef::MonVer(packet) => {
                            info!(
                                "SW version: {} HW version: {}; Extensions: {:?}",
                                packet.software_version(),
                                packet.hardware_version(),
                                packet.extension().collect::<Vec<&str>>()
                            );
                        },
                        PacketRef::NavPvt(pvt) => {
                            let has_time = pvt.fix_type() == GnssFixType::Fix3D
                                || pvt.fix_type() == GnssFixType::GPSPlusDeadReckoning
                                || pvt.fix_type() == GnssFixType::TimeOnlyFix;
                            let has_posvel = pvt.fix_type() == GnssFixType::Fix3D
                                || pvt.fix_type() == GnssFixType::GPSPlusDeadReckoning;

                            if has_posvel {
                                let pos: PositionLLA = pvt.into();
                                let vel: Velocity = pvt.into();
                                info!(
                                    "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                                    pos.lat, pos.lon, pos.alt
                                );
                                info!(
                                    "Speed: {:.2} m/s Heading: {:.2} degrees",
                                    vel.speed, vel.heading
                                );
                                info!("Sol: {pvt:?}");
                            }

                            if has_time {
                                let time: DateTime<Utc> = pvt.try_into().unwrap();
                                info!("Time: {time:?}");
                            }

                        },
                        PacketRef::EsfRaw(raw) => {
                            info!("Got raw message: {raw:?}");
                        },
                        PacketRef::NavSat(sat) => {

                        }
                        _ => {
                            info!("{packet_ref:?}");
                        },
                    }
                },
            }
        }).await;
    }
}
