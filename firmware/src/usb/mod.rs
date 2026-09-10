//! J6 CDC transport. RX/control never waits for telemetry or sensor readiness.
mod battery;
mod gnss;
mod telemetry;

use crate::{state, tasks::display};
use aevia_firmware::command::{Command, Decoder};
use core::fmt::Write;
use embassy_futures::{
    join::join3,
    select::{Either3, select3},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use esp_hal::{
    Async,
    gpio::Input,
    peripherals::USB_HS,
    uart::Uart,
    usb::otg::{
        Usb,
        embassy_usb_device::{Config as DriverConfig, Driver},
    },
};

const PACKET_SIZE: usize = 512;
// One atomic request holds the nonce and operation together. Most recent
// request wins if a host sends several without waiting for STREAM READY.
static SESSION: Signal<CriticalSectionRawMutex, Option<u32>> = Signal::new();

pub async fn run(
    spawner: embassy_executor::Spawner,
    peripheral: USB_HS<'static>,
    command: Uart<'static, Async>,
    data: Uart<'static, Async>,
    _reset: Input<'static>,
) {
    // Static CDC resources let GNSS retain its own scheduled task rather than
    // sharing a poll with potentially expensive console telemetry formatting.
    static ENDPOINT_BUFFER: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    let driver = Driver::new(
        Usb::new_hs(peripheral),
        ENDPOINT_BUFFER.init([0; 2048]),
        DriverConfig::default(),
    );
    let mut config = embassy_usb::Config::new(0x303a, 0x4001);
    config.max_packet_size_0 = 64;
    config.manufacturer = Some("AEVIA");
    config.product = Some("V2 Mini Base Firmware");
    config.serial_number = Some("V2MINI0001");
    static CONFIG_DESCRIPTOR: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    static BOS_DESCRIPTOR: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    static CONTROL_BUFFER: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static CDC_STATE: static_cell::StaticCell<State<'static>> = static_cell::StaticCell::new();
    static GNSS_STATE: static_cell::StaticCell<State<'static>> = static_cell::StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUFFER.init([0; 64]),
    );
    let (sender, receiver) = CdcAcmClass::new(
        &mut builder,
        CDC_STATE.init(State::new()),
        PACKET_SIZE as u16,
    )
    .split();
    let (gnss_sender, gnss_receiver, gnss_control) = CdcAcmClass::new(
        &mut builder,
        GNSS_STATE.init(State::new()),
        PACKET_SIZE as u16,
    )
    .split_with_control();
    let mut device = builder.build();
    spawner.spawn(
        gnss::run(command, data, gnss_sender, gnss_receiver, gnss_control)
            .expect("GNSS owner task"),
    );
    join3(device.run(), transmit(sender), receive(receiver)).await;
}

async fn receive<'d>(mut receiver: Receiver<'d, Driver<'d>>) {
    let mut decoder = Decoder::default();
    let mut packet = [0; PACKET_SIZE];
    loop {
        receiver.wait_connection().await;
        decoder.reset();
        loop {
            let Ok(length) = receiver.read_packet(&mut packet).await else {
                break;
            };
            for &byte in &packet[..length] {
                match decoder.push(byte) {
                    Some(Command::Bootloader) => enter_bootloader(),
                    Some(Command::Stream(nonce)) => SESSION.signal(Some(nonce)),
                    Some(Command::Stop) => SESSION.signal(None),
                    Some(Command::LcdReinitialize) => display::reinitialize(),
                    Some(Command::LcdLight(duty)) => display::backlight::request(duty),
                    Some(Command::LcdTest) => display::test_pattern(),
                    Some(Command::GnssBridge) => gnss::request(true),
                    Some(Command::GnssNormal) => gnss::request(false),
                    _ => {}
                }
            }
        }
        decoder.reset();
        SESSION.signal(None);
    }
}

fn enter_bootloader() -> ! {
    esp_hal::peripherals::LP_SYS::regs()
        .sys_ctrl()
        .modify(|_, writer| writer.force_download_boot().set_bit());
    esp_hal::system::software_reset();
}

async fn transmit<'d>(mut sender: Sender<'d, Driver<'d>>) {
    loop {
        sender.wait_connection().await;
        state::capture(false);
        let mut next_status = Instant::now();
        let mut line = heapless::String::<1536>::new();
        loop {
            line.clear();
            let formatted = match select3(
                SESSION.wait(),
                Timer::at(next_status),
                state::MEASUREMENTS.receive(),
            )
            .await
            {
                Either3::First(nonce) => {
                    state::capture(false);
                    if let Some(nonce) = nonce {
                        write!(
                            line,
                            "STREAM READY {nonce:08x} {}\r\n",
                            Instant::now().as_micros()
                        )
                    } else {
                        line.push_str("STREAM STOPPED\r\n")
                            .map_err(|_| core::fmt::Error)
                    }
                    .map(|()| {
                        // The same writer sends the marker before any newly
                        // queued RAW records, including under USB backpressure.
                        state::capture(nonce.is_some());
                    })
                }
                Either3::Second(()) => {
                    next_status = Instant::now() + Duration::from_secs(1);
                    telemetry::status(&mut line)
                }
                Either3::Third(measurement) => telemetry::measurement(&mut line, measurement),
            };
            if formatted.is_err() {
                line.clear();
                let _ = line.push_str("ERROR telemetry-overflow\r\n");
            }
            if send_line(&mut sender, line.as_bytes()).await.is_err() {
                // Do not accumulate measurements while a host has stopped
                // reading. RX remains live and a new STREAM starts a session.
                state::capture(false);
                Timer::after_millis(100).await;
                break;
            }
        }
    }
}

async fn send_line<'d>(sender: &mut Sender<'d, Driver<'d>>, bytes: &[u8]) -> Result<(), ()> {
    with_timeout(Duration::from_millis(250), async {
        for packet in bytes.chunks(PACKET_SIZE) {
            sender.write_packet(packet).await.map_err(|_| ())?;
        }
        // Terminate a full-size transfer so CDC hosts do not wait for another
        // packet to deliver an exactly 512-byte line.
        if bytes.len().is_multiple_of(PACKET_SIZE) {
            sender.write_packet(&[]).await.map_err(|_| ())?;
        }
        Ok(())
    })
    .await
    .map_err(|_| ())?
}
