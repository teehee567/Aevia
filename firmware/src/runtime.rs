#![deny(clippy::mem_forget, clippy::large_stack_frames)]

use crate::loader::{
    boot,
    recovery::{self, Line, Reply},
    state::Mode,
    update::{Error, Updater},
};
use crate::device::{self, storage::Storage, usb};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    peripherals::TIMG0,
    timer::timg::{TimerGroup, Wdt},
};
use static_cell::StaticCell;

static USB_BUFFERS: StaticCell<usb::Buffers<'static>> = StaticCell::new();
static LINE: StaticCell<Line> = StaticCell::new();
static PACKET: StaticCell<[u8; 512]> = StaticCell::new();
static UPDATER: StaticCell<Result<Updater<Storage<'static>>, Error>> = StaticCell::new();
type Updates = Result<Updater<Storage<'static>>, Error>;

pub async fn run(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);
    let timers = TimerGroup::new(peripherals.TIMG0);
    let watchdog = boot::watchdog(timers.wdt);
    esp_rtos::start(timers.timer0, peripherals.FROM_CPU_INTR0);

    let buffers = USB_BUFFERS.init_with(usb::Buffers::new);
    let (mut device, serial) = usb::init(peripherals.USB_HS, buffers);
    let updater = UPDATER.init_with(|| {
        Storage::new(peripherals.FLASH)
            .and_then(|(storage, running)| Updater::new(storage, running))
    });
    let mode = match updater.as_mut() {
        Ok(updater) => updater.startup(boot::fault_reset()),
        Err(error) => Err(*error),
    };
    spawner.spawn(serve(serial, updater, mode, watchdog).unwrap());
    device.run().await
}

#[embassy_executor::task]
async fn serve(
    serial: usb::Serial<'static>,
    updater: &'static mut Updates,
    mode: Result<Mode, Error>,
    mut watchdog: Wdt<TIMG0<'static>>,
) {
    let (mut sender, mut receiver, control) = serial.split_with_control();
    let line = LINE.init_with(Line::new);
    let packet = PACKET.init_with(|| [0; 512]);

    let mut last_data = Instant::now();
    loop {
        watchdog.feed();
        let received = select3(
            control.control_changed(),
            receiver.read_packet(packet),
            Timer::after_millis(250),
        )
        .await;
        // A control change also catches a close/reopen between timer ticks.
        if matches!(received, Either3::First(()) | Either3::Second(Err(_))) || !control.dtr() {
            if let Ok(updater) = updater.as_mut() {
                updater.abort();
            }
            line.clear();
            // A disconnected endpoint returns immediately. Do not spin.
            Timer::after_millis(10).await;
            continue;
        }
        if Instant::now() - last_data > Duration::from_secs(30) {
            if let Ok(updater) = updater.as_mut() {
                updater.abort();
            }
            line.clear();
            last_data = Instant::now();
        }
        let Either3::Second(Ok(len)) = received else {
            continue;
        };
        last_data = Instant::now();
        for &byte in &packet[..len] {
            let Some(input) = line.push(byte) else {
                continue;
            };
            let result = match (updater.as_mut(), input) {
                (Ok(updater), Ok(input)) => match mode {
                    Ok(Mode::App) => device::command(input),
                    Ok(Mode::Recovery) => recovery::command(updater, input),
                    Err(error) => Err(error),
                },
                (Ok(updater), Err(error)) => {
                    updater.abort();
                    Err(error)
                }
                (Err(error), _) => Err(*error),
            };
            let reboot = matches!(result, Ok(Reply::Reboot));
            let response = match result {
                Ok(Reply::Text(text)) => text,
                Ok(Reply::Reboot) => b"OK reboot\n",
                Err(error) => error.response(),
            };
            let sent = with_timeout(Duration::from_secs(1), sender.write_packet(response)).await;
            if reboot {
                Timer::after_millis(100).await;
                esp_hal::system::software_reset();
            }
            if !matches!(sent, Ok(Ok(()))) {
                if let Ok(updater) = updater.as_mut() {
                    updater.abort();
                }
                line.clear();
                break;
            }
        }
    }
}
