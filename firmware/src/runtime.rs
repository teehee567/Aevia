#![deny(clippy::mem_forget, clippy::large_stack_frames)]

use crate::{
    commands::Response as SystemResponse,
    device::{self, processing, sensors, storage::Storage, usb},
    loader::{
        boot, recovery,
        state::Mode,
        update::{Error, Updater},
    },
    protocol::{Frame, Request, Response},
};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    clock::CpuClock,
    peripherals::TIMG0,
    system::Stack,
    timer::timg::{TimerGroup, Wdt},
};
use static_cell::{ConstStaticCell, StaticCell};

static USB_BUFFERS: StaticCell<usb::Buffers<'static>> = StaticCell::new();
static UPDATER: StaticCell<Updates> = StaticCell::new();
static ESTIMATOR_STACK: ConstStaticCell<Stack<32768>> = ConstStaticCell::new(Stack::new());
type Updates = Result<Updater<Storage<'static>>, Error>;

pub async fn run(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);
    let timers = TimerGroup::new(peripherals.TIMG0);
    let watchdog = boot::watchdog(timers.wdt);
    esp_rtos::start(timers.timer0, peripherals.FROM_CPU_INTR0);

    let updater = UPDATER.init_with(|| {
        Storage::new(peripherals.FLASH)
            .and_then(|(storage, running)| Updater::new(storage, running))
    });
    let mode = match updater.as_mut() {
        Ok(updater) => updater.startup(boot::fault_reset()),
        Err(error) => Err(*error),
    };
    if matches!(mode, Ok(Mode::App)) {
        let psram = esp_hal::psram::Psram::new(peripherals.PSRAM, Default::default());
        let (address, len) = psram.raw_parts();
        let address = address as usize;
        // This range belongs exclusively to the estimator, never the heap/DMA.
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            peripherals.FROM_CPU_INTR1,
            ESTIMATOR_STACK.take(),
            move || unsafe { processing::run(address, len) },
        );
        sensors::start(
            spawner,
            sensors::Board {
                uart1: peripherals.UART1,
                uart2: peripherals.UART2,
                spi2: peripherals.SPI2,
                spi3: peripherals.SPI3,
                i2c0: peripherals.I2C0,
                gpio4: peripherals.GPIO4,
                gpio6: peripherals.GPIO6,
                gpio7: peripherals.GPIO7,
                gpio8: peripherals.GPIO8,
                gpio9: peripherals.GPIO9,
                gpio10: peripherals.GPIO10,
                gpio11: peripherals.GPIO11,
                gpio12: peripherals.GPIO12,
                gpio13: peripherals.GPIO13,
                gpio14: peripherals.GPIO14,
                gpio15: peripherals.GPIO15,
                gpio16: peripherals.GPIO16,
                gpio17: peripherals.GPIO17,
                gpio18: peripherals.GPIO18,
                gpio19: peripherals.GPIO19,
                gpio39: peripherals.GPIO39,
                gpio42: peripherals.GPIO42,
                gpio44: peripherals.GPIO44,
                gpio46: peripherals.GPIO46,
                gpio47: peripherals.GPIO47,
                gpio48: peripherals.GPIO48,
                gpio49: peripherals.GPIO49,
            },
        );
    }
    let buffers = USB_BUFFERS.init_with(usb::Buffers::new);
    let (mut usb_device, serial) =
        usb::init(peripherals.USB_HS, buffers, !matches!(mode, Ok(Mode::App)));
    spawner.spawn(serve(serial, updater, mode, watchdog).unwrap());
    usb_device.run().await
}

#[embassy_executor::task]
async fn serve(
    serial: usb::Serial<'static>,
    updater: &'static mut Updates,
    mode: Result<Mode, Error>,
    mut watchdog: Wdt<TIMG0<'static>>,
) {
    let (mut sender, mut receiver, control) = serial.split_with_control();
    let mut frame = Frame::<272>::new();
    let mut packet = [0; 512];
    let mut output = [0; 512];
    let mut last_data = Instant::now();
    loop {
        watchdog.feed();
        let received = select3(
            control.control_changed(),
            receiver.read_packet(&mut packet),
            Timer::after_millis(250),
        )
        .await;
        if matches!(received, Either3::First(()) | Either3::Second(Err(_))) || !control.dtr() {
            abort(updater);
            frame.clear();
            Timer::after_millis(10).await;
            continue;
        }
        if Instant::now() - last_data > Duration::from_secs(30) {
            abort(updater);
            frame.clear();
            last_data = Instant::now();
        }
        let Either3::Second(Ok(len)) = received else {
            continue;
        };
        last_data = Instant::now();
        let mut input = &packet[..len];
        while !input.is_empty() {
            let Some(request) = frame.feed::<Request<'_>>(&mut input) else {
                break;
            };
            let result = match (updater.as_mut(), request) {
                (Ok(updater), Ok(Request::System(command))) => match mode {
                    Ok(Mode::App) => device::command(updater, command),
                    Ok(Mode::Recovery) => recovery::command(updater, command),
                    Err(error) => Err(error),
                }
                .map(Response::System),
                (Ok(_), Ok(Request::Device(command))) if matches!(mode, Ok(Mode::App)) => {
                    Ok(Response::Device(device::interface::command(command)))
                }
                (Err(error), _) => Err(*error),
                _ => Err(Error::Command),
            };
            let response = result.unwrap_or_else(|error| {
                abort(updater);
                Response::System(SystemResponse::Error(error))
            });
            let reboot = matches!(response, Response::System(SystemResponse::Reboot));
            // All current responses fit one HS packet; chunking also supports
            // larger future responses without tying framing to USB packets.
            let sent = match postcard::to_slice_cobs(&response, &mut output) {
                Ok(bytes) => with_timeout(Duration::from_secs(1), async {
                    for packet in bytes.chunks(512) {
                        sender.write_packet(packet).await?;
                    }
                    Ok::<_, embassy_usb::driver::EndpointError>(())
                })
                .await
                .is_ok_and(|result| result.is_ok()),
                Err(_) => false,
            };
            if reboot {
                Timer::after_millis(100).await;
                esp_hal::system::software_reset();
            }
            if !sent {
                abort(updater);
                frame.clear();
                break;
            }
        }
    }
}

fn abort(updater: &mut Updates) {
    if let Ok(updater) = updater.as_mut() {
        updater.abort();
    }
}
