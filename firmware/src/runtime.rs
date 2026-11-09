#![deny(clippy::mem_forget, clippy::large_stack_frames)]

use crate::{
    commands::Response as SystemResponse,
    device::{self, memory, processing, sensors, storage::Storage, usb},
    loader::{
        boot, recovery,
        state::Mode,
        update::{Error, Updater},
    },
    protocol::{Frame, Request, Response},
};
use core::sync::atomic::{AtomicU8, Ordering};
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
// Preflight's nested configuration/covariance validation shares the worker's
// stack with Processor::start. The original 32 KiB also held its large caller
// frame and reset the core during preflight on the fitted board.
static ESTIMATOR_STACK: ConstStaticCell<Stack<65536>> = ConstStaticCell::new(Stack::new());
type Updates = Result<Updater<Storage<'static>>, Error>;

static STARTUP_STAGE: AtomicU8 = AtomicU8::new(0);

pub fn startup_stage() -> u8 {
    STARTUP_STAGE.load(Ordering::Acquire)
}

pub(crate) fn set_startup_stage(stage: u8) {
    STARTUP_STAGE.store(stage, Ordering::Release);
}

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
    let buffers = USB_BUFFERS.init_with(usb::Buffers::new);
    let (usb_device, serial) =
        usb::init(peripherals.USB_HS, buffers, !matches!(mode, Ok(Mode::App)));
    spawner.spawn(serve(serial, updater, mode, watchdog).unwrap());
    spawner.spawn(usb_task(usb_device).unwrap());
    if matches!(mode, Ok(Mode::App)) {
        STARTUP_STAGE.store(0, Ordering::Release);
        // Enumerate USB and service commands before touching application hardware.
        Timer::after_secs(2).await;
        STARTUP_STAGE.store(1, Ordering::Release);
        Timer::after_millis(50).await;
        let psram = esp_hal::psram::Psram::new(
            peripherals.PSRAM,
            esp_hal::psram::PsramConfig {
                timing: esp_hal::psram::PsramTimingParams::MHZ_125,
                ..Default::default()
            },
        );
        let (address, len) = psram.raw_parts();
        let address = address as usize;
        STARTUP_STAGE.store(2, Ordering::Release);
        Timer::after_millis(50).await;
        enable_second_core_cache();
        // This range belongs exclusively to the estimator, never the heap/DMA.
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            peripherals.FROM_CPU_INTR1,
            ESTIMATOR_STACK.take(),
            move || {
                // Let core 0 finish peripheral setup before the worker touches
                // queues and publishes snapshots from the second core.
                while startup_stage() < 4 {
                    esp_rtos::CurrentThreadHandle::get()
                        .delay(esp_hal::time::Duration::from_millis(1));
                }
                STARTUP_STAGE.store(5, Ordering::Release);
                // Core 1's ROM PMA is RX over flash AND PSRAM. Restore the
                // tested board-specific RW split before the first PSRAM store.
                if memory::bring_up(psram).is_err() {
                    device::interface::publish(device::interface::Snapshot {
                        updated_us: Instant::now().as_micros(),
                        phase: device::interface::Phase::Failed,
                        processing_errors: 1,
                        error_kind: Some(device::interface::EngineError::Resources),
                        ..Default::default()
                    });
                    STARTUP_STAGE.store(11, Ordering::Release);
                    return;
                }
                unsafe { processing::run(address, len) }
            },
        );
        STARTUP_STAGE.store(3, Ordering::Release);
        Timer::after_millis(50).await;
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
        STARTUP_STAGE.store(4, Ordering::Release);
    }
    core::future::pending().await
}

/// The single-core bootloader can leave CPU1's instruction bus shut. The S31
/// HAL starts CPU1 at a flash address, so open its cache bus before releasing it.
/// Mirrors ESP-IDF cache_ll_l1_enable_bus(1, CACHE_BUS_IBUS0).
#[esp_hal::ram]
fn enable_second_core_cache() {
    // SAFETY: CPU1 is still held in reset. Only its instruction-cache bus gate
    // is changed; CPU0's live cache and the shared data-cache settings remain.
    let cache = unsafe { &*esp32s31::CACHE::ptr() };
    cache
        .l1_icache_ctrl()
        .modify(|_, w| w.l1_icache_shut_ibus1().clear_bit());
    core::sync::atomic::fence(Ordering::SeqCst);
}

#[embassy_executor::task]
async fn usb_task(mut device: usb::Device<'static>) {
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
