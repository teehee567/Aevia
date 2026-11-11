#![deny(clippy::mem_forget, clippy::large_stack_frames)]

use crate::{
    commands::Response as SystemResponse,
    device::{
        self, board, display, maintenance, power, processing, recording, sensors, storage::Storage,
        supervision, usb,
    },
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
use esp_rtos::embassy::InterruptExecutor;
use static_cell::{ConstStaticCell, StaticCell};

static ACQUISITION: StaticCell<InterruptExecutor<2>> = StaticCell::new();
static GNSS_ACQUISITION: StaticCell<InterruptExecutor<3>> = StaticCell::new();

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
    let board = board::Board::new(esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::max()),
    ));
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);
    let timers = TimerGroup::new(board.timers);
    let watchdog = boot::watchdog(timers.wdt);
    esp_rtos::start(timers.timer0, board.scheduler_interrupt);
    // SWI0/1 belong to the scheduler. Keep blocking power I/O on the main
    // executor, below the SDK's priority-1 timer that wakes acquisition waits.
    let acquisition = ACQUISITION
        .init(InterruptExecutor::new(board.acquisition_interrupt))
        .start(board::ACQUISITION_PRIORITY);
    let gnss_acquisition = GNSS_ACQUISITION
        .init(InterruptExecutor::new(board.gnss_interrupt))
        .start(board::GNSS_PRIORITY);

    let updater = UPDATER.init_with(|| {
        Storage::new(board.flash).and_then(|(storage, running)| Updater::new(storage, running))
    });
    let mode = match updater.as_mut() {
        Ok(updater) => updater.startup(boot::fault_reset()),
        Err(error) => Err(*error),
    };
    if updater
        .as_ref()
        .is_ok_and(|updater| updater.rollback_requested())
    {
        esp_hal::system::software_reset();
    }
    // A rollback handoff must leave the retained panic for the image that
    // actually serves USB. Consume it only after the immediate reboot check.
    crate::bootdiagnostics::initialize();
    let buffers = USB_BUFFERS.init_with(usb::Buffers::new);
    let (usb_device, serial) = usb::init(board.usb, buffers, !matches!(mode, Ok(Mode::App)));
    spawner.spawn(serve(serial, updater, mode, watchdog).unwrap());
    spawner.spawn(usb_task(usb_device).unwrap());
    let mut power_bus = board.power_bus;
    // Apply the configured charging policy in recovery as well as application
    // mode. The subsequent bus owner retries and checks readback every second.
    power::maintain(&mut power_bus);
    if matches!(mode, Ok(Mode::App)) {
        STARTUP_STAGE.store(0, Ordering::Release);
        // Enumerate USB and service commands before touching application hardware.
        Timer::after_secs(2).await;
        STARTUP_STAGE.store(1, Ordering::Release);
        Timer::after_millis(50).await;
        let application = board.application.initialize();
        let memory = application.memory;
        spawner.spawn(
            maintenance::task(power_bus, application.maintenance).expect("maintenance task"),
        );
        if board::DISPLAY_FITTED {
            spawner.spawn(display::task(application.display).expect("display task"));
        }
        STARTUP_STAGE.store(2, Ordering::Release);
        Timer::after_millis(50).await;
        // This range belongs exclusively to the estimator, never the heap/DMA.
        esp_rtos::start_second_core(
            application.cpu,
            application.worker_interrupt,
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
                let (address, len) = match memory.prepare() {
                    Ok(region) => region,
                    Err(_) => {
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
                };
                if device::configuration::DATA_COLLECTION_ONLY {
                    unsafe { recording::run(application.recording_slot, address, len) }
                } else {
                    unsafe { processing::run(address, len) }
                }
            },
        );
        STARTUP_STAGE.store(3, Ordering::Release);
        Timer::after_millis(50).await;
        // Release the worker before acquisition: its destructive PSRAM check
        // and workspace copy must not compete with DRDY deadlines or fill the
        // bounded measurement queues before their consumer can run.
        STARTUP_STAGE.store(4, Ordering::Release);
        let _ = with_timeout(Duration::from_secs(20), async {
            loop {
                if matches!(startup_stage(), 10 | 11)
                    || matches!(
                        device::interface::snapshot().phase,
                        device::interface::Phase::Failed
                    )
                {
                    break;
                }
                Timer::after_millis(10).await;
            }
        })
        .await;
        sensors::start(spawner, acquisition, gnss_acquisition, application.sensors);
    } else {
        spawner.spawn(power::recovery_task(power_bus).unwrap());
    }
    core::future::pending().await
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
    let mut frame = Frame::<{ crate::protocol::FRAME_SIZE }>::new();
    let mut packet = [0; 512];
    let mut output = [0; crate::protocol::RESPONSE_FRAME_SIZE];
    let mut last_data = Instant::now();
    let mut monitor = supervision::Monitor::new(last_data.as_micros());
    let application = matches!(mode, Ok(Mode::App));
    let mut confirmed = false;
    let mut confirm_after_us = 0;
    loop {
        let now = Instant::now().as_micros();
        let readings = supervision::readings();
        if monitor.healthy(now, application, startup_stage(), readings) {
            watchdog.feed();
        }
        if application && !confirmed && now >= confirm_after_us && monitor.confirmable(now) {
            // Confirmation performs no sensor work and is idempotent. Leave
            // an unconfirmed trial recoverable if flash confirmation fails.
            if let Ok(updater) = updater.as_mut() {
                confirmed = updater.confirm_app().is_ok();
                confirm_after_us = now.saturating_add(3_000_000);
            }
        }
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
                (Ok(_), Ok(Request::Device(command)))
                    if matches!(mode, Ok(Mode::App)) || command.available_in_recovery() =>
                {
                    Ok(Response::Device(device::interface::command(command)))
                }
                (Ok(_), Ok(Request::Recording(request))) if matches!(mode, Ok(Mode::App)) => {
                    Ok(Response::Recording(recording::command(request)))
                }
                (Err(error), _) => Err(*error),
                _ => Err(Error::Command),
            };
            let response = result.unwrap_or_else(|error| {
                abort(updater);
                Response::System(SystemResponse::Error(error))
            });
            let reboot = matches!(response, Response::System(SystemResponse::Reboot));
            // Recording reads span several HS packets. Framing is independent
            // of packet boundaries; ordinary telemetry remains compact.
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
