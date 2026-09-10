//! Second CDC interface: raw, bidirectional UM980 COM2 transport. Only the first
//! interface parses firmware commands; binary receiver traffic is never parsed.
use super::{PACKET_SIZE, send_line};
use crate::{drivers::um980, state, tasks};
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb::class::cdc_acm::{ControlChanged, ParityType, Receiver, Sender, StopBits};
use esp_hal::{Async, uart::Uart, usb::otg::embassy_usb_device::Driver};

static MODE: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub fn request(enabled: bool) {
    MODE.signal(enabled);
}

#[derive(Clone, Copy)]
pub struct Status {
    pub baud: u32,
    pub to_receiver: u32,
    pub from_receiver: u32,
    pub errors: u32,
}
pub static STATUS: state::Shared<Status> = state::Shared::new(Status {
    baud: um980::DATA_BAUD,
    to_receiver: 0,
    from_receiver: 0,
    errors: 0,
});

#[embassy_executor::task]
pub async fn run(
    mut command: Uart<'static, Async>,
    mut data: Uart<'static, Async>,
    mut sender: Sender<'static, Driver<'static>>,
    mut receiver: Receiver<'static, Driver<'static>>,
    control: ControlChanged<'static>,
) {
    let mut bridge = false;
    loop {
        let work = async {
            if bridge {
                // Start each explicit bridge session at a known receiver baud.
                // No profile writes occur again until GNSSNORMAL or a reboot.
                if prepare(&mut command, &mut data).await.is_err() {
                    state::GNSS_ERROR.update(|last| *last = Some("bridge-baud-setup"));
                    state::GNSS.update(|status| status.phase = state::Phase::Fault);
                    core::future::pending::<()>().await;
                }
                state::FIX.update(|fix| fix.clear());
                state::GNSS.update(|status| status.phase = state::Phase::Bridge);
                transfer(&mut data, &mut sender, &mut receiver, &control).await;
            } else {
                tasks::gnss::run(&mut command, &mut data).await;
            }
        };
        if let Either::First(enabled) = select(MODE.wait(), work).await {
            if bridge && !enabled {
                // COM1 remains available to restore COM2 after host baud changes.
                let _ = restore_baud(&mut command, &mut data).await;
            }
            bridge = enabled;
        }
    }
}

async fn restore_baud(
    command: &mut Uart<'_, Async>,
    data: &mut Uart<'_, Async>,
) -> Result<(), &'static str> {
    um980::set_baud(command, 115_200)?;
    um980::write(command, b"\r\nCONFIG COM2 460800\r\n").await?;
    Timer::after_millis(100).await;
    um980::set_baud(data, um980::DATA_BAUD)
}

async fn prepare(
    command: &mut Uart<'_, Async>,
    data: &mut Uart<'_, Async>,
) -> Result<(), &'static str> {
    restore_baud(command, data).await?;
    um980::write(command, b"BESTNAVA COM2 0.05\r\n").await
}

async fn transfer<'d>(
    data: &mut Uart<'_, Async>,
    sender: &mut Sender<'d, Driver<'d>>,
    receiver: &mut Receiver<'d, Driver<'d>>,
    control: &ControlChanged<'d>,
) {
    loop {
        receiver.wait_connection().await;
        let coding = receiver.line_coding();
        // Embassy's 8000-baud initial value is not a host UART selection.
        let baud = if coding.data_rate() == 8_000 {
            um980::DATA_BAUD
        } else {
            coding.data_rate()
        };
        if coding.data_bits() != 8
            || coding.parity_type() != ParityType::None
            || coding.stop_bits() != StopBits::One
            || !(9_600..=4_000_000).contains(&baud)
            || um980::set_baud(data, baud).is_err()
        {
            state::GNSS_ERROR.update(|last| *last = Some("bridge-requires-8N1-valid-baud"));
            control.control_changed().await;
            continue;
        }
        STATUS.update(|status| status.baud = baud);
        let (rx, tx) = data.split_mut();
        let host_to_receiver = async {
            let mut packet = [0; PACKET_SIZE];
            loop {
                let Ok(length) = receiver.read_packet(&mut packet).await else {
                    return;
                };
                if embedded_io_async::Write::write_all(tx, &packet[..length])
                    .await
                    .is_err()
                {
                    error();
                    return;
                }
                STATUS.update(|status| {
                    status.to_receiver = status.to_receiver.wrapping_add(length as u32);
                });
            }
        };
        let receiver_to_host = async {
            let mut packet = [0; PACKET_SIZE];
            loop {
                match rx.read_async(&mut packet).await {
                    Ok(0) => {}
                    Ok(length) => {
                        if send_line(sender, &packet[..length]).await.is_err() {
                            error();
                        } else {
                            STATUS.update(|status| {
                                status.from_receiver =
                                    status.from_receiver.wrapping_add(length as u32);
                            });
                        }
                    }
                    Err(_) => error(),
                }
            }
        };
        // Control changes update UART baud. Mode changes are handled by the
        // outer owner, which cancels both directions before normal acquisition.
        select(
            control.control_changed(),
            select(host_to_receiver, receiver_to_host),
        )
        .await;
        let _ = with_timeout(Duration::from_millis(100), async {
            embedded_io_async::Write::flush(data).await
        })
        .await;
        // Avoid a busy loop during physical USB disconnection.
        Timer::after_millis(1).await;
    }
}

fn error() {
    STATUS.update(|status| status.errors = status.errors.wrapping_add(1));
}
