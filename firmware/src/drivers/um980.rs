//! UM980 standalone 50 Hz profile on COM1 commands / COM2 data.
//! SIGNALGROUP persists itself and can restart the receiver; other settings are volatile.
use crate::gnss::{LineDecoder, StandaloneRateCheck, valid_unicore_log};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::{
    Async,
    uart::{Config, RxConfig, Uart},
};
pub const DATA_BAUD: u32 = 460_800;
pub const COMMAND_BAUD: u32 = 460_800;
pub const PPK_OBSERVATIONS: &[u8] = b"OBSVMB COM1 0.2\r\n";
pub const PPK_EPHEMERIDES: &[u8] = b"GPSEPHB COM1\r\nGPSEPHB COM1 ONCHANGED\r\nGLOEPHB COM1\r\nGLOEPHB COM1 ONCHANGED\r\nBDSEPHB COM1\r\nBDSEPHB COM1 ONCHANGED\r\nGALEPHB COM1\r\nGALEPHB COM1 ONCHANGED\r\nQZSSEPHB COM1\r\nQZSSEPHB COM1 ONCHANGED\r\nIRNSSEPHB COM1\r\nIRNSSEPHB COM1 ONCHANGED\r\nBD3EPHB COM1\r\nBD3EPHB COM1 ONCHANGED\r\n";

/// Confirms CRC-valid receiver epochs, including no-fix epochs indoors.
pub async fn configure(
    command: &mut Uart<'_, Async>,
    data: &mut Uart<'_, Async>,
) -> Result<(), &'static str> {
    if !probe_split_path(command, data).await {
        return Err("version-probe-failed");
    }
    configure_split_path(command, data).await
}
async fn probe_split_path(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
) -> bool {
    for command_baud in [115_200, COMMAND_BAUD] {
        if set_baud(command_uart, command_baud).is_err() {
            continue;
        }
        // 921600 remains a migration probe for the earlier high-rate stage image.
        for data_baud in [115_200, DATA_BAUD, 921_600] {
            if set_baud(data_uart, data_baud).is_err() {
                continue;
            }
            drain(data_uart);
            if write(
                command_uart,
                b"\r\nUNLOG COM1\r\nUNLOG COM2\r\nVERSIONA COM2\r\n",
            )
            .await
            .is_err()
            {
                continue;
            }
            if with_timeout(Duration::from_secs(2), async {
                let mut decoder = LineDecoder::<1024>::new();
                let mut chunk = [0; 128];
                loop {
                    let length = match embedded_io_async::Read::read(data_uart, &mut chunk).await {
                        Ok(length) => length,
                        Err(_) => continue,
                    };
                    for byte in &chunk[..length] {
                        if let Some(line) = decoder.push(*byte)
                            && valid_unicore_log(line, b"#VERSIONA,")
                        {
                            return;
                        }
                    }
                }
            })
            .await
            .is_ok()
            {
                drain(command_uart);
                return true;
            }
        }
    }
    false
}

async fn configure_split_path(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
) -> Result<(), &'static str> {
    write(command_uart, b"CONFIG SIGNALGROUP 8\r\n").await?;
    Timer::after_secs(3).await;
    let mut ready = false;
    for _ in 0..3 {
        if probe_split_path(command_uart, data_uart).await {
            ready = true;
            break;
        }
        Timer::after_millis(500).await;
    }
    if !ready {
        return Err("no VERSIONA COM2 after SIGNALGROUP request");
    }
    // Send complete command lines together: awaiting between command text and
    // CRLF can exceed the receiver's input timeout under scheduler pressure.
    write(command_uart, b"UNLOG COM1\r\nUNLOG COM2\r\nMODE ROVER AUTOMOTIVE\r\nCONFIG PVTALG AUTO\r\nCONFIG ANTIJAM AUTO\r\nCONFIG COM2 460800\r\n").await?;
    // COM2 sends 148-byte SPP frames. A 32-byte watermark leaves room in the
    // FIFO for interrupt latency and lets RX-idle deliver the tail as before.
    set_baud_with_threshold(data_uart, DATA_BAUD, 32)?;
    drain(data_uart);
    write(command_uart, b"CONFIG COM1 460800\r\n").await?;
    set_baud_with_threshold(command_uart, COMMAND_BAUD, 64)?;
    drain(command_uart);
    write(command_uart, b"SPPNAVB COM2 0.02\r\n").await?;
    // Confirm the requested receiver epoch spacing, even if the
    // assembled board cannot return CONFIG or acknowledgement lines on COM1.
    with_timeout(Duration::from_secs(8), async {
        let mut decoder = crate::timing::wire::Decoder::<256>::new();
        let mut chunk = [0; 256];
        let mut rate = StandaloneRateCheck::default();
        loop {
            let Ok(length) = embedded_io_async::Read::read(data_uart, &mut chunk).await else {
                continue;
            };
            for byte in &chunk[..length] {
                let Ok(Some(crate::timing::wire::Frame::SppNav(nav))) = decoder.push(*byte) else {
                    continue;
                };
                if nav.time_reference == 0
                    && rate.observe_epoch(u32::from(nav.gps_week), nav.gps_tow_ms)
                {
                    return;
                }
            }
        }
    })
    .await
    .map_err(|_| "50Hz-SPP-binary-stream-not-observed")?;
    // Volatile timing profile. The live task separately requires a complete
    // checksummed CONFIG readback before it accepts PPSSTATUS labels.
    write(
        command_uart,
        b"CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0\r\nPPSSTATUSB COM2 1\r\n",
    )
    .await?;
    Ok(())
}

pub fn set_baud(uart: &mut Uart<'_, Async>, baud: u32) -> Result<(), &'static str> {
    set_baud_with_threshold(uart, baud, 16)
}
fn set_baud_with_threshold(
    uart: &mut Uart<'_, Async>,
    baud: u32,
    threshold: u16,
) -> Result<(), &'static str> {
    uart.apply_config(
        &Config::default()
            .with_baudrate(baud)
            // Hardware batching keeps reception armed; RX-idle timeout still
            // delivers short replies and raw observation frames promptly.
            .with_rx(RxConfig::default().with_fifo_full_threshold(threshold)),
    )
    .map_err(|_| "ESP UART configuration failed")
}

fn drain(uart: &mut Uart<'_, Async>) {
    let mut chunk = [0; 128];
    // Bound this operation even if an old logging profile is flooding COM1.
    for _ in 0..32 {
        if matches!(uart.read_buffered(&mut chunk), Ok(0)) {
            break;
        }
    }
}

pub async fn write(uart: &mut Uart<'_, Async>, bytes: &[u8]) -> Result<(), &'static str> {
    with_timeout(Duration::from_millis(500), async {
        embedded_io_async::Write::write_all(uart, bytes)
            .await
            .map_err(|_| "UM980 command UART write failed")?;
        embedded_io_async::Write::flush(uart)
            .await
            .map_err(|_| "UM980 command UART flush failed")
    })
    .await
    .map_err(|_| "command-write-timeout")?
}
