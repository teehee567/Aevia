//! UM980 profile for the live trajectory proof of concept.
//!
//! Unicore N4 command manual R1.14, CONFIG SIGNALGROUP/COM and BESTNAV:
//! https://en.unicorecomm.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf
//! Group 8 is required for 50 Hz PVT. Unlike the remaining runtime settings,
//! SIGNALGROUP saves itself and restarts the receiver when its value changes.

use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    Async,
    uart::{Config, RxConfig, Uart},
};

use crate::gps::{
    LineDecoder, parse_bestnava, parse_bestnava_status, parse_command_ack, parse_signalgroup,
    valid_unicore_log,
};

pub const DATA_BAUD: u32 = 460_800;

#[derive(Clone, Copy, Debug)]
pub struct ProfileReady {
    pub command_port: u8,
    pub command_baud: u32,
    pub signalgroup: u8,
    pub readback_verified: bool,
    pub target_rate_hz: u32,
}

/// Useful standalone profile for receiver firmware whose group-8 SPP engine
/// only computes at 1 Hz. Requires valid position AND velocity at 50 ms epochs.
pub async fn configure_20hz_standalone(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
) -> Result<ProfileReady, &'static str> {
    if !probe_split_path(command_uart, data_uart).await {
        return Err("COM1 to COM2 VERSIONA probe failed");
    }
    configure_split_path(command_uart, data_uart, 1, 50, true).await
}

/// Apply and read back the receiver profile, tolerating a previous firmware's
/// volatile COM1 baud rate and the restart caused by selecting signal group 8.
/// A successful result confirms configuration, not satellite lock or observed
/// rate: callers must also count CRC-valid BESTNAV receiver epoch increments.
pub async fn configure_50hz(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
) -> Result<ProfileReady, &'static str> {
    // First try the previously demonstrated TX1 -> receiver -> RX2 route. A
    // CRC-valid VERSIONA explicitly routed to COM2 proves command execution
    // without depending on an acknowledgement returning over COM1.
    if probe_split_path(command_uart, data_uart).await {
        return configure_split_path(command_uart, data_uart, 8, 20, false).await;
    }
    // Some assembled test boards provide the working command TX on COM1 but
    // no usable return path. COM2 is bidirectional in the default RXTYPE AUTO
    // mode, so use its independently verified RX path if COM1 cannot reply.
    if let Ok(command_baud) = discover_command_baud(command_uart, 1).await {
        let command_baud = configure_receiver(command_uart, 1, command_baud).await?;
        command(command_uart, b"CONFIG COM2 460800").await?;
        set_baud(data_uart, DATA_BAUD)?;
        drain(data_uart);
        command(command_uart, b"BESTNAVA COM2 0.02").await?;
        return Ok(ProfileReady {
            command_port: 1,
            command_baud,
            signalgroup: 8,
            readback_verified: true,
            target_rate_hz: 50,
        });
    }

    let command_baud = discover_command_baud(data_uart, 2).await?;
    configure_receiver(data_uart, 2, command_baud).await?;
    // The acknowledgement may be transmitted at either baud during the port
    // transition. Reconnect with a fresh command at the target rate instead.
    write(data_uart, b"CONFIG COM2 460800\r\n").await?;
    set_baud(data_uart, DATA_BAUD)?;
    Timer::after_millis(100).await;
    command(data_uart, b"UNLOG COM2").await?;
    command(data_uart, b"BESTNAVA COM2 0.02").await?;
    Ok(ProfileReady {
        command_port: 2,
        command_baud: DATA_BAUD,
        signalgroup: 8,
        readback_verified: true,
        target_rate_hz: 50,
    })
}

async fn probe_split_path(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
) -> bool {
    if set_baud(command_uart, 115_200).is_err() {
        return false;
    }
    for data_baud in [115_200, DATA_BAUD] {
        if set_baud(data_uart, data_baud).is_err() {
            continue;
        }
        drain(data_uart);
        if write(command_uart, b"\r\nUNLOG COM2\r\nVERSIONA COM2\r\n")
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
                    if let Some(line) = decoder.push(*byte) {
                        if valid_unicore_log(line, b"#VERSIONA,") {
                            return;
                        }
                    }
                }
            }
        })
        .await
        .is_ok()
        {
            return true;
        }
    }
    false
}

async fn configure_split_path(
    command_uart: &mut Uart<'_, Async>,
    data_uart: &mut Uart<'_, Async>,
    signalgroup: u8,
    epoch_ms: u32,
    require_valid: bool,
) -> Result<ProfileReady, &'static str> {
    let group_command: &[u8] = if signalgroup == 1 {
        b"CONFIG SIGNALGROUP 1\r\n"
    } else {
        b"CONFIG SIGNALGROUP 8\r\n"
    };
    write(command_uart, group_command).await?;
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
    set_baud(data_uart, DATA_BAUD)?;
    drain(data_uart);
    let log_command: &[u8] = if epoch_ms == 50 {
        b"BESTNAVA COM2 0.05\r\n"
    } else {
        b"BESTNAVA COM2 0.02\r\n"
    };
    write(command_uart, log_command).await?;
    // Confirm the requested receiver epoch spacing, even if the
    // assembled board cannot return CONFIG or acknowledgement lines on COM1.
    with_timeout(Duration::from_secs(8), async {
        let mut decoder = LineDecoder::<1024>::new();
        let mut chunk = [0; 256];
        let mut previous = None;
        let mut matching_steps = 0;
        loop {
            let Ok(length) = embedded_io_async::Read::read(data_uart, &mut chunk).await else {
                continue;
            };
            for byte in &chunk[..length] {
                let Some(line) = decoder.push(*byte) else {
                    continue;
                };
                if require_valid && parse_bestnava(line).is_err() {
                    continue;
                }
                let Ok(status) = parse_bestnava_status(line) else {
                    continue;
                };
                let epoch = u64::from(status.gps_week) * 604_800_000 + u64::from(status.gps_tow_ms);
                if previous.is_some_and(|value| epoch == value + u64::from(epoch_ms)) {
                    matching_steps += 1;
                } else {
                    matching_steps = 0;
                }
                previous = Some(epoch);
                if matching_steps >= 10 {
                    return;
                }
            }
        }
    })
    .await
    .map_err(|_| {
        if require_valid {
            "valid 20Hz standalone stream not observed"
        } else {
            "50Hz stream not observed after COM1 profile request"
        }
    })?;
    Ok(ProfileReady {
        command_port: 1,
        command_baud: 115_200,
        signalgroup,
        readback_verified: false,
        target_rate_hz: 1000 / epoch_ms,
    })
}

async fn configure_receiver(
    command_uart: &mut Uart<'_, Async>,
    port: u8,
    mut command_baud: u32,
) -> Result<u32, &'static str> {
    if signalgroup(command_uart).await != Some(8) {
        // Some receiver versions restart before sending their acknowledgement.
        // Readback after reconnect, rather than write success, is authoritative.
        match command(command_uart, b"CONFIG SIGNALGROUP 8").await {
            Ok(()) | Err("command response timeout") => {}
            Err(error) => return Err(error),
        }
        Timer::after(Duration::from_secs(2)).await;
        let mut reconnected = false;
        for _ in 0..3 {
            if let Ok(baud) = discover_command_baud(command_uart, port).await {
                command_baud = baud;
                reconnected = true;
                break;
            }
        }
        if !reconnected {
            return Err("UM980 did not reconnect after SIGNALGROUP restart");
        }
        if signalgroup(command_uart).await != Some(8) {
            return Err("UM980 SIGNALGROUP 8 readback failed");
        }
    }

    command(command_uart, b"UNLOG COM2").await?;
    command(command_uart, b"MODE ROVER AUTOMOTIVE").await?;
    command(command_uart, b"CONFIG PVTALG AUTO").await?;
    command(command_uart, b"CONFIG ANTIJAM AUTO").await?;
    // A typical ASCII BESTNAV frame is ~350 bytes: ~17.5 kB/s at 50 Hz,
    // above the 11.52 kB/s capacity of 115200 8N1. 460800 leaves >2x headroom.
    Ok(command_baud)
}

async fn discover_command_baud(uart: &mut Uart<'_, Async>, port: u8) -> Result<u32, &'static str> {
    let unlog: &[u8] = if port == 1 {
        b"UNLOG COM1"
    } else {
        b"UNLOG COM2"
    };
    for baud in [
        115_200, 460_800, 921_600, 230_400, 57_600, 38_400, 19_200, 9_600,
    ] {
        set_baud(uart, baud)?;
        drain(uart);
        // Clear any partial command left by probing at a different baud.
        write(uart, b"\r\n").await?;
        if command(uart, unlog).await.is_ok() {
            return Ok(baud);
        }
    }
    Err("UM980 command port not found at supported baud rates")
}

fn set_baud(uart: &mut Uart<'_, Async>, baud: u32) -> Result<(), &'static str> {
    uart.apply_config(
        &Config::default()
            .with_baudrate(baud)
            .with_rx(RxConfig::default().with_fifo_full_threshold(64)),
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

async fn write(uart: &mut Uart<'_, Async>, bytes: &[u8]) -> Result<(), &'static str> {
    embedded_io_async::Write::write_all(uart, bytes)
        .await
        .map_err(|_| "UM980 command UART write failed")?;
    embedded_io_async::Write::flush(uart)
        .await
        .map_err(|_| "UM980 command UART flush failed")
}

async fn command(uart: &mut Uart<'_, Async>, text: &[u8]) -> Result<(), &'static str> {
    drain(uart);
    let mut line = heapless::Vec::<u8, 96>::new();
    line.extend_from_slice(text)
        .map_err(|_| "command too long")?;
    line.extend_from_slice(b"\r\n")
        .map_err(|_| "command too long")?;
    write(uart, &line).await?;
    let mut decoder = LineDecoder::<512>::new();
    let mut chunk = [0; 128];
    let result = with_timeout(Duration::from_millis(600), async {
        loop {
            let length = match embedded_io_async::Read::read(uart, &mut chunk).await {
                Ok(length) => length,
                // Framing/overflow from probing or a prior saved log must not
                // permanently prevent recovery of the next complete response.
                Err(_) => continue,
            };
            for byte in &chunk[..length] {
                if let Some(line) = decoder.push(*byte) {
                    if let Some(ok) = parse_command_ack(line, text) {
                        return if ok {
                            Ok(())
                        } else {
                            Err("UM980 rejected configuration command")
                        };
                    }
                }
            }
        }
    })
    .await;
    result.map_err(|_| "command response timeout")?
}

async fn signalgroup(uart: &mut Uart<'_, Async>) -> Option<u8> {
    drain(uart);
    write(uart, b"CONFIG\r\n").await.ok()?;
    let mut decoder = LineDecoder::<512>::new();
    let mut chunk = [0; 128];
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut group = None;
    // Consume the full configuration response so its trailing settings cannot
    // fill the hardware FIFO while the next command is sent.
    while Instant::now() < deadline {
        let length = match with_timeout(
            Duration::from_millis(250),
            embedded_io_async::Read::read(uart, &mut chunk),
        )
        .await
        {
            Ok(Ok(length)) => length,
            Ok(Err(_)) => continue,
            Err(_) => break,
        };
        for byte in &chunk[..length] {
            if let Some(line) = decoder.push(*byte) {
                if let Some(value) = parse_signalgroup(line) {
                    group = Some(value);
                }
            }
        }
    }
    group
}
