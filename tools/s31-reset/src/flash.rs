//! Button-free application -> ROM -> flash -> application round trip on J6.
use s31_reset::{APP_PID, ROM_PID, VID};
use serialport::{SerialPortInfo, SerialPortType};
use std::{
    env,
    error::Error,
    io::{self, Read, Write},
    path::PathBuf,
    process::Command,
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Options {
    elf: PathBuf,
    app: Option<String>,
    rom: Option<String>,
}

fn options() -> Result<Options> {
    let mut args = env::args().skip(1);
    let Some(elf) = args.next() else {
        return Err(usage().into());
    };
    if elf == "--help" || elf == "-h" {
        println!("{}", usage());
        std::process::exit(0);
    }
    let mut options = Options {
        elf: elf.into(),
        app: None,
        rom: None,
    };
    while let Some(flag) = args.next() {
        let value = args.next().ok_or_else(|| usage().to_owned())?;
        match flag.as_str() {
            "--app-port" => options.app = Some(value),
            "--rom-port" => options.rom = Some(value),
            _ => return Err(usage().into()),
        }
    }
    options.elf = options.elf.canonicalize()?;
    if !options.elf.is_file() {
        return Err("firmware ELF is not a file".into());
    }
    // Check before asking the running board to leave application mode.
    let mut file = std::fs::File::open(&options.elf)?;
    let mut magic = [0; 4];
    file.read_exact(&mut magic)?;
    if magic != *b"\x7fELF" {
        return Err("firmware input must be an ELF".into());
    }
    Ok(options)
}

fn usage() -> &'static str {
    "usage: aevia-flash <firmware ELF> [--app-port COM5] [--rom-port COM4]"
}

fn choose(ports: &[SerialPortInfo], pid: u16, explicit: Option<&str>) -> Result<Option<String>> {
    let matches: Vec<_> = ports
        .iter()
        .filter(|port| {
            matches!(&port.port_type, SerialPortType::UsbPort(info)
            if info.vid == VID && info.pid == pid
                && (pid != APP_PID || info.interface.is_none_or(|interface| interface < 2)))
                && explicit.is_none_or(|name| port.port_name.eq_ignore_ascii_case(name))
        })
        .collect();
    match matches.as_slice() {
        [] => Ok(None),
        [port] => Ok(Some(port.port_name.clone())),
        _ => Err(format!(
            "multiple USB devices with PID {pid:04x}; specify --app-port and --rom-port"
        )
        .into()),
    }
}

fn wait_port(pid: u16, explicit: Option<&str>) -> Result<String> {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        // Windows can lose a device between enumeration and opening its
        // properties while ROM/application interfaces are being replaced.
        if let Ok(ports) = serialport::available_ports()
            && let Some(port) = choose(&ports, pid, explicit)?
        {
            return Ok(port);
        }
        if Instant::now() >= deadline {
            return Err(
                format!("USB {VID:04x}:{pid:04x} did not enumerate within 20 seconds").into(),
            );
        }
        thread::sleep(Duration::from_millis(200));
    }
}

fn request_bootloader(port: &str) -> Result<()> {
    let mut serial = serialport::new(port, 115_200)
        .timeout(Duration::from_secs(2))
        .open()?;
    serial.write_data_terminal_ready(true)?;
    // Leading newline discards a partial command from a prior host process.
    serial.write_all(b"\nBOOTLOADER\n")?;
    // Windows may report device removal from flush after the accepted command.
    // Enumeration below is authoritative; do not retry commands on another port.
    let _ = serial.flush();
    Ok(())
}

fn verify_application(port: &str) -> Result<()> {
    // Windows may publish the COM interface before it is ready to be opened.
    let open_deadline = Instant::now() + Duration::from_secs(5);
    let mut serial = loop {
        match serialport::new(port, 115_200)
            .timeout(Duration::from_millis(500))
            .open()
        {
            Ok(serial) => break serial,
            Err(error) if Instant::now() >= open_deadline => return Err(error.into()),
            Err(_) => thread::sleep(Duration::from_millis(200)),
        }
    };
    serial.write_data_terminal_ready(true)?;
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut line = Vec::new();
    let mut bytes = [0; 1024];
    let mut first_uptime = None;
    while Instant::now() < deadline {
        let length = match serial.read(&mut bytes) {
            Ok(length) => length,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                ) =>
            {
                continue;
            }
            Err(error) => return Err(error.into()),
        };
        for &byte in &bytes[..length] {
            if byte == b'\n' {
                if let Ok(text) = std::str::from_utf8(&line)
                    && let Some(uptime) = base_uptime(text)
                {
                    println!("{}", text.trim_end());
                    match first_uptime {
                        Some(previous) if uptime > previous => return Ok(()),
                        Some(_) => {
                            return Err(
                                "application restarted or stopped advancing after flashing".into(),
                            );
                        }
                        None => first_uptime = Some(uptime),
                    }
                }
                line.clear();
            } else if line.len() < 4096 {
                line.push(byte);
            }
        }
    }
    Err("application USB appeared but two advancing base STATUS records were not received".into())
}

fn base_uptime(line: &str) -> Option<u64> {
    if !line.starts_with("STATUS ") {
        return None;
    }
    let fields: Vec<_> = line.split_ascii_whitespace().collect();
    if !fields.contains(&"firmware=base") || !fields.contains(&"protocol=1") {
        return None;
    }
    fields
        .iter()
        .find_map(|field| field.strip_prefix("uptime-ms=")?.parse().ok())
}

fn main() -> Result<()> {
    let options = options()?;
    let version = Command::new("espflash").arg("--version").output()?;
    if !version.status.success()
        || !String::from_utf8_lossy(&version.stdout)
            .trim()
            .ends_with("4.5.0")
    {
        return Err("this S31 flash sequence requires espflash 4.5.0".into());
    }
    let ports = serialport::available_ports()?;
    let app = choose(&ports, APP_PID, options.app.as_deref())?;
    let existing_rom = choose(&ports, ROM_PID, options.rom.as_deref())?;
    if app.is_some() && existing_rom.is_some() {
        return Err(
            "both application and ROM devices are present; connect only the board being flashed"
                .into(),
        );
    }
    let rom = if let Some(app) = app {
        println!("Requesting ROM download on {app}");
        request_bootloader(&app)?;
        wait_port(ROM_PID, options.rom.as_deref())?
    } else if let Some(rom) = existing_rom {
        rom
    } else {
        return Err("no V2 Mini application or S31 ROM USB found on J6".into());
    };

    println!("Flashing {} on {rom}", options.elf.display());
    let status = Command::new("espflash")
        .args([
            "flash",
            "--port",
            &rom,
            "--chip",
            "esp32s31",
            "--before",
            "no-reset",
            "--after",
            "no-reset-no-stub",
            "--flash-size",
            "16mb",
            "--non-interactive",
            "--skip-update-check",
        ])
        .arg(&options.elf)
        .status()?;
    if !status.success() {
        return Err(
            "espflash failed; board remains in download mode, correct the error and rerun".into(),
        );
    }
    if let Err(error) = s31_reset::reset_rom(&rom) {
        // The watchdog may remove ROM USB before Windows completes the final
        // transaction. Advancing application records below still prove success.
        eprintln!("ROM connection closed during reset ({error}); checking application restart");
    }
    let app = wait_port(APP_PID, options.app.as_deref())?;
    verify_application(&app)?;
    println!("PASS: flashed and restarted without buttons; base firmware responds on {app}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn port(name: &str, vid: u16, pid: u16) -> SerialPortInfo {
        SerialPortInfo {
            port_name: name.into(),
            port_type: SerialPortType::UsbPort(serialport::UsbPortInfo {
                vid,
                pid,
                serial_number: None,
                manufacturer: None,
                product: None,
                interface: None,
            }),
        }
    }
    #[test]
    fn discovers_reenumerated_port_by_usb_identity() {
        let ports = [port("COM19", VID, ROM_PID), port("COM5", 0x1234, APP_PID)];
        assert_eq!(
            choose(&ports, ROM_PID, None).unwrap().as_deref(),
            Some("COM19")
        );
        assert_eq!(choose(&ports, APP_PID, None).unwrap(), None);
        assert_eq!(choose(&ports, ROM_PID, Some("COM4")).unwrap(), None);
    }
    #[test]
    fn ambiguous_boards_require_explicit_selection() {
        let ports = [port("COM5", VID, APP_PID), port("COM6", VID, APP_PID)];
        assert!(choose(&ports, APP_PID, None).is_err());
        assert_eq!(
            choose(&ports, APP_PID, Some("com6")).unwrap().as_deref(),
            Some("COM6")
        );
    }
    #[test]
    fn composite_gnss_interface_is_never_selected_for_flashing() {
        let mut console = port("COM5", VID, APP_PID);
        let mut gnss = port("COM6", VID, APP_PID);
        if let SerialPortType::UsbPort(info) = &mut console.port_type {
            info.interface = Some(0);
        }
        if let SerialPortType::UsbPort(info) = &mut gnss.port_type {
            info.interface = Some(2);
        }
        let ports = [gnss, console];
        assert_eq!(
            choose(&ports, APP_PID, None).unwrap().as_deref(),
            Some("COM5")
        );
        assert_eq!(choose(&ports, APP_PID, Some("COM6")).unwrap(), None);
    }
    #[test]
    fn post_flash_verification_requires_base_protocol_and_uptime() {
        assert_eq!(
            base_uptime("STATUS firmware=base protocol=1 uptime-ms=123\r"),
            Some(123)
        );
        assert_eq!(base_uptime("STATUS imu=ready"), None);
        assert_eq!(
            base_uptime("STATUS firmware=base protocol=2 uptime-ms=123"),
            None
        );
        assert_eq!(
            base_uptime("RAW STATUS firmware=base protocol=1 uptime-ms=123"),
            None
        );
        assert_eq!(
            base_uptime("STATUS firmware=base protocol=1 uptime-ms=oops"),
            None
        );
    }
}
