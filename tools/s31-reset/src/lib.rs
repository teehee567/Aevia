//! S31 ROM reset shared by the standalone reset command and flash runner.
use espflash::{
    connection::{Connection, ResetAfterOperation, ResetBeforeOperation},
    target::Chip,
};
use serialport::{SerialPort, UsbPortInfo};
use std::{error::Error, time::Duration};

pub const VID: u16 = 0x303a;
pub const APP_PID: u16 = 0x4001;
pub const ROM_PID: u16 = 0x0020;

pub fn reset_rom(port_name: &str) -> Result<(), Box<dyn Error>> {
    let mut serial = serialport::new(port_name, 115_200)
        .timeout(Duration::from_secs(3))
        .open_native()?;
    serial.write_data_terminal_ready(true)?;
    let info = UsbPortInfo {
        interface: None,
        vid: VID,
        pid: ROM_PID,
        serial_number: None,
        manufacturer: None,
        product: None,
    };
    let mut connection = Connection::new(
        serial,
        info,
        ResetAfterOperation::HardReset,
        ResetBeforeOperation::NoReset,
        115_200,
    );
    connection.begin()?;
    // espflash 4.5 CLI omits S31 in its watchdog-reset dispatch. The target's
    // library method supports it and works after exiting the RAM flash stub.
    Chip::Esp32s31.rtc_wdt_reset(&mut connection)?;
    Ok(())
}
