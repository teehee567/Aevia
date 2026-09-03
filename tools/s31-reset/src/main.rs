use std::{env, error::Error, io, time::Duration};

use espflash::{
    connection::{Connection, ResetAfterOperation, ResetBeforeOperation},
    target::Chip,
};
use serialport::UsbPortInfo;

const ESPRESSIF_VID: u16 = 0x303a;
const ESP32_S31_ROM_PID: u16 = 0x0020;
const ROM_BAUD: u32 = 115_200;

fn main() -> Result<(), Box<dyn Error>> {
    let port_name = env::args().nth(1).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, "usage: s31-reset <COM port>")
    })?;

    let serial = serialport::new(&port_name, ROM_BAUD)
        .timeout(Duration::from_secs(3))
        .open_native()?;
    let port_info = UsbPortInfo {
        vid: ESPRESSIF_VID,
        pid: ESP32_S31_ROM_PID,
        serial_number: None,
        manufacturer: Some("Espressif".into()),
        product: Some("ESP32-S31 ROM".into()),
    };
    let mut connection = Connection::new(
        serial,
        port_info,
        ResetAfterOperation::HardReset,
        ResetBeforeOperation::NoReset,
        ROM_BAUD,
    );

    connection.begin()?;
    Chip::Esp32s31.rtc_wdt_reset(&mut connection)?;
    println!("ESP32-S31 watchdog reset requested on {port_name}");
    Ok(())
}
