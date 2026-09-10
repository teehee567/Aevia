use std::{env, error::Error, io};

fn main() -> Result<(), Box<dyn Error>> {
    let port = env::args().nth(1).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, "usage: s31-reset <COM port>")
    })?;
    s31_reset::reset_rom(&port)?;
    println!("ESP32-S31 watchdog reset requested on {port}");
    Ok(())
}
