#[cfg(target_arch = "riscv32")]
pub(crate) mod storage;
#[cfg(target_arch = "riscv32")]
pub(crate) mod usb;

use core::str::from_utf8;

use crate::loader::{recovery::Reply, update::Error};


// standard firmware


pub fn command(line: &[u8]) -> Result<Reply, Error> {
    match from_utf8(line)
        .map_err(|_| Error::Command)?
        .trim()
    {
        "INFO" => Ok(Reply::Text(b"APP 1\n")),
        "RECOVER" => Ok(Reply::Reboot),
        _ => Err(Error::Command),
    }
}
