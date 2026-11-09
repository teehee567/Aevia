pub mod interface;
pub mod timing;

#[cfg(target_arch = "riscv32")]
mod gnss;
#[cfg(target_arch = "riscv32")]
mod imu;
#[cfg(target_arch = "riscv32")]
pub mod processing;
#[cfg(target_arch = "riscv32")]
mod profile;
#[cfg(target_arch = "riscv32")]
pub(crate) mod sensors;
#[cfg(target_arch = "riscv32")]
mod status;
#[cfg(target_arch = "riscv32")]
pub(crate) mod storage;
#[cfg(target_arch = "riscv32")]
pub(crate) mod usb;

use crate::{
    commands::{Command, Response},
    loader::update::{Error, Flash, Updater},
};

pub fn command<F: Flash>(
    updater: &mut Updater<F>,
    command: Command<'_>,
) -> Result<Response, Error> {
    match command {
        Command::Info => Ok(Response::Info {
            recovery: false,
            max_size: 0,
            chunk: 0,
        }),
        Command::Recover => {
            updater.request_recovery()?;
            Ok(Response::Reboot)
        }
        _ => Err(Error::Command),
    }
}
