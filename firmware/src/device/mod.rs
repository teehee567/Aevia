#[cfg(target_arch = "riscv32")]
pub(crate) mod board;
pub mod configuration;
pub mod interface;
#[cfg(test)]
mod interface_tests;
#[cfg(any(target_arch = "riscv32", test))]
pub(crate) mod platform;
pub(crate) mod power;
pub mod recording;
#[cfg(any(target_arch = "riscv32", test))]
pub(crate) mod supervision;
pub mod timing;

#[cfg(target_arch = "riscv32")]
pub(crate) mod display;
#[cfg(any(target_arch = "riscv32", test))]
mod gnss;
#[cfg(any(target_arch = "riscv32", test))]
mod imu;
#[cfg(target_arch = "riscv32")]
pub(crate) mod maintenance;
#[cfg(target_arch = "riscv32")]
pub mod processing;
#[cfg(any(target_arch = "riscv32", test))]
mod processing_support;
mod profile;
#[cfg(target_arch = "riscv32")]
pub(crate) mod sensors;
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
        Command::ProtocolInfo => Ok(Response::ProtocolInfo(crate::protocol::info(false))),
        Command::Recover => {
            #[cfg(target_arch = "riscv32")]
            if recording::recording_active() {
                return Err(Error::Command);
            }
            updater.request_recovery()?;
            Ok(Response::Reboot)
        }
        #[cfg(target_arch = "riscv32")]
        Command::BootStatus => Ok(Response::BootStatus {
            stage: crate::runtime::startup_stage(),
        }),
        _ => Err(Error::Command),
    }
}
