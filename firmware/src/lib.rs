//! Portable firmware protocols and policies. Host tests exercise these same modules.
#![no_std]
#[cfg(test)]
extern crate std;

pub mod battery;
pub mod buttons;
pub mod command;
pub mod gnss;
pub mod imu;
#[cfg(any(test, target_arch = "riscv32"))]
pub mod memory;
