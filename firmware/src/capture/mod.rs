//! Hardware-latched acquisition clock and portable wrap validation.
pub mod counter;
#[cfg(target_arch = "riscv32")]
mod hardware;
#[cfg(target_arch = "riscv32")]
pub use hardware::*;
