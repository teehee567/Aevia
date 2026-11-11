//! Race recording is independent of estimator readiness and USB connections.
pub mod api;
#[cfg(any(target_arch = "riscv32", test))]
mod control;
pub mod filesystem;
pub mod format;
#[cfg(any(target_arch = "riscv32", test))]
mod queue;

#[cfg(target_arch = "riscv32")]
mod hardware;
#[cfg(target_arch = "riscv32")]
pub(crate) use hardware::{capture, command, confirmation_ready, recording_active, run};
