//! Receiver lifecycle and PPS verification policy.
mod diagnostics;
mod navigation;
pub(super) mod observation;
#[cfg(target_arch = "riscv32")]
mod receiver;
pub(super) mod transport;
mod verification;
#[cfg(target_arch = "riscv32")]
pub(super) use receiver::task;

#[cfg(target_arch = "riscv32")]
pub use super::interface::GnssStatus;

#[cfg(target_arch = "riscv32")]
static STATUS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<GnssStatus>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(GnssStatus {
    config_verified: false,
    config_queries: 0,
    config_timeouts: 0,
    navigation_time_ready: false,
    last_pps_quality: None,
    last_parser_error: None,
}));

#[cfg(target_arch = "riscv32")]
pub(super) fn status() -> GnssStatus {
    #[cfg(target_arch = "riscv32")]
    {
        STATUS.lock(core::cell::Cell::get)
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        GnssStatus::default()
    }
}

#[cfg(target_arch = "riscv32")]
fn publish(status: GnssStatus) {
    STATUS.lock(|value| value.set(status));
}
