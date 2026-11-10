//! PPS-labelled GPS epochs mapped onto the same free-running clock as IMU DRDY.
//!
//! UART arrival time is used only to disambiguate the first pulse label. Once
//! locked, pulse association and observation epochs use the fitted hardware
//! counter. The session timeline never steps when the fit changes.

#[cfg(target_arch = "riscv32")]
pub(super) use super::platform::capture;

#[cfg(any(target_arch = "riscv32", test))]
mod clock;
#[cfg(target_arch = "riscv32")]
pub(super) use clock::{ClockSync, PpsLabel};

/// Timing diagnostics are estimates, not a hardware accuracy qualification.
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct TimingStatus {
    pub pps_locked: bool,
    pub pps_edges: u32,
    pub pps_pairs: u32,
    pub pps_rejected: u32,
    pub pps_age_ms: Option<u32>,
    pub fit_residual_ns: u32,
    pub generation: u32,
    pub capture_errors: u32,
    pub imu_timing_rejected: u32,
    pub gnss_unsynchronized: u32,
}

#[cfg(target_arch = "riscv32")]
mod shared {
    use super::*;
    use core::{
        cell::RefCell,
        sync::atomic::{AtomicU32, Ordering},
    };
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

    pub static CLOCK: Mutex<CriticalSectionRawMutex, RefCell<ClockSync>> =
        Mutex::new(RefCell::new(ClockSync::new()));
    pub static UNSYNCHRONIZED: AtomicU32 = AtomicU32::new(0);

    pub fn observe_pulse() {
        if let Some(at) = capture::pps_ns() {
            CLOCK.lock(|clock| clock.borrow_mut().pulse(at));
        }
    }

    pub fn status() -> TimingStatus {
        let now = capture::now_ns();
        let mut status = CLOCK.lock(|clock| {
            let mut clock = clock.borrow_mut();
            if now.is_none() && clock.status(0).pps_locked {
                clock.invalidate();
            }
            clock.status(now.unwrap_or(0))
        });
        if now.is_none() {
            status.pps_age_ms = None;
        }
        status.capture_errors = capture::ERRORS.load(Ordering::Relaxed);
        status.imu_timing_rejected =
            crate::device::sensors::IMU_TIMING_ERRORS.load(Ordering::Relaxed);
        status.gnss_unsynchronized = UNSYNCHRONIZED.load(Ordering::Relaxed);
        status
    }
}

#[cfg(target_arch = "riscv32")]
pub(super) use shared::{CLOCK, UNSYNCHRONIZED, observe_pulse, status};

#[cfg(test)]
mod tests;
