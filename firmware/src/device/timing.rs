//! PPS-labelled GPS epochs mapped onto the same free-running clock as IMU DRDY.
//!
//! UART arrival time is used only to disambiguate the first pulse label. Once
//! locked, pulse association and observation epochs use the fitted hardware
//! counter. The session timeline never steps when the fit changes.

#[cfg(target_arch = "riscv32")]
pub(super) use super::platform::capture;

#[cfg(any(target_arch = "riscv32", test))]
mod clock;
#[cfg(any(target_arch = "riscv32", test))]
mod shared_clock;
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
    use core::sync::atomic::{AtomicU32, Ordering};

    pub static CLOCK: shared_clock::SharedClock = shared_clock::SharedClock::new();
    pub static UNSYNCHRONIZED: AtomicU32 = AtomicU32::new(0);
    static LAST_RECORDED_PULSE: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        core::cell::Cell<(u64, u32)>,
    > = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new((0, 0)));

    pub fn observe_pulse() {
        if let Some((at, raw_ticks)) = capture::pps_evidence() {
            let sequence = LAST_RECORDED_PULSE.lock(|cell| {
                let (last, sequence) = cell.get();
                if at <= last {
                    return None;
                }
                let sequence = sequence.wrapping_add(1);
                cell.set((at, sequence));
                Some(sequence)
            });
            if let Some(sequence) = sequence {
                let mut payload = [0_u8; 24];
                payload[0] = 1;
                payload[4..12].copy_from_slice(&at.to_le_bytes());
                payload[12..16].copy_from_slice(&raw_ticks.to_le_bytes());
                payload[16..20].copy_from_slice(&sequence.to_le_bytes());
                payload[20..24]
                    .copy_from_slice(&capture::ERRORS.load(Ordering::Relaxed).to_le_bytes());
                crate::device::recording::capture(5, at, &payload);
            }
            let _ = CLOCK.update(|clock| clock.pulse(at));
        }
    }

    pub fn status() -> TimingStatus {
        let now = capture::now_ns();
        let mut status = CLOCK
            .update(|clock| {
                if now.is_none() && clock.status(0).pps_locked {
                    clock.invalidate();
                }
                clock.status(now.unwrap_or(0))
            })
            .unwrap_or_else(|status| status);
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
