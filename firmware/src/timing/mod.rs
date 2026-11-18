//! PPS-labelled receiver epochs on the hardware IMU capture clock.
mod clock;
mod shared_clock;
mod verification;
pub mod wire;
pub use clock::{Epoch, PpsLabel};
pub use verification::{Action, Verification};

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(target_arch = "riscv32", derive(serde::Serialize))]
pub struct TimingStatus {
    pub pps_config_verified: bool,
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

pub fn gps_epoch_ms(week: u32, tow_ms: u32) -> Option<u64> {
    (tow_ms < 604_800_000).then(|| u64::from(week) * 604_800_000 + u64::from(tow_ms))
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    static CLOCK: shared_clock::SharedClock = shared_clock::SharedClock::new();
    static UNSYNCHRONIZED: AtomicU32 = AtomicU32::new(0);
    static IMU_REJECTED: AtomicU32 = AtomicU32::new(0);
    static CONFIG_VERIFIED: AtomicBool = AtomicBool::new(false);
    pub fn is_configuration_verified() -> bool {
        CONFIG_VERIFIED.load(Ordering::Acquire)
    }
    pub fn configuration_verified(verified: bool) {
        if CONFIG_VERIFIED.swap(verified, Ordering::AcqRel) && !verified {
            invalidate();
        }
    }
    pub fn invalidate() {
        let _ = CLOCK.update(clock::ClockSync::invalidate);
    }
    pub fn observe_pulse() {
        if let Some(at) = crate::capture::pps_ns() {
            let _ = CLOCK.update(|clock| clock.pulse(at));
        }
    }
    pub fn observe_label(label: PpsLabel, received_ns: u64) {
        observe_pulse();
        let _ = CLOCK.update(|clock| clock.label(label, received_ns));
    }
    pub fn map_epoch(week: u32, tow_ms: u32, now_ns: u64) -> Option<Epoch> {
        let gps_ns = gps_epoch_ms(week, tow_ms)?.checked_mul(1_000_000)?;
        map_gps_ns(gps_ns, now_ns)
    }
    pub fn map_gps_ns(gps_ns: u64, now_ns: u64) -> Option<Epoch> {
        let value = CLOCK
            .update(|clock| clock.epoch(gps_ns, now_ns))
            .ok()
            .flatten();
        if value.is_none() {
            UNSYNCHRONIZED.fetch_add(1, Ordering::Relaxed);
        }
        value
    }
    pub fn imu_rejected() {
        IMU_REJECTED.fetch_add(1, Ordering::Relaxed);
    }
    pub fn status() -> TimingStatus {
        let now = crate::capture::now_ns();
        let mut status = CLOCK
            .update(|clock| {
                if now.is_none() {
                    clock.invalidate();
                }
                clock.status(now.unwrap_or(0))
            })
            .unwrap_or_else(|s| s);
        if now.is_none() {
            status.pps_age_ms = None;
        }
        status.capture_errors = crate::capture::ERRORS.load(Ordering::Relaxed);
        status.pps_config_verified = CONFIG_VERIFIED.load(Ordering::Relaxed);
        status.imu_timing_rejected = IMU_REJECTED.load(Ordering::Relaxed);
        status.gnss_unsynchronized = UNSYNCHRONIZED.load(Ordering::Relaxed);
        status
    }
    #[embassy_executor::task]
    pub async fn maintenance() {
        loop {
            crate::capture::maintain();
            observe_pulse();
            embassy_time::Timer::after_millis(100).await;
        }
    }
}
#[cfg(target_arch = "riscv32")]
pub use hardware::*;

#[cfg(test)]
mod tests;
