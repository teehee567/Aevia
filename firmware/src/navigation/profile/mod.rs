//! Explicitly unqualified V2 Mini bench profile. Factory scale factors are
//! usable for bring-up; installation, noise and timing priors still need capture.

use aevia_trajectory::time::DurationNs;

#[cfg(target_arch = "riscv32")]
pub(super) const IMU_RATE_ONE_SIGMA_RPS: f64 = 0.001;
#[cfg(target_arch = "riscv32")]
pub(super) const IMU_ACCELERATION_ONE_SIGMA_MPS2: f64 = 0.03;

pub(super) const MAX_GNSS_AGE_NS: u64 = 2_000_000_000;

pub(super) const FUSION_DELAY: DurationNs = DurationNs::from_ns(100_000_000);
pub(super) const SMOOTHING_LAG: DurationNs = DurationNs::from_ns(20_000_000);
pub(super) const SNAPSHOT_INTERVAL_US: u64 = 100_000;

#[cfg(any(target_arch = "riscv32", test))]
pub(super) use configuration::{FRAME, engine};

#[cfg(any(target_arch = "riscv32", test))]
mod configuration;
