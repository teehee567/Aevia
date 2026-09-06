//! Private embedded-live implementation.
//!
//! The public engine module adapts semantic observations/configuration into
//! these fixed-size types. Nothing in this module allocates or performs device
//! I/O, and no solver/matrix type crosses the public engine seam.

pub(crate) mod core;
pub(crate) mod dense_history;
pub(crate) mod eskf;
pub(crate) mod initializer;
pub(crate) mod predictor;
pub(crate) mod preintegration;
pub(crate) mod reanchor;
mod rts_window;
pub(crate) mod scheduler;
pub(crate) mod smoothing;
pub(crate) mod state;

pub(crate) use core::{
    DrainReport, GnssQualityUpdate, LiveCore, LiveCoreConfig, LiveCoreError, LiveCoreHistory,
    LiveCoreInput, LiveCoreSeed, LiveCoreState,
};
pub(crate) use dense_history::{DenseCovariance, DenseEndpoint};
pub(crate) use eskf::{
    ConsiderCovariance, CovariancePolicy, GnssObservation, GnssUpdateOutcome, MAX_CONSIDER,
    NavConsiderCovariance, NisGate, ProcessNoise, SharedMeasurementJacobians, UpdateDecision,
    independent_clock_consider_covariance_into, transition_consider_covariance_into,
    zero_consider_covariance,
};
pub(crate) use initializer::{
    AlignmentConfig, GnssInitializationFix, InitialHeadingSource, InitializationPhase, Initializer,
    StationaryConfig,
};
pub(crate) use predictor::PredictorConfig;
pub(crate) use preintegration::{CompactCovariance3, GapModel, ImuInterval, ImuNoise};
pub(crate) use reanchor::{EcefAnchor, ReanchorMonitor, ReanchorPolicy};
pub(crate) use scheduler::{EnqueueDisposition, OrderKey, Scheduled, WorkQuota};
pub(crate) use state::{MechanizationContext, NavState};

/// Compile-time ceilings for the initial V2 Mini live implementation. Runtime
/// preflight may select smaller logical capacities but can never grow them.
pub(crate) const IMU_HISTORY_CAPACITY: usize = 1_024;
pub(crate) const MEASUREMENT_QUEUE_CAPACITY: usize = 128;
pub(crate) const DENSE_HISTORY_CAPACITY: usize = 208;
pub(crate) const MAX_IMU_RATE_HZ: u32 = 1_750;
pub(crate) const MAX_POSITION_RATE_HZ: u32 = 50;
pub(crate) const MAX_VELOCITY_RATE_HZ: u32 = 50;
pub(crate) const MAX_NAVIGATION_RATE_HZ: u32 = 400;
pub(crate) const MAX_HISTORY_HORIZON_NS: u64 = 500_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct CapacityRequest {
    pub(crate) imu_rate_hz: u32,
    pub(crate) position_rate_hz: u32,
    pub(crate) velocity_rate_hz: u32,
    pub(crate) navigation_rate_hz: u32,
    pub(crate) fusion_and_guard_ns: u64,
    pub(crate) transition_reserve: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct RequiredCapacities {
    pub(crate) imu: usize,
    pub(crate) measurements: usize,
    pub(crate) dense_segments: usize,
}

impl CapacityRequest {
    pub(crate) fn preflight(self) -> Result<RequiredCapacities, CapacityError> {
        if self.imu_rate_hz == 0
            || self.navigation_rate_hz == 0
            || self.imu_rate_hz > MAX_IMU_RATE_HZ
            || self.position_rate_hz > MAX_POSITION_RATE_HZ
            || self.velocity_rate_hz > MAX_VELOCITY_RATE_HZ
            || self.navigation_rate_hz > MAX_NAVIGATION_RATE_HZ
            || self.fusion_and_guard_ns > MAX_HISTORY_HORIZON_NS
        {
            return Err(CapacityError::OutsideProfile);
        }
        let imu = required_for_rate(self.imu_rate_hz, self.fusion_and_guard_ns, 2)?;
        let measurement_rate = self
            .position_rate_hz
            .checked_add(self.velocity_rate_hz)
            .ok_or(CapacityError::ArithmeticOverflow)?;
        let measurements = required_for_rate(
            measurement_rate,
            self.fusion_and_guard_ns,
            self.transition_reserve,
        )?;
        let dense_segments =
            required_for_rate(self.navigation_rate_hz, self.fusion_and_guard_ns, 8)?;
        if imu > IMU_HISTORY_CAPACITY
            || measurements > MEASUREMENT_QUEUE_CAPACITY
            || dense_segments > DENSE_HISTORY_CAPACITY
        {
            return Err(CapacityError::InsufficientCompiledCapacity);
        }
        Ok(RequiredCapacities {
            imu,
            measurements,
            dense_segments,
        })
    }
}

fn required_for_rate(
    rate_hz: u32,
    horizon_ns: u64,
    reserve: usize,
) -> Result<usize, CapacityError> {
    let product = u64::from(rate_hz)
        .checked_mul(horizon_ns)
        .ok_or(CapacityError::ArithmeticOverflow)?;
    let epochs = product
        .checked_add(999_999_999)
        .ok_or(CapacityError::ArithmeticOverflow)?
        / 1_000_000_000;
    usize::try_from(epochs)
        .ok()
        .and_then(|value| value.checked_add(reserve))
        .ok_or(CapacityError::ArithmeticOverflow)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CapacityError {
    OutsideProfile,
    ArithmeticOverflow,
    InsufficientCompiledCapacity,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maximum_profile_fits_compiled_capacities() {
        let required = CapacityRequest {
            imu_rate_hz: 1_750,
            position_rate_hz: 50,
            velocity_rate_hz: 50,
            navigation_rate_hz: 400,
            fusion_and_guard_ns: 500_000_000,
            transition_reserve: 16,
        }
        .preflight()
        .unwrap();
        assert_eq!(required.imu, 877);
        assert_eq!(required.measurements, 66);
        assert_eq!(required.dense_segments, 208);
    }

    #[test]
    fn rate_or_horizon_cannot_expand_at_runtime() {
        let too_fast = CapacityRequest {
            imu_rate_hz: 1_751,
            position_rate_hz: 50,
            velocity_rate_hz: 50,
            navigation_rate_hz: 400,
            fusion_and_guard_ns: 500_000_000,
            transition_reserve: 16,
        };
        assert_eq!(too_fast.preflight(), Err(CapacityError::OutsideProfile));
    }
}
