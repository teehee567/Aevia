//! Bound initializer work to the lifetime of an accepted paired GNSS seed.

use aevia_trajectory::observation::InputDisposition;

#[derive(Default)]
pub(super) struct InitializationGate {
    oldest_seed_support_ns: Option<u64>,
    expiry_observed: bool,
}

impl InitializationGate {
    pub fn accepted_seed(&mut self, position_ns: u64, velocity_ns: u64) {
        let oldest = position_ns.min(velocity_ns);
        if self.oldest_seed_support_ns != Some(oldest) {
            self.oldest_seed_support_ns = Some(oldest);
            self.expiry_observed = false;
        }
    }

    pub fn permits_imu(&self, initializing: bool, support_ns: u64, maximum_age_ns: u64) -> bool {
        if !initializing {
            // An established navigation state must retain inertial propagation
            // during a GNSS outage, including its degraded/dead-reckoning phase.
            return true;
        }
        self.oldest_seed_support_ns.is_some_and(|seed| {
            // A fresh GNSS update can lead the oldest queued IMU interval.
            // Let the engine decide when that fix becomes usable; only stale
            // evidence closes this gate. Feed the first expired real interval
            // so the initializer itself evicts stale fixes and static means.
            !self.expiry_observed || support_ns.saturating_sub(seed) <= maximum_age_ns
        })
    }

    pub fn observe_imu_result(
        &mut self,
        initializing: bool,
        support_ns: u64,
        maximum_age_ns: u64,
        disposition: Option<InputDisposition>,
    ) {
        if initializing
            && matches!(
                disposition,
                Some(InputDisposition::InitializationOnly | InputDisposition::Fused)
            )
            && self
                .oldest_seed_support_ns
                .is_some_and(|seed| support_ns.saturating_sub(seed) > maximum_age_ns)
        {
            self.expiry_observed = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const AGE: u64 = 2_000_000_000;

    #[test]
    fn no_seed_drains_raw_only_but_active_navigation_is_never_gated() {
        let gate = InitializationGate::default();
        assert!(!gate.permits_imu(true, 10 * AGE, AGE));
        assert!(gate.permits_imu(false, 10 * AGE, AGE));
    }

    #[test]
    fn expires_at_oldest_field_support_after_one_real_expiry_observation() {
        let mut gate = InitializationGate::default();
        gate.accepted_seed(10 * AGE, 9 * AGE);
        assert!(gate.permits_imu(true, 10 * AGE, AGE));
        gate.observe_imu_result(
            true,
            10 * AGE,
            AGE,
            Some(InputDisposition::InitializationOnly),
        );
        assert!(gate.permits_imu(true, 10 * AGE + 1, AGE));
        gate.observe_imu_result(
            true,
            10 * AGE + 1,
            AGE,
            Some(InputDisposition::InitializationOnly),
        );
        assert!(!gate.permits_imu(true, 10 * AGE + 2, AGE));
        assert!(gate.permits_imu(false, 100 * AGE, AGE));
        // A duplicate support cannot repeatedly re-open an expired seed.
        gate.accepted_seed(10 * AGE, 9 * AGE);
        assert!(!gate.permits_imu(true, 10 * AGE + 2, AGE));
    }

    #[test]
    fn queued_imu_before_a_new_seed_is_not_misclassified_as_stale() {
        let mut gate = InitializationGate::default();
        gate.accepted_seed(10 * AGE, 10 * AGE);
        gate.observe_imu_result(
            true,
            12 * AGE,
            AGE,
            Some(InputDisposition::InitializationOnly),
        );
        assert!(!gate.permits_imu(true, 12 * AGE, AGE));
        gate.accepted_seed(14 * AGE, 13 * AGE);
        assert!(gate.permits_imu(true, 12 * AGE, AGE));
        gate.observe_imu_result(
            true,
            12 * AGE,
            AGE,
            Some(InputDisposition::InitializationOnly),
        );
        assert!(gate.permits_imu(true, 14 * AGE, AGE));
    }

    #[test]
    fn rejected_expiry_interval_does_not_claim_engine_observed_expiry() {
        let mut gate = InitializationGate::default();
        gate.accepted_seed(1, 1);
        assert!(gate.permits_imu(true, 2 * AGE, AGE));
        for disposition in [
            None,
            Some(InputDisposition::RetainedForOffline),
            Some(InputDisposition::StatisticallyRejected),
            Some(InputDisposition::TooLateForLive),
            Some(InputDisposition::QueuedForFusion),
            Some(InputDisposition::Downweighted),
        ] {
            gate.observe_imu_result(true, 2 * AGE, AGE, disposition);
            assert!(gate.permits_imu(true, 2 * AGE + 1, AGE), "{disposition:?}");
        }
        gate.observe_imu_result(
            true,
            2 * AGE + 1,
            AGE,
            Some(InputDisposition::InitializationOnly),
        );
        assert!(!gate.permits_imu(true, 2 * AGE + 2, AGE));
    }
}
