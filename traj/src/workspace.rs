//! Explicit caller-owned placement for the allocator-free live engine.
//!
//! The two storage objects are intentionally distinct so firmware linker
//! sections can place hot filter/metric state in internal SRAM and rolling
//! evidence/dense trajectory state in PSRAM. `LiveSession` only borrows them.

use core::mem::{align_of, size_of};

use heapless::Vec as FixedVec;

use crate::{
    frame::TerrestrialFrame,
    ids::{ContentDigestV1, SourceId, TrajectoryRevision},
    live::{
        ConsiderCovariance, Initializer, LiveCoreHistory, LiveCoreState, zero_consider_covariance,
    },
    metric::{
        LiveMetricScratch, LiveMetricTracker, LiveMetricUpdate, MAX_METRIC_MUTATIONS_PER_STEP,
    },
    trajectory::Trajectory,
};

/// Maximum distinct normalized sources tracked by the live sequence guard.
pub const MAX_LIVE_SOURCES: usize = 16;

/// Physical region asserted by the firmware's linker/runtime integration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryRegion {
    InternalSram,
    Psram,
}

/// Required placement and exact compiled object sizes for one live profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WorkspaceRequirements {
    internal_sram_bytes: usize,
    internal_sram_alignment: usize,
    psram_bytes: usize,
    psram_alignment: usize,
    maximum_stack_bytes: usize,
    profile_digest: ContentDigestV1,
}

impl WorkspaceRequirements {
    #[must_use]
    pub const fn internal_sram_bytes(self) -> usize {
        self.internal_sram_bytes
    }

    #[must_use]
    pub const fn internal_sram_alignment(self) -> usize {
        self.internal_sram_alignment
    }

    #[must_use]
    pub const fn psram_bytes(self) -> usize {
        self.psram_bytes
    }

    #[must_use]
    pub const fn psram_alignment(self) -> usize {
        self.psram_alignment
    }

    #[must_use]
    pub const fn maximum_stack_bytes(self) -> usize {
        self.maximum_stack_bytes
    }

    #[must_use]
    pub const fn profile_digest(self) -> ContentDigestV1 {
        self.profile_digest
    }

    pub(crate) const fn compiled(
        maximum_stack_bytes: usize,
        profile_digest: ContentDigestV1,
    ) -> Self {
        Self {
            internal_sram_bytes: size_of::<LiveInternalWorkspace>(),
            internal_sram_alignment: align_of::<LiveInternalWorkspace>(),
            psram_bytes: size_of::<LivePsramWorkspace>(),
            psram_alignment: align_of::<LivePsramWorkspace>(),
            maximum_stack_bytes,
            profile_digest,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct SourceSequence {
    pub(crate) source: SourceId,
    pub(crate) latest: u64,
}

/// Hot persistent storage. The alignment is explicit even if the platform's
/// natural alignment is currently smaller, preventing an ABI change from
/// silently violating a linker-section contract.
#[repr(C, align(16))]
pub struct LiveInternalWorkspace {
    pub(crate) core: LiveCoreState,
    pub(crate) initializer: Option<Initializer>,
    /// Current segment's complete fixed consider prior. Keeping this beside
    /// the initializer preserves clock/calibration correlations across a
    /// reinitialization without putting a 32x32 matrix on the session handle.
    pub(crate) consider_seed_covariance: ConsiderCovariance,
    pub(crate) last_metric_update: Option<LiveMetricUpdate>,
    pub(crate) sequences: FixedVec<SourceSequence, MAX_LIVE_SOURCES>,
    // Reserved for bounded mutation staging at the public transaction seam.
    pub(crate) mutation_scratch: [u8; MAX_METRIC_MUTATIONS_PER_STEP],
}

impl LiveInternalWorkspace {
    /// Creates empty storage. Firmware should construct this directly in its
    /// internal-SRAM static/linker section rather than on the live task stack.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            core: LiveCoreState::placeholder(),
            initializer: None,
            consider_seed_covariance: zero_consider_covariance(),
            last_metric_update: None,
            sequences: FixedVec::new(),
            mutation_scratch: [0; MAX_METRIC_MUTATIONS_PER_STEP],
        }
    }

    pub(crate) fn clear(&mut self) {
        self.core.reset();
        self.initializer = None;
        self.consider_seed_covariance.fill(0.0);
        self.last_metric_update = None;
        self.sequences.clear();
        self.mutation_scratch.fill(0);
    }
}

impl Default for LiveInternalWorkspace {
    fn default() -> Self {
        Self::new()
    }
}

// These are compile-time release guards, not substitutes for the final
// linker-map and stack high-water measurements on the fitted module.  Keeping
// them here prevents a field addition from silently exceeding the initial
// ESP32-S31 internal-memory allocation between test runs.
const _: () = assert!(
    size_of::<LiveInternalWorkspace>() + 32 * 1_024 <= 192 * 1_024,
    "live internal workspace plus stack exceeds the ESP32-S31 budget"
);

/// Cold rolling storage intended for PSRAM. It contains no pointer into
/// itself, so callers may place it before binding a session.
#[repr(C, align(16))]
pub struct LivePsramWorkspace {
    pub(crate) history: LiveCoreHistory,
    pub(crate) trajectory: Trajectory,
    /// Cold transaction destination for a clock-segment consider-prior
    /// transform. A candidate is committed to the internal-SRAM seed only
    /// after every fallible covariance/filter check succeeds.
    pub(crate) consider_seed_transaction: ConsiderCovariance,
    /// The compiled definitions and tombstone/result ledger are cold,
    /// navigation-cadence state; keeping them in PSRAM prevents a 64-rule
    /// plan from consuming scarce internal SRAM.
    pub(crate) metric_tracker: LiveMetricTracker,
    /// Speculative metric state and candidate/output journals. This remains
    /// distinct from the persistent tracker so a failed bounded evaluation
    /// cannot partially commit a live result ledger.
    pub(crate) metric_scratch: LiveMetricScratch,
}

impl LivePsramWorkspace {
    /// Constructs the caller-owned rolling store. This is setup-time work;
    /// session start later clears it in place and returns only a small handle.
    #[must_use]
    pub const fn new(frame: TerrestrialFrame) -> Self {
        Self {
            history: LiveCoreHistory::new(),
            trajectory: Trajectory::new(frame, TrajectoryRevision::new(0)),
            consider_seed_transaction: zero_consider_covariance(),
            metric_tracker: LiveMetricTracker::unconfigured(),
            metric_scratch: LiveMetricScratch::new(),
        }
    }

    pub(crate) fn clear(&mut self, frame: TerrestrialFrame, revision: TrajectoryRevision) {
        self.history.clear();
        self.trajectory.reset(frame, revision);
        self.consider_seed_transaction.fill(0.0);
        // The tracker and scratch are configured directly in place by
        // `LivePlan::start`; do not construct either large object here.
    }
}

// PSRAM is cache-attached and is touched by bounded history/metric work on the
// live path. Its capacity is a fixed profile contract, while exact cache-
// contention latency still requires measurement on the fitted module.
const LIVE_PSRAM_BUDGET_BYTES: usize = crate::config::LiveResourceLimits::V2_MINI_RTS.psram_bytes;
const LIVE_PSRAM_ROLLBACK_MARGIN_BYTES: usize = 32 * 1_024;
const _: () = assert!(
    size_of::<LivePsramWorkspace>() + LIVE_PSRAM_ROLLBACK_MARGIN_BYTES <= LIVE_PSRAM_BUDGET_BYTES,
    "live PSRAM workspace consumes the ESP32-S31 rollback margin"
);

/// Borrowed binding of the two physically distinct workspace regions.
pub struct LiveWorkspace<'a> {
    pub(crate) internal: &'a mut LiveInternalWorkspace,
    pub(crate) psram: &'a mut LivePsramWorkspace,
    pub(crate) internal_region: MemoryRegion,
    pub(crate) psram_region: MemoryRegion,
}

impl<'a> LiveWorkspace<'a> {
    /// Binds caller-owned objects and records the placement asserted by the
    /// firmware adapter. Preflight/start verifies both region identities and
    /// runtime addresses/alignment; the linker map remains the hardware proof.
    #[must_use]
    pub fn bind(
        internal: &'a mut LiveInternalWorkspace,
        internal_region: MemoryRegion,
        psram: &'a mut LivePsramWorkspace,
        psram_region: MemoryRegion,
    ) -> Self {
        Self {
            internal,
            psram,
            internal_region,
            psram_region,
        }
    }

    #[must_use]
    pub const fn asserted_regions(&self) -> (MemoryRegion, MemoryRegion) {
        (self.internal_region, self.psram_region)
    }

    pub(crate) fn validate(&self, required: WorkspaceRequirements) -> bool {
        self.internal_region == MemoryRegion::InternalSram
            && self.psram_region == MemoryRegion::Psram
            && size_of::<LiveInternalWorkspace>() >= required.internal_sram_bytes
            && size_of::<LivePsramWorkspace>() >= required.psram_bytes
            && (core::ptr::from_ref::<LiveInternalWorkspace>(self.internal) as usize)
                .is_multiple_of(required.internal_sram_alignment)
            && (core::ptr::from_ref::<LivePsramWorkspace>(self.psram) as usize)
                .is_multiple_of(required.psram_alignment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{
        CoordinateEpoch, ReferenceEllipsoid, TerrestrialRealization, Wgs84Realization,
    };
    use crate::ids::FrameId;

    fn with_large_stack(test: fn()) {
        std::thread::Builder::new()
            .name("trajectory-workspace-test".into())
            .stack_size(16 * 1_024 * 1_024)
            .spawn(test)
            .expect("large-stack test thread must start")
            .join()
            .expect("large-stack test thread must finish");
    }

    fn frame() -> TerrestrialFrame {
        TerrestrialFrame::new(
            FrameId::new(1),
            TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
            CoordinateEpoch::from_decimal_year(2024.0).unwrap(),
            ReferenceEllipsoid::WGS84,
        )
    }

    #[test]
    fn regions_are_distinct_aligned_and_exactly_reported() {
        with_large_stack(|| {
            let mut internal = LiveInternalWorkspace::new();
            let mut psram = LivePsramWorkspace::new(frame());
            let workspace = LiveWorkspace::bind(
                &mut internal,
                MemoryRegion::InternalSram,
                &mut psram,
                MemoryRegion::Psram,
            );
            let required =
                WorkspaceRequirements::compiled(32 * 1024, ContentDigestV1::from_bytes([1; 32]));
            assert!(workspace.validate(required));
            assert_eq!(
                required.internal_sram_bytes(),
                size_of::<LiveInternalWorkspace>()
            );
            assert_eq!(required.psram_bytes(), size_of::<LivePsramWorkspace>());
            assert!(
                required.internal_sram_bytes() + required.maximum_stack_bytes() <= 192 * 1_024,
                "internal={} stack={}",
                required.internal_sram_bytes(),
                required.maximum_stack_bytes()
            );
            assert!(
                required.psram_bytes() + LIVE_PSRAM_ROLLBACK_MARGIN_BYTES
                    <= LIVE_PSRAM_BUDGET_BYTES,
                "psram={} margin={}",
                required.psram_bytes(),
                LIVE_PSRAM_ROLLBACK_MARGIN_BYTES
            );
        });
    }

    #[test]
    fn swapped_region_assertion_fails_closed() {
        with_large_stack(|| {
            let mut internal = LiveInternalWorkspace::new();
            let mut psram = LivePsramWorkspace::new(frame());
            let workspace = LiveWorkspace::bind(
                &mut internal,
                MemoryRegion::Psram,
                &mut psram,
                MemoryRegion::InternalSram,
            );
            let required = WorkspaceRequirements::compiled(1, ContentDigestV1::from_bytes([0; 32]));
            assert!(!workspace.validate(required));
        });
    }

    #[test]
    fn clock_transaction_scratch_is_exactly_one_psram_covariance_and_clears() {
        with_large_stack(|| {
            assert_eq!(size_of::<ConsiderCovariance>(), 32 * 32 * size_of::<f32>());
            let mut psram = LivePsramWorkspace::new(frame());
            psram.consider_seed_transaction.fill(7.0);

            psram.clear(frame(), TrajectoryRevision::new(2));

            assert!(
                psram
                    .consider_seed_transaction
                    .iter()
                    .all(|value| *value == 0.0)
            );
        });
    }
}
