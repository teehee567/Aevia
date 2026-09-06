//! Live and offline processing resource limits.

use crate::error::ValidationError;
use crate::time::DurationNs;

/// Explicit live-resource ceilings for the complete V2 Mini firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LiveResourceLimits {
    /// Internal SRAM for engine state, stack, scratch, metrics, and hot history.
    pub internal_sram_bytes: usize,
    /// PSRAM for bounded cold trajectory history/scratch.
    pub psram_bytes: usize,
    /// Live-task stack high-water ceiling.
    pub stack_bytes: usize,
    /// Firmware flash attributable to live trajectory and metric code.
    pub flash_bytes: usize,
    /// Total configured gates plus targets.
    pub maximum_gates_and_targets: u16,
    /// Active root candidates retained per segment.
    pub maximum_active_candidates_per_segment: u8,
    /// Metric mutations returned by one step.
    pub maximum_metric_mutations_per_step: u8,
    /// Separately budgeted recorder queue.
    pub recorder_queue_bytes: usize,
    /// SD-stall interval covered by that queue.
    pub recorder_stall_coverage: DurationNs,
}

impl LiveResourceLimits {
    /// Plan-specified initial V2 Mini ceilings.
    pub const V2_MINI_INITIAL: Self = Self {
        internal_sram_bytes: 192 * 1_024,
        psram_bytes: 1_024 * 1_024,
        stack_bytes: 32 * 1_024,
        flash_bytes: 1_572_864,
        maximum_gates_and_targets: 64,
        maximum_active_candidates_per_segment: 4,
        maximum_metric_mutations_per_step: 16,
        recorder_queue_bytes: 1_024 * 1_024,
        recorder_stall_coverage: DurationNs::from_ns(2_000_000_000),
    };

    /// Development ceiling for the bounded live extended RTS workspace.
    /// This explicitly expands cold PSRAM from the original 1 MiB contract;
    /// timing, linker placement, and stack use still require qualification on
    /// the complete fitted firmware before release.
    pub const V2_MINI_RTS: Self = Self {
        psram_bytes: 3 * 1_024 * 1_024,
        ..Self::V2_MINI_INITIAL
    };

    /// Validates non-zero bounds and containment within the current V2 Mini
    /// RTS development ceiling. Validation does not attest hardware timing.
    pub const fn validate_v2_mini(self) -> Result<Self, ValidationError> {
        if self.internal_sram_bytes == 0
            || self.internal_sram_bytes > Self::V2_MINI_INITIAL.internal_sram_bytes
            || self.psram_bytes > Self::V2_MINI_RTS.psram_bytes
            || self.stack_bytes == 0
            || self.stack_bytes > Self::V2_MINI_INITIAL.stack_bytes
            || self.stack_bytes > self.internal_sram_bytes
            || self.flash_bytes == 0
            || self.flash_bytes > Self::V2_MINI_INITIAL.flash_bytes
            || self.maximum_gates_and_targets == 0
            || self.maximum_gates_and_targets > Self::V2_MINI_INITIAL.maximum_gates_and_targets
            || self.maximum_active_candidates_per_segment == 0
            || self.maximum_active_candidates_per_segment
                > Self::V2_MINI_INITIAL.maximum_active_candidates_per_segment
            || self.maximum_metric_mutations_per_step == 0
            || self.maximum_metric_mutations_per_step
                > Self::V2_MINI_INITIAL.maximum_metric_mutations_per_step
            || self.recorder_queue_bytes > Self::V2_MINI_INITIAL.recorder_queue_bytes
            || self.recorder_stall_coverage.as_ns()
                > Self::V2_MINI_INITIAL.recorder_stall_coverage.as_ns()
        {
            Err(ValidationError::CapacityExceeded)
        } else {
            Ok(self)
        }
    }
}

/// Hard host resource limits applied before offline/advanced processing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OfflineResourceLimits {
    /// Peak resident engine memory.
    pub peak_memory_bytes: u64,
    /// Seekable temporary storage.
    pub temporary_storage_bytes: u64,
    /// Maximum result-sidecar bytes.
    pub output_bytes: u64,
    /// Maximum worker threads/tasks.
    pub worker_count: u16,
    /// Optional deterministic elapsed-work-unit ceiling.
    pub elapsed_work_limit: Option<u64>,
}

impl OfflineResourceLimits {
    /// Validates non-zero limits.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.peak_memory_bytes == 0
            || self.output_bytes == 0
            || self.worker_count == 0
            || matches!(self.elapsed_work_limit, Some(0))
        {
            Err(ValidationError::CapacityExceeded)
        } else {
            Ok(self)
        }
    }
}
