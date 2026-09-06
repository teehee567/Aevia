//! Offline `f64` solution-level forward filter and fixed-interval smoother.
//!
//! Numerical state, covariance blocks, and storage choices remain private.
//! Callers provide semantic evidence and receive only an engine-owned
//! [`Trajectory`] plus semantic result records.

use crate::{quality::DiagnosticCounts, trajectory::Trajectory};

mod catalog;
mod distance_uncertainty;
mod estimation;
mod evidence;
mod filter;
mod forward;
mod inertial;
mod initialization;
mod math;
mod measurement;
mod metric_uncertainty;
mod propagation;
mod publication;
mod run;
mod smoothing;

#[cfg(test)]
mod tests;

pub(crate) use estimation::PsdSolver;
pub(crate) use evidence::drive_captured_replay;
pub(crate) use run::run_offline;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OfflineRunSummary {
    pub state_count: u64,
    pub objective: f64,
    pub attempted_ieks_passes: u8,
    pub accepted_ieks_passes: u8,
    pub diagnostics: DiagnosticCounts,
    pub used_seekable_store: bool,
    pub state_store_record_bytes: u64,
}

/// Completed offline candidate.  The engine may publish or compare this
/// candidate without learning which storage/numerical implementation produced
/// it.
pub struct OfflineRun {
    pub trajectory: Trajectory,
    pub summary: OfflineRunSummary,
}

/// Whether this candidate retains a joint model from which correlated
/// posterior paths can be sampled.
///
/// The completed candidate publishes trajectory marginals after its private
/// state store and backward-conditionals have been destroyed.  It deliberately
/// does not pretend those marginals define a joint motion distribution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PosteriorSamplingAvailability {
    /// Sampling is unavailable because the complete joint motion/state model
    /// was not retained by this processing backend.
    JointModelNotRetained,
}

impl OfflineRun {
    /// Reports posterior-path sampling availability without silently treating
    /// per-epoch marginals as independent samples.
    #[must_use]
    pub const fn posterior_sampling_availability(&self) -> PosteriorSamplingAvailability {
        PosteriorSamplingAvailability::JointModelNotRetained
    }
}
