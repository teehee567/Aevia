//! Public construction and execution seam for live and host processing.
//!
//! Semantic configuration and observations cross this seam; estimator and
//! platform implementation details remain in private responsibility modules.

use crate::config::{LiveSpec, ProcessingSpec};
use crate::frame::{EcefPosition, EcefVelocity, OrientationEcefFromBody};
use crate::ids::ObservationId;
use crate::metric::MetricMutation;
use crate::observation::InputDisposition;
use crate::quality::{DiagnosticCounts, EstimateQuality, ObservabilityReport};
use crate::time::{SessionTime, TimeSpan};

mod bindings;
mod digest;
mod live;
mod process;

pub use digest::{captured_summary_digest_v1, captured_update_digest_v1};
pub use live::{LiveBuilder, LivePlan, LiveSession};
pub use process::{PreparedProcess, ProcessBuilder};

/// The only embedded hardware/profile family implemented by this crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LivePlatform {
    Esp32S31Wroom3N16R16V,
}

/// Public high-level phase; private initializer/filter substates stay hidden.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LivePhase {
    Initializing,
    Navigating,
    Degraded,
    Finishing,
    Finished,
}

/// Low-copy predicted-present projection intended for a live display.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveProjection {
    pub time: SessionTime,
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    pub orientation_ecef_from_body: OrientationEcefFromBody,
    pub quality: EstimateQuality,
    pub observability: ObservabilityReport,
}

/// Statistical outcome produced when an earlier queued GNSS member crosses
/// the delayed frontier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FusionOutcome {
    pub observation: ObservationId,
    pub disposition: InputDisposition,
    pub normalized_innovation_squared: Option<f32>,
}

/// Borrowed per-call output. Its mutation slice is valid only until the next
/// mutable operation on the session.
#[derive(Debug)]
pub struct LiveUpdate<'a> {
    pub input: Option<(ObservationId, InputDisposition)>,
    pub fusion: Option<FusionOutcome>,
    pub corrected_interval: Option<TimeSpan>,
    /// New fixed-ENU anchor generation committed during this call.
    pub reanchor_generation: Option<u32>,
    pub navigation_watermark: Option<SessionTime>,
    pub metric_watermark: Option<SessionTime>,
    pub present: Option<LiveProjection>,
    pub mutations: &'a [MetricMutation],
    pub diagnostics: DiagnosticCounts,
    pub phase: LivePhase,
    /// Unspent corrected-frontier credits from this call. Fixed-capacity
    /// ingestion, transfer/reanchor, metric, and projection work is not
    /// represented by this value.
    pub work_remaining: u32,
}

/// Final bounded live-session summary suitable for caller-owned persistent
/// storage. It never implies that the complete session trajectory is resident.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveSummary {
    pub terminal_time: Option<SessionTime>,
    pub retained_trajectory_span: Option<TimeSpan>,
    pub diagnostics: DiagnosticCounts,
    pub finalized_metric_results: u16,
}

/// Result of one bounded finish/drain call.
#[derive(Debug)]
pub struct LiveFinishUpdate<'a> {
    pub complete: bool,
    pub update: LiveUpdate<'a>,
}

impl Default for LiveSummary {
    fn default() -> Self {
        Self {
            terminal_time: None,
            retained_trajectory_span: None,
            diagnostics: DiagnosticCounts::default(),
            finalized_metric_results: 0,
        }
    }
}

/// Namespace for the two deliberately different engine workflows.
#[derive(Clone, Copy, Debug, Default)]
pub struct TrajectoryEngine;

impl TrajectoryEngine {
    #[must_use]
    pub fn live<'a>(spec: LiveSpec<'a>) -> LiveBuilder<'a> {
        LiveBuilder::new(spec)
    }

    #[must_use]
    pub fn process<'a>(spec: ProcessingSpec<'a>) -> ProcessBuilder<'a> {
        ProcessBuilder::new(spec)
    }
}
