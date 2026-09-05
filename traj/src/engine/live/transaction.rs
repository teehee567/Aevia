//! Small ingest and source-sequence journals for transactional rejection.

use super::{GnssQualityEvidence, InitializationFixEvidence, LiveSession, PendingClockTransition};
use crate::error::{StepError, ValidationError};
use crate::ids::{ClockModelId, ClockSegmentId, ObservationId};
use crate::live::EcefAnchor;
use crate::quality::{DiagnosticCounts, GnssState, HeadingSource, Integrity, TimingQuality};
use crate::time::SessionTime;
use crate::workspace::{MAX_LIVE_SOURCES, SourceSequence};

#[cfg(test)]
#[path = "transaction_tests.rs"]
mod tests;

#[derive(Clone, Copy)]
pub(super) enum SequenceUndo {
    Updated { index: usize, previous: u64 },
    Added,
}

pub(super) fn begin_source_sequence(
    sequences: &mut heapless::Vec<SourceSequence, MAX_LIVE_SOURCES>,
    id: ObservationId,
) -> Result<SequenceUndo, StepError> {
    if let Some((index, sequence)) = sequences
        .iter_mut()
        .enumerate()
        .find(|(_, entry)| entry.source == id.source)
    {
        if id.sequence == sequence.latest {
            return Err(StepError::DuplicateObservation {
                source: id.source,
                sequence: id.sequence,
            });
        }
        if id.sequence < sequence.latest {
            return Err(StepError::NonMonotonicSequence {
                source: id.source,
                previous: sequence.latest,
                received: id.sequence,
            });
        }
        let previous = sequence.latest;
        sequence.latest = id.sequence;
        return Ok(SequenceUndo::Updated { index, previous });
    }
    if sequences.is_full() {
        return Err(StepError::InvalidObservation(
            ValidationError::CapacityExceeded,
        ));
    }
    sequences
        .push(SourceSequence {
            source: id.source,
            latest: id.sequence,
        })
        .map_err(|_| StepError::WorkspaceContract)?;
    Ok(SequenceUndo::Added)
}

pub(super) fn rollback_source_sequence(
    sequences: &mut heapless::Vec<SourceSequence, MAX_LIVE_SOURCES>,
    undo: SequenceUndo,
) {
    match undo {
        SequenceUndo::Updated { index, previous } => {
            if let Some(sequence) = sequences.get_mut(index) {
                sequence.latest = previous;
            }
        }
        SequenceUndo::Added => {
            let _ = sequences.pop();
        }
    }
}

/// Small journal for every session field that observation ingestion may
/// change before returning a contract error. Large core/history mutations are
/// excluded deliberately: their ingest entry points implement their own
/// bounded preflight-and-commit transactions.
#[derive(Clone, Copy)]
pub(super) struct LiveIngestCheckpoint {
    initializer: Option<crate::live::Initializer>,
    had_core: bool,
    anchor: Option<EcefAnchor>,
    latest_initialization_fix: Option<InitializationFixEvidence>,
    current_clock_model: Option<ClockModelId>,
    current_clock_segment: ClockSegmentId,
    last_clock_transition_time: Option<SessionTime>,
    pending_clock_transition: Option<PendingClockTransition>,
    clock_reference_time: SessionTime,
    clock_uncertainty_valid: bool,
    last_accepted_imu_end: Option<SessionTime>,
    heading_source: HeadingSource,
    heading_variance_rad2: Option<f64>,
    gnss_state: GnssState,
    last_gnss_evidence: Option<GnssQualityEvidence>,
    timing_quality: TimingQuality,
    integrity: Integrity,
    predictor_tracking_degraded: bool,
    predictor_gap: bool,
    predictor_degraded_input: bool,
    diagnostics: DiagnosticCounts,
}

impl LiveIngestCheckpoint {
    pub(super) fn capture(session: &LiveSession<'_, '_>) -> Self {
        Self {
            initializer: session.internal.initializer,
            had_core: session.internal.core.is_active(),
            anchor: session.anchor,
            latest_initialization_fix: session.latest_initialization_fix,
            current_clock_model: session.current_clock_model,
            current_clock_segment: session.current_clock_segment,
            last_clock_transition_time: session.last_clock_transition_time,
            pending_clock_transition: session.pending_clock_transition,
            clock_reference_time: session.clock_reference_time,
            clock_uncertainty_valid: session.clock_uncertainty_valid,
            last_accepted_imu_end: session.last_accepted_imu_end,
            heading_source: session.heading_source,
            heading_variance_rad2: session.heading_variance_rad2,
            gnss_state: session.gnss_state,
            last_gnss_evidence: session.last_gnss_evidence,
            timing_quality: session.timing_quality,
            integrity: session.integrity,
            predictor_tracking_degraded: session.predictor_tracking_degraded,
            predictor_gap: session.predictor_gap,
            predictor_degraded_input: session.predictor_degraded_input,
            diagnostics: session.diagnostics,
        }
    }

    pub(super) fn restore(self, session: &mut LiveSession<'_, '_>) {
        session.internal.initializer = self.initializer;
        if !self.had_core {
            // Initialization activates the caller-owned core in place before
            // its first atomic IMU ingest. Restore the inactive placeholder
            // and history if the containing observation ultimately rejects.
            session.internal.core.reset();
            session.psram.history.clear();
        }
        session.anchor = self.anchor;
        session.latest_initialization_fix = self.latest_initialization_fix;
        session.current_clock_model = self.current_clock_model;
        session.current_clock_segment = self.current_clock_segment;
        session.last_clock_transition_time = self.last_clock_transition_time;
        session.pending_clock_transition = self.pending_clock_transition;
        session.clock_reference_time = self.clock_reference_time;
        session.clock_uncertainty_valid = self.clock_uncertainty_valid;
        session.last_accepted_imu_end = self.last_accepted_imu_end;
        session.heading_source = self.heading_source;
        session.heading_variance_rad2 = self.heading_variance_rad2;
        session.gnss_state = self.gnss_state;
        session.last_gnss_evidence = self.last_gnss_evidence;
        session.timing_quality = self.timing_quality;
        session.integrity = self.integrity;
        session.predictor_tracking_degraded = self.predictor_tracking_degraded;
        session.predictor_gap = self.predictor_gap;
        session.predictor_degraded_input = self.predictor_degraded_input;
        session.diagnostics = self.diagnostics;
    }
}
