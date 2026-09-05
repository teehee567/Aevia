//! Offline replay and fixed-interval trajectory refinement for computers and phones.
//!
//! The public surface is intentionally semantic: borrowed evidence, a
//! transactional result sink, and the completed engine-owned trajectory.  All
//! matrices, solver state, and seekable state-store handles remain private.
//!
//! Offline smoothing currently requires each nonzero interval-average IMU
//! sample covariance to span at most one stored propagation edge. A GNSS or
//! clock cut inside such a support returns `ProcessError::CapabilityUnavailable`;
//! captured live replay retains the sample latent and supports those cuts.
//! Missing IMU support also requires an explicit supported reinitialization.
//! Dense means remain available when the full inertial covariance cannot be
//! represented by the offline conditional bridge; interior uncertainty is
//! then explicitly unavailable instead of using an independent-noise fit.

mod ports;
mod solver;
mod store;

pub use ports::{
    CAPTURED_REINITIALIZATION_SCHEMA_V2, CAPTURED_REPLAY_CONTRACT_V2, CapturedLiveFinishCall,
    CapturedLiveStepCall, CapturedReinitializationInputV2, CapturedReplayContract,
    ClockModelEvidence, ControlChangeEvidence, EvidenceEnd, EvidenceEvent, EvidenceGap,
    EvidenceGapReason, EvidenceManifest, EvidenceSource, MAX_OFFLINE_CLOCK_MODELS,
    ReinitializationEvidence, ReinitializationReason, ReplaySeedEvidence, ResultDescriptor,
    ResultEnd, ResultRecord, ResultRecordBounds, ResultSink, ResultSinkAttestation,
    ResultSinkPreflight, SliceEvidenceSource, SmoothedStateRecord,
};
pub use solver::{OfflineRun, OfflineRunSummary, PosteriorSamplingAvailability};

pub(crate) use ports::{CapturedTranscriptDigestV1, SinkTransaction};
pub(crate) use solver::{drive_captured_replay, run_offline};
pub(crate) use store::{FIXED_RECORD_HEADER_BYTES, FixedRecordStore, FixedRecordStoreKind};
