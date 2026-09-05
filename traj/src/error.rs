//! Owned, allocation-free error types at the engine seam.

use core::fmt;

use crate::{ids::SourceId, time::SessionTime};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValidationError {
    NonFinite,
    InvalidCovariance,
    InvalidRotation,
    InvalidTimeSpan,
    TimeOverflow,
    TimeOutOfRange,
    InvalidFrame,
    InvalidReferencePoint,
    InvalidMetricDefinition,
    CapacityExceeded,
    IncompatibleDefinition,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PrepareError {
    InvalidDefinition(ValidationError),
    IncompatibleProfile,
    FrameUnresolved,
    CalibrationUnavailable,
    CapabilityUnavailable,
    PlatformUnsupported,
    EvidenceUnavailable,
    /// The artifact may claim captured observations, but the host-facing
    /// semantic port cannot yet reconstruct the exact sequence of live calls,
    /// controls, seeds, and bounded work decisions required for behavioral
    /// equivalence.  This is deliberately distinct from a missing compiled
    /// backend and from missing sensor evidence.
    ReplayContractUnavailable,
    InsufficientResources,
    InvalidWorkspaceAlignment,
    UnqualifiedProfile,
    NotRestartable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepError {
    DuplicateObservation {
        source: SourceId,
        sequence: u64,
    },
    NonMonotonicSequence {
        source: SourceId,
        previous: u64,
        received: u64,
    },
    InvalidObservation(ValidationError),
    FrameMismatch,
    ClockDiscontinuity,
    WorkspaceContract,
    /// Bounded estimator operation failed its numerical/integrity contract.
    EstimatorFailure,
    /// Compiled live output storage could not accept another value.
    OutputCapacityExceeded,
    /// Bounded continuous-metric evaluation failed.
    MetricFailure,
    AlreadyFinishing,
    Finished,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcessError {
    InvalidEvidence,
    IncompleteEvidence,
    EvidenceLineageConflict,
    SourceFailure,
    SinkFailure,
    StorageExhausted,
    StorageCorrupt,
    Cancelled,
    NumericalNonConvergence,
    AdvancedCapabilityFailure,
    /// A complete captured call contract was present, but the live core's
    /// canonical output identity differed from the recorded expectation.
    ReplayMismatch,
    CapabilityUnavailable,
    ResourceLimit,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueryError {
    OutsideAvailableSpan {
        requested: SessionTime,
        earliest: Option<SessionTime>,
        latest: Option<SessionTime>,
    },
    FrameUnavailable,
    ReferencePointUnavailable,
    ObservabilityUnavailable,
    InvalidRequest,
    BackingStoreFailure,
    TrajectoryInvalid,
}

macro_rules! display_as_debug {
    ($($ty:ty),+ $(,)?) => {$ (
        impl fmt::Display for $ty {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(formatter, "{self:?}")
            }
        }
    )+ };
}

display_as_debug!(
    ValidationError,
    PrepareError,
    StepError,
    ProcessError,
    QueryError
);

#[cfg(feature = "offline")]
impl std::error::Error for ValidationError {}
#[cfg(feature = "offline")]
impl std::error::Error for PrepareError {}
#[cfg(feature = "offline")]
impl std::error::Error for StepError {}
#[cfg(feature = "offline")]
impl std::error::Error for ProcessError {}
#[cfg(feature = "offline")]
impl std::error::Error for QueryError {}
