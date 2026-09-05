//! Orthogonal estimate-quality and observability dimensions.

use crate::time::{DurationNs, SessionTime};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EstimateStage {
    Predicted,
    Provisional,
    Finalized,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Validity {
    Nominal,
    Degraded,
    Invalid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GnssState {
    Fixed,
    Float,
    Standalone,
    Dgps,
    Ppp,
    Absent,
    Suspect,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimingQuality {
    PpsCorrelated,
    Modeled,
    ArrivalOnly,
    Discontinuous,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HeadingSource {
    Supplied,
    Gyrocompass,
    DynamicAlignment,
    NonHolonomicConstraint,
    None,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HeadingObservability {
    Supplied,
    Gyrocompassed,
    DynamicallyAligned,
    Constrained,
    Unobservable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Integrity {
    Monitored,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CovarianceConditioning {
    UnconditionalModel,
    ConditionalOnSelection,
    Unavailable,
}

/// Quality dimensions for one trajectory state or metric result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EstimateQuality {
    pub stage: EstimateStage,
    pub validity: Validity,
    pub gnss: GnssState,
    pub timing: TimingQuality,
    pub integrity: Integrity,
    pub covariance: CovarianceConditioning,
    pub imu_gap: bool,
    /// The local propagation support includes an upstream-degraded IMU input.
    /// Its supplied covariance already includes the preparation uncertainty.
    pub degraded_input: bool,
}

impl EstimateQuality {
    pub const INVALID: Self = Self {
        stage: EstimateStage::Provisional,
        validity: Validity::Invalid,
        gnss: GnssState::Absent,
        timing: TimingQuality::Discontinuous,
        integrity: Integrity::Unavailable,
        covariance: CovarianceConditioning::Unavailable,
        imu_gap: false,
        degraded_input: false,
    };
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObservabilityReport {
    pub heading_source: HeadingSource,
    pub heading: HeadingObservability,
    pub heading_variance_rad2: Option<f64>,
    pub course_available: bool,
    pub body_axis_quantities_available: bool,
    pub angular_acceleration_available: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UnavailableReason {
    Unobservable,
    InsufficientSignalToNoise,
    MissingUncertainty,
    /// A marginal exists, but the cross-covariance or augmented joint state
    /// required by this derived quantity was not retained. This is distinct
    /// from wholly missing uncertainty and must never be treated as
    /// independence.
    MissingCorrelation,
    OutsideQualifiedRange,
    UnsupportedAtProcessingLevel,
    FrameUnresolved,
    Gap,
    IllConditioned,
    Ambiguous,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FieldValue<T: Copy> {
    Available(T),
    Unavailable(UnavailableReason),
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DiagnosticCounts {
    pub imu_epochs_accepted: u64,
    pub imu_epochs_rejected: u64,
    pub gnss_updates_fused: u64,
    pub gnss_updates_rejected: u64,
    pub gnss_updates_downweighted: u64,
    pub observations_too_late: u64,
    pub clock_discontinuities: u32,
    pub reinitializations: u32,
    pub covariance_repairs: u32,
    pub metric_ambiguities: u32,
    pub output_overflows: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QualityInterval {
    pub start: SessionTime,
    pub end: SessionTime,
    pub validity: Validity,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContributionReport {
    pub gnss_outage_age: Option<DurationNs>,
    pub last_gnss_epoch: Option<SessionTime>,
    pub imu_only: bool,
}
