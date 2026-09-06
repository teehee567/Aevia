//! Fixed-size live observations and allocation-free raw-GNSS semantic views.

use crate::{
    error::ValidationError,
    frame::{EcefPosition, EcefVelocity, SensorAngularRate, SensorSpecificForce},
    ids::{
        ClockModelId, ClockSegmentId, ContentDigestV1, EphemerisIssue, FrameId, InputProfileId,
        ObservationId, ReferencePointId,
    },
    math::{FiniteF64, NonNegativeF64},
    time::{DurationNs, ObservationTime, SampleSupport, SessionTime},
    uncertainty::{
        Covariance3, CrossCovariance3, MAX_SHARED_PARAMETER_DIMENSION, MeasurementUncertainty,
        Variance, is_positive_semidefinite_2x2,
    },
};

/// Hard semantic safety bound for a borrowed raw GNSS epoch.
///
/// A recording profile normally declares a smaller measured maximum.  This
/// bound prevents hostile metadata from presenting an unbounded collection to
/// validators even on a host.
pub const MAX_RAW_SIGNALS_PER_EPOCH: usize = 1_024;

/// Validity and saturation state for X, Y, and Z axes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AxisStatus {
    valid: [bool; 3],
    saturated: [bool; 3],
}

impl AxisStatus {
    /// Constructs per-axis status in X/Y/Z order.
    #[must_use]
    pub const fn new(valid: [bool; 3], saturated: [bool; 3]) -> Self {
        Self { valid, saturated }
    }

    /// Status for a complete, unsaturated vector.
    pub const VALID: Self = Self::new([true; 3], [false; 3]);

    /// Returns the validity bit for an axis index in `0..3`.
    #[must_use]
    pub const fn is_valid(self, axis: usize) -> bool {
        axis < 3 && self.valid[axis]
    }

    /// Returns the saturation bit for an axis index in `0..3`.
    #[must_use]
    pub const fn is_saturated(self, axis: usize) -> bool {
        axis < 3 && self.saturated[axis]
    }

    /// Returns whether all axes are valid and unsaturated.
    #[must_use]
    pub const fn is_complete(self) -> bool {
        self.valid[0]
            && self.valid[1]
            && self.valid[2]
            && !self.saturated[0]
            && !self.saturated[1]
            && !self.saturated[2]
    }
}

/// One timed calibrated angular-rate vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedAngularRate {
    /// Calibrated body-relative-inertial rate in sensor axes.
    pub value: SensorAngularRate,
    /// Effective timing, support, and timing provenance.
    pub time: ObservationTime,
    /// Measurement covariance or explicit configured model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
    /// Per-axis sensor validity and saturation.
    pub axes: AxisStatus,
}

impl TimedAngularRate {
    /// Validates checked effective-time arithmetic and angular-rate interval-average
    /// support.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.time.effective_time()?;
        if !matches!(
            self.time.support,
            SampleSupport::IntervalAverage { duration } if duration.as_ns() > 0
        ) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// One timed calibrated specific-force vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedSpecificForce {
    /// Calibrated specific force in sensor axes.
    pub value: SensorSpecificForce,
    /// Effective timing, support, and timing provenance.
    pub time: ObservationTime,
    /// Measurement covariance or explicit configured model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
    /// Per-axis sensor validity and saturation.
    pub axes: AxisStatus,
}

impl TimedSpecificForce {
    /// Validates checked effective-time arithmetic and specific-force interval-average
    /// support.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.time.effective_time()?;
        if !matches!(
            self.time.support,
            SampleSupport::IntervalAverage { duration } if duration.as_ns() > 0
        ) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Upstream assessment of a prepared IMU interval.
///
/// Sensor decoding, calibration, channel selection and temporal alignment must
/// finish before constructing an observation. Degraded inputs remain usable;
/// their supplied covariance must already account for the preparation performed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuStatus {
    /// Complete calibrated measurement within its qualified operating range.
    Valid,
    /// Usable prepared measurement with reduced quality, reflected in covariance.
    Degraded,
    /// Missing or unusable measurement; normal engine gap policy applies.
    Unavailable,
    /// Source is initializing; integration must restart from qualified evidence.
    Initializing,
    /// Source continuity was lost; integration must restart from qualified evidence.
    Discontinuity,
}

/// One prepared IMU interval, independent of sensor model or acquisition format.
///
/// Angular rate and specific force are calibrated SI interval averages in the
/// declared measurement frame. Both must cover the same nonzero interval on the
/// same clock. The engine validates this contract but never resamples channels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuObservation {
    id: ObservationId,
    measurement_frame: FrameId,
    profile: InputProfileId,
    angular_rate: TimedAngularRate,
    specific_force: TimedSpecificForce,
    status: ImuStatus,
}

impl ImuObservation {
    /// Constructs an aligned, calibrated IMU measurement with explicit quality.
    pub fn new(
        id: ObservationId,
        measurement_frame: FrameId,
        profile: InputProfileId,
        angular_rate: TimedAngularRate,
        specific_force: TimedSpecificForce,
        status: ImuStatus,
    ) -> Result<Self, ValidationError> {
        angular_rate.validate()?;
        specific_force.validate()?;
        let end = angular_rate.time.effective_time()?;
        if specific_force.time.effective_time()? != end
            || angular_rate.time.clock_model != specific_force.time.clock_model
            || angular_rate.time.support != specific_force.time.support
        {
            return Err(ValidationError::InvalidTimeSpan);
        }
        let SampleSupport::IntervalAverage { duration } = angular_rate.time.support else {
            return Err(ValidationError::IncompatibleDefinition);
        };
        let duration_ns =
            i64::try_from(duration.as_ns()).map_err(|_| ValidationError::TimeOutOfRange)?;
        end.as_ns()
            .checked_sub(duration_ns)
            .ok_or(ValidationError::TimeOverflow)?;
        Ok(Self {
            id,
            measurement_frame,
            profile,
            angular_rate,
            specific_force,
            status,
        })
    }

    /// Returns the stable observation identity.
    #[must_use]
    pub const fn id(self) -> ObservationId {
        self.id
    }

    /// Returns the coordinate frame of both prepared vectors.
    #[must_use]
    pub const fn measurement_frame(self) -> FrameId {
        self.measurement_frame
    }

    /// Returns the immutable prepared-input contract identity.
    #[must_use]
    pub const fn profile(self) -> InputProfileId {
        self.profile
    }

    /// Returns the calibrated angular-rate interval average.
    #[must_use]
    pub const fn angular_rate(self) -> TimedAngularRate {
        self.angular_rate
    }

    /// Returns the calibrated specific-force interval average.
    #[must_use]
    pub const fn specific_force(self) -> TimedSpecificForce {
        self.specific_force
    }

    /// Returns the upstream measurement status.
    #[must_use]
    pub const fn status(self) -> ImuStatus {
        self.status
    }

    /// Returns whether a usable measurement carries upstream degradation.
    #[must_use]
    pub const fn is_degraded(self) -> bool {
        matches!(self.status, ImuStatus::Degraded)
    }

    /// Returns whether integration must restart instead of bridging a short gap.
    #[must_use]
    pub const fn breaks_continuity(self) -> bool {
        matches!(
            self.status,
            ImuStatus::Initializing | ImuStatus::Discontinuity
        )
    }

    /// Classifies complete-vector integration without preparing or repairing data.
    #[must_use]
    pub const fn integration_eligibility(self) -> ImuIntegrationEligibility {
        match self.status {
            ImuStatus::Initializing => return ImuIntegrationEligibility::RejectInitialization,
            ImuStatus::Discontinuity => return ImuIntegrationEligibility::RejectDiscontinuity,
            ImuStatus::Unavailable => return ImuIntegrationEligibility::RejectUnavailable,
            ImuStatus::Valid | ImuStatus::Degraded => {}
        }
        if !self.angular_rate.axes.is_complete() {
            return ImuIntegrationEligibility::RejectAngularRate;
        }
        if !self.specific_force.axes.is_complete() {
            return ImuIntegrationEligibility::RejectSpecificForce;
        }
        ImuIntegrationEligibility::Complete
    }
}

/// Complete-vector integration disposition for a prepared interval.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuIntegrationEligibility {
    /// Both prepared vectors are complete and usable.
    Complete,
    /// Initialization-status samples are never integrated.
    RejectInitialization,
    /// A source discontinuity prevents integration.
    RejectDiscontinuity,
    /// Upstream preparation marked the interval unavailable.
    RejectUnavailable,
    /// At least one angular-rate component is unusable.
    RejectAngularRate,
    /// At least one specific-force component is unusable.
    RejectSpecificForce,
}

/// Receiver health reported by the accepted firmware profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiverHealth {
    /// All required health diagnostics passed.
    Healthy,
    /// One or more diagnostics are suspect or stale.
    Suspect,
    /// Receiver reported a definite hardware/solution fault.
    Fault,
    /// Required health diagnostics were unavailable.
    Unknown,
}

/// Timed ECEF position field from a receiver solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssPosition {
    /// Antenna reference-point ECEF position in metres.
    pub value: EcefPosition,
    /// Position measurement epoch, independent of velocity time.
    /// The caller must account for receiver latency in the effective time.
    pub time: ObservationTime,
    /// Named terrestrial frame realization and epoch.
    pub frame: FrameId,
    /// Full/diagonal covariance or configured model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
    /// Whether the prepared measurement is valid for fusion.
    /// Invalid fields remain recordable but are never used by the estimator.
    pub valid: bool,
}

impl GnssPosition {
    /// Validates checked effective-time arithmetic. Invalid measurements
    /// remain representable evidence and are rejected by fusion policy.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.time.effective_time()?;
        Ok(self)
    }
}

/// Timed ECEF vector velocity field from a receiver solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssVelocity {
    /// Antenna reference-point ECEF vector velocity in metres per second.
    pub value: EcefVelocity,
    /// Velocity measurement epoch, independent of position time.
    /// The caller must account for receiver latency in the effective time.
    pub time: ObservationTime,
    /// Named terrestrial frame realization and epoch.
    pub frame: FrameId,
    /// Full/diagonal covariance or configured model.
    pub uncertainty: MeasurementUncertainty<Covariance3>,
    /// Whether the prepared measurement is valid for fusion.
    /// Invalid fields remain recordable but are never used by the estimator.
    pub valid: bool,
}

impl GnssVelocity {
    /// Validates checked effective-time arithmetic. Invalid measurements
    /// remain representable evidence and are rejected by fusion policy.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.time.effective_time()?;
        Ok(self)
    }
}

/// One independently timed receiver diagnostic.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedDiagnostic<T: Copy> {
    /// Diagnostic value.
    pub value: T,
    /// Receiver epoch and timing provenance for the diagnostic.
    pub time: ObservationTime,
    /// Age of the information at its effective epoch.
    pub age: DurationNs,
}

impl<T: Copy> TimedDiagnostic<T> {
    /// Validates checked effective-time arithmetic.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.time.effective_time()?;
        Ok(self)
    }
}

/// Independently optional and timed receiver health and freshness diagnostics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssDiagnostics {
    /// Correction age.
    pub correction_age: Option<TimedDiagnostic<DurationNs>>,
    /// Overall solution age.
    pub solution_age: Option<TimedDiagnostic<DurationNs>>,
    /// Receiver hardware/solution health.
    pub health: Option<TimedDiagnostic<ReceiverHealth>>,
}

impl GnssDiagnostics {
    /// Validates timing on every present diagnostic.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if let Some(value) = self.correction_age {
            value.validate()?;
        }
        if let Some(value) = self.solution_age {
            value.validate()?;
        }
        if let Some(value) = self.health {
            value.validate()?;
        }
        Ok(self)
    }
}

/// One fixed-size receiver-solution observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSolutionObservation {
    id: ObservationId,
    antenna_reference_point: ReferencePointId,
    position: Option<GnssPosition>,
    velocity: Option<GnssVelocity>,
    position_velocity_cross_covariance: Option<CrossCovariance3>,
    diagnostics: GnssDiagnostics,
}

impl GnssSolutionObservation {
    /// Validates and constructs an independently timed position/vector-velocity
    /// observation. At least one of `position` or `velocity` must be present.
    pub fn new(
        id: ObservationId,
        antenna_reference_point: ReferencePointId,
        position: Option<GnssPosition>,
        velocity: Option<GnssVelocity>,
        position_velocity_cross_covariance: Option<CrossCovariance3>,
        diagnostics: GnssDiagnostics,
    ) -> Result<Self, ValidationError> {
        if position.is_none() && velocity.is_none() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if let Some(value) = position {
            value.validate()?;
        }
        if let Some(value) = velocity {
            value.validate()?;
        }
        if let (Some(position), Some(velocity)) = (position, velocity) {
            if position.frame != velocity.frame {
                return Err(ValidationError::InvalidFrame);
            }
        }
        match (
            position_velocity_cross_covariance,
            position.and_then(|value| value.uncertainty.provided()),
            velocity.and_then(|value| value.uncertainty.provided()),
        ) {
            (Some(cross), Some(position), Some(velocity))
                if cross.forms_valid_joint(position, velocity) => {}
            (Some(_), _, _) => return Err(ValidationError::InvalidCovariance),
            (None, _, _) => {}
        }
        diagnostics.validate()?;
        Ok(Self {
            id,
            antenna_reference_point,
            position,
            velocity,
            position_velocity_cross_covariance,
            diagnostics,
        })
    }

    /// Returns the stable observation identity.
    #[must_use]
    pub const fn id(self) -> ObservationId {
        self.id
    }

    /// Returns the physical antenna reference point.
    #[must_use]
    pub const fn antenna_reference_point(self) -> ReferencePointId {
        self.antenna_reference_point
    }

    /// Returns the independently timed position field.
    #[must_use]
    pub const fn position(self) -> Option<GnssPosition> {
        self.position
    }

    /// Returns the independently timed vector-velocity field.
    #[must_use]
    pub const fn velocity(self) -> Option<GnssVelocity> {
        self.velocity
    }

    /// Returns measured position/velocity cross-covariance, when supplied.
    #[must_use]
    pub const fn position_velocity_cross_covariance(self) -> Option<CrossCovariance3> {
        self.position_velocity_cross_covariance
    }

    /// Returns independently timed diagnostics.
    #[must_use]
    pub const fn diagnostics(self) -> GnssDiagnostics {
        self.diagnostics
    }
}

/// Reason a fitted clock segment ended.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClockDiscontinuityReason {
    /// PPS was lost or no longer met the profile quality gate.
    PpsLoss,
    /// Receiver time jumped or reset.
    ReceiverReset,
    /// MCU timer reset or changed rate.
    TimerReset,
    /// Fitted residuals exceeded the model's validity envelope.
    FitInvalid,
    /// Explicit acquisition reconfiguration.
    Reconfigured,
}

/// Dimension of the live affine-clock error `(offset_s, fractional_drift)`.
pub const CLOCK_CONSIDER_DIMENSION: usize = 2;

/// Exact centered Gaussian bridge from the old live consider vector to the
/// next clock segment.
///
/// If the previous active consider vector is `c`, the next two clock errors
/// are `A*c + eta`, where `eta` is independent of `c` and has covariance `Q`.
/// Calibration/installation coordinates after the first two entries retain
/// their identity. This representation is sufficient to transform both the
/// navigation/consider cross covariance and the complete consider covariance
/// without exposing an estimator matrix type at the public seam.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockAffineBridge {
    active_consider_dimension: u8,
    next_reference_time: SessionTime,
    next_clock_from_previous_consider:
        [[f32; MAX_SHARED_PARAMETER_DIMENSION]; CLOCK_CONSIDER_DIMENSION],
    innovation_covariance_upper: [f32; 3],
}

impl ClockAffineBridge {
    /// Validates a fixed-capacity bridge. Coefficients outside the declared
    /// active prefix must be exact zero so profile mismatches fail closed.
    /// `innovation_covariance_upper` is `(offset variance, cross, drift
    /// variance)` in seconds/fractional-frequency units.
    pub fn new(
        active_consider_dimension: usize,
        next_reference_time: SessionTime,
        next_clock_from_previous_consider: [[f32; MAX_SHARED_PARAMETER_DIMENSION]; 2],
        innovation_covariance_upper: [f32; 3],
    ) -> Result<Self, ValidationError> {
        if !(CLOCK_CONSIDER_DIMENSION..=MAX_SHARED_PARAMETER_DIMENSION)
            .contains(&active_consider_dimension)
        {
            return Err(ValidationError::CapacityExceeded);
        }
        if !next_clock_from_previous_consider
            .iter()
            .flatten()
            .all(|value| value.is_finite())
        {
            return Err(ValidationError::NonFinite);
        }
        if next_clock_from_previous_consider.iter().any(|row| {
            row[active_consider_dimension..]
                .iter()
                .any(|value| *value != 0.0)
        }) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        validate_clock_covariance(innovation_covariance_upper)?;
        Ok(Self {
            active_consider_dimension: active_consider_dimension as u8,
            next_reference_time,
            next_clock_from_previous_consider,
            innovation_covariance_upper,
        })
    }

    /// Number of live consider coordinates consumed by the bridge.
    #[must_use]
    pub const fn active_consider_dimension(self) -> usize {
        self.active_consider_dimension as usize
    }

    /// Epoch at which the next segment's offset/drift error is expressed.
    #[must_use]
    pub const fn next_reference_time(self) -> SessionTime {
        self.next_reference_time
    }

    /// Row-major `2 x MAX_SHARED_PARAMETER_DIMENSION` Gaussian map.
    #[must_use]
    pub const fn next_clock_from_previous_consider(
        &self,
    ) -> &[[f32; MAX_SHARED_PARAMETER_DIMENSION]; CLOCK_CONSIDER_DIMENSION] {
        &self.next_clock_from_previous_consider
    }

    /// Upper triangle `(offset variance, cross, drift variance)` of `Q`.
    #[must_use]
    pub const fn innovation_covariance_upper(self) -> [f32; 3] {
        self.innovation_covariance_upper
    }
}

/// Absolute next-segment clock prior whose independence from retained
/// calibration/installation uncertainty is explicit.
///
/// Applying this prior necessarily discards and reinitializes a live
/// navigation state: setting its navigation cross covariance to zero while
/// retaining that state would silently erase shared timing information.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IndependentClockPrior {
    reference_time: SessionTime,
    covariance_upper: [f32; 3],
}

impl IndependentClockPrior {
    /// Validates `(offset variance, cross, drift variance)`.
    pub fn new(
        reference_time: SessionTime,
        covariance_upper: [f32; 3],
    ) -> Result<Self, ValidationError> {
        validate_clock_covariance(covariance_upper)?;
        Ok(Self {
            reference_time,
            covariance_upper,
        })
    }

    #[must_use]
    pub const fn reference_time(self) -> SessionTime {
        self.reference_time
    }

    #[must_use]
    pub const fn covariance_upper(self) -> [f32; 3] {
        self.covariance_upper
    }
}

/// Shared clock-uncertainty handling declared at a segment transition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClockTransitionUncertainty {
    /// Preserve live navigation through an exact affine Gaussian bridge.
    AffineBridge(ClockAffineBridge),
    /// Restart navigation from an explicitly independent next-segment prior.
    IndependentPrior(IndependentClockPrior),
    /// No qualified relationship or prior is available; live fusion remains
    /// unavailable until a later independent prior is supplied.
    Unavailable,
}

/// A fixed-size transition between immutable fitted clock segments.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockTransitionObservation {
    /// Canonical input identity.
    pub id: ObservationId,
    /// Transition time on the unchanged session timeline.
    pub at: SessionTime,
    /// Previous fitted model, if a qualified segment existed.
    pub previous_model: Option<ClockModelId>,
    /// New fitted model, if one qualified immediately.
    pub next_model: Option<ClockModelId>,
    /// New contiguous segment identity.
    pub next_segment: ClockSegmentId,
    /// Why the prior segment ended.
    pub reason: ClockDiscontinuityReason,
    /// Exact covariance bridge, restart prior, or explicit unavailability.
    pub uncertainty: ClockTransitionUncertainty,
}

impl ClockTransitionObservation {
    /// Validates model/uncertainty consistency independent of an engine
    /// profile. The live engine additionally requires an affine bridge's
    /// active dimension to match its preflighted consider layout.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.previous_model == self.next_model {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if self.next_model.is_none()
            && !matches!(self.uncertainty, ClockTransitionUncertainty::Unavailable)
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

fn validate_clock_covariance(covariance_upper: [f32; 3]) -> Result<(), ValidationError> {
    if !covariance_upper.iter().all(|value| value.is_finite()) {
        return Err(ValidationError::NonFinite);
    }
    let [offset_variance, cross, drift_variance] = covariance_upper;
    if offset_variance < 0.0 || drift_variance < 0.0 {
        return Err(ValidationError::InvalidCovariance);
    }
    let offset = f64::from(offset_variance);
    let cross = f64::from(cross);
    let drift = f64::from(drift_variance);
    if !is_positive_semidefinite_2x2(offset, cross, drift, 256.0 * f64::from(f32::EPSILON)) {
        return Err(ValidationError::InvalidCovariance);
    }
    Ok(())
}

/// Fixed-size observations accepted by [`crate::engine::LiveSession`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LiveObservation {
    /// Calibrated high-rate IMU evidence.
    Imu(ImuObservation),
    /// Receiver-solution position/vector-velocity evidence.
    GnssSolution(GnssSolutionObservation),
    /// Explicit clock-segment transition.
    ClockTransition(ClockTransitionObservation),
}

impl LiveObservation {
    /// Returns the stable input identity shared by all variants.
    #[must_use]
    pub const fn id(self) -> ObservationId {
        match self {
            Self::Imu(value) => value.id(),
            Self::GnssSolution(value) => value.id(),
            Self::ClockTransition(value) => value.id,
        }
    }
}

/// Deterministic corrected-frontier work credit for one live call.
///
/// This meters navigation-frontier operations, including IMU-slice planning,
/// filter propagation, delayed measurement updates, and segment commits. It is
/// not a wall-clock or whole-call budget: observation ingestion, bounded
/// history transfer/reanchoring, metric refresh, and present projection have
/// separate fixed-capacity bounds and still require target-hardware timing
/// qualification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub struct WorkQuota(u16);

impl WorkQuota {
    /// Largest frontier-work quota accepted for one call.
    pub const MAX_UNITS: u32 = u16::MAX as u32;

    /// Constructs a non-zero quota measured in profile-defined frontier-work
    /// units.
    pub const fn new(units: u32) -> Result<Self, ValidationError> {
        if units == 0 || units > Self::MAX_UNITS {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(Self(units as u16))
        }
    }

    /// Returns profile-defined frontier-work units.
    #[must_use]
    pub const fn units(self) -> u32 {
        self.0 as u32
    }

    pub(crate) const fn credits(self) -> u16 {
        self.0
    }
}

/// One transactional call into a live session.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveStep<'a> {
    /// At most one borrowed fixed-size observation; `None` drains frontier work.
    pub observation: Option<&'a LiveObservation>,
    /// Deterministic corrected-frontier work limit for this call.
    pub work: WorkQuota,
}

/// Statistical or policy outcome for a contract-valid input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InputDisposition {
    /// Contract-valid observation was queued behind the delayed frontier; its
    /// eventual statistical outcome is reported by a later live update.
    QueuedForFusion,
    /// Observation contributed to an estimator update.
    Fused,
    /// Innovation testing rejected the observation.
    StatisticallyRejected,
    /// Robust selection retained the observation with reduced weight.
    Downweighted,
    /// Observation arrived at or before an already processed live frontier.
    TooLateForLive,
    /// Observation was used only by initialization logic.
    InitializationOnly,
    /// Evidence was retained for host processing but not used live.
    RetainedForOffline,
}

/// Supported satellite constellation identity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Constellation {
    /// GPS.
    Gps,
    /// GLONASS.
    Glonass,
    /// Galileo.
    Galileo,
    /// BeiDou.
    Beidou,
    /// QZSS.
    Qzss,
    /// NavIC/IRNSS.
    Navic,
    /// SBAS.
    Sbas,
    /// Extensible receiver-defined constellation code.
    Other(u16),
}

/// Satellite identity independent of a particular signal.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct SatelliteId {
    /// Constellation.
    pub constellation: Constellation,
    /// Constellation-specific satellite/slot number.
    pub vehicle: u16,
}

impl SatelliteId {
    /// Validates a non-zero satellite/slot number.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        if self.vehicle == 0 {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Extensible signal identity and carrier-frequency metadata.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SignalId {
    /// Constellation owning the signal definition.
    pub constellation: Constellation,
    /// Versioned signal code within that constellation.
    pub code: u16,
    /// Carrier frequency in hertz when known.
    pub carrier_frequency_hz: Option<NonNegativeF64>,
}

impl SignalId {
    /// Validates a non-zero extensible signal code and any supplied positive
    /// carrier frequency.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        if self.code == 0
            || matches!(self.carrier_frequency_hz, Some(frequency) if frequency.get() == 0.0)
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Sign convention for receiver Doppler measurements.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DopplerSignConvention {
    /// Positive Doppler means increasing carrier phase.
    PositivePhaseRate,
    /// Positive Doppler means decreasing geometric range.
    PositiveClosingRange,
}

/// Raw pseudorange in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct PseudorangeMetres(FiniteF64);

impl PseudorangeMetres {
    /// Validates a strictly positive finite pseudorange.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        let value = FiniteF64::new(value)?;
        if value.get() <= 0.0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self(value))
    }

    /// Returns metres.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0.get()
    }
}

/// Raw carrier phase in cycles.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct CarrierPhaseCycles(FiniteF64);

impl CarrierPhaseCycles {
    /// Validates a finite carrier-phase value.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        Ok(Self(FiniteF64::new(value)?))
    }

    /// Returns carrier cycles.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0.get()
    }
}

/// Raw Doppler in hertz.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct DopplerHertz(FiniteF64);

impl DopplerHertz {
    /// Validates a finite Doppler value.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        Ok(Self(FiniteF64::new(value)?))
    }

    /// Returns hertz.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0.get()
    }
}

/// Complete opaque tracking word plus decoder revision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TrackingStatusWord {
    /// Receiver/log interpretation revision.
    pub revision: u16,
    /// Complete unmodified receiver word.
    pub raw: u64,
}

/// Documented receiver-native validity and continuity indicators.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReceiverTrackingIndicators {
    /// Code validity, when documented for the accepted message revision.
    pub code_valid: Option<bool>,
    /// Carrier-phase validity.
    pub phase_valid: Option<bool>,
    /// Doppler validity.
    pub doppler_valid: Option<bool>,
    /// Half-cycle ambiguity indication.
    pub half_cycle: Option<bool>,
    /// Parity-known indication.
    pub parity_known: Option<bool>,
    /// Receiver-native loss-of-lock indication.
    pub loss_of_lock: Option<bool>,
    /// Receiver-native cycle-slip indication.
    pub cycle_slip: Option<bool>,
}

/// Derived slip/continuity indicators with explicit derivation revision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DerivedContinuityIndicators {
    /// Lock-time reset detected.
    pub lock_time_reset: bool,
    /// Geometry-free combination discontinuity detected.
    pub geometry_free: bool,
    /// Melbourne-Wübbena discontinuity detected.
    pub melbourne_wubbena: bool,
    /// Doppler/carrier-phase inconsistency detected.
    pub doppler_phase: bool,
    /// Estimator innovation discontinuity detected.
    pub innovation: bool,
    /// Versioned derivation logic.
    pub derivation_revision: u32,
}

/// One typed raw signal observation with no heap ownership.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawSignalObservation {
    /// Satellite identity.
    pub satellite: SatelliteId,
    /// Signal identity and carrier frequency.
    pub signal: SignalId,
    /// Optional code pseudorange.
    pub pseudorange: Option<PseudorangeMetres>,
    /// Optional ambiguity-bearing carrier phase.
    pub carrier_phase: Option<CarrierPhaseCycles>,
    /// Optional Doppler.
    pub doppler: Option<DopplerHertz>,
    /// Doppler sign convention when Doppler is present.
    pub doppler_sign: DopplerSignConvention,
    /// Receiver-provided code standard deviation in metres.
    pub code_standard_deviation_m: Option<NonNegativeF64>,
    /// Receiver-provided phase standard deviation in cycles.
    pub phase_standard_deviation_cycles: Option<NonNegativeF64>,
    /// Carrier-to-noise density in dB-Hz.
    pub cn0_db_hz: Option<NonNegativeF64>,
    /// Continuous lock duration.
    pub lock_time: DurationNs,
    /// Complete opaque receiver status.
    pub tracking_status: TrackingStatusWord,
    /// Documented receiver indicators.
    pub receiver_indicators: ReceiverTrackingIndicators,
    /// Derived continuity/slip indicators.
    pub derived_indicators: DerivedContinuityIndicators,
    /// Ephemeris issue used for this satellite, when known.
    pub ephemeris_issue: Option<EphemerisIssue>,
}

impl RawSignalObservation {
    /// Validates identity and requires at least one raw observable.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.satellite.vehicle == 0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if self.signal.code == 0
            || self.signal.constellation != self.satellite.constellation
            || matches!(
                self.signal.carrier_frequency_hz,
                Some(frequency) if frequency.get() == 0.0
            )
            || (self.carrier_phase.is_some() && self.signal.carrier_frequency_hz.is_none())
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if self.pseudorange.is_none() && self.carrier_phase.is_none() && self.doppler.is_none() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Receiver role for a raw GNSS epoch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiverRole {
    /// Moving rover receiver.
    Rover,
    /// Surveyed or otherwise defined base receiver.
    Base,
}

/// Receiver clock estimate supplied with a raw epoch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReceiverClockObservation {
    /// Clock offset in seconds.
    pub offset_seconds: FiniteF64,
    /// Clock drift in seconds per second.
    pub drift_seconds_per_second: Option<FiniteF64>,
    /// Offset variance or configured model.
    pub offset_uncertainty: MeasurementUncertainty<Variance>,
    /// Drift variance or configured model when drift is supplied.
    pub drift_uncertainty: Option<MeasurementUncertainty<Variance>>,
}

/// Borrowed raw GNSS epoch for recording or host tight processing.
///
/// The view owns no variable-sized memory. Its signal count is checked against
/// both this semantic hard limit and the selected recording profile upstream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawGnssObservation<'a> {
    id: ObservationId,
    time: ObservationTime,
    antenna_reference_point: ReferencePointId,
    role: ReceiverRole,
    receiver_clock: Option<ReceiverClockObservation>,
    signals: &'a [RawSignalObservation],
    correction_provenance: Option<ContentDigestV1>,
}

impl<'a> RawGnssObservation<'a> {
    /// Validates and constructs a bounded borrowed epoch.
    pub fn new(
        id: ObservationId,
        time: ObservationTime,
        antenna_reference_point: ReferencePointId,
        role: ReceiverRole,
        receiver_clock: Option<ReceiverClockObservation>,
        signals: &'a [RawSignalObservation],
        correction_provenance: Option<ContentDigestV1>,
    ) -> Result<Self, ValidationError> {
        time.effective_time()?;
        if signals.is_empty() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if signals.len() > MAX_RAW_SIGNALS_PER_EPOCH {
            return Err(ValidationError::CapacityExceeded);
        }
        for signal in signals {
            signal.validate()?;
        }
        if receiver_clock.is_some_and(|clock| {
            clock.drift_seconds_per_second.is_some() != clock.drift_uncertainty.is_some()
        }) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self {
            id,
            time,
            antenna_reference_point,
            role,
            receiver_clock,
            signals,
            correction_provenance,
        })
    }

    /// Returns the raw epoch identity.
    #[must_use]
    pub const fn id(self) -> ObservationId {
        self.id
    }

    /// Returns the raw receiver epoch.
    #[must_use]
    pub const fn time(self) -> ObservationTime {
        self.time
    }

    /// Returns the receiver antenna reference point.
    #[must_use]
    pub const fn antenna_reference_point(self) -> ReferencePointId {
        self.antenna_reference_point
    }

    /// Returns rover/base role.
    #[must_use]
    pub const fn role(self) -> ReceiverRole {
        self.role
    }

    /// Returns optional receiver clock data.
    #[must_use]
    pub const fn receiver_clock(self) -> Option<ReceiverClockObservation> {
        self.receiver_clock
    }

    /// Returns the bounded borrowed signal collection.
    #[must_use]
    pub const fn signals(self) -> &'a [RawSignalObservation] {
        self.signals
    }

    /// Returns external correction/base provenance when attached.
    #[must_use]
    pub const fn correction_provenance(self) -> Option<ContentDigestV1> {
        self.correction_provenance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ids::{SourceId, UncertaintyModelId},
        time::{SampleSupport, SignedDurationNs, TimingBasis},
    };

    fn observation_time(ns: i64) -> ObservationTime {
        ObservationTime {
            registered_at: SessionTime::from_ns(ns),
            correction: SignedDurationNs::from_ns(0),
            independent_one_sigma: DurationNs::from_ns(10),
            clock_model: ClockModelId::new(1),
            support: SampleSupport::Point,
            basis: TimingBasis::PpsCorrelated,
        }
    }

    fn imu_time(ns: i64) -> ObservationTime {
        ObservationTime {
            support: SampleSupport::IntervalAverage {
                duration: DurationNs::from_ns(1_000_000),
            },
            ..observation_time(ns)
        }
    }

    fn imu(accel_axes: AxisStatus, status: ImuStatus) -> ImuObservation {
        let covariance = Covariance3::diagonal(1.0, 1.0, 1.0).unwrap();
        ImuObservation::new(
            ObservationId::new(SourceId::new(1), 2),
            FrameId::new(2),
            InputProfileId::new(3),
            TimedAngularRate {
                value: SensorAngularRate::new(0.0, 0.0, 0.0).unwrap(),
                time: imu_time(10),
                uncertainty: MeasurementUncertainty::Provided(covariance),
                axes: AxisStatus::VALID,
            },
            TimedSpecificForce {
                value: SensorSpecificForce::new(0.0, 0.0, 9.8).unwrap(),
                time: imu_time(10),
                uncertainty: MeasurementUncertainty::Provided(covariance),
                axes: accel_axes,
            },
            status,
        )
        .unwrap()
    }

    #[test]
    fn incomplete_or_saturated_vectors_are_not_repaired_by_the_engine() {
        for axes in [
            AxisStatus::new([false, true, true], [false; 3]),
            AxisStatus::new([true; 3], [true, false, false]),
        ] {
            let sample = imu(axes, ImuStatus::Valid);
            assert_eq!(
                sample.integration_eligibility(),
                ImuIntegrationEligibility::RejectSpecificForce
            );
            let sample = ImuObservation {
                angular_rate: TimedAngularRate {
                    axes,
                    ..sample.angular_rate()
                },
                specific_force: TimedSpecificForce {
                    axes: AxisStatus::VALID,
                    ..sample.specific_force()
                },
                ..sample
            };
            assert_eq!(
                sample.integration_eligibility(),
                ImuIntegrationEligibility::RejectAngularRate
            );
        }
    }

    #[test]
    fn prepared_input_status_distinguishes_degradation_gaps_and_discontinuity() {
        for (status, eligibility, continuity) in [
            (ImuStatus::Valid, ImuIntegrationEligibility::Complete, false),
            (
                ImuStatus::Degraded,
                ImuIntegrationEligibility::Complete,
                false,
            ),
            (
                ImuStatus::Unavailable,
                ImuIntegrationEligibility::RejectUnavailable,
                false,
            ),
            (
                ImuStatus::Initializing,
                ImuIntegrationEligibility::RejectInitialization,
                true,
            ),
            (
                ImuStatus::Discontinuity,
                ImuIntegrationEligibility::RejectDiscontinuity,
                true,
            ),
        ] {
            let sample = imu(AxisStatus::VALID, status);
            assert_eq!(sample.integration_eligibility(), eligibility);
            assert_eq!(sample.breaks_continuity(), continuity);
            assert_eq!(sample.is_degraded(), status == ImuStatus::Degraded);
        }
    }

    #[test]
    fn prepared_vectors_require_a_common_epoch_clock_and_support() {
        let sample = imu(AxisStatus::VALID, ImuStatus::Valid);
        let force = sample.specific_force();
        for time in [
            ObservationTime {
                registered_at: SessionTime::from_ns(11),
                ..force.time
            },
            ObservationTime {
                clock_model: ClockModelId::new(2),
                ..force.time
            },
            ObservationTime {
                support: SampleSupport::IntervalAverage {
                    duration: DurationNs::from_ns(2),
                },
                ..force.time
            },
        ] {
            assert_eq!(
                ImuObservation::new(
                    sample.id(),
                    sample.measurement_frame(),
                    sample.profile(),
                    sample.angular_rate(),
                    TimedSpecificForce { time, ..force },
                    sample.status()
                ),
                Err(ValidationError::InvalidTimeSpan)
            );
        }
        let time = ObservationTime {
            registered_at: SessionTime::from_ns(i64::MIN),
            ..force.time
        };
        assert_eq!(
            ImuObservation::new(
                sample.id(),
                sample.measurement_frame(),
                sample.profile(),
                TimedAngularRate {
                    time,
                    ..sample.angular_rate()
                },
                TimedSpecificForce { time, ..force },
                sample.status()
            ),
            Err(ValidationError::TimeOverflow)
        );
    }

    #[test]
    fn gnss_position_and_velocity_retain_independent_epochs() {
        let covariance = Covariance3::diagonal(1.0, 1.0, 1.0).unwrap();
        let observation = GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), 1),
            ReferencePointId::new(1),
            Some(GnssPosition {
                value: EcefPosition::new(1.0, 2.0, 3.0).unwrap(),
                time: observation_time(100),
                frame: FrameId::new(4),
                uncertainty: MeasurementUncertainty::Provided(covariance),
                valid: true,
            }),
            Some(GnssVelocity {
                value: EcefVelocity::new(4.0, 5.0, 6.0).unwrap(),
                time: observation_time(90),
                frame: FrameId::new(4),
                uncertainty: MeasurementUncertainty::Provided(covariance),
                valid: true,
            }),
            Some(CrossCovariance3::from_matrix([[0.0; 3]; 3]).unwrap()),
            GnssDiagnostics {
                correction_age: None,
                solution_age: None,
                health: None,
            },
        )
        .unwrap();
        assert_eq!(
            observation
                .position()
                .unwrap()
                .time
                .effective_time()
                .unwrap(),
            SessionTime::from_ns(100)
        );
        assert_eq!(
            observation
                .velocity()
                .unwrap()
                .time
                .effective_time()
                .unwrap(),
            SessionTime::from_ns(90)
        );
    }

    #[test]
    fn gnss_cross_covariance_requires_provided_compatible_marginals() {
        let modeled = MeasurementUncertainty::Modeled(UncertaintyModelId::new(1));
        let result = GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), 1),
            ReferencePointId::new(1),
            Some(GnssPosition {
                value: EcefPosition::new(1.0, 2.0, 3.0).unwrap(),
                time: observation_time(100),
                frame: FrameId::new(4),
                uncertainty: modeled,
                valid: true,
            }),
            Some(GnssVelocity {
                value: EcefVelocity::new(0.0, 0.0, 0.0).unwrap(),
                time: observation_time(100),
                frame: FrameId::new(4),
                uncertainty: modeled,
                valid: true,
            }),
            Some(CrossCovariance3::from_matrix([[0.0; 3]; 3]).unwrap()),
            GnssDiagnostics {
                correction_age: None,
                solution_age: None,
                health: None,
            },
        );
        assert_eq!(result, Err(ValidationError::InvalidCovariance));
    }

    #[test]
    fn raw_epoch_is_borrowed_bounded_and_requires_an_observable() {
        let empty_signal = RawSignalObservation {
            satellite: SatelliteId {
                constellation: Constellation::Gps,
                vehicle: 1,
            },
            signal: SignalId {
                constellation: Constellation::Gps,
                code: 1,
                carrier_frequency_hz: Some(NonNegativeF64::new(1_575_420_000.0).unwrap()),
            },
            pseudorange: None,
            carrier_phase: None,
            doppler: None,
            doppler_sign: DopplerSignConvention::PositivePhaseRate,
            code_standard_deviation_m: None,
            phase_standard_deviation_cycles: None,
            cn0_db_hz: None,
            lock_time: DurationNs::ZERO,
            tracking_status: TrackingStatusWord {
                revision: 1,
                raw: 0,
            },
            receiver_indicators: ReceiverTrackingIndicators {
                code_valid: None,
                phase_valid: None,
                doppler_valid: None,
                half_cycle: None,
                parity_known: None,
                loss_of_lock: None,
                cycle_slip: None,
            },
            derived_indicators: DerivedContinuityIndicators {
                lock_time_reset: false,
                geometry_free: false,
                melbourne_wubbena: false,
                doppler_phase: false,
                innovation: false,
                derivation_revision: 1,
            },
            ephemeris_issue: None,
        };
        assert_eq!(
            RawGnssObservation::new(
                ObservationId::new(SourceId::new(4), 1),
                observation_time(0),
                ReferencePointId::new(1),
                ReceiverRole::Rover,
                None,
                &[empty_signal],
                None,
            ),
            Err(ValidationError::IncompatibleDefinition)
        );
    }

    #[test]
    fn work_quota_enforces_the_single_call_frontier_credit_range() {
        assert_eq!(
            WorkQuota::new(0),
            Err(ValidationError::IncompatibleDefinition)
        );
        assert_eq!(
            WorkQuota::new(WorkQuota::MAX_UNITS).unwrap().units(),
            WorkQuota::MAX_UNITS
        );
        assert_eq!(
            WorkQuota::new(WorkQuota::MAX_UNITS + 1),
            Err(ValidationError::IncompatibleDefinition)
        );
    }

    #[test]
    fn clock_bridge_is_fixed_bounded_and_covariance_checked() {
        let mut mapping = [[0.0; MAX_SHARED_PARAMETER_DIMENSION]; 2];
        mapping[0][0] = 1.0;
        mapping[0][1] = 2.0;
        mapping[1][1] = 1.0;
        let bridge =
            ClockAffineBridge::new(4, SessionTime::from_ns(50), mapping, [4.0, 1.0, 2.0]).unwrap();
        assert_eq!(bridge.active_consider_dimension(), 4);
        assert_eq!(bridge.next_reference_time(), SessionTime::from_ns(50));

        mapping[0][4] = 1.0;
        assert_eq!(
            ClockAffineBridge::new(4, SessionTime::ZERO, mapping, [1.0, 0.0, 1.0]),
            Err(ValidationError::IncompatibleDefinition)
        );
        assert_eq!(
            IndependentClockPrior::new(SessionTime::ZERO, [1.0, 2.0, 1.0]),
            Err(ValidationError::InvalidCovariance)
        );
    }

    #[test]
    fn clock_covariance_psd_check_handles_f32_extremes() {
        let tiny = f32::from_bits(1);
        assert!(IndependentClockPrior::new(SessionTime::ZERO, [tiny, tiny, tiny]).is_ok());
        assert_eq!(
            IndependentClockPrior::new(SessionTime::ZERO, [tiny, tiny * 2.0, tiny]),
            Err(ValidationError::InvalidCovariance)
        );

        let huge = f32::MAX / 4.0;
        assert!(IndependentClockPrior::new(SessionTime::ZERO, [huge, huge, huge]).is_ok());
        assert_eq!(
            IndependentClockPrior::new(SessionTime::ZERO, [huge, huge * 2.0, huge]),
            Err(ValidationError::InvalidCovariance)
        );
    }

    #[test]
    fn clock_transition_requires_a_next_model_for_numeric_uncertainty() {
        let prior = IndependentClockPrior::new(SessionTime::ZERO, [1.0, 0.0, 1.0]).unwrap();
        let transition = ClockTransitionObservation {
            id: ObservationId::new(SourceId::new(8), 1),
            at: SessionTime::ZERO,
            previous_model: Some(ClockModelId::new(1)),
            next_model: None,
            next_segment: ClockSegmentId::new(2),
            reason: ClockDiscontinuityReason::PpsLoss,
            uncertainty: ClockTransitionUncertainty::IndependentPrior(prior),
        };
        assert_eq!(
            transition.validate(),
            Err(ValidationError::IncompatibleDefinition)
        );
    }
}
