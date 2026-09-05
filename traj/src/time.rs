//! Exact session time and timing provenance.

use core::cmp::Ordering;

use crate::{error::ValidationError, ids::ClockModelId};

/// Nanoseconds relative to one immutable [`SessionEpoch`].
#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
#[repr(transparent)]
pub struct SessionTime(i64);

impl SessionTime {
    /// Session-relative zero.
    pub const ZERO: Self = Self(0);

    /// Creates a session-relative timestamp.
    #[must_use]
    pub const fn from_ns(value: i64) -> Self {
        Self(value)
    }

    /// Returns session-relative nanoseconds.
    #[must_use]
    pub const fn as_ns(self) -> i64 {
        self.0
    }

    /// Adds a signed correction with overflow checking.
    pub const fn checked_add(self, duration: SignedDurationNs) -> Option<Self> {
        match self.0.checked_add(duration.0) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Computes a signed difference with overflow checking.
    pub const fn checked_duration_since(self, earlier: Self) -> Option<SignedDurationNs> {
        match self.0.checked_sub(earlier.0) {
            Some(value) => Some(SignedDurationNs(value)),
            None => None,
        }
    }
}

impl Ord for SessionTime {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.cmp(&other.0)
    }
}

impl PartialOrd for SessionTime {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Signed nanoseconds.
#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct SignedDurationNs(i64);

impl SignedDurationNs {
    /// Creates a signed duration.
    #[must_use]
    pub const fn from_ns(value: i64) -> Self {
        Self(value)
    }

    /// Returns nanoseconds.
    #[must_use]
    pub const fn as_ns(self) -> i64 {
        self.0
    }

    /// Converts to seconds for numerical calculations.
    #[must_use]
    pub fn as_seconds_f64(self) -> f64 {
        self.0 as f64 * 1.0e-9
    }
}

/// Non-negative nanoseconds.
#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct DurationNs(u64);

impl DurationNs {
    /// Zero duration.
    pub const ZERO: Self = Self(0);

    /// Creates a non-negative duration.
    #[must_use]
    pub const fn from_ns(value: u64) -> Self {
        Self(value)
    }

    /// Returns nanoseconds.
    #[must_use]
    pub const fn as_ns(self) -> u64 {
        self.0
    }

    /// Converts to seconds for numerical calculations.
    #[must_use]
    pub fn as_seconds_f64(self) -> f64 {
        self.0 as f64 * 1.0e-9
    }
}

/// Immutable origin for one monotonic session timeline.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SessionEpoch {
    /// Continuous GPS time origin.
    Gps { week: u32, tow_ns: u64 },
    /// A local boot/timer origin used before GPS time is resolved.
    Local {
        boot_nonce: u128,
        timer_origin: u64,
        timer_hz: u32,
    },
}

impl SessionEpoch {
    /// Validates ranges that can be checked without an external clock model.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        match self {
            Self::Gps { tow_ns, .. } if tow_ns >= 604_800_000_000_000 => {
                Err(ValidationError::TimeOutOfRange)
            }
            Self::Local { timer_hz: 0, .. } => Err(ValidationError::TimeOutOfRange),
            _ => Ok(self),
        }
    }
}

/// How a sensor value is supported in time.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleSupport {
    /// Filtered value associated with one effective instant.
    Point,
    /// Average over a closed-open interval ending at the registered epoch.
    IntervalAverage { duration: DurationNs },
    /// Interpolated sample with an explicit support width.
    Interpolated { support: DurationNs },
}

/// Provenance for an effective observation epoch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimingBasis {
    PpsCorrelated,
    SensorCounterAnchored,
    ModeledLatency,
    ArrivalOnly,
}

/// Exact observation timing plus independent sample-specific uncertainty.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ObservationTime {
    pub registered_at: SessionTime,
    pub correction: SignedDurationNs,
    pub independent_one_sigma: DurationNs,
    pub clock_model: ClockModelId,
    pub support: SampleSupport,
    pub basis: TimingBasis,
}

impl ObservationTime {
    /// Returns the corrected measurement epoch.
    pub const fn effective_time(self) -> Result<SessionTime, ValidationError> {
        match self.registered_at.checked_add(self.correction) {
            Some(value) => Ok(value),
            None => Err(ValidationError::TimeOverflow),
        }
    }
}

/// A non-empty closed session span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimeSpan {
    start: SessionTime,
    end: SessionTime,
}

impl TimeSpan {
    /// Creates a span when `end >= start`.
    pub const fn new(start: SessionTime, end: SessionTime) -> Result<Self, ValidationError> {
        if end.as_ns() < start.as_ns() {
            return Err(ValidationError::InvalidTimeSpan);
        }
        Ok(Self { start, end })
    }

    #[must_use]
    pub const fn start(self) -> SessionTime {
        self.start
    }

    #[must_use]
    pub const fn end(self) -> SessionTime {
        self.end
    }

    #[must_use]
    pub const fn contains(self, time: SessionTime) -> bool {
        time.as_ns() >= self.start.as_ns() && time.as_ns() <= self.end.as_ns()
    }
}

/// Immutable later resolution from local time to continuous GPS time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EpochResolution {
    pub clock_model: ClockModelId,
    pub local_reference: SessionTime,
    pub gps_week: u32,
    pub gps_tow_ns: u64,
    pub offset_variance_ns2: f64,
    pub drift_variance: f64,
    pub offset_drift_covariance_ns: f64,
}

impl EpochResolution {
    pub fn validate(self) -> Result<Self, ValidationError> {
        let finite = self.offset_variance_ns2.is_finite()
            && self.drift_variance.is_finite()
            && self.offset_drift_covariance_ns.is_finite();
        if !finite || self.offset_variance_ns2 < 0.0 || self.drift_variance < 0.0 {
            return Err(ValidationError::InvalidCovariance);
        }
        if self.gps_tow_ns >= 604_800_000_000_000 {
            return Err(ValidationError::TimeOutOfRange);
        }
        if !covariance_2x2_is_psd(
            self.offset_variance_ns2,
            self.offset_drift_covariance_ns,
            self.drift_variance,
        ) {
            return Err(ValidationError::InvalidCovariance);
        }
        Ok(self)
    }
}

/// Overflow/underflow-safe positive-semidefinite test for a symmetric 2x2
/// covariance `[a, c; c, b]`.
pub(crate) fn covariance_2x2_is_psd(a: f64, c: f64, b: f64) -> bool {
    if !a.is_finite() || !b.is_finite() || !c.is_finite() || a < 0.0 || b < 0.0 {
        return false;
    }
    let cross = c.abs();
    if cross == 0.0 {
        return true;
    }
    if a == 0.0 || b == 0.0 {
        return false;
    }

    // `c^2 <= a*b` is equivalent to `|c|/sqrt(b) <= sqrt(a)`, but this
    // arrangement never forms either potentially overflowing/underflowing
    // product. The right side is finite for every finite `a`; an overflowing
    // left division is therefore decisively non-PSD.
    let normalized_cross = cross / crate::scalar_math::sqrt(b);
    normalized_cross.is_finite()
        && normalized_cross <= crate::scalar_math::sqrt(a) * (1.0 + 256.0 * f64::EPSILON)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn effective_time_is_checked() {
        let time = ObservationTime {
            registered_at: SessionTime::from_ns(i64::MAX),
            correction: SignedDurationNs::from_ns(1),
            independent_one_sigma: DurationNs::ZERO,
            clock_model: ClockModelId::new(1),
            support: SampleSupport::Point,
            basis: TimingBasis::PpsCorrelated,
        };
        assert_eq!(time.effective_time(), Err(ValidationError::TimeOverflow));
    }

    #[test]
    fn span_rejects_reverse_time() {
        assert_eq!(
            TimeSpan::new(SessionTime::from_ns(2), SessionTime::from_ns(1)),
            Err(ValidationError::InvalidTimeSpan)
        );
    }

    #[test]
    fn epoch_resolution_covariance_is_scale_independent() {
        let resolution = EpochResolution {
            clock_model: ClockModelId::new(1),
            local_reference: SessionTime::from_ns(0),
            gps_week: 2_400,
            gps_tow_ns: 0,
            offset_variance_ns2: 1.0e-150,
            drift_variance: 1.0e-150,
            offset_drift_covariance_ns: 1.0e-150,
        };
        assert_eq!(resolution.validate(), Ok(resolution));

        let indefinite = EpochResolution {
            offset_drift_covariance_ns: 2.0e-150,
            ..resolution
        };
        assert_eq!(
            indefinite.validate(),
            Err(ValidationError::InvalidCovariance)
        );

        let smallest = f64::from_bits(1);
        let underflow_indefinite = EpochResolution {
            offset_variance_ns2: smallest,
            drift_variance: smallest,
            offset_drift_covariance_ns: smallest * 2.0,
            ..resolution
        };
        assert_eq!(
            underflow_indefinite.validate(),
            Err(ValidationError::InvalidCovariance)
        );

        let extreme_psd = EpochResolution {
            offset_variance_ns2: f64::MAX,
            drift_variance: f64::MAX,
            offset_drift_covariance_ns: f64::MAX,
            ..resolution
        };
        assert_eq!(extreme_psd.validate(), Ok(extreme_psd));
    }
}
