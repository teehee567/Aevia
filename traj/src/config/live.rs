//! Live initialization, clock uncertainty, and session specifications.

use super::{EngineConfig, LiveResourceLimits};
use crate::error::ValidationError;
use crate::ids::{ClockModelId, ClockSegmentId, SessionId};
use crate::math::FiniteF64;
use crate::metric::{LiveMetricPlan, MetricDefinition};
use crate::time::SessionTime;
use crate::uncertainty::{
    MAX_SHARED_PARAMETER_DIMENSION, SharedParameterCovariance, Variance,
    is_positive_semidefinite_2x2,
};

/// Optional user/survey supplied body heading used during initialization.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialHeading {
    /// Heading in radians, normalized by the constructor to `[-π, π)`.
    radians: f64,
    /// Supplied heading variance in rad².
    pub variance: Variance,
}

impl InitialHeading {
    /// Validates and normalizes a heading angle.
    pub fn new(radians: f64, variance: Variance) -> Result<Self, ValidationError> {
        if !radians.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        let two_pi = 2.0 * core::f64::consts::PI;
        let shifted_remainder = (radians + core::f64::consts::PI) % two_pi;
        let positive_remainder = if shifted_remainder < 0.0 {
            shifted_remainder + two_pi
        } else {
            shifted_remainder
        };
        let normalized = positive_remainder - core::f64::consts::PI;
        Ok(Self {
            radians: normalized,
            variance,
        })
    }

    /// Returns normalized heading radians.
    #[must_use]
    pub const fn radians(self) -> f64 {
        self.radians
    }
}

/// Maximum calibration/installation coordinates alongside two clock entries
/// in the bounded live consider vector.
pub const MAX_INITIAL_CLOCK_SHARED_DIMENSION: usize = MAX_SHARED_PARAMETER_DIMENSION - 2;

/// Clock-to-calibration/installation cross covariance in the exact shared
/// parameter ordering of [`super::SharedParameterSet`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockSharedCrossCovariance {
    shared_dimension: u8,
    values: [[f64; MAX_SHARED_PARAMETER_DIMENSION]; 2],
}

impl ClockSharedCrossCovariance {
    /// Validates a fixed-capacity `2 x shared_dimension` block. Values outside
    /// the declared prefix must be exact zero to prevent dimension mismatch
    /// from being hidden by unused storage.
    pub fn new(
        shared_dimension: usize,
        values: [[f64; MAX_SHARED_PARAMETER_DIMENSION]; 2],
    ) -> Result<Self, ValidationError> {
        if shared_dimension > MAX_INITIAL_CLOCK_SHARED_DIMENSION {
            return Err(ValidationError::CapacityExceeded);
        }
        if !values.iter().flatten().all(|value| value.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        if values
            .iter()
            .any(|row| row[shared_dimension..].iter().any(|value| *value != 0.0))
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self {
            shared_dimension: shared_dimension as u8,
            values,
        })
    }

    /// Declares exact independence from a shared block of known dimension.
    pub fn independent(shared_dimension: usize) -> Result<Self, ValidationError> {
        Self::new(shared_dimension, [[0.0; MAX_SHARED_PARAMETER_DIMENSION]; 2])
    }

    #[must_use]
    pub const fn shared_dimension(self) -> usize {
        self.shared_dimension as usize
    }

    #[must_use]
    pub const fn values(&self) -> &[[f64; MAX_SHARED_PARAMETER_DIMENSION]; 2] {
        &self.values
    }
}

/// Fixed-mean affine clock offset/drift prior carried in the first two
/// Schmidt/consider coordinates of an embedded session.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialClockConsiderPrior {
    /// Fitted clock model to which the prior applies.
    pub model: ClockModelId,
    /// Initial contiguous clock segment carrying this model/prior.
    pub segment: ClockSegmentId,
    /// Session epoch at which offset/drift covariance is expressed.
    pub reference_time: SessionTime,
    /// Clock-offset variance in seconds squared.
    pub offset_variance_s2: Variance,
    /// Fractional-frequency/drift variance.
    pub drift_variance: Variance,
    /// Offset/drift covariance in seconds.
    pub offset_drift_covariance_s: FiniteF64,
    /// Declared covariance with every calibration/installation coordinate.
    pub cross_covariance_with_shared: ClockSharedCrossCovariance,
}

impl InitialClockConsiderPrior {
    /// Validates the complete two-by-two covariance as positive semidefinite.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.model.get() == 0 || self.segment.get() == 0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        if !is_positive_semidefinite_2x2(
            self.offset_variance_s2.get(),
            self.offset_drift_covariance_s.get(),
            self.drift_variance.get(),
            256.0 * f64::EPSILON,
        ) {
            Err(ValidationError::InvalidCovariance)
        } else {
            Ok(self)
        }
    }

    /// Validates the complete joint clock-plus-shared covariance and exact
    /// ordering/dimension agreement with the calibration bundle.
    pub fn validate_with_shared(
        self,
        shared: SharedParameterCovariance<'_>,
    ) -> Result<Self, ValidationError> {
        self.validate()?;
        validate_initial_consider_covariance(shared, self)?;
        Ok(self)
    }
}

/// Complete semantic input for one allocator-free live session.
#[derive(Clone, Debug, PartialEq)]
pub struct LiveSpec<'a> {
    /// Session identity used for deterministic result namespaces.
    pub session_id: SessionId,
    /// Immutable estimator/installation configuration.
    pub engine: EngineConfig<'a>,
    /// Validated bounded metric definitions.
    /// Compiled definitions live in caller-owned immutable configuration
    /// storage. Borrowing them keeps the maximum-capacity plan out of the
    /// ESP32-S31 preflight/start call frames; the active tracker copies the
    /// required definitions directly into PSRAM during start.
    pub metrics: &'a LiveMetricPlan,
    /// Complete firmware-wide live resource limits.
    pub resources: LiveResourceLimits,
    /// Optional supplied initial heading.
    pub initial_heading: Option<InitialHeading>,
    /// Initial active clock segment's shared offset/drift uncertainty.
    pub initial_clock_prior: InitialClockConsiderPrior,
}

impl LiveSpec<'_> {
    /// Validates semantics and live plan/resource compatibility. Production
    /// preflight must separately require [`EngineConfig::is_qualified`].
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.session_id.is_zero() {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.engine.validate()?;
        self.initial_clock_prior
            .validate_with_shared(self.engine.calibration.shared_parameters.covariance)?;
        self.resources.validate_v2_mini()?;
        let mut gate_target_count = 0_usize;
        for definition in self.metrics.plan().definitions() {
            match definition {
                MetricDefinition::Lap(plan) => gate_target_count += plan.gates().len(),
                MetricDefinition::Drag(plan) => gate_target_count += plan.targets().len(),
                MetricDefinition::Distance(_)
                | MetricDefinition::Activity(_)
                | MetricDefinition::Ski(_) => {}
            }
        }
        let metric_limits = self.metrics.limits();
        if gate_target_count > usize::from(self.resources.maximum_gates_and_targets)
            || metric_limits.max_active_candidates
                > self.resources.maximum_active_candidates_per_segment
            || metric_limits.max_mutations_per_step
                > self.resources.maximum_metric_mutations_per_step
        {
            return Err(ValidationError::CapacityExceeded);
        }
        Ok(())
    }
}

fn validate_initial_consider_covariance(
    shared: SharedParameterCovariance<'_>,
    clock: InitialClockConsiderPrior,
) -> Result<(), ValidationError> {
    let shared_dimension = shared.dimension();
    if clock.cross_covariance_with_shared.shared_dimension() != shared_dimension {
        return Err(ValidationError::InvalidCovariance);
    }
    let total_dimension = shared_dimension
        .checked_add(2)
        .ok_or(ValidationError::CapacityExceeded)?;
    let mut upper =
        [0.0; MAX_SHARED_PARAMETER_DIMENSION * (MAX_SHARED_PARAMETER_DIMENSION + 1) / 2];
    let mut packed = 0;
    let mut shared_packed = 0;
    let clock_cross = clock.cross_covariance_with_shared.values();
    let shared_upper = shared.upper_triangle();
    for row in 0..total_dimension {
        for column in row..total_dimension {
            upper[packed] = match (row, column) {
                (0, 0) => clock.offset_variance_s2.get(),
                (0, 1) => clock.offset_drift_covariance_s.get(),
                (0, column) => clock_cross[0][column - 2],
                (1, 1) => clock.drift_variance.get(),
                (1, column) => clock_cross[1][column - 2],
                (_, _) => {
                    let value = shared_upper[shared_packed];
                    shared_packed += 1;
                    value
                }
            };
            packed += 1;
        }
    }
    SharedParameterCovariance::new(total_dimension, &upper[..packed]).map(|_| ())
}
