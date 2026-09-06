//! Input, dynamics, fusion, and live navigation tuning.

use super::AttachmentModel;
use crate::error::ValidationError;
use crate::ids::{ContentDigestV1, DynamicsProfileId, InputProfileId};
use crate::math::{FiniteF64, NonNegativeF64, Probability};
use crate::time::DurationNs;
use crate::uncertainty::{Covariance3, MAX_SHARED_PARAMETER_DIMENSION, Variance};
use core::num::NonZeroU16;

/// Qualified receiver-solution temporal-correlation treatment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GnssCorrelationPolicy {
    /// Fixed deterministic decimation/blocking schedule.
    FixedDecimation { accept_every: NonZeroU16 },
    /// Sequence-level covariance inflation covering inter-epoch correlation.
    SequenceInflation { multiplier: NonNegativeF64 },
    /// Host-only first-order Gauss-Markov receiver error state.
    GaussMarkov {
        correlation_time: DurationNs,
        driving_variance: Variance,
    },
}

impl GnssCorrelationPolicy {
    /// Validates decimation/correlation parameters.
    pub fn validate(self) -> Result<Self, ValidationError> {
        match self {
            Self::SequenceInflation { multiplier } if multiplier.get() < 1.0 => {
                Err(ValidationError::IncompatibleDefinition)
            }
            Self::GaussMarkov {
                correlation_time, ..
            } if correlation_time.as_ns() == 0 => Err(ValidationError::IncompatibleDefinition),
            _ => Ok(self),
        }
    }
}

/// Stationary/motion classifier and ZUPT semantics for a dynamics profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationaryClassifierSpec {
    /// Probability of remaining stationary per classifier update.
    pub probability_stays_stationary: Probability,
    /// Probability of motion transitioning to stationary.
    pub probability_motion_to_stationary: Probability,
    /// Posterior threshold for entering stationary state.
    pub enter_probability: Probability,
    /// Posterior threshold for leaving stationary state.
    pub exit_probability: Probability,
    /// Minimum evidence window samples.
    pub minimum_window_samples: u16,
    /// ZUPT velocity covariance.
    pub zupt_covariance: Covariance3,
    /// ZUPT NIS rejection threshold.
    pub zupt_nis_threshold: NonNegativeF64,
}

impl StationaryClassifierSpec {
    /// Validates classifier hysteresis and non-empty evidence window.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.minimum_window_samples == 0
            || self.exit_probability.get() >= self.enter_probability.get()
            || self.zupt_nis_threshold.get() == 0.0
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Body-yaw observability thresholds for supplied/static/dynamic alignment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingObservabilitySpec {
    /// Minimum accumulated whitened yaw information.
    pub minimum_yaw_information: NonNegativeF64,
    /// Maximum posterior yaw variance in rad² for availability.
    pub maximum_yaw_variance_rad2: NonNegativeF64,
    /// Minimum horizontal velocity SNR for course output.
    pub minimum_course_snr: NonNegativeF64,
    /// Maximum course variance in rad².
    pub maximum_course_variance_rad2: NonNegativeF64,
    /// Required threshold dwell.
    pub dwell: DurationNs,
}

/// GNSS robust-fusion and temporal-correlation behavior.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssFusionSpec {
    /// Position covariance floor by receiver solution class family.
    pub position_covariance_floor: Covariance3,
    /// Velocity covariance floor.
    pub velocity_covariance_floor: Covariance3,
    /// Joint/update NIS rejection threshold.
    pub nis_rejection_threshold: NonNegativeF64,
    /// Robust-weight transition threshold.
    pub robust_weight_threshold: NonNegativeF64,
    /// Upper bound on covariance inflation applied by robust weighting.
    pub maximum_covariance_inflation: NonNegativeF64,
    /// Maximum accepted correction age.
    pub maximum_correction_age: DurationNs,
    /// Inter-epoch correlation treatment.
    pub correlation: GnssCorrelationPolicy,
}

impl GnssFusionSpec {
    /// Validates non-zero gates and a qualified correlation policy.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.nis_rejection_threshold.get() == 0.0
            || self.robust_weight_threshold.get() == 0.0
            || self.maximum_covariance_inflation.get() < 1.0
            || self.maximum_correction_age.as_ns() == 0
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.correlation.validate()?;
        Ok(self)
    }
}

/// Process-noise covariance densities for one dynamics validity envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProcessNoiseSpec {
    /// Accelerometer sample noise-density covariance.
    pub accelerometer: Covariance3,
    /// Gyroscope sample noise-density covariance.
    pub gyroscope: Covariance3,
    /// Accelerometer-bias random-walk covariance density.
    pub accelerometer_bias: Covariance3,
    /// Gyroscope-bias random-walk covariance density.
    pub gyroscope_bias: Covariance3,
}

/// Immutable motion/measurement-model profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicsProfileSpec {
    /// Stable dynamics profile identity.
    pub id: DynamicsProfileId,
    /// Physical attachment model qualified with this profile.
    pub attachment: AttachmentModel,
    /// Process-noise behavior.
    pub process_noise: ProcessNoiseSpec,
    /// Stationarity and ZUPT behavior.
    pub stationary: StationaryClassifierSpec,
    /// Heading/course observability behavior.
    pub heading: HeadingObservabilitySpec,
    /// Receiver-solution fusion behavior.
    pub gnss: GnssFusionSpec,
    /// Whether a calibrated land-vehicle non-holonomic constraint may be used.
    pub permits_non_holonomic_constraint: bool,
    /// Canonical full profile digest.
    pub digest: ContentDigestV1,
}

impl DynamicsProfileSpec {
    /// Validates all nested behavior.
    pub fn validate(self) -> Result<Self, ValidationError> {
        self.stationary.validate()?;
        self.gnss.validate()?;
        if self.id.get() == 0
            || self.digest.is_zero()
            || self.heading.minimum_yaw_information.get() == 0.0
            || self.heading.maximum_yaw_variance_rad2.get() == 0.0
            || self.heading.minimum_course_snr.get() == 0.0
            || self.heading.maximum_course_variance_rad2.get() == 0.0
            || self.heading.dwell.as_ns() == 0
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Prepared-measurement rate and engine capacity contract.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputProfileSpec {
    /// Stable prepared-input profile ID.
    pub id: InputProfileId,
    /// Minimum and maximum prepared IMU interval rate.
    pub imu_rate_hz_range: (NonNegativeF64, NonNegativeF64),
    /// Scheduling/buffer acceptance ceiling.
    pub maximum_imu_samples_per_second: u16,
    /// Maximum independently timed position updates per second.
    pub maximum_position_updates_per_second: u16,
    /// Maximum independently timed velocity updates per second.
    pub maximum_velocity_updates_per_second: u16,
    /// Maximum raw observations per receiver epoch; zero for solution-only inputs.
    pub maximum_raw_signals_per_epoch: u16,
    /// Canonical complete profile digest.
    pub digest: ContentDigestV1,
}

impl InputProfileSpec {
    /// Validates ordered non-zero rate limits and bounded raw count.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        if self.id.get() == 0
            || self.digest.is_zero()
            || self.imu_rate_hz_range.0.get() <= 0.0
            || self.imu_rate_hz_range.1.get() < self.imu_rate_hz_range.0.get()
            || self.maximum_imu_samples_per_second == 0
            || self.maximum_position_updates_per_second == 0
            || self.maximum_velocity_updates_per_second == 0
            || self.maximum_raw_signals_per_epoch as usize
                > crate::observation::MAX_RAW_SIGNALS_PER_EPOCH
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Deterministic covariance repair policy shared by processing levels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CovarianceRepairPolicy {
    /// Maximum permitted repair attempts after one operation.
    pub maximum_attempts: u8,
    /// Maximum total normalized diagonal regularization.
    pub maximum_total_regularization: NonNegativeF64,
}

/// Every numerical degree of freedom needed to instantiate the allocator-free
/// live initializer, gap model, predictor, and covariance conditioner. These
/// values are part of the qualified executable profile; the engine never
/// invents hidden tuning defaults at startup.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmbeddedLiveTuning {
    /// Normal-gravity magnitude used by the stationary classifier.
    pub gravity_magnitude_mps2: NonNegativeF64,
    /// Local vertical derivative of down-positive gravity magnitude in s^-2.
    /// A normal Earth surface profile normally supplies a small negative value.
    pub gravity_vertical_gradient_s2: FiniteF64,
    /// Gyroscope contribution scale in the stationary GLRT score.
    pub stationary_gyro_score_variance: NonNegativeF64,
    /// Specific-force norm contribution scale in the stationary GLRT score.
    pub stationary_force_norm_score_variance: NonNegativeF64,
    /// Minimum stationary samples before coarse alignment may complete.
    pub minimum_coarse_alignment_samples: u32,
    /// Minimum stationary samples before gyrocompassing may be attempted.
    pub minimum_gyrocompass_samples: u32,
    /// Whether this immutable executable profile has separately qualified
    /// gyrocompassing and its heading-uncertainty model.
    pub gyrocompassing_qualified: bool,
    /// Minimum norm of the Earth-rate/gravity TRIAD cross product.
    pub minimum_earth_rate_cross_gravity: NonNegativeF64,
    /// Largest sample variance accepted for static specific force.
    pub maximum_static_force_variance: NonNegativeF64,
    /// Largest sample variance accepted for static angular rate.
    pub maximum_static_gyro_variance: NonNegativeF64,
    /// Initial roll/pitch error variance.
    pub roll_pitch_variance_rad2: NonNegativeF64,
    /// Initial yaw variance when heading remains unavailable.
    pub unobservable_yaw_variance_rad2: NonNegativeF64,
    /// Residual accelerometer-bias prior after system calibration.
    pub accelerometer_bias_prior_mps2: [FiniteF64; 3],
    /// Residual gyroscope-bias prior after system calibration.
    pub gyroscope_bias_prior_rad_s: [FiniteF64; 3],
    /// Initial accelerometer-bias variances.
    pub accelerometer_bias_variance: [NonNegativeF64; 3],
    /// Initial gyroscope-bias variances.
    pub gyroscope_bias_variance: [NonNegativeF64; 3],
    /// Bounded jerk used when bridging a short complete-vector IMU gap.
    pub gap_jerk_one_sigma_mps3: [NonNegativeF64; 3],
    /// Bounded angular acceleration used for the same gap model.
    pub gap_angular_acceleration_one_sigma_rad_s2: [NonNegativeF64; 3],
    /// Norm of bias correction over which preintegrated first-order Jacobians
    /// remain qualified.
    pub bias_correction_validity_norm: NonNegativeF64,
    /// Predictor hard-reset velocity discrepancy.
    pub predictor_reset_velocity_mps: NonNegativeF64,
    /// Predictor hard-reset attitude discrepancy.
    pub predictor_reset_attitude_rad: NonNegativeF64,
    /// Per-error-coordinate scaling for covariance conditioning.
    pub covariance_state_scales: [NonNegativeF64; 15],
    /// Per-error-coordinate minimum variances.
    pub covariance_minimum_variances: [NonNegativeF64; 15],
    /// First normalized diagonal regularization attempt.
    pub covariance_repair_initial: NonNegativeF64,
    /// Multiplicative growth between regularization attempts.
    pub covariance_repair_growth: NonNegativeF64,
}

impl EmbeddedLiveTuning {
    /// Rejects incomplete or non-physical tuning before any live state exists.
    pub fn validate(self) -> Result<Self, ValidationError> {
        let strictly_positive = [
            self.gravity_magnitude_mps2,
            self.stationary_gyro_score_variance,
            self.stationary_force_norm_score_variance,
            self.minimum_earth_rate_cross_gravity,
            self.maximum_static_force_variance,
            self.maximum_static_gyro_variance,
            self.roll_pitch_variance_rad2,
            self.unobservable_yaw_variance_rad2,
            self.bias_correction_validity_norm,
            self.predictor_reset_velocity_mps,
            self.predictor_reset_attitude_rad,
            self.covariance_repair_initial,
            self.covariance_repair_growth,
        ];
        if self.minimum_coarse_alignment_samples == 0
            || self.minimum_gyrocompass_samples < self.minimum_coarse_alignment_samples
            || strictly_positive.iter().any(|value| value.get() <= 0.0)
            || self
                .covariance_state_scales
                .iter()
                .any(|value| value.get() <= 0.0)
            || self.covariance_repair_growth.get() < 1.0
            || self.gravity_vertical_gradient_s2.get().abs() > 1.0e-3
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(self)
    }
}

/// Live mechanization, delayed fusion, smoothing, and predictor behavior.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigationProfileSpec {
    /// Profile revision.
    pub revision: u32,
    /// Corrected navigation cadence.
    pub navigation_cadence_hz: u16,
    /// Effective-time reorder delay.
    pub fusion_delay: DurationNs,
    /// Additional fixed-lag extended RTS lookahead before trajectory and
    /// metric publication. Zero keeps the forward-filter comparison path;
    /// supported live profiles retain at most 100 milliseconds.
    pub smoothing_lag: DurationNs,
    /// Additional retained history beyond the delay.
    pub history_guard: DurationNs,
    /// Largest complete-vector IMU gap that may be bridged.
    pub maximum_bridgeable_imu_gap: DurationNs,
    /// Re-anchor threshold in metres.
    pub reanchor_distance_m: NonNegativeF64,
    /// Re-anchor hysteresis in metres.
    pub reanchor_hysteresis_m: NonNegativeF64,
    /// Fixed consider-block dimension, including active clock offset/drift.
    pub consider_dimension: u8,
    /// Predictor complementary-correction time constant.
    pub predictor_time_constant: DurationNs,
    /// Predictor hard-reset position discrepancy in metres.
    pub predictor_reset_position_m: NonNegativeF64,
    /// Covariance regularization contract.
    pub covariance_repair: CovarianceRepairPolicy,
    /// Complete qualified embedded numerical tuning with no hidden defaults.
    pub embedded_tuning: EmbeddedLiveTuning,
    /// Canonical complete profile digest.
    pub digest: ContentDigestV1,
}

impl NavigationProfileSpec {
    /// Validates bounded cadence, delay/history, gap, and consider dimensions.
    pub fn validate(self) -> Result<Self, ValidationError> {
        let history = match self
            .fusion_delay
            .as_ns()
            .checked_add(self.history_guard.as_ns())
        {
            Some(value) => value,
            None => return Err(ValidationError::TimeOverflow),
        };
        if self.revision == 0
            || self.digest.is_zero()
            || self.navigation_cadence_hz < 200
            || self.navigation_cadence_hz > 400
            || history > 500_000_000
            || self.smoothing_lag.as_ns() > 100_000_000
            || self.maximum_bridgeable_imu_gap.as_ns() > 10_000_000
            || self.maximum_bridgeable_imu_gap.as_ns() == 0
            || self.reanchor_distance_m.get() == 0.0
            || self.reanchor_hysteresis_m.get() >= self.reanchor_distance_m.get()
            || self.consider_dimension < 2
            || self.consider_dimension as usize > MAX_SHARED_PARAMETER_DIMENSION
            || self.predictor_time_constant.as_ns() == 0
            || self.predictor_reset_position_m.get() == 0.0
            || self.covariance_repair.maximum_attempts == 0
            || self.covariance_repair.maximum_attempts > 4
            || self.covariance_repair.maximum_total_regularization.get()
                < self.embedded_tuning.covariance_repair_initial.get()
        {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.embedded_tuning.validate()?;
        Ok(self)
    }
}
