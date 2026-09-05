//! Support-aware, allocator-free IMU preintegration.
//!
//! Angular-rate and specific-force values are interval averages.  Each interval is integrated once
//! with the SO(3) exponential and its first two integrals; composition of those
//! rotations supplies coning/sculling behaviour without applying a second
//! midpoint correction later in the ESKF.

use nalgebra::{ArrayStorage, Matrix3, Quaternion, SMatrix, UnitQuaternion, Vector3};

use crate::time::SessionTime;

use super::state::{
    left_jacobian, right_jacobian, second_rotation_integral, skew, so3_exp, vector_is_finite,
};

pub(crate) const PREINT_DIM: usize = 9;
pub(crate) const BIAS_DIM: usize = 6;
pub(crate) const MAX_BATCH_SAMPLES: u16 = 32;

pub(crate) type PreintCovariance = SMatrix<f32, PREINT_DIM, PREINT_DIM>;
pub(crate) type BiasJacobian = SMatrix<f32, PREINT_DIM, BIAS_DIM>;
pub(crate) type GapDerivativeCovariance = SMatrix<f32, BIAS_DIM, BIAS_DIM>;
pub(crate) type GapJacobian = SMatrix<f32, PREINT_DIM, BIAS_DIM>;
pub(crate) type ImuSampleCovariance = SMatrix<f32, BIAS_DIM, BIAS_DIM>;
pub(crate) type ImuSampleJacobian = SMatrix<f32, PREINT_DIM, BIAS_DIM>;

pub(crate) fn imu_sample_covariance(
    accelerometer: CompactCovariance3,
    gyroscope: CompactCovariance3,
) -> ImuSampleCovariance {
    let mut covariance = ImuSampleCovariance::zeros();
    covariance
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&accelerometer.to_matrix());
    covariance
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&gyroscope.to_matrix());
    covariance
}

/// Compact symmetric sample covariance kept with delayed raw IMU evidence.
/// Full matrices are reconstructed only when a slice is preintegrated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CompactCovariance3 {
    upper: [f32; 6],
}

impl CompactCovariance3 {
    pub(crate) const ZERO: Self = Self { upper: [0.0; 6] };

    pub(crate) fn from_matrix(matrix: Matrix3<f32>) -> Result<Self, PreintegrationError> {
        if !covariance_density_is_valid(&matrix) {
            return Err(PreintegrationError::InvalidSampleCovariance);
        }
        Ok(Self {
            upper: [
                matrix[(0, 0)],
                matrix[(0, 1)],
                matrix[(0, 2)],
                matrix[(1, 1)],
                matrix[(1, 2)],
                matrix[(2, 2)],
            ],
        })
    }

    pub(crate) const fn to_matrix(self) -> Matrix3<f32> {
        Matrix3::from_array_storage(ArrayStorage([
            [self.upper[0], self.upper[1], self.upper[2]],
            [self.upper[1], self.upper[3], self.upper[4]],
            [self.upper[2], self.upper[4], self.upper[5]],
        ]))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ImuNoise {
    /// Continuous accelerometer covariance density, including cross-axis
    /// terms, in `(m/s^2)^2/Hz`.
    pub(crate) accel_covariance_density: Matrix3<f32>,
    /// Continuous gyro covariance density, including cross-axis terms, in
    /// `(rad/s)^2/Hz`.
    pub(crate) gyro_covariance_density: Matrix3<f32>,
}

impl ImuNoise {
    pub(crate) fn is_valid(&self) -> bool {
        covariance_density_is_valid(&self.accel_covariance_density)
            && covariance_density_is_valid(&self.gyro_covariance_density)
    }
}

/// Checks a symmetric 3-by-3 covariance density without decomposition or
/// allocation. Scaling first prevents the principal-minor products from
/// overflowing when a valid configuration is close to `f32::MAX`.
pub(super) fn covariance_density_is_valid(value: &Matrix3<f32>) -> bool {
    if !value.iter().all(|entry| entry.is_finite())
        || value[(0, 1)] != value[(1, 0)]
        || value[(0, 2)] != value[(2, 0)]
        || value[(1, 2)] != value[(2, 1)]
    {
        return false;
    }
    let scale = value
        .iter()
        .fold(0.0_f32, |largest, entry| largest.max(entry.abs()));
    if scale == 0.0 {
        return true;
    }
    let normalized = value / scale;
    let tolerance = 128.0 * f32::EPSILON;
    if (0..3).any(|axis| normalized[(axis, axis)] < -tolerance) {
        return false;
    }
    for (first, second) in [(0, 1), (0, 2), (1, 2)] {
        let minor = normalized[(first, first)] * normalized[(second, second)]
            - normalized[(first, second)] * normalized[(second, first)];
        if !minor.is_finite() || minor < -tolerance {
            return false;
        }
    }
    let determinant = normalized.determinant();
    determinant.is_finite() && determinant >= -tolerance
}

/// One complete, already calibrated and support-aligned IMU interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ImuInterval {
    pub(crate) start: SessionTime,
    pub(crate) end: SessionTime,
    pub(crate) omega_ib_b: Vector3<f32>,
    pub(crate) specific_force_b: Vector3<f32>,
    pub(crate) degraded_input: bool,
    /// Zero for measured support. A synthetic held-vector gap stores one plus
    /// the nanoseconds elapsed from the beginning of the original gap to this
    /// piece's start. The sentinel encoding keeps split metadata to four bytes
    /// in the 1,024-entry embedded raw-history ring.
    pub(crate) gap_elapsed_ns_plus_one: u32,
    /// Fixed installation rotation needed to reconstruct the two calibration
    /// consider Jacobians without storing two redundant 3-by-3 matrices.
    pub(crate) body_from_sensor: UnitQuaternion<f32>,
    /// Covariance of this prepared interval-average specific-force vector in body
    /// axes. It is independent of the continuous profile noise density.
    pub(crate) accel_sample_covariance: CompactCovariance3,
    /// Covariance of this prepared interval-average angular-rate vector in body axes.
    pub(crate) gyro_sample_covariance: CompactCovariance3,
    /// First coordinate of the installation boresight/misalignment block in
    /// the fixed consider vector, when this profile propagates it.
    pub(crate) calibration_consider_start: Option<u8>,
}

impl ImuInterval {
    pub(crate) const fn is_gap(&self) -> bool {
        self.gap_elapsed_ns_plus_one != 0
    }

    pub(crate) fn gap_elapsed_seconds(&self) -> Option<f32> {
        self.gap_elapsed_ns_plus_one
            .checked_sub(1)
            .map(|nanoseconds| nanoseconds as f32 * 1.0e-9)
    }

    pub(crate) fn gap_origin(&self) -> Result<Option<SessionTime>, PreintegrationError> {
        let Some(elapsed_ns) = self.gap_elapsed_ns_plus_one.checked_sub(1) else {
            return Ok(None);
        };
        let elapsed_ns = i64::from(elapsed_ns);
        self.start
            .as_ns()
            .checked_sub(elapsed_ns)
            .map(SessionTime::from_ns)
            .map(Some)
            .ok_or(PreintegrationError::TimeOverflow)
    }

    pub(crate) fn duration_seconds(&self) -> Result<f32, PreintegrationError> {
        let ns = self
            .end
            .as_ns()
            .checked_sub(self.start.as_ns())
            .ok_or(PreintegrationError::TimeOverflow)?;
        if ns <= 0 {
            return Err(PreintegrationError::InvalidDuration);
        }
        let seconds = ns as f32 * 1.0e-9;
        if !seconds.is_finite() || seconds <= 0.0 {
            return Err(PreintegrationError::InvalidDuration);
        }
        Ok(seconds)
    }

    pub(crate) fn validate(&self) -> Result<(), PreintegrationError> {
        self.duration_seconds()?;
        if !vector_is_finite(&self.omega_ib_b)
            || !vector_is_finite(&self.specific_force_b)
            || !self
                .body_from_sensor
                .to_rotation_matrix()
                .matrix()
                .iter()
                .all(|value| value.is_finite())
            || !covariance_density_is_valid(&self.accel_sample_covariance.to_matrix())
            || !covariance_density_is_valid(&self.gyro_sample_covariance.to_matrix())
        {
            return Err(PreintegrationError::NonFinite);
        }
        Ok(())
    }

    /// Builds the explicit interval used to bridge a short, declared loss of
    /// IMU support. The last qualified vector is held. Constant angular-
    /// acceleration and jerk uncertainty
    /// is integrated separately by [`Preintegrator::push_gap`]; folding it
    /// into white-noise density would give the wrong power of elapsed time.
    pub(crate) fn bridge_after(
        previous: Self,
        target: SessionTime,
        model: GapModel,
    ) -> Result<Self, PreintegrationError> {
        model.validate()?;
        previous.validate()?;
        let gap_ns = target
            .as_ns()
            .checked_sub(previous.end.as_ns())
            .ok_or(PreintegrationError::TimeOverflow)?;
        if gap_ns <= 0 {
            return Err(PreintegrationError::InvalidDuration);
        }
        if gap_ns > model.maximum_gap_ns {
            return Err(PreintegrationError::GapTooLong);
        }
        Ok(Self {
            start: previous.end,
            end: target,
            omega_ib_b: previous.omega_ib_b,
            specific_force_b: previous.specific_force_b,
            degraded_input: previous.degraded_input,
            gap_elapsed_ns_plus_one: 1,
            body_from_sensor: previous.body_from_sensor,
            accel_sample_covariance: previous.accel_sample_covariance,
            gyro_sample_covariance: previous.gyro_sample_covariance,
            calibration_consider_start: previous.calibration_consider_start,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GapModel {
    pub(crate) maximum_gap_ns: i64,
    pub(crate) angular_acceleration_one_sigma: Vector3<f32>,
    pub(crate) jerk_one_sigma: Vector3<f32>,
}

impl GapModel {
    pub(crate) const V2_MINI_MAXIMUM_GAP_NS: i64 = 10_000_000;

    pub(crate) fn validate(&self) -> Result<(), PreintegrationError> {
        if self.maximum_gap_ns <= 0
            || self.maximum_gap_ns > Self::V2_MINI_MAXIMUM_GAP_NS
            || !vector_is_finite(&self.angular_acceleration_one_sigma)
            || !vector_is_finite(&self.jerk_one_sigma)
            || self
                .angular_acceleration_one_sigma
                .iter()
                .any(|value| *value < 0.0)
            || self.jerk_one_sigma.iter().any(|value| *value < 0.0)
        {
            return Err(PreintegrationError::InvalidGapModel);
        }
        Ok(())
    }

    fn derivative_covariance(&self) -> GapDerivativeCovariance {
        let mut covariance = GapDerivativeCovariance::zeros();
        covariance
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Matrix3::from_diagonal(
                &self.jerk_one_sigma.component_mul(&self.jerk_one_sigma),
            ));
        covariance
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&Matrix3::from_diagonal(
                &self
                    .angular_acceleration_one_sigma
                    .component_mul(&self.angular_acceleration_one_sigma),
            ));
        covariance
    }
}

/// Sensitivity to one held-vector gap's constant body jerk and angular
/// acceleration. The same six latent variables remain correlated when the
/// scheduler cuts the gap at navigation or GNSS epochs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PreintegratedGap {
    pub(crate) origin: SessionTime,
    pub(crate) derivative_covariance: GapDerivativeCovariance,
    pub(crate) jacobian: GapJacobian,
    pub(crate) active_at_end: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PreintegratedBatch {
    pub(crate) start: SessionTime,
    pub(crate) end: SessionTime,
    pub(crate) delta_rotation: UnitQuaternion<f32>,
    pub(crate) delta_velocity_b0: Vector3<f32>,
    pub(crate) delta_position_b0: Vector3<f32>,
    /// Error ordering is `(delta_theta, delta_v, delta_p)`.
    pub(crate) covariance: PreintCovariance,
    /// Contribution of the sample latent already active at the batch start.
    /// Its `J C J'` marginal is included in `covariance`; the explicit
    /// Jacobian lets the ESKF retain the cross term across scheduler cuts.
    pub(crate) leading_sample: Option<PreintegratedImuSample>,
    /// Newly encountered final sample when it remains active after the batch.
    pub(crate) trailing_sample: Option<PreintegratedImuSample>,
    pub(crate) gap: Option<PreintegratedGap>,
    /// Column ordering is `(delta_ba, delta_bg)`.
    pub(crate) bias_jacobian: BiasJacobian,
    pub(crate) linearization_accel_bias: Vector3<f32>,
    pub(crate) linearization_gyro_bias: Vector3<f32>,
    pub(crate) mean_omega_ib_b: Vector3<f32>,
    pub(crate) mean_specific_force_b: Vector3<f32>,
    pub(crate) sample_count: u16,
    pub(crate) degraded: bool,
    pub(crate) degraded_input: bool,
    pub(crate) correction_validity_norm: f32,
    pub(crate) calibration_consider_start: Option<u8>,
    pub(crate) mean_specific_force_consider_jacobian: Matrix3<f32>,
    pub(crate) mean_angular_rate_consider_jacobian: Matrix3<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PreintegratedImuSample {
    pub(crate) covariance: ImuSampleCovariance,
    pub(crate) jacobian: ImuSampleJacobian,
    pub(crate) active_at_end: bool,
}

impl PreintegratedBatch {
    pub(crate) fn duration_seconds(&self) -> Result<f32, PreintegrationError> {
        let ns = self
            .end
            .as_ns()
            .checked_sub(self.start.as_ns())
            .ok_or(PreintegrationError::TimeOverflow)?;
        if ns <= 0 {
            return Err(PreintegrationError::InvalidDuration);
        }
        Ok(ns as f32 * 1.0e-9)
    }

    /// First-order bias correction around the exact recorded linearization
    /// point.  A batch outside its declared correction radius must be rebuilt.
    pub(crate) fn corrected(
        &self,
        accel_bias: Vector3<f32>,
        gyro_bias: Vector3<f32>,
    ) -> Result<Self, PreintegrationError> {
        let delta_ba = accel_bias - self.linearization_accel_bias;
        let delta_bg = gyro_bias - self.linearization_gyro_bias;
        let combined_norm =
            crate::scalar_math::sqrt(delta_ba.norm_squared() + delta_bg.norm_squared());
        if !combined_norm.is_finite() || combined_norm > self.correction_validity_norm {
            return Err(PreintegrationError::BiasCorrectionOutsideValidity);
        }

        let mut delta_bias = nalgebra::SVector::<f32, BIAS_DIM>::zeros();
        delta_bias.fixed_rows_mut::<3>(0).copy_from(&delta_ba);
        delta_bias.fixed_rows_mut::<3>(3).copy_from(&delta_bg);
        let correction = self.bias_jacobian * delta_bias;

        let mut result = *self;
        result.delta_rotation *= so3_exp(correction.fixed_rows::<3>(0).into_owned());
        result.delta_velocity_b0 += correction.fixed_rows::<3>(3).into_owned();
        result.delta_position_b0 += correction.fixed_rows::<3>(6).into_owned();
        result.linearization_accel_bias = accel_bias;
        result.linearization_gyro_bias = gyro_bias;
        Ok(result)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Preintegrator {
    start: SessionTime,
    end: SessionTime,
    delta_rotation: UnitQuaternion<f32>,
    delta_velocity_b0: Vector3<f32>,
    delta_position_b0: Vector3<f32>,
    covariance: PreintCovariance,
    open_accel_sample_covariance: CompactCovariance3,
    open_gyro_sample_covariance: CompactCovariance3,
    open_sample_jacobian: ImuSampleJacobian,
    open_sample_active: bool,
    open_sample_from_before_batch: bool,
    open_sample_active_after_piece: bool,
    leading_accel_sample_covariance: CompactCovariance3,
    leading_gyro_sample_covariance: CompactCovariance3,
    leading_sample_jacobian: ImuSampleJacobian,
    leading_sample_present: bool,
    gap_origin: Option<SessionTime>,
    gap_derivative_covariance: GapDerivativeCovariance,
    gap_jacobian: GapJacobian,
    /// Absolute gap sensitivity at this batch's first gap epoch, propagated
    /// into the current preintegration error coordinates.  Subtracting it
    /// from the absolute end sensitivity preserves one correlated derivative
    /// latent when a scheduler boundary starts a batch part-way through a
    /// held-vector gap.
    gap_propagated_baseline: GapJacobian,
    gap_active_at_end: bool,
    bias_jacobian: BiasJacobian,
    linearization_accel_bias: Vector3<f32>,
    linearization_gyro_bias: Vector3<f32>,
    weighted_omega_sum: Vector3<f32>,
    weighted_force_sum: Vector3<f32>,
    total_seconds: f32,
    sample_count: u16,
    degraded: bool,
    degraded_input: bool,
    calibration_consider_start: Option<u8>,
    weighted_specific_force_consider_jacobian: Matrix3<f32>,
    weighted_angular_rate_consider_jacobian: Matrix3<f32>,
    last_interval: Option<ImuInterval>,
    correction_validity_norm: f32,
}

impl Preintegrator {
    /// Valid empty representation for caller-owned/static storage.
    pub(crate) const fn placeholder() -> Self {
        Self {
            start: SessionTime::ZERO,
            end: SessionTime::ZERO,
            delta_rotation: UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            delta_velocity_b0: Vector3::new(0.0, 0.0, 0.0),
            delta_position_b0: Vector3::new(0.0, 0.0, 0.0),
            covariance: PreintCovariance::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; PREINT_DIM],
            )),
            open_accel_sample_covariance: CompactCovariance3::ZERO,
            open_gyro_sample_covariance: CompactCovariance3::ZERO,
            open_sample_jacobian: ImuSampleJacobian::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; BIAS_DIM],
            )),
            open_sample_active: false,
            open_sample_from_before_batch: false,
            open_sample_active_after_piece: false,
            leading_accel_sample_covariance: CompactCovariance3::ZERO,
            leading_gyro_sample_covariance: CompactCovariance3::ZERO,
            leading_sample_jacobian: ImuSampleJacobian::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; BIAS_DIM],
            )),
            leading_sample_present: false,
            gap_origin: None,
            gap_derivative_covariance: GapDerivativeCovariance::from_array_storage(ArrayStorage(
                [[0.0; BIAS_DIM]; BIAS_DIM],
            )),
            gap_jacobian: GapJacobian::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; BIAS_DIM],
            )),
            gap_propagated_baseline: GapJacobian::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; BIAS_DIM],
            )),
            gap_active_at_end: false,
            bias_jacobian: BiasJacobian::from_array_storage(ArrayStorage(
                [[0.0; PREINT_DIM]; BIAS_DIM],
            )),
            linearization_accel_bias: Vector3::new(0.0, 0.0, 0.0),
            linearization_gyro_bias: Vector3::new(0.0, 0.0, 0.0),
            weighted_omega_sum: Vector3::new(0.0, 0.0, 0.0),
            weighted_force_sum: Vector3::new(0.0, 0.0, 0.0),
            total_seconds: 0.0,
            sample_count: 0,
            degraded: false,
            degraded_input: false,
            calibration_consider_start: None,
            weighted_specific_force_consider_jacobian: Matrix3::from_array_storage(ArrayStorage(
                [[0.0; 3]; 3],
            )),
            weighted_angular_rate_consider_jacobian: Matrix3::from_array_storage(ArrayStorage(
                [[0.0; 3]; 3],
            )),
            last_interval: None,
            correction_validity_norm: 0.0,
        }
    }

    /// Clears all accumulated evidence without constructing another large
    /// preintegrator value.
    pub(crate) fn reset(&mut self) {
        self.start = SessionTime::ZERO;
        self.end = SessionTime::ZERO;
        self.delta_rotation = UnitQuaternion::identity();
        self.delta_velocity_b0.fill(0.0);
        self.delta_position_b0.fill(0.0);
        self.covariance.fill(0.0);
        self.open_accel_sample_covariance = CompactCovariance3::ZERO;
        self.open_gyro_sample_covariance = CompactCovariance3::ZERO;
        self.open_sample_jacobian.fill(0.0);
        self.open_sample_active = false;
        self.open_sample_from_before_batch = false;
        self.open_sample_active_after_piece = false;
        self.leading_accel_sample_covariance = CompactCovariance3::ZERO;
        self.leading_gyro_sample_covariance = CompactCovariance3::ZERO;
        self.leading_sample_jacobian.fill(0.0);
        self.leading_sample_present = false;
        self.gap_origin = None;
        self.gap_derivative_covariance.fill(0.0);
        self.gap_jacobian.fill(0.0);
        self.gap_propagated_baseline.fill(0.0);
        self.gap_active_at_end = false;
        self.bias_jacobian.fill(0.0);
        self.linearization_accel_bias.fill(0.0);
        self.linearization_gyro_bias.fill(0.0);
        self.weighted_omega_sum.fill(0.0);
        self.weighted_force_sum.fill(0.0);
        self.total_seconds = 0.0;
        self.sample_count = 0;
        self.degraded = false;
        self.degraded_input = false;
        self.calibration_consider_start = None;
        self.weighted_specific_force_consider_jacobian.fill(0.0);
        self.weighted_angular_rate_consider_jacobian.fill(0.0);
        self.last_interval = None;
        self.correction_validity_norm = 0.0;
    }

    /// Initializes an existing empty slot. Invalid inputs leave it reset.
    pub(crate) fn initialize(
        &mut self,
        start: SessionTime,
        accel_bias: Vector3<f32>,
        gyro_bias: Vector3<f32>,
        correction_validity_norm: f32,
    ) -> Result<(), PreintegrationError> {
        self.reset();
        if !vector_is_finite(&accel_bias)
            || !vector_is_finite(&gyro_bias)
            || !correction_validity_norm.is_finite()
            || correction_validity_norm <= 0.0
        {
            return Err(PreintegrationError::NonFinite);
        }
        self.start = start;
        self.end = start;
        self.linearization_accel_bias = accel_bias;
        self.linearization_gyro_bias = gyro_bias;
        self.correction_validity_norm = correction_validity_norm;
        Ok(())
    }

    pub(crate) fn new(
        start: SessionTime,
        accel_bias: Vector3<f32>,
        gyro_bias: Vector3<f32>,
        correction_validity_norm: f32,
    ) -> Result<Self, PreintegrationError> {
        let mut result = Self::placeholder();
        result.initialize(start, accel_bias, gyro_bias, correction_validity_norm)?;
        Ok(result)
    }

    pub(crate) fn end(&self) -> SessionTime {
        self.end
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.sample_count == 0
    }

    pub(crate) fn push(
        &mut self,
        interval: ImuInterval,
        noise: ImuNoise,
    ) -> Result<(), PreintegrationError> {
        if interval.is_gap() {
            return Err(PreintegrationError::InvalidGapModel);
        }
        self.push_internal(interval, noise, None, false, false, false)
    }

    pub(crate) fn push_piece(
        &mut self,
        interval: ImuInterval,
        noise: ImuNoise,
        continues_previous_sample: bool,
        sample_active_after_piece: bool,
    ) -> Result<(), PreintegrationError> {
        if interval.is_gap() {
            return Err(PreintegrationError::InvalidGapModel);
        }
        self.push_internal(
            interval,
            noise,
            None,
            continues_previous_sample,
            false,
            sample_active_after_piece,
        )
    }

    /// Integrates one declared synthetic gap using the configured constant-
    /// derivative uncertainty model. Keeping this separate from [`Self::push`]
    /// prevents a synthetic interval from silently omitting its gap process
    /// covariance.
    pub(crate) fn push_gap(
        &mut self,
        interval: ImuInterval,
        noise: ImuNoise,
        model: GapModel,
        continues_after_piece: bool,
    ) -> Result<(), PreintegrationError> {
        model.validate()?;
        if !interval.is_gap() {
            return Err(PreintegrationError::InvalidGapModel);
        }
        let continues_previous_sample = interval.gap_elapsed_ns_plus_one > 1;
        self.push_internal(
            interval,
            noise,
            Some(model),
            continues_previous_sample,
            continues_after_piece,
            continues_after_piece,
        )
    }

    pub(crate) fn push_gap_piece(
        &mut self,
        interval: ImuInterval,
        noise: ImuNoise,
        model: GapModel,
        continues_previous_sample: bool,
        gap_active_after_piece: bool,
        sample_active_after_piece: bool,
    ) -> Result<(), PreintegrationError> {
        model.validate()?;
        if !interval.is_gap() {
            return Err(PreintegrationError::InvalidGapModel);
        }
        self.push_internal(
            interval,
            noise,
            Some(model),
            continues_previous_sample,
            gap_active_after_piece,
            sample_active_after_piece,
        )
    }

    fn push_internal(
        &mut self,
        interval: ImuInterval,
        noise: ImuNoise,
        gap_model: Option<GapModel>,
        continues_previous_sample: bool,
        gap_active_after_piece: bool,
        sample_active_after_piece: bool,
    ) -> Result<(), PreintegrationError> {
        interval.validate()?;
        if !noise.is_valid() {
            return Err(PreintegrationError::NonFinite);
        }
        if interval.start != self.end {
            return Err(PreintegrationError::Discontinuous {
                expected: self.end,
                received: interval.start,
            });
        }
        if self.sample_count == MAX_BATCH_SAMPLES {
            return Err(PreintegrationError::BatchCapacity);
        }
        if self.sample_count == 0 {
            self.calibration_consider_start = interval.calibration_consider_start;
        } else if self.calibration_consider_start != interval.calibration_consider_start {
            return Err(PreintegrationError::CalibrationModelChanged);
        }

        let gap_elapsed_seconds = interval.gap_elapsed_seconds();
        let mut initialize_gap_baseline = false;
        match (gap_model, interval.gap_origin()?) {
            (Some(model), Some(origin)) => {
                let derivative_covariance = model.derivative_covariance();
                if let Some(existing_origin) = self.gap_origin {
                    if existing_origin != origin
                        || self.gap_derivative_covariance != derivative_covariance
                    {
                        // One batch has bounded storage for one correlated gap
                        // latent. Never marginalize an unknown cross term as if
                        // a later gap were independent.
                        return Err(PreintegrationError::MultipleGapLatents);
                    }
                } else {
                    self.gap_origin = Some(origin);
                    self.gap_derivative_covariance = derivative_covariance;
                    initialize_gap_baseline = true;
                }
            }
            (None, None) => {}
            _ => return Err(PreintegrationError::InvalidGapModel),
        }

        let dt = interval.duration_seconds()?;
        let corrected_omega = interval.omega_ib_b - self.linearization_gyro_bias;
        let corrected_force = interval.specific_force_b - self.linearization_accel_bias;
        let theta = corrected_omega * dt;
        let d_rotation = so3_exp(theta);
        let rotation_before = self.delta_rotation.to_rotation_matrix().into_inner();
        let force_velocity = left_jacobian(theta) * corrected_force * dt;
        let force_position = second_rotation_integral(theta) * corrected_force * (dt * dt);

        self.delta_position_b0 += self.delta_velocity_b0 * dt + rotation_before * force_position;
        self.delta_velocity_b0 += rotation_before * force_velocity;

        self.propagate_error_covariance(
            dt,
            theta,
            corrected_force,
            rotation_before,
            noise,
            interval.accel_sample_covariance,
            interval.gyro_sample_covariance,
            continues_previous_sample,
            sample_active_after_piece,
            gap_elapsed_seconds,
            initialize_gap_baseline,
        );
        self.gap_active_at_end = gap_model.is_some() && gap_active_after_piece;
        self.delta_rotation *= d_rotation;
        self.delta_rotation.renormalize();
        self.weighted_omega_sum += interval.omega_ib_b * dt;
        self.weighted_force_sum += interval.specific_force_b * dt;
        self.total_seconds += dt;
        self.sample_count += 1;
        self.end = interval.end;
        self.degraded |= interval.is_gap();
        self.degraded_input |= interval.degraded_input;
        if interval.calibration_consider_start.is_some() {
            let body_from_sensor = interval.body_from_sensor.to_rotation_matrix().into_inner();
            self.weighted_specific_force_consider_jacobian +=
                -skew(&interval.specific_force_b) * body_from_sensor * dt;
            self.weighted_angular_rate_consider_jacobian +=
                -skew(&interval.omega_ib_b) * body_from_sensor * dt;
        }
        self.last_interval = Some(interval);
        Ok(())
    }

    /// Extends a short missing interval with the last qualified vector and a
    /// profile-bounded angular-acceleration/jerk covariance inflation.
    #[cfg(test)]
    pub(crate) fn bridge_gap(
        &mut self,
        target: SessionTime,
        noise: ImuNoise,
        model: GapModel,
    ) -> Result<(), PreintegrationError> {
        let previous = self
            .last_interval
            .ok_or(PreintegrationError::NoBridgeSource)?;
        self.push_gap_piece(
            ImuInterval::bridge_after(previous, target, model)?,
            noise,
            model,
            true,
            false,
            false,
        )
    }

    pub(crate) fn batch(&self) -> Result<PreintegratedBatch, PreintegrationError> {
        if self.sample_count == 0 || self.total_seconds <= 0.0 {
            return Err(PreintegrationError::EmptyBatch);
        }
        let gap = self.gap_origin.map(|origin| PreintegratedGap {
            origin,
            derivative_covariance: self.gap_derivative_covariance,
            jacobian: self.gap_jacobian,
            active_at_end: self.gap_active_at_end,
        });
        let mut covariance = gap.map_or(self.covariance, |gap| {
            self.covariance + gap.jacobian * gap.derivative_covariance * gap.jacobian.transpose()
        });
        let leading_covariance = imu_sample_covariance(
            self.leading_accel_sample_covariance,
            self.leading_gyro_sample_covariance,
        );
        let open_covariance = imu_sample_covariance(
            self.open_accel_sample_covariance,
            self.open_gyro_sample_covariance,
        );
        let mut leading_sample = self
            .leading_sample_present
            .then_some(PreintegratedImuSample {
                covariance: leading_covariance,
                jacobian: self.leading_sample_jacobian,
                active_at_end: false,
            });
        if self.leading_sample_present {
            covariance += self.leading_sample_jacobian
                * leading_covariance
                * self.leading_sample_jacobian.transpose();
        }
        let mut trailing_sample = None;
        if self.open_sample_active {
            covariance +=
                self.open_sample_jacobian * open_covariance * self.open_sample_jacobian.transpose();
            let contribution = PreintegratedImuSample {
                covariance: open_covariance,
                jacobian: self.open_sample_jacobian,
                active_at_end: self.open_sample_active_after_piece,
            };
            if self.open_sample_from_before_batch {
                leading_sample = Some(contribution);
            } else if self.open_sample_active_after_piece {
                trailing_sample = Some(contribution);
            }
        }
        if !covariance.iter().all(|value| value.is_finite()) {
            return Err(PreintegrationError::NonFinite);
        }
        Ok(PreintegratedBatch {
            start: self.start,
            end: self.end,
            delta_rotation: self.delta_rotation,
            delta_velocity_b0: self.delta_velocity_b0,
            delta_position_b0: self.delta_position_b0,
            covariance: (covariance + covariance.transpose()) * 0.5,
            leading_sample,
            trailing_sample,
            gap,
            bias_jacobian: self.bias_jacobian,
            linearization_accel_bias: self.linearization_accel_bias,
            linearization_gyro_bias: self.linearization_gyro_bias,
            mean_omega_ib_b: self.weighted_omega_sum / self.total_seconds,
            mean_specific_force_b: self.weighted_force_sum / self.total_seconds,
            sample_count: self.sample_count,
            degraded: self.degraded,
            degraded_input: self.degraded_input,
            correction_validity_norm: self.correction_validity_norm,
            calibration_consider_start: self.calibration_consider_start,
            mean_specific_force_consider_jacobian: self.weighted_specific_force_consider_jacobian
                / self.total_seconds,
            mean_angular_rate_consider_jacobian: self.weighted_angular_rate_consider_jacobian
                / self.total_seconds,
        })
    }

    fn propagate_error_covariance(
        &mut self,
        dt: f32,
        theta: Vector3<f32>,
        force_b: Vector3<f32>,
        rotation_before: Matrix3<f32>,
        noise: ImuNoise,
        accel_sample_covariance: CompactCovariance3,
        gyro_sample_covariance: CompactCovariance3,
        continues_previous_sample: bool,
        sample_active_after_piece: bool,
        gap_elapsed_seconds: Option<f32>,
        initialize_gap_baseline: bool,
    ) {
        let mut transition = PreintCovariance::identity();
        let d_rotation_inverse = so3_exp(theta).inverse().to_rotation_matrix().into_inner();
        transition
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&d_rotation_inverse);
        let integrated_force_velocity = left_jacobian(theta) * force_b * dt;
        let integrated_force_position = second_rotation_integral(theta) * force_b * (dt * dt);
        transition
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-rotation_before * skew(&integrated_force_velocity)));
        transition
            .fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-rotation_before * skew(&integrated_force_position)));
        transition
            .fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(Matrix3::identity() * dt));

        // Integrate instantaneous white-noise impulses independently, while
        // integrating a held sample's sensitivity before applying its one
        // covariance. A single interval-average noise map would incorrectly
        // give the white-acceleration position variance Q*dt^3/4 instead of
        // Q*dt^3/3 and lose the gyro/force cross terms.
        let mut density = ImuSampleCovariance::zeros();
        density
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&noise.accel_covariance_density);
        density
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&noise.gyro_covariance_density);
        let mut sample_response = ImuSampleJacobian::zeros();
        let mut white_noise_covariance = PreintCovariance::zeros();
        // Three-point Gauss-Legendre is exact for the complete zero-rate
        // white-noise model (degree four), with bounded work at nonzero rate.
        for (fraction, weight) in [
            (0.112_701_66_f32, 5.0 / 18.0),
            (0.5, 4.0 / 9.0),
            (0.887_298_35, 5.0 / 18.0),
        ] {
            let remaining = dt * (1.0 - fraction);
            let remaining_theta = theta * (1.0 - fraction);
            let rotation_at_noise =
                rotation_before * so3_exp(theta * fraction).to_rotation_matrix().into_inner();
            let mut response = ImuSampleJacobian::zeros();
            response
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(-so3_exp(-remaining_theta).to_rotation_matrix().into_inner()));
            response
                .fixed_view_mut::<3, 3>(3, 0)
                .copy_from(&(-rotation_at_noise));
            response
                .fixed_view_mut::<3, 3>(6, 0)
                .copy_from(&(-rotation_at_noise * remaining));
            response.fixed_view_mut::<3, 3>(3, 3).copy_from(
                &(rotation_at_noise
                    * skew(&(left_jacobian(remaining_theta) * force_b * remaining))),
            );
            response.fixed_view_mut::<3, 3>(6, 3).copy_from(
                &(rotation_at_noise
                    * skew(
                        &(second_rotation_integral(remaining_theta)
                            * force_b
                            * (remaining * remaining)),
                    )),
            );
            sample_response += response * (dt * weight);
            white_noise_covariance += response * density * response.transpose() * (dt * weight);
        }
        // These held-input integrals have simple exact closed forms.
        sample_response
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-right_jacobian(theta) * dt));
        sample_response
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-rotation_before * left_jacobian(theta) * dt));
        sample_response
            .fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-rotation_before * second_rotation_integral(theta) * (dt * dt)));

        self.covariance = transition * self.covariance * transition.transpose();
        if self.leading_sample_present {
            self.leading_sample_jacobian = transition * self.leading_sample_jacobian;
        }
        if self.open_sample_active {
            self.open_sample_jacobian = transition * self.open_sample_jacobian;
        }

        if !continues_previous_sample {
            if self.open_sample_active {
                if self.open_sample_from_before_batch {
                    if self.leading_sample_present {
                        // A batch can begin inside only one external sample.
                        self.covariance.fill(f32::NAN);
                        return;
                    }
                    self.leading_accel_sample_covariance = self.open_accel_sample_covariance;
                    self.leading_gyro_sample_covariance = self.open_gyro_sample_covariance;
                    self.leading_sample_jacobian = self.open_sample_jacobian;
                    self.leading_sample_present = true;
                } else {
                    self.covariance += self.open_sample_jacobian
                        * imu_sample_covariance(
                            self.open_accel_sample_covariance,
                            self.open_gyro_sample_covariance,
                        )
                        * self.open_sample_jacobian.transpose();
                }
            }
            self.open_accel_sample_covariance = accel_sample_covariance;
            self.open_gyro_sample_covariance = gyro_sample_covariance;
            self.open_sample_jacobian.fill(0.0);
            self.open_sample_active = true;
            self.open_sample_from_before_batch = false;
        } else if !self.open_sample_active {
            self.open_accel_sample_covariance = accel_sample_covariance;
            self.open_gyro_sample_covariance = gyro_sample_covariance;
            self.open_sample_jacobian.fill(0.0);
            self.open_sample_active = true;
            self.open_sample_from_before_batch = true;
        } else if self.open_accel_sample_covariance != accel_sample_covariance
            || self.open_gyro_sample_covariance != gyro_sample_covariance
        {
            self.covariance.fill(f32::NAN);
            return;
        }
        self.open_sample_jacobian += sample_response;
        self.open_sample_active_after_piece = sample_active_after_piece;

        self.covariance += white_noise_covariance;
        self.covariance = (self.covariance + self.covariance.transpose()) * 0.5;

        self.gap_propagated_baseline = transition * self.gap_propagated_baseline;
        if let Some(elapsed) = gap_elapsed_seconds {
            let omega_b = theta / dt;
            // The bounded gap model defines its six constant derivatives in
            // the body axes at the original gap epoch.  Recover that frozen
            // basis from the nominal rotation at this piece's start.
            let origin_rotation = rotation_before
                * so3_exp(omega_b * -elapsed)
                    .to_rotation_matrix()
                    .into_inner();
            if initialize_gap_baseline {
                self.gap_propagated_baseline = transition
                    * absolute_constant_derivative_gap_jacobian(
                        elapsed,
                        omega_b,
                        force_b,
                        origin_rotation,
                    );
            }
            let absolute_end = absolute_constant_derivative_gap_jacobian(
                elapsed + dt,
                omega_b,
                force_b,
                origin_rotation,
            );
            self.gap_jacobian = absolute_end - self.gap_propagated_baseline;
        } else {
            self.gap_jacobian = transition * self.gap_jacobian;
        }

        self.bias_jacobian = transition * self.bias_jacobian + sample_response;
    }
}

/// Absolute first-order sensitivity at elapsed time `T` to the one constant
/// jerk/angular-acceleration latent declared for a held-vector gap.
///
/// Derivative axes are frozen at the original gap epoch.  The polynomial
/// coefficients are the exact integrals of that declared constant-derivative
/// model: `T^2/2` into angle/velocity and `T^3/6` into position.  The held
/// force's angular-acceleration coupling integrates once more.  Expressing an
/// absolute sensitivity, then subtracting the propagated sensitivity at a
/// batch's start, makes the model invariant to internal navigation/GNSS cuts;
/// no split is treated as an independent draw.
fn absolute_constant_derivative_gap_jacobian(
    elapsed: f32,
    omega_b: Vector3<f32>,
    force_b: Vector3<f32>,
    origin_rotation: Matrix3<f32>,
) -> GapJacobian {
    let elapsed2 = elapsed * elapsed;
    let elapsed3 = elapsed2 * elapsed;
    let elapsed4 = elapsed2 * elapsed2;
    let angle_velocity_coefficient = 0.5 * elapsed2;
    let position_coefficient = elapsed3 / 6.0;
    let force_velocity_coefficient = elapsed3 / 6.0;
    let force_position_coefficient = elapsed4 / 24.0;
    let current_from_origin = so3_exp(omega_b * elapsed)
        .inverse()
        .to_rotation_matrix()
        .into_inner();
    let origin_force_skew = origin_rotation * skew(&force_b);
    let mut derivative_map = GapJacobian::zeros();
    derivative_map
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(origin_rotation * -angle_velocity_coefficient));
    derivative_map
        .fixed_view_mut::<3, 3>(6, 0)
        .copy_from(&(origin_rotation * -position_coefficient));
    derivative_map
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(current_from_origin * -angle_velocity_coefficient));
    derivative_map
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(origin_force_skew * force_velocity_coefficient));
    derivative_map
        .fixed_view_mut::<3, 3>(6, 3)
        .copy_from(&(origin_force_skew * force_position_coefficient));
    derivative_map
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PreintegrationError {
    NonFinite,
    InvalidDuration,
    TimeOverflow,
    Discontinuous {
        expected: SessionTime,
        received: SessionTime,
    },
    BatchCapacity,
    EmptyBatch,
    InvalidGapModel,
    InvalidSampleCovariance,
    MultipleGapLatents,
    GapTooLong,
    #[cfg(test)]
    NoBridgeSource,
    CalibrationModelChanged,
    BiasCorrectionOutsideValidity,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quiet_noise() -> ImuNoise {
        ImuNoise {
            accel_covariance_density: Matrix3::identity() * 1.0e-6,
            gyro_covariance_density: Matrix3::identity() * 1.0e-8,
        }
    }

    fn interval(
        start_ns: i64,
        end_ns: i64,
        omega: Vector3<f32>,
        force: Vector3<f32>,
    ) -> ImuInterval {
        ImuInterval {
            start: SessionTime::from_ns(start_ns),
            end: SessionTime::from_ns(end_ns),
            omega_ib_b: omega,
            specific_force_b: force,
            degraded_input: false,
            gap_elapsed_ns_plus_one: 0,
            body_from_sensor: UnitQuaternion::identity(),
            accel_sample_covariance: CompactCovariance3::ZERO,
            gyro_sample_covariance: CompactCovariance3::ZERO,
            calibration_consider_start: None,
        }
    }

    #[test]
    fn constant_force_integrates_velocity_and_position() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        preintegrator
            .push(
                interval(0, 10_000_000, Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0)),
                quiet_noise(),
            )
            .unwrap();
        let batch = preintegrator.batch().unwrap();
        assert!((batch.delta_velocity_b0.x - 0.02).abs() < 1.0e-6);
        assert!((batch.delta_position_b0.x - 0.0001).abs() < 1.0e-7);
    }

    #[test]
    fn constant_rotation_composes_on_so3() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        for index in 0..10 {
            preintegrator
                .push(
                    interval(
                        index * 1_000_000,
                        (index + 1) * 1_000_000,
                        Vector3::new(0.0, 0.0, 1.0),
                        Vector3::zeros(),
                    ),
                    quiet_noise(),
                )
                .unwrap();
        }
        let angle = preintegrator
            .batch()
            .unwrap()
            .delta_rotation
            .scaled_axis()
            .z;
        assert!((angle - 0.01).abs() < 1.0e-5);
    }

    #[test]
    fn discontinuity_is_never_silently_integrated() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        let error = preintegrator
            .push(
                interval(1, 1_000_001, Vector3::zeros(), Vector3::zeros()),
                quiet_noise(),
            )
            .unwrap_err();
        assert!(matches!(error, PreintegrationError::Discontinuous { .. }));
    }

    #[test]
    fn gap_bridge_is_bounded_and_degraded() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        preintegrator
            .push(
                interval(0, 1_000_000, Vector3::zeros(), Vector3::zeros()),
                quiet_noise(),
            )
            .unwrap();
        let model = GapModel {
            maximum_gap_ns: 10_000_000,
            angular_acceleration_one_sigma: Vector3::repeat(1.0),
            jerk_one_sigma: Vector3::repeat(10.0),
        };
        preintegrator
            .bridge_gap(SessionTime::from_ns(11_000_000), quiet_noise(), model)
            .unwrap();
        assert!(preintegrator.batch().unwrap().degraded);
        assert_eq!(
            preintegrator.bridge_gap(SessionTime::from_ns(21_000_001), quiet_noise(), model),
            Err(PreintegrationError::GapTooLong)
        );
    }

    #[test]
    fn synthetic_gap_cannot_omit_its_derivative_model() {
        let previous = interval(0, 1_000_000, Vector3::zeros(), Vector3::zeros());
        let model = GapModel {
            maximum_gap_ns: 10_000_000,
            angular_acceleration_one_sigma: Vector3::repeat(1.0),
            jerk_one_sigma: Vector3::repeat(1.0),
        };
        let gap =
            ImuInterval::bridge_after(previous, SessionTime::from_ns(11_000_000), model).unwrap();
        let mut preintegrator = Preintegrator::new(
            SessionTime::from_ns(1_000_000),
            Vector3::zeros(),
            Vector3::zeros(),
            1.0,
        )
        .unwrap();
        assert_eq!(
            preintegrator.push(gap, quiet_noise()),
            Err(PreintegrationError::InvalidGapModel)
        );
    }

    #[test]
    fn ten_millisecond_constant_derivative_gap_has_analytic_cross_covariance() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        let preceding = interval(0, 1_000_000, Vector3::zeros(), Vector3::new(0.0, 0.0, 10.0));
        let zero_noise = ImuNoise {
            accel_covariance_density: Matrix3::zeros(),
            gyro_covariance_density: Matrix3::zeros(),
        };
        preintegrator.push(preceding, zero_noise).unwrap();
        let model = GapModel {
            maximum_gap_ns: 10_000_000,
            angular_acceleration_one_sigma: Vector3::new(2.0, 0.0, 0.0),
            jerk_one_sigma: Vector3::new(3.0, 0.0, 0.0),
        };
        preintegrator
            .bridge_gap(SessionTime::from_ns(11_000_000), zero_noise, model)
            .unwrap();
        let covariance = preintegrator.batch().unwrap().covariance;
        let dt = 0.01_f32;
        let angular_variance = 4.0_f32;
        let jerk_variance = 9.0_f32;
        let force = 10.0_f32;
        let expected = [
            ((0, 0), angular_variance * dt.powi(4) / 4.0),
            ((3, 3), jerk_variance * dt.powi(4) / 4.0),
            ((6, 6), jerk_variance * dt.powi(6) / 36.0),
            ((3, 6), jerk_variance * dt.powi(5) / 12.0),
            ((0, 4), -force * angular_variance * dt.powi(5) / 12.0),
            ((0, 7), -force * angular_variance * dt.powi(6) / 48.0),
            (
                (4, 7),
                force * force * angular_variance * dt.powi(7) / 144.0,
            ),
        ];
        for ((row, column), analytic) in expected {
            let tolerance = analytic.abs() * 2.0e-5 + 1.0e-19;
            assert!(
                (covariance[(row, column)] - analytic).abs() <= tolerance,
                "entry ({row}, {column}) was {}, expected {analytic}",
                covariance[(row, column)]
            );
            assert_eq!(covariance[(row, column)], covariance[(column, row)]);
        }
    }

    #[test]
    fn held_derivative_gap_is_equivalent_when_split() {
        let zero_noise = ImuNoise {
            accel_covariance_density: Matrix3::zeros(),
            gyro_covariance_density: Matrix3::zeros(),
        };
        let model = GapModel {
            maximum_gap_ns: 10_000_000,
            angular_acceleration_one_sigma: Vector3::new(0.4, 0.5, 0.6),
            jerk_one_sigma: Vector3::new(2.0, 3.0, 4.0),
        };
        let source = interval(
            -1_000_000,
            0,
            Vector3::new(0.2, -0.1, 0.3),
            Vector3::new(1.0, 2.0, 9.0),
        );
        let full_gap =
            ImuInterval::bridge_after(source, SessionTime::from_ns(10_000_000), model).unwrap();

        let mut unsplit =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        unsplit
            .push_gap(full_gap, zero_noise, model, false)
            .unwrap();

        let mut prefix = full_gap;
        prefix.end = SessionTime::from_ns(4_000_000);
        let mut suffix = full_gap;
        suffix.start = prefix.end;
        suffix.gap_elapsed_ns_plus_one = 4_000_001;
        let mut split =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        split.push_gap(prefix, zero_noise, model, true).unwrap();
        split.push_gap(suffix, zero_noise, model, false).unwrap();

        let unsplit = unsplit.batch().unwrap();
        let split = split.batch().unwrap();
        assert!((unsplit.covariance - split.covariance).norm() < 1.0e-12);
        assert!((unsplit.gap.unwrap().jacobian - split.gap.unwrap().jacobian).norm() < 1.0e-9);
        assert!(!split.gap.unwrap().active_at_end);
    }

    #[test]
    fn covariance_remains_symmetric_and_nonnegative_on_diagonal() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        preintegrator
            .push(
                interval(
                    0,
                    2_000_000,
                    Vector3::new(0.2, -0.1, 0.3),
                    Vector3::new(1.0, 2.0, 9.0),
                ),
                quiet_noise(),
            )
            .unwrap();
        let covariance = preintegrator.batch().unwrap().covariance;
        assert!((covariance - covariance.transpose()).norm() < 1.0e-8);
        assert!((0..PREINT_DIM).all(|index| covariance[(index, index)] >= 0.0));
    }

    #[test]
    fn off_axis_imu_covariance_density_reaches_preintegrated_covariance() {
        let accel_covariance_density = Matrix3::new(
            4.0e-6, 1.0e-6, 0.0, //
            1.0e-6, 3.0e-6, 0.5e-6, //
            0.0, 0.5e-6, 2.0e-6,
        );
        let gyro_covariance_density = Matrix3::new(
            4.0e-8, -1.0e-8, 0.0, //
            -1.0e-8, 3.0e-8, 0.5e-8, //
            0.0, 0.5e-8, 2.0e-8,
        );
        let dt = 0.01_f32;
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        let sample = interval(0, 10_000_000, Vector3::zeros(), Vector3::zeros());
        let noise = ImuNoise {
            accel_covariance_density,
            gyro_covariance_density,
        };
        preintegrator.push(sample, noise).unwrap();
        let covariance = preintegrator.batch().unwrap().covariance;

        assert!((covariance[(3, 4)] - accel_covariance_density[(0, 1)] * dt).abs() < 1.0e-12);
        assert!((covariance[(0, 1)] - gyro_covariance_density[(0, 1)] * dt).abs() < 1.0e-14);
        assert_ne!(covariance[(3, 4)], 0.0);
        assert_ne!(covariance[(0, 1)], 0.0);
    }

    #[test]
    fn interval_average_sample_covariance_is_added_without_replacing_profile_density() {
        let profile_accel = Matrix3::new(
            4.0e-4, 1.0e-4, 0.0, //
            1.0e-4, 3.0e-4, 0.0, //
            0.0, 0.0, 2.0e-4,
        );
        let sample_accel = Matrix3::new(
            4.0e-3, -1.0e-3, 0.0, //
            -1.0e-3, 3.0e-3, 0.0, //
            0.0, 0.0, 2.0e-3,
        );
        let mut sample = interval(0, 10_000_000, Vector3::zeros(), Vector3::zeros());
        sample.accel_sample_covariance = CompactCovariance3::from_matrix(sample_accel).unwrap();
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        preintegrator
            .push(
                sample,
                ImuNoise {
                    accel_covariance_density: profile_accel,
                    gyro_covariance_density: Matrix3::zeros(),
                },
            )
            .unwrap();
        let covariance = preintegrator.batch().unwrap().covariance;
        let dt = 0.01_f32;
        let expected_increment = profile_accel * dt + sample_accel * (dt * dt);
        assert!((covariance[(3, 4)] - expected_increment[(0, 1)]).abs() < 1.0e-10);
        assert!((covariance[(3, 3)] - expected_increment[(0, 0)]).abs() < 1.0e-10);
        assert_ne!(covariance[(3, 4)], profile_accel[(0, 1)] * dt);
        assert_ne!(covariance[(3, 4)], sample_accel[(0, 1)] * dt * dt);
    }

    #[test]
    fn continuous_white_noise_has_analytic_position_and_gyro_force_moments() {
        let mut preintegrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        let dt = 0.2_f32;
        let accel_density = 2.0;
        let gyro_density = 0.3;
        let force = 4.0;
        preintegrator
            .push(
                interval(
                    0,
                    200_000_000,
                    Vector3::zeros(),
                    Vector3::new(0.0, force, 0.0),
                ),
                ImuNoise {
                    accel_covariance_density: Matrix3::identity() * accel_density,
                    gyro_covariance_density: Matrix3::identity() * gyro_density,
                },
            )
            .unwrap();
        let covariance = preintegrator.batch().unwrap().covariance;
        let gyro_force_density = force * force * gyro_density;
        for (row, column, expected) in [
            (0, 5, force * gyro_density * dt.powi(2) / 2.0),
            (0, 8, force * gyro_density * dt.powi(3) / 6.0),
            (
                5,
                5,
                accel_density * dt + gyro_force_density * dt.powi(3) / 3.0,
            ),
            (
                5,
                8,
                accel_density * dt.powi(2) / 2.0 + gyro_force_density * dt.powi(4) / 8.0,
            ),
            (
                8,
                8,
                accel_density * dt.powi(3) / 3.0 + gyro_force_density * dt.powi(5) / 20.0,
            ),
        ] {
            assert!((covariance[(row, column)] - expected).abs() < expected.abs() * 1.0e-6);
        }
    }

    #[test]
    fn rotating_held_sample_and_white_noise_are_invariant_to_support_splits() {
        let noise = ImuNoise {
            accel_covariance_density: Matrix3::identity() * 0.2,
            gyro_covariance_density: Matrix3::identity() * 0.03,
        };
        for held_sample in [false, true] {
            let mut sample = interval(
                0,
                200_000_000,
                Vector3::new(0.6, -0.3, 0.4),
                Vector3::new(2.0, -3.0, 8.0),
            );
            if held_sample {
                sample.accel_sample_covariance =
                    CompactCovariance3::from_matrix(Matrix3::identity() * 0.7).unwrap();
                sample.gyro_sample_covariance =
                    CompactCovariance3::from_matrix(Matrix3::identity() * 0.1).unwrap();
            }
            let mut unsplit =
                Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0)
                    .unwrap();
            let mut split = unsplit;
            unsplit.push_piece(sample, noise, false, false).unwrap();
            let mut first = sample;
            first.end = SessionTime::from_ns(70_000_000);
            let mut second = sample;
            second.start = first.end;
            split.push_piece(first, noise, false, true).unwrap();
            split.push_piece(second, noise, true, false).unwrap();
            let expected = unsplit.batch().unwrap();
            let actual = split.batch().unwrap();
            assert!((actual.delta_velocity_b0 - expected.delta_velocity_b0).norm() < 3.0e-7);
            assert!((actual.delta_position_b0 - expected.delta_position_b0).norm() < 3.0e-8);
            for row in 0..PREINT_DIM {
                for column in 0..PREINT_DIM {
                    let scale = crate::scalar_math::sqrt(
                        expected.covariance[(row, row)] * expected.covariance[(column, column)],
                    );
                    assert!(
                        (actual.covariance[(row, column)] - expected.covariance[(row, column)])
                            .abs()
                            <= scale * 2.0e-6,
                        "held={held_sample}, covariance[{row},{column}] differs: {} versus {}",
                        actual.covariance[(row, column)],
                        expected.covariance[(row, column)],
                    );
                }
            }
        }
    }

    #[test]
    fn finite_rotation_bias_jacobian_matches_reintegration() {
        let sample = interval(
            0,
            200_000_000,
            Vector3::new(0.6, -0.3, 0.4),
            Vector3::new(2.0, -3.0, 8.0),
        );
        let integrate = |accel_bias: Vector3<f64>, gyro_bias: Vector3<f64>| {
            let dt = sample.duration_seconds().unwrap() as f64;
            let omega = sample.omega_ib_b.cast::<f64>() - gyro_bias;
            let force = sample.specific_force_b.cast::<f64>() - accel_bias;
            let mut velocity = Vector3::<f64>::zeros();
            let mut position = Vector3::<f64>::zeros();
            const STEPS: usize = 256;
            for index in 0..=STEPS {
                let time = dt * index as f64 / STEPS as f64;
                let weight = if index == 0 || index == STEPS {
                    1.0
                } else if index % 2 == 0 {
                    2.0
                } else {
                    4.0
                };
                let rotated_force = UnitQuaternion::from_scaled_axis(omega * time) * force;
                velocity += rotated_force * weight;
                position += rotated_force * (weight * (dt - time));
            }
            (
                UnitQuaternion::from_scaled_axis(omega * dt),
                velocity * (dt / (3.0 * STEPS as f64)),
                position * (dt / (3.0 * STEPS as f64)),
            )
        };
        let mut integrator =
            Preintegrator::new(SessionTime::ZERO, Vector3::zeros(), Vector3::zeros(), 1.0).unwrap();
        integrator.push(sample, quiet_noise()).unwrap();
        let nominal = integrator.batch().unwrap();
        let nominal_rotation = integrate(Vector3::zeros(), Vector3::zeros()).0;
        let epsilon = 1.0e-5;
        for column in 0..BIAS_DIM {
            let mut accel_delta = Vector3::zeros();
            let mut gyro_delta = Vector3::zeros();
            if column < 3 {
                accel_delta[column] = epsilon;
            } else {
                gyro_delta[column - 3] = epsilon;
            }
            let plus = integrate(accel_delta, gyro_delta);
            let minus = integrate(-accel_delta, -gyro_delta);
            let mut difference = nalgebra::SVector::<f64, PREINT_DIM>::zeros();
            difference
                .fixed_rows_mut::<3>(0)
                .copy_from(&(nominal_rotation.inverse() * plus.0).scaled_axis());
            let rotation_minus = (nominal_rotation.inverse() * minus.0).scaled_axis();
            for axis in 0..3 {
                difference[axis] -= rotation_minus[axis];
            }
            difference
                .fixed_rows_mut::<3>(3)
                .copy_from(&(plus.1 - minus.1));
            difference
                .fixed_rows_mut::<3>(6)
                .copy_from(&(plus.2 - minus.2));
            difference /= 2.0 * epsilon;
            assert!(
                (difference.cast::<f32>() - nominal.bias_jacobian.column(column)).norm() < 2.0e-7,
                "bias column {column}: {difference:?} versus {:?}",
                nominal.bias_jacobian.column(column).into_owned(),
            );
        }
    }
}
