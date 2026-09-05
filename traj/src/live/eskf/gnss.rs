//! GNSS observation contracts, validation, and measurement linearization.

use super::{
    ConsiderJacobian3, Eskf, EskfError, GapNavCrossCovariance, MAX_CONSIDER,
    MeasurementConsiderJacobian, MeasurementMatrix,
    covariance::{active_principal_block_is_psd, matrix3_is_psd},
    update::{LinearMeasurement, cholesky_active},
};
use crate::{
    live::{
        preintegration::{BIAS_DIM, ImuSampleCovariance},
        state::{ATT, GYRO_BIAS, MechanizationContext, POS, VEL, skew},
    },
    quality::{GnssState, TimingQuality},
    time::SessionTime,
};
use nalgebra::{Matrix3, Vector3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct NisGate {
    pub(crate) soft_3d: f32,
    pub(crate) hard_3d: f32,
    pub(crate) soft_6d: f32,
    pub(crate) hard_6d: f32,
    pub(crate) maximum_covariance_inflation: f32,
}

impl NisGate {
    pub(crate) fn validate(&self) -> Result<(), EskfError> {
        let values = [
            self.soft_3d,
            self.hard_3d,
            self.soft_6d,
            self.hard_6d,
            self.maximum_covariance_inflation,
        ];
        if !values.iter().all(|value| value.is_finite() && *value > 0.0)
            || self.hard_3d < self.soft_3d
            || self.hard_6d < self.soft_6d
            || self.maximum_covariance_inflation < 1.0
        {
            return Err(EskfError::InvalidConfiguration);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SharedMeasurementJacobians {
    pub(crate) position: ConsiderJacobian3,
    pub(crate) velocity: ConsiderJacobian3,
}

impl Default for SharedMeasurementJacobians {
    fn default() -> Self {
        Self {
            position: ConsiderJacobian3::zeros(),
            velocity: ConsiderJacobian3::zeros(),
        }
    }
}

/// A receiver-solution observation already transformed into the active ENU
/// anchor. Position and velocity retain independent epochs at the public seam;
/// this type is used only after the scheduler has propagated to `time`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GnssObservation {
    pub(crate) time: SessionTime,
    pub(crate) position_n: Option<Vector3<f32>>,
    pub(crate) velocity_n: Option<Vector3<f32>>,
    pub(crate) position_covariance_n: Matrix3<f32>,
    pub(crate) velocity_covariance_n: Matrix3<f32>,
    pub(crate) position_velocity_cross_n: Option<Matrix3<f32>>,
    pub(crate) imu_to_antenna_b: Vector3<f32>,
    /// Calibrated body inertial rate at this observation's effective epoch.
    /// It must not be copied from the later packet-arrival epoch.
    pub(crate) omega_ib_b: Vector3<f32>,
    /// Calibrated body specific force at this observation's effective epoch.
    /// It supplies the velocity measurement's temporal sensitivity.
    pub(crate) specific_force_b: Vector3<f32>,
    /// Qualified derivative of body-relative-Earth angular rate at the actual
    /// measurement epoch, in body axes. Required for velocity timestamp
    /// sensitivity when the antenna lever arm is non-zero.
    pub(crate) angular_acceleration_eb_b: Option<Vector3<f32>>,
    /// Independent covariance of the qualified angular-acceleration estimate.
    pub(crate) angular_acceleration_covariance_b: Matrix3<f32>,
    /// First coordinate of the active affine clock `(offset_s, drift)` block.
    pub(crate) clock_consider_start: Option<u8>,
    /// Epoch at which the affine clock offset/drift covariance is expressed.
    pub(crate) clock_reference_time: SessionTime,
    /// First coordinate of the body-frame three-axis antenna lever-arm block.
    pub(crate) lever_arm_consider_start: Option<u8>,
    /// Sample-specific position timestamp uncertainty. Shared clock-fit
    /// uncertainty belongs in the clock consider block, never here.
    pub(crate) position_independent_timing_sigma_s: f32,
    /// Sample-specific velocity timestamp uncertainty. Shared clock-fit
    /// uncertainty belongs in the clock consider block, never here.
    pub(crate) velocity_independent_timing_sigma_s: f32,
    /// Additional profile-specific shared sensitivities. Clock and lever-arm
    /// terms above are added to these matrices.
    pub(crate) shared_jacobians: SharedMeasurementJacobians,
    pub(crate) receiver_healthy: bool,
    /// Receiver state to publish only if this scheduled member actually
    /// contributes through a fused or robustly downweighted update.
    pub(crate) quality_state: GnssState,
    /// Timing provenance paired with `quality_state`. Queueing or rejecting
    /// the member must never make this candidate visible as estimate quality.
    pub(crate) quality_timing: TimingQuality,
}

impl GnssObservation {
    pub(crate) fn validate(&self) -> Result<(), EskfError> {
        validate_gnss(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GnssUpdateOutcome {
    pub(crate) position: Option<UpdateDecision>,
    pub(crate) velocity: Option<UpdateDecision>,
    pub(crate) joint: Option<UpdateDecision>,
}

impl GnssUpdateOutcome {
    fn empty() -> Self {
        Self {
            position: None,
            velocity: None,
            joint: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum UpdateDecision {
    Fused {
        nis: f32,
    },
    Downweighted {
        nis: f32,
        inflation: f32,
    },
    RejectedInnovation {
        nis: f32,
    },
    RejectedHealth,
    /// The observation is semantically valid, but its rigid-body timing
    /// model requires support-aligned inertial kinematics that are not
    /// available. This is an input disposition, not estimator divergence.
    RejectedInsufficientKinematics,
}

impl Eskf {
    #[cfg(test)]
    pub(crate) fn update_gnss(
        &mut self,
        observation: &GnssObservation,
        context: &MechanizationContext,
        gate: NisGate,
    ) -> Result<GnssUpdateOutcome, EskfError> {
        self.update_gnss_with_imu_sample(observation, context, gate, None, None)
    }

    pub(crate) fn update_gnss_with_imu_sample(
        &mut self,
        observation: &GnssObservation,
        context: &MechanizationContext,
        gate: NisGate,
        sample_covariance: Option<&ImuSampleCovariance>,
        mut sample_cross: Option<&mut GapNavCrossCovariance>,
    ) -> Result<GnssUpdateOutcome, EskfError> {
        if sample_covariance.is_some() != sample_cross.is_some() {
            return Err(EskfError::ImuSampleLatentMismatch);
        }
        if sample_covariance
            .is_some_and(|covariance| !active_principal_block_is_psd(covariance, BIAS_DIM))
            || sample_cross
                .as_deref()
                .is_some_and(|cross| cross.iter().any(|value| !value.is_finite()))
        {
            return Err(EskfError::InvalidCovariance);
        }
        gate.validate()?;
        if observation.time != self.state.time {
            return Err(EskfError::TimeMismatch);
        }
        validate_gnss(observation)?;
        validate_gnss_consider_blocks(observation, self.active_consider)?;
        let mut outcome = GnssUpdateOutcome::empty();
        if !observation.receiver_healthy {
            if observation.position_n.is_some() {
                outcome.position = Some(UpdateDecision::RejectedHealth);
            }
            if observation.velocity_n.is_some() {
                outcome.velocity = Some(UpdateDecision::RejectedHealth);
            }
            return Ok(outcome);
        }

        if let (Some(position), Some(velocity), Some(cross)) = (
            observation.position_n,
            observation.velocity_n,
            observation.position_velocity_cross_n,
        ) {
            let linearization = self.gnss_linearization(
                observation,
                context,
                Some(position),
                Some(velocity),
                Some(cross),
            )?;
            outcome.joint = Some(self.linear_update(
                linearization,
                gate.soft_6d,
                gate.hard_6d,
                gate.maximum_covariance_inflation,
                sample_covariance,
                sample_cross.as_deref_mut(),
            )?);
            return Ok(outcome);
        }

        if let Some(position) = observation.position_n {
            let linearization =
                self.gnss_linearization(observation, context, Some(position), None, None)?;
            outcome.position = Some(self.linear_update(
                linearization,
                gate.soft_3d,
                gate.hard_3d,
                gate.maximum_covariance_inflation,
                sample_covariance,
                sample_cross.as_deref_mut(),
            )?);
        }
        if let Some(velocity) = observation.velocity_n {
            let linearization =
                self.gnss_linearization(observation, context, None, Some(velocity), None)?;
            outcome.velocity = Some(self.linear_update(
                linearization,
                gate.soft_3d,
                gate.hard_3d,
                gate.maximum_covariance_inflation,
                sample_covariance,
                sample_cross.as_deref_mut(),
            )?);
        }
        Ok(outcome)
    }

    pub(super) fn gnss_linearization(
        &self,
        observation: &GnssObservation,
        context: &MechanizationContext,
        position: Option<Vector3<f32>>,
        velocity: Option<Vector3<f32>>,
        cross: Option<Matrix3<f32>>,
    ) -> Result<LinearMeasurement, EskfError> {
        let rotation = self
            .state
            .orientation_n_from_b
            .to_rotation_matrix()
            .into_inner();
        let lever = observation.imu_to_antenna_b;
        let velocity_epoch_is_uncertain = velocity.is_some()
            && (observation.velocity_independent_timing_sigma_s > 0.0
                || observation.clock_consider_start.is_some());
        let lever_needs_angular_acceleration =
            velocity_epoch_is_uncertain && lever.norm_squared() > 0.0;
        let angular_acceleration = match (
            lever_needs_angular_acceleration,
            observation.angular_acceleration_eb_b,
        ) {
            (true, Some(value)) => value,
            (true, None) => return Err(EskfError::MissingAngularAccelerationForTiming),
            (false, value) => value.unwrap_or_else(Vector3::zeros),
        };
        let omega_earth_b = rotation.transpose() * context.earth_rate_n;
        let omega_eb_b = observation.omega_ib_b - self.state.gyro_bias_b - omega_earth_b;
        let rotational_velocity_b = omega_eb_b.cross(&lever);
        let predicted_position = self.state.position_n + rotation * lever;
        let predicted_velocity = self.state.velocity_n + rotation * rotational_velocity_b;
        // Angular acceleration is not observable from a single receiver
        // solution. The live profile therefore uses the explicitly recorded
        // measurement-epoch force and a constant-angular-rate rigid-body term.
        let corrected_specific_force = observation.specific_force_b - self.state.accel_bias_b;
        let body_acceleration = rotation * corrected_specific_force
            + context.gravity_at(&self.state.position_n)
            - 2.0 * context.earth_rate_n.cross(&self.state.velocity_n);
        // For v_a = v + R(omega_eb^b x r), differentiation in the fixed ENU
        // frame gives R[omega]x([omega]x r) + R(alpha_eb^b x r). The latter is
        // the tangential term and must use body-relative-Earth alpha at this
        // observation's actual epoch (not packet arrival).
        let tangential_acceleration = rotation * angular_acceleration.cross(&lever);
        let antenna_acceleration = body_acceleration
            + rotation * omega_eb_b.cross(&omega_eb_b.cross(&lever))
            + tangential_acceleration;
        let tangential_jacobian = -rotation * skew(&lever);
        let tangential_acceleration_covariance = tangential_jacobian
            * observation.angular_acceleration_covariance_b
            * tangential_jacobian.transpose();
        let clock_elapsed_seconds = if observation.clock_consider_start.is_some() {
            observation
                .time
                .checked_duration_since(observation.clock_reference_time)
                .ok_or(EskfError::InvalidMeasurement)?
                .as_seconds_f64() as f32
        } else {
            0.0
        };

        let mut result = LinearMeasurement::zeros();
        let mut row = 0;
        if let Some(measured_position) = position {
            let measurement_row = row;
            result
                .residual
                .fixed_rows_mut::<3>(row)
                .copy_from(&(measured_position - predicted_position));
            result
                .h_nav
                .fixed_view_mut::<3, 3>(row, POS)
                .copy_from(&Matrix3::identity());
            result
                .h_nav
                .fixed_view_mut::<3, 3>(row, ATT)
                .copy_from(&(-rotation * skew(&lever)));
            result
                .h_consider
                .fixed_view_mut::<3, MAX_CONSIDER>(row, 0)
                .copy_from(&observation.shared_jacobians.position);
            result
                .noise
                .fixed_view_mut::<3, 3>(row, row)
                .copy_from(&observation.position_covariance_n);
            if let Some(start) = observation.lever_arm_consider_start {
                add_consider_block(
                    &mut result.h_consider,
                    measurement_row,
                    usize::from(start),
                    &rotation,
                );
            }
            if let Some(start) = observation.clock_consider_start {
                add_clock_consider_sensitivity(
                    &mut result.h_consider,
                    measurement_row,
                    usize::from(start),
                    clock_elapsed_seconds,
                    &predicted_velocity,
                );
            }
            add_timing_variance(
                &mut result.noise,
                measurement_row,
                observation.position_independent_timing_sigma_s,
                &predicted_velocity,
            );
            row += 3;
        }
        if let Some(measured_velocity) = velocity {
            let measurement_row = row;
            result
                .residual
                .fixed_rows_mut::<3>(row)
                .copy_from(&(measured_velocity - predicted_velocity));
            result
                .h_nav
                .fixed_view_mut::<3, 3>(row, VEL)
                .copy_from(&Matrix3::identity());
            let attitude_jacobian = -rotation * skew(&rotational_velocity_b)
                + rotation * skew(&lever) * skew(&omega_earth_b);
            result
                .h_nav
                .fixed_view_mut::<3, 3>(row, ATT)
                .copy_from(&attitude_jacobian);
            result
                .h_nav
                .fixed_view_mut::<3, 3>(row, GYRO_BIAS)
                .copy_from(&(rotation * skew(&lever)));
            // The held sample latent is the recorded-rate error
            // `(observed - true)`. Therefore the true rigid-body velocity has
            // sensitivity `+R [r]x` to that latent, matching the propagation
            // convention used by the preintegrator.
            result
                .h_sample
                .fixed_view_mut::<3, 3>(row, 3)
                .copy_from(&(rotation * skew(&lever)));
            result
                .h_consider
                .fixed_view_mut::<3, MAX_CONSIDER>(row, 0)
                .copy_from(&observation.shared_jacobians.velocity);
            result
                .noise
                .fixed_view_mut::<3, 3>(row, row)
                .copy_from(&observation.velocity_covariance_n);
            if let Some(start) = observation.lever_arm_consider_start {
                let lever_sensitivity = rotation * skew(&omega_eb_b);
                add_consider_block(
                    &mut result.h_consider,
                    measurement_row,
                    usize::from(start),
                    &lever_sensitivity,
                );
            }
            if let Some(start) = observation.clock_consider_start {
                add_clock_consider_sensitivity(
                    &mut result.h_consider,
                    measurement_row,
                    usize::from(start),
                    clock_elapsed_seconds,
                    &antenna_acceleration,
                );
                let start = usize::from(start);
                let clock_variance = self.consider_covariance[(start, start)]
                    + 2.0 * clock_elapsed_seconds * self.consider_covariance[(start, start + 1)]
                    + clock_elapsed_seconds
                        * clock_elapsed_seconds
                        * self.consider_covariance[(start + 1, start + 1)];
                add_timing_sensitivity_covariance(
                    &mut result.noise,
                    measurement_row,
                    clock_variance.max(0.0),
                    &tangential_acceleration_covariance,
                );
            }
            add_timing_variance(
                &mut result.noise,
                measurement_row,
                observation.velocity_independent_timing_sigma_s,
                &antenna_acceleration,
            );
            add_timing_sensitivity_covariance(
                &mut result.noise,
                measurement_row,
                observation.velocity_independent_timing_sigma_s
                    * observation.velocity_independent_timing_sigma_s,
                &tangential_acceleration_covariance,
            );
            row += 3;
        }
        if let Some(position_velocity_cross) = cross {
            result
                .noise
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&position_velocity_cross);
            result
                .noise
                .fixed_view_mut::<3, 3>(3, 0)
                .copy_from(&position_velocity_cross.transpose());
        }
        result.dimension = row;
        if !result
            .residual
            .iter()
            .chain(result.h_nav.iter())
            .chain(result.h_consider.iter())
            .chain(result.h_sample.iter())
            .chain(result.noise.iter())
            .all(|value| value.is_finite())
        {
            return Err(EskfError::InvalidMeasurement);
        }
        Ok(result)
    }
}

fn add_consider_block(
    target: &mut MeasurementConsiderJacobian,
    row: usize,
    column: usize,
    value: &Matrix3<f32>,
) {
    for output in 0..3 {
        for coordinate in 0..3 {
            target[(row + output, column + coordinate)] += value[(output, coordinate)];
        }
    }
}

fn add_clock_consider_sensitivity(
    target: &mut MeasurementConsiderJacobian,
    row: usize,
    column: usize,
    elapsed_seconds: f32,
    temporal_sensitivity: &Vector3<f32>,
) {
    for axis in 0..3 {
        // Embedded clock coordinate zero is an offset in seconds; coordinate
        // one is fractional drift at `clock_reference_time`.
        target[(row + axis, column)] += temporal_sensitivity[axis];
        target[(row + axis, column + 1)] += temporal_sensitivity[axis] * elapsed_seconds;
    }
}

fn add_timing_variance(
    target: &mut MeasurementMatrix,
    row: usize,
    sigma_seconds: f32,
    temporal_sensitivity: &Vector3<f32>,
) {
    let variance_seconds = sigma_seconds * sigma_seconds;
    for output in 0..3 {
        for other_output in 0..3 {
            target[(row + output, row + other_output)] += temporal_sensitivity[output]
                * temporal_sensitivity[other_output]
                * variance_seconds;
        }
    }
}

fn add_timing_sensitivity_covariance(
    target: &mut MeasurementMatrix,
    row: usize,
    timing_variance_seconds2: f32,
    sensitivity_covariance: &Matrix3<f32>,
) {
    for output in 0..3 {
        for other_output in 0..3 {
            target[(row + output, row + other_output)] +=
                sensitivity_covariance[(output, other_output)] * timing_variance_seconds2;
        }
    }
}

fn validate_gnss(observation: &GnssObservation) -> Result<(), EskfError> {
    if observation.position_n.is_none() && observation.velocity_n.is_none() {
        return Err(EskfError::InvalidMeasurement);
    }
    let finite_vector = |value: &Vector3<f32>| value.iter().all(|entry| entry.is_finite());
    if observation
        .position_n
        .as_ref()
        .is_some_and(|value| !finite_vector(value))
        || observation
            .velocity_n
            .as_ref()
            .is_some_and(|value| !finite_vector(value))
        || !finite_vector(&observation.imu_to_antenna_b)
        || !finite_vector(&observation.omega_ib_b)
        || !finite_vector(&observation.specific_force_b)
        || observation
            .angular_acceleration_eb_b
            .as_ref()
            .is_some_and(|value| !finite_vector(value))
        || !observation.position_independent_timing_sigma_s.is_finite()
        || observation.position_independent_timing_sigma_s < 0.0
        || !observation.velocity_independent_timing_sigma_s.is_finite()
        || observation.velocity_independent_timing_sigma_s < 0.0
        || !observation
            .position_covariance_n
            .iter()
            .all(|value| value.is_finite())
        || !observation
            .velocity_covariance_n
            .iter()
            .all(|value| value.is_finite())
        || !observation
            .angular_acceleration_covariance_b
            .iter()
            .all(|value| value.is_finite())
        || !observation
            .shared_jacobians
            .position
            .iter()
            .chain(observation.shared_jacobians.velocity.iter())
            .all(|value| value.is_finite())
    {
        return Err(EskfError::InvalidMeasurement);
    }
    if let Some(start) = observation.clock_consider_start {
        if usize::from(start).saturating_add(2) > MAX_CONSIDER
            || observation
                .time
                .checked_duration_since(observation.clock_reference_time)
                .is_none()
        {
            return Err(EskfError::InvalidConsiderBlock);
        }
    }
    if observation
        .lever_arm_consider_start
        .is_some_and(|start| usize::from(start).saturating_add(3) > MAX_CONSIDER)
    {
        return Err(EskfError::InvalidConsiderBlock);
    }
    if observation.position_n.is_some() && !matrix3_is_psd(&observation.position_covariance_n) {
        return Err(EskfError::InvalidMeasurementCovariance);
    }
    if observation.velocity_n.is_some() && !matrix3_is_psd(&observation.velocity_covariance_n) {
        return Err(EskfError::InvalidMeasurementCovariance);
    }
    if !matrix3_is_psd(&observation.angular_acceleration_covariance_b)
        || (observation.angular_acceleration_eb_b.is_none()
            && observation.angular_acceleration_covariance_b != Matrix3::zeros())
    {
        return Err(EskfError::InvalidMeasurementCovariance);
    }
    if let Some(cross) = observation.position_velocity_cross_n {
        if !cross.iter().all(|value| value.is_finite()) {
            return Err(EskfError::InvalidMeasurementCovariance);
        }
        let mut joint = MeasurementMatrix::zeros();
        joint
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&observation.position_covariance_n);
        joint
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&observation.velocity_covariance_n);
        joint.fixed_view_mut::<3, 3>(0, 3).copy_from(&cross);
        joint
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&cross.transpose());
        if cholesky_active(&joint, 6).is_none() {
            return Err(EskfError::InvalidMeasurementCovariance);
        }
    }
    Ok(())
}

fn validate_gnss_consider_blocks(
    observation: &GnssObservation,
    active_consider: usize,
) -> Result<(), EskfError> {
    if active_consider > MAX_CONSIDER
        || observation
            .clock_consider_start
            .is_some_and(|start| usize::from(start).saturating_add(2) > active_consider)
        || observation
            .lever_arm_consider_start
            .is_some_and(|start| usize::from(start).saturating_add(3) > active_consider)
    {
        return Err(EskfError::InvalidConsiderBlock);
    }
    for column in active_consider..MAX_CONSIDER {
        for row in 0..3 {
            if observation.shared_jacobians.position[(row, column)] != 0.0
                || observation.shared_jacobians.velocity[(row, column)] != 0.0
            {
                return Err(EskfError::InvalidConsiderBlock);
            }
        }
    }
    Ok(())
}
