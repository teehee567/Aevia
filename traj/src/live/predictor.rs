//! Low-latency present-time output predictor.
//!
//! This is deliberately not a second estimator: it propagates only a nominal
//! state, receives bounded complementary corrections from the delayed ESKF,
//! and never supplies finalized events or covariance-grade claims.

use nalgebra::Vector3;

use super::{
    eskf::{EskfError, propagate_nominal},
    preintegration::PreintegratedBatch,
    reanchor::{ReanchorError, ReanchorTransform},
    state::{ATT, MechanizationContext, NavState, NavVector, POS, VEL, so3_log},
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PredictorConfig {
    pub(crate) position_time_constant_s: f32,
    pub(crate) velocity_time_constant_s: f32,
    pub(crate) attitude_time_constant_s: f32,
    pub(crate) position_reset_threshold_m: f32,
    pub(crate) velocity_reset_threshold_mps: f32,
    pub(crate) attitude_reset_threshold_rad: f32,
}

impl PredictorConfig {
    pub(crate) fn validate(&self) -> Result<(), PredictorError> {
        let positive = [
            self.position_time_constant_s,
            self.velocity_time_constant_s,
            self.attitude_time_constant_s,
            self.position_reset_threshold_m,
            self.velocity_reset_threshold_mps,
            self.attitude_reset_threshold_rad,
        ];
        if positive
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(PredictorError::InvalidConfiguration);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct PredictorTrackingError {
    pub(crate) position_m: f32,
    pub(crate) velocity_mps: f32,
    pub(crate) attitude_rad: f32,
    pub(crate) hard_resets: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct PendingCorrection {
    position_n: Vector3<f32>,
    velocity_n: Vector3<f32>,
    attitude_b: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct OutputPredictor {
    pub(crate) state: NavState,
    config: PredictorConfig,
    pending: PendingCorrection,
    pub(crate) tracking_error: PredictorTrackingError,
    pub(crate) degraded_input: bool,
}

impl OutputPredictor {
    pub(crate) const fn placeholder() -> Self {
        Self {
            state: NavState::placeholder(),
            config: PredictorConfig {
                position_time_constant_s: 0.0,
                velocity_time_constant_s: 0.0,
                attitude_time_constant_s: 0.0,
                position_reset_threshold_m: 0.0,
                velocity_reset_threshold_mps: 0.0,
                attitude_reset_threshold_rad: 0.0,
            },
            pending: PendingCorrection {
                position_n: Vector3::new(0.0, 0.0, 0.0),
                velocity_n: Vector3::new(0.0, 0.0, 0.0),
                attitude_b: Vector3::new(0.0, 0.0, 0.0),
            },
            degraded_input: false,
            tracking_error: PredictorTrackingError {
                position_m: 0.0,
                velocity_mps: 0.0,
                attitude_rad: 0.0,
                hard_resets: 0,
            },
        }
    }

    pub(crate) fn reset(&mut self) {
        self.state = NavState::placeholder();
        self.config.position_time_constant_s = 0.0;
        self.config.velocity_time_constant_s = 0.0;
        self.config.attitude_time_constant_s = 0.0;
        self.config.position_reset_threshold_m = 0.0;
        self.config.velocity_reset_threshold_mps = 0.0;
        self.config.attitude_reset_threshold_rad = 0.0;
        self.pending.position_n.fill(0.0);
        self.pending.velocity_n.fill(0.0);
        self.pending.attitude_b.fill(0.0);
        self.tracking_error = PredictorTrackingError::default();
        self.degraded_input = false;
    }

    pub(crate) fn initialize(
        &mut self,
        state: NavState,
        config: PredictorConfig,
    ) -> Result<(), PredictorError> {
        self.reset();
        config.validate()?;
        if !state.is_finite() {
            return Err(PredictorError::NonFinite);
        }
        self.state = state;
        self.config = config;
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn new(state: NavState, config: PredictorConfig) -> Result<Self, PredictorError> {
        let mut result = Self::placeholder();
        result.initialize(state, config)?;
        Ok(result)
    }

    pub(crate) fn propagate(
        &mut self,
        batch: &PreintegratedBatch,
        context: &MechanizationContext,
    ) -> Result<(), PredictorError> {
        let corrected = batch
            .corrected(self.state.accel_bias_b, self.state.gyro_bias_b)
            .map_err(PredictorError::Preintegration)?;
        let dt = corrected
            .duration_seconds()
            .map_err(PredictorError::Preintegration)?;
        propagate_nominal(&mut self.state, &corrected, context).map_err(PredictorError::Eskf)?;
        self.apply_complementary_correction(dt)?;
        self.degraded_input = batch.degraded_input;
        Ok(())
    }

    /// Returns whether the most recently measured delayed-frontier
    /// discrepancy crossed a configured hard-reset threshold. The error is
    /// deliberately retained after the reset so the public present estimate
    /// can report the tracking excursion for at least one update cycle.
    pub(crate) fn tracking_degraded(&self) -> bool {
        self.tracking_error.position_m > self.config.position_reset_threshold_m
            || self.tracking_error.velocity_mps > self.config.velocity_reset_threshold_mps
            || self.tracking_error.attitude_rad > self.config.attitude_reset_threshold_rad
    }

    /// Transfers the delayed-filter discrepancy at a matched historical epoch
    /// to the present output state. `predicted_at_frontier` must come from the
    /// predictor history, not from the corrected trajectory.
    pub(crate) fn correct_from_frontier(
        &mut self,
        corrected_at_frontier: &NavState,
        predicted_at_frontier: &NavState,
    ) -> Result<(), PredictorError> {
        if corrected_at_frontier.time != predicted_at_frontier.time
            || predicted_at_frontier.time > self.state.time
        {
            return Err(PredictorError::TimeMismatch);
        }
        let lag_ns = self
            .state
            .time
            .as_ns()
            .checked_sub(predicted_at_frontier.time.as_ns())
            .ok_or(PredictorError::TimeOverflow)?;
        let lag_s = lag_ns as f32 * 1.0e-9;
        let position_error_at_frontier =
            corrected_at_frontier.position_n - predicted_at_frontier.position_n;
        let velocity_error = corrected_at_frontier.velocity_n - predicted_at_frontier.velocity_n;
        let attitude_error_at_frontier = so3_log(
            &(predicted_at_frontier.orientation_n_from_b.inverse()
                * corrected_at_frontier.orientation_n_from_b),
        );
        let relative_prediction =
            predicted_at_frontier.orientation_n_from_b.inverse() * self.state.orientation_n_from_b;
        let attitude_error_now =
            relative_prediction.inverse_transform_vector(&attitude_error_at_frontier);
        let position_error_now = position_error_at_frontier + velocity_error * lag_s;

        self.tracking_error.position_m = position_error_now.norm();
        self.tracking_error.velocity_mps = velocity_error.norm();
        self.tracking_error.attitude_rad = attitude_error_now.norm();
        if !self.tracking_error.position_m.is_finite()
            || !self.tracking_error.velocity_mps.is_finite()
            || !self.tracking_error.attitude_rad.is_finite()
        {
            return Err(PredictorError::NonFinite);
        }

        // Biases are random-walk states in the delayed filter, not smoothing
        // corrections.  Carry their newest estimate straight into future
        // predictor preintegration so the low-latency path does not continue
        // mechanizing with an obsolete bias after a GNSS update.
        self.state.accel_bias_b = corrected_at_frontier.accel_bias_b;
        self.state.gyro_bias_b = corrected_at_frontier.gyro_bias_b;

        let hard_reset = self.tracking_error.position_m > self.config.position_reset_threshold_m
            || self.tracking_error.velocity_mps > self.config.velocity_reset_threshold_mps
            || self.tracking_error.attitude_rad > self.config.attitude_reset_threshold_rad;
        if hard_reset {
            let mut correction = NavVector::zeros();
            correction
                .fixed_rows_mut::<3>(POS)
                .copy_from(&position_error_now);
            correction
                .fixed_rows_mut::<3>(VEL)
                .copy_from(&velocity_error);
            correction
                .fixed_rows_mut::<3>(ATT)
                .copy_from(&attitude_error_now);
            self.state
                .inject(&correction)
                .map_err(PredictorError::State)?;
            self.pending = PendingCorrection::default();
            self.tracking_error.hard_resets = self.tracking_error.hard_resets.saturating_add(1);
        } else {
            self.pending.position_n += position_error_now;
            self.pending.velocity_n += velocity_error;
            self.pending.attitude_b += attitude_error_now;
        }
        Ok(())
    }

    fn apply_complementary_correction(&mut self, dt: f32) -> Result<(), PredictorError> {
        let position_fraction = dt / (self.config.position_time_constant_s + dt);
        let velocity_fraction = dt / (self.config.velocity_time_constant_s + dt);
        let attitude_fraction = dt / (self.config.attitude_time_constant_s + dt);
        let position = self.pending.position_n * position_fraction;
        let velocity = self.pending.velocity_n * velocity_fraction;
        let attitude = self.pending.attitude_b * attitude_fraction;
        let mut correction = NavVector::zeros();
        correction.fixed_rows_mut::<3>(POS).copy_from(&position);
        correction.fixed_rows_mut::<3>(VEL).copy_from(&velocity);
        correction.fixed_rows_mut::<3>(ATT).copy_from(&attitude);
        self.state
            .inject(&correction)
            .map_err(PredictorError::State)?;
        self.pending.position_n -= position;
        self.pending.velocity_n -= velocity;
        self.pending.attitude_b -= attitude;
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn pending_norms(&self) -> (f32, f32, f32) {
        (
            self.pending.position_n.norm(),
            self.pending.velocity_n.norm(),
            self.pending.attitude_b.norm(),
        )
    }

    pub(super) fn validate_reanchor(
        &self,
        transform: &ReanchorTransform,
    ) -> Result<(), ReanchorError> {
        transform.validate_state(self.state)?;
        let pending_position = transform.map_vector(self.pending.position_n);
        let pending_velocity = transform.map_vector(self.pending.velocity_n);
        if pending_position
            .iter()
            .chain(pending_velocity.iter())
            .chain(self.pending.attitude_b.iter())
            .any(|value| !value.is_finite())
        {
            return Err(ReanchorError::NonFinite);
        }
        Ok(())
    }

    /// Applies a transform previously accepted by `validate_reanchor`.
    pub(super) fn apply_reanchor(&mut self, transform: &ReanchorTransform) {
        self.state = transform.map_state(self.state);
        self.pending.position_n = transform.map_vector(self.pending.position_n);
        self.pending.velocity_n = transform.map_vector(self.pending.velocity_n);
        // Right-multiplicative attitude corrections are body-tangent vectors
        // and are invariant under a left change of navigation frame.
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PredictorError {
    InvalidConfiguration,
    NonFinite,
    TimeMismatch,
    TimeOverflow,
    Preintegration(super::preintegration::PreintegrationError),
    Eskf(EskfError),
    State(super::state::StateError),
}

#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;

    use super::*;
    use crate::live::reanchor::EcefAnchor;
    use crate::time::SessionTime;

    fn config() -> PredictorConfig {
        PredictorConfig {
            position_time_constant_s: 0.1,
            velocity_time_constant_s: 0.1,
            attitude_time_constant_s: 0.1,
            position_reset_threshold_m: 10.0,
            velocity_reset_threshold_mps: 10.0,
            attitude_reset_threshold_rad: 1.0,
        }
    }

    #[test]
    fn small_frontier_correction_is_smoothed_not_reset() {
        let state = NavState::stationary(SessionTime::from_ns(1_000_000_000));
        let mut predictor = OutputPredictor::new(state, config()).unwrap();
        let mut predicted_frontier = state;
        predicted_frontier.time = SessionTime::from_ns(500_000_000);
        let mut corrected_frontier = predicted_frontier;
        corrected_frontier.position_n.x = 1.0;
        predictor
            .correct_from_frontier(&corrected_frontier, &predicted_frontier)
            .unwrap();
        assert!(!predictor.tracking_degraded());
        assert_eq!(predictor.state.position_n.x, 0.0);
        assert!((predictor.pending_norms().0 - 1.0).abs() < 1.0e-6);
        assert_eq!(predictor.tracking_error.hard_resets, 0);
    }

    #[test]
    fn excessive_discrepancy_causes_bounded_hard_reset() {
        let state = NavState::stationary(SessionTime::from_ns(1_000_000_000));
        let mut predictor = OutputPredictor::new(state, config()).unwrap();
        let mut corrected = state;
        corrected.position_n.x = 20.0;
        predictor.correct_from_frontier(&corrected, &state).unwrap();
        assert!(predictor.tracking_degraded());
        assert!((predictor.state.position_n.x - 20.0).abs() < 1.0e-6);
        assert_eq!(predictor.tracking_error.hard_resets, 1);
        assert_eq!(predictor.pending_norms(), (0.0, 0.0, 0.0));
    }

    #[test]
    fn velocity_error_is_projected_across_frontier_lag() {
        let mut current = NavState::stationary(SessionTime::from_ns(1_000_000_000));
        current.velocity_n.x = 1.0;
        let mut predictor = OutputPredictor::new(current, config()).unwrap();
        let predicted = NavState::stationary(SessionTime::ZERO);
        let mut corrected = predicted;
        corrected.velocity_n.x = 0.5;
        predictor
            .correct_from_frontier(&corrected, &predicted)
            .unwrap();
        let (position, velocity, _) = predictor.pending_norms();
        assert!((position - 0.5).abs() < 1.0e-6);
        assert!((velocity - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn reanchor_rotates_pending_navigation_corrections_in_one_step() {
        let old = EcefAnchor::new(
            1,
            Vector3::new(6_000_000.0, 100.0, 50.0),
            Matrix3::identity(),
        )
        .unwrap();
        let angle = 0.6_f64;
        let new = EcefAnchor::new(
            2,
            Vector3::new(6_000_010.0, 80.0, 55.0),
            Matrix3::new(
                angle.cos(),
                -angle.sin(),
                0.0,
                angle.sin(),
                angle.cos(),
                0.0,
                0.0,
                0.0,
                1.0,
            ),
        )
        .unwrap();
        let mut state = NavState::stationary(SessionTime::from_ns(1_000_000_000));
        state.position_n = Vector3::new(30.0, -4.0, 2.0);
        state.velocity_n = Vector3::new(3.0, 1.0, -0.2);
        let mut predictor = OutputPredictor::new(state, config()).unwrap();
        let predicted = state;
        let mut corrected = predicted;
        corrected.position_n += Vector3::new(0.5, -0.2, 0.1);
        corrected.velocity_n += Vector3::new(-0.1, 0.3, 0.2);
        corrected.orientation_n_from_b *=
            super::super::state::so3_exp(Vector3::new(0.01, -0.02, 0.03));
        predictor
            .correct_from_frontier(&corrected, &predicted)
            .unwrap();

        let state_before = predictor.state;
        let pending_before = predictor.pending;
        let transform = ReanchorTransform::between(&old, &new).unwrap();
        predictor.validate_reanchor(&transform).unwrap();
        predictor.apply_reanchor(&transform);

        assert!(
            (new.position_to_ecef(predictor.state.position_n)
                - old.position_to_ecef(state_before.position_n))
            .norm()
                < 2.0e-5
        );
        assert!(
            (new.vector_to_ecef(predictor.pending.position_n)
                - old.vector_to_ecef(pending_before.position_n))
            .norm()
                < 2.0e-6
        );
        assert!(
            (new.vector_to_ecef(predictor.pending.velocity_n)
                - old.vector_to_ecef(pending_before.velocity_n))
            .norm()
                < 2.0e-6
        );
        assert_eq!(predictor.pending.attitude_b, pending_before.attitude_b);
    }
}
