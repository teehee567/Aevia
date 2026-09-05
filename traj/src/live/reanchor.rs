//! Deterministic fixed-ENU re-anchoring while preserving the physical state.

use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::frame::ReferenceEllipsoid;

use super::{
    eskf::Eskf,
    state::{MechanizationContext, NavMatrix, NavState, POS, VEL},
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EcefAnchor {
    pub(crate) generation: u32,
    pub(crate) origin_ecef_m: Vector3<f64>,
    /// Rotation from ECEF vector coordinates to this fixed ENU frame.
    pub(crate) ecef_to_n: Matrix3<f64>,
}

impl EcefAnchor {
    /// Builds the fixed geodetic ENU basis for an ECEF origin. The conversion
    /// uses the configured terrestrial frame's reference ellipsoid; the Earth
    /// centre is deliberately rejected because local up is undefined there.
    pub(crate) fn from_origin(
        generation: u32,
        origin_ecef_m: Vector3<f64>,
        ellipsoid: ReferenceEllipsoid,
    ) -> Result<Self, ReanchorError> {
        if origin_ecef_m.iter().any(|value| !value.is_finite()) {
            return Err(ReanchorError::NonFinite);
        }
        let x = origin_ecef_m.x;
        let y = origin_ecef_m.y;
        let z = origin_ecef_m.z;
        let horizontal = crate::scalar_math::sqrt(x * x + y * y);
        if horizontal == 0.0 && z == 0.0 {
            return Err(ReanchorError::UndefinedTangentFrame);
        }
        let a = ellipsoid.semi_major_axis_m();
        let flattening = 1.0 / ellipsoid.inverse_flattening();
        let b = a * (1.0 - flattening);
        let e2 = flattening * (2.0 - flattening);
        let ep2 = (a * a - b * b) / (b * b);
        let longitude = crate::scalar_math::atan2(y, x);
        let theta = crate::scalar_math::atan2(z * a, horizontal * b);
        let sin_theta = crate::scalar_math::sin(theta);
        let cos_theta = crate::scalar_math::cos(theta);
        let latitude = crate::scalar_math::atan2(
            z + ep2 * b * sin_theta * sin_theta * sin_theta,
            horizontal - e2 * a * cos_theta * cos_theta * cos_theta,
        );
        let sin_latitude = crate::scalar_math::sin(latitude);
        let cos_latitude = crate::scalar_math::cos(latitude);
        let sin_longitude = crate::scalar_math::sin(longitude);
        let cos_longitude = crate::scalar_math::cos(longitude);
        let ecef_to_n = Matrix3::new(
            -sin_longitude,
            cos_longitude,
            0.0,
            -sin_latitude * cos_longitude,
            -sin_latitude * sin_longitude,
            cos_latitude,
            cos_latitude * cos_longitude,
            cos_latitude * sin_longitude,
            sin_latitude,
        );
        Self::new(generation, origin_ecef_m, ecef_to_n)
    }

    pub(crate) fn new(
        generation: u32,
        origin_ecef_m: Vector3<f64>,
        ecef_to_n: Matrix3<f64>,
    ) -> Result<Self, ReanchorError> {
        if origin_ecef_m
            .iter()
            .chain(ecef_to_n.iter())
            .any(|value| !value.is_finite())
        {
            return Err(ReanchorError::NonFinite);
        }
        let orthogonality = ecef_to_n * ecef_to_n.transpose() - Matrix3::identity();
        if orthogonality.norm() > 1.0e-9 || (ecef_to_n.determinant() - 1.0).abs() > 1.0e-9 {
            return Err(ReanchorError::InvalidRotation);
        }
        Ok(Self {
            generation,
            origin_ecef_m,
            ecef_to_n,
        })
    }

    pub(crate) fn position_to_ecef(&self, position_n: Vector3<f32>) -> Vector3<f64> {
        self.origin_ecef_m + self.ecef_to_n.transpose() * position_n.cast::<f64>()
    }

    pub(crate) fn position_from_ecef(&self, position_ecef: Vector3<f64>) -> Vector3<f32> {
        (self.ecef_to_n * (position_ecef - self.origin_ecef_m)).cast::<f32>()
    }

    pub(crate) fn vector_from_ecef(&self, vector_ecef: Vector3<f64>) -> Vector3<f32> {
        (self.ecef_to_n * vector_ecef).cast::<f32>()
    }

    pub(crate) fn vector_to_ecef(&self, vector_n: Vector3<f32>) -> Vector3<f64> {
        self.ecef_to_n.transpose() * vector_n.cast::<f64>()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ReanchorPolicy {
    pub(crate) trigger_distance_m: f32,
    pub(crate) rearm_distance_m: f32,
}

impl ReanchorPolicy {
    pub(crate) fn validate(&self) -> Result<(), ReanchorError> {
        if !self.trigger_distance_m.is_finite()
            || !self.rearm_distance_m.is_finite()
            || self.trigger_distance_m <= 0.0
            || self.rearm_distance_m < 0.0
            || self.rearm_distance_m >= self.trigger_distance_m
        {
            return Err(ReanchorError::InvalidPolicy);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ReanchorMonitor {
    policy: ReanchorPolicy,
    armed: bool,
}

impl ReanchorMonitor {
    pub(crate) fn new(policy: ReanchorPolicy) -> Result<Self, ReanchorError> {
        policy.validate()?;
        Ok(Self {
            policy,
            armed: true,
        })
    }

    pub(crate) fn observe(&mut self, position_n: Vector3<f32>) -> bool {
        let distance = position_n.norm();
        if !distance.is_finite() {
            return false;
        }
        if self.armed && distance >= self.policy.trigger_distance_m {
            self.armed = false;
            return true;
        }
        if !self.armed && distance <= self.policy.rearm_distance_m {
            self.armed = true;
        }
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ReanchorTransform {
    pub(crate) new_n_from_old_n: Matrix3<f32>,
    pub(crate) covariance_jacobian: NavMatrix,
    new_n_from_old_n_f64: Matrix3<f64>,
    old_origin_in_new_n_m: Vector3<f64>,
    new_origin_in_old_n_m: Vector3<f64>,
}

impl ReanchorTransform {
    /// Precomputes the complete affine frame change.  Keeping the translation
    /// in `f64` avoids subtracting two Earth-sized ECEF positions through an
    /// `f32` intermediate when many retained local states are transformed.
    pub(crate) fn between(
        old_anchor: &EcefAnchor,
        new_anchor: &EcefAnchor,
    ) -> Result<Self, ReanchorError> {
        EcefAnchor::new(
            old_anchor.generation,
            old_anchor.origin_ecef_m,
            old_anchor.ecef_to_n,
        )?;
        EcefAnchor::new(
            new_anchor.generation,
            new_anchor.origin_ecef_m,
            new_anchor.ecef_to_n,
        )?;
        if new_anchor.generation <= old_anchor.generation {
            return Err(ReanchorError::GenerationNotIncreasing);
        }
        let rotation_f64 = new_anchor.ecef_to_n * old_anchor.ecef_to_n.transpose();
        let old_origin_in_new_n_m =
            new_anchor.ecef_to_n * (old_anchor.origin_ecef_m - new_anchor.origin_ecef_m);
        let new_origin_in_old_n_m =
            old_anchor.ecef_to_n * (new_anchor.origin_ecef_m - old_anchor.origin_ecef_m);
        if rotation_f64
            .iter()
            .chain(old_origin_in_new_n_m.iter())
            .chain(new_origin_in_old_n_m.iter())
            .any(|value| !value.is_finite())
        {
            return Err(ReanchorError::NonFinite);
        }
        let rotation = rotation_f64.cast::<f32>();
        let orthogonality = rotation * rotation.transpose() - Matrix3::identity();
        if rotation.iter().any(|value| !value.is_finite())
            || orthogonality.norm() > 2.0e-5
            || (rotation.determinant() - 1.0).abs() > 2.0e-5
        {
            return Err(ReanchorError::InvalidRotation);
        }
        let mut jacobian = NavMatrix::identity();
        jacobian
            .fixed_view_mut::<3, 3>(POS, POS)
            .copy_from(&rotation);
        jacobian
            .fixed_view_mut::<3, 3>(VEL, VEL)
            .copy_from(&rotation);
        Ok(Self {
            new_n_from_old_n: rotation,
            covariance_jacobian: jacobian,
            new_n_from_old_n_f64: rotation_f64,
            old_origin_in_new_n_m,
            new_origin_in_old_n_m,
        })
    }

    pub(crate) fn map_position(&self, position_old_n: Vector3<f32>) -> Vector3<f32> {
        (self.new_n_from_old_n_f64 * position_old_n.cast::<f64>() + self.old_origin_in_new_n_m)
            .cast::<f32>()
    }

    pub(crate) fn map_vector(&self, vector_old_n: Vector3<f32>) -> Vector3<f32> {
        self.new_n_from_old_n * vector_old_n
    }

    pub(crate) fn map_covariance(&self, covariance_old_n: Matrix3<f32>) -> Matrix3<f32> {
        let mapped = self.new_n_from_old_n * covariance_old_n * self.new_n_from_old_n.transpose();
        (mapped + mapped.transpose()) * 0.5
    }

    pub(crate) fn map_state(&self, state: NavState) -> NavState {
        let mut result = state;
        result.position_n = self.map_position(state.position_n);
        result.velocity_n = self.map_vector(state.velocity_n);
        let frame_rotation = UnitQuaternion::from_rotation_matrix(
            &Rotation3::from_matrix_unchecked(self.new_n_from_old_n),
        );
        result.orientation_n_from_b = frame_rotation * state.orientation_n_from_b;
        result.orientation_n_from_b.renormalize();
        result
    }

    pub(crate) fn validate_state(&self, state: NavState) -> Result<(), ReanchorError> {
        if !self.map_state(state).is_finite() {
            return Err(ReanchorError::NonFinite);
        }
        Ok(())
    }

    pub(crate) fn map_context(
        &self,
        context: MechanizationContext,
    ) -> Result<MechanizationContext, ReanchorError> {
        // The old gravity model is g_old(p_old) = g0_old + G_old p_old.
        // Evaluate it at the new origin, rotate the vector, and rotate both
        // axes of its spatial derivative.  This preserves the same physical
        // first-order vector field rather than merely rotating g0.
        let rotation = self.new_n_from_old_n_f64;
        let gradient_old = context.gravity_gradient_n.cast::<f64>();
        let gravity_at_new_origin_old_n =
            context.gravity_at_anchor_n.cast::<f64>() + gradient_old * self.new_origin_in_old_n_m;
        let earth_rate_n = (rotation * context.earth_rate_n.cast::<f64>()).cast::<f32>();
        let gravity_at_anchor_n = (rotation * gravity_at_new_origin_old_n).cast::<f32>();
        let gravity_gradient_n = (rotation * gradient_old * rotation.transpose()).cast::<f32>();
        MechanizationContext::new(earth_rate_n, gravity_at_anchor_n, gravity_gradient_n)
            .map_err(|_| ReanchorError::NonFinite)
    }

    /// Maps a filter into caller-owned scratch. The destination may be
    /// modified on error and must not alias the live source; callers publish
    /// it only after every reanchor validation has succeeded.
    #[inline(never)]
    pub(crate) fn map_filter_into(
        &self,
        filter: &Eskf,
        result: &mut Eskf,
    ) -> Result<(), ReanchorError> {
        *result = *filter;
        result.state = self.map_state(filter.state);
        result.covariance =
            self.covariance_jacobian * filter.covariance * self.covariance_jacobian.transpose();
        result.nav_consider_covariance = self.covariance_jacobian * filter.nav_consider_covariance;
        result.gap_nav_cross_covariance =
            self.covariance_jacobian * filter.gap_nav_cross_covariance;
        result.covariance = (result.covariance + result.covariance.transpose()) * 0.5;
        if !result.state.is_finite()
            || !result.covariance.iter().all(|value| value.is_finite())
            || !result
                .nav_consider_covariance
                .iter()
                .chain(result.gap_nav_cross_covariance.iter())
                .all(|value| value.is_finite())
        {
            return Err(ReanchorError::NonFinite);
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) fn transform_state(
    state: &mut NavState,
    old_anchor: &EcefAnchor,
    new_anchor: &EcefAnchor,
) -> Result<ReanchorTransform, ReanchorError> {
    let transform = ReanchorTransform::between(old_anchor, new_anchor)?;
    transform.validate_state(*state)?;
    *state = transform.map_state(*state);
    Ok(transform)
}

#[cfg(test)]
pub(crate) fn transform_filter(
    filter: &mut Eskf,
    old_anchor: &EcefAnchor,
    new_anchor: &EcefAnchor,
) -> Result<ReanchorTransform, ReanchorError> {
    let transform = ReanchorTransform::between(old_anchor, new_anchor)?;
    let mut transformed = *filter;
    transform.map_filter_into(filter, &mut transformed)?;
    *filter = transformed;
    Ok(transform)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ReanchorError {
    NonFinite,
    UndefinedTangentFrame,
    InvalidRotation,
    InvalidPolicy,
    GenerationNotIncreasing,
}

#[cfg(test)]
mod tests {
    use super::super::{
        eskf::{ConsiderCovariance, CovariancePolicy, NavConsiderCovariance, ProcessNoise},
        state::NavMatrix,
    };
    use super::*;
    use crate::time::SessionTime;

    fn anchor(generation: u32, origin: Vector3<f64>, yaw: f64) -> EcefAnchor {
        let c = yaw.cos();
        let s = yaw.sin();
        EcefAnchor::new(
            generation,
            origin,
            Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn physical_ecef_position_survives_round_trip_reanchor() {
        let old = anchor(1, Vector3::new(6_000_000.0, 1_000.0, 2_000.0), 0.0);
        let new = anchor(2, Vector3::new(6_000_100.0, 1_010.0, 1_995.0), 0.3);
        let mut state = NavState::stationary(SessionTime::ZERO);
        state.position_n = Vector3::new(150.0, 25.0, -2.0);
        state.velocity_n = Vector3::new(4.0, -1.0, 0.2);
        let physical_before = old.position_to_ecef(state.position_n);
        let speed_before = state.velocity_n.norm();
        transform_state(&mut state, &old, &new).unwrap();
        let physical_after = new.position_to_ecef(state.position_n);
        assert!((physical_after - physical_before).norm() < 2.0e-5);
        assert!((state.velocity_n.norm() - speed_before).abs() < 1.0e-5);
    }

    #[test]
    fn covariance_rotation_preserves_trace_for_navigation_vectors() {
        let old = anchor(1, Vector3::zeros(), 0.0);
        let new = anchor(2, Vector3::new(10.0, 0.0, 0.0), 0.7);
        let mut filter = Eskf::new(
            NavState::stationary(SessionTime::ZERO),
            NavMatrix::identity(),
            NavConsiderCovariance::zeros(),
            ConsiderCovariance::zeros(),
            0,
            ProcessNoise {
                accel_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-8,
                gyro_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-10,
            },
            CovariancePolicy::conservative_candidate(),
        )
        .unwrap();
        let trace_before = filter.covariance.trace();
        transform_filter(&mut filter, &old, &new).unwrap();
        assert!((filter.covariance.trace() - trace_before).abs() < 1.0e-4);
    }

    #[test]
    fn reanchor_rotates_an_active_gap_cross_covariance() {
        let old = anchor(1, Vector3::zeros(), 0.0);
        let new = anchor(2, Vector3::new(10.0, 0.0, 0.0), 0.7);
        let mut filter = Eskf::new(
            NavState::stationary(SessionTime::ZERO),
            NavMatrix::identity(),
            NavConsiderCovariance::zeros(),
            ConsiderCovariance::zeros(),
            0,
            ProcessNoise {
                accel_bias_random_walk_covariance_density: Matrix3::zeros(),
                gyro_bias_random_walk_covariance_density: Matrix3::zeros(),
            },
            CovariancePolicy::conservative_candidate(),
        )
        .unwrap();
        filter.gap_origin = Some(SessionTime::ZERO);
        filter.gap_derivative_covariance.fill(0.0);
        for index in 0..6 {
            filter.gap_derivative_covariance[(index, index)] = 1.0;
        }
        for row in 0..15 {
            for column in 0..6 {
                filter.gap_nav_cross_covariance[(row, column)] =
                    (row * 6 + column + 1) as f32 * 1.0e-4;
            }
        }
        let before = filter.gap_nav_cross_covariance;
        let transform = ReanchorTransform::between(&old, &new).unwrap();
        let expected = transform.covariance_jacobian * before;

        transform_filter(&mut filter, &old, &new).unwrap();

        assert_eq!(filter.gap_origin, Some(SessionTime::ZERO));
        assert!((filter.gap_nav_cross_covariance - expected).norm() < 1.0e-7);
    }

    #[test]
    fn hysteresis_prevents_reanchor_chatter() {
        let mut monitor = ReanchorMonitor::new(ReanchorPolicy {
            trigger_distance_m: 1_000.0,
            rearm_distance_m: 100.0,
        })
        .unwrap();
        assert!(monitor.observe(Vector3::new(1_001.0, 0.0, 0.0)));
        assert!(!monitor.observe(Vector3::new(1_100.0, 0.0, 0.0)));
        assert!(!monitor.observe(Vector3::new(50.0, 0.0, 0.0)));
        assert!(monitor.observe(Vector3::new(1_001.0, 0.0, 0.0)));
    }
}
