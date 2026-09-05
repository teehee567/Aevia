//! Nominal navigation state and the small fixed-size math used by live fusion.

use nalgebra::{ArrayStorage, Matrix3, Quaternion, SMatrix, UnitQuaternion, Vector3};

use crate::time::SessionTime;

pub(crate) const NAV_DIM: usize = 15;
pub(crate) const POS: usize = 0;
pub(crate) const VEL: usize = 3;
pub(crate) const ATT: usize = 6;
pub(crate) const ACC_BIAS: usize = 9;
pub(crate) const GYRO_BIAS: usize = 12;

pub(crate) type NavMatrix = SMatrix<f32, NAV_DIM, NAV_DIM>;
pub(crate) type NavVector = nalgebra::SVector<f32, NAV_DIM>;

/// Nominal state at the IMU sensing centre in one fixed-anchor ENU frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct NavState {
    pub(crate) time: SessionTime,
    pub(crate) position_n: Vector3<f32>,
    pub(crate) velocity_n: Vector3<f32>,
    pub(crate) orientation_n_from_b: UnitQuaternion<f32>,
    pub(crate) accel_bias_b: Vector3<f32>,
    pub(crate) gyro_bias_b: Vector3<f32>,
}

impl NavState {
    /// Valid finite state used only while caller-owned live storage is
    /// inactive. It can be built directly in static storage.
    pub(crate) const fn placeholder() -> Self {
        Self {
            time: SessionTime::ZERO,
            position_n: Vector3::new(0.0, 0.0, 0.0),
            velocity_n: Vector3::new(0.0, 0.0, 0.0),
            orientation_n_from_b: UnitQuaternion::new_unchecked(Quaternion::new(
                1.0, 0.0, 0.0, 0.0,
            )),
            accel_bias_b: Vector3::new(0.0, 0.0, 0.0),
            gyro_bias_b: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    #[cfg(test)]
    pub(crate) fn stationary(time: SessionTime) -> Self {
        Self {
            time,
            position_n: Vector3::zeros(),
            velocity_n: Vector3::zeros(),
            orientation_n_from_b: UnitQuaternion::identity(),
            accel_bias_b: Vector3::zeros(),
            gyro_bias_b: Vector3::zeros(),
        }
    }

    pub(crate) fn is_finite(&self) -> bool {
        vector_is_finite(&self.position_n)
            && vector_is_finite(&self.velocity_n)
            && self
                .orientation_n_from_b
                .quaternion()
                .coords
                .iter()
                .all(|value| value.is_finite())
            && vector_is_finite(&self.accel_bias_b)
            && vector_is_finite(&self.gyro_bias_b)
    }

    /// Applies an ESKF correction using the right-multiplicative convention.
    pub(crate) fn inject(&mut self, correction: &NavVector) -> Result<Matrix3<f32>, StateError> {
        if !correction.iter().all(|value| value.is_finite()) {
            return Err(StateError::NonFinite);
        }

        self.position_n += correction.fixed_rows::<3>(POS).into_owned();
        self.velocity_n += correction.fixed_rows::<3>(VEL).into_owned();
        let delta_theta = correction.fixed_rows::<3>(ATT).into_owned();
        self.orientation_n_from_b *= so3_exp(delta_theta);
        self.orientation_n_from_b.renormalize();
        self.accel_bias_b += correction.fixed_rows::<3>(ACC_BIAS).into_owned();
        self.gyro_bias_b += correction.fixed_rows::<3>(GYRO_BIAS).into_owned();

        if !self.is_finite() {
            return Err(StateError::NonFinite);
        }

        // For R_true = R_nom Exp(delta_theta), injection by delta_hat changes
        // the remaining tangent error by J_r(delta_hat).
        Ok(right_jacobian(delta_theta))
    }
}

/// State-independent quantities cached for one fixed ENU anchor segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct MechanizationContext {
    /// Earth angular rate expressed in the fixed ENU axes.
    pub(crate) earth_rate_n: Vector3<f32>,
    /// Normal gravity (including centrifugal acceleration) at the anchor.
    pub(crate) gravity_at_anchor_n: Vector3<f32>,
    /// First spatial derivative of normal gravity in the fixed ENU axes.
    pub(crate) gravity_gradient_n: Matrix3<f32>,
}

impl MechanizationContext {
    pub(crate) const fn placeholder() -> Self {
        Self {
            earth_rate_n: Vector3::new(0.0, 0.0, 0.0),
            gravity_at_anchor_n: Vector3::new(0.0, 0.0, 0.0),
            gravity_gradient_n: Matrix3::from_array_storage(ArrayStorage([[0.0; 3]; 3])),
        }
    }

    pub(crate) fn reset(&mut self) {
        self.earth_rate_n.fill(0.0);
        self.gravity_at_anchor_n.fill(0.0);
        self.gravity_gradient_n.fill(0.0);
    }

    pub(crate) fn new(
        earth_rate_n: Vector3<f32>,
        gravity_at_anchor_n: Vector3<f32>,
        gravity_gradient_n: Matrix3<f32>,
    ) -> Result<Self, StateError> {
        let result = Self {
            earth_rate_n,
            gravity_at_anchor_n,
            gravity_gradient_n,
        };
        if !vector_is_finite(&result.earth_rate_n)
            || !vector_is_finite(&result.gravity_at_anchor_n)
            || !result
                .gravity_gradient_n
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(StateError::NonFinite);
        }
        Ok(result)
    }

    pub(crate) fn gravity_at(&self, position_n: &Vector3<f32>) -> Vector3<f32> {
        self.gravity_at_anchor_n + self.gravity_gradient_n * position_n
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum StateError {
    NonFinite,
}

pub(crate) fn vector_is_finite(vector: &Vector3<f32>) -> bool {
    vector.iter().all(|value| value.is_finite())
}

pub(crate) fn skew(vector: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

pub(crate) fn so3_exp(delta: Vector3<f32>) -> UnitQuaternion<f32> {
    UnitQuaternion::from_scaled_axis(delta)
}

pub(crate) fn so3_log(rotation: &UnitQuaternion<f32>) -> Vector3<f32> {
    rotation.scaled_axis()
}

/// SO(3) right Jacobian, including its cancellation-safe small-angle branch.
pub(crate) fn right_jacobian(theta: Vector3<f32>) -> Matrix3<f32> {
    let theta2 = theta.norm_squared();
    let theta_x = skew(&theta);
    let theta_x2 = theta_x * theta_x;
    let (a, b, _) = rotation_integral_coefficients(theta2);
    Matrix3::identity() - theta_x * a + theta_x2 * b
}

/// Integral `int_0^1 Exp([theta] u) du`, used for rotating interval-average
/// specific force without a second, inconsistent sculling correction.
pub(crate) fn left_jacobian(theta: Vector3<f32>) -> Matrix3<f32> {
    let theta2 = theta.norm_squared();
    let theta_x = skew(&theta);
    let theta_x2 = theta_x * theta_x;
    let (a, b, _) = rotation_integral_coefficients(theta2);
    Matrix3::identity() + theta_x * a + theta_x2 * b
}

/// Integral `int_0^1 (1-u) Exp([theta] u) du` for within-interval position.
pub(crate) fn second_rotation_integral(theta: Vector3<f32>) -> Matrix3<f32> {
    let theta2 = theta.norm_squared();
    let theta_x = skew(&theta);
    let theta_x2 = theta_x * theta_x;
    let (_, b, c) = rotation_integral_coefficients(theta2);
    Matrix3::identity() * 0.5 + theta_x * b + theta_x2 * c
}

/// The trigonometric numerators lose several orders of precision at normal
/// IMU increments in f32, particularly the fourth-order position numerator.
/// These series keep the truncation below f32 roundoff through one radian.
fn rotation_integral_coefficients(theta2: f32) -> (f32, f32, f32) {
    if theta2 <= 1.0 {
        let a = 0.5
            + theta2
                * (-1.0 / 24.0
                    + theta2 * (1.0 / 720.0 + theta2 * (-1.0 / 40_320.0 + theta2 / 3_628_800.0)));
        let b = 1.0 / 6.0
            + theta2
                * (-1.0 / 120.0
                    + theta2
                        * (1.0 / 5_040.0 + theta2 * (-1.0 / 362_880.0 + theta2 / 39_916_800.0)));
        let c = 1.0 / 24.0
            + theta2
                * (-1.0 / 720.0
                    + theta2
                        * (1.0 / 40_320.0
                            + theta2 * (-1.0 / 3_628_800.0 + theta2 / 479_001_600.0)));
        return (a, b, c);
    }
    let magnitude = crate::scalar_math::sqrt(theta2);
    let cosine = crate::scalar_math::cos(magnitude);
    let a = (1.0 - cosine) / theta2;
    let b = (magnitude - crate::scalar_math::sin(magnitude)) / (theta2 * magnitude);
    let c = (0.5 * theta2 + cosine - 1.0) / (theta2 * theta2);
    (a, b, c)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_matrix_close(left: &Matrix3<f32>, right: &Matrix3<f32>, tolerance: f32) {
        assert!(
            (left - right).norm() <= tolerance,
            "left={left:?} right={right:?}"
        );
    }

    #[test]
    fn skew_reproduces_cross_product() {
        let a = Vector3::new(0.2, -0.7, 1.3);
        let b = Vector3::new(-2.0, 0.4, 0.1);
        assert!((skew(&a) * b - a.cross(&b)).norm() < 1.0e-6);
    }

    #[test]
    fn exp_log_round_trip_on_reachable_increment() {
        let theta = Vector3::new(0.01, -0.02, 0.03);
        assert!((so3_log(&so3_exp(theta)) - theta).norm() < 1.0e-6);
    }

    #[test]
    fn jacobians_have_correct_zero_limits() {
        let zero = Vector3::zeros();
        assert_matrix_close(&right_jacobian(zero), &Matrix3::identity(), 0.0);
        assert_matrix_close(&left_jacobian(zero), &Matrix3::identity(), 0.0);
        assert_matrix_close(
            &second_rotation_integral(zero),
            &(Matrix3::identity() * 0.5),
            0.0,
        );
    }

    #[test]
    fn rotation_integrals_match_f64_quadrature_at_sensor_scale_angles() {
        let axis = Vector3::<f64>::new(1.0, -2.0, 3.0).normalize();
        for angle in [0.000_11, 0.000_3, 0.001_1, 0.01, 0.1, 0.5, 1.0, 1.01, 2.0] {
            let theta = (axis * angle).cast::<f32>();
            let theta64 = theta.cast::<f64>();
            let mut first = Matrix3::<f64>::zeros();
            let mut second = Matrix3::<f64>::zeros();
            // Simpson integration of the actual rotation is independent of
            // the coefficient formulas and stays accurate near zero.
            const STEPS: usize = 512;
            for index in 0..=STEPS {
                let u = index as f64 / STEPS as f64;
                let weight = if index == 0 || index == STEPS {
                    1.0
                } else if index % 2 == 0 {
                    2.0
                } else {
                    4.0
                };
                let rotation = UnitQuaternion::from_scaled_axis(theta64 * u)
                    .to_rotation_matrix()
                    .into_inner();
                first += rotation * weight;
                second += rotation * (weight * (1.0 - u));
            }
            first /= 3.0 * STEPS as f64;
            second /= 3.0 * STEPS as f64;
            assert_matrix_close(&left_jacobian(theta), &first.cast::<f32>(), 2.0e-7);
            assert_matrix_close(
                &right_jacobian(theta),
                &first.transpose().cast::<f32>(),
                2.0e-7,
            );
            assert_matrix_close(
                &second_rotation_integral(theta),
                &second.cast::<f32>(),
                2.0e-7,
            );
        }
    }

    #[test]
    fn state_injection_is_right_multiplicative() {
        let mut state = NavState::stationary(SessionTime::ZERO);
        let mut correction = NavVector::zeros();
        correction[ATT + 2] = 0.1;
        let reset = state.inject(&correction).unwrap();
        assert!((so3_log(&state.orientation_n_from_b).z - 0.1).abs() < 1.0e-6);
        assert!((reset - right_jacobian(Vector3::new(0.0, 0.0, 0.1))).norm() < 1.0e-6);
    }
}
