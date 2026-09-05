//! Small validated numerical value types used at the public semantic seam.
//!
//! The estimator is free to use a different scalar policy internally.  These
//! types deliberately hide `nalgebra` so changing that implementation detail
//! cannot alter the engine interface or serialized contracts.

use core::ops::{Add, Neg, Sub};

use nalgebra::{Quaternion, UnitQuaternion as NaUnitQuaternion, Vector3 as NaVector3};

use crate::error::ValidationError;

/// A finite scalar.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct FiniteF64(f64);

impl FiniteF64 {
    /// Validates and constructs a finite scalar.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(ValidationError::NonFinite)
        }
    }

    /// Returns the contained scalar.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0
    }
}

/// A finite scalar greater than or equal to zero.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct NonNegativeF64(f64);

impl NonNegativeF64 {
    /// Validates and constructs a finite non-negative scalar.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        if !value.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        if value < 0.0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self(value))
    }

    /// Returns the contained scalar.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0
    }
}

/// A finite probability in the closed interval `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct Probability(f64);

impl Probability {
    /// Validates and constructs a probability.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        if !value.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        if !(0.0..=1.0).contains(&value) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        Ok(Self(value))
    }

    /// Returns the probability as a scalar.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0
    }
}

/// A finite three-dimensional vector with no implied frame or units.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Vector3 {
    components: [f64; 3],
}

impl Vector3 {
    /// The zero vector.
    pub const ZERO: Self = Self {
        components: [0.0; 3],
    };

    /// Validates and constructs a vector.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self, ValidationError> {
        Self::from_components([x, y, z])
    }

    /// Validates and constructs a vector from ordered components.
    pub fn from_components(components: [f64; 3]) -> Result<Self, ValidationError> {
        if components.iter().all(|value| value.is_finite()) {
            Ok(Self { components })
        } else {
            Err(ValidationError::NonFinite)
        }
    }

    /// Returns the ordered components.
    #[must_use]
    pub const fn components(self) -> [f64; 3] {
        self.components
    }

    /// Returns the X component.
    #[must_use]
    pub const fn x(self) -> f64 {
        self.components[0]
    }

    /// Returns the Y component.
    #[must_use]
    pub const fn y(self) -> f64 {
        self.components[1]
    }

    /// Returns the Z component.
    #[must_use]
    pub const fn z(self) -> f64 {
        self.components[2]
    }

    /// Computes the Euclidean inner product.
    #[must_use]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x() * rhs.x() + self.y() * rhs.y() + self.z() * rhs.z()
    }

    /// Computes the right-handed cross product.
    #[must_use]
    pub fn cross(self, rhs: Self) -> Self {
        // Products and sums of finite sensor-scale inputs are finite under all
        // qualified profiles.  Keeping this operation infallible makes vector
        // algebra usable in hot paths; run-time divergence is checked by the
        // estimator after every propagation.
        Self {
            components: [
                self.y() * rhs.z() - self.z() * rhs.y(),
                self.z() * rhs.x() - self.x() * rhs.z(),
                self.x() * rhs.y() - self.y() * rhs.x(),
            ],
        }
    }

    /// Returns the squared Euclidean norm.
    #[must_use]
    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    /// Scales every component.
    #[must_use]
    pub fn scaled(self, factor: f64) -> Result<Self, ValidationError> {
        if !factor.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        Self::from_components([self.x() * factor, self.y() * factor, self.z() * factor])
    }

    fn as_nalgebra(self) -> NaVector3<f64> {
        NaVector3::new(self.x(), self.y(), self.z())
    }

    fn from_nalgebra(value: NaVector3<f64>) -> Self {
        Self {
            components: [value.x, value.y, value.z],
        }
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            components: [self.x() + rhs.x(), self.y() + rhs.y(), self.z() + rhs.z()],
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            components: [self.x() - rhs.x(), self.y() - rhs.y(), self.z() - rhs.z()],
        }
    }
}

impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            components: [-self.x(), -self.y(), -self.z()],
        }
    }
}

/// A validated unit quaternion in scalar-first `(w, x, y, z)` order.
///
/// Construction rejects a non-unit input instead of silently interpreting an
/// arbitrary quaternion as a rotation.  The sign is canonicalized so the two
/// quaternion representations of the same rotation have one deterministic
/// semantic representation.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct UnitQuaternion {
    wxyz: [f64; 4],
}

impl UnitQuaternion {
    const UNIT_NORM_SQUARED_TOLERANCE: f64 = 1.0e-10;

    /// The identity rotation.
    pub const IDENTITY: Self = Self {
        wxyz: [1.0, 0.0, 0.0, 0.0],
    };

    /// Validates a scalar-first unit quaternion.
    pub fn from_wxyz(wxyz: [f64; 4]) -> Result<Self, ValidationError> {
        if !wxyz.iter().all(|value| value.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        let norm_squared = wxyz.iter().map(|value| value * value).sum::<f64>();
        if (norm_squared - 1.0).abs() > Self::UNIT_NORM_SQUARED_TOLERANCE {
            return Err(ValidationError::InvalidRotation);
        }
        // The acceptance tolerance is intentionally wider than roundoff, so
        // normalize an accepted near-unit input before storing it. Keeping the
        // unchecked magnitude would violate this type's unit-rotation
        // invariant and slowly distort composed rotations.
        Ok(Self::from_computed(NaUnitQuaternion::new_normalize(
            Quaternion::new(wxyz[0], wxyz[1], wxyz[2], wxyz[3]),
        )))
    }

    /// Constructs the exponential-map rotation for a rotation vector in
    /// radians.
    pub fn from_rotation_vector(rotation: Vector3) -> Result<Self, ValidationError> {
        Ok(Self::from_computed(NaUnitQuaternion::from_scaled_axis(
            rotation.as_nalgebra(),
        )))
    }

    /// Returns scalar-first canonical components.
    #[must_use]
    pub const fn components_wxyz(self) -> [f64; 4] {
        self.wxyz
    }

    /// Returns the inverse rotation.
    #[must_use]
    pub fn inverse(self) -> Self {
        Self::from_computed(self.as_nalgebra().inverse())
    }

    /// Composes this rotation with `rhs`, applying `rhs` first.
    #[must_use]
    pub fn multiply(self, rhs: Self) -> Self {
        Self::from_computed(self.as_nalgebra() * rhs.as_nalgebra())
    }

    /// Rotates a vector from the quaternion's source frame into its target
    /// frame.
    #[must_use]
    pub fn rotate_vector(self, vector: Vector3) -> Vector3 {
        Vector3::from_nalgebra(self.as_nalgebra() * vector.as_nalgebra())
    }

    /// Returns the shortest exponential-map rotation vector in radians.
    #[must_use]
    pub fn rotation_vector(self) -> Vector3 {
        Vector3::from_nalgebra(self.as_nalgebra().scaled_axis())
    }

    /// Interpolates along the shortest geodesic on SO(3).
    pub fn slerp(self, rhs: Self, fraction: f64) -> Result<Self, ValidationError> {
        if !fraction.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        if !(0.0..=1.0).contains(&fraction) {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.as_nalgebra()
            .try_slerp(&rhs.as_nalgebra(), fraction, f64::EPSILON)
            .map(Self::from_computed)
            .ok_or(ValidationError::InvalidRotation)
    }

    /// Returns the equivalent row-major rotation matrix.
    #[must_use]
    pub fn rotation_matrix(self) -> [[f64; 3]; 3] {
        let matrix = self.as_nalgebra().to_rotation_matrix();
        let value = matrix.matrix();
        [
            [value[(0, 0)], value[(0, 1)], value[(0, 2)]],
            [value[(1, 0)], value[(1, 1)], value[(1, 2)]],
            [value[(2, 0)], value[(2, 1)], value[(2, 2)]],
        ]
    }

    fn as_nalgebra(self) -> NaUnitQuaternion<f64> {
        NaUnitQuaternion::from_quaternion(Quaternion::new(
            self.wxyz[0],
            self.wxyz[1],
            self.wxyz[2],
            self.wxyz[3],
        ))
    }

    fn from_computed(value: NaUnitQuaternion<f64>) -> Self {
        let quaternion = value.quaternion();
        let mut wxyz = [quaternion.w, quaternion.i, quaternion.j, quaternion.k];
        let negate = wxyz[0] < 0.0
            || (wxyz[0] == 0.0
                && (wxyz[1] < 0.0
                    || (wxyz[1] == 0.0 && (wxyz[2] < 0.0 || (wxyz[2] == 0.0 && wxyz[3] < 0.0)))));
        if negate {
            for value in &mut wxyz {
                *value = -*value;
            }
        }
        Self { wxyz }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn scalar_types_reject_non_finite_and_out_of_domain_values() {
        assert_eq!(FiniteF64::new(f64::NAN), Err(ValidationError::NonFinite));
        assert_eq!(
            NonNegativeF64::new(-0.1),
            Err(ValidationError::IncompatibleDefinition)
        );
        assert_eq!(
            Probability::new(1.000_001),
            Err(ValidationError::IncompatibleDefinition)
        );
        assert_eq!(Probability::new(0.25).unwrap().get(), 0.25);
    }

    #[test]
    fn vectors_validate_and_obey_right_handed_cross_product() {
        assert_eq!(
            Vector3::new(f64::INFINITY, 0.0, 0.0),
            Err(ValidationError::NonFinite)
        );
        let x = Vector3::new(1.0, 0.0, 0.0).unwrap();
        let y = Vector3::new(0.0, 1.0, 0.0).unwrap();
        assert_eq!(x.cross(y).components(), [0.0, 0.0, 1.0]);
        assert_eq!((x + y).components(), [1.0, 1.0, 0.0]);
    }

    #[test]
    fn quaternion_rejects_non_unit_input_and_canonicalizes_sign() {
        assert_eq!(
            UnitQuaternion::from_wxyz([2.0, 0.0, 0.0, 0.0]),
            Err(ValidationError::InvalidRotation)
        );
        let positive = UnitQuaternion::from_wxyz([1.0, 0.0, 0.0, 0.0]).unwrap();
        let negative = UnitQuaternion::from_wxyz([-1.0, 0.0, 0.0, 0.0]).unwrap();
        assert_eq!(positive, negative);

        let accepted_near_unit = UnitQuaternion::from_wxyz([1.0 + 1.0e-12, 0.0, 0.0, 0.0])
            .unwrap()
            .components_wxyz();
        assert_eq!(accepted_near_unit, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn quaternion_rotation_composition_and_inverse_are_consistent() {
        let quarter_turn = UnitQuaternion::from_rotation_vector(
            Vector3::new(0.0, 0.0, core::f64::consts::FRAC_PI_2).unwrap(),
        )
        .unwrap();
        let x = Vector3::new(1.0, 0.0, 0.0).unwrap();
        let y = quarter_turn.rotate_vector(x);
        assert_close(y.x(), 0.0, 1.0e-12);
        assert_close(y.y(), 1.0, 1.0e-12);
        let recovered = quarter_turn.inverse().rotate_vector(y);
        assert_close(recovered.x(), 1.0, 1.0e-12);
        assert_close(recovered.y(), 0.0, 1.0e-12);
    }

    #[test]
    fn slerp_has_exact_endpoints_and_expected_midpoint() {
        let start = UnitQuaternion::IDENTITY;
        let end = UnitQuaternion::from_rotation_vector(
            Vector3::new(0.0, 0.0, core::f64::consts::PI).unwrap(),
        )
        .unwrap();
        assert_eq!(start.slerp(end, 0.0).unwrap(), start);
        assert_eq!(start.slerp(end, 1.0).unwrap(), end);
        let midpoint = start.slerp(end, 0.5).unwrap();
        let rotated = midpoint.rotate_vector(Vector3::new(1.0, 0.0, 0.0).unwrap());
        assert_close(rotated.x(), 0.0, 1.0e-12);
        assert_close(rotated.y(), 1.0, 1.0e-12);
    }
}
