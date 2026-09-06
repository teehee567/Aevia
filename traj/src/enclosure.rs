//! Allocator-free outward interval arithmetic for the production root graph.
//!
//! Binary64 production arithmetic requires IEEE-754 round-to-nearest with
//! gradual underflow and no implicit contraction. Every operation expands
//! outwards; pinned `fpmath` elementary functions use their documented error
//! bounds. Target qualification remains separate from this source contract.
//! Software scalar adapters are retained only as host regression oracles.

#![allow(dead_code)]

use core::ops::{Add, Div, Mul, Neg, Sub};

#[cfg(test)]
use fpmath::{SoftF32, SoftF64};

// fpmath documents < 0.5 ULP for sqrt and < 1 ULP for sin/cos. Two representable
// steps deliberately exceed either documented error bound.
const TRANSCENDENTAL_EXPANSION_ULPS: u8 = 2;
const SO3_SMALL_ANGLE_SQUARED: f64 = 1.0e-4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum EnclosureError {
    NonFinite,
    InvalidBounds,
    DivisionThroughZero,
    Domain,
    Unbounded,
}

pub(crate) trait EnclosureScalar:
    Copy
    + PartialEq
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
{
    fn from_f64_enclosing(value: f64) -> Result<(Self, Self), EnclosureError>;
    fn zero() -> Self;
    fn one() -> Self;
    fn neg_one() -> Self;
    fn to_f64(self) -> f64;
    fn is_finite(self) -> bool;
    fn next_down(self) -> Self;
    fn next_up(self) -> Self;
    fn sqrt(self) -> Self;
    fn sin_cos(self) -> (Self, Self);
}

impl EnclosureScalar for f64 {
    fn from_f64_enclosing(value: f64) -> Result<(Self, Self), EnclosureError> {
        if value.is_finite() {
            Ok((value, value))
        } else {
            Err(EnclosureError::NonFinite)
        }
    }
    fn zero() -> Self {
        0.0
    }
    fn one() -> Self {
        1.0
    }
    fn neg_one() -> Self {
        -1.0
    }
    fn to_f64(self) -> f64 {
        self
    }
    fn is_finite(self) -> bool {
        self.is_finite()
    }
    fn next_down(self) -> Self {
        self.next_down()
    }
    fn next_up(self) -> Self {
        self.next_up()
    }
    fn sqrt(self) -> Self {
        fpmath::sqrt(self)
    }
    fn sin_cos(self) -> (Self, Self) {
        (fpmath::sin(self), fpmath::cos(self))
    }
}

#[cfg(test)]
impl EnclosureScalar for SoftF32 {
    fn from_f64_enclosing(value: f64) -> Result<(Self, Self), EnclosureError> {
        let (lower, upper) = f32_enclosure_bits_from_f64(value)?;
        Ok((Self::from_bits(lower), Self::from_bits(upper)))
    }

    fn zero() -> Self {
        Self::from_bits(0)
    }

    fn one() -> Self {
        Self::from_host(1.0)
    }

    fn neg_one() -> Self {
        Self::from_host(-1.0)
    }

    fn to_f64(self) -> f64 {
        f64::from(self.to_host())
    }

    fn is_finite(self) -> bool {
        self.is_finite()
    }

    fn next_down(self) -> Self {
        if self.is_nan() || self == Self::neg_infinity() {
            return self;
        }
        if self == Self::zero() {
            return Self::from_bits(0x8000_0001);
        }
        let bits = self.to_bits();
        if bits & 0x8000_0000 == 0 {
            Self::from_bits(bits - 1)
        } else {
            Self::from_bits(bits + 1)
        }
    }

    fn next_up(self) -> Self {
        if self.is_nan() || self == Self::infinity() {
            return self;
        }
        if self == Self::zero() {
            return Self::from_bits(1);
        }
        let bits = self.to_bits();
        if bits & 0x8000_0000 == 0 {
            Self::from_bits(bits + 1)
        } else {
            Self::from_bits(bits - 1)
        }
    }

    fn sqrt(self) -> Self {
        fpmath::sqrt(self)
    }

    fn sin_cos(self) -> (Self, Self) {
        fpmath::sin_cos(self)
    }
}

#[cfg(test)]
impl EnclosureScalar for SoftF64 {
    fn from_f64_enclosing(value: f64) -> Result<(Self, Self), EnclosureError> {
        if !value.is_finite() {
            return Err(EnclosureError::NonFinite);
        }
        let exact = Self::from_bits(value.to_bits());
        Ok((exact, exact))
    }

    fn zero() -> Self {
        Self::from_bits(0)
    }

    fn one() -> Self {
        Self::from_host(1.0)
    }

    fn neg_one() -> Self {
        Self::from_host(-1.0)
    }

    fn to_f64(self) -> f64 {
        self.to_host()
    }

    fn is_finite(self) -> bool {
        self.is_finite()
    }

    fn next_down(self) -> Self {
        if self.is_nan() || self == Self::neg_infinity() {
            return self;
        }
        if self == Self::zero() {
            return Self::from_bits(0x8000_0000_0000_0001);
        }
        let bits = self.to_bits();
        if bits & 0x8000_0000_0000_0000 == 0 {
            Self::from_bits(bits - 1)
        } else {
            Self::from_bits(bits + 1)
        }
    }

    fn next_up(self) -> Self {
        if self.is_nan() || self == Self::infinity() {
            return self;
        }
        if self == Self::zero() {
            return Self::from_bits(1);
        }
        let bits = self.to_bits();
        if bits & 0x8000_0000_0000_0000 == 0 {
            Self::from_bits(bits + 1)
        } else {
            Self::from_bits(bits - 1)
        }
    }

    fn sqrt(self) -> Self {
        fpmath::sqrt(self)
    }

    fn sin_cos(self) -> (Self, Self) {
        fpmath::sin_cos(self)
    }
}

/// Converts a binary64 input to adjacent binary32 bounds using integer bit
/// arithmetic only. This avoids silently reintroducing target-native floating
/// conversion into the deterministic SoftF32 backend.
fn f32_enclosure_bits_from_f64(value: f64) -> Result<(u32, u32), EnclosureError> {
    const F64_EXPONENT_MASK: u64 = 0x7ff0_0000_0000_0000;
    const F64_FRACTION_MASK: u64 = 0x000f_ffff_ffff_ffff;
    const F32_MAX_AS_F64_BITS: u64 = 0x47ef_ffff_e000_0000;

    let bits = value.to_bits();
    let sign = bits >> 63;
    let magnitude_bits = bits & !0x8000_0000_0000_0000;
    if magnitude_bits & F64_EXPONENT_MASK == F64_EXPONENT_MASK {
        return Err(EnclosureError::NonFinite);
    }
    if magnitude_bits == 0 {
        let zero = (sign as u32) << 31;
        return Ok((zero, zero));
    }
    if magnitude_bits > F32_MAX_AS_F64_BITS {
        return Err(EnclosureError::Unbounded);
    }

    let exponent_bits = ((magnitude_bits & F64_EXPONENT_MASK) >> 52) as i32;
    let fraction = magnitude_bits & F64_FRACTION_MASK;
    let (mantissa, exponent_two, binary_exponent) = if exponent_bits == 0 {
        let leading = 63 - fraction.leading_zeros() as i32;
        (fraction, -1_074, leading - 1_074)
    } else {
        (
            (1_u64 << 52) | fraction,
            exponent_bits - 1_023 - 52,
            exponent_bits - 1_023,
        )
    };

    let (positive_lower, positive_upper) = if binary_exponent < -126 {
        let (units, has_remainder) = shifted_floor(mantissa, exponent_two + 149);
        debug_assert!(units < (1_u64 << 23));
        let lower = units as u32;
        let upper = lower + u32::from(has_remainder);
        (lower, upper)
    } else {
        debug_assert!(binary_exponent <= 127);
        let target_shift = exponent_two - (binary_exponent - 23);
        let (significand, has_remainder) = shifted_floor(mantissa, target_shift);
        debug_assert!((1_u64 << 23) <= significand && significand < (1_u64 << 24));
        let encoded_exponent = ((binary_exponent + 127) as u32) << 23;
        let lower = encoded_exponent | (significand as u32 - (1 << 23));
        let upper = lower + u32::from(has_remainder);
        (lower, upper)
    };

    if sign == 0 {
        Ok((positive_lower, positive_upper))
    } else {
        Ok((positive_upper | 0x8000_0000, positive_lower | 0x8000_0000))
    }
}

fn shifted_floor(mantissa: u64, binary_shift: i32) -> (u64, bool) {
    if binary_shift >= 0 {
        return (mantissa << binary_shift as u32, false);
    }
    let right = binary_shift.unsigned_abs();
    if right >= 64 {
        return (0, mantissa != 0);
    }
    let discarded_mask = (1_u64 << right) - 1;
    (mantissa >> right, mantissa & discarded_mask != 0)
}

/// Closed interval whose every arithmetic result is expanded outwards.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EnclosureV1<S: EnclosureScalar> {
    lower: S,
    upper: S,
}

#[cfg(test)]
pub(crate) type LiveEnclosureV1 = EnclosureV1<SoftF32>;
#[cfg(test)]
pub(crate) type OfflineEnclosureV1 = EnclosureV1<SoftF64>;
pub(crate) type NativeEnclosureV1 = EnclosureV1<f64>;

impl<S: EnclosureScalar> EnclosureV1<S> {
    fn new(lower: S, upper: S) -> Result<Self, EnclosureError> {
        if !lower.is_finite() || !upper.is_finite() {
            return Err(EnclosureError::Unbounded);
        }
        if lower > upper {
            return Err(EnclosureError::InvalidBounds);
        }
        Ok(Self { lower, upper })
    }

    pub(crate) fn point_f64(value: f64) -> Result<Self, EnclosureError> {
        let (lower, upper) = S::from_f64_enclosing(value)?;
        Self::new(lower, upper)
    }

    pub(crate) fn from_f64_bounds(lower: f64, upper: f64) -> Result<Self, EnclosureError> {
        if lower > upper {
            return Err(EnclosureError::InvalidBounds);
        }
        let (lower, _) = S::from_f64_enclosing(lower)?;
        let (_, upper) = S::from_f64_enclosing(upper)?;
        Self::new(lower, upper)
    }

    fn exact(value: S) -> Result<Self, EnclosureError> {
        Self::new(value, value)
    }

    pub(crate) fn zero() -> Self {
        Self {
            lower: S::zero(),
            upper: S::zero(),
        }
    }

    pub(crate) fn one() -> Self {
        Self {
            lower: S::one(),
            upper: S::one(),
        }
    }

    pub(crate) fn lower_f64(self) -> f64 {
        self.lower.to_f64()
    }

    pub(crate) fn upper_f64(self) -> f64 {
        self.upper.to_f64()
    }

    pub(crate) fn contains_f64(self, value: f64) -> bool {
        value.is_finite() && self.lower_f64() <= value && value <= self.upper_f64()
    }

    pub(crate) fn contains_zero(self) -> bool {
        self.lower <= S::zero() && self.upper >= S::zero()
    }

    fn widen(mut self, ulps: u8) -> Result<Self, EnclosureError> {
        let mut remaining = ulps;
        while remaining > 0 {
            self.lower = self.lower.next_down();
            self.upper = self.upper.next_up();
            remaining -= 1;
        }
        Self::new(self.lower, self.upper)
    }

    pub(crate) fn neg(self) -> Result<Self, EnclosureError> {
        Self::new(-self.upper, -self.lower)
    }

    pub(crate) fn add(self, rhs: Self) -> Result<Self, EnclosureError> {
        if rhs.lower == S::zero() && rhs.upper == S::zero() {
            return Ok(self);
        }
        if self.lower == S::zero() && self.upper == S::zero() {
            return Ok(rhs);
        }
        Self::new(
            (self.lower + rhs.lower).next_down(),
            (self.upper + rhs.upper).next_up(),
        )
    }

    pub(crate) fn sub(self, rhs: Self) -> Result<Self, EnclosureError> {
        Self::new(
            (self.lower - rhs.upper).next_down(),
            (self.upper - rhs.lower).next_up(),
        )
    }

    pub(crate) fn mul(self, rhs: Self) -> Result<Self, EnclosureError> {
        if (rhs.lower == S::zero() && rhs.upper == S::zero())
            || (self.lower == S::zero() && self.upper == S::zero())
        {
            return Ok(Self::zero());
        }
        let products = [
            self.lower * rhs.lower,
            self.lower * rhs.upper,
            self.upper * rhs.lower,
            self.upper * rhs.upper,
        ];
        let mut lower = products[0];
        let mut upper = products[0];
        for value in products.into_iter().skip(1) {
            if value < lower {
                lower = value;
            }
            if value > upper {
                upper = value;
            }
        }
        Self::new(lower.next_down(), upper.next_up())
    }

    pub(crate) fn scale_f64(self, rhs: f64) -> Result<Self, EnclosureError> {
        self.mul(Self::point_f64(rhs)?)
    }

    pub(crate) fn div(self, rhs: Self) -> Result<Self, EnclosureError> {
        if rhs.contains_zero() {
            return Err(EnclosureError::DivisionThroughZero);
        }
        let quotients = [
            self.lower / rhs.lower,
            self.lower / rhs.upper,
            self.upper / rhs.lower,
            self.upper / rhs.upper,
        ];
        let mut lower = quotients[0];
        let mut upper = quotients[0];
        for value in quotients.into_iter().skip(1) {
            if value < lower {
                lower = value;
            }
            if value > upper {
                upper = value;
            }
        }
        Self::new(lower.next_down(), upper.next_up())
    }

    pub(crate) fn square(self) -> Result<Self, EnclosureError> {
        let lower_square = self.lower * self.lower;
        let upper_square = self.upper * self.upper;
        let maximum = if lower_square > upper_square {
            lower_square
        } else {
            upper_square
        };
        let minimum = if self.contains_zero() {
            S::zero()
        } else if lower_square < upper_square {
            lower_square
        } else {
            upper_square
        };
        let lower = if minimum == S::zero() {
            S::zero()
        } else {
            minimum.next_down()
        };
        Self::new(lower, maximum.next_up())
    }

    pub(crate) fn abs(self) -> Result<Self, EnclosureError> {
        if self.lower >= S::zero() {
            return Ok(self);
        }
        if self.upper <= S::zero() {
            return self.neg();
        }
        let negative_lower = -self.lower;
        let upper = if negative_lower > self.upper {
            negative_lower
        } else {
            self.upper
        };
        Self::new(S::zero(), upper)
    }

    pub(crate) fn sqrt(self) -> Result<Self, EnclosureError> {
        if self.lower < S::zero() {
            return Err(EnclosureError::Domain);
        }
        let lower_value = self.lower.sqrt();
        let upper_value = self.upper.sqrt();
        if !lower_value.is_finite() || !upper_value.is_finite() {
            return Err(EnclosureError::Unbounded);
        }
        let expanded = Self::new(lower_value, upper_value)?.widen(TRANSCENDENTAL_EXPANSION_ULPS)?;
        Self::new(
            if expanded.lower < S::zero() {
                S::zero()
            } else {
                expanded.lower
            },
            expanded.upper,
        )
    }

    /// Endpoint evaluations plus every enclosed multiple of pi/2. Extrema
    /// indices use an outward interval quotient, so uncertain range reduction
    /// can only widen the answer. Wide or unresolvable reductions use [-1,1].
    pub(crate) fn sin_cos(self) -> Result<(Self, Self), EnclosureError> {
        let unit = Self::new(S::neg_one(), S::one())?;
        let half_pi =
            Self::point_f64(core::f64::consts::FRAC_PI_2)?.widen(TRANSCENDENTAL_EXPANSION_ULPS)?;
        let indices = self.div(half_pi)?;
        let first = fpmath::ceil(indices.lower_f64());
        let last = fpmath::floor(indices.upper_f64());
        if last - first >= 4.0
            || first.abs() > 4_503_599_627_370_496.0
            || last.abs() > 4_503_599_627_370_496.0
        {
            return Ok((unit, unit));
        }

        let (sin_lower, cos_lower) = self.lower.sin_cos();
        let (sin_upper, cos_upper) = self.upper.sin_cos();
        let mut sin = interval_from_approximate_endpoints(sin_lower, sin_upper)?;
        let mut cos = interval_from_approximate_endpoints(cos_lower, cos_upper)?;

        for index in (first as i64)..=(last as i64) {
            match index.rem_euclid(4) {
                0 => cos.upper = S::one(),
                1 => sin.upper = S::one(),
                2 => cos.lower = S::neg_one(),
                _ => sin.lower = S::neg_one(),
            }
        }
        Ok((
            Self::new(sin.lower, sin.upper)?,
            Self::new(cos.lower, cos.upper)?,
        ))
    }

    fn overlaps(self, rhs: Self) -> bool {
        self.lower <= rhs.upper && self.upper >= rhs.lower
    }

    fn symmetric_magnitude(self) -> Result<Self, EnclosureError> {
        let magnitude = self.abs()?;
        Self::new(-magnitude.upper, magnitude.upper)
    }

    pub(crate) fn dot(left: [Self; 3], right: [Self; 3]) -> Result<Self, EnclosureError> {
        left[0]
            .mul(right[0])?
            .add(left[1].mul(right[1])?)?
            .add(left[2].mul(right[2])?)
    }

    pub(crate) fn cross(left: [Self; 3], right: [Self; 3]) -> Result<[Self; 3], EnclosureError> {
        Ok([
            left[1].mul(right[2])?.sub(left[2].mul(right[1])?)?,
            left[2].mul(right[0])?.sub(left[0].mul(right[2])?)?,
            left[0].mul(right[1])?.sub(left[1].mul(right[0])?)?,
        ])
    }

    pub(crate) fn norm(vector: [Self; 3]) -> Result<Self, EnclosureError> {
        vector[0]
            .square()?
            .add(vector[1].square()?)?
            .add(vector[2].square()?)?
            .sqrt()
    }
}

fn interval_from_approximate_endpoints<S: EnclosureScalar>(
    left: S,
    right: S,
) -> Result<EnclosureV1<S>, EnclosureError> {
    let (lower, upper) = if left <= right {
        (left, right)
    } else {
        (right, left)
    };
    EnclosureV1::new(lower, upper)?.widen(TRANSCENDENTAL_EXPANSION_ULPS)
}

/// Value and first two parameter derivatives, all as outward intervals.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EnclosedJet2<S: EnclosureScalar> {
    pub value: EnclosureV1<S>,
    pub first: EnclosureV1<S>,
    pub second: EnclosureV1<S>,
}

impl<S: EnclosureScalar> EnclosedJet2<S> {
    pub(crate) fn constant(value: EnclosureV1<S>) -> Self {
        Self {
            value,
            first: EnclosureV1::zero(),
            second: EnclosureV1::zero(),
        }
    }

    pub(crate) fn independent(value: EnclosureV1<S>) -> Self {
        Self {
            value,
            first: EnclosureV1::one(),
            second: EnclosureV1::zero(),
        }
    }

    pub(crate) fn add(self, rhs: Self) -> Result<Self, EnclosureError> {
        Ok(Self {
            value: self.value.add(rhs.value)?,
            first: self.first.add(rhs.first)?,
            second: self.second.add(rhs.second)?,
        })
    }

    pub(crate) fn sub(self, rhs: Self) -> Result<Self, EnclosureError> {
        Ok(Self {
            value: self.value.sub(rhs.value)?,
            first: self.first.sub(rhs.first)?,
            second: self.second.sub(rhs.second)?,
        })
    }

    pub(crate) fn mul(self, rhs: Self) -> Result<Self, EnclosureError> {
        let two = EnclosureV1::point_f64(2.0)?;
        Ok(Self {
            value: self.value.mul(rhs.value)?,
            first: self.first.mul(rhs.value)?.add(self.value.mul(rhs.first)?)?,
            second: self
                .second
                .mul(rhs.value)?
                .add(two.mul(self.first.mul(rhs.first)?)?)?
                .add(self.value.mul(rhs.second)?)?,
        })
    }

    pub(crate) fn scale_f64(self, rhs: f64) -> Result<Self, EnclosureError> {
        self.mul(Self::constant(EnclosureV1::point_f64(rhs)?))
    }

    pub(crate) fn square(self) -> Result<Self, EnclosureError> {
        let two = EnclosureV1::point_f64(2.0)?;
        Ok(Self {
            value: self.value.square()?,
            first: two.mul(self.value.mul(self.first)?)?,
            second: two.mul(self.first.square()?.add(self.value.mul(self.second)?)?)?,
        })
    }

    pub(crate) fn cube(self) -> Result<Self, EnclosureError> {
        self.square()?.mul(self)
    }

    pub(crate) fn reciprocal(self) -> Result<Self, EnclosureError> {
        let one = EnclosureV1::one();
        let two = EnclosureV1::point_f64(2.0)?;
        let value = one.div(self.value)?;
        let value_squared = value.square()?;
        let value_cubed = value_squared.mul(value)?;
        Ok(Self {
            value,
            first: self.first.neg()?.mul(value_squared)?,
            second: two
                .mul(self.first.square()?)?
                .mul(value_cubed)?
                .sub(self.second.mul(value_squared)?)?,
        })
    }

    pub(crate) fn div(self, rhs: Self) -> Result<Self, EnclosureError> {
        self.mul(rhs.reciprocal()?)
    }

    pub(crate) fn sqrt(self) -> Result<Self, EnclosureError> {
        let root = self.value.sqrt()?;
        let two = EnclosureV1::point_f64(2.0)?;
        let four = EnclosureV1::point_f64(4.0)?;
        Ok(Self {
            value: root,
            first: self.first.div(two.mul(root)?)?,
            second: self.second.div(two.mul(root)?)?.sub(
                self.first
                    .square()?
                    .div(four.mul(root.square()?.mul(root)?)?)?,
            )?,
        })
    }

    pub(crate) fn sin_cos(self) -> Result<(Self, Self), EnclosureError> {
        let (sin_value, cos_value) = self.value.sin_cos()?;
        let first_squared = self.first.square()?;
        let sin = Self {
            value: sin_value,
            first: cos_value.mul(self.first)?,
            second: sin_value
                .neg()?
                .mul(first_squared)?
                .add(cos_value.mul(self.second)?)?,
        };
        let cos = Self {
            value: cos_value,
            first: sin_value.neg()?.mul(self.first)?,
            second: cos_value
                .neg()?
                .mul(first_squared)?
                .sub(sin_value.mul(self.second)?)?,
        };
        Ok((sin, cos))
    }
}

/// Applies `exp(s [rotation_vector]x)` to one body-frame vector.  The
/// small-angle branch avoids division by the rotation norm and adds explicit
/// Taylor remainders.  Point derivatives use the same operation with
/// `[rotation_vector]x^k vector`, so this routine does not differentiate the
/// interval rotation itself.
pub(crate) fn rodrigues_rotate<S: EnclosureScalar>(
    parameter: EnclosureV1<S>,
    rotation_vector: [f64; 3],
    vector: [f64; 3],
) -> Result<[EnclosureV1<S>; 3], EnclosureError> {
    rodrigues_rotate_enclosed(parameter, rotation_vector, enclose_vector(vector)?)
}

pub(crate) fn rodrigues_rotate_enclosed<S: EnclosureScalar>(
    parameter: EnclosureV1<S>,
    rotation_vector: [f64; 3],
    source: [EnclosureV1<S>; 3],
) -> Result<[EnclosureV1<S>; 3], EnclosureError> {
    if parameter.lower_f64() < 0.0 || parameter.upper_f64() > 1.0 {
        return Err(EnclosureError::Domain);
    }
    let phi = enclose_vector(rotation_vector)?;
    if rotation_vector == [0.0; 3] {
        return Ok(source);
    }

    let phi_cross_source = EnclosureV1::cross(phi, source)?;
    let phi_cross_twice = EnclosureV1::cross(phi, phi_cross_source)?;
    // Preserve the dependency in ||phi||^2. Generic interval multiplication
    // of an exact zero component by itself would otherwise extend below zero.
    let phi_squared = phi[0]
        .square()?
        .add(phi[1].square()?)?
        .add(phi[2].square()?)?;
    let parameter_squared = parameter.square()?;
    let angle_squared = parameter_squared.mul(phi_squared)?;

    let (sine_over_norm, one_minus_cosine_over_norm_squared) =
        if angle_squared.upper_f64() <= SO3_SMALL_ANGLE_SQUARED {
            // sinc(x) = 1 - x^2/3! + x^4/5! - x^6/7! + R8,
            // |R8| <= |x|^8/9!.  cosc(x) has the analogous 10! remainder.
            let z = angle_squared;
            let z2 = z.square()?;
            let z3 = z2.mul(z)?;
            let z4 = z2.square()?;
            // Form exact rational coefficients inside the enclosure. A binary
            // `f64` approximation of 1/n would not satisfy the SoftF64 proof
            // obligation by itself.
            let reciprocal =
                |denominator: f64| EnclosureV1::one().div(EnclosureV1::point_f64(denominator)?);
            let sinc = EnclosureV1::one()
                .sub(z.mul(reciprocal(6.0)?)?)?
                .add(z2.mul(reciprocal(120.0)?)?)?
                .sub(z3.mul(reciprocal(5_040.0)?)?)?
                .add(z4.mul(reciprocal(362_880.0)?)?.symmetric_magnitude()?)?;
            let cosc = EnclosureV1::point_f64(0.5)?
                .sub(z.mul(reciprocal(24.0)?)?)?
                .add(z2.mul(reciprocal(720.0)?)?)?
                .sub(z3.mul(reciprocal(40_320.0)?)?)?
                .add(z4.mul(reciprocal(3_628_800.0)?)?.symmetric_magnitude()?)?;
            (parameter.mul(sinc)?, parameter_squared.mul(cosc)?)
        } else {
            let phi_norm = phi_squared.sqrt()?;
            let angle = parameter.mul(phi_norm)?;
            let (sin_angle, cos_angle) = angle.sin_cos()?;
            (
                sin_angle.div(phi_norm)?,
                EnclosureV1::one().sub(cos_angle)?.div(phi_squared)?,
            )
        };

    let mut rotated = source;
    for axis in 0..3 {
        rotated[axis] = rotated[axis]
            .add(phi_cross_source[axis].mul(sine_over_norm)?)?
            .add(phi_cross_twice[axis].mul(one_minus_cosine_over_norm_squared)?)?;
    }
    Ok(rotated)
}

pub(crate) fn matrix3_mul_vector<S: EnclosureScalar>(
    matrix: [[f64; 3]; 3],
    vector: [EnclosureV1<S>; 3],
) -> Result<[EnclosureV1<S>; 3], EnclosureError> {
    let rows = [
        enclose_vector(matrix[0])?,
        enclose_vector(matrix[1])?,
        enclose_vector(matrix[2])?,
    ];
    Ok([
        EnclosureV1::dot(rows[0], vector)?,
        EnclosureV1::dot(rows[1], vector)?,
        EnclosureV1::dot(rows[2], vector)?,
    ])
}

fn enclose_vector<S: EnclosureScalar>(
    vector: [f64; 3],
) -> Result<[EnclosureV1<S>; 3], EnclosureError> {
    Ok([
        EnclosureV1::point_f64(vector[0])?,
        EnclosureV1::point_f64(vector[1])?,
        EnclosureV1::point_f64(vector[2])?,
    ])
}

/// Bowring ellipsoid-up expression evaluated as an interval second-order jet.
/// `atan2` is intentionally eliminated: sine and cosine of each angle are the
/// normalized numerator/denominator pair, avoiding a branch cut and an
/// additional transcendental error contract.
pub(crate) fn ellipsoid_up_jet<S: EnclosureScalar>(
    position: [EnclosedJet2<S>; 3],
    semi_major_axis_m: f64,
    inverse_flattening: f64,
) -> Result<[EnclosedJet2<S>; 3], EnclosureError> {
    if semi_major_axis_m <= 0.0 || inverse_flattening <= 1.0 {
        return Err(EnclosureError::Domain);
    }
    let one = EnclosedJet2::constant(EnclosureV1::one());
    let two = EnclosedJet2::constant(EnclosureV1::point_f64(2.0)?);
    let a = EnclosedJet2::constant(EnclosureV1::point_f64(semi_major_axis_m)?);
    let inverse_f = EnclosedJet2::constant(EnclosureV1::point_f64(inverse_flattening)?);
    let flattening = one.div(inverse_f)?;
    let b_over_a = one.sub(flattening)?;
    let b = a.mul(b_over_a)?;
    let e2 = flattening.mul(two.sub(flattening)?)?;
    let ep2 = e2.div(one.sub(e2)?)?;

    let horizontal = position[0].square()?.add(position[1].square()?)?.sqrt()?;
    let sin_longitude = position[1].div(horizontal)?;
    let cos_longitude = position[0].div(horizontal)?;

    // Scaling both atan2 arguments by the same positive a*b keeps the jet
    // norm near unity and prevents SoftF32 second-derivative intermediates
    // from overflowing at terrestrial ECEF magnitudes.
    let theta_y = position[2].div(b)?;
    let theta_x = horizontal.div(a)?;
    let theta_norm = theta_y.square()?.add(theta_x.square()?)?.sqrt()?;
    let sin_theta = theta_y.div(theta_norm)?;
    let cos_theta = theta_x.div(theta_norm)?;

    // Apply the same conditioning to Bowring's latitude pair by dividing both
    // arguments by a.
    let latitude_y = position[2]
        .div(a)?
        .add(ep2.mul(b_over_a)?.mul(sin_theta.cube()?)?)?;
    let latitude_x = horizontal.div(a)?.sub(e2.mul(cos_theta.cube()?)?)?;
    let latitude_norm = latitude_y.square()?.add(latitude_x.square()?)?.sqrt()?;
    let sin_latitude = latitude_y.div(latitude_norm)?;
    let cos_latitude = latitude_x.div(latitude_norm)?;

    Ok([
        cos_latitude.mul(cos_longitude)?,
        cos_latitude.mul(sin_longitude)?,
        sin_latitude,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_scalar_keeps_gradual_underflow_inside_its_bounds() {
        let tiny = NativeEnclosureV1::point_f64(1.0e-200).unwrap();
        let underflow = tiny.mul(tiny).unwrap();
        assert!(underflow.lower_f64() <= 0.0);
        assert!(underflow.upper_f64() > 0.0);
        let negative = tiny.neg().unwrap().mul(tiny).unwrap();
        assert!(negative.lower_f64() < 0.0);
        assert!(negative.upper_f64() >= 0.0);
        // sqrt(2^-1074) = 2^-537 exactly, including at the smallest input.
        let root = NativeEnclosureV1::point_f64(f64::from_bits(1))
            .unwrap()
            .sqrt()
            .unwrap();
        let exact = f64::from_bits(486_u64 << 52);
        assert!(root.lower_f64() < exact && root.upper_f64() > exact);
    }

    fn assert_contains(interval: LiveEnclosureV1, value: f64) {
        assert!(
            interval.contains_f64(value),
            "[{:.17e}, {:.17e}] does not contain {value:.17e}",
            interval.lower_f64(),
            interval.upper_f64()
        );
    }

    fn host_cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
        [
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        ]
    }

    fn host_rodrigues(parameter: f64, phi: [f64; 3], vector: [f64; 3]) -> [f64; 3] {
        let norm_squared = phi.iter().map(|value| value * value).sum::<f64>();
        if norm_squared == 0.0 {
            return vector;
        }
        let norm = norm_squared.sqrt();
        let angle = parameter * norm;
        let first_scale = angle.sin() / norm;
        let second_scale = (1.0 - angle.cos()) / norm_squared;
        let first = host_cross(phi, vector);
        let second = host_cross(phi, first);
        [
            vector[0] + first_scale * first[0] + second_scale * second[0],
            vector[1] + first_scale * first[1] + second_scale * second[1],
            vector[2] + first_scale * first[2] + second_scale * second[2],
        ]
    }

    fn host_bowring_up(position: [f64; 3]) -> [f64; 3] {
        let [x, y, z] = position;
        let a = 6_378_137.0;
        let flattening = 1.0 / 298.257_223_563;
        let b = a * (1.0 - flattening);
        let e2 = flattening * (2.0 - flattening);
        let ep2 = (a * a - b * b) / (b * b);
        let horizontal = x.hypot(y);
        let longitude = y.atan2(x);
        let theta = (z * a).atan2(horizontal * b);
        let (sin_theta, cos_theta) = theta.sin_cos();
        let latitude =
            (z + ep2 * b * sin_theta.powi(3)).atan2(horizontal - e2 * a * cos_theta.powi(3));
        let (sin_latitude, cos_latitude) = latitude.sin_cos();
        let (sin_longitude, cos_longitude) = longitude.sin_cos();
        [
            cos_latitude * cos_longitude,
            cos_latitude * sin_longitude,
            sin_latitude,
        ]
    }

    fn affine_jet(intercept: f64, slope: f64, lower: f64, upper: f64) -> EnclosedJet2<SoftF32> {
        let at_lower = intercept + slope * lower;
        let at_upper = intercept + slope * upper;
        EnclosedJet2 {
            value: LiveEnclosureV1::from_f64_bounds(at_lower.min(at_upper), at_lower.max(at_upper))
                .unwrap(),
            first: LiveEnclosureV1::point_f64(slope).unwrap(),
            second: LiveEnclosureV1::point_f64(0.0).unwrap(),
        }
    }

    #[test]
    fn soft_f32_conversion_and_arithmetic_enclose_f64_oracles() {
        let values = [
            -16_777_217.0,
            -core::f64::consts::PI,
            -1.0e-30,
            -0.0,
            f64::from(f32::from_bits(1)),
            1.0 / 3.0,
            core::f64::consts::PI,
            16_777_217.0,
        ];
        for value in values {
            assert_contains(LiveEnclosureV1::point_f64(value).unwrap(), value);
        }

        let pairs = [
            (-10.25, 3.5),
            (-1.0e-20, 3.0e-20),
            (1.0 / 3.0, 7.0 / 11.0),
            (65_537.0, 0.125),
        ];
        for (left, right) in pairs {
            let left_interval = LiveEnclosureV1::point_f64(left).unwrap();
            let right_interval = LiveEnclosureV1::point_f64(right).unwrap();
            assert_contains(left_interval.add(right_interval).unwrap(), left + right);
            assert_contains(left_interval.sub(right_interval).unwrap(), left - right);
            assert_contains(left_interval.mul(right_interval).unwrap(), left * right);
            assert_contains(left_interval.div(right_interval).unwrap(), left / right);
        }
    }

    #[test]
    fn division_domain_overflow_and_sqrt_domain_fail_closed() {
        let numerator = LiveEnclosureV1::point_f64(1.0).unwrap();
        let through_zero = LiveEnclosureV1::from_f64_bounds(-1.0, 1.0).unwrap();
        assert_eq!(
            numerator.div(through_zero),
            Err(EnclosureError::DivisionThroughZero)
        );
        assert_eq!(
            LiveEnclosureV1::from_f64_bounds(-1.0, 4.0).unwrap().sqrt(),
            Err(EnclosureError::Domain)
        );
        assert_eq!(
            LiveEnclosureV1::point_f64(f64::MAX),
            Err(EnclosureError::Unbounded)
        );

        let f64_subnormal = f64::from_bits(1);
        assert_contains(
            LiveEnclosureV1::point_f64(f64_subnormal).unwrap(),
            f64_subnormal,
        );
        assert_eq!(
            <SoftF32 as EnclosureScalar>::next_down(SoftF32::from_bits(0)).to_bits(),
            0x8000_0001
        );
        assert_eq!(
            <SoftF32 as EnclosureScalar>::next_up(SoftF32::from_bits(0x8000_0000)).to_bits(),
            1
        );
    }

    #[test]
    fn multiplication_encloses_all_sign_quadrants() {
        let cases = [
            ((-3.0, -2.0), (-5.0, -4.0)),
            ((-3.0, -2.0), (4.0, 5.0)),
            ((-3.0, 2.0), (-5.0, 4.0)),
            ((2.0, 3.0), (-5.0, 4.0)),
        ];
        for ((a0, a1), (b0, b1)) in cases {
            let a = LiveEnclosureV1::from_f64_bounds(a0, a1).unwrap();
            let b = LiveEnclosureV1::from_f64_bounds(b0, b1).unwrap();
            let product = a.mul(b).unwrap();
            for left in [a0, (a0 + a1) * 0.5, a1] {
                for right in [b0, (b0 + b1) * 0.5, b1] {
                    assert_contains(product, left * right);
                }
            }
        }
    }

    #[test]
    fn trigonometric_extrema_are_never_inferred_from_endpoint_samples() {
        let around_peak = LiveEnclosureV1::from_f64_bounds(1.4, 1.8).unwrap();
        let (sin, cos) = around_peak.sin_cos().unwrap();
        assert_contains(sin, 1.0);
        for sample in [1.4, 1.5, core::f64::consts::FRAC_PI_2, 1.7, 1.8] {
            assert_contains(sin, sample.sin());
            assert_contains(cos, sample.cos());
        }

        let multiple_periods = LiveEnclosureV1::from_f64_bounds(-10.0, 10.0).unwrap();
        let (sin, cos) = multiple_periods.sin_cos().unwrap();
        assert_eq!((sin.lower_f64(), sin.upper_f64()), (-1.0, 1.0));
        assert_eq!((cos.lower_f64(), cos.upper_f64()), (-1.0, 1.0));
    }

    #[test]
    fn interval_jets_enclose_value_and_first_two_derivatives() {
        let parameter = LiveEnclosureV1::from_f64_bounds(-0.75, 0.9).unwrap();
        let (sin, cos) = EnclosedJet2::independent(parameter).sin_cos().unwrap();
        for sample in [-0.75_f64, -0.5, 0.0, 0.5, 0.9] {
            assert_contains(sin.value, sample.sin());
            assert_contains(sin.first, sample.cos());
            assert_contains(sin.second, -sample.sin());
            assert_contains(cos.value, sample.cos());
            assert_contains(cos.first, -sample.sin());
            assert_contains(cos.second, -sample.cos());
        }
    }

    #[test]
    fn soft_f64_adapter_expands_every_non_exact_operation() {
        let one = OfflineEnclosureV1::point_f64(1.0).unwrap();
        let three = OfflineEnclosureV1::point_f64(3.0).unwrap();
        let third = one.div(three).unwrap();
        assert!(third.lower_f64() < 1.0 / 3.0);
        assert!(third.upper_f64() > 1.0 / 3.0);

        let root_two = OfflineEnclosureV1::point_f64(2.0).unwrap().sqrt().unwrap();
        assert!(root_two.contains_f64(2.0_f64.sqrt()));
        assert!(root_two.lower_f64() < root_two.upper_f64());
    }

    #[test]
    fn vector_norm_and_cross_are_outward_enclosed_without_allocation() {
        let vector = [
            LiveEnclosureV1::point_f64(3.0).unwrap(),
            LiveEnclosureV1::point_f64(4.0).unwrap(),
            LiveEnclosureV1::point_f64(0.0).unwrap(),
        ];
        assert_contains(LiveEnclosureV1::norm(vector).unwrap(), 5.0);
        let axis = [
            LiveEnclosureV1::point_f64(0.0).unwrap(),
            LiveEnclosureV1::point_f64(0.0).unwrap(),
            LiveEnclosureV1::point_f64(1.0).unwrap(),
        ];
        let cross = LiveEnclosureV1::cross(axis, vector).unwrap();
        assert_contains(cross[0], -4.0);
        assert_contains(cross[1], 3.0);
        assert_contains(cross[2], 0.0);
    }

    #[test]
    fn rodrigues_encloses_regular_zero_and_small_angle_cells() {
        let vector = [1.0, -0.25, 0.5];
        for (phi, lower, upper) in [
            ([0.0, 0.0, 0.75], 0.1, 0.9),
            ([0.2, -0.4, 0.7], 0.1, 0.9),
            ([1.0e-5, -2.0e-5, 3.0e-5], 0.0, 1.0),
            ([0.0, 0.0, 0.0], 0.25, 0.75),
        ] {
            let enclosed = rodrigues_rotate::<SoftF32>(
                LiveEnclosureV1::from_f64_bounds(lower, upper).unwrap(),
                phi,
                vector,
            )
            .unwrap();
            for parameter in [lower, (lower + upper) * 0.5, upper] {
                let oracle = host_rodrigues(parameter, phi, vector);
                for axis in 0..3 {
                    assert_contains(enclosed[axis], oracle[axis]);
                }
            }
        }

        assert_eq!(
            rodrigues_rotate::<SoftF32>(
                LiveEnclosureV1::from_f64_bounds(-0.1, 0.5).unwrap(),
                [0.0, 0.0, 0.1],
                vector,
            ),
            Err(EnclosureError::Domain)
        );
    }

    #[test]
    fn ellipsoid_up_graph_encloses_bowring_oracle_and_derivatives() {
        let lower = 0.2;
        let upper = 0.8;
        let intercept = [4_510_000.0, 451_000.0, 4_480_000.0];
        let slope = [300.0, -40.0, 120.0];
        let position = [
            affine_jet(intercept[0], slope[0], lower, upper),
            affine_jet(intercept[1], slope[1], lower, upper),
            affine_jet(intercept[2], slope[2], lower, upper),
        ];
        let up = ellipsoid_up_jet(position, 6_378_137.0, 298.257_223_563).unwrap();
        for parameter in [lower, 0.35, 0.5, 0.65, upper] {
            let at = [
                intercept[0] + slope[0] * parameter,
                intercept[1] + slope[1] * parameter,
                intercept[2] + slope[2] * parameter,
            ];
            let oracle = host_bowring_up(at);
            for axis in 0..3 {
                assert_contains(up[axis].value, oracle[axis]);
            }
        }

        // Finite differences are only a regression oracle here; external MPFR
        // and independent-interval qualification is intentionally still a
        // release gate.
        let parameter = 0.5;
        // A wider symmetric step avoids cancellation in the f64-only second
        // difference (the true curvature is around 1e-9 per s^2).
        let h = 5.0e-2;
        let sample = |s: f64| {
            host_bowring_up([
                intercept[0] + slope[0] * s,
                intercept[1] + slope[1] * s,
                intercept[2] + slope[2] * s,
            ])
        };
        let before = sample(parameter - h);
        let center = sample(parameter);
        let after = sample(parameter + h);
        for axis in 0..3 {
            let first = (after[axis] - before[axis]) / (2.0 * h);
            let second = (after[axis] - 2.0 * center[axis] + before[axis]) / (h * h);
            assert_contains(up[axis].first, first);
            assert_contains(up[axis].second, second);
        }

        let origin_axis = EnclosedJet2::constant(LiveEnclosureV1::point_f64(0.0).unwrap());
        assert!(ellipsoid_up_jet([origin_axis; 3], 6_378_137.0, 298.257_223_563).is_err());
    }
}
