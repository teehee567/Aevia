//! Allocation-free scalar transcendental functions for host and firmware builds.
//!
//! Keeping these wrappers private lets the estimator use the same call sites for
//! `f32` and `f64` without pulling the software-float enclosure implementation
//! into the embedded dependency graph.

pub(crate) trait ScalarMath: Copy {
    #[cfg(test)]
    fn asin(self) -> Self;
    fn atan2(self, other: Self) -> Self;
    fn cos(self) -> Self;
    fn sin(self) -> Self;
    fn sqrt(self) -> Self;
}

impl ScalarMath for f32 {
    #[cfg(test)]
    fn asin(self) -> Self {
        libm::asinf(self)
    }

    fn atan2(self, other: Self) -> Self {
        libm::atan2f(self, other)
    }

    fn cos(self) -> Self {
        libm::cosf(self)
    }

    fn sin(self) -> Self {
        libm::sinf(self)
    }

    fn sqrt(self) -> Self {
        libm::sqrtf(self)
    }
}

impl ScalarMath for f64 {
    #[cfg(test)]
    fn asin(self) -> Self {
        libm::asin(self)
    }

    fn atan2(self, other: Self) -> Self {
        libm::atan2(self, other)
    }

    fn cos(self) -> Self {
        libm::cos(self)
    }

    fn sin(self) -> Self {
        libm::sin(self)
    }

    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }
}

#[cfg(test)]
pub(crate) fn asin<T: ScalarMath>(value: T) -> T {
    value.asin()
}

pub(crate) fn atan2<T: ScalarMath>(y: T, x: T) -> T {
    y.atan2(x)
}

pub(crate) fn cos<T: ScalarMath>(value: T) -> T {
    value.cos()
}

pub(crate) fn sin<T: ScalarMath>(value: T) -> T {
    value.sin()
}

pub(crate) fn sin_cos<T: ScalarMath>(value: T) -> (T, T) {
    (value.sin(), value.cos())
}

pub(crate) fn sqrt<T: ScalarMath>(value: T) -> T {
    value.sqrt()
}
