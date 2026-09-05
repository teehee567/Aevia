//! Validated covariance and explicit measurement-uncertainty semantics.

use crate::{error::ValidationError, ids::UncertaintyModelId, math::NonNegativeF64};

/// Maximum supported dimension for a profile's fixed shared-parameter block.
///
/// The bound keeps validation allocation-free and makes resource requirements
/// knowable before an embedded session starts.
pub const MAX_SHARED_PARAMETER_DIMENSION: usize = 32;

/// Supplied numerical uncertainty or an explicit configured model reference.
///
/// A missing receiver covariance must use [`Self::Modeled`]; callers must not
/// substitute zero variance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MeasurementUncertainty<T: Copy> {
    /// Numerical covariance or variance supplied with the semantic value.
    Provided(T),
    /// Uncertainty supplied by the identified immutable model.
    Modeled(UncertaintyModelId),
}

impl<T: Copy> MeasurementUncertainty<T> {
    /// Returns the provided value, or `None` when a configured model applies.
    #[must_use]
    pub const fn provided(self) -> Option<T> {
        match self {
            Self::Provided(value) => Some(value),
            Self::Modeled(_) => None,
        }
    }

    /// Returns the configured model identity, if one supplies uncertainty.
    #[must_use]
    pub const fn model(self) -> Option<UncertaintyModelId> {
        match self {
            Self::Provided(_) => None,
            Self::Modeled(model) => Some(model),
        }
    }
}

/// A finite non-negative scalar variance.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct Variance(NonNegativeF64);

impl Variance {
    /// Validates a variance in the square of the associated unit.
    pub fn new(value: f64) -> Result<Self, ValidationError> {
        NonNegativeF64::new(value)
            .map(Self)
            .map_err(|error| match error {
                ValidationError::NonFinite => ValidationError::NonFinite,
                _ => ValidationError::InvalidCovariance,
            })
    }

    /// Returns the scalar variance.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0.get()
    }
}

/// A symmetric positive-semidefinite three-dimensional covariance.
///
/// Storage order is the upper triangle `(xx, xy, xz, yy, yz, zz)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Covariance3 {
    upper: [f64; 6],
}

impl Covariance3 {
    /// An exact zero covariance, intended only for mathematically exact
    /// transforms and tests—not as a substitute for missing uncertainty.
    pub const ZERO: Self = Self { upper: [0.0; 6] };

    /// Constructs a diagonal covariance.
    pub fn diagonal(xx: f64, yy: f64, zz: f64) -> Result<Self, ValidationError> {
        Self::from_upper_triangle([xx, 0.0, 0.0, yy, 0.0, zz])
    }

    /// Validates an upper-triangle covariance in `(xx, xy, xz, yy, yz, zz)`
    /// order.
    pub fn from_upper_triangle(upper: [f64; 6]) -> Result<Self, ValidationError> {
        if !upper.iter().all(|value| value.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        let candidate = Self { upper };
        if upper[0] < 0.0
            || upper[3] < 0.0
            || upper[5] < 0.0
            || !is_positive_semidefinite(candidate.to_matrix())
        {
            return Err(ValidationError::InvalidCovariance);
        }
        Ok(candidate)
    }

    /// Validates a complete symmetric matrix without silently symmetrizing it.
    pub fn from_matrix(matrix: [[f64; 3]; 3]) -> Result<Self, ValidationError> {
        if !matrix.iter().flatten().all(|value| value.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        if matrix[0][1] != matrix[1][0]
            || matrix[0][2] != matrix[2][0]
            || matrix[1][2] != matrix[2][1]
        {
            return Err(ValidationError::InvalidCovariance);
        }
        Self::from_upper_triangle([
            matrix[0][0],
            matrix[0][1],
            matrix[0][2],
            matrix[1][1],
            matrix[1][2],
            matrix[2][2],
        ])
    }

    /// Returns the compact upper-triangle representation.
    #[must_use]
    pub const fn upper_triangle(self) -> [f64; 6] {
        self.upper
    }

    /// Returns the complete symmetric matrix.
    #[must_use]
    pub const fn to_matrix(self) -> [[f64; 3]; 3] {
        [
            [self.upper[0], self.upper[1], self.upper[2]],
            [self.upper[1], self.upper[3], self.upper[4]],
            [self.upper[2], self.upper[4], self.upper[5]],
        ]
    }

    /// Returns one diagonal variance by axis index, or `None` outside `0..3`.
    #[must_use]
    pub const fn variance(self, axis: usize) -> Option<f64> {
        match axis {
            0 => Some(self.upper[0]),
            1 => Some(self.upper[3]),
            2 => Some(self.upper[5]),
            _ => None,
        }
    }
}

/// A finite 3-by-3 cross-covariance, which need not itself be symmetric.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct CrossCovariance3 {
    matrix: [[f64; 3]; 3],
}

impl CrossCovariance3 {
    /// Validates a row-major cross-covariance.
    pub fn from_matrix(matrix: [[f64; 3]; 3]) -> Result<Self, ValidationError> {
        if matrix.iter().flatten().all(|value| value.is_finite()) {
            Ok(Self { matrix })
        } else {
            Err(ValidationError::NonFinite)
        }
    }

    /// Returns the row-major cross-covariance matrix.
    #[must_use]
    pub const fn to_matrix(self) -> [[f64; 3]; 3] {
        self.matrix
    }

    /// Checks whether this cross block and the supplied marginal covariances
    /// form one positive-semidefinite 6-by-6 covariance.
    #[must_use]
    pub fn forms_valid_joint(self, first: Covariance3, second: Covariance3) -> bool {
        let first = first.to_matrix();
        let second = second.to_matrix();
        let mut joint = [[0.0; 6]; 6];
        for row in 0..3 {
            for column in 0..3 {
                joint[row][column] = first[row][column];
                joint[row + 3][column + 3] = second[row][column];
                joint[row][column + 3] = self.matrix[row][column];
                joint[column + 3][row] = self.matrix[row][column];
            }
        }
        is_positive_semidefinite(joint)
    }
}

/// Covariance fields returned by an explicit kinematic-state query.
///
/// Optional fields correspond to quantities that may be unavailable for a
/// dense segment.  The mandatory position/velocity/attitude marginals always
/// remain explicit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicCovariance {
    position: Covariance3,
    velocity: Covariance3,
    position_velocity: Option<CrossCovariance3>,
    /// Right-multiplicative attitude-error covariance in the body tangent
    /// basis. It is invariant to a change of terrestrial output frame.
    attitude_error: Covariance3,
    angular_rate: Option<Covariance3>,
    angular_acceleration: Option<Covariance3>,
    kinematic_acceleration: Option<Covariance3>,
    specific_force: Option<Covariance3>,
}

impl KinematicCovariance {
    /// Constructs mandatory kinematic covariance blocks and validates any
    /// position/velocity cross block jointly.
    pub fn new(
        position: Covariance3,
        velocity: Covariance3,
        position_velocity: Option<CrossCovariance3>,
        attitude_error: Covariance3,
    ) -> Result<Self, ValidationError> {
        if position_velocity.is_some_and(|cross| !cross.forms_valid_joint(position, velocity)) {
            return Err(ValidationError::InvalidCovariance);
        }
        Ok(Self {
            position,
            velocity,
            position_velocity,
            attitude_error,
            angular_rate: None,
            angular_acceleration: None,
            kinematic_acceleration: None,
            specific_force: None,
        })
    }

    /// Adds optional dynamic-quantity covariance blocks.
    #[must_use]
    pub const fn with_dynamic_covariances(
        mut self,
        angular_rate: Option<Covariance3>,
        angular_acceleration: Option<Covariance3>,
        kinematic_acceleration: Option<Covariance3>,
        specific_force: Option<Covariance3>,
    ) -> Self {
        self.angular_rate = angular_rate;
        self.angular_acceleration = angular_acceleration;
        self.kinematic_acceleration = kinematic_acceleration;
        self.specific_force = specific_force;
        self
    }

    /// Returns the position covariance.
    #[must_use]
    pub const fn position(self) -> Covariance3 {
        self.position
    }

    /// Returns the velocity covariance.
    #[must_use]
    pub const fn velocity(self) -> Covariance3 {
        self.velocity
    }

    /// Returns the position/velocity cross-covariance when retained.
    #[must_use]
    pub const fn position_velocity(self) -> Option<CrossCovariance3> {
        self.position_velocity
    }

    /// Returns the right-multiplicative attitude-error covariance in radians
    /// squared.
    #[must_use]
    pub const fn attitude_error(self) -> Covariance3 {
        self.attitude_error
    }

    /// Returns body angular-rate covariance when available.
    #[must_use]
    pub const fn angular_rate(self) -> Option<Covariance3> {
        self.angular_rate
    }

    /// Returns body angular-acceleration covariance when available.
    #[must_use]
    pub const fn angular_acceleration(self) -> Option<Covariance3> {
        self.angular_acceleration
    }

    /// Returns kinematic-acceleration covariance when available.
    #[must_use]
    pub const fn kinematic_acceleration(self) -> Option<Covariance3> {
        self.kinematic_acceleration
    }

    /// Returns specific-force covariance when available.
    #[must_use]
    pub const fn specific_force(self) -> Option<Covariance3> {
        self.specific_force
    }
}

/// Borrowed joint covariance for a bounded, fixed-order shared-parameter block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SharedParameterCovariance<'a> {
    dimension: u8,
    upper_triangle: &'a [f64],
}

impl<'a> SharedParameterCovariance<'a> {
    /// Validates a compact row-major upper triangle.
    pub fn new(dimension: usize, upper_triangle: &'a [f64]) -> Result<Self, ValidationError> {
        if dimension == 0 || dimension > MAX_SHARED_PARAMETER_DIMENSION {
            return Err(ValidationError::CapacityExceeded);
        }
        let required = dimension
            .checked_mul(dimension + 1)
            .and_then(|value| value.checked_div(2))
            .ok_or(ValidationError::CapacityExceeded)?;
        if upper_triangle.len() != required {
            return Err(ValidationError::InvalidCovariance);
        }
        if !upper_triangle.iter().all(|value| value.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        let mut matrix = [[0.0; MAX_SHARED_PARAMETER_DIMENSION]; MAX_SHARED_PARAMETER_DIMENSION];
        let mut index = 0;
        for row in 0..dimension {
            for column in row..dimension {
                let value = upper_triangle[index];
                matrix[row][column] = value;
                matrix[column][row] = value;
                index += 1;
            }
        }
        if (0..dimension).any(|index| matrix[index][index] < 0.0)
            || !is_positive_semidefinite_prefix(&matrix, dimension)
        {
            return Err(ValidationError::InvalidCovariance);
        }
        Ok(Self {
            dimension: dimension as u8,
            upper_triangle,
        })
    }

    /// Returns the fixed block dimension.
    #[must_use]
    pub const fn dimension(self) -> usize {
        self.dimension as usize
    }

    /// Returns the canonical upper-triangle values.
    #[must_use]
    pub const fn upper_triangle(self) -> &'a [f64] {
        self.upper_triangle
    }
}

fn is_positive_semidefinite<const N: usize>(matrix: [[f64; N]; N]) -> bool {
    is_positive_semidefinite_prefix(&matrix, N)
}

/// Checks a symmetric two-by-two covariance without forming either product in
/// `a * b - cross * cross`. For a non-zero cross term, the PSD condition is
/// equivalently `|cross| / max(a, b) <= min(a, b) / |cross|`; both quotients
/// remain meaningful when the original products overflow or underflow.
pub(crate) fn is_positive_semidefinite_2x2(
    a: f64,
    cross: f64,
    b: f64,
    relative_determinant_tolerance: f64,
) -> bool {
    if !a.is_finite()
        || !cross.is_finite()
        || !b.is_finite()
        || !relative_determinant_tolerance.is_finite()
        || a < 0.0
        || b < 0.0
        || !(0.0..1.0).contains(&relative_determinant_tolerance)
    {
        return false;
    }
    let cross = cross.abs();
    if cross == 0.0 {
        return true;
    }
    if a == 0.0 || b == 0.0 {
        return false;
    }
    let larger_diagonal = a.max(b);
    let smaller_diagonal = a.min(b);
    let normalized_cross = cross / larger_diagonal;
    let normalized_bound = (smaller_diagonal / cross) / (1.0 - relative_determinant_tolerance);
    normalized_cross <= normalized_bound
}

// Pivot-free LDLᵀ is sufficient here because covariance matrices are symmetric
// PSD. A numerically zero pivot is accepted only when all remaining values in
// that column are also zero within the same scale-aware tolerance.
fn is_positive_semidefinite_prefix<const N: usize>(
    matrix: &[[f64; N]; N],
    dimension: usize,
) -> bool {
    let mut lower = [[0.0; N]; N];
    let mut diagonal = [0.0; N];
    let mut scale = 0.0_f64;
    for row in matrix.iter().take(dimension) {
        for value in row.iter().take(dimension) {
            scale = scale.max(value.abs());
        }
    }
    let tolerance = 256.0 * f64::EPSILON * scale;

    for row in 0..dimension {
        for column in 0..row {
            let mut residual = matrix[row][column];
            for prior in 0..column {
                residual -= lower[row][prior] * diagonal[prior] * lower[column][prior];
            }
            if diagonal[column].abs() <= tolerance {
                if residual.abs() > tolerance {
                    return false;
                }
                lower[row][column] = 0.0;
            } else {
                lower[row][column] = residual / diagonal[column];
            }
        }

        let mut pivot = matrix[row][row];
        for prior in 0..row {
            pivot -= lower[row][prior] * lower[row][prior] * diagonal[prior];
        }
        if !pivot.is_finite() || pivot < -tolerance {
            return false;
        }
        diagonal[row] = if pivot.abs() <= tolerance { 0.0 } else { pivot };
        lower[row][row] = 1.0;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn covariance_rejects_asymmetry_negative_variance_and_indefiniteness() {
        assert_eq!(
            Covariance3::diagonal(-1.0, 1.0, 1.0),
            Err(ValidationError::InvalidCovariance)
        );
        assert_eq!(
            Covariance3::from_matrix([[1.0, 0.1, 0.0], [0.2, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            Err(ValidationError::InvalidCovariance)
        );
        assert_eq!(
            Covariance3::from_upper_triangle([1.0, 2.0, 0.0, 1.0, 0.0, 1.0]),
            Err(ValidationError::InvalidCovariance)
        );
    }

    #[test]
    fn covariance_accepts_positive_semidefinite_rank_deficiency() {
        let covariance = Covariance3::from_upper_triangle([1.0, 1.0, 0.0, 1.0, 0.0, 0.0]).unwrap();
        assert_eq!(covariance.variance(0), Some(1.0));
        assert_eq!(covariance.variance(2), Some(0.0));
    }

    #[test]
    fn covariance_psd_tolerance_tracks_tiny_matrix_scale() {
        let scale = 1.0e-24;
        assert!(
            Covariance3::from_upper_triangle([scale, 0.5 * scale, 0.0, scale, 0.0, scale,]).is_ok()
        );
        assert_eq!(
            Covariance3::from_upper_triangle([scale, 2.0 * scale, 0.0, scale, 0.0, scale,]),
            Err(ValidationError::InvalidCovariance)
        );
    }

    #[test]
    fn cross_covariance_is_validated_as_a_joint_matrix() {
        let marginal = Covariance3::diagonal(1.0, 1.0, 1.0).unwrap();
        let valid = CrossCovariance3::from_matrix([[0.5, 0.0, 0.0]; 3]).unwrap();
        let invalid = CrossCovariance3::from_matrix([[2.0, 0.0, 0.0]; 3]).unwrap();
        assert!(valid.forms_valid_joint(marginal, marginal));
        assert!(!invalid.forms_valid_joint(marginal, marginal));
    }

    #[test]
    fn modeled_uncertainty_cannot_be_confused_with_zero_covariance() {
        let model = UncertaintyModelId::new(42);
        let uncertainty: MeasurementUncertainty<Covariance3> =
            MeasurementUncertainty::Modeled(model);
        assert_eq!(uncertainty.provided(), None);
        assert_eq!(uncertainty.model(), Some(model));
    }

    #[test]
    fn shared_parameter_covariance_checks_shape_and_psd() {
        let valid = [1.0, 0.25, 1.0];
        assert!(SharedParameterCovariance::new(2, &valid).is_ok());
        assert_eq!(
            SharedParameterCovariance::new(2, &[1.0, 2.0, 1.0]),
            Err(ValidationError::InvalidCovariance)
        );
        assert_eq!(
            SharedParameterCovariance::new(2, &[1.0, 0.0]),
            Err(ValidationError::InvalidCovariance)
        );
    }

    #[test]
    fn kinematic_cross_block_must_be_jointly_valid() {
        let marginal = Covariance3::diagonal(1.0, 1.0, 1.0).unwrap();
        let invalid = CrossCovariance3::from_matrix([[4.0, 0.0, 0.0]; 3]).unwrap();
        assert_eq!(
            KinematicCovariance::new(marginal, marginal, Some(invalid), marginal),
            Err(ValidationError::InvalidCovariance)
        );
    }
}
