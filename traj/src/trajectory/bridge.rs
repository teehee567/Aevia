//! Host conditional bridge covariance and endpoint linearization.

use crate::error::{QueryError, ValidationError};
#[cfg(feature = "offline")]
use crate::uncertainty::CrossCovariance3;
use crate::uncertainty::{Covariance3, KinematicCovariance};
#[cfg(not(any(feature = "offline", test)))]
use nalgebra::ComplexField;
#[cfg(feature = "offline")]
use nalgebra::DMatrix;
#[cfg(feature = "offline")]
use std::boxed::Box;

#[cfg(feature = "offline")]
pub(super) const BRIDGE_KINEMATIC_DIMENSION: usize = 9;

#[cfg(feature = "offline")]
pub(super) const BRIDGE_ENDPOINT_DIMENSION: usize = 2 * BRIDGE_KINEMATIC_DIMENSION;

#[cfg(feature = "offline")]
pub(super) const BRIDGE_POSITION: usize = 0;

#[cfg(feature = "offline")]
pub(super) const BRIDGE_VELOCITY: usize = 3;

#[cfg(feature = "offline")]
pub(super) const BRIDGE_ATTITUDE: usize = 6;

/// Complete host-only input for one continuous-discrete conditional bridge.
///
/// The endpoint covariance ordering is `[p0, v0, theta0, p1, v1, theta1]`.
/// It is deliberately a joint matrix: two endpoint marginals cannot justify
/// an interior covariance. Process spectral densities are expressed in ECEF
/// acceleration and right-multiplicative body-attitude tangent coordinates.
/// Continuous densities and interval-average sample covariances remain
/// distinct because their discrete-time scaling and interior bridge kernels
/// are different.
#[cfg(feature = "offline")]
pub(crate) struct DenseBridgeInput {
    /// Whether the independent process model below represents the solver's
    /// covariance. False preserves endpoint marginals and dense means while
    /// leaving interior uncertainty explicitly unavailable.
    pub covariance_available: bool,
    pub endpoint_joint_covariance:
        Box<[[f64; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION]>,
    pub acceleration_spectral_density_ecef: [[f64; 3]; 3],
    pub attitude_spectral_density_body: [[f64; 3]; 3],
    pub acceleration_interval_average_covariance_ecef: [[f64; 3]; 3],
    pub angular_rate_interval_average_covariance_body: [[f64; 3]; 3],
    /// Corrected-IMU nominal endpoint before the endpoint error bridge.
    pub reintegrated_position_ecef_m: [f64; 3],
    pub reintegrated_velocity_ecef_mps: [f64; 3],
    /// Corrected Earth-relative rotation integrated from the start endpoint.
    pub integrated_rotation_body: [f64; 3],
}

/// First-order bridge coordinates used by host metric uncertainty without
/// exposing dense coefficients through the public trajectory API.
#[cfg(feature = "offline")]
pub(crate) struct DenseBridgeLinearization {
    pub start_jacobian: DMatrix<f64>,
    pub end_jacobian: DMatrix<f64>,
}

#[cfg(feature = "offline")]
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DenseConditionalBridge {
    pub(super) covariance_available: bool,
    pub(super) endpoint_joint_covariance:
        Box<[[f64; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION]>,
    pub(super) acceleration_spectral_density_ecef: [[f64; 3]; 3],
    pub(super) attitude_spectral_density_body: [[f64; 3]; 3],
    pub(super) acceleration_interval_average_covariance_ecef: [[f64; 3]; 3],
    pub(super) angular_rate_interval_average_covariance_body: [[f64; 3]; 3],
}

#[cfg(feature = "offline")]
impl DenseConditionalBridge {
    pub(super) fn new(input: &DenseBridgeInput) -> Result<Self, ValidationError> {
        validate_symmetric_psd(
            &DMatrix::from_row_slice(
                BRIDGE_ENDPOINT_DIMENSION,
                BRIDGE_ENDPOINT_DIMENSION,
                input.endpoint_joint_covariance.as_flattened(),
            ),
            true,
        )?;
        if input.covariance_available {
            validate_symmetric_psd(
                &DMatrix::from_row_slice(
                    3,
                    3,
                    input.acceleration_spectral_density_ecef.as_flattened(),
                ),
                true,
            )?;
            validate_symmetric_psd(
                &DMatrix::from_row_slice(3, 3, input.attitude_spectral_density_body.as_flattened()),
                true,
            )?;
            validate_symmetric_psd(
                &DMatrix::from_row_slice(
                    3,
                    3,
                    input
                        .acceleration_interval_average_covariance_ecef
                        .as_flattened(),
                ),
                true,
            )?;
            validate_symmetric_psd(
                &DMatrix::from_row_slice(
                    3,
                    3,
                    input
                        .angular_rate_interval_average_covariance_body
                        .as_flattened(),
                ),
                true,
            )?;
        }
        Ok(Self {
            covariance_available: input.covariance_available,
            endpoint_joint_covariance: input.endpoint_joint_covariance.clone(),
            acceleration_spectral_density_ecef: input.acceleration_spectral_density_ecef,
            attitude_spectral_density_body: input.attitude_spectral_density_body,
            acceleration_interval_average_covariance_ecef: input
                .acceleration_interval_average_covariance_ecef,
            angular_rate_interval_average_covariance_body: input
                .angular_rate_interval_average_covariance_body,
        })
    }

    pub(super) fn endpoint_joint(&self) -> DMatrix<f64> {
        DMatrix::from_row_slice(
            BRIDGE_ENDPOINT_DIMENSION,
            BRIDGE_ENDPOINT_DIMENSION,
            self.endpoint_joint_covariance.as_flattened(),
        )
    }

    pub(super) fn unconditioned_process_cross(
        &self,
        first_seconds: f64,
        second_seconds: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        if !first_seconds.is_finite()
            || !second_seconds.is_finite()
            || first_seconds < 0.0
            || second_seconds < 0.0
        {
            return Err(QueryError::TrajectoryInvalid);
        }
        let (earlier, later, transpose) = if first_seconds <= second_seconds {
            (first_seconds, second_seconds, false)
        } else {
            (second_seconds, first_seconds, true)
        };
        let acceleration =
            DMatrix::from_row_slice(3, 3, self.acceleration_spectral_density_ecef.as_flattened());
        let attitude =
            DMatrix::from_row_slice(3, 3, self.attitude_spectral_density_body.as_flattened());
        let acceleration_sample = DMatrix::from_row_slice(
            3,
            3,
            self.acceleration_interval_average_covariance_ecef
                .as_flattened(),
        );
        let angular_rate_sample = DMatrix::from_row_slice(
            3,
            3,
            self.angular_rate_interval_average_covariance_body
                .as_flattened(),
        );
        let mut cross = DMatrix::zeros(BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION);
        set_dense_block(
            &mut cross,
            BRIDGE_POSITION,
            BRIDGE_POSITION,
            &(&acceleration * (earlier.powi(2) * (3.0 * later - earlier) / 6.0)),
        );
        set_dense_block(
            &mut cross,
            BRIDGE_POSITION,
            BRIDGE_VELOCITY,
            &(&acceleration * (0.5 * earlier.powi(2))),
        );
        set_dense_block(
            &mut cross,
            BRIDGE_VELOCITY,
            BRIDGE_POSITION,
            &(&acceleration * (earlier * later - 0.5 * earlier.powi(2))),
        );
        set_dense_block(
            &mut cross,
            BRIDGE_VELOCITY,
            BRIDGE_VELOCITY,
            &(&acceleration * earlier),
        );
        set_dense_block(
            &mut cross,
            BRIDGE_ATTITUDE,
            BRIDGE_ATTITUDE,
            &(&attitude * earlier),
        );
        // One interval-average sample error is a single held random vector,
        // not a continuous density. Its effect at two interior epochs is
        // therefore fully correlated over this stored support.
        add_dense_block(
            &mut cross,
            BRIDGE_POSITION,
            BRIDGE_POSITION,
            &(&acceleration_sample * (0.25 * earlier.powi(2) * later.powi(2))),
        );
        add_dense_block(
            &mut cross,
            BRIDGE_POSITION,
            BRIDGE_VELOCITY,
            &(&acceleration_sample * (0.5 * earlier.powi(2) * later)),
        );
        add_dense_block(
            &mut cross,
            BRIDGE_VELOCITY,
            BRIDGE_POSITION,
            &(&acceleration_sample * (0.5 * earlier * later.powi(2))),
        );
        add_dense_block(
            &mut cross,
            BRIDGE_VELOCITY,
            BRIDGE_VELOCITY,
            &(&acceleration_sample * (earlier * later)),
        );
        add_dense_block(
            &mut cross,
            BRIDGE_ATTITUDE,
            BRIDGE_ATTITUDE,
            &(&angular_rate_sample * (earlier * later)),
        );
        if transpose {
            Ok(cross.transpose())
        } else {
            Ok(cross)
        }
    }

    pub(super) fn conditional_process_cross(
        &self,
        duration_seconds: f64,
        first_parameter: f64,
        second_parameter: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        if !duration_seconds.is_finite()
            || duration_seconds <= 0.0
            || !first_parameter.is_finite()
            || !second_parameter.is_finite()
            || !(0.0..=1.0).contains(&first_parameter)
            || !(0.0..=1.0).contains(&second_parameter)
        {
            return Err(QueryError::TrajectoryInvalid);
        }
        if first_parameter == 0.0
            || first_parameter == 1.0
            || second_parameter == 0.0
            || second_parameter == 1.0
        {
            return Ok(DMatrix::zeros(
                BRIDGE_KINEMATIC_DIMENSION,
                BRIDGE_KINEMATIC_DIMENSION,
            ));
        }
        let first = duration_seconds * first_parameter;
        let second = duration_seconds * second_parameter;
        let endpoint = self.unconditioned_process_cross(duration_seconds, duration_seconds)?;
        let Some(cholesky) = endpoint.clone().cholesky() else {
            return Err(QueryError::TrajectoryInvalid);
        };
        let first_endpoint = self.unconditioned_process_cross(first, duration_seconds)?;
        let endpoint_second = self.unconditioned_process_cross(duration_seconds, second)?;
        let solved = cholesky.solve(&endpoint_second);
        let conditional =
            self.unconditioned_process_cross(first, second)? - first_endpoint * solved;
        if !conditional.iter().all(|value| value.is_finite()) {
            return Err(QueryError::TrajectoryInvalid);
        }
        Ok(conditional)
    }
}

#[cfg(feature = "offline")]
pub(super) fn validate_symmetric_psd(
    matrix: &DMatrix<f64>,
    allow_semidefinite: bool,
) -> Result<(), ValidationError> {
    if matrix.nrows() != matrix.ncols()
        || !matrix.iter().all(|value| value.is_finite())
        || (0..matrix.nrows()).any(|row| {
            (row + 1..matrix.ncols()).any(|column| matrix[(row, column)] != matrix[(column, row)])
        })
    {
        return Err(ValidationError::InvalidCovariance);
    }
    let scale = matrix.amax();
    let tolerance = 16_384.0 * f64::EPSILON * scale * matrix.nrows() as f64;
    let minimum = matrix
        .clone()
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    if minimum < -tolerance || (!allow_semidefinite && minimum <= tolerance) {
        Err(ValidationError::InvalidCovariance)
    } else {
        Ok(())
    }
}

#[cfg(feature = "offline")]
pub(super) fn set_dense_block(
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
    value: &DMatrix<f64>,
) {
    debug_assert_eq!(value.shape(), (3, 3));
    for local_row in 0..3 {
        for local_column in 0..3 {
            target[(row + local_row, column + local_column)] = value[(local_row, local_column)];
        }
    }
}

#[cfg(feature = "offline")]
pub(super) fn add_dense_block(
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
    value: &DMatrix<f64>,
) {
    debug_assert_eq!(value.shape(), (3, 3));
    for local_row in 0..3 {
        for local_column in 0..3 {
            target[(row + local_row, column + local_column)] += value[(local_row, local_column)];
        }
    }
}

#[cfg(feature = "offline")]
pub(super) fn dense_kinematic_covariance(
    covariance: &DMatrix<f64>,
) -> Result<KinematicCovariance, QueryError> {
    if covariance.shape() != (BRIDGE_KINEMATIC_DIMENSION, BRIDGE_KINEMATIC_DIMENSION)
        || !covariance.iter().all(|value| value.is_finite())
    {
        return Err(QueryError::TrajectoryInvalid);
    }
    let covariance = (covariance + covariance.transpose()) * 0.5;
    validate_symmetric_psd(&covariance, true).map_err(|_| QueryError::TrajectoryInvalid)?;
    let block = |start: usize| {
        let mut matrix = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                matrix[row][column] = covariance[(start + row, start + column)];
            }
        }
        Covariance3::from_matrix(matrix).map_err(|_| QueryError::TrajectoryInvalid)
    };
    let position = block(BRIDGE_POSITION)?;
    let velocity = block(BRIDGE_VELOCITY)?;
    let attitude = block(BRIDGE_ATTITUDE)?;
    let mut position_velocity = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            position_velocity[row][column] =
                covariance[(BRIDGE_POSITION + row, BRIDGE_VELOCITY + column)];
        }
    }
    let cross = CrossCovariance3::from_matrix(position_velocity)
        .ok()
        .filter(|value| value.forms_valid_joint(position, velocity));
    KinematicCovariance::new(position, velocity, cross, attitude)
        .or_else(|_| KinematicCovariance::new(position, velocity, None, attitude))
        .map_err(|_| QueryError::TrajectoryInvalid)
}
