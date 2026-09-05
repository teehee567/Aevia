//! State-coordinate constants and small vector and covariance operations.

use crate::{
    error::ProcessError,
    uncertainty::{Covariance3, CrossCovariance3, KinematicCovariance},
};

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

pub(super) const NAVIGATION_DIMENSION: usize = 15;

pub(super) const COLORED_ERROR_DIMENSION: usize = 3;

pub(super) const POSITION: usize = 0;

pub(super) const VELOCITY: usize = 3;

pub(super) const ATTITUDE: usize = 6;

pub(super) const ACCELEROMETER_BIAS: usize = 9;

pub(super) const GYROSCOPE_BIAS: usize = 12;

pub(super) const EARTH_RATE_RAD_S: f64 = 7.292_115_0e-5;

pub(super) const EARTH_MU_M3_S2: f64 = 3.986_004_418e14;

pub(super) const EARTH_J2: f64 = 1.082_626_68e-3;

pub(super) fn kinematic_covariance(
    state: &DMatrix<f64>,
) -> Result<KinematicCovariance, ProcessError> {
    let position = covariance3_from_block(state, POSITION)?;
    let velocity = covariance3_from_block(state, VELOCITY)?;
    let attitude = covariance3_from_block(state, ATTITUDE)?;
    let cross_matrix = array_matrix3(state, POSITION, VELOCITY);
    let cross = CrossCovariance3::from_matrix(cross_matrix)
        .ok()
        .filter(|value| value.forms_valid_joint(position, velocity));
    KinematicCovariance::new(position, velocity, cross, attitude)
        .or_else(|_| KinematicCovariance::new(position, velocity, None, attitude))
        .map_err(|_| ProcessError::NumericalNonConvergence)
}

pub(super) fn covariance3_from_block(
    matrix: &DMatrix<f64>,
    start: usize,
) -> Result<Covariance3, ProcessError> {
    let mut block = array_matrix3(matrix, start, start);
    for row in 0..3 {
        for column in row + 1..3 {
            let value = 0.5 * (block[row][column] + block[column][row]);
            block[row][column] = value;
            block[column][row] = value;
        }
        if block[row][row] < 0.0 && block[row][row] > -1.0e-12 {
            block[row][row] = 0.0;
        }
    }
    Covariance3::from_matrix(block).map_err(|_| ProcessError::NumericalNonConvergence)
}

pub(super) fn vector3(value: [f64; 3]) -> Vector3<f64> {
    Vector3::new(value[0], value[1], value[2])
}

pub(super) fn array3(value: Vector3<f64>) -> [f64; 3] {
    [value.x, value.y, value.z]
}

pub(super) fn dvector3(value: Vector3<f64>) -> DVector<f64> {
    DVector::from_column_slice(&[value.x, value.y, value.z])
}

pub(super) fn set_vector3(target: &mut DVector<f64>, row: usize, value: Vector3<f64>) {
    for axis in 0..3 {
        target[row + axis] = value[axis];
    }
}

pub(super) fn matrix3_from_array(value: [[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::new(
        value[0][0],
        value[0][1],
        value[0][2],
        value[1][0],
        value[1][1],
        value[1][2],
        value[2][0],
        value[2][1],
        value[2][2],
    )
}

pub(super) fn matrix3_to_array(value: Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [value[(0, 0)], value[(0, 1)], value[(0, 2)]],
        [value[(1, 0)], value[(1, 1)], value[(1, 2)]],
        [value[(2, 0)], value[(2, 1)], value[(2, 2)]],
    ]
}

pub(super) fn symmetric3(value: Matrix3<f64>) -> Matrix3<f64> {
    (value + value.transpose()) * 0.5
}

pub(super) fn set_matrix3(
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
    value: &Matrix3<f64>,
) {
    for local_row in 0..3 {
        for local_column in 0..3 {
            target[(row + local_row, column + local_column)] = value[(local_row, local_column)];
        }
    }
}

pub(super) fn set_dynamic_matrix3(
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
    value: &DMatrix<f64>,
) {
    for local_row in 0..3 {
        for local_column in 0..3 {
            target[(row + local_row, column + local_column)] = value[(local_row, local_column)];
        }
    }
}

pub(super) fn set_rect_matrix3(
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
    value: &Matrix3<f64>,
) {
    for local_row in 0..3 {
        for local_column in 0..3 {
            target[(row + local_row, column + local_column)] = value[(local_row, local_column)];
        }
    }
}

pub(super) fn set_identity3(target: &mut DMatrix<f64>, row: usize, column: usize) {
    for axis in 0..3 {
        target[(row + axis, column + axis)] = 1.0;
    }
}

pub(super) fn array_matrix3(matrix: &DMatrix<f64>, row: usize, column: usize) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for local_row in 0..3 {
        for local_column in 0..3 {
            result[local_row][local_column] = matrix[(row + local_row, column + local_column)];
        }
    }
    result
}

pub(super) fn skew(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

pub(super) fn symmetric(matrix: DMatrix<f64>) -> DMatrix<f64> {
    (&matrix + matrix.transpose()) * 0.5
}

pub(super) fn copy_block(
    source: &DMatrix<f64>,
    target: &mut DMatrix<f64>,
    row: usize,
    column: usize,
) {
    for source_row in 0..source.nrows() {
        for source_column in 0..source.ncols() {
            target[(row + source_row, column + source_column)] =
                source[(source_row, source_column)];
        }
    }
}
