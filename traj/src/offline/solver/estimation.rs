//! Schmidt updates, linear solves, and covariance conditioning.

use crate::{
    error::ProcessError,
    math::{UnitQuaternion, Vector3 as SemanticVector3},
    observation::InputDisposition,
    offline::store::{StoredCovariance, StoredNominal},
};

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use super::{
    filter::ActiveImuSample,
    math::{
        ACCELEROMETER_BIAS, ATTITUDE, COLORED_ERROR_DIMENSION, GYROSCOPE_BIAS,
        NAVIGATION_DIMENSION, POSITION, VELOCITY, array3, copy_block, set_matrix3, skew, symmetric,
    },
    measurement::MeasurementOutcome,
    smoothing::{boxminus, boxplus_with_reset, covariance_from_reference, covariance_to_reference},
};

pub(super) fn left_solve(
    left: &DMatrix<f64>,
    right: &DMatrix<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    if left.nrows() != left.ncols()
        || right.nrows() != left.nrows()
        || !left.iter().all(|value| value.is_finite())
        || !right.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let solved = left
        .clone()
        .lu()
        .solve(right)
        .ok_or(ProcessError::NumericalNonConvergence)?;
    if solved.iter().all(|value| value.is_finite()) {
        Ok(solved)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn right_solve(
    left: &DMatrix<f64>,
    right: &DMatrix<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    if right.nrows() != right.ncols()
        || left.ncols() != right.nrows()
        || !left.iter().all(|value| value.is_finite())
        || !right.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let solved_transpose = right
        .transpose()
        .lu()
        .solve(&left.transpose())
        .ok_or(ProcessError::NumericalNonConvergence)?;
    let solved = solved_transpose.transpose();
    if solved.iter().all(|value| value.is_finite()) {
        Ok(solved)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

#[allow(clippy::too_many_arguments)]
#[cfg(test)]
pub(super) fn schmidt_update(
    nominal: &mut StoredNominal,
    covariance: &mut StoredCovariance,
    h_state: &DMatrix<f64>,
    h_consider: &DMatrix<f64>,
    consider_covariance: &DMatrix<f64>,
    residual: &mut DVector<f64>,
    base_noise: &DMatrix<f64>,
    robust_threshold: f64,
    rejection_threshold: f64,
    maximum_covariance_inflation: f64,
    maximum_repair_attempts: u8,
    maximum_regularization: f64,
    repair_count: &mut u32,
) -> Result<MeasurementOutcome, ProcessError> {
    schmidt_update_affine(
        nominal,
        covariance,
        h_state,
        h_consider,
        consider_covariance,
        residual,
        base_noise,
        robust_threshold,
        rejection_threshold,
        maximum_covariance_inflation,
        maximum_repair_attempts,
        maximum_regularization,
        repair_count,
        None,
        1.0,
    )
}

/// Applies one Schmidt update while retaining the currently held IMU sample
/// as a fixed-mean ephemeral nuisance variable. This preserves both the
/// state/sample cross-covariance created by partial propagation and the same
/// gyro sample's direct lever-arm velocity sensitivity.
#[allow(clippy::too_many_arguments)]
pub(super) fn schmidt_update_affine_with_sample(
    nominal: &mut StoredNominal,
    covariance: &mut StoredCovariance,
    sample: &mut ActiveImuSample,
    h_state: &DMatrix<f64>,
    h_consider: &DMatrix<f64>,
    h_sample: &DMatrix<f64>,
    consider_covariance: &DMatrix<f64>,
    residual_at_reference: &mut DVector<f64>,
    base_noise: &DMatrix<f64>,
    robust_threshold: f64,
    rejection_threshold: f64,
    maximum_covariance_inflation: f64,
    maximum_repair_attempts: u8,
    maximum_regularization: f64,
    repair_count: &mut u32,
    linearization_reference: Option<&StoredNominal>,
    damping: f64,
) -> Result<MeasurementOutcome, ProcessError> {
    let state_dimension = covariance.state.nrows();
    let consider_dimension = consider_covariance.nrows();
    if covariance.state_consider.shape() != (state_dimension, consider_dimension)
        || sample.state_cross.shape() != (state_dimension, 6)
        || sample.covariance_body.shape() != (6, 6)
        || h_sample.shape() != (h_state.nrows(), 6)
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let mut combined = StoredCovariance {
        state: covariance.state.clone(),
        state_consider: DMatrix::zeros(state_dimension, consider_dimension + 6),
    };
    copy_block(
        &covariance.state_consider,
        &mut combined.state_consider,
        0,
        0,
    );
    copy_block(
        &sample.state_cross,
        &mut combined.state_consider,
        0,
        consider_dimension,
    );
    let mut combined_consider = DMatrix::zeros(consider_dimension + 6, consider_dimension + 6);
    copy_block(consider_covariance, &mut combined_consider, 0, 0);
    copy_block(
        &sample.covariance_body,
        &mut combined_consider,
        consider_dimension,
        consider_dimension,
    );
    let mut combined_h = DMatrix::zeros(h_state.nrows(), consider_dimension + 6);
    copy_block(h_consider, &mut combined_h, 0, 0);
    copy_block(h_sample, &mut combined_h, 0, consider_dimension);

    let outcome = schmidt_update_affine(
        nominal,
        &mut combined,
        h_state,
        &combined_h,
        &combined_consider,
        residual_at_reference,
        base_noise,
        robust_threshold,
        rejection_threshold,
        maximum_covariance_inflation,
        maximum_repair_attempts,
        maximum_regularization,
        repair_count,
        linearization_reference,
        damping,
    )?;
    covariance.state = combined.state;
    for row in 0..state_dimension {
        for column in 0..consider_dimension {
            covariance.state_consider[(row, column)] = combined.state_consider[(row, column)];
        }
        for column in 0..6 {
            sample.state_cross[(row, column)] =
                combined.state_consider[(row, consider_dimension + column)];
        }
    }
    Ok(outcome)
}

/// Schmidt measurement update for an affine model linearized about a
/// previous-pass state.  `residual_at_reference` is `z - h(reference)`; the
/// innovation therefore also subtracts `H (current ⊟ reference)`.  This is
/// the term a normal EKF update omits when its Jacobian is merely evaluated at
/// a guide trajectory.
#[allow(clippy::too_many_arguments)]
pub(super) fn schmidt_update_affine(
    nominal: &mut StoredNominal,
    covariance: &mut StoredCovariance,
    h_state: &DMatrix<f64>,
    h_consider: &DMatrix<f64>,
    consider_covariance: &DMatrix<f64>,
    residual_at_reference: &mut DVector<f64>,
    base_noise: &DMatrix<f64>,
    robust_threshold: f64,
    rejection_threshold: f64,
    maximum_covariance_inflation: f64,
    maximum_repair_attempts: u8,
    maximum_regularization: f64,
    repair_count: &mut u32,
    linearization_reference: Option<&StoredNominal>,
    damping: f64,
) -> Result<MeasurementOutcome, ProcessError> {
    if robust_threshold <= 0.0
        || rejection_threshold <= 0.0
        || !robust_threshold.is_finite()
        || !rejection_threshold.is_finite()
        || !maximum_covariance_inflation.is_finite()
        || maximum_covariance_inflation < 1.0
        || !damping.is_finite()
        || !(0.0..=1.0).contains(&damping)
        || damping == 0.0
        || h_state.ncols() != covariance.state.nrows()
        || h_consider.ncols() != consider_covariance.nrows()
        || consider_covariance.nrows() != consider_covariance.ncols()
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let reference = linearization_reference
        .cloned()
        .unwrap_or_else(|| nominal.clone());
    if reference.time != nominal.time {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let prior_delta = boxminus(nominal, &reference, covariance.state.nrows())?;
    let prior_reset = injection_reset(&prior_delta)?;
    let mut reference_covariance = if linearization_reference.is_some() {
        covariance_to_reference(covariance, &prior_reset)?
    } else {
        covariance.clone()
    };
    let residual = residual_at_reference.clone() - h_state * &prior_delta;
    if !residual.iter().all(|value| value.is_finite()) {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let cross_state = h_state * &reference_covariance.state
        + h_consider * reference_covariance.state_consider.transpose();
    let cross_consider =
        h_state * &reference_covariance.state_consider + h_consider * consider_covariance;
    let mut measurement_covariance = symmetric(
        &cross_state * h_state.transpose() + &cross_consider * h_consider.transpose() + base_noise,
    );
    let mut solved_residual = solve_spd(
        &measurement_covariance,
        &DMatrix::from_column_slice(residual.len(), 1, residual.as_slice()),
        maximum_repair_attempts,
        maximum_regularization,
    )?;
    let mut nis = (residual.transpose() * &solved_residual)[(0, 0)];
    if !nis.is_finite() || nis < -256.0 * f64::EPSILON {
        return Err(ProcessError::NumericalNonConvergence);
    }
    nis = nis.max(0.0);
    if nis > rejection_threshold {
        return Ok(MeasurementOutcome {
            disposition: InputDisposition::StatisticallyRejected,
            objective: 0.5 * rejection_threshold,
            reset_basis: DMatrix::identity(covariance.state.nrows(), covariance.state.ncols()),
        });
    }
    let mut disposition = InputDisposition::Fused;
    let effective_robust_threshold = robust_threshold.min(rejection_threshold);
    let mut effective_noise = base_noise.clone();
    if nis > effective_robust_threshold {
        let inflation = (nis / effective_robust_threshold)
            .max(1.0)
            .min(maximum_covariance_inflation);
        effective_noise *= inflation;
        measurement_covariance = symmetric(
            &cross_state * h_state.transpose()
                + &cross_consider * h_consider.transpose()
                + &effective_noise,
        );
        solved_residual = solve_spd(
            &measurement_covariance,
            &DMatrix::from_column_slice(residual.len(), 1, residual.as_slice()),
            maximum_repair_attempts,
            maximum_regularization,
        )?;
        nis = (residual.transpose() * &solved_residual)[(0, 0)].max(0.0);
        disposition = InputDisposition::Downweighted;
    }

    let numerator = &reference_covariance.state * h_state.transpose()
        + &reference_covariance.state_consider * h_consider.transpose();
    let gain = solve_spd(
        &measurement_covariance,
        &numerator.transpose(),
        maximum_repair_attempts,
        maximum_regularization,
    )?
    .transpose()
        * damping;
    let posterior_delta = &prior_delta + &gain * residual.clone();
    let previous_nominal = nominal.clone();
    let (posterior_nominal, posterior_reset) = boxplus_with_reset(&reference, &posterior_delta)?;
    let relative_correction = boxminus(
        &posterior_nominal,
        &previous_nominal,
        covariance.state.nrows(),
    )?;
    let reset_basis = injection_reset(&relative_correction)?;

    // Schmidt update: the fixed consider mean and P_cc are unchanged.  Only
    // estimable rows receive a gain, while P_xc retains the correlation needed
    // by future observations and the backward consider recursion.
    let identity = DMatrix::identity(
        reference_covariance.state.nrows(),
        reference_covariance.state.ncols(),
    );
    let state_transform = &identity - &gain * h_state;
    let consider_transform = -&gain * h_consider;
    let updated_state_consider = &state_transform * &reference_covariance.state_consider
        + &consider_transform * consider_covariance;
    reference_covariance.state = symmetric(
        &state_transform * &reference_covariance.state * state_transform.transpose()
            + &state_transform
                * &reference_covariance.state_consider
                * consider_transform.transpose()
            + &consider_transform
                * reference_covariance.state_consider.transpose()
                * state_transform.transpose()
            + &consider_transform * consider_covariance * consider_transform.transpose()
            + &gain * &effective_noise * gain.transpose(),
    );
    reference_covariance.state_consider = updated_state_consider;
    covariance_from_reference(&mut reference_covariance, &posterior_reset)?;
    repair_covariance(
        &mut reference_covariance.state,
        maximum_repair_attempts,
        maximum_regularization,
        repair_count,
    )?;
    *nominal = posterior_nominal;
    *covariance = reference_covariance;
    *residual_at_reference = residual;
    Ok(MeasurementOutcome {
        disposition,
        objective: if nis <= effective_robust_threshold {
            0.5 * nis
        } else {
            let root = (nis / effective_robust_threshold).sqrt();
            effective_robust_threshold * (root - 0.5)
        },
        reset_basis,
    })
}

pub(super) fn inject_error(
    nominal: &mut StoredNominal,
    correction: &DVector<f64>,
) -> Result<DMatrix<f64>, ProcessError> {
    if correction.len() < NAVIGATION_DIMENSION || !correction.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    for axis in 0..3 {
        nominal.position_ecef[axis] += correction[POSITION + axis];
        nominal.velocity_ecef[axis] += correction[VELOCITY + axis];
        nominal.accelerometer_bias_body[axis] += correction[ACCELEROMETER_BIAS + axis];
        nominal.gyroscope_bias_body[axis] += correction[GYROSCOPE_BIAS + axis];
        if correction.len() >= NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION {
            nominal.colored_gnss_error[axis] += correction[NAVIGATION_DIMENSION + axis];
        }
    }
    let attitude_error = Vector3::new(
        correction[ATTITUDE],
        correction[ATTITUDE + 1],
        correction[ATTITUDE + 2],
    );
    let semantic_error = SemanticVector3::from_components(array3(attitude_error))
        .map_err(|_| ProcessError::NumericalNonConvergence)?;
    nominal.orientation_ecef_from_body = nominal.orientation_ecef_from_body.multiply(
        UnitQuaternion::from_rotation_vector(semantic_error)
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
    );
    if !nominal.is_finite() {
        return Err(ProcessError::NumericalNonConvergence);
    }
    injection_reset(correction)
}

pub(super) fn injection_reset(correction: &DVector<f64>) -> Result<DMatrix<f64>, ProcessError> {
    if correction.len() < NAVIGATION_DIMENSION || !correction.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let attitude_error = Vector3::new(
        correction[ATTITUDE],
        correction[ATTITUDE + 1],
        correction[ATTITUDE + 2],
    );
    let mut reset = DMatrix::identity(correction.len(), correction.len());
    let angle_squared = attitude_error.norm_squared();
    let (linear_coefficient, quadratic_coefficient) = if angle_squared < 1.0e-12 {
        (
            0.5 - angle_squared / 24.0 + angle_squared * angle_squared / 720.0,
            1.0 / 6.0 - angle_squared / 120.0 + angle_squared * angle_squared / 5_040.0,
        )
    } else {
        let angle = angle_squared.sqrt();
        (
            (1.0 - angle.cos()) / angle_squared,
            (angle - angle.sin()) / (angle_squared * angle),
        )
    };
    let cross = skew(&attitude_error);
    let attitude_reset =
        Matrix3::identity() - linear_coefficient * cross + quadratic_coefficient * cross * cross;
    set_matrix3(&mut reset, ATTITUDE, ATTITUDE, &attitude_reset);
    Ok(reset)
}

pub(super) fn solve_spd(
    matrix: &DMatrix<f64>,
    right_hand_side: &DMatrix<f64>,
    maximum_attempts: u8,
    maximum_regularization: f64,
) -> Result<DMatrix<f64>, ProcessError> {
    if matrix.nrows() != matrix.ncols()
        || right_hand_side.nrows() != matrix.nrows()
        || !matrix.iter().all(|value| value.is_finite())
        || !right_hand_side.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let mut candidate = symmetric(matrix.clone());
    let scale = (0..candidate.nrows())
        .map(|index| candidate[(index, index)].abs())
        .fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let mut regularization = 0.0;
    let mut increment = 256.0 * f64::EPSILON * scale;
    for attempt in 0..=maximum_attempts {
        if let Some(cholesky) = candidate.clone().cholesky() {
            return Ok(cholesky.solve(right_hand_side));
        }
        if attempt == maximum_attempts
            || regularization + increment > maximum_regularization
            || !increment.is_finite()
        {
            break;
        }
        for diagonal in 0..candidate.nrows() {
            candidate[(diagonal, diagonal)] += increment;
        }
        regularization += increment;
        increment *= 10.0;
    }
    Err(ProcessError::NumericalNonConvergence)
}

pub(super) fn repair_covariance(
    covariance: &mut DMatrix<f64>,
    maximum_attempts: u8,
    maximum_regularization: f64,
    repair_count: &mut u32,
) -> Result<(), ProcessError> {
    *covariance = symmetric(covariance.clone());
    if covariance.clone().cholesky().is_some() {
        return Ok(());
    }
    let scale = (0..covariance.nrows())
        .map(|index| covariance[(index, index)].abs())
        .fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let mut total = 0.0;
    let mut increment = 256.0 * f64::EPSILON * scale;
    for _ in 0..maximum_attempts {
        if total + increment > maximum_regularization || !increment.is_finite() {
            break;
        }
        for diagonal in 0..covariance.nrows() {
            covariance[(diagonal, diagonal)] += increment;
        }
        total += increment;
        *repair_count = repair_count.saturating_add(1);
        if covariance.clone().cholesky().is_some() {
            return Ok(());
        }
        increment *= 10.0;
    }
    Err(ProcessError::NumericalNonConvergence)
}

pub(super) fn matrix_is_psd(matrix: &DMatrix<f64>) -> bool {
    if matrix.nrows() != matrix.ncols() || !matrix.iter().all(|value| value.is_finite()) {
        return false;
    }
    let scale = matrix
        .iter()
        .map(|value| value.abs())
        .fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return true;
    }
    let Ok(dimension) = u32::try_from(matrix.nrows().max(1)) else {
        return false;
    };
    let symmetry_tolerance = 512.0 * f64::EPSILON * scale * f64::from(dimension);
    for row in 0..matrix.nrows() {
        for column in 0..row {
            if (matrix[(row, column)] - matrix[(column, row)]).abs() > symmetry_tolerance {
                return false;
            }
        }
    }
    let symmetric = symmetric(matrix.clone());
    let tolerance = 512.0 * f64::EPSILON * scale * f64::from(dimension);
    symmetric
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .all(|value| *value >= -tolerance)
}
