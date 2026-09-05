//! Fixed-interval smoothing and tangent-space covariance transforms.

use crate::{
    error::ProcessError,
    offline::store::{StateStore, StoredCovariance, StoredNominal},
};

use nalgebra::{DMatrix, DVector};

use super::{
    catalog::ConsiderCatalog,
    estimation::{inject_error, injection_reset, repair_covariance, solve_spd},
    forward::WorkTracker,
    math::{
        ACCELEROMETER_BIAS, ATTITUDE, COLORED_ERROR_DIMENSION, GYROSCOPE_BIAS,
        NAVIGATION_DIMENSION, POSITION, VELOCITY, copy_block, symmetric,
    },
};

pub(super) struct SmoothingOutcome {
    pub(super) maximum_step: f64,
    pub(super) covariance_repairs: u32,
}

pub(super) fn smooth_store(
    store: &mut dyn StateStore,
    catalog: &ConsiderCatalog,
    maximum_repair_attempts: u8,
    maximum_regularization: f64,
    work: &mut WorkTracker<'_>,
) -> Result<SmoothingOutcome, ProcessError> {
    let count = store.len();
    if count < 2 {
        return Err(ProcessError::IncompleteEvidence);
    }
    let (state_dimension, consider_dimension) = store.dimensions();
    let last_index = count - 1;
    let mut terminal = store.get(last_index).map_err(ProcessError::from)?;
    terminal.smoothed = Some(terminal.filtered.clone());
    terminal.smoothed_covariance = Some(terminal.filtered_covariance.clone());
    terminal.smoothed_backward_gain = None;
    store
        .set(last_index, &terminal)
        .map_err(ProcessError::from)?;
    let mut maximum_step = 0.0_f64;
    let mut covariance_repairs = 0_u32;

    for index in (0..last_index).rev() {
        let mut current = store.get(index).map_err(ProcessError::from)?;
        let next = store.get(index + 1).map_err(ProcessError::from)?;
        if !next.connected_from_previous {
            current.smoothed = Some(current.filtered.clone());
            current.smoothed_covariance = Some(current.filtered_covariance.clone());
            current.smoothed_backward_gain = None;
            store.set(index, &current).map_err(ProcessError::from)?;
            work.advance(1)?;
            continue;
        }
        let next_smoothed = next.smoothed.as_ref().ok_or(ProcessError::StorageCorrupt)?;
        let next_smoothed_covariance = next
            .smoothed_covariance
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        let state_residual = boxminus(next_smoothed, &next.predicted, state_dimension)?;
        let next_reset = injection_reset(&state_residual)?;
        let next_smoothed_in_predicted_basis =
            covariance_to_reference(next_smoothed_covariance, &next_reset)?;
        // Schmidt/consider smoothing conditions the earlier estimable state on
        // the next estimable state while keeping the consider mean and P_cc
        // fixed. Using a full augmented RTS difference after restoring P_cc
        // loses the covariance reduction that was deliberately withheld from
        // the consider rows. The state-only gain plus the separate P_xc
        // recursion is the exact linear-Gaussian consider update.
        let cross_state = &next.adjacent_cross_covariance;
        let solved = solve_spd(
            &next.predicted_covariance.state,
            &cross_state.transpose(),
            maximum_repair_attempts,
            maximum_regularization,
        )?;
        let gain = solved.transpose();
        let correction = &gain * state_residual;
        maximum_step = maximum_step.max(correction.norm());
        let (smoothed, smoothing_reset) = boxplus_with_reset(&current.filtered, &correction)?;
        let mut smoothed_covariance = StoredCovariance {
            state: symmetric(
                &current.filtered_covariance.state
                    + &gain
                        * (&next_smoothed_in_predicted_basis.state
                            - &next.predicted_covariance.state)
                        * gain.transpose(),
            ),
            state_consider: &current.filtered_covariance.state_consider
                + &gain
                    * (&next_smoothed_in_predicted_basis.state_consider
                        - &next.predicted_covariance.state_consider),
        };

        // Adjacent smoothed covariance before the two manifold-basis resets.
        let augmented_dimension = state_dimension + consider_dimension;
        let mut adjacent_cross = DMatrix::zeros(augmented_dimension, augmented_dimension);
        copy_block(
            &(&gain * &next_smoothed_in_predicted_basis.state),
            &mut adjacent_cross,
            0,
            0,
        );
        copy_block(
            &smoothed_covariance.state_consider,
            &mut adjacent_cross,
            0,
            state_dimension,
        );
        copy_block(
            &next_smoothed_in_predicted_basis.state_consider.transpose(),
            &mut adjacent_cross,
            state_dimension,
            0,
        );
        copy_block(
            &catalog.covariance,
            &mut adjacent_cross,
            state_dimension,
            state_dimension,
        );
        covariance_from_reference(&mut smoothed_covariance, &smoothing_reset)?;
        let mut smoothing_repairs = 0_u32;
        repair_covariance(
            &mut smoothed_covariance.state,
            maximum_repair_attempts,
            maximum_regularization,
            &mut smoothing_repairs,
        )?;
        covariance_repairs = covariance_repairs.saturating_add(smoothing_repairs);
        let mut current_augmented_reset =
            DMatrix::identity(augmented_dimension, augmented_dimension);
        let mut next_augmented_reset = DMatrix::identity(augmented_dimension, augmented_dimension);
        copy_block(&smoothing_reset, &mut current_augmented_reset, 0, 0);
        copy_block(&next_reset, &mut next_augmented_reset, 0, 0);
        let adjacent_cross =
            current_augmented_reset * adjacent_cross * next_augmented_reset.transpose();
        let next_final_augmented =
            augmented_covariance(next_smoothed_covariance, &catalog.covariance);
        // A rank-deficient exact consider prior makes the conditional
        // non-unique. Smoothing remains valid, but cross-time metric uncertainty
        // fails closed rather than regularizing an undeclared relationship.
        let smoothed_backward_gain = next_final_augmented
            .cholesky()
            .map(|cholesky| cholesky.solve(&adjacent_cross.transpose()).transpose())
            .filter(|value| value.iter().all(|entry| entry.is_finite()));
        current.smoothed = Some(smoothed);
        current.smoothed_covariance = Some(smoothed_covariance);
        current.smoothed_backward_gain = smoothed_backward_gain;
        store.set(index, &current).map_err(ProcessError::from)?;
        work.advance(1)?;
    }
    Ok(SmoothingOutcome {
        maximum_step,
        covariance_repairs,
    })
}

/// Maximum manifold step between successive IEKS iterates.  Stores must have
/// identical topology: accepting a pass that silently added, removed, or
/// reconnected an epoch would make convergence and cross-covariance semantics
/// undefined.
pub(super) fn maximum_smoothed_difference(
    candidate: &mut dyn StateStore,
    accepted: &mut dyn StateStore,
) -> Result<f64, ProcessError> {
    if candidate.len() != accepted.len() || candidate.dimensions() != accepted.dimensions() {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let state_dimension = candidate.dimensions().0;
    let mut maximum = 0.0_f64;
    for index in 0..candidate.len() {
        let candidate_step = candidate.get(index).map_err(ProcessError::from)?;
        let accepted_step = accepted.get(index).map_err(ProcessError::from)?;
        if candidate_step.filtered.time != accepted_step.filtered.time
            || candidate_step.connected_from_previous != accepted_step.connected_from_previous
        {
            return Err(ProcessError::NumericalNonConvergence);
        }
        let candidate_state = candidate_step
            .smoothed
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        let accepted_state = accepted_step
            .smoothed
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        let difference = boxminus(candidate_state, accepted_state, state_dimension)?.norm();
        if !difference.is_finite() {
            return Err(ProcessError::NumericalNonConvergence);
        }
        maximum = maximum.max(difference);
    }
    Ok(maximum)
}

pub(super) fn augmented_covariance(
    covariance: &StoredCovariance,
    consider_covariance: &DMatrix<f64>,
) -> DMatrix<f64> {
    let state_dimension = covariance.state.nrows();
    let consider_dimension = consider_covariance.nrows();
    let mut result = DMatrix::zeros(
        state_dimension + consider_dimension,
        state_dimension + consider_dimension,
    );
    copy_block(&covariance.state, &mut result, 0, 0);
    copy_block(&covariance.state_consider, &mut result, 0, state_dimension);
    copy_block(
        &covariance.state_consider.transpose(),
        &mut result,
        state_dimension,
        0,
    );
    copy_block(
        consider_covariance,
        &mut result,
        state_dimension,
        state_dimension,
    );
    symmetric(result)
}

pub(super) fn covariance_to_reference(
    covariance: &StoredCovariance,
    reset: &DMatrix<f64>,
) -> Result<StoredCovariance, ProcessError> {
    if reset.nrows() != reset.ncols()
        || reset.nrows() != covariance.state.nrows()
        || !reset.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    let decomposition = reset.clone().lu();
    let left = decomposition
        .solve(&covariance.state)
        .ok_or(ProcessError::NumericalNonConvergence)?;
    let state = decomposition
        .solve(&left.transpose())
        .ok_or(ProcessError::NumericalNonConvergence)?
        .transpose();
    let state_consider = decomposition
        .solve(&covariance.state_consider)
        .ok_or(ProcessError::NumericalNonConvergence)?;
    if state.iter().all(|value| value.is_finite())
        && state_consider.iter().all(|value| value.is_finite())
    {
        Ok(StoredCovariance {
            state: symmetric(state),
            state_consider,
        })
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn covariance_from_reference(
    covariance: &mut StoredCovariance,
    reset: &DMatrix<f64>,
) -> Result<(), ProcessError> {
    if reset.nrows() != reset.ncols()
        || reset.nrows() != covariance.state.nrows()
        || !reset.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    covariance.state = symmetric(reset * &covariance.state * reset.transpose());
    covariance.state_consider = reset * &covariance.state_consider;
    if covariance.state.iter().all(|value| value.is_finite())
        && covariance
            .state_consider
            .iter()
            .all(|value| value.is_finite())
    {
        Ok(())
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn boxminus(
    value: &StoredNominal,
    reference: &StoredNominal,
    state_dimension: usize,
) -> Result<DVector<f64>, ProcessError> {
    let mut result = DVector::zeros(state_dimension);
    for axis in 0..3 {
        result[POSITION + axis] = value.position_ecef[axis] - reference.position_ecef[axis];
        result[VELOCITY + axis] = value.velocity_ecef[axis] - reference.velocity_ecef[axis];
        result[ACCELEROMETER_BIAS + axis] =
            value.accelerometer_bias_body[axis] - reference.accelerometer_bias_body[axis];
        result[GYROSCOPE_BIAS + axis] =
            value.gyroscope_bias_body[axis] - reference.gyroscope_bias_body[axis];
        if state_dimension >= NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION {
            result[NAVIGATION_DIMENSION + axis] =
                value.colored_gnss_error[axis] - reference.colored_gnss_error[axis];
        }
    }
    let attitude = reference
        .orientation_ecef_from_body
        .inverse()
        .multiply(value.orientation_ecef_from_body)
        .rotation_vector()
        .components();
    for axis in 0..3 {
        result[ATTITUDE + axis] = attitude[axis];
    }
    if result.iter().all(|value| value.is_finite()) {
        Ok(result)
    } else {
        Err(ProcessError::NumericalNonConvergence)
    }
}

pub(super) fn boxplus_with_reset(
    reference: &StoredNominal,
    correction: &DVector<f64>,
) -> Result<(StoredNominal, DMatrix<f64>), ProcessError> {
    let mut result = reference.clone();
    let reset = inject_error(&mut result, correction)?;
    Ok((result, reset))
}
