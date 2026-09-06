//! Fixed-interval smoothing and tangent-space covariance transforms.

use crate::{
    error::ProcessError,
    offline::store::{StateStore, StoredCovariance, StoredImuSample, StoredNominal},
};

use nalgebra::{DMatrix, DVector};

use super::{
    catalog::ConsiderCatalog,
    estimation::{inject_error, injection_reset, repair_covariance, solve_psd},
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
    let (n, m) = store.dimensions();
    let d = n + 6;
    let last_index = count - 1;
    let mut terminal = store.get(last_index).map_err(ProcessError::from)?;
    terminal.smoothed = Some(terminal.filtered.clone());
    terminal.smoothed_covariance = Some(terminal.filtered_covariance.clone());
    terminal.smoothed_sample = Some(terminal.filtered_sample.clone());
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
            current.smoothed_sample = Some(current.filtered_sample.clone());
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
        let next_smoothed_sample = next
            .smoothed_sample
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        let next_final =
            sample_estimable_covariance(next_smoothed_covariance, next_smoothed_sample);
        let next_predicted =
            sample_estimable_covariance(&next.predicted_covariance, &next.predicted_sample);
        let current_filtered =
            sample_estimable_covariance(&current.filtered_covariance, &current.filtered_sample);
        let residual = boxminus(next_smoothed, &next.predicted, d)?;
        let next_reset = injection_reset(&residual)?;
        let next_reference = covariance_to_reference(&next_final, &next_reset)?;
        // Both navigation and the retained sample are estimable. Keeping the
        // sample here makes every interior support cut a Markov transition.
        let mut forward_cross = next.adjacent_sample_cross.clone();
        copy_block(&next.adjacent_cross_covariance, &mut forward_cross, 0, 0);
        let gain = solve_psd(&next_predicted.state, &forward_cross.transpose())?.transpose();
        let correction = &gain * residual;
        maximum_step = maximum_step.max(correction.norm());
        let (mut smoothed, reset) = boxplus_with_reset(&current.filtered, &correction)?;
        refresh_smoothed_kinematics(&current.filtered, &mut smoothed)?;
        let mut covariance = StoredCovariance {
            state: symmetric(
                &current_filtered.state
                    + &gain * (&next_reference.state - &next_predicted.state) * gain.transpose(),
            ),
            state_consider: &current_filtered.state_consider
                + &gain * (&next_reference.state_consider - &next_predicted.state_consider),
        };
        // Form adjacent joint in [x,s,c], then reorder to the persistent
        // public solver convention [x,c,s].
        let mut adjacent = DMatrix::zeros(d + m, d + m);
        copy_block(&(&gain * &next_reference.state), &mut adjacent, 0, 0);
        copy_block(&covariance.state_consider, &mut adjacent, 0, d);
        copy_block(
            &next_reference.state_consider.transpose(),
            &mut adjacent,
            d,
            0,
        );
        copy_block(&catalog.covariance, &mut adjacent, d, d);
        covariance_from_reference(&mut covariance, &reset)?;
        repair_covariance(
            &mut covariance.state,
            maximum_repair_attempts,
            maximum_regularization,
            &mut covariance_repairs,
        )?;
        let mut left_reset = DMatrix::identity(d + m, d + m);
        let mut right_reset = DMatrix::identity(d + m, d + m);
        copy_block(&reset, &mut left_reset, 0, 0);
        copy_block(&next_reset, &mut right_reset, 0, 0);
        let adjacent = left_reset * adjacent * right_reset.transpose();
        let old_index = |i| {
            if i < n {
                i
            } else if i < n + m {
                d + i - n
            } else {
                n + i - n - m
            }
        };
        let adjacent =
            DMatrix::from_fn(d + m, d + m, |r, c| adjacent[(old_index(r), old_index(c))]);
        let next_joint = joint_covariance(
            next_smoothed_covariance,
            next_smoothed_sample,
            &catalog.covariance,
        );
        current.smoothed_backward_gain =
            Some(solve_psd(&next_joint, &adjacent.transpose())?.transpose());
        current.smoothed = Some(smoothed);
        current.smoothed_covariance = Some(StoredCovariance {
            state: covariance.state.view((0, 0), (n, n)).into_owned(),
            state_consider: covariance.state_consider.rows(0, n).into_owned(),
        });
        current.smoothed_sample = Some(StoredImuSample {
            start: current.filtered_sample.start,
            end: current.filtered_sample.end,
            covariance_body: covariance.state.view((n, n), (6, 6)).into_owned(),
            state_cross: covariance.state.view((0, n), (n, 6)).into_owned(),
            consider_cross: covariance.state_consider.rows(n, 6).into_owned(),
        });
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
    let state_dimension = candidate.dimensions().0 + 6;
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

fn refresh_smoothed_kinematics(
    filtered: &StoredNominal,
    smoothed: &mut StoredNominal,
) -> Result<(), ProcessError> {
    use super::math::{EARTH_RATE_RAD_S, array3, matrix3_from_array, vector3};
    use nalgebra::{Matrix3, Vector3};
    let earth = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let before = matrix3_from_array(filtered.orientation_ecef_from_body.rotation_matrix())
        .transpose()
        * earth;
    let raw_rate = vector3(filtered.angular_rate_body)
        + vector3(filtered.gyroscope_bias_body)
        + before
        + Vector3::from_column_slice(&filtered.imu_sample_error_body[3..]);
    let imu = super::filter::HeldImu {
        start: filtered.time,
        time: filtered.time,
        angular_rate_body: array3(raw_rate),
        specific_force_body: core::array::from_fn(|i| {
            filtered.specific_force_body[i] + filtered.imu_sample_error_body[i]
        }),
        accelerometer_covariance: Matrix3::zeros(),
        gyroscope_covariance: Matrix3::zeros(),
        degraded_input: false,
    };
    super::inertial::refresh_inertial_kinematics(smoothed, &imu)
}

pub(super) fn sample_estimable_covariance(
    covariance: &StoredCovariance,
    sample: &StoredImuSample,
) -> StoredCovariance {
    let n = covariance.state.nrows();
    let m = covariance.state_consider.ncols();
    let mut state = DMatrix::zeros(n + 6, n + 6);
    copy_block(&covariance.state, &mut state, 0, 0);
    copy_block(&sample.state_cross, &mut state, 0, n);
    copy_block(&sample.state_cross.transpose(), &mut state, n, 0);
    copy_block(&sample.covariance_body, &mut state, n, n);
    let mut state_consider = DMatrix::zeros(n + 6, m);
    copy_block(&covariance.state_consider, &mut state_consider, 0, 0);
    copy_block(&sample.consider_cross, &mut state_consider, n, 0);
    StoredCovariance {
        state,
        state_consider,
    }
}

/// Full retained joint, ordered [navigation, fixed consider, held IMU error].
pub(super) fn joint_covariance(
    covariance: &StoredCovariance,
    sample: &StoredImuSample,
    consider: &DMatrix<f64>,
) -> DMatrix<f64> {
    let n = covariance.state.nrows();
    let m = consider.nrows();
    let mut result = DMatrix::zeros(n + m + 6, n + m + 6);
    copy_block(
        &augmented_covariance(covariance, consider),
        &mut result,
        0,
        0,
    );
    copy_block(&sample.state_cross, &mut result, 0, n + m);
    copy_block(&sample.state_cross.transpose(), &mut result, n + m, 0);
    copy_block(&sample.consider_cross, &mut result, n + m, n);
    copy_block(&sample.consider_cross.transpose(), &mut result, n, n + m);
    copy_block(&sample.covariance_body, &mut result, n + m, n + m);
    result
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
        if state_dimension == NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION
            || state_dimension == NAVIGATION_DIMENSION + COLORED_ERROR_DIMENSION + 6
        {
            result[NAVIGATION_DIMENSION + axis] =
                value.colored_gnss_error[axis] - reference.colored_gnss_error[axis];
        }
    }
    if state_dimension >= NAVIGATION_DIMENSION + 6 {
        for axis in 0..6 {
            result[state_dimension - 6 + axis] =
                value.imu_sample_error_body[axis] - reference.imu_sample_error_body[axis];
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
