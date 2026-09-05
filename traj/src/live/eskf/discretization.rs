//! Continuous navigation dynamics and discrete transition/process noise.

use super::{
    EskfError, ProcessNoise,
    matrix::{multiply_nav, multiply_nav_right_transpose, symmetrize_nav},
};
#[cfg(test)]
use super::{PreintegrationNavJacobian, propagation::preintegration_nav_mapping_into};
use crate::live::{
    preintegration::PreintegratedBatch,
    state::{
        ACC_BIAS, ATT, GYRO_BIAS, MechanizationContext, NAV_DIM, NavMatrix, NavState, POS, VEL,
        skew,
    },
};
use nalgebra::Matrix3;

const DISCRETIZATION_LOCAL_NORM_LIMIT: f32 = 0.25;

const MAX_DISCRETIZATION_SQUARINGS: u8 = 12;

const EXPONENTIAL_TAYLOR_ORDER: u8 = 10;

const LYAPUNOV_TAYLOR_ORDER: u8 = 12;

pub(super) fn continuous_state_matrix_into(
    state: &NavState,
    batch: &PreintegratedBatch,
    context: &MechanizationContext,
    continuous: &mut NavMatrix,
) {
    let rotation = state.orientation_n_from_b.to_rotation_matrix().into_inner();
    let corrected_force = batch.mean_specific_force_b - state.accel_bias_b;
    let corrected_omega = batch.mean_omega_ib_b - state.gyro_bias_b;
    continuous.fill(0.0);
    continuous
        .fixed_view_mut::<3, 3>(POS, VEL)
        .copy_from(&Matrix3::identity());
    continuous
        .fixed_view_mut::<3, 3>(VEL, POS)
        .copy_from(&context.gravity_gradient_n);
    continuous
        .fixed_view_mut::<3, 3>(VEL, VEL)
        .copy_from(&(skew(&context.earth_rate_n) * -2.0));
    continuous
        .fixed_view_mut::<3, 3>(VEL, ATT)
        .copy_from(&(-rotation * skew(&corrected_force)));
    continuous
        .fixed_view_mut::<3, 3>(VEL, ACC_BIAS)
        .copy_from(&(-rotation));
    continuous
        .fixed_view_mut::<3, 3>(ATT, ATT)
        .copy_from(&(-skew(&corrected_omega)));
    continuous
        .fixed_view_mut::<3, 3>(ATT, GYRO_BIAS)
        .copy_from(&(-Matrix3::identity()));
}

/// Fixed-size scaling-and-squaring exponential. Each local Taylor series is
/// evaluated only after `||F h||_inf <= 0.25`; at order ten its normwise
/// remainder is below `exp(0.25) * 0.25^11 / 11! < 8e-15`, well beneath f32
/// roundoff. The fixed squaring cap covers `||F dt||_inf <= 1024` and rejects
/// more extreme inputs rather than returning an unvalidated covariance.
#[inline(never)]
pub(super) fn state_transition_into(
    continuous: &NavMatrix,
    dt: f32,
    transition: &mut NavMatrix,
    term: &mut NavMatrix,
    temporary: &mut NavMatrix,
) -> Result<(), EskfError> {
    let (local_dt, squarings) = discretization_scale(continuous, dt)?;
    taylor_transition(continuous, local_dt, transition, term, temporary);
    for _ in 0..squarings {
        multiply_nav(transition, transition, temporary);
        transition.copy_from(temporary);
    }
    if transition.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(EskfError::NumericalFailure)
    }
}

fn taylor_transition(
    continuous: &NavMatrix,
    dt: f32,
    transition: &mut NavMatrix,
    term: &mut NavMatrix,
    temporary: &mut NavMatrix,
) {
    transition.fill(0.0);
    term.fill(0.0);
    for diagonal in 0..NAV_DIM {
        transition[(diagonal, diagonal)] = 1.0;
        term[(diagonal, diagonal)] = 1.0;
    }
    for order in 1..=EXPONENTIAL_TAYLOR_ORDER {
        multiply_nav(term, continuous, temporary);
        let scale = dt / f32::from(order);
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                term[(row, column)] = temporary[(row, column)] * scale;
                transition[(row, column)] += term[(row, column)];
            }
        }
    }
}

fn discretization_scale(continuous: &NavMatrix, dt: f32) -> Result<(f32, u8), EskfError> {
    if !dt.is_finite() || dt <= 0.0 || !continuous.iter().all(|value| value.is_finite()) {
        return Err(EskfError::NumericalFailure);
    }
    let mut norm = 0.0_f32;
    for row in 0..NAV_DIM {
        let row_sum = (0..NAV_DIM)
            .map(|column| continuous[(row, column)].abs())
            .sum::<f32>();
        norm = norm.max(row_sum);
    }
    let mut local_dt = dt;
    let mut scaled_norm = norm * dt;
    let mut squarings = 0;
    while scaled_norm > DISCRETIZATION_LOCAL_NORM_LIMIT {
        if squarings == MAX_DISCRETIZATION_SQUARINGS {
            return Err(EskfError::DiscretizationBoundExceeded);
        }
        local_dt *= 0.5;
        scaled_norm *= 0.5;
        squarings += 1;
    }
    if local_dt.is_finite() && scaled_norm.is_finite() {
        Ok((local_dt, squarings))
    } else {
        Err(EskfError::NumericalFailure)
    }
}

/// Discretizes continuous accelerometer/gyro bias random walks through the
/// complete 15-state error dynamics. For the local step the Lyapunov series
/// has operator norm at most 0.5; order twelve leaves a relative tail below
/// `exp(0.5) * 0.5^12 / 13! < 7e-14`. Exact doubling then preserves all
/// attitude/velocity/position, bias, and cross blocks across the full batch.
#[inline(never)]
pub(super) fn add_bias_random_walk_discrete_covariance(
    continuous: &NavMatrix,
    dt: f32,
    process_noise: ProcessNoise,
    target: &mut NavMatrix,
    transition: &mut NavMatrix,
    covariance: &mut NavMatrix,
    term: &mut NavMatrix,
    temporary: &mut NavMatrix,
) -> Result<(), EskfError> {
    let (local_dt, squarings) = discretization_scale(continuous, dt)?;
    taylor_transition(continuous, local_dt, transition, term, temporary);
    term.fill(0.0);
    term.fixed_view_mut::<3, 3>(ACC_BIAS, ACC_BIAS)
        .copy_from(&(process_noise.accel_bias_random_walk_covariance_density * local_dt));
    term.fixed_view_mut::<3, 3>(GYRO_BIAS, GYRO_BIAS)
        .copy_from(&(process_noise.gyro_bias_random_walk_covariance_density * local_dt));
    covariance.copy_from(term);
    for order in 2..=LYAPUNOV_TAYLOR_ORDER {
        let factor = local_dt / f32::from(order);
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                let mut value = 0.0;
                for inner in 0..NAV_DIM {
                    value += continuous[(row, inner)] * term[(inner, column)]
                        + term[(row, inner)] * continuous[(column, inner)];
                }
                temporary[(row, column)] = value * factor;
            }
        }
        term.copy_from(temporary);
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                covariance[(row, column)] += term[(row, column)];
            }
        }
    }
    symmetrize_nav(covariance);
    for _ in 0..squarings {
        multiply_nav(transition, covariance, term);
        multiply_nav_right_transpose(term, transition, temporary);
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                temporary[(row, column)] += covariance[(row, column)];
            }
        }
        symmetrize_nav(temporary);
        covariance.copy_from(temporary);
        multiply_nav(transition, transition, term);
        transition.copy_from(term);
    }
    if covariance.iter().all(|value| value.is_finite()) {
        for row in 0..NAV_DIM {
            for column in 0..NAV_DIM {
                target[(row, column)] += covariance[(row, column)];
            }
        }
        Ok(())
    } else {
        Err(EskfError::NumericalFailure)
    }
}

#[cfg(test)]
pub(super) fn continuous_state_matrix(
    state: &NavState,
    batch: &PreintegratedBatch,
    context: &MechanizationContext,
) -> NavMatrix {
    let mut result = NavMatrix::zeros();
    continuous_state_matrix_into(state, batch, context, &mut result);
    result
}

#[cfg(test)]
pub(super) fn preintegration_nav_mapping(
    force_rotation: &Matrix3<f32>,
) -> PreintegrationNavJacobian {
    let mut result = PreintegrationNavJacobian::zeros();
    preintegration_nav_mapping_into(force_rotation, &mut result);
    result
}

#[cfg(test)]
pub(super) fn state_transition(continuous: &NavMatrix, dt: f32) -> Result<NavMatrix, EskfError> {
    let mut result = NavMatrix::zeros();
    let mut term = NavMatrix::zeros();
    let mut temporary = NavMatrix::zeros();
    state_transition_into(continuous, dt, &mut result, &mut term, &mut temporary)?;
    Ok(result)
}

#[cfg(test)]
pub(super) fn bias_random_walk_discrete_covariance(
    continuous: &NavMatrix,
    dt: f32,
    process_noise: ProcessNoise,
) -> Result<NavMatrix, EskfError> {
    let mut result = NavMatrix::zeros();
    let mut transition = NavMatrix::zeros();
    let mut covariance = NavMatrix::zeros();
    let mut term = NavMatrix::zeros();
    let mut temporary = NavMatrix::zeros();
    add_bias_random_walk_discrete_covariance(
        continuous,
        dt,
        process_noise,
        &mut result,
        &mut transition,
        &mut covariance,
        &mut term,
        &mut temporary,
    )?;
    Ok(result)
}
