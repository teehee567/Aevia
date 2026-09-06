use super::*;
use crate::{
    live::state::{right_jacobian, so3_exp},
    time::SessionTime,
};
use nalgebra::Vector3;

fn covariance() -> RtsCovariance {
    let mut result = RtsCovariance::new();
    result.nav = NavMatrix::identity();
    result
}

fn transition() -> RtsTransition {
    let mut result = RtsTransition::new();
    for axis in 0..NAV_DIM {
        result.nav[(axis, axis)] = 1.0;
    }
    result
}

fn cross(covariance: &RtsCovariance, consider: &ConsiderCovariance) -> AugmentedNavCross {
    AugmentedNavCross::from_fn(|row, column| covariance.joint_entry(row, column, consider))
}

fn at(ns: i64) -> NavState {
    NavState::stationary(SessionTime::from_ns(ns))
}

fn assert_close(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual {actual}, expected {expected}"
    );
}

#[test]
fn ordinary_linear_gaussian_step_matches_analytical_rts() {
    let current = covariance();
    let consider = ConsiderCovariance::zeros();
    let initial_cross = cross(&current, &consider);
    let mut prediction = current;
    prediction.nav[(0, 0)] = 2.0;
    let mut next = RtsEstimate::new();
    next.state = at(1);
    next.state.position_n.x = 2.0;
    next.covariance.copy_from(&prediction);
    next.covariance.nav[(0, 0)] = 2.0 / 3.0;
    next.predicted_smoothed_cross = cross(&next.covariance, &consider);
    let mut output = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &prediction,
            consider: &consider,
            active_consider: 0,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert_close(output.state.position_n.x, 1.0, 2.0e-6);
    assert_close(output.covariance.nav[(0, 0)], 2.0 / 3.0, 2.0e-6);
    assert_eq!(output.state.time, SessionTime::ZERO);
    assert_close(output.predicted_smoothed_cross[(0, 0)], 2.0 / 3.0, 2.0e-6);
    // A backward pass never modifies the forward endpoint passed to it.
    assert_eq!(next.state.position_n.x, 2.0);

    // Full ESKF covariance products can lose one ulp on an unobserved yaw
    // diagonal. Preserve the real correction without inventing yaw information.
    next.covariance.nav[(ATT + 2, ATT + 2)] -= f32::EPSILON;
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            predicted_filtered_cross: &initial_cross,
            filtered_reset: &Matrix3::identity(),
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &prediction,
            consider: &consider,
            active_consider: 0,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert_close(output.state.position_n.x, 1.0, 2.0e-6);
    assert_close(output.covariance.nav[(0, 0)], 2.0 / 3.0, 2.0e-6);
    assert_eq!(output.covariance.nav[(ATT + 2, ATT + 2)], 1.0);
}

#[test]
fn no_new_measurement_leaves_history_and_cross_unchanged() {
    let mut current = covariance();
    current.nav[(0, 0)] = 3.0;
    current.nav_consider[(0, 0)] = 0.5;
    let mut consider = ConsiderCovariance::zeros();
    consider[(0, 0)] = 2.0;
    let initial_cross = cross(&current, &consider);
    let mut prediction = current;
    prediction.nav[(0, 0)] += 1.0;
    let mut next = RtsEstimate::new();
    next.state = at(1);
    next.covariance.copy_from(&prediction);
    next.predicted_smoothed_cross = cross(&prediction, &consider);
    let mut output = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &prediction,
            consider: &consider,
            active_consider: 1,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert_eq!(output.state, at(0));
    assert_eq!(output.covariance, current);
    assert_eq!(output.predicted_smoothed_cross, initial_cross);
}

#[test]
fn shared_parameter_in_both_process_and_measurement_matches_direct_conditioning() {
    // x1=x0+c; y=x1+c+v. Independent x0,c,v each have variance 1.
    // The forward Schmidt gain is 1/2. Ordinary nav-only RTS would use 1/2
    // again and produce y/4; the correct historical correction is y/6.
    let current = covariance();
    let mut consider = ConsiderCovariance::zeros();
    consider[(0, 0)] = 1.0;
    let initial_cross = cross(&current, &consider);
    let mut edge = transition();
    edge.nav[(0, CONSIDER_START)] = 1.0;
    let mut prediction = current;
    prediction.nav[(0, 0)] = 2.0;
    prediction.nav_consider[(0, 0)] = 1.0;
    let mut next = RtsEstimate::new();
    next.state = at(1);
    next.state.position_n.x = 1.5; // y=3, forward gain=1/2.
    next.covariance.copy_from(&prediction);
    next.covariance.nav[(0, 0)] = 0.5;
    next.covariance.nav_consider[(0, 0)] = 0.0;
    next.predicted_smoothed_cross = cross(&next.covariance, &consider);
    let mut output = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &edge,
            next_predicted_state: &at(1),
            next_predicted: &prediction,
            consider: &consider,
            active_consider: 1,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert_close(output.state.position_n.x, 0.5, 2.0e-6);
    assert_close(output.covariance.nav[(0, 0)], 5.0 / 6.0, 2.0e-6);
    assert_close(output.covariance.nav_consider[(0, 0)], -1.0 / 3.0, 2.0e-6);
    assert_eq!(consider[(0, 0)], 1.0);
}

#[test]
fn held_sample_and_gap_are_retained_with_shared_parameters() {
    // Independent x,c,s,g,q,v; x+=x+c+2s+3g+q and y=x++c+2s+v.
    // q has variance .2; all other primitive uncertainties have variance 1.
    let mut current = covariance();
    current.sample[(0, 0)] = 1.0;
    current.gap[(0, 0)] = 1.0;
    let mut consider = ConsiderCovariance::zeros();
    consider[(0, 0)] = 1.0;
    let initial_cross = cross(&current, &consider);
    let mut edge = transition();
    edge.nav[(0, CONSIDER_START)] = 1.0;
    edge.nav[(0, SAMPLE_START)] = 2.0;
    edge.nav[(0, GAP_START)] = 3.0;
    edge.retain_sample = true;
    edge.retain_gap = true;
    let mut prediction = current;
    prediction.nav[(0, 0)] = 15.2;
    prediction.nav_consider[(0, 0)] = 1.0;
    prediction.nav_sample[(0, 0)] = 2.0;
    prediction.nav_gap[(0, 0)] = 3.0;
    let innovation = 31.2;
    let gain = 20.2 / innovation;
    let mut next = RtsEstimate::new();
    next.state = at(1);
    next.state.position_n.x = gain * 3.0;
    next.covariance.copy_from(&prediction);
    next.covariance.nav[(0, 0)] -= gain * 20.2;
    next.covariance.nav_consider[(0, 0)] -= gain * 2.0;
    next.covariance.nav_sample[(0, 0)] -= gain * 4.0;
    next.covariance.nav_gap[(0, 0)] -= gain * 3.0;
    next.predicted_smoothed_cross = cross(&next.covariance, &consider);
    let mut output = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &edge,
            next_predicted_state: &at(1),
            next_predicted: &prediction,
            consider: &consider,
            active_consider: 1,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert_close(output.state.position_n.x, 3.0 / innovation, 2.0e-5);
    assert_close(
        output.covariance.nav[(0, 0)],
        1.0 - 1.0 / innovation,
        2.0e-5,
    );
    assert_close(
        output.covariance.nav_consider[(0, 0)],
        -2.0 / innovation,
        2.0e-5,
    );
    assert_close(
        output.covariance.nav_sample[(0, 0)],
        -4.0 / innovation,
        2.0e-5,
    );
    assert_close(output.covariance.nav_gap[(0, 0)], -3.0 / innovation, 2.0e-5);
    assert_eq!(output.covariance.sample, current.sample);
    assert_eq!(output.covariance.gap, current.gap);

    // A deterministic covariance ensemble is an independent oracle: the 12
    // points +/-sqrt(6)*e_i have exactly identity covariance. Apply the actual
    // historical estimator to primitive errors and measure its error moments.
    let estimator_gain = output.state.position_n.x / 3.0;
    let mut variance = 0.0;
    let mut parameter_cross = 0.0;
    let mut sample_cross = 0.0;
    let mut gap_cross = 0.0;
    for axis in 0..6 {
        for sign in [-1.0, 1.0] {
            let mut primitive = [0.0; 6];
            primitive[axis] = sign * crate::scalar_math::sqrt(6.0);
            let measurement = primitive[0]
                + 2.0 * primitive[1]
                + 4.0 * primitive[2]
                + 3.0 * primitive[3]
                + crate::scalar_math::sqrt(0.2) * primitive[4]
                + primitive[5];
            let error = primitive[0] - estimator_gain * measurement;
            variance += error * error / 12.0;
            parameter_cross += error * primitive[1] / 12.0;
            sample_cross += error * primitive[2] / 12.0;
            gap_cross += error * primitive[3] / 12.0;
        }
    }
    assert_close(output.covariance.nav[(0, 0)], variance, 2.0e-5);
    assert_close(
        output.covariance.nav_consider[(0, 0)],
        parameter_cross,
        2.0e-5,
    );
    assert_close(output.covariance.nav_sample[(0, 0)], sample_cross, 2.0e-5);
    assert_close(output.covariance.nav_gap[(0, 0)], gap_cross, 2.0e-5);
}

#[test]
fn right_attitude_tangent_is_transported_in_both_covariance_and_error_cross() {
    let mut current = covariance();
    current.nav[(ATT, ATT)] = 2.0;
    current.nav[(ATT + 1, ATT + 1)] = 3.0;
    current.nav[(POS, ATT)] = 0.2;
    current.nav[(ATT, POS)] = 0.2;
    let consider = ConsiderCovariance::zeros();
    let initial_cross = cross(&current, &consider);
    let delta = Vector3::new(0.2, -0.1, 0.3);
    let reset = right_jacobian(delta);
    let posterior_reference = current.nav * 0.5;
    let mut next = RtsEstimate::new();
    next.state = at(1);
    next.state.orientation_n_from_b = so3_exp(delta);
    next.predicted_to_smoothed_reset = reset;
    transport_navigation_covariance(&posterior_reference, &reset, &mut next.covariance.nav);
    let mut reference_cross = AugmentedNavCross::zeros();
    reference_cross
        .fixed_rows_mut::<NAV_DIM>(0)
        .copy_from(&posterior_reference);
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            next.predicted_smoothed_cross[(row, column)] =
                right_transported_entry(&reference_cross, row, column, &reset);
        }
    }
    let mut output = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &current,
            consider: &consider,
            active_consider: 0,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    assert!((so3_log(&output.state.orientation_n_from_b) - delta).norm() < 2.0e-6);
    assert!((output.covariance.nav - next.covariance.nav).norm() < 5.0e-6);
    assert!((output.predicted_smoothed_cross - next.predicted_smoothed_cross).norm() < 5.0e-6);

    // Multiple actual injections require the product of their reset Jacobians.
    let first_delta = Vector3::new(0.4, 0.0, -0.2);
    let second_delta = Vector3::new(-0.1, 0.3, 0.05);
    let cumulative_reset = right_jacobian(second_delta) * right_jacobian(first_delta);
    next.state.orientation_n_from_b = so3_exp(first_delta) * so3_exp(second_delta);
    next.predicted_to_smoothed_reset = cumulative_reset;
    transport_navigation_covariance(
        &posterior_reference,
        &cumulative_reset,
        &mut next.covariance.nav,
    );
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            next.predicted_smoothed_cross[(row, column)] =
                right_transported_entry(&reference_cross, row, column, &cumulative_reset);
        }
    }
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &current,
            predicted: &current,
            predicted_filtered_cross: &initial_cross,
            filtered_reset: &Matrix3::identity(),
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &current,
            consider: &consider,
            active_consider: 0,
        },
        &next,
        &mut output,
        &mut RtsScratch::new(),
    )
    .unwrap();
    let total_delta = so3_log(&next.state.orientation_n_from_b);
    assert!((so3_log(&output.state.orientation_n_from_b) - total_delta).norm() < 3.0e-6);
    let mut expected = NavMatrix::zeros();
    transport_navigation_covariance(
        &posterior_reference,
        &right_jacobian(total_delta),
        &mut expected,
    );
    assert!((output.covariance.nav - expected).norm() < 6.0e-6);
}

#[test]
fn perfectly_correlated_nuisance_axes_use_their_supported_subspace() {
    let mut factor = PsdFactor::new();
    factor
        .factor(3, |row, column| {
            [[2.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]][row][column]
        })
        .unwrap();
    let mut rhs = AugmentedNavCross::zeros();
    rhs[(0, 0)] = 1.5;
    rhs[(1, 0)] = 1.0;
    rhs[(2, 0)] = 1.0;
    factor.solve(&mut rhs, 1).unwrap();
    assert_close(2.0 * rhs[(0, 0)] + rhs[(1, 0)] + rhs[(2, 0)], 1.5, 2.0e-6);
    assert_close(rhs[(0, 0)] + rhs[(1, 0)] + rhs[(2, 0)], 1.0, 2.0e-6);
}

#[test]
fn indefinite_covariance_is_rejected_without_regularizing_it() {
    let mut factor = PsdFactor::new();
    assert_eq!(
        factor.factor(2, |row, column| if row == column { 1.0 } else { 2.0 }),
        Err(SmoothingError::InvalidCovariance)
    );
}

#[test]
fn multiple_backward_steps_match_standard_rts_recursion() {
    let consider = ConsiderCovariance::zeros();
    let first = covariance();
    let first_cross = cross(&first, &consider);
    let mut middle_prediction = first;
    middle_prediction.nav[(0, 0)] = 1.5;
    let mut middle_filtered = middle_prediction;
    middle_filtered.nav[(0, 0)] = 0.6;
    let middle_cross = cross(&middle_filtered, &consider);
    let mut middle_state = at(1);
    middle_state.position_n.x = 0.6;
    let mut final_prediction = middle_filtered;
    final_prediction.nav[(0, 0)] = 0.85;
    let mut final_predicted_state = at(2);
    final_predicted_state.position_n.x = 0.6;
    let mut final_estimate = RtsEstimate::new();
    final_estimate.state = final_predicted_state;
    final_estimate.state.position_n.x += 0.68 * (2.0 - 0.6);
    final_estimate.covariance.copy_from(&final_prediction);
    final_estimate.covariance.nav[(0, 0)] = 0.272;
    final_estimate.predicted_smoothed_cross = cross(&final_estimate.covariance, &consider);
    let mut middle = RtsEstimate::new();
    let mut scratch = RtsScratch::new();
    backward_step(
        &RtsStep {
            filtered_state: &middle_state,
            filtered: &middle_filtered,
            predicted: &middle_prediction,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &middle_cross,
            next_transition: &transition(),
            next_predicted_state: &final_predicted_state,
            next_predicted: &final_prediction,
            consider: &consider,
            active_consider: 0,
        },
        &final_estimate,
        &mut middle,
        &mut scratch,
    )
    .unwrap();
    let middle_gain = 0.6 / 0.85;
    let middle_mean = 0.6 + middle_gain * (final_estimate.state.position_n.x - 0.6);
    let middle_variance = 0.6 + middle_gain * middle_gain * (0.272 - 0.85);
    assert_close(middle.state.position_n.x, middle_mean, 2.0e-6);
    assert_close(middle.covariance.nav[(0, 0)], middle_variance, 2.0e-6);
    let mut initial = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &first,
            predicted: &first,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &first_cross,
            next_transition: &transition(),
            next_predicted_state: &at(1),
            next_predicted: &middle_prediction,
            consider: &consider,
            active_consider: 0,
        },
        &middle,
        &mut initial,
        &mut scratch,
    )
    .unwrap();
    assert_close(initial.state.position_n.x, middle_mean / 1.5, 2.0e-6);
    assert_close(
        initial.covariance.nav[(0, 0)],
        1.0 + (middle_variance - 1.5) / 2.25,
        2.0e-6,
    );
}

#[test]
fn multistep_schmidt_covariance_matches_primitive_error_ensemble() {
    // Each row is the coefficient of six independent unit-variance primitive
    // errors [initial x, c, q1, v1, q2, v2]. Their dot products give an exact
    // ensemble oracle for this deliberately suboptimal forward Schmidt filter.
    let dot = |a: &[f32; 6], b: &[f32; 6]| -> f32 {
        a.iter().zip(b).map(|(left, right)| left * right).sum()
    };
    let c = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
    let initial_error = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let predicted_error = [1.0, 1.0, crate::scalar_math::sqrt(0.5), 0.0, 0.0, 0.0];
    let observation1 = [1.0, 2.0, crate::scalar_math::sqrt(0.5), 1.0, 0.0, 0.0];
    let gain1 = dot(&predicted_error, &observation1) / dot(&observation1, &observation1);
    let filtered_error = core::array::from_fn(|i| predicted_error[i] - gain1 * observation1[i]);
    let mut next_predicted_error = filtered_error;
    next_predicted_error[1] += 1.0;
    next_predicted_error[4] = 0.5;
    let mut observation2 = next_predicted_error;
    observation2[1] += 2.0;
    observation2[5] = crate::scalar_math::sqrt(0.8);
    let gain2 = dot(&next_predicted_error, &observation2) / dot(&observation2, &observation2);
    let final_error = core::array::from_fn(|i| next_predicted_error[i] - gain2 * observation2[i]);
    let final_correction_error = core::array::from_fn(|i| gain2 * observation2[i]);
    let middle_smoother_gain = dot(&filtered_error, &final_correction_error)
        / dot(&final_correction_error, &final_correction_error);
    let smoothed_error = core::array::from_fn(|i| {
        filtered_error[i] - middle_smoother_gain * final_correction_error[i]
    });
    let middle_correction_error = core::array::from_fn(|i| predicted_error[i] - smoothed_error[i]);
    let initial_smoother_gain = dot(&initial_error, &middle_correction_error)
        / dot(&middle_correction_error, &middle_correction_error);
    let initial_smoothed_error = core::array::from_fn(|i| {
        initial_error[i] - initial_smoother_gain * middle_correction_error[i]
    });

    let mut consider = ConsiderCovariance::zeros();
    consider[(0, 0)] = 1.0;
    let initial_covariance = covariance();
    let initial_cross = cross(&initial_covariance, &consider);
    let mut predicted = covariance();
    predicted.nav[(0, 0)] = dot(&predicted_error, &predicted_error);
    predicted.nav_consider[(0, 0)] = dot(&predicted_error, &c);
    let mut filtered = covariance();
    filtered.nav[(0, 0)] = dot(&filtered_error, &filtered_error);
    filtered.nav_consider[(0, 0)] = dot(&filtered_error, &c);
    let mut predicted_filtered = cross(&filtered, &consider);
    predicted_filtered[(0, 0)] = dot(&predicted_error, &filtered_error);
    let mut next_prediction = covariance();
    next_prediction.nav[(0, 0)] = dot(&next_predicted_error, &next_predicted_error);
    next_prediction.nav_consider[(0, 0)] = dot(&next_predicted_error, &c);
    let mut terminal = RtsEstimate::new();
    terminal.covariance.copy_from(&next_prediction);
    terminal.covariance.nav[(0, 0)] = dot(&final_error, &final_error);
    terminal.covariance.nav_consider[(0, 0)] = dot(&final_error, &c);
    terminal.predicted_smoothed_cross = cross(&terminal.covariance, &consider);
    terminal.predicted_smoothed_cross[(0, 0)] = dot(&next_predicted_error, &final_error);
    let mut middle_state = at(1);
    middle_state.position_n.x = gain1 * 1.2;
    let mut next_predicted_state = at(2);
    next_predicted_state.position_n.x = middle_state.position_n.x;
    terminal.state = next_predicted_state;
    terminal.state.position_n.x += gain2 * (-0.7 - next_predicted_state.position_n.x);
    let mut edge = transition();
    edge.nav[(0, CONSIDER_START)] = 1.0;
    let mut middle = RtsEstimate::new();
    let mut scratch = RtsScratch::new();
    backward_step(
        &RtsStep {
            filtered_state: &middle_state,
            filtered: &filtered,
            predicted: &predicted,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &predicted_filtered,
            next_transition: &edge,
            next_predicted_state: &next_predicted_state,
            next_predicted: &next_prediction,
            consider: &consider,
            active_consider: 1,
        },
        &terminal,
        &mut middle,
        &mut scratch,
    )
    .unwrap();
    assert_close(
        middle.state.position_n.x,
        middle_state.position_n.x
            + middle_smoother_gain
                * (terminal.state.position_n.x - next_predicted_state.position_n.x),
        2.0e-6,
    );
    assert_close(
        middle.covariance.nav[(0, 0)],
        dot(&smoothed_error, &smoothed_error),
        2.0e-6,
    );
    assert_close(
        middle.covariance.nav_consider[(0, 0)],
        dot(&smoothed_error, &c),
        2.0e-6,
    );
    assert_close(
        middle.predicted_smoothed_cross[(0, 0)],
        dot(&predicted_error, &smoothed_error),
        2.0e-6,
    );
    assert_close(
        middle.predicted_smoothed_cross[(CONSIDER_START, 0)],
        dot(&c, &smoothed_error),
        2.0e-6,
    );
    let mut initial = RtsEstimate::new();
    backward_step(
        &RtsStep {
            filtered_state: &at(0),
            filtered: &initial_covariance,
            predicted: &initial_covariance,
            filtered_reset: &Matrix3::identity(),
            predicted_filtered_cross: &initial_cross,
            next_transition: &edge,
            next_predicted_state: &at(1),
            next_predicted: &predicted,
            consider: &consider,
            active_consider: 1,
        },
        &middle,
        &mut initial,
        &mut scratch,
    )
    .unwrap();
    assert_close(
        initial.state.position_n.x,
        initial_smoother_gain * middle.state.position_n.x,
        3.0e-6,
    );
    assert_close(
        initial.covariance.nav[(0, 0)],
        dot(&initial_smoothed_error, &initial_smoothed_error),
        3.0e-6,
    );
    assert_close(
        initial.covariance.nav_consider[(0, 0)],
        dot(&initial_smoothed_error, &c),
        3.0e-6,
    );
}
