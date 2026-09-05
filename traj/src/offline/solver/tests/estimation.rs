use super::*;

#[test]
fn schmidt_update_clamps_consider_mean_and_covariance() {
    let mut nominal = nominal(0, 0.0);
    let mut covariance = covariance(1.0, 2.0);
    covariance.state_consider[(POSITION, 0)] = 0.2;
    let consider = DMatrix::identity(1, 1) * 2.0;
    let mut h_state = DMatrix::zeros(1, NAVIGATION_DIMENSION);
    h_state[(0, POSITION)] = 1.0;
    let h_consider = DMatrix::from_element(1, 1, 1.0);
    let mut residual = DVector::from_element(1, 1.0);
    let mut repairs = 0;
    let outcome = schmidt_update(
        &mut nominal,
        &mut covariance,
        &h_state,
        &h_consider,
        &consider,
        &mut residual,
        &DMatrix::from_element(1, 1, 0.5),
        4.0,
        25.0,
        10.0,
        3,
        1.0e-6,
        &mut repairs,
    )
    .unwrap();
    assert_eq!(outcome.disposition, InputDisposition::Fused);
    assert_ne!(covariance.state_consider[(POSITION, 0)], 0.2);
    assert!(nominal.position_ecef[0] > 6_378_137.0);
}

#[test]
fn consider_rts_matches_direct_linear_gaussian_schmidt_update() {
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let consider = DMatrix::identity(1, 1);
    let mut prior = covariance(1.0, 1.0);
    prior.state[(POSITION, POSITION)] = 2.0;
    prior.state_consider[(POSITION, 0)] = 0.4;
    let mut transition = DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    transition[(POSITION, POSITION)] = 0.8;
    let mut consider_transition = DMatrix::zeros(NAVIGATION_DIMENSION, 1);
    consider_transition[(POSITION, 0)] = 0.3;
    let process = DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION) * 0.5;
    let predicted = StoredCovariance {
        state: symmetric(
            &transition * &prior.state * transition.transpose()
                + &transition * &prior.state_consider * consider_transition.transpose()
                + &consider_transition * prior.state_consider.transpose() * transition.transpose()
                + &consider_transition * &consider * consider_transition.transpose()
                + &process,
        ),
        state_consider: &transition * &prior.state_consider + &consider_transition * &consider,
    };
    let mut terminal_nominal = nominal(1_000_000_000, 0.0);
    let mut terminal = predicted.clone();
    let mut h_state = DMatrix::zeros(1, NAVIGATION_DIMENSION);
    h_state[(0, POSITION)] = 1.0;
    let h_consider = DMatrix::zeros(1, 1);
    let mut residual = DVector::zeros(1);
    let mut repairs = 0;
    schmidt_update(
        &mut terminal_nominal,
        &mut terminal,
        &h_state,
        &h_consider,
        &consider,
        &mut residual,
        &DMatrix::from_element(1, 1, 0.7),
        10.0,
        100.0,
        1.0,
        3,
        1.0e-6,
        &mut repairs,
    )
    .unwrap();

    // Independent reference: apply the same future scalar observation
    // directly to [x_0, c]. Process noise and observation noise combine.
    let mut expected_nominal = nominal(0, 0.0);
    let mut expected = prior.clone();
    let mut effective_h_state = DMatrix::zeros(1, NAVIGATION_DIMENSION);
    effective_h_state[(0, POSITION)] = 0.8;
    let effective_h_consider = DMatrix::from_element(1, 1, 0.3);
    let mut expected_residual = DVector::zeros(1);
    schmidt_update(
        &mut expected_nominal,
        &mut expected,
        &effective_h_state,
        &effective_h_consider,
        &consider,
        &mut expected_residual,
        &DMatrix::from_element(1, 1, 1.2),
        10.0,
        100.0,
        1.0,
        3,
        1.0e-6,
        &mut repairs,
    )
    .unwrap();

    let mut first = step(0, 0.0, 1.0, 1.0);
    first.predicted_covariance = prior.clone();
    first.filtered_covariance = prior;
    let mut second = step(1_000_000_000, 0.0, 1.0, 1.0);
    second.predicted_covariance = predicted;
    second.filtered_covariance = terminal;
    second.transition = transition;
    second.consider_transition = consider_transition;
    second.process_covariance = process;
    second.adjacent_cross_covariance = &first.filtered_covariance.state
        * second.transition.transpose()
        + &first.filtered_covariance.state_consider * second.consider_transition.transpose();
    let mut planned = plan_store(NAVIGATION_DIMENSION, &consider, 2, limits).unwrap();
    planned.store.push(&first).unwrap();
    planned.store.push(&second).unwrap();
    let catalog = ConsiderCatalog {
        parameters: Vec::new(),
        clocks: Vec::new(),
        covariance: consider,
    };
    let control = RunControl {
        continue_running: &continue_running,
        progress: &report_progress,
    };
    let mut work = WorkTracker::new(control, None, 2, 0);
    smooth_store(planned.store.as_mut(), &catalog, 3, 1.0e-6, &mut work).unwrap();
    let smoothed = planned.store.get(0).unwrap();
    let actual = smoothed.smoothed_covariance.unwrap();
    assert!(
        (actual.state[(POSITION, POSITION)] - expected.state[(POSITION, POSITION)]).abs() < 1.0e-10,
        "state variance: actual={}, expected={}",
        actual.state[(POSITION, POSITION)],
        expected.state[(POSITION, POSITION)]
    );
    assert!(
        (actual.state_consider[(POSITION, 0)] - expected.state_consider[(POSITION, 0)]).abs()
            < 1.0e-10,
        "state/consider: actual={}, expected={}",
        actual.state_consider[(POSITION, 0)],
        expected.state_consider[(POSITION, 0)]
    );
    let gain = smoothed.smoothed_backward_gain.unwrap();
    assert!((gain[(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION)] - 1.0).abs() < 1.0e-12);
}

#[test]
fn affine_schmidt_update_uses_the_complete_reference_residual() {
    let reference = nominal(0, 0.0);
    let mut current = nominal(0, 2.0);
    let mut covariance = covariance(1.0, 1.0);
    let consider = DMatrix::identity(1, 1);
    let mut h_state = DMatrix::zeros(1, NAVIGATION_DIMENSION);
    h_state[(0, POSITION)] = 1.0;
    let h_consider = DMatrix::zeros(1, 1);
    // z - h(reference) = 10.  Since the prior mean is two metres from the
    // reference, the affine innovation is 10 - H*2 = 8.  With P=R=1,
    // the posterior guide-coordinate mean is 2 + 0.5*8 = 6.
    let mut residual = DVector::from_element(1, 10.0);
    let mut repairs = 0;
    schmidt_update_affine(
        &mut current,
        &mut covariance,
        &h_state,
        &h_consider,
        &consider,
        &mut residual,
        &DMatrix::identity(1, 1),
        100.0,
        1_000.0,
        10.0,
        3,
        1.0e-6,
        &mut repairs,
        Some(&reference),
        1.0,
    )
    .unwrap();
    assert!((residual[0] - 8.0).abs() < 1.0e-12);
    assert!((current.position_ecef[0] - (6_378_137.0 + 6.0)).abs() < 1.0e-9);
}

#[test]
fn affine_dynamics_retains_the_nonlinear_defect_and_epochwise_offset() {
    let reference_current = nominal(0, 0.0);
    let current = nominal(0, 2.0);
    let reference_next = nominal(1_000_000_000, 10.0);
    let nonlinear_prediction = nominal(0, 11.0);
    let transition = DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    let consider_transition = DMatrix::zeros(NAVIGATION_DIMENSION, 1);
    let process_covariance = DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    let (next, transformed_transition, _, _, transformed_sample) = affine_propagation(
        &current,
        &reference_current,
        &reference_next,
        nonlinear_prediction,
        &transition,
        &consider_transition,
        &process_covariance,
        &DMatrix::zeros(NAVIGATION_DIMENSION, 6),
    )
    .unwrap();
    // One metre of model defect plus the current two-metre guide offset.
    assert!((next.position_ecef[0] - (6_378_137.0 + 13.0)).abs() < 1.0e-9);
    assert!((transformed_transition - transition).norm() < 1.0e-12);
    assert_eq!(transformed_sample, DMatrix::zeros(NAVIGATION_DIMENSION, 6));
}

#[test]
fn ieks_guide_interpolates_every_epoch_but_never_crosses_a_gap() {
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let consider = DMatrix::identity(1, 1);
    let mut planned = plan_store(NAVIGATION_DIMENSION, &consider, 2, limits).unwrap();
    let mut first = step(0, 0.0, 1.0, 1.0);
    first.smoothed = Some(nominal(0, 0.0));
    first.smoothed_covariance = Some(covariance(1.0, 1.0));
    let mut second = step(1_000_000_000, 10.0, 1.0, 1.0);
    second.smoothed = Some(nominal(1_000_000_000, 10.0));
    second.smoothed_covariance = Some(covariance(1.0, 1.0));
    planned.store.push(&first).unwrap();
    planned.store.push(&second).unwrap();

    let midpoint = smoothed_state_at(planned.store.as_mut(), SessionTime::from_ns(500_000_000))
        .unwrap()
        .unwrap();
    assert!((midpoint.position_ecef[0] - (6_378_137.0 + 5.0)).abs() < 1.0e-9);

    second.connected_from_previous = false;
    planned.store.set(1, &second).unwrap();
    assert!(
        smoothed_state_at(planned.store.as_mut(), SessionTime::from_ns(500_000_000))
            .unwrap()
            .is_none()
    );
}

#[test]
fn rts_future_information_corrects_the_earlier_state_without_estimating_consider() {
    let limits = OfflineResourceLimits {
        peak_memory_bytes: 64 * 1_024 * 1_024,
        temporary_storage_bytes: 0,
        output_bytes: 1_024 * 1_024,
        worker_count: 1,
        elapsed_work_limit: None,
    };
    let consider = DMatrix::identity(1, 1);
    let mut planned = plan_store(NAVIGATION_DIMENSION, &consider, 2, limits).unwrap();
    planned.store.push(&step(0, 0.0, 1.0, 1.0)).unwrap();
    planned
        .store
        .push(&step(1_000_000_000, 10.0, 2.0, 0.1))
        .unwrap();
    let catalog = ConsiderCatalog {
        parameters: Vec::new(),
        clocks: Vec::new(),
        covariance: DMatrix::identity(1, 1),
    };
    let control = RunControl {
        continue_running: &continue_running,
        progress: &report_progress,
    };
    let mut work = WorkTracker::new(control, None, 2, 0);
    smooth_store(planned.store.as_mut(), &catalog, 3, 1.0e-6, &mut work).unwrap();
    let first = planned.store.get(0).unwrap();
    let first_smoothed = first.smoothed.unwrap();
    assert!((first_smoothed.position_ecef[0] - (6_378_137.0 + 5.0)).abs() < 1.0e-9);
    assert_eq!(planned.store.consider_covariance(), &consider);
}

#[test]
fn manifold_boxplus_and_boxminus_round_trip() {
    let reference = nominal(0, 0.0);
    let mut correction = DVector::zeros(NAVIGATION_DIMENSION);
    correction[POSITION] = 2.0;
    correction[VELOCITY + 1] = -0.5;
    correction[ATTITUDE + 2] = 0.01;
    correction[GYROSCOPE_BIAS] = 0.001;
    let value = boxplus_with_reset(&reference, &correction).unwrap().0;
    let recovered = boxminus(&value, &reference, NAVIGATION_DIMENSION).unwrap();
    assert!((&recovered - correction).norm() < 1.0e-10);
}

#[test]
fn psd_validation_rejects_asymmetry_without_a_unit_scale_floor() {
    let tiny = 1.0e-240;
    let symmetric_rank_one = DMatrix::from_row_slice(2, 2, &[tiny, tiny, tiny, tiny]);
    assert!(matrix_is_psd(&symmetric_rank_one));

    let asymmetric = DMatrix::from_row_slice(2, 2, &[tiny, tiny, 0.5 * tiny, tiny]);
    assert!(!matrix_is_psd(&asymmetric));

    let indefinite = DMatrix::from_row_slice(2, 2, &[tiny, 2.0 * tiny, 2.0 * tiny, tiny]);
    assert!(!matrix_is_psd(&indefinite));
}
