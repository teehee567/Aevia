//! Independent Gaussian-process oracles for coupled dense conditioning.

use super::*;

const NAV: usize = 15;
const CONSIDER: usize = 2;
const DIM: usize = NAV + CONSIDER + 6;

fn symmetric(matrix: DMatrix<f64>) -> DMatrix<f64> {
    (&matrix + matrix.transpose()) * 0.5
}

/// Van Loan's block exponential, using nalgebra's Padé matrix exponential.
/// This is independent of the production Taylor/Lyapunov coefficient cache.
fn van_loan(f: &DMatrix<f64>, q: &DMatrix<f64>, duration: f64) -> (DMatrix<f64>, DMatrix<f64>) {
    let n = f.nrows();
    let mut block = DMatrix::zeros(2 * n, 2 * n);
    block.view_mut((0, 0), (n, n)).copy_from(f);
    block.view_mut((0, n), (n, n)).copy_from(q);
    block.view_mut((n, n), (n, n)).copy_from(&(-f.transpose()));
    let exponential = (block * duration).exp();
    let transition = exponential.view((0, 0), (n, n)).into_owned();
    let process = exponential.view((0, n), (n, n)) * transition.transpose();
    (transition, symmetric(process))
}

fn fixture(
    duration: f64,
    replace_sample: bool,
    deterministic: bool,
) -> (CoupledDenseBridge, DMatrix<f64>) {
    let mut f = DMatrix::zeros(DIM, DIM);
    let rotation = nalgebra::UnitQuaternion::from_scaled_axis(Vector3::new(0.3, -0.2, 0.5))
        .to_rotation_matrix()
        .into_inner();
    let rate = Vector3::new(0.4, -0.7, 1.2);
    let force = Vector3::new(1.1, -2.0, 8.0);
    f.view_mut((0, 3), (3, 3)).copy_from(&Matrix3::identity());
    f.view_mut((3, 6), (3, 3))
        .copy_from(&(-rotation * skew(force)));
    f.view_mut((3, 9), (3, 3)).copy_from(&(-rotation));
    f.view_mut((6, 6), (3, 3)).copy_from(&(-skew(rate)));
    f.view_mut((6, 12), (3, 3))
        .copy_from(&(-Matrix3::identity()));
    f.view_mut((3, NAV + CONSIDER), (3, 3))
        .copy_from(&(-rotation));
    f.view_mut((6, NAV + CONSIDER + 3), (3, 3))
        .copy_from(&(-Matrix3::identity()));
    f[(3, NAV)] = 0.4;
    f[(4, NAV)] = -0.2;
    f[(6, NAV + 1)] = 0.15;
    f[(8, NAV + 1)] = -0.1;
    let gyro = if deterministic {
        Matrix3::zeros()
    } else {
        Matrix3::new(0.02, 0.003, -0.002, 0.003, 0.03, 0.001, -0.002, 0.001, 0.01)
    };
    let mut q = DMatrix::zeros(DIM, DIM);
    if !deterministic {
        let acceleration =
            rotation * Matrix3::from_diagonal(&Vector3::new(0.2, 0.1, 0.15)) * rotation.transpose();
        q.view_mut((3, 3), (3, 3)).copy_from(&acceleration);
        q.view_mut((6, 6), (3, 3)).copy_from(&gyro);
        for axis in 0..3 {
            q[(9 + axis, 9 + axis)] = 0.003;
            q[(12 + axis, 12 + axis)] = 0.0005;
        }
    }
    q = symmetric(q);
    // The starting sample is already correlated with navigation and fixed
    // calibration: this represents an interior cut after an earlier update.
    let mut factor = DMatrix::identity(DIM, DIM) * 0.4;
    factor[(0, NAV)] = 0.2;
    factor[(4, NAV + CONSIDER)] = -0.12;
    factor[(7, NAV + CONSIDER + 4)] = 0.15;
    factor[(NAV + CONSIDER + 2, NAV + 1)] = 0.1;
    let prior = &factor * factor.transpose();
    let (phi, process) = van_loan(&f, &q, duration);
    let mut cross = &prior * phi.transpose();
    let mut end = &phi * &prior * phi.transpose() + process;
    if replace_sample {
        cross.columns_mut(NAV + CONSIDER, 6).fill(0.0);
        end.rows_mut(NAV + CONSIDER, 6).fill(0.0);
        end.columns_mut(NAV + CONSIDER, 6).fill(0.0);
        for axis in 0..6 {
            end[(NAV + CONSIDER + axis, NAV + CONSIDER + axis)] = 1.7;
        }
    }
    let mut endpoints = DMatrix::zeros(2 * DIM, 2 * DIM);
    endpoints.view_mut((0, 0), (DIM, DIM)).copy_from(&prior);
    endpoints.view_mut((0, DIM), (DIM, DIM)).copy_from(&cross);
    endpoints
        .view_mut((DIM, 0), (DIM, DIM))
        .copy_from(&cross.transpose());
    endpoints.view_mut((DIM, DIM), (DIM, DIM)).copy_from(&end);
    let model = CoupledDenseBridge {
        duration_seconds: duration,
        state_dimension: NAV,
        continuous: f,
        noise_density: q,
        endpoint_joint: symmetric(endpoints),
        start_to_reference: DMatrix::identity(DIM, DIM),
        end_to_reference: DMatrix::identity(DIM, DIM),
        reference_start_orientation: [1.0, 0.0, 0.0, 0.0],
        reference_body_rate: [0.4, -0.7, 1.2],
        rate_mapping: DMatrix::zeros(3, DIM),
        gyro_density: core::array::from_fn(|r| core::array::from_fn(|c| gyro[(r, c)])),
        parameter_ids: std::vec![0;DIM],
        cache: FlowCache::default(),
    };
    model.validate().unwrap();
    (model, prior)
}

fn augmented_process(model: &CoupledDenseBridge) -> (DMatrix<f64>, DMatrix<f64>) {
    let mut f = DMatrix::zeros(DIM + 3, DIM + 3);
    f.view_mut((0, 0), (DIM, DIM)).copy_from(&model.continuous);
    let mut q = DMatrix::zeros(DIM + 3, DIM + 3);
    q.view_mut((0, 0), (DIM, DIM))
        .copy_from(&model.noise_density);
    for r in 0..3 {
        for c in 0..3 {
            q[(DIM + r, DIM + c)] = model.gyro_density[r][c];
            q[(6 + r, DIM + c)] = -model.gyro_density[r][c];
            q[(DIM + c, 6 + r)] = -model.gyro_density[r][c];
        }
    }
    (f, q)
}

/// Joint prior of [z(t),r(T)] and [z(u),r(T)], with T the full edge.
fn prior_output_cross(
    model: &CoupledDenseBridge,
    prior: &DMatrix<f64>,
    first: f64,
    second: f64,
) -> DMatrix<f64> {
    if first > second {
        return prior_output_cross(model, prior, second, first).transpose();
    }
    let (f, q) = augmented_process(model);
    let (left, left_q) = van_loan(&f, &q, first);
    let (right, right_q) = van_loan(&f, &q, second);
    let (between, _) = van_loan(&f, &q, second - first);
    let (_, terminal_q) = van_loan(&f, &q, model.duration_seconds);
    let mut initial = DMatrix::zeros(DIM + 3, DIM + 3);
    initial.view_mut((0, 0), (DIM, DIM)).copy_from(prior);
    let mut result = &left * initial * right.transpose() + &left_q * between.transpose();
    result
        .view_mut((0, DIM), (DIM, 3))
        .copy_from(&left_q.view((0, DIM), (DIM, 3)));
    result
        .view_mut((DIM, 0), (3, DIM))
        .copy_from(&right_q.view((DIM, 0), (3, DIM)));
    result
        .view_mut((DIM, DIM), (3, 3))
        .copy_from(&terminal_q.view((DIM, DIM), (3, 3)));
    result
}

fn assert_matrix_close(actual: &DMatrix<f64>, expected: &DMatrix<f64>) {
    assert_eq!(actual.shape(), expected.shape());
    let error = (actual - expected).amax();
    assert!(
        error < 2.0e-10 * expected.amax().max(1.0e-5),
        "matrix error {error:e}, scale {:e}",
        expected.amax()
    );
}

#[test]
fn cached_flow_matches_independent_van_loan_discretization() {
    for duration in [0.0001, 0.03, 1.4] {
        let (model, _) = fixture(duration, false, false);
        let (f, q) = augmented_process(&model);
        for parameter in [0.0, 0.013, 0.37, 0.81, 1.0] {
            let time = duration * parameter;
            let (expected_transition, expected_process) = van_loan(&f, &q, time);
            let (transition, process) = model.discretize(time).unwrap();
            assert_matrix_close(&transition, &expected_transition);
            assert_matrix_close(&process, &expected_process);
            let repeated = model.discretize(time).unwrap();
            assert_eq!(repeated, (transition, process));
        }
    }
}

#[test]
fn coupled_conditioning_recovers_unconditional_prior_with_shared_or_replaced_sample() {
    for replaced in [false, true] {
        for deterministic in [false, true] {
            let (model, prior) = fixture(0.15, replaced, deterministic);
            for parameter in [0.0, 0.13, 0.53, 0.91, 1.0] {
                let expected =
                    prior_output_cross(&model, &prior, 0.15 * parameter, 0.15 * parameter);
                let actual = model.output_covariance(0.15, parameter).unwrap();
                assert_matrix_close(&actual, &expected);
            }
            let first = model.linearization(0.15, 0.2).unwrap();
            let second = model.linearization(0.15, 0.7).unwrap();
            let mut left = DMatrix::zeros(DIM + 3, 2 * DIM);
            let mut right = DMatrix::zeros(DIM + 3, 2 * DIM);
            left.columns_mut(0, DIM).copy_from(&first.start_jacobian);
            left.columns_mut(DIM, DIM).copy_from(&first.end_jacobian);
            right.columns_mut(0, DIM).copy_from(&second.start_jacobian);
            right.columns_mut(DIM, DIM).copy_from(&second.end_jacobian);
            let actual = left * &model.endpoint_joint * right.transpose()
                + model.conditional_cross(0.15, 0.2, 0.7).unwrap();
            assert_matrix_close(&actual, &prior_output_cross(&model, &prior, 0.03, 0.105));
        }
    }
}

#[test]
fn coupled_conditioning_matches_batch_future_observation_for_gyro_integral_and_sample() {
    for replaced in [false, true] {
        let (mut model, prior) = fixture(0.2, replaced, false);
        // A future attitude observation informs old held gyro error and
        // the same white gyro integral used by offset-point velocity.
        let coordinate = DIM + 7;
        let variance = model.endpoint_joint[(coordinate, coordinate)] + 0.03;
        let cross = model.endpoint_joint.column(coordinate).into_owned();
        model.endpoint_joint =
            symmetric(&model.endpoint_joint - &cross * cross.transpose() / variance);
        for parameter in [0.0, 0.31, 0.8, 1.0] {
            let predicted = prior_output_cross(&model, &prior, 0.2 * parameter, 0.2 * parameter);
            let cross = prior_output_cross(&model, &prior, 0.2 * parameter, 0.2)
                .column(7)
                .into_owned();
            let expected = predicted - &cross * cross.transpose() / variance;
            assert_matrix_close(&model.output_covariance(0.2, parameter).unwrap(), &expected);
        }
    }
}

#[test]
fn coupled_endpoint_covariances_survive_noncommuting_nominal_tangent_resets() {
    let (mut model, _) = fixture(0.3, true, false);
    let corrections = [
        Vector3::new(0.25, -0.18, 0.1),
        Vector3::new(-0.12, 0.22, 0.19),
    ];
    let mut joint_reset = DMatrix::identity(2 * DIM, 2 * DIM);
    for (endpoint, correction) in corrections.iter().enumerate() {
        let reset = right_jacobian(*correction);
        joint_reset
            .view_mut((endpoint * DIM + 6, endpoint * DIM + 6), (3, 3))
            .copy_from(&reset);
        let inverse = reset.try_inverse().unwrap();
        if endpoint == 0 {
            model
                .start_to_reference
                .view_mut((6, 6), (3, 3))
                .copy_from(&inverse);
        } else {
            model
                .end_to_reference
                .view_mut((6, 6), (3, 3))
                .copy_from(&inverse);
        }
    }
    model.endpoint_joint =
        symmetric(&joint_reset * &model.endpoint_joint * joint_reset.transpose());
    let reference = ReferencePoint::new(
        crate::ids::ReferencePointId::new(1),
        ReferencePointKind::ImuSensingCenter,
        crate::frame::BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
        crate::ids::SharedParameterId::new(1),
        crate::uncertainty::MeasurementUncertainty::Provided(
            crate::uncertainty::Covariance3::diagonal(0.0, 0.0, 0.0).unwrap(),
        ),
    );
    let rotation = |value: [f64; 3]| {
        crate::math::UnitQuaternion::from_rotation_vector(
            crate::math::Vector3::from_components(value).unwrap(),
        )
        .unwrap()
    };
    for endpoint in 0..2 {
        let parameter = endpoint as f64;
        let earth = rotation([
            0.0,
            0.0,
            -7.292_115_0e-5 * model.duration_seconds * parameter,
        ]);
        let body = rotation(
            model
                .reference_body_rate
                .map(|rate| rate * model.duration_seconds * parameter),
        );
        let reference_orientation = earth.multiply(body);
        let correction = corrections[endpoint];
        let orientation =
            reference_orientation.multiply(rotation([correction.x, correction.y, correction.z]));
        let base = BaseKinematics {
            position: [0.0; 3],
            velocity: [0.0; 3],
            acceleration: [0.0; 3],
            orientation,
            angular_rate_body: [0.0; 3],
            angular_acceleration_body: [0.0; 3],
            specific_force_body: [0.0; 3],
        };
        let projection = model
            .point_projection(model.duration_seconds, parameter, &base, reference)
            .unwrap();
        let covariance = &projection
            * model
                .output_covariance(model.duration_seconds, parameter)
                .unwrap()
            * projection.transpose();
        let expected = model
            .endpoint_joint
            .view((endpoint * DIM, endpoint * DIM), (9, 9))
            .into_owned();
        assert_matrix_close(&covariance, &expected);
    }
}
