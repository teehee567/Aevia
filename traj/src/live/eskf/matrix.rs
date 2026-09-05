//! Fixed-size matrix products into caller-owned buffers.

use super::{
    ConsiderCovariance, GapNavCrossCovariance, MAX_CONSIDER, NavConsiderCovariance,
    PreintegrationNavJacobian,
};
use crate::live::{
    preintegration::{BIAS_DIM, PREINT_DIM},
    state::{NAV_DIM, NavMatrix},
};
use nalgebra::SMatrix;

pub(super) fn multiply_nav(left: &NavMatrix, right: &NavMatrix, output: &mut NavMatrix) {
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            for inner in 0..NAV_DIM {
                value += left[(row, inner)] * right[(inner, column)];
            }
            output[(row, column)] = value;
        }
    }
}

pub(super) fn multiply_nav_right_transpose(
    left: &NavMatrix,
    right: &NavMatrix,
    output: &mut NavMatrix,
) {
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            for inner in 0..NAV_DIM {
                value += left[(row, inner)] * right[(column, inner)];
            }
            output[(row, column)] = value;
        }
    }
}

pub(super) fn multiply_nav_consider(
    left: &NavMatrix,
    right: &NavConsiderCovariance,
    output: &mut NavConsiderCovariance,
) {
    for row in 0..NAV_DIM {
        for column in 0..MAX_CONSIDER {
            let mut value = 0.0;
            for inner in 0..NAV_DIM {
                value += left[(row, inner)] * right[(inner, column)];
            }
            output[(row, column)] = value;
        }
    }
}

pub(super) fn multiply_gamma_consider(
    gamma: &NavConsiderCovariance,
    covariance: &ConsiderCovariance,
    active_consider: usize,
    output: &mut NavConsiderCovariance,
) {
    output.fill(0.0);
    for row in 0..NAV_DIM {
        for column in 0..active_consider {
            for inner in 0..active_consider {
                output[(row, column)] += gamma[(row, inner)] * covariance[(inner, column)];
            }
        }
    }
}

pub(super) fn multiply_nav_gap(
    left: &NavMatrix,
    right: &GapNavCrossCovariance,
    output: &mut GapNavCrossCovariance,
) {
    for row in 0..NAV_DIM {
        for column in 0..BIAS_DIM {
            let mut value = 0.0;
            for inner in 0..NAV_DIM {
                value += left[(row, inner)] * right[(inner, column)];
            }
            output[(row, column)] = value;
        }
    }
}

pub(super) fn multiply_preintegration_gap(
    mapping: &PreintegrationNavJacobian,
    jacobian: &SMatrix<f32, PREINT_DIM, BIAS_DIM>,
    output: &mut GapNavCrossCovariance,
) {
    for row in 0..NAV_DIM {
        for column in 0..BIAS_DIM {
            let mut value = 0.0;
            for inner in 0..PREINT_DIM {
                value += mapping[(row, inner)] * jacobian[(inner, column)];
            }
            output[(row, column)] = value;
        }
    }
}

pub(super) fn add_symmetric_cross_product(
    target: &mut NavMatrix,
    left: &GapNavCrossCovariance,
    right: &GapNavCrossCovariance,
) {
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            for latent in 0..BIAS_DIM {
                target[(row, column)] += left[(row, latent)] * right[(column, latent)]
                    + right[(row, latent)] * left[(column, latent)];
            }
        }
    }
}

pub(super) fn symmetrize_nav(matrix: &mut NavMatrix) {
    for row in 0..NAV_DIM {
        for column in (row + 1)..NAV_DIM {
            let value = 0.5 * (matrix[(row, column)] + matrix[(column, row)]);
            matrix[(row, column)] = value;
            matrix[(column, row)] = value;
        }
    }
}
