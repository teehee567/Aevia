//! Bounded extended Rauch--Tung--Striebel smoothing for the live Schmidt filter.
//!
//! The usual RTS covariance difference assumes an optimal forward filter. The
//! live filter deliberately gives shared parameters and retained IMU/gap errors
//! zero measurement gain. Its smoother therefore also retains
//! `M = Cov(predicted_error, smoothed_navigation_error)`. This extra statistic
//! gives the covariance of the next navigation correction without treating
//! fixed-mean nuisance errors as independent noise or as estimated parameters.
//!
//! At each backward step the navigation gain minimizes mean squared error with
//! respect to the retained next navigation correction. For an ordinary Kalman
//! filter this reduces to RTS. With Schmidt nuisance states it is a constrained
//! smoother; it cannot recover information discarded by the forward estimator.
//! Every matrix and temporary is fixed size and caller owned.

#[cfg(test)]
mod tests;

use nalgebra::{ArrayStorage, Matrix3, SMatrix};

use super::{
    eskf::{ConsiderCovariance, GapNavCrossCovariance, MAX_CONSIDER, NavConsiderCovariance},
    preintegration::{BIAS_DIM, GapDerivativeCovariance, ImuSampleCovariance},
    state::{ACC_BIAS, ATT, GYRO_BIAS, NAV_DIM, NavMatrix, NavState, NavVector, POS, VEL, so3_log},
};

pub(crate) const CONSIDER_START: usize = NAV_DIM;
pub(crate) const SAMPLE_START: usize = CONSIDER_START + MAX_CONSIDER;
pub(crate) const GAP_START: usize = SAMPLE_START + BIAS_DIM;
pub(crate) const AUG_DIM: usize = GAP_START + BIAS_DIM;

pub(crate) type AugmentedNavCross = SMatrix<f32, AUG_DIM, NAV_DIM>;
pub(crate) type NavAugmentedMatrix = SMatrix<f32, NAV_DIM, AUG_DIM>;
type AugmentedMatrix = SMatrix<f32, AUG_DIM, AUG_DIM>;

/// Compact joint covariance in [navigation, shared parameter, held sample, gap]
/// order. Nuisance means have zero gain, so their mutual cross blocks stay zero.
/// The shared-parameter marginal is common to the whole window and supplied
/// separately; a change to that marginal must terminate the window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RtsCovariance {
    pub(crate) nav: NavMatrix,
    pub(crate) nav_consider: NavConsiderCovariance,
    pub(crate) nav_sample: GapNavCrossCovariance,
    pub(crate) nav_gap: GapNavCrossCovariance,
    pub(crate) sample: ImuSampleCovariance,
    pub(crate) gap: GapDerivativeCovariance,
}

impl RtsCovariance {
    pub(crate) const fn new() -> Self {
        Self {
            nav: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_consider: NavConsiderCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; MAX_CONSIDER],
            )),
            nav_sample: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; BIAS_DIM],
            )),
            nav_gap: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; BIAS_DIM],
            )),
            sample: ImuSampleCovariance::from_array_storage(ArrayStorage(
                [[0.0; BIAS_DIM]; BIAS_DIM],
            )),
            gap: GapDerivativeCovariance::from_array_storage(ArrayStorage(
                [[0.0; BIAS_DIM]; BIAS_DIM],
            )),
        }
    }

    pub(crate) fn copy_from(&mut self, source: &Self) {
        self.nav.copy_from(&source.nav);
        self.nav_consider.copy_from(&source.nav_consider);
        self.nav_sample.copy_from(&source.nav_sample);
        self.nav_gap.copy_from(&source.nav_gap);
        self.sample.copy_from(&source.sample);
        self.gap.copy_from(&source.gap);
    }

    pub(crate) fn joint_entry(
        &self,
        row: usize,
        column: usize,
        consider: &ConsiderCovariance,
    ) -> f32 {
        let (row, column) = if row > column {
            (column, row)
        } else {
            (row, column)
        };
        match (row, column) {
            (0..NAV_DIM, 0..NAV_DIM) => self.nav[(row, column)],
            (0..NAV_DIM, CONSIDER_START..SAMPLE_START) => {
                self.nav_consider[(row, column - CONSIDER_START)]
            }
            (0..NAV_DIM, SAMPLE_START..GAP_START) => self.nav_sample[(row, column - SAMPLE_START)],
            (0..NAV_DIM, GAP_START..AUG_DIM) => self.nav_gap[(row, column - GAP_START)],
            (CONSIDER_START..SAMPLE_START, CONSIDER_START..SAMPLE_START) => {
                consider[(row - CONSIDER_START, column - CONSIDER_START)]
            }
            (SAMPLE_START..GAP_START, SAMPLE_START..GAP_START) => {
                self.sample[(row - SAMPLE_START, column - SAMPLE_START)]
            }
            (GAP_START..AUG_DIM, GAP_START..AUG_DIM) => {
                self.gap[(row - GAP_START, column - GAP_START)]
            }
            _ => 0.0,
        }
    }
}

/// Complete Markov transition. The navigation rows include the sensitivities
/// to shared parameters and to any sample/gap already present at the left end.
/// A new independent nuisance at the right end has a zero persistence flag.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RtsTransition {
    pub(crate) nav: NavAugmentedMatrix,
    pub(crate) retain_sample: bool,
    pub(crate) retain_gap: bool,
}

impl RtsTransition {
    pub(crate) const fn new() -> Self {
        Self {
            nav: NavAugmentedMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; AUG_DIM])),
            retain_sample: false,
            retain_gap: false,
        }
    }
}

#[derive(Debug, PartialEq)]
pub(crate) struct RtsEstimate {
    pub(crate) state: NavState,
    pub(crate) covariance: RtsCovariance,
    /// Product of actual forward and backward attitude injection resets.
    /// It cannot in general be reconstructed from the total quaternion delta.
    pub(crate) predicted_to_smoothed_reset: Matrix3<f32>,
    /// Left coordinates: the epoch's original predicted tangent. Right
    /// coordinates: `state`'s smoothed tangent.
    pub(crate) predicted_smoothed_cross: AugmentedNavCross,
}

impl RtsEstimate {
    pub(crate) const fn new() -> Self {
        Self {
            state: NavState::placeholder(),
            covariance: RtsCovariance::new(),
            predicted_to_smoothed_reset: Matrix3::from_array_storage(ArrayStorage([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ])),
            predicted_smoothed_cross: AugmentedNavCross::from_array_storage(ArrayStorage(
                [[0.0; AUG_DIM]; NAV_DIM],
            )),
        }
    }
}

pub(crate) struct RtsStep<'a> {
    pub(crate) filtered_state: &'a NavState,
    pub(crate) filtered: &'a RtsCovariance,
    pub(crate) predicted: &'a RtsCovariance,
    /// Cov(original predicted augmented error, final filtered navigation error).
    /// Capture every sequential measurement update and its attitude reset.
    /// Nuisance columns of the full cross are unchanged predicted columns.
    pub(crate) predicted_filtered_cross: &'a AugmentedNavCross,
    pub(crate) filtered_reset: &'a Matrix3<f32>,
    pub(crate) next_transition: &'a RtsTransition,
    pub(crate) next_predicted_state: &'a NavState,
    pub(crate) next_predicted: &'a RtsCovariance,
    pub(crate) consider: &'a ConsiderCovariance,
    pub(crate) active_consider: usize,
}

/// Cold PSRAM workspace. No large matrix is returned by value or put on the
/// task stack. Output is a candidate: callers discard it if a step fails.
#[derive(Debug, PartialEq)]
pub(crate) struct RtsScratch {
    factor: PsdFactor,
    work: AugmentedNavCross,
    transport: AugmentedNavCross,
    cross: AugmentedNavCross,
    predicted_cross: AugmentedNavCross,
    difference_covariance: NavMatrix,
    next_reference_covariance: NavMatrix,
    gain: NavMatrix,
    nav_work: NavMatrix,
}

impl RtsScratch {
    pub(crate) const fn new() -> Self {
        Self {
            factor: PsdFactor::new(),
            work: AugmentedNavCross::from_array_storage(ArrayStorage([[0.0; AUG_DIM]; NAV_DIM])),
            transport: AugmentedNavCross::from_array_storage(ArrayStorage(
                [[0.0; AUG_DIM]; NAV_DIM],
            )),
            cross: AugmentedNavCross::from_array_storage(ArrayStorage([[0.0; AUG_DIM]; NAV_DIM])),
            predicted_cross: AugmentedNavCross::from_array_storage(ArrayStorage(
                [[0.0; AUG_DIM]; NAV_DIM],
            )),
            difference_covariance: NavMatrix::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; NAV_DIM],
            )),
            next_reference_covariance: NavMatrix::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; NAV_DIM],
            )),
            gain: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_work: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SmoothingError {
    InvalidCovariance,
    NumericalFailure,
    InvalidState,
}

/// One backward pass step. Terminal initialization is exactly the last forward
/// state/covariance and its predicted-to-filtered cross; no endpoint correction
/// or measurement replay is performed here.
#[inline(never)]
pub(crate) fn backward_step(
    step: &RtsStep<'_>,
    next: &RtsEstimate,
    output: &mut RtsEstimate,
    scratch: &mut RtsScratch,
) -> Result<(), SmoothingError> {
    if step.active_consider > MAX_CONSIDER
        || !step.filtered_state.is_finite()
        || !step.next_predicted_state.is_finite()
        || !next.state.is_finite()
        || next.state.time != step.next_predicted_state.time
    {
        return Err(SmoothingError::InvalidState);
    }
    let residual = boxminus(&next.state, step.next_predicted_state);
    let inverse_reset = next
        .predicted_to_smoothed_reset
        .try_inverse()
        .ok_or(SmoothingError::NumericalFailure)?;
    transport_navigation_covariance(
        &next.covariance.nav,
        &inverse_reset,
        &mut scratch.next_reference_covariance,
    );
    // V = Cov(e_next_pred, d), d = e_next_pred_nav - e_next_smooth_nav.
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            let reference_cross = right_transported_entry(
                &next.predicted_smoothed_cross,
                row,
                column,
                &inverse_reset,
            );
            scratch.work[(row, column)] =
                step.next_predicted.joint_entry(row, column, step.consider) - reference_cross;
        }
    }
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            // Ppred + Psmooth - M - M' = V + V' + Psmooth - Ppred.
            scratch.difference_covariance[(row, column)] = scratch.work[(row, column)]
                + scratch.work[(column, row)]
                + scratch.next_reference_covariance[(row, column)]
                - step.next_predicted.nav[(row, column)];
        }
    }
    symmetrize(&mut scratch.difference_covariance);
    scratch.factor.factor(AUG_DIM, |row, column| {
        if inactive_consider(row, step.active_consider)
            || inactive_consider(column, step.active_consider)
        {
            0.0
        } else {
            step.next_predicted.joint_entry(row, column, step.consider)
        }
    })?;
    scratch.factor.solve(&mut scratch.work, NAV_DIM)?;

    // F' Ppred^+ V. The nuisance transition is either identity (same latent)
    // or zero (an ended/replaced sample or gap).
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            for nav in 0..NAV_DIM {
                let coefficient = step.next_transition.nav[(nav, row)];
                if coefficient != 0.0 {
                    value += coefficient * scratch.work[(nav, column)];
                }
            }
            let retained = (CONSIDER_START..SAMPLE_START).contains(&row)
                || ((SAMPLE_START..GAP_START).contains(&row) && step.next_transition.retain_sample)
                || ((GAP_START..AUG_DIM).contains(&row) && step.next_transition.retain_gap);
            if retained {
                value += scratch.work[(row, column)];
            }
            scratch.transport[(row, column)] = value;
        }
    }
    for row in 0..AUG_DIM {
        // A zero-variance nuisance has a zero row in a valid joint covariance.
        // Do not spend MCU cycles multiplying the unused fixed-capacity axes.
        if row >= NAV_DIM && step.filtered.joint_entry(row, row, step.consider) == 0.0 {
            for column in 0..NAV_DIM {
                scratch.cross[(row, column)] = 0.0;
                scratch.predicted_cross[(row, column)] = 0.0;
            }
            continue;
        }
        for column in 0..NAV_DIM {
            let mut filtered_cross = 0.0;
            let mut predicted_cross = 0.0;
            for source in 0..AUG_DIM {
                let value = scratch.transport[(source, column)];
                if value == 0.0 {
                    continue;
                }
                filtered_cross += step.filtered.joint_entry(row, source, step.consider) * value;
                let prior_filtered = if source < NAV_DIM {
                    step.predicted_filtered_cross[(row, source)]
                } else {
                    step.predicted.joint_entry(row, source, step.consider)
                };
                predicted_cross += prior_filtered * value;
            }
            scratch.cross[(row, column)] = filtered_cross;
            scratch.predicted_cross[(row, column)] = predicted_cross;
        }
    }

    // Only navigation has smoothing gain. D is often rank deficient (a single
    // GNSS position update has at most rank three), so a PSD solve is required.
    // D subtracts full-sized covariances. Its rounding error scales with
    // those inputs, not its own nearly-zero diagonals. Use prediction scales
    // so a one-ulp negative remainder on an unobserved axis is recognized as
    // lost information. The original D remains in the Joseph calculation.
    scratch.factor.factor_with_reference(
        NAV_DIM,
        |row, column| scratch.difference_covariance[(row, column)],
        |axis| step.next_predicted.nav[(axis, axis)],
        128.0 * f32::EPSILON,
    )?;
    scratch.work.fill(0.0);
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            scratch.work[(row, column)] = scratch.cross[(column, row)];
        }
    }
    scratch
        .factor
        .solve_with_roundoff(&mut scratch.work, NAV_DIM, |column| {
            64.0 * f32::EPSILON * crate::scalar_math::sqrt(step.filtered.nav[(column, column)])
        })?;
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            scratch.gain[(row, column)] = scratch.work[(column, row)];
        }
    }
    let mut correction = NavVector::zeros();
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            correction[row] += scratch.gain[(row, column)] * residual[column];
        }
    }
    output.state = *step.filtered_state;
    let reset = output
        .state
        .inject(&correction)
        .map_err(|_| SmoothingError::InvalidState)?;
    output.predicted_to_smoothed_reset = reset * step.filtered_reset;
    output.covariance.copy_from(step.filtered);
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            for source in 0..NAV_DIM {
                value +=
                    scratch.gain[(row, source)] * scratch.difference_covariance[(source, column)];
            }
            scratch.nav_work[(row, column)] = value;
        }
    }
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = step.filtered.nav[(row, column)];
            for source in 0..NAV_DIM {
                value += -scratch.gain[(row, source)] * scratch.cross[(column, source)]
                    - scratch.cross[(row, source)] * scratch.gain[(column, source)]
                    + scratch.nav_work[(row, source)] * scratch.gain[(column, source)];
            }
            scratch.next_reference_covariance[(row, column)] = value;
        }
        for nuisance in NAV_DIM..AUG_DIM {
            let mut value = step.filtered.joint_entry(row, nuisance, step.consider);
            for source in 0..NAV_DIM {
                value -= scratch.gain[(row, source)] * scratch.cross[(nuisance, source)];
            }
            set_navigation_nuisance(&mut output.covariance, row, nuisance, value);
        }
    }
    transport_navigation_covariance(
        &scratch.next_reference_covariance,
        &reset,
        &mut output.covariance.nav,
    );
    reset_navigation_cross(&mut output.covariance.nav_consider, &reset);
    reset_navigation_cross(&mut output.covariance.nav_sample, &reset);
    reset_navigation_cross(&mut output.covariance.nav_gap, &reset);
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            let mut value = step.predicted_filtered_cross[(row, column)];
            for source in 0..NAV_DIM {
                value -= scratch.predicted_cross[(row, source)] * scratch.gain[(column, source)];
            }
            scratch.work[(row, column)] = value;
        }
    }
    for row in 0..AUG_DIM {
        for column in 0..NAV_DIM {
            output.predicted_smoothed_cross[(row, column)] =
                right_transported_entry(&scratch.work, row, column, &reset);
        }
    }
    symmetrize(&mut output.covariance.nav);
    if output
        .predicted_smoothed_cross
        .iter()
        .chain(output.covariance.nav.iter())
        .chain(output.covariance.nav_consider.iter())
        .chain(output.covariance.nav_sample.iter())
        .chain(output.covariance.nav_gap.iter())
        .any(|value| !value.is_finite())
    {
        return Err(SmoothingError::NumericalFailure);
    }
    scratch
        .factor
        .factor(NAV_DIM, |row, column| output.covariance.nav[(row, column)])?;
    Ok(())
}

fn inactive_consider(axis: usize, active: usize) -> bool {
    (CONSIDER_START + active..SAMPLE_START).contains(&axis)
}

fn boxminus(value: &NavState, reference: &NavState) -> NavVector {
    let mut result = NavVector::zeros();
    result
        .fixed_rows_mut::<3>(POS)
        .copy_from(&(value.position_n - reference.position_n));
    result
        .fixed_rows_mut::<3>(VEL)
        .copy_from(&(value.velocity_n - reference.velocity_n));
    result
        .fixed_rows_mut::<3>(ACC_BIAS)
        .copy_from(&(value.accel_bias_b - reference.accel_bias_b));
    result
        .fixed_rows_mut::<3>(GYRO_BIAS)
        .copy_from(&(value.gyro_bias_b - reference.gyro_bias_b));
    result.fixed_rows_mut::<3>(ATT).copy_from(&so3_log(
        &(reference.orientation_n_from_b.inverse() * value.orientation_n_from_b),
    ));
    result
}

fn right_transported_entry(
    matrix: &AugmentedNavCross,
    row: usize,
    column: usize,
    reset: &Matrix3<f32>,
) -> f32 {
    if (ATT..ATT + 3).contains(&column) {
        let mut value = 0.0;
        for axis in 0..3 {
            value += matrix[(row, ATT + axis)] * reset[(column - ATT, axis)];
        }
        value
    } else {
        matrix[(row, column)]
    }
}

fn transport_navigation_covariance(
    source: &NavMatrix,
    reset: &Matrix3<f32>,
    target: &mut NavMatrix,
) {
    for row in 0..NAV_DIM {
        for column in 0..NAV_DIM {
            let mut value = 0.0;
            let row_attitude = (ATT..ATT + 3).contains(&row);
            let column_attitude = (ATT..ATT + 3).contains(&column);
            for a in 0..if row_attitude { 3 } else { 1 } {
                let source_row = if row_attitude { ATT + a } else { row };
                let left = if row_attitude {
                    reset[(row - ATT, a)]
                } else {
                    1.0
                };
                for b in 0..if column_attitude { 3 } else { 1 } {
                    let source_column = if column_attitude { ATT + b } else { column };
                    let right = if column_attitude {
                        reset[(column - ATT, b)]
                    } else {
                        1.0
                    };
                    value += left * source[(source_row, source_column)] * right;
                }
            }
            target[(row, column)] = value;
        }
    }
}

fn reset_navigation_cross<const COLUMNS: usize>(
    matrix: &mut SMatrix<f32, NAV_DIM, COLUMNS>,
    reset: &Matrix3<f32>,
) {
    for column in 0..COLUMNS {
        let before = [
            matrix[(ATT, column)],
            matrix[(ATT + 1, column)],
            matrix[(ATT + 2, column)],
        ];
        for row in 0..3 {
            matrix[(ATT + row, column)] =
                (0..3).map(|axis| reset[(row, axis)] * before[axis]).sum();
        }
    }
}

fn set_navigation_nuisance(
    covariance: &mut RtsCovariance,
    row: usize,
    nuisance: usize,
    value: f32,
) {
    if nuisance < SAMPLE_START {
        covariance.nav_consider[(row, nuisance - CONSIDER_START)] = value;
    } else if nuisance < GAP_START {
        covariance.nav_sample[(row, nuisance - SAMPLE_START)] = value;
    } else {
        covariance.nav_gap[(row, nuisance - GAP_START)] = value;
    }
}

fn symmetrize(matrix: &mut NavMatrix) {
    for row in 0..NAV_DIM {
        for column in 0..row {
            let value = 0.5 * (matrix[(row, column)] + matrix[(column, row)]);
            matrix[(row, column)] = value;
            matrix[(column, row)] = value;
        }
    }
}

/// Diagonally scaled, symmetrically pivoted PSD Cholesky. A singular system is
/// solved on its supported subspace; no jitter invents independent uncertainty
/// for inactive or perfectly correlated nuisance axes.
#[derive(Debug, PartialEq)]
struct PsdFactor {
    work: AugmentedMatrix,
    lower: AugmentedMatrix,
    scales: [f32; AUG_DIM],
    permutation: [usize; AUG_DIM],
    intermediate: [f32; AUG_DIM],
    solution: [f32; AUG_DIM],
    dimension: usize,
    rank: usize,
}

impl PsdFactor {
    const fn new() -> Self {
        Self {
            work: AugmentedMatrix::from_array_storage(ArrayStorage([[0.0; AUG_DIM]; AUG_DIM])),
            lower: AugmentedMatrix::from_array_storage(ArrayStorage([[0.0; AUG_DIM]; AUG_DIM])),
            scales: [0.0; AUG_DIM],
            permutation: [0; AUG_DIM],
            intermediate: [0.0; AUG_DIM],
            solution: [0.0; AUG_DIM],
            dimension: 0,
            rank: 0,
        }
    }

    fn factor(
        &mut self,
        dimension: usize,
        entry: impl Fn(usize, usize) -> f32,
    ) -> Result<(), SmoothingError> {
        self.factor_with_reference(
            dimension,
            &entry,
            |axis| entry(axis, axis),
            8.0 * f32::EPSILON,
        )
    }

    fn factor_with_reference(
        &mut self,
        dimension: usize,
        entry: impl Fn(usize, usize) -> f32,
        reference_variance: impl Fn(usize) -> f32,
        rank_tolerance: f32,
    ) -> Result<(), SmoothingError> {
        const NEGATIVE_TOLERANCE: f32 = 128.0 * f32::EPSILON;
        self.dimension = dimension;
        self.rank = 0;
        self.lower.fill(0.0);
        for axis in 0..dimension {
            let variance = reference_variance(axis);
            if !variance.is_finite() || variance < 0.0 {
                return Err(SmoothingError::InvalidCovariance);
            }
            self.scales[axis] = crate::scalar_math::sqrt(variance);
            self.permutation[axis] = axis;
        }
        for row in 0..dimension {
            for column in 0..dimension {
                let value = 0.5 * (entry(row, column) + entry(column, row));
                if !value.is_finite() {
                    return Err(SmoothingError::InvalidCovariance);
                }
                let scale = self.scales[row] * self.scales[column];
                self.work[(row, column)] = if scale > 0.0 {
                    value / scale
                } else if value == 0.0 {
                    0.0
                } else {
                    return Err(SmoothingError::InvalidCovariance);
                };
            }
        }
        for pivot in 0..dimension {
            let mut largest = pivot;
            for candidate in pivot..dimension {
                if self.work[(candidate, candidate)] < -NEGATIVE_TOLERANCE {
                    return Err(SmoothingError::InvalidCovariance);
                }
                if self.work[(candidate, candidate)] > self.work[(largest, largest)] {
                    largest = candidate;
                }
            }
            if self.work[(largest, largest)] <= rank_tolerance {
                for row in pivot..dimension {
                    for column in pivot..dimension {
                        if self.work[(row, column)].abs() > NEGATIVE_TOLERANCE {
                            return Err(SmoothingError::InvalidCovariance);
                        }
                    }
                }
                break;
            }
            if largest != pivot {
                self.work.swap_rows(pivot, largest);
                self.work.swap_columns(pivot, largest);
                self.lower.swap_rows(pivot, largest);
                self.permutation.swap(pivot, largest);
            }
            let diagonal = crate::scalar_math::sqrt(self.work[(pivot, pivot)]);
            self.lower[(pivot, pivot)] = diagonal;
            for row in pivot + 1..dimension {
                self.lower[(row, pivot)] = self.work[(row, pivot)] / diagonal;
            }
            for row in pivot + 1..dimension {
                for column in pivot + 1..=row {
                    let value = self.work[(row, column)]
                        - self.lower[(row, pivot)] * self.lower[(column, pivot)];
                    self.work[(row, column)] = value;
                    self.work[(column, row)] = value;
                }
            }
            self.rank += 1;
        }
        Ok(())
    }

    fn solve(&mut self, rhs: &mut AugmentedNavCross, columns: usize) -> Result<(), SmoothingError> {
        self.solve_with_roundoff(rhs, columns, |_| 0.0)
    }

    fn solve_with_roundoff(
        &mut self,
        rhs: &mut AugmentedNavCross,
        columns: usize,
        roundoff: impl Fn(usize) -> f32,
    ) -> Result<(), SmoothingError> {
        for column in 0..columns {
            self.intermediate.fill(0.0);
            self.solution.fill(0.0);
            let mut rhs_scale: f32 = 0.0;
            for row in 0..self.dimension {
                let original = self.permutation[row];
                let scale = self.scales[original];
                let value = if scale > 0.0 {
                    rhs[(original, column)] / scale
                } else if rhs[(original, column)] == 0.0 {
                    0.0
                } else {
                    return Err(SmoothingError::NumericalFailure);
                };
                rhs_scale = rhs_scale.max(value.abs());
                self.intermediate[row] = value;
            }
            for row in 0..self.dimension {
                let mut value = self.intermediate[row];
                for source in 0..row.min(self.rank) {
                    value -= self.lower[(row, source)] * self.intermediate[source];
                }
                if row < self.rank {
                    self.intermediate[row] = value / self.lower[(row, row)];
                } else if value.abs() > (0.002 * rhs_scale).max(roundoff(column)) {
                    return Err(SmoothingError::NumericalFailure);
                }
            }
            for row in (0..self.rank).rev() {
                let mut value = self.intermediate[row];
                for source in row + 1..self.rank {
                    value -= self.lower[(source, row)] * self.solution[source];
                }
                self.solution[row] = value / self.lower[(row, row)];
            }
            for row in 0..self.dimension {
                let original = self.permutation[row];
                rhs[(original, column)] = if self.scales[original] > 0.0 {
                    self.solution[row] / self.scales[original]
                } else {
                    0.0
                };
                if !rhs[(original, column)].is_finite() {
                    return Err(SmoothingError::NumericalFailure);
                }
            }
        }
        Ok(())
    }
}
