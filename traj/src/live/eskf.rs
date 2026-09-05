//! Fifteen-state right-multiplicative ESKF used by the embedded live profile.

mod clock;
mod covariance;
mod discretization;
mod gnss;
mod matrix;
mod propagation;
mod update;

#[cfg(test)]
mod tests;

use crate::{
    live::{
        preintegration::{
            BIAS_DIM, GapDerivativeCovariance, PREINT_DIM, PreintegrationError,
            covariance_density_is_valid,
        },
        state::{NAV_DIM, NavMatrix, NavState, StateError},
    },
    time::SessionTime,
    uncertainty::MAX_SHARED_PARAMETER_DIMENSION,
};
pub(crate) use clock::{
    independent_clock_consider_covariance_into, transition_consider_covariance_into,
};
pub(crate) use covariance::CovariancePolicy;
use covariance::active_principal_block_is_psd;
pub(crate) use gnss::{
    GnssObservation, GnssUpdateOutcome, NisGate, SharedMeasurementJacobians, UpdateDecision,
};
use nalgebra::{ArrayStorage, Matrix3, SMatrix, SVector};
pub(crate) use propagation::propagate_nominal;

pub(crate) const MAX_CONSIDER: usize = MAX_SHARED_PARAMETER_DIMENSION;

const MAX_MEASUREMENT_DIM: usize = 6;

pub(crate) type NavConsiderCovariance = SMatrix<f32, NAV_DIM, MAX_CONSIDER>;

pub(crate) type ConsiderCovariance = SMatrix<f32, MAX_CONSIDER, MAX_CONSIDER>;

pub(crate) type ConsiderJacobian3 = SMatrix<f32, 3, MAX_CONSIDER>;

type MeasurementVector = SVector<f32, MAX_MEASUREMENT_DIM>;

type MeasurementMatrix = SMatrix<f32, MAX_MEASUREMENT_DIM, MAX_MEASUREMENT_DIM>;

type MeasurementNavJacobian = SMatrix<f32, MAX_MEASUREMENT_DIM, NAV_DIM>;

type MeasurementConsiderJacobian = SMatrix<f32, MAX_MEASUREMENT_DIM, MAX_CONSIDER>;

type MeasurementSampleJacobian = SMatrix<f32, MAX_MEASUREMENT_DIM, BIAS_DIM>;

type NavMeasurementCross = SMatrix<f32, NAV_DIM, MAX_MEASUREMENT_DIM>;

type ConsiderMeasurementCross = SMatrix<f32, MAX_CONSIDER, MAX_MEASUREMENT_DIM>;

type SampleMeasurementCross = SMatrix<f32, BIAS_DIM, MAX_MEASUREMENT_DIM>;

pub(crate) type GapNavCrossCovariance = SMatrix<f32, NAV_DIM, BIAS_DIM>;

type PreintegrationNavJacobian = SMatrix<f32, NAV_DIM, PREINT_DIM>;

/// Caller-owned propagation workspace. The live engine places this cold
/// scratch in PSRAM so fixed-size matrix products never consume the S31 task
/// stack. No value here survives one propagation transaction.
#[derive(Debug, PartialEq)]
pub(crate) struct EskfPropagationScratch {
    continuous: NavMatrix,
    transition: NavMatrix,
    nav_a: NavMatrix,
    nav_b: NavMatrix,
    nav_c: NavMatrix,
    mapping: PreintegrationNavJacobian,
    gamma: NavConsiderCovariance,
    cross_a: NavConsiderCovariance,
    gap_a: GapNavCrossCovariance,
    gap_b: GapNavCrossCovariance,
}

impl EskfPropagationScratch {
    pub(crate) const fn new() -> Self {
        Self {
            continuous: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            transition: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_a: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_b: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_c: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            mapping: PreintegrationNavJacobian::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; PREINT_DIM],
            )),
            gamma: NavConsiderCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; MAX_CONSIDER],
            )),
            cross_a: NavConsiderCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; MAX_CONSIDER],
            )),
            gap_a: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; BIAS_DIM],
            )),
            gap_b: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; BIAS_DIM],
            )),
        }
    }

    pub(crate) fn stage_sample_cross(&mut self, source: &GapNavCrossCovariance) {
        self.gap_a.copy_from(source);
    }

    pub(crate) fn sample_candidate_mut(&mut self) -> &mut GapNavCrossCovariance {
        &mut self.gap_a
    }

    pub(crate) fn commit_sample_candidate_into(&self, target: &mut GapNavCrossCovariance) {
        target.copy_from(&self.gap_a);
    }
}

pub(crate) const fn zero_consider_covariance() -> ConsiderCovariance {
    ConsiderCovariance::from_array_storage(nalgebra::ArrayStorage(
        [[0.0; MAX_CONSIDER]; MAX_CONSIDER],
    ))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ProcessNoise {
    pub(crate) accel_bias_random_walk_covariance_density: Matrix3<f32>,
    pub(crate) gyro_bias_random_walk_covariance_density: Matrix3<f32>,
}

impl ProcessNoise {
    pub(crate) fn is_valid(&self) -> bool {
        covariance_density_is_valid(&self.accel_bias_random_walk_covariance_density)
            && covariance_density_is_valid(&self.gyro_bias_random_walk_covariance_density)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Eskf {
    pub(crate) state: NavState,
    pub(crate) covariance: NavMatrix,
    /// Navigation/shared-parameter cross covariance.
    pub(crate) nav_consider_covariance: NavConsiderCovariance,
    /// Fixed-mean shared parameter covariance. It is never estimated here.
    pub(crate) consider_covariance: ConsiderCovariance,
    pub(crate) active_consider: usize,
    /// Cross covariance with the one six-coordinate constant-derivative gap
    /// latent that may span scheduler cuts. `None` means it has already been
    /// marginalized into the navigation covariance.
    pub(crate) gap_origin: Option<SessionTime>,
    pub(crate) gap_derivative_covariance: GapDerivativeCovariance,
    pub(crate) gap_nav_cross_covariance: GapNavCrossCovariance,
    process_noise: ProcessNoise,
    covariance_policy: CovariancePolicy,
    pub(crate) covariance_repairs: u32,
    pub(crate) total_normalized_repair: f32,
}

impl Eskf {
    /// Valid inactive representation for statically placed live storage.
    pub(crate) const fn placeholder() -> Self {
        Self {
            state: NavState::placeholder(),
            covariance: NavMatrix::from_array_storage(ArrayStorage([[0.0; NAV_DIM]; NAV_DIM])),
            nav_consider_covariance: NavConsiderCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; MAX_CONSIDER],
            )),
            consider_covariance: zero_consider_covariance(),
            active_consider: 0,
            gap_origin: None,
            gap_derivative_covariance: GapDerivativeCovariance::from_array_storage(ArrayStorage(
                [[0.0; BIAS_DIM]; BIAS_DIM],
            )),
            gap_nav_cross_covariance: GapNavCrossCovariance::from_array_storage(ArrayStorage(
                [[0.0; NAV_DIM]; BIAS_DIM],
            )),
            process_noise: ProcessNoise {
                accel_bias_random_walk_covariance_density: Matrix3::from_array_storage(
                    ArrayStorage([[0.0; 3]; 3]),
                ),
                gyro_bias_random_walk_covariance_density: Matrix3::from_array_storage(
                    ArrayStorage([[0.0; 3]; 3]),
                ),
            },
            covariance_policy: CovariancePolicy {
                state_scales: [0.0; NAV_DIM],
                minimum_variance: [0.0; NAV_DIM],
                repair_initial: 0.0,
                repair_growth: 0.0,
                maximum_total_repair: 0.0,
                maximum_repair_attempts: 0,
            },
            covariance_repairs: 0,
            total_normalized_repair: 0.0,
        }
    }

    pub(crate) fn reset(&mut self) {
        self.state = NavState::placeholder();
        self.covariance.fill(0.0);
        self.nav_consider_covariance.fill(0.0);
        self.consider_covariance.fill(0.0);
        self.active_consider = 0;
        self.gap_origin = None;
        self.gap_derivative_covariance.fill(0.0);
        self.gap_nav_cross_covariance.fill(0.0);
        self.process_noise
            .accel_bias_random_walk_covariance_density
            .fill(0.0);
        self.process_noise
            .gyro_bias_random_walk_covariance_density
            .fill(0.0);
        self.covariance_policy.state_scales.fill(0.0);
        self.covariance_policy.minimum_variance.fill(0.0);
        self.covariance_policy.repair_initial = 0.0;
        self.covariance_policy.repair_growth = 0.0;
        self.covariance_policy.maximum_total_repair = 0.0;
        self.covariance_policy.maximum_repair_attempts = 0;
        self.covariance_repairs = 0;
        self.total_normalized_repair = 0.0;
    }

    /// Initializes the existing covariance storage in place. Any validation
    /// or conditioning failure restores the inactive placeholder.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn initialize(
        &mut self,
        state: NavState,
        covariance: &NavMatrix,
        nav_consider_covariance: &NavConsiderCovariance,
        consider_covariance: &ConsiderCovariance,
        active_consider: usize,
        process_noise: ProcessNoise,
        covariance_policy: CovariancePolicy,
    ) -> Result<(), EskfError> {
        self.reset();
        if !state.is_finite()
            || !process_noise.is_valid()
            || !covariance_policy.is_valid()
            || active_consider > MAX_CONSIDER
            || !nav_consider_covariance
                .iter()
                .all(|value| value.is_finite())
            || !consider_covariance.iter().all(|value| value.is_finite())
        {
            return Err(EskfError::InvalidConfiguration);
        }
        if !active_principal_block_is_psd(consider_covariance, active_consider) {
            return Err(EskfError::InvalidCovariance);
        }
        self.state = state;
        self.covariance.copy_from(covariance);
        self.nav_consider_covariance
            .copy_from(nav_consider_covariance);
        self.consider_covariance.copy_from(consider_covariance);
        self.active_consider = active_consider;
        self.process_noise = process_noise;
        self.covariance_policy = covariance_policy;
        if let Err(error) = self.condition_covariance() {
            self.reset();
            return Err(error);
        }
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn new(
        state: NavState,
        covariance: NavMatrix,
        nav_consider_covariance: NavConsiderCovariance,
        consider_covariance: ConsiderCovariance,
        active_consider: usize,
        process_noise: ProcessNoise,
        covariance_policy: CovariancePolicy,
    ) -> Result<Self, EskfError> {
        let mut result = Self::placeholder();
        result.initialize(
            state,
            &covariance,
            &nav_consider_covariance,
            &consider_covariance,
            active_consider,
            process_noise,
            covariance_policy,
        )?;
        Ok(result)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum EskfError {
    InvalidConfiguration,
    InvalidCovariance,
    CovarianceNotPositiveSemidefinite,
    InvalidMeasurement,
    InvalidMeasurementCovariance,
    InvalidConsiderBlock,
    InvalidInnovationCovariance,
    TimeMismatch,
    PropagationIntervalTooLong,
    NumericalFailure,
    DiscretizationBoundExceeded,
    GapLatentMismatch,
    ImuSampleLatentMismatch,
    MissingAngularAccelerationForTiming,
    Preintegration(PreintegrationError),
    State(StateError),
}
