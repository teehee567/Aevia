//! Affine clock-consider covariance transitions.

use super::{
    ConsiderCovariance, Eskf, EskfError, MAX_CONSIDER, covariance::active_principal_block_is_psd,
};
use crate::live::state::NAV_DIM;

impl Eskf {
    /// Atomically replaces the first two shared coordinates with the next
    /// clock segment under a centered affine Gaussian bridge. Navigation
    /// covariance is a marginal and therefore remains unchanged; `Pxc` and
    /// `Pcc` are transformed exactly in the live scalar type.
    #[cfg(test)]
    pub(crate) fn transition_clock_consider(
        &mut self,
        active_consider: usize,
        next_clock_from_previous: &[[f32; MAX_CONSIDER]; 2],
        innovation_covariance_upper: [f32; 3],
    ) -> Result<(), EskfError> {
        let mut candidate = *self;
        self.transition_clock_consider_into(
            &mut candidate,
            active_consider,
            next_clock_from_previous,
            innovation_covariance_upper,
        )?;
        *self = candidate;
        Ok(())
    }

    /// Computes a clock transition into caller-owned scratch, leaving the
    /// live source untouched on every error path.
    #[inline(never)]
    pub(in crate::live) fn transition_clock_consider_into(
        &self,
        candidate: &mut Self,
        active_consider: usize,
        next_clock_from_previous: &[[f32; MAX_CONSIDER]; 2],
        innovation_covariance_upper: [f32; 3],
    ) -> Result<(), EskfError> {
        if active_consider != self.active_consider || candidate.active_consider != active_consider {
            return Err(EskfError::InvalidConsiderBlock);
        }
        transition_consider_covariance_into(
            &self.consider_covariance,
            active_consider,
            next_clock_from_previous,
            innovation_covariance_upper,
            &mut candidate.consider_covariance,
        )?;
        candidate.nav_consider_covariance = self.nav_consider_covariance;
        for nav in 0..NAV_DIM {
            for next_clock in 0..2 {
                let mut covariance = 0.0;
                for previous in 0..active_consider {
                    covariance += self.nav_consider_covariance[(nav, previous)]
                        * next_clock_from_previous[next_clock][previous];
                }
                candidate.nav_consider_covariance[(nav, next_clock)] = covariance;
            }
        }
        if !candidate
            .nav_consider_covariance
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(EskfError::InvalidCovariance);
        }
        Ok(())
    }
}

/// Computes an affine clock-prior transform into caller-owned transaction
/// scratch. The source is never mutated; callers must not commit `candidate`
/// unless this function succeeds.
#[inline(never)]
pub(crate) fn transition_consider_covariance_into(
    previous: &ConsiderCovariance,
    active_consider: usize,
    next_clock_from_previous: &[[f32; MAX_CONSIDER]; 2],
    innovation_covariance_upper: [f32; 3],
    candidate: &mut ConsiderCovariance,
) -> Result<(), EskfError> {
    if !(2..=MAX_CONSIDER).contains(&active_consider)
        || !previous.iter().all(|value| value.is_finite())
        || !next_clock_from_previous
            .iter()
            .flatten()
            .all(|value| value.is_finite())
        || next_clock_from_previous
            .iter()
            .any(|row| row[active_consider..].iter().any(|value| *value != 0.0))
        || !clock_innovation_is_psd(innovation_covariance_upper)
    {
        return Err(EskfError::InvalidConsiderBlock);
    }

    candidate.copy_from(previous);
    for next_clock in 0..2 {
        for retained in 2..active_consider {
            let mut covariance = 0.0;
            for previous_coordinate in 0..active_consider {
                covariance += next_clock_from_previous[next_clock][previous_coordinate]
                    * previous[(previous_coordinate, retained)];
            }
            candidate[(next_clock, retained)] = covariance;
            candidate[(retained, next_clock)] = covariance;
        }
    }
    for left in 0..2 {
        for right in left..2 {
            let mut covariance = match (left, right) {
                (0, 0) => innovation_covariance_upper[0],
                (0, 1) => innovation_covariance_upper[1],
                (1, 1) => innovation_covariance_upper[2],
                _ => 0.0,
            };
            for previous_left in 0..active_consider {
                for previous_right in 0..active_consider {
                    covariance += next_clock_from_previous[left][previous_left]
                        * previous[(previous_left, previous_right)]
                        * next_clock_from_previous[right][previous_right];
                }
            }
            candidate[(left, right)] = covariance;
            candidate[(right, left)] = covariance;
        }
    }
    if !candidate.iter().all(|value| value.is_finite())
        || !active_principal_block_is_psd(&candidate, active_consider)
    {
        return Err(EskfError::InvalidCovariance);
    }
    Ok(())
}

/// Computes an independent clock-prior replacement into caller-owned
/// transaction scratch. The source is never mutated; callers must not commit
/// `candidate` unless this function succeeds.
#[inline(never)]
pub(crate) fn independent_clock_consider_covariance_into(
    previous: &ConsiderCovariance,
    active_consider: usize,
    covariance_upper: [f32; 3],
    candidate: &mut ConsiderCovariance,
) -> Result<(), EskfError> {
    if !(2..=MAX_CONSIDER).contains(&active_consider)
        || !previous.iter().all(|value| value.is_finite())
        || !clock_innovation_is_psd(covariance_upper)
    {
        return Err(EskfError::InvalidConsiderBlock);
    }
    candidate.copy_from(previous);
    for coordinate in 2..active_consider {
        candidate[(0, coordinate)] = 0.0;
        candidate[(coordinate, 0)] = 0.0;
        candidate[(1, coordinate)] = 0.0;
        candidate[(coordinate, 1)] = 0.0;
    }
    candidate[(0, 0)] = covariance_upper[0];
    candidate[(0, 1)] = covariance_upper[1];
    candidate[(1, 0)] = covariance_upper[1];
    candidate[(1, 1)] = covariance_upper[2];
    if !candidate.iter().all(|value| value.is_finite())
        || !active_principal_block_is_psd(candidate, active_consider)
    {
        return Err(EskfError::InvalidCovariance);
    }
    Ok(())
}

pub(super) fn clock_innovation_is_psd(covariance_upper: [f32; 3]) -> bool {
    if !covariance_upper.iter().all(|value| value.is_finite()) {
        return false;
    }
    let [offset, cross, drift] = covariance_upper;
    if offset < 0.0 || drift < 0.0 {
        return false;
    }
    if cross == 0.0 {
        return true;
    }
    // A PSD matrix with a zero diagonal must have a zero corresponding row.
    // Compare correlation magnitudes using square roots and division instead
    // of forming `offset * drift` or `cross * cross`, which can overflow or
    // underflow even when every supplied f32 is finite.
    if offset == 0.0 || drift == 0.0 {
        return false;
    }
    let larger_variance = offset.max(drift);
    let smaller_variance = offset.min(drift);
    let normalized_cross = cross.abs() / crate::scalar_math::sqrt(larger_variance);
    let correlation_bound =
        crate::scalar_math::sqrt(smaller_variance) * (1.0 + 256.0 * f32::EPSILON);
    normalized_cross.is_finite() && normalized_cross <= correlation_bound
}
