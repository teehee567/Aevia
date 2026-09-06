//! Forward-filter propagation and storage of connected integration steps.

use crate::{
    error::ProcessError,
    observation::InputDisposition,
    offline::store::{
        StateStore, StoredCovariance, StoredDynamics, StoredImuSample, StoredIntegrationImu,
        StoredNominal, StoredStep,
    },
    time::SessionTime,
};

use nalgebra::{DMatrix, DVector};

use super::{
    estimation::{injection_reset, left_solve, repair_covariance, right_solve},
    filter::{ActiveImuSample, HeldImu, OfflineFilter},
    inertial::{propagation_model, refresh_inertial_kinematics},
    math::{copy_block, symmetric},
    smoothing::{boxminus, boxplus_with_reset},
};

impl<'a> OfflineFilter<'a> {
    pub(super) fn propagate_to(
        &mut self,
        target: SessionTime,
        guide: Option<&StoredNominal>,
    ) -> Result<(), ProcessError> {
        let Some(state) = self.nominal.as_ref() else {
            return Ok(());
        };
        if target < state.time {
            return Err(ProcessError::InvalidEvidence);
        }
        if target == state.time {
            return Ok(());
        }
        let held = self
            .held_imu
            .clone()
            .ok_or(ProcessError::IncompleteEvidence)?;
        if state.time < held.start || target > held.time {
            // The qualified interval average is a piecewise-constant input
            // only over its declared support. Never extrapolate it across an
            // unobserved interval.
            return Err(ProcessError::IncompleteEvidence);
        }
        self.propagate_with_imu(target, &held, guide)
    }

    pub(super) fn flush_held_interval(
        &mut self,
        store: &mut dyn StateStore,
        guide: Option<&StoredNominal>,
    ) -> Result<bool, ProcessError> {
        let Some(held) = self.held_imu.clone() else {
            return Ok(false);
        };
        let Some(state) = self.nominal.as_ref() else {
            return Ok(false);
        };
        if state.time < held.start || state.time > held.time {
            return Err(ProcessError::InvalidEvidence);
        }
        if state.time == held.time {
            return Ok(false);
        }
        self.propagate_with_imu(held.time, &held, guide)?;
        self.store_current(None, None, store)?;
        Ok(true)
    }

    pub(super) fn propagate_with_imu(
        &mut self,
        target: SessionTime,
        imu: &HeldImu,
        guide: Option<&StoredNominal>,
    ) -> Result<(), ProcessError> {
        let current_nominal = self.nominal.clone().ok_or(ProcessError::InvalidEvidence)?;
        let mut covariance = self
            .covariance
            .clone()
            .ok_or(ProcessError::InvalidEvidence)?;
        let duration = target
            .checked_duration_since(current_nominal.time)
            .ok_or(ProcessError::InvalidEvidence)?;
        let dt = duration.as_seconds_f64();
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ProcessError::InvalidEvidence);
        }
        let active_sample = self
            .active_imu_sample
            .clone()
            .ok_or(ProcessError::IncompleteEvidence)?;
        if active_sample.start != imu.start
            || active_sample.end != imu.time
            || active_sample.state_cross.shape() != (self.state_dimension, 6)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        let (next_nominal, transition, consider_transition, process_covariance, sample_influence) =
            if self.relinearized {
                let guide_current = self
                    .guide_nominal
                    .clone()
                    .ok_or(ProcessError::NumericalNonConvergence)?;
                if guide_current.time != current_nominal.time {
                    return Err(ProcessError::NumericalNonConvergence);
                }
                let dynamics = propagation_model(
                    self.config,
                    self.catalog,
                    &guide_current,
                    imu,
                    dt,
                    self.state_dimension,
                    self.colored_error,
                )?;
                self.retain_dynamics(&guide_current, &dynamics, target);
                let mut extrapolated_guide = dynamics.nominal.clone();
                extrapolated_guide.time = target;
                let guide_next = guide.unwrap_or(&extrapolated_guide);
                if guide_current.time != current_nominal.time || guide_next.time != target {
                    return Err(ProcessError::NumericalNonConvergence);
                }
                let affine = affine_propagation(
                    &current_nominal,
                    &guide_current,
                    guide_next,
                    dynamics.nominal,
                    &dynamics.transition,
                    &dynamics.consider_transition,
                    &dynamics.process_covariance,
                    &dynamics.sample_influence,
                )?;
                self.guide_nominal = Some(guide_next.clone());
                affine
            } else {
                let dynamics = propagation_model(
                    self.config,
                    self.catalog,
                    &current_nominal,
                    imu,
                    dt,
                    self.state_dimension,
                    self.colored_error,
                )?;
                self.retain_dynamics(&current_nominal, &dynamics, target);
                (
                    dynamics.nominal,
                    dynamics.transition,
                    dynamics.consider_transition,
                    dynamics.process_covariance,
                    dynamics.sample_influence,
                )
            };
        let mut next_nominal = next_nominal;
        next_nominal.time = target;
        next_nominal.imu_sample_error_body = current_nominal.imu_sample_error_body;
        refresh_inertial_kinematics(&mut next_nominal, imu)?;

        let propagated_sample_cross = &transition * &active_sample.state_cross
            + &consider_transition * active_sample.consider_cross.transpose();
        let p_xx = &transition * &covariance.state * transition.transpose()
            + &transition * &covariance.state_consider * consider_transition.transpose()
            + &consider_transition * covariance.state_consider.transpose() * transition.transpose()
            + &consider_transition * &self.catalog.covariance * consider_transition.transpose()
            + &process_covariance
            + &propagated_sample_cross * sample_influence.transpose()
            + &sample_influence * propagated_sample_cross.transpose()
            + &sample_influence * &active_sample.covariance_body * sample_influence.transpose();
        let propagated_state_consider = &transition * &covariance.state_consider
            + &consider_transition * &self.catalog.covariance
            + &sample_influence * &active_sample.consider_cross;
        covariance.state = symmetric(p_xx);
        covariance.state_consider = propagated_state_consider;
        repair_covariance(
            &mut covariance.state,
            self.config
                .navigation_profile
                .covariance_repair
                .maximum_attempts,
            self.config
                .navigation_profile
                .covariance_repair
                .maximum_total_regularization
                .get(),
            &mut self.diagnostics.covariance_repairs,
        )?;

        self.process_accumulator =
            &transition * &self.process_accumulator * transition.transpose() + &process_covariance;
        self.consider_transition_accumulator =
            &transition * &self.consider_transition_accumulator + &consider_transition;
        self.sample_influence_accumulator =
            &transition * &self.sample_influence_accumulator + &sample_influence;
        self.transition_accumulator = &transition * &self.transition_accumulator;
        let next_sample_cross =
            propagated_sample_cross + &sample_influence * &active_sample.covariance_body;
        self.nominal = Some(next_nominal);
        self.covariance = Some(covariance);
        self.active_imu_sample = Some(ActiveImuSample {
            state_cross: next_sample_cross,
            ..active_sample
        });
        self.integration_imu = Some(StoredIntegrationImu {
            start: current_nominal.time,
            end: target,
            angular_rate_body: imu.angular_rate_body,
            specific_force_body: imu.specific_force_body,
        });
        Ok(())
    }

    fn retain_dynamics(
        &mut self,
        reference: &StoredNominal,
        model: &super::inertial::PropagationModel,
        target: SessionTime,
    ) {
        let mut end = model.nominal.clone();
        end.time = target;
        self.dynamics = Some(StoredDynamics {
            reference_start: reference.clone(),
            reference_end: end,
            continuous: model.continuous.clone(),
            noise_density: model.noise_density.clone(),
            consider_rate_mapping: model.consider_rate_mapping.clone(),
            sample_rate_mapping: model.sample_rate_mapping.clone(),
        });
    }

    /// Replace the completed support's latent by the next independent sample
    /// at the same epoch without creating a zero-duration smoothing edge.
    pub(super) fn replace_stored_sample(
        &mut self,
        store: &mut dyn StateStore,
    ) -> Result<(), ProcessError> {
        let nominal = self.nominal.as_ref().ok_or(ProcessError::InvalidEvidence)?;
        if self.last_stored_time != Some(nominal.time) || store.len() == 0 {
            return Ok(());
        }
        let sample = self
            .active_imu_sample
            .as_ref()
            .ok_or(ProcessError::IncompleteEvidence)?
            .stored();
        let index = store.len() - 1;
        let mut step = store.get(index).map_err(ProcessError::from)?;
        step.predicted.imu_sample_error_body = [0.0; 6];
        step.filtered = nominal.clone();
        step.predicted_sample = sample.clone();
        step.filtered_sample = sample.clone();
        step.adjacent_sample_cross
            .columns_mut(self.state_dimension, 6)
            .fill(0.0);
        store.set(index, &step).map_err(ProcessError::from)?;
        self.last_stored_sample = Some(sample);
        Ok(())
    }

    pub(super) fn store_current(
        &mut self,
        predicted_override: Option<(StoredNominal, StoredCovariance, StoredImuSample)>,
        measurement: Option<(InputDisposition, f64, DMatrix<f64>)>,
        store: &mut dyn StateStore,
    ) -> Result<(), ProcessError> {
        let filtered = self.nominal.clone().ok_or(ProcessError::InvalidEvidence)?;
        let filtered_sample = self
            .active_imu_sample
            .as_ref()
            .ok_or(ProcessError::IncompleteEvidence)?
            .stored();
        let filtered_covariance = self
            .covariance
            .clone()
            .ok_or(ProcessError::InvalidEvidence)?;
        if self.last_stored_time == Some(filtered.time) && store.len() > 0 {
            let index = store.len() - 1;
            let mut existing = store.get(index).map_err(ProcessError::from)?;
            if let Some((disposition, contribution, reset)) = &measurement {
                existing.reset_basis = reset * &existing.reset_basis;
                existing.objective_contribution += *contribution;
                existing.disposition = Some(*disposition);
            }
            existing.filtered = filtered.clone();
            existing.filtered_covariance = filtered_covariance.clone();
            existing.filtered_sample = filtered_sample.clone();
            if self.dynamics.is_some() {
                existing.dynamics = self.dynamics.take();
            }
            existing.gnss_state = self.gnss_state;
            existing.timing_quality = self.timing_quality;
            existing.degraded_input |= self.held_imu.as_ref().is_some_and(|imu| imu.degraded_input);
            if let Some(integration_imu) = self.integration_imu.take() {
                if existing
                    .integration_imu
                    .as_ref()
                    .is_some_and(|present| present != &integration_imu)
                {
                    return Err(ProcessError::StorageCorrupt);
                }
                existing.integration_imu = Some(integration_imu);
            }
            store.set(index, &existing).map_err(ProcessError::from)?;
            self.last_stored_covariance = Some(filtered_covariance);
            self.last_stored_sample = Some(filtered_sample);
            self.transition_accumulator.fill(0.0);
            self.transition_accumulator.fill_diagonal(1.0);
            self.consider_transition_accumulator.fill(0.0);
            self.process_accumulator.fill(0.0);
            self.sample_influence_accumulator.fill(0.0);
            self.last_stored_sample_cross = self.active_imu_sample.as_ref().map_or_else(
                || DMatrix::zeros(self.state_dimension, 6),
                |sample| sample.state_cross.clone(),
            );
            return Ok(());
        }

        let (predicted, predicted_covariance, predicted_sample) = predicted_override
            .unwrap_or_else(|| {
                (
                    filtered.clone(),
                    filtered_covariance.clone(),
                    filtered_sample.clone(),
                )
            });
        let consider_dimension = self.catalog.covariance.nrows();
        let sample_covariance = self.last_stored_sample.as_ref().map_or_else(
            || DMatrix::zeros(6, 6),
            |sample| sample.covariance_body.clone(),
        );
        let propagated_stored_sample_cross = &self.transition_accumulator
            * &self.last_stored_sample_cross
            + self.last_stored_sample.as_ref().map_or_else(
                || DMatrix::zeros(self.state_dimension, 6),
                |sample| &self.consider_transition_accumulator * sample.consider_cross.transpose(),
            );
        let sample_process = &propagated_stored_sample_cross
            * self.sample_influence_accumulator.transpose()
            + &self.sample_influence_accumulator * propagated_stored_sample_cross.transpose()
            + &self.sample_influence_accumulator
                * &sample_covariance
                * self.sample_influence_accumulator.transpose();
        let adjacent_cross_covariance = self.last_stored_covariance.as_ref().map_or_else(
            || DMatrix::zeros(self.state_dimension, self.state_dimension),
            |previous| {
                &previous.state * self.transition_accumulator.transpose()
                    + &previous.state_consider * self.consider_transition_accumulator.transpose()
                    + &self.last_stored_sample_cross * self.sample_influence_accumulator.transpose()
            },
        );
        let mut adjacent_sample_cross =
            DMatrix::zeros(self.state_dimension + 6, self.state_dimension + 6);
        copy_block(&adjacent_cross_covariance, &mut adjacent_sample_cross, 0, 0);
        if let Some(previous) = &self.last_stored_sample {
            copy_block(
                &previous.state_cross,
                &mut adjacent_sample_cross,
                0,
                self.state_dimension,
            );
            let sample_to_next = previous.state_cross.transpose()
                * self.transition_accumulator.transpose()
                + &previous.consider_cross * self.consider_transition_accumulator.transpose()
                + &previous.covariance_body * self.sample_influence_accumulator.transpose();
            copy_block(
                &sample_to_next,
                &mut adjacent_sample_cross,
                self.state_dimension,
                0,
            );
            copy_block(
                &previous.covariance_body,
                &mut adjacent_sample_cross,
                self.state_dimension,
                self.state_dimension,
            );
        }
        let reset_basis = measurement.as_ref().map_or_else(
            || DMatrix::identity(self.state_dimension, self.state_dimension),
            |value| value.2.clone(),
        );
        let step = StoredStep {
            connected_from_previous: self.connected && self.last_stored_time.is_some(),
            predicted,
            filtered: filtered.clone(),
            smoothed: None,
            predicted_covariance,
            filtered_covariance: filtered_covariance.clone(),
            smoothed_covariance: None,
            predicted_sample,
            filtered_sample: filtered_sample.clone(),
            smoothed_sample: None,
            adjacent_sample_cross,
            transition: self.transition_accumulator.clone(),
            consider_transition: self.consider_transition_accumulator.clone(),
            process_covariance: symmetric(&self.process_accumulator + sample_process),
            integration_imu: self.integration_imu.take(),
            dynamics: self.dynamics.take(),
            reset_basis,
            smoothed_backward_gain: None,
            adjacent_cross_covariance,
            disposition: measurement.as_ref().map(|value| value.0),
            gnss_state: self.gnss_state,
            timing_quality: self.timing_quality,
            degraded_input: self.held_imu.as_ref().is_some_and(|imu| imu.degraded_input),
            objective_contribution: measurement.as_ref().map_or(0.0, |value| value.1),
        };
        if store.dimensions() != (self.state_dimension, consider_dimension)
            || store.len() >= store.maximum_records()
        {
            return Err(ProcessError::StorageExhausted);
        }
        store.push(&step).map_err(ProcessError::from)?;
        self.last_stored_time = Some(filtered.time);
        self.last_stored_covariance = Some(filtered_covariance);
        self.last_stored_sample = Some(filtered_sample);
        self.transition_accumulator.fill(0.0);
        self.transition_accumulator.fill_diagonal(1.0);
        self.consider_transition_accumulator.fill(0.0);
        self.process_accumulator.fill(0.0);
        self.sample_influence_accumulator.fill(0.0);
        self.last_stored_sample_cross = self.active_imu_sample.as_ref().map_or_else(
            || DMatrix::zeros(self.state_dimension, 6),
            |sample| sample.state_cross.clone(),
        );
        Ok(())
    }
}

/// Forms the affine IEKS dynamics step about two consecutive points on the
/// preceding smoothed trajectory.  For
///
/// `x_next = f(reference_current) + F (x_current - reference_current)`,
///
/// the nonlinear defect `f(reference_current) ⊟ reference_next` is retained;
/// simply propagating the current estimate with a Jacobian evaluated at the
/// guide would not be a relinearized Gauss–Newton step.
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub(super) fn affine_propagation(
    current: &StoredNominal,
    reference_current: &StoredNominal,
    reference_next: &StoredNominal,
    mut nonlinear_reference_prediction: StoredNominal,
    reference_transition: &DMatrix<f64>,
    reference_consider_transition: &DMatrix<f64>,
    reference_process_covariance: &DMatrix<f64>,
    reference_sample_influence: &DMatrix<f64>,
) -> Result<
    (
        StoredNominal,
        DMatrix<f64>,
        DMatrix<f64>,
        DMatrix<f64>,
        DMatrix<f64>,
    ),
    ProcessError,
> {
    let state_dimension = reference_transition.nrows();
    if current.time != reference_current.time
        || reference_next.time <= reference_current.time
        || reference_transition.ncols() != state_dimension
        || reference_consider_transition.nrows() != state_dimension
        || reference_process_covariance.shape() != (state_dimension, state_dimension)
        || reference_sample_influence.shape() != (state_dimension, 6)
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    nonlinear_reference_prediction.time = reference_next.time;
    let current_delta = boxminus(current, reference_current, state_dimension)?;
    let dynamics_defect = boxminus(
        &nonlinear_reference_prediction,
        reference_next,
        state_dimension,
    )?;
    // `reference_transition` ends in the tangent basis of
    // `f(reference_current)`.  Move it through the nonlinear defect into the
    // `reference_next` basis before adding affine coordinates.
    let defect_reset = injection_reset(&dynamics_defect)?;
    let transition_at_next_reference = left_solve(&defect_reset, reference_transition)?;
    let consider_at_next_reference = left_solve(&defect_reset, reference_consider_transition)?;
    let process_left = left_solve(&defect_reset, reference_process_covariance)?;
    let process_at_next_reference = right_solve(&process_left, &defect_reset.transpose())?;
    let sample_at_next_reference = left_solve(&defect_reset, reference_sample_influence)?;
    let sample_delta = DVector::from_iterator(
        6,
        (0..6)
            .map(|i| current.imu_sample_error_body[i] - reference_current.imu_sample_error_body[i]),
    );
    let next_delta = dynamics_defect
        + &transition_at_next_reference * &current_delta
        + &sample_at_next_reference * sample_delta;
    let (next, next_reset) = boxplus_with_reset(reference_next, &next_delta)?;
    let current_reset = injection_reset(&current_delta)?;
    let transition_in_reference = right_solve(&transition_at_next_reference, &current_reset)?;
    let transition = &next_reset * transition_in_reference;
    let consider_transition = &next_reset * consider_at_next_reference;
    let process_covariance =
        symmetric(&next_reset * process_at_next_reference * next_reset.transpose());
    let sample_influence = &next_reset * sample_at_next_reference;
    if !transition.iter().all(|value| value.is_finite())
        || !consider_transition.iter().all(|value| value.is_finite())
        || !process_covariance.iter().all(|value| value.is_finite())
        || !sample_influence.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::NumericalNonConvergence);
    }
    Ok((
        next,
        transition,
        consider_transition,
        process_covariance,
        sample_influence,
    ))
}
