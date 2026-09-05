//! GNSS observation validation, covariance preparation, and delayed measurement scheduling.

use super::conversion::{
    add_matrix, finite_f32, map_core_step_error, na_vector, rotate_covariance_to_n,
    rotate_cross_to_n, scale_covariance, scale_cross, vector_f32,
};
use super::{InitializationFixEvidence, LiveSession};
use crate::config::{GnssCorrelationPolicy, UncertaintyModelKind};
use crate::error::{StepError, ValidationError};
use crate::frame::ReferencePointKind;
use crate::ids::ObservationId;
use crate::live::{
    EcefAnchor, EnqueueDisposition, GnssObservation, LiveCore, OrderKey, Scheduled,
    SharedMeasurementJacobians,
};
use crate::observation::{
    GnssSolutionObservation, InputDisposition, ReceiverHealth, RtkState, SolutionClass,
};
use crate::quality::{GnssState, Integrity, TimingQuality};
use crate::time::{ObservationTime, SessionTime, TimingBasis};
use crate::uncertainty::{Covariance3, MeasurementUncertainty};
use nalgebra::{Matrix3, Vector3 as NaVector3};

const GNSS_VELOCITY_CLASS: u8 = 3;

const GNSS_POSITION_CLASS: u8 = 2;

impl LiveSession<'_, '_> {
    #[inline(never)]
    pub(super) fn ingest_gnss(
        &mut self,
        observation: GnssSolutionObservation,
    ) -> Result<InputDisposition, StepError> {
        self.validate_antenna(observation)?;
        let position = observation.position();
        let velocity = observation.velocity();
        if position.is_some_and(|value| value.frame != self.engine.processing_frame.id())
            || velocity.is_some_and(|value| value.frame != self.engine.processing_frame.id())
        {
            return Err(StepError::FrameMismatch);
        }
        let mut before_pending_boundary = false;
        let mut at_or_after_pending_boundary = false;
        for time in [
            position.map(|value| value.time),
            velocity.map(|value| value.time),
        ]
        .into_iter()
        .flatten()
        {
            let effective = time
                .effective_time()
                .map_err(StepError::InvalidObservation)?;
            self.validate_point_clock_model(time.clock_model, effective)?;
            if let Some(pending) = self.pending_clock_transition {
                if effective < pending.observation.at {
                    before_pending_boundary = true;
                } else {
                    at_or_after_pending_boundary = true;
                }
            }
        }
        if observation.position_velocity_cross_covariance().is_some()
            && position.zip(velocity).is_some_and(|(position, velocity)| {
                position.time.effective_time().ok() != velocity.time.effective_time().ok()
            })
        {
            // A cross-covariance between independently timed fields cannot be
            // split into two independent live updates. Retain the complete
            // observation for a backend with a two-time correlated factor.
            self.diagnostics.gnss_updates_rejected =
                self.diagnostics.gnss_updates_rejected.saturating_add(1);
            return Ok(InputDisposition::RetainedForOffline);
        }
        if at_or_after_pending_boundary {
            if let Some(pending) = self.pending_clock_transition {
                if pending.preserve_navigation && self.internal.core.is_active() {
                    // The next-segment timing Jacobian cannot enter the old
                    // consider block while the corrected frontier is still on the
                    // previous segment.
                    self.diagnostics.gnss_updates_rejected =
                        self.diagnostics.gnss_updates_rejected.saturating_add(1);
                    return Ok(InputDisposition::RetainedForOffline);
                }
                let _ = self.commit_clock_transition(pending.observation, false);
                if before_pending_boundary {
                    // Independently timed members straddling the boundary cannot
                    // be fused as one initialization/update transaction.
                    self.diagnostics.gnss_updates_rejected =
                        self.diagnostics.gnss_updates_rejected.saturating_add(1);
                    return Ok(InputDisposition::RetainedForOffline);
                }
            }
        }
        if !self.clock_uncertainty_valid {
            self.integrity = Integrity::Unavailable;
            self.timing_quality = TimingQuality::Discontinuous;
            self.diagnostics.gnss_updates_rejected =
                self.diagnostics.gnss_updates_rejected.saturating_add(1);
            return Ok(InputDisposition::RetainedForOffline);
        }
        if let GnssCorrelationPolicy::FixedDecimation { accept_every } =
            self.engine.dynamics_profile.gnss.correlation
        {
            if !observation
                .id()
                .sequence
                .is_multiple_of(u64::from(accept_every.get()))
            {
                return Ok(InputDisposition::RetainedForOffline);
            }
        }

        let valid_position =
            position.filter(|value| value.solution_class != SolutionClass::Invalid);
        let valid_velocity =
            velocity.filter(|value| value.solution_class != SolutionClass::Invalid);
        if valid_position.is_none() && valid_velocity.is_none() {
            self.diagnostics.gnss_updates_rejected =
                self.diagnostics.gnss_updates_rejected.saturating_add(1);
            return Ok(InputDisposition::RetainedForOffline);
        }
        let lever_definition = self
            .engine
            .calibration
            .shared_parameters
            .definition(self.engine.installation.imu_to_gnss_antenna.parameter_id)
            .ok_or(StepError::WorkspaceContract)?;
        for epoch in [
            valid_position.and_then(|value| value.time.effective_time().ok()),
            valid_velocity.and_then(|value| value.time.effective_time().ok()),
        ]
        .into_iter()
        .flatten()
        {
            if !lever_definition.validity.contains(epoch) {
                return Err(StepError::InvalidObservation(
                    ValidationError::IncompatibleDefinition,
                ));
            }
        }

        let maximum_diagnostic_age_ns = self
            .engine
            .dynamics_profile
            .gnss
            .maximum_correction_age
            .as_ns();
        let position_receiver_healthy = valid_position.is_some_and(|value| {
            receiver_healthy_at(observation, value.time, maximum_diagnostic_age_ns)
        });
        let velocity_receiver_healthy = valid_velocity.is_some_and(|value| {
            receiver_healthy_at(observation, value.time, maximum_diagnostic_age_ns)
        });
        let receiver_healthy = valid_position.is_none_or(|_| position_receiver_healthy)
            && valid_velocity.is_none_or(|_| velocity_receiver_healthy);

        if !self.internal.core.is_active() {
            if !receiver_healthy {
                return Ok(InputDisposition::RetainedForOffline);
            }
            let Some(position) = valid_position else {
                return Ok(InputDisposition::RetainedForOffline);
            };
            let position_epoch = position
                .time
                .effective_time()
                .map_err(StepError::InvalidObservation)?;
            let position_ecef = na_vector(position.value.components());
            let anchor = self.anchor.map_or_else(
                || {
                    EcefAnchor::from_origin(
                        0,
                        position_ecef,
                        self.engine.processing_frame.ellipsoid(),
                    )
                    .map_err(|_| StepError::InvalidObservation(ValidationError::InvalidFrame))
                },
                Ok,
            )?;
            // A position-only fix is useful for choosing the local anchor, but
            // absence of vector velocity is not evidence of zero velocity.
            let Some(velocity) = valid_velocity else {
                self.anchor = Some(anchor);
                self.latest_initialization_fix = None;
                return Ok(InputDisposition::InitializationOnly);
            };
            let velocity_epoch = velocity
                .time
                .effective_time()
                .map_err(StepError::InvalidObservation)?;
            let position_covariance = self.resolve_gnss_covariance(
                position.uncertainty,
                self.engine.dynamics_profile.gnss.position_covariance_floor,
            )?;
            let velocity_n = anchor.vector_from_ecef(na_vector(velocity.value.components()));
            let velocity_covariance_n = rotate_covariance_to_n(
                &anchor,
                self.resolve_gnss_covariance(
                    velocity.uncertainty,
                    self.engine.dynamics_profile.gnss.velocity_covariance_floor,
                )?,
            )?;
            let cross = if velocity_epoch == position_epoch {
                observation
                    .position_velocity_cross_covariance()
                    .map(|value| rotate_cross_to_n(&anchor, value.to_matrix()))
                    .transpose()?
            } else {
                None
            };
            self.latest_initialization_fix = Some(InitializationFixEvidence {
                position_epoch,
                velocity_epoch,
                position_n: anchor.position_from_ecef(position_ecef),
                velocity_n,
                position_covariance_n: rotate_covariance_to_n(&anchor, position_covariance)?,
                velocity_covariance_n,
                position_velocity_cross_n: cross,
                position_independent_timing_sigma_s: position
                    .time
                    .independent_one_sigma
                    .as_seconds_f64() as f32,
            });
            self.anchor = Some(anchor);
            return Ok(InputDisposition::InitializationOnly);
        }

        let anchor = self.anchor.ok_or(StepError::WorkspaceContract)?;
        let multiplier = match self.engine.dynamics_profile.gnss.correlation {
            GnssCorrelationPolicy::SequenceInflation { multiplier } => multiplier.get(),
            GnssCorrelationPolicy::FixedDecimation { .. } => 1.0,
            GnssCorrelationPolicy::GaussMarkov { .. } => {
                return Err(StepError::WorkspaceContract);
            }
        };
        let same_epoch = valid_position
            .zip(valid_velocity)
            .map(|(left, right)| {
                Ok(left
                    .time
                    .effective_time()
                    .map_err(StepError::InvalidObservation)?
                    == right
                        .time
                        .effective_time()
                        .map_err(StepError::InvalidObservation)?)
            })
            .transpose()?
            .unwrap_or(false);
        let shared = SharedMeasurementJacobians::default();
        let lever = vector_f32(
            self.engine
                .installation
                .imu_to_gnss_antenna
                .mean
                .components_m(),
        )?;
        let id = observation.id();
        let mut scheduled = [None, None];
        if same_epoch {
            let position = valid_position.ok_or(StepError::WorkspaceContract)?;
            let velocity = valid_velocity.ok_or(StepError::WorkspaceContract)?;
            let epoch = position
                .time
                .effective_time()
                .map_err(StepError::InvalidObservation)?;
            scheduled[0] = Some(Scheduled {
                key: order_key(epoch, GNSS_POSITION_CLASS, id),
                value: GnssObservation {
                    time: epoch,
                    position_n: Some(
                        anchor.position_from_ecef(na_vector(position.value.components())),
                    ),
                    velocity_n: Some(
                        anchor.vector_from_ecef(na_vector(velocity.value.components())),
                    ),
                    position_covariance_n: rotate_covariance_to_n(
                        &anchor,
                        scale_covariance(
                            self.resolve_gnss_covariance(
                                position.uncertainty,
                                self.engine.dynamics_profile.gnss.position_covariance_floor,
                            )?,
                            multiplier,
                        )?,
                    )?,
                    velocity_covariance_n: rotate_covariance_to_n(
                        &anchor,
                        scale_covariance(
                            self.resolve_gnss_covariance(
                                velocity.uncertainty,
                                self.engine.dynamics_profile.gnss.velocity_covariance_floor,
                            )?,
                            multiplier,
                        )?,
                    )?,
                    position_velocity_cross_n: observation
                        .position_velocity_cross_covariance()
                        .map(|value| {
                            rotate_cross_to_n(&anchor, scale_cross(value.to_matrix(), multiplier))
                        })
                        .transpose()?,
                    imu_to_antenna_b: lever,
                    // Replaced by LiveCore with support-aligned values at the
                    // measurement epoch immediately before linearization.
                    omega_ib_b: NaVector3::zeros(),
                    specific_force_b: NaVector3::zeros(),
                    angular_acceleration_eb_b: None,
                    angular_acceleration_covariance_b: Matrix3::zeros(),
                    clock_consider_start: self.clock_uncertainty_valid.then_some(0),
                    clock_reference_time: self.clock_reference_time,
                    lever_arm_consider_start: Some(self.consider_layout.antenna_lever_start),
                    position_independent_timing_sigma_s: finite_f32(
                        position.time.independent_one_sigma.as_seconds_f64(),
                    )?,
                    velocity_independent_timing_sigma_s: finite_f32(
                        velocity.time.independent_one_sigma.as_seconds_f64(),
                    )?,
                    shared_jacobians: shared,
                    receiver_healthy,
                    quality_state: gnss_state(observation.rtk_state(), receiver_healthy),
                    quality_timing: timing_quality(position.time),
                },
            });
        } else {
            if let Some(position) = valid_position {
                let epoch = position
                    .time
                    .effective_time()
                    .map_err(StepError::InvalidObservation)?;
                scheduled[0] = Some(Scheduled {
                    key: order_key(epoch, GNSS_POSITION_CLASS, id),
                    value: GnssObservation {
                        time: epoch,
                        position_n: Some(
                            anchor.position_from_ecef(na_vector(position.value.components())),
                        ),
                        velocity_n: None,
                        position_covariance_n: rotate_covariance_to_n(
                            &anchor,
                            scale_covariance(
                                self.resolve_gnss_covariance(
                                    position.uncertainty,
                                    self.engine.dynamics_profile.gnss.position_covariance_floor,
                                )?,
                                multiplier,
                            )?,
                        )?,
                        velocity_covariance_n: Matrix3::zeros(),
                        position_velocity_cross_n: None,
                        imu_to_antenna_b: lever,
                        omega_ib_b: NaVector3::zeros(),
                        specific_force_b: NaVector3::zeros(),
                        angular_acceleration_eb_b: None,
                        angular_acceleration_covariance_b: Matrix3::zeros(),
                        clock_consider_start: self.clock_uncertainty_valid.then_some(0),
                        clock_reference_time: self.clock_reference_time,
                        lever_arm_consider_start: Some(self.consider_layout.antenna_lever_start),
                        position_independent_timing_sigma_s: finite_f32(
                            position.time.independent_one_sigma.as_seconds_f64(),
                        )?,
                        velocity_independent_timing_sigma_s: 0.0,
                        shared_jacobians: shared,
                        receiver_healthy: position_receiver_healthy,
                        quality_state: gnss_state(
                            observation.rtk_state(),
                            position_receiver_healthy,
                        ),
                        quality_timing: timing_quality(position.time),
                    },
                });
            }
            if let Some(velocity) = valid_velocity {
                let epoch = velocity
                    .time
                    .effective_time()
                    .map_err(StepError::InvalidObservation)?;
                scheduled[1] = Some(Scheduled {
                    key: order_key(epoch, GNSS_VELOCITY_CLASS, id),
                    value: GnssObservation {
                        time: epoch,
                        position_n: None,
                        velocity_n: Some(
                            anchor.vector_from_ecef(na_vector(velocity.value.components())),
                        ),
                        position_covariance_n: Matrix3::zeros(),
                        velocity_covariance_n: rotate_covariance_to_n(
                            &anchor,
                            scale_covariance(
                                self.resolve_gnss_covariance(
                                    velocity.uncertainty,
                                    self.engine.dynamics_profile.gnss.velocity_covariance_floor,
                                )?,
                                multiplier,
                            )?,
                        )?,
                        position_velocity_cross_n: None,
                        imu_to_antenna_b: lever,
                        omega_ib_b: NaVector3::zeros(),
                        specific_force_b: NaVector3::zeros(),
                        angular_acceleration_eb_b: None,
                        angular_acceleration_covariance_b: Matrix3::zeros(),
                        clock_consider_start: self.clock_uncertainty_valid.then_some(0),
                        clock_reference_time: self.clock_reference_time,
                        lever_arm_consider_start: Some(self.consider_layout.antenna_lever_start),
                        position_independent_timing_sigma_s: 0.0,
                        velocity_independent_timing_sigma_s: finite_f32(
                            velocity.time.independent_one_sigma.as_seconds_f64(),
                        )?,
                        shared_jacobians: shared,
                        receiver_healthy: velocity_receiver_healthy,
                        quality_state: gnss_state(
                            observation.rtk_state(),
                            velocity_receiver_healthy,
                        ),
                        quality_timing: timing_quality(velocity.time),
                    },
                });
            }
        }
        if !self.internal.core.is_active() {
            return Err(StepError::WorkspaceContract);
        }
        let mut core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
        let dispositions = core
            .ingest_gnss_pair(scheduled)
            .map_err(map_core_step_error)?;
        if dispositions
            .iter()
            .flatten()
            .all(|value| *value == EnqueueDisposition::TooLateForLive)
        {
            self.diagnostics.observations_too_late =
                self.diagnostics.observations_too_late.saturating_add(1);
            Ok(InputDisposition::TooLateForLive)
        } else {
            Ok(InputDisposition::QueuedForFusion)
        }
    }

    fn validate_antenna(&self, observation: GnssSolutionObservation) -> Result<(), StepError> {
        let valid = self
            .engine
            .installation
            .reference_points
            .iter()
            .any(|point| {
                point.id() == observation.antenna_reference_point()
                    && point.kind() == ReferencePointKind::GnssAntennaPhaseCenter
                    && point.parameter_id()
                        == self.engine.installation.imu_to_gnss_antenna.parameter_id
            });
        if valid {
            Ok(())
        } else {
            Err(StepError::InvalidObservation(
                ValidationError::InvalidReferencePoint,
            ))
        }
    }

    fn resolve_gnss_covariance(
        &self,
        uncertainty: MeasurementUncertainty<Covariance3>,
        floor: Covariance3,
    ) -> Result<[[f64; 3]; 3], StepError> {
        let covariance = match uncertainty {
            MeasurementUncertainty::Provided(value) => value,
            MeasurementUncertainty::Modeled(id) => self
                .engine
                .uncertainty_models
                .iter()
                .find(|model| model.id == id)
                .and_then(|model| match model.kind {
                    UncertaintyModelKind::ConstantCovariance3(value) => Some(value),
                    UncertaintyModelKind::ConstantVariance(_)
                    | UncertaintyModelKind::SequenceBound { .. } => None,
                })
                .ok_or(StepError::InvalidObservation(
                    ValidationError::InvalidCovariance,
                ))?,
        };
        Ok(add_matrix(covariance.to_matrix(), floor.to_matrix()))
    }
}

fn receiver_healthy_at(
    observation: GnssSolutionObservation,
    measurement_time: ObservationTime,
    maximum_age_ns: u64,
) -> bool {
    let diagnostics = observation.diagnostics();
    let health = diagnostics.health.is_some_and(|value| {
        value.value == ReceiverHealth::Healthy
            && diagnostic_information_age_at(value, measurement_time)
                .is_some_and(|age| age <= maximum_age_ns)
    });
    let correction_fresh = diagnostics.correction_age.is_none_or(|value| {
        diagnostic_information_age_at(value, measurement_time)
            .and_then(|information_age| value.value.as_ns().checked_add(information_age))
            .is_some_and(|age| age <= maximum_age_ns)
    });
    let solution_fresh = diagnostics.solution_age.is_none_or(|value| {
        diagnostic_information_age_at(value, measurement_time)
            .and_then(|information_age| value.value.as_ns().checked_add(information_age))
            .is_some_and(|age| age <= maximum_age_ns)
    });
    health && correction_fresh && solution_fresh
}

fn diagnostic_information_age_at<T: Copy>(
    diagnostic: crate::observation::TimedDiagnostic<T>,
    measurement_time: ObservationTime,
) -> Option<u64> {
    if diagnostic.time.clock_model != measurement_time.clock_model {
        return None;
    }
    let diagnostic_epoch = diagnostic.time.effective_time().ok()?;
    let measurement_epoch = measurement_time.effective_time().ok()?;
    let elapsed_ns = measurement_epoch
        .as_ns()
        .checked_sub(diagnostic_epoch.as_ns())?;
    let elapsed_ns = u64::try_from(elapsed_ns).ok()?;
    diagnostic
        .age
        .as_ns()
        .checked_add(elapsed_ns)?
        .checked_add(diagnostic.time.independent_one_sigma.as_ns())
        .and_then(|age| age.checked_add(measurement_time.independent_one_sigma.as_ns()))
}

pub(super) fn gnss_state(state: RtkState, healthy: bool) -> GnssState {
    if !healthy {
        return GnssState::Suspect;
    }
    match state {
        RtkState::Fixed => GnssState::Fixed,
        RtkState::Float => GnssState::Float,
        RtkState::Dgps => GnssState::Dgps,
        RtkState::Ppp => GnssState::Ppp,
        RtkState::Standalone => GnssState::Standalone,
        RtkState::Invalid => GnssState::Suspect,
    }
}

pub(super) fn timing_quality(time: ObservationTime) -> TimingQuality {
    match time.basis {
        TimingBasis::PpsCorrelated | TimingBasis::SensorCounterAnchored => {
            TimingQuality::PpsCorrelated
        }
        TimingBasis::ModeledLatency => TimingQuality::Modeled,
        TimingBasis::ArrivalOnly => TimingQuality::ArrivalOnly,
    }
}

fn order_key(time: SessionTime, class: u8, id: ObservationId) -> OrderKey {
    OrderKey {
        time,
        class,
        source: id.source.get(),
        sequence: id.sequence,
    }
}
