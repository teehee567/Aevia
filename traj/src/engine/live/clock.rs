//! Clock segment boundaries, uncertainty transitions, and observation clock checks.

use super::conversion::map_core_step_error;
use super::{LiveSession, PendingClockTransition};
use crate::error::{StepError, ValidationError};
use crate::ids::ClockModelId;
use crate::live::{
    ImuInterval, LiveCore, independent_clock_consider_covariance_into,
    transition_consider_covariance_into,
};
use crate::observation::{
    ClockTransitionObservation, ClockTransitionUncertainty, InputDisposition,
};
use crate::quality::{Integrity, TimingQuality};
use crate::time::SessionTime;

impl LiveSession<'_, '_> {
    #[inline(never)]
    pub(super) fn ingest_clock_transition(
        &mut self,
        transition: &ClockTransitionObservation,
    ) -> Result<InputDisposition, StepError> {
        transition
            .validate()
            .map_err(StepError::InvalidObservation)?;
        let already_finalized = self
            .last_clock_transition_time
            .is_some_and(|previous| transition.at <= previous)
            || self
                .psram
                .trajectory
                .span()
                .is_some_and(|span| transition.at <= span.end())
            || if self.internal.core.is_active() {
                LiveCore::attach(&mut self.internal.core, &mut self.psram.history)
                    .status()
                    .map_err(map_core_step_error)?
                    .corrected_frontier
                    .is_some_and(|frontier| transition.at <= frontier)
            } else {
                false
            };
        if already_finalized {
            return Ok(InputDisposition::TooLateForLive);
        }
        if self.pending_clock_transition.is_some()
            || transition.next_segment.get() <= self.current_clock_segment.get()
        {
            return Err(StepError::ClockDiscontinuity);
        }
        if transition.previous_model != self.current_clock_model {
            return Err(StepError::ClockDiscontinuity);
        }
        let active_consider = usize::from(self.engine.navigation_profile.consider_dimension);
        self.preflight_clock_transition_uncertainty(transition, active_consider)?;
        let affine = matches!(
            transition.uncertainty,
            ClockTransitionUncertainty::AffineBridge(_)
        );
        let preserve_navigation = affine
            && self.clock_uncertainty_valid
            && self.internal.core.is_active()
            && LiveCore::attach(&mut self.internal.core, &mut self.psram.history)
                .can_schedule_clock_transition_at(transition.at);
        let boundary_is_future = self
            .last_accepted_imu_end
            .is_none_or(|trusted_through| transition.at > trusted_through);
        let corrected_boundary_is_future = preserve_navigation
            && LiveCore::attach(&mut self.internal.core, &mut self.psram.history)
                .status()
                .map_err(map_core_step_error)?
                .corrected_frontier
                .is_none_or(|frontier| frontier < transition.at);
        if boundary_is_future || corrected_boundary_is_future {
            self.pending_clock_transition = Some(PendingClockTransition {
                observation: *transition,
                preserve_navigation,
            });
            return Ok(InputDisposition::QueuedForFusion);
        }
        Ok(self.commit_clock_transition(*transition, false))
    }

    #[inline(never)]
    pub(super) fn commit_clock_transition(
        &mut self,
        transition: ClockTransitionObservation,
        preserve_navigation: bool,
    ) -> InputDisposition {
        let active_consider = usize::from(self.engine.navigation_profile.consider_dimension);
        let preserved = match transition.uncertainty {
            ClockTransitionUncertainty::AffineBridge(bridge) => {
                // Preflight staged this exact seed before the transition was
                // accepted. Recompute at the boundary because an earlier
                // qualified transition may have changed the seed while this
                // control waited behind the corrected frontier.
                let seed_valid = transition_consider_covariance_into(
                    &self.internal.consider_seed_covariance,
                    active_consider,
                    bridge.next_clock_from_previous_consider(),
                    bridge.innovation_covariance_upper(),
                    &mut self.psram.consider_seed_transaction,
                )
                .is_ok();
                let navigation_preserved =
                    seed_valid && preserve_navigation && self.internal.core.is_active() && {
                        let mut core =
                            LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
                        matches!(
                            core.transition_clock_consider(
                                active_consider,
                                bridge.next_clock_from_previous_consider(),
                                bridge.innovation_covariance_upper(),
                            ),
                            Ok(())
                        )
                    };
                if seed_valid {
                    self.internal
                        .consider_seed_covariance
                        .copy_from(&self.psram.consider_seed_transaction);
                }
                if !navigation_preserved {
                    self.invalidate_navigation_at(transition.at);
                    self.latest_initialization_fix = None;
                }
                self.clock_reference_time = bridge.next_reference_time();
                self.clock_uncertainty_valid = seed_valid;
                navigation_preserved
            }
            ClockTransitionUncertainty::IndependentPrior(prior) => {
                let seed_valid = independent_clock_consider_covariance_into(
                    &self.internal.consider_seed_covariance,
                    active_consider,
                    prior.covariance_upper(),
                    &mut self.psram.consider_seed_transaction,
                )
                .is_ok();
                if seed_valid {
                    self.internal
                        .consider_seed_covariance
                        .copy_from(&self.psram.consider_seed_transaction);
                }
                // Independence from the old navigation state is true only
                // after that state and its Pxc have been discarded.
                self.invalidate_navigation_at(transition.at);
                self.latest_initialization_fix = None;
                self.clock_reference_time = prior.reference_time();
                self.clock_uncertainty_valid = seed_valid;
                false
            }
            ClockTransitionUncertainty::Unavailable => {
                self.invalidate_navigation_at(transition.at);
                self.latest_initialization_fix = None;
                self.clock_uncertainty_valid = false;
                false
            }
        };
        self.current_clock_model = transition.next_model;
        self.current_clock_segment = transition.next_segment;
        self.last_clock_transition_time = Some(transition.at);
        self.pending_clock_transition = None;
        self.timing_quality = TimingQuality::Discontinuous;
        self.integrity = Integrity::Unavailable;
        self.diagnostics.clock_discontinuities =
            self.diagnostics.clock_discontinuities.saturating_add(1);
        if preserved {
            InputDisposition::Fused
        } else {
            InputDisposition::RetainedForOffline
        }
    }

    fn preflight_clock_transition_uncertainty(
        &mut self,
        transition: &ClockTransitionObservation,
        active_consider: usize,
    ) -> Result<(), StepError> {
        match transition.uncertainty {
            ClockTransitionUncertainty::AffineBridge(bridge) => {
                if bridge.active_consider_dimension() != active_consider {
                    return Err(StepError::InvalidObservation(
                        ValidationError::IncompatibleDefinition,
                    ));
                }
                transition_consider_covariance_into(
                    &self.internal.consider_seed_covariance,
                    active_consider,
                    bridge.next_clock_from_previous_consider(),
                    bridge.innovation_covariance_upper(),
                    &mut self.psram.consider_seed_transaction,
                )
                .map_err(|_| StepError::InvalidObservation(ValidationError::InvalidCovariance))?;
            }
            ClockTransitionUncertainty::IndependentPrior(prior) => {
                independent_clock_consider_covariance_into(
                    &self.internal.consider_seed_covariance,
                    active_consider,
                    prior.covariance_upper(),
                    &mut self.psram.consider_seed_transaction,
                )
                .map_err(|_| StepError::InvalidObservation(ValidationError::InvalidCovariance))?;
            }
            ClockTransitionUncertainty::Unavailable => {}
        }
        Ok(())
    }

    pub(super) fn validate_point_clock_model(
        &self,
        observed: ClockModelId,
        at: SessionTime,
    ) -> Result<(), StepError> {
        let expected = self
            .pending_clock_transition
            .map_or(self.current_clock_model, |pending| {
                if at < pending.observation.at {
                    self.current_clock_model
                } else {
                    pending.observation.next_model
                }
            });
        if expected.is_some_and(|model| model != observed) {
            Err(StepError::ClockDiscontinuity)
        } else {
            Ok(())
        }
    }

    pub(super) fn prepare_imu_clock_boundary(
        &mut self,
        observed: ClockModelId,
        interval: ImuInterval,
    ) -> Result<(), StepError> {
        let Some(pending) = self.pending_clock_transition else {
            return self.validate_point_clock_model(observed, interval.end);
        };
        let boundary = pending.observation.at;
        let expected = if interval.end <= boundary {
            self.current_clock_model
        } else if interval.start >= boundary {
            pending.observation.next_model
        } else {
            // A single fitted clock segment cannot truthfully label an IMU
            // average whose support straddles the segment boundary.
            return Err(StepError::ClockDiscontinuity);
        };
        if expected.is_some_and(|model| model != observed) {
            return Err(StepError::ClockDiscontinuity);
        }
        if interval.start >= boundary
            && (!pending.preserve_navigation || !self.internal.core.is_active())
        {
            let _ = self.commit_clock_transition(pending.observation, false);
        }
        Ok(())
    }
}
