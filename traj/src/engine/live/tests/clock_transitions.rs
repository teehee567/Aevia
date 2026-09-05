//! Clock transitions.

use super::*;

#[test]
fn future_clock_transition_is_queued_without_applying_its_covariance_or_model_early() {
    with_large_stack(
        future_clock_transition_is_queued_without_applying_its_covariance_or_model_early_case,
    );
}

fn future_clock_transition_is_queued_without_applying_its_covariance_or_model_early_case() {
    with_navigating_live_session(|session| {
        let seed_before = session.internal.consider_seed_covariance;
        let diagnostics_before = session.diagnostics;
        let transition = clock_transition_at(
            1,
            SessionTime::from_ns(REPLAY_END_NS + 10_000_000),
            Some(ClockModelId::new(1)),
            Some(ClockModelId::new(2)),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(affine_bridge(1.0)),
        );

        let update = session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((transition.id(), InputDisposition::QueuedForFusion))
        );
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.internal.consider_seed_covariance, seed_before);
        assert_eq!(session.diagnostics, diagnostics_before);
        assert_eq!(session.phase(), LivePhase::Navigating);
    });
}

#[test]
fn affine_clock_transition_applies_only_when_corrected_frontier_reaches_boundary() {
    with_large_stack(
        affine_clock_transition_applies_only_when_corrected_frontier_reaches_boundary_case,
    );
}

fn affine_clock_transition_applies_only_when_corrected_frontier_reaches_boundary_case() {
    with_navigating_live_session(|session| {
        let boundary = SessionTime::from_ns(REPLAY_END_NS + 10_000_000);
        let bridge = affine_bridge_at(1.0, boundary);
        let mut expected = ConsiderCovariance::zeros();
        transition_consider_covariance_into(
            &session.internal.consider_seed_covariance,
            8,
            bridge.next_clock_from_previous_consider(),
            bridge.innovation_covariance_upper(),
            &mut expected,
        )
        .unwrap();
        let transition = clock_transition_at(
            1,
            boundary,
            Some(ClockModelId::new(1)),
            Some(ClockModelId::new(2)),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(bridge),
        );
        session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        for observation in [
            stationary_imu(7, 35_000_000),
            stationary_imu(8, 40_000_000),
            stationary_imu_for_model(9, 45_000_000, ClockModelId::new(2)),
        ] {
            session
                .step(LiveStep {
                    observation: Some(&observation),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
        }
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
        assert_ne!(session.internal.consider_seed_covariance, expected);

        let crossing = stationary_imu_for_model(10, 50_000_000, ClockModelId::new(2));
        let update = session
            .step(LiveStep {
                observation: Some(&crossing),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(update.input, Some((crossing.id(), InputDisposition::Fused)));
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(2)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(2));
        assert_eq!(session.last_clock_transition_time, Some(boundary));
        assert_eq!(session.internal.consider_seed_covariance, expected);
        assert!(session.pending_clock_transition.is_none());
        assert_eq!(session.phase(), LivePhase::Navigating);
    });
}

#[test]
fn future_independent_clock_prior_reinitializes_at_first_next_segment_support() {
    with_large_stack(
        future_independent_clock_prior_reinitializes_at_first_next_segment_support_case,
    );
}

fn future_independent_clock_prior_reinitializes_at_first_next_segment_support_case() {
    with_navigating_live_session(|session| {
        let boundary = SessionTime::from_ns(40_000_000);
        let prior = IndependentClockPrior::new(boundary, [2.0, 0.25, 3.0]).unwrap();
        let mut expected = ConsiderCovariance::zeros();
        independent_clock_consider_covariance_into(
            &session.internal.consider_seed_covariance,
            8,
            prior.covariance_upper(),
            &mut expected,
        )
        .unwrap();
        let transition = clock_transition_at(
            1,
            boundary,
            Some(ClockModelId::new(1)),
            Some(ClockModelId::new(2)),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::IndependentPrior(prior),
        );
        let queued = session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();
        assert_eq!(
            queued.input,
            Some((transition.id(), InputDisposition::QueuedForFusion))
        );
        assert_eq!(session.phase(), LivePhase::Navigating);

        for observation in [stationary_imu(7, 35_000_000), stationary_imu(8, 40_000_000)] {
            session
                .step(LiveStep {
                    observation: Some(&observation),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
        }
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));

        let first_next = stationary_imu_for_model(9, 45_000_000, ClockModelId::new(2));
        let update = session
            .step(LiveStep {
                observation: Some(&first_next),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((first_next.id(), InputDisposition::InitializationOnly))
        );
        assert_eq!(update.phase, LivePhase::Initializing);
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(2)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(2));
        assert_eq!(session.last_clock_transition_time, Some(boundary));
        assert_eq!(session.clock_reference_time, boundary);
        assert_eq!(session.internal.consider_seed_covariance, expected);
        assert!(session.pending_clock_transition.is_none());
    });
}

#[test]
fn future_unavailable_clock_transition_invalidates_only_at_the_boundary() {
    with_large_stack(future_unavailable_clock_transition_invalidates_only_at_the_boundary_case);
}

fn future_unavailable_clock_transition_invalidates_only_at_the_boundary_case() {
    with_navigating_live_session(|session| {
        let boundary = SessionTime::from_ns(40_000_000);
        let transition = clock_transition_at(
            1,
            boundary,
            Some(ClockModelId::new(1)),
            None,
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::Unavailable,
        );
        session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();
        assert_eq!(session.phase(), LivePhase::Navigating);
        assert!(session.clock_uncertainty_valid);

        let old = stationary_imu(7, 35_000_000);
        session
            .step(LiveStep {
                observation: Some(&old),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));

        let unqualified = stationary_imu_for_model(8, 45_000_000, ClockModelId::new(99));
        let update = session
            .step(LiveStep {
                observation: Some(&unqualified),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((unqualified.id(), InputDisposition::RetainedForOffline))
        );
        assert_eq!(update.phase, LivePhase::Initializing);
        assert_eq!(session.current_clock_model, None);
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(2));
        assert!(!session.clock_uncertainty_valid);
        assert!(session.pending_clock_transition.is_none());
    });
}

#[test]
fn imu_support_may_not_straddle_a_pending_clock_boundary() {
    with_large_stack(imu_support_may_not_straddle_a_pending_clock_boundary_case);
}

fn imu_support_may_not_straddle_a_pending_clock_boundary_case() {
    with_navigating_live_session(|session| {
        let boundary = SessionTime::from_ns(37_500_000);
        let transition = clock_transition_at(
            1,
            boundary,
            Some(ClockModelId::new(1)),
            Some(ClockModelId::new(2)),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(affine_bridge_at(1.0, boundary)),
        );
        session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();
        let before = session.internal.consider_seed_covariance;
        let crossing = stationary_imu_for_model(7, 40_000_000, ClockModelId::new(2));

        assert!(matches!(
            session.step(LiveStep {
                observation: Some(&crossing),
                work: WorkQuota::new(128).unwrap(),
            }),
            Err(StepError::ClockDiscontinuity)
        ));
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
        assert_eq!(session.internal.consider_seed_covariance, before);
        assert!(session.pending_clock_transition.is_some());
    });
}

#[test]
fn finalized_clock_boundary_is_too_late_without_mutating_clock_state() {
    with_large_stack(finalized_clock_boundary_is_too_late_without_mutating_clock_state_case);
}

fn finalized_clock_boundary_is_too_late_without_mutating_clock_state_case() {
    with_navigating_live_session(|session| {
        let seed_before = session.internal.consider_seed_covariance;
        let diagnostics_before = session.diagnostics;
        let transition = clock_transition_at(
            1,
            SessionTime::from_ns(10_000_000),
            Some(ClockModelId::new(1)),
            Some(ClockModelId::new(2)),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(affine_bridge(1.0)),
        );

        let update = session
            .step(LiveStep {
                observation: Some(&transition),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((transition.id(), InputDisposition::TooLateForLive))
        );
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
        assert_eq!(session.internal.consider_seed_covariance, seed_before);
        assert_eq!(session.diagnostics, diagnostics_before);
        assert!(session.pending_clock_transition.is_none());
    });
}

#[test]
fn duplicate_or_regressing_clock_segment_is_rejected_transactionally() {
    with_large_stack(duplicate_or_regressing_clock_segment_is_rejected_transactionally_case);
}

fn duplicate_or_regressing_clock_segment_is_rejected_transactionally_case() {
    with_navigating_live_session(|session| {
        let seed_before = session.internal.consider_seed_covariance;
        let diagnostics_before = session.diagnostics;
        let duplicate_segment = clock_transition(
            1,
            ClockModelId::new(2),
            ClockSegmentId::new(1),
            ClockTransitionUncertainty::AffineBridge(affine_bridge(1.0)),
        );

        assert!(matches!(
            session.step(LiveStep {
                observation: Some(&duplicate_segment),
                work: WorkQuota::new(128).unwrap(),
            }),
            Err(StepError::ClockDiscontinuity)
        ));
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));
        assert_eq!(session.internal.consider_seed_covariance, seed_before);
        assert_eq!(session.diagnostics, diagnostics_before);
        assert!(session.pending_clock_transition.is_none());
    });
}

#[test]
fn affine_clock_transition_commits_psram_seed_at_the_corrected_boundary() {
    with_large_stack(affine_clock_transition_commits_psram_seed_at_the_corrected_boundary_case);
}

fn affine_clock_transition_commits_psram_seed_at_the_corrected_boundary_case() {
    with_navigating_live_session(|session| {
        let seed_before = session.internal.consider_seed_covariance;
        let bridge = affine_bridge(1.0);
        let mut expected = ConsiderCovariance::zeros();
        transition_consider_covariance_into(
            &session.internal.consider_seed_covariance,
            8,
            bridge.next_clock_from_previous_consider(),
            bridge.innovation_covariance_upper(),
            &mut expected,
        )
        .unwrap();
        let observation = clock_transition(
            1,
            ClockModelId::new(2),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(bridge),
        );

        let update = session
            .step(LiveStep {
                observation: Some(&observation),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((observation.id(), InputDisposition::QueuedForFusion))
        );
        assert_eq!(update.phase, LivePhase::Navigating);
        assert_eq!(session.internal.consider_seed_covariance, seed_before);
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(1));

        for observation in [
            stationary_imu_for_model(7, 35_000_000, ClockModelId::new(2)),
            stationary_imu_for_model(8, 40_000_000, ClockModelId::new(2)),
        ] {
            session
                .step(LiveStep {
                    observation: Some(&observation),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
        }

        assert_eq!(session.internal.consider_seed_covariance, expected);
        assert_eq!(session.psram.consider_seed_transaction, expected);
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(2)));
        assert_eq!(session.current_clock_segment, ClockSegmentId::new(2));
        assert_eq!(session.phase(), LivePhase::Navigating);
    });
}

#[test]
fn rejected_affine_clock_candidate_never_commits_and_sequence_can_retry() {
    with_large_stack(rejected_affine_clock_candidate_never_commits_and_sequence_can_retry_case);
}

fn rejected_affine_clock_candidate_never_commits_and_sequence_can_retry_case() {
    with_navigating_live_session(|session| {
        let seed_before = session.internal.consider_seed_covariance;
        let diagnostics_before = session.diagnostics;
        let invalid = clock_transition(
            1,
            ClockModelId::new(2),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(affine_bridge(f32::MAX)),
        );

        assert!(matches!(
            session.step(LiveStep {
                observation: Some(&invalid),
                work: WorkQuota::new(128).unwrap(),
            }),
            Err(StepError::InvalidObservation(
                ValidationError::InvalidCovariance
            ))
        ));
        assert_eq!(session.internal.consider_seed_covariance, seed_before);
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(1)));
        assert_eq!(session.diagnostics, diagnostics_before);
        assert_eq!(session.phase(), LivePhase::Navigating);

        let retry = clock_transition(
            1,
            ClockModelId::new(2),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::AffineBridge(affine_bridge(1.0)),
        );
        assert!(
            session
                .step(LiveStep {
                    observation: Some(&retry),
                    work: WorkQuota::new(128).unwrap(),
                })
                .is_ok()
        );
    });
}

#[test]
fn independent_clock_prior_commits_seed_before_navigation_is_discarded() {
    with_large_stack(independent_clock_prior_commits_seed_before_navigation_is_discarded_case);
}

fn independent_clock_prior_commits_seed_before_navigation_is_discarded_case() {
    with_navigating_live_session(|session| {
        let prior =
            IndependentClockPrior::new(SessionTime::from_ns(REPLAY_END_NS), [2.0, 0.25, 3.0])
                .unwrap();
        let mut expected = ConsiderCovariance::zeros();
        independent_clock_consider_covariance_into(
            &session.internal.consider_seed_covariance,
            8,
            prior.covariance_upper(),
            &mut expected,
        )
        .unwrap();
        let observation = clock_transition(
            1,
            ClockModelId::new(2),
            ClockSegmentId::new(2),
            ClockTransitionUncertainty::IndependentPrior(prior),
        );

        let update = session
            .step(LiveStep {
                observation: Some(&observation),
                work: WorkQuota::new(128).unwrap(),
            })
            .unwrap();

        assert_eq!(
            update.input,
            Some((observation.id(), InputDisposition::RetainedForOffline))
        );
        assert_eq!(update.phase, LivePhase::Initializing);
        assert_eq!(session.internal.consider_seed_covariance, expected);
        assert_eq!(session.psram.consider_seed_transaction, expected);
        assert_eq!(session.current_clock_model, Some(ClockModelId::new(2)));
    });
}
