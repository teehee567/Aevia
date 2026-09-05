//! Ambiguity regression tests.

use super::*;

#[test]
fn every_cycle_slip_detector_ends_the_arc() {
    let clean = CycleSlipEvidence::NONE
        .with(ContinuityIndicator::ReceiverCycleSlip, false)
        .with(ContinuityIndicator::ReceiverLossOfLock, false)
        .with(ContinuityIndicator::HalfCycleDiscontinuity, false)
        .with(ContinuityIndicator::LockTimeReset, false)
        .with(ContinuityIndicator::GeometryFreeDiscontinuity, false)
        .with(ContinuityIndicator::MelbourneWubbenaDiscontinuity, false)
        .with(ContinuityIndicator::DopplerPhaseDiscontinuity, false)
        .with(ContinuityIndicator::InnovationDiscontinuity, false)
        .with(ContinuityIndicator::ReceiverClockJump, false)
        .with(ContinuityIndicator::ValidatedDiscontinuity, false);
    assert_eq!(
        clean.continuity_event(),
        AmbiguityContinuityEvent::Continuous
    );
    for indicator in [
        ContinuityIndicator::ReceiverCycleSlip,
        ContinuityIndicator::HalfCycleDiscontinuity,
        ContinuityIndicator::LockTimeReset,
        ContinuityIndicator::GeometryFreeDiscontinuity,
        ContinuityIndicator::MelbourneWubbenaDiscontinuity,
        ContinuityIndicator::DopplerPhaseDiscontinuity,
        ContinuityIndicator::InnovationDiscontinuity,
    ] {
        assert_eq!(
            clean.with(indicator, true).continuity_event(),
            AmbiguityContinuityEvent::ReceiverCycleSlip
        );
    }
    assert_eq!(
        clean
            .with(ContinuityIndicator::ReceiverLossOfLock, true)
            .continuity_event(),
        AmbiguityContinuityEvent::LossOfLock
    );
    assert_eq!(
        clean
            .with(ContinuityIndicator::ReceiverClockJump, true)
            .continuity_event(),
        AmbiguityContinuityEvent::ReceiverClockJump
    );
    assert!(clean.is_available(ContinuityIndicator::ReceiverCycleSlip));
    assert!(!CycleSlipEvidence::NONE.is_available(ContinuityIndicator::ReceiverCycleSlip));
}

#[test]
fn reference_change_preserves_physical_arc_and_requires_covariance_transform() {
    let mut state = AmbiguityArcState::new(
        AmbiguitySignalKey {
            satellite: satellite(3),
            signal_code: 1,
        },
        AmbiguityArcId::new(10).unwrap(),
        Some(satellite(7)),
    )
    .unwrap();
    let transition = state
        .apply(AmbiguityContinuityEvent::ReferenceSatelliteChanged {
            new_reference: satellite(8),
        })
        .unwrap();
    assert_eq!(state.arc().get(), 10);
    assert_eq!(
        transition,
        AmbiguityArcTransition::Rereferenced {
            arc: AmbiguityArcId::new(10).unwrap(),
            previous_reference: Some(satellite(7)),
            new_reference: satellite(8),
            covariance_action: ReferenceCovarianceAction::FullLinearReparameterizationRequired,
        }
    );
}

#[test]
fn slips_end_arcs_but_integrity_failure_only_resets_fix_hold() {
    let mut state = AmbiguityArcState::new(
        AmbiguitySignalKey {
            satellite: satellite(3),
            signal_code: 1,
        },
        AmbiguityArcId::new(4).unwrap(),
        None,
    )
    .unwrap();
    let fix = ConditionalAmbiguityFix {
        kind: ConditionalFixKind::Partial,
        fixed_ambiguity_count: 2,
        available_ambiguity_count: 3,
        hypothesis_digest: digest(44),
        uncertainty_basis: ConditionalUncertaintyBasis::SelectedRobustWeightsAndIntegerHypothesis,
    };
    state.accept_conditional_fix(fix).unwrap();
    assert_eq!(
        state
            .apply(AmbiguityContinuityEvent::IntegrityFailure)
            .unwrap(),
        AmbiguityArcTransition::FixResetWithoutEndingArc {
            arc: AmbiguityArcId::new(4).unwrap()
        }
    );
    assert_eq!(state.arc().get(), 4);
    assert_eq!(state.fix_state(), AmbiguityFixState::Float);
    assert_eq!(
        state
            .apply(AmbiguityContinuityEvent::ReceiverCycleSlip)
            .unwrap(),
        AmbiguityArcTransition::Restarted {
            ended: AmbiguityArcId::new(4).unwrap(),
            started: AmbiguityArcId::new(5).unwrap(),
            reason: ArcTerminationReason::ReceiverCycleSlip,
        }
    );
}
