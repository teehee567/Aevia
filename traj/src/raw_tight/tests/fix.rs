//! Fix regression tests.

use super::*;

fn thresholds() -> AmbiguityFixThresholds {
    AmbiguityFixThresholds {
        minimum_ratio: 3.0,
        minimum_success_rate: 0.999,
        maximum_residual_rms: 0.03,
        minimum_temporal_consistency_epochs: 5,
    }
}

fn fix_evidence() -> AmbiguityFixEvidence {
    AmbiguityFixEvidence {
        ratio: 4.0,
        success_rate: 0.9999,
        residual_rms: 0.01,
        temporally_consistent_epochs: 10,
        fixed_ambiguity_count: 8,
        available_ambiguity_count: 10,
        slip_free: true,
        integrity_valid: true,
        hypothesis_digest: digest(90),
    }
}

#[test]
fn a_fix_cannot_exist_without_a_qualified_registered_backend() {
    assert_eq!(
        assess_ambiguity_fix(None, thresholds(), fix_evidence()),
        Err(FixAssessmentError::BackendNotRegistered)
    );
    let mut backend = registration();
    backend.qualification.passed = false;
    assert_eq!(
        assess_ambiguity_fix(Some(&backend), thresholds(), fix_evidence()),
        Err(FixAssessmentError::InvalidRegistration(
            RawTightRegistrationError::QualificationFailed
        ))
    );
}

#[test]
fn accepted_integer_result_is_partial_and_explicitly_conditional() {
    assert_eq!(
        assess_ambiguity_fix(Some(&registration()), thresholds(), fix_evidence()).unwrap(),
        AmbiguityFixDecision::Conditional(ConditionalAmbiguityFix {
            kind: ConditionalFixKind::Partial,
            fixed_ambiguity_count: 8,
            available_ambiguity_count: 10,
            hypothesis_digest: digest(90),
            uncertainty_basis:
                ConditionalUncertaintyBasis::SelectedRobustWeightsAndIntegerHypothesis,
        })
    );
}

#[test]
fn failed_integer_validation_remains_float_with_precise_reason() {
    let mut weak = fix_evidence();
    weak.ratio = 2.0;
    assert_eq!(
        assess_ambiguity_fix(Some(&registration()), thresholds(), weak).unwrap(),
        AmbiguityFixDecision::Float(FixRejectionReason::RatioBelowThreshold)
    );
}
