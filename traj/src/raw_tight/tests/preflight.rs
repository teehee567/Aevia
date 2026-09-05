//! Preflight regression tests.

use super::*;

#[test]
fn complete_evidence_still_fails_without_registered_backend() {
    let selections = required_selections();
    let lineage = EvidenceLineage::new(&selections).unwrap();
    let failure = preflight(request(lineage), limits()).unwrap_err();
    assert_eq!(failure.error, RawTightPreflightError::BackendNotRegistered);
    assert!(failure.estimate.is_some());
    assert_eq!(
        failure.diagnostic().final_outcome,
        Some(ProcessingAttemptOutcome::NotCompiled)
    );
}

#[test]
fn missing_capabilities_are_reported_individually() {
    let selections = required_selections();
    let lineage = EvidenceLineage::new(&selections).unwrap();
    for missing in [
        Capability::NormalizedImu,
        Capability::RawRoverGnss,
        Capability::BaseOrCorrection,
        Capability::Ephemerides,
        Capability::Timing,
        Capability::Configuration,
        Capability::CompleteEnd,
    ] {
        let mut candidate = request(lineage);
        candidate.capabilities = Capabilities::NONE;
        for capability in [
            Capability::NormalizedImu,
            Capability::RawRoverGnss,
            Capability::BaseOrCorrection,
            Capability::Ephemerides,
            Capability::Timing,
            Capability::Configuration,
            Capability::CompleteEnd,
        ] {
            if capability != missing {
                candidate.capabilities = candidate.capabilities.with(capability);
            }
        }
        assert_eq!(
            preflight_with_registration(candidate, limits(), Some(&registration()))
                .unwrap_err()
                .error,
            RawTightPreflightError::MissingEvidence(missing)
        );
    }
}

#[test]
fn raw_tight_cannot_be_selected_implicitly() {
    let selections = required_selections();
    let lineage = EvidenceLineage::new(&selections).unwrap();
    let mut candidate = request(lineage);
    candidate.explicitly_required = false;
    assert_eq!(
        preflight_with_registration(candidate, limits(), Some(&registration()))
            .unwrap_err()
            .error,
        RawTightPreflightError::ExplicitSelectionRequired
    );
}

#[test]
fn semantic_evidence_checks_fail_individually() {
    let selections = required_selections();
    let lineage = EvidenceLineage::new(&selections).unwrap();
    let complete = request(lineage).evidence_checks;
    for expected in [
        RawTightEvidenceRequirement::RoverBaseEpochAlignment,
        RawTightEvidenceRequirement::BaseCoordinateFrameAndEpoch,
        RawTightEvidenceRequirement::SignalFrequencyMetadata,
        RawTightEvidenceRequirement::EphemerisIssueAndValidity,
        RawTightEvidenceRequirement::CorrectionAge,
        RawTightEvidenceRequirement::TrackingContinuity,
        RawTightEvidenceRequirement::DifferencedCovarianceInputs,
    ] {
        let mut candidate = request(lineage);
        candidate.evidence_checks = complete.without(expected);
        assert_eq!(
            preflight_with_registration(candidate, limits(), Some(&registration()))
                .unwrap_err()
                .error,
            RawTightPreflightError::EvidenceCheckFailed(expected)
        );
    }
}

#[test]
fn fused_pvt_is_rejected_even_when_recorded_under_another_source() {
    let required = required_selections();
    let pvt = selection(
        99,
        EvidenceClass::GnssSolution,
        EvidenceLineageKind::Captured,
        EvidenceUse::Fusion,
    );
    let selections = [
        required[0],
        required[1],
        required[2],
        required[3],
        required[4],
        required[5],
        pvt,
    ];
    let lineage = EvidenceLineage::new(&selections).unwrap();
    assert_eq!(
        preflight_with_registration(request(lineage), limits(), Some(&registration()))
            .unwrap_err()
            .error,
        RawTightPreflightError::PvtDoubleUse {
            source: SourceId::new(99)
        }
    );
}

#[test]
fn pvt_is_permitted_only_as_initialization_or_diagnostic_evidence() {
    for usage in [EvidenceUse::InitializationOnly, EvidenceUse::DiagnosticOnly] {
        let required = required_selections();
        let pvt = selection(
            99,
            EvidenceClass::GnssSolution,
            EvidenceLineageKind::Captured,
            usage,
        );
        let selections = [
            required[0],
            required[1],
            required[2],
            required[3],
            required[4],
            required[5],
            pvt,
        ];
        let lineage = EvidenceLineage::new(&selections).unwrap();
        assert!(
            preflight_with_registration(request(lineage), limits(), Some(&registration())).is_ok()
        );
    }
}

#[test]
fn incomplete_span_lineage_is_not_hidden_by_manifest_capability() {
    let mut selections = required_selections();
    selections[3].span = span(0, 49);
    let lineage = EvidenceLineage::new(&selections).unwrap();
    assert_eq!(
        preflight_with_registration(request(lineage), limits(), Some(&registration()))
            .unwrap_err()
            .error,
        RawTightPreflightError::MissingLineage(EvidenceClass::Ephemeris)
    );
}
