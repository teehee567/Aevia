//! Regression tests for captured replay preflight tests.

use super::replay::{
    validate_replay_end, validate_replay_finish_call, validate_replay_step_call,
    verify_replay_identity,
};
use super::selection::{offline_level_evidence_preflight, solution_level_lineage_available};
use super::*;
use crate::observation::WorkQuota;
use crate::offline::{CapturedLiveFinishCall, CapturedLiveStepCall};
use crate::provenance::{EvidenceClass, EvidenceLineageKind, EvidenceUse};
use crate::time::{SessionTime, TimeSpan};
use crate::{
    ids::{ContentDigestV1, NormalizationRevision, SourceId},
    provenance::{EvidenceLineage, EvidenceSelection},
};

fn complete_capabilities() -> Capabilities {
    Capabilities::NONE
        .with(Capability::CapturedReplay)
        .with(Capability::OfflineSmooth)
        .with(Capability::NormalizedImu)
        .with(Capability::GnssSolution)
        .with(Capability::Timing)
        .with(Capability::Configuration)
        .with(Capability::CompleteEnd)
}

#[test]
fn captured_capability_bit_cannot_select_an_inexact_runner() {
    assert_eq!(
        offline_level_evidence_preflight(
            ProcessingLevel::CapturedReplay,
            complete_capabilities(),
            true,
            false,
        ),
        Err(PrepareError::ReplayContractUnavailable)
    );
    assert_eq!(
        offline_level_evidence_preflight(
            ProcessingLevel::OfflineSmooth,
            complete_capabilities(),
            true,
            false,
        ),
        Ok(())
    );
    assert_eq!(
        offline_level_evidence_preflight(
            ProcessingLevel::OfflineSmooth,
            complete_capabilities(),
            false,
            false,
        ),
        Err(PrepareError::EvidenceUnavailable)
    );

    let without_captured_records = complete_capabilities();
    let without_captured_records = Capabilities::from_bits(
        without_captured_records.bits() & !(1_u64 << Capability::CapturedReplay as u8),
    )
    .unwrap();
    assert_eq!(
        offline_level_evidence_preflight(
            ProcessingLevel::CapturedReplay,
            without_captured_records,
            true,
            true,
        ),
        Err(PrepareError::EvidenceUnavailable)
    );
    assert_eq!(
        offline_level_evidence_preflight(
            ProcessingLevel::CapturedReplay,
            complete_capabilities(),
            true,
            true,
        ),
        Ok(())
    );
}

#[test]
fn replay_calls_fail_closed_on_missing_finish_identity_and_wrong_order() {
    let digest = ContentDigestV1::from_bytes([9; 32]);
    assert_eq!(
        validate_replay_step_call(
            CapturedLiveStepCall {
                call_index: 1,
                observation_record_sequence: None,
                work: WorkQuota::new(1).unwrap(),
                expected_bit_exact_update_digest: digest,
            },
            0,
            2,
        ),
        Err(ProcessError::InvalidEvidence)
    );
    assert_eq!(
        validate_replay_finish_call(
            CapturedLiveFinishCall {
                call_index: 0,
                work: WorkQuota::new(1).unwrap(),
                expected_complete: true,
                expected_bit_exact_update_digest: digest,
                expected_summary_digest: None,
            },
            0,
            2,
        ),
        Err(ProcessError::InvalidEvidence)
    );
    assert_eq!(
        verify_replay_identity(digest, ContentDigestV1::from_bytes([8; 32])),
        Err(ProcessError::ReplayMismatch)
    );
    assert_eq!(
        validate_replay_end(true, false, false, 1, 1, 2, 2),
        Err(ProcessError::IncompleteEvidence)
    );
    assert_eq!(
        validate_replay_end(true, true, false, 1, 1, 2, 2),
        Err(ProcessError::IncompleteEvidence)
    );
    assert_eq!(validate_replay_end(true, true, false, 2, 2, 2, 2), Ok(()));
}

#[test]
fn captured_replay_requires_complete_explicit_captured_lineage() {
    let span = TimeSpan::new(SessionTime::from_ns(10), SessionTime::from_ns(20)).unwrap();
    let digest = ContentDigestV1::from_bytes([7; 32]);
    let revision = Some(NormalizationRevision::new(1));
    let selections = [
        EvidenceSelection {
            source: SourceId::new(1),
            class: EvidenceClass::Imu,
            span,
            lineage: EvidenceLineageKind::Captured,
            normalization_revision: revision,
            digest,
            usage: EvidenceUse::Fusion,
        },
        EvidenceSelection {
            source: SourceId::new(2),
            class: EvidenceClass::GnssSolution,
            span,
            lineage: EvidenceLineageKind::Captured,
            normalization_revision: revision,
            digest,
            usage: EvidenceUse::Fusion,
        },
        EvidenceSelection {
            source: SourceId::new(3),
            class: EvidenceClass::Timing,
            span,
            lineage: EvidenceLineageKind::Captured,
            normalization_revision: revision,
            digest,
            usage: EvidenceUse::Fusion,
        },
        EvidenceSelection {
            source: SourceId::new(4),
            class: EvidenceClass::Control,
            span,
            lineage: EvidenceLineageKind::Captured,
            normalization_revision: revision,
            digest,
            usage: EvidenceUse::Fusion,
        },
    ];
    let lineage = EvidenceLineage::new(&selections).unwrap();
    assert!(captured_replay_lineage_available(lineage, span));

    let without_control = EvidenceLineage::new(&selections[..3]).unwrap();
    assert!(!captured_replay_lineage_available(without_control, span));

    let mut recomputed = selections;
    recomputed[0].lineage = EvidenceLineageKind::Recomputed;
    assert!(solution_level_lineage_available(
        EvidenceLineage::new(&recomputed).unwrap(),
        span,
        false,
    ));
    assert!(!captured_replay_lineage_available(
        EvidenceLineage::new(&recomputed).unwrap(),
        span,
    ));
}

#[cfg(all(test, feature = "offline"))]
fn captured_replay_lineage_available(
    lineage: crate::provenance::EvidenceLineage<'_>,
    span: TimeSpan,
) -> bool {
    solution_level_lineage_available(lineage, span, true)
}

#[test]
fn fallback_classification_includes_candidate_resource_failures() {
    assert!(runtime_failure_allows_fallback(
        ProcessError::ReplayMismatch
    ));
    assert!(runtime_failure_allows_fallback(
        ProcessError::IncompleteEvidence
    ));
    assert!(!runtime_failure_allows_fallback(
        ProcessError::SourceFailure
    ));
    assert!(!runtime_failure_allows_fallback(ProcessError::SinkFailure));
    assert!(!runtime_failure_allows_fallback(ProcessError::Cancelled));
    assert!(runtime_failure_allows_fallback(ProcessError::ResourceLimit));
}
