//! Shared fixtures for raw-tight responsibility-specific regression suites.

use super::*;
use crate::{
    ids::NormalizationRevision, observation::Constellation, provenance::EvidenceSelection,
    time::SessionTime,
};

fn digest(value: u8) -> ContentDigestV1 {
    ContentDigestV1::from_bytes([value; 32])
}

fn span(start: i64, end: i64) -> TimeSpan {
    TimeSpan::new(SessionTime::from_ns(start), SessionTime::from_ns(end)).unwrap()
}

fn selection(
    source: u32,
    class: EvidenceClass,
    lineage: EvidenceLineageKind,
    usage: EvidenceUse,
) -> EvidenceSelection {
    EvidenceSelection {
        source: SourceId::new(source),
        class,
        span: span(0, 100),
        lineage,
        normalization_revision: match lineage {
            EvidenceLineageKind::Captured | EvidenceLineageKind::Recomputed => {
                Some(NormalizationRevision::new(1))
            }
            EvidenceLineageKind::Raw | EvidenceLineageKind::External => None,
        },
        digest: digest(u8::try_from(source).unwrap()),
        usage,
    }
}

fn required_selections() -> [EvidenceSelection; 6] {
    [
        selection(
            1,
            EvidenceClass::Imu,
            EvidenceLineageKind::Captured,
            EvidenceUse::Fusion,
        ),
        selection(
            2,
            EvidenceClass::RawGnss,
            EvidenceLineageKind::Raw,
            EvidenceUse::Fusion,
        ),
        selection(
            3,
            EvidenceClass::BaseOrCorrection,
            EvidenceLineageKind::External,
            EvidenceUse::Fusion,
        ),
        selection(
            4,
            EvidenceClass::Ephemeris,
            EvidenceLineageKind::Raw,
            EvidenceUse::Fusion,
        ),
        selection(
            5,
            EvidenceClass::Timing,
            EvidenceLineageKind::Captured,
            EvidenceUse::Fusion,
        ),
        selection(
            6,
            EvidenceClass::Control,
            EvidenceLineageKind::Captured,
            EvidenceUse::Fusion,
        ),
    ]
}

fn capabilities() -> Capabilities {
    let mut result = Capabilities::NONE;
    for capability in [
        Capability::NormalizedImu,
        Capability::RawRoverGnss,
        Capability::BaseOrCorrection,
        Capability::Ephemerides,
        Capability::Timing,
        Capability::Configuration,
        Capability::CompleteEnd,
    ] {
        result = result.with(capability);
    }
    result
}

fn shape() -> RawTightProblemShape {
    RawTightProblemShape {
        imu_samples: 100_000,
        rover_epochs: 1_000,
        base_or_correction_epochs: 1_000,
        raw_signal_samples: 20_000,
        pseudorange_samples: 18_000,
        ambiguity_carrier_phase_samples: 15_000,
        tdcp_samples: 0,
        doppler_samples: 17_000,
        ephemeris_records: 50,
        proposed_keyframes: 2_000,
        receiver_clock_nodes: 2_000,
        troposphere_nodes: 100,
        inter_system_bias_coordinates: 4,
        maximum_simultaneous_ambiguity_arcs: 80,
        requested_output_epochs: 10_000,
    }
}

fn request(lineage: EvidenceLineage<'_>) -> RawTightPreflightRequest<'_> {
    RawTightPreflightRequest {
        span: span(0, 100),
        capabilities: capabilities(),
        evidence_lineage: lineage,
        evidence_restartable: true,
        explicitly_required: true,
        profile_qualified: true,
        evidence_checks: RawTightEvidenceChecks::ALL,
        shape: shape(),
    }
}

fn limits() -> OfflineResourceLimits {
    OfflineResourceLimits {
        peak_memory_bytes: u64::MAX,
        temporary_storage_bytes: u64::MAX,
        output_bytes: u64::MAX,
        worker_count: 8,
        elapsed_work_limit: None,
    }
}

fn registration() -> RawTightBackendRegistration {
    RawTightBackendRegistration {
        backend_version: BackendVersionId::new(3),
        identity: RawTightImplementationIdentity {
            adapter_abi_revision: RAW_TIGHT_ADAPTER_ABI_REVISION,
            algorithm_revision: 1,
            implementation_digest: digest(20),
            native_build_digest: digest(21),
            graph_backend: RawGraphBackendKind::IndependentNative,
            gtsam_source_digest: None,
        },
        qualification: RawTightQualificationReceipt {
            specification: QualificationSpecId::new(2),
            specification_digest: digest(22),
            report_digest: digest(23),
            cycle_slip_cases: 10_000,
            reference_switch_cases: 2_000,
            rejected_incorrect_fix_cases: 1_000,
            covariance_coverage_sessions: 100,
            passed: true,
        },
        safety: RawTightSafetyClaims::ALL,
        maximum_signal_samples: 1_000_000,
        maximum_simultaneous_ambiguity_arcs: 1_000,
        maximum_integer_candidates: 1_024,
    }
}

fn satellite(vehicle: u16) -> SatelliteId {
    SatelliteId {
        constellation: Constellation::Gps,
        vehicle,
    }
}

use super::preflight::preflight_with_registration;

mod preflight;

mod ambiguity;

mod phase_ledger;

mod fix;

mod registration;

use crate::config::OfflineResourceLimits;
use crate::ids::BackendVersionId;
use crate::ids::ContentDigestV1;
use crate::ids::ObservationId;
use crate::ids::QualificationSpecId;
use crate::ids::SourceId;
use crate::observation::SatelliteId;
use crate::provenance::Capabilities;
use crate::provenance::Capability;
use crate::provenance::EvidenceClass;
use crate::provenance::EvidenceLineage;
use crate::provenance::EvidenceLineageKind;
use crate::provenance::EvidenceUse;
use crate::provenance::ProcessingAttemptOutcome;
use crate::time::TimeSpan;
