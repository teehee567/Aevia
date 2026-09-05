//! Canonical semantic identities for captured live calls and summaries.

use super::{LiveSummary, LiveUpdate};
use crate::error::ProcessError;
use crate::ids::{ContentDigestV1, ObservationId};
use crate::metric::{MetricMutation, WithdrawalReason};
use crate::observation::InputDisposition;
use crate::quality::{DiagnosticCounts, EstimateQuality, ObservabilityReport};
use crate::time::{SessionTime, TimeSpan};
use sha2::{Digest, Sha256};

/// Canonical same-build identity for every semantic field returned by one
/// live call. Floats are encoded by canonical IEEE bits, metric values reuse
/// the artifact semantic codec, and no pointer or native Rust layout is read.
///
/// # Errors
///
/// Returns [`ProcessError::InvalidEvidence`] if an engine-produced metric
/// mutation cannot be represented by the fixed canonical v1 codec.
#[must_use]
pub fn captured_update_digest_v1(update: &LiveUpdate<'_>) -> Result<ContentDigestV1, ProcessError> {
    let mut hash = Sha256::new();
    hash.update(b"aevia.captured-live-update.v1\0");
    hash_optional_observation_outcome(&mut hash, update.input);
    hash_bool(&mut hash, update.fusion.is_some());
    if let Some(fusion) = update.fusion {
        hash_observation_id(&mut hash, fusion.observation);
        hash_u8(&mut hash, fusion.disposition as u8);
        hash_bool(&mut hash, fusion.normalized_innovation_squared.is_some());
        if let Some(value) = fusion.normalized_innovation_squared {
            hash_f32(&mut hash, value);
        }
    }
    hash_optional_span(&mut hash, update.corrected_interval);
    hash_optional_u32(&mut hash, update.reanchor_generation);
    hash_optional_time(&mut hash, update.navigation_watermark);
    hash_optional_time(&mut hash, update.metric_watermark);
    hash_bool(&mut hash, update.present.is_some());
    if let Some(present) = update.present {
        hash_i64(&mut hash, present.time.as_ns());
        for value in present.position.components() {
            hash_f64(&mut hash, value);
        }
        for value in present.velocity.components() {
            hash_f64(&mut hash, value);
        }
        for value in present
            .orientation_ecef_from_body
            .quaternion()
            .components_wxyz()
        {
            hash_f64(&mut hash, value);
        }
        hash_quality(&mut hash, present.quality);
        hash_observability(&mut hash, present.observability);
    }
    hash_u32(
        &mut hash,
        u32::try_from(update.mutations.len()).unwrap_or(u32::MAX),
    );
    for mutation in update.mutations {
        match *mutation {
            MetricMutation::Upsert {
                id,
                revision,
                value,
            } => {
                hash_u8(&mut hash, 0);
                hash_u64(&mut hash, id.run_namespace());
                hash_u64(&mut hash, id.allocation());
                hash_u64(&mut hash, revision);
                let mut bytes = [0_u8; 512];
                let length = crate::metric::encode_metric_value_identity_v1(value, &mut bytes)
                    .map_err(|_| ProcessError::InvalidEvidence)?;
                hash_u32(&mut hash, u32::try_from(length).unwrap_or(u32::MAX));
                hash.update(&bytes[..length]);
            }
            MetricMutation::Withdraw {
                id,
                revision,
                reason,
            } => {
                hash_u8(&mut hash, 1);
                hash_u64(&mut hash, id.run_namespace());
                hash_u64(&mut hash, id.allocation());
                hash_u64(&mut hash, revision);
                hash_u8(&mut hash, withdrawal_reason_tag(reason));
            }
            MetricMutation::Finalize { id, revision } => {
                hash_u8(&mut hash, 2);
                hash_u64(&mut hash, id.run_namespace());
                hash_u64(&mut hash, id.allocation());
                hash_u64(&mut hash, revision);
            }
        }
    }
    hash_diagnostics(&mut hash, update.diagnostics);
    hash_u8(&mut hash, update.phase as u8);
    hash_u32(&mut hash, update.work_remaining);
    Ok(ContentDigestV1::from_bytes(hash.finalize().into()))
}

/// Canonical identity of the final bounded live summary.
#[must_use]
pub fn captured_summary_digest_v1(summary: LiveSummary) -> ContentDigestV1 {
    let mut hash = Sha256::new();
    hash.update(b"aevia.captured-live-summary.v1\0");
    hash_optional_time(&mut hash, summary.terminal_time);
    hash_optional_span(&mut hash, summary.retained_trajectory_span);
    hash_diagnostics(&mut hash, summary.diagnostics);
    hash_u16(&mut hash, summary.finalized_metric_results);
    ContentDigestV1::from_bytes(hash.finalize().into())
}

fn hash_optional_observation_outcome(
    hash: &mut Sha256,
    value: Option<(ObservationId, InputDisposition)>,
) {
    hash_bool(hash, value.is_some());
    if let Some((id, disposition)) = value {
        hash_observation_id(hash, id);
        hash_u8(hash, disposition as u8);
    }
}

fn hash_observation_id(hash: &mut Sha256, id: ObservationId) {
    hash_u32(hash, id.source.get());
    hash_u64(hash, id.sequence);
}

fn hash_optional_time(hash: &mut Sha256, value: Option<SessionTime>) {
    hash_bool(hash, value.is_some());
    if let Some(value) = value {
        hash_i64(hash, value.as_ns());
    }
}

fn hash_optional_span(hash: &mut Sha256, value: Option<TimeSpan>) {
    hash_bool(hash, value.is_some());
    if let Some(value) = value {
        hash_i64(hash, value.start().as_ns());
        hash_i64(hash, value.end().as_ns());
    }
}

fn hash_optional_u32(hash: &mut Sha256, value: Option<u32>) {
    hash_bool(hash, value.is_some());
    if let Some(value) = value {
        hash_u32(hash, value);
    }
}

fn hash_quality(hash: &mut Sha256, value: EstimateQuality) {
    hash_u8(hash, value.stage as u8);
    hash_u8(hash, value.validity as u8);
    hash_u8(hash, value.gnss as u8);
    hash_u8(hash, value.timing as u8);
    hash_u8(hash, value.integrity as u8);
    hash_u8(hash, value.covariance as u8);
    hash_bool(hash, value.imu_gap);
    hash_bool(hash, value.degraded_input);
}

fn hash_observability(hash: &mut Sha256, value: ObservabilityReport) {
    hash_u8(hash, value.heading_source as u8);
    hash_u8(hash, value.heading as u8);
    hash_bool(hash, value.heading_variance_rad2.is_some());
    if let Some(variance) = value.heading_variance_rad2 {
        hash_f64(hash, variance);
    }
    hash_bool(hash, value.course_available);
    hash_bool(hash, value.body_axis_quantities_available);
    hash_bool(hash, value.angular_acceleration_available);
}

fn hash_diagnostics(hash: &mut Sha256, value: DiagnosticCounts) {
    hash_u64(hash, value.imu_epochs_accepted);
    hash_u64(hash, value.imu_epochs_rejected);
    hash_u64(hash, value.gnss_updates_fused);
    hash_u64(hash, value.gnss_updates_rejected);
    hash_u64(hash, value.gnss_updates_downweighted);
    hash_u64(hash, value.observations_too_late);
    hash_u32(hash, value.clock_discontinuities);
    hash_u32(hash, value.reinitializations);
    hash_u32(hash, value.covariance_repairs);
    hash_u32(hash, value.metric_ambiguities);
    hash_u32(hash, value.output_overflows);
}

const fn withdrawal_reason_tag(value: WithdrawalReason) -> u8 {
    match value {
        WithdrawalReason::RetrospectiveRuleChanged => 0,
        WithdrawalReason::TrajectoryReinitialized => 1,
        WithdrawalReason::QualityInvalidated => 2,
        WithdrawalReason::OutputSuperseded => 3,
    }
}

fn hash_bool(hash: &mut Sha256, value: bool) {
    hash_u8(hash, u8::from(value));
}

fn hash_u8(hash: &mut Sha256, value: u8) {
    hash.update([value]);
}

fn hash_u16(hash: &mut Sha256, value: u16) {
    hash.update(value.to_le_bytes());
}

fn hash_u32(hash: &mut Sha256, value: u32) {
    hash.update(value.to_le_bytes());
}

fn hash_u64(hash: &mut Sha256, value: u64) {
    hash.update(value.to_le_bytes());
}

fn hash_i64(hash: &mut Sha256, value: i64) {
    hash.update(value.to_le_bytes());
}

fn hash_f32(hash: &mut Sha256, value: f32) {
    hash_u32(hash, if value == 0.0 { 0 } else { value.to_bits() });
}

fn hash_f64(hash: &mut Sha256, value: f64) {
    hash_u64(hash, if value == 0.0 { 0 } else { value.to_bits() });
}
