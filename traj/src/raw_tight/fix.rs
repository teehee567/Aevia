//! Assess integer hypotheses against frozen conditional-fix acceptance gates.

use super::{
    ConditionalAmbiguityFix, ConditionalFixKind, RawTightBackendRegistration,
    RawTightRegistrationError, digest_is_zero,
};
use crate::ids::ContentDigestV1;

/// Frozen integer-acceptance gates. No threshold is silently supplied by the
/// adapter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct AmbiguityFixThresholds {
    pub minimum_ratio: f64,
    pub minimum_success_rate: f64,
    pub maximum_residual_rms: f64,
    pub minimum_temporal_consistency_epochs: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct AmbiguityFixEvidence {
    pub ratio: f64,
    pub success_rate: f64,
    pub residual_rms: f64,
    pub temporally_consistent_epochs: u16,
    pub fixed_ambiguity_count: u16,
    pub available_ambiguity_count: u16,
    pub slip_free: bool,
    pub integrity_valid: bool,
    pub hypothesis_digest: ContentDigestV1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ConditionalUncertaintyBasis {
    SelectedRobustWeightsAndIntegerHypothesis,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FixRejectionReason {
    SlipOrLockDiscontinuity,
    IntegrityInvalid,
    NoAmbiguities,
    RatioBelowThreshold,
    SuccessRateBelowThreshold,
    ResidualAboveThreshold,
    TemporalConsistencyInsufficient,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AmbiguityFixDecision {
    Float(FixRejectionReason),
    Conditional(ConditionalAmbiguityFix),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FixAssessmentError {
    BackendNotRegistered,
    InvalidRegistration(RawTightRegistrationError),
    InvalidThresholds,
    InvalidEvidence,
    MissingHypothesisDigest,
}

pub(crate) fn assess_ambiguity_fix(
    registration: Option<&RawTightBackendRegistration>,
    thresholds: AmbiguityFixThresholds,
    evidence: AmbiguityFixEvidence,
) -> Result<AmbiguityFixDecision, FixAssessmentError> {
    let Some(registration) = registration.copied() else {
        return Err(FixAssessmentError::BackendNotRegistered);
    };
    registration
        .validate()
        .map_err(FixAssessmentError::InvalidRegistration)?;
    if !thresholds.minimum_ratio.is_finite()
        || thresholds.minimum_ratio <= 1.0
        || !thresholds.minimum_success_rate.is_finite()
        || !(0.0..=1.0).contains(&thresholds.minimum_success_rate)
        || thresholds.minimum_success_rate == 0.0
        || !thresholds.maximum_residual_rms.is_finite()
        || thresholds.maximum_residual_rms <= 0.0
        || thresholds.minimum_temporal_consistency_epochs == 0
    {
        return Err(FixAssessmentError::InvalidThresholds);
    }
    if !evidence.ratio.is_finite()
        || evidence.ratio < 0.0
        || !evidence.success_rate.is_finite()
        || !(0.0..=1.0).contains(&evidence.success_rate)
        || !evidence.residual_rms.is_finite()
        || evidence.residual_rms < 0.0
        || evidence.fixed_ambiguity_count > evidence.available_ambiguity_count
    {
        return Err(FixAssessmentError::InvalidEvidence);
    }
    let rejection = if !evidence.slip_free {
        Some(FixRejectionReason::SlipOrLockDiscontinuity)
    } else if !evidence.integrity_valid {
        Some(FixRejectionReason::IntegrityInvalid)
    } else if evidence.fixed_ambiguity_count == 0 || evidence.available_ambiguity_count == 0 {
        Some(FixRejectionReason::NoAmbiguities)
    } else if evidence.ratio < thresholds.minimum_ratio {
        Some(FixRejectionReason::RatioBelowThreshold)
    } else if evidence.success_rate < thresholds.minimum_success_rate {
        Some(FixRejectionReason::SuccessRateBelowThreshold)
    } else if evidence.residual_rms > thresholds.maximum_residual_rms {
        Some(FixRejectionReason::ResidualAboveThreshold)
    } else if evidence.temporally_consistent_epochs < thresholds.minimum_temporal_consistency_epochs
    {
        Some(FixRejectionReason::TemporalConsistencyInsufficient)
    } else {
        None
    };
    if let Some(reason) = rejection {
        return Ok(AmbiguityFixDecision::Float(reason));
    }
    if digest_is_zero(evidence.hypothesis_digest) {
        return Err(FixAssessmentError::MissingHypothesisDigest);
    }
    let kind = if evidence.fixed_ambiguity_count == evidence.available_ambiguity_count {
        ConditionalFixKind::Full
    } else {
        ConditionalFixKind::Partial
    };
    Ok(AmbiguityFixDecision::Conditional(ConditionalAmbiguityFix {
        kind,
        fixed_ambiguity_count: evidence.fixed_ambiguity_count,
        available_ambiguity_count: evidence.available_ambiguity_count,
        hypothesis_digest: evidence.hypothesis_digest,
        uncertainty_basis: ConditionalUncertaintyBasis::SelectedRobustWeightsAndIntegerHypothesis,
    }))
}
