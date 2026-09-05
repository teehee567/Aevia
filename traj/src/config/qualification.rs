//! Measured qualification specifications, reports, and validation.

use super::{LiveRootEnclosureQualificationV1, NumericProfileSpec};
use crate::error::ValidationError;
use crate::ids::{ContentDigestV1, QualificationSpecId};
use crate::math::{NonNegativeF64, Probability};
use crate::time::DurationNs;

/// Frozen numeric pass/fail contract completed before acceptance data is read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QualificationSpecV1 {
    /// Stable qualification identity.
    pub id: QualificationSpecId,
    /// Minimum independent sessions.
    pub minimum_session_count: u16,
    /// Minimum total independent truth duration.
    pub minimum_total_duration: DurationNs,
    /// Maximum position RMSE in metres.
    pub maximum_position_rmse_m: NonNegativeF64,
    /// Maximum velocity RMSE in metres per second.
    pub maximum_velocity_rmse_mps: NonNegativeF64,
    /// Maximum attitude RMSE in radians.
    pub maximum_attitude_rmse_rad: NonNegativeF64,
    /// Maximum event-time absolute error in seconds.
    pub maximum_event_time_error_s: NonNegativeF64,
    /// Maximum hard-failure probability.
    pub maximum_hard_failure_rate: Probability,
    /// Minimum empirical covariance coverage probability.
    pub minimum_empirical_coverage: Probability,
    /// Maximum absolute lag-one innovation autocorrelation.
    pub maximum_innovation_autocorrelation: Probability,
    /// Maximum cross-target continuous numeric discrepancy.
    pub maximum_cross_target_numeric_error: NonNegativeF64,
    /// Maximum root-time residual in seconds.
    pub maximum_root_time_residual_s: NonNegativeF64,
    /// Maximum quadrature absolute error.
    pub maximum_quadrature_error: NonNegativeF64,
    /// Maximum re-anchor ECEF-state discrepancy in metres.
    pub maximum_reanchor_error_m: NonNegativeF64,
    /// Minimum decoder/property/fuzz case count.
    pub minimum_fuzz_cases: u64,
    /// Minimum numerical-Jacobian comparisons.
    pub minimum_jacobian_cases: u32,
    /// Minimum Monte Carlo trials.
    pub minimum_monte_carlo_trials: u32,
    /// Minimum adversarial metric-root cases.
    pub minimum_adversarial_root_cases: u32,
    /// Canonical complete qualification-spec digest.
    pub digest: ContentDigestV1,
}

impl QualificationSpecV1 {
    /// Rejects qualitative/blank numeric gates.
    pub const fn validate(self) -> Result<Self, ValidationError> {
        if self.id.get() == 0
            || self.digest.is_zero()
            || self.minimum_session_count == 0
            || self.minimum_total_duration.as_ns() == 0
            || self.maximum_position_rmse_m.get() == 0.0
            || self.maximum_velocity_rmse_mps.get() == 0.0
            || self.maximum_attitude_rmse_rad.get() == 0.0
            || self.maximum_event_time_error_s.get() == 0.0
            || self.minimum_empirical_coverage.get() == 0.0
            || self.maximum_cross_target_numeric_error.get() == 0.0
            || self.maximum_root_time_residual_s.get() == 0.0
            || self.maximum_quadrature_error.get() == 0.0
            || self.maximum_reanchor_error_m.get() == 0.0
            || self.minimum_fuzz_cases == 0
            || self.minimum_jacobian_cases == 0
            || self.minimum_monte_carlo_trials == 0
            || self.minimum_adversarial_root_cases == 0
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Immutable measured outcome of one frozen qualification campaign.
///
/// A specification by itself is only a set of thresholds. Production
/// selection additionally requires this digest-bound report, which ties the
/// measured corpus and target/toolchain to the exact engine configuration and
/// records every scalar used by the pass/fail decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QualificationReportV1 {
    pub specification_id: QualificationSpecId,
    pub specification_digest: ContentDigestV1,
    pub configuration_digest: ContentDigestV1,
    pub corpus_digest: ContentDigestV1,
    pub target_digest: ContentDigestV1,
    pub report_digest: ContentDigestV1,
    pub session_count: u16,
    pub total_duration: DurationNs,
    pub position_rmse_m: NonNegativeF64,
    pub velocity_rmse_mps: NonNegativeF64,
    pub attitude_rmse_rad: NonNegativeF64,
    pub event_time_error_s: NonNegativeF64,
    pub hard_failure_rate: Probability,
    pub empirical_coverage: Probability,
    pub innovation_autocorrelation: Probability,
    pub cross_target_numeric_error: NonNegativeF64,
    pub root_time_residual_s: NonNegativeF64,
    pub quadrature_error: NonNegativeF64,
    pub reanchor_error_m: NonNegativeF64,
    pub fuzz_cases: u64,
    pub jacobian_cases: u32,
    pub monte_carlo_trials: u32,
    pub adversarial_root_cases: u32,
    /// Optional measured attestation for the exact live non-polynomial root
    /// backend. `None` is the normal development/default state and fails
    /// closed only for metric definitions that need that backend.
    pub live_root_enclosure: Option<LiveRootEnclosureQualificationV1>,
}

impl QualificationReportV1 {
    fn validate_against(
        self,
        specification: QualificationSpecV1,
        configuration_digest: ContentDigestV1,
    ) -> Result<Self, ValidationError> {
        specification.validate()?;
        if self.specification_id != specification.id
            || self.specification_digest != specification.digest
            || self.configuration_digest != configuration_digest
            || self.corpus_digest.is_zero()
            || self.target_digest.is_zero()
            || self.report_digest.is_zero()
            || self.session_count < specification.minimum_session_count
            || self.total_duration < specification.minimum_total_duration
            || self.position_rmse_m.get() > specification.maximum_position_rmse_m.get()
            || self.velocity_rmse_mps.get() > specification.maximum_velocity_rmse_mps.get()
            || self.attitude_rmse_rad.get() > specification.maximum_attitude_rmse_rad.get()
            || self.event_time_error_s.get() > specification.maximum_event_time_error_s.get()
            || self.hard_failure_rate.get() > specification.maximum_hard_failure_rate.get()
            || self.empirical_coverage.get() < specification.minimum_empirical_coverage.get()
            || self.innovation_autocorrelation.get()
                > specification.maximum_innovation_autocorrelation.get()
            || self.cross_target_numeric_error.get()
                > specification.maximum_cross_target_numeric_error.get()
            || self.root_time_residual_s.get() > specification.maximum_root_time_residual_s.get()
            || self.quadrature_error.get() > specification.maximum_quadrature_error.get()
            || self.reanchor_error_m.get() > specification.maximum_reanchor_error_m.get()
            || self.fuzz_cases < specification.minimum_fuzz_cases
            || self.jacobian_cases < specification.minimum_jacobian_cases
            || self.monte_carlo_trials < specification.minimum_monte_carlo_trials
            || self.adversarial_root_cases < specification.minimum_adversarial_root_cases
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Whether a processing/profile combination has passed its frozen contract.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QualificationStatus<'a> {
    /// Profile exists for development but may not be selected for production.
    Unqualified,
    /// The exact configuration passed both the frozen specification and its
    /// attached immutable measured report.
    Qualified {
        specification: &'a QualificationSpecV1,
        report: &'a QualificationReportV1,
    },
}

impl QualificationStatus<'_> {
    /// Returns whether production selection is allowed after validation.
    #[must_use]
    pub const fn is_qualified(self) -> bool {
        matches!(self, Self::Qualified { .. })
    }

    /// Validates the supplied threshold contract and binds its measured report
    /// to the exact engine configuration being selected.
    pub fn validate_for_configuration(
        self,
        configuration_digest: ContentDigestV1,
    ) -> Result<Self, ValidationError> {
        match self {
            Self::Unqualified => Ok(self),
            Self::Qualified {
                specification,
                report,
            } => {
                report.validate_against(*specification, configuration_digest)?;
                Ok(self)
            }
        }
    }

    pub(super) fn validate_numeric_attestations(
        self,
        numeric_profile: NumericProfileSpec,
    ) -> Result<Self, ValidationError> {
        if let Self::Qualified {
            report,
            specification: _,
        } = self
        {
            if let Some(attestation) = report.live_root_enclosure {
                attestation.validate_against(
                    numeric_profile,
                    report.target_digest,
                    report.adversarial_root_cases,
                )?;
            }
        }
        Ok(self)
    }

    pub(super) fn live_root_enclosure(
        self,
        numeric_profile: NumericProfileSpec,
    ) -> Option<LiveRootEnclosureQualificationV1> {
        let Self::Qualified {
            report,
            specification: _,
        } = self
        else {
            return None;
        };
        report.live_root_enclosure.filter(|attestation| {
            attestation
                .validate_against(
                    numeric_profile,
                    report.target_digest,
                    report.adversarial_root_cases,
                )
                .is_ok()
        })
    }
}
