//! Present projections, observability, and time-dependent quality evidence.

use super::conversion::{map_core_step_error, project_nav_state};
use super::{GnssQualityEvidence, LiveSession};
use crate::config::EngineConfig;
use crate::engine::{FusionOutcome, LiveProjection};
use crate::error::StepError;
use crate::ids::{ObservationId, SourceId};
use crate::live::{
    DenseEndpoint, DrainReport, EcefAnchor, GnssUpdateOutcome, InitialHeadingSource, LiveCore,
    OrderKey, UpdateDecision,
};
use crate::observation::InputDisposition;
use crate::quality::{
    CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
    HeadingSource, Integrity, ObservabilityReport, Validity,
};
use crate::time::SessionTime;
use nalgebra::{Matrix3, Vector3 as NaVector3};

impl LiveSession<'_, '_> {
    #[inline(never)]
    pub(super) fn present_projection(&mut self) -> Result<Option<LiveProjection>, StepError> {
        if !self.internal.core.is_active() {
            return Ok(None);
        }
        let anchor = self.anchor.ok_or(StepError::WorkspaceContract)?;
        let core = LiveCore::attach(&mut self.internal.core, &mut self.psram.history);
        let present = core.present_state().map_err(map_core_step_error)?;
        let predictor_degraded =
            self.predictor_tracking_degraded || self.predictor_gap || self.predictor_degraded_input;
        Ok(Some(project_nav_state(
            present,
            &anchor,
            self.quality_at(
                EstimateStage::Predicted,
                self.predictor_gap,
                present.time,
                self.last_gnss_evidence,
                predictor_degraded,
            ),
            unavailable_present_observability(self.heading_source),
        )?))
    }

    pub(super) fn quality_at(
        &self,
        stage: EstimateStage,
        imu_gap: bool,
        time: SessionTime,
        evidence: Option<GnssQualityEvidence>,
        additional_degradation: bool,
    ) -> EstimateQuality {
        let fresh_evidence = evidence.filter(|value| self.gnss_evidence_is_fresh_at(*value, time));
        let outage = evidence.is_some() && fresh_evidence.is_none();
        let robustly_downweighted = fresh_evidence.is_some_and(|value| value.downweighted);
        EstimateQuality {
            stage,
            validity: if imu_gap || additional_degradation || outage || robustly_downweighted {
                Validity::Degraded
            } else {
                Validity::Nominal
            },
            gnss: fresh_evidence.map_or(GnssState::Absent, |value| value.state),
            timing: if self.clock_uncertainty_valid {
                fresh_evidence.map_or(self.timing_quality, |value| value.timing)
            } else {
                self.timing_quality
            },
            integrity: if fresh_evidence.is_some() && self.clock_uncertainty_valid {
                Integrity::Monitored
            } else {
                Integrity::Unavailable
            },
            covariance: if stage == EstimateStage::Predicted {
                CovarianceConditioning::Unavailable
            } else {
                CovarianceConditioning::ConditionalOnSelection
            },
            imu_gap,
            degraded_input: stage == EstimateStage::Predicted && self.predictor_degraded_input,
        }
    }

    fn gnss_evidence_is_fresh_at(&self, evidence: GnssQualityEvidence, time: SessionTime) -> bool {
        time.as_ns()
            .checked_sub(evidence.epoch.as_ns())
            .and_then(|age| u64::try_from(age).ok())
            .is_some_and(|age| {
                age <= self
                    .engine
                    .dynamics_profile
                    .gnss
                    .maximum_correction_age
                    .as_ns()
            })
    }

    pub(super) fn gnss_evidence_is_stale_at(&self, time: SessionTime) -> bool {
        self.last_gnss_evidence
            .is_some_and(|evidence| !self.gnss_evidence_is_fresh_at(evidence, time))
    }

    pub(super) fn commit_gnss_evidence(&mut self, evidence: Option<GnssQualityEvidence>) {
        let Some(evidence) = evidence else {
            return;
        };
        self.last_gnss_evidence = Some(evidence);
        self.gnss_state = evidence.state;
        self.integrity = if self.clock_uncertainty_valid {
            Integrity::Monitored
        } else {
            Integrity::Unavailable
        };
    }

    pub(super) fn update_diagnostics(&mut self, report: &DrainReport) {
        self.diagnostics.gnss_updates_fused = self
            .diagnostics
            .gnss_updates_fused
            .saturating_add(u64::from(report.gnss_fused));
        self.diagnostics.gnss_updates_rejected = self
            .diagnostics
            .gnss_updates_rejected
            .saturating_add(u64::from(report.gnss_rejected));
        self.diagnostics.gnss_updates_downweighted = self
            .diagnostics
            .gnss_updates_downweighted
            .saturating_add(u64::from(report.gnss_downweighted));
    }
}

fn unavailable_present_observability(heading_source: HeadingSource) -> ObservabilityReport {
    // Predictor-only output deliberately carries no current covariance. It
    // therefore cannot pass variance/SNR gates using a stale corrected
    // marginal, even when the original heading source remains identified.
    ObservabilityReport {
        heading_source,
        heading: HeadingObservability::Unobservable,
        heading_variance_rad2: None,
        course_available: false,
        body_axis_quantities_available: false,
        angular_acceleration_available: false,
    }
}

pub(super) fn corrected_observability(
    endpoint: DenseEndpoint,
    anchor: &EcefAnchor,
    engine: &EngineConfig<'_>,
    heading_source: HeadingSource,
) -> Result<ObservabilityReport, StepError> {
    let position_ecef = anchor.position_to_ecef(endpoint.state.position_n);
    let instantaneous =
        EcefAnchor::from_origin(0, position_ecef, engine.processing_frame.ellipsoid())
            .map_err(|_| StepError::EstimatorFailure)?;
    let up_ecef_f64 = NaVector3::new(
        instantaneous.ecef_to_n[(2, 0)],
        instantaneous.ecef_to_n[(2, 1)],
        instantaneous.ecef_to_n[(2, 2)],
    );
    let up_ecef = up_ecef_f64.cast::<f32>();

    let ecef_from_n = anchor.ecef_to_n.transpose().cast::<f32>();
    let velocity_ecef = ecef_from_n * endpoint.state.velocity_n;
    let velocity_covariance_ecef =
        ecef_from_n * endpoint.covariance.velocity * ecef_from_n.transpose();
    let horizontal_projector = Matrix3::identity() - up_ecef * up_ecef.transpose();
    let horizontal_velocity = horizontal_projector * velocity_ecef;
    let horizontal_covariance =
        horizontal_projector * velocity_covariance_ecef * horizontal_projector.transpose();
    let horizontal_speed = horizontal_velocity.norm();
    let maximum_horizontal_variance = horizontal_covariance
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .copied()
        .fold(0.0_f32, f32::max)
        .max(0.0);
    let course_snr = if maximum_horizontal_variance == 0.0 {
        if horizontal_speed > 0.0 {
            f32::INFINITY
        } else {
            0.0
        }
    } else {
        horizontal_speed / crate::scalar_math::sqrt(maximum_horizontal_variance)
    };
    let course_variance =
        horizontal_velocity
            .try_normalize(1.0e-8)
            .map_or(f32::INFINITY, |course_direction| {
                let lateral = up_ecef.cross(&course_direction);
                (lateral.transpose() * horizontal_covariance * lateral)[(0, 0)]
                    / (horizontal_speed * horizontal_speed).max(f32::MIN_POSITIVE)
            });
    let course_available = !course_snr.is_nan()
        && f64::from(course_snr) >= engine.dynamics_profile.heading.minimum_course_snr.get()
        && course_variance.is_finite()
        && f64::from(course_variance)
            <= engine
                .dynamics_profile
                .heading
                .maximum_course_variance_rad2
                .get();

    let up_n = (anchor.ecef_to_n * up_ecef_f64).cast::<f32>();
    let rotation_n_from_body = endpoint
        .state
        .orientation_n_from_b
        .to_rotation_matrix()
        .into_inner();
    let local_up_body = rotation_n_from_body.transpose() * up_n;
    let heading_variance =
        (local_up_body.transpose() * endpoint.covariance.attitude * local_up_body)[(0, 0)].max(0.0);
    let heading_variance_rad2 = heading_variance
        .is_finite()
        .then_some(f64::from(heading_variance));
    let heading_available = heading_source != HeadingSource::None
        && heading_variance_rad2.is_some_and(|variance| {
            variance
                <= engine
                    .dynamics_profile
                    .heading
                    .maximum_yaw_variance_rad2
                    .get()
        });
    let heading = if heading_available {
        match heading_source {
            HeadingSource::Supplied => HeadingObservability::Supplied,
            HeadingSource::Gyrocompass => HeadingObservability::Gyrocompassed,
            HeadingSource::DynamicAlignment => HeadingObservability::DynamicallyAligned,
            HeadingSource::NonHolonomicConstraint => HeadingObservability::Constrained,
            HeadingSource::None => HeadingObservability::Unobservable,
        }
    } else {
        HeadingObservability::Unobservable
    };
    Ok(ObservabilityReport {
        heading_source,
        heading,
        heading_variance_rad2,
        course_available,
        body_axis_quantities_available: heading_available,
        angular_acceleration_available: false,
    })
}

pub(super) fn fusion_outcome(key: OrderKey, outcome: GnssUpdateOutcome) -> FusionOutcome {
    let decision = outcome.joint.or(outcome.position).or(outcome.velocity);
    let (disposition, nis) = match decision {
        Some(UpdateDecision::Fused { nis }) => (InputDisposition::Fused, Some(nis)),
        Some(UpdateDecision::Downweighted { nis, .. }) => {
            (InputDisposition::Downweighted, Some(nis))
        }
        Some(UpdateDecision::RejectedInnovation { nis }) => {
            (InputDisposition::StatisticallyRejected, Some(nis))
        }
        Some(UpdateDecision::RejectedHealth)
        | Some(UpdateDecision::RejectedInsufficientKinematics)
        | None => (InputDisposition::StatisticallyRejected, None),
    };
    FusionOutcome {
        observation: ObservationId::new(SourceId::new(key.source), key.sequence),
        disposition,
        normalized_innovation_squared: nis,
    }
}

pub(super) fn heading_source(source: InitialHeadingSource) -> HeadingSource {
    match source {
        InitialHeadingSource::Supplied => HeadingSource::Supplied,
        InitialHeadingSource::Gyrocompass => HeadingSource::Gyrocompass,
        InitialHeadingSource::DynamicConstraint => HeadingSource::DynamicAlignment,
        InitialHeadingSource::None => HeadingSource::None,
    }
}
