//! Conservative uncertainty, quality, and observability combination.

use crate::error::ValidationError;
use crate::quality::{
    CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
    HeadingSource, Integrity, ObservabilityReport, TimingQuality, Validity,
};
use crate::uncertainty::{Covariance3, KinematicCovariance};

pub(super) fn conservative_covariance(
    first: KinematicCovariance,
    second: KinematicCovariance,
) -> Result<KinematicCovariance, ValidationError> {
    let position = diagonal_envelope(first.position(), second.position())?;
    let velocity = diagonal_envelope(first.velocity(), second.velocity())?;
    let attitude = diagonal_envelope(first.attitude_error(), second.attitude_error())?;
    let angular_rate = optional_envelope(first.angular_rate(), second.angular_rate())?;
    let angular_acceleration =
        optional_envelope(first.angular_acceleration(), second.angular_acceleration())?;
    let kinematic_acceleration = optional_envelope(
        first.kinematic_acceleration(),
        second.kinematic_acceleration(),
    )?;
    let specific_force = optional_envelope(first.specific_force(), second.specific_force())?;
    Ok(
        KinematicCovariance::new(position, velocity, None, attitude)?.with_dynamic_covariances(
            angular_rate,
            angular_acceleration,
            kinematic_acceleration,
            specific_force,
        ),
    )
}

pub(super) fn diagonal_envelope(
    first: Covariance3,
    second: Covariance3,
) -> Result<Covariance3, ValidationError> {
    Covariance3::diagonal(
        first
            .variance(0)
            .unwrap_or(0.0)
            .max(second.variance(0).unwrap_or(0.0)),
        first
            .variance(1)
            .unwrap_or(0.0)
            .max(second.variance(1).unwrap_or(0.0)),
        first
            .variance(2)
            .unwrap_or(0.0)
            .max(second.variance(2).unwrap_or(0.0)),
    )
}

pub(super) fn optional_envelope(
    first: Option<Covariance3>,
    second: Option<Covariance3>,
) -> Result<Option<Covariance3>, ValidationError> {
    match (first, second) {
        (Some(left), Some(right)) => diagonal_envelope(left, right).map(Some),
        _ => Ok(None),
    }
}

pub(super) fn conservative_quality(
    first: EstimateQuality,
    second: EstimateQuality,
) -> EstimateQuality {
    EstimateQuality {
        stage: worse_stage(first.stage, second.stage),
        validity: worse_validity(first.validity, second.validity),
        gnss: if first.gnss == second.gnss {
            first.gnss
        } else {
            GnssState::Suspect
        },
        timing: worse_timing(first.timing, second.timing),
        integrity: if first.integrity == Integrity::Monitored
            && second.integrity == Integrity::Monitored
        {
            Integrity::Monitored
        } else {
            Integrity::Unavailable
        },
        covariance: worse_covariance(first.covariance, second.covariance),
        imu_gap: first.imu_gap || second.imu_gap,
        degraded_input: first.degraded_input || second.degraded_input,
    }
}

pub(super) fn conservative_observability(
    first: ObservabilityReport,
    second: ObservabilityReport,
) -> ObservabilityReport {
    let heading_matches =
        first.heading == second.heading && first.heading_source == second.heading_source;
    ObservabilityReport {
        heading_source: if heading_matches {
            first.heading_source
        } else {
            HeadingSource::None
        },
        heading: if heading_matches {
            first.heading
        } else {
            HeadingObservability::Unobservable
        },
        heading_variance_rad2: match (first.heading_variance_rad2, second.heading_variance_rad2) {
            (Some(left), Some(right)) => Some(left.max(right)),
            _ => None,
        },
        course_available: first.course_available && second.course_available,
        body_axis_quantities_available: first.body_axis_quantities_available
            && second.body_axis_quantities_available,
        angular_acceleration_available: first.angular_acceleration_available
            && second.angular_acceleration_available,
    }
}

pub(super) const fn worse_stage(first: EstimateStage, second: EstimateStage) -> EstimateStage {
    match (first, second) {
        (EstimateStage::Predicted, _) | (_, EstimateStage::Predicted) => EstimateStage::Predicted,
        (EstimateStage::Provisional, _) | (_, EstimateStage::Provisional) => {
            EstimateStage::Provisional
        }
        _ => EstimateStage::Finalized,
    }
}

pub(super) const fn worse_validity(first: Validity, second: Validity) -> Validity {
    match (first, second) {
        (Validity::Invalid, _) | (_, Validity::Invalid) => Validity::Invalid,
        (Validity::Degraded, _) | (_, Validity::Degraded) => Validity::Degraded,
        _ => Validity::Nominal,
    }
}

pub(super) const fn worse_timing(first: TimingQuality, second: TimingQuality) -> TimingQuality {
    use TimingQuality::{ArrivalOnly, Discontinuous, Modeled, PpsCorrelated};
    match (first, second) {
        (Discontinuous, _) | (_, Discontinuous) => Discontinuous,
        (ArrivalOnly, _) | (_, ArrivalOnly) => ArrivalOnly,
        (Modeled, _) | (_, Modeled) => Modeled,
        _ => PpsCorrelated,
    }
}

pub(super) const fn worse_covariance(
    first: CovarianceConditioning,
    second: CovarianceConditioning,
) -> CovarianceConditioning {
    use CovarianceConditioning::{ConditionalOnSelection, Unavailable, UnconditionalModel};
    match (first, second) {
        (Unavailable, _) | (_, Unavailable) => Unavailable,
        (ConditionalOnSelection, _) | (_, ConditionalOnSelection) => ConditionalOnSelection,
        _ => UnconditionalModel,
    }
}
