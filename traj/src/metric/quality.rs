//! Conservative validity across retained trajectory spans.

use super::report::MetricError;
use crate::{
    quality::{EstimateQuality, Validity},
    time::SessionTime,
    trajectory::Trajectory,
};

pub(super) const fn worse_metric_validity(first: Validity, second: Validity) -> Validity {
    match (first, second) {
        (Validity::Invalid, _) | (_, Validity::Invalid) => Validity::Invalid,
        (Validity::Degraded, _) | (_, Validity::Degraded) => Validity::Degraded,
        (Validity::Nominal, Validity::Nominal) => Validity::Nominal,
    }
}

pub(super) const fn metric_validity(quality: EstimateQuality) -> Validity {
    if quality.imu_gap && matches!(quality.validity, Validity::Nominal) {
        Validity::Degraded
    } else {
        quality.validity
    }
}

pub(super) fn retained_span_quality(
    trajectory: &Trajectory,
    start: SessionTime,
    end: SessionTime,
) -> Result<EstimateQuality, MetricError> {
    let span = trajectory.span().ok_or(MetricError::EmptyTrajectory)?;
    trajectory.conservative_quality_over_span(start.max(span.start()), end.min(span.end()))
}
