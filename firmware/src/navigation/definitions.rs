//! Device metrics remain unavailable without a qualified numerical profile.
use aevia_trajectory::{ValidationError, metric::MetricPlan};

/// The bench image runs navigation with an empty metric plan. Do not publish
/// drag results by manufacturing the required root-enclosure attestation.
pub fn populate(plan: &mut MetricPlan) -> Result<(), ValidationError> {
    if plan.definitions().is_empty() {
        Ok(())
    } else {
        Err(ValidationError::InvalidMetricDefinition)
    }
}

// Keep the large MetricDefinition temporary out of the caller's preflight
// frame. The complete MetricPlan itself lives in the caller's PSRAM region.
#[inline(never)]
#[cfg(test)]
pub fn populate_intended_drag(plan: &mut MetricPlan) -> Result<(), ValidationError> {
    use aevia_trajectory::{ids::*, metric::*, time::DurationNs};
    let mut drag = DragPlan::new(
        MetricDefinitionId::new(1),
        ReferencePointId::new(1),
        LaunchRule::FirstSustainedMotion {
            threshold_mps: 0.5,
            dwell: DurationNs::from_ns(200_000_000),
        },
    );
    drag.push_target(DragTarget::Speed {
        id: TargetId::new(1),
        quantity: SpeedQuantity::InstantaneousHorizontal,
        metres_per_second: 100.0 / 3.6,
        direction: TargetDirection::Ascending,
    })?;
    drag.push_target(DragTarget::quarter_mile_horizontal_path(TargetId::new(2)))?;
    plan.push(MetricDefinition::Drag(drag))
}
