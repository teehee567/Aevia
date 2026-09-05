//! Live calibration layout and metric capability preflight.

use super::LiveConsiderLayout;
use crate::config::{EngineConfig, SharedParameterKind, SharedUncertaintyTreatment};
use crate::engine::bindings::validate_surveyed_gate_bindings;
use crate::error::{PrepareError, ValidationError};
use crate::live::MAX_CONSIDER;
use crate::trajectory::MAX_ROOT_ISOLATION_EVALUATIONS;

#[cfg(test)]
#[path = "preflight_tests.rs"]
mod tests;

pub(super) fn compile_live_consider_layout(
    engine: &EngineConfig<'_>,
) -> Result<LiveConsiderLayout, PrepareError> {
    let shared = engine.calibration.shared_parameters;
    if !matches!(
        shared.treatment,
        SharedUncertaintyTreatment::SchmidtConsider
    ) {
        // The current schema carries only the qualification digest for a
        // sequence bound, not an executable numeric bound. Silently treating
        // it as zero or as independent observation noise would be unsound.
        return Err(PrepareError::IncompatibleProfile);
    }
    if engine.installation.body_from_imu.parameter_id
        == engine.installation.imu_to_gnss_antenna.parameter_id
    {
        return Err(PrepareError::IncompatibleProfile);
    }

    let mut coordinate = 2_usize;
    let mut imu_boresight_start = None;
    let mut antenna_lever_start = None;
    for definition in shared.definitions {
        let dimension = definition.mean.dimension();
        let is_imu = definition.id == engine.installation.body_from_imu.parameter_id;
        let is_lever = definition.id == engine.installation.imu_to_gnss_antenna.parameter_id;
        let supported = if is_imu {
            dimension == 3
                && matches!(
                    definition.kind,
                    SharedParameterKind::BoresightRadians
                        | SharedParameterKind::MisalignmentRadians
                )
        } else if is_lever {
            dimension == 3 && definition.kind == SharedParameterKind::LeverArmMetres
        } else {
            false
        };
        if !supported {
            // Scale, g-sensitivity, delay, survey, and profile-defined
            // parameters require explicit process/measurement sensitivities.
            // Until supplied, a Schmidt profile containing one is rejected.
            return Err(PrepareError::IncompatibleProfile);
        }
        let start = u8::try_from(coordinate).map_err(|_| PrepareError::IncompatibleProfile)?;
        if is_imu {
            imu_boresight_start = Some(start);
        }
        if is_lever {
            antenna_lever_start = Some(start);
        }
        coordinate = coordinate
            .checked_add(dimension)
            .ok_or(PrepareError::IncompatibleProfile)?;
    }
    if coordinate != usize::from(engine.navigation_profile.consider_dimension)
        || coordinate > MAX_CONSIDER
    {
        return Err(PrepareError::IncompatibleProfile);
    }
    Ok(LiveConsiderLayout {
        imu_boresight_start: imu_boresight_start.ok_or(PrepareError::IncompatibleProfile)?,
        antenna_lever_start: antenna_lever_start.ok_or(PrepareError::IncompatibleProfile)?,
    })
}

pub(super) fn validate_live_metric_bindings(
    engine: &EngineConfig<'_>,
    metrics: &crate::metric::LiveMetricPlan,
    initial_heading_available: bool,
) -> Result<(), PrepareError> {
    validate_surveyed_gate_bindings(engine, metrics.plan())?;
    let mut needs_non_polynomial_roots = false;
    for definition in metrics.plan().definitions() {
        let reference = match definition {
            crate::metric::MetricDefinition::Distance(plan) => plan.reference_point,
            crate::metric::MetricDefinition::Lap(plan) => {
                if plan
                    .gates()
                    .iter()
                    .any(|gate| gate.frame != engine.processing_frame.id())
                {
                    return Err(PrepareError::FrameUnresolved);
                }
                plan.reference_point
            }
            crate::metric::MetricDefinition::Drag(plan) => plan.reference_point,
            crate::metric::MetricDefinition::Activity(plan) => plan.reference_point,
            crate::metric::MetricDefinition::Ski(_) => {
                return Err(PrepareError::IncompatibleProfile);
            }
        };
        if !engine
            .installation
            .reference_points
            .iter()
            .any(|point| point.id() == reference)
        {
            return Err(PrepareError::InvalidDefinition(
                ValidationError::InvalidReferencePoint,
            ));
        }
        let has_offset = engine
            .installation
            .reference_points
            .iter()
            .find(|point| point.id() == reference)
            .is_some_and(|point| point.imu_to_point().components_m() != [0.0; 3]);
        if !initial_heading_available && live_definition_requires_body_yaw(definition) {
            return Err(PrepareError::CapabilityUnavailable);
        }
        if has_offset && live_definition_requires_angular_acceleration(definition) {
            return Err(PrepareError::CapabilityUnavailable);
        }
        needs_non_polynomial_roots |=
            live_definition_needs_non_polynomial_roots(definition, has_offset);
    }

    let requested_evaluations = u32::from(metrics.limits().max_root_evaluations);
    if requested_evaluations > MAX_ROOT_ISOLATION_EVALUATIONS {
        return Err(PrepareError::IncompatibleProfile);
    }
    if needs_non_polynomial_roots {
        let attestation = engine
            .live_root_enclosure_qualification()
            .ok_or(PrepareError::CapabilityUnavailable)?;
        if requested_evaluations > attestation.maximum_root_evaluations_per_scalar {
            return Err(PrepareError::CapabilityUnavailable);
        }
    }
    Ok(())
}

fn live_definition_requires_body_yaw(definition: &crate::metric::MetricDefinition) -> bool {
    use crate::metric::{
        DistanceQuantity, DragTarget, LaunchRule, MetricDefinition, Rollout, SpeedQuantity,
    };

    let body_speed = |quantity| {
        matches!(
            quantity,
            SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude
        )
    };
    let body_distance = |quantity| {
        matches!(
            quantity,
            DistanceQuantity::BodyLongitudinalSigned | DistanceQuantity::BodyLongitudinalAbsolute
        )
    };

    match definition {
        MetricDefinition::Distance(plan) => body_distance(plan.quantity),
        MetricDefinition::Lap(plan) => plan.crossing_speed.is_some_and(body_speed),
        MetricDefinition::Drag(plan) => {
            matches!(
                plan.launch,
                LaunchRule::SpeedThreshold { quantity, .. } if body_speed(quantity)
            ) || matches!(
                plan.rollout,
                Rollout::Distance { quantity, .. } if body_distance(quantity)
            ) || plan.targets().iter().any(|target| match *target {
                DragTarget::Speed { quantity, .. } => body_speed(quantity),
                DragTarget::Distance { quantity, .. } => body_distance(quantity),
            })
        }
        MetricDefinition::Activity(plan) => {
            body_speed(plan.moving_speed) || body_speed(plan.peak_speed)
        }
        MetricDefinition::Ski(_) => true,
    }
}

fn live_definition_requires_angular_acceleration(
    definition: &crate::metric::MetricDefinition,
) -> bool {
    matches!(
        definition,
        crate::metric::MetricDefinition::Drag(plan)
            if matches!(plan.launch, crate::metric::LaunchRule::AccelerationChangePoint { .. })
    )
}

fn live_definition_needs_non_polynomial_roots(
    definition: &crate::metric::MetricDefinition,
    has_offset: bool,
) -> bool {
    use crate::metric::{
        DistanceQuantity, DragTarget, LaunchRule, MetricDefinition, Rollout, SpeedQuantity,
    };

    let speed = |quantity| has_offset || !matches!(quantity, SpeedQuantity::Spatial3d);
    let distance_value = |quantity| has_offset || !matches!(quantity, DistanceQuantity::Spatial3d);

    match definition {
        MetricDefinition::Distance(plan) => distance_value(plan.quantity),
        MetricDefinition::Lap(plan) => has_offset || plan.crossing_speed.is_some_and(speed),
        MetricDefinition::Drag(plan) => {
            has_offset
                || match plan.launch {
                    LaunchRule::FirstSustainedMotion { .. } => true,
                    LaunchRule::SpeedThreshold { quantity, .. } => speed(quantity),
                    LaunchRule::AccelerationChangePoint { .. }
                    | LaunchRule::ExternalTimestamp(_) => false,
                }
                // A cumulative-distance event roots an integral, which is
                // non-polynomial even when the origin-point spatial-speed
                // threshold itself has a quartic polynomial form.
                || matches!(plan.rollout, Rollout::Distance { .. })
                || plan.targets().iter().any(|target| match *target {
                    DragTarget::Speed { quantity, .. } => speed(quantity),
                    DragTarget::Distance { .. } => true,
                })
        }
        MetricDefinition::Activity(plan) => {
            has_offset
                || speed(plan.moving_speed)
                || speed(plan.peak_speed)
                || plan.include_horizontal_distance
                || !plan.splits_m().is_empty()
        }
        // Ski plans are already rejected from live preflight, but treating
        // one conservatively here keeps this classifier fail closed.
        MetricDefinition::Ski(_) => true,
    }
}
