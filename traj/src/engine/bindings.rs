//! Shared surveyed-gate binding validation for live and host plans.

use crate::config::EngineConfig;
use crate::error::PrepareError;

pub(super) fn validate_surveyed_gate_bindings(
    engine: &EngineConfig<'_>,
    metrics: &crate::metric::MetricPlan,
) -> Result<(), PrepareError> {
    for definition in metrics.definitions() {
        let crate::metric::MetricDefinition::Lap(plan) = definition else {
            continue;
        };
        if !engine.processing_frame.supports_surveyed_geometry() {
            return Err(PrepareError::FrameUnresolved);
        }
        for gate in plan.gates() {
            if gate.frame.get() == 0 || gate.frame != engine.processing_frame.id() {
                return Err(PrepareError::FrameUnresolved);
            }
            let survey_sigma_m = gate
                .survey_variance_normal_m2
                .map(crate::scalar_math::sqrt)
                .ok_or(PrepareError::FrameUnresolved)?;
            let operation_is_qualified = engine.coordinate_operations.iter().any(|operation| {
                operation.source() == gate.frame
                    && operation.target() == engine.processing_frame.id()
                    && operation.supports_surveyed_accuracy(survey_sigma_m)
            });
            if !operation_is_qualified {
                return Err(PrepareError::FrameUnresolved);
            }
        }
    }
    Ok(())
}
