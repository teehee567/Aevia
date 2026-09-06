//! Shared surveyed-gate binding validation for live and host plans.

use crate::config::{
    EngineConfig, SharedParameterKind, SharedParameterMean, SharedParameterSet,
    SharedUncertaintyTreatment,
};
use crate::error::PrepareError;
use crate::metric::{FiniteGate, GateSurveyUncertainty};
use crate::time::TimeSpan;

pub(super) fn validate_surveyed_gate_bindings(
    engine: &EngineConfig<'_>,
    metrics: &crate::metric::MetricPlan,
    required_span: Option<TimeSpan>,
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
            let survey_sigma_m = crate::scalar_math::sqrt(survey_normal_variance(
                engine.calibration.shared_parameters,
                gate,
                required_span,
            )?);
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

fn survey_normal_variance(
    shared: SharedParameterSet<'_>,
    gate: &FiniteGate,
    required_span: Option<TimeSpan>,
) -> Result<f64, PrepareError> {
    let parameter = match gate.survey_uncertainty {
        GateSurveyUncertainty::Exact => return Ok(0.0),
        GateSurveyUncertainty::UnspecifiedVariance(variance)
        | GateSurveyUncertainty::Independent(variance)
            if variance.is_finite() && variance >= 0.0 =>
        {
            return Ok(variance);
        }
        GateSurveyUncertainty::Shared(parameter) => parameter,
        _ => return Err(PrepareError::FrameUnresolved),
    };
    if shared.treatment != SharedUncertaintyTreatment::SchmidtConsider {
        return Err(PrepareError::IncompatibleProfile);
    }
    let mut start = 0;
    let definition = shared
        .definitions
        .iter()
        .find(|definition| {
            if definition.id == parameter {
                true
            } else {
                start += definition.mean.dimension();
                false
            }
        })
        .ok_or(PrepareError::FrameUnresolved)?;
    if definition.kind != SharedParameterKind::SurveyMetres
        || !matches!(definition.mean, SharedParameterMean::Vector3(mean) if mean.components() == [0.0; 3])
        || required_span.is_some_and(|span| {
            !definition.validity.contains(span.start()) || !definition.validity.contains(span.end())
        })
    {
        return Err(PrepareError::FrameUnresolved);
    }
    let dimension = shared.covariance.dimension();
    let mut variance = 0.0;
    for row in 0..3 {
        for column in 0..3 {
            let low = start + row.min(column);
            let high = start + row.max(column);
            let index = low * dimension - low * low.saturating_sub(1) / 2 + high - low;
            let covariance = shared
                .covariance
                .upper_triangle()
                .get(index)
                .ok_or(PrepareError::FrameUnresolved)?;
            variance += gate.normal_ecef[row] * covariance * gate.normal_ecef[column];
        }
    }
    if !variance.is_finite() || variance < 0.0 {
        return Err(PrepareError::FrameUnresolved);
    }
    Ok(variance)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::SharedParameterDefinition,
        ids::{FrameId, GateId, SharedParameterId},
        math::{FiniteF64, Vector3},
        metric::CrossingDirection,
        time::{DurationNs, SessionTime},
        uncertainty::SharedParameterCovariance,
    };

    #[test]
    fn shared_survey_binding_projects_full_covariance_and_checks_semantics_and_validity() {
        let span = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(100)).unwrap();
        let gate = FiniteGate::new(
            GateId::new(1),
            FrameId::new(1),
            [0.0; 3],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            10.0,
            10.0,
            CrossingDirection::Either,
            1.0,
            1.0,
            DurationNs::ZERO,
            None,
        )
        .unwrap()
        .with_shared_survey_parameter(SharedParameterId::new(9))
        .unwrap();
        let prefix = SharedParameterDefinition {
            id: SharedParameterId::new(7),
            kind: SharedParameterKind::ClockDrift,
            mean: SharedParameterMean::Scalar(FiniteF64::new(0.0).unwrap()),
            validity: span,
        };
        let survey = SharedParameterDefinition {
            id: SharedParameterId::new(9),
            kind: SharedParameterKind::SurveyMetres,
            mean: SharedParameterMean::Vector3(Vector3::new(0.0, 0.0, 0.0).unwrap()),
            validity: span,
        };
        let definitions = [prefix, survey];
        let covariance = SharedParameterCovariance::new(
            4,
            &[1.0, 0.0, 0.0, 0.0, 0.04, 0.01, 0.0, 0.09, 0.0, 0.16],
        )
        .unwrap();
        let shared = SharedParameterSet {
            definitions: &definitions,
            covariance,
            treatment: SharedUncertaintyTreatment::SchmidtConsider,
        };
        assert!(
            (survey_normal_variance(shared, &gate, Some(span)).unwrap() - 0.075).abs() < 1.0e-15
        );
        let longer = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(101)).unwrap();
        assert_eq!(
            survey_normal_variance(shared, &gate, Some(longer)),
            Err(PrepareError::FrameUnresolved)
        );
        for wrong_survey in [
            SharedParameterDefinition {
                kind: SharedParameterKind::LeverArmMetres,
                ..survey
            },
            SharedParameterDefinition {
                id: SharedParameterId::new(10),
                ..survey
            },
            SharedParameterDefinition {
                mean: SharedParameterMean::Vector3(Vector3::new(0.0, 1.0, 0.0).unwrap()),
                ..survey
            },
        ] {
            let definitions = [prefix, wrong_survey];
            assert_eq!(
                survey_normal_variance(
                    SharedParameterSet {
                        definitions: &definitions,
                        ..shared
                    },
                    &gate,
                    Some(span)
                ),
                Err(PrepareError::FrameUnresolved),
            );
        }
    }
}
