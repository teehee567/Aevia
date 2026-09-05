//! Shared calibration and clock coordinates retained by the consider-state model.

use crate::{
    config::{EngineConfig, SharedParameterKind},
    error::ProcessError,
    ids::{ClockModelId, SharedParameterId},
    offline::ports::{ClockModelEvidence, MAX_OFFLINE_CLOCK_MODELS},
    time::{SessionTime, TimeSpan},
};

use nalgebra::DMatrix;

use std::vec::Vec;

use super::{estimation::matrix_is_psd, math::copy_block};

#[derive(Clone)]
pub(super) struct OwnedClockModel {
    pub(super) model: ClockModelId,
    pub(super) segment: crate::ids::ClockSegmentId,
    pub(super) validity: TimeSpan,
    pub(super) reference_time: SessionTime,
    pub(super) offset_ns: f64,
    pub(super) fractional_drift: f64,
    pub(super) covariance_upper: [f64; 3],
    pub(super) cross_covariance_with_prior: Vec<f64>,
    pub(super) offset_index: usize,
}

#[derive(Clone, Copy)]
pub(super) struct ParameterCoordinate {
    pub(super) id: SharedParameterId,
    pub(super) kind: SharedParameterKind,
    pub(super) validity: TimeSpan,
    pub(super) start: usize,
    pub(super) dimension: usize,
}

pub(super) struct ConsiderCatalog {
    pub(super) parameters: Vec<ParameterCoordinate>,
    pub(super) clocks: Vec<OwnedClockModel>,
    pub(super) covariance: DMatrix<f64>,
}

impl ConsiderCatalog {
    pub(super) fn from_config(config: &EngineConfig<'_>) -> Result<Self, ProcessError> {
        let dimension = config.calibration.shared_parameters.scalar_dimension();
        let mut covariance = DMatrix::zeros(dimension, dimension);
        let mut compact_index = 0;
        for row in 0..dimension {
            for column in row..dimension {
                let value = config
                    .calibration
                    .shared_parameters
                    .covariance
                    .upper_triangle()[compact_index];
                covariance[(row, column)] = value;
                covariance[(column, row)] = value;
                compact_index += 1;
            }
        }
        let mut parameters = Vec::new();
        parameters
            .try_reserve_exact(config.calibration.shared_parameters.definitions.len())
            .map_err(|_| ProcessError::ResourceLimit)?;
        let mut start = 0;
        for definition in config.calibration.shared_parameters.definitions {
            let dimension = definition.mean.dimension();
            match definition.kind {
                SharedParameterKind::BoresightRadians
                | SharedParameterKind::MisalignmentRadians
                    if definition.id != config.installation.body_from_imu.parameter_id
                        || dimension != 3 =>
                {
                    return Err(ProcessError::CapabilityUnavailable);
                }
                // These names alone do not identify a sensor/axis mapping or
                // define how a scalar applies to a calibrated measurement.
                SharedParameterKind::Scale
                | SharedParameterKind::GyroGSensitivity
                | SharedParameterKind::Other(_) => {
                    return Err(ProcessError::CapabilityUnavailable);
                }
                _ => {}
            }
            let parameter = ParameterCoordinate {
                id: definition.id,
                kind: definition.kind,
                validity: definition.validity,
                start,
                dimension,
            };
            start = start
                .checked_add(parameter.dimension)
                .ok_or(ProcessError::ResourceLimit)?;
            parameters.push(parameter);
        }
        if start != dimension || !matrix_is_psd(&covariance) {
            return Err(ProcessError::InvalidEvidence);
        }
        Ok(Self {
            parameters,
            clocks: Vec::new(),
            covariance,
        })
    }

    pub(super) fn push_clock(
        &mut self,
        evidence: ClockModelEvidence<'_>,
    ) -> Result<(), ProcessError> {
        if self.clocks.len() >= MAX_OFFLINE_CLOCK_MODELS
            || self
                .clocks
                .iter()
                .any(|present| present.model == evidence.model)
        {
            return Err(ProcessError::InvalidEvidence);
        }
        let prior_dimension = self.covariance.nrows();
        evidence.validate(prior_dimension)?;
        let new_dimension = prior_dimension
            .checked_add(2)
            .ok_or(ProcessError::ResourceLimit)?;
        let mut expanded = DMatrix::zeros(new_dimension, new_dimension);
        copy_block(&self.covariance, &mut expanded, 0, 0);
        for row in 0..2 {
            for column in 0..prior_dimension {
                let value = evidence.cross_covariance_with_prior[row * prior_dimension + column];
                expanded[(prior_dimension + row, column)] = value;
                expanded[(column, prior_dimension + row)] = value;
            }
        }
        expanded[(prior_dimension, prior_dimension)] = evidence.covariance_upper[0];
        expanded[(prior_dimension, prior_dimension + 1)] = evidence.covariance_upper[1];
        expanded[(prior_dimension + 1, prior_dimension)] = evidence.covariance_upper[1];
        expanded[(prior_dimension + 1, prior_dimension + 1)] = evidence.covariance_upper[2];
        self.covariance = expanded;
        self.clocks.push(OwnedClockModel {
            model: evidence.model,
            segment: evidence.segment,
            validity: evidence.validity,
            reference_time: evidence.reference_time,
            offset_ns: evidence.offset_ns,
            fractional_drift: evidence.fractional_drift,
            covariance_upper: evidence.covariance_upper,
            cross_covariance_with_prior: evidence.cross_covariance_with_prior.to_vec(),
            offset_index: prior_dimension,
        });
        Ok(())
    }

    pub(super) fn parameter(&self, id: SharedParameterId) -> Option<ParameterCoordinate> {
        self.parameters.iter().copied().find(|value| value.id == id)
    }

    pub(super) fn covers_span(&self, span: TimeSpan) -> bool {
        self.parameters.iter().all(|parameter| {
            parameter.validity.contains(span.start()) && parameter.validity.contains(span.end())
        })
    }

    pub(super) fn clock(&self, id: ClockModelId, time: SessionTime) -> Option<&OwnedClockModel> {
        self.clocks
            .iter()
            .find(|clock| clock.model == id && clock.validity.contains(time))
    }
}
