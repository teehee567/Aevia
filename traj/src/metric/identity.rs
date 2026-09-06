//! Canonical version-one identities for metric result values.

use super::{
    definition::{DistanceQuantity, SpeedQuantity},
    report::{MetricDefinitionDiagnosticReason, MetricResultValue},
};
use crate::{
    error::ValidationError,
    quality::{EstimateStage, FieldValue, UnavailableReason, Validity},
    time::TimeSpan,
};

pub(super) fn gate_definition_digest_v1(
    gate: &super::definition::FiniteGate,
) -> crate::ids::ContentDigestV1 {
    use super::definition::{CrossingDirection, GateSurveyUncertainty};
    use sha2::{Digest, Sha256};

    let mut digest = Sha256::new();
    digest.update(b"aevia.finite-gate.v1\0");
    digest.update(gate.id.get().to_le_bytes());
    digest.update(gate.frame.get().to_le_bytes());
    for value in gate
        .center_ecef_m
        .iter()
        .chain(gate.normal_ecef.iter())
        .chain(gate.width_axis_ecef.iter())
        .chain(gate.height_axis_ecef.iter())
        .copied()
        .chain([
            gate.width_m,
            gate.height_m,
            gate.minimum_normal_speed_mps,
            gate.rearm_distance_m,
        ])
    {
        digest.update(
            (if value == 0.0 { 0.0 } else { value })
                .to_bits()
                .to_le_bytes(),
        );
    }
    digest.update([match gate.direction {
        CrossingDirection::NegativeToPositive => 0,
        CrossingDirection::PositiveToNegative => 1,
        CrossingDirection::Either => 2,
    }]);
    digest.update(gate.minimum_crossing_interval.as_ns().to_le_bytes());
    match gate.survey_uncertainty {
        GateSurveyUncertainty::Exact => digest.update([0]),
        GateSurveyUncertainty::Unspecified => digest.update([1]),
        GateSurveyUncertainty::UnspecifiedVariance(variance) => {
            digest.update([2]);
            digest.update(
                (if variance == 0.0 { 0.0 } else { variance })
                    .to_bits()
                    .to_le_bytes(),
            );
        }
        GateSurveyUncertainty::Independent(variance) => {
            digest.update([3]);
            digest.update(
                (if variance == 0.0 { 0.0 } else { variance })
                    .to_bits()
                    .to_le_bytes(),
            );
        }
        GateSurveyUncertainty::Shared(id) => {
            digest.update([4]);
            digest.update(id.get().to_le_bytes());
        }
    }
    crate::ids::ContentDigestV1::from_bytes(digest.finalize().into())
}

/// Encodes the engine-owned canonical v1 identity of one metric value.
///
/// Result artifact codecs intentionally live in a separate package. Captured
/// replay nevertheless needs a stable identity for mutations, so this narrow
/// helper owns only the semantic field ordering used by that identity. It is
/// not a general artifact serializer.
pub(crate) fn encode_metric_value_identity_v1(
    value: MetricResultValue,
    output: &mut [u8],
) -> Result<usize, ValidationError> {
    let mut writer = MetricIdentityWriter::new(output);
    match value {
        MetricResultValue::Distance(value) => {
            writer.u8(0)?;
            writer.u32(value.definition.get())?;
            writer.u8(distance_quantity_identity(value.quantity))?;
            writer.u32(value.reference_point.get())?;
            writer.span(value.span)?;
            writer.f64(value.metres)?;
            writer.f64(value.numerical_error_m)?;
            writer.field(value.uncertainty_one_sigma_m)?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::GateCrossing(value) => {
            writer.u8(1)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.gate.get())?;
            writer.i64(value.time.as_ns())?;
            writer.field(value.time_one_sigma_s)?;
            writer.f64(value.normal_speed_mps)?;
            writer.bool(value.crossing_speed.is_some())?;
            if let Some((quantity, speed)) = value.crossing_speed {
                writer.u8(speed_quantity_identity(quantity))?;
                writer.field(speed)?;
            }
            writer.bool(value.crossing_speed_one_sigma_mps.is_some())?;
            if let Some((quantity, sigma)) = value.crossing_speed_one_sigma_mps {
                writer.u8(speed_quantity_identity(quantity))?;
                writer.field(sigma)?;
            }
            writer.u32(value.reference_point.get())?;
            writer.u32(value.occurrence)?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::Lap(value) => {
            writer.u8(2)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.lap_index)?;
            writer.u32(value.start_gate.get())?;
            writer.u32(value.end_gate.get())?;
            writer.i64(value.start.as_ns())?;
            writer.i64(value.end.as_ns())?;
            writer.f64(value.elapsed_seconds)?;
            writer.field(value.elapsed_one_sigma_s)?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::DragTarget(value) => {
            writer.u8(3)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.target.get())?;
            writer.i64(value.launch_time.as_ns())?;
            writer.i64(value.event_time.as_ns())?;
            writer.field(value.event_time_one_sigma_s)?;
            writer.f64(value.elapsed_seconds)?;
            writer.field(value.elapsed_one_sigma_s)?;
            writer.field(value.rollout_adjusted_seconds)?;
            writer.bool(value.terminal_speed.is_some())?;
            if let Some((quantity, speed)) = value.terminal_speed {
                writer.u8(speed_quantity_identity(quantity))?;
                writer.f64(speed)?;
            }
            writer.bool(value.terminal_speed_one_sigma_mps.is_some())?;
            if let Some((quantity, sigma)) = value.terminal_speed_one_sigma_mps {
                writer.u8(speed_quantity_identity(quantity))?;
                writer.field(sigma)?;
            }
            writer.field(value.terminal_speed_slope_mps2)?;
            writer.u32(value.reference_point.get())?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::Activity(value) => {
            writer.u8(4)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.reference_point.get())?;
            writer.span(value.span)?;
            writer.f64(value.elapsed_seconds)?;
            writer.f64(value.moving_seconds)?;
            writer.field(value.horizontal_distance_m)?;
            writer.field(value.spatial_distance_m)?;
            writer.field(value.ascent_m)?;
            writer.field(value.descent_m)?;
            writer.u8(speed_quantity_identity(value.peak_speed))?;
            writer.f64(value.peak_speed_mps)?;
            writer.u64(value.peak_window.as_ns())?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::ActivitySplit(value) => {
            writer.u8(5)?;
            writer.u32(value.definition.get())?;
            writer.u16(value.split_index)?;
            writer.f64(value.horizontal_distance_m)?;
            writer.i64(value.time.as_ns())?;
            writer.f64(value.elapsed_seconds)?;
            writer.u32(value.reference_point.get())?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::SkiSegment(value) => {
            writer.u8(6)?;
            writer.u32(value.definition.get())?;
            writer.u8(value.state as u8)?;
            writer.i64(value.start.as_ns())?;
            writer.i64(value.end.as_ns())?;
            writer.f64(value.confidence)?;
            writer.u32(value.reference_point.get())?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::Ski(value) => {
            writer.u8(7)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.downhill_segments)?;
            writer.u32(value.lift_segments)?;
            writer.u32(value.ascent_segments)?;
            writer.f64(value.downhill_seconds)?;
            writer.u32(value.reference_point.get())?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
        MetricResultValue::Unavailable(value) => {
            writer.u8(8)?;
            writer.u32(value.definition.get())?;
            writer.u32(value.reference_point.get())?;
            writer.u8(metric_diagnostic_reason_identity(value.reason))?;
            writer.u8(stage_identity(value.stage))?;
            writer.u8(validity_identity(value.validity))?;
        }
    }
    Ok(writer.finish())
}

struct MetricIdentityWriter<'a> {
    output: &'a mut [u8],
    position: usize,
}

impl<'a> MetricIdentityWriter<'a> {
    const fn new(output: &'a mut [u8]) -> Self {
        Self {
            output,
            position: 0,
        }
    }

    fn bytes(&mut self, bytes: &[u8]) -> Result<(), ValidationError> {
        let end = self
            .position
            .checked_add(bytes.len())
            .ok_or(ValidationError::CapacityExceeded)?;
        self.output
            .get_mut(self.position..end)
            .ok_or(ValidationError::CapacityExceeded)?
            .copy_from_slice(bytes);
        self.position = end;
        Ok(())
    }

    fn u8(&mut self, value: u8) -> Result<(), ValidationError> {
        self.bytes(&[value])
    }

    fn bool(&mut self, value: bool) -> Result<(), ValidationError> {
        self.u8(u8::from(value))
    }

    fn u16(&mut self, value: u16) -> Result<(), ValidationError> {
        self.bytes(&value.to_le_bytes())
    }

    fn u32(&mut self, value: u32) -> Result<(), ValidationError> {
        self.bytes(&value.to_le_bytes())
    }

    fn u64(&mut self, value: u64) -> Result<(), ValidationError> {
        self.bytes(&value.to_le_bytes())
    }

    fn i64(&mut self, value: i64) -> Result<(), ValidationError> {
        self.bytes(&value.to_le_bytes())
    }

    fn f64(&mut self, value: f64) -> Result<(), ValidationError> {
        if !value.is_finite() {
            return Err(ValidationError::NonFinite);
        }
        self.u64(value.to_bits())
    }

    fn span(&mut self, value: TimeSpan) -> Result<(), ValidationError> {
        self.i64(value.start().as_ns())?;
        self.i64(value.end().as_ns())
    }

    fn field(&mut self, value: FieldValue<f64>) -> Result<(), ValidationError> {
        match value {
            FieldValue::Available(value) => {
                self.u8(0)?;
                self.f64(value)
            }
            FieldValue::Unavailable(reason) => {
                self.u8(1)?;
                self.u8(unavailable_reason_identity(reason))
            }
        }
    }

    const fn finish(self) -> usize {
        self.position
    }
}

const fn speed_quantity_identity(value: SpeedQuantity) -> u8 {
    match value {
        SpeedQuantity::InstantaneousHorizontal => 0,
        SpeedQuantity::Spatial3d => 1,
        SpeedQuantity::BodyLongitudinalSigned => 2,
        SpeedQuantity::BodyLongitudinalMagnitude => 3,
    }
}

const fn distance_quantity_identity(value: DistanceQuantity) -> u8 {
    match value {
        DistanceQuantity::HorizontalPath => 0,
        DistanceQuantity::Spatial3d => 1,
        DistanceQuantity::BodyLongitudinalSigned => 2,
        DistanceQuantity::BodyLongitudinalAbsolute => 3,
    }
}

const fn unavailable_reason_identity(value: UnavailableReason) -> u8 {
    match value {
        UnavailableReason::Unobservable => 0,
        UnavailableReason::InsufficientSignalToNoise => 1,
        UnavailableReason::MissingUncertainty => 2,
        UnavailableReason::OutsideQualifiedRange => 3,
        UnavailableReason::UnsupportedAtProcessingLevel => 4,
        UnavailableReason::FrameUnresolved => 5,
        UnavailableReason::Gap => 6,
        UnavailableReason::IllConditioned => 7,
        UnavailableReason::Ambiguous => 8,
        UnavailableReason::MissingCorrelation => 9,
    }
}

const fn metric_diagnostic_reason_identity(value: MetricDefinitionDiagnosticReason) -> u8 {
    match value {
        MetricDefinitionDiagnosticReason::InvalidDefinition => 0,
        MetricDefinitionDiagnosticReason::ReferencePointUnavailable => 1,
        MetricDefinitionDiagnosticReason::FrameMismatch => 2,
        MetricDefinitionDiagnosticReason::Unobservable => 3,
        MetricDefinitionDiagnosticReason::Ambiguous => 4,
        MetricDefinitionDiagnosticReason::UnsupportedAtProcessingLevel => 5,
        MetricDefinitionDiagnosticReason::AttachmentModelUnavailable => 6,
    }
}

const fn stage_identity(value: EstimateStage) -> u8 {
    match value {
        EstimateStage::Predicted => 0,
        EstimateStage::Provisional => 1,
        EstimateStage::Finalized => 2,
    }
}

const fn validity_identity(value: Validity) -> u8 {
    match value {
        Validity::Nominal => 0,
        Validity::Degraded => 1,
        Validity::Invalid => 2,
    }
}
