//! Validated serialization of offline dense-segment records.

use super::TrajectoryKnot;
#[cfg(feature = "offline")]
use super::bridge::{
    BRIDGE_ATTITUDE, BRIDGE_ENDPOINT_DIMENSION, BRIDGE_KINEMATIC_DIMENSION, BRIDGE_POSITION,
    BRIDGE_VELOCITY, DenseBridgeInput,
};
use super::dense::DenseSegment;
#[cfg(feature = "offline")]
use super::storage::{BackedSegmentRecord, OFFLINE_SEGMENT_PAYLOAD_BYTES, coupled_payload_bytes};
use crate::error::ValidationError;
use crate::frame::{BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody};
use crate::math::UnitQuaternion;
use crate::quality::{
    CovarianceConditioning, EstimateQuality, EstimateStage, GnssState, HeadingObservability,
    HeadingSource, Integrity, ObservabilityReport, TimingQuality, Validity,
};
use crate::time::SessionTime;
#[cfg(feature = "offline")]
use crate::uncertainty::CrossCovariance3;
use crate::uncertainty::{Covariance3, KinematicCovariance};
#[cfg(feature = "offline")]
use std::boxed::Box;
#[cfg(feature = "offline")]
use std::vec::Vec;

#[cfg(feature = "offline")]
pub(super) fn encode_offline_segment_record(
    target: &mut Vec<u8>,
    start: TrajectoryKnot,
    end: TrajectoryKnot,
    input: &DenseBridgeInput,
    store_indices: (u64, u64),
) -> Result<(), ValidationError> {
    target.clear();
    target.extend_from_slice(&store_indices.0.to_le_bytes());
    target.extend_from_slice(&store_indices.1.to_le_bytes());
    target.push(u8::from(input.covariance_available));
    for value in input.endpoint_joint_covariance.iter().flatten() {
        put_offline_f64(target, *value);
    }
    for value in input
        .acceleration_spectral_density_ecef
        .iter()
        .flatten()
        .chain(input.attitude_spectral_density_body.iter().flatten())
        .chain(
            input
                .acceleration_interval_average_covariance_ecef
                .iter()
                .flatten(),
        )
        .chain(
            input
                .angular_rate_interval_average_covariance_body
                .iter()
                .flatten(),
        )
        .chain(input.reintegrated_position_ecef_m.iter())
        .chain(input.reintegrated_velocity_ecef_mps.iter())
        .chain(input.integrated_rotation_body.iter())
    {
        put_offline_f64(target, *value);
    }
    encode_offline_knot(target, start);
    encode_offline_knot(target, end);
    let dimension = input.coupled.as_ref().map_or(0, |value| value.dimension());
    if let Some(model) = &input.coupled {
        target.extend_from_slice(&(dimension as u64).to_le_bytes());
        target.extend_from_slice(&(model.state_dimension as u64).to_le_bytes());
        for matrix in [
            &model.continuous,
            &model.noise_density,
            &model.endpoint_joint,
            &model.start_to_reference,
            &model.end_to_reference,
            &model.rate_mapping,
        ] {
            for value in matrix.iter() {
                put_offline_f64(target, *value);
            }
        }
        for id in &model.parameter_ids {
            target.extend_from_slice(&id.to_le_bytes());
        }
        for value in core::iter::once(&model.duration_seconds)
            .chain(model.reference_start_orientation.iter())
            .chain(model.reference_body_rate.iter())
            .chain(model.gyro_density.iter().flatten())
        {
            put_offline_f64(target, *value);
        }
    }
    if target.len() as u64 != coupled_payload_bytes(dimension)? {
        return Err(ValidationError::CapacityExceeded);
    }
    Ok(())
}

#[cfg(feature = "offline")]
pub(super) fn encode_offline_knot(target: &mut Vec<u8>, knot: TrajectoryKnot) {
    target.extend_from_slice(&knot.time.as_ns().to_le_bytes());
    for value in knot
        .position_ecef
        .components()
        .iter()
        .chain(knot.velocity_ecef.components().iter())
        .chain(
            knot.orientation_ecef_from_body
                .quaternion()
                .components_wxyz()
                .iter(),
        )
        .chain(knot.specific_force_body.components().iter())
    {
        put_offline_f64(target, *value);
    }
    target.extend_from_slice(&[
        encode_estimate_stage(knot.quality.stage),
        encode_validity(knot.quality.validity),
        encode_gnss_state(knot.quality.gnss),
        encode_timing_quality(knot.quality.timing),
        encode_integrity(knot.quality.integrity),
        encode_covariance_conditioning(knot.quality.covariance),
        u8::from(knot.quality.imu_gap),
        u8::from(knot.quality.degraded_input),
        encode_heading_source(knot.observability.heading_source),
        encode_heading_observability(knot.observability.heading),
        u8::from(knot.observability.heading_variance_rad2.is_some()),
    ]);
    put_offline_f64(
        target,
        knot.observability.heading_variance_rad2.unwrap_or(0.0),
    );
    target.extend_from_slice(&[
        u8::from(knot.observability.course_available),
        u8::from(knot.observability.body_axis_quantities_available),
        u8::from(knot.observability.angular_acceleration_available),
    ]);
}

#[cfg(feature = "offline")]
pub(super) fn decode_offline_segment_record(
    bytes: &[u8],
    ordinal: usize,
) -> Result<BackedSegmentRecord, ValidationError> {
    if (bytes.len() as u64) < OFFLINE_SEGMENT_PAYLOAD_BYTES {
        return Err(ValidationError::CapacityExceeded);
    }
    let mut cursor = OfflineDecodeCursor { bytes, position: 0 };
    let store_indices = (cursor.u64()?, cursor.u64()?);
    let covariance_available = cursor.boolean()?;
    if store_indices.1 <= store_indices.0 {
        return Err(ValidationError::InvalidTimeSpan);
    }
    let mut endpoint_joint_covariance =
        Box::new([[0.0; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION]);
    for row in 0..BRIDGE_ENDPOINT_DIMENSION {
        for column in 0..BRIDGE_ENDPOINT_DIMENSION {
            endpoint_joint_covariance[row][column] = cursor.f64()?;
        }
    }
    let acceleration_spectral_density_ecef = cursor.matrix3()?;
    let attitude_spectral_density_body = cursor.matrix3()?;
    let acceleration_interval_average_covariance_ecef = cursor.matrix3()?;
    let angular_rate_interval_average_covariance_body = cursor.matrix3()?;
    let reintegrated_position_ecef_m = cursor.array3()?;
    let reintegrated_velocity_ecef_mps = cursor.array3()?;
    let integrated_rotation_body = cursor.array3()?;
    let start_decoded = cursor.knot()?;
    let end_decoded = cursor.knot()?;
    let coupled = if cursor.position < bytes.len() {
        let d = usize::try_from(cursor.u64()?).map_err(|_| ValidationError::CapacityExceeded)?;
        let n = usize::try_from(cursor.u64()?).map_err(|_| ValidationError::CapacityExceeded)?;
        if coupled_payload_bytes(d)? != bytes.len() as u64 || d == 0 {
            return Err(ValidationError::CapacityExceeded);
        }
        let continuous = cursor.dynamic_matrix(d, d)?;
        let noise_density = cursor.dynamic_matrix(d, d)?;
        let endpoint_joint = cursor.dynamic_matrix(2 * d, 2 * d)?;
        let start_to_reference = cursor.dynamic_matrix(d, d)?;
        let end_to_reference = cursor.dynamic_matrix(d, d)?;
        let rate_mapping = cursor.dynamic_matrix(3, d)?;
        let mut parameter_ids = Vec::with_capacity(d);
        for _ in 0..d {
            parameter_ids.push(cursor.u64()?);
        }
        Some(Box::new(super::coupled::CoupledDenseBridge {
            state_dimension: n,
            continuous,
            noise_density,
            endpoint_joint,
            start_to_reference,
            end_to_reference,
            rate_mapping,
            parameter_ids,
            duration_seconds: cursor.f64()?,
            reference_start_orientation: cursor.array4()?,
            reference_body_rate: cursor.array3()?,
            gyro_density: cursor.matrix3()?,
            cache: Default::default(),
        }))
    } else {
        None
    };
    if cursor.position != bytes.len() {
        return Err(ValidationError::IncompatibleDefinition);
    }
    let start_covariance = covariance_from_endpoint_joint(&endpoint_joint_covariance, 0)?;
    let end_covariance =
        covariance_from_endpoint_joint(&endpoint_joint_covariance, BRIDGE_KINEMATIC_DIMENSION)?;
    let start = start_decoded.into_knot(start_covariance)?;
    let end = end_decoded.into_knot(end_covariance)?;
    let input = DenseBridgeInput {
        coupled,
        covariance_available,
        endpoint_joint_covariance,
        acceleration_spectral_density_ecef,
        attitude_spectral_density_body,
        acceleration_interval_average_covariance_ecef,
        angular_rate_interval_average_covariance_body,
        reintegrated_position_ecef_m,
        reintegrated_velocity_ecef_mps,
        integrated_rotation_body,
    };
    let (segment, conditional_bridge) = DenseSegment::new_conditional(start, end, &input)?;
    Ok(BackedSegmentRecord {
        ordinal,
        segment,
        conditional_bridge,
        store_indices,
    })
}

#[cfg(feature = "offline")]
pub(super) fn covariance_from_endpoint_joint(
    joint: &[[f64; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION],
    offset: usize,
) -> Result<KinematicCovariance, ValidationError> {
    let block = |row_offset: usize, column_offset: usize| {
        core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                joint[offset + row_offset + row][offset + column_offset + column]
            })
        })
    };
    let position = Covariance3::from_matrix(block(BRIDGE_POSITION, BRIDGE_POSITION))?;
    let velocity = Covariance3::from_matrix(block(BRIDGE_VELOCITY, BRIDGE_VELOCITY))?;
    let attitude = Covariance3::from_matrix(block(BRIDGE_ATTITUDE, BRIDGE_ATTITUDE))?;
    let cross = CrossCovariance3::from_matrix(block(BRIDGE_POSITION, BRIDGE_VELOCITY))
        .ok()
        .filter(|value| value.forms_valid_joint(position, velocity));
    KinematicCovariance::new(position, velocity, cross, attitude)
        .or_else(|_| KinematicCovariance::new(position, velocity, None, attitude))
}

#[cfg(feature = "offline")]
pub(super) struct DecodedOfflineKnot {
    pub(super) time: SessionTime,
    pub(super) position: [f64; 3],
    pub(super) velocity: [f64; 3],
    pub(super) orientation: [f64; 4],
    pub(super) specific_force: [f64; 3],
    pub(super) quality: EstimateQuality,
    pub(super) observability: ObservabilityReport,
}

#[cfg(feature = "offline")]
impl DecodedOfflineKnot {
    pub(super) fn into_knot(
        self,
        covariance: KinematicCovariance,
    ) -> Result<TrajectoryKnot, ValidationError> {
        Ok(TrajectoryKnot {
            time: self.time,
            position_ecef: EcefPosition::from_components(self.position)?,
            velocity_ecef: EcefVelocity::from_components(self.velocity)?,
            orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(
                UnitQuaternion::from_wxyz(self.orientation)?,
            ),
            specific_force_body: BodyVector::from_components(self.specific_force)?,
            covariance,
            quality: self.quality,
            observability: self.observability,
        })
    }
}

#[cfg(feature = "offline")]
pub(super) struct OfflineDecodeCursor<'a> {
    pub(super) bytes: &'a [u8],
    pub(super) position: usize,
}

#[cfg(feature = "offline")]
impl OfflineDecodeCursor<'_> {
    fn dynamic_matrix(
        &mut self,
        rows: usize,
        columns: usize,
    ) -> Result<nalgebra::DMatrix<f64>, ValidationError> {
        let mut matrix = nalgebra::DMatrix::zeros(rows, columns);
        for value in matrix.iter_mut() {
            *value = self.f64()?;
        }
        Ok(matrix)
    }
    pub(super) fn take<const N: usize>(&mut self) -> Result<[u8; N], ValidationError> {
        let end = self
            .position
            .checked_add(N)
            .ok_or(ValidationError::CapacityExceeded)?;
        let source = self
            .bytes
            .get(self.position..end)
            .ok_or(ValidationError::IncompatibleDefinition)?;
        let mut result = [0; N];
        result.copy_from_slice(source);
        self.position = end;
        Ok(result)
    }

    pub(super) fn u8(&mut self) -> Result<u8, ValidationError> {
        Ok(self.take::<1>()?[0])
    }

    pub(super) fn boolean(&mut self) -> Result<bool, ValidationError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(ValidationError::IncompatibleDefinition),
        }
    }

    pub(super) fn u64(&mut self) -> Result<u64, ValidationError> {
        Ok(u64::from_le_bytes(self.take()?))
    }

    pub(super) fn i64(&mut self) -> Result<i64, ValidationError> {
        Ok(i64::from_le_bytes(self.take()?))
    }

    pub(super) fn f64(&mut self) -> Result<f64, ValidationError> {
        let value = f64::from_bits(u64::from_le_bytes(self.take()?));
        if value.is_finite() {
            Ok(value)
        } else {
            Err(ValidationError::NonFinite)
        }
    }

    pub(super) fn array3(&mut self) -> Result<[f64; 3], ValidationError> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }

    pub(super) fn array4(&mut self) -> Result<[f64; 4], ValidationError> {
        Ok([self.f64()?, self.f64()?, self.f64()?, self.f64()?])
    }

    pub(super) fn matrix3(&mut self) -> Result<[[f64; 3]; 3], ValidationError> {
        let mut result = [[0.0; 3]; 3];
        for row in &mut result {
            for value in row {
                *value = self.f64()?;
            }
        }
        Ok(result)
    }

    pub(super) fn knot(&mut self) -> Result<DecodedOfflineKnot, ValidationError> {
        let time = SessionTime::from_ns(self.i64()?);
        let position = self.array3()?;
        let velocity = self.array3()?;
        let orientation = self.array4()?;
        let specific_force = self.array3()?;
        let quality = EstimateQuality {
            stage: decode_estimate_stage(self.u8()?)?,
            validity: decode_validity(self.u8()?)?,
            gnss: decode_gnss_state(self.u8()?)?,
            timing: decode_timing_quality(self.u8()?)?,
            integrity: decode_integrity(self.u8()?)?,
            covariance: decode_covariance_conditioning(self.u8()?)?,
            imu_gap: self.boolean()?,
            degraded_input: self.boolean()?,
        };
        let heading_source = decode_heading_source(self.u8()?)?;
        let heading = decode_heading_observability(self.u8()?)?;
        let has_heading_variance = self.boolean()?;
        let heading_variance = self.f64()?;
        let observability = ObservabilityReport {
            heading_source,
            heading,
            heading_variance_rad2: has_heading_variance.then_some(heading_variance),
            course_available: self.boolean()?,
            body_axis_quantities_available: self.boolean()?,
            angular_acceleration_available: self.boolean()?,
        };
        Ok(DecodedOfflineKnot {
            time,
            position,
            velocity,
            orientation,
            specific_force,
            quality,
            observability,
        })
    }
}

#[cfg(feature = "offline")]
pub(super) fn put_offline_f64(target: &mut Vec<u8>, value: f64) {
    target.extend_from_slice(&value.to_bits().to_le_bytes());
}

#[cfg(feature = "offline")]
pub(super) fn encode_estimate_stage(value: EstimateStage) -> u8 {
    match value {
        EstimateStage::Predicted => 0,
        EstimateStage::Provisional => 1,
        EstimateStage::Finalized => 2,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_estimate_stage(value: u8) -> Result<EstimateStage, ValidationError> {
    match value {
        0 => Ok(EstimateStage::Predicted),
        1 => Ok(EstimateStage::Provisional),
        2 => Ok(EstimateStage::Finalized),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_validity(value: Validity) -> u8 {
    match value {
        Validity::Nominal => 0,
        Validity::Degraded => 1,
        Validity::Invalid => 2,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_validity(value: u8) -> Result<Validity, ValidationError> {
    match value {
        0 => Ok(Validity::Nominal),
        1 => Ok(Validity::Degraded),
        2 => Ok(Validity::Invalid),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_gnss_state(value: GnssState) -> u8 {
    match value {
        // Use a new tag so old readers cannot mistake generic health for RTK fixed.
        GnssState::Healthy => 7,
        GnssState::Absent => 5,
        GnssState::Suspect => 6,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_gnss_state(value: u8) -> Result<GnssState, ValidationError> {
    match value {
        // Legacy positioning-mode tags all represented accepted GNSS evidence.
        0..=4 | 7 => Ok(GnssState::Healthy),
        5 => Ok(GnssState::Absent),
        6 => Ok(GnssState::Suspect),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_timing_quality(value: TimingQuality) -> u8 {
    match value {
        TimingQuality::PpsCorrelated => 0,
        TimingQuality::Modeled => 1,
        TimingQuality::ArrivalOnly => 2,
        TimingQuality::Discontinuous => 3,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_timing_quality(value: u8) -> Result<TimingQuality, ValidationError> {
    match value {
        0 => Ok(TimingQuality::PpsCorrelated),
        1 => Ok(TimingQuality::Modeled),
        2 => Ok(TimingQuality::ArrivalOnly),
        3 => Ok(TimingQuality::Discontinuous),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_integrity(value: Integrity) -> u8 {
    match value {
        Integrity::Monitored => 0,
        Integrity::Unavailable => 1,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_integrity(value: u8) -> Result<Integrity, ValidationError> {
    match value {
        0 => Ok(Integrity::Monitored),
        1 => Ok(Integrity::Unavailable),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_covariance_conditioning(value: CovarianceConditioning) -> u8 {
    match value {
        CovarianceConditioning::UnconditionalModel => 0,
        CovarianceConditioning::ConditionalOnSelection => 1,
        CovarianceConditioning::Unavailable => 2,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_covariance_conditioning(
    value: u8,
) -> Result<CovarianceConditioning, ValidationError> {
    match value {
        0 => Ok(CovarianceConditioning::UnconditionalModel),
        1 => Ok(CovarianceConditioning::ConditionalOnSelection),
        2 => Ok(CovarianceConditioning::Unavailable),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_heading_source(value: HeadingSource) -> u8 {
    match value {
        HeadingSource::Supplied => 0,
        HeadingSource::Gyrocompass => 1,
        HeadingSource::DynamicAlignment => 2,
        HeadingSource::NonHolonomicConstraint => 3,
        HeadingSource::None => 4,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_heading_source(value: u8) -> Result<HeadingSource, ValidationError> {
    match value {
        0 => Ok(HeadingSource::Supplied),
        1 => Ok(HeadingSource::Gyrocompass),
        2 => Ok(HeadingSource::DynamicAlignment),
        3 => Ok(HeadingSource::NonHolonomicConstraint),
        4 => Ok(HeadingSource::None),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}

#[cfg(feature = "offline")]
pub(super) fn encode_heading_observability(value: HeadingObservability) -> u8 {
    match value {
        HeadingObservability::Supplied => 0,
        HeadingObservability::Gyrocompassed => 1,
        HeadingObservability::DynamicallyAligned => 2,
        HeadingObservability::Constrained => 3,
        HeadingObservability::Unobservable => 4,
    }
}

#[cfg(feature = "offline")]
pub(super) fn decode_heading_observability(
    value: u8,
) -> Result<HeadingObservability, ValidationError> {
    match value {
        0 => Ok(HeadingObservability::Supplied),
        1 => Ok(HeadingObservability::Gyrocompassed),
        2 => Ok(HeadingObservability::DynamicallyAligned),
        3 => Ok(HeadingObservability::Constrained),
        4 => Ok(HeadingObservability::Unobservable),
        _ => Err(ValidationError::IncompatibleDefinition),
    }
}
