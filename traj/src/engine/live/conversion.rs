//! Checked scalar, covariance, and frame conversions at the public estimator seam.

use crate::engine::LiveProjection;
use crate::error::{StepError, ValidationError};
use crate::frame::{BodyVector, EcefPosition, EcefVelocity, OrientationEcefFromBody};
use crate::live::{DenseCovariance, DenseEndpoint, EcefAnchor, EnqueueDisposition, LiveCoreError};
use crate::math::UnitQuaternion;
use crate::quality::{EstimateQuality, ObservabilityReport};
use crate::trajectory::TrajectoryKnot;
use crate::uncertainty::{Covariance3, CrossCovariance3, KinematicCovariance};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion as NaUnitQuaternion, Vector3 as NaVector3};

#[cfg(test)]
#[path = "conversion_tests.rs"]
mod tests;

const EARTH_RATE_RAD_S: f64 = 7.292_115_0e-5;

pub(super) fn trajectory_knot(
    endpoint: DenseEndpoint,
    degraded: bool,
    degraded_input: bool,
    anchor: &EcefAnchor,
    quality: EstimateQuality,
    observability: ObservabilityReport,
) -> Result<TrajectoryKnot, StepError> {
    let projection = project_nav_state(endpoint.state, anchor, quality, observability)?;
    Ok(TrajectoryKnot {
        time: endpoint.state.time,
        position_ecef: projection.position,
        velocity_ecef: projection.velocity,
        orientation_ecef_from_body: projection.orientation_ecef_from_body,
        specific_force_body: BodyVector::from_components([
            endpoint.specific_force_b.x as f64,
            endpoint.specific_force_b.y as f64,
            endpoint.specific_force_b.z as f64,
        ])
        .map_err(StepError::InvalidObservation)?,
        covariance: kinematic_covariance(endpoint.covariance, anchor)?,
        quality: EstimateQuality {
            imu_gap: degraded,
            degraded_input,
            ..quality
        },
        observability,
    })
}

pub(super) fn project_nav_state(
    state: crate::live::NavState,
    anchor: &EcefAnchor,
    quality: EstimateQuality,
    observability: ObservabilityReport,
) -> Result<LiveProjection, StepError> {
    let position = anchor.position_to_ecef(state.position_n);
    let velocity = anchor.vector_to_ecef(state.velocity_n);
    let ecef_from_n = anchor.ecef_to_n.transpose();
    let q_ecef_from_n =
        NaUnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(ecef_from_n));
    let q_n_from_body = state.orientation_n_from_b.cast::<f64>();
    let mut q = q_ecef_from_n * q_n_from_body;
    // The f32 filter's unit quaternion carries f32 rounding error. Normalize
    // the computed f64 rotation before applying the public f64 unit tolerance.
    q.renormalize();
    let raw = q.quaternion();
    let orientation = UnitQuaternion::from_wxyz([raw.w, raw.i, raw.j, raw.k])
        .map_err(StepError::InvalidObservation)?;
    Ok(LiveProjection {
        time: state.time,
        position: EcefPosition::from_components([position.x, position.y, position.z])
            .map_err(StepError::InvalidObservation)?,
        velocity: EcefVelocity::from_components([velocity.x, velocity.y, velocity.z])
            .map_err(StepError::InvalidObservation)?,
        orientation_ecef_from_body: OrientationEcefFromBody::from_quaternion(orientation),
        quality,
        observability,
    })
}

fn kinematic_covariance(
    covariance: DenseCovariance,
    anchor: &EcefAnchor,
) -> Result<KinematicCovariance, StepError> {
    let rotation = anchor.ecef_to_n.transpose().cast::<f32>();
    let position = covariance3_from_na(rotation * covariance.position * rotation.transpose())?;
    let velocity = covariance3_from_na(rotation * covariance.velocity * rotation.transpose())?;
    // The live ESKF uses a right-multiplicative attitude error. Its three
    // coordinates are in the body tangent basis, so changing the local ENU
    // anchor rotates position/velocity errors but not this block.
    let attitude = covariance3_from_na(covariance.attitude)?;
    let cross_matrix = rotation * covariance.position_velocity * rotation.transpose();
    let cross = CrossCovariance3::from_matrix(matrix_to_array(cross_matrix))
        .map_err(StepError::InvalidObservation)?;
    KinematicCovariance::new(position, velocity, Some(cross), attitude)
        .map_err(StepError::InvalidObservation)
}

fn covariance3_from_na(matrix: Matrix3<f32>) -> Result<Covariance3, StepError> {
    Covariance3::from_matrix(matrix_to_array(matrix)).map_err(StepError::InvalidObservation)
}

fn matrix_to_array(matrix: Matrix3<f32>) -> [[f64; 3]; 3] {
    [
        [
            matrix[(0, 0)] as f64,
            matrix[(0, 1)] as f64,
            matrix[(0, 2)] as f64,
        ],
        [
            matrix[(1, 0)] as f64,
            matrix[(1, 1)] as f64,
            matrix[(1, 2)] as f64,
        ],
        [
            matrix[(2, 0)] as f64,
            matrix[(2, 1)] as f64,
            matrix[(2, 2)] as f64,
        ],
    ]
}

pub(super) fn earth_rate_n(anchor: &EcefAnchor) -> Result<NaVector3<f32>, StepError> {
    vector_f32_from_na(anchor.ecef_to_n * NaVector3::new(0.0, 0.0, EARTH_RATE_RAD_S))
}

pub(super) fn rotate_covariance_to_n(
    anchor: &EcefAnchor,
    covariance: [[f64; 3]; 3],
) -> Result<Matrix3<f32>, StepError> {
    let covariance = matrix_f64(covariance);
    matrix_f32_from_na(anchor.ecef_to_n * covariance * anchor.ecef_to_n.transpose())
}

pub(super) fn rotate_cross_to_n(
    anchor: &EcefAnchor,
    covariance: [[f64; 3]; 3],
) -> Result<Matrix3<f32>, StepError> {
    let covariance = matrix_f64(covariance);
    matrix_f32_from_na(anchor.ecef_to_n * covariance * anchor.ecef_to_n.transpose())
}

pub(super) fn add_matrix(left: [[f64; 3]; 3], right: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    core::array::from_fn(|row| {
        core::array::from_fn(|column| left[row][column] + right[row][column])
    })
}

pub(super) fn scale_covariance(
    covariance: [[f64; 3]; 3],
    multiplier: f64,
) -> Result<[[f64; 3]; 3], StepError> {
    if !multiplier.is_finite() || multiplier < 1.0 {
        return Err(StepError::WorkspaceContract);
    }
    Ok(core::array::from_fn(|row| {
        core::array::from_fn(|column| covariance[row][column] * multiplier)
    }))
}

pub(super) fn scale_cross(covariance: [[f64; 3]; 3], multiplier: f64) -> [[f64; 3]; 3] {
    core::array::from_fn(|row| core::array::from_fn(|column| covariance[row][column] * multiplier))
}

fn matrix_f64(value: [[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::from_row_slice(&[
        value[0][0],
        value[0][1],
        value[0][2],
        value[1][0],
        value[1][1],
        value[1][2],
        value[2][0],
        value[2][1],
        value[2][2],
    ])
}

fn matrix_f32(value: [[f64; 3]; 3]) -> Result<Matrix3<f32>, StepError> {
    matrix_f32_from_na(matrix_f64(value))
}

fn matrix_f32_from_na(value: Matrix3<f64>) -> Result<Matrix3<f32>, StepError> {
    if value
        .iter()
        .any(|entry| !entry.is_finite() || entry.abs() > f32::MAX as f64)
    {
        return Err(StepError::InvalidObservation(ValidationError::NonFinite));
    }
    Ok(value.cast::<f32>())
}

pub(super) fn matrix_from_array(value: [[f64; 3]; 3]) -> Result<Matrix3<f32>, StepError> {
    matrix_f32(value)
}

pub(super) fn vector_f32(value: [f64; 3]) -> Result<NaVector3<f32>, StepError> {
    vector_f32_from_na(na_vector(value))
}

fn vector_f32_from_na(value: NaVector3<f64>) -> Result<NaVector3<f32>, StepError> {
    if value
        .iter()
        .any(|entry| !entry.is_finite() || entry.abs() > f32::MAX as f64)
    {
        return Err(StepError::InvalidObservation(ValidationError::NonFinite));
    }
    Ok(value.cast::<f32>())
}

pub(super) fn na_vector(value: [f64; 3]) -> NaVector3<f64> {
    NaVector3::new(value[0], value[1], value[2])
}

pub(super) fn finite_f32(value: f64) -> Result<f32, StepError> {
    if !value.is_finite() || value.abs() > f32::MAX as f64 {
        Err(StepError::InvalidObservation(ValidationError::NonFinite))
    } else {
        Ok(value as f32)
    }
}

pub(super) fn covariance_density(covariance: Covariance3) -> Result<Matrix3<f32>, StepError> {
    let source = covariance.to_matrix();
    let converted = matrix_f32(source)?;
    for row in 0..3 {
        for column in 0..3 {
            if source[row][column] != 0.0 && converted[(row, column)] == 0.0 {
                return Err(StepError::InvalidObservation(
                    ValidationError::InvalidCovariance,
                ));
            }
        }
    }
    Ok(converted)
}

pub(super) fn vector_f32_finite(values: [crate::math::FiniteF64; 3]) -> NaVector3<f32> {
    NaVector3::new(
        values[0].get() as f32,
        values[1].get() as f32,
        values[2].get() as f32,
    )
}

pub(super) fn vector_f32_nonnegative(values: [crate::math::NonNegativeF64; 3]) -> NaVector3<f32> {
    NaVector3::new(
        values[0].get() as f32,
        values[1].get() as f32,
        values[2].get() as f32,
    )
}

pub(super) fn array_f32_nonnegative<const N: usize>(
    values: [crate::math::NonNegativeF64; N],
) -> [f32; N] {
    core::array::from_fn(|index| values[index].get() as f32)
}

pub(super) fn map_core_step_error(error: LiveCoreError) -> StepError {
    match error {
        LiveCoreError::InputClosed => StepError::AlreadyFinishing,
        LiveCoreError::MeasurementQueueRejected(EnqueueDisposition::CapacityExceeded)
        | LiveCoreError::RawImuHistoryFull
        | LiveCoreError::SmoothingHistoryFull
        | LiveCoreError::PredictorHistoryFull => StepError::OutputCapacityExceeded,
        LiveCoreError::MeasurementQueueRejected(EnqueueDisposition::Duplicate) => {
            StepError::WorkspaceContract
        }
        LiveCoreError::MeasurementTimeMismatch
        | LiveCoreError::ImuOverlapOrRegression
        | LiveCoreError::MissingInitialImuSupport
        | LiveCoreError::ImuIntervalTooLong => {
            StepError::InvalidObservation(ValidationError::InvalidTimeSpan)
        }
        _ => StepError::EstimatorFailure,
    }
}
