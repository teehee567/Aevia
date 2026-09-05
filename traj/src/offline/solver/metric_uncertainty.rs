//! Metric uncertainty projected from smoothed cross-time covariance.

use crate::{
    config::{SharedParameterKind, SharedUncertaintyTreatment},
    frame::{ReferenceEllipsoid, ReferencePoint, ReferencePointKind},
    metric::{DistancePlan, DistanceQuantity, EventTimeSensitivity, MetricUncertaintyProvider},
    offline::store::StateStore,
    quality::{CovarianceConditioning, FieldValue, UnavailableReason},
    time::TimeSpan,
    trajectory::Trajectory,
    uncertainty::KinematicCovariance,
};

use nalgebra::{DMatrix, DVector};

use super::{
    catalog::{ConsiderCatalog, ParameterCoordinate},
    estimation::matrix_is_psd,
    initialization::geodetic_north_up,
    math::{
        ATTITUDE, NAVIGATION_DIMENSION, POSITION, VELOCITY, matrix3_from_array, skew, symmetric,
        vector3,
    },
    smoothing::augmented_covariance,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct SegmentStoreIndices {
    pub(super) start: u64,
    pub(super) end: u64,
}

pub(super) struct EventProjection {
    pub(super) indices: SegmentStoreIndices,
    pub(super) gradients: [DVector<f64>; 2],
    pub(super) segment_index: usize,
    pub(super) parameter: f64,
    pub(super) conditional_process_gradient: DVector<f64>,
    pub(super) has_conditional_process: bool,
}

/// Sequential adapter from the private smoothed state store to the metric
/// uncertainty seam. It retains no session-sized covariance matrix: arbitrary
/// cross-time blocks are recovered by composing the stored backward
/// conditionals between the requested epochs.
pub(super) struct OfflineMetricUncertainty<'a> {
    pub(super) store: &'a mut dyn StateStore,
    pub(super) reference_points: &'a [ReferencePoint],
    pub(super) catalog: &'a ConsiderCatalog,
    pub(super) treatment: SharedUncertaintyTreatment,
}

impl<'a> OfflineMetricUncertainty<'a> {
    pub(super) fn new(
        store: &'a mut dyn StateStore,
        reference_points: &'a [ReferencePoint],
        catalog: &'a ConsiderCatalog,
        treatment: SharedUncertaintyTreatment,
    ) -> Self {
        Self {
            store,
            reference_points,
            catalog,
            treatment,
        }
    }

    pub(super) fn smoothed_augmented_covariance(
        &mut self,
        index: u64,
    ) -> Result<DMatrix<f64>, UnavailableReason> {
        let step = self
            .store
            .get(index)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        let covariance = step
            .smoothed_covariance
            .as_ref()
            .ok_or(UnavailableReason::MissingCorrelation)?;
        let result = augmented_covariance(covariance, &self.catalog.covariance);
        let expected = self.store.dimensions().0 + self.store.dimensions().1;
        if result.shape() != (expected, expected)
            || !result.iter().all(|value| value.is_finite())
            || !matrix_is_psd(&result)
        {
            return Err(UnavailableReason::MissingCorrelation);
        }
        Ok(result)
    }

    /// Returns Cov(z_left, z_right) in final smoothed tangent bases. The walk
    /// is linear in epoch separation and stores only one augmented conditional
    /// per epoch, never an O(N^2) covariance table.
    pub(super) fn augmented_cross_covariance(
        &mut self,
        left: u64,
        right: u64,
    ) -> Result<DMatrix<f64>, UnavailableReason> {
        if left == right {
            return self.smoothed_augmented_covariance(left);
        }
        if left > right {
            return self
                .augmented_cross_covariance(right, left)
                .map(|value| value.transpose());
        }
        let mut cross = self.smoothed_augmented_covariance(right)?;
        for index in (left..right).rev() {
            let step = self
                .store
                .get(index)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let next = self
                .store
                .get(index + 1)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            if !next.connected_from_previous {
                return Err(UnavailableReason::Gap);
            }
            let gain = step
                .smoothed_backward_gain
                .as_ref()
                .ok_or(UnavailableReason::MissingCorrelation)?;
            if gain.ncols() != cross.nrows()
                || gain.nrows() != cross.nrows()
                || !gain.iter().all(|value| value.is_finite())
            {
                return Err(UnavailableReason::MissingCorrelation);
            }
            cross = gain * cross;
            if !cross.iter().all(|value| value.is_finite()) {
                return Err(UnavailableReason::MissingCorrelation);
            }
        }
        Ok(cross)
    }

    pub(super) fn reference_point(
        &self,
        event: &EventTimeSensitivity,
    ) -> Result<(ReferencePoint, Option<ParameterCoordinate>), UnavailableReason> {
        if self.treatment != SharedUncertaintyTreatment::SchmidtConsider {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let point = self
            .reference_points
            .iter()
            .copied()
            .find(|point| point.id() == event.reference_point)
            .ok_or(UnavailableReason::MissingUncertainty)?;
        if point.kind() == ReferencePointKind::ImuSensingCenter
            && point.imu_to_point().components_m() == [0.0; 3]
        {
            // The navigation state is defined at this physical point. Any
            // boresight/calibration influence is already jointly carried in
            // P_xx/P_xc; there is no direct lever-arm derivative to add.
            return Ok((point, None));
        }
        let coordinate = self
            .catalog
            .parameter(point.parameter_id())
            .filter(|coordinate| {
                coordinate.kind == SharedParameterKind::LeverArmMetres
                    && coordinate.dimension == 3
                    && coordinate.validity.contains(event.time)
            })
            .ok_or(UnavailableReason::MissingCorrelation)?;
        Ok((point, Some(coordinate)))
    }

    pub(super) fn event_projection(
        &mut self,
        trajectory: &Trajectory,
        event: &EventTimeSensitivity,
    ) -> Result<EventProjection, UnavailableReason> {
        // A gate survey variance has no stable shared-parameter identity in the
        // current MetricPlan interface. A nonzero value therefore lacks its
        // state/consider and other-survey cross terms and must fail closed.
        if event.gate.is_some() {
            match event.gate_survey_variance_m2 {
                Some(0.0) => {}
                Some(_) => return Err(UnavailableReason::MissingCorrelation),
                None => return Err(UnavailableReason::MissingUncertainty),
            }
        }
        let (start, end) = trajectory
            .offline_segment_store_indices(event.segment_index)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        let indices = SegmentStoreIndices { start, end };
        if indices.end != indices.start.saturating_add(1) {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let start_step = self
            .store
            .get(indices.start)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        let end_step = self
            .store
            .get(indices.end)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        if !end_step.connected_from_previous {
            return Err(UnavailableReason::Gap);
        }
        let start = start_step
            .smoothed
            .as_ref()
            .ok_or(UnavailableReason::MissingCorrelation)?;
        let end = end_step
            .smoothed
            .as_ref()
            .ok_or(UnavailableReason::MissingCorrelation)?;
        let duration = end
            .time
            .checked_duration_since(start.time)
            .ok_or(UnavailableReason::MissingCorrelation)?
            .as_seconds_f64();
        let s = event.parameter;
        if !duration.is_finite() || duration <= 0.0 || !s.is_finite() || !(0.0..=1.0).contains(&s) {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let expected_time = trajectory
            .time_at_parameter(event.segment_index, s)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        if expected_time != event.time || start.time >= end.time {
            return Err(UnavailableReason::MissingCorrelation);
        }

        let (point, lever_coordinate) = self.reference_point(event)?;
        // A nonzero lever makes point velocity depend on the bridge's angular
        // rate process. The present kinematic bridge retains attitude error
        // but not a differentiable gyro-noise-rate state, so that case must
        // stay unavailable rather than omit a covariance term.
        if point.imu_to_point().components_m() != [0.0; 3] {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let state_dimension = self.store.dimensions().0;
        let consider_dimension = self.store.dimensions().1;
        let augmented_dimension = state_dimension + consider_dimension;
        if state_dimension < NAVIGATION_DIMENSION {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let mut start_gradient = DVector::zeros(augmented_dimension);
        let mut end_gradient = DVector::zeros(augmented_dimension);
        let sensitivity = DVector::from_column_slice(&[
            event.state.position[0],
            event.state.position[1],
            event.state.position[2],
            event.state.velocity[0],
            event.state.velocity[1],
            event.state.velocity[2],
            event.state.attitude[0],
            event.state.attitude[1],
            event.state.attitude[2],
        ]);
        let (linearization, has_conditional_process) = trajectory
            .dense_bridge_linearization_at_parameter(event.segment_index, s)
            .map(|value| (value, true))
            .unwrap_or_else(|_| {
                let mut start = DMatrix::zeros(9, 9);
                let mut end = DMatrix::zeros(9, 9);
                let s2 = s * s;
                let s3 = s2 * s;
                let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
                let h10 = s3 - 2.0 * s2 + s;
                let h01 = -2.0 * s3 + 3.0 * s2;
                let h11 = s3 - s2;
                let dv_p0 = (6.0 * s2 - 6.0 * s) / duration;
                let dv_v0 = 3.0 * s2 - 4.0 * s + 1.0;
                let dv_p1 = (-6.0 * s2 + 6.0 * s) / duration;
                let dv_v1 = 3.0 * s2 - 2.0 * s;
                for axis in 0..3 {
                    start[(POSITION + axis, POSITION + axis)] = h00;
                    start[(POSITION + axis, VELOCITY + axis)] = duration * h10;
                    start[(VELOCITY + axis, POSITION + axis)] = dv_p0;
                    start[(VELOCITY + axis, VELOCITY + axis)] = dv_v0;
                    start[(ATTITUDE + axis, ATTITUDE + axis)] = 1.0 - s;
                    end[(POSITION + axis, POSITION + axis)] = h01;
                    end[(POSITION + axis, VELOCITY + axis)] = duration * h11;
                    end[(VELOCITY + axis, POSITION + axis)] = dv_p1;
                    end[(VELOCITY + axis, VELOCITY + axis)] = dv_v1;
                    end[(ATTITUDE + axis, ATTITUDE + axis)] = s;
                }
                (
                    crate::trajectory::DenseBridgeLinearization {
                        start_jacobian: start,
                        end_jacobian: end,
                    },
                    false,
                )
            });
        let start_projected = linearization.start_jacobian.transpose() * &sensitivity;
        let end_projected = linearization.end_jacobian.transpose() * &sensitivity;
        for coordinate in 0..9 {
            start_gradient[coordinate] = start_projected[coordinate];
            end_gradient[coordinate] = end_projected[coordinate];
        }
        if let Some(lever_coordinate) = lever_coordinate {
            if lever_coordinate.start + 3 > consider_dimension {
                return Err(UnavailableReason::MissingCorrelation);
            }
            let estimate = trajectory
                .metric_estimate_at_parameter(
                    event.segment_index,
                    event.parameter,
                    event.reference_point,
                )
                .map_err(|_| UnavailableReason::MissingUncertainty)?;
            let rotation = matrix3_from_array(
                estimate
                    .orientation_ecef_from_body
                    .quaternion()
                    .rotation_matrix(),
            );
            let omega = vector3(estimate.angular_rate_body_relative_ecef.components());
            let lever_sensitivity = rotation.transpose() * vector3(event.state.position)
                + (rotation * skew(&omega)).transpose() * vector3(event.state.velocity);
            for axis in 0..3 {
                start_gradient[state_dimension + lever_coordinate.start + axis] +=
                    lever_sensitivity[axis];
            }
        }
        if !start_gradient.iter().all(|value| value.is_finite())
            || !end_gradient.iter().all(|value| value.is_finite())
        {
            return Err(UnavailableReason::MissingCorrelation);
        }
        Ok(EventProjection {
            indices,
            gradients: [start_gradient, end_gradient],
            segment_index: event.segment_index,
            parameter: s,
            conditional_process_gradient: sensitivity,
            has_conditional_process,
        })
    }

    pub(super) fn projection_cross_covariance(
        &mut self,
        trajectory: &Trajectory,
        first: &EventProjection,
        second: &EventProjection,
    ) -> Result<f64, UnavailableReason> {
        let first_terms = [
            (first.indices.start, &first.gradients[0]),
            (first.indices.end, &first.gradients[1]),
        ];
        let second_terms = [
            (second.indices.start, &second.gradients[0]),
            (second.indices.end, &second.gradients[1]),
        ];
        let mut value = 0.0;
        let mut scale = 0.0;
        for (first_index, first_gradient) in first_terms {
            for (second_index, second_gradient) in second_terms {
                let cross = self.augmented_cross_covariance(first_index, second_index)?;
                let contribution = (first_gradient.transpose() * &cross * second_gradient)[(0, 0)];
                if !contribution.is_finite() {
                    return Err(UnavailableReason::MissingCorrelation);
                }
                value += contribution;
                scale += contribution.abs();
            }
        }
        if first.has_conditional_process
            && second.has_conditional_process
            && first.segment_index == second.segment_index
        {
            let process = trajectory
                .dense_bridge_process_cross_covariance(
                    first.segment_index,
                    first.parameter,
                    second.parameter,
                )
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let contribution = (first.conditional_process_gradient.transpose()
                * process
                * &second.conditional_process_gradient)[(0, 0)];
            if !contribution.is_finite() {
                return Err(UnavailableReason::MissingCorrelation);
            }
            value += contribution;
            scale += contribution.abs();
        }
        if !value.is_finite() {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let tolerance = 1_024.0 * f64::EPSILON * scale;
        if core::ptr::eq(first, second) && value < -tolerance {
            return Err(UnavailableReason::MissingCorrelation);
        }
        Ok(if core::ptr::eq(first, second) {
            value.max(0.0)
        } else {
            value
        })
    }

    pub(super) fn integrated_distance_variance_m2(
        &mut self,
        trajectory: &Trajectory,
        plan: DistancePlan,
        span: TimeSpan,
    ) -> Result<f64, UnavailableReason> {
        if self.treatment != SharedUncertaintyTreatment::SchmidtConsider
            || trajectory.span() != Some(span)
            || matches!(
                plan.quantity,
                DistanceQuantity::BodyLongitudinalSigned
                    | DistanceQuantity::BodyLongitudinalAbsolute
            )
        {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let reference = self
            .reference_points
            .iter()
            .find(|point| point.id() == plan.reference_point)
            .ok_or(UnavailableReason::MissingUncertainty)?;
        if reference.kind() != ReferencePointKind::ImuSensingCenter
            || reference.imu_to_point().components_m() != [0.0; 3]
        {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let augmented_dimension = self.store.dimensions().0 + self.store.dimensions().1;
        let mut propagated_gradient = DVector::zeros(augmented_dimension);
        let mut variance = 0.0_f64;
        let mut variance_scale = 0.0_f64;
        let mut previous_end: Option<u64> = None;
        for segment_index in 0..trajectory.segment_count() {
            let (start, end) = trajectory
                .offline_segment_store_indices(segment_index)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let indices = SegmentStoreIndices { start, end };
            if indices.end != indices.start.saturating_add(1)
                || previous_end.is_some_and(|end| end != indices.start)
            {
                return Err(UnavailableReason::Gap);
            }
            let (start_gradient, end_gradient, process_variance) =
                distance_segment_linearization(trajectory, segment_index, plan)?;
            let mut start_augmented = DVector::zeros(augmented_dimension);
            let mut end_augmented = DVector::zeros(augmented_dimension);
            for coordinate in 0..9 {
                start_augmented[coordinate] = start_gradient[coordinate];
                end_augmented[coordinate] = end_gradient[coordinate];
            }
            let effective = propagated_gradient + start_augmented;
            let start_covariance = self.smoothed_augmented_covariance(indices.start)?;
            let end_covariance = self.smoothed_augmented_covariance(indices.end)?;
            let step = self
                .store
                .get(indices.start)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let gain = step
                .smoothed_backward_gain
                .as_ref()
                .ok_or(UnavailableReason::MissingCorrelation)?;
            if gain.shape() != (augmented_dimension, augmented_dimension) {
                return Err(UnavailableReason::MissingCorrelation);
            }
            let conditional =
                symmetric(&start_covariance - gain * &end_covariance * gain.transpose());
            if !matrix_is_psd(&conditional) {
                return Err(UnavailableReason::MissingCorrelation);
            }
            let eliminated = (effective.transpose() * conditional * &effective)[(0, 0)];
            if !eliminated.is_finite() || !process_variance.is_finite() || process_variance < 0.0 {
                return Err(UnavailableReason::MissingCorrelation);
            }
            let contribution = eliminated.max(0.0) + process_variance;
            variance += contribution;
            variance_scale += contribution.abs();
            propagated_gradient = end_augmented + gain.transpose() * effective;
            previous_end = Some(indices.end);
        }
        let terminal = previous_end.ok_or(UnavailableReason::MissingCorrelation)?;
        let terminal_covariance = self.smoothed_augmented_covariance(terminal)?;
        let terminal_contribution =
            (propagated_gradient.transpose() * terminal_covariance * propagated_gradient)[(0, 0)];
        variance += terminal_contribution;
        variance_scale += terminal_contribution.abs();
        if !variance.is_finite() {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let tolerance = 32_768.0 * f64::EPSILON * variance_scale;
        if variance < -tolerance {
            Err(UnavailableReason::MissingCorrelation)
        } else {
            Ok(variance.max(0.0))
        }
    }
}

impl MetricUncertaintyProvider for OfflineMetricUncertainty<'_> {
    fn kinematic_covariance_at(
        &mut self,
        trajectory: &Trajectory,
        segment_index: usize,
        parameter: f64,
        reference_point: crate::ids::ReferencePointId,
    ) -> Result<KinematicCovariance, UnavailableReason> {
        let estimate = trajectory
            .metric_estimate_at_parameter(segment_index, parameter, reference_point)
            .map_err(|_| UnavailableReason::MissingUncertainty)?;
        if estimate.quality.covariance == CovarianceConditioning::Unavailable {
            return Err(UnavailableReason::MissingCorrelation);
        }
        Ok(estimate.covariance)
    }

    fn event_time_variance_s2(
        &mut self,
        trajectory: &Trajectory,
        event: &EventTimeSensitivity,
    ) -> FieldValue<f64> {
        let result = self
            .event_projection(trajectory, event)
            .and_then(|projection| {
                self.projection_cross_covariance(trajectory, &projection, &projection)
            });
        result.map_or_else(FieldValue::Unavailable, FieldValue::Available)
    }

    fn event_time_cross_covariance_s2(
        &mut self,
        trajectory: &Trajectory,
        first: &EventTimeSensitivity,
        second: &EventTimeSensitivity,
    ) -> FieldValue<f64> {
        let result = self
            .event_projection(trajectory, first)
            .and_then(|first_projection| {
                self.event_projection(trajectory, second)
                    .and_then(|second_projection| {
                        self.projection_cross_covariance(
                            trajectory,
                            &first_projection,
                            &second_projection,
                        )
                    })
            });
        result.map_or_else(FieldValue::Unavailable, FieldValue::Available)
    }

    fn integrated_distance_one_sigma_m(
        &mut self,
        trajectory: &Trajectory,
        plan: DistancePlan,
        span: TimeSpan,
    ) -> FieldValue<f64> {
        self.integrated_distance_variance_m2(trajectory, plan, span)
            .map(f64::sqrt)
            .map_or_else(FieldValue::Unavailable, FieldValue::Available)
    }
}

pub(super) const DISTANCE_QUADRATURE_PARAMETERS: [f64; 8] = [
    0.019_855_071_751_231_884,
    0.101_666_761_293_186_64,
    0.237_233_795_041_835_5,
    0.408_282_678_752_175_1,
    0.591_717_321_247_824_9,
    0.762_766_204_958_164_5,
    0.898_333_238_706_813_4,
    0.980_144_928_248_768_1,
];

pub(super) const DISTANCE_QUADRATURE_WEIGHTS: [f64; 8] = [
    0.050_614_268_145_188_13,
    0.111_190_517_226_687_24,
    0.156_853_322_938_943_65,
    0.181_341_891_689_180_88,
    0.181_341_891_689_180_88,
    0.156_853_322_938_943_65,
    0.111_190_517_226_687_24,
    0.050_614_268_145_188_13,
];

pub(super) fn distance_segment_linearization(
    trajectory: &Trajectory,
    segment_index: usize,
    plan: DistancePlan,
) -> Result<(DVector<f64>, DVector<f64>, f64), UnavailableReason> {
    let duration = trajectory.segment_duration_seconds(segment_index);
    if !duration.is_finite() || duration <= 0.0 {
        return Err(UnavailableReason::MissingCorrelation);
    }
    let mut start_gradient = DVector::zeros(9);
    let mut end_gradient = DVector::zeros(9);
    let mut node_gradients: [Option<DVector<f64>>; 8] = core::array::from_fn(|_| None);
    for node in 0..DISTANCE_QUADRATURE_PARAMETERS.len() {
        let parameter = DISTANCE_QUADRATURE_PARAMETERS[node];
        let estimate = trajectory
            .metric_estimate_at_parameter(segment_index, parameter, plan.reference_point)
            .map_err(|_| UnavailableReason::MissingUncertainty)?;
        let sensitivity =
            distance_speed_sensitivity(&estimate, plan.quantity, trajectory.frame().ellipsoid())?;
        let weighted = sensitivity * (DISTANCE_QUADRATURE_WEIGHTS[node] * duration);
        let linearization = trajectory
            .dense_bridge_linearization_at_parameter(segment_index, parameter)
            .map_err(|_| UnavailableReason::MissingCorrelation)?;
        start_gradient += linearization.start_jacobian.transpose() * &weighted;
        end_gradient += linearization.end_jacobian.transpose() * &weighted;
        node_gradients[node] = Some(weighted);
    }
    let mut process_variance = 0.0_f64;
    let mut process_variance_scale = 0.0_f64;
    for first in 0..DISTANCE_QUADRATURE_PARAMETERS.len() {
        for second in 0..DISTANCE_QUADRATURE_PARAMETERS.len() {
            let cross = trajectory
                .dense_bridge_process_cross_covariance(
                    segment_index,
                    DISTANCE_QUADRATURE_PARAMETERS[first],
                    DISTANCE_QUADRATURE_PARAMETERS[second],
                )
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let first_gradient = node_gradients[first]
                .as_ref()
                .ok_or(UnavailableReason::MissingCorrelation)?;
            let second_gradient = node_gradients[second]
                .as_ref()
                .ok_or(UnavailableReason::MissingCorrelation)?;
            let contribution = (first_gradient.transpose() * cross * second_gradient)[(0, 0)];
            process_variance += contribution;
            process_variance_scale += contribution.abs();
        }
    }
    if !start_gradient
        .iter()
        .chain(end_gradient.iter())
        .all(|value| value.is_finite())
        || !process_variance.is_finite()
    {
        return Err(UnavailableReason::MissingCorrelation);
    }
    let tolerance = 32_768.0 * f64::EPSILON * process_variance_scale;
    if process_variance < -tolerance {
        return Err(UnavailableReason::MissingCorrelation);
    }
    Ok((start_gradient, end_gradient, process_variance.max(0.0)))
}

pub(super) fn distance_speed_sensitivity(
    estimate: &crate::trajectory::KinematicEstimate,
    quantity: DistanceQuantity,
    ellipsoid: ReferenceEllipsoid,
) -> Result<DVector<f64>, UnavailableReason> {
    let position = vector3(estimate.position.components());
    let velocity = vector3(estimate.velocity.components());
    let mut sensitivity = DVector::zeros(9);
    let velocity_direction = match quantity {
        DistanceQuantity::Spatial3d => velocity
            .try_normalize(1.0e-9)
            .ok_or(UnavailableReason::IllConditioned)?,
        DistanceQuantity::HorizontalPath => {
            let (_, up) = geodetic_north_up(position, ellipsoid)
                .map_err(|_| UnavailableReason::IllConditioned)?;
            let horizontal = velocity - up * velocity.dot(&up);
            let direction = horizontal
                .try_normalize(1.0e-9)
                .ok_or(UnavailableReason::IllConditioned)?;
            // Horizontal speed also changes as the ellipsoidal normal rotates
            // with position. Use two converging central differences and fail
            // closed if the local derivative is not numerically stable.
            let step = (position.norm() * f64::EPSILON.sqrt()).max(0.01);
            for axis in 0..3 {
                let derivative = |h: f64| -> Result<f64, UnavailableReason> {
                    let mut positive = position;
                    let mut negative = position;
                    positive[axis] += h;
                    negative[axis] -= h;
                    let (_, positive_up) = geodetic_north_up(positive, ellipsoid)
                        .map_err(|_| UnavailableReason::IllConditioned)?;
                    let (_, negative_up) = geodetic_north_up(negative, ellipsoid)
                        .map_err(|_| UnavailableReason::IllConditioned)?;
                    let positive_horizontal = velocity - positive_up * velocity.dot(&positive_up);
                    let negative_horizontal = velocity - negative_up * velocity.dot(&negative_up);
                    Ok((positive_horizontal.norm() - negative_horizontal.norm()) / (2.0 * h))
                };
                let coarse = derivative(step)?;
                let fine = derivative(step * 0.5)?;
                let scale = coarse.abs().max(fine.abs()).max(1.0e-12);
                if !coarse.is_finite()
                    || !fine.is_finite()
                    || (coarse - fine).abs() > 1.0e-4 * scale + 1.0e-10
                {
                    return Err(UnavailableReason::IllConditioned);
                }
                sensitivity[POSITION + axis] = fine;
            }
            direction
        }
        DistanceQuantity::BodyLongitudinalSigned | DistanceQuantity::BodyLongitudinalAbsolute => {
            return Err(UnavailableReason::MissingCorrelation);
        }
    };
    for axis in 0..3 {
        sensitivity[VELOCITY + axis] = velocity_direction[axis];
    }
    Ok(sensitivity)
}
