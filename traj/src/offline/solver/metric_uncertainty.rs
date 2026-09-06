//! Metric uncertainty projected from smoothed cross-time covariance.

use crate::{
    config::{SharedParameterKind, SharedUncertaintyTreatment},
    frame::{ReferencePoint, ReferencePointKind},
    metric::{
        DistancePlan, EventTimeSensitivity, GateSurveyUncertainty, MetricUncertaintyProvider,
    },
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
    math::{NAVIGATION_DIMENSION, matrix3_from_array, skew, vector3},
    smoothing::{augmented_covariance, joint_covariance},
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
    pub(super) reference_point: crate::ids::ReferencePointId,
    pub(super) independent_survey: Option<(crate::ids::GateId, f64, f64)>,
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
        let result = step.smoothed_sample.as_ref().map_or_else(
            || augmented_covariance(covariance, &self.catalog.covariance),
            |sample| joint_covariance(covariance, sample, &self.catalog.covariance),
        );
        let expected = self.store.dimensions().0
            + self.store.dimensions().1
            + if step.smoothed_sample.is_some() { 6 } else { 0 };
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
        if self.catalog.parameter(point.parameter_id()).is_none()
            && matches!(point.uncertainty(), crate::uncertainty::MeasurementUncertainty::Provided(covariance)
            if covariance.to_matrix() == [[0.0;3];3])
        {
            // A known rigid offset still needs attitude/rate projection, but
            // contributes no additional uncertain lever coordinates.
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
        let mut independent_survey = None;
        if let Some(gate) = event.gate {
            match event.gate_survey_uncertainty {
                GateSurveyUncertainty::Exact | GateSurveyUncertainty::Shared(_) => {}
                GateSurveyUncertainty::Independent(variance) => {
                    independent_survey =
                        Some((gate, variance, event.gate_survey_coefficient_s_per_m));
                }
                GateSurveyUncertainty::Unspecified => {
                    return Err(UnavailableReason::MissingUncertainty);
                }
                GateSurveyUncertainty::UnspecifiedVariance(_) => {
                    return Err(UnavailableReason::MissingCorrelation);
                }
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
        let state_dimension = self.store.dimensions().0;
        let consider_dimension = self.store.dimensions().1;
        let augmented_dimension = self.smoothed_augmented_covariance(indices.start)?.nrows();
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
        let (linearization, has_conditional_process) = match trajectory.dense_point_linearization(
            event.segment_index,
            s,
            event.reference_point,
        ) {
            Ok(linearization) => (linearization, true),
            Err(_)
                if (s == 0.0 || s == 1.0)
                    && point.imu_to_point().components_m() == [0.0; 3]
                    && !trajectory
                        .has_conditional_bridge(event.segment_index)
                        .map_err(|_| UnavailableReason::MissingCorrelation)? =>
            {
                let mut start = DMatrix::zeros(9, 9);
                let mut end = DMatrix::zeros(9, 9);
                if s == 0.0 {
                    start.fill_diagonal(1.0);
                } else {
                    end.fill_diagonal(1.0);
                }
                (
                    crate::trajectory::DenseBridgeLinearization {
                        start_jacobian: start,
                        end_jacobian: end,
                    },
                    false,
                )
            }
            Err(_) => return Err(UnavailableReason::MissingCorrelation),
        };
        let dimension = linearization.start_jacobian.ncols();
        if linearization.start_jacobian.nrows() != 9
            || linearization.end_jacobian.shape() != (9, dimension)
            || (dimension != 9 && dimension != augmented_dimension)
            || dimension > augmented_dimension
        {
            return Err(UnavailableReason::MissingCorrelation);
        }
        let start_projected = linearization.start_jacobian.transpose() * &sensitivity;
        let end_projected = linearization.end_jacobian.transpose() * &sensitivity;
        for coordinate in 0..start_projected.len() {
            start_gradient[coordinate] = start_projected[coordinate];
            end_gradient[coordinate] = end_projected[coordinate];
        }
        if let Some(lever_coordinate) = lever_coordinate.filter(|_| start_projected.len() == 9) {
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
        if let GateSurveyUncertainty::Shared(id) = event.gate_survey_uncertainty {
            let coordinate = self
                .catalog
                .parameter(id)
                .filter(|p| {
                    p.kind == SharedParameterKind::SurveyMetres
                        && p.dimension == 3
                        && p.start
                            .checked_add(3)
                            .is_some_and(|end| end <= consider_dimension)
                        && p.validity.contains(event.time)
                })
                .ok_or(UnavailableReason::MissingCorrelation)?;
            for axis in 0..3 {
                start_gradient[state_dimension + coordinate.start + axis] -=
                    event.state.position[axis];
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
            reference_point: event.reference_point,
            independent_survey,
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
                .dense_point_process_cross(
                    first.segment_index,
                    first.parameter,
                    second.parameter,
                    first.reference_point,
                    second.reference_point,
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
        if let (
            Some((first_gate, first_variance, first_coefficient)),
            Some((second_gate, second_variance, second_coefficient)),
        ) = (first.independent_survey, second.independent_survey)
        {
            if first_gate == second_gate {
                if first_variance != second_variance {
                    return Err(UnavailableReason::MissingCorrelation);
                }
                let survey = first_coefficient * second_coefficient * first_variance;
                value += survey;
                scale += survey.abs();
            }
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
}

impl MetricUncertaintyProvider for OfflineMetricUncertainty<'_> {
    fn event_value_variance(
        &mut self,
        trajectory: &Trajectory,
        event: &EventTimeSensitivity,
        fixed_time: crate::metric::StateSensitivity,
        event_slope: f64,
    ) -> FieldValue<f64> {
        let result = (|| {
            if !event_slope.is_finite() {
                return Err(UnavailableReason::IllConditioned);
            }
            let mut projected = self.event_projection(trajectory, event)?;
            let fixed = self.event_projection(
                trajectory,
                &EventTimeSensitivity {
                    state: fixed_time,
                    gate: None,
                    gate_survey_uncertainty: GateSurveyUncertainty::Exact,
                    gate_survey_coefficient_s_per_m: 0.0,
                    ..*event
                },
            )?;
            for index in 0..2 {
                projected.gradients[index] =
                    &projected.gradients[index] * event_slope + &fixed.gradients[index];
            }
            projected.conditional_process_gradient = projected.conditional_process_gradient
                * event_slope
                + fixed.conditional_process_gradient;
            if let Some((_, _, coefficient)) = &mut projected.independent_survey {
                *coefficient *= event_slope;
            }
            self.projection_cross_covariance(trajectory, &projected, &projected)
        })();
        result.map_or_else(FieldValue::Unavailable, FieldValue::Available)
    }
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
