//! Dense metric sampling, event roots, and extrema queries.

#[cfg(feature = "offline")]
use super::bridge::DenseBridgeLinearization;
use super::enclosed::RootExpression;
use super::jets::speed_value_jet;
use super::math::{dot, norm, query_to_metric, scale, sub, tangent_basis, vector};
use super::quality::conservative_quality;
use super::roots::{EndpointOwnership, filter_owned_roots, isolate_enclosed_roots_with_budget};
use super::{KinematicEstimate, MAX_SEGMENT_ROOTS, ScalarKinematics, Trajectory};
#[cfg(feature = "offline")]
use crate::error::QueryError;
use crate::ids::ReferencePointId;
use crate::metric::{
    MetricError, NumericalWorkBudget, SpeedQuantity, isolate_polynomial_coefficients_with_budget,
};
use crate::quality::{EstimateQuality, FieldValue};
use crate::time::SessionTime;
use heapless::Vec as FixedVec;
#[cfg(feature = "offline")]
use nalgebra::DMatrix;

impl Trajectory {
    pub(crate) fn segment_duration_seconds(&self, index: usize) -> f64 {
        self.segment_lease(index)
            .map_or(f64::NAN, |lease| lease.segment().duration_seconds)
    }

    pub(crate) fn time_at_parameter(
        &self,
        index: usize,
        parameter: f64,
    ) -> Result<SessionTime, MetricError> {
        self.segment_lease(index)
            .map_err(query_to_metric)?
            .segment()
            .time_at(parameter)
    }

    pub(crate) fn segment_parameter_overlap(
        &self,
        index: usize,
        start: SessionTime,
        end: SessionTime,
    ) -> Option<(f64, f64)> {
        let lease = self.segment_lease(index).ok()?;
        let segment = lease.segment();
        let overlap_start = if start > segment.start.time {
            start
        } else {
            segment.start.time
        };
        let overlap_end = if end < segment.end.time {
            end
        } else {
            segment.end.time
        };
        if overlap_end < overlap_start {
            return None;
        }
        let lower = segment.parameter_at(overlap_start).ok()?;
        let upper = segment.parameter_at(overlap_end).ok()?;
        Some((lower, upper))
    }

    pub(crate) fn scalar_kinematics_at(
        &self,
        time: SessionTime,
        reference_point: ReferencePointId,
    ) -> Result<ScalarKinematics, MetricError> {
        let (index, parameter) = self.locate(time).map_err(query_to_metric)?;
        self.scalar_kinematics_at_parameter(index, parameter, reference_point)
    }

    pub(crate) fn scalar_kinematics_at_parameter(
        &self,
        index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<ScalarKinematics, MetricError> {
        let estimate = self
            .evaluate(index, parameter, reference_point)
            .map_err(query_to_metric)?;
        let position = estimate.position.components();
        let velocity = estimate.velocity.components();
        let acceleration = match estimate.kinematic_acceleration {
            FieldValue::Available(value) => Some(value.components()),
            FieldValue::Unavailable(_) => None,
        };
        let up = tangent_basis(position, self.frame.ellipsoid())?.up;
        let vertical_speed = dot(velocity, up);
        let horizontal_velocity = sub(velocity, scale(up, vertical_speed));
        let body_longitudinal = if estimate.observability.body_axis_quantities_available {
            Some(
                estimate
                    .orientation_ecef_from_body
                    .quaternion()
                    .inverse()
                    .rotate_vector(vector(velocity).map_err(|_| MetricError::NumericalFailure)?)
                    .x(),
            )
        } else {
            None
        };
        Ok(ScalarKinematics {
            time: estimate.time,
            position_ecef_m: position,
            velocity_ecef_mps: velocity,
            acceleration_ecef_mps2: acceleration,
            horizontal_speed_mps: norm(horizontal_velocity),
            vertical_speed_mps: vertical_speed,
            body_longitudinal_speed_mps: body_longitudinal,
            quality: estimate.quality,
        })
    }

    /// Conservatively combines trajectory quality over every dense segment
    /// intersecting the closed query span.
    ///
    /// Interior quality is derived from both endpoints, so a synthetic IMU
    /// bridge or any other degraded interval cannot be hidden by evaluating
    /// only a nominal endpoint.
    pub(crate) fn conservative_quality_over_span(
        &self,
        start: SessionTime,
        end: SessionTime,
    ) -> Result<EstimateQuality, MetricError> {
        if end < start {
            return Err(MetricError::OutsideTrajectory);
        }
        let mut combined = None;
        for index in 0..self.segment_count() {
            let lease = self.segment_lease(index).map_err(query_to_metric)?;
            let segment = lease.segment();
            let Some((lower, upper)) = self.segment_parameter_overlap(index, start, end) else {
                continue;
            };
            let conditional_covariance = {
                #[cfg(feature = "offline")]
                {
                    lease.conditional_bridge().is_some()
                }
                #[cfg(not(feature = "offline"))]
                {
                    false
                }
            };
            let lower_quality = segment.quality_at(lower, conditional_covariance).0;
            let upper_quality = segment.quality_at(upper, conditional_covariance).0;
            let interval_quality = conservative_quality(lower_quality, upper_quality);
            combined = Some(combined.map_or(interval_quality, |prior| {
                conservative_quality(prior, interval_quality)
            }));
        }
        combined.ok_or(MetricError::OutsideTrajectory)
    }

    /// Returns the complete kinematic estimate used by the private metric
    /// uncertainty cursor. In contrast with scalar metric evaluation, callers
    /// must inspect `quality.covariance`: an interior Hermite marginal remains
    /// deliberately unavailable until a conditional bridge supplies its joint
    /// covariance.
    pub(crate) fn metric_estimate_at_parameter(
        &self,
        index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<KinematicEstimate, MetricError> {
        self.evaluate(index, parameter, reference_point)
            .map_err(query_to_metric)
    }

    /// Endpoint Jacobians of one host conditional bridge in the kinematic
    /// ordering `[position, velocity, right attitude error]`.
    #[cfg(all(test, feature = "offline"))]
    pub(crate) fn dense_bridge_linearization_at_parameter(
        &self,
        index: usize,
        parameter: f64,
    ) -> Result<DenseBridgeLinearization, QueryError> {
        let lease = self.segment_lease(index)?;
        let segment = lease.segment();
        if lease.conditional_bridge().is_none() {
            return Err(QueryError::TrajectoryInvalid);
        }
        segment.bridge_endpoint_jacobians(parameter)
    }

    #[cfg(feature = "offline")]
    pub(crate) fn has_conditional_bridge(&self, index: usize) -> Result<bool, QueryError> {
        Ok(self.segment_lease(index)?.conditional_bridge().is_some())
    }

    /// Complete point transform, including attitude/rate/lever correlations.
    #[cfg(feature = "offline")]
    pub(crate) fn dense_point_linearization(
        &self,
        index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
    ) -> Result<DenseBridgeLinearization, QueryError> {
        let lease = self.segment_lease(index)?;
        let segment = lease.segment();
        let bridge = lease
            .conditional_bridge()
            .ok_or(QueryError::TrajectoryInvalid)?;
        let reference = self.reference_point_for_query(reference_point)?;
        if let Some(model) = &bridge.coupled {
            let h = model.point_projection(
                segment.duration_seconds,
                parameter,
                &segment.base_kinematics(parameter)?,
                reference,
            )?;
            let linear = model.linearization(segment.duration_seconds, parameter)?;
            return Ok(DenseBridgeLinearization {
                start_jacobian: &h * linear.start_jacobian,
                end_jacobian: &h * linear.end_jacobian,
            });
        }
        if reference.imu_to_point().components_m() != [0.0; 3] {
            return Err(QueryError::ReferencePointUnavailable);
        }
        segment.bridge_endpoint_jacobians(parameter)
    }

    #[cfg(feature = "offline")]
    pub(crate) fn dense_point_process_cross(
        &self,
        index: usize,
        first: f64,
        second: f64,
        first_point: ReferencePointId,
        second_point: ReferencePointId,
    ) -> Result<DMatrix<f64>, QueryError> {
        let lease = self.segment_lease(index)?;
        let segment = lease.segment();
        let bridge = lease
            .conditional_bridge()
            .ok_or(QueryError::TrajectoryInvalid)?;
        if let Some(model) = &bridge.coupled {
            let first_h = model.point_projection(
                segment.duration_seconds,
                first,
                &segment.base_kinematics(first)?,
                self.reference_point_for_query(first_point)?,
            )?;
            let second_h = model.point_projection(
                segment.duration_seconds,
                second,
                &segment.base_kinematics(second)?,
                self.reference_point_for_query(second_point)?,
            )?;
            return Ok(first_h
                * model.conditional_cross(segment.duration_seconds, first, second)?
                * second_h.transpose());
        }
        if self
            .reference_point_for_query(first_point)?
            .imu_to_point()
            .components_m()
            != [0.0; 3]
            || self
                .reference_point_for_query(second_point)?
                .imu_to_point()
                .components_m()
                != [0.0; 3]
        {
            return Err(QueryError::ReferencePointUnavailable);
        }
        bridge.conditional_process_cross(segment.duration_seconds, first, second)
    }

    /// Conditional process cross-covariance between two points in the same
    /// host bridge. Different process intervals are independent only after
    /// conditioning on their shared optimized endpoints; callers must retain
    /// the endpoint cross-time covariance separately.
    #[cfg(all(test, feature = "offline"))]
    pub(crate) fn dense_bridge_process_cross_covariance(
        &self,
        index: usize,
        first_parameter: f64,
        second_parameter: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        let lease = self.segment_lease(index)?;
        let segment = lease.segment();
        let bridge = lease
            .conditional_bridge()
            .ok_or(QueryError::TrajectoryInvalid)?;
        bridge.conditional_process_cross(
            segment.duration_seconds,
            first_parameter,
            second_parameter,
        )
    }

    #[cfg(test)]
    pub(crate) fn gate_roots(
        &self,
        index: usize,
        reference_point: ReferencePointId,
        center_ecef: [f64; 3],
        normal_ecef: [f64; 3],
        x_tolerance: f64,
        value_tolerance: f64,
    ) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError> {
        let mut budget = NumericalWorkBudget::root_only(self.root_evaluation_budget);
        self.gate_roots_with_budget(
            index,
            reference_point,
            center_ecef,
            normal_ecef,
            x_tolerance,
            value_tolerance,
            &mut budget,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn gate_roots_with_budget(
        &self,
        index: usize,
        reference_point: ReferencePointId,
        center_ecef: [f64; 3],
        normal_ecef: [f64; 3],
        x_tolerance: f64,
        value_tolerance: f64,
        budget: &mut NumericalWorkBudget,
    ) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError> {
        budget.cap_root_evaluations(self.root_evaluation_budget);
        let lease = self.segment_lease(index).map_err(query_to_metric)?;
        let segment = lease.segment();
        let reference = self.reference_point_for_metric(reference_point)?;
        let lever = reference.imu_to_point().components_m();
        let parameter_tolerance = x_tolerance / segment.duration_seconds;
        let ownership = self.segment_endpoint_ownership(index, 0.0, 1.0);
        if lever == [0.0; 3] {
            let roots = isolate_polynomial_coefficients_with_budget(
                segment.plane_polynomial(center_ecef, normal_ecef),
                3,
                0.0,
                1.0,
                parameter_tolerance,
                value_tolerance,
                budget,
            )?;
            return filter_owned_roots(roots, 0.0, 1.0, parameter_tolerance, ownership);
        }
        let expression = RootExpression::new(segment, lever)?;
        isolate_enclosed_roots_with_budget(
            0.0,
            1.0,
            parameter_tolerance,
            value_tolerance,
            ownership,
            budget,
            |lower, upper| expression.gate(lower, upper, center_ecef, normal_ecef),
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn speed_roots_with_budget(
        &self,
        index: usize,
        reference_point: ReferencePointId,
        quantity: SpeedQuantity,
        target_mps: f64,
        lower: f64,
        upper: f64,
        x_tolerance: f64,
        value_tolerance: f64,
        budget: &mut NumericalWorkBudget,
    ) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError> {
        budget.cap_root_evaluations(self.root_evaluation_budget);
        let lease = self.segment_lease(index).map_err(query_to_metric)?;
        let segment = lease.segment();
        if !self.body_axis_metric_outputs_available()
            && matches!(
                quantity,
                SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude
            )
        {
            return Err(MetricError::Unobservable);
        }
        let reference = self.reference_point_for_metric(reference_point)?;
        let lever = reference.imu_to_point().components_m();
        let parameter_tolerance = x_tolerance / segment.duration_seconds;
        let ownership = self.segment_endpoint_ownership(index, lower, upper);
        if matches!(quantity, SpeedQuantity::Spatial3d) && lever == [0.0; 3] {
            let roots = isolate_polynomial_coefficients_with_budget(
                segment.spatial_speed_squared_polynomial(target_mps),
                4,
                lower,
                upper,
                parameter_tolerance,
                value_tolerance.max(target_mps * value_tolerance),
                budget,
            )?;
            return filter_owned_roots(roots, lower, upper, parameter_tolerance, ownership);
        }
        let expression = RootExpression::new(segment, lever)?;
        let equation_tolerance = match quantity {
            SpeedQuantity::BodyLongitudinalSigned => value_tolerance,
            SpeedQuantity::InstantaneousHorizontal
            | SpeedQuantity::Spatial3d
            | SpeedQuantity::BodyLongitudinalMagnitude => {
                value_tolerance.max(target_mps.abs() * value_tolerance * 2.0)
            }
        };
        isolate_enclosed_roots_with_budget(
            lower,
            upper,
            parameter_tolerance,
            equation_tolerance,
            ownership,
            budget,
            |cell_lower, cell_upper| {
                expression.speed(
                    cell_lower,
                    cell_upper,
                    self.frame.ellipsoid(),
                    quantity,
                    Some(target_mps),
                )
            },
        )
    }

    pub(crate) fn speed_slope_at_parameter(
        &self,
        index: usize,
        parameter: f64,
        reference_point: ReferencePointId,
        quantity: SpeedQuantity,
    ) -> Result<f64, MetricError> {
        let lease = self.segment_lease(index).map_err(query_to_metric)?;
        let segment = lease.segment();
        if !self.body_axis_metric_outputs_available()
            && matches!(
                quantity,
                SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude
            )
        {
            return Err(MetricError::Unobservable);
        }
        let reference = self.reference_point_for_metric(reference_point)?;
        let speed = speed_value_jet(
            segment,
            reference.imu_to_point().components_m(),
            self.frame.ellipsoid(),
            quantity,
            parameter,
        )?;
        Ok(speed.derivative / segment.duration_seconds)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn speed_extrema_parameters_with_budget(
        &self,
        index: usize,
        reference_point: ReferencePointId,
        quantity: SpeedQuantity,
        x_tolerance: f64,
        value_tolerance: f64,
        budget: &mut NumericalWorkBudget,
    ) -> Result<FixedVec<f64, MAX_SEGMENT_ROOTS>, MetricError> {
        budget.cap_root_evaluations(self.root_evaluation_budget);
        let lease = self.segment_lease(index).map_err(query_to_metric)?;
        let segment = lease.segment();
        if !self.body_axis_metric_outputs_available()
            && matches!(
                quantity,
                SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude
            )
        {
            return Err(MetricError::Unobservable);
        }
        let reference = self.reference_point_for_metric(reference_point)?;
        let lever = reference.imu_to_point().components_m();
        let parameter_tolerance = x_tolerance / segment.duration_seconds;
        let ownership = self.segment_endpoint_ownership(index, 0.0, 1.0);
        if matches!(quantity, SpeedQuantity::Spatial3d) && lever == [0.0; 3] {
            let squared = segment.spatial_speed_squared_polynomial(0.0);
            let derivative = [
                squared[1],
                2.0 * squared[2],
                3.0 * squared[3],
                4.0 * squared[4],
                0.0,
            ];
            let roots = isolate_polynomial_coefficients_with_budget(
                derivative,
                3,
                0.0,
                1.0,
                parameter_tolerance,
                value_tolerance,
                budget,
            )?;
            return filter_owned_roots(roots, 0.0, 1.0, parameter_tolerance, ownership);
        }
        let expression = RootExpression::new(segment, lever)?;
        isolate_enclosed_roots_with_budget(
            0.0,
            1.0,
            parameter_tolerance,
            value_tolerance,
            ownership,
            budget,
            |cell_lower, cell_upper| {
                expression.speed(
                    cell_lower,
                    cell_upper,
                    self.frame.ellipsoid(),
                    quantity,
                    None,
                )
            },
        )
    }

    pub(super) fn segment_endpoint_ownership(
        &self,
        index: usize,
        lower: f64,
        upper: f64,
    ) -> EndpointOwnership {
        EndpointOwnership {
            // A clipped search starts inside the interval and therefore owns
            // its first candidate. At a dense seam, this is the right-hand
            // interval required by the half-open ownership rule.
            lower: lower >= 0.0,
            upper: upper < 1.0 || index + 1 == self.segment_count(),
        }
    }
}
