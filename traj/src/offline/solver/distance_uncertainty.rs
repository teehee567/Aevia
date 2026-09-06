//! Correlated distance uncertainty integrated through the complete dense point model.

use super::{
    estimation::matrix_is_psd,
    initialization::geodetic_north_up,
    math::{ATTITUDE, POSITION, VELOCITY, matrix3_from_array, skew, symmetric, vector3},
    metric_uncertainty::{OfflineMetricUncertainty, SegmentStoreIndices},
};
use crate::{
    config::{SharedParameterKind, SharedUncertaintyTreatment},
    frame::{ReferenceEllipsoid, ReferencePointKind},
    metric::{
        DistancePlan, DistanceQuantity, MetricEvaluationLimits, NumericalWorkBudget, SpeedQuantity,
    },
    quality::UnavailableReason,
    time::TimeSpan,
    trajectory::Trajectory,
    uncertainty::MeasurementUncertainty,
};
use core::ops::AddAssign;
use nalgebra::DVector;

impl OfflineMetricUncertainty<'_> {
    pub(super) fn integrated_distance_variance_m2(
        &mut self,
        trajectory: &Trajectory,
        plan: DistancePlan,
        span: TimeSpan,
    ) -> Result<f64, UnavailableReason> {
        if self.treatment != SharedUncertaintyTreatment::SchmidtConsider {
            return Err(UnavailableReason::MissingCorrelation);
        }
        if !trajectory.span().is_some_and(|available| {
            available.contains(span.start()) && available.contains(span.end())
        }) {
            return Err(UnavailableReason::Gap);
        }
        let reference = self
            .reference_points
            .iter()
            .find(|point| point.id() == plan.reference_point)
            .ok_or(UnavailableReason::MissingUncertainty)?;
        let lever_coordinate = if reference.kind() != ReferencePointKind::ImuSensingCenter
            || reference.imu_to_point().components_m() != [0.0; 3]
        {
            match self.catalog.parameter(reference.parameter_id()) {
                Some(coordinate)
                    if coordinate.kind == SharedParameterKind::LeverArmMetres
                        && coordinate.dimension == 3
                        && coordinate.validity.contains(span.start())
                        && coordinate.validity.contains(span.end()) =>
                {
                    Some(coordinate)
                }
                None if matches!(reference.uncertainty(), MeasurementUncertainty::Provided(covariance) if covariance.to_matrix() == [[0.0; 3]; 3]) => {
                    None
                }
                _ => return Err(UnavailableReason::MissingCorrelation),
            }
        } else {
            None
        };
        let mut propagated_gradient: Option<DVector<f64>> = None;
        let mut variance = 0.0_f64;
        let mut variance_scale = 0.0_f64;
        let mut previous_end: Option<u64> = None;
        let mut covered_until = span.start();
        for segment_index in 0..trajectory.segment_count() {
            let Some((lower, upper)) =
                trajectory.segment_parameter_overlap(segment_index, span.start(), span.end())
            else {
                continue;
            };
            if lower == upper {
                continue;
            }
            let start_time = trajectory
                .time_at_parameter(segment_index, lower)
                .map_err(|_| UnavailableReason::Gap)?;
            let end_time = trajectory
                .time_at_parameter(segment_index, upper)
                .map_err(|_| UnavailableReason::Gap)?;
            if start_time != covered_until {
                return Err(UnavailableReason::Gap);
            }
            let (start, end) = trajectory
                .offline_segment_store_indices(segment_index)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let indices = SegmentStoreIndices { start, end };
            if indices.end != indices.start.saturating_add(1)
                || previous_end.is_some_and(|end| end != indices.start)
            {
                return Err(UnavailableReason::Gap);
            }
            let start_covariance = self.smoothed_augmented_covariance(indices.start)?;
            let end_covariance = self.smoothed_augmented_covariance(indices.end)?;
            let augmented_dimension = start_covariance.nrows();
            let (start_gradient, end_gradient, process_variance) = distance_segment_linearization(
                trajectory,
                segment_index,
                plan,
                lower,
                upper,
                augmented_dimension,
                lever_coordinate.map(|coordinate| self.store.dimensions().0 + coordinate.start),
            )?;
            let previous = propagated_gradient
                .take()
                .unwrap_or_else(|| DVector::zeros(augmented_dimension));
            if previous.len() != augmented_dimension
                || end_covariance.shape() != start_covariance.shape()
            {
                return Err(UnavailableReason::MissingCorrelation);
            }
            let effective = previous + start_gradient;
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
            propagated_gradient = Some(end_gradient + gain.transpose() * effective);
            previous_end = Some(indices.end);
            covered_until = end_time;
        }
        if covered_until != span.end() {
            return Err(UnavailableReason::Gap);
        }
        let terminal = previous_end.ok_or(UnavailableReason::MissingCorrelation)?;
        let propagated_gradient =
            propagated_gradient.ok_or(UnavailableReason::MissingCorrelation)?;
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
    lower: f64,
    upper: f64,
    augmented_dimension: usize,
    legacy_lever_start: Option<usize>,
) -> Result<(DVector<f64>, DVector<f64>, f64), UnavailableReason> {
    let duration = trajectory.segment_duration_seconds(segment_index);
    if !duration.is_finite() || duration <= 0.0 || !(0.0..upper).contains(&lower) || upper > 1.0 {
        return Err(UnavailableReason::MissingCorrelation);
    }
    let mut start_gradient = DVector::zeros(augmented_dimension);
    let mut end_gradient = DVector::zeros(augmented_dimension);
    // Speed-magnitude sensitivities jump at reversals. Resolve those exact
    // support cuts before quadrature, as the mean-distance evaluator does.
    let root_quantity = match plan.quantity {
        DistanceQuantity::BodyLongitudinalAbsolute => Some(SpeedQuantity::BodyLongitudinalSigned),
        DistanceQuantity::Spatial3d => Some(SpeedQuantity::Spatial3d),
        DistanceQuantity::HorizontalPath => Some(SpeedQuantity::InstantaneousHorizontal),
        DistanceQuantity::BodyLongitudinalSigned => None,
    };
    let limits = MetricEvaluationLimits::default();
    let mut root_budget = NumericalWorkBudget::root_only(limits.maximum_root_evaluations);
    let roots = if let Some(quantity) = root_quantity {
        trajectory
            .speed_roots_with_budget(
                segment_index,
                plan.reference_point,
                quantity,
                0.0,
                lower,
                upper,
                limits.absolute_root_tolerance_s,
                limits.value_tolerance,
                &mut root_budget,
            )
            .map_err(|_| UnavailableReason::IllConditioned)?
    } else {
        heapless::Vec::new()
    };
    let mut nodes =
        std::vec::Vec::with_capacity((roots.len() + 1) * DISTANCE_QUADRATURE_PARAMETERS.len());
    let mut panel_start = lower;
    for panel_end in roots
        .iter()
        .copied()
        .filter(|root| *root > lower && *root < upper)
        .chain(core::iter::once(upper))
    {
        let panel_width = panel_end - panel_start;
        if panel_width <= 0.0 {
            return Err(UnavailableReason::IllConditioned);
        }
        for node in 0..DISTANCE_QUADRATURE_PARAMETERS.len() {
            let parameter = panel_start + panel_width * DISTANCE_QUADRATURE_PARAMETERS[node];
            let estimate = trajectory
                .metric_estimate_at_parameter(segment_index, parameter, plan.reference_point)
                .map_err(|_| UnavailableReason::MissingUncertainty)?;
            let sensitivity = distance_speed_sensitivity(
                &estimate,
                plan.quantity,
                trajectory.frame().ellipsoid(),
            )?;
            let weighted =
                sensitivity * (DISTANCE_QUADRATURE_WEIGHTS[node] * duration * panel_width);
            let linearization = trajectory
                .dense_point_linearization(segment_index, parameter, plan.reference_point)
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let dimension = linearization.start_jacobian.ncols();
            if linearization.start_jacobian.nrows() != 9
                || linearization.end_jacobian.shape() != (9, dimension)
                || (dimension != 9 && dimension != augmented_dimension)
                || dimension > augmented_dimension
            {
                return Err(UnavailableReason::MissingCorrelation);
            }
            start_gradient
                .rows_mut(0, dimension)
                .add_assign(linearization.start_jacobian.transpose() * &weighted);
            end_gradient
                .rows_mut(0, dimension)
                .add_assign(linearization.end_jacobian.transpose() * &weighted);
            if let Some(start) = legacy_lever_start.filter(|_| dimension == 9) {
                if start + 3 > augmented_dimension {
                    return Err(UnavailableReason::MissingCorrelation);
                }
                let rotation = matrix3_from_array(
                    estimate
                        .orientation_ecef_from_body
                        .quaternion()
                        .rotation_matrix(),
                );
                let omega = vector3(estimate.angular_rate_body_relative_ecef.components());
                let derivative = rotation.transpose() * weighted.rows(POSITION, 3)
                    + (rotation * skew(&omega)).transpose() * weighted.rows(VELOCITY, 3);
                for axis in 0..3 {
                    start_gradient[start + axis] += derivative[axis];
                }
            }
            nodes.push((parameter, weighted));
        }
        panel_start = panel_end;
    }
    let mut process_variance = 0.0_f64;
    let mut process_variance_scale = 0.0_f64;
    // Subpanels share the same conditioned process; keep every cross term
    // across cuts rather than treating their uncertainty as independent.
    for first in 0..nodes.len() {
        for second in 0..nodes.len() {
            let cross = trajectory
                .dense_point_process_cross(
                    segment_index,
                    nodes[first].0,
                    nodes[second].0,
                    plan.reference_point,
                    plan.reference_point,
                )
                .map_err(|_| UnavailableReason::MissingCorrelation)?;
            let first_gradient = &nodes[first].1;
            let second_gradient = &nodes[second].1;
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
            if !estimate.observability.body_axis_quantities_available {
                return Err(UnavailableReason::Unobservable);
            }
            let rotation = matrix3_from_array(
                estimate
                    .orientation_ecef_from_body
                    .quaternion()
                    .rotation_matrix(),
            );
            let body_velocity = rotation.transpose() * velocity;
            let factor = if quantity == DistanceQuantity::BodyLongitudinalAbsolute {
                if body_velocity.x.abs() <= 1.0e-9 {
                    return Err(UnavailableReason::IllConditioned);
                }
                body_velocity.x.signum()
            } else {
                1.0
            };
            let attitude = nalgebra::Vector3::x().cross(&body_velocity) * factor;
            for axis in 0..3 {
                sensitivity[ATTITUDE + axis] = attitude[axis];
            }
            rotation.column(0).into_owned() * factor
        }
    };
    for axis in 0..3 {
        sensitivity[VELOCITY + axis] = velocity_direction[axis];
    }
    Ok(sensitivity)
}
