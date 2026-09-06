//! Conditional covariance of the complete linearized inertial process.
//!
//! Static calibration and the held IMU error remain in the endpoint state.
//! Only navigation coordinates constrain the right endpoint: its held sample
//! may already belong to the next support. Angular input uncertainty uses the
//! finite average over this propagation support, never a derivative of white
//! gyro noise.

use super::{
    bridge::{DenseBridgeLinearization, validate_symmetric_psd},
    dense::BaseKinematics,
};
use crate::{
    error::{QueryError, ValidationError},
    frame::{ReferencePoint, ReferencePointKind},
};
use nalgebra::{DMatrix, Matrix3, Vector3};

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoupledDenseBridge {
    pub duration_seconds: f64,
    pub state_dimension: usize,
    pub continuous: DMatrix<f64>,
    pub noise_density: DMatrix<f64>,
    /// Joint covariance of the two final [navigation, consider, sample] states.
    pub endpoint_joint: DMatrix<f64>,
    /// Maps final endpoint errors to the stored propagation reference bases.
    pub start_to_reference: DMatrix<f64>,
    pub end_to_reference: DMatrix<f64>,
    /// Nominal propagation reference used to recover the interior tangent basis.
    pub reference_start_orientation: [f64; 4],
    pub reference_body_rate: [f64; 3],
    pub rate_mapping: DMatrix<f64>,
    pub gyro_density: [[f64; 3]; 3],
    /// Lever-arm identity at each first scalar coordinate (zero elsewhere).
    pub parameter_ids: std::vec::Vec<u64>,
    pub cache: FlowCache,
}

/// Derived data is deliberately absent from equality and record identity.
#[derive(Debug, Default)]
pub(crate) struct FlowCache(std::sync::OnceLock<Result<CompiledFlow, QueryError>>);
impl Clone for FlowCache {
    fn clone(&self) -> Self {
        Self::default()
    }
}
impl PartialEq for FlowCache {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

#[derive(Debug)]
struct CompiledFlow {
    duration: f64,
    squarings: u32,
    transition_terms: std::vec::Vec<DMatrix<f64>>,
    process_terms: std::vec::Vec<DMatrix<f64>>,
    endpoint_transition: DMatrix<f64>,
    endpoint_process: DMatrix<f64>,
    conditional: crate::offline::PsdSolver,
    samples: std::sync::Mutex<std::vec::Vec<(u64, DMatrix<f64>, DMatrix<f64>)>>,
}

impl CompiledFlow {
    fn evaluate_terms(
        duration: f64,
        time: f64,
        squarings: u32,
        phi_terms: &[DMatrix<f64>],
        q_terms: &[DMatrix<f64>],
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let dimension = phi_terms[0].nrows();
        let fraction = time / duration;
        let mut phi = DMatrix::identity(dimension, dimension);
        let mut q = DMatrix::zeros(dimension, dimension);
        let mut power = fraction;
        for term in phi_terms {
            phi += term * power;
            power *= fraction;
        }
        power = fraction;
        for term in q_terms {
            q += term * power;
            power *= fraction;
        }
        for _ in 0..squarings {
            q = &phi * &q * phi.transpose() + q;
            phi = &phi * &phi;
        }
        (phi, (&q + q.transpose()) * 0.5)
    }

    fn evaluate(&self, time: f64) -> Result<(DMatrix<f64>, DMatrix<f64>), QueryError> {
        if !time.is_finite() || time < 0.0 || time > self.duration * (1.0 + 8.0 * f64::EPSILON) {
            return Err(QueryError::InvalidRequest);
        }
        if time == self.duration {
            return Ok((
                self.endpoint_transition.clone(),
                self.endpoint_process.clone(),
            ));
        }
        let mut samples = self
            .samples
            .lock()
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        if let Some((_, phi, q)) = samples.iter().find(|(key, _, _)| *key == time.to_bits()) {
            return Ok((phi.clone(), q.clone()));
        }
        let (phi, q) = Self::evaluate_terms(
            self.duration,
            time,
            self.squarings,
            &self.transition_terms,
            &self.process_terms,
        );
        if samples.len() == 16 {
            samples.remove(0);
        }
        samples.push((time.to_bits(), phi.clone(), q.clone()));
        Ok((phi, q))
    }
}

impl CoupledDenseBridge {
    pub(crate) fn dimension(&self) -> usize {
        self.continuous.nrows()
    }

    pub(crate) fn validate(&self) -> Result<(), ValidationError> {
        let d = self.dimension();
        if !self.duration_seconds.is_finite()
            || self.duration_seconds <= 0.0
            || self.state_dimension < 15
            || self
                .state_dimension
                .checked_add(6)
                .is_none_or(|minimum| d < minimum)
            || self.continuous.shape() != (d, d)
            || self.noise_density.shape() != (d, d)
            || self.start_to_reference.shape() != (d, d)
            || self.end_to_reference.shape() != (d, d)
            || self.endpoint_joint.shape() != (2 * d, 2 * d)
            || self.rate_mapping.shape() != (3, d)
            || self.parameter_ids.len() != d
            || !self
                .continuous
                .iter()
                .chain(self.rate_mapping.iter())
                .chain(self.start_to_reference.iter())
                .chain(self.end_to_reference.iter())
                .chain(self.reference_start_orientation.iter())
                .chain(self.reference_body_rate.iter())
                .all(|value| value.is_finite())
        {
            return Err(ValidationError::InvalidCovariance);
        }
        crate::math::UnitQuaternion::from_wxyz(self.reference_start_orientation)?;
        validate_symmetric_psd(&self.endpoint_joint, true)?;
        validate_symmetric_psd(&self.noise_density, true)?;
        // The retained rate integral shares its gyro noise with attitude.
        // Validate that joint process, not just either diagonal block.
        validate_symmetric_psd(&self.augmented_noise_density(), true)?;
        Ok(())
    }

    fn augmented_noise_density(&self) -> DMatrix<f64> {
        let d = self.dimension();
        let mut q = DMatrix::zeros(d + 3, d + 3);
        q.view_mut((0, 0), (d, d)).copy_from(&self.noise_density);
        // The attitude equation contains -gyro noise, while the retained
        // integral r contains +gyro noise. Their covariance is essential.
        for r in 0..3 {
            for c in 0..3 {
                q[(d + r, d + c)] = self.gyro_density[r][c];
                q[(6 + r, d + c)] = -self.gyro_density[r][c];
                q[(d + c, 6 + r)] = -self.gyro_density[r][c];
            }
        }
        q
    }

    fn compile_flow(&self) -> Result<CompiledFlow, QueryError> {
        let d = self.dimension();
        let mut f = DMatrix::zeros(d + 3, d + 3);
        f.view_mut((0, 0), (d, d)).copy_from(&self.continuous);
        let q = self.augmented_noise_density();
        let norm = f
            .row_iter()
            .map(|row| row.iter().map(|x| x.abs()).sum::<f64>())
            .fold(0.0, f64::max);
        let mut local_dt = self.duration_seconds;
        let mut squarings = 0;
        while norm * local_dt > 0.25 {
            if squarings == 20 {
                return Err(QueryError::TrajectoryInvalid);
            }
            local_dt *= 0.5;
            squarings += 1;
        }
        let mut phi_term = DMatrix::identity(d + 3, d + 3);
        let mut q_term = q * local_dt;
        let mut transition_terms = std::vec::Vec::with_capacity(14);
        let mut process_terms = std::vec![q_term.clone()];
        for order in 1..=14 {
            phi_term = (&phi_term * &f) * (local_dt / f64::from(order));
            transition_terms.push(phi_term.clone());
            q_term = (&f * &q_term + &q_term * f.transpose()) * (local_dt / f64::from(order + 1));
            process_terms.push(q_term.clone());
        }
        let (endpoint_transition, endpoint_process) = CompiledFlow::evaluate_terms(
            self.duration_seconds,
            self.duration_seconds,
            squarings,
            &transition_terms,
            &process_terms,
        );
        let conditional = crate::offline::PsdSolver::new(
            &endpoint_process
                .view((0, 0), (self.state_dimension, self.state_dimension))
                .into_owned(),
        )
        .map_err(|_| QueryError::TrajectoryInvalid)?;
        Ok(CompiledFlow {
            duration: self.duration_seconds,
            squarings,
            transition_terms,
            process_terms,
            endpoint_transition,
            endpoint_process,
            conditional,
            samples: std::sync::Mutex::new(std::vec::Vec::new()),
        })
    }

    fn flow(&self) -> Result<&CompiledFlow, QueryError> {
        self.cache
            .0
            .get_or_init(|| self.compile_flow())
            .as_ref()
            .map_err(|error| *error)
    }

    fn discretize(&self, time: f64) -> Result<(DMatrix<f64>, DMatrix<f64>), QueryError> {
        self.flow()?.evaluate(time)
    }

    fn process_cross(&self, first: f64, second: f64) -> Result<DMatrix<f64>, QueryError> {
        if first > second {
            return self.process_cross(second, first).map(|x| x.transpose());
        }
        let (_, q) = self.discretize(first)?;
        let (phi, _) = self.discretize(second - first)?;
        Ok(q * phi.transpose())
    }

    // Output coordinates are [z(t), integral-of-gyro-noise-over-entire-edge].
    fn output_process_cross(
        &self,
        duration: f64,
        first: f64,
        second: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        let d = self.dimension();
        let mut result = self.process_cross(first, second)?;
        let first_end = self.process_cross(first, duration)?;
        let end_second = self.process_cross(duration, second)?;
        let (_, end) = self.discretize(duration)?;
        result
            .view_mut((0, d), (d, 3))
            .copy_from(&first_end.view((0, d), (d, 3)));
        result
            .view_mut((d, 0), (3, d))
            .copy_from(&end_second.view((d, 0), (3, d)));
        result
            .view_mut((d, d), (3, 3))
            .copy_from(&end.view((d, d), (3, 3)));
        Ok(result)
    }

    pub(crate) fn linearization(
        &self,
        duration: f64,
        parameter: f64,
    ) -> Result<DenseBridgeLinearization, QueryError> {
        if !duration.is_finite() || duration <= 0.0 || !(0.0..=1.0).contains(&parameter) {
            return Err(QueryError::InvalidRequest);
        }
        let d = self.dimension();
        let n = self.state_dimension;
        let (phi, _) = self.discretize(duration * parameter)?;
        let (end_phi, _) = self.discretize(duration)?;
        let cross = self.output_process_cross(duration, duration * parameter, duration)?;
        let rhs = cross.view((0, 0), (d + 3, n)).transpose().into_owned();
        let gain = self
            .flow()?
            .conditional
            .solve(&rhs)
            .map_err(|_| QueryError::TrajectoryInvalid)?
            .transpose();
        let mut prior = phi.view((0, 0), (d + 3, d)).into_owned();
        prior.view_mut((d, 0), (3, d)).fill(0.0);
        let start = (prior - &gain * end_phi.view((0, 0), (n, d))) * &self.start_to_reference;
        let end = &gain * self.end_to_reference.rows(0, n);
        Ok(DenseBridgeLinearization {
            start_jacobian: start,
            end_jacobian: end,
        })
    }

    pub(crate) fn conditional_cross(
        &self,
        duration: f64,
        first: f64,
        second: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        let d = self.dimension();
        let n = self.state_dimension;
        let raw = self.output_process_cross(duration, duration * first, duration * second)?;
        let first_end = self.output_process_cross(duration, duration * first, duration)?;
        let end_second = self.output_process_cross(duration, duration, duration * second)?;
        let solved = self
            .flow()?
            .conditional
            .solve(&end_second.view((0, 0), (n, d + 3)).into_owned())
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        Ok(raw - first_end.view((0, 0), (d + 3, n)) * solved)
    }

    pub(crate) fn output_covariance(
        &self,
        duration: f64,
        parameter: f64,
    ) -> Result<DMatrix<f64>, QueryError> {
        let d = self.dimension();
        let linear = self.linearization(duration, parameter)?;
        let mut h = DMatrix::zeros(d + 3, 2 * d);
        h.view_mut((0, 0), (d + 3, d))
            .copy_from(&linear.start_jacobian);
        h.view_mut((0, d), (d + 3, d))
            .copy_from(&linear.end_jacobian);
        let covariance = &h * &self.endpoint_joint * h.transpose()
            + self.conditional_cross(duration, parameter, parameter)?;
        Ok((&covariance + covariance.transpose()) * 0.5)
    }

    pub(super) fn point_projection(
        &self,
        duration: f64,
        parameter: f64,
        base: &BaseKinematics,
        reference: ReferencePoint,
    ) -> Result<DMatrix<f64>, QueryError> {
        let d = self.dimension();
        let mut h = DMatrix::zeros(9, d + 3);
        for axis in 0..9 {
            h[(axis, axis)] = 1.0;
        }
        let q0 = crate::math::UnitQuaternion::from_wxyz(self.reference_start_orientation)
            .map_err(|_| QueryError::TrajectoryInvalid)?;
        let rotation = |value: [f64; 3]| {
            crate::math::UnitQuaternion::from_rotation_vector(
                crate::math::Vector3::from_components(value)
                    .map_err(|_| QueryError::TrajectoryInvalid)?,
            )
            .map_err(|_| QueryError::TrajectoryInvalid)
        };
        let earth = rotation([0.0, 0.0, -7.292_115_0e-5 * duration * parameter])?;
        let body = rotation(
            self.reference_body_rate
                .map(|rate| rate * duration * parameter),
        )?;
        let reference_orientation = earth.multiply(q0).multiply(body);
        let correction = Vector3::from_column_slice(
            &reference_orientation
                .inverse()
                .multiply(base.orientation)
                .rotation_vector()
                .components(),
        );
        let reset = right_jacobian(correction);
        h.view_mut((6, 6), (3, 3)).copy_from(&reset);
        let lever = Vector3::from_column_slice(&reference.imu_to_point().components_m());
        if lever.norm_squared() == 0.0 && reference.kind() == ReferencePointKind::ImuSensingCenter {
            return Ok(h);
        }
        let rotation = Matrix3::from_row_slice(base.orientation.rotation_matrix().as_flattened());
        let omega = Vector3::from_column_slice(&base.angular_rate_body);
        let position_attitude = -rotation * skew(lever) * reset;
        let velocity_attitude = -rotation * skew(omega.cross(&lever)) * reset;
        h.view_mut((0, 6), (3, 3)).copy_from(&position_attitude);
        h.view_mut((3, 6), (3, 3)).copy_from(&velocity_attitude);
        let rate_to_velocity = -rotation * skew(lever);
        let rate_effect = rate_to_velocity * &self.rate_mapping;
        for row in 0..3 {
            for column in 0..d {
                h[(3 + row, column)] += rate_effect[(row, column)];
            }
        }
        // The white gyro process has a finite support-average, correlated
        // with the same endpoint and interior motion being projected.
        h.view_mut((3, d), (3, 3))
            .copy_from(&(-rate_to_velocity / duration));
        let id = u64::from(reference.parameter_id().get());
        let coordinate = self
            .parameter_ids
            .iter()
            .position(|candidate| *candidate == id);
        if let Some(coordinate) = coordinate {
            if coordinate + 3 > d {
                return Err(QueryError::TrajectoryInvalid);
            }
            for row in 0..3 {
                for column in 0..3 {
                    h[(row, coordinate + column)] += rotation[(row, column)];
                    h[(3 + row, coordinate + column)] += (rotation * skew(omega))[(row, column)];
                }
            }
        } else if !matches!(reference.uncertainty(),crate::uncertainty::MeasurementUncertainty::Provided(covariance) if covariance.to_matrix()==[[0.0;3];3])
        {
            return Err(QueryError::ReferencePointUnavailable);
        }
        Ok(h)
    }
}

pub(super) fn skew(value: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -value.z, value.y, value.z, 0.0, -value.x, -value.y, value.x, 0.0,
    )
}

fn right_jacobian(value: Vector3<f64>) -> Matrix3<f64> {
    let angle = value.norm();
    let cross = skew(value);
    let (a, b) = if angle < 1.0e-5 {
        (
            0.5 - angle * angle / 24.0,
            1.0 / 6.0 - angle * angle / 120.0,
        )
    } else {
        (
            (1.0 - angle.cos()) / angle.powi(2),
            (angle - angle.sin()) / angle.powi(3),
        )
    };
    Matrix3::identity() - cross * a + cross * cross * b
}

#[cfg(test)]
#[path = "coupled_tests.rs"]
mod tests;
