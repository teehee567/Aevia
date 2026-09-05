//! Rolling causal dense output behind the corrected navigation frontier.

use nalgebra::{ArrayStorage, Matrix3, UnitQuaternion, Vector3};

use crate::time::SessionTime;

use super::{
    reanchor::{ReanchorError, ReanchorTransform},
    state::{ATT, NavMatrix, NavState, POS, VEL, so3_exp, so3_log},
};

/// Compact covariance projection required by continuous public kinematics.
/// The full 15-by-15 filter covariance remains hot state and is not duplicated
/// at every rolling endpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DenseCovariance {
    pub(crate) position: nalgebra::Matrix3<f32>,
    pub(crate) velocity: nalgebra::Matrix3<f32>,
    pub(crate) position_velocity: nalgebra::Matrix3<f32>,
    pub(crate) attitude: nalgebra::Matrix3<f32>,
}

impl DenseCovariance {
    pub(crate) const fn placeholder() -> Self {
        let zero = Matrix3::from_array_storage(ArrayStorage([[0.0; 3]; 3]));
        Self {
            position: zero,
            velocity: zero,
            position_velocity: zero,
            attitude: zero,
        }
    }

    pub(crate) fn from_navigation(covariance: &NavMatrix) -> Self {
        Self {
            position: covariance.fixed_view::<3, 3>(POS, POS).into_owned(),
            velocity: covariance.fixed_view::<3, 3>(VEL, VEL).into_owned(),
            position_velocity: covariance.fixed_view::<3, 3>(POS, VEL).into_owned(),
            attitude: covariance.fixed_view::<3, 3>(ATT, ATT).into_owned(),
        }
    }

    fn mapped(self, transform: &ReanchorTransform) -> Self {
        Self {
            position: transform.map_covariance(self.position),
            velocity: transform.map_covariance(self.velocity),
            position_velocity: transform.new_n_from_old_n
                * self.position_velocity
                * transform.new_n_from_old_n.transpose(),
            // The ESKF uses a right-multiplicative attitude error, expressed
            // in body tangent coordinates.  A left frame rotation therefore
            // does not rotate this block.
            attitude: self.attitude,
        }
    }

    fn is_finite(&self) -> bool {
        self.position
            .iter()
            .chain(self.velocity.iter())
            .chain(self.position_velocity.iter())
            .chain(self.attitude.iter())
            .all(|value| value.is_finite())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DenseEndpoint {
    pub(crate) state: NavState,
    pub(crate) specific_force_b: Vector3<f32>,
    /// Corrected navigation covariance in the endpoint's local error basis.
    /// Predictor-only segments carry the last corrected covariance and never
    /// expose it as a present-time covariance claim.
    pub(crate) covariance: DenseCovariance,
}

impl DenseEndpoint {
    pub(crate) const fn placeholder() -> Self {
        Self {
            state: NavState::placeholder(),
            specific_force_b: Vector3::new(0.0, 0.0, 0.0),
            covariance: DenseCovariance::placeholder(),
        }
    }

    pub(super) fn mapped_reanchor(
        self,
        transform: &ReanchorTransform,
    ) -> Result<Self, ReanchorError> {
        transform.validate_state(self.state)?;
        let covariance = self.covariance.mapped(transform);
        if !covariance.is_finite() {
            return Err(ReanchorError::NonFinite);
        }
        Ok(Self {
            state: transform.map_state(self.state),
            specific_force_b: self.specific_force_b,
            covariance,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DenseSegment {
    pub(crate) id: u64,
    pub(crate) start: DenseEndpoint,
    pub(crate) end: DenseEndpoint,
    attitude_delta: Vector3<f32>,
    duration_seconds: f32,
    pub(crate) degraded: bool,
    pub(crate) degraded_input: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DenseState {
    pub(crate) time: SessionTime,
    pub(crate) position_n: Vector3<f32>,
    pub(crate) velocity_n: Vector3<f32>,
    pub(crate) acceleration_n: Vector3<f32>,
    pub(crate) orientation_n_from_b: UnitQuaternion<f32>,
    pub(crate) angular_rate_body_relative_n: Vector3<f32>,
    pub(crate) angular_acceleration_b: Vector3<f32>,
    pub(crate) specific_force_b: Vector3<f32>,
    pub(crate) degraded: bool,
    pub(crate) degraded_input: bool,
}

impl DenseSegment {
    #[cfg(test)]
    pub(crate) fn new(
        id: u64,
        start: DenseEndpoint,
        end: DenseEndpoint,
        degraded: bool,
        degraded_input: bool,
    ) -> Result<Self, DenseHistoryError> {
        let relative = start.state.orientation_n_from_b.inverse() * end.state.orientation_n_from_b;
        Self::new_imu_conditioned(id, start, end, so3_log(&relative), degraded, degraded_input)
    }

    pub(crate) fn new_imu_conditioned(
        id: u64,
        start: DenseEndpoint,
        end: DenseEndpoint,
        integrated_attitude_delta: Vector3<f32>,
        degraded: bool,
        degraded_input: bool,
    ) -> Result<Self, DenseHistoryError> {
        let duration_ns = end
            .state
            .time
            .as_ns()
            .checked_sub(start.state.time.as_ns())
            .ok_or(DenseHistoryError::TimeOverflow)?;
        if duration_ns <= 0 || !start.state.is_finite() || !end.state.is_finite() {
            return Err(DenseHistoryError::InvalidSegment);
        }
        if start
            .specific_force_b
            .iter()
            .chain(end.specific_force_b.iter())
            .chain(integrated_attitude_delta.iter())
            .any(|value| !value.is_finite())
        {
            return Err(DenseHistoryError::InvalidSegment);
        }
        let duration_seconds = duration_ns as f32 * 1.0e-9;
        if !duration_seconds.is_finite() || duration_seconds <= 0.0 {
            return Err(DenseHistoryError::InvalidSegment);
        }
        let nominal_end = start.state.orientation_n_from_b * so3_exp(integrated_attitude_delta);
        let endpoint_correction =
            so3_log(&(nominal_end.inverse() * end.state.orientation_n_from_b));
        if endpoint_correction.iter().any(|value| !value.is_finite())
            || endpoint_correction.norm() >= core::f32::consts::PI - 1.0e-5
        {
            return Err(DenseHistoryError::InvalidSegment);
        }
        Ok(Self {
            id,
            start,
            end,
            attitude_delta: integrated_attitude_delta,
            duration_seconds,
            degraded,
            degraded_input,
        })
    }

    pub(crate) fn start_time(&self) -> SessionTime {
        self.start.state.time
    }

    pub(crate) fn end_time(&self) -> SessionTime {
        self.end.state.time
    }

    pub(crate) fn integrated_attitude_delta(&self) -> Vector3<f32> {
        self.attitude_delta
    }

    pub(crate) fn state_at(&self, time: SessionTime) -> Result<DenseState, DenseHistoryError> {
        if time < self.start_time() || time > self.end_time() {
            return Err(DenseHistoryError::OutsideSpan);
        }
        let elapsed_ns = time
            .as_ns()
            .checked_sub(self.start_time().as_ns())
            .ok_or(DenseHistoryError::TimeOverflow)?;
        let u = (elapsed_ns as f32 * 1.0e-9 / self.duration_seconds).clamp(0.0, 1.0);
        let u2 = u * u;
        let u3 = u2 * u;
        let dt = self.duration_seconds;

        let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
        let h10 = u3 - 2.0 * u2 + u;
        let h01 = -2.0 * u3 + 3.0 * u2;
        let h11 = u3 - u2;
        let position = self.start.state.position_n * h00
            + self.start.state.velocity_n * (dt * h10)
            + self.end.state.position_n * h01
            + self.end.state.velocity_n * (dt * h11);

        let dh00 = (6.0 * u2 - 6.0 * u) / dt;
        let dh10 = 3.0 * u2 - 4.0 * u + 1.0;
        let dh01 = (-6.0 * u2 + 6.0 * u) / dt;
        let dh11 = 3.0 * u2 - 2.0 * u;
        let velocity = self.start.state.position_n * dh00
            + self.start.state.velocity_n * dh10
            + self.end.state.position_n * dh01
            + self.end.state.velocity_n * dh11;

        let d2h00 = (12.0 * u - 6.0) / (dt * dt);
        let d2h10 = (6.0 * u - 4.0) / dt;
        let d2h01 = (-12.0 * u + 6.0) / (dt * dt);
        let d2h11 = (6.0 * u - 2.0) / dt;
        let acceleration = self.start.state.position_n * d2h00
            + self.start.state.velocity_n * d2h10
            + self.end.state.position_n * d2h01
            + self.end.state.velocity_n * d2h11;

        // Follow the corrected-IMU nominal and distribute only its endpoint
        // error over the interval.  The product curve keeps attitude, body
        // rate, and body angular acceleration mutually consistent even when
        // the nominal and correction rotations do not commute.
        let nominal_end = self.start.state.orientation_n_from_b * so3_exp(self.attitude_delta);
        let endpoint_correction =
            so3_log(&(nominal_end.inverse() * self.end.state.orientation_n_from_b));
        let correction_at_u = so3_exp(endpoint_correction * u);
        let orientation = if u == 0.0 {
            self.start.state.orientation_n_from_b
        } else if u == 1.0 {
            self.end.state.orientation_n_from_b
        } else {
            self.start.state.orientation_n_from_b
                * so3_exp(self.attitude_delta * u)
                * correction_at_u
        };
        let rotated_nominal = correction_at_u.inverse_transform_vector(&self.attitude_delta);
        let normalized_rate = rotated_nominal + endpoint_correction;
        let normalized_acceleration = -endpoint_correction.cross(&rotated_nominal);
        let specific_force =
            self.start.specific_force_b * (1.0 - u) + self.end.specific_force_b * u;
        Ok(DenseState {
            time,
            position_n: position,
            velocity_n: velocity,
            acceleration_n: acceleration,
            orientation_n_from_b: orientation,
            angular_rate_body_relative_n: normalized_rate / dt,
            angular_acceleration_b: normalized_acceleration / (dt * dt),
            specific_force_b: specific_force,
            degraded: self.degraded,
            degraded_input: self.degraded_input,
        })
    }

    fn validate_reanchor(&self, transform: &ReanchorTransform) -> Result<(), ReanchorError> {
        self.start.mapped_reanchor(transform)?;
        self.end.mapped_reanchor(transform)?;
        Ok(())
    }

    fn apply_reanchor(&mut self, transform: &ReanchorTransform) {
        self.start.state = transform.map_state(self.start.state);
        self.end.state = transform.map_state(self.end.state);
        self.start.covariance = self.start.covariance.mapped(transform);
        self.end.covariance = self.end.covariance.mapped(transform);
        // The body-frame nominal delta and the endpoint correction implied by
        // it are invariant to the same left frame rotation at both endpoints.
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct DenseHistory<const N: usize> {
    segments: [Option<DenseSegment>; N],
    head: usize,
    len: usize,
}

impl<const N: usize> DenseHistory<N> {
    pub(crate) const fn new() -> Self {
        Self {
            segments: [None; N],
            head: 0,
            len: 0,
        }
    }

    pub(crate) fn push(&mut self, segment: DenseSegment) -> Result<(), DenseHistoryError> {
        if N == 0 || self.len == N {
            return Err(DenseHistoryError::Capacity);
        }
        if let Some(latest) = self.latest() {
            if latest.end_time() != segment.start_time() || latest.id >= segment.id {
                return Err(DenseHistoryError::Discontinuous);
            }
        }
        let index = (self.head + self.len) % N;
        self.segments[index] = Some(segment);
        self.len += 1;
        Ok(())
    }

    pub(crate) fn oldest(&self) -> Option<&DenseSegment> {
        if self.len == 0 {
            None
        } else {
            self.segments[self.head].as_ref()
        }
    }

    pub(crate) fn latest(&self) -> Option<&DenseSegment> {
        if self.len == 0 || N == 0 {
            None
        } else {
            let index = (self.head + self.len - 1) % N;
            self.segments[index].as_ref()
        }
    }

    pub(crate) fn pop_oldest(&mut self) -> Option<DenseSegment> {
        if self.len == 0 || N == 0 {
            return None;
        }
        let result = self.segments[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        result
    }

    pub(crate) fn discard_ending_at_or_before(&mut self, time: SessionTime) -> usize {
        let mut discarded = 0;
        while self
            .oldest()
            .is_some_and(|segment| segment.end_time() <= time)
        {
            self.pop_oldest();
            discarded += 1;
        }
        discarded
    }

    /// Half-open ownership: a shared endpoint belongs to the segment on its
    /// right; only the final segment owns its terminal endpoint.
    pub(crate) fn state_at(&self, time: SessionTime) -> Result<DenseState, DenseHistoryError> {
        for offset in 0..self.len {
            let index = (self.head + offset) % N;
            let segment = self.segments[index]
                .as_ref()
                .ok_or(DenseHistoryError::Corrupt)?;
            let final_segment = offset + 1 == self.len;
            if time >= segment.start_time()
                && (time < segment.end_time() || (final_segment && time == segment.end_time()))
            {
                return segment.state_at(time);
            }
        }
        Err(DenseHistoryError::OutsideSpan)
    }

    pub(crate) const fn len(&self) -> usize {
        self.len
    }

    pub(crate) const fn available(&self) -> usize {
        N - self.len
    }

    pub(super) fn validate_reanchor(
        &self,
        transform: &ReanchorTransform,
    ) -> Result<(), DenseHistoryError> {
        for offset in 0..self.len {
            let index = (self.head + offset) % N;
            self.segments[index]
                .as_ref()
                .ok_or(DenseHistoryError::Corrupt)?
                .validate_reanchor(transform)
                .map_err(DenseHistoryError::Reanchor)?;
        }
        Ok(())
    }

    /// Applies a transform previously accepted by `validate_reanchor`.
    pub(super) fn apply_reanchor(&mut self, transform: &ReanchorTransform) {
        for offset in 0..self.len {
            let index = (self.head + offset) % N;
            if let Some(segment) = self.segments[index].as_mut() {
                segment.apply_reanchor(transform);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DenseHistoryError {
    InvalidSegment,
    TimeOverflow,
    OutsideSpan,
    Capacity,
    Discontinuous,
    Corrupt,
    Reanchor(ReanchorError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn endpoint(time_ns: i64, position_x: f32, velocity_x: f32) -> DenseEndpoint {
        let mut state = NavState::stationary(SessionTime::from_ns(time_ns));
        state.position_n.x = position_x;
        state.velocity_n.x = velocity_x;
        DenseEndpoint {
            state,
            specific_force_b: Vector3::zeros(),
            covariance: DenseCovariance::from_navigation(&NavMatrix::identity()),
        }
    }

    #[test]
    fn hermite_dense_output_has_matching_endpoint_position_and_velocity() {
        let segment = DenseSegment::new(
            1,
            endpoint(0, 0.0, 1.0),
            endpoint(1_000_000_000, 1.0, 1.0),
            false,
            false,
        )
        .unwrap();
        let start = segment.state_at(SessionTime::ZERO).unwrap();
        let end = segment
            .state_at(SessionTime::from_ns(1_000_000_000))
            .unwrap();
        assert!((start.position_n.x - 0.0).abs() < 1.0e-6);
        assert!((start.velocity_n.x - 1.0).abs() < 1.0e-6);
        assert!((end.position_n.x - 1.0).abs() < 1.0e-6);
        assert!((end.velocity_n.x - 1.0).abs() < 1.0e-6);
        assert!(start.acceleration_n.norm() < 1.0e-6);
    }

    #[test]
    fn imu_nominal_attitude_is_endpoint_conditioned_with_coherent_derivatives() {
        let start = endpoint(0, 0.0, 0.0);
        let integrated = Vector3::new(0.0, 0.0, 0.4);
        let correction = Vector3::new(0.2, 0.0, 0.0);
        let mut end = endpoint(1_000_000_000, 0.0, 0.0);
        end.state.orientation_n_from_b =
            start.state.orientation_n_from_b * so3_exp(integrated) * so3_exp(correction);
        let segment =
            DenseSegment::new_imu_conditioned(1, start, end, integrated, false, false).unwrap();

        let at_start = segment.state_at(SessionTime::ZERO).unwrap();
        let at_end = segment
            .state_at(SessionTime::from_ns(1_000_000_000))
            .unwrap();
        assert!(
            so3_log(&(at_start.orientation_n_from_b.inverse() * start.state.orientation_n_from_b))
                .norm()
                < 1.0e-6
        );
        assert!(
            so3_log(&(at_end.orientation_n_from_b.inverse() * end.state.orientation_n_from_b))
                .norm()
                < 1.0e-6
        );

        let before = segment.state_at(SessionTime::from_ns(499_000_000)).unwrap();
        let middle = segment.state_at(SessionTime::from_ns(500_000_000)).unwrap();
        let after = segment.state_at(SessionTime::from_ns(501_000_000)).unwrap();
        let numerical_rate =
            so3_log(&(before.orientation_n_from_b.inverse() * after.orientation_n_from_b)) / 0.002;
        let numerical_acceleration =
            (after.angular_rate_body_relative_n - before.angular_rate_body_relative_n) / 0.002;
        assert!((middle.angular_rate_body_relative_n - numerical_rate).norm() < 2.0e-4);
        assert!((middle.angular_acceleration_b - numerical_acceleration).norm() < 2.0e-4);

        let endpoint_delta =
            so3_log(&(start.state.orientation_n_from_b.inverse() * end.state.orientation_n_from_b));
        let endpoint_slerp = start.state.orientation_n_from_b * so3_exp(endpoint_delta * 0.5);
        assert!(so3_log(&(endpoint_slerp.inverse() * middle.orientation_n_from_b)).norm() > 1.0e-3);
    }

    #[test]
    fn imu_endpoint_correction_at_pi_cut_is_rejected() {
        let start = endpoint(0, 0.0, 0.0);
        let mut end = endpoint(1_000_000_000, 0.0, 0.0);
        end.state.orientation_n_from_b =
            start.state.orientation_n_from_b * so3_exp(Vector3::x() * core::f32::consts::PI);

        assert!(matches!(
            DenseSegment::new_imu_conditioned(1, start, end, Vector3::zeros(), false, false,),
            Err(DenseHistoryError::InvalidSegment)
        ));
    }

    #[test]
    fn shared_endpoint_is_owned_by_right_segment() {
        let left = DenseSegment::new(
            1,
            endpoint(0, 0.0, 0.0),
            endpoint(10, 1.0, 0.0),
            false,
            false,
        )
        .unwrap();
        let mut right_start = endpoint(10, 2.0, 0.0);
        right_start.specific_force_b.x = 7.0;
        let right =
            DenseSegment::new(2, right_start, endpoint(20, 3.0, 0.0), false, false).unwrap();
        let mut history = DenseHistory::<2>::new();
        history.push(left).unwrap();
        history.push(right).unwrap();
        let at_boundary = history.state_at(SessionTime::from_ns(10)).unwrap();
        assert!((at_boundary.position_n.x - 2.0).abs() < 1.0e-6);
        assert!((at_boundary.specific_force_b.x - 7.0).abs() < 1.0e-6);
    }

    #[test]
    fn full_history_never_silently_overwrites() {
        let one = DenseSegment::new(
            1,
            endpoint(0, 0.0, 0.0),
            endpoint(10, 0.0, 0.0),
            false,
            false,
        )
        .unwrap();
        let two = DenseSegment::new(
            2,
            endpoint(10, 0.0, 0.0),
            endpoint(20, 0.0, 0.0),
            false,
            false,
        )
        .unwrap();
        let mut history = DenseHistory::<1>::new();
        history.push(one).unwrap();
        assert_eq!(history.push(two), Err(DenseHistoryError::Capacity));
        assert_eq!(history.latest().unwrap().id, 1);
    }

    #[test]
    fn discard_releases_only_consumed_segments() {
        let one = DenseSegment::new(
            1,
            endpoint(0, 0.0, 0.0),
            endpoint(10, 0.0, 0.0),
            false,
            false,
        )
        .unwrap();
        let two = DenseSegment::new(
            2,
            endpoint(10, 0.0, 0.0),
            endpoint(20, 0.0, 0.0),
            false,
            false,
        )
        .unwrap();
        let mut history = DenseHistory::<2>::new();
        history.push(one).unwrap();
        history.push(two).unwrap();
        assert_eq!(
            history.discard_ending_at_or_before(SessionTime::from_ns(10)),
            1
        );
        assert_eq!(history.oldest().unwrap().id, 2);
    }
}
