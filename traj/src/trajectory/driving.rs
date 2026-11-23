//! Driving telemetry at the IMU origin on the corrected, fused trajectory.
use super::{
    Trajectory,
    math::{dot, norm, scale, sub, tangent_basis},
};
use crate::{
    error::QueryError,
    ids::ReferencePointId,
    quality::{CovarianceConditioning, Validity},
    time::SessionTime,
};

#[derive(Clone, Copy, Debug)]
pub struct DrivingEstimate {
    pub position_ecef_m: [f64; 3],
    pub course_deg: Option<f64>,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub speed_mps: f64,
    pub heading_deg: Option<f64>,
    /// Gravity-free kinematic acceleration in vehicle forward, left and up axes.
    pub vehicle_g: Option<[f64; 3]>,
    pub validity: Validity,
}

impl Trajectory {
    pub fn driving_estimate(
        &self,
        time: SessionTime,
        point: ReferencePointId,
        mounting_confirmed: bool,
    ) -> Result<DrivingEstimate, QueryError> {
        let state = self.state_at(time, point)?;
        if state.quality.validity == Validity::Invalid || state.quality.imu_gap {
            return Err(QueryError::TrajectoryInvalid);
        }
        let p = state.position.components();
        let v = state.velocity.components();
        let ellipsoid = self.frame.ellipsoid();
        let basis = tangent_basis(p, ellipsoid).map_err(|_| QueryError::TrajectoryInvalid)?;
        let up = basis.up;
        let a = ellipsoid.semi_major_axis_m();
        let f = ellipsoid.inverse_flattening().recip();
        let e2 = f * (2.0 - f);
        let horizontal = sub(v, scale(up, dot(v, up)));
        let rotation = state.orientation_ecef_from_body.quaternion();
        let bearing = |vector| {
            let east = dot(vector, basis.east?);
            let north = dot(vector, basis.north?);
            (libm::hypot(east, north) > f64::EPSILON)
                .then(|| (libm::atan2(east, north).to_degrees() + 360.0) % 360.0)
        };
        let installed = mounting_confirmed && self.attachment.permits_body_axis_quantities();
        let heading_deg = (installed && state.observability.body_axis_quantities_available)
            .then(|| bearing(rotation.rotation_matrix().map(|row| row[0])))
            .flatten();
        let course_deg = state
            .observability
            .course_available
            .then(|| bearing(horizontal))
            .flatten();
        let vehicle_g = if installed
            && self.reference_point_for_query(point)?.kind()
                == crate::frame::ReferencePointKind::ImuSensingCenter
            && state.quality.covariance != CovarianceConditioning::Unavailable
        {
            let up_body = rotation
                .inverse()
                .rotate_vector(
                    crate::math::Vector3::from_components(up)
                        .map_err(|_| QueryError::TrajectoryInvalid)?,
                )
                .components();
            gravity_free_g(
                state.specific_force_body.components(),
                up_body,
                state.covariance.attitude_error().to_matrix(),
            )
        } else {
            None
        };
        Ok(DrivingEstimate {
            position_ecef_m: p,
            course_deg,
            latitude_deg: libm::atan2(up[2], libm::hypot(up[0], up[1])).to_degrees(),
            longitude_deg: libm::atan2(p[1], p[0]).to_degrees(),
            altitude_m: dot(p, up) - a * libm::sqrt(1.0 - e2 * up[2] * up[2]),
            speed_mps: norm(horizontal),
            heading_deg,
            vehicle_g,
            validity: state.quality.validity,
        })
    }
}

/// The gravity direction depends on tilt alone. The live attitude covariance
/// uses right-multiplicative errors in body axes. Project out rotation about
/// gravity before applying the 0.15 rad combined RMS tilt display limit.
fn gravity_free_g(
    force_body: [f64; 3],
    up_body: [f64; 3],
    covariance: [[f64; 3]; 3],
) -> Option<[f64; 3]> {
    let trace = covariance[0][0] + covariance[1][1] + covariance[2][2];
    let tilt_variance = trace - dot(up_body, covariance.map(|row| dot(row, up_body)));
    (tilt_variance <= 0.15 * 0.15)
        .then(|| core::array::from_fn(|i| force_body[i] / 9.80665 - up_body[i]))
}

/// A directed map gate with a finite width. The app's map gates have no surveyed altitude.
#[derive(Clone, Copy, Debug)]
pub struct DrivingGate {
    center: [f64; 3],
    forward: [f64; 3],
    across: [f64; 3],
    half_width: f64,
}

impl DrivingGate {
    pub fn from_wgs84(lat_deg: f64, lon_deg: f64, course_deg: f64, width_m: f64) -> Option<Self> {
        if !(-90.0..=90.0).contains(&lat_deg)
            || !(-180.0..=180.0).contains(&lon_deg)
            || !course_deg.is_finite()
            || !(1.0..=500.0).contains(&width_m)
        {
            return None;
        }
        let (lat, lon, h) = (
            lat_deg.to_radians(),
            lon_deg.to_radians(),
            course_deg.to_radians(),
        );
        let (sl, cl) = (libm::sin(lat), libm::cos(lat));
        let (so, co) = (libm::sin(lon), libm::cos(lon));
        let ellipsoid = crate::frame::ReferenceEllipsoid::WGS84;
        let f = ellipsoid.inverse_flattening().recip();
        let e2 = f * (2.0 - f);
        let n = ellipsoid.semi_major_axis_m() / libm::sqrt(1.0 - e2 * sl * sl);
        let east = [-so, co, 0.0];
        let north = [-sl * co, -sl * so, cl];
        let (sh, ch) = (libm::sin(h), libm::cos(h));
        Some(Self {
            center: [n * cl * co, n * cl * so, n * (1.0 - e2) * sl],
            forward: core::array::from_fn(|i| sh * east[i] + ch * north[i]),
            across: core::array::from_fn(|i| ch * east[i] - sh * north[i]),
            half_width: width_m / 2.0,
        })
    }
}

/// Preliminary driving metrics interpolate adjacent corrected knots. These never
/// enter the qualified dense-metric report stream or claim timing uncertainty.
#[derive(Clone, Copy, Debug, Default)]
pub struct DrivingTotals {
    pub distance_m: f64,
    pub duration_s: f64,
    pub peak_mps: f64,
    pub laps: u32,
    pub lap_start_s: Option<f64>,
    pub last_lap_end_s: Option<f64>,
    pub last_lap_s: Option<f64>,
    pub best_lap_s: Option<f64>,
    pub last_sector_s: Option<f64>,
    pub sector: u8,
    pub launch_s: Option<f64>,
    pub timing_origin_s: Option<f64>,
    pub speed_time_s: Option<f64>,
    pub distance_time_s: Option<f64>,
}

struct DragAttempt {
    launch_s: f64,
    launch_distance_m: f64,
    origin_s: Option<f64>,
    speed_time_s: Option<f64>,
    distance_time_s: Option<f64>,
}

pub struct DrivingRun {
    pub totals: DrivingTotals,
    gates: heapless::Vec<DrivingGate, 16>,
    next_gate: usize,
    gate_armed: bool,
    last_crossing: Option<f64>,
    start_mps: f64,
    end_mps: f64,
    target_m: f64,
    rollout: bool,
    below_since: Option<f64>,
    attempt: Option<DragAttempt>,
    last_end: Option<SessionTime>,
}

impl DrivingRun {
    pub fn new(start_mps: f64, end_mps: f64, target_m: f64, rollout: bool) -> Self {
        Self {
            totals: DrivingTotals::default(),
            gates: heapless::Vec::new(),
            next_gate: 0,
            gate_armed: false,
            last_crossing: None,
            start_mps,
            end_mps,
            target_m,
            rollout,
            below_since: None,
            attempt: None,
            last_end: None,
        }
    }
    pub fn add_gate(&mut self, gate: DrivingGate) -> Result<(), crate::ValidationError> {
        self.gates
            .push(gate)
            .map_err(|_| crate::ValidationError::CapacityExceeded)
    }
    pub fn interrupt(&mut self) {
        self.totals.lap_start_s = None;
        self.totals.sector = 0;
        self.next_gate = 0;
        self.gate_armed = false;
        self.last_crossing = None;
        self.below_since = None;
        self.attempt = None;
        self.last_end = None;
    }
    pub fn advance(
        &mut self,
        path: &Trajectory,
        span: crate::time::TimeSpan,
    ) -> Result<(), QueryError> {
        let (first, _) = path.locate(span.start())?;
        for index in first..path.segment_count() {
            let lease = path.segment_lease(index)?;
            let s = lease.segment();
            if s.start.time >= span.end() {
                break;
            }
            let start = s.start.time.max(span.start());
            let end = s.end.time.min(span.end());
            if self.last_end != Some(start) {
                self.interrupt();
            }
            if [&s.start, &s.end].iter().any(|k| {
                k.quality.validity == Validity::Invalid
                    || k.quality.imu_gap
                    || k.quality.gnss != crate::quality::GnssState::Healthy
            }) {
                self.interrupt();
                continue;
            }
            let endpoint = |time| -> Result<([f64; 3], [f64; 3]), QueryError> {
                if time == s.start.time {
                    return Ok((
                        s.start.position_ecef.components(),
                        s.start.velocity_ecef.components(),
                    ));
                }
                if time == s.end.time {
                    return Ok((
                        s.end.position_ecef.components(),
                        s.end.velocity_ecef.components(),
                    ));
                }
                let point = s.base_kinematics(s.parameter_at(time)?)?;
                Ok((point.position, point.velocity))
            };
            let (position, velocity) = endpoint(start)?;
            let (next_position, next_velocity) = endpoint(end)?;
            let up = tangent_basis(position, path.frame.ellipsoid())
                .map_err(|_| QueryError::TrajectoryInvalid)?
                .up;
            let horizontal = |v| norm(sub(v, scale(up, dot(v, up))));
            self.interval(
                start.as_ns() as f64 * 1e-9,
                (end.as_ns() - start.as_ns()) as f64 * 1e-9,
                position,
                next_position,
                horizontal(velocity),
                horizontal(next_velocity),
            );
            self.last_end = Some(end);
        }
        Ok(())
    }
    /// Apply the same timing rules to successive compact estimator points.
    /// Clip the interval to the board's exact recording boundaries.
    pub fn advance_points(
        &mut self,
        a: crate::engine::driving::DrivingPoint,
        b: crate::engine::driving::DrivingPoint,
        start_us: u64,
        stop_us: Option<u64>,
        support_us: u32,
    ) {
        let dt = b.epoch_us.saturating_sub(a.epoch_us);
        if dt == 0 || crate::engine::driving::bridge_duration_us(dt, support_us).is_none() {
            self.interrupt();
            return;
        }
        let start = a.epoch_us.max(start_us);
        let end = b.epoch_us.min(stop_us.unwrap_or(u64::MAX));
        if end <= start {
            return;
        }
        let at = |t: u64| {
            let f = (t - a.epoch_us) as f64 / dt as f64;
            let p = core::array::from_fn(|k| {
                a.position_ecef_m[k] + f * (b.position_ecef_m[k] - a.position_ecef_m[k])
            });
            (
                p,
                f64::from(a.speed_mps) + f * (f64::from(b.speed_mps) - f64::from(a.speed_mps)),
            )
        };
        let (p, v) = at(start);
        let (q, w) = at(end);
        self.interval(start as f64 * 1e-6, (end - start) as f64 * 1e-6, p, q, v, w);
    }
    fn interval(&mut self, t: f64, dt: f64, p: [f64; 3], q: [f64; 3], v: f64, w: f64) {
        let initial_distance = self.totals.distance_m;
        let distance_at =
            |fraction: f64| initial_distance + dt * fraction * (v + 0.5 * fraction * (w - v));
        let end_distance = distance_at(1.0);
        self.totals.distance_m = end_distance;
        self.totals.duration_s += dt;
        self.totals.peak_mps = self.totals.peak_mps.max(v).max(w);
        if v.max(w) >= 1.0 {
            self.cross_gate(t, dt, p, q);
        }
        // A standing start uses the same 0.5 m/s motion threshold as the
        // receiver measurement plan. Re-arm only after returning below launch.
        let threshold = if self.start_mps == 0.0 {
            0.5
        } else {
            self.start_mps
        };
        if v < threshold {
            let below_since = *self.below_since.get_or_insert(t);
            if w >= threshold {
                let fraction = (threshold - v) / (w - v);
                let launch_s = t + dt * fraction;
                if self.start_mps > 0. || launch_s - below_since >= 0.5 {
                    self.attempt = Some(DragAttempt {
                        launch_s,
                        launch_distance_m: distance_at(fraction),
                        origin_s: (!self.rollout).then_some(launch_s),
                        speed_time_s: None,
                        distance_time_s: None,
                    });
                }
            }
        }
        let falling = v >= threshold && w < threshold;
        let above_until = if falling {
            t + dt * (v - threshold) / (v - w)
        } else {
            t + dt
        };
        if falling {
            self.below_since = Some(above_until);
        } else if w >= threshold {
            self.below_since = None;
        }
        let Some(attempt) = self.attempt.as_mut() else {
            return;
        };
        // Invert the integrated linear velocity without subtracting close roots.
        let cross_distance = |target: f64| {
            let d = target - initial_distance;
            if d < 0.0 || target > end_distance || end_distance <= initial_distance {
                return None;
            }
            let vf = libm::sqrt((v * v + 2.0 * (w - v) * d / dt).max(0.0));
            Some(t + 2.0 * d / (v + vf).max(f64::MIN_POSITIVE))
        };
        if self.rollout && attempt.origin_s.is_none() {
            attempt.origin_s = cross_distance(attempt.launch_distance_m + 0.3048);
        }
        if let Some(origin) = attempt.origin_s {
            if attempt.speed_time_s.is_none() && v < self.end_mps && w >= self.end_mps {
                let elapsed = t + dt * (self.end_mps - v) / (w - v) - origin;
                if elapsed >= 0.0 {
                    attempt.speed_time_s = Some(elapsed);
                }
            }
            if attempt.distance_time_s.is_none() {
                attempt.distance_time_s = cross_distance(attempt.launch_distance_m + self.target_m)
                    .map(|end| end - origin);
            }
        }
        if self.start_mps > 0. || above_until - attempt.launch_s >= 0.2 {
            self.totals.launch_s = Some(attempt.launch_s);
            self.totals.timing_origin_s = attempt.origin_s;
            self.totals.speed_time_s = attempt.speed_time_s;
            self.totals.distance_time_s = attempt.distance_time_s;
        }
        if falling {
            self.attempt = None;
        }
    }
    fn cross_gate(&mut self, t: f64, dt: f64, start: [f64; 3], end: [f64; 3]) {
        if let Some(g) = self.gates.get(self.next_gate) {
            let a = dot(sub(start, g.center), g.forward);
            let b = dot(sub(end, g.center), g.forward);
            self.gate_armed |= a <= -5.0;
            if self.gate_armed && a < 0.0 && b >= 0.0 && b - a >= dt {
                let u = -a / (b - a);
                let cross =
                    core::array::from_fn(|i| start[i] + u * (end[i] - start[i]) - g.center[i]);
                let when = t + dt * u;
                if dot(cross, g.across).abs() <= g.half_width
                    && self.last_crossing.is_none_or(|last| when > last)
                {
                    self.totals.last_sector_s = self.last_crossing.map(|last| when - last);
                    self.totals.sector = self.next_gate as u8;
                    if self.next_gate == 0 {
                        if let Some(start) = self.totals.lap_start_s {
                            let lap = when - start;
                            self.totals.laps += 1;
                            self.totals.last_lap_s = Some(lap);
                            self.totals.last_lap_end_s = Some(when);
                            self.totals.best_lap_s =
                                Some(self.totals.best_lap_s.map_or(lap, |b| b.min(lap)));
                        }
                        self.totals.lap_start_s = Some(when);
                    }
                    self.last_crossing = Some(when);
                    self.next_gate = (self.next_gate + 1) % self.gates.len();
                    self.gate_armed = false;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_yaw_on_a_bank_preserves_signed_vehicle_g() {
        // Gravity has a component along vehicle left on a bank.
        let up = [0., 0.6, 0.8];
        let covariance = core::array::from_fn(|i| {
            core::array::from_fn(|j| 4. * up[i] * up[j] + if i == j { 0.001 } else { 0. })
        });
        let force = up.map(|v| v * 9.80665);
        let resting = gravity_free_g(force, up, covariance).unwrap();
        assert!(resting.iter().all(|x| x.abs() < 1e-10));
        let g = gravity_free_g(
            [force[0] + 0.2 * 9.80665, force[1] - 0.4 * 9.80665, force[2]],
            up,
            covariance,
        )
        .unwrap();
        assert!((g[0] - 0.2).abs() < 1e-10 && (g[1] + 0.4).abs() < 1e-10);
        assert!(gravity_free_g(force, up, [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]).is_none());
    }
    #[test]
    fn rolling_drag_interpolates_crossings_and_excludes_gaps() {
        let mut run = DrivingRun::new(2., 8., 8., false);
        run.interval(0., 2., [0.; 3], [0.; 3], 0., 4.);
        run.interval(2., 3., [0.; 3], [0.; 3], 4., 10.);
        assert_eq!(run.totals.speed_time_s, Some(3.));
        assert_eq!(run.totals.distance_time_s, Some(2.));
        assert_eq!(run.totals.distance_m, 25.);
        let mut interrupted = DrivingRun::new(2., 8., 8., false);
        interrupted.interval(0., 2., [0.; 3], [0.; 3], 0., 4.);
        interrupted.interrupt();
        interrupted.interval(10., 3., [0.; 3], [0.; 3], 4., 10.);
        assert_eq!(interrupted.totals.speed_time_s, None);
        assert_eq!(interrupted.totals.distance_time_s, None);
    }
    #[test]
    fn standing_launch_rejects_brief_motion_and_retains_original_rollout_origin() {
        let mut run = DrivingRun::new(0., 2., 2., true);
        let feed = |run: &mut DrivingRun, t, dt, v, w| run.interval(t, dt, [0.; 3], [0.; 3], v, w);
        feed(&mut run, 0., 1., 0., 0.);
        feed(&mut run, 1., 0.05, 0., 1.);
        feed(&mut run, 1.05, 0.05, 1., 0.);
        assert!(run.totals.launch_s.is_none());
        feed(&mut run, 1.1, 1., 0., 0.);
        feed(&mut run, 2.1, 2., 0., 4.);
        let launch = run.totals.launch_s.unwrap();
        let origin = run.totals.timing_origin_s.unwrap();
        assert!((launch - 2.35).abs() < 1e-9 && origin > launch);
        assert!((origin + run.totals.speed_time_s.unwrap() - 3.1).abs() < 1e-9);
    }
    #[test]
    fn compact_timing_clips_run_edges_and_interpolates_a_short_missing_batch() {
        use crate::engine::driving::DrivingPoint;
        let point = |t: u64| DrivingPoint {
            epoch_us: t,
            position_ecef_m: [0.; 3],
            speed_mps: (t as f64 * 1e-6 * 2.) as f32,
        };
        let mut r = DrivingRun::new(1., 2., 10., false);
        let (start, stop) = (453_000, 1_003_000);
        let mut previous = point(450_000);
        for t in (455_000..=1_010_000).step_by(5000) {
            if t == 500_000 {
                continue;
            }
            let next = point(t);
            r.advance_points(previous, next, start, Some(stop), 5000);
            previous = next;
        }
        assert!((r.totals.speed_time_s.unwrap() - 0.5).abs() < 1e-6);
        assert!((r.totals.duration_s - 0.550).abs() < 1e-8);
        assert!((r.totals.distance_m - (1.003 * 1.003 - 0.453 * 0.453)).abs() < 1e-6);
    }

    #[test]
    fn gate_requires_forward_crossing_inside_width_in_sector_order() {
        let mut run = DrivingRun::new(0., 30., 400., false);
        for x in [0., 30.] {
            run.add_gate(DrivingGate {
                center: [x, 0., 0.],
                forward: [1., 0., 0.],
                across: [0., 1., 0.],
                half_width: 5.,
            })
            .unwrap();
        }
        run.cross_gate(0., 1., [10., 0., 0.], [-10., 0., 0.]);
        run.cross_gate(1., 1., [-10., 8., 0.], [10., 8., 0.]);
        assert_eq!(run.totals.lap_start_s, None);
        run.cross_gate(2., 1., [-10., 0., 0.], [10., 0., 0.]);
        run.cross_gate(3., 1., [-10., 0., 0.], [10., 0., 0.]);
        assert_eq!(run.totals.laps, 0);
        run.cross_gate(4., 1., [20., 0., 0.], [40., 0., 0.]);
        run.cross_gate(8., 1., [-10., 0., 0.], [10., 0., 0.]);
        assert_eq!(run.totals.last_lap_s, Some(6.));
        assert_eq!(run.totals.last_sector_s, Some(4.));
        run.interrupt();
        assert_eq!(run.totals.lap_start_s, None);
        assert_eq!(run.totals.last_lap_end_s, Some(8.5));
    }
}
