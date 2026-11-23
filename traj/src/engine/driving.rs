//! Low-latency GNSS/inertial complementary filter for driving telemetry.
//! Alignment starts at rest. Its level frame has arbitrary yaw until a measured
//! horizontal change in GNSS velocity identifies the corresponding IMU direction.
use crate::{frame::ReferenceEllipsoid, live::EcefAnchor};
use heapless::Deque;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

type Vec3 = Vector3<f32>;
const G: f32 = 9.80665;
// The four-sample adapter can lose part of a batch to a single late read.
pub(crate) fn bridge_duration_us(elapsed: u64, support: u32) -> Option<u32> {
    let support = u64::from(support);
    (elapsed.saturating_add(2) >= support && elapsed.saturating_sub(support) <= 12_000)
        .then(|| elapsed.saturating_sub(support) as u32)
}

const FIX_TIMEOUT_US: u64 = 1_000_000;
const MAX_COAST_US: u64 = 2_000_000;
const MAX_IMU_RECOVERY_GAP_US: u64 = 80_000;
const MAX_FIX_LAG_US: u64 = 300_000;
const MAX_POSITION_SIGMA_M: f32 = 30.;

#[derive(Clone, Copy, Debug)]
pub struct DrivingFix {
    pub position_us: u64,
    pub velocity_us: u64,
    pub position_ecef_m: [f64; 3],
    pub velocity_ecef_mps: [f64; 3],
    pub speed_sigma_mps: f32,
    pub position_sigma_m: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct DrivingPoint {
    pub epoch_us: u64,
    pub position_ecef_m: [f64; 3],
    pub speed_mps: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct DrivingMotion {
    pub point: DrivingPoint,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub height_m: f64,
    pub course_deg: Option<f32>,
    pub heading_deg: Option<f32>,
    pub vehicle_g: Option<[f32; 3]>,
    pub position_degraded: bool,
    pub inertial_only: bool,
}

#[derive(Clone, Copy)]
struct InertialEpoch {
    time: u64,
    velocity: Vec3,
    impulse: Vec3,
}

pub struct DrivingEstimator {
    mount: UnitQuaternion<f32>,
    tilt: Option<UnitQuaternion<f32>>,
    bias: Vec3,
    static_time: f32,
    force_sum: Vec3,
    gyro_sum: Vec3,
    last_imu: u64,
    last_force: Vec3,
    last_gyro: Vec3,
    pub bridged_intervals: u64,
    pub recovered_imu_gaps: u64,
    pub lost_alignment: u64,
    needs_aid_after: Option<u64>,
    anchor: Option<EcefAnchor>,
    position: Vector3<f64>,
    velocity: Vec3,
    impulse: Vec3,
    body_g: Vec3,
    yaw: Option<f32>,
    alignment: Option<(u64, Vec3, Vec3)>,
    fixes: Option<(u64, u64, Vec3, f32)>,
    position_fix_us: Option<u64>,
    tilt_reference: Option<(u64, Vec3, Vec3)>,
    history: Deque<InertialEpoch, 96>,
    still: bool,
}

impl DrivingEstimator {
    pub fn new(body_from_sensor_wxyz: [f32; 4]) -> Option<Self> {
        let [w, x, y, z] = body_from_sensor_wxyz;
        let q = Quaternion::new(w, x, y, z);
        if !q.norm_squared().is_finite() || (q.norm_squared() - 1.).abs() > 0.001 {
            return None;
        }
        Some(Self {
            mount: UnitQuaternion::new_normalize(q),
            tilt: None,
            bias: Vec3::zeros(),
            static_time: 0.,
            force_sum: Vec3::zeros(),
            gyro_sum: Vec3::zeros(),
            last_imu: 0,
            last_force: Vec3::zeros(),
            last_gyro: Vec3::zeros(),
            bridged_intervals: 0,
            recovered_imu_gaps: 0,
            lost_alignment: 0,
            needs_aid_after: None,
            anchor: None,
            position: Vector3::zeros(),
            velocity: Vec3::zeros(),
            impulse: Vec3::zeros(),
            body_g: Vec3::zeros(),
            yaw: None,
            alignment: None,
            fixes: None,
            position_fix_us: None,
            tilt_reference: None,
            history: Deque::new(),
            still: false,
        })
    }

    fn restart_alignment(&mut self) {
        if self.tilt.is_some() {
            self.lost_alignment += 1;
        }
        self.tilt = None;
        self.yaw = None;
        self.alignment = None;
        self.tilt_reference = None;
        self.history.clear();
        self.static_time = 0.;
        self.force_sum.fill(0.);
        self.gyro_sum.fill(0.);
        self.velocity.fill(0.);
        self.impulse.fill(0.);
        self.still = false;
        self.needs_aid_after = None;
    }

    /// A GNSS clock remap invalidates old aiding epochs. The IMU attitude and
    /// its history are already on the local clock and remain usable.
    pub fn reset_gnss_clock(&mut self) {
        self.fixes = None;
        self.position_fix_us = None;
        self.alignment = None;
        self.tilt_reference = None;
    }

    /// Input is interval-average specific force and angular rate at a common effective epoch.
    pub fn observe_imu(
        &mut self,
        end_us: u64,
        duration_us: u32,
        force: [f32; 3],
        rate: [f32; 3],
    ) -> bool {
        if end_us <= self.last_imu
            || !(1..=20_000).contains(&duration_us)
            || !force.iter().chain(&rate).all(|v| v.is_finite())
        {
            return false;
        }
        let elapsed = end_us - self.last_imu;
        let gap = if self.last_imu == 0 {
            Some(0)
        } else {
            bridge_duration_us(elapsed, duration_us)
        };
        if gap.is_none() {
            if self.tilt.is_some()
                && (u64::from(duration_us)..=MAX_IMU_RECOVERY_GAP_US).contains(&elapsed)
            {
                // Skip the unsupported interval. Retain the last attitude
                // and wait for fresh Doppler before publishing fused motion.
                self.alignment = None;
                self.tilt_reference = None;
                self.history.clear();
                self.needs_aid_after = Some(end_us);
                self.still = false;
            } else {
                self.restart_alignment();
            }
        }
        self.last_imu = end_us;
        let dt = duration_us as f32 * 1e-6;
        let force = self.mount * Vec3::from(force);
        let gyro = self.mount * Vec3::from(rate);
        let previous_force = core::mem::replace(&mut self.last_force, force);
        let previous_gyro = core::mem::replace(&mut self.last_gyro, gyro);
        let stopped = self.fixes.is_some_and(|(_, vtime, v, sigma)| {
            end_us.abs_diff(vtime) < 250_000
                && sigma < 0.6
                && v.norm() < (2.5 * sigma).clamp(0.22, 0.65)
        });
        let quiet = stopped && (force.norm() - G).abs() < 0.3 && gyro.norm() < 0.04;
        let Some(mut attitude) = self.tilt else {
            if quiet {
                self.force_sum += force * dt;
                self.gyro_sum += gyro * dt;
                self.static_time += dt;
                if self.static_time >= 0.5 {
                    let direction = self.force_sum.normalize();
                    self.tilt = Some(
                        UnitQuaternion::rotation_between(&direction, &Vec3::z()).unwrap_or_else(
                            || {
                                UnitQuaternion::from_axis_angle(
                                    &Vec3::x_axis(),
                                    core::f32::consts::PI,
                                )
                            },
                        ),
                    );
                    self.bias = self.gyro_sum / self.static_time;
                }
            } else {
                self.force_sum.fill(0.);
                self.gyro_sum.fill(0.);
                self.static_time = 0.;
            }
            return true;
        };
        // Integrate the mean of the two neighboring measurements across a short
        // unsupported interval. Preserve the measured force for the G display.
        let (dt, mean_force, mean_gyro) = if let Some(us @ 3..) = gap {
            self.bridged_intervals += 1;
            let missing = us as f32 * 1e-6;
            let weight = 0.5 * missing / (dt + missing);
            (
                dt + missing,
                force + (previous_force - force) * weight,
                gyro + (previous_gyro - gyro) * weight,
            )
        } else {
            (dt, force, gyro)
        };
        let rate = mean_gyro - self.bias;
        let half = UnitQuaternion::from_scaled_axis(rate * (dt * 0.5));
        let mid = attitude * half;
        let acceleration = mid * mean_force - Vec3::z() * G;
        attitude = mid * half;
        self.still = quiet && acceleration.norm() < 0.18 && self.velocity.norm() < 0.5;
        if self.still {
            let predicted = attitude.inverse_transform_vector(&Vec3::z());
            attitude *=
                UnitQuaternion::from_scaled_axis(force.normalize().cross(&predicted) * (dt * 0.35));
            self.bias += (gyro - self.bias) * (dt * 0.1);
        }
        attitude.renormalize();
        self.tilt = Some(attitude);
        self.body_g = force / G - attitude.inverse_transform_vector(&Vec3::z());
        let before = self.velocity;
        if self.still {
            self.velocity.fill(0.);
        } else {
            self.velocity += acceleration * dt;
        }
        self.impulse += acceleration * dt;
        if let (Some(anchor), Some(yaw)) = (&self.anchor, self.yaw) {
            self.position +=
                anchor.vector_to_ecef(turn((before + self.velocity) * 0.5, yaw)) * f64::from(dt);
        }
        if self.history.is_full() {
            self.history.pop_front();
        }
        let _ = self.history.push_back(InertialEpoch {
            time: end_us,
            velocity: self.velocity,
            impulse: self.impulse,
        });
        true
    }

    fn inertial_at(&self, at: u64) -> Option<InertialEpoch> {
        let mut after = *self.history.back()?;
        if at >= after.time {
            return (at - after.time <= 20_000).then_some(after);
        }
        for before in self.history.iter().rev() {
            if before.time <= at {
                let f = (at - before.time) as f32 / (after.time - before.time) as f32;
                return Some(InertialEpoch {
                    time: at,
                    velocity: before.velocity.lerp(&after.velocity, f),
                    impulse: before.impulse.lerp(&after.impulse, f),
                });
            }
            after = *before;
        }
        None
    }

    pub fn observe_fix(&mut self, fix: DrivingFix) -> bool {
        if !fix
            .position_ecef_m
            .iter()
            .chain(&fix.velocity_ecef_mps)
            .all(|x| x.is_finite())
            || !fix.speed_sigma_mps.is_finite()
            || !(0.0..=3.).contains(&fix.speed_sigma_mps)
            || !fix.position_sigma_m.is_finite()
            || fix.position_sigma_m < 0.
            || fix.velocity_us > fix.position_us
            || fix.position_us - fix.velocity_us > MAX_FIX_LAG_US
            || self.last_imu.saturating_sub(fix.velocity_us) > MAX_FIX_LAG_US
            || self
                .fixes
                .is_some_and(|(p, v, _, _)| fix.position_us <= p || fix.velocity_us <= v)
        {
            return false;
        }
        let pos = Vector3::from(fix.position_ecef_m);
        if !(6_300_000.0..=6_500_000.0).contains(&pos.norm()) {
            return false;
        }
        let Some(mut anchor) = self
            .anchor
            .or_else(|| EcefAnchor::from_origin(0, pos, ReferenceEllipsoid::WGS84).ok())
        else {
            return false;
        };
        let mut v = anchor.vector_from_ecef(Vector3::from(fix.velocity_ecef_mps));
        let previous = self.fixes;
        let elapsed = previous.map_or(0.02, |(_, t, _, _)| (fix.velocity_us - t) as f32 * 1e-6);
        let dt = elapsed.min(1.);
        if let Some((_, _, old_v, old_sigma)) = previous {
            if elapsed <= 1.
                && (v - old_v).norm() > 30. * dt + 4. * (old_sigma + fix.speed_sigma_mps).max(0.25)
            {
                return false;
            }
            if elapsed > 1. {
                self.alignment = None;
                self.tilt_reference = None;
                if let Some(yaw) = self.yaw {
                    self.history.clear();
                    self.velocity = turn(v, -yaw);
                }
                // Before yaw is known, the IMU still tracks tilt and velocity
                // in its level frame. A gap in IMU support resets them in observe_imu.
            }
        }
        // The receiver reports independent position and Doppler uncertainties.
        // Continue aiding velocity through position-only degradation. Position
        // stays on the propagated path and timing cannot use that interval.
        let position_age = self.position_fix_us.map(|time| fix.position_us - time);
        let position_recent = position_age.is_some_and(|age| age <= FIX_TIMEOUT_US);
        let accept_position = fix.position_sigma_m <= MAX_POSITION_SIGMA_M
            && (!position_recent
                || (pos - self.position).norm()
                    <= f64::from(
                        (v.norm() + 10.) * (dt + 0.3) + (6. * fix.position_sigma_m).max(25.),
                    ));
        // Keep horizontal speed and gravity local on longer drives. Re-express
        // the integrated state in the new ENU basis at the same physical point.
        if accept_position
            && (pos - anchor.origin_ecef_m).norm_squared() > 4_000_000.
            && let Some(yaw) = self.yaw
            && let Ok(next) = EcefAnchor::from_origin(0, pos, ReferenceEllipsoid::WGS84)
        {
            let basis = (next.ecef_to_n * anchor.ecef_to_n.transpose()).cast::<f32>();
            let rotation = UnitQuaternion::from_rotation_matrix(
                &nalgebra::Rotation3::from_matrix_unchecked(basis),
            ) * UnitQuaternion::from_axis_angle(&Vec3::z_axis(), yaw);
            self.tilt = self.tilt.map(|q| rotation * q);
            self.velocity = rotation * self.velocity;
            for epoch in &mut self.history {
                epoch.velocity = rotation * epoch.velocity;
                epoch.impulse = rotation * (epoch.impulse - self.impulse);
            }
            self.impulse.fill(0.);
            self.alignment = None;
            self.tilt_reference = None;
            self.yaw = Some(0.);
            anchor = next;
            v = anchor.vector_from_ecef(Vector3::from(fix.velocity_ecef_mps));
        }
        let at = self.inertial_at(fix.velocity_us);
        if let Some(sample) = at {
            // Learn yaw from vector changes, including a reversing launch. A
            // constant course alone provides no evidence of vehicle orientation.
            match self.alignment {
                Some((t, old_v, old_impulse)) if fix.velocity_us - t <= 1_500_000 => {
                    let inertial = sample.impulse - old_impulse;
                    let satellite = v - old_v;
                    let a = inertial.xy().norm();
                    let b = satellite.xy().norm();
                    if fix.velocity_us - t >= 250_000
                        && a > 0.8
                        && b > (4. * fix.speed_sigma_mps).max(0.8)
                        && (a - b).abs() < 0.45 * a
                    {
                        let measured = libm::atan2f(
                            inertial.x * satellite.y - inertial.y * satellite.x,
                            inertial.x * satellite.x + inertial.y * satellite.y,
                        );
                        self.yaw = Some(
                            self.yaw
                                .map_or(measured, |yaw| yaw + 0.12 * wrap(measured - yaw)),
                        );
                        self.alignment = Some((fix.velocity_us, v, sample.impulse));
                    }
                }
                _ => self.alignment = Some((fix.velocity_us, v, sample.impulse)),
            }
            // A persistent horizontal difference between measured and integrated
            // delta-velocity identifies gravity leaking through the tilt estimate.
            if let (Some(yaw), Some(attitude)) = (self.yaw, self.tilt) {
                match self.tilt_reference {
                    Some((t, old_v, old_impulse)) if fix.velocity_us - t >= 500_000 => {
                        let residual = turn(v - old_v, -yaw) - (sample.impulse - old_impulse);
                        let span = (fix.velocity_us - t) as f32 * 1e-6;
                        if !self.still && span < 1.5 && residual.xy().norm() < 3. * span {
                            let correction = Vec3::z().cross(&residual) * (0.12 / G);
                            self.tilt =
                                Some(UnitQuaternion::from_scaled_axis(correction) * attitude);
                        }
                        self.tilt_reference = Some((fix.velocity_us, v, sample.impulse));
                    }
                    None => self.tilt_reference = Some((fix.velocity_us, v, sample.impulse)),
                    _ => {}
                }
            }
            let recovering = self
                .needs_aid_after
                .is_some_and(|after| fix.velocity_us >= after);
            if recovering {
                self.needs_aid_after = None;
                self.recovered_imu_gaps += 1;
            }
            let gain = if recovering {
                1.
            } else {
                dt / (dt + (0.12 + fix.speed_sigma_mps).clamp(0.15, 1.5))
            };
            if !self.still {
                let residual = if let Some(yaw) = self.yaw {
                    turn(v, -yaw) - sample.velocity
                } else {
                    let predicted = sample.velocity.xy();
                    let correction = predicted.try_normalize(0.02).unwrap_or_default()
                        * (v.xy().norm() - predicted.norm());
                    Vec3::new(correction.x, correction.y, v.z - sample.velocity.z)
                };
                let correction = residual * gain;
                self.velocity += correction;
                // Later delayed innovations must see corrections already applied.
                for epoch in self
                    .history
                    .iter_mut()
                    .rev()
                    .take_while(|e| e.time >= fix.velocity_us)
                {
                    epoch.velocity += correction;
                }
            }
        }
        // Before yaw alignment, propagate position between fixes using the
        // observed vector. Inertial speed remains available in the level frame.
        if self.yaw.is_none() && previous.is_some() {
            self.position += anchor.vector_to_ecef(v) * f64::from(dt);
        }
        if accept_position || previous.is_none() {
            let age = self.last_imu.saturating_sub(fix.position_us) as f64 * 1e-6;
            let target = pos + Vector3::from(fix.velocity_ecef_mps) * age;
            let gain = if !position_recent || self.tilt.is_none() {
                1.
            } else {
                dt / (dt + (fix.position_sigma_m * 0.15).clamp(0.1, 2.))
            };
            self.position += (target - self.position) * f64::from(gain);
            if accept_position {
                self.position_fix_us = Some(fix.position_us);
            }
        }
        self.anchor = Some(anchor);
        self.fixes = Some((fix.position_us, fix.velocity_us, v, fix.speed_sigma_mps));
        true
    }

    fn solution(&self, fix_timeout_us: u64) -> Option<DrivingPoint> {
        let (_, fix, _, _) = self.fixes?;
        if self.tilt.is_none()
            || self.history.is_empty()
            || self.needs_aid_after.is_some()
            || self.last_imu.abs_diff(fix) > fix_timeout_us
        {
            return None;
        }
        Some(DrivingPoint {
            epoch_us: self.last_imu,
            position_ecef_m: self.position.into(),
            speed_mps: self.velocity.xy().norm(),
        })
    }

    pub fn point(&self) -> Option<DrivingPoint> {
        if !self.has_current_position() {
            return None;
        }
        self.solution(FIX_TIMEOUT_US)
    }

    fn has_current_position(&self) -> bool {
        self.fixes
            .is_some_and(|(position, _, _, _)| self.position_fix_us == Some(position))
    }

    pub fn motion(&self, mounted: bool) -> Option<DrivingMotion> {
        let point = self.solution(MAX_COAST_US)?;
        let inertial_only = self
            .fixes
            .is_some_and(|(_, fix, _, _)| self.last_imu.saturating_sub(fix) > FIX_TIMEOUT_US);
        let tangent = EcefAnchor::from_origin(0, self.position, ReferenceEllipsoid::WGS84).ok()?;
        let up = tangent.ecef_to_n.row(2).transpose();
        let lat = libm::atan2(up.z, libm::hypot(up.x, up.y));
        let ellipsoid = ReferenceEllipsoid::WGS84;
        let f = 1. / ellipsoid.inverse_flattening();
        let height = self.position.dot(&up)
            - ellipsoid.semi_major_axis_m() * libm::sqrt(1. - f * (2. - f) * up.z * up.z);
        let bearing = |v: Vec3| (libm::atan2f(v.x, v.y).to_degrees() + 360.) % 360.;
        let course = self
            .yaw
            .filter(|_| point.speed_mps > 1.)
            .map(|yaw| bearing(turn(self.velocity, yaw)));
        let forward = self.tilt? * Vec3::x();
        let heading = self
            .yaw
            .filter(|_| mounted && forward.xy().norm() > 0.1)
            .map(|yaw| bearing(turn(forward, yaw)));
        Some(DrivingMotion {
            point,
            latitude_deg: lat.to_degrees(),
            longitude_deg: libm::atan2(self.position.y, self.position.x).to_degrees(),
            height_m: height,
            course_deg: course,
            heading_deg: heading,
            vehicle_g: mounted.then(|| self.body_g.into()),
            position_degraded: !self.has_current_position(),
            inertial_only,
        })
    }
}

fn turn(v: Vec3, yaw: f32) -> Vec3 {
    let (s, c) = libm::sincosf(yaw);
    Vec3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
}
fn wrap(angle: f32) -> f32 {
    libm::atan2f(libm::sinf(angle), libm::cosf(angle))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(t: u64, acceleration: f32) -> DrivingFix {
        // Vehicle x points north at the equator. Doppler is 60 ms behind position.
        let elapsed = |time: u64| time.saturating_sub(1_000_000) as f64 * 1e-6;
        let velocity_us = t.saturating_sub(60_000);
        DrivingFix {
            position_us: t,
            velocity_us,
            position_ecef_m: [
                6_378_137.,
                0.,
                0.5 * f64::from(acceleration) * elapsed(t).powi(2),
            ],
            velocity_ecef_mps: [0., 0., f64::from(acceleration) * elapsed(velocity_us)],
            speed_sigma_mps: 0.05,
            position_sigma_m: 1.,
        }
    }

    fn launch(acceleration: f32) -> DrivingEstimator {
        let mut filter = DrivingEstimator::new([1., 0., 0., 0.]).unwrap();
        for i in 1..=600 {
            let t = i * 5_000;
            let a = if t <= 1_000_000 { 0. } else { acceleration };
            assert!(filter.observe_imu(t, 5_000, [a, 0., G], [0.; 3]));
            if t >= 100_000 && i % 4 == 0 {
                assert!(filter.observe_fix(fix(t, acceleration)));
            }
        }
        filter
    }

    #[test]
    fn accelerating_and_reversing_have_distinct_heading_and_course() {
        for direction in [1., -1.] {
            let mut filter = launch(direction * 3.);
            let motion = filter.motion(true).unwrap();
            assert!((motion.point.speed_mps - 6.).abs() < 0.025);
            let heading = motion.heading_deg.unwrap();
            assert!(heading.min(360. - heading) < 0.25);
            let course = motion.course_deg.unwrap();
            let expected = if direction > 0. { 0. } else { 180. };
            assert!(wrap((course - expected).to_radians()).abs() < 0.005);
            assert!((motion.point.position_ecef_m[2] - f64::from(direction) * 6.).abs() < 0.15);
            let before = motion.point.speed_mps;
            filter.observe_imu(3_005_000, 5_000, [3. * direction, -2., G], [0.; 3]);
            let after = filter.motion(true).unwrap();
            assert!(after.point.speed_mps - before > 0.014);
            let g = after.vehicle_g.unwrap();
            assert!((g[0] - 3. * direction / G).abs() < 0.001 && (g[1] + 2. / G).abs() < 0.001);
            assert!(filter.motion(false).unwrap().vehicle_g.is_none());
        }
    }

    #[test]
    fn constant_speed_left_turn_separates_lateral_g_from_speed_change() {
        let mut filter = launch(3.);
        let yaw_rate = 0.3f32;
        let speed = 6.;
        for i in 1..=600 {
            let t = 3_000_000 + i * 5000;
            filter.observe_imu(t, 5000, [0., speed * yaw_rate, G], [0., 0., yaw_rate]);
            if i % 4 == 0 {
                let mut f = fix(t, 3.);
                let a = (t - 3_000_000) as f32 * 1e-6 * yaw_rate;
                f.position_ecef_m = [
                    6_378_137.,
                    f64::from(speed / yaw_rate * (libm::cosf(a) - 1.)),
                    6. + f64::from(speed / yaw_rate * libm::sinf(a)),
                ];
                if f.velocity_us >= 3_000_000 {
                    let a = (f.velocity_us - 3_000_000) as f32 * 1e-6 * yaw_rate;
                    f.velocity_ecef_mps = [
                        0.,
                        f64::from(-speed * libm::sinf(a)),
                        f64::from(speed * libm::cosf(a)),
                    ];
                }
                assert!(filter.observe_fix(f));
            }
        }
        let m = filter.motion(true).unwrap();
        assert!((m.point.speed_mps - speed).abs() < 0.03);
        let g = m.vehicle_g.unwrap();
        assert!(
            g[0].abs() < 0.002 && (g[1] - speed * yaw_rate / G).abs() < 0.002 && g[2].abs() < 0.002
        );
        for angle in [m.heading_deg, m.course_deg] {
            assert!(wrap(angle.unwrap().to_radians() + 0.9).abs() < 0.01);
        }
    }

    #[test]
    fn a_short_missing_batch_preserves_the_impulse_but_long_gaps_interrupt() {
        let mut f = launch(3.);
        let v = f.point().unwrap().speed_mps;
        assert!(f.observe_imu(3_013_000, 5_000, [3., 0., G], [0.; 3]));
        assert!((f.point().unwrap().speed_mps - v - 0.039).abs() < 0.001);
        assert_eq!(f.bridged_intervals, 1);
        assert!(f.observe_imu(3_035_000, 5_000, [3., 0., G], [0.; 3]));
        assert!(f.point().is_none());
    }

    #[test]
    fn missing_imu_burst_reacquires_while_still_moving() {
        let mut f = launch(3.);
        assert!(f.observe_imu(3_060_000, 5_000, [3., 0., G], [0.; 3]));
        assert!(f.point().is_none());
        for i in 13..=28 {
            let t = 3_000_000 + i * 5_000;
            assert!(f.observe_imu(t, 5_000, [3., 0., G], [0.; 3]));
            if i % 4 == 0 {
                assert!(f.observe_fix(fix(t, 3.)));
            }
        }
        let motion = f
            .motion(true)
            .expect("fresh Doppler must recover while moving");
        assert!((motion.point.speed_mps - 6.42).abs() < 0.05);
    }

    #[test]
    fn continuous_imu_can_coast_through_a_brief_gnss_outage() {
        let mut f = launch(3.);
        for i in 1..=440 {
            f.observe_imu(3_000_000 + i * 5_000, 5_000, [3., 0., G], [0.; 3]);
            if i == 250 {
                assert!(f.point().is_none());
                let motion = f.motion(true).expect("fresh IMU should keep live speed");
                assert!((motion.point.speed_mps - 9.75).abs() < 0.04);
                assert!(motion.inertial_only);
            }
        }
        assert!(f.motion(true).is_none());
    }

    #[test]
    fn gnss_clock_remap_does_not_discard_imu_alignment() {
        let mut f = launch(3.);
        f.reset_gnss_clock();
        assert!(f.motion(true).is_none());
        assert!(f.observe_imu(3_005_000, 5_000, [3., 0., G], [0.; 3]));
        let mut remapped = fix(2_980_000, 3.);
        remapped.velocity_us = 2_960_000;
        assert!(f.observe_fix(remapped));
        assert!(f.motion(true).is_some());
    }

    #[test]
    fn weak_position_during_motion_keeps_doppler_aided_speed() {
        let mut filter = launch(3.);
        for i in 1..=480 {
            let t = 3_000_000 + i * 5_000;
            assert!(filter.observe_imu(t, 5_000, [3., 0., G], [0.; 3]));
            if i % 4 == 0 {
                let mut satellite = fix(t, 3.);
                // The board reported 33–40 m position uncertainty while
                // Doppler remained below 0.2 m/s and PPS stayed locked.
                satellite.position_sigma_m = 40.;
                satellite.speed_sigma_mps = 0.19;
                satellite.position_ecef_m[1] += 38.;
                assert!(filter.observe_fix(satellite));
            }
        }
        let motion = filter
            .motion(true)
            .expect("fresh Doppler and IMU still support speed");
        assert!((motion.point.speed_mps - 13.2).abs() < 0.03);
        assert!(motion.point.position_ecef_m[1].abs() < 0.1);
        assert!(
            filter.point().is_none(),
            "weak position must interrupt a timing attempt"
        );
        assert!(motion.position_degraded);
        assert!(filter.observe_imu(5_405_000, 5_000, [3., 0., G], [0.; 3]));
        assert!(filter.observe_fix(fix(5_420_000, 3.)));
        assert!(filter.point().is_some(), "good position restores timing");
        assert!(!filter.motion(true).unwrap().position_degraded);
    }

    #[test]
    fn a_pitched_pickup_keeps_speed_with_weak_gnss_position() {
        let mut filter = DrivingEstimator::new([1., 0., 0., 0.]).unwrap();
        let since_pickup = |t: u64| t.saturating_sub(1_000_000) as f32 * 1e-6;
        let pitch = |t: u64| since_pickup(t).min(0.6) * 1.6;
        for i in 1..=1_000 {
            let t = i * 5_000;
            let mid = t - 2_500;
            let lifting = (1_000_000..1_600_000).contains(&mid);
            let attitude = UnitQuaternion::from_axis_angle(&Vec3::y_axis(), pitch(mid));
            let force = attitude.inverse_transform_vector(&Vec3::new(
                if lifting { 2.5 } else { 0. },
                0.,
                G,
            ));
            assert!(filter.observe_imu(
                t,
                5_000,
                force.into(),
                [0., if lifting { 1.6 } else { 0. }, 0.],
            ));
            if t >= 100_000 && i % 4 == 0 {
                let mut satellite = fix(t, 0.);
                let elapsed = since_pickup(t);
                let ramp = elapsed.min(0.6);
                satellite.position_ecef_m[2] =
                    f64::from(1.25 * ramp * ramp + 1.5 * (elapsed - ramp));
                satellite.velocity_ecef_mps[2] =
                    f64::from(since_pickup(satellite.velocity_us).min(0.6) * 2.5);
                satellite.position_sigma_m = if t > 1_000_000 { 39. } else { 4. };
                assert!(filter.observe_fix(satellite));
            }
            if t > 1_000_000 {
                let motion = filter
                    .motion(false)
                    .expect("pickup interrupted fused motion");
                assert!(motion.vehicle_g.is_none());
            }
        }
        let motion = filter.motion(false).unwrap();
        assert!((motion.point.speed_mps - 1.5).abs() < 0.03);
        assert!(motion.position_degraded && filter.point().is_none());
    }

    #[test]
    fn fixes_reacquire_before_yaw_with_continuous_imu_support() {
        for acceleration in [0., 3.] {
            let mut f = DrivingEstimator::new([1., 0., 0., 0.]).unwrap();
            for i in 1..=650 {
                let t = i * 5000;
                let a = if t > 1_000_000 { acceleration } else { 0. };
                assert!(f.observe_imu(t, 5000, [a, 0., G], [0.; 3]));
                if t == 2_400_000 {
                    assert!(f.point().is_none());
                }
                if i % 4 == 0 && t >= 100_000 && !(900_000..2_400_000).contains(&t) {
                    assert!(f.observe_fix(fix(t, acceleration)));
                }
                if t == 2_400_000 {
                    let m = f.motion(true).unwrap();
                    assert!((m.point.speed_mps - acceleration * 1.4).abs() < 0.03);
                    assert!(m.heading_deg.is_none());
                }
            }
            assert_eq!(
                f.motion(true).unwrap().heading_deg.is_some(),
                acceleration > 0.
            );
        }
    }

    #[test]
    fn gravity_is_removed_on_a_bank_and_gaps_withhold_motion() {
        let mut f = DrivingEstimator::new([1., 0., 0., 0.]).unwrap();
        let gravity = [0., 0.6 * G, 0.8 * G];
        for n in 1..=200 {
            let t = n * 5000;
            f.observe_imu(t, 5000, gravity, [0.; 3]);
        }
        assert!(f.point().is_none());
        for n in 201..=400 {
            let t = n * 5000;
            f.observe_imu(t, 5000, gravity, [0.; 3]);
            if n % 4 == 0 {
                assert!(f.observe_fix(fix(t, 0.)));
            }
        }
        assert!(
            f.motion(true)
                .unwrap()
                .vehicle_g
                .unwrap()
                .iter()
                .all(|v| v.abs() < 1e-4)
        );
        let mut bad = fix(2_020_000, 0.);
        bad.velocity_ecef_mps[2] = 35.;
        assert!(!f.observe_fix(bad));
        assert!(!f.observe_fix(fix(2_000_000, 0.)));
        // One second without a fix expires the solution even with perfect IMU support.
        for n in 401..=620 {
            f.observe_imu(n * 5000, 5000, gravity, [0.; 3]);
        }
        assert!(f.point().is_none());
        f.observe_imu(3_200_000, 5000, gravity, [0.; 3]);
        assert!(f.point().is_none());
    }
}
