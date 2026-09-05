//! Explicit navigation initialization and stationary-evidence handling.

use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::time::SessionTime;

use super::state::{ACC_BIAS, ATT, GYRO_BIAS, NavMatrix, NavState, POS, VEL, skew};

pub(crate) const STATIONARY_WINDOW_CAPACITY: usize = 512;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum InitializationPhase {
    Uninitialized,
    CoarseAligning,
    FineAligning,
    Navigating,
    Invalid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum InitialHeadingSource {
    Supplied,
    Gyrocompass,
    DynamicConstraint,
    None,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct StationaryConfig {
    pub(crate) gravity_magnitude: f32,
    pub(crate) gyro_score_variance: f32,
    pub(crate) force_norm_score_variance: f32,
    pub(crate) probability_stays_stationary: f32,
    pub(crate) probability_motion_becomes_stationary: f32,
    pub(crate) enter_probability: f32,
    pub(crate) exit_probability: f32,
    pub(crate) minimum_window_samples: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct AlignmentConfig {
    pub(crate) stationary: StationaryConfig,
    pub(crate) minimum_coarse_samples: u32,
    pub(crate) minimum_gyrocompass_samples: u32,
    pub(crate) gyrocompassing_qualified: bool,
    pub(crate) minimum_earth_rate_cross_gravity: f32,
    pub(crate) maximum_force_variance: f32,
    pub(crate) maximum_gyro_variance: f32,
    pub(crate) minimum_dynamic_yaw_information: f32,
    pub(crate) maximum_dynamic_yaw_variance: f32,
    pub(crate) roll_pitch_variance: f32,
    pub(crate) unobservable_yaw_variance: f32,
    pub(crate) accel_bias_prior: Vector3<f32>,
    pub(crate) gyro_bias_prior: Vector3<f32>,
    pub(crate) accel_bias_variance: Vector3<f32>,
    pub(crate) gyro_bias_variance: Vector3<f32>,
}

impl AlignmentConfig {
    pub(crate) fn validate(&self) -> Result<(), InitializationError> {
        let stationary = self.stationary;
        let probabilities = [
            stationary.probability_stays_stationary,
            stationary.probability_motion_becomes_stationary,
            stationary.enter_probability,
            stationary.exit_probability,
        ];
        if !stationary.gravity_magnitude.is_finite()
            || stationary.gravity_magnitude <= 0.0
            || !stationary.gyro_score_variance.is_finite()
            || stationary.gyro_score_variance <= 0.0
            || !stationary.force_norm_score_variance.is_finite()
            || stationary.force_norm_score_variance <= 0.0
            || probabilities
                .iter()
                .any(|probability| !probability.is_finite() || !(0.0..=1.0).contains(probability))
            || stationary.exit_probability >= stationary.enter_probability
            || stationary.minimum_window_samples == 0
            || usize::from(stationary.minimum_window_samples) > STATIONARY_WINDOW_CAPACITY
            || self.minimum_coarse_samples == 0
            || self.minimum_gyrocompass_samples < self.minimum_coarse_samples
        {
            return Err(InitializationError::InvalidConfiguration);
        }
        let positive = [
            self.minimum_earth_rate_cross_gravity,
            self.maximum_force_variance,
            self.maximum_gyro_variance,
            self.minimum_dynamic_yaw_information,
            self.maximum_dynamic_yaw_variance,
            self.roll_pitch_variance,
            self.unobservable_yaw_variance,
        ];
        if positive
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
            || self
                .accel_bias_prior
                .iter()
                .chain(self.gyro_bias_prior.iter())
                .chain(self.accel_bias_variance.iter())
                .chain(self.gyro_bias_variance.iter())
                .any(|value| !value.is_finite())
            || self.accel_bias_variance.iter().any(|value| *value < 0.0)
            || self.gyro_bias_variance.iter().any(|value| *value < 0.0)
        {
            return Err(InitializationError::InvalidConfiguration);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GnssInitializationFix {
    pub(crate) time: SessionTime,
    /// Oldest effective epoch contributing to this extrapolated fix. This is
    /// the freshness origin; `time` is the extrapolated state epoch and must
    /// not refresh old receiver evidence indefinitely.
    pub(crate) evidence_oldest_time: SessionTime,
    pub(crate) position_n: Vector3<f32>,
    pub(crate) velocity_n: Vector3<f32>,
    pub(crate) position_covariance_n: Matrix3<f32>,
    pub(crate) velocity_covariance_n: Matrix3<f32>,
    /// Cov(position, velocity) at `time`; this block need not be symmetric.
    pub(crate) position_velocity_cross_n: Matrix3<f32>,
    pub(crate) zero_velocity_nis: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct InitializationResult {
    pub(crate) state: NavState,
    pub(crate) covariance: NavMatrix,
    pub(crate) heading_source: InitialHeadingSource,
    pub(crate) stationary_probability: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct DynamicYawCandidate {
    yaw_rad_enu: f32,
    variance: f32,
    information: f32,
    constraint_valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct VectorMoments {
    count: u32,
    mean: Vector3<f32>,
    m2: Vector3<f32>,
}

impl VectorMoments {
    const fn new() -> Self {
        Self {
            count: 0,
            mean: Vector3::new(0.0, 0.0, 0.0),
            m2: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    fn push(&mut self, value: Vector3<f32>) {
        self.count = self.count.saturating_add(1);
        let delta = value - self.mean;
        self.mean += delta / self.count as f32;
        let delta_after = value - self.mean;
        self.m2 += delta.component_mul(&delta_after);
    }

    fn maximum_variance(&self) -> f32 {
        if self.count < 2 {
            return f32::INFINITY;
        }
        (self.m2 / (self.count - 1) as f32).max()
    }

    fn clear(&mut self) {
        *self = Self::new();
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct StationaryClassifier {
    scores: [f32; STATIONARY_WINDOW_CAPACITY],
    head: usize,
    len: usize,
    score_sum: f32,
    stationary_probability: f32,
    stationary_latched: bool,
    config: StationaryConfig,
    latest_zero_velocity_nis: Option<(SessionTime, f32)>,
}

impl StationaryClassifier {
    pub(crate) fn new(config: StationaryConfig) -> Result<Self, InitializationError> {
        let wrapper = AlignmentConfig {
            stationary: config,
            minimum_coarse_samples: 1,
            minimum_gyrocompass_samples: 1,
            gyrocompassing_qualified: false,
            minimum_earth_rate_cross_gravity: 1.0e-9,
            maximum_force_variance: 1.0,
            maximum_gyro_variance: 1.0,
            minimum_dynamic_yaw_information: 1.0,
            maximum_dynamic_yaw_variance: 1.0,
            roll_pitch_variance: 1.0,
            unobservable_yaw_variance: 1.0,
            accel_bias_prior: Vector3::zeros(),
            gyro_bias_prior: Vector3::zeros(),
            accel_bias_variance: Vector3::zeros(),
            gyro_bias_variance: Vector3::zeros(),
        };
        wrapper.validate()?;
        Ok(Self {
            scores: [0.0; STATIONARY_WINDOW_CAPACITY],
            head: 0,
            len: 0,
            score_sum: 0.0,
            stationary_probability: 0.5,
            stationary_latched: false,
            config,
            latest_zero_velocity_nis: None,
        })
    }

    pub(crate) fn observe_gnss_zero_velocity_nis(
        &mut self,
        evidence_time: SessionTime,
        nis: Option<f32>,
    ) {
        self.latest_zero_velocity_nis = nis
            .filter(|value| value.is_finite() && *value >= 0.0)
            .map(|value| (evidence_time, value));
    }

    pub(crate) fn observe_imu(
        &mut self,
        time: SessionTime,
        maximum_gnss_age_ns: u64,
        omega_ib_b: Vector3<f32>,
        specific_force_b: Vector3<f32>,
    ) -> Result<f32, InitializationError> {
        if omega_ib_b
            .iter()
            .chain(specific_force_b.iter())
            .any(|value| !value.is_finite())
        {
            return Err(InitializationError::NonFinite);
        }
        let force_error = specific_force_b.norm() - self.config.gravity_magnitude;
        let score = omega_ib_b.norm_squared() / self.config.gyro_score_variance
            + force_error * force_error / self.config.force_norm_score_variance;
        if self.len == STATIONARY_WINDOW_CAPACITY {
            self.score_sum -= self.scores[self.head];
        } else {
            self.len += 1;
        }
        self.scores[self.head] = score;
        self.score_sum += score;
        self.head = (self.head + 1) % STATIONARY_WINDOW_CAPACITY;

        let mean_score = self.score_sum / self.len as f32;
        let fresh_gnss_nis = self
            .latest_zero_velocity_nis
            .filter(|(evidence_time, _)| {
                evidence_is_fresh(time, *evidence_time, maximum_gnss_age_ns)
            })
            .map(|(_, value)| value);
        if fresh_gnss_nis.is_none() {
            self.latest_zero_velocity_nis = None;
        }
        // Missing receiver evidence is neutral, not a perfect zero-velocity
        // innovation. More importantly, it cannot latch the classifier and
        // authorize a ZUPT/alignment transition by itself.
        let gnss_score = fresh_gnss_nis.unwrap_or(1.0);
        // Bounded likelihoods avoid exponentials and remain monotone in the
        // calibrated GLRT/Mahalanobis evidence.
        let combined_score = mean_score + gnss_score;
        let stationary_likelihood = 1.0 / (1.0 + combined_score);
        let motion_likelihood = 1.0 - stationary_likelihood;
        let prior = self.config.probability_stays_stationary * self.stationary_probability
            + self.config.probability_motion_becomes_stationary
                * (1.0 - self.stationary_probability);
        let stationary_weight = prior * stationary_likelihood;
        let motion_weight = (1.0 - prior) * motion_likelihood;
        let normalizer = stationary_weight + motion_weight;
        self.stationary_probability = if normalizer > f32::EPSILON {
            stationary_weight / normalizer
        } else {
            prior
        };
        if self.len >= usize::from(self.config.minimum_window_samples) {
            if fresh_gnss_nis.is_none() {
                self.stationary_latched = false;
            } else if self.stationary_latched {
                if self.stationary_probability < self.config.exit_probability {
                    self.stationary_latched = false;
                }
            } else if self.stationary_probability > self.config.enter_probability {
                self.stationary_latched = true;
            }
        }
        Ok(self.stationary_probability)
    }

    pub(crate) fn probability(&self) -> f32 {
        self.stationary_probability
    }

    pub(crate) fn stationary(&self) -> bool {
        self.stationary_latched
    }

    const fn decision_ready(&self) -> bool {
        self.len >= self.config.minimum_window_samples as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Initializer {
    pub(crate) phase: InitializationPhase,
    config: AlignmentConfig,
    classifier: StationaryClassifier,
    force_moments: VectorMoments,
    gyro_moments: VectorMoments,
    latest_fix: Option<GnssInitializationFix>,
    supplied_yaw: Option<(f32, f32)>,
    dynamic_yaw: Option<DynamicYawCandidate>,
}

impl Initializer {
    // Keep this required frame distinct for the generated-stack audit.
    #[inline(never)]
    pub(crate) fn new(config: AlignmentConfig) -> Result<Self, InitializationError> {
        config.validate()?;
        Ok(Self {
            phase: InitializationPhase::Uninitialized,
            config,
            classifier: StationaryClassifier::new(config.stationary)?,
            force_moments: VectorMoments::new(),
            gyro_moments: VectorMoments::new(),
            latest_fix: None,
            supplied_yaw: None,
            dynamic_yaw: None,
        })
    }

    pub(crate) fn observe_gnss(
        &mut self,
        fix: GnssInitializationFix,
    ) -> Result<(), InitializationError> {
        if fix
            .position_n
            .iter()
            .chain(fix.velocity_n.iter())
            .chain(fix.position_covariance_n.iter())
            .chain(fix.velocity_covariance_n.iter())
            .chain(fix.position_velocity_cross_n.iter())
            .any(|value| !value.is_finite())
        {
            return Err(InitializationError::NonFinite);
        }
        self.classifier
            .observe_gnss_zero_velocity_nis(fix.evidence_oldest_time, fix.zero_velocity_nis);
        self.latest_fix = Some(fix);
        Ok(())
    }

    pub(crate) fn observe_imu(
        &mut self,
        time: SessionTime,
        maximum_gnss_age_ns: u64,
        omega_ib_b: Vector3<f32>,
        specific_force_b: Vector3<f32>,
    ) -> Result<(), InitializationError> {
        if self.latest_fix.is_some_and(|fix| {
            !evidence_is_fresh(time, fix.evidence_oldest_time, maximum_gnss_age_ns)
        }) {
            self.latest_fix = None;
        }
        self.classifier
            .observe_imu(time, maximum_gnss_age_ns, omega_ib_b, specific_force_b)?;
        if self.classifier.stationary() {
            if self.phase == InitializationPhase::Uninitialized {
                self.phase = InitializationPhase::CoarseAligning;
            }
            self.force_moments.push(specific_force_b);
            self.gyro_moments.push(omega_ib_b);
            if self.force_moments.count >= self.config.minimum_coarse_samples
                && self.latest_fix.is_some()
            {
                self.phase = InitializationPhase::FineAligning;
            }
        } else if self.phase == InitializationPhase::Uninitialized
            && !self.classifier.decision_ready()
        {
            // Retain the bounded classifier warm-up samples. If the completed
            // window does not latch stationary they are discarded below, so
            // moving evidence can never contaminate an alignment mean.
            self.force_moments.push(specific_force_b);
            self.gyro_moments.push(omega_ib_b);
        } else {
            // Static means cannot straddle a classified motion interval.
            self.force_moments.clear();
            self.gyro_moments.clear();
            if matches!(
                self.phase,
                InitializationPhase::CoarseAligning | InitializationPhase::FineAligning
            ) {
                self.phase = InitializationPhase::Uninitialized;
            }
        }
        Ok(())
    }

    /// Supplies a yaw in ENU mathematical convention (zero east, positive
    /// towards north), with an independent prior variance.
    pub(crate) fn provide_heading(
        &mut self,
        yaw_rad_enu: f32,
        variance: f32,
    ) -> Result<(), InitializationError> {
        if !yaw_rad_enu.is_finite() || !variance.is_finite() || variance <= 0.0 {
            return Err(InitializationError::NonFinite);
        }
        self.supplied_yaw = Some((yaw_rad_enu, variance));
        Ok(())
    }

    /// Accepts only a candidate whose upstream observability calculation has
    /// eliminated nuisance directions and validated the motion constraint.
    #[cfg(test)]
    pub(crate) fn provide_dynamic_yaw(
        &mut self,
        yaw_rad_enu: f32,
        variance: f32,
        information: f32,
        constraint_valid: bool,
    ) -> Result<(), InitializationError> {
        if !yaw_rad_enu.is_finite()
            || !variance.is_finite()
            || variance <= 0.0
            || !information.is_finite()
            || information < 0.0
        {
            return Err(InitializationError::NonFinite);
        }
        self.dynamic_yaw = Some(DynamicYawCandidate {
            yaw_rad_enu,
            variance,
            information,
            constraint_valid,
        });
        Ok(())
    }

    pub(crate) fn try_initialize(
        &mut self,
        earth_rate_n: Vector3<f32>,
    ) -> Result<Option<InitializationResult>, InitializationError> {
        if self.phase != InitializationPhase::FineAligning {
            return Ok(None);
        }
        let fix = self.latest_fix.ok_or(InitializationError::MissingGnss)?;
        if self.force_moments.maximum_variance() > self.config.maximum_force_variance
            || self.gyro_moments.maximum_variance() > self.config.maximum_gyro_variance
        {
            return Ok(None);
        }
        let corrected_force_b = self.force_moments.mean - self.config.accel_bias_prior;
        let corrected_force_norm = corrected_force_b.norm();
        if !corrected_force_norm.is_finite() || corrected_force_norm <= 1.0e-6 {
            return Err(InitializationError::DegenerateAlignment);
        }
        let body_up = corrected_force_b
            .try_normalize(1.0e-6)
            .ok_or(InitializationError::DegenerateAlignment)?;
        let nav_up = Vector3::z();
        let tilt = gravity_tilt(body_up, nav_up);

        let (orientation, yaw_variance, heading_source) =
            if let Some((yaw, variance)) = self.supplied_yaw {
                (
                    impose_enu_yaw(tilt, yaw),
                    variance,
                    InitialHeadingSource::Supplied,
                )
            } else {
                let gyrocompass_orientation = if self.config.gyrocompassing_qualified
                    && self.gyro_moments.count >= self.config.minimum_gyrocompass_samples
                {
                    let body_earth_rate = self.gyro_moments.mean - self.config.gyro_bias_prior;
                    match triad_rotation(
                        body_up,
                        body_earth_rate,
                        nav_up,
                        earth_rate_n,
                        self.config.minimum_earth_rate_cross_gravity,
                    ) {
                        Ok(orientation) => Some(orientation),
                        Err(InitializationError::YawUnobservable) => None,
                        Err(error) => return Err(error),
                    }
                } else {
                    None
                };
                if let Some(orientation) = gyrocompass_orientation {
                    (
                        orientation,
                        self.config.roll_pitch_variance,
                        InitialHeadingSource::Gyrocompass,
                    )
                } else if let Some(candidate) = self.dynamic_yaw.filter(|candidate| {
                    candidate.constraint_valid
                        && candidate.information >= self.config.minimum_dynamic_yaw_information
                        && candidate.variance <= self.config.maximum_dynamic_yaw_variance
                }) {
                    (
                        impose_enu_yaw(tilt, candidate.yaw_rad_enu),
                        candidate.variance,
                        InitialHeadingSource::DynamicConstraint,
                    )
                } else {
                    (
                        tilt,
                        self.config.unobservable_yaw_variance,
                        InitialHeadingSource::None,
                    )
                }
            };

        let state = NavState {
            time: fix.time,
            position_n: fix.position_n,
            velocity_n: fix.velocity_n,
            orientation_n_from_b: orientation,
            accel_bias_b: self.config.accel_bias_prior,
            gyro_bias_b: self.config.gyro_bias_prior,
        };
        let mut covariance = NavMatrix::zeros();
        covariance
            .fixed_view_mut::<3, 3>(POS, POS)
            .copy_from(&fix.position_covariance_n);
        covariance
            .fixed_view_mut::<3, 3>(VEL, VEL)
            .copy_from(&fix.velocity_covariance_n);
        covariance
            .fixed_view_mut::<3, 3>(POS, VEL)
            .copy_from(&fix.position_velocity_cross_n);
        covariance
            .fixed_view_mut::<3, 3>(VEL, POS)
            .copy_from(&fix.position_velocity_cross_n.transpose());
        // Attitude error is right-multiplicative, so the unobservable yaw
        // direction is local/nav up expressed in the body tangent. Gravity
        // constrains only its orthogonal plane.
        let yaw_projector_b = body_up * body_up.transpose();
        let tilt_projector_b = Matrix3::identity() - yaw_projector_b;
        let accel_bias_covariance = Matrix3::from_diagonal(&self.config.accel_bias_variance);
        // Corrected specific force is `f_measured - b_prior`. An error in the
        // bias prior therefore perturbs the inferred right-tangent tilt by
        // skew(body_up) * delta_bias / |f_corrected|. Retain both the induced
        // attitude marginal and its correlation with the bias state.
        let bias_to_tilt = skew(&body_up) / corrected_force_norm;
        let attitude_bias_covariance = bias_to_tilt * accel_bias_covariance;
        let attitude_covariance = tilt_projector_b * self.config.roll_pitch_variance
            + yaw_projector_b * yaw_variance
            + attitude_bias_covariance * bias_to_tilt.transpose();
        if !attitude_covariance
            .iter()
            .chain(attitude_bias_covariance.iter())
            .all(|value| value.is_finite())
        {
            return Err(InitializationError::DegenerateAlignment);
        }
        covariance
            .fixed_view_mut::<3, 3>(ATT, ATT)
            .copy_from(&attitude_covariance);
        covariance
            .fixed_view_mut::<3, 3>(ATT, ACC_BIAS)
            .copy_from(&attitude_bias_covariance);
        covariance
            .fixed_view_mut::<3, 3>(ACC_BIAS, ATT)
            .copy_from(&attitude_bias_covariance.transpose());
        for axis in 0..3 {
            covariance[(ACC_BIAS + axis, ACC_BIAS + axis)] = self.config.accel_bias_variance[axis];
            covariance[(GYRO_BIAS + axis, GYRO_BIAS + axis)] = self.config.gyro_bias_variance[axis];
        }
        self.phase = InitializationPhase::Navigating;
        Ok(Some(InitializationResult {
            state,
            covariance,
            heading_source,
            stationary_probability: self.classifier.probability(),
        }))
    }
}

fn evidence_is_fresh(now: SessionTime, evidence_time: SessionTime, maximum_age_ns: u64) -> bool {
    now.checked_duration_since(evidence_time)
        .is_some_and(|age| age.as_ns() >= 0 && age.as_ns() as u64 <= maximum_age_ns)
}

fn impose_enu_yaw(tilt: UnitQuaternion<f32>, desired_yaw: f32) -> UnitQuaternion<f32> {
    let forward = tilt.transform_vector(&Vector3::x());
    let current_yaw = crate::scalar_math::atan2(forward.y, forward.x);
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), desired_yaw - current_yaw) * tilt
}

fn gravity_tilt(body_up: Vector3<f32>, nav_up: Vector3<f32>) -> UnitQuaternion<f32> {
    UnitQuaternion::rotation_between(&body_up, &nav_up).unwrap_or_else(|| {
        // Exact anti-parallel gravity still determines tilt; only the yaw-like
        // choice of the half-turn axis is ambiguous. Choose body X
        // deterministically and retain that ambiguity in the yaw covariance.
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f32::consts::PI)
    })
}

fn triad_rotation(
    primary_body: Vector3<f32>,
    secondary_body: Vector3<f32>,
    primary_nav: Vector3<f32>,
    secondary_nav: Vector3<f32>,
    minimum_cross_norm: f32,
) -> Result<UnitQuaternion<f32>, InitializationError> {
    let b1 = primary_body
        .try_normalize(1.0e-8)
        .ok_or(InitializationError::DegenerateAlignment)?;
    let n1 = primary_nav
        .try_normalize(1.0e-8)
        .ok_or(InitializationError::DegenerateAlignment)?;
    let body_cross = b1.cross(&secondary_body);
    let nav_cross = n1.cross(&secondary_nav);
    if body_cross.norm() < minimum_cross_norm || nav_cross.norm() < minimum_cross_norm {
        return Err(InitializationError::YawUnobservable);
    }
    let b2 = body_cross.normalize();
    let b3 = b1.cross(&b2);
    let n2 = nav_cross.normalize();
    let n3 = n1.cross(&n2);
    let body_basis = Matrix3::from_columns(&[b1, b2, b3]);
    let nav_basis = Matrix3::from_columns(&[n1, n2, n3]);
    let rotation = nav_basis * body_basis.transpose();
    if (rotation.determinant() - 1.0).abs() > 1.0e-3 {
        return Err(InitializationError::DegenerateAlignment);
    }
    Ok(UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(rotation),
    ))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum InitializationError {
    InvalidConfiguration,
    NonFinite,
    MissingGnss,
    DegenerateAlignment,
    YawUnobservable,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> AlignmentConfig {
        AlignmentConfig {
            stationary: StationaryConfig {
                gravity_magnitude: 9.806_65,
                gyro_score_variance: 1.0,
                force_norm_score_variance: 1.0,
                probability_stays_stationary: 0.999,
                probability_motion_becomes_stationary: 0.5,
                enter_probability: 0.6,
                exit_probability: 0.2,
                minimum_window_samples: 2,
            },
            minimum_coarse_samples: 4,
            minimum_gyrocompass_samples: 8,
            gyrocompassing_qualified: false,
            minimum_earth_rate_cross_gravity: 1.0e-6,
            maximum_force_variance: 1.0e-4,
            maximum_gyro_variance: 1.0e-6,
            minimum_dynamic_yaw_information: 10.0,
            maximum_dynamic_yaw_variance: 0.1,
            roll_pitch_variance: 1.0e-3,
            unobservable_yaw_variance: 10.0,
            accel_bias_prior: Vector3::zeros(),
            gyro_bias_prior: Vector3::zeros(),
            accel_bias_variance: Vector3::repeat(0.01),
            gyro_bias_variance: Vector3::repeat(0.001),
        }
    }

    fn fix() -> GnssInitializationFix {
        GnssInitializationFix {
            time: SessionTime::from_ns(10),
            evidence_oldest_time: SessionTime::from_ns(10),
            position_n: Vector3::new(1.0, 2.0, 3.0),
            velocity_n: Vector3::zeros(),
            position_covariance_n: Matrix3::identity(),
            velocity_covariance_n: Matrix3::identity(),
            position_velocity_cross_n: Matrix3::zeros(),
            zero_velocity_nis: Some(0.0),
        }
    }

    fn feed_static(initializer: &mut Initializer, count: usize) {
        feed_static_force(initializer, count, Vector3::z() * 9.806_65);
    }

    fn feed_static_force(
        initializer: &mut Initializer,
        count: usize,
        specific_force_b: Vector3<f32>,
    ) {
        initializer.observe_gnss(fix()).unwrap();
        for index in 0..count {
            initializer
                .observe_imu(
                    SessionTime::from_ns(10 + index as i64),
                    1_000,
                    Vector3::new(0.0, 5.0e-5, 5.0e-5),
                    specific_force_b,
                )
                .unwrap();
        }
    }

    #[test]
    fn unobservable_yaw_still_initializes_without_claiming_heading() {
        let mut config = config();
        config.accel_bias_variance = Vector3::zeros();
        let mut initializer = Initializer::new(config).unwrap();
        feed_static(&mut initializer, 4);
        assert_eq!(initializer.phase, InitializationPhase::FineAligning);
        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();

        assert_eq!(result.heading_source, InitialHeadingSource::None);
        let mapped_up = result
            .state
            .orientation_n_from_b
            .transform_vector(&Vector3::z());
        assert!((mapped_up - Vector3::z()).norm() < 1.0e-6);
        assert!((result.covariance[(ATT, ATT)] - 1.0e-3).abs() < 1.0e-7);
        assert!((result.covariance[(ATT + 1, ATT + 1)] - 1.0e-3).abs() < 1.0e-7);
        assert!((result.covariance[(ATT + 2, ATT + 2)] - 10.0).abs() < 1.0e-6);
        assert_eq!(initializer.phase, InitializationPhase::Navigating);
    }

    #[test]
    fn opposite_gravity_direction_uses_a_deterministic_arbitrary_yaw() {
        let mut config = config();
        config.accel_bias_variance = Vector3::zeros();
        let mut initializer = Initializer::new(config).unwrap();
        feed_static_force(
            &mut initializer,
            4,
            -Vector3::z() * config.stationary.gravity_magnitude,
        );

        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();

        assert_eq!(result.heading_source, InitialHeadingSource::None);
        let mapped_up = result
            .state
            .orientation_n_from_b
            .transform_vector(&-Vector3::z());
        assert!((mapped_up - Vector3::z()).norm() < 1.0e-5);
    }

    #[test]
    fn accelerometer_bias_prior_is_removed_from_tilt_and_its_uncertainty_is_retained() {
        let mut config = config();
        config.accel_bias_prior = Vector3::new(0.0, 0.2, -0.3);
        config.accel_bias_variance = Vector3::new(0.04, 0.09, 0.16);
        let mut initializer = Initializer::new(config).unwrap();
        feed_static_force(
            &mut initializer,
            4,
            Vector3::new(config.stationary.gravity_magnitude, 0.2, -0.3),
        );

        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();
        let mapped_up = result
            .state
            .orientation_n_from_b
            .transform_vector(&Vector3::x());
        assert!((mapped_up - Vector3::z()).norm() < 1.0e-5);

        let gravity = config.stationary.gravity_magnitude;
        let attitude = result.covariance.fixed_view::<3, 3>(ATT, ATT);
        assert!((attitude[(0, 0)] - config.unobservable_yaw_variance).abs() < 1.0e-5);
        assert!(
            (attitude[(1, 1)]
                - (config.roll_pitch_variance + config.accel_bias_variance.z / gravity.powi(2)))
            .abs()
                < 1.0e-6
        );
        assert!(
            (attitude[(2, 2)]
                - (config.roll_pitch_variance + config.accel_bias_variance.y / gravity.powi(2)))
            .abs()
                < 1.0e-6
        );
        let attitude_bias = result.covariance.fixed_view::<3, 3>(ATT, ACC_BIAS);
        assert!((attitude_bias[(1, 2)] + config.accel_bias_variance.z / gravity).abs() < 1.0e-6);
        assert!((attitude_bias[(2, 1)] - config.accel_bias_variance.y / gravity).abs() < 1.0e-6);
    }

    #[test]
    fn nonrepresentable_bias_induced_tilt_uncertainty_fails_closed() {
        let mut config = config();
        config.accel_bias_prior = Vector3::z() * (config.stationary.gravity_magnitude - 4.0e-6);
        config.accel_bias_variance = Vector3::repeat(f32::MAX);
        let mut initializer = Initializer::new(config).unwrap();
        feed_static(&mut initializer, 4);

        assert_eq!(
            initializer.try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5)),
            Err(InitializationError::DegenerateAlignment)
        );
        assert_eq!(initializer.phase, InitializationPhase::FineAligning);
    }

    #[test]
    fn initialization_seeds_the_full_symmetric_position_velocity_cross_blocks() {
        let mut config = config();
        config.accel_bias_variance = Vector3::zeros();
        let mut initializer = Initializer::new(config).unwrap();
        let mut fix = fix();
        fix.position_velocity_cross_n = Matrix3::new(
            0.01, 0.02, 0.03, //
            0.04, 0.05, 0.06, //
            0.07, 0.08, 0.09,
        );
        initializer.observe_gnss(fix).unwrap();
        for index in 0..4 {
            initializer
                .observe_imu(
                    SessionTime::from_ns(10 + index),
                    1_000,
                    Vector3::new(0.0, 5.0e-5, 5.0e-5),
                    Vector3::z() * 9.806_65,
                )
                .unwrap();
        }

        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();
        assert_eq!(
            result.covariance.fixed_view::<3, 3>(POS, VEL),
            fix.position_velocity_cross_n
        );
        assert_eq!(
            result.covariance.fixed_view::<3, 3>(VEL, POS),
            fix.position_velocity_cross_n.transpose()
        );
    }

    #[test]
    fn supplied_heading_completes_alignment() {
        let mut initializer = Initializer::new(config()).unwrap();
        feed_static(&mut initializer, 4);
        initializer.provide_heading(0.7, 0.02).unwrap();
        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();
        assert_eq!(result.heading_source, InitialHeadingSource::Supplied);
        let forward = result
            .state
            .orientation_n_from_b
            .transform_vector(&Vector3::x());
        assert!((forward.y.atan2(forward.x) - 0.7).abs() < 1.0e-4);
        assert_eq!(initializer.phase, InitializationPhase::Navigating);
    }

    #[test]
    fn gyrocompass_uses_earth_rate_without_zeroing_the_mean_as_bias() {
        let mut config = config();
        config.gyrocompassing_qualified = true;
        let mut initializer = Initializer::new(config).unwrap();
        feed_static(&mut initializer, 8);
        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();
        assert_eq!(result.heading_source, InitialHeadingSource::Gyrocompass);
        assert_eq!(result.state.gyro_bias_b, Vector3::zeros());
    }

    #[test]
    fn unqualified_profile_never_emits_a_gyrocompass_heading() {
        let mut initializer = Initializer::new(config()).unwrap();
        feed_static(&mut initializer, 8);

        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();

        assert_eq!(result.heading_source, InitialHeadingSource::None);
        assert!((result.covariance[(ATT + 2, ATT + 2)] - 10.0).abs() < 1.0e-6);
    }

    #[test]
    fn invalid_dynamic_constraint_does_not_block_tilt_only_initialization() {
        let mut initializer = Initializer::new(config()).unwrap();
        feed_static(&mut initializer, 4);
        initializer
            .provide_dynamic_yaw(0.0, 0.01, 100.0, false)
            .unwrap();
        let result = initializer
            .try_initialize(Vector3::new(0.0, 5.0e-5, 5.0e-5))
            .unwrap()
            .unwrap();
        assert_eq!(result.heading_source, InitialHeadingSource::None);
    }

    #[test]
    fn unobservable_gyrocompass_geometry_falls_back_to_tilt_only_initialization() {
        let mut initializer = Initializer::new(config()).unwrap();
        feed_static(&mut initializer, 8);

        let result = initializer
            .try_initialize(Vector3::z() * 5.0e-5)
            .unwrap()
            .unwrap();

        assert_eq!(result.heading_source, InitialHeadingSource::None);
        assert!((result.covariance[(ATT + 2, ATT + 2)] - 10.0).abs() < 1.0e-6);
    }

    #[test]
    fn classifier_hysteresis_rejects_large_motion_score() {
        let mut classifier = StationaryClassifier::new(config().stationary).unwrap();
        classifier.observe_gnss_zero_velocity_nis(SessionTime::ZERO, Some(0.0));
        classifier
            .observe_imu(
                SessionTime::ZERO,
                1_000,
                Vector3::zeros(),
                Vector3::z() * 9.806_65,
            )
            .unwrap();
        classifier
            .observe_imu(
                SessionTime::from_ns(1),
                1_000,
                Vector3::zeros(),
                Vector3::z() * 9.806_65,
            )
            .unwrap();
        assert!(classifier.stationary());
        for index in 0..STATIONARY_WINDOW_CAPACITY {
            classifier
                .observe_imu(
                    SessionTime::from_ns(2 + index as i64),
                    1_000,
                    Vector3::repeat(10.0),
                    Vector3::repeat(30.0),
                )
                .unwrap();
        }
        assert!(!classifier.stationary());
    }

    #[test]
    fn missing_gnss_nis_is_not_perfect_stationary_evidence() {
        let mut missing = StationaryClassifier::new(config().stationary).unwrap();
        for time in 0..4 {
            missing
                .observe_imu(
                    SessionTime::from_ns(time),
                    10,
                    Vector3::zeros(),
                    Vector3::z() * 9.806_65,
                )
                .unwrap();
        }
        assert!(!missing.stationary());

        let mut measured = StationaryClassifier::new(config().stationary).unwrap();
        measured.observe_gnss_zero_velocity_nis(SessionTime::ZERO, Some(0.0));
        for time in 0..4 {
            measured
                .observe_imu(
                    SessionTime::from_ns(time),
                    10,
                    Vector3::zeros(),
                    Vector3::z() * 9.806_65,
                )
                .unwrap();
        }
        assert!(measured.stationary());
    }

    #[test]
    fn stale_gnss_evidence_unlatches_and_cannot_seed_alignment() {
        let mut initializer = Initializer::new(config()).unwrap();
        initializer.observe_gnss(fix()).unwrap();
        initializer
            .observe_imu(
                SessionTime::from_ns(10),
                5,
                Vector3::zeros(),
                Vector3::z() * 9.806_65,
            )
            .unwrap();
        initializer
            .observe_imu(
                SessionTime::from_ns(11),
                5,
                Vector3::zeros(),
                Vector3::z() * 9.806_65,
            )
            .unwrap();
        assert!(initializer.classifier.stationary());

        initializer
            .observe_imu(
                SessionTime::from_ns(16),
                5,
                Vector3::zeros(),
                Vector3::z() * 9.806_65,
            )
            .unwrap();
        assert!(!initializer.classifier.stationary());
        assert!(initializer.latest_fix.is_none());
        assert_eq!(initializer.phase, InitializationPhase::Uninitialized);
    }
}
