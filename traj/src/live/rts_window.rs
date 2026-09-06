//! Fixed-capacity storage and resumable single-pass RTS publication.

use nalgebra::{ArrayStorage, Matrix3};

use super::{
    core::GnssQualityUpdate,
    dense_history::{DenseCovariance, DenseEndpoint, DenseHistoryError, DenseSegment},
    eskf::{ConsiderCovariance, Eskf, GapNavCrossCovariance, RtsUpdateCapture},
    preintegration::ImuSampleCovariance,
    reanchor::{ReanchorError, ReanchorTransform},
    smoothing::{
        AUG_DIM, AugmentedNavCross, RtsCovariance, RtsEstimate, RtsScratch, RtsStep, RtsTransition,
        SmoothingError, backward_step,
    },
    state::{NAV_DIM, NavState},
};
use crate::time::SessionTime;

pub(super) const MAX_SMOOTHING_LAG_NS: i64 = 100_000_000;
pub(super) const RTS_NODE_CAPACITY: usize = 64;
pub(super) const RTS_STEP_CREDITS: u16 = 64;

#[derive(Debug, PartialEq)]
struct RtsNode {
    predicted_state: NavState,
    filtered_state: NavState,
    filtered_reset: Matrix3<f32>,
    predicted: RtsCovariance,
    filtered: RtsCovariance,
    predicted_filtered: AugmentedNavCross,
    transition: RtsTransition,
    incoming: Option<DenseSegment>,
    endpoint: DenseEndpoint,
    quality: Option<GnssQualityUpdate>,
}

impl RtsNode {
    const fn new() -> Self {
        Self {
            predicted_state: NavState::placeholder(),
            filtered_state: NavState::placeholder(),
            filtered_reset: Matrix3::from_array_storage(ArrayStorage([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ])),
            predicted: RtsCovariance::new(),
            filtered: RtsCovariance::new(),
            predicted_filtered: AugmentedNavCross::from_array_storage(ArrayStorage(
                [[0.0; AUG_DIM]; NAV_DIM],
            )),
            transition: RtsTransition::new(),
            incoming: None,
            endpoint: DenseEndpoint::placeholder(),
            quality: None,
        }
    }
}

#[derive(Debug, PartialEq)]
pub(super) struct RtsWindow {
    nodes: [RtsNode; RTS_NODE_CAPACITY],
    head: usize,
    len: usize,
    dirty: bool,
    cursor: Option<usize>,
    frozen_start: Option<DenseEndpoint>,
    consider: ConsiderCovariance,
    active_consider: usize,
    next: RtsEstimate,
    candidate: RtsEstimate,
    scratch: RtsScratch,
}

impl RtsWindow {
    pub(super) const fn new() -> Self {
        Self {
            nodes: [const { RtsNode::new() }; RTS_NODE_CAPACITY],
            head: 0,
            len: 0,
            dirty: false,
            cursor: None,
            frozen_start: None,
            consider: super::eskf::zero_consider_covariance(),
            active_consider: 0,
            next: RtsEstimate::new(),
            candidate: RtsEstimate::new(),
            scratch: RtsScratch::new(),
        }
    }

    pub(super) const fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub(super) const fn has_tail(&self) -> bool {
        self.len > 1
    }
    pub(super) const fn is_busy(&self) -> bool {
        self.cursor.is_some()
    }
    pub(super) const fn is_full(&self) -> bool {
        self.len == RTS_NODE_CAPACITY
    }

    pub(super) fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.dirty = false;
        self.cursor = None;
        self.frozen_start = None;
        self.consider.fill(0.0);
        self.active_consider = 0;
    }

    fn index(&self, offset: usize) -> usize {
        (self.head + offset) % RTS_NODE_CAPACITY
    }

    pub(super) fn seed(
        &mut self,
        filter: &Eskf,
        sample: &ImuSampleCovariance,
        sample_cross: &GapNavCrossCovariance,
        endpoint: DenseEndpoint,
        quality: Option<GnssQualityUpdate>,
    ) {
        debug_assert!(self.is_empty());
        self.consider.copy_from(&filter.consider_covariance);
        self.active_consider = filter.active_consider;
        self.len = 1;
        let node = &mut self.nodes[self.head];
        capture_covariance(&mut node.predicted, filter, sample, sample_cross);
        node.filtered.copy_from(&node.predicted);
        node.predicted_state = filter.state;
        node.filtered_state = filter.state;
        node.filtered_reset = Matrix3::identity();
        node.endpoint = endpoint;
        node.incoming = None;
        node.quality = quality;
        for row in 0..AUG_DIM {
            for column in 0..NAV_DIM {
                node.predicted_filtered[(row, column)] =
                    node.predicted.joint_entry(row, column, &self.consider);
            }
        }
    }

    pub(super) fn push_prediction(
        &mut self,
        filter: &Eskf,
        sample: &ImuSampleCovariance,
        sample_cross: &GapNavCrossCovariance,
        transition: &RtsTransition,
        quality: Option<GnssQualityUpdate>,
    ) {
        debug_assert!(self.len > 0 && !self.is_full() && !self.is_busy());
        let index = self.index(self.len);
        let node = &mut self.nodes[index];
        node.predicted_state = filter.state;
        node.filtered_state = filter.state;
        node.filtered_reset = Matrix3::identity();
        capture_covariance(&mut node.predicted, filter, sample, sample_cross);
        node.filtered.copy_from(&node.predicted);
        node.transition.nav.copy_from(&transition.nav);
        node.transition.retain_sample = transition.retain_sample;
        node.transition.retain_gap = transition.retain_gap;
        node.incoming = None;
        node.quality = quality;
        node.endpoint = DenseEndpoint {
            state: filter.state,
            specific_force_b: nalgebra::Vector3::zeros(),
            covariance: DenseCovariance::from_navigation(&filter.covariance),
        };
        for row in 0..AUG_DIM {
            for column in 0..NAV_DIM {
                node.predicted_filtered[(row, column)] =
                    node.predicted.joint_entry(row, column, &self.consider);
            }
        }
        self.len += 1;
    }

    /// Selecting a new right-owned sample at the prediction epoch changes
    /// only its latent coordinates. No navigation information is added.
    pub(super) fn replace_predicted_sample(&mut self, sample: &ImuSampleCovariance) {
        let index = self.index(self.len - 1);
        let node = &mut self.nodes[index];
        node.predicted.sample.copy_from(sample);
        node.predicted.nav_sample.fill(0.0);
        node.filtered.sample.copy_from(sample);
        node.filtered.nav_sample.fill(0.0);
        node.transition.retain_sample = false;
        for row in super::smoothing::SAMPLE_START..super::smoothing::GAP_START {
            for column in 0..NAV_DIM {
                node.predicted_filtered[(row, column)] = 0.0;
            }
        }
    }

    pub(super) fn finish_node(
        &mut self,
        filter: &Eskf,
        sample: &ImuSampleCovariance,
        sample_cross: &GapNavCrossCovariance,
        update: &RtsUpdateCapture,
        incoming: DenseSegment,
        quality: Option<GnssQualityUpdate>,
    ) {
        let index = self.index(self.len - 1);
        let node = &mut self.nodes[index];
        node.filtered_state = filter.state;
        node.filtered_reset.copy_from(&update.attitude_reset);
        capture_covariance(&mut node.filtered, filter, sample, sample_cross);
        node.endpoint = incoming.end;
        node.incoming = Some(incoming);
        node.quality = quality;
        // B = Cov(predicted augmented error, filtered navigation error).
        // Nuisance errors use zero Schmidt gain and keep their coordinates.
        let unchanged = (0..NAV_DIM).all(|r| {
            (0..AUG_DIM).all(|c| update.nav_transform[(r, c)] == if r == c { 1.0 } else { 0.0 })
        });
        for row in 0..AUG_DIM {
            for column in 0..NAV_DIM {
                if unchanged {
                    node.predicted_filtered[(row, column)] =
                        node.predicted.joint_entry(row, column, &self.consider);
                    continue;
                }
                let mut value = 0.0;
                for inner in 0..AUG_DIM {
                    let coefficient = update.nav_transform[(column, inner)];
                    if coefficient != 0.0 {
                        value +=
                            node.predicted.joint_entry(row, inner, &self.consider) * coefficient;
                    }
                }
                node.predicted_filtered[(row, column)] = value;
            }
        }
    }

    pub(super) fn observation_accepted(&mut self) {
        self.dirty = true;
    }

    pub(super) fn ready(&self, at: SessionTime, lag_ns: i64, flush: bool) -> bool {
        if self.len < 2 {
            return false;
        }
        let time = self.nodes[self.index(1)].filtered_state.time;
        flush || at.as_ns().saturating_sub(time.as_ns()) >= lag_ns
    }

    /// One backward node per quota charge; while a pass is active the caller
    /// does not advance the forward filter, keeping all references immutable.
    pub(super) fn smooth_one(&mut self) -> Result<bool, SmoothingError> {
        if !self.dirty {
            return Ok(true);
        }
        if self.cursor.is_none() {
            let last = &self.nodes[self.index(self.len - 1)];
            self.next.state = last.filtered_state;
            self.next
                .predicted_to_smoothed_reset
                .copy_from(&last.filtered_reset);
            self.next.covariance.copy_from(&last.filtered);
            self.next
                .predicted_smoothed_cross
                .copy_from(&last.predicted_filtered);
            self.cursor = self.len.checked_sub(2);
            if self.cursor.is_none() {
                self.dirty = false;
                return Ok(true);
            }
        }
        let offset = self.cursor.expect("nonempty RTS backward pass");
        let current = &self.nodes[self.index(offset)];
        let next = &self.nodes[self.index(offset + 1)];
        let step = RtsStep {
            filtered_state: &current.filtered_state,
            filtered_reset: &current.filtered_reset,
            filtered: &current.filtered,
            predicted: &current.predicted,
            predicted_filtered_cross: &current.predicted_filtered,
            next_transition: &next.transition,
            next_predicted_state: &next.predicted_state,
            next_predicted: &next.predicted,
            consider: &self.consider,
            active_consider: self.active_consider,
        };
        backward_step(&step, &self.next, &mut self.candidate, &mut self.scratch)?;
        let index = self.index(offset);
        self.nodes[index].endpoint.state = self.candidate.state;
        self.nodes[index].endpoint.covariance =
            DenseCovariance::from_navigation(&self.candidate.covariance.nav);
        core::mem::swap(&mut self.next, &mut self.candidate);
        self.cursor = offset.checked_sub(1);
        if self.cursor.is_none() {
            self.dirty = false;
        }
        Ok(!self.dirty)
    }

    pub(super) fn publish_one(
        &mut self,
    ) -> Result<
        (
            DenseSegment,
            Option<GnssQualityUpdate>,
            Option<GnssQualityUpdate>,
        ),
        DenseHistoryError,
    > {
        debug_assert!(self.len >= 2 && !self.dirty);
        let first = &self.nodes[self.head];
        let next = &self.nodes[self.index(1)];
        let incoming = next.incoming.ok_or(DenseHistoryError::Corrupt)?;
        let start = self.frozen_start.unwrap_or(first.endpoint);
        let end = next.endpoint;
        let segment = DenseSegment::new_imu_conditioned(
            incoming.id,
            start,
            end,
            incoming.integrated_attitude_delta(),
            incoming.degraded,
            incoming.degraded_input,
        )?;
        let quality = (first.quality, next.quality);
        self.frozen_start = Some(end);
        self.head = self.index(1);
        self.len -= 1;
        Ok((segment, quality.0, quality.1))
    }

    pub(super) fn validate_reanchor(
        &self,
        transform: &ReanchorTransform,
    ) -> Result<(), ReanchorError> {
        if self.is_busy() {
            return Err(ReanchorError::NonFinite);
        }
        for offset in 0..self.len {
            let node = &self.nodes[self.index(offset)];
            transform.validate_state(node.predicted_state)?;
            transform.validate_state(node.filtered_state)?;
            node.endpoint.mapped_reanchor(transform)?;
            validate_covariance_reanchor(&node.predicted, transform)?;
            validate_covariance_reanchor(&node.filtered, transform)?;
            validate_cross_reanchor(&node.predicted_filtered, transform)?;
            validate_transition_reanchor(&node.transition, transform)?;
            if let Some(incoming) = node.incoming {
                incoming.start.mapped_reanchor(transform)?;
                incoming.end.mapped_reanchor(transform)?;
            }
        }
        if let Some(endpoint) = self.frozen_start {
            endpoint.mapped_reanchor(transform)?;
        }
        Ok(())
    }

    pub(super) fn apply_reanchor(&mut self, transform: &ReanchorTransform) {
        for offset in 0..self.len {
            let index = self.index(offset);
            let node = &mut self.nodes[index];
            node.predicted_state = transform.map_state(node.predicted_state);
            node.filtered_state = transform.map_state(node.filtered_state);
            node.endpoint = node
                .endpoint
                .mapped_reanchor(transform)
                .expect("validated RTS endpoint");
            map_covariance(&mut node.predicted, transform);
            map_covariance(&mut node.filtered, transform);
            map_augmented_nav_cross(&mut node.predicted_filtered, transform);
            // Fnew = J F J^-1; J is an orthogonal frame rotation.
            let mut row = [0.0_f32; AUG_DIM];
            for axis in 0..NAV_DIM {
                for column in 0..AUG_DIM {
                    row[column] = (0..NAV_DIM)
                        .map(|k| {
                            transform.covariance_jacobian[(axis, k)]
                                * node.transition.nav[(k, column)]
                        })
                        .sum();
                }
                // Stage all output rows in the existing estimate cross scratch.
                for column in 0..AUG_DIM {
                    self.candidate.predicted_smoothed_cross[(column, axis)] = row[column];
                }
            }
            for axis in 0..NAV_DIM {
                for column in 0..AUG_DIM {
                    node.transition.nav[(axis, column)] = if column < NAV_DIM {
                        (0..NAV_DIM)
                            .map(|k| {
                                self.candidate.predicted_smoothed_cross[(k, axis)]
                                    * transform.covariance_jacobian[(column, k)]
                            })
                            .sum()
                    } else {
                        self.candidate.predicted_smoothed_cross[(column, axis)]
                    };
                }
            }
            if let Some(incoming) = node.incoming.as_mut() {
                incoming.start = incoming
                    .start
                    .mapped_reanchor(transform)
                    .expect("validated RTS start");
                incoming.end = incoming
                    .end
                    .mapped_reanchor(transform)
                    .expect("validated RTS end");
            }
        }
        if let Some(endpoint) = self.frozen_start.as_mut() {
            *endpoint = endpoint
                .mapped_reanchor(transform)
                .expect("validated frozen endpoint");
        }
    }
}

fn capture_covariance(
    target: &mut RtsCovariance,
    filter: &Eskf,
    sample: &ImuSampleCovariance,
    sample_cross: &GapNavCrossCovariance,
) {
    target.nav.copy_from(&filter.covariance);
    target
        .nav_consider
        .copy_from(&filter.nav_consider_covariance);
    target.nav_sample.copy_from(sample_cross);
    target.sample.copy_from(sample);
    target.nav_gap.copy_from(&filter.gap_nav_cross_covariance);
    target.gap.copy_from(&filter.gap_derivative_covariance);
}

/// Preflight exactly the retained matrix transforms using small row buffers;
/// a failed reanchor must not modify the window or overflow after hot commit.
fn validate_covariance_reanchor(
    covariance: &RtsCovariance,
    transform: &ReanchorTransform,
) -> Result<(), ReanchorError> {
    let j = &transform.covariance_jacobian;
    for row in 0..NAV_DIM {
        let mut mapped_row = [0.0; NAV_DIM];
        for (column, value) in mapped_row.iter_mut().enumerate() {
            *value = (0..NAV_DIM)
                .map(|k| j[(row, k)] * covariance.nav[(k, column)])
                .sum();
        }
        for column in 0..NAV_DIM {
            let value: f32 = (0..NAV_DIM).map(|k| mapped_row[k] * j[(column, k)]).sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
        for column in 0..super::eskf::MAX_CONSIDER {
            let value: f32 = (0..NAV_DIM)
                .map(|k| j[(row, k)] * covariance.nav_consider[(k, column)])
                .sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
        for column in 0..super::preintegration::BIAS_DIM {
            let sample: f32 = (0..NAV_DIM)
                .map(|k| j[(row, k)] * covariance.nav_sample[(k, column)])
                .sum();
            let gap: f32 = (0..NAV_DIM)
                .map(|k| j[(row, k)] * covariance.nav_gap[(k, column)])
                .sum();
            if !sample.is_finite() || !gap.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
    }
    Ok(())
}

fn validate_cross_reanchor(
    cross: &AugmentedNavCross,
    transform: &ReanchorTransform,
) -> Result<(), ReanchorError> {
    let j = &transform.covariance_jacobian;
    for column in 0..NAV_DIM {
        let mut nav_column = [0.0; NAV_DIM];
        for row in 0..AUG_DIM {
            let value: f32 = (0..NAV_DIM).map(|k| cross[(row, k)] * j[(column, k)]).sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
            if row < NAV_DIM {
                nav_column[row] = value;
            }
        }
        for row in 0..NAV_DIM {
            let value: f32 = (0..NAV_DIM).map(|k| j[(row, k)] * nav_column[k]).sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
    }
    Ok(())
}

fn validate_transition_reanchor(
    transition: &RtsTransition,
    transform: &ReanchorTransform,
) -> Result<(), ReanchorError> {
    let j = &transform.covariance_jacobian;
    for row in 0..NAV_DIM {
        let mut mapped_row = [0.0; AUG_DIM];
        for (column, value) in mapped_row.iter_mut().enumerate() {
            *value = (0..NAV_DIM)
                .map(|k| j[(row, k)] * transition.nav[(k, column)])
                .sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
        for column in 0..NAV_DIM {
            let value: f32 = (0..NAV_DIM).map(|k| mapped_row[k] * j[(column, k)]).sum();
            if !value.is_finite() {
                return Err(ReanchorError::NonFinite);
            }
        }
    }
    Ok(())
}

fn map_covariance(covariance: &mut RtsCovariance, transform: &ReanchorTransform) {
    let j = &transform.covariance_jacobian;
    covariance.nav = j * covariance.nav * j.transpose();
    covariance.nav_consider = j * covariance.nav_consider;
    covariance.nav_sample = j * covariance.nav_sample;
    covariance.nav_gap = j * covariance.nav_gap;
}

fn map_augmented_nav_cross(cross: &mut AugmentedNavCross, transform: &ReanchorTransform) {
    let j = &transform.covariance_jacobian;
    // Independent tiny row/column staging avoids an augmented stack matrix.
    let mut row = [0.0; NAV_DIM];
    for r in 0..AUG_DIM {
        for c in 0..NAV_DIM {
            row[c] = (0..NAV_DIM).map(|k| cross[(r, k)] * j[(c, k)]).sum();
        }
        for c in 0..NAV_DIM {
            cross[(r, c)] = row[c];
        }
    }
    for c in 0..NAV_DIM {
        for r in 0..NAV_DIM {
            row[r] = (0..NAV_DIM).map(|k| j[(r, k)] * cross[(k, c)]).sum();
        }
        for r in 0..NAV_DIM {
            cross[(r, c)] = row[r];
        }
    }
}
