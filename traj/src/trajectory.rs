//! Opaque continuous rigid-body trajectory and physically qualified queries.
//!
//! Dense segment coefficients and storage are private. Public callers query
//! kinematics at exact session times or ask the engine-owned metric evaluator
//! to consume the path.

#[cfg(feature = "offline")]
use self::bridge::DenseConditionalBridge;
use self::dense::DenseSegment;
use self::math::query_to_metric;
#[cfg(feature = "offline")]
use self::storage::OfflineSegmentBacking;
use self::storage::{SegmentStorage, new_segment_storage, push_segment};
use crate::config::AttachmentModel;
use crate::error::{QueryError, ValidationError};
#[cfg(any(test, feature = "offline"))]
use crate::frame::ReferencePointKind;
use crate::frame::{
    BodyAngularAcceleration, BodyAngularRate, BodyVector, EcefAcceleration, EcefPosition,
    EcefVelocity, OrientationEcefFromBody, OutputFrame, ReferencePoint, TerrestrialFrame,
};
use crate::ids::{ReferencePointId, TrajectoryRevision};
use crate::metric::MetricError;
use crate::quality::{EstimateQuality, FieldValue, ObservabilityReport};
use crate::time::{SessionTime, TimeSpan};
use crate::uncertainty::KinematicCovariance;
use heapless::Vec as FixedVec;
#[cfg(feature = "offline")]
use std::boxed::Box;
#[cfg(feature = "offline")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "offline")]
mod bridge;
#[cfg(feature = "offline")]
mod codec;
mod dense;
mod jets;
mod math;
mod metric_queries;
mod quality;
mod query;
mod roots;
mod storage;
#[cfg(feature = "offline")]
pub(crate) use bridge::{DenseBridgeInput, DenseBridgeLinearization};
#[cfg(feature = "offline")]
pub(crate) use storage::OfflineTrajectoryStorageBounds;

/// Maximum dense segments retained in an allocator-free rolling trajectory.
pub const MAX_EMBEDDED_TRAJECTORY_SEGMENTS: usize = 208;

/// Maximum named rigid reference points in one trajectory.
pub const MAX_REFERENCE_POINTS: usize = 16;

/// Maximum roots returned from a single dense interval.
pub(crate) const MAX_SEGMENT_ROOTS: usize = 12;

/// Absolute implementation ceiling for interval and point-enclosure
/// evaluations for one scalar function on one dense segment. A live
/// trajectory carries the smaller budget selected by its compiled plan.
pub(crate) const MAX_ROOT_ISOLATION_EVALUATIONS: u32 = 2_048;

/// Complete endpoint information used to construct a dense segment. This is
/// crate-visible so the live filter and host smoother can share one trajectory
/// implementation without making coefficients part of the public interface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct TrajectoryKnot {
    pub time: SessionTime,
    pub position_ecef: EcefPosition,
    pub velocity_ecef: EcefVelocity,
    pub orientation_ecef_from_body: OrientationEcefFromBody,
    pub specific_force_body: BodyVector,
    pub covariance: KinematicCovariance,
    pub quality: EstimateQuality,
    pub observability: ObservabilityReport,
}

/// Public kinematic state at one named physical point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicEstimate {
    pub time: SessionTime,
    pub reference_point: ReferencePointId,
    pub frame: OutputFrame,
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    pub orientation_ecef_from_body: OrientationEcefFromBody,
    pub angular_rate_body_relative_ecef: BodyAngularRate,
    /// Angular acceleration is unavailable until the selected dense model has
    /// a qualified derivative and uncertainty bandwidth.
    pub angular_acceleration_body_relative_ecef: FieldValue<BodyAngularAcceleration>,
    /// Translational acceleration is available at the IMU origin. An offset
    /// point additionally requires observable angular acceleration.
    pub kinematic_acceleration: FieldValue<EcefAcceleration>,
    pub specific_force_body: BodyVector,
    pub covariance: KinematicCovariance,
    pub quality: EstimateQuality,
    pub observability: ObservabilityReport,
    pub revision: TrajectoryRevision,
}

/// Scalar/vector fields used privately by metric evaluation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ScalarKinematics {
    pub time: SessionTime,
    pub position_ecef_m: [f64; 3],
    pub velocity_ecef_mps: [f64; 3],
    pub acceleration_ecef_mps2: Option<[f64; 3]>,
    pub horizontal_speed_mps: f64,
    pub vertical_speed_mps: f64,
    pub body_longitudinal_speed_mps: Option<f64>,
    /// Conservative quality of the dense state used for this scalar view.
    pub quality: EstimateQuality,
}

/// Engine-owned continuous trajectory. Its storage and interpolation model are
/// deliberately opaque to callers.
#[derive(Clone, Debug)]
pub struct Trajectory {
    frame: TerrestrialFrame,
    revision: TrajectoryRevision,
    attachment: AttachmentModel,
    reference_points: FixedVec<ReferencePoint, MAX_REFERENCE_POINTS>,
    segments: SegmentStorage,
    #[cfg(feature = "offline")]
    conditional_bridges: std::vec::Vec<Option<Box<DenseConditionalBridge>>>,
    #[cfg(feature = "offline")]
    offline_backing: Option<Arc<Mutex<OfflineSegmentBacking>>>,
    #[cfg(feature = "offline")]
    offline_segment_count: usize,
    #[cfg(feature = "offline")]
    offline_span: Option<TimeSpan>,
    root_evaluation_budget: u32,
}

impl Trajectory {
    /// Creates an empty engine-owned path. Kept crate-visible so external
    /// callers cannot manufacture a trajectory independently of processing.
    #[must_use]
    pub(crate) const fn new(frame: TerrestrialFrame, revision: TrajectoryRevision) -> Self {
        Self {
            frame,
            revision,
            // Construction without a bound installation must fail closed:
            // sensor/antenna kinematics remain useful, while vehicle/person
            // rigid-point and body-forward claims require an explicit upgrade.
            attachment: AttachmentModel::DeviceTrajectoryOnly,
            reference_points: FixedVec::new(),
            segments: new_segment_storage(),
            #[cfg(feature = "offline")]
            conditional_bridges: std::vec::Vec::new(),
            #[cfg(feature = "offline")]
            offline_backing: None,
            #[cfg(feature = "offline")]
            offline_segment_count: 0,
            #[cfg(feature = "offline")]
            offline_span: None,
            root_evaluation_budget: MAX_ROOT_ISOLATION_EVALUATIONS,
        }
    }

    /// Reuses caller-owned segment storage for a new engine revision without
    /// constructing or moving another large trajectory value.
    pub(crate) fn reset(&mut self, frame: TerrestrialFrame, revision: TrajectoryRevision) {
        self.frame = frame;
        self.revision = revision;
        self.attachment = AttachmentModel::DeviceTrajectoryOnly;
        self.reference_points.clear();
        self.segments.clear();
        #[cfg(feature = "offline")]
        self.conditional_bridges.clear();
        #[cfg(feature = "offline")]
        {
            self.offline_backing = None;
            self.offline_segment_count = 0;
            self.offline_span = None;
        }
        self.root_evaluation_budget = MAX_ROOT_ISOLATION_EVALUATIONS;
    }

    /// Binds the immutable installation attachment before trajectory values
    /// are produced. A populated trajectory cannot be relabelled afterward.
    pub(crate) fn set_attachment_model(
        &mut self,
        attachment: AttachmentModel,
    ) -> Result<(), ValidationError> {
        if self.segment_count() != 0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.attachment = attachment;
        Ok(())
    }

    /// Physical attachment claim governing reference-point and body-axis
    /// outputs from this handle.
    #[must_use]
    pub const fn attachment_model(&self) -> AttachmentModel {
        self.attachment
    }

    /// Ends the current rolling navigation span without discarding the
    /// immutable reference-point contract compiled for this session.
    ///
    /// A later reinitialization starts a distinct trajectory span. Keeping
    /// old and new dense segments in the same handle would falsely imply that
    /// the unrepresented interval between them was continuous.
    pub(crate) fn clear_segments_preserving_reference_points(&mut self) {
        self.segments.clear();
        #[cfg(feature = "offline")]
        self.conditional_bridges.clear();
        #[cfg(feature = "offline")]
        {
            self.offline_backing = None;
            self.offline_segment_count = 0;
            self.offline_span = None;
        }
    }

    /// Applies the already-validated fixed live root budget. This is set at
    /// session start, outside both the IMU/filter path and metric traversal.
    pub(crate) fn set_root_evaluation_budget(
        &mut self,
        evaluations: u32,
    ) -> Result<(), ValidationError> {
        if evaluations == 0 || evaluations > MAX_ROOT_ISOLATION_EVALUATIONS {
            return Err(ValidationError::CapacityExceeded);
        }
        self.root_evaluation_budget = evaluations;
        Ok(())
    }

    pub(crate) fn add_reference_point(
        &mut self,
        point: ReferencePoint,
    ) -> Result<(), ValidationError> {
        if self
            .reference_points
            .iter()
            .any(|present| present.id() == point.id())
        {
            return Err(ValidationError::InvalidReferencePoint);
        }
        self.reference_points
            .push(point)
            .map_err(|_| ValidationError::CapacityExceeded)
    }

    fn reference_point_for_query(
        &self,
        id: ReferencePointId,
    ) -> Result<ReferencePoint, QueryError> {
        let point = self
            .reference_points
            .iter()
            .find(|point| point.id() == id)
            .copied()
            .ok_or(QueryError::ReferencePointUnavailable)?;
        if !self.attachment.permits_reference_point(point.kind()) {
            return Err(QueryError::ReferencePointUnavailable);
        }
        Ok(point)
    }

    #[cfg(any(test, feature = "offline"))]
    pub(crate) fn configured_reference_point_kind(
        &self,
        id: ReferencePointId,
    ) -> Result<ReferencePointKind, MetricError> {
        self.reference_points
            .iter()
            .find(|point| point.id() == id)
            .map(|point| point.kind())
            .ok_or(MetricError::ReferencePointUnavailable)
    }

    fn reference_point_for_metric(
        &self,
        id: ReferencePointId,
    ) -> Result<ReferencePoint, MetricError> {
        self.reference_point_for_query(id).map_err(query_to_metric)
    }

    #[must_use]
    pub(crate) const fn body_axis_metric_outputs_available(&self) -> bool {
        self.attachment.permits_body_axis_quantities()
    }

    #[cfg(test)]
    pub(crate) fn push_hermite_segment(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
    ) -> Result<(), ValidationError> {
        if let Some(previous) = self.segments.last() {
            if start.time < previous.end.time {
                return Err(ValidationError::InvalidTimeSpan);
            }
        }
        let segment = DenseSegment::new(start, end)?;
        push_segment(&mut self.segments, segment)?;
        #[cfg(feature = "offline")]
        self.conditional_bridges.push(None);
        Ok(())
    }

    /// Appends a corrected-IMU, endpoint-conditioned host segment.
    #[cfg(all(test, feature = "offline"))]
    pub(crate) fn push_conditional_bridge_segment(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        input: DenseBridgeInput,
    ) -> Result<(), ValidationError> {
        if let Some(previous) = self.segments.last() {
            if start.time < previous.end.time {
                return Err(ValidationError::InvalidTimeSpan);
            }
        }
        let (segment, bridge) = DenseSegment::new_conditional(start, end, &input)?;
        self.segments.push(segment);
        self.conditional_bridges.push(Some(Box::new(bridge)));
        debug_assert_eq!(self.segments.len(), self.conditional_bridges.len());
        Ok(())
    }

    /// Appends one live segment while retaining only the compiled rolling
    /// horizon. Removal is bounded by the fixed V2 Mini capacity and occurs
    /// only at the navigation cadence, never at the raw IMU rate.
    #[cfg(test)]
    pub(crate) fn push_rolling_hermite_segment(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
    ) -> Result<(), ValidationError> {
        // Validate the replacement before evicting the oldest live segment.
        // A malformed new segment must leave the rolling trajectory exactly
        // unchanged so the public live step can uphold its transaction seam.
        if let Some(previous) = self.segments.last() {
            if start.time < previous.end.time {
                return Err(ValidationError::InvalidTimeSpan);
            }
        }
        let segment = DenseSegment::new(start, end)?;
        if self.segments.len() == MAX_EMBEDDED_TRAJECTORY_SEGMENTS {
            self.segments.remove(0);
            #[cfg(feature = "offline")]
            self.conditional_bridges.remove(0);
        }
        push_segment(&mut self.segments, segment)?;
        #[cfg(feature = "offline")]
        self.conditional_bridges.push(None);
        Ok(())
    }

    /// Appends one live segment whose nominal attitude came from corrected
    /// IMU propagation and whose residual endpoint error is conditioned over
    /// the segment in SO(3).
    pub(crate) fn push_rolling_imu_segment(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        integrated_rotation_body: [f64; 3],
    ) -> Result<(), ValidationError> {
        if let Some(previous) = self.segments.last() {
            if start.time < previous.end.time {
                return Err(ValidationError::InvalidTimeSpan);
            }
        }
        let segment = DenseSegment::new_imu_conditioned(start, end, integrated_rotation_body)?;
        if self.segments.len() == MAX_EMBEDDED_TRAJECTORY_SEGMENTS {
            self.segments.remove(0);
            #[cfg(feature = "offline")]
            self.conditional_bridges.remove(0);
        }
        push_segment(&mut self.segments, segment)?;
        #[cfg(feature = "offline")]
        self.conditional_bridges.push(None);
        Ok(())
    }

    /// Copies every finalized segment not yet present in a host accumulator.
    /// A discontinuity caused by allowing the rolling source to evict unseen
    /// data is rejected instead of silently publishing a truncated replay.
    #[cfg(feature = "offline")]
    pub(crate) fn append_unseen_segments_from(
        &mut self,
        rolling: &Trajectory,
        committed: Option<TimeSpan>,
    ) -> Result<(), ValidationError> {
        let last_end = self.segments.last().map(|segment| segment.end.time);
        if let Some(committed) = committed {
            let retained = rolling.span().ok_or(ValidationError::InvalidTimeSpan)?;
            if !retained.contains(committed.start()) || !retained.contains(committed.end()) {
                return Err(ValidationError::CapacityExceeded);
            }
            if last_end.is_none() && retained.start() != committed.start() {
                return Err(ValidationError::CapacityExceeded);
            }
        }
        let mut candidates = rolling
            .segments
            .iter()
            .enumerate()
            .filter(|(_, segment)| last_end.is_none_or(|end| segment.end.time > end));
        if let (Some(end), Some((_, first))) = (last_end, candidates.clone().next()) {
            if first.start.time != end {
                return Err(ValidationError::InvalidTimeSpan);
            }
        }
        for (index, segment) in candidates.by_ref() {
            self.segments.push(*segment);
            self.conditional_bridges.push(
                rolling
                    .conditional_bridges
                    .get(index)
                    .cloned()
                    .ok_or(ValidationError::InvalidTimeSpan)?,
            );
        }
        Ok(())
    }

    /// Visits newly appended knots starting at a stable full-path ordinal.
    /// The first segment contributes ordinals zero and one; each later segment
    /// contributes only its end knot. This lets a transaction stage a growing
    /// replay path once without rescanning or duplicating its immutable prefix.
    #[cfg(feature = "offline")]
    pub(crate) fn try_for_each_knot_from<E>(
        &self,
        first_ordinal: u64,
        mut visit: impl FnMut(TrajectoryKnot) -> Result<(), E>,
    ) -> Result<u64, E> {
        let Some(first) = self.segments.first() else {
            return Ok(0);
        };
        let total = u64::try_from(self.segments.len())
            .unwrap_or(u64::MAX)
            .saturating_add(1);
        if first_ordinal == 0 {
            visit(first.start)?;
        }
        let first_segment = usize::try_from(first_ordinal.saturating_sub(1)).unwrap_or(usize::MAX);
        for segment in self.segments.iter().skip(first_segment) {
            visit(segment.end)?;
        }
        Ok(total)
    }

    /// Returns the complete available span, if at least one dense segment is
    /// present.
    #[must_use]
    pub fn span(&self) -> Option<TimeSpan> {
        #[cfg(feature = "offline")]
        if self.offline_backing.is_some() {
            return self.offline_span;
        }
        TimeSpan::new(
            self.segments.first()?.start.time,
            self.segments.last()?.end.time,
        )
        .ok()
    }

    #[must_use]
    pub const fn frame(&self) -> TerrestrialFrame {
        self.frame
    }

    #[must_use]
    pub const fn revision(&self) -> TrajectoryRevision {
        self.revision
    }

    pub(crate) fn segment_count(&self) -> usize {
        #[cfg(feature = "offline")]
        if self.offline_backing.is_some() {
            return self.offline_segment_count;
        }
        self.segments.len()
    }
}

#[cfg(test)]
mod tests;
