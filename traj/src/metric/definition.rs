//! Physical metric quantities, event geometry, and definition types.

use super::{
    MAX_ACTIVITY_SPLITS, MAX_DRAG_TARGETS, MAX_LAP_GATES,
    geometry::{all_finite, cross, dot, normalize, scale, sub},
};
use crate::{
    config::AttachmentModel,
    error::ValidationError,
    frame::ReferencePointKind,
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId, SharedParameterId, TargetId},
    time::{DurationNs, SessionTime},
};
use heapless::Vec as FixedVec;

/// A fully qualified instantaneous speed quantity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SpeedQuantity {
    InstantaneousHorizontal,
    Spatial3d,
    BodyLongitudinalSigned,
    BodyLongitudinalMagnitude,
}

/// A fully qualified accumulated distance quantity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum DistanceQuantity {
    HorizontalPath,
    Spatial3d,
    BodyLongitudinalSigned,
    BodyLongitudinalAbsolute,
}

/// Required sign of a finite-plane crossing.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CrossingDirection {
    NegativeToPositive,
    PositiveToNegative,
    Either,
}

impl CrossingDirection {
    pub(super) fn accepts(self, normal_speed_mps: f64) -> bool {
        match self {
            Self::NegativeToPositive => normal_speed_mps > 0.0,
            Self::PositiveToNegative => normal_speed_mps < 0.0,
            Self::Either => normal_speed_mps != 0.0,
        }
    }
}

/// Correlation model for a gate's fixed survey displacement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GateSurveyUncertainty {
    /// Gate displacement is treated as exact.
    Exact,
    /// A scalar marginal alone does not specify its correlation with navigation.
    Unspecified,
    /// Legacy normal-displacement variance in m² with unspecified correlation.
    UnspecifiedVariance(f64),
    /// The scalar normal displacement is independent of navigation and of
    /// other gate IDs. Repeated crossings of this gate share the same error.
    Independent(f64),
    /// Three residual ECEF displacement coordinates in the engine's joint
    /// `SurveyMetres` catalog. Their supplied mean must be zero: the resolved
    /// gate center already includes the nominal survey correction.
    Shared(SharedParameterId),
}

impl GateSurveyUncertainty {
    pub(crate) fn validate(self) -> Result<(), ValidationError> {
        match self {
            Self::UnspecifiedVariance(variance) | Self::Independent(variance)
                if !variance.is_finite() || variance < 0.0 =>
            {
                Err(ValidationError::InvalidMetricDefinition)
            }
            Self::Shared(id) if id.get() == 0 => Err(ValidationError::InvalidMetricDefinition),
            _ => Ok(()),
        }
    }
}

/// A finite oriented gate in an already resolved ECEF frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteGate {
    pub id: GateId,
    pub frame: FrameId,
    pub center_ecef_m: [f64; 3],
    pub normal_ecef: [f64; 3],
    pub width_axis_ecef: [f64; 3],
    pub height_axis_ecef: [f64; 3],
    pub width_m: f64,
    pub height_m: f64,
    pub direction: CrossingDirection,
    pub minimum_normal_speed_mps: f64,
    pub rearm_distance_m: f64,
    pub minimum_crossing_interval: DurationNs,
    pub survey_uncertainty: GateSurveyUncertainty,
}

impl FiniteGate {
    /// Constructs an orthonormal finite gate from a normal and an in-plane
    /// width direction.  The height direction is derived deterministically.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: GateId,
        frame: FrameId,
        center_ecef_m: [f64; 3],
        normal_ecef: [f64; 3],
        width_axis_ecef: [f64; 3],
        width_m: f64,
        height_m: f64,
        direction: CrossingDirection,
        minimum_normal_speed_mps: f64,
        rearm_distance_m: f64,
        minimum_crossing_interval: DurationNs,
        survey_variance_normal_m2: Option<f64>,
    ) -> Result<Self, ValidationError> {
        if !all_finite(&center_ecef_m)
            || !all_finite(&normal_ecef)
            || !all_finite(&width_axis_ecef)
            || !width_m.is_finite()
            || !height_m.is_finite()
            || !minimum_normal_speed_mps.is_finite()
            || !rearm_distance_m.is_finite()
            || width_m <= 0.0
            || height_m <= 0.0
            || minimum_normal_speed_mps < 0.0
            || rearm_distance_m < 0.0
            || survey_variance_normal_m2.is_some_and(|value| !value.is_finite() || value < 0.0)
        {
            return Err(ValidationError::InvalidMetricDefinition);
        }

        let normal = normalize(normal_ecef).ok_or(ValidationError::InvalidMetricDefinition)?;
        let projected_width = sub(width_axis_ecef, scale(normal, dot(width_axis_ecef, normal)));
        let width_axis =
            normalize(projected_width).ok_or(ValidationError::InvalidMetricDefinition)?;
        let height_axis =
            normalize(cross(normal, width_axis)).ok_or(ValidationError::InvalidMetricDefinition)?;

        Ok(Self {
            id,
            frame,
            center_ecef_m,
            normal_ecef: normal,
            width_axis_ecef: width_axis,
            height_axis_ecef: height_axis,
            width_m,
            height_m,
            direction,
            minimum_normal_speed_mps,
            rearm_distance_m,
            minimum_crossing_interval,
            survey_uncertainty: match survey_variance_normal_m2 {
                None => GateSurveyUncertainty::Unspecified,
                Some(0.0) => GateSurveyUncertainty::Exact,
                Some(variance) => GateSurveyUncertainty::UnspecifiedVariance(variance),
            },
        })
    }

    /// Declares the supplied normal survey variance independent of navigation
    /// and other physical gates. Reusing this gate ID reuses that fixed error.
    pub fn with_independent_survey(mut self) -> Result<Self, ValidationError> {
        let variance = match self.survey_uncertainty {
            GateSurveyUncertainty::Exact => 0.0,
            GateSurveyUncertainty::UnspecifiedVariance(value)
            | GateSurveyUncertainty::Independent(value) => value,
            _ => return Err(ValidationError::InvalidMetricDefinition),
        };
        if !variance.is_finite() || variance < 0.0 {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.survey_uncertainty = GateSurveyUncertainty::Independent(variance);
        Ok(self)
    }

    /// Binds survey displacement to a stable three-coordinate shared parameter.
    /// Its joint covariance replaces the legacy scalar marginal; engine
    /// preflight checks the kind, mean, and coordinate-operation accuracy.
    pub fn with_shared_survey_parameter(
        mut self,
        parameter: SharedParameterId,
    ) -> Result<Self, ValidationError> {
        if parameter.get() == 0 {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.survey_uncertainty = GateSurveyUncertainty::Shared(parameter);
        Ok(self)
    }

    /// Canonical gate-definition identity, including survey correlation and
    /// shared parameter identity. A caller assembling a metric-plan digest
    /// can bind this complete gate definition without relying on Debug output.
    #[must_use]
    pub fn canonical_digest_v1(&self) -> crate::ids::ContentDigestV1 {
        super::identity::gate_definition_digest_v1(self)
    }

    pub(super) fn contains(&self, point_ecef_m: [f64; 3], tolerance_m: f64) -> bool {
        let offset = sub(point_ecef_m, self.center_ecef_m);
        dot(offset, self.width_axis_ecef).abs() <= self.width_m * 0.5 + tolerance_m
            && dot(offset, self.height_axis_ecef).abs() <= self.height_m * 0.5 + tolerance_m
    }
}

/// A path-distance request.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistancePlan {
    pub definition: MetricDefinitionId,
    pub quantity: DistanceQuantity,
    pub reference_point: ReferencePointId,
    /// Absolute quadrature-error allowance for the complete measurement,
    /// shared across all trajectory segments and speed-sign subdivisions.
    pub absolute_tolerance_m: f64,
    /// Relative allowance for the complete signed or unsigned distance.
    /// Reported numerical error must not exceed the larger of the absolute
    /// allowance and this fraction of the absolute reported distance.
    pub relative_tolerance: f64,
}

/// An ordered lap/sector definition.
#[derive(Clone, Debug, PartialEq)]
pub struct LapPlan {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub crossing_speed: Option<SpeedQuantity>,
    /// Maximum crossing occurrences retained for each gate during one run.
    /// This bounds live result IDs and tombstones before the session starts.
    pub(super) maximum_occurrences_per_gate: u16,
    pub(super) gates: FixedVec<FiniteGate, MAX_LAP_GATES>,
}

impl LapPlan {
    #[must_use]
    pub const fn new(
        definition: MetricDefinitionId,
        reference_point: ReferencePointId,
        crossing_speed: Option<SpeedQuantity>,
    ) -> Self {
        Self {
            definition,
            reference_point,
            crossing_speed,
            maximum_occurrences_per_gate: 16,
            gates: FixedVec::new(),
        }
    }

    /// Sets the semantic occurrence bound used by both live preflight and
    /// evaluation.
    pub fn set_maximum_occurrences_per_gate(
        &mut self,
        maximum: u16,
    ) -> Result<(), ValidationError> {
        if maximum == 0 {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.maximum_occurrences_per_gate = maximum;
        Ok(())
    }

    #[must_use]
    pub const fn maximum_occurrences_per_gate(&self) -> u16 {
        self.maximum_occurrences_per_gate
    }

    pub fn push_gate(&mut self, gate: FiniteGate) -> Result<(), ValidationError> {
        gate.survey_uncertainty.validate()?;
        if self.gates.iter().any(|present| present.id == gate.id) {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.gates
            .push(gate)
            .map_err(|_| ValidationError::CapacityExceeded)
    }

    #[must_use]
    pub fn gates(&self) -> &[FiniteGate] {
        self.gates.as_slice()
    }
}

/// Rule establishing the drag-event time origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LaunchRule {
    FirstSustainedMotion {
        threshold_mps: f64,
        dwell: DurationNs,
    },
    SpeedThreshold {
        quantity: SpeedQuantity,
        threshold_mps: f64,
        dwell: DurationNs,
    },
    AccelerationChangePoint {
        minimum_acceleration_mps2: f64,
        dwell: DurationNs,
    },
    ExternalTimestamp(SessionTime),
}

/// Explicit rollout applied to elapsed drag time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Rollout {
    None,
    Distance {
        quantity: DistanceQuantity,
        metres: f64,
    },
}

impl Rollout {
    /// Standard one-foot rollout measured as horizontal path length.
    pub const ONE_FOOT_HORIZONTAL_PATH: Self = Self::Distance {
        quantity: DistanceQuantity::HorizontalPath,
        metres: 0.3048,
    };
}

/// Required direction at a speed target.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TargetDirection {
    Ascending,
    /// A braking target. Its named speed quantity must cross at or below the
    /// enclosing plan's stop threshold and remain below that threshold for
    /// the configured stop dwell before the result can finalize.
    Descending,
    Either,
}

/// A speed or distance target within a drag plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DragTarget {
    Speed {
        id: TargetId,
        quantity: SpeedQuantity,
        metres_per_second: f64,
        direction: TargetDirection,
    },
    Distance {
        id: TargetId,
        quantity: DistanceQuantity,
        metres: f64,
    },
}

impl DragTarget {
    #[must_use]
    pub const fn id(self) -> TargetId {
        match self {
            Self::Speed { id, .. } | Self::Distance { id, .. } => id,
        }
    }

    /// Quarter-mile target using horizontal path length, exactly 402.336 m.
    #[must_use]
    pub const fn quarter_mile_horizontal_path(id: TargetId) -> Self {
        Self::Distance {
            id,
            quantity: DistanceQuantity::HorizontalPath,
            metres: 402.336,
        }
    }
}

/// Drag, acceleration, and braking definition.
#[derive(Clone, Debug, PartialEq)]
pub struct DragPlan {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub launch: LaunchRule,
    pub rollout: Rollout,
    /// Stop threshold applied in each explicitly descending target's own
    /// nonnegative [`SpeedQuantity`]. Descending targets above this threshold
    /// and signed-longitudinal braking targets are rejected because they do
    /// not provide bounded, unambiguous time-to-stop support.
    pub stop_threshold_mps: f64,
    pub stop_dwell: DurationNs,
    pub(super) targets: FixedVec<DragTarget, MAX_DRAG_TARGETS>,
}

impl DragPlan {
    #[must_use]
    pub const fn new(
        definition: MetricDefinitionId,
        reference_point: ReferencePointId,
        launch: LaunchRule,
    ) -> Self {
        Self {
            definition,
            reference_point,
            launch,
            rollout: Rollout::None,
            stop_threshold_mps: 0.5,
            stop_dwell: DurationNs::from_ns(500_000_000),
            targets: FixedVec::new(),
        }
    }

    pub fn push_target(&mut self, target: DragTarget) -> Result<(), ValidationError> {
        if self
            .targets
            .iter()
            .any(|present| present.id() == target.id())
        {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.targets
            .push(target)
            .map_err(|_| ValidationError::CapacityExceeded)
    }

    #[must_use]
    pub fn targets(&self) -> &[DragTarget] {
        self.targets.as_slice()
    }
}

/// Full-session activity totals and optional distance splits.
#[derive(Clone, Debug, PartialEq)]
pub struct ActivityPlan {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub include_horizontal_distance: bool,
    pub include_spatial_distance: bool,
    pub moving_speed: SpeedQuantity,
    pub moving_threshold_mps: f64,
    pub peak_speed: SpeedQuantity,
    /// `ZERO` selects the instantaneous peak. Nonzero window-average peaks
    /// are reserved but rejected until a bounded window integrator exists.
    pub peak_window: DurationNs,
    pub(super) splits_m: FixedVec<f64, MAX_ACTIVITY_SPLITS>,
}

impl ActivityPlan {
    #[must_use]
    pub const fn new(definition: MetricDefinitionId, reference_point: ReferencePointId) -> Self {
        Self {
            definition,
            reference_point,
            include_horizontal_distance: true,
            include_spatial_distance: true,
            moving_speed: SpeedQuantity::InstantaneousHorizontal,
            moving_threshold_mps: 0.5,
            peak_speed: SpeedQuantity::InstantaneousHorizontal,
            peak_window: DurationNs::ZERO,
            splits_m: FixedVec::new(),
        }
    }

    pub fn push_split(&mut self, metres: f64) -> Result<(), ValidationError> {
        if !metres.is_finite()
            || metres <= 0.0
            || self.splits_m.last().is_some_and(|last| metres <= *last)
        {
            return Err(ValidationError::InvalidMetricDefinition);
        }
        self.splits_m
            .push(metres)
            .map_err(|_| ValidationError::CapacityExceeded)
    }

    #[must_use]
    pub fn splits_m(&self) -> &[f64] {
        self.splits_m.as_slice()
    }
}

/// States produced by host ski segmentation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(u8)]
pub enum SkiState {
    Stationary = 0,
    Downhill = 1,
    Ascent = 2,
    Lift = 3,
    Other = 4,
}

impl SkiState {
    #[cfg(feature = "offline")]
    pub(super) const ALL: [Self; 5] = [
        Self::Stationary,
        Self::Downhill,
        Self::Ascent,
        Self::Lift,
        Self::Other,
    ];

    #[cfg(feature = "offline")]
    pub(super) const fn index(self) -> usize {
        self as usize
    }
}

/// Versioned HMM parameters. Scores are log-probability affine models over
/// `[horizontal speed, vertical speed, spatial acceleration]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkiHmmModel {
    pub initial_log_probability: [f64; 5],
    pub transition_log_probability: [[f64; 5]; 5],
    pub emission_bias: [f64; 5],
    pub emission_weight: [[f64; 3]; 5],
}

/// Host ski-segmentation definition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkiPlan {
    pub definition: MetricDefinitionId,
    pub reference_point: ReferencePointId,
    pub sample_period: DurationNs,
    pub minimum_segment_duration: DurationNs,
    pub model: SkiHmmModel,
}

/// One explicit metric definition.
#[derive(Clone, Debug, PartialEq)]
pub enum MetricDefinition {
    Distance(DistancePlan),
    Lap(LapPlan),
    Drag(DragPlan),
    Activity(ActivityPlan),
    Ski(SkiPlan),
}

impl MetricDefinition {
    #[must_use]
    pub const fn id(&self) -> MetricDefinitionId {
        match self {
            Self::Distance(plan) => plan.definition,
            Self::Lap(plan) => plan.definition,
            Self::Drag(plan) => plan.definition,
            Self::Activity(plan) => plan.definition,
            Self::Ski(plan) => plan.definition,
        }
    }

    /// Physical point selected by this complete definition.
    #[must_use]
    pub const fn reference_point(&self) -> ReferencePointId {
        match self {
            Self::Distance(plan) => plan.reference_point,
            Self::Lap(plan) => plan.reference_point,
            Self::Drag(plan) => plan.reference_point,
            Self::Activity(plan) => plan.reference_point,
            Self::Ski(plan) => plan.reference_point,
        }
    }

    /// Whether any requested value uses a claimed body-forward axis.
    #[must_use]
    pub fn requires_body_axis_quantities(&self) -> bool {
        match self {
            Self::Distance(plan) => distance_quantity_requires_body_axis(plan.quantity),
            Self::Lap(plan) => plan
                .crossing_speed
                .is_some_and(speed_quantity_requires_body_axis),
            Self::Drag(plan) => {
                matches!(
                    plan.launch,
                    LaunchRule::SpeedThreshold { quantity, .. }
                        if speed_quantity_requires_body_axis(quantity)
                ) || matches!(
                    plan.rollout,
                    Rollout::Distance { quantity, .. }
                        if distance_quantity_requires_body_axis(quantity)
                ) || plan.targets.iter().any(|target| match *target {
                    DragTarget::Speed { quantity, .. } => {
                        speed_quantity_requires_body_axis(quantity)
                    }
                    DragTarget::Distance { quantity, .. } => {
                        distance_quantity_requires_body_axis(quantity)
                    }
                })
            }
            Self::Activity(plan) => {
                speed_quantity_requires_body_axis(plan.moving_speed)
                    || speed_quantity_requires_body_axis(plan.peak_speed)
            }
            // The current ski model consumes frame-independent horizontal,
            // vertical, and acceleration magnitudes. Future configured body
            // outputs must opt in here explicitly.
            Self::Ski(_) => false,
        }
    }

    /// Checks the complete definition against one physical attachment and
    /// selected reference-point kind.
    #[must_use]
    pub fn is_permitted_by_attachment(
        &self,
        attachment: AttachmentModel,
        reference_point_kind: ReferencePointKind,
    ) -> bool {
        attachment.permits_reference_point(reference_point_kind)
            && (!self.requires_body_axis_quantities() || attachment.permits_body_axis_quantities())
    }
}

const fn speed_quantity_requires_body_axis(quantity: SpeedQuantity) -> bool {
    matches!(
        quantity,
        SpeedQuantity::BodyLongitudinalSigned | SpeedQuantity::BodyLongitudinalMagnitude
    )
}

const fn distance_quantity_requires_body_axis(quantity: DistanceQuantity) -> bool {
    matches!(
        quantity,
        DistanceQuantity::BodyLongitudinalSigned | DistanceQuantity::BodyLongitudinalAbsolute
    )
}
