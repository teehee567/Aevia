//! Shared test vocabulary and responsibility-specific live and host regression suites.

use super::*;
use crate::{
    config::{
        AttachmentModel, CalibrationBundle, CalibrationPolicy, ClockSharedCrossCovariance,
        CovarianceRepairPolicy, DynamicsProfileSpec, EmbeddedLiveTuning, FmaPolicy,
        GnssCorrelationPolicy, GnssFusionSpec, HeadingObservabilitySpec, InitialClockConsiderPrior,
        InitialHeading, InputProfileSpec, Installation, LeverArmParameter, LiveResourceLimits,
        NumericProfileSpec, ProcessNoiseSpec, ProcessingResultSpec, QualificationReportV1,
        QualificationSpecV1, QualificationStatus, RotationParameter, SharedParameterDefinition,
        SharedParameterMean, SharedParameterSet, SharedUncertaintyTreatment,
        StationaryClassifierSpec,
    },
    frame::{
        BodyLeverArm, CoordinateEpoch, CoordinateOperation, CoordinateOperationKind, EcefPosition,
        EcefVelocity, ReferenceEllipsoid, ReferencePoint, ReferencePointKind, SensorAngularRate,
        SensorSpecificForce, SensorToBodyRotation, TerrestrialFrame, TerrestrialRealization,
        Wgs84Realization,
    },
    ids::{
        CalibrationRevision, ClockModelId, ClockSegmentId, ContentDigestV1, CoordinateOperationId,
        DynamicsProfileId, FrameId, InputProfileId, MetricDefinitionId, NormalizationRevision,
        ObservationId, QualificationSpecId, ReferencePointId, ResultRevisionId, SessionId,
        SharedParameterId, SourceId, TargetId, TrajectoryRevision,
    },
    math::{FiniteF64, NonNegativeF64, Probability, UnitQuaternion, Vector3},
    metric::{
        DistancePlan, DistanceQuantity, DragPlan, DragTarget, LaunchRule, LiveMetricLimits,
        MetricDefinition, MetricPlan, SpeedQuantity, TargetDirection,
    },
    observation::{
        AxisStatus, ClockAffineBridge, ClockDiscontinuityReason, ClockTransitionObservation,
        ClockTransitionUncertainty, GnssDiagnostics, GnssPosition, GnssSolutionObservation,
        GnssVelocity, ImuObservation, ImuStatus, IndependentClockPrior, ReceiverHealth, RtkState,
        SolutionClass, TimedAngularRate, TimedDiagnostic, TimedSpecificForce, VelocityMethod,
    },
    offline::{
        CAPTURED_REINITIALIZATION_SCHEMA_V2, CapturedReinitializationInputV2,
        CapturedReplayContract, ClockModelEvidence, ControlChangeEvidence, EvidenceEnd,
        EvidenceGap, EvidenceGapReason, EvidenceManifest, EvidenceSource, ReinitializationEvidence,
        ReinitializationReason, ResultDescriptor, ResultSink, ResultSinkAttestation,
        ResultSinkPreflight, SliceEvidenceSource,
    },
    provenance::{EvidenceLineage, EvidenceSelection, ProcessingAttempt, SpanCapabilities},
    time::{DurationNs, ObservationTime, SampleSupport, SignedDurationNs, TimingBasis},
    uncertainty::{Covariance3, MeasurementUncertainty, SharedParameterCovariance, Variance},
};

mod profile_fixture;
use profile_fixture::*;

mod observation_fixture;
use observation_fixture::*;

mod replay_fixture;
use replay_fixture::*;

mod port_fixture;
use port_fixture::*;

mod session_fixture;
use session_fixture::*;

mod gnss_quality;

mod imu_quality;

mod live_preflight;

mod clock_transitions;

mod captured_replay;

mod offline_evidence;

mod fallback;

mod replay_limits;

use super::quality::corrected_observability;
use crate::config::EngineConfig;
use crate::config::LiveSpec;
use crate::config::OfflineResourceLimits;
use crate::config::ProcessingLevel;
use crate::config::ProcessingSpec;
use crate::config::RunControl;
use crate::config::ScalarPolicy;
use crate::config::SharedParameterKind;
use crate::engine::{
    FusionOutcome, TrajectoryEngine, captured_summary_digest_v1, captured_update_digest_v1,
};
use crate::error::PrepareError;
use crate::error::ProcessError;
use crate::error::StepError;
use crate::error::ValidationError;
use crate::live::ConsiderCovariance;
use crate::live::DenseCovariance;
use crate::live::DenseEndpoint;
use crate::live::EcefAnchor;
use crate::live::LiveCore;
use crate::live::LiveCoreError;
use crate::live::LiveCoreInput;
use crate::live::MAX_CONSIDER;
use crate::live::independent_clock_consider_covariance_into;
use crate::live::transition_consider_covariance_into;
use crate::observation::InputDisposition;
use crate::observation::LiveObservation;
use crate::observation::LiveStep;
use crate::observation::WorkQuota;
use crate::offline::CapturedLiveFinishCall;
use crate::offline::CapturedLiveStepCall;
use crate::offline::CapturedTranscriptDigestV1;
use crate::offline::EvidenceEvent;
use crate::offline::ResultEnd;
use crate::offline::ResultRecord;
use crate::provenance::Capabilities;
use crate::provenance::Capability;
use crate::provenance::EvidenceClass;
use crate::provenance::EvidenceLineageKind;
use crate::provenance::EvidenceUse;
use crate::provenance::ProcessingAttemptOutcome;
use crate::quality::GnssState;
use crate::quality::HeadingSource;
use crate::quality::Integrity;
use crate::quality::Validity;
use crate::time::SessionTime;
use crate::time::TimeSpan;
use crate::workspace::LiveInternalWorkspace;
use crate::workspace::LivePsramWorkspace;
use crate::workspace::LiveWorkspace;
use crate::workspace::MemoryRegion;
use nalgebra::Vector3 as NaVector3;
