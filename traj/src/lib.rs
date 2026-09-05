#![cfg_attr(not(any(feature = "offline", test)), no_std)]
#![doc = include_str!("../README.md")]

#[cfg(all(feature = "gtsam-system", feature = "gtsam-vendored"))]
compile_error!("gtsam-system and gtsam-vendored are mutually exclusive");

#[cfg(all(
    any(
        feature = "gtsam-system",
        feature = "gtsam-vendored",
        feature = "raw-tight"
    ),
    any(target_os = "none", target_os = "espidf")
))]
compile_error!("GTSAM and raw-tight processing are workstation-only");

#[cfg(any(feature = "offline", test))]
extern crate std;

pub mod config;
#[cfg(any(feature = "offline", test))]
mod enclosure;
pub mod engine;
pub mod error;
pub mod frame;
pub mod ids;
mod live;
pub mod math;
pub mod metric;
pub mod observation;
#[cfg(feature = "offline")]
pub mod offline;
pub mod provenance;
pub mod quality;
mod scalar_math;
pub mod time;
pub mod trajectory;
pub mod uncertainty;
pub mod workspace;

#[cfg(any(feature = "gtsam-system", feature = "gtsam-vendored"))]
mod advanced;
#[cfg(feature = "raw-tight")]
mod raw_tight;

pub use config::{
    CapturedReplayComparison, EngineConfig, LiveSpec, ProcessingLevel, ProcessingPolicy,
    ProcessingResultSpec, ProcessingSpec,
};
pub use engine::{LivePlan, LiveSession, PreparedProcess, TrajectoryEngine};
pub use error::{PrepareError, ProcessError, QueryError, StepError, ValidationError};
pub use metric::{
    LiveMetricPlan, MetricDefinitionDiagnostic, MetricDefinitionDiagnosticReason, MetricPlan,
    MetricResult, MetricResults,
};
pub use observation::{LiveObservation, LiveStep};
pub use trajectory::{KinematicEstimate, Trajectory};
pub use workspace::{LiveWorkspace, WorkspaceRequirements};
