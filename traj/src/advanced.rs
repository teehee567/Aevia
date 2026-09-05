//! Private, feature-gated boundary for the optional GTSAM graph smoother.
//!
//! This module deliberately contains no FFI and no substitute graph solver.
//! Enabling a Cargo feature proves only that the Rust-side adapter contract was
//! compiled.  A result may be produced only after a reviewed native adapter is
//! linked and its immutable registration passes the checks below.  The current
//! crate has no such native linkage, so the production hook remains empty and
//! preflight fails closed with [`AdvancedPreflightError::BackendNotRegistered`].

#![allow(
    dead_code,
    reason = "dormant private boundary until native adapter linkage exists"
)]

use crate::{
    config::{CalibrationPolicy, OfflineResourceLimits, ProcessingLevel},
    error::{PrepareError, ProcessError},
    ids::{BackendVersionId, ContentDigestV1, QualificationSpecId},
    provenance::{Capabilities, Capability, ProcessingAttemptOutcome},
    time::TimeSpan,
};

/// Rust/native contract revision.  A native wrapper must match this exactly;
/// the GTSAM version alone is not an ABI contract.
pub(crate) const ADVANCED_ADAPTER_ABI_REVISION: u32 = 1;

/// Native distribution selected by the mutually exclusive Cargo features.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum GtsamLinkMode {
    System,
    Vendored,
}

#[cfg(feature = "gtsam-system")]
pub(crate) const COMPILED_GTSAM_LINK_MODE: GtsamLinkMode = GtsamLinkMode::System;
#[cfg(all(not(feature = "gtsam-system"), feature = "gtsam-vendored"))]
pub(crate) const COMPILED_GTSAM_LINK_MODE: GtsamLinkMode = GtsamLinkMode::Vendored;

/// Identity attested by a reviewed native bridge at process start.
///
/// `compiled_source_digest` is computed from the source actually used to build
/// the native library. `reviewed_source_digest` is the release allow-list
/// value. They must match byte-for-byte. A system build additionally needs an
/// ABI fingerprint of the discovered binary; a vendored build instead pins the
/// reviewed patch set applied to the source tree.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GtsamNativeIdentity {
    gtsam_version: [u16; 3],
    adapter_abi_revision: u32,
    compiled_source_digest: ContentDigestV1,
    reviewed_source_digest: ContentDigestV1,
    native_build_digest: ContentDigestV1,
    system_binary_abi_digest: Option<ContentDigestV1>,
    vendored_patch_digest: Option<ContentDigestV1>,
}

/// Immutable evidence that the backend/profile pair passed its frozen gates.
///
/// This is backend-specific evidence in addition to the selected engine
/// profile's qualification.  It covers the equation/Jacobian equivalence and
/// empirical uncertainty gates required before GTSAM may be selected.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedQualificationReceipt {
    specification: QualificationSpecId,
    specification_digest: ContentDigestV1,
    report_digest: ContentDigestV1,
    navigation_equation_fixture_count: u32,
    numerical_jacobian_fixture_count: u32,
    empirical_coverage_session_count: u16,
    passed: bool,
}

/// One equation/integrity property attested by the reviewed adapter. Merely
/// locating the named GTSAM classes does not prove these properties.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
enum AdvancedSafetyProperty {
    EarthFixedTermOwnership = 0,
    SeparateBiasMarkovChain = 1,
    CombinedImuFactorDisabled = 2,
    GnssFactorsUseActualEpochs = 3,
    LeverArmAndVectorVelocityJacobians = 4,
    PrivateFactorJacobians = 5,
    RobustMarginalsLabeledConditional = 6,
}

/// Compact set of reviewed safety properties.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub(crate) struct AdvancedSafetyClaims(u16);

impl AdvancedSafetyClaims {
    const ALL: Self = Self((1_u16 << 7) - 1);

    const fn all_required(self) -> bool {
        self.0 == Self::ALL.0
    }

    #[cfg(test)]
    const fn without(self, property: AdvancedSafetyProperty) -> Self {
        Self(self.0 & !(1_u16 << property as u8))
    }
}

/// Metadata owned by the native adapter; no GTSAM or C++ type crosses this
/// boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedBackendRegistration {
    link_mode: GtsamLinkMode,
    backend_version: BackendVersionId,
    native: GtsamNativeIdentity,
    qualification: AdvancedQualificationReceipt,
    safety: AdvancedSafetyClaims,
}

/// Why native registration is not acceptable for this build.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AdvancedRegistrationError {
    LinkModeMismatch {
        compiled: GtsamLinkMode,
        registered: GtsamLinkMode,
    },
    AdapterAbiMismatch {
        expected: u32,
        registered: u32,
    },
    InvalidGtsamVersion,
    MissingNativeBuildIdentity,
    SourceDigestMismatch,
    MissingSystemAbiFingerprint,
    UnexpectedSystemAbiFingerprint,
    MissingVendoredPatchDigest,
    UnexpectedVendoredPatchDigest,
    InvalidBackendVersion,
    QualificationFailed,
    IncompleteQualificationEvidence,
    MissingSafetyInvariant,
}

impl AdvancedBackendRegistration {
    fn validate(self) -> Result<Self, AdvancedRegistrationError> {
        if self.link_mode != COMPILED_GTSAM_LINK_MODE {
            return Err(AdvancedRegistrationError::LinkModeMismatch {
                compiled: COMPILED_GTSAM_LINK_MODE,
                registered: self.link_mode,
            });
        }
        if self.native.adapter_abi_revision != ADVANCED_ADAPTER_ABI_REVISION {
            return Err(AdvancedRegistrationError::AdapterAbiMismatch {
                expected: ADVANCED_ADAPTER_ABI_REVISION,
                registered: self.native.adapter_abi_revision,
            });
        }
        if self.native.gtsam_version == [0; 3] {
            return Err(AdvancedRegistrationError::InvalidGtsamVersion);
        }
        if digest_is_zero(self.native.native_build_digest) {
            return Err(AdvancedRegistrationError::MissingNativeBuildIdentity);
        }
        if digest_is_zero(self.native.compiled_source_digest)
            || digest_is_zero(self.native.reviewed_source_digest)
            || self.native.compiled_source_digest != self.native.reviewed_source_digest
        {
            return Err(AdvancedRegistrationError::SourceDigestMismatch);
        }
        match self.link_mode {
            GtsamLinkMode::System => {
                if self
                    .native
                    .system_binary_abi_digest
                    .is_none_or(digest_is_zero)
                {
                    return Err(AdvancedRegistrationError::MissingSystemAbiFingerprint);
                }
                if self.native.vendored_patch_digest.is_some() {
                    return Err(AdvancedRegistrationError::UnexpectedVendoredPatchDigest);
                }
            }
            GtsamLinkMode::Vendored => {
                if self.native.system_binary_abi_digest.is_some() {
                    return Err(AdvancedRegistrationError::UnexpectedSystemAbiFingerprint);
                }
                if self.native.vendored_patch_digest.is_none_or(digest_is_zero) {
                    return Err(AdvancedRegistrationError::MissingVendoredPatchDigest);
                }
            }
        }
        if self.backend_version.get() == 0 {
            return Err(AdvancedRegistrationError::InvalidBackendVersion);
        }
        if !self.qualification.passed {
            return Err(AdvancedRegistrationError::QualificationFailed);
        }
        if self.qualification.specification.get() == 0
            || digest_is_zero(self.qualification.specification_digest)
            || digest_is_zero(self.qualification.report_digest)
            || self.qualification.navigation_equation_fixture_count == 0
            || self.qualification.numerical_jacobian_fixture_count == 0
            || self.qualification.empirical_coverage_session_count == 0
        {
            return Err(AdvancedRegistrationError::IncompleteQualificationEvidence);
        }
        if !self.safety.all_required() {
            return Err(AdvancedRegistrationError::MissingSafetyInvariant);
        }
        Ok(self)
    }
}

/// Counts derived by a restartable semantic-evidence scan before graph
/// construction.  Keyframe choice is already made by the adaptive policy, so
/// the resource report describes the proposed graph rather than a hidden
/// worst-case cadence guess.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedProblemShape {
    pub imu_samples: u64,
    pub gnss_position_factors: u64,
    pub gnss_velocity_factors: u64,
    pub stationary_factors: u64,
    pub vehicle_constraint_factors: u64,
    pub proposed_keyframes: u64,
    pub proposed_bias_nodes: u64,
    pub clock_segments: u32,
    pub refined_shared_parameter_coordinates: u16,
    pub requested_output_epochs: u64,
}

/// Inputs to the private advanced candidate preflight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct AdvancedPreflightRequest {
    pub span: TimeSpan,
    pub capabilities: Capabilities,
    pub evidence_restartable: bool,
    pub offline_initializer_available: bool,
    pub profile_qualified: bool,
    pub calibration_policy: CalibrationPolicy,
    pub shape: AdvancedProblemShape,
}

/// Fixed algorithmic pass bounds exposed as deterministic progress units.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedPassLimits {
    pub reverse_initialization_passes: u8,
    pub robust_reclassification_passes: u8,
    pub levenberg_marquardt_iterations: u16,
    pub marginal_batches: u32,
}

/// Conservative resource estimate produced before a native graph exists.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedResourceEstimate {
    pub estimation_model_revision: u32,
    pub proposed_keyframes: u64,
    pub proposed_bias_nodes: u64,
    pub state_coordinate_count: u64,
    pub factor_count: u64,
    pub estimated_sparse_nonzeros: u64,
    pub estimated_peak_memory_bytes: u64,
    pub estimated_temporary_storage_bytes: u64,
    pub estimated_output_bytes: u64,
    pub progress_work_units: u64,
    pub minimum_worker_count: u16,
    pub pass_limits: AdvancedPassLimits,
}

/// Resource ceiling that rejected a proposed graph.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AdvancedResourceKind {
    PeakMemory,
    TemporaryStorage,
    OutputStorage,
    WorkerCount,
    ElapsedWork,
}

/// Exact reason an advanced candidate cannot start.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AdvancedPreflightError {
    InvalidProblemShape,
    EstimateOverflow,
    MissingEvidence(Capability),
    EvidenceNotRestartable,
    OfflineInitializerUnavailable,
    ProfileUnqualified,
    BackendNotRegistered,
    InvalidRegistration(AdvancedRegistrationError),
    InvalidResourceLimits,
    InsufficientResource {
        resource: AdvancedResourceKind,
        required: u64,
        available: u64,
    },
}

impl AdvancedPreflightError {
    /// Stable fallback/provenance category recorded by the host engine.
    pub(crate) const fn attempt_outcome(self) -> ProcessingAttemptOutcome {
        match self {
            Self::MissingEvidence(_)
            | Self::EvidenceNotRestartable
            | Self::OfflineInitializerUnavailable => ProcessingAttemptOutcome::EvidenceUnavailable,
            Self::ProfileUnqualified | Self::InvalidRegistration(_) => {
                ProcessingAttemptOutcome::NotQualified
            }
            Self::BackendNotRegistered => ProcessingAttemptOutcome::NotCompiled,
            Self::InvalidProblemShape | Self::EstimateOverflow | Self::InvalidResourceLimits => {
                ProcessingAttemptOutcome::Failed
            }
            Self::InsufficientResource { .. } => ProcessingAttemptOutcome::InsufficientResources,
        }
    }

    pub(crate) const fn prepare_error(self) -> PrepareError {
        match self {
            Self::MissingEvidence(_)
            | Self::EvidenceNotRestartable
            | Self::OfflineInitializerUnavailable => PrepareError::EvidenceUnavailable,
            Self::ProfileUnqualified | Self::InvalidRegistration(_) => {
                PrepareError::UnqualifiedProfile
            }
            Self::BackendNotRegistered => PrepareError::CapabilityUnavailable,
            Self::InvalidProblemShape | Self::EstimateOverflow | Self::InvalidResourceLimits => {
                PrepareError::InsufficientResources
            }
            Self::InsufficientResource { .. } => PrepareError::InsufficientResources,
        }
    }

    pub(crate) const fn process_error(self) -> ProcessError {
        match self {
            Self::MissingEvidence(_)
            | Self::EvidenceNotRestartable
            | Self::OfflineInitializerUnavailable => ProcessError::IncompleteEvidence,
            Self::BackendNotRegistered => ProcessError::CapabilityUnavailable,
            Self::InsufficientResource { .. }
            | Self::EstimateOverflow
            | Self::InvalidResourceLimits => ProcessError::ResourceLimit,
            Self::InvalidProblemShape | Self::ProfileUnqualified | Self::InvalidRegistration(_) => {
                ProcessError::AdvancedCapabilityFailure
            }
        }
    }
}

/// Failure plus any resource estimate that was safely available.  This lets a
/// rejected candidate retain useful diagnostics without publishing solver
/// state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedPreflightFailure {
    pub error: AdvancedPreflightError,
    pub estimate: Option<AdvancedResourceEstimate>,
}

/// Candidate-stage diagnostic suitable for immutable attempt provenance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AdvancedCandidateDiagnostic {
    pub level: ProcessingLevel,
    /// `None` means preflight succeeded but no transactional result has yet
    /// committed; it must never be recorded as a successful attempt early.
    pub final_outcome: Option<ProcessingAttemptOutcome>,
    pub error: Option<AdvancedPreflightError>,
    pub estimate: Option<AdvancedResourceEstimate>,
    pub link_mode: GtsamLinkMode,
    pub backend_version: Option<BackendVersionId>,
}

impl AdvancedPreflightFailure {
    pub(crate) const fn diagnostic(self) -> AdvancedCandidateDiagnostic {
        AdvancedCandidateDiagnostic {
            level: ProcessingLevel::AdvancedGraph,
            final_outcome: Some(self.error.attempt_outcome()),
            error: Some(self.error),
            estimate: self.estimate,
            link_mode: COMPILED_GTSAM_LINK_MODE,
            backend_version: None,
        }
    }
}

/// Validated native identity and graph bounds.  It contains no graph, factor,
/// sparse matrix, or native-library handle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PreparedAdvancedBackend {
    registration: AdvancedBackendRegistration,
    estimate: AdvancedResourceEstimate,
}

impl PreparedAdvancedBackend {
    pub(crate) const fn estimate(self) -> AdvancedResourceEstimate {
        self.estimate
    }

    pub(crate) const fn backend_version(self) -> BackendVersionId {
        self.registration.backend_version
    }

    pub(crate) const fn native_source_digest(self) -> ContentDigestV1 {
        self.registration.native.compiled_source_digest
    }

    pub(crate) const fn diagnostic(self) -> AdvancedCandidateDiagnostic {
        AdvancedCandidateDiagnostic {
            level: ProcessingLevel::AdvancedGraph,
            // The host runner records Succeeded only after transactional
            // result commit.
            final_outcome: None,
            error: None,
            estimate: Some(self.estimate),
            link_mode: self.registration.link_mode,
            backend_version: Some(self.registration.backend_version),
        }
    }
}

/// Preflights the compiled adapter.  This release intentionally has no native
/// registration, so a feature-enabled build remains truthful and unavailable.
pub(crate) fn preflight(
    request: AdvancedPreflightRequest,
    limits: OfflineResourceLimits,
) -> Result<PreparedAdvancedBackend, AdvancedPreflightFailure> {
    preflight_with_registration(request, limits, compiled_backend_registration())
}

fn preflight_with_registration(
    request: AdvancedPreflightRequest,
    limits: OfflineResourceLimits,
    registration: Option<&AdvancedBackendRegistration>,
) -> Result<PreparedAdvancedBackend, AdvancedPreflightFailure> {
    let estimate =
        estimate_resources(request.shape, request.calibration_policy).map_err(|error| {
            AdvancedPreflightFailure {
                error,
                estimate: None,
            }
        })?;
    let failure = |error| AdvancedPreflightFailure {
        error,
        estimate: Some(estimate),
    };

    for capability in [
        Capability::NormalizedImu,
        Capability::GnssSolution,
        Capability::Timing,
        Capability::Configuration,
        Capability::CompleteEnd,
    ] {
        if !request.capabilities.contains(capability) {
            return Err(failure(AdvancedPreflightError::MissingEvidence(capability)));
        }
    }
    if !request.evidence_restartable {
        return Err(failure(AdvancedPreflightError::EvidenceNotRestartable));
    }
    if !request.offline_initializer_available {
        return Err(failure(
            AdvancedPreflightError::OfflineInitializerUnavailable,
        ));
    }
    if !request.profile_qualified {
        return Err(failure(AdvancedPreflightError::ProfileUnqualified));
    }
    let Some(registration) = registration.copied() else {
        return Err(failure(AdvancedPreflightError::BackendNotRegistered));
    };
    let registration = registration
        .validate()
        .map_err(|error| failure(AdvancedPreflightError::InvalidRegistration(error)))?;
    limits
        .validate()
        .map_err(|_| failure(AdvancedPreflightError::InvalidResourceLimits))?;
    enforce_limits(estimate, limits).map_err(failure)?;

    Ok(PreparedAdvancedBackend {
        registration,
        estimate,
    })
}

#[allow(
    clippy::too_many_lines,
    reason = "keep the checked revisioned estimate formula auditable in one place"
)]
fn estimate_resources(
    shape: AdvancedProblemShape,
    calibration_policy: CalibrationPolicy,
) -> Result<AdvancedResourceEstimate, AdvancedPreflightError> {
    if shape.imu_samples == 0
        || shape.gnss_position_factors == 0
        || shape.gnss_velocity_factors == 0
        || shape.proposed_keyframes < 2
        || shape.proposed_keyframes > shape.imu_samples
        || shape.proposed_bias_nodes == 0
        || shape.proposed_bias_nodes > shape.proposed_keyframes
        || shape.clock_segments == 0
        || shape.requested_output_epochs == 0
        || (calibration_policy == CalibrationPolicy::Fixed
            && shape.refined_shared_parameter_coordinates != 0)
        || (calibration_policy == CalibrationPolicy::RefineWithPriors
            && shape.refined_shared_parameter_coordinates == 0)
    {
        return Err(AdvancedPreflightError::InvalidProblemShape);
    }

    let checked_add = |left: u64, right: u64| {
        left.checked_add(right)
            .ok_or(AdvancedPreflightError::EstimateOverflow)
    };
    let checked_mul = |left: u64, right: u64| {
        left.checked_mul(right)
            .ok_or(AdvancedPreflightError::EstimateOverflow)
    };

    let keyframe_coordinates = checked_mul(shape.proposed_keyframes, 15)?;
    let bias_coordinates = checked_mul(shape.proposed_bias_nodes, 6)?;
    let clock_coordinates = checked_mul(u64::from(shape.clock_segments), 2)?;
    let state_coordinate_count = checked_add(
        checked_add(keyframe_coordinates, bias_coordinates)?,
        checked_add(
            clock_coordinates,
            u64::from(shape.refined_shared_parameter_coordinates),
        )?,
    )?;

    let imu_factors = shape.proposed_keyframes - 1;
    let bias_factors = shape.proposed_bias_nodes.saturating_sub(1);
    let mut factor_count = checked_add(imu_factors, bias_factors)?;
    for count in [
        shape.gnss_position_factors,
        shape.gnss_velocity_factors,
        shape.stationary_factors,
        shape.vehicle_constraint_factors,
        4, // gauge, initial state, initial bias, and clock priors
    ] {
        factor_count = checked_add(factor_count, count)?;
    }

    // Revision 1 is intentionally conservative: it stores block-graph
    // adjacency plus three numerical workspaces and does not assume a
    // particular sparse ordering advantage.
    let graph_block_nonzeros = checked_mul(factor_count, 96)?;
    let state_diagonal_nonzeros = checked_mul(state_coordinate_count, 2)?;
    let shared_parameter_cross = checked_mul(
        u64::from(shape.refined_shared_parameter_coordinates),
        checked_mul(shape.proposed_keyframes, 15)?,
    )?;
    let estimated_sparse_nonzeros = checked_add(
        checked_add(graph_block_nonzeros, state_diagonal_nonzeros)?,
        shared_parameter_cross,
    )?;

    let graph_bytes = checked_mul(factor_count, 448)?;
    let state_bytes = checked_mul(state_coordinate_count, 64)?;
    let sparse_bytes = checked_mul(estimated_sparse_nonzeros, 32)?;
    let relinearization_bytes = checked_mul(shape.proposed_keyframes, 2_048)?;
    let estimated_peak_memory_bytes = checked_add(
        checked_add(graph_bytes, state_bytes)?,
        checked_add(sparse_bytes, relinearization_bytes)?,
    )?;

    let estimated_temporary_storage_bytes = checked_add(
        checked_mul(shape.imu_samples, 112)?,
        checked_add(
            checked_mul(factor_count, 192)?,
            checked_mul(estimated_sparse_nonzeros, 8)?,
        )?,
    )?;
    let estimated_output_bytes = checked_add(
        checked_mul(shape.requested_output_epochs, 512)?,
        checked_mul(shape.proposed_keyframes, 1_024)?,
    )?;

    let marginal_batches_u64 = shape.proposed_keyframes.div_ceil(256).max(1);
    let marginal_batches = u32::try_from(marginal_batches_u64)
        .map_err(|_| AdvancedPreflightError::EstimateOverflow)?;
    let pass_limits = AdvancedPassLimits {
        reverse_initialization_passes: 2,
        robust_reclassification_passes: 4,
        levenberg_marquardt_iterations: 50,
        marginal_batches,
    };
    let solver_sweeps = u64::from(pass_limits.reverse_initialization_passes)
        + u64::from(pass_limits.robust_reclassification_passes)
        + u64::from(pass_limits.levenberg_marquardt_iterations);
    let progress_work_units = checked_add(
        checked_mul(shape.imu_samples, 3)?,
        checked_add(
            checked_mul(factor_count, solver_sweeps)?,
            checked_mul(u64::from(marginal_batches), 256)?,
        )?,
    )?;

    Ok(AdvancedResourceEstimate {
        estimation_model_revision: 1,
        proposed_keyframes: shape.proposed_keyframes,
        proposed_bias_nodes: shape.proposed_bias_nodes,
        state_coordinate_count,
        factor_count,
        estimated_sparse_nonzeros,
        estimated_peak_memory_bytes,
        estimated_temporary_storage_bytes,
        estimated_output_bytes,
        progress_work_units,
        minimum_worker_count: 1,
        pass_limits,
    })
}

fn enforce_limits(
    estimate: AdvancedResourceEstimate,
    limits: OfflineResourceLimits,
) -> Result<(), AdvancedPreflightError> {
    for (resource, required, available) in [
        (
            AdvancedResourceKind::PeakMemory,
            estimate.estimated_peak_memory_bytes,
            limits.peak_memory_bytes,
        ),
        (
            AdvancedResourceKind::TemporaryStorage,
            estimate.estimated_temporary_storage_bytes,
            limits.temporary_storage_bytes,
        ),
        (
            AdvancedResourceKind::OutputStorage,
            estimate.estimated_output_bytes,
            limits.output_bytes,
        ),
        (
            AdvancedResourceKind::WorkerCount,
            u64::from(estimate.minimum_worker_count),
            u64::from(limits.worker_count),
        ),
    ] {
        if required > available {
            return Err(AdvancedPreflightError::InsufficientResource {
                resource,
                required,
                available,
            });
        }
    }
    if let Some(available) = limits.elapsed_work_limit {
        if estimate.progress_work_units > available {
            return Err(AdvancedPreflightError::InsufficientResource {
                resource: AdvancedResourceKind::ElapsedWork,
                required: estimate.progress_work_units,
                available,
            });
        }
    }
    Ok(())
}

/// There is intentionally no weak symbol, dynamic loader, or ambient system
/// discovery.  Native integration must replace this value with a reviewed,
/// statically linked registration in the same private module.
const COMPILED_BACKEND_REGISTRATION: Option<AdvancedBackendRegistration> = None;

fn compiled_backend_registration() -> Option<&'static AdvancedBackendRegistration> {
    COMPILED_BACKEND_REGISTRATION.as_ref()
}

fn digest_is_zero(digest: ContentDigestV1) -> bool {
    digest.as_bytes().iter().all(|byte| *byte == 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::SessionTime;

    fn digest(value: u8) -> ContentDigestV1 {
        ContentDigestV1::from_bytes([value; 32])
    }

    fn capabilities() -> Capabilities {
        let mut result = Capabilities::NONE;
        for capability in [
            Capability::NormalizedImu,
            Capability::GnssSolution,
            Capability::Timing,
            Capability::Configuration,
            Capability::CompleteEnd,
        ] {
            result = result.with(capability);
        }
        result
    }

    fn shape() -> AdvancedProblemShape {
        AdvancedProblemShape {
            imu_samples: 10_000,
            gnss_position_factors: 250,
            gnss_velocity_factors: 250,
            stationary_factors: 20,
            vehicle_constraint_factors: 0,
            proposed_keyframes: 200,
            proposed_bias_nodes: 21,
            clock_segments: 2,
            refined_shared_parameter_coordinates: 0,
            requested_output_epochs: 2_000,
        }
    }

    fn request() -> AdvancedPreflightRequest {
        AdvancedPreflightRequest {
            span: TimeSpan::new(SessionTime::from_ns(0), SessionTime::from_ns(5_000_000_000))
                .unwrap(),
            capabilities: capabilities(),
            evidence_restartable: true,
            offline_initializer_available: true,
            profile_qualified: true,
            calibration_policy: CalibrationPolicy::Fixed,
            shape: shape(),
        }
    }

    fn limits() -> OfflineResourceLimits {
        OfflineResourceLimits {
            peak_memory_bytes: u64::MAX,
            temporary_storage_bytes: u64::MAX,
            output_bytes: u64::MAX,
            worker_count: 8,
            elapsed_work_limit: None,
        }
    }

    fn registration() -> AdvancedBackendRegistration {
        let (system_binary_abi_digest, vendored_patch_digest) = match COMPILED_GTSAM_LINK_MODE {
            GtsamLinkMode::System => (Some(digest(3)), None),
            GtsamLinkMode::Vendored => (None, Some(digest(4))),
        };
        AdvancedBackendRegistration {
            link_mode: COMPILED_GTSAM_LINK_MODE,
            backend_version: BackendVersionId::new(7),
            native: GtsamNativeIdentity {
                gtsam_version: [4, 3, 0],
                adapter_abi_revision: ADVANCED_ADAPTER_ABI_REVISION,
                compiled_source_digest: digest(1),
                reviewed_source_digest: digest(1),
                native_build_digest: digest(2),
                system_binary_abi_digest,
                vendored_patch_digest,
            },
            qualification: AdvancedQualificationReceipt {
                specification: QualificationSpecId::new(9),
                specification_digest: digest(5),
                report_digest: digest(6),
                navigation_equation_fixture_count: 1_000,
                numerical_jacobian_fixture_count: 10_000,
                empirical_coverage_session_count: 100,
                passed: true,
            },
            safety: AdvancedSafetyClaims::ALL,
        }
    }

    #[test]
    fn feature_without_native_registration_fails_closed_but_reports_estimate() {
        let failure = preflight(request(), limits()).unwrap_err();
        assert_eq!(failure.error, AdvancedPreflightError::BackendNotRegistered);
        assert!(failure.estimate.is_some());
        assert_eq!(
            failure.diagnostic().final_outcome,
            Some(ProcessingAttemptOutcome::NotCompiled)
        );
        assert_eq!(
            failure.error.prepare_error(),
            PrepareError::CapabilityUnavailable
        );
    }

    #[test]
    fn every_required_capability_is_checked_precisely() {
        for missing in [
            Capability::NormalizedImu,
            Capability::GnssSolution,
            Capability::Timing,
            Capability::Configuration,
            Capability::CompleteEnd,
        ] {
            let mut candidate = request();
            candidate.capabilities = Capabilities::NONE;
            for capability in [
                Capability::NormalizedImu,
                Capability::GnssSolution,
                Capability::Timing,
                Capability::Configuration,
                Capability::CompleteEnd,
            ] {
                if capability != missing {
                    candidate.capabilities = candidate.capabilities.with(capability);
                }
            }
            assert_eq!(
                preflight_with_registration(candidate, limits(), Some(&registration()))
                    .unwrap_err()
                    .error,
                AdvancedPreflightError::MissingEvidence(missing)
            );
        }
    }

    #[test]
    fn native_source_must_equal_the_reviewed_digest() {
        let mut backend = registration();
        backend.native.compiled_source_digest = digest(99);
        let failure = preflight_with_registration(request(), limits(), Some(&backend)).unwrap_err();
        assert_eq!(
            failure.error,
            AdvancedPreflightError::InvalidRegistration(
                AdvancedRegistrationError::SourceDigestMismatch
            )
        );
    }

    #[test]
    fn registration_mode_must_match_the_compiled_feature() {
        let mut backend = registration();
        backend.link_mode = match COMPILED_GTSAM_LINK_MODE {
            GtsamLinkMode::System => GtsamLinkMode::Vendored,
            GtsamLinkMode::Vendored => GtsamLinkMode::System,
        };
        assert_eq!(
            backend.validate(),
            Err(AdvancedRegistrationError::LinkModeMismatch {
                compiled: COMPILED_GTSAM_LINK_MODE,
                registered: backend.link_mode,
            })
        );
    }

    #[test]
    fn mode_specific_native_identity_is_mandatory() {
        let mut backend = registration();
        match COMPILED_GTSAM_LINK_MODE {
            GtsamLinkMode::System => backend.native.system_binary_abi_digest = None,
            GtsamLinkMode::Vendored => backend.native.vendored_patch_digest = None,
        }
        let expected = match COMPILED_GTSAM_LINK_MODE {
            GtsamLinkMode::System => AdvancedRegistrationError::MissingSystemAbiFingerprint,
            GtsamLinkMode::Vendored => AdvancedRegistrationError::MissingVendoredPatchDigest,
        };
        assert_eq!(backend.validate(), Err(expected));
    }

    #[test]
    fn unqualified_backend_and_profile_are_distinct() {
        let mut unqualified_profile = request();
        unqualified_profile.profile_qualified = false;
        assert_eq!(
            preflight_with_registration(unqualified_profile, limits(), Some(&registration()))
                .unwrap_err()
                .error,
            AdvancedPreflightError::ProfileUnqualified
        );

        let mut backend = registration();
        backend.qualification.passed = false;
        assert_eq!(
            preflight_with_registration(request(), limits(), Some(&backend))
                .unwrap_err()
                .error,
            AdvancedPreflightError::InvalidRegistration(
                AdvancedRegistrationError::QualificationFailed
            )
        );
    }

    #[test]
    fn equation_safety_claims_are_mandatory() {
        let mut backend = registration();
        backend.safety = backend
            .safety
            .without(AdvancedSafetyProperty::EarthFixedTermOwnership);
        assert_eq!(
            backend.validate(),
            Err(AdvancedRegistrationError::MissingSafetyInvariant)
        );
    }

    #[test]
    fn resource_rejection_identifies_required_and_available_bytes() {
        let estimate = estimate_resources(shape(), CalibrationPolicy::Fixed).unwrap();
        let mut small = limits();
        small.peak_memory_bytes = estimate.estimated_peak_memory_bytes - 1;
        let failure =
            preflight_with_registration(request(), small, Some(&registration())).unwrap_err();
        assert_eq!(
            failure.error,
            AdvancedPreflightError::InsufficientResource {
                resource: AdvancedResourceKind::PeakMemory,
                required: estimate.estimated_peak_memory_bytes,
                available: estimate.estimated_peak_memory_bytes - 1,
            }
        );
    }

    #[test]
    fn valid_injected_registration_prepares_without_exposing_graph_types() {
        let prepared =
            preflight_with_registration(request(), limits(), Some(&registration())).unwrap();
        assert_eq!(prepared.backend_version(), BackendVersionId::new(7));
        assert_eq!(prepared.native_source_digest(), digest(1));
        assert!(prepared.estimate().factor_count > 500);
    }

    #[test]
    fn refine_with_priors_requires_explicit_solve_for_coordinates() {
        let mut candidate = request();
        candidate.calibration_policy = CalibrationPolicy::RefineWithPriors;
        assert_eq!(
            preflight_with_registration(candidate, limits(), Some(&registration()))
                .unwrap_err()
                .error,
            AdvancedPreflightError::InvalidProblemShape
        );
        candidate.shape.refined_shared_parameter_coordinates = 6;
        assert!(preflight_with_registration(candidate, limits(), Some(&registration())).is_ok());
    }
}
