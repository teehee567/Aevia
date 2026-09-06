//! Host processing preflight, backend execution, fallback, and result provenance.

#[cfg(feature = "offline")]
use self::attempt::{AttemptEvidenceSource, AttemptResultSink};
#[cfg(feature = "offline")]
use self::replay::run_captured_replay;
#[cfg(feature = "offline")]
use self::selection::runtime_failure_allows_fallback;
#[cfg(feature = "offline")]
use self::selection::{
    attempt_outcome_for_process_error, process_error_for_prepare_error, select_next_offline_level,
    select_offline_level,
};
#[cfg(feature = "offline")]
use super::bindings::validate_surveyed_gate_bindings;
use crate::config::ProcessingSpec;
#[cfg(feature = "offline")]
use crate::config::{OfflineResourceLimits, ProcessingLevel, RunControl};
use crate::error::PrepareError;
#[cfg(feature = "offline")]
use crate::error::ProcessError;
#[cfg(feature = "offline")]
use crate::ids::BackendVersionId;
#[cfg(feature = "offline")]
use crate::provenance::{
    BackendProvenance, Capabilities, Capability, MAX_PROCESSING_ATTEMPTS, ProcessingAttempt,
    ProcessingAttemptOutcome, ResultProvenance,
};

#[cfg(feature = "offline")]
mod attempt;
#[cfg(feature = "offline")]
mod replay;
#[cfg(feature = "offline")]
mod selection;

#[cfg(test)]
#[cfg(feature = "offline")]
mod preflight_tests;

/// Host process builder. The concrete source/sink types remain generic at run
/// time and no host dependency enters the default embedded build.
#[derive(Clone, Debug)]
pub struct ProcessBuilder<'a> {
    #[cfg(feature = "offline")]
    spec: ProcessingSpec<'a>,
    #[cfg(not(feature = "offline"))]
    marker: core::marker::PhantomData<&'a ()>,
}

/// Preflighted host request. Optional native backends remain private adapters.
#[derive(Clone, Debug)]
pub struct PreparedProcess<'a> {
    #[cfg(feature = "offline")]
    spec: ProcessingSpec<'a>,
    #[cfg(feature = "offline")]
    level: ProcessingLevel,
    #[cfg(feature = "offline")]
    manifest: crate::offline::EvidenceManifest,
    #[cfg(feature = "offline")]
    limits: OfflineResourceLimits,
    #[cfg(feature = "offline")]
    attempts: heapless::Vec<ProcessingAttempt, MAX_PROCESSING_ATTEMPTS>,
    #[cfg(not(feature = "offline"))]
    marker: core::marker::PhantomData<&'a ()>,
}

#[cfg(feature = "offline")]
impl<'a> ProcessBuilder<'a> {
    pub fn preflight(
        self,
        manifest: crate::offline::EvidenceManifest,
        limits: OfflineResourceLimits,
    ) -> Result<PreparedProcess<'a>, PrepareError> {
        self.spec
            .validate()
            .map_err(PrepareError::InvalidDefinition)?;
        validate_surveyed_gate_bindings(
            &self.spec.engine,
            &self.spec.metrics,
            Some(manifest.span_capabilities.span),
        )?;
        if !self.spec.engine.is_qualified() {
            return Err(PrepareError::UnqualifiedProfile);
        }
        limits.validate().map_err(PrepareError::InvalidDefinition)?;
        manifest
            .validate()
            .map_err(PrepareError::InvalidDefinition)?;
        let manifest_span = manifest.span_capabilities.span;
        let lineage_digest = self
            .spec
            .evidence_lineage
            .canonical_digest_v1()
            .map_err(PrepareError::InvalidDefinition)?;
        let lineage_outside_manifest =
            self.spec
                .evidence_lineage
                .selections()
                .iter()
                .any(|selection| {
                    !manifest_span.contains(selection.span.start())
                        || !manifest_span.contains(selection.span.end())
                });
        if manifest.configuration_digest != self.spec.engine.digest
            || manifest.normalization_digest != lineage_digest
            || !manifest
                .span_capabilities
                .span
                .contains(self.spec.span.start())
            || !manifest
                .span_capabilities
                .span
                .contains(self.spec.span.end())
            || !manifest.span_capabilities.has_valid_end
            || !manifest.restartable
            || lineage_outside_manifest
        {
            return Err(if !manifest.restartable {
                PrepareError::NotRestartable
            } else {
                PrepareError::EvidenceUnavailable
            });
        }
        let (level, attempts) = select_offline_level(&self.spec, manifest, limits)?;
        Ok(PreparedProcess {
            spec: self.spec,
            level,
            manifest,
            limits,
            attempts,
        })
    }
}

#[cfg(not(feature = "offline"))]
impl ProcessBuilder<'_> {
    /// The embedded-only crate intentionally has no host evidence/source types.
    pub fn preflight_unavailable(self) -> Result<(), PrepareError> {
        let _ = self;
        Err(PrepareError::CapabilityUnavailable)
    }
}

#[cfg(feature = "offline")]
impl PreparedProcess<'_> {
    #[must_use]
    pub const fn selected_level(&self) -> ProcessingLevel {
        self.level
    }

    pub fn run<S: crate::offline::EvidenceSource, K: crate::offline::ResultSink>(
        &self,
        source: &mut S,
        sink: &mut K,
        control: RunControl<'_>,
    ) -> Result<crate::offline::OfflineRun, ProcessError> {
        let mut level = self.level;
        let mut attempts = self.attempts.clone();
        loop {
            let (result, external_port_failed) = {
                let mut tracked_source = AttemptEvidenceSource {
                    inner: source,
                    failed: false,
                };
                let mut tracked_sink = AttemptResultSink {
                    inner: sink,
                    failed: false,
                };
                let result = self.run_candidate(
                    level,
                    attempts.as_slice(),
                    &mut tracked_source,
                    &mut tracked_sink,
                    control,
                );
                (result, tracked_source.failed || tracked_sink.failed)
            };
            match result {
                Ok(run) => return Ok(run),
                Err(error)
                    if matches!(
                        self.spec.policy,
                        crate::config::ProcessingPolicy::BestQualified { .. }
                    ) && !external_port_failed
                        && runtime_failure_allows_fallback(error) =>
                {
                    let current = attempts
                        .last_mut()
                        .ok_or(ProcessError::AdvancedCapabilityFailure)?;
                    if current.level != level
                        || current.outcome != ProcessingAttemptOutcome::Succeeded
                    {
                        return Err(ProcessError::AdvancedCapabilityFailure);
                    }
                    current.outcome = attempt_outcome_for_process_error(error);
                    level = match select_next_offline_level(
                        &self.spec,
                        self.manifest,
                        self.limits,
                        &mut attempts,
                    )
                    .map_err(process_error_for_prepare_error)?
                    {
                        Some(next) => next,
                        None => return Err(error),
                    };
                }
                Err(error) => return Err(error),
            }
        }
    }

    fn run_candidate<S: crate::offline::EvidenceSource, K: crate::offline::ResultSink>(
        &self,
        level: ProcessingLevel,
        attempts: &[ProcessingAttempt],
        source: &mut S,
        sink: &mut K,
        control: RunControl<'_>,
    ) -> Result<crate::offline::OfflineRun, ProcessError> {
        match level {
            ProcessingLevel::OfflineSmooth => {
                let provenance = self.result_provenance(
                    ProcessingLevel::OfflineSmooth,
                    Capabilities::NONE
                        .with(Capability::OfflineSmooth)
                        .with(Capability::FullOfflineMetrics),
                    attempts,
                );
                crate::offline::run_offline(
                    &self.spec,
                    self.manifest,
                    self.limits,
                    provenance,
                    source,
                    sink,
                    control,
                )
            }
            ProcessingLevel::CapturedReplay => run_captured_replay(
                &self.spec,
                self.manifest,
                self.limits,
                self.result_provenance(
                    ProcessingLevel::CapturedReplay,
                    Capabilities::NONE
                        .with(Capability::CapturedReplay)
                        .with(Capability::FullOfflineMetrics),
                    attempts,
                ),
                source,
                sink,
                control,
            ),
            ProcessingLevel::AdvancedGraph | ProcessingLevel::RawTight => {
                Err(ProcessError::CapabilityUnavailable)
            }
            ProcessingLevel::EmbeddedLive => Err(ProcessError::CapabilityUnavailable),
        }
    }

    fn result_provenance<'run>(
        &'run self,
        level: ProcessingLevel,
        capabilities: Capabilities,
        attempts: &'run [ProcessingAttempt],
    ) -> ResultProvenance<'run> {
        ResultProvenance {
            result_revision: self.spec.result.result_revision,
            source_session: self.manifest.session_id,
            source_span: self.spec.span,
            source_digest: self.manifest.source_logical_digest,
            normalization_digest: self.manifest.normalization_digest,
            configuration_digest: self.spec.engine.digest,
            installation_digest: self.spec.engine.installation.digest,
            calibration_revision: self.spec.engine.calibration.revision,
            calibration_digest: self.spec.engine.calibration.digest,
            uncertainty_digest: self.spec.result.uncertainty_digest,
            metric_plan_digest: self.spec.result.metric_plan_digest,
            requested_policy: self.spec.policy,
            actual_backend: BackendProvenance {
                level,
                version: BackendVersionId::new(1),
                native_source_digest: None,
            },
            attempts,
            parents: self.spec.result.parents,
            external_inputs: self.spec.result.external_inputs,
            capabilities,
        }
    }
}

impl<'a> ProcessBuilder<'a> {
    pub(super) fn new(spec: ProcessingSpec<'a>) -> Self {
        #[cfg(feature = "offline")]
        {
            Self { spec }
        }
        #[cfg(not(feature = "offline"))]
        {
            let _ = spec;
            Self {
                marker: core::marker::PhantomData,
            }
        }
    }
}
