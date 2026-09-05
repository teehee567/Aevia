//! Port fixture.

use super::*;

pub(super) struct FailingEvidenceSource<'a> {
    pub(super) inner: SliceEvidenceSource<'a>,
    pub(super) fail_next: bool,
}

impl EvidenceSource for FailingEvidenceSource<'_> {
    fn manifest(&self) -> EvidenceManifest {
        self.inner.manifest()
    }

    fn restart(&mut self) -> Result<(), ProcessError> {
        self.inner.restart()
    }

    fn next(&mut self) -> Result<Option<EvidenceEvent<'_>>, ProcessError> {
        if self.fail_next {
            self.fail_next = false;
            Err(ProcessError::ResourceLimit)
        } else {
            self.inner.next()
        }
    }
}

pub(super) struct MutatingRestartEvidenceSource {
    pub(super) manifest: EvidenceManifest,
    pub(super) baseline: Vec<EvidenceEvent<'static>>,
    pub(super) changed: Vec<EvidenceEvent<'static>>,
    pub(super) restart_count: u32,
    pub(super) index: usize,
}

impl EvidenceSource for MutatingRestartEvidenceSource {
    fn manifest(&self) -> EvidenceManifest {
        self.manifest
    }

    fn restart(&mut self) -> Result<(), ProcessError> {
        self.restart_count = self.restart_count.saturating_add(1);
        self.index = 0;
        Ok(())
    }

    fn next(&mut self) -> Result<Option<EvidenceEvent<'_>>, ProcessError> {
        let events = if self.restart_count <= 1 {
            &self.baseline
        } else {
            &self.changed
        };
        let event = events.get(self.index).copied();
        self.index = self.index.saturating_add(usize::from(event.is_some()));
        Ok(event)
    }
}

pub(super) struct RecordingSink {
    pub(super) preflights: u32,
    pub(super) begins: u32,
    pub(super) commits: u32,
    pub(super) aborts: u32,
    pub(super) active: bool,
    pub(super) reserved: bool,
    pub(super) resource_limit_on_preflight: Option<u32>,
    pub(super) resource_limit_on_write: Option<u64>,
    pub(super) resource_limit_on_every_write: bool,
    pub(super) write_attempts: u64,
    pub(super) attested_bytes: u64,
    pub(super) preflight_backends: Vec<ProcessingLevel>,
    pub(super) begun_backends: Vec<ProcessingLevel>,
    pub(super) staged_backend: Option<ProcessingLevel>,
    pub(super) staged_attempts: Vec<ProcessingAttempt>,
    pub(super) staged_states: u64,
    pub(super) staged_metrics: u32,
    pub(super) staged_metric_results: usize,
    pub(super) staged_end: Option<ResultEnd>,
    pub(super) backend: Option<ProcessingLevel>,
    pub(super) attempts: Vec<ProcessingAttempt>,
    pub(super) states: u64,
    pub(super) metrics: u32,
    pub(super) metric_results: usize,
    pub(super) end: Option<ResultEnd>,
}

impl Default for RecordingSink {
    fn default() -> Self {
        Self {
            preflights: 0,
            begins: 0,
            commits: 0,
            aborts: 0,
            active: false,
            reserved: false,
            resource_limit_on_preflight: None,
            resource_limit_on_write: None,
            resource_limit_on_every_write: false,
            write_attempts: 0,
            attested_bytes: 0,
            preflight_backends: Vec::new(),
            begun_backends: Vec::new(),
            staged_backend: None,
            staged_attempts: Vec::new(),
            staged_states: 0,
            staged_metrics: 0,
            staged_metric_results: 0,
            staged_end: None,
            backend: None,
            attempts: Vec::new(),
            states: 0,
            metrics: 0,
            metric_results: 0,
            end: None,
        }
    }
}

pub(super) fn recording_sink_transaction_bytes(
    request: ResultSinkPreflight<'_>,
) -> Result<u64, ProcessError> {
    // Sink-specific mock framing: the fixture defines its own exact byte
    // representation instead of borrowing an in-memory Rust struct size.
    let descriptor = request.descriptor();
    let provenance_entries = descriptor
        .provenance
        .attempts
        .len()
        .checked_add(descriptor.provenance.parents.len())
        .and_then(|count| count.checked_add(descriptor.provenance.external_inputs.len()))
        .and_then(|count| u64::try_from(count).ok())
        .ok_or(ProcessError::ResourceLimit)?;
    let records = request.records();
    u64::from(records.descriptor_records())
        .checked_mul(16)
        .and_then(|bytes| {
            provenance_entries
                .checked_mul(8)
                .and_then(|entries| bytes.checked_add(entries))
        })
        .and_then(|bytes| {
            records
                .maximum_state_records()
                .checked_mul(32)
                .and_then(|states| bytes.checked_add(states))
        })
        .and_then(|bytes| bytes.checked_add(u64::from(records.metric_frames()) * 16))
        .and_then(|bytes| {
            records
                .maximum_metric_results()
                .checked_mul(8)
                .and_then(|metrics| bytes.checked_add(metrics))
        })
        .and_then(|bytes| bytes.checked_add(u64::from(records.end_records()) * 16))
        .ok_or(ProcessError::ResourceLimit)
}

impl ResultSink for RecordingSink {
    fn preflight<'a>(
        &mut self,
        request: ResultSinkPreflight<'a>,
    ) -> Result<ResultSinkAttestation<'a>, ProcessError> {
        assert!(!self.active);
        self.preflights += 1;
        self.preflight_backends
            .push(request.descriptor().provenance.actual_backend.level);
        if self.resource_limit_on_preflight == Some(self.preflights) {
            return Err(ProcessError::ResourceLimit);
        }
        let bytes = recording_sink_transaction_bytes(request)?;
        let attestation = request.attest(bytes)?;
        self.attested_bytes = attestation.exact_transaction_bytes();
        self.reserved = true;
        Ok(attestation)
    }

    fn begin(&mut self, descriptor: ResultDescriptor<'_>) -> Result<(), ProcessError> {
        assert!(!self.active);
        assert!(self.reserved);
        self.begins += 1;
        self.active = true;
        self.begun_backends
            .push(descriptor.provenance.actual_backend.level);
        self.staged_backend = Some(descriptor.provenance.actual_backend.level);
        self.staged_attempts = descriptor.provenance.attempts.to_vec();
        Ok(())
    }

    fn write(&mut self, record: ResultRecord<'_>) -> Result<(), ProcessError> {
        assert!(self.active);
        self.write_attempts += 1;
        if self.resource_limit_on_every_write
            || self.resource_limit_on_write == Some(self.write_attempts)
        {
            return Err(ProcessError::ResourceLimit);
        }
        match record {
            ResultRecord::State(_) => self.staged_states += 1,
            ResultRecord::Metrics(values) => {
                self.staged_metrics += 1;
                self.staged_metric_results = values.len();
            }
            ResultRecord::End(end) => self.staged_end = Some(end),
        }
        Ok(())
    }

    fn commit(&mut self) -> Result<(), ProcessError> {
        assert!(self.active);
        self.active = false;
        self.reserved = false;
        self.commits += 1;
        self.backend = self.staged_backend.take();
        self.attempts = core::mem::take(&mut self.staged_attempts);
        self.states = self.staged_states;
        self.metrics = self.staged_metrics;
        self.metric_results = self.staged_metric_results;
        self.end = self.staged_end.take();
        self.staged_states = 0;
        self.staged_metrics = 0;
        self.staged_metric_results = 0;
        Ok(())
    }

    fn abort(&mut self) {
        if self.active || self.reserved {
            self.active = false;
            self.reserved = false;
            self.aborts += 1;
        }
        self.staged_backend = None;
        self.staged_attempts.clear();
        self.staged_states = 0;
        self.staged_metrics = 0;
        self.staged_metric_results = 0;
        self.staged_end = None;
    }
}
