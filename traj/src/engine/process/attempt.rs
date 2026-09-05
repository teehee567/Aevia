//! Track external source and sink failures for one backend attempt.

use crate::error::ProcessError;
use crate::offline::{EvidenceEvent, ResultDescriptor, ResultRecord};

#[cfg(feature = "offline")]
pub(super) struct AttemptEvidenceSource<'a, S> {
    pub(super) inner: &'a mut S,
    pub(super) failed: bool,
}

#[cfg(feature = "offline")]
pub(super) struct AttemptResultSink<'a, K> {
    pub(super) inner: &'a mut K,
    pub(super) failed: bool,
}

#[cfg(feature = "offline")]
impl<S: crate::offline::EvidenceSource> crate::offline::EvidenceSource
    for AttemptEvidenceSource<'_, S>
{
    fn manifest(&self) -> crate::offline::EvidenceManifest {
        self.inner.manifest()
    }

    fn restart(&mut self) -> Result<(), ProcessError> {
        match self.inner.restart() {
            Ok(()) => Ok(()),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }

    fn next(&mut self) -> Result<Option<EvidenceEvent<'_>>, ProcessError> {
        match self.inner.next() {
            Ok(event) => Ok(event),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }
}

#[cfg(feature = "offline")]
impl<K: crate::offline::ResultSink> crate::offline::ResultSink for AttemptResultSink<'_, K> {
    fn preflight<'a>(
        &mut self,
        request: crate::offline::ResultSinkPreflight<'a>,
    ) -> Result<crate::offline::ResultSinkAttestation<'a>, ProcessError> {
        match self.inner.preflight(request) {
            Ok(attestation) => Ok(attestation),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }

    fn begin(&mut self, descriptor: ResultDescriptor<'_>) -> Result<(), ProcessError> {
        match self.inner.begin(descriptor) {
            Ok(()) => Ok(()),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }

    fn write(&mut self, record: ResultRecord<'_>) -> Result<(), ProcessError> {
        match self.inner.write(record) {
            Ok(()) => Ok(()),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }

    fn commit(&mut self) -> Result<(), ProcessError> {
        match self.inner.commit() {
            Ok(()) => Ok(()),
            Err(error) => {
                self.failed = true;
                Err(error)
            }
        }
    }

    fn abort(&mut self) {
        self.inner.abort();
    }
}
