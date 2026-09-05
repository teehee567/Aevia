//! Bounded resident storage, host backing, and segment leases.

#[cfg(not(feature = "offline"))]
use super::MAX_EMBEDDED_TRAJECTORY_SEGMENTS;
use super::Trajectory;
#[cfg(feature = "offline")]
use super::TrajectoryKnot;
#[cfg(feature = "offline")]
use super::bridge::{BRIDGE_ENDPOINT_DIMENSION, DenseBridgeInput, DenseConditionalBridge};
#[cfg(feature = "offline")]
use super::codec::{decode_offline_segment_record, encode_offline_segment_record};
use super::dense::DenseSegment;
#[cfg(feature = "offline")]
use crate::error::ProcessError;
use crate::error::{QueryError, ValidationError};
#[cfg(feature = "offline")]
use crate::offline::{FIXED_RECORD_HEADER_BYTES, FixedRecordStore, FixedRecordStoreKind};
#[cfg(feature = "offline")]
use crate::time::TimeSpan;
#[cfg(not(feature = "offline"))]
use heapless::Vec as FixedVec;
#[cfg(feature = "offline")]
use std::boxed::Box;
#[cfg(all(test, feature = "offline"))]
use std::path::PathBuf;
#[cfg(feature = "offline")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "offline")]
use std::vec::Vec;

#[cfg(feature = "offline")]
pub(super) const OFFLINE_SEGMENT_MAGIC: [u8; 8] = *b"AEVTR03\0";

#[cfg(feature = "offline")]
pub(super) const OFFLINE_SEGMENT_CACHE_SLOTS: usize = 2;

#[cfg(feature = "offline")]
pub(super) const OFFLINE_KNOT_BYTES: u64 = 134;

#[cfg(feature = "offline")]
pub(super) const OFFLINE_SEGMENT_PAYLOAD_BYTES: u64 = 2 * OFFLINE_KNOT_BYTES
    + 2 * 8
    + 1
    + (BRIDGE_ENDPOINT_DIMENSION * BRIDGE_ENDPOINT_DIMENSION * 8) as u64
    + (4 * 3 * 3 * 8) as u64
    + (3 * 3 * 8) as u64;

#[cfg(feature = "offline")]
pub(super) const OFFLINE_BACKING_METADATA_ALLOWANCE_BYTES: u64 = 4 * 1_024;

#[cfg(feature = "offline")]
pub(super) type SegmentStorage = std::vec::Vec<DenseSegment>;

#[cfg(not(feature = "offline"))]
pub(super) type SegmentStorage = FixedVec<DenseSegment, MAX_EMBEDDED_TRAJECTORY_SEGMENTS>;

#[cfg(feature = "offline")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct OfflineTrajectoryStorageBounds {
    pub(crate) record_bytes: u64,
    pub(crate) memory_peak_bytes: u64,
    pub(crate) seekable_peak_bytes: u64,
    pub(crate) seekable_temporary_bytes: u64,
}

#[cfg(feature = "offline")]
#[derive(Debug)]
pub(super) struct BackedSegmentRecord {
    pub(super) ordinal: usize,
    pub(super) segment: DenseSegment,
    pub(super) conditional_bridge: DenseConditionalBridge,
    pub(super) store_indices: (u64, u64),
}

#[cfg(feature = "offline")]
pub(super) struct OfflineSegmentBacking {
    pub(super) store: FixedRecordStore,
    pub(super) io_buffer: Vec<u8>,
    pub(super) cache: [Option<Arc<BackedSegmentRecord>>; OFFLINE_SEGMENT_CACHE_SLOTS],
    pub(super) next_cache_slot: usize,
}

#[cfg(feature = "offline")]
impl core::fmt::Debug for OfflineSegmentBacking {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter
            .debug_struct("OfflineSegmentBacking")
            .field("store", &self.store)
            .field("cache_entries", &self.cache.iter().flatten().count())
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "offline")]
impl OfflineSegmentBacking {
    pub(super) fn new(
        kind: FixedRecordStoreKind,
        maximum_segments: u64,
    ) -> Result<Self, ProcessError> {
        let store = FixedRecordStore::new(
            kind,
            OFFLINE_SEGMENT_MAGIC,
            OFFLINE_SEGMENT_PAYLOAD_BYTES,
            maximum_segments,
        )?;
        let capacity = usize::try_from(OFFLINE_SEGMENT_PAYLOAD_BYTES)
            .map_err(|_| ProcessError::ResourceLimit)?;
        let mut io_buffer = Vec::new();
        io_buffer
            .try_reserve_exact(capacity)
            .map_err(|_| ProcessError::StorageExhausted)?;
        if io_buffer.capacity() != capacity {
            return Err(ProcessError::StorageExhausted);
        }
        io_buffer.resize(capacity, 0);
        Ok(Self {
            store,
            io_buffer,
            cache: core::array::from_fn(|_| None),
            next_cache_slot: 0,
        })
    }

    pub(super) fn push(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        input: DenseBridgeInput,
        store_indices: (u64, u64),
    ) -> Result<(), ProcessError> {
        encode_offline_segment_record(&mut self.io_buffer, start, end, &input, store_indices)
            .map_err(|_| ProcessError::NumericalNonConvergence)?;
        // Validate the exact object that a later cache miss will reconstruct
        // before committing its bytes to either backing.
        DenseSegment::new_conditional(start, end, &input)
            .map_err(|_| ProcessError::NumericalNonConvergence)?;
        self.store.push(&self.io_buffer).map_err(ProcessError::from)
    }

    pub(super) fn finish(&mut self) -> Result<(), ProcessError> {
        self.store.finish().map_err(ProcessError::from)
    }

    pub(super) fn get(&mut self, index: usize) -> Result<Arc<BackedSegmentRecord>, QueryError> {
        for cached in self.cache.iter().flatten() {
            if cached.ordinal == index {
                return Ok(Arc::clone(cached));
            }
        }
        let index_u64 = u64::try_from(index).map_err(|_| QueryError::InvalidRequest)?;
        self.store
            .read_into(index_u64, &mut self.io_buffer)
            .map_err(|_| QueryError::BackingStoreFailure)?;
        let record = Arc::new(
            decode_offline_segment_record(&self.io_buffer, index)
                .map_err(|_| QueryError::BackingStoreFailure)?,
        );
        let slot = self.next_cache_slot;
        self.cache[slot] = Some(Arc::clone(&record));
        self.next_cache_slot = (slot + 1) % OFFLINE_SEGMENT_CACHE_SLOTS;
        Ok(record)
    }
}

#[cfg(feature = "offline")]
pub(super) enum SegmentLease<'a> {
    Resident {
        segment: &'a DenseSegment,
        conditional_bridge: Option<&'a DenseConditionalBridge>,
    },
    Backed(Arc<BackedSegmentRecord>),
}

#[cfg(not(feature = "offline"))]
pub(super) enum SegmentLease<'a> {
    Resident { segment: &'a DenseSegment },
}

impl SegmentLease<'_> {
    pub(super) fn segment(&self) -> &DenseSegment {
        match self {
            Self::Resident { segment, .. } => segment,
            #[cfg(feature = "offline")]
            Self::Backed(record) => &record.segment,
        }
    }

    #[cfg(feature = "offline")]
    pub(super) fn conditional_bridge(&self) -> Option<&DenseConditionalBridge> {
        match self {
            Self::Resident {
                conditional_bridge, ..
            } => conditional_bridge.filter(|bridge| bridge.covariance_available),
            Self::Backed(record) => record
                .conditional_bridge
                .covariance_available
                .then_some(&record.conditional_bridge),
        }
    }

    #[cfg(feature = "offline")]
    pub(super) fn store_indices(&self) -> Option<(u64, u64)> {
        match self {
            Self::Resident { .. } => None,
            Self::Backed(record) => Some(record.store_indices),
        }
    }
}

#[cfg(feature = "offline")]
pub(super) const fn new_segment_storage() -> SegmentStorage {
    std::vec::Vec::new()
}

#[cfg(not(feature = "offline"))]
pub(super) const fn new_segment_storage() -> SegmentStorage {
    FixedVec::new()
}

#[cfg(feature = "offline")]
pub(super) fn push_segment(
    storage: &mut SegmentStorage,
    segment: DenseSegment,
) -> Result<(), ValidationError> {
    storage.push(segment);
    Ok(())
}

#[cfg(not(feature = "offline"))]
pub(super) fn push_segment(
    storage: &mut SegmentStorage,
    segment: DenseSegment,
) -> Result<(), ValidationError> {
    storage
        .push(segment)
        .map_err(|_| ValidationError::CapacityExceeded)
}

impl Trajectory {
    /// Reserves the complete host replay segment store before processing.
    ///
    /// Requiring the reported `Vec` capacity to equal the requested bound
    /// keeps the preflight byte calculation and the allocation contract in
    /// lockstep; a platform allocator that chooses a larger logical capacity
    /// fails closed instead of silently exceeding
    /// [`crate::config::OfflineResourceLimits`].
    #[cfg(feature = "offline")]
    pub(crate) fn try_reserve_segments_exact(
        &mut self,
        maximum_segments: usize,
    ) -> Result<(), ValidationError> {
        if !self.segments.is_empty() || self.segments.capacity() != 0 {
            return Err(ValidationError::IncompatibleDefinition);
        }
        self.segments
            .try_reserve_exact(maximum_segments)
            .map_err(|_| ValidationError::CapacityExceeded)?;
        self.conditional_bridges
            .try_reserve_exact(maximum_segments)
            .map_err(|_| ValidationError::CapacityExceeded)?;
        if self.segments.capacity() != maximum_segments
            || self.conditional_bridges.capacity() != maximum_segments
        {
            return Err(ValidationError::CapacityExceeded);
        }
        Ok(())
    }

    /// Exact logical bytes occupied by one host dense-segment allocation.
    #[cfg(feature = "offline")]
    pub(crate) const fn dense_segment_size_bytes() -> usize {
        core::mem::size_of::<DenseSegment>()
            + core::mem::size_of::<Option<Box<DenseConditionalBridge>>>()
    }

    /// Complete bounded resource model for a full-session offline trajectory.
    ///
    /// The cache bound includes two retained decoded segments, one transient
    /// replacement while an `Arc` lease is returned, each bridge's heap-owned
    /// 18-by-18 endpoint covariance, the reusable record I/O buffer, and a
    /// conservative fixed allowance for `Arc`/mutex/path/vector metadata.
    #[cfg(feature = "offline")]
    pub(crate) fn offline_storage_bounds(
        maximum_segments: u64,
    ) -> Result<OfflineTrajectoryStorageBounds, ValidationError> {
        if maximum_segments == 0 {
            return Err(ValidationError::CapacityExceeded);
        }
        let record_bytes = OFFLINE_SEGMENT_PAYLOAD_BYTES
            .checked_add(4)
            .ok_or(ValidationError::CapacityExceeded)?;
        let record_total = record_bytes
            .checked_mul(maximum_segments)
            .ok_or(ValidationError::CapacityExceeded)?;
        let decoded_record_bytes =
            u64::try_from(
                core::mem::size_of::<BackedSegmentRecord>()
                    + core::mem::size_of::<
                        [[f64; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION],
                    >()
                    + 2 * core::mem::size_of::<usize>(),
            )
            .map_err(|_| ValidationError::CapacityExceeded)?;
        let decoded_slots = u64::try_from(OFFLINE_SEGMENT_CACHE_SLOTS + 1)
            .map_err(|_| ValidationError::CapacityExceeded)?;
        // Decoding constructs a validated bridge from a temporary input. The
        // bridge currently owns a clone of that input's endpoint covariance,
        // so conservatively count one additional endpoint matrix during the
        // cache-miss handoff instead of relying on allocator reuse.
        let transient_endpoint_covariance_bytes = u64::try_from(core::mem::size_of::<
            [[f64; BRIDGE_ENDPOINT_DIMENSION]; BRIDGE_ENDPOINT_DIMENSION],
        >())
        .map_err(|_| ValidationError::CapacityExceeded)?;
        let cache_bytes = decoded_record_bytes
            .checked_mul(decoded_slots)
            .and_then(|bytes| bytes.checked_add(OFFLINE_SEGMENT_PAYLOAD_BYTES))
            .and_then(|bytes| bytes.checked_add(transient_endpoint_covariance_bytes))
            .and_then(|bytes| bytes.checked_add(OFFLINE_BACKING_METADATA_ALLOWANCE_BYTES))
            .ok_or(ValidationError::CapacityExceeded)?;
        Ok(OfflineTrajectoryStorageBounds {
            record_bytes,
            memory_peak_bytes: cache_bytes
                .checked_add(record_total)
                .ok_or(ValidationError::CapacityExceeded)?,
            seekable_peak_bytes: cache_bytes,
            seekable_temporary_bytes: record_total
                .checked_add(FIXED_RECORD_HEADER_BYTES)
                .ok_or(ValidationError::CapacityExceeded)?,
        })
    }

    #[cfg(feature = "offline")]
    pub(crate) fn prepare_offline_storage(
        &mut self,
        maximum_segments: u64,
        kind: FixedRecordStoreKind,
    ) -> Result<(), ProcessError> {
        if self.offline_backing.is_some()
            || !self.segments.is_empty()
            || !self.conditional_bridges.is_empty()
        {
            return Err(ProcessError::InvalidEvidence);
        }
        let maximum = usize::try_from(maximum_segments).map_err(|_| ProcessError::ResourceLimit)?;
        if maximum == 0 {
            return Err(ProcessError::ResourceLimit);
        }
        let backing = OfflineSegmentBacking::new(kind, maximum_segments)?;
        self.offline_backing = Some(Arc::new(Mutex::new(backing)));
        self.offline_segment_count = 0;
        self.offline_span = None;
        Ok(())
    }

    #[cfg(feature = "offline")]
    pub(crate) fn push_offline_conditional_bridge_segment(
        &mut self,
        start: TrajectoryKnot,
        end: TrajectoryKnot,
        input: DenseBridgeInput,
        store_indices: (u64, u64),
    ) -> Result<(), ProcessError> {
        let backing = self
            .offline_backing
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        if store_indices.1 <= store_indices.0
            || self
                .offline_span
                .is_some_and(|span| start.time < span.end())
        {
            return Err(ProcessError::NumericalNonConvergence);
        }
        let next_count = self
            .offline_segment_count
            .checked_add(1)
            .ok_or(ProcessError::ResourceLimit)?;
        backing
            .lock()
            .map_err(|_| ProcessError::StorageCorrupt)?
            .push(start, end, input, store_indices)?;
        self.offline_segment_count = next_count;
        self.offline_span = Some(
            TimeSpan::new(
                self.offline_span.map_or(start.time, TimeSpan::start),
                end.time,
            )
            .map_err(|_| ProcessError::NumericalNonConvergence)?,
        );
        Ok(())
    }

    #[cfg(feature = "offline")]
    pub(crate) fn finish_offline_storage(&mut self) -> Result<(), ProcessError> {
        let backing = self
            .offline_backing
            .as_ref()
            .ok_or(ProcessError::StorageCorrupt)?;
        if self.offline_segment_count == 0 {
            return Err(ProcessError::IncompleteEvidence);
        }
        backing
            .lock()
            .map_err(|_| ProcessError::StorageCorrupt)?
            .finish()
    }

    pub(super) fn segment_lease(&self, index: usize) -> Result<SegmentLease<'_>, QueryError> {
        #[cfg(feature = "offline")]
        if let Some(backing) = &self.offline_backing {
            if index >= self.offline_segment_count {
                return Err(QueryError::BackingStoreFailure);
            }
            return backing
                .lock()
                .map_err(|_| QueryError::BackingStoreFailure)?
                .get(index)
                .map(SegmentLease::Backed);
        }
        let segment = self
            .segments
            .get(index)
            .ok_or(QueryError::BackingStoreFailure)?;
        Ok(SegmentLease::Resident {
            segment,
            #[cfg(feature = "offline")]
            conditional_bridge: self
                .conditional_bridges
                .get(index)
                .ok_or(QueryError::BackingStoreFailure)?
                .as_deref(),
        })
    }

    #[cfg(feature = "offline")]
    pub(crate) fn offline_segment_store_indices(
        &self,
        index: usize,
    ) -> Result<(u64, u64), QueryError> {
        let lease = self.segment_lease(index)?;
        if let Some(indices) = lease.store_indices() {
            Ok(indices)
        } else {
            let start = u64::try_from(index).map_err(|_| QueryError::InvalidRequest)?;
            Ok((start, start.saturating_add(1)))
        }
    }

    #[cfg(all(test, feature = "offline"))]
    pub(super) fn offline_backing_path_for_test(&self) -> Option<PathBuf> {
        self.offline_backing
            .as_ref()?
            .lock()
            .ok()?
            .store
            .path_for_test()
    }

    #[cfg(all(test, feature = "offline"))]
    pub(super) fn corrupt_offline_record_for_test(
        &self,
        index: u64,
        byte: u64,
    ) -> Result<(), QueryError> {
        let backing = self
            .offline_backing
            .as_ref()
            .ok_or(QueryError::BackingStoreFailure)?;
        let mut backing = backing
            .lock()
            .map_err(|_| QueryError::BackingStoreFailure)?;
        backing.cache.fill(None);
        backing
            .store
            .corrupt_record_byte_for_test(index, byte)
            .map_err(|_| QueryError::BackingStoreFailure)
    }
}
