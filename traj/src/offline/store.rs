//! Private bounded fixed-record state storage.
//!
//! The seekable adapter uses a constant-size record derived during preflight,
//! so random and reverse access need no session-sized in-memory index.  Every
//! record has its own CRC-32C and is decoded with exact dimension/length checks.

use std::{
    boxed::Box,
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
    vec::Vec,
};

use nalgebra::DMatrix;

use crate::{
    config::OfflineResourceLimits,
    error::ProcessError,
    math::UnitQuaternion,
    observation::InputDisposition,
    quality::{GnssState, TimingQuality},
    time::SessionTime,
};

const FILE_MAGIC: [u8; 8] = *b"AEVST01\0";
const FILE_HEADER_BYTES: u64 = 64;
const FILE_FORMAT_VERSION: u32 = 3;
const MAX_TEMPFILE_ATTEMPTS: u64 = 32;

#[derive(Clone, Debug)]
pub(super) struct StoredNominal {
    pub time: SessionTime,
    pub position_ecef: [f64; 3],
    pub velocity_ecef: [f64; 3],
    pub orientation_ecef_from_body: UnitQuaternion,
    pub accelerometer_bias_body: [f64; 3],
    pub gyroscope_bias_body: [f64; 3],
    pub colored_gnss_error: [f64; 3],
    pub specific_force_body: [f64; 3],
    pub angular_rate_body: [f64; 3],
}

impl StoredNominal {
    fn encoded_len() -> usize {
        // time + p/v/q/ba/bg/colored/f/omega
        8 + (3 + 3 + 4 + 3 + 3 + 3 + 3 + 3) * 8
    }

    fn encode(&self, bytes: &mut Vec<u8>) {
        put_i64(bytes, self.time.as_ns());
        for value in self
            .position_ecef
            .iter()
            .chain(self.velocity_ecef.iter())
            .chain(self.orientation_ecef_from_body.components_wxyz().iter())
            .chain(self.accelerometer_bias_body.iter())
            .chain(self.gyroscope_bias_body.iter())
            .chain(self.colored_gnss_error.iter())
            .chain(self.specific_force_body.iter())
            .chain(self.angular_rate_body.iter())
        {
            put_f64(bytes, *value);
        }
    }

    fn decode(cursor: &mut DecodeCursor<'_>) -> Result<Self, StoreError> {
        let time = SessionTime::from_ns(cursor.i64()?);
        let position_ecef = cursor.array3()?;
        let velocity_ecef = cursor.array3()?;
        let orientation_ecef_from_body =
            UnitQuaternion::from_wxyz(cursor.array4()?).map_err(|_| StoreError::Corrupt)?;
        let accelerometer_bias_body = cursor.array3()?;
        let gyroscope_bias_body = cursor.array3()?;
        let colored_gnss_error = cursor.array3()?;
        let specific_force_body = cursor.array3()?;
        let angular_rate_body = cursor.array3()?;
        Ok(Self {
            time,
            position_ecef,
            velocity_ecef,
            orientation_ecef_from_body,
            accelerometer_bias_body,
            gyroscope_bias_body,
            colored_gnss_error,
            specific_force_body,
            angular_rate_body,
        })
    }

    pub(super) fn is_finite(&self) -> bool {
        self.position_ecef
            .iter()
            .chain(self.velocity_ecef.iter())
            .chain(self.orientation_ecef_from_body.components_wxyz().iter())
            .chain(self.accelerometer_bias_body.iter())
            .chain(self.gyroscope_bias_body.iter())
            .chain(self.colored_gnss_error.iter())
            .chain(self.specific_force_body.iter())
            .chain(self.angular_rate_body.iter())
            .all(|value| value.is_finite())
    }
}

#[derive(Clone, Debug)]
pub(super) struct StoredCovariance {
    pub state: DMatrix<f64>,
    pub state_consider: DMatrix<f64>,
}

impl StoredCovariance {
    fn encode(&self, bytes: &mut Vec<u8>) {
        put_matrix(bytes, &self.state);
        put_matrix(bytes, &self.state_consider);
    }

    fn decode(
        cursor: &mut DecodeCursor<'_>,
        state_dimension: usize,
        consider_dimension: usize,
    ) -> Result<Self, StoreError> {
        Ok(Self {
            state: cursor.matrix(state_dimension, state_dimension)?,
            state_consider: cursor.matrix(state_dimension, consider_dimension)?,
        })
    }

    fn all_finite(&self) -> bool {
        self.state.iter().all(|value| value.is_finite())
            && self.state_consider.iter().all(|value| value.is_finite())
    }
}

/// Exact interval-average IMU value used to form one stored transition.
///
/// This is kept separate from [`StoredNominal::specific_force_body`] and
/// `angular_rate_body`: a same-epoch measurement update may change the
/// endpoint attitude/bias tangent state, while dense reintegration must use
/// the immutable sensor value that generated the process interval.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct StoredIntegrationImu {
    pub start: SessionTime,
    pub end: SessionTime,
    pub angular_rate_body: [f64; 3],
    pub specific_force_body: [f64; 3],
}

impl StoredIntegrationImu {
    const fn encoded_len() -> usize {
        // start/end + angular rate + specific force
        2 * 8 + 6 * 8
    }

    fn encode(&self, bytes: &mut Vec<u8>) {
        put_i64(bytes, self.start.as_ns());
        put_i64(bytes, self.end.as_ns());
        for value in self
            .angular_rate_body
            .iter()
            .chain(self.specific_force_body.iter())
        {
            put_f64(bytes, *value);
        }
    }

    fn decode(cursor: &mut DecodeCursor<'_>) -> Result<Self, StoreError> {
        let result = Self {
            start: SessionTime::from_ns(cursor.i64()?),
            end: SessionTime::from_ns(cursor.i64()?),
            angular_rate_body: cursor.array3()?,
            specific_force_body: cursor.array3()?,
        };
        if result.is_valid() {
            Ok(result)
        } else {
            Err(StoreError::Corrupt)
        }
    }

    fn is_valid(&self) -> bool {
        self.start < self.end
            && self
                .angular_rate_body
                .iter()
                .chain(self.specific_force_body.iter())
                .all(|value| value.is_finite())
    }
}

#[derive(Clone, Debug)]
pub(super) struct StoredStep {
    pub connected_from_previous: bool,
    pub predicted: StoredNominal,
    pub filtered: StoredNominal,
    pub smoothed: Option<StoredNominal>,
    pub predicted_covariance: StoredCovariance,
    pub filtered_covariance: StoredCovariance,
    pub smoothed_covariance: Option<StoredCovariance>,
    /// Estimable-state transition from the previous stored epoch.
    pub transition: DMatrix<f64>,
    /// Fixed-mean consider sensitivity from the previous stored epoch.
    pub consider_transition: DMatrix<f64>,
    pub process_covariance: DMatrix<f64>,
    /// Immutable IMU interval used by `transition`/`process_covariance`.
    /// `None` is valid only for an initialization or discontinuous epoch.
    pub integration_imu: Option<StoredIntegrationImu>,
    pub reset_basis: DMatrix<f64>,
    /// Backward conditional from the next smoothed augmented error state into
    /// this one, expressed in both epochs' final smoothed tangent bases. The
    /// augmented ordering is `[estimable state, fixed consider parameters]`.
    /// It is populated by the RTS/consider pass and permits adjacent and
    /// arbitrary cross-time covariance recovery without an O(N^2) store.
    pub smoothed_backward_gain: Option<DMatrix<f64>>,
    /// Forward filtered-to-predicted estimable-state cross block. This is not
    /// a smoothed adjacent covariance; it remains separate from the backward
    /// conditional above so the two semantics cannot be confused.
    pub adjacent_cross_covariance: DMatrix<f64>,
    pub disposition: Option<InputDisposition>,
    pub gnss_state: GnssState,
    pub timing_quality: TimingQuality,
    pub degraded_input: bool,
    pub objective_contribution: f64,
}

impl StoredStep {
    pub(super) fn encoded_len(state_dimension: usize, consider_dimension: usize) -> Option<usize> {
        let nominal = StoredNominal::encoded_len();
        let covariance = matrix_bytes(state_dimension, state_dimension)?
            .checked_add(matrix_bytes(state_dimension, consider_dimension)?)?;
        // flags/enums/objective + immutable integration IMU +
        // predicted/filtered/smoothed nominals + three covariance triples +
        // transition/Gamma/Q/reset/adjacent-cross.
        let augmented_dimension = state_dimension.checked_add(consider_dimension)?;
        17_usize
            .checked_add(StoredIntegrationImu::encoded_len())?
            .checked_add(3_usize.checked_mul(nominal)?)?
            .checked_add(3_usize.checked_mul(covariance)?)?
            .checked_add(4_usize.checked_mul(matrix_bytes(state_dimension, state_dimension)?)?)?
            .checked_add(matrix_bytes(state_dimension, consider_dimension)?)?
            .checked_add(matrix_bytes(augmented_dimension, augmented_dimension)?)
    }

    fn validate_dimensions(
        &self,
        state_dimension: usize,
        consider_dimension: usize,
    ) -> Result<(), StoreError> {
        let covariance_ok = |value: &StoredCovariance| {
            value.state.shape() == (state_dimension, state_dimension)
                && value.state_consider.shape() == (state_dimension, consider_dimension)
                && value.all_finite()
        };
        if !self.predicted.is_finite()
            || !self.filtered.is_finite()
            || self
                .smoothed
                .as_ref()
                .is_some_and(|value| !value.is_finite())
            || !covariance_ok(&self.predicted_covariance)
            || !covariance_ok(&self.filtered_covariance)
            || self
                .smoothed_covariance
                .as_ref()
                .is_some_and(|value| !covariance_ok(value))
            || self.transition.shape() != (state_dimension, state_dimension)
            || self.consider_transition.shape() != (state_dimension, consider_dimension)
            || self.process_covariance.shape() != (state_dimension, state_dimension)
            || self
                .integration_imu
                .as_ref()
                .is_some_and(|value| !value.is_valid())
            || (self.connected_from_previous && self.integration_imu.is_none())
            || self
                .integration_imu
                .as_ref()
                .is_some_and(|value| value.end != self.filtered.time)
            || self.reset_basis.shape() != (state_dimension, state_dimension)
            || self.smoothed_backward_gain.as_ref().is_some_and(|value| {
                let augmented_dimension = state_dimension.saturating_add(consider_dimension);
                value.shape() != (augmented_dimension, augmented_dimension)
                    || !value.iter().all(|entry| entry.is_finite())
            })
            || self.adjacent_cross_covariance.shape() != (state_dimension, state_dimension)
            || !self.transition.iter().all(|value| value.is_finite())
            || !self
                .consider_transition
                .iter()
                .all(|value| value.is_finite())
            || !self
                .process_covariance
                .iter()
                .all(|value| value.is_finite())
            || !self.reset_basis.iter().all(|value| value.is_finite())
            || !self
                .adjacent_cross_covariance
                .iter()
                .all(|value| value.is_finite())
            || !self.objective_contribution.is_finite()
        {
            return Err(StoreError::Dimension);
        }
        Ok(())
    }

    fn encode(
        &self,
        state_dimension: usize,
        consider_dimension: usize,
    ) -> Result<Vec<u8>, StoreError> {
        self.validate_dimensions(state_dimension, consider_dimension)?;
        let expected = Self::encoded_len(state_dimension, consider_dimension)
            .ok_or(StoreError::IntegerOverflow)?;
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(expected)
            .map_err(|_| StoreError::Exhausted)?;
        bytes.push(u8::from(self.connected_from_previous));
        bytes.push(u8::from(self.smoothed.is_some()));
        bytes.push(u8::from(self.smoothed_covariance.is_some()));
        bytes.push(encode_disposition(self.disposition));
        bytes.push(encode_gnss(self.gnss_state));
        bytes.push(encode_timing(self.timing_quality));
        bytes.push(u8::from(self.degraded_input));
        bytes.push(u8::from(self.smoothed_backward_gain.is_some()));
        bytes.push(u8::from(self.integration_imu.is_some()));
        self.integration_imu
            .as_ref()
            .unwrap_or(&StoredIntegrationImu {
                start: SessionTime::ZERO,
                end: SessionTime::from_ns(10),
                angular_rate_body: [0.0; 3],
                specific_force_body: [0.0; 3],
            })
            .encode(&mut bytes);
        self.predicted.encode(&mut bytes);
        self.filtered.encode(&mut bytes);
        self.smoothed
            .as_ref()
            .unwrap_or(&self.filtered)
            .encode(&mut bytes);
        self.predicted_covariance.encode(&mut bytes);
        self.filtered_covariance.encode(&mut bytes);
        self.smoothed_covariance
            .as_ref()
            .unwrap_or(&self.filtered_covariance)
            .encode(&mut bytes);
        put_matrix(&mut bytes, &self.transition);
        put_matrix(&mut bytes, &self.consider_transition);
        put_matrix(&mut bytes, &self.process_covariance);
        put_matrix(&mut bytes, &self.reset_basis);
        if let Some(gain) = &self.smoothed_backward_gain {
            put_matrix(&mut bytes, gain);
        } else {
            let augmented_dimension = state_dimension
                .checked_add(consider_dimension)
                .ok_or(StoreError::IntegerOverflow)?;
            let zeros = augmented_dimension
                .checked_mul(augmented_dimension)
                .ok_or(StoreError::IntegerOverflow)?;
            for _ in 0..zeros {
                put_f64(&mut bytes, 0.0);
            }
        }
        put_matrix(&mut bytes, &self.adjacent_cross_covariance);
        put_f64(&mut bytes, self.objective_contribution);
        if bytes.len() != expected {
            return Err(StoreError::Dimension);
        }
        Ok(bytes)
    }

    fn decode(
        bytes: &[u8],
        state_dimension: usize,
        consider_dimension: usize,
    ) -> Result<Self, StoreError> {
        if bytes.len()
            != Self::encoded_len(state_dimension, consider_dimension)
                .ok_or(StoreError::IntegerOverflow)?
        {
            return Err(StoreError::Corrupt);
        }
        let mut cursor = DecodeCursor { bytes, position: 0 };
        let connected_from_previous = cursor.boolean()?;
        let has_smoothed = cursor.boolean()?;
        let has_smoothed_covariance = cursor.boolean()?;
        let disposition = decode_disposition(cursor.u8()?)?;
        let gnss_state = decode_gnss(cursor.u8()?)?;
        let timing_quality = decode_timing(cursor.u8()?)?;
        let degraded_input = cursor.boolean()?;
        let has_smoothed_backward_gain = cursor.boolean()?;
        let has_integration_imu = cursor.boolean()?;
        let decoded_integration_imu = StoredIntegrationImu::decode(&mut cursor)?;
        let predicted = StoredNominal::decode(&mut cursor)?;
        let filtered = StoredNominal::decode(&mut cursor)?;
        let decoded_smoothed = StoredNominal::decode(&mut cursor)?;
        let predicted_covariance =
            StoredCovariance::decode(&mut cursor, state_dimension, consider_dimension)?;
        let filtered_covariance =
            StoredCovariance::decode(&mut cursor, state_dimension, consider_dimension)?;
        let decoded_smoothed_covariance =
            StoredCovariance::decode(&mut cursor, state_dimension, consider_dimension)?;
        let transition = cursor.matrix(state_dimension, state_dimension)?;
        let consider_transition = cursor.matrix(state_dimension, consider_dimension)?;
        let process_covariance = cursor.matrix(state_dimension, state_dimension)?;
        let reset_basis = cursor.matrix(state_dimension, state_dimension)?;
        let augmented_dimension = state_dimension
            .checked_add(consider_dimension)
            .ok_or(StoreError::IntegerOverflow)?;
        let decoded_smoothed_backward_gain =
            cursor.matrix(augmented_dimension, augmented_dimension)?;
        let adjacent_cross_covariance = cursor.matrix(state_dimension, state_dimension)?;
        let objective_contribution = cursor.f64()?;
        if cursor.position != bytes.len() {
            return Err(StoreError::Corrupt);
        }
        let result = Self {
            connected_from_previous,
            predicted,
            filtered,
            smoothed: has_smoothed.then_some(decoded_smoothed),
            predicted_covariance,
            filtered_covariance,
            smoothed_covariance: has_smoothed_covariance.then_some(decoded_smoothed_covariance),
            transition,
            consider_transition,
            process_covariance,
            integration_imu: has_integration_imu.then_some(decoded_integration_imu),
            reset_basis,
            smoothed_backward_gain: has_smoothed_backward_gain
                .then_some(decoded_smoothed_backward_gain),
            adjacent_cross_covariance,
            disposition,
            gnss_state,
            timing_quality,
            degraded_input,
            objective_contribution,
        };
        result.validate_dimensions(state_dimension, consider_dimension)?;
        Ok(result)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum StoreError {
    Exhausted,
    Corrupt,
    ReadIo,
    WriteIo,
    Dimension,
    IntegerOverflow,
    OutOfRange,
}

impl From<StoreError> for ProcessError {
    fn from(value: StoreError) -> Self {
        match value {
            StoreError::Exhausted | StoreError::WriteIo => Self::StorageExhausted,
            StoreError::Corrupt | StoreError::Dimension | StoreError::ReadIo => {
                Self::StorageCorrupt
            }
            StoreError::IntegerOverflow | StoreError::OutOfRange => Self::ResourceLimit,
        }
    }
}

pub(super) trait StateStore {
    fn dimensions(&self) -> (usize, usize);
    fn consider_covariance(&self) -> &DMatrix<f64>;
    fn len(&self) -> u64;
    fn maximum_records(&self) -> u64;
    fn push(&mut self, step: &StoredStep) -> Result<(), StoreError>;
    fn get(&mut self, index: u64) -> Result<StoredStep, StoreError>;
    fn set(&mut self, index: u64, step: &StoredStep) -> Result<(), StoreError>;
    fn finish(&mut self) -> Result<(), StoreError>;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum StoreKind {
    Memory,
    SeekableTemporary,
}

pub(super) struct PlannedStore {
    pub store: Box<dyn StateStore>,
    pub kind: StoreKind,
    pub record_bytes: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct StateStoreResourceBounds {
    pub record_bytes: u64,
    pub memory_peak_bytes: u64,
    pub seekable_peak_bytes: u64,
    pub seekable_temporary_bytes: u64,
}

pub(super) fn state_store_resource_bounds(
    state_dimension: usize,
    consider_covariance: &DMatrix<f64>,
    maximum_records: u64,
) -> Result<StateStoreResourceBounds, ProcessError> {
    if maximum_records == 0 {
        return Err(ProcessError::ResourceLimit);
    }
    if consider_covariance.nrows() != consider_covariance.ncols()
        || !consider_covariance.iter().all(|value| value.is_finite())
    {
        return Err(ProcessError::InvalidEvidence);
    }
    let consider_dimension = consider_covariance.nrows();
    let payload_bytes = StoredStep::encoded_len(state_dimension, consider_dimension)
        .and_then(|value| u64::try_from(value).ok())
        .ok_or(ProcessError::ResourceLimit)?;
    let record_bytes = payload_bytes
        .checked_add(4)
        .ok_or(ProcessError::ResourceLimit)?;
    let record_total = record_bytes
        .checked_mul(maximum_records)
        .ok_or(ProcessError::ResourceLimit)?;
    let static_consider_bytes = u64::try_from(
        matrix_bytes(consider_dimension, consider_dimension).ok_or(ProcessError::ResourceLimit)?,
    )
    .map_err(|_| ProcessError::ResourceLimit)?;
    // Account for Vec/matrix metadata and allocator rounding in the direct
    // adapter.  It is selected only when the conservative complete bound fits.
    let memory_bound = record_total
        .checked_add(record_total / 4)
        .and_then(|value| value.checked_add(static_consider_bytes))
        .and_then(|value| value.checked_add(1 << 20))
        .ok_or(ProcessError::ResourceLimit)?;
    let file_total = record_total
        .checked_add(FILE_HEADER_BYTES)
        .ok_or(ProcessError::ResourceLimit)?;
    let seekable_peak_bytes = record_bytes
        .checked_mul(3)
        .and_then(|value| value.checked_add(static_consider_bytes))
        .ok_or(ProcessError::ResourceLimit)?;
    Ok(StateStoreResourceBounds {
        record_bytes,
        memory_peak_bytes: memory_bound,
        seekable_peak_bytes,
        seekable_temporary_bytes: file_total,
    })
}

pub(super) fn plan_store_kind(
    state_dimension: usize,
    consider_covariance: &DMatrix<f64>,
    maximum_records: u64,
    limits: OfflineResourceLimits,
    kind: StoreKind,
) -> Result<PlannedStore, ProcessError> {
    let bounds =
        state_store_resource_bounds(state_dimension, consider_covariance, maximum_records)?;
    match kind {
        StoreKind::Memory => {
            if bounds.memory_peak_bytes > limits.peak_memory_bytes {
                return Err(ProcessError::StorageExhausted);
            }
            let store = MemoryStore::new(state_dimension, consider_covariance, maximum_records)?;
            Ok(PlannedStore {
                store: Box::new(store),
                kind,
                record_bytes: bounds.record_bytes,
            })
        }
        StoreKind::SeekableTemporary => {
            if bounds.seekable_peak_bytes > limits.peak_memory_bytes
                || bounds.seekable_temporary_bytes > limits.temporary_storage_bytes
            {
                return Err(ProcessError::StorageExhausted);
            }
            let payload_bytes = bounds
                .record_bytes
                .checked_sub(4)
                .ok_or(ProcessError::ResourceLimit)?;
            let store = FileStore::new(
                state_dimension,
                consider_covariance,
                maximum_records,
                payload_bytes,
            )?;
            Ok(PlannedStore {
                store: Box::new(store),
                kind,
                record_bytes: bounds.record_bytes,
            })
        }
    }
}

#[cfg(test)]
pub(super) fn plan_store(
    state_dimension: usize,
    consider_covariance: &DMatrix<f64>,
    maximum_records: u64,
    limits: OfflineResourceLimits,
) -> Result<PlannedStore, ProcessError> {
    let bounds =
        state_store_resource_bounds(state_dimension, consider_covariance, maximum_records)?;
    if bounds.memory_peak_bytes <= limits.peak_memory_bytes {
        return plan_store_kind(
            state_dimension,
            consider_covariance,
            maximum_records,
            limits,
            StoreKind::Memory,
        );
    }
    plan_store_kind(
        state_dimension,
        consider_covariance,
        maximum_records,
        limits,
        StoreKind::SeekableTemporary,
    )
}

struct MemoryStore {
    state_dimension: usize,
    consider_dimension: usize,
    consider_covariance: DMatrix<f64>,
    maximum_records: u64,
    records: Vec<StoredStep>,
}

impl MemoryStore {
    fn new(
        state_dimension: usize,
        consider_covariance: &DMatrix<f64>,
        maximum_records: u64,
    ) -> Result<Self, ProcessError> {
        let consider_dimension = consider_covariance.nrows();
        let capacity = usize::try_from(maximum_records).map_err(|_| ProcessError::ResourceLimit)?;
        let mut records = Vec::new();
        records
            .try_reserve_exact(capacity)
            .map_err(|_| ProcessError::StorageExhausted)?;
        Ok(Self {
            state_dimension,
            consider_dimension,
            consider_covariance: consider_covariance.clone(),
            maximum_records,
            records,
        })
    }
}

impl StateStore for MemoryStore {
    fn dimensions(&self) -> (usize, usize) {
        (self.state_dimension, self.consider_dimension)
    }

    fn consider_covariance(&self) -> &DMatrix<f64> {
        &self.consider_covariance
    }

    fn len(&self) -> u64 {
        self.records.len() as u64
    }

    fn maximum_records(&self) -> u64 {
        self.maximum_records
    }

    fn push(&mut self, step: &StoredStep) -> Result<(), StoreError> {
        if self.len() >= self.maximum_records {
            return Err(StoreError::Exhausted);
        }
        step.validate_dimensions(self.state_dimension, self.consider_dimension)?;
        self.records.push(step.clone());
        Ok(())
    }

    fn get(&mut self, index: u64) -> Result<StoredStep, StoreError> {
        self.records
            .get(usize::try_from(index).map_err(|_| StoreError::OutOfRange)?)
            .cloned()
            .ok_or(StoreError::OutOfRange)
    }

    fn set(&mut self, index: u64, step: &StoredStep) -> Result<(), StoreError> {
        step.validate_dimensions(self.state_dimension, self.consider_dimension)?;
        let target = self
            .records
            .get_mut(usize::try_from(index).map_err(|_| StoreError::OutOfRange)?)
            .ok_or(StoreError::OutOfRange)?;
        *target = step.clone();
        Ok(())
    }

    fn finish(&mut self) -> Result<(), StoreError> {
        Ok(())
    }
}

struct FileStore {
    state_dimension: usize,
    consider_dimension: usize,
    consider_covariance: DMatrix<f64>,
    consider_checksum: u32,
    maximum_records: u64,
    payload_bytes: u64,
    count: u64,
    file: Option<File>,
    path: PathBuf,
}

impl FileStore {
    fn new(
        state_dimension: usize,
        consider_covariance: &DMatrix<f64>,
        maximum_records: u64,
        payload_bytes: u64,
    ) -> Result<Self, ProcessError> {
        let consider_dimension = consider_covariance.nrows();
        let consider_checksum = crc32c_matrix(consider_covariance);
        let (mut file, path) = create_temporary_file()?;
        let slot_bytes = payload_bytes
            .checked_add(4)
            .ok_or(ProcessError::ResourceLimit)?;
        let allocated = slot_bytes
            .checked_mul(maximum_records)
            .and_then(|value| value.checked_add(FILE_HEADER_BYTES))
            .ok_or(ProcessError::ResourceLimit)?;
        if file.set_len(allocated).is_err() {
            let _ = std::fs::remove_file(&path);
            return Err(ProcessError::StorageExhausted);
        }
        let header = encode_file_header(
            state_dimension,
            consider_dimension,
            payload_bytes,
            maximum_records,
            consider_checksum,
        )?;
        if file
            .seek(SeekFrom::Start(0))
            .and_then(|_| file.write_all(&header))
            .is_err()
        {
            let _ = std::fs::remove_file(&path);
            return Err(ProcessError::StorageExhausted);
        }
        Ok(Self {
            state_dimension,
            consider_dimension,
            consider_covariance: consider_covariance.clone(),
            consider_checksum,
            maximum_records,
            payload_bytes,
            count: 0,
            file: Some(file),
            path,
        })
    }

    fn slot_offset(&self, index: u64) -> Result<u64, StoreError> {
        if index >= self.maximum_records {
            return Err(StoreError::OutOfRange);
        }
        self.payload_bytes
            .checked_add(4)
            .and_then(|slot| slot.checked_mul(index))
            .and_then(|relative| relative.checked_add(FILE_HEADER_BYTES))
            .ok_or(StoreError::IntegerOverflow)
    }

    fn validate_header(&mut self) -> Result<(), StoreError> {
        let mut header = [0_u8; 64];
        let file = self.file.as_mut().ok_or(StoreError::Corrupt)?;
        file.seek(SeekFrom::Start(0))
            .and_then(|_| file.read_exact(&mut header))
            .map_err(|_| StoreError::ReadIo)?;
        let stored_checksum = decode_u32(&header[60..64])?;
        let state_dimension =
            u32::try_from(self.state_dimension).map_err(|_| StoreError::IntegerOverflow)?;
        let consider_dimension =
            u32::try_from(self.consider_dimension).map_err(|_| StoreError::IntegerOverflow)?;
        if header[..8] != FILE_MAGIC
            || decode_u32(&header[8..12])? != FILE_FORMAT_VERSION
            || decode_u32(&header[12..16])? != state_dimension
            || decode_u32(&header[16..20])? != consider_dimension
            || header[20..24].iter().any(|value| *value != 0)
            || decode_u64(&header[24..32])? != self.payload_bytes
            || decode_u64(&header[32..40])? != self.maximum_records
            || decode_u32(&header[40..44])? != self.consider_checksum
            || header[44..60].iter().any(|value| *value != 0)
            || stored_checksum != crc32c(&header[..60])
        {
            return Err(StoreError::Corrupt);
        }
        Ok(())
    }

    fn write_at(&mut self, index: u64, step: &StoredStep) -> Result<(), StoreError> {
        self.validate_header()?;
        let payload = step.encode(self.state_dimension, self.consider_dimension)?;
        if u64::try_from(payload.len()).map_err(|_| StoreError::IntegerOverflow)?
            != self.payload_bytes
        {
            return Err(StoreError::Dimension);
        }
        let checksum = crc32c(&payload).to_le_bytes();
        let offset = self.slot_offset(index)?;
        let file = self.file.as_mut().ok_or(StoreError::Corrupt)?;
        file.seek(SeekFrom::Start(offset))
            .and_then(|_| file.write_all(&payload))
            .and_then(|()| file.write_all(&checksum))
            .map_err(|_| StoreError::WriteIo)
    }
}

impl StateStore for FileStore {
    fn dimensions(&self) -> (usize, usize) {
        (self.state_dimension, self.consider_dimension)
    }

    fn consider_covariance(&self) -> &DMatrix<f64> {
        &self.consider_covariance
    }

    fn len(&self) -> u64 {
        self.count
    }

    fn maximum_records(&self) -> u64 {
        self.maximum_records
    }

    fn push(&mut self, step: &StoredStep) -> Result<(), StoreError> {
        if self.count >= self.maximum_records {
            return Err(StoreError::Exhausted);
        }
        self.write_at(self.count, step)?;
        self.count += 1;
        Ok(())
    }

    fn get(&mut self, index: u64) -> Result<StoredStep, StoreError> {
        if index >= self.count {
            return Err(StoreError::OutOfRange);
        }
        self.validate_header()?;
        let payload_len = usize::try_from(self.payload_bytes).map_err(|_| StoreError::Exhausted)?;
        let mut payload = Vec::new();
        payload
            .try_reserve_exact(payload_len)
            .map_err(|_| StoreError::Exhausted)?;
        payload.resize(payload_len, 0);
        let mut checksum = [0_u8; 4];
        let offset = self.slot_offset(index)?;
        let file = self.file.as_mut().ok_or(StoreError::Corrupt)?;
        file.seek(SeekFrom::Start(offset))
            .and_then(|_| file.read_exact(&mut payload))
            .and_then(|()| file.read_exact(&mut checksum))
            .map_err(|_| StoreError::ReadIo)?;
        if u32::from_le_bytes(checksum) != crc32c(&payload) {
            return Err(StoreError::Corrupt);
        }
        StoredStep::decode(&payload, self.state_dimension, self.consider_dimension)
    }

    fn set(&mut self, index: u64, step: &StoredStep) -> Result<(), StoreError> {
        if index >= self.count {
            return Err(StoreError::OutOfRange);
        }
        self.write_at(index, step)
    }

    fn finish(&mut self) -> Result<(), StoreError> {
        self.validate_header()?;
        let actual = self
            .payload_bytes
            .checked_add(4)
            .and_then(|slot| slot.checked_mul(self.count))
            .and_then(|relative| relative.checked_add(FILE_HEADER_BYTES))
            .ok_or(StoreError::IntegerOverflow)?;
        let file = self.file.as_mut().ok_or(StoreError::Corrupt)?;
        file.set_len(actual).map_err(|_| StoreError::WriteIo)?;
        file.sync_data().map_err(|_| StoreError::WriteIo)
    }
}

impl Drop for FileStore {
    fn drop(&mut self) {
        if let Some(file) = self.file.take() {
            let _ = file.sync_data();
            drop(file);
        }
        let _ = std::fs::remove_file(&self.path);
    }
}

pub(crate) const FIXED_RECORD_HEADER_BYTES: u64 = 64;
const FIXED_RECORD_FORMAT_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FixedRecordStoreKind {
    Memory,
    SeekableTemporary,
}

enum FixedRecordBacking {
    Memory(Vec<u8>),
    SeekableTemporary { file: Option<File>, path: PathBuf },
}

/// Fixed-width checksummed records used by the returned offline trajectory.
///
/// The resident adapter reserves its complete byte bound before the first
/// append. The seekable adapter retains no session-sized index: record offsets
/// are derived from the fixed payload width, and every read validates both the
/// sealed file header and the record CRC before exposing bytes to a decoder.
pub(crate) struct FixedRecordStore {
    kind: FixedRecordStoreKind,
    magic: [u8; 8],
    payload_bytes: u64,
    maximum_records: u64,
    count: u64,
    sealed: bool,
    header: [u8; FIXED_RECORD_HEADER_BYTES as usize],
    backing: FixedRecordBacking,
}

impl core::fmt::Debug for FixedRecordStore {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter
            .debug_struct("FixedRecordStore")
            .field("kind", &self.kind)
            .field("payload_bytes", &self.payload_bytes)
            .field("maximum_records", &self.maximum_records)
            .field("count", &self.count)
            .field("sealed", &self.sealed)
            .finish_non_exhaustive()
    }
}

impl FixedRecordStore {
    pub(crate) fn new(
        kind: FixedRecordStoreKind,
        magic: [u8; 8],
        payload_bytes: u64,
        maximum_records: u64,
    ) -> Result<Self, ProcessError> {
        if payload_bytes == 0 || maximum_records == 0 || magic == [0; 8] {
            return Err(ProcessError::ResourceLimit);
        }
        let slot_bytes = payload_bytes
            .checked_add(4)
            .ok_or(ProcessError::ResourceLimit)?;
        let record_bytes = slot_bytes
            .checked_mul(maximum_records)
            .ok_or(ProcessError::ResourceLimit)?;
        let header = encode_fixed_record_header(magic, payload_bytes, maximum_records, 0, false);
        let backing = match kind {
            FixedRecordStoreKind::Memory => {
                let capacity =
                    usize::try_from(record_bytes).map_err(|_| ProcessError::ResourceLimit)?;
                let mut bytes = Vec::new();
                bytes
                    .try_reserve_exact(capacity)
                    .map_err(|_| ProcessError::StorageExhausted)?;
                if bytes.capacity() != capacity {
                    return Err(ProcessError::StorageExhausted);
                }
                FixedRecordBacking::Memory(bytes)
            }
            FixedRecordStoreKind::SeekableTemporary => {
                let (mut file, path) = create_temporary_file()?;
                let allocated = record_bytes
                    .checked_add(FIXED_RECORD_HEADER_BYTES)
                    .ok_or(ProcessError::ResourceLimit)?;
                if file
                    .set_len(allocated)
                    .and_then(|()| file.seek(SeekFrom::Start(0)).map(|_| ()))
                    .and_then(|()| file.write_all(&header))
                    .is_err()
                {
                    drop(file);
                    let _ = std::fs::remove_file(&path);
                    return Err(ProcessError::StorageExhausted);
                }
                FixedRecordBacking::SeekableTemporary {
                    file: Some(file),
                    path,
                }
            }
        };
        Ok(Self {
            kind,
            magic,
            payload_bytes,
            maximum_records,
            count: 0,
            sealed: false,
            header,
            backing,
        })
    }

    pub(crate) fn push(&mut self, payload: &[u8]) -> Result<(), StoreError> {
        if self.sealed || self.count >= self.maximum_records {
            return Err(StoreError::Exhausted);
        }
        if u64::try_from(payload.len()).map_err(|_| StoreError::IntegerOverflow)?
            != self.payload_bytes
        {
            return Err(StoreError::Dimension);
        }
        let checksum = crc32c(payload).to_le_bytes();
        let offset = self.slot_offset(self.count)?;
        match &mut self.backing {
            FixedRecordBacking::Memory(bytes) => {
                let expected = usize::try_from(
                    self.count
                        .checked_mul(self.payload_bytes + 4)
                        .ok_or(StoreError::IntegerOverflow)?,
                )
                .map_err(|_| StoreError::IntegerOverflow)?;
                if bytes.len() != expected {
                    return Err(StoreError::Corrupt);
                }
                bytes.extend_from_slice(payload);
                bytes.extend_from_slice(&checksum);
            }
            FixedRecordBacking::SeekableTemporary { file, .. } => {
                let file = file.as_mut().ok_or(StoreError::Corrupt)?;
                file.seek(SeekFrom::Start(offset))
                    .and_then(|_| file.write_all(payload))
                    .and_then(|()| file.write_all(&checksum))
                    .map_err(|_| StoreError::WriteIo)?;
            }
        }
        self.count += 1;
        Ok(())
    }

    pub(crate) fn finish(&mut self) -> Result<(), StoreError> {
        if self.sealed {
            return self.validate_header();
        }
        self.header = encode_fixed_record_header(
            self.magic,
            self.payload_bytes,
            self.maximum_records,
            self.count,
            true,
        );
        if let FixedRecordBacking::SeekableTemporary { file, .. } = &mut self.backing {
            let actual = self
                .count
                .checked_mul(self.payload_bytes + 4)
                .and_then(|bytes| bytes.checked_add(FIXED_RECORD_HEADER_BYTES))
                .ok_or(StoreError::IntegerOverflow)?;
            let file = file.as_mut().ok_or(StoreError::Corrupt)?;
            file.seek(SeekFrom::Start(0))
                .and_then(|_| file.write_all(&self.header))
                .and_then(|()| file.set_len(actual))
                .and_then(|()| file.sync_data())
                .map_err(|_| StoreError::WriteIo)?;
        }
        self.sealed = true;
        Ok(())
    }

    pub(crate) fn read_into(&mut self, index: u64, target: &mut [u8]) -> Result<(), StoreError> {
        if !self.sealed || index >= self.count {
            return Err(StoreError::OutOfRange);
        }
        if u64::try_from(target.len()).map_err(|_| StoreError::IntegerOverflow)?
            != self.payload_bytes
        {
            return Err(StoreError::Dimension);
        }
        self.validate_header()?;
        let relative = index
            .checked_mul(self.payload_bytes + 4)
            .ok_or(StoreError::IntegerOverflow)?;
        let mut checksum = [0_u8; 4];
        match &mut self.backing {
            FixedRecordBacking::Memory(bytes) => {
                let start = usize::try_from(relative).map_err(|_| StoreError::IntegerOverflow)?;
                let payload_end = start
                    .checked_add(target.len())
                    .ok_or(StoreError::IntegerOverflow)?;
                let checksum_end = payload_end
                    .checked_add(4)
                    .ok_or(StoreError::IntegerOverflow)?;
                target.copy_from_slice(bytes.get(start..payload_end).ok_or(StoreError::Corrupt)?);
                checksum.copy_from_slice(
                    bytes
                        .get(payload_end..checksum_end)
                        .ok_or(StoreError::Corrupt)?,
                );
            }
            FixedRecordBacking::SeekableTemporary { file, .. } => {
                let offset = relative
                    .checked_add(FIXED_RECORD_HEADER_BYTES)
                    .ok_or(StoreError::IntegerOverflow)?;
                let file = file.as_mut().ok_or(StoreError::Corrupt)?;
                file.seek(SeekFrom::Start(offset))
                    .and_then(|_| file.read_exact(target))
                    .and_then(|()| file.read_exact(&mut checksum))
                    .map_err(|_| StoreError::ReadIo)?;
            }
        }
        if u32::from_le_bytes(checksum) != crc32c(target) {
            return Err(StoreError::Corrupt);
        }
        Ok(())
    }

    fn slot_offset(&self, index: u64) -> Result<u64, StoreError> {
        index
            .checked_mul(self.payload_bytes + 4)
            .and_then(|bytes| bytes.checked_add(FIXED_RECORD_HEADER_BYTES))
            .ok_or(StoreError::IntegerOverflow)
    }

    fn validate_header(&mut self) -> Result<(), StoreError> {
        let mut header = self.header;
        if let FixedRecordBacking::SeekableTemporary { file, .. } = &mut self.backing {
            let file = file.as_mut().ok_or(StoreError::Corrupt)?;
            file.seek(SeekFrom::Start(0))
                .and_then(|_| file.read_exact(&mut header))
                .map_err(|_| StoreError::ReadIo)?;
        }
        if header
            != encode_fixed_record_header(
                self.magic,
                self.payload_bytes,
                self.maximum_records,
                self.count,
                true,
            )
        {
            return Err(StoreError::Corrupt);
        }
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn path_for_test(&self) -> Option<PathBuf> {
        match &self.backing {
            FixedRecordBacking::Memory(_) => None,
            FixedRecordBacking::SeekableTemporary { path, .. } => Some(path.clone()),
        }
    }

    #[cfg(test)]
    pub(crate) fn allocated_record_bytes_for_test(&self) -> u64 {
        match &self.backing {
            FixedRecordBacking::Memory(bytes) => bytes.capacity() as u64,
            FixedRecordBacking::SeekableTemporary { .. } => 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn corrupt_record_byte_for_test(
        &mut self,
        index: u64,
        byte: u64,
    ) -> Result<(), StoreError> {
        if index >= self.count || byte >= self.payload_bytes {
            return Err(StoreError::OutOfRange);
        }
        let relative = index
            .checked_mul(self.payload_bytes + 4)
            .and_then(|offset| offset.checked_add(byte))
            .ok_or(StoreError::IntegerOverflow)?;
        match &mut self.backing {
            FixedRecordBacking::Memory(bytes) => {
                let index = usize::try_from(relative).map_err(|_| StoreError::IntegerOverflow)?;
                let target = bytes.get_mut(index).ok_or(StoreError::Corrupt)?;
                *target ^= 0x80;
            }
            FixedRecordBacking::SeekableTemporary { file, .. } => {
                let offset = relative
                    .checked_add(FIXED_RECORD_HEADER_BYTES)
                    .ok_or(StoreError::IntegerOverflow)?;
                let file = file.as_mut().ok_or(StoreError::Corrupt)?;
                let mut value = [0_u8; 1];
                file.seek(SeekFrom::Start(offset))
                    .and_then(|_| file.read_exact(&mut value))
                    .map_err(|_| StoreError::ReadIo)?;
                value[0] ^= 0x80;
                file.seek(SeekFrom::Start(offset))
                    .and_then(|_| file.write_all(&value))
                    .map_err(|_| StoreError::WriteIo)?;
            }
        }
        Ok(())
    }
}

impl Drop for FixedRecordStore {
    fn drop(&mut self) {
        if let FixedRecordBacking::SeekableTemporary { file, path } = &mut self.backing {
            if let Some(file) = file.take() {
                let _ = file.sync_data();
                drop(file);
            }
            let _ = std::fs::remove_file(path);
        }
    }
}

fn encode_fixed_record_header(
    magic: [u8; 8],
    payload_bytes: u64,
    maximum_records: u64,
    count: u64,
    sealed: bool,
) -> [u8; FIXED_RECORD_HEADER_BYTES as usize] {
    let mut header = [0_u8; FIXED_RECORD_HEADER_BYTES as usize];
    header[..8].copy_from_slice(&magic);
    header[8..12].copy_from_slice(&FIXED_RECORD_FORMAT_VERSION.to_le_bytes());
    header[12] = u8::from(sealed);
    header[16..24].copy_from_slice(&payload_bytes.to_le_bytes());
    header[24..32].copy_from_slice(&maximum_records.to_le_bytes());
    header[32..40].copy_from_slice(&count.to_le_bytes());
    let checksum = crc32c(&header[..60]);
    header[60..64].copy_from_slice(&checksum.to_le_bytes());
    header
}

fn create_temporary_file() -> Result<(File, PathBuf), ProcessError> {
    create_temporary_file_in(&std::env::temp_dir())
}

fn create_temporary_file_in(directory: &Path) -> Result<(File, PathBuf), ProcessError> {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let process = u64::from(std::process::id());
    for _ in 0..MAX_TEMPFILE_ATTEMPTS {
        let counter = COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = directory.join(format!("aevia-traj-{process:08x}-{counter:016x}.state"));
        match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&path)
        {
            Ok(file) => return Ok((file, path)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(_) => return Err(ProcessError::StorageExhausted),
        }
    }
    Err(ProcessError::StorageExhausted)
}

fn encode_file_header(
    state_dimension: usize,
    consider_dimension: usize,
    payload_bytes: u64,
    maximum_records: u64,
    consider_checksum: u32,
) -> Result<[u8; 64], ProcessError> {
    let state_dimension =
        u32::try_from(state_dimension).map_err(|_| ProcessError::ResourceLimit)?;
    let consider_dimension =
        u32::try_from(consider_dimension).map_err(|_| ProcessError::ResourceLimit)?;
    let mut header = [0_u8; 64];
    header[..8].copy_from_slice(&FILE_MAGIC);
    header[8..12].copy_from_slice(&FILE_FORMAT_VERSION.to_le_bytes());
    header[12..16].copy_from_slice(&state_dimension.to_le_bytes());
    header[16..20].copy_from_slice(&consider_dimension.to_le_bytes());
    header[24..32].copy_from_slice(&payload_bytes.to_le_bytes());
    header[32..40].copy_from_slice(&maximum_records.to_le_bytes());
    header[40..44].copy_from_slice(&consider_checksum.to_le_bytes());
    let checksum = crc32c(&header[..60]);
    header[60..64].copy_from_slice(&checksum.to_le_bytes());
    Ok(header)
}

fn matrix_bytes(rows: usize, columns: usize) -> Option<usize> {
    rows.checked_mul(columns)?.checked_mul(8)
}

fn decode_u32(bytes: &[u8]) -> Result<u32, StoreError> {
    let source = bytes.get(..4).ok_or(StoreError::Corrupt)?;
    let mut value = [0_u8; 4];
    value.copy_from_slice(source);
    Ok(u32::from_le_bytes(value))
}

fn decode_u64(bytes: &[u8]) -> Result<u64, StoreError> {
    let source = bytes.get(..8).ok_or(StoreError::Corrupt)?;
    let mut value = [0_u8; 8];
    value.copy_from_slice(source);
    Ok(u64::from_le_bytes(value))
}

fn put_i64(bytes: &mut Vec<u8>, value: i64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn put_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

fn put_matrix(bytes: &mut Vec<u8>, matrix: &DMatrix<f64>) {
    for value in matrix.iter() {
        put_f64(bytes, *value);
    }
}

struct DecodeCursor<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl DecodeCursor<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N], StoreError> {
        let end = self
            .position
            .checked_add(N)
            .ok_or(StoreError::IntegerOverflow)?;
        let source = self
            .bytes
            .get(self.position..end)
            .ok_or(StoreError::Corrupt)?;
        let mut result = [0_u8; N];
        result.copy_from_slice(source);
        self.position = end;
        Ok(result)
    }

    fn u8(&mut self) -> Result<u8, StoreError> {
        Ok(self.take::<1>()?[0])
    }

    fn boolean(&mut self) -> Result<bool, StoreError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(StoreError::Corrupt),
        }
    }

    fn i64(&mut self) -> Result<i64, StoreError> {
        Ok(i64::from_le_bytes(self.take()?))
    }

    fn f64(&mut self) -> Result<f64, StoreError> {
        let value = f64::from_bits(u64::from_le_bytes(self.take()?));
        if value.is_finite() {
            Ok(value)
        } else {
            Err(StoreError::Corrupt)
        }
    }

    fn array3(&mut self) -> Result<[f64; 3], StoreError> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }

    fn array4(&mut self) -> Result<[f64; 4], StoreError> {
        Ok([self.f64()?, self.f64()?, self.f64()?, self.f64()?])
    }

    fn matrix(&mut self, rows: usize, columns: usize) -> Result<DMatrix<f64>, StoreError> {
        let length = rows
            .checked_mul(columns)
            .ok_or(StoreError::IntegerOverflow)?;
        let mut values = Vec::new();
        values
            .try_reserve_exact(length)
            .map_err(|_| StoreError::Exhausted)?;
        for _ in 0..length {
            values.push(self.f64()?);
        }
        Ok(DMatrix::from_vec(rows, columns, values))
    }
}

fn encode_disposition(value: Option<InputDisposition>) -> u8 {
    match value {
        None => 0,
        Some(InputDisposition::Fused) => 1,
        Some(InputDisposition::StatisticallyRejected) => 2,
        Some(InputDisposition::Downweighted) => 3,
        Some(InputDisposition::TooLateForLive) => 4,
        Some(InputDisposition::InitializationOnly) => 5,
        Some(InputDisposition::RetainedForOffline) => 6,
        Some(InputDisposition::QueuedForFusion) => 7,
    }
}

fn decode_disposition(value: u8) -> Result<Option<InputDisposition>, StoreError> {
    Ok(match value {
        0 => None,
        1 => Some(InputDisposition::Fused),
        2 => Some(InputDisposition::StatisticallyRejected),
        3 => Some(InputDisposition::Downweighted),
        4 => Some(InputDisposition::TooLateForLive),
        5 => Some(InputDisposition::InitializationOnly),
        6 => Some(InputDisposition::RetainedForOffline),
        7 => Some(InputDisposition::QueuedForFusion),
        _ => return Err(StoreError::Corrupt),
    })
}

fn encode_gnss(value: GnssState) -> u8 {
    match value {
        GnssState::Fixed => 0,
        GnssState::Float => 1,
        GnssState::Standalone => 2,
        GnssState::Dgps => 3,
        GnssState::Ppp => 4,
        GnssState::Absent => 5,
        GnssState::Suspect => 6,
    }
}

fn decode_gnss(value: u8) -> Result<GnssState, StoreError> {
    Ok(match value {
        0 => GnssState::Fixed,
        1 => GnssState::Float,
        2 => GnssState::Standalone,
        3 => GnssState::Dgps,
        4 => GnssState::Ppp,
        5 => GnssState::Absent,
        6 => GnssState::Suspect,
        _ => return Err(StoreError::Corrupt),
    })
}

fn encode_timing(value: TimingQuality) -> u8 {
    match value {
        TimingQuality::PpsCorrelated => 0,
        TimingQuality::Modeled => 1,
        TimingQuality::ArrivalOnly => 2,
        TimingQuality::Discontinuous => 3,
    }
}

fn decode_timing(value: u8) -> Result<TimingQuality, StoreError> {
    Ok(match value {
        0 => TimingQuality::PpsCorrelated,
        1 => TimingQuality::Modeled,
        2 => TimingQuality::ArrivalOnly,
        3 => TimingQuality::Discontinuous,
        _ => return Err(StoreError::Corrupt),
    })
}

fn crc32c(bytes: &[u8]) -> u32 {
    let mut crc = !0_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0x82f6_3b78 & mask);
        }
    }
    !crc
}

fn crc32c_matrix(matrix: &DMatrix<f64>) -> u32 {
    let mut crc = !0_u32;
    for value in matrix.iter() {
        for byte in value.to_bits().to_le_bytes() {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                let mask = 0_u32.wrapping_sub(crc & 1);
                crc = (crc >> 1) ^ (0x82f6_3b78 & mask);
            }
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal(time: i64) -> StoredNominal {
        StoredNominal {
            time: SessionTime::from_ns(time),
            position_ecef: [6_378_137.0, 0.0, 0.0],
            velocity_ecef: [1.0, 2.0, 3.0],
            orientation_ecef_from_body: UnitQuaternion::IDENTITY,
            accelerometer_bias_body: [0.01, 0.02, 0.03],
            gyroscope_bias_body: [0.001, 0.002, 0.003],
            colored_gnss_error: [0.1, 0.2, 0.3],
            specific_force_body: [0.0, 0.0, 9.8],
            angular_rate_body: [0.0, 0.0, 0.1],
        }
    }

    fn covariance(n: usize, m: usize) -> StoredCovariance {
        StoredCovariance {
            state: DMatrix::identity(n, n),
            state_consider: DMatrix::zeros(n, m),
        }
    }

    fn step(n: usize, m: usize) -> StoredStep {
        StoredStep {
            connected_from_previous: true,
            predicted: nominal(10),
            filtered: nominal(10),
            smoothed: Some(nominal(10)),
            predicted_covariance: covariance(n, m),
            filtered_covariance: covariance(n, m),
            smoothed_covariance: Some(covariance(n, m)),
            transition: DMatrix::identity(n, n),
            consider_transition: DMatrix::zeros(n, m),
            process_covariance: DMatrix::identity(n, n) * 0.01,
            integration_imu: Some(StoredIntegrationImu {
                start: SessionTime::ZERO,
                end: SessionTime::from_ns(10),
                angular_rate_body: [0.1, 0.2, 0.3],
                specific_force_body: [1.0, 2.0, 3.0],
            }),
            reset_basis: DMatrix::identity(n, n),
            smoothed_backward_gain: Some(DMatrix::identity(n + m, n + m) * 0.25),
            adjacent_cross_covariance: DMatrix::identity(n, n) * 0.5,
            disposition: Some(InputDisposition::Fused),
            gnss_state: GnssState::Fixed,
            timing_quality: TimingQuality::PpsCorrelated,
            degraded_input: false,
            objective_contribution: 3.5,
        }
    }

    #[test]
    fn fixed_record_round_trip_preserves_every_smoother_block() {
        let original = step(18, 5);
        let encoded = original.encode(18, 5).unwrap();
        assert_eq!(encoded.len(), StoredStep::encoded_len(18, 5).unwrap());
        let decoded = StoredStep::decode(&encoded, 18, 5).unwrap();
        assert_eq!(
            decoded.filtered.position_ecef,
            original.filtered.position_ecef
        );
        assert_eq!(decoded.transition, original.transition);
        assert_eq!(decoded.consider_transition, original.consider_transition);
        assert_eq!(decoded.integration_imu, original.integration_imu);
        assert_eq!(
            decoded.smoothed_backward_gain,
            original.smoothed_backward_gain
        );
        assert_eq!(
            decoded.adjacent_cross_covariance,
            original.adjacent_cross_covariance
        );
        assert_eq!(decoded.objective_contribution, 3.5);
    }

    #[test]
    fn memory_store_supports_reverse_random_access_and_replacement() {
        let consider = DMatrix::identity(2, 2);
        let mut store = MemoryStore::new(15, &consider, 3).unwrap();
        let mut first = step(15, 2);
        first.filtered.time = SessionTime::from_ns(1);
        first.integration_imu.as_mut().unwrap().end = SessionTime::from_ns(1);
        let mut second = step(15, 2);
        second.filtered.time = SessionTime::from_ns(2);
        second.integration_imu.as_mut().unwrap().end = SessionTime::from_ns(2);
        store.push(&first).unwrap();
        store.push(&second).unwrap();
        assert_eq!(store.get(1).unwrap().filtered.time, SessionTime::from_ns(2));
        second.objective_contribution = 9.0;
        store.set(1, &second).unwrap();
        assert_eq!(store.get(1).unwrap().objective_contribution, 9.0);
        assert_eq!(store.get(0).unwrap().filtered.time, SessionTime::from_ns(1));
    }

    #[test]
    fn seekable_store_checksums_records_and_cleans_up() {
        let (path, mut store) = {
            let consider = DMatrix::identity(2, 2);
            let store = FileStore::new(
                15,
                &consider,
                2,
                StoredStep::encoded_len(15, 2).unwrap() as u64,
            )
            .unwrap();
            (store.path.clone(), store)
        };
        store.push(&step(15, 2)).unwrap();
        assert_eq!(store.get(0).unwrap().gnss_state, GnssState::Fixed);
        store.finish().unwrap();
        drop(store);
        assert!(!path.exists());
    }

    #[test]
    fn temporary_backing_open_failure_is_storage_exhaustion_not_source_failure() {
        let (file, regular_file_path) = create_temporary_file().unwrap();
        assert_eq!(
            create_temporary_file_in(&regular_file_path).unwrap_err(),
            ProcessError::StorageExhausted
        );
        drop(file);
        std::fs::remove_file(regular_file_path).unwrap();
    }

    #[test]
    fn seekable_store_preflight_includes_the_file_header() {
        let consider = DMatrix::identity(2, 2);
        let maximum_records = 100_u64;
        let record_bytes = u64::try_from(StoredStep::encoded_len(15, 2).unwrap()).unwrap() + 4;
        let record_total = record_bytes.checked_mul(maximum_records).unwrap();
        let cache_bytes = record_bytes * 3 + 2 * 2 * 8;
        let limits = OfflineResourceLimits {
            peak_memory_bytes: cache_bytes,
            temporary_storage_bytes: record_total,
            output_bytes: 1,
            worker_count: 1,
            elapsed_work_limit: None,
        };
        assert!(matches!(
            plan_store(15, &consider, maximum_records, limits),
            Err(ProcessError::StorageExhausted)
        ));

        let fitting = OfflineResourceLimits {
            temporary_storage_bytes: record_total + FILE_HEADER_BYTES,
            ..limits
        };
        let planned = plan_store(15, &consider, maximum_records, fitting).unwrap();
        assert_eq!(planned.kind, StoreKind::SeekableTemporary);
    }

    #[test]
    fn forced_store_kind_obeys_the_complete_preflight_bound() {
        let consider = DMatrix::identity(2, 2);
        let maximum_records = 100_u64;
        let bounds = state_store_resource_bounds(15, &consider, maximum_records).unwrap();
        let memory_short = OfflineResourceLimits {
            peak_memory_bytes: bounds.memory_peak_bytes - 1,
            temporary_storage_bytes: u64::MAX,
            output_bytes: 1,
            worker_count: 1,
            elapsed_work_limit: None,
        };
        assert!(matches!(
            plan_store_kind(
                15,
                &consider,
                maximum_records,
                memory_short,
                StoreKind::Memory,
            ),
            Err(ProcessError::StorageExhausted)
        ));

        let seekable_short = OfflineResourceLimits {
            peak_memory_bytes: bounds.seekable_peak_bytes,
            temporary_storage_bytes: bounds.seekable_temporary_bytes - 1,
            ..memory_short
        };
        assert!(matches!(
            plan_store_kind(
                15,
                &consider,
                maximum_records,
                seekable_short,
                StoreKind::SeekableTemporary,
            ),
            Err(ProcessError::StorageExhausted)
        ));
    }

    #[test]
    fn seekable_store_rejects_corrupted_record_before_decode() {
        let consider = DMatrix::identity(2, 2);
        let mut store = FileStore::new(
            15,
            &consider,
            1,
            StoredStep::encoded_len(15, 2).unwrap() as u64,
        )
        .unwrap();
        store.push(&step(15, 2)).unwrap();
        let corrupt_at = FILE_HEADER_BYTES + 17;
        store
            .file
            .as_mut()
            .unwrap()
            .seek(SeekFrom::Start(corrupt_at))
            .unwrap();
        let mut byte = [0_u8; 1];
        store.file.as_mut().unwrap().read_exact(&mut byte).unwrap();
        byte[0] ^= 0x80;
        store
            .file
            .as_mut()
            .unwrap()
            .seek(SeekFrom::Start(corrupt_at))
            .unwrap();
        store.file.as_mut().unwrap().write_all(&byte).unwrap();
        assert!(matches!(store.get(0), Err(StoreError::Corrupt)));
    }

    #[test]
    fn seekable_store_rejects_corrupted_header_before_access() {
        let consider = DMatrix::identity(2, 2);
        let mut store = FileStore::new(
            15,
            &consider,
            1,
            StoredStep::encoded_len(15, 2).unwrap() as u64,
        )
        .unwrap();
        store.push(&step(15, 2)).unwrap();
        let file = store.file.as_mut().unwrap();
        file.seek(SeekFrom::Start(12)).unwrap();
        file.write_all(&[0xff]).unwrap();
        assert!(matches!(store.get(0), Err(StoreError::Corrupt)));
    }

    #[test]
    fn crc_detects_any_payload_change() {
        let original = step(15, 2).encode(15, 2).unwrap();
        let expected = crc32c(&original);
        for index in [0, original.len() / 2, original.len() - 1] {
            let mut corrupt = original.clone();
            corrupt[index] ^= 1;
            assert_ne!(crc32c(&corrupt), expected);
        }
    }

    #[test]
    fn fixed_record_backing_is_seekable_checksummed_and_cleans_up() {
        let payload_bytes = 32_u64;
        let mut store = FixedRecordStore::new(
            FixedRecordStoreKind::SeekableTemporary,
            *b"AEVTR01\0",
            payload_bytes,
            2,
        )
        .unwrap();
        let path = store.path_for_test().unwrap();
        let first = [0x12_u8; 32];
        let second = [0x34_u8; 32];
        store.push(&first).unwrap();
        store.push(&second).unwrap();
        store.finish().unwrap();

        let mut decoded = [0_u8; 32];
        store.read_into(1, &mut decoded).unwrap();
        assert_eq!(decoded, second);
        store.corrupt_record_byte_for_test(1, 7).unwrap();
        assert!(matches!(
            store.read_into(1, &mut decoded),
            Err(StoreError::Corrupt)
        ));
        drop(store);
        assert!(!path.exists());
    }

    #[test]
    fn fixed_memory_record_backing_has_an_exact_preflighted_capacity() {
        let mut store =
            FixedRecordStore::new(FixedRecordStoreKind::Memory, *b"AEVTR01\0", 7, 3).unwrap();
        assert_eq!(store.allocated_record_bytes_for_test(), 3 * (7 + 4));
        store.push(&[1; 7]).unwrap();
        store.push(&[2; 7]).unwrap();
        store.finish().unwrap();
        let mut decoded = [0; 7];
        store.read_into(0, &mut decoded).unwrap();
        assert_eq!(decoded, [1; 7]);
    }
}
