//! CPU1 owns SD, filesystem state and all blocking reads/writes. Acquisition
//! reserves and publishes queue slots under a short lock; PSRAM copies run
//! outside that lock so a cache miss cannot mask acquisition interrupts.
use super::{
    api::*,
    control::Mailbox,
    filesystem::{self, FileSystem, RecordingFile},
    format::{self, PageBuilder, Record},
    queue::{Queue, ReadState, STORAGE_BYTES},
};
use crate::device::{
    board::RecordingSlot,
    interface::{self, Phase, Snapshot},
    sensors, supervision, timing,
};
use aevia_device_hal::sdcard::{self, DmaScratch, Esp32SdHost, SdCard};
use core::{
    cell::{Cell, RefCell},
    mem::{MaybeUninit, size_of},
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;
use esp_hal::{delay::Delay, time::Duration};
use esp_rtos::CurrentThreadHandle;

type Card = SdCard<Esp32SdHost<'static, 'static, 0>, Delay>;
static SCRATCH: static_cell::StaticCell<DmaScratch> = static_cell::StaticCell::new();
static QUEUE: Mutex<CriticalSectionRawMutex, RefCell<Option<Queue<'static>>>> =
    Mutex::new(RefCell::new(None));
static CONTROL: Mutex<CriticalSectionRawMutex, RefCell<Mailbox>> =
    Mutex::new(RefCell::new(Mailbox::new()));
static STATUS: Mutex<CriticalSectionRawMutex, Cell<Option<Status>>> = Mutex::new(Cell::new(None));
static SAVED_IMU: AtomicU32 = AtomicU32::new(0);
const QUEUE_BYTES: usize = STORAGE_BYTES;
const FILE_SECTORS: u32 = 2 * 1024 * 1024 * 1024 / 512;
const FLUSH_INTERVAL_US: u64 = 250_000;
const PRODUCER_DRAIN_TIMEOUT_US: u64 = 2_000_000;

pub(crate) fn capture(kind: u8, at_ns: u64, payload: &[u8]) -> bool {
    let Some(record) = Record::new(kind, 0, at_ns, payload) else {
        return false;
    };
    let reservation = QUEUE.lock(|q| q.borrow_mut().as_mut().and_then(Queue::reserve_write));
    let Some(reservation) = reservation else {
        return false;
    };
    let written = reservation.write(record);
    QUEUE.lock(|q| q.borrow_mut().as_mut().unwrap().publish(written));
    true
}
fn pop() -> Option<Record> {
    let reservation = QUEUE.lock(|q| {
        q.borrow_mut()
            .as_mut()
            .map_or(ReadState::Empty, Queue::reserve_read)
    });
    let ReadState::Ready(reservation) = reservation else {
        return None;
    };
    let (record, complete) = reservation.read();
    QUEUE.lock(|q| q.borrow_mut().as_mut().unwrap().release(complete));
    Some(record)
}
fn enable(value: bool) {
    QUEUE.lock(|q| {
        if let Some(q) = q.borrow_mut().as_mut() {
            q.enabled = value;
        }
    });
}
fn queue_status() -> (u32, u64, u32, u64) {
    QUEUE.lock(|q| {
        q.borrow().as_ref().map_or((0, 0, 0, 0), |q| {
            (q.len() as u32, q.accepted, q.dropped, q.next_sequence)
        })
    })
}
fn current_status() -> Status {
    let mut status = STATUS.lock(Cell::get).unwrap_or_default();
    let (queued, accepted, dropped, _) = queue_status();
    status.queued_records = queued;
    status.accepted_records = accepted;
    status.dropped_records = dropped;
    status
}
pub(crate) fn command(request: Request) -> Response {
    let status = current_status();
    CONTROL.lock(|m| m.borrow_mut().request(request, status))
}
pub(crate) fn recording_active() -> bool {
    matches!(current_status().state, State::Recording | State::Stopping)
}
pub(crate) fn confirmation_ready() -> bool {
    let status = current_status();
    status.state == State::Recording
        && status.dropped_records == 0
        && status.io_errors == 0
        && SAVED_IMU.load(Ordering::Relaxed) >= 100
}
fn progress() {
    supervision::worker_heartbeat();
}
fn fs_error<E>(error: filesystem::Error<E>) -> Error {
    match error {
        filesystem::Error::Storage(_) => Error::Io,
        filesystem::Error::Unsupported => Error::UnsupportedFilesystem,
        filesystem::Error::ReadOnly => Error::ReadOnly,
        filesystem::Error::NoSpace | filesystem::Error::DirectoryFull => Error::NoSpace,
        filesystem::Error::Bounds => Error::OutOfRange,
        _ => Error::Corrupt,
    }
}

struct Active {
    file: RecordingFile,
    nonce: u64,
    page_index: u32,
    page: PageBuilder,
    sequence_base: u64,
    prefix: u64,
    last_source_sequence: Option<u64>,
}
impl Active {
    fn session(&self, closed: bool) -> Session {
        Session {
            session: self.file.id(),
            file_bytes: self.file.capacity_bytes(),
            valid_bytes: u64::from(self.page_index) * 512,
            closed,
        }
    }
}
struct Worker {
    card: Card,
    fs: Option<FileSystem>,
    active: Option<Active>,
    status: Status,
    next_file: u32,
    recording_id: u32,
    previous_file: u32,
    last_closed: Option<Session>,
    read_cache: Option<(RecordingFile, Session)>,
    sector_cache: Option<(u32, u32, [u8; 512])>,
    last_flush: u64,
    last_health: u64,
    last_snapshot: u64,
    previous_imu: u32,
    pending_imu: u32,
    snapshot: Snapshot,
}
impl Worker {
    fn publish(&mut self) {
        self.status.active_session = self.active.as_ref().map(|a| a.session(false));
        STATUS.lock(|s| s.set(Some(self.status)));
    }
    fn fault(&mut self, error: Error) {
        enable(false);
        self.status.state = State::Fault;
        self.status.last_error = Some(error);
        self.status.io_errors = self.status.io_errors.saturating_add(1);
        self.publish();
    }
    fn mount(&mut self) -> Result<(), Error> {
        let info = self
            .card
            .initialize(sdcard::Config::default())
            .map_err(|_| Error::NoCard)?;
        self.status.capacity_bytes = info.sector_count.saturating_mul(512);
        let fs = FileSystem::mount(&mut self.card, &mut progress).map_err(fs_error)?;
        self.status.filesystem = Filesystem::Exfat;
        let mut max_id = 0;
        fs.visit(&mut self.card, &mut progress, &mut |file| {
            max_id = max_id.max(file.id())
        })
        .map_err(fs_error)?;
        self.next_file = max_id.checked_add(1).ok_or(Error::NoSpace)?;
        self.fs = Some(fs);
        self.status.state = State::Ready;
        self.publish();
        Ok(())
    }
    fn open_file(&mut self, sequence_base: u64) -> Result<(), Error> {
        let fs = self.fs.as_mut().ok_or(Error::NotReady)?;
        let file = fs
            .create(&mut self.card, self.next_file, FILE_SECTORS, &mut progress)
            .map_err(fs_error)?;
        self.next_file = self.next_file.checked_add(1).ok_or(Error::NoSpace)?;
        if file.sector_count() < 8 {
            return Err(Error::NoSpace);
        }
        let rng = esp_hal::rng::Rng::new();
        let nonce = (u64::from(rng.random()) << 32) | u64::from(rng.random());
        self.active = Some(Active {
            file,
            nonce,
            page_index: 0,
            page: PageBuilder::new(),
            sequence_base,
            prefix: 0,
            last_source_sequence: None,
        });
        let mut identity = [0; 21];
        identity[0] = 1;
        identity[1..5].copy_from_slice(&self.recording_id.to_le_bytes());
        identity[5..9].copy_from_slice(&file.id().to_le_bytes());
        identity[9..13].copy_from_slice(&self.previous_file.to_le_bytes());
        identity[13..21].copy_from_slice(&sequence_base.to_le_bytes());
        self.append(&Record::new(14, 0, now_ns(), &identity).unwrap())?;
        self.active.as_mut().unwrap().prefix = 1;
        let metadata:&[&[u8]]=&[
            b"{\"type\":\"session\",\"firmware\":\"0.1.0-race\",\"format\":1,\"collection_only\":true,\"time\":\"capture_monotonic_ns\",\"qualified\":false}",
            b"{\"type\":\"imu_profile\",\"sensor\":\"SCH16T\",\"decimation\":5,\"nominal_hz\":737.5,\"gyro_delay_ns\":1400000,\"accel_delay_ns\":1500000,\"delay_uncertainty_ns\":1200000,\"axes\":\"native_sensor\"}",
            b"{\"type\":\"gnss_profile\",\"sensor\":\"UM980\",\"uart_baud\":460800,\"pvt_hz\":50,\"raw_observation_hz\":1,\"pps_hz\":1,\"raw_stream\":\"Unicore native\",\"calibration\":\"mounting and lever arm unqualified\"}",
        ];
        for payload in metadata {
            let a = self.active.as_ref().unwrap();
            let record = Record::new(1, a.prefix, now_ns(), payload).unwrap();
            self.append(&record)?;
            self.active.as_mut().unwrap().prefix += 1;
        }
        let source_metadata = concat!(
            "{\"type\":\"source_build\",\"sha256\":\"",
            env!("AEVIA_BUILD_ID"),
            "\"}"
        );
        let prefix = self.active.as_ref().unwrap().prefix;
        self.append(&Record::new(1, prefix, now_ns(), source_metadata.as_bytes()).unwrap())?;
        self.active.as_mut().unwrap().prefix += 1;
        if let Some(payload) = crate::device::imu::recording_metadata() {
            let prefix = self.active.as_ref().unwrap().prefix;
            self.append(&Record::new(9, prefix, now_ns(), &payload).unwrap())?;
            self.active.as_mut().unwrap().prefix += 1;
        }
        // Each boot's reset/panic provenance is available even before GNSS time.
        let mut reset = [0; 128];
        if let Ok(bytes) = postcard::to_slice(&crate::bootdiagnostics::status(), &mut reset) {
            let prefix = self.active.as_ref().unwrap().prefix;
            self.append(&Record::new(12, prefix, now_ns(), bytes).unwrap())?;
            self.active.as_mut().unwrap().prefix += 1;
        }
        self.flush_page(false)?;
        self.status.state = State::Recording;
        self.status.last_error = None;
        self.last_flush = Instant::now().as_micros();
        self.read_cache = None;
        self.sector_cache = None;
        self.publish();
        Ok(())
    }
    fn start(&mut self) -> Result<Session, Error> {
        if self.status.state == State::Recording {
            return Ok(self.active.as_ref().unwrap().session(false));
        }
        if self.active.is_some() {
            return Err(Error::NotReady);
        }
        if self.fs.is_none() {
            self.mount()?;
        }
        let reset = QUEUE.lock(|q| q.borrow_mut().as_mut().is_some_and(Queue::reset));
        if !reset {
            return Err(Error::Busy);
        }
        SAVED_IMU.store(0, Ordering::Relaxed);
        self.pending_imu = 0;
        // These counters describe the new recording run, including all its
        // rotated files. An earlier missing-card probe must not permanently
        // prevent a later successful run from passing its preflight.
        self.status.io_errors = 0;
        self.status.written_pages = 0;
        self.recording_id = self.next_file;
        self.previous_file = 0;
        self.open_file(0)?;
        enable(true);
        self.bootstrap()?;
        Ok(self.active.as_ref().unwrap().session(false))
    }
    fn bootstrap(&mut self) -> Result<(), Error> {
        let generation = crate::device::gnss::recording::request_bootstrap();
        let prefix = self.active.as_ref().unwrap().prefix;
        self.append(&Record::new(13, prefix, now_ns(), &generation.to_le_bytes()).unwrap())?;
        self.active.as_mut().unwrap().prefix += 1;
        Ok(())
    }
    fn append(&mut self, record: &Record) -> Result<(), Error> {
        let mut encoded = [0; format::RECORD_BYTES];
        let len = record.encode(&mut encoded);
        let mut rest = &encoded[..len];
        while !rest.is_empty() {
            let active = self.active.as_mut().ok_or(Error::NotRecording)?;
            let used = active.page.append(rest);
            rest = &rest[used..];
            if active.page.is_full() {
                self.flush_page(false)?;
            }
        }
        Ok(())
    }
    fn flush_page(&mut self, closed: bool) -> Result<(), Error> {
        let a = self.active.as_mut().ok_or(Error::NotRecording)?;
        if a.page.is_empty() && !closed {
            return Ok(());
        }
        let bytes = a.page.finish(a.file.id(), a.page_index, a.nonce, closed);
        let fs = self.fs.as_mut().ok_or(Error::NotReady)?;
        fs.write_sector(&mut self.card, &a.file, a.page_index, bytes)
            .map_err(fs_error)?;
        // The HAL checks card ready after every sector; this barrier makes the
        // final page and explicit flush acknowledgement obey the same contract.
        if closed {
            fs.flush(&mut self.card).map_err(fs_error)?;
        }
        a.page_index += 1;
        a.page.clear();
        self.status.written_pages = self.status.written_pages.saturating_add(1);
        SAVED_IMU.fetch_add(self.pending_imu, Ordering::Relaxed);
        self.pending_imu = 0;
        progress();
        Ok(())
    }
    fn drain_one(&mut self) -> Result<bool, Error> {
        let Some(mut record) = pop() else {
            return Ok(false);
        };
        let rotate = self
            .active
            .as_ref()
            .is_some_and(|a| a.page_index + 4 >= a.file.sector_count());
        if rotate {
            let next_base = self
                .active
                .as_ref()
                .unwrap()
                .last_source_sequence
                .map_or(0, |s| s.saturating_add(1));
            self.close_file(false)?;
            self.open_file(next_base)?;
            self.bootstrap()?;
        }
        let a = self.active.as_mut().ok_or(Error::NotRecording)?;
        a.last_source_sequence = Some(record.sequence);
        record.sequence = record
            .sequence
            .saturating_sub(a.sequence_base)
            .saturating_add(a.prefix);
        self.append(&record)?;
        if record.kind == 2 {
            self.pending_imu = self.pending_imu.saturating_add(1);
        }
        Ok(true)
    }
    fn health(&self) -> [u8; 41] {
        let (queued, accepted, dropped, _) = queue_status();
        let mut bytes = [0; 41];
        bytes[0] = 1;
        bytes[1..9].copy_from_slice(&accepted.to_le_bytes());
        bytes[9..13].copy_from_slice(&dropped.to_le_bytes());
        for (offset, value) in [
            (13, sensors::IMU_SAMPLES.load(Ordering::Relaxed)),
            (17, sensors::IMU_ERRORS.load(Ordering::Relaxed)),
            (21, sensors::GNSS_UART_ERRORS.load(Ordering::Relaxed)),
            (25, sensors::GNSS_PARSE_ERRORS.load(Ordering::Relaxed)),
        ] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes[29..37].copy_from_slice(&self.status.written_pages.to_le_bytes());
        bytes[37..41].copy_from_slice(&queued.to_le_bytes());
        bytes
    }
    fn close_file(&mut self, stopping: bool) -> Result<Session, Error> {
        let a = self.active.as_ref().ok_or(Error::NotRecording)?;
        let sequence = if stopping {
            queue_status().3
        } else {
            a.last_source_sequence
                .map_or(a.sequence_base, |s| s.saturating_add(1))
        };
        let local_sequence = sequence
            .saturating_sub(a.sequence_base)
            .saturating_add(a.prefix);
        let mut intent = [0; 6];
        intent[0] = 1;
        intent[1] = u8::from(!stopping);
        intent[2..6].copy_from_slice(&(if stopping { 0 } else { self.next_file }).to_le_bytes());
        self.append(&Record::new(15, local_sequence, now_ns(), &intent).unwrap())?;
        self.append(
            &Record::new(
                255,
                local_sequence.saturating_add(1),
                now_ns(),
                &self.health(),
            )
            .unwrap(),
        )?;
        self.flush_page(true)?;
        let session = self.active.take().unwrap().session(true);
        self.last_closed = Some(session);
        self.previous_file = session.session;
        self.status.active_session = None;
        Ok(session)
    }
    fn stop(&mut self) -> Result<Session, Error> {
        if self.status.state == State::Ready {
            return self.last_closed.ok_or(Error::NotRecording);
        }
        if self.status.state != State::Recording {
            return Err(Error::NotRecording);
        }
        enable(false);
        self.status.state = State::Stopping;
        self.publish();
        let mut pending_since = None;
        loop {
            if self.drain_one()? {
                pending_since = None;
                self.telemetry();
                continue;
            }
            if queue_status().0 == 0 {
                break;
            }
            // Disable prevents new reservations, but a preempted producer may
            // still own the oldest slot. Do not seal or reset until it publishes
            // and drains. A stalled producer faults without claiming closure.
            let now = Instant::now().as_micros();
            let started = *pending_since.get_or_insert(now);
            if now.saturating_sub(started) >= PRODUCER_DRAIN_TIMEOUT_US {
                return Err(Error::Corrupt);
            }
            self.telemetry();
            CurrentThreadHandle::get().delay(Duration::from_micros(100));
        }
        let session = self.close_file(true)?;
        self.status.state = State::Ready;
        self.publish();
        Ok(session)
    }
    fn inspect_file(&mut self, file: RecordingFile) -> Result<Session, Error> {
        if let Some(a) = self.active.as_ref().filter(|a| a.file.id() == file.id()) {
            return Ok(a.session(false));
        }
        let fs = self.fs.as_ref().ok_or(Error::NotReady)?;
        let mut bytes = [0; 512];
        fs.read_sector(&mut self.card, &file, 0, &mut bytes)
            .map_err(fs_error)?;
        let Some(first) =
            format::inspect(&bytes).filter(|p| p.session == file.id() && p.index == 0)
        else {
            return Ok(Session {
                session: file.id(),
                file_bytes: file.capacity_bytes(),
                valid_bytes: 0,
                closed: false,
            });
        };
        // Append-only writer stops at the first failed sector. Binary search
        // locates its tail without reading gigabytes of preallocated capacity.
        // The downloader still verifies every intervening CRC independently.
        let mut low = 1;
        let mut high = file.sector_count();
        let mut closed = first.closed;
        while low < high {
            let mid = low + (high - low) / 2;
            fs.read_sector(&mut self.card, &file, mid, &mut bytes)
                .map_err(fs_error)?;
            if let Some(info) = format::inspect(&bytes)
                .filter(|p| p.session == file.id() && p.index == mid && p.nonce == first.nonce)
            {
                low = mid + 1;
                closed = info.closed;
            } else {
                high = mid;
            }
            progress();
        }
        // Binary search may finish on an invalid probe; fetch the exact last
        // committed page for the closed flag instead of retaining a midpoint.
        if low > 1 {
            fs.read_sector(&mut self.card, &file, low - 1, &mut bytes)
                .map_err(fs_error)?;
            closed = format::inspect(&bytes).is_some_and(|p| p.closed);
        }
        Ok(Session {
            session: file.id(),
            file_bytes: file.capacity_bytes(),
            valid_bytes: u64::from(low) * 512,
            closed,
        })
    }
    fn list(&mut self, after_id: u32) -> Result<Option<Session>, Error> {
        let fs = self.fs.as_ref().ok_or(Error::NotReady)?;
        let mut next: Option<RecordingFile> = None;
        fs.visit(&mut self.card, &mut progress, &mut |file| {
            if file.id() > after_id && next.is_none_or(|n| file.id() < n.id()) {
                next = Some(file);
            }
        })
        .map_err(fs_error)?;
        next.map(|file| self.inspect_file(file)).transpose()
    }
    fn read(&mut self, id: u32, offset: u64, len: u16) -> Result<Chunk, Error> {
        if self.status.state == State::Recording || self.status.state == State::Stopping {
            return Err(Error::RecordingActive);
        }
        if len == 0 || usize::from(len) > MAX_READ_BYTES {
            return Err(Error::InvalidLength);
        }
        if self.read_cache.is_none_or(|(file, _)| file.id() != id) {
            let file = self
                .fs
                .as_ref()
                .ok_or(Error::NotReady)?
                .find(&mut self.card, id, &mut progress)
                .map_err(fs_error)?
                .ok_or(Error::SessionNotFound)?;
            let info = self.inspect_file(file)?;
            self.read_cache = Some((file, info));
        }
        let (file, info) = self.read_cache.unwrap();
        if offset
            .checked_add(u64::from(len))
            .is_none_or(|end| end > info.valid_bytes)
        {
            return Err(Error::OutOfRange);
        }
        let mut data = [0; MAX_READ_BYTES];
        let mut index = 0;
        while index < usize::from(len) {
            let position = offset + index as u64;
            let sector = (position / 512) as u32;
            let within_sector = (position % 512) as usize;
            let remaining = usize::from(len) - index;
            if within_sector == 0 && remaining >= 512 {
                let count = remaining / 512 * 512;
                self.fs
                    .as_ref()
                    .unwrap()
                    .read_sectors(
                        &mut self.card,
                        &file,
                        sector,
                        &mut data[index..index + count],
                    )
                    .map_err(fs_error)?;
                index += count;
                progress();
                continue;
            }
            if self
                .sector_cache
                .as_ref()
                .is_none_or(|(cached_id, cached_sector, _)| {
                    *cached_id != id || *cached_sector != sector
                })
            {
                let mut bytes = [0; 512];
                self.fs
                    .as_ref()
                    .unwrap()
                    .read_sector(&mut self.card, &file, sector, &mut bytes)
                    .map_err(fs_error)?;
                self.sector_cache = Some((id, sector, bytes));
            }
            let count = remaining.min(512 - within_sector);
            data[index..index + count].copy_from_slice(
                &self.sector_cache.as_ref().unwrap().2[within_sector..within_sector + count],
            );
            index += count;
        }
        let bytes = &data[..usize::from(len)];
        Ok(Chunk {
            session: id,
            offset,
            bytes: Bytes::new(bytes)?,
            crc32: format::crc32(bytes),
        })
    }
    fn service(&mut self) {
        let request = CONTROL.lock(|m| m.borrow_mut().take());
        let Some(request) = request else { return };
        let result = match request.command {
            Command::Start => self.start().map(Reply::Started),
            Command::Stop => self.stop().map(Reply::Stopped),
            Command::List { after_id } => self.list(after_id).map(Reply::Entry),
            Command::Read {
                session,
                offset,
                len,
            } => self.read(session, offset, len).map(Reply::Data),
            _ => Err(Error::UnknownRequest),
        };
        if let Err(error) = result
            && matches!(
                error,
                Error::Io
                    | Error::Corrupt
                    | Error::NoSpace
                    | Error::NoCard
                    | Error::ReadOnly
                    | Error::UnsupportedFilesystem
            )
        {
            self.fault(error);
        }
        self.publish();
        CONTROL.lock(|m| {
            m.borrow_mut()
                .complete(request, result.unwrap_or_else(Reply::Error))
        });
    }
    fn telemetry(&mut self) {
        progress();
        while let Ok(sample) = sensors::IMU.try_receive() {
            self.snapshot.acceleration_mps2 = sample.acceleration_mps2.map(|v| v as f32);
            self.snapshot.angular_rate_rps = sample.angular_rate_rps.map(|v| v as f32);
            self.snapshot.last_imu_us = sample.captured_ns / 1000;
            self.snapshot.imu_received = self.snapshot.imu_received.saturating_add(1);
        }
        while let Ok(sample) = sensors::GNSS.try_receive() {
            self.snapshot.gnss_received = self.snapshot.gnss_received.saturating_add(1);
            let _ = sample;
        }
        let now = Instant::now().as_micros();
        if now.saturating_sub(self.last_snapshot) >= 100_000 {
            let acquired = sensors::acquisition_status();
            self.snapshot.updated_us = now;
            self.snapshot.phase = match self.status.state {
                State::Recording => Phase::Recording,
                State::Fault => Phase::Failed,
                _ => Phase::Starting,
            };
            self.snapshot.imu_rate_hz =
                (acquired.imu_samples.wrapping_sub(self.previous_imu) as f32) * 1_000_000.0
                    / (now.saturating_sub(self.last_snapshot).max(1) as f32);
            self.previous_imu = acquired.imu_samples;
            self.last_snapshot = now;
            self.snapshot.imu_samples = acquired.imu_samples;
            self.snapshot.gnss_samples = acquired.gnss_samples;
            self.snapshot.gnss_rate_hz = acquired.gnss_rate_hz as f32;
            self.snapshot.imu_errors = acquired.imu_errors;
            self.snapshot.gnss_errors = acquired.gnss_errors;
            self.snapshot.gnss_uart_errors = acquired.gnss_uart_errors;
            self.snapshot.gnss_parse_errors = acquired.gnss_parse_errors;
            self.snapshot.imu_drops = acquired.imu_drops;
            self.snapshot.gnss_drops = acquired.gnss_drops;
            self.snapshot.last_gnss_us = acquired.last_gnss_us.unwrap_or(0);
            self.snapshot.timing = timing::status();
            interface::publish(self.snapshot);
            self.publish();
        }
    }
}
fn now_ns() -> u64 {
    timing::capture::now_ns().unwrap_or_else(|| Instant::now().as_micros().saturating_mul(1000))
}

/// # Safety
/// The caller transfers exclusive ownership of the tested PSRAM region. The
/// estimator must never start in this build, and no heap/DMA uses this region.
pub(crate) unsafe fn run(slot: RecordingSlot, address: usize, len: usize) -> ! {
    assert!(len >= QUEUE_BYTES && address.is_multiple_of(core::mem::align_of::<Record>()));
    let slots = unsafe {
        core::slice::from_raw_parts_mut(
            address as *mut MaybeUninit<Record>,
            QUEUE_BYTES / size_of::<Record>(),
        )
    };
    QUEUE.lock(|q| *q.borrow_mut() = Some(Queue::new(slots)));
    let host = Esp32SdHost::new(slot, SCRATCH.init(DmaScratch::new()));
    let mut worker = Worker {
        card: SdCard::new(host, Delay::new()),
        fs: None,
        active: None,
        status: Status::default(),
        next_file: 1,
        recording_id: 0,
        previous_file: 0,
        last_closed: None,
        read_cache: None,
        sector_cache: None,
        last_flush: 0,
        last_health: 0,
        last_snapshot: 0,
        previous_imu: 0,
        pending_imu: 0,
        snapshot: Snapshot::default(),
    };
    if let Err(error) = worker.mount().and_then(|()| worker.start().map(|_| ())) {
        worker.fault(error);
    }
    // A missing/unsupported card does not hide USB diagnostics. The worker
    // remains responsive; Start can retry a newly inserted card explicitly.
    crate::runtime::set_startup_stage(10);
    loop {
        worker.telemetry();
        worker.service();
        let mut busy = false;
        if worker.status.state == State::Recording {
            for _ in 0..64 {
                match worker.drain_one() {
                    Ok(true) => busy = true,
                    Ok(false) => break,
                    Err(error) => {
                        worker.fault(error);
                        break;
                    }
                }
            }
            let now = Instant::now().as_micros();
            if worker.status.state == State::Recording
                && now.saturating_sub(worker.last_flush) >= FLUSH_INTERVAL_US
            {
                if let Err(error) = worker.flush_page(false) {
                    worker.fault(error);
                }
                worker.last_flush = now;
                worker.publish();
            }
            if now.saturating_sub(worker.last_health) >= 1_000_000 {
                capture(10, now_ns(), &worker.health());
                capture(
                    11,
                    now_ns(),
                    &crate::device::gnss::recording::status().payload(),
                );
                worker.last_health = now;
            }
        }
        if !busy {
            CurrentThreadHandle::get().delay(Duration::from_micros(500));
        }
    }
}
