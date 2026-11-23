use super::{
    filesystem::{self, FileSystem, RecordingFile},
    format::{self, PageBuilder, Record},
    sdcard::{self, DmaScratch, Esp32SdHost, SdCard},
};
use crate::app::Sample;
use core::cell::RefCell;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use esp_hal::{
    Blocking,
    sdmmc::Slot,
    time::{Duration, Instant},
};
use esp_rtos::CurrentThreadHandle;
use heapless::Deque;
use serde::Serialize;
// The SD protocol waits a millisecond between ready polls. Let CPU1 idle
// during that interval, retaining the driver's polling and error checks.
struct SdPollDelay;
impl embedded_hal::delay::DelayNs for SdPollDelay {
    fn delay_ns(&mut self, ns: u32) {
        CurrentThreadHandle::get().delay(Duration::from_micros(u64::from(ns.div_ceil(1000))));
    }
}
type Card = SdCard<Esp32SdHost<'static, 'static, 0>, SdPollDelay>;
const FILE_SECTORS: u32 = 128 * 1024 * 1024 / 512;
static SCRATCH: static_cell::StaticCell<DmaScratch> = static_cell::StaticCell::new();

#[derive(Clone, Copy, Serialize)]
pub struct Status {
    pub state: &'static str,
    pub error: Option<&'static str>,
    pub capacity_bytes: u64,
    pub file_id: Option<u32>,
    pub written_pages: u32,
    pub dropped: u32,
    pub pending: usize,
    pub raw_pending: usize,
    pub raw_dropped: u32,
    pub closed: bool,
}
struct Shared {
    desired: bool,
    queue: Deque<Sample, 384>,
    raw: Deque<crate::ppk::Chunk, 32>,
    status: Status,
    gnss_loss: [GnssTransportLoss; 2],
}
#[derive(Clone, Copy, Serialize)]
struct GnssTransportLoss {
    port: &'static str,
    uart_errors: u64,
    dropped_reads: u64,
    dropped_bytes: u64,
    observation_dropped: u64,
}
impl GnssTransportLoss {
    const fn new(port: &'static str) -> Self {
        Self {
            port,
            uart_errors: 0,
            dropped_reads: 0,
            dropped_bytes: 0,
            observation_dropped: 0,
        }
    }
}
static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> = Mutex::new(RefCell::new(Shared {
    desired: false,
    queue: Deque::new(),
    raw: Deque::new(),
    gnss_loss: [
        GnssTransportLoss::new("data"),
        GnssTransportLoss::new("command"),
    ],
    status: Status {
        state: "starting",
        error: None,
        capacity_bytes: 0,
        file_id: None,
        written_pages: 0,
        dropped: 0,
        pending: 0,
        raw_pending: 0,
        raw_dropped: 0,
        closed: true,
    },
}));
pub fn snapshot() -> Status {
    SHARED.lock(|s| {
        let s = s.borrow();
        Status {
            pending: s.queue.len() + s.raw.len(),
            raw_pending: s.raw.len(),
            ..s.status
        }
    })
}
pub fn request_start() {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.desired || !s.status.closed {
            return;
        }
        s.queue.clear();
        s.raw.clear();
        s.desired = true;
        s.status.state = "opening";
        s.status.error = None;
        s.status.dropped = 0;
        s.status.raw_dropped = 0;
        s.status.written_pages = 0;
        s.status.closed = false;
        s.gnss_loss = [
            GnssTransportLoss::new("data"),
            GnssTransportLoss::new("command"),
        ];
    });
}
pub fn request_stop() {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        s.desired = false;
        if !s.status.closed {
            s.status.state = "stopping";
        }
    });
}
pub fn capture(sample: Sample) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.desired && s.queue.push_back(sample).is_err() {
            s.status.dropped = s.status.dropped.saturating_add(1);
        }
    });
}
pub fn capture_raw(chunk: crate::ppk::Chunk) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.desired && s.raw.push_back(chunk).is_err() {
            s.status.raw_dropped = s.status.raw_dropped.saturating_add(1);
            s.status.dropped = s.status.dropped.saturating_add(1);
        }
    });
}
/// The application gates this to the recording and its stop drain. Physical
/// UART errors have unknown byte loss; rejected byte batches retain exact units.
pub fn note_gnss_transport_loss(port: crate::uart_pipe::Port, dropped_bytes: Option<usize>) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if !s.desired {
            return;
        }
        let loss = &mut s.gnss_loss[port.index()];
        if let Some(bytes) = dropped_bytes {
            loss.dropped_reads = loss.dropped_reads.saturating_add(1);
            loss.dropped_bytes = loss.dropped_bytes.saturating_add(bytes as u64);
        } else {
            loss.uart_errors = loss.uart_errors.saturating_add(1);
        }
    });
}
pub fn note_gnss_observation_loss() {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        if s.desired {
            s.gnss_loss[0].observation_dropped =
                s.gnss_loss[0].observation_dropped.saturating_add(1);
        }
    });
}
fn desired() -> bool {
    SHARED.lock(|s| s.borrow().desired)
}
fn pop() -> Option<Sample> {
    SHARED.lock(|s| s.borrow_mut().queue.pop_front())
}
fn pop_raw() -> Option<crate::ppk::Chunk> {
    SHARED.lock(|s| s.borrow_mut().raw.pop_front())
}
fn update(f: impl FnOnce(&mut Status)) {
    SHARED.lock(|s| f(&mut s.borrow_mut().status));
}
fn progress() {
    crate::navigation::tick();
    CurrentThreadHandle::get().delay(Duration::from_micros(10));
}
fn fault(reason: &'static str) {
    SHARED.lock(|s| {
        let mut s = s.borrow_mut();
        s.desired = false;
        s.queue.clear();
        s.raw.clear();
        s.status.error = Some(reason);
        s.status.state = "fault";
        s.status.closed = true;
    });
}
fn error<E>(value: filesystem::Error<E>) -> &'static str {
    match value {
        filesystem::Error::Unsupported => "exfat-required",
        filesystem::Error::ReadOnly => "volume-dirty-or-read-only",
        filesystem::Error::NoSpace | filesystem::Error::DirectoryFull => "card-full",
        filesystem::Error::Storage(_) => "card-io",
        _ => "filesystem-invalid",
    }
}
#[derive(Clone, Copy)]
pub enum Query {
    Files(u32),
    Read { id: u32, sector: u32 },
}
#[derive(Clone, Copy, Serialize)]
pub struct FileInfo {
    pub id: u32,
    pub bytes: u64,
    pub closed: bool,
}
// Fixed-capacity mailbox avoids heap allocation during a USB storage request.
#[allow(clippy::large_enum_variant)]
pub enum Answer {
    File(Option<FileInfo>),
    Data([u8; 512]),
    Error(&'static str),
}
struct Mailbox {
    query: Option<Query>,
    answer: Option<Answer>,
    busy: bool,
}
static MAILBOX: Mutex<CriticalSectionRawMutex, RefCell<Mailbox>> =
    Mutex::new(RefCell::new(Mailbox {
        query: None,
        answer: None,
        busy: false,
    }));
static ANSWER_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub async fn query(request: Query) -> Answer {
    let accepted = MAILBOX.lock(|m| {
        let mut m = m.borrow_mut();
        if m.busy {
            return false;
        }
        m.answer = None;
        m.busy = true;
        m.query = Some(request);
        true
    });
    if !accepted {
        return Answer::Error("storage-busy");
    }
    // Read the answer under the mailbox lock. A disconnect can leave a stale
    // wakeup, which is harmless because the next request clears the old answer.
    embassy_time::with_timeout(embassy_time::Duration::from_secs(10), async {
        loop {
            if let Some(answer) = MAILBOX.lock(|m| m.borrow_mut().answer.take()) {
                return answer;
            }
            ANSWER_READY.wait().await;
        }
    })
    .await
    .unwrap_or(Answer::Error("storage-request-timeout"))
}
struct Active {
    file: RecordingFile,
    nonce: u64,
    index: u32,
    page: PageBuilder,
    last_snapshot_us: u64,
    sequence: u64,
}
struct Writer {
    card: Card,
    fs: Option<FileSystem>,
    active: Option<Active>,
    next_id: u32,
    last_flush: u64,
    read_cache: Option<(RecordingFile, FileInfo)>,
    raw_open: Option<u32>,
    previous_file: Option<u32>,
}
impl Writer {
    fn mount(&mut self) -> Result<(), &'static str> {
        let info = self
            .card
            .initialize(sdcard::Config::default())
            .map_err(|_| "card-unavailable")?;
        update(|s| s.capacity_bytes = info.sector_count.saturating_mul(512));
        let fs = FileSystem::mount(&mut self.card, &mut progress).map_err(error)?;
        let mut max_id = 0;
        fs.visit(&mut self.card, &mut progress, &mut |file| {
            max_id = max_id.max(file.id())
        })
        .map_err(error)?;
        self.next_id = max_id.checked_add(1).ok_or("file-id-exhausted")?;
        self.fs = Some(fs);
        Ok(())
    }
    fn open(&mut self) -> Result<(), &'static str> {
        self.read_cache = None;
        if self.fs.is_none() {
            self.mount()?;
        }
        let file = self
            .fs
            .as_mut()
            .unwrap()
            .create(&mut self.card, self.next_id, FILE_SECTORS, &mut progress)
            .map_err(error)?;
        self.next_id = self.next_id.checked_add(1).ok_or("file-id-exhausted")?;
        // Leave a complete bounded native frame plus concurrent sensor records
        // before rotating. Small extents cannot provide that contract.
        if file.sector_count() < 2048 {
            return Err("card-full");
        }
        let rng = esp_hal::rng::Rng::new();
        self.active = Some(Active {
            file,
            nonce: (u64::from(rng.random()) << 32) | u64::from(rng.random()),
            index: 0,
            page: PageBuilder::new(),
            last_snapshot_us: 0,
            sequence: 0,
        });
        self.raw_open = None;
        self.append(1,Instant::now().duration_since_epoch().as_micros()*1000,
            b"{\"schema\":\"aevia-usb-v1\",\"gnss_kind\":24,\"imu_kind\":23,\"qualified\":false,\"time\":\"hardware-capture-with-explicit-uncertainty\",\"axes\":\"native-sensor\",\"gnss_log\":\"SPPNAVB\",\"gnss_hz\":50,\"signalgroup\":8,\"imu_logging\":\"native-dec5\",\"nav_average\":4}")?;
        self.append(1,Instant::now().duration_since_epoch().as_micros()*1000,
            b"{\"gnss_wire\":\"unicore-binary\",\"gnss_native_id\":46,\"gnss_time_ref\":\"GPS\",\"imu_wire\":\"safespi-48\"}")?;
        self.append(1,Instant::now().duration_since_epoch().as_micros()*1000,
            b"{\"ppk_kind\":25,\"ppk_schema\":1,\"ppk_format\":\"unicore\",\"ppk_storage\":\"sd-only\",\"ppk_requested_hz\":5,\"ppk_ids\":[12,106,107,108,109,110,112,2999],\"ppk_window\":\"frame-first-byte-in-window;finish-admitted-frame-at-stop\"}")?;
        #[derive(Serialize)]
        struct Part {
            file_id: u32,
            previous_file_id: Option<u32>,
        }
        let mut metadata = [0; 128];
        let used = serde_json_core::to_slice(
            &Part {
                file_id: file.id(),
                previous_file_id: self.previous_file,
            },
            &mut metadata,
        )
        .map_err(|_| "metadata-too-large")?;
        self.append(
            1,
            Instant::now().duration_since_epoch().as_micros() * 1000,
            &metadata[..used],
        )?;
        let setup_time = Instant::now().duration_since_epoch().as_micros() * 1000;
        crate::navigation::race::visit_configuration(|command| {
            let mut data = [0; 32];
            let payload =
                postcard::to_slice(&command, &mut data).map_err(|_| "driving-setup-size")?;
            self.append(27, setup_time, payload)
        })?;
        self.flush(false)?;
        update(|s| {
            s.file_id = Some(file.id());
            s.state = "recording";
            s.closed = false;
        });
        Ok(())
    }
    fn append(&mut self, kind: u8, at_ns: u64, payload: &[u8]) -> Result<(), &'static str> {
        let a = self.active.as_mut().ok_or("not-recording")?;
        let record = Record::new(kind, a.sequence, at_ns, payload).ok_or("record-too-large")?;
        a.sequence += 1;
        for mut part in record.parts() {
            while !part.is_empty() {
                let a = self.active.as_mut().unwrap();
                let n = a.page.append(part);
                part = &part[n..];
                if a.page.is_full() {
                    self.flush(false)?;
                }
            }
        }
        Ok(())
    }
    fn flush(&mut self, closed: bool) -> Result<(), &'static str> {
        let a = self.active.as_mut().ok_or("not-recording")?;
        if a.page.is_empty() && !closed {
            return Ok(());
        }
        if a.index >= a.file.sector_count() {
            return Err("file-full");
        }
        let bytes = a.page.finish(a.file.id(), a.index, a.nonce, closed);
        let fs = self.fs.as_mut().ok_or("not-mounted")?;
        fs.write_sector(&mut self.card, &a.file, a.index, bytes)
            .map_err(error)?;
        if closed {
            fs.flush(&mut self.card).map_err(error)?;
        }
        a.index += 1;
        a.page.clear();
        update(|s| s.written_pages = s.written_pages.saturating_add(1));
        self.last_flush = Instant::now().duration_since_epoch().as_micros();
        progress();
        Ok(())
    }
    fn driving_snapshot(&mut self, terminal: bool) -> Result<(), &'static str> {
        let a = self.active.as_mut().ok_or("not-recording")?;
        let published = crate::navigation::published_us();
        if !terminal && a.last_snapshot_us == published {
            return Ok(());
        }
        let (mut frame, _) = crate::app::display_frame(false);
        let Some(start) = frame.started_at_us else {
            return Ok(());
        };
        if terminal {
            let Some(stop) = frame.stopped_at_us else {
                return Ok(());
            };
            frame.uptime_us = stop;
            frame.motion = None;
        } else if !frame.recording || published < start {
            return Ok(());
        } else if frame.motion.is_some_and(|m| m.t_us < start) {
            frame.motion = None;
        }
        a.last_snapshot_us = published;
        let mut payload = [0; crate::display::MAX_DRIVING_FRAME_BYTES];
        let bytes =
            postcard::to_slice(&frame, &mut payload).map_err(|_| "driving-snapshot-size")?;
        self.append(26, frame.uptime_us * 1000, bytes)
    }

    fn sample(&mut self, sample: Sample) -> Result<(), &'static str> {
        // Leave room for an end record, then rotate without dropping the triggering sample.
        if self
            .active
            .as_ref()
            .is_some_and(|a| a.index + 1024 >= a.file.sector_count())
            && self.raw_open.is_none()
        {
            self.close(false)?;
            self.open()?;
        }
        let mut data = [0; 128];
        let (kind, at_ns, len) = super::codec::encode(sample, &mut data);
        self.append(kind, at_ns, &data[..len])
    }
    fn raw(&mut self, chunk: crate::ppk::Chunk) -> Result<(), &'static str> {
        if chunk.first()
            && self
                .active
                .as_ref()
                .is_some_and(|a| a.index + 1024 >= a.file.sector_count())
        {
            self.close(false)?;
            self.open()?;
        }
        self.raw_open = if chunk.last() {
            None
        } else {
            Some(chunk.frame_seq())
        };
        self.append(
            25,
            chunk.received_us().saturating_mul(1000),
            chunk.as_bytes(),
        )
    }
    fn close(&mut self, terminal: bool) -> Result<(), &'static str> {
        #[derive(Serialize)]
        struct TransportMetadata {
            gnss_transport_loss: GnssTransportLoss,
        }
        let transport = SHARED.lock(|s| s.borrow().gnss_loss);
        for loss in transport {
            let mut metadata = [0; 256];
            let used = serde_json_core::to_slice(
                &TransportMetadata {
                    gnss_transport_loss: loss,
                },
                &mut metadata,
            )
            .map_err(|_| "transport-metadata-too-large")?;
            self.append(
                1,
                Instant::now().duration_since_epoch().as_micros() * 1000,
                &metadata[..used],
            )?;
        }
        let dropped = snapshot().dropped;
        self.append(
            22,
            Instant::now().duration_since_epoch().as_micros() * 1000,
            &dropped.to_le_bytes(),
        )?;
        self.flush(true)?;
        self.previous_file = if terminal {
            None
        } else {
            self.active.as_ref().map(|a| a.file.id())
        };
        self.active = None;
        self.raw_open = None;
        if terminal {
            update(|s| {
                s.state = "ready";
                s.closed = true;
            });
        }
        Ok(())
    }
    fn inspect(&mut self, file: RecordingFile) -> Result<FileInfo, &'static str> {
        let fs = self.fs.as_ref().ok_or("not-mounted")?;
        let mut bytes = [0; 512];
        fs.read_sector(&mut self.card, &file, 0, &mut bytes)
            .map_err(error)?;
        let Some(first) =
            format::inspect(&bytes).filter(|p| p.session == file.id() && p.index == 0)
        else {
            return Ok(FileInfo {
                id: file.id(),
                bytes: 0,
                closed: false,
            });
        };
        let (mut low, mut high) = (1, file.sector_count());
        while low < high {
            let mid = low + (high - low) / 2;
            fs.read_sector(&mut self.card, &file, mid, &mut bytes)
                .map_err(error)?;
            if format::inspect(&bytes)
                .is_some_and(|p| p.session == file.id() && p.index == mid && p.nonce == first.nonce)
            {
                low = mid + 1;
            } else {
                high = mid;
            }
            progress();
        }
        fs.read_sector(&mut self.card, &file, low - 1, &mut bytes)
            .map_err(error)?;
        let closed = format::inspect(&bytes).is_some_and(|p| p.closed);
        Ok(FileInfo {
            id: file.id(),
            bytes: u64::from(low) * 512,
            closed,
        })
    }
    fn query(&mut self, request: Query) -> Result<Answer, &'static str> {
        if self.active.is_some() || desired() {
            return Err("stop-recording-before-download");
        }
        if self.fs.is_none() {
            self.mount()?;
        }
        match request {
            Query::Files(after) => {
                let mut found: Option<RecordingFile> = None;
                self.fs
                    .as_ref()
                    .unwrap()
                    .visit(&mut self.card, &mut progress, &mut |file| {
                        if file.id() > after && found.is_none_or(|f| file.id() < f.id()) {
                            found = Some(file);
                        }
                    })
                    .map_err(error)?;
                Ok(Answer::File(found.map(|f| self.inspect(f)).transpose()?))
            }
            Query::Read { id, sector } => {
                if self.read_cache.is_none_or(|(f, _)| f.id() != id) {
                    let file = self
                        .fs
                        .as_ref()
                        .unwrap()
                        .find(&mut self.card, id, &mut progress)
                        .map_err(error)?
                        .ok_or("file-not-found")?;
                    let info = self.inspect(file)?;
                    self.read_cache = Some((file, info));
                }
                let (file, info) = self.read_cache.unwrap();
                if u64::from(sector) * 512 >= info.bytes {
                    return Err("sector-out-of-range");
                }
                let mut bytes = [0; 512];
                self.fs
                    .as_ref()
                    .unwrap()
                    .read_sector(&mut self.card, &file, sector, &mut bytes)
                    .map_err(error)?;
                Ok(Answer::Data(bytes))
            }
        }
    }
    fn service(&mut self) {
        let request = MAILBOX.lock(|m| m.borrow_mut().query.take());
        if let Some(request) = request {
            let answer = self.query(request).unwrap_or_else(Answer::Error);
            MAILBOX.lock(|m| {
                let mut m = m.borrow_mut();
                m.answer = Some(answer);
                m.busy = false;
            });
            ANSWER_READY.signal(());
        }
    }
}

/// Runs only on CPU1. Owns all SD protocol, filesystem and blocking DMA operations.
pub fn run(slot: Slot<'static, 0, Blocking>) -> ! {
    let mut writer = Writer {
        card: SdCard::new(
            Esp32SdHost::new(slot, SCRATCH.init(DmaScratch::new())),
            SdPollDelay,
        ),
        fs: None,
        active: None,
        next_id: 1,
        last_flush: 0,
        read_cache: None,
        raw_open: None,
        previous_file: None,
    };
    match writer.mount() {
        Ok(()) => update(|s| s.state = "ready"),
        Err(e) => fault(e),
    }
    loop {
        writer.service();
        // CPU1 exclusively owns the estimator and the card. Bound navigation
        // work between storage operations; acquisition remains on CPU0.
        crate::navigation::tick();
        let result = (|| {
            if desired() && writer.active.is_none() {
                writer.open()?;
            }
            if writer.active.is_some() {
                for _ in 0..64 {
                    let sample = pop();
                    let raw = pop_raw();
                    if sample.is_none() && raw.is_none() {
                        break;
                    }
                    if let Some(sample) = sample {
                        writer.sample(sample)?;
                    }
                    if let Some(raw) = raw {
                        writer.raw(raw)?;
                    }
                }
                writer.driving_snapshot(false)?;
                if !desired() && snapshot().pending == 0 {
                    // Let the IMU interval crossing Stop finalize interpolated timings.
                    let awaiting = crate::app::recording_window().1.is_some_and(|stop| {
                        crate::navigation::awaiting_stop(stop)
                            && Instant::now()
                                .duration_since_epoch()
                                .as_micros()
                                .saturating_sub(stop)
                                < 300_000
                    });
                    if !awaiting {
                        writer.driving_snapshot(true)?;
                        writer.close(true)?;
                    }
                } else if Instant::now()
                    .duration_since_epoch()
                    .as_micros()
                    .saturating_sub(writer.last_flush)
                    > 250_000
                {
                    writer.flush(false)?;
                }
            } else if !desired() && !snapshot().closed {
                update(|s| {
                    s.state = "ready";
                    s.closed = true;
                });
            }
            Ok::<_, &'static str>(())
        })();
        if let Err(e) = result {
            writer.active = None;
            writer.fs = None;
            fault(e);
        }
        CurrentThreadHandle::get().delay(Duration::from_millis(1));
    }
}
