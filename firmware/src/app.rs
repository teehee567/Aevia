//! Acquisition and bounded application protocol. USB never owns sensor I/O.
use crate::response::Response;
use core::{cell::RefCell, fmt::Write};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{Blocking, uart::Uart};
use serde::{Deserialize, Serialize};
#[path = "app/gnss_runtime.rs"]
mod gnss_runtime;
pub use gnss_runtime::{gnss, gnss_acquire};

pub const RESPONSE_SIZE: usize = 8192;
pub use crate::sample::Sample;
#[derive(Clone, Copy, Serialize)]
struct RuntimeTiming {
    gnss_work_us: u64,
    gnss_work_max_us: u32,
    gnss_io_work_us: u64,
    gnss_reads: u32,
    gnss_zero_reads: u32,
    gnss_data_bytes: u64,
    gnss_raw_bytes: u64,
    imu_work_us: u64,
    imu_work_max_us: u32,
    imu_intervals: u32,
    binary_work_us: u64,
    binary_records: u32,
    cobs_work_us: u64,
    wire_elapsed_us: u64,
    wire_bytes: u64,
    wire_frames: u32,
    response_elapsed_us: u64,
    response_frames: u32,
    packet_io_us: u64,
    packet_yield_us: u64,
    packets: u32,
}
static RUNTIME_TIMING: Mutex<CriticalSectionRawMutex, RefCell<RuntimeTiming>> =
    Mutex::new(RefCell::new(RuntimeTiming {
        gnss_work_us: 0,
        gnss_work_max_us: 0,
        gnss_io_work_us: 0,
        gnss_reads: 0,
        gnss_zero_reads: 0,
        gnss_data_bytes: 0,
        gnss_raw_bytes: 0,
        imu_work_us: 0,
        imu_work_max_us: 0,
        imu_intervals: 0,
        binary_work_us: 0,
        binary_records: 0,
        cobs_work_us: 0,
        wire_elapsed_us: 0,
        wire_bytes: 0,
        wire_frames: 0,
        response_elapsed_us: 0,
        response_frames: 0,
        packet_io_us: 0,
        packet_yield_us: 0,
        packets: 0,
    }));
#[derive(Clone, Copy, Serialize)]
struct GnssUartDiagnostics {
    errors: u32,
    queue_dropped: u32,
    last_error: Option<&'static str>,
    last_port: Option<&'static str>,
    last_error_us: Option<u64>,
}
static GNSS_UART: Mutex<CriticalSectionRawMutex, RefCell<GnssUartDiagnostics>> =
    Mutex::new(RefCell::new(GnssUartDiagnostics {
        errors: 0,
        queue_dropped: 0,
        last_error: None,
        last_port: None,
        last_error_us: None,
    }));
#[derive(Clone, Copy)]
struct PpkRuntime {
    configured: bool,
    command_errors: u32,
    diagnostics: crate::ppk::Diagnostics,
    in_flight_received_us: Option<u64>,
}
static PPK: Mutex<CriticalSectionRawMutex, RefCell<PpkRuntime>> =
    Mutex::new(RefCell::new(PpkRuntime {
        configured: false,
        command_errors: 0,
        diagnostics: crate::ppk::Decoder::new().diagnostics(),
        in_flight_received_us: None,
    }));
struct GnssObservation {
    sequence: u64,
    received_us: u64,
    native: crate::gnss_binary::NativeSpp,
    mapping: Option<crate::timing::Epoch>,
}
#[derive(Clone, Copy, Serialize)]
struct GnssNativeStatus {
    received_us: u64,
    time_reference: u8,
    time_status: u8,
    gps_week: u16,
    gps_tow_ms: u32,
    position_status: u32,
    position_type: u32,
    velocity_status: u32,
    velocity_type: u32,
    version: u32,
    output_delay_ms: u16,
}
static GNSS_NATIVE: Mutex<CriticalSectionRawMutex, RefCell<Option<GnssNativeStatus>>> =
    Mutex::new(RefCell::new(None));
static GNSS_OBSERVATIONS: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    GnssObservation,
    24,
> = embassy_sync::channel::Channel::new();
static GNSS_OBSERVATION_DRAIN: Mutex<
    CriticalSectionRawMutex,
    RefCell<crate::recording::DrainCounter>,
> = Mutex::new(RefCell::new(crate::recording::DrainCounter::new()));
static UART_PIPE: Mutex<CriticalSectionRawMutex, RefCell<crate::uart_pipe::Ring>> =
    Mutex::new(RefCell::new(crate::uart_pipe::Ring::new()));
static UART_READABLE: embassy_sync::signal::Signal<CriticalSectionRawMutex, ()> =
    embassy_sync::signal::Signal::new();
static GNSS_RECEIVING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static GNSS_CONFIG_QUERY: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
static GNSS_VERIFICATION_INVALIDATE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);
static GNSS_DRAIN: Mutex<CriticalSectionRawMutex, RefCell<crate::recording::DrainCounter>> =
    Mutex::new(RefCell::new(crate::recording::DrainCounter::new()));
#[derive(Clone, Copy, Serialize)]
struct ImuDiagnostics {
    component_id: u16,
    asic_id: u16,
    last_error: Option<crate::imu::ImuError>,
    last_error_us: Option<u64>,
    last_clipping_us: Option<u64>,
    acquisition: crate::imu::AcquisitionTiming,
    raw_queue_dropped: u32,
}
static IMU_DIAGNOSTICS: Mutex<CriticalSectionRawMutex, RefCell<ImuDiagnostics>> =
    Mutex::new(RefCell::new(ImuDiagnostics {
        component_id: 0,
        asic_id: 0,
        last_error: None,
        last_error_us: None,
        last_clipping_us: None,
        acquisition: crate::imu::AcquisitionTiming::new(),
        raw_queue_dropped: 0,
    }));
fn imu_diagnostics(sensor: &crate::drivers::sch16t::Imu<'_>, error: Option<crate::imu::ImuError>) {
    IMU_DIAGNOSTICS.lock(|diagnostics| {
        let mut diagnostics = diagnostics.borrow_mut();
        diagnostics.component_id = sensor.component_id;
        diagnostics.asic_id = sensor.asic_id;
        diagnostics.acquisition = sensor.acquisition_timing;
        if let Some(error) = error {
            let now = Instant::now().as_micros();
            diagnostics.last_error = Some(error);
            diagnostics.last_error_us = Some(now);
            if error.is_clipping() {
                diagnostics.last_clipping_us = Some(now);
            }
        }
    });
}
pub(crate) fn imu_clipping(now_us: u64) -> bool {
    IMU_DIAGNOSTICS.lock(|diagnostics| {
        diagnostics
            .borrow()
            .last_clipping_us
            .and_then(|time| now_us.checked_sub(time))
            .is_some_and(|age| age < 2_000_000)
    })
}
enum AcquisitionMessage {
    Interval(crate::imu::NativeInterval),
    Stop(u64),
}
static IMU_INTERVALS: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    AcquisitionMessage,
    256,
> = embassy_sync::channel::Channel::new();
static STOP_FENCE: Mutex<CriticalSectionRawMutex, RefCell<crate::recording::StopFence>> =
    Mutex::new(RefCell::new(crate::recording::StopFence::new()));
static PPK_STOP_FENCE: Mutex<CriticalSectionRawMutex, RefCell<crate::recording::FrameFence>> =
    Mutex::new(RefCell::new(crate::recording::FrameFence::new()));
static PPK_EPHEMERIDES: Mutex<CriticalSectionRawMutex, RefCell<bool>> =
    Mutex::new(RefCell::new(false));
static STORAGE_STOP_PENDING: Mutex<CriticalSectionRawMutex, RefCell<bool>> =
    Mutex::new(RefCell::new(false));
// 0 idle, 1 requested, 2 command transmitted and receiver given settling time.
static RECEIVER_RESTORE: Mutex<CriticalSectionRawMutex, RefCell<u8>> = Mutex::new(RefCell::new(0));
pub(crate) async fn wait_for_sd_close() -> bool {
    let deadline = Instant::now() + Duration::from_secs(10);
    while (stop_pending() || !crate::storage::snapshot().closed) && Instant::now() < deadline {
        Timer::after_millis(10).await;
    }
    !stop_pending() && crate::storage::snapshot().closed
}

/// Keep the previous session visible until each Stop handler takes its reply.
pub(crate) struct StopReply;

impl Drop for StopReply {
    fn drop(&mut self) {
        update(|s| s.stop_replies -= 1);
    }
}

pub(crate) async fn finish_recording() -> Result<StopReply, &'static str> {
    update(|s| s.stop_replies += 1);
    let reply = StopReply;
    stop_recording();
    if !wait_for_sd_close().await {
        return Err("SD stop flush timeout");
    }
    if let Some(error) = crate::storage::snapshot().error {
        return Err(error);
    }
    if let Some(stop) = recording_window().1 {
        let deadline = Instant::now() + Duration::from_millis(600);
        while Instant::now() < deadline {
            if !crate::navigation::awaiting_stop(stop) {
                break;
            }
            Timer::after_millis(20).await;
        }
    }
    Ok(reply)
}

pub async fn prepare_recovery() -> bool {
    update(|s| {
        s.starting = false;
        if s.recording.active {
            s.recording.stop(Instant::now().as_micros());
            request_storage_stop(s.recording.stopped_at_us.unwrap());
        } else if !stop_pending() {
            crate::storage::request_stop();
        }
    });
    if !wait_for_sd_close().await {
        return false;
    }
    // Preserve compatibility with older MCU images that probe command COM1 at
    // 115200 only. No receiver settings are saved to nonvolatile memory. Failure
    // must not trap recovery if GNSS is disconnected or already malfunctioning.
    RECEIVER_RESTORE.lock(|state| *state.borrow_mut() = 1);
    let deadline = Instant::now() + Duration::from_secs(2);
    while RECEIVER_RESTORE.lock(|state| *state.borrow() != 2) && Instant::now() < deadline {
        Timer::after_millis(10).await;
    }
    true
}
fn stop_pending() -> bool {
    STOP_FENCE.lock(|fence| fence.borrow().pending())
        || GNSS_DRAIN.lock(|drain| drain.borrow().pending())
        || GNSS_OBSERVATION_DRAIN.lock(|drain| drain.borrow().pending())
        || PPK_STOP_FENCE.lock(|fence| fence.borrow().pending())
        || STORAGE_STOP_PENDING.lock(|pending| *pending.borrow())
}
fn maybe_close_storage() {
    if !STORAGE_STOP_PENDING.lock(|pending| *pending.borrow()) {
        return;
    }
    let observations_pending = GNSS_OBSERVATION_DRAIN.lock(|drain| {
        let mut drain = drain.borrow_mut();
        drain.finish_if_drained();
        drain.pending()
    });
    let input_pending = GNSS_DRAIN.lock(|drain| {
        let mut drain = drain.borrow_mut();
        drain.finish_if_drained();
        drain.pending()
    });
    // Only after all pre-stop reads were decoded can an empty parser prove that
    // no queued header remains to start an admitted raw frame.
    let in_flight = PPK.lock(|state| state.borrow().in_flight_received_us);
    PPK_STOP_FENCE.lock(|fence| {
        fence
            .borrow_mut()
            .observe_after_drain(input_pending, in_flight)
    });
    if input_pending
        || observations_pending
        || STOP_FENCE.lock(|fence| fence.borrow().pending())
        || PPK_STOP_FENCE.lock(|fence| fence.borrow().pending())
    {
        return;
    }
    if STORAGE_STOP_PENDING.lock(|pending| core::mem::replace(&mut *pending.borrow_mut(), false)) {
        crate::storage::request_stop();
    }
}
fn request_storage_stop(boundary: u64) {
    if cfg!(feature = "diagnostic-usb-only") || cfg!(feature = "diagnostic-sensors-only") {
        crate::storage::request_stop();
    } else {
        STORAGE_STOP_PENDING.lock(|pending| *pending.borrow_mut() = true);
        STOP_FENCE.lock(|fence| fence.borrow_mut().request(boundary));
        GNSS_DRAIN.lock(|drain| drain.borrow_mut().request_stop());
        GNSS_OBSERVATION_DRAIN.lock(|drain| drain.borrow_mut().request_stop());
        PPK_STOP_FENCE.lock(|fence| fence.borrow_mut().request(boundary, Some(0)));
    }
}
// Called only by the integer producer, after observing the cutoff. Its marker
// follows every older native interval, even if ordinary-context work was delayed.
fn fence_after(observed_us: u64) {
    STOP_FENCE.lock(|fence| {
        let mut fence = fence.borrow_mut();
        if let Some(boundary) = fence.boundary_after(observed_us)
            && IMU_INTERVALS
                .try_send(AcquisitionMessage::Stop(boundary))
                .is_ok()
        {
            fence.queued(boundary);
        }
    });
}
fn fence_before(captured_ns: u64) {
    STOP_FENCE.lock(|fence| {
        let mut fence = fence.borrow_mut();
        if let Some(boundary) = fence.boundary_before_ns(captured_ns)
            && IMU_INTERVALS
                .try_send(AcquisitionMessage::Stop(boundary))
                .is_ok()
        {
            fence.queued(boundary);
        }
    });
}
#[derive(Clone, Copy, Serialize)]
struct SerializationTiming {
    gnss_health_max_us: u32,
    imu_health_max_us: u32,
    status_tail_max_us: u32,
    sample_max_us: u32,
    cobs_max_us: u32,
}
static SERIALIZATION_TIMING: Mutex<CriticalSectionRawMutex, RefCell<SerializationTiming>> =
    Mutex::new(RefCell::new(SerializationTiming {
        gnss_health_max_us: 0,
        imu_health_max_us: 0,
        status_tail_max_us: 0,
        sample_max_us: 0,
        cobs_max_us: 0,
    }));
fn serialized(section: u8, began: Instant) {
    let elapsed = began.elapsed().as_micros().min(u64::from(u32::MAX)) as u32;
    SERIALIZATION_TIMING.lock(|timing| {
        let mut timing = timing.borrow_mut();
        let maximum = match section {
            0 => &mut timing.gnss_health_max_us,
            1 => &mut timing.imu_health_max_us,
            2 => &mut timing.status_tail_max_us,
            3 => &mut timing.sample_max_us,
            _ => &mut timing.cobs_max_us,
        };
        *maximum = (*maximum).max(elapsed);
    });
}
pub fn wire_serialized(began: Instant) {
    serialized(4, began);
    RUNTIME_TIMING.lock(|state| {
        let mut state = state.borrow_mut();
        state.cobs_work_us = state
            .cobs_work_us
            .saturating_add(began.elapsed().as_micros());
    });
}
pub fn wire_sent(began: Instant, bytes: usize) {
    RUNTIME_TIMING.lock(|state| {
        let mut state = state.borrow_mut();
        state.wire_elapsed_us = state
            .wire_elapsed_us
            .saturating_add(began.elapsed().as_micros());
        state.wire_bytes = state.wire_bytes.saturating_add(bytes as u64);
        state.wire_frames = state.wire_frames.saturating_add(1);
    });
}
pub fn response_built(began: Instant) {
    RUNTIME_TIMING.lock(|state| {
        let mut state = state.borrow_mut();
        state.response_elapsed_us = state
            .response_elapsed_us
            .saturating_add(began.elapsed().as_micros());
        state.response_frames = state.response_frames.saturating_add(1);
    });
}
pub fn packet_sent(io_began: Instant, yield_began: Instant) {
    RUNTIME_TIMING.lock(|state| {
        let mut state = state.borrow_mut();
        state.packet_io_us = state
            .packet_io_us
            .saturating_add(yield_began.as_micros().saturating_sub(io_began.as_micros()));
        state.packet_yield_us = state
            .packet_yield_us
            .saturating_add(yield_began.elapsed().as_micros());
        state.packets = state.packets.saturating_add(1);
    });
}
#[derive(Clone, Copy)]
struct Health {
    status: &'static str,
    samples: u32,
    errors: u32,
    last_us: u64,
}
impl Health {
    const fn new() -> Self {
        Self {
            status: "starting",
            samples: 0,
            errors: 0,
            last_us: 0,
        }
    }
}
struct State {
    recording: crate::recording::Recording,
    starting: bool,
    stop_replies: u8,
    session: u32,
    rejected_fix_count: u32,
    queue_enabled: bool,
    queue: crate::preview_queue::PreviewQueue,
    gnss: Health,
    imu: Health,
    last_gnss: Option<Sample>,
    last_imu: Option<Sample>,
}
static STATE: Mutex<CriticalSectionRawMutex, RefCell<State>> = Mutex::new(RefCell::new(State {
    recording: crate::recording::Recording::new(),
    starting: false,
    stop_replies: 0,
    session: 0,
    rejected_fix_count: 0,
    queue_enabled: false,
    queue: crate::preview_queue::PreviewQueue::new(),
    gnss: Health::new(),
    imu: Health::new(),
    last_gnss: None,
    last_imu: None,
}));
fn update(f: impl FnOnce(&mut State)) {
    STATE.lock(|s| f(&mut s.borrow_mut()));
}
fn enqueue(sample: Sample) {
    let capture = STATE.lock(|cell| {
        let s = &mut *cell.borrow_mut();
        complete_recording_start(s);
        if let Sample::Gnss {
            valid,
            latency_ms,
            hacc_m,
            ..
        } = sample
            && (!valid || latency_ms != 0 || !(0.0..=25.0).contains(&hacc_m))
        {
            s.rejected_fix_count = s.rejected_fix_count.wrapping_add(1);
        }
        let (health, latest, t_us) = match sample {
            Sample::Gnss { t_us, .. } => (&mut s.gnss, &mut s.last_gnss, t_us),
            Sample::Imu { t_us, .. } => (&mut s.imu, &mut s.last_imu, t_us),
        };
        health.status = "ready";
        health.samples = health.samples.saturating_add(1);
        health.last_us = t_us;
        *latest = Some(sample);
        let support_start_us = match sample {
            Sample::Imu {
                captured_ns,
                interval_ns,
                ..
            } => captured_ns.saturating_sub(u64::from(interval_ns)) / 1000,
            _ => t_us,
        };
        if s.queue_enabled
            && s.recording
                .started_at_us
                .is_none_or(|start| support_start_us >= start)
            && s.queue.push_back(sample).is_err()
        {
            s.recording.note_overflow(t_us);
        }
        match sample {
            Sample::Imu {
                captured_ns,
                interval_ns,
                ..
            } => s.recording.contains_interval_ns(
                captured_ns.saturating_sub(u64::from(interval_ns)),
                captured_ns,
            ),
            _ => s.recording.contains(t_us),
        }
    });
    if capture {
        crate::storage::capture(sample);
    }
}
pub fn toggle_recording() {
    if STATE.lock(|s| {
        let s = s.borrow();
        s.starting || s.recording.active
    }) {
        stop_recording();
    } else {
        let _ = begin_recording_start(None);
    }
}
fn begin_recording_start(raw_usb: Option<bool>) -> Result<(), ()> {
    STATE.lock(|cell| {
        let mut s = cell.borrow_mut();
        if s.recording.active || s.starting {
            // A second display client must not disable an active USB raw capture.
            if raw_usb == Some(true) {
                s.queue_enabled = true;
            }
            return Ok(());
        }
        if s.stop_replies != 0 || stop_pending() || !crate::storage::snapshot().closed {
            return Err(());
        }
        // Open the SD extent before admitting samples into the recording window.
        s.starting = true;
        if let Some(raw) = raw_usb {
            s.queue_enabled = raw;
        }
        crate::storage::request_start();
        Ok(())
    })
}
pub(crate) async fn start_recording(raw_usb: bool) -> Result<(), &'static str> {
    if begin_recording_start(Some(raw_usb)).is_err() {
        return Err("previous recording still closing");
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let (active, starting) = STATE.lock(|cell| {
            let mut s = cell.borrow_mut();
            complete_recording_start(&mut s);
            (s.recording.active, s.starting)
        });
        if active {
            break;
        }
        if !starting {
            return Err("recording start cancelled");
        }
        let sd = crate::storage::snapshot();
        if sd.error.is_some() || Instant::now() >= deadline {
            update(|s| s.starting = false);
            crate::storage::request_stop();
            return Err("SD start preparation failed");
        }
        Timer::after_millis(5).await;
    }
    Ok(())
}

pub(crate) fn stop_recording() {
    update(|s| {
        if s.recording.active {
            s.recording.stop(Instant::now().as_micros());
            request_storage_stop(s.recording.stopped_at_us.unwrap());
        }
        if s.starting {
            s.starting = false;
            crate::storage::request_stop();
        }
    });
}

fn complete_recording_start(s: &mut State) {
    if s.starting && crate::storage::snapshot().state == "recording" {
        s.starting = false;
        s.recording.start(Instant::now().as_micros());
        s.session = s.session.wrapping_add(1);
        s.queue.clear();
        PPK_EPHEMERIDES.lock(|request| *request.borrow_mut() = true);
    }
}
pub fn recording_window() -> (Option<u64>, Option<u64>) {
    STATE.lock(|c| {
        let s = c.borrow();
        (s.recording.started_at_us, s.recording.stopped_at_us)
    })
}
pub fn dashboard() -> (Option<u16>, bool, bool) {
    let speed = crate::navigation::snapshot()
        .motion
        .map(|m| (m.speed_mps * 3.6).clamp(0., 999.) as u16);
    STATE.lock(|cell| {
        let s = cell.borrow();
        (
            speed,
            s.recording.active,
            s.imu.status == "ready"
                && Instant::now().as_micros().saturating_sub(s.imu.last_us) < 100000,
        )
    })
}
pub fn sensor_indicators() -> ([u8; 3], [u8; 3]) {
    STATE.lock(|cell| {
        let s = cell.borrow();
        let now = Instant::now().as_micros();
        let gnss = if GNSS_UART
            .lock(|diagnostics| diagnostics.borrow().last_error_us)
            .is_some_and(|last| now.saturating_sub(last) < 2_000_000)
        {
            [0, 100, 0]
        } else if s.gnss.status == "ready" {
            match s.last_gnss {
                Some(Sample::Gnss {
                    valid: true, t_us, ..
                }) if now.saturating_sub(t_us) < 1_000_000 => [100, 0, 0],
                _ => [70, 100, 0],
            }
        } else if matches!(s.gnss.status, "starting" | "configuring") {
            [0, 0, 100]
        } else {
            [0, 100, 0]
        };
        let imu = if s.imu.status == "ready" && now.saturating_sub(s.imu.last_us) < 100_000 {
            if IMU_DIAGNOSTICS
                .lock(|diagnostics| diagnostics.borrow().last_error_us)
                .is_some_and(|last| now.saturating_sub(last) < 2_000_000)
            {
                [70, 100, 0]
            } else {
                [100, 0, 0]
            }
        } else if matches!(s.imu.status, "starting" | "initializing") {
            [70, 100, 0]
        } else {
            [0, 100, 0]
        };
        (gnss, imu)
    })
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request<'a> {
    op: &'a str,
    after: Option<u32>,
    id: Option<u32>,
    sector: Option<u32>,
    count: Option<u8>,
    drive: Option<crate::display::Configure>,
}
fn json<T: Serialize>(out: &mut Response<RESPONSE_SIZE>, value: &T) -> Result<(), ()> {
    out.serialize(value).map_err(|_| ())
}
pub async fn request(text: &str, out: &mut Response<RESPONSE_SIZE>) {
    const INVALID_REQUEST: &str = "{\"ok\":false,\"error\":\"invalid application request\"}";
    out.clear();
    let request = match serde_json_core::from_str::<Request<'_>>(text) {
        Ok((request, consumed)) if text[consumed..].trim().is_empty() => request,
        _ => {
            let _ = out.push_str(INVALID_REQUEST);
            return;
        }
    };
    if let Some(command) = request.drive {
        let ok = request.op == "configure_drive" && configure_drive(command).is_ok();
        let _ = out.push_str(if ok {
            r#"{"ok":true}"#
        } else {
            INVALID_REQUEST
        });
        return;
    }
    if matches!(request.op, "power" | "power_sample") {
        if request.after.is_some()
            || request.id.is_some()
            || request.sector.is_some()
            || request.count.is_some()
        {
            let _ = out.push_str(INVALID_REQUEST);
            return;
        }
        match crate::tasks::power::measure(request.op == "power_sample").await {
            Ok(measurement) => {
                let _ = out.push_str(r#"{"ok":true,"measurement":"#);
                let _ = json(out, &measurement);
                let _ = out.push('}');
            }
            Err(error) => {
                let _ = write!(out, "{{\"ok\":false,\"error\":\"{error}\"}}");
            }
        }
        return;
    }
    if request.op == "read_blocks" {
        if read_blocks(&request, out).await.is_err() {
            out.clear();
            let _ = out.push_str(r#"{"ok":false,"error":"sector read failed"}"#);
        }
        return;
    }
    if request.count.is_some() {
        let _ = out.push_str(INVALID_REQUEST);
        return;
    }
    if matches!(request.op, "start" | "start_display")
        && request.after.is_none()
        && request.id.is_none()
        && request.sector.is_none()
        && let Err(message) = start_recording(request.op == "start").await
    {
        let _ = write!(out, "{{\"ok\":false,\"error\":\"{message}\"}}");
        return;
    }
    if matches!(request.op, "files" | "read") {
        let query = match (request.op, request.after, request.id, request.sector) {
            ("files", after, None, None) => Some(crate::storage::Query::Files(after.unwrap_or(0))),
            ("read", None, Some(id), Some(sector)) => {
                Some(crate::storage::Query::Read { id, sector })
            }
            _ => None,
        };
        let answer = match query {
            Some(query) => crate::storage::query(query).await,
            None => crate::storage::Answer::Error("invalid-storage-request"),
        };
        match answer {
            crate::storage::Answer::File(file) => {
                let _ = out.push_str("{\"ok\":true,\"file\":");
                let _ = json(out, &file);
                let _ = out.push('}');
            }
            crate::storage::Answer::Data(bytes) => {
                let _ = write!(
                    out,
                    "{{\"ok\":true,\"id\":{},\"sector\":{},\"hex\":\"",
                    request.id.unwrap_or(0),
                    request.sector.unwrap_or(0)
                );
                let _ = out.push_hex(&bytes);
                let _ = out.push_str("\"}");
            }
            crate::storage::Answer::Error(error) => {
                let _ = write!(out, "{{\"ok\":false,\"error\":\"{error}\"}}");
            }
        }
        return;
    }
    let _stop_reply = if request.op == "stop"
        && request.after.is_none()
        && request.id.is_none()
        && request.sector.is_none()
    {
        match finish_recording().await {
            Ok(reply) => Some(reply),
            Err(error) => {
                let _ = write!(out, "{{\"ok\":false,\"error\":\"{error}\"}}");
                return;
            }
        }
    } else {
        None
    };
    if make_response(&request, out).await.is_err() {
        out.clear();
        let _ = out.push_str(INVALID_REQUEST);
    }
}
async fn read_blocks(request: &Request<'_>, out: &mut Response<RESPONSE_SIZE>) -> Result<(), ()> {
    let (Some(id), Some(first), Some(count @ 1..=8), None) =
        (request.id, request.sector, request.count, request.after)
    else {
        return Err(());
    };
    let end = first.checked_add(u32::from(count)).ok_or(())?;
    write!(out, r#"{{"ok":true,"id":{id},"sector":{first},"blocks":["#).map_err(|_| ())?;
    for sector in first..end {
        let crate::storage::Answer::Data(bytes) =
            crate::storage::query(crate::storage::Query::Read { id, sector }).await
        else {
            return Err(());
        };
        if sector != first {
            out.push(',').map_err(|_| ())?;
        }
        out.push('"').map_err(|_| ())?;
        out.push_base64(&bytes).map_err(|_| ())?;
        out.push('"').map_err(|_| ())?;
    }
    out.push_str("]}").map_err(|_| ())
}

pub(crate) fn latest_gnss_preview(now: u64) -> Option<crate::display::Motion> {
    let sample = STATE.lock(|c| c.borrow().last_gnss);
    let Sample::Gnss {
        t_us,
        lat,
        lon,
        alt_m,
        speed_mps,
        heading_deg,
        sacc_mps,
        valid: true,
        ..
    } = sample?
    else {
        return None;
    };
    if now.checked_sub(t_us)? >= 1_000_000
        || !(0.0..=150.0).contains(&speed_mps)
        || !(0.0..=3.0).contains(&sacc_mps)
    {
        return None;
    }
    Some(crate::display::Motion {
        t_us,
        lat,
        lon,
        alt_m: alt_m as f32,
        speed_mps: speed_mps as f32,
        course_deg: (speed_mps > (3. * sacc_mps).max(1.)).then_some(heading_deg as f32),
        heading_deg: None,
        vehicle_g: None,
        degraded: true,
    })
}

pub(crate) fn display_frame(
    live: bool,
) -> (crate::display::DrivingFrame, crate::display::MotionSource) {
    use crate::display::MotionSource::{Fused, PositionDegraded};
    let nav = crate::navigation::snapshot();
    let frame = STATE.lock(|c| {
        let s = c.borrow();
        let now = Instant::now().as_micros();
        let (satellites, fix_valid) = match s.last_gnss {
            Some(Sample::Gnss {
                satellites,
                valid,
                t_us,
                ..
            }) if now.saturating_sub(t_us) < 1000000 => (Some(satellites), valid),
            _ => (None, false),
        };
        crate::display::DrivingFrame {
            uptime_us: now,
            session: s.session,
            recording: s.recording.active,
            started_at_us: s.recording.started_at_us,
            stopped_at_us: s.recording.stopped_at_us,
            battery_percent: crate::tasks::power::snapshot().percent,
            gnss_count: s.gnss.samples,
            imu_count: s.imu.samples,
            satellites,
            fix_valid,
            mounted: crate::navigation::race::mounted(),
            motion: nav
                .motion
                .filter(|_| live || matches!(nav.motion_source, Fused | PositionDegraded)),
            timing: nav.driving,
        }
    });
    (frame, nav.motion_source)
}
pub(crate) fn configure_drive(command: crate::display::Configure) -> Result<(), ()> {
    STATE.lock(|c| {
        let s = c.borrow();
        if s.starting
            || s.recording.active
            || s.stop_replies != 0
            || !crate::storage::snapshot().closed
        {
            return Err(());
        }
        crate::navigation::race::configure(command)
    })
}
fn display_response(out: &mut Response<RESPONSE_SIZE>) -> Result<(), ()> {
    let (frame, source) = display_frame(true);
    let mut buf = [0; crate::display::MAX_DRIVING_FRAME_BYTES];
    let data = postcard::to_slice(&frame, &mut buf).map_err(|_| ())?;
    out.push_str(r#"{"ok":true,"d2":""#).map_err(|_| ())?;
    out.push_base64(data).map_err(|_| ())?;
    out.push_str(r#"","motion_source":"#).map_err(|_| ())?;
    json(out, &source)?;
    out.push_str(r#","imu_clipping":"#).map_err(|_| ())?;
    json(out, &imu_clipping(frame.uptime_us))?;
    out.push('}').map_err(|_| ())
}

async fn make_response(request: &Request<'_>, out: &mut Response<RESPONSE_SIZE>) -> Result<(), ()> {
    if !matches!(
        request.op,
        "status"
            | "start"
            | "start_display"
            | "display"
            | "stop"
            | "poll"
            | "drain"
            | "drain_binary"
            | "drain_base64"
    ) || request.after.is_some()
        || request.id.is_some()
        || request.sector.is_some()
    {
        return Err(());
    }
    if request.op == "display" {
        return display_response(out);
    }
    if matches!(
        request.op,
        "poll" | "drain" | "drain_binary" | "drain_base64"
    ) {
        update(|s| s.queue_enabled = true);
    }
    if matches!(request.op, "drain" | "drain_binary" | "drain_base64") {
        let encoding = match request.op {
            "drain_base64" => SampleEncoding::Base64,
            "drain_binary" => SampleEncoding::Hex,
            _ => SampleEncoding::Json,
        };
        return make_compact_response(out, encoding).await;
    }
    if matches!(request.op, "start" | "start_display")
        && !STATE.lock(|s| s.borrow().recording.active)
    {
        return Err(());
    }
    let (recording, started_at_us, stopped_at_us, session, gnss, imu, last_gnss, last_imu) = STATE
        .lock(|cell| {
            let s = cell.borrow();
            (
                s.recording.active,
                s.recording.started_at_us,
                s.recording.stopped_at_us,
                s.session,
                s.gnss,
                s.imu,
                s.last_gnss,
                s.last_imu,
            )
        });
    let now = Instant::now().as_micros();
    write!(out, "{{\"ok\":true,\"protocol\":1,\"display_v2\":true,\"read_blocks\":8,\"compact_poll\":true,\"binary_poll\":true,\"base64_poll\":true,\"device\":\"Aevia V2 Mini\",\"firmware\":\"0.3.0\",\"recording\":{recording},\"session\":{session},\"stopped_at_us\":").map_err(|_| ())?;
    json(out, &stopped_at_us)?;
    out.push_str(",\"started_at_us\":").map_err(|_| ())?;
    json(out, &started_at_us)?;
    out.push_str(",\"mounted\":").map_err(|_| ())?;
    json(out, &crate::navigation::race::mounted())?;
    out.push_str(",\"usb_queue\":").map_err(|_| ())?;
    json(out, &STATE.lock(|state| state.borrow().queue.status()))?;
    out.push_str(",\"acquisition_profile\":{\"gnss_log\":\"SPPNAVB\",\"gnss_wire\":\"unicore-binary\",\"imu_wire\":\"safespi-48\",\"gnss_requested_hz\":50,\"signalgroup\":8,\"data_baud\":460800,\"raw_baud\":460800,\"imu_logging\":\"native-dec5\",\"imu_nominal_hz\":737.5,\"imu_navigation_average\":4,\"usb_decimation\":1}").map_err(|_| ())?;
    out.push_str(",\"build_mode\":").map_err(|_| ())?;
    json(
        out,
        &if cfg!(feature = "diagnostic-usb-only") {
            "diagnostic-usb-only"
        } else if cfg!(feature = "diagnostic-sensors-only") {
            "diagnostic-sensors-only"
        } else if cfg!(feature = "diagnostic-no-navigation") {
            "diagnostic-no-navigation"
        } else {
            "application"
        },
    )?;
    out.push_str(",\"gnss\":").map_err(|_| ())?;
    let began = Instant::now();
    health_json(out, gnss, last_gnss, now)?;
    out.push_str(",\"gnss_native\":").map_err(|_| ())?;
    json(out, &GNSS_NATIVE.lock(|state| *state.borrow()))?;
    out.push_str(",\"gnss_uart\":").map_err(|_| ())?;
    json(out, &GNSS_UART.lock(|diagnostics| *diagnostics.borrow()))?;
    out.push_str(",\"gnss_byte_queue\":").map_err(|_| ())?;
    json(out, &UART_PIPE.lock(|pipe| pipe.borrow().status()))?;
    let ppk = PPK.lock(|state| *state.borrow());
    let d = ppk.diagnostics;
    let sd = crate::storage::snapshot();
    write!(out, ",\"ppk\":{{\"storage\":\"sd-only\",\"format\":\"unicore\",\"requested_hz\":5,\"configured\":{},\"command_errors\":{},\"frames\":{},\"observation_frames\":{},\"ephemeris_frames\":{},\"bytes\":{},\"crc_failures\":{},\"truncated_frames\":{},\"maximum_frame_bytes\":{},\"last_observation_count\":{},\"dropped\":{},\"pending\":{},\"last_received_us\":", ppk.configured, ppk.command_errors, d.frames, d.observation_frames, d.ephemeris_frames, d.bytes, d.crc_failures, d.truncated_frames, d.maximum_frame_bytes, d.last_observation_count, sd.raw_dropped, sd.raw_pending).map_err(|_| ())?;
    json(out, &d.last_received_us)?;
    out.push_str(",\"last_observation_us\":").map_err(|_| ())?;
    json(out, &d.last_observation_us)?;
    out.push('}').map_err(|_| ())?;
    serialized(0, began);
    embassy_futures::yield_now().await;
    out.push_str(",\"imu\":").map_err(|_| ())?;
    let began = Instant::now();
    health_json(out, imu, last_imu, now)?;
    out.push_str(",\"imu_clipping\":").map_err(|_| ())?;
    json(out, &imu_clipping(now))?;
    serialized(1, began);
    embassy_futures::yield_now().await;
    let mut began = Instant::now();
    out.push_str(",\"imu_diagnostics\":").map_err(|_| ())?;
    json(
        out,
        &IMU_DIAGNOSTICS.lock(|diagnostics| *diagnostics.borrow()),
    )?;
    let battery = crate::tasks::power::snapshot();
    write!(
        out,
        ",\"battery\":{{\"status\":\"{}\",\"percent\":",
        battery.state.label()
    )
    .map_err(|_| ())?;
    json(out, &battery.percent)?;
    out.push_str(",\"millivolts\":").map_err(|_| ())?;
    json(out, &battery.millivolts)?;
    out.push_str(",\"power_source\":").map_err(|_| ())?;
    json(out, &battery.power_source)?;
    out.push_str(",\"charge_stage\":").map_err(|_| ())?;
    json(out, &battery.charge_stage)?;
    out.push_str(",\"rate_percent_per_hour\":")
        .map_err(|_| ())?;
    json(out, &battery.rate_percent_per_hour)?;
    out.push_str(",\"temperature_zone\":").map_err(|_| ())?;
    json(out, &battery.temperature_zone())?;
    out.push_str(",\"temperatures_c\":").map_err(|_| ())?;
    json(
        out,
        &crate::tasks::thermal::packed().map(crate::display::unpack_temperature),
    )?;
    out.push_str(",\"charger_error\":").map_err(|_| ())?;
    json(out, &battery.charger_error)?;
    out.push_str(",\"gauge_error\":").map_err(|_| ())?;
    json(out, &battery.gauge_error)?;
    out.push_str(",\"faults\":").map_err(|_| ())?;
    json(out, &battery.faults)?;
    out.push_str(",\"errors\":").map_err(|_| ())?;
    json(out, &battery.errors)?;
    serialized(2, began);
    embassy_futures::yield_now().await;
    began = Instant::now();
    out.push_str("},\"buttons\":").map_err(|_| ())?;
    json(out, &crate::drivers::tca9536::pressed())?;
    out.push_str(",\"sd\":").map_err(|_| ())?;
    json(out, &crate::storage::snapshot())?;
    serialized(2, began);
    embassy_futures::yield_now().await;
    began = Instant::now();
    out.push_str(",\"navigation\":").map_err(|_| ())?;
    json(out, &crate::navigation::snapshot())?;
    serialized(2, began);
    embassy_futures::yield_now().await;
    began = Instant::now();
    out.push_str(",\"installation\":").map_err(|_| ())?;
    json(out, &crate::navigation::settings::snapshot())?;
    serialized(2, began);
    embassy_futures::yield_now().await;
    began = Instant::now();
    out.push_str(",\"capture_timing\":").map_err(|_| ())?;
    json(out, &crate::timing::status())?;
    out.push_str(",\"serialization_timing\":").map_err(|_| ())?;
    json(out, &SERIALIZATION_TIMING.lock(|timing| *timing.borrow()))?;
    out.push_str(",\"runtime_timing\":").map_err(|_| ())?;
    json(out, &RUNTIME_TIMING.lock(|timing| *timing.borrow()))?;
    write!(
        out,
        ",\"accessories\":{{\"display_fitted\":{},\"buttons_fitted\":{}}}",
        crate::configuration::DISPLAY_FITTED,
        crate::configuration::BUTTONS_FITTED
    )
    .map_err(|_| ())?;
    write!(out, ",\"led\":\"{}\",\"display\":\"{}\",\"storage\":\"sd-exfat-and-usb-host\",\"bluetooth\":\"ble-display-v2\",\"timing\":\"hardware-capture-with-unqualified-delay-model\",\"samples\":[", crate::drivers::lp5813::status(), crate::tasks::display::status()).map_err(|_| ())?;
    serialized(2, began);
    embassy_futures::yield_now().await;
    for i in 0..if request.op == "poll" { 8 } else { 0 } {
        let Some(sample) = STATE.lock(|cell| cell.borrow_mut().queue.pop_front()) else {
            break;
        };
        let began = Instant::now();
        if out
            .serialize_separated_reserved(if i == 0 { "" } else { "," }, &sample, 128)
            .is_err()
        {
            // Full legacy diagnostics can leave room for fewer than eight
            // samples. Restore the uncommitted sample before yielding.
            STATE.lock(|cell| {
                let _ = cell.borrow_mut().queue.push_front(sample);
            });
            break;
        }
        serialized(3, began);
        embassy_futures::yield_now().await;
    }
    let (pending, dropped) = STATE.lock(|cell| {
        let s = cell.borrow();
        (s.queue.len(), s.recording.dropped)
    });
    let now = Instant::now().as_micros();
    write!(
        out,
        "],\"pending\":{pending},\"dropped\":{dropped},\"uptime_us\":{now}}}"
    )
    .map_err(|_| ())?;
    Ok(())
}
#[derive(Clone, Copy, PartialEq)]
enum SampleEncoding {
    Json,
    Hex,
    Base64,
}
async fn make_compact_response(
    out: &mut Response<RESPONSE_SIZE>,
    encoding: SampleEncoding,
) -> Result<(), ()> {
    let binary = encoding != SampleEncoding::Json;
    let mut base64 = crate::response::Base64Records::new();
    let limit = match encoding {
        SampleEncoding::Json => 16,
        SampleEncoding::Hex => 32,
        SampleEncoding::Base64 => 48,
    };
    let (recording, session, started, stopped) = STATE.lock(|cell| {
        let s = cell.borrow();
        (
            s.recording.active,
            s.session,
            s.recording.started_at_us,
            s.recording.stopped_at_us,
        )
    });
    if binary {
        write!(out, "{{\"ok\":true,\"protocol\":1,\"compact\":true,\"binary\":true,\"recording\":{recording},\"session\":{session},\"started_at_us\":").map_err(|_| ())?;
        json(out, &started)?;
        out.push_str(",\"stopped_at_us\":").map_err(|_| ())?;
        json(out, &stopped)?;
        out.push_str(if encoding == SampleEncoding::Base64 {
            ",\"records_b64\":\""
        } else {
            ",\"records_hex\":\""
        })
        .map_err(|_| ())?;
    } else {
        out.begin_compact(recording, session, started, stopped)
            .map_err(|_| ())?;
    }
    let mut last_yield = Instant::now();
    for index in 0_usize..limit {
        let Some(sample) = STATE.lock(|cell| cell.borrow_mut().queue.pop_front()) else {
            break;
        };
        let began = Instant::now();
        let fits = if binary {
            let mut data = [0; 128];
            let (kind, _, len) = crate::storage::codec::encode(sample, &mut data);
            if encoding == SampleEncoding::Base64 {
                out.record_base64(&mut base64, kind, &data[..len], 128)
                    .is_ok()
            } else {
                out.record_hex(kind, &data[..len], 128).is_ok()
            }
        } else {
            out.serialize_separated_reserved(if index == 0 { "" } else { "," }, &sample, 128)
                .is_ok()
        };
        if binary {
            RUNTIME_TIMING.lock(|state| {
                let mut state = state.borrow_mut();
                state.binary_work_us = state
                    .binary_work_us
                    .saturating_add(began.elapsed().as_micros());
                state.binary_records = state.binary_records.saturating_add(u32::from(fits));
            });
        }
        if !fits {
            // No await occurred since pop. Only ordinary-context consumers push
            // this queue, so its newly freed front slot is still available.
            STATE.lock(|cell| {
                let _ = cell.borrow_mut().queue.push_front(sample);
            });
            break;
        }
        serialized(3, began);
        // Each cooperative yield costs approximately one native IMU period on
        // this executor. IRQ acquisition remains preemptive; bound ordinary
        // context work by four milliseconds instead. There is no extra yield
        // after the last record: endpoint I/O follows this bounded response.
        if index + 1 < limit
            && (!binary || last_yield.elapsed().as_micros() >= 4_000)
            && STATE.lock(|cell| !cell.borrow().queue.is_empty())
        {
            embassy_futures::yield_now().await;
            last_yield = Instant::now();
        }
    }
    if encoding == SampleEncoding::Base64 {
        out.finish_base64(&mut base64).map_err(|_| ())?;
    }
    let (pending, dropped) = STATE.lock(|cell| {
        let s = cell.borrow();
        (s.queue.len(), s.recording.dropped)
    });
    if binary {
        let now = Instant::now().as_micros();
        write!(
            out,
            "\",\"pending\":{pending},\"dropped\":{dropped},\"uptime_us\":{now}}}"
        )
        .map_err(|_| ())
    } else {
        out.finish_compact(pending, dropped, || Instant::now().as_micros())
            .map_err(|_| ())
    }
}
fn health_json(
    out: &mut Response<RESPONSE_SIZE>,
    health: Health,
    latest: Option<Sample>,
    now: u64,
) -> Result<(), ()> {
    let age = now.saturating_sub(health.last_us);
    let status = if health.status == "ready" && age > 1_000_000 {
        "stale"
    } else {
        health.status
    };
    write!(
        out,
        "{{\"status\":\"{status}\",\"count\":{},\"errors\":{},\"age_us\":{age},\"latest\":",
        health.samples, health.errors
    )
    .map_err(|_| ())?;
    json(out, &latest)?;
    out.push('}').map_err(|_| ())
}
#[embassy_executor::task]
pub async fn imu_acquire(mut sensor: crate::drivers::sch16t::Imu<'static>) {
    let mut generation = 0_u32;
    loop {
        generation = generation.wrapping_add(1);
        crate::tasks::thermal::set_imu(None);
        update(|s| s.imu.status = "initializing");
        if let Err(error) = sensor.initialize().await {
            imu_diagnostics(&sensor, Some(error));
            update(|s| {
                s.imu.status = "initialization-failed";
                s.imu.errors = s.imu.errors.saturating_add(1);
            });
            fence_after(Instant::now().as_micros());
            Timer::after_secs(3).await;
            continue;
        }
        imu_diagnostics(&sensor, None);
        fence_after(Instant::now().as_micros());
        let mut failures = 0;
        let mut next_temperature_us = 0;
        loop {
            match sensor.next_native().await {
                Ok(mut interval) => {
                    imu_diagnostics(&sensor, None);
                    failures = 0;
                    interval.generation = generation;
                    let observed_us = interval.captured_ns / 1000;
                    // A post-cutoff preview interval must not steal the last
                    // free slot from the pending stop fence under overload.
                    fence_before(interval.captured_ns);
                    if IMU_INTERVALS
                        .try_send(AcquisitionMessage::Interval(interval))
                        .is_err()
                    {
                        // Never block acquisition on the lower-priority consumer.
                        // Its next accepted generation discards any partial batch.
                        generation = generation.wrapping_add(1);
                        imu_diagnostics(
                            &sensor,
                            Some(crate::imu::ImuError::AcquisitionQueueOverflow),
                        );
                        IMU_DIAGNOSTICS.lock(|diagnostics| {
                            let mut diagnostics = diagnostics.borrow_mut();
                            diagnostics.raw_queue_dropped =
                                diagnostics.raw_queue_dropped.saturating_add(1);
                        });
                        update(|s| {
                            s.imu.status = "sample-error";
                            s.imu.errors = s.imu.errors.saturating_add(1);
                        });
                    }
                    fence_after(observed_us);
                    // Stay on the SPI owner's executor and read only after
                    // an entire inertial burst has been handed off.
                    if observed_us >= next_temperature_us {
                        next_temperature_us = observed_us + 10_000_000;
                        crate::tasks::thermal::set_imu(sensor.temperature_c());
                    }
                }
                Err(error) => {
                    generation = generation.wrapping_add(1);
                    imu_diagnostics(&sensor, Some(error));
                    update(|s| {
                        s.imu.status = "sample-error";
                        s.imu.errors = s.imu.errors.saturating_add(1);
                    });
                    failures += 1;
                    fence_after(Instant::now().as_micros());
                    if failures >= 20 {
                        break;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn imu() {
    let mut averager = crate::imu::Averager::default();
    loop {
        let interval = match IMU_INTERVALS.receive().await {
            AcquisitionMessage::Interval(interval) => interval,
            AcquisitionMessage::Stop(boundary) => {
                averager = crate::imu::Averager::default();
                STOP_FENCE.lock(|fence| fence.borrow_mut().complete(boundary));
                maybe_close_storage();
                continue;
            }
        };
        let processing_began = Instant::now();
        let aligned = interval.align();
        if let Some(sample) = averager.push_aligned(aligned) {
            crate::navigation::capture_imu(sample);
        }
        let Some(aligned) = aligned else {
            continue;
        };
        let sample = aligned.scaled();
        enqueue(Sample::Imu {
            t_us: sample.monotonic_us,
            dt_us: sample.interval_us,
            accel: sample.acceleration_mps2,
            gyro: sample.angular_rate_rps,
            valid: true,
            captured_ns: sample.captured_ns,
            interval_ns: sample.interval_ns,
            effective_end_ns: sample.effective_end_ns,
            timing_uncertainty_ns: sample.timing_uncertainty_ns,
            aligned_gyro: sample.aligned_angular_rate_rps,
        });
        RUNTIME_TIMING.lock(|state| {
            let mut state = state.borrow_mut();
            let elapsed = processing_began.elapsed().as_micros();
            state.imu_work_us = state.imu_work_us.saturating_add(elapsed);
            state.imu_work_max_us = state
                .imu_work_max_us
                .max(elapsed.min(u64::from(u32::MAX)) as u32);
            state.imu_intervals = state.imu_intervals.saturating_add(1);
        });
    }
}
