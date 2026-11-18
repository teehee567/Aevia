//! Native acquisition and persistence. The collector owns the serial port so
//! Flutter frame timing cannot drop sensor data.
use crate::model::*;
use serde_json::{json, Value};
use serialport::{SerialPort, SerialPortType};
use std::{
    fs::{self, File, OpenOptions},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        Mutex, OnceLock,
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const MAX_IMPORT_BYTES: u64 = 256 * 1024 * 1024;
// Raw observations and projected samples share this budget. The other half of
// the canonical file ceiling is reserved for metadata and analysis output.
const MAX_CAPTURE_DATA_BYTES: u64 = MAX_IMPORT_BYTES / 2;
const COLLECT_TURN_BUDGET: Duration = Duration::from_millis(125);
const MAX_CAPTURE_BYTES: u64 = 8 * 1024 * 1024 * 1024;
const MAX_SAMPLES: usize = 2_000_000;
static SERVICE: OnceLock<Mutex<Service>> = OnceLock::new();
static CANCEL_PROCESSING: AtomicBool = AtomicBool::new(false);
static PROCESSING_ACTIVE: AtomicBool = AtomicBool::new(false);
pub(crate) fn processing_cancelled() -> bool {
    CANCEL_PROCESSING.load(Ordering::Relaxed)
}

struct Capture {
    session: Session,
    journal: File,
    journal_line: Vec<u8>,
    clock: SampleClock,
    last_sync: Instant,
    last_dropped: u64,
    analyzed_samples: usize,
    interrupted: bool,
    started: Instant,
    arrival_origin_us: Option<u64>,
    stop_at_us: Option<u64>,
    data_bytes: u64,
    health_start: Value,
    health_latest: Value,
    health_counter_resets: Vec<String>,
    stop_boundary_confirmed: bool,
    stop_tail_pending: Option<u64>,
    stop_tail_received: u64,
    evidence: Option<crate::evidence::Writer>,
    binary_imu_count: u64,
    binary_imu_interval_ns: u64,
    binary_pre_start_count: u64,
    binary_first_imu: Option<Value>,
    binary_last_imu: Option<Value>,
}
struct Service {
    root: PathBuf,
    preferences: Preferences,
    sessions: Vec<Value>,
    port: Option<Box<dyn SerialPort>>,
    port_name: Option<String>,
    status: String,
    latest: Value,
    last_seen: Option<Instant>,
    last_full_status: Option<Instant>,
    sequence: u64,
    gnss_rate: Option<f64>,
    imu_rate: Option<f64>,
    imu_rate_sample: Option<(Instant, u64)>,
    rate_sample: Option<(Instant, u64)>,
    capture: Option<Capture>,
    processing: Option<String>,
}

pub fn request(input: String) -> String {
    let result = (|| -> Result<Value, String> {
        let query: Value = serde_json::from_str(&input).map_err(|e| e.to_string())?;
        let operation = text(&query, "op")?;
        if operation == "cancel_processing" {
            let active = PROCESSING_ACTIVE.load(Ordering::Relaxed);
            if active {
                CANCEL_PROCESSING.store(true, Ordering::Relaxed);
            }
            return Ok(json!({"cancel_requested":active}));
        }
        let mutex = SERVICE.get_or_init(|| {
            let service = Service::new();
            std::thread::Builder::new()
                .name("aevia-acquisition".into())
                .stack_size(32 * 1024 * 1024)
                .spawn(|| {
                    let mut delay = Duration::from_millis(40);
                    loop {
                        // The state lock from the preceding turn is gone before
                        // pacing. Backlogged receivers need another bounded turn
                        // promptly; idle/error paths retain the normal cadence.
                        std::thread::sleep(delay);
                        delay = Duration::from_millis(40);
                        if let Some(mutex) = SERVICE.get() {
                            if let Ok(mut service) = mutex.lock() {
                                let before = service.sequence;
                                service.collect();
                                delay = service.next_collection_delay(
                                    service.port.is_some() && service.sequence != before,
                                );
                            }
                        }
                    }
                })
                .expect("acquisition worker");
            std::thread::Builder::new()
                .name("aevia-live-metrics".into())
                .stack_size(32 * 1024 * 1024)
                .spawn(live_metrics_worker)
                .expect("live metrics worker");
            Mutex::new(service)
        });
        let mut service = mutex
            .lock()
            .map_err(|_| "Acquisition service stopped unexpectedly")?;
        if matches!(operation, "stop" | "disconnect") && service.capture.is_some() {
            let capture = service.stop_acquisition()?.ok_or("Recording disappeared")?;
            if operation == "disconnect" {
                service.port = None;
                service.port_name = None;
            }
            let mut offline = service.offline_copy();
            service.processing = Some("finalizing recording".into());
            service.status = "Recording stopped; saving and analyzing".into();
            CANCEL_PROCESSING.store(false, Ordering::Relaxed);
            PROCESSING_ACTIVE.store(true, Ordering::Relaxed);
            drop(service);
            let capture_id = capture.session.id.clone();
            let interrupted = capture.interrupted;
            let result = offline.finalize_capture(capture);
            let mut service = mutex
                .lock()
                .map_err(|_| "Acquisition service stopped unexpectedly")?;
            service.processing = None;
            service.sessions = offline.sessions;
            PROCESSING_ACTIVE.store(false, Ordering::Relaxed);
            CANCEL_PROCESSING.store(false, Ordering::Relaxed);
            service.status = if result.is_ok() && interrupted {
                "Incomplete recording saved; capture journal retained".into()
            } else if result.is_ok() {
                "Recording saved".into()
            } else if !service
                .sessions
                .iter()
                .any(|s| s["id"].as_str() == Some(&capture_id))
            {
                "Saving failed; the capture journal is retained for recovery".into()
            } else {
                "Recording saved before analysis; reanalyze from the library".into()
            };
            result?;
            return Ok(service.state());
        }
        if matches!(
            operation,
            "import"
                | "demo"
                | "reanalyze"
                | "session"
                | "export"
                | "inspect"
                | "ppk_status"
                | "prepare_ppk"
                | "run_ppk"
                | "import_ppk"
        ) {
            if service.capture.is_some() {
                return Err("Stop recording before processing sessions".into());
            }
            if service.processing.is_some() {
                return Err("A session is already processing".into());
            }
            let mut offline = service.offline_copy();
            service.processing = Some(operation.into());
            CANCEL_PROCESSING.store(false, Ordering::Relaxed);
            PROCESSING_ACTIVE.store(true, Ordering::Relaxed);
            drop(service);
            let result = offline.execute(&query);
            let mut service = mutex
                .lock()
                .map_err(|_| "Acquisition service stopped unexpectedly")?;
            service.processing = None;
            PROCESSING_ACTIVE.store(false, Ordering::Relaxed);
            if result.is_ok() {
                service.sessions = offline.sessions;
            }
            CANCEL_PROCESSING.store(false, Ordering::Relaxed);
            return result;
        }
        if operation == "start" && service.processing.is_some() {
            return Err(
                "Wait for session processing to finish or cancel it before recording".into(),
            );
        }
        service.execute(&query)
    })();
    match result {
        Ok(data) => json!({"ok":true,"data":data}),
        Err(error) => json!({"ok":false,"error":error}),
    }
    .to_string()
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn live_metrics_worker() {
    loop {
        std::thread::sleep(Duration::from_secs(1));
        let snapshot = SERVICE.get().and_then(|mutex| {
            mutex.lock().ok().and_then(|service| {
                service
                    .capture
                    .as_ref()
                    .filter(|c| {
                        c.session.samples.len() >= 2
                            && c.session.samples.len() != c.analyzed_samples
                    })
                    .map(|c| {
                        (
                            c.session.id.clone(),
                            c.session.samples.clone(),
                            c.session.track.clone(),
                            c.session.drag.clone(),
                            c.last_dropped,
                        )
                    })
            })
        });
        if let Some((id, samples, track, drag, dropped)) = snapshot {
            // No serial lock is held while Traj evaluates the snapshot.
            let health = if dropped > 0 {
                vec![json!({"kind":"health","dropped":dropped})]
            } else {
                vec![]
            };
            let analysis =
                crate::trajectory_adapter::analyze(&samples, &health, track.as_ref(), &drag);
            if let Some(mutex) = SERVICE.get() {
                if let Ok(mut service) = mutex.lock() {
                    if let Some(capture) = service.capture.as_mut().filter(|c| c.session.id == id) {
                        capture.session.analysis = analysis;
                        capture.analyzed_samples = samples.len();
                    }
                }
            }
        }
    }
}
fn number(value: &Value, key: &str) -> Option<f64> {
    value
        .get(key)
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite())
}
fn sensor_record_ns(record: &Value) -> Option<u128> {
    if record["kind"] == "imu" {
        if let Some(ns) = record["captured_ns"].as_u64() {
            return Some(u128::from(ns));
        }
    }
    record["t_us"].as_u64().map(|us| u128::from(us) * 1000)
}
fn text<'a>(q: &'a Value, key: &str) -> Result<&'a str, String> {
    q.get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| format!("Missing {key}"))
}
fn io_error(e: impl std::fmt::Display) -> String {
    e.to_string()
}
fn safe_id(id: &str) -> Result<(), String> {
    if id.is_empty()
        || id.len() > 96
        || !id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
    {
        Err("Invalid identifier".into())
    } else {
        Ok(())
    }
}

impl Service {
    fn new() -> Self {
        let root = std::env::var_os("AEVIA_DATA_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                std::env::var_os("LOCALAPPDATA")
                    .map(PathBuf::from)
                    .or_else(|| {
                        std::env::var_os("HOME").map(|p| PathBuf::from(p).join(".local/share"))
                    })
                    .unwrap_or_else(std::env::temp_dir)
                    .join("Aevia")
            });
        let mut result = Self {
            root,
            preferences: Preferences::default(),
            sessions: vec![],
            port: None,
            port_name: None,
            status: "Disconnected".into(),
            latest: Value::Null,
            last_seen: None,
            last_full_status: None,
            sequence: 0,
            gnss_rate: None,
            imu_rate: None,
            imu_rate_sample: None,
            rate_sample: None,
            capture: None,
            processing: None,
        };
        if let Err(e) = fs::create_dir_all(&result.root) {
            result.status = format!("Storage unavailable: {e}");
        }
        if let Ok(bytes) = fs::read(result.root.join("preferences.json")) {
            if let Ok(prefs) = serde_json::from_slice(&bytes) {
                result.preferences = prefs;
            }
        }
        result.refresh_sessions();
        result
    }
    fn offline_copy(&self) -> Self {
        Self {
            root: self.root.clone(),
            preferences: self.preferences.clone(),
            sessions: self.sessions.clone(),
            port: None,
            port_name: None,
            status: "Offline processing".into(),
            latest: Value::Null,
            last_seen: None,
            last_full_status: None,
            sequence: 0,
            gnss_rate: None,
            imu_rate: None,
            imu_rate_sample: None,
            rate_sample: None,
            capture: None,
            processing: None,
        }
    }
    fn execute(&mut self, q: &Value) -> Result<Value, String> {
        if self.capture.is_some()
            && matches!(
                text(q, "op")?,
                "import" | "demo" | "reanalyze" | "export" | "device_sessions" | "download"
            )
        {
            return Err("Stop recording before importing, processing or exporting sessions".into());
        }
        match text(q, "op")? {
            "state" | "poll" => Ok(self.state()),
            "devices" => devices(),
            "device_sessions" => self.device_sessions(),
            "download" => self.download(q),
            "inspect" => {
                let started = Instant::now();
                let session = crate::legacy::import(Path::new(text(q, "path")?))?;
                let valid: Vec<_> = session.samples.iter().filter(|s| s.valid).collect();
                Ok(
                    json!({"samples":session.samples.len(),"valid":valid.len(),"parse_s":started.elapsed().as_secs_f64(),"duration_s":session.duration_s,
                    "max_speed_mps":valid.iter().filter_map(|s|s.speed_mps).reduce(f64::max),
                    "minimum_speed_mps":valid.iter().filter_map(|s|s.speed_mps).reduce(f64::min),
                    "maximum_hacc_m":valid.iter().filter_map(|s|s.hacc_m).reduce(f64::max),
                    "nonzero_latency":valid.iter().filter(|s|s.velocity_latency_ms.is_some_and(|v|v>0)).count(),
                    "receiver_timed":valid.iter().filter(|s|s.receiver_timed).count(),"legacy":session.raw_records.iter().find(|r|r["kind"]=="legacy_capture")}),
                )
            }
            "connect" => {
                if self.capture.is_some() {
                    return Err("Stop recording before changing devices".into());
                }
                let name = text(q, "port")?;
                let mut port = serialport::new(name, 115200)
                    .timeout(Duration::from_millis(800))
                    .open()
                    .map_err(io_error)?;
                port.write_data_terminal_ready(true).map_err(io_error)?;
                port.write_request_to_send(false).map_err(io_error)?;
                let _ = port.clear(serialport::ClearBuffer::All);
                let status = transact(port.as_mut(), "status")?;
                if status.get("protocol").and_then(Value::as_u64) != Some(1) {
                    return Err(
                        "Unsupported Aevia application protocol; install the matching firmware"
                            .into(),
                    );
                }
                self.port = Some(port);
                self.port_name = Some(name.into());
                self.merge_device_reply(status);
                self.status = "Connected via USB".into();
                Ok(self.state())
            }
            "disconnect" => {
                if self.capture.is_some() {
                    self.stop()?;
                }
                self.port = None;
                self.port_name = None;
                self.status = "Disconnected".into();
                Ok(self.state())
            }
            "start" => {
                self.start(q)?;
                Ok(self.state())
            }
            "stop" => {
                self.stop()?;
                Ok(self.state())
            }
            "sessions" => {
                self.refresh_sessions();
                Ok(json!(self.sessions))
            }
            "session" => {
                let s = self.load(text(q, "id")?)?;
                Ok(json!(s))
            }
            "import" => {
                let s = self.import(Path::new(text(q, "path")?))?;
                Ok(json!(s))
            }
            "export" => self.export(q),
            "ppk_status" => {
                let session = q["id"].as_str().map(|id| self.load(id)).transpose()?;
                Ok(crate::ppk::status(session.as_ref()))
            }
            "prepare_ppk" => crate::ppk::prepare(&self.load(text(q, "id")?)?, q),
            "run_ppk" => {
                if let Some(id) = q["source_id"].as_str() {
                    self.load(id)?;
                }
                let (mut session, mut result) = crate::ppk::run(q)?;
                self.analyze(&mut session)?;
                self.save(&session)?;
                result["session"] = json!(session);
                Ok(result)
            }
            "import_ppk" => {
                if let Some(id) = q["source_id"].as_str() {
                    self.load(id)?;
                }
                let path = Path::new(text(q, "path")?);
                let mut session = crate::ppk::import_pos(path, q["source_id"].as_str())?;
                let copy = self.root.join(format!("{}.pos", session.id));
                copy_evidence(path, &copy)?;
                session.provenance["retained_solution_file"] = json!(copy);
                session.raw_records.push(json!({"kind":"raw_capture_file","path":copy,"format":"RTKLIB-POS","source":"ppk"}));
                self.analyze(&mut session)?;
                self.save(&session)?;
                Ok(json!(session))
            }
            "demo" => {
                let mut s = demo_session();
                self.analyze(&mut s)?;
                self.save(&s)?;
                Ok(json!(s))
            }
            "save_track" => {
                let mut track: Track =
                    serde_json::from_value(q.get("track").cloned().ok_or("Missing track")?)
                        .map_err(io_error)?;
                if track.id.is_empty() {
                    track.id = format!("track-{}", now_ms());
                }
                validate_track(&track)?;
                self.preferences.tracks.retain(|t| t.id != track.id);
                self.preferences.tracks.push(track);
                self.save_preferences()?;
                Ok(self.state())
            }
            "delete_track" => {
                let id = text(q, "id")?;
                self.preferences.tracks.retain(|t| t.id != id);
                self.save_preferences()?;
                Ok(self.state())
            }
            "set_drag" => {
                let config: DragConfig =
                    serde_json::from_value(q.get("config").cloned().ok_or("Missing config")?)
                        .map_err(io_error)?;
                validate_drag(&config)?;
                self.preferences.drag = config;
                self.save_preferences()?;
                Ok(self.state())
            }
            "reanalyze" => {
                let mut s = self.load(text(q, "id")?)?;
                s.track = q
                    .get("track_id")
                    .and_then(Value::as_str)
                    .and_then(|id| self.preferences.tracks.iter().find(|t| t.id == id).cloned());
                s.drag = self.preferences.drag.clone();
                self.analyze(&mut s)?;
                self.save(&s)?;
                Ok(json!(s))
            }
            _ => Err("Unknown operation".into()),
        }
    }
    fn next_collection_query(&self) -> Value {
        if self.latest["base64_poll"] == true
            || self.latest["compact_poll"] == true
            || self.latest["binary_poll"] == true
        {
            if self
                .last_full_status
                .is_none_or(|at| at.elapsed() >= Duration::from_secs(1))
            {
                json!({"op":"status"})
            } else {
                json!({"op":if self.latest["base64_poll"]==true{"drain_base64"}else if self.latest["binary_poll"]==true{"drain_binary"}else{"drain"}})
            }
        } else {
            // Older firmware's full poll remains the compatibility path.
            json!({"op":"poll"})
        }
    }
    fn merge_device_reply(&mut self, mut reply: Value) -> Vec<Value> {
        let now = Instant::now();
        let compact = reply["compact"] == true;
        let fresh_gnss_count = reply["gnss"]["count"].as_u64();
        let fresh_imu_count = reply["imu"]["count"].as_u64();
        let clock_reset = reply["uptime_us"]
            .as_u64()
            .zip(self.latest["uptime_us"].as_u64())
            .is_some_and(|(next, previous)| next < previous);
        let records = reply
            .as_object_mut()
            .and_then(|o| o.remove("samples"))
            .and_then(|v| {
                if let Value::Array(records) = v {
                    Some(records)
                } else {
                    None
                }
            })
            .unwrap_or_default();
        if clock_reset {
            self.latest = Value::Null;
            self.last_full_status = None;
            self.rate_sample = None;
            self.gnss_rate = None;
            self.imu_rate = None;
            self.imu_rate_sample = None;
        }
        if compact {
            if !self.latest.is_object() {
                self.latest = json!({});
            }
            if let Value::Object(reply_fields) = reply {
                let latest_fields = self.latest.as_object_mut().unwrap();
                for (key, value) in reply_fields {
                    match value {
                        Value::Object(sensor) if matches!(key.as_str(), "gnss" | "imu") => {
                            let latest = latest_fields.entry(key).or_insert(Value::Null);
                            if let Some(fields) = latest.as_object_mut() {
                                fields.extend(sensor);
                            } else {
                                *latest = Value::Object(sensor);
                            }
                        }
                        value => {
                            latest_fields.insert(key, value);
                        }
                    }
                }
            }
        } else {
            self.latest = reply;
            self.latest["diagnostics_observed_at_us"] = self.latest["uptime_us"].clone();
            self.last_full_status = Some(now);
        }
        // Queued observations may predate the latest full status. Only advance
        // each sensor once, using its newest observation from the batch.
        let mut newest = [
            (
                "gnss",
                sensor_record_ns(&self.latest["gnss"]["latest"]),
                None,
            ),
            ("imu", sensor_record_ns(&self.latest["imu"]["latest"]), None),
        ];
        for record in &records {
            let (_, timestamp, latest) = match record["kind"].as_str() {
                Some("gnss") => &mut newest[0],
                Some("imu") => &mut newest[1],
                _ => continue,
            };
            let Some(at) = sensor_record_ns(record) else {
                continue;
            };
            if timestamp.is_none_or(|previous| at > previous) {
                *timestamp = Some(at);
                *latest = Some(record);
            }
        }
        for (kind, _, latest) in newest {
            if let Some(record) = latest {
                if !self.latest[kind].is_object() {
                    self.latest[kind] = json!({});
                }
                let mut latest = record.clone();
                if let Some(fields) = latest.as_object_mut() {
                    fields.remove("_wire_kind");
                    fields.remove("_wire_hex");
                }
                self.latest[kind]["latest"] = latest;
            }
            if self.latest[kind].is_object() {
                let age = self.latest["uptime_us"]
                    .as_u64()
                    .zip(self.latest[kind]["latest"]["t_us"].as_u64())
                    .and_then(|(now, at)| now.checked_sub(at));
                self.latest[kind]["age_us"] = json!(age);
            }
        }
        self.last_seen = Some(now);
        // Drain replies omit counters. Reusing a merged count here would
        // repeatedly manufacture zero-Hz intervals between full status reads.
        if let Some(count) = fresh_gnss_count {
            match self.rate_sample {
                Some((time, previous))
                    if count >= previous && now.duration_since(time) >= Duration::from_secs(1) =>
                {
                    self.gnss_rate =
                        Some((count - previous) as f64 / now.duration_since(time).as_secs_f64());
                    self.rate_sample = Some((now, count));
                }
                Some((_, previous)) if count >= previous => {}
                _ => {
                    self.gnss_rate = None;
                    self.rate_sample = Some((now, count));
                }
            }
        }
        if let Some(count) = fresh_imu_count {
            match self.imu_rate_sample {
                Some((time, previous))
                    if count >= previous && now.duration_since(time) >= Duration::from_secs(1) =>
                {
                    self.imu_rate =
                        Some((count - previous) as f64 / now.duration_since(time).as_secs_f64());
                    self.imu_rate_sample = Some((now, count));
                }
                Some((_, previous)) if count >= previous => {}
                _ => {
                    self.imu_rate = None;
                    self.imu_rate_sample = Some((now, count));
                }
            }
        }
        records
    }
    fn sensor_age_us(&self, kind: &str) -> Option<u64> {
        self.latest[kind]["age_us"].as_u64().map(|age| {
            age.saturating_add(
                self.last_seen
                    .map(|at| at.elapsed().as_micros().min(u128::from(u64::MAX)) as u64)
                    .unwrap_or(0),
            )
        })
    }
    fn state(&self) -> Value {
        let mut gnss_health = self.latest["gnss"].clone();
        let mut imu_health = self.latest["imu"].clone();
        if gnss_health.is_object() {
            gnss_health["age_us"] = json!(self.sensor_age_us("gnss"));
        }
        if imu_health.is_object() {
            imu_health["age_us"] = json!(self.sensor_age_us("imu"));
        }
        let mut ppk_health = self.latest["ppk"].clone();
        if ppk_health.is_object() {
            let age = self.latest["uptime_us"]
                .as_u64()
                .zip(ppk_health["last_observation_us"].as_u64())
                .and_then(|(now, at)| now.checked_sub(at))
                .map(|age| {
                    age.saturating_add(
                        self.last_seen
                            .map(|t| t.elapsed().as_micros().min(u128::from(u64::MAX)) as u64)
                            .unwrap_or(0),
                    ) / 1000
                });
            ppk_health["age_ms"] = json!(age);
        }
        let gnss_health = &gnss_health;
        let gnss = gnss_health.get("latest").unwrap_or(gnss_health);
        let stale = self.port.is_none()
            || self
                .last_seen
                .is_none_or(|t| t.elapsed() > Duration::from_secs(2));
        let age_us = self.sensor_age_us("gnss").unwrap_or(u64::MAX);
        let diagnostics_stale = self
            .last_full_status
            .is_none_or(|t| t.elapsed() > Duration::from_secs(2));
        let valid = !stale
            && age_us < 1_000_000
            && gnss.get("valid").and_then(Value::as_bool) == Some(true);
        let elapsed = self
            .capture
            .as_ref()
            .map(|c| c.started.elapsed().as_secs_f64())
            .unwrap_or(0.0);
        let analysis = self.capture.as_ref().map(|c| &c.session.analysis);
        let completed = analysis.map(|a| a.laps.len()).unwrap_or(0);
        let last_crossing = analysis.and_then(|a| a.laps.last()).map(|lap| lap.end_s);
        let best = analysis.and_then(|a| a.best_lap_s);
        let metric_elapsed = self
            .capture
            .as_ref()
            .and_then(|c| c.session.samples.last())
            .map(|s| s.time_s);
        json!({"device":{"connected":self.port.is_some(),"port":self.port_name,"status":self.status,
            "recording":self.latest.get("recording"),
            "firmware":self.latest.get("firmware").or_else(||self.latest.get("protocol")),"battery_percent":if stale||diagnostics_stale{None}else{self.latest["battery"].get("percent")},
            "diagnostics_stale":diagnostics_stale,"diagnostics_age_ms":self.last_full_status.map(|t|t.elapsed().as_millis() as u64),
            "gnss_hz":if stale||diagnostics_stale{None}else{self.gnss_rate},"imu_hz":if stale||diagnostics_stale{None}else{self.imu_rate},
            "acquisition_profile":self.latest.get("acquisition_profile"),"ppk":ppk_health,"satellites":gnss.get("satellites"),
            "fix_quality":if stale {"stale"} else if valid {"valid"} else {"no fix"},
            "dropped":self.latest.get("dropped"),"usb_queue":self.latest.get("usb_queue"),"imu":imu_health,"gnss":gnss_health,"gnss_uart":self.latest.get("gnss_uart"),"gnss_byte_queue":self.latest.get("gnss_byte_queue"),
            "imu_status":self.latest["imu"].get("status"),"storage":self.latest.get("storage"),
            "storage_status":self.latest.get("storage"),"sd":self.latest.get("sd"),"battery":self.latest.get("battery"),"bluetooth":"unavailable",
            "navigation":self.latest.get("navigation"),"installation":self.latest.get("installation"),"capture_timing":self.latest.get("capture_timing"),
            "imu_diagnostics":self.latest.get("imu_diagnostics"),
            "led":self.latest.get("led"),"display":self.latest.get("display"),"buttons":self.latest.get("buttons"),"accessories":self.latest.get("accessories")},
            "live":{"time_s":number(&self.latest,"uptime_us").map(|v|v/1e6),
                "speed_mps":if valid {number(gnss,"speed_mps")}else{None},
                "lat":if valid{number(gnss,"lat")}else{None},"lon":if valid{number(gnss,"lon")}else{None},
                "altitude_m":if valid{number(gnss,"alt_m")}else{None},"heading_deg":if valid{number(gnss,"heading_deg")}else{None},
                "accel_long_g":null,"accel_lat_g":null,"recording":self.capture.is_some(),"elapsed_s":elapsed,
                "capture_interrupted":self.capture.as_ref().is_some_and(|c|c.interrupted),
                "capture_error":self.capture.as_ref().and_then(|c|c.session.incomplete_reason.as_ref()),
                "lap":completed,"current_lap_s":last_crossing.zip(metric_elapsed).map(|(t,now)|(now-t).max(0.0)),"best_lap_s":best,"delta_s":null,"valid":valid,"stale":stale,
                "analysis":analysis,"analysis_samples":self.capture.as_ref().map(|c|c.analyzed_samples),
                "age_ms":if age_us==u64::MAX{None}else{Some(age_us/1000)},"sequence":self.sequence},
            "sessions":self.sessions,"tracks":self.preferences.tracks,"drag":self.preferences.drag,
            "storage_path":self.root.to_string_lossy(),"processing":self.processing})
    }
    fn start(&mut self, q: &Value) -> Result<(), String> {
        if self.capture.is_some() {
            return Err("Already recording".into());
        }
        let port = self
            .port
            .as_mut()
            .ok_or("Connect a V2 Mini before recording")?;
        let id = format!("session-{}", now_ms());
        let name = q
            .get("name")
            .and_then(Value::as_str)
            .filter(|s| !s.trim().is_empty())
            .unwrap_or("Track session")
            .to_owned();
        if name.len() > 128 {
            return Err("Session names must be at most 128 bytes".into());
        }
        let mut session = Session {
            format_version: 1,
            id: id.clone(),
            name,
            created_ms: now_ms(),
            duration_s: 0.0,
            sample_count: 0,
            max_speed_mps: None,
            distance_m: 0.0,
            source: "V2 Mini USB · receiver observations".into(),
            incomplete_reason: None,
            provenance: Value::Null,
            samples: vec![],
            raw_records: vec![],
            track: q
                .get("track_id")
                .and_then(Value::as_str)
                .and_then(|id| self.preferences.tracks.iter().find(|t| t.id == id).cloned()),
            drag: self.preferences.drag.clone(),
            analysis: Analysis::default(),
        };
        let evidence = if self.latest["base64_poll"] == true || self.latest["binary_poll"] == true {
            let path = self.root.join(format!("{id}.raw.aev"));
            let writer = crate::evidence::Writer::create(&path, now_ms(), MAX_CAPTURE_BYTES)?;
            session.raw_records.push(json!({"kind":"raw_capture_file","path":path.to_string_lossy(),"format":"AEVLOG1","source":"usb-binary"}));
            Some(writer)
        } else {
            None
        };
        let mut journal = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(self.root.join(format!("{id}.capture.jsonl")))
            .map_err(io_error)?;
        writeln!(journal, "{}", json!({"session":session})).map_err(io_error)?;
        journal.sync_data().map_err(io_error)?;
        let response = transact(port.as_mut(), "start")?;
        self.merge_device_reply(response);
        self.capture = Some(Capture {
            session,
            journal,
            journal_line: Vec::new(),
            clock: SampleClock::default(),
            last_sync: Instant::now(),
            last_dropped: 0,
            analyzed_samples: 0,
            interrupted: false,
            started: Instant::now(),
            arrival_origin_us: None,
            stop_at_us: None,
            data_bytes: 0,
            health_start: capture_health_snapshot(&self.latest),
            health_latest: capture_health_snapshot(&self.latest),
            health_counter_resets: vec![],
            stop_boundary_confirmed: false,
            stop_tail_pending: None,
            stop_tail_received: 0,
            evidence,
            binary_imu_count: 0,
            binary_imu_interval_ns: 0,
            binary_pre_start_count: 0,
            binary_first_imu: None,
            binary_last_imu: None,
        });
        if let Some(capture) = self.capture.as_mut() {
            if let Some(writer) = capture.evidence.as_mut() {
                let start = capture.health_start["started_at_us"]
                    .as_u64()
                    .or_else(|| capture.health_start["observed_at_us"].as_u64())
                    .unwrap_or(0);
                let metadata=serde_json::to_vec(&json!({"schema":"aevia-usb-binary-v1","source":"usb","qualified":false,"started_at_us":start})).map_err(io_error)?;
                if let Err(error) = writer.append(1, start.saturating_mul(1000), &metadata) {
                    interrupt_capture(capture, &error);
                    return Err(error);
                }
            }
            let start = json!({"kind":"capture_start","schema":1,"health":capture.health_start});
            if let Err(error) = journal_record(capture, &start) {
                interrupt_capture(capture, &error);
                return Err(error);
            }
            capture.session.raw_records.push(start);
        }
        self.status = "Recording to computer".into();
        Ok(())
    }
    fn stop(&mut self) -> Result<(), String> {
        let Some(capture) = self.stop_acquisition()? else {
            return Ok(());
        };
        self.finalize_capture(capture)
    }
    fn stop_acquisition(&mut self) -> Result<Option<Capture>, String> {
        if self.capture.is_none() {
            return Ok(None);
        }
        if let Some(port) = self.port.as_mut() {
            let pending_tail;
            match transact(port.as_mut(), "stop") {
                Ok(value) => {
                    pending_tail = value.get("pending").and_then(Value::as_u64);
                    if let Some(c) = self.capture.as_mut() {
                        if c.stop_at_us.is_none() {
                            observe_capture_health(c, &value);
                        }
                        c.stop_at_us = stopped_at_us(&value);
                        c.stop_boundary_confirmed = value["stopped_at_us"].is_u64();
                        c.stop_tail_pending = pending_tail;
                    }
                    self.merge_device_reply(value);
                }
                Err(e) => {
                    self.status = format!(
                        "Device has not confirmed stop: {e}. Received data remains in the journal."
                    );
                    return Err(self.status.clone());
                }
            }
            // The stop fence has queued every pre-stop publication. Drain that
            // fixed FIFO tail, not an ever-refilling live preview queue. Older
            // replies without pending retain the bounded empty-queue fallback.
            if !self.collect_for_tail(Duration::from_secs(12), pending_tail) {
                if let Some(capture) = self.capture.as_mut() {
                    interrupt_capture(capture, "The stopped USB queue could not be fully drained; received evidence and the recovery journal are retained");
                }
            }
        }
        if let Some(capture) = &self.capture {
            capture.journal.sync_all().map_err(io_error)?;
        }
        Ok(self.capture.take())
    }
    fn finalize_capture(&mut self, mut capture: Capture) -> Result<(), String> {
        if let Some(evidence) = capture.evidence.as_mut() {
            evidence.close(
                capture.stop_at_us.unwrap_or(0).saturating_mul(1000),
                capture.last_dropped.min(u64::from(u32::MAX)) as u32,
            )?;
        }
        if capture.binary_imu_count > 0 {
            capture.session.raw_records.push(json!({"kind":"capture_timing_summary","schema":1,
                "timed_imu_records":capture.binary_imu_count,"first_imu":capture.binary_first_imu,
                "last_imu":capture.binary_last_imu,"per_sample_evidence":"Retained USB AEVLOG1 sidecar contains every binary sensor payload."}));
        }
        let health = capture_health_record(&capture);
        // Preserve lifecycle metadata in recovery evidence as well as canonical
        // storage. This bypasses the sensor stop filter intentionally.
        journal_record(&mut capture, &health)?;
        capture.journal.sync_all().map_err(io_error)?;
        capture.session.raw_records.push(health);
        if capture.interrupted {
            capture.session.raw_records.push(json!({"kind":"raw_capture_file",
                "path":format!("{}.capture.jsonl", capture.session.id),
                "reason":"Interrupted capture: complete received raw evidence is retained in this journal."}));
        }
        capture.session.analysis.warnings.push(
            "Final session processing has not completed; reanalyze this recording to finish."
                .into(),
        );
        self.save(&capture.session)?;
        self.analyze(&mut capture.session)?;
        let result = self.save(&capture.session);
        drop(capture.journal);
        if result.is_ok() && !capture.interrupted {
            let _ = fs::remove_file(
                self.root
                    .join(format!("{}.capture.jsonl", capture.session.id)),
            );
        }
        self.status = if capture.interrupted {
            "Incomplete recording saved; capture journal retained".into()
        } else if self.port.is_some() {
            "Connected via USB".into()
        } else {
            "Disconnected; recording saved".into()
        };
        result
    }
    fn collect(&mut self) {
        self.collect_for(COLLECT_TURN_BUDGET);
    }
    fn next_collection_delay(&self, received_successfully: bool) -> Duration {
        if received_successfully
            && self.latest["pending"].as_u64().is_some_and(|n| n > 0)
            && !self
                .capture
                .as_ref()
                .is_some_and(|capture| capture.interrupted)
        {
            Duration::from_millis(2)
        } else {
            Duration::from_millis(40)
        }
    }
    fn collect_for(&mut self, budget: Duration) -> bool {
        self.collect_for_tail(budget, None)
    }
    fn collect_for_tail(&mut self, budget: Duration, tail_records: Option<u64>) -> bool {
        if tail_records == Some(0) {
            return true;
        }
        if self.port.is_none() || self.capture.as_ref().is_some_and(|c| c.interrupted) {
            return false;
        }
        self.collect_with_tail(budget, tail_records, |service| {
            let query = service.next_collection_query();
            transact_query(service.port.as_mut().unwrap().as_mut(), &query)
        })
    }
    #[cfg(test)]
    fn collect_with(
        &mut self,
        budget: Duration,
        poll: impl FnMut(&mut Self) -> Result<Value, String>,
    ) -> bool {
        self.collect_with_tail(budget, None, poll)
    }
    fn collect_with_tail(
        &mut self,
        budget: Duration,
        tail_records: Option<u64>,
        mut poll: impl FnMut(&mut Self) -> Result<Value, String>,
    ) -> bool {
        if tail_records == Some(0) {
            return true;
        }
        let started = Instant::now();
        let mut remaining_tail = tail_records;
        for batch in 0..64 {
            // Finish an in-flight reply, then yield before starting another.
            // Serial transaction timeout remains the bound on one slow reply.
            if batch > 0 && started.elapsed() >= budget {
                break;
            }
            let result = poll(self);
            match result {
                Ok(value) => {
                    let pending = value.get("pending").and_then(Value::as_u64).unwrap_or(0);
                    let records = self.merge_device_reply(value);
                    if let Some(remaining) = &mut remaining_tail {
                        let received = (*remaining).min(records.len() as u64);
                        *remaining -= received;
                        if let Some(capture) = self.capture.as_mut() {
                            capture.stop_tail_received += received;
                        }
                    }
                    self.sequence += 1;
                    if let Some(capture) = self.capture.as_mut() {
                        if capture.stop_at_us.is_none() {
                            observe_capture_health(capture, &self.latest);
                        }
                        // A button stop can precede the host's Stop & Save.
                        // Apply its stable boundary before accepting this batch.
                        if let Some(end) = self.latest.get("stopped_at_us").and_then(Value::as_u64)
                        {
                            capture.stop_at_us = Some(end);
                            capture.stop_boundary_confirmed = true;
                        }
                        let dropped = self
                            .latest
                            .get("dropped")
                            .and_then(Value::as_u64)
                            .unwrap_or(0);
                        if dropped != capture.last_dropped {
                            capture.last_dropped = dropped;
                            let health = json!({"kind":"health","dropped":dropped,"t_us":self.latest.get("uptime_us")});
                            if let Err(e) = append_record(capture, health) {
                                self.status = format!("Capture journal write failed: {e}");
                                interrupt_capture(capture, &e);
                            }
                        }
                    }
                    if let Some(capture) = self.capture.as_mut() {
                        for record in records {
                            // A poll already removed this complete batch from the
                            // device. Preserve its remainder even after a capacity
                            // boundary; never silently discard received evidence.
                            let result = if capture.interrupted {
                                journal_record(capture, &record).map(|_| ())
                            } else {
                                append_record(capture, record)
                            };
                            if let Err(e) = result {
                                self.status = format!(
                                    "Recording interrupted: {e}. Stop to save received data."
                                );
                                interrupt_capture(capture, &e);
                            }
                        }
                        if let Some(c) = self.capture.as_mut() {
                            if c.interrupted || c.last_sync.elapsed() > Duration::from_secs(1) {
                                if let Some(evidence) = c.evidence.as_mut() {
                                    if let Err(e) = evidence.sync() {
                                        interrupt_capture(c, &e);
                                        self.status = format!("Binary evidence sync failed: {e}");
                                        break;
                                    }
                                }
                                if let Err(e) = c.journal.sync_data() {
                                    self.status = format!("Capture journal sync failed: {e}");
                                    interrupt_capture(c, &e.to_string());
                                    break;
                                }
                                c.last_sync = Instant::now();
                            }
                        }
                    }
                    if remaining_tail == Some(0) || (remaining_tail.is_none() && pending == 0) {
                        return true;
                    }
                    if self.capture.as_ref().is_some_and(|c| c.interrupted) {
                        break;
                    }
                }
                Err(error) => {
                    self.port = None;
                    self.status =
                        format!("Connection lost: {error}. Stop to save received recording.");
                    if let Some(capture) = self.capture.as_mut() {
                        interrupt_capture(capture, &format!("USB connection lost: {error}"));
                    }
                    break;
                }
            }
        }
        false
    }
    fn analyze(&self, s: &mut Session) -> Result<(), String> {
        if processing_cancelled() {
            return Err("Analysis cancelled".into());
        }
        s.analysis = if s.provenance["kind"] == "ppk" {
            Analysis {
                engine: "RTKLIB ? corrected positions".into(),
                valid_samples: s.samples.iter().filter(|sample| sample.valid).count(),
                ..Analysis::default()
            }
        } else {
            crate::trajectory_adapter::analyze(
                &s.samples,
                &s.raw_records,
                s.track.as_ref(),
                &s.drag,
            )
        };
        if let Some(reason) = &s.incomplete_reason {
            s.analysis
                .warnings
                .push(format!("Incomplete recording: {reason}"));
        }
        add_capture_health_warnings(&s.raw_records, &mut s.analysis.warnings);
        crate::legacy::add_timing_warnings(&s.raw_records, &mut s.analysis.warnings);
        if s.raw_records
            .iter()
            .any(|record| record["kind"] == "legacy_capture")
        {
            match crate::legacy::transport_loss_summary(&s.raw_records) {
                Ok(summary) => {
                    crate::legacy::add_transport_warnings(&summary, &mut s.analysis.warnings)
                }
                Err(error) => s.analysis.warnings.push(format!(
                    "GNSS transport-loss metadata could not be validated: {error}"
                )),
            }
        }
        if let Some(warnings) = s.provenance["warnings"].as_array() {
            for warning in warnings.iter().filter_map(Value::as_str) {
                s.analysis.warnings.push(warning.to_owned());
            }
        }
        for record in &s.raw_records {
            if record["kind"] == "missing_raw_evidence" {
                s.analysis.warnings.push(format!("Referenced raw evidence is unavailable: {}. This session preserves projected samples only.",record["path"].as_str().unwrap_or("unknown")));
            }
        }
        s.sample_count = s.samples.len();
        s.duration_s = s
            .duration_s
            .max(s.samples.last().map(|s| s.time_s).unwrap_or(0.0));
        s.max_speed_mps = s.analysis.max_speed_mps;
        s.distance_m = s.analysis.distance_m;
        if processing_cancelled() {
            return Err("Analysis cancelled".into());
        }
        Ok(())
    }
    fn load(&self, id: &str) -> Result<Session, String> {
        safe_id(id)?;
        let path = self.root.join(format!("{id}.json"));
        read_session(&path)
    }
    fn save(&mut self, s: &Session) -> Result<(), String> {
        self.save_with_limit(s, MAX_IMPORT_BYTES)
    }
    fn save_with_limit(&mut self, s: &Session, max_bytes: u64) -> Result<(), String> {
        safe_id(&s.id)?;
        validate_session(s)?;
        write_json_with_limit(
            &self.root.join(format!("{}.json", s.id)),
            s,
            max_bytes,
            |path| read_session(path).map(|_| ()),
        )?;
        self.refresh_sessions();
        Ok(())
    }
    fn save_preferences(&self) -> Result<(), String> {
        write_json(&self.root.join("preferences.json"), &self.preferences)
    }
    fn refresh_sessions(&mut self) {
        self.sessions = fs::read_dir(&self.root)
            .into_iter()
            .flatten()
            .filter_map(Result::ok)
            .filter_map(|entry| {
                let path = entry.path();
                if path.extension()?.to_str()? != "json"
                    || path.file_name()?.to_str()? == "preferences.json"
                {
                    return None;
                }
                read_session(&path).ok().map(|s| summary(&s))
            })
            .collect();
        self.sessions
            .sort_by_key(|s| std::cmp::Reverse(s["created_ms"].as_u64().unwrap_or(0)));
    }
    fn import(&mut self, path: &Path) -> Result<Session, String> {
        let size = fs::metadata(path).map_err(io_error)?.len();
        let extension = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_lowercase();
        let limit = if extension == "aev" {
            MAX_CAPTURE_BYTES
        } else {
            MAX_IMPORT_BYTES
        };
        if size > limit {
            return Err(format!("Import exceeds {} MiB", limit / 1024 / 1024));
        }
        let mut session = if extension == "aev" {
            crate::legacy::import(path)?
        } else if extension == "csv" {
            import_csv(&fs::read_to_string(path).map_err(io_error)?)?
        } else if extension == "jsonl" {
            recover_journal(&fs::read_to_string(path).map_err(io_error)?)?
        } else {
            read_session(path)?
        };
        validate_session(&session)?;
        session.id = format!("import-{}", now_ms());
        if extension == "aev"
            && path.parent().and_then(|p| p.canonicalize().ok()) == self.root.canonicalize().ok()
        {
            if let Some(id) = path
                .file_stem()
                .and_then(|v| v.to_str())
                .filter(|id| id.starts_with("import-") && safe_id(id).is_ok())
            {
                if !self.root.join(format!("{id}.json")).exists() {
                    session.id = id.into();
                }
            }
        }
        session.created_ms = now_ms();
        session.source = format!(
            "Imported {}",
            path.file_name().unwrap_or_default().to_string_lossy()
        );
        if extension == "aev" {
            let destination = self.root.join(format!("{}.aev", session.id));
            if path.canonicalize().ok() != destination.canonicalize().ok() {
                fs::copy(path, &destination).map_err(io_error)?;
            }
            session
                .raw_records
                .push(json!({"kind":"raw_capture_file","path":destination.to_string_lossy()}));
        }
        if extension != "aev" {
            let mut missing = Vec::new();
            for (index, record) in session
                .raw_records
                .iter_mut()
                .filter(|r| r["kind"] == "raw_capture_file")
                .enumerate()
            {
                let Some(reference) = record["path"].as_str() else {
                    continue;
                };
                let referenced = Path::new(reference);
                let source = if referenced.is_absolute() {
                    referenced.to_owned()
                } else {
                    path.parent().unwrap_or(Path::new(".")).join(referenced)
                };
                if !source.is_file() {
                    missing.push(json!({"kind":"missing_raw_evidence","path":source}));
                    continue;
                }
                let ext = source.extension().and_then(|v| v.to_str()).unwrap_or("raw");
                let dest = self.root.join(format!("{}.raw-{index}.{ext}", session.id));
                copy_evidence(&source, &dest)?;
                record["path"] = json!(dest);
            }
            session.raw_records.extend(missing);
        }
        self.analyze(&mut session)?;
        self.save(&session)?;
        Ok(session)
    }
    fn export(&self, q: &Value) -> Result<Value, String> {
        let mut s = self.load(text(q, "id")?)?;
        let path = Path::new(text(q, "path")?);
        let format = q.get("format").and_then(Value::as_str).unwrap_or("csv");
        if path.exists() {
            return Err("Export destination exists; choose a new filename".into());
        }
        let mut paired_files = Vec::new();
        if format == "json" {
            // Copy complete raw evidence beside portable JSON and use sibling
            // names; importing then takes its own copy into managed storage.
            for (index, record) in s
                .raw_records
                .iter_mut()
                .filter(|r| r["kind"] == "raw_capture_file")
                .enumerate()
            {
                let source = Path::new(record["path"].as_str().ok_or("Missing raw evidence path")?);
                let source = if source.is_absolute() {
                    source.to_owned()
                } else {
                    self.root.join(source)
                };
                let ext = source.extension().and_then(|v| v.to_str()).unwrap_or("raw");
                let name = format!(
                    "{}.raw-{index}.{ext}",
                    path.file_stem().unwrap_or_default().to_string_lossy()
                );
                let dest = path.with_file_name(&name);
                copy_evidence(&source, &dest)?;
                paired_files.push(dest);
                record["path"] = json!(name);
            }
        }
        let content = match format {
            "csv" => export_csv(&s),
            "gpx" => export_gpx(&s),
            // Use the canonical representation so an export of a supported
            // session does not exceed the import ceiling through whitespace.
            "json" => serde_json::to_string(&s).map_err(io_error)?,
            _ => return Err("Export format must be csv, gpx or json".into()),
        };
        // Never overwrite the application's canonical session with an export.
        if path == self.root.join(format!("{}.json", s.id)) {
            return Err("Choose an export destination outside the session file".into());
        }
        let mut file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(path)
            .map_err(|e| format!("Cannot create export (choose a new filename): {e}"))?;
        file.write_all(content.as_bytes()).map_err(io_error)?;
        file.sync_all().map_err(io_error)?;
        Ok(
            json!({"path":path.to_string_lossy(),"samples":s.samples.len(),"format":format,"paired_files":paired_files,"warnings":if format=="json"{vec![]}else{vec!["CSV/GPX exports contain projected GNSS data only. Use JSON and its paired files to retain full raw evidence."]}}),
        )
    }
    fn device_sessions(&mut self) -> Result<Value, String> {
        let port = self.port.as_mut().ok_or("Connect a V2 Mini first")?;
        let mut files = vec![];
        let mut after = 0u64;
        for _ in 0..1000 {
            let response = transact_query(port.as_mut(), &json!({"op":"files","after":after}))?;
            let Some(file) = response.get("file").filter(|v| !v.is_null()) else {
                return Ok(json!(files));
            };
            let id = file["id"]
                .as_u64()
                .ok_or("Invalid device file identifier")?;
            if id <= after {
                return Err("Device file listing did not advance".into());
            }
            after = id;
            files.push(file.clone());
        }
        Err("Device file listing exceeds 1000 sessions".into())
    }
    fn download(&mut self, q: &Value) -> Result<Value, String> {
        let id = q
            .get("id")
            .and_then(Value::as_u64)
            .ok_or("Missing numeric device file id")?;
        let list = self.device_sessions()?;
        let entry = list
            .as_array()
            .and_then(|a| a.iter().find(|f| f["id"].as_u64() == Some(id)))
            .ok_or("Device session not found")?;
        let bytes = entry["bytes"].as_u64().ok_or("Missing device file size")?;
        if bytes == 0 || bytes % 512 != 0 || bytes > MAX_CAPTURE_BYTES {
            return Err("Invalid or oversized device recording".into());
        }
        let path = self.root.join(format!("download-{id}-{}.aev", now_ms()));
        let mut file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&path)
            .map_err(io_error)?;
        let port = self.port.as_mut().ok_or("Device disconnected")?;
        for sector in 0..bytes / 512 {
            let response =
                transact_query(port.as_mut(), &json!({"op":"read","id":id,"sector":sector}))?;
            if response["id"].as_u64() != Some(id) || response["sector"].as_u64() != Some(sector) {
                return Err("Device returned a different file or sector".into());
            }
            let hex = response["hex"].as_str().ok_or("Missing sector bytes")?;
            if hex.len() != 1024 {
                return Err("Device sector has an invalid length".into());
            }
            let mut sector_bytes = [0u8; 512];
            for (i, pair) in hex.as_bytes().as_chunks::<2>().0.iter().enumerate() {
                let nibble = |b: u8| -> Result<u8, String> {
                    match b {
                        b'0'..=b'9' => Ok(b - b'0'),
                        b'a'..=b'f' => Ok(b - b'a' + 10),
                        b'A'..=b'F' => Ok(b - b'A' + 10),
                        _ => Err("Device sent non-hex data".into()),
                    }
                };
                sector_bytes[i] = (nibble(pair[0])? << 4) | nibble(pair[1])?;
            }
            file.write_all(&sector_bytes).map_err(io_error)?;
        }
        file.sync_all().map_err(io_error)?;
        drop(file);
        let session = self.import(&path)?;
        let _ = fs::remove_file(path);
        Ok(json!(session))
    }
}

fn copy_evidence(source: &Path, destination: &Path) -> Result<(), String> {
    let mut input =
        File::open(source).map_err(|e| format!("Raw evidence {}: {e}", source.display()))?;
    if input.metadata().map_err(io_error)?.len() > MAX_CAPTURE_BYTES {
        return Err("Raw evidence exceeds 8 GiB".into());
    }
    let mut output = OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(destination)
        .map_err(io_error)?;
    let mut buffer = [0; 64 * 1024];
    loop {
        if processing_cancelled() {
            return Err("Raw copy cancelled; partial destination retained".into());
        }
        let count = std::io::Read::read(&mut input, &mut buffer).map_err(io_error)?;
        if count == 0 {
            break;
        }
        output.write_all(&buffer[..count]).map_err(io_error)?;
    }
    output.sync_all().map_err(io_error)
}
fn summary(s: &Session) -> Value {
    json!({"id":s.id,"name":s.name,"created_ms":s.created_ms,"duration_s":s.duration_s,"sample_count":s.sample_count,"max_speed_mps":s.max_speed_mps,"distance_m":s.distance_m,"source":s.source,"provenance":s.provenance,"incomplete_reason":s.incomplete_reason,"lap_count":s.analysis.laps.len(),"best_lap_s":s.analysis.best_lap_s})
}
fn write_json(path: &Path, value: &impl serde::Serialize) -> Result<(), String> {
    write_json_with_limit(path, value, MAX_IMPORT_BYTES, |_| Ok(()))
}
struct LimitedWriter<W> {
    inner: W,
    written: u64,
    limit: u64,
}
impl<W: Write> Write for LimitedWriter<W> {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        if bytes.len() as u64 > self.limit.saturating_sub(self.written) {
            return Err(std::io::Error::other(format!(
                "Session exceeds {} byte storage limit; existing files and capture journal are retained",
                self.limit
            )));
        }
        let count = self.inner.write(bytes)?;
        self.written += count as u64;
        Ok(count)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}
fn write_json_with_limit(
    path: &Path,
    value: &impl serde::Serialize,
    limit: u64,
    verify: impl FnOnce(&Path) -> Result<(), String>,
) -> Result<(), String> {
    let temp = path.with_extension("tmp");
    let write_result = (|| {
        let file = File::create(&temp).map_err(io_error)?;
        let mut writer = LimitedWriter {
            inner: BufWriter::new(file),
            written: 0,
            limit,
        };
        serde_json::to_writer(&mut writer, value).map_err(io_error)?;
        writer.flush().map_err(io_error)?;
        writer.inner.get_ref().sync_all().map_err(io_error)
    })();
    if let Err(error) = write_result.and_then(|()| verify(&temp)) {
        let _ = fs::remove_file(&temp);
        return Err(error);
    }
    // Windows rename cannot replace an existing destination. Keep a recoverable
    // prior copy until the new complete file has moved into place.
    let backup = path.with_extension("bak");
    if path.exists() {
        if backup.exists() {
            fs::remove_file(&backup).map_err(io_error)?;
        }
        fs::rename(path, &backup).map_err(io_error)?;
    }
    if let Err(e) = fs::rename(&temp, path) {
        if backup.exists() {
            let _ = fs::rename(&backup, path);
        }
        return Err(io_error(e));
    }
    if backup.exists() {
        let _ = fs::remove_file(backup);
    }
    Ok(())
}
fn read_session(path: &Path) -> Result<Session, String> {
    if fs::metadata(path).map_err(io_error)?.len() > MAX_IMPORT_BYTES {
        return Err("Session exceeds 256 MiB".into());
    }
    let session: Session = serde_json::from_slice(&fs::read(path).map_err(io_error)?)
        .map_err(|e| format!("Not an Aevia session JSON: {e}"))?;
    validate_session(&session)?;
    Ok(session)
}
fn validate_session(s: &Session) -> Result<(), String> {
    if s.format_version != 1 || s.samples.len() > MAX_SAMPLES {
        return Err("Unsupported or oversized session".into());
    }
    let mut previous = None;
    for sample in &s.samples {
        if !sample.time_s.is_finite()
            || sample.time_s < 0.0
            || previous.is_some_and(|v| sample.time_s <= v)
        {
            return Err("Session timestamps must be finite and strictly increasing".into());
        }
        if sample
            .lat
            .is_some_and(|v| !v.is_finite() || !(-90.0..=90.0).contains(&v))
            || sample
                .lon
                .is_some_and(|v| !v.is_finite() || !(-180.0..=180.0).contains(&v))
            || sample
                .speed_mps
                .is_some_and(|v| !v.is_finite() || !(0.0..=500.0).contains(&v))
        {
            return Err("Invalid position or speed in session".into());
        }
        previous = Some(sample.time_s);
    }
    if let Some(track) = &s.track {
        validate_track(track)?;
    }
    validate_drag(&s.drag)
}
fn validate_track(t: &Track) -> Result<(), String> {
    safe_id(&t.id)?;
    if t.name.trim().is_empty() || t.name.len() > 128 || t.sectors.len() > 15 {
        return Err("Track needs a name and at most 15 sectors".into());
    }
    for gate in std::iter::once(&t.start).chain(&t.sectors) {
        if !gate.lat.is_finite()
            || !(-90.0..=90.0).contains(&gate.lat)
            || !gate.lon.is_finite()
            || !(-180.0..=180.0).contains(&gate.lon)
            || !gate.heading_deg.is_finite()
            || !gate.width_m.is_finite()
            || !(1.0..=500.0).contains(&gate.width_m)
        {
            return Err("Invalid gate coordinates, heading or width (1–500 m)".into());
        }
    }
    Ok(())
}
fn validate_drag(d: &DragConfig) -> Result<(), String> {
    if !d.start_kph.is_finite()
        || !d.end_kph.is_finite()
        || !d.distance_m.is_finite()
        || d.start_kph < 0.0
        || d.end_kph <= d.start_kph
        || d.end_kph > 1000.0
        || !(1.0..=10000.0).contains(&d.distance_m)
    {
        Err("Drag test needs increasing speeds (0–1000 km/h) and distance 1–10000 m".into())
    } else {
        Ok(())
    }
}

#[derive(Default)]
pub(crate) struct SampleClock {
    origin: Option<u64>,
    previous: Option<u64>,
    receiver: bool,
    gap: bool,
}
impl SampleClock {
    pub(crate) fn sample(&mut self, record: &Value) -> Result<Option<Sample>, String> {
        if record.get("kind").and_then(Value::as_str) != Some("gnss") {
            return Ok(None);
        }
        let gps_week = record
            .get("gps_week")
            .and_then(Value::as_u64)
            .filter(|v| *v > 0 && *v <= u32::MAX as u64);
        let gps_tow = record
            .get("gps_tow_ms")
            .and_then(Value::as_u64)
            .filter(|v| *v < 604_800_000);
        let epoch = gps_week
            .zip(gps_tow)
            .and_then(|(w, t)| w.checked_mul(604_800_000)?.checked_add(t));
        if self.origin.is_none() && epoch.is_none() && record.get("gps_week").is_some() {
            self.gap = true;
            return Ok(None);
        }
        if self.origin.is_none() {
            self.receiver = epoch.is_some();
        }
        let current = if self.receiver {
            epoch
        } else {
            record.get("t_us").and_then(Value::as_u64)
        };
        let Some(current) = current else {
            self.gap = true;
            return Ok(None);
        };
        if let Some(previous) = self.previous {
            if current == previous {
                return Ok(None);
            }
            if current < previous {
                return Err(
                    "GNSS clock moved backwards; raw capture preserved without timestamp repair"
                        .into(),
                );
            }
        }
        self.previous = Some(current);
        let origin = *self.origin.get_or_insert(current);
        let sample = Sample {
            time_s: (current - origin) as f64 / if self.receiver { 1e3 } else { 1e6 },
            receiver_timed: self.receiver,
            mapped_ns: record.get("mapped_ns").and_then(Value::as_u64),
            mapping_uncertainty_ns: record.get("mapping_uncertainty_ns").and_then(Value::as_u64),
            mapping_generation: record
                .get("mapping_generation")
                .and_then(Value::as_u64)
                .and_then(|v| u32::try_from(v).ok()),
            velocity_latency_ms: record
                .get("latency_ms")
                .and_then(Value::as_u64)
                .and_then(|v| u32::try_from(v).ok()),
            lat: number(record, "lat"),
            lon: number(record, "lon"),
            speed_mps: number(record, "speed_mps"),
            altitude_m: number(record, "alt_m"),
            heading_deg: number(record, "heading_deg"),
            valid: !self.gap
                && record
                    .get("valid")
                    .and_then(Value::as_bool)
                    .unwrap_or(false),
            gps_week: gps_week.map(|v| v as u32),
            gps_tow_ms: gps_tow.map(|v| v as u32),
            hacc_m: number(record, "hacc_m"),
            vacc_m: number(record, "vacc_m"),
            sacc_mps: number(record, "sacc_mps"),
            vertical_mps: number(record, "vertical_mps"),
            satellites: record
                .get("satellites")
                .and_then(Value::as_u64)
                .and_then(|v| u8::try_from(v).ok()),
            ..Default::default()
        };
        self.gap = false;
        Ok(Some(sample))
    }
}
fn append_record(c: &mut Capture, record: Value) -> Result<(), String> {
    append_record_with_limit(c, record, MAX_CAPTURE_DATA_BYTES)
}
fn before_capture_start(capture: &Capture, record: &Value) -> bool {
    capture.health_start["started_at_us"]
        .as_u64()
        .zip(record["t_us"].as_u64())
        .is_some_and(|(start, at)| at < start)
}
fn stopped_at_us(value: &Value) -> Option<u64> {
    value
        .get("stopped_at_us")
        .and_then(Value::as_u64)
        .or_else(|| value.get("uptime_us").and_then(Value::as_u64))
}
fn capture_health_snapshot(value: &Value) -> Value {
    let counters = json!({"gnss_count":value["gnss"].get("count"),"gnss_errors":value["gnss"].get("errors"),
        "imu_count":value["imu"].get("count"),"imu_errors":value["imu"].get("errors"),
        "imu_timing_rejected":value["capture_timing"].get("imu_timing_rejected"),
        "capture_errors":value["capture_timing"].get("capture_errors"),
        "gnss_unsynchronized":value["capture_timing"].get("gnss_unsynchronized"),
        "usb_queue_dropped":value.get("dropped"),"sd_queue_dropped":value["sd"].get("dropped"),
        "navigation_queue_dropped":value["navigation"].get("queue_dropped"),
        "navigation_errors":value["navigation"].get("errors"),
        "ppk_frames":value["ppk"].get("frames"),"ppk_observation_frames":value["ppk"].get("observation_frames"),
        "ppk_ephemeris_frames":value["ppk"].get("ephemeris_frames"),"ppk_crc_failures":value["ppk"].get("crc_failures"),
        "ppk_truncated_frames":value["ppk"].get("truncated_frames"),"ppk_queue_dropped":value["ppk"].get("dropped"),
        "ppk_command_errors":value["ppk"].get("command_errors"),
        "gnss_data_dropped_reads":value["gnss_byte_queue"]["data"].get("dropped_reads"),
        "gnss_data_dropped_bytes":value["gnss_byte_queue"]["data"].get("dropped_bytes"),
        "gnss_command_dropped_reads":value["gnss_byte_queue"]["command"].get("dropped_reads"),
        "gnss_command_dropped_bytes":value["gnss_byte_queue"]["command"].get("dropped_bytes")});
    json!({"observed_at_us":value.get("uptime_us"),"started_at_us":value.get("started_at_us"),
        "diagnostics_observed_at_us":value.get("diagnostics_observed_at_us").or_else(||value.get("uptime_us")),
        "stopped_at_us":value.get("stopped_at_us"),"device_session":value.get("session"),
        "firmware":value.get("firmware"),"build_mode":value.get("build_mode"),
        "gnss_status":value["gnss"].get("status"),"imu_status":value["imu"].get("status"),
        "imu_last_error":value["imu_diagnostics"].get("last_error"),
        "capture_timing":value.get("capture_timing"),"installation":value.get("installation"),
        "acquisition_profile":value.get("acquisition_profile"),"ppk":value.get("ppk"),"usb_queue":value.get("usb_queue"),"gnss_byte_queue":value.get("gnss_byte_queue"),
        "sd_state":value["sd"].get("state"),"sd_closed":value["sd"].get("closed"),
        "sd_file_id":value["sd"].get("file_id"),"sd_error":value["sd"].get("error"),
        "counters":counters})
}

fn capture_health_record(capture: &Capture) -> Value {
    let mut deltas = serde_json::Map::new();
    let mut reset = capture.health_counter_resets.clone();
    if let Some(counters) = capture.health_latest["counters"].as_object() {
        for (name, end) in counters {
            let start = capture.health_start["counters"]
                .get(name)
                .and_then(Value::as_u64);
            let end = end.as_u64();
            if start.zip(end).is_some_and(|(start, end)| end < start) && !reset.contains(name) {
                reset.push(name.clone());
            }
            deltas.insert(
                name.clone(),
                json!(if reset.contains(name) {
                    None
                } else {
                    start
                        .zip(end)
                        .and_then(|(start, end)| end.checked_sub(start))
                }),
            );
        }
    }
    let mut imu_count = capture.binary_imu_count;
    let mut gnss_count = 0u64;
    let mut valid_gnss = 0u64;
    let mut pre_start = capture.binary_pre_start_count;
    let mut interval_sum = capture.binary_imu_interval_ns;
    let mut first_imu_ns = capture
        .binary_first_imu
        .as_ref()
        .and_then(|v| v["captured_ns"].as_u64());
    let mut last_imu_ns = capture
        .binary_last_imu
        .as_ref()
        .and_then(|v| v["captured_ns"].as_u64());
    for record in &capture.session.raw_records {
        pre_start += u64::from(before_capture_start(capture, record));
        match record["kind"].as_str() {
            Some("imu") => {
                imu_count += 1;
                interval_sum =
                    interval_sum.saturating_add(record["interval_ns"].as_u64().unwrap_or(0));
                if let Some(at) = record["captured_ns"].as_u64() {
                    first_imu_ns.get_or_insert(at);
                    last_imu_ns = Some(at);
                }
            }
            Some("gnss") => {
                gnss_count += 1;
                valid_gnss += u64::from(record["valid"] == true);
            }
            _ => {}
        }
    }
    json!({"kind":"capture_health","schema":1,"start":capture.health_start,"end":capture.health_latest,
        "start_response_us":capture.health_start["observed_at_us"],
        "started_at_us":capture.health_start["started_at_us"],"stopped_at_us":capture.stop_at_us,
        "stop_tail_pending":capture.stop_tail_pending,"stop_tail_received":capture.stop_tail_received,
        "stop_boundary_source":if capture.stop_boundary_confirmed{"firmware"}else if capture.stop_at_us.is_some(){"response-time-fallback"}else{"unconfirmed"},
        "interrupted":capture.interrupted,"incomplete_reason":capture.session.incomplete_reason,
        "counter_deltas":deltas,"counters_reset":reset,
        "counter_window":"Start and end status observations; button-stop diagnostics may include time before the next host poll.",
        "observations":{"gnss_records":gnss_count,"valid_gnss_records":valid_gnss,"imu_records":imu_count,
            "pre_start_records":pre_start,
            "imu_interval_sum_ns":interval_sum,"first_imu_capture_ns":first_imu_ns,"last_imu_capture_ns":last_imu_ns}})
}
fn observe_capture_health(capture: &mut Capture, value: &Value) {
    let next = capture_health_snapshot(value);
    if let Some(counters) = next["counters"].as_object() {
        for (name, value) in counters {
            if value
                .as_u64()
                .zip(capture.health_latest["counters"][name].as_u64())
                .is_some_and(|(next, previous)| next < previous)
                && !capture.health_counter_resets.contains(name)
            {
                capture.health_counter_resets.push(name.clone());
            }
        }
    }
    capture.health_latest = next;
}
fn add_capture_health_warnings(records: &[Value], warnings: &mut Vec<String>) {
    for health in records.iter().filter(|r| r["kind"] == "capture_health") {
        let deltas = &health["counter_deltas"];
        let sensor = [
            ("gnss_errors", "GNSS errors"),
            ("imu_errors", "IMU errors"),
            ("imu_timing_rejected", "IMU timing rejections"),
            ("capture_errors", "capture errors"),
            ("ppk_crc_failures", "raw PPK CRC failures"),
            ("ppk_truncated_frames", "raw PPK truncated frames"),
            ("ppk_command_errors", "raw PPK command errors"),
        ]
        .into_iter()
        .filter_map(|(key, label)| {
            deltas[key]
                .as_u64()
                .filter(|n| *n > 0)
                .map(|n| format!("{n} {label}"))
        })
        .collect::<Vec<_>>();
        if !sensor.is_empty() {
            warnings.push(format!("Sensor acquisition reported {} in the recorded diagnostic window. Sensor gaps are separate from USB/SD queue drops.", sensor.join(", ")));
        }
        let queues = [
            ("usb_queue_dropped", "USB"),
            ("sd_queue_dropped", "SD"),
            ("navigation_queue_dropped", "navigation"),
            ("ppk_queue_dropped", "raw PPK SD"),
            ("gnss_data_dropped_reads", "GNSS data-port UART read chunks"),
            ("gnss_data_dropped_bytes", "GNSS data-port UART bytes"),
            (
                "gnss_command_dropped_reads",
                "GNSS command-port UART read chunks",
            ),
            ("gnss_command_dropped_bytes", "GNSS command-port UART bytes"),
        ]
        .into_iter()
        .filter_map(|(key, label)| {
            deltas[key]
                .as_u64()
                .filter(|n| *n > 0)
                .map(|n| format!("{n} {label}"))
        })
        .collect::<Vec<_>>();
        if !queues.is_empty() {
            warnings.push(format!("Recorded diagnostic window contains {} queue drops; these are separate from sensor acquisition failures.",queues.join(", ")));
        }
        if health["counters_reset"]
            .as_array()
            .is_some_and(|a| !a.is_empty())
        {
            warnings.push("Some device counters reset during capture; their diagnostic deltas are unavailable.".into());
        }
        if health["stop_boundary_source"] != "firmware" {
            warnings.push("An exact device stop boundary was not confirmed; the persisted boundary source identifies the available evidence.".into());
        }
        if let Some(count) = health["observations"]["pre_start_records"]
            .as_u64()
            .filter(|n| *n > 0)
        {
            warnings.push(format!("{count} late-published records precede the device start boundary; they remain in raw evidence and are excluded from projected analysis samples."));
        }
    }
}
fn interrupt_capture(capture: &mut Capture, reason: &str) {
    capture.interrupted = true;
    capture
        .session
        .incomplete_reason
        .get_or_insert_with(|| reason.to_owned());
}
fn journal_record(c: &mut Capture, record: &Value) -> Result<u64, String> {
    c.journal_line.clear();
    serde_json::to_writer(&mut c.journal_line, record).map_err(io_error)?;
    c.journal_line.push(b'\n');
    c.journal.write_all(&c.journal_line).map_err(io_error)?;
    Ok(c.journal_line.len() as u64)
}
fn append_record_with_limit(
    c: &mut Capture,
    mut record: Value,
    data_limit: u64,
) -> Result<(), String> {
    let after_stop = c.stop_at_us.is_some_and(|end| {
        if record["kind"] == "imu" {
            if let Some(captured_ns) = record["captured_ns"].as_u64() {
                // Preserve the hardware timestamp's precision. Comparing its
                // truncated t_us can admit post-stop preview within the same us.
                return u128::from(captured_ns) > u128::from(end) * 1000;
            }
        }
        record["t_us"].as_u64().is_some_and(|at| at > end)
    });
    if after_stop {
        return Ok(());
    }
    let fields = record
        .as_object_mut()
        .ok_or("Invalid sensor record object")?;
    let wire_kind = fields.remove("_wire_kind").and_then(|v| v.as_u64());
    let wire_hex = fields.remove("_wire_hex");
    if let (Some(evidence), Some(kind), Some(Value::String(hex))) =
        (c.evidence.as_mut(), wire_kind, wire_hex)
    {
        let payload = crate::binary::from_hex(&hex)?;
        let at_ns = if kind == 23 {
            record["captured_ns"].as_u64()
        } else {
            record["t_us"].as_u64().and_then(|v| v.checked_mul(1000))
        }
        .ok_or("Missing binary capture timestamp")?;
        if let Err(error) = evidence.append(kind as u8, at_ns, &payload) {
            journal_record(c, &record)?;
            return Err(error);
        }
        if kind == 23 {
            c.binary_imu_count += 1;
            c.binary_pre_start_count += u64::from(before_capture_start(c, &record));
            if let Some(t) = record["t_us"].as_u64() {
                let origin = *c.arrival_origin_us.get_or_insert(t);
                c.session.duration_s = c
                    .session
                    .duration_s
                    .max(t.saturating_sub(origin) as f64 / 1e6);
            }
            c.binary_imu_interval_ns = c
                .binary_imu_interval_ns
                .saturating_add(record["interval_ns"].as_u64().unwrap_or(0));
            c.binary_first_imu.get_or_insert_with(|| record.clone());
            c.binary_last_imu = Some(record);
            return Ok(());
        }
    }
    let raw_bytes = journal_record(c, &record)?;
    // Older firmware can publish a queued pre-start observation after the
    // start command. Preserve its raw evidence without extending this run's
    // receiver trajectory across the actual device start boundary.
    let sample = if before_capture_start(c, &record) {
        None
    } else {
        c.clock.sample(&record)?
    };
    let sample_bytes = sample
        .as_ref()
        .map(serde_json::to_vec)
        .transpose()
        .map_err(io_error)?
        .map_or(0, |bytes| bytes.len() as u64 + 1);
    if c.data_bytes
        .saturating_add(raw_bytes)
        .saturating_add(sample_bytes)
        > data_limit
        || (sample.is_some() && c.session.samples.len() >= MAX_SAMPLES)
    {
        let reason = format!("Capture capacity reached ({} MiB observation budget); received raw evidence remains in the capture journal", data_limit / 1024 / 1024);
        interrupt_capture(c, &reason);
        return Err(reason);
    }
    c.data_bytes += raw_bytes + sample_bytes;
    if let Some(t) = record.get("t_us").and_then(Value::as_u64) {
        let origin = *c.arrival_origin_us.get_or_insert(t);
        c.session.duration_s = c
            .session
            .duration_s
            .max(t.saturating_sub(origin) as f64 / 1e6);
    }
    c.session.raw_records.push(record);
    if let Some(sample) = sample {
        c.session.duration_s = c.session.duration_s.max(sample.time_s);
        c.session.samples.push(sample);
        c.session.sample_count = c.session.samples.len();
    }
    Ok(())
}

fn devices() -> Result<Value, String> {
    Ok(json!(serialport::available_ports().map_err(io_error)?.into_iter().filter_map(|p|{
    match p.port_type {SerialPortType::UsbPort(info) if info.vid==0xc0c0 && (info.pid==0xfeed || info.pid==0xcafe)=>Some(json!({"port":p.port_name,"name":info.product.unwrap_or_else(||"Aevia V2 Mini".into()),"recovery":info.pid==0xcafe})),_=>None}
}).collect::<Vec<_>>()))
}

#[cfg(test)]
fn encode_request(operation: &str) -> Vec<u8> {
    encode_query(&json!({"op":operation}))
}
fn encode_query(query: &Value) -> Vec<u8> {
    let message = query.to_string();
    let mut payload = vec![3];
    let mut length = message.len();
    loop {
        let mut b = (length & 127) as u8;
        length >>= 7;
        if length > 0 {
            b |= 128;
        }
        payload.push(b);
        if length == 0 {
            break;
        }
    }
    payload.extend_from_slice(message.as_bytes());
    let mut output = vec![0; payload.len() + payload.len() / 254 + 2];
    let n = cobs::encode(&payload, &mut output);
    output.truncate(n + 1);
    output[n] = 0;
    output
}
fn decode_reply(frame: &[u8]) -> Result<Value, String> {
    let mut decoded = vec![0; frame.len()];
    let n = cobs::decode(frame, &mut decoded).map_err(|_| "Corrupt COBS response")?;
    decoded.truncate(n);
    if decoded.first() != Some(&3) {
        return Err("Device is in recovery or rejected the command".into());
    }
    let mut length = 0usize;
    let mut offset = 1;
    let mut shift = 0;
    loop {
        let b = *decoded.get(offset).ok_or("Truncated response")?;
        offset += 1;
        if shift >= 28 {
            return Err("Oversized response".into());
        }
        length |= usize::from(b & 127) << shift;
        if b & 128 == 0 {
            break;
        }
        shift += 7;
    }
    if length != decoded.len() - offset {
        return Err("Incorrect response length".into());
    }
    let mut value: Value = serde_json::from_slice(&decoded[offset..]).map_err(io_error)?;
    if value.get("ok").and_then(Value::as_bool) != Some(true) {
        return Err(value
            .get("error")
            .and_then(Value::as_str)
            .unwrap_or("Device rejected request")
            .into());
    }
    if value["binary"] == true {
        if value.get("samples").is_some_and(|samples| {
            !samples.is_null() && !samples.as_array().is_some_and(Vec::is_empty)
        }) {
            return Err("Reply contains both JSON and binary sample batches".into());
        }
        let records = match (value.get("records_b64"), value.get("records_hex")) {
            (Some(Value::String(text)), None) => crate::binary::decode_base64_records(text)?,
            (None, Some(Value::String(text))) => crate::binary::decode_records(text)?,
            (None, None) => return Err("Missing binary batch".into()),
            _ => return Err("Ambiguous or invalid binary batch encoding".into()),
        };
        value["samples"] = Value::Array(records);
        let fields = value.as_object_mut().ok_or("Invalid response object")?;
        fields.remove("records_hex");
        fields.remove("records_b64");
    } else if value.get("records_b64").is_some() || value.get("records_hex").is_some() {
        return Err("Binary records lack the binary reply flag".into());
    }
    Ok(value)
}
fn transact(port: &mut dyn SerialPort, op: &str) -> Result<Value, String> {
    transact_query(port, &json!({"op":op}))
}
fn transact_query(port: &mut dyn SerialPort, query: &Value) -> Result<Value, String> {
    let operation = query["op"].as_str().unwrap_or("");
    let timeout = if matches!(operation, "start" | "stop" | "files" | "read") {
        Duration::from_secs(12)
    } else {
        Duration::from_millis(800)
    };
    port.set_timeout(timeout).map_err(io_error)?;
    port.write_all(&encode_query(query)).map_err(io_error)?;
    port.flush().map_err(io_error)?;
    let mut frame = Vec::with_capacity(4096);
    let start = Instant::now();
    loop {
        let mut buffer = [0u8; 512];
        // Block for the first byte, then drain already buffered USB packets in
        // chunks. One ReadFile call per byte starves high-rate IMU capture.
        let available = port.bytes_to_read().map_err(io_error)? as usize;
        let wanted = available.clamp(1, buffer.len());
        let count = port.read(&mut buffer[..wanted]).map_err(io_error)?;
        if count == 0 {
            return Err("USB stream closed".into());
        }
        if let Some(end) = buffer[..count].iter().position(|b| *b == 0) {
            frame.extend_from_slice(&buffer[..end]);
            if end + 1 != count {
                return Err("Unexpected bytes after device response".into());
            }
            return decode_reply(&frame);
        }
        frame.extend_from_slice(&buffer[..count]);
        if frame.len() > 16384 || start.elapsed() > timeout {
            return Err("Oversized or timed out device reply".into());
        }
    }
}

pub(crate) fn blank_session(name: &str) -> Session {
    Session {
        format_version: 1,
        id: format!("session-{}", now_ms()),
        name: name.into(),
        created_ms: now_ms(),
        duration_s: 0.0,
        sample_count: 0,
        max_speed_mps: None,
        distance_m: 0.0,
        source: "Imported".into(),
        incomplete_reason: None,
        provenance: Value::Null,
        samples: vec![],
        raw_records: vec![],
        track: None,
        drag: DragConfig::default(),
        analysis: Analysis::default(),
    }
}
fn import_csv(content: &str) -> Result<Session, String> {
    let mut lines = content.lines();
    let header = lines.next().ok_or("Empty CSV")?;
    let columns: Vec<_> = header
        .trim_start_matches('\u{feff}')
        .split(',')
        .map(str::trim)
        .collect();
    for required in ["time_s", "lat", "lon", "speed_mps"] {
        if !columns.contains(&required) {
            return Err(format!(
                "CSV needs column {required}; use an Aevia CSV export as the template"
            ));
        }
    }
    let mut session = blank_session("Imported session");
    for (line_no, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let values: Vec<_> = line.split(',').map(str::trim).collect();
        if values.len() != columns.len() {
            return Err(format!(
                "CSV line {} has the wrong number of columns",
                line_no + 2
            ));
        }
        let get = |name: &str| -> Result<Option<f64>, String> {
            match columns
                .iter()
                .position(|c| *c == name)
                .and_then(|i| values.get(i))
            {
                None | Some(&"") => Ok(None),
                Some(value) => value
                    .parse::<f64>()
                    .map(Some)
                    .map_err(|_| format!("Invalid {name} on CSV line {}", line_no + 2)),
            }
        };
        let explicit_valid = columns
            .iter()
            .position(|c| *c == "valid")
            .map(|i| values[i]);
        let valid = match explicit_valid {
            Some("true" | "1") => true,
            Some("false" | "0") => false,
            Some(_) => return Err(format!("Invalid validity on line {}", line_no + 2)),
            None => false,
        };
        session.samples.push(Sample {
            time_s: get("time_s")?.ok_or("Missing timestamp")?,
            lat: get("lat")?,
            lon: get("lon")?,
            speed_mps: get("speed_mps")?,
            altitude_m: get("altitude_m")?,
            heading_deg: get("heading_deg")?,
            accel_long_g: get("accel_long_g")?,
            accel_lat_g: get("accel_lat_g")?,
            valid,
            hacc_m: get("hacc_m")?,
            vacc_m: get("vacc_m")?,
            sacc_mps: get("sacc_mps")?,
            ..Default::default()
        });
        if session.samples.len() > MAX_SAMPLES {
            return Err("CSV has too many samples".into());
        }
    }
    if session.samples.is_empty() {
        return Err("CSV contains no samples".into());
    }
    validate_session(&session)?;
    Ok(session)
}
fn export_csv(session: &Session) -> String {
    let mut output=String::from("time_s,lat,lon,speed_mps,altitude_m,heading_deg,accel_long_g,accel_lat_g,valid,hacc_m,vacc_m,sacc_mps\n");
    let val = |v: Option<f64>| v.map(|v| format!("{v:.9}")).unwrap_or_default();
    for s in &session.samples {
        output.push_str(&format!(
            "{:.9},{},{},{},{},{},{},{},{},{},{},{}\n",
            s.time_s,
            val(s.lat),
            val(s.lon),
            val(s.speed_mps),
            val(s.altitude_m),
            val(s.heading_deg),
            val(s.accel_long_g),
            val(s.accel_lat_g),
            s.valid,
            val(s.hacc_m),
            val(s.vacc_m),
            val(s.sacc_mps)
        ));
    }
    output
}
fn xml_escape(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
fn export_gpx(session: &Session) -> String {
    let mut output=format!("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.1\" creator=\"Aevia\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk><name>{}</name><trkseg>\n",xml_escape(&session.name));
    let mut active = false;
    for s in &session.samples {
        if let (true, Some(lat), Some(lon)) = (s.valid, s.lat, s.lon) {
            if !active && output.ends_with("</trkseg>\n") {
                output.push_str("<trkseg>\n");
            }
            active = true;
            output.push_str(&format!("<trkpt lat=\"{lat:.9}\" lon=\"{lon:.9}\">"));
            if let Some(alt) = s.altitude_m {
                output.push_str(&format!("<ele>{alt:.3}</ele>"));
            }
            // Device monotonic time is not UTC; do not invent a GPX timestamp.
            output.push_str("</trkpt>\n");
        } else if active {
            output.push_str("</trkseg>\n");
            active = false;
        }
    }
    if active || !output.ends_with("</trkseg>\n") {
        output.push_str("</trkseg>");
    }
    output.push_str("</trk></gpx>\n");
    output
}
fn recover_journal(content: &str) -> Result<Session, String> {
    let mut lines = content.lines();
    let header: Value =
        serde_json::from_str(lines.next().ok_or("Empty journal")?).map_err(io_error)?;
    let mut session: Session = serde_json::from_value(
        header
            .get("session")
            .cloned()
            .ok_or("Missing journal header")?,
    )
    .map_err(io_error)?;
    let records: Vec<_> = lines.collect();
    let mut clock = SampleClock::default();
    for (index, line) in records.iter().enumerate() {
        let record: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(_) if index + 1 == records.len() => break,
            Err(e) => return Err(format!("Corrupt journal record {}: {e}", index + 2)),
        };
        match clock.sample(&record) {
            Ok(Some(sample)) => session.samples.push(sample),
            Ok(None) => {}
            // A receiver clock reset terminates the recovered span. Raw
            // evidence remains in the original journal for a separate session.
            Err(_) => break,
        }
        session.raw_records.push(record);
    }
    session.name.push_str(" (recovered)");
    session.incomplete_reason =
        Some("Recovered capture journal; the recording did not complete a verified save.".into());
    Ok(session)
}
fn demo_session() -> Session {
    let mut session = blank_session("Demo · club circuit");
    session.id = format!("demo-{}", now_ms());
    session.source = "Simulated demonstration · not a recorded drive".into();
    let latitude = -33.8f64;
    let longitude = 150.8;
    let radius_e = 260.0;
    let radius_n = 180.0;
    let metres_lat = 111_320.0;
    let metres_lon = metres_lat * latitude.to_radians().cos();
    let omega = std::f64::consts::TAU / 85.0;
    for i in 0..3001 {
        let t = i as f64 / 10.0;
        let theta = omega * t - 0.3 + 0.035 * (t / 18.0).sin();
        let rate = omega + 0.035 / 18.0 * (t / 18.0).cos();
        let east = radius_e * theta.sin();
        let north = radius_n * theta.cos();
        let ve = radius_e * theta.cos() * rate;
        let vn = -radius_n * theta.sin() * rate;
        session.samples.push(Sample {
            time_s: t,
            lat: Some(latitude + north / metres_lat),
            lon: Some(longitude + east / metres_lon),
            speed_mps: Some(ve.hypot(vn)),
            altitude_m: Some(35.0 + theta.sin()),
            heading_deg: Some(ve.atan2(vn).to_degrees().rem_euclid(360.0)),
            valid: true,
            hacc_m: Some(0.5),
            vacc_m: Some(1.0),
            sacc_mps: Some(0.05),
            ..Default::default()
        });
    }
    session.track = Some(Track {
        id: "demo-circuit".into(),
        name: "Demo club circuit".into(),
        start: Gate {
            lat: latitude + radius_n / metres_lat,
            lon: longitude,
            heading_deg: 90.0,
            width_m: 30.0,
        },
        sectors: vec![
            Gate {
                lat: latitude,
                lon: longitude + radius_e / metres_lon,
                heading_deg: 180.0,
                width_m: 30.0,
            },
            Gate {
                lat: latitude - radius_n / metres_lat,
                lon: longitude,
                heading_deg: 270.0,
                width_m: 30.0,
            },
        ],
    });
    session
}

#[cfg(test)]
mod tests {
    use super::*;
    struct TestStorage(PathBuf);
    impl TestStorage {
        fn new() -> Self {
            let unique = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let path = std::env::temp_dir()
                .join(format!("aevia-retention-{}-{unique}", std::process::id()));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
        fn service(&self) -> Service {
            Service {
                root: self.0.clone(),
                preferences: Preferences::default(),
                sessions: vec![],
                port: None,
                port_name: None,
                status: String::new(),
                latest: Value::Null,
                last_seen: None,
                last_full_status: None,
                sequence: 0,
                gnss_rate: None,
                imu_rate: None,
                imu_rate_sample: None,
                rate_sample: None,
                capture: None,
                processing: None,
            }
        }
        fn capture(&self) -> Capture {
            let mut session = blank_session("Retention boundary");
            session.id = "session-test".into();
            let mut journal = File::create(self.0.join("session-test.capture.jsonl")).unwrap();
            writeln!(journal, "{}", json!({"session":session})).unwrap();
            Capture {
                session,
                journal,
                journal_line: Vec::new(),
                clock: SampleClock::default(),
                last_sync: Instant::now(),
                last_dropped: 0,
                analyzed_samples: 0,
                interrupted: false,
                started: Instant::now(),
                arrival_origin_us: None,
                stop_at_us: None,
                data_bytes: 0,
                health_start: Value::Null,
                health_latest: Value::Null,
                health_counter_resets: vec![],
                stop_boundary_confirmed: false,
                stop_tail_pending: None,
                stop_tail_received: 0,
                evidence: None,
                binary_imu_count: 0,
                binary_imu_interval_ns: 0,
                binary_pre_start_count: 0,
                binary_first_imu: None,
                binary_last_imu: None,
            }
        }
    }
    impl Drop for TestStorage {
        fn drop(&mut self) {
            // This directory was exclusively created by this fixture above.
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    #[test]
    fn binary_imu_evidence_stays_bounded_and_reimports_exact_support() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        let path = storage.0.join("raw.aev");
        capture.evidence =
            Some(crate::evidence::Writer::create(&path, 77, 8 * 1024 * 1024).unwrap());
        let journal_bytes = capture.journal.metadata().unwrap().len();
        for index in 0..10_000u64 {
            let at = 10_000_000 + index * 1_375_000;
            let sample = crate::sample::Sample::Imu {
                t_us: at / 1000,
                dt_us: 1375,
                accel: [-0.0, 1.25, 9.8],
                gyro: [0.00020720869813913682, 0.2, 0.3],
                valid: true,
                captured_ns: at,
                interval_ns: 1_375_000,
                effective_end_ns: at - 1_500_000,
                timing_uncertainty_ns: 1_200_000,
                aligned_gyro: [0.1, 0.2, 0.3],
            };
            let mut payload = [0; 128];
            let (kind, _, len) = crate::firmware_record_codec::encode(sample, &mut payload);
            let mut bytes = vec![kind, len as u8, 0];
            bytes.extend_from_slice(&payload[..len]);
            let record = crate::binary::decode_records(&crate::binary::to_hex(&bytes))
                .unwrap()
                .remove(0);
            append_record_with_limit(&mut capture, record, 1).unwrap();
        }
        assert_eq!(capture.session.raw_records.len(), 0);
        assert_eq!(capture.data_bytes, 0);
        assert_eq!(capture.journal.metadata().unwrap().len(), journal_bytes);
        assert_eq!(capture.binary_imu_count, 10_000);
        assert!(capture.session.duration_s > 13.0);
        capture
            .evidence
            .as_mut()
            .unwrap()
            .close(14_000_000_000, 0)
            .unwrap();
        let imported = crate::legacy::import(&path).unwrap();
        let timing = imported
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["timed_imu_records"], 10_000);
        assert_eq!(
            timing["last_imu"]["captured_ns"],
            capture.binary_last_imu.as_ref().unwrap()["captured_ns"]
        );
        assert_eq!(
            timing["first_imu"]["gyro"][0].as_f64().unwrap().to_bits(),
            0.00020720869813913682f64.to_bits()
        );
    }
    #[test]
    fn measured_imu_rate_only_changes_on_fresh_counters() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        service.merge_device_reply(json!({"uptime_us":1,"imu":{"count":10},"binary_poll":true}));
        service.imu_rate_sample = Some((Instant::now() - Duration::from_secs(1), 10));
        service.merge_device_reply(
            json!({"uptime_us":1_000_001,"imu":{"count":737},"binary_poll":true}),
        );
        let rate = service.imu_rate.unwrap();
        assert!((rate - 727.0).abs() < 1.0);
        service.merge_device_reply(
            json!({"compact":true,"binary":true,"uptime_us":1_100_001,"samples":[]}),
        );
        assert_eq!(service.imu_rate, Some(rate));
        assert_eq!(
            service.next_collection_query(),
            json!({"op":"drain_binary"})
        );
    }
    #[test]
    fn transport_negotiation_prefers_base64_and_rejects_ambiguous_batch_envelopes() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        service.merge_device_reply(
            json!({"uptime_us":1000,"base64_poll":true,"binary_poll":true,"compact_poll":true}),
        );
        assert_eq!(
            service.next_collection_query(),
            json!({"op":"drain_base64"})
        );
        service.latest["base64_poll"] = json!(false);
        assert_eq!(
            service.next_collection_query(),
            json!({"op":"drain_binary"})
        );
        service.latest["binary_poll"] = json!(false);
        assert_eq!(service.next_collection_query(), json!({"op":"drain"}));
        service.latest["compact_poll"] = json!(false);
        assert_eq!(service.next_collection_query(), json!({"op":"poll"}));
        let reply = |value: &Value| {
            let encoded = encode_query(value);
            decode_reply(&encoded[..encoded.len() - 1])
        };
        let value=reply(&json!({"ok":true,"compact":true,"binary":true,"records_b64":"","recording":false,"stopped_at_us":123,"pending":7,"dropped":2})).unwrap();
        assert_eq!(value["samples"], json!([]));
        assert_eq!(value["stopped_at_us"], 123);
        assert_eq!(value["pending"], 7);
        assert_eq!(value["dropped"], 2);
        assert!(value.get("records_b64").is_none());
        assert!(
            reply(&json!({"ok":true,"binary":true,"records_hex":"","records_b64":""})).is_err()
        );
        assert!(reply(&json!({"ok":true,"binary":true,"records_b64":"","samples":[{}]})).is_err());
        assert!(reply(&json!({"ok":true,"binary":true,"records_b64":null})).is_err());
        assert!(reply(&json!({"ok":true,"records_b64":""})).is_err());
        assert!(reply(&json!({"ok":true,"binary":true,"records_hex":""})).is_ok());
    }
    #[test]
    fn producer_base64_reply_keeps_stop_precision_and_never_replays_or_rewinds_latest() {
        use crate::firmware_response::{Base64Records, Response};
        let storage = TestStorage::new();
        let mut service = storage.service();
        let mut capture = storage.capture();
        let path = storage.0.join("base64.aev");
        capture.evidence = Some(crate::evidence::Writer::create(&path, 1, 1_048_576).unwrap());
        service.capture = Some(capture);
        let newer = json!({"kind":"imu","captured_ns":10_100_000,"t_us":10100,"valid":true});
        service.merge_device_reply(
            json!({"base64_poll":true,"uptime_us":12000,"imu":{"latest":newer,"count":2}}),
        );
        let mut response = Response::<8192>::new();
        let mut encoder = Base64Records::new();
        response.push_str("{\"ok\":true,\"compact\":true,\"binary\":true,\"recording\":false,\"session\":7,\"started_at_us\":5000,\"stopped_at_us\":10000,\"records_b64\":\"").unwrap();
        for at in [10_000_000, 10_000_001] {
            let sample = crate::sample::Sample::Imu {
                t_us: at / 1000,
                dt_us: 1375,
                accel: [-0.0, 1.0, 9.8],
                gyro: [0.1, 0.2, 0.3],
                valid: true,
                captured_ns: at,
                interval_ns: 1_375_000,
                effective_end_ns: at - 1_500_000,
                timing_uncertainty_ns: 1_200_000,
                aligned_gyro: [0.1, 0.2, 0.3],
            };
            let mut payload = [0; 128];
            let (kind, _, len) = crate::firmware_record_codec::encode(sample, &mut payload);
            response
                .record_base64(&mut encoder, kind, &payload[..len], 200)
                .unwrap();
        }
        response.finish_base64(&mut encoder).unwrap();
        response
            .push_str("\",\"pending\":5,\"dropped\":3,\"uptime_us\":13000}")
            .unwrap();
        let wire = encode_query(&serde_json::from_str(response.as_str()).unwrap());
        let mut calls = 0;
        assert!(
            service.collect_with_tail(Duration::from_secs(1), Some(2), |_| {
                calls += 1;
                decode_reply(&wire[..wire.len() - 1])
            })
        );
        assert_eq!(calls, 1);
        assert_eq!(service.latest["imu"]["latest"], newer);
        assert_eq!(service.latest["recording"], false);
        assert!(service.latest.get("records_b64").is_none());
        assert!(service.latest.get("samples").is_none());
        let capture = service.capture.as_mut().unwrap();
        assert_eq!(capture.binary_imu_count, 1);
        assert_eq!(capture.last_dropped, 3);
        assert_eq!(capture.stop_tail_received, 2);
        assert_eq!(capture.stop_at_us, Some(10000));
        assert_eq!(
            capture.binary_first_imu.as_ref().unwrap()["captured_ns"],
            10_000_000
        );
        capture
            .evidence
            .as_mut()
            .unwrap()
            .close(10_000_000, 3)
            .unwrap();
        let imported = crate::legacy::import(&path).unwrap();
        let timing = imported
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["timed_imu_records"], 1);
        assert!(service
            .merge_device_reply(json!({"compact":true,"pending":0,"uptime_us":14000}))
            .is_empty());
        assert_eq!(service.capture.as_ref().unwrap().binary_imu_count, 1);
    }
    #[test]
    fn json_export_and_import_copy_paired_evidence_before_original_disappears() {
        let source = TestStorage::new();
        let destination = TestStorage::new();
        let portable = TestStorage::new();
        let mut service = source.service();
        let mut session = blank_session("Paired source");
        let raw = source.0.join("source.aev");
        fs::write(&raw, b"exact raw evidence").unwrap();
        session
            .raw_records
            .push(json!({"kind":"raw_capture_file","path":raw,"source":"usb-binary"}));
        service.save(&session).unwrap();
        let exported = portable.0.join("portable.json");
        let result = service
            .export(&json!({"id":session.id,"path":exported,"format":"json"}))
            .unwrap();
        assert_eq!(result["paired_files"].as_array().unwrap().len(), 1);
        let exported_session = read_session(&exported).unwrap();
        assert!(
            !Path::new(exported_session.raw_records[0]["path"].as_str().unwrap()).is_absolute()
        );
        drop(service);
        drop(source);
        let mut target = destination.service();
        let imported = target.import(&exported).unwrap();
        drop(portable);
        let paired = Path::new(imported.raw_records[0]["path"].as_str().unwrap());
        assert_eq!(fs::read(paired).unwrap(), b"exact raw evidence");
        assert!(target.load(&imported.id).is_ok());
    }
    #[test]
    fn ppk_reanalysis_retains_quality_and_never_generates_receiver_metrics() {
        let storage = TestStorage::new();
        let service = storage.service();
        let mut session = blank_session("PPK");
        session.provenance =
            json!({"kind":"ppk","parent_session_id":"spp-source","warnings":["Position-only"]});
        session.samples.push(Sample {
            time_s: 0.0,
            lat: Some(-33.0),
            lon: Some(151.0),
            altitude_m: Some(20.0),
            valid: true,
            ppk_quality: Some(json!({"q":2,"status":"float"})),
            ..Sample::default()
        });
        service.analyze(&mut session).unwrap();
        assert_eq!(session.analysis.valid_samples, 1);
        assert!(session.analysis.laps.is_empty() && session.analysis.drag_runs.is_empty());
        assert!(session.analysis.max_speed_mps.is_none());
        assert_eq!(session.analysis.warnings, vec!["Position-only"]);
        assert_eq!(session.provenance["parent_session_id"], "spp-source");
    }
    #[test]
    fn capture_health_retains_profile_and_raw_ppk_error_deltas() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        let start = json!({"acquisition_profile":{"gnss_requested_hz":50},"ppk":{"observation_frames":20,"crc_failures":1,"dropped":2,"truncated_frames":0},"gnss_byte_queue":{"capacity_bytes":8192,"data":{"dropped_reads":2,"dropped_bytes":200},"command":{"dropped_reads":1,"dropped_bytes":10}}});
        capture.health_start = capture_health_snapshot(&start);
        capture.health_latest = capture_health_snapshot(
            &json!({"acquisition_profile":{"gnss_requested_hz":50},"ppk":{"observation_frames":30,"crc_failures":3,"dropped":5,"truncated_frames":1},"gnss_byte_queue":{"capacity_bytes":8192,"data":{"dropped_reads":5,"dropped_bytes":500},"command":{"dropped_reads":1,"dropped_bytes":14}}}),
        );
        let health = capture_health_record(&capture);
        assert_eq!(
            health["start"]["acquisition_profile"]["gnss_requested_hz"],
            50
        );
        assert_eq!(health["counter_deltas"]["ppk_crc_failures"], 2);
        assert_eq!(health["counter_deltas"]["ppk_queue_dropped"], 3);
        assert_eq!(health["start"]["gnss_byte_queue"]["capacity_bytes"], 8192);
        assert_eq!(health["counter_deltas"]["gnss_data_dropped_reads"], 3);
        assert_eq!(health["counter_deltas"]["gnss_data_dropped_bytes"], 300);
        assert_eq!(health["counter_deltas"]["gnss_command_dropped_reads"], 0);
        assert_eq!(health["counter_deltas"]["gnss_command_dropped_bytes"], 4);
        let mut warnings = vec![];
        add_capture_health_warnings(&[health], &mut warnings);
        assert!(warnings.iter().any(|s| s.contains("raw PPK CRC")));
        assert!(warnings.iter().any(|s| s.contains("raw PPK SD")));
        assert!(warnings
            .iter()
            .any(|s| s.contains("300 GNSS data-port UART bytes")));
    }
    #[test]
    fn canonical_byte_boundary_saves_reloadably_and_rejects_without_replacing() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let mut session = blank_session("At the byte boundary");
        session.id = "session-boundary".into();
        session
            .raw_records
            .push(json!({"kind":"imu","raw":"evidence"}));
        let bytes = serde_json::to_vec(&session).unwrap();
        let path = storage.0.join("session-boundary.json");
        service
            .save_with_limit(&session, bytes.len() as u64)
            .unwrap();
        assert_eq!(fs::read(&path).unwrap(), bytes);
        assert_eq!(
            service.load(&session.id).unwrap().raw_records,
            session.raw_records
        );
        assert_eq!(
            service.sessions.len(),
            1,
            "successful saves must remain library-visible"
        );
        session.name.push('!');
        assert!(service
            .save_with_limit(&session, bytes.len() as u64)
            .is_err());
        assert_eq!(
            fs::read(&path).unwrap(),
            bytes,
            "oversized replacement must preserve the prior complete file"
        );
        assert!(!path.with_extension("tmp").exists());
        service.refresh_sessions();
        assert_eq!(service.sessions.len(), 1);
        session.samples = vec![Sample {
            time_s: -1.0,
            ..Default::default()
        }];
        assert!(
            service.save(&session).is_err(),
            "save and load must apply the same validation"
        );
        session.samples.clear();
        session.duration_s = f64::NAN;
        assert!(
            service.save(&session).is_err(),
            "serialized output must also pass the real loader before replacement"
        );
        assert_eq!(fs::read(path).unwrap(), bytes);
    }
    #[test]
    fn capacity_interruption_preserves_boundary_record_and_recovery_journal() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let mut capture = storage.capture();
        let record = json!({"kind":"imu","t_us":100,"raw":"complete evidence"});
        let limit = serde_json::to_vec(&record).unwrap().len() as u64 + 1;
        append_record_with_limit(&mut capture, record.clone(), limit).unwrap();
        assert_eq!(capture.data_bytes, limit);
        assert!(append_record_with_limit(&mut capture, record.clone(), limit).is_err());
        assert!(capture.interrupted);
        // Remainder of a drained USB batch is still written after interruption.
        journal_record(
            &mut capture,
            &json!({"kind":"imu","t_us":101,"raw":"batch remainder"}),
        )
        .unwrap();
        service.capture = Some(capture);
        assert_eq!(service.state()["live"]["capture_interrupted"], true);
        assert!(service.state()["live"]["capture_error"]
            .as_str()
            .unwrap()
            .contains("capacity"));
        let capture = service.capture.take().unwrap();
        service.finalize_capture(capture).unwrap();
        let journal = fs::read_to_string(storage.0.join("session-test.capture.jsonl")).unwrap();
        assert_eq!(
            journal.lines().count(),
            5,
            "header, every received record and final diagnostics must survive"
        );
        let recovered = recover_journal(&journal).unwrap();
        assert_eq!(recovered.raw_records.len(), 4);
        assert!(recovered.incomplete_reason.is_some());
        let mut saved = service.load("session-test").unwrap();
        assert!(saved.incomplete_reason.is_some());
        service.analyze(&mut saved).unwrap();
        assert!(saved
            .analysis
            .warnings
            .iter()
            .any(|w| w.contains("Incomplete recording:")));
        assert_eq!(service.sessions.len(), 1);
        assert!(service.sessions[0]["incomplete_reason"].is_string());
    }
    #[test]
    fn no_fix_capture_persists_bounded_health_and_separates_sensor_errors_from_drops() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let mut capture = storage.capture();
        capture.health_start = capture_health_snapshot(&json!({"uptime_us":100,
            "started_at_us":90,"gnss":{"count":2524,"errors":13},"imu":{"count":23329,"errors":2371},
            "capture_timing":{"imu_timing_rejected":2367,"capture_errors":0},
            "dropped":0,"sd":{"dropped":0},"navigation":{"queue_dropped":0}}));
        capture.health_latest = capture_health_snapshot(&json!({"uptime_us":30000000,
            "stopped_at_us":29999999,"gnss":{"count":3022,"errors":22},"imu":{"count":26731,"errors":5039},
            "capture_timing":{"imu_timing_rejected":5022,"capture_errors":0},
            "dropped":0,"sd":{"dropped":0},"navigation":{"queue_dropped":0}}));
        for (tow, at) in [(1000, 1000), (1050, 51000)] {
            let mut record = gnss(1, tow, at);
            record["valid"] = json!(false);
            record["lat"] = json!(0.0);
            record["lon"] = json!(0.0);
            record["speed_mps"] = json!(0.0);
            append_record(&mut capture, record).unwrap();
        }
        capture.stop_at_us = Some(29_999_999);
        capture.stop_boundary_confirmed = true;
        service.finalize_capture(capture).unwrap();
        let saved = service.load("session-test").unwrap();
        assert_eq!(saved.samples.len(), 2);
        assert!(saved.samples.iter().all(|s| !s.valid));
        assert_eq!(saved.analysis.valid_samples, 0);
        assert!(saved.analysis.laps.is_empty() && saved.analysis.drag_runs.is_empty());
        assert!(saved.max_speed_mps.is_none());
        let summaries: Vec<_> = saved
            .raw_records
            .iter()
            .filter(|r| r["kind"] == "capture_health")
            .collect();
        assert_eq!(summaries.len(), 1);
        let summary = summaries[0];
        assert_eq!(summary["started_at_us"], 90);
        assert_eq!(summary["stopped_at_us"], 29_999_999);
        assert_eq!(summary["stop_boundary_source"], "firmware");
        assert_eq!(summary["counter_deltas"]["imu_errors"], 2668);
        assert_eq!(summary["counter_deltas"]["gnss_errors"], 9);
        assert_eq!(summary["counter_deltas"]["imu_timing_rejected"], 2655);
        assert_eq!(summary["counter_deltas"]["usb_queue_dropped"], 0);
        assert!(saved.analysis.warnings.iter().any(
            |w| w.contains("2668 IMU errors") && w.contains("separate from USB/SD queue drops")
        ));
    }
    #[test]
    fn diagnostic_counter_reset_is_unavailable_not_zero_loss() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        capture.health_start = capture_health_snapshot(&json!({"imu":{"errors":8}}));
        capture.health_latest = capture_health_snapshot(&json!({"imu":{"errors":2}}));
        let record = capture_health_record(&capture);
        assert!(record["counter_deltas"]["imu_errors"].is_null());
        assert_eq!(record["counters_reset"], json!(["imu_errors"]));
        let mut warnings = vec![];
        add_capture_health_warnings(&[record], &mut warnings);
        assert!(warnings.iter().any(|w| w.contains("counters reset")));
        capture.health_latest = capture.health_start.clone();
        observe_capture_health(&mut capture, &json!({"imu":{"errors":2}}));
        observe_capture_health(&mut capture, &json!({"imu":{"errors":20}}));
        let record = capture_health_record(&capture);
        assert!(record["counter_deltas"]["imu_errors"].is_null(),
            "a counter reset must remain explicit even after its value surpasses the starting count");
    }
    #[test]
    fn usb_json_roundtrip_preserves_raw_gyro_bits() {
        // Values observed in an actual SD/USB comparison that changed by one
        // ULP under serde_json's default fast floating-point parser.
        let values = [
            0.00020720869813913682f64,
            -1.4915348803645755e-7,
            -0.00012088164897035419,
            -0.00018816830998063866,
        ];
        for value in values {
            let encoded =
                serde_json::to_string(&json!({"kind":"imu","gyro":[value,0.0,0.0]})).unwrap();
            let received: Value = serde_json::from_str(&encoded).unwrap();
            assert_eq!(
                received["gyro"][0].as_f64().unwrap().to_bits(),
                value.to_bits()
            );
            let saved = serde_json::to_string(&received).unwrap();
            let loaded: Value = serde_json::from_str(&saved).unwrap();
            assert_eq!(
                loaded["gyro"][0].as_f64().unwrap().to_bits(),
                value.to_bits()
            );
        }
    }
    #[test]
    fn full_poll_compatibility_retains_batches_once_and_prefers_newer_status_fix() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        service.capture = Some(storage.capture());
        assert_eq!(service.next_collection_query(), json!({"op":"poll"}));
        let queued = gnss(2400, 1000, 900);
        let latest = gnss(2400, 1050, 990);
        assert!(service.collect_with(Duration::from_secs(1), |_|Ok(json!({
            "uptime_us":1000,"pending":0,"dropped":0,"gnss":{"count":2,"latest":latest},"samples":[queued]}))));
        assert_eq!(
            service.capture.as_ref().unwrap().session.raw_records.len(),
            1
        );
        assert_eq!(service.latest["gnss"]["latest"], latest);
        assert_eq!(service.latest["gnss"]["age_us"], 10);
        assert!(
            service.latest.get("samples").is_none(),
            "a cached batch must never be replayed"
        );
        assert!(service.collect_with(Duration::from_secs(1), |_|Ok(json!({
            "uptime_us":1100,"pending":0,"dropped":0,"gnss":{"count":2,"latest":latest},"samples":[]}))));
        assert_eq!(
            service.capture.as_ref().unwrap().session.raw_records.len(),
            1
        );
        assert_eq!(service.next_collection_query(), json!({"op":"poll"}));
    }
    #[test]
    fn compact_merge_preserves_diagnostics_lifecycle_and_newest_sample_age() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let latest = gnss(2400, 1000, 1_900_000);
        service.merge_device_reply(json!({"compact_poll":true,"uptime_us":2_000_000,
            "battery":{"percent":50},"gnss":{"count":30,"errors":1,"latest":latest},"samples":[]}));
        let baseline = Instant::now() - Duration::from_secs(2);
        service.rate_sample = Some((baseline, 10));
        service.gnss_rate = Some(20.0);
        let records = service.merge_device_reply(json!({"compact":true,"uptime_us":2_100_000,
            "recording":false,"session":8,"started_at_us":1_500_000,"stopped_at_us":2_000_000,
            "dropped":7,"pending":15,"samples":[gnss(2400,950,1_800_000)]}));
        assert_eq!(records.len(), 1);
        assert_eq!(service.latest["gnss"]["latest"], latest);
        assert_eq!(service.latest["gnss"]["age_us"], 200_000);
        assert_eq!(service.latest["gnss"]["count"], 30);
        assert_eq!(service.latest["battery"]["percent"], 50);
        assert_eq!(service.latest["dropped"], 7);
        assert_eq!(service.latest["stopped_at_us"], 2_000_000);
        assert_eq!(service.latest["recording"], false);
        assert_eq!(
            service.rate_sample,
            Some((baseline, 10)),
            "merged counters are not fresh measurements"
        );
        assert_eq!(service.gnss_rate, Some(20.0));
        assert_eq!(service.next_collection_query(), json!({"op":"drain"}));
        service.merge_device_reply(
            json!({"compact":true,"uptime_us":3_200_000,"pending":0,"samples":[]}),
        );
        assert_eq!(service.latest["gnss"]["age_us"], 1_300_000);
        assert!(service.sensor_age_us("gnss").unwrap() >= 1_300_000);
        assert_eq!(service.latest["diagnostics_observed_at_us"], 2_000_000);
        let newer = gnss(2400, 1100, 3_190_000);
        assert_eq!(
            service
                .merge_device_reply(
                    json!({"compact":true,"uptime_us":3_200_000,"pending":0,"samples":[newer]})
                )
                .len(),
            1
        );
        assert_eq!(service.latest["gnss"]["latest"], newer);
        assert_eq!(service.latest["gnss"]["age_us"], 10_000);
        assert!(service
            .merge_device_reply(json!({"compact":true,"uptime_us":3_210_000,"pending":0}))
            .is_empty());
        service.last_full_status = Some(Instant::now() - Duration::from_secs(2));
        assert_eq!(service.next_collection_query(), json!({"op":"status"}));
        service.merge_device_reply(json!({"compact_poll":true,"uptime_us":3_220_000,"gnss":{"count":50,"latest":newer},"samples":[]}));
        assert!(service
            .gnss_rate
            .is_some_and(|hz| (19.0..=20.1).contains(&hz)));
        assert_eq!(service.next_collection_query(), json!({"op":"drain"}));
    }
    #[test]
    fn compact_clock_reset_clears_old_sensor_cache_and_future_age_is_unknown() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        service.merge_device_reply(json!({"compact_poll":true,"uptime_us":1_000_000,
            "gnss":{"count":20,"latest":gnss(2400,1000,900_000)}}));
        service.merge_device_reply(json!({"compact":true,"uptime_us":100,"samples":[]}));
        assert!(service.latest["gnss"].is_null());
        assert!(service.last_full_status.is_none());
        assert_eq!(service.next_collection_query(), json!({"op":"poll"}));
        service.merge_device_reply(
            json!({"compact":true,"uptime_us":200,"samples":[gnss(2400,1050,201)]}),
        );
        assert!(
            service.sensor_age_us("gnss").is_none(),
            "future-relative timestamps must not be clamped to fresh"
        );
        service.merge_device_reply(json!({"compact":true,"uptime_us":300,"samples":[]}));
        assert_eq!(service.latest["gnss"]["age_us"], 99);
    }
    #[test]
    fn batch_merge_keeps_newest_observation_per_sensor_and_returns_all_records() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let gnss = gnss(2400, 1100, 3200);
        let newest_imu = json!({"kind":"imu","t_us":3199,"captured_ns":3_199_900,
            "accel":[1,2,3],"_wire_kind":23,"_wire_hex":"abcd"});
        let older_imu = json!({"kind":"imu","t_us":3199,"captured_ns":3_199_800});
        let duplicate_imu = json!({"kind":"imu","t_us":3199,"captured_ns":3_199_900,
            "accel":[4,5,6]});
        let samples = vec![
            gnss.clone(),
            older_imu.clone(),
            newest_imu,
            older_imu,
            duplicate_imu,
        ];
        let records =
            service.merge_device_reply(json!({"compact":true,"uptime_us":3210,"samples":samples}));
        assert_eq!(records, samples);
        assert_eq!(service.latest["gnss"]["latest"], gnss);
        assert_eq!(service.latest["imu"]["latest"]["accel"], json!([1, 2, 3]));
        assert!(service.latest["imu"]["latest"].get("_wire_hex").is_none());
        assert_eq!(service.latest["imu"]["age_us"], 11);
    }
    #[test]
    fn imu_stop_filter_preserves_nanosecond_boundary_instead_of_truncated_microseconds() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        capture.stop_at_us = Some(100);
        for captured_ns in [99_999u64, 100_000, 100_001] {
            append_record(
                &mut capture,
                json!({"kind":"imu","captured_ns":captured_ns,
                "t_us":captured_ns/1000}),
            )
            .unwrap();
        }
        // Older records without ns evidence keep their declared us boundary.
        append_record(&mut capture, json!({"kind":"imu","t_us":100})).unwrap();
        assert_eq!(capture.session.raw_records.len(), 3);
        assert_eq!(capture.session.raw_records[0]["captured_ns"], 99_999);
        assert_eq!(capture.session.raw_records[1]["captured_ns"], 100_000);
        assert!(capture.session.raw_records[2]["captured_ns"].is_null());
        capture.journal.flush().unwrap();
        let journal = fs::read_to_string(storage.0.join("session-test.capture.jsonl")).unwrap();
        assert_eq!(
            journal.lines().count(),
            4,
            "post-boundary preview must not enter raw recording evidence"
        );
    }
    #[test]
    fn confirmed_stop_boundary_excludes_preview_during_slow_sd_flush() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        capture.stop_at_us = stopped_at_us(&json!({"stopped_at_us":100,"uptime_us":9_000_000}));
        for kind in ["gnss", "imu", "health"] {
            append_record(&mut capture, json!({"kind":kind,"t_us":100,"valid":false})).unwrap();
            append_record(&mut capture, json!({"kind":kind,"t_us":101,"valid":false})).unwrap();
        }
        assert_eq!(capture.session.raw_records.len(), 3);
        assert!(capture.session.raw_records.iter().all(|r| r["t_us"] == 100));
        capture.journal.flush().unwrap();
        assert_eq!(
            fs::read_to_string(storage.0.join("session-test.capture.jsonl"))
                .unwrap()
                .lines()
                .count(),
            4
        );
        assert_eq!(stopped_at_us(&json!({"uptime_us":900})), Some(900));
        assert_eq!(
            stopped_at_us(&json!({"stopped_at_us":0,"uptime_us":900})),
            Some(0)
        );
    }
    #[test]
    fn slow_backlog_yields_between_complete_batches_and_stop_drain_preserves_tail() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        service.capture = Some(storage.capture());
        let mut batches = 0;
        let drained = service.collect_with(COLLECT_TURN_BUDGET, |_| {
            // Model a successful but slow transport, not an error/timeout. A
            // count-only loop would retain the mutex for all 64 such replies.
            std::thread::sleep(Duration::from_millis(60));
            batches += 1;
            Ok(json!({"pending":120,"dropped":0,"samples":[
                {"kind":"imu","t_us":batches*10}, {"kind":"imu","t_us":batches*10+1}]}))
        });
        assert!(!drained);
        assert_eq!(
            service.next_collection_delay(service.sequence > 0),
            Duration::from_millis(2),
            "an intact queued backlog must resume promptly after releasing the turn"
        );
        assert_eq!(
            service.next_collection_delay(false),
            Duration::from_millis(40),
            "a failed transport must not busy-spin on cached pending"
        );
        service.capture.as_mut().unwrap().interrupted = true;
        assert_eq!(
            service.next_collection_delay(true),
            Duration::from_millis(40),
            "interrupted evidence collection must not spin"
        );
        service.capture.as_mut().unwrap().interrupted = false;
        assert!(
            (1..=3).contains(&batches),
            "slow backlog must yield after the elapsed budget"
        );
        let before = service.capture.as_ref().unwrap().session.raw_records.len();
        assert_eq!(
            before,
            batches as usize * 2,
            "an in-flight batch must be retained completely"
        );
        service.capture.as_mut().unwrap().stop_at_us = Some(batches * 10 + 10);
        let mut tail_batches = 0;
        let drained = service.collect_with(Duration::from_secs(1), |_| {
            batches += 1;
            tail_batches += 1;
            Ok(
                json!({"pending":if tail_batches==1{8}else{0},"dropped":0,"samples":[
                {"kind":"imu","t_us":batches*10},{"kind":"imu","t_us":batches*10+1}]}),
            )
        });
        assert!(drained);
        assert_eq!(
            service.next_collection_delay(true),
            Duration::from_millis(40),
            "a drained queue returns to normal pacing"
        );
        assert_eq!(tail_batches, 2);
        assert_eq!(
            service.capture.as_ref().unwrap().session.raw_records.len(),
            before + 1,
            "stop draining must retain the boundary record and exclude later preview samples"
        );
    }
    #[test]
    fn late_pre_start_observations_remain_raw_without_entering_projected_timeline() {
        let storage = TestStorage::new();
        let mut capture = storage.capture();
        capture.health_start =
            capture_health_snapshot(&json!({"started_at_us":10000,"uptime_us":10010}));
        let early = gnss(2400, 1000, 9000);
        append_record(&mut capture, early.clone()).unwrap();
        append_record(&mut capture, gnss(2400, 1050, 10000)).unwrap();
        assert_eq!(capture.session.raw_records[0], early);
        assert_eq!(capture.session.samples.len(), 1);
        assert_eq!(capture.session.samples[0].time_s, 0.0);
        let health = capture_health_record(&capture);
        assert_eq!(health["observations"]["pre_start_records"], 1);
        let mut warnings = vec![];
        add_capture_health_warnings(&[health], &mut warnings);
        assert!(warnings
            .iter()
            .any(|w| w.contains("late-published records")));
    }
    #[test]
    fn fixed_stop_tail_finishes_even_while_preview_queue_keeps_refilling() {
        let storage = TestStorage::new();
        let mut service = storage.service();
        let mut capture = storage.capture();
        capture.stop_at_us = Some(10);
        capture.stop_tail_pending = Some(10);
        service.capture = Some(capture);
        let mut batches = 0u64;
        let drained = service.collect_with_tail(Duration::from_secs(1), Some(10), |_| {
            let samples: Vec<_> = (1..=8)
                .map(|index| json!({"kind":"imu","t_us":batches*8+index}))
                .collect();
            batches += 1;
            // A busy live preview never becomes empty after the stop fence.
            Ok(json!({"pending":120,"dropped":0,"samples":samples}))
        });
        assert!(drained);
        assert_eq!(batches, 2);
        let capture = service.capture.as_ref().unwrap();
        assert_eq!(capture.session.raw_records.len(), 10);
        assert_eq!(capture.stop_tail_received, 10);
        assert!(!capture.interrupted);
        assert!(
            service.collect_with_tail(Duration::from_secs(1), Some(0), |_| panic!(
                "An empty stop tail must not consume preview"
            ))
        );
    }
    #[test]
    fn demo_exercises_three_complete_laps_and_sectors() {
        let session = demo_session();
        let result = crate::trajectory_adapter::analyze(
            &session.samples,
            &[],
            session.track.as_ref(),
            &session.drag,
        );
        assert_eq!(result.laps.len(), 3, "{:?}", result.warnings);
        assert!(result
            .laps
            .iter()
            .all(|l| l.valid && l.sectors_s.len() == 3));
        assert!(result.best_lap_s.is_some_and(|v| (80.0..90.0).contains(&v)));
        assert!(result.theoretical_best_s.is_some());
    }
    fn gnss(week: u64, tow: u64, arrival: u64) -> Value {
        json!({"kind":"gnss","gps_week":week,"gps_tow_ms":tow,"t_us":arrival,"valid":true})
    }
    #[test]
    fn receiver_epoch_ignores_usb_jitter_and_handles_week_rollover() {
        let mut clock = SampleClock::default();
        let first = clock
            .sample(&gnss(2400, 604_799_900, 100))
            .unwrap()
            .unwrap();
        let second = clock.sample(&gnss(2401, 0, 900_000)).unwrap().unwrap();
        assert_eq!(first.time_s, 0.0);
        assert!((second.time_s - 0.1).abs() < 1e-12);
        assert!(second.receiver_timed);
    }
    #[test]
    fn duplicate_epoch_is_not_a_new_fix_and_backward_clock_fails() {
        let mut clock = SampleClock::default();
        clock.sample(&gnss(2400, 1000, 100)).unwrap();
        assert!(clock.sample(&gnss(2400, 1000, 200)).unwrap().is_none());
        assert!(clock.sample(&gnss(2400, 900, 300)).is_err());
    }
    #[test]
    fn receiver_clock_never_falls_back_to_arrival_mid_session() {
        let mut clock = SampleClock::default();
        clock.sample(&gnss(2400, 1000, 100)).unwrap();
        assert!(clock
            .sample(&json!({"kind":"gnss","t_us":200,"valid":true}))
            .unwrap()
            .is_none());
        let next = clock.sample(&gnss(2400, 1100, 300)).unwrap().unwrap();
        assert!(!next.valid);
        assert!(next.receiver_timed);
        assert_eq!(next.time_s, 0.1);
    }
    #[test]
    fn arrival_only_capture_never_changes_domain_when_receiver_time_appears() {
        let mut clock = SampleClock::default();
        clock
            .sample(&json!({"kind":"gnss","t_us":100,"valid":true}))
            .unwrap();
        let next = clock.sample(&gnss(2400, 1000, 100100)).unwrap().unwrap();
        assert!(!next.receiver_timed);
        assert_eq!(next.time_s, 0.1);
    }
    #[test]
    fn request_frame_matches_postcard_layout() {
        let encoded = encode_request("poll");
        let mut raw = vec![0; encoded.len()];
        let n = cobs::decode(&encoded[..encoded.len() - 1], &mut raw).unwrap();
        assert_eq!(&raw[..n], b"\x03\x0d{\"op\":\"poll\"}");
    }
    #[test]
    fn response_rejects_trailing_or_truncated_payload() {
        for payload in [
            b"\x03\x02{}x".as_slice(),
            b"\x03\x05{}".as_slice(),
            b"\x02".as_slice(),
        ] {
            let mut encoded = vec![0; payload.len() + 2];
            let n = cobs::encode(payload, &mut encoded);
            assert!(decode_reply(&encoded[..n]).is_err());
        }
    }
    #[test]
    fn csv_roundtrip_preserves_missing_and_invalid() {
        let mut s = blank_session("test");
        s.samples = vec![
            Sample {
                time_s: 0.0,
                lat: Some(1.0),
                lon: Some(2.0),
                speed_mps: Some(3.0),
                valid: true,
                ..Default::default()
            },
            Sample {
                time_s: 0.1,
                valid: false,
                ..Default::default()
            },
        ];
        let parsed = import_csv(&export_csv(&s)).unwrap();
        assert!(parsed.samples[0].valid);
        assert!(!parsed.samples[1].valid);
        assert_eq!(parsed.samples[1].speed_mps, None);
    }
    #[test]
    fn import_rejects_reversed_clock_and_invalid_coordinates() {
        assert!(import_csv("time_s,lat,lon,speed_mps,valid\n1,0,0,2,true\n0,0,0,2,true").is_err());
        assert!(import_csv("time_s,lat,lon,speed_mps,valid\n0,91,0,2,true").is_err());
    }
    #[test]
    fn csv_without_validity_is_not_assumed_valid() {
        let s = import_csv("time_s,lat,lon,speed_mps\n0,0,0,2").unwrap();
        assert!(!s.samples[0].valid);
    }
    #[test]
    fn invalid_gate_and_drag_fail() {
        let mut t = demo_session().track.unwrap();
        t.start.width_m = 0.0;
        assert!(validate_track(&t).is_err());
        let d = DragConfig {
            start_kph: 100.0,
            end_kph: 50.0,
            ..DragConfig::default()
        };
        assert!(validate_drag(&d).is_err());
    }
    #[test]
    fn gpx_escapes_names_and_breaks_signal_gaps() {
        let mut s = blank_session("A&B <test>");
        s.samples = vec![
            Sample {
                time_s: 0.0,
                lat: Some(1.0),
                lon: Some(2.0),
                valid: true,
                ..Default::default()
            },
            Sample {
                time_s: 1.0,
                valid: false,
                ..Default::default()
            },
            Sample {
                time_s: 2.0,
                lat: Some(1.0),
                lon: Some(2.1),
                valid: true,
                ..Default::default()
            },
        ];
        let gpx = export_gpx(&s);
        assert!(gpx.contains("A&amp;B &lt;test&gt;"));
        assert_eq!(gpx.matches("<trkseg>").count(), 2);
        assert_eq!(gpx.matches("</trkseg>").count(), 2);
        assert!(!gpx.contains("<time>"));
    }
    #[test]
    fn ids_cannot_escape_storage() {
        assert!(safe_id("../preferences").is_err());
        assert!(safe_id("C:\\tmp").is_err());
        assert!(safe_id("session-123").is_ok());
    }
}
