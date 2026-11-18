//! Byte-preserving rover export and explicit external RTKLIB processing.
//! Corrected positions remain separate from receiver Doppler-based metrics.
use crate::{
    legacy_um980::BinaryFrame,
    model::{Sample, Session},
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, File, OpenOptions},
    io::{BufRead, BufReader, BufWriter, Read, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const LIMIT: u64 = 8 * 1024 * 1024 * 1024;
const POS_LIMIT: u64 = 256 * 1024 * 1024;
const RELEASE: &str = "RTKLIB Explorer 2.5.1 (62d4677ed8425a4e2748c6d390b500d1afb493fc)";
const BASE_WARNING: &str = "PPK requires overlapping base observations, matching signals and navigation data, and verified base coordinates and antenna metadata. A fixed ambiguity flag alone is not an accuracy guarantee.";
const POSITION_WARNING: &str = "PPK positions are a separate corrected trace. Receiver Doppler speed, vehicle acceleration, lap and drag metrics are unavailable for this result; no velocities were invented from position differences.";

#[derive(Default)]
pub(crate) struct RawAssembler {
    pub(crate) fragments: u64,
    current: Option<(u32, usize, u64, Vec<u8>)>,
    previous: Option<u32>,
    frames: u64,
    invalid: u64,
    incomplete: u64,
    sequence_gaps: u64,
    observations: u64,
    nonempty_observations: u64,
    message_counts: BTreeMap<u16, u64>,
    first_epoch: Option<(u16, u32)>,
    last_epoch: Option<(u16, u32)>,
    unqualified_epochs: u64,
}
fn u32at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}
impl RawAssembler {
    pub(crate) fn push(
        &mut self,
        at_ns: u64,
        payload: &[u8],
        writer: &mut impl Write,
    ) -> Result<(), String> {
        if payload.len() <= 24
            || payload.len() > 256
            || payload[..2] != [1, 1]
            || payload[2] & !3 != 0
            || payload[3] != 0
        {
            return Err("Invalid PPK fragment schema".into());
        }
        self.fragments += 1;
        let seq = u32at(payload, 4);
        let length = u32at(payload, 8) as usize;
        let offset = u32at(payload, 12) as usize;
        let received = u64::from_le_bytes(payload[16..24].try_into().unwrap());
        let raw = &payload[24..];
        let first = payload[2] & 1 != 0;
        let last = payload[2] & 2 != 0;
        if !(28..=65_563).contains(&length)
            || offset.checked_add(raw.len()).is_none_or(|end| end > length)
            || first != (offset == 0)
            || last != (offset + raw.len() == length)
            || received.checked_mul(1000) != Some(at_ns)
        {
            return Err("Invalid PPK fragment boundaries or timestamp".into());
        }
        if first {
            if self.current.take().is_some() {
                self.incomplete += 1;
            }
            if let Some(old) = self.previous {
                let advance = seq.wrapping_sub(old);
                if advance == 0 || advance > u32::MAX / 2 {
                    return Err("PPK frame sequence repeated or reversed".into());
                }
                self.sequence_gaps += u64::from(advance - 1);
            }
            self.previous = Some(seq);
            self.current = Some((seq, length, received, Vec::with_capacity(length)));
        }
        let Some((current_seq, current_len, current_received, bytes)) = self.current.as_mut()
        else {
            self.incomplete += 1;
            return Ok(());
        };
        if (*current_seq, *current_len, *current_received, bytes.len())
            != (seq, length, received, offset)
        {
            self.current = None;
            self.incomplete += 1;
            return Ok(());
        }
        bytes.extend_from_slice(raw);
        if last {
            let (_, _, _, bytes) = self.current.take().unwrap();
            let Ok(frame) = BinaryFrame::parse(&bytes) else {
                self.invalid += 1;
                return Ok(());
            };
            let id = frame.message_id();
            if id == 12 {
                let data = frame.payload();
                if data.len() < 4
                    || usize::try_from(u32at(data, 0))
                        .ok()
                        .and_then(|n| n.checked_mul(40))
                        .and_then(|n| n.checked_add(4))
                        != Some(data.len())
                {
                    self.invalid += 1;
                    return Ok(());
                }
                self.observations += 1;
                self.nonempty_observations += u64::from(u32at(data, 0) > 0);
                let head = frame.header_bytes();
                let epoch = (u16::from_le_bytes([head[10], head[11]]), u32at(head, 12));
                if head[8] == 0 && head[9] == 160 && epoch.1 < 604_800_000 {
                    self.first_epoch.get_or_insert(epoch);
                    self.last_epoch = Some(epoch);
                } else {
                    self.unqualified_epochs += 1;
                }
            }
            *self.message_counts.entry(id).or_default() += 1;
            self.frames += 1;
            writer
                .write_all(frame.wire_bytes())
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }
    pub(crate) fn finish(&mut self) {
        if self.current.take().is_some() {
            self.incomplete += 1;
        }
    }
    pub(crate) fn summary(&self) -> Value {
        json!({"kind":"ppk_raw_summary","format":"unicore","fragments":self.fragments,"complete_valid_frames":self.frames,"invalid_frames":self.invalid,"incomplete_fragments_or_frames":self.incomplete,"missing_frame_sequences":self.sequence_gaps,"observation_epochs":self.observations,"nonempty_observation_epochs":self.nonempty_observations,"message_counts":self.message_counts,"first_gps_epoch":self.first_epoch,"last_gps_epoch":self.last_epoch,"unqualified_observation_epochs":self.unqualified_epochs})
    }
    fn warnings(&self) -> Vec<String> {
        let mut w=vec![BASE_WARNING.into(),"This export covers one SD recording part. Download and convert other parts separately when a recording rotated; ephemerides may be in an earlier part. Supply matching navigation files explicitly.".into()];
        if self.invalid + self.incomplete + self.sequence_gaps > 0 {
            w.push(format!("Raw evidence has {} invalid native frames, {} incomplete fragments/frames and {} missing frame sequences. Only complete CRC-valid frames were exported; the original AEV remains unchanged.",self.invalid,self.incomplete,self.sequence_gaps));
        }
        if self.nonempty_observations == 0 {
            w.push("No nonempty rover observation epochs were captured. This file cannot produce a PPK trajectory.".into());
        }
        if !self
            .message_counts
            .keys()
            .any(|id| matches!(*id, 106..=110 | 112))
        {
            w.push("This part has no supported broadcast ephemeris frames; obtain matching navigation data or another recording part.".into());
        }
        if self.unqualified_epochs > 0 {
            w.push("Some observation epochs lack GPST/FINE receiver time qualification.".into());
        }
        if self.message_counts.contains_key(&2999) {
            w.push("BDS-3 BD3EPH frames are preserved, but the pinned converter does not decode message 2999. Its RINEX may omit those ephemerides.".into());
        }
        w
    }
}

fn string<'a>(q: &'a Value, key: &str) -> Result<&'a str, String> {
    q[key]
        .as_str()
        .filter(|s| !s.is_empty())
        .ok_or_else(|| format!("Missing {key}"))
}
fn check_cancel() -> Result<(), String> {
    if crate::service::processing_cancelled() {
        Err("PPK processing cancelled; source files and generated artifacts retained".into())
    } else {
        Ok(())
    }
}
fn file(path: &Path, limit: u64) -> Result<File, String> {
    let f = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if f.metadata().map_err(|e| e.to_string())?.len() > limit {
        return Err(format!("{} exceeds supported size", path.display()));
    }
    Ok(f)
}
fn fresh_directory(root: &Path, label: &str) -> Result<PathBuf, String> {
    fs::create_dir_all(root).map_err(|e| e.to_string())?;
    let n = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let dir = root.join(format!("aevia-{label}-{n}"));
    fs::create_dir(&dir).map_err(|e| e.to_string())?;
    dir.canonicalize().map_err(|e| e.to_string())
}
fn create(path: &Path) -> Result<File, String> {
    OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(path)
        .map_err(|e| format!("{}: {e}", path.display()))
}
fn write_json(path: &Path, value: &Value) -> Result<(), String> {
    let mut writer = BufWriter::new(create(path)?);
    serde_json::to_writer_pretty(&mut writer, value).map_err(|e| e.to_string())?;
    writer.flush().map_err(|e| e.to_string())?;
    writer.get_ref().sync_all().map_err(|e| e.to_string())
}
fn bounded_lines(mut reader: impl BufRead) -> impl Iterator<Item = Result<String, String>> {
    std::iter::from_fn(move || {
        let mut line = String::new();
        match Read::by_ref(&mut reader).take(16_385).read_line(&mut line) {
            Ok(0) => None,
            Ok(n) if n > 16_384 => Some(Err("Text input line exceeds 16 KiB".into())),
            Ok(_) => Some(Ok(line)),
            Err(error) => Some(Err(error.to_string())),
        }
    })
}

fn fingerprint(path: &Path) -> Result<Value, String> {
    let mut file = file(path, LIMIT)?;
    let size = file.metadata().map_err(|e| e.to_string())?.len();
    let mut hash = Sha256::new();
    let mut bytes = [0; 64 * 1024];
    loop {
        check_cancel()?;
        let n = file.read(&mut bytes).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        hash.update(&bytes[..n]);
    }
    Ok(json!({"path":path.to_string_lossy(),"bytes":size,"sha256":format!("{:x}",hash.finalize())}))
}
pub(crate) fn tool(name: &str, explicit: Option<&str>) -> Option<PathBuf> {
    if let Some(path) = explicit {
        return Path::new(path).is_file().then(|| PathBuf::from(path));
    }
    let exe = if cfg!(windows) {
        format!("{name}.exe")
    } else {
        name.to_owned()
    };
    let mut dirs = Vec::new();
    if let Some(path) = std::env::var_os("AEVIA_PPK_TOOLS") {
        dirs.push(PathBuf::from(path));
    }
    if let Ok(current) = std::env::current_exe() {
        if let Some(parent) = current.parent() {
            dirs.push(parent.join("ppk-tools"));
        }
    }
    dirs.push(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/ppk-tools/RTKLIB-EX-2.5.1-bin/RTKLIB_EX_2.5.1"),
    );
    if let Some(path) = std::env::var_os("PATH") {
        dirs.extend(std::env::split_paths(&path));
    }
    dirs.into_iter().map(|p| p.join(&exe)).find(|p| p.is_file())
}
fn raw_source(session: &Session) -> Option<PathBuf> {
    if !session
        .raw_records
        .iter()
        .any(|r| r["kind"] == "ppk_raw_summary" && r["fragments"].as_u64().unwrap_or(0) > 0)
    {
        return None;
    }
    session
        .raw_records
        .iter()
        .find(|r| r["kind"] == "raw_capture_file" && r["source"] != "usb-binary")
        .and_then(|r| r["path"].as_str())
        .map(PathBuf::from)
        .filter(|p| p.is_file())
}
pub(crate) fn status(session: Option<&Session>) -> Value {
    let raw = session.and_then(raw_source);
    let mut warnings = vec![BASE_WARNING.to_owned()];
    if raw.is_none() {
        warnings.push("Raw rover observations are stored on SD. Stop recording, download the matching device recording, then select that imported session. USB sensor captures alone do not contain PPK observations.".into());
    }
    json!({"tools":{"convbin":tool("convbin",None).map(|p|p.to_string_lossy().into_owned()),"rnx2rtkp":tool("rnx2rtkp",None).map(|p|p.to_string_lossy().into_owned())},"raw_available":raw.is_some(),"raw_source":raw.map(|p|p.to_string_lossy().into_owned()),"warnings":warnings,"recommended_tool":RELEASE})
}
fn invoke(executable: &Path, args: &[String], dir: &Path, label: &str) -> Result<Value, String> {
    check_cancel()?;
    let stdout = dir.join(format!("{label}.stdout.log"));
    let stderr = dir.join(format!("{label}.stderr.log"));
    let mut command = Command::new(executable);
    command
        .args(args)
        .current_dir(dir)
        .stdin(Stdio::null())
        .stdout(Stdio::from(create(&stdout)?))
        .stderr(Stdio::from(create(&stderr)?));
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    let mut child = command
        .spawn()
        .map_err(|e| format!("Cannot launch {}: {e}", executable.display()))?;
    let start = Instant::now();
    let status = loop {
        if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
            break status;
        }
        let solution_too_large =
            fs::metadata(dir.join("solution.pos")).is_ok_and(|m| m.len() > POS_LIMIT);
        let too_large = [&stdout, &stderr]
            .iter()
            .any(|p| fs::metadata(p).is_ok_and(|m| m.len() > 16 * 1024 * 1024));
        if crate::service::processing_cancelled()
            || start.elapsed() > Duration::from_secs(1800)
            || too_large
            || solution_too_large
        {
            let _ = child.kill();
            let _ = child.wait();
            return Err(format!("{label} cancelled or exceeded the 30 minute / 16 MiB log bound; artifacts retained in {}",dir.display()));
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let result = json!({"tool":fingerprint(executable)?,"arguments":args,"exit_code":status.code(),"elapsed_s":start.elapsed().as_secs_f64(),"stdout":stdout,"stderr":stderr});
    write_json(&dir.join(format!("{label}.process.json")), &result)?;
    if !status.success() {
        return Err(format!(
            "{label} failed ({}); logs retained in {}",
            status,
            dir.display()
        ));
    }
    Ok(result)
}
fn rinex_has_records(path: &Path) -> bool {
    let Ok(file) = file(path, LIMIT) else {
        return false;
    };
    let mut body = false;
    for line in bounded_lines(BufReader::new(file)).take(100_000) {
        let Ok(line) = line else {
            return false;
        };
        if body && !line.trim().is_empty() {
            return true;
        }
        if line.contains("END OF HEADER") {
            body = true;
        }
    }
    false
}
/// Inspect only our converter's RINEX 3 output, keeping one epoch's satellites.
/// A declared L type is not evidence of a populated carrier-phase measurement.
fn rinex_observation_quality(path: &Path) -> Result<Value, String> {
    let mut types = BTreeMap::<char, Vec<String>>::new();
    let mut current_system = None;
    let mut body = false;
    let mut epoch = false;
    let mut epochs = 0u64;
    let mut phase_values = 0u64;
    let mut code_phase_pairs = 0u64;
    let mut phase_satellites = BTreeSet::<String>::new();
    let mut pair_satellites = BTreeSet::<String>::new();
    let mut maximum_phase = 0usize;
    let mut maximum_pairs = 0usize;
    let mut qualified_epochs = 0u64;
    for (index, line) in bounded_lines(BufReader::new(file(path, LIMIT)?)).enumerate() {
        if index % 1024 == 0 {
            check_cancel()?;
        }
        let line = line?;
        if !line.is_ascii() {
            return Err("Non-ASCII generated RINEX observation data".into());
        }
        if index == 0
            && line[..9.min(line.len())]
                .trim()
                .parse::<f64>()
                .ok()
                .is_none_or(|version| !(3.0..4.0).contains(&version))
        {
            return Err("Readiness inspection supports generated RINEX 3 observations".into());
        }
        if !body {
            if line.contains("SYS / # / OBS TYPES") {
                if let Some(system) = line.chars().next().filter(|c| c.is_ascii_uppercase()) {
                    current_system = Some(system);
                }
                let system = current_system.ok_or("Missing RINEX observation system")?;
                types.entry(system).or_default().extend(
                    line[..60.min(line.len())]
                        .split_whitespace()
                        .filter(|token| {
                            token.len() == 3
                                && matches!(token.as_bytes()[0], b'C' | b'L' | b'D' | b'S')
                                && token.as_bytes()[1].is_ascii_digit()
                        })
                        .map(str::to_owned),
                );
            }
            if line.contains("END OF HEADER") {
                body = true;
            }
            continue;
        }
        if line.starts_with('>') {
            if epoch {
                maximum_phase = maximum_phase.max(phase_satellites.len());
                maximum_pairs = maximum_pairs.max(pair_satellites.len());
                qualified_epochs += u64::from(pair_satellites.len() >= 4);
            }
            let fields: Vec<_> = line.split_whitespace().collect();
            let flag = fields
                .get(7)
                .and_then(|v| v.parse::<u8>().ok())
                .ok_or("Invalid RINEX observation epoch")?;
            if flag > 1 {
                return Err(
                    "Unsupported event epoch in generated RINEX; review converter output".into(),
                );
            }
            epochs += 1;
            epoch = true;
            phase_satellites.clear();
            pair_satellites.clear();
            continue;
        }
        if line.trim().is_empty() {
            continue;
        }
        if !epoch || line.len() < 3 {
            return Err("Observation record without a RINEX epoch".into());
        }
        let system = line.as_bytes()[0] as char;
        let columns = types
            .get(&system)
            .ok_or("Observation system has no RINEX type declaration")?;
        let populated = |column: usize| -> bool {
            let start = 3 + column * 16;
            if start >= line.trim_end().len() {
                return false;
            }
            line[start..(start + 14).min(line.len())]
                .trim()
                .parse::<f64>()
                .is_ok_and(|value| value.is_finite() && value != 0.0)
        };
        for (column, kind) in columns
            .iter()
            .enumerate()
            .filter(|(_, kind)| kind.starts_with('L'))
        {
            if !populated(column) {
                continue;
            }
            phase_values += 1;
            phase_satellites.insert(line[..3].to_owned());
            let code = format!("C{}", &kind[1..]);
            if columns
                .iter()
                .position(|kind| kind == &code)
                .is_some_and(populated)
            {
                code_phase_pairs += 1;
                pair_satellites.insert(line[..3].to_owned());
            }
        }
    }
    if epoch {
        maximum_phase = maximum_phase.max(phase_satellites.len());
        maximum_pairs = maximum_pairs.max(pair_satellites.len());
        qualified_epochs += u64::from(pair_satellites.len() >= 4);
    }
    Ok(
        json!({"observation_epochs":epochs,"carrier_phase_values":phase_values,"code_phase_pairs":code_phase_pairs,"maximum_carrier_satellites_per_epoch":maximum_phase,"maximum_code_phase_satellites_per_epoch":maximum_pairs,"epochs_with_four_code_phase_satellites":qualified_epochs}),
    )
}
fn rinex_navigation_count(path: &Path) -> Result<u64, String> {
    let mut body = false;
    let mut count = 0;
    for (index, line) in bounded_lines(BufReader::new(file(path, LIMIT)?)).enumerate() {
        if index % 1024 == 0 {
            check_cancel()?;
        }
        let line = line?;
        let bytes = line.as_bytes();
        if body
            && bytes.len() > 3
            && matches!(bytes[0], b'G' | b'R' | b'E' | b'C' | b'J' | b'S' | b'I')
            && bytes[1].is_ascii_digit()
            && bytes[2].is_ascii_digit()
        {
            count += 1;
        }
        if line.contains("END OF HEADER") {
            body = true;
        }
    }
    Ok(count)
}
pub(crate) fn prepare(session: &Session, q: &Value) -> Result<Value, String> {
    let source = raw_source(session).ok_or(
        "Select a downloaded SD recording containing raw PPK fragments; USB-only capture has none",
    )?;
    let dir = fresh_directory(Path::new(string(q, "output_dir")?), "rover")?;
    let raw = dir.join("rover.unc");
    let obs = dir.join("rover.obs");
    let nav = dir.join("rover.nav");
    let manifest = dir.join("manifest.json");
    let mut writer = BufWriter::with_capacity(64 * 1024, create(&raw)?);
    let mut assembler = RawAssembler::default();
    let imported = crate::legacy::visit_records(&source, |kind, at, payload| {
        if kind == 25 {
            assembler.push(at, payload, &mut writer)?;
        }
        Ok(())
    })?;
    assembler.finish();
    writer.flush().map_err(|e| e.to_string())?;
    drop(writer);
    let mut warnings = assembler.warnings();
    let transport_loss = crate::legacy::transport_loss_summary(&imported.raw_records)?;
    crate::legacy::add_transport_warnings(&transport_loss, &mut warnings);
    let command_transport_lost = crate::legacy::command_transport_lost(&transport_loss);
    if command_transport_lost {
        warnings.push("Raw PPK command-port input loss was recorded. The retained .unc remains exportable, but this part is not marked ready even when its surviving frames pass CRC checks.".into());
    }
    let mut conversion = Value::Null;
    let valid_container = imported
        .raw_records
        .iter()
        .find(|r| r["kind"] == "legacy_capture")
        .is_some_and(|r| r["closed"] == true && r["missing_sequences"] == 0);
    if !valid_container {
        warnings.push("The source SD part is incomplete or has missing AEV records; exported rover evidence is incomplete.".into());
    }
    if let Some(converter) = tool("convbin", q["convbin_path"].as_str()) {
        if assembler.frames > 0 {
            let args = vec![
                "-r".into(),
                "unicore".into(),
                "-v".into(),
                "3.04".into(),
                "-od".into(),
                "-os".into(),
                "-o".into(),
                obs.to_string_lossy().into_owned(),
                "-n".into(),
                nav.to_string_lossy().into_owned(),
                raw.to_string_lossy().into_owned(),
            ];
            match invoke(&converter, &args, &dir, "convbin") {
                Ok(report) => conversion = report,
                Err(error) => warnings.push(error),
            }
        }
    } else {
        warnings.push("Unicore-capable convbin was not found. Raw .unc and its manifest are retained; select a converter executable.".into());
    }
    let observation_quality = match rinex_observation_quality(&obs) {
        Ok(value) => value,
        Err(error) => {
            warnings.push(format!("RINEX observation inspection unavailable: {error}"));
            Value::Null
        }
    };
    let navigation_records = rinex_navigation_count(&nav).unwrap_or(0);
    let phase_values = observation_quality["carrier_phase_values"]
        .as_u64()
        .unwrap_or(0);
    let maximum_phase = observation_quality["maximum_carrier_satellites_per_epoch"]
        .as_u64()
        .unwrap_or(0);
    if phase_values == 0 {
        warnings.push("Rover RINEX contains no populated carrier-phase observations. Code-only observations cannot support carrier-phase PPK; successful conversion alone is not readiness.".into());
    }
    if maximum_phase < 4 {
        warnings.push(format!("At most {maximum_phase} satellites have populated carrier phase in an epoch. Fewer than four tracked phase satellites are insufficient for this relative positioning workflow; no positioning accuracy is established."));
    }
    let ready = observation_quality["epochs_with_four_code_phase_satellites"]
        .as_u64()
        .unwrap_or(0)
        > 0
        && navigation_records > 0
        && assembler.nonempty_observations > 0
        && assembler.invalid + assembler.incomplete + assembler.sequence_gaps == 0
        && valid_container
        && !command_transport_lost;
    if !rinex_has_records(&obs) {
        warnings.push("Converter produced no usable RINEX observation body.".into());
    }
    if !rinex_has_records(&nav) {
        warnings.push("Converter produced no usable navigation body. Select matching external navigation data when processing.".into());
    }
    write_json(
        &manifest,
        &json!({"schema":1,"source_session_id":session.id,"source":fingerprint(&source)?,"rover_raw":fingerprint(&raw)?,"raw_summary":assembler.summary(),"container":imported.raw_records,"conversion":conversion,"observation_quality":observation_quality,"navigation_records":navigation_records,"gnss_transport_loss":transport_loss,"ready":ready,"warnings":warnings,"recommended_tool":RELEASE}),
    )?;
    Ok(
        json!({"directory":dir,"rover_raw":raw,"rover_obs":obs,"rover_nav":nav,"manifest":manifest,"observation_quality":observation_quality,"navigation_records":navigation_records,"gnss_transport_loss":transport_loss,"ready":ready,"warnings":warnings}),
    )
}

pub(crate) fn import_pos(path: &Path, source_id: Option<&str>) -> Result<Session, String> {
    let mut session = crate::service::blank_session("PPK · corrected positions");
    session.id = format!("ppk-{}", session.created_ms);
    session.source = "RTKLIB PPK result · position-only analysis".into();
    let reader = BufReader::new(file(path, POS_LIMIT)?);
    let mut header = false;
    let mut expected_columns = 15;
    let mut extra_header: Vec<String> = vec![];
    let mut origin = None;
    let mut previous = None;
    let mut counts = BTreeMap::<String, u64>::new();
    for (index, line) in bounded_lines(reader).enumerate() {
        if index % 1024 == 0 {
            check_cancel()?;
        }
        let line = line.map_err(|e| e.to_string())?;
        if line.len() > 16384 {
            return Err("Oversized .pos line".into());
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('%') {
            if line.contains("latitude(") || line.split_whitespace().nth(1) == Some("GPST") {
                let labels: Vec<_> = line.trim_start_matches('%').split_whitespace().collect();
                let expected = [
                    "GPST",
                    "latitude(deg)",
                    "longitude(deg)",
                    "height(m)",
                    "Q",
                    "ns",
                    "sdn(m)",
                    "sde(m)",
                    "sdu(m)",
                    "sdne(m)",
                    "sdeu(m)",
                    "sdun(m)",
                    "age(s)",
                    "ratio",
                ];
                let velocity = [
                    "vn(m/s)", "ve(m/s)", "vu(m/s)", "sdvn", "sdve", "sdvu", "sdvne", "sdveu",
                    "sdvun",
                ];
                if labels.len() < expected.len()
                    || labels[..expected.len()] != expected
                    || (!labels[expected.len()..].is_empty()
                        && labels[expected.len()..] != velocity)
                {
                    return Err("Unsupported .pos header column order or units; use the documented GPST LLH covariance output".into());
                }
                expected_columns = labels.len() + 1;
                extra_header = labels[expected.len()..]
                    .iter()
                    .map(|s| (*s).to_owned())
                    .collect();
                header = true;
            }
            continue;
        }
        if !header {
            return Err("Import requires RTKLIB .pos with GPST, latitude(deg), longitude(deg), height(m), Q and covariance columns; ECEF/DMS/UTC formats must be converted explicitly".into());
        }
        let columns: Vec<_> = line.split_whitespace().collect();
        if columns.len() != expected_columns {
            return Err(format!("Incomplete .pos row {}", index + 1));
        }
        let number = |i: usize| -> Result<f64, String> {
            columns
                .get(i)
                .ok_or("Missing .pos column")?
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| format!("Invalid numeric .pos row {}", index + 1))
        };
        let (week, tow) = if columns[0].contains('/') {
            calendar_gpst(columns[0], columns[1])?
        } else {
            let week = number(0)?;
            if week.fract() != 0.0 || !(0.0..=10000.0).contains(&week) {
                return Err("Invalid GPS week".into());
            }
            (week as u32, number(1)?)
        };
        if !(0.0..604800.0).contains(&tow) {
            return Err("Invalid GPS time of week".into());
        }
        let epoch = week as f64 * 604800.0 + tow;
        if previous.is_some_and(|old| epoch <= old) {
            return Err("PPK epochs repeat or reverse; original solution is retained".into());
        }
        previous = Some(epoch);
        let origin = *origin.get_or_insert(epoch);
        let lat = number(2)?;
        let lon = number(3)?;
        let alt = number(4)?;
        let q = number(5)?;
        let satellites = number(6)?;
        if lat.abs() > 90.0
            || lon.abs() > 180.0
            || q.fract() != 0.0
            || !(0.0..=255.0).contains(&q)
            || satellites.fract() != 0.0
            || !(0.0..=255.0).contains(&satellites)
        {
            return Err("Invalid PPK coordinates/quality/satellite count".into());
        }
        let q = q as u8;
        let quality = match q {
            0 => "none",
            1 => "fixed",
            2 => "float",
            3 => "sbas",
            4 => "dgps",
            5 => "single",
            6 => "ppp",
            7 => "dead_reckoning",
            _ => "unknown",
        };
        *counts.entry(quality.into()).or_default() += 1;
        let sigma = [number(7)?, number(8)?, number(9)?];
        if sigma.iter().any(|v| *v < 0.0) {
            return Err("Negative PPK standard deviation".into());
        }
        let cross = [number(10)?, number(11)?, number(12)?];
        if sigma
            .iter()
            .chain(cross.iter())
            .any(|x| !(x * x).is_finite())
        {
            return Err("PPK covariance exceeds finite range".into());
        }
        let signed_square = |x: f64| x.abs() * x;
        let metadata = json!({"q":q,"status":quality,"sd_n_m":sigma[0],"sd_e_m":sigma[1],"sd_u_m":sigma[2],"signed_sqrt_covariance_ne_eu_un_m":cross,"covariance_ne_eu_un_m2":cross.map(signed_square),"age_s":number(13)?,"ratio":number(14)?,"gps_tow_s":tow,"velocity_source":"unavailable","extra_column_labels":extra_header,"extra_columns":&columns[15..]});
        session.samples.push(Sample {
            time_s: epoch - origin,
            lat: Some(lat),
            lon: Some(lon),
            altitude_m: Some(alt),
            valid: (1..=6).contains(&q),
            gps_week: Some(week),
            gps_tow_ms: Some((tow * 1000.0).floor() as u32),
            satellites: Some(satellites as u8),
            ppk_quality: Some(metadata),
            ..Sample::default()
        });
        if session.samples.len() > 250_000 {
            return Err("PPK import exceeds 250,000 solution epochs; split processing by time without discarding original .pos evidence".into());
        }
    }
    if session.samples.is_empty() {
        return Err("Processor produced no position solutions; inspect its logs and overlapping rover/base/navigation inputs".into());
    }
    let warnings = vec![POSITION_WARNING.to_owned(), BASE_WARNING.to_owned()];
    session.provenance = json!({"kind":"ppk","parent_session_id":source_id,"quality_counts":counts,"solution_file":fingerprint(path)?,"time_system":"GPST","height_reference":"RTKLIB LLH output (verify ellipsoidal height setting)","warnings":warnings});
    session.analysis.warnings = warnings;
    session.sample_count = session.samples.len();
    session.duration_s = session.samples.last().unwrap().time_s;
    Ok(session)
}
fn calendar_gpst(date: &str, time: &str) -> Result<(u32, f64), String> {
    let d: Vec<_> = date
        .split('/')
        .map(str::parse::<i32>)
        .collect::<Result<_, _>>()
        .map_err(|_| "Invalid GPST date")?;
    let t: Vec<_> = time
        .split(':')
        .map(str::parse::<f64>)
        .collect::<Result<_, _>>()
        .map_err(|_| "Invalid GPST time")?;
    if d.len() != 3
        || t.len() != 3
        || !(1980..=2100).contains(&d[0])
        || !(1..=12).contains(&d[1])
        || !(0.0..24.0).contains(&t[0])
        || !(0.0..60.0).contains(&t[1])
        || !(0.0..60.0).contains(&t[2])
    {
        return Err("Invalid GPST calendar epoch".into());
    }
    let leap = |y: i32| y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
    let months = [
        31,
        if leap(d[0]) { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    if d[2] < 1 || d[2] > months[(d[1] - 1) as usize] {
        return Err("Invalid GPST calendar day".into());
    }
    let days = (1980..d[0])
        .map(|y| if leap(y) { 366 } else { 365 })
        .sum::<i32>()
        + months[..(d[1] - 1) as usize].iter().sum::<i32>()
        + d[2]
        - 6;
    if days < 0 {
        return Err("Date precedes GPS epoch".into());
    }
    Ok((
        (days / 7) as u32,
        (days % 7) as f64 * 86400.0 + t[0] * 3600.0 + t[1] * 60.0 + t[2],
    ))
}
fn base_position(path: &Path) -> Result<[f64; 3], String> {
    for line in bounded_lines(BufReader::new(file(path, LIMIT)?)).take(10_000) {
        let line = line.map_err(|e| e.to_string())?;
        if line.contains("END OF HEADER") {
            break;
        }
        if line.contains("APPROX POSITION XYZ") {
            let values: Vec<_> = line
                .split_whitespace()
                .take(3)
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
            if let Ok(result) = values.try_into() {
                return valid_ecef(result);
            }
        }
    }
    Err(
        "Base RINEX lacks usable APPROX POSITION XYZ. Supply verified base_ecef_m coordinates"
            .into(),
    )
}
fn valid_ecef(xyz: [f64; 3]) -> Result<[f64; 3], String> {
    let norm = xyz.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !xyz.iter().all(|v| v.is_finite()) || !(6e6..7e6).contains(&norm) {
        return Err("Base ECEF coordinates are invalid or unavailable".into());
    }
    Ok(xyz)
}
pub(crate) fn run(q: &Value) -> Result<(Session, Value), String> {
    let rover = Path::new(string(q, "rover_obs")?)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    let base = Path::new(string(q, "base_obs")?)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    let nav = q["nav_paths"]
        .as_array()
        .filter(|v| !v.is_empty())
        .ok_or("At least one navigation file is required")?
        .iter()
        .map(|v| {
            v.as_str()
                .map(PathBuf::from)
                .ok_or("Invalid navigation path".to_owned())
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|p| p.canonicalize().map_err(|e| e.to_string()))
        .collect::<Result<Vec<_>, _>>()?;
    if nav.len() > 32 {
        return Err("At most 32 navigation files may be processed at once".into());
    }
    if !rinex_has_records(&rover)
        || !rinex_has_records(&base)
        || nav.iter().any(|p| !rinex_has_records(p))
    {
        return Err("Rover/base/navigation RINEX files must contain data records".into());
    }
    let mut inputs = vec![fingerprint(&rover)?, fingerprint(&base)?];
    for path in &nav {
        inputs.push(fingerprint(path)?);
    }
    let explicit = q["base_ecef_m"].as_array();
    let xyz = if let Some(values) = explicit {
        if values.len() != 3 {
            return Err("base_ecef_m requires three coordinates".into());
        }
        valid_ecef([
            values[0].as_f64().ok_or("Invalid base X")?,
            values[1].as_f64().ok_or("Invalid base Y")?,
            values[2].as_f64().ok_or("Invalid base Z")?,
        ])?
    } else {
        base_position(&base)?
    };
    let processor = tool("rnx2rtkp", q["rnx2rtkp_path"].as_str())
        .ok_or("rnx2rtkp was not found; select the RTKLIB Explorer executable")?;
    let dir = fresh_directory(Path::new(string(q, "output_dir")?), "ppk")?;
    let config = dir.join("processing.conf");
    let mut configuration = String::new();
    if let Some(path) = q["config_path"].as_str().filter(|s| !s.is_empty()) {
        let mut f = file(Path::new(path), 1024 * 1024)?;
        f.read_to_string(&mut configuration)
            .map_err(|e| e.to_string())?;
        inputs.push(fingerprint(Path::new(path))?);
    }
    // Keep user processing options, with an explicit output representation and
    // base coordinate source that the importer can represent without guessing.
    configuration.push_str("\npos1-posmode=kinematic\npos1-soltype=combined\nout-solformat=llh\nout-outhead=on\nout-outopt=on\nout-outvel=off\nout-timesys=gpst\nout-timeform=tow\nout-timendec=6\nout-degform=deg\nout-height=ellipsoidal\n");
    create(&config)?
        .write_all(configuration.as_bytes())
        .map_err(|e| e.to_string())?;
    let solution = dir.join("solution.pos");
    let report = dir.join("report.json");
    let mut args = vec![
        "-k".into(),
        config.to_string_lossy().into_owned(),
        "-p".into(),
        "2".into(),
        "-c".into(),
        "-r".into(),
        xyz[0].to_string(),
        xyz[1].to_string(),
        xyz[2].to_string(),
        "-o".into(),
        solution.to_string_lossy().into_owned(),
        rover.to_string_lossy().into_owned(),
        base.to_string_lossy().into_owned(),
    ];
    args.extend(nav.iter().map(|p| p.to_string_lossy().into_owned()));
    let process = invoke(&processor, &args, &dir, "rnx2rtkp")?;
    let mut warnings = vec![BASE_WARNING.to_owned(), POSITION_WARNING.to_owned()];
    if explicit.is_none() {
        warnings.push("Base coordinates came from the base RINEX APPROX POSITION XYZ header. Verify their survey/reference frame and antenna corrections before interpreting the result.".into());
    }
    let mut provenance = json!({"schema":1,"source_session_id":q.get("source_id"),"inputs":inputs,"base_ecef_m":xyz,"base_coordinate_source":if explicit.is_some(){"user-supplied"}else{"base-RINEX-header"},"effective_configuration":fingerprint(&config)?,"process":process,"warnings":warnings});
    // Write the report even when a successful processor emits no solutions.
    write_json(&report, &provenance)?;
    let mut session = import_pos(&solution, q["source_id"].as_str())?;
    provenance["report_path"] = json!(report);
    session.raw_records.push(
        json!({"kind":"raw_capture_file","format":"RTKLIB-POS","source":"ppk","path":solution}),
    );
    session.provenance["processing"] = provenance;
    session.provenance["warnings"] = json!(warnings);
    Ok((
        session,
        json!({"solution_path":solution,"report_path":report,"warnings":warnings}),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Temp(PathBuf);
    impl Temp {
        fn new() -> Self {
            Self(fresh_directory(&std::env::temp_dir(), "ppk-test").unwrap())
        }
    }
    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    fn native_frame(observations: usize, tow: u32) -> Vec<u8> {
        let mut bytes = vec![0; 24 + 4 + 40 * observations];
        bytes[..3].copy_from_slice(&[0xaa, 0x44, 0xb5]);
        bytes[4..6].copy_from_slice(&12u16.to_le_bytes());
        bytes[6..8].copy_from_slice(&((4 + 40 * observations) as u16).to_le_bytes());
        bytes[9] = 160;
        bytes[10..12].copy_from_slice(&2400u16.to_le_bytes());
        bytes[12..16].copy_from_slice(&tow.to_le_bytes());
        bytes[24..28].copy_from_slice(&(observations as u32).to_le_bytes());
        for index in 0..observations {
            let row = &mut bytes[28 + index * 40..68 + index * 40];
            row[2..4].copy_from_slice(&((index + 1) as u16).to_le_bytes());
            row[4..12].copy_from_slice(&20_000_000f64.to_le_bytes());
            row[12..20].copy_from_slice(&100_000_000f64.to_le_bytes());
            row[24..28].copy_from_slice(&1.5f32.to_le_bytes());
            row[28..30].copy_from_slice(&4500u16.to_le_bytes());
            row[32..36].copy_from_slice(&10f32.to_le_bytes());
            row[36..40].copy_from_slice(&((1u32 << 10) | (1 << 12)).to_le_bytes());
        }
        let mut crc = 0u32;
        for &b in &bytes {
            crc ^= u32::from(b);
            for _ in 0..8 {
                crc = (crc >> 1) ^ if crc & 1 != 0 { 0xedb88320 } else { 0 };
            }
        }
        bytes.extend_from_slice(&crc.to_le_bytes());
        bytes
    }
    fn chunks(bytes: &[u8]) -> Vec<crate::firmware_ppk::Chunk> {
        let mut decoder = crate::firmware_ppk::Decoder::new();
        bytes
            .iter()
            .filter_map(|&b| decoder.push(b, 10_000))
            .collect()
    }
    #[test]
    fn actual_firmware_fragments_reassemble_byte_exactly_through_aev() {
        let temp = Temp::new();
        let path = temp.0.join("receiver.aev");
        let frame = native_frame(32, 100_000);
        let chunks = chunks(&frame);
        assert!(chunks.len() > 4);
        let mut writer = crate::evidence::Writer::create(&path, 1, 1024 * 1024).unwrap();
        for chunk in chunks {
            writer
                .append(25, chunk.received_us() * 1000, chunk.as_bytes())
                .unwrap();
        }
        writer.close(11_000_000, 0).unwrap();
        drop(writer);
        let mut raw = Vec::new();
        let mut assembler = RawAssembler::default();
        let session = crate::legacy::visit_records(&path, |kind, at, payload| {
            if kind == 25 {
                assembler.push(at, payload, &mut raw)?;
            }
            Ok(())
        })
        .unwrap();
        assembler.finish();
        assert_eq!(raw, frame);
        assert_eq!(assembler.frames, 1);
        assert_eq!(assembler.nonempty_observations, 1);
        let summary = session
            .raw_records
            .iter()
            .find(|r| r["kind"] == "ppk_raw_summary")
            .unwrap();
        assert_eq!(summary["complete_valid_frames"], 1);
    }
    #[test]
    fn missing_fragment_and_bad_native_crc_are_not_exported() {
        let original = native_frame(32, 100_000);
        let chunks = chunks(&original);
        let mut assembler = RawAssembler::default();
        let mut output = vec![];
        for (index, chunk) in chunks.iter().enumerate() {
            if index != 1 {
                assembler
                    .push(10_000_000, chunk.as_bytes(), &mut output)
                    .unwrap();
            }
        }
        assembler.finish();
        assert!(output.is_empty());
        assert!(assembler.incomplete > 0);
        let mut corrupt = original;
        corrupt[30] ^= 1;
        let mut assembler = RawAssembler::default();
        for chunk in self::chunks(&corrupt) {
            assembler
                .push(10_000_000, chunk.as_bytes(), &mut output)
                .unwrap();
        }
        assert_eq!(assembler.invalid, 1);
        assert!(output.is_empty());
    }
    const HEADER:&str="% (lat/lon/height=WGS84/ellipsoidal)\n% GPST latitude(deg) longitude(deg) height(m) Q ns sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio\n";
    #[test]
    fn pos_preserves_fixed_float_covariance_lineage_without_inventing_velocity() {
        let temp = Temp::new();
        let path = temp.0.join("solution.pos");
        fs::write(&path,format!("{HEADER}2400 100 -33 151 42 1 12 .01 .02 .03 -.004 .005 -.006 .2 4.5\n2400 100.2 -33.00001 151.00001 43 2 11 .1 .2 .3 -.04 .05 -.06 .3 1.2\n")).unwrap();
        let session = import_pos(&path, Some("original-spp")).unwrap();
        assert_eq!(session.provenance["parent_session_id"], "original-spp");
        assert_eq!(session.provenance["quality_counts"]["fixed"], 1);
        assert_eq!(session.provenance["quality_counts"]["float"], 1);
        assert!(session
            .samples
            .iter()
            .all(|s| s.speed_mps.is_none() && s.heading_deg.is_none() && !s.receiver_timed));
        let q = session.samples[0].ppk_quality.as_ref().unwrap();
        assert_eq!(q["ratio"], 4.5);
        assert_eq!(q["covariance_ne_eu_un_m2"][0], -0.000016);
    }
    #[test]
    fn pos_rejects_reordered_header_repeated_epoch_and_unknown_units() {
        let temp = Temp::new();
        let path = temp.0.join("solution.pos");
        let row = "2400 100 -33 151 42 2 12 .01 .02 .03 0 0 0 .2 4.5\n";
        for text in [
            format!("{}{row}", HEADER.replace("sdn(m) sde(m)", "sde(m) sdn(m)")),
            format!("{HEADER}{row}{row}"),
            format!("{}{row}", HEADER.replace("GPST", "UTC")),
        ] {
            fs::write(&path, text).unwrap();
            assert!(import_pos(&path, None).is_err());
        }
        assert_eq!(calendar_gpst("1980/01/06", "00:00:00").unwrap(), (0, 0.0));
        assert!(calendar_gpst("2025/02/29", "00:00:00").is_err());
    }
    #[test]
    fn pinned_converter_reads_native_unicore_observations_when_installed() {
        let Some(converter) = tool("convbin", None) else {
            return;
        };
        let temp = Temp::new();
        let raw = temp.0.join("synthetic-codec-fixture.unc");
        let mut bytes = native_frame(8, 100_000);
        bytes.extend_from_slice(&native_frame(8, 100_200));
        fs::write(&raw, bytes).unwrap();
        let obs = temp.0.join("synthetic.obs");
        let nav = temp.0.join("synthetic.nav");
        invoke(
            &converter,
            &[
                "-r".into(),
                "unicore".into(),
                "-od".into(),
                "-os".into(),
                "-o".into(),
                obs.to_string_lossy().into_owned(),
                "-n".into(),
                nav.to_string_lossy().into_owned(),
                raw.to_string_lossy().into_owned(),
            ],
            &temp.0,
            "convbin",
        )
        .unwrap();
        assert!(rinex_has_records(&obs));
        let quality = rinex_observation_quality(&obs).unwrap();
        assert_eq!(quality["observation_epochs"], 2);
        assert_eq!(quality["carrier_phase_values"], 16);
        assert_eq!(quality["maximum_code_phase_satellites_per_epoch"], 8);
        let text = fs::read_to_string(obs).unwrap();
        assert!(text.contains("G01"));
        assert!(text.contains("20000000.000"));
        // Synthetic encoding fixture is not a positioning/accuracy fixture.
        assert!(!rinex_has_records(&nav));
    }
    #[test]
    fn declared_carrier_type_with_empty_values_is_not_ppk_ready() {
        let temp = Temp::new();
        let path = temp.0.join("observations.obs");
        let header=format!("{:60}RINEX VERSION / TYPE\n{:60}SYS / # / OBS TYPES\n{:60}END OF HEADER\n> 2026 11 18 02 32 48.8000000  0  4\n","     3.04           OBSERVATION DATA    M: Mixed","G    3 C1C L1C S1C","");
        for (phase, expected) in [("", 0), ("0.000", 0), ("100000000.000", 4)] {
            let rows = (1..=4)
                .map(|sat| {
                    format!(
                        "G{sat:02}{:14.3}  {phase:>14}  {:14.3}  \n",
                        20_000_000.0, 45.0
                    )
                })
                .collect::<String>();
            fs::write(&path, format!("{header}{rows}")).unwrap();
            let quality = rinex_observation_quality(&path).unwrap();
            assert_eq!(quality["carrier_phase_values"], expected);
            assert_eq!(
                quality["epochs_with_four_code_phase_satellites"],
                u64::from(expected == 4)
            );
        }
        let rows = (1..=4)
            .map(|sat| {
                format!(
                    "G{sat:02}{:14.3}  {:14.3}  {:14.3}  \n",
                    if sat == 1 { 0.0 } else { 20_000_000.0 },
                    100_000_000.0,
                    45.0
                )
            })
            .collect::<String>();
        fs::write(&path, format!("{header}{rows}")).unwrap();
        let quality = rinex_observation_quality(&path).unwrap();
        assert_eq!(quality["maximum_carrier_satellites_per_epoch"], 4);
        assert_eq!(quality["maximum_code_phase_satellites_per_epoch"], 3);
        assert_eq!(quality["epochs_with_four_code_phase_satellites"], 0);
    }
}
