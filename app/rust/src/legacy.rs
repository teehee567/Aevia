//! Verified import of the original AEVLOG1 capture container. Page/record CRCs,
//! sequence IDs and UART offsets are checked before navigation is published.
//! The source binary is retained alongside the imported session by the caller.
use crate::{
    legacy_um980::{BestNavXyz, BinaryDecoder},
    model::Session,
    service::{blank_session, SampleClock},
};
use serde_json::json;
use std::{
    fs::File,
    io::{BufReader, Read},
    path::Path,
};

const PAGE: usize = 512;
const CRC_TABLE: [u32; 256] = crc_table();
const fn crc_table() -> [u32; 256] {
    let mut table = [0; 256];
    let mut i = 0;
    while i < 256 {
        let mut v = i as u32;
        let mut j = 0;
        while j < 8 {
            v = (v >> 1) ^ (0xedb8_8320u32 & 0u32.wrapping_sub(v & 1));
            j += 1;
        }
        table[i] = v;
        i += 1;
    }
    table
}
fn crc32(bytes: &[u8]) -> u32 {
    let mut value = !0u32;
    for &b in bytes {
        value = (value >> 8) ^ CRC_TABLE[((value ^ u32::from(b)) & 255) as usize];
    }
    !value
}
fn u16_at(b: &[u8], i: usize) -> u16 {
    u16::from_le_bytes(b[i..i + 2].try_into().unwrap())
}
fn u32_at(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}
fn u64_at(b: &[u8], i: usize) -> u64 {
    u64::from_le_bytes(b[i..i + 8].try_into().unwrap())
}

pub fn import(path: &Path) -> Result<Session, String> {
    let file = File::open(path).map_err(|e| e.to_string())?;
    let size = file.metadata().map_err(|e| e.to_string())?.len();
    parse(
        BufReader::with_capacity(64 * 1024, file),
        size,
        path.file_stem()
            .and_then(|v| v.to_str())
            .unwrap_or("Legacy recording"),
    )
}
fn parse(reader: impl Read, size: u64, name: &str) -> Result<Session, String> {
    parse_with_records(reader, size, name, |_, _, _| Ok(()))
}
pub(crate) fn visit_records(
    path: &Path,
    visitor: impl FnMut(u8, u64, &[u8]) -> Result<(), String>,
) -> Result<Session, String> {
    let file = File::open(path).map_err(|e| e.to_string())?;
    let size = file.metadata().map_err(|e| e.to_string())?.len();
    if size > 8 * 1024 * 1024 * 1024 {
        return Err("Raw capture exceeds 8 GiB supported limit".into());
    }
    parse_with_records(
        BufReader::with_capacity(64 * 1024, file),
        size,
        "Raw evidence",
        visitor,
    )
}
fn parse_with_records(
    mut reader: impl Read,
    size: u64,
    name: &str,
    mut visitor: impl FnMut(u8, u64, &[u8]) -> Result<(), String>,
) -> Result<Session, String> {
    if size == 0 || !size.is_multiple_of(512) {
        return Err("Legacy recording has an empty or partial page".into());
    }
    let mut session = blank_session(name);
    let mut page = [0u8; PAGE];
    let mut pending = Vec::with_capacity(1024);
    let mut identity = None;
    let mut closed = false;
    let mut previous = None;
    let mut last_kind = 0;
    let mut missing = 0u64;
    let mut record_count = 0u64;
    let mut uses_driving_estimates = false;
    let mut setup_gates = None;
    let mut mounted = None;
    let mut display_duration = None;

    let mut origin_at = None;
    let mut duration_s = 0.0f64;
    let mut decoders = [BinaryDecoder::<16384>::new(), BinaryDecoder::<16384>::new()];
    let mut offsets = [None, None];
    let mut clock = SampleClock::default();
    let mut uart_missing = 0u64;
    let mut decode_errors = 0u64;
    let mut ppk = crate::ppk::RawAssembler::default();
    let mut timed_imu_count = 0u64;
    let mut imu_interval_sum_ns = 0u64;
    let mut previous_imu_capture_ns = None;
    let mut imu_support_gap_count = 0u64;
    let mut imu_support_gap_ns = 0u64;
    let mut imu_support_overlap_count = 0u64;
    let mut imu_support_overlap_ns = 0u64;
    let mut imu_nonadvancing_count = 0u64;
    let mut imu_invalid_support_count = 0u64;
    let mut imu_invalid_record_count = 0u64;
    let mut mapped_gnss_count = 0u64;
    let mut unmapped_gnss_count = 0u64;
    let mut first_timed_imu = None;
    let mut last_timed_imu = None;
    for index in 0..size / 512 {
        if index % 128 == 0 && crate::service::processing_cancelled() {
            return Err("Import cancelled".into());
        }
        reader.read_exact(&mut page).map_err(|e| e.to_string())?;
        if closed {
            return Err("Data follows the closed recording page".into());
        }
        let length = u16_at(&page, 16) as usize;
        let flags = u16_at(&page, 18);
        let key = (u32_at(&page, 8), u64_at(&page, 20));
        if &page[..8] != b"AEVLOG1\0"
            || u32_at(&page, 12) as u64 != index
            || length > 480
            || flags > 1
            || page[28 + length..508].iter().any(|&b| b != 0)
            || crc32(&page[..508]) != u32_at(&page, 508)
        {
            return Err(format!(
                "Legacy page {index} failed header, padding or CRC verification"
            ));
        }
        if identity.is_some_and(|old| old != key) {
            return Err("Legacy session identity changed within the file".into());
        }
        identity = Some(key);
        closed = flags == 1;
        pending.extend_from_slice(&page[28..28 + length]);
        while pending.len() >= 24 {
            let kind = pending[0];
            let n = u16_at(&pending, 2) as usize;
            let seq = u64_at(&pending, 4);
            let at_ns = u64_at(&pending, 12);
            let origin = *origin_at.get_or_insert(at_ns);
            duration_s = duration_s.max(at_ns.saturating_sub(origin) as f64 / 1e9);
            let total = 28 + n;
            if pending[1] != 1 || n > 256 || u32_at(&pending, 20) != 0 {
                return Err("Unsupported legacy record header".into());
            }
            if pending.len() < total {
                break;
            }
            if crc32(&pending[..24 + n]) != u32_at(&pending, 24 + n) {
                return Err(format!("Legacy record {seq} failed CRC verification"));
            }
            if previous.is_some_and(|old| seq <= old)
                || matches!(last_kind, 22 | 255)
                || (last_kind == 15 && kind != 255)
            {
                return Err("Legacy sequence/end ordering is corrupt".into());
            }
            missing += seq - previous.map_or(0, |old| old + 1);
            previous = Some(seq);
            last_kind = kind;
            record_count += 1;
            let payload = &pending[24..24 + n];
            visitor(kind, at_ns, payload)?;
            match kind {
                27 => {
                    use crate::display::Configure;
                    let (command, tail) = postcard::take_from_bytes::<Configure>(payload)
                        .map_err(|e| e.to_string())?;
                    if !tail.is_empty() {
                        return Err("Trailing driving configuration bytes".into());
                    }
                    match command {
                        Configure::Begin {
                            start_centi_kph,
                            end_centi_kph,
                            distance_mm,
                            rollout,
                            mounted: confirmed,
                            gates,
                        } if setup_gates.is_none() && gates <= 16 => {
                            uses_driving_estimates = true;
                            session.samples.clear();
                            setup_gates = Some(usize::from(gates));
                            mounted = Some(confirmed);
                            session.drag = crate::model::DragConfig {
                                start_kph: f64::from(start_centi_kph) / 100.,
                                end_kph: f64::from(end_centi_kph) / 100.,
                                distance_m: f64::from(distance_mm) / 1000.,
                                rollout,
                            };
                        }
                        Configure::Gate { index, gate }
                            if setup_gates.is_some_and(|count| usize::from(index) < count)
                                && usize::from(index)
                                    == session
                                        .track
                                        .as_ref()
                                        .map_or(0, |t| 1 + t.sectors.len()) =>
                        {
                            let g = crate::model::Gate {
                                lat: f64::from(gate.lat_e7) / 1e7,
                                lon: f64::from(gate.lon_e7) / 1e7,
                                heading_deg: f64::from(gate.heading_cdeg) / 100.,
                                width_m: f64::from(gate.width_cm) / 100.,
                            };
                            if let Some(t) = &mut session.track {
                                t.sectors.push(g);
                            } else {
                                session.track = Some(crate::model::Track {
                                    id: "recorded-track".into(),
                                    name: "Recorded gates".into(),
                                    start: g,
                                    sectors: Vec::new(),
                                });
                            }
                        }
                        _ => return Err("Invalid driving configuration sequence".into()),
                    }
                }
                26 => {
                    let value = crate::driving::decode(payload)?;
                    if value["uptime_us"].as_u64() != Some(at_ns / 1000) {
                        return Err(
                            "Driving snapshot timestamp disagrees with record header".into()
                        );
                    }
                    if !uses_driving_estimates {
                        session.samples.clear();
                        uses_driving_estimates = true;
                    }
                    let mut record = crate::driving::capture_record(&value)
                        .ok_or("Missing driving snapshot fields")?;
                    if value["recording"] == false {
                        record["kind"] = json!("fused_summary");
                        display_duration = value["stopped_at_us"]
                            .as_u64()
                            .zip(value["started_at_us"].as_u64())
                            .and_then(|(stop, start)| stop.checked_sub(start))
                            .map(|us| us as f64 * 1e-6);
                    } else if let Some(sample) = crate::driving::sample(&record) {
                        // A repeated display read can refer to the same underlying
                        // measurement. Preserve the frame and publish its epoch once.
                        if session
                            .samples
                            .last()
                            .is_none_or(|last| sample.time_s > last.time_s)
                        {
                            session.samples.push(sample);
                        }
                    }
                    session.raw_records.push(record);
                }
                25 => {
                    ppk.push(at_ns, payload, &mut std::io::sink())?;
                }
                23 => {
                    let record = timed_imu_record(payload)?;
                    if record["captured_ns"].as_u64() != Some(at_ns) {
                        return Err("IMU capture timestamp disagrees with record header".into());
                    }
                    timed_imu_count += 1;
                    let interval = u64::from(u32_at(payload, 8));
                    imu_interval_sum_ns = imu_interval_sum_ns.saturating_add(interval);
                    imu_invalid_record_count += u64::from(record["valid"] != true);
                    let support_start = at_ns.checked_sub(interval);
                    if interval == 0 || support_start.is_none() {
                        imu_invalid_support_count += 1;
                    }
                    if let Some(previous) = previous_imu_capture_ns {
                        imu_nonadvancing_count += u64::from(at_ns <= previous);
                        if let Some(start) = support_start.filter(|_| interval > 0) {
                            if start > previous {
                                imu_support_gap_count += 1;
                                imu_support_gap_ns =
                                    imu_support_gap_ns.saturating_add(start - previous);
                            } else if start < previous {
                                imu_support_overlap_count += 1;
                                imu_support_overlap_ns =
                                    imu_support_overlap_ns.saturating_add(previous - start);
                            }
                        }
                    }
                    previous_imu_capture_ns = Some(at_ns);
                    first_timed_imu.get_or_insert_with(|| record.clone());
                    last_timed_imu = Some(record);
                    // All per-sample vectors/timing stay in the retained binary.
                    // Keep bounded first/last diagnostics instead of duplicating
                    // a high-rate multi-hour IMU capture in the UI session JSON.
                }
                24 => {
                    let record = mapped_gnss_record(payload)?;
                    if record["mapped_ns"].is_null() {
                        unmapped_gnss_count += 1;
                    } else {
                        mapped_gnss_count += 1;
                    }
                    if let Some(sample) = clock.sample(&record)?.filter(|_| !uses_driving_estimates)
                    {
                        session.samples.push(sample);
                    }
                }
                22 => {
                    if payload.len() != 4 {
                        return Err("Invalid V2 Mini close record".into());
                    }
                    session.raw_records.push(
                        json!({"kind":"health","dropped":u32_at(payload,0),"t_us":at_ns/1000}),
                    );
                }
                20 => {
                    let record = gnss_record(payload)?;
                    if let Some(sample) = clock.sample(&record)?.filter(|_| !uses_driving_estimates)
                    {
                        session.samples.push(sample);
                    }
                }
                21 => {
                    if payload.len() != 61 {
                        return Err("Invalid V2 Mini IMU record length".into());
                    }
                    // Original intervals/vectors remain in the retained .aev
                    // for calibrated reprocessing; no invented body axes.
                }
                1 => {
                    let metadata = serde_json::from_slice::<serde_json::Value>(payload)
                        .map_err(|e| format!("Invalid legacy metadata: {e}"))?;
                    session
                        .raw_records
                        .push(json!({"kind":"legacy_metadata","value":metadata}));
                }
                3 | 4 => {
                    if payload.len() < 24
                        || payload[0] != 1
                        || payload[1] & !1 != 0
                        || payload[2..4] != [0, 0]
                        || payload[22..24] != [0, 0]
                        || u16_at(payload, 20) as usize != payload.len() - 24
                    {
                        return Err("Invalid legacy UART record".into());
                    }
                    let port = if kind == 3 { 1 } else { 0 };
                    let offset = u64_at(payload, 12);
                    let receipt = u64_at(payload, 4);
                    if let Some(next) = offsets[port] {
                        if offset < next {
                            return Err("Legacy UART offsets moved backwards".into());
                        }
                        if offset != next {
                            uart_missing += offset - next;
                            decoders[port].reset();
                        }
                    }
                    offsets[port] = Some(offset + (payload.len() - 24) as u64);
                    for &b in &payload[24..] {
                        match decoders[port].push(b) {
                            Ok(Some(frame)) if frame.message_id() == 240 => {
                                let nav = frame.best_nav_xyz().map_err(|e| {
                                    format!("Malformed historical BESTNAVXYZ: {e:?}")
                                })?;
                                let record = nav_record(nav, receipt);
                                if let Some(sample) =
                                    clock.sample(&record)?.filter(|_| !uses_driving_estimates)
                                {
                                    session.samples.push(sample);
                                }
                            }
                            Err(_) => decode_errors += 1,
                            _ => {}
                        }
                    }
                }
                10 | 255 => {
                    if payload.len() != 41 || payload[0] != 1 {
                        return Err("Unsupported legacy health payload".into());
                    }
                    session.raw_records.push(json!({"kind":"health","t_us":at_ns/1000,"dropped":u32_at(payload,9),"imu_errors":u32_at(payload,17),"gnss_uart_errors":u32_at(payload,21),"gnss_parse_errors":u32_at(payload,25)}));
                }
                _ => {}
            }
            pending.drain(..total);
            if session.samples.len() > 2_000_000 {
                return Err("Recording exceeds two million GNSS samples".into());
            }
        }
    }
    if closed && (!pending.is_empty() || !matches!(last_kind, 22 | 255)) {
        return Err("Closed legacy capture lacks its complete end marker".into());
    }
    if timed_imu_count > 0 || mapped_gnss_count > 0 || unmapped_gnss_count > 0 {
        session.raw_records.push(json!({"kind":"capture_timing_summary","schema":1,"timed_imu_records":timed_imu_count,"mapped_gnss_records":mapped_gnss_count,"unmapped_gnss_records":unmapped_gnss_count,
            "imu_interval_sum_ns":imu_interval_sum_ns,"imu_support_gap_count":imu_support_gap_count,"imu_support_gap_ns":imu_support_gap_ns,
            "imu_support_overlap_count":imu_support_overlap_count,"imu_support_overlap_ns":imu_support_overlap_ns,
            "imu_nonadvancing_count":imu_nonadvancing_count,"imu_invalid_support_count":imu_invalid_support_count,"imu_invalid_record_count":imu_invalid_record_count,
            "continuity_scope":"Adjacent raw captured_ns - interval_ns support; effective_end_ns is retained separately and is not used for this check.",
            "first_imu":first_timed_imu,"last_imu":last_timed_imu,"per_sample_evidence":"Retained AEVLOG1 binary: IMU kind23 and GNSS kind24; all original vectors and timing are preserved."}));
    }
    if ppk.fragments > 0 {
        ppk.finish();
        session.raw_records.push(ppk.summary());
    }
    session.raw_records.push(json!({"kind":"legacy_capture","format":"AEVLOG1","closed":closed,"records":record_count,"missing_sequences":missing,"uart_missing_bytes":uart_missing,"decoder_errors":decode_errors,"trailing_record_bytes":pending.len(),"raw_preserved":true}));
    session.sample_count = session.samples.len();
    session.duration_s = duration_s.max(session.samples.last().map(|s| s.time_s).unwrap_or(0.0));
    session.source = "Imported AEVLOG1 receiver observations; original raw capture retained".into();
    if closed
        && setup_gates.is_some_and(|expected| {
            expected != session.track.as_ref().map_or(0, |t| 1 + t.sectors.len())
        })
    {
        return Err("Incomplete driving gate configuration".into());
    }
    if uses_driving_estimates {
        session.duration_s = display_duration.unwrap_or(session.duration_s);
        session.provenance = json!({"kind":"display_v2","estimator":"DrivingEstimator","nominal_hz":5,"native_capture":true,"mount_confirmed":mounted,"warnings":["On-device driving estimates use development timing and installation calibration."]});
        session.analysis = crate::driving::analysis(&session);
    }
    add_timing_warnings(&session.raw_records, &mut session.analysis.warnings);
    let transport = transport_loss_summary(&session.raw_records)?;
    add_transport_warnings(&transport, &mut session.analysis.warnings);
    Ok(session)
}

/// Terminal transport counters are cumulative from this recording's admission,
/// including earlier rotated parts. Repeated snapshots must never be summed.
pub(crate) fn transport_loss_summary(
    records: &[serde_json::Value],
) -> Result<serde_json::Value, String> {
    let mut out = json!({"data":null,"command":null,"scope":"Session-window cumulative totals at this part's close; may include earlier rotated parts. Repeated snapshots are not summed."});
    for record in records.iter().filter(|r| r["kind"] == "legacy_metadata") {
        let Some(loss) = record["value"].get("gnss_transport_loss") else {
            continue;
        };
        let port = loss["port"]
            .as_str()
            .filter(|p| matches!(*p, "data" | "command"))
            .ok_or("Invalid terminal GNSS transport-loss port")?;
        if out[port].is_null() {
            out[port] = json!({"counter_regressed":false,"observation_dropped":null,"observation_counter_available":true});
        }
        for key in [
            "uart_errors",
            "dropped_reads",
            "dropped_bytes",
            "observation_dropped",
        ] {
            if key == "observation_dropped" && loss.get(key).is_none() {
                out[port]["observation_counter_available"] = json!(false);
                continue;
            }
            let next = loss[key]
                .as_u64()
                .ok_or("Invalid terminal GNSS transport-loss counter")?;
            let previous = out[port][key].as_u64();
            if previous.is_some_and(|old| next < old) {
                out[port]["counter_regressed"] = json!(true);
            }
            out[port][key] = json!(previous.map_or(next, |old| old.max(next)));
        }
    }
    Ok(out)
}
pub(crate) fn command_transport_lost(summary: &serde_json::Value) -> bool {
    ["uart_errors", "dropped_reads", "dropped_bytes"]
        .iter()
        .any(|key| summary["command"][*key].as_u64().is_some_and(|n| n > 0))
        || summary["command"]["counter_regressed"] == true
}
pub(crate) fn add_transport_warnings(summary: &serde_json::Value, warnings: &mut Vec<String>) {
    let mut unknown = Vec::new();
    for (port, role) in [("data", "SPP/PPS"), ("command", "raw PPK/commands")] {
        let loss = &summary[port];
        if loss.is_null() {
            unknown.push(port);
            continue;
        }
        let errors = loss["uart_errors"].as_u64().unwrap_or(0);
        let reads = loss["dropped_reads"].as_u64().unwrap_or(0);
        let bytes = loss["dropped_bytes"].as_u64().unwrap_or(0);
        if errors > 0 || reads > 0 || bytes > 0 {
            warnings.push(format!("Recorded GNSS {port}-port ({role}) transport loss: {errors} physical UART errors, {reads} rejected read chunks, {bytes} known dropped bytes. UART-error byte counts are unknown. These session-window totals may include earlier rotated parts and are separate from SD queue drops."));
        }
        if let Some(count) = loss["observation_dropped"]
            .as_u64()
            .filter(|count| *count > 0)
        {
            warnings.push(format!("Recorded GNSS {port}-port observation-channel loss: {count} complete parsed SPP observations were rejected by the owned channel. This is an observation count, separate from UART bytes and SD queue drops; no byte count is inferred."));
        }
        if port == "data" && loss["observation_counter_available"] != true {
            warnings.push("Parsed SPP observation-channel loss was not recorded in this older metadata; its count remains unknown.".into());
        }
        if loss["counter_regressed"] == true {
            warnings.push(format!("GNSS {port}-port terminal counters regressed within this part; maxima are retained and transport completeness is unconfirmed."));
        }
    }
    if !unknown.is_empty() {
        warnings.push(format!("GNSS transport-loss totals are unavailable for {} port(s) in this AEV part; input loss is unknown. Zero SD queue drops do not establish lossless UART input.",unknown.join(" / ")));
    }
}

pub(crate) fn add_timing_warnings(records: &[serde_json::Value], warnings: &mut Vec<String>) {
    for summary in records
        .iter()
        .filter(|r| r["kind"] == "capture_timing_summary")
    {
        let count = |key: &str| summary[key].as_u64().unwrap_or(0);
        let gaps = count("imu_support_gap_count");
        let overlaps = count("imu_support_overlap_count");
        let nonadvancing = count("imu_nonadvancing_count");
        let invalid = count("imu_invalid_support_count");
        let invalid_records = count("imu_invalid_record_count");
        if gaps + overlaps + nonadvancing + invalid + invalid_records > 0 {
            warnings.push(format!("Retained IMU capture intervals are discontinuous or invalid: {gaps} gaps ({} ns), {overlaps} overlaps ({} ns), {nonadvancing} repeated/reversed capture times, {invalid} invalid interval supports and {invalid_records} invalid records. These are derived from adjacent raw capture intervals, independently of SD queue-drop counters; the number or cause of missing samples is not inferred.",count("imu_support_gap_ns"),count("imu_support_overlap_ns")));
        }
    }
}

fn gnss_record(payload: &[u8]) -> Result<serde_json::Value, String> {
    if payload.len() != 94 {
        return Err("Invalid V2 Mini GNSS record length".into());
    }
    if payload[93] > 1 {
        return Err("Invalid V2 Mini GNSS validity flag".into());
    }
    let mut record = json!({"kind":"gnss","t_us":u64_at(payload,0),"gps_week":u32_at(payload,8),"gps_tow_ms":u32_at(payload,12),"latency_ms":u32_at(payload,88),"satellites":payload[92],"valid":payload[93]==1});
    for (i, name) in [
        "lat",
        "lon",
        "alt_m",
        "speed_mps",
        "heading_deg",
        "vertical_mps",
        "hacc_m",
        "vacc_m",
        "sacc_mps",
    ]
    .iter()
    .enumerate()
    {
        let value = f64::from_le_bytes(payload[16 + i * 8..24 + i * 8].try_into().unwrap());
        record[*name] = json!(if value.is_finite() { Some(value) } else { None });
        if !value.is_finite() {
            record["valid"] = json!(false);
        }
    }
    Ok(record)
}

pub(crate) fn timed_imu_record(payload: &[u8]) -> Result<serde_json::Value, String> {
    if payload.len() != 101 || payload[100] > 1 {
        return Err("Invalid nanosecond IMU record layout".into());
    }
    let captured_ns = u64_at(payload, 0);
    let interval_ns = u32_at(payload, 8);
    let effective_end_ns = u64_at(payload, 12);
    if payload[100] == 1
        && (interval_ns == 0
            || effective_end_ns < u64::from(interval_ns)
            || effective_end_ns > captured_ns)
    {
        return Err("Invalid nanosecond IMU sample support".into());
    }
    let vector = |offset: usize| -> Result<[f64; 3], String> {
        let values = std::array::from_fn(|axis| {
            f64::from_le_bytes(
                payload[offset + axis * 8..offset + (axis + 1) * 8]
                    .try_into()
                    .unwrap(),
            )
        });
        if values.iter().all(|v| v.is_finite()) {
            Ok(values)
        } else {
            Err("Nonfinite nanosecond IMU vector".into())
        }
    };
    Ok(
        json!({"kind":"imu","schema":1,"captured_ns":captured_ns,"interval_ns":interval_ns,"effective_end_ns":effective_end_ns,"timing_uncertainty_ns":u64_at(payload,20),
        "accel":vector(28)?,"gyro":vector(52)?,"aligned_gyro":vector(76)?,"valid":payload[100]==1}),
    )
}

pub(crate) fn mapped_gnss_record(payload: &[u8]) -> Result<serde_json::Value, String> {
    if payload.len() != 115 || payload[94] > 1 {
        return Err("Invalid mapped GNSS record layout".into());
    }
    let mut record = gnss_record(&payload[..94])?;
    let present = payload[94] == 1;
    let mapped_ns = u64_at(payload, 95);
    let uncertainty = u64_at(payload, 103);
    let generation = u32_at(payload, 111);
    if !present && (mapped_ns != 0 || uncertainty != 0 || generation != 0) {
        return Err("Absent GNSS mapping contains nonzero metadata".into());
    }
    record["mapped_ns"] = json!(present.then_some(mapped_ns));
    record["mapping_uncertainty_ns"] = json!(present.then_some(uncertainty));
    record["mapping_generation"] = json!(present.then_some(generation));
    Ok(record)
}

fn nav_record(nav: BestNavXyz, receipt: u64) -> serde_json::Value {
    let (lat, lon, alt) = geodetic(nav.position_ecef_m);
    let (sin_lat, cos_lat) = lat.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon.to_radians().sin_cos();
    let [vx, vy, vz] = nav.velocity_ecef_mps;
    let east = -sin_lon * vx + cos_lon * vy;
    let north = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz;
    let vertical = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz;
    let correction =
        nav.differential_age_s.is_finite() && (0.0..=2.0).contains(&nav.differential_age_s);
    let valid = nav.header.gps_epoch_ms().is_some()
        && nav.position_valid()
        && nav.velocity_valid()
        && nav.solution_age_s.is_finite()
        && (0.0..=2.0).contains(&nav.solution_age_s)
        && (nav.position_type == 16 || correction)
        && (matches!(nav.velocity_type, 8 | 16) || correction);
    let position_sigma = nav
        .position_sigma_m
        .into_iter()
        .map(f64::from)
        .fold(0.0, f64::max);
    let velocity_sigma = nav
        .velocity_sigma_mps
        .into_iter()
        .map(f64::from)
        .fold(0.0, f64::max);
    let mut record = json!({"kind":"gnss","t_us":receipt,"gps_week":nav.header.week,"gps_tow_ms":nav.header.milliseconds_of_week,
        "lat":lat,"lon":lon,"alt_m":alt,"speed_mps":east.hypot(north),"heading_deg":east.atan2(north).to_degrees().rem_euclid(360.0),"vertical_mps":vertical,
        "hacc_m":position_sigma,"vacc_m":position_sigma,"sacc_mps":velocity_sigma,"satellites":nav.satellites_used,"latency_ms":(nav.velocity_latency_s.max(0.0)as f64*1000.0).round()as u64,"valid":valid});
    if nav.header.gps_epoch_ms().is_none() {
        let map = record.as_object_mut().unwrap();
        map.remove("gps_week");
        map.remove("gps_tow_ms");
    }
    record
}
fn geodetic([x, y, z]: [f64; 3]) -> (f64, f64, f64) {
    const A: f64 = 6_378_137.0;
    const E2: f64 = 6.694_379_990_141_316_5e-3;
    let p = x.hypot(y);
    let lon = y.atan2(x);
    let mut lat = z.atan2(p * (1.0 - E2));
    for _ in 0..8 {
        let n = A / (1.0 - E2 * lat.sin().powi(2)).sqrt();
        lat = (z + E2 * n * lat.sin()).atan2(p);
    }
    let n = A / (1.0 - E2 * lat.sin().powi(2)).sqrt();
    let alt = if lat.cos().abs() > 1e-8 {
        p / lat.cos() - n
    } else {
        z.abs() - n * (1.0 - E2)
    };
    (lat.to_degrees(), lon.to_degrees(), alt)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn actual_firmware_encoder_matches_host_timing_decoders() {
        let captured_ns = 9_007_199_254_741_123u64;
        let imu = crate::sample::Sample::Imu {
            t_us: captured_ns / 1000,
            dt_us: 1355,
            accel: [1.25, -2.5, 9.8],
            gyro: [0.1, -0.2, 0.3],
            valid: true,
            captured_ns,
            interval_ns: 1_355_933,
            effective_end_ns: captured_ns - 1_500_123,
            timing_uncertainty_ns: 1_200_999,
            aligned_gyro: [0.101, -0.202, 0.303],
        };
        let mut buffer = [0u8; 128];
        let (kind, at_ns, len) = crate::firmware_record_codec::encode(imu, &mut buffer);
        assert_eq!((kind, at_ns, len), (23, captured_ns, 101));
        let decoded = timed_imu_record(&buffer[..len]).unwrap();
        let usb = serde_json::to_value(imu).unwrap();
        for field in [
            "captured_ns",
            "interval_ns",
            "effective_end_ns",
            "timing_uncertainty_ns",
            "accel",
            "gyro",
            "aligned_gyro",
            "valid",
        ] {
            assert_eq!(decoded[field], usb[field], "{field}");
        }
        for mapped in [false, true] {
            let gnss = crate::sample::Sample::Gnss {
                t_us: 9_007_199_274_741,
                gps_week: 2400,
                gps_tow_ms: 1000,
                lat: -33.8,
                lon: 150.8,
                alt_m: 35.5,
                speed_mps: 10.0,
                heading_deg: 90.0,
                vertical_mps: 0.2,
                hacc_m: 0.5,
                vacc_m: 0.8,
                sacc_mps: 0.05,
                latency_ms: 0,
                satellites: 25,
                valid: true,
                mapped_ns: mapped.then_some(9_007_199_224_741_112),
                mapping_uncertainty_ns: mapped.then_some(1_111_333),
                mapping_generation: mapped.then_some(7),
            };
            let (kind, at_ns, len) = crate::firmware_record_codec::encode(gnss, &mut buffer);
            assert_eq!((kind, at_ns, len), (24, 9_007_199_274_741_000, 115));
            assert_eq!(
                mapped_gnss_record(&buffer[..len]).unwrap(),
                serde_json::to_value(gnss).unwrap()
            );
        }
    }
    fn record(kind: u8, seq: u64, payload: &[u8]) -> Vec<u8> {
        let mut bytes = vec![0; 24];
        bytes[0] = kind;
        bytes[1] = 1;
        bytes[2..4].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        bytes[4..12].copy_from_slice(&seq.to_le_bytes());
        bytes[12..20].copy_from_slice(&(seq * 100_000_000).to_le_bytes());
        bytes.extend_from_slice(payload);
        let crc = crc32(&bytes);
        bytes.extend_from_slice(&crc.to_le_bytes());
        bytes
    }
    fn page(records: &[Vec<u8>]) -> [u8; 512] {
        let data = records.concat();
        let mut page = [0u8; 512];
        page[..8].copy_from_slice(b"AEVLOG1\0");
        page[8..12].copy_from_slice(&1u32.to_le_bytes());
        page[16..18].copy_from_slice(&(data.len() as u16).to_le_bytes());
        page[18] = 1;
        page[28..28 + data.len()].copy_from_slice(&data);
        let crc = crc32(&page[..508]);
        page[508..].copy_from_slice(&crc.to_le_bytes());
        page
    }
    fn fix(tow: u32) -> Vec<u8> {
        let mut bytes = vec![0u8; 94];
        bytes[..8].copy_from_slice(&(tow as u64 * 1000).to_le_bytes());
        bytes[8..12].copy_from_slice(&2400u32.to_le_bytes());
        bytes[12..16].copy_from_slice(&tow.to_le_bytes());
        for (i, v) in [0.0f64, 0.0, 0.0, 10.0, 90.0, 0.0, 1.0, 1.0, 0.1]
            .iter()
            .enumerate()
        {
            bytes[16 + i * 8..24 + i * 8].copy_from_slice(&v.to_le_bytes());
        }
        bytes[92] = 20;
        bytes[93] = 1;
        bytes
    }
    fn record_at(kind: u8, seq: u64, at_ns: u64, payload: &[u8]) -> Vec<u8> {
        let mut bytes = record(kind, seq, payload);
        bytes[12..20].copy_from_slice(&at_ns.to_le_bytes());
        let n = bytes.len();
        let crc = crc32(&bytes[..n - 4]);
        bytes[n - 4..].copy_from_slice(&crc.to_le_bytes());
        bytes
    }
    fn timed_imu() -> Vec<u8> {
        let captured = 9_007_199_254_741_123u64;
        let mut bytes = vec![0u8; 101];
        bytes[..8].copy_from_slice(&captured.to_le_bytes());
        bytes[8..12].copy_from_slice(&1_355_933u32.to_le_bytes());
        bytes[12..20].copy_from_slice(&(captured - 1_500_123).to_le_bytes());
        bytes[20..28].copy_from_slice(&1_200_999u64.to_le_bytes());
        for (i, value) in [1.25f64, -2.5, 9.8, 0.1, -0.2, 0.3, 0.101, -0.202, 0.303]
            .into_iter()
            .enumerate()
        {
            bytes[28 + i * 8..36 + i * 8].copy_from_slice(&value.to_le_bytes());
        }
        bytes[100] = 1;
        bytes
    }
    fn mapped_fix() -> Vec<u8> {
        let mut bytes = fix(1000);
        bytes.resize(115, 0);
        bytes[94] = 1;
        bytes[95..103].copy_from_slice(&9_007_199_224_741_112u64.to_le_bytes());
        bytes[103..111].copy_from_slice(&1_111_333u64.to_le_bytes());
        bytes[111..].copy_from_slice(&7u32.to_le_bytes());
        bytes[..8].copy_from_slice(&9_007_199_274_741u64.to_le_bytes());
        bytes
    }
    #[test]
    fn versioned_timing_records_preserve_nanoseconds_vectors_and_mapping() {
        let imu = timed_imu();
        let gnss = mapped_fix();
        let p = page(&[
            record_at(23, 0, u64_at(&imu, 0), &imu),
            record_at(24, 1, u64_at(&gnss, 0) * 1000, &gnss),
            record_at(22, 2, u64_at(&gnss, 0) * 1000 + 1, &0u32.to_le_bytes()),
        ]);
        let s = parse(std::io::Cursor::new(p), 512, "timing").unwrap();
        assert_eq!(s.samples.len(), 1);
        let sample = &s.samples[0];
        assert_eq!(sample.mapped_ns, Some(9_007_199_224_741_112));
        assert_eq!(sample.mapping_uncertainty_ns, Some(1_111_333));
        assert_eq!(sample.mapping_generation, Some(7));
        assert_eq!(
            sample.time_s, 0.0,
            "mapping must not replace receiver-epoch analysis time"
        );
        let timing = s
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["timed_imu_records"], 1);
        assert_eq!(timing["mapped_gnss_records"], 1);
        let record = &timing["first_imu"];
        assert_eq!(record["captured_ns"].as_u64(), Some(9_007_199_254_741_123));
        assert_eq!(record["interval_ns"].as_u64(), Some(1_355_933));
        assert_eq!(
            record["effective_end_ns"].as_u64(),
            Some(9_007_199_253_241_000)
        );
        assert_eq!(record["timing_uncertainty_ns"].as_u64(), Some(1_200_999));
        assert_eq!(record["accel"], json!([1.25, -2.5, 9.8]));
        assert_eq!(record["gyro"], json!([0.1, -0.2, 0.3]));
        assert_eq!(record["aligned_gyro"], json!([0.101, -0.202, 0.303]));
        assert_ne!(
            record["gyro"], record["aligned_gyro"],
            "raw gyro must never be replaced by alignment output"
        );
    }
    fn interval_fixture(captures: &[(u64, u32)]) -> Session {
        let mut bytes = Vec::new();
        let mut page = crate::format::PageBuilder::new();
        let mut index = 0;
        for (sequence, &(at, interval)) in captures.iter().enumerate() {
            let mut payload = timed_imu();
            payload[..8].copy_from_slice(&at.to_le_bytes());
            payload[8..12].copy_from_slice(&interval.to_le_bytes());
            payload[12..20].copy_from_slice(&(at - 1_000_000).to_le_bytes());
            let record = record_at(23, sequence as u64, at, &payload);
            let mut cursor = 0;
            while cursor < record.len() {
                cursor += page.append(&record[cursor..]);
                if page.is_full() {
                    bytes.extend_from_slice(page.finish(1, index, 1, false));
                    index += 1;
                    page.clear();
                }
            }
        }
        let end = record_at(
            22,
            captures.len() as u64,
            captures.last().unwrap().0,
            &0u32.to_le_bytes(),
        );
        let mut cursor = 0;
        while cursor < end.len() {
            cursor += page.append(&end[cursor..]);
            if page.is_full() {
                bytes.extend_from_slice(page.finish(1, index, 1, false));
                index += 1;
                page.clear();
            }
        }
        bytes.extend_from_slice(page.finish(1, index, 1, true));
        let size = bytes.len() as u64;
        parse(
            std::io::Cursor::new(bytes),
            size,
            "Interval support fixture",
        )
        .unwrap()
    }
    #[test]
    fn interval_gaps_and_overlaps_warn_even_with_zero_sd_queue_drops() {
        let session = interval_fixture(&[
            (10_000_000, 1_000_000),
            (12_000_000, 1_000_000),
            (12_500_000, 1_000_000),
        ]);
        let timing = session
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["imu_interval_sum_ns"], 3_000_000);
        assert_eq!(timing["imu_support_gap_count"], 1);
        assert_eq!(timing["imu_support_gap_ns"], 1_000_000);
        assert_eq!(timing["imu_support_overlap_count"], 1);
        assert_eq!(timing["imu_support_overlap_ns"], 500_000);
        assert_eq!(timing["imu_nonadvancing_count"], 0);
        assert!(session
            .raw_records
            .iter()
            .any(|r| r["kind"] == "health" && r["dropped"] == 0));
        assert!(session
            .analysis
            .warnings
            .iter()
            .any(|w| w.contains("1 gaps")
                && w.contains("1 overlaps")
                && w.contains("not inferred")));
        let mut reanalysis = vec![];
        add_timing_warnings(&session.raw_records, &mut reanalysis);
        add_transport_warnings(
            &transport_loss_summary(&session.raw_records).unwrap(),
            &mut reanalysis,
        );
        assert_eq!(session.analysis.warnings, reanalysis);
    }
    #[test]
    fn adjacent_raw_support_is_contiguous_despite_separate_effective_delay() {
        let session = interval_fixture(&[
            (10_000_000, 1_000_000),
            (11_000_000, 1_000_000),
            (12_000_000, 1_000_000),
        ]);
        let timing = session
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["imu_support_gap_count"], 0);
        assert_eq!(timing["imu_support_overlap_count"], 0);
        assert!(!session
            .analysis
            .warnings
            .iter()
            .any(|warning| warning.contains("IMU capture intervals")));
        assert_ne!(
            timing["first_imu"]["captured_ns"],
            timing["first_imu"]["effective_end_ns"]
        );
        let repeated = interval_fixture(&[
            (10_000_000, 1_000_000),
            (10_000_000, 1_000_000),
            (9_000_000, 1_000_000),
        ]);
        let timing = repeated
            .raw_records
            .iter()
            .find(|r| r["kind"] == "capture_timing_summary")
            .unwrap();
        assert_eq!(timing["imu_nonadvancing_count"], 2);
        assert_eq!(timing["imu_support_overlap_count"], 2);
    }
    #[test]
    fn absent_mapping_is_null_and_malformed_versioned_metadata_is_rejected() {
        let mut absent = fix(1000);
        absent.resize(115, 0);
        let value = mapped_gnss_record(&absent).unwrap();
        assert!(
            value["mapped_ns"].is_null()
                && value["mapping_uncertainty_ns"].is_null()
                && value["mapping_generation"].is_null()
        );
        absent[95] = 1;
        assert!(mapped_gnss_record(&absent).is_err());
        let mut imu = timed_imu();
        imu[100] = 2;
        assert!(timed_imu_record(&imu).is_err());
        imu = timed_imu();
        imu[8..12].fill(0);
        assert!(timed_imu_record(&imu).is_err());
        imu = timed_imu();
        imu[28..36].copy_from_slice(&f64::NAN.to_le_bytes());
        assert!(timed_imu_record(&imu).is_err());
        assert!(timed_imu_record(&timed_imu()[..100]).is_err());
        assert!(mapped_gnss_record(&mapped_fix()[..114]).is_err());
    }
    #[test]
    fn older_microsecond_imu_and_gnss_records_still_import() {
        let mut imu = vec![0u8; 61];
        imu[..8].copy_from_slice(&1_000_000u64.to_le_bytes());
        imu[8..12].copy_from_slice(&1355u32.to_le_bytes());
        imu[60] = 1;
        let p = page(&[
            record(20, 0, &fix(1000)),
            record(21, 1, &imu),
            record(22, 2, &0u32.to_le_bytes()),
        ]);
        let s = parse(std::io::Cursor::new(p), 512, "compatibility").unwrap();
        assert_eq!(s.samples.len(), 1);
        assert!(s.samples[0].mapped_ns.is_none());
        assert!(!s
            .raw_records
            .iter()
            .any(|r| r["kind"] == "capture_timing_summary"));
    }
    #[test]
    fn native_log_uses_fused_motion_and_preserves_the_configured_drag_target() {
        use crate::display::{Configure, DrivingFrame, Motion, Timing};
        let mut buf = [0; 208];
        let setup = Configure::Begin {
            start_centi_kph: 0,
            end_centi_kph: 6000,
            distance_mm: 100000,
            rollout: true,
            mounted: true,
            gates: 0,
        };
        let mut records = vec![
            record(27, 0, postcard::to_slice(&setup, &mut buf).unwrap()),
            record(20, 1, &fix(1000)),
        ];
        let short = page(&[
            records[0].clone(),
            records[1].clone(),
            record(22, 2, &[0; 4]),
        ]);
        let short = parse(std::io::Cursor::new(short), 512, "no motion yet").unwrap();
        assert!(short.samples.is_empty() && short.provenance["kind"] == "display_v2");
        let mut frame = DrivingFrame {
            uptime_us: 1_200_000,
            session: 1,
            recording: true,
            started_at_us: Some(1_000_000),
            stopped_at_us: None,
            battery_percent: None,
            gnss_count: 10,
            imu_count: 40,
            satellites: None,
            fix_valid: true,
            mounted: true,
            motion: Some(Motion {
                t_us: 1_190_000,
                speed_mps: 2.,
                ..Motion::default()
            }),
            timing: Timing::default(),
        };
        for i in 0..3 {
            records.push(record_at(
                26,
                2 + i,
                frame.uptime_us * 1000,
                postcard::to_slice(&frame, &mut buf).unwrap(),
            ));
            frame.uptime_us += 200_000;
            frame.motion = None;
            if i == 1 {
                frame.recording = false;
                frame.stopped_at_us = Some(frame.uptime_us);
                frame.timing.distance_m = 1.25;
            }
        }
        records.push(record(22, 5, &[0; 4]));
        let pages = page(&records);
        let s = parse(std::io::Cursor::new(pages), 512, "driving").unwrap();
        assert_eq!(s.provenance["estimator"], "DrivingEstimator");
        assert_eq!(s.drag.end_kph, 60.);
        assert!(s.drag.rollout);
        assert_eq!(s.samples.len(), 2);
        assert_eq!(s.samples[0].speed_mps, Some(2.));
        assert!(!s.samples[1].valid && s.samples[1].speed_mps.is_none());
        assert!((s.duration_s - 0.6).abs() < 1e-8);
        assert_eq!(s.analysis.distance_m, 1.25);
    }

    #[test]
    fn new_sd_schema_imports_receiver_epochs_and_closed_health() {
        let p = page(&[
            record(20, 0, &fix(1000)),
            record(20, 1, &fix(1100)),
            record(22, 2, &0u32.to_le_bytes()),
        ]);
        let s = parse(std::io::Cursor::new(p), 512, "test").unwrap();
        assert_eq!(s.samples.len(), 2);
        assert_eq!(s.samples[1].time_s, 0.1);
        assert!(s.samples[1].receiver_timed && s.samples[1].valid);
        assert_eq!(s.raw_records.last().unwrap()["closed"], true);
    }
    #[test]
    fn page_and_record_corruption_both_fail_closed() {
        let p = page(&[
            record(20, 0, &fix(1000)),
            record(22, 1, &0u32.to_le_bytes()),
        ]);
        let mut corrupt = p;
        corrupt[60] ^= 1;
        assert!(parse(std::io::Cursor::new(corrupt), 512, "test").is_err());
        let crc = crc32(&corrupt[..508]);
        corrupt[508..].copy_from_slice(&crc.to_le_bytes());
        assert!(parse(std::io::Cursor::new(corrupt), 512, "test").is_err());
        let no_end = page(&[record(20, 0, &fix(1000))]);
        assert!(parse(std::io::Cursor::new(no_end), 512, "test").is_err());
    }
    #[test]
    fn terminal_uart_losses_are_preserved_despite_zero_sd_drop_counter() {
        let data = json!({"gnss_transport_loss":{"port":"data","uart_errors":2,"dropped_reads":0,"dropped_bytes":0,"observation_dropped":7}});
        let command = json!({"gnss_transport_loss":{"port":"command","uart_errors":1,"dropped_reads":3,"dropped_bytes":512,"observation_dropped":0}});
        let bytes = page(&[
            record(1, 0, &serde_json::to_vec(&data).unwrap()),
            record(1, 1, &serde_json::to_vec(&command).unwrap()),
            record(22, 2, &0u32.to_le_bytes()),
        ]);
        let session = parse(std::io::Cursor::new(bytes), 512, "Terminal transport").unwrap();
        let summary = transport_loss_summary(&session.raw_records).unwrap();
        assert_eq!(summary["data"]["uart_errors"], 2);
        assert_eq!(summary["data"]["observation_dropped"], 7);
        assert!(session.analysis.warnings.iter().any(|warning| warning
            .contains("7 complete parsed SPP observations")
            && warning.contains("no byte count is inferred")));
        assert_eq!(summary["command"]["dropped_reads"], 3);
        assert_eq!(summary["command"]["dropped_bytes"], 512);
        assert!(command_transport_lost(&summary));
        assert!(session
            .raw_records
            .iter()
            .any(|r| r["kind"] == "health" && r["dropped"] == 0));
        assert!(session.raw_records.iter().any(|r| r["value"] == command));
        assert!(session
            .analysis
            .warnings
            .iter()
            .any(|w| w.contains("command-port")
                && w.contains("512 known dropped bytes")
                && w.contains("separate from SD")));
        let data_only=transport_loss_summary(&[json!({"kind":"legacy_metadata","value":data}),json!({"kind":"legacy_metadata","value":{"gnss_transport_loss":{"port":"command","uart_errors":0,"dropped_reads":0,"dropped_bytes":0}}})]).unwrap();
        assert!(
            !command_transport_lost(&data_only),
            "SPP/PPS port diagnostics must not be mislabelled raw-command loss"
        );
    }
    #[test]
    fn missing_transport_totals_are_unknown_and_repeated_cumulative_totals_are_not_summed() {
        let missing = transport_loss_summary(&[]).unwrap();
        assert!(missing["data"].is_null() && missing["command"].is_null());
        let mut warnings = vec![];
        add_transport_warnings(&missing, &mut warnings);
        assert!(warnings[0].contains("input loss is unknown"));
        let records=[3,5,4].map(|count|json!({"kind":"legacy_metadata","value":{"gnss_transport_loss":{"port":"command","uart_errors":0,"dropped_reads":count,"dropped_bytes":count*100}}}));
        let summary = transport_loss_summary(&records).unwrap();
        assert_eq!(summary["command"]["dropped_reads"], 5);
        assert_eq!(summary["command"]["dropped_bytes"], 500);
        assert_eq!(summary["command"]["counter_regressed"], true);
        assert!(summary["command"]["observation_dropped"].is_null());
        assert_eq!(summary["command"]["observation_counter_available"], false);
        let data=transport_loss_summary(&[json!({"kind":"legacy_metadata","value":{"gnss_transport_loss":{"port":"data","uart_errors":0,"dropped_reads":0,"dropped_bytes":0}}})]).unwrap();
        warnings.clear();
        add_transport_warnings(&data, &mut warnings);
        assert!(warnings
            .iter()
            .any(|warning| warning.contains("observation-channel loss was not recorded")));
    }
    #[test]
    fn malformed_terminal_loss_counter_is_not_treated_as_zero() {
        for loss in [
            json!({"port":"command","uart_errors":0,"dropped_reads":-1,"dropped_bytes":0}),
            json!({"port":"command","uart_errors":0,"dropped_reads":0}),
            json!({"port":"other","uart_errors":0,"dropped_reads":0,"dropped_bytes":0}),
            json!({"port":"data","uart_errors":0,"dropped_reads":0,"dropped_bytes":0,"observation_dropped":-1}),
        ] {
            let bytes = page(&[
                record(
                    1,
                    0,
                    &serde_json::to_vec(&json!({"gnss_transport_loss":loss})).unwrap(),
                ),
                record(22, 1, &0u32.to_le_bytes()),
            ]);
            assert!(parse(std::io::Cursor::new(bytes), 512, "Invalid terminal counter").is_err());
        }
    }
    #[test]
    fn standard_crc_and_ecef_conversion() {
        assert_eq!(crc32(b"123456789"), 0xcbf43926);
        let (lat, lon, alt) = geodetic([6_378_137.0, 0.0, 0.0]);
        assert!(lat.abs() < 1e-9 && lon.abs() < 1e-9 && alt.abs() < 1e-6);
    }
    #[test]
    fn corruption_is_rejected_before_records_are_used() {
        let mut p = [0; 512];
        p[..8].copy_from_slice(b"AEVLOG1\0");
        assert_ne!(crc32(&p[..508]), u32_at(&p, 508));
    }
}
