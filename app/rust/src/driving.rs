//! Projection of board-owned driving results into the phone's session format.
use crate::display::{Configure, DrivingFrame, MapGate};
use crate::model::{Analysis, DragConfig, DragRun, Lap, Sample, Session, Track};
use serde_json::{json, Value};

pub fn decode(bytes: &[u8]) -> Result<Value, String> {
    if bytes.len() > crate::display::MAX_DRIVING_FRAME_BYTES {
        return Err("Oversized driving snapshot".into());
    }
    let (frame, rest) =
        postcard::take_from_bytes::<DrivingFrame>(bytes).map_err(|e| e.to_string())?;
    if !rest.is_empty() {
        return Err("Trailing driving snapshot bytes".into());
    }
    Ok(display_value(frame))
}

fn display_value(frame: DrivingFrame) -> Value {
    json!({"compact":true,"display_v2":true,"uptime_us":frame.uptime_us,"session":frame.session,
        "recording":frame.recording,"started_at_us":frame.started_at_us,"stopped_at_us":frame.stopped_at_us,
        "display_battery":frame.battery_percent,"gnss":{"count":frame.gnss_count},"imu":{"count":frame.imu_count},
        "mounted":frame.mounted,"satellites":frame.satellites,"gnss_fix_valid":frame.fix_valid,"motion":frame.motion,"timing_results":frame.timing})
}

/// Used only on a BLE connection whose settings write was acknowledged.
#[cfg(any(windows, target_os = "ios", test))]
pub fn decode_battery_display(
    bytes: &[u8],
    detail_version: u8,
) -> Result<(Value, Option<Value>), String> {
    use crate::display::{
        unpack_temperature, BatteryTelemetry, MotionSource, BATTERY_TELEMETRY_TAG,
        IMU_CLIPPING_BIT, MAX_BLE_FRAME_BYTES, MAX_DRIVING_FRAME_BYTES, THERMAL_TELEMETRY_TAG,
    };
    if bytes.len() > MAX_BLE_FRAME_BYTES {
        return Err("Oversized Bluetooth display packet".into());
    }
    let (source, clipping, bytes) = if detail_version >= 3 {
        let (&code, payload) = bytes.split_last().ok_or("Missing live motion source")?;
        let (source_code, clipping) = if detail_version >= 4 {
            (code & !IMU_CLIPPING_BIT, code & IMU_CLIPPING_BIT != 0)
        } else {
            (code, false)
        };
        let source = MotionSource::from_wire(source_code).ok_or("Invalid live motion source")?;
        (Some(source), clipping, payload)
    } else {
        (None, false, bytes)
    };
    let (frame, rest) =
        postcard::take_from_bytes::<DrivingFrame>(bytes).map_err(|e| e.to_string())?;
    if bytes.len() - rest.len() > MAX_DRIVING_FRAME_BYTES {
        return Err("Oversized driving snapshot".into());
    }
    let battery = if rest.is_empty() {
        None
    } else {
        let (data, thermal) = if let Some(data) = rest.strip_prefix(THERMAL_TELEMETRY_TAG) {
            (data, true)
        } else {
            (
                rest.strip_prefix(BATTERY_TELEMETRY_TAG)
                    .ok_or("Unknown Bluetooth display trailer")?,
                false,
            )
        };
        let (sample, tail) =
            postcard::take_from_bytes::<BatteryTelemetry>(data).map_err(|e| e.to_string())?;
        let mut value = serde_json::to_value(sample).map_err(|e| e.to_string())?;
        match (thermal, tail) {
            (true, [charger, imu, cpu]) => {
                value["temperatures_c"] = json!([*charger, *imu, *cpu].map(unpack_temperature));
            }
            (false, []) => {}
            _ => return Err("Invalid battery telemetry length".into()),
        }
        Some(value)
    };
    let mut value = display_value(frame);
    if let Some(source) = source {
        if (source == MotionSource::Waiting) != value["motion"].is_null() {
            return Err("Live motion source did not match the display frame".into());
        }
        value["motion_source"] = json!(source);
    }
    if detail_version >= 4 {
        value["imu_clipping"] = json!(clipping);
    }
    Ok((value, battery))
}

pub fn configuration(drag: &DragConfig, track: Option<&Track>, mounted: bool) -> Vec<Configure> {
    let gates: Vec<_> = track
        .into_iter()
        .flat_map(|t| std::iter::once(&t.start).chain(&t.sectors))
        .collect();
    let mut commands = vec![Configure::Begin {
        start_centi_kph: (drag.start_kph * 100.).round() as u32,
        end_centi_kph: (drag.end_kph * 100.).round() as u32,
        distance_mm: (drag.distance_m * 1000.).round() as u32,
        rollout: drag.rollout,
        mounted,
        gates: gates.len() as u8,
    }];
    for (index, g) in gates.into_iter().enumerate() {
        commands.push(Configure::Gate {
            index: index as u8,
            gate: MapGate {
                lat_e7: (g.lat * 1e7).round() as i32,
                lon_e7: (g.lon * 1e7).round() as i32,
                heading_cdeg: ((g.heading_deg.rem_euclid(360.) * 100.).round() as u32 % 36000)
                    as u16,
                width_cm: (g.width_m * 100.).round() as u16,
            },
        });
    }
    commands
}

pub fn capture_record(value: &Value) -> Option<Value> {
    let timing = value
        .get("timing_results")
        .or_else(|| value["navigation"].get("driving"))?;
    let motion = value
        .get("motion")
        .unwrap_or(&value["navigation"]["motion"]);
    let t = motion["t_us"]
        .as_u64()
        .or_else(|| value["uptime_us"].as_u64())?;
    let source = value
        .get("motion_source")
        .unwrap_or(&value["navigation"]["motion_source"]);
    Some(
        json!({"kind":"fused_display","t_us":t,"motion":motion,"timing":timing,
        "motion_source":source,"started_at_us":value["started_at_us"]}),
    )
}
pub fn sample(record: &Value) -> Option<Sample> {
    let t = record["t_us"]
        .as_u64()?
        .checked_sub(record["started_at_us"].as_u64()?)?;
    let m = &record["motion"];
    Some(Sample {
        time_s: t as f64 * 1e-6,
        lat: m["lat"].as_f64(),
        lon: m["lon"].as_f64(),
        altitude_m: m["alt_m"].as_f64(),
        speed_mps: m["speed_mps"].as_f64(),
        heading_deg: m["heading_deg"].as_f64(),
        course_deg: m["course_deg"].as_f64(),
        accel_long_g: m["vehicle_g"][0].as_f64(),
        accel_lat_g: m["vehicle_g"][1].as_f64(),
        valid: m.is_object()
            && !matches!(
                record["motion_source"].as_str(),
                Some("gnss" | "inertial" | "held")
            ),
        ..Default::default()
    })
}

/// Replay the metric reports retained with snapshots. A missing report never
/// becomes a crossing inferred from a 5 Hz display sample.
pub fn analysis(session: &Session) -> Analysis {
    let mut result = Analysis {
        engine: "Traj · on-device fused driving estimates".into(),
        valid_samples: session.samples.iter().filter(|s| s.valid).count(),
        ..Default::default()
    };
    if session.provenance["native_capture"] == true {
        let dropped = session
            .raw_records
            .iter()
            .filter(|r| r["kind"] == "health")
            .filter_map(|r| r["dropped"].as_u64())
            .max()
            .unwrap_or(0);
        if dropped > 0 {
            result.warnings.push(format!(
                "The SD recording queue dropped {dropped} entries (sensor records or raw GNSS chunks) from the start of the recording through this file part."
            ));
        }
    }
    let mut reported_laps = 0;
    let mut last_launch = None;
    let mut sector_times = Vec::new();
    let mut active_lap_start = None;
    for record in session.raw_records.iter().filter(|r| {
        matches!(r["kind"].as_str(), Some("fused_display" | "fused_summary"))
            && r["timing"].is_object()
    }) {
        let t = &record["timing"];
        let Some(origin) = record["started_at_us"].as_u64() else {
            continue;
        };
        let relative = |v: u64| v.saturating_sub(origin) as f64 * 1e-6;
        result.average_speed_mps = t["average_mps"].as_f64();
        result.distance_m = t["distance_m"].as_f64().unwrap_or_default();
        result.max_speed_mps = t["peak_mps"].as_f64();
        result.best_lap_s = t["best_lap_s"].as_f64();
        let number = t["laps"].as_u64().unwrap_or(0) as u32;
        reported_laps = reported_laps.max(number);
        let start = t["lap_start_us"].as_u64();
        let sector = t["sector"].as_u64().unwrap_or(0) as usize;
        let split = t["last_sector_s"].as_f64();
        // A split belongs to an observed sequence within a particular lap.
        // Missing snapshots can hide a gate or an entire lap.
        if start.is_some() && start == active_lap_start && sector == sector_times.len() + 1 {
            if let Some(split) = split {
                sector_times.push(split);
            }
        }
        if let (Some(end), Some(duration)) =
            (t["last_lap_end_us"].as_u64(), t["last_lap_s"].as_f64())
        {
            if number > 0 && result.laps.last().is_none_or(|lap| lap.number < number) {
                let end_s = relative(end);
                let mut sectors = Vec::new();
                if let Some(previous) = active_lap_start {
                    let same_lap = (relative(end) - relative(previous) - duration).abs() < 0.001;
                    if same_lap
                        && start == Some(end)
                        && sector == 0
                        && session
                            .track
                            .as_ref()
                            .is_some_and(|tr| tr.sectors.len() == sector_times.len())
                    {
                        if let Some(final_split) = split {
                            sectors.extend_from_slice(&sector_times);
                            sectors.push(final_split);
                        }
                    }
                }
                result.laps.push(Lap {
                    number,
                    start_s: end_s - duration,
                    end_s,
                    duration_s: duration,
                    valid: true,
                    sectors_s: sectors,
                    ..Default::default()
                });
            }
        }
        if active_lap_start != start {
            sector_times.clear();
            active_lap_start = start;
        }
        if let Some(launch) = t["launch_us"].as_u64() {
            let speed = t["speed_time_s"].as_f64();
            let distance = t["distance_time_s"].as_f64();
            if speed.is_some() || distance.is_some() {
                if last_launch != Some(launch) {
                    result.drag_runs.push(DragRun {
                        number: result.drag_runs.len() as u32 + 1,
                        start_s: relative(launch),
                        valid: true,
                        ..Default::default()
                    });
                    last_launch = Some(launch);
                }
                let run = result.drag_runs.last_mut().unwrap();
                run.speed_time_s = speed;
                run.distance_time_s = distance;
                run.end_s = t["timing_origin_us"]
                    .as_u64()
                    .map(relative)
                    .unwrap_or(run.start_s)
                    + speed.unwrap_or_default().max(distance.unwrap_or_default());
            }
        }
    }
    // A snapshot gap can hide a completed lap. The board's best remains
    // authoritative even when session-wide averages cannot be recovered.
    if reported_laps as usize == result.laps.len() {
        crate::trajectory_adapter::summarize_laps(&mut result);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::display::{Motion, Timing, MAX_DRIVING_FRAME_BYTES};

    #[test]
    fn full_snapshot_fits_gatt_and_usb_and_preserves_signed_acceleration() {
        let epoch = Some(u64::MAX);
        let elapsed = Some(999.0);
        let timing = Timing {
            distance_m: 2000.,
            average_mps: elapsed,
            peak_mps: elapsed,
            laps: u32::MAX,
            lap_start_us: epoch,
            last_lap_end_us: epoch,
            last_lap_s: elapsed,
            best_lap_s: elapsed,
            sector: 15,
            last_sector_s: elapsed,
            launch_us: epoch,
            timing_origin_us: epoch,
            speed_time_s: elapsed,
            distance_time_s: elapsed,
        };
        let frame = DrivingFrame {
            uptime_us: u64::MAX,
            session: u32::MAX,
            recording: true,
            started_at_us: epoch,
            stopped_at_us: epoch,
            battery_percent: Some(100),
            gnss_count: u32::MAX,
            imu_count: u32::MAX,
            satellites: Some(99),
            fix_valid: true,
            mounted: true,
            motion: Some(Motion {
                t_us: u64::MAX,
                speed_mps: 11.,
                course_deg: Some(15.),
                heading_deg: Some(14.),
                vehicle_g: Some([0.25, -0.5, 0.]),
                ..Motion::default()
            }),
            timing,
        };
        let mut buffer = [0; MAX_DRIVING_FRAME_BYTES];
        let encoded = postcard::to_slice(&frame, &mut buffer).unwrap();
        let value = decode(encoded).unwrap();
        let point = sample(&capture_record(&value).unwrap()).unwrap();
        assert_eq!(point.accel_long_g, Some(0.25));
        assert_eq!(point.accel_lat_g, Some(-0.5));
        assert_eq!(point.speed_mps, Some(11.));
        assert!(decode(&encoded[..encoded.len() - 1]).is_err());

        assert_eq!(
            decode_battery_display(encoded, 2).unwrap(),
            (value.clone(), None)
        );
        let battery = crate::display::BatteryTelemetry {
            status: crate::display::BatteryState::Charging,
            percent: Some(81),
            millivolts: Some(4012),
            rate_percent_per_hour: Some(3.12),
            charge_stage: Some(crate::display::ChargeStage::Taper),
            power_source: Some(crate::display::PowerSource::External),
            temperature_zone: Some(crate::display::TemperatureZone::Warm),
        };
        let mut packet = encoded.to_vec();
        packet.extend_from_slice(crate::display::BATTERY_TELEMETRY_TAG);
        assert!(decode_battery_display(&packet, 2).is_err());
        packet.extend_from_slice(postcard::to_slice(&battery, &mut [0; 20]).unwrap());
        let (motion, telemetry) = decode_battery_display(&packet, 2).unwrap();
        assert_eq!(motion, value);
        assert_eq!(telemetry.unwrap(), serde_json::to_value(battery).unwrap());
        assert!(decode(&packet).is_err());
        assert!(decode_battery_display(&packet[..packet.len() - 1], 2).is_err());
        packet.push(0);
        assert!(decode_battery_display(&packet, 2).is_err());
        packet.pop();
        packet[encoded.len()..encoded.len() + 4]
            .copy_from_slice(crate::display::THERMAL_TELEMETRY_TAG);
        packet.extend_from_slice(&[93, 51, 255]);
        let (_, thermal) = decode_battery_display(&packet, 2).unwrap();
        assert_eq!(thermal.unwrap()["temperatures_c"], json!([38, -4, null]));
        packet.push(crate::display::MotionSource::Gnss as u8);
        let (preview, thermal) = decode_battery_display(&packet, 3).unwrap();
        assert_eq!(preview["motion_source"], "gnss");
        assert_eq!(thermal.unwrap()["temperatures_c"], json!([38, -4, null]));
        let saved = sample(&capture_record(&preview).unwrap()).unwrap();
        assert_eq!(saved.speed_mps, Some(11.));
        assert!(!saved.valid);
        assert!(decode_battery_display(&packet, 2).is_err());
        *packet.last_mut().unwrap() |= crate::display::IMU_CLIPPING_BIT;
        let (clipped, _) = decode_battery_display(&packet, 4).unwrap();
        assert_eq!(clipped["motion_source"], "gnss");
        assert_eq!(clipped["imu_clipping"], true);
        assert!(decode_battery_display(&packet, 3).is_err());
        *packet.last_mut().unwrap() = 0xff;
        assert!(decode_battery_display(&packet, 3).is_err());
        packet.pop();
        assert!(decode_battery_display(&packet[..packet.len() - 1], 2).is_err());
        assert!(decode(&packet).is_err());
        packet.truncate(packet.len() - 3);
        packet[encoded.len()] ^= 0xff;
        assert!(decode_battery_display(&packet, 2).is_err());
        assert!(decode_battery_display(&[0; crate::display::MAX_BLE_FRAME_BYTES + 1], 2).is_err());
    }
}
