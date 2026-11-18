//! Maps recorded receiver data into Traj's receiver-only measurement path.
//! Raw IMU is preserved for future calibrated reprocessing, never replaced by
//! synthetic gravity, course-as-body-heading or zero timing uncertainty.
use crate::model::{Analysis, DragConfig, DragRun, Gate, Lap, Sample, Track};
use aevia_trajectory::{
    frame::{
        CoordinateEpoch, EcefPosition, EcefVelocity, ReferenceEllipsoid, TerrestrialFrame,
        TerrestrialRealization, Wgs84Realization,
    },
    ids::{FrameId, GateId, MetricDefinitionId, ReferencePointId, TargetId},
    metric::{
        CrossingDirection, DistancePlan, DistanceQuantity, DragPlan, DragTarget, FiniteGate,
        LapPlan, LaunchRule, MetricDefinition, MetricPlan, MetricResultValue, Rollout,
        SpeedQuantity, TargetDirection,
    },
    quality::{FieldValue, TimingQuality, Validity},
    receiver::{ReceiverFix, ReceiverTrack},
    time::{DurationNs, SessionTime},
};
use serde_json::Value;

const POINT: ReferencePointId = ReferencePointId::new(1);
const FRAME: FrameId = FrameId::new(1);
const MAX_GAP_NS: u64 = 1_500_000_000;
// Receiver-only admission policy, not an accuracy/confidence claim. A larger
// disagreement needs calibrated fusion; a Hermite curve must not turn GNSS
// position jitter into acceleration the receiver velocity never measured.
const MAX_ABSOLUTE_VELOCITY_CORRECTION_MPS: f64 = 0.5;
const MAX_RELATIVE_VELOCITY_CORRECTION: f64 = 0.02;
const MAX_DRAG_ATTEMPT_FIXES: usize = 10_000;

#[cfg(test)]
#[path = "trajectory_adapter_tests.rs"]
mod behavior_tests;

pub fn analyze(
    samples: &[Sample],
    raw_records: &[Value],
    track: Option<&Track>,
    drag: &DragConfig,
) -> Analysis {
    let mut out = Analysis {
        engine: "Traj · receiver interpolation (GNSS only)".into(),
        ..Analysis::default()
    };
    out.warnings.push("GNSS-only interpolation: IMU fusion and statistical timing uncertainty are unavailable. Displayed milliseconds are not an accuracy claim.".into());
    if !raw_records.is_empty() {
        out.warnings.push("Raw sensor records are retained. Calibrated mounting, receiver/IMU time alignment and interval support are required for fused reprocessing.".into());
    }
    if let Some(dropped) = raw_records
        .iter()
        .filter_map(|r| r.get("dropped").and_then(Value::as_u64))
        .max()
        .filter(|v| *v > 0)
    {
        out.warnings.push(format!(
            "Device reported {dropped} dropped sensor records; captured data is incomplete."
        ));
    }
    if samples
        .iter()
        .any(|s| s.velocity_latency_ms.is_some_and(|v| v > 0))
    {
        out.warnings.push("Receiver velocity latency is nonzero. Those fixes are excluded until position and velocity epochs can be aligned.".into());
    }
    for legacy in raw_records.iter().filter(|r| r["kind"] == "legacy_capture") {
        for (field, label) in [
            ("missing_sequences", "missing container records"),
            ("uart_missing_bytes", "missing UART bytes"),
            ("decoder_errors", "rejected GNSS frames"),
        ] {
            if let Some(n) = legacy[field].as_u64().filter(|v| *v > 0) {
                out.warnings.push(format!(
                    "Legacy capture contains {n} {label}; the original binary is retained."
                ));
            }
        }
        if legacy["closed"] == false {
            out.warnings.push("Legacy recording was not closed cleanly; only complete verified records were imported.".into());
        }
    }
    if drag.start_kph == 0.0 {
        out.warnings.push("Standing-start timing begins at sustained 0.5 m/s motion after at least 0.5 s stationary; it is not an extrapolated true-zero launch.".into());
    }
    let mut runs: Vec<Vec<(ReceiverFix, &Sample)>> = vec![];
    let mut run: Vec<(ReceiverFix, &Sample)> = vec![];
    let mut inconsistent_intervals = 0usize;
    for sample in samples {
        let fix = receiver_fix(sample);
        let disconnected = match (run.last(), fix) {
            (Some((last, _)), Some(next)) => {
                let dt = next.time.as_ns() - last.time.as_ns();
                let inconsistent =
                    dt > 0 && dt as u64 <= MAX_GAP_NS && !receiver_interval_consistent(last, &next);
                if inconsistent {
                    inconsistent_intervals += 1;
                }
                dt <= 0 || dt as u64 > MAX_GAP_NS || inconsistent
            }
            (Some(_), None) => true,
            _ => false,
        };
        if disconnected && !run.is_empty() {
            runs.push(std::mem::take(&mut run));
        }
        if let Some(fix) = fix {
            run.push((fix, sample));
            out.valid_samples += 1;
        }
    }
    if !run.is_empty() {
        runs.push(run);
    }
    if out.valid_samples < samples.len() {
        out.warnings.push(format!("{} samples lack a usable fix, position, altitude, course or speed; gaps are not interpolated.", samples.len() - out.valid_samples));
    }
    if runs.len() > 1 {
        out.warnings.push(format!(
            "{} separate receiver spans; timing never crosses a signal gap or clock reset.",
            runs.len()
        ));
    }
    if inconsistent_intervals > 0 {
        out.warnings.push(format!("{inconsistent_intervals} receiver intervals disagree between position and velocity. They are excluded from lap, drag and distance measurements; raw samples remain available. Receiver-only admission limits interpolation correction to 0.5 m/s or 2% of endpoint speed, whichever is larger."));
    }
    if inconsistent_intervals > 0 || out.valid_samples < samples.len() || runs.len() > 1 {
        out.warnings.push("Distance totals cover admitted receiver intervals only; excluded gaps are not counted as travelled distance.".into());
    }
    let valid_speeds: Vec<f64> = samples
        .iter()
        .filter(|s| receiver_fix(s).is_some())
        .filter_map(|s| s.speed_mps)
        .collect();
    out.max_speed_mps = valid_speeds.iter().copied().reduce(f64::max);
    let mut total_time = 0.0;
    for run in runs.iter().filter(|r| r.len() >= 2) {
        if crate::service::processing_cancelled() {
            out.warnings.push("Analysis cancelled.".into());
            return out;
        }
        total_time += (run.last().unwrap().0.time.as_ns() - run[0].0.time.as_ns()) as f64 * 1e-9;
        // Numerical work is bounded per distance chunk; adjacent chunks share
        // one endpoint and therefore neither lose nor duplicate an interval.
        for start in (0..run.len() - 1).step_by(128) {
            if crate::service::processing_cancelled() {
                out.warnings.push("Analysis cancelled.".into());
                return out;
            }
            let end = (start + 129).min(run.len());
            let fixes: Vec<_> = run[start..end].iter().map(|x| x.0).collect();
            let mut plan = MetricPlan::new(1);
            let definition = MetricDefinition::Distance(DistancePlan {
                definition: MetricDefinitionId::new(1),
                quantity: DistanceQuantity::HorizontalPath,
                reference_point: POINT,
                absolute_tolerance_m: 0.001,
                relative_tolerance: 1e-6,
            });
            if plan.push(definition).is_ok() {
                for result in measure(&fixes, &plan, &mut out.warnings) {
                    if let MetricResultValue::Distance(d) = result {
                        out.distance_m += d.metres;
                    }
                }
            }
        }
        if let Some(track) = track {
            analyze_laps(run, track, &mut out);
        }
        analyze_drag(run, drag, &mut out);
    }
    if total_time > 0.0 {
        out.average_speed_mps = Some(out.distance_m / total_time);
    }
    let mut durations: Vec<_> = out
        .laps
        .iter()
        .filter(|l| l.valid)
        .map(|l| l.duration_s)
        .collect();
    durations.sort_by(f64::total_cmp);
    out.best_lap_s = durations.first().copied();
    if !durations.is_empty() {
        let n = durations.len();
        out.average_lap_s = Some(durations.iter().sum::<f64>() / n as f64);
        out.median_lap_s = Some((durations[(n - 1) / 2] + durations[n / 2]) / 2.0);
        let sector_count = out
            .laps
            .iter()
            .filter(|l| l.valid)
            .map(|l| l.sectors_s.len())
            .max()
            .unwrap_or(0);
        if sector_count > 0 {
            let mut total = 0.0;
            for index in 0..sector_count {
                total += out
                    .laps
                    .iter()
                    .filter(|l| l.valid && l.sectors_s.len() == sector_count)
                    .filter_map(|l| l.sectors_s.get(index).copied())
                    .reduce(f64::min)
                    .unwrap_or(0.0);
            }
            out.theoretical_best_s = Some(total);
        }
    }
    out.warnings.sort();
    out.warnings.dedup();
    out
}

fn receiver_fix(s: &Sample) -> Option<ReceiverFix> {
    if s.velocity_latency_ms.is_some_and(|v| v > 0) {
        return None;
    }
    if !s.valid || !s.time_s.is_finite() || s.time_s.abs() > 8e9 {
        return None;
    }
    let (lat, lon, alt, speed, heading) =
        (s.lat?, s.lon?, s.altitude_m?, s.speed_mps?, s.heading_deg?);
    if ![lat, lon, alt, speed, heading]
        .iter()
        .all(|x| x.is_finite())
        || lat.abs() > 90.0
        || lon.abs() > 180.0
        || !(-10_000.0..=100_000.0).contains(&alt)
        || !(0.0..=200.0).contains(&speed)
        || s.hacc_m
            .is_some_and(|x| !x.is_finite() || !(0.0..=25.0).contains(&x))
    {
        return None;
    }
    let (east, north, up) = basis(lat, lon);
    let heading = heading.to_radians();
    let vertical = match s.vertical_mps {
        Some(v) if v.is_finite() && v.abs() <= 200.0 => v,
        Some(_) => return None,
        None => 0.0,
    };
    let velocity = std::array::from_fn::<_, 3, _>(|i| {
        speed * (east[i] * heading.sin() + north[i] * heading.cos()) + up[i] * vertical
    });
    let p = ecef(lat, lon, alt);
    Some(ReceiverFix {
        time: SessionTime::from_ns((s.time_s * 1e9).round() as i64),
        position: EcefPosition::new(p[0], p[1], p[2]).ok()?,
        velocity: EcefVelocity::new(velocity[0], velocity[1], velocity[2]).ok()?,
        timing: if s.receiver_timed {
            TimingQuality::Modeled
        } else {
            TimingQuality::ArrivalOnly
        },
    })
}

fn receiver_interval_consistent(first: &ReceiverFix, last: &ReceiverFix) -> bool {
    let dt = (last.time.as_ns() - first.time.as_ns()) as f64 * 1e-9;
    if dt <= 0.0 {
        return false;
    }
    let displacement = sub(last.position.components(), first.position.components());
    let initial = first.velocity.components();
    let final_velocity = last.velocity.components();
    let residual = std::array::from_fn(|axis| {
        displacement[axis] / dt - (initial[axis] + final_velocity[axis]) * 0.5
    });
    // v_Hermite(u) - [(1-u)v0 + uv1] = 6u(1-u) residual.
    // Its exact maximum norm is 1.5 ||residual|| for u in [0,1].
    let correction = 1.5 * norm(residual);
    let allowance = MAX_ABSOLUTE_VELOCITY_CORRECTION_MPS
        .max(MAX_RELATIVE_VELOCITY_CORRECTION * norm(initial).max(norm(final_velocity)));
    correction.is_finite() && correction <= allowance
}

/// Loose upper bound using the quadratic Bezier velocity controls. The
/// triangle/convex-hull inequalities include interior Hermite overshoot.
fn path_distance_upper_bound(run: &[(ReceiverFix, &Sample)]) -> f64 {
    let mut total = 0.0f64;
    for pair in run.windows(2) {
        let dt = (pair[1].0.time.as_ns() - pair[0].0.time.as_ns()) as f64 * 1e-9;
        let displacement = sub(
            pair[1].0.position.components(),
            pair[0].0.position.components(),
        );
        let initial = pair[0].0.velocity.components();
        let final_velocity = pair[1].0.velocity.components();
        // Integral <= dt/3 (|v0|_1 + |3 delta_p/dt - v0 - v1|_1 + |v1|_1).
        // Triangle bound avoids the middle control's division/cancellation:
        // integral <= |delta_p|_1 + 2dt/3 (|v0|_1 + |v1|_1).
        let l1 = |v: [f64; 3]| {
            v.into_iter()
                .fold(0.0f64, |sum, c| (sum + c.abs()).next_up())
        };
        let bound = (l1(displacement)
            + (2.0 * dt / 3.0).next_up() * (l1(initial) + l1(final_velocity)).next_up())
        .next_up();
        // One micrometre per interval conservatively loosens this skip bound
        // for Earth-scale coordinate subtraction and coefficient rounding.
        total = (total + bound + 1e-6).next_up();
    }
    total
}

fn receiver_supports_crossing(run: &[(ReceiverFix, &Sample)], time: f64, threshold: f64) -> bool {
    const ROOT_TIME_TOLERANCE: f64 = 4e-6;
    let index = run.partition_point(|(_, sample)| sample.time_s < time - ROOT_TIME_TOLERANCE);
    let from = index.saturating_sub(1);
    let to = (index + 2).min(run.len());
    run[from..to].windows(2).any(|pair| {
        let a = pair[0].1;
        let b = pair[1].1;
        let (Some(before), Some(after)) = (a.speed_mps, b.speed_mps) else {
            return false;
        };
        a.time_s - ROOT_TIME_TOLERANCE <= time
            && b.time_s + ROOT_TIME_TOLERANCE >= time
            && before <= threshold
            && after >= threshold
            && after > before
    })
}

fn frame() -> TerrestrialFrame {
    TerrestrialFrame::new(
        FRAME,
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).expect("constant epoch"),
        ReferenceEllipsoid::WGS84,
    )
}
fn measure(
    fixes: &[ReceiverFix],
    plan: &MetricPlan,
    warnings: &mut Vec<String>,
) -> Vec<MetricResultValue> {
    match ReceiverTrack::from_fixes(frame(), POINT, fixes, DurationNs::from_ns(MAX_GAP_NS)) {
        Ok(path) => match path.measure_with_limits(
            plan,
            aevia_trajectory::metric::MetricEvaluationLimits {
                absolute_root_tolerance_s: 1e-6,
                value_tolerance: 1e-6,
                maximum_root_evaluations: (fixes.len() as u32)
                    .saturating_mul(512)
                    .clamp(2048, 20_000_000),
                maximum_quadrature_evaluations: (fixes.len() as u32)
                    .saturating_mul(512)
                    .clamp(16384, 20_000_000),
                ..Default::default()
            },
        ) {
            Ok(results) => results
                .as_slice()
                .iter()
                .filter_map(|r| {
                    if let MetricResultValue::Unavailable(d) = r.value {
                        warnings.push(format!("Traj measurement unavailable: {:?}", d.reason));
                        None
                    } else {
                        Some(r.value)
                    }
                })
                .collect(),
            Err(e) => {
                warnings.push(format!("Traj measurement failed: {e:?}"));
                vec![]
            }
        },
        Err(e) => {
            warnings.push(format!("Receiver span rejected: {e:?}"));
            vec![]
        }
    }
}

fn gate(g: &Gate, id: u32, samples: &[(ReceiverFix, &Sample)]) -> Option<FiniteGate> {
    let altitude = samples
        .iter()
        .min_by(|a, b| {
            let d = |s: &Sample| {
                (s.lat.unwrap_or(g.lat) - g.lat).powi(2) + (s.lon.unwrap_or(g.lon) - g.lon).powi(2)
            };
            d(a.1).total_cmp(&d(b.1))
        })?
        .1
        .altitude_m?;
    let (east, north, _) = basis(g.lat, g.lon);
    let h = g.heading_deg.to_radians();
    let normal = std::array::from_fn(|i| east[i] * h.sin() + north[i] * h.cos());
    let width = std::array::from_fn(|i| east[i] * h.cos() - north[i] * h.sin());
    FiniteGate::new(
        GateId::new(id),
        FRAME,
        ecef(g.lat, g.lon, altitude),
        normal,
        width,
        g.width_m,
        100.0,
        CrossingDirection::NegativeToPositive,
        1.0,
        5.0,
        DurationNs::from_ns(2_000_000_000),
        None,
    )
    .ok()
}
fn analyze_laps(run: &[(ReceiverFix, &Sample)], track: &Track, out: &mut Analysis) {
    let mut lap = LapPlan::new(
        MetricDefinitionId::new(2),
        POINT,
        Some(SpeedQuantity::InstantaneousHorizontal),
    );
    if lap.set_maximum_occurrences_per_gate(1000).is_err() {
        return;
    }
    for (i, g) in std::iter::once(&track.start)
        .chain(&track.sectors)
        .enumerate()
    {
        let Some(gate) = gate(g, i as u32 + 1, run) else {
            out.warnings.push("Track gate is invalid.".into());
            return;
        };
        if lap.push_gate(gate).is_err() {
            out.warnings
                .push("Track exceeds Traj's 16-gate limit.".into());
            return;
        }
    }
    let mut plan = MetricPlan::new(2);
    if plan.push(MetricDefinition::Lap(lap)).is_err() {
        out.warnings.push("Invalid lap definition.".into());
        return;
    }
    let fixes: Vec<_> = run.iter().map(|x| x.0).collect();
    let results = measure(&fixes, &plan, &mut out.warnings);
    for result in &results {
        if let MetricResultValue::Lap(l) = result {
            let start_s = seconds(l.start);
            let end_s = seconds(l.end);
            let mut crossing_times: Vec<f64> = results
                .iter()
                .filter_map(|r| match r {
                    MetricResultValue::GateCrossing(g)
                        if g.gate != GateId::new(1) && g.time > l.start && g.time < l.end =>
                    {
                        Some(seconds(g.time))
                    }
                    _ => None,
                })
                .collect();
            crossing_times.sort_by(f64::total_cmp);
            crossing_times.push(end_s);
            let mut prior = start_s;
            let sectors_s = crossing_times
                .into_iter()
                .map(|t| {
                    let d = t - prior;
                    prior = t;
                    d
                })
                .collect();
            out.laps.push(Lap {
                number: out.laps.len() as u32 + 1,
                start_s,
                end_s,
                duration_s: l.elapsed_seconds,
                valid: l.validity != Validity::Invalid,
                sectors_s,
                max_speed_mps: run
                    .iter()
                    .filter(|x| x.1.time_s >= start_s && x.1.time_s <= end_s)
                    .filter_map(|x| x.1.speed_mps)
                    .reduce(f64::max),
            });
        }
    }
}

fn analyze_drag(run: &[(ReceiverFix, &Sample)], config: &DragConfig, out: &mut Analysis) {
    if ![config.start_kph, config.end_kph, config.distance_m]
        .iter()
        .all(|v| v.is_finite())
        || config.start_kph < 0.0
        || config.end_kph <= config.start_kph
        || config.distance_m <= 0.0
    {
        out.warnings.push("Invalid drag thresholds.".into());
        return;
    }
    let threshold = (config.start_kph / 3.6).max(0.5);
    let mut start = 0;
    let mut had_launch = false;
    for i in 1..run.len() {
        let previous = run[i - 1].1.speed_mps.unwrap_or(0.0);
        let current = run[i].1.speed_mps.unwrap_or(0.0);
        if previous < threshold && current >= threshold {
            // An arbitrarily long wait must not consume the bounded attempt
            // window. Keep evidence of the standing dwell, or the rolling
            // threshold's preceding observation, before measuring the launch.
            if config.start_kph == 0.0 {
                let earliest = run[i - 1].1.time_s - 0.5;
                start += run[start..i]
                    .partition_point(|(_, sample)| sample.time_s < earliest)
                    .saturating_sub(1);
            } else {
                start = i - 1;
            }
            had_launch = true;
        }
        if current < threshold && previous >= threshold {
            if had_launch {
                drag_attempt(&run[start..=i], config, out);
            }
            // Recording may begin during an already-running drive. Its first
            // slowdown still rearms a later fully observed launch.
            start = i;
            had_launch = false;
        }
    }
    if had_launch {
        drag_attempt(&run[start..], config, out);
    }
}
fn drag_attempt(run: &[(ReceiverFix, &Sample)], config: &DragConfig, out: &mut Analysis) {
    if run.len() < 2 {
        return;
    }
    if crate::service::processing_cancelled() {
        return;
    }
    let truncated = run.len() > MAX_DRAG_ATTEMPT_FIXES;
    let run = &run[..run.len().min(MAX_DRAG_ATTEMPT_FIXES)];
    if truncated {
        out.warnings.push("Drag analysis uses the first 10,000 fixes of each attempt. Targets completed within that window are retained; later targets remain unavailable.".into());
    }
    let speed_supported = run.windows(2).any(|pair| {
        let (Some(a), Some(b)) = (pair[0].1.speed_mps, pair[1].1.speed_mps) else {
            return false;
        };
        a < config.end_kph / 3.6 && b >= config.end_kph / 3.6
    });
    let distance_supported = path_distance_upper_bound(run) >= config.distance_m;
    if !speed_supported && !distance_supported {
        return;
    }
    if config.start_kph == 0.0 {
        let first_motion = run
            .iter()
            .position(|x| x.1.speed_mps.unwrap_or(0.0) >= 0.5)
            .unwrap_or(0);
        if first_motion == 0 || run[first_motion - 1].1.time_s - run[0].1.time_s < 0.5 {
            return;
        }
    }
    let launch = LaunchRule::SpeedThreshold {
        quantity: SpeedQuantity::InstantaneousHorizontal,
        threshold_mps: (config.start_kph / 3.6).max(0.5),
        dwell: DurationNs::from_ns(100_000_000),
    };
    let mut drag = DragPlan::new(MetricDefinitionId::new(3), POINT, launch);
    if config.rollout {
        drag.rollout = Rollout::ONE_FOOT_HORIZONTAL_PATH;
    }
    if speed_supported
        && drag
            .push_target(DragTarget::Speed {
                id: TargetId::new(1),
                quantity: SpeedQuantity::InstantaneousHorizontal,
                metres_per_second: config.end_kph / 3.6,
                direction: TargetDirection::Ascending,
            })
            .is_err()
    {
        return;
    }
    if distance_supported
        && drag
            .push_target(DragTarget::Distance {
                id: TargetId::new(2),
                quantity: DistanceQuantity::HorizontalPath,
                metres: config.distance_m,
            })
            .is_err()
    {
        return;
    }
    let mut plan = MetricPlan::new(3);
    if plan.push(MetricDefinition::Drag(drag)).is_err() {
        return;
    }
    let fixes: Vec<_> = run.iter().map(|x| x.0).collect();
    let mut result = DragRun {
        number: out.drag_runs.len() as u32 + 1,
        valid: true,
        ..DragRun::default()
    };
    let event_path =
        ReceiverTrack::from_fixes(frame(), POINT, &fixes, DurationNs::from_ns(MAX_GAP_NS)).ok();
    let mut reached = false;
    for value in measure(&fixes, &plan, &mut out.warnings) {
        if let MetricResultValue::DragTarget(d) = value {
            if !receiver_supports_crossing(
                run,
                seconds(d.launch_time),
                (config.start_kph / 3.6).max(0.5),
            ) {
                out.warnings.push("A curve-only launch lacked an ascending receiver-speed crossing and was excluded.".into());
                continue;
            }
            if d.target == TargetId::new(1)
                && !receiver_supports_crossing(run, seconds(d.event_time), config.end_kph / 3.6)
            {
                out.warnings.push("A curve-only speed target lacked an ascending receiver-speed crossing and was excluded.".into());
                continue;
            }
            reached = true;
            result.start_s = seconds(d.launch_time);
            result.end_s = result.end_s.max(seconds(d.event_time));
            result.valid &= d.validity != Validity::Invalid;
            let elapsed = if config.rollout {
                match d.rollout_adjusted_seconds {
                    FieldValue::Available(t) => Some(t),
                    _ => None,
                }
            } else {
                Some(d.elapsed_seconds)
            };
            if d.target == TargetId::new(1) {
                result.speed_time_s = elapsed;
            } else {
                result.distance_time_s = elapsed;
                result.trap_speed_mps = event_path
                    .as_ref()
                    .and_then(|path| path.horizontal_speed_at(d.event_time, POINT).ok());
            }
        }
    }
    if reached {
        if result.speed_time_s.is_none() || result.distance_time_s.is_none() {
            result.reason = Some("One target was not reached before the run ended.".into());
        }
        out.drag_runs.push(result);
    }
}
fn seconds(t: SessionTime) -> f64 {
    t.as_ns() as f64 * 1e-9
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}
fn basis(lat: f64, lon: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (s, c) = lat.to_radians().sin_cos();
    let (sl, cl) = lon.to_radians().sin_cos();
    ([-sl, cl, 0.0], [-s * cl, -s * sl, c], [c * cl, c * sl, s])
}
fn ecef(lat: f64, lon: f64, alt: f64) -> [f64; 3] {
    let (s, c) = lat.to_radians().sin_cos();
    let (sl, cl) = lon.to_radians().sin_cos();
    let e2 = 6.694_379_990_14e-3;
    let n = 6_378_137.0 / (1.0 - e2 * s * s).sqrt();
    [
        (n + alt) * c * cl,
        (n + alt) * c * sl,
        (n * (1.0 - e2) + alt) * s,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    fn samples() -> Vec<Sample> {
        (0..101)
            .map(|i| {
                let t = i as f64 / 10.0;
                Sample {
                    time_s: t,
                    lat: Some(0.0),
                    lon: Some((10.0 * t / 6_378_137.0).to_degrees()),
                    altitude_m: Some(0.0),
                    speed_mps: Some(10.0),
                    heading_deg: Some(90.0),
                    valid: true,
                    ..Sample::default()
                }
            })
            .collect()
    }
    #[test]
    fn distance_uses_traj_and_does_not_bridge_invalid_samples() {
        let mut data = samples();
        let a = analyze(&data, &[], None, &DragConfig::default());
        assert!(
            (a.distance_m - 100.0).abs() < 0.01,
            "{} {:?}",
            a.distance_m,
            a.warnings
        );
        data[50].valid = false;
        let b = analyze(&data, &[], None, &DragConfig::default());
        assert!((b.distance_m - 98.0).abs() < 0.01, "{}", b.distance_m);
        assert_eq!(b.valid_samples, 100);
    }
    #[test]
    fn no_fix_does_not_create_a_track_or_times() {
        let result = analyze(&[], &[], None, &DragConfig::default());
        assert_eq!(result.distance_m, 0.0);
        assert!(result.laps.is_empty());
        assert!(result.drag_runs.is_empty());
        assert_eq!(result.max_speed_mps, None);
    }
}
