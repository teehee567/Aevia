//! End-to-end checks of application conventions at Traj's receiver boundary.
//!
//! The fixtures have analytic geometry/timing, independent of the adapter's
//! metric results. They exercise the public recording sample shape rather than
//! constructing internal metric reports.

use super::*;

const EQUATOR_RADIUS_M: f64 = 6_378_137.0;
const EQUATOR_MERIDIAN_RADIUS_M: f64 = 6_335_439.327_292_82;

fn sample(time_s: f64, east: f64, north: f64, ve: f64, vn: f64) -> Sample {
    Sample {
        time_s,
        lat: Some((north / EQUATOR_MERIDIAN_RADIUS_M).to_degrees()),
        lon: Some((east / EQUATOR_RADIUS_M).to_degrees()),
        altitude_m: Some(0.0),
        speed_mps: Some(ve.hypot(vn)),
        heading_deg: Some(ve.atan2(vn).to_degrees().rem_euclid(360.0)),
        valid: true,
        receiver_timed: true,
        hacc_m: Some(0.5),
        ..Sample::default()
    }
}

fn circle(reverse: bool) -> Vec<Sample> {
    let angular_speed = std::f64::consts::TAU / 20.0;
    (0..=250)
        .map(|i| {
            let time = i as f64 / 5.0;
            let direction = if reverse { -1.0 } else { 1.0 };
            let theta = direction * (angular_speed * time - std::f64::consts::FRAC_PI_2);
            sample(
                time,
                50.0 * theta.cos(),
                50.0 * theta.sin(),
                -50.0 * angular_speed * direction * theta.sin(),
                50.0 * angular_speed * direction * theta.cos(),
            )
        })
        .collect()
}

fn circle_track() -> Track {
    let gate = |east: f64, north: f64, heading_deg| Gate {
        lat: (north / EQUATOR_MERIDIAN_RADIUS_M).to_degrees(),
        lon: (east / EQUATOR_RADIUS_M).to_degrees(),
        heading_deg,
        width_m: 12.0,
    };
    Track {
        id: "circle".into(),
        name: "Analytic 20-second circle".into(),
        start: gate(50.0, 0.0, 0.0),
        sectors: vec![gate(0.0, 50.0, 270.0), gate(-50.0, 0.0, 180.0)],
    }
}

fn acceleration() -> Vec<Sample> {
    (0..=100)
        .map(|i| {
            let time = i as f64 / 10.0;
            let moving = (time - 1.0).max(0.0);
            // Two m/s² from rest after one second stationary: x = t².
            sample(time, moving * moving, 0.0, 2.0 * moving, 0.0)
        })
        .collect()
}

fn drag_config() -> DragConfig {
    DragConfig {
        start_kph: 0.0,
        end_kph: 36.0,
        distance_m: 20.0,
        rollout: false,
    }
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual {actual:.9}, expected {expected:.9}, tolerance {tolerance}"
    );
}

#[test]
fn northbound_start_and_ordered_sectors_measure_complete_laps() {
    let result = analyze(&circle(false), &[], Some(&circle_track()), &drag_config());
    assert_eq!(result.laps.len(), 2, "{result:?}");
    for (index, lap) in result.laps.iter().enumerate() {
        assert!(lap.valid);
        assert_eq!(lap.number, index as u32 + 1);
        close(lap.start_s, 5.0 + index as f64 * 20.0, 0.002);
        close(lap.duration_s, 20.0, 0.002);
        assert_eq!(lap.sectors_s.len(), 3, "{lap:?}");
        for (actual, expected) in lap.sectors_s.iter().zip([5.0, 5.0, 10.0]) {
            close(*actual, expected, 0.002);
        }
    }
    close(result.best_lap_s.unwrap(), 20.0, 0.002);
    close(result.theoretical_best_s.unwrap(), 20.0, 0.002);
    close(result.average_lap_s.unwrap(), 20.0, 0.002);
    close(result.median_lap_s.unwrap(), 20.0, 0.002);
}

#[test]
fn reverse_direction_and_a_gate_outside_the_recorded_line_do_not_count_laps() {
    let result = analyze(&circle(true), &[], Some(&circle_track()), &drag_config());
    assert!(result.laps.is_empty(), "{result:?}");
    let mut track = circle_track();
    track.start.lon = (100.0 / EQUATOR_RADIUS_M).to_degrees();
    let missed = analyze(&circle(false), &[], Some(&track), &drag_config());
    assert!(missed.laps.is_empty(), "{missed:?}");
}

#[test]
fn an_invalid_fix_and_a_long_time_gap_each_discard_only_the_crossing_lap() {
    let source = circle(false);
    let mut invalid = source.clone();
    invalid[75].valid = false; // t = 15, inside the first complete lap.
    let mut gap = source;
    gap.retain(|s| !(14.0..=16.0).contains(&s.time_s));
    for samples in [invalid, gap] {
        let result = analyze(&samples, &[], Some(&circle_track()), &drag_config());
        assert_eq!(result.laps.len(), 1, "{result:?}");
        close(result.laps[0].start_s, 25.0, 0.002);
        close(result.laps[0].end_s, 45.0, 0.002);
        assert!(result
            .warnings
            .iter()
            .any(|w| w.contains("separate receiver spans")));
    }
}

#[test]
fn standing_acceleration_matches_independent_speed_and_distance_times() {
    let result = analyze(&acceleration(), &[], None, &drag_config());
    assert_eq!(result.drag_runs.len(), 1, "{result:?}");
    let run = &result.drag_runs[0];
    // Launch at 0.5 m/s: tau = .25 s, x = .0625 m. End speed at tau = 5.
    close(run.start_s, 1.25, 0.002);
    close(run.speed_time_s.unwrap(), 4.75, 0.002);
    close(
        run.distance_time_s.unwrap(),
        20.0625_f64.sqrt() - 0.25,
        0.002,
    );
    close(run.trap_speed_mps.unwrap(), 2.0 * 20.0625_f64.sqrt(), 0.003);
    assert!(run.valid);
    assert!(run.reason.is_none());
}

#[test]
fn completed_drag_targets_survive_a_long_cruise_but_later_targets_stay_unavailable() {
    let samples: Vec<_> = (0..=10_100)
        .map(|i| {
            let time = i as f64 / 10.0;
            let (east, speed) = if time <= 10.0 {
                let moving = (time - 1.0).max(0.0);
                (moving * moving, 2.0 * moving)
            } else if time <= 1000.0 {
                (81.0 + 18.0 * (time - 10.0), 18.0)
            } else {
                // A second acceleration is after the bounded analysis window.
                let moving = time - 1000.0;
                (
                    81.0 + 18.0 * (time - 10.0) + moving * moving,
                    18.0 + 2.0 * moving,
                )
            };
            sample(time, east, 0.0, speed, 0.0)
        })
        .collect();
    let run: Vec<_> = samples
        .iter()
        .map(|sample| (receiver_fix(sample).unwrap(), sample))
        .collect();
    let mut result = Analysis::default();
    drag_attempt(&run, &drag_config(), &mut result);
    assert_eq!(result.drag_runs.len(), 1, "{result:?}");
    close(result.drag_runs[0].speed_time_s.unwrap(), 4.75, 0.002);
    close(
        result.drag_runs[0].distance_time_s.unwrap(),
        20.0625_f64.sqrt() - 0.25,
        0.002,
    );
    assert!(result
        .warnings
        .iter()
        .any(|warning| warning.contains("10,000 fixes")));

    let mut late = Analysis::default();
    drag_attempt(
        &run,
        &DragConfig {
            end_kph: 90.0,
            ..drag_config()
        },
        &mut late,
    );
    assert_eq!(late.drag_runs.len(), 1, "{late:?}");
    assert!(late.drag_runs[0].speed_time_s.is_none());
    assert!(late.drag_runs[0].distance_time_s.is_some());
}

#[test]
fn a_long_stationary_wait_does_not_consume_the_drag_attempt_window() {
    let samples: Vec<_> = (0..=10_150)
        .map(|i| {
            let time = i as f64 / 10.0;
            let moving = (time - 1005.0).max(0.0);
            sample(time, moving * moving, 0.0, 2.0 * moving, 0.0)
        })
        .collect();
    let run: Vec<_> = samples
        .iter()
        .map(|sample| (receiver_fix(sample).unwrap(), sample))
        .collect();
    let mut result = Analysis::default();
    analyze_drag(&run, &drag_config(), &mut result);
    assert_eq!(result.drag_runs.len(), 1, "{result:?}");
    close(result.drag_runs[0].start_s, 1005.25, 0.002);
    close(result.drag_runs[0].speed_time_s.unwrap(), 4.75, 0.002);
}

#[test]
fn recording_during_a_drive_can_time_a_later_fully_observed_launch() {
    let samples: Vec<_> = (0..=170)
        .map(|i| {
            let time = i as f64 / 10.0;
            let (east, speed) = if time <= 6.0 {
                (12.0 * time - time * time, 12.0 - 2.0 * time)
            } else {
                let moving = (time - 7.0).max(0.0);
                (36.0 + moving * moving, 2.0 * moving)
            };
            sample(time, east, 0.0, speed, 0.0)
        })
        .collect();
    let result = analyze(&samples, &[], None, &drag_config());
    assert_eq!(result.drag_runs.len(), 1, "{result:?}");
    close(result.drag_runs[0].start_s, 7.25, 0.002);
    close(result.drag_runs[0].speed_time_s.unwrap(), 4.75, 0.002);
}

#[test]
fn one_foot_rollout_changes_the_timing_origin_not_the_target_position() {
    let samples = acceleration();
    let plain = analyze(&samples, &[], None, &drag_config());
    let mut config = drag_config();
    config.rollout = true;
    let adjusted = analyze(&samples, &[], None, &config);
    assert_eq!(adjusted.drag_runs.len(), 1, "{adjusted:?}");
    let original = &plain.drag_runs[0];
    let rollout = &adjusted.drag_runs[0];
    let excluded = (0.0625_f64 + 0.3048).sqrt() - 0.25;
    close(
        original.speed_time_s.unwrap() - rollout.speed_time_s.unwrap(),
        excluded,
        0.002,
    );
    close(
        original.distance_time_s.unwrap() - rollout.distance_time_s.unwrap(),
        excluded,
        0.002,
    );
    close(rollout.start_s, original.start_s, 1e-6);
    close(rollout.end_s, original.end_s, 1e-6);
    close(
        rollout.trap_speed_mps.unwrap(),
        original.trap_speed_mps.unwrap(),
        1e-6,
    );
}

#[test]
fn rolling_intervals_require_an_ascending_start_crossing() {
    let samples = acceleration();
    let mut config = drag_config();
    config.start_kph = 18.0; // 5 m/s to 10 m/s at 2 m/s².
    let result = analyze(&samples, &[], None, &config);
    assert_eq!(result.drag_runs.len(), 1, "{result:?}");
    close(result.drag_runs[0].speed_time_s.unwrap(), 2.5, 0.002);
    close(result.drag_runs[0].start_s, 3.5, 0.002);
    let already_moving: Vec<_> = samples.into_iter().filter(|s| s.time_s >= 4.0).collect();
    let result = analyze(&already_moving, &[], None, &config);
    assert!(
        result.drag_runs.is_empty(),
        "A start before recording must not be invented: {result:?}"
    );
}

#[test]
fn consecutive_acceleration_runs_are_separate_and_braking_is_not_a_launch() {
    let samples: Vec<_> = (0..=280)
        .map(|i| {
            let time = i as f64 / 10.0;
            let cycle = (time / 14.0).floor();
            let phase = time - cycle * 14.0;
            let (distance, speed) = if phase < 1.0 {
                (0.0, 0.0)
            } else if phase <= 7.0 {
                let tau = phase - 1.0;
                (tau * tau, 2.0 * tau)
            } else if phase <= 13.0 {
                let tau = phase - 7.0;
                (36.0 + 12.0 * tau - tau * tau, 12.0 - 2.0 * tau)
            } else {
                (72.0, 0.0)
            };
            sample(time, cycle * 72.0 + distance, 0.0, speed, 0.0)
        })
        .collect();
    let result = analyze(&samples, &[], None, &drag_config());
    assert_eq!(result.drag_runs.len(), 2, "{result:?}");
    for (index, run) in result.drag_runs.iter().enumerate() {
        assert_eq!(run.number, index as u32 + 1);
        close(run.start_s, 1.25 + index as f64 * 14.0, 0.002);
        close(run.speed_time_s.unwrap(), 4.75, 0.002);
        close(
            run.distance_time_s.unwrap(),
            20.0625_f64.sqrt() - 0.25,
            0.002,
        );
    }
}

#[test]
fn uncorrelated_or_invalid_receiver_evidence_is_not_silently_repaired() {
    let source = acceleration();
    let mut delayed = source.clone();
    for sample in &mut delayed {
        sample.velocity_latency_ms = Some(20);
    }
    let result = analyze(&delayed, &[], None, &drag_config());
    assert_eq!(result.valid_samples, 0);
    assert!(result.drag_runs.is_empty());
    assert!(result
        .warnings
        .iter()
        .any(|w| w.contains("velocity latency")));

    let mut missing_course = source;
    for sample in &mut missing_course {
        sample.heading_deg = None;
    }
    let result = analyze(&missing_course, &[], None, &drag_config());
    assert!(result.drag_runs.is_empty());
    assert_eq!(result.distance_m, 0.0);
}

#[test]
fn interrupted_acceleration_never_completes_a_target_across_missing_or_jumped_positions() {
    let mut invalid = acceleration();
    invalid[40].valid = false; // Neither target has been reached at t = 4 s.
    let mut jumped = acceleration();
    for sample in &mut jumped[40..] {
        sample.lon = sample
            .lon
            .map(|lon| lon + (1_000.0 / EQUATOR_RADIUS_M).to_degrees());
    }
    for samples in [invalid, jumped] {
        let result = analyze(&samples, &[], None, &drag_config());
        assert!(result.drag_runs.is_empty(), "{result:?}");
        assert!(
            result.distance_m < 81.0,
            "A position jump must not become travelled distance: {result:?}"
        );
        assert!(result
            .warnings
            .iter()
            .any(|w| w.contains("separate receiver spans")));
    }
}

#[test]
fn engine_drag_targets_work_at_sample_boundaries() {
    let fixes: Vec<_> = acceleration().iter().filter_map(receiver_fix).collect();
    for quantity in [
        SpeedQuantity::InstantaneousHorizontal,
        SpeedQuantity::Spatial3d,
    ] {
        for external in [false, true] {
            for distance in [false, true] {
                let launch = if external {
                    LaunchRule::ExternalTimestamp(SessionTime::from_ns(1_250_000_000))
                } else {
                    LaunchRule::SpeedThreshold {
                        quantity,
                        threshold_mps: 0.5,
                        dwell: DurationNs::from_ns(100_000_000),
                    }
                };
                let mut drag = DragPlan::new(MetricDefinitionId::new(3), POINT, launch);
                drag.push_target(if distance {
                    DragTarget::Distance {
                        id: TargetId::new(1),
                        quantity: DistanceQuantity::HorizontalPath,
                        metres: 20.0,
                    }
                } else {
                    DragTarget::Speed {
                        id: TargetId::new(1),
                        quantity,
                        metres_per_second: 10.0,
                        direction: TargetDirection::Ascending,
                    }
                })
                .unwrap();
                let mut plan = MetricPlan::new(3);
                plan.push(MetricDefinition::Drag(drag)).unwrap();
                let mut warnings = vec![];
                let results = measure(&fixes, &plan, &mut warnings);
                assert!(warnings.is_empty(), "{warnings:?}");
                assert_eq!(results.len(), 1);
                let MetricResultValue::DragTarget(report) = results[0] else {
                    panic!("Expected measured target")
                };
                close(
                    report.elapsed_seconds,
                    if distance {
                        20.0625_f64.sqrt() - 0.25
                    } else {
                        4.75
                    },
                    0.002,
                );
            }
        }
    }
}

#[test]
fn slowly_moving_noisy_positions_cannot_become_hundred_kph_drag_results() {
    let data: Vec<_> = (0..=500)
        .map(|i| {
            let t = i as f64 / 50.0;
            let (position, speed) = if t < 1.0 {
                (0.0, 0.0)
            } else {
                // The Doppler velocity only reaches 1 m/s. Opposite 1 m position
                // errors at 50 Hz create huge Hermite derivatives without fusion.
                (t - 1.0 + if i % 2 == 0 { 1.0 } else { -1.0 }, 1.0)
            };
            sample(t, position, 0.0, speed, 0.0)
        })
        .collect();
    let original = data.clone();
    let result = analyze(&data, &[], None, &DragConfig::default());
    assert!(result.drag_runs.is_empty(), "{result:?}");
    assert!(
        result.distance_m < 1e-3,
        "Jitter must not become travelled distance: {result:?}"
    );
    assert_eq!(result.max_speed_mps, Some(1.0));
    assert!(result
        .warnings
        .iter()
        .any(|w| w.contains("disagree between position and velocity")));
    assert!(data
        .iter()
        .zip(original)
        .all(|(a, b)| a.lon == b.lon && a.speed_mps == b.speed_mps && a.valid == b.valid));
}

#[test]
fn small_admitted_curve_overshoot_still_requires_receiver_threshold_support() {
    // Every interval remains within the 0.5 m/s correction policy. Neither
    // receiver endpoint reaches the configured 10 m/s target, although the
    // admitted midpoint derivative does. No speed target may be published.
    let mut data = acceleration();
    for point in &mut data {
        let moving = (point.time_s - 1.0).max(0.0);
        let speed = (2.0 * moving).min(9.9);
        let distance = if moving <= 4.95 {
            moving * moving
        } else {
            4.95 * 4.95 + (moving - 4.95) * 9.9
        };
        let perturbation = if point.time_s > 6.0 {
            0.01 * (std::f64::consts::PI * point.time_s * 10.0).cos()
        } else {
            0.0
        };
        *point = sample(point.time_s, distance + perturbation, 0.0, speed, 0.0);
    }
    let config = DragConfig {
        distance_m: 10_000.0,
        ..drag_config()
    };
    let fixes: Vec<_> = data.iter().filter_map(receiver_fix).collect();
    assert!(fixes
        .windows(2)
        .all(|p| receiver_interval_consistent(&p[0], &p[1])));
    let path =
        ReceiverTrack::from_fixes(frame(), POINT, &fixes, DurationNs::from_ns(MAX_GAP_NS)).unwrap();
    assert!(
        path.horizontal_speed_at(SessionTime::from_ns(6_550_000_000), POINT)
            .unwrap()
            > 10.0
    );
    let result = analyze(&data, &[], None, &config);
    assert!(result.drag_runs.is_empty(), "{result:?}");
    assert!(result.max_speed_mps.unwrap() < 10.0);
}

#[test]
fn distance_preflight_bound_covers_an_independent_accelerating_path() {
    let data = acceleration();
    let run: Vec<_> = data.iter().map(|s| (receiver_fix(s).unwrap(), s)).collect();
    // Analytic x(10)-x(0) = 81 m. The deliberately loose upper bound must
    // never discard its reachable 20 m target.
    assert!(path_distance_upper_bound(&run) >= 81.0);
    assert!(receiver_supports_crossing(&run, 6.0, 10.0));
    assert!(!receiver_supports_crossing(&run, 2.0, 10.0));
}
