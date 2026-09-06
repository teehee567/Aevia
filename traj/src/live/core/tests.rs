//! Behavioral tests for the embedded delayed-filter orchestrator.

#[path = "smoothing_lifecycle_tests.rs"]
mod smoothing_lifecycle;

use core::mem::size_of;

use crate::{
    live::{
        eskf::{
            ConsiderCovariance, CovariancePolicy, Eskf, GapNavCrossCovariance, GnssObservation,
            NavConsiderCovariance, NisGate, ProcessNoise,
        },
        initializer::{InitialHeadingSource, InitializationResult},
        predictor::PredictorConfig,
        preintegration::{
            CompactCovariance3, GapModel, ImuInterval, ImuNoise, imu_sample_covariance,
        },
        reanchor::{EcefAnchor, ReanchorError, ReanchorTransform},
        scheduler::{EnqueueDisposition, OrderKey, Scheduled, WorkQuota},
        state::{MechanizationContext, NavMatrix, NavState},
    },
    quality::{GnssState, TimingQuality},
    time::SessionTime,
};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::*;

fn with_large_stack(test: fn()) {
    std::thread::Builder::new()
        .name("trajectory-live-core-test".into())
        .stack_size(16 * 1_024 * 1_024)
        .spawn(test)
        .expect("large-stack test thread must start")
        .join()
        .expect("large-stack test thread must finish");
}

fn config() -> LiveCoreConfig {
    LiveCoreConfig {
        fusion_delay_ns: 10_000_000,
        smoothing_lag_ns: 0,
        navigation_period_ns: 2_500_000,
        bias_correction_validity_norm: 1.0,
        mechanization: MechanizationContext::new(
            Vector3::new(0.0, 5.0e-5, 5.0e-5),
            Vector3::new(0.0, 0.0, -9.806_65),
            Matrix3::zeros(),
        )
        .unwrap(),
        imu_noise: ImuNoise {
            accel_covariance_density: Matrix3::identity() * 1.0e-6,
            gyro_covariance_density: Matrix3::identity() * 1.0e-8,
        },
        process_noise: ProcessNoise {
            accel_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-8,
            gyro_bias_random_walk_covariance_density: Matrix3::identity() * 1.0e-10,
        },
        covariance_policy: CovariancePolicy::conservative_candidate(),
        nis_gate: NisGate {
            soft_3d: 7.815,
            hard_3d: 25.0,
            soft_6d: 12.592,
            hard_6d: 35.0,
            maximum_covariance_inflation: 10.0,
        },
        predictor: PredictorConfig {
            position_time_constant_s: 0.1,
            velocity_time_constant_s: 0.1,
            attitude_time_constant_s: 0.1,
            position_reset_threshold_m: 20.0,
            velocity_reset_threshold_mps: 20.0,
            attitude_reset_threshold_rad: 1.0,
        },
        gap: GapModel {
            maximum_gap_ns: 10_000_000,
            angular_acceleration_one_sigma: Vector3::repeat(0.1),
            jerk_one_sigma: Vector3::repeat(1.0),
        },
    }
}

struct TestSeed {
    initialization: InitializationResult,
    nav_consider_covariance: NavConsiderCovariance,
    consider_covariance: ConsiderCovariance,
    active_consider: usize,
}

impl TestSeed {
    fn borrowed(&self) -> LiveCoreSeed<'_> {
        LiveCoreSeed {
            initialization: &self.initialization,
            nav_consider_covariance: &self.nav_consider_covariance,
            consider_covariance: &self.consider_covariance,
            active_consider: self.active_consider,
        }
    }
}

fn seed() -> TestSeed {
    TestSeed {
        initialization: InitializationResult {
            state: NavState::stationary(SessionTime::ZERO),
            covariance: NavMatrix::identity(),
            heading_source: InitialHeadingSource::Supplied,
            stationary_probability: 1.0,
        },
        nav_consider_covariance: NavConsiderCovariance::zeros(),
        consider_covariance: ConsiderCovariance::zeros(),
        active_consider: 0,
    }
}

fn imu(start_ns: i64, end_ns: i64) -> ImuInterval {
    ImuInterval {
        start: SessionTime::from_ns(start_ns),
        end: SessionTime::from_ns(end_ns),
        omega_ib_b: Vector3::new(0.0, 5.0e-5, 5.0e-5),
        specific_force_b: Vector3::new(0.0, 0.0, 9.806_65),
        degraded_input: false,
        gap_elapsed_ns_plus_one: 0,
        body_from_sensor: nalgebra::UnitQuaternion::identity(),
        accel_sample_covariance: crate::live::CompactCovariance3::ZERO,
        gyro_sample_covariance: crate::live::CompactCovariance3::ZERO,
        calibration_consider_start: None,
    }
}

fn gnss(time_ns: i64, sequence: u64, x: f32) -> Scheduled<GnssObservation> {
    let time = SessionTime::from_ns(time_ns);
    Scheduled {
        key: OrderKey {
            time,
            class: 3,
            source: 1,
            sequence,
        },
        value: GnssObservation {
            time,
            position_n: Some(Vector3::new(x, 0.0, 0.0)),
            velocity_n: None,
            position_covariance_n: Matrix3::identity(),
            velocity_covariance_n: Matrix3::identity(),
            position_velocity_cross_n: None,
            imu_to_antenna_b: Vector3::zeros(),
            omega_ib_b: Vector3::new(0.0, 5.0e-5, 5.0e-5),
            specific_force_b: Vector3::new(0.0, 0.0, 9.806_65),
            angular_acceleration_eb_b: None,
            angular_acceleration_covariance_b: Matrix3::zeros(),
            clock_consider_start: None,
            clock_reference_time: SessionTime::ZERO,
            lever_arm_consider_start: None,
            position_independent_timing_sigma_s: 0.0,
            velocity_independent_timing_sigma_s: 0.0,
            shared_jacobians: super::super::eskf::SharedMeasurementJacobians::default(),
            receiver_healthy: true,
            quality_state: GnssState::Healthy,
            quality_timing: TimingQuality::PpsCorrelated,
        },
    }
}

fn smoothed_run(credits: u16, with_gnss: bool) -> (std::vec::Vec<DenseSegment>, NavState) {
    let mut history = LiveCoreHistory::new();
    let mut tuning = config();
    tuning.smoothing_lag_ns = 100_000_000;
    let mut state = LiveCoreState::new(tuning, seed().borrowed(), &history).unwrap();
    let mut core = LiveCore::attach(&mut state, &mut history);
    if with_gnss {
        core.ingest_gnss(gnss(100_000_000, 1, 1.0)).unwrap();
        core.ingest_gnss(gnss(170_000_000, 2, 1.2)).unwrap();
    }
    for millisecond in 0..250 {
        core.ingest(LiveCoreInput::Imu(imu(
            millisecond * 1_000_000,
            (millisecond + 1) * 1_000_000,
        )))
        .unwrap();
    }
    let mut segments = std::vec::Vec::new();
    let mut calls = 0;
    loop {
        let report = core.drain(&mut WorkQuota::new(credits)).unwrap();
        while let Some(segment) = core.pop_corrected_segment() {
            segments.push(segment);
        }
        calls += 1;
        assert!(calls < 10_000, "smoothing did not make bounded progress");
        if report.blocked_on == DrainBlock::AwaitingDelayedFrontier {
            break;
        }
    }
    let status = core.status().unwrap();
    assert_eq!(
        status.corrected_frontier,
        Some(SessionTime::from_ns(240_000_000))
    );
    assert_eq!(
        status.published_frontier,
        Some(SessionTime::from_ns(140_000_000))
    );
    assert_eq!(
        core.present_state().unwrap().time,
        SessionTime::from_ns(250_000_000)
    );
    core.finish().unwrap();
    loop {
        let report = core.drain(&mut WorkQuota::new(credits)).unwrap();
        while let Some(segment) = core.pop_corrected_segment() {
            segments.push(segment);
        }
        calls += 1;
        assert!(calls < 10_000, "smoother tail did not finish");
        if report.blocked_on == DrainBlock::Finished {
            break;
        }
    }
    assert!(core.status().unwrap().drained);
    assert_eq!(
        core.status().unwrap().published_frontier,
        Some(SessionTime::from_ns(250_000_000))
    );
    (segments, *core.corrected_state())
}

#[test]
fn rts_later_gnss_corrects_history_and_preserves_final_endpoint_and_boundaries() {
    with_large_stack(|| {
        let (segments, filtered) = smoothed_run(u16::MAX, true);
        assert_eq!(segments.len(), 100);
        assert!(segments[0].start.state.position_n.x > 0.1);
        assert!(segments[0].start.covariance.position[(0, 0)] < 1.0);
        assert_eq!(segments.last().unwrap().end.state, filtered);
        for pair in segments.windows(2) {
            assert_eq!(pair[0].end, pair[1].start, "published boundary was revised");
        }
    });
}

#[test]
fn rts_quota_partition_does_not_change_published_trajectory() {
    with_large_stack(|| {
        let (whole, whole_end) = smoothed_run(u16::MAX, true);
        let (chunked, chunked_end) = smoothed_run(96, true);
        assert_eq!(whole_end, chunked_end);
        assert_eq!(whole, chunked);
    });
}

#[test]
fn rts_no_gnss_does_not_invent_a_correction() {
    with_large_stack(|| {
        let (segments, _) = smoothed_run(96, false);
        for segment in segments {
            assert!(segment.end.state.position_n.norm() < 1.0e-6);
            assert!(segment.end.state.velocity_n.norm() < 1.0e-5);
        }
    });
}

fn anchor(generation: u32, origin: Vector3<f64>, yaw: f64) -> EcefAnchor {
    let cosine = yaw.cos();
    let sine = yaw.sin();
    EcefAnchor::new(
        generation,
        origin,
        Matrix3::new(cosine, -sine, 0.0, sine, cosine, 0.0, 0.0, 0.0, 1.0),
    )
    .unwrap()
}

fn ecef_velocity(anchor: &EcefAnchor, velocity_n: Vector3<f32>) -> Vector3<f64> {
    anchor.vector_to_ecef(velocity_n)
}

fn covariance_ecef(anchor: &EcefAnchor, covariance_n: Matrix3<f32>) -> Matrix3<f64> {
    let ecef_from_n = anchor.ecef_to_n.transpose();
    ecef_from_n * covariance_n.cast::<f64>() * ecef_from_n.transpose()
}

fn ecef_orientation(
    anchor: &EcefAnchor,
    orientation_n_from_b: UnitQuaternion<f32>,
) -> Matrix3<f64> {
    anchor.ecef_to_n.transpose()
        * orientation_n_from_b
            .to_rotation_matrix()
            .into_inner()
            .cast::<f64>()
}

fn first_queued(core: &LiveCore<'_>) -> Scheduled<GnssObservation> {
    let mut result = None;
    core.scheduler
        .try_for_each_measurement(|entry| {
            if result.is_none() {
                result = Some(*entry);
            }
            Ok::<(), ()>(())
        })
        .unwrap();
    result.unwrap()
}

#[test]
fn construction_reports_static_memory_and_rejects_dirty_history() {
    with_large_stack(|| {
        let sizes = LiveCore::sizes();
        assert!(sizes.core_bytes > size_of::<Eskf>());
        assert_eq!(sizes.history_bytes, size_of::<LiveCoreHistory>());
        let mut history = LiveCoreHistory::new();
        history.imu.push_back(imu(0, 1)).unwrap();
        let mut state = LiveCoreState::placeholder();
        let seed = seed();
        assert_eq!(
            state.initialize(&config(), &seed.borrowed(), &history),
            Err(LiveCoreError::HistoryNotEmptyOrInvalidSeed)
        );
        assert!(!state.is_active());
        assert_eq!(history.raw_imu_len(), 1);
    });
}

#[test]
fn in_place_initialization_matches_constructor_and_failure_resets() {
    with_large_stack(|| {
        let history = LiveCoreHistory::new();
        let config = config();
        let seed = seed();
        let constructed = LiveCoreState::new(config, seed.borrowed(), &history).unwrap();
        let mut in_place = LiveCoreState::placeholder();
        in_place
            .initialize(&config, &seed.borrowed(), &history)
            .unwrap();
        assert_eq!(in_place, constructed);
        assert!(in_place.is_active());

        let mut invalid = config;
        invalid.navigation_period_ns = 0;
        assert_eq!(
            in_place.initialize(&invalid, &seed.borrowed(), &history),
            Err(LiveCoreError::InvalidConfiguration)
        );
        assert!(!in_place.is_active());
        assert_eq!(in_place, LiveCoreState::placeholder());
        assert!(history.is_empty());
    });
}

#[test]
fn delayed_frontier_and_present_predictor_are_distinct() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        for index in 0..5 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }
        assert_eq!(
            core.present_state().unwrap().time,
            SessionTime::from_ns(12_500_000)
        );
        let mut quota = WorkQuota::new(100);
        let report = core.drain(&mut quota).unwrap();
        assert_eq!(report.finalized_segments, 1);
        assert_eq!(core.corrected_state().time, SessionTime::from_ns(2_500_000));
        assert_eq!(
            core.status().unwrap().corrected_frontier,
            Some(SessionTime::from_ns(2_500_000))
        );
    });
}

#[test]
fn same_epoch_gnss_is_applied_before_segment_becomes_immutable() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        assert_eq!(
            core.ingest_gnss(gnss(2_500_000, 1, 1.0)).unwrap(),
            EnqueueDisposition::Queued
        );
        for index in 0..5 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }
        let mut quota = WorkQuota::new(200);
        let report = core.drain(&mut quota).unwrap();
        assert_eq!(report.gnss_updates, 1);
        let terminal = core
            .corrected_dense_state_at(SessionTime::from_ns(2_500_000))
            .unwrap();
        assert!(terminal.position_n.x > 0.0);
        assert_eq!(
            core.ingest_gnss(gnss(2_500_000, 2, 0.0)).unwrap(),
            EnqueueDisposition::TooLateForLive
        );
    });
}

#[test]
fn gnss_pair_duplicate_rejection_leaves_live_queue_unchanged() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        let duplicate = gnss(2_500_000, 1, 1.0);
        assert_eq!(
            core.ingest_gnss_pair([Some(duplicate), Some(duplicate)]),
            Err(LiveCoreError::MeasurementQueueRejected(
                EnqueueDisposition::Duplicate
            ))
        );
        assert_eq!(core.scheduler.queued_measurements(), 0);
        assert_eq!(
            core.ingest_gnss(gnss(2_500_000, 2, 2.0)).unwrap(),
            EnqueueDisposition::Queued
        );
        assert_eq!(core.scheduler.queued_measurements(), 1);
    });
}

#[test]
fn one_drain_preserves_all_accepted_gnss_quality_transitions_in_order() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        let first = gnss(2_500_000, 1, 0.0);
        let mut second = gnss(5_000_000, 2, 0.0);
        second.value.quality_timing = TimingQuality::Modeled;
        assert_eq!(core.ingest_gnss(first).unwrap(), EnqueueDisposition::Queued);
        assert_eq!(
            core.ingest_gnss(second).unwrap(),
            EnqueueDisposition::Queued
        );
        for index in 0..6 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }

        let mut quota = WorkQuota::new(1_000);
        let report = core.drain(&mut quota).unwrap();
        let mut updates = report.gnss_quality_updates();
        assert_eq!(
            updates.next(),
            Some(GnssQualityUpdate {
                epoch: SessionTime::from_ns(2_500_000),
                state: GnssState::Healthy,
                timing: TimingQuality::PpsCorrelated,
                downweighted: false,
            })
        );
        assert_eq!(
            updates.next(),
            Some(GnssQualityUpdate {
                epoch: SessionTime::from_ns(5_000_000),
                state: GnssState::Healthy,
                timing: TimingQuality::Modeled,
                downweighted: false,
            })
        );
        assert_eq!(updates.next(), None);
    });
}

#[test]
fn queued_previous_segment_measurement_blocks_clock_relabel_transactionally() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut seeded = seed();
        seeded.active_consider = 2;
        seeded.consider_covariance[(0, 0)] = 1.0;
        seeded.consider_covariance[(1, 1)] = 1.0;
        let mut state = LiveCoreState::new(config(), seeded.borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        core.ingest_gnss(gnss(2_500_000, 1, 1.0)).unwrap();
        let before = core.filter;
        let mut identity = [[0.0; super::super::MAX_CONSIDER]; 2];
        identity[0][0] = 1.0;
        identity[1][1] = 1.0;

        assert_eq!(
            core.transition_clock_consider(2, &identity, [0.0; 3]),
            Err(LiveCoreError::ClockTransitionRequiresReinitialization)
        );
        assert_eq!(core.filter, before);
        assert_eq!(core.scheduler.queued_measurements(), 1);
    });
}

#[test]
fn short_gap_is_materialized_and_marks_corrected_output_degraded() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        core.ingest_imu(imu(0, 2_500_000)).unwrap();
        let disposition = core.ingest_imu(imu(5_000_000, 7_500_000)).unwrap();
        assert_eq!(
            disposition,
            IngestDisposition::ImuAccepted {
                stored_intervals: 2,
                predictor_segments: 2,
                gap_bridged: true,
            }
        );
        assert!(core.status().unwrap().predictor_gap);
        core.finish().unwrap();
        let mut quota = WorkQuota::new(1_000);
        core.drain(&mut quota).unwrap();
        assert!(
            core.corrected_dense_state_at(SessionTime::from_ns(4_000_000))
                .unwrap()
                .degraded
        );
        assert!(!core.status().unwrap().predictor_gap);
    });
}

#[test]
fn finish_flushes_partial_predictor_and_drains_exactly_to_last_imu() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        core.ingest_imu(imu(0, 1_000_000)).unwrap();
        let finish = core.finish().unwrap();
        assert!(finish.predictor_segment_flushed);
        assert_eq!(finish.terminal_time, SessionTime::from_ns(1_000_000));
        let mut quota = WorkQuota::new(1_000);
        let report = core.drain(&mut quota).unwrap();
        assert_eq!(report.blocked_on, DrainBlock::Finished);
        assert_eq!(core.corrected_state().time, SessionTime::from_ns(1_000_000));
        assert!(core.status().unwrap().drained);
    });
}

#[test]
fn an_insufficient_quota_does_not_consume_raw_evidence() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        for index in 0..5 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }
        let before = core.status().unwrap().retained_imu_intervals;
        let mut quota = WorkQuota::new(FILTER_PROPAGATION_CREDITS);
        let report = core.drain(&mut quota).unwrap();
        assert_eq!(report.blocked_on, DrainBlock::QuotaExhausted);
        assert_eq!(quota.remaining(), FILTER_PROPAGATION_CREDITS - 1);
        assert_eq!(core.status().unwrap().retained_imu_intervals, before);
        assert_eq!(core.corrected_state().time, SessionTime::ZERO);
    });
}

#[test]
fn partial_propagation_planning_is_charged_without_committing_evidence() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        for index in 0..10 {
            let start = index * 1_250_000;
            core.ingest_imu(imu(start, start + 1_250_000)).unwrap();
        }

        let retained_before = core.status().unwrap().retained_imu_intervals;
        // One credit commits the initial frontier. Of the remaining nine,
        // one is spent planning the first of the two required IMU slices;
        // eight stay reserved for a propagation that cannot yet run.
        let mut partial = WorkQuota::new(
            FRONTIER_COMMIT_CREDITS + IMU_SLICE_CREDITS + FILTER_PROPAGATION_CREDITS,
        );
        let report = core.drain(&mut partial).unwrap();
        assert_eq!(report.blocked_on, DrainBlock::QuotaExhausted);
        assert_eq!(report.frontier_commits, 1);
        assert_eq!(report.imu_slices, 0);
        assert_eq!(report.filter_propagations, 0);
        assert_eq!(partial.remaining(), FILTER_PROPAGATION_CREDITS);
        assert_eq!(
            core.status().unwrap().retained_imu_intervals,
            retained_before
        );
        assert_eq!(core.corrected_state().time, SessionTime::ZERO);

        let mut complete = WorkQuota::new(2 * IMU_SLICE_CREDITS + FILTER_PROPAGATION_CREDITS);
        let report = core.drain(&mut complete).unwrap();
        assert_eq!(report.blocked_on, DrainBlock::QuotaExhausted);
        assert_eq!(report.imu_slices, 2);
        assert_eq!(report.filter_propagations, 1);
        assert_eq!(complete.remaining(), 0);
        assert_eq!(core.corrected_state().time, SessionTime::from_ns(2_500_000));
        assert_eq!(
            core.status().unwrap().retained_imu_intervals,
            retained_before - 2
        );
    });
}

#[test]
fn reanchor_is_atomic_and_preserves_all_retained_physical_state() {
    with_large_stack(|| {
        let old = anchor(1, Vector3::new(6_000_000.0, 1_000.0, 2_000.0), -0.15);
        let new = anchor(2, Vector3::new(6_000_120.0, 970.0, 2_008.0), 0.42);
        let old_again = EcefAnchor::new(3, old.origin_ecef_m, old.ecef_to_n).unwrap();
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);

        let mut delayed = gnss(15_000_000, 77, 18.0);
        delayed.value.velocity_n = Some(Vector3::new(2.0, -0.4, 0.1));
        delayed.value.position_velocity_cross_n = Some(Matrix3::identity() * 0.1);
        delayed.value.shared_jacobians.position[(0, 0)] = 0.75;
        delayed.value.shared_jacobians.velocity[(1, 1)] = -0.25;
        assert_eq!(
            core.ingest_gnss(delayed).unwrap(),
            EnqueueDisposition::Queued
        );
        for index in 0..8 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }
        // Retain one finalized corrected segment and one pending segment;
        // the 15 ms GNSS fix remains delayed beyond the 10 ms target.
        let mut quota = WorkQuota::new(20);
        core.drain(&mut quota).unwrap();
        assert_eq!(core.history.corrected.len(), 1);
        assert!(core.pending_corrected_segment.is_some());
        assert_eq!(core.scheduler.queued_measurements(), 1);

        let filter_before = core.filter;
        let present_before = core.present_state().unwrap();
        let corrected_dense_before = core
            .corrected_dense_state_at(SessionTime::from_ns(1_000_000))
            .unwrap();
        let predictor_dense_before = core
            .predictor_dense_state_at(SessionTime::from_ns(9_000_000))
            .unwrap();
        let queued_before = first_queued(&core);
        let raw_before = *core.history.imu.front().unwrap();
        let pending_start_before = core.pending_corrected_segment.unwrap().start;
        let gravity_probe_ecef = old.position_to_ecef(Vector3::new(20.0, -5.0, 2.0));
        let gravity_before = old.vector_to_ecef(
            core.context
                .gravity_at(&old.position_from_ecef(gravity_probe_ecef)),
        );

        core.reanchor(&old, &new).unwrap();

        let filter_after = core.filter;
        assert!(
            (new.position_to_ecef(filter_after.state.position_n)
                - old.position_to_ecef(filter_before.state.position_n))
            .norm()
                < 2.0e-4
        );
        assert!(
            (ecef_velocity(&new, filter_after.state.velocity_n)
                - ecef_velocity(&old, filter_before.state.velocity_n))
            .norm()
                < 2.0e-5
        );
        assert!(
            (ecef_orientation(&new, filter_after.state.orientation_n_from_b)
                - ecef_orientation(&old, filter_before.state.orientation_n_from_b))
            .norm()
                < 2.0e-5
        );
        let present_after = core.present_state().unwrap();
        assert!(
            (new.position_to_ecef(present_after.position_n)
                - old.position_to_ecef(present_before.position_n))
            .norm()
                < 2.0e-4
        );

        let corrected_dense_after = core
            .corrected_dense_state_at(SessionTime::from_ns(1_000_000))
            .unwrap();
        assert!(
            (new.position_to_ecef(corrected_dense_after.position_n)
                - old.position_to_ecef(corrected_dense_before.position_n))
            .norm()
                < 2.0e-4
        );
        assert!(
            (new.vector_to_ecef(corrected_dense_after.velocity_n)
                - old.vector_to_ecef(corrected_dense_before.velocity_n))
            .norm()
                < 2.0e-5
        );
        assert!(
            (new.vector_to_ecef(corrected_dense_after.acceleration_n)
                - old.vector_to_ecef(corrected_dense_before.acceleration_n))
            .norm()
                < 2.0e-4
        );
        assert!(
            (ecef_orientation(&new, corrected_dense_after.orientation_n_from_b)
                - ecef_orientation(&old, corrected_dense_before.orientation_n_from_b))
            .norm()
                < 2.0e-5
        );
        assert_eq!(
            corrected_dense_after.specific_force_b,
            corrected_dense_before.specific_force_b
        );
        let predictor_dense_after = core
            .predictor_dense_state_at(SessionTime::from_ns(9_000_000))
            .unwrap();
        assert!(
            (new.position_to_ecef(predictor_dense_after.position_n)
                - old.position_to_ecef(predictor_dense_before.position_n))
            .norm()
                < 2.0e-4
        );
        let pending_start_after = core.pending_corrected_segment.unwrap().start;
        assert!(
            (new.position_to_ecef(pending_start_after.state.position_n)
                - old.position_to_ecef(pending_start_before.state.position_n))
            .norm()
                < 2.0e-4
        );

        let queued_after = first_queued(&core);
        assert_eq!(queued_after.key, queued_before.key);
        assert!(
            (new.position_to_ecef(queued_after.value.position_n.unwrap())
                - old.position_to_ecef(queued_before.value.position_n.unwrap()))
            .norm()
                < 2.0e-4
        );
        assert!(
            (new.vector_to_ecef(queued_after.value.velocity_n.unwrap())
                - old.vector_to_ecef(queued_before.value.velocity_n.unwrap()))
            .norm()
                < 2.0e-5
        );
        assert!(
            (covariance_ecef(&new, queued_after.value.position_covariance_n)
                - covariance_ecef(&old, queued_before.value.position_covariance_n))
            .norm()
                < 2.0e-5
        );
        assert!(
            (covariance_ecef(&new, queued_after.value.velocity_covariance_n)
                - covariance_ecef(&old, queued_before.value.velocity_covariance_n))
            .norm()
                < 2.0e-5
        );
        let old_ecef_position_jacobian =
            old.ecef_to_n.transpose() * queued_before.value.shared_jacobians.position.cast::<f64>();
        let new_ecef_position_jacobian =
            new.ecef_to_n.transpose() * queued_after.value.shared_jacobians.position.cast::<f64>();
        assert!((new_ecef_position_jacobian - old_ecef_position_jacobian).norm() < 2.0e-5);
        assert_eq!(*core.history.imu.front().unwrap(), raw_before);
        let gravity_after = new.vector_to_ecef(
            core.context
                .gravity_at(&new.position_from_ecef(gravity_probe_ecef)),
        );
        assert!((gravity_after - gravity_before).norm() < 2.0e-5);

        core.reanchor(&new, &old_again).unwrap();
        assert!((core.filter.state.position_n - filter_before.state.position_n).norm() < 2.0e-4);
        assert!((core.filter.state.velocity_n - filter_before.state.velocity_n).norm() < 2.0e-5);
        assert!((core.filter.covariance - filter_before.covariance).norm() < 2.0e-4);
        let queued_round_trip = first_queued(&core);
        assert!(
            (queued_round_trip.value.position_n.unwrap() - queued_before.value.position_n.unwrap())
                .norm()
                < 2.0e-4
        );

        let state_before_failed_change = core.filter.state;
        let queued_before_failed_change = first_queued(&core);
        assert_eq!(
            core.reanchor(&old_again, &old),
            Err(LiveCoreError::Reanchor(
                ReanchorError::GenerationNotIncreasing
            ))
        );
        assert_eq!(core.filter.state, state_before_failed_change);
        assert_eq!(first_queued(&core), queued_before_failed_change);
        assert_eq!(*core.history.imu.front().unwrap(), raw_before);
    });
}

#[test]
fn gnss_fusion_uses_measurement_epoch_imu_not_packet_arrival_values() {
    fn run(
        arrival_omega: Vector3<f32>,
        arrival_force: Vector3<f32>,
    ) -> (NavState, NavMatrix, DrainReport) {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        let mut observation = gnss(2_500_000, 1, 0.1);
        observation.value.velocity_n = Some(Vector3::zeros());
        observation.value.imu_to_antenna_b = Vector3::new(1.0, 0.2, -0.1);
        observation.value.omega_ib_b = arrival_omega;
        observation.value.specific_force_b = arrival_force;
        observation.value.angular_acceleration_eb_b = Some(Vector3::zeros());
        observation.value.angular_acceleration_covariance_b = Matrix3::zeros();
        observation.value.position_independent_timing_sigma_s = 0.01;
        observation.value.velocity_independent_timing_sigma_s = 0.01;
        core.ingest_gnss(observation).unwrap();
        for index in 0..5 {
            let start = index * 2_500_000;
            core.ingest_imu(imu(start, start + 2_500_000)).unwrap();
        }
        let mut quota = WorkQuota::new(200);
        let report = core.drain(&mut quota).unwrap();
        (core.filter.state, core.filter.covariance, report)
    }

    with_large_stack(|| {
        let normal = run(Vector3::zeros(), Vector3::zeros());
        let delayed_packet = run(
            Vector3::new(40.0, -25.0, 12.0),
            Vector3::new(500.0, -800.0, 200.0),
        );
        assert_eq!(normal, delayed_packet);
        assert_eq!(normal.2.gnss_updates, 1);
    });
}

#[test]
fn held_sample_covariance_is_invariant_to_navigation_cadence() {
    fn run(period_ns: i64) -> (NavMatrix, GapNavCrossCovariance) {
        let mut configuration = config();
        configuration.navigation_period_ns = period_ns;
        configuration.mechanization =
            MechanizationContext::new(Vector3::zeros(), Vector3::zeros(), Matrix3::zeros())
                .unwrap();
        configuration.imu_noise.accel_covariance_density.fill(0.0);
        configuration.imu_noise.gyro_covariance_density.fill(0.0);
        configuration
            .process_noise
            .accel_bias_random_walk_covariance_density
            .fill(0.0);
        configuration
            .process_noise
            .gyro_bias_random_walk_covariance_density
            .fill(0.0);
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(configuration, seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        let mut sample = imu(0, 10_000_000);
        sample.omega_ib_b.fill(0.0);
        sample.specific_force_b.fill(0.0);
        sample.accel_sample_covariance =
            CompactCovariance3::from_matrix(Matrix3::identity() * 9.0).unwrap();
        sample.gyro_sample_covariance =
            CompactCovariance3::from_matrix(Matrix3::identity() * 4.0).unwrap();
        core.ingest_imu(sample).unwrap();
        core.finish().unwrap();
        core.drain(&mut WorkQuota::new(1_000)).unwrap();
        (
            core.filter.covariance,
            core.history.active_imu_sample_nav_cross,
        )
    }
    with_large_stack(|| {
        let unsplit = run(10_000_000);
        let split = run(2_500_000);
        assert!((unsplit.0 - split.0).norm() < 1.0e-6);
        assert!((unsplit.1 - split.1).norm() < 1.0e-7);
        assert!((split.1[(super::super::state::VEL, 0)] + 0.09).abs() < 1.0e-7);
        assert!((split.1[(super::super::state::POS, 0)] + 0.00045).abs() < 1.0e-9);
    });
}

#[test]
fn gnss_sample_correlation_uses_first_and_right_owned_supports() {
    with_large_stack(|| {
        for epoch in [0, 4_999_999, 5_000_000, 5_000_001] {
            let mut history = LiveCoreHistory::new();
            let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
            let mut core = LiveCore::attach(&mut state, &mut history);
            let mut first = imu(0, 5_000_000);
            first.omega_ib_b = Vector3::new(0.1, 0.2, 0.3);
            first.gyro_sample_covariance =
                CompactCovariance3::from_matrix(Matrix3::identity() * 0.2).unwrap();
            let mut second = imu(5_000_000, 10_000_000);
            second.omega_ib_b = Vector3::new(-0.2, 0.4, 0.5);
            second.gyro_sample_covariance =
                CompactCovariance3::from_matrix(Matrix3::identity() * 0.4).unwrap();
            core.ingest_imu(first).unwrap();
            core.ingest_imu(second).unwrap();
            let mut scheduled = gnss(epoch, 1, 0.0);
            scheduled.value.position_n = None;
            scheduled.value.velocity_n = Some(Vector3::zeros());
            scheduled.value.imu_to_antenna_b = Vector3::new(0.8, -0.2, 0.1);
            core.ingest_gnss(scheduled).unwrap();
            core.finish().unwrap();
            let mut report = DrainReport::new();
            if epoch != 0 {
                core.propagate_to(
                    SessionTime::from_ns(epoch),
                    &mut WorkQuota::new(1_000),
                    &mut report,
                )
                .unwrap();
            }
            let owner = if epoch < 5_000_000 { first } else { second };
            let kinematics = core
                .corrected_kinematics_at(SessionTime::from_ns(epoch))
                .unwrap();
            assert_eq!(kinematics.sample.omega_ib_b, owner.omega_ib_b);
            let mut expected_filter = core.filter;
            let mut expected_cross = if epoch == 0 || epoch == 5_000_000 {
                GapNavCrossCovariance::zeros()
            } else {
                core.history.active_imu_sample_nav_cross
            };
            let mut expected_observation = scheduled.value;
            expected_observation.omega_ib_b = owner.omega_ib_b;
            expected_observation.specific_force_b = owner.specific_force_b;
            expected_observation.angular_acceleration_eb_b = kinematics.angular_acceleration_eb_b;
            expected_observation.angular_acceleration_covariance_b =
                kinematics.angular_acceleration_covariance_b;
            expected_filter
                .update_gnss_with_imu_sample(
                    &expected_observation,
                    &core.context,
                    core.nis_gate,
                    Some(&imu_sample_covariance(
                        owner.accel_sample_covariance,
                        owner.gyro_sample_covariance,
                    )),
                    Some(&mut expected_cross),
                )
                .unwrap();
            core.fuse_next_measurement(&mut report).unwrap();
            assert_eq!(core.filter, expected_filter, "epoch {epoch}");
            assert_eq!(
                core.history.active_imu_sample_nav_cross, expected_cross,
                "epoch {epoch}"
            );
            assert!(expected_cross.norm() > 0.0);
            // The newly fused sample must also be retained by the next
            // propagation, including a GNSS epoch at its left endpoint.
            core.propagate_to(
                SessionTime::from_ns(epoch + 1_000_000),
                &mut WorkQuota::new(1_000),
                &mut report,
            )
            .unwrap();
        }
    });
}

#[test]
fn initialization_sample_cross_rotates_with_navigation_and_clears() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        {
            let mut core = LiveCore::attach(&mut state, &mut history);
            let mut sample = imu(0, 5_000_000);
            sample.gyro_sample_covariance =
                CompactCovariance3::from_matrix(Matrix3::identity() * 0.2).unwrap();
            let lever = Vector3::new(0.8, -0.2, 0.1);
            core.ingest_imu(sample).unwrap();
            core.seed_initial_imu_sample(sample, lever).unwrap();
            let initial_cross = core.history.active_imu_sample_nav_cross;
            assert_eq!(
                initial_cross.fixed_view::<3, 3>(super::super::state::VEL, 3),
                -super::super::state::skew(&lever) * 0.2
            );
            let old = anchor(0, Vector3::new(6_378_137.0, 0.0, 0.0), 0.0);
            let new = anchor(1, old.origin_ecef_m, 0.3);
            let transform = ReanchorTransform::between(&old, &new).unwrap();
            core.reanchor(&old, &new).unwrap();
            assert_eq!(
                core.history.active_imu_sample_nav_cross,
                transform.covariance_jacobian * initial_cross
            );
            assert_eq!(
                core.history
                    .active_imu_sample
                    .unwrap()
                    .gyro_sample_covariance_b,
                sample.gyro_sample_covariance
            );
        }
        history.clear();
        assert!(history.is_empty());
        assert_eq!(
            history.active_imu_sample_nav_cross,
            GapNavCrossCovariance::zeros()
        );
    });
}

#[test]
fn a_measurement_at_gap_start_retains_the_held_sample_correlation() {
    fn run(with_measurement: bool) -> (NavMatrix, GapNavCrossCovariance) {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(config(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        let mut sample = imu(0, 5_000_000);
        sample.accel_sample_covariance =
            CompactCovariance3::from_matrix(Matrix3::identity() * 9.0).unwrap();
        sample.gyro_sample_covariance =
            CompactCovariance3::from_matrix(Matrix3::identity() * 4.0).unwrap();
        core.ingest_imu(sample).unwrap();
        let mut next = sample;
        next.start = SessionTime::from_ns(7_500_000);
        next.end = SessionTime::from_ns(10_000_000);
        core.ingest_imu(next).unwrap();
        if with_measurement {
            let mut observation = gnss(5_000_000, 1, 0.0);
            observation.value.receiver_healthy = false;
            core.ingest_gnss(observation).unwrap();
        }
        core.finish().unwrap();
        core.drain_through(
            &mut WorkQuota::new(1_000),
            Some(SessionTime::from_ns(7_500_000)),
        )
        .unwrap();
        (
            core.filter.covariance,
            core.history.active_imu_sample_nav_cross,
        )
    }
    with_large_stack(|| {
        let uninterrupted = run(false);
        let with_boundary = run(true);
        assert!((uninterrupted.0 - with_boundary.0).norm() < 1.0e-7);
        assert!((uninterrupted.1 - with_boundary.1).norm() < 1.0e-8);
    });
}
