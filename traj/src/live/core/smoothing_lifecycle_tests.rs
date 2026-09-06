//! RTS lifecycle checks that avoid dependence on backward-solve numerics.

use super::*;
use crate::live::{
    eskf::RtsUpdateCapture,
    smoothing::{AUG_DIM, NavAugmentedMatrix},
    state::{NAV_DIM, POS},
};

fn tuning() -> LiveCoreConfig {
    LiveCoreConfig {
        smoothing_lag_ns: 100_000_000,
        ..config()
    }
}

#[test]
fn pending_rts_measurement_capture_rotates_and_round_trips_with_reanchor() {
    with_large_stack(|| {
        let old = anchor(1, Vector3::new(6_000_000.0, 1_000.0, 2_000.0), -0.15);
        let new = anchor(2, Vector3::new(6_000_120.0, 970.0, 2_008.0), 0.42);
        let mut history = LiveCoreHistory::new();
        let mut initialization = seed();
        initialization.initialization.covariance[(POS, POS)] = 4.0;
        let mut state = LiveCoreState::new(tuning(), initialization.borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        core.ingest_imu(imu(0, 5_000_000)).unwrap();
        core.ingest_imu(imu(5_000_000, 10_000_000)).unwrap();
        core.ingest_imu(imu(10_000_000, 15_000_000)).unwrap();
        core.ingest_gnss(gnss(2_500_000, 1, 0.01)).unwrap();
        let mut report = DrainReport::new();
        assert!(
            core.propagate_to(
                SessionTime::from_ns(2_500_000),
                &mut WorkQuota::new(u16::MAX),
                &mut report
            )
            .unwrap()
        );
        core.fuse_next_measurement(&mut report).unwrap();
        assert!(core.pending_corrected_segment.is_some());
        let before = core.history.smoothing_update.nav_transform;
        let transform = ReanchorTransform::between(&old, &new).unwrap();
        let mut expected = NavAugmentedMatrix::zeros();
        let left = transform.covariance_jacobian * before;
        expected.fixed_columns_mut::<NAV_DIM>(0).copy_from(
            &(left.fixed_columns::<NAV_DIM>(0) * transform.covariance_jacobian.transpose()),
        );
        expected
            .fixed_columns_mut::<{ AUG_DIM - NAV_DIM }>(NAV_DIM)
            .copy_from(&left.fixed_columns::<{ AUG_DIM - NAV_DIM }>(NAV_DIM));
        assert!((expected - before).norm() > 0.01);
        core.reanchor(&old, &new).unwrap();
        assert!((core.history.smoothing_update.nav_transform - expected).norm() < 1.0e-6);
        let old_again = EcefAnchor::new(3, old.origin_ecef_m, old.ecef_to_n).unwrap();
        core.reanchor(&new, &old_again).unwrap();
        assert!((core.history.smoothing_update.nav_transform - before).norm() < 1.0e-6);
        assert!(core.history.smoothing.has_tail());
    });
}

#[test]
fn exact_clock_boundary_flushes_rts_tail_before_consider_transition() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut initialization = seed();
        initialization.active_consider = 2;
        initialization.consider_covariance[(0, 0)] = 0.04;
        initialization.consider_covariance[(1, 1)] = 0.01;
        let mut state = LiveCoreState::new(tuning(), initialization.borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        for millisecond in 0..40 {
            core.ingest_imu(imu(millisecond * 1_000_000, (millisecond + 1) * 1_000_000))
                .unwrap();
        }
        core.drain(&mut WorkQuota::new(20)).unwrap();
        assert!(core.history.smoothing.has_tail());
        let mut clock_mapping = [[0.0; crate::live::MAX_CONSIDER]; 2];
        clock_mapping[0][0] = 2.0;
        clock_mapping[1][1] = 1.0;
        assert_eq!(
            core.transition_clock_consider(2, &clock_mapping, [0.0; 3]),
            Err(LiveCoreError::ClockTransitionRequiresReinitialization)
        );
        let at = SessionTime::from_ns(25_000_000);
        core.drain_through(&mut WorkQuota::new(u16::MAX), Some(at))
            .unwrap();
        let status = core.status().unwrap();
        assert_eq!(status.corrected_frontier, Some(at));
        assert_eq!(status.published_frontier, Some(at));
        assert!(!core.history.smoothing.has_tail());
        core.transition_clock_consider(2, &clock_mapping, [0.0; 3])
            .unwrap();
        assert!((core.filter.consider_covariance[(0, 0)] - 0.16).abs() < 1.0e-6);
        assert!(core.history.smoothing.is_empty());
        let mut previous = None;
        while let Some(segment) = core.pop_corrected_segment() {
            if let Some(end) = previous {
                assert_eq!(segment.start.state, end);
            }
            previous = Some(segment.end.state);
        }
        assert_eq!(previous.unwrap().time, at);
    });
}

#[test]
fn reset_clears_rts_tail_quality_and_capture_before_workspace_reuse() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(tuning(), seed().borrowed(), &history).unwrap();
        {
            let mut core = LiveCore::attach(&mut state, &mut history);
            for millisecond in 0..30 {
                core.ingest_imu(imu(millisecond * 1_000_000, (millisecond + 1) * 1_000_000))
                    .unwrap();
            }
            core.drain(&mut WorkQuota::new(u16::MAX)).unwrap();
            assert!(core.history.smoothing.has_tail());
        }
        state.reset();
        history.clear();
        assert!(history.is_empty());
        assert_eq!(history.smoothing_update, RtsUpdateCapture::new());
        assert_eq!(
            history.smoothing_update_transaction,
            RtsUpdateCapture::new()
        );
        assert_eq!(history.current_quality, None);
        assert_eq!(history.endpoint_quality, None);
        assert_eq!(history.published_frontier, None);
        state
            .initialize(&tuning(), &seed().borrowed(), &history)
            .unwrap();
        assert!(state.is_active());
    });
}

#[test]
fn excessive_distinct_epoch_burst_reports_rts_capacity_without_publishing_early() {
    with_large_stack(|| {
        let mut history = LiveCoreHistory::new();
        let mut state = LiveCoreState::new(tuning(), seed().borrowed(), &history).unwrap();
        let mut core = LiveCore::attach(&mut state, &mut history);
        for millisecond in 1..=100 {
            let mut observation = gnss(millisecond * 1_000_000, millisecond as u64, 0.0);
            observation.value.receiver_healthy = false;
            core.ingest_gnss(observation).unwrap();
        }
        for millisecond in 0..120 {
            core.ingest_imu(imu(millisecond * 1_000_000, (millisecond + 1) * 1_000_000))
                .unwrap();
        }
        assert_eq!(
            core.drain(&mut WorkQuota::new(u16::MAX)),
            Err(LiveCoreError::SmoothingHistoryFull)
        );
        assert_eq!(core.history.corrected.len(), 0);
        assert_eq!(core.history.published_frontier, Some(SessionTime::ZERO));
        assert!(core.history.smoothing.is_full());
    });
}
