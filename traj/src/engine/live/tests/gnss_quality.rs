//! Gnss quality.

use super::*;

#[test]
fn gnss_quality_changes_only_after_an_actual_accepted_update() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            fuse_nominal_position_at_25ms(session);
            let accepted = session.last_gnss_evidence;

            let too_late = position_update(3, 25_000_000, 0.0, healthy_at(25_000_000), None);
            let update = session
                .step(LiveStep {
                    observation: Some(&too_late),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert_eq!(
                update.input,
                Some((too_late.id(), InputDisposition::TooLateForLive))
            );
            drop(update);
            assert_eq!(session.last_gnss_evidence, accepted);

            let outlier = position_update(4, 30_000_000, 1_000_000.0, healthy_at(30_000_000), None);
            session
                .step(LiveStep {
                    observation: Some(&outlier),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            let imu = stationary_imu(8, 40_000_000);
            let update = session
                .step(LiveStep {
                    observation: Some(&imu),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert!(matches!(
                update.fusion,
                Some(FusionOutcome {
                    disposition: InputDisposition::StatisticallyRejected,
                    ..
                })
            ));
            drop(update);
            assert_eq!(session.last_gnss_evidence, accepted);
        });
    });
}

#[test]
fn stale_timed_diagnostics_are_rejected_without_becoming_quality_evidence() {
    with_large_stack(|| {
        with_navigating_live_session_and_gnss_age(DurationNs::from_ns(10_000_000), |session| {
            let stale_solution_age = TimedDiagnostic {
                value: DurationNs::from_ns(8_000_000),
                time: point_time(20_000_000),
                age: DurationNs::from_ns(3_000_000),
            };
            let observation = position_update(
                2,
                25_000_000,
                0.0,
                healthy_at(25_000_000),
                Some(stale_solution_age),
            );
            session
                .step(LiveStep {
                    observation: Some(&observation),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            let imu = stationary_imu(7, 35_000_000);
            let update = session
                .step(LiveStep {
                    observation: Some(&imu),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            assert!(matches!(
                update.fusion,
                Some(FusionOutcome {
                    disposition: InputDisposition::StatisticallyRejected,
                    normalized_innovation_squared: None,
                    ..
                })
            ));
            drop(update);
            assert!(session.last_gnss_evidence.is_none());
        });
    });
}

#[test]
fn gnss_outage_ages_present_quality_and_live_phase_to_degraded() {
    with_large_stack(|| {
        with_navigating_live_session_and_gnss_age(DurationNs::from_ns(10_000_000), |session| {
            fuse_nominal_position_at_25ms(session);
            {
                let present = session.present_projection().unwrap().unwrap();
                assert_eq!(present.quality.gnss, GnssState::Healthy);
                assert_eq!(present.quality.integrity, Integrity::Monitored);
                assert_eq!(session.phase(), LivePhase::Navigating);
            }

            let imu = stationary_imu(8, 40_000_000);
            let update = session
                .step(LiveStep {
                    observation: Some(&imu),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();
            let present = update.present.unwrap();
            assert_eq!(present.quality.gnss, GnssState::Absent);
            assert_eq!(present.quality.validity, Validity::Degraded);
            assert_eq!(present.quality.integrity, Integrity::Unavailable);
            assert_eq!(update.phase, LivePhase::Degraded);
        });
    });
}

#[test]
fn one_drain_stamps_each_historical_segment_from_ordered_fusion_evidence() {
    with_large_stack(|| {
        with_navigating_live_session(|session| {
            let first = position_update(2, 25_000_000, 0.0, healthy_at(25_000_000), None);
            let LiveObservation::GnssSolution(second) =
                position_update(3, 35_000_000, 0.0, healthy_at(35_000_000), None)
            else {
                unreachable!();
            };
            let mut position = second.position().unwrap();
            position.time.basis = TimingBasis::ModeledLatency;
            let second = LiveObservation::GnssSolution(
                GnssSolutionObservation::new(
                    second.id(),
                    second.antenna_reference_point(),
                    Some(position),
                    None,
                    None,
                    second.diagnostics(),
                )
                .unwrap(),
            );
            for observation in [&first, &second] {
                session
                    .step(LiveStep {
                        observation: Some(observation),
                        work: WorkQuota::new(1).unwrap(),
                    })
                    .unwrap();
            }
            let defer = stationary_imu(7, 35_000_000);
            session
                .step(LiveStep {
                    observation: Some(&defer),
                    work: WorkQuota::new(1).unwrap(),
                })
                .unwrap();
            let defer_again = stationary_imu(8, 40_000_000);
            session
                .step(LiveStep {
                    observation: Some(&defer_again),
                    work: WorkQuota::new(1).unwrap(),
                })
                .unwrap();
            let drain = stationary_imu(9, 45_000_000);
            session
                .step(LiveStep {
                    observation: Some(&drain),
                    work: WorkQuota::new(128).unwrap(),
                })
                .unwrap();

            let reference = ReferencePointId::new(1);
            assert_eq!(
                session
                    .trajectory()
                    .state_at(SessionTime::from_ns(20_000_000), reference)
                    .unwrap()
                    .quality
                    .gnss,
                GnssState::Absent
            );
            assert_eq!(
                session
                    .trajectory()
                    .state_at(SessionTime::from_ns(27_500_000), reference)
                    .unwrap()
                    .quality
                    .gnss,
                GnssState::Healthy
            );
            assert_eq!(
                session
                    .trajectory()
                    .state_at(SessionTime::from_ns(27_500_000), reference)
                    .unwrap()
                    .quality
                    .timing,
                TimingQuality::PpsCorrelated
            );
            assert_eq!(
                session
                    .trajectory()
                    .state_at(SessionTime::from_ns(35_000_000), reference)
                    .unwrap()
                    .quality
                    .gnss,
                GnssState::Healthy
            );
            assert_eq!(
                session
                    .trajectory()
                    .state_at(SessionTime::from_ns(35_000_000), reference)
                    .unwrap()
                    .quality
                    .timing,
                TimingQuality::Modeled
            );
        });
    });
}
