//! End-to-end coverage for the solver's coupled continuous uncertainty.

use super::*;

#[test]
fn offline_coupled_inertial_covariance_is_available_between_knots() {
    with_large_stack(|| {
        let (mut spec, manifest, events) = replay_fixture(false);
        spec.policy = crate::config::ProcessingPolicy::require(ProcessingLevel::OfflineSmooth);
        let point = spec
            .engine
            .installation
            .reference_points
            .iter()
            .find(|point| point.kind() == ReferencePointKind::ImuSensingCenter)
            .unwrap()
            .id();
        let prepared = TrajectoryEngine::process(spec)
            .preflight(manifest, offline_limits())
            .unwrap();
        let mut source = SliceEvidenceSource::new(manifest, &events);
        let mut sink = RecordingSink::default();
        let run = prepared.run(&mut source, &mut sink, run_control()).unwrap();
        let estimate = run
            .trajectory
            .state_at(SessionTime::from_ns(17_500_000), point)
            .unwrap();
        assert_ne!(
            estimate.quality.covariance,
            crate::quality::CovarianceConditioning::Unavailable
        );
        assert!(
            estimate
                .covariance
                .position()
                .to_matrix()
                .iter()
                .flatten()
                .all(|value| value.is_finite())
        );
        let antenna = run
            .trajectory
            .state_at(SessionTime::from_ns(17_500_000), ReferencePointId::new(2))
            .unwrap();
        assert_ne!(
            antenna.quality.covariance,
            crate::quality::CovarianceConditioning::Unavailable
        );
        assert_eq!(
            antenna.angular_rate_uncertainty_support,
            Some(
                TimeSpan::new(
                    SessionTime::from_ns(15_000_000),
                    SessionTime::from_ns(20_000_000)
                )
                .unwrap()
            )
        );
        assert_ne!(antenna.covariance, estimate.covariance);
        for time in [15_000_000, 17_500_000, 20_000_000] {
            let estimate = run
                .trajectory
                .state_at(SessionTime::from_ns(time), ReferencePointId::new(2))
                .unwrap();
            let covariance = estimate.covariance.velocity().to_matrix();
            for axis in 0..3 {
                assert!(covariance[axis][axis] >= 0.0);
            }
        }
    });
}
