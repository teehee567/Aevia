use super::*;

// engine() initializes static profile arrays: keep their borrowed lifetimes
// exclusive across the profile-backed session fixtures.
static PROFILE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn sample() -> GnssSample {
    GnssSample {
        position_m: [6_378_137.0, 0.0, 0.0],
        velocity_mps: [0.0; 3],
        position_sigma_m: [1.0; 3],
        velocity_sigma_mps: [0.1; 3],
        epoch_ms: 1000,
        received_us: 1_000_000,
        position_time_ns: 1_000_000_000,
        velocity_time_ns: 875_000_000,
        timing_uncertainty_ns: 500_000,
        clock_generation: 1,
        position_valid: true,
        velocity_valid: true,
        satellites: 20,
    }
}

#[test]
fn health_is_available_at_each_independent_measurement_epoch() {
    let observation = observation(sample(), 1, FrameId::new(10)).unwrap();
    let health = observation.diagnostics().health.unwrap();
    assert_eq!(health.value, ReceiverHealth::Healthy);
    assert_eq!(health.time, observation.velocity().unwrap().time);
    assert!(
        health.time.effective_time().unwrap()
            < observation
                .position()
                .unwrap()
                .time
                .effective_time()
                .unwrap()
    );
}

#[test]
fn invalid_companions_do_not_poison_valid_measurement_health() {
    for (position_valid, velocity_valid) in [(false, true), (true, false)] {
        let observation = observation(
            GnssSample {
                position_valid,
                velocity_valid,
                ..sample()
            },
            1,
            FrameId::new(10),
        )
        .unwrap();
        let health = observation.diagnostics().health.unwrap();
        assert_eq!(health.value, ReceiverHealth::Healthy);
        let valid_time = if velocity_valid {
            observation.velocity().unwrap().time
        } else {
            observation.position().unwrap().time
        };
        assert_eq!(health.time, valid_time);
    }
}

#[test]
fn session_fuses_latency_corrected_velocity_and_each_valid_companion() {
    // Exercise the production adapter and the public estimator API together.
    // Large workspaces belong on the host thread's explicitly sized stack.
    std::thread::Builder::new()
        .stack_size(32 * 1024 * 1024)
        .spawn(|| {
            let _profile = PROFILE_LOCK.lock().unwrap();
            use aevia_trajectory::{
                TrajectoryEngine,
                config::{
                    ClockSharedCrossCovariance, InitialClockConsiderPrior, InitialHeading,
                    LiveResourceLimits, LiveSpec,
                },
                frame::{SensorAngularRate, SensorSpecificForce},
                ids::{ClockSegmentId, InputProfileId, SessionId},
                math::FiniteF64,
                metric::{LiveMetricLimits, LiveMetricPlan, MetricPlan},
                observation::{
                    AxisStatus, ImuObservation, ImuStatus, LiveObservation, LiveStep,
                    TimedAngularRate, TimedSpecificForce, WorkQuota,
                },
                uncertainty::Variance,
                workspace::{
                    LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion,
                },
            };
            use std::boxed::Box;
            let engine = unsafe { crate::device::profile::engine() };
            let mut metrics = Box::new(LiveMetricPlan::placeholder());
            MetricPlan::new(1)
                .compile_live_into(LiveMetricLimits::default(), &mut metrics)
                .unwrap();
            let plan = TrajectoryEngine::live(LiveSpec {
                session_id: SessionId::from_bytes(*b"GNSS-REGRESSION!"),
                engine,
                metrics: &metrics,
                resources: LiveResourceLimits::V2_MINI_RTS,
                initial_heading: Some(
                    InitialHeading::new(0.0, Variance::new(0.01).unwrap()).unwrap(),
                ),
                initial_clock_prior: InitialClockConsiderPrior {
                    model: ClockModelId::new(1),
                    segment: ClockSegmentId::new(1),
                    reference_time: SessionTime::ZERO,
                    offset_variance_s2: Variance::new(1e-12).unwrap(),
                    drift_variance: Variance::new(1e-12).unwrap(),
                    offset_drift_covariance_s: FiniteF64::new(0.0).unwrap(),
                    cross_covariance_with_shared: ClockSharedCrossCovariance::independent(6)
                        .unwrap(),
                },
            })
            .preflight_development()
            .unwrap();
            let mut internal = Box::new(LiveInternalWorkspace::new());
            let mut psram = Box::new(LivePsramWorkspace::new(crate::device::profile::FRAME));
            let mut session = plan
                .start(LiveWorkspace::bind(
                    &mut internal,
                    MemoryRegion::InternalSram,
                    &mut psram,
                    MemoryRegion::Psram,
                ))
                .unwrap();
            // This regression isolates adapter semantics; drain the delayed
            // frontier fully rather than testing the device's scheduling budget.
            let quota = WorkQuota::new(WorkQuota::MAX_UNITS).unwrap();
            let imu = |sequence: u64| {
                let time = ObservationTime {
                    registered_at: SessionTime::from_ns(sequence as i64 * 5_000_000),
                    support: SampleSupport::IntervalAverage {
                        duration: DurationNs::from_ns(5_000_000),
                    },
                    ..observation_time(0, 0).unwrap()
                };
                let covariance = MeasurementUncertainty::Provided(
                    Covariance3::diagonal(1e-6, 1e-6, 1e-6).unwrap(),
                );
                LiveObservation::Imu(
                    ImuObservation::new(
                        ObservationId::new(SourceId::new(1), sequence),
                        FrameId::new(20),
                        InputProfileId::new(1),
                        TimedAngularRate {
                            value: SensorAngularRate::new(0.0, 0.0, 0.0).unwrap(),
                            time,
                            uncertainty: covariance,
                            axes: AxisStatus::VALID,
                        },
                        TimedSpecificForce {
                            value: SensorSpecificForce::new(0.0, 0.0, 9.80665).unwrap(),
                            time,
                            uncertainty: covariance,
                            axes: AxisStatus::VALID,
                        },
                        ImuStatus::Valid,
                    )
                    .unwrap(),
                )
            };
            let mut gnss_sequence = 0;
            // A valid position/velocity pair must remain paired for initialization.
            for sequence in 1..=400 {
                if sequence % 4 == 0 {
                    gnss_sequence += 1;
                    let time = sequence * 5_000_000;
                    let fix = LiveObservation::GnssSolution(
                        observation(
                            GnssSample {
                                position_time_ns: time,
                                velocity_time_ns: time,
                                timing_uncertainty_ns: 0,
                                ..sample()
                            },
                            gnss_sequence,
                            engine.processing_frame.id(),
                        )
                        .unwrap(),
                    );
                    session
                        .step(LiveStep {
                            observation: Some(&fix),
                            work: quota,
                        })
                        .unwrap();
                }
                session
                    .step(LiveStep {
                        observation: Some(&imu(sequence)),
                        work: quota,
                    })
                    .unwrap();
            }
            assert!(
                session.trajectory().span().is_some(),
                "paired GNSS must initialize navigation"
            );
            // Drain startup's delayed fusion queue before measuring each case.
            for sequence in 401..=440 {
                session
                    .step(LiveStep {
                        observation: Some(&imu(sequence)),
                        work: quota,
                    })
                    .unwrap();
            }
            let mut sequence = 440;
            for (position_valid, velocity_valid, latency_ns, expected_updates) in [
                (true, true, 1_000_000, 2),
                (false, true, 0, 1),
                (true, false, 0, 1),
            ] {
                let time = (sequence + 1) * 5_000_000;
                gnss_sequence += 1;
                let fix = LiveObservation::GnssSolution(
                    observation(
                        GnssSample {
                            position_time_ns: time,
                            velocity_time_ns: time - latency_ns,
                            position_valid,
                            velocity_valid,
                            timing_uncertainty_ns: 0,
                            ..sample()
                        },
                        gnss_sequence,
                        engine.processing_frame.id(),
                    )
                    .unwrap(),
                );
                let before = session.diagnostics();
                session
                    .step(LiveStep {
                        observation: Some(&fix),
                        work: quota,
                    })
                    .unwrap();
                for next in sequence + 1..=sequence + 40 {
                    session
                        .step(LiveStep {
                            observation: Some(&imu(next)),
                            work: quota,
                        })
                        .unwrap();
                }
                for _ in 0..8 {
                    session
                        .step(LiveStep {
                            observation: None,
                            work: quota,
                        })
                        .unwrap();
                }
                sequence += 40;
                let after = session.diagnostics();
                assert_eq!(
                    after.gnss_updates_fused - before.gnss_updates_fused,
                    expected_updates,
                    "position={position_valid} velocity={velocity_valid} latency={latency_ns}"
                );
                assert_eq!(after.gnss_updates_rejected, before.gnss_updates_rejected);
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn only_an_engine_accepted_position_velocity_pair_can_start_inertial_initialization() {
    use aevia_trajectory::observation::InputDisposition;
    for (position_valid, velocity_valid) in
        [(true, true), (false, true), (true, false), (false, false)]
    {
        let fix = observation(
            GnssSample {
                position_valid,
                velocity_valid,
                ..sample()
            },
            1,
            FrameId::new(10),
        )
        .unwrap();
        assert_eq!(
            initialization_seed_accepted(fix, InputDisposition::InitializationOnly),
            position_valid && velocity_valid
        );
        for disposition in [
            InputDisposition::RetainedForOffline,
            InputDisposition::TooLateForLive,
            InputDisposition::StatisticallyRejected,
        ] {
            assert!(!initialization_seed_accepted(fix, disposition));
        }
    }
}

#[test]
fn device_quota_preserves_fusion_frontier_at_realistic_sensor_cadence() {
    std::thread::Builder::new()
        .stack_size(32 * 1024 * 1024)
        .spawn(|| {
            let _profile = PROFILE_LOCK.lock().unwrap();
            use aevia_trajectory::{
                TrajectoryEngine,
                config::{
                    ClockSharedCrossCovariance, InitialClockConsiderPrior, LiveResourceLimits,
                    LiveSpec,
                },
                frame::{SensorAngularRate, SensorSpecificForce},
                ids::{ClockSegmentId, InputProfileId, SessionId},
                math::FiniteF64,
                metric::{LiveMetricLimits, LiveMetricPlan, MetricPlan},
                observation::{
                    AxisStatus, ImuObservation, ImuStatus, LiveObservation, LiveStep,
                    TimedAngularRate, TimedSpecificForce, WorkQuota,
                },
                uncertainty::Variance,
                workspace::{
                    LiveInternalWorkspace, LivePsramWorkspace, LiveWorkspace, MemoryRegion,
                },
            };
            use std::boxed::Box;
            let engine = unsafe { crate::device::profile::engine() };
            let mut metrics = Box::new(LiveMetricPlan::placeholder());
            MetricPlan::new(1)
                .compile_live_into(LiveMetricLimits::default(), &mut metrics)
                .unwrap();
            let plan = TrajectoryEngine::live(LiveSpec {
                session_id: SessionId::from_bytes(*b"DEVICE-QUOTA-128"),
                engine,
                metrics: &metrics,
                resources: LiveResourceLimits::V2_MINI_RTS,
                initial_heading: None,
                initial_clock_prior: InitialClockConsiderPrior {
                    model: ClockModelId::new(1),
                    segment: ClockSegmentId::new(1),
                    reference_time: SessionTime::ZERO,
                    offset_variance_s2: Variance::new(5.0e-6 * 5.0e-6).unwrap(),
                    drift_variance: Variance::new(500.0e-6 * 500.0e-6).unwrap(),
                    offset_drift_covariance_s: FiniteF64::new(0.0).unwrap(),
                    cross_covariance_with_shared: ClockSharedCrossCovariance::independent(6)
                        .unwrap(),
                },
            })
            .preflight_development()
            .unwrap();
            let mut internal = Box::new(LiveInternalWorkspace::new());
            let mut psram = Box::new(LivePsramWorkspace::new(crate::device::profile::FRAME));
            let mut session = plan
                .start(LiveWorkspace::bind(
                    &mut internal,
                    MemoryRegion::InternalSram,
                    &mut psram,
                    MemoryRegion::Psram,
                ))
                .unwrap();
            session.use_development_imu_timing_approximation().unwrap();
            let quota = WorkQuota::new(128).unwrap();
            let gyro_covariance = MeasurementUncertainty::Provided(
                Covariance3::diagonal(0.001 * 0.001, 0.001 * 0.001, 0.001 * 0.001).unwrap(),
            );
            let acceleration_covariance = MeasurementUncertainty::Provided(
                Covariance3::diagonal(0.03 * 0.03, 0.03 * 0.03, 0.03 * 0.03).unwrap(),
            );
            const BASE_NS: u64 = 2_000_000_000;
            const RATE_HZ: u64 = 1454;
            let mut previous_capture = BASE_NS;
            let mut gnss_epoch = BASE_NS + 20_000_000;
            let mut gnss_sequence = 0;
            let mut seeded = false;
            let mut forwarded_imus = 0;
            let mut frontier_times = std::vec::Vec::new();
            for sequence in 1..=RATE_HZ * 2 {
                // Quantize the sensor cadence to the same 100ns capture ticks.
                let capture = BASE_NS + (sequence * 10_000_000 / RATE_HZ) * 100;
                // GNSS delivery follows its measurement epoch by 5ms; velocity
                // is separately end-stamped 1ms earlier, as in receiver output.
                while gnss_epoch + 5_000_000 <= capture {
                    gnss_sequence += 1;
                    let solution = observation(
                        GnssSample {
                            epoch_ms: gnss_epoch / 1_000_000,
                            received_us: (gnss_epoch + 5_000_000) / 1000,
                            position_time_ns: gnss_epoch,
                            velocity_time_ns: gnss_epoch - 1_000_000,
                            ..sample()
                        },
                        gnss_sequence,
                        engine.processing_frame.id(),
                    )
                    .unwrap();
                    let input = LiveObservation::GnssSolution(solution);
                    let update = session
                        .step(LiveStep {
                            observation: Some(&input),
                            work: quota,
                        })
                        .unwrap();
                    if let Some((_, disposition)) = update.input {
                        seeded |= initialization_seed_accepted(solution, disposition);
                    }
                    gnss_epoch += 20_000_000;
                }
                if seeded {
                    let time = ObservationTime {
                        registered_at: SessionTime::from_ns(capture as i64),
                        correction: SignedDurationNs::from_ns(-1_500_000),
                        independent_one_sigma: DurationNs::from_ns(1_200_000),
                        clock_model: ClockModelId::new(1),
                        support: SampleSupport::IntervalAverage {
                            duration: DurationNs::from_ns(capture - previous_capture),
                        },
                        basis: TimingBasis::SensorCounterAnchored,
                    };
                    let input = LiveObservation::Imu(
                        ImuObservation::new(
                            ObservationId::new(SourceId::new(1), sequence),
                            FrameId::new(20),
                            InputProfileId::new(1),
                            TimedAngularRate {
                                value: SensorAngularRate::new(0.0, 0.0, 0.0).unwrap(),
                                time,
                                uncertainty: gyro_covariance,
                                axes: AxisStatus::VALID,
                            },
                            TimedSpecificForce {
                                value: SensorSpecificForce::new(0.0, 0.0, 9.80665).unwrap(),
                                time,
                                uncertainty: acceleration_covariance,
                                axes: AxisStatus::VALID,
                            },
                            ImuStatus::Degraded,
                        )
                        .unwrap(),
                    );
                    let update = session
                        .step(LiveStep {
                            observation: Some(&input),
                            work: quota,
                        })
                        .unwrap();
                    if let Some(frontier) = update.navigation_watermark {
                        let time = frontier.as_ns();
                        if frontier_times.last() != Some(&time) {
                            frontier_times.push(time);
                        }
                    }
                    forwarded_imus += 1;
                }
                previous_capture = capture;
            }
            let diagnostics = session.diagnostics();
            assert!(seeded && gnss_sequence >= 90);
            assert_eq!(diagnostics.imu_epochs_accepted, forwarded_imus);
            assert_eq!(diagnostics.imu_epochs_rejected, 0);
            assert_eq!(diagnostics.gnss_updates_rejected, 0);
            assert_eq!(diagnostics.observations_too_late, 0);
            assert_eq!(diagnostics.reinitializations, 0);
            assert_eq!(diagnostics.output_overflows, 0);
            assert!(diagnostics.gnss_updates_fused >= 50, "{diagnostics:?}");
            let end = session
                .trajectory()
                .span()
                .expect("bounded quota must publish a trajectory")
                .end();
            let lag_ns = previous_capture - end.as_ns() as u64;
            let budget = crate::device::profile::FUSION_DELAY.as_ns()
                + crate::device::profile::SMOOTHING_LAG.as_ns()
                + 30_000_000;
            assert!(
                lag_ns <= budget,
                "frontier lag {lag_ns}ns exceeds {budget}ns with device quota"
            );
            assert!(frontier_times.len() >= 300, "frontier advances={}", frontier_times.len());
            let cadence_ns = 1_000_000_000 / i64::from(engine.navigation_profile.navigation_cadence_hz);
            let maximum_frontier_step = frontier_times.windows(2).skip(1).map(|pair| pair[1] - pair[0]).max().unwrap();
            assert!(maximum_frontier_step <= cadence_ns * 2, "maximum frontier step={maximum_frontier_step}ns");
            std::println!("bounded quota128: {forwarded_imus} IMUs accepted, {gnss_sequence} GNSS epochs, {} fused fields, {} frontier advances, max frontier step={maximum_frontier_step}ns, final lag={lag_ns}ns", diagnostics.gnss_updates_fused, frontier_times.len());
            // This is deterministic work-budget/semantic validation on a host;
            // it places no bound on CPU execution time on the physical board.
        })
        .unwrap()
        .join()
        .unwrap();
}
