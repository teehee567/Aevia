//! Observation fixture.

use super::*;

pub(super) fn point_time(time_ns: i64) -> ObservationTime {
    ObservationTime {
        registered_at: SessionTime::from_ns(time_ns),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::ZERO,
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis: TimingBasis::PpsCorrelated,
    }
}

pub(super) fn imu_time_for_model(end_ns: i64, clock_model: ClockModelId) -> ObservationTime {
    ObservationTime {
        support: SampleSupport::IntervalAverage {
            duration: DurationNs::from_ns(5_000_000),
        },
        clock_model,
        ..point_time(end_ns)
    }
}

pub(super) fn initialization_fix() -> LiveObservation {
    let time = point_time(5_000_000);
    let covariance = MeasurementUncertainty::Provided(covariance(0.01));
    LiveObservation::GnssSolution(
        GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), 1),
            ReferencePointId::new(2),
            Some(GnssPosition {
                value: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
                time,
                frame: FrameId::new(10),
                uncertainty: covariance,
                solution_class: SolutionClass::RtkFixed,
                receiver_latency: None,
            }),
            Some(GnssVelocity {
                value: EcefVelocity::new(0.0, 0.001, 0.0).unwrap(),
                time,
                frame: FrameId::new(10),
                uncertainty: covariance,
                solution_class: SolutionClass::RtkFixed,
                method: VelocityMethod::Doppler,
                receiver_latency: None,
            }),
            None,
            RtkState::Fixed,
            GnssDiagnostics {
                dop: None,
                used_signals: None,
                correction_age: None,
                solution_age: None,
                health: Some(TimedDiagnostic {
                    value: ReceiverHealth::Healthy,
                    time,
                    age: DurationNs::ZERO,
                }),
            },
        )
        .unwrap(),
    )
}

pub(super) fn asynchronous_solution_update(
    sequence: u64,
    position_epoch_ns: i64,
    velocity_epoch_ns: i64,
) -> LiveObservation {
    let position_time = point_time(position_epoch_ns);
    let velocity_time = point_time(velocity_epoch_ns);
    let covariance = MeasurementUncertainty::Provided(covariance(0.01));
    LiveObservation::GnssSolution(
        GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), sequence),
            ReferencePointId::new(2),
            Some(GnssPosition {
                value: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
                time: position_time,
                frame: FrameId::new(10),
                uncertainty: covariance,
                solution_class: SolutionClass::RtkFixed,
                receiver_latency: None,
            }),
            Some(GnssVelocity {
                value: EcefVelocity::new(0.0, 0.001, 0.0).unwrap(),
                time: velocity_time,
                frame: FrameId::new(10),
                uncertainty: covariance,
                solution_class: SolutionClass::RtkFixed,
                method: VelocityMethod::Doppler,
                receiver_latency: None,
            }),
            None,
            RtkState::Fixed,
            GnssDiagnostics {
                dop: None,
                used_signals: None,
                correction_age: None,
                solution_age: None,
                health: Some(TimedDiagnostic {
                    value: ReceiverHealth::Healthy,
                    time: position_time,
                    age: DurationNs::ZERO,
                }),
            },
        )
        .unwrap(),
    )
}

pub(super) fn position_update(
    sequence: u64,
    epoch_ns: i64,
    ecef_x_offset_m: f64,
    rtk_state: RtkState,
    health: TimedDiagnostic<ReceiverHealth>,
    solution_age: Option<TimedDiagnostic<DurationNs>>,
) -> LiveObservation {
    let time = point_time(epoch_ns);
    LiveObservation::GnssSolution(
        GnssSolutionObservation::new(
            ObservationId::new(SourceId::new(2), sequence),
            ReferencePointId::new(2),
            Some(GnssPosition {
                value: EcefPosition::new(6_378_137.0 + ecef_x_offset_m, 0.0, 0.0).unwrap(),
                time,
                frame: FrameId::new(10),
                uncertainty: MeasurementUncertainty::Provided(covariance(1.0)),
                solution_class: match rtk_state {
                    RtkState::Fixed => SolutionClass::RtkFixed,
                    RtkState::Float => SolutionClass::RtkFloat,
                    RtkState::Dgps => SolutionClass::Dgps,
                    RtkState::Ppp => SolutionClass::Ppp,
                    RtkState::Standalone => SolutionClass::Standalone,
                    RtkState::Invalid => SolutionClass::Invalid,
                },
                receiver_latency: None,
            }),
            None,
            None,
            rtk_state,
            GnssDiagnostics {
                dop: None,
                used_signals: None,
                correction_age: None,
                solution_age,
                health: Some(health),
            },
        )
        .unwrap(),
    )
}

pub(super) fn healthy_at(epoch_ns: i64) -> TimedDiagnostic<ReceiverHealth> {
    TimedDiagnostic {
        value: ReceiverHealth::Healthy,
        time: point_time(epoch_ns),
        age: DurationNs::ZERO,
    }
}

pub(super) fn stationary_imu(sequence: u64, end_ns: i64) -> LiveObservation {
    stationary_imu_for_model(sequence, end_ns, ClockModelId::new(1))
}

pub(super) fn stationary_imu_with_specific_force(
    sequence: u64,
    end_ns: i64,
    components: [f64; 3],
) -> LiveObservation {
    let LiveObservation::Imu(original) = stationary_imu(sequence, end_ns) else {
        unreachable!();
    };
    let mut force = original.specific_force();
    force.value = SensorSpecificForce::from_components(components).unwrap();
    LiveObservation::Imu(
        ImuObservation::new(
            original.id(),
            original.measurement_frame(),
            original.profile(),
            original.angular_rate(),
            force,
            original.status(),
        )
        .unwrap(),
    )
}

pub(super) fn imu_with_status(observation: LiveObservation, status: ImuStatus) -> LiveObservation {
    let LiveObservation::Imu(imu) = observation else {
        panic!("IMU fixture required")
    };
    LiveObservation::Imu(
        ImuObservation::new(
            imu.id(),
            imu.measurement_frame(),
            imu.profile(),
            imu.angular_rate(),
            imu.specific_force(),
            status,
        )
        .unwrap(),
    )
}

pub(super) fn stationary_imu_for_model(
    sequence: u64,
    end_ns: i64,
    clock_model: ClockModelId,
) -> LiveObservation {
    let time = imu_time_for_model(end_ns, clock_model);
    let uncertainty = MeasurementUncertainty::Provided(covariance(1.0e-4));
    LiveObservation::Imu(
        ImuObservation::new(
            ObservationId::new(SourceId::new(1), sequence),
            FrameId::new(20),
            InputProfileId::new(1),
            TimedAngularRate {
                value: SensorAngularRate::new(0.0, 0.0, 0.0).unwrap(),
                time,
                uncertainty,
                axes: AxisStatus::VALID,
            },
            TimedSpecificForce {
                value: SensorSpecificForce::new(0.0, 0.0, 9.806_65).unwrap(),
                time,
                uncertainty,
                axes: AxisStatus::VALID,
            },
            ImuStatus::Valid,
        )
        .unwrap(),
    )
}
