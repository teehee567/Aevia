//! Shared fixtures for offline solver behavior tests.

use crate::{
    config::{
        GnssCorrelationPolicy, OfflineResourceLimits, RunControl, SharedParameterKind,
        SharedUncertaintyTreatment,
    },
    error::ProcessError,
    frame::{
        BodyLeverArm, BodyVector, CoordinateEpoch, EcefPosition, EcefVelocity,
        OrientationEcefFromBody, ReferenceEllipsoid, ReferencePoint, ReferencePointKind,
        SensorAngularRate, SensorSpecificForce, TerrestrialFrame, TerrestrialRealization,
        Wgs84Realization,
    },
    ids::{
        ClockModelId, ContentDigestV1, FrameId, GateId, InputProfileId, ObservationId,
        ReferencePointId, SessionId, SharedParameterId, SourceId, TrajectoryRevision,
    },
    math::{NonNegativeF64, UnitQuaternion},
    metric::{EventTimeSensitivity, MetricUncertaintyProvider, StateSensitivity},
    observation::{
        AxisStatus, GnssDiagnostics, GnssPosition, GnssSolutionObservation, GnssVelocity,
        ImuIntegrationEligibility, ImuObservation, ImuStatus, InputDisposition, LiveObservation,
        ReceiverHealth, RtkState, SolutionClass, TimedAngularRate, TimedDiagnostic,
        TimedSpecificForce, VelocityMethod,
    },
    offline::{
        ports::{ControlChangeEvidence, EvidenceEnd, EvidenceEvent, EvidenceManifest},
        store::{
            FixedRecordStoreKind, StoreKind, StoredCovariance, StoredIntegrationImu, StoredNominal,
            StoredStep, state_store_resource_bounds, *,
        },
    },
    provenance::{Capabilities, Capability, SpanCapabilities},
    quality::{
        CovarianceConditioning, EstimateQuality, EstimateStage, FieldValue, GnssState,
        HeadingObservability, HeadingSource, Integrity, ObservabilityReport, TimingQuality,
        UnavailableReason, Validity,
    },
    time::{
        DurationNs, ObservationTime, SampleSupport, SessionTime, SignedDurationNs, TimeSpan,
        TimingBasis,
    },
    trajectory::{DenseBridgeInput, Trajectory, TrajectoryKnot},
    uncertainty::{Covariance3, KinematicCovariance, MeasurementUncertainty},
};

use nalgebra::{DMatrix, DVector, Matrix3, UnitQuaternion as NaUnitQuaternion, Vector3};

use std::{boxed::Box, vec::Vec};

use super::{
    catalog::*, estimation::*, evidence::*, filter::*, forward::*, inertial::*, initialization::*,
    math::*, measurement::*, metric_uncertainty::*, propagation::*, publication::*, run::*,
    smoothing::*,
};

mod estimation;
mod evidence;
mod inertial;
mod initialization;
mod publication;

fn nominal(time: i64, position_x: f64) -> StoredNominal {
    StoredNominal {
        time: SessionTime::from_ns(time),
        position_ecef: [6_378_137.0 + position_x, 0.0, 0.0],
        velocity_ecef: [0.0; 3],
        orientation_ecef_from_body: UnitQuaternion::IDENTITY,
        accelerometer_bias_body: [0.0; 3],
        gyroscope_bias_body: [0.0; 3],
        colored_gnss_error: [0.0; 3],
        specific_force_body: [9.806_65, 0.0, 0.0],
        angular_rate_body: [0.0; 3],
    }
}

fn covariance(state_variance: f64, consider_variance: f64) -> StoredCovariance {
    let _ = consider_variance;
    StoredCovariance {
        state: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION) * state_variance,
        state_consider: DMatrix::zeros(NAVIGATION_DIMENSION, 1),
    }
}

fn step(time: i64, filtered_x: f64, predicted_variance: f64, filtered_variance: f64) -> StoredStep {
    StoredStep {
        connected_from_previous: time != 0,
        predicted: nominal(time, 0.0),
        filtered: nominal(time, filtered_x),
        smoothed: None,
        predicted_covariance: covariance(predicted_variance, 1.0),
        filtered_covariance: covariance(filtered_variance, 1.0),
        smoothed_covariance: None,
        transition: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
        consider_transition: DMatrix::zeros(NAVIGATION_DIMENSION, 1),
        process_covariance: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
        integration_imu: (time != 0).then_some(StoredIntegrationImu {
            start: SessionTime::from_ns(time - 1),
            end: SessionTime::from_ns(time),
            angular_rate_body: [0.0; 3],
            specific_force_body: [9.806_65, 0.0, 0.0],
        }),
        reset_basis: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
        smoothed_backward_gain: None,
        adjacent_cross_covariance: DMatrix::identity(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION),
        disposition: Some(InputDisposition::Fused),
        gnss_state: GnssState::Fixed,
        timing_quality: TimingQuality::PpsCorrelated,
        degraded_input: false,
        objective_contribution: 0.0,
    }
}

fn continue_running(_: u64) -> bool {
    true
}

fn report_progress(_: u64, _: u64) {}

fn point_time(time_ns: i64) -> ObservationTime {
    ObservationTime {
        registered_at: SessionTime::from_ns(time_ns),
        correction: SignedDurationNs::from_ns(0),
        independent_one_sigma: DurationNs::ZERO,
        clock_model: ClockModelId::new(1),
        support: SampleSupport::Point,
        basis: TimingBasis::PpsCorrelated,
    }
}

fn interval_time(end_ns: i64, duration_ns: u64) -> ObservationTime {
    ObservationTime {
        support: SampleSupport::IntervalAverage {
            duration: DurationNs::from_ns(duration_ns),
        },
        ..point_time(end_ns)
    }
}

fn numerical_initial_antenna_to_imu_jacobian(
    orientation_ecef_from_body: NaUnitQuaternion<f64>,
    angular_rate_ib_body: Vector3<f64>,
    gyroscope_bias_body: Vector3<f64>,
    lever_body: Vector3<f64>,
) -> DMatrix<f64> {
    let antenna_position = Vector3::new(10.0, -20.0, 30.0);
    let antenna_velocity = Vector3::new(4.0, -5.0, 6.0);
    let accelerometer_bias_body = Vector3::new(-0.04, 0.05, -0.06);
    let earth_rate_ecef = Vector3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let mapped = |error: &DVector<f64>| {
        let attitude_error =
            Vector3::new(error[ATTITUDE], error[ATTITUDE + 1], error[ATTITUDE + 2]);
        let perturbed_orientation =
            orientation_ecef_from_body * NaUnitQuaternion::from_scaled_axis(attitude_error);
        let rotation = perturbed_orientation.to_rotation_matrix().into_inner();
        let perturbed_gyroscope_bias = gyroscope_bias_body
            + Vector3::new(
                error[GYROSCOPE_BIAS],
                error[GYROSCOPE_BIAS + 1],
                error[GYROSCOPE_BIAS + 2],
            );
        let earth_rate_body = rotation.transpose() * earth_rate_ecef;
        let omega_eb_body = angular_rate_ib_body - perturbed_gyroscope_bias - earth_rate_body;
        let imu_position = antenna_position
            + Vector3::new(error[POSITION], error[POSITION + 1], error[POSITION + 2])
            - rotation * lever_body;
        let imu_velocity = antenna_velocity
            + Vector3::new(error[VELOCITY], error[VELOCITY + 1], error[VELOCITY + 2])
            - rotation * omega_eb_body.cross(&lever_body);
        let perturbed_accelerometer_bias = accelerometer_bias_body
            + Vector3::new(
                error[ACCELEROMETER_BIAS],
                error[ACCELEROMETER_BIAS + 1],
                error[ACCELEROMETER_BIAS + 2],
            );
        let mut output = DVector::zeros(NAVIGATION_DIMENSION);
        set_vector3(&mut output, POSITION, imu_position);
        set_vector3(&mut output, VELOCITY, imu_velocity);
        set_vector3(&mut output, ATTITUDE, attitude_error);
        set_vector3(
            &mut output,
            ACCELEROMETER_BIAS,
            perturbed_accelerometer_bias,
        );
        set_vector3(&mut output, GYROSCOPE_BIAS, perturbed_gyroscope_bias);
        output
    };

    let step = 1.0e-6;
    let mut jacobian = DMatrix::zeros(NAVIGATION_DIMENSION, NAVIGATION_DIMENSION);
    for column in 0..NAVIGATION_DIMENSION {
        let mut positive = DVector::zeros(NAVIGATION_DIMENSION);
        let mut negative = DVector::zeros(NAVIGATION_DIMENSION);
        positive[column] = step;
        negative[column] = -step;
        jacobian.set_column(
            column,
            &((mapped(&positive) - mapped(&negative)) / (2.0 * step)),
        );
    }
    jacobian
}

fn gnss_solution(
    sequence: u64,
    position_time: Option<ObservationTime>,
    velocity_time: Option<ObservationTime>,
    health: Option<ReceiverHealth>,
    correction_age: Option<(u64, u64)>,
    solution_age: Option<u64>,
) -> GnssSolutionObservation {
    let diagnostic_time = position_time
        .or(velocity_time)
        .unwrap_or_else(|| point_time(0));
    let uncertainty =
        MeasurementUncertainty::Provided(Covariance3::diagonal(0.25, 0.25, 0.25).unwrap());
    GnssSolutionObservation::new(
        ObservationId::new(SourceId::new(7), sequence),
        ReferencePointId::new(4),
        position_time.map(|time| GnssPosition {
            value: EcefPosition::new(6_378_137.0, 0.0, 0.0).unwrap(),
            time,
            frame: FrameId::new(2),
            uncertainty,
            solution_class: SolutionClass::RtkFixed,
            receiver_latency: None,
        }),
        velocity_time.map(|time| GnssVelocity {
            value: EcefVelocity::new(0.0, 12.0, 0.0).unwrap(),
            time,
            frame: FrameId::new(2),
            uncertainty,
            solution_class: SolutionClass::RtkFixed,
            method: VelocityMethod::Doppler,
            receiver_latency: None,
        }),
        None,
        RtkState::Fixed,
        GnssDiagnostics {
            dop: None,
            used_signals: None,
            correction_age: correction_age.map(|(value, age)| TimedDiagnostic {
                value: DurationNs::from_ns(value),
                time: diagnostic_time,
                age: DurationNs::from_ns(age),
            }),
            solution_age: solution_age.map(|value| TimedDiagnostic {
                value: DurationNs::from_ns(value),
                time: diagnostic_time,
                age: DurationNs::ZERO,
            }),
            health: health.map(|value| TimedDiagnostic {
                value,
                time: diagnostic_time,
                age: DurationNs::ZERO,
            }),
        },
    )
    .unwrap()
}

fn with_health_diagnostic(
    solution: GnssSolutionObservation,
    health: TimedDiagnostic<ReceiverHealth>,
) -> GnssSolutionObservation {
    GnssSolutionObservation::new(
        solution.id(),
        solution.antenna_reference_point(),
        solution.position(),
        solution.velocity(),
        solution.position_velocity_cross_covariance(),
        solution.rtk_state(),
        GnssDiagnostics {
            dop: None,
            used_signals: None,
            correction_age: None,
            solution_age: None,
            health: Some(health),
        },
    )
    .unwrap()
}

fn imu_observation(
    rate_time: ObservationTime,
    force_time: ObservationTime,
    reset: bool,
) -> ImuObservation {
    let uncertainty =
        MeasurementUncertainty::Provided(Covariance3::diagonal(0.01, 0.01, 0.01).unwrap());
    ImuObservation::new(
        ObservationId::new(SourceId::new(3), 1),
        FrameId::new(5),
        InputProfileId::new(6),
        TimedAngularRate {
            value: SensorAngularRate::new(0.0, 0.0, 0.0).unwrap(),
            time: rate_time,
            uncertainty,
            axes: AxisStatus::VALID,
        },
        TimedSpecificForce {
            value: SensorSpecificForce::new(0.0, 0.0, 9.8).unwrap(),
            time: force_time,
            uncertainty,
            axes: AxisStatus::VALID,
        },
        if reset {
            ImuStatus::Discontinuity
        } else {
            ImuStatus::Valid
        },
    )
    .unwrap()
}
