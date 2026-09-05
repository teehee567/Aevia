use crate::error::ValidationError;
use crate::frame::{
    BodyLeverArm, CoordinateOperation, ReferencePoint, ReferencePointKind, SensorToBodyRotation,
    TerrestrialFrame,
};
use crate::ids::{
    CalibrationRevision, ClockModelId, ClockSegmentId, ContentDigestV1, DynamicsProfileId,
    InputProfileId, QualificationSpecId, SharedParameterId,
};
use crate::math::{FiniteF64, NonNegativeF64, Probability};
use crate::metric::MetricPlan;
use crate::time::{DurationNs, SessionTime, TimeSpan};
use crate::uncertainty::{
    MAX_SHARED_PARAMETER_DIMENSION, MeasurementUncertainty, SharedParameterCovariance, Variance,
};

use super::*;
use crate::{
    frame::{
        BodyVector, CoordinateEpoch, ReferenceEllipsoid, TerrestrialRealization, Wgs84Realization,
    },
    ids::{
        CoordinateOperationId, FrameId, MetricDefinitionId, ReferencePointId, UncertaintyModelId,
    },
    metric::{DistancePlan, DistanceQuantity, LiveMetricLimits, MetricDefinition},
};

#[test]
fn processing_preferences_reject_duplicates_embedded_and_best_raw_tight() {
    assert_eq!(
        ProcessingPreference::new(&[
            ProcessingLevel::OfflineSmooth,
            ProcessingLevel::OfflineSmooth
        ]),
        Err(ValidationError::IncompatibleDefinition)
    );
    assert_eq!(
        ProcessingPreference::new(&[ProcessingLevel::EmbeddedLive]),
        Err(ValidationError::IncompatibleDefinition)
    );
    assert_eq!(
        ProcessingPolicy::best_qualified(&[
            ProcessingLevel::AdvancedGraph,
            ProcessingLevel::RawTight
        ]),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn v2_resource_limits_reject_stack_outside_internal_sram() {
    let mut limits = LiveResourceLimits::V2_MINI_INITIAL;
    limits.internal_sram_bytes = 1_024;
    assert_eq!(
        limits.validate_v2_mini(),
        Err(ValidationError::CapacityExceeded)
    );
}

#[test]
fn qualification_cannot_have_blank_numeric_gates() {
    let zero = NonNegativeF64::new(0.0).unwrap();
    let spec = QualificationSpecV1 {
        id: QualificationSpecId::new(1),
        minimum_session_count: 0,
        minimum_total_duration: DurationNs::ZERO,
        maximum_position_rmse_m: zero,
        maximum_velocity_rmse_mps: zero,
        maximum_attitude_rmse_rad: zero,
        maximum_event_time_error_s: zero,
        maximum_hard_failure_rate: Probability::new(0.0).unwrap(),
        minimum_empirical_coverage: Probability::new(0.0).unwrap(),
        maximum_innovation_autocorrelation: Probability::new(0.0).unwrap(),
        maximum_cross_target_numeric_error: zero,
        maximum_root_time_residual_s: zero,
        maximum_quadrature_error: zero,
        maximum_reanchor_error_m: zero,
        minimum_fuzz_cases: 0,
        minimum_jacobian_cases: 0,
        minimum_monte_carlo_trials: 0,
        minimum_adversarial_root_cases: 0,
        digest: ContentDigestV1::from_bytes([0; 32]),
    };
    assert_eq!(
        spec.validate(),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn numeric_profile_rejects_placeholder_build_identity() {
    let profile = NumericProfileSpec {
        revision: 1,
        scalar_policy: ScalarPolicy::EmbeddedMixedF32F64,
        fma_policy: FmaPolicy::Disabled,
        minimum_rust_version: (1, 87, 0),
        fpmath_source_digest: ContentDigestV1::from_bytes([0; 32]),
        toolchain_digest: ContentDigestV1::from_bytes([1; 32]),
        digest: ContentDigestV1::from_bytes([2; 32]),
    };
    assert_eq!(
        profile.validate(),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn disabled_implicit_contraction_allows_explicit_fused_algorithm_steps() {
    let profile = NumericProfileSpec {
        revision: 1,
        scalar_policy: ScalarPolicy::EmbeddedMixedF32F64,
        fma_policy: FmaPolicy::Disabled,
        minimum_rust_version: (1, 87, 0),
        fpmath_source_digest: ContentDigestV1::from_bytes([1; 32]),
        toolchain_digest: ContentDigestV1::from_bytes([2; 32]),
        digest: ContentDigestV1::from_bytes([3; 32]),
    };
    assert_eq!(profile.validate(), Ok(profile));
}

#[test]
fn initial_clock_prior_validates_complete_shared_joint_covariance() {
    let shared_upper = [16.0, 0.0, 25.0];
    let shared = SharedParameterCovariance::new(2, &shared_upper).unwrap();
    let mut cross = [[0.0; MAX_SHARED_PARAMETER_DIMENSION]; 2];
    cross[0][0] = 1.0;
    cross[1][1] = 2.0;
    let prior = InitialClockConsiderPrior {
        model: ClockModelId::new(1),
        segment: ClockSegmentId::new(1),
        reference_time: SessionTime::ZERO,
        offset_variance_s2: Variance::new(4.0).unwrap(),
        drift_variance: Variance::new(9.0).unwrap(),
        offset_drift_covariance_s: FiniteF64::new(0.0).unwrap(),
        cross_covariance_with_shared: ClockSharedCrossCovariance::new(2, cross).unwrap(),
    };
    assert_eq!(prior.validate_with_shared(shared), Ok(prior));

    let missing_segment = InitialClockConsiderPrior {
        segment: ClockSegmentId::new(0),
        ..prior
    };
    assert_eq!(
        missing_segment.validate_with_shared(shared),
        Err(ValidationError::IncompatibleDefinition)
    );

    let wrong_dimension = InitialClockConsiderPrior {
        cross_covariance_with_shared: ClockSharedCrossCovariance::independent(1).unwrap(),
        ..prior
    };
    assert_eq!(
        wrong_dimension.validate_with_shared(shared),
        Err(ValidationError::InvalidCovariance)
    );

    cross[0][0] = 100.0;
    let non_psd = InitialClockConsiderPrior {
        cross_covariance_with_shared: ClockSharedCrossCovariance::new(2, cross).unwrap(),
        ..prior
    };
    assert_eq!(
        non_psd.validate_with_shared(shared),
        Err(ValidationError::InvalidCovariance)
    );
}

#[test]
fn initial_clock_prior_psd_check_handles_f64_product_extremes() {
    let prior = |diagonal: f64, cross: f64| InitialClockConsiderPrior {
        model: ClockModelId::new(1),
        segment: ClockSegmentId::new(1),
        reference_time: SessionTime::ZERO,
        offset_variance_s2: Variance::new(diagonal).unwrap(),
        drift_variance: Variance::new(diagonal).unwrap(),
        offset_drift_covariance_s: FiniteF64::new(cross).unwrap(),
        cross_covariance_with_shared: ClockSharedCrossCovariance::independent(0).unwrap(),
    };

    let tiny = f64::MIN_POSITIVE;
    assert_eq!(prior(tiny, tiny).validate(), Ok(prior(tiny, tiny)));
    assert_eq!(
        prior(tiny, tiny * 2.0).validate(),
        Err(ValidationError::InvalidCovariance)
    );

    let huge = f64::MAX / 4.0;
    assert_eq!(prior(huge, huge).validate(), Ok(prior(huge, huge)));
    assert_eq!(
        prior(huge, huge * 2.0).validate(),
        Err(ValidationError::InvalidCovariance)
    );
}

#[test]
fn qualification_requires_a_matching_passing_measured_report() {
    let maximum = NonNegativeF64::new(1.0).unwrap();
    let specification = QualificationSpecV1 {
        id: QualificationSpecId::new(7),
        minimum_session_count: 2,
        minimum_total_duration: DurationNs::from_ns(10),
        maximum_position_rmse_m: maximum,
        maximum_velocity_rmse_mps: maximum,
        maximum_attitude_rmse_rad: maximum,
        maximum_event_time_error_s: maximum,
        maximum_hard_failure_rate: Probability::new(0.1).unwrap(),
        minimum_empirical_coverage: Probability::new(0.9).unwrap(),
        maximum_innovation_autocorrelation: Probability::new(0.2).unwrap(),
        maximum_cross_target_numeric_error: maximum,
        maximum_root_time_residual_s: maximum,
        maximum_quadrature_error: maximum,
        maximum_reanchor_error_m: maximum,
        minimum_fuzz_cases: 10,
        minimum_jacobian_cases: 10,
        minimum_monte_carlo_trials: 10,
        minimum_adversarial_root_cases: 10,
        digest: ContentDigestV1::from_bytes([7; 32]),
    };
    let configuration_digest = ContentDigestV1::from_bytes([8; 32]);
    let passing = QualificationReportV1 {
        specification_id: specification.id,
        specification_digest: specification.digest,
        configuration_digest,
        corpus_digest: ContentDigestV1::from_bytes([9; 32]),
        target_digest: ContentDigestV1::from_bytes([10; 32]),
        report_digest: ContentDigestV1::from_bytes([11; 32]),
        session_count: 2,
        total_duration: DurationNs::from_ns(10),
        position_rmse_m: maximum,
        velocity_rmse_mps: maximum,
        attitude_rmse_rad: maximum,
        event_time_error_s: maximum,
        hard_failure_rate: Probability::new(0.1).unwrap(),
        empirical_coverage: Probability::new(0.9).unwrap(),
        innovation_autocorrelation: Probability::new(0.2).unwrap(),
        cross_target_numeric_error: maximum,
        root_time_residual_s: maximum,
        quadrature_error: maximum,
        reanchor_error_m: maximum,
        fuzz_cases: 10,
        jacobian_cases: 10,
        monte_carlo_trials: 10,
        adversarial_root_cases: 10,
        live_root_enclosure: None,
    };
    let status = QualificationStatus::Qualified {
        specification: &specification,
        report: &passing,
    };
    assert!(
        status
            .validate_for_configuration(configuration_digest)
            .is_ok()
    );
    assert_eq!(
        status.validate_for_configuration(ContentDigestV1::from_bytes([12; 32])),
        Err(ValidationError::IncompatibleDefinition)
    );

    let failing = QualificationReportV1 {
        session_count: 1,
        ..passing
    };
    assert_eq!(
        QualificationStatus::Qualified {
            specification: &specification,
            report: &failing,
        }
        .validate_for_configuration(configuration_digest),
        Err(ValidationError::IncompatibleDefinition)
    );

    let numeric_profile = NumericProfileSpec {
        revision: 1,
        scalar_policy: ScalarPolicy::EmbeddedMixedF32F64,
        fma_policy: FmaPolicy::Disabled,
        minimum_rust_version: (1, 87, 0),
        fpmath_source_digest: ContentDigestV1::from_bytes([13; 32]),
        toolchain_digest: ContentDigestV1::from_bytes([14; 32]),
        digest: ContentDigestV1::from_bytes([15; 32]),
    };
    assert_eq!(status.live_root_enclosure(numeric_profile), None);

    let attestation = LiveRootEnclosureQualificationV1 {
        backend_id: NATIVE_F64_TAYLOR_ROOT_BACKEND_ID,
        backend_revision: NATIVE_F64_TAYLOR_ROOT_BACKEND_REVISION,
        numeric_profile_digest: numeric_profile.digest,
        target_digest: passing.target_digest,
        toolchain_digest: numeric_profile.toolchain_digest,
        input_envelope_digest: ContentDigestV1::from_bytes([16; 32]),
        mpfr_oracle_corpus_digest: ContentDigestV1::from_bytes([17; 32]),
        independent_interval_oracle_digest: ContentDigestV1::from_bytes([18; 32]),
        target_bit_fixture_digest: ContentDigestV1::from_bytes([19; 32]),
        oracle_case_count: 10,
        oracle_escape_count: 0,
        maximum_oracle_exclusion_error: NonNegativeF64::new(0.0).unwrap(),
        maximum_root_evaluations_per_scalar: 256,
        maximum_operations_per_scalar: 10_000,
        linked_code_size_bytes: 4_096,
    };
    let attested_report = QualificationReportV1 {
        live_root_enclosure: Some(attestation),
        ..passing
    };
    let attested_status = QualificationStatus::Qualified {
        specification: &specification,
        report: &attested_report,
    };
    assert_eq!(
        attested_status.live_root_enclosure(numeric_profile),
        Some(attestation)
    );
    assert_eq!(
        attested_status.validate_numeric_attestations(numeric_profile),
        Ok(attested_status)
    );

    let mismatched_report = QualificationReportV1 {
        live_root_enclosure: Some(LiveRootEnclosureQualificationV1 {
            toolchain_digest: ContentDigestV1::from_bytes([20; 32]),
            ..attestation
        }),
        ..passing
    };
    assert_eq!(
        QualificationStatus::Qualified {
            specification: &specification,
            report: &mismatched_report,
        }
        .validate_numeric_attestations(numeric_profile),
        Err(ValidationError::IncompatibleDefinition)
    );
}

#[test]
fn installation_requires_zero_origin_imu_and_an_antenna_point() {
    let covariance = MeasurementUncertainty::Modeled(UncertaintyModelId::new(1));
    let points = [ReferencePoint::new(
        ReferencePointId::new(1),
        ReferencePointKind::ImuSensingCenter,
        BodyLeverArm::from_body_vector(BodyVector::new(0.1, 0.0, 0.0).unwrap()),
        SharedParameterId::new(1),
        covariance,
    )];
    let installation = Installation {
        imu_sensor_frame: FrameId::new(9),
        body_from_imu: RotationParameter {
            parameter_id: SharedParameterId::new(1),
            mean: SensorToBodyRotation::from_quaternion(crate::math::UnitQuaternion::IDENTITY),
            uncertainty: covariance,
        },
        imu_to_gnss_antenna: LeverArmParameter {
            parameter_id: SharedParameterId::new(2),
            mean: BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
            uncertainty: covariance,
        },
        reference_points: &points,
        attachment: AttachmentModel::RigidBody,
        dynamics_profile: DynamicsProfileId::new(1),
        calibration_revision: CalibrationRevision::new(1),
        digest: ContentDigestV1::from_bytes([1; 32]),
    };
    assert_eq!(
        installation.validate(),
        Err(ValidationError::InvalidReferencePoint)
    );
}

#[test]
fn device_trajectory_attachment_only_permits_package_reference_points() {
    let attachment = AttachmentModel::DeviceTrajectoryOnly;
    assert!(attachment.permits_reference_point(ReferencePointKind::ImuSensingCenter));
    assert!(attachment.permits_reference_point(ReferencePointKind::GnssAntennaPhaseCenter));
    assert!(attachment.permits_reference_point(ReferencePointKind::InstrumentPackage));
    assert!(!attachment.permits_reference_point(ReferencePointKind::RigidBodyPoint));
    assert!(!attachment.permits_body_axis_quantities());

    let rigid = AttachmentModel::RigidBody;
    assert!(rigid.permits_reference_point(ReferencePointKind::RigidBodyPoint));
    assert!(rigid.permits_body_axis_quantities());
}

#[test]
fn installation_rejects_placeholder_identities() {
    let uncertainty = MeasurementUncertainty::Modeled(UncertaintyModelId::new(1));
    let antenna_lever = BodyLeverArm::new(0.2, 0.0, 0.0).unwrap();
    let points = [
        ReferencePoint::new(
            ReferencePointId::new(1),
            ReferencePointKind::ImuSensingCenter,
            BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
            SharedParameterId::new(3),
            uncertainty,
        ),
        ReferencePoint::new(
            ReferencePointId::new(2),
            ReferencePointKind::GnssAntennaPhaseCenter,
            antenna_lever,
            SharedParameterId::new(2),
            uncertainty,
        ),
    ];
    let valid = Installation {
        imu_sensor_frame: FrameId::new(9),
        body_from_imu: RotationParameter {
            parameter_id: SharedParameterId::new(1),
            mean: SensorToBodyRotation::from_quaternion(crate::math::UnitQuaternion::IDENTITY),
            uncertainty,
        },
        imu_to_gnss_antenna: LeverArmParameter {
            parameter_id: SharedParameterId::new(2),
            mean: antenna_lever,
            uncertainty,
        },
        reference_points: &points,
        attachment: AttachmentModel::RigidBody,
        dynamics_profile: DynamicsProfileId::new(1),
        calibration_revision: CalibrationRevision::new(1),
        digest: ContentDigestV1::from_bytes([1; 32]),
    };
    assert_eq!(valid.validate(), Ok(valid));
    for invalid in [
        Installation {
            imu_sensor_frame: FrameId::new(0),
            ..valid
        },
        Installation {
            dynamics_profile: DynamicsProfileId::new(0),
            ..valid
        },
        Installation {
            calibration_revision: CalibrationRevision::new(0),
            ..valid
        },
        Installation {
            digest: ContentDigestV1::from_bytes([0; 32]),
            ..valid
        },
    ] {
        assert_eq!(
            invalid.validate(),
            Err(ValidationError::IncompatibleDefinition)
        );
    }

    let invalid_points = [
        ReferencePoint::new(
            ReferencePointId::new(0),
            ReferencePointKind::ImuSensingCenter,
            BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
            SharedParameterId::new(3),
            uncertainty,
        ),
        points[1],
    ];
    assert_eq!(
        Installation {
            reference_points: &invalid_points,
            ..valid
        }
        .validate(),
        Err(ValidationError::InvalidReferencePoint)
    );
}

#[test]
fn shared_and_calibration_definitions_reject_placeholder_identity() {
    let validity = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(1_000_000)).unwrap();
    let definition = SharedParameterDefinition {
        id: SharedParameterId::new(1),
        kind: SharedParameterKind::DelayNs,
        mean: SharedParameterMean::Scalar(FiniteF64::new(0.0).unwrap()),
        validity,
    };
    let covariance = SharedParameterCovariance::new(1, &[1.0]).unwrap();
    let valid_set = SharedParameterSet {
        definitions: core::slice::from_ref(&definition),
        covariance,
        treatment: SharedUncertaintyTreatment::SchmidtConsider,
    };
    assert_eq!(valid_set.validate(), Ok(valid_set));

    let zero_definition = SharedParameterDefinition {
        id: SharedParameterId::new(0),
        ..definition
    };
    assert_eq!(
        SharedParameterSet {
            definitions: core::slice::from_ref(&zero_definition),
            ..valid_set
        }
        .validate(),
        Err(ValidationError::IncompatibleDefinition)
    );
    assert_eq!(
        SharedParameterSet {
            treatment: SharedUncertaintyTreatment::QualifiedSequenceBound {
                qualification_digest: ContentDigestV1::from_bytes([0; 32]),
            },
            ..valid_set
        }
        .validate(),
        Err(ValidationError::IncompatibleDefinition)
    );

    let valid_bundle = CalibrationBundle {
        revision: CalibrationRevision::new(1),
        input_profile: InputProfileId::new(1),
        shared_parameters: valid_set,
        digest: ContentDigestV1::from_bytes([2; 32]),
    };
    assert_eq!(valid_bundle.validate(), Ok(valid_bundle));
    for invalid in [
        CalibrationBundle {
            revision: CalibrationRevision::new(0),
            ..valid_bundle
        },
        CalibrationBundle {
            input_profile: InputProfileId::new(0),
            ..valid_bundle
        },
        CalibrationBundle {
            digest: ContentDigestV1::from_bytes([0; 32]),
            ..valid_bundle
        },
    ] {
        assert_eq!(
            invalid.validate(),
            Err(ValidationError::IncompatibleDefinition)
        );
    }
}

#[test]
fn live_metric_capacity_is_checked_against_firmware_resource_limit() {
    let mut plan = MetricPlan::new(1);
    plan.push(MetricDefinition::Distance(DistancePlan {
        definition: MetricDefinitionId::new(1),
        quantity: DistanceQuantity::HorizontalPath,
        reference_point: ReferencePointId::new(1),
        absolute_tolerance_m: 0.001,
        relative_tolerance: 1.0e-6,
    }))
    .unwrap();
    let live = plan.compile_live(LiveMetricLimits::default()).unwrap();
    assert_eq!(live.plan().definitions().len(), 1);
}

#[test]
fn coordinate_frame_helpers_are_constructible_without_allocation() {
    let epoch = CoordinateEpoch::from_decimal_year(2026.0).unwrap();
    let frame = TerrestrialFrame::new(
        FrameId::new(1),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        epoch,
        ReferenceEllipsoid::WGS84,
    );
    let operation = CoordinateOperation::new(
        CoordinateOperationId::new(1),
        frame.id(),
        frame.id(),
        crate::frame::CoordinateOperationKind::Identity,
        ContentDigestV1::from_bytes([1; 32]),
        None,
        Some(NonNegativeF64::new(0.0).unwrap()),
        false,
    )
    .unwrap();
    assert!(operation.supports_surveyed_accuracy(0.0));
}
