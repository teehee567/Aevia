//! Profile fixture.

use super::*;

pub(super) fn digest(byte: u8) -> ContentDigestV1 {
    ContentDigestV1::from_bytes([byte; 32])
}

pub(super) fn nonnegative(value: f64) -> NonNegativeF64 {
    NonNegativeF64::new(value).unwrap()
}

pub(super) fn finite(value: f64) -> FiniteF64 {
    FiniteF64::new(value).unwrap()
}

pub(super) fn variance(value: f64) -> Variance {
    Variance::new(value).unwrap()
}

pub(super) fn covariance(value: f64) -> Covariance3 {
    Covariance3::diagonal(value, value, value).unwrap()
}

pub(super) fn initial_clock_prior() -> InitialClockConsiderPrior {
    InitialClockConsiderPrior {
        model: ClockModelId::new(1),
        segment: ClockSegmentId::new(1),
        reference_time: SessionTime::ZERO,
        offset_variance_s2: variance(1.0),
        drift_variance: variance(1.0),
        offset_drift_covariance_s: finite(0.0),
        cross_covariance_with_shared: ClockSharedCrossCovariance::independent(6).unwrap(),
    }
}

pub(super) fn qualified_engine(validity: TimeSpan) -> EngineConfig<'static> {
    let boresight = SharedParameterId::new(1);
    let antenna_lever = SharedParameterId::new(2);
    let shared_definitions = std::boxed::Box::leak(
        vec![
            SharedParameterDefinition {
                id: boresight,
                kind: SharedParameterKind::BoresightRadians,
                mean: SharedParameterMean::Vector3(Vector3::ZERO),
                validity,
            },
            SharedParameterDefinition {
                id: antenna_lever,
                kind: SharedParameterKind::LeverArmMetres,
                mean: SharedParameterMean::Vector3(Vector3::ZERO),
                validity,
            },
        ]
        .into_boxed_slice(),
    );
    let mut shared_upper = Vec::with_capacity(21);
    for row in 0..6 {
        for column in row..6 {
            shared_upper.push(if row == column { 1.0 } else { 0.0 });
        }
    }
    let shared_upper = std::boxed::Box::leak(shared_upper.into_boxed_slice());
    let shared_parameters = SharedParameterSet {
        definitions: shared_definitions,
        covariance: SharedParameterCovariance::new(6, shared_upper).unwrap(),
        treatment: SharedUncertaintyTreatment::SchmidtConsider,
    };
    let supplied_covariance = MeasurementUncertainty::Provided(covariance(1.0));
    let antenna_offset = BodyLeverArm::new(0.2, 0.0, 0.0).unwrap();
    let reference_points = std::boxed::Box::leak(
        vec![
            ReferencePoint::new(
                ReferencePointId::new(1),
                ReferencePointKind::ImuSensingCenter,
                BodyLeverArm::new(0.0, 0.0, 0.0).unwrap(),
                boresight,
                supplied_covariance,
            ),
            ReferencePoint::new(
                ReferencePointId::new(2),
                ReferencePointKind::GnssAntennaPhaseCenter,
                antenna_offset,
                antenna_lever,
                supplied_covariance,
            ),
        ]
        .into_boxed_slice(),
    );
    let dynamics_id = DynamicsProfileId::new(1);
    let calibration_revision = CalibrationRevision::new(1);
    let installation = Installation {
        imu_sensor_frame: FrameId::new(20),
        body_from_imu: RotationParameter {
            parameter_id: boresight,
            mean: SensorToBodyRotation::from_quaternion(UnitQuaternion::IDENTITY),
            uncertainty: supplied_covariance,
        },
        imu_to_gnss_antenna: LeverArmParameter {
            parameter_id: antenna_lever,
            mean: antenna_offset,
            uncertainty: supplied_covariance,
        },
        reference_points,
        attachment: AttachmentModel::RigidBody,
        dynamics_profile: dynamics_id,
        calibration_revision,
        digest: digest(2),
    };
    let input_profile_id = InputProfileId::new(1);
    let calibration = CalibrationBundle {
        revision: calibration_revision,
        input_profile: input_profile_id,
        shared_parameters,
        digest: digest(3),
    };
    let input_profile = InputProfileSpec {
        id: input_profile_id,
        imu_rate_hz_range: (nonnegative(200.0), nonnegative(400.0)),
        maximum_imu_samples_per_second: 400,
        maximum_position_updates_per_second: 20,
        maximum_velocity_updates_per_second: 20,
        maximum_raw_signals_per_epoch: 64,
        digest: digest(7),
    };
    let process_covariance = covariance(1.0e-4);
    let dynamics_profile = DynamicsProfileSpec {
        id: dynamics_id,
        attachment: AttachmentModel::RigidBody,
        process_noise: ProcessNoiseSpec {
            accelerometer: process_covariance,
            gyroscope: process_covariance,
            accelerometer_bias: process_covariance,
            gyroscope_bias: process_covariance,
        },
        stationary: StationaryClassifierSpec {
            probability_stays_stationary: Probability::new(0.9).unwrap(),
            probability_motion_to_stationary: Probability::new(0.1).unwrap(),
            enter_probability: Probability::new(0.8).unwrap(),
            exit_probability: Probability::new(0.2).unwrap(),
            minimum_window_samples: 1,
            zupt_covariance: covariance(1.0),
            zupt_nis_threshold: nonnegative(10.0),
        },
        heading: HeadingObservabilitySpec {
            minimum_yaw_information: nonnegative(1.0),
            maximum_yaw_variance_rad2: nonnegative(1.0),
            minimum_course_snr: nonnegative(1.0),
            maximum_course_variance_rad2: nonnegative(1.0),
            dwell: DurationNs::from_ns(1),
        },
        gnss: GnssFusionSpec {
            position_covariance_floor: covariance(1.0e-4),
            velocity_covariance_floor: covariance(1.0e-4),
            nis_rejection_threshold: nonnegative(10.0),
            robust_weight_threshold: nonnegative(5.0),
            maximum_covariance_inflation: nonnegative(2.0),
            maximum_correction_age: DurationNs::from_ns(1_000_000_000),
            correlation: GnssCorrelationPolicy::FixedDecimation {
                accept_every: core::num::NonZeroU16::MIN,
            },
        },
        permits_non_holonomic_constraint: false,
        digest: digest(8),
    };
    let positive = nonnegative(1.0);
    let navigation_profile = crate::config::NavigationProfileSpec {
        revision: 1,
        navigation_cadence_hz: 200,
        fusion_delay: DurationNs::from_ns(10_000_000),
        smoothing_lag: DurationNs::ZERO,
        history_guard: DurationNs::from_ns(10_000_000),
        maximum_bridgeable_imu_gap: DurationNs::from_ns(5_000_000),
        reanchor_distance_m: nonnegative(100.0),
        reanchor_hysteresis_m: nonnegative(10.0),
        consider_dimension: 8,
        predictor_time_constant: DurationNs::from_ns(100_000_000),
        predictor_reset_position_m: nonnegative(10.0),
        covariance_repair: CovarianceRepairPolicy {
            maximum_attempts: 2,
            maximum_total_regularization: nonnegative(1.0e-4),
        },
        embedded_tuning: EmbeddedLiveTuning {
            gravity_magnitude_mps2: nonnegative(9.806_65),
            gravity_vertical_gradient_s2: finite(0.0),
            stationary_gyro_score_variance: positive,
            stationary_force_norm_score_variance: positive,
            minimum_coarse_alignment_samples: 1,
            minimum_gyrocompass_samples: 1,
            gyrocompassing_qualified: false,
            minimum_earth_rate_cross_gravity: nonnegative(1.0e-8),
            maximum_static_force_variance: positive,
            maximum_static_gyro_variance: positive,
            roll_pitch_variance_rad2: positive,
            unobservable_yaw_variance_rad2: positive,
            accelerometer_bias_prior_mps2: [finite(0.0); 3],
            gyroscope_bias_prior_rad_s: [finite(0.0); 3],
            accelerometer_bias_variance: [positive; 3],
            gyroscope_bias_variance: [positive; 3],
            gap_jerk_one_sigma_mps3: [positive; 3],
            gap_angular_acceleration_one_sigma_rad_s2: [positive; 3],
            bias_correction_validity_norm: positive,
            predictor_reset_velocity_mps: positive,
            predictor_reset_attitude_rad: positive,
            covariance_state_scales: [positive; 15],
            covariance_minimum_variances: [nonnegative(1.0e-8); 15],
            covariance_repair_initial: nonnegative(1.0e-8),
            covariance_repair_growth: nonnegative(10.0),
        },
        digest: digest(9),
    };
    let numeric_profile = NumericProfileSpec {
        revision: 1,
        scalar_policy: ScalarPolicy::EmbeddedMixedF32F64,
        fma_policy: FmaPolicy::Disabled,
        minimum_rust_version: (1, 87, 0),
        fpmath_source_digest: digest(10),
        toolchain_digest: digest(11),
        digest: digest(12),
    };
    let processing_frame = TerrestrialFrame::new(
        FrameId::new(10),
        TerrestrialRealization::Wgs84(Wgs84Realization::G2296),
        CoordinateEpoch::from_decimal_year(2026.0).unwrap(),
        ReferenceEllipsoid::WGS84,
    );
    let coordinate_operations = std::boxed::Box::leak(
        vec![
            CoordinateOperation::new(
                CoordinateOperationId::new(1),
                processing_frame.id(),
                processing_frame.id(),
                CoordinateOperationKind::Identity,
                digest(13),
                None,
                Some(nonnegative(0.0)),
                false,
            )
            .unwrap(),
        ]
        .into_boxed_slice(),
    );
    let configuration_digest = digest(1);
    let maximum = nonnegative(1.0);
    let qualification_spec = std::boxed::Box::leak(std::boxed::Box::new(QualificationSpecV1 {
        id: QualificationSpecId::new(1),
        minimum_session_count: 1,
        minimum_total_duration: DurationNs::from_ns(1),
        maximum_position_rmse_m: maximum,
        maximum_velocity_rmse_mps: maximum,
        maximum_attitude_rmse_rad: maximum,
        maximum_event_time_error_s: maximum,
        maximum_hard_failure_rate: Probability::new(1.0).unwrap(),
        minimum_empirical_coverage: Probability::new(0.5).unwrap(),
        maximum_innovation_autocorrelation: Probability::new(1.0).unwrap(),
        maximum_cross_target_numeric_error: maximum,
        maximum_root_time_residual_s: maximum,
        maximum_quadrature_error: maximum,
        maximum_reanchor_error_m: maximum,
        minimum_fuzz_cases: 1,
        minimum_jacobian_cases: 1,
        minimum_monte_carlo_trials: 1,
        minimum_adversarial_root_cases: 1,
        digest: digest(14),
    }));
    let qualification_report = std::boxed::Box::leak(std::boxed::Box::new(QualificationReportV1 {
        specification_id: qualification_spec.id,
        specification_digest: qualification_spec.digest,
        configuration_digest,
        corpus_digest: digest(15),
        target_digest: digest(16),
        report_digest: digest(17),
        session_count: 1,
        total_duration: DurationNs::from_ns(1),
        position_rmse_m: maximum,
        velocity_rmse_mps: maximum,
        attitude_rmse_rad: maximum,
        event_time_error_s: maximum,
        hard_failure_rate: Probability::new(1.0).unwrap(),
        empirical_coverage: Probability::new(0.5).unwrap(),
        innovation_autocorrelation: Probability::new(1.0).unwrap(),
        cross_target_numeric_error: maximum,
        root_time_residual_s: maximum,
        quadrature_error: maximum,
        reanchor_error_m: maximum,
        fuzz_cases: 1,
        jacobian_cases: 1,
        monte_carlo_trials: 1,
        adversarial_root_cases: 1,
        live_root_enclosure: None,
    }));
    EngineConfig {
        installation,
        calibration,
        input_profile,
        dynamics_profile,
        navigation_profile,
        numeric_profile,
        processing_frame,
        coordinate_operations,
        uncertainty_models: &[],
        qualification: QualificationStatus::Qualified {
            specification: qualification_spec,
            report: qualification_report,
        },
        digest: configuration_digest,
    }
}

pub(super) const REPLAY_END_NS: i64 = 30_000_000;

pub(super) fn processing_spec() -> ProcessingSpec<'static> {
    let span = TimeSpan::new(SessionTime::ZERO, SessionTime::from_ns(REPLAY_END_NS)).unwrap();
    processing_spec_for_span(span)
}

pub(super) fn processing_spec_for_span(span: TimeSpan) -> ProcessingSpec<'static> {
    let engine = qualified_engine(span);
    let selections = std::boxed::Box::leak(
        [
            EvidenceClass::Imu,
            EvidenceClass::GnssSolution,
            EvidenceClass::Timing,
            EvidenceClass::Control,
        ]
        .into_iter()
        .enumerate()
        .map(|(index, class)| EvidenceSelection {
            source: SourceId::new(index as u32 + 1),
            class,
            span,
            lineage: EvidenceLineageKind::Captured,
            normalization_revision: Some(NormalizationRevision::new(1)),
            digest: digest(30 + index as u8),
            usage: EvidenceUse::Fusion,
        })
        .collect::<Vec<_>>()
        .into_boxed_slice(),
    );
    let mut metrics = MetricPlan::new(7);
    metrics
        .push(MetricDefinition::Distance(DistancePlan {
            definition: MetricDefinitionId::new(1),
            quantity: DistanceQuantity::Spatial3d,
            reference_point: ReferencePointId::new(1),
            absolute_tolerance_m: 1.0e-6,
            relative_tolerance: 1.0e-9,
        }))
        .unwrap();
    ProcessingSpec {
        engine,
        span,
        policy: crate::config::ProcessingPolicy::require(ProcessingLevel::CapturedReplay),
        evidence_lineage: EvidenceLineage::new(selections).unwrap(),
        calibration_policy: CalibrationPolicy::Fixed,
        metrics,
        result: ProcessingResultSpec {
            result_revision: ResultRevisionId::new(1),
            trajectory_revision: TrajectoryRevision::new(1),
            uncertainty_digest: digest(40),
            metric_plan_digest: digest(41),
            parents: &[],
            external_inputs: &[],
        },
    }
}

pub(super) fn with_best_qualified(
    mut spec: ProcessingSpec<'static>,
    levels: &[ProcessingLevel],
) -> ProcessingSpec<'static> {
    spec.policy = crate::config::ProcessingPolicy::best_qualified(levels).unwrap();
    spec
}
