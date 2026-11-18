//! Exercise the actual device profile/definitions, without MMIO or a receiver.
#[path = "../src/navigation/definitions.rs"]
mod definitions;
#[path = "../src/navigation/initialization.rs"]
mod initialization;
#[path = "../src/navigation/measurements.rs"]
mod measurements;
#[path = "../src/navigation/memory.rs"]
mod memory;
#[path = "../src/navigation/profile/mod.rs"]
#[allow(dead_code)]
mod profile;
#[path = "../src/navigation/settings.rs"]
mod settings;

use aevia_trajectory::{
    PrepareError, TrajectoryEngine,
    config::{
        ClockSharedCrossCovariance, InitialClockConsiderPrior, LiveResourceLimits, LiveSpec,
        QualificationStatus,
    },
    ids::*,
    math::FiniteF64,
    metric::*,
    time::SessionTime,
    uncertainty::Variance,
    workspace::{LiveInternalWorkspace, LivePsramWorkspace},
};

#[test]
fn actual_device_profile_preserves_qualification_mounting_resource_and_metric_gates() {
    // Host debug frames are intentionally unrestricted here. The test checks
    // declared firmware workspace budgets; it does not attest device stack use.
    std::thread::Builder::new().stack_size(16*1024*1024).spawn(|| {
        let mut definitions=MetricPlan::new(1);
        definitions::populate(&mut definitions).expect("actual firmware navigation-only plan");
        assert!(definitions.definitions().is_empty());
        let mut metrics=LiveMetricPlan::placeholder();
        definitions.compile_live_into(LiveMetricLimits::default(),&mut metrics).expect("default targets must compile for live use");

        // This is the only test calling the initializer. Its static backing
        // arrays are written exactly once and stay immutable while borrowed.
        let engine=unsafe{profile::engine()};
        engine.validate().expect("actual firmware profile must be semantically valid");
        assert!(matches!(engine.qualification,QualificationStatus::Unqualified));
        let settings=settings::snapshot();
        assert!(!settings.qualified);
        assert_eq!(engine.installation.body_from_imu.mean.quaternion().components_wxyz(),settings.body_from_imu_wxyz);
        assert_eq!(engine.installation.imu_to_gnss_antenna.mean.components_m(),settings.imu_to_antenna_metres);
        assert!(settings.imu_timing_sigma_ns>0);
        for prior in [engine.installation.body_from_imu.uncertainty,engine.installation.imu_to_gnss_antenna.uncertainty] {
            let covariance=prior.provided().expect("unknown installation must carry its declared prior").to_matrix();
            assert!((0..3).all(|axis|covariance[axis][axis]>0.0));
        }
        assert_eq!(engine.processing_frame,profile::FRAME);
        let spec=LiveSpec{
            session_id:SessionId::from_bytes(*b"AEVIA-DEVICE-001"),engine,metrics:&metrics,
            resources:LiveResourceLimits::V2_MINI_RTS,initial_heading:None,
            initial_clock_prior:InitialClockConsiderPrior{
                model:ClockModelId::new(1),segment:ClockSegmentId::new(1),reference_time:SessionTime::ZERO,
                offset_variance_s2:Variance::new(0.025*0.025).unwrap(),drift_variance:Variance::new(500e-6*500e-6).unwrap(),offset_drift_covariance_s:FiniteF64::new(0.0).unwrap(),
                cross_covariance_with_shared:ClockSharedCrossCovariance::independent(6).unwrap(),
            },
        };
        assert!(matches!(TrajectoryEngine::live(spec.clone()).preflight(),Err(PrepareError::UnqualifiedProfile)));
        let plan=TrajectoryEngine::live(spec.clone()).preflight_development().unwrap_or_else(|error|panic!("actual device development preflight: {error:?}"));
        let req=plan.requirements();let limits=spec.resources;
        assert!(req.internal_sram_bytes()+req.maximum_stack_bytes()<=limits.internal_sram_bytes);
        assert!(req.maximum_stack_bytes()<=limits.stack_bytes);
        assert!(req.psram_bytes()<=limits.psram_bytes);
        assert!(core::mem::size_of::<LiveInternalWorkspace>()<=req.internal_sram_bytes());
        let psram=core::mem::size_of::<LivePsramWorkspace>()+core::mem::size_of::<LiveMetricPlan>()+core::mem::size_of::<MetricPlan>()+3*64;
        assert!(psram<memory::BYTES);
        assert!(plan.required_imu_capacity()>=263);
        assert!(plan.required_measurement_capacity()>=15);
        assert!(plan.required_dense_segment_capacity()>0);
        std::println!("device contract internal={} stack={} psram={} placed_psram={} imu={} measurements={} dense={}",req.internal_sram_bytes(),req.maximum_stack_bytes(),req.psram_bytes(),psram,plan.required_imu_capacity(),plan.required_measurement_capacity(),plan.required_dense_segment_capacity());
        let mut intended_definitions=MetricPlan::new(2);
        definitions::populate_intended_drag(&mut intended_definitions).unwrap();
        let [MetricDefinition::Drag(drag)]=intended_definitions.definitions() else {panic!("one intended drag definition expected");};
        assert_eq!(drag.reference_point,ReferencePointId::new(1));
        assert_eq!(drag.targets(),&[
            DragTarget::Speed{id:TargetId::new(1),quantity:SpeedQuantity::InstantaneousHorizontal,metres_per_second:100.0/3.6,direction:TargetDirection::Ascending},
            DragTarget::quarter_mile_horizontal_path(TargetId::new(2)),
        ]);
        let mut intended_metrics=LiveMetricPlan::placeholder();
        intended_definitions.compile_live_into(LiveMetricLimits::default(),&mut intended_metrics).unwrap();
        let mut with_drag=spec.clone();with_drag.metrics=&intended_metrics;
        assert!(matches!(TrajectoryEngine::live(with_drag).preflight_development(),Err(PrepareError::CapabilityUnavailable)));
        let mut undersized=spec.clone();undersized.resources.psram_bytes=1;
        assert!(matches!(TrajectoryEngine::live(undersized).preflight_development(),Err(PrepareError::InsufficientResources)));
        let mut invalid=spec;invalid.engine.dynamics_profile.permits_non_holonomic_constraint=true;
        assert!(matches!(TrajectoryEngine::live(invalid).preflight_development(),Err(PrepareError::IncompatibleProfile)));
    }).unwrap().join().unwrap();
}
