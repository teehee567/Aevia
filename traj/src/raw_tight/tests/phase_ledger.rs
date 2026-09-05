//! Phase ledger regression tests.

use super::*;

#[test]
fn phase_and_tdcp_double_use_requires_full_joint_covariance() {
    let key = PhaseSampleKey {
        observation: ObservationId::new(SourceId::new(2), 10),
        satellite: satellite(3),
        signal_code: 1,
    };
    let mut ledger = PhaseUseLedger::<4>::new();
    ledger
        .record(
            key,
            PhaseContribution::AmbiguityCarrier {
                full_joint_tdcp_covariance_retained: false,
            },
        )
        .unwrap();
    assert_eq!(
        ledger.record(
            key,
            PhaseContribution::Tdcp {
                factor_id: 1,
                full_joint_carrier_covariance_retained: false,
                shared_middle_epoch_covariance_retained: true,
            }
        ),
        Err(PhaseUseError::CarrierTdcpWithoutFullJointCovariance)
    );

    let mut covariance_aware = PhaseUseLedger::<4>::new();
    covariance_aware
        .record(
            key,
            PhaseContribution::AmbiguityCarrier {
                full_joint_tdcp_covariance_retained: true,
            },
        )
        .unwrap();
    covariance_aware
        .record(
            key,
            PhaseContribution::Tdcp {
                factor_id: 1,
                full_joint_carrier_covariance_retained: true,
                shared_middle_epoch_covariance_retained: true,
            },
        )
        .unwrap();
}

#[test]
fn adjacent_tdcp_factors_require_shared_middle_epoch_covariance() {
    let key = PhaseSampleKey {
        observation: ObservationId::new(SourceId::new(2), 11),
        satellite: satellite(3),
        signal_code: 1,
    };
    let mut ledger = PhaseUseLedger::<4>::new();
    ledger
        .record(
            key,
            PhaseContribution::Tdcp {
                factor_id: 1,
                full_joint_carrier_covariance_retained: false,
                shared_middle_epoch_covariance_retained: false,
            },
        )
        .unwrap();
    assert_eq!(
        ledger.record(
            key,
            PhaseContribution::Tdcp {
                factor_id: 2,
                full_joint_carrier_covariance_retained: false,
                shared_middle_epoch_covariance_retained: true,
            }
        ),
        Err(PhaseUseError::AdjacentTdcpWithoutSharedEpochCovariance)
    );
}
