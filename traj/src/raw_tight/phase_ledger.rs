//! Prevent unsupported carrier-phase and TDCP sample reuse.

use crate::ids::ObservationId;
use crate::observation::SatelliteId;
use heapless::Vec;

/// One carrier-phase sample, before choosing ambiguity or TDCP use.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PhaseSampleKey {
    pub observation: ObservationId,
    pub satellite: SatelliteId,
    pub signal_code: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PhaseContribution {
    AmbiguityCarrier {
        full_joint_tdcp_covariance_retained: bool,
    },
    Tdcp {
        factor_id: u64,
        full_joint_carrier_covariance_retained: bool,
        shared_middle_epoch_covariance_retained: bool,
    },
    DiagnosticOnly,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PhaseSampleUse {
    key: PhaseSampleKey,
    carrier: Option<bool>,
    tdcp_factor_ids: [u64; 2],
    tdcp_joint_carrier: [bool; 2],
    tdcp_shared_epoch: [bool; 2],
    tdcp_count: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PhaseUseError {
    InvalidSampleKey,
    InvalidFactorIdentity,
    CapacityExceeded,
    DuplicateCarrierFactor,
    DuplicateTdcpFactor,
    TooManyTdcpFactors,
    CarrierTdcpWithoutFullJointCovariance,
    AdjacentTdcpWithoutSharedEpochCovariance,
}

/// Fixed-capacity ledger enforcing carrier/TDCP exclusivity and the covariance
/// exception. Adjacent TDCP factors may share their middle epoch only when both
/// declare the induced covariance retained.
pub(crate) struct PhaseUseLedger<const CAPACITY: usize> {
    entries: Vec<PhaseSampleUse, CAPACITY>,
}

impl<const CAPACITY: usize> PhaseUseLedger<CAPACITY> {
    pub(crate) const fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub(crate) fn record(
        &mut self,
        key: PhaseSampleKey,
        contribution: PhaseContribution,
    ) -> Result<(), PhaseUseError> {
        if key.signal_code == 0 || key.satellite.vehicle == 0 {
            return Err(PhaseUseError::InvalidSampleKey);
        }
        match contribution {
            PhaseContribution::DiagnosticOnly => return Ok(()),
            PhaseContribution::Tdcp { factor_id: 0, .. } => {
                return Err(PhaseUseError::InvalidFactorIdentity);
            }
            PhaseContribution::AmbiguityCarrier { .. } | PhaseContribution::Tdcp { .. } => {}
        }
        let index = if let Some(index) = self.entries.iter().position(|entry| entry.key == key) {
            index
        } else {
            self.entries
                .push(PhaseSampleUse {
                    key,
                    carrier: None,
                    tdcp_factor_ids: [0; 2],
                    tdcp_joint_carrier: [false; 2],
                    tdcp_shared_epoch: [false; 2],
                    tdcp_count: 0,
                })
                .map_err(|_| PhaseUseError::CapacityExceeded)?;
            self.entries.len() - 1
        };
        let entry = &mut self.entries[index];
        match contribution {
            PhaseContribution::AmbiguityCarrier {
                full_joint_tdcp_covariance_retained,
            } => {
                if entry.carrier.is_some() {
                    return Err(PhaseUseError::DuplicateCarrierFactor);
                }
                for tdcp_index in 0..usize::from(entry.tdcp_count) {
                    if !full_joint_tdcp_covariance_retained || !entry.tdcp_joint_carrier[tdcp_index]
                    {
                        return Err(PhaseUseError::CarrierTdcpWithoutFullJointCovariance);
                    }
                }
                entry.carrier = Some(full_joint_tdcp_covariance_retained);
            }
            PhaseContribution::Tdcp {
                factor_id,
                full_joint_carrier_covariance_retained,
                shared_middle_epoch_covariance_retained,
            } => {
                let count = usize::from(entry.tdcp_count);
                if entry.tdcp_factor_ids[..count].contains(&factor_id) {
                    return Err(PhaseUseError::DuplicateTdcpFactor);
                }
                if count == 2 {
                    return Err(PhaseUseError::TooManyTdcpFactors);
                }
                if let Some(carrier_joint) = entry.carrier {
                    if !carrier_joint || !full_joint_carrier_covariance_retained {
                        return Err(PhaseUseError::CarrierTdcpWithoutFullJointCovariance);
                    }
                }
                if count == 1
                    && (!entry.tdcp_shared_epoch[0] || !shared_middle_epoch_covariance_retained)
                {
                    return Err(PhaseUseError::AdjacentTdcpWithoutSharedEpochCovariance);
                }
                entry.tdcp_factor_ids[count] = factor_id;
                entry.tdcp_joint_carrier[count] = full_joint_carrier_covariance_retained;
                entry.tdcp_shared_epoch[count] = shared_middle_epoch_covariance_retained;
                entry.tdcp_count += 1;
            }
            PhaseContribution::DiagnosticOnly => return Ok(()),
        }
        Ok(())
    }
}
