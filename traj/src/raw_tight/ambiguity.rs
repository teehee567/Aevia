//! Physical ambiguity arc continuity, reference changes, and conditional fix hold.

use super::{ConditionalAmbiguityFix, ConditionalFixKind, digest_is_zero};
use crate::ids::ContentDigestV1;
use crate::observation::SatelliteId;

/// Stable identity for one physical ambiguity arc.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub(crate) struct AmbiguityArcId(u64);

impl AmbiguityArcId {
    pub(crate) const fn new(value: u64) -> Option<Self> {
        if value == 0 { None } else { Some(Self(value)) }
    }

    pub(crate) const fn get(self) -> u64 {
        self.0
    }
}

/// Satellite/signal ambiguity identity independent of a reference satellite.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AmbiguitySignalKey {
    pub satellite: SatelliteId,
    pub signal_code: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AmbiguityFixState {
    Float,
    Conditional {
        kind: ConditionalFixKind,
        hypothesis_digest: ContentDigestV1,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AmbiguityContinuityEvent {
    Continuous,
    ReceiverCycleSlip,
    LossOfLock,
    ReceiverClockJump,
    ValidatedDiscontinuity,
    ReferenceSatelliteChanged { new_reference: SatelliteId },
    IntegrityFailure,
}

/// Receiver and derived continuity indicator for one signal epoch. Availability
/// and asserted state are tracked separately, so an absent receiver-native bit
/// is not rewritten to `false`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum ContinuityIndicator {
    ReceiverCycleSlip = 0,
    ReceiverLossOfLock = 1,
    HalfCycleDiscontinuity = 2,
    LockTimeReset = 3,
    GeometryFreeDiscontinuity = 4,
    MelbourneWubbenaDiscontinuity = 5,
    DopplerPhaseDiscontinuity = 6,
    InnovationDiscontinuity = 7,
    ReceiverClockJump = 8,
    ValidatedDiscontinuity = 9,
}

/// Compact continuity evidence with explicit indicator availability.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct CycleSlipEvidence {
    available: u16,
    asserted: u16,
}

impl CycleSlipEvidence {
    pub(crate) const NONE: Self = Self {
        available: 0,
        asserted: 0,
    };

    pub(crate) const fn with(self, indicator: ContinuityIndicator, asserted: bool) -> Self {
        let bit = 1_u16 << indicator as u8;
        Self {
            available: self.available | bit,
            asserted: if asserted {
                self.asserted | bit
            } else {
                self.asserted & !bit
            },
        }
    }

    pub(crate) const fn is_available(self, indicator: ContinuityIndicator) -> bool {
        self.available & (1_u16 << indicator as u8) != 0
    }

    pub(crate) const fn is_asserted(self, indicator: ContinuityIndicator) -> bool {
        self.asserted & (1_u16 << indicator as u8) != 0
    }

    pub(crate) const fn continuity_event(self) -> AmbiguityContinuityEvent {
        if self.is_asserted(ContinuityIndicator::ReceiverClockJump) {
            AmbiguityContinuityEvent::ReceiverClockJump
        } else if self.is_asserted(ContinuityIndicator::ReceiverLossOfLock) {
            AmbiguityContinuityEvent::LossOfLock
        } else if self.is_asserted(ContinuityIndicator::ReceiverCycleSlip)
            || self.is_asserted(ContinuityIndicator::HalfCycleDiscontinuity)
            || self.is_asserted(ContinuityIndicator::LockTimeReset)
            || self.is_asserted(ContinuityIndicator::GeometryFreeDiscontinuity)
            || self.is_asserted(ContinuityIndicator::MelbourneWubbenaDiscontinuity)
            || self.is_asserted(ContinuityIndicator::DopplerPhaseDiscontinuity)
            || self.is_asserted(ContinuityIndicator::InnovationDiscontinuity)
        {
            AmbiguityContinuityEvent::ReceiverCycleSlip
        } else if self.is_asserted(ContinuityIndicator::ValidatedDiscontinuity) {
            AmbiguityContinuityEvent::ValidatedDiscontinuity
        } else {
            AmbiguityContinuityEvent::Continuous
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ArcTerminationReason {
    ReceiverCycleSlip,
    LossOfLock,
    ReceiverClockJump,
    ValidatedDiscontinuity,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AmbiguityArcTransition {
    Continued {
        arc: AmbiguityArcId,
    },
    Restarted {
        ended: AmbiguityArcId,
        started: AmbiguityArcId,
        reason: ArcTerminationReason,
    },
    Rereferenced {
        arc: AmbiguityArcId,
        previous_reference: Option<SatelliteId>,
        new_reference: SatelliteId,
        /// This enum variant can be emitted only when the full ambiguity mean
        /// and covariance are to be linearly reparameterized.
        covariance_action: ReferenceCovarianceAction,
    },
    FixResetWithoutEndingArc {
        arc: AmbiguityArcId,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ReferenceCovarianceAction {
    FullLinearReparameterizationRequired,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AmbiguityArcError {
    InvalidInitialArc,
    InvalidReferenceSatellite,
    ArcIdExhausted,
    FixHypothesisMissing,
}

/// Physical-arc state. Reference changes and integrity failures do not invent a
/// slip; only the four documented discontinuity events increment `arc`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AmbiguityArcState {
    pub key: AmbiguitySignalKey,
    arc: AmbiguityArcId,
    reference: Option<SatelliteId>,
    fix: AmbiguityFixState,
}

impl AmbiguityArcState {
    pub(crate) fn new(
        key: AmbiguitySignalKey,
        arc: AmbiguityArcId,
        reference: Option<SatelliteId>,
    ) -> Result<Self, AmbiguityArcError> {
        if key.signal_code == 0 || key.satellite.vehicle == 0 || arc.get() == 0 {
            return Err(AmbiguityArcError::InvalidInitialArc);
        }
        if reference.is_some_and(|satellite| satellite.vehicle == 0) {
            return Err(AmbiguityArcError::InvalidReferenceSatellite);
        }
        Ok(Self {
            key,
            arc,
            reference,
            fix: AmbiguityFixState::Float,
        })
    }

    pub(crate) const fn arc(self) -> AmbiguityArcId {
        self.arc
    }

    pub(crate) const fn fix_state(self) -> AmbiguityFixState {
        self.fix
    }

    pub(crate) fn apply(
        &mut self,
        event: AmbiguityContinuityEvent,
    ) -> Result<AmbiguityArcTransition, AmbiguityArcError> {
        match event {
            AmbiguityContinuityEvent::ReceiverCycleSlip => {
                self.restart(ArcTerminationReason::ReceiverCycleSlip)
            }
            AmbiguityContinuityEvent::LossOfLock => self.restart(ArcTerminationReason::LossOfLock),
            AmbiguityContinuityEvent::ReceiverClockJump => {
                self.restart(ArcTerminationReason::ReceiverClockJump)
            }
            AmbiguityContinuityEvent::ValidatedDiscontinuity => {
                self.restart(ArcTerminationReason::ValidatedDiscontinuity)
            }
            AmbiguityContinuityEvent::ReferenceSatelliteChanged { new_reference } => {
                if new_reference.vehicle == 0 {
                    return Err(AmbiguityArcError::InvalidReferenceSatellite);
                }
                if self.reference == Some(new_reference) {
                    return Ok(AmbiguityArcTransition::Continued { arc: self.arc });
                }
                let previous_reference = self.reference.replace(new_reference);
                Ok(AmbiguityArcTransition::Rereferenced {
                    arc: self.arc,
                    previous_reference,
                    new_reference,
                    covariance_action:
                        ReferenceCovarianceAction::FullLinearReparameterizationRequired,
                })
            }
            AmbiguityContinuityEvent::IntegrityFailure => {
                self.fix = AmbiguityFixState::Float;
                Ok(AmbiguityArcTransition::FixResetWithoutEndingArc { arc: self.arc })
            }
            AmbiguityContinuityEvent::Continuous => {
                Ok(AmbiguityArcTransition::Continued { arc: self.arc })
            }
        }
    }

    fn restart(
        &mut self,
        reason: ArcTerminationReason,
    ) -> Result<AmbiguityArcTransition, AmbiguityArcError> {
        let ended = self.arc;
        let next = self
            .arc
            .get()
            .checked_add(1)
            .and_then(AmbiguityArcId::new)
            .ok_or(AmbiguityArcError::ArcIdExhausted)?;
        self.arc = next;
        self.fix = AmbiguityFixState::Float;
        Ok(AmbiguityArcTransition::Restarted {
            ended,
            started: next,
            reason,
        })
    }

    pub(crate) fn accept_conditional_fix(
        &mut self,
        fix: ConditionalAmbiguityFix,
    ) -> Result<(), AmbiguityArcError> {
        if digest_is_zero(fix.hypothesis_digest) {
            return Err(AmbiguityArcError::FixHypothesisMissing);
        }
        self.fix = AmbiguityFixState::Conditional {
            kind: fix.kind,
            hypothesis_digest: fix.hypothesis_digest,
        };
        Ok(())
    }
}
