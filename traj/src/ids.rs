//! Stable semantic identifiers used across live, replay, and refined runs.

macro_rules! id_type {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
        #[repr(transparent)]
        pub struct $name(u32);

        impl $name {
            /// Creates an identifier from its stable numeric representation.
            #[must_use]
            pub const fn new(value: u32) -> Self {
                Self(value)
            }

            /// Returns the stable numeric representation.
            #[must_use]
            pub const fn get(self) -> u32 {
                self.0
            }
        }
    };
}

id_type!(SourceId, "A sensor or semantic evidence source.");
id_type!(ClockSegmentId, "A contiguous fitted clock-model segment.");
id_type!(ClockModelId, "A versioned clock model.");
id_type!(
    UncertaintyModelId,
    "A versioned measurement uncertainty model."
);
id_type!(
    SharedParameterId,
    "A correlated installation or calibration parameter."
);
id_type!(FrameId, "A terrestrial or sensor frame definition.");
id_type!(
    CoordinateOperationId,
    "A recorded coordinate transformation."
);
id_type!(
    ReferencePointId,
    "A named point rigidly attached to the device body."
);
id_type!(
    DynamicsProfileId,
    "A qualified motion and estimator dynamics profile."
);
id_type!(
    InputProfileId,
    "An immutable prepared-input rate and uncertainty contract."
);
id_type!(
    CalibrationRevision,
    "An immutable calibration-bundle revision."
);
id_type!(
    MetricDefinitionId,
    "A metric definition inside a metric plan."
);
id_type!(GateId, "A surveyed finite oriented gate.");
id_type!(TargetId, "A speed or distance target.");
id_type!(ResultRevisionId, "An immutable processed-result revision.");
id_type!(
    TrajectoryRevision,
    "A trajectory mutation revision within a run."
);
id_type!(
    BackendVersionId,
    "A concrete engine/backend implementation version."
);
id_type!(QualificationSpecId, "An immutable qualification contract.");
id_type!(
    NormalizationRevision,
    "A captured or recomputed normalization revision."
);
id_type!(EphemerisIssue, "A GNSS ephemeris issue/version identifier.");

/// Stable identity and within-source ordering for one observation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(C)]
pub struct ObservationId {
    /// Evidence source.
    pub source: SourceId,
    /// Strictly increasing sequence within that source.
    pub sequence: u64,
}

impl ObservationId {
    /// Creates an observation identifier.
    #[must_use]
    pub const fn new(source: SourceId, sequence: u64) -> Self {
        Self { source, sequence }
    }
}

/// Opaque session identity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(transparent)]
pub struct SessionId([u8; 16]);

impl SessionId {
    /// Creates a session identity from canonical bytes.
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 16]) -> Self {
        Self(bytes)
    }

    /// Returns canonical bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 16] {
        &self.0
    }

    /// Returns whether this is the reserved all-zero placeholder.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        let mut index = 0;
        while index < self.0.len() {
            if self.0[index] != 0 {
                return false;
            }
            index += 1;
        }
        true
    }
}

/// Algorithm-tagged SHA-256 digest of canonical semantic content.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(C)]
pub struct ContentDigestV1([u8; 32]);

impl ContentDigestV1 {
    /// Creates a digest from canonical bytes.
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Returns canonical digest bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Returns whether this is the all-zero placeholder rather than a
    /// content-derived identity.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        let mut index = 0;
        while index < self.0.len() {
            if self.0[index] != 0 {
                return false;
            }
            index += 1;
        }
        true
    }
}

/// Stable identity for a live result across upsert/finalize/withdraw mutations.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(C)]
pub struct LiveResultId {
    run_namespace: u64,
    allocation: u64,
}

impl LiveResultId {
    /// Creates an opaque identity from its replay-stable parts.
    #[must_use]
    pub const fn new(run_namespace: u64, allocation: u64) -> Self {
        Self {
            run_namespace,
            allocation,
        }
    }

    /// Returns the deterministic run namespace.
    #[must_use]
    pub const fn run_namespace(self) -> u64 {
        self.run_namespace
    }

    /// Returns the monotonically allocated counter.
    #[must_use]
    pub const fn allocation(self) -> u64 {
        self.allocation
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifiers_round_trip_without_hidden_normalization() {
        let source = SourceId::new(u32::MAX);
        assert_eq!(source.get(), u32::MAX);
        let session = SessionId::from_bytes([0xa5; 16]);
        assert_eq!(session.as_bytes(), &[0xa5; 16]);
    }

    #[test]
    fn zero_digest_is_never_a_content_identity() {
        assert!(ContentDigestV1::from_bytes([0; 32]).is_zero());
        let mut bytes = [0; 32];
        bytes[31] = 1;
        assert!(!ContentDigestV1::from_bytes(bytes).is_zero());
    }

    #[test]
    fn zero_session_is_reserved_for_missing_identity() {
        assert!(SessionId::from_bytes([0; 16]).is_zero());
        assert!(!SessionId::from_bytes([1; 16]).is_zero());
    }
}
