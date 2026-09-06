//! Numeric profiles and qualification of the root enclosure backend.

use crate::error::ValidationError;
use crate::ids::ContentDigestV1;
use crate::math::NonNegativeF64;

/// Scalar precision used by an executable numeric profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScalarPolicy {
    /// Validated embedded mixed precision with an `f64` ECEF anchor.
    EmbeddedMixedF32F64,
    /// Host-wide double precision.
    F64,
}

/// Contract controlling fused multiply-add differences across replay targets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FmaPolicy {
    /// Compiler must not implicitly contract ordinary multiply/add
    /// expressions. Explicit `mul_add` calls remain fused algorithm steps.
    Disabled,
    /// Contraction is permitted and captured replay must use it consistently.
    Permitted,
    /// Profile requires fused operations on all qualified targets.
    Required,
}

/// Immutable executable numerical behavior.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NumericProfileSpec {
    /// Profile revision.
    pub revision: u32,
    /// Embedded or host scalar policy.
    pub scalar_policy: ScalarPolicy,
    /// FMA contraction policy.
    pub fma_policy: FmaPolicy,
    /// Minimum supported Rust compiler version `(major, minor, patch)`.
    pub minimum_rust_version: (u16, u16, u16),
    /// Reviewed `fpmath` source digest used by metric enclosures.
    pub fpmath_source_digest: ContentDigestV1,
    /// Compiler/flags/math-backend digest.
    pub toolchain_digest: ContentDigestV1,
    /// Canonical complete profile digest.
    pub digest: ContentDigestV1,
}

impl NumericProfileSpec {
    /// Validates the plan's minimum compiler contract.
    pub fn validate(self) -> Result<Self, ValidationError> {
        if self.revision == 0
            || self.fpmath_source_digest.is_zero()
            || self.toolchain_digest.is_zero()
            || self.digest.is_zero()
            || self.minimum_rust_version.0 < 1
            || (self.minimum_rust_version.0 == 1 && self.minimum_rust_version.1 < 86)
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}

/// Legacy development-backend identity. Reports bearing it cannot qualify the
/// production expression enclosure backend.
pub const NATIVE_F64_TAYLOR_ROOT_BACKEND_ID: ContentDigestV1 = ContentDigestV1::from_bytes([
    0xcd, 0x37, 0xed, 0xcf, 0x68, 0x3c, 0x1a, 0x45, 0xbc, 0x03, 0x79, 0xa6, 0xc9, 0x03, 0x52, 0x5d,
    0xd0, 0x2a, 0x50, 0x48, 0xd3, 0x60, 0x67, 0x84, 0x56, 0xbc, 0x9a, 0x45, 0x07, 0xe6, 0x6a, 0xa8,
]);

/// Source-contract revision of [`NATIVE_F64_TAYLOR_ROOT_BACKEND_ID`].
pub const NATIVE_F64_TAYLOR_ROOT_BACKEND_REVISION: u32 = 0;

/// Production outward-interval expression backend: binary64 arithmetic,
/// pinned fpmath elementary functions, rigid-point SO(3) products and Bowring
/// ellipsoid-normal derivatives. This identity does not claim software-float
/// execution or measured target qualification.
///
/// SHA-256 of `aevia-trajectory/EnclosureNativeF64V1/revision-1`.
pub const ENCLOSURE_NATIVE_F64_ROOT_BACKEND_ID: ContentDigestV1 = ContentDigestV1::from_bytes([
    0xe8, 0xf2, 0xcd, 0x87, 0x77, 0x50, 0x6d, 0x28, 0xdf, 0x8f, 0x89, 0x02, 0x02, 0xcd, 0x2b, 0xca,
    0xa0, 0x26, 0xfc, 0xa9, 0x06, 0x74, 0x89, 0x00, 0x5d, 0x32, 0xf2, 0x0a, 0x02, 0x8b, 0xa6, 0xf6,
]);

/// Source-contract revision of [`ENCLOSURE_NATIVE_F64_ROOT_BACKEND_ID`].
pub const ENCLOSURE_NATIVE_F64_ROOT_BACKEND_REVISION: u32 = 1;

/// Measured qualification evidence for the exact live non-polynomial root
/// backend compiled into this crate.
///
/// Absence is meaningful: polynomial origin-point gates and spatial-speed
/// roots remain usable, but live preflight rejects definitions that require a
/// non-polynomial root.  This record is intentionally richer than a feature
/// boolean so a result cannot be transferred to another implementation,
/// numeric profile, target, toolchain, or input envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveRootEnclosureQualificationV1 {
    /// Exact backend algorithm/source contract.
    pub backend_id: ContentDigestV1,
    /// Exact backend source-contract revision.
    pub backend_revision: u32,
    /// Numeric profile whose scalar/FMA policy was exercised.
    pub numeric_profile_digest: ContentDigestV1,
    /// Actual target/module fixture identity.
    pub target_digest: ContentDigestV1,
    /// Compiler, flags, and math-backend identity.
    pub toolchain_digest: ContentDigestV1,
    /// Qualified reachable segment/lever/rate/input envelope.
    pub input_envelope_digest: ContentDigestV1,
    /// High-precision MPFR oracle corpus and generator identity.
    pub mpfr_oracle_corpus_digest: ContentDigestV1,
    /// Independent host interval implementation and corpus identity.
    pub independent_interval_oracle_digest: ContentDigestV1,
    /// Actual-target bit-fixture corpus identity.
    pub target_bit_fixture_digest: ContentDigestV1,
    /// Number of oracle cells/cases exercised.
    pub oracle_case_count: u64,
    /// Any oracle value outside the reported enclosure is a hard failure.
    pub oracle_escape_count: u64,
    /// Largest measured amount by which an oracle escaped an enclosure.
    pub maximum_oracle_exclusion_error: NonNegativeF64,
    /// Largest root-oracle evaluation budget covered by the campaign.
    pub maximum_root_evaluations_per_scalar: u32,
    /// Static operation ceiling for one scalar root request.
    pub maximum_operations_per_scalar: u32,
    /// Measured linked code-size contribution on the bound target.
    pub linked_code_size_bytes: u32,
}

impl LiveRootEnclosureQualificationV1 {
    pub(super) fn validate_against(
        self,
        numeric_profile: NumericProfileSpec,
        report_target_digest: ContentDigestV1,
        minimum_oracle_cases: u32,
    ) -> Result<Self, ValidationError> {
        if self.backend_id != ENCLOSURE_NATIVE_F64_ROOT_BACKEND_ID
            || self.backend_revision != ENCLOSURE_NATIVE_F64_ROOT_BACKEND_REVISION
            || self.numeric_profile_digest != numeric_profile.digest
            || self.target_digest != report_target_digest
            || self.toolchain_digest != numeric_profile.toolchain_digest
            || self.input_envelope_digest.is_zero()
            || self.mpfr_oracle_corpus_digest.is_zero()
            || self.independent_interval_oracle_digest.is_zero()
            || self.target_bit_fixture_digest.is_zero()
            || self.oracle_case_count < u64::from(minimum_oracle_cases)
            || self.oracle_escape_count != 0
            || self.maximum_oracle_exclusion_error.get() != 0.0
            || self.maximum_root_evaluations_per_scalar == 0
            || self.maximum_operations_per_scalar == 0
            || self.linked_code_size_bytes == 0
            || numeric_profile.scalar_policy != ScalarPolicy::EmbeddedMixedF32F64
            || numeric_profile.fma_policy != FmaPolicy::Disabled
        {
            Err(ValidationError::IncompatibleDefinition)
        } else {
            Ok(self)
        }
    }
}
