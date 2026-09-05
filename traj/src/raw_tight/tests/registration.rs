//! Registration regression tests.

use super::*;

#[test]
fn missing_backend_safety_claim_is_not_registration() {
    let mut backend = registration();
    backend.safety = backend
        .safety
        .without(RawTightSafetyProperty::PvtInitializationOnly);
    assert_eq!(
        backend.validate(),
        Err(RawTightRegistrationError::MissingSafetyInvariant)
    );
}

#[test]
fn registration_cannot_name_an_uncompiled_graph_backend() {
    let mut backend = registration();
    backend.identity.graph_backend = if cfg!(feature = "gtsam-system") {
        RawGraphBackendKind::GtsamVendored
    } else {
        RawGraphBackendKind::GtsamSystem
    };
    backend.identity.gtsam_source_digest = Some(digest(77));
    assert_eq!(
        backend.validate(),
        Err(RawTightRegistrationError::GraphBackendNotCompiled)
    );
}
