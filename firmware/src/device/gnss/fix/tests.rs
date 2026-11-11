use super::*;
use aevia_device_hal::um980::Header;

fn message() -> BestNavXyz {
    BestNavXyz {
        header: Header {
            message_id: 240,
            time_reference: 0,
            time_status: 160,
            week: 2400,
            milliseconds_of_week: 1000,
            output_delay_ms: 10,
        },
        position_status: 0,
        position_type: 16,
        position_ecef_m: [6_378_137.0, 0.0, 0.0],
        position_sigma_m: [1.0; 3],
        velocity_status: 0,
        velocity_type: 8,
        velocity_ecef_mps: [0.1, 0.2, 0.3],
        velocity_sigma_mps: [0.1; 3],
        velocity_latency_s: 0.02,
        differential_age_s: 0.0,
        solution_age_s: 0.0,
        satellites_used: 12,
    }
}

#[test]
fn receiver_fix_is_available_before_pps_or_estimator_alignment_and_expires() {
    let mut state = FixState::default();
    state.navigation(message(), 1_000_000);
    let value = state.at(1_100_000);
    assert!(!value.qualified);
    assert!(value.last_pps.is_none());
    assert!(value.position_valid && value.velocity_valid);
    assert_eq!(value.received_age_us, Some(100_000));
    assert_eq!(value.position_age_us, Some(110_000));
    assert!(value.velocity_age_us.unwrap().abs_diff(130_000) <= 1);
    assert_eq!(value.satellites, 12);
    let stale = state.at(1_500_000);
    assert!(!stale.position_valid && !stale.velocity_valid);
    assert!(stale.position_ecef_m.is_none() && stale.velocity_ecef_mps.is_none());
    assert!(stale.position_sigma_m.is_none() && stale.velocity_sigma_mps.is_none());
    assert_eq!(stale.gps_epoch_ms, value.gps_epoch_ms);
}

#[test]
fn repeated_or_backward_epochs_cannot_refresh_a_stale_fix() {
    let mut state = FixState::default();
    state.navigation(message(), 1_000_000);
    state.navigation(message(), 2_000_000);
    let duplicate = state.at(2_000_000);
    assert_eq!(duplicate.received_age_us, Some(0));
    assert_eq!(duplicate.position_age_us, Some(1_010_000));
    assert!(!duplicate.position_valid && !duplicate.velocity_valid);
    let mut old = message();
    old.header.milliseconds_of_week -= 20;
    state.navigation(old, 2_020_000);
    assert!(!state.at(2_020_000).position_valid);
    assert!(state.at(2_020_000).position_age_us.is_none());
    let mut next = message();
    next.header.milliseconds_of_week += 20;
    state.navigation(next, 2_040_000);
    assert!(state.at(2_040_000).position_valid);
}

#[test]
fn independent_validity_and_receiver_ages_are_preserved_without_nan() {
    let mut state = FixState::default();
    let mut fix = message();
    fix.position_ecef_m = [f64::NAN; 3];
    fix.differential_age_s = f32::NAN;
    fix.solution_age_s = f32::INFINITY;
    state.navigation(fix, 1_000_000);
    let value = state.at(1_000_000);
    assert!(!value.position_valid && value.velocity_valid);
    assert!(value.position_ecef_m.is_none() && value.position_sigma_m.is_none());
    assert!(value.solution_age_s.is_none() && value.differential_age_s.is_none());
    // A valid standalone position can survive invalid Doppler metadata.
    let mut fix = message();
    fix.header.milliseconds_of_week += 20;
    fix.velocity_latency_s = -1.0;
    state.navigation(fix, 1_020_000);
    let value = state.at(1_020_000);
    assert!(value.position_valid && !value.velocity_valid);
    assert!(value.velocity_ecef_mps.is_none() && value.velocity_latency_s.is_none());
}

#[test]
fn unknown_time_future_reads_and_reconfiguration_never_report_a_live_fix() {
    let mut state = FixState::default();
    let mut fix = message();
    fix.header.time_status = 20;
    state.navigation(fix, 1_000_000);
    let value = state.at(1_000_000);
    assert!(value.gps_epoch_ms.is_none());
    assert!(value.position_age_us.is_none() && value.velocity_age_us.is_none());
    assert!(!value.position_valid && !value.velocity_valid);
    assert_eq!(value.time_status, 20);
    state.navigation(message(), 2_000_000);
    assert!(!state.at(1_999_999).position_valid);
    state.invalidate();
    assert!(!state.at(2_000_000).position_valid);
    state.navigation(message(), 2_020_000);
    assert!(state.at(2_020_000).position_valid);
}

#[test]
fn raw_fix_retains_navigation_time_rejections_and_original_field_precision() {
    for (week, milliseconds) in [(0, 0), (u16::MAX, 1000)] {
        let mut fix = message();
        fix.header.week = week;
        fix.header.milliseconds_of_week = milliseconds;
        let mut state = FixState::default();
        state.navigation(fix, 1_000_000);
        let value = state.at(1_000_000);
        assert_eq!(value.gps_epoch_ms, fix.header.gps_epoch_ms());
        assert!(!value.position_valid && !value.velocity_valid);
        assert!(value.position_ecef_m.is_none() && value.velocity_ecef_mps.is_none());
    }

    let mut fix = message();
    fix.position_sigma_m = [0.123_456_79, 0.5, 2.0];
    fix.velocity_sigma_mps = [0.000_123_456_79, 0.125, 0.25];
    let mut state = FixState::default();
    state.navigation(fix, 1_000_000);
    let value = state.at(1_000_000);
    assert!(value.position_valid && value.velocity_valid);
    assert_eq!(value.position_ecef_m, Some(fix.position_ecef_m));
    assert_eq!(value.velocity_ecef_mps, Some(fix.velocity_ecef_mps));
    assert_eq!(value.position_sigma_m, Some(fix.position_sigma_m));
    assert_eq!(value.velocity_sigma_mps, Some(fix.velocity_sigma_mps));
}

#[test]
fn pps_diagnostics_retain_the_raw_rejected_label_and_capture_evidence() {
    let mut state = FixState::default();
    let fix = message();
    let pps = PpsStatus {
        header: fix.header,
        quality: 1,
        week: 2400,
        milliseconds_of_week: 0,
        pulse_error_ns: -12345,
        offset_time_10ps: 2_000_000_000,
        config: 0x03e80021,
    };
    state.pps(pps, 1_000_000, Some(999_999_900), Some(970_000_000));
    state.navigation(fix, 1_020_000);
    let diagnostic = state.at(1_030_000).last_pps.unwrap();
    assert_eq!(diagnostic.quality, 1);
    assert_eq!(diagnostic.config, 0x03e80021);
    assert_eq!(diagnostic.pulse_error_ns, -12345);
    assert_eq!(diagnostic.offset_time_10ps, 2_000_000_000);
    assert_eq!(diagnostic.received_capture_ns, Some(999_999_900));
    assert_eq!(diagnostic.latest_pulse_ns, Some(970_000_000));
    assert_eq!(
        diagnostic.header_epoch_ms.unwrap() - diagnostic.pulse_epoch_ms.unwrap(),
        1000
    );
}

#[test]
fn full_fix_and_pps_response_fit_runtime_frame_and_keep_the_appended_tag() {
    use crate::{device::interface::DeviceResponse, protocol::Response};
    let mut state = FixState::default();
    state.navigation(message(), u64::MAX - 10);
    let mut value = state.at(u64::MAX);
    value.received_age_us = Some(u64::MAX);
    value.gps_epoch_ms = Some(u64::MAX);
    value.position_status = u32::MAX;
    value.position_type = u32::MAX;
    value.velocity_status = u32::MAX;
    value.velocity_type = u32::MAX;
    value.position_age_us = Some(u64::MAX);
    value.velocity_age_us = Some(u64::MAX);
    value.last_pps = Some(GnssPpsDiagnostic {
        received_us: u64::MAX,
        header_epoch_ms: Some(u64::MAX),
        pulse_epoch_ms: Some(u64::MAX),
        output_delay_ms: u16::MAX,
        quality: u32::MAX,
        pulse_error_ns: i32::MIN,
        offset_time_10ps: i32::MIN,
        config: u32::MAX,
        received_capture_ns: Some(u64::MAX),
        latest_pulse_ns: Some(u64::MAX),
    });
    let mut bytes = [0; crate::protocol::RESPONSE_FRAME_SIZE];
    let bytes = postcard::to_slice_cobs(
        &Response::Device(DeviceResponse::GnssFixStatus(value)),
        &mut bytes,
    )
    .unwrap();
    assert_eq!(&bytes[1..3], &[1, 10]);
    let Response::Device(DeviceResponse::GnssFixStatus(actual)) =
        postcard::from_bytes_cobs(bytes).unwrap()
    else {
        panic!()
    };
    assert_eq!(actual.last_pps.unwrap().received_capture_ns, Some(u64::MAX));
    assert_eq!(actual.gps_epoch_ms, Some(u64::MAX));
}
