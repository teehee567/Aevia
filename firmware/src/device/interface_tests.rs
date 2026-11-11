use super::interface::*;
use crate::protocol::{Request, Response};

#[test]
fn evaluation_commands_append_without_changing_existing_indices() {
    let mut buffer = [0; 16];
    for (command, frame) in [
        (DeviceCommand::Status, &[2, 1, 1, 0][..]),
        (DeviceCommand::Latest, &[3, 1, 1, 0][..]),
        (DeviceCommand::PowerStatus, &[3, 1, 3, 0][..]),
        (DeviceCommand::PeripheralStatus, &[3, 1, 4, 0][..]),
        (DeviceCommand::AcquisitionStatus, &[3, 1, 5, 0][..]),
        (DeviceCommand::GnssStatus, &[3, 1, 6, 0][..]),
        (DeviceCommand::ImuDiagnostics, &[3, 1, 7, 0][..]),
        (DeviceCommand::ResetStatus, &[3, 1, 8, 0][..]),
        (DeviceCommand::GnssTransportStatus, &[3, 1, 9, 0][..]),
        (DeviceCommand::PowerDiagnostics, &[3, 1, 10, 0][..]),
        (DeviceCommand::GnssFixStatus, &[3, 1, 11, 0][..]),
        (DeviceCommand::PowerChargeStatus, &[3, 1, 12, 0][..]),
    ] {
        assert_eq!(
            postcard::to_slice_cobs(&Request::Device(command), &mut buffer).unwrap(),
            frame
        );
    }
}

#[test]
fn charge_status_appends_and_round_trips_without_changing_legacy_power_status() {
    let value = PowerChargeStatus {
        updated_us: Some(u64::MAX),
        verified: true,
        charging_enabled: Some(true),
        charge_current_ma: Some(u16::MAX),
        charge_voltage_mv: Some(u16::MAX),
        input_current_limit_ma: Some(u16::MAX),
        charger_status0: Some(u8::MAX),
        charger_status1: Some(u8::MAX),
        fault_status: Some(u8::MAX),
        precharge_current_ma: Some(u16::MAX),
        termination_current_ma: Some(u16::MAX),
        charger_control0: Some(u8::MAX),
        charger_control1: Some(u8::MAX),
        ntc_control0: Some(u8::MAX),
    };
    let mut output = [0; 512];
    let bytes = postcard::to_slice_cobs(
        &Response::Device(DeviceResponse::PowerChargeStatus(value)),
        &mut output,
    )
    .unwrap();
    assert_eq!(&bytes[1..3], &[1, 11]);
    let Response::Device(DeviceResponse::PowerChargeStatus(actual)) =
        postcard::from_bytes_cobs(bytes).unwrap()
    else {
        panic!()
    };
    assert_eq!(actual, value);
    assert!(DeviceCommand::PowerChargeStatus.available_in_recovery());

    let legacy = PowerStatus {
        part_info: Some(0x18),
        control: Some(0x20),
        charger_status: Some(0x28),
        verified: true,
        errors: 5,
    };
    let bytes = postcard::to_slice(
        &Response::Device(DeviceResponse::PowerStatus(legacy)),
        &mut output,
    )
    .unwrap();
    assert_eq!(bytes, &[1, 2, 1, 0x18, 1, 0x20, 1, 0x28, 1, 5]);
}

#[test]
fn power_diagnostics_response_appends_and_preserves_worst_case_evidence() {
    let value = PowerDiagnostics {
        attempts: u32::MAX,
        successes: u32::MAX,
        last_success_us: Some(u64::MAX),
        stage_failures: [u32::MAX; 6],
        last_failure_stages: u8::MAX,
        last_failure: Some(PowerFailure {
            stage: 6,
            error_kind: 8,
            i2c_kind: Some(7),
            i2c_source: Some(12),
            register: Some(0x16),
            operation: 2,
            expected: Some(u8::MAX),
            actual: Some(u8::MAX),
            at_us: u64::MAX,
        }),
        failed_part_info: Some(u8::MAX),
        failed_pre_control: Some(u8::MAX),
        failed_post_control: Some(u8::MAX),
        failed_charger_status: Some(u8::MAX),
    };
    let mut output = [0; 512];
    let bytes = postcard::to_slice_cobs(
        &Response::Device(DeviceResponse::PowerDiagnostics(value)),
        &mut output,
    )
    .unwrap();
    assert_eq!(&bytes[1..3], &[1, 9]);
    let Response::Device(DeviceResponse::PowerDiagnostics(actual)) =
        postcard::from_bytes_cobs(bytes).unwrap()
    else {
        panic!()
    };
    assert_eq!(actual.stage_failures, [u32::MAX; 6]);
    assert_eq!(actual.last_failure.unwrap().at_us, u64::MAX);
    assert_eq!(actual.last_failure.unwrap().i2c_source, Some(12));
}

#[test]
fn uart_transport_response_is_appended_and_fits_the_runtime_buffer() {
    let port = GnssUartPortStatus {
        rx_bytes: u64::MAX,
        rx_errors: u32::MAX,
        fifo_overflows: u32::MAX,
        glitches: u32::MAX,
        framing_errors: u32::MAX,
        parity_errors: u32::MAX,
        other_errors: u32::MAX,
    };
    let status = GnssTransportStatus {
        ports: [port; 2],
        last_rx_error: Some(GnssUartError {
            port: 2,
            code: 5,
            received_us: u64::MAX,
        }),
    };
    let mut output = [0; 512];
    let response = Response::Device(DeviceResponse::GnssTransportStatus(status));
    let bytes = postcard::to_slice_cobs(&response, &mut output).unwrap();
    // Both envelope tags are nonzero: Device=1, appended response=8.
    assert_eq!(&bytes[1..3], &[1, 8]);
    let decoded: Response = postcard::from_bytes_cobs(bytes).unwrap();
    let Response::Device(DeviceResponse::GnssTransportStatus(actual)) = decoded else {
        panic!()
    };
    assert_eq!(actual, status);
    // The preceding response keeps its old tag and layout.
    let bytes = postcard::to_slice_cobs(
        &Response::Device(DeviceResponse::ResetStatus(ResetStatus::default())),
        &mut output,
    )
    .unwrap();
    assert_eq!(&bytes[1..3], &[1, 7]);
}

#[test]
fn largest_snapshot_response_fits_the_runtime_encode_buffer() {
    let timing = super::timing::TimingStatus {
        pps_locked: true,
        pps_edges: u32::MAX,
        pps_pairs: u32::MAX,
        pps_rejected: u32::MAX,
        pps_age_ms: Some(u32::MAX),
        fit_residual_ns: u32::MAX,
        generation: u32::MAX,
        capture_errors: u32::MAX,
        imu_timing_rejected: u32::MAX,
        gnss_unsynchronized: u32::MAX,
    };
    let snapshot = Snapshot {
        updated_us: u64::MAX,
        phase: Phase::Failed,
        qualified: true,
        epoch_us: Some(u64::MAX),
        horizontal_speed_mps: Some(f32::MAX),
        vertical_speed_mps: Some(f32::MAX),
        position_ecef_m: Some([f64::MAX; 3]),
        velocity_ecef_mps: Some([f32::MAX; 3]),
        acceleration_mps2: [f32::MAX; 3],
        angular_rate_rps: [f32::MAX; 3],
        satellites: u8::MAX,
        imu_rate_hz: f32::MAX,
        gnss_rate_hz: f32::MAX,
        imu_samples: u32::MAX,
        gnss_samples: u32::MAX,
        imu_errors: u32::MAX,
        gnss_errors: u32::MAX,
        gnss_uart_errors: u32::MAX,
        gnss_parse_errors: u32::MAX,
        last_imu_us: u64::MAX,
        last_gnss_us: u64::MAX,
        imu_received: u64::MAX,
        gnss_received: u64::MAX,
        imu_accepted: u64::MAX,
        gnss_fused: u64::MAX,
        input_rejected: u64::MAX,
        too_late: u64::MAX,
        processing_errors: u32::MAX,
        error_kind: Some(EngineError::Metric),
        maximum_step_us: u32::MAX,
        imu_drops: u32::MAX,
        gnss_drops: u32::MAX,
        timing,
    };
    let response = Response::Device(DeviceResponse::Snapshot(snapshot));
    let mut output = [0; 512];
    let bytes = postcard::to_slice_cobs(&response, &mut output).unwrap();
    let decoded: Response = postcard::from_bytes_cobs(bytes).unwrap();
    let Response::Device(DeviceResponse::Snapshot(actual)) = decoded else {
        panic!()
    };
    assert_eq!(actual.updated_us, u64::MAX);
    assert_eq!(actual.timing.gnss_unsynchronized, u32::MAX);
}

#[test]
fn retained_imu_rejection_diagnostics_fit_one_runtime_response_and_round_trip() {
    let diagnostics = ImuDiagnostics {
        attempts: u32::MAX,
        ready: true,
        error_kind: u8::MAX,
        register: Some(u8::MAX),
        expected: Some(u16::MAX),
        actual: Some(u16::MAX),
        component: Some(u16::MAX),
        asic: Some(u16::MAX),
        status_registers: Some([u16::MAX; 10]),
        status_frame_faults: u16::MAX,
        driver_errors: u32::MAX,
        timing_errors: u32::MAX,
        gap_errors: u32::MAX,
        range_errors: u32::MAX,
        drdy_timeouts: u32::MAX,
        spi_timeouts: u32::MAX,
        spi_last_phase: u8::MAX,
        spi_last_context: u8::MAX,
        spi_last_elapsed_us: u32::MAX,
        spi_max_elapsed_us: u32::MAX,
        spi_last_cmd: u32::MAX,
        spi_last_clock: u32::MAX,
        spi_last_source: u32::MAX,
        timing_late_starts: u32::MAX,
        timing_late_finishes: u32::MAX,
        timing_overwritten: u32::MAX,
        max_start_latency_us: u32::MAX,
        max_burst_us: u32::MAX,
        last_start_latency_us: u32::MAX,
        last_burst_us: u32::MAX,
        cycle_clock_active: true,
        cycle_clock_hz: u32::MAX,
        numeric_range_errors: u32::MAX,
        last_numeric_range_at_ns: Some(u64::MAX),
        last_numeric_rate_abs_dps: Some([f64::NAN, f64::INFINITY, f64::MAX]),
        last_numeric_accel_abs_mps2: Some([f64::MAX; 3]),
        last_sensor_status_at_ns: Some(u64::MAX),
        last_sensor_status_register: Some(u8::MAX),
        last_sensor_status_flags: Some(u8::MAX),
        last_sensor_status_payload: Some(u32::MAX),
        last_sensor_status_counter: Some(u8::MAX),
    };
    let response = Response::Device(DeviceResponse::ImuDiagnostics(diagnostics));
    let mut output = [0; 512];
    let bytes = postcard::to_slice_cobs(&response, &mut output).unwrap();
    let decoded: Response = postcard::from_bytes_cobs(bytes).unwrap();
    let Response::Device(DeviceResponse::ImuDiagnostics(actual)) = decoded else {
        panic!()
    };
    assert_eq!(actual.numeric_range_errors, u32::MAX);
    assert!(actual.last_numeric_rate_abs_dps.unwrap()[0].is_nan());
    assert_eq!(actual.last_numeric_range_at_ns, Some(u64::MAX));
    assert_eq!(actual.last_sensor_status_payload, Some(u32::MAX));
    assert_eq!(actual.last_sensor_status_counter, Some(u8::MAX));
}
