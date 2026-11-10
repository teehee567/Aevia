use super::*;

#[test]
fn uart_settings_reject_invalid_rates_before_hardware_setup() {
    assert_eq!(GnssUartSettings::new(0), Err(ConfigurationError::Baudrate));
    assert_eq!(
        GnssUartSettings::new(5_000_001),
        Err(ConfigurationError::Baudrate)
    );
    assert_eq!(
        GnssUartSettings::new(5_000_000).unwrap().baudrate(),
        5_000_000
    );
    assert_eq!(GnssUartSettings::default().baudrate(), 115_200);
}
#[test]
fn module_claims_and_sdk_availability_are_distinct() {
    const { assert!(MODULE_CAPABILITIES.internal_temperature_sensor) };
    assert_eq!(
        backend_availability(Service::InternalTemperature),
        BackendAvailability::UnavailableInSdk
    );
    assert_eq!(
        backend_availability(Service::Wifi),
        BackendAvailability::RequiresRadioStack
    );
    assert_eq!(
        backend_availability(Service::Usb),
        BackendAvailability::RequiresUsbStack
    );
    assert_eq!(
        backend_availability(Service::HardwareEdgeCapture),
        BackendAvailability::RequiresCaptureBackend
    );
    assert_eq!(
        backend_availability(Service::Adc),
        BackendAvailability::VendorPeripheral
    );
}
#[test]
fn adc_voltage_uses_measured_offset_and_never_assumes_twelve_bit_scaling() {
    let calibration = AdcCalibration::new((2198, 0), (4198, 2_000_000)).unwrap();
    assert_eq!(calibration.microvolts(2198), Ok(0));
    assert_eq!(calibration.microvolts(3198), Ok(1_000_000));
    assert_eq!(calibration.microvolts(4198), Ok(2_000_000));
    assert_eq!(
        calibration.microvolts(2197),
        Err(CalibrationError::OutOfRange)
    );
    assert_eq!(
        calibration.microvolts(4199),
        Err(CalibrationError::OutOfRange)
    );
}
#[test]
fn adc_calibration_and_poll_budget_reject_invalid_configuration() {
    for (low, high) in [
        ((0, 0), (0, 1)),
        ((2, 0), (1, 1)),
        ((0, 0), (4394, 1)),
        ((0, 1), (4393, 1)),
        ((0, 2), (4393, 1)),
    ] {
        assert_eq!(
            AdcCalibration::new(low, high),
            Err(ConfigurationError::Calibration)
        );
    }
    assert_eq!(AdcPollBudget::new(0), Err(ConfigurationError::PollBudget));
    assert_eq!(AdcPollBudget::new(17).unwrap().attempts(), 17);
}
#[test]
fn adc_interpolation_uses_wide_arithmetic_without_overflow() {
    let calibration = AdcCalibration::new((0, 0), (4393, u32::MAX)).unwrap();
    assert_eq!(calibration.microvolts(4393), Ok(u32::MAX));
    assert_eq!(
        calibration.microvolts(2196),
        Ok((2196_u64 * u64::from(u32::MAX) / 4393) as u32)
    );
}
