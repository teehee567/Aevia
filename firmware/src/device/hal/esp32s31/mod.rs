//! ESP32-S31-WROOM-3 platform boundaries and V2 Mini peripheral setup.
//!
//! Hardware authority: local `data/datasheets/esp32-s31-wroom-3_datasheet_en.pdf`,
//! preliminary v0.5, §§1, 3 and 5.2 (module capabilities, pins and peripherals).
//! Concrete bindings follow the installed `esp-hal` 1.2.2 source, especially
//! `i2c/master`, `spi/master`, `uart`, `analog/adc/s31.rs` and generated S31
//! metadata. Wiring is fixed by the PCB-derived `crate::board` module.
//!
//! The datasheet advertises Wi-Fi 6, Bluetooth LE/Classic, 802.15.4 and internal
//! temperature sensing. A module datasheet is not a radio protocol stack. Radio
//! services implement `interfaces::{PacketTransport,GattServer}` in the selected
//! vendor stack; USB CDC supplies `embedded_io::{Read,Write}` plus
//! `interfaces::UsbConnection`. Native USB HS and Serial/JTAG are distinct ports.
//! Native SD and flash/RAM adapters live in `sdcard` and `memory` respectively.
//!
//! **SDK distinction:** esp-hal 1.2.2's S31 metadata does not expose `TSENS` or
//! enable `soc_has_tsens`, despite having a generic driver file for other chips.
//! No fabricated S31 internal-temperature binding is provided. The external
//! NTC, charger and IMU temperature drivers remain available. S31 SAR ADC is
//! supported, but its SDK has no calibration scheme: raw codes are 0..=4393,
//! with ground approximately 2198, not a 0..4095 voltage scale. The adapter here
//! requires measured two-point calibration and never assumes reference voltage.
//!
//! With feature `esp32s31` on RISC-V, explicit `init_*` functions consume only
//! the supplied peripheral/pin owners and configure hardware. Call the vendor's
//! global `init` once in the application first. Other constructors do no I/O.
//! UART/SPI return the vendor drivers, retaining their blocking semantics;
//! use their async modes with caller deadlines when needed. I2C has explicit
//! bus/software timeouts; the ADC adapter has a finite polling budget.

/// Driver defaults for the fitted board, not maximum silicon frequencies.
pub const IMU_SPI_HZ: u32 = 8_000_000;
pub const DISPLAY_SPI_HZ: u32 = 24_000_000;
pub const GNSS_RX_FIFO_THRESHOLD: u16 = 32;
pub const MAX_UART_BAUD: u32 = 5_000_000;
pub const ADC_FULL_SCALE_CODE: u16 = 4393;
pub const ADC_APPROXIMATE_GROUND_CODE: u16 = 2198;

/// Capabilities advertised by the local module specification. Their presence
/// does not assert that this crate includes a protocol stack or an SDK binding.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ModuleCapabilities {
    pub main_uart_controllers: u8,
    pub low_power_uart_controllers: u8,
    pub general_purpose_spi_controllers: u8,
    pub adc_units: u8,
    pub adc_channels_per_unit: u8,
    pub wifi_6_2_4_ghz: bool,
    pub bluetooth_le_5_4: bool,
    pub bluetooth_classic: bool,
    pub ieee_802_15_4: bool,
    pub usb_high_speed_otg: bool,
    pub usb_serial_jtag: bool,
    pub internal_temperature_sensor: bool,
}
pub const MODULE_CAPABILITIES: ModuleCapabilities = ModuleCapabilities {
    main_uart_controllers: 4,
    low_power_uart_controllers: 1,
    general_purpose_spi_controllers: 2,
    adc_units: 2,
    adc_channels_per_unit: 8,
    wifi_6_2_4_ghz: true,
    bluetooth_le_5_4: true,
    bluetooth_classic: true,
    ieee_802_15_4: true,
    usb_high_speed_otg: true,
    usb_serial_jtag: true,
    internal_temperature_sensor: true,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Service {
    Gpio,
    I2c,
    Spi,
    Uart,
    Adc,
    Usb,
    Wifi,
    BluetoothLe,
    BluetoothClassic,
    Ieee802154,
    InternalTemperature,
    HardwareEdgeCapture,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BackendAvailability {
    /// The optional native feature exposes a vendor peripheral driver.
    VendorPeripheral,
    /// Needs a USB class/protocol stack in addition to the vendor controller.
    RequiresUsbStack,
    /// Needs a selected vendor radio/protocol stack implementing the boundary.
    RequiresRadioStack,
    /// Not exposed for S31 by the pinned vendor SDK.
    UnavailableInSdk,
    /// The caller supplies a real hardware-latched capture implementation.
    RequiresCaptureBackend,
}
/// Availability in the pinned SDK, independently of advertised silicon support.
pub const fn backend_availability(service: Service) -> BackendAvailability {
    match service {
        Service::Gpio | Service::I2c | Service::Spi | Service::Uart | Service::Adc => {
            BackendAvailability::VendorPeripheral
        }
        Service::Usb => BackendAvailability::RequiresUsbStack,
        Service::Wifi | Service::BluetoothLe | Service::BluetoothClassic | Service::Ieee802154 => {
            BackendAvailability::RequiresRadioStack
        }
        Service::InternalTemperature => BackendAvailability::UnavailableInSdk,
        Service::HardwareEdgeCapture => BackendAvailability::RequiresCaptureBackend,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigurationError {
    Baudrate,
    Calibration,
    PollBudget,
}

/// 8-N-1 UART settings. A receiver's supported baud rates are a separate limit;
/// this validates only the module/SDK's 1..=5 Mbaud range, not receiver acceptance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GnssUartSettings {
    baudrate: u32,
}
impl GnssUartSettings {
    pub const fn new(baudrate: u32) -> Result<Self, ConfigurationError> {
        if baudrate == 0 || baudrate > MAX_UART_BAUD {
            Err(ConfigurationError::Baudrate)
        } else {
            Ok(Self { baudrate })
        }
    }
    pub const fn baudrate(self) -> u32 {
        self.baudrate
    }
}
impl Default for GnssUartSettings {
    fn default() -> Self {
        Self { baudrate: 115_200 }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CalibrationError {
    OutOfRange,
}

/// Two measured ADC calibration points: (raw code, voltage at the pin in µV).
/// Voltage must rise with code. No extrapolation, factory calibration claim,
/// fixed ground-code correction or assumed ADC reference voltage is applied.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdcCalibration {
    low_code: u16,
    high_code: u16,
    low_microvolts: u32,
    high_microvolts: u32,
}
impl AdcCalibration {
    pub const fn new(low: (u16, u32), high: (u16, u32)) -> Result<Self, ConfigurationError> {
        if low.0 >= high.0 || high.0 > ADC_FULL_SCALE_CODE || low.1 >= high.1 {
            return Err(ConfigurationError::Calibration);
        }
        Ok(Self {
            low_code: low.0,
            high_code: high.0,
            low_microvolts: low.1,
            high_microvolts: high.1,
        })
    }
    /// Linear interpolation, truncated to whole µV. Invalid codes are errors.
    pub fn microvolts(self, code: u16) -> Result<u32, CalibrationError> {
        if code < self.low_code || code > self.high_code {
            return Err(CalibrationError::OutOfRange);
        }
        let numerator =
            u64::from(code - self.low_code) * u64::from(self.high_microvolts - self.low_microvolts);
        Ok(self.low_microvolts + (numerator / u64::from(self.high_code - self.low_code)) as u32)
    }
}

/// Finite polling budget for one conversion attempt. A timeout may leave the
/// same channel's conversion pending; another read can complete that conversion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdcPollBudget(u32);
impl AdcPollBudget {
    pub const fn new(attempts: u32) -> Result<Self, ConfigurationError> {
        if attempts == 0 {
            Err(ConfigurationError::PollBudget)
        } else {
            Ok(Self(attempts))
        }
    }
    pub const fn attempts(self) -> u32 {
        self.0
    }
}

#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
mod native;
#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
pub use native::*;

#[cfg(test)]
mod tests;
