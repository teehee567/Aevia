//! Concrete bindings to the pinned S31 SDK. No peripheral stealing or PAC writes.
pub use esp_hal as vendor;

use super::{
    AdcCalibration, AdcPollBudget, CalibrationError, DISPLAY_SPI_HZ, GNSS_RX_FIFO_THRESHOLD,
    GnssUartSettings, IMU_SPI_HZ,
};
use crate::interfaces::AnalogInput;
use esp_hal::{
    Blocking,
    analog::adc::{self, Adc, AdcChannel, AdcPin},
    i2c::master::{self as i2c, BusTimeout, I2c, SoftwareTimeout},
    peripherals::{
        GPIO6, GPIO7, GPIO11, GPIO12, GPIO13, GPIO16, GPIO17, GPIO46, GPIO47, GPIO48, GPIO49, I2C0,
        SPI2, SPI3, UART1, UART2,
    },
    spi::{
        Mode,
        master::{self as spi, Spi},
    },
    time::{Duration, Rate},
    uart::{self, Uart},
};

/// Initialize the fixed Mini shared bus at 100 kHz. Clock stretching is limited
/// to 1000 bus cycles (rounded up by the SDK); each transaction also has a 20 ms
/// software deadline. No component register is read or written by this setup.
pub fn init_power_i2c<'d>(
    peripheral: I2C0<'d>,
    scl: GPIO6<'d>,
    sda: GPIO7<'d>,
) -> Result<I2c<'d, Blocking>, i2c::ConfigError> {
    I2c::new(
        peripheral,
        i2c::Config::default()
            .with_frequency(Rate::from_hz(crate::board::POWER_I2C.frequency_hz))
            .with_timeout(BusTimeout::BusCycles(1000))
            .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(20))),
    )
    .map(|bus| bus.with_scl(scl).with_sda(sda))
}

/// Initialize the IMU's mode 0 SPI bus. CS/reset/DRDY remain with the caller;
/// pair this bus with `bus::ExclusiveSpiDevice` and the appropriate GPIOs.
pub fn init_imu_spi<'d>(
    peripheral: SPI2<'d>,
    sclk: GPIO12<'d>,
    mosi: GPIO11<'d>,
    miso: GPIO13<'d>,
) -> Result<Spi<'d, Blocking>, spi::ConfigError> {
    Spi::new(
        peripheral,
        spi::Config::default()
            .with_frequency(Rate::from_hz(IMU_SPI_HZ))
            .with_mode(Mode::_0),
    )
    .map(|bus| bus.with_sck(sclk).with_mosi(mosi).with_miso(miso))
}

/// Initialize the write-only display bus; leaves CS/DC/reset/backlight ownership
/// with the caller. 24 MHz matches the existing fitted-board setup.
pub fn init_display_spi<'d>(
    peripheral: SPI3<'d>,
    sclk: GPIO16<'d>,
    mosi: GPIO17<'d>,
) -> Result<Spi<'d, Blocking>, spi::ConfigError> {
    Spi::new(
        peripheral,
        spi::Config::default()
            .with_frequency(Rate::from_hz(DISPLAY_SPI_HZ))
            .with_mode(Mode::_0),
    )
    .map(|bus| bus.with_sck(sclk).with_mosi(mosi))
}

fn uart_configuration(settings: GnssUartSettings) -> uart::Config {
    uart::Config::default()
        .with_baudrate(settings.baudrate())
        .with_data_bits(uart::DataBits::_8)
        .with_parity(uart::Parity::None)
        .with_stop_bits(uart::StopBits::_1)
        .with_rx(uart::RxConfig::default().with_fifo_full_threshold(GNSS_RX_FIFO_THRESHOLD))
}

/// Initialize MCU UART1 to UM980 COM1. MCU TX47 feeds receiver RX; MCU RX46
/// receives receiver TX. No receiver commands or autobaud are sent.
pub fn init_gnss_com1<'d>(
    peripheral: UART1<'d>,
    tx: GPIO47<'d>,
    rx: GPIO46<'d>,
    settings: GnssUartSettings,
) -> Result<Uart<'d, Blocking>, uart::ConfigError> {
    Uart::new(peripheral, uart_configuration(settings)).map(|uart| uart.with_tx(tx).with_rx(rx))
}

/// Initialize MCU UART2 to UM980 COM2: MCU TX49, MCU RX48. The fitted board's
/// actual return paths must be verified before relying on configuration replies.
pub fn init_gnss_com2<'d>(
    peripheral: UART2<'d>,
    tx: GPIO49<'d>,
    rx: GPIO48<'d>,
    settings: GnssUartSettings,
) -> Result<Uart<'d, Blocking>, uart::ConfigError> {
    Uart::new(peripheral, uart_configuration(settings)).map(|uart| uart.with_tx(tx).with_rx(rx))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdcReadError {
    /// Preserves the SDK's peripheral error payload. S31 currently reports only
    /// WouldBlock; retain the Other case if the pinned implementation changes.
    Peripheral(()),
    Timeout,
    Calibration(CalibrationError),
}

/// Own an already configured SDK ADC and channel, with measured calibration.
/// Construction performs no I/O. The channel uses the S31 SDK's raw scheme;
/// passing a fictitious factory calibration is intentionally impossible here.
/// Conversion polling is bounded, including when the ADC never completes.
/// After a timeout, retry this channel before releasing the driver to another
/// channel: the SDK may still have its original active conversion cached.
pub struct CalibratedAdc<'d, A, P> {
    adc: Adc<'d, A, Blocking>,
    pin: AdcPin<P, A>,
    calibration: AdcCalibration,
    budget: AdcPollBudget,
}
impl<'d, A, P> CalibratedAdc<'d, A, P> {
    pub const fn new(
        adc: Adc<'d, A, Blocking>,
        pin: AdcPin<P, A>,
        calibration: AdcCalibration,
        budget: AdcPollBudget,
    ) -> Self {
        Self {
            adc,
            pin,
            calibration,
            budget,
        }
    }
    pub fn release(self) -> (Adc<'d, A, Blocking>, AdcPin<P, A>) {
        (self.adc, self.pin)
    }
}
impl<'d, A: adc::RegisterAccess + 'd, P: AdcChannel> AnalogInput for CalibratedAdc<'d, A, P> {
    type Error = AdcReadError;
    fn read_microvolts(&mut self) -> Result<u32, Self::Error> {
        for _ in 0..self.budget.attempts() {
            match self.adc.read_oneshot(&mut self.pin) {
                Ok(raw) => {
                    return self
                        .calibration
                        .microvolts(raw)
                        .map_err(AdcReadError::Calibration);
                }
                Err(nb::Error::WouldBlock) => core::hint::spin_loop(),
                Err(nb::Error::Other(error)) => return Err(AdcReadError::Peripheral(error)),
            }
        }
        Err(AdcReadError::Timeout)
    }
}
