//! V2 Mini wiring, read from `hardware/v2_mini_pcb/v2_mini.kicad_pcb`
//! and its production BOM. GPIO numbers are MCU GPIOs, not module pad numbers.
//! Constants reserve no peripherals and perform no configuration.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct I2cWiring {
    pub scl: u8,
    pub sda: u8,
    pub frequency_hz: u32,
}

/// Standard-mode accommodates the BQ25622(E) timing. Never use the LP5813's
/// higher bus limit as the limit of this shared bus.
pub const POWER_I2C: I2cWiring = I2cWiring {
    scl: 6,
    sda: 7,
    frequency_hz: 100_000,
};

pub mod address {
    pub const BQ25622: u8 = 0x6b;
    pub const MAX17048: u8 = 0x36;
    pub const TCA9536A: u8 = 0x40;
    // LP5813 encodes register page in the two low address bits.
    pub const LP5813_BASE: u8 = 0x50;
}

pub mod imu {
    pub const SCLK: u8 = 12;
    pub const MOSI: u8 = 11;
    pub const MISO: u8 = 13;
    pub const CS_N: u8 = 10;
    pub const RESET_N: u8 = 9;
    pub const DRDY: u8 = 14;
    pub const SPI_MODE: super::super::embedded_hal::spi::Mode =
        super::super::embedded_hal::spi::MODE_0;
    pub const MAX_SPI_HZ: u32 = 10_000_000;
}

pub mod gnss {
    // These names describe the MCU direction; receiver RX is MCU TX.
    pub const COM1_MCU_TX: u8 = 47;
    pub const COM1_MCU_RX: u8 = 46;
    pub const COM2_MCU_TX: u8 = 49;
    pub const COM2_MCU_RX: u8 = 48;
    pub const RESET_N: u8 = 44;
    pub const PPS: u8 = 42;
    pub const PVT_STATUS: u8 = 50;
    pub const RTK_STATUS: u8 = 51;
    pub const ERROR_STATUS: u8 = 52;
}

pub mod display {
    pub const SCLK: u8 = 16;
    pub const MOSI: u8 = 17;
    pub const CS_N: u8 = 18;
    pub const DATA_COMMAND: u8 = 19;
    pub const RESET_N: u8 = 15;
    pub const BACKLIGHT: u8 = 4;
    pub const TEARING_EFFECT: u8 = 5;
}

pub mod sd {
    pub const CLK: u8 = 24;
    pub const CMD: u8 = 25;
    pub const DATA: [u8; 4] = [20, 21, 22, 23];
    /// Socket CD contact is unconnected. Probe card presence through SDMMC.
    pub const CARD_DETECT: Option<u8> = None;
}

pub mod power {
    pub const KILL_N: u8 = 8;
    pub const INTERRUPT_N: u8 = 38;
    pub const CHARGER_INTERRUPT_N: u8 = 0;
    pub const CHARGER_STATUS_N: u8 = 1;
    pub const CHARGER_POWER_GOOD_N: u8 = 40;
    pub const BATTERY_ALERT_N: u8 = 2;
    pub const LED_ENABLE: u8 = 39;
    pub const USB_VBUS_SENSE: u8 = 35;
    pub const USB_CC1_SENSE: u8 = 43;
    pub const USB_CC2_SENSE: u8 = 45;
    // TH1 is measured by charger TS ADC, not by a direct MCU ADC GPIO.
    pub const TS_PULLUP_OHMS: u32 = 5_230;
    pub const TS_PARALLEL_OHMS: u32 = 30_100;
}

pub mod controls {
    // SW3, SW6, SW4, SW5 respectively. Active low at expander P0..P3.
    pub const UI_BUTTON_PORTS: [u8; 4] = [0, 1, 2, 3];
    pub const BOOT_BUTTON: u8 = 61;
    // SW1 resets MCU EN; SW7 feeds MAX16169 PB_IN, neither is a GPIO button.
}

/// V2 Mini USB-C exposes the MCU USB data pads, through series resistors;
/// they are dedicated pads, not a remappable GPIO pair. Tag-Connect J5 is
/// USB Serial/JTAG on GPIO33/34 and is a separate connection.
pub mod usb {
    pub const SERIAL_JTAG_DM: u8 = 33;
    pub const SERIAL_JTAG_DP: u8 = 34;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn functional_gpio_assignments_do_not_overlap() {
        let pins = [
            POWER_I2C.scl,
            POWER_I2C.sda,
            imu::SCLK,
            imu::MOSI,
            imu::MISO,
            imu::CS_N,
            imu::RESET_N,
            imu::DRDY,
            gnss::COM1_MCU_TX,
            gnss::COM1_MCU_RX,
            gnss::COM2_MCU_TX,
            gnss::COM2_MCU_RX,
            gnss::RESET_N,
            gnss::PPS,
            gnss::PVT_STATUS,
            gnss::RTK_STATUS,
            gnss::ERROR_STATUS,
            display::SCLK,
            display::MOSI,
            display::CS_N,
            display::DATA_COMMAND,
            display::RESET_N,
            display::BACKLIGHT,
            display::TEARING_EFFECT,
            sd::CLK,
            sd::CMD,
            sd::DATA[0],
            sd::DATA[1],
            sd::DATA[2],
            sd::DATA[3],
            power::KILL_N,
            power::INTERRUPT_N,
            power::CHARGER_INTERRUPT_N,
            power::CHARGER_STATUS_N,
            power::CHARGER_POWER_GOOD_N,
            power::BATTERY_ALERT_N,
            power::LED_ENABLE,
            power::USB_VBUS_SENSE,
            power::USB_CC1_SENSE,
            power::USB_CC2_SENSE,
            controls::BOOT_BUTTON,
            usb::SERIAL_JTAG_DM,
            usb::SERIAL_JTAG_DP,
        ];
        for (i, pin) in pins.iter().enumerate() {
            assert!(!pins[..i].contains(pin));
        }
    }
}
