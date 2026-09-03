//! Read-only bring-up probes for the devices on the V2 Mini power I2C bus.
//!
//! These functions intentionally contain no register writes. The charger starts
//! autonomously in hardware, and its battery policy must be validated separately.

use embedded_hal::i2c::I2c;

pub const BQ2562X_ADDRESS: u8 = 0x6b;
pub const MAX17048_ADDRESS: u8 = 0x36;
pub const TCA9536A_ADDRESS: u8 = 0x40;
pub const LP5813A_PAGE_0_ADDRESS: u8 = 0x50;
const LP5813_BROADCAST_PAGE_0_ADDRESS: u8 = 0x6c;

const LP5813_CHIP_ENABLE: u16 = 0x000;
const LP5813_DEVICE_CONFIG_0: u16 = 0x001;
const LP5813_DEVICE_CONFIG_1: u16 = 0x002;
const LP5813_DEVICE_CONFIG_12: u16 = 0x00d;
const LP5813_CMD_UPDATE: u16 = 0x010;
const LP5813_LED_ENABLE_0: u16 = 0x020;
const LP5813_LED_ENABLE_1: u16 = 0x021;
const LP5813_MANUAL_DC_FIRST_MATRIX_CHANNEL: u16 = 0x034;
const LP5813_MANUAL_PWM_FIRST_MATRIX_CHANNEL: u16 = 0x044;
const LP5813_CONFIG_ERROR_STATUS: u16 = 0x300;
pub const LP5813_MATRIX_CHANNEL_COUNT: usize = 12;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Bq2562xSnapshot {
    pub part_information: u8,
    pub charge_current_limit: u16,
    pub charge_voltage_limit: u16,
    pub minimum_system_voltage: u16,
    pub charger_control_0: u8,
    pub charger_control_3: u8,
    pub charger_status_0: u8,
    pub charger_status_1: u8,
    pub fault_status_0: u8,
}

impl Bq2562xSnapshot {
    pub const fn part_number(self) -> u8 {
        (self.part_information >> 3) & 0x07
    }

    pub const fn device_revision(self) -> u8 {
        self.part_information & 0x07
    }

    pub const fn part_name(self) -> &'static str {
        match self.part_number() {
            1 => "BQ25622",
            3 => "BQ25622E",
            4 => "BQ25628E",
            _ => "unknown BQ2562x",
        }
    }

    pub const fn is_supported_charger(self) -> bool {
        matches!(self.part_number(), 1 | 3 | 4)
    }

    pub const fn charge_state(self) -> u8 {
        (self.charger_status_1 >> 3) & 0x03
    }

    pub const fn vbus_state(self) -> u8 {
        self.charger_status_1 & 0x07
    }
}

pub fn read_bq2562x<I>(bus: &mut I) -> Result<Bq2562xSnapshot, I::Error>
where
    I: I2c,
{
    let part_information = read_u8(bus, BQ2562X_ADDRESS, 0x38)?;
    let charge_current_limit = read_u16_le(bus, BQ2562X_ADDRESS, 0x02)?;
    let charge_voltage_limit = read_u16_le(bus, BQ2562X_ADDRESS, 0x04)?;
    let minimum_system_voltage = read_u16_le(bus, BQ2562X_ADDRESS, 0x0e)?;
    let charger_control_0 = read_u8(bus, BQ2562X_ADDRESS, 0x14)?;
    let charger_control_3 = read_u8(bus, BQ2562X_ADDRESS, 0x18)?;
    let mut status = [0; 3];
    bus.write_read(BQ2562X_ADDRESS, &[0x1d], &mut status)?;

    Ok(Bq2562xSnapshot {
        part_information,
        charge_current_limit,
        charge_voltage_limit,
        minimum_system_voltage,
        charger_control_0,
        charger_control_3,
        charger_status_0: status[0],
        charger_status_1: status[1],
        fault_status_0: status[2],
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Max17048Snapshot {
    pub version: u16,
    pub cell_voltage: u16,
    pub state_of_charge: u16,
    pub status: u16,
}

impl Max17048Snapshot {
    /// The datasheet defines production versions as 0x001x.
    pub const fn has_expected_version(self) -> bool {
        self.version & 0xfff0 == 0x0010
    }

    /// Integer millivolts. The register scale is 78.125 uV/LSB, or 5/64 mV/LSB.
    pub const fn cell_millivolts(self) -> u32 {
        (self.cell_voltage as u32 * 5) / 64
    }

    /// State of charge in hundredths of one percent.
    pub const fn state_of_charge_hundredths(self) -> u16 {
        ((self.state_of_charge as u32 * 100) / 256) as u16
    }
}

pub fn read_max17048<I>(bus: &mut I) -> Result<Max17048Snapshot, I::Error>
where
    I: I2c,
{
    Ok(Max17048Snapshot {
        version: read_u16_be(bus, MAX17048_ADDRESS, 0x08)?,
        cell_voltage: read_u16_be(bus, MAX17048_ADDRESS, 0x02)?,
        state_of_charge: read_u16_be(bus, MAX17048_ADDRESS, 0x04)?,
        status: read_u16_be(bus, MAX17048_ADDRESS, 0x1a)?,
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Tca9536aSnapshot {
    pub inputs: u8,
    pub configuration: u8,
}

impl Tca9536aSnapshot {
    pub const fn is_reset_and_idle(self) -> bool {
        self.configuration == 0xff && self.inputs & 0x0f == 0x0f
    }
}

pub fn read_tca9536a<I>(bus: &mut I) -> Result<Tca9536aSnapshot, I::Error>
where
    I: I2c,
{
    Ok(Tca9536aSnapshot {
        inputs: read_u8(bus, TCA9536A_ADDRESS, 0x00)?,
        configuration: read_u8(bus, TCA9536A_ADDRESS, 0x03)?,
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Lp5813aSnapshot {
    pub chip_enable: u8,
    pub device_config_0: u8,
    pub device_config_1: u8,
    pub device_config_2: u8,
}

impl Lp5813aSnapshot {
    pub const fn is_reset_state(self) -> bool {
        self.chip_enable == 0
            && self.device_config_0 == 0
            && self.device_config_1 == 0
            && self.device_config_2 == 0xe4
    }
}

/// Reads LP5813A registers 0x000..=0x003 after its EN pin has been high for at least 1 ms.
pub fn read_lp5813a_reset_state<I>(bus: &mut I) -> Result<Lp5813aSnapshot, I::Error>
where
    I: I2c,
{
    let mut registers = [0; 4];
    bus.write_read(LP5813A_PAGE_0_ADDRESS, &[0x00], &mut registers)?;
    Ok(Lp5813aSnapshot {
        chip_enable: registers[0],
        device_config_0: registers[1],
        device_config_1: registers[2],
        device_config_2: registers[3],
    })
}

/// LP5813 encodes register bits 9:8 in the low two bits of its 7-bit I2C address.
pub const fn lp5813a_address_for(register: u16) -> Option<u8> {
    if register <= 0x03ff {
        Some(LP5813A_PAGE_0_ADDRESS | ((register >> 8) as u8 & 0x03))
    } else {
        None
    }
}

/// Configures LP5813A for 4-scan manual drive at a conservative 1.6 mA peak.
///
/// This is the TI-documented test sequence adapted to the board's 4 x 3 LED
/// matrix. It returns the configuration-error register and leaves PWM at zero.
pub fn configure_lp5813a_matrix_test<I>(bus: &mut I) -> Result<u8, (&'static str, I::Error)>
where
    I: I2c,
{
    // TI requires this sequence once after power-up. Broadcast page 0 is
    // 0x6c; register bits 9:8 continue in the low two address bits.
    for (register, value) in [
        (0x000, 0x01),
        (0x350, 0x05),
        (0x350, 0x08),
        (0x350, 0x01),
        (0x350, 0x03),
        (0x351, 0x27),
        (0x350, 0x00),
        (0x000, 0x00),
    ] {
        write_lp5813a_at_base(bus, LP5813_BROADCAST_PAGE_0_ADDRESS, register, value)
            .map_err(|error| ("slave-addressing", error))?;
    }

    write_lp5813a(bus, LP5813_CHIP_ENABLE, 0x01).map_err(|error| ("chip-enable", error))?;
    write_lp5813a(bus, LP5813_DEVICE_CONFIG_0, 0x26).map_err(|error| ("device-config-0", error))?;
    write_lp5813a(bus, LP5813_DEVICE_CONFIG_1, 0x40).map_err(|error| ("device-config-1", error))?;
    write_lp5813a(bus, LP5813_DEVICE_CONFIG_12, 0x0b)
        .map_err(|error| ("device-config-12", error))?;
    write_lp5813a(bus, LP5813_CMD_UPDATE, 0x55).map_err(|error| ("config-update", error))?;

    let config_error = read_lp5813a(bus, LP5813_CONFIG_ERROR_STATUS)
        .map_err(|error| ("config-error-read", error))?;
    if config_error != 0 {
        return Ok(config_error);
    }

    write_lp5813a(bus, LP5813_LED_ENABLE_0, 0xf0).map_err(|error| ("led-enable-0", error))?;
    write_lp5813a(bus, LP5813_LED_ENABLE_1, 0xff).map_err(|error| ("led-enable-1", error))?;
    for channel in 0..LP5813_MATRIX_CHANNEL_COUNT as u16 {
        write_lp5813a(bus, LP5813_MANUAL_DC_FIRST_MATRIX_CHANNEL + channel, 0x10)
            .map_err(|error| ("dot-current", error))?;
        write_lp5813a(bus, LP5813_MANUAL_PWM_FIRST_MATRIX_CHANNEL + channel, 0x00)
            .map_err(|error| ("pwm-clear", error))?;
    }

    Ok(config_error)
}

/// Sets every physical matrix LED to the same manual PWM duty cycle.
pub fn set_lp5813a_matrix_pwm<I>(bus: &mut I, pwm: u8) -> Result<(), I::Error>
where
    I: I2c,
{
    for channel in 0..LP5813_MATRIX_CHANNEL_COUNT as u16 {
        write_lp5813a(bus, LP5813_MANUAL_PWM_FIRST_MATRIX_CHANNEL + channel, pwm)?;
    }
    Ok(())
}

fn read_lp5813a<I>(bus: &mut I, register: u16) -> Result<u8, I::Error>
where
    I: I2c,
{
    let address = lp5813a_address_for(register).expect("LP5813 register out of range");
    read_u8(bus, address, register as u8)
}

fn write_lp5813a<I>(bus: &mut I, register: u16, value: u8) -> Result<(), I::Error>
where
    I: I2c,
{
    write_lp5813a_at_base(bus, LP5813A_PAGE_0_ADDRESS, register, value)
}

fn write_lp5813a_at_base<I>(
    bus: &mut I,
    page_0_address: u8,
    register: u16,
    value: u8,
) -> Result<(), I::Error>
where
    I: I2c,
{
    let address = page_0_address | ((register >> 8) as u8 & 0x03);
    bus.write(address, &[register as u8, value])
}

fn read_u8<I>(bus: &mut I, address: u8, register: u8) -> Result<u8, I::Error>
where
    I: I2c,
{
    let mut value = [0];
    bus.write_read(address, &[register], &mut value)?;
    Ok(value[0])
}

fn read_u16_le<I>(bus: &mut I, address: u8, register: u8) -> Result<u16, I::Error>
where
    I: I2c,
{
    let mut value = [0; 2];
    bus.write_read(address, &[register], &mut value)?;
    Ok(u16::from_le_bytes(value))
}

fn read_u16_be<I>(bus: &mut I, address: u8, register: u8) -> Result<u16, I::Error>
where
    I: I2c,
{
    let mut value = [0; 2];
    bus.write_read(address, &[register], &mut value)?;
    Ok(u16::from_be_bytes(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_the_bq25622_i2c_address() {
        assert_eq!(BQ2562X_ADDRESS, 0x6b);
    }

    #[test]
    fn decodes_supported_bq2562x_identities() {
        let snapshot = Bq2562xSnapshot {
            part_information: 0x22,
            charge_current_limit: 0,
            charge_voltage_limit: 0,
            minimum_system_voltage: 0,
            charger_control_0: 0,
            charger_control_3: 0,
            charger_status_0: 0,
            charger_status_1: 0,
            fault_status_0: 0,
        };

        assert!(snapshot.is_supported_charger());
        assert_eq!(snapshot.part_name(), "BQ25628E");
        assert_eq!(snapshot.part_number(), 4);
        assert_eq!(snapshot.device_revision(), 2);

        let bq25622 = Bq2562xSnapshot {
            part_information: 0x0a,
            ..snapshot
        };
        assert!(bq25622.is_supported_charger());
        assert_eq!(bq25622.part_name(), "BQ25622");
    }

    #[test]
    fn converts_max17048_register_scales() {
        let snapshot = Max17048Snapshot {
            version: 0x0012,
            cell_voltage: 0xc000,
            state_of_charge: 0x3280,
            status: 0,
        };

        assert!(snapshot.has_expected_version());
        assert_eq!(snapshot.cell_millivolts(), 3_840);
        assert_eq!(snapshot.state_of_charge_hundredths(), 5_050);
    }

    #[test]
    fn validates_tca9536a_reset_and_idle_state() {
        assert!(
            Tca9536aSnapshot {
                inputs: 0xff,
                configuration: 0xff,
            }
            .is_reset_and_idle()
        );
        assert!(
            !Tca9536aSnapshot {
                inputs: 0xfe,
                configuration: 0xff,
            }
            .is_reset_and_idle()
        );
    }

    #[test]
    fn maps_lp5813_register_pages_into_the_address() {
        assert_eq!(lp5813a_address_for(0x000), Some(0x50));
        assert_eq!(lp5813a_address_for(0x100), Some(0x51));
        assert_eq!(lp5813a_address_for(0x300), Some(0x53));
        assert_eq!(lp5813a_address_for(0x400), None);
    }

    #[test]
    fn lp5813_broadcast_address_uses_the_same_register_page_bits() {
        assert_eq!(LP5813_BROADCAST_PAGE_0_ADDRESS, 0x6c);
        assert_eq!(
            LP5813_BROADCAST_PAGE_0_ADDRESS | ((0x350 >> 8) & 0x03),
            0x6f
        );
    }
}
