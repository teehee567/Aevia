//! Read-only power evidence for the sensor-supply investigation.
//!
//! BQ25628E: https://www.ti.com/lit/ds/symlink/bq25628e.pdf (Rev. C)
//! BQ25622E: https://www.ti.com/lit/ds/symlink/bq25622e.pdf (Rev. C)
//! MAX17048: https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf
//! Only register-pointer writes followed by reads are issued. No register
//! values, charging settings, ADC controls, gauge resets or flags are written.

use core::fmt::Write as _;

use esp_hal::{
    Blocking,
    i2c::master::{BusTimeout, Config, Error, I2c, SoftwareTimeout},
    time::{Duration, Rate},
};

/// Call once after initializing I2C0 on GPIO6/SCL and GPIO7/SDA. Each operation
/// has a hardware SCL timeout and a 10 ms software timeout; all loops are fixed
/// length. A missing battery may also make its directly powered gauge absent.
pub fn capture(i2c: &mut I2c<'_, Blocking>) -> heapless::String<160> {
    let mut text = heapless::String::new();
    let config = Config::default()
        .with_frequency(Rate::from_khz(100))
        .with_timeout(BusTimeout::BusCycles(100))
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(10)));
    if i2c.apply_config(&config).is_err() {
        let _ = text.push_str("POWER i2c-config-error");
        return text;
    }
    let _ = text.push_str("POWER");
    match read::<1>(i2c, 0x6a, 0x38) {
        Ok(part) => {
            let _ = write!(text, " bq={:02x}", part[0]);
            if part[0] >> 3 & 7 == 4 {
                bq_status(i2c, &mut text);
            } else {
                let _ = text.push_str(" unexpected-part");
            }
        }
        Err(error) => {
            let nack = matches!(&error, Error::AcknowledgeCheckFailed(_));
            let _ = write!(text, " bq={}", error_label(error));
            if nack {
                // Both datasheets, section 8.6: fitted BQ25628E uses 0x6A,
                // schematic/BOM BQ25622E uses 0x6B. Read only the alternative
                // identity; its register fields are not interchangeable.
                match read::<1>(i2c, 0x6b, 0x38) {
                    Ok(part) => {
                        let _ = write!(text, " bq6b-id={:02x}", part[0]);
                    }
                    Err(error) => {
                        let _ = write!(text, " bq6b={}", error_label(error));
                    }
                }
            }
        }
    }
    match read::<2>(i2c, 0x36, 0x08) {
        Ok(version) => {
            let _ = write!(text, " gauge={:04x}", u16::from_be_bytes(version));
            match read::<2>(i2c, 0x36, 0x02) {
                Ok(voltage) => {
                    // VCELL is a big-endian 16-bit word with 78.125 uV/LSB.
                    let millivolts = (u32::from(u16::from_be_bytes(voltage)) * 5 + 32) / 64;
                    let _ = write!(text, " bat={}mV", millivolts);
                }
                Err(error) => {
                    let _ = write!(text, " bat={}", error_label(error));
                }
            }
        }
        Err(error) => {
            let _ = write!(text, " gauge={}", error_label(error));
        }
    }
    text
}

fn bq_status(i2c: &mut I2c<'_, Blocking>, text: &mut heapless::String<160>) {
    // Stop before the read-to-clear event flags at 0x20..0x22.
    match read::<3>(i2c, 0x6a, 0x1d) {
        Ok(status) => {
            let _ = write!(
                text,
                " st={:02x}/{:02x} fault={:02x}",
                status[0], status[1], status[2]
            );
        }
        Err(error) => {
            let _ = write!(text, " st={}", error_label(error));
        }
    }
    if let Ok(limit) = read::<1>(i2c, 0x6a, 0x19) {
        let _ = write!(text, " ext-ilim={}", u8::from(limit[0] & 4 != 0));
    }
    let controls = match read::<2>(i2c, 0x6a, 0x26) {
        Ok(value) => value,
        Err(error) => {
            let _ = write!(text, " adc={}", error_label(error));
            return;
        }
    };
    let _ = write!(text, " adc={:02x}/{:02x}", controls[0], controls[1]);
    // A disabled ADC retains old register contents, so do not report them as
    // current readings. A running one-shot is not yet a completed conversion.
    if controls[0] & 0xc0 != 0x80 {
        let _ = text.push_str(" ibat=unavailable");
        return;
    }
    // Preserve source register values for diagnosis. The two selected fields
    // are read only when their ADC channels are enabled.
    if controls[1] & 0x08 == 0 {
        if let Ok(raw) = read::<2>(i2c, 0x6a, 0x32) {
            let millivolts = u32::from((u16::from_le_bytes(raw) >> 1) & 0x0fff) * 199 / 100;
            let _ = write!(text, " sys={}mV", millivolts);
        }
    }
    if controls[1] & 0x40 == 0 {
        if let Ok(raw) = read::<2>(i2c, 0x6a, 0x2a) {
            let current = i16::from_le_bytes(raw);
            if current == i16::MIN {
                let _ = text.push_str(" ibat=aborted");
            } else {
                // Signed field 15:2, with 4 mA per bit step.
                let _ = write!(text, " ibat={}mA", (current >> 2) * 4);
            }
        }
    } else {
        let _ = text.push_str(" ibat=disabled");
    }
}

fn read<const N: usize>(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
    register: u8,
) -> Result<[u8; N], Error> {
    let mut bytes = [0; N];
    i2c.write_read(address, &[register], &mut bytes)?;
    Ok(bytes)
}

fn error_label(error: Error) -> &'static str {
    match error {
        Error::AcknowledgeCheckFailed(_) => "nack",
        Error::Timeout => "timeout",
        Error::ArbitrationLost => "arbitration",
        _ => "i2c-error",
    }
}
