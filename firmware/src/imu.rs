//! SCH16T sample types and SafeSPI frame validation, independent of the HAL.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub acceleration_mps2: [f64; 3],
    pub angular_rate_rps: [f64; 3],
    /// End of the averaged interval in the MCU monotonic clock. DRY is timed
    /// in software when serviced; LPF delay has not been calibrated out.
    pub monotonic_us: u64,
    pub interval_us: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuError {
    Spi,
    Crc(u64),
    Address {
        expected: u8,
        frame: u64,
    },
    Command(u64),
    Status(u64),
    StartupStatus {
        before_common: u16,
        registers: [u16; 10],
        frame: u64,
        asic: u16,
    },
    Format(u64),
    Identity(u16),
    Readback {
        address: u8,
        expected: u16,
        actual: u16,
    },
    DataReadyTimeout,
    CounterMismatch {
        previous: [u8; 6],
        current: [u8; 6],
    },
    SampleGap {
        elapsed_us: u32,
        data_ready: bool,
    },
    OutsideCalibratedRange,
}

pub fn request_frame(address: u8, write: Option<u16>) -> u64 {
    let mut frame = (u64::from(address) << 38) | (1 << 35); // FT=1, 48-bit.
    if let Some(value) = write {
        frame |= (1 << 37) | (u64::from(value) << 8);
    }
    frame | u64::from(crc8(frame))
}

pub fn counters_are_consecutive(previous: [u8; 6], current: [u8; 6]) -> bool {
    previous
        .into_iter()
        .zip(current)
        .all(|(old, new)| new == ((old + 1) & 15))
}

pub fn signed_sensor(frame: u64) -> i32 {
    // Signed 20-bit payload occupies bits 27:8.
    (((frame >> 8) as u32 & 0x000f_ffff) << 12) as i32 >> 12
}

pub fn validate_response(frame: u64, expected: u8, strict_status: bool) -> Result<(), ImuError> {
    if frame == 0 || frame == 0xffff_ffff_ffff || frame as u8 != crc8(frame) {
        return Err(ImuError::Crc(frame));
    }
    if ((frame >> 37) & 0x3ff) != u64::from(expected) {
        return Err(ImuError::Address { expected, frame });
    }
    if frame & (1 << 35) != 0 {
        return Err(ImuError::Command(frame));
    }
    if strict_status && (frame & ((1 << 36) | (3 << 33))) != 0 {
        return Err(ImuError::Status(frame));
    }
    Ok(())
}

// Murata's augmented CRC-8 reference algorithm, Fig. 17. The eight zero
// payload bits are shifted through as well as the 40-bit message.
pub fn crc8(frame: u64) -> u8 {
    let data = frame & 0xffff_ffff_ff00;
    let mut crc = 0xff_u8;
    for bit in (0..48).rev() {
        let data_bit = ((data >> bit) & 1) as u8;
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ 0x2f ^ data_bit
        } else {
            (crc << 1) | data_bit
        };
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_single_bit_corruption_is_rejected() {
        let frame = (0x0a_u64 << 37) | (1234 << 8) | (1 << 47);
        let frame = frame | u64::from(crc8(frame));
        assert_eq!(validate_response(frame, 0x0a, true), Ok(()));
        for bit in 0..48 {
            assert!(validate_response(frame ^ (1 << bit), 0x0a, true).is_err());
        }
        assert!(matches!(
            validate_response(frame, 0x0b, true),
            Err(ImuError::Address { .. })
        ));
        for invalid in [0, 0xffff_ffff_ffff] {
            assert!(matches!(
                validate_response(invalid, 0, false),
                Err(ImuError::Crc(_))
            ));
        }
    }

    #[test]
    fn sign_extension_preserves_sensor_range_endpoints() {
        assert_eq!(signed_sensor(0x7ffff << 8), 524287);
        assert_eq!(signed_sensor(0x80000 << 8), -524288);
        assert_eq!(signed_sensor(0xfffff << 8), -1);
    }

    #[test]
    fn independent_counters_must_all_advance_including_wrap() {
        assert!(counters_are_consecutive(
            [15, 0, 1, 3, 7, 8],
            [0, 1, 2, 4, 8, 9]
        ));
        assert!(!counters_are_consecutive(
            [15, 0, 1, 3, 7, 8],
            [0, 1, 2, 4, 8, 8]
        ));
        assert!(!counters_are_consecutive([0; 6], [2; 6]));
    }
}
