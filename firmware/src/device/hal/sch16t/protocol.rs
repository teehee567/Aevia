//! SafeSPI 48-bit framing: local datasheet §§6.2–6.5, Figure 14.
use super::Register;

/// TA9:TA8 strap address (§6.3). Most boards ground both pins.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SlaveAddress {
    #[default]
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

/// Return status is carried on every response; writes report Normal by design.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorStatus {
    Normal,
    Error,
    Saturation,
    Initializing,
}

/// Structural/CRC/command faults; sensor health remains in `Response` metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrameError {
    NoResponse,
    Crc,
    Address,
    DataType,
    Command,
}

/// Complete decoded metadata. Sensor data uses signed 20-bit values, whereas
/// non-sensor registers use the low 16 information bits. Temperature is an
/// exception: its signed 16-bit value is MSB-aligned in the sensor payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Response {
    pub register: Register,
    pub written: bool,
    pub status: SensorStatus,
    pub common_error: bool,
    pub counter: u8,
    pub payload: u32,
}

impl Response {
    pub const fn healthy(self) -> bool {
        matches!(self.status, SensorStatus::Normal) && !self.common_error
    }

    pub const fn information(self) -> u16 {
        self.payload as u16
    }

    pub const fn signed_sensor(self) -> i32 {
        ((self.payload << 12) as i32) >> 12
    }

    pub fn temperature_celsius(self) -> Option<f32> {
        if self.register == Register::Temperature && !self.written {
            Some((self.payload >> 4) as u16 as i16 as f32 / 100.0)
        } else {
            None
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Request {
    pub register: Register,
    pub value: Option<u16>,
}

impl Request {
    pub const fn read(register: Register) -> Self {
        Self {
            register,
            value: None,
        }
    }

    pub const fn write(register: Register, value: u16) -> Self {
        Self {
            register,
            value: Some(value),
        }
    }

    pub fn bytes(self, slave: SlaveAddress) -> [u8; 6] {
        let address = ((slave as u16) << 8) | u16::from(self.register.address());
        let mut frame = (u64::from(address) << 38) | (1 << 35);
        if let Some(value) = self.value {
            frame |= (1 << 37) | (u64::from(value) << 8);
        }
        frame |= u64::from(crc8(frame));
        let bytes = frame.to_be_bytes();
        [bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]]
    }

    pub fn decode(self, bytes: [u8; 6], slave: SlaveAddress) -> Result<Response, FrameError> {
        let frame = u64::from_be_bytes([
            0, 0, bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5],
        ]);
        if frame == 0 || frame == 0xffffffffffff {
            return Err(FrameError::NoResponse);
        }
        if frame as u8 != crc8(frame) {
            return Err(FrameError::Crc);
        }
        let address = ((slave as u16) << 8) | u16::from(self.register.address());
        if ((frame >> 37) & 0x3ff) != u64::from(address) {
            return Err(FrameError::Address);
        }
        if frame & (1 << 35) != 0 {
            return Err(FrameError::Command);
        }
        let sensor_data = self.register.is_sensor() && self.value.is_none();
        if (frame & (1 << 47) != 0) != sensor_data {
            return Err(FrameError::DataType);
        }
        let status = match (frame >> 33) & 3 {
            0 => SensorStatus::Normal,
            1 => SensorStatus::Error,
            2 => SensorStatus::Saturation,
            _ => SensorStatus::Initializing,
        };
        Ok(Response {
            register: self.register,
            written: self.value.is_some(),
            status,
            common_error: frame & (1 << 36) != 0,
            counter: ((frame >> 29) & 15) as u8,
            payload: ((frame >> 8) & 0xfffff) as u32,
        })
    }
}

const fn zero_input_crc_table() -> [u8; 256] {
    let mut table = [0; 256];
    let mut seed = 0;
    while seed < table.len() {
        let mut crc = seed as u8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x2f
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[seed] = crc;
        seed += 1;
    }
    table
}

// ROM storage avoids constructing a table on the interrupt stack. SCH16T uses
// an augmented LFSR: each byte enters after eight shifts, so the recurrence is
// TABLE[crc] XOR byte, rather than the usual TABLE[crc XOR byte].
static CRC_TABLE: [u8; 256] = zero_input_crc_table();

/// Murata's augmented CRC algorithm (Figure 17): clear CRC byte, process all
/// 48 bits, initial 0xff, polynomial 0x2f, no output inversion.
pub fn crc8(frame: u64) -> u8 {
    let bytes = (frame & 0xffffffffff00).to_be_bytes();
    let mut crc = 0xff_u8;
    for &byte in &bytes[2..] {
        crc = CRC_TABLE[usize::from(crc)] ^ byte;
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bit_reference(frame: u64) -> u8 {
        let data = frame & 0xffffffffff00;
        let mut crc = 0xff_u8;
        for bit in (0..48).rev() {
            let input = ((data >> bit) & 1) as u8;
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x2f ^ input
            } else {
                (crc << 1) | input
            };
        }
        crc
    }

    #[test]
    fn byte_crc_matches_augmented_bit_reference_for_all_byte_pairs_and_mixed_frames() {
        for first in 0..=255_u64 {
            for last in 0..=255_u64 {
                let frame = (first << 40) | 0x0034_5678_0000 | (last << 8) | 0xab;
                assert_eq!(crc8(frame), bit_reference(frame));
            }
        }
        let mut frame = 0x9e37_79b9_7f4a_7c15_u64;
        for _ in 0..4096 {
            frame = frame
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            assert_eq!(crc8(frame), bit_reference(frame));
        }
        for frame in [0, u64::MAX, 0x0d68_0000_01d3, 0x0d68_0000_038d] {
            assert_eq!(crc8(frame), bit_reference(frame));
        }
    }

    #[test]
    fn datasheet_command_vectors_match_table_27() {
        assert_eq!(
            Request::read(Register::RateX1).bytes(SlaveAddress::Zero),
            [0x00, 0x48, 0, 0, 0, 0xac]
        );
        assert_eq!(
            Request::read(Register::FilterRate).bytes(SlaveAddress::Zero),
            [0x09, 0x48, 0, 0, 0, 0xfa]
        );
        assert_eq!(
            Request::write(Register::Mode, 1).bytes(SlaveAddress::Zero),
            [0x0d, 0x68, 0, 0, 1, 0xd3]
        );
        assert_eq!(
            Request::write(Register::Mode, 3).bytes(SlaveAddress::Zero),
            [0x0d, 0x68, 0, 0, 3, 0x8d]
        );
        assert_eq!(
            Request::write(Register::Reset, 10).bytes(SlaveAddress::Zero),
            [0x0d, 0xa8, 0, 0, 10, 0xc3]
        );
        assert_eq!(
            Request::write(Register::FilterRate, 0xdb).bytes(SlaveAddress::Zero),
            [0x09, 0x68, 0, 0, 0xdb, 0x9b]
        );
    }

    #[test]
    fn datasheet_sensor_and_temperature_examples() {
        let rate = Request::read(Register::RateX1)
            .decode([0x80, 0x20, 0x0f, 0xfe, 0, 0xad], SlaveAddress::Zero)
            .unwrap();
        assert_eq!(rate.signed_sensor(), -512);
        assert!(rate.healthy());
        let acc = Request::read(Register::AccY1)
            .decode([0x80, 0xa0, 0, 0x0d, 0xc0, 0xdb], SlaveAddress::Zero)
            .unwrap();
        assert_eq!(acc.signed_sensor(), 3520);
        // §7.2.3 prints EAh for this frame, but Figure 17 computes 83h.
        // EAh is the CRC of a different payload (0x82000dc000). Reject the
        // inconsistent printed CRC; do not weaken validation for that typo.
        assert_eq!(
            Request::read(Register::Temperature)
                .decode([0x82, 0, 0, 0x0d, 0xc0, 0xea], SlaveAddress::Zero),
            Err(FrameError::Crc)
        );
        let temp = Request::read(Register::Temperature)
            .decode([0x82, 0, 0, 0x0d, 0xc0, 0x83], SlaveAddress::Zero)
            .unwrap();
        assert_eq!(temp.temperature_celsius(), Some(2.2));
    }

    fn wire(mut frame: u64) -> [u8; 6] {
        frame |= u64::from(crc8(frame));
        let b = frame.to_be_bytes();
        [b[2], b[3], b[4], b[5], b[6], b[7]]
    }

    #[test]
    fn address_crc_type_and_status_are_validated() {
        let request = Request::read(Register::RateX1);
        assert_eq!(
            request.decode([0; 6], SlaveAddress::Zero),
            Err(FrameError::NoResponse)
        );
        assert_eq!(
            request.decode(wire(1 << 47 | 2 << 37), SlaveAddress::Zero),
            Err(FrameError::Address)
        );
        assert_eq!(
            request.decode(wire(1 << 37), SlaveAddress::Zero),
            Err(FrameError::DataType)
        );
        assert_eq!(
            request.decode(wire(1 << 47 | 1 << 37 | 1 << 35), SlaveAddress::Zero),
            Err(FrameError::Command)
        );
        let mut corrupt = wire(1 << 47 | 1 << 37);
        corrupt[3] ^= 1;
        assert_eq!(
            request.decode(corrupt, SlaveAddress::Zero),
            Err(FrameError::Crc)
        );
        let response = request
            .decode(
                wire(1 << 47 | 1 << 37 | 1 << 36 | 2 << 33 | 15 << 29),
                SlaveAddress::Zero,
            )
            .unwrap();
        assert_eq!(response.status, SensorStatus::Saturation);
        assert_eq!(response.counter, 15);
        assert!(!response.healthy());
        let addressed = request
            .decode(wire(1 << 47 | 0x301 << 37), SlaveAddress::Three)
            .unwrap();
        assert!(addressed.healthy());
    }
}
