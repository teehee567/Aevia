//! Read-only protocol helpers for V2 Mini peripheral connection probes.

/// SafeSPI read request for the SCH16T component ID at target address zero.
pub const SCH16T_READ_COMPONENT_ID: u64 = 0x0f08_0000_0092;

/// Documented SCH16T software-reset request.
pub const SCH16T_SOFT_RESET: u64 = 0x0da8_0000_0ac3;

/// Builds a six-byte, most-significant-byte-first SafeSPI transfer.
pub const fn sch16t_request_bytes(request: u64) -> [u8; 6] {
    let bytes = request.to_be_bytes();
    [bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]]
}

/// Converts a six-byte SafeSPI response to its 48-bit representation.
pub const fn sch16t_frame(bytes: [u8; 6]) -> u64 {
    ((bytes[0] as u64) << 40)
        | ((bytes[1] as u64) << 32)
        | ((bytes[2] as u64) << 24)
        | ((bytes[3] as u64) << 16)
        | ((bytes[4] as u64) << 8)
        | bytes[5] as u64
}

/// Murata SafeSPI v2 CRC-8 (initial value 0xff, polynomial 0x2f).
pub const fn sch16t_crc8(frame: u64) -> u8 {
    let data = frame & 0xffff_ffff_ff00;
    let mut crc = 0xff_u8;
    let mut bit = 48_u8;

    while bit != 0 {
        bit -= 1;
        let data_bit = ((data >> bit) & 1) as u8;
        crc = if crc & 0x80 != 0 {
            crc.wrapping_shl(1) ^ 0x2f ^ data_bit
        } else {
            crc.wrapping_shl(1) | data_bit
        };
    }

    crc
}

pub const fn sch16t_frame_has_valid_crc(frame: u64) -> bool {
    frame != 0 && frame != 0xffff_ffff_ffff && frame as u8 == sch16t_crc8(frame)
}

pub const fn sch16t_component_id(frame: u64) -> u16 {
    ((frame >> 8) & 0xffff) as u16
}

/// A VERSIONA reply is sufficient to prove the UM980 UART link without an antenna.
pub fn is_um980_version_reply(bytes: &[u8]) -> bool {
    contains(bytes, b"#VERSIONA") || contains(bytes, b"UM980")
}

/// Accepts either the explicit identity reply or unsolicited NMEA as proof that
/// the UM980-to-MCU UART direction is electrically connected.
pub fn is_um980_connection_response(bytes: &[u8]) -> bool {
    is_um980_version_reply(bytes)
        || contains(bytes, b"GNGGA,")
        || contains(bytes, b"GNRMC,")
        || contains(bytes, b"GPGGA,")
        || contains(bytes, b"GPRMC,")
        || contains(bytes, b"$command,")
        || contains(bytes, b"GRAMMAR ERROR")
}

/// Fix fields extracted from a complete GGA sentence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Um980Gga {
    /// NMEA fix quality: zero means invalid/no fix; non-zero means a valid fix.
    pub fix_quality: u8,
    /// Number of satellites used in the reported solution.
    pub satellites: u8,
    /// Whether both latitude and longitude fields are populated.
    pub coordinates_present: bool,
}

/// Finds and parses one complete GP/GN GGA sentence from a UART byte buffer.
pub fn parse_um980_gga(bytes: &[u8]) -> Option<Um980Gga> {
    let gn_start = find(bytes, b"$GNGGA,");
    let gp_start = find(bytes, b"$GPGGA,");
    let start = match (gn_start, gp_start) {
        (Some(gn), Some(gp)) => gn.min(gp),
        (Some(gn), None) => gn,
        (None, Some(gp)) => gp,
        (None, None) => return None,
    };
    let sentence_tail = &bytes[start..];
    let end = sentence_tail
        .iter()
        .position(|byte| *byte == b'\r' || *byte == b'\n')?;
    let sentence = &sentence_tail[..end];

    let latitude = nmea_field(sentence, 2)?;
    let longitude = nmea_field(sentence, 4)?;
    let fix_quality = parse_ascii_u8(nmea_field(sentence, 6)?)?;
    let satellite_field = nmea_field(sentence, 7)?;
    let satellites = if fix_quality == 0 && satellite_field.is_empty() {
        0
    } else {
        parse_ascii_u8(satellite_field)?
    };

    Some(Um980Gga {
        fix_quality,
        satellites,
        coordinates_present: !latitude.is_empty() && !longitude.is_empty(),
    })
}

fn nmea_field(sentence: &[u8], index: usize) -> Option<&[u8]> {
    sentence.split(|byte| *byte == b',').nth(index)
}

fn parse_ascii_u8(bytes: &[u8]) -> Option<u8> {
    if bytes.is_empty() {
        return None;
    }

    let mut value = 0_u8;
    for byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add(byte - b'0')?;
    }
    Some(value)
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    find(haystack, needle).is_some()
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn component_id_request_has_the_documented_crc() {
        assert_eq!(sch16t_crc8(SCH16T_READ_COMPONENT_ID), 0x92);
        assert_eq!(
            sch16t_request_bytes(SCH16T_READ_COMPONENT_ID),
            [0x0f, 0x08, 0x00, 0x00, 0x00, 0x92]
        );
    }

    #[test]
    fn software_reset_request_has_the_documented_crc() {
        assert_eq!(sch16t_crc8(SCH16T_SOFT_RESET), 0xc3);
        assert_eq!(
            sch16t_request_bytes(SCH16T_SOFT_RESET),
            [0x0d, 0xa8, 0x00, 0x00, 0x0a, 0xc3]
        );
    }

    #[test]
    fn parses_and_validates_a_safespi_frame() {
        let frame_without_crc = 0x0000_0000_2100;
        let frame = frame_without_crc | sch16t_crc8(frame_without_crc) as u64;
        let bytes = sch16t_request_bytes(frame);

        assert_eq!(sch16t_frame(bytes), frame);
        assert!(sch16t_frame_has_valid_crc(frame));
        assert_eq!(sch16t_component_id(frame), 0x0021);
        assert!(!sch16t_frame_has_valid_crc(0));
        assert!(!sch16t_frame_has_valid_crc(0xffff_ffff_ffff));
    }

    #[test]
    fn recognizes_um980_version_responses() {
        assert!(is_um980_version_reply(b"#VERSIONA,COM1,0,0.0,FINE,0,0;..."));
        assert!(is_um980_version_reply(b"receiver=UM980\r\n"));
        assert!(!is_um980_version_reply(b"$GNGGA,,,,,,0,,,,,,,,*00\r\n"));
    }

    #[test]
    fn recognizes_um980_connection_without_an_antenna() {
        assert!(is_um980_connection_response(
            b"$GNGGA,,,,,,0,,,,,,,,*00\r\n"
        ));
        assert!(is_um980_connection_response(
            b"$command,VERSIONA,response: OK*45\r\n"
        ));
        assert!(is_um980_connection_response(b"GRAMMAR ERROR,1,"));
        assert!(is_um980_connection_response(b"GNRMC,,V,,,"));
        assert!(!is_um980_connection_response(b"random bytes"));
    }

    #[test]
    fn parses_a_valid_um980_gga_fix() {
        let gga = parse_um980_gga(
            b"noise\r\n$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n",
        )
        .expect("complete GGA sentence");

        assert_eq!(gga.fix_quality, 1);
        assert_eq!(gga.satellites, 8);
        assert!(gga.coordinates_present);
    }

    #[test]
    fn parses_an_indoor_no_fix_um980_gga() {
        let gga = parse_um980_gga(b"$GPGGA,123520.00,,,,,0,,99.99,,,,,,*48\r\n")
            .expect("complete GGA sentence");

        assert_eq!(gga.fix_quality, 0);
        assert_eq!(gga.satellites, 0);
        assert!(!gga.coordinates_present);
    }

    #[test]
    fn rejects_a_truncated_um980_gga() {
        assert!(parse_um980_gga(b"$GNGGA,123519.00,4807").is_none());
    }
}
