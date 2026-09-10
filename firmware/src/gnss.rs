//! CRC-checked parsing for the UM980's combined position/velocity output.

/// Confirms ten consecutive 50 ms receiver epochs, independently of fix status.
#[derive(Default)]
pub struct StandaloneRateCheck {
    previous: Option<u64>,
    steps: u8,
}

impl StandaloneRateCheck {
    pub fn observe(&mut self, line: &[u8]) -> bool {
        let Ok(status) = parse_bestnava_status(line) else {
            return false;
        };
        let epoch = u64::from(status.gps_week) * 604_800_000 + u64::from(status.gps_tow_ms);
        self.steps = if self.previous.is_some_and(|previous| epoch == previous + 50) {
            self.steps.saturating_add(1)
        } else {
            0
        };
        self.previous = Some(epoch);
        self.steps >= 10
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParseError {
    Format,
    Checksum,
    NoSolution,
    Range,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BestNav<'a> {
    pub gps_week: u32,
    pub gps_tow_ms: u32,
    pub latitude_e7: i32,
    pub longitude_e7: i32,
    /// Height above mean sea level. Add `undulation_mm` for WGS84 ellipsoid height.
    pub height_mm: i32,
    pub undulation_mm: i32,
    pub latitude_sigma_mm: u32,
    pub longitude_sigma_mm: u32,
    pub height_sigma_mm: u32,
    pub satellites_tracked: u8,
    pub satellites_used: u8,
    pub position_type: &'a [u8],
    pub velocity_type: &'a [u8],
    pub latency_ms: u32,
    pub horizontal_speed_mm_s: u32,
    pub track_hundredths_deg: u32,
    pub vertical_speed_mm_s: i32,
    pub vertical_speed_sigma_mm_s: u32,
    pub horizontal_speed_sigma_mm_s: u32,
}

/// A navigation record that can cross a task queue without retaining the UART
/// decoder's line buffer. The numeric payload is shared with `BestNav`; only
/// its two variable-length solution labels need separate storage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OwnedBestNav {
    fields: BestNav<'static>,
    position_type: [u8; 32],
    position_type_len: usize,
    velocity_type: [u8; 32],
    velocity_type_len: usize,
}

impl BestNav<'_> {
    pub fn to_owned(self) -> OwnedBestNav {
        let mut position_type = [0; 32];
        let mut velocity_type = [0; 32];
        let position_type_len = self.position_type.len().min(position_type.len());
        let velocity_type_len = self.velocity_type.len().min(velocity_type.len());
        position_type[..position_type_len]
            .copy_from_slice(&self.position_type[..position_type_len]);
        velocity_type[..velocity_type_len]
            .copy_from_slice(&self.velocity_type[..velocity_type_len]);
        OwnedBestNav {
            fields: BestNav {
                position_type: b"",
                velocity_type: b"",
                ..self
            },
            position_type,
            position_type_len,
            velocity_type,
            velocity_type_len,
        }
    }
}

impl OwnedBestNav {
    pub fn as_nav(&self) -> BestNav<'_> {
        BestNav {
            position_type: &self.position_type[..self.position_type_len],
            velocity_type: &self.velocity_type[..self.velocity_type_len],
            ..self.fields
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BestNavStatus<'a> {
    pub gps_week: u32,
    pub gps_tow_ms: u32,
    pub position_status: &'a [u8],
    pub position_type: &'a [u8],
    pub velocity_status: &'a [u8],
    pub velocity_type: &'a [u8],
    pub satellites_tracked: u8,
    pub satellites_used: u8,
}

/// Incrementally extracts Unicore `#...\n` logs and `$...\n` acknowledgements.
pub struct LineDecoder<const N: usize> {
    bytes: [u8; N],
    length: usize,
    collecting: bool,
    overflowed: bool,
}

impl<const N: usize> Default for LineDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LineDecoder<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            length: 0,
            collecting: false,
            overflowed: false,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        if byte == b'#' || byte == b'$' {
            self.bytes[0] = byte;
            self.length = 1;
            self.collecting = true;
            self.overflowed = false;
            return None;
        }

        if !self.collecting {
            return None;
        }

        if self.length < N {
            self.bytes[self.length] = byte;
            self.length += 1;
        } else {
            self.overflowed = true;
        }

        if byte != b'\n' {
            return None;
        }

        self.collecting = false;
        if self.overflowed {
            self.length = 0;
            self.overflowed = false;
            None
        } else {
            Some(&self.bytes[..self.length])
        }
    }
}

/// Parse one Unicore `BESTNAVA` record. The ASCII CRC is mandatory.
pub fn parse_bestnava(line: &[u8]) -> Result<BestNav<'_>, ParseError> {
    let (header, body) = split_bestnava(line)?;
    let status = parse_bestnava_status_fields(header, body)?;
    if status.position_status != b"SOL_COMPUTED" || status.velocity_status != b"SOL_COMPUTED" {
        return Err(ParseError::NoSolution);
    }

    let latitude_e7 = parse_decimal_scaled_i32(csv_field(body, 2).ok_or(ParseError::Format)?, 7)?;
    let longitude_e7 = parse_decimal_scaled_i32(csv_field(body, 3).ok_or(ParseError::Format)?, 7)?;
    if !(-900_000_000..=900_000_000).contains(&latitude_e7)
        || !(-1_800_000_000..=1_800_000_000).contains(&longitude_e7)
    {
        return Err(ParseError::Range);
    }

    Ok(BestNav {
        gps_week: status.gps_week,
        gps_tow_ms: status.gps_tow_ms,
        latitude_e7,
        longitude_e7,
        height_mm: parse_decimal_scaled_i32(csv_field(body, 4).ok_or(ParseError::Format)?, 3)?,
        undulation_mm: parse_decimal_scaled_i32(csv_field(body, 5).ok_or(ParseError::Format)?, 3)?,
        latitude_sigma_mm: parse_decimal_scaled_u32(
            csv_field(body, 7).ok_or(ParseError::Format)?,
            3,
        )?,
        longitude_sigma_mm: parse_decimal_scaled_u32(
            csv_field(body, 8).ok_or(ParseError::Format)?,
            3,
        )?,
        height_sigma_mm: parse_decimal_scaled_u32(
            csv_field(body, 9).ok_or(ParseError::Format)?,
            3,
        )?,
        satellites_tracked: status.satellites_tracked,
        satellites_used: status.satellites_used,
        position_type: status.position_type,
        velocity_type: status.velocity_type,
        latency_ms: parse_decimal_scaled_u32(csv_field(body, 23).ok_or(ParseError::Format)?, 3)?,
        horizontal_speed_mm_s: parse_decimal_scaled_u32(
            csv_field(body, 25).ok_or(ParseError::Format)?,
            3,
        )?,
        track_hundredths_deg: parse_decimal_scaled_u32(
            csv_field(body, 26).ok_or(ParseError::Format)?,
            2,
        )?,
        vertical_speed_mm_s: parse_decimal_scaled_i32(
            csv_field(body, 27).ok_or(ParseError::Format)?,
            3,
        )?,
        vertical_speed_sigma_mm_s: parse_decimal_scaled_u32(
            csv_field(body, 28).ok_or(ParseError::Format)?,
            3,
        )?,
        horizontal_speed_sigma_mm_s: parse_decimal_scaled_u32(
            csv_field(body, 29).ok_or(ParseError::Format)?,
            3,
        )?,
    })
}

/// Parse the timing, solution state, and satellite counts even when BESTNAV
/// has no valid position. The frame and CRC must still be valid.
pub fn parse_bestnava_status(line: &[u8]) -> Result<BestNavStatus<'_>, ParseError> {
    let (header, body) = split_bestnava(line)?;
    parse_bestnava_status_fields(header, body)
}

/// Read the checksum-protected response for one particular command. Unicore
/// includes the leading `$` in its command/configuration XOR checksum, unlike
/// ordinary NMEA sentences.
pub fn parse_command_ack(line: &[u8], expected_command: &[u8]) -> Option<bool> {
    let body = checked_response_body(line)?;
    let response = body.strip_prefix(b"command,")?;
    let separator = b",response: ";
    let index = response
        .windows(separator.len())
        .position(|part| part == separator)?;
    if !response[..index].eq_ignore_ascii_case(expected_command) {
        return None;
    }
    Some(response[index + separator.len()..].eq_ignore_ascii_case(b"OK"))
}

pub fn parse_signalgroup(line: &[u8]) -> Option<u8> {
    let body = checked_response_body(line)?;
    let fields = body.strip_prefix(b"CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP ")?;
    let number = fields.split(|byte| *byte == b' ').next()?;
    core::str::from_utf8(number).ok()?.parse().ok()
}

/// Check a named Unicore ASCII record without decoding its message fields.
pub fn valid_unicore_log(line: &[u8], prefix: &[u8]) -> bool {
    let line = line.strip_suffix(b"\n").unwrap_or(line);
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let Some(star) = line.iter().position(|byte| *byte == b'*') else {
        return false;
    };
    line.starts_with(prefix)
        && line.first() == Some(&b'#')
        && line.len() == star + 9
        && parse_hex_u32(&line[star + 1..]).is_ok_and(|crc| crc == unicore_crc32(&line[1..star]))
}

fn checked_response_body(line: &[u8]) -> Option<&[u8]> {
    let line = line.strip_suffix(b"\n").unwrap_or(line);
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    if line.first() != Some(&b'$') {
        return None;
    }
    let star = line.iter().position(|byte| *byte == b'*')?;
    if line.len() != star + 3 {
        return None;
    }
    let expected = u8::from_str_radix(core::str::from_utf8(&line[star + 1..]).ok()?, 16).ok()?;
    let actual = line[..star]
        .iter()
        .fold(0_u8, |checksum, byte| checksum ^ byte);
    (actual == expected).then_some(&line[1..star])
}

fn split_bestnava(line: &[u8]) -> Result<(&[u8], &[u8]), ParseError> {
    let line = line.strip_suffix(b"\n").unwrap_or(line);
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let star = line
        .iter()
        .position(|byte| *byte == b'*')
        .ok_or(ParseError::Format)?;
    if line.first() != Some(&b'#') || line.len() != star + 9 {
        return Err(ParseError::Format);
    }

    let expected_crc = parse_hex_u32(&line[star + 1..])?;
    if unicore_crc32(&line[1..star]) != expected_crc {
        return Err(ParseError::Checksum);
    }

    let semicolon = line[..star]
        .iter()
        .position(|byte| *byte == b';')
        .ok_or(ParseError::Format)?;
    let header = &line[1..semicolon];
    let body = &line[semicolon + 1..star];

    if csv_field(header, 0) != Some(b"BESTNAVA") {
        return Err(ParseError::Format);
    }

    Ok((header, body))
}

fn parse_bestnava_status_fields<'a>(
    header: &'a [u8],
    body: &'a [u8],
) -> Result<BestNavStatus<'a>, ParseError> {
    Ok(BestNavStatus {
        gps_week: parse_u32(csv_field(header, 4).ok_or(ParseError::Format)?)?,
        gps_tow_ms: parse_decimal_scaled_u32(csv_field(header, 5).ok_or(ParseError::Format)?, 0)?,
        position_status: csv_field(body, 0).ok_or(ParseError::Format)?,
        position_type: csv_field(body, 1).ok_or(ParseError::Format)?,
        velocity_status: csv_field(body, 21).ok_or(ParseError::Format)?,
        velocity_type: csv_field(body, 22).ok_or(ParseError::Format)?,
        satellites_tracked: parse_u8(csv_field(body, 13).ok_or(ParseError::Format)?)?,
        satellites_used: parse_u8(csv_field(body, 14).ok_or(ParseError::Format)?)?,
    })
}

pub fn unicore_crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
    }
    crc
}

fn csv_field(line: &[u8], index: usize) -> Option<&[u8]> {
    line.split(|byte| *byte == b',').nth(index)
}

fn parse_u8(value: &[u8]) -> Result<u8, ParseError> {
    u8::try_from(parse_u32(value)?).map_err(|_| ParseError::Range)
}

fn parse_u32(value: &[u8]) -> Result<u32, ParseError> {
    if value.is_empty() {
        return Err(ParseError::Format);
    }

    let mut result = 0_u32;
    for byte in value {
        if !byte.is_ascii_digit() {
            return Err(ParseError::Format);
        }
        result = result
            .checked_mul(10)
            .and_then(|result| result.checked_add(u32::from(byte - b'0')))
            .ok_or(ParseError::Range)?;
    }
    Ok(result)
}

fn parse_hex_u32(value: &[u8]) -> Result<u32, ParseError> {
    if value.len() != 8 {
        return Err(ParseError::Format);
    }

    let mut result = 0_u32;
    for byte in value {
        let digit = match byte {
            b'0'..=b'9' => u32::from(byte - b'0'),
            b'a'..=b'f' => u32::from(byte - b'a') + 10,
            b'A'..=b'F' => u32::from(byte - b'A') + 10,
            _ => return Err(ParseError::Format),
        };
        result = (result << 4) | digit;
    }
    Ok(result)
}

fn parse_decimal_scaled_u32(value: &[u8], fractional_digits: u8) -> Result<u32, ParseError> {
    if value.first() == Some(&b'-') {
        return Err(ParseError::Range);
    }
    u32::try_from(parse_decimal_scaled_i64(value, fractional_digits)?)
        .map_err(|_| ParseError::Range)
}

fn parse_decimal_scaled_i32(value: &[u8], fractional_digits: u8) -> Result<i32, ParseError> {
    i32::try_from(parse_decimal_scaled_i64(value, fractional_digits)?)
        .map_err(|_| ParseError::Range)
}

fn parse_decimal_scaled_i64(value: &[u8], fractional_digits: u8) -> Result<i64, ParseError> {
    if value.is_empty() {
        return Err(ParseError::Format);
    }

    let (negative, value) = match value.first() {
        Some(b'-') => (true, &value[1..]),
        Some(b'+') => (false, &value[1..]),
        _ => (false, value),
    };
    if value.is_empty() {
        return Err(ParseError::Format);
    }

    let dot = value
        .iter()
        .position(|byte| *byte == b'.')
        .unwrap_or(value.len());
    let mut result = i64::from(parse_u32(&value[..dot])?);
    for _ in 0..fractional_digits {
        result = result.checked_mul(10).ok_or(ParseError::Range)?;
    }

    if dot < value.len() {
        let fraction = &value[dot + 1..];
        if fraction.is_empty() || fraction.iter().any(|byte| !byte.is_ascii_digit()) {
            return Err(ParseError::Format);
        }
        let mut scaled_fraction = 0_i64;
        let mut consumed = 0_u8;
        for byte in fraction {
            if consumed == fractional_digits {
                break;
            }
            scaled_fraction = scaled_fraction * 10 + i64::from(byte - b'0');
            consumed += 1;
        }
        while consumed < fractional_digits {
            scaled_fraction *= 10;
            consumed += 1;
        }
        result = result
            .checked_add(scaled_fraction)
            .ok_or(ParseError::Range)?;
    }

    if negative {
        result.checked_neg().ok_or(ParseError::Range)
    } else {
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::{format, vec::Vec};

    const VALID_BESTNAV: &[u8] = b"#BESTNAVA,120,GPS,FINE,2434,345678900,0,0,18,16;SOL_COMPUTED,SINGLE,-35.9011846,149.1616646,700.0,20.0,WGS84,0.8000,0.9000,1.5000,\"0\",0.000,0.000,34,22,0,0,0,0,0,0,SOL_COMPUTED,DOPPLER_VELOCITY,0.010,0.000,28.4583,180.0,0.1,0.2,0.03*59f08c6a\r\n";
    const MANUAL_BESTNAV: &[u8] = b"#BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;SOL_COMPUTED,SINGLE,40.07895888272,116.23651029820,65.8312,-8.4925,WGS84,1.2221,1.1053,2.1970,\"0\",0.000,0.000,50,28,28,0,1,12,12,41,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0046,335.592288,0.0045,0.0194,0.0123*c1b4f7fe\r\n";

    #[test]
    fn parses_synchronised_bestnav_fix() {
        let fix = parse_bestnava(VALID_BESTNAV).unwrap();

        assert_eq!(fix.gps_week, 2434);
        assert_eq!(fix.gps_tow_ms, 345_678_900);
        assert_eq!(fix.latitude_e7, -359_011_846);
        assert_eq!(fix.longitude_e7, 1_491_616_646);
        assert_eq!(fix.horizontal_speed_mm_s, 28_458);
        assert_eq!(fix.track_hundredths_deg, 18_000);
        assert_eq!(fix.satellites_tracked, 34);
        assert_eq!(fix.satellites_used, 22);
        assert_eq!(fix.position_type, b"SINGLE");
        assert_eq!(fix.velocity_type, b"DOPPLER_VELOCITY");
    }

    #[test]
    fn accepts_the_um980_manual_crc_example() {
        let fix = parse_bestnava(MANUAL_BESTNAV).unwrap();
        assert_eq!(fix.satellites_used, 28);
        assert_eq!(fix.horizontal_speed_mm_s, 4);
        assert_eq!(fix.horizontal_speed_sigma_mm_s, 12);
        assert_eq!(fix.height_mm, 65_831);
        assert_eq!(fix.undulation_mm, -8_492);
        assert_eq!(fix.vertical_speed_mm_s, 4);
        assert_eq!(fix.vertical_speed_sigma_mm_s, 19);
    }

    #[test]
    fn owned_navigation_survives_reusing_the_input_buffer() {
        let mut line = MANUAL_BESTNAV.to_vec();
        let owned = parse_bestnava(&line).unwrap().to_owned();
        line.fill(0);
        assert_eq!(owned.as_nav(), parse_bestnava(MANUAL_BESTNAV).unwrap());
    }

    #[test]
    fn checks_unicore_ack_checksum_and_matches_the_requested_command() {
        let ack = b"$command,VERSIONA,response: OK*45\r\n";
        assert_eq!(parse_command_ack(ack, b"VERSIONA"), Some(true));
        assert_eq!(parse_command_ack(ack, b"UNLOG COM1"), None);
        assert_eq!(
            parse_command_ack(b"$command,VERSIONA,response: OK*44", b"VERSIONA"),
            None
        );
        assert_eq!(
            parse_command_ack(
                b"$command,SATSXXXXB COM1 1,response: PARSING FAILD NO MATCHING FUNC SATSXXXXB*21",
                b"SATSXXXXB COM1 1"
            ),
            Some(false)
        );
        assert_eq!(
            parse_signalgroup(b"$CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP 8*1c\r\n"),
            Some(8)
        );
        assert_eq!(
            parse_signalgroup(b"$CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP 3 6*01\r\n"),
            Some(3)
        );
    }

    #[test]
    fn rejects_corrupted_crc_and_invalid_solution() {
        let mut corrupt = VALID_BESTNAV.to_vec();
        corrupt[40] ^= 1;
        assert_eq!(parse_bestnava(&corrupt), Err(ParseError::Checksum));
        assert!(valid_unicore_log(VALID_BESTNAV, b"#BESTNAVA,"));
        assert!(!valid_unicore_log(VALID_BESTNAV, b"#VERSIONA,"));
        assert!(!valid_unicore_log(&corrupt, b"#BESTNAVA,"));

        let invalid = bestnav_with_crc(b"#BESTNAVA,120,GPS,FINE,2434,345678900,0,0,18,16;INSUFFICIENT_OBS,NONE,0,0,0,0,WGS84,0,0,0,\"0\",0,0,0,0,0,0,0,0,0,0,INSUFFICIENT_OBS,NONE,0,0,0,0,0,0,0");
        assert_eq!(parse_bestnava(&invalid), Err(ParseError::NoSolution));
        let status = parse_bestnava_status(&invalid).unwrap();
        assert_eq!(status.position_status, b"INSUFFICIENT_OBS");
        assert_eq!(status.satellites_tracked, 0);
        assert_eq!(status.gps_tow_ms, 345_678_900);
    }

    #[test]
    fn line_decoder_recovers_after_noise_and_oversize_input() {
        let mut decoder = LineDecoder::<512>::new();
        for byte in b"noise\r\n#" {
            assert_eq!(decoder.push(*byte), None);
        }
        for _ in 0..600 {
            assert_eq!(decoder.push(b'X'), None);
        }
        assert_eq!(decoder.push(b'\n'), None);

        let mut parsed = false;
        for byte in VALID_BESTNAV {
            if let Some(line) = decoder.push(*byte) {
                parsed = parse_bestnava(line).is_ok();
            }
        }
        assert!(parsed);
    }

    fn bestnav_with_crc(body: &[u8]) -> Vec<u8> {
        let mut result = body.to_vec();
        result.push(b'*');
        let crc = unicore_crc32(&body[1..]);
        result.extend_from_slice(format!("{crc:08x}\r\n").as_bytes());
        result
    }

    #[test]
    fn bring_up_accepts_no_fix_epochs_but_not_bad_crc_or_wrong_rate() {
        fn frame(epoch: u32) -> Vec<u8> {
            bestnav_with_crc(format!("#BESTNAVA,120,GPS,FINE,2434,{epoch},0,0,18,16;INSUFFICIENT_OBS,NONE,0,0,0,0,WGS84,0,0,0,\"0\",0,0,0,0,0,0,0,0,0,0,INSUFFICIENT_OBS,NONE,0,0,0,0,0,0,0").as_bytes())
        }
        let mut rate = StandaloneRateCheck::default();
        for i in 0..=10 {
            let line = frame(345_678_000 + i * 50);
            assert_eq!(parse_bestnava(&line), Err(ParseError::NoSolution));
            assert_eq!(rate.observe(&line), i == 10);
        }
        assert!(!rate.observe(&frame(345_679_000))); // gap resets confirmation
        for i in 0..20 {
            assert!(!rate.observe(&frame(345_680_000 + i * 1000)));
        }
        let mut rate = StandaloneRateCheck::default();
        for i in 0..20 {
            let mut corrupt = frame(345_678_000 + i * 50);
            corrupt[10] ^= 1;
            assert!(!rate.observe(&corrupt));
        }
    }
}
