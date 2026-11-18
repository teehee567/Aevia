//! Mixed navigation / CONFIG ASCII and PPSSTATUSB / SPPNAVB decoder.
//! Binary payloads, including unsupported oversized logs, are never searched
//! for configuration replies. A truncated frame requires an idle reset.
use super::{PpsLabel, gps_epoch_ms};
const SYNC: [u8; 3] = [0xaa, 0x44, 0xb5];
#[derive(Clone, Copy)]
enum Mode {
    Search,
    Line,
    Binary,
    Skip(usize),
}
#[derive(Clone, Copy, Debug)]
pub enum Frame<'a> {
    Line(&'a [u8]),
    Pps(PpsLabel),
    InvalidPps,
    SppNav(crate::gnss_binary::NativeSpp),
    InvalidSppNav,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Capacity,
    Checksum,
    InvalidLine,
}
pub struct Decoder<const N: usize> {
    bytes: [u8; N],
    len: usize,
    total: usize,
    mode: Mode,
}
impl<const N: usize> Default for Decoder<N> {
    fn default() -> Self {
        Self::new()
    }
}
impl<const N: usize> Decoder<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            total: 0,
            mode: Mode::Search,
        }
    }
    pub fn reset(&mut self) {
        self.len = 0;
        self.total = 0;
        self.mode = Mode::Search;
    }
    pub fn push(&mut self, b: u8) -> Result<Option<Frame<'_>>, DecodeError> {
        if N < 28 {
            return Err(DecodeError::Capacity);
        }
        if let Mode::Skip(left) = self.mode {
            if left <= 1 {
                self.reset();
            } else {
                self.mode = Mode::Skip(left - 1);
            }
            return Ok(None);
        }
        if matches!(self.mode, Mode::Search) {
            if b == SYNC[0] {
                self.mode = Mode::Binary;
            } else if matches!(b, b'#' | b'$' | b'!') {
                self.mode = Mode::Line;
            } else {
                return Ok(None);
            }
        }
        if matches!(self.mode, Mode::Binary) && self.len < 3 {
            if b == SYNC[self.len] {
                self.bytes[self.len] = b;
                self.len += 1;
            } else {
                self.reset();
                if b == SYNC[0] {
                    self.mode = Mode::Binary;
                    self.bytes[0] = b;
                    self.len = 1;
                } else if matches!(b, b'#' | b'$' | b'!') {
                    self.mode = Mode::Line;
                    self.bytes[0] = b;
                    self.len = 1;
                }
            }
            return Ok(None);
        }
        if matches!(self.mode, Mode::Line) {
            if b == b'\n' {
                let len = self.len;
                self.reset();
                return Ok(Some(Frame::Line(&self.bytes[..len])));
            }
            if b == SYNC[0] {
                self.reset();
                self.mode = Mode::Binary;
                self.bytes[0] = b;
                self.len = 1;
                return Ok(None);
            }
            if b != b'\r' && !(0x20..=0x7e).contains(&b) {
                self.reset();
                return Err(DecodeError::InvalidLine);
            }
            if matches!(b, b'#' | b'$' | b'!') {
                self.len = 0;
            }
            if self.len == N {
                self.reset();
                return Err(DecodeError::Capacity);
            }
            self.bytes[self.len] = b;
            self.len += 1;
            return Ok(None);
        }
        self.bytes[self.len] = b;
        self.len += 1;
        if self.len == 8 {
            self.total = 28 + usize::from(u16::from_le_bytes([self.bytes[6], self.bytes[7]]));
            if self.total > N {
                self.mode = Mode::Skip(self.total - self.len);
                return Ok(None);
            }
        }
        if self.total != 0 && self.len == self.total {
            let len = self.len;
            self.reset();
            let data = &self.bytes[..len - 4];
            let expected = u32::from_le_bytes(self.bytes[len - 4..len].try_into().unwrap());
            if crate::gnss::unicore_crc32(data) != expected {
                return Err(DecodeError::Checksum);
            }
            match u16::from_le_bytes([data[4], data[5]]) {
                9000 => return Ok(Some(parse_pps(data).map_or(Frame::InvalidPps, Frame::Pps))),
                crate::gnss_binary::MESSAGE_ID => {
                    return Ok(Some(
                        crate::gnss_binary::NativeSpp::from_frame(data)
                            .map_or(Frame::InvalidSppNav, Frame::SppNav),
                    ));
                }
                _ => return Ok(None),
            }
        }
        Ok(None)
    }
}
fn parse_pps(bytes: &[u8]) -> Option<PpsLabel> {
    if bytes.len() != 84 || bytes[8] != 0 || bytes[9] != 160 {
        return None;
    }
    let u32_at = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
    Some(PpsLabel {
        epoch_ms: gps_epoch_ms(u32_at(28), u32_at(32))?,
        header_epoch_ms: gps_epoch_ms(
            u32::from(u16::from_le_bytes([bytes[10], bytes[11]])),
            u32_at(12),
        )?,
        output_delay_ns: u64::from(u16::from_le_bytes([bytes[22], bytes[23]])) * 1_000_000,
        quality: u32_at(24),
        pulse_error_ns: i32::from_le_bytes(bytes[36..40].try_into().unwrap()),
        config: u32_at(44),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn binary(id: u16, payload: &[u8]) -> std::vec::Vec<u8> {
        let mut b = std::vec![0;24];
        b[..3].copy_from_slice(&SYNC);
        b[4..6].copy_from_slice(&id.to_le_bytes());
        b[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        b[9] = 160;
        b[10..12].copy_from_slice(&2400u16.to_le_bytes());
        b[12..16].copy_from_slice(&1000u32.to_le_bytes());
        b.extend_from_slice(payload);
        b.extend_from_slice(&crate::gnss::unicore_crc32(&b).to_le_bytes());
        b
    }
    #[test]
    fn both_binary_and_oversized_binary_hide_embedded_config_lines() {
        for n in [false, true] {
            let mut d = Decoder::<128>::new();
            let mut p = std::vec![];
            p.extend_from_slice(
                b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0*00\r\n",
            );
            if n {
                p.resize(1000, 0);
            }
            for b in binary(999, &p) {
                assert!(d.push(b).unwrap().is_none());
            }
            let mut lines = 0;
            for b in b"#BESTNAVA,example\r\n" {
                if let Some(Frame::Line(line)) = d.push(*b).unwrap() {
                    assert_eq!(line, b"#BESTNAVA,example\r");
                    lines += 1;
                }
            }
            assert_eq!(lines, 1);
        }
    }
    #[test]
    fn typed_pps_checks_crc_header_quality_and_full_week_fields() {
        let mut p = [0; 60];
        p[0..4].copy_from_slice(&3u32.to_le_bytes());
        p[4..8].copy_from_slice(&2400u32.to_le_bytes());
        p[8..12].copy_from_slice(&1000u32.to_le_bytes());
        p[20..24].copy_from_slice(&((1000u32 << 16) | 0x20).to_le_bytes());
        let b = binary(9000, &p);
        let mut d = Decoder::<128>::new();
        let mut seen = false;
        for byte in &b {
            if let Some(Frame::Pps(label)) = d.push(*byte).unwrap() {
                assert_eq!(label.epoch_ms, 2400 * 604_800_000 + 1000);
                assert_eq!(label.quality, 3);
                seen = true;
            }
        }
        assert!(seen);
        let mut bad = b.clone();
        bad[30] ^= 1;
        let mut rejected = false;
        for byte in bad {
            rejected |= d.push(byte).is_err();
        }
        assert!(rejected);
        let mut bad = b;
        bad[9] = 100;
        let end = bad.len() - 4;
        let crc = crate::gnss::unicore_crc32(&bad[..end]);
        bad[end..].copy_from_slice(&crc.to_le_bytes());
        let mut invalid = false;
        for byte in bad {
            invalid |= matches!(d.push(byte).unwrap(), Some(Frame::InvalidPps));
        }
        assert!(invalid);
    }

    #[test]
    fn native_spp_mixed_with_pps_and_ascii_never_searches_inside_payload() {
        let mut payload = [0; 120];
        payload[..4].copy_from_slice(&1u32.to_le_bytes());
        let embedded = b"$CONFIG,PPS,malicious-looking-payload\r\n";
        payload[8..8 + embedded.len()].copy_from_slice(embedded);
        payload[80..83].copy_from_slice(&SYNC);
        let spp = binary(46, &payload);
        let mut pps = [0; 60];
        pps[4..8].copy_from_slice(&2400u32.to_le_bytes());
        pps[8..12].copy_from_slice(&1000u32.to_le_bytes());
        let mut decoder = Decoder::<256>::new();
        let mut kinds = std::vec::Vec::new();
        let mut stream = spp;
        stream.extend(binary(9000, &pps));
        stream.extend(b"#SPPNAVA,legacy-support\r\n");
        for byte in stream {
            match decoder.push(byte).unwrap() {
                Some(Frame::SppNav(raw)) => {
                    assert_eq!(raw.position_status(), 1);
                    assert_eq!(raw.gps_week, 2400);
                    assert_eq!(raw.gps_tow_ms, 1000);
                    assert_eq!(raw.body, payload);
                    assert_eq!(raw.measurement(), Ok(None));
                    kinds.push(46);
                }
                Some(Frame::Pps(_)) => kinds.push(9000),
                Some(Frame::Line(line)) => {
                    assert_eq!(line, b"#SPPNAVA,legacy-support\r");
                    kinds.push(0);
                }
                Some(_) => panic!("unexpected invalid frame"),
                None => {}
            }
        }
        assert_eq!(kinds, [46, 9000, 0]);
    }

    #[test]
    fn native_spp_crc_length_status_and_no_fix_epoch_accounting() {
        let mut decoder = Decoder::<256>::new();
        for payload in [std::vec![0; 119], std::vec![0; 121]] {
            let mut invalid = 0;
            for byte in binary(46, &payload) {
                invalid += usize::from(matches!(
                    decoder.push(byte).unwrap(),
                    Some(Frame::InvalidSppNav)
                ));
            }
            assert_eq!(invalid, 1);
        }
        let mut payload = [0; 120];
        payload[72..76].copy_from_slice(&u32::MAX.to_le_bytes());
        let mut invalid = 0;
        for byte in binary(46, &payload) {
            invalid += usize::from(matches!(
                decoder.push(byte).unwrap(),
                Some(Frame::InvalidSppNav)
            ));
        }
        assert_eq!(invalid, 1);
        payload[72..76].copy_from_slice(&1u32.to_le_bytes());
        payload[..4].copy_from_slice(&1u32.to_le_bytes());
        let mut corrupted = binary(46, &payload);
        corrupted[24 + 8] ^= 0x80;
        let mut checksum_errors = 0;
        for byte in corrupted {
            match decoder.push(byte) {
                Err(DecodeError::Checksum) => checksum_errors += 1,
                Ok(None) => {}
                _ => panic!("corrupted frame must not emit a native record"),
            }
        }
        assert_eq!(checksum_errors, 1);
        let mut epochs = std::vec::Vec::new();
        for index in 0..50u32 {
            let mut packet = binary(46, &payload);
            packet[9] = 20; // No FINE time: retain raw receiver epochs, no mapping.
            packet[12..16].copy_from_slice(&(1000 + index * 20).to_le_bytes());
            let end = packet.len() - 4;
            let crc = crate::gnss::unicore_crc32(&packet[..end]);
            packet[end..].copy_from_slice(&crc.to_le_bytes());
            for byte in packet {
                if let Some(Frame::SppNav(raw)) = decoder.push(byte).unwrap() {
                    assert_eq!(raw.qualified_epoch_ms(), None);
                    assert_eq!(raw.measurement(), Ok(None));
                    epochs.push(raw.gps_tow_ms);
                }
            }
        }
        assert_eq!(epochs.len(), 50);
        assert!(epochs.windows(2).all(|pair| pair[1] - pair[0] == 20));
    }
}
