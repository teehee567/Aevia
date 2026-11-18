//! Fixed-capacity UTF-8 response with JSON serialized directly into its tail.
use core::fmt::{self, Write as _};
use serde::Serialize;

/// A single RFC 4648 stream spanning record boundaries. Only the unfinished
/// one or two source bytes remain here; payloads are never copied to scratch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Base64Records {
    carry: [u8; 2],
    len: u8,
    finished: bool,
}

impl Base64Records {
    pub const fn new() -> Self {
        Self {
            carry: [0; 2],
            len: 0,
            finished: false,
        }
    }
}

impl Default for Base64Records {
    fn default() -> Self {
        Self::new()
    }
}

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const HEX: &[u8; 16] = b"0123456789abcdef";

fn base64_triplet(a: u8, b: u8, c: u8) -> [u8; 4] {
    [
        BASE64[usize::from(a >> 2)],
        BASE64[usize::from(((a & 3) << 4) | (b >> 4))],
        BASE64[usize::from(((b & 15) << 2) | (c >> 6))],
        BASE64[usize::from(c & 63)],
    ]
}

/// Frame a nonempty, zero-free application reply using the Postcard layout
/// for variant 3 (a varint byte length followed by the UTF-8 text).
pub fn frame_app_reply(text: &str, buffer: &mut [u8]) -> Option<usize> {
    const COBS_DATA: usize = 254;
    let bytes = text.as_bytes();
    let len = bytes.len();
    if !(1..=0x3fff).contains(&len) || bytes.contains(&0) {
        return None;
    }
    let header = [
        3,
        len as u8 | if len >= 128 { 128 } else { 0 },
        (len >> 7) as u8,
    ];
    let header = &header[..if len < 128 { 2 } else { 3 }];
    let raw_len = header.len() + len;
    // Postcard emits a final code byte even after an exact 254-byte block.
    let frame_len = raw_len + raw_len / COBS_DATA + 2;
    let (delimiter, frame) = buffer.get_mut(..frame_len)?.split_last_mut()?;
    *delimiter = 0;
    let (first, rest) = frame.split_at_mut(frame.len().min(COBS_DATA + 1));
    first[0] = first.len() as u8;
    let (header_out, first_data) = first[1..].split_at_mut(header.len());
    header_out.copy_from_slice(header);
    let (prefix, mut remaining) = bytes.split_at(first_data.len());
    first_data.copy_from_slice(prefix);
    for block in rest.chunks_mut(COBS_DATA + 1) {
        block[0] = block.len() as u8;
        let (source, tail) = remaining.split_at(block.len() - 1);
        block[1..].copy_from_slice(source);
        remaining = tail;
    }
    Some(frame_len)
}

pub struct Response<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> Response<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn begin_compact(
        &mut self,
        recording: bool,
        session: u32,
        started: Option<u64>,
        stopped: Option<u64>,
    ) -> fmt::Result {
        write!(
            self,
            "{{\"ok\":true,\"protocol\":1,\"compact\":true,\"recording\":{recording},\"session\":{session},\"started_at_us\":"
        )?;
        self.serialize(&started).map_err(|_| fmt::Error)?;
        self.push_str(",\"stopped_at_us\":")?;
        self.serialize(&stopped).map_err(|_| fmt::Error)?;
        self.push_str(",\"samples\":[")
    }

    pub fn finish_compact(
        &mut self,
        pending: usize,
        dropped: u32,
        clock: impl FnOnce() -> u64,
    ) -> fmt::Result {
        // Read the anchor only after all samples were serialized. Producer tasks
        // can append newer samples while a compact batch yields between values.
        let now = clock();
        write!(
            self,
            "],\"pending\":{pending},\"dropped\":{dropped},\"uptime_us\":{now}}}"
        )
    }

    pub fn push_str(&mut self, value: &str) -> fmt::Result {
        if value.len() > N - self.len {
            return Err(fmt::Error);
        }
        let end = self.len + value.len();
        self.bytes[self.len..end].copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }

    /// Bounded binary record transport. The existing SD payload is preserved
    /// byte for byte, without float formatting or an intermediate hex buffer.
    pub fn record_hex(&mut self, kind: u8, payload: &[u8], reserved: usize) -> fmt::Result {
        let len = u16::try_from(payload.len()).map_err(|_| fmt::Error)?;
        let needed = payload
            .len()
            .checked_add(3)
            .and_then(|n| n.checked_mul(2))
            .ok_or(fmt::Error)?;
        if needed
            .checked_add(reserved)
            .is_none_or(|n| n > N - self.len)
        {
            return Err(fmt::Error);
        }
        let [low, high] = len.to_le_bytes();
        self.push_hex(&[kind, low, high])?;
        self.push_hex(payload)
    }

    pub fn push_hex(&mut self, bytes: &[u8]) -> fmt::Result {
        if bytes.len() > (N - self.len) / 2 {
            return Err(fmt::Error);
        }
        let end = self.len + 2 * bytes.len();
        let pairs = self.bytes[self.len..end].as_chunks_mut::<2>().0;
        for (pair, &byte) in pairs.iter_mut().zip(bytes) {
            *pair = [HEX[usize::from(byte >> 4)], HEX[usize::from(byte & 15)]];
        }
        self.len = end;
        Ok(())
    }

    /// Append [kind:u8, payload_len:u16LE, payload] to one standard base64
    /// stream. Reserve the caller's footer AND the eventual final padded
    /// quartet before changing either the output or the two-byte carry.
    pub fn record_base64(
        &mut self,
        encoder: &mut Base64Records,
        kind: u8,
        payload: &[u8],
        reserved: usize,
    ) -> fmt::Result {
        let len = u16::try_from(payload.len()).map_err(|_| fmt::Error)?;
        if encoder.finished {
            return Err(fmt::Error);
        }
        // payload fits u16, so this arithmetic is bounded on supported targets.
        let total = usize::from(encoder.len) + 3 + payload.len();
        let padded = total.div_ceil(3) * 4;
        if padded
            .checked_add(reserved)
            .is_none_or(|needed| needed > N - self.len)
        {
            return Err(fmt::Error);
        }
        let [low, high] = len.to_le_bytes();
        self.base64_bytes(encoder, &[kind, low, high]);
        self.base64_bytes(encoder, payload);
        Ok(())
    }

    /// Emit the stream's only padding and seal the encoder. Idempotent after
    /// success; on insufficient capacity both output and carry remain intact.
    pub fn finish_base64(&mut self, encoder: &mut Base64Records) -> fmt::Result {
        if encoder.finished {
            return Ok(());
        }
        if encoder.len != 0 {
            if N - self.len < 4 {
                return Err(fmt::Error);
            }
            let [a, b] = encoder.carry;
            self.bytes[self.len] = BASE64[usize::from(a >> 2)];
            self.bytes[self.len + 1] =
                BASE64[usize::from(((a & 3) << 4) | if encoder.len == 2 { b >> 4 } else { 0 })];
            self.bytes[self.len + 2] = if encoder.len == 2 {
                BASE64[usize::from((b & 15) << 2)]
            } else {
                b'='
            };
            self.bytes[self.len + 3] = b'=';
            self.len += 4;
        }
        encoder.len = 0;
        encoder.finished = true;
        Ok(())
    }

    /// Called only after the entire record and final padding have passed the
    /// fit check, so no partially written quartet can escape on an error.
    fn base64_bytes(&mut self, encoder: &mut Base64Records, mut bytes: &[u8]) {
        if encoder.len != 0 {
            let needed = 3 - usize::from(encoder.len);
            if bytes.len() < needed {
                if let Some(&byte) = bytes.first() {
                    encoder.carry[usize::from(encoder.len)] = byte;
                    encoder.len += 1;
                }
                return;
            }
            let (a, b, c) = if encoder.len == 1 {
                (encoder.carry[0], bytes[0], bytes[1])
            } else {
                (encoder.carry[0], encoder.carry[1], bytes[0])
            };
            self.bytes[self.len..self.len + 4].copy_from_slice(&base64_triplet(a, b, c));
            self.len += 4;
            encoder.len = 0;
            bytes = &bytes[needed..];
        }
        let (triplets, tail) = bytes.as_chunks::<3>();
        let end = self.len + triplets.len() * 4;
        let quads = self.bytes[self.len..end].as_chunks_mut::<4>().0;
        for (quad, &[a, b, c]) in quads.iter_mut().zip(triplets) {
            *quad = base64_triplet(a, b, c);
        }
        self.len = end;
        encoder.carry[..tail.len()].copy_from_slice(tail);
        encoder.len = tail.len() as u8;
    }

    pub fn push(&mut self, value: char) -> fmt::Result {
        self.push_str(value.encode_utf8(&mut [0; 4]))
    }

    pub fn as_str(&self) -> &str {
        // Only complete UTF-8 strings/chars and successful JSON serialization
        // enter the visible prefix. Keep the check at this public boundary.
        core::str::from_utf8(&self.bytes[..self.len]).expect("response UTF-8 invariant")
    }

    pub fn serialize<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), serde_json_core::ser::Error> {
        // On failure the serializer may modify unused bytes, but the committed
        // prefix remains unchanged and valid UTF-8. No scratch copy is needed.
        self.serialize_separated_reserved("", value, 0)
    }

    pub fn serialize_separated_reserved<T: Serialize + ?Sized>(
        &mut self,
        separator: &str,
        value: &T,
        reserved: usize,
    ) -> Result<(), serde_json_core::ser::Error> {
        let limit = N
            .checked_sub(reserved)
            .filter(|limit| *limit >= self.len)
            .ok_or(serde_json_core::ser::Error::BufferFull)?;
        let start = self
            .len
            .checked_add(separator.len())
            .filter(|start| *start <= limit)
            .ok_or(serde_json_core::ser::Error::BufferFull)?;
        // Separator and partially serialized bytes stay outside the committed
        // prefix until the entire value fits, including the caller's footer.
        self.bytes[self.len..start].copy_from_slice(separator.as_bytes());
        let appended = serde_json_core::to_slice(value, &mut self.bytes[start..limit])?;
        self.len = start + appended;
        Ok(())
    }
}

impl<const N: usize> Default for Response<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Write for Response<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        self.push_str(value)
    }

    fn write_char(&mut self, value: char) -> fmt::Result {
        self.push(value)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::fmt::Write;

    #[test]
    fn app_frames_match_postcard_length_and_cobs_block_boundaries() {
        for (length, header) in [
            (127, &[3, 127][..]),
            (128, &[3, 128, 1]),
            (250, &[3, 250, 1]),
            (251, &[3, 251, 1]),
            (252, &[3, 252, 1]),
            (505, &[3, 249, 3]),
            (8192, &[3, 128, 64]),
        ] {
            let text = "x".repeat(length);
            let mut buffer = [0xaa; 8256];
            let size = frame_app_reply(&text, &mut buffer).unwrap();
            assert_eq!(buffer[size - 1], 0);
            assert_eq!(buffer[size], 0xaa);

            let mut decoded = std::vec::Vec::new();
            for block in buffer[..size - 1].chunks(255) {
                assert_eq!(usize::from(block[0]), block.len());
                decoded.extend_from_slice(&block[1..]);
            }
            assert_eq!(&decoded[..header.len()], header);
            assert_eq!(&decoded[header.len()..], text.as_bytes());
            if matches!(length, 251 | 505) {
                assert_eq!(buffer[size - 2], 1);
            }
            assert_eq!(frame_app_reply(&text, &mut buffer[..size - 1]), None);
        }

        let mut buffer = [0xaa; 16];
        let size = frame_app_reply("é", &mut buffer).unwrap();
        assert_eq!(&buffer[..size], &[5, 3, 2, 0xc3, 0xa9, 0]);
        assert_eq!(frame_app_reply("\0", &mut buffer), None);
        assert_eq!(frame_app_reply("", &mut buffer), None);
    }

    // Independent bit-stream reference: no production triplet/carry helpers.
    fn reference_base64(bytes: &[u8]) -> std::string::String {
        let alphabet = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        let mut result = std::string::String::new();
        let mut accumulator = 0_u32;
        let mut bits = 0;
        for &byte in bytes {
            accumulator = (accumulator << 8) | u32::from(byte);
            bits += 8;
            while bits >= 6 {
                bits -= 6;
                result.push(char::from(alphabet[((accumulator >> bits) & 63) as usize]));
            }
            accumulator &= (1 << bits) - 1;
        }
        if bits != 0 {
            result.push(char::from(alphabet[(accumulator << (6 - bits)) as usize]));
        }
        while !result.len().is_multiple_of(4) {
            result.push('=');
        }
        result
    }

    fn reference_record(stream: &mut std::vec::Vec<u8>, kind: u8, payload: &[u8]) {
        stream.push(kind);
        stream.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        stream.extend_from_slice(payload);
    }

    #[test]
    fn base64_matches_rfc4648_vectors_across_every_small_chunk_boundary() {
        for (plain, encoded) in [
            ("", ""),
            ("f", "Zg=="),
            ("fo", "Zm8="),
            ("foo", "Zm9v"),
            ("foob", "Zm9vYg=="),
            ("fooba", "Zm9vYmE="),
            ("foobar", "Zm9vYmFy"),
        ] {
            assert_eq!(reference_base64(plain.as_bytes()), encoded);
            for chunk_size in 1..=4 {
                let mut response = Response::<32>::new();
                let mut encoder = Base64Records::new();
                for chunk in plain.as_bytes().chunks(chunk_size) {
                    response.base64_bytes(&mut encoder, chunk);
                }
                response.finish_base64(&mut encoder).unwrap();
                assert_eq!(response.as_str(), encoded);
                response.finish_base64(&mut encoder).unwrap();
                assert_eq!(response.as_str(), encoded);
                assert!(response.record_base64(&mut encoder, 23, &[], 0).is_err());
            }
        }
        assert_eq!(core::mem::size_of::<Base64Records>(), 4);
    }

    #[test]
    fn base64_mixed_real_imu_and_gnss_records_preserve_one_continuous_stream() {
        let imu = crate::sample::Sample::Imu {
            t_us: 1_234_567,
            dt_us: 1375,
            accel: [1.0, -2.0, 9.81],
            gyro: [0.1, -0.2, 0.3],
            valid: true,
            captured_ns: (1_u64 << 53) + 1,
            interval_ns: 1_375_000,
            effective_end_ns: (1_u64 << 53) - 100,
            timing_uncertainty_ns: 1200,
            aligned_gyro: [0.11, -0.22, 0.33],
        };
        let gnss = crate::sample::Sample::Gnss {
            t_us: 1_234_890,
            gps_week: 2400,
            gps_tow_ms: 123_456,
            lat: -33.0,
            lon: 151.0,
            alt_m: 34.0,
            speed_mps: 20.0,
            heading_deg: 120.0,
            vertical_mps: -0.3,
            hacc_m: 0.4,
            vacc_m: 0.5,
            sacc_mps: 0.6,
            latency_ms: 20,
            satellites: 17,
            valid: true,
            mapped_ns: Some((1_u64 << 53) + 17),
            mapping_uncertainty_ns: Some(500_000),
            mapping_generation: Some(7),
        };
        let mut response = Response::<8192>::new();
        let mut encoder = Base64Records::new();
        let mut expected = std::vec::Vec::new();
        for sample in [imu, imu, gnss, imu, gnss, gnss, imu] {
            let mut payload = [0; 128];
            let (kind, _, len) = crate::storage::codec::encode(sample, &mut payload);
            assert!(matches!((kind, len), (23, 101) | (24, 115)));
            response
                .record_base64(&mut encoder, kind, &payload[..len], 0)
                .unwrap();
            reference_record(&mut expected, kind, &payload[..len]);
            assert!(!response.as_str().contains('='));
        }
        response.finish_base64(&mut encoder).unwrap();
        assert_eq!(response.as_str(), reference_base64(&expected));
    }

    #[test]
    fn base64_worst_case_48_gnss_records_fit_with_399_byte_envelope() {
        let mut response = Response::<8192>::new();
        let mut encoder = Base64Records::new();
        response.push_str(&"p".repeat(199)).unwrap();
        let mut expected = std::vec::Vec::new();
        for record in 0..48 {
            let mut payload = [0; 115];
            for (index, byte) in payload.iter_mut().enumerate() {
                *byte = (index as u8).wrapping_mul(29).wrapping_add(record);
            }
            response
                .record_base64(&mut encoder, 24, &payload, 200)
                .unwrap();
            reference_record(&mut expected, 24, &payload);
        }
        response.finish_base64(&mut encoder).unwrap();
        assert_eq!(&response.as_str()[199..], reference_base64(&expected));
        response.push_str(&"s".repeat(200)).unwrap();
        assert_eq!(response.as_str().len(), 7552 + 399);
    }

    #[test]
    fn base64_failed_record_keeps_output_and_carry_and_reserves_final_padding() {
        let mut response = Response::<16>::new();
        let mut encoder = Base64Records::new();
        response.record_base64(&mut encoder, 23, &[1], 4).unwrap();
        assert_eq!(encoder.len, 1);
        let saved = encoder;
        let before = response.as_str().to_owned();
        for (payload, reserved) in [(&[2; 5][..], 4), (&[][..], usize::MAX)] {
            assert!(
                response
                    .record_base64(&mut encoder, 24, payload, reserved)
                    .is_err()
            );
            assert_eq!(response.as_str(), before);
            assert_eq!(encoder, saved);
        }
        assert!(
            response
                .record_base64(&mut encoder, 24, &std::vec![0; 65536], 0)
                .is_err()
        );
        assert_eq!(response.as_str(), before);
        assert_eq!(encoder, saved);
        response.record_base64(&mut encoder, 24, &[], 4).unwrap();
        response.finish_base64(&mut encoder).unwrap();
        assert_eq!(
            response.as_str(),
            reference_base64(&[23, 1, 0, 1, 24, 0, 0])
        );
        response.push_str("END!").unwrap();
        assert_eq!(response.as_str().len(), 16);

        // A caller consuming promised padding space must get an atomic failure.
        let mut response = Response::<8>::new();
        let mut encoder = Base64Records::new();
        response.record_base64(&mut encoder, 23, &[1], 0).unwrap();
        response.push('x').unwrap();
        let before = response.as_str().to_owned();
        let saved = encoder;
        assert!(response.finish_base64(&mut encoder).is_err());
        assert_eq!(response.as_str(), before);
        assert_eq!(encoder, saved);
    }

    #[test]
    fn base64_every_tail_and_two_byte_carry_rolls_back_at_exact_capacity() {
        for payload_len in 0..=128 {
            let payload: std::vec::Vec<u8> = (0..payload_len).map(|n| n as u8).collect();
            let mut response = Response::<256>::new();
            let mut encoder = Base64Records::new();
            let mut expected = std::vec::Vec::new();
            response
                .record_base64(&mut encoder, 255, &payload, 0)
                .unwrap();
            reference_record(&mut expected, 255, &payload);
            assert_eq!(usize::from(encoder.len), payload_len % 3);
            response.finish_base64(&mut encoder).unwrap();
            assert_eq!(response.as_str(), reference_base64(&expected));
        }
        let mut response = Response::<12>::new();
        let mut encoder = Base64Records::new();
        response
            .record_base64(&mut encoder, 23, &[0xff, 0xfe], 4)
            .unwrap();
        assert_eq!(encoder.len, 2);
        let saved = encoder;
        let before = response.as_str().to_owned();
        assert!(response.record_base64(&mut encoder, 24, &[], 4).is_err());
        assert_eq!(response.as_str(), before);
        assert_eq!(encoder, saved);
        response.finish_base64(&mut encoder).unwrap();
        assert_eq!(response.as_str(), reference_base64(&[23, 2, 0, 0xff, 0xfe]));
        response.push_str("END!").unwrap();
        assert_eq!(response.as_str().len(), 12);
    }

    #[test]
    fn binary_hex_record_is_exact_and_failed_append_is_atomic() {
        let mut response = Response::<32>::new();
        response.push('"').unwrap();
        response.record_hex(23, &[0, 0xff, 0x80, 1], 1).unwrap();
        assert_eq!(response.as_str(), "\"17040000ff8001");
        let before = response.as_str().to_owned();
        assert!(response.record_hex(24, &[0; 8], 1).is_err());
        assert_eq!(response.as_str(), before);
        response.push('"').unwrap();
        assert_eq!(response.as_str(), "\"17040000ff8001\"");
    }

    #[test]
    fn sector_hex_matches_two_digit_lowercase_encoding() {
        let bytes: [u8; 512] = core::array::from_fn(|i| i as u8);
        let expected: std::string::String = bytes.iter().map(|b| std::format!("{b:02x}")).collect();
        let mut response = Response::<1026>::new();
        response.push('"').unwrap();
        response.push_hex(&bytes).unwrap();
        assert!(response.push_hex(&[0]).is_err());
        response.push('"').unwrap();
        assert_eq!(&response.as_str()[1..1025], expected);
    }

    #[test]
    fn compact_clock_anchor_is_read_after_all_yield_time_samples() {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct Stamp {
            t_us: u64,
        }
        #[derive(serde::Deserialize)]
        struct Batch {
            samples: [Stamp; 2],
            uptime_us: u64,
        }
        let clock = core::cell::Cell::new(10_u64);
        let mut response = Response::<512>::new();
        response.begin_compact(true, 7, Some(5), None).unwrap();
        for (index, time) in [20, 35].into_iter().enumerate() {
            clock.set(time); // A producer runs during the builder's yield.
            response
                .serialize_separated_reserved(
                    if index == 0 { "" } else { "," },
                    &Stamp { t_us: clock.get() },
                    128,
                )
                .unwrap();
        }
        clock.set(50);
        response.finish_compact(3, 4, || clock.get()).unwrap();
        let (batch, _) = serde_json_core::from_str::<Batch>(response.as_str()).unwrap();
        assert_eq!(batch.uptime_us, 50);
        assert!(
            batch
                .samples
                .iter()
                .all(|sample| sample.t_us <= batch.uptime_us)
        );
        assert_eq!(response.as_str().matches("\"uptime_us\":").count(), 1);
    }

    #[test]
    fn bounded_batch_reserves_footer_and_failed_value_does_not_commit_comma() {
        let mut response = Response::<12>::new();
        response.push_str("{\"v\":").unwrap();
        response
            .serialize_separated_reserved("", "abcd", 1)
            .unwrap();
        response.push('}').unwrap();
        assert_eq!(response.as_str(), "{\"v\":\"abcd\"}");

        let mut response = Response::<7>::new();
        response.push('[').unwrap();
        response.serialize_separated_reserved("", &12, 1).unwrap();
        assert!(response.serialize_separated_reserved(",", &999, 1).is_err());
        assert_eq!(response.as_str(), "[12");
        assert!(
            response
                .serialize_separated_reserved(",", &0, usize::MAX)
                .is_err()
        );
        assert_eq!(response.as_str(), "[12");
        response.push(']').unwrap();
        assert_eq!(response.as_str(), "[12]");
    }

    #[test]
    fn telemetry_wire_matches_the_previous_temporary_buffer_path() {
        let sample = crate::sample::Sample::Imu {
            t_us: 123_456,
            dt_us: 5_500,
            accel: [0.0, -0.0, 9.81],
            gyro: [1e-12, -1.25, 2.5],
            valid: true,
            captured_ns: 123_456_789,
            interval_ns: 5_500_000,
            effective_end_ns: 121_956_789,
            timing_uncertainty_ns: 1_200_000,
            aligned_gyro: [1e-12, -1.2, 2.4],
        };
        let mut old_bytes = [0; 1536];
        let old_len = serde_json_core::to_slice(&sample, &mut old_bytes).unwrap();
        let old_json = core::str::from_utf8(&old_bytes[..old_len]).unwrap();
        let expected = std::format!("{{\"ok\":true,\"samples\":[{old_json}]}}");

        let mut response = Response::<2048>::new();
        response.push_str("{\"ok\":true,\"samples\":[").unwrap();
        response.serialize(&sample).unwrap();
        response.push_str("]}").unwrap();
        assert_eq!(response.as_str(), expected);
    }

    #[test]
    fn json_preserves_unicode_escaping_and_appends_without_a_scratch_limit() {
        let mut response = Response::<2048>::new();
        response.push('[').unwrap();
        response.serialize("é🦀\n\"\\").unwrap();
        response.push(',').unwrap();
        response.serialize(&Option::<u8>::None).unwrap();
        response.push(']').unwrap();
        assert_eq!(response.as_str(), "[\"é🦀\\n\\\"\\\\\",null]");

        response.clear();
        let large = "x".repeat(1800);
        response.serialize(large.as_str()).unwrap();
        assert_eq!(response.as_str(), std::format!("\"{large}\""));
    }

    #[test]
    fn exact_capacity_and_failed_appends_preserve_utf8() {
        let mut response = Response::<8>::new();
        response.push_str("Aé").unwrap();
        response.push('🦀').unwrap();
        response.serialize(&1).unwrap();
        assert_eq!(response.as_str(), "Aé🦀1");
        assert!(response.push('é').is_err());
        assert!(response.push_str("x").is_err());
        assert_eq!(response.as_str(), "Aé🦀1");
        response.clear();
        response.serialize("é").unwrap();
        assert_eq!(response.as_str(), "\"é\"");

        let mut empty = Response::<0>::new();
        empty.push_str("").unwrap();
        assert!(empty.serialize(&()).is_err());
        assert!(empty.push('x').is_err());
        assert_eq!(empty.as_str(), "");
    }

    #[test]
    fn a_partial_json_error_never_commits_or_leaks_its_tail() {
        let mut response = Response::<7>::new();
        response.push_str("ok").unwrap();
        // The opening quote and character fit; the closing quote does not.
        assert_eq!(
            response.serialize("🦀"),
            Err(serde_json_core::ser::Error::BufferFull)
        );
        assert_eq!(response.as_str(), "ok");
        response.push_str("done").unwrap();
        assert_eq!(response.as_str(), "okdone");
        response.clear();
        response.serialize(&true).unwrap();
        assert_eq!(response.as_str(), "true");
    }

    #[test]
    fn formatting_is_bounded_and_serializable_errors_preserve_the_prefix() {
        struct OversizedAfterElement;
        impl Serialize for OversizedAfterElement {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                use serde::ser::SerializeSeq;
                let mut sequence = serializer.serialize_seq(Some(2))?;
                sequence.serialize_element("first")?;
                sequence.serialize_element(
                    "this second element cannot fit into the bounded output remaining after the first",
                )?;
                sequence.end()
            }
        }
        let mut response = Response::<64>::new();
        write!(&mut response, "session={}:{}", 7, 'é').unwrap();
        assert_eq!(response.as_str(), "session=7:é");
        assert!(response.serialize(&OversizedAfterElement).is_err());
        assert_eq!(response.as_str(), "session=7:é");
        response.push('!').unwrap();
        assert_eq!(response.as_str(), "session=7:é!");

        let mut small = Response::<3>::new();
        assert!(write!(&mut small, "{}:{}", "ok", 42).is_err());
        assert!(small.as_str().len() <= 3);
    }
}
