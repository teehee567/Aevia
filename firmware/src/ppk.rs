//! Bounded, integer-only capture of native Unicore observation/ephemeris frames.
//!
//! N4 frames have a 24-byte header and a little-endian CRC32 trailer. We retain
//! the original bytes, including bad-CRC frames, in kind-25 record chunks. A
//! consumer must reassemble and check the whole frame before using observations;
//! intermediate chunks have no independent receiver integrity guarantee.
//!
//! Call `push` for every UART byte, even outside recording. Sequence numbers then
//! reveal frames lost by downstream queues. `received_us` is the host reception
//! time of the first sync byte, not the receiver measurement epoch. A truncated
//! frame needs an explicit reset after a transport gap: payloads are opaque and
//! are never searched for an apparent ASCII line or another binary header.

pub const CHUNK_HEADER_LEN: usize = 24;
pub const CHUNK_RAW_CAPACITY: usize = 232;
pub const CHUNK_CAPACITY: usize = CHUNK_HEADER_LEN + CHUNK_RAW_CAPACITY;
pub const MAX_FRAME_LEN: u32 = 24 + u16::MAX as u32 + 4;
const SYNC: [u8; 3] = [0xaa, 0x44, 0xb5];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chunk {
    bytes: [u8; CHUNK_CAPACITY],
    len: u16,
}

impl Chunk {
    /// Exact kind-25 payload: schema=1, native-Unicore format=1, flags, zero,
    /// sequence:u32, frame length:u32, offset:u32, reception time:u64, raw bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }

    pub fn raw_bytes(&self) -> &[u8] {
        &self.bytes[CHUNK_HEADER_LEN..usize::from(self.len)]
    }

    pub fn frame_seq(&self) -> u32 {
        u32::from_le_bytes(self.bytes[4..8].try_into().unwrap())
    }

    pub fn total_len(&self) -> u32 {
        u32::from_le_bytes(self.bytes[8..12].try_into().unwrap())
    }

    pub fn offset(&self) -> u32 {
        u32::from_le_bytes(self.bytes[12..16].try_into().unwrap())
    }

    pub fn received_us(&self) -> u64 {
        u64::from_le_bytes(self.bytes[16..24].try_into().unwrap())
    }

    pub fn first(&self) -> bool {
        self.bytes[2] & 1 != 0
    }

    pub fn last(&self) -> bool {
        self.bytes[2] & 2 != 0
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(serde::Serialize))]
pub struct Diagnostics {
    /// Tracked frames completed with a correct receiver CRC (not a fix claim).
    pub frames: u32,
    pub observation_frames: u32,
    pub ephemeris_frames: u32,
    /// Tracked message IDs identified, including frames later truncated.
    pub frames_started: u32,
    /// Tracked frames received through their final CRC byte, including failures.
    pub frames_completed: u32,
    pub crc_failures: u32,
    /// Received bytes belonging to tracked frames, including partial frames.
    pub bytes: u64,
    pub chunks: u32,
    /// Complete frames whose message IDs were not selected for PPK capture.
    pub skipped_frames: u32,
    /// Explicit transport resets, even if the parser was already idle.
    pub resets: u32,
    /// Binary candidates interrupted by a reset, including undecided headers.
    pub truncated_frames: u32,
    pub truncated_tracked_frames: u32,
    /// First-byte reception times of the most recent CRC-valid selected frames.
    pub last_received_us: Option<u64>,
    pub last_observation_us: Option<u64>,
    /// OBSVM's reported count, not a claim that each observation is usable.
    pub last_observation_count: u32,
    pub last_message_id: u16,
    /// Largest selected frame length declared by a complete header.
    pub maximum_frame_bytes: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Search,
    Ascii,
    Binary,
}

pub struct Decoder {
    raw: [u8; CHUNK_RAW_CAPACITY],
    raw_len: usize,
    mode: Mode,
    position: u32,
    total: u32,
    tracked: bool,
    received_us: u64,
    crc: u32,
    expected_crc: u32,
    frame_seq: u32,
    next_seq: u32,
    message_id: u16,
    observation_count: u32,
    diagnostics: Diagnostics,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            raw: [0; CHUNK_RAW_CAPACITY],
            raw_len: 0,
            mode: Mode::Search,
            position: 0,
            total: 0,
            tracked: false,
            received_us: 0,
            crc: 0,
            expected_crc: 0,
            frame_seq: 0,
            next_seq: 0,
            message_id: 0,
            observation_count: 0,
            diagnostics: Diagnostics {
                frames: 0,
                observation_frames: 0,
                ephemeris_frames: 0,
                frames_started: 0,
                frames_completed: 0,
                crc_failures: 0,
                bytes: 0,
                chunks: 0,
                skipped_frames: 0,
                resets: 0,
                truncated_frames: 0,
                truncated_tracked_frames: 0,
                last_received_us: None,
                last_observation_us: None,
                last_observation_count: 0,
                last_message_id: 0,
                maximum_frame_bytes: 0,
            },
        }
    }

    pub const fn diagnostics(&self) -> Diagnostics {
        self.diagnostics
    }

    /// Includes an undecided header from its first sync byte. Once its ID is
    /// known to be unrelated, that frame no longer blocks a recording stop.
    pub fn inflight_received_us(&self) -> Option<u64> {
        (self.mode == Mode::Binary && (self.position < 6 || self.tracked))
            .then_some(self.received_us)
    }

    /// Abandon a partial frame only on a known transport discontinuity/timeout.
    /// Already emitted chunks remain evidence of the incomplete frame; a later
    /// frame receives a new sequence number. No truncated LAST chunk is emitted.
    pub fn reset(&mut self) {
        self.diagnostics.resets = self.diagnostics.resets.saturating_add(1);
        if self.mode == Mode::Binary {
            self.diagnostics.truncated_frames = self.diagnostics.truncated_frames.saturating_add(1);
            if self.tracked {
                self.diagnostics.truncated_tracked_frames =
                    self.diagnostics.truncated_tracked_frames.saturating_add(1);
            }
        }
        self.clear_frame();
    }

    pub fn push(&mut self, byte: u8, now_us: u64) -> Option<Chunk> {
        if self.mode == Mode::Ascii {
            if byte == b'\n' {
                self.mode = Mode::Search;
            }
            return None;
        }
        if self.mode == Mode::Search {
            self.search(byte, now_us);
            return None;
        }
        if self.position < 3 && byte != SYNC[self.position as usize] {
            self.clear_frame();
            self.search(byte, now_us);
            return None;
        }

        // The first eight header bytes fit even before message selection. Once
        // selected, raw_len only counts the current bounded output chunk.
        if self.position < 8 || self.tracked {
            self.raw[self.raw_len] = byte;
            self.raw_len += 1;
        }
        if self.total == 0 || self.position < self.total - 4 {
            if self.position < 8 || self.tracked {
                self.crc = crc_byte(self.crc, byte);
            }
        } else if self.tracked {
            self.expected_crc |= u32::from(byte) << ((self.position - (self.total - 4)) * 8);
        }
        self.position += 1;

        if self.position == 6 {
            self.message_id = u16::from_le_bytes([self.raw[4], self.raw[5]]);
            self.tracked = tracked_id(self.message_id);
            if self.tracked {
                self.frame_seq = self.next_seq;
                self.next_seq = self.next_seq.wrapping_add(1);
                self.diagnostics.frames_started = self.diagnostics.frames_started.saturating_add(1);
                self.diagnostics.bytes = self.diagnostics.bytes.saturating_add(6);
            }
        } else if self.tracked {
            self.diagnostics.bytes = self.diagnostics.bytes.saturating_add(1);
        }
        if self.position == 8 {
            self.total = 28 + u32::from(u16::from_le_bytes([self.raw[6], self.raw[7]]));
            if self.tracked {
                self.diagnostics.maximum_frame_bytes =
                    self.diagnostics.maximum_frame_bytes.max(self.total);
            }
        }
        if self.message_id == 12 && self.position == 28 && self.total >= 32 {
            self.observation_count = u32::from_le_bytes(self.raw[24..28].try_into().unwrap());
        }

        let last = self.total != 0 && self.position == self.total;
        if last {
            if self.tracked {
                self.diagnostics.frames_completed =
                    self.diagnostics.frames_completed.saturating_add(1);
                if self.crc != self.expected_crc {
                    self.diagnostics.crc_failures = self.diagnostics.crc_failures.saturating_add(1);
                } else {
                    self.diagnostics.frames = self.diagnostics.frames.saturating_add(1);
                    self.diagnostics.last_received_us = Some(self.received_us);
                    self.diagnostics.last_message_id = self.message_id;
                    if self.message_id == 12 {
                        self.diagnostics.observation_frames =
                            self.diagnostics.observation_frames.saturating_add(1);
                        self.diagnostics.last_observation_us = Some(self.received_us);
                        self.diagnostics.last_observation_count = self.observation_count;
                    } else {
                        self.diagnostics.ephemeris_frames =
                            self.diagnostics.ephemeris_frames.saturating_add(1);
                    }
                }
            } else {
                self.diagnostics.skipped_frames = self.diagnostics.skipped_frames.saturating_add(1);
            }
        }
        let chunk = if self.tracked && (self.raw_len == CHUNK_RAW_CAPACITY || last) {
            Some(self.emit(last))
        } else {
            None
        };
        if last {
            self.clear_frame();
        }
        chunk
    }

    fn search(&mut self, byte: u8, now_us: u64) {
        if byte == SYNC[0] {
            self.mode = Mode::Binary;
            self.received_us = now_us;
            self.raw[0] = byte;
            self.raw_len = 1;
            self.position = 1;
            self.crc = crc_byte(0, byte);
        } else if matches!(byte, b'#' | b'$' | b'!') {
            self.mode = Mode::Ascii;
        }
    }

    fn clear_frame(&mut self) {
        self.mode = Mode::Search;
        self.raw_len = 0;
        self.position = 0;
        self.total = 0;
        self.tracked = false;
        self.crc = 0;
        self.expected_crc = 0;
        self.message_id = 0;
        self.observation_count = 0;
    }

    fn emit(&mut self, last: bool) -> Chunk {
        let offset = self.position - self.raw_len as u32;
        let mut bytes = [0; CHUNK_CAPACITY];
        bytes[0] = 1;
        bytes[1] = 1;
        bytes[2] = u8::from(offset == 0) | (u8::from(last) << 1);
        bytes[4..8].copy_from_slice(&self.frame_seq.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.total.to_le_bytes());
        bytes[12..16].copy_from_slice(&offset.to_le_bytes());
        bytes[16..24].copy_from_slice(&self.received_us.to_le_bytes());
        bytes[24..24 + self.raw_len].copy_from_slice(&self.raw[..self.raw_len]);
        let len = (CHUNK_HEADER_LEN + self.raw_len) as u16;
        self.raw_len = 0;
        self.diagnostics.chunks = self.diagnostics.chunks.saturating_add(1);
        Chunk { bytes, len }
    }
}

/// OBSVM and GPS/GLO/BDS/Galileo/QZSS/NavIC/BDS3 ephemerides. Compressed
/// observations are intentionally absent: the configured profile is OBSVMB.
pub const fn tracked_id(id: u16) -> bool {
    matches!(id, 12 | 106..=110 | 112 | 2999)
}

// Four-bit lookup keeps the per-byte work bounded without a 1 KiB table. The
// Unicore CRC starts at zero and has no final XOR (unlike ethernet CRC32).
fn crc_byte(mut crc: u32, byte: u8) -> u32 {
    const TABLE: [u32; 16] = [
        0x0000_0000,
        0x1db7_1064,
        0x3b6e_20c8,
        0x26d9_30ac,
        0x76dc_4190,
        0x6b6b_51f4,
        0x4db2_6158,
        0x5005_713c,
        0xedb8_8320,
        0xf00f_9344,
        0xd6d6_a3e8,
        0xcb61_b38c,
        0x9b64_c2b0,
        0x86d3_d2d4,
        0xa00a_e278,
        0xbdbd_f21c,
    ];
    crc ^= u32::from(byte);
    crc = (crc >> 4) ^ TABLE[(crc & 15) as usize];
    (crc >> 4) ^ TABLE[(crc & 15) as usize]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn frame(id: u16, payload: &[u8]) -> Vec<u8> {
        let mut bytes = std::vec![0; 24];
        bytes[..3].copy_from_slice(&SYNC);
        bytes[4..6].copy_from_slice(&id.to_le_bytes());
        bytes[6..8].copy_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
        bytes[9] = 160;
        bytes[10..12].copy_from_slice(&2400u16.to_le_bytes());
        bytes[12..16].copy_from_slice(&123_400u32.to_le_bytes());
        bytes.extend_from_slice(payload);
        // Independent bitwise implementation already used by the ASCII parser.
        bytes.extend_from_slice(&crate::gnss::unicore_crc32(&bytes).to_le_bytes());
        bytes
    }

    fn feed(decoder: &mut Decoder, bytes: &[u8], start_us: u64) -> Vec<Chunk> {
        bytes
            .iter()
            .enumerate()
            .filter_map(|(i, &byte)| decoder.push(byte, start_us + i as u64))
            .collect()
    }

    fn reassemble(chunks: &[Chunk], expected: &[u8], seq: u32, start_us: u64) {
        assert!(!chunks.is_empty());
        let mut raw = Vec::new();
        for (i, chunk) in chunks.iter().enumerate() {
            assert_eq!(chunk.as_bytes()[..2], [1, 1]);
            assert_eq!(chunk.as_bytes()[3], 0);
            assert_eq!(chunk.as_bytes()[2] & !3, 0);
            assert!(chunk.as_bytes().len() <= CHUNK_CAPACITY);
            assert!(!chunk.raw_bytes().is_empty());
            assert_eq!(chunk.frame_seq(), seq);
            assert_eq!(chunk.total_len() as usize, expected.len());
            assert_eq!(chunk.offset() as usize, raw.len());
            assert_eq!(chunk.received_us(), start_us);
            assert_eq!(chunk.first(), i == 0);
            assert_eq!(chunk.last(), i + 1 == chunks.len());
            raw.extend_from_slice(chunk.raw_bytes());
        }
        assert_eq!(raw, expected);
    }

    #[test]
    fn crc_matches_known_zero_seed_and_bitwise_reference() {
        let crc = b"123456789".iter().fold(0, |c, &b| crc_byte(c, b));
        assert_eq!(crc, 0x2dfd_2d88);
        for seed in [0, 1, u32::MAX, 0x1234_5678] {
            for byte in 0..=u8::MAX {
                let mut expected = seed ^ u32::from(byte);
                for _ in 0..8 {
                    expected = (expected >> 1) ^ (0xedb8_8320 * (expected & 1));
                }
                assert_eq!(crc_byte(seed, byte), expected);
            }
        }
    }

    #[test]
    fn all_selected_ids_preserve_raw_header_payload_and_crc() {
        let mut decoder = Decoder::new();
        for (seq, id) in [12, 106, 107, 108, 109, 110, 112, 2999]
            .into_iter()
            .enumerate()
        {
            let bytes = frame(id, b"#CONFIG\r\n\xaa\x44\xb5\0 raw evidence");
            let chunks = feed(&mut decoder, &bytes, 500 + seq as u64 * 100);
            reassemble(&chunks, &bytes, seq as u32, 500 + seq as u64 * 100);
            assert_eq!(decoder.inflight_received_us(), None);
        }
        let d = decoder.diagnostics();
        assert_eq!(d.frames_started, 8);
        assert_eq!(d.frames_completed, 8);
        assert_eq!(d.crc_failures, 0);
        assert_eq!(d.chunks, 8);
        assert_eq!(d.resets, 0);
    }

    #[test]
    fn chunk_boundaries_and_maximum_u16_payload_use_constant_memory() {
        assert!(core::mem::size_of::<Decoder>() < 512);
        for length in [0, 203, 204, 205, 436, 437, 1000, u16::MAX as usize] {
            let payload: Vec<u8> = (0..length).map(|i| (i % 256) as u8).collect();
            let bytes = frame(12, &payload);
            let mut decoder = Decoder::new();
            let chunks = feed(&mut decoder, &bytes, 123);
            reassemble(&chunks, &bytes, 0, 123);
            assert_eq!(chunks.len(), bytes.len().div_ceil(CHUNK_RAW_CAPACITY));
            assert_eq!(decoder.diagnostics().bytes, bytes.len() as u64);
            assert_eq!(decoder.diagnostics().crc_failures, 0);
            assert!(bytes.len() as u32 <= MAX_FRAME_LEN);
        }
    }

    #[test]
    fn unknown_binary_frames_hide_nested_logs_and_selected_frames() {
        let nested = frame(12, b"nested");
        let mut payload = b"#bogus\n!ack\r\n".to_vec();
        payload.extend_from_slice(&nested);
        payload.resize(2048, 0xaa);
        let mut decoder = Decoder::new();
        assert!(feed(&mut decoder, &frame(9000, &payload), 10).is_empty());
        let actual = frame(106, b"actual");
        let chunks = feed(&mut decoder, &actual, 3000);
        reassemble(&chunks, &actual, 0, 3000);
        assert_eq!(decoder.diagnostics().skipped_frames, 1);
        assert_eq!(decoder.diagnostics().frames_started, 1);
    }

    #[test]
    fn ascii_lines_never_publish_embedded_binary_candidates() {
        let nested = frame(12, &[1; 260]);
        // Pick a fixture whose trailer cannot end the intentionally malformed
        // ASCII line before the complete embedded frame has been consumed.
        assert!(!nested.contains(&b'\n'));
        for prefix in *b"#$!" {
            let mut decoder = Decoder::new();
            assert!(decoder.push(prefix, 1).is_none());
            assert!(feed(&mut decoder, &nested, 2).is_empty());
            assert_eq!(decoder.inflight_received_us(), None);
            assert!(decoder.push(b'\n', 1000).is_none());
            let actual = frame(12, b"after line");
            reassemble(&feed(&mut decoder, &actual, 2000), &actual, 0, 2000);
        }
    }

    #[test]
    fn corrupt_frames_are_preserved_and_marked_in_diagnostics() {
        let mut bytes = frame(12, &[7; 600]);
        bytes[400] ^= 1;
        let mut decoder = Decoder::new();
        let chunks = feed(&mut decoder, &bytes, 300);
        reassemble(&chunks, &bytes, 0, 300);
        assert_eq!(decoder.diagnostics().frames_completed, 1);
        assert_eq!(decoder.diagnostics().crc_failures, 1);
        let good = frame(108, b"next");
        reassemble(&feed(&mut decoder, &good, 2000), &good, 1, 2000);
        assert_eq!(decoder.diagnostics().crc_failures, 1);
    }

    #[test]
    fn payload_metadata_has_exact_kind25_little_endian_layout() {
        let bytes = frame(12, &[0; 4]);
        let mut decoder = Decoder::new();
        decoder.next_seq = 0x1234_5678;
        let time = 0x0123_4567_89ab_cdef;
        let chunks = feed(&mut decoder, &bytes, time);
        assert_eq!(chunks.len(), 1);
        assert_eq!(
            &chunks[0].as_bytes()[..24],
            &[
                1, 1, 3, 0, 0x78, 0x56, 0x34, 0x12, 32, 0, 0, 0, 0, 0, 0, 0, 0xef, 0xcd, 0xab,
                0x89, 0x67, 0x45, 0x23, 0x01,
            ]
        );
        assert_eq!(&chunks[0].as_bytes()[24..], &bytes);
    }

    #[test]
    fn observation_diagnostics_only_advance_after_a_valid_complete_crc() {
        let mut payload = std::vec![0; 4 + 40 * 123];
        payload[..4].copy_from_slice(&123u32.to_le_bytes());
        let bytes = frame(12, &payload);
        let mut decoder = Decoder::new();
        feed(&mut decoder, &bytes[..bytes.len() - 1], 100);
        assert_eq!(decoder.diagnostics().last_observation_us, None);
        assert_eq!(decoder.diagnostics().frames, 0);
        assert!(decoder.push(*bytes.last().unwrap(), 10_000).is_some());
        let valid = decoder.diagnostics();
        assert_eq!(valid.frames, 1);
        assert_eq!(valid.observation_frames, 1);
        assert_eq!(valid.ephemeris_frames, 0);
        assert_eq!(valid.last_observation_count, 123);
        assert_eq!(valid.last_message_id, 12);
        assert_eq!(valid.last_received_us, Some(100));
        assert_eq!(valid.last_observation_us, Some(100));
        assert_eq!(valid.maximum_frame_bytes, bytes.len() as u32);

        let mut corrupted = bytes.clone();
        corrupted[24] = 255;
        feed(&mut decoder, &corrupted, 20_000);
        let invalid = decoder.diagnostics();
        assert_eq!(invalid.frames, 1);
        assert_eq!(invalid.frames_completed, 2);
        assert_eq!(invalid.crc_failures, 1);
        assert_eq!(invalid.last_observation_count, 123);
        assert_eq!(invalid.last_observation_us, Some(100));

        feed(&mut decoder, &frame(106, b"ephemeris"), 30_000);
        let eph = decoder.diagnostics();
        assert_eq!(eph.frames, 2);
        assert_eq!(eph.ephemeris_frames, 1);
        assert_eq!(eph.last_received_us, Some(30_000));
        assert_eq!(eph.last_observation_us, Some(100));
    }

    #[test]
    fn stop_fence_includes_undecided_header_and_excludes_known_untracked() {
        for id in [12, 9000] {
            let bytes = frame(id, b"body");
            let mut decoder = Decoder::new();
            for (i, byte) in bytes.iter().take(5).enumerate() {
                assert!(decoder.push(*byte, 700 + i as u64).is_none());
                assert_eq!(decoder.inflight_received_us(), Some(700));
            }
            assert!(decoder.push(bytes[5], 705).is_none());
            assert_eq!(decoder.inflight_received_us(), (id == 12).then_some(700));
            feed(&mut decoder, &bytes[6..], 706);
            assert_eq!(decoder.inflight_received_us(), None);
        }
    }

    #[test]
    fn reset_preserves_diagnostics_and_sequence_gap_for_truncated_frame() {
        let partial = frame(12, &[1; 1000]);
        let mut decoder = Decoder::new();
        let chunks = feed(&mut decoder, &partial[..500], 0);
        assert_eq!(chunks.len(), 2);
        assert!(!chunks[1].last());
        assert_eq!(decoder.inflight_received_us(), Some(0));
        decoder.reset();
        assert_eq!(decoder.inflight_received_us(), None);
        let next = frame(109, b"after transport discontinuity");
        reassemble(&feed(&mut decoder, &next, 1000), &next, 1, 1000);
        let d = decoder.diagnostics();
        assert_eq!(d.resets, 1);
        assert_eq!(d.truncated_frames, 1);
        assert_eq!(d.truncated_tracked_frames, 1);
        assert_eq!(d.frames_started, 2);
        assert_eq!(d.frames_completed, 1);
        assert_eq!(d.bytes, 500 + next.len() as u64);
    }

    #[test]
    fn overlapping_sync_uses_last_candidate_time_and_sequence_wraps() {
        let mut decoder = Decoder::new();
        decoder.next_seq = u32::MAX;
        assert!(decoder.push(0xaa, 10).is_none());
        assert!(decoder.push(0x44, 11).is_none());
        let bytes = frame(12, b"good");
        reassemble(&feed(&mut decoder, &bytes, 20), &bytes, u32::MAX, 20);
        reassemble(&feed(&mut decoder, &bytes, 30), &bytes, 0, 30);
        assert_eq!(decoder.diagnostics().frames_started, 2);
        assert_eq!(decoder.diagnostics().truncated_frames, 0);
    }
}
