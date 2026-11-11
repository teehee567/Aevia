//! Native UART capture and observed optional-log capability evidence.
use aevia_device_hal::um980::Header;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct RawStatus {
    pub generation: u32,
    pub observation_frames: u32,
    pub advancing_epochs: u32,
    pub last_observation_us: Option<u64>,
    pub last_epoch_ms: Option<u64>,
    /// GPSEPH, GLOEPH, BDSEPH, GALEPH, QZSSEPH in bits 0..4.
    pub ephemeris_mask: u8,
    pub ephemeris_frames: u32,
    pub optional_write_failures: u32,
    // An in-progress frame at the file boundary contains bytes from the
    // preceding generation. Preserve framing but do not credit its completion.
    partial_from_previous: [bool; 2],
}

impl RawStatus {
    pub fn begin_generation(&mut self, generation: u32, partial: [bool; 2]) -> bool {
        if self.generation == generation {
            return false;
        }
        *self = Self {
            generation,
            partial_from_previous: partial,
            ..Self::default()
        };
        true
    }

    /// Called on a complete frame or framing failure on this port. False means
    /// this frame started before the current file's bootstrap generation.
    pub fn complete_frame(&mut self, port: usize) -> bool {
        !core::mem::replace(&mut self.partial_from_previous[port], false)
    }

    pub fn observe(&mut self, header: Header, received_us: u64) {
        if header.message_id == 12 {
            self.observation_frames = self.observation_frames.saturating_add(1);
            self.last_observation_us = Some(received_us);
            if let Some(epoch) = header.gps_epoch_ms()
                && self.last_epoch_ms.is_none_or(|old| epoch > old)
            {
                self.advancing_epochs = self.advancing_epochs.saturating_add(1);
                self.last_epoch_ms = Some(epoch);
            }
        }
        if let Some(bit) = header
            .message_id
            .checked_sub(106)
            .filter(|value| *value < 5)
        {
            self.ephemeris_mask |= 1 << bit;
            self.ephemeris_frames = self.ephemeris_frames.saturating_add(1);
        }
    }

    pub fn payload(self) -> [u8; 40] {
        let mut bytes = [0; 40];
        bytes[0] = 1;
        bytes[1] = self.ephemeris_mask;
        bytes[2] = u8::from(self.last_observation_us.is_some())
            | (u8::from(self.last_epoch_ms.is_some()) << 1);
        bytes[4..8].copy_from_slice(&self.observation_frames.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.advancing_epochs.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.ephemeris_frames.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.optional_write_failures.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.generation.to_le_bytes());
        bytes[24..32].copy_from_slice(&self.last_observation_us.unwrap_or(0).to_le_bytes());
        bytes[32..40].copy_from_slice(&self.last_epoch_ms.unwrap_or(0).to_le_bytes());
        bytes
    }
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::*;
    use core::cell::Cell;
    use core::sync::atomic::{AtomicU32, Ordering};
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
    static OFFSETS: Mutex<CriticalSectionRawMutex, Cell<[u64; 3]>> = Mutex::new(Cell::new([0; 3]));
    static STATUS: Mutex<CriticalSectionRawMutex, Cell<Option<RawStatus>>> =
        Mutex::new(Cell::new(None));
    static BOOTSTRAP_GENERATION: AtomicU32 = AtomicU32::new(0);
    /// Call after enabling the capture queue for a new session. Acquisition
    /// performs the paced command writes; the SD worker never touches its UART.
    pub(crate) fn request_bootstrap() -> u32 {
        BOOTSTRAP_GENERATION
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1)
    }
    pub(crate) fn bootstrap_generation() -> u32 {
        BOOTSTRAP_GENERATION.load(Ordering::Acquire)
    }
    pub(crate) fn status() -> RawStatus {
        STATUS.lock(Cell::get).unwrap_or_default()
    }
    pub(crate) fn publish(status: RawStatus, at: u64) {
        STATUS.lock(|cell| cell.set(Some(status)));
        crate::device::recording::capture(11, at, &status.payload());
    }
    /// kind3=COM2 RX, kind4=COM1 RX, kind8=COM1 attempted TX.
    pub(crate) fn uart(kind: u8, bytes: &[u8], received_us: u64, captured_ns: Option<u64>) {
        let index = match kind {
            3 => 1,
            4 => 0,
            8 => 2,
            _ => return,
        };
        let mut offset = OFFSETS.lock(|cell| {
            let mut offsets = cell.get();
            let previous = offsets[index];
            offsets[index] = previous.saturating_add(bytes.len() as u64);
            cell.set(offsets);
            previous
        });
        for chunk in bytes.chunks(232) {
            let mut payload = [0; 256];
            payload[0] = 1;
            payload[1] = u8::from(captured_ns.is_some());
            payload[4..12].copy_from_slice(&received_us.to_le_bytes());
            payload[12..20].copy_from_slice(&offset.to_le_bytes());
            payload[20..22].copy_from_slice(&(chunk.len() as u16).to_le_bytes());
            payload[24..24 + chunk.len()].copy_from_slice(chunk);
            crate::device::recording::capture(
                kind,
                captured_ns.unwrap_or(0),
                &payload[..24 + chunk.len()],
            );
            offset += chunk.len() as u64;
        }
    }
    pub(crate) fn fault(port: u8, code: u8, detail: u8) {
        let mut payload = [0; 8];
        payload[0] = 1;
        payload[1] = 2;
        payload[2] = code;
        payload[3] = detail;
        payload[4] = port;
        crate::device::recording::capture(
            6,
            crate::device::timing::capture::now_ns().unwrap_or(0),
            &payload,
        );
    }
}
#[cfg(target_arch = "riscv32")]
pub(super) use hardware::{bootstrap_generation, fault, publish, uart};
#[cfg(target_arch = "riscv32")]
pub(crate) use hardware::{request_bootstrap, status};

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn observed_raw_epochs_do_not_treat_repeats_as_advancing() {
        let mut status = RawStatus::default();
        let mut header = Header {
            message_id: 12,
            time_reference: 0,
            time_status: 160,
            week: 2440,
            milliseconds_of_week: 1000,
            output_delay_ms: 0,
        };
        status.observe(header, 50);
        status.observe(header, 60);
        header.milliseconds_of_week = 2000;
        status.observe(header, 70);
        assert_eq!(status.observation_frames, 3);
        assert_eq!(status.advancing_epochs, 2);
        header.message_id = 109;
        status.observe(header, 80);
        assert_eq!(status.ephemeris_mask, 8);
        assert_eq!(status.ephemeris_frames, 1);
        assert_eq!(status.payload().len(), 40);
    }

    #[test]
    fn receiver_reconfiguration_preserves_current_file_but_new_generation_resets_coverage() {
        let mut status = RawStatus::default();
        assert!(status.begin_generation(7, [false; 2]));
        let mut header = Header {
            message_id: 12,
            time_reference: 0,
            time_status: 160,
            week: 2440,
            milliseconds_of_week: 1000,
            output_delay_ms: 0,
        };
        status.observe(header, 50);
        header.message_id = 106;
        status.observe(header, 60);
        status.optional_write_failures = 1;
        // Replaying configuration for this file is not a new recording: its
        // actual evidence and write failures must not disappear.
        assert!(!status.begin_generation(7, [false; 2]));
        assert_eq!(status.observation_frames, 1);
        assert_eq!(status.ephemeris_frames, 1);
        assert_eq!(status.optional_write_failures, 1);
        assert!(status.begin_generation(8, [false; 2]));
        assert_eq!(status.generation, 8);
        assert_eq!(status.observation_frames, 0);
        assert_eq!(status.advancing_epochs, 0);
        assert_eq!(status.ephemeris_frames, 0);
        assert_eq!(status.ephemeris_mask, 0);
        assert_eq!(status.optional_write_failures, 0);
        assert!(status.last_epoch_ms.is_none());
    }

    #[test]
    fn cross_file_frame_is_preserved_but_not_credited_to_the_new_generation() {
        use aevia_device_hal::um980::{Decoder, Frame, crc32};

        fn wire(id: u16, length: usize) -> std::vec::Vec<u8> {
            let mut bytes = std::vec![0; 24 + length];
            bytes[..3].copy_from_slice(&[0xaa, 0x44, 0xb5]);
            bytes[4..6].copy_from_slice(&id.to_le_bytes());
            bytes[6..8].copy_from_slice(&(length as u16).to_le_bytes());
            bytes[9] = 160;
            bytes[10..12].copy_from_slice(&2440_u16.to_le_bytes());
            bytes[12..16].copy_from_slice(&1000_u32.to_le_bytes());
            bytes.extend_from_slice(&crc32(&bytes).to_le_bytes());
            bytes
        }
        let old_frame = wire(12, 1024);
        let mut decoder = Decoder::<512>::new();
        let mut status = RawStatus::default();
        status.begin_generation(1, [false; 2]);
        for &byte in &old_frame[..100] {
            assert_eq!(decoder.push(byte), Ok(None));
        }
        assert!(decoder.has_partial_frame());
        status.begin_generation(2, [false, decoder.has_partial_frame()]);
        let mut completed = 0;
        for byte in old_frame[100..]
            .iter()
            .chain(wire(12, 1024).iter())
            .chain(wire(106, 224).iter())
        {
            if let Some(frame) = decoder.push(*byte).unwrap() {
                let header = match frame {
                    Frame::SkippedBinary { header, .. } => header,
                    Frame::Binary(frame) => frame.header(),
                    _ => panic!("binary payload escaped framing"),
                };
                completed += 1;
                if status.complete_frame(1) {
                    status.observe(header, 123);
                }
            }
        }
        assert_eq!(completed, 3); // Every original frame still decodes.
        assert_eq!(status.observation_frames, 1); // Only the complete new-file OBSVM.
        assert_eq!(status.advancing_epochs, 1);
        assert_eq!(status.ephemeris_frames, 1); // Bounded frames use the same gate.
        assert_eq!(status.ephemeris_mask, 1);
        assert!(!decoder.has_partial_frame());
    }
}
