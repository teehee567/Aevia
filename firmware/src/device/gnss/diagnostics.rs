//! Cheap per-port history; byte copies for telemetry happen only on failure.
use super::super::interface::GnssParserError;
use aevia_device_hal::um980::DecodeError;

pub(super) struct ParserContext {
    bytes: [u8; 64],
    next: usize,
    len: u8,
}

impl ParserContext {
    pub const fn new() -> Self {
        Self {
            bytes: [0; 64],
            next: 0,
            len: 0,
        }
    }

    #[inline]
    pub fn push(&mut self, byte: u8) {
        self.bytes[self.next] = byte;
        self.next = (self.next + 1) & 63;
        self.len = self.len.saturating_add(1).min(64);
    }

    pub fn failure(
        &self,
        port: u8,
        error: DecodeError,
        message_id: Option<u16>,
        received_us: u64,
    ) -> GnssParserError {
        let mut context = [[0; 32]; 2];
        let start = if self.len == 64 { self.next } else { 0 };
        for index in 0..usize::from(self.len) {
            context[index / 32][index % 32] = self.bytes[(start + index) & 63];
        }
        GnssParserError {
            port,
            code: error_code(error),
            message_id,
            received_us,
            context_len: self.len,
            context,
        }
    }
}

// Keep this exhaustive and explicit: wire diagnostic codes must not depend on
// Rust enum layout, and a new HAL variant must be mapped deliberately.
fn error_code(error: DecodeError) -> u8 {
    match error {
        DecodeError::BufferTooSmall => 1,
        DecodeError::Oversized => 2,
        DecodeError::Length => 3,
        DecodeError::Sync => 4,
        DecodeError::Checksum => 5,
        DecodeError::InvalidAscii => 6,
        DecodeError::MessageType => 7,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_context_is_chronological_before_and_after_ring_wrap() {
        let mut context = ParserContext::new();
        for byte in 0..3 {
            context.push(byte);
        }
        let failure = context.failure(1, DecodeError::Checksum, None, 123);
        assert_eq!(failure.context_len, 3);
        assert_eq!(&failure.context[0][..4], &[0, 1, 2, 0]);
        for byte in 3..100 {
            context.push(byte);
        }
        let failure = context.failure(2, DecodeError::Length, Some(240), 456);
        assert_eq!(failure.context_len, 64);
        assert_eq!(failure.port, 2);
        assert_eq!(failure.message_id, Some(240));
        assert_eq!(failure.received_us, 456);
        for (index, byte) in failure.context.iter().flatten().enumerate() {
            assert_eq!(*byte, 36 + index as u8);
        }
    }

    #[test]
    fn codes_are_stable_and_context_includes_the_failing_decoder_byte() {
        use aevia_device_hal::um980::Decoder;
        for (error, code) in [
            (DecodeError::BufferTooSmall, 1),
            (DecodeError::Oversized, 2),
            (DecodeError::Length, 3),
            (DecodeError::Sync, 4),
            (DecodeError::Checksum, 5),
            (DecodeError::InvalidAscii, 6),
            (DecodeError::MessageType, 7),
        ] {
            assert_eq!(error_code(error), code);
        }
        let mut decoder = Decoder::<512>::new();
        let mut context = ParserContext::new();
        let mut failure = None;
        for &byte in b"$CONFIG,PPS,wrong*00\r\n" {
            context.push(byte);
            if let Err(error) = decoder.push(byte) {
                failure = Some(context.failure(1, error, None, 1));
            }
        }
        let failure = failure.expect("invalid checksummed line must retain diagnostic evidence");
        assert_eq!(failure.code, 5);
        assert_eq!(failure.context_len, 22);
        assert_eq!(&failure.context[0][..22], b"$CONFIG,PPS,wrong*00\r\n");
    }
}

#[cfg(test)]
mod wire_tests {
    use super::*;
    use crate::{
        device::interface::{DeviceResponse, GnssStatus},
        protocol::Response,
    };

    #[test]
    fn parser_evidence_survives_the_actual_bounded_wire_envelope() {
        let mut context = ParserContext::new();
        for byte in 0..=u8::MAX {
            context.push(byte);
        }
        let failure = context.failure(2, DecodeError::MessageType, Some(u16::MAX), u64::MAX);
        let response = Response::Device(DeviceResponse::GnssStatus(GnssStatus {
            config_queries: u32::MAX,
            config_timeouts: u32::MAX,
            last_parser_error: Some(failure),
            ..Default::default()
        }));
        let mut bytes = [0; 512];
        let encoded = postcard::to_slice_cobs(&response, &mut bytes).unwrap();
        let Response::Device(DeviceResponse::GnssStatus(status)) =
            postcard::from_bytes_cobs(encoded).unwrap()
        else {
            panic!("GNSS diagnostic response required");
        };
        let actual = status.last_parser_error.unwrap();
        assert_eq!(actual.port, 2);
        assert_eq!(actual.code, 7);
        assert_eq!(actual.message_id, Some(u16::MAX));
        assert_eq!(actual.received_us, u64::MAX);
        assert_eq!(actual.context_len, 64);
        assert_eq!(actual.context, failure.context);
        assert_eq!(status.config_queries, u32::MAX);
    }
}
