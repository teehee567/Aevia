//! Frozen, bounded USB recording API. Every operation is serviced by the SD owner.
use core::fmt;
use serde::{Deserialize, Deserializer, Serialize, Serializer, de};

pub const MAX_READ_BYTES: usize = 4096;
pub const LOG_FORMAT_VERSION: u16 = 1;

/// A caller keeps one operation outstanding. Result polls use the original ID.
/// Repeating an identical ID/operation must not repeat its storage side effects.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Request {
    pub request_id: u32,
    pub command: Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Command {
    Status,
    Start,
    Stop,
    List { after_id: u32 },
    Read { session: u32, offset: u64, len: u16 },
    Result,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Response {
    pub request_id: u32,
    pub reply: Reply,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
// Fixed storage keeps the asynchronous SD/USB boundary allocation-free.
#[allow(clippy::large_enum_variant)]
pub enum Reply {
    Pending,
    Status(Status),
    Started(Session),
    Stopped(Session),
    Entry(Option<Session>),
    Data(Chunk),
    Error(Error),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum State {
    #[default]
    Initializing,
    Ready,
    Recording,
    Stopping,
    Fault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Filesystem {
    #[default]
    Unknown,
    Fat32,
    Exfat,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Error {
    NoCard,
    NotReady,
    UnsupportedFilesystem,
    Busy,
    UnknownRequest,
    RequestConflict,
    InvalidLength,
    SessionNotFound,
    OutOfRange,
    Io,
    Corrupt,
    NoSpace,
    ReadOnly,
    RecordingActive,
    NotRecording,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Status {
    /// Next operation token. Status/Result do not consume a token; hosts query
    /// this before issuing work instead of choosing a token at random.
    pub next_request_id: u32,
    pub state: State,
    pub filesystem: Filesystem,
    pub capacity_bytes: u64,
    pub free_bytes: Option<u64>,
    pub active_session: Option<Session>,
    pub queued_records: u32,
    pub accepted_records: u64,
    pub dropped_records: u32,
    pub written_pages: u64,
    pub io_errors: u32,
    pub last_error: Option<Error>,
    pub max_read_len: u16,
    pub format_version: u16,
}

impl Default for Status {
    fn default() -> Self {
        Self {
            next_request_id: 1,
            state: State::Initializing,
            filesystem: Filesystem::Unknown,
            capacity_bytes: 0,
            free_bytes: None,
            active_session: None,
            queued_records: 0,
            accepted_records: 0,
            dropped_records: 0,
            written_pages: 0,
            io_errors: 0,
            last_error: None,
            max_read_len: MAX_READ_BYTES as u16,
            format_version: LOG_FORMAT_VERSION,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Session {
    pub session: u32,
    /// Physical preallocated file length, not the amount of recorded data.
    pub file_bytes: u64,
    /// Recoverable page prefix in bytes, always a multiple of 512. The host
    /// independently validates every page and record; discovery is not a scan.
    pub valid_bytes: u64,
    /// A final page was committed. This does not assert loss-free acquisition.
    pub closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Chunk {
    pub session: u32,
    pub offset: u64,
    pub bytes: Bytes,
    /// IEEE CRC32 of bytes, independent of the log's page/record checksums.
    pub crc32: u32,
}

/// A variable-length byte sequence with fixed storage and no heap allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bytes {
    data: [u8; MAX_READ_BYTES],
    len: u16,
}

impl Bytes {
    pub fn new(value: &[u8]) -> Result<Self, Error> {
        if value.len() > MAX_READ_BYTES {
            return Err(Error::InvalidLength);
        }
        let mut data = [0; MAX_READ_BYTES];
        data[..value.len()].copy_from_slice(value);
        Ok(Self {
            data,
            len: value.len() as u16,
        })
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

impl Serialize for Bytes {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.as_slice().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Bytes {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Visitor;
        impl<'de> de::Visitor<'de> for Visitor {
            type Value = Bytes;
            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("at most 4096 bytes")
            }
            fn visit_seq<A: de::SeqAccess<'de>>(self, mut sequence: A) -> Result<Bytes, A::Error> {
                let mut value = Bytes {
                    data: [0; MAX_READ_BYTES],
                    len: 0,
                };
                while let Some(byte) = sequence.next_element::<u8>()? {
                    let index = usize::from(value.len);
                    if index == MAX_READ_BYTES {
                        return Err(de::Error::invalid_length(index + 1, &self));
                    }
                    value.data[index] = byte;
                    value.len += 1;
                }
                Ok(value)
            }
        }
        deserializer.deserialize_seq(Visitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol;

    #[test]
    fn recording_envelopes_append_and_maximum_chunks_fit_existing_frames() {
        let mut output = [0; protocol::RESPONSE_FRAME_SIZE];
        let golden = protocol::Request::Recording(Request {
            request_id: 42,
            command: Command::Read {
                session: 7,
                offset: 256,
                len: 256,
            },
        });
        assert_eq!(
            postcard::to_slice_cobs(&golden, &mut output).unwrap(),
            &[9, 2, 42, 4, 7, 128, 2, 128, 2, 0]
        );
        let request = protocol::Request::Recording(Request {
            request_id: u32::MAX,
            command: Command::Read {
                session: u32::MAX,
                offset: u64::MAX,
                len: MAX_READ_BYTES as u16,
            },
        });
        assert_eq!(postcard::to_slice(&request, &mut output).unwrap()[0], 2);
        let frame = postcard::to_slice_cobs(&request, &mut output).unwrap();
        assert!(frame.len() <= protocol::FRAME_SIZE);
        for data in [[0; MAX_READ_BYTES], [255; MAX_READ_BYTES]] {
            let expected = Response {
                request_id: u32::MAX,
                reply: Reply::Data(Chunk {
                    session: u32::MAX,
                    offset: u64::MAX,
                    bytes: Bytes::new(&data).unwrap(),
                    crc32: u32::MAX,
                }),
            };
            let response = protocol::Response::Recording(expected);
            assert_eq!(postcard::to_slice(&response, &mut output).unwrap()[0], 2);
            let frame = postcard::to_slice_cobs(&response, &mut output).unwrap();
            let protocol::Response::Recording(actual) = postcard::from_bytes_cobs(frame).unwrap()
            else {
                panic!()
            };
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn byte_sequence_length_is_checked_on_construction_and_decoding() {
        assert_eq!(
            Bytes::new(&[0; MAX_READ_BYTES + 1]),
            Err(Error::InvalidLength)
        );
        let mut serialized = [0; MAX_READ_BYTES + 8];
        let oversized =
            postcard::to_slice(&&[0u8; MAX_READ_BYTES + 1][..], &mut serialized).unwrap();
        assert!(postcard::from_bytes::<Bytes>(oversized).is_err());
        for len in [0, 1, 255, 256, MAX_READ_BYTES - 1, MAX_READ_BYTES] {
            let expected = Bytes::new(&[63; MAX_READ_BYTES][..len]).unwrap();
            let encoded = postcard::to_slice(&expected, &mut serialized).unwrap();
            assert_eq!(postcard::from_bytes::<Bytes>(encoded).unwrap(), expected);
        }
    }

    #[test]
    fn malformed_download_lengths_and_overflow_frames_cannot_become_valid_responses() {
        let mut encoded = [0; protocol::RESPONSE_FRAME_SIZE];
        // Correct envelope/COBS with an oversized byte-sequence declaration.
        let oversized = (2u8, 1u8, 5u8, 7u8, 0u8, &[0u8; MAX_READ_BYTES + 1][..], 0u8);
        let mut input: &[u8] = postcard::to_slice_cobs(&oversized, &mut encoded).unwrap();
        let mut parser = protocol::Frame::<{ protocol::RESPONSE_FRAME_SIZE }>::new();
        assert!(matches!(
            parser.feed::<protocol::Response>(&mut input),
            Some(Err(_))
        ));
        assert!(input.is_empty());

        // Claims 4096 bytes but carries one byte: never borrow or decode a tail.
        let mut truncated = [2, 1, 5, 7, 0, 0x80, 0x20, 9, 0];
        assert!(postcard::from_bytes::<protocol::Response>(&truncated).is_err());
        truncated[5] = 0x81; // 4097 also fails before exposing any payload.
        assert!(postcard::from_bytes::<protocol::Response>(&truncated).is_err());

        let mut too_long = [1u8; protocol::RESPONSE_FRAME_SIZE + 1];
        too_long[protocol::RESPONSE_FRAME_SIZE] = 0;
        let mut input = too_long.as_slice();
        assert!(matches!(
            parser.feed::<protocol::Response>(&mut input),
            Some(Err(_))
        ));
        let mut next: &[u8] = &[1, 1, 1, 0]; // Existing System::Ok remains decodable.
        assert!(matches!(
            parser.feed::<protocol::Response>(&mut next),
            Some(Ok(protocol::Response::System(
                crate::commands::Response::Ok
            )))
        ));
    }
}
