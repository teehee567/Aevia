//! Bounded binary N4 framing follows the existing firmware's
//! fixed 24-byte header (`AA 44 B5`) and zero-initialized reflected CRC32.

const SYNC: [u8; 3] = [0xaa, 0x44, 0xb5];
const HEADER_LEN: usize = 24;
const OVERHEAD: usize = HEADER_LEN + 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    BufferTooSmall,
    Oversized,
    Length,
    Sync,
    Checksum,
    MessageType,
}

/// A complete CRC-validated N4 frame including its header and trailing CRC.
/// Unknown message IDs are accepted; raw access supports the full log catalog.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BinaryFrame<'a> {
    bytes: &'a [u8],
}

impl<'a> BinaryFrame<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, DecodeError> {
        if bytes.len() < OVERHEAD {
            return Err(DecodeError::Length);
        }
        if bytes[..3] != SYNC {
            return Err(DecodeError::Sync);
        }
        let payload_len = usize::from(u16::from_le_bytes([bytes[6], bytes[7]]));
        if bytes.len() != OVERHEAD + payload_len {
            return Err(DecodeError::Length);
        }
        let end = bytes.len() - 4;
        let expected = u32::from_le_bytes(bytes[end..].try_into().unwrap());
        if crc32(&bytes[..end]) != expected {
            return Err(DecodeError::Checksum);
        }
        Ok(Self { bytes })
    }

    pub const fn wire_bytes(self) -> &'a [u8] {
        self.bytes
    }

    pub fn header_bytes(self) -> &'a [u8] {
        &self.bytes[..HEADER_LEN]
    }

    pub fn payload(self) -> &'a [u8] {
        &self.bytes[HEADER_LEN..self.bytes.len() - 4]
    }

    pub fn message_id(self) -> u16 {
        u16::from_le_bytes([self.bytes[4], self.bytes[5]])
    }
}

/// Allocation-free N4 decoder with capacity including the 28 framing bytes.
/// A frame borrows the decoder until the next push. Noise and overlapping sync
/// prefixes are tolerated. Oversized/corrupt frames return an error, then the
/// decoder searches for a new sync; no error is silently treated as a sample.
///
/// A plausible truncated frame requires the caller to call `reset` on an idle
/// deadline. Sync bytes inside a valid payload do not restart the decoder.
/// `N < 28` is reported as `BufferTooSmall` by `push`, without panicking.
pub struct BinaryDecoder<const N: usize = 512> {
    bytes: [u8; N],
    len: usize,
    total: usize,
}

impl<const N: usize> BinaryDecoder<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            total: 0,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.total = 0;
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<BinaryFrame<'_>>, DecodeError> {
        if N < OVERHEAD {
            return Err(DecodeError::BufferTooSmall);
        }
        if self.len < SYNC.len() {
            if byte == SYNC[self.len] {
                self.bytes[self.len] = byte;
                self.len += 1;
            } else if byte == SYNC[0] {
                self.bytes[0] = byte;
                self.len = 1;
            } else {
                self.len = 0;
            }
            return Ok(None);
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        if self.len == 8 {
            self.total = OVERHEAD + usize::from(u16::from_le_bytes([self.bytes[6], self.bytes[7]]));
            if self.total > N {
                self.retain_sync_tail();
                return Err(DecodeError::Oversized);
            }
        }
        if self.total != 0 && self.len == self.total {
            let len = self.len;
            if let Err(error) = BinaryFrame::parse(&self.bytes[..len]) {
                self.retain_sync_tail();
                return Err(error);
            }
            self.reset();
            return Ok(Some(BinaryFrame {
                bytes: &self.bytes[..len],
            }));
        }
        Ok(None)
    }

    // Keep a partial sync ending at the failing byte, so AA44/AA prefixes at
    // the end of malformed framing can join the next input byte.
    fn retain_sync_tail(&mut self) {
        let keep = if self.len >= 2 && self.bytes[self.len - 2..self.len] == SYNC[..2] {
            2
        } else if self.len != 0 && self.bytes[self.len - 1] == SYNC[0] {
            1
        } else {
            0
        };
        if keep != 0 {
            self.bytes.copy_within(self.len - keep..self.len, 0);
        }
        self.len = keep;
        self.total = 0;
    }
}

/// N4 CRC32: reflected polynomial 0xEDB88320, initial zero, no final XOR.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for &byte in bytes {
        crc = crc32_byte(crc, byte);
    }
    crc
}

const fn crc32_table() -> [u32; 256] {
    let mut table = [0; 256];
    let mut index = 0;
    while index < table.len() {
        let mut value = index as u32;
        let mut bit = 0;
        while bit < 8 {
            value = (value >> 1) ^ (0xedb8_8320 & 0_u32.wrapping_sub(value & 1));
            bit += 1;
        }
        table[index] = value;
        index += 1;
    }
    table
}

static CRC32_TABLE: [u32; 256] = crc32_table();

#[inline]
fn crc32_byte(crc: u32, byte: u8) -> u32 {
    (crc >> 8) ^ CRC32_TABLE[((crc ^ u32::from(byte)) & 0xff) as usize]
}
