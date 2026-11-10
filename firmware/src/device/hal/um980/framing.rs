//! Bounded stream decoders. Binary N4 framing follows the existing firmware's
//! fixed 24-byte header (`AA 44 B5`) and zero-initialized reflected CRC32.

const SYNC: [u8; 3] = [0xaa, 0x44, 0xb5];
pub(super) const HEADER_LEN: usize = 24;
const OVERHEAD: usize = HEADER_LEN + 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    BufferTooSmall,
    Oversized,
    Length,
    Sync,
    Checksum,
    InvalidAscii,
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

    pub const fn buffered_len(&self) -> usize {
        self.len
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

impl<const N: usize> Default for BinaryDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// N4 CRC32: reflected polynomial 0xEDB88320, initial zero, no final XOR.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0_u32.wrapping_sub(crc & 1));
        }
    }
    crc
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SentenceKind {
    /// Standard NMEA XOR excludes the leading '$' or '!'.
    Nmea,
    /// Existing UM980 CONFIG response evidence XORs the leading '$' as well.
    UnicoreConfig,
}

/// Checked ASCII sentence without its optional trailing CR/LF.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AsciiSentence<'a> {
    bytes: &'a [u8],
    star: usize,
    kind: SentenceKind,
}

impl<'a> AsciiSentence<'a> {
    pub fn parse(mut bytes: &'a [u8]) -> Result<Self, DecodeError> {
        if let Some(without_lf) = bytes.strip_suffix(b"\n") {
            bytes = without_lf;
        }
        if let Some(without_cr) = bytes.strip_suffix(b"\r") {
            bytes = without_cr;
        }
        if bytes.len() < 5 || !matches!(bytes[0], b'$' | b'!') {
            return Err(DecodeError::InvalidAscii);
        }
        if !bytes.iter().all(|byte| (0x20..=0x7e).contains(byte)) {
            return Err(DecodeError::InvalidAscii);
        }
        let star = bytes.len() - 3;
        if bytes[star] != b'*' || bytes[1..star].contains(&b'*') {
            return Err(DecodeError::InvalidAscii);
        }
        let kind = if bytes.starts_with(b"$CONFIG,") {
            SentenceKind::UnicoreConfig
        } else {
            SentenceKind::Nmea
        };
        let start = usize::from(kind == SentenceKind::Nmea);
        let checksum = bytes[start..star].iter().fold(0, |xor, byte| xor ^ byte);
        let expected = hex(bytes[star + 1])
            .and_then(|high| hex(bytes[star + 2]).map(|low| high * 16 + low))
            .ok_or(DecodeError::InvalidAscii)?;
        if checksum != expected {
            return Err(DecodeError::Checksum);
        }
        Ok(Self { bytes, star, kind })
    }

    pub const fn kind(self) -> SentenceKind {
        self.kind
    }

    pub const fn wire_bytes(self) -> &'a [u8] {
        self.bytes
    }

    /// Sentence content without the sentinel or '*HH' checksum.
    pub fn body(self) -> &'a [u8] {
        &self.bytes[1..self.star]
    }

    /// Check every parameter of the typed PPS profile, not just the status bit
    /// returned in PPSSTATUS. Useful only when this complete CONFIG readback
    /// was actually received on a working return path.
    pub fn confirms_gps_rising_one_hz(self) -> bool {
        self.kind == SentenceKind::UnicoreConfig
            && self.body() == b"CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0"
    }
}

fn hex(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

/// Bounded CRLF/LF ASCII sentence decoder. A new '$' or '!' discards a partial
/// sentence, permitting recovery from noise or truncation. This is separate
/// from `BinaryDecoder`: do not treat bytes found inside binary payloads as
/// independently authenticated receiver responses.
pub struct AsciiDecoder<const N: usize = 256> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> AsciiDecoder<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<AsciiSentence<'_>>, DecodeError> {
        if N < 5 {
            return Err(DecodeError::BufferTooSmall);
        }
        if matches!(byte, b'$' | b'!') {
            self.bytes[0] = byte;
            self.len = 1;
            return Ok(None);
        }
        if self.len == 0 {
            return Ok(None);
        }
        if byte == b'\n' {
            let len = self.len;
            self.reset();
            return AsciiSentence::parse(&self.bytes[..len]).map(Some);
        }
        if byte != b'\r' && !(0x20..=0x7e).contains(&byte) {
            self.reset();
            return Err(DecodeError::InvalidAscii);
        }
        if self.len == N {
            self.reset();
            return Err(DecodeError::Oversized);
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        Ok(None)
    }
}

impl<const N: usize> Default for AsciiDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// A message from the mixed UART stream. Binary payloads are opaque to the
/// ASCII decoder, preventing embedded '$...*HH' bytes from becoming responses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame<'a> {
    Binary(BinaryFrame<'a>),
    Ascii(AsciiSentence<'a>),
}

#[derive(Clone, Copy)]
enum Mode {
    Search,
    Binary,
    Ascii,
}

/// Mixed-stream decoder for UART ports with binary logs and ASCII replies.
/// Capacity N applies separately to a complete binary frame and ASCII line.
/// Unsupported Unicore long ASCII '#...' messages are skipped; use the raw
/// UART read API if those logs are enabled. RTCM input is sent with corrections.
pub struct Decoder<const N: usize = 512> {
    binary: BinaryDecoder<N>,
    ascii: AsciiDecoder<N>,
    mode: Mode,
}

impl<const N: usize> Decoder<N> {
    pub const fn new() -> Self {
        Self {
            binary: BinaryDecoder::new(),
            ascii: AsciiDecoder::new(),
            mode: Mode::Search,
        }
    }

    pub fn reset(&mut self) {
        self.binary.reset();
        self.ascii.reset();
        self.mode = Mode::Search;
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<Frame<'_>>, DecodeError> {
        if matches!(self.mode, Mode::Search) {
            self.mode = if self.binary.buffered_len() != 0 || byte == SYNC[0] {
                Mode::Binary
            } else if matches!(byte, b'$' | b'!') {
                Mode::Ascii
            } else {
                return Ok(None);
            };
        }
        if matches!(self.mode, Mode::Binary)
            && self.binary.buffered_len() < 3
            && matches!(byte, b'$' | b'!')
        {
            self.binary.reset();
            self.mode = Mode::Ascii;
        }
        if matches!(self.mode, Mode::Ascii) && byte == SYNC[0] {
            self.ascii.reset();
            self.binary.reset();
            self.mode = Mode::Binary;
        }
        match self.mode {
            Mode::Search => Ok(None),
            Mode::Binary => match self.binary.push(byte) {
                Ok(Some(frame)) => {
                    self.mode = Mode::Search;
                    Ok(Some(Frame::Binary(frame)))
                }
                Ok(None) => Ok(None),
                Err(error) => {
                    self.mode = Mode::Search;
                    Err(error)
                }
            },
            Mode::Ascii => {
                if byte == b'\n' {
                    self.mode = Mode::Search;
                }
                self.ascii.push(byte).map(|frame| frame.map(Frame::Ascii))
            }
        }
    }
}

impl<const N: usize> Default for Decoder<N> {
    fn default() -> Self {
        Self::new()
    }
}
