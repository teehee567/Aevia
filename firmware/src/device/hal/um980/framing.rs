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

// OBSVM frames can occupy the UART at line rate for many FIFO fills. One
// lookup replaces eight per-byte polynomial rounds in both bounded and
// streaming decoding; the N4 initial value and final-XOR convention stay zero.
static CRC32_TABLE: [u32; 256] = crc32_table();

#[inline]
fn crc32_byte(crc: u32, byte: u8) -> u32 {
    (crc >> 8) ^ CRC32_TABLE[((crc ^ u32::from(byte)) & 0xff) as usize]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SentenceKind {
    /// Standard NMEA XOR excludes the leading '$' or '!'.
    Nmea,
    /// Existing UM980 CONFIG response evidence XORs the leading '$' as well.
    UnicoreConfig,
    /// UM980 command acknowledgements also XOR the leading '$'. They do not
    /// establish any configuration readback or PPS timing property.
    UnicoreCommand,
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
        } else if bytes.starts_with(b"$command,") {
            SentenceKind::UnicoreCommand
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
    /// Complete CRC-valid binary frame larger than the typed decoder buffer.
    /// Its original bytes must be retained by the UART capture consumer.
    SkippedBinary {
        message_id: u16,
        payload_len: u16,
        header: super::Header,
    },
}

#[derive(Clone, Copy)]
enum Mode {
    Search,
    Binary,
    Ascii,
    Streaming,
}

#[derive(Clone, Copy)]
struct Streaming {
    remaining: usize,
    message_id: u16,
    payload_len: u16,
    crc: u32,
    expected: u32,
    header: [u8; HEADER_LEN],
    position: usize,
}

/// Mixed-stream decoder for UART ports with binary logs and ASCII replies.
/// Capacity N applies separately to a complete binary frame and ASCII line.
/// Larger binary frames are consumed with a streaming CRC and reported as
/// `SkippedBinary`; their payload never enters the ASCII/search state.
/// Unsupported Unicore long ASCII '#...' messages are skipped; use the raw
/// UART read API if those logs are enabled. RTCM input is sent with corrections.
pub struct Decoder<const N: usize = 512> {
    binary: BinaryDecoder<N>,
    ascii: AsciiDecoder<N>,
    mode: Mode,
    streaming: Option<Streaming>,
}

impl<const N: usize> Decoder<N> {
    pub const fn new() -> Self {
        Self {
            binary: BinaryDecoder::new(),
            ascii: AsciiDecoder::new(),
            mode: Mode::Search,
            streaming: None,
        }
    }

    pub fn reset(&mut self) {
        self.binary.reset();
        self.ascii.reset();
        self.mode = Mode::Search;
        self.streaming = None;
    }

    /// Whether the next completed frame could contain previously pushed bytes.
    /// This lets a raw recorder avoid crediting a frame spanning a file boundary
    /// while retaining decoder state and the exact underlying UART byte stream.
    pub fn has_partial_frame(&self) -> bool {
        match self.mode {
            Mode::Search | Mode::Binary => self.binary.buffered_len() != 0,
            Mode::Ascii => self.ascii.len != 0,
            Mode::Streaming => true,
        }
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<Frame<'_>>, DecodeError> {
        if let Some(stream) = self.streaming.as_mut() {
            if stream.position < HEADER_LEN {
                stream.header[stream.position] = byte;
            }
            stream.position += 1;
            if stream.remaining > 4 {
                stream.crc = crc32_byte(stream.crc, byte);
            } else {
                stream.expected |= u32::from(byte) << ((4 - stream.remaining) * 8);
            }
            stream.remaining -= 1;
            if stream.remaining != 0 {
                return Ok(None);
            }
            let stream = self.streaming.take().unwrap();
            self.mode = Mode::Search;
            return if stream.crc == stream.expected {
                Ok(Some(Frame::SkippedBinary {
                    message_id: stream.message_id,
                    payload_len: stream.payload_len,
                    header: super::Header::from_bytes(&stream.header),
                }))
            } else {
                Err(DecodeError::Checksum)
            };
        }
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
        // Length is known at byte eight. Consume larger native observation
        // logs without storing their payload or searching inside it for ASCII
        // commands/sync. Memory remains bounded and the complete CRC is checked.
        if matches!(self.mode, Mode::Binary) && self.binary.len == 7 {
            let payload_len = u16::from_le_bytes([self.binary.bytes[6], byte]);
            let total = usize::from(payload_len) + OVERHEAD;
            if total > N {
                let mut header = [0; HEADER_LEN];
                header[..7].copy_from_slice(&self.binary.bytes[..7]);
                header[7] = byte;
                self.streaming = Some(Streaming {
                    remaining: total - 8,
                    message_id: u16::from_le_bytes([self.binary.bytes[4], self.binary.bytes[5]]),
                    payload_len,
                    crc: crc32_byte(crc32(&self.binary.bytes[..7]), byte),
                    expected: 0,
                    header,
                    position: 8,
                });
                self.binary.reset();
                self.mode = Mode::Streaming;
                return Ok(None);
            }
        }
        match self.mode {
            Mode::Search | Mode::Streaming => Ok(None),
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
