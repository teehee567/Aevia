//! Version 1 capture container. Every 512-byte page and every record has an
//! IEEE CRC32. A preallocated file's unwritten tail is never treated as data.
pub const PAGE_BYTES: usize = 512;
pub const PAGE_PAYLOAD: usize = 480;
pub const RECORD_PAYLOAD: usize = 256;
pub const RECORD_HEADER: usize = 24;
pub const RECORD_BYTES: usize = RECORD_HEADER + RECORD_PAYLOAD + 4;
pub const MAGIC: &[u8; 8] = b"AEVLOG1\0";

pub fn crc32(data: &[u8]) -> u32 {
    let mut value = !0_u32;
    for &byte in data {
        value ^= u32::from(byte);
        for _ in 0..8 {
            value = (value >> 1) ^ (0xedb8_8320 & 0_u32.wrapping_sub(value & 1));
        }
    }
    !value
}

#[derive(Clone, Copy)]
pub struct Record {
    pub kind: u8,
    pub len: u16,
    pub sequence: u64,
    pub at_ns: u64,
    pub payload: [u8; RECORD_PAYLOAD],
}
impl Record {
    pub const EMPTY: Self = Self {
        kind: 0,
        len: 0,
        sequence: 0,
        at_ns: 0,
        payload: [0; RECORD_PAYLOAD],
    };
    pub fn new(kind: u8, sequence: u64, at_ns: u64, payload: &[u8]) -> Option<Self> {
        if payload.len() > RECORD_PAYLOAD {
            return None;
        }
        let mut result = Self {
            kind,
            len: payload.len() as u16,
            sequence,
            at_ns,
            ..Self::EMPTY
        };
        result.payload[..payload.len()].copy_from_slice(payload);
        Some(result)
    }
    pub fn encode(&self, out: &mut [u8; RECORD_BYTES]) -> usize {
        let len = usize::from(self.len);
        out[..RECORD_HEADER].fill(0);
        out[0] = self.kind;
        out[1] = 1;
        out[2..4].copy_from_slice(&self.len.to_le_bytes());
        out[4..12].copy_from_slice(&self.sequence.to_le_bytes());
        out[12..20].copy_from_slice(&self.at_ns.to_le_bytes());
        out[RECORD_HEADER..RECORD_HEADER + len].copy_from_slice(&self.payload[..len]);
        let end = RECORD_HEADER + len;
        let crc = crc32(&out[..end]);
        out[end..end + 4].copy_from_slice(&crc.to_le_bytes());
        end + 4
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PageInfo {
    pub session: u32,
    pub index: u32,
    pub nonce: u64,
    pub payload_len: u16,
    pub closed: bool,
}

pub fn inspect(page: &[u8; PAGE_BYTES]) -> Option<PageInfo> {
    if &page[..8] != MAGIC
        || crc32(&page[..508]) != u32::from_le_bytes(page[508..512].try_into().ok()?)
    {
        return None;
    }
    let payload_len = u16::from_le_bytes(page[16..18].try_into().ok()?);
    let flags = u16::from_le_bytes(page[18..20].try_into().ok()?);
    if usize::from(payload_len) > PAGE_PAYLOAD || flags > 1 {
        return None;
    }
    // Padding is canonical so corrupted length cannot conceal trailing bytes.
    if page[28 + usize::from(payload_len)..508]
        .iter()
        .any(|&v| v != 0)
    {
        return None;
    }
    Some(PageInfo {
        session: u32::from_le_bytes(page[8..12].try_into().ok()?),
        index: u32::from_le_bytes(page[12..16].try_into().ok()?),
        nonce: u64::from_le_bytes(page[20..28].try_into().ok()?),
        payload_len,
        closed: flags == 1,
    })
}

pub struct PageBuilder {
    bytes: [u8; PAGE_BYTES],
    used: usize,
}
impl Default for PageBuilder {
    fn default() -> Self {
        Self::new()
    }
}
impl PageBuilder {
    pub const fn new() -> Self {
        Self {
            bytes: [0; PAGE_BYTES],
            used: 0,
        }
    }
    pub fn is_empty(&self) -> bool {
        self.used == 0
    }
    pub fn is_full(&self) -> bool {
        self.used == PAGE_PAYLOAD
    }
    pub fn append(&mut self, bytes: &[u8]) -> usize {
        let n = bytes.len().min(PAGE_PAYLOAD - self.used);
        self.bytes[28 + self.used..28 + self.used + n].copy_from_slice(&bytes[..n]);
        self.used += n;
        n
    }
    pub fn finish(
        &mut self,
        session: u32,
        index: u32,
        nonce: u64,
        closed: bool,
    ) -> &[u8; PAGE_BYTES] {
        self.bytes[..8].copy_from_slice(MAGIC);
        self.bytes[8..12].copy_from_slice(&session.to_le_bytes());
        self.bytes[12..16].copy_from_slice(&index.to_le_bytes());
        self.bytes[16..18].copy_from_slice(&(self.used as u16).to_le_bytes());
        self.bytes[18..20].copy_from_slice(&u16::from(closed).to_le_bytes());
        self.bytes[20..28].copy_from_slice(&nonce.to_le_bytes());
        let crc = crc32(&self.bytes[..508]);
        self.bytes[508..].copy_from_slice(&crc.to_le_bytes());
        &self.bytes
    }
    pub fn clear(&mut self) {
        self.bytes.fill(0);
        self.used = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc_reference_and_record_bytes_are_stable() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        let record = Record::new(3, 42, 1234567, b"abc").unwrap();
        let mut out = [0; RECORD_BYTES];
        let n = record.encode(&mut out);
        assert_eq!(n, 31);
        assert_eq!(&out[..4], &[3, 1, 3, 0]);
        assert_eq!(u64::from_le_bytes(out[4..12].try_into().unwrap()), 42);
        assert_eq!(&out[24..27], b"abc");
        assert_eq!(
            u32::from_le_bytes(out[27..31].try_into().unwrap()),
            crc32(&out[..27])
        );
        assert!(Record::new(3, 0, 0, &[0; 257]).is_none());
    }
    #[test]
    fn page_round_trip_corruption_and_padding() {
        let mut builder = PageBuilder::new();
        assert_eq!(builder.append(b"hello"), 5);
        let page = *builder.finish(2, 3, 19, true);
        assert_eq!(
            inspect(&page),
            Some(PageInfo {
                session: 2,
                index: 3,
                nonce: 19,
                payload_len: 5,
                closed: true
            })
        );
        for bit in 0..4096 {
            let mut corrupt = page;
            corrupt[bit / 8] ^= 1 << (bit % 8);
            assert!(inspect(&corrupt).is_none());
        }
        builder.clear();
        assert!(builder.is_empty());
        assert_eq!(builder.append(&[9; 500]), 480);
        assert!(builder.is_full());
        assert_eq!(builder.append(b"x"), 0);
    }
    #[test]
    fn records_span_pages_without_padding_in_stream() {
        let a = Record::new(2, 1, 2, &[7; 256]).unwrap();
        let mut raw = [0; RECORD_BYTES];
        let n = a.encode(&mut raw);
        let mut original = std::vec::Vec::new();
        for _ in 0..5 {
            original.extend_from_slice(&raw[..n]);
        }
        let mut result = std::vec::Vec::new();
        let mut builder = PageBuilder::new();
        let mut source = original.as_slice();
        let mut index = 0;
        while !source.is_empty() {
            let n = builder.append(source);
            source = &source[n..];
            if builder.is_full() || source.is_empty() {
                let page = builder.finish(1, index, 99, source.is_empty());
                let info = inspect(page).unwrap();
                result.extend_from_slice(&page[28..28 + usize::from(info.payload_len)]);
                builder.clear();
                index += 1;
            }
        }
        assert_eq!(result, original);
    }
}
