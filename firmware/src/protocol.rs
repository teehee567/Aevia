use crate::loader::update::Error;
use serde::Deserialize;

// System::Data with a 256-byte payload, worst-case u32 offset, and COBS overhead.
pub const FRAME_SIZE: usize = 272;

/// Fixed-memory framing shared by system and device transports.
/// An oversized frame is discarded through its delimiter, never decoded as a tail.
pub struct Frame<const N: usize = FRAME_SIZE> {
    bytes: [u8; N],
    len: usize,
    discard: bool,
}

impl<const N: usize> Frame<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            discard: false,
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.discard = false;
    }

    /// Consume up to one frame; `input` retains any subsequent frames.
    /// Borrowed payloads remain valid until the next feed or clear.
    pub fn feed<'de, T: Deserialize<'de>>(
        &'de mut self,
        input: &mut &[u8],
    ) -> Option<Result<T, Error>> {
        let end = input.iter().position(|&byte| byte == 0);
        let count = end.map_or(input.len(), |end| end + 1);
        let (part, rest) = input.split_at(count);
        *input = rest;
        if !self.discard {
            if count > N.saturating_sub(self.len) {
                self.discard = true;
            } else {
                self.bytes[self.len..self.len + count].copy_from_slice(part);
                self.len += count;
            }
        }
        end?;
        let len = core::mem::take(&mut self.len);
        if core::mem::take(&mut self.discard) {
            return Some(Err(Error::Command));
        }
        let decoded = match cobs::decode_in_place(&mut self.bytes[..len]) {
            Ok(decoded) => decoded,
            Err(_) => return Some(Err(Error::Command)),
        };
        Some(match postcard::take_from_bytes(&self.bytes[..decoded]) {
            Ok((value, [])) => Ok(value),
            _ => Err(Error::Command),
        })
    }
}

impl<const N: usize> Default for Frame<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Protocol v1. Variant order and field types are part of the wire contract.
#[derive(Debug, serde::Serialize, Deserialize)]
pub enum Request<'a> {
    System(#[serde(borrow)] crate::commands::Command<'a>),
    Device(crate::device::interface::DeviceCommand),
}

#[derive(Debug, serde::Serialize, Deserialize)]
// A fixed-capacity wire envelope avoids heap allocation in the USB path.
#[allow(clippy::large_enum_variant)]
pub enum Response {
    System(crate::commands::Response),
    Device(crate::device::interface::DeviceResponse),
}
