//! N4 binary framing and BESTNAVXYZ decoding for AEVLOG1 import and PPK export.
//! Historical layouts come from firmware commit 31a20b4.
mod framing;
mod messages;
pub use framing::{BinaryDecoder, BinaryFrame};
pub use messages::BestNavXyz;

#[cfg(test)]
pub use framing::crc32;
