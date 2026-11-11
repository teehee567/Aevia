use crate::loader::update::Error;
use serde::{Deserialize, Serialize};

/// System commands. Variant order is part of the Postcard wire protocol.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum Command<'a> {
    Info,
    Abort,
    Run,
    Begin {
        size: u32,
        hash: [u8; 32],
    },
    Data {
        offset: u32,
        bytes: &'a [u8],
    },
    Commit,
    Recover,
    /// Current application startup stage (available before acquisition starts).
    BootStatus,
    /// Fixed-layout discovery; query before decoding versioned device telemetry.
    ProtocolInfo,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Response {
    Ok,
    Reboot,
    Info {
        recovery: bool,
        max_size: u32,
        chunk: u16,
    },
    Error(Error),
    BootStatus {
        stage: u8,
    },
    ProtocolInfo(crate::protocol::ProtocolInfo),
}
