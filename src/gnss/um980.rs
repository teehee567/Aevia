use alloc::vec::Vec;

use super::{Fix, GnssReceiver, GnssUpdate};

const BAUD: u32 = 921600;

#[derive(Default)]
pub struct Um980Receiver {
    data: Vec<u8>,
}

impl Um980Receiver {
    pub fn new() -> Self {
        Self::default()
    }
}

impl GnssReceiver for Um980Receiver {
    fn baudrate(&self) -> u32 {
        BAUD
    }

    // fix to use the um980 binary
    fn init_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"UNLOG\r\n");
        out.extend_from_slice(b"GPGGA 0.05\r\n");
        out.extend_from_slice(b"SAVECONFIG\r\n");
        out
    }

    fn consume(&mut self, bytes: &[u8], cb: &mut dyn FnMut(GnssUpdate)) {
        todo!()
    }
}