use alloc::vec::Vec;

use super::{Fix, GnssReceiver, GnssUpdate};

const BAUD: u32 = 115200;

#[derive(Default)]
pub struct Um980Receiver {
    line: Vec<u8>,
}

impl Um980Receiver {
    pub fn new() -> Self {
        Self::default()
    }
}

fn field(s: &str, n: usize) -> Option<&str> {
    s.split(',').nth(n)
}

fn parse(line: &str) -> Option<Fix> {
    let (header, rest) = line.strip_prefix('#')?.split_once(';')?;
    if !header.starts_with("BESTNAVA") {
        return None;
    }
    let body = rest.split('*').next()?;
    Some(Fix {
        has_fix: field(body, 0)? == "SOL_COMPUTED",
        lat: field(body, 2)?.parse().ok()?,
        lon: field(body, 3)?.parse().ok()?,
        alt_m: field(body, 4)?.parse().ok()?,
        speed_mps: field(body, 25)?.parse().ok()?,
        heading_deg: field(body, 26)?.parse().ok()?,
        vel_latency_s: field(body, 23)?.parse().ok()?,
        speed_std_mps: field(body, 29)?.parse().ok()?,
        gps_week: field(header, 4)?.parse().ok()?,
        tow_ms: field(header, 5)?.parse().ok()?,
    })
}

impl GnssReceiver for Um980Receiver {
    fn baudrate(&self) -> u32 {
        BAUD
    }

    fn init_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"UNLOG\r\n");
        out.extend_from_slice(b"BESTNAVA 0.05\r\n");
        out.extend_from_slice(b"SAVECONFIG\r\n");
        out
    }

    // eventually use bestnavb for binary

    fn consume(&mut self, bytes: &[u8], cb: &mut dyn FnMut(GnssUpdate)) {
        for &b in bytes {
            if b == b'\n' {
                if let Ok(s) = core::str::from_utf8(&self.line) {
                    if let Some(fix) = parse(s.trim_end_matches('\r')) {
                        cb(GnssUpdate::Fix(fix));
                    }
                }
                self.line.clear();
            } else {
                self.line.push(b);
            }
        }
    }
}
