//! CONFIG readback policy; an acknowledgement cannot authenticate PPS timing.
const RESPONSE_US: u64 = 2_000_000;
const RETRY_US: u64 = 3_000_000;
const RECHECK_US: u64 = 30_000_000;
const CONFIRM: &[u8] = b"CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0";

#[derive(Debug, PartialEq, Eq)]
pub enum Action {
    None,
    Query,
    TimedOut,
}

#[derive(Default)]
pub struct Verification {
    verified: bool,
    deadline_us: Option<u64>,
    retry_at_us: u64,
}
impl Verification {
    pub fn verified(&self) -> bool {
        self.verified
    }
    pub fn poll(&mut self, now_us: u64) -> Action {
        if let Some(deadline) = self.deadline_us {
            if now_us >= deadline {
                self.failed(now_us);
                return Action::TimedOut;
            }
        } else if now_us >= self.retry_at_us {
            self.deadline_us = Some(now_us.saturating_add(RESPONSE_US));
            return Action::Query;
        }
        Action::None
    }
    pub fn failed(&mut self, now_us: u64) {
        self.verified = false;
        self.deadline_us = None;
        self.retry_at_us = now_us.saturating_add(RETRY_US);
    }
    pub fn observe(&mut self, line: &[u8], now_us: u64) {
        let Some(body) = config_body(line) else {
            return;
        };
        if body.starts_with(b"CONFIG,PPS,") && body != CONFIRM {
            self.failed(now_us);
        } else if body == CONFIRM && self.deadline_us.is_some_and(|deadline| now_us < deadline) {
            self.verified = true;
            self.deadline_us = None;
            self.retry_at_us = now_us.saturating_add(RECHECK_US);
        }
    }
}

fn config_body(mut line: &[u8]) -> Option<&[u8]> {
    if let Some(value) = line.strip_suffix(b"\n") {
        line = value;
    }
    if let Some(value) = line.strip_suffix(b"\r") {
        line = value;
    }
    if !line.starts_with(b"$CONFIG,")
        || line.len() < 12
        || !line.iter().all(|b| (0x20..=0x7e).contains(b))
    {
        return None;
    }
    let star = line.len() - 3;
    if line[star] != b'*' || line[1..star].contains(&b'*') {
        return None;
    }
    let hex = |b| match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    };
    let expected = hex(line[star + 1])? * 16 + hex(line[star + 2])?;
    // UM980 CONFIG replies include '$' in their XOR, unlike NMEA.
    (line[..star].iter().fold(0, |sum, b| sum ^ b) == expected).then_some(&line[1..star])
}

#[cfg(test)]
mod tests {
    use super::*;
    fn reply(state: &mut Verification, body: &[u8], now: u64) {
        let sum = body.iter().fold(b'$', |sum, b| sum ^ b);
        state.observe(
            std::format!("${}*{sum:02X}\r\n", core::str::from_utf8(body).unwrap()).as_bytes(),
            now,
        );
    }
    #[test]
    fn absent_late_and_acknowledgement_replies_never_verify() {
        let mut v = Verification::default();
        assert_eq!(v.poll(0), Action::Query);
        reply(&mut v, b"command,CONFIG,response: OK", 100);
        assert!(!v.verified());
        assert_eq!(v.poll(RESPONSE_US), Action::TimedOut);
        reply(&mut v, CONFIRM, RESPONSE_US + 1);
        assert!(!v.verified());
        assert_eq!(v.poll(RESPONSE_US + RETRY_US), Action::Query);
        reply(&mut v, CONFIRM, RESPONSE_US + RETRY_US + 1);
        assert!(v.verified());
    }
    #[test]
    fn missing_renewal_or_contradictory_readback_revokes_verification() {
        let mut v = Verification::default();
        v.poll(0);
        reply(&mut v, CONFIRM, 100);
        assert!(v.verified());
        assert_eq!(v.poll(30_000_100), Action::Query);
        assert_eq!(v.poll(32_000_100), Action::TimedOut);
        assert!(!v.verified());
        v.poll(35_000_100);
        reply(&mut v, CONFIRM, 35_000_200);
        assert!(v.verified());
        reply(
            &mut v,
            b"CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 1000",
            35_000_300,
        );
        assert!(!v.verified());
    }
    #[test]
    fn malformed_and_unchecksummed_readback_never_verifies() {
        let mut v = Verification::default();
        v.poll(0);
        v.observe(CONFIRM, 100);
        v.observe(
            b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0*00",
            100,
        );
        assert!(!v.verified());
    }
}
