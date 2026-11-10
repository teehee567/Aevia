//! Retry CONFIG readback while the normal mixed UART decoders keep receiving.
use aevia_device_hal::um980::AsciiSentence;

const RESPONSE_US: u64 = 2_000_000;
const RETRY_US: u64 = 3_000_000;
const RECHECK_US: u64 = 30_000_000;

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Action {
    None,
    Query,
    TimedOut,
}

#[derive(Default)]
pub(super) struct PpsVerification {
    verified: bool,
    deadline_us: Option<u64>,
    retry_at_us: u64,
}

impl PpsVerification {
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

    pub fn observe(&mut self, line: AsciiSentence<'_>, now_us: u64) {
        if line.kind() == aevia_device_hal::um980::SentenceKind::UnicoreConfig
            && line.body().starts_with(b"CONFIG,PPS,")
            && !line.confirms_gps_rising_one_hz()
        {
            // A checksummed contradiction revokes an earlier readback, even
            // outside a query window. Never retain a known-wrong configuration.
            self.failed(now_us);
        } else if self.deadline_us.is_some_and(|end| now_us < end)
            && line.confirms_gps_rising_one_hz()
        {
            self.verified = true;
            self.deadline_us = None;
            self.retry_at_us = now_us.saturating_add(RECHECK_US);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aevia_device_hal::um980::{Decoder, Frame};

    fn reply(state: &mut PpsVerification, body: &[u8], now_us: u64) {
        use std::format;
        let checksum = body.iter().fold(b'$', |sum, byte| sum ^ byte);
        let text = format!(
            "${}*{checksum:02X}\r\n",
            core::str::from_utf8(body).unwrap()
        );
        let mut decoder = Decoder::<512>::new();
        for byte in text.bytes() {
            if let Ok(Some(Frame::Ascii(line))) = decoder.push(byte) {
                state.observe(line, now_us);
            }
        }
    }

    const CONFIRM: &[u8] = b"CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0";

    #[test]
    fn lost_readback_retries_without_receiver_reconfiguration() {
        let mut state = PpsVerification::default();
        assert_eq!(state.poll(0), Action::Query);
        // Normal navigation/decoder polls continue for the entire response
        // window and backoff. None can open the PPS/fusion gate.
        for now in (20_000..2_000_000).step_by(20_000) {
            assert_eq!(state.poll(now), Action::None);
            assert!(!state.verified());
        }
        assert_eq!(state.poll(2_000_000), Action::TimedOut);
        reply(&mut state, CONFIRM, 2_000_001); // Late reply is not verification.
        assert!(!state.verified());
        assert_eq!(state.poll(4_999_999), Action::None);
        assert_eq!(state.poll(5_000_000), Action::Query);
        reply(&mut state, CONFIRM, 5_100_000);
        assert!(state.verified());
        assert_eq!(state.poll(35_099_999), Action::None);
        assert_eq!(state.poll(35_100_000), Action::Query);
    }

    #[test]
    fn failed_writes_and_wrong_configuration_never_open_the_gate() {
        let mut state = PpsVerification::default();
        assert_eq!(state.poll(0), Action::Query);
        state.failed(500_000);
        assert_eq!(state.poll(3_499_999), Action::None);
        assert_eq!(state.poll(3_500_000), Action::Query);
        reply(
            &mut state,
            b"CONFIG,PPS,CONFIG PPS ENABLE GPS NEGATIVE 100000 1000 0 0",
            3_600_000,
        );
        assert!(!state.verified());
        assert_eq!(state.poll(6_599_999), Action::None);
        assert_eq!(state.poll(6_600_000), Action::Query);
        reply(&mut state, CONFIRM, 6_700_000);
        assert!(state.verified());
    }

    #[test]
    fn command_acknowledgements_never_substitute_for_full_pps_readback() {
        let mut state = PpsVerification::default();
        assert_eq!(state.poll(0), Action::Query);
        reply(&mut state, b"command,CONFIG,response: OK", 100_000);
        assert!(!state.verified());
        assert_eq!(state.poll(RESPONSE_US), Action::TimedOut);
        assert_eq!(state.poll(RESPONSE_US + RETRY_US), Action::Query);
        reply(&mut state, CONFIRM, 5_100_000);
        assert!(state.verified());
        reply(&mut state, b"command,CONFIG,response: OK", 5_200_000);
        assert!(state.verified());
    }
}

#[cfg(test)]
mod renewal_tests {
    use super::*;
    use aevia_device_hal::um980::AsciiSentence;
    fn reply(body: &str) -> std::string::String {
        let checksum = body.bytes().fold(b'$', |sum, byte| sum ^ byte);
        std::format!("${body}*{checksum:02X}")
    }
    #[test]
    fn lost_revalidation_and_conflicting_config_revoke_verification() {
        let good = reply("CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0");
        let wrong = reply("CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 1000");
        let mut state = PpsVerification::default();
        assert_eq!(state.poll(0), Action::Query);
        state.observe(AsciiSentence::parse(good.as_bytes()).unwrap(), 100);
        assert!(state.verified());
        assert_eq!(state.poll(30_000_100), Action::Query);
        assert!(state.verified());
        assert_eq!(state.poll(32_000_100), Action::TimedOut);
        assert!(!state.verified());
        assert_eq!(state.poll(35_000_100), Action::Query);
        state.observe(AsciiSentence::parse(good.as_bytes()).unwrap(), 35_100_100);
        assert!(state.verified());
        state.observe(AsciiSentence::parse(wrong.as_bytes()).unwrap(), 35_200_100);
        assert!(!state.verified());
    }
}
