//! Time-based debounce for the four active-low TCA9536A inputs.

const DEBOUNCE_MS: u64 = 30;
const STALE_MS: u64 = 250;

#[derive(Default)]
pub struct Debouncer {
    candidate: u8,
    since_ms: Option<u64>,
    last_ms: Option<u64>,
    stable: Option<u8>,
}

impl Debouncer {
    /// Returns a pressed-bit mask only after a complete stable interval.
    /// Failed/missing reads invalidate the state and restart debounce.
    pub fn update(&mut self, now_ms: u64, input: Option<u8>) -> Option<u8> {
        let Some(input) = input else {
            *self = Self::default();
            return None;
        };
        if self
            .last_ms
            .is_none_or(|last| now_ms.saturating_sub(last) >= STALE_MS)
        {
            self.since_ms = None;
            self.stable = None;
        }
        self.last_ms = Some(now_ms);
        let pressed = !input & 0x0f;
        if self.since_ms.is_none() || self.candidate != pressed {
            self.candidate = pressed;
            self.since_ms = Some(now_ms);
        }
        if now_ms.saturating_sub(self.since_ms.unwrap()) >= DEBOUNCE_MS {
            self.stable = Some(pressed);
        }
        self.stable
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bouncing_press_and_release_do_not_create_edges() {
        let mut buttons = Debouncer::default();
        assert_eq!(buttons.update(0, Some(0xff)), None);
        assert_eq!(buttons.update(30, Some(0xff)), Some(0));
        assert_eq!(buttons.update(40, Some(0xfe)), Some(0));
        assert_eq!(buttons.update(50, Some(0xff)), Some(0));
        assert_eq!(buttons.update(60, Some(0xfe)), Some(0));
        assert_eq!(buttons.update(89, Some(0xfe)), Some(0));
        assert_eq!(buttons.update(90, Some(0xfe)), Some(1));
        assert_eq!(buttons.update(100, Some(0xff)), Some(1));
        assert_eq!(buttons.update(130, Some(0xff)), Some(0));
    }

    #[test]
    fn missing_reads_and_long_gaps_require_new_debounce() {
        let mut buttons = Debouncer::default();
        buttons.update(0, Some(0xf0));
        assert_eq!(buttons.update(30, Some(0xf0)), Some(15));
        assert_eq!(buttons.update(40, None), None);
        assert_eq!(buttons.update(50, Some(0xf0)), None);
        assert_eq!(buttons.update(80, Some(0xf0)), Some(15));
        assert_eq!(buttons.update(500, Some(0xf0)), None);
    }
}
