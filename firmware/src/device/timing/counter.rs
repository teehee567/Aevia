//! Extend the 32-bit 10MHz capture counter without guessing across long gaps.

pub(super) const TICK_NS: u64 = 100;
const HALF_WRAP_US: u64 = (1_u64 << 31) / 10;

pub(super) struct Counter {
    last: u32,
    ticks: u64,
    observed_us: u64,
}

impl Counter {
    pub const fn new(raw: u32, observed_us: u64) -> Self {
        Self {
            last: raw,
            ticks: raw as u64,
            observed_us,
        }
    }

    pub fn advance(&mut self, raw: u32, observed_us: u64) -> Option<u64> {
        let elapsed = observed_us.checked_sub(self.observed_us)?;
        if elapsed >= HALF_WRAP_US {
            return None;
        }
        let delta = u64::from(raw.wrapping_sub(self.last));
        // Detect a stopped or incorrectly configured capture clock. Allow
        // 100us for cross-read uncertainty plus a 1000ppm clock-rate margin.
        if delta.abs_diff(elapsed * 10) > 1000 + elapsed / 100 {
            return None;
        }
        self.ticks = self.ticks.checked_add(delta)?;
        self.last = raw;
        self.observed_us = observed_us;
        Some(self.ticks)
    }

    pub fn captured(&self, raw: u32) -> Option<u64> {
        let age = u64::from(self.last.wrapping_sub(raw));
        if age * TICK_NS > 5_000_000_000 {
            return None;
        }
        self.ticks.checked_sub(age)
    }
}
