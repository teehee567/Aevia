//! Extend the 32-bit 10MHz capture counter without guessing across long gaps.

pub(crate) const TICK_NS: u64 = 100;
const HALF_WRAP_TICKS: u64 = 1_u64 << 31;
const HALF_WRAP_US: u64 = HALF_WRAP_TICKS / 10;

pub(crate) struct Counter {
    last: u32,
    ticks: u64,
    // The reference belongs to the last wall-clock validation, not the last
    // fast read. Otherwise frequent reads could hide a wrong-rate clock.
    checked_ticks: u64,
    checked_us: u64,
}

impl Counter {
    pub const fn new(raw: u32, observed_us: u64) -> Self {
        Self {
            last: raw,
            ticks: raw as u64,
            checked_ticks: raw as u64,
            checked_us: observed_us,
        }
    }

    /// Extend using only the hardware counter. The caller must also perform
    /// periodic `advance` validation, and prevent resumption after an outage
    /// longer than half a wrap (214.7 s). Firmware validates every 100 ms;
    /// task supervision plus the hardware watchdog reset a stalled owner well
    /// before half a wrap, including the bounded startup grace period.
    pub fn advance_fast(&mut self, raw: u32) -> Option<u64> {
        let delta = u64::from(raw.wrapping_sub(self.last));
        if delta >= HALF_WRAP_TICKS {
            return None;
        }
        self.ticks = self.ticks.checked_add(delta)?;
        self.last = raw;
        Some(self.ticks)
    }

    pub fn advance(&mut self, raw: u32, observed_us: u64) -> Option<u64> {
        let elapsed = observed_us.checked_sub(self.checked_us)?;
        if elapsed >= HALF_WRAP_US {
            return None;
        }
        let ticks = self.advance_fast(raw)?;
        let delta = ticks.checked_sub(self.checked_ticks)?;
        // Detect a stopped or incorrectly configured capture clock. Allow
        // 100us for cross-read uncertainty plus a 1000ppm clock-rate margin.
        if delta.abs_diff(elapsed * 10) > 1000 + elapsed / 100 {
            return None;
        }
        self.checked_ticks = ticks;
        self.checked_us = observed_us;
        Some(ticks)
    }

    pub fn captured(&self, raw: u32) -> Option<u64> {
        let age = u64::from(self.last.wrapping_sub(raw));
        if age * TICK_NS > 5_000_000_000 {
            return None;
        }
        self.ticks.checked_sub(age)
    }
}
