//! CPU-local short intervals. The acquisition CPU runs at a fixed configured
//! frequency; these intervals do not replace hardware measurement timestamps.

const HALF_WRAP: u32 = 1 << 31;

#[derive(Clone, Copy)]
pub(crate) struct ClockRate(u32);

impl ClockRate {
    pub(crate) fn new(hz: u32) -> Option<Self> {
        (hz != 0).then_some(Self(hz))
    }

    pub(crate) fn cycles_for_ns(self, ns: u32) -> Option<u32> {
        // All configured S31 CPU frequencies are whole MHz. Keep that common
        // path in native-width arithmetic; retain exact ceiling for other rates.
        let cycles = if self.0.is_multiple_of(1_000_000) {
            ns.checked_mul(self.0 / 1_000_000)
                .map(|n| u64::from(n.div_ceil(1000)))
                .unwrap_or_else(|| (u64::from(ns) * u64::from(self.0)).div_ceil(1_000_000_000))
        } else {
            (u64::from(ns) * u64::from(self.0)).div_ceil(1_000_000_000)
        };
        (cycles < u64::from(HALF_WRAP)).then_some(cycles as u32)
    }

    pub(crate) fn micros_for_cycles(self, cycles: u32) -> u32 {
        if self.0.is_multiple_of(1_000_000) {
            cycles / (self.0 / 1_000_000)
        } else {
            (u64::from(cycles) * 1_000_000 / u64::from(self.0)) as u32
        }
    }
}

/// Refuse ambiguous intervals rather than interpreting a backward/stopped
/// counter as an indefinitely unexpired deadline.
pub(crate) fn elapsed(start: u32, now: u32) -> Option<u32> {
    let ticks = now.wrapping_sub(start);
    (ticks < HALF_WRAP).then_some(ticks)
}

#[cfg(target_arch = "riscv32")]
pub(crate) mod native {
    use super::*;
    use aevia_device_hal::embedded_hal::delay::DelayNs;
    use esp_hal::delay::Delay;

    #[derive(Clone, Copy)]
    pub(crate) struct CycleClock(ClockRate);

    impl CycleClock {
        /// Enable and validate the CPU0 counter before selecting the fast path.
        /// Both executors owning this adapter remain on CPU0, without DVFS.
        pub(crate) fn prepare() -> Option<Self> {
            debug_assert_eq!(
                esp_hal::system::Cpu::current(),
                esp_hal::system::Cpu::ProCpu
            );
            let rate = ClockRate::new(esp_hal::clock::cpu_clock().as_hz())?;
            let inhibited: u32;
            unsafe {
                core::arch::asm!("csrrc {old}, mcountinhibit, {mask}", old = out(reg) _, mask = in(reg) 1_u32, options(nostack));
                core::arch::asm!("csrr {value}, mcountinhibit", value = out(reg) inhibited, options(nostack));
            }
            if inhibited & 1 != 0 {
                return None;
            }
            let clock = Self(rate);
            let before = clock.now();
            Delay::new().delay_micros(10);
            let ticks = elapsed(before, clock.now())?;
            // The timed interval must advance the counter. Reject absent or
            // implausibly slow counters instead of entering an unbounded delay.
            if ticks < rate.cycles_for_ns(5_000)? {
                return None;
            }
            Some(clock)
        }

        #[inline]
        pub(crate) fn now(self) -> u32 {
            let cycles: u32;
            // Keep a compiler memory barrier around the CSR read so GPIO/SPI
            // writes cannot move across a setup/hold delay or deadline sample.
            unsafe {
                core::arch::asm!("csrr {value}, mcycle", value = out(reg) cycles, options(nostack));
            }
            cycles
        }
        pub(crate) fn rate(self) -> ClockRate {
            self.0
        }
        pub(crate) fn hz(self) -> u32 {
            self.0.0
        }
    }

    #[derive(Clone, Copy)]
    pub(crate) struct ImuDelay {
        clock: Option<CycleClock>,
        vendor: Delay,
    }
    impl ImuDelay {
        pub(crate) fn new(clock: Option<CycleClock>) -> Self {
            Self {
                clock,
                vendor: Delay::new(),
            }
        }
    }
    impl DelayNs for ImuDelay {
        fn delay_ns(&mut self, ns: u32) {
            if ns <= 10_000
                && let Some(clock) = self.clock
                && let Some(cycles) = clock.rate().cycles_for_ns(ns)
            {
                let start = clock.now();
                while elapsed(start, clock.now()).is_some_and(|ticks| ticks < cycles) {
                    core::hint::spin_loop();
                }
                return;
            }
            self.vendor.delay_ns(ns);
        }
        fn delay_us(&mut self, us: u32) {
            if us <= 10 {
                self.delay_ns(us * 1000);
            } else {
                self.vendor.delay_micros(us);
            }
        }
        fn delay_ms(&mut self, ms: u32) {
            self.vendor.delay_millis(ms);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rounded_cycles_never_shorten_requested_setup_or_hold() {
        for hz in [1_500_000, 40_000_000, 240_000_000, 400_000_000, 500_000_000] {
            let rate = ClockRate::new(hz).unwrap();
            for ns in [0, 1, 3, 17, 999, 1000, 1001, 10_000, 100_000] {
                let cycles = rate.cycles_for_ns(ns).unwrap();
                assert!(u64::from(cycles) * 1_000_000_000 >= u64::from(ns) * u64::from(hz));
                if cycles != 0 {
                    assert!(u64::from(cycles - 1) * 1_000_000_000 < u64::from(ns) * u64::from(hz));
                }
            }
        }
        assert!(ClockRate::new(0).is_none());
        assert!(
            ClockRate::new(u32::MAX)
                .unwrap()
                .cycles_for_ns(u32::MAX)
                .is_none()
        );
    }
    #[test]
    fn short_cycle_intervals_cross_wrap_and_refuse_ambiguous_gaps() {
        let rate = ClockRate::new(400_000_000).unwrap();
        let budget = rate.cycles_for_ns(100_000).unwrap();
        let start = u32::MAX - 10_000;
        assert_eq!(
            elapsed(start, start.wrapping_add(budget - 1)),
            Some(budget - 1)
        );
        assert_eq!(elapsed(start, start.wrapping_add(budget)), Some(budget));
        assert_eq!(rate.micros_for_cycles(budget), 100);
        assert!(elapsed(start, start.wrapping_add(HALF_WRAP)).is_none());
    }
}
