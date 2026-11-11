use super::TimingStatus;

const SECOND: u64 = 1_000_000_000;
const HISTORY: usize = 8;
const ASSOCIATION_WINDOW_NS: u64 = 100_000_000;
const MAX_RESIDUAL_NS: u64 = 50_000;
const MAX_AGE_NS: u64 = 3 * SECOND;

#[derive(Clone, Copy, Debug)]
pub(crate) struct PpsLabel {
    pub epoch_ms: u64,
    pub header_epoch_ms: u64,
    pub output_delay_ns: u64,
    pub quality: u32,
    pub pulse_error_ns: i32,
    pub config: u32,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Epoch {
    pub time_ns: u64,
    pub uncertainty_ns: u64,
    pub generation: u32,
}

#[derive(Clone, Copy, Default)]
struct Pair {
    gps_ns: u64,
    local_ns: u64,
}

#[derive(Clone, Copy)]
pub(crate) struct ClockSync {
    pulses: [u64; HISTORY],
    pulse_count: usize,
    pairs: [Pair; HISTORY],
    pair_count: usize,
    status: TimingStatus,
}

impl ClockSync {
    pub const fn new() -> Self {
        Self {
            pulses: [0; HISTORY],
            pulse_count: 0,
            pairs: [Pair {
                gps_ns: 0,
                local_ns: 0,
            }; HISTORY],
            pair_count: 0,
            status: TimingStatus {
                pps_locked: false,
                pps_edges: 0,
                pps_pairs: 0,
                pps_rejected: 0,
                pps_age_ms: None,
                fit_residual_ns: 0,
                generation: 0,
                capture_errors: 0,
                imu_timing_rejected: 0,
                gnss_unsynchronized: 0,
            },
        }
    }

    pub fn invalidate(&mut self) {
        self.pair_count = 0;
        self.status.pps_locked = false;
        self.status.generation = self.status.generation.wrapping_add(1);
    }

    /// Bounded-publication failure: preserve counters while making neither a
    /// synchronization nor an unchecked age claim. No projection is performed.
    pub fn invalidate_with_status(&mut self) -> TimingStatus {
        self.invalidate();
        let mut status = self.status;
        status.pps_age_ms = None;
        status
    }

    pub fn pulse(&mut self, at: u64) {
        if self.pulse_count != 0 && at <= self.pulses[self.pulse_count - 1] {
            return;
        }
        if self.pulse_count == HISTORY {
            self.pulses.copy_within(1.., 0);
            self.pulse_count -= 1;
        }
        self.pulses[self.pulse_count] = at;
        self.pulse_count += 1;
        self.status.pps_edges = self.status.pps_edges.wrapping_add(1);
    }

    /// Integer payload labels retain the documented preceding-pulse semantics.
    /// Fractional payloads seen on the fitted receiver are not used as pulse
    /// timestamps: correlate an integer FINE/GPST header with the configured
    /// physical edge instead. Raw payload and offset remain diagnostic evidence.
    pub fn label(&mut self, label: PpsLabel, received_ns: u64) {
        if !(2..=3).contains(&label.quality)
            || label.config & 0xff != 0x20
            || label.config >> 16 != 1000
            || label.pulse_error_ns.unsigned_abs() > 10_000
        {
            self.reject();
            return;
        }
        let header_correlated = !label.epoch_ms.is_multiple_of(1000);
        let epoch_ms = if !header_correlated {
            label.epoch_ms
        } else if label.header_epoch_ms.is_multiple_of(1000)
            && label
                .header_epoch_ms
                .checked_sub(label.epoch_ms)
                .is_some_and(|age| age < 1000)
        {
            // N4 header week/ms is the GPST observation epoch; output delay
            // relates it to transmission. CONFIG selects an integer-second
            // rising edge. The header is already integral: never round payload
            // or header, and never apply the undocumented offsetTime relation.
            // Retained Build17548 captures show payload lags of 20 and 160ms;
            // only its presence within the preceding second is a sanity check.
            label.header_epoch_ms
        } else {
            self.reject();
            return;
        };
        let Some(gps_ns) = epoch_ms.checked_mul(1_000_000) else {
            self.reject();
            return;
        };
        if self.pair_count != 0 && gps_ns == self.pairs[self.pair_count - 1].gps_ns {
            return;
        }

        // After acquisition, the affine fit identifies the pulse independently
        // of UART delays. Startup's wide window only selects one integer second.
        let prediction = if self.pair_count >= 2 {
            self.project(gps_ns)
        } else {
            label
                .header_epoch_ms
                .checked_sub(epoch_ms)
                .and_then(|age| age.checked_mul(1_000_000))
                .and_then(|age| age.checked_add(label.output_delay_ns))
                .and_then(|age| received_ns.checked_sub(age))
        };
        let Some(prediction) = prediction else {
            self.reject();
            return;
        };
        let window = if self.pair_count >= 2 {
            MAX_RESIDUAL_NS
        } else {
            ASSOCIATION_WINDOW_NS
        };
        let mut candidates = self.pulses[..self.pulse_count]
            .iter()
            .copied()
            .filter(|pulse| pulse.abs_diff(prediction) <= window);
        let Some(local_ns) = candidates.next() else {
            self.reject();
            return;
        };
        let age = received_ns.checked_sub(local_ns);
        if candidates.next().is_some()
            || age.is_none_or(|age| age > MAX_AGE_NS)
            || (header_correlated
                && self.pair_count < 2
                && (self.pulses[..self.pulse_count].last() != Some(&local_ns)
                    || age.is_none_or(|age| age > ASSOCIATION_WINDOW_NS)))
        {
            self.reject();
            return;
        }
        if self.pair_count != 0 {
            let previous = self.pairs[self.pair_count - 1];
            if gps_ns.checked_sub(previous.gps_ns) != Some(SECOND)
                || local_ns
                    .checked_sub(previous.local_ns)
                    .is_none_or(|dt| dt.abs_diff(SECOND) > 500_000)
            {
                self.reject();
                return;
            }
        }
        if self.pair_count == HISTORY {
            self.pairs.copy_within(1.., 0);
            self.pair_count -= 1;
        }
        self.pairs[self.pair_count] = Pair { gps_ns, local_ns };
        self.pair_count += 1;
        self.status.pps_pairs = self.status.pps_pairs.wrapping_add(1);
        if self.pair_count >= 3 {
            let residual = self.pairs[..self.pair_count]
                .iter()
                .map(|pair| {
                    self.project(pair.gps_ns)
                        .map_or(u64::MAX, |predicted| predicted.abs_diff(pair.local_ns))
                })
                .max()
                .unwrap_or(0);
            if residual > MAX_RESIDUAL_NS {
                self.reject();
                return;
            }
            self.status.fit_residual_ns = residual as u32;
            self.status.pps_locked = true;
        }
    }

    fn reject(&mut self) {
        self.status.pps_rejected = self.status.pps_rejected.wrapping_add(1);
        self.invalidate();
    }

    fn project(&self, gps_ns: u64) -> Option<u64> {
        if self.pair_count < 2 {
            return None;
        }
        let first = self.pairs[0];
        let last = self.pairs[self.pair_count - 1];
        let span = last.gps_ns.checked_sub(first.gps_ns)?;
        let local_span = last.local_ns.checked_sub(first.local_ns)?;
        if span == 0 {
            return None;
        }
        // Subtract integer origins before scaling; absolute GPS ns lose useful
        // precision when converted directly to floating point.
        let delta = i128::from(gps_ns) - i128::from(last.gps_ns);
        let mapped = i128::from(last.local_ns) + delta * i128::from(local_span) / i128::from(span);
        u64::try_from(mapped).ok()
    }

    pub fn epoch(&mut self, gps_ns: u64, now_ns: u64) -> Option<Epoch> {
        if !self.status(now_ns).pps_locked {
            return None;
        }
        let last = self.pairs[self.pair_count - 1];
        if gps_ns.abs_diff(last.gps_ns) > MAX_AGE_NS {
            return None;
        }
        let time_ns = self.project(gps_ns)?;
        // Includes 1ms wire epoch granularity and residual/extrapolation margin.
        // This is a conservative development model, not a calibrated bound.
        let uncertainty_ns = 500_000
            + u64::from(self.status.fit_residual_ns)
            + gps_ns.abs_diff(last.gps_ns) / 100_000;
        Some(Epoch {
            time_ns,
            uncertainty_ns,
            generation: self.status.generation,
        })
    }

    pub fn status(&mut self, now_ns: u64) -> TimingStatus {
        self.status.pps_age_ms = self.pulses[..self.pulse_count]
            .last()
            .map(|last| (now_ns.saturating_sub(*last) / 1_000_000).min(u64::from(u32::MAX)) as u32);
        if self.status.pps_locked
            && (self.status.pps_age_ms.is_none_or(|age| age > 1500)
                || now_ns.saturating_sub(self.pairs[self.pair_count - 1].local_ns) > MAX_AGE_NS)
        {
            self.invalidate();
        }
        self.status
    }
}
