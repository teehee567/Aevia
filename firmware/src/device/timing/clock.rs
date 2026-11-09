use super::TimingStatus;

pub(crate) const PPS_COMMAND: &[u8] = b"CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0\r\n";

/// N4 CONFIG replies use an XOR over the complete body, including '$'.
/// Verify all timing parameters, including RF/user delays absent in PPSSTATUS.
pub(crate) fn valid_pps_config(line: &[u8]) -> bool {
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let Some(star) = line.iter().rposition(|byte| *byte == b'*') else {
        return false;
    };
    if line.len() != star + 3 {
        return false;
    }
    let checksum = core::str::from_utf8(&line[star + 1..])
        .ok()
        .and_then(|hex| u8::from_str_radix(hex, 16).ok());
    checksum == Some(line[..star].iter().fold(0, |xor, byte| xor ^ byte))
        && line[..star].strip_prefix(b"$CONFIG,PPS,") == Some(&PPS_COMMAND[..PPS_COMMAND.len() - 2])
}

const SECOND: u64 = 1_000_000_000;
const WEEK_MS: u64 = 604_800_000;
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

    /// PPSSTATUS's payload labels the pulse; its header can be a second later.
    /// Require ENABLE/GPS/rising/1000ms readback, quality 2/3 and whole seconds.
    pub fn label(&mut self, label: PpsLabel, received_ns: u64) {
        if !(2..=3).contains(&label.quality)
            || label.config & 0xff != 0x20
            || label.config >> 16 != 1000
            || label.epoch_ms % 1000 != 0
            || label.pulse_error_ns.unsigned_abs() > 10_000
        {
            self.reject();
            return;
        }
        let Some(gps_ns) = label.epoch_ms.checked_mul(1_000_000) else {
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
                .checked_sub(label.epoch_ms)
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
        if candidates.next().is_some() || received_ns.saturating_sub(local_ns) > MAX_AGE_NS {
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

pub(crate) fn gps_ms(week: u32, tow_ms: u32) -> Option<u64> {
    if u64::from(tow_ms) >= WEEK_MS {
        return None;
    }
    u64::from(week)
        .checked_mul(WEEK_MS)?
        .checked_add(u64::from(tow_ms))
}
