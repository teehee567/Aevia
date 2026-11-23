//! Continuous mapping of receiver epochs to the IMU clock for development timing.

#[derive(Default)]
pub(super) struct GnssClock {
    last: Option<(u64, u64)>, // receiver milliseconds and local measurement microseconds
    transport_delay_us: Option<u64>,
}

pub(super) struct FixTime {
    pub time_us: u64,
    pub source: &'static str,
    pub discontinuity: bool,
}

impl GnssClock {
    pub fn observe(&mut self, gps_ms: u64, received: u64, pps_us: Option<u64>) -> Option<FixTime> {
        let elapsed = match self.last {
            Some((gps, _)) if gps_ms <= gps => return None,
            Some((gps, _)) => (gps_ms - gps).checked_mul(1000)?,
            None => 0,
        };
        let pps = pps_us.filter(|t| *t <= received && received - *t <= 300_000);
        let (target, source) = if let Some(epoch) = pps {
            let delay = received - epoch;
            self.transport_delay_us = Some(
                self.transport_delay_us
                    .map_or(delay, |old| (15 * old + delay) / 16),
            );
            (epoch, "pps-disciplined-development")
        } else if let Some(delay) = self.transport_delay_us {
            (received.saturating_sub(delay), "pps-holdover-development")
        } else {
            (received, "arrival-only-development")
        };
        let mut discontinuity = false;
        let time_us = if let Some((_, previous)) = self.last {
            // The receiver time of week advances through a PPS generation change.
            // Bound clock slewing so changing the time reference cannot invent
            // an abrupt velocity innovation or duplicate a measurement epoch.
            let predicted = previous.checked_add(elapsed)?;
            if predicted.abs_diff(target) > 300_000 {
                discontinuity = true;
                target
            } else {
                let slew = (elapsed / 100).clamp(1, 1000);
                let correction = target.abs_diff(predicted).min(slew);
                if target >= predicted {
                    predicted + correction
                } else {
                    predicted - correction
                }
            }
        } else {
            target
        };
        self.last = Some((gps_ms, time_us));
        Some(FixTime {
            time_us,
            source,
            discontinuity,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clock_advances_during_pps_loss_and_reacquires_without_a_recording_restart() {
        let mut c = GnssClock::default();
        let first = c.observe(1000, 1_020_000, None).unwrap();
        assert_eq!(first.source, "arrival-only-development");
        let mut last = first.time_us;
        for i in 1..=250 {
            let t = 1_000_000 + i * 20_000;
            let mapped = (!(101..=160).contains(&i)).then_some(t);
            let f = c
                .observe(1000 + i * 20, t + 17_000 + (i % 4) * 1200, mapped)
                .unwrap();
            assert!(!f.discontinuity && (19_800..=20_200).contains(&(f.time_us - last)));
            if i == 150 {
                assert_eq!(f.source, "pps-holdover-development");
            }
            last = f.time_us;
        }
        assert!(last.abs_diff(6_000_000) < 1000);
        assert!(c.observe(6000, 6_020_000, None).is_none());
        let jumped = c.observe(6020, 7_000_000, None).unwrap();
        assert!(jumped.discontinuity);
    }
}
