//! Publish clock state without running its wide-integer arithmetic while the
//! cross-core critical section masks the higher-priority IMU executor.
use super::TimingStatus;
use super::clock::ClockSync;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

const MAX_ATTEMPTS: usize = 3;

#[derive(Clone, Copy)]
struct VersionedClock {
    state: ClockSync,
    revision: u64,
}

pub(crate) struct SharedClock {
    inner: Mutex<CriticalSectionRawMutex, RefCell<VersionedClock>>,
}

impl SharedClock {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(VersionedClock {
                state: ClockSync::new(),
                revision: 0,
            })),
        }
    }

    /// The operation may be retried after a concurrent publication, so it must
    /// affect only the supplied clock. Snapshot copying and validated commit
    /// are the only interrupt-masked work. Projection, fit residuals and age
    /// arithmetic execute with acquisition interrupts enabled.
    ///
    /// Every writer participates, including status expiry and pulse capture.
    /// Checking the publication revision (not only fit generation) prevents a
    /// fit or read from erasing a newly captured pulse or concurrent invalidation.
    /// Repeated contention invalidates the latest fit and returns its retained
    /// diagnostics. It never falls back to arithmetic inside the critical
    /// section or spins without a bound. A discarded physical capture remains
    /// in its hardware latch for the next maintenance tick; a discarded label
    /// requires ordinary reacquisition before navigation resumes.
    pub fn update<T>(
        &self,
        mut operation: impl FnMut(&mut ClockSync) -> T,
    ) -> Result<T, TimingStatus> {
        for _ in 0..MAX_ATTEMPTS {
            let mut snapshot = self.inner.lock(|current| *current.borrow());
            let result = operation(&mut snapshot.state);
            let committed = self.inner.lock(|current| {
                let mut current = current.borrow_mut();
                if current.revision != snapshot.revision {
                    return false;
                }
                snapshot.revision = snapshot.revision.wrapping_add(1);
                *current = snapshot;
                true
            });
            if committed {
                return Ok(result);
            }
        }
        Err(self.inner.lock(|current| {
            let mut current = current.borrow_mut();
            current.revision = current.revision.wrapping_add(1);
            current.state.invalidate_with_status()
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::timing::clock::PpsLabel;

    const SECOND: u64 = 1_000_000_000;

    fn label(second: u64) -> PpsLabel {
        PpsLabel {
            epoch_ms: second * 1000,
            header_epoch_ms: second * 1000,
            output_delay_ns: 20_000_000,
            quality: 3,
            pulse_error_ns: 5,
            config: (1000 << 16) | 0x20,
        }
    }

    #[test]
    fn concurrent_capture_survives_a_retried_label_without_double_counting() {
        let shared = SharedClock::new();
        shared.update(|clock| clock.pulse(10 * SECOND)).unwrap();
        let mut attempts = 0;
        shared
            .update(|clock| {
                clock.label(label(100), 10 * SECOND + 20_000_000);
                attempts += 1;
                if attempts == 1 {
                    // Simulate a competing publication between snapshot and commit.
                    shared.update(|clock| clock.pulse(11 * SECOND)).unwrap();
                }
            })
            .unwrap();
        let status = shared.update(|clock| clock.status(11 * SECOND)).unwrap();
        assert_eq!(attempts, 2);
        assert_eq!(status.pps_edges, 2);
        assert_eq!(status.pps_pairs, 1);
        assert_eq!(status.pps_rejected, 0);
        assert_eq!(status.pps_age_ms, Some(0));
    }

    #[test]
    fn concurrent_invalidation_cannot_be_overwritten_or_return_a_stale_epoch() {
        let shared = SharedClock::new();
        for second in 100..103 {
            shared
                .update(|clock| {
                    let local = (second - 90) * SECOND;
                    clock.pulse(local);
                    clock.label(label(second), local + 20_000_000);
                })
                .unwrap();
        }
        assert!(
            shared
                .update(|clock| clock.status(12 * SECOND))
                .unwrap()
                .pps_locked
        );
        let mut attempts = 0;
        let epoch = shared
            .update(|clock| {
                let epoch = clock.epoch(102 * SECOND, 12 * SECOND + 20_000_000);
                attempts += 1;
                if attempts == 1 {
                    assert!(epoch.is_some());
                    shared.update(ClockSync::invalidate).unwrap();
                }
                epoch
            })
            .unwrap();
        assert_eq!(attempts, 2);
        assert!(epoch.is_none());
        let status = shared
            .update(|clock| clock.status(12 * SECOND + 20_000_000))
            .unwrap();
        assert!(!status.pps_locked);
        assert_eq!(status.generation, 1);
    }

    #[test]
    fn repeated_conflicts_are_bounded_and_invalidate_without_losing_latest_counters() {
        let shared = SharedClock::new();
        let mut attempts = 0;
        let failure = shared
            .update(|clock| {
                clock.pulse(SECOND);
                attempts += 1;
                shared
                    .update(|clock| clock.pulse((10 + attempts) * SECOND))
                    .unwrap();
            })
            .unwrap_err();
        assert_eq!(attempts, MAX_ATTEMPTS as u64);
        assert!(!failure.pps_locked);
        assert_eq!(failure.generation, 1);
        assert_eq!(failure.pps_edges, MAX_ATTEMPTS as u32);
        assert_eq!(failure.pps_age_ms, None);
        let status = shared
            .update(|clock| clock.status((10 + attempts) * SECOND))
            .unwrap();
        assert_eq!(status.pps_edges, MAX_ATTEMPTS as u32);
        assert_eq!(status.pps_age_ms, Some(0));
    }
}
