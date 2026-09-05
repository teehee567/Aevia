//! Bounded chronological scheduling for the single delayed-fusion frontier.

use core::cmp::Ordering;

use crate::time::SessionTime;

pub(crate) const MAX_LIVE_HORIZON_NS: i64 = 500_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct OrderKey {
    pub(crate) time: SessionTime,
    /// Deterministic same-epoch order. Clock/control changes precede inertial
    /// propagation, which precedes GNSS and derived constraints.
    pub(crate) class: u8,
    pub(crate) source: u32,
    pub(crate) sequence: u64,
}

impl Ord for OrderKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.time
            .cmp(&other.time)
            .then_with(|| self.class.cmp(&other.class))
            .then_with(|| self.source.cmp(&other.source))
            .then_with(|| self.sequence.cmp(&other.sequence))
    }
}

impl PartialOrd for OrderKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Scheduled<T: Copy> {
    pub(crate) key: OrderKey,
    pub(crate) value: T,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum QueueError {
    Capacity,
    DuplicateKey,
}

/// Sorted, fixed-capacity queue. Insertion is O(N), with `N` fixed by the
/// preflighted live profile (128 for V2 Mini), and never allocates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ChronologicalQueue<T: Copy, const N: usize> {
    entries: [Option<Scheduled<T>>; N],
    len: usize,
}

impl<T: Copy, const N: usize> ChronologicalQueue<T, N> {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [None; N],
            len: 0,
        }
    }

    pub(crate) const fn len(&self) -> usize {
        self.len
    }

    pub(crate) const fn capacity(&self) -> usize {
        N
    }

    /// Restores the fixed backing array without constructing a second queue.
    pub(crate) fn clear(&mut self) {
        for entry in &mut self.entries {
            *entry = None;
        }
        self.len = 0;
    }

    fn preflight_push(&self, entry: &Scheduled<T>) -> Result<(), QueueError> {
        if self.len == N {
            return Err(QueueError::Capacity);
        }
        for index in 0..self.len {
            if self.entries[index].is_some_and(|existing| existing.key == entry.key) {
                return Err(QueueError::DuplicateKey);
            }
        }
        Ok(())
    }

    /// Inserts after capacity and duplicate-key checks have succeeded. This
    /// cannot fail and is used to commit a small atomic batch without copying
    /// the complete fixed-capacity queue onto the MCU stack.
    fn push_prevalidated(&mut self, entry: Scheduled<T>) {
        let mut insertion = 0;
        while insertion < self.len {
            let Some(existing) = self.entries[insertion] else {
                // This cannot be produced through the safe interface, but do
                // not turn an internal invariant failure into a device panic.
                break;
            };
            match existing.key.cmp(&entry.key) {
                Ordering::Less => insertion += 1,
                Ordering::Equal => break,
                Ordering::Greater => break,
            }
        }
        for index in (insertion..self.len).rev() {
            self.entries[index + 1] = self.entries[index];
        }
        self.entries[insertion] = Some(entry);
        self.len += 1;
    }

    pub(crate) fn first(&self) -> Option<&Scheduled<T>> {
        self.entries.first().and_then(Option::as_ref)
    }

    pub(crate) fn pop_first(&mut self) -> Option<Scheduled<T>> {
        let first = self.entries.first_mut()?.take()?;
        for index in 1..self.len {
            self.entries[index - 1] = self.entries[index];
        }
        self.len -= 1;
        self.entries[self.len] = None;
        Some(first)
    }

    fn try_for_each<E>(
        &self,
        mut visit: impl FnMut(&Scheduled<T>) -> Result<(), E>,
    ) -> Result<(), E> {
        for index in 0..self.len {
            if let Some(entry) = self.entries[index].as_ref() {
                visit(entry)?;
            }
        }
        Ok(())
    }

    fn for_each_mut(&mut self, mut visit: impl FnMut(&mut Scheduled<T>)) {
        for index in 0..self.len {
            if let Some(entry) = self.entries[index].as_mut() {
                visit(entry);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum EnqueueDisposition {
    Queued,
    TooLateForLive,
    CapacityExceeded,
    Duplicate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SchedulerError {
    InvalidFusionDelay,
    ImuTimeNotIncreasing,
    FrontierRegression,
    FrontierBeyondTarget,
    TimeOverflow,
    AlreadyFinishing,
    NoTrustedImu,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct FrontierScheduler<T: Copy, const N: usize> {
    fusion_delay_ns: i64,
    measurements: ChronologicalQueue<T, N>,
    latest_trusted_imu: Option<SessionTime>,
    processed_frontier: Option<SessionTime>,
    finishing: bool,
}

impl<T: Copy, const N: usize> FrontierScheduler<T, N> {
    /// Valid inactive representation suitable for static caller-owned storage.
    pub(crate) const fn placeholder() -> Self {
        Self {
            fusion_delay_ns: 0,
            measurements: ChronologicalQueue::new(),
            latest_trusted_imu: None,
            processed_frontier: None,
            finishing: false,
        }
    }

    pub(crate) const fn validate_fusion_delay(fusion_delay_ns: i64) -> Result<(), SchedulerError> {
        if fusion_delay_ns < 0 || fusion_delay_ns > MAX_LIVE_HORIZON_NS {
            Err(SchedulerError::InvalidFusionDelay)
        } else {
            Ok(())
        }
    }

    /// Initializes an existing scheduler in place. Failure leaves it empty.
    pub(crate) fn initialize(&mut self, fusion_delay_ns: i64) -> Result<(), SchedulerError> {
        self.reset();
        Self::validate_fusion_delay(fusion_delay_ns)?;
        self.fusion_delay_ns = fusion_delay_ns;
        Ok(())
    }

    pub(crate) fn reset(&mut self) {
        self.fusion_delay_ns = 0;
        self.measurements.clear();
        self.latest_trusted_imu = None;
        self.processed_frontier = None;
        self.finishing = false;
    }

    #[cfg(test)]
    pub(crate) fn new(fusion_delay_ns: i64) -> Result<Self, SchedulerError> {
        let mut result = Self::placeholder();
        result.initialize(fusion_delay_ns)?;
        Ok(result)
    }

    pub(crate) fn observe_trusted_imu(&mut self, time: SessionTime) -> Result<(), SchedulerError> {
        if self.finishing {
            return Err(SchedulerError::AlreadyFinishing);
        }
        if self
            .latest_trusted_imu
            .is_some_and(|previous| time <= previous)
        {
            return Err(SchedulerError::ImuTimeNotIncreasing);
        }
        self.latest_trusted_imu = Some(time);
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn enqueue(&mut self, entry: Scheduled<T>) -> EnqueueDisposition {
        let disposition = self.classify_enqueue(&entry);
        if disposition != EnqueueDisposition::Queued {
            return disposition;
        }
        self.measurements.push_prevalidated(entry);
        EnqueueDisposition::Queued
    }

    /// Read-only classification used by bounded transactional ingest.
    pub(super) fn classify_enqueue(&self, entry: &Scheduled<T>) -> EnqueueDisposition {
        if self
            .processed_frontier
            .is_some_and(|frontier| entry.key.time <= frontier)
        {
            return EnqueueDisposition::TooLateForLive;
        }
        match self.measurements.preflight_push(entry) {
            Ok(()) => EnqueueDisposition::Queued,
            Err(QueueError::Capacity) => EnqueueDisposition::CapacityExceeded,
            Err(QueueError::DuplicateKey) => EnqueueDisposition::Duplicate,
        }
    }

    /// Atomically enqueues up to two already validated observations. The
    /// queue is read-only until all capacity and duplicate checks pass.
    pub(super) fn enqueue_pair_atomic(
        &mut self,
        entries: &[Option<Scheduled<T>>; 2],
    ) -> Result<[Option<EnqueueDisposition>; 2], EnqueueDisposition> {
        let mut dispositions = [None; 2];
        let mut queued = 0_usize;
        for (index, entry) in entries.iter().enumerate() {
            let Some(entry) = entry else {
                continue;
            };
            let disposition = self.classify_enqueue(entry);
            match disposition {
                EnqueueDisposition::Queued => {
                    for previous_index in 0..index {
                        if dispositions[previous_index] == Some(EnqueueDisposition::Queued)
                            && entries[previous_index]
                                .is_some_and(|previous| previous.key == entry.key)
                        {
                            return Err(EnqueueDisposition::Duplicate);
                        }
                    }
                    queued += 1;
                }
                EnqueueDisposition::TooLateForLive => {}
                EnqueueDisposition::CapacityExceeded | EnqueueDisposition::Duplicate => {
                    return Err(disposition);
                }
            }
            dispositions[index] = Some(disposition);
        }
        if queued > self.measurements.capacity() - self.measurements.len() {
            return Err(EnqueueDisposition::CapacityExceeded);
        }

        for (entry, disposition) in entries.iter().zip(dispositions) {
            if let (Some(entry), Some(EnqueueDisposition::Queued)) = (*entry, disposition) {
                // Capacity and all existing/intra-pair duplicates were checked
                // above, so each insertion is infallible.
                self.measurements.push_prevalidated(entry);
            }
        }
        Ok(dispositions)
    }

    pub(crate) fn target(&self) -> Result<SessionTime, SchedulerError> {
        let latest = self
            .latest_trusted_imu
            .ok_or(SchedulerError::NoTrustedImu)?;
        if self.finishing {
            return Ok(latest);
        }
        let target = latest
            .as_ns()
            .checked_sub(self.fusion_delay_ns)
            .ok_or(SchedulerError::TimeOverflow)?;
        Ok(SessionTime::from_ns(target))
    }

    pub(crate) fn next_measurement(&self) -> Result<Option<&Scheduled<T>>, SchedulerError> {
        let target = self.target()?;
        Ok(self
            .measurements
            .first()
            .filter(|entry| entry.key.time <= target))
    }

    pub(crate) fn pop_next_measurement(&mut self) -> Result<Option<Scheduled<T>>, SchedulerError> {
        if self.next_measurement()?.is_none() {
            return Ok(None);
        }
        Ok(self.measurements.pop_first())
    }

    /// Advances only after the caller has completed all propagation and
    /// measurement work through `new_frontier`.
    pub(crate) fn commit_frontier(
        &mut self,
        new_frontier: SessionTime,
    ) -> Result<(), SchedulerError> {
        if self
            .processed_frontier
            .is_some_and(|previous| new_frontier < previous)
        {
            return Err(SchedulerError::FrontierRegression);
        }
        if new_frontier > self.target()? {
            return Err(SchedulerError::FrontierBeyondTarget);
        }
        self.processed_frontier = Some(new_frontier);
        Ok(())
    }

    pub(crate) fn processed_frontier(&self) -> Option<SessionTime> {
        self.processed_frontier
    }

    pub(crate) fn latest_trusted_imu(&self) -> Option<SessionTime> {
        self.latest_trusted_imu
    }

    pub(crate) fn queued_measurements(&self) -> usize {
        self.measurements.len()
    }

    pub(super) fn try_for_each_measurement<E>(
        &self,
        visit: impl FnMut(&Scheduled<T>) -> Result<(), E>,
    ) -> Result<(), E> {
        self.measurements.try_for_each(visit)
    }

    pub(super) fn for_each_measurement_mut(&mut self, visit: impl FnMut(&mut Scheduled<T>)) {
        self.measurements.for_each_mut(visit);
    }

    /// Irrevocably removes the normal delay. The terminal target is the last
    /// complete trusted IMU epoch and is never extrapolated.
    pub(crate) fn finish(&mut self) -> Result<SessionTime, SchedulerError> {
        if self.finishing {
            return Err(SchedulerError::AlreadyFinishing);
        }
        self.finishing = true;
        self.latest_trusted_imu.ok_or(SchedulerError::NoTrustedImu)
    }

    pub(crate) const fn is_finishing(&self) -> bool {
        self.finishing
    }
}

/// Deterministic corrected-frontier operation credit. Time is observed for
/// qualification, but frontier-loop termination depends only on this integer
/// and fixed capacities. Fixed-capacity work outside the frontier is not
/// charged here.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct WorkQuota {
    remaining: u16,
}

impl WorkQuota {
    pub(crate) const fn new(credits: u16) -> Self {
        Self { remaining: credits }
    }

    pub(crate) fn take(&mut self, cost: u16) -> bool {
        if cost > self.remaining {
            return false;
        }
        self.remaining -= cost;
        true
    }

    pub(crate) const fn remaining(self) -> u16 {
        self.remaining
    }
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct MetricWatermark {
    pub(crate) consumed_through: SessionTime,
    pub(crate) maximum_future_support_ns: i64,
}

#[cfg(test)]
impl MetricWatermark {
    pub(crate) fn finalized_through(self) -> Result<SessionTime, SchedulerError> {
        if self.maximum_future_support_ns < 0 {
            return Err(SchedulerError::TimeOverflow);
        }
        self.consumed_through
            .as_ns()
            .checked_sub(self.maximum_future_support_ns)
            .map(SessionTime::from_ns)
            .ok_or(SchedulerError::TimeOverflow)
    }
}

/// FIFO history for already time-ordered high-rate IMU intervals. A full ring
/// is an explicit overload; unread evidence is never overwritten.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct FixedRing<T: Copy, const N: usize> {
    entries: [Option<T>; N],
    head: usize,
    len: usize,
}

impl<T: Copy, const N: usize> FixedRing<T, N> {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [None; N],
            head: 0,
            len: 0,
        }
    }

    pub(crate) fn push_back(&mut self, value: T) -> Result<(), QueueError> {
        if self.len == N || N == 0 {
            return Err(QueueError::Capacity);
        }
        let index = (self.head + self.len) % N;
        self.entries[index] = Some(value);
        self.len += 1;
        Ok(())
    }

    pub(crate) fn get(&self, offset: usize) -> Option<&T> {
        if offset >= self.len || N == 0 {
            None
        } else {
            self.entries[(self.head + offset) % N].as_ref()
        }
    }

    #[cfg(test)]
    pub(crate) fn front(&self) -> Option<&T> {
        self.get(0)
    }

    pub(crate) fn pop_front(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let value = self.entries[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        value
    }

    pub(crate) const fn len(&self) -> usize {
        self.len
    }

    pub(crate) const fn available(&self) -> usize {
        N - self.len
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(time: i64, sequence: u64, value: u8) -> Scheduled<u8> {
        Scheduled {
            key: OrderKey {
                time: SessionTime::from_ns(time),
                class: 3,
                source: 1,
                sequence,
            },
            value,
        }
    }

    #[test]
    fn arbitrary_arrival_order_is_drained_chronologically() {
        let mut scheduler = FrontierScheduler::<u8, 8>::new(100).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(1_000))
            .unwrap();
        assert_eq!(
            scheduler.enqueue(entry(850, 3, 3)),
            EnqueueDisposition::Queued
        );
        assert_eq!(
            scheduler.enqueue(entry(700, 1, 1)),
            EnqueueDisposition::Queued
        );
        assert_eq!(
            scheduler.enqueue(entry(800, 2, 2)),
            EnqueueDisposition::Queued
        );
        assert_eq!(scheduler.pop_next_measurement().unwrap().unwrap().value, 1);
        assert_eq!(scheduler.pop_next_measurement().unwrap().unwrap().value, 2);
        assert_eq!(scheduler.pop_next_measurement().unwrap().unwrap().value, 3);
    }

    #[test]
    fn observation_at_committed_frontier_is_too_late() {
        let mut scheduler = FrontierScheduler::<u8, 4>::new(100).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(1_000))
            .unwrap();
        scheduler
            .commit_frontier(SessionTime::from_ns(900))
            .unwrap();
        assert_eq!(
            scheduler.enqueue(entry(900, 1, 1)),
            EnqueueDisposition::TooLateForLive
        );
        assert_eq!(
            scheduler.enqueue(entry(901, 2, 2)),
            EnqueueDisposition::Queued
        );
    }

    #[test]
    fn finish_advances_target_to_last_trusted_imu_without_extrapolation() {
        let mut scheduler = FrontierScheduler::<u8, 4>::new(100).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(1_000))
            .unwrap();
        assert_eq!(scheduler.target().unwrap(), SessionTime::from_ns(900));
        assert_eq!(scheduler.finish().unwrap(), SessionTime::from_ns(1_000));
        assert_eq!(scheduler.target().unwrap(), SessionTime::from_ns(1_000));
    }

    #[test]
    fn capacity_overflow_is_explicit_and_does_not_overwrite() {
        let mut scheduler = FrontierScheduler::<u8, 2>::new(0).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(10))
            .unwrap();
        assert_eq!(
            scheduler.enqueue(entry(1, 1, 1)),
            EnqueueDisposition::Queued
        );
        assert_eq!(
            scheduler.enqueue(entry(2, 2, 2)),
            EnqueueDisposition::Queued
        );
        assert_eq!(
            scheduler.enqueue(entry(3, 3, 3)),
            EnqueueDisposition::CapacityExceeded
        );
        assert_eq!(scheduler.queued_measurements(), 2);
    }

    #[test]
    fn pair_capacity_failure_is_atomic_without_copying_the_queue() {
        let mut scheduler = FrontierScheduler::<u8, 2>::new(0).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(10))
            .unwrap();
        scheduler.enqueue(entry(1, 1, 1));
        assert_eq!(
            scheduler.enqueue_pair_atomic(&[Some(entry(2, 2, 2)), Some(entry(3, 3, 3))]),
            Err(EnqueueDisposition::CapacityExceeded)
        );
        assert_eq!(scheduler.queued_measurements(), 1);
        assert_eq!(scheduler.pop_next_measurement().unwrap().unwrap().value, 1);
    }

    #[test]
    fn pair_duplicate_failure_is_atomic_for_existing_and_intra_pair_keys() {
        let mut scheduler = FrontierScheduler::<u8, 4>::new(0).unwrap();
        scheduler
            .observe_trusted_imu(SessionTime::from_ns(10))
            .unwrap();
        let duplicate = entry(2, 2, 2);
        assert_eq!(
            scheduler.enqueue_pair_atomic(&[Some(duplicate), Some(duplicate)]),
            Err(EnqueueDisposition::Duplicate)
        );
        assert_eq!(scheduler.queued_measurements(), 0);

        scheduler.enqueue(entry(1, 1, 1));
        assert_eq!(
            scheduler.enqueue_pair_atomic(&[Some(entry(3, 3, 3)), Some(entry(1, 1, 9))]),
            Err(EnqueueDisposition::Duplicate)
        );
        assert_eq!(scheduler.queued_measurements(), 1);
        assert_eq!(scheduler.pop_next_measurement().unwrap().unwrap().value, 1);
    }

    #[test]
    fn work_quota_never_underflows() {
        let mut quota = WorkQuota::new(5);
        assert!(quota.take(3));
        assert!(!quota.take(3));
        assert_eq!(quota.remaining(), 2);
    }

    #[test]
    fn metric_watermark_is_distinct_from_navigation_frontier() {
        let watermark = MetricWatermark {
            consumed_through: SessionTime::from_ns(5_000),
            maximum_future_support_ns: 2_000,
        };
        assert_eq!(
            watermark.finalized_through().unwrap(),
            SessionTime::from_ns(3_000)
        );
    }

    #[test]
    fn fixed_ring_wraps_without_reordering_or_overwrite() {
        let mut ring = FixedRing::<u8, 3>::new();
        ring.push_back(1).unwrap();
        ring.push_back(2).unwrap();
        assert_eq!(ring.pop_front(), Some(1));
        ring.push_back(3).unwrap();
        ring.push_back(4).unwrap();
        assert_eq!(ring.push_back(5), Err(QueueError::Capacity));
        assert_eq!(ring.pop_front(), Some(2));
        assert_eq!(ring.pop_front(), Some(3));
        assert_eq!(ring.pop_front(), Some(4));
    }
}
