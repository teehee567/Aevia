//! CPU0-only USB sample backlog with a tested, explicitly reserved PSRAM tail.
//!
//! Acquisition interrupts never access this queue. All methods run in CPU0's
//! ordinary executor context under the application's existing state mutex. CPU1
//! tests the full fitted memory, flushes its test cache lines, then releases the
//! final MiB to one queue. Its navigation arena excludes that tail permanently.
//! No live sample is shared between CPU caches or referenced through this ring.
//!
//! Boot starts with the existing internal 128-sample queue. Promotion waits for
//! that queue to become empty; copying an entire backlog inside the state
//! critical section would delay acquisition interrupts. A recording start's
//! intentional clear also permits promotion. Unknown protection or a failed
//! memory test leaves the internal queue available with a diagnostic reason.

use crate::sample::Sample;
use core::mem::{MaybeUninit, size_of};
use heapless::Deque;

pub const INTERNAL_CAPACITY: usize = 128;
pub const PSRAM_CAPACITY: usize = 4096;
const PSRAM_RESERVATION_BYTES: usize = 1024 * 1024;
const _: () = assert!(PSRAM_CAPACITY * size_of::<Sample>() <= PSRAM_RESERVATION_BYTES);
#[cfg(target_arch = "riscv32")]
const _: () = {
    assert!(PSRAM_CAPACITY * size_of::<Sample>() <= crate::navigation::memory::PREVIEW_BYTES);
    assert!(
        crate::navigation::memory::PREVIEW_BASE.is_multiple_of(core::mem::align_of::<Sample>())
    );
};

#[cfg(target_arch = "riscv32")]
static CLAIMED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Raw, read-only CSR evidence captured immediately after a promotion refusal.
/// Array indices are hardware entry numbers; addresses retain their CSR encoding.
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct ProtectionSnapshot {
    pub cpu: u8,
    pub pma_cfg: [u32; 16],
    pub pma_addr: [u32; 16],
    pub pmp_cfg: [u32; 4],
    pub pmp_addr: [u32; 16],
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct Status {
    pub backing: &'static str,
    pub capacity: usize,
    pub pending: usize,
    pub psram_tested: bool,
    pub error: Option<&'static str>,
    pub protection: Option<ProtectionSnapshot>,
}

/// A non-owning descriptor whose only backing is the exclusively claimed raw
/// PSRAM reservation. No reference into that storage escapes an operation.
struct Ring {
    address: usize,
    capacity: usize,
    head: usize,
    len: usize,
}

// Returning the rejected value preserves the existing allocator-free Deque API.
#[allow(clippy::result_large_err)]
impl Ring {
    /// # Safety
    /// `address` must be aligned writable storage for `capacity` Samples, with
    /// exclusive access for this descriptor's lifetime. All slots begin empty.
    unsafe fn new(address: usize, capacity: usize) -> Self {
        Self {
            address,
            capacity,
            head: 0,
            len: 0,
        }
    }

    fn slot(&self, index: usize) -> *mut MaybeUninit<Sample> {
        debug_assert!(index < self.capacity);
        // SAFETY: creation established an aligned allocation of capacity slots.
        unsafe { (self.address as *mut MaybeUninit<Sample>).add(index) }
    }

    fn push_back(&mut self, sample: Sample) -> Result<(), Sample> {
        if self.len == self.capacity {
            return Err(sample);
        }
        let index = (self.head + self.len) % self.capacity;
        // SAFETY: this unoccupied slot is exclusively owned by the ring.
        unsafe { self.slot(index).write(MaybeUninit::new(sample)) };
        self.len += 1;
        Ok(())
    }

    fn push_front(&mut self, sample: Sample) -> Result<(), Sample> {
        if self.len == self.capacity {
            return Err(sample);
        }
        self.head = if self.head == 0 {
            self.capacity - 1
        } else {
            self.head - 1
        };
        // SAFETY: the preceding slot is unoccupied when the ring is not full.
        unsafe { self.slot(self.head).write(MaybeUninit::new(sample)) };
        self.len += 1;
        Ok(())
    }

    fn pop_front(&mut self) -> Option<Sample> {
        if self.len == 0 {
            return None;
        }
        // SAFETY: every occupied slot was initialized by push, and Sample is
        // Copy with no destructor. The returned value owns no PSRAM reference.
        let sample = unsafe { self.slot(self.head).read().assume_init() };
        self.head = (self.head + 1) % self.capacity;
        self.len -= 1;
        Some(sample)
    }

    fn clear(&mut self) {
        // Sample is Copy; forgotten occupied slots need no destructor.
        self.head = 0;
        self.len = 0;
    }
}

pub struct PreviewQueue {
    internal: Deque<Sample, INTERNAL_CAPACITY>,
    external: Option<Ring>,
    attempted: bool,
    error: Option<&'static str>,
    protection: Option<ProtectionSnapshot>,
}

impl Default for PreviewQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(clippy::result_large_err)]
impl PreviewQueue {
    pub const fn new() -> Self {
        Self {
            internal: Deque::new(),
            external: None,
            attempted: false,
            error: None,
            protection: None,
        }
    }

    pub fn push_back(&mut self, sample: Sample) -> Result<(), Sample> {
        self.check_owner();
        self.try_promote();
        match &mut self.external {
            Some(ring) => ring.push_back(sample),
            None => self.internal.push_back(sample),
        }
    }

    pub fn push_front(&mut self, sample: Sample) -> Result<(), Sample> {
        self.check_owner();
        self.try_promote();
        match &mut self.external {
            Some(ring) => ring.push_front(sample),
            None => self.internal.push_front(sample),
        }
    }

    pub fn pop_front(&mut self) -> Option<Sample> {
        self.check_owner();
        let sample = match &mut self.external {
            Some(ring) => ring.pop_front(),
            None => self.internal.pop_front(),
        };
        self.try_promote();
        sample
    }

    pub fn clear(&mut self) {
        self.check_owner();
        match &mut self.external {
            Some(ring) => ring.clear(),
            None => self.internal.clear(),
        }
        self.try_promote();
    }

    pub fn len(&self) -> usize {
        self.external
            .as_ref()
            .map_or(self.internal.len(), |ring| ring.len)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn capacity(&self) -> usize {
        self.external
            .as_ref()
            .map_or(INTERNAL_CAPACITY, |ring| ring.capacity)
    }

    pub fn status(&self) -> Status {
        Status {
            backing: if self.external.is_some() {
                "psram"
            } else {
                "internal"
            },
            capacity: self.capacity(),
            pending: self.len(),
            psram_tested: memory_tested(),
            error: self.error,
            protection: self.protection,
        }
    }

    fn check_owner(&self) {
        #[cfg(target_arch = "riscv32")]
        if self.external.is_some() {
            assert_eq!(
                esp_hal::system::Cpu::current(),
                esp_hal::system::Cpu::ProCpu,
                "PSRAM preview queue accessed outside CPU0"
            );
        }
    }

    #[cfg_attr(not(target_arch = "riscv32"), allow(clippy::needless_return))]
    fn try_promote(&mut self) {
        if self.external.is_some() || self.attempted || !self.internal.is_empty() {
            return;
        }
        #[cfg(target_arch = "riscv32")]
        {
            use crate::navigation::memory;
            use core::sync::atomic::Ordering;
            if memory::preview_test_failed() {
                self.attempted = true;
                self.error = Some("psram-startup-test-failed");
                return;
            }
            if !memory::preview_ready() {
                return;
            }
            self.attempted = true;
            if esp_hal::system::Cpu::current() != esp_hal::system::Cpu::ProCpu {
                self.error = Some("psram-preview-wrong-core");
                return;
            }
            if CLAIMED
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                self.error = Some("psram-preview-already-owned");
                return;
            }
            // SAFETY: readiness acquired the completion of CPU1's destructive
            // full-memory test, and the atomic claim permits only one owner.
            if let Err(error) = unsafe { memory::enable_core0_preview_access() } {
                CLAIMED.store(false, Ordering::Release);
                self.protection = Some(memory::protection_snapshot());
                self.error = Some(match error {
                    memory::Error::WrongCore => "psram-preview-wrong-core",
                    _ => "psram-preview-unexpected-protection",
                });
                return;
            }
            debug_assert!(PSRAM_CAPACITY * size_of::<Sample>() <= memory::PREVIEW_BYTES);
            // SAFETY: this aligned reservation is outside every navigation
            // arena, was fully tested, and is now owned only by this CPU0 ring.
            self.external = Some(unsafe { Ring::new(memory::PREVIEW_BASE, PSRAM_CAPACITY) });
        }
    }
}

fn memory_tested() -> bool {
    #[cfg(target_arch = "riscv32")]
    {
        crate::navigation::memory::preview_ready()
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sequence: u64) -> Sample {
        Sample::Imu {
            t_us: sequence,
            dt_us: 1376,
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            valid: true,
            captured_ns: sequence * 1000,
            interval_ns: 1_376_000,
            effective_end_ns: sequence * 1000,
            timing_uncertainty_ns: 100,
            aligned_gyro: [7.0, 8.0, 9.0],
        }
    }
    fn sequence(sample: Sample) -> u64 {
        match sample {
            Sample::Imu { t_us, .. } | Sample::Gnss { t_us, .. } => t_us,
        }
    }
    fn push(queue: &mut PreviewQueue, value: u64) {
        assert!(queue.push_back(sample(value)).is_ok());
    }

    #[test]
    fn internal_fallback_has_original_capacity_and_never_overwrites() {
        let mut queue = PreviewQueue::new();
        for i in 0..INTERNAL_CAPACITY {
            push(&mut queue, i as u64);
        }
        assert_eq!(sequence(queue.push_back(sample(999)).unwrap_err()), 999);
        assert_eq!(queue.len(), INTERNAL_CAPACITY);
        assert_eq!(queue.status().backing, "internal");
        assert!(!queue.status().psram_tested);
        for i in 0..INTERNAL_CAPACITY {
            assert_eq!(sequence(queue.pop_front().unwrap()), i as u64);
        }
        assert!(queue.pop_front().is_none());
    }

    #[test]
    fn external_capacity_wrap_restore_and_clear_preserve_order() {
        let mut backing = std::vec![MaybeUninit::<Sample>::uninit(); PSRAM_CAPACITY];
        let mut queue = PreviewQueue::new();
        // SAFETY: backing outlives queue, is aligned, and is never otherwise used.
        queue.external = Some(unsafe { Ring::new(backing.as_mut_ptr() as usize, backing.len()) });
        for i in 0..PSRAM_CAPACITY {
            push(&mut queue, i as u64);
        }
        assert!(queue.push_back(sample(999_999)).is_err());
        assert!(queue.push_front(sample(888_888)).is_err());
        for i in 0..1234 {
            assert_eq!(sequence(queue.pop_front().unwrap()), i);
        }
        for i in PSRAM_CAPACITY..PSRAM_CAPACITY + 1234 {
            push(&mut queue, i as u64);
        }
        let first = queue.pop_front().unwrap();
        assert!(queue.push_front(first).is_ok());
        for i in 1234..PSRAM_CAPACITY + 1234 {
            assert_eq!(sequence(queue.pop_front().unwrap()), i as u64);
        }
        assert!(queue.is_empty());
        assert_eq!(queue.capacity(), PSRAM_CAPACITY);
        assert_eq!(queue.status().backing, "psram");
        push(&mut queue, 10);
        queue.clear();
        assert!(queue.pop_front().is_none());
        push(&mut queue, 11);
        assert_eq!(sequence(queue.pop_front().unwrap()), 11);
    }

    #[test]
    fn bounded_operations_match_reference_deque_across_many_wraps() {
        use std::collections::VecDeque;
        let mut backing = std::vec![MaybeUninit::<Sample>::uninit(); 17];
        let mut queue = PreviewQueue::new();
        // SAFETY: exclusive aligned test storage outlives the queue.
        queue.external = Some(unsafe { Ring::new(backing.as_mut_ptr() as usize, backing.len()) });
        let mut expected = VecDeque::new();
        let mut state = 17_u32;
        for i in 0..20_000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            match state % 7 {
                0..=2 => {
                    let accepted = queue.push_back(sample(i)).is_ok();
                    assert_eq!(accepted, expected.len() < 17);
                    if accepted {
                        expected.push_back(i);
                    }
                }
                3 => {
                    let accepted = queue.push_front(sample(i)).is_ok();
                    assert_eq!(accepted, expected.len() < 17);
                    if accepted {
                        expected.push_front(i);
                    }
                }
                4..=5 => assert_eq!(queue.pop_front().map(sequence), expected.pop_front()),
                _ => {
                    queue.clear();
                    expected.clear();
                }
            }
            assert_eq!(queue.len(), expected.len());
        }
    }

    #[test]
    fn external_backlog_fits_the_reserved_megabyte_without_static_growth() {
        assert!(PSRAM_CAPACITY * size_of::<Sample>() <= PSRAM_RESERVATION_BYTES);
        assert!(size_of::<PreviewQueue>() < INTERNAL_CAPACITY * size_of::<Sample>() + 384);
    }

    #[test]
    fn refused_protection_is_reported_losslessly_with_bounded_json() {
        let mut queue = PreviewQueue::new();
        queue.error = Some("psram-preview-unexpected-protection");
        queue.protection = Some(ProtectionSnapshot {
            cpu: 0,
            pma_cfg: [u32::MAX; 16],
            pma_addr: [u32::MAX; 16],
            pmp_cfg: [u32::MAX; 4],
            pmp_addr: [u32::MAX; 16],
        });
        let mut buffer = [0; 1024];
        let written = serde_json_core::to_slice(&queue.status(), &mut buffer).unwrap();
        let json = core::str::from_utf8(&buffer[..written]).unwrap();
        assert!(json.contains("\"backing\":\"internal\""));
        assert!(json.contains("\"capacity\":128"));
        assert!(json.contains("\"cpu\":0"));
        assert_eq!(json.matches("4294967295").count(), 52);
        assert_eq!(queue.capacity(), INTERNAL_CAPACITY);
    }
}
