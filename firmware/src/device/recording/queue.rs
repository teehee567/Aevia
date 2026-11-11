use super::format::Record;
use core::{marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

pub const STORAGE_BYTES: usize = 4 * 1024 * 1024;
const MAX_SLOTS: usize = STORAGE_BYTES / size_of::<Record>();
const READY_WORDS: usize = MAX_SLOTS.div_ceil(32);

/// Serialized admission/publication metadata, kept in internal SRAM. Record
/// storage may be PSRAM: only the exclusive grants below access its bytes, and
/// their copies must run outside the caller's interrupt-masking metadata lock.
///
/// Reservation order assigns both the ring position and source sequence. A
/// nested producer may publish first, but the consumer never skips an unfinished
/// head. Used slots include pending writes and a claimed read until release.
pub struct Queue<'a> {
    storage: NonNull<MaybeUninit<Record>>,
    capacity: usize,
    ready: [u32; READY_WORDS],
    read: usize,
    used: usize,
    writing: usize,
    reading: bool,
    pub enabled: bool,
    pub next_sequence: u64,
    pub accepted: u64,
    pub dropped: u32,
    owned: PhantomData<&'a mut [MaybeUninit<Record>]>,
}

// SAFETY: exclusive grants, tracked in this non-Clone metadata owner, are the
// only way to access storage. Metadata methods require &mut self and callers
// serialize them across cores; moving the owner does not access record bytes.
unsafe impl Send for Queue<'_> {}

struct Slot<'a> {
    storage: NonNull<MaybeUninit<Record>>,
    index: usize,
    owned: PhantomData<&'a mut Record>,
}

// SAFETY: a slot token exclusively owns its record bytes until it is consumed.
// The queue cannot reuse/reset that storage while the token is outstanding.
unsafe impl Send for Slot<'_> {}

#[must_use = "a reserved producer must write and publish before the queue can drain"]
pub struct WriteReservation<'a> {
    slot: Slot<'a>,
    sequence: u64,
}

#[must_use = "publish the completed write under the queue metadata lock"]
pub struct Written<'a>(Slot<'a>);

impl<'a> WriteReservation<'a> {
    /// Copy outside the metadata lock. Publication is a separate operation so
    /// the lock never masks interrupts during a PSRAM cache miss or write.
    pub fn write(self, mut record: Record) -> Written<'a> {
        record.sequence = self.sequence;
        // SAFETY: reservation owns this unique, unoccupied slot. No reader is
        // allowed until the resulting Written token is published.
        unsafe {
            self.slot
                .storage
                .as_ptr()
                .add(self.slot.index)
                .write(MaybeUninit::new(record))
        };
        Written(self.slot)
    }
}

#[must_use = "a claimed consumer must read and release before the queue can drain"]
pub struct ReadReservation<'a>(Slot<'a>);

#[must_use = "release the copied slot under the queue metadata lock"]
pub struct ReadComplete<'a>(Slot<'a>);

impl<'a> ReadReservation<'a> {
    /// Copy outside the metadata lock. The producer cannot reuse the slot until
    /// the returned completion token is released, including when the ring is full.
    pub fn read(self) -> (Record, ReadComplete<'a>) {
        // SAFETY: reserve_read only issues one token for the initialized head;
        // no writer can reuse it until release consumes the completion token.
        let record = unsafe {
            self.0
                .storage
                .as_ptr()
                .add(self.0.index)
                .read()
                .assume_init()
        };
        (record, ReadComplete(self.0))
    }
}

pub enum ReadState<'a> {
    Empty,
    Pending,
    Ready(ReadReservation<'a>),
}

impl<'a> Queue<'a> {
    pub fn new(slots: &'a mut [MaybeUninit<Record>]) -> Self {
        assert!(!slots.is_empty() && slots.len() <= MAX_SLOTS);
        Self {
            storage: NonNull::new(slots.as_mut_ptr()).unwrap(),
            capacity: slots.len(),
            ready: [0; READY_WORDS],
            read: 0,
            used: 0,
            writing: 0,
            reading: false,
            enabled: false,
            next_sequence: 0,
            accepted: 0,
            dropped: 0,
            owned: PhantomData,
        }
    }

    pub fn len(&self) -> usize {
        self.used
    }

    pub fn reserve_write(&mut self) -> Option<WriteReservation<'a>> {
        if !self.enabled {
            return None;
        }
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.saturating_add(1);
        if self.used == self.capacity {
            self.dropped = self.dropped.saturating_add(1);
            return None;
        }
        let index = (self.read + self.used) % self.capacity;
        // Also clears a stale bit after a disabled reset discarded old records.
        self.ready[index / 32] &= !(1 << (index % 32));
        self.used += 1;
        self.writing += 1;
        Some(WriteReservation {
            slot: Slot {
                storage: self.storage,
                index,
                owned: PhantomData,
            },
            sequence,
        })
    }

    pub fn publish(&mut self, written: Written<'a>) {
        assert_eq!(written.0.storage, self.storage, "wrong queue publication");
        assert!(self.writing != 0);
        let index = written.0.index;
        assert_eq!(self.ready[index / 32] & (1 << (index % 32)), 0);
        self.ready[index / 32] |= 1 << (index % 32);
        self.writing -= 1;
        self.accepted = self.accepted.saturating_add(1);
    }

    pub fn reserve_read(&mut self) -> ReadState<'a> {
        if self.used == 0 {
            return ReadState::Empty;
        }
        if self.reading || self.ready[self.read / 32] & (1 << (self.read % 32)) == 0 {
            return ReadState::Pending;
        }
        self.reading = true;
        ReadState::Ready(ReadReservation(Slot {
            storage: self.storage,
            index: self.read,
            owned: PhantomData,
        }))
    }

    pub fn release(&mut self, complete: ReadComplete<'a>) {
        assert_eq!(complete.0.storage, self.storage, "wrong queue release");
        assert!(self.reading && complete.0.index == self.read);
        self.ready[self.read / 32] &= !(1 << (self.read % 32));
        self.read = (self.read + 1) % self.capacity;
        self.used -= 1;
        self.reading = false;
    }

    /// Discard only a disabled, quiescent queue. An unfinished producer/read
    /// retains its slot across disable; resetting it would race that copy and
    /// could publish an old record into a new recording run.
    pub fn reset(&mut self) -> bool {
        if self.enabled || self.writing != 0 || self.reading {
            return false;
        }
        self.read = 0;
        self.used = 0;
        self.next_sequence = 0;
        self.accepted = 0;
        self.dropped = 0;
        // No PSRAM access or bitmap sweep: reservation clears each reused bit.
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(value: u8) -> Record {
        Record::new(2, 99, u64::from(value), &[value; 256]).unwrap()
    }
    fn push(q: &mut Queue<'_>, value: u8) -> bool {
        let Some(write) = q.reserve_write() else {
            return false;
        };
        q.publish(write.write(record(value)));
        true
    }
    fn pop(q: &mut Queue<'_>) -> Record {
        let ReadState::Ready(read) = q.reserve_read() else {
            panic!("record not ready")
        };
        let (record, complete) = read.read();
        q.release(complete);
        record
    }

    #[test]
    fn nested_producer_completion_preserves_reservation_order() {
        let mut slots = [MaybeUninit::uninit(); 3];
        let mut q = Queue::new(&mut slots);
        q.enabled = true;
        let first = q.reserve_write().unwrap();
        let second = q.reserve_write().unwrap();
        q.publish(second.write(record(2)));
        assert!(matches!(q.reserve_read(), ReadState::Pending));
        assert_eq!(q.len(), 2);
        q.publish(first.write(record(1)));
        let first = pop(&mut q);
        let second = pop(&mut q);
        assert_eq!((first.sequence, first.payload), (0, [1; 256]));
        assert_eq!((second.sequence, second.payload), (1, [2; 256]));
        assert!(matches!(q.reserve_read(), ReadState::Empty));
    }

    #[test]
    fn stop_keeps_pending_producer_in_drain_and_refuses_reset() {
        let mut slots = [MaybeUninit::uninit(); 2];
        let mut q = Queue::new(&mut slots);
        q.enabled = true;
        let old = q.reserve_write().unwrap();
        q.enabled = false;
        assert!(q.reserve_write().is_none());
        assert_eq!(q.next_sequence, 1);
        assert!(matches!(q.reserve_read(), ReadState::Pending));
        assert_eq!(q.len(), 1); // Stop must not mistake Pending for drained.
        assert!(!q.reset());
        let old = old.write(record(7));
        assert!(!q.reset()); // Copy complete is not yet publication complete.
        q.publish(old);
        assert_eq!(pop(&mut q).payload, [7; 256]);
        assert_eq!(q.len(), 0);
        assert!(q.reset());
        q.enabled = true;
        assert!(push(&mut q, 8));
        assert_eq!(pop(&mut q).sequence, 0);
    }

    #[test]
    fn full_ring_does_not_reuse_claimed_read_and_drop_sequence_survives_wrap() {
        let mut slots = [MaybeUninit::uninit(); 2];
        let mut q = Queue::new(&mut slots);
        q.enabled = true;
        assert!(push(&mut q, 1));
        assert!(push(&mut q, 2));
        let ReadState::Ready(read) = q.reserve_read() else {
            panic!()
        };
        assert!(matches!(q.reserve_read(), ReadState::Pending));
        assert!(!push(&mut q, 3));
        let (first, complete) = read.read();
        assert_eq!(first.payload, [1; 256]);
        assert!(!push(&mut q, 4)); // Reading has finished; release is still needed.
        q.release(complete);
        assert!(push(&mut q, 5));
        assert_eq!(pop(&mut q).sequence, 1);
        let last = pop(&mut q);
        assert_eq!((last.sequence, last.payload), (4, [5; 256]));
        assert_eq!(q.dropped, 2);
        assert_eq!(q.accepted, 3);
    }

    #[test]
    fn disabled_reset_refuses_read_token_and_clears_reused_ready_bits() {
        let mut slots = [MaybeUninit::uninit(); 2];
        let mut q = Queue::new(&mut slots);
        q.enabled = true;
        assert!(push(&mut q, 1));
        assert!(push(&mut q, 2));
        assert!(!q.reset());
        let ReadState::Ready(read) = q.reserve_read() else {
            panic!()
        };
        q.enabled = false;
        assert!(!q.reset());
        let (_, complete) = read.read();
        assert!(!q.reset());
        q.release(complete);
        assert!(q.reset()); // Discards the remaining, fully published record.
        q.enabled = true;
        let first = q.reserve_write().unwrap();
        let second = q.reserve_write().unwrap();
        q.publish(first.write(record(3)));
        assert_eq!(pop(&mut q).payload, [3; 256]);
        assert!(matches!(q.reserve_read(), ReadState::Pending));
        q.publish(second.write(record(4)));
        assert_eq!(pop(&mut q).payload, [4; 256]);
    }

    #[test]
    fn different_core_can_publish_after_consumer_observes_pending() {
        use std::sync::{Barrier, Mutex};
        let mut slots = [MaybeUninit::uninit(); 2];
        let mut q = Queue::new(&mut slots);
        q.enabled = true;
        let first = q.reserve_write().unwrap();
        let mut q = Mutex::new(q);
        let rendezvous = Barrier::new(2);
        std::thread::scope(|scope| {
            scope.spawn(|| {
                rendezvous.wait();
                let written = first.write(record(17));
                q.lock().unwrap().publish(written);
            });
            assert!(matches!(
                q.lock().unwrap().reserve_read(),
                ReadState::Pending
            ));
            rendezvous.wait();
        });
        assert_eq!(pop(q.get_mut().unwrap()).payload, [17; 256]);
    }
}
