//! Integer-only packed UART reads between interrupt IO and ordinary parsing.
//!
//! The owner supplies synchronization. Each successful publication retains its
//! original read timestamp, port and stream generation. Storage is packed so
//! small reads consume only their bytes plus a 31-byte header, not a 128-byte
//! slot. A failed publication leaves all queued reads intact and marks a gap in
//! that port's stream. Consumers must also observe `generations()` when empty:
//! a loss or explicit reset may occur without another successful publication.

pub const CAPACITY_BYTES: usize = 8192;
pub const MAX_READ_BYTES: usize = 128;
const HEADER_BYTES: usize = 31;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Port {
    Data,
    Command,
}

impl Port {
    pub const fn index(self) -> usize {
        match self {
            Self::Data => 0,
            Self::Command => 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadBatch {
    pub sequence: u64,
    pub received_us: u64,
    /// Optional hardware capture clock at receipt, never recomputed by parsing.
    pub captured_ns: Option<u64>,
    pub generation: u32,
    pub port: Port,
    pub bytes: [u8; MAX_READ_BYTES],
    pub len: usize,
}

impl ReadBatch {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct PortStatus {
    pub generation: u32,
    pub dropped_reads: u64,
    pub dropped_bytes: u64,
}

impl PortStatus {
    const fn new() -> Self {
        Self {
            generation: 0,
            dropped_reads: 0,
            dropped_bytes: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct Status {
    pub capacity_bytes: usize,
    /// Includes the packed metadata headers as well as received bytes.
    pub queued_bytes: usize,
    pub queued_reads: usize,
    /// Zero before the first accepted read; rejected reads never advance it.
    pub published_sequence: u64,
    pub data: PortStatus,
    pub command: PortStatus,
}

pub struct Ring {
    bytes: [u8; CAPACITY_BYTES],
    head: usize,
    queued_bytes: usize,
    queued_reads: usize,
    published_sequence: u64,
    ports: [PortStatus; 2],
}

impl Default for Ring {
    fn default() -> Self {
        Self::new()
    }
}

impl Ring {
    /// Intended for a static synchronized owner; no heap allocation is needed.
    pub const fn new() -> Self {
        Self {
            bytes: [0; CAPACITY_BYTES],
            head: 0,
            queued_bytes: 0,
            queued_reads: 0,
            published_sequence: 0,
            ports: [PortStatus::new(); 2],
        }
    }

    /// Publish one nonempty read of at most 128 bytes atomically under the
    /// caller's lock. Invalid sizes, exhausted storage and sequence exhaustion
    /// are explicit losses; none replaces an older read or advances a watermark.
    #[allow(clippy::result_unit_err)]
    pub fn try_push(
        &mut self,
        port: Port,
        received_us: u64,
        captured_ns: Option<u64>,
        bytes: &[u8],
    ) -> Result<u64, ()> {
        if bytes.is_empty()
            || bytes.len() > MAX_READ_BYTES
            || HEADER_BYTES + bytes.len() > CAPACITY_BYTES - self.queued_bytes
        {
            self.record_loss(port, bytes.len());
            return Err(());
        }
        let Some(sequence) = self.published_sequence.checked_add(1) else {
            self.record_loss(port, bytes.len());
            return Err(());
        };
        let mut header = [0; HEADER_BYTES];
        header[..8].copy_from_slice(&sequence.to_le_bytes());
        header[8..16].copy_from_slice(&received_us.to_le_bytes());
        header[16..20].copy_from_slice(&self.ports[port.index()].generation.to_le_bytes());
        header[20] = port.index() as u8;
        header[21] = bytes.len() as u8;
        header[22] = u8::from(captured_ns.is_some());
        header[23..31].copy_from_slice(&captured_ns.unwrap_or(0).to_le_bytes());
        self.append(&header);
        self.append(bytes);
        self.queued_reads += 1;
        self.published_sequence = sequence;
        Ok(sequence)
    }

    pub fn pop(&mut self) -> Option<ReadBatch> {
        if self.queued_reads == 0 {
            return None;
        }
        let mut header = [0; HEADER_BYTES];
        self.remove(&mut header);
        let mut batch = ReadBatch {
            sequence: u64::from_le_bytes(header[..8].try_into().unwrap()),
            received_us: u64::from_le_bytes(header[8..16].try_into().unwrap()),
            captured_ns: (header[22] != 0)
                .then(|| u64::from_le_bytes(header[23..31].try_into().unwrap())),
            generation: u32::from_le_bytes(header[16..20].try_into().unwrap()),
            port: match header[20] {
                0 => Port::Data,
                1 => Port::Command,
                _ => unreachable!("packed UART port was not written by this ring"),
            },
            bytes: [0; MAX_READ_BYTES],
            len: usize::from(header[21]),
        };
        self.remove(&mut batch.bytes[..batch.len]);
        self.queued_reads -= 1;
        Some(batch)
    }

    /// Advance a stream generation without discarding earlier queued reads or
    /// counting an unknown number of lost bytes. The owner records UART errors
    /// separately and wakes the consumer even when no read can be published.
    pub fn reset_port(&mut self, port: Port) {
        let status = &mut self.ports[port.index()];
        status.generation = status.generation.wrapping_add(1);
    }

    /// Index 0 is Data and index 1 is Command, as with `Port::index()`.
    pub fn generations(&self) -> [u32; 2] {
        [self.ports[0].generation, self.ports[1].generation]
    }

    pub fn status(&self) -> Status {
        Status {
            capacity_bytes: CAPACITY_BYTES,
            queued_bytes: self.queued_bytes,
            queued_reads: self.queued_reads,
            published_sequence: self.published_sequence,
            data: self.ports[0],
            command: self.ports[1],
        }
    }

    fn record_loss(&mut self, port: Port, bytes: usize) {
        self.reset_port(port);
        let status = &mut self.ports[port.index()];
        status.dropped_reads = status.dropped_reads.saturating_add(1);
        status.dropped_bytes = status.dropped_bytes.saturating_add(bytes as u64);
    }

    fn append(&mut self, bytes: &[u8]) {
        let tail = (self.head + self.queued_bytes) % CAPACITY_BYTES;
        let first = bytes.len().min(CAPACITY_BYTES - tail);
        self.bytes[tail..tail + first].copy_from_slice(&bytes[..first]);
        self.bytes[..bytes.len() - first].copy_from_slice(&bytes[first..]);
        self.queued_bytes += bytes.len();
    }

    fn remove(&mut self, bytes: &mut [u8]) {
        let first = bytes.len().min(CAPACITY_BYTES - self.head);
        let remaining = bytes.len() - first;
        bytes[..first].copy_from_slice(&self.bytes[self.head..self.head + first]);
        bytes[first..].copy_from_slice(&self.bytes[..remaining]);
        self.head = (self.head + bytes.len()) % CAPACITY_BYTES;
        self.queued_bytes -= bytes.len();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_tiny_reads_ports_and_exact_timestamps() {
        let mut queue = Ring::new();
        let base = (1_u64 << 53) + 17;
        for i in 0..200 {
            let port = if i % 2 == 0 {
                Port::Data
            } else {
                Port::Command
            };
            assert_eq!(
                queue.try_push(port, base + i, Some(u64::MAX - i), &[i as u8]),
                Ok(i + 1)
            );
        }
        assert_eq!(queue.status().queued_reads, 200);
        assert_eq!(queue.status().queued_bytes, 200 * (HEADER_BYTES + 1));
        for i in 0..200 {
            let batch = queue.pop().unwrap();
            assert_eq!(batch.sequence, i + 1);
            assert_eq!(batch.received_us, base + i);
            assert_eq!(batch.captured_ns, Some(u64::MAX - i));
            assert_eq!(batch.port.index(), i as usize % 2);
            assert_eq!(batch.generation, 0);
            assert_eq!(batch.as_bytes(), &[i as u8]);
        }
        assert!(queue.pop().is_none());
        assert_eq!(queue.status().queued_bytes, 0);
        assert_eq!(queue.status().published_sequence, 200);
        assert!(core::mem::size_of::<Ring>() <= CAPACITY_BYTES + 128);
    }

    #[test]
    fn fills_exactly_without_overwriting_and_marks_only_lost_port() {
        let mut queue = Ring::new();
        for i in 0..51 {
            assert_eq!(
                queue.try_push(Port::Data, i, None, &[i as u8; 128]),
                Ok(i + 1)
            );
        }
        // 51*159 + 31+52 = 8192; the final read fits exactly.
        assert_eq!(queue.try_push(Port::Command, 51, None, &[0x55; 52]), Ok(52));
        assert_eq!(queue.status().queued_bytes, CAPACITY_BYTES);
        assert_eq!(queue.try_push(Port::Command, 55, None, &[0xff; 3]), Err(()));
        let status = queue.status();
        assert_eq!(status.published_sequence, 52);
        assert_eq!(status.data.dropped_reads, 0);
        assert_eq!(status.command.dropped_reads, 1);
        assert_eq!(status.command.dropped_bytes, 3);
        assert_eq!(queue.generations(), [0, 1]);
        for i in 0..51 {
            let batch = queue.pop().unwrap();
            assert_eq!(batch.sequence, i + 1);
            assert_eq!(batch.as_bytes(), &[i as u8; 128]);
        }
        let old_command = queue.pop().unwrap();
        assert_eq!(old_command.generation, 0);
        assert_eq!(old_command.as_bytes(), &[0x55; 52]);
        // Even with no further data the generation reveals the missing read.
        assert!(queue.pop().is_none());
        assert_eq!(queue.generations(), [0, 1]);
        assert_eq!(queue.try_push(Port::Command, 60, None, &[1, 2]), Ok(53));
        assert_eq!(queue.pop().unwrap().generation, 1);
    }

    #[test]
    fn explicit_reset_preserves_queued_epochs_and_success_watermark() {
        let mut queue = Ring::new();
        assert_eq!(queue.try_push(Port::Data, 10, None, &[1]), Ok(1));
        queue.reset_port(Port::Data);
        queue.reset_port(Port::Command);
        assert_eq!(queue.try_push(Port::Data, 11, None, &[2]), Ok(2));
        assert_eq!(queue.pop().unwrap().generation, 0);
        assert_eq!(queue.pop().unwrap().generation, 1);
        assert_eq!(queue.generations(), [1, 1]);
        assert_eq!(queue.status().published_sequence, 2);
        assert_eq!(queue.status().data.dropped_reads, 0);
        assert_eq!(queue.status().command.dropped_bytes, 0);
    }

    #[test]
    fn malformed_reads_and_exhausted_sequence_never_publish_a_record() {
        let mut queue = Ring::new();
        assert!(queue.try_push(Port::Data, 1, None, &[]).is_err());
        assert!(queue.try_push(Port::Data, 2, None, &[0; 129]).is_err());
        assert_eq!(queue.status().queued_bytes, 0);
        assert_eq!(queue.status().published_sequence, 0);
        assert_eq!(queue.status().data.dropped_reads, 2);
        assert_eq!(queue.status().data.dropped_bytes, 129);
        queue.published_sequence = u64::MAX - 1;
        assert_eq!(queue.try_push(Port::Command, 3, None, &[3]), Ok(u64::MAX));
        assert!(queue.try_push(Port::Command, 4, None, &[4]).is_err());
        assert_eq!(queue.pop().unwrap().sequence, u64::MAX);
        assert!(queue.pop().is_none());
        assert_eq!(queue.status().command.dropped_reads, 1);
        assert_eq!(queue.status().published_sequence, u64::MAX);
    }

    #[test]
    fn packed_wraps_and_loss_accounting_match_reference_queue() {
        use std::collections::VecDeque;
        let mut queue = Ring::new();
        let mut expected = VecDeque::new();
        let mut used = 0;
        let mut sequence = 0;
        let mut ports = [PortStatus::new(); 2];
        let mut random = 0x6a09_e667_u32;
        for iteration in 0..30_000_u64 {
            random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let port = if random & 1 == 0 {
                Port::Data
            } else {
                Port::Command
            };
            match (random >> 16) % 10 {
                0 => {
                    queue.reset_port(port);
                    ports[port.index()].generation = ports[port.index()].generation.wrapping_add(1);
                }
                1..=4 => {
                    let actual = queue.pop();
                    let reference: Option<ReadBatch> = expected.pop_front();
                    if let Some(batch) = reference {
                        used -= HEADER_BYTES + batch.len;
                    }
                    assert_eq!(actual, reference);
                }
                _ => {
                    let len = ((random >> 8) as usize % MAX_READ_BYTES) + 1;
                    let bytes = [iteration as u8; MAX_READ_BYTES];
                    let timestamp = u64::MAX - iteration;
                    let captured_ns = (random & 4 != 0).then_some(timestamp);
                    let result = queue.try_push(port, timestamp, captured_ns, &bytes[..len]);
                    let status = &mut ports[port.index()];
                    if used + HEADER_BYTES + len <= CAPACITY_BYTES {
                        sequence += 1;
                        assert_eq!(result, Ok(sequence));
                        let mut copied = [0; MAX_READ_BYTES];
                        copied[..len].copy_from_slice(&bytes[..len]);
                        expected.push_back(ReadBatch {
                            sequence,
                            received_us: timestamp,
                            captured_ns,
                            generation: status.generation,
                            port,
                            bytes: copied,
                            len,
                        });
                        used += HEADER_BYTES + len;
                    } else {
                        assert_eq!(result, Err(()));
                        status.generation = status.generation.wrapping_add(1);
                        status.dropped_reads += 1;
                        status.dropped_bytes += len as u64;
                    }
                }
            }
            let status = queue.status();
            assert_eq!(status.queued_bytes, used);
            assert_eq!(status.queued_reads, expected.len());
            assert_eq!(status.published_sequence, sequence);
            assert_eq!(status.data, ports[0]);
            assert_eq!(status.command, ports[1]);
        }
        while let Some(batch) = expected.pop_front() {
            assert_eq!(queue.pop(), Some(batch));
        }
        assert!(queue.pop().is_none());
        assert_eq!(queue.status().queued_bytes, 0);
    }
}
