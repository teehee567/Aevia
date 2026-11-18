//! Recording boundary is independent of delayed storage/USB acknowledgements.
#[derive(Clone, Copy)]
pub struct Recording {
    pub active: bool,
    pub started_at_us: Option<u64>,
    pub stopped_at_us: Option<u64>,
    pub dropped: u32,
}

impl Recording {
    pub const fn new() -> Self {
        Self {
            active: false,
            started_at_us: None,
            stopped_at_us: None,
            dropped: 0,
        }
    }

    pub fn start(&mut self, now_us: u64) {
        if !self.active {
            self.active = true;
            self.started_at_us = Some(now_us);
            self.stopped_at_us = None;
            self.dropped = 0;
        }
    }

    pub fn stop(&mut self, now_us: u64) {
        if self.active {
            self.active = false;
            self.stopped_at_us = Some(now_us);
        }
    }

    /// Acquisition time determines membership even when publication is delayed
    /// until after stop. Samples from before a later start remain excluded.
    pub fn contains(&self, t_us: u64) -> bool {
        self.started_at_us.is_some_and(|start| t_us >= start)
            && self.stopped_at_us.is_none_or(|stop| t_us <= stop)
    }

    /// Every native interval contributing to an averaged IMU sample must lie
    /// inside the session. Retain nanoseconds so rounding cannot admit support
    /// extending beyond either boundary.
    pub fn contains_interval_ns(&self, start_ns: u64, end_ns: u64) -> bool {
        let Some(started_ns) = self.started_at_us.and_then(|start| start.checked_mul(1000)) else {
            return false;
        };
        if start_ns < started_ns || end_ns < start_ns {
            return false;
        }
        self.stopped_at_us.is_none_or(|stop| {
            stop.checked_mul(1000)
                .is_some_and(|stopped_ns| end_ns <= stopped_ns)
        })
    }

    pub fn note_overflow(&mut self, t_us: u64) {
        if self.contains(t_us) {
            self.dropped = self.dropped.saturating_add(1);
        }
    }
}

impl Default for Recording {
    fn default() -> Self {
        Self::new()
    }
}

/// A fixed publication watermark for a producer that cannot be interrupted by
/// the recording transition. Preview after the snapshot cannot extend its drain.
pub struct DrainCounter {
    produced: u64,
    consumed: u64,
    stop_target: Option<u64>,
}
impl DrainCounter {
    pub const fn new() -> Self {
        Self {
            produced: 0,
            consumed: 0,
            stop_target: None,
        }
    }
    pub fn next_sequence(&self) -> u64 {
        self.produced.saturating_add(1)
    }
    pub fn produced(&mut self, sequence: u64) {
        self.produced = sequence;
    }
    pub fn consumed(&mut self, sequence: u64) {
        self.consumed = self.consumed.max(sequence);
    }
    pub fn request_stop(&mut self) {
        if self.stop_target.is_none() {
            self.stop_target = Some(self.produced);
        }
    }
    pub fn pending(&self) -> bool {
        self.stop_target.is_some()
    }
    pub fn finish_if_drained(&mut self) -> bool {
        if self
            .stop_target
            .is_some_and(|target| self.consumed >= target)
        {
            self.stop_target = None;
            true
        } else {
            false
        }
    }
}
impl Default for DrainCounter {
    fn default() -> Self {
        Self::new()
    }
}

/// Raw GNSS frames are admitted by their first received byte. A stop waits for
/// an already-started frame, including its undecided header, to finish/reset.
pub struct FrameFence {
    boundary: Option<u64>,
}
impl FrameFence {
    pub const fn new() -> Self {
        Self { boundary: None }
    }
    pub fn request(&mut self, boundary: u64, in_flight_received_us: Option<u64>) {
        // The producer publishes this snapshot before every await/reset. With
        // no admitted frame, stopping must not await a configuration operation
        // that could take longer than the storage flush deadline.
        if self.boundary.is_none() && in_flight_received_us.is_some_and(|first| first <= boundary) {
            self.boundary = Some(boundary);
        }
    }
    pub fn pending(&self) -> bool {
        self.boundary.is_some()
    }
    pub fn observe(&mut self, in_flight_received_us: Option<u64>) {
        if self
            .boundary
            .is_some_and(|end| in_flight_received_us.is_none_or(|start| start > end))
        {
            self.boundary = None;
        }
    }
    /// A queued pre-stop header may not have reached the decoder yet. An empty
    /// parser proves completion only once that fixed input watermark is drained.
    pub fn observe_after_drain(&mut self, input_pending: bool, in_flight_received_us: Option<u64>) {
        if !input_pending {
            self.observe(in_flight_received_us);
        }
    }
}
impl Default for FrameFence {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[test]
fn raw_frame_stop_waits_existing_frame_but_never_new_preview() {
    let mut fence = FrameFence::new();
    fence.request(100, Some(99));
    fence.observe(Some(99));
    assert!(fence.pending());
    fence.request(200, Some(150)); // Idempotent stop cannot move its boundary.
    fence.observe(Some(100));
    assert!(fence.pending());
    fence.observe(Some(101));
    assert!(!fence.pending());
    fence.request(200, Some(199));
    fence.observe(None); // Completion or explicitly diagnosed truncation.
    assert!(!fence.pending());
    fence.request(300, None); // Receiver is configuring; no callback is needed.
    assert!(!fence.pending());
    fence.request(300, Some(301)); // A new preview frame cannot hold closure.
    assert!(!fence.pending());
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StopStage {
    Idle,
    Waiting(u64),
    Queued(u64),
}

/// Orders storage stop after previously acquired intervals in the raw queue.
/// A full queue leaves the request Waiting so acquisition can retry its marker.
#[derive(Clone, Copy, Debug)]
pub struct StopFence {
    stage: StopStage,
}

impl StopFence {
    pub const fn new() -> Self {
        Self {
            stage: StopStage::Idle,
        }
    }

    /// Call once at the recording's active-to-stopped transition. Repeated
    /// requests while pending preserve the first boundary and its queue state.
    pub fn request(&mut self, boundary: u64) {
        if self.stage == StopStage::Idle {
            self.stage = StopStage::Waiting(boundary);
        }
    }

    /// The producer may enqueue this marker once it has observed the cutoff.
    pub fn boundary_after(&self, observed_us: u64) -> Option<u64> {
        match self.stage {
            StopStage::Waiting(boundary) if observed_us >= boundary => Some(boundary),
            _ => None,
        }
    }

    /// A strictly newer interval is preview data. Queue the eligible stop fence
    /// before it so preview cannot repeatedly take the last free queue slot.
    pub fn boundary_before_ns(&self, observed_ns: u64) -> Option<u64> {
        let boundary = self.boundary_after(observed_ns / 1000)?;
        (observed_ns > boundary.checked_mul(1000)?).then_some(boundary)
    }

    /// Call only after successfully enqueuing the matching stop marker.
    pub fn queued(&mut self, boundary: u64) -> bool {
        if self.stage == StopStage::Waiting(boundary) {
            self.stage = StopStage::Queued(boundary);
            true
        } else {
            false
        }
    }

    /// Only the consumer of the matching queued marker may release the fence.
    pub fn complete(&mut self, boundary: u64) -> bool {
        if self.stage == StopStage::Queued(boundary) {
            self.stage = StopStage::Idle;
            true
        } else {
            false
        }
    }

    pub fn pending(&self) -> bool {
        self.stage != StopStage::Idle
    }
}

impl Default for StopFence {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queued_header_and_post_stop_frame_tail_keep_storage_open_until_decoded() {
        use crate::uart_pipe::{Port, Ring};
        let mut raw = std::vec![0_u8; 28];
        raw[..3].copy_from_slice(&[0xaa, 0x44, 0xb5]);
        raw[4..6].copy_from_slice(&12_u16.to_le_bytes());
        raw[6..8].copy_from_slice(&4_u16.to_le_bytes());
        raw.extend_from_slice(&crate::gnss::unicore_crc32(&raw).to_le_bytes());
        let mut pipe = Ring::new();
        let mut drain = DrainCounter::new();
        let mut fence = FrameFence::new();
        let mut decoder = crate::ppk::Decoder::new();
        drain.produced(
            pipe.try_push(Port::Command, 99, Some(99_000), &raw[..2])
                .unwrap(),
        );
        drain.request_stop();
        fence.request(100, Some(0));
        fence.observe_after_drain(drain.pending(), decoder.inflight_received_us());
        assert!(fence.pending()); // Header is still queued, parser is empty.
        let head = pipe.pop().unwrap();
        for byte in head.as_bytes() {
            assert!(decoder.push(*byte, head.received_us).is_none());
        }
        drain.consumed(head.sequence);
        assert!(drain.finish_if_drained());
        fence.observe_after_drain(drain.pending(), decoder.inflight_received_us());
        assert!(fence.pending()); // Undecided old header must finish after stop.
        drain.produced(
            pipe.try_push(Port::Command, 101, Some(101_000), &raw[2..])
                .unwrap(),
        );
        let tail = pipe.pop().unwrap();
        let mut complete = None;
        for byte in tail.as_bytes() {
            if let Some(chunk) = decoder.push(*byte, tail.received_us) {
                complete = Some(chunk);
            }
        }
        let chunk = complete.unwrap();
        assert!(chunk.last());
        assert_eq!(chunk.received_us(), 99);
        assert_eq!(decoder.diagnostics().observation_frames, 1);
        fence.observe_after_drain(drain.pending(), decoder.inflight_received_us());
        assert!(!fence.pending());
        // Later preview can be queued but cannot extend the fixed stop target.
        assert!(!drain.pending());
    }

    #[test]
    fn observation_drain_waits_for_old_tail_and_ignores_later_preview() {
        let mut drain = DrainCounter::new();
        assert!(!drain.finish_if_drained());
        drain.produced(drain.next_sequence());
        drain.produced(drain.next_sequence());
        drain.request_stop();
        drain.produced(drain.next_sequence());
        drain.request_stop(); // Preserve the original fixed watermark.
        drain.consumed(1);
        assert!(drain.pending());
        assert!(!drain.finish_if_drained());
        drain.consumed(2);
        assert!(drain.finish_if_drained());
        assert!(!drain.pending());
        assert!(!drain.finish_if_drained());
    }

    #[test]
    fn independent_observation_and_raw_watermarks_both_finish_before_closure() {
        let mut observations = DrainCounter::new();
        let mut raw = DrainCounter::new();
        observations.produced(7);
        raw.produced(51);
        observations.request_stop();
        raw.request_stop();
        observations.produced(8); // Post-stop preview cannot extend either target.
        raw.produced(52);
        raw.consumed(51);
        assert!(raw.finish_if_drained());
        observations.consumed(6);
        assert!(!observations.finish_if_drained());
        assert!(observations.pending());
        observations.request_stop(); // Idempotent Stop retains the earlier target.
        observations.consumed(7);
        assert!(observations.finish_if_drained());
        assert!(!observations.pending() && !raw.pending());
    }

    #[test]
    fn stop_marker_precedes_newer_preview_but_not_exact_cutoff_interval() {
        let mut fence = StopFence::new();
        fence.request(100);
        assert_eq!(fence.boundary_before_ns(99_999), None);
        assert_eq!(fence.boundary_before_ns(100_000), None);
        assert_eq!(fence.boundary_before_ns(100_001), Some(100));
        // A failed enqueue leaves the marker eligible for the next free slot.
        assert_eq!(fence.boundary_before_ns(101_000), Some(100));
        assert!(fence.queued(100));
        assert_eq!(fence.boundary_before_ns(102_000), None);
    }

    #[test]
    fn stop_boundary_survives_flush_delay_and_repeated_stop() {
        let mut state = Recording::new();
        state.stop(50);
        assert_eq!(state.stopped_at_us, None);
        state.start(100);
        state.start(900);
        assert_eq!(state.started_at_us, Some(100));
        state.stop(1_000);
        // The response is assembled again after the SD flush. Its wall time
        // must not extend the recorded interval or change the original stop.
        state.stop(10_001_000);
        assert_eq!(state.stopped_at_us, Some(1_000));
        assert!(!state.active);
        assert_eq!(state.started_at_us, Some(100));
        state.start(20_000_000);
        assert!(state.active);
        assert_eq!(state.stopped_at_us, None);
        assert_eq!(state.started_at_us, Some(20_000_000));
    }

    #[test]
    fn preview_and_flush_time_overflows_are_not_recording_losses() {
        let mut state = Recording::new();
        state.note_overflow(40);
        assert_eq!(state.dropped, 0);
        state.start(50);
        state.note_overflow(49); // Delayed pre-start publication is still preview.
        assert_eq!(state.dropped, 0);
        state.note_overflow(50);
        state.start(60); // Idempotent start must retain recording losses.
        assert_eq!(state.dropped, 1);
        state.stop(100);
        for _ in 0..500 {
            state.note_overflow(101);
        }
        assert_eq!(state.dropped, 1);
        state.note_overflow(99); // Acquired in-window, published after stop.
        state.note_overflow(100); // The stop boundary is inclusive.
        assert_eq!(state.dropped, 3);
        state.start(200);
        state.note_overflow(100); // A previous session cannot enter the new one.
        assert_eq!(state.dropped, 0);
    }

    #[test]
    fn membership_uses_capture_time_and_includes_both_session_boundaries() {
        let mut state = Recording::new();
        assert!(!state.contains(100));
        state.start(100);
        assert!(!state.contains(99));
        assert!(state.contains(100));
        assert!(state.contains(200));
        state.stop(200);
        assert!(!state.active);
        assert!(state.contains(100));
        assert!(state.contains(150));
        assert!(state.contains(200));
        assert!(!state.contains(201));
        state.start(300);
        assert!(!state.contains(200));
        assert!(state.contains(300));
    }

    #[test]
    fn imu_membership_requires_complete_support_at_nanosecond_precision() {
        let mut state = Recording::new();
        assert!(!state.contains_interval_ns(100_000, 200_000));
        state.start(100);
        assert!(state.contains_interval_ns(100_000, 200_000));
        // An averaged sample ending after start still includes pre-start data.
        assert!(!state.contains_interval_ns(99_999, 150_000));
        assert!(!state.contains_interval_ns(150_000, 149_999));
        state.stop(200);
        // Delayed publication remains valid when all support is in the window.
        assert!(state.contains_interval_ns(100_000, 200_000));
        assert!(state.contains_interval_ns(125_000, 175_000));
        // Rounding this endpoint down to microseconds must not admit it.
        assert!(!state.contains_interval_ns(150_000, 200_001));
        assert!(!state.contains_interval_ns(99_999, 200_000));
        state.start(300);
        assert!(!state.contains_interval_ns(100_000, 200_000));
        assert!(!state.contains_interval_ns(299_999, 350_000));
        assert!(state.contains_interval_ns(300_000, 350_000));
    }

    #[test]
    fn stop_fence_waits_for_cutoff_and_retries_when_the_queue_is_full() {
        let mut fence = StopFence::new();
        assert!(!fence.pending());
        assert_eq!(fence.boundary_after(500), None);
        fence.request(100);
        assert!(fence.pending());
        assert_eq!(fence.boundary_after(99), None);
        assert_eq!(fence.boundary_after(100), Some(100));
        // Enqueue failed: do not call queued. A later service point retries.
        assert_eq!(fence.boundary_after(150), Some(100));
        assert!(!fence.complete(100));
        assert!(!fence.queued(99));
        assert!(fence.queued(100));
        assert_eq!(fence.boundary_after(500), None);
        assert!(fence.pending());
        assert!(!fence.complete(99));
        assert!(fence.complete(100));
        assert!(!fence.pending());
        assert!(!fence.complete(100));
    }

    #[test]
    fn repeated_requests_preserve_boundary_and_stale_completion_cannot_release() {
        let mut fence = StopFence::new();
        fence.request(100);
        fence.request(100);
        fence.request(200);
        assert_eq!(fence.boundary_after(200), Some(100));
        assert!(fence.queued(100));
        assert!(!fence.queued(100));
        fence.request(200);
        assert_eq!(fence.boundary_after(200), None);
        assert!(!fence.complete(200));
        assert!(fence.complete(100));

        fence.request(300);
        assert!(!fence.complete(100));
        assert_eq!(fence.boundary_after(300), Some(300));
        assert!(fence.queued(300));
        assert!(!fence.complete(100));
        assert!(fence.pending());
        assert!(fence.complete(300));
        assert!(!fence.pending());
    }
}
