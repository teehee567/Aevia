//! Shared sensor snapshots and a bounded, explicitly enabled USB capture queue.

use aevia_firmware::imu::ImuSample;
use core::cell::RefCell;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};

pub struct Shared<T>(Mutex<CriticalSectionRawMutex, RefCell<T>>);
impl<T> Shared<T> {
    pub const fn new(value: T) -> Self {
        Self(Mutex::new(RefCell::new(value)))
    }
    pub fn update<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        self.0.lock(|cell| f(&mut cell.borrow_mut()))
    }
}
impl<T: Clone> Shared<T> {
    pub fn get(&self) -> T {
        self.0.lock(|cell| cell.borrow().clone())
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Phase {
    Starting,
    Configuring,
    Ready,
    Bridge,
    Fault,
}
impl Phase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Starting => "starting",
            Self::Configuring => "configuring",
            Self::Ready => "ready",
            Self::Bridge => "bridge",
            Self::Fault => "fault",
        }
    }
}

#[derive(Clone, Copy)]
pub struct SensorStatus {
    pub phase: Phase,
    pub records: u32,
    pub valid: u32,
    pub errors: u32,
    pub last_us: Option<u64>,
}
impl SensorStatus {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Starting,
            records: 0,
            valid: 0,
            errors: 0,
            last_us: None,
        }
    }
    pub fn label(self, now_us: u64) -> &'static str {
        if matches!(self.phase, Phase::Ready)
            && self
                .last_us
                .is_none_or(|last| now_us.saturating_sub(last) >= 3_000_000)
        {
            "stale"
        } else {
            self.phase.label()
        }
    }
}

pub static IMU: Shared<SensorStatus> = Shared::new(SensorStatus::new());
pub static GNSS: Shared<SensorStatus> = Shared::new(SensorStatus::new());
pub static IMU_ERROR: Shared<Option<aevia_firmware::imu::ImuError>> = Shared::new(None);
pub static GNSS_ERROR: Shared<Option<&'static str>> = Shared::new(None);
pub static FIX: Shared<heapless::String<32>> = Shared::new(heapless::String::new());
pub static MEMORY: Shared<Result<usize, Option<aevia_firmware::memory::Error>>> =
    Shared::new(Err(None));

// Inline storage intentionally avoids an allocator and preserves arrival order
// across sensors. The 32-entry queue has a fixed ~33 KiB SRAM budget.
#[allow(clippy::large_enum_variant)]
pub enum Measurement {
    Imu(ImuSample),
    Gnss {
        received_us: u64,
        line: heapless::String<1024>,
    },
}

struct Capture {
    enabled: bool,
    drops: u32,
}
static CAPTURE: Shared<Capture> = Shared::new(Capture {
    enabled: false,
    drops: 0,
});
pub static MEASUREMENTS: Channel<CriticalSectionRawMutex, Measurement, 32> = Channel::new();

/// Producers never await USB. Gate and queue operations share the same critical
/// section so stopping cannot race an enqueue from core 1.
pub fn enqueue(measurement: Measurement) {
    CAPTURE.update(|capture| {
        if capture.enabled && MEASUREMENTS.try_send(measurement).is_err() {
            capture.drops = capture.drops.wrapping_add(1);
        }
    });
}

/// Called only by the USB writer, which orders session markers and RAW records.
pub fn capture(enabled: bool) {
    CAPTURE.update(|capture| {
        capture.enabled = enabled;
        MEASUREMENTS.clear();
    });
}
pub fn drops() -> u32 {
    CAPTURE.update(|capture| capture.drops)
}
