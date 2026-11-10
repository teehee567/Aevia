//! Reset diagnostics in the SDK-owned RTC persistent section, never RTC store
//! registers reserved for ROM/bootloader use. A checksum rejects interrupted
//! panic writes. The USB-serving boot consumes the record after any immediate
//! rollback handoff, so an intermediate boot cannot discard its cause and a
//! later unrelated reset cannot inherit an old panic.
use crate::device::interface::{PanicRecord, ResetStatus};

const MAGIC: u32 = 0x504e_4331;
const WORDS: usize = 7;

fn hash_file(file: &str) -> u32 {
    file.bytes().fold(0x811c_9dc5, |hash, byte| {
        (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
    })
}

fn checksum(words: &[u32]) -> u32 {
    words
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .fold(0x811c_9dc5, |hash, byte| {
            (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
        })
}

fn decode(words: [u32; WORDS]) -> Option<PanicRecord> {
    if words[0] != MAGIC
        || words[6] != checksum(&words[..6])
        || words[4] > 1
        || words[5] > u32::from(u8::MAX)
    {
        return None;
    }
    Some(PanicRecord {
        file_hash: words[1],
        line: words[2],
        column: words[3],
        core: words[4] as u8,
        startup_stage: words[5] as u8,
    })
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::*;
    use core::{
        cell::Cell,
        ptr,
        sync::atomic::{AtomicBool, Ordering, compiler_fence},
    };
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};

    // esp-hal's S31 linker script includes rtc_fast.x and initializes this
    // section only after initial power-on. Arrays of u32 satisfy Persistable.
    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut PANIC: [u32; WORDS] = [0; WORDS];
    // Ordinary initialized SRAM: each boot may record a new first panic.
    // Never take a critical-section lock from a panic handler, which might
    // already hold that lock or be interrupted on the other core.
    static RECORDING: AtomicBool = AtomicBool::new(false);
    static STATUS: Mutex<CriticalSectionRawMutex, Cell<ResetStatus>> =
        Mutex::new(Cell::new(ResetStatus {
            reset_reason: None,
            panic: None,
        }));

    pub fn initialize() {
        let mut words = [0; WORDS];
        // SAFETY: called once on core0 before CPU1/tasks start. Only the panic
        // handler writes subsequently, immediately followed by software reset.
        unsafe {
            let base = ptr::addr_of!(PANIC).cast::<u32>();
            for (index, word) in words.iter_mut().enumerate() {
                *word = base.add(index).read_volatile();
            }
            ptr::addr_of_mut!(PANIC).cast::<u32>().write_volatile(0);
        }
        STATUS.lock(|status| {
            status.set(ResetStatus {
                reset_reason: esp_hal::system::reset_reason().map(|reason| reason as u8),
                panic: decode(words),
            })
        });
    }

    pub fn status() -> ResetStatus {
        STATUS.lock(Cell::get)
    }

    pub fn record_panic(info: &core::panic::PanicInfo<'_>) {
        if RECORDING.swap(true, Ordering::AcqRel) {
            return;
        }
        let location = info.location();
        let mut words = [
            MAGIC,
            location.map_or(0, |location| hash_file(location.file())),
            location.map_or(0, |location| location.line()),
            location.map_or(0, |location| location.column()),
            esp_hal::system::Cpu::current() as u32,
            u32::from(crate::runtime::startup_stage()),
            0,
        ];
        words[6] = checksum(&words[..6]);
        // SAFETY: exclusive diagnostic storage, no Rust references are created.
        // The atomic claim permits one writer only. A concurrent reset/second
        // panic can interrupt this write; the checksum detects that case.
        unsafe {
            let base = ptr::addr_of_mut!(PANIC).cast::<u32>();
            base.write_volatile(0);
            for (index, word) in words.iter().enumerate().skip(1) {
                base.add(index).write_volatile(*word);
            }
            compiler_fence(Ordering::SeqCst);
            base.write_volatile(MAGIC);
            core::arch::asm!("fence rw, rw", options(nostack));
        }
    }
}

#[cfg(target_arch = "riscv32")]
pub use hardware::{initialize, record_panic, status};

#[cfg(not(target_arch = "riscv32"))]
pub fn status() -> ResetStatus {
    ResetStatus::default()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn accepts_committed_panic_and_rejects_empty_or_torn_records() {
        let mut words = [
            MAGIC,
            hash_file("firmware/src/runtime.rs"),
            100,
            7,
            1,
            10,
            0,
        ];
        words[6] = checksum(&words[..6]);
        assert_eq!(decode(words).unwrap().line, 100);
        assert_eq!(decode(words).unwrap().startup_stage, 10);
        assert!(decode([0; WORDS]).is_none());
        for index in 0..WORDS {
            let mut torn = words;
            torn[index] ^= 1;
            assert!(decode(torn).is_none());
        }
    }
}
