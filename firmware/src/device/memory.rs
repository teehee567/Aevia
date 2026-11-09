//! ESP32-S31 rev 0.0 core-1 PSRAM access and startup memory check.
//!
//! Core 1 enters through ROM, which leaves PMA15 covering 0x40000000..0x60000000
//! with RX permissions. The pinned HAL maps PSRAM but does not change that
//! per-core attribute: reads work and the first store raises access fault 7.
//! Split that ROM region, keeping flash and unused addresses RX and granting
//! only the fitted 16 MiB PSRAM window RW (no execute permission).
//!
//! CSR encodings: Espressif components/riscv/include/riscv/csr.h and
//! components/esp_hw_support/port/esp32s31/cpu_region_protect.c.

pub const BASE: usize = 0x5000_0000;
pub const BYTES: usize = 16 * 1024 * 1024;

/// Consume the HAL memory handle on core 1 before any allocation is placed in it.
/// Base firmware reserves this entire range; no allocator uses it yet.
#[cfg(target_arch = "riscv32")]
pub fn bring_up(psram: esp_hal::psram::Psram) -> Result<usize, Error> {
    let (base, bytes) = psram.raw_parts();
    // SAFETY: the owned HAL handle provides the mapped range. This module is
    // its only user; no live objects or DMA buffers reside in PSRAM.
    unsafe {
        enable_core1_access(base as usize, bytes)?;
        test_memory(base as usize, bytes)?;
    }
    Ok(bytes)
}
const ENABLE: u32 = 1;
const LOCK: u32 = 1 << 29;
const ROM_RX: u32 = 0xc000_0015;
const ROM_ADDRESS: u32 = 0x13ff_ffff;
const FLASH_ADDRESS: u32 = 0x11ff_ffff;
const RAM_RW: u32 = 0xc000_0019;
const RAM_ADDRESS: u32 = 0x141f_ffff;
const TAIL_START: u32 = 0x1440_0000; // 0x51000000 >> 2
const TAIL_END: u32 = 0x1800_0000; // 0x60000000 >> 2
const TAIL_RX: u32 = 0x4000_0015;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    WrongMemoryRange,
    UnexpectedProtection,
    #[cfg(target_arch = "riscv32")]
    WrongCore,
    #[cfg(target_arch = "riscv32")]
    MemoryMismatch {
        offset: usize,
        expected: u32,
        actual: u32,
    },
}

#[derive(Clone, Copy)]
struct Protection {
    cfg: [u32; 5],  // PMA8, 9, 10, 11, 15
    addr: [u32; 4], // PMA8, 9, 11, 15
    pmp: [u32; 4],
}

impl Protection {
    fn configured(self) -> bool {
        self.cfg[0] & (ENABLE | LOCK) == 0
            && self.cfg[1] == TAIL_RX
            && self.cfg[2] & (ENABLE | LOCK) == 0
            && self.cfg[3] == RAM_RW
            && self.cfg[4] == ROM_RX
            && self.addr == [TAIL_START, TAIL_END, RAM_ADDRESS, FLASH_ADDRESS]
            && self.pmp == [0; 4]
    }

    fn validate(self, base: usize, bytes: usize) -> Result<(), Error> {
        if base != BASE || bytes != BYTES {
            return Err(Error::WrongMemoryRange);
        }
        if self.configured() {
            return Ok(());
        }
        // Do not overwrite an application/bootloader-owned or locked region.
        // PMA10 must also be disabled: changing PMA9's address would change the
        // lower bound of a TOR entry in that slot.
        if self.cfg[..4].iter().any(|cfg| cfg & (ENABLE | LOCK) != 0)
            || self.cfg[4] != ROM_RX
            || self.addr[3] != ROM_ADDRESS
            || self.pmp != [0; 4]
        {
            return Err(Error::UnexpectedProtection);
        }
        Ok(())
    }
}

#[cfg(target_arch = "riscv32")]
fn protection() -> Protection {
    macro_rules! csr {
        ($address:literal) => {{
            let value: u32;
            unsafe { core::arch::asm!("csrr {value}, {address}",
                value = out(reg) value, address = const $address, options(nostack)); }
            value
        }};
    }
    Protection {
        cfg: [
            csr!(0xbc8),
            csr!(0xbc9),
            csr!(0xbca),
            csr!(0xbcb),
            csr!(0xbcf),
        ],
        addr: [csr!(0xbd8), csr!(0xbd9), csr!(0xbdb), csr!(0xbdf)],
        pmp: [csr!(0x3a0), csr!(0x3a1), csr!(0x3a2), csr!(0x3a3)],
    }
}

/// Grant core 1 access to the HAL's exclusively reserved PSRAM.
///
/// # Safety
/// Call on core 1 before any PSRAM writes, with the mapped range returned by
/// the HAL. PMA8..11 are reserved for this module for the life of that core.
#[cfg(target_arch = "riscv32")]
pub unsafe fn enable_core1_access(base: usize, bytes: usize) -> Result<(), Error> {
    if esp_hal::system::Cpu::current() != esp_hal::system::Cpu::AppCpu {
        return Err(Error::WrongCore);
    }
    let before = protection();
    before.validate(base, bytes)?;
    if !before.configured() {
        // Shrink the ROM entry before adding disjoint regions. Flash remains
        // RX throughout, so instructions and constants remain accessible.
        unsafe {
            core::arch::asm!(
                "csrw 0xbdf, {flash}",
                "csrw 0xbd8, {tail_start}",
                "csrw 0xbd9, {tail_end}",
                "csrw 0xbc9, {tail_cfg}",
                "csrw 0xbdb, {ram}",
                "csrw 0xbcb, {ram_cfg}",
                "fence rw, rw",
                flash = in(reg) FLASH_ADDRESS,
                tail_start = in(reg) TAIL_START, tail_end = in(reg) TAIL_END,
                tail_cfg = in(reg) TAIL_RX,
                ram = in(reg) RAM_ADDRESS, ram_cfg = in(reg) RAM_RW,
                options(nostack),
            );
        }
    }
    if !protection().configured() {
        return Err(Error::UnexpectedProtection);
    }
    Ok(())
}

/// Destructively test all fitted PSRAM before handing it to any allocator.
///
/// # Safety
/// The caller owns the whole initialized, mapped range exclusively. No live
/// objects may reside in it. Run after enabling access on this core.
#[cfg(target_arch = "riscv32")]
pub unsafe fn test_memory(base: usize, bytes: usize) -> Result<(), Error> {
    if base != BASE || bytes != BYTES {
        return Err(Error::WrongMemoryRange);
    }
    let words = bytes / 4;
    let ptr = base as *mut u32;
    for invert in [0, u32::MAX] {
        for index in 0..words {
            let value = pattern(index) ^ invert;
            unsafe { ptr.add(index).write_volatile(value) };
        }
        // Force the pattern to physical PSRAM and discard cached copies before
        // checking it. The test must not pass using only cache-resident values.
        unsafe {
            flush_and_invalidate(base as u32, bytes as u32);
        }
        for index in 0..words {
            let expected = pattern(index) ^ invert;
            let actual = unsafe { ptr.add(index).read_volatile() };
            if actual != expected {
                return Err(Error::MemoryMismatch {
                    offset: index * 4,
                    expected,
                    actual,
                });
            }
        }
    }
    Ok(())
}

#[cfg(target_arch = "riscv32")]
#[esp_hal::ram]
unsafe fn flush_and_invalidate(base: u32, bytes: u32) {
    // Same ROM entry points and L1 D-cache mask used by the pinned HAL.
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(map: u32, address: u32, bytes: u32);
        fn Cache_Invalidate_Addr(map: u32, address: u32, bytes: u32);
    }
    unsafe {
        Cache_WriteBack_Addr(1 << 4, base, bytes);
        Cache_Invalidate_Addr(1 << 4, base, bytes);
    }
}

#[cfg(target_arch = "riscv32")]
fn pattern(index: usize) -> u32 {
    (index as u32).wrapping_mul(0x9e37_79b9).rotate_left(7) ^ 0xa5a5_5a5a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom() -> Protection {
        Protection {
            cfg: [0x0800_0000, 0x0800_0000, 0x0800_0000, 0x0800_0000, ROM_RX],
            addr: [0, 0, 0, ROM_ADDRESS],
            pmp: [0; 4],
        }
    }

    #[test]
    fn accepts_observed_rom_state_and_repeated_setup() {
        assert_eq!(rom().validate(BASE, BYTES), Ok(()));
        let configured = Protection {
            cfg: [0x0800_0000, TAIL_RX, 0x0800_0000, RAM_RW, ROM_RX],
            addr: [TAIL_START, TAIL_END, RAM_ADDRESS, FLASH_ADDRESS],
            pmp: [0; 4],
        };
        assert!(configured.configured());
        assert_eq!(configured.validate(BASE, BYTES), Ok(()));
    }

    #[test]
    fn refuses_different_memory_or_existing_protection() {
        assert_eq!(
            rom().validate(BASE + 4, BYTES),
            Err(Error::WrongMemoryRange)
        );
        assert_eq!(
            rom().validate(BASE, BYTES * 2),
            Err(Error::WrongMemoryRange)
        );
        for index in 0..4 {
            for bit in [ENABLE, LOCK] {
                let mut state = rom();
                state.cfg[index] |= bit;
                assert_eq!(
                    state.validate(BASE, BYTES),
                    Err(Error::UnexpectedProtection)
                );
            }
        }
        let mut state = rom();
        state.pmp[3] = 0x80;
        assert_eq!(
            state.validate(BASE, BYTES),
            Err(Error::UnexpectedProtection)
        );
        let mut state = rom();
        state.addr[3] = FLASH_ADDRESS;
        assert_eq!(
            state.validate(BASE, BYTES),
            Err(Error::UnexpectedProtection)
        );
    }

    #[test]
    fn split_grants_write_only_to_fitted_psram() {
        fn napot(addr: u32) -> (u64, u64) {
            let mask = (1_u64 << addr.trailing_ones()) - 1;
            let base = (u64::from(addr) & !mask) << 2;
            (base, base + ((mask + 1) << 3))
        }
        assert_eq!(napot(ROM_ADDRESS), (0x4000_0000, 0x6000_0000));
        assert_eq!(napot(FLASH_ADDRESS), (0x4000_0000, BASE as u64));
        assert_eq!(napot(RAM_ADDRESS), (BASE as u64, (BASE + BYTES) as u64));
        assert_eq!((TAIL_START as usize) << 2, BASE + BYTES);
        assert_eq!((TAIL_END as u64) << 2, 0x6000_0000);
        assert_eq!(ROM_RX & 8, 0);
        assert_eq!(TAIL_RX & 8, 0);
        assert_ne!(RAM_RW & 8, 0);
        assert_eq!(RAM_RW & 4, 0); // Data-only memory is not executable.
    }
}
