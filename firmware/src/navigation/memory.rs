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
/// The final MiB belongs exclusively to CPU0's USB preview queue after bring-up.
/// CPU1's estimator is never handed this portion, including during resets.
pub const PREVIEW_BYTES: usize = 1024 * 1024;
pub const NAVIGATION_BYTES: usize = BYTES - PREVIEW_BYTES;
pub const PREVIEW_BASE: usize = BASE + NAVIGATION_BYTES;

#[cfg(target_arch = "riscv32")]
static PSRAM_STATE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

#[cfg(target_arch = "riscv32")]
pub fn preview_ready() -> bool {
    PSRAM_STATE.load(core::sync::atomic::Ordering::Acquire) == 1
}

#[cfg(target_arch = "riscv32")]
pub fn preview_test_failed() -> bool {
    PSRAM_STATE.load(core::sync::atomic::Ordering::Acquire) == 2
}

/// Consume the HAL memory handle on core 1 before any allocation is placed in it.
/// No allocator may use this range until its full startup test completes.
#[cfg(target_arch = "riscv32")]
pub fn bring_up(psram: esp_hal::psram::Psram) -> Result<usize, Error> {
    use core::sync::atomic::Ordering;
    // Never destructively retest after CPU0 could have claimed its reservation.
    PSRAM_STATE
        .compare_exchange(0, 3, Ordering::AcqRel, Ordering::Acquire)
        .map_err(|_| Error::AlreadyInitialized)?;
    let (base, bytes) = psram.raw_parts();
    // SAFETY: the owned HAL handle provides the mapped range. This module is
    // its only user; no live objects or DMA buffers reside in PSRAM.
    let result = unsafe {
        enable_core1_access(base as usize, bytes).and_then(|()| test_memory(base as usize, bytes))
    };
    if let Err(error) = result {
        PSRAM_STATE.store(2, Ordering::Release);
        return Err(error);
    }
    // Finish CPU1's test-cache ownership before publishing the disjoint tail.
    // CPU1 never touches this tail again; no cross-core live queue is created.
    unsafe { flush_and_invalidate(PREVIEW_BASE as u32, PREVIEW_BYTES as u32) };
    PSRAM_STATE.store(1, Ordering::Release);
    Ok(NAVIGATION_BYTES)
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
    AlreadyInitialized,
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

/// Accept the complete CPU0 bootloader PMA map observed on the fitted board.
/// Entry 9 is cacheable RWX for 0x50000000..0x54000000. HAL subsequently installs
/// locked PMP guards for internal .rwtext, .stack and data; they must survive.
/// PSRAM is PMP-unmatched and therefore accessible in machine mode. Do not
/// rewrite this map merely to remove execute permission from data storage.
fn core0_existing_access(state: &crate::preview_queue::ProtectionSnapshot) -> bool {
    const CFG: [u32; 16] = [
        0xc000_0001,
        0x0000_0001,
        0x4000_0001,
        0x0000_0001,
        0x4000_0001,
        0x4000_0015,
        0x4000_0001,
        0x4000_0015,
        0x4000_0001,
        0x4000_001d,
        0x4000_0001,
        0x4000_0015,
        0x4000_0001,
        0x4000_0019,
        0x4000_0001,
        0,
    ];
    const ADDR: [u32; 16] = [
        0x01ff_ffff,
        0x0b80_2000,
        0x0bc0_0000,
        0x0bc2_0000,
        0x0be0_0000,
        0x0be1_4000,
        0x1000_0000,
        0x1100_0000,
        0x1400_0000,
        0x1500_0000,
        0x2800_0000,
        0x2900_0000,
        0x3000_0000,
        0x3100_0000,
        0x3fff_ffe0,
        0,
    ];
    state.cpu == 0
        && state.pma_cfg == CFG
        && state.pma_addr == ADDR
        && pmp_leaves_psram_unmatched(&state.pmp_cfg, &state.pmp_addr)
}

/// Decode all standard PMP address modes conservatively. Even a permissive or
/// unlocked entry overlapping PSRAM is declined: this path only accepts a
/// demonstrably unmatched region and never changes somebody else's policy.
fn pmp_leaves_psram_unmatched(cfg: &[u32; 4], addr: &[u32; 16]) -> bool {
    for index in 0..16 {
        let entry = (cfg[index / 4] >> ((index % 4) * 8)) & 0xff;
        if entry & 0x60 != 0 || entry & 3 == 2 {
            return false; // Reserved bits or reserved W=1/R=0 encoding.
        }
        let address = u64::from(addr[index]);
        let (low, high) = match (entry >> 3) & 3 {
            0 => continue, // Locked OFF entries also serve as TOR anchors.
            1 => (
                if index == 0 {
                    0
                } else {
                    u64::from(addr[index - 1]) << 2
                },
                address << 2,
            ),
            2 => (address << 2, (address << 2) + 4),
            _ => {
                let mask = (1_u64 << addr[index].trailing_ones()) - 1;
                let base = (address & !mask) << 2;
                (base, base + ((mask + 1) << 3))
            }
        };
        if high <= low || (low < (BASE + BYTES) as u64 && high > BASE as u64) {
            return false;
        }
    }
    true
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

/// Read every implemented PMA/PMP entry without changing protection or touching
/// PSRAM. The queue calls this on CPU0 only after its access guard declines.
#[cfg(target_arch = "riscv32")]
pub fn protection_snapshot() -> crate::preview_queue::ProtectionSnapshot {
    macro_rules! csr {
        ($address:literal) => {{
            let value: u32;
            unsafe { core::arch::asm!("csrr {value}, {address}",
                value = out(reg) value, address = const $address, options(nostack)); }
            value
        }};
    }
    crate::preview_queue::ProtectionSnapshot {
        cpu: if esp_hal::system::Cpu::current() == esp_hal::system::Cpu::ProCpu {
            0
        } else {
            1
        },
        pma_cfg: [
            csr!(0xbc0),
            csr!(0xbc1),
            csr!(0xbc2),
            csr!(0xbc3),
            csr!(0xbc4),
            csr!(0xbc5),
            csr!(0xbc6),
            csr!(0xbc7),
            csr!(0xbc8),
            csr!(0xbc9),
            csr!(0xbca),
            csr!(0xbcb),
            csr!(0xbcc),
            csr!(0xbcd),
            csr!(0xbce),
            csr!(0xbcf),
        ],
        pma_addr: [
            csr!(0xbd0),
            csr!(0xbd1),
            csr!(0xbd2),
            csr!(0xbd3),
            csr!(0xbd4),
            csr!(0xbd5),
            csr!(0xbd6),
            csr!(0xbd7),
            csr!(0xbd8),
            csr!(0xbd9),
            csr!(0xbda),
            csr!(0xbdb),
            csr!(0xbdc),
            csr!(0xbdd),
            csr!(0xbde),
            csr!(0xbdf),
        ],
        pmp_cfg: [csr!(0x3a0), csr!(0x3a1), csr!(0x3a2), csr!(0x3a3)],
        pmp_addr: [
            csr!(0x3b0),
            csr!(0x3b1),
            csr!(0x3b2),
            csr!(0x3b3),
            csr!(0x3b4),
            csr!(0x3b5),
            csr!(0x3b6),
            csr!(0x3b7),
            csr!(0x3b8),
            csr!(0x3b9),
            csr!(0x3ba),
            csr!(0x3bb),
            csr!(0x3bc),
            csr!(0x3bd),
            csr!(0x3be),
            csr!(0x3bf),
        ],
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
    unsafe { enable_current_core_access(base, bytes) }
}

/// Check or enable CPU0 access only after CPU1 released its tested reservation.
///
/// # Safety
/// The caller may use only PREVIEW_BASE..PREVIEW_BASE+PREVIEW_BYTES, and must
/// have acquired `preview_ready`. Only one CPU0 queue may own that reservation.
#[cfg(target_arch = "riscv32")]
pub unsafe fn enable_core0_preview_access() -> Result<(), Error> {
    if esp_hal::system::Cpu::current() != esp_hal::system::Cpu::ProCpu {
        return Err(Error::WrongCore);
    }
    if !preview_ready() {
        return Err(Error::UnexpectedProtection);
    }
    if core0_existing_access(&protection_snapshot()) {
        return Ok(());
    }
    // The alternative is the exact ROM state already supported on CPU1. Unknown
    // or locked layouts fail before any CSR is changed or PSRAM is touched.
    unsafe { enable_current_core_access(BASE, BYTES) }
}

#[cfg(target_arch = "riscv32")]
unsafe fn enable_current_core_access(base: usize, bytes: usize) -> Result<(), Error> {
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

    fn observed_cpu0() -> crate::preview_queue::ProtectionSnapshot {
        // Exact diagnostic-capture CSR evidence, with the HAL's internal-memory
        // RX and RW guards still locked. These guards depend on linker symbols.
        crate::preview_queue::ProtectionSnapshot {
            cpu: 0,
            pma_cfg: [
                3221225473, 1, 1073741825, 1, 1073741825, 1073741845, 1073741825, 1073741845,
                1073741825, 1073741853, 1073741825, 1073741845, 1073741825, 1073741849, 1073741825,
                0,
            ],
            pma_addr: [
                33554431, 192946176, 197132288, 197263360, 199229440, 199311360, 268435456,
                285212672, 335544320, 352321536, 671088640, 687865856, 805306368, 822083584,
                1073741792, 0,
            ],
            pmp_cfg: [2340457856, 35712, 0, 0],
            pmp_addr: [
                197133120, 197135328, 197233088, 197243872, 197135360, 197233056, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0,
            ],
        }
    }

    #[test]
    fn observed_cpu0_map_accepts_internal_guards_without_changing_them() {
        let expected = observed_cpu0();
        let original_cfg = expected.pmp_cfg;
        let original_addr = expected.pmp_addr;
        assert!(core0_existing_access(&expected));
        assert_eq!(expected.pmp_cfg, original_cfg);
        assert_eq!(expected.pmp_addr, original_addr);
        let mut moved = expected;
        moved.pmp_addr[2] += 128; // Stack boundary may move between builds.
        assert!(core0_existing_access(&moved));
        let mut empty = expected;
        empty.pmp_cfg = [0; 4];
        assert!(core0_existing_access(&empty));
        let mut wrong_cpu = expected;
        wrong_cpu.cpu = 1;
        assert!(!core0_existing_access(&wrong_cpu));
    }

    #[test]
    fn cpu0_acceptance_declines_any_unknown_pma_layout() {
        let expected = observed_cpu0();
        for index in 0..16 {
            let mut altered = expected;
            altered.pma_cfg[index] ^= 8;
            assert!(!core0_existing_access(&altered));
            let mut altered = expected;
            altered.pma_addr[index] ^= 1;
            assert!(!core0_existing_access(&altered));
        }
    }

    #[test]
    fn every_pmp_address_mode_declines_full_or_partial_psram_overlap() {
        let mut addr = [0; 16];
        let mut cfg = [0; 4];
        // Place each mode in the last entry to exercise all configuration CSRs.
        for (mode, low, high) in [
            (1, (BASE / 4) as u32 - 1, (BASE / 4) as u32 + 1),
            (1, (BASE / 4) as u32, ((BASE + BYTES) / 4) as u32),
            (2, 0, (PREVIEW_BASE / 4) as u32),
            (3, 0, RAM_ADDRESS),
            (3, 0, u32::MAX), // A full-width NAPOT must not overflow to empty.
        ] {
            addr[14] = low;
            addr[15] = high;
            cfg[3] = ((mode << 3) | 0x83) << 24;
            assert!(!pmp_leaves_psram_unmatched(&cfg, &addr));
        }
        // OFF addresses are anchors, not independently matching regions.
        cfg[3] = 0x80 << 24;
        assert!(pmp_leaves_psram_unmatched(&cfg, &addr));
        // Disjoint NA4 and NAPOT entries leave PSRAM unmatched too.
        addr[15] = 0x0bc0_0000;
        for mode in [2, 3] {
            cfg[3] = ((mode << 3) | 0x83) << 24;
            assert!(pmp_leaves_psram_unmatched(&cfg, &addr));
        }
        // Reserved encoding and reversed TOR ranges are not accepted.
        cfg[3] = 0x8a << 24;
        assert!(!pmp_leaves_psram_unmatched(&cfg, &addr));
        cfg[3] = 0xab << 24;
        assert!(!pmp_leaves_psram_unmatched(&cfg, &addr));
        cfg[3] = 0x8b << 24;
        addr[14] = addr[15] + 1;
        assert!(!pmp_leaves_psram_unmatched(&cfg, &addr));
    }

    #[test]
    fn preview_reservation_is_disjoint_aligned_and_inside_tested_memory() {
        assert_eq!(PREVIEW_BASE, 0x50f0_0000);
        assert_eq!(PREVIEW_BASE + PREVIEW_BYTES, BASE + BYTES);
        assert_eq!(BASE + NAVIGATION_BYTES, PREVIEW_BASE);
        assert_eq!(PREVIEW_BASE % 64, 0);
        assert_eq!(PREVIEW_BYTES % 64, 0);
        const { assert!(NAVIGATION_BYTES > 2_107_760) };
    }
}
