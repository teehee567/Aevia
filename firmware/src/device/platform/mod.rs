//! S31 support missing from the pinned vendor SDK. Board construction is the
//! only initialization caller; application modules consume clocks and memory.
pub(crate) mod counter;
mod cycles;
mod display;
mod imu_spi;
mod memory;

#[cfg(target_arch = "riscv32")]
pub(crate) mod capture;
#[cfg(target_arch = "riscv32")]
pub(crate) use cycles::native::ImuDelay;
#[cfg(target_arch = "riscv32")]
pub(crate) use display::native::DisplayBus;
#[cfg(target_arch = "riscv32")]
pub(crate) use imu_spi::native::ImuBus;
#[cfg(target_arch = "riscv32")]
pub(crate) use imu_spi::native::{
    diagnostics as imu_spi_diagnostics, set_context as imu_spi_context,
};

#[cfg(target_arch = "riscv32")]
pub(crate) struct EstimatorMemory(esp_hal::psram::Psram);

/// Call after the board's PPS/DRDY pins are digital inputs, before CPU1 starts.
/// The platform exclusively reserves MCPWM0 and its capture matrix signals.
#[cfg(target_arch = "riscv32")]
pub(crate) fn initialize(psram: esp_hal::peripherals::PSRAM<'static>) -> EstimatorMemory {
    use aevia_device_hal::board;
    let psram = esp_hal::psram::Psram::new(
        psram,
        esp_hal::psram::PsramConfig {
            timing: esp_hal::psram::PsramTimingParams::MHZ_125,
            ..Default::default()
        },
    );
    enable_second_core_cache();
    capture::init(board::gnss::PPS, board::imu::DRDY);
    EstimatorMemory(psram)
}

#[cfg(target_arch = "riscv32")]
impl EstimatorMemory {
    /// Consume the reservation on CPU1; restore its PMA and destructively check
    /// physical memory before returning a range for the worker's exclusive use.
    pub(crate) fn prepare(self) -> Result<(usize, usize), memory::Error> {
        let (address, len) = self.0.raw_parts();
        memory::bring_up(self.0)?;
        Ok((address as usize, len))
    }
}

/// ESP-IDF cache_ll_l1_enable_bus(1, CACHE_BUS_IBUS0). CPU1 is held in reset.
#[cfg(target_arch = "riscv32")]
#[esp_hal::ram]
fn enable_second_core_cache() {
    // SAFETY: startup's only caller runs before releasing CPU1. CPU0's live
    // instruction cache and the shared data cache configuration are untouched.
    let cache = unsafe { &*esp32s31::CACHE::ptr() };
    cache
        .l1_icache_ctrl()
        .modify(|_, w| w.l1_icache_shut_ibus1().clear_bit());
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}
