//! `esp-hal`1.2.2 native SDMMC adapter; no SPI fallback or unsafe DMA pointers.
use super::{BusWidth, Command, NativeHost, ResponseKind, SECTOR_BYTES};
use esp_hal::{
    Blocking,
    dma::{DmaBufError, aligned::DmaAlignedMut},
    sdmmc::{self, BlockingError, CommandFlags, Slot},
};

/// Own a scratch buffer in internal SRAM (for example a normal SRAM stack or
/// static allocation). The SDK validates its memory region/alignment before DMA.
/// The reference must remain uniquely owned by this host. Scratch also handles
/// unaligned application buffers and immutable/flash-backed write inputs.
#[repr(C, align(64))]
pub struct DmaScratch {
    bytes: [u8; SECTOR_BYTES],
}
impl DmaScratch {
    pub const fn new() -> Self {
        Self {
            bytes: [0; SECTOR_BYTES],
        }
    }
}
impl Default for DmaScratch {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub enum Esp32SdHostError {
    Controller(BlockingError),
    Configuration(sdmmc::Error),
    Dma(DmaBufError),
    InvalidTransfer,
}

/// Wrap an already wired blocking Slot. V2 Mini uses SDHOST slot0's fixed
/// GPIO20..25 pads, with no card-detect or write-protect contact connected.
/// Creating this wrapper performs no I/O. `SdCard::initialize` sets clock/width.
pub struct Esp32SdHost<'a, 'd, const S: u8> {
    slot: Slot<'d, S, Blocking>,
    scratch: &'a mut DmaScratch,
}
impl<'a, 'd, const S: u8> Esp32SdHost<'a, 'd, S> {
    pub const fn new(slot: Slot<'d, S, Blocking>, scratch: &'a mut DmaScratch) -> Self {
        Self { slot, scratch }
    }
    pub fn release(self) -> (Slot<'d, S, Blocking>, &'a mut DmaScratch) {
        (self.slot, self.scratch)
    }
}
impl<const S: u8> NativeHost for Esp32SdHost<'_, '_, S> {
    type Error = Esp32SdHostError;
    fn set_bus(&mut self, width: BusWidth, frequency_hz: u32) -> Result<(), Self::Error> {
        self.slot
            .set_bus_low_level(
                match width {
                    BusWidth::One => sdmmc::BusWidth::Bit1,
                    BusWidth::Four => sdmmc::BusWidth::Bit4,
                },
                frequency_hz,
            )
            .map_err(Esp32SdHostError::Configuration)
    }
    fn send_initial_clocks(&mut self) -> Result<(), Self::Error> {
        self.slot
            .send_init_sequence()
            .map_err(Esp32SdHostError::Controller)
    }
    fn command(&mut self, command: Command) -> Result<[u32; 4], Self::Error> {
        self.slot
            .command_blocking(
                command.index,
                command.argument,
                match command.response {
                    ResponseKind::None => sdmmc::ResponseLen::None,
                    ResponseKind::Short => sdmmc::ResponseLen::Short,
                    ResponseKind::Long => sdmmc::ResponseLen::Long,
                },
                command.check_crc,
                CommandFlags {
                    wait_complete: true,
                    stop_abort: false,
                    busy: command.wait_busy,
                },
            )
            .map_err(Esp32SdHostError::Controller)
    }
    fn read_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &mut [u8],
    ) -> Result<u32, Self::Error> {
        if !matches!(data.len(), 8 | SECTOR_BYTES) {
            return Err(Esp32SdHostError::InvalidTransfer);
        }
        // S31 internal SRAM is uncached, with 4-byte DMA size alignment;
        // an 8-byte SCR is valid. PSRAM has 64-byte size alignment, hence
        // this scratch must be allocated in SRAM as documented above.
        let dma = DmaAlignedMut::new(&mut self.scratch.bytes[..data.len()])
            .map_err(Esp32SdHostError::Dma)?;
        let result = self
            .slot
            .read_blocks_blocking(command, argument, dma, data.len() as u16, 1)
            .map_err(Esp32SdHostError::Controller)?;
        data.copy_from_slice(&self.scratch.bytes[..data.len()]);
        Ok(result[0])
    }
    fn write_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &[u8; SECTOR_BYTES],
    ) -> Result<u32, Self::Error> {
        self.scratch.bytes.copy_from_slice(data);
        let dma = DmaAlignedMut::new(&mut self.scratch.bytes[..]).map_err(Esp32SdHostError::Dma)?;
        let result = self
            .slot
            .write_blocks_blocking(command, argument, dma, SECTOR_BYTES as u16, 1)
            .map_err(Esp32SdHostError::Controller)?;
        Ok(result[0])
    }
    fn card_present(&self) -> bool {
        self.slot.is_card_present()
    }
    fn write_protected(&self) -> bool {
        self.slot.is_write_protected()
    }
    fn is_response_timeout(error: &Self::Error) -> bool {
        matches!(
            error,
            Esp32SdHostError::Controller(BlockingError::Op(sdmmc::Error::ResponseTimeout))
        )
    }
}
