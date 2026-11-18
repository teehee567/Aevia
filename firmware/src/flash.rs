// aligned unencrypted flash access for the loader through the sdk rom helpers

use esp_hal::{
    peripherals::{CPU_CTRL, FLASH},
    system::{Cpu, CpuControl, is_running},
};
use esp_storage::ll;

const WORD_SIZE: usize = 4;
const SECTOR_SIZE: u32 = 4096;

// the esp idf rom descriptor starts with the device id followed by the chip size
#[repr(C)]
struct RomFlashChip {
    _device_id: u32,
    chip_size: u32,
}

unsafe extern "C" {
    static rom_spiflash_legacy_data: *const RomFlashChip;
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error {
    NotAligned,
    OutOfBounds,
    Encrypted,
    Io(i32),
}

pub struct Flash<'d> {
    _flash: FLASH<'d>,
    cpu_control: CpuControl<'d>,
    capacity: usize,
    unlocked: bool,
}

impl<'d> Flash<'d> {
    pub fn new(flash: FLASH<'d>, cpu_control: CPU_CTRL<'d>) -> Self {
        // use the size set by the bootloader because reading the chip id is unreliable here
        let capacity =
            unsafe { rom_spiflash_legacy_data.as_ref() }.map_or(0, |chip| chip.chip_size as usize);

        Self {
            _flash: flash,
            cpu_control: CpuControl::new(cpu_control),
            capacity,
            unlocked: false,
        }
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }

    pub fn read_nor(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.check(offset, bytes.len(), WORD_SIZE)?;
        if !(bytes.as_ptr() as usize).is_multiple_of(WORD_SIZE) {
            return Err(Error::NotAligned);
        }

        // the buffer is checked and the sdk handles interrupts during the rom read
        unsafe { ll::spiflash_read(offset, bytes.as_mut_ptr().cast(), bytes.len() as u32) }
            .map_err(Error::Io)
    }

    pub fn write_nor(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.check(offset, bytes.len(), WORD_SIZE)?;
        if !(bytes.as_ptr() as usize).is_multiple_of(WORD_SIZE) {
            return Err(Error::NotAligned);
        }

        // the buffer is checked and the write helper parks the other core for the sdk rom call
        self.with_write(|| unsafe {
            ll::spiflash_write(offset, bytes.as_ptr().cast(), bytes.len() as u32)
        })
    }

    pub fn erase(&mut self, start: u32, end: u32) -> Result<(), Error> {
        let length = end.checked_sub(start).ok_or(Error::OutOfBounds)?;
        self.check(start, length as usize, SECTOR_SIZE as usize)?;

        for address in (start..end).step_by(SECTOR_SIZE as usize) {
            // the range contains whole sectors and the write helper handles the other core
            self.with_write(|| unsafe { ll::spiflash_erase_sector(address / SECTOR_SIZE) })?;
        }
        Ok(())
    }

    pub fn mapped_start(&self) -> u32 {
        // inspect the first spi mmu entry and restore the selector without changing the mapping
        unsafe {
            let selector = 0x2050_0380 as *mut u32;
            let entry = 0x2050_037c as *const u32;
            let previous = selector.read_volatile();

            selector.write_volatile(0);
            let page = entry.read_volatile() & 0x7ff;
            selector.write_volatile(previous);

            page << 16
        }
    }

    fn check(&self, offset: u32, length: usize, alignment: usize) -> Result<(), Error> {
        if esp_storage::flash_encryption() {
            return Err(Error::Encrypted);
        }
        if !(offset as usize).is_multiple_of(alignment) || !length.is_multiple_of(alignment) {
            return Err(Error::NotAligned);
        }
        if length > self.capacity || offset as usize > self.capacity - length {
            return Err(Error::OutOfBounds);
        }
        Ok(())
    }

    fn with_write(&mut self, operation: impl FnOnce() -> Result<(), i32>) -> Result<(), Error> {
        // Acquire the cross-core critical section BEFORE parking CPU1, so it
        // cannot be parked while owning a shared lock needed by this core.
        critical_section::with(|_| self.with_write_critical(operation))
    }

    fn with_write_critical(
        &mut self,
        operation: impl FnOnce() -> Result<(), i32>,
    ) -> Result<(), Error> {
        // park the other core only if it is running so stopped cores stay stopped
        let other_core = Cpu::other().find(|&core| is_running(core));
        if let Some(core) = other_core {
            unsafe { self.cpu_control.park_core(core) };
        }

        let result = (|| {
            if !self.unlocked {
                // this driver owns the flash and the sdk handles interrupts during the rom call
                unsafe { ll::spiflash_unlock() }?;
                self.unlocked = true;
            }
            operation()
        })();

        if let Some(core) = other_core {
            self.cpu_control.unpark_core(core);
        }
        result.map_err(Error::Io)
    }
}
