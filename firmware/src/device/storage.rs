use aevia_device_hal::memory::{EspFlash, V2_MINI_FLASH_BYTES};
use esp_bootloader_esp_idf::partitions::{self, AppPartitionSubType as App, PartitionType as Kind};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;

use crate::loader::{
    layout::{self, Partition},
    update::{Error, Flash},
};

pub struct Storage<'d>(EspFlash<'d>);

impl<'d> Storage<'d> {
    pub fn new(flash: FLASH<'d>) -> Result<(Self, u8), Error> {
        // Recover persists its mode while the estimator is still running.
        // The storage driver's default policy rejects writes with core 1 live.
        let mut flash = FlashStorage::new(flash).multicore_auto_park();
        let mut buffer = [0; 5 * 32];
        let table =
            partitions::read_partition_table(&mut flash, &mut buffer).map_err(|_| Error::Layout)?;

        // This loader uses the four-entry layout in partitions.csv.
        if table.len() != 4 || esp_storage::flash_encryption() {
            return Err(Error::Layout);
        }
        layout::validate(table.iter().map(|entry| Partition {
            kind: entry.raw_type(),
            subtype: entry.raw_subtype(),
            offset: entry.offset(),
            size: entry.len(),
            flags: entry.flags(),
        }))?;
        let running = match table
            .booted_partition()
            .map_err(|_| Error::Layout)?
            .map(|part| part.partition_type())
        {
            Some(Kind::App(App::Ota0)) => 0,
            Some(Kind::App(App::Ota1)) => 1,
            _ => return Err(Error::Layout),
        };
        let mut flash =
            EspFlash::v2_mini(flash, 0, V2_MINI_FLASH_BYTES).map_err(|_| Error::Layout)?;
        flash.initialize().map_err(|_| Error::Layout)?;
        Ok((Self(flash), running))
    }
}

impl Flash for Storage<'_> {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.0
            .read(u64::from(address), bytes)
            .map_err(|_| Error::Flash)
    }
    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), Error> {
        // Acquire the shared lock before esp-storage parks core 1, and keep
        // interrupts masked until it unparks. The driver's private flash lock
        // only covers the ROM call: otherwise a local interrupt can wait for
        // the shared lock while its owner is parked on the other core.
        critical_section::with(|_| {
            self.0
                .program(u64::from(address), bytes)
                .map_err(|_| Error::Flash)
        })
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let length = to.checked_sub(from).ok_or(Error::Flash)?;
        // Match write's lock ordering across the complete park/erase/unpark.
        // Loader callers erase one sector per operation, never an entire slot.
        critical_section::with(|_| {
            self.0
                .erase(u64::from(from), u64::from(length))
                .map_err(|_| Error::Flash)
        })
    }
}
