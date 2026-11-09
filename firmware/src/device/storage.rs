use esp_bootloader_esp_idf::partitions::{self, AppPartitionSubType as App, PartitionType as Kind};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;

use crate::loader::update::{Error, Flash};

pub struct Storage<'d>(FlashStorage<'d>);

impl<'d> Storage<'d> {
    pub fn new(flash: FLASH<'d>) -> Result<(Self, u8), Error> {
        let mut flash = FlashStorage::new(flash);
        let mut buffer = [0; 5 * 32];
        let table =
            partitions::read_partition_table(&mut flash, &mut buffer).map_err(|_| Error::Layout)?;

        // This loader uses the four-entry layout in partitions.csv.
        if table.len() != 4 || esp_storage::flash_encryption() {
            return Err(Error::Layout);
        }
        let running = match table
            .booted_partition()
            .map_err(|_| Error::Layout)?
            .map(|part| part.partition_type())
        {
            Some(Kind::App(App::Ota0)) => 0,
            Some(Kind::App(App::Ota1)) => 1,
            _ => return Err(Error::Layout),
        };
        Ok((Self(flash), running))
    }
}

impl Flash for Storage<'_> {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.0.read_nor(address, bytes).map_err(|_| Error::Flash)
    }
    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), Error> {
        self.0.write_nor(address, bytes).map_err(|_| Error::Flash)
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.0.erase(from, to).map_err(|_| Error::Flash)
    }
}
