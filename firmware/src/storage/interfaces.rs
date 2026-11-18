/// One logical sector is always 512 bytes, independent of bus block size.
pub trait BlockStorage {
    type Error;
    fn sector_count(&self) -> u64;
    fn read_sector(&mut self, sector: u64, data: &mut [u8; 512]) -> Result<(), Self::Error>;
    fn write_sector(&mut self, sector: u64, data: &[u8; 512]) -> Result<(), Self::Error>;
    fn flush(&mut self) -> Result<(), Self::Error>;
}

/// Optional bounded contiguous reads, without changing single-sector writes.
/// Accept one to eight complete 512-byte sectors. Validate the entire range
/// before I/O; propagate transfer/CRC/status errors without partial success.
/// On error the caller must discard all returned buffer contents.
pub trait ReadSectors: BlockStorage {
    fn read_sectors(&mut self, first_sector: u64, data: &mut [u8]) -> Result<(), Self::Error>;
}
