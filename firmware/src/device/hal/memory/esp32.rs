//! ESP storage adapter. Source: installed esp-storage0.10 `common.rs`,
//! `nor_flash.rs`, `storage.rs` and `lib.rs`; initialization/encryption and
//! multicore policy remain explicit. The SDK owns flash-execution exclusion.
use super::{Error, NorRegion};
use esp_storage::{FlashStorage, FlashStorageError};

/// Raw, unencrypted flash region. Supply an already initialized unique vendor
/// `FlashStorage`; creating that vendor object can perform ROM setup and is the
/// caller's responsibility. Call `initialize()` to check encryption before I/O.
/// The SDK's default policy rejects writes while another core runs. Opt in to
/// SDK auto-parking with `with_multicore_auto_park` when appropriate.
///
/// Unaligned slice pointers can require a 4096-byte temporary SDK stack buffer;
/// use word-aligned buffers or budget the caller's stack accordingly.
pub struct EspFlash<'d> {
    region: NorRegion<FlashStorage<'d>>,
    initialized: bool,
}
impl<'d> EspFlash<'d> {
    pub fn new(
        storage: FlashStorage<'d>,
        base: u64,
        length: u64,
    ) -> Result<Self, Error<FlashStorageError>> {
        Ok(Self {
            region: NorRegion::new(storage, base, length)?,
            initialized: false,
        })
    }
    pub fn v2_mini(
        storage: FlashStorage<'d>,
        base: u64,
        length: u64,
    ) -> Result<Self, Error<FlashStorageError>> {
        Ok(Self {
            region: NorRegion::v2_mini(storage, base, length)?,
            initialized: false,
        })
    }
    /// Reads the hardware encryption state; never erases/programs or changes it.
    pub fn initialize(&mut self) -> Result<(), Error<FlashStorageError>> {
        self.initialized = false;
        if esp_storage::flash_encryption() {
            return Err(Error::EncryptedFlash);
        }
        self.initialized = true;
        Ok(())
    }
    /// Select the SDK's automatic flash-write coordination rather than
    /// ignoring another core. Does no I/O; parking occurs during later writes.
    pub fn with_multicore_auto_park(mut self) -> Self {
        self.region.backend = self.region.backend.multicore_auto_park();
        self
    }
    pub fn release(self) -> FlashStorage<'d> {
        self.region.release()
    }
    pub const fn capacity_bytes(&self) -> u64 {
        self.region.capacity_bytes()
    }
    pub const fn erase_size_bytes(&self) -> u32 {
        self.region.erase_size_bytes()
    }
    pub const fn base_offset_bytes(&self) -> u64 {
        self.region.base_offset_bytes()
    }
    fn ready(&self) -> Result<(), Error<FlashStorageError>> {
        if self.initialized {
            Ok(())
        } else {
            Err(Error::NotInitialized)
        }
    }
    pub fn read(&mut self, offset: u64, data: &mut [u8]) -> Result<(), Error<FlashStorageError>> {
        self.ready()?;
        self.region.read(offset, data)
    }
    pub fn program(&mut self, offset: u64, data: &[u8]) -> Result<(), Error<FlashStorageError>> {
        self.ready()?;
        self.region.program(offset, data)
    }
    pub fn erase(&mut self, offset: u64, length: u64) -> Result<(), Error<FlashStorageError>> {
        self.ready()?;
        self.region.erase(offset, length)
    }
}
impl crate::interfaces::NorStorage for EspFlash<'_> {
    type Error = Error<FlashStorageError>;
    fn capacity_bytes(&self) -> u64 {
        EspFlash::capacity_bytes(self)
    }
    fn erase_size_bytes(&self) -> u32 {
        EspFlash::erase_size_bytes(self)
    }
    fn read(&mut self, offset: u64, data: &mut [u8]) -> Result<(), Self::Error> {
        EspFlash::read(self, offset, data)
    }
    fn program(&mut self, offset: u64, data: &[u8]) -> Result<(), Self::Error> {
        EspFlash::program(self, offset, data)
    }
    fn erase(&mut self, offset: u64, length: u64) -> Result<(), Self::Error> {
        EspFlash::erase(self, offset, length)
    }
}
