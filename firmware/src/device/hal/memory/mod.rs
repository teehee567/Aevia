//! Bounded NOR storage and safe, exclusively owned RAM / PSRAM views.
//!
//! The local `data/datasheets/esp32-s31-wroom-3_datasheet_en.pdf` Table 1-1
//! identifies Mini's N16R16V as 16 MB Quad-SPI flash plus 16 MB Octal-SPI PSRAM.
//! The earlier V2 BOM names MX25U51279GXDR00 and S80KS5122GABHV020, while its
//! netlist identifies APS512XXN (with an APS256XXN symbol) instead. None of those
//! external-memory command sets is supplied in `data/datasheets`; this module
//! does not infer interchangeable opcodes, voltage, latency or bus configuration.
//! Select and initialize an exact, verified MCU/backend before constructing a
//! view. `Part` records that distinction without inventing legacy geometry.
//!
//! `NorRegion` translates only an explicitly selected byte range to an
//! `embedded-storage` 0.3 NOR backend. It never rounds an erase outward, erases
//! during programming, or reconstructs partially written data. The backend owns
//! program/erase completion, flash-execution exclusion, multicore/cache handling
//! and hardware deadlines. NOR writes retain its erase-before-write / repeated
//! programming restrictions and can be partially completed on an I/O failure.
//!
//! `RamSlice` accepts a valid exclusive Rust slice, including a caller-owned
//! PSRAM allocation. Vendor initialization, PMA mapping, cache configuration and
//! allocator lifetime remain with the caller; no raw pointer is accepted here.
//! No constructor performs I/O, clears RAM, or changes memory/controller mode.

use core::convert::Infallible;
use embedded_storage::nor_flash::{NorFlash, NorFlashError, NorFlashErrorKind};

#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
mod esp32;
#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
pub use esp32::EspFlash;

pub const V2_MINI_FLASH_BYTES: u64 = 16 * 1024 * 1024;
pub const V2_MINI_PSRAM_BYTES: usize = 16 * 1024 * 1024;

/// Identity metadata only; it never selects a bus mode or probes the device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Part {
    Esp32S31N16R16VFlash,
    Esp32S31N16R16VPsram,
    MacronixMx25u51279Gxdr00,
    InfineonS80ks5122Gabhv020,
    ApMemoryAps512xxn,
}
impl Part {
    /// Only capacities explicitly verified by the supplied module datasheet.
    /// Legacy parts require a selected backend's independently verified geometry.
    pub const fn verified_capacity_bytes(self) -> Option<u64> {
        match self {
            Self::Esp32S31N16R16VFlash => Some(V2_MINI_FLASH_BYTES),
            Self::Esp32S31N16R16VPsram => Some(V2_MINI_PSRAM_BYTES as u64),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<E> {
    Backend(E),
    InvalidGeometry,
    OutOfBounds,
    Unaligned,
    AddressWidth,
    NotInitialized,
    EncryptedFlash,
    CapacityMismatch { expected: u64, actual: u64 },
}
impl<E: NorFlashError> NorFlashError for Error<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Backend(error) => error.kind(),
            Self::OutOfBounds | Self::AddressWidth => NorFlashErrorKind::OutOfBounds,
            Self::Unaligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// An explicit erase-aligned subrange. Reads/programs/erases are relative to
/// this range and cannot access neighboring bootloader, application or metadata
/// regions. This is a bounds mechanism, not an interpretation of a partition
/// table: the caller must choose a range that may actually be modified.
///
/// Pass `&mut backend` to retain its ownership when construction fails. Geometry
/// and capacity getters must be metadata-only, as required of the backend here.
pub struct NorRegion<F> {
    backend: F,
    base: u32,
    length: u32,
}
impl<F: NorFlash> NorRegion<F> {
    pub fn new(backend: F, base: u64, length: u64) -> Result<Self, Error<F::Error>> {
        let sizes = [F::READ_SIZE, F::WRITE_SIZE, F::ERASE_SIZE];
        if sizes
            .iter()
            .any(|size| *size == 0 || *size > u32::MAX as usize)
        {
            return Err(Error::InvalidGeometry);
        }
        let end = base.checked_add(length).ok_or(Error::OutOfBounds)?;
        if end > backend.capacity() as u64 {
            return Err(Error::OutOfBounds);
        }
        // NorFlash uses u32 for both start and exclusive erase end.
        if end > u64::from(u32::MAX) {
            return Err(Error::AddressWidth);
        }
        if sizes
            .iter()
            .any(|size| !base.is_multiple_of(*size as u64) || !length.is_multiple_of(*size as u64))
        {
            return Err(Error::Unaligned);
        }
        Ok(Self {
            backend,
            base: base as u32,
            length: length as u32,
        })
    }

    /// Verify the supplied backend is the Mini module's full 16 MiB flash,
    /// then constrain it to the explicitly requested range. No part is probed.
    pub fn v2_mini(backend: F, base: u64, length: u64) -> Result<Self, Error<F::Error>> {
        let actual = backend.capacity() as u64;
        if actual != V2_MINI_FLASH_BYTES {
            return Err(Error::CapacityMismatch {
                expected: V2_MINI_FLASH_BYTES,
                actual,
            });
        }
        Self::new(backend, base, length)
    }
    pub const fn base_offset_bytes(&self) -> u64 {
        self.base as u64
    }
    pub const fn capacity_bytes(&self) -> u64 {
        self.length as u64
    }
    pub const fn read_size_bytes(&self) -> usize {
        F::READ_SIZE
    }
    pub const fn program_size_bytes(&self) -> usize {
        F::WRITE_SIZE
    }
    pub const fn erase_size_bytes(&self) -> u32 {
        F::ERASE_SIZE as u32
    }
    pub fn release(self) -> F {
        self.backend
    }

    fn checked_range(
        &self,
        offset: u64,
        length: u64,
        alignment: usize,
    ) -> Result<u32, Error<F::Error>> {
        let end = offset.checked_add(length).ok_or(Error::OutOfBounds)?;
        if end > u64::from(self.length) {
            return Err(Error::OutOfBounds);
        }
        // Empty operations within bounds, including at end, never touch the
        // backend and do not require transfer alignment.
        if length != 0
            && (!offset.is_multiple_of(alignment as u64)
                || !length.is_multiple_of(alignment as u64))
        {
            return Err(Error::Unaligned);
        }
        Ok(self.base + offset as u32)
    }
    pub fn read(&mut self, offset: u64, bytes: &mut [u8]) -> Result<(), Error<F::Error>> {
        let address = self.checked_range(offset, bytes.len() as u64, F::READ_SIZE)?;
        if bytes.is_empty() {
            return Ok(());
        }
        self.backend.read(address, bytes).map_err(Error::Backend)
    }
    /// Explicit NOR programming. This never implicitly erases or retries.
    pub fn program(&mut self, offset: u64, bytes: &[u8]) -> Result<(), Error<F::Error>> {
        let address = self.checked_range(offset, bytes.len() as u64, F::WRITE_SIZE)?;
        if bytes.is_empty() {
            return Ok(());
        }
        self.backend.write(address, bytes).map_err(Error::Backend)
    }
    /// Erase exactly `length` bytes starting at relative `offset`. Both must
    /// be erase-aligned; the second argument is a length, not an end address.
    pub fn erase(&mut self, offset: u64, length: u64) -> Result<(), Error<F::Error>> {
        let start = self.checked_range(offset, length, F::ERASE_SIZE)?;
        if length == 0 {
            return Ok(());
        }
        self.backend
            .erase(start, start + length as u32)
            .map_err(Error::Backend)
    }
}
impl<F: NorFlash> crate::interfaces::NorStorage for NorRegion<F> {
    type Error = Error<F::Error>;
    fn capacity_bytes(&self) -> u64 {
        NorRegion::capacity_bytes(self)
    }
    fn erase_size_bytes(&self) -> u32 {
        NorRegion::erase_size_bytes(self)
    }
    fn read(&mut self, offset: u64, data: &mut [u8]) -> Result<(), Self::Error> {
        NorRegion::read(self, offset, data)
    }
    fn program(&mut self, offset: u64, data: &[u8]) -> Result<(), Self::Error> {
        NorRegion::program(self, offset, data)
    }
    fn erase(&mut self, offset: u64, length: u64) -> Result<(), Self::Error> {
        NorRegion::erase(self, offset, length)
    }
}

/// Borrowed RAM view. The same type works with internal SRAM or an exclusively
/// owned PSRAM allocation; it makes no claim about the slice's physical origin.
pub struct RamSlice<'a> {
    bytes: &'a mut [u8],
}
impl<'a> RamSlice<'a> {
    pub const fn new(bytes: &'a mut [u8]) -> Self {
        Self { bytes }
    }
    pub const fn capacity_bytes(&self) -> usize {
        self.bytes.len()
    }
    pub fn release(self) -> &'a mut [u8] {
        self.bytes
    }
    pub fn read(&self, offset: usize, data: &mut [u8]) -> Result<(), Error<Infallible>> {
        let end = offset.checked_add(data.len()).ok_or(Error::OutOfBounds)?;
        let source = self.bytes.get(offset..end).ok_or(Error::OutOfBounds)?;
        data.copy_from_slice(source);
        Ok(())
    }
    pub fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), Error<Infallible>> {
        let end = offset.checked_add(data.len()).ok_or(Error::OutOfBounds)?;
        let target = self.bytes.get_mut(offset..end).ok_or(Error::OutOfBounds)?;
        target.copy_from_slice(data);
        Ok(())
    }
    pub fn window(
        &mut self,
        offset: usize,
        length: usize,
    ) -> Result<RamSlice<'_>, Error<Infallible>> {
        let end = offset.checked_add(length).ok_or(Error::OutOfBounds)?;
        Ok(RamSlice::new(
            self.bytes.get_mut(offset..end).ok_or(Error::OutOfBounds)?,
        ))
    }
}
impl crate::interfaces::RamStorage for RamSlice<'_> {
    type Error = Error<Infallible>;
    fn capacity_bytes(&self) -> usize {
        RamSlice::capacity_bytes(self)
    }
    fn read(&mut self, offset: usize, data: &mut [u8]) -> Result<(), Self::Error> {
        RamSlice::read(self, offset, data)
    }
    fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), Self::Error> {
        RamSlice::write(self, offset, data)
    }
}

#[cfg(test)]
mod tests;
