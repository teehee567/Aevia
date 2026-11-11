//! Native SD memory cards for V2 Mini's 4-bit SDMMC wiring.
//!
//! Protocol source: SD Association, Physical Layer Simplified Specification
//! [v6.00](https://www.sdcard.org/cms/wp-content/themes/sdcard-org/dl.php?f=Part1_Physical_Layer_Simplified_Specification_Ver6.00.pdf),
//! §§4.2, 4.3, 4.7, 4.9, 4.10, 5.3 and 5.6. The installed `sdio` 0.5.3 source
//! corroborates command/response layouts. Host API and DMA handling follow
//! installed `esp-hal` 1.2.2 `sdmmc` and `dma::aligned` implementations.
//!
//! This driver uses native SD, not SPI or SDIO I/O devices. It supports SDSC,
//! SDHC and SDXC at 3.3 V, default-speed signaling up to 25 MHz, single-sector
//! writes and reads of up to eight sectors. It does not switch to 1.8 V,
//! UHS/high-speed modes or queued writes.
//! Supply stabilization, peripheral/pin setup, electrical pullups, filesystem and
//! bus synchronization belong to the caller. Constructors perform no I/O.
//!
//! Polling has explicit finite attempt budgets. Each host operation must itself
//! enforce a hardware deadline; a polling budget cannot interrupt a stalled host.
//! V2 Mini has no wired card-detect pin. Removal is then discovered by protocol
//! failure; a successfully replaced card still requires explicit initialization.
//! Any host/status failure invalidates the cached initialization. A failed write
//! can have modified the card; no power-loss atomicity is promised.

use crate::interfaces::{BlockStorage, ReadSectors};
use embedded_hal::delay::DelayNs;

#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
mod esp32;
#[cfg(all(feature = "esp32s31", target_arch = "riscv32"))]
pub use esp32::{DmaScratch, Esp32SdHost, Esp32SdHostError};

pub const SECTOR_BYTES: usize = 512;
pub const MAX_READ_SECTORS: usize = 8;
pub const MAX_READ_BYTES: usize = MAX_READ_SECTORS * SECTOR_BYTES;
pub const INITIAL_CLOCK_HZ: u32 = 400_000;
pub const MAX_DEFAULT_CLOCK_HZ: u32 = 25_000_000;
const VOLTAGE_WINDOW: u32 = 0x0030_0000; // 3.2–3.4 V, 3.3 V host.
const R1_ERRORS: u32 = 0xfff9_a008;
const APP_CMD: u32 = 1 << 5;
const READY: u32 = 1 << 8;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusWidth {
    One,
    Four,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResponseKind {
    None,
    Short,
    Long,
}

/// Native command. Long responses use numeric little-word order: words[0]
/// holds bits31:0, words[3] holds bits127:96. A short payload is words[0].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Command {
    pub index: u8,
    pub argument: u32,
    pub response: ResponseKind,
    pub check_crc: bool,
    pub wait_busy: bool,
}
impl Command {
    const fn short(index: u8, argument: u32) -> Self {
        Self {
            index,
            argument,
            response: ResponseKind::Short,
            check_crc: true,
            wait_busy: false,
        }
    }
    const fn long(index: u8, argument: u32) -> Self {
        Self {
            index,
            argument,
            response: ResponseKind::Long,
            check_crc: true,
            wait_busy: false,
        }
    }
}

/// Single-owner native SD host. Implementations must check command/data CRCs
/// where requested, complete DMA before return, and bound command/data/busy waits.
/// Data functions perform one transfer and return the card's raw R1 status.
pub trait NativeHost {
    type Error;
    fn set_bus(&mut self, width: BusWidth, frequency_hz: u32) -> Result<(), Self::Error>;
    /// At least 74 clocks with CMD/data released high, after power stabilizes.
    fn send_initial_clocks(&mut self) -> Result<(), Self::Error>;
    fn command(&mut self, command: Command) -> Result<[u32; 4], Self::Error>;
    /// CRC-checked CMD51/SCR (8 bytes), CMD17 (512 bytes), or CMD18 (2..8
    /// sectors). CMD18 must wait for CMD12 stop completion before return and
    /// preserve detected command/data/stop failures. No partial success is
    /// permitted. The caller also checks card status after the transfer.
    fn read_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &mut [u8],
    ) -> Result<u32, Self::Error>;
    fn write_data(
        &mut self,
        command: u8,
        argument: u32,
        data: &[u8; SECTOR_BYTES],
    ) -> Result<u32, Self::Error>;
    /// Without a CD contact, return true; protocol errors detect absence.
    fn card_present(&self) -> bool {
        true
    }
    fn write_protected(&self) -> bool {
        false
    }
    /// Only a response timeout permits legacy SDv1 CMD8 fallback. CRC, data,
    /// arbitration and other transport errors must return false here.
    fn is_response_timeout(error: &Self::Error) -> bool;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub bus_width: BusWidth,
    pub clock_hz: u32,
    /// ACMD41 attempts, one millisecond delay between attempts.
    pub initialization_attempts: u16,
    /// CMD13 attempts, one millisecond delay between attempts.
    pub ready_attempts: u16,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            bus_width: BusWidth::Four,
            clock_hz: MAX_DEFAULT_CLOCK_HZ,
            initialization_attempts: 1000,
            ready_attempts: 1000,
        }
    }
}
impl Config {
    fn valid(self) -> bool {
        (INITIAL_CLOCK_HZ..=MAX_DEFAULT_CLOCK_HZ).contains(&self.clock_hz)
            && self.initialization_attempts != 0
            && self.ready_attempts != 0
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    Host(E),
    InvalidConfiguration,
    NotInitialized,
    NoCard,
    WriteProtected,
    OutOfRange,
    UnsupportedCard,
    Voltage,
    InterfaceCondition,
    InvalidCsd,
    InvalidScr,
    ApplicationCommand,
    InvalidRca,
    InitializationTimeout,
    ReadyTimeout,
    CardStatus(u32),
    CardState(u8),
    InvalidLength,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Status {
    pub raw: u32,
}
impl Status {
    pub const fn ready_for_data(self) -> bool {
        self.raw & READY != 0
    }
    pub const fn state(self) -> u8 {
        ((self.raw >> 9) & 15) as u8
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CardInfo {
    pub relative_address: u16,
    pub block_addressed: bool,
    pub sector_count: u64,
    pub bus_width: BusWidth,
    pub clock_hz: u32,
    pub write_protected: bool,
    /// CID and CSD are numeric little-word order as specified by NativeHost.
    pub cid: [u32; 4],
    pub csd: [u32; 4],
    pub scr: [u8; 8],
    pub operation_conditions: u32,
}

pub struct SdCard<H, D> {
    host: H,
    delay: D,
    info: Option<CardInfo>,
    ready_attempts: u16,
}
impl<H, D> SdCard<H, D> {
    pub const fn new(host: H, delay: D) -> Self {
        Self {
            host,
            delay,
            info: None,
            ready_attempts: 0,
        }
    }
    pub const fn info(&self) -> Option<&CardInfo> {
        self.info.as_ref()
    }
    pub fn release(self) -> (H, D) {
        (self.host, self.delay)
    }
    /// Call after an externally detected removal/reset or external bus access.
    pub fn invalidate(&mut self) {
        self.info = None;
    }
}
impl<H: NativeHost, D: DelayNs> SdCard<H, D> {
    pub fn initialize(&mut self, config: Config) -> Result<CardInfo, Error<H::Error>> {
        if !config.valid() {
            return Err(Error::InvalidConfiguration);
        }
        self.info = None;
        self.ready_attempts = config.ready_attempts;
        self.present()?;
        self.host
            .set_bus(BusWidth::One, INITIAL_CLOCK_HZ)
            .map_err(Error::Host)?;
        self.delay.delay_ms(1);
        self.host.send_initial_clocks().map_err(Error::Host)?;
        self.command(Command {
            index: 0,
            argument: 0,
            response: ResponseKind::None,
            check_crc: false,
            wait_busy: false,
        })?;
        let version2 = match self.host.command(Command::short(8, 0x1aa)) {
            Ok(words) if words[0] & (1 << 22) != 0 && words[0] & (R1_ERRORS & !(1 << 22)) == 0 => {
                false
            }
            Ok(words) if words[0] == 0x1aa => true,
            Ok(_) => return Err(Error::InterfaceCondition),
            Err(e) if H::is_response_timeout(&e) => false,
            Err(e) => return Err(Error::Host(e)),
        };
        let mut ocr = 0;
        for attempt in 0..config.initialization_attempts {
            self.app_command(0)?;
            let mut command =
                Command::short(41, VOLTAGE_WINDOW | if version2 { 1 << 30 } else { 0 });
            command.check_crc = false; // OCR R3 carries no CRC.
            ocr = self.command(command)?[0];
            if ocr & (1 << 31) != 0 {
                break;
            }
            if attempt + 1 < config.initialization_attempts {
                self.delay.delay_ms(1);
            }
        }
        if ocr & (1 << 31) == 0 {
            return Err(Error::InitializationTimeout);
        }
        if ocr & VOLTAGE_WINDOW == 0 {
            return Err(Error::Voltage);
        }
        if ocr & (1 << 24 | 1 << 27) != 0 {
            return Err(Error::UnsupportedCard);
        }
        let block_addressed = version2 && ocr & (1 << 30) != 0;
        let cid = self.command(Command::long(2, 0))?;
        let r6 = self.command(Command::short(3, 0))?[0];
        if r6 & 0xe000 != 0 {
            return Err(Error::CardStatus(r6 & 0xffff));
        }
        let rca = (r6 >> 16) as u16;
        if rca == 0 {
            return Err(Error::InvalidRca);
        }
        let csd = self.command(Command::long(9, u32::from(rca) << 16))?;
        let geometry = Geometry::parse(csd, block_addressed).map_err(|e| match e {
            GeometryError::Unsupported => Error::UnsupportedCard,
            GeometryError::Invalid => Error::InvalidCsd,
        })?;
        let mut select = Command::short(7, u32::from(rca) << 16);
        select.wait_busy = true;
        let selected = self.command(select)?[0];
        self.check_status(selected)?;
        self.wait_ready(rca)?;
        self.app_command(rca)?;
        let mut scr = [0; 8];
        let status = self.host.read_data(51, 0, &mut scr).map_err(Error::Host)?;
        self.check_status(status)?;
        let value = u64::from_be_bytes(scr);
        if value >> 60 != 0 || (value >> 48) & 1 == 0 {
            return Err(Error::InvalidScr);
        }
        let bus_width = if config.bus_width == BusWidth::Four && (value >> 48) & 4 != 0 {
            BusWidth::Four
        } else {
            BusWidth::One
        };
        self.app_command(rca)?;
        let width = self.command(Command::short(
            6,
            if bus_width == BusWidth::Four { 2 } else { 0 },
        ))?[0];
        self.check_status(width)?;
        let clock_hz = config.clock_hz.min(geometry.maximum_clock_hz);
        self.host
            .set_bus(bus_width, clock_hz)
            .map_err(Error::Host)?;
        if !block_addressed {
            let block_length = self.command(Command::short(16, SECTOR_BYTES as u32))?[0];
            self.check_status(block_length)?;
        }
        self.wait_ready(rca)?;
        let info = CardInfo {
            relative_address: rca,
            block_addressed,
            sector_count: geometry.sectors,
            bus_width,
            clock_hz,
            write_protected: geometry.read_only,
            cid,
            csd,
            scr,
            operation_conditions: ocr,
        };
        self.info = Some(info);
        Ok(info)
    }

    fn present(&mut self) -> Result<(), Error<H::Error>> {
        if self.host.card_present() {
            Ok(())
        } else {
            self.info = None;
            Err(Error::NoCard)
        }
    }
    fn command(&mut self, command: Command) -> Result<[u32; 4], Error<H::Error>> {
        self.present()?;
        self.host.command(command).map_err(|e| {
            self.info = None;
            Error::Host(e)
        })
    }
    fn check_status(&mut self, status: u32) -> Result<Status, Error<H::Error>> {
        if status & R1_ERRORS != 0 {
            self.info = None;
            Err(Error::CardStatus(status))
        } else {
            Ok(Status { raw: status })
        }
    }
    fn app_command(&mut self, rca: u16) -> Result<(), Error<H::Error>> {
        let response = self.command(Command::short(55, u32::from(rca) << 16))?[0];
        self.check_status(response)?;
        if response & APP_CMD == 0 {
            return Err(Error::ApplicationCommand);
        }
        Ok(())
    }
    fn wait_ready(&mut self, rca: u16) -> Result<Status, Error<H::Error>> {
        for attempt in 0..self.ready_attempts {
            let response = self.command(Command::short(13, u32::from(rca) << 16))?[0];
            let status = self.check_status(response)?;
            if status.state() == 4 && status.ready_for_data() {
                return Ok(status);
            }
            if !matches!(status.state(), 4..=7) {
                self.info = None;
                return Err(Error::CardState(status.state()));
            }
            if attempt + 1 < self.ready_attempts {
                self.delay.delay_ms(1);
            }
        }
        self.info = None;
        Err(Error::ReadyTimeout)
    }
    pub fn status(&mut self) -> Result<Status, Error<H::Error>> {
        let info = self.info.ok_or(Error::NotInitialized)?;
        let response = self.command(Command::short(13, u32::from(info.relative_address) << 16))?[0];
        self.check_status(response)
    }
    fn address(&self, sector: u64) -> Result<(CardInfo, u32), Error<H::Error>> {
        let info = self.info.ok_or(Error::NotInitialized)?;
        if sector >= info.sector_count {
            return Err(Error::OutOfRange);
        }
        let value = if info.block_addressed {
            sector
        } else {
            sector
                .checked_mul(SECTOR_BYTES as u64)
                .ok_or(Error::OutOfRange)?
        };
        Ok((info, u32::try_from(value).map_err(|_| Error::OutOfRange)?))
    }
    pub fn read_sector(
        &mut self,
        sector: u64,
        data: &mut [u8; SECTOR_BYTES],
    ) -> Result<(), Error<H::Error>> {
        self.read_sectors(sector, data)
    }
    /// Read a bounded contiguous range. Entire-range validation precedes the
    /// first card command; transport/status failures invalidate initialization.
    /// Discard the complete destination buffer if this returns an error.
    pub fn read_sectors(&mut self, sector: u64, data: &mut [u8]) -> Result<(), Error<H::Error>> {
        if data.is_empty()
            || data.len() > MAX_READ_BYTES
            || !data.len().is_multiple_of(SECTOR_BYTES)
        {
            return Err(Error::InvalidLength);
        }
        let count = (data.len() / SECTOR_BYTES) as u64;
        let (info, address) = self.address(sector)?;
        let last = sector.checked_add(count - 1).ok_or(Error::OutOfRange)?;
        // Also checks byte-address representability for SDSC, not only capacity.
        self.address(last)?;
        self.wait_ready(info.relative_address)?;
        let command = if count == 1 { 17 } else { 18 };
        let response = self.host.read_data(command, address, data).map_err(|e| {
            self.info = None;
            Error::Host(e)
        })?;
        self.check_status(response)?;
        self.wait_ready(info.relative_address)?;
        Ok(())
    }
    pub fn write_sector(
        &mut self,
        sector: u64,
        data: &[u8; SECTOR_BYTES],
    ) -> Result<(), Error<H::Error>> {
        let (info, address) = self.address(sector)?;
        self.present()?;
        if info.write_protected || self.host.write_protected() {
            return Err(Error::WriteProtected);
        }
        self.wait_ready(info.relative_address)?;
        let response = self.host.write_data(24, address, data).map_err(|e| {
            self.info = None;
            Error::Host(e)
        })?;
        self.check_status(response)?;
        self.wait_ready(info.relative_address)?;
        Ok(())
    }
    pub fn flush(&mut self) -> Result<(), Error<H::Error>> {
        let info = self.info.ok_or(Error::NotInitialized)?;
        self.wait_ready(info.relative_address).map(|_| ())
    }
}
impl<H: NativeHost, D: DelayNs> BlockStorage for SdCard<H, D> {
    type Error = Error<H::Error>;
    fn sector_count(&self) -> u64 {
        self.info.map_or(0, |info| info.sector_count)
    }
    fn read_sector(
        &mut self,
        sector: u64,
        data: &mut [u8; SECTOR_BYTES],
    ) -> Result<(), Self::Error> {
        SdCard::read_sector(self, sector, data)
    }
    fn write_sector(&mut self, sector: u64, data: &[u8; SECTOR_BYTES]) -> Result<(), Self::Error> {
        SdCard::write_sector(self, sector, data)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        SdCard::flush(self)
    }
}

impl<H: NativeHost, D: DelayNs> ReadSectors for SdCard<H, D> {
    fn read_sectors(&mut self, first_sector: u64, data: &mut [u8]) -> Result<(), Self::Error> {
        SdCard::read_sectors(self, first_sector, data)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum GeometryError {
    Unsupported,
    Invalid,
}
struct Geometry {
    sectors: u64,
    maximum_clock_hz: u32,
    read_only: bool,
}
impl Geometry {
    fn parse(words: [u32; 4], block_addressed: bool) -> Result<Self, GeometryError> {
        let raw = u128::from(words[0])
            | u128::from(words[1]) << 32
            | u128::from(words[2]) << 64
            | u128::from(words[3]) << 96;
        let structure = (raw >> 126) as u8;
        let read_length = ((raw >> 80) & 15) as u8;
        let write_length = ((raw >> 22) & 15) as u8;
        let classes = ((raw >> 84) & 0xfff) as u16;
        if classes & 4 == 0 || !(9..=11).contains(&read_length) {
            return Err(GeometryError::Invalid);
        }
        let sectors = match (structure, block_addressed) {
            (0, false) => {
                let size = ((raw >> 62) & 0xfff) as u64 + 1;
                let multiplier = ((raw >> 47) & 7) as u8 + 2;
                size << (multiplier + read_length - 9)
            }
            (1, true) if read_length == 9 => (((raw >> 48) & 0x3f_ffff) as u64 + 1) * 1024,
            _ => return Err(GeometryError::Unsupported),
        };
        if sectors == 0 || (!block_addressed && sectors > (1_u64 << 31) / 512) {
            return Err(GeometryError::Invalid);
        }
        let speed = ((raw >> 96) & 0xff) as u8;
        let units = [100_000_u32, 1_000_000, 10_000_000, 100_000_000];
        let values = [
            0_u32, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
        ];
        if speed & 0x80 != 0 || speed & 7 > 3 || values[usize::from((speed >> 3) & 15)] == 0 {
            return Err(GeometryError::Invalid);
        }
        let maximum_clock_hz =
            (units[usize::from(speed & 7)] / 10) * values[usize::from((speed >> 3) & 15)];
        if maximum_clock_hz < INITIAL_CLOCK_HZ {
            return Err(GeometryError::Invalid);
        }
        let read_only =
            raw & (3 << 12) != 0 || classes & (1 << 4) == 0 || !(9..=11).contains(&write_length);
        Ok(Self {
            sectors,
            maximum_clock_hz: maximum_clock_hz.min(MAX_DEFAULT_CLOCK_HZ),
            read_only,
        })
    }
}

#[cfg(test)]
mod tests;
