use super::state::{self, Mode, Plan};
use sha2::{Digest, Sha256};

// matches particion.csv
pub const OTA_DATA: u32 = 0x9000;
pub const APP_START: u32 = 0x10000;
pub const APP_SIZE: u32 = 0x400000;
pub const SECTOR: u32 = 4096;
pub const CHUNK: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Error {
    Command,
    Size,
    Sequence,
    Image,
    Hash,
    Flash,
    Layout,
}

/// NOR semantics: writes only clear bits; erase ranges are sector-aligned.
pub trait Flash {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<(), Error>;
    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), Error>;
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
}

struct Transfer {
    size: u32,
    received: u32,
    expected: [u8; 32],
    hash: Sha256,
}

pub struct Updater<F> {
    flash: F,
    transfer: Option<Transfer>,
    running: u8,
    plan: Plan,
}

impl<F: Flash> Updater<F> {
    pub fn new(mut flash: F, running: u8) -> Result<Self, Error> {
        let plan = Plan::read(&mut flash, running)?;
        Ok(Self {
            flash,
            transfer: None,
            running,
            plan,
        })
    }

    pub fn startup(&mut self, fault: bool) -> Result<Mode, Error> {
        state::startup(&mut self.flash, fault, self.plan.fell_back)
    }

    pub fn request_app(&mut self) -> Result<(), Error> {
        self.abort();
        state::request_app(&mut self.flash)
    }

    pub fn request_recovery(&mut self) -> Result<(), Error> {
        self.abort();
        state::recover(&mut self.flash)
    }

    pub fn abort(&mut self) {
        self.transfer = None;
    }

    pub fn active(&self) -> bool {
        self.transfer.is_some()
    }

    pub fn begin(&mut self, size: u32, expected: [u8; 32]) -> Result<(), Error> {
        self.abort();
        if !(80..=APP_SIZE).contains(&size) || !size.is_multiple_of(4) {
            return Err(Error::Size);
        }
        state::recover(&mut self.flash)?;
        self.plan = Plan::read(&mut self.flash, self.running)?;
        self.transfer = Some(Transfer {
            size,
            received: 0,
            expected,
            hash: Sha256::new(),
        });
        Ok(())
    }

    pub fn data(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        // Take the session: every error abandons it, including a failed write.
        let mut transfer = self.transfer.take().ok_or(Error::Sequence)?;
        let remaining = transfer.size - transfer.received;
        if offset != transfer.received
            || bytes.len() != (remaining as usize).min(CHUNK)
            || bytes.is_empty()
        {
            return Err(Error::Sequence);
        }
        if offset == 0 {
            // Standard ESP32-S31 app image, with the image hash enabled.
            if bytes[0] != 0xe9
                || !(1..=16).contains(&bytes[1])
                || u16::from_le_bytes([bytes[12], bytes[13]]) != 32
                || bytes[23] != 1
            {
                return Err(Error::Image);
            }
        }
        let address = self.target_start() + offset;
        if offset.is_multiple_of(SECTOR) {
            self.flash.erase(address, address + SECTOR)?;
        }
        self.flash.write(address, bytes)?;
        let mut readback = [0; CHUNK];
        self.flash.read(address, &mut readback[..bytes.len()])?;
        if &readback[..bytes.len()] != bytes {
            return Err(Error::Flash);
        }
        transfer.hash.update(&readback[..bytes.len()]);
        transfer.received += bytes.len() as u32;
        self.transfer = Some(transfer);
        Ok(())
    }

    /// Publish the verified inactive slot, request app mode, then reboot.
    pub fn commit(&mut self) -> Result<(), Error> {
        let transfer = self.transfer.take().ok_or(Error::Sequence)?;
        if transfer.received != transfer.size {
            return Err(Error::Sequence);
        }
        if transfer.hash.finalize().as_slice() != transfer.expected {
            return Err(Error::Hash);
        }
        self.check_image_length(transfer.size)?;

        self.plan.publish(&mut self.flash)?;
        state::request_app(&mut self.flash)
    }

    fn target_start(&self) -> u32 {
        APP_START + self.plan.target as u32 * APP_SIZE
    }

    fn check_image_length(&mut self, size: u32) -> Result<(), Error> {
        let mut header = [0; 24];
        let start = self.target_start();
        self.flash.read(start, &mut header)?;
        let mut offset = 24u32;
        for _ in 0..header[1] {
            let data_start = offset
                .checked_add(8)
                .filter(|end| *end <= size)
                .ok_or(Error::Image)?;
            let mut segment = [0; 8];
            self.flash.read(start + offset, &mut segment)?;
            let length = u32::from_le_bytes(segment[4..8].try_into().unwrap());
            if !length.is_multiple_of(4) {
                return Err(Error::Image);
            }
            offset = data_start
                .checked_add(length)
                .filter(|end| *end <= size)
                .ok_or(Error::Image)?;
        }
        // Segment data, zero padding + checksum to 16 bytes, then SHA-256.
        // Require an exact end: stale bytes beyond this upload must never
        // complete a truncated image. ESP-IDF verifies the actual contents.
        let image_end = ((offset + 16) & !15) + 32;
        if image_end != size {
            return Err(Error::Image);
        }
        Ok(())
    }
}
