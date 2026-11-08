use super::update::{Error, Flash, OTA_DATA, SECTOR};

pub const MODE_DATA: u32 = 0xb000;
const APPLICATION: [u8; 4] = *b"APP1";
const START_APPLICATION: [u8; 4] = *b"RUN1";
const RECOVERY: [u8; 4] = *b"REC1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    App,
    Recovery,
}

pub fn startup(flash: &mut impl Flash, fault: bool, fell_back: bool) -> Result<Mode, Error> {
    let mut marker = [0; 4];
    flash.read(MODE_DATA, &mut marker)?;
    if !fell_back && (marker == START_APPLICATION || (marker == APPLICATION && !fault)) {
        set_mode(flash, APPLICATION)?;
        Ok(Mode::App)
    } else {
        recover(flash)?;
        Ok(Mode::Recovery)
    }
}

pub fn recover(flash: &mut impl Flash) -> Result<(), Error> {
    set_mode(flash, RECOVERY)
}

pub fn request_app(flash: &mut impl Flash) -> Result<(), Error> {
    set_mode(flash, START_APPLICATION)
}

fn set_mode(flash: &mut impl Flash, marker: [u8; 4]) -> Result<(), Error> {
    let mut previous = [0; 4];
    flash.read(MODE_DATA, &mut previous)?;
    if previous != marker {
        flash.erase(MODE_DATA, MODE_DATA + SECTOR)?;
        flash.write(MODE_DATA, &marker)?;
        flash.read(MODE_DATA, &mut previous)?;
        if previous != marker {
            return Err(Error::Flash);
        }
    }
    Ok(())
}

pub struct Plan {
    pub target: u8,
    pub sequence: u32,
    pub metadata: u32,
    pub fell_back: bool,
}

impl Plan {
    pub fn read(flash: &mut impl Flash, running: u8) -> Result<Self, Error> {
        if running > 1 {
            return Err(Error::Layout);
        }
        let mut sequences = [None; 2];
        for (index, sequence) in sequences.iter_mut().enumerate() {
            let mut entry = [0; 32];
            flash.read(OTA_DATA + index as u32 * SECTOR, &mut entry)?;
            let value = u32::from_le_bytes(entry[..4].try_into().unwrap());
            let state = u32::from_le_bytes(entry[24..28].try_into().unwrap());
            let crc = u32::from_le_bytes(entry[28..32].try_into().unwrap());
            if value != 0
                && value != u32::MAX
                && state != 3
                && state != 4
                && crc == sequence_crc(value)
            {
                *sequence = Some(value);
            }
        }
        let latest = sequences.iter().flatten().copied().max().unwrap_or(0);
        let selected = if latest == 0 { 0 } else { (latest - 1) % 2 };
        // Preserve the selector for the running image, even if ESP-IDF fell
        // back from a structurally invalid update in the other slot.
        let preserve = sequences
            .iter()
            .enumerate()
            .filter_map(|(index, value)| {
                value
                    .filter(|seq| (seq - 1) % 2 == running as u32)
                    .map(|seq| (index, seq))
            })
            .max_by_key(|(_, seq)| *seq)
            .map(|(index, _)| index);
        let target = 1 - running;
        let mut sequence = latest.checked_add(1).ok_or(Error::Sequence)?;
        if (sequence - 1) % 2 != target as u32 {
            sequence = sequence.checked_add(1).ok_or(Error::Sequence)?;
        }
        if sequence == u32::MAX {
            return Err(Error::Sequence);
        }
        Ok(Self {
            target,
            sequence,
            metadata: OTA_DATA + (preserve.map_or(0, |index| 1 - index) as u32) * SECTOR,
            fell_back: selected != running as u32,
        })
    }

    pub fn publish(&self, flash: &mut impl Flash) -> Result<(), Error> {
        let mut entry = [0xff; 32];
        // VALID is understood by the stock ESP-IDF bootloader. Application
        // faults are handled by Rust startup, not custom trial-boot code.
        entry[24..28].copy_from_slice(&2u32.to_le_bytes());
        entry[28..32].copy_from_slice(&sequence_crc(self.sequence).to_le_bytes());
        flash.erase(self.metadata, self.metadata + SECTOR)?;
        flash.write(self.metadata + 4, &entry[4..])?;
        let mut readback = [0; 32];
        flash.read(self.metadata, &mut readback)?;
        if readback != entry {
            return Err(Error::Flash);
        }
        // Commit last. Until this word is complete, the old slot stays selected.
        flash.write(self.metadata, &self.sequence.to_le_bytes())?;
        entry[..4].copy_from_slice(&self.sequence.to_le_bytes());
        flash.read(self.metadata, &mut readback)?;
        if readback != entry {
            return Err(Error::Flash);
        }
        Ok(())
    }
}

// ESP-IDF esp_rom_crc32_le(UINT32_MAX, &sequence, 4).
pub fn sequence_crc(sequence: u32) -> u32 {
    let mut crc = 0u32;
    for byte in sequence.to_le_bytes() {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}
