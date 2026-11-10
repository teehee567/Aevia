use super::update::{Error, Flash, OTA_DATA, SECTOR};

pub const MODE_DATA: u32 = 0xb000;
const APPLICATION: [u8; 4] = *b"APP1";
const START_APPLICATION: [u8; 4] = *b"RUN1";
const RECOVERY: [u8; 4] = *b"REC1";
const TRIAL: [u8; 4] = *b"TRY1";
const OTA_NEW: u32 = 0;
const OTA_PENDING: u32 = 1;
const OTA_VALID: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    App,
    Recovery,
}

pub fn startup(flash: &mut impl Flash, fault: bool, fell_back: bool) -> Result<Mode, Error> {
    startup_with_marker(flash, fault, fell_back, APPLICATION)
}

pub(super) fn startup_trial(
    flash: &mut impl Flash,
    fault: bool,
    fell_back: bool,
) -> Result<Mode, Error> {
    startup_with_marker(flash, fault, fell_back, TRIAL)
}

pub(super) fn trial_started(flash: &mut impl Flash) -> Result<bool, Error> {
    let mut marker = [0; 4];
    flash.read(MODE_DATA, &mut marker)?;
    Ok(marker == TRIAL)
}

pub(super) fn confirm_app(flash: &mut impl Flash) -> Result<(), Error> {
    set_mode(flash, APPLICATION)
}

fn startup_with_marker(
    flash: &mut impl Flash,
    fault: bool,
    fell_back: bool,
    application: [u8; 4],
) -> Result<Mode, Error> {
    let mut marker = [0; 4];
    flash.read(MODE_DATA, &mut marker)?;
    if !fell_back && (marker == START_APPLICATION || (marker == APPLICATION && !fault)) {
        set_mode(flash, application)?;
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
    running: Option<Selector>,
    fallback_seed: Option<Selector>,
    has_fallback: bool,
}

#[derive(Clone, Copy)]
struct Selector {
    metadata: u32,
    sequence: u32,
    state: u32,
}

impl Selector {
    fn trial(self) -> bool {
        matches!(self.state, OTA_NEW | OTA_PENDING)
    }
    fn healthy(self) -> bool {
        self.state == OTA_VALID || self.state == u32::MAX
    }
}

impl Plan {
    pub fn read(flash: &mut impl Flash, running: u8) -> Result<Self, Error> {
        if running > 1 {
            return Err(Error::Layout);
        }
        let mut selectors = [None; 2];
        for (index, selector) in selectors.iter_mut().enumerate() {
            let mut entry = [0; 32];
            flash.read(OTA_DATA + index as u32 * SECTOR, &mut entry)?;
            let value = u32::from_le_bytes(entry[..4].try_into().unwrap());
            let state = u32::from_le_bytes(entry[24..28].try_into().unwrap());
            let crc = u32::from_le_bytes(entry[28..32].try_into().unwrap());
            if value != 0
                && value != u32::MAX
                && matches!(state, OTA_NEW | OTA_PENDING | OTA_VALID | u32::MAX)
                && crc == sequence_crc(value)
            {
                *selector = Some(Selector {
                    metadata: OTA_DATA + index as u32 * SECTOR,
                    sequence: value,
                    state,
                });
            }
        }
        let latest = selectors
            .iter()
            .flatten()
            .map(|entry| entry.sequence)
            .max()
            .unwrap_or(0);
        let selected = if latest == 0 { 0 } else { (latest - 1) % 2 };
        // Preserve the selector for the running image, even if ESP-IDF fell
        // back from a structurally invalid update in the other slot.
        let preserve = selectors
            .iter()
            .enumerate()
            .filter_map(|(index, value)| {
                value
                    .filter(|entry| (entry.sequence - 1) % 2 == running as u32)
                    .map(|entry| (index, entry))
            })
            .max_by_key(|(_, entry)| entry.sequence);
        let target = 1 - running;
        let mut sequence = latest.checked_add(1).ok_or(Error::Sequence)?;
        if (sequence - 1) % 2 != target as u32 {
            sequence = sequence.checked_add(1).ok_or(Error::Sequence)?;
        }
        // A factory/manual installation can boot slot0 with erased otadata.
        // Seed a durable known-running selector before publishing its first OTA.
        let fallback_seed = if preserve.is_none() {
            let mut seed = latest.checked_add(1).ok_or(Error::Sequence)?;
            if (seed - 1) % 2 != u32::from(running) {
                seed = seed.checked_add(1).ok_or(Error::Sequence)?;
            }
            sequence = seed.checked_add(1).ok_or(Error::Sequence)?;
            Some(Selector {
                metadata: OTA_DATA + SECTOR,
                sequence: seed,
                state: OTA_VALID,
            })
        } else {
            None
        };
        if sequence == u32::MAX {
            return Err(Error::Sequence);
        }
        Ok(Self {
            target,
            sequence,
            metadata: OTA_DATA + (preserve.map_or(0, |(index, _)| 1 - index) as u32) * SECTOR,
            fell_back: selected != running as u32,
            running: preserve.map(|(_, entry)| entry),
            fallback_seed,
            has_fallback: selectors
                .iter()
                .flatten()
                .any(|entry| (entry.sequence - 1) % 2 != u32::from(running) && entry.healthy()),
        })
    }

    pub fn publish(&self, flash: &mut impl Flash) -> Result<(), Error> {
        if let Some(seed) = self.fallback_seed {
            publish_selector(flash, seed)?;
        }
        publish_selector(
            flash,
            Selector {
                metadata: self.metadata,
                sequence: self.sequence,
                state: OTA_NEW,
            },
        )
    }

    pub fn is_trial(&self) -> bool {
        !self.fell_back && self.running.is_some_and(Selector::trial)
    }

    pub fn protects_fallback(&self) -> bool {
        self.is_trial() && self.has_fallback
    }

    /// A selector with a zero sequence is invalid in ESP-IDF. Invalidate only
    /// the trial record; retain the previous healthy sector and both images.
    /// This app-level fallback requires startup to run. A rollback-enabled
    /// second-stage bootloader is required for faults before Rust startup.
    pub fn rollback(&self, flash: &mut impl Flash) -> Result<bool, Error> {
        if !self.is_trial() || !self.has_fallback {
            return Ok(false);
        }
        let running = self.running.ok_or(Error::Sequence)?;
        flash.write(running.metadata, &0_u32.to_le_bytes())?;
        let mut value = [0; 4];
        flash.read(running.metadata, &mut value)?;
        if value != [0; 4] {
            return Err(Error::Flash);
        }
        Ok(true)
    }

    pub fn confirm(&mut self, flash: &mut impl Flash) -> Result<(), Error> {
        if !self.is_trial() {
            return Ok(());
        }
        let mut running = self.running.ok_or(Error::Sequence)?;
        running.state = OTA_VALID;
        // Erasing/replacing the trial's metadata leaves its healthy fallback
        // intact. Sequence is committed last, as during the original upload.
        publish_selector(flash, running)?;
        self.running = Some(running);
        Ok(())
    }
}

fn publish_selector(flash: &mut impl Flash, selector: Selector) -> Result<(), Error> {
    let mut entry = [0xff; 32];
    entry[24..28].copy_from_slice(&selector.state.to_le_bytes());
    entry[28..32].copy_from_slice(&sequence_crc(selector.sequence).to_le_bytes());
    flash.erase(selector.metadata, selector.metadata + SECTOR)?;
    flash.write(selector.metadata + 4, &entry[4..])?;
    let mut readback = [0; 32];
    flash.read(selector.metadata, &mut readback)?;
    if readback != entry {
        return Err(Error::Flash);
    }
    // Commit last. Until this word is complete, the old slot stays selected.
    flash.write(selector.metadata, &selector.sequence.to_le_bytes())?;
    entry[..4].copy_from_slice(&selector.sequence.to_le_bytes());
    flash.read(selector.metadata, &mut readback)?;
    if readback != entry {
        return Err(Error::Flash);
    }
    Ok(())
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
