use super::{
    recovery::{self, Line, Reply},
    state::{self, MODE_DATA, Mode},
    update::*,
};
use crate::device;
use sha2::{Digest, Sha256};
use std::{cell::RefCell, format, rc::Rc, string::String, vec, vec::Vec};

#[derive(Clone)]
struct Memory(Rc<RefCell<Nor>>);
struct Nor {
    bytes: Vec<u8>,
    running: u8,
    mutations: usize,
    cut_at: usize,
    corrupt_read: bool,
}

// Keep only two sectors per app in the model; flash addresses remain real.
fn index(address: u32) -> usize {
    if address >= APP_START + APP_SIZE {
        (address - APP_SIZE + 2 * SECTOR) as usize
    } else {
        address as usize
    }
}

impl Memory {
    fn new(running: u8) -> Self {
        let mut bytes = vec![0xff; (APP_START + 4 * SECTOR) as usize];
        let start = index(APP_START + running as u32 * APP_SIZE);
        bytes[start..start + 2 * SECTOR as usize].fill(0x5a);
        bytes[MODE_DATA as usize..MODE_DATA as usize + 4].copy_from_slice(b"REC1");
        // Stock ESP-IDF metadata for the installed application.
        let sequence = running as u32 + 1;
        bytes[OTA_DATA as usize..OTA_DATA as usize + 4].copy_from_slice(&sequence.to_le_bytes());
        bytes[OTA_DATA as usize + 28..OTA_DATA as usize + 32]
            .copy_from_slice(&state::sequence_crc(sequence).to_le_bytes());
        Self(Rc::new(RefCell::new(Nor {
            bytes,
            running,
            mutations: 0,
            cut_at: usize::MAX,
            corrupt_read: false,
        })))
    }

    fn selected(&self) -> u8 {
        let flash = self.0.borrow();
        let mut latest = 0;
        for offset in [OTA_DATA, OTA_DATA + SECTOR] {
            let entry = &flash.bytes[offset as usize..offset as usize + 32];
            let seq = u32::from_le_bytes(entry[..4].try_into().unwrap());
            let crc = u32::from_le_bytes(entry[28..32].try_into().unwrap());
            if seq != u32::MAX && crc == state::sequence_crc(seq) {
                latest = latest.max(seq);
            }
        }
        if latest == 0 {
            0
        } else {
            ((latest - 1) % 2) as u8
        }
    }

    fn app_bytes(&self, slot: u8, size: usize) -> Vec<u8> {
        let start = index(APP_START + slot as u32 * APP_SIZE);
        self.0.borrow().bytes[start..start + size].to_vec()
    }
}

impl Nor {
    fn tick(&mut self) -> Result<(), Error> {
        if self.mutations == self.cut_at {
            return Err(Error::Flash);
        }
        self.mutations += 1;
        Ok(())
    }
    fn writable(&self, address: u32) -> bool {
        (OTA_DATA..MODE_DATA + SECTOR).contains(&address)
            || (APP_START + (1 - self.running) as u32 * APP_SIZE
                ..APP_START + (2 - self.running) as u32 * APP_SIZE)
                .contains(&address)
    }
}

impl Flash for Memory {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<(), Error> {
        let flash = self.0.borrow();
        bytes.copy_from_slice(&flash.bytes[index(address)..index(address) + bytes.len()]);
        if flash.corrupt_read && address >= APP_START {
            bytes[0] ^= 1;
        }
        Ok(())
    }
    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), Error> {
        assert!(address.is_multiple_of(4) && bytes.len().is_multiple_of(4));
        let mut flash = self.0.borrow_mut();
        assert!(flash.writable(address) && flash.writable(address + bytes.len() as u32 - 1));
        for (word_index, word) in bytes.as_chunks::<4>().0.iter().enumerate() {
            flash.tick()?;
            for (i, byte) in word.iter().enumerate() {
                flash.bytes[index(address) + word_index * 4 + i] &= byte;
            }
        }
        Ok(())
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        assert!(from.is_multiple_of(SECTOR) && to.is_multiple_of(SECTOR));
        let mut flash = self.0.borrow_mut();
        assert!(flash.writable(from) && flash.writable(to - 1));
        for address in (from..to).step_by(64) {
            flash.tick()?;
            flash.bytes[index(address)..index(address) + 64].fill(0xff);
        }
        Ok(())
    }
}

fn image(size: usize) -> Vec<u8> {
    let mut bytes = vec![0xa5; size];
    bytes[..24].fill(0);
    bytes[0] = 0xe9;
    bytes[1] = 1;
    bytes[12..14].copy_from_slice(&32u16.to_le_bytes());
    bytes[23] = 1;
    bytes[24..28].copy_from_slice(&0x2f00_0000u32.to_le_bytes());
    bytes[28..32].copy_from_slice(&(size as u32 - 80).to_le_bytes());
    bytes
}

fn send(updater: &mut Updater<Memory>, bytes: &[u8]) -> Result<(), Error> {
    updater.begin(bytes.len() as u32, Sha256::digest(bytes).into())?;
    for (chunk_index, chunk) in bytes.chunks(CHUNK).enumerate() {
        updater.data((chunk_index * CHUNK) as u32, chunk)?;
    }
    updater.commit()
}

#[test]
fn every_power_cut_preserves_the_running_image_in_both_slots() {
    let bytes = image(2 * SECTOR as usize - 16);
    for running in [0, 1] {
        let reference = Memory::new(running);
        send(
            &mut Updater::new(reference.clone(), running).unwrap(),
            &bytes,
        )
        .unwrap();
        let mutations = reference.0.borrow().mutations;
        for cut in 0..=mutations {
            let flash = Memory::new(running);
            flash.0.borrow_mut().cut_at = cut;
            let mut updater = Updater::new(flash.clone(), running).unwrap();
            let result = send(&mut updater, &bytes);
            assert!(
                flash
                    .app_bytes(running, 2 * SECTOR as usize)
                    .iter()
                    .all(|byte| *byte == 0x5a)
            );
            let selected = flash.selected();
            if selected != running {
                assert_eq!(flash.app_bytes(selected, bytes.len()), bytes);
            }
            flash.0.borrow_mut().cut_at = usize::MAX;
            let mut rebooted = Updater::new(flash.clone(), selected).unwrap();
            let mode = rebooted.startup(false).unwrap();
            assert_eq!(
                mode,
                if result.is_ok() {
                    Mode::App
                } else {
                    Mode::Recovery
                }
            );
        }
    }
}

#[test]
fn modes_recover_on_fault_and_persist_until_explicit_run() {
    let flash = Memory::new(0);
    let mut updater = Updater::new(flash.clone(), 0).unwrap();
    assert_eq!(updater.startup(false).unwrap(), Mode::Recovery);
    updater.request_app().unwrap();
    assert_eq!(updater.startup(true).unwrap(), Mode::App);
    assert_eq!(updater.startup(false).unwrap(), Mode::App);
    assert_eq!(updater.startup(true).unwrap(), Mode::Recovery);
    assert_eq!(updater.startup(false).unwrap(), Mode::Recovery);
    assert!(matches!(device::command(b"RECOVER"), Ok(Reply::Reboot)));
    assert!(device::command(b"COMMIT").is_err());
}

#[test]
fn consecutive_updates_alternate_slots_and_preserve_the_current_selector() {
    let flash = Memory::new(0);
    let bytes = image(512);
    send(&mut Updater::new(flash.clone(), 0).unwrap(), &bytes).unwrap();
    assert_eq!(flash.selected(), 1);
    flash.0.borrow_mut().running = 1;
    let mut next = Updater::new(flash.clone(), 1).unwrap();
    assert_eq!(next.startup(true).unwrap(), Mode::App);
    send(&mut next, &bytes).unwrap();
    assert_eq!(flash.selected(), 0);
    assert_eq!(flash.app_bytes(1, bytes.len()), bytes);
}

#[test]
fn invalid_candidate_fallback_enters_recovery_and_can_be_replaced() {
    let flash = Memory::new(0);
    send(&mut Updater::new(flash.clone(), 0).unwrap(), &image(256)).unwrap();
    // Simulate stock image validation falling back to the original slot.
    let mut fallback = Updater::new(flash.clone(), 0).unwrap();
    assert_eq!(fallback.startup(false).unwrap(), Mode::Recovery);
    send(&mut fallback, &image(512)).unwrap();
    assert_eq!(flash.selected(), 1);
}

#[test]
fn incomplete_invalid_or_corrupted_uploads_do_not_change_boot_selection() {
    let bytes = image(512);
    for failure in 0..4 {
        let flash = Memory::new(0);
        let mut updater = Updater::new(flash.clone(), 0).unwrap();
        updater.begin(512, [0; 32]).unwrap();
        if failure == 0 {
            flash.0.borrow_mut().corrupt_read = true;
        }
        let first = updater.data(if failure == 3 { 256 } else { 0 }, &bytes[..256]);
        if first.is_ok() && failure == 1 {
            updater.data(256, &bytes[256..]).unwrap();
        }
        assert!(updater.commit().is_err());
        assert_eq!(flash.selected(), 0);
        assert!(!updater.active());
    }
    let flash = Memory::new(0);
    let mut updater = Updater::new(flash.clone(), 0).unwrap();
    assert_eq!(send(&mut updater, &image(8192)[..4096]), Err(Error::Image));
    let mut wrong_chip = image(256);
    wrong_chip[12] = 9;
    assert_eq!(send(&mut updater, &wrong_chip), Err(Error::Image));
    for size in [0, 24, 255, APP_SIZE + 4, u32::MAX] {
        assert_eq!(updater.begin(size, [0; 32]), Err(Error::Size));
    }
}

#[test]
fn text_protocol_handles_fragmentation_and_discards_overflow() {
    let flash = Memory::new(0);
    let mut updater = Updater::new(flash.clone(), 0).unwrap();
    let bytes = image(256);
    let hex: String = bytes.iter().map(|byte| format!("{byte:02x}")).collect();
    let digest: String = Sha256::digest(&bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    let commands = format!("INFO\r\nBEGIN 256 {digest}\nDATA 0 {hex}\nCOMMIT\n");
    let mut line = Line::new();
    let mut replies = 0;
    for packet in commands.as_bytes().chunks(13) {
        for byte in packet {
            if let Some(input) = line.push(*byte) {
                recovery::command(&mut updater, input.unwrap()).unwrap();
                replies += 1;
            }
        }
    }
    assert_eq!(replies, 4);
    assert_eq!(flash.selected(), 1);
    for _ in 0..600 {
        assert!(line.push(b'x').is_none());
    }
    for byte in b"COMMIT" {
        assert!(line.push(*byte).is_none());
    }
    assert!(matches!(line.push(b'\n'), Some(Err(Error::Command))));
    for command in [
        b"DATA -1 00".as_slice(),
        b"DATA 0 xz",
        b"COMMIT extra",
        b"\xff",
    ] {
        updater.begin(256, [0; 32]).unwrap();
        assert!(recovery::command(&mut updater, command).is_err());
        assert!(!updater.active());
    }
}

#[test]
fn ota_crc_matches_esp_idf_golden_records() {
    assert_eq!(state::sequence_crc(1), 0x4743_989a);
    assert_eq!(state::sequence_crc(2), 0x55f6_3774);
    assert_eq!(state::sequence_crc(3), 0xed4a_5011);
}
