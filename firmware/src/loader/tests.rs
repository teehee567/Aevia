use super::{
    recovery,
    state::{self, MODE_DATA, Mode},
    update::*,
};
use crate::{commands::Command, protocol::Frame};
use sha2::{Digest, Sha256};
use std::{cell::RefCell, rc::Rc, vec, vec::Vec};

#[test]
fn protocol_discovery_is_read_only_in_both_modes_and_during_upload() {
    use crate::commands::Response;
    let memory = Memory::new(0);
    let mut updater = Updater::new(memory.clone(), 0).unwrap();
    let bytes = image(512);
    updater
        .begin(bytes.len() as u32, Sha256::digest(&bytes).into())
        .unwrap();
    let before = memory.0.borrow().mutations;
    assert_eq!(
        recovery::command(&mut updater, Command::ProtocolInfo),
        Ok(Response::ProtocolInfo(crate::protocol::info(true)))
    );
    assert_eq!(
        crate::device::command(&mut updater, Command::ProtocolInfo),
        Ok(Response::ProtocolInfo(crate::protocol::info(false)))
    );
    assert_eq!(memory.0.borrow().mutations, before);
    // Discovery must not abort the active transfer or change its sequence.
    updater.data(0, &bytes[..256]).unwrap();
    updater.data(256, &bytes[256..]).unwrap();
    updater.commit().unwrap();
}

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
            let state = u32::from_le_bytes(entry[24..28].try_into().unwrap());
            if seq != 0
                && seq != u32::MAX
                && state != 3
                && state != 4
                && crc == state::sequence_crc(seq)
            {
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
    assert_eq!(
        recovery::command(&mut updater, Command::Recover),
        Err(Error::Command)
    );
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
    next.confirm_app().unwrap();
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
fn binary_protocol_handles_fragmentation_and_discards_overflow() {
    let flash = Memory::new(0);
    let mut updater = Updater::new(flash.clone(), 0).unwrap();
    let bytes = image(256);
    let mut encoded = Vec::new();
    let mut buffer = [0; 272];
    for command in [
        Command::Info,
        Command::Begin {
            size: 256,
            hash: Sha256::digest(&bytes).into(),
        },
        Command::Data {
            offset: 0,
            bytes: &bytes,
        },
        Command::Commit,
    ] {
        encoded.extend_from_slice(postcard::to_slice_cobs(&command, &mut buffer).unwrap());
    }
    let mut frame = Frame::<272>::new();
    let mut replies = 0;
    for packet in encoded.chunks(13) {
        let mut input = packet;
        while !input.is_empty() {
            if let Some(command) = frame.feed::<Command<'_>>(&mut input) {
                recovery::command(&mut updater, command.unwrap()).unwrap();
                replies += 1;
            }
        }
    }
    assert_eq!(replies, 4);
    assert_eq!(flash.selected(), 1);

    // The tail of an oversized frame must never become a second command.
    let oversized_bytes = [1; 300];
    let mut oversized = oversized_bytes.as_slice();
    assert!(frame.feed::<Command<'_>>(&mut oversized).is_none());
    let mut terminator = [1, 0].as_slice();
    assert!(matches!(
        frame.feed::<Command<'_>>(&mut terminator),
        Some(Err(Error::Command))
    ));
    let valid_bytes = postcard::to_slice_cobs(&Command::Info, &mut buffer).unwrap();
    let mut valid: &[u8] = valid_bytes;
    assert!(matches!(
        frame.feed::<Command<'_>>(&mut valid),
        Some(Ok(Command::Info))
    ));

    updater.begin(256, [0; 32]).unwrap();
    assert_eq!(
        recovery::command(&mut updater, Command::Recover),
        Err(Error::Command)
    );
    assert!(!updater.active());
}

#[test]
fn ota_crc_matches_esp_idf_golden_records() {
    assert_eq!(state::sequence_crc(1), 0x4743_989a);
    assert_eq!(state::sequence_crc(2), 0x55f6_3774);
    assert_eq!(state::sequence_crc(3), 0xed4a_5011);
}

#[test]
fn unconfirmed_trial_rolls_back_and_confirmed_trial_survives_reboot() {
    for running in [0, 1] {
        for confirm in [false, true] {
            let flash = Memory::new(running);
            send(
                &mut Updater::new(flash.clone(), running).unwrap(),
                &image(256),
            )
            .unwrap();
            let candidate = 1 - running;
            assert_eq!(flash.selected(), candidate);
            flash.0.borrow_mut().running = candidate;
            let mut trial = Updater::new(flash.clone(), candidate).unwrap();
            assert_eq!(trial.startup(true).unwrap(), Mode::App);
            // An unconfirmed app must not erase its known-good fallback.
            assert_eq!(trial.begin(256, [0; 32]), Err(Error::Sequence));
            if confirm {
                trial.confirm_app().unwrap();
                let before = flash.0.borrow().mutations;
                trial.confirm_app().unwrap();
                assert_eq!(flash.0.borrow().mutations, before);
            }
            let mut reboot = Updater::new(flash.clone(), candidate).unwrap();
            assert_eq!(
                reboot.startup(false).unwrap(),
                if confirm { Mode::App } else { Mode::Recovery }
            );
            assert_eq!(reboot.rollback_requested(), !confirm);
            assert_eq!(flash.selected(), if confirm { candidate } else { running });
            if !confirm {
                flash.0.borrow_mut().running = running;
                assert_eq!(
                    Updater::new(flash.clone(), running)
                        .unwrap()
                        .startup(true)
                        .unwrap(),
                    Mode::App
                );
            }
        }
    }
}

#[test]
fn first_ota_seeds_an_erased_selector_for_the_known_running_slot() {
    for running in [0, 1] {
        let flash = Memory::new(running);
        flash.0.borrow_mut().bytes[OTA_DATA as usize..(OTA_DATA + 2 * SECTOR) as usize].fill(0xff);
        send(
            &mut Updater::new(flash.clone(), running).unwrap(),
            &image(256),
        )
        .unwrap();
        let candidate = 1 - running;
        flash.0.borrow_mut().running = candidate;
        let mut trial = Updater::new(flash.clone(), candidate).unwrap();
        assert_eq!(trial.startup(true).unwrap(), Mode::App);
        let mut reboot = Updater::new(flash.clone(), candidate).unwrap();
        assert_eq!(reboot.startup(true).unwrap(), Mode::Recovery);
        assert!(reboot.rollback_requested());
        assert_eq!(flash.selected(), running);
    }
}

#[test]
fn bootloader_pending_verify_state_still_requires_health_confirmation() {
    let flash = Memory::new(0);
    send(&mut Updater::new(flash.clone(), 0).unwrap(), &image(256)).unwrap();
    let address = (OTA_DATA + SECTOR + 24) as usize;
    assert_eq!(
        &flash.0.borrow().bytes[address..address + 4],
        &0_u32.to_le_bytes()
    );
    // A rollback-enabled second-stage loader rewrites NEW to PENDING_VERIFY.
    flash.0.borrow_mut().bytes[address..address + 4].copy_from_slice(&1_u32.to_le_bytes());
    flash.0.borrow_mut().running = 1;
    let mut candidate = Updater::new(flash.clone(), 1).unwrap();
    assert_eq!(candidate.startup(false).unwrap(), Mode::App);
    candidate.confirm_app().unwrap();
    assert_eq!(
        &flash.0.borrow().bytes[address..address + 4],
        &2_u32.to_le_bytes()
    );
}

#[test]
fn first_ota_power_cuts_never_select_an_incomplete_image() {
    let make_flash = || {
        let flash = Memory::new(0);
        flash.0.borrow_mut().bytes[OTA_DATA as usize..(OTA_DATA + 2 * SECTOR) as usize].fill(0xff);
        flash
    };
    let bytes = image(256);
    let reference = make_flash();
    send(&mut Updater::new(reference.clone(), 0).unwrap(), &bytes).unwrap();
    for cut in 0..=reference.0.borrow().mutations {
        let flash = make_flash();
        flash.0.borrow_mut().cut_at = cut;
        let _ = send(&mut Updater::new(flash.clone(), 0).unwrap(), &bytes);
        assert_eq!(flash.app_bytes(0, 256), vec![0x5a; 256]);
        if flash.selected() == 1 {
            assert_eq!(flash.app_bytes(1, bytes.len()), bytes);
        }
    }
}

#[test]
fn confirmation_power_cuts_preserve_the_previous_selector_and_both_images() {
    let source = Memory::new(0);
    send(&mut Updater::new(source.clone(), 0).unwrap(), &image(256)).unwrap();
    source.0.borrow_mut().running = 1;
    let mut trial = Updater::new(source.clone(), 1).unwrap();
    trial.startup(false).unwrap();
    let before = source.0.borrow().mutations;
    trial.confirm_app().unwrap();
    let mutations = source.0.borrow().mutations - before;
    for cut in 0..=mutations {
        let flash = Memory::new(0);
        send(&mut Updater::new(flash.clone(), 0).unwrap(), &image(256)).unwrap();
        flash.0.borrow_mut().running = 1;
        let mut trial = Updater::new(flash.clone(), 1).unwrap();
        trial.startup(false).unwrap();
        let old_selector =
            flash.0.borrow().bytes[OTA_DATA as usize..(OTA_DATA + 32) as usize].to_vec();
        let baseline = flash.0.borrow().mutations;
        flash.0.borrow_mut().cut_at = baseline + cut;
        let _ = trial.confirm_app();
        assert_eq!(
            &flash.0.borrow().bytes[OTA_DATA as usize..(OTA_DATA + 32) as usize],
            old_selector.as_slice()
        );
        assert_eq!(flash.app_bytes(0, 256), vec![0x5a; 256]);
        assert_eq!(flash.app_bytes(1, 256), image(256));
        assert!(flash.selected() <= 1);
    }
}

#[test]
fn startup_diagnostics_preserve_existing_wire_indices() {
    use crate::{
        commands::Response as SystemResponse,
        protocol::{Request, Response},
    };
    let mut buffer = [0; 32];
    assert_eq!(
        postcard::to_slice_cobs(&Request::System(Command::Run), &mut buffer).unwrap(),
        &[1, 2, 2, 0],
    );
    assert_eq!(
        postcard::to_slice_cobs(&Request::System(Command::BootStatus), &mut buffer).unwrap(),
        &[1, 2, 7, 0],
    );
    assert_eq!(
        postcard::to_slice_cobs(
            &Response::System(SystemResponse::BootStatus { stage: 4 }),
            &mut buffer
        )
        .unwrap(),
        &[1, 3, 4, 4, 0],
    );
    let flash = Memory::new(0);
    let mut updater = Updater::new(flash, 0).unwrap();
    assert_eq!(
        recovery::command(&mut updater, Command::BootStatus),
        Ok(SystemResponse::BootStatus { stage: 0 })
    );
}
