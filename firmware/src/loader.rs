// framed usb commands and verified application uploads

use crate::{flash::Flash, usb};
use core::future::Future;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb::driver::EndpointError;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const APP_START: u32 = 0x90000;
const APP_SIZE: u32 = 0xf70000;
const BLOCK_SIZE: usize = 256;
const FRAME_SIZE: usize = BLOCK_SIZE + 8;
const STATE: u32 = 0x9000;
const RECOVERY_START: u32 = 0x10000;
const SECTOR: u32 = 4096;

#[derive(Deserialize)]
enum Command<'a> {
    Recover,
    Upload { size: u32, sha256: [u8; 32] },
    Data(#[serde(borrow)] &'a [u8]),
}

#[derive(Serialize)]
enum Response {
    Ready,
    Reboot,
    Error,
}

fn decode(frame: &mut [u8]) -> Result<Command<'_>, ()> {
    if frame.len() > FRAME_SIZE || frame.last() != Some(&0) {
        return Err(());
    }

    let length = cobs::decode_in_place(frame).map_err(|_| ())?;
    let (command, rest) = postcard::take_from_bytes(&frame[..length]).map_err(|_| ())?;
    if !rest.is_empty() {
        return Err(());
    }

    match command {
        Command::Recover => Ok(command),
        Command::Upload { size, .. } if (288..=APP_SIZE).contains(&size) => Ok(command),
        Command::Data(data) if !data.is_empty() && data.len() <= BLOCK_SIZE => Ok(command),
        _ => Err(()),
    }
}

// these values match the boot records used by the c bootloader
#[repr(u32)]
enum BootState {
    Recovery = 0x31434552,
    StartApp = 0x314e5552,
}

#[repr(align(4))]
struct Buffer<const N: usize>([u8; N]);

// keep bytes left over when usb packets split or combine command frames
struct Reader {
    receiver: usb::Input,
    packet: [u8; usb::PACKET_SIZE],
    start: usize,
    end: usize,
}

impl Reader {
    async fn byte(&mut self) -> Result<u8, EndpointError> {
        while self.start == self.end {
            self.end = self.receiver.read_packet(&mut self.packet).await?;
            self.start = 0;
        }

        let byte = self.packet[self.start];
        self.start += 1;
        Ok(byte)
    }

    async fn command<'a>(&mut self, frame: &'a mut [u8; FRAME_SIZE]) -> Result<Command<'a>, ()> {
        // wait for the first byte before starting the timeout
        frame[0] = self.byte().await.map_err(|_| ())?;

        let read_frame = async {
            let mut length = 1;
            while frame[length - 1] != 0 && length < frame.len() {
                frame[length] = self.byte().await?;
                length += 1;
            }
            Ok::<_, EndpointError>(length)
        };

        let length = io(read_frame).await.ok_or(())?;
        decode(&mut frame[..length])
    }
}

fn validate_layout(flash: &mut Flash<'_>, recovery: bool) -> Option<()> {
    if flash.capacity() != (APP_START + APP_SIZE) as usize || esp_storage::flash_encryption() {
        return None;
    }

    let mut bytes = Buffer([0; 128]);
    flash.read_nor(0x8000, &mut bytes.0).ok()?;
    if !valid_partition_table(&bytes.0) {
        return None;
    }

    let expected_start = if recovery { RECOVERY_START } else { APP_START };
    if flash.mapped_start() != expected_start {
        return None;
    }
    Some(())
}

// check the partition records here because the sdk flash size probe is unreliable
fn valid_partition_table(bytes: &[u8; 128]) -> bool {
    use md5::{Digest, Md5};

    let expected = [
        (STATE, SECTOR, 1, 6),
        (RECOVERY_START, APP_START - RECOVERY_START, 0, 0),
        (APP_START, APP_SIZE, 0, 0x10),
    ];
    let entries = bytes[..96].as_chunks::<32>().0;

    for (entry, (offset, size, kind, subtype)) in entries.iter().zip(expected) {
        if entry[..4] != [0xaa, 0x50, kind, subtype] {
            return false;
        }
        if entry[4..8] != offset.to_le_bytes() || entry[8..12] != size.to_le_bytes() {
            return false;
        }
        if entry[28..32] != [0; 4] {
            return false;
        }
    }

    if bytes[96..98] != [0xeb, 0xeb] {
        return false;
    }
    if bytes[98..112].iter().any(|&byte| byte != 0xff) {
        return false;
    }

    let checksum = Md5::digest(&bytes[..96]);
    bytes[112..128] == checksum[..]
}

async fn io<T, E>(future: impl Future<Output = Result<T, E>>) -> Option<T> {
    let result = with_timeout(Duration::from_secs(30), future).await.ok()?;
    result.ok()
}

async fn respond(tx: &mut usb::Output, response: Response) -> Option<()> {
    let mut buffer = [0; 8];
    let frame = postcard::to_slice_cobs(&response, &mut buffer).ok()?;
    io(tx.write_packet(frame)).await
}

fn boot(flash: &mut Flash<'_>, state: BootState, size: u32) -> Option<()> {
    let mut record = Buffer([0; 8]);
    record.0[..4].copy_from_slice(&(state as u32).to_le_bytes());
    record.0[4..].copy_from_slice(&size.to_le_bytes());

    let mut check = Buffer([0; 8]);
    flash.read_nor(STATE, &mut check.0).ok()?;
    if record.0 == check.0 {
        return Some(());
    }

    flash.erase(STATE, STATE + SECTOR).ok()?;

    // write the marker last so an interrupted write leaves recovery selected
    flash.write_nor(STATE + 4, &record.0[4..]).ok()?;
    flash.write_nor(STATE, &record.0[..4]).ok()?;
    flash.read_nor(STATE, &mut check.0).ok()?;

    if record.0 != check.0 {
        return None;
    }
    Some(())
}

pub async fn run(serial: usb::Serial, mut flash: Flash<'static>, recovery: bool) {
    let (mut tx, receiver, control) = serial.split_with_control();
    let mut rx = Reader {
        receiver,
        packet: [0; usb::PACKET_SIZE],
        start: 0,
        end: 0,
    };

    loop {
        rx.receiver.wait_connection().await;
        while !control.dtr() {
            control.control_changed().await;
        }

        let closed = async {
            while control.dtr() {
                control.control_changed().await;
            }
        };
        let pending_request = request(&mut rx, &mut tx, &mut flash, recovery);
        let result = select(closed, pending_request).await;

        match result {
            Either::Second(Some(Response::Reboot)) => {
                // a committed reboot still needs to happen if the host misses the reply
                let reply = respond(&mut tx, Response::Reboot);
                let _ = with_timeout(Duration::from_secs(1), reply).await;
                Timer::after_millis(100).await;
                return;
            }
            Either::Second(Some(Response::Ready)) => {
                if respond(&mut tx, Response::Ready).await.is_some() {
                    continue;
                }
            }
            Either::Second(_) => {
                let _ = respond(&mut tx, Response::Error).await;
            }
            Either::First(()) => {}
        }

        rx.start = 0;
        rx.end = 0;
        while control.dtr() {
            control.control_changed().await;
        }
    }
}

async fn request(
    rx: &mut Reader,
    tx: &mut usb::Output,
    flash: &mut Flash<'_>,
    recovery: bool,
) -> Option<Response> {
    let mut frame = [0; FRAME_SIZE];
    let command = rx.command(&mut frame).await.ok()?;
    validate_layout(flash, recovery)?;

    let (size, expected_hash) = match command {
        Command::Recover => {
            if recovery {
                return Some(Response::Ready);
            }

            boot(flash, BootState::Recovery, 0)?;
            return Some(Response::Reboot);
        }
        Command::Upload { size, sha256 } => (size, sha256),
        Command::Data(_) => return None,
    };

    boot(flash, BootState::Recovery, 0)?;
    if !recovery {
        return Some(Response::Reboot);
    }

    respond(tx, Response::Ready).await?;
    let mut hash = Sha256::new();
    let mut buffer = Buffer([0xff; BLOCK_SIZE]);

    for offset in (0..size).step_by(BLOCK_SIZE) {
        let length = (size - offset).min(BLOCK_SIZE as u32) as usize;
        let command = io(rx.command(&mut frame)).await?;
        let Command::Data(data) = command else {
            return None;
        };
        if data.len() != length {
            return None;
        }

        buffer.0.fill(0xff);
        buffer.0[..length].copy_from_slice(data);

        if offset == 0 {
            let valid_image = buffer.0[0] == 0xe9;
            let valid_chip = buffer.0[12..14] == [32, 0];
            if !valid_image || !valid_chip {
                return None;
            }
        }

        let address = APP_START + offset;
        if offset % SECTOR == 0 {
            flash.erase(address, address + SECTOR).ok()?;
        }

        let write_length = length.next_multiple_of(4);
        let block = &mut buffer.0[..write_length];
        flash.write_nor(address, block).ok()?;
        flash.read_nor(address, block).ok()?;
        hash.update(&block[..length]);

        let next_offset = offset + length as u32;
        if next_offset < size {
            respond(tx, Response::Ready).await?;
        }
        embassy_futures::yield_now().await;
    }

    let actual_hash = hash.finalize();
    if actual_hash[..] != expected_hash {
        return None;
    }

    boot(flash, BootState::StartApp, size)?;
    Some(Response::Reboot)
}
