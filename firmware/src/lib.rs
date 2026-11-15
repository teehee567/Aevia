#![no_std]

mod app;
mod usb;

use core::future::Future;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Read;
use esp_hal::{
    peripherals::TIMG0,
    timer::timg::{MwdtStage, MwdtStageAction, TimerGroup, Wdt},
};
use esp_storage::FlashStorage;
use sha2::{Digest, Sha256};
use static_cell::StaticCell;

const STATE: u32 = 0x9000;
const APP_START: u32 = 0x90000;
const APP_SIZE: u32 = 0xf70000;
static USB: StaticCell<usb::Buffers> = StaticCell::new();
static RX: StaticCell<[u8; 512]> = StaticCell::new();

#[repr(align(4))]
struct Buffer<const N: usize>([u8; N]);

pub async fn run(spawner: Spawner, recovery: bool) -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 8176);
    let timers = TimerGroup::new(p.TIMG0);
    let mut watchdog = timers.wdt;
    watchdog.set_timeout(MwdtStage::Stage0, esp_hal::time::Duration::from_secs(10));
    watchdog.set_stage_action(MwdtStage::Stage0, MwdtStageAction::ResetSystem);
    watchdog.enable();
    esp_rtos::start(timers.timer0, p.FROM_CPU_INTR0);
    let mut flash = FlashStorage::new(p.FLASH).multicore_auto_park();
    // A recovery binary uploaded into the app slot must never overwrite itself.
    if recovery {
        let mut bytes = [0; 128];
        let table =
            esp_bootloader_esp_idf::partitions::read_partition_table(&mut flash, &mut bytes)
                .unwrap();
        assert_eq!(table.booted_partition().unwrap().unwrap().offset(), 0x10000);
    }
    let (mut device, serial) = usb::init(p.USB_HS, USB.init_with(usb::Buffers::default));
    spawner.spawn(feed(watchdog).unwrap());
    spawner.spawn(serve(serial, flash, recovery).unwrap());
    if !recovery {
        spawner.spawn(app::run().unwrap());
    }
    device.run().await
}

#[embassy_executor::task]
async fn feed(mut watchdog: Wdt<TIMG0<'static>>) {
    loop {
        Timer::after_secs(1).await;
        watchdog.feed();
    }
}

async fn io<T, E>(future: impl Future<Output = Result<T, E>>) -> Option<T> {
    with_timeout(Duration::from_secs(30), future)
        .await
        .ok()?
        .ok()
}

fn boot(flash: &mut FlashStorage<'_>, marker: [u8; 4], size: u32) -> Option<()> {
    let mut record = Buffer([0; 8]);
    record.0[..4].copy_from_slice(&marker);
    record.0[4..].copy_from_slice(&size.to_le_bytes());
    let mut check = Buffer([0; 8]);
    flash.read_nor(STATE, &mut check.0).ok()?;
    if record.0 == check.0 {
        return Some(());
    }
    flash.erase(STATE, STATE + 4096).ok()?;
    // Commit the marker last; interrupted writes remain in recovery.
    flash.write_nor(STATE + 4, &record.0[4..]).ok()?;
    flash.write_nor(STATE, &record.0[..4]).ok()?;
    flash.read_nor(STATE, &mut check.0).ok()?;
    (record.0 == check.0).then_some(())
}

#[embassy_executor::task]
async fn serve(serial: usb::Serial, mut flash: FlashStorage<'static>, recovery: bool) {
    let (mut tx, rx, control) = serial.split_with_control();
    let mut rx = rx.into_buffered(RX.init([0; 512]));
    if recovery {
        let _ = boot(&mut flash, *b"REC1", 0);
    }
    while !control.dtr() {
        control.control_changed().await;
    }
    let closed = async {
        loop {
            control.control_changed().await;
            if !control.dtr() {
                break;
            }
        }
    };
    if matches!(
        select(closed, upload(&mut rx, &mut tx, &mut flash, recovery)).await,
        Either::Second(None)
    ) {
        let _ = io(tx.write_packet(&[2])).await;
    }
    Timer::after_millis(100).await;
    esp_hal::system::software_reset();
}

async fn upload(
    rx: &mut usb::Input,
    tx: &mut usb::Output,
    flash: &mut FlashStorage<'_>,
    recovery: bool,
) -> Option<()> {
    let mut header = [0; 40];
    io(rx.read_exact(&mut header)).await?;
    let size = u32::from_le_bytes(header[4..8].try_into().unwrap());
    if &header[..4] != b"FW04" || !(288..=APP_SIZE).contains(&size) {
        return None;
    }
    boot(flash, *b"REC1", 0)?;
    if !recovery {
        return io(tx.write_packet(&[1])).await;
    }
    io(tx.write_packet(&[0])).await?;
    let mut hash = Sha256::new();
    let mut buffer = Buffer([0xff; 256]);
    for offset in (0..size).step_by(256) {
        let n = (size - offset).min(256) as usize;
        buffer.0.fill(0xff);
        io(rx.read_exact(&mut buffer.0[..n])).await?;
        if offset == 0 && (buffer.0[0] != 0xe9 || buffer.0[12..14] != [32, 0]) {
            return None;
        }
        let address = APP_START + offset;
        if offset % 4096 == 0 {
            flash.erase(address, address + 4096).ok()?;
        }
        let written = n.next_multiple_of(4);
        flash.write_nor(address, &buffer.0[..written]).ok()?;
        flash.read_nor(address, &mut buffer.0[..written]).ok()?;
        hash.update(&buffer.0[..n]);
        if offset + (n as u32) < size {
            io(tx.write_packet(&[0])).await?;
        }
    }
    if hash.finalize()[..] != header[8..] {
        return None;
    }
    boot(flash, *b"RUN1", size)?;
    io(tx.write_packet(&[1])).await
}
