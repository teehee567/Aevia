//! D32 is the power/communication indicator: blue until GNSS messages arrive.
//! LP5813 register map: https://www.ti.com/lit/ds/symlink/lp5813.pdf

use core::cell::Cell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Instant, Timer};
use esp_hal::{Blocking, gpio::Output, i2c::master::I2c};

static LAST_GNSS_MS: Mutex<CriticalSectionRawMutex, Cell<Option<u64>>> =
    Mutex::new(Cell::new(None));
static STATUS: Mutex<CriticalSectionRawMutex, Cell<&'static str>> =
    Mutex::new(Cell::new("starting"));

/// A CRC-checked navigation message proves communication, even without a fix.
pub fn gnss_received() {
    LAST_GNSS_MS.lock(|last| last.set(Some(Instant::now().as_millis())));
}

pub fn gnss_reconfiguring() {
    LAST_GNSS_MS.lock(|last| last.set(None));
}

fn gnss_active() -> bool {
    LAST_GNSS_MS.lock(|last| {
        last.get()
            .is_some_and(|last_ms| Instant::now().as_millis().saturating_sub(last_ms) < 2000)
    })
}

pub fn status() -> &'static str {
    STATUS.lock(Cell::get)
}

fn report(message: &'static str) {
    STATUS.lock(|state| state.set(message));
}

fn read<const N: usize>(
    bus: &mut I2c<'_, Blocking>,
    register: u16,
) -> Result<[u8; N], &'static str> {
    let mut data = [0; N];
    bus.write_read(0x50 | (register >> 8) as u8, &[register as u8], &mut data)
        .map_err(|_| "error i2c-read")?;
    Ok(data)
}

fn write(bus: &mut I2c<'_, Blocking>, register: u8, value: u8) -> Result<(), &'static str> {
    bus.write(0x50, &[register, value])
        .map_err(|_| "error i2c-write")
}

async fn initialize(bus: &mut I2c<'_, Blocking>) -> Result<(), &'static str> {
    write(bus, 0x00, 1)?;
    Timer::after_millis(10).await;
    // Keep the proven four-scan setup and 3 V target (3.3 V pass-through).
    write(bus, 0x02, 0x40)?;
    write(bus, 0x03, 0xe4)?;
    // TI §9.2.3.5 recommends this threshold to avoid false short detection.
    write(bus, 0x0d, 0x0b)?;
    write(bus, 0x10, 0x55)?;
    Timer::after_millis(10).await;
    if read::<4>(bus, 0x000)? != [1, 0, 0x40, 0xe4]
        || read::<1>(bus, 0x00d)? != [0x0b]
        || read::<5>(bus, 0x300)? != [0; 5]
    {
        return Err("error configuration");
    }
    // D32 = group A: A0 green, A1 red, A2 blue. Enable only green/blue.
    // 2 mA peak at 1/4 scan and half PWM gives a dim, steady status light.
    write(bus, 0x34, 20)?;
    write(bus, 0x36, 20)?;
    write(bus, 0x20, 0x50)?;
    write(bus, 0x21, 0)?;
    Ok(())
}

async fn set_color(bus: &mut I2c<'_, Blocking>, green: bool) -> Result<(), &'static str> {
    let mut pwm = [0; 12];
    pwm[if green { 0 } else { 2 }] = 128;
    // One transaction changes D32; the other eleven colour channels stay off.
    bus.write(0x50, &[0x44, pwm[0], 0, pwm[2]])
        .map_err(|_| "error i2c-write")?;
    Timer::after_millis(10).await;
    if read::<12>(bus, 0x044)? != pwm || read::<2>(bus, 0x020)? != [0x50, 0] {
        return Err("error led-readback");
    }
    if read::<5>(bus, 0x300)? != [0; 5] {
        return Err("error driver-fault");
    }
    report(if green { "green" } else { "blue" });
    Ok(())
}

async fn run(bus: &mut I2c<'_, Blocking>) -> Result<(), &'static str> {
    initialize(bus).await?;
    let mut displayed = None;
    loop {
        let green = gnss_active();
        if displayed != Some(green) {
            set_color(bus, green).await?;
            displayed = Some(green);
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
pub async fn task(mut bus: I2c<'static, Blocking>, mut enable: Output<'static>) {
    loop {
        enable.set_low();
        Timer::after_millis(10).await;
        enable.set_high();
        Timer::after_millis(10).await;
        if let Err(error) = run(&mut bus).await {
            report(error);
        }
        enable.set_low();
        Timer::after_secs(3).await;
    }
}
