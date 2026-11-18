//! Four functional RGB indicators, driven by the sole power-I2C owner.
//! LP5813 register map: https://www.ti.com/lit/ds/symlink/lp5813.pdf

use crate::battery::State;
use core::cell::Cell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Instant, Timer};
use esp_hal::{Async, gpio::Output, i2c::master::I2c};

// Enable all twelve multiplexed channels while direct-output enables stay off.
const STATUS_LED_ENABLE: [u8; 2] = [0xf0, 0xff];

static STATUS: Mutex<CriticalSectionRawMutex, Cell<&'static str>> =
    Mutex::new(Cell::new("starting"));

pub fn status() -> &'static str {
    STATUS.lock(Cell::get)
}

fn report(message: &'static str) {
    STATUS.lock(|state| state.set(message));
}

async fn read<const N: usize>(
    bus: &mut I2c<'_, Async>,
    register: u16,
) -> Result<[u8; N], &'static str> {
    let mut data = [0; N];
    bus.write_read_async(0x50 | (register >> 8) as u8, &[register as u8], &mut data)
        .await
        .map_err(|_| "error i2c-read")?;
    Ok(data)
}

async fn write(bus: &mut I2c<'_, Async>, register: u8, value: u8) -> Result<(), &'static str> {
    bus.write_async(0x50, &[register, value])
        .await
        .map_err(|_| "error i2c-write")
}

async fn initialize(bus: &mut I2c<'_, Async>) -> Result<(), &'static str> {
    write(bus, 0x00, 1).await?;
    Timer::after_millis(10).await;
    // Keep the proven four-scan setup and 3 V target (3.3 V pass-through).
    write(bus, 0x02, 0x40).await?;
    write(bus, 0x03, 0xe4).await?;
    // TI section 9.2.3.5 recommends this threshold to avoid false short detection.
    write(bus, 0x0d, 0x0b).await?;
    write(bus, 0x10, 0x55).await?;
    Timer::after_millis(10).await;
    if read::<4>(bus, 0x000).await? != [1, 0, 0x40, 0xe4]
        || read::<1>(bus, 0x00d).await? != [0x0b]
        || read::<5>(bus, 0x300).await? != [0; 5]
    {
        return Err("error configuration");
    }
    // 2 mA peak at 1/4 scan and half PWM keeps status lights dim.
    for register in 0x34..=0x3f {
        write(bus, register, 20).await?;
    }
    write(bus, 0x20, STATUS_LED_ENABLE[0]).await?;
    write(bus, 0x21, STATUS_LED_ENABLE[1]).await?;
    Ok(())
}

fn status_pwm(state: State, external_power: bool) -> [u8; 12] {
    let mut pwm = [0; 12];
    let channels = match state {
        State::Unknown => [0, 0, 0],
        State::Charging => [128, 0, 0],
        State::Bypass => [0, 0, 128],
        State::Battery => [0, 128, 128],
        State::Low => [0, 128, 0],
    };
    // PCB anodes: D32 OUT0(group A), D35 OUT1(B), D33 OUT2(C), D34 OUT3(D).
    // All groups map their sequential sink channels as green, red, blue.
    let (gnss, imu) = crate::app::sensor_indicators();
    pwm[..3].copy_from_slice(&gnss);
    pwm[3..6].copy_from_slice(&channels);
    pwm[6..9].copy_from_slice(&imu);
    let sd = crate::storage::snapshot();
    let recording = crate::app::dashboard().1;
    let flash = Instant::now().as_millis() % 1000 < 500;
    let activity = if sd.error.is_some() || sd.dropped > 0 {
        [0, 100, 0]
    } else if sd.state == "stopping" {
        [if flash { 70 } else { 0 }, if flash { 100 } else { 0 }, 0]
    } else if recording {
        [if flash { 100 } else { 0 }, 0, 0]
    } else {
        [0, 0, 80]
    };
    pwm[9..12].copy_from_slice(&activity);
    if !external_power {
        // Preserve the colour and blink pattern at a quarter of the USB duty.
        for channel in &mut pwm {
            *channel /= 4;
        }
    }
    pwm
}

pub struct Indicator {
    enable: Output<'static>,
    applied_pwm: Option<[u8; 12]>,
    retry_ms: u64,
}

impl Indicator {
    pub fn new(enable: Output<'static>) -> Self {
        Self {
            enable,
            applied_pwm: None,
            retry_ms: 0,
        }
    }

    pub async fn update(&mut self, bus: &mut I2c<'_, Async>, state: State, external_power: bool) {
        if Instant::now().as_millis() < self.retry_ms {
            return;
        }
        let result = async {
            if self.applied_pwm.is_none() {
                self.enable.set_low();
                Timer::after_millis(10).await;
                self.enable.set_high();
                Timer::after_millis(10).await;
                initialize(bus).await?;
            }
            let pwm = status_pwm(state, external_power);
            if self.applied_pwm != Some(pwm) {
                // Include every channel so previous colours are cleared.
                let mut payload = [0; 13];
                payload[0] = 0x44;
                payload[1..].copy_from_slice(&pwm);
                bus.write_async(0x50, &payload)
                    .await
                    .map_err(|_| "error i2c-write")?;
                Timer::after_millis(10).await;
            }
            // Detect driver resets and faults even when the colour is steady.
            if read::<12>(bus, 0x044).await? != pwm
                || read::<2>(bus, 0x020).await? != STATUS_LED_ENABLE
            {
                return Err("error led-readback");
            }
            if read::<5>(bus, 0x300).await? != [0; 5] {
                return Err("error driver-fault");
            }
            self.applied_pwm = Some(pwm);
            report("functional-status");
            Ok(())
        }
        .await;
        if let Err(error) = result {
            report(error);
            self.enable.set_low();
            self.applied_pwm = None;
            self.retry_ms = Instant::now().as_millis() + 3_000;
        }
    }
}
