//! D35 edge battery indicator, driven by the battery task that owns power I2C.
//! LP5813 register map: https://www.ti.com/lit/ds/symlink/lp5813.pdf

use aevia_firmware::battery::State;
use core::cell::Cell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Instant, Timer};
use esp_hal::{Async, gpio::Output, i2c::master::I2c};

// D35 anodes are on OUT1: B0 sinks green on OUT2, B1 red on OUT3,
// B2 blue on OUT0. Their enable bits span the two enable registers.
const STATUS_LED_ENABLE: [u8; 2] = [0x80, 0x03];

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
    // D35 = group B: B0 green, B1 red, B2 blue.
    // 2 mA peak at 1/4 scan and half PWM gives a dim, steady status light.
    write(bus, 0x37, 20).await?;
    write(bus, 0x38, 20).await?;
    write(bus, 0x39, 20).await?;
    write(bus, 0x20, STATUS_LED_ENABLE[0]).await?;
    write(bus, 0x21, STATUS_LED_ENABLE[1]).await?;
    Ok(())
}

async fn set_color(bus: &mut I2c<'_, Async>, state: State) -> Result<(), &'static str> {
    let mut pwm = [0; 12];
    let (color, channels) = match state {
        State::Unknown => ("off", [0, 0, 0]),
        State::Charging => ("green", [128, 0, 0]),
        State::Bypass => ("blue", [0, 0, 128]),
        State::Battery => ("purple", [0, 128, 128]),
        State::Low => ("red", [0, 128, 0]),
    };
    pwm[3..6].copy_from_slice(&channels);
    // Write all PWM values so the previous indicator cannot retain a colour.
    let mut payload = [0; 13];
    payload[0] = 0x44;
    payload[1..].copy_from_slice(&pwm);
    bus.write_async(0x50, &payload)
        .await
        .map_err(|_| "error i2c-write")?;
    Timer::after_millis(10).await;
    if read::<12>(bus, 0x044).await? != pwm || read::<2>(bus, 0x020).await? != STATUS_LED_ENABLE {
        return Err("error led-readback");
    }
    if read::<5>(bus, 0x300).await? != [0; 5] {
        return Err("error driver-fault");
    }
    report(color);
    Ok(())
}

pub struct Indicator {
    enable: Output<'static>,
    ready: bool,
    retry_ms: u64,
}

impl Indicator {
    pub fn new(enable: Output<'static>) -> Self {
        Self {
            enable,
            ready: false,
            retry_ms: 0,
        }
    }

    pub async fn update(&mut self, bus: &mut I2c<'_, Async>, state: State) {
        if Instant::now().as_millis() < self.retry_ms {
            return;
        }
        let result = async {
            if !self.ready {
                self.enable.set_low();
                Timer::after_millis(10).await;
                self.enable.set_high();
                Timer::after_millis(10).await;
                initialize(bus).await?;
                self.ready = true;
            }
            // Verify even a steady colour, detecting driver resets and faults.
            set_color(bus, state).await
        }
        .await;
        if let Err(error) = result {
            report(error);
            self.enable.set_low();
            self.ready = false;
            self.retry_ms = Instant::now().as_millis() + 3_000;
        }
    }
}
