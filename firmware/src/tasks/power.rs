//! Sole owner of power I2C: battery monitor, indicator and UI buttons.
use crate::battery::*;
use core::cell::Cell;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    Async,
    gpio::{Input, Output},
    i2c::master::{Error, I2c},
};

static SNAPSHOT: Mutex<CriticalSectionRawMutex, Cell<Snapshot>> =
    Mutex::new(Cell::new(Snapshot::new()));

pub fn snapshot() -> Snapshot {
    SNAPSHOT.lock(Cell::get).fresh(Instant::now().as_millis())
}

#[derive(Clone, Copy, serde::Serialize)]
pub struct Measurement {
    pub at_us: u64,
    pub charger_part: u8,
    pub configuration: ChargerConfiguration,
    pub adc: Option<PowerAdc>,
    pub adc_unavailable_reason: Option<&'static str>,
    pub charger_temperature_c: Option<i16>,
}

static REQUEST: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RESULT: Signal<CriticalSectionRawMutex, Result<Measurement, &'static str>> = Signal::new();

pub async fn measure(explicit_sample: bool) -> Result<Measurement, &'static str> {
    RESULT.reset();
    REQUEST.signal(explicit_sample);
    with_timeout(Duration::from_secs(1), RESULT.wait())
        .await
        .map_err(|_| "power-task-timeout")?
}

struct Bus(I2c<'static, Async>);

impl Registers for Bus {
    async fn read<const N: usize>(
        &mut self,
        address: u8,
        register: u8,
    ) -> Result<[u8; N], &'static str> {
        let mut bytes = [0; N];
        self.0
            .write_read_async(address, &[register], &mut bytes)
            .await
            .map_err(|error| match error {
                Error::AcknowledgeCheckFailed(_) => "nack",
                Error::Timeout => "timeout",
                Error::ArbitrationLost => "arbitration",
                _ => "i2c-error",
            })?;
        Ok(bytes)
    }
}

async fn sample_input(
    bus: &mut Bus,
    detected: Option<u8>,
    power_good_n: &Input<'_>,
    explicit_sample: bool,
) -> Result<Measurement, &'static str> {
    let address = detected.ok_or("unknown-charger")?;
    let part = bus.read::<1>(address, 0x38).await?[0];
    let configuration = read_charger_configuration(bus, address, part).await?;
    let adc_unavailable_reason =
        configuration.adc_unavailable_reason(power_good_n.is_low(), explicit_sample);
    let (adc, charger_temperature_c) = if adc_unavailable_reason.is_none() {
        let (adc, temperature) = read_one_shot(bus, address, power_good_n).await?;
        (Some(adc), temperature)
    } else {
        (None, None)
    };
    Ok(Measurement {
        at_us: Instant::now().as_micros(),
        charger_part: part,
        configuration,
        adc,
        adc_unavailable_reason,
        charger_temperature_c,
    })
}

async fn read_one_shot(
    bus: &mut Bus,
    address: u8,
    power_good_n: &Input<'_>,
) -> Result<(PowerAdc, Option<i16>), &'static str> {
    const ADC_CONTROL: u8 = 0x26;
    const ADC_ENABLE: u8 = 0x80;
    const ADC_ONE_SHOT: u8 = 0x40;
    let control = bus.read::<1>(address, ADC_CONTROL).await?[0];
    if control & ADC_ENABLE != 0 {
        return Err("adc-busy");
    }
    let disabled_channels = bus.read::<1>(address, 0x27).await?[0];
    if disabled_channels & 0xf0 != 0 {
        return Err("power-adc-channel-disabled");
    }
    bus.0
        .write_async(address, &[ADC_CONTROL, ADC_ENABLE | ADC_ONE_SHOT])
        .await
        .map_err(|_| "adc-enable-failed")?;
    let reading = async {
        // At 12-bit resolution the charger needs about 30 ms per channel.
        // Sleep through conversion instead of repeatedly waking and polling I2C.
        let deadline = Instant::now() + Duration::from_millis(350);
        Timer::after_millis(30 * u64::from(disabled_channels.count_zeros())).await;
        loop {
            let status = bus.read::<1>(address, 0x1d).await?[0];
            let adc = bus.read::<1>(address, ADC_CONTROL).await?[0];
            if status & 0x40 != 0 && adc & ADC_ENABLE == 0 {
                break;
            }
            if Instant::now() >= deadline {
                return Err("adc-timeout");
            }
            Timer::after_millis(10).await;
        }
        let input = bus.read(address, 0x28).await?;
        let battery = bus.read(address, 0x30).await?;
        let temperature = if disabled_channels & 0x02 == 0 {
            bus.read::<2>(address, 0x36).await.ok().and_then(|bytes| {
                let half_c = (i16::from_le_bytes(bytes) << 4) >> 4;
                (-80..=280)
                    .contains(&half_c)
                    .then(|| (half_c + half_c.signum()) / 2)
            })
        } else {
            None
        };
        if !power_good_n.is_low() {
            return Err("source-changing");
        }
        Ok((PowerAdc::decode(input, battery)?, temperature))
    }
    .await;
    bus.0
        .write_async(address, &[ADC_CONTROL, control])
        .await
        .map_err(|_| "adc-restore-failed")?;
    reading
}

#[embassy_executor::task]
pub async fn task(bus: I2c<'static, Async>, enable: Output<'static>, power_good_n: Input<'static>) {
    // This task is the only subsequent bus owner; LED recovery never
    // sleeps through a battery poll or requires a second bus lock.
    let mut bus = Bus(bus);
    let mut monitor = Monitor::new();
    let mut buttons = crate::drivers::tca9536::Buttons::new();
    let mut next_battery = Instant::now();
    let mut detected = None;
    let mut led = crate::drivers::lp5813::Indicator::new(enable);
    let mut previous_buttons = 0;
    let mut charger_temperature_deadline = None;
    let mut next_cpu_temperature = Instant::now();
    loop {
        if crate::configuration::BUTTONS_FITTED {
            buttons.poll(&mut bus.0, Instant::now().as_millis()).await;
            if let Some(pressed) = crate::drivers::tca9536::pressed() {
                if pressed & !previous_buttons & 1 != 0 {
                    crate::app::toggle_recording();
                }
                previous_buttons = pressed;
            } else {
                previous_buttons = 0;
            }
        }
        let now = Instant::now();
        if now >= next_battery {
            if now >= next_cpu_temperature {
                next_cpu_temperature = now + Duration::from_secs(10);
                super::thermal::sample_cpu().await;
            }
            if charger_temperature_deadline.is_some_and(|deadline| now >= deadline)
                || !power_good_n.is_low() && charger_temperature_deadline.is_some()
            {
                super::thermal::set_charger(None);
                charger_temperature_deadline = None;
            }
            next_battery = now + Duration::from_millis(500);
            let before = power_good_n.is_low();
            let mut charger = read_charger(&mut bus, &mut detected).await;
            let gauge = read_gauge(&mut bus).await;
            let power_good = power_good_n.is_low();
            if before != power_good {
                charger = Err("source-changing");
            }
            let snapshot = monitor.update(Instant::now().as_millis(), charger, gauge, power_good);
            SNAPSHOT.lock(|cell| cell.set(snapshot));
            led.update(&mut bus.0, snapshot.state, power_good).await;
        }
        let wake = if crate::configuration::BUTTONS_FITTED {
            Instant::now() + Duration::from_millis(10)
        } else {
            next_battery
        };
        if let Either::Second(explicit_sample) = select(Timer::at(wake), REQUEST.wait()).await {
            let result = sample_input(&mut bus, detected, &power_good_n, explicit_sample).await;
            if let Ok(measurement) = &result
                && measurement.adc.is_some()
            {
                super::thermal::set_charger(measurement.charger_temperature_c);
                charger_temperature_deadline = Some(Instant::now() + Duration::from_secs(30));
            }
            RESULT.signal(result);
        }
    }
}
