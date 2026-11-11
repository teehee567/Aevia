//! Shared power-bus owner on the main executor, below the acquisition executor
//! and its timer/GPIO wakeups. No bus handle crosses executor contexts.
use super::{
    board::{MaintenanceResources, PowerBus},
    interface::{self, PeripheralStatus, Phase},
    power, sensors,
};
use aevia_device_hal::{
    bus::SharedI2cDevice,
    lp5813::{self, Lp5813, MiniLed},
    max17048::{self, Max17048},
    tca9536::{ButtonDebouncer, Buttons, Tca9536},
};
use core::{cell::RefCell, sync::atomic::Ordering};
use embassy_time::{Instant, Timer};
use esp_hal::delay::Delay;

fn due(last: Option<Instant>, interval_ms: u64) -> bool {
    last.is_none_or(|last| last.elapsed().as_millis() >= interval_ms)
}

fn led_frame(state: &interface::Snapshot, brightness: u8, imu: bool, gnss: bool) -> [u8; 16] {
    let mut pwm = [0; 16];
    // Physical channel order comes exclusively from HAL, including B's colors.
    let colors = [
        (
            MiniLed::D32,
            if matches!(state.phase, Phase::Failed) {
                [brightness, 0, 0]
            } else {
                [0, 0, brightness]
            },
        ),
        (
            MiniLed::D35,
            match state.phase {
                Phase::Navigating | Phase::Recording => [0, brightness, 0],
                Phase::Degraded | Phase::Failed => [brightness, 0, 0],
                _ => [0, 0, brightness],
            },
        ),
        (
            MiniLed::D33,
            if gnss {
                [0, brightness, 0]
            } else {
                [brightness, 0, 0]
            },
        ),
        (
            MiniLed::D34,
            if imu {
                [0, brightness, 0]
            } else {
                [brightness, 0, 0]
            },
        ),
    ];
    for (led, rgb) in colors {
        for (channel, value) in led.channels().into_iter().zip(rgb) {
            pwm[channel.index() as usize] = value;
        }
    }
    pwm
}

#[embassy_executor::task]
pub(crate) async fn task(bus: PowerBus, resources: MaintenanceResources) {
    let mut enable = resources.led_enable;
    let _power_hold = resources.power_hold;
    // Every handle is used only in this task; RefCell borrows end at each
    // transaction and are never held across an await or an interrupt context.
    let bus = RefCell::new(bus);
    let mut leds = Lp5813::new(SharedI2cDevice::new(&bus), lp5813::Variant::A);
    let mut gauge = Max17048::new(SharedI2cDevice::new(&bus), max17048::Variant::Max17048);
    let mut buttons = Tca9536::v2_mini(SharedI2cDevice::new(&bus));
    let mut debounce = ButtonDebouncer::new(30, Buttons::NONE);
    let mut buttons_ready = false;
    let mut health = PeripheralStatus::default();
    let mut last_pwm = None;
    let mut last_led_attempt = None;
    let mut last_button_attempt = None;
    let mut last_slow_check = None;
    let mut last_button_check = None;
    let mut last_raw_gnss = 0;
    let mut last_gnss_seen = None;
    loop {
        let slow = due(last_slow_check, 1000);
        if slow {
            // This executor preempts rendering, including panel reset delays.
            power::maintain(SharedI2cDevice::new(&bus));
            match if super::board::BATTERY_FITTED {
                Some(gauge.measurements())
            } else {
                None
            } {
                Some(Ok(value)) => {
                    health.battery_voltage_uv = Some(value.voltage_microvolts());
                    health.battery_soc_percent = Some(value.state_of_charge_percent());
                }
                Some(Err(_)) => {
                    health.gauge_errors = health.gauge_errors.saturating_add(1);
                    health.battery_voltage_uv = None;
                    health.battery_soc_percent = None;
                }
                None => {
                    health.battery_voltage_uv = None;
                    health.battery_soc_percent = None;
                }
            }
            if health.leds_ready {
                let healthy = leds.faults().is_ok_and(|f| {
                    !f.configuration_error
                        && !f.thermal_shutdown
                        && (f.open_channels | f.short_channels) & 0xfff0 == 0
                });
                if !healthy {
                    health.led_errors = health.led_errors.saturating_add(1);
                    health.leds_ready = false;
                    enable.set_low();
                    last_led_attempt = Some(Instant::now());
                }
            }
            last_slow_check = Some(Instant::now());
        }
        if !health.leds_ready && due(last_led_attempt, 3000) {
            last_led_attempt = Some(Instant::now());
            last_pwm = None;
            enable.set_low();
            Timer::after_millis(10).await;
            enable.set_high();
            Timer::after_millis(10).await;
            let result = (|| {
                leds.initialize(&lp5813::Config::v2_mini(), &mut Delay::new())?;
                leds.set_enabled(0)?;
                leds.set_manual_pwms(&[0; 16])?;
                let mut currents = [0; 16];
                currents[4..].fill(20); // 2 mA peak/channel, four-scan matrix.
                leds.set_manual_currents(&currents)?;
                leds.clear_faults(true, true, true)?;
                leds.set_enabled(0xfff0)
            })();
            health.leds_ready = result.is_ok();
            if !health.leds_ready {
                enable.set_low();
                health.led_errors = health.led_errors.saturating_add(1);
            }
        }
        if super::board::BUTTONS_FITTED && !buttons_ready && due(last_button_attempt, 3000) {
            last_button_attempt = Some(Instant::now());
            buttons_ready = buttons.configure_buttons().is_ok();
            if !buttons_ready {
                health.buttons_errors = health.buttons_errors.saturating_add(1);
            }
            debounce = ButtonDebouncer::new(30, Buttons::NONE);
        }
        if buttons_ready && due(last_button_check, 50) {
            last_button_check = Some(Instant::now());
            match buttons.buttons() {
                Ok(raw) => {
                    health.buttons_pressed = Some(
                        debounce
                            .update(Instant::now().as_millis(), raw)
                            .state
                            .pressed_mask(),
                    )
                }
                Err(_) => {
                    health.buttons_pressed = None;
                    health.buttons_errors = health.buttons_errors.saturating_add(1);
                    buttons_ready = false;
                    last_button_attempt = Some(Instant::now());
                }
            }
        }
        let now = Instant::now().as_micros();
        let state = interface::snapshot();
        let settings = interface::indicators();
        let imu = state.last_imu_us != 0 && now.saturating_sub(state.last_imu_us) < 100_000;
        let raw_gnss = sensors::GNSS_SAMPLES.load(Ordering::Relaxed);
        if raw_gnss != last_raw_gnss {
            last_raw_gnss = raw_gnss;
            last_gnss_seen = Some(Instant::now());
        }
        let gnss = last_gnss_seen.is_some_and(|last| last.elapsed().as_millis() < 1000);
        let pwm = led_frame(&state, settings.led_brightness, imu, gnss);
        if health.leds_ready && last_pwm != Some(pwm) {
            if leds.set_manual_pwms(&pwm).is_ok() {
                last_pwm = Some(pwm);
            } else {
                enable.set_low();
                health.leds_ready = false;
                health.led_errors = health.led_errors.saturating_add(1);
                last_led_attempt = Some(Instant::now());
            }
        }
        health.updated_us = Instant::now().as_micros();
        interface::publish_peripherals(health);
        Timer::after_millis(25).await;
    }
}
