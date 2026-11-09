//! Low-rate status UI. Display writes yield after each scanline so UART and
//! data-ready capture can run between transfers; no full-screen framebuffer.
use super::{
    interface::{self, Phase},
    sensors,
};
use core::{
    convert::Infallible,
    fmt::{self, Write},
    sync::atomic::Ordering,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::{Rgb565, raw::RawU16},
    prelude::*,
    text::Text,
};
use esp_hal::{Async, Blocking, gpio::Output, i2c::master::I2c, spi::master::Spi};

const WIDTH: usize = 240;
const HEIGHT: usize = 16;

pub struct Display<'d> {
    spi: Spi<'d, Blocking>,
    cs: Output<'d>,
    dc: Output<'d>,
    reset: Output<'d>,
    backlight: Output<'d>,
}
impl<'d> Display<'d> {
    pub fn new(
        spi: Spi<'d, Blocking>,
        cs: Output<'d>,
        dc: Output<'d>,
        reset: Output<'d>,
        backlight: Output<'d>,
    ) -> Self {
        Self {
            spi,
            cs,
            dc,
            reset,
            backlight,
        }
    }
    fn command(&mut self, command: u8, data: &[u8]) -> Result<(), ()> {
        self.cs.set_low();
        self.dc.set_low();
        let result = self.spi.write(&[command]);
        self.dc.set_high();
        let result = result.and_then(|_| self.spi.write(data));
        self.cs.set_high();
        result.map_err(|_| ())
    }
    async fn initialize(&mut self) -> Result<(), ()> {
        self.backlight.set_low();
        self.reset.set_low();
        Timer::after_millis(20).await;
        self.reset.set_high();
        Timer::after_millis(120).await;
        self.command(0x01, &[])?; // ST7789 software reset.
        Timer::after_millis(150).await;
        self.command(0x11, &[])?;
        Timer::after_millis(120).await;
        self.command(0x3a, &[0x55])?; // RGB565.
        self.command(0x36, &[0])?;
        self.command(0x21, &[])?; // IPS panel inversion.
        self.command(0x13, &[])?;
        self.command(0x29, &[])?;
        let black = [0; WIDTH * 2];
        for y in 0..240 {
            self.row(y, &black).await?;
        }
        self.backlight.set_high();
        Ok(())
    }
    async fn row(&mut self, y: u16, pixels: &[u8; WIDTH * 2]) -> Result<(), ()> {
        // Adafruit 4520's 240x240 visible panel starts at ST7789 row 80.
        let row = (y + 80).to_be_bytes();
        self.command(0x2a, &[0, 0, 0, 239])?;
        self.command(0x2b, &[row[0], row[1], row[0], row[1]])?;
        self.command(0x2c, pixels)?;
        Timer::after_micros(1).await;
        Ok(())
    }
    async fn line(&mut self, y: u16, text: &str, color: Rgb565) -> Result<(), ()> {
        let mut strip = Strip([0; WIDTH * HEIGHT]);
        Text::new(
            text,
            Point::new(4, 11),
            MonoTextStyle::new(&FONT_6X10, color),
        )
        .draw(&mut strip)
        .unwrap();
        let mut row = [0; WIDTH * 2];
        for r in 0..HEIGHT {
            for (dst, value) in row
                .chunks_exact_mut(2)
                .zip(&strip.0[r * WIDTH..(r + 1) * WIDTH])
            {
                dst.copy_from_slice(&value.to_be_bytes());
            }
            self.row(y + r as u16, &row).await?;
        }
        Ok(())
    }
}
struct Strip([u16; WIDTH * HEIGHT]);
impl OriginDimensions for Strip {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
impl DrawTarget for Strip {
    type Color = Rgb565;
    type Error = Infallible;
    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Infallible> {
        for Pixel(point, color) in pixels {
            if point.x >= 0 && point.x < WIDTH as i32 && point.y >= 0 && point.y < HEIGHT as i32 {
                self.0[point.y as usize * WIDTH + point.x as usize] =
                    RawU16::from(color).into_inner();
            }
        }
        Ok(())
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
struct Line {
    bytes: [u8; 40],
    len: usize,
}
impl Line {
    const fn new() -> Self {
        Self {
            bytes: [0; 40],
            len: 0,
        }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}
impl Write for Line {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if text.len() > self.bytes.len() - self.len {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}

async fn led_write(bus: &mut I2c<'_, Async>, register: u8, bytes: &[u8]) -> Result<(), ()> {
    let mut transfer = [0; 17];
    transfer[0] = register;
    transfer[1..=bytes.len()].copy_from_slice(bytes);
    with_timeout(
        Duration::from_millis(10),
        bus.write_async(0x50, &transfer[..bytes.len() + 1]),
    )
    .await
    .map_err(|_| ())?
    .map_err(|_| ())
}
async fn led_faults(bus: &mut I2c<'_, Async>) -> Result<(), ()> {
    let mut status = [0; 5];
    with_timeout(
        Duration::from_millis(10),
        bus.write_read_async(0x53, &[0], &mut status),
    )
    .await
    .map_err(|_| ())?
    .map_err(|_| ())?;
    if status == [0; 5] { Ok(()) } else { Err(()) }
}
async fn led_initialize(bus: &mut I2c<'_, Async>, enable: &mut Output<'_>) -> Result<(), ()> {
    enable.set_low();
    Timer::after_millis(10).await;
    enable.set_high();
    Timer::after_millis(10).await;
    led_write(bus, 0, &[1]).await?;
    Timer::after_millis(10).await;
    for (register, value) in [(2, 0x40), (3, 0xe4), (0x0d, 0x0b), (0x10, 0x55)] {
        led_write(bus, register, &[value]).await?;
    }
    Timer::after_millis(10).await;
    led_write(bus, 0x34, &[20; 12]).await?; // 2mA/channel, quarter scan.
    led_write(bus, 0x20, &[0xf0, 0xff]).await?;
    led_faults(bus).await
}

#[embassy_executor::task]
pub async fn task(
    mut bus: I2c<'static, Async>,
    mut enable: Output<'static>,
    mut display: Display<'static>,
    _power: Output<'static>,
) {
    let mut leds_ready = false;
    let mut display_ready = false;
    let mut old = [Line::new(); 8];
    let mut last_display = Instant::now();
    let mut last_pwm = None;
    let mut last_fault_check = Instant::now();
    loop {
        if !leds_ready {
            leds_ready = led_initialize(&mut bus, &mut enable).await.is_ok();
            last_pwm = None;
        }
        if !display_ready {
            display_ready = display.initialize().await.is_ok();
            old = [Line::new(); 8];
        }
        let now = Instant::now().as_micros();
        let state = interface::snapshot();
        let settings = interface::indicators();
        let imu = state.last_imu_us != 0 && now.saturating_sub(state.last_imu_us) < 100_000;
        let gnss = state.last_gnss_us != 0 && now.saturating_sub(state.last_gnss_us) < 1_000_000;
        let b = settings.led_brightness;
        let mut pwm = [0; 12];
        // D32 system (A: green/red/blue), D35 processing (B: blue/green/red),
        // D33 GNSS (C: red/blue/green), D34 IMU (D: green/red/blue).
        pwm[if matches!(state.phase, Phase::Failed) {
            1
        } else {
            2
        }] = b;
        pwm[if matches!(state.phase, Phase::Navigating) {
            4
        } else if matches!(state.phase, Phase::Degraded | Phase::Failed) {
            5
        } else {
            3
        }] = b;
        pwm[if gnss { 8 } else { 7 }] = b;
        pwm[if imu { 9 } else { 10 }] = b;
        if leds_ready && last_pwm != Some(pwm) {
            if led_write(&mut bus, 0x44, &pwm).await.is_ok() {
                last_pwm = Some(pwm);
            } else {
                enable.set_low();
                leds_ready = false;
            }
        }
        if leds_ready && last_fault_check.elapsed().as_millis() >= 1000 {
            last_fault_check = Instant::now();
            if led_faults(&mut bus).await.is_err() {
                enable.set_low();
                leds_ready = false;
            }
        }
        if settings.display_enabled {
            if display_ready {
                display.backlight.set_high();
            }
        } else {
            display.backlight.set_low();
        }
        if display_ready && settings.display_enabled && last_display.elapsed().as_millis() >= 200 {
            last_display = Instant::now();
            let mut lines = [Line::new(); 8];
            let phase = match state.phase {
                Phase::Starting => "STARTING",
                Phase::Initializing => "ALIGNING",
                Phase::Navigating => "NAVIGATING",
                Phase::Degraded => "DEGRADED",
                Phase::Failed => "ENGINE ERROR",
            };
            let _ = write!(lines[0], "AEVIA / {}", phase);
            if let Some(speed) = state.horizontal_speed_mps {
                let _ = write!(lines[1], "SPEED {:6.1} km/h", speed * 3.6);
            } else {
                let _ = write!(lines[1], "SPEED -- km/h");
            }
            let _ = write!(
                lines[2],
                "GNSS {} Hz  SAT {}",
                sensors::GNSS_RATE_HZ.load(Ordering::Relaxed),
                state.satellites
            );
            let _ = write!(
                lines[3],
                "IMU {}  PPS {}",
                if imu { "OK" } else { "WAIT" },
                if state.timing.pps_locked {
                    "LOCK"
                } else {
                    "WAIT"
                }
            );
            let _ = write!(
                lines[4],
                "DROPS I:{} G:{}",
                state.imu_drops, state.gnss_drops
            );
            let _ = write!(
                lines[5],
                "ERRORS I:{} G:{}",
                sensors::IMU_ERRORS.load(Ordering::Relaxed),
                sensors::GNSS_ERRORS.load(Ordering::Relaxed)
            );
            let _ = write!(
                lines[6],
                "ENGINE {}  MAX {}us",
                state.processing_errors, state.maximum_step_us
            );
            let _ = write!(lines[7], "DEVELOPMENT / UNCALIBRATED");
            for (i, line) in lines.iter().enumerate() {
                if *line != old[i] {
                    let color = if i == 0 {
                        Rgb565::CYAN
                    } else if i >= 4 {
                        Rgb565::YELLOW
                    } else {
                        Rgb565::WHITE
                    };
                    if display
                        .line(12 + i as u16 * 26, line.as_str(), color)
                        .await
                        .is_err()
                    {
                        display.backlight.set_low();
                        display_ready = false;
                        break;
                    }
                    old[i] = *line;
                }
            }
        }
        Timer::after_millis(if leds_ready && display_ready {
            100
        } else {
            3000
        })
        .await;
    }
}
