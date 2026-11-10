//! Evaluation rendering. Panel failures are bounded by the platform bus;
//! power, gauge, buttons and LEDs execute independently on the maintenance IRQ.
use super::{
    interface::{self, Phase},
    platform::DisplayBus,
    sensors,
};
use aevia_device_hal::{
    bus::ExclusiveSpiDevice,
    display::{self, St7789},
};
use core::{
    convert::Infallible,
    fmt::{self, Write},
    sync::atomic::Ordering,
};
use embassy_time::{Instant, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::{Rgb565, raw::RawU16},
    prelude::*,
    text::Text,
};
use esp_hal::{delay::Delay, gpio::Output};

const WIDTH: usize = 240;
const HEIGHT: usize = 16;
type PanelSpi = ExclusiveSpiDevice<DisplayBus, Output<'static>, Delay>;
type Panel = St7789<PanelSpi, Output<'static>, Output<'static>, Output<'static>, Delay>;

pub(crate) struct Display {
    panel: Panel,
}
impl Display {
    pub fn new(
        spi: DisplayBus,
        cs: Output<'static>,
        dc: Output<'static>,
        reset: Output<'static>,
        backlight: Output<'static>,
    ) -> Self {
        let mut spi = ExclusiveSpiDevice::new(spi, cs, Delay::new());
        spi.initialize().expect("display CS");
        Self {
            panel: St7789::new(
                spi,
                dc,
                reset,
                backlight,
                Delay::new(),
                display::Config::v2_mini(),
            ),
        }
    }
    async fn initialize(&mut self) -> Result<(), ()> {
        self.panel.initialize().map_err(|_| ())?;
        let black = [0; WIDTH];
        for y in 0..240 {
            self.panel.write_row(0, y, &black).map_err(|_| ())?;
            Timer::after_micros(1).await;
        }
        // Keep undefined frame RAM hidden; caller chooses backlight after clear.
        Ok(())
    }
    fn backlight(&mut self, enabled: bool) {
        let _ = self.panel.set_backlight(enabled);
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
        for r in 0..HEIGHT {
            self.panel
                .write_row(0, y + r as u16, &strip.0[r * WIDTH..(r + 1) * WIDTH])
                .map_err(|_| ())?;
            Timer::after_micros(1).await;
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

fn due(last: Option<Instant>, interval_ms: u64) -> bool {
    last.is_none_or(|last| last.elapsed().as_millis() >= interval_ms)
}

#[embassy_executor::task]
pub(crate) async fn task(mut display: Display) {
    let mut ready = false;
    let mut errors: u32 = 0;
    let mut old = [Line::new(); 8];
    let mut last_display = None;
    let mut last_display_attempt = None;
    loop {
        if !ready && due(last_display_attempt, 3000) {
            last_display_attempt = Some(Instant::now());
            ready = display.initialize().await.is_ok();
            old = [Line::new(); 8];
            if !ready {
                display.backlight(false);
                errors = errors.saturating_add(1);
            }
        }
        let state = interface::snapshot();
        let settings = interface::indicators();
        let now = Instant::now().as_micros();
        let imu = state.last_imu_us != 0 && now.saturating_sub(state.last_imu_us) < 100_000;
        display.backlight(ready && settings.display_enabled);
        if ready && settings.display_enabled && due(last_display, 200) {
            last_display = Some(Instant::now());
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
                        display.backlight(false);
                        ready = false;
                        errors = errors.saturating_add(1);
                        last_display_attempt = Some(Instant::now());
                        break;
                    }
                    old[i] = *line;
                }
            }
        }
        interface::publish_display(ready, errors);
        Timer::after_millis(25).await;
    }
}
