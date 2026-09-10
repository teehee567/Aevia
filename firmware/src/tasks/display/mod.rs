//! Panel bring-up and optional test pattern. A failed panel cannot stall USB or sensors.
pub mod backlight;

use crate::{
    drivers::st7789::{Lcd, Readback},
    state::Shared,
};
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::gpio::Input;

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub state: &'static str,
    pub te_edges: u32,
    pub frames: u32,
    pub panel: Readback,
    pub init_regs: u32,
    pub dc_levels: u32,
    pub sclk_probe: u32,
}
pub static STATUS: Shared<Snapshot> = Shared::new(Snapshot {
    state: "starting",
    te_edges: 0,
    frames: 0,
    panel: Readback {
        id: 0,
        registers: 0,
    },
    init_regs: 0,
    dc_levels: 0,
    sclk_probe: 0,
});
static REINITIALIZE: AtomicBool = AtomicBool::new(false);
static TEST: AtomicBool = AtomicBool::new(false);

pub fn reinitialize() {
    REINITIALIZE.store(true, Ordering::Relaxed);
}
pub fn test_pattern() {
    TEST.store(true, Ordering::Relaxed);
}

#[embassy_executor::task]
pub async fn run(mut lcd: Lcd, mut te: Input<'static>, sclk_probe: u32) {
    STATUS.update(|status| status.sclk_probe = sclk_probe);
    if sclk_probe != 26 {
        STATUS.update(|status| status.state = "sclk-check-failed");
        core::future::pending::<()>().await; // SCLK is disconnected until reboot.
    }
    loop {
        backlight::allow(false);
        STATUS.update(|status| status.state = "initializing");
        let dc = lcd.check_dc();
        STATUS.update(|status| status.dc_levels = dc);
        if dc != 5 {
            if let Ok(panel) = lcd.readback() {
                STATUS.update(|status| status.panel = panel);
            }
            STATUS.update(|status| status.state = "dc-stuck-low");
            while !REINITIALIZE.swap(false, Ordering::Relaxed) {
                Timer::after_millis(100).await;
            }
            continue;
        }
        let result = async {
            lcd.initialize().await?;
            let panel = lcd.readback()?;
            STATUS.update(|status| {
                status.init_regs = panel.registers;
                status.panel = panel;
            });
            lcd.write_frame(|_, row| row.fill(0)).await?;
            lcd.display_on()?;
            Timer::after_millis(100).await;
            let panel = lcd.readback()?;
            STATUS.update(|status| {
                status.panel = panel;
                status.frames = status.frames.wrapping_add(1);
            });
            backlight::allow(true);
            backlight::request(10);
            let mut marker = 4;
            loop {
                let start = Instant::now();
                let mut edges = 0;
                while start.elapsed() < Duration::from_millis(500) {
                    if with_timeout(Duration::from_millis(50), te.wait_for_rising_edge())
                        .await
                        .is_ok()
                    {
                        edges += 1;
                    }
                }
                STATUS.update(|status| {
                    status.te_edges = status.te_edges.wrapping_add(edges);
                    status.state = if edges >= 5 { "scanning" } else { "no-te" };
                });
                if TEST.load(Ordering::Relaxed) {
                    lcd.write_frame(|y, row| render_pattern(y, row, marker))
                        .await?;
                    STATUS.update(|status| status.frames = status.frames.wrapping_add(1));
                    marker = if marker >= 220 { 4 } else { marker + 12 };
                }
                if REINITIALIZE.swap(false, Ordering::Relaxed) {
                    break;
                }
            }
            Ok::<(), esp_hal::spi::Error>(())
        }
        .await;
        if result.is_err() {
            backlight::allow(false);
            STATUS.update(|status| status.state = "spi-error");
            Timer::after_secs(3).await;
        }
    }
}

fn render_pattern(y: u16, row: &mut [u8; 480], marker: u16) {
    let colors = [0xf800_u16, 0x07e0, 0x001f, 0xffe0, 0x07ff, 0xf81f];
    for x in 0..240_u16 {
        let color = if !(3..237).contains(&x) || !(3..237).contains(&y) {
            0xffff
        } else if y >= 210 {
            if (marker..marker + 12).contains(&x) {
                0xffff
            } else {
                0
            }
        } else {
            colors[(x / 40) as usize]
        };
        row[x as usize * 2..x as usize * 2 + 2].copy_from_slice(&color.to_be_bytes());
    }
}
