//! Small on-device instrument view. Failed GPIO/panel checks keep the backlight off.
use crate::drivers::st7789::Lcd;
use core::{cell::Cell, fmt::Write};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::Timer;
use esp_hal::gpio::Output;
static STATUS: Mutex<CriticalSectionRawMutex, Cell<&'static str>> =
    Mutex::new(Cell::new(if crate::configuration::DISPLAY_FITTED {
        "starting"
    } else {
        "not-fitted"
    }));
pub fn status() -> &'static str {
    STATUS.lock(Cell::get)
}
fn report(value: &'static str) {
    STATUS.lock(|s| s.set(value));
}

#[embassy_executor::task]
pub async fn run(mut lcd: Lcd, mut backlight: Output<'static>, sclk: u32) {
    if !crate::configuration::DISPLAY_FITTED {
        return;
    }
    if sclk != 26 {
        report("sclk-check-failed");
        return;
    }
    if lcd.check_dc() != 5 {
        report("dc-check-failed");
        return;
    }
    loop {
        backlight.set_low();
        report("initializing");
        let result: Result<(), esp_hal::spi::Error> = async {
            lcd.initialize().await?;
            let registers = lcd.readback()?;
            // Floating/shorted reads are not proof that a panel is present.
            if registers.id == 0 || registers.id == 0xffffff || registers.registers == 0 {
                report("no-panel-readback");
                return Ok(());
            }
            lcd.display_on()?;
            loop {
                let (speed, recording, imu) = crate::app::dashboard();
                let sd = crate::storage::snapshot();
                let mut value = heapless::String::<8>::new();
                if let Some(speed) = speed {
                    let _ = write!(value, "{speed}");
                } else {
                    let _ = value.push_str("---");
                }
                lcd.write_frame(|y, row| {
                    row.fill(0);
                    text(row, y, 12, 12, "AEVIA", 2, 0xbdf7);
                    text(row, y, 12, 48, &value, 6, 0xffff);
                    text(row, y, 12, 100, "KM/H", 2, 0xbdf7);
                    text(
                        row,
                        y,
                        12,
                        132,
                        if speed.is_some() {
                            "GNSS FIX"
                        } else {
                            "GNSS NO FIX"
                        },
                        2,
                        if speed.is_some() { 0x07ff } else { 0xfd20 },
                    );
                    text(
                        row,
                        y,
                        12,
                        156,
                        if imu { "IMU READY" } else { "IMU WAIT" },
                        2,
                        0xbdf7,
                    );
                    text(
                        row,
                        y,
                        12,
                        190,
                        if recording { "RECORDING" } else { "READY" },
                        2,
                        if recording { 0xf800 } else { 0x07ff },
                    );
                    text(
                        row,
                        y,
                        12,
                        218,
                        if sd.error.is_some() {
                            "SD FAULT - USB HOST"
                        } else if !sd.closed {
                            "SD RECORDING"
                        } else {
                            "SD READY"
                        },
                        1,
                        0xbdf7,
                    );
                })
                .await?;
                backlight.set_high();
                report("ready");
                Timer::after_millis(250).await;
            }
        }
        .await;
        if result.is_err() {
            report("spi-error");
        }
        backlight.set_low();
        Timer::after_secs(5).await;
    }
}

fn text(row: &mut [u8; 480], y: u16, left: u16, top: u16, value: &str, scale: u16, color: u16) {
    if y < top || y >= top + 7 * scale {
        return;
    }
    let glyph_row = ((y - top) / scale) as usize;
    for (i, ch) in value.bytes().enumerate() {
        let bits = glyph(ch)[glyph_row];
        for column in 0..5 {
            if bits & (1 << (4 - column)) == 0 {
                continue;
            }
            let x = left + (i as u16 * 6 + column) * scale;
            for dx in 0..scale {
                if x + dx < 240 {
                    let offset = usize::from(x + dx) * 2;
                    row[offset..offset + 2].copy_from_slice(&color.to_be_bytes());
                }
            }
        }
    }
}
fn glyph(ch: u8) -> [u8; 7] {
    match ch {
        b'A' => [14, 17, 17, 31, 17, 17, 17],
        b'B' => [30, 17, 17, 30, 17, 17, 30],
        b'C' => [14, 17, 16, 16, 16, 17, 14],
        b'D' => [30, 17, 17, 17, 17, 17, 30],
        b'E' => [31, 16, 16, 30, 16, 16, 31],
        b'F' => [31, 16, 16, 30, 16, 16, 16],
        b'G' => [14, 17, 16, 23, 17, 17, 15],
        b'H' => [17, 17, 17, 31, 17, 17, 17],
        b'I' => [31, 4, 4, 4, 4, 4, 31],
        b'K' => [17, 18, 20, 24, 20, 18, 17],
        b'M' => [17, 27, 21, 21, 17, 17, 17],
        b'N' => [17, 25, 25, 21, 19, 19, 17],
        b'O' => [14, 17, 17, 17, 17, 17, 14],
        b'R' => [30, 17, 17, 30, 20, 18, 17],
        b'S' => [15, 16, 16, 14, 1, 1, 30],
        b'T' => [31, 4, 4, 4, 4, 4, 4],
        b'U' => [17, 17, 17, 17, 17, 17, 14],
        b'V' => [17, 17, 17, 17, 17, 10, 4],
        b'W' => [17, 17, 17, 21, 21, 21, 10],
        b'X' => [17, 17, 10, 4, 10, 17, 17],
        b'Y' => [17, 17, 10, 4, 4, 4, 4],
        b'0' => [14, 17, 19, 21, 25, 17, 14],
        b'1' => [4, 12, 4, 4, 4, 4, 14],
        b'2' => [14, 17, 1, 2, 4, 8, 31],
        b'3' => [30, 1, 1, 14, 1, 1, 30],
        b'4' => [2, 6, 10, 18, 31, 2, 2],
        b'5' => [31, 16, 16, 30, 1, 1, 30],
        b'6' => [14, 16, 16, 30, 17, 17, 14],
        b'7' => [31, 1, 2, 4, 8, 8, 8],
        b'8' => [14, 17, 17, 14, 17, 17, 14],
        b'9' => [14, 17, 17, 15, 1, 1, 14],
        b'-' => [0, 0, 0, 31, 0, 0, 0],
        b'/' => [1, 1, 2, 4, 8, 16, 16],
        _ => [0; 7],
    }
}
