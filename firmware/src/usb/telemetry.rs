//! Human-readable base status and backwards-compatible RAW capture records.
use crate::{
    drivers::{lp5813, tca9536},
    state::{self, Measurement},
    tasks::display,
};
use core::fmt::{self, Write};
use embassy_time::Instant;

pub fn status(text: &mut impl Write) -> fmt::Result {
    let now = Instant::now().as_micros();
    let imu = state::IMU.get();
    let gnss = state::GNSS.get();
    let lcd = display::STATUS.get();
    write!(
        text,
        "STATUS firmware=base version={} protocol=1 uptime-ms={} imu={} gnss={} imu-records={} gnss-records={} gnss-valid={} fix={} led={} lcd={} lcd-te={} lcd-frames={} lcd-light={} lcd-id={:06x} lcd-regs={:08x} lcd-init={:08x} lcd-dc={} lcd-sclk={}",
        env!("CARGO_PKG_VERSION"),
        now / 1000,
        imu.label(now),
        gnss.label(now),
        imu.records,
        gnss.records,
        gnss.valid,
        state::FIX.get(),
        lp5813::status(),
        lcd.state,
        lcd.te_edges,
        lcd.frames,
        display::backlight::duty(),
        lcd.panel.id,
        lcd.panel.registers,
        lcd.init_regs,
        lcd.dc_levels,
        lcd.sclk_probe
    )?;
    super::battery::append_status(text)?;
    let bridge = super::gnss::STATUS.get();
    write!(
        text,
        " bridge-baud={} bridge-tx={} bridge-rx={} bridge-errors={}",
        bridge.baud, bridge.to_receiver, bridge.from_receiver, bridge.errors
    )?;
    match tca9536::pressed() {
        Some(mask) => write!(text, " buttons={mask:x}")?,
        None => text.write_str(" buttons=unknown")?,
    }
    match state::MEMORY.get() {
        Ok(bytes) => write!(
            text,
            " psram=ready psram-tested={}MiB",
            bytes / (1024 * 1024)
        )?,
        Err(None) => text.write_str(" psram=testing psram-tested=0MiB")?,
        Err(Some(error)) => write!(text, " psram=fault psram-error={error:?}")?,
    }
    write!(
        text,
        " capture-errors={} queue-drops={} last-imu-error={:?} last-gnss-error={:?} reset={:?}\r\n",
        imu.errors.wrapping_add(gnss.errors),
        state::drops(),
        state::IMU_ERROR.get(),
        state::GNSS_ERROR.get(),
        esp_hal::rtc_cntl::reset_reason(esp_hal::system::Cpu::ProCpu)
    )
}

pub fn measurement(text: &mut impl Write, measurement: Measurement) -> fmt::Result {
    match measurement {
        Measurement::Imu(sample) => {
            let a = sample.acceleration_mps2;
            let g = sample.angular_rate_rps;
            write!(
                text,
                "RAW IMU {} {} {:.7} {:.7} {:.7} {:.9} {:.9} {:.9}\r\n",
                sample.monotonic_us, sample.interval_us, a[0], a[1], a[2], g[0], g[1], g[2]
            )
        }
        Measurement::Gnss { received_us, line } => {
            write!(text, "RAW GNSS {received_us} {line}\r\n")
        }
    }
}
