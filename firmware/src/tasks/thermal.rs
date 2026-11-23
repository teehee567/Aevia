//! Latest whole-degree readings for diagnostics. No filtering or sample history.
use core::sync::atomic::{AtomicU8, Ordering::Relaxed};
use embassy_time::Timer;

static CHARGER: AtomicU8 = AtomicU8::new(255);
static IMU: AtomicU8 = AtomicU8::new(255);
static CPU: AtomicU8 = AtomicU8::new(255);

pub fn set_charger(value: Option<i16>) {
    CHARGER.store(crate::display::pack_temperature(value), Relaxed);
}

pub fn set_imu(value: Option<i16>) {
    IMU.store(crate::display::pack_temperature(value), Relaxed);
}

pub fn set_cpu(value: Option<i16>) {
    CPU.store(crate::display::pack_temperature(value), Relaxed);
}

pub fn packed() -> [u8; 3] {
    [CHARGER.load(Relaxed), IMU.load(Relaxed), CPU.load(Relaxed)]
}

/// ESP-IDF's S31 driver uses the LP TSENS block and a DAC range in the
/// internal analog bus. Keep the current DAC range so PHY calibration and
/// this sampler can share the sensor. Power it only long enough to sample.
pub async fn sample_cpu() {
    // SAFETY: These are the S31 register blocks. The sampler only owns LP
    // TSENS temporarily and leaves existing clock and sensor enables intact.
    let clock = unsafe { &*esp32s31::LP_PERICLKRST::PTR }.tsens_ctrl();
    let control = unsafe { &*esp32s31::LP_TSENS::PTR }.ctrl();
    let (clock_enabled, power_enabled, sample_enabled) = critical_section::with(|_| {
        let clock_enabled = clock.read().lp_tsens_clk_en().bit();
        clock.modify(|_, w| w.lp_tsens_clk_en().set_bit());
        let state = control.read();
        let (power_enabled, sample_enabled) = (state.power_up().bit(), state.sample_en().bit());
        control.modify(|_, w| w.power_up().set_bit().sample_en().set_bit());
        (clock_enabled, power_enabled, sample_enabled)
    });
    // The IDF driver allows at least 200 µs for the converter to settle.
    Timer::after_millis(1).await;
    let reading = critical_section::with(|_| {
        let range = cpu_dac_range();
        let raw = control.read().out().bits();
        control.modify(|_, w| {
            w.power_up()
                .bit(power_enabled)
                .sample_en()
                .bit(sample_enabled)
        });
        clock.modify(|_, w| w.lp_tsens_clk_en().bit(clock_enabled));
        range.and_then(|range| cpu_degrees(raw, range))
    });
    set_cpu(reading);
}

/// Read only the SAR peripheral's TSENS DAC field. Follow the pinned HAL's
/// S31 analog master routing and restore the modem clock and selection fields.
fn cpu_dac_range() -> Option<u8> {
    const SAR_PERIF_MASK: u32 = 1 << 9;
    const BUSY: u32 = 1 << 25;
    const DAC_READ: u32 = (6 << 8) | 0x69;
    // SAFETY: callers hold the critical section. No analog data is written.
    unsafe {
        let bus = &*esp32s31::I2C_ANA_MST::PTR;
        let modem = &*esp32s31::MODEM_LPCON::PTR;
        let syscon = &*esp32s31::MODEM_SYSCON::PTR;
        let old_clk = modem.clk_conf().read().clk_i2c_mst_en().bit();
        let old_force = modem.clk_conf_force_on().read().clk_i2c_mst_fo().bit();
        let old_source = syscon.clk_conf().read().clk_i2c_mst_sel_160m().bit();
        syscon
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());
        modem
            .clk_conf_force_on()
            .modify(|_, w| w.clk_i2c_mst_fo().set_bit());
        modem.clk_conf().modify(|_, w| w.clk_i2c_mst_en().set_bit());
        let addr = esp32s31::I2C_ANA_MST::PTR.cast::<u32>();
        let master = usize::from(bus.ana_conf2().read().bits() & (SAR_PERIF_MASK << 4) == 0);
        let ctrl = addr.add(master);
        let reading = if (addr.read_volatile() | addr.add(1).read_volatile()) & BUSY == 0 {
            let old_select = bus.ana_conf1().read().bits();
            bus.ana_conf1()
                .write(|w| w.bits(!(SAR_PERIF_MASK << 2) & 0x00ff_ffff));
            ctrl.cast_mut().write_volatile(DAC_READ);
            let result = (0..256).find_map(|_| {
                let value = ctrl.read_volatile();
                (value & BUSY == 0).then_some(((value >> 16) & 15) as u8)
            });
            bus.ana_conf1().write(|w| w.bits(old_select));
            result
        } else {
            None
        };
        modem
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().bit(old_clk));
        modem
            .clk_conf_force_on()
            .modify(|_, w| w.clk_i2c_mst_fo().bit(old_force));
        syscon
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(old_source));
        reading
    }
}

fn cpu_degrees(raw: u8, dac: u8) -> Option<i16> {
    let (offset, min, max) = match dac {
        5 => (-2, 50, 125),
        7 => (-1, 20, 100),
        15 => (0, -10, 80),
        11 => (1, -30, 50),
        10 => (2, -40, 20),
        _ => return None,
    };
    // S31 EFUSE_RD_SYS_PART1_DATA4.TEMP_CALIBRATION (signed magnitude).
    // The older pinned PAC doesn't give the individual field a name.
    let efuse = unsafe {
        esp32s31::EFUSE::PTR
            .cast::<u32>()
            .add(0x78 / 4)
            .read_volatile()
    };
    let magnitude = (efuse & 0x1ff) as i32;
    let correction = if efuse & 0x200 != 0 {
        -magnitude
    } else {
        magnitude
    };
    let centidegrees = (4386 * i32::from(raw) - 278800 * offset - 205200) / 100 - correction * 10;
    let rounded = (centidegrees + centidegrees.signum() * 50) / 100;
    (min..=max).contains(&rounded).then_some(rounded as i16)
}
