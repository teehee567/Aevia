// watchdog settings take effect after an explicit commit

use esp_hal::{
    peripherals::{TIMG0, TIMG1},
    time::Duration,
    timer::timg::{MwdtStage, TimerGroupInstance, Wdt},
};

pub fn disable_boot_watchdogs(_timer0: &mut TIMG0<'_>, _timer1: &mut TIMG1<'_>) {
    // apply the disable settings written by hal startup while both timer groups are borrowed
    commit::<TIMG0<'_>>();
    commit::<TIMG1<'_>>();
}

pub fn start(watchdog: &mut Wdt<TIMG0<'_>>) {
    let timeout = Duration::from_secs(10);
    watchdog.set_timeout(MwdtStage::Stage0, timeout);

    // enabling uses a system reset and clears the later stages and flash boot mode
    watchdog.enable();
    commit::<TIMG0<'_>>();
    watchdog.feed();
}

fn commit<T: TimerGroupInstance>() {
    // each caller holds the timer group or its watchdog while these registers are updated
    let registers = unsafe { &*T::register_block() };

    registers
        .wdtwprotect()
        .write(|writer| unsafe { writer.wdt_wkey().bits(0x50d8_3aa1) });

    registers
        .wdtconfig0()
        .modify(|_, writer| writer.wdt_conf_update_en().set_bit());

    registers
        .wdtwprotect()
        .write(|writer| unsafe { writer.wdt_wkey().bits(0) });
}
