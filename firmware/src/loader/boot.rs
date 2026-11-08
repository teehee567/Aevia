use esp_hal::{
    rtc_cntl::SocResetReason,
    time::Duration,
    timer::timg::{MwdtStage, MwdtStageAction, TimerGroupInstance, Wdt},
};

pub fn fault_reset() -> bool {
    !matches!(
        esp_hal::system::reset_reason(),
        Some(SocResetReason::ChipPowerOn | SocResetReason::CoreDeepSleep)
    )
}

pub fn watchdog<T: TimerGroupInstance>(mut watchdog: Wdt<T>) -> Wdt<T> {
    watchdog.set_timeout(MwdtStage::Stage0, Duration::from_secs(10));
    watchdog.set_stage_action(MwdtStage::Stage0, MwdtStageAction::ResetSystem);
    watchdog.enable();
    watchdog
}

pub fn enter_recovery() -> ! {
    esp_hal::system::software_reset()
}
