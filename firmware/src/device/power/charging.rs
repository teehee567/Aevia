//! Conservative supervised single-cell charging. TI BQ25622E SLUSFA3C,
//! sections 8.3.5 and 8.6.2: limits are absolute mA/mV register encodings.
//! Never write input limits, BATFET/HiZ controls, NTC configuration or timers.
use super::*;
use crate::device::configuration;
use aevia_device_hal::bq25622::WordRegister;

const WORDS: [WordRegister; 7] = [
    WordRegister::ChargeCurrent,
    WordRegister::ChargeVoltage,
    WordRegister::InputCurrent,
    WordRegister::InputVoltage,
    WordRegister::MinimumSystemVoltage,
    WordRegister::PrechargeCurrent,
    WordRegister::TerminationCurrent,
];
const BYTES: [ByteRegister; 6] = [
    ByteRegister::ChargeControl0,
    ByteRegister::ChargeTimer,
    ByteRegister::ChargerControl1,
    ByteRegister::NtcControl0,
    ByteRegister::NtcControl1,
    ByteRegister::NtcControl2,
];
const LIMITS: [(usize, u16, u16); 4] = [
    (
        0,
        configuration::CHARGE_CURRENT_MA,
        (configuration::CHARGE_CURRENT_MA / 80) << 6,
    ),
    (
        1,
        configuration::CHARGE_VOLTAGE_MV,
        (configuration::CHARGE_VOLTAGE_MV / 10) << 3,
    ),
    (
        5,
        configuration::PRECHARGE_CURRENT_MA,
        (configuration::PRECHARGE_CURRENT_MA / 20) << 4,
    ),
    (
        6,
        configuration::TERMINATION_CURRENT_MA,
        (configuration::TERMINATION_CURRENT_MA / 10) << 3,
    ),
];

#[derive(Clone, Copy)]
struct Configuration {
    words: [u16; 7],
    bytes: [u8; 6],
}
impl Configuration {
    fn limits_match(self) -> bool {
        LIMITS.iter().all(|&(i, _, raw)| self.words[i] == raw)
    }

    fn protection_mismatch(self) -> Option<(ByteRegister, u8, u8)> {
        // 40 mA termination is invalid with Q4_FULLON; keep the power-path bit
        // untouched and decline charging. Termination and safety timers must
        // already be active. Never clear TS_IGNORE to conceal a changed setup.
        if self.bytes[0] & 0x44 != 0x04 {
            Some((BYTES[0], 0x04, self.bytes[0] & 0x44))
        } else if self.bytes[1] & 0x04 == 0 {
            Some((BYTES[1], 0x04, self.bytes[1] & 0x04))
        } else if self.bytes[3] & 0x80 != 0 {
            Some((BYTES[3], 0, self.bytes[3] & 0x80))
        } else {
            None
        }
    }

    fn preserved_mismatch(self, before: Self) -> Option<u8> {
        for i in [2, 3, 4] {
            if self.words[i] != before.words[i] {
                return Some(WORDS[i] as u8);
            }
        }
        for i in [0, 1, 3, 4, 5] {
            if self.bytes[i] != before.bytes[i] {
                return Some(BYTES[i] as u8);
            }
        }
        if self.bytes[2] & 0xd8 != before.bytes[2] & 0xd8 {
            return Some(BYTES[2] as u8);
        }
        None
    }
}

fn read_configuration<B: I2c>(
    charger: &mut Bq25622<B>,
    status: &mut PowerChargeStatus,
) -> Result<Configuration, ChargerError<B::Error>> {
    let mut config = Configuration {
        words: [0; 7],
        bytes: [0; 6],
    };
    for (i, register) in WORDS.into_iter().enumerate() {
        let raw = charger.read_word(register)?;
        config.words[i] = raw;
        match register {
            WordRegister::ChargeCurrent => {
                status.charge_current_ma = Some(((raw >> 6) & 0x3f) * 80)
            }
            WordRegister::ChargeVoltage => {
                status.charge_voltage_mv = Some(((raw >> 3) & 0x1ff) * 10)
            }
            WordRegister::InputCurrent => {
                status.input_current_limit_ma = Some(((raw >> 4) & 0xff) * 20)
            }
            WordRegister::PrechargeCurrent => {
                status.precharge_current_ma = Some(((raw >> 4) & 0x1f) * 20)
            }
            WordRegister::TerminationCurrent => {
                status.termination_current_ma = Some(((raw >> 3) & 0x3f) * 10)
            }
            _ => {}
        }
    }
    for (i, register) in BYTES.into_iter().enumerate() {
        let raw = charger.read_byte(register)?;
        config.bytes[i] = raw;
        match register {
            ByteRegister::ChargeControl0 => status.charger_control0 = Some(raw),
            ByteRegister::ChargerControl1 => {
                status.charger_control1 = Some(raw);
                status.charging_enabled = Some(raw & 0x20 != 0);
            }
            ByteRegister::NtcControl0 => status.ntc_control0 = Some(raw),
            _ => {}
        }
    }
    Ok(config)
}

fn mismatch(register: u8, expected: Option<u8>, actual: Option<u8>, at_us: u64) -> PowerFailure {
    PowerFailure {
        stage: 6,
        error_kind: 8,
        register: Some(register),
        operation: 1,
        expected,
        actual,
        at_us,
        ..Default::default()
    }
}

/// A timestamp is retained for diagnosis, but stale or future readbacks cannot
/// be mistaken for the current charging state.
pub(super) fn fresh_status(status: PowerChargeStatus, now_us: u64) -> PowerChargeStatus {
    if status
        .updated_us
        .and_then(|at| now_us.checked_sub(at))
        .is_some_and(|age| age <= 3_000_000)
    {
        status
    } else {
        PowerChargeStatus {
            updated_us: status.updated_us,
            ..Default::default()
        }
    }
}

pub(super) fn enforce<B: I2c>(
    bus: B,
    previous_errors: u32,
    diagnostics: &mut PowerDiagnostics,
    now: impl Fn() -> u64,
) -> (PowerStatus, PowerChargeStatus)
where
    B::Error: DiagnosticError,
{
    diagnostics.attempts = diagnostics.attempts.saturating_add(1);
    let transaction = Cell::new(Transaction::default());
    let mut charger = Bq25622::new(
        ObservedBus {
            bus,
            last: &transaction,
        },
        Variant::Bq25622E,
    );
    let mut status = PowerStatus::default();
    let mut charge = PowerChargeStatus::default();
    let mut stages = 0;
    let mut before_control = None;
    macro_rules! attempt {
        ($stage:expr, $expression:expr) => {
            $expression.map_err(|error| {
                retain(
                    diagnostics,
                    &mut stages,
                    failure($stage, &error, transaction.get(), now()),
                );
            })
        };
    }
    let verified = (|| -> Result<(), ()> {
        // Check silicon before any possible write, including failure shutdown.
        let identity = charger.verify_identity();
        if let Err(ChargerError::Identity { found, .. }) = &identity {
            status.part_info = Some(found.raw);
        }
        status.part_info = Some(attempt!(1, identity)?.raw);
        let before = attempt!(2, read_configuration(&mut charger, &mut charge))?;
        before_control = Some(before.bytes[2]);
        if let Some((reg, expected, actual)) = before.protection_mismatch() {
            retain(
                diagnostics,
                &mut stages,
                mismatch(reg as u8, Some(expected), Some(actual), now()),
            );
            return Err(());
        }
        let expected_control = charging_disabled(before.bytes[2]) | 0x20;
        if !before.limits_match() || before.bytes[2] & 0x03 != 0 {
            // Disable and verify before programming. No current/voltage write
            // is allowed while EN_CHG is observed set. An active watchdog can
            // restore defaults after the initial reads; disable it and repeat
            // all readbacks even when those first limits already matched.
            attempt!(3, charger.disable_charging_and_watchdog())?;
            for (i, value, raw) in LIMITS {
                if before.words[i] != raw {
                    attempt!(3, charger.set_limit(WORDS[i], value))?;
                }
            }
            // Re-read every limit plus preserved controls before enabling. A
            // partial/ignored write cannot leave an unverified profile enabled.
            charge = PowerChargeStatus::default();
            let after = attempt!(4, read_configuration(&mut charger, &mut charge))?;
            if let Some(reg) = after.preserved_mismatch(before) {
                retain(diagnostics, &mut stages, mismatch(reg, None, None, now()));
                return Err(());
            }
            if let Some(&(i, _, _)) = LIMITS.iter().find(|&&(i, _, raw)| after.words[i] != raw) {
                retain(
                    diagnostics,
                    &mut stages,
                    mismatch(WORDS[i] as u8, None, None, now()),
                );
                return Err(());
            }
        }
        // Do not restart the charge cycle on the once-per-second policy poll.
        if charge.charger_control1 != Some(expected_control) {
            attempt!(
                3,
                charger.update_byte(ByteRegister::ChargerControl1, 0x23, 0x20)
            )?;
        }
        let actual = attempt!(4, charger.read_byte(ByteRegister::ChargerControl1))?;
        charge.charger_control1 = Some(actual);
        charge.charging_enabled = Some(actual & 0x20 != 0);
        status.control = Some(actual);
        if actual != expected_control {
            retain(
                diagnostics,
                &mut stages,
                mismatch(
                    ByteRegister::ChargerControl1 as u8,
                    Some(expected_control),
                    Some(actual),
                    now(),
                ),
            );
            return Err(());
        }
        let actual = attempt!(5, charger.status())?;
        charge.charger_status0 = Some(actual.status0);
        charge.charger_status1 = Some(actual.status1);
        charge.fault_status = Some(actual.faults);
        status.charger_status = Some(actual.status1);
        Ok(())
    })()
    .is_ok();

    if !verified {
        // Best effort on every failed attempt, retaining the first cause. HAL
        // still refuses writes if silicon identity cannot be verified. No
        // BATFET, discharge, input-limit or high-impedance action is issued.
        let _ = attempt!(
            3,
            charger.update_byte(ByteRegister::ChargerControl1, 0x23, 0)
        );
        charge.charger_control1 =
            attempt!(4, charger.read_byte(ByteRegister::ChargerControl1)).ok();
        charge.charging_enabled = charge.charger_control1.map(|value| value & 0x20 != 0);
        status.control = charge.charger_control1;
        charge.charger_status0 = None;
        charge.charger_status1 = None;
        charge.fault_status = None;
        if let Ok(actual) = attempt!(5, charger.status()) {
            charge.charger_status0 = Some(actual.status0);
            charge.charger_status1 = Some(actual.status1);
            charge.fault_status = Some(actual.faults);
        }
        status.charger_status = charge.charger_status1;
    }
    status.verified = verified;
    status.errors = previous_errors.saturating_add(u32::from(!verified));
    charge.verified = verified;
    charge.updated_us = Some(now());
    if verified {
        diagnostics.successes = diagnostics.successes.saturating_add(1);
        diagnostics.last_success_us = charge.updated_us;
    } else {
        diagnostics.last_failure_stages = stages;
        diagnostics.failed_part_info = status.part_info;
        diagnostics.failed_pre_control = before_control;
        diagnostics.failed_post_control = status.control;
        diagnostics.failed_charger_status = status.charger_status;
    }
    (status, charge)
}

#[cfg(test)]
mod tests;
