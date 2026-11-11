//! Latest receiver fix, independent of PPS association and estimator progress.
use super::super::interface::{GnssFixStatus, GnssPpsDiagnostic};
use super::navigation::ValidatedFields;
use aevia_device_hal::um980::{BestNavXyz, PpsStatus};

pub const MAX_FIX_AGE_US: u64 = 500_000;

#[derive(Clone, Copy, Default)]
struct FixState {
    value: GnssFixStatus,
    last_epoch_ms: Option<u64>,
    epoch_received_us: Option<u64>,
    epoch_output_delay_us: u64,
}

impl FixState {
    fn navigation(&mut self, message: BestNavXyz, received_us: u64) {
        let epoch = message.header.gps_epoch_ms();
        let forward = epoch.is_some_and(|epoch| self.last_epoch_ms.is_none_or(|last| epoch > last));
        if forward {
            self.last_epoch_ms = epoch;
            self.epoch_received_us = Some(received_us);
            self.epoch_output_delay_us = u64::from(message.header.output_delay_ms) * 1000;
        }
        let in_epoch = epoch.is_some() && epoch == self.last_epoch_ms;
        let prepared = ValidatedFields::prepare(&message);
        let position_valid = in_epoch
            && prepared
                .as_ref()
                .is_some_and(|fields| fields.position_valid);
        let velocity_valid = in_epoch
            && prepared
                .as_ref()
                .is_some_and(|fields| fields.velocity_valid);
        self.value = GnssFixStatus {
            qualified: false,
            received_us: Some(received_us),
            received_age_us: Some(0),
            gps_epoch_ms: epoch,
            time_reference: message.header.time_reference,
            time_status: message.header.time_status,
            output_delay_ms: message.header.output_delay_ms,
            position_status: message.position_status,
            position_type: message.position_type,
            velocity_status: message.velocity_status,
            velocity_type: message.velocity_type,
            satellites: message.satellites_used,
            position_valid,
            velocity_valid,
            position_ecef_m: position_valid.then_some(message.position_ecef_m),
            velocity_ecef_mps: velocity_valid.then_some(message.velocity_ecef_mps),
            position_sigma_m: position_valid.then_some(message.position_sigma_m),
            velocity_sigma_mps: velocity_valid.then_some(message.velocity_sigma_mps),
            position_age_us: None,
            velocity_age_us: None,
            solution_age_s: finite_age(message.solution_age_s),
            differential_age_s: finite_age(message.differential_age_s),
            velocity_latency_s: finite_age(message.velocity_latency_s),
            last_pps: self.value.last_pps,
        };
    }

    fn pps(
        &mut self,
        message: PpsStatus,
        received_us: u64,
        now_ns: Option<u64>,
        pulse_ns: Option<u64>,
    ) {
        self.value.last_pps = Some(GnssPpsDiagnostic {
            received_us,
            header_epoch_ms: message.header.gps_epoch_ms(),
            pulse_epoch_ms: message.gps_epoch_ms(),
            output_delay_ms: message.header.output_delay_ms,
            quality: message.quality,
            pulse_error_ns: message.pulse_error_ns,
            offset_time_10ps: message.offset_time_10ps,
            config: message.config,
            received_capture_ns: now_ns,
            latest_pulse_ns: pulse_ns,
        });
    }

    fn invalidate(&mut self) {
        self.last_epoch_ms = None;
        self.epoch_received_us = None;
        self.value.position_valid = false;
        self.value.velocity_valid = false;
        self.value.position_ecef_m = None;
        self.value.velocity_ecef_mps = None;
        self.value.position_sigma_m = None;
        self.value.velocity_sigma_mps = None;
    }

    fn at(&self, now_us: u64) -> GnssFixStatus {
        let mut value = self.value;
        value.received_age_us = value
            .received_us
            .and_then(|received| now_us.checked_sub(received));
        let current_epoch =
            value.gps_epoch_ms.is_some() && value.gps_epoch_ms == self.last_epoch_ms;
        let age = self
            .epoch_received_us
            .filter(|_| current_epoch)
            .and_then(|received| now_us.checked_sub(received))
            .and_then(|elapsed| elapsed.checked_add(self.epoch_output_delay_us));
        value.position_age_us = field_age(age, value.solution_age_s);
        value.velocity_age_us = field_age(age, value.velocity_latency_s);
        value.position_valid &= value.received_age_us.is_some()
            && value
                .position_age_us
                .is_some_and(|age| age <= MAX_FIX_AGE_US);
        value.velocity_valid &= value.received_age_us.is_some()
            && value
                .velocity_age_us
                .is_some_and(|age| age <= MAX_FIX_AGE_US);
        if !value.position_valid {
            value.position_ecef_m = None;
            value.position_sigma_m = None;
        }
        if !value.velocity_valid {
            value.velocity_ecef_mps = None;
            value.velocity_sigma_mps = None;
        }
        value
    }
}

fn finite_age(value: f32) -> Option<f32> {
    (value.is_finite() && value >= 0.0).then_some(value)
}

fn field_age(base_us: Option<u64>, seconds: Option<f32>) -> Option<u64> {
    let micros = f64::from(seconds?) * 1_000_000.0;
    if micros >= u64::MAX as f64 {
        return None;
    }
    base_us?.checked_add(micros as u64)
}

#[cfg(target_arch = "riscv32")]
mod shared {
    use super::*;
    use core::cell::Cell;
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
    static FIX: Mutex<CriticalSectionRawMutex, Cell<Option<FixState>>> =
        Mutex::new(Cell::new(None));

    fn update(action: impl FnOnce(&mut FixState)) {
        // The GNSS task is the sole writer. Keep validation and software-f64
        // arithmetic preemptible by the IMU executor; only copy publication
        // state under the cross-core critical section.
        let mut state = FIX.lock(Cell::get).unwrap_or_default();
        action(&mut state);
        FIX.lock(|cell| cell.set(Some(state)));
    }

    pub(crate) fn navigation(message: BestNavXyz, received_us: u64) {
        update(|state| state.navigation(message, received_us));
    }

    pub(crate) fn pps(
        message: PpsStatus,
        received_us: u64,
        now_ns: Option<u64>,
        pulse_ns: Option<u64>,
    ) {
        update(|state| state.pps(message, received_us, now_ns, pulse_ns));
    }

    pub(crate) fn invalidate() {
        update(FixState::invalidate);
    }

    pub(crate) fn status() -> GnssFixStatus {
        let state = FIX.lock(Cell::get).unwrap_or_default();
        state.at(embassy_time::Instant::now().as_micros())
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) use shared::{invalidate, navigation, pps, status};

#[cfg(test)]
mod tests;
