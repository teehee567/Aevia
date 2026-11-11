//! Watchdog eligibility follows task progress, not satellite lock or valid data.
//! A disconnected sensor may retry indefinitely; a wedged task must not be kept
//! alive by an otherwise responsive USB connection.

const STARTUP_GRACE_US: u64 = 30_000_000;
const PROGRESS_TIMEOUT_US: u64 = 5_000_000;
const CONFIRMATION_WINDOW_US: u64 = 5_000_000;
const CONFIRMATION_INPUT_TIMEOUT_US: u64 = 1_000_000;

#[derive(Clone, Copy, Default)]
pub(crate) struct Readings {
    pub worker: u32,
    /// PPS/capture maintenance task, independent of receiver decoding.
    pub acquisition: u32,
    /// Progress of the receiver owner, including bounded configuration/retries.
    pub gnss_owner: u32,
    /// Progress of whichever task currently owns IMU initialization/sampling.
    /// This advances during bounded retries, independently of valid samples.
    pub imu_owner: u32,
    /// Coherent, timing-validated raw IMU samples admitted by acquisition.
    pub imu_samples: u32,
    pub capture_errors: u32,
    pub recording_required: bool,
    pub recording_ready: bool,
}

pub(crate) struct Monitor {
    started_us: u64,
    worker_changed_us: u64,
    acquisition_changed_us: u64,
    gnss_changed_us: u64,
    imu_owner_changed_us: u64,
    imu_changed_us: u64,
    previous: Readings,
    worker_seen: bool,
    acquisition_seen: bool,
    gnss_seen: bool,
    imu_owner_seen: bool,
    imu_seen: bool,
    ready_since_us: Option<u64>,
    failed: bool,
}

impl Monitor {
    pub(crate) fn new(now_us: u64) -> Self {
        Self {
            started_us: now_us,
            worker_changed_us: now_us,
            acquisition_changed_us: now_us,
            gnss_changed_us: now_us,
            imu_owner_changed_us: now_us,
            imu_changed_us: now_us,
            previous: Readings::default(),
            worker_seen: false,
            acquisition_seen: false,
            gnss_seen: false,
            imu_owner_seen: false,
            imu_seen: false,
            ready_since_us: None,
            failed: false,
        }
    }

    pub(crate) fn healthy(
        &mut self,
        now_us: u64,
        application: bool,
        startup_stage: u8,
        readings: Readings,
    ) -> bool {
        if readings.worker != self.previous.worker {
            self.worker_changed_us = now_us;
            self.worker_seen = true;
        }
        if readings.acquisition != self.previous.acquisition {
            self.acquisition_changed_us = now_us;
            self.acquisition_seen = true;
        }
        if readings.gnss_owner != self.previous.gnss_owner {
            self.gnss_changed_us = now_us;
            self.gnss_seen = true;
        }
        if readings.imu_owner != self.previous.imu_owner {
            self.imu_owner_changed_us = now_us;
            self.imu_owner_seen = true;
        }
        if readings.imu_samples != self.previous.imu_samples {
            self.imu_changed_us = now_us;
            self.imu_seen = true;
        }
        self.previous = readings;
        // Confirmation protects against a functionally broken acquisition
        // image. PPS maintenance and a live idle worker are insufficient:
        // the fitted IMU must produce validated samples and the GNSS owner
        // itself must progress. Use the tighter confirmation timeout for every
        // owner: the five-second watchdog tolerance equals the confirmation
        // window and would otherwise bless a task with only one heartbeat.
        // A GNSS fix/PPS lock is not required.
        if application
            && self.ready(now_us, startup_stage, readings)
            && (!readings.recording_required || readings.recording_ready)
            && self.gnss_seen
            && self.imu_seen
            && now_us.saturating_sub(self.worker_changed_us) <= CONFIRMATION_INPUT_TIMEOUT_US
            && now_us.saturating_sub(self.acquisition_changed_us) <= CONFIRMATION_INPUT_TIMEOUT_US
            && now_us.saturating_sub(self.gnss_changed_us) <= CONFIRMATION_INPUT_TIMEOUT_US
            && now_us.saturating_sub(self.imu_owner_changed_us) <= CONFIRMATION_INPUT_TIMEOUT_US
            && now_us.saturating_sub(self.imu_changed_us) <= CONFIRMATION_INPUT_TIMEOUT_US
        {
            self.ready_since_us.get_or_insert(now_us);
        } else {
            self.ready_since_us = None;
        }
        // Recovery has no acquisition/estimator tasks and must remain usable
        // during a long upload. Its USB owner still has to feed the watchdog.
        if !application {
            return true;
        }
        if self.failed {
            return false;
        }
        if now_us.saturating_sub(self.started_us) < STARTUP_GRACE_US {
            return true;
        }
        let progressing = self.ready(now_us, startup_stage, readings);
        self.failed = !progressing;
        progressing
    }

    /// Watchdog eligibility tolerates sensor failure while the owning tasks
    /// retry. Trial confirmation separately requires successful IMU data.
    pub(crate) fn ready(&self, now_us: u64, startup_stage: u8, readings: Readings) -> bool {
        startup_stage == 10
            && !self.failed
            && readings.capture_errors == 0
            && self.worker_seen
            && self.acquisition_seen
            && self.gnss_seen
            && self.imu_owner_seen
            && now_us.saturating_sub(self.worker_changed_us) <= PROGRESS_TIMEOUT_US
            && now_us.saturating_sub(self.acquisition_changed_us) <= PROGRESS_TIMEOUT_US
            && now_us.saturating_sub(self.gnss_changed_us) <= PROGRESS_TIMEOUT_US
            && now_us.saturating_sub(self.imu_owner_changed_us) <= PROGRESS_TIMEOUT_US
    }

    pub(crate) fn confirmable(&self, now_us: u64) -> bool {
        !self.failed
            && self
                .ready_since_us
                .is_some_and(|since| now_us.saturating_sub(since) >= CONFIRMATION_WINDOW_US)
    }
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::Readings;
    use core::sync::atomic::{AtomicU32, Ordering};

    static WORKER: AtomicU32 = AtomicU32::new(0);
    static ACQUISITION: AtomicU32 = AtomicU32::new(0);
    static GNSS_OWNER: AtomicU32 = AtomicU32::new(0);
    static IMU_OWNER: AtomicU32 = AtomicU32::new(0);

    pub(crate) fn worker_heartbeat() {
        WORKER.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn acquisition_heartbeat() {
        ACQUISITION.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn gnss_heartbeat() {
        GNSS_OWNER.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn imu_heartbeat() {
        IMU_OWNER.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn readings() -> Readings {
        Readings {
            worker: WORKER.load(Ordering::Relaxed),
            acquisition: ACQUISITION.load(Ordering::Relaxed),
            gnss_owner: GNSS_OWNER.load(Ordering::Relaxed),
            imu_owner: IMU_OWNER.load(Ordering::Relaxed),
            imu_samples: super::super::sensors::IMU_SAMPLES.load(Ordering::Relaxed),
            capture_errors: super::super::platform::capture::ERRORS.load(Ordering::Relaxed),
            recording_required: super::super::configuration::DATA_COLLECTION_ONLY,
            recording_ready: super::super::recording::confirmation_ready(),
        }
    }
}
#[cfg(target_arch = "riscv32")]
pub(crate) use hardware::{
    acquisition_heartbeat, gnss_heartbeat, imu_heartbeat, readings, worker_heartbeat,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stalled_worker_cannot_be_kept_alive_by_usb_or_acquisition() {
        let mut monitor = Monitor::new(0);
        for second in 0..30 {
            assert!(monitor.healthy(
                second * 1_000_000,
                true,
                10,
                Readings {
                    worker: 1,
                    acquisition: second as u32 + 1,
                    gnss_owner: second as u32 + 1,
                    imu_owner: second as u32 + 1,
                    capture_errors: 0,
                    ..Readings::default()
                }
            ));
        }
        assert!(!monitor.healthy(
            30_000_000,
            true,
            10,
            Readings {
                worker: 1,
                acquisition: 31,
                gnss_owner: 31,
                imu_owner: 31,
                capture_errors: 0,
                ..Readings::default()
            }
        ));
        // Once unhealthy, a transient restart cannot conceal the fault before
        // the hardware watchdog resets into recovery.
        assert!(!monitor.healthy(
            31_000_000,
            true,
            10,
            Readings {
                worker: 2,
                acquisition: 32,
                gnss_owner: 32,
                imu_owner: 32,
                capture_errors: 0,
                ..Readings::default()
            }
        ));
    }

    #[test]
    fn no_sensor_or_no_fix_does_not_fail_a_live_pipeline() {
        let mut monitor = Monitor::new(0);
        for second in 0..120 {
            let readings = Readings {
                worker: second as u32 + 1,
                acquisition: second as u32 + 1,
                gnss_owner: second as u32 + 1,
                imu_owner: second as u32 + 1,
                capture_errors: 0,
                ..Readings::default()
            };
            assert!(monitor.healthy(second * 1_000_000, true, 10, readings));
        }
    }

    #[test]
    fn startup_is_bounded_and_failed_capture_cannot_be_confirmed() {
        let mut monitor = Monitor::new(0);
        assert!(monitor.healthy(STARTUP_GRACE_US - 1, true, 5, Readings::default()));
        assert!(!monitor.healthy(STARTUP_GRACE_US, true, 5, Readings::default()));
        let mut monitor = Monitor::new(0);
        let broken = Readings {
            worker: 1,
            acquisition: 1,
            gnss_owner: 1,
            imu_owner: 1,
            capture_errors: 1,
            ..Readings::default()
        };
        assert!(!monitor.ready(0, 10, broken));
        assert!(!monitor.healthy(STARTUP_GRACE_US, true, 10, broken));
    }

    #[test]
    fn acquisition_stall_is_detected_and_recovery_has_no_sensor_requirement() {
        let mut monitor = Monitor::new(0);
        let readings = Readings {
            worker: 1,
            acquisition: 1,
            gnss_owner: 1,
            imu_owner: 1,
            capture_errors: 0,
            ..Readings::default()
        };
        assert!(monitor.healthy(1_000_000, true, 10, readings));
        assert!(monitor.ready(1_000_000, 10, readings));
        assert!(!monitor.healthy(
            STARTUP_GRACE_US,
            true,
            10,
            Readings {
                worker: 2,
                ..readings
            }
        ));
        assert!(monitor.healthy(90_000_000, false, 0, Readings::default()));
    }

    #[test]
    fn wrapping_heartbeat_is_progress_even_when_the_counter_becomes_zero() {
        let mut monitor = Monitor::new(0);
        assert!(monitor.healthy(
            29_000_000,
            true,
            10,
            Readings {
                worker: u32::MAX,
                acquisition: u32::MAX,
                gnss_owner: u32::MAX,
                imu_owner: u32::MAX,
                capture_errors: 0,
                ..Readings::default()
            }
        ));
        assert!(monitor.healthy(30_000_000, true, 10, Readings::default()));
        assert!(monitor.ready(30_000_000, 10, Readings::default()));
    }

    #[test]
    fn collection_trial_requires_persisted_data_but_bad_card_keeps_usb_alive() {
        let mut monitor = Monitor::new(0);
        for second in 0..=40_u32 {
            let readings = Readings {
                worker: second + 1,
                acquisition: second + 1,
                gnss_owner: second + 1,
                imu_owner: second + 1,
                imu_samples: second + 1,
                recording_required: true,
                recording_ready: second >= 35,
                ..Readings::default()
            };
            let now = u64::from(second) * 1_000_000;
            assert!(monitor.healthy(now, true, 10, readings));
            assert_eq!(monitor.confirmable(now), second == 40);
        }
    }

    #[test]
    fn confirmation_requires_sustained_progress_after_startup() {
        let mut monitor = Monitor::new(0);
        assert!(!monitor.confirmable(0));
        for second in 0..=5 {
            let readings = Readings {
                worker: second + 1,
                acquisition: second + 1,
                gnss_owner: second + 1,
                imu_owner: second + 1,
                imu_samples: second + 1,
                capture_errors: 0,
                ..Readings::default()
            };
            assert!(monitor.healthy(u64::from(second) * 1_000_000, true, 10, readings));
            assert_eq!(
                monitor.confirmable(u64::from(second) * 1_000_000),
                second == 5
            );
        }
        assert!(monitor.healthy(6_000_000, true, 11, Readings::default()));
        assert!(!monitor.confirmable(6_000_000));
    }

    #[test]
    fn missing_or_stalled_imu_data_never_confirms_but_retry_tasks_stay_alive() {
        for missing in 0..2 {
            let mut monitor = Monitor::new(0);
            for second in 0..60_u32 {
                let readings = Readings {
                    worker: second + 1,
                    acquisition: second + 1,
                    gnss_owner: second + 1,
                    imu_owner: second + 1,
                    imu_samples: if missing == 0 { 0 } else { 1 },
                    capture_errors: 0,
                    ..Readings::default()
                };
                let now = u64::from(second) * 1_000_000;
                assert!(monitor.healthy(now, true, 10, readings));
                assert!(!monitor.confirmable(now));
            }
        }
    }

    #[test]
    fn one_initial_heartbeat_cannot_confirm_a_stalled_task_at_the_watchdog_boundary() {
        for stalled in 0..4 {
            let mut monitor = Monitor::new(0);
            for second in 0..=10_u32 {
                let count = second + 1;
                let readings = Readings {
                    worker: if stalled == 0 { 1 } else { count },
                    acquisition: if stalled == 1 { 1 } else { count },
                    gnss_owner: if stalled == 2 { 1 } else { count },
                    imu_owner: if stalled == 3 { 1 } else { count },
                    imu_samples: count,
                    capture_errors: 0,
                    ..Readings::default()
                };
                let now = u64::from(second) * 1_000_000;
                // Startup retry tolerance must not bless a wedged candidate.
                assert!(monitor.healthy(now, true, 10, readings));
                assert!(
                    !monitor.confirmable(now),
                    "owner {stalled}, second {second}"
                );
            }
        }
    }

    #[test]
    fn confirmation_restarts_after_sample_loss_and_preserves_counter_wrap_progress() {
        let mut monitor = Monitor::new(0);
        for second in 0..=13_u32 {
            // Sampling stops after second5 and resumes at second8. All
            // unrelated tasks keep running, so watchdog liveness stays valid.
            let count = if (6..8).contains(&second) { 5 } else { second };
            let readings = Readings {
                worker: second + 1,
                acquisition: second + 1,
                gnss_owner: second + 1,
                imu_owner: second + 1,
                imu_samples: (u32::MAX - 2).wrapping_add(count),
                capture_errors: 0,
                ..Readings::default()
            };
            let now = u64::from(second) * 1_000_000;
            assert!(monitor.healthy(now, true, 10, readings));
            assert_eq!(monitor.confirmable(now), matches!(second, 5 | 6 | 13));
        }
    }

    #[test]
    fn independent_sensor_owner_stalls_cannot_be_hidden_by_live_worker_and_capture() {
        for stalled in 0..2 {
            let mut monitor = Monitor::new(0);
            for second in 0..=STARTUP_GRACE_US / 1_000_000 {
                let count = second as u32 + 1;
                let readings = Readings {
                    worker: count,
                    acquisition: count,
                    gnss_owner: if stalled == 0 { 1 } else { count },
                    imu_owner: if stalled == 1 { 1 } else { count },
                    imu_samples: count,
                    capture_errors: 0,
                    ..Readings::default()
                };
                let now = second * 1_000_000;
                assert_eq!(
                    monitor.healthy(now, true, 10, readings),
                    now < STARTUP_GRACE_US,
                );
                if now > PROGRESS_TIMEOUT_US {
                    assert!(!monitor.confirmable(now));
                }
            }
        }
    }
}
