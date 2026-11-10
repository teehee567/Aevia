//! SCH16T acquisition policy. SafeSPI, startup checks and physical scaling live
//! in the device HAL; this module associates coherent bursts with hardware edges
//! and aligns their interval support for the estimator.
use aevia_device_hal::sch16t::{
    AxisDecimation, AxisFilters, Config, Decimation, Filter, MotionSample, UserInterface,
};

const GYRO_DELAY_NS: u64 = 1_400_000;
const ACCEL_DELAY_NS: u64 = 1_500_000;
#[cfg(target_arch = "riscv32")]
pub(super) const DELAY_UNCERTAINTY_NS: u64 = 1_200_000;
// DEC4's earliest update is ~637 us. Leave margin before that next update.
const READ_DEADLINE_NS: u64 = 600_000;

/// Last driver failure is retained through later success. Kinds: 0 none,
/// 1 SPI, 2 framing, 3 configuration, 4 identity, 5 readback, 6 startup status,
/// 7 sensor status, 8 other driver state. `actual` carries the framing/status
/// subcode for kinds 2/7; register/status values are otherwise unmodified.
pub use super::interface::ImuDiagnostics;

impl ImuDiagnostics {
    fn with_error<E>(self, error: &aevia_device_hal::sch16t::Error<E>) -> Self {
        self.with_error_at(error, None)
    }

    fn with_error_at<E>(
        self,
        error: &aevia_device_hal::sch16t::Error<E>,
        captured_ns: Option<u64>,
    ) -> Self {
        use aevia_device_hal::sch16t::Error as DriverError;
        let mut next = Self {
            ready: false,
            error_kind: 0,
            register: None,
            expected: None,
            actual: None,
            component: None,
            asic: None,
            status_registers: None,
            status_frame_faults: 0,
            driver_errors: self.driver_errors.saturating_add(1),
            ..self
        };
        match error {
            DriverError::Spi(_) => next.error_kind = 1,
            DriverError::Frame(error) => {
                next.error_kind = 2;
                next.actual = Some(*error as u16);
            }
            DriverError::Configuration(_) => next.error_kind = 3,
            DriverError::Identity { component, asic } => {
                next.error_kind = 4;
                next.component = Some(*component);
                next.asic = Some(*asic);
            }
            DriverError::Readback {
                register,
                expected,
                actual,
            } => {
                next.error_kind = 5;
                next.register = Some(register.address());
                next.expected = Some(*expected);
                next.actual = Some(*actual);
            }
            DriverError::StartupStatus(status) => {
                next.error_kind = 6;
                next.status_registers = Some(status.registers);
                next.status_frame_faults = status.frame_faults;
            }
            DriverError::SensorStatus {
                register,
                status,
                common_error,
                payload,
                counter,
            } => {
                next.error_kind = 7;
                next.register = Some(register.address());
                next.actual = Some((*status as u16) << 1 | u16::from(*common_error));
                next.last_sensor_status_at_ns = captured_ns;
                next.last_sensor_status_register = Some(register.address());
                next.last_sensor_status_flags =
                    Some((*status as u8) << 1 | u8::from(*common_error));
                next.last_sensor_status_payload = Some(*payload);
                next.last_sensor_status_counter = Some(*counter);
            }
            _ => next.error_kind = 8,
        }
        next
    }

    fn with_numeric_range(self, motion: &MotionSample, captured_ns: u64) -> Self {
        Self {
            numeric_range_errors: self.numeric_range_errors.saturating_add(1),
            last_numeric_range_at_ns: Some(captured_ns),
            last_numeric_rate_abs_dps: Some(motion.angular_rate_dps.map(f64::abs)),
            last_numeric_accel_abs_mps2: Some(motion.acceleration_mps2.map(f64::abs)),
            ..self
        }
    }
}

#[cfg(target_arch = "riscv32")]
static DIAGNOSTICS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<Option<ImuDiagnostics>>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(None));
#[cfg(target_arch = "riscv32")]
static READY_STATE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(target_arch = "riscv32")]
pub(crate) fn diagnostics() -> ImuDiagnostics {
    let mut value = super::platform::imu_spi_diagnostics(
        DIAGNOSTICS.lock(core::cell::Cell::get).unwrap_or_default(),
    );
    value.ready = READY_STATE.load(core::sync::atomic::Ordering::Relaxed);
    read_timing::diagnostics(value)
}

#[cfg(target_arch = "riscv32")]
mod read_timing {
    use super::{ImuDiagnostics, READ_DEADLINE_NS};
    use core::sync::atomic::{AtomicU32, Ordering};
    static LATE_STARTS: AtomicU32 = AtomicU32::new(0);
    static LATE_FINISHES: AtomicU32 = AtomicU32::new(0);
    static OVERWRITTEN: AtomicU32 = AtomicU32::new(0);
    static MAX_START: AtomicU32 = AtomicU32::new(0);
    static MAX_BURST: AtomicU32 = AtomicU32::new(0);
    static LAST_START: AtomicU32 = AtomicU32::new(0);
    static LAST_BURST: AtomicU32 = AtomicU32::new(0);
    pub(super) fn observe(edge: u64, started: u64, finished: u64, latest: Option<u64>) {
        let start = (started.saturating_sub(edge) / 1000).min(u64::from(u32::MAX)) as u32;
        let burst = (finished.saturating_sub(started) / 1000).min(u64::from(u32::MAX)) as u32;
        LAST_START.store(start, Ordering::Relaxed);
        LAST_BURST.store(burst, Ordering::Relaxed);
        MAX_START.fetch_max(start, Ordering::Relaxed);
        MAX_BURST.fetch_max(burst, Ordering::Relaxed);
        if started.saturating_sub(edge) > READ_DEADLINE_NS {
            LATE_STARTS.fetch_add(1, Ordering::Relaxed);
        }
        if finished.saturating_sub(edge) > READ_DEADLINE_NS {
            LATE_FINISHES.fetch_add(1, Ordering::Relaxed);
        }
        if latest != Some(edge) {
            OVERWRITTEN.fetch_add(1, Ordering::Relaxed);
        }
    }
    pub(super) fn diagnostics(mut value: ImuDiagnostics) -> ImuDiagnostics {
        value.timing_late_starts = LATE_STARTS.load(Ordering::Relaxed);
        value.timing_late_finishes = LATE_FINISHES.load(Ordering::Relaxed);
        value.timing_overwritten = OVERWRITTEN.load(Ordering::Relaxed);
        value.max_start_latency_us = MAX_START.load(Ordering::Relaxed);
        value.max_burst_us = MAX_BURST.load(Ordering::Relaxed);
        value.last_start_latency_us = LAST_START.load(Ordering::Relaxed);
        value.last_burst_us = LAST_BURST.load(Ordering::Relaxed);
        value
    }
}

#[cfg(target_arch = "riscv32")]
fn publish_diagnostics(value: ImuDiagnostics) {
    READY_STATE.store(value.ready, core::sync::atomic::Ordering::Relaxed);
    DIAGNOSTICS.lock(|current| current.set(Some(value)));
}

fn configuration() -> Config {
    Config {
        rate_filters: AxisFilters::all(Filter::Lpf3),
        acc_filters: AxisFilters::all(Filter::Lpf3),
        aux_acc_filters: AxisFilters::all(Filter::Lpf3),
        rate_decimation: AxisDecimation::all(Decimation::Dec4),
        acc_decimation: AxisDecimation::all(Decimation::Dec4),
        interface: UserInterface {
            data_ready_enabled: true,
            ..Default::default()
        },
        ..Default::default()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    #[cfg(any(target_arch = "riscv32", test))]
    Driver,
    #[cfg(any(target_arch = "riscv32", test))]
    Timeout,
    Gap,
    Range,
    Timing,
}

#[derive(Default)]
struct Recovery {
    hardware_failures: u8,
}

impl Recovery {
    fn success(&mut self) {
        self.hardware_failures = 0;
    }
    fn error(&mut self, error: Error) -> bool {
        if matches!(error, Error::Driver | Error::Timeout) {
            self.hardware_failures = self.hardware_failures.saturating_add(1);
        } else {
            // Scheduling gaps and out-of-range motion are missing support,
            // not proof of a broken chip. Alignment owns continuity reseeding.
            self.hardware_failures = 0;
        }
        self.hardware_failures >= 4
    }
}

#[derive(Clone, Copy, Debug)]
struct AlignedSample {
    captured_ns: u64,
    end_ns: u64,
    duration_ns: u32,
    acceleration_mps2: [f64; 3],
    angular_rate_rps: [f64; 3],
}

#[derive(Default)]
struct Alignment {
    last: Option<(MotionSample, u64)>,
}

#[derive(Default)]
struct EdgeProgress(Option<u64>);

impl EdgeProgress {
    /// A GPIO can remain high after its SPI burst. Claim each captured rising
    /// edge once, so pin-level readiness cannot trigger repeated sensor reads.
    fn claim(&mut self, edge_ns: u64) -> bool {
        if self.0.is_some_and(|previous| edge_ns <= previous) {
            return false;
        }
        self.0 = Some(edge_ns);
        true
    }
}

impl Alignment {
    fn clear(&mut self) {
        self.last = None;
    }

    fn push(
        &mut self,
        motion: MotionSample,
        edge_ns: u64,
        finished_ns: u64,
        latest_edge_ns: Option<u64>,
    ) -> Result<Option<AlignedSample>, Error> {
        if latest_edge_ns != Some(edge_ns)
            || finished_ns
                .checked_sub(edge_ns)
                .is_none_or(|age| age > READ_DEADLINE_NS)
            || edge_ns < ACCEL_DELAY_NS
        {
            self.clear();
            return Err(Error::Timing);
        }
        if motion
            .angular_rate_dps
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 300.0)
            || motion
                .acceleration_mps2
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 80.0)
        {
            self.clear();
            return Err(Error::Range);
        }
        let Some((previous, previous_ns)) = self.last.replace((motion, edge_ns)) else {
            return Ok(None);
        };
        let Some(duration) = edge_ns.checked_sub(previous_ns) else {
            self.clear();
            return Err(Error::Timing);
        };
        if !(600_000..=800_000).contains(&duration) || motion.counter_advance(&previous) != [1; 6] {
            // The current coherent burst seeds the next consecutive pair.
            return Err(Error::Gap);
        }
        let fraction = (ACCEL_DELAY_NS - GYRO_DELAY_NS) as f64 / duration as f64;
        Ok(Some(AlignedSample {
            captured_ns: edge_ns,
            end_ns: edge_ns - ACCEL_DELAY_NS,
            duration_ns: duration as u32,
            acceleration_mps2: motion.acceleration_mps2,
            angular_rate_rps: core::array::from_fn(|i| {
                motion.angular_rate_rps[i]
                    + fraction * (previous.angular_rate_rps[i] - motion.angular_rate_rps[i])
            }),
        }))
    }
}

#[cfg(target_arch = "riscv32")]
mod hardware {
    use super::*;
    use crate::device::{
        platform::{ImuBus, ImuDelay},
        sensors,
        timing::capture,
    };
    use aevia_device_hal::{
        bus::ExclusiveSpiDevice,
        sch16t::{MotionOutput, Sch16t},
    };
    use core::sync::atomic::Ordering;
    use embassy_executor::{SendSpawner, Spawner};
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
    use embassy_time::{Duration, Timer, with_timeout};
    use esp_hal::gpio::{Input, Output};

    type Device<'d> = ExclusiveSpiDevice<ImuBus<'d>, Output<'d>, ImuDelay>;

    pub struct Imu<'d> {
        driver: Sch16t<Device<'d>, ImuDelay>,
        reset: Output<'d>,
        ready: Input<'d>,
        alignment: Alignment,
        edges: EdgeProgress,
    }

    impl<'d> Imu<'d> {
        pub fn new(spi: ImuBus<'d>, cs: Output<'d>, reset: Output<'d>, ready: Input<'d>) -> Self {
            let delay = spi.delay();
            let mut device = ExclusiveSpiDevice::new(spi, cs, delay);
            // MCU GPIO outputs are infallible; establish inactive CS before use.
            assert!(device.initialize().is_ok());
            Self {
                driver: Sch16t::new(device, delay),
                reset,
                ready,
                alignment: Alignment::default(),
                edges: EdgeProgress::default(),
            }
        }

        async fn initialize(&mut self) -> Result<(), Error> {
            crate::device::platform::imu_spi_context(false);
            let previous = diagnostics();
            publish_diagnostics(ImuDiagnostics {
                attempts: previous.attempts.saturating_add(1),
                ready: false,
                ..previous
            });
            self.alignment.clear();
            self.edges = EdgeProgress::default();
            self.reset.set_low();
            Timer::after_millis(2).await;
            self.reset.set_high();
            Timer::after_millis(32).await;
            // The portable HAL owns verification and scaling configuration.
            // Its synchronous settling delay runs only on the main executor;
            // acquisition and GNSS interrupt executors remain preemptible.
            let identity = self.driver.initialize(configuration()).map_err(|error| {
                publish_diagnostics(diagnostics().with_error(&error));
                Error::Driver
            })?;
            // EN_SENSOR starts DRDY before the HAL's 215 ms settling wait.
            // Drain that old output without treating its age as a sampling
            // failure, then transfer ownership for fresh-edge acquisition.
            if let Some(edge) = capture::drdy_ns() {
                self.edges.claim(edge);
            }
            self.driver
                .motion(MotionOutput::Decimated)
                .map_err(|error| {
                    publish_diagnostics(diagnostics().with_error(&error));
                    Error::Driver
                })?;
            publish_diagnostics(ImuDiagnostics {
                ready: true,
                component: Some(identity.component),
                asic: Some(identity.asic),
                ..diagnostics()
            });
            Ok(())
        }

        async fn next_sample(&mut self) -> Result<sensors::ImuSample, Error> {
            crate::device::platform::imu_spi_context(true);
            loop {
                if with_timeout(Duration::from_millis(100), self.ready.wait_for_high())
                    .await
                    .is_err()
                {
                    self.alignment.clear();
                    return Err(Error::Timeout);
                }
                let Some(edge_ns) = capture::drdy_ns() else {
                    self.alignment.clear();
                    return Err(Error::Timing);
                };
                if !self.edges.claim(edge_ns) {
                    // Yield on the actual level transition instead of polling
                    // the same latch and monopolizing the acquisition IRQ.
                    if with_timeout(Duration::from_millis(100), self.ready.wait_for_low())
                        .await
                        .is_err()
                    {
                        self.alignment.clear();
                        return Err(Error::Timeout);
                    }
                    continue;
                }
                let Some(started_ns) = capture::now_ns() else {
                    self.alignment.clear();
                    return Err(Error::Timing);
                };
                // HAL drains all six axes even when a frame fails, rearming DRDY.
                let motion = match self.driver.motion(MotionOutput::Decimated) {
                    Ok(motion) => motion,
                    Err(error) => {
                        publish_diagnostics(diagnostics().with_error_at(&error, Some(edge_ns)));
                        self.alignment.clear();
                        return Err(
                            if matches!(
                                error,
                                aevia_device_hal::sch16t::Error::SensorStatus {
                                    status: aevia_device_hal::sch16t::SensorStatus::Saturation,
                                    common_error: false,
                                    ..
                                }
                            ) {
                                Error::Range
                            } else {
                                Error::Driver
                            },
                        );
                    }
                };
                let Some(finished_ns) = capture::now_ns() else {
                    self.alignment.clear();
                    return Err(Error::Timing);
                };
                sensors::IMU_MAX_READ_LATENCY_US.fetch_max(
                    (finished_ns.saturating_sub(edge_ns) / 1000).min(u64::from(u32::MAX)) as u32,
                    Ordering::Relaxed,
                );
                let latest = capture::drdy_ns();
                read_timing::observe(edge_ns, started_ns, finished_ns, latest);
                let aligned = self.alignment.push(motion, edge_ns, finished_ns, latest);
                if matches!(aligned, Err(Error::Range)) {
                    publish_diagnostics(diagnostics().with_numeric_range(&motion, edge_ns));
                }
                let Some(sample) = aligned? else {
                    continue;
                };
                return Ok(sensors::ImuSample {
                    captured_ns: sample.captured_ns,
                    end_ns: sample.end_ns,
                    duration_ns: sample.duration_ns,
                    acceleration_mps2: sample.acceleration_mps2,
                    angular_rate_rps: sample.angular_rate_rps,
                    saturated: false,
                });
            }
        }
    }

    // Transfer complete ownership: neither executor can access the SPI driver
    // while the other initializes/samples it. Queues never carry measurements.
    static INITIALIZE: Channel<CriticalSectionRawMutex, Imu<'static>, 1> = Channel::new();
    static READY: Channel<CriticalSectionRawMutex, Imu<'static>, 1> = Channel::new();

    pub fn start(main: Spawner, acquisition: SendSpawner, imu: Imu<'static>) {
        assert!(INITIALIZE.try_send(imu).is_ok());
        main.spawn(initializer().expect("IMU initializer task"));
        acquisition.spawn(acquire().expect("IMU acquisition task"));
    }

    #[embassy_executor::task]
    async fn initializer() {
        loop {
            let mut imu = INITIALIZE.receive().await;
            loop {
                match imu.initialize().await {
                    Ok(()) => break,
                    Err(_) => {
                        sensors::IMU_ERRORS.fetch_add(1, Ordering::Relaxed);
                        Timer::after_secs(1).await;
                    }
                }
            }
            READY.send(imu).await;
        }
    }

    #[embassy_executor::task]
    async fn acquire() {
        loop {
            let mut imu = READY.receive().await;
            let mut recovery = Recovery::default();
            loop {
                match imu.next_sample().await {
                    Ok(sample) => {
                        recovery.success();
                        READY_STATE.store(true, Ordering::Relaxed);
                        sensors::record_imu(sample.captured_ns / 1000);
                        if sensors::IMU.try_send(sample).is_err() {
                            sensors::IMU_DROPS.fetch_add(1, Ordering::Relaxed);
                        }
                        sensors::observe_queue_depths();
                    }
                    Err(error) => {
                        sensors::IMU_ERRORS.fetch_add(1, Ordering::Relaxed);
                        let mut diag = diagnostics();
                        match error {
                            Error::Timing => {
                                diag.timing_errors = diag.timing_errors.saturating_add(1)
                            }
                            Error::Gap => diag.gap_errors = diag.gap_errors.saturating_add(1),
                            Error::Range => diag.range_errors = diag.range_errors.saturating_add(1),
                            Error::Timeout => {
                                diag.drdy_timeouts = diag.drdy_timeouts.saturating_add(1)
                            }
                            Error::Driver => {}
                        }
                        publish_diagnostics(diag);
                        if matches!(error, Error::Timing | Error::Gap) {
                            sensors::IMU_TIMING_ERRORS.fetch_add(1, Ordering::Relaxed);
                        }
                        if recovery.error(error) {
                            break;
                        }
                    }
                }
                // Normal bursts clear DRDY; waiting for its next high level
                // yields directly to the GPIO interrupt, without depending on
                // the lower-priority RTOS timer. If it is already high, leave a
                // real gap so an overloaded level-ready source cannot keep the
                // interrupt executor running forever. The hardware measurement
                // deadline remains 600 us in either path.
                if imu.ready.is_high() {
                    Timer::after_micros(100).await;
                }
            }
            INITIALIZE.send(imu).await;
        }
    }
}

#[cfg(target_arch = "riscv32")]
pub use hardware::{Imu, start};

#[cfg(test)]
mod tests {
    use super::*;

    fn motion(counter: u8, gyro: f64) -> MotionSample {
        MotionSample {
            raw_rate: [0; 3],
            raw_acceleration: [0; 3],
            angular_rate_dps: [gyro.to_degrees(); 3],
            angular_rate_rps: [gyro; 3],
            acceleration_mps2: [1.0, 2.0, 9.8],
            counters: [counter; 6],
        }
    }

    fn push(
        alignment: &mut Alignment,
        sample: MotionSample,
        edge: u64,
    ) -> Result<Option<AlignedSample>, Error> {
        alignment.push(sample, edge, edge + 50_000, Some(edge))
    }

    #[test]
    fn preserves_real_intervals_wraps_and_aligns_gyro_support() {
        let mut alignment = Alignment::default();
        assert!(
            push(&mut alignment, motion(15, 1.0), 10_000_000)
                .unwrap()
                .is_none()
        );
        let sample = push(&mut alignment, motion(0, 2.0), 10_700_000)
            .unwrap()
            .unwrap();
        assert_eq!(sample.captured_ns, 10_700_000);
        assert_eq!(sample.end_ns, 9_200_000);
        assert_eq!(sample.duration_ns, 700_000);
        assert!((sample.angular_rate_rps[0] - (2.0 - 1.0 / 7.0)).abs() < 1.0e-12);
        assert_eq!(sample.acceleration_mps2, [1.0, 2.0, 9.8]);
    }

    #[test]
    fn missing_or_repeated_axis_never_stretches_an_interval() {
        for advance in [0, 2] {
            let mut alignment = Alignment::default();
            push(&mut alignment, motion(1, 1.0), 10_000_000).unwrap();
            let mut current = motion(2, 2.0);
            current.counters[4] = 1 + advance;
            assert_eq!(
                push(&mut alignment, current, 10_700_000).unwrap_err(),
                Error::Gap
            );
            let mut next = motion(3, 2.0);
            next.counters[4] = current.counters[4] + 1;
            assert!(push(&mut alignment, next, 11_400_000).unwrap().is_some());
        }
        let mut alignment = Alignment::default();
        push(&mut alignment, motion(1, 1.0), 10_000_000).unwrap();
        // Even a modulo-16 counter wrap cannot hide an outage.
        assert_eq!(
            push(&mut alignment, motion(2, 1.0), 21_900_000).unwrap_err(),
            Error::Gap
        );
    }

    #[test]
    fn late_overwritten_or_backward_capture_discards_alignment_history() {
        for (finished, latest) in [
            (11_300_001, Some(10_700_000)),
            (10_750_000, Some(11_400_000)),
            (10_699_999, Some(10_700_000)),
        ] {
            let mut alignment = Alignment::default();
            push(&mut alignment, motion(1, 1.0), 10_000_000).unwrap();
            assert_eq!(
                alignment
                    .push(motion(2, 2.0), 10_700_000, finished, latest)
                    .unwrap_err(),
                Error::Timing
            );
            assert!(
                push(&mut alignment, motion(3, 2.0), 11_400_000)
                    .unwrap()
                    .is_none()
            );
        }
    }

    #[test]
    fn invalid_motion_cannot_seed_a_future_interval() {
        for value in [f64::NAN, f64::INFINITY, 81.0] {
            let mut alignment = Alignment::default();
            let mut invalid = motion(0, 1.0);
            invalid.acceleration_mps2[1] = value;
            assert_eq!(
                push(&mut alignment, invalid, 10_000_000).unwrap_err(),
                Error::Range
            );
            assert!(
                push(&mut alignment, motion(1, 1.0), 10_700_000)
                    .unwrap()
                    .is_none()
            );
        }
    }

    #[test]
    fn rejected_numeric_motion_retains_absolute_axes_and_capture_time() {
        let mut alignment = Alignment::default();
        let mut rejected = motion(0, 1.0);
        rejected.angular_rate_dps = [-301.25, 2.0, -3.0];
        rejected.acceleration_mps2 = [-4.0, 5.0, -81.5];
        let edge_ns = 10_000_000;
        assert_eq!(
            push(&mut alignment, rejected, edge_ns).unwrap_err(),
            Error::Range
        );
        let retained = ImuDiagnostics::default().with_numeric_range(&rejected, edge_ns);
        assert_eq!(retained.numeric_range_errors, 1);
        assert_eq!(retained.last_numeric_range_at_ns, Some(edge_ns));
        assert_eq!(retained.last_numeric_rate_abs_dps, Some([301.25, 2.0, 3.0]));
        assert_eq!(retained.last_numeric_accel_abs_mps2, Some([4.0, 5.0, 81.5]));
        assert!(
            push(&mut alignment, motion(1, 1.0), 10_700_000)
                .unwrap()
                .is_none()
        );

        // A later transport error must not erase the numeric failure evidence.
        let retained = retained.with_error::<()>(&aevia_device_hal::sch16t::Error::Spi(()));
        assert_eq!(retained.last_numeric_range_at_ns, Some(edge_ns));
        assert_eq!(retained.last_numeric_rate_abs_dps, Some([301.25, 2.0, 3.0]));

        rejected.angular_rate_dps = [f64::NAN, f64::NEG_INFINITY, 0.0];
        assert_eq!(
            push(&mut alignment, rejected, 11_400_000).unwrap_err(),
            Error::Range
        );
        let retained = retained.with_numeric_range(&rejected, 11_400_000);
        assert_eq!(retained.numeric_range_errors, 2);
        let axes = retained.last_numeric_rate_abs_dps.unwrap();
        assert!(axes[0].is_nan());
        assert_eq!(axes[1], f64::INFINITY);
    }

    #[test]
    fn rejected_sensor_status_retains_raw_payload_independently_of_later_errors() {
        use aevia_device_hal::sch16t::{Error as DriverError, Register, SensorStatus};
        let retained = ImuDiagnostics::default().with_error_at::<()>(
            &DriverError::SensorStatus {
                register: Register::RateY2,
                status: SensorStatus::Saturation,
                common_error: false,
                payload: 0x0e1dc0,
                counter: 9,
            },
            Some(42_000_000),
        );
        assert_eq!(retained.last_sensor_status_at_ns, Some(42_000_000));
        assert_eq!(
            retained.last_sensor_status_register,
            Some(Register::RateY2.address())
        );
        assert_eq!(retained.last_sensor_status_flags, Some(4));
        assert_eq!(retained.last_sensor_status_payload, Some(0x0e1dc0));
        assert_eq!(retained.last_sensor_status_counter, Some(9));
        let retained = retained.with_error::<()>(&DriverError::Spi(()));
        assert_eq!(retained.last_sensor_status_payload, Some(0x0e1dc0));
        assert_eq!(retained.last_sensor_status_at_ns, Some(42_000_000));
    }

    #[test]
    fn profile_keeps_all_axes_on_same_decimated_output() {
        let config = configuration();
        assert!(config.validate().is_ok());
        assert_eq!(config.rate_decimation, config.acc_decimation);
        assert_eq!(
            config.rate_decimation,
            AxisDecimation::all(Decimation::Dec4)
        );
        assert!(config.interface.data_ready_enabled);
        assert!(!config.interface.data_ready_active_low);
    }

    #[test]
    fn driver_diagnostics_preserve_the_actual_startup_failure() {
        use aevia_device_hal::sch16t::{Error as DriverError, Register, Status};
        let previous = ImuDiagnostics {
            attempts: 7,
            ready: true,
            ..Default::default()
        };
        let failed = previous.with_error::<()>(&DriverError::Readback {
            register: Register::SelfTest,
            expected: 0x07fe,
            actual: 0x0002,
        });
        assert!(!failed.ready);
        assert_eq!(failed.attempts, 7);
        assert_eq!(failed.error_kind, 5);
        assert_eq!(failed.register, Some(0x34));
        assert_eq!(failed.expected, Some(0x07fe));
        assert_eq!(failed.actual, Some(2));
        let registers = [0xfffe; 10];
        let failed = previous.with_error::<()>(&DriverError::StartupStatus(Status {
            registers,
            frame_faults: 4,
        }));
        assert_eq!(failed.error_kind, 6);
        assert_eq!(failed.status_registers, Some(registers));
        assert_eq!(failed.status_frame_faults, 4);
    }

    #[test]
    fn held_high_gpio_cannot_claim_one_captured_epoch_twice() {
        let mut edges = EdgeProgress::default();
        assert!(edges.claim(10_000_000));
        for _ in 0..100 {
            assert!(!edges.claim(10_000_000));
        }
        assert!(!edges.claim(9_000_000));
        assert!(edges.claim(10_700_000));
        assert!(!edges.claim(10_700_000));
        assert!(edges.claim(11_400_000));
    }

    #[test]
    fn scheduling_rejections_and_motion_range_do_not_reset_healthy_hardware() {
        let mut recovery = Recovery::default();
        for _ in 0..100 {
            for error in [Error::Timing, Error::Gap, Error::Range] {
                assert!(!recovery.error(error));
            }
        }
        for _ in 0..3 {
            assert!(!recovery.error(Error::Driver));
        }
        assert!(!recovery.error(Error::Timing));
        for _ in 0..3 {
            assert!(!recovery.error(Error::Timeout));
        }
        recovery.success();
        for _ in 0..3 {
            assert!(!recovery.error(Error::Driver));
        }
        assert!(recovery.error(Error::Timeout));
    }
}
