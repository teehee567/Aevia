//! SCH16T-K01 acquisition with bounded startup and validated interval samples.
//!
//! Register values, frame layout and startup sequence: Murata Doc. 11624 Rev. 6,
//! `data/datasheets/sch16t-k01-datasheet-full.pdf`, sections 5–7. The sensor
//! applies its factory calibration internally. No assembled-board calibration
//! or boresight calibration is claimed by this driver.

use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Input, Output},
    spi::master::Spi,
};

const FREQ_CNTR: u8 = 0x13;
const COMP_ID: u8 = 0x3c;
const CTRL_MODE: u8 = 0x35;
const SYS_TEST: u8 = 0x37;
const BATCH_SIZE: usize = 4;

// LPF3 Bessel, DYN1, DEC5 on every decimated axis. Native ODR is F_PRIM/32,
// nominally 737.5 Hz (690.625–784.375 Hz over the oscillator range).
// All native sets are read; four consecutive sets form one ~184 Hz interval
// average. This driver does not perform high-rate coning/sculling compensation.
const PROFILE: &[(u8, u16)] = &[
    (0x25, 0x00db),
    (0x26, 0x00db),
    (0x27, 0x00db),
    (0x28, 0x1324),
    (0x29, 0x1324),
    (0x2a, 0x0000),
    (0x2b, 0x0000),
    (0x2c, 0x0000),
    (0x2d, 0x0000),
    (0x2e, 0x0000),
    (0x33, 0x202c), // 3.3 V, normal slew rate, active-high data-ready.
    (0x34, 0x0ffe), // Keep continuous and startup self-tests enabled.
];

use aevia_firmware::imu::*;

struct NativeSample {
    rates: [i32; 3],
    accelerations: [i32; 3],
    counters: [u8; 6],
}

pub struct Imu<'d> {
    spi: Spi<'d, Blocking>,
    chip_select: Output<'d>,
    reset_n: Output<'d>,
    data_ready: Input<'d>,
    pending_address: Option<u8>,
    strict_status: bool,
    last_status_frame: u64,
    last_counter: Option<[u8; 6]>,
    last_time_us: Option<u64>,
    pub last_sample_error: Option<ImuError>,
    pub component_id: u16,
    pub asic_id: u16,
}

impl<'d> Imu<'d> {
    /// SPI must be mode 0, MSB first, at no more than 10 MHz. CS is controlled
    /// here, independently of the SPI peripheral's hardware CS output.
    pub fn new(
        spi: Spi<'d, Blocking>,
        chip_select: Output<'d>,
        reset_n: Output<'d>,
        data_ready: Input<'d>,
    ) -> Self {
        Self {
            spi,
            chip_select,
            reset_n,
            data_ready,
            pending_address: None,
            strict_status: false,
            last_status_frame: 0,
            last_counter: None,
            last_time_us: None,
            last_sample_error: None,
            component_id: 0,
            asic_id: 0,
        }
    }

    pub async fn initialize(&mut self) -> Result<(), ImuError> {
        let mut last_error = ImuError::DataReadyTimeout;
        for _ in 0..5 {
            match self.initialize_once().await {
                Ok(()) => return Ok(()),
                Err(error) => last_error = error,
            }
        }
        Err(last_error)
    }

    async fn initialize_once(&mut self) -> Result<(), ImuError> {
        self.strict_status = false;
        self.last_status_frame = 0;
        self.pending_address = None;
        self.last_counter = None;
        self.last_time_us = None;
        self.chip_select.set_high();
        self.reset_n.set_low();
        Timer::after_millis(2).await;
        self.reset_n.set_high();
        // Mode 0 holds SCK low for the complete NVM/SPI startup interval.
        Timer::after_millis(32).await;

        self.component_id = self.read_register(COMP_ID)?;
        if self.component_id != 0x0023 {
            return Err(ImuError::Identity(self.component_id));
        }
        self.asic_id = self.read_register(0x3b)?;
        if self.asic_id & 0x0f00 != 0 {
            return Err(ImuError::Identity(self.asic_id));
        }
        self.write_register(SYS_TEST, 0x5a3c)?;
        self.verify_register(SYS_TEST, 0x5a3c)?;
        self.write_register(SYS_TEST, 0)?;
        self.verify_register(SYS_TEST, 0)?;
        for &(address, value) in PROFILE {
            self.write_register(address, value)?;
        }
        // Prove the control image before EOI freezes configuration and the DSP
        // integrity monitor takes ownership of it.
        for &(address, value) in PROFILE {
            self.verify_register(address, value)?;
        }
        self.write_register(CTRL_MODE, 1)?;
        self.verify_register(CTRL_MODE, 1)?;
        Timer::after_millis(215).await;

        // These first status reads clear intentionally latched startup flags.
        let before = self.read_status()?;
        self.write_register(CTRL_MODE, 3)?;
        self.verify_register(CTRL_MODE, 3)?;
        Timer::after_millis(3).await;
        // Retire the FREQ_CNTR reply requested before the clearing delay; its
        // status belongs to that earlier request, not the post-delay state.
        self.exchange(FREQ_CNTR, None)?;
        let mut healthy_passes = 0;
        let mut last_fault = None;
        for _ in 0..10 {
            self.last_status_frame = 0;
            let registers = self.read_status()?;
            // Read the complete diagnostic set before rejecting it. Aborting
            // on the pending FREQ_CNTR response hides the actual fault bits.
            // Acquisition still requires two wholly healthy status passes.
            if registers != [0xffff; 10] || self.last_status_frame != 0 {
                healthy_passes = 0;
                last_fault = Some(ImuError::StartupStatus {
                    before_common: before[2],
                    registers,
                    frame: self.last_status_frame,
                    asic: self.asic_id,
                });
            } else {
                healthy_passes += 1;
                if healthy_passes == 2 {
                    break;
                }
            }
            // Reading STAT_COM acknowledges the mode-transition/DSP flags,
            // but their FTREE_TDEL clearing is asynchronous (nominal 2.5 ms).
            // Do not immediately re-read the same latched fault or reset
            // before the second diagnostic pass can observe its clearance.
            Timer::after_millis(3).await;
            self.exchange(FREQ_CNTR, None)?;
        }
        if healthy_passes != 2 {
            return Err(last_fault.unwrap_or(ImuError::DataReadyTimeout));
        }
        self.strict_status = true;
        for &(address, value) in PROFILE {
            self.verify_register(address, value)?;
        }
        self.verify_register(CTRL_MODE, 3)?;
        self.verify_register(COMP_ID, 0x0023)?;
        Ok(())
    }

    fn read_status(&mut self) -> Result<[u16; 10], ImuError> {
        // All bits, including reserved bits, are one for healthy registers
        // STAT_SUM through STAT_ACC_Z (tables 38–51).
        let mut registers = [0; 10];
        for (offset, value) in registers.iter_mut().enumerate() {
            *value = self.read_register(0x14 + offset as u8)?;
        }
        Ok(registers)
    }

    fn verify_register(&mut self, address: u8, expected: u16) -> Result<(), ImuError> {
        let actual = self.read_register(address)?;
        if actual != expected {
            return Err(ImuError::Readback {
                address,
                expected,
                actual,
            });
        }
        Ok(())
    }

    fn read_register(&mut self, address: u8) -> Result<u16, ImuError> {
        self.exchange(address, None)?;
        let frame = self.exchange(FREQ_CNTR, None)?.ok_or(ImuError::Spi)?;
        if frame >> 47 != 0 {
            return Err(ImuError::Format(frame));
        }
        Ok(((frame >> 8) & 0xffff) as u16)
    }

    fn write_register(&mut self, address: u8, value: u16) -> Result<(), ImuError> {
        self.exchange(address, Some(value))?;
        self.exchange(FREQ_CNTR, None)?;
        Ok(())
    }

    /// Reads one complete interval, discarding the whole batch on any fault.
    /// Calling again after an acquisition error is safe: all six registers are
    /// drained on a bad burst, and the next burst establishes a fresh baseline.
    pub async fn next_sample(&mut self) -> Result<ImuSample, ImuError> {
        let mut acceleration = [0_i64; 3];
        let mut angular_rate = [0_i64; 3];
        let mut count = 0;
        let mut start_us = 0;
        loop {
            if with_timeout(Duration::from_millis(100), self.data_ready.wait_for_high())
                .await
                .is_err()
            {
                self.last_counter = None;
                self.last_time_us = None;
                self.last_sample_error = Some(ImuError::DataReadyTimeout);
                return Err(ImuError::DataReadyTimeout);
            }
            let end_us = Instant::now().as_micros();
            let NativeSample {
                rates,
                accelerations,
                counters: counter,
            } = match self.read_native() {
                Ok(sample) => sample,
                Err(error) => {
                    self.last_counter = None;
                    self.last_time_us = None;
                    self.last_sample_error = Some(error);
                    return Err(error);
                }
            };
            let previous_counter = self.last_counter.replace(counter);
            let previous_time = self.last_time_us.replace(end_us);
            let (Some(previous_counter), Some(previous_time)) = (previous_counter, previous_time)
            else {
                continue; // Establish the first complete interval boundary.
            };
            let elapsed_us = end_us.saturating_sub(previous_time);
            // The six counters are independent and can have distinct startup
            // offsets. Every axis must advance by one; equality of their raw
            // values is neither required nor specified by the sensor.
            if !counters_are_consecutive(previous_counter, counter) {
                let error = ImuError::CounterMismatch {
                    previous: previous_counter,
                    current: counter,
                };
                self.last_sample_error = Some(error);
                return Err(error);
            }
            // A complete counter wrap cannot hide a long capture outage.
            if !(500..=2_800).contains(&elapsed_us) {
                let error = ImuError::SampleGap {
                    elapsed_us: elapsed_us.min(u64::from(u32::MAX)) as u32,
                    data_ready: self.data_ready.is_high(),
                };
                self.last_sample_error = Some(error);
                return Err(error);
            }
            if count == 0 {
                start_us = previous_time;
            }
            for axis in 0..3 {
                acceleration[axis] += i64::from(accelerations[axis]);
                angular_rate[axis] += i64::from(rates[axis]);
            }
            count += 1;
            if count == BATCH_SIZE {
                return Ok(ImuSample {
                    acceleration_mps2: acceleration
                        .map(|value| value as f64 / (BATCH_SIZE as f64 * 3200.0)),
                    angular_rate_rps: angular_rate.map(|value| {
                        value as f64 * core::f64::consts::PI / (BATCH_SIZE as f64 * 1600.0 * 180.0)
                    }),
                    monotonic_us: end_us,
                    interval_us: (end_us - start_us) as u32,
                });
            }
        }
    }

    fn read_native(&mut self) -> Result<NativeSample, ImuError> {
        let mut frames = [0_u64; 6];
        let mut error = self.exchange(0x0a, None).err();
        // Pipeline all six decimated-axis reads and consume every response,
        // even if one is invalid, so DRY can rearm for the next complete set.
        for (index, frame) in frames.iter_mut().enumerate() {
            let next_address = if index == 5 {
                FREQ_CNTR
            } else {
                0x0b + index as u8
            };
            match self.exchange(next_address, None) {
                Ok(Some(value)) => *frame = value,
                Ok(None) => {
                    error.get_or_insert(ImuError::Spi);
                }
                Err(fault) => {
                    error.get_or_insert(fault);
                }
            }
        }
        if let Some(error) = error {
            return Err(error);
        }
        let counters = frames.map(|frame| ((frame >> 29) & 15) as u8);
        let mut values = [0_i32; 6];
        for (index, frame) in frames.into_iter().enumerate() {
            if frame >> 47 != 1 {
                return Err(ImuError::Format(frame));
            }
            let value = signed_sensor(frame);
            // Electrical headroom beyond the factory-qualified range is not
            // treated as calibrated evidence. Status saturation is also checked.
            let maximum = if index < 3 { 300 * 1600 } else { 80 * 3200 };
            if value.abs() > maximum {
                return Err(ImuError::OutsideCalibratedRange);
            }
            values[index] = value;
        }
        Ok(NativeSample {
            rates: [values[0], values[1], values[2]],
            accelerations: [values[3], values[4], values[5]],
            counters,
        })
    }

    fn exchange(&mut self, address: u8, write: Option<u16>) -> Result<Option<u64>, ImuError> {
        let request = request_frame(address, write);
        let bytes = request.to_be_bytes();
        let mut wire = [bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]];
        // Datasheet minima are sub-microsecond; 1 us before/after each CS edge
        // covers both setup/hold and inter-frame spacing without an async yield.
        let delay = Delay::new();
        delay.delay_micros(1);
        self.chip_select.set_low();
        delay.delay_micros(1);
        let result = self.spi.transfer(&mut wire);
        delay.delay_micros(1);
        self.chip_select.set_high();
        if result.is_err() {
            self.pending_address = None;
            return Err(ImuError::Spi);
        }
        let previous = self.pending_address.replace(address);
        let Some(expected) = previous else {
            return Ok(None);
        };
        let frame =
            u64::from_be_bytes([0, 0, wire[0], wire[1], wire[2], wire[3], wire[4], wire[5]]);
        if frame & ((1 << 36) | (3 << 33)) != 0 {
            self.last_status_frame = frame;
        }
        validate_response(frame, expected, self.strict_status)?;
        Ok(Some(frame))
    }
}
