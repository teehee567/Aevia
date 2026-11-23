//! SCH16T sample types and SafeSPI frame validation, independent of the HAL.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub acceleration_mps2: [f64; 3],
    pub angular_rate_rps: [f64; 3],
    /// Raw hardware-captured end of the averaged interval, rounded to us for
    /// legacy storage and USB. The precision fields below retain nanoseconds.
    pub monotonic_us: u64,
    pub interval_us: u32,
    pub captured_ns: u64,
    pub interval_ns: u32,
    /// Development approximation of acceleration LPF3 group delay. This and
    /// the interpolated gyro are separate from the original recorded vectors.
    pub effective_end_ns: u64,
    pub aligned_angular_rate_rps: [f64; 3],
    pub timing_uncertainty_ns: u64,
}

pub const GYRO_DELAY_NS: u64 = 1_400_000;
pub const ACCEL_DELAY_NS: u64 = 1_500_000;
pub const DELAY_UNCERTAINTY_NS: u64 = 1_200_000;

/// Integer-only interval handed from interrupt acquisition to normal context.
#[derive(Clone, Copy, Debug)]
pub struct NativeInterval {
    pub captured_ns: u64,
    pub interval_ns: u32,
    pub rates: [i32; 3],
    pub previous_rates: [i32; 3],
    pub accelerations: [i32; 3],
    pub generation: u32,
}

impl NativeInterval {
    /// Scale every accepted hardware interval in ordinary executor context.
    /// Recording keeps native support; navigation may separately average four.
    pub fn scaled(self) -> Option<ImuSample> {
        self.align().map(AlignedInterval::scaled)
    }

    pub(crate) fn align(self) -> Option<AlignedInterval> {
        if !(1_200_000..=1_600_000).contains(&self.interval_ns) || self.captured_ns < ACCEL_DELAY_NS
        {
            return None;
        }
        let fraction = (ACCEL_DELAY_NS - GYRO_DELAY_NS) as f64 / f64::from(self.interval_ns);
        Some(AlignedInterval {
            native: self,
            gyro_counts: core::array::from_fn(|axis| {
                f64::from(self.rates[axis])
                    + fraction * f64::from(self.previous_rates[axis] - self.rates[axis])
            }),
        })
    }
}

/// Aligned counts are shared by native logging and navigation averaging.
#[derive(Clone, Copy)]
pub(crate) struct AlignedInterval {
    native: NativeInterval,
    gyro_counts: [f64; 3],
}

impl AlignedInterval {
    pub(crate) fn scaled(self) -> ImuSample {
        let sample = self.native;
        let angular_scale = core::f64::consts::PI / (1600.0 * 180.0);
        ImuSample {
            acceleration_mps2: sample.accelerations.map(|v| f64::from(v) / 3200.0),
            angular_rate_rps: sample.rates.map(|v| f64::from(v) * angular_scale),
            monotonic_us: sample.captured_ns / 1000,
            interval_us: sample.interval_ns / 1000,
            captured_ns: sample.captured_ns,
            interval_ns: sample.interval_ns,
            effective_end_ns: sample.captured_ns - ACCEL_DELAY_NS,
            aligned_angular_rate_rps: self.gyro_counts.map(|v| v * angular_scale),
            timing_uncertainty_ns: DELAY_UNCERTAINTY_NS,
        }
    }
}

/// Floating-point averaging runs only in the normal CPU0 executor. A changed
/// generation or discontinuous interval discards the incomplete four-set batch.
#[derive(Default)]
pub struct Averager {
    generation: Option<u32>,
    count: u8,
    start_ns: u64,
    end_ns: u64,
    acceleration: [i64; 3],
    angular_rate: [i64; 3],
    aligned_rate: [f64; 3],
}
impl Averager {
    pub fn push(&mut self, sample: NativeInterval) -> Option<ImuSample> {
        self.push_aligned(sample.align())
    }

    pub(crate) fn push_aligned(&mut self, interval: Option<AlignedInterval>) -> Option<ImuSample> {
        let Some(AlignedInterval {
            native: sample,
            gyro_counts,
        }) = interval
        else {
            *self = Self::default();
            return None;
        };
        let start = sample
            .captured_ns
            .checked_sub(u64::from(sample.interval_ns))?;
        if self.generation != Some(sample.generation) || (self.count != 0 && start != self.end_ns) {
            *self = Self::default();
            self.generation = Some(sample.generation);
        }
        if self.count == 0 {
            self.start_ns = start;
        }
        self.end_ns = sample.captured_ns;
        for (axis, rate) in gyro_counts.into_iter().enumerate() {
            self.acceleration[axis] += i64::from(sample.accelerations[axis]);
            self.angular_rate[axis] += i64::from(sample.rates[axis]);
            self.aligned_rate[axis] += rate;
        }
        self.count += 1;
        if self.count != 4 {
            return None;
        }
        let result = ImuSample {
            acceleration_mps2: self.acceleration.map(|value| value as f64 / (4.0 * 3200.0)),
            angular_rate_rps: self
                .angular_rate
                .map(|value| value as f64 * core::f64::consts::PI / (4.0 * 1600.0 * 180.0)),
            monotonic_us: self.end_ns / 1000,
            interval_us: ((self.end_ns - self.start_ns) / 1000) as u32,
            captured_ns: self.end_ns,
            interval_ns: (self.end_ns - self.start_ns) as u32,
            effective_end_ns: self.end_ns - ACCEL_DELAY_NS,
            aligned_angular_rate_rps: self
                .aligned_rate
                .map(|value| value * core::f64::consts::PI / (4.0 * 1600.0 * 180.0)),
            timing_uncertainty_ns: DELAY_UNCERTAINTY_NS,
        };
        self.count = 0;
        self.acceleration = [0; 3];
        self.angular_rate = [0; 3];
        self.aligned_rate = [0.0; 3];
        Some(result)
    }
}

/// DEC5's fastest documented native period is >1.274 ms. A completed burst
/// must stay inside 1 ms and retain its original hardware latch throughout.
pub fn capture_is_coherent(
    edge_ns: u64,
    finished_ns: Option<u64>,
    latest_edge_ns: Option<u64>,
) -> bool {
    edge_ns >= ACCEL_DELAY_NS
        && latest_edge_ns == Some(edge_ns)
        && finished_ns
            .and_then(|end| end.checked_sub(edge_ns))
            .is_some_and(|age| age <= 1_000_000)
}

/// Cumulative native acquisition diagnostics. Reasons may overlap: a burst
/// can both exceed the deadline and observe a new DRDY edge.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(target_arch = "riscv32", derive(serde::Serialize))]
pub struct AcquisitionTiming {
    pub attempts: u32,
    pub start_lateness_max_ns: u32,
    pub burst_max_ns: u32,
    /// Sum of measured burst durations, including bursts subsequently rejected.
    /// Missing or backwards start/finish timestamps do not contribute.
    pub burst_total_ns: u64,
    pub finish_age_max_ns: u32,
    pub late_finishes: u32,
    pub changed_edges: u32,
    pub missing_capture: u32,
}
impl AcquisitionTiming {
    pub const fn new() -> Self {
        Self {
            attempts: 0,
            start_lateness_max_ns: 0,
            burst_max_ns: 0,
            burst_total_ns: 0,
            finish_age_max_ns: 0,
            late_finishes: 0,
            changed_edges: 0,
            missing_capture: 0,
        }
    }

    pub fn observe(
        &mut self,
        edge: Option<u64>,
        start: Option<u64>,
        finish: Option<u64>,
        latest_edge: Option<u64>,
    ) {
        self.attempts = self.attempts.saturating_add(1);
        if edge.is_none() || start.is_none() || finish.is_none() || latest_edge.is_none() {
            self.missing_capture = self.missing_capture.saturating_add(1);
        }
        if let Some(edge) = edge {
            if latest_edge.is_some_and(|latest| latest != edge) {
                self.changed_edges = self.changed_edges.saturating_add(1);
            }
            if let Some(age) = start.and_then(|at| at.checked_sub(edge)) {
                self.start_lateness_max_ns = self
                    .start_lateness_max_ns
                    .max(age.min(u64::from(u32::MAX)) as u32);
            }
            if let Some(age) = finish.and_then(|at| at.checked_sub(edge)) {
                self.finish_age_max_ns = self
                    .finish_age_max_ns
                    .max(age.min(u64::from(u32::MAX)) as u32);
                if age > 1_000_000 {
                    self.late_finishes = self.late_finishes.saturating_add(1);
                }
            }
        }
        if let Some(duration) = finish
            .zip(start)
            .and_then(|(end, start)| end.checked_sub(start))
        {
            self.burst_total_ns = self.burst_total_ns.saturating_add(duration);
            self.burst_max_ns = self
                .burst_max_ns
                .max(duration.min(u64::from(u32::MAX)) as u32);
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(serde::Serialize))]
pub enum ImuError {
    Spi,
    Crc(u64),
    Address {
        expected: u8,
        frame: u64,
    },
    Command(u64),
    Status(u64),
    StartupStatus {
        before_common: u16,
        registers: [u16; 10],
        frame: u64,
        asic: u16,
    },
    Format(u64),
    Identity(u16),
    Readback {
        address: u8,
        expected: u16,
        actual: u16,
    },
    DataReadyTimeout,
    CaptureTiming,
    CounterMismatch {
        previous: [u8; 6],
        current: [u8; 6],
    },
    SampleGap {
        elapsed_us: u32,
        data_ready: bool,
    },
    OutsideCalibratedRange,
    AcquisitionQueueOverflow,
}

impl ImuError {
    /// SafeSPI S[1:0] = 10 means saturation, with IDS clear.
    pub fn is_clipping(self) -> bool {
        match self {
            Self::OutsideCalibratedRange => true,
            Self::Status(frame) => frame & ((1 << 36) | (3 << 33)) == 2 << 33,
            _ => false,
        }
    }
}

pub const fn request_frame(address: u8, write: Option<u16>) -> u64 {
    let mut frame = ((address as u64) << 38) | (1 << 35); // FT=1, 48-bit.
    if let Some(value) = write {
        frame |= (1 << 37) | ((value as u64) << 8);
    }
    frame | crc8(frame) as u64
}

pub fn counters_are_consecutive(previous: [u8; 6], current: [u8; 6]) -> bool {
    previous
        .into_iter()
        .zip(current)
        .all(|(old, new)| new == ((old + 1) & 15))
}

pub fn signed_sensor(frame: u64) -> i32 {
    // Signed 20-bit payload occupies bits 27:8.
    (((frame >> 8) as u32 & 0x000f_ffff) << 12) as i32 >> 12
}

#[cfg_attr(target_arch = "riscv32", esp_hal::ram)]
pub fn validate_response(frame: u64, expected: u8, strict_status: bool) -> Result<(), ImuError> {
    if frame == 0 || frame == 0xffff_ffff_ffff || frame as u8 != crc8(frame) {
        return Err(ImuError::Crc(frame));
    }
    if ((frame >> 37) & 0x3ff) != u64::from(expected) {
        return Err(ImuError::Address { expected, frame });
    }
    if frame & (1 << 35) != 0 {
        return Err(ImuError::Command(frame));
    }
    if strict_status && (frame & ((1 << 36) | (3 << 33))) != 0 {
        return Err(ImuError::Status(frame));
    }
    Ok(())
}

// Eight zero-input steps of Murata's augmented CRC-8, Fig. 17. The table is
// evaluated at compile time and occupies 256 bytes in flash, not SRAM.
const CRC8_SHIFT8: [u8; 256] = {
    let mut table = [0; 256];
    let mut index = 0;
    while index < table.len() {
        let mut crc = index as u8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x2f
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[index] = crc;
        index += 1;
    }
    table
};

// Each input byte enters the low eight bits during those eight shifts, so an
// augmented byte step is table[crc] XOR byte (not table[crc XOR byte]). Process
// the 40 message bits, then the mandatory eight zero bits replacing the CRC.
pub const fn crc8(frame: u64) -> u8 {
    let bytes = frame.to_be_bytes();
    let mut crc = 0xff_u8;
    let mut i = 2;
    while i < 7 {
        crc = CRC8_SHIFT8[crc as usize] ^ bytes[i];
        i += 1;
    }
    CRC8_SHIFT8[crc as usize]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn native(index: u64, generation: u32) -> NativeInterval {
        NativeInterval {
            captured_ns: 10_000_000 + index * 1_375_000,
            interval_ns: 1_375_000,
            rates: [1600, 0, -1600],
            previous_rates: [1600, 0, -1600],
            accelerations: [3200, -6400, 31392],
            generation,
        }
    }

    #[test]
    fn normal_context_averaging_preserves_scaling_and_exact_support() {
        let mut average = Averager::default();
        for index in 0..3 {
            assert!(average.push(native(index, 1)).is_none());
        }
        let sample = average.push(native(3, 1)).unwrap();
        assert_eq!(sample.acceleration_mps2, [1.0, -2.0, 9.81]);
        assert!((sample.angular_rate_rps[0] - core::f64::consts::PI / 180.0).abs() < 1e-15);
        assert_eq!(sample.angular_rate_rps, sample.aligned_angular_rate_rps);
        assert_eq!(sample.captured_ns, 14_125_000);
        assert_eq!(sample.interval_ns, 5_500_000);
        assert_eq!(sample.effective_end_ns, 12_625_000);
    }

    #[test]
    fn native_recording_preserves_every_interval_separately_from_navigation() {
        let mut average = Averager::default();
        let mut recorded = std::vec::Vec::new();
        let mut navigation = std::vec::Vec::new();
        for i in 0..12 {
            let interval = native(i, 1);
            recorded.push(interval.scaled().unwrap());
            if let Some(sample) = average.push(interval) {
                navigation.push(sample);
            }
        }
        assert_eq!(recorded.len(), 12);
        assert_eq!(navigation.len(), 3);
        for (i, sample) in recorded.iter().enumerate() {
            assert_eq!(sample.captured_ns, native(i as u64, 1).captured_ns);
            assert_eq!(sample.interval_ns, 1_375_000);
            assert_eq!(sample.acceleration_mps2, [1.0, -2.0, 9.81]);
            assert_eq!(sample.effective_end_ns, sample.captured_ns - ACCEL_DELAY_NS);
            assert_eq!(sample.angular_rate_rps, sample.aligned_angular_rate_rps);
        }
        for (recorded, averaged) in recorded.chunks(4).zip(navigation) {
            assert_eq!(averaged.captured_ns, recorded[3].captured_ns);
            assert_eq!(
                averaged.interval_ns,
                recorded.iter().map(|s| s.interval_ns).sum::<u32>()
            );
        }
        let mut invalid = native(0, 1);
        invalid.interval_ns = 2_000_000;
        assert!(invalid.scaled().is_none());
    }

    #[test]
    fn raw_queue_loss_or_timestamp_gap_cannot_complete_an_old_batch() {
        for change_generation in [false, true] {
            let mut average = Averager::default();
            assert!(average.push(native(0, 1)).is_none());
            assert!(average.push(native(1, 1)).is_none());
            let (first, generation) = if change_generation { (2, 2) } else { (3, 1) };
            for index in first..first + 3 {
                assert!(average.push(native(index, generation)).is_none());
            }
            let sample = average.push(native(first + 3, generation)).unwrap();
            assert_eq!(sample.interval_ns, 5_500_000);
            assert_eq!(
                sample.captured_ns,
                native(first + 3, generation).captured_ns
            );
        }
    }

    #[test]
    fn acquisition_diagnostics_separate_spi_duration_from_late_service() {
        let mut timing = AcquisitionTiming::new();
        // Prompt service but a long SPI transfer.
        timing.observe(
            Some(10_000_000),
            Some(10_010_000),
            Some(11_100_000),
            Some(10_000_000),
        );
        // Short SPI transfer started after another DRDY edge had arrived.
        timing.observe(
            Some(20_000_000),
            Some(21_400_000),
            Some(21_500_000),
            Some(21_375_000),
        );
        timing.observe(None, None, None, None);
        assert_eq!(timing.attempts, 3);
        assert_eq!(timing.start_lateness_max_ns, 1_400_000);
        assert_eq!(timing.burst_max_ns, 1_090_000);
        assert_eq!(timing.burst_total_ns, 1_190_000);
        assert_eq!(timing.finish_age_max_ns, 1_500_000);
        assert_eq!(timing.late_finishes, 2);
        assert_eq!(timing.changed_edges, 1);
        assert_eq!(timing.missing_capture, 1);
    }

    #[test]
    fn burst_total_ignores_missing_or_backwards_measurements_and_never_wraps() {
        let mut timing = AcquisitionTiming::new();
        timing.observe(None, Some(100), Some(250), None);
        timing.observe(None, Some(100), None, None);
        timing.observe(None, None, Some(250), None);
        timing.observe(None, Some(250), Some(100), None);
        assert_eq!(timing.burst_total_ns, 150);
        assert_eq!(timing.burst_max_ns, 150);
        timing.burst_total_ns = u64::MAX - 10;
        timing.observe(None, Some(100), Some(111), None);
        assert_eq!(timing.burst_total_ns, u64::MAX);
    }

    #[test]
    fn safespi_request_and_response_golden_vectors_remain_unchanged() {
        // Fixed transactions used by initialization and native acquisition.
        assert_eq!(request_frame(0x3c, None), 0x0f08_0000_0092); // COMP_ID
        assert_eq!(request_frame(0x13, None), 0x04c8_0000_0087); // FREQ_CNTR
        assert_eq!(request_frame(0x37, Some(0x5a3c)), 0x0de8_005a_3cc2); // SYS_TEST
        assert_eq!(request_frame(0x35, Some(3)), 0x0d68_0000_038d); // CTRL_MODE
        assert_eq!(validate_response(0x8140_0004_d230, 0x0a, true), Ok(()));
        assert_eq!(crc8(0), 0x60);
        assert_eq!(crc8(0xffff_ffff_ffff), 0xac);
    }

    #[test]
    fn byte_crc_matches_augmented_bitwise_reference_for_payloads_and_ignored_bits() {
        fn reference(frame: u64) -> u8 {
            let data = frame & 0xffff_ffff_ff00;
            let mut crc = 0xff_u8;
            for bit in (0..48).rev() {
                let data_bit = ((data >> bit) & 1) as u8;
                crc = if crc & 0x80 != 0 {
                    (crc << 1) ^ 0x2f ^ data_bit
                } else {
                    (crc << 1) | data_bit
                };
            }
            crc
        }
        // Exercise every byte at each payload position, both sparse and set.
        for shift in [8, 16, 24, 32, 40] {
            for byte in 0_u64..=255 {
                let value = byte << shift;
                for frame in [value, !value] {
                    assert_eq!(crc8(frame), reference(frame));
                }
            }
        }
        let mut frame = 0x6a09_e667_f3bc_c909_u64;
        for _ in 0..65_536 {
            frame ^= frame << 13;
            frame ^= frame >> 7;
            frame ^= frame << 17;
            assert_eq!(crc8(frame), reference(frame));
            // Host-container high bits and the received CRC byte are excluded.
            assert_eq!(crc8(frame), crc8(frame ^ 0xffff_0000_0000_00ff));
        }
    }

    #[test]
    fn stale_overwritten_missing_or_backward_hardware_capture_is_rejected() {
        assert!(capture_is_coherent(
            10_000_000,
            Some(11_000_000),
            Some(10_000_000)
        ));
        for (finished, latest) in [
            (Some(11_000_001), Some(10_000_000)),
            (Some(10_100_000), Some(11_400_000)),
            (Some(9_999_999), Some(10_000_000)),
            (None, Some(10_000_000)),
            (Some(10_100_000), None),
        ] {
            assert!(!capture_is_coherent(10_000_000, finished, latest));
        }
    }

    #[test]
    fn every_single_bit_corruption_is_rejected() {
        let frame = (0x0a_u64 << 37) | (1234 << 8) | (1 << 47);
        let frame = frame | u64::from(crc8(frame));
        assert_eq!(validate_response(frame, 0x0a, true), Ok(()));
        for bit in 0..48 {
            assert!(validate_response(frame ^ (1 << bit), 0x0a, true).is_err());
        }
        assert!(matches!(
            validate_response(frame, 0x0b, true),
            Err(ImuError::Address { .. })
        ));
        for invalid in [0, 0xffff_ffff_ffff] {
            assert!(matches!(
                validate_response(invalid, 0, false),
                Err(ImuError::Crc(_))
            ));
        }
    }

    #[test]
    fn sign_extension_preserves_sensor_range_endpoints() {
        assert_eq!(signed_sensor(0x7ffff << 8), 524287);
        assert_eq!(signed_sensor(0x80000 << 8), -524288);
        assert_eq!(signed_sensor(0xfffff << 8), -1);
    }

    #[test]
    fn clipping_uses_sensor_saturation_or_the_calibrated_range_check() {
        let captured_saturation = 0x0264_0007_3bbe;
        assert_eq!(
            validate_response(captured_saturation, 0x13, true),
            Err(ImuError::Status(captured_saturation))
        );
        assert!(ImuError::Status(captured_saturation).is_clipping());
        assert!(ImuError::OutsideCalibratedRange.is_clipping());
        assert!(!ImuError::Status(captured_saturation | (1 << 36)).is_clipping());
        assert!(!ImuError::CaptureTiming.is_clipping());
    }

    #[test]
    fn independent_counters_must_all_advance_including_wrap() {
        assert!(counters_are_consecutive(
            [15, 0, 1, 3, 7, 8],
            [0, 1, 2, 4, 8, 9]
        ));
        assert!(!counters_are_consecutive(
            [15, 0, 1, 3, 7, 8],
            [0, 1, 2, 4, 8, 8]
        ));
        assert!(!counters_are_consecutive([0; 6], [2; 6]));
    }
}
