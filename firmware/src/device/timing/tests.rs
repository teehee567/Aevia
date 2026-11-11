use super::clock::{ClockSync, PpsLabel};
use crate::device::platform::counter;
const SECOND: u64 = 1_000_000_000;

#[test]
fn hardware_counter_wrap_and_ambiguous_outage() {
    let mut counter = counter::Counter::new(u32::MAX - 99, 1000);
    assert_eq!(counter.advance(100, 1020), Some(u64::from(u32::MAX) + 101));
    assert_eq!(
        counter.captured(u32::MAX - 49),
        Some(u64::from(u32::MAX) - 49)
    );
    assert!(counter.advance(200, 300_000_000).is_none());
    let mut stopped = counter::Counter::new(0, 0);
    assert!(stopped.advance(0, 100_000).is_none());
}

#[test]
fn fast_reads_extend_wraps_without_resetting_the_validation_reference() {
    let start = u32::MAX - 999;
    let mut counter = counter::Counter::new(start, 1000);
    assert_eq!(counter.advance_fast(500), Some(u64::from(start) + 1500));
    assert_eq!(
        counter.captured(u32::MAX - 499),
        Some(u64::from(start) + 500)
    );
    assert_eq!(counter.advance_fast(1000), Some(u64::from(start) + 2000));
    assert_eq!(counter.advance(2000, 1300), Some(u64::from(start) + 3000));
    // A backwards or ambiguous raw advance never guesses a new counter epoch.
    assert!(counter.advance_fast(1999).is_none());
}

#[test]
fn periodic_validation_detects_wrong_rate_even_with_continuous_fast_reads() {
    for ticks_per_ms in [0, 9000, 11000] {
        let mut counter = counter::Counter::new(0, 0);
        for ms in 1..=100 {
            assert!(counter.advance_fast(ms * ticks_per_ms).is_some());
        }
        assert!(counter.advance(100 * ticks_per_ms, 100_000).is_none());
    }
    let mut counter = counter::Counter::new(0, 0);
    for ms in 1..=100 {
        assert!(counter.advance_fast(ms * 10_000).is_some());
    }
    assert_eq!(counter.advance(1_000_000, 100_000), Some(1_000_000));
    // Fast hardware reads cannot conceal an ambiguous wall-clock outage.
    assert!(counter.advance_fast(1_100_000).is_some());
    assert!(counter.advance(1_100_000, 300_000_000).is_none());
}

fn label(second: u64) -> PpsLabel {
    PpsLabel {
        epoch_ms: second * 1000,
        header_epoch_ms: (second + 1) * 1000,
        output_delay_ns: 20_000_000,
        quality: 3,
        pulse_error_ns: 5,
        config: (1000 << 16) | 0x20,
    }
}

fn locked_clock() -> ClockSync {
    let mut clock = ClockSync::new();
    // Labels arrive one second after their associated pulse, as in the N4
    // manual's example. Captures include the intervening newer pulse.
    for second in 100..104 {
        clock.pulse((second - 90) * SECOND);
        if second > 100 {
            clock.label(label(second - 1), (second - 90) * SECOND + 22_000_000);
        }
    }
    clock
}

#[test]
fn pulse_payload_labels_previous_second_and_uart_delay_does_not_retime_a_fix() {
    let mut clock = locked_clock();
    let first = clock.epoch(103_020_000_000, 13_040_000_000).unwrap();
    let later = clock.epoch(103_020_000_000, 13_400_000_000).unwrap();
    assert_eq!(first.time_ns, 13_020_000_000);
    assert_eq!(first.time_ns, later.time_ns);
    assert_eq!(first.generation, later.generation);
    assert!(first.uncertainty_ns >= 500_000);
    let velocity = clock.epoch(102_970_000_000, 13_400_000_000).unwrap();
    assert_eq!(first.time_ns - velocity.time_ns, 50_000_000);
}

#[test]
fn pps_loss_invalid_quality_and_wrong_configuration_never_use_arrival_fallback() {
    let mut clock = locked_clock();
    assert!(clock.epoch(103_000_000_000, 14_600_000_000).is_none());
    for invalid in [
        PpsLabel {
            quality: 1,
            ..label(103)
        },
        PpsLabel {
            config: (1000 << 16) | 0x30,
            ..label(103)
        },
    ] {
        let mut clock = locked_clock();
        clock.label(invalid, 14_020_000_000);
        assert!(!clock.status(14_020_000_000).pps_locked);
    }
}

#[test]
fn gps_week_rollover_and_crystal_drift_preserve_the_local_timeline() {
    let gps_ms = |week, milliseconds_of_week| {
        aevia_device_hal::um980::Header {
            message_id: 240,
            time_reference: 0,
            time_status: 160,
            week,
            milliseconds_of_week,
            output_delay_ms: 0,
        }
        .gps_epoch_ms()
    };
    let week_end = gps_ms(2400, 604_799_000).unwrap();
    let next_week = gps_ms(2401, 0).unwrap();
    assert_eq!(next_week - week_end, 1000);
    assert!(gps_ms(2400, 604_800_000).is_none());
    let mut clock = ClockSync::new();
    for step in 0..4 {
        let epoch = week_end - 1000 + step * 1000;
        let local = 10 * SECOND + step * 1_000_100_000;
        clock.pulse(local);
        clock.label(
            PpsLabel {
                epoch_ms: epoch,
                header_epoch_ms: epoch,
                ..label(0)
            },
            local + 22_000_000,
        );
    }
    let mapped = clock
        .epoch((next_week + 1020) * 1_000_000, 13_050_000_000)
        .unwrap();
    assert_eq!(mapped.time_ns, 13_020_302_000);
}

#[test]
fn missing_capture_or_nonconsecutive_pulse_label_requires_reacquisition() {
    let mut clock = locked_clock();
    clock.pulse(15 * SECOND);
    clock.label(label(105), 16_022_000_000);
    assert!(!clock.status(16_022_000_000).pps_locked);
    assert_eq!(clock.status(16_022_000_000).pps_rejected, 1);
}

#[test]
fn integer_header_correlates_physical_pulse_despite_boot_varying_payload_lag() {
    let mut clock = ClockSync::new();
    // Antenna-backed Build17548 boots have shown 20 and 160ms payload lag.
    // Header epoch/output delay independently agree with physical PPS receipt.
    // Include other lags to prove no payload rounding or fixed correction.
    for second in 100..104 {
        let pulse = (second - 90) * SECOND;
        clock.pulse(pulse);
        clock.label(
            PpsLabel {
                epoch_ms: second * 1000 - [20, 160, 321, 999][(second - 100) as usize],
                header_epoch_ms: second * 1000,
                output_delay_ns: 23_000_000,
                ..label(second)
            },
            pulse + 30_000_000,
        );
    }
    assert!(clock.status(13_100_000_000).pps_locked);
    assert_eq!(clock.status(13_100_000_000).pps_pairs, 4);
    assert_eq!(clock.status(13_100_000_000).pps_rejected, 0);
    assert_eq!(
        clock
            .epoch(103_020_000_000, 13_100_000_000)
            .unwrap()
            .time_ns,
        13_020_000_000
    );
    // Once locked, delayed delivery still associates against physical history.
    clock.pulse(14 * SECOND);
    clock.label(
        PpsLabel {
            epoch_ms: 103_980,
            header_epoch_ms: 104_000,
            output_delay_ns: 23_000_000,
            ..label(104)
        },
        14_400_000_000,
    );
    assert!(clock.status(14_400_000_000).pps_locked);
    assert_eq!(
        clock
            .epoch(104_020_000_000, 14_400_000_000)
            .unwrap()
            .time_ns,
        14_020_000_000
    );
}

#[test]
fn fractional_pps_tags_require_integer_header_and_preceding_second_sanity() {
    for (epoch_ms, header_epoch_ms) in [
        (100_980, 102_000),
        (100_980, 100_980),
        (101_020, 101_000),
        (100_840, 100_999),
        (100_840, 101_001),
    ] {
        let mut clock = locked_clock();
        clock.label(
            PpsLabel {
                epoch_ms,
                header_epoch_ms,
                ..label(103)
            },
            13_030_000_000,
        );
        let status = clock.status(13_030_000_000);
        assert!(
            !status.pps_locked,
            "epoch={epoch_ms} header={header_epoch_ms}"
        );
        assert_eq!(status.pps_rejected, 1);
    }
}

#[test]
fn header_correlated_pulses_cross_gps_week_without_changing_continuity() {
    let rollover_ms = 2401 * 604_800_000;
    let mut clock = ClockSync::new();
    for step in 0..4 {
        let pulse_ms = rollover_ms - 1000 + step * 1000;
        let local = 10 * SECOND + step * SECOND;
        clock.pulse(local);
        clock.label(
            PpsLabel {
                epoch_ms: pulse_ms - 160,
                header_epoch_ms: pulse_ms,
                output_delay_ns: 23_000_000,
                ..label(0)
            },
            local + 30_000_000,
        );
    }
    assert!(clock.status(13_100_000_000).pps_locked);
    assert_eq!(
        clock
            .epoch((rollover_ms + 2020) * 1_000_000, 13_100_000_000)
            .unwrap()
            .time_ns,
        13_020_000_000
    );
}

#[test]
fn header_correlation_needs_one_recent_past_physical_edge_at_startup() {
    let header_label = PpsLabel {
        epoch_ms: 99_840,
        header_epoch_ms: 100_000,
        ..label(100)
    };
    // Missing, stale, future, or ambiguous captures must not manufacture lock.
    for (pulses, received) in [
        ([0, 0], 10_030_000_000),
        ([9 * SECOND, 0], 10_030_000_000),
        ([10 * SECOND, 0], 10_120_000_000),
        ([10_040_000_000, 0], 10_030_000_000),
        ([10 * SECOND, 10_010_000_000], 10_030_000_000),
    ] {
        let mut clock = ClockSync::new();
        for pulse in pulses.into_iter().filter(|pulse| *pulse != 0) {
            clock.pulse(pulse);
        }
        clock.label(header_label, received);
        let status = clock.status(received);
        assert_eq!(status.pps_pairs, 0);
        assert_eq!(status.pps_rejected, 1);
    }
    let mut clock = ClockSync::new();
    clock.pulse(10 * SECOND);
    clock.label(header_label, 10_030_000_000);
    assert_eq!(clock.status(10_030_000_000).pps_pairs, 1);
    assert!(!clock.status(10_030_000_000).pps_locked);
}

#[test]
fn header_correlation_cannot_switch_integer_seconds_after_acquisition() {
    let mut clock = ClockSync::new();
    for second in 100..103 {
        let pulse = (second - 90) * SECOND;
        clock.pulse(pulse);
        clock.label(
            PpsLabel {
                epoch_ms: second * 1000 - 160,
                header_epoch_ms: second * 1000,
                ..label(second)
            },
            pulse + 30_000_000,
        );
    }
    assert!(clock.status(12_030_000_000).pps_locked);
    clock.pulse(13 * SECOND);
    clock.label(
        PpsLabel {
            epoch_ms: 103_840,
            header_epoch_ms: 104_000, // Wrong by one second despite plausible payload.
            ..label(104)
        },
        13_030_000_000,
    );
    assert!(!clock.status(13_030_000_000).pps_locked);
    assert_eq!(clock.status(13_030_000_000).pps_rejected, 1);
}
