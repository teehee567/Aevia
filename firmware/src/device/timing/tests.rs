use super::clock::{ClockSync, PpsLabel, gps_ms};
use super::counter;
const SECOND: u64 = 1_000_000_000;

#[test]
fn pps_configuration_readback_checks_checksum_and_zero_delay() {
    // N4 CONFIG example with pulse width changed from 500000 to 100000us.
    let reply = b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0*6A\r";
    assert!(super::clock::valid_pps_config(reply));
    assert!(!super::clock::valid_pps_config(
        b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0*00"
    ));
    assert!(!super::clock::valid_pps_config(
        b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 1*6B"
    ));
}

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
