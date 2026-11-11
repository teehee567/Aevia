//! Receiver-independent navigation policy. Wire decoding belongs to the HAL;
//! only usable solution fields and their distinct measurement epochs leave here.
use aevia_device_hal::um980::{BestNavXyz, Header};

/// Field eligibility shared by raw fix publication and estimator preparation.
/// Keep vector construction and covariance conversion out of the receive path
/// when collection only needs the original receiver values and validity flags.
pub(super) struct ValidatedFields {
    epoch_ms: u64,
    position_gps_ns: u64,
    pub position_valid: bool,
    pub velocity_valid: bool,
}

impl ValidatedFields {
    pub fn prepare(message: &BestNavXyz) -> Option<Self> {
        let epoch_ms = message.header.gps_epoch_ms()?;
        let position_gps_ns = epoch_ms.checked_mul(1_000_000)?;
        // Receiver header cadence does not prove freshness of its underlying fix.
        // Correction ages are irrelevant to SINGLE position and Doppler velocity;
        // unknown/correction-dependent types conservatively require a fresh age.
        let correction_fresh = age_fresh(message.differential_age_s);
        let position_valid = message.position_valid()
            && age_fresh(message.solution_age_s)
            && (message.position_type == 16 || correction_fresh);
        let velocity_valid = message.velocity_valid()
            && message.velocity_latency_s <= 1.0
            && (matches!(message.velocity_type, 8 | 16) || correction_fresh);
        if !position_valid && !velocity_valid {
            return None;
        }
        // A valid latency is at most one second. Modern GPS epochs cannot
        // underflow, so raw fix publication needs no software-f64 time conversion.
        // Preserve the original whole-message rejection near the GPS origin.
        if velocity_valid && position_gps_ns < 1_000_000_000 {
            position_gps_ns.checked_sub(velocity_latency_ns(message))?;
        }
        Some(Self {
            epoch_ms,
            position_gps_ns,
            position_valid,
            velocity_valid,
        })
    }
}

pub(super) struct Navigation {
    pub epoch_ms: u64,
    pub position_gps_ns: u64,
    pub velocity_gps_ns: u64,
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    pub position_sigma_m: [f64; 3],
    pub velocity_sigma_mps: [f64; 3],
    pub position_valid: bool,
    pub velocity_valid: bool,
    pub satellites: u8,
}

impl Navigation {
    pub fn prepare(message: BestNavXyz) -> Option<Self> {
        let ValidatedFields {
            epoch_ms,
            position_gps_ns,
            position_valid,
            velocity_valid,
        } = ValidatedFields::prepare(&message)?;
        let latency_ns = if velocity_valid {
            velocity_latency_ns(&message)
        } else {
            0
        };
        Some(Self {
            epoch_ms,
            position_gps_ns,
            velocity_gps_ns: position_gps_ns.checked_sub(latency_ns)?,
            // The estimator represents invalid measurements with explicit
            // flags, but still requires finite vector/covariance constructors.
            position_m: if position_valid {
                message.position_ecef_m
            } else {
                [0.0; 3]
            },
            velocity_mps: if velocity_valid {
                message.velocity_ecef_mps
            } else {
                [0.0; 3]
            },
            position_sigma_m: if position_valid {
                message.position_sigma_m.map(f64::from)
            } else {
                [1.0; 3]
            },
            velocity_sigma_mps: if velocity_valid {
                message.velocity_sigma_mps.map(f64::from)
            } else {
                [1.0; 3]
            },
            position_valid,
            velocity_valid,
            satellites: message.satellites_used,
        })
    }
}

fn age_fresh(seconds: f32) -> bool {
    const MAX_AGE_SECONDS: f32 = super::super::profile::MAX_GNSS_AGE_NS as f32 / 1_000_000_000.0;
    // The applied two-second bound is exactly representable. Keep a future
    // profile change from silently rounding the original nanosecond cutoff.
    const _: () = assert!(
        MAX_AGE_SECONDS as f64 * 1_000_000_000.0 == super::super::profile::MAX_GNSS_AGE_NS as f64
    );
    seconds.is_finite() && (0.0..=MAX_AGE_SECONDS).contains(&seconds)
}

fn velocity_latency_ns(message: &BestNavXyz) -> u64 {
    (f64::from(message.velocity_latency_s) * 1_000_000_000.0) as u64
}

/// Count actual advancing receiver epochs, independent of requested log rate,
/// solution validity and PPS lock. Duplicate or backward epochs never fuse.
#[derive(Debug, PartialEq, Eq)]
pub(super) enum EpochState {
    /// Structurally valid navigation before the receiver has trustworthy time.
    TimeUnavailable,
    Stalled,
    Advancing(u64),
}

#[derive(Default)]
pub(super) struct EpochProgress(Option<u64>);
impl EpochProgress {
    pub fn observe(&mut self, header: Header) -> EpochState {
        match header.gps_epoch_ms() {
            None => EpochState::TimeUnavailable,
            Some(epoch) if self.advance(epoch) => EpochState::Advancing(epoch),
            Some(_) => EpochState::Stalled,
        }
    }

    fn advance(&mut self, epoch_ms: u64) -> bool {
        if self.0.is_some_and(|last| epoch_ms <= last) {
            return false;
        }
        self.0 = Some(epoch_ms);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aevia_device_hal::um980::Header;

    fn fix() -> BestNavXyz {
        BestNavXyz {
            header: Header {
                message_id: 240,
                time_reference: 0,
                time_status: 160,
                week: 2400,
                milliseconds_of_week: 1000,
                output_delay_ms: 0,
            },
            position_status: 0,
            position_type: 16,
            position_ecef_m: [6_378_137.0, 0.0, 0.0],
            position_sigma_m: [1.0; 3],
            velocity_status: 0,
            velocity_type: 16,
            velocity_ecef_mps: [1.0, 2.0, 3.0],
            velocity_sigma_mps: [0.1; 3],
            velocity_latency_s: 0.125,
            differential_age_s: 0.0,
            solution_age_s: 0.0,
            satellites_used: 20,
        }
    }

    #[test]
    fn measurement_support_uses_receiver_epoch_and_velocity_latency() {
        let nav = Navigation::prepare(fix()).unwrap();
        assert_eq!(nav.epoch_ms, 2400 * 604_800_000 + 1000);
        assert_eq!(nav.position_gps_ns - nav.velocity_gps_ns, 125_000_000);
        assert!(nav.position_valid && nav.velocity_valid);
        assert_eq!(nav.satellites, 20);
    }

    #[test]
    fn invalid_velocity_does_not_destroy_position_or_leak_nan() {
        for latency in [f32::NAN, f32::INFINITY, -1.0, 1.01] {
            let mut message = fix();
            message.velocity_ecef_mps = [f64::NAN; 3];
            message.velocity_latency_s = latency;
            let nav = Navigation::prepare(message).unwrap();
            assert!(nav.position_valid);
            assert!(!nav.velocity_valid);
            assert_eq!(nav.velocity_gps_ns, nav.position_gps_ns);
            assert_eq!(nav.velocity_mps, [0.0; 3]);
            assert!(
                nav.velocity_sigma_mps
                    .iter()
                    .all(|v| v.is_finite() && *v > 0.0)
            );
        }
    }

    #[test]
    fn velocity_only_solution_remains_usable() {
        let mut message = fix();
        message.position_ecef_m = [f64::NAN; 3];
        message.position_sigma_m = [f32::NAN; 3];
        let nav = Navigation::prepare(message).unwrap();
        assert!(!nav.position_valid);
        assert!(nav.velocity_valid);
        assert_eq!(nav.position_m, [0.0; 3]);
        assert!(
            nav.position_sigma_m
                .iter()
                .all(|v| v.is_finite() && *v > 0.0)
        );
    }

    #[test]
    fn invalid_time_or_both_invalid_solutions_never_reach_estimator() {
        let mut message = fix();
        message.header.time_reference = 1;
        assert!(Navigation::prepare(message).is_none());
        message = fix();
        message.header.time_status = 100;
        assert!(Navigation::prepare(message).is_none());
        message = fix();
        message.position_type = 0;
        message.velocity_type = 0;
        assert!(Navigation::prepare(message).is_none());
    }

    #[test]
    fn stale_or_malformed_position_ages_preserve_independent_doppler_velocity() {
        for age in [2.001, f32::NAN, f32::INFINITY, -0.1] {
            let mut message = fix();
            message.velocity_type = 8; // Independent Doppler velocity.
            message.solution_age_s = age;
            let nav = Navigation::prepare(message).unwrap();
            assert!(!nav.position_valid);
            assert!(nav.velocity_valid);
        }
        let mut message = fix();
        message.position_type = 50; // Correction-dependent RTK position.
        message.velocity_type = 8;
        message.differential_age_s = 3.0;
        let nav = Navigation::prepare(message).unwrap();
        assert!(!nav.position_valid);
        assert!(nav.velocity_valid);
    }

    #[test]
    fn standalone_fixes_do_not_require_nonexistent_corrections() {
        for age in [100.0, f32::NAN] {
            let mut message = fix();
            message.differential_age_s = age;
            let nav = Navigation::prepare(message).unwrap();
            assert!(nav.position_valid && nav.velocity_valid);
        }
        let mut message = fix();
        message.position_type = 50;
        message.velocity_type = 50;
        message.differential_age_s = f32::NAN;
        assert!(Navigation::prepare(message).is_none());
    }

    #[test]
    fn age_comparison_preserves_nan_sign_and_exact_cutoff_behavior() {
        let cutoff = 2.0_f32.to_bits();
        for bits in [
            0,
            1,
            (-0.0_f32).to_bits(),
            (-f32::MIN_POSITIVE).to_bits(),
            cutoff - 1,
            cutoff,
            cutoff + 1,
            f32::MAX.to_bits(),
            f32::INFINITY.to_bits(),
            f32::NEG_INFINITY.to_bits(),
            f32::NAN.to_bits(),
            0xffff_ffff,
        ] {
            let seconds = f32::from_bits(bits);
            let previous = seconds.is_finite()
                && seconds >= 0.0
                && f64::from(seconds) * 1_000_000_000.0
                    <= super::super::super::profile::MAX_GNSS_AGE_NS as f64;
            assert_eq!(age_fresh(seconds), previous, "age bits {bits:08x}");
        }
        assert!(age_fresh(f32::from_bits(cutoff)));
        assert!(!age_fresh(f32::from_bits(cutoff + 1)));
    }

    #[test]
    fn field_validation_preserves_velocity_epoch_underflow_and_overflow_rejection() {
        for (week, milliseconds, latency, valid) in [
            (0, 0, 0.0, true),
            (0, 0, 1.0e-10, true), // Sub-nanosecond latency truncates to zero.
            (0, 0, 0.125, false),
            (0, 124, 0.125, false),
            (0, 125, 0.125, true),
            (0, 999, 1.0, false),
            (0, 1000, 1.0, true),
            (2400, 1000, 1.0, true),
            (u16::MAX, 1000, 0.0, false),
        ] {
            let mut message = fix();
            message.header.week = week;
            message.header.milliseconds_of_week = milliseconds;
            message.velocity_latency_s = latency;
            let fields = ValidatedFields::prepare(&message);
            let nav = Navigation::prepare(message);
            assert_eq!(fields.is_some(), valid, "{week}/{milliseconds}/{latency}");
            assert_eq!(nav.is_some(), valid);
            if let Some(nav) = nav {
                let fields = fields.unwrap();
                assert!(fields.position_valid && fields.velocity_valid);
                assert_eq!(nav.epoch_ms, fields.epoch_ms);
                assert_eq!(nav.position_gps_ns, fields.position_gps_ns);
                assert_eq!(
                    nav.position_gps_ns - nav.velocity_gps_ns,
                    (f64::from(latency) * 1_000_000_000.0) as u64
                );
            }
        }
        // Invalid velocity is independent: it cannot underflow a valid position.
        let mut message = fix();
        message.header.week = 0;
        message.header.milliseconds_of_week = 0;
        message.velocity_latency_s = f32::NAN;
        let fields = ValidatedFields::prepare(&message).unwrap();
        assert!(fields.position_valid && !fields.velocity_valid);
    }

    #[test]
    fn cold_start_navigation_stays_alive_until_time_becomes_fine() {
        let mut progress = EpochProgress::default();
        let mut header = fix().header;
        header.time_status = 20; // Receiver still acquiring trusted time.
        for _ in 0..1000 {
            assert_eq!(progress.observe(header), EpochState::TimeUnavailable);
        }
        header.time_status = 160;
        let epoch = header.gps_epoch_ms().unwrap();
        assert_eq!(progress.observe(header), EpochState::Advancing(epoch));
        assert_eq!(progress.observe(header), EpochState::Stalled);
        header.milliseconds_of_week += 20;
        assert_eq!(progress.observe(header), EpochState::Advancing(epoch + 20));
    }

    #[test]
    fn epochs_advance_once_and_reject_receiver_time_reversal() {
        let mut progress = EpochProgress::default();
        assert!(progress.advance(1000));
        assert!(!progress.advance(1000));
        assert!(!progress.advance(980));
        assert!(progress.advance(1020));
        assert!(!progress.advance(1000));
    }
}
