//! Versioned native capture payloads. All integers and floats are little endian.
use aevia_device_hal::sch16t::{MotionSample, Response, SensorStatus};

pub(super) fn native(
    frames: [Option<Response>; 6],
    motion: Option<MotionSample>,
    edge: u64,
    started: u64,
    finished: Option<u64>,
    latest: Option<u64>,
    read_deadline_ns: u64,
) -> [u8; 120] {
    let mut bytes = [0; 120];
    bytes[0] = 1;
    // Bit1 uses the applied profile deadline recorded at kind9 offset88.
    // Raw edge/start/finish/latest times remain available for stricter analysis.
    bytes[1] = u8::from(motion.is_some())
        | (u8::from(
            latest == Some(edge)
                && finished.is_some_and(|end| {
                    end.checked_sub(edge)
                        .is_some_and(|age| age <= read_deadline_ns)
                }),
        ) << 1)
        | (u8::from(latest.is_some()) << 2)
        | (u8::from(finished.is_some()) << 3);
    for (offset, value) in [
        (4, edge),
        (12, started),
        (20, finished.unwrap_or(0)),
        (28, latest.unwrap_or(0)),
    ] {
        bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    for (axis, frame) in frames.into_iter().enumerate() {
        if let Some(frame) = frame {
            bytes[2] |= 1 << axis;
            let offset = 36 + axis * 4;
            bytes[offset..offset + 4].copy_from_slice(&frame.signed_sensor().to_le_bytes());
            bytes[60 + axis] = frame.counter;
            bytes[66 + axis] = match frame.status {
                SensorStatus::Normal => 0,
                SensorStatus::Error => 1,
                SensorStatus::Saturation => 2,
                SensorStatus::Initializing => 3,
            } | (u8::from(frame.common_error) << 2);
        }
    }
    if let Some(motion) = motion {
        for (axis, value) in motion
            .angular_rate_rps
            .into_iter()
            .chain(motion.acceleration_mps2)
            .enumerate()
        {
            let offset = 72 + axis * 8;
            bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
    }
    bytes
}

#[cfg(target_arch = "riscv32")]
pub(super) fn fault(error: super::Error, diagnostics: super::ImuDiagnostics, at: u64) {
    let mut bytes = [0; 32];
    bytes[0] = 1; // schema
    bytes[1] = 1; // IMU source
    bytes[2] = match error {
        super::Error::Driver => 1,
        super::Error::Timeout => 2,
        super::Error::Gap => 3,
        super::Error::Range => 4,
        super::Error::Timing => 5,
    };
    bytes[3] = diagnostics.error_kind;
    bytes[4..12].copy_from_slice(
        &diagnostics
            .last_sensor_status_at_ns
            .unwrap_or(0)
            .to_le_bytes(),
    );
    bytes[12] = diagnostics.register.unwrap_or(0xff);
    bytes[13] = diagnostics.last_sensor_status_flags.unwrap_or(0);
    bytes[14..16].copy_from_slice(&diagnostics.expected.unwrap_or(0).to_le_bytes());
    bytes[16..18].copy_from_slice(&diagnostics.actual.unwrap_or(0).to_le_bytes());
    bytes[18] = diagnostics.last_sensor_status_counter.unwrap_or(0);
    bytes[19] = u8::from(diagnostics.last_sensor_status_at_ns.is_some());
    bytes[20..24].copy_from_slice(
        &diagnostics
            .last_sensor_status_payload
            .unwrap_or(0)
            .to_le_bytes(),
    );
    crate::device::recording::capture(6, at, &bytes);
}

#[cfg(test)]
mod tests {
    use super::*;
    use aevia_device_hal::sch16t::Register;
    #[test]
    fn timing_flag_uses_the_applied_profile_bound_without_losing_raw_evidence() {
        for native_profile in [false, true] {
            let deadline = super::super::read_deadline_ns(native_profile);
            for age in [600_000, 600_001, 1_000_000, 1_000_001, 18_545_200] {
                let bytes = native(
                    [None; 6],
                    None,
                    10_000_000,
                    10_050_000,
                    Some(10_000_000 + age),
                    Some(10_000_000),
                    deadline,
                );
                assert_eq!(bytes[1] & 2 != 0, age <= deadline);
                assert_eq!(
                    u64::from_le_bytes(bytes[20..28].try_into().unwrap()),
                    10_000_000 + age
                );
                assert_eq!(bytes[1] & 12, 12);
            }
        }
    }

    #[test]
    fn raw_faulted_axes_and_timing_validity_are_preserved_independently() {
        let axis = Response {
            register: Register::RateX2,
            written: false,
            status: SensorStatus::Saturation,
            common_error: true,
            counter: 15,
            payload: 0xfffff,
        };
        let bytes = native(
            [Some(axis), None, None, None, None, None],
            None,
            1_000_000,
            1_010_000,
            Some(1_100_000),
            Some(1_000_000),
            super::super::NATIVE_READ_DEADLINE_NS,
        );
        assert_eq!(&bytes[..4], &[1, 14, 1, 0]);
        assert_eq!(i32::from_le_bytes(bytes[36..40].try_into().unwrap()), -1);
        assert_eq!(bytes[60], 15);
        assert_eq!(bytes[66], 6);
        assert_eq!(&bytes[72..], &[0; 48]);
        let stale = native(
            [Some(axis); 6],
            None,
            1_000_000,
            1_010_000,
            Some(1_700_000),
            Some(2_000_000),
            super::super::NATIVE_READ_DEADLINE_NS,
        );
        assert_eq!(stale[1], 12);
        let missing = native(
            [None; 6],
            None,
            1,
            2,
            None,
            None,
            super::super::NATIVE_READ_DEADLINE_NS,
        );
        assert_eq!(missing[1], 0);
    }
}
