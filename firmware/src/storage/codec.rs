//! Exact version 1 application record payloads. All integers and floats are
//! little endian; only the returned prefix belongs to the encoded record.
use crate::sample::Sample;

/// Return `(record_kind, hardware_or_receipt_timestamp_ns, payload_bytes)`.
pub fn encode(sample: Sample, data: &mut [u8; 128]) -> (u8, u64, usize) {
    let mut len = 0;
    fn put(data: &mut [u8], len: &mut usize, bytes: &[u8]) {
        data[*len..*len + bytes.len()].copy_from_slice(bytes);
        *len += bytes.len();
    }
    let (kind, at_ns) = match sample {
        Sample::Gnss {
            t_us,
            gps_week,
            gps_tow_ms,
            lat,
            lon,
            alt_m,
            speed_mps,
            heading_deg,
            vertical_mps,
            hacc_m,
            vacc_m,
            sacc_mps,
            latency_ms,
            satellites,
            valid,
            mapped_ns,
            mapping_uncertainty_ns,
            mapping_generation,
        } => {
            put(data, &mut len, &t_us.to_le_bytes());
            put(data, &mut len, &gps_week.to_le_bytes());
            put(data, &mut len, &gps_tow_ms.to_le_bytes());
            for v in [
                lat,
                lon,
                alt_m,
                speed_mps,
                heading_deg,
                vertical_mps,
                hacc_m,
                vacc_m,
                sacc_mps,
            ] {
                put(data, &mut len, &v.to_le_bytes());
            }
            put(data, &mut len, &latency_ms.to_le_bytes());
            put(data, &mut len, &[satellites, u8::from(valid)]);
            let mapped = mapped_ns
                .zip(mapping_uncertainty_ns)
                .zip(mapping_generation);
            put(data, &mut len, &[u8::from(mapped.is_some())]);
            let ((epoch, sigma), generation) = mapped.unwrap_or(((0, 0), 0));
            put(data, &mut len, &epoch.to_le_bytes());
            put(data, &mut len, &sigma.to_le_bytes());
            put(data, &mut len, &generation.to_le_bytes());
            (24, t_us.saturating_mul(1000))
        }
        Sample::Imu {
            t_us,
            accel,
            gyro,
            valid,
            captured_ns,
            interval_ns,
            effective_end_ns,
            timing_uncertainty_ns,
            aligned_gyro,
            ..
        } => {
            let _ = t_us;
            put(data, &mut len, &captured_ns.to_le_bytes());
            put(data, &mut len, &interval_ns.to_le_bytes());
            put(data, &mut len, &effective_end_ns.to_le_bytes());
            put(data, &mut len, &timing_uncertainty_ns.to_le_bytes());
            for v in accel.into_iter().chain(gyro).chain(aligned_gyro) {
                put(data, &mut len, &v.to_le_bytes());
            }
            put(data, &mut len, &[u8::from(valid)]);
            (23, captured_ns)
        }
    };
    (kind, at_ns, len)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn u64_at(bytes: &[u8], offset: usize) -> u64 {
        u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
    }
    fn u32_at(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }
    fn gnss(mapping: (Option<u64>, Option<u64>, Option<u32>)) -> Sample {
        Sample::Gnss {
            t_us: 7_099_123_456_789,
            gps_week: 2444,
            gps_tow_ms: 604_799_987,
            lat: -33.1234567,
            lon: 151.7654321,
            alt_m: -5.5,
            speed_mps: 23.75,
            heading_deg: 271.25,
            vertical_mps: -1.125,
            hacc_m: 0.625,
            vacc_m: 1.875,
            sacc_mps: 0.09375,
            latency_ms: 75,
            satellites: 31,
            valid: true,
            mapped_ns: mapping.0,
            mapping_uncertainty_ns: mapping.1,
            mapping_generation: mapping.2,
        }
    }

    #[test]
    fn imu_kind23_preserves_raw_aligned_vectors_and_exact_nanosecond_support() {
        // Larger than exact integer f64 precision, and not microsecond aligned.
        let captured = 9_007_199_254_741_193;
        let effective = captured - 1_500_017;
        let accel = [-0.0, 1.25, -9.8125];
        let gyro = [0.125, -0.25, 0.375];
        let aligned = [0.0625, -0.1875, 0.3125];
        let sample = Sample::Imu {
            // Deliberately inconsistent legacy projections detect accidental
            // conversion of the ns fields back through the lossy us schema.
            t_us: 17,
            dt_us: 2,
            accel,
            gyro,
            valid: true,
            captured_ns: captured,
            interval_ns: 5_432_199,
            effective_end_ns: effective,
            timing_uncertainty_ns: 1_200_013,
            aligned_gyro: aligned,
        };
        let mut out = [0xa5; 128];
        let (kind, at_ns, len) = encode(sample, &mut out);
        assert_eq!((kind, at_ns, len), (23, captured, 101));
        assert_eq!(u64_at(&out, 0), captured);
        assert_eq!(u32_at(&out, 8), 5_432_199);
        assert_eq!(u64_at(&out, 12), effective);
        assert_eq!(u64_at(&out, 20), 1_200_013);
        for (offset, values) in [(28, accel), (52, gyro), (76, aligned)] {
            for (axis, value) in values.into_iter().enumerate() {
                assert_eq!(u64_at(&out, offset + axis * 8), value.to_bits());
            }
        }
        assert_eq!(out[100], 1);
        assert!(out[101..].iter().all(|b| *b == 0xa5));

        // The envelope must also retain the hardware timestamp without rounding.
        let record = super::super::format::Record::new(kind, 7, at_ns, &out[..len]).unwrap();
        let mut envelope = [0; super::super::format::RECORD_BYTES];
        let written = record.encode(&mut envelope);
        assert_eq!(written, 24 + 101 + 4);
        assert_eq!(u64_at(&envelope, 12), captured);
        assert_eq!(&envelope[24..125], &out[..101]);
    }

    #[test]
    fn gnss_kind24_offsets_preserve_receiver_epoch_and_complete_clock_mapping() {
        let mapped = 9_007_199_254_741_193;
        let sigma = 500_019;
        let generation = 0x89ab_cdef;
        let mut out = [0xa5; 128];
        let (kind, at_ns, len) = encode(
            gnss((Some(mapped), Some(sigma), Some(generation))),
            &mut out,
        );
        assert_eq!((kind, at_ns, len), (24, 7_099_123_456_789_000, 115));
        assert_eq!(u64_at(&out, 0), 7_099_123_456_789);
        assert_eq!(u32_at(&out, 8), 2444);
        assert_eq!(u32_at(&out, 12), 604_799_987);
        let expected: [f64; 9] = [
            -33.1234567,
            151.7654321,
            -5.5,
            23.75,
            271.25,
            -1.125,
            0.625,
            1.875,
            0.09375,
        ];
        for (index, value) in expected.into_iter().enumerate() {
            assert_eq!(u64_at(&out, 16 + index * 8), value.to_bits());
        }
        assert_eq!(u32_at(&out, 88), 75);
        assert_eq!(&out[92..95], &[31, 1, 1]);
        assert_eq!(u64_at(&out, 95), mapped);
        assert_eq!(u64_at(&out, 103), sigma);
        assert_eq!(u32_at(&out, 111), generation);
        assert!(out[115..].iter().all(|b| *b == 0xa5));
    }

    #[test]
    fn every_incomplete_gnss_mapping_has_canonical_absent_bytes() {
        let mut out = [0; 128];
        for mask in 0..7 {
            out.fill(0xa5);
            let mapping = (
                (mask & 1 != 0).then_some(1_234_567_891),
                (mask & 2 != 0).then_some(500_019),
                (mask & 4 != 0).then_some(7),
            );
            assert_eq!(encode(gnss(mapping), &mut out).2, 115);
            assert_eq!(&out[94..115], &[0; 21], "partial mapping mask={mask}");
        }
        // Presence is explicit. Even all-zero Some values differ from None;
        // scientific validation belongs upstream of this lossless encoder.
        encode(gnss((Some(0), Some(0), Some(0))), &mut out);
        assert_eq!(out[94], 1);
        assert_eq!(&out[95..115], &[0; 20]);
    }

    #[test]
    fn receipt_timestamp_overflow_saturates_without_changing_payload() {
        let mut sample = gnss((None, None, None));
        if let Sample::Gnss { t_us, valid, .. } = &mut sample {
            *t_us = u64::MAX;
            *valid = false;
        }
        let mut out = [0; 128];
        assert_eq!(encode(sample, &mut out), (24, u64::MAX, 115));
        assert_eq!(u64_at(&out, 0), u64::MAX);
        assert_eq!(out[93], 0);
    }
}
