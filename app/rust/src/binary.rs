//! Shared SD payloads carried in bounded binary USB batches.
use serde_json::{json, Value};

const MAX_BASE64_RECORDS: usize = 48;
const MAX_BASE64_BYTES: usize = MAX_BASE64_RECORDS * (3 + 115);

/// RFC 4648 standard alphabet with required canonical padding and zero unused
/// bits. Whitespace, URL-safe substitutions and partial quanta are rejected.
pub(crate) fn from_base64(text: &str) -> Result<Vec<u8>, String> {
    if !text.len().is_multiple_of(4) || text.len() > MAX_BASE64_BYTES.div_ceil(3) * 4 {
        return Err("Invalid base64 batch length".into());
    }
    let sextet = |byte: u8| -> Result<u8, String> {
        match byte {
            b'A'..=b'Z' => Ok(byte - b'A'),
            b'a'..=b'z' => Ok(byte - b'a' + 26),
            b'0'..=b'9' => Ok(byte - b'0' + 52),
            b'+' => Ok(62),
            b'/' => Ok(63),
            _ => Err("Invalid base64 alphabet or padding".into()),
        }
    };
    let quanta = text.as_bytes().as_chunks::<4>().0;
    let mut bytes = Vec::with_capacity(quanta.len() * 3);
    for (index, quantum) in quanta.iter().enumerate() {
        let last = index + 1 == quanta.len();
        let a = sextet(quantum[0])?;
        let b = sextet(quantum[1])?;
        bytes.push((a << 2) | (b >> 4));
        if quantum[2] == b'=' {
            if !last || quantum[3] != b'=' || b & 15 != 0 {
                return Err("Noncanonical base64 padding".into());
            }
            continue;
        }
        let c = sextet(quantum[2])?;
        bytes.push((b << 4) | (c >> 2));
        if quantum[3] == b'=' {
            if !last || c & 3 != 0 {
                return Err("Noncanonical base64 padding".into());
            }
            continue;
        }
        bytes.push((c << 6) | sextet(quantum[3])?);
    }
    Ok(bytes)
}

pub(crate) fn from_hex(text: &str) -> Result<Vec<u8>, String> {
    if !text.len().is_multiple_of(2) || text.len() > 32_768 {
        return Err("Invalid binary record hex length".into());
    }
    text.as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| {
            let n = |b: u8| match b {
                b'0'..=b'9' => Ok(b - b'0'),
                b'a'..=b'f' => Ok(b - b'a' + 10),
                b'A'..=b'F' => Ok(b - b'A' + 10),
                _ => Err("Non-hex binary record data".to_owned()),
            };
            Ok(n(pair[0])? * 16 + n(pair[1])?)
        })
        .collect()
}
pub(crate) fn to_hex(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        out.push(DIGITS[(b >> 4) as usize] as char);
        out.push(DIGITS[(b & 15) as usize] as char);
    }
    out
}
pub(crate) fn decode_records(text: &str) -> Result<Vec<Value>, String> {
    let bytes = from_hex(text)?;
    decode_record_bytes(&bytes, 32)
}
pub(crate) fn decode_base64_records(text: &str) -> Result<Vec<Value>, String> {
    decode_record_bytes(&from_base64(text)?, MAX_BASE64_RECORDS)
}
fn decode_record_bytes(bytes: &[u8], max_records: usize) -> Result<Vec<Value>, String> {
    let mut records = vec![];
    let mut at = 0;
    while at < bytes.len() {
        if records.len() >= max_records {
            return Err("Binary batch exceeds negotiated record limit".into());
        }
        if bytes.len() - at < 3 {
            return Err("Truncated binary record header".into());
        }
        let kind = bytes[at];
        let length = u16::from_le_bytes([bytes[at + 1], bytes[at + 2]]) as usize;
        at += 3;
        if length > 256 || bytes.len() - at < length {
            return Err("Truncated or oversized binary payload".into());
        }
        let payload = &bytes[at..at + length];
        let mut record = match kind {
            23 => {
                let mut record = crate::legacy::timed_imu_record(payload)?;
                record["t_us"] = json!(record["captured_ns"].as_u64().unwrap() / 1000);
                record["dt_us"] = json!(record["interval_ns"].as_u64().unwrap() / 1000);
                record
            }
            24 => crate::legacy::mapped_gnss_record(payload)?,
            _ => return Err(format!("Unsupported binary USB record kind {kind}")),
        };
        // Exact payload is carried only to the host evidence writer, then
        // removed before live display/projected canonical samples are stored.
        record["_wire_kind"] = json!(kind);
        record["_wire_hex"] = json!(to_hex(payload));
        records.push(record);
        at += length;
    }
    Ok(records)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sensor(index: usize, gnss: bool) -> crate::sample::Sample {
        let at = 9_007_199_254_741_193 + index as u64 * 1_375_000;
        if gnss {
            crate::sample::Sample::Gnss {
                t_us: at / 1000,
                gps_week: 2400,
                gps_tow_ms: 1000 + index as u32 * 20,
                lat: -33.0,
                lon: 151.0,
                alt_m: -0.0,
                speed_mps: 20.0,
                heading_deg: 120.0,
                vertical_mps: -0.3,
                hacc_m: 0.4,
                vacc_m: 0.5,
                sacc_mps: 0.6,
                latency_ms: 0,
                satellites: 17,
                valid: true,
                mapped_ns: Some(at),
                mapping_uncertainty_ns: Some(500_000),
                mapping_generation: Some(7),
            }
        } else {
            crate::sample::Sample::Imu {
                t_us: at / 1000,
                dt_us: 1375,
                accel: [-0.0, 1.25, 9.8],
                gyro: [0.00020720869813913682, -0.00018816830998063866, 0.3],
                valid: true,
                captured_ns: at,
                interval_ns: 1_375_000,
                effective_end_ns: at - 1_500_000,
                timing_uncertainty_ns: 1_200_000,
                aligned_gyro: [0.1, 0.2, 0.3],
            }
        }
    }
    #[test]
    fn firmware_base64_stream_roundtrips_all_padding_modes_and_48_record_payloads() {
        use crate::firmware_response::{Base64Records, Response};
        for count in [0, 1, 2, 3, 31, 32, 47, 48] {
            for pattern in 0..3 {
                let mut response = Response::<8192>::new();
                let mut encoder = Base64Records::new();
                let mut expected = Vec::new();
                let mut payloads = Vec::new();
                for index in 0..count {
                    let sample = sensor(index, pattern == 1 || (pattern == 2 && index % 3 == 0));
                    let mut payload = [0; 128];
                    let (kind, _, len) = crate::firmware_record_codec::encode(sample, &mut payload);
                    response
                        .record_base64(&mut encoder, kind, &payload[..len], 399)
                        .unwrap();
                    expected.push(kind);
                    expected.extend_from_slice(&(len as u16).to_le_bytes());
                    expected.extend_from_slice(&payload[..len]);
                    payloads.push((kind, payload[..len].to_vec()));
                }
                response.finish_base64(&mut encoder).unwrap();
                assert_eq!(
                    from_base64(response.as_str()).unwrap(),
                    expected,
                    "count={count},pattern={pattern}"
                );
                let records = decode_base64_records(response.as_str()).unwrap();
                assert_eq!(records.len(), count);
                for (index, (record, (kind, payload))) in records.iter().zip(payloads).enumerate() {
                    assert_eq!(
                        from_hex(record["_wire_hex"].as_str().unwrap()).unwrap(),
                        payload
                    );
                    let original = serde_json::to_value(sensor(index, kind == 24)).unwrap();
                    if kind == 23 {
                        assert_eq!(record["captured_ns"], original["captured_ns"]);
                        assert_eq!(
                            record["accel"][0].as_f64().unwrap().to_bits(),
                            (-0.0f64).to_bits()
                        );
                        assert_eq!(
                            record["gyro"][0].as_f64().unwrap().to_bits(),
                            0.00020720869813913682f64.to_bits()
                        );
                    } else {
                        assert_eq!(record["mapped_ns"], original["mapped_ns"]);
                        assert_eq!(
                            record["alt_m"].as_f64().unwrap().to_bits(),
                            (-0.0f64).to_bits()
                        );
                    }
                }
                if count == 48 && pattern == 1 {
                    assert_eq!(response.as_str().len(), 7552);
                    assert_eq!(expected.len(), 5664);
                }
            }
        }
    }
    #[test]
    fn excessive_record_counts_and_bad_suffixes_do_not_publish_valid_prefixes() {
        use crate::firmware_response::{Base64Records, Response};
        let mut response = Response::<8192>::new();
        let mut encoder = Base64Records::new();
        for index in 0..49 {
            let mut payload = [0; 128];
            let (kind, _, len) =
                crate::firmware_record_codec::encode(sensor(index, false), &mut payload);
            response
                .record_base64(&mut encoder, kind, &payload[..len], 0)
                .unwrap();
        }
        response.finish_base64(&mut encoder).unwrap();
        assert!(
            from_base64(response.as_str()).is_ok(),
            "49 IMUs fit the byte bound but exceed the record bound"
        );
        assert!(decode_base64_records(response.as_str()).is_err());
        response.clear();
        let mut encoder = Base64Records::new();
        let mut payload = [0; 128];
        let (kind, _, len) = crate::firmware_record_codec::encode(sensor(0, false), &mut payload);
        response
            .record_base64(&mut encoder, kind, &payload[..len], 0)
            .unwrap();
        response.record_base64(&mut encoder, 99, &[], 0).unwrap();
        response.finish_base64(&mut encoder).unwrap();
        assert!(decode_base64_records(response.as_str()).is_err());
    }
    #[test]
    fn base64_requires_standard_alphabet_canonical_padding_and_bounded_length() {
        for (text, bytes) in [
            ("", ""),
            ("Zg==", "f"),
            ("Zm8=", "fo"),
            ("Zm9v", "foo"),
            ("Zm9vYg==", "foob"),
            ("Zm9vYmE=", "fooba"),
            ("Zm9vYmFy", "foobar"),
        ] {
            assert_eq!(from_base64(text).unwrap(), bytes.as_bytes());
        }
        for text in [
            "Zg", "Zg=", "Zh==", "Zm9=", "Zg==AAAA", "Zg======", "AA=A", "=AAA", "____", "----",
            "Zm 8", "Zg==\n",
        ] {
            assert!(from_base64(text).is_err(), "{text:?}");
        }
        assert_eq!(from_base64(&"A".repeat(7552)).unwrap().len(), 5664);
        assert!(from_base64(&"A".repeat(7556)).is_err());
        assert!(
            decode_base64_records("AAAA").is_err(),
            "valid base64 is not necessarily a valid record batch"
        );
    }
    #[test]
    fn actual_firmware_payload_retains_exact_nanoseconds_and_raw_vector_bits() {
        let at = 9_007_199_254_741_193;
        let sample = crate::sample::Sample::Imu {
            t_us: at / 1000,
            dt_us: 1355,
            accel: [-0.0, 1.25, 9.8],
            gyro: [0.00020720869813913682, -0.00018816830998063866, 0.3],
            valid: true,
            captured_ns: at,
            interval_ns: 1_355_933,
            effective_end_ns: at - 1_500_000,
            timing_uncertainty_ns: 1_200_000,
            aligned_gyro: [0.1, 0.2, 0.3],
        };
        let mut payload = [0; 128];
        let (kind, _, len) = crate::firmware_record_codec::encode(sample, &mut payload);
        let mut bytes = vec![kind, len as u8, 0];
        bytes.extend_from_slice(&payload[..len]);
        let decoded = decode_records(&to_hex(&bytes)).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0]["captured_ns"].as_u64(), Some(at));
        assert_eq!(
            decoded[0]["gyro"],
            serde_json::to_value(sample).unwrap()["gyro"]
        );
        assert_eq!(
            decoded[0]["accel"][0].as_f64().unwrap().to_bits(),
            (-0.0f64).to_bits()
        );
        assert_eq!(
            from_hex(decoded[0]["_wire_hex"].as_str().unwrap()).unwrap(),
            payload[..len]
        );
    }
    #[test]
    fn partial_unknown_and_bad_hex_batches_fail_without_publishing_a_prefix() {
        for text in ["00", "17010000", "ff0000", "xx", "1"] {
            assert!(decode_records(text).is_err(), "{text}");
        }
        assert!(decode_records("").unwrap().is_empty());
    }
}
