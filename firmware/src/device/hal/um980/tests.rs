use super::*;
use std::{vec, vec::Vec};

fn binary(id: u16, payload: &[u8]) -> Vec<u8> {
    let mut wire = vec![0; 24];
    wire[..3].copy_from_slice(&[0xaa, 0x44, 0xb5]);
    wire[4..6].copy_from_slice(&id.to_le_bytes());
    wire[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    wire[9] = 160;
    wire[10..12].copy_from_slice(&2400_u16.to_le_bytes());
    wire[12..16].copy_from_slice(&1234_u32.to_le_bytes());
    wire.extend_from_slice(payload);
    wire.extend_from_slice(&crc32(&wire).to_le_bytes());
    wire
}

#[test]
fn crc_uses_n4_zero_seed_and_no_final_xor() {
    assert_eq!(crc32(b"123456789"), 0x2dfd_2d88);
    assert_eq!(crc32(b""), 0);
}

#[test]
fn binary_decoder_handles_noise_overlap_and_unknown_messages() {
    let wire = binary(12345, &[1, 2, 3, 4]);
    let mut decoder = BinaryDecoder::<64>::new();
    for byte in [0, 0xff, 0xaa, 0x44, 0xaa] {
        assert_eq!(decoder.push(byte), Ok(None));
    }
    let mut frames = 0;
    for byte in wire {
        if let Some(frame) = decoder.push(byte).unwrap() {
            assert_eq!(frame.message_id(), 12345);
            assert_eq!(frame.payload(), &[1, 2, 3, 4]);
            assert_eq!(frame.best_nav_xyz(), Err(DecodeError::MessageType));
            frames += 1;
        }
    }
    assert_eq!(frames, 1);
}

#[test]
fn binary_rejects_oversize_and_crc_then_recovers() {
    let mut decoder = BinaryDecoder::<32>::new();
    let too_long = [0xaa, 0x44, 0xb5, 0, 0, 0, 0xff, 0xff];
    for byte in &too_long[..7] {
        assert_eq!(decoder.push(*byte), Ok(None));
    }
    assert_eq!(decoder.push(too_long[7]), Err(DecodeError::Oversized));
    let mut corrupt = binary(37, &[]);
    corrupt[20] ^= 1;
    let mut errors = 0;
    for byte in corrupt {
        if decoder.push(byte) == Err(DecodeError::Checksum) {
            errors += 1;
        }
    }
    assert_eq!(errors, 1);
    let mut frames = 0;
    for byte in binary(37, &[]) {
        if decoder.push(byte).unwrap().is_some() {
            frames += 1;
        }
    }
    assert_eq!(frames, 1);
    assert_eq!(
        BinaryDecoder::<0>::new().push(0),
        Err(DecodeError::BufferTooSmall)
    );
    decoder.push(0xaa).unwrap();
    decoder.reset();
    assert_eq!(decoder.buffered_len(), 0);
}

#[test]
fn bestnav_decodes_little_endian_fields_without_fabricating_validity() {
    let mut payload = [0; 112];
    payload[4..8].copy_from_slice(&16_u32.to_le_bytes());
    payload[48..52].copy_from_slice(&16_u32.to_le_bytes());
    payload[96..100].copy_from_slice(&0.375_f32.to_le_bytes());
    payload[100..104].copy_from_slice(&0.125_f32.to_le_bytes());
    for i in 0..3 {
        payload[8 + i * 8..16 + i * 8].copy_from_slice(&(i as f64 - 10.5).to_le_bytes());
        payload[32 + i * 4..36 + i * 4].copy_from_slice(&0.5_f32.to_le_bytes());
        payload[52 + i * 8..60 + i * 8].copy_from_slice(&(i as f64 + 2.25).to_le_bytes());
        payload[76 + i * 4..80 + i * 4].copy_from_slice(&0.25_f32.to_le_bytes());
    }
    payload[92..96].copy_from_slice(&0.125_f32.to_le_bytes());
    payload[105] = 23;
    let wire = binary(240, &payload);
    let frame = BinaryFrame::parse(&wire).unwrap();
    let sample = frame.best_nav_xyz().unwrap();
    assert_eq!(sample.position_ecef_m, [-10.5, -9.5, -8.5]);
    assert_eq!(sample.velocity_ecef_mps, [2.25, 3.25, 4.25]);
    assert_eq!(sample.satellites_used, 23);
    assert_eq!(
        sample.header.gps_epoch_ms(),
        Some(2400 * 604_800_000 + 1234)
    );
    assert!(sample.position_valid());
    assert!(sample.velocity_valid());
    let mut invalid = sample;
    invalid.position_ecef_m[1] = f64::NAN;
    invalid.velocity_latency_s = f32::NAN;
    assert!(!invalid.position_valid());
    assert!(!invalid.velocity_valid());
    let wrong_length = binary(240, &[]);
    assert_eq!(
        BinaryFrame::parse(&wrong_length).unwrap().best_nav_xyz(),
        Err(DecodeError::Length)
    );
    assert_eq!(sample.differential_age_s, 0.375);
    assert_eq!(sample.solution_age_s, 0.125);
}

#[test]
fn pps_decodes_signed_error_and_rejects_invalid_week_time() {
    let mut payload = [0; 60];
    payload[4..8].copy_from_slice(&2400_u32.to_le_bytes());
    payload[8..12].copy_from_slice(&2000_u32.to_le_bytes());
    payload[12..16].copy_from_slice(&(-18_i32).to_le_bytes());
    let wire = binary(9000, &payload);
    let sample = BinaryFrame::parse(&wire).unwrap().pps_status().unwrap();
    assert_eq!(sample.pulse_error_ns, -18);
    assert_eq!(sample.gps_epoch_ms(), Some(2400 * 604_800_000 + 2000));
    let mut invalid = sample;
    invalid.header.time_reference = 1;
    assert_eq!(invalid.gps_epoch_ms(), None);
    invalid = sample;
    invalid.milliseconds_of_week = 604_800_000;
    assert_eq!(invalid.gps_epoch_ms(), None);
}

#[test]
fn ascii_checks_standard_nmea_and_distinct_config_checksum() {
    let nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    assert_eq!(
        AsciiSentence::parse(nmea).unwrap().kind(),
        SentenceKind::Nmea
    );
    assert_eq!(
        AsciiSentence::parse(b"$GPGGA,123*00"),
        Err(DecodeError::Checksum)
    );
    let body = b"$CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0";
    let checksum = body.iter().fold(0, |xor, byte| xor ^ byte);
    let config = std::format!(
        "{}*{:02X}\r\n",
        core::str::from_utf8(body).unwrap(),
        checksum
    );
    let sentence = AsciiSentence::parse(config.as_bytes()).unwrap();
    assert_eq!(sentence.kind(), SentenceKind::UnicoreConfig);
    assert!(sentence.confirms_gps_rising_one_hz());
    let mut decoder = AsciiDecoder::<128>::new();
    for byte in b"junk$bad incomplete" {
        assert_eq!(decoder.push(*byte), Ok(None));
    }
    let mut count = 0;
    for byte in nmea {
        if decoder.push(*byte).unwrap().is_some() {
            count += 1;
        }
    }
    assert_eq!(count, 1);
    assert_eq!(
        AsciiDecoder::<0>::new().push(b'$'),
        Err(DecodeError::BufferTooSmall)
    );
}

#[test]
fn mixed_decoder_does_not_interpret_ascii_inside_binary_payload() {
    let mut decoder = Decoder::<128>::new();
    let nmea = b"$A*41\r\n";
    let wire = binary(123, nmea);
    let mut frames = Vec::new();
    for byte in wire.iter().chain(nmea) {
        if let Some(frame) = decoder.push(*byte).unwrap() {
            frames.push(match frame {
                Frame::Binary(value) => (1, value.message_id()),
                Frame::Ascii(_) => (2, 0),
            });
        }
    }
    assert_eq!(frames, vec![(1, 123), (2, 0)]);
}

#[derive(Default)]
struct Uart {
    output: Vec<u8>,
    flushes: u32,
    zero: bool,
}
impl embedded_io::ErrorType for Uart {
    type Error = embedded_io::ErrorKind;
}
impl embedded_io::Write for Uart {
    fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
        if self.zero {
            return Ok(0);
        }
        let len = bytes.len().min(2);
        self.output.extend_from_slice(&bytes[..len]);
        Ok(len)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flushes += 1;
        Ok(())
    }
}

#[test]
fn commands_are_complete_before_io_and_handle_short_writes() {
    let mut receiver = Um980::new(Uart::default());
    assert_eq!(
        receiver.command(Command::Raw("CONFIG\r\nSAVECONFIG")),
        Err(Error::Command(CommandError::InvalidCharacters))
    );
    assert!(receiver.io_mut().output.is_empty());
    receiver
        .command(Command::Log {
            port: Port::Com2,
            message: LogMessage::BestNavXyzBinary,
            period: Period::from_millis(20).unwrap(),
        })
        .unwrap();
    receiver
        .command(Command::SetBaud {
            port: Port::Com2,
            baud: Baud::B460800,
        })
        .unwrap();
    assert_eq!(
        receiver.io_mut().output,
        b"BESTNAVXYZB COM2 0.020\r\nCONFIG COM2 460800\r\n"
    );
    assert_eq!(receiver.io_mut().flushes, 2);
    receiver.io_mut().zero = true;
    assert_eq!(
        receiver.command(Command::QueryConfig),
        Err(Error::WriteZero)
    );
    let mut small = [0xff; 3];
    assert_eq!(
        Command::QueryConfig.encode(&mut small),
        Err(CommandError::BufferTooSmall)
    );
    assert_eq!(small, [0xff; 3]);
    assert_eq!(Period::from_millis(0), Err(CommandError::PeriodOutOfRange));
}
