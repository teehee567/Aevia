//! Native N4 SPPNAVB (ID 46). Framing/CRC belongs to `timing::wire`.
//!
//! The N4 R1.14 SPPNAV and reference tables define the 24-byte header,
//! 120-byte body and solution enums. `NativeSpp` only copies bytes/integers;
//! float decoding and navigation rounding must run in ordinary task context.

pub const MESSAGE_ID: u16 = 46;
pub const PAYLOAD_BYTES: usize = 120;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    Format,
    Status,
    NonFinite,
    Range,
    Datum,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NativeSpp {
    pub time_reference: u8,
    pub time_status: u8,
    pub gps_week: u16,
    pub gps_tow_ms: u32,
    pub version: u32,
    pub output_delay_ms: u16,
    /// Original little-endian payload, including all original float bits.
    pub body: [u8; PAYLOAD_BYTES],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurement {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    /// Orthometric (MSL) height; add undulation for ellipsoid height.
    pub height_m: f64,
    pub undulation_m: f64,
    pub latitude_sigma_m: f64,
    pub longitude_sigma_m: f64,
    pub height_sigma_m: f64,
    pub latency_s: f64,
    pub horizontal_speed_m_s: f64,
    pub track_deg: f64,
    pub vertical_speed_m_s: f64,
    pub vertical_speed_sigma_m_s: f64,
    pub horizontal_speed_sigma_m_s: f64,
}

impl NativeSpp {
    /// Accept an already CRC-checked header and body, without its four-byte CRC.
    /// No float loads, arithmetic or validation occurs here.
    pub fn from_frame(frame: &[u8]) -> Result<Self, ParseError> {
        if frame.len() != 24 + PAYLOAD_BYTES
            || frame[..3] != [0xaa, 0x44, 0xb5]
            || u16::from_le_bytes([frame[4], frame[5]]) != MESSAGE_ID
            || u16::from_le_bytes([frame[6], frame[7]]) != PAYLOAD_BYTES as u16
        {
            return Err(ParseError::Format);
        }
        let mut body = [0; PAYLOAD_BYTES];
        body.copy_from_slice(&frame[24..]);
        let value = Self {
            time_reference: frame[8],
            time_status: frame[9],
            gps_week: u16::from_le_bytes([frame[10], frame[11]]),
            gps_tow_ms: u32_at(frame, 12),
            version: u32_at(frame, 16),
            output_delay_ms: u16::from_le_bytes([frame[22], frame[23]]),
            body,
        };
        if value.position_status() > 4 || value.velocity_status() > 4 {
            return Err(ParseError::Status);
        }
        Ok(value)
    }

    pub fn position_status(&self) -> u32 {
        u32_at(&self.body, 0)
    }
    pub fn position_type(&self) -> u32 {
        u32_at(&self.body, 4)
    }
    pub fn velocity_status(&self) -> u32 {
        u32_at(&self.body, 72)
    }
    pub fn velocity_type(&self) -> u32 {
        u32_at(&self.body, 76)
    }
    pub fn satellites_tracked(&self) -> u8 {
        self.body[64]
    }
    pub fn satellites_used(&self) -> u8 {
        self.body[65]
    }

    /// Only a GPST/FINE header is suitable for mapping onto the PPS clock.
    /// A non-FINE/no-fix frame is still retained for diagnostics and rate checks.
    pub fn qualified_epoch_ms(&self) -> Option<u64> {
        if self.time_reference != 0 || self.time_status != 160 {
            return None;
        }
        crate::timing::gps_epoch_ms(u32::from(self.gps_week), self.gps_tow_ms)
    }

    /// Ordinary context only. Time qualification remains a separate decision:
    /// a caller must not label a non-GPST header as a GPS measurement epoch.
    /// Legitimate no-fix/unsupported solutions may contain arbitrary sentinels.
    #[inline(never)]
    pub fn measurement(&self) -> Result<Option<Measurement>, ParseError> {
        if self.position_status() > 4 || self.velocity_status() > 4 {
            return Err(ParseError::Status);
        }
        if self.position_status() != 0
            || self.velocity_status() != 0
            || !matches!(self.position_type(), 16..=18)
            || self.velocity_type() != 8
        {
            return Ok(None);
        }
        if u32_at(&self.body, 36) != 61 {
            return Err(ParseError::Datum);
        }
        let value = Measurement {
            latitude_deg: f64_at(&self.body, 8),
            longitude_deg: f64_at(&self.body, 16),
            height_m: f64_at(&self.body, 24),
            undulation_m: f32_at(&self.body, 32),
            latitude_sigma_m: f32_at(&self.body, 40),
            longitude_sigma_m: f32_at(&self.body, 44),
            height_sigma_m: f32_at(&self.body, 48),
            latency_s: f32_at(&self.body, 80),
            horizontal_speed_m_s: f64_at(&self.body, 88),
            track_deg: f64_at(&self.body, 96),
            vertical_speed_m_s: f64_at(&self.body, 104),
            vertical_speed_sigma_m_s: f32_at(&self.body, 112),
            horizontal_speed_sigma_m_s: f32_at(&self.body, 116),
        };
        if ![
            value.latitude_deg,
            value.longitude_deg,
            value.height_m,
            value.undulation_m,
            value.latitude_sigma_m,
            value.longitude_sigma_m,
            value.height_sigma_m,
            value.latency_s,
            value.horizontal_speed_m_s,
            value.track_deg,
            value.vertical_speed_m_s,
            value.vertical_speed_sigma_m_s,
            value.horizontal_speed_sigma_m_s,
        ]
        .iter()
        .all(|number| number.is_finite())
        {
            return Err(ParseError::NonFinite);
        }
        if !(-90.0..=90.0).contains(&value.latitude_deg)
            || !(-180.0..=180.0).contains(&value.longitude_deg)
            || !(0.0..=360.0).contains(&value.track_deg)
            || value.horizontal_speed_m_s < 0.0
            || value.latency_s < 0.0
            || value.latency_s > f64::from(u32::MAX) / 1000.0
            || value.latitude_sigma_m < 0.0
            || value.longitude_sigma_m < 0.0
            || value.height_sigma_m < 0.0
            || value.vertical_speed_sigma_m_s < 0.0
            || value.horizontal_speed_sigma_m_s < 0.0
            || self.satellites_used() == 0
            || self.satellites_used() > self.satellites_tracked()
        {
            return Err(ParseError::Range);
        }
        Ok(Some(value))
    }
}

impl Measurement {
    /// Ordinary-context adapter for the existing integer navigation interface.
    /// USB/SD samples must use `Measurement` directly to retain native precision.
    /// A receiver-reported zero uncertainty is retained; downstream navigation
    /// qualification decides whether that uncertainty is usable.
    pub fn navigation(&self, raw: &NativeSpp) -> Option<crate::gnss::OwnedBestNav> {
        let position_type: &[u8] = match raw.position_type() {
            16 => b"SINGLE",
            17 => b"PSRDIFF",
            18 => b"SBAS",
            _ => return None,
        };
        if raw.position_status() != 0 || raw.velocity_status() != 0 || raw.velocity_type() != 8 {
            return None;
        }
        Some(
            crate::gnss::BestNav {
                gps_week: u32::from(raw.gps_week),
                gps_tow_ms: raw.gps_tow_ms,
                latitude_e7: scaled_i32(self.latitude_deg, 10_000_000.0)?,
                longitude_e7: scaled_i32(self.longitude_deg, 10_000_000.0)?,
                height_mm: scaled_i32(self.height_m, 1000.0)?,
                undulation_mm: scaled_i32(self.undulation_m, 1000.0)?,
                latitude_sigma_mm: scaled_u32(self.latitude_sigma_m, 1000.0)?,
                longitude_sigma_mm: scaled_u32(self.longitude_sigma_m, 1000.0)?,
                height_sigma_mm: scaled_u32(self.height_sigma_m, 1000.0)?,
                satellites_tracked: raw.satellites_tracked(),
                satellites_used: raw.satellites_used(),
                position_type,
                velocity_type: b"DOPPLER_VELOCITY",
                latency_ms: scaled_u32(self.latency_s, 1000.0)?,
                horizontal_speed_mm_s: scaled_u32(self.horizontal_speed_m_s, 1000.0)?,
                track_hundredths_deg: scaled_u32(self.track_deg, 100.0)?,
                vertical_speed_mm_s: scaled_i32(self.vertical_speed_m_s, 1000.0)?,
                vertical_speed_sigma_mm_s: scaled_u32(self.vertical_speed_sigma_m_s, 1000.0)?,
                horizontal_speed_sigma_mm_s: scaled_u32(self.horizontal_speed_sigma_m_s, 1000.0)?,
            }
            .to_owned(),
        )
    }
}

fn scaled_i32(value: f64, scale: f64) -> Option<i32> {
    let rounded = libm::round(value * scale);
    (rounded.is_finite() && rounded >= f64::from(i32::MIN) && rounded <= f64::from(i32::MAX))
        .then_some(rounded as i32)
}
fn scaled_u32(value: f64, scale: f64) -> Option<u32> {
    let rounded = libm::round(value * scale);
    (value >= 0.0 && rounded.is_finite() && rounded <= f64::from(u32::MAX))
        .then_some(rounded as u32)
}
fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}
fn f32_at(bytes: &[u8], offset: usize) -> f64 {
    f64::from(f32::from_bits(u32_at(bytes, offset)))
}
fn f64_at(bytes: &[u8], offset: usize) -> f64 {
    f64::from_bits(u64::from_le_bytes(
        bytes[offset..offset + 8].try_into().unwrap(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> [u8; 144] {
        let mut frame = [0; 144];
        frame[..3].copy_from_slice(&[0xaa, 0x44, 0xb5]);
        frame[4..6].copy_from_slice(&MESSAGE_ID.to_le_bytes());
        frame[6..8].copy_from_slice(&(PAYLOAD_BYTES as u16).to_le_bytes());
        frame[9] = 160;
        frame[10..12].copy_from_slice(&2400u16.to_le_bytes());
        frame[12..16].copy_from_slice(&604_799_980u32.to_le_bytes());
        frame[16..20].copy_from_slice(&17548u32.to_le_bytes());
        frame[22..24].copy_from_slice(&7u16.to_le_bytes());
        let b = &mut frame[24..];
        b[4..8].copy_from_slice(&16u32.to_le_bytes());
        b[36..40].copy_from_slice(&61u32.to_le_bytes());
        b[64] = 24;
        b[65] = 19;
        b[76..80].copy_from_slice(&8u32.to_le_bytes());
        for (offset, value) in [
            (8, -33.1234567890123f64),
            (16, 151.9876543210987),
            (24, 82.123456789),
            (88, 12.123456789),
            (96, 273.123456789),
            (104, -0.123456789),
        ] {
            b[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [
            (32, 23.75f32),
            (40, 0.125),
            (44, 0.25),
            (48, 0.5),
            (80, 0.0075),
            (112, 0.03125),
            (116, 0.015625),
        ] {
            b[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        frame
    }

    #[test]
    fn native_bits_precision_offsets_and_epoch_survive_ownership() {
        let frame = fixture();
        let raw = NativeSpp::from_frame(&frame).unwrap();
        assert_eq!(&raw.body, &frame[24..]);
        assert!(core::mem::size_of::<NativeSpp>() <= 144);
        assert_eq!(raw.version, 17548);
        assert_eq!(raw.output_delay_ms, 7);
        assert_eq!(raw.satellites_tracked(), 24);
        assert_eq!(raw.satellites_used(), 19);
        assert_eq!(raw.qualified_epoch_ms(), Some(2401 * 604_800_000 - 20));
        let m = raw.measurement().unwrap().unwrap();
        assert_eq!(m.latitude_deg.to_bits(), (-33.1234567890123f64).to_bits());
        assert_eq!(m.longitude_deg.to_bits(), 151.9876543210987f64.to_bits());
        assert_eq!(m.height_m.to_bits(), 82.123456789f64.to_bits());
        assert_eq!(m.horizontal_speed_m_s.to_bits(), 12.123456789f64.to_bits());
        assert_eq!(m.track_deg.to_bits(), 273.123456789f64.to_bits());
        assert_eq!(m.vertical_speed_m_s.to_bits(), (-0.123456789f64).to_bits());
        assert_eq!(m.vertical_speed_sigma_m_s, 0.03125);
        assert_eq!(m.horizontal_speed_sigma_m_s, 0.015625);
    }

    #[test]
    fn framing_status_and_time_qualification_are_separate() {
        let frame = fixture();
        for end in 0..144 {
            assert_eq!(
                NativeSpp::from_frame(&frame[..end]),
                Err(ParseError::Format)
            );
        }
        for offset in [0, 4, 6] {
            let mut bad = frame;
            bad[offset] ^= 1;
            assert_eq!(NativeSpp::from_frame(&bad), Err(ParseError::Format));
        }
        for offset in [24, 24 + 72] {
            let mut bad = frame;
            bad[offset..offset + 4].copy_from_slice(&5u32.to_le_bytes());
            assert_eq!(NativeSpp::from_frame(&bad), Err(ParseError::Status));
        }
        let mut raw = NativeSpp::from_frame(&frame).unwrap();
        raw.time_status = 20;
        assert_eq!(raw.qualified_epoch_ms(), None);
        assert!(raw.measurement().unwrap().is_some());
        raw.time_status = 160;
        raw.time_reference = 1;
        assert_eq!(raw.qualified_epoch_ms(), None);
        raw.time_reference = 0;
        raw.gps_tow_ms = 604_800_000;
        assert_eq!(raw.qualified_epoch_ms(), None);
    }

    #[test]
    fn no_fix_sentinels_do_not_become_measurements_or_parse_failures() {
        for status in 1u32..=4 {
            let mut frame = fixture();
            frame[24..28].copy_from_slice(&status.to_le_bytes());
            frame[32..40].copy_from_slice(&f64::NAN.to_le_bytes());
            let raw = NativeSpp::from_frame(&frame).unwrap();
            assert!(raw.qualified_epoch_ms().is_some());
            assert_eq!(raw.measurement(), Ok(None));
        }
        for pos_type in [0u32, 1, 8, 50, u32::MAX] {
            let mut raw = NativeSpp::from_frame(&fixture()).unwrap();
            raw.body[4..8].copy_from_slice(&pos_type.to_le_bytes());
            assert_eq!(raw.measurement(), Ok(None));
        }
        for pos_type in [16u32, 17, 18] {
            let mut raw = NativeSpp::from_frame(&fixture()).unwrap();
            raw.body[4..8].copy_from_slice(&pos_type.to_le_bytes());
            assert!(raw.measurement().unwrap().is_some());
        }
    }

    #[test]
    fn computed_fix_rejects_invalid_numbers_uncertainty_datum_and_counts() {
        let original = NativeSpp::from_frame(&fixture()).unwrap();
        for offset in [8, 16, 24, 88, 96, 104] {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut raw = original;
                raw.body[offset..offset + 8].copy_from_slice(&bad.to_le_bytes());
                assert_eq!(raw.measurement(), Err(ParseError::NonFinite));
            }
        }
        for offset in [32, 40, 44, 48, 80, 112, 116] {
            let mut raw = original;
            raw.body[offset..offset + 4].copy_from_slice(&f32::NAN.to_le_bytes());
            assert_eq!(raw.measurement(), Err(ParseError::NonFinite));
        }
        for (offset, value) in [
            (8, 90.000001f64),
            (16, -180.000001),
            (88, -0.01),
            (96, 360.01),
        ] {
            let mut raw = original;
            raw.body[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
            assert_eq!(raw.measurement(), Err(ParseError::Range));
        }
        for offset in [40, 44, 48, 112, 116] {
            for sigma in [-0.1f32, -f32::MIN_POSITIVE] {
                let mut raw = original;
                raw.body[offset..offset + 4].copy_from_slice(&sigma.to_le_bytes());
                assert_eq!(raw.measurement(), Err(ParseError::Range));
            }
        }
        let mut raw = original;
        raw.body[36] = 0;
        assert_eq!(raw.measurement(), Err(ParseError::Datum));
        for count in [0, 25] {
            let mut raw = original;
            raw.body[65] = count;
            assert_eq!(raw.measurement(), Err(ParseError::Range));
        }
    }

    #[test]
    fn zero_reported_uncertainty_is_preserved_and_latency_cannot_saturate() {
        let mut raw = NativeSpp::from_frame(&fixture()).unwrap();
        for offset in [40, 44, 48, 112, 116] {
            raw.body[offset..offset + 4].copy_from_slice(&0.0f32.to_le_bytes());
        }
        let measurement = raw.measurement().unwrap().unwrap();
        assert_eq!(measurement.latitude_sigma_m, 0.0);
        assert_eq!(measurement.longitude_sigma_m, 0.0);
        assert_eq!(measurement.height_sigma_m, 0.0);
        assert_eq!(measurement.vertical_speed_sigma_m_s, 0.0);
        assert_eq!(measurement.horizontal_speed_sigma_m_s, 0.0);
        let owned = measurement.navigation(&raw).unwrap();
        let nav = owned.as_nav();
        assert_eq!(nav.latitude_sigma_mm, 0);
        assert_eq!(nav.longitude_sigma_mm, 0);
        assert_eq!(nav.height_sigma_mm, 0);
        assert_eq!(nav.vertical_speed_sigma_mm_s, 0);
        assert_eq!(nav.horizontal_speed_sigma_mm_s, 0);
        raw.body[80..84].copy_from_slice(&4_294_967.0f32.to_le_bytes());
        let measurement = raw.measurement().unwrap().unwrap();
        assert_eq!(
            measurement.navigation(&raw).unwrap().as_nav().latency_ms,
            4_294_967_000
        );
        for latency in [4_294_968.0f32, f32::MAX, -0.001] {
            raw.body[80..84].copy_from_slice(&latency.to_le_bytes());
            assert_eq!(raw.measurement(), Err(ParseError::Range));
        }
    }

    #[test]
    fn navigation_rounding_is_checked_and_does_not_replace_native_values() {
        let raw = NativeSpp::from_frame(&fixture()).unwrap();
        let mut m = raw.measurement().unwrap().unwrap();
        let owned = m.navigation(&raw).unwrap();
        let nav = owned.as_nav();
        assert_eq!(nav.latitude_e7, -331234568);
        assert_eq!(nav.longitude_e7, 1519876543);
        assert_eq!(nav.height_mm, 82123);
        assert_eq!(nav.horizontal_speed_mm_s, 12123);
        assert_eq!(nav.track_hundredths_deg, 27312);
        assert_eq!(nav.vertical_speed_mm_s, -123);
        assert_eq!(nav.position_type, b"SINGLE");
        assert_eq!(nav.velocity_type, b"DOPPLER_VELOCITY");
        assert_eq!(m.latitude_deg.to_bits(), (-33.1234567890123f64).to_bits());
        m.height_m = f64::from(i32::MAX) / 1000.0;
        assert_eq!(m.navigation(&raw).unwrap().as_nav().height_mm, i32::MAX);
        m.height_m += 0.001;
        assert!(m.navigation(&raw).is_none());
        m.height_m = f64::NAN;
        assert!(m.navigation(&raw).is_none());
        assert_eq!(scaled_i32(-1.5, 1.0), Some(-2));
        assert_eq!(scaled_u32(-0.0001, 1.0), None);
        assert_eq!(scaled_u32(f64::from(u32::MAX) + 1.0, 1.0), None);
    }
}
