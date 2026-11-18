//! BESTNAVXYZ fields used to import legacy AEVLOG1 recordings (Unicore N4 R1.14).
use super::framing::{BinaryFrame, DecodeError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// Raw time reference: 0 is GPST in the existing N4 profile.
    pub time_reference: u8,
    /// Raw time quality: 160 is FINE in the existing N4 profile.
    pub time_status: u8,
    pub week: u16,
    pub milliseconds_of_week: u32,
}

impl Header {
    fn from_bytes(bytes: &[u8]) -> Self {
        Self {
            time_reference: bytes[8],
            time_status: bytes[9],
            week: u16_at(bytes, 10),
            milliseconds_of_week: u32_at(bytes, 12),
        }
    }
    /// GPST milliseconds since its epoch, only for the existing verified
    /// GPST/FINE header profile and a timestamp inside a GPS week.
    pub fn gps_epoch_ms(self) -> Option<u64> {
        if self.time_reference != 0
            || self.time_status != 160
            || self.milliseconds_of_week >= 604_800_000
        {
            return None;
        }
        Some(u64::from(self.week) * 604_800_000 + u64::from(self.milliseconds_of_week))
    }
}

impl BinaryFrame<'_> {
    pub fn best_nav_xyz(self) -> Result<BestNavXyz, DecodeError> {
        if self.message_id() != 240 {
            return Err(DecodeError::MessageType);
        }
        let p = self.payload();
        if p.len() != 112 {
            return Err(DecodeError::Length);
        }
        Ok(BestNavXyz {
            header: Header::from_bytes(self.header_bytes()),
            position_status: u32_at(p, 0),
            position_type: u32_at(p, 4),
            position_ecef_m: core::array::from_fn(|i| f64_at(p, 8 + i * 8)),
            position_sigma_m: core::array::from_fn(|i| f32_at(p, 32 + i * 4)),
            velocity_status: u32_at(p, 44),
            velocity_type: u32_at(p, 48),
            velocity_ecef_mps: core::array::from_fn(|i| f64_at(p, 52 + i * 8)),
            velocity_sigma_mps: core::array::from_fn(|i| f32_at(p, 76 + i * 4)),
            velocity_latency_s: f32_at(p, 92),
            differential_age_s: f32_at(p, 96),
            solution_age_s: f32_at(p, 100),
            satellites_used: p[105],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BestNavXyz {
    pub header: Header,
    pub position_status: u32,
    pub position_type: u32,
    pub position_ecef_m: [f64; 3],
    pub position_sigma_m: [f32; 3],
    pub velocity_status: u32,
    pub velocity_type: u32,
    pub velocity_ecef_mps: [f64; 3],
    pub velocity_sigma_mps: [f32; 3],
    pub velocity_latency_s: f32,
    /// Age of differential corrections, seconds; not applicable to standalone
    /// position or Doppler velocity. Validate before using correction-based fixes.
    pub differential_age_s: f32,
    /// Age of the position solution, seconds, distinct from velocity latency.
    pub solution_age_s: f32,
    pub satellites_used: u8,
}

impl BestNavXyz {
    /// The receiver reported a computed, non-NONE finite position with positive
    /// standard deviations. Application age, fix type and quality limits remain
    /// the caller's decision; this does not qualify the physical installation.
    pub fn position_valid(self) -> bool {
        self.position_status == 0
            && self.position_type != 0
            && self.position_ecef_m.iter().all(|value| value.is_finite())
            && self
                .position_sigma_m
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
    }

    pub fn velocity_valid(self) -> bool {
        self.velocity_status == 0
            && self.velocity_type != 0
            && self.velocity_ecef_mps.iter().all(|value| value.is_finite())
            && self
                .velocity_sigma_mps
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
            && self.velocity_latency_s.is_finite()
            && self.velocity_latency_s >= 0.0
    }
}

fn u16_at(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
}

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn f32_at(bytes: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn f64_at(bytes: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}
