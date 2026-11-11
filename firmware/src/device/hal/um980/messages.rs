//! Typed layouts evidenced by existing `device/gnss.rs` (Unicore N4 R1.14).
//! Values are decoded without assuming a valid solution, PPS association or
//! time synchronization. Callers can inspect invalid/faulted receiver output.
use super::{BinaryFrame, DecodeError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub message_id: u16,
    /// Raw time reference: 0 is GPST in the existing N4 profile.
    pub time_reference: u8,
    /// Raw time quality: 160 is FINE in the existing N4 profile.
    pub time_status: u8,
    pub week: u16,
    pub milliseconds_of_week: u32,
    pub output_delay_ms: u16,
}

impl Header {
    pub(super) fn from_bytes(bytes: &[u8]) -> Self {
        Self {
            message_id: u16_at(bytes, 4),
            time_reference: bytes[8],
            time_status: bytes[9],
            week: u16_at(bytes, 10),
            milliseconds_of_week: u32_at(bytes, 12),
            output_delay_ms: u16_at(bytes, 22),
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
    pub fn header(self) -> Header {
        Header::from_bytes(self.header_bytes())
    }

    pub fn best_nav_xyz(self) -> Result<BestNavXyz, DecodeError> {
        if self.message_id() != 240 {
            return Err(DecodeError::MessageType);
        }
        let p = self.payload();
        if p.len() != 112 {
            return Err(DecodeError::Length);
        }
        Ok(BestNavXyz {
            header: self.header(),
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

    pub fn pps_status(self) -> Result<PpsStatus, DecodeError> {
        if self.message_id() != 9000 {
            return Err(DecodeError::MessageType);
        }
        let p = self.payload();
        if p.len() != 60 {
            return Err(DecodeError::Length);
        }
        Ok(PpsStatus {
            header: self.header(),
            quality: u32_at(p, 0),
            week: u32_at(p, 4),
            milliseconds_of_week: u32_at(p, 8),
            pulse_error_ns: i32::from_le_bytes(p[12..16].try_into().unwrap()),
            offset_time_10ps: i32::from_le_bytes(p[16..20].try_into().unwrap()),
            config: u32_at(p, 20),
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PpsStatus {
    pub header: Header,
    pub quality: u32,
    pub week: u32,
    pub milliseconds_of_week: u32,
    pub pulse_error_ns: i32,
    /// PPS offset relative to observation time, in 0.01 ns units (N4 R1.2).
    pub offset_time_10ps: i32,
    pub config: u32,
}

impl PpsStatus {
    /// A timestamp alone does not authenticate the PPS configuration or
    /// associate this message with a captured electrical edge.
    pub fn gps_epoch_ms(self) -> Option<u64> {
        self.header.gps_epoch_ms()?;
        if self.milliseconds_of_week >= 604_800_000 {
            return None;
        }
        Some(u64::from(self.week) * 604_800_000 + u64::from(self.milliseconds_of_week))
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
