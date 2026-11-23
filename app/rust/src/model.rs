use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Sample {
    pub time_s: f64,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub speed_mps: Option<f64>,
    pub altitude_m: Option<f64>,
    pub heading_deg: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub course_deg: Option<f64>,
    pub accel_long_g: Option<f64>,
    pub accel_lat_g: Option<f64>,
    pub valid: bool,
    /// True only when time_s was normalized from an advancing receiver epoch.
    #[serde(default)]
    pub receiver_timed: bool,
    #[serde(default)]
    pub velocity_latency_ms: Option<u32>,
    /// Receiver epoch mapped into the device capture clock. These fields are
    /// evidence only; receiver-only metrics continue to use receiver time_s.
    #[serde(default)]
    pub mapped_ns: Option<u64>,
    #[serde(default)]
    pub mapping_uncertainty_ns: Option<u64>,
    #[serde(default)]
    pub mapping_generation: Option<u32>,
    #[serde(default)]
    pub gps_week: Option<u32>,
    #[serde(default)]
    pub gps_tow_ms: Option<u32>,
    #[serde(default)]
    pub hacc_m: Option<f64>,
    #[serde(default)]
    pub vacc_m: Option<f64>,
    #[serde(default)]
    pub sacc_mps: Option<f64>,
    #[serde(default)]
    pub vertical_mps: Option<f64>,
    #[serde(default)]
    pub satellites: Option<u8>,
    #[serde(default)]
    pub ppk_quality: Option<Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Gate {
    pub lat: f64,
    pub lon: f64,
    pub heading_deg: f64,
    pub width_m: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Track {
    pub id: String,
    pub name: String,
    pub start: Gate,
    #[serde(default)]
    pub sectors: Vec<Gate>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DragConfig {
    pub start_kph: f64,
    pub end_kph: f64,
    pub distance_m: f64,
    pub rollout: bool,
}
impl Default for DragConfig {
    fn default() -> Self {
        Self {
            start_kph: 0.0,
            end_kph: 100.0,
            distance_m: 402.336,
            rollout: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Lap {
    pub number: u32,
    pub start_s: f64,
    pub end_s: f64,
    pub duration_s: f64,
    pub valid: bool,
    pub sectors_s: Vec<f64>,
    pub max_speed_mps: Option<f64>,
}
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DragRun {
    pub number: u32,
    pub start_s: f64,
    pub end_s: f64,
    pub speed_time_s: Option<f64>,
    pub distance_time_s: Option<f64>,
    pub trap_speed_mps: Option<f64>,
    pub slope_percent: Option<f64>,
    pub valid: bool,
    pub reason: Option<String>,
}
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Analysis {
    pub laps: Vec<Lap>,
    pub drag_runs: Vec<DragRun>,
    pub best_lap_s: Option<f64>,
    pub theoretical_best_s: Option<f64>,
    pub average_lap_s: Option<f64>,
    pub median_lap_s: Option<f64>,
    pub distance_m: f64,
    pub max_speed_mps: Option<f64>,
    pub average_speed_mps: Option<f64>,
    pub valid_samples: usize,
    pub engine: String,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Session {
    pub format_version: u32,
    pub id: String,
    pub name: String,
    pub created_ms: u64,
    pub duration_s: f64,
    pub sample_count: usize,
    pub max_speed_mps: Option<f64>,
    pub distance_m: f64,
    pub source: String,
    #[serde(default)]
    pub provenance: Value,
    /// Persisted capture interruption; analysis must not erase incomplete evidence.
    #[serde(default)]
    pub incomplete_reason: Option<String>,
    pub samples: Vec<Sample>,
    #[serde(default)]
    pub raw_records: Vec<Value>,
    #[serde(default)]
    pub track: Option<Track>,
    #[serde(default)]
    pub drag: DragConfig,
    #[serde(default)]
    pub analysis: Analysis,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Preferences {
    pub tracks: Vec<Track>,
    pub drag: DragConfig,
}
