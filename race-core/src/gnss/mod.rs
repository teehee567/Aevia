use alloc::vec::Vec;

mod m10;
mod um980;

#[allow(unused)]
pub use m10::UbloxReceiver;
#[allow(unused)]
pub use um980::Um980Receiver;

#[derive(Clone, Copy, Debug)]
pub enum GnssUpdate {
    // location spp
    Fix(Fix),

    // amount of sats visible
    Sats { count: u8, best_cno: u8 },
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub speed_mps: f32,
    pub heading_deg: f32,
    pub speed_std_mps: f32,
    pub vel_latency_s: f32,
    pub gps_week: u16,
    pub tow_ms: u32,
    pub has_fix: bool,
}

pub trait GnssReceiver {
    fn baudrate(&self) -> u32;

    // startup
    fn init_bytes(&self) -> Vec<u8>;

    // main run
    fn consume(&mut self, bytes: &[u8], cb: &mut dyn FnMut(GnssUpdate));
}
