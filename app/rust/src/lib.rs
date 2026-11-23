pub mod api;
mod binary;
#[cfg(any(windows, target_os = "ios"))]
mod ble;
mod board_recording;
#[path = "../../../firmware/src/display.rs"]
mod display;
mod driving;
mod evidence;
mod frb_generated;
mod legacy;
mod legacy_um980;
pub mod model;
mod ppk;
mod service;
mod trajectory_adapter;

// Exercise the real firmware encoder against the host importer without pulling
// embedded runtime dependencies into the desktop build.
#[cfg(test)]
#[path = "../../../firmware/src/storage/codec.rs"]
mod firmware_record_codec;
#[allow(dead_code)]
#[path = "../../../firmware/src/storage/format.rs"]
mod format;
#[cfg(test)]
#[path = "../../../firmware/src/sample.rs"]
mod sample;

#[cfg(test)]
#[allow(dead_code)]
#[allow(clippy::byte_char_slices)]
#[path = "../../../firmware/src/ppk.rs"]
mod firmware_ppk;

#[cfg(test)]
mod gnss {
    pub fn unicore_crc32(bytes: &[u8]) -> u32 {
        crate::legacy_um980::crc32(bytes)
    }
}

#[cfg(test)]
#[allow(dead_code)]
#[allow(clippy::write_literal)]
#[path = "../../../firmware/src/response.rs"]
mod firmware_response;
#[cfg(test)]
mod storage {
    pub(crate) mod codec {
        pub(crate) use crate::firmware_record_codec::encode;
    }
}
