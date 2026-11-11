use crate::loader::update::Error;
use serde::Deserialize;

// System::Data with a 256-byte payload, worst-case u32 offset, and COBS overhead.
pub const FRAME_SIZE: usize = 272;
/// Maximum COBS response size, including the terminating zero.
pub const RESPONSE_FRAME_SIZE: usize = 4608;

pub const PROTOCOL_VERSION: u16 = 1;
/// First frozen app schema. Increment for any change to an existing payload's
/// field order/type or variant meaning; append-only commands use capability masks.
pub const SCHEMA_VERSION: u16 = 1;

pub mod capability {
    pub const USB_POLL: u32 = 1 << 0;
    pub const FIRMWARE_UPDATE: u32 = 1 << 1;
    pub const ECEF_NAVIGATION: u32 = 1 << 2;
    pub const IMU: u32 = 1 << 3;
    pub const GNSS: u32 = 1 << 4;
    pub const POWER_DIAGNOSTICS: u32 = 1 << 5;
    pub const RADIO: u32 = 1 << 6;
    pub const SD_RECORDING: u32 = 1 << 7;
    pub const BATTERY_CHARGING: u32 = 1 << 8;
}

pub mod accessory {
    pub const LEDS: u16 = 1 << 0;
    pub const DISPLAY: u16 = 1 << 1;
    pub const BUTTONS: u16 = 1 << 2;
    pub const BATTERY: u16 = 1 << 3;
}

/// Bootstrap payload: its layout is frozen independently of SCHEMA_VERSION.
/// New discovery fields require a new command/response, never extending this
/// struct. Unknown mask bits may be ignored. Availability is not health.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, Deserialize)]
pub struct ProtocolInfo {
    pub protocol_version: u16,
    pub schema_version: u16,
    pub firmware_version: [u16; 3],
    pub recovery: bool,
    pub max_request_frame: u16,
    pub max_response_frame: u16,
    /// Bit N means system command index N is available in the current mode.
    pub system_commands: u32,
    /// Bit N means device command index N is available in the current mode.
    pub device_commands: u32,
    /// Features implemented by this build, independent of mode or sensor health.
    pub capabilities: u32,
    /// Compiled accessory selection; antenna presence is not auto-detected.
    pub fitted_accessories: u16,
    pub snapshot_interval_us: u32,
    pub navigation_max_age_us: u32,
    pub publication_max_age_us: u32,
}

const fn version_component(value: &str) -> u16 {
    let bytes = value.as_bytes();
    let mut result: u16 = 0;
    let mut index = 0;
    while index < bytes.len() {
        assert!(bytes[index].is_ascii_digit());
        result = match result.checked_mul(10) {
            Some(value) => value,
            None => panic!("firmware version component exceeds u16"),
        };
        result = match result.checked_add((bytes[index] - b'0') as u16) {
            Some(value) => value,
            None => panic!("firmware version component exceeds u16"),
        };
        index += 1;
    }
    result
}

pub fn info(recovery: bool) -> ProtocolInfo {
    use crate::device::{
        configuration,
        interface::{self, DeviceCommand},
    };
    // Enum order is checked against encoded wire tags in the regression suite.
    let device_commands = [
        DeviceCommand::Status,
        DeviceCommand::Latest,
        DeviceCommand::SetIndicators(interface::Indicators::default()),
        DeviceCommand::PowerStatus,
        DeviceCommand::PeripheralStatus,
        DeviceCommand::AcquisitionStatus,
        DeviceCommand::GnssStatus,
        DeviceCommand::ImuDiagnostics,
        DeviceCommand::ResetStatus,
        DeviceCommand::GnssTransportStatus,
        DeviceCommand::PowerDiagnostics,
        DeviceCommand::GnssFixStatus,
        DeviceCommand::PowerChargeStatus,
    ]
    .iter()
    .enumerate()
    .fold(0, |mask, (index, command)| {
        mask | if !recovery || command.available_in_recovery() {
            1 << index
        } else {
            0
        }
    });
    ProtocolInfo {
        protocol_version: PROTOCOL_VERSION,
        schema_version: SCHEMA_VERSION,
        firmware_version: [
            const { version_component(env!("CARGO_PKG_VERSION_MAJOR")) },
            const { version_component(env!("CARGO_PKG_VERSION_MINOR")) },
            const { version_component(env!("CARGO_PKG_VERSION_PATCH")) },
        ],
        recovery,
        max_request_frame: FRAME_SIZE as u16,
        max_response_frame: RESPONSE_FRAME_SIZE as u16,
        system_commands: if recovery {
            0b1_1011_1111
        } else {
            0b1_1100_0001
        },
        device_commands,
        capabilities: capability::USB_POLL
            | capability::FIRMWARE_UPDATE
            | if configuration::DATA_COLLECTION_ONLY {
                0
            } else {
                capability::ECEF_NAVIGATION
            }
            | capability::IMU
            | capability::GNSS
            | capability::POWER_DIAGNOSTICS
            | capability::SD_RECORDING
            | if configuration::BATTERY_FITTED && configuration::CHARGING_ENABLED {
                capability::BATTERY_CHARGING
            } else {
                0
            },
        fitted_accessories: accessory::LEDS
            | if configuration::DISPLAY_FITTED {
                accessory::DISPLAY
            } else {
                0
            }
            | if configuration::BUTTONS_FITTED {
                accessory::BUTTONS
            } else {
                0
            }
            | if configuration::BATTERY_FITTED {
                accessory::BATTERY
            } else {
                0
            },
        snapshot_interval_us: interface::SNAPSHOT_INTERVAL_US as u32,
        navigation_max_age_us: interface::NAVIGATION_MAX_AGE_US as u32,
        publication_max_age_us: interface::PUBLICATION_MAX_AGE_US as u32,
    }
}

/// Fixed-memory framing shared by system and device transports.
/// An oversized frame is discarded through its delimiter, never decoded as a tail.
pub struct Frame<const N: usize = FRAME_SIZE> {
    bytes: [u8; N],
    len: usize,
    discard: bool,
}

impl<const N: usize> Frame<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            discard: false,
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.discard = false;
    }

    /// Consume up to one frame; `input` retains any subsequent frames.
    /// Borrowed payloads remain valid until the next feed or clear.
    pub fn feed<'de, T: Deserialize<'de>>(
        &'de mut self,
        input: &mut &[u8],
    ) -> Option<Result<T, Error>> {
        let end = input.iter().position(|&byte| byte == 0);
        let count = end.map_or(input.len(), |end| end + 1);
        let (part, rest) = input.split_at(count);
        *input = rest;
        if !self.discard {
            if count > N.saturating_sub(self.len) {
                self.discard = true;
            } else {
                self.bytes[self.len..self.len + count].copy_from_slice(part);
                self.len += count;
            }
        }
        end?;
        let len = core::mem::take(&mut self.len);
        if core::mem::take(&mut self.discard) {
            return Some(Err(Error::Command));
        }
        let decoded = match cobs::decode_in_place(&mut self.bytes[..len]) {
            Ok(decoded) => decoded,
            Err(_) => return Some(Err(Error::Command)),
        };
        Some(match postcard::take_from_bytes(&self.bytes[..decoded]) {
            Ok((value, [])) => Ok(value),
            _ => Err(Error::Command),
        })
    }
}

impl<const N: usize> Default for Frame<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Protocol v1. Variant order and field types are part of the wire contract.
#[derive(Debug, serde::Serialize, Deserialize)]
pub enum Request<'a> {
    System(#[serde(borrow)] crate::commands::Command<'a>),
    Device(crate::device::interface::DeviceCommand),
    Recording(crate::device::recording::api::Request),
}

#[derive(Debug, serde::Serialize, Deserialize)]
// A fixed-capacity wire envelope avoids heap allocation in the USB path.
#[allow(clippy::large_enum_variant)]
pub enum Response {
    System(crate::commands::Response),
    Device(crate::device::interface::DeviceResponse),
    Recording(crate::device::recording::api::Response),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::{Command, Response as SystemResponse};

    #[test]
    fn discovery_request_is_appended_and_bootstrap_layout_is_frozen() {
        let mut bytes = [0; RESPONSE_FRAME_SIZE];
        assert_eq!(
            postcard::to_slice_cobs(&Request::System(Command::ProtocolInfo), &mut bytes).unwrap(),
            &[1, 2, 8, 0]
        );
        // Deliberately independent of the current build's feature/version values.
        let fixture = ProtocolInfo {
            protocol_version: 1,
            schema_version: 2,
            firmware_version: [3, 4, 5],
            recovery: true,
            max_request_frame: 272,
            max_response_frame: 512,
            system_commands: 511,
            device_commands: 2047,
            capabilities: 63,
            fitted_accessories: 1,
            snapshot_interval_us: 100_000,
            navigation_max_age_us: 320_000,
            publication_max_age_us: 500_000,
        };
        let encoded = postcard::to_slice(
            &Response::System(SystemResponse::ProtocolInfo(fixture)),
            &mut bytes,
        )
        .unwrap();
        assert_eq!(
            encoded,
            &[
                0, 5, 1, 2, 3, 4, 5, 1, 0x90, 2, 0x80, 4, 0xff, 3, 0xff, 15, 63, 1, 0xa0, 0x8d, 6,
                0x80, 0xc4, 19, 0xa0, 0xc2, 30
            ]
        );
        let response = postcard::to_slice_cobs(
            &Response::System(SystemResponse::ProtocolInfo(fixture)),
            &mut bytes,
        )
        .unwrap();
        let mut input: &[u8] = response;
        let mut frame = Frame::<RESPONSE_FRAME_SIZE>::new();
        let Some(Ok(Response::System(SystemResponse::ProtocolInfo(actual)))) =
            frame.feed(&mut input)
        else {
            panic!()
        };
        assert!(input.is_empty());
        assert_eq!(actual, fixture);
    }

    #[test]
    fn discovery_reports_current_build_limits_and_mode_specific_support() {
        let app = info(false);
        let recovery = info(true);
        assert_eq!(app.max_request_frame as usize, FRAME_SIZE);
        assert_eq!(app.max_response_frame as usize, RESPONSE_FRAME_SIZE);
        assert_eq!(app.snapshot_interval_us, 100_000);
        assert_eq!(app.navigation_max_age_us, 320_000);
        assert_eq!(app.publication_max_age_us, 500_000);
        assert_eq!(app.protocol_version, 1);
        assert_eq!(app.schema_version, 1);
        assert_eq!(app.capabilities & capability::RADIO, 0);
        assert_eq!(
            app.capabilities & capability::BATTERY_CHARGING != 0,
            crate::device::configuration::BATTERY_FITTED
                && crate::device::configuration::CHARGING_ENABLED
        );
        assert_ne!(app.capabilities & capability::SD_RECORDING, 0);
        assert_eq!(
            app.capabilities & capability::ECEF_NAVIGATION != 0,
            !crate::device::configuration::DATA_COLLECTION_ONLY
        );
        assert_eq!(app.fitted_accessories & accessory::LEDS, accessory::LEDS);
        assert_eq!(
            app.fitted_accessories & accessory::BATTERY != 0,
            crate::device::configuration::BATTERY_FITTED
        );
        assert_eq!(
            recovery.device_commands,
            (1 << 3) | (1 << 8) | (1 << 10) | (1 << 12)
        );
        assert_eq!(
            app.system_commands,
            (1 << 0) | (1 << 6) | (1 << 7) | (1 << 8)
        );
        assert_eq!(recovery.system_commands, 0x1ff & !(1 << 6));
        for index in 0..13 {
            assert_ne!(app.device_commands & (1 << index), 0);
        }
    }
}
