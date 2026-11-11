# Initial application interface

This document describes protocol 1, schema 1. The authoritative field order and
types are in `src/protocol.rs`, `src/commands.rs`, `src/device/interface.rs` and
`src/device/timing.rs`. This is the first explicitly versioned firmware schema;
older development images cannot be assumed compatible. Measurements remain
unqualified (`Snapshot.qualified=false`): factory scale, nominal filter delay,
identity mounting and a zero nominal antenna offset are development settings.

The current race build uses `DATA_COLLECTION_ONLY`: it records original sensor
data to SD and supplies acquisition/fix diagnostics. Discovery clears the ECEF
navigation capability and sets SD recording. CPU1 owns persistence, so this build
does not run the live trajectory estimator. See [race-recording.md](race-recording.md) for
USB control, resumable downloads, file integrity and offline exports.

## Connect, discover, poll

1. Open the J6 USB CDC interface with DTR asserted. VID is `0xC0C0`; application
   PID is `0xFEED`, recovery PID is `0xCAFE`. This development board uses USB serial
   string `AEVIADEVBOARD`; it is not a unique production-device identifier. CDC
   line speed is not the GNSS UART speed; the host probe uses 115200.
2. Send `System::ProtocolInfo`: complete frame `01 02 08 00`. Decode its fixed
   bootstrap payload below. Require supported `protocol_version` and
   `schema_version` before decoding other telemetry. `System::Error(Command)`
   means the image predates discovery; report unsupported firmware.
3. In recovery, application acquisition is stopped. `System::Run`
   (`01 02 02 00`) requests application mode and responds `System::Reboot`.
   Close, rediscover and reopen the new PID after reboot, then query discovery
   again. If startup returns to recovery, read ResetStatus and report that
   failure rather than repeatedly sending Run.
4. Poll Status or Latest at 10 Hz for the current snapshot. Poll diagnostics
   at about 1 Hz. Faster polling returns repeated snapshots; it does not raise
   the 10 Hz publication rate. BootStatus stage 10 means the worker is ready,
   not that there is a fix. GNSS cold start and estimator initialization may
   take longer; keep acquisition/fix/timing states visible independently.
5. Keep one request outstanding. Replies preserve request order but have no
   transaction identifier. After timeout or disconnect, stop displaying cached
   telemetry as live, close/reopen, clear the receive buffer and rediscover.
   Use a host timeout of at least 3 seconds; startup/peripheral work can briefly
   delay USB. A successful reply does not imply a fresh navigation epoch.

## Framing and serialization

Each frame is one Postcard value, COBS encoded, with one terminating `0x00`.
Collect bytes across USB reads until the delimiter: CDC packet boundaries are
not frame boundaries. A single read may contain multiple frames. Request limits
are 272 bytes including COBS/delimiter; the current response limit is 4608 bytes.
Bootstrap discovery fits 512 bytes; then allocate/reconfigure from its advertised
limit. Earlier images advertised 512. Existing telemetry layouts and smaller
responses are unchanged; only recording downloads use the larger allowance. Reject
oversized frames through their delimiter. Require exactly one decoded value
with no trailing Postcard bytes. The firmware's `protocol::Frame` implements
these rules, and `tools/board-probe` emits JSON from complete hex response lines.
Its `--describe` argument prints its compiled application ProtocolInfo as JSON
without contacting hardware, useful for matching host tools to release bundles.

Postcard uses unsigned LEB128 for unsigned integers wider than one byte and
enum indices, byte-valued `u8`/bool, little-endian IEEE754 floats, and fields in
Rust declaration order. Fixed arrays have no length prefix. `Option<T>` is a
`0`/`1` discriminator followed by `T` only for `Some`; byte slices have a varint
length. Signed integers wider than one byte use ZigZag before LEB128 encoding.
Rust structs are positional, not named maps. Do not send JSON or text to the
device. Use a tested Postcard implementation or the shared Rust types.

Outer envelope tags: System=0, Device=1, Recording=2, for both requests and
responses. Recording is appended; all previous outer and inner indices remain
unchanged. Recording is currently available in application mode only.
Unknown commands, malformed frames and unsupported commands in the current
mode return `System::Error(Command)` and abort an in-progress upload. Other
system error indices are Size=1, Sequence=2, Image=3, Hash=4, Flash=5, Layout=6.
Never infer a successful upload from a transport write alone.

## Bootstrap discovery

System command index 8 and response index 5 are `ProtocolInfo`. Its field order
is frozen independently of the telemetry schema. New discovery fields require
a different command/response. Unknown mask bits can be ignored.

| Field | Wire type | Meaning |
| --- | --- | --- |
| protocol_version | u16 | Framing/envelope contract; 1 |
| schema_version | u16 | Existing telemetry payload contract; 1 |
| firmware_version | [u16; 3] | Cargo major, minor, patch; separate from wire compatibility |
| recovery | bool | Current mode |
| max_request_frame | u16 | 272, including COBS/delimiter |
| max_response_frame | u16 | 4608 in this build, including COBS/delimiter; use the advertised value |
| system_commands | u32 | Bit N: system command N supported in this mode |
| device_commands | u32 | Bit N: device command N supported in this mode |
| capabilities | u32 | Features implemented by this build, independent of present health |
| fitted_accessories | u16 | Compiled accessory selection, not automatic detection |
| snapshot_interval_us | u32 | 100000 (10 Hz) |
| navigation_max_age_us | u32 | 320000; includes configured fusion/smoothing delay |
| publication_max_age_us | u32 | 500000; worker publication expiry |

Capability bits: 0 USB request/reply, 1 inactive-slot firmware update, 2 ECEF
navigation, 3 IMU, 4 GNSS, 5 power diagnostics, 6 radio, 7 SD recording, 8 battery
charging. The current collection build sets bits 0, 1, 3, 4, 5, 7 and 8; live ECEF
navigation and radio are unavailable. Charging support requires both the fitted
battery selection and the enabled charge profile; it is not a current-flow flag.
Availability of SD recording does not mean
a card is mounted or recording successfully; query Recording Status. Accessory
bits: 0 LEDs, 1 display, 2 buttons, 3 battery. LEDs and battery are fitted in the current
build; `src/device/configuration.rs` is the shared source for startup/discovery.
Antenna presence and sky visibility are not detected by this mask.

Existing field layouts/meanings cannot change without a schema-version change.
Appending new command/response variants preserves existing layouts and is
discoverable through command masks. Never reorder enums. Firmware version alone
is not a decoder compatibility check. Discovery describes support, not an
acceptance-test result or measurement qualification.

## Command and response indices

| System command | Index | Available in | Response |
| --- | --- | --- | --- |
| Info | 0 | Both | Info (2): recovery bool, max_size u32, chunk u16 |
| Abort | 1 | Recovery | Ok (0) |
| Run | 2 | Recovery | Reboot (1) |
| Begin | 3 | Recovery | Ok; body size u32, SHA-256 [u8;32] |
| Data | 4 | Recovery | Ok; body offset u32, byte slice, maximum 256 bytes |
| Commit | 5 | Recovery | Reboot |
| Recover | 6 | Application | Reboot |
| BootStatus | 7 | Both | BootStatus (4): stage u8 |
| ProtocolInfo | 8 | Both | ProtocolInfo (5) |

System::Error is response index 3. Info in application reports zero update
size/chunk; switch to recovery and query Info before uploading. A full updater
example is `tools/board_check.py`. Run/Recover mode is persisted across reboot.
An upload requires ordered offsets, complete size, matching SHA-256 and a valid
ESP application image. Commit selects the candidate and reboots. Device-side
confirmation requires sustained acquisition/worker health; an app must reconnect
and inspect diagnostics instead of treating Commit as proof of a healthy boot.

| Device command | Request index | Response index | Recovery availability |
| --- | --- | --- | --- |
| Status | 0 | Snapshot=0 | No |
| Latest | 1 | Snapshot=0 | No |
| SetIndicators | 2 | Indicators=1 | No |
| PowerStatus | 3 | PowerStatus=2 | Yes |
| PeripheralStatus | 4 | PeripheralStatus=3 | No |
| AcquisitionStatus | 5 | AcquisitionStatus=4 | No |
| GnssStatus | 6 | GnssStatus=5 | No |
| ImuDiagnostics | 7 | ImuDiagnostics=6 | No |
| ResetStatus | 8 | ResetStatus=7 | Yes |
| GnssTransportStatus | 9 | GnssTransportStatus=8 | No |
| PowerDiagnostics | 10 | PowerDiagnostics=9 | Yes |
| GnssFixStatus | 11 | GnssFixStatus=10 | No |
| PowerChargeStatus | 12 | PowerChargeStatus=11 | Yes |

Status frame is `02 01 01 00`; Latest is `03 01 01 00`. Other no-body device
commands use `03 01 NN 00`, where `NN` is the request index. SetIndicators body
is `display_enabled: bool, led_brightness: u8` (0..255). Its echoed response
acknowledges the desired volatile setting; PeripheralStatus reports whether
hardware is ready. A disabled/unfitted display is not an error.

## Telemetry interpretation

All names ending `_us` are microseconds, `_ns` nanoseconds and `_ms` milliseconds.
Device timestamps are monotonic boot/session time, not Unix time or UTC. Reset
invalidates their relationship to previous samples. Poll AcquisitionStatus to
read current uptime; `Snapshot.updated_us` is its worker publication time, not
the time of the host request. `epoch_us` is the trajectory's measurement epoch.
Do not replace a missing epoch with USB arrival time.

| Snapshot field(s) | Units and meaning |
| --- | --- |
| position_ecef_m | Optional [f64;3], WGS84 ECEF X/Y/Z metres, at IMU sensing center |
| velocity_ecef_mps | Optional [f32;3], ECEF X/Y/Z metres/second at the same epoch/point |
| horizontal_speed_mps | Optional f32, nonnegative speed in the WGS84 ellipsoid tangent plane |
| vertical_speed_mps | Optional f32, signed along ellipsoid normal; positive upward |
| acceleration_mps2 | [f32;3], last raw factory-scaled sensor specific force, includes gravity; sensor X/Y/Z axes |
| angular_rate_rps | [f32;3], last raw sensor X/Y/Z angular rates, radians/second |
| last_imu_us | Last IMU hardware capture time processed by worker; zero before any sample |
| last_gnss_us | Last GNSS arrival processed by worker; zero before any sample |
| satellites | Satellite count retained from latest GNSS sample reaching worker; not a live fix-validity flag |
| imu_rate_hz, gnss_rate_hz | Observed raw admitted rates, updated about once/second; not requested receiver settings |
| imu_samples, gnss_samples | Raw acquisition sample counters (u32 wrapping) |
| imu_received, gnss_received | Inputs observed by worker, including subsequently rejected ones |
| imu_accepted, gnss_fused | Estimator acceptance/fusion counters; raw acquisition does not imply fusion |
| input_rejected, too_late | Estimator rejection/late-observation counts |
| imu_errors, gnss_errors, gnss_uart_errors, gnss_parse_errors | Cumulative error evidence, retained across recovery |
| processing_errors, error_kind | Cumulative processing failures and last failure kind; a retained error is not by itself current failure |
| imu_drops, gnss_drops | Queue overflow losses; never silently repeated measurements |
| maximum_step_us | Maximum measured estimator processing-step duration since boot |

`Phase` indices: Starting=0, Initializing=1, Navigating=2, Degraded=3, Failed=4,
Recording=5. Recording is an appended state for the collection build. It means
the storage worker is recording; it does not imply navigation, a GNSS fix,
loss-free storage, calibration or measurement qualification. Its optional
navigation values remain absent. Poll recording status for queue/write health.
Starting/Initializing may have valid raw sensor data and no navigation. Failed
reports an estimator/startup failure while USB diagnostics remain usable.
Degraded can retain recent predicted navigation briefly; present it as degraded.
Only show normal navigation when phase is Navigating and the required optional
fields are present. `qualified=false` remains distinct from a functioning fix.
No orientation/quaternion, covariance or RTCM-input command is promised by this
interface. Recording commands use their own appended namespace below.

An old/missing/future trajectory epoch clears all optional navigation fields.
Recent USB replies or new input arrivals cannot renew a frozen trajectory.
Worker publication older than 500 ms also clears navigation. A stale IMU clears
navigation values at publication; stale GNSS or missing PPS lock degrades the
phase. The raw IMU arrays, rate values, satellite count and retained counters
are diagnostic last values and do not become Option::None on expiry: use their
timestamps and acquisition/IMU status before displaying them as live. On host
disconnect or lack of new publications, independently expire displayed data
using host monotonic time; no device query can invalidate an offline UI cache.

`TimingStatus` gives PPS lock, pulse/pair/reject counters, optional pulse age in
milliseconds, fit residual in nanoseconds, fit generation, capture failures,
IMU timing rejections and unsynchronized GNSS inputs. A valid GNSS receiver fix
alone does not establish PPS synchronization or estimator fusion. Configuration
verification, trusted navigation time and enough valid PPS labels are required.

AcquisitionStatus reads raw rates, counters, queue depths/high-water values,
latest accepted sample times and timing diagnostics without depending on the
worker. Its `imu_max_read_latency_us` includes rejected late reads. Depths and
counters are sampled separately across cores, not as an atomic sensor epoch.
Compare counter deltas after startup settling; counters normally reset at boot.
Only raw u32 sample counts should be treated as modulo 2^32. Error/drop counters
are diagnostic totals; an unchanged nonzero total may describe a startup event.

GnssFixStatus is the independent receiver solution; it remains queryable while
PPS association or estimator initialization is incomplete. ECEF position and
velocity are independently optional and have their own `*_valid`, `*_sigma`
and `*_age_us` fields. Each expires after 500000 us, including receiver output
delay plus position solution age or velocity latency. Repeated reports of a
frozen epoch do not refresh it. Sigma arrays are receiver-reported one-sigma
uncertainties in metres and metres/second; they are not fused covariances.
`gps_epoch_ms` is GPST milliseconds since 1980-01-06 only when receiver time is
trusted; it is not Unix/UTC and requires a leap-second conversion for UTC.
`received_us`/`received_age_us` refer to the last message's monotonic arrival.
Statuses, type numbers, time-status bytes and satellite count are retained raw
UM980 values; consult the pinned HAL definitions for numeric meanings, and use
the explicit validity flags for display. `solution_age_s`, `differential_age_s`
and `velocity_latency_s` are optional finite nonnegative receiver values in
seconds. `last_pps` retains the latest raw pulse report: receiver/header GPS
epochs, quality/config, signed pulse error in ns, output delay in ms, receive
time, raw signed `offset_time_10ps` in 10-picosecond units, and local
capture/pulse timestamps in ns. These are diagnostic evidence
and do not themselves declare synchronization. Raw fix and fused navigation
must be labeled separately; both remain unqualified.

PowerStatus verification and peripheral readings expire after three seconds
without fresh owner updates. Battery voltage is microvolts, battery SOC is
percent, and absent battery values are None. Buttons are an optional active-high
mask in SW3, SW6, SW4, SW5 order and have no control actions. The current build
polls the fitted battery gauge, leaves absent buttons unavailable, and applies
the conservative charge profile described below. ResetStatus,
ImuDiagnostics, PowerDiagnostics and GNSS transport/parser records retain
failure evidence across later success; inspect current readiness and deltas
alongside those records. Retention is normally within one boot; ResetStatus
explicitly includes the preceding boot's checksum-validated panic evidence.

PowerChargeStatus is a separate, appended read-only response. Its field order is:

```
updated_us:Option<u64>, verified:bool, charging_enabled:Option<bool>,
charge_current_ma:Option<u16>, charge_voltage_mv:Option<u16>,
input_current_limit_ma:Option<u16>, charger_status0:Option<u8>,
charger_status1:Option<u8>, fault_status:Option<u8>,
precharge_current_ma:Option<u16>, termination_current_ma:Option<u16>,
charger_control0:Option<u8>, charger_control1:Option<u8>, ntc_control0:Option<u8>
```

These are actual charger register readbacks. Current fields are configured
limits in milliamperes; the voltage field is the regulation limit in millivolts.
They are not measured battery current or voltage. The applied profile is
800 mA charge current, 4100 mV regulation, 80 mA precharge and 40 mA termination.
The input-current limit, power-path configuration and NTC protection are retained;
their readbacks let the host inspect the resulting state. Raw status/control/fault
bytes use the BQ25622 register definitions in the pinned HAL.

`updated_us` is the charger owner's monotonic publication time. Readbacks are
optional until obtained, and verification expires after three seconds without
fresh owner data. `verified` in both power responses means the applied policy
was verified; consult the charging capability and PowerChargeStatus rather than
assuming a verified policy always disables charging. `charging_enabled` reports
the enable control, not active charging. Input availability, temperature,
battery state and faults can inhibit current even when charging is enabled;
a full battery may validly be in termination/idle. Inspect `fault_status` and
the raw status bytes separately. There is no host command to change charge
limits, enable charging, or bypass protection in this interface.

## Minimal Rust polling

```rust,ignore
use firmware::{commands, device::interface::DeviceCommand, protocol};
let mut request = [0; protocol::FRAME_SIZE];
let bytes = postcard::to_slice_cobs(
    &protocol::Request::System(commands::Command::ProtocolInfo), &mut request)?;
// write_all(bytes); collect one complete zero-terminated reply.
let mut decoder = protocol::Frame::<{ protocol::RESPONSE_FRAME_SIZE }>::new();
let mut incoming: &[u8] = received_bytes;
let discovery = decoder.feed::<protocol::Response>(&mut incoming);
// Require ProtocolInfo { protocol_version: 1, schema_version: 1, .. }.
let bytes = postcard::to_slice_cobs(
    &protocol::Request::Device(DeviceCommand::Latest), &mut request)?;
// Poll, await reply, then schedule the next request. Do not block sensor workers.
```

The host crate compiles without ESP hardware. Build `tools/board-probe` for
`x86_64-pc-windows-msvc` on Windows and use its JSON output when developing a
decoder in another language. Wire fixtures and frame/limit/freshness regressions
run in the firmware host test suite; this does not replace the connected-board
acceptance procedure in `evaluation.md`.
