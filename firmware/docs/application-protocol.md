# Application telemetry protocol v1

The original loader postcard/COBS framing and variants 0–2 remain unchanged.
Application requests use variant 3, followed by a postcard string (unsigned
LEB128 byte length and UTF-8 JSON), then COBS encoding and a zero delimiter.
Application replies use the same variant 3 string layout. USB packets can split
or combine frames. One request is allowed in flight. DTR must be asserted.

Requests are at most 240 UTF-8 bytes; ordinary control requests have one `op` field:

```json
{"op":"status"}
{"op":"start"}
{"op":"poll"}
{"op":"drain"}
{"op":"drain_binary"}
{"op":"drain_base64"}
{"op":"stop"}
```

`start` is idempotent while recording. It first opens the SD file, with a ten-second deadline, then admits the recording. Allow twelve
seconds for Start and Stop replies. A failed preparation cancels admission; it
cannot start later after reporting failure. A new recording clears queued preview
samples, resets the overflow count and increments the session number. Nullable
`started_at_us` records that transition and survives idempotent starts and stops;
it changes only on a new recording. `stop`
preserves queued USB samples and reports success after the SD queue drains and
the final page flushes. A storage fault or timeout returns an error. Nullable
`stopped_at_us` records the MCU time of
the recording-to-stopped transition, before waiting for that flush. It remains
unchanged across repeated stops and is cleared by the next new start. Hosts
must use this boundary to exclude preview samples produced during the flush;
response `uptime_us` is the later response time. `poll` drains at most 8 entries. Poll again
immediately while `pending` is nonzero. In raw USB mode, preview samples are queued too;
SD receives a separate queue of admitted recording-window samples. Responses are bounded
to 8,192 JSON bytes plus framing. `ok:false` includes a human-readable `error`.
USB uses 4,096 PSRAM samples after successful startup/promotion, with an explicit
128-sample internal fallback. `usb_queue` reports `backing`, `capacity`, `pending`,
`psram_tested` and nullable `error`. SD queues 384 sensor samples and 32 raw chunks.
Starting a recording resets its driving metrics. Navigation stays warm across recording boundaries. The driving estimator uses advancing receiver epochs and bounds time corrections across PPS outages and reacquisition. Status reports PPS, holdover, or arrival-based development timing.

Full responses advertise `compact_poll:true`. Supported hosts should use `drain`
for streaming and request full `status` about once per second. `drain` returns
only `ok`, `protocol:1`, `compact:true`, `recording`, `session`, `uptime_us`,
`started_at_us`, `stopped_at_us`, `pending`, `dropped`, and `samples`. Merge these
fields into the last full status, replacing each response's samples rather than
reusing an earlier array. Compact batches hold up to 16 samples and stop earlier
if the response capacity is reached; the next sample stays queued. `pending`
and `dropped` are sampled after the compact batch is drained. Its single
`uptime_us` anchor is read at the end, after every returned sample. Older hosts and
firmware can continue using full `poll` responses. Start and stop always return
full diagnostics. Full legacy `poll` is also capacity bounded and restores a sample
that does not fit; all response clock anchors follow their returned samples.

Firmware advertises `base64_poll:true` and `binary_poll:true`. Prefer
`drain_base64`, whose compact envelope includes `binary:true` and `records_b64`
instead of `samples`. It holds up to 48 concatenated records encoded as one
canonical RFC 4648 base64 stream, with the standard alphabet and padding only at
the end. The unchanged 8,192-byte response bound includes its metadata. Decode
and validate the whole batch before publishing records; mixed representations,
noncanonical padding or malformed records are errors.

For compatible older hosts, `drain_binary` remains available. Its envelope
has the same compact control/window/pending fields plus `binary:true` and
`records_hex` instead of `samples`. Hex decodes to concatenated records:
`kind:u8`, `payload_length:u16LE`, then the exact payload bytes. Only kind 23
(101-byte IMU) and kind 24 (115-byte GNSS) are currently sent this way, at most
32 per response. No sensor decimation is applied. Decode these payloads using
the SD layouts below and project legacy `t_us = captured_ns / 1000` and
`dt_us = interval_ns / 1000` by integer truncation. Preserve the original ns
fields and f64 values. PPK raw chunks are SD-only. The Python capture client
automatically selects base64, hex, compact JSON, or legacy polling by capability.

Full successful control responses contain `protocol`, `device`, `firmware`, `recording`,
`session`, `uptime_us`, `started_at_us`, `stopped_at_us`, `build_mode`, `pending`, `dropped`,
`compact_poll`, `binary_poll`, `base64_poll`, `usb_queue`, `acquisition_profile`, `ppk`, `gnss`,
`gnss_native`, `gnss_uart`, `imu`, `imu_diagnostics`, `battery`, `buttons`,
`led`, `display`, `storage`, `sd`, `bluetooth`, `timing`, `navigation`,
`installation`, `capture_timing`, `serialization_timing`, `accessories`, and `samples`.

Each sensor health object has `status`, cumulative `count`, cumulative `errors`,
`age_us`, and `latest` (a sample or null). Healthy sensor transport does not mean
a valid GNSS fix. Missing and stale data must not become zero-valued live data.
`dropped` counts USB queue overflow for in-window samples, including delayed
publication during the stop drain. Preview outside the window does not add
recording losses. It resets only for a new recording.
IMU admission requires its entire `[captured_ns - interval_ns, captured_ns]`
support inside the recording window, comparing microsecond boundaries multiplied
by 1000 without rounding the sample endpoint. GNSS uses its observation `t_us`.
These bounds refer to raw captured interval support. The unqualified LPF delay
model shifts effective support earlier; its start can precede Start by the filter
delay. `effective_end_ns` and the associated modeled interval remain unchanged,
without clipping or relabelling them as raw support.
Start excludes pending observations from before the new window. Stop drains
pre-cutoff native IMU intervals and the fixed GNSS observation watermark before
requesting SD closure. An eligible stop marker takes priority over newer preview
intervals, so preview cannot starve the drain. Raw PPK frames use the first byte's
MCU reception timestamp for admission. A frame that started before Start is
excluded entirely; a frame admitted before Stop is completed through its CRC
before closure even if its final bytes arrive after Stop. Undecided headers also
hold this fence. A UART error, 100 ms idle gap, or two-second frame deadline
explicitly truncates/reset the parser and releases the fence; incomplete fragments
remain evidence and must not be exported as valid observations.
`build_mode` is `application` for the complete default build; explicit diagnostic
features identify themselves and do not imply that disabled hardware was tested.
`imu_diagnostics` retains the component/ASIC IDs and the most recent exact
`last_error` plus its MCU `last_error_us`. A past error remains visible after
successful samples; use the health error count and timestamp to assess recurrence.
Its `acquisition` counters separate late starts, SPI burst duration, finishes
past the 1 ms coherence limit, changed DRDY edges, and missing captures.
`burst_total_ns` accumulates measured native burst time, including subsequently
rejected bursts; its delta divided by elapsed MCU nanoseconds measures that work.
`raw_queue_dropped` counts native intervals lost between integer interrupt
acquisition and ordinary-context scaling. Loss changes the generation and
discards the incomplete average rather than bridging the gap.
`serialization_timing` reports maximum wall time of synchronous JSON sections and
COBS encoding, including interrupt preemption, in microseconds. These cumulative diagnostics reset on reboot;
overlapping acquisition rejection reasons must not be summed as distinct samples.
`runtime_timing` adds cumulative microsecond work totals and counts for GNSS
processing/bytes/reads (`gnss_work_us`) and separate interrupt copy work
(`gnss_io_work_us`), IMU scaling/publication, binary encoding, complete response
building, COBS, complete wire transmission, endpoint waits and explicit packet
yields. These are overlapping wall times, not disjoint CPU utilization counters.
Compare deltas across full status snapshots. Binary serialization yields after
four milliseconds of work, without an extra yield after the final record, and
transmission yields after eight 512-byte packets;
sensor interrupt executors remain preemptive during these bounded groups.
`gnss_uart` reports counted UART read errors and last error/port/time. Its
`queue_dropped` counts complete parsed SPP observations lost between integer
acquisition and ordinary publication. `gnss_byte_queue` reports `capacity_bytes:16384`,
`queued_bytes` (including packed headers), `queued_reads`, `published_sequence`,
and per-port `data`/`command` objects with `generation`, `dropped_reads` and
`dropped_bytes`. Only COM1 command/raw reads use this queue; COM2 SPP/PPS is
decoded directly in the integer interrupt executor. Each accepted raw read
retains its original MCU receive timestamp; delayed decoding does not replace it.
FIFO errors or byte-queue loss change the affected port's generation, abandon
incomplete frames after earlier queued bytes are consumed, and increment
`gnss.errors`; indoor no-fix records do not increment transport errors. Stop
first drains the fixed pre-stop read watermark, then completes any admitted raw
frame before closing storage. The independent parsed-SPP watermark must also
finish. New preview frames cannot extend either boundary.
Every closed SD part also writes two kind-1 JSON records immediately before its
kind-22 end record: `{"gnss_transport_loss":{"port":"data","uart_errors":0,
"dropped_reads":0,"dropped_bytes":0,"observation_dropped":0}}` and the corresponding `command` record.
These unsigned counters are cumulative over the recording and its stop drain;
rotation does not reset them. UART error byte counts are unknown, so they remain
separate from exact rejected-read counts/bytes. `observation_dropped` counts
complete parsed SPP observations rejected by their bounded channel; it is zero
on the command port. Kind-22 continues to count only
storage queue losses. Missing transport metadata in older files means unknown,
and zero transport loss does not establish adequate carrier observations.

`acquisition_profile` identifies `SPPNAVB`, `gnss_wire:unicore-binary`,
`imu_wire:safespi-48`, requested GNSS 50 Hz, SIGNALGROUP 8,
COM2 data baud 460800, COM1 raw baud 460800, native DEC5 IMU logging, nominal IMU
737.5 Hz, navigation average length 4, and USB decimation 1. Rates measured from
counter deltas or exact intervals take precedence over nominal sensor clocks.
The receiver stream uses native SPPNAVB message ID 46: a 24-byte header,
120-byte payload and four-byte receiver CRC. Its GPS week/TOW confirms the
requested 20 ms cadence independently of fix validity. Only GPST/FINE headers
may enter PPS mapping; non-GPS time references are rejected for GPS-labelled
sample publication. `gnss_native` contains the latest CRC-validated header and
raw position/velocity status/type codes, with its original receipt timestamp.
These are latest-frame diagnostics, not additional kind-24 fields. Ordinary
context validates and decodes native floating-point values. USB/SD retain native
f64 latitude, longitude, speed, heading and vertical speed; only the navigation
adapter rounds coordinates/velocity into its existing integer interface.
Kind-24 altitude remains MSL height plus geoid undulation. The IMU's physical
interface already uses 48-bit binary SafeSPI requests/replies with CRC8 and
status/counter validation; kind 23 preserves the resulting scaled vectors and
native interval timing.
`ppk` contains `storage:sd-only`, `format:unicore`, `requested_hz:5`, `configured`
(a CRC-valid OBSVM has been observed), `command_errors`, `frames`,
`observation_frames`, `ephemeris_frames`, `bytes`, `crc_failures`, `truncated_frames`,
`maximum_frame_bytes`, `last_observation_count`, `dropped`, `pending`, and nullable
`last_received_us`/`last_observation_us`. Counts are cumulative since boot except
SD raw `dropped`, which resets for each recording. Correct CRC or zero queue drops
does not imply carrier lock, adequate ephemerides, a fix, or lossless UART input.

GNSS sample fields:

| Field | Meaning |
|---|---|
| `kind` | `gnss` |
| `t_us` | MCU monotonic receive time, not calibrated effective time |
| `gps_week`, `gps_tow_ms` | Receiver GPS epoch, no UTC leap-second conversion |
| `lat`, `lon` | WGS84 degrees |
| `alt_m` | Ellipsoid height, MSL height plus reported undulation |
| `speed_mps`, `heading_deg`, `vertical_mps` | Receiver velocity/course |
| `hacc_m` | Maximum of latitude/longitude one-sigma uncertainties |
| `vacc_m`, `sacc_mps` | Vertical position / horizontal speed one-sigma uncertainty |
| `latency_ms` | Receiver-reported velocity latency |
| `satellites` | Satellites used |
| `valid` | Both receiver solutions computed and complete CRC-valid numeric record |
| `mapped_ns` | Optional PPS-mapped local position epoch; null without verified mapping |
| `mapping_uncertainty_ns` | Optional nonzero mapping one-sigma uncertainty |
| `mapping_generation` | Optional mapping segment identity; do not bridge generations silently |

No-fix GNSS records retain epoch and satellite count with `valid:false`; their
zero position/velocity placeholders must never be used for measurements.

IMU sample fields: `kind:imu`, `t_us` (raw capture rounded to microseconds), `dt_us` (rounded support duration),
`accel` (three sensor-frame m/s² specific-force values), `gyro` (three sensor-frame
rad/s angular rates), and `valid:true`. Invalid SafeSPI samples are excluded and
increment errors; support gaps remain observable in the next sample timestamp.
`captured_ns` and `interval_ns` preserve hardware timing precision.
`effective_end_ns` applies the development acceleration LPF delay approximation;
`timing_uncertainty_ns` preserves its nonzero uncertainty. `aligned_gyro` is the
separate gyro series interpolated to the acceleration support. Original `gyro`
values remain unchanged. These factory-scaled sensor-frame values have no
measured vehicle boresight or qualified filter delay. They are not vehicle G.

`battery` reports status plus nullable percent/millivolts, nullable
`charger_error`/`gauge_error`, charger `faults` and cumulative `errors`.
`rate_percent_per_hour` is the signed average rate of change of SOC reported by
the MAX17048. Positive values indicate rising SOC. A usable rate requires a settled
gauge with valid SOC. `power_source` is `external` or `battery` when the charger's
VBUS status agrees with its power-good pin. `charge_stage` is `not_charging`,
`constant_current`, `taper` or `top_off`. Constant current includes trickle,
precharge and fast charge. The charger combines idle and terminated in
`not_charging`. `temperature_zone` decodes the charger's TS thermistor
status. It can report warm, cool, or a sensor fault without measuring a
temperature in degrees. `battery.temperatures_c` is an array of whole-degree
Celsius readings in charger die, IMU, then processor order. The IMU and
processor take a single sample each every ten seconds. Unavailable readings
are null. A fresh charger ADC conversion also reports `charger_temperature_c`
in its response and makes that reading available in status for up to 30
seconds on external power.
`{"op":"power"}` returns `measurement.at_us` on the board clock, the raw
`charger_part`, and a `configuration` object read from the identified charger.
Its fields are `charge_current_limit_ma`, `termination_current_ma`,
`termination_enabled`, `regulation_voltage_mv`, `watchdog_seconds` (0 means
disabled) and `mode` (`host` or `default` before any ADC conversion).
The voltage and current limits are the configured values before thermal or
input regulation.

With an enabled charger watchdog, `power` performs only register reads and
returns `adc:null` with `adc_unavailable_reason:"watchdog-enabled"`. With
external power and a disabled watchdog it takes one fresh ADC sample. Without
external power the reason is `external-power-unavailable`.

`{"op":"power_sample"}` explicitly requests a fresh conversion on external
power even with the watchdog enabled. Use this for bench diagnostics. The ADC
control write can start host mode. An enabled watchdog subsequently expiring
halves the programmed charge current, even after ADC control is restored.
Repeated bench samples across expiry cycles can reduce the current to its
minimum. [TI charger watchdog behavior, section 8.4.1](https://www.ti.com/lit/ds/symlink/bq25622e.pdf#page=30)

A successful conversion returns `adc_unavailable_reason:null` and
`measurement.adc` with `input_mv`, `input_ma`, `battery_mv` and
`battery_ma`. Positive battery current indicates charging. Battery current
is nullable if its direction changes during conversion. Input voltage times
current includes power used for charging.
`buttons` is a nullable
four-bit pressed mask. A missing gauge must not be displayed as zero charge.
`storage:sd-exfat-and-usb-host` and `bluetooth:ble-display-v2` describe this build's
capabilities. `sd` contains state, error, capacity_bytes, file_id, written_pages,
dropped, pending, raw_pending, raw_dropped, and closed. Start requires writable
SD readiness. A later SD fault does not stop the already-running USB stream.
Display GPIO/readback failures leave the backlight off and report
the reason; USB and sensor acquisition remain available.
`accessories` explicitly reports `display_fitted` and `buttons_fitted`; both are
false for the connected hardware. `navigation.qualified` is always false for
this profile. `phase`, `timing`, nullable `speed_mps`/`position_ecef_m`, epoch,
accepted/fused/rejected counts, queue drops and maximum step duration report the
actual worker. `imu_bridged` counts short IMU gaps interpolated by the current
driving estimator. `metrics:preliminary-driving` identifies the driving timings
in `navigation.driving`. The qualified `drag` entries remain null. `installation` exposes
the build's quaternion, lever arm and timing prior. `capture_timing` reports
verified PPS configuration, lock/age/residual/generation and rejection counts.

While stopped, download committed recording sectors using:

```json
{"op":"files","after":0}
{"op":"read","id":1,"sector":0}
```

`files` returns `{"ok":true,"file":{"id":1,"bytes":512,"closed":true}}`,
or a null file when there are no more IDs. Advance `after` to list the next file.
`bytes` is committed CRC-page capacity, excluding the preallocated tail. `read`
returns id, sector and `hex` containing exactly 512 bytes (1024 hex characters).
Allow 12 seconds for a response. Read failures, out-of-range sectors and attempts
to download during recording return errors. Dirty volumes permit retrieval but
cannot create new recordings.

SD files retain the historical AEVLOG1 v1 CRC page and record headers. Legacy
kind 20 is 94 bytes: u64 t_us, u32 GPS week, u32 GPS TOW ms, nine f64 values
(lat, lon, alt, speed, heading, vertical speed, horizontal accuracy, vertical
accuracy, speed accuracy), u32 latency_ms, u8 satellites, u8 valid. Kind 21 is
61 bytes: u64 t_us, u32 dt_us, six f64 values (accel then gyro), u8 valid.
Current recordings write kind 23 IMU (101 bytes): u64 captured_ns, u32 interval_ns,
u64 effective_end_ns, u64 timing_uncertainty_ns, six f64 original values (accel,
gyro), three f64 aligned_gyro values, u8 valid. Its record-header timestamp is
the exact captured_ns. Kind 24 GNSS (115 bytes) contains the complete kind 20
payload followed by u8 mapped_valid, u64 mapped_ns, u64 mapping_uncertainty_ns,
u32 mapping_generation. An absent mapping has flag 0 and all three fields zero;
it must decode as missing, never as an epoch at zero. All numbers are
little-endian. Kind 1 is JSON metadata. Kind 22 closes a file with
the u32 SD queue-drop count; the last page also sets the closed bit. CRC checking
and session nonce/index continuity are mandatory on import, including after
power interruption. A recording can span multiple rotated files. Additional
kind-1 metadata supplies `file_id` and nullable `previous_file_id` for provenance.
Rotation leaves a conservative frame budget and occurs outside a raw frame;
each selected part may still lack ephemerides contained in a different part.

Kind 25 stores up to 256 bytes per fragment. Its first 24 bytes are:

| Offset | Field |
|---|---|
| 0 | schema u8 = 1 |
| 1 | format u8 = 1 (native Unicore) |
| 2 | flags u8: bit 0 FIRST, bit 1 LAST |
| 3 | reserved u8 = 0 |
| 4 | frame sequence u32LE, advancing across selected raw frames |
| 8 | complete frame length u32LE |
| 12 | fragment byte offset u32LE |
| 16 | first-byte MCU receive time u64LE |
| 24 onward | up to 232 unchanged receiver bytes |

The record timestamp is receive time multiplied by 1000. Reassemble contiguous
offsets with matching sequence, length and timestamp through LAST. Preserve the
24-byte `AA 44 B5` header, payload and four-byte receiver CRC trailer. Validate
the complete Unicore CRC (initial 0, no final XOR), ID and payload bounds before
writing `.unc` or converting observations. Intermediate chunks are not independently
receiver-validated, and a bad final CRC remains recorded for diagnosis. Selected
IDs are OBSVM 12 and ephemerides 106–110, 112 and 2999. A converter's support for
each constellation/message must be checked; preserving 2999 does not imply that
a particular RTKLIB build decodes it. PPK also requires suitable base observations,
navigation data and satellite reception; indoor empty frames cannot produce a solution.

## Display snapshots and batched file reads

A full status advertises `display_v2: true`. The `display` command returns
`{"ok":true,"d2":"<base64>"}`. Decode the padded Base64 field as the shared
Postcard `DrivingFrame`. Its motion epoch is the corrected trajectory time in
microseconds on the board clock. Speed is fused horizontal speed at the IMU.
Vehicle G is signed forward, left and up, with gravity removed. It requires
confirmed rigid mounting and observable tilt. Body heading and course over
ground have independent availability. Missing estimates are encoded as absent.
[Wire schema](../src/display.rs)

When GNSS position uncertainty exceeds the driving estimator's threshold,
valid Doppler and IMU samples can still supply live fused speed. The frame
sets `motion.degraded` for this interval. The estimator propagates position
until a usable fix returns and interrupts lap and drag timing across the gap.
The live preview can also coast on the IMU for up to two seconds from the
last accepted Doppler epoch, show a fresh GNSS-only reading while IMU fusion
recovers, or hold the last reading for up to 500 ms from its original
epoch. These sources are excluded from on-device timing and from the SD
driving snapshots. USB status labels them in `navigation.motion_source`.
The USB `display` response carries the same source alongside `d2`.

The frame contains the current device/session clock, recording state, battery,
GNSS and IMU counters, satellite/fix diagnostics and preliminary timing results.
Traj computes gate and drag crossings between corrected knots on the device.
A display read leaves the raw USB queue alone. `start_display` records native
evidence to SD. Each SD part starts with kind 27 records containing the committed
Postcard `Configure::Begin` and its `Gate` entries. Kind 26 stores the same
Postcard `DrivingFrame` as the display, at roughly 5 Hz. Its header epoch equals
`uptime_us` in nanoseconds. An unaligned frame carries absent motion. The last
part includes a terminal frame at the stop epoch with final timings and absent
motion, followed by the normal close record. `start` also enables raw USB preview.

Send `{"op":"configure_drive","drive":{"Mount":{"confirmed":true}}}` to
confirm alignment while idle. Before a run send a `Begin` with speed targets
in hundredths of km/h, distance in mm, rollout, mounting and gate count.
Send `Gate` entries in order, starting at index zero. Coordinates are in
1e-7 degrees, bearing in hundredths of a degree and width in cm. Gate zero
is start/finish. The final gate commits the configuration. Settings last
until reconfiguration or power off.

A receiver advertising `read_blocks: 8` accepts
`{"op":"read_blocks","id":12,"sector":0,"count":8}` for a closed file.
The reply echoes the file ID and first sector, and contains a `blocks` array
with one padded Base64 string per consecutive 512-byte sector. Count is 1–8.
The last request must fit the file. Any failed sector produces an error for the
whole request. Existing clients can continue using the single-sector `read`.

## Bluetooth LE

The foreground client discovers service `cf2c0001-183e-4cd6-a712-c719ac51bd52`.
The characteristics use the same UUID suffix.

| Characteristic UUID prefix | Operation | Payload |
| --- | --- | --- |
| `cf2c0004` | Read | Postcard `DrivingFrame` from `src/display.rs`. Poll at 5 Hz. Long-read continuations use the same snapshot. |
| `cf2c0003` | Write with response | One byte. `1` starts SD recording. `0` stops it and waits for the file to close. Invalid commands and failed operations return a GATT error. |
| `cf2c0005` | Write with response | Postcard `Configure` command while idle, or four ASCII bytes `BAT1`, `BAT2`, or `BAT3` to opt into extended live details for this connection. |

A successful `BAT1` write enables a battery trailer on the next initial display
read and the first initial read after each five-second interval. The value
starts with the ordinary Postcard `DrivingFrame` (at most 208 bytes). A trailer
contains four ASCII bytes `BAT1` and one Postcard `BatteryTelemetry` (at most
20 bytes), giving a 232-byte upper bound for the negotiated BLE value. Read
Blob continuations retain the frozen value. Disconnect clears the opt-in.
Older firmware rejects the opt-in write and continues returning plain frames.

`BAT2` uses the same cadence and battery payload, with a `BAT2` tag and
three additional bytes in charger die, IMU, then processor order. A byte
is the whole Celsius reading plus 55. The value 255 means unavailable.
The upper bound is 235 bytes. Existing `BAT1` clients keep their 232-byte
bound.

`BAT3` adds one source byte at the end of every live frame, after the optional
`BAT2` trailer. Source codes are 0 waiting, 1 fused, 2 fused with weak position,
3 IMU coasting, 4 GNSS only, and 5 holding the last reading. The maximum frame
size is 236 bytes. The app retries `BAT2` and then `BAT1` on older receivers.

`BAT4` reserves bit 7 of that same source byte for recent IMU saturation
or an out-of-calibration-range sample. The lower seven bits carry the source.
The bit stays set for two seconds after the latest clipping event. USB
`status` and `display` report the flag as `imu_clipping`. The app tries
`BAT4` before the other settings writes.

The receiver accepts one client without pairing. A dropped connection leaves SD recording active.
Full diagnostics, raw sample drains and file downloads use USB.
