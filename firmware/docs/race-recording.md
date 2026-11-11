# Race recording and USB retrieval

The collection build preserves original sensor evidence on the installed SD
card. It starts recording after a supported card mounts, without buttons or a
USB data host. The connected board now has a single-cell battery; battery-only
runtime and USB-to-battery handover still require physical acceptance. The build
reads battery voltage and uncalibrated gauge SOC. Its initial charging profile
targets 800 mA with a 4.10 V ceiling, subject to existing charger protections
and available USB input power. Query PowerChargeStatus for actual readbacks.
Closing the host app or disconnecting USB data does not stop recording. Removing
all power can lose the buffered tail. `Stop` disables new capture,
drains queued records and commits a final page before acknowledging success.
Download after stopping. `Start` resumes into a new file; an already active
recording returns its current session. Files rotate automatically near 2 GiB.

This build uses an existing supported exFAT volume and never formats or deletes
files. A nominal 128 GB capacity does not establish that a card has a supported
filesystem or enough contiguous free space. Status reports mount/write failures.
Files are preallocated, so their directory size includes an unused tail. Only
the recorded page prefix is downloaded. A dirty or unsupported volume is not
silently repaired. Back up recordings before performing card maintenance.

Live trajectory processing is disabled in this build. Raw data is useful before
PPS lock, estimator initialization or a receiver fix; those states must not
control whether original evidence is retained. Factory scaling, nominal filter
delay, mounting and antenna lever arm remain unqualified. Logging success does
not establish navigation accuracy or a future PPK result.

## Host commands

Install `firmware/tools/requirements.txt` for live USB commands and use the
matching `aevia-board-decode.exe` built by `tools/build_release.ps1`. Offline
verification/export require only Python's standard library. From the repository
root, using an appropriate Python executable:

```powershell
python firmware/tools/recording.py status
python firmware/tools/recording.py preflight --seconds 10
python firmware/tools/recording.py stop
python firmware/tools/recording.py list
python firmware/tools/recording.py download-all --output captures/race --resume
python firmware/tools/recording.py verify captures/race/session-00000001.aev --require-raw-gnss
python firmware/tools/recording.py export captures/race/session-00000001.aev --output captures/race/export-1
python firmware/tools/recording.py start
```

Global options precede the command: `--port COM4`, `--serial AEVIADEVBOARD`,
`--decoder path/to/aevia-board-decode.exe`, and `--timeout 30`. The development
serial string is not a unique production identity; use an explicit port if
multiple devices are connected. The tool requires protocol/schema 1 and the SD
recording capability. Application mode is required; it never silently runs or
updates firmware to make recording commands available.

`preflight` requires active recording, increasing accepted-record and durable
page counts, no queue drops, and no reported storage error over its interval.
It also requires advancing IMU samples and GNSS COM2 input, a ready IMU at both
endpoints, and unchanged sensor/UART error counters. It reports the observed IMU
rate and receiver fix state. Add `preflight --require-fix` before a race to also
require fresh receiver position and velocity at the final check. Without that
option, collection before a fix is allowed. These checks establish live progress,
not sensor calibration or the content/quality of raw satellite observations.
After stopping/downloading, `verify --require-raw-gnss` additionally requires
in-file advancing OBSVM and ephemeris coverage evidence and raw COM2 bytes.
Each new file requests a fresh raw-data bootstrap and records its generation;
the gate requires at least two advancing OBSVM epochs and received ephemerides
from that generation. A short file can fail before these arrive. Preserve the
whole recording set. A failing gate exits nonzero and reports its reasons.

Downloads use `.part` plus `.part.json`. Resume matches the session metadata and
first-page SHA-256, including the random per-file nonce, then validates all
existing complete pages. An incomplete half-page is downloaded again. Existing
completed destinations are never intentionally overwritten. After transfer,
the tool validates every page and record, saves SHA-256 and integrity summary,
and renames the partial file to the requested name. Interrupted downloads stay
available for `--resume`; commands never delete files on the device.

`download-all` collects every listed part and writes `recordings.json` with
session IDs, hashes and recording/rotation links. Existing files are skipped
only after their full integrity, device identity and length match. Missing
previous/next rotated parts are reported and cause a nonzero exit. A single
closed part is not necessarily the whole race: preserve all parts in the index.
For a selected part, use `download ID --output file.aev [--resume]`.

The export directory must be new or empty. Outputs are:

| File | Content |
| --- | --- |
| `gnss-com2.bin` | Exact receiver COM2 RX bytes, including original native messages |
| `gnss-com1.bin` | Exact receiver COM1 RX bytes and command responses |
| `gnss-com1-tx-attempted.bin` | Attempted COM1 command writes; not acknowledgements |
| `gnss-chunks.jsonl` | Original per-stream offsets, receipt time, capture-clock validity, export offsets and record sequence |
| `imu.csv` | Native signed counts, original counters/statuses, all timing fields and original valid f64 scale outputs |
| `timing.jsonl` | PPS, faults, temperature, verified configuration, health, raw GNSS coverage, reset evidence and end records |
| `metadata.jsonl` | Metadata with original payload bytes retained |
| `summary.json` | Checksums, page/record counts, sequence/byte gaps, invalid IMU flags, raw GNSS coverage and terminal counters |

Concatenated UART exports splice across missing chunks when a loss occurred.
Use the JSONL offsets and summary; never present such a stream as complete.
Initial byte offsets may be nonzero because offsets count all traffic since
boot, including before recording. Offsets persist across receiver reconfiguration.
IMU rows with invalid scaling have empty scaled-value cells, preserving their
original raw counts and flags without fabricating zero motion.

`closed` only means a final page was committed. `sealed_and_contiguous` requires
that marker, the end record, no record gaps, no reported queue drops and no UART
byte gaps. It does not certify IMU timing, satellite measurements or the SD
controller's behavior through arbitrary power failure. Check separately reported
sensor errors and invalid IMU timing/scaling counts. An unclosed valid prefix can
be recovered and exported; any trailing partial record is reported and omitted.

## USB contract

The appended outer request and response tag is 2, `Recording`. The existing
System=0 and Device=1 layouts are unchanged. Request frames remain bounded to
272 bytes and replies to 4608 bytes, including COBS and the zero delimiter.
Bootstrap discovery fits 512 bytes. The host grows its receive bound only after
checking ProtocolInfo, then uses Status.max_read_len (4096 in this build) for
reads. Earlier 256-byte download chunks remain supported. Large frames span USB
packets; packet boundaries are not message boundaries. The identity probe still
reads exactly one 512-byte page, and resume always verifies whole 512-byte pages.
Authoritative Rust types are `src/device/recording/api.rs`.

Request fields are `request_id:u32`, followed by `command`:

| Command tag | Body | Completion reply |
| --- | --- | --- |
| Status=0 | None | Status |
| Start=1 | None | Started(Session) |
| Stop=2 | None | Stopped(Session) |
| List=3 | after_id:u32 | Entry(Option<Session>), smallest session greater than after_id |
| Read=4 | session:u32, offset:u64, len:u16 | Data(Chunk), 1..4096 requested bytes; obey Status.max_read_len |
| Result=5 | None | Pending or cached completion for request_id |

Response fields are `request_id:u32`, followed by `reply`: Pending=0, Status=1,
Started=2, Stopped=3, Entry=4, Data=5, Error=6. `Session` field order is
`session:u32, file_bytes:u64, valid_bytes:u64, closed:bool`. `file_bytes` is the
physical preallocation. `valid_bytes` is the discovered committed page prefix,
a multiple of 512; discovery uses bounded tail probes, not a full integrity
scan. Hosts must verify every downloaded page. `Chunk` fields are
`session:u32, offset:u64, bytes:byte-sequence, crc32:u32`; byte-sequence has the
Postcard length prefix and at most 4096 bytes. Its CRC32 covers only those bytes.
Reads fail with RecordingActive until capture has stopped.

`Status` fields, in wire order:

```
next_request_id:u32, state:State, filesystem:Filesystem, capacity_bytes:u64,
free_bytes:Option<u64>, active_session:Option<Session>, queued_records:u32,
accepted_records:u64, dropped_records:u32, written_pages:u64, io_errors:u32,
last_error:Option<Error>, max_read_len:u16, format_version:u16
```

State tags: Initializing=0, Ready=1, Recording=2, Stopping=3, Fault=4.
Filesystem tags: Unknown=0, Fat32=1 (reserved; not currently mounted), Exfat=2.
Free space may be unknown (`None`). Counters are diagnostic snapshots across
queue/worker ownership, not one atomic acquisition epoch. Accepted records are
queued captures, while written pages indicate storage progress; neither is a
count of qualified measurements. Queue/storage counters reset on manual Start
and span automatic file rotations. Sensor error counters remain boot-scoped;
raw GNSS coverage counters reset with the per-file bootstrap generation.

Error tags: NoCard=0, NotReady=1, UnsupportedFilesystem=2, Busy=3,
UnknownRequest=4, RequestConflict=5, InvalidLength=6, SessionNotFound=7,
OutOfRange=8, Io=9, Corrupt=10, NoSpace=11, ReadOnly=12, RecordingActive=13,
NotRecording=14. Recording errors are contained in Recording replies; they are
not loader errors or successful acknowledgements.

First query Status with request ID 0. Use `next_request_id` for exactly one new
storage operation and retain that ID. Poll Result with the same ID while Pending.
Status and Result do not consume IDs. Identical pending/recent completed requests
replay the original outcome; the worker keeps eight completed results. A reused
ID with different arguments conflicts. Older evicted IDs are rejected and cannot
repeat a Start/Stop. Token space does not wrap; zero next ID means exhausted.
IDs reset on MCU restart, so never automatically replay a mutation after reboot.
Rediscover status and reconcile the active session first. A transport timeout is
not evidence that Start/Stop failed. Only one host and one operation are supported.

## On-card format version 1

All following integers and floats are little-endian. Every page is 512 bytes:

| Offset | Field |
| --- | --- |
| 0 | 8-byte magic `AEVLOG1\0` |
| 8 | session u32 |
| 12 | page index u32, starting at 0 |
| 16 | payload length u16, 0..480 |
| 18 | flags u16, only bit0 final/closed is defined |
| 20 | random per-file nonce u64 |
| 28 | payload, followed by canonical zero padding through byte507 |
| 508 | IEEE CRC32 over bytes0..507 |

The payloads concatenate into a record stream; a record may cross page
boundaries. A record header is 24 bytes: kind u8 at0, version u8=1 at1,
payload length u16<=256 at2, sequence u64 at4, timestamp_ns u64 at12, reserved
u32=0 at20. Payload follows, then IEEE CRC32 over header+payload. Sequences
increase and include attempted captures that overflowed the queue, exposing
gaps. File prefixes begin at sequence0; automatic rotation rebases the sequence.
Do not substitute USB or file modification time for capture timestamps.

Kinds: 1 metadata JSON object; 2 native IMU; 3 COM2 RX; 4 COM1 RX; 5 PPS;
6 fault; 7 temperature; 8 COM1 attempted TX; 9 verified IMU configuration;
10 health; 11 raw GNSS coverage; 12 Postcard ResetStatus evidence;
13 expected raw-GNSS bootstrap generation (u32, exactly4 bytes);
14 recording identity; 15 close intent; 255 end.
Unknown kinds retain their bytes in exports. Each defined sensor payload below
begins with version byte1; reserved fields are zero.

### Native sensor payloads

Kind2, 120 bytes: flags at1 (bit0 all axes healthy/scaled values present,
bit1 latest edge equals edge and read completed within the active admission
bound recorded in kind9 at offset88, bit2 latest edge present, bit3 finish
present); valid-axis mask at2; reserved at3. Four u64 values
at4/12/20/28 are edge/read-start/read-finish/latest-edge nanoseconds. Six i32
counts at36 are gyro XYZ then accelerometer XYZ, preserving signed20-bit values.
Six counter bytes are at60; six status bytes at66 encode Normal=0, Error=1,
Saturation=2, Initializing=3, with common-error bit2. Three original, unaligned
f64 angular rates in radians/s are at72 and three accelerations in m/s² at96.
Scaled fields are zero on wire when flag0 is clear and must be treated as absent.
All native bursts are recorded before interval alignment, including rejected
timing/status samples.

Kinds3/4/8: 24-byte prefix plus at most232 original bytes. Flags at1 bit0 mean
capture clock valid. Receipt monotonic microseconds u64 are at4, per-stream byte
offset u64 at12, byte count u16 at20; reserved2..3 and22..23. Exact bytes begin24.
The record timestamp is the capture clock or zero with the validity bit clear.
Kind8 means attempted write; consult kind6 GNSS TX faults and COM1 replies for
the result. No bytes are synthesized or removed from the native streams.

Kind5, 24 bytes: physical pulse time u64 at4, raw MCPWM ticks u32 at12, unique
pulse sequence u32 at16 and capture-error count u32 at20. These are independent
physical edges; receiver UART PPS labels remain in the raw stream.

Kind6 IMU fault, 32 bytes: source1 at1; code at2 (Driver1, Timeout2, Gap3,
Range4, Timing5), driver error kind at3, retained fault edge u64 at4, register at12
(255 absent), status at13, expected/actual u16 at14/16, counter at18, edge-valid
at19 and original sensor payload u32 at20. GNSS fault is8 bytes: source2 at1,
code RX1/Parse2/TX3 at2, detail at3, port0 unknown/1 COM1/2 COM2 at4.

Kind7, 8 bytes: valid byte at1, f32 die temperature °C at4, timestamp at read
start. Kind9,104 bytes: register count12 at1, component/ASIC u16 at2/4, serial
three u16 at6, twelve `(register:u8,value:u16)` pairs at12, gyro delay/accel
delay/delay uncertainty/tick period u32 nanoseconds at48/52/56/60, f64 rad/s per
LSB, m/s² per LSB and nominal Hz at64/72/80, deadline/minimum interval/maximum
interval u32 nanoseconds at88/92/96, output1 decimated at100 and profile schema1
unqualified at101. The applied admission bound is 1,000,000 ns for native data
collection and 600,000 ns for navigation; use the recorded value for each file.
The current worker repeats cached verified configuration in
new-file prefixes when it is available; preserve the whole recording set.

Kinds10/255,41 bytes: accepted records u64 at1; dropped records u32 at9; IMU
sample/error counts u32 at13/17; GNSS UART/parse errors u32 at21/25; written
pages u64 at29; queue depth u32 at37. These counters are captured before the end
record and its final page, exclude per-file metadata prefixes, and may span
rotated files. Do not equate accepted_records with the number of file records.

Kind11,40 bytes: ephemeris-seen mask at1 (bits0..4 message IDs106..110), flags at2
(last-receipt present bit0, last-epoch present bit1); OBSVM frames/advancing
epochs/ephemeris frames/optional TX failures u32 at4/8/12/16; bootstrap generation
u32 at20; last OBSVM receipt
u64 microseconds at24 and GPST epoch u64 milliseconds at32. Recognition requires
native message CRC; it does not validate every channel observable. Counters reset
when the receiver owner processes a new-file bootstrap request. Ignore coverage
from other generations; never subtract counters across a generation change.
Kind13 records the expected generation before native capture records in the file.

Kind14,21 bytes: version1 at0, recording ID u32 at1 (first file of Start), file ID
u32 at5, previous file ID u32 at9 (zero for the first part), source sequence base
u64 at13. Kind15,6 bytes: version1 at0, reason byte at1 (0 user Stop, 1 rotation),
expected next file ID u32 at2 (zero for Stop). It immediately precedes kind255.
Use these links to group parts and detect missing segments. An individually
closed file does not establish that every part of the recording is present.

The configured native stream includes OBSVMB at1Hz and GPS/GLO/BDS/GAL/QZSS
ephemerides on change, alongside PVT50Hz and PPS1Hz. Raw observations preserve
options for external offline processing; this firmware and the existing
trajectory backend do not provide a PPK solution. Review actual coverage and
data gaps for each race instead of assuming configuration guarantees reception.
