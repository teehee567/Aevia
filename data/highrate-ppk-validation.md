# Higher-rate GNSS, native IMU and PPK

This follow-up raises the V2 Mini recording rates and adds a separate PPK
workflow. The connected board is indoors, with battery, GNSS and IMU fitted;
no position fix is expected here. The receiver intermittently picks up a small
number of signals and cached ephemerides despite being indoors.

## Acquisition and data paths

- UM980 standalone `SPPNAVB` at 50 Hz, `CONFIG SIGNALGROUP 8`. This is the
  receiver's native binary standalone solution log, with a 20 ms epoch interval.
  Its 24-byte header, 120-byte payload and four-byte CRC total 148 bytes per
  update (7,400 bytes/s at 50 Hz, before other messages). SG8 changes
  the available signals; it does not retain SG1's GLONASS/NavIC selection.
- SCH16T native DEC5 intervals are recorded without four-sample decimation.
  The nominal rate is 737.5 Hz; this fitted sensor measures approximately
  727.2 Hz. Original vectors, capture timestamps and interval support remain
  in the recording. The development navigation worker separately averages
  four intervals and retains its existing qualification limits.
- The SCH16T already uses 48-bit binary SafeSPI transfers at 8 MHz, with
  frame CRC, status and sample-counter checks. Native binary GNSS frames are
  distinct from the USB wrapper: the app receives compact binary sensor records
  encoded as base64 inside bounded JSON replies. Older hex and JSON commands
  remain available for compatibility.
- Raw Unicore `OBSVMB` at 5 Hz plus broadcast ephemerides are recorded on SD.
  Raw observation rate and SPP rate are independent. At 150 signal observations,
  native OBSVM alone requires 30,160 bytes/s at 5 Hz; 50 Hz raw output would
  require 301,600 bytes/s, beyond these UART links.
- USB uses bounded binary batches containing the exact sensor payloads used
  by SD. The desktop writes full-rate evidence to a checksummed `.raw.aev`
  sidecar and keeps bounded IMU diagnostics in session JSON. USB telemetry does
  not include the raw GNSS observations needed for PPK; download the SD file.
- Recording timestamps begin after SD setup. Sensor losses, USB/SD queue
  drops and raw-frame CRC failures are reported independently.

## PPK workflow

1. Record with a clear sky view, then stop and download the device recording
   from **Receiver → Device recordings**.
2. In **PPK**, select that downloaded recording and an output directory. Use
   **Prepare RINEX** to retain the original `.unc` stream and convert complete,
   CRC-checked frames to RINEX with Unicore-capable `convbin`.
3. Supply a stationary base's observations covering the same time, matching
   navigation data and verified base antenna coordinates. **Run PPK** invokes
   `rnx2rtkp` in kinematic combined mode.
4. The result is a separate session. Replay exposes fixed/float/single quality,
   reported covariance and ambiguity ratio. The manifest records input hashes,
   executable/configuration provenance and retained processing logs.

Existing RTKLIB `.pos` solutions can also be imported. The importer requires
GPST, decimal-degree LLH and the documented quality/covariance columns.
Position-only PPK results do not invent velocity, acceleration, lap or drag
measurements. The raw recording remains intact.

Each export covers one SD file part. For rotated recordings, retain the other
parts and provide their navigation data as needed. Empty indoor observation
frames, missing ephemerides and incomplete fragments remain explicit warnings.
The app's tool installation and portability instructions are in
[app/README.md](../app/README.md).

## Verification evidence

Local bench artifacts are under `target/bench/v2-mini-validation/` unless a
different path is listed. This directory is ignored by Git.

The first staged build demonstrated 50.0215 Hz GNSS and 727.2408 Hz IMU during
a 30.073 s device-time preview, with no new sensor errors or queue drops.
Longer recording tests then exposed USB queue losses and errors on the 921600
baud GNSS link. Those staged builds are diagnostic evidence, not the final
release. Their artifacts remain under `target/bench/highrate-stage-*` and
`target/bench/native-highrate-stage*`.

The pinned RTKLIB Explorer 2.5.1 processor successfully processed the upstream
`07590920` rover / `30400920` base fixture and matching navigation data: 120
positions, comprising 46 fixed and 74 float solutions. The actual converter
also accepted a native Unicore observation fixture. These are software tests
using reference inputs, not corrected measurements from the indoor board.
The report is `target/ppk-smoke/processor-smoke.json`.

A physical SD download also verified the real capture-to-converter path:
`import-1794969836209.aev` contains 101 nonempty, GPST-qualified OBSVM epochs
and six GPS ephemeris frames, with no native CRC errors, incomplete fragments
or missing frame sequences. The actual converter produced 101 RINEX epochs
and six navigation records. This indoor input contains one satellite's code
and signal-strength observations, without carrier phase; it cannot support
PPK. The capture also has an independently detected 9.6276 ms IMU support gap
from a diagnostic firmware build and is not the final integrity validation.
Evidence is retained in `target/bench/backend-continuity-audit/` and
`target/bench/native-highrate-ppk/`.

The binary profile ran on the fitted V2 Mini over COM8. SPPNAVB headers reported
receiver build 17548, GPST/FINE time and insufficient observations for both
position and velocity. The Python probe recorded 1,003 GNSS frames and 14,579
valid IMU intervals in 20.054 s. All GNSS epochs were 20 ms apart and all IMU
intervals were contiguous. The measured rates were 50.0006 Hz and 727.0552 Hz.
UART, sensor, USB, SD and PPK loss and error counters did not increase during
the recording. The SD file closed as recording 25. One raw-port FIFO error
occurred earlier during receiver startup, before recording admission.

The tested application image was `firmware/target/upload/firmware-highrate-native-binary.bin`, SHA-256
`48df468282b01b3e38dc6499b221a1243f5754643b2a3e52ce9f275b2822fd3a`.
The release interrupt audit checked 210 reachable functions and found zero
floating-point instructions. CPU0 has 34,640 bytes reserved for its stack.
Python readiness and capture results are in `native-binary-ready.json` and
`native-binary-gate.audit.json`. See `native-binary-audit/audit.json` for the
interrupt audit.

The desktop collector received 6,008 GNSS frames and 87,358 valid IMU intervals
in a separate 120.157 s recording. Every receiver epoch was 20 ms apart. The
checksummed USB sidecar had no duplicate records or IMU interval gaps and
retained all 72 records pending at Stop. The USB buffer held 29–51 samples at
the 30 s checkpoints. The receiver emitted 601 raw observation frames and two
ephemerides during the test. Loss and error counters stayed unchanged during
recording and SD file 26 closed cleanly. The Python-driven collector report is
`native-binary-sustained/audit.json`.

The Python download of SD file 25 verified all 4,239 page CRCs and record
sequences. Its 15,582 sensor records matched the USB capture exactly, including
all floating-point bits and nanosecond timestamps. The file's saved transport
loss counters were zero for both UARTs. The comparison is in
`native-binary-gate.comparison.json`.

The same file yielded 103 complete, CRC-valid native frames: 101 empty 5 Hz
observation epochs and two broadcast ephemerides. `convbin` accepted the
export and produced a RINEX navigation record. The indoor recording had no
usable carrier-phase observations, so the app reported it as insufficient
for PPK. Export results are in `native-binary-ppk25/prepare.jsonl`.

The desktop build includes RTKLIB Explorer 2.5.1 for conversion and processing.
The Windows release build, 17 Flutter workflow tests, 108 Rust backend tests,
159 firmware host tests, and 30 Python protocol tests passed.

## USB performance build

On the same board, the median reply for 96 full 48-sample USB batches fell
from 27.5 ms to 14.9 ms. Sequential reads of the same 256 SD sectors rose
from 31.1 to 59.4 KiB/s and returned identical bytes.

The first optimization builds exposed GNSS and IMU queue losses at recording
start. The validated build briefly lets the UART accumulate bytes during bursts
of short reads. The subsequent 12.067 s and 20.050 s indoor recordings reported
zero new acquisition, transport, SD, or PPK losses. All 9,376 sensor records
from SD recording 31 matched the USB capture, including nanosecond timestamps
and floating-point bits. The comparison and warm-start audit are
`usb-performance-final-1794985615.comparison.json` and
`usb-performance-warm-1794985724.audit.json` in the local evidence directory.

The tested application image is
`firmware/target/upload/firmware-usb-performance-final.bin`. Its release build,
Clippy, and host checks passed. The local post-link audit found zero
floating-point instructions along the inspected acquisition interrupt paths
and a 34,048-byte CPU0 stack reservation.

## Sources and qualification

- [Unicore N4 command manual R1.14](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf):
  SG8 rate support, standalone navigation logs, native observations,
  ephemerides and independent UART configuration.
- [Unicore Build17548 release note, mirrored by SparkFun](https://community.sparkfun.com/uploads/short-url/cslDsPIR9OgE3WDot9epiqZczjf.pdf):
  the December 2024 release adds 50 Hz PVT in SG8. No receiver firmware upgrade
  is performed here; earlier local evidence identifies this build on the board.
- [Murata SCH16T product documentation](https://www.murata.com/en-us/products/sensor/gyro/overview/lineup/sch16t)
  and the supplied `data/datasheets/sch16t-k01-datasheet-full.pdf`, document
  11624 Rev 6: native output rate, decimation, filters and timing.
- [RTKLIB Explorer source](https://github.com/rtklibexplorer/RTKLIB): the
  Unicore decoder and RINEX/PPK command-line tools.

Indoor output cadence and data integrity do not establish outdoor position
accuracy, ambiguity reliability, full-sky raw bandwidth or calibrated IMU/GNSS
fusion. Those require suitable rover/base recordings and mounting/timing
qualification.
