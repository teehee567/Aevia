# Aevia application

Flutter interface and Rust session/receiver service for the V2 Mini. The desktop
workflow connects over USB, records receiver evidence, and uses Traj for
GNSS-only lap, sector, distance and drag measurements. The original fused Traj
estimator remains separate from this receiver-only path.

## Run and build

From `app/`, with Flutter, Rust and the Windows C++ desktop build tools installed:

```powershell
flutter pub get
flutter run -d windows
flutter build windows --release
```

The existing Cargokit integration builds the Rust library. If the Rust bridge
API changes, regenerate its Dart and Rust bindings before building:

```powershell
flutter_rust_bridge_codegen generate
```

Run the application and backend checks from `app/`:

```powershell
flutter analyze
flutter test
cargo test -p rust_lib_app
```

The interface adapts to compact windows, with a navigation drawer below 880 px.
On Windows or iOS, open **Receiver → Connect Bluetooth**, allow Bluetooth
access, then open **Live**. Keep a single available V2 Mini nearby during
discovery. Start and stop recording from Live to save a sampled session in
the app and full-rate data on the receiver SD card. Windows also supports USB
for detailed diagnostics, full-rate host capture and SD downloads.

To browse the receiver's SD card, connect over USB, stop recording, and open
**Board data → Refresh** below Diagnostics. The directory lists the newest
file IDs first, their committed sizes, and whether each file closed successfully.
**Preview** reads up to four sectors for the file's recorded metadata, start
uptime and, when available, duration and SD queue drops at closure.
**Download & analyze** transfers the selected file into the app library.
Keep the USB cable connected while the progress indicator is shown. An open
recording cannot be downloaded while the device is actively recording.
An interrupted file can be downloaded while stopped; its committed records are
imported with an incomplete-recording warning.

## Record and inspect a session

1. Connect the V2 Mini USB data port and open **Receiver**. Refresh the device
   list and connect to the application port. A recovery-loader port is listed
   explicitly and cannot be selected for recording; install application firmware
   using the firmware uploader first. Close other serial monitors before opening
   the port.
2. In **Tracks**, select **Create track** for lap timing. Search for a circuit,
   address or latitude and longitude, then drag to pan and pinch or scroll to
   zoom. Select **Draw start / finish** and mark the two edges of the track.
   Check the travel arrow and use **Reverse direction** if needed. Add sector
   gates in driving order. **Gate coordinates** lets you enter exact values,
   and the editor can copy a live fix or a recorded position from analysis.
3. In **Live**, choose a track and optional session name, then start recording.
   The G-meter shows the receiver's accelerometer axes with gravity included.
   Tilt the receiver to test it over USB without a GNSS fix or recording. Speed
   is hidden when the fix is invalid or stale. Stop and save to create a session
   in the library.
4. Open a session from **Sessions**. Replay its recorded position, inspect speed,
   acceleration or altitude, compare laps and review sector and drag results.
   The slider and chart share a replay cursor. Drag the chart to seek; scroll or
   pinch the local track view to zoom and drag it to pan.
5. Select a saved track and use **Apply track & reanalyze** to recompute results.
   This also applies the current drag configuration. It updates that session's
   track/drag analysis while retaining its captured evidence.

Place-name search uses [Photon](https://github.com/komoot/photon). The map uses
[EOX satellite imagery](https://maps.eox.at/) worldwide and
[NSW Spatial Services aerial imagery](https://data.nsw.gov.au/data/dataset/nsw-imagery)
for close views in NSW. Where satellite imagery lacks road detail, switch to
the street map. Map imagery and place-name search require internet access.
Coordinates can also be entered directly into the gate fields while offline.
To use a licensed high-resolution satellite provider for another area, build
with `--dart-define=AEVIA_SATELLITE_TILES=<XYZ template>` and
`--dart-define=AEVIA_SATELLITE_ATTRIBUTION=<provider credit>`. The template uses
`{z}`, `{x}` and `{y}` with Web Mercator tiles through native zoom 19.

The reference overlay aligns laps by elapsed time from the start of the lap. It
does not represent distance-aligned delta or certify a common racing-line
position. The replay map uses local coordinates and works offline.

## Driving display traffic

With matching app and receiver firmware, Live requests one snapshot every 200 ms.
Starting a recording saves display fixes to the host library and full-rate evidence
to the receiver SD card. Select **Copy full-rate raw data over USB** before
starting if you also need the full sensor stream on the computer. Older receivers
use full status for preview and their existing raw protocol for host recordings.

| On the link | When | Example size |
| --- | --- | ---: |
| GNSS position, altitude, speed, heading, vertical velocity, horizontal accuracy, satellites, validity, latency and receiver/device timestamps | Each display snapshot | 52 B |
| Accelerometer XYZ, observation timestamp and validity for the G-meter | Each display snapshot | 19 B |
| Device clock, session number, recording/stop state, battery percentage and GNSS total/rejected-fix counters | Each display snapshot | 14 B |
| Entire framed exchange, including the request, Base64, JSON and serial framing | 5 exchanges/s | 159 B each, 795 B/s, 2.86 MB/hour |
| Traj lap count, current/last/best lap, session distance and average speed | Computed on the host from receiver fixes during a session | 0 additional receiver bytes |
| Full diagnostics and capabilities | Connect, start and stop. Every 5 s on Receiver, about once a second on Diagnostics. Full-rate host capture also samples health about once a second. | Around 4.7 kB in an example status, up to the 8 KiB response buffer before framing |
| Raw USB copy, when selected | 50 GNSS fixes/s and nominally 737.5 IMU intervals/s | About 110 kB/s in Base64 records before batch envelopes |
| SD download, on request | Eight 512 B sectors per request, fewer in the final batch | About 5.6 kB exchanged per 4 KiB of file |

Board data checks receiver status every five seconds while visible over USB.
Reading the directory, previewing a file and downloading it are manual actions.

Display sizes use the encoded one-hour-uptime fixture in the native test suite.
Integer lengths and absent sensor fields change the actual size. Rates are USB
application traffic, including framing where indicated. Bluetooth throughput and
energy use require measurement on hardware.

Browsing saved sessions or hiding the app stops preview requests. During an
active host recording the native collector continues at the selected rate.
The live speed, position and heading come from GNSS. The G-meter uses sensor
axes including gravity. Traj evaluates the sampled fixes for session metrics.
Download the SD recording for full-rate timing analysis of short events.

## Drag testing

In **Drag testing**, choose preset or custom speed thresholds, a distance and
optional one-foot rollout, then save the configuration before recording. A
session can contain multiple runs. Analysis reports the configured speed time,
distance time, end speed, slope and validity when those measurements are
available. The displayed end speed is not an independently measured averaged
trap speed.

Standing-start timing uses the engine's sustained-motion detection after a
stationary period. Its exact threshold and limitations are shown with the
analysis. It is not an extrapolated true-zero launch. Signal gaps are not bridged
for lap or drag measurements.

## Files and exports

The session library location is shown in **Sessions**. The backend persists
tracks, drag configuration and sessions there. Recording maintains a raw
`.capture.jsonl` journal so interrupted captures can be imported afterward.

**Import session** supports Aevia session JSON, telemetry CSV, recoverable
capture JSONL and AEVLOG1 `.aev` recordings. Binary imports verify page and record
CRCs and retain a raw source copy beside the imported session. Incomplete files
retain their usable committed records and show warnings. CSV uses named columns
such as `time_s`, `lat`, `lon`, `speed_mps` and `valid`.

Long imports and reanalysis keep receiver status polling available. Use
**Cancel processing** to stop an analysis; the original source remains intact.
Stopping a host recording saves the received samples before analysis, so a
cancelled final analysis can be retried from the library.

Full-rate host capture on firmware advertising `base64_poll` uses strict padded RFC 4648 base64 batches
(`drain_base64`, up to 48 records) carrying exact kind-23/24 sensor payloads.
The host falls back to `binary_poll` hex batches (up to 32 records), compact JSON,
then the older full JSON poll when those are the available capabilities. During recording the host streams them into a checksummed `.raw.aev`
sidecar, up to 8 GiB. Full IMU vectors and nanosecond timing stay in that file;
canonical JSON keeps bounded first/last IMU diagnostics and counts. GNSS raw
records and projected samples retain a 128 MiB observation budget within the
256 MiB session/import limit. Older JSON-only firmware keeps the previous
bounded journal path. The raw sidecar is flushed alongside the journal and is
retained after interruption; retain both for recovery. This avoids storing every
727 Hz IMU sample as an in-memory JSON object. The collector measures GNSS/IMU
rates from fresh status counters; requested rates are separate configuration
fields. Reaching
the budget stops host collection with an explicit interruption message; save
the received data and start another recording. The capture journal is retained,
including received records at the boundary. Interrupted sessions keep their
incomplete reason after reanalysis. Saves are checked for reloadability before
replacing an existing session.

**Export session** writes telemetry CSV, a GPX position trace or complete Aevia
JSON. Desktop file dialogs are available for choosing files and destinations;
paths may also be entered directly. An existing destination is rejected by the
backend, so use a new filename for each export. JSON export copies referenced raw
files beside the JSON and uses relative sibling paths. Move all of those files
together. Import makes its own copy in the library, so deleting the source bundle
does not delete the imported raw evidence. An unavailable raw reference produces
an explicit warning. CSV and GPX contain projected GNSS data and omit full raw
evidence; use the paired JSON export for archival transfer.

**Explore demo** creates a synthetic session through the same backend and
analysis workflow. It is explicitly labelled as demonstration data in the
library and analysis view.

## PPK rover data and corrected sessions

The **PPK** workspace exports native Unicore rover evidence, converts it to RINEX,
and optionally invokes an external RTKLIB processor with user-selected base and
navigation files. It keeps the original SPP session unchanged. The current
firmware requests 50 Hz native binary SPPNAVB (message 46) with SG8 and 5 Hz
OBSVMB plus broadcast ephemerides. The app shows the requested rates and
observed counters separately. The tested
UART profile uses separate 460800-baud data and raw-observation ports. Configuration
acceptance and no-fix cadence do not establish outdoor positioning accuracy.

Raw PPK observations are **SD-only**. A USB host sensor sidecar has full IMU/GNSS
sensor records but does not contain carrier observations. After stopping, download
the matching device recording and select that imported session in PPK. Use
**Prepare RINEX** to create a new subfolder under the selected output folder
containing the following files.

- `rover.unc`: unchanged complete native receiver frames, reassembled from AEV
  kind-25 fragments and independently checked against the receiver CRC.
- `rover.obs` and `rover.nav`, when the converter can produce RINEX data.
- `manifest.json`, source/raw SHA256 hashes, frame/epoch counts, discontinuity
  diagnostics, converter arguments, executable hash and retained process logs.

Incomplete fragments and bad native CRC frames stay in the original AEV. They
are excluded from `.unc`, explicitly counted, and prevent a ready result. An
empty/no-signal observation stream cannot produce a corrected trajectory.
Readiness also requires populated code and carrier-phase observations on at
least four satellites in an epoch, plus navigation records. Code-only indoor
observations remain available for inspection but are not marked ready for PPK.
Saved UART and byte-queue losses are reported separately from SD drops; known
losses on the raw-observation port prevent a ready result. These checks do not
establish base overlap or positioning accuracy.
Preparation currently covers **one selected SD part**. If recording rotated,
download each part and prepare it separately. Ephemerides may reside in an
earlier part; select all matching navigation files when processing. Raw BDS-3
message 2999 is preserved, but the pinned converter does not decode that message.

Install the optional Windows command-line tools from the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app/rust/tools/install_ppk_tools.ps1
```

This verifies the official RTKLIB Explorer 2.5.1 release archive against pinned
SHA256 `0b20c940b1939b0701f0ee027daf5225221120dbff50c3f563f8705fd0b496f5`,
and installs only `convbin.exe`, `rnx2rtkp.exe` and their retained notices under
`target/ppk-tools`. The execution-policy setting applies only to that process.
For a distributable Windows app, pass `-Destination <runner-directory>/ppk-tools`
and keep `RTKLIB-LICENSE.txt` and `RTKLIB-NOTICE.txt` alongside the executables.
The app searches that directory, `AEVIA_PPK_TOOLS`, the workspace installation and
PATH; the PPK file pickers can explicitly select another compatible executable.
The archive and source are from the [official release](https://github.com/rtklibexplorer/RTKLIB/releases/tag/v2.5.1),
source commit `62d4677ed8425a4e2748c6d390b500d1afb493fc`. Stock older RTKLIB builds
may lack `convbin -r unicore`. Native Unicore recordings use `.unc`, not `.ubx`.

For relative PPK, select rover RINEX observations, overlapping base observations,
and matching broadcast navigation files. The base must observe common signals
during the rover interval. Supply verified base ECEF coordinates and antenna
metadata; otherwise the processor uses the base RINEX `APPROX POSITION XYZ`
header with an explicit verification warning. No base/correction dataset is
included for the user's capture. Antenna offsets, reference frame, receiver
tracking quality and sky view remain necessary inputs to interpreting results.
An optional RTKLIB configuration is retained; the app explicitly selects kinematic
combined processing, GPST, degree LLH, ellipsoidal height and position-only output.

Each processing run creates a distinct folder and session with source lineage,
input and executable hashes, effective configuration, logs and original `.pos`.
External execution is cancellable and bounded to 30 minutes, 16 MiB per process
log, and a 256 MiB solution file; partial artifacts remain available. Input raw
files are limited to 8 GiB and imported `.pos` results to 250,000 epochs. Split
long processing runs by time while preserving all original evidence.

**Import corrected .pos** accepts the documented RTKLIB GPST week/TOW or GPST
calendar output with ordered degree-LLH, Q, satellite count, covariance, age and
ratio columns. Other time systems, DMS/ECEF layouts or reordered columns are
rejected explicitly. Fixed, float and other Q states remain distinct; reported
standard deviations and signed cross-covariance fields are retained. Corrected
positions can be replayed, but receiver Doppler speed, heading, vehicle acceleration,
lap and drag metrics remain unavailable. Position differences are not silently
promoted to measured velocity. Reanalysis preserves those limits and provenance.
A fixed ambiguity state or reported covariance is not an independent accuracy
measurement or a promise of centimetre performance.

The [hardware validation record](../data/highrate-ppk-validation.md) documents
the converter and processor checks and links their software references.

## Measurement limits

The app's performance analysis uses Traj receiver interpolation. Statistical
timing uncertainty and live predictive delta are not enabled by the app.
The Receiver view separately reports the device's development IMU/GNSS estimator
and its unqualified status. Raw sensor records are retained for later calibrated processing. Mounting
calibration, receiver/IMU clock alignment and interval support must be established
before presenting fused results. Displayed milliseconds are formatting, not a
claim of millisecond accuracy.

Receiver-only interpolation also requires agreement between consecutive GNSS
positions and velocities. The adapter bounds the extra velocity introduced by
each Hermite interval and excludes the interval if that correction exceeds
0.5 m/s or 2% of endpoint speed, whichever is larger. Excluded intervals split
the trajectory: lap and drag timing cannot cross them, and distance totals cover
only the remaining intervals. Raw samples and the original capture remain
available. This is a conservative admission policy that reduces availability,
especially on noisy or stationary captures; it is not an accuracy guarantee or
a substitute for calibrated IMU/GNSS fusion.

Both drag launch and speed-target times must have an ascending crossing in the
receiver's own speed samples. An interior interpolation overshoot alone cannot
complete a target. A conservative whole-interval distance bound skips targets
that cannot be reached. Receiver-only drag analysis uses the first 10,000 fixes
of each attempt to bound processing work. Completed targets in that window stay
available even if the driver continues cruising; later targets remain unavailable.
Cancellation is checked between bounded
processing chunks and attempts.

## Recording diagnostics

Keep the original `.aev` or `.raw.aev` alongside session JSON for raw
reprocessing. JSON retains GNSS clock mappings and bounded IMU diagnostics.
The binary holds the original IMU vectors, aligned gyro, and nanosecond
timestamps and interval support.

Native USB recordings retain start/end health snapshots and counter changes.
Analysis reports sensor errors and timing rejections separately from USB, SD,
and navigation queue drops. A counter reset makes its delta unavailable.
Newer firmware provides device start/stop timestamps. Older firmware falls
back to the uptime reported with its response.

The live view uses the displayed observation's own timestamp for freshness
and calculates sensor rates from new device counters. Charts show gaps and
invalid samples. G-force channels require valid acceleration data.

Capture health includes the requested acquisition profile, raw PPK status, and
per-port GNSS byte losses. SD imports report terminal transport counters
separately from SD queue drops. Rotated parts contain cumulative totals, and
older or interrupted files may have unknown totals. See the
[application protocol](../firmware/docs/application-protocol.md) for capture
boundaries, transport counters, and binary batch limits.

SD imports also retain a bounded IMU continuity summary: the sum of retained raw
intervals, gap and overlap transition counts and nanoseconds, repeated/reversed
capture timestamps, and invalid supports/records. Adjacent support is checked as
`captured_ns - interval_ns` against the previous `captured_ns`. The separately
retained `effective_end_ns` delay approximation does not alter this check. Any
discontinuity produces a persistent analysis warning even when SD queue drops
are zero. It does not infer a missing-sample count or cause, and a continuous
retained span alone cannot prove that nothing was lost outside that span.
