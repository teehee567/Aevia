# Aevia USB and firmware baseline

This record covers the original 20 Hz GNSS and averaged IMU build. See the
[higher-rate validation record](highrate-ppk-validation.md) for subsequent
50 Hz SPP, native-rate IMU, PPK, and USB performance measurements.

## Bench hardware

- V2 Mini connected over USB; existing application loader found on COM8.
- Battery and IMU connected.
- GNSS connected; no indoor fix is expected.
- Display is not connected; retain display support without requiring it.
- Extra buttons do not work; normal operation must not depend on them.
- Four RGB LEDs must communicate useful device state.

## Implemented

- Flutter live/receiver/library/analysis/track/drag workflows, responsive layouts,
  explicit missing/stale/invalid/demo values, native file dialogs.
- Native USB collection separate from UI timing, recoverable journals, persistent
  tracks/sessions, GNSS epoch normalization, CSV/GPX/JSON import/export.
- Verified legacy AEVLOG1 container decoding with retained raw source copies,
  plus standalone SD recording retrieval, including interrupted files while
  stopped. Physical transfer of a closed recording is verified below.
- Receiver-only Traj measurement wrapper; no synthetic inertial observations.
  Existing INS implementation and production qualification gates retained.
- Existing lap/drag/distance evaluators used for custom gates/sectors, statistics,
  sustained-motion start, rolling thresholds, distance targets and rollout.
- Firmware GNSS/IMU drivers, battery, display, controls and LED drivers restored
  selectively from repository history; current recovery protocol preserved.
- Standalone exFAT SD recording, bounded independent queues, CRC pages/records,
  stop flush and USB file retrieval. Does not format the card.
- Battery/IMU enabled; extra buttons and absent display explicitly disabled.
  Four functional LEDs and an app legend show GNSS, IMU, recording/storage and
  power state. Indoor no-fix is separate from receiver communication failure.
- Development on-device Traj navigation with bounded work and PSRAM placement;
  hardware PPS/DRDY capture and explicit unqualified installation/timing status.
  On-device drag definitions remain disabled because root qualification is absent.
- Versioned timing records retain original IMU vectors, separate aligned gyro,
  exact nanosecond capture/support/uncertainty and optional GNSS PPS mapping.
  Old SD record formats remain readable. Full raw reprocessing uses the retained
  binary; UI session JSON contains bounded IMU diagnostics plus GNSS mappings.

## Verification

- Final Windows release app build succeeds with SD/cancellation, timing-record,
  receiver-admission and device-diagnostics changes. Executable:
  `app/build/windows/x64/runner/Release/app.exe`.
- Flutter analyzer clean and thirteen workflow tests pass, including compact layout,
  missing accessories and development estimator status. Two rendered-layout
  checks pass; desktop three-lap demo and compact live images inspected.
- Traj offline suite: 446 passed after root budget and endpoint fixes.
- Receiver adapter analytic tests: standing/rolling acceleration, rollout,
  repeated runs, finite/directional gates, ordered sectors, gaps and jumps pass.
- Native backend: 59 tests and strict app-only clippy pass, including actual
  firmware encoder → importer/USB schema agreement and integers above 2^53.
- Traj allocator-free RISC-V target check passes after endpoint fixes.
- The final firmware development release and strict app-only clippy pass;
  final on-disk binaries use the default development configuration.
  114 host tests and 24 uploader/protocol tests pass. Actual
  profile tests prove development startup succeeds and production rejects the
  unqualified profile; unsupported drag roots remain rejected.
- The 3,505,808-byte application image fit its partition. The existing
  recovery loader was used for the upload.
- Existing run09 capture imports with invalid/arrival-only GNSS samples and no
  invented measurements. Actual run08 retains 106154 samples and 54533 usable
  receiver fixes; 30192 inconsistent intervals are excluded. Reanalysis produces
  zero drag runs, fixing nine false 0–100 results from position-noise overshoot.
  It completes in 24.55 seconds including compilation, without numerical budget
  or ambiguity errors. Original recordings remain unchanged.
- Long analysis releases the USB mutex, with concurrent state requests measured
  at 0 ms and cancellation verified. Stopping capture persists samples before
  final analysis so cancellation leaves the session and journal recoverable.
- Completion audit fixed stale live values during blocked/failed polls, replay
  values in gaps/past a trace end, and interrupted-capture visibility. Verified
  with local-clock, replay, interruption and incomplete-download regressions.
- Exact save/load size limits and reload verification prevent a successful save
  from disappearing from the library. Capture interrupts explicitly at its
  observation budget and retains the boundary/batch journal and incomplete reason.
- The MCU now returns a stable stop timestamp before SD flushing; host filtering
  excludes subsequent preview records, and preview overflow does not count as
  lost recording samples. Delayed/idempotent stops are covered by regressions.
- Completed drag targets survive a long subsequent cruise. Long stationary waits
  and recordings that begin while moving still permit later observed launches;
  the bounded attempt window never claims targets outside its evidence.

## Hardware verification completed

- The application ran on COM8 with PSRAM, CPU1 navigation, and a
  127,865,454,592-byte exFAT SD card. The tested image SHA-256 was
  `53a233561548221586ae22dbb22cae2ae648267267c4837d68494c405b720abc`.
- A 60.18949-second native app capture saved 10,940 IMU samples and 1,204 GNSS
  observations. All 10,939 adjacent IMU intervals are exactly contiguous in
  nanoseconds; intervals are about 5.501 ms. All GNSS observations are expected
  indoor no-fix records, with no fabricated position, speed, lap or drag result.
- Sensor, timing, capture, navigation, USB and SD error/drop deltas were all zero.
  Battery remained 96%, approximately 4.156–4.158 V, USB bypass, with no gauge or
  charger communication errors. LED configuration/readback remained functional.
  One startup IMU rejection and one startup command-UART overflow remain visible
  in historical counters; neither recurred during the recording.
- SD file 14 closed successfully: 1,689,600 bytes and 3,300 pages. Download and
  import passed every page/record CRC. All 12,144 sensor records match the USB
  capture exactly, including floating-point bits and nanosecond timestamps.
- Every raw IMU acquisition interval lies inside the recording boundary; there
  are no pre-start records. Stop reported five queued records and the host
  drained all five. The separately retained, unqualified group-delay model can
  shift effective support before Start; raw acquisition boundaries do not claim
  physical effective-time clipping.
- Evidence is retained under ignored `target/bench/native-hardware-3/` and
  `target/bench/native-capture-comparison-3.json`. Earlier failing traces remain
  under `target/bench/` for comparison.
- Boot diagnosis fixed a real CPU0 stack overflow. The final linker reserve is
  33,008 bytes; the USB poll frame is 1,424 bytes, with about 6.6 KiB of reviewed
  nested main/interrupt call frames. Integer-only IMU/GNSS interrupt paths were
  audited for absence of floating-point instructions and floating ROM helpers.
- Integer acquisition, buffered GNSS observations, faster validated CRC/SPI,
  direct JSON serialization and capability-negotiated compact draining resolved
  the measured timing/transport failures. Compact preview measured approximately
  181.8 IMU Hz and 20.0 GNSS Hz; full diagnostics refresh about once per second.
- The actual Windows release app was inspected through Computer Use. Demo lap
  analysis and replay worked. The app displayed 96% battery, 20.0 Hz GNSS, no fix, zero satellites, IMU
  ready, SD ready and the absent accessories correctly.
- Native capture persistence now retains bounded start/end diagnostic snapshots,
  exact available recording boundaries, sensor-error and queue-drop deltas,
  raw observation counts and interval coverage. Replay/analysis warnings keep
  sensor losses separate from transfer losses. JSON float round-trip parsing
  avoids the one-ULP changes found in the first USB/SD comparison.
- Native USB collector turns yield after 125 ms between complete responses.
  Stop drains the fixed FIFO count observed in its reply even when preview
  continues, retaining the journal and an interruption reason on failure.
  IMU stop filtering compares exact nanoseconds, not rounded microseconds.

## Scope limits

- Bluetooth is optional and remains disabled.
- Display support is implemented but the panel is absent; extra buttons remain
  disabled. Normal operation uses the app controls.
- Outdoor GNSS/PPS validation, measured mounting/filter-delay calibration and
  estimator qualification remain necessary before precision fused results.
- Stack call-frame audits and the bench recording are not runtime high-water
  measurements or proof of indefinite endurance under every workload.
