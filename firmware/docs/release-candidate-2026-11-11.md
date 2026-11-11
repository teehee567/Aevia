# Initial firmware candidate: 2026-11-11

This is the historical navigation candidate. The current collection-first
candidate is described in [race-candidate-2026-11-11.md](race-candidate-2026-11-11.md).

**Hardware acceptance is incomplete. Do not treat this candidate as a verified
initial release.** Only firmware, its host tools and its documentation changed.
The board became available after reconnection but disappeared again before the
next combined image could be installed. The latest source includes changes
that are still awaiting a board test. A further power/USB reconnect is requested.

## Connected-board findings

The USB-powered board had an antenna, with no battery, buttons or display.
Charging stayed disabled. The installed application supplied trusted GNSS time
and approximately 50 advancing receiver epochs per second. A 20-second baseline
had approximately 1454 admitted IMU samples per second with no new sensor,
transport or queue-drop errors, but every PPS label was rejected and no
navigation observations reached the estimator.

Diagnostic images established valid standalone position and independent Doppler
velocity with 27–28 satellites. The receiver reported PPS quality 3 and verified
GPS/rising/1 Hz/zero-delay configuration. Its PPS payload carried the preceding
50 Hz observation tag (`...980` milliseconds) while the header named the next
integer second. The separately decoded signed offset was about -0.373 ms; it
does not explain or correct the 20 ms tag difference.

After reconnection, the payload instead lagged its header by 160 ms. This
disproved the earlier inference that it always represented the immediately
preceding 50 Hz observation. The adapter now correlates an already integral
FINE/GPS header with the configured physical edge; it never rounds the payload
or applies its unexplained offset. Startup requires a unique recent capture,
then consecutive pulses and the original fit residual bound establish lock.
The documented header and CONFIG contracts support this correlation, but the
fractional payload's precise semantics and electrical timing remain unqualified.
See [timing.md](timing.md).

The first diagnostic implementation caused many late IMU reads by performing
fix validation inside a global critical section. Validation now runs outside
the lock. Later diagnostic runs still had intermittent IMU timing rejections;
steady-state acceptance of the final source remains outstanding. Wide-integer
clock fitting and projection now also execute outside shared interrupt locks,
using bounded revision-checked publication to preserve concurrent captures.
The firmware-local release configuration now uses speed optimization.

One trial startup returned to recovery with reset reason 7 (core watchdog),
without a retained Rust panic. A repeat installation of the same image ran
through a 40-second startup capture and a further 20-second observation. A
subsequent mapped-PPS diagnostic image lost USB after its last complete poll at
approximately 13 seconds uptime. That poll had no accepted PPS pairs or estimator
observations. Windows subsequently reported no connected Aevia USB device,
including no failed USB enumeration. A physical USB reconnect was requested.
The user subsequently restored the USB connection. The original disappearance's
cause remains unverified; the first retained capture after reconnection reported
a power-on reset and a valid independent receiver fix.

Source review found a separate concrete flash lock-ordering race in the pinned
SDK: it can park core 1 before masking core 0 interrupts, allowing an interrupt
to wait on a lock held by the parked core. The final source acquires the shared
critical section before each flash program/sector erase and keeps it through
unparking. The fix has now been installed through the inactive-slot updater;
repeated recovery and fault acceptance remain outstanding.

## Navigation workload findings after reconnection

The installed candidate reached three PPS pairs with a 50 ns fitted residual and
admitted GNSS and IMU observations to the estimator. This exposed overload during
initialization: admitted IMU cadence fell to approximately 1 kHz, late reads and
queue drops accumulated, and GNSS transport fell behind. No sustained fused
trajectory was produced. The later speed-optimized, short-lock image still
showed this initialization overload. These captures are failed acceptance, not
release evidence.

The next profile uses the sensor's supported DEC5 setting on all six axes:
737.5 Hz nominal, approximately 727 Hz expected on the fitted sensor. It retains
the 600 us read deadline, verifies consecutive sensor counters and real interval
support, and preserves the unqualified LPF3 delay and uncertainty model. The
200 Hz navigation cadence remains unchanged. Readiness still depends on physical
validation; changing the configured cadence does not excuse drops or late reads.

SCH16T conversion now multiplies by compile-time reciprocal sensitivities rather
than repeatedly dividing in software. Exhaustive signed 20-bit input tests
across every supported range bound the difference below 1e-8 sensor counts.
Initializer work now stops when its accepted paired GNSS support expires;
one actually admitted IMU interval first lets the engine clear stale evidence.
Idle calls are skipped during initialization. Active navigation and degraded
inertial propagation remain unaffected by that gate.

## App interface and build

- `System::ProtocolInfo` discovers the protocol/schema, version, supported
  commands, compiled accessories, features and frame/freshness limits in either
  application or recovery mode.
- `Device::GnssFixStatus` exposes independently valid, expiring receiver fixes
  and raw PPS evidence before estimator alignment. It is distinct from fused
  `Snapshot` navigation and never claims qualification.
- Existing command indices and telemetry layouts remain unchanged. The new
  interfaces are described in [app-interface.md](app-interface.md).
- Trial confirmation requires recent progress from every required task.
- `board_check.py --navigation --fix` checks actual navigation/PPS progress;
  the earlier acquisition-only gate cannot certify navigation.
- `tools/build_release.ps1` builds the image, matching decoder, interface guide,
  pinned dependency records and checksum manifest. It does not flash or declare
  hardware acceptance. The final candidate bundle is under
  `build/candidate-2026-11-11-r2/`. The earlier bundle is retained separately
  and does not contain the latest timing/performance changes.

The optimized device build and strict device Clippy passed. Firmware regressions
(120), HAL regressions (181), Python tool regressions (32) and decoder tests (2)
passed: 335 automated tests. The build bundle records exact image/source hashes.

## Remaining release gate

1. Restore power/USB access, inspect reset evidence, then load the combined
   candidate through the inactive-slot updater and
   verify sustained startup and trial confirmation.
2. Verify PPS association, three-pulse timing lock and advancing estimator
   initialization/fusion on the antenna-equipped board. Diagnose throughput or
   initialization failures without relaxing validity or timing checks.
3. Resolve any new steady-state IMU timing errors and run the documented
   30-minute stationary navigation gate, with raw fix/transport/power evidence.
4. Repeat Recover/Run, update rejection and rollback checks with the final flash
   coordination fix. Preserve the image digest and complete capture verdict.

Workspace evidence is retained under `build/release-check/`, including
`baseline.json`, `fix-first.json`, `pps-sequence.json`, `offset-immediate.json`,
`offset-low-poll.json`, `pps-fixed-o3-retry.json` and their raw frame journals.
Additional reconnect evidence is in `reconnected-before.json`,
`candidate-startup.json`, `timing-speed.json` and `timing-speed-reconnect.json`.
The `timing-speed` upload capture lost its USB handle during early startup;
subsequent reads found the application running with advancing uptime and no
retained panic. This interruption remains recorded as a failed capture.
Windows later recorded removal at 02:15:26 Sydney time, before `dec5-startup`
could even open the serial device. That failed upload attempt sent no commands
and installed no bytes. The new DEC5 image is built but not installed. Windows
also recorded earlier standby transitions, but none coincided with that latest
removal; a host sleep cause has not been established.
These diagnostic captures are not final-candidate acceptance. Precise receiver
coordinates remain in the local captures, not this repository summary.

This acquisition profile still has no radio stack or SD session recording.
Calibration, mounting, antenna lever arm, motion and electrical timing tests
remain separate from functional app integration. Measurements stay unqualified.
