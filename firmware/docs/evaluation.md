# V2 Mini firmware evaluation

The rebuild integrates the HAL and includes receiver, IMU startup and transfer
recovery fixes established during connected-board evaluation. Host validation and a
linked image establish software readiness for bench evaluation. Hardware timing,
receiver return paths, calibration and battery characterization remain physical
acceptance gates. The firmware deliberately reports unqualified measurements.

## Start and observe

1. Build using the firmware README and flash through the existing loader/slot
   workflow. Use J6 USB-HS with DTR asserted. New installations enter recovery;
   send Run (`01 02 02 00`) and reconnect after reboot.
2. Read BootStatus (`01 02 07 00`); stage 10 means estimator startup completed.
   Stage 11 reports PSRAM verification failure. A failed estimator must leave
   USB and acquisition diagnostics accessible.
3. Poll Status (`02 01 01 00`), AcquisitionStatus (`03 01 05 00`),
   PeripheralStatus (`03 01 04 00`) and PowerStatus (`03 01 03 00`). Responses
   are the typed Postcard/COBS envelopes in `device/interface.rs`. USB packet
   boundaries are not frame boundaries. Existing command indices are preserved.
4. Confirm power verification, LED colors and all four button bits. Check panel
   orientation and gauge availability only when those accessories are fitted.
   Buttons are diagnostic-only. No thermistor
   curve, charging profile or gauge battery model is guessed by the application.

## Sustained acquisition

After stage 10, record counter baselines and run for at least 30 minutes, first
stationary and then through representative motion. Poll USB throughout. Acquisition
starts after worker memory/workspace initialization. Flash confirmation may still
interrupt startup acquisition; compare steady-state counter deltas after settling,
not only totals since reset. A display-equipped variant needs a separate run with
the display enabled and disabled.

| Check | Evaluation gate |
| --- | --- |
| IMU cadence | Raw admitted rate near the configured DEC5 rate (737.5 Hz nominal, 690.625..784.375 Hz from the sensor clock range; about 727 Hz expected on this board); investigate persistent reductions. Raw counts are separate from estimator acceptance. |
| Read timing | Captured-edge-to-completed-read latency below 600 us in normal operation, verified with a logic analyser. Late bursts increase timing errors and never enter the estimator. |
| Overload | No new steady-state drops under expected workload; queue high-water observations leave capacity margin (IMU 128, GNSS 16). |
| GNSS epochs | Advancing trusted rate reported honestly. Cold start without FINE time must not restart the receiver repeatedly. |
| PPS | Verify physical rising edges, 1 Hz rate and full CONFIG PPS readback. Lock, residual and age must remain healthy before fusion. |
| Estimator | With antenna reception and an accepted position/velocity seed: stage 10, advancing accepted/fused counts and trajectory epoch; record maximum_step_us, processing errors and late observations. Before that seed, raw IMUs drain while imu_accepted remains zero. |
| Resources | Memory check passes, no unexpected recovery resets, USB queryable and independent peripheral errors. |

The capture clock uses one hardware counter for PPS/DRDY. Measure actual filter
delays and alignment before narrowing uncertainty. Set mounting, calibration and
antenna lever arm in `profile/configuration.rs` before assessing measurement accuracy. The
receiver CONFIG readback must succeed independently of navigation reception;
unverified PPS configuration keeps fusion disabled. Readback has succeeded on
the connected board, including after the initial query timed out.

## Fault and recovery runs

- Interrupt GNSS delivery and stop advancing FINE epochs while traffic continues.
  Expect observable errors, bounded retry and receiver reconfiguration. Withhold
  PPS independently: lock must expire without arrival-time substitution.
- Drop the first CONFIG readback while retaining advancing navigation. Confirm
  raw counts/rates keep advancing, CONFIG retries after the response window and
  backoff, and a subsequent valid readback permits normal PPS lock/fusion.
  Wrong/malformed readback must never enable fusion. Fault the command UART
  independently and confirm it cannot starve or reconfigure healthy COM2.
- Interrupt IMU DRDY or communication. Expect errors, transfer to the initializer
  and recovery without repeated samples. Gaps must remain visible to processing.
- Exercise missing/faulted low-rate peripherals in a suitable bench setup.
  Acquisition must continue. LED/panel retries are independent; stale peripheral
  readings and power verification expire after three seconds. Exercise a stuck
  SPI UPDATE/USR completion: each FIFO operation must time out within 2 ms of
  CPU execution plus acquisition/maintenance preemption, reset SPI3, release CS,
  blank the panel and retry initialization after three seconds. Power readbacks
  must continue through repeated display resets and failed transfers. USB
  command latency can increase during main-executor initialization. Confirm
  IMU latency remains below its deadline with simultaneous GNSS traffic and
  missing/faulted power-bus devices.
- Keep input arrivals/publications advancing while rejecting estimator input or
  holding the retained trajectory epoch fixed. Speed/position/velocity must
  expire after the profile's 320 ms epoch-age budget, with degraded phase.
  Normal 120 ms fusion/smoothing latency and publication jitter must stay usable.
  Stop publication separately and confirm output expires. Request Recover with the worker
  live, verify loader access and the bench charging policy, then Run back into
  the application. Flash safety host regressions cover torn writes and boot
  selection, not electrical faults.

SD session logging and radio stacks are outside this acquisition profile. They
need their own application behavior and throughput evaluation.


## Connected setup

The current board has an attached GNSS antenna with partial sky view and no
display, battery or buttons. Panel rendering,
battery-gauge polling and button-expander polling are disabled by explicit
`DISPLAY_FITTED`, `BATTERY_FITTED` and `BUTTONS_FITTED` flags in board.rs.
Panel readiness, button state and battery values remain unavailable. LED/charger
health are still observed. Trusted GNSS time, PPS lock and navigation output must
be checked with the antenna attached before claiming navigation readiness. A
receiver fix or verified CONFIG readback alone does not establish successful
PPS association or estimator fusion. The earlier antenna-free capture only
established acquisition readiness.

Use tools/board_check.py with the host decoder to retain typed responses and raw
frames. ResetStatus is readable in recovery and distinguishes watchdog reset
from a retained Rust panic. ImuDiagnostics separates driver faults, late starts,
late finishes, overwritten captures and SPI timeout phase/context. Initial flash
confirmation can pause acquisition; compare steady-state deltas after settling.
Each flash program or sector erase holds the shared critical section from before
core 1 is parked until after it resumes. In esp-storage 0.10.0,
[`MultiCoreStrategy::with`](https://docs.rs/crate/esp-storage/0.10.0/source/src/common.rs)
parks outside the driver's
[private ROM-call lock](https://docs.rs/crate/esp-storage/0.10.0/source/src/lib.rs).
Without the outer shared lock, an interrupt can wait on a lock owned by the
parked core during either gap. The driver resumes core 1 on returned flash errors
as well as success; the outer lock then releases. This covers individual bounded
operations, including trial confirmation and Recover, rather than a whole upload.
Repeated confirmation/Recover cycles with a live worker must verify that the
intermittent watchdog reset no longer occurs. Measure their acquisition pauses
separately from the settled navigation gate; host tests cannot exercise CPU parking.
The firmware enforces a 1ms native-collection completed-read deadline, retains
600us for navigation, and reports
unqualified measurements until installation/calibration evidence is available.
The acquisition profile exercises raw acquisition and queue draining. The
navigation profile below also requires sustained estimator work and output;
electrical timing and measurement accuracy still require separate qualification.

### Reproducible stationary bench gate

Build the decoder with the Windows host target (the decoder enables the host's
standard-library critical-section implementation):

The host scripts require Python and the pinned serial dependency in
`tools/requirements.txt`. Install it into a local environment, for example with
`python -m pip install -r firmware/tools/requirements.txt` from the repository
root. The commands below use `python` for that environment's interpreter.

```powershell
# From firmware/
cargo build --release --locked
New-Item -ItemType Directory -Force ../build/bench | Out-Null
espflash save-image --chip esp32s31 --flash-size 16mb `
  --partition-table partitions.csv --target-app-partition slot0 `
  ../target/riscv32imafc-unknown-none-elf/release/firmware ../build/bench/firmware.bin
cargo build --manifest-path tools/board-probe/Cargo.toml --offline `
  --target x86_64-pc-windows-msvc --target-dir ../target/board-tools `
  --config 'unstable.build-std=[]'
# From the repository root, using a Python environment with pyserial installed
python firmware/tools/board_check.py `
  --decoder target/board-tools/x86_64-pc-windows-msvc/debug/aevia-board-decode.exe `
  --image build/bench/firmware.bin `
  --run --settle 30 --seconds 1810 --interval 5 --bench --stationary `
  --transport --power-diagnostics --fail-fast `
  --output build/bench/stationary.json
```

The device stays on a stationary desk, powered from USB. `--bench` writes a
companion `stationary.summary.json` verdict and exits unsuccessfully if a gate
fails. Raw frames and typed responses remain in the original JSON, including
partial captures on failure or interruption. A failed query retains its received
bytes and error even if framing, decoding or response validation fails. The final
poll occurs at the end of the requested duration. The verdict records actual observed device uptime and
duration, decoder and uploaded-image digests, rates, queue high-water marks and
initial reset provenance. An already installed image has no inferred image hash.
`--transport` requires the new matching firmware and decoder and adds raw UART
COM1/COM2 byte/error counters. The verdict reports per-port counter deltas, rejects
new UART errors and requires COM2 received bytes to advance on every poll even
without an antenna. COM1 may remain idle between CONFIG queries. The flag is
optional so older captures and firmware remain usable; captured transport data
automatically participates in offline acceptance when present.

An append-only `stationary.frames.jsonl` journal is flushed after each received
frame, before decoding, and after each decoded/error result. Session IDs and
record indices relate those events. This preserves received data if the host
process is terminated before its final JSON write. Reusing the same output path
appends another session to its journal. `--fail-fast` compares health after each
complete poll once two polls exist and ends on the first failed gate; the ongoing
checks do not require the unfinished requested soak duration. `--power-diagnostics`
retains charger failure-stage and bus-error detail and includes its baseline/final
values in the verdict; PowerStatus continues to govern acceptance.
`--fix` retains the receiver's independent position/velocity validity, age,
uncertainty and raw PPS association evidence through the appended GnssFixStatus
command. It requires a matching firmware/decoder and remains optional for old
captures. Its values can diagnose reception before PPS lock or estimator startup.

For scripts importing `Probe`, supply `args.output` to enable its journal and call
`probe.close()` in a `finally` block to close USB, decoder and journal handles.
`probe.query("power_diagnostics", bytes([1, 10]))` also works in recovery. Omitting
`args.output` keeps journaling optional for read-only or mocked callers.

The DEC5 gate requires advancing IMU counts at 675..800 Hz on every poll interval,
continuous IMU readiness without reinitialization or cycle-clock fallback, no new
acquisition/transfer/parser/processing errors, CONFIG timeouts or queue drops, no
retained worker failure or panic, stable reset provenance, fresh worker and
peripheral maintenance, verified charger policy and verified GNSS PPS CONFIG
readback. It accepts unavailable satellites, PPS lock and navigation. Existing
startup error totals and lifetime latency/high-water maxima are recorded; only
changes to error counters during observation fail the gate. Hardware timing must
still be measured independently. `--stationary` additionally checks gravity
magnitude at 8.8..10.8 m/s2 and angular-rate magnitude below 0.15 rad/s. These broad
orientation-independent bounds detect gross faults; they do not calibrate sensors.
Missing display, buttons and battery require no actions or measurements.
Five-second USB snapshots do not characterize sensor noise. The observed gyro
spread on this desk exceeds what intrinsic noise density alone would explain;
use rigid, vibration-isolated full-rate stationary captures and temperature
tracking before making noise, bias or timing-accuracy claims. The configured
LPF3 bandwidths are 280 Hz for gyro and 240 Hz for acceleration, with the
datasheet's Dyn1 scaling; this functional gate does not narrow those filters.

### Antenna-backed navigation gate

Use `--navigation` after PPS lock and navigation have settled. It implies
`--bench` and adds requirements on every recorded poll: phase `Navigating`,
trusted navigation time, valid PPS quality, PPS lock and age, fit residual within
the firmware's 50 us bound, finite non-null position/velocity/speeds and a
trajectory epoch within the profile's 320 ms age budget. Trajectory epochs,
accepted IMU counts and fused GNSS counts must advance at each poll. PPS edges
and accepted labels must advance over the observation. New clock resets, PPS
rejections, unsynchronized GNSS, rejected estimator input or late observations
fail the gate. Existing settled error baselines remain recorded.
When `--fix` is present, every poll must also contain valid finite raw position
and velocity, with reception and individual measurement ages no greater than
500 ms. Those checks apply only to the navigation profile.

```powershell
python firmware/tools/board_check.py `
  --decoder target/board-tools/x86_64-pc-windows-msvc/debug/aevia-board-decode.exe `
  --run --settle 30 --seconds 1810 --interval 5 --stationary --navigation `
  --transport --power-diagnostics --fix --fail-fast `
  --output build/bench/navigation.json
python firmware/tools/bench_acceptance.py build/bench/navigation.json `
  --minimum-seconds 1800 --stationary --navigation `
  --output build/bench/navigation.replay.json
```

The settling delay is not a fix timeout: allow the receiver to acquire time and
a solution before starting the acceptance capture. A capture that includes
cold-start acquisition correctly fails the sustained navigation gate. Use the
ordinary acquisition profile while diagnosing startup. Both profiles leave
`qualified=false`; these tests establish functional progress and output
availability, not calibrated accuracy. The normal five-second poll interval is
long enough to observe fusion progress and does not measure every estimator step.

Re-evaluate the retained evidence without a board, requiring the full 30 minutes:

```powershell
python firmware/tools/bench_acceptance.py build/bench/stationary.json `
  --minimum-seconds 1800 --stationary --output build/bench/review.json
python -m unittest discover -s firmware/tools -p 'test_*.py' -v
```

A short capture uses the same checks but establishes only its recorded duration.
Record a separate antenna-backed navigation soak and the physical fault/recovery
runs above before expanding acceptance to those operating conditions.

The USB/loader rejection check is also reproducible. Run it only after the
installed application has passed startup and its trial confirmation window.
It writes the inactive slot's first sector, rejects malformed or incomplete
uploads, and returns to the original application without selecting a candidate:

```powershell
python firmware/tools/board_fault_check.py `
  --decoder target/board-tools/x86_64-pc-windows-msvc/debug/aevia-board-decode.exe `
  --image build/bench/firmware.bin --output build/bench/faults.json
```

This checks recovery with the acquisition worker live, charging-disabled
readback in recovery, empty/unknown/trailing/malformed/oversized commands,
coalesced and fragmented frames, invalid upload sizes/headers/order/hashes,
Abort, malformed requests during an upload, and disconnect cancellation.
Reprogram the inactive slot with a valid image afterwards if both slots must
remain bootable. It does not exercise electrical power cuts or pre-Rust rollback.

`tools/board_rollback_check.py` tests a deliberately staged application panic and
then restores a supplied healthy image. Its candidate must reach USB startup and
panic on core 0 at stage 1, before application initialization or trial
confirmation. Supply the exact source hash and panic line for that build. The
fixture verifies the retained panic after automatic fallback, advancing IMU
samples and power policy, and a clean normal-image restoration. It retains both
image digests and the complete wire journal. The
[connected-board record](bench-readiness-2026-11-10.md) documents the tested
candidate, restored image and observed results.

## Release decision

This build is suitable for continued bench evaluation. It is not a qualified
measurement release. Completing the software hardening does not remove these
physical and deployment gates:

| Remaining gate | Quality or availability consequence | Required evidence |
| --- | --- | --- |
| Antenna-backed PPS and GNSS | Wrong pulse association or epoch semantics can bias fusion; loss/reacquisition must remain observable. | Physical pulse/DRDY timing, valid first seed, advancing fused epochs, delayed delivery and signal-loss tests. |
| Navigation workload on S31 | Raw acquisition before a seed avoids normal estimator work, so an antenna-free soak cannot establish processing capacity. | Sustained IMU acceptance, 50 Hz GNSS fusion and 200 Hz frontier progress without drops or growing lag. The host quota regression proves work-credit semantics only. |
| Installation and sensor timing | Nominal mounting, zero lever arm, filter delays, noise and correlation assumptions can bias estimates or understate uncertainty. The development timing approximation does not propagate independent timing uncertainty. | Measured installation/calibration, uncertainty coverage and a supported qualified timing path; retain qualified=false until then. |
| Rollback before Rust startup | A candidate that fails before application rollback runs can prevent automatic return to the healthy image. | Provision and fault-test a verified rollback-enabled bootloader; application fallback alone covers later failures. |
| Motion envelope and endurance | Saturation or excessive dynamics remove real measurement support; stationary acquisition does not establish field behavior. | Representative motion within the sensor envelope, longer stationary/moving runs and electrical interruption tests. |

The current profile excludes session recording, radios, display and battery
operation. These are separate product variants/features, not validated behavior
of this connected setup. Trial confirmation establishes functional acquisition
and task progress; it is not a navigation accuracy attestation.
