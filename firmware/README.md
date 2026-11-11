# V2 Mini firmware

The current build prioritizes race data collection. See [recording and USB
retrieval](docs/race-recording.md) for the no-app workflow and [candidate
validation](docs/race-candidate-2026-11-11.md) for verified results and remaining
hardware checks. It is not yet a hardware-validated race release.

Button-free sensor recording for the
ESP32-S31 / UM980 / SCH16T-K01 board. The application configures the UM980 for
50 Hz binary BESTNAVXYZ and attempts each SCH16T DEC5 epoch (737.5 Hz nominal),
rejecting late or ambiguous reads.
The receiver's actual advancing epoch rate is reported; requesting 50 Hz does
not guarantee that rate in every receiver firmware/positioning mode.

Core 0 handles acquisition, USB and indicators. Acquisition begins after the
worker completes its PSRAM check and workspace setup, avoiding startup queue
overflow and cache contention. Core 1 owns the SD card and filesystem, with a
4 MiB PSRAM capture queue. Original IMU bursts, UART data, physical PPS edges,
temperature, configuration and fault evidence are recorded before estimator
filtering. The collector starts without a fix or PPS lock. Live trajectory
processing is disabled by `configuration::DATA_COLLECTION_ONLY`; its existing
implementation remains available for later integration. Capture never waits
for SD writes or USB. Overload increments counters and leaves explicit sequence
gaps. Files rotate near 2 GiB and USB commands retrieve their checksummed prefix.

The application consumes `aevia-device-hal` directly for SCH16T, UM980
framing/messages/commands, SDMMC, BQ25622E, LP5813, ST7789, MAX17048, TCA9536A and flash.
HAL startup handling now acknowledges both post-EOI status reads before judging final sensor health; persistent faults still fail initialization. `device/board.rs` constructs the fitted resources
using HAL's wiring constants. `device/platform` owns missing S31 capture,
PMA/cache setup and bounded panel bus access; `device/timing` owns clock fitting
and timestamp policy. Device code also owns scheduling and application policy. The
parent workspace excludes HAL's independent workspace; its exact `firmwar[e]`
member glob prevents Cargo's explicit-member prefix rule overriding exclusion.

IMU acquisition runs alone on SWI2 at priority 5. GNSS and PPS maintenance
run on SWI3 at priority 3, so UART decoding cannot hold up an IMU read. GPIO
and UART wakeup interrupts run at priority 6. The shared power bus has one owner on the main executor, below the
SDK priority-1 timer that wakes timed acquisition waits. It enforces the charger
policy and maintains fitted gauge, button and LED health. USB, optional panel
and IMU initialization also run on the main executor; ownership channels move the IMU between initialization
and sampling. Blocking panel delays and power-bus recovery cannot monopolize
acquisition. USB can be delayed by initialization (approximately 420 ms for the
panel); display transfers yield between rows. The local panel bus limits each
64-byte FIFO operation, including register update, to 2 ms. A timeout resets
SPI3, releases CS, blanks the panel and triggers reinitialization after three
seconds. The vendor's 20 ms I2C transaction deadline excludes bus clearing/
recovery, which can add 50 ms per recovery phase. These operations remain
preemptible by acquisition.

The development profile uses factory sensor scaling, an identity mounting
rotation and zero nominal antenna lever arm with uncertainty. GPIO42/PPS and
GPIO14/IMU DRDY are latched against one 10MHz hardware counter. Valid PPSSTATUS
labels fit GPS measurement epochs to this counter, independently of UART
delivery time; GNSS fusion waits for timing lock. The IMU uses hardware edge
times, nominal LPF delay correction and gyro/acceleration support alignment.
`qualified` is always false. Set the real installation and calibration in
`src/device/profile/configuration.rs` before relying on measurement accuracy. Filter delays
still need board calibration. The explicit development timing approximation
retains the supplied uncertainty as metadata but does not propagate it or
resample the interval; those inputs remain degraded. See [timing implementation](docs/timing.md)
for sources, assumptions, rejection rules and board validation. The firmware
records original evidence to an existing supported exFAT card. Radios remain
disabled. No card formatting, automatic deletion or overwrite is performed.

For the connected single-cell 2000 mAh, 3.7 V battery, the development charging
profile targets 800 mA and a conservative 4.10 V ceiling through the GPIO6/7
power I2C bus. Limits are read back before enabling charge; the watchdog is
disabled and existing input limits, power paths and temperature protections
are preserved. Actual charging may be reduced or suspended by those limits.
The policy is checked every second in application and recovery modes.
This is a running-firmware policy, not a hardware
interlock: the charger can use its autonomous defaults before firmware runs,
including during ROM recovery. The board's two-wire battery connector has no
cell thermistor input; the onboard thermistor measures its own location.
`DeviceCommand::PowerStatus` (appended index 3, frame
`03 01 03 00`) returns the part ID, control/status register readbacks,
verification result and error count, including in recovery. Existing command,
response and snapshot layouts are unchanged; the new response is appended at
device response index 2. The shared bus uses 100kHz to meet the charger's
standard-mode timing requirements. See [TI BQ25622E Rev. C, sections 8.4.1,
8.5.1 and 8.6.2.10](https://www.ti.com/lit/ds/symlink/bq25622e.pdf).

`PowerChargeStatus` (device command 12, response 11) returns fresh readbacks of
the programmed charge limits, charge-enable state, input-current limit, charger
status and faults. These are not measured battery-current values. Existing
`PeripheralStatus` reports battery voltage and the gauge's SOC estimate.
Use `board_check.py --power-charge --bench` for the charging-profile gate;
older captures without this evidence retain the charging-disabled bench gate.

## Build and flash

For app development, start with the [versioned application interface](docs/app-interface.md).
`System::ProtocolInfo` (`01 02 08 00`) reports schema/firmware versions, per-mode
command masks, fitted accessories, supported features and frame/freshness limits.
Existing telemetry layouts are preserved; discovery is appended at system
command index 8 and response index 5.

On Windows, produce a matching application image, ELF, decoder and checksum
manifest from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File firmware/tools/build_release.ps1
```

The helper runs host tests, builds with pinned dependency lockfiles and packages
artifacts under `firmware/build/releases/<UTC timestamp>/`. It never flashes.
`manifest.json` records tool versions, source/artifact hashes and the decoder's
compiled ProtocolInfo; `build-status.json` must say `complete: true`. A bundle
does not by itself establish hardware acceptance. Pass explicit `-Cargo` or
`-Espflash` paths if the tools are outside PATH and `~/.cargo/bin`.

```sh
cargo build --release -p firmware
espflash flash --partition-table partitions.csv \
  ../target/riscv32imafc-unknown-none-elf/release/firmware
```

The `espflash flash` command uses the MCU's ROM download connection. An already
installed application on J6 uses the inactive-slot USB updater: generate an
application `.bin` with `espflash save-image`, then pass it to `board_check.py
--image` with the matching host decoder. The Windows build/upload commands are
in the [evaluation procedure](docs/evaluation.md#reproducible-stationary-bench-gate).

Use J6 (USB-HS) for the binary command connection. A new installation starts
in recovery. Send `Request::System(Command::Run)` to enter the application.
For initial bring-up, that complete encoded frame is `01 02 02 00` in hex;
open the CDC port with DTR asserted and send those four bytes. USB reconnects
with the application PID after reboot. Application `DeviceCommand::Status`
is the frame `02 01 01 00`; its response uses the typed schema below.
`Request::System(Command::Recover)` persists recovery mode and reboots back to
the loader. Fault resets also enter recovery. Recovery keeps sensor acquisition
and the estimator stopped while programming the inactive firmware slot.

USB enumerates before application hardware initialization, with a two-second
startup grace period. `Request::System(Command::BootStatus)` (`01 02 07 00`)
reports the last reached stage: 0 before startup/in recovery, 1 before PSRAM,
2 before second-core startup, 3 with the worker launched and awaiting release, 4 when the worker is
released with acquisition still pending,
5 during the full PSRAM check, 6 while copying the estimator workspace,
7 during metric compilation, 8 during preflight, 9 during session creation,
10 when the session is ready, and 11 if the memory check fails. This command and its response are
appended to the existing enums; existing wire indices are unchanged.

PSRAM uses the fitted board's tested 125 MHz setup. Before the estimator's first
store, core 1 restores the ROM PMA split from the previous base firmware:
flash remains read-only, and only the fitted 16 MiB PSRAM window becomes writable
and non-executable. Two complementary address-dependent patterns are flushed
and verified across all 16 MiB before handing the range to the estimator. An
unexpected protection map or memory mismatch reports a resource failure while
keeping USB available.
The estimator reserves a 64 KiB internal-SRAM stack for its nested preflight
validation as well as steady-state processing. Flash mode changes use the
storage driver's automatic core parking so Recover works with the worker live.

GNSS acquisition continues when the full PPS configuration cannot be read back.
This reports a GNSS error
and keeps PPS association/fusion disabled; raw receiver epoch counts and rates
can still be inspected. CONFIG readback has a two-second response window and
retries after a three-second backoff while both UARTs continue decoding. A
failed command return path cannot restart healthy COM2 reception. Only the
exact checksummed GPS/rising/1 Hz/zero-delay readback opens the PPS gate; three
valid pulse labels are still required for timing lock. No navigation timestamp
is substituted for a missing timing lock.

## Wire interface

Every request and response is **Postcard, then COBS, terminated by `0x00`**.
USB packet boundaries are independent of frame boundaries. Commands use a
272-byte receive buffer, with oversized frames discarded through their zero
terminator. Malformed commands abort any firmware update in progress.

`src/protocol.rs` defines the v1 envelopes:

- `Request::System(commands::Command)`: Info, Abort, Run, Begin, Data, Commit,
  Recover, BootStatus. Firmware upload chunks are at most 256 bytes and carry raw bytes,
  with a raw 32-byte SHA-256 digest in Begin. Upload commands work in recovery.
- `Request::Device(device::interface::DeviceCommand)`: Status, Latest,
  SetIndicators, PowerStatus, PeripheralStatus, AcquisitionStatus. Application
  commands are independent of loader commands. PowerStatus also works in
  recovery. Both Status and Latest return the current snapshot.
- `Response::System(commands::Response)` or
  `Response::Device(device::interface::DeviceResponse)`.

For a Rust caller, encode with `postcard::to_slice_cobs(&request, &mut buffer)`
and decode the reply after collecting a zero-terminated frame. System enum
indices are Info=0, Abort=1, Run=2, Begin=3, Data=4, Commit=5, Recover=6.
The outer request indices are System=0 and Device=1. Variant order and field
types are the protocol contract; do not reorder variants in an existing wire
version. This is incompatible with the previous newline/text protocol.

The transport-independent device interface supplies ECEF position/velocity,
horizontal/vertical speed, current IMU values, sensor rates, satellite count,
estimator phase, PPS lock/age/fit residual, timing rejects, dropped samples and
capture/processing error counters. Snapshot now includes a `TimingStatus` field;
host decoders must use the matching current schema.
Reads copy the latest snapshot; they do not consume sensor queues. Future
Wi-Fi/Bluetooth transports can call the same interface without owning the
estimator. There is no unsolicited high-rate USB stream.

`PeripheralStatus` (command index 4, frame `03 01 04 00`) reports panel/LED
readiness, per-peripheral error counts, read-only battery voltage/SOC and button
state. The mask is active-high in P0..P3 order (SW3, SW6, SW4, SW5); buttons have
no assigned actions. Readings expire after three seconds without an update.
Gauge SOC uses the gauge's current model and is not a calibrated battery
assessment. An unavailable gauge returns missing values. This firmware does
not program a battery model or enable charging.

`AcquisitionStatus` (index 5, frame `03 01 05 00`) reports raw acquisition
counters, last accepted IMU/trusted GNSS arrival times, UART/parser errors,
actual GNSS rate, queue depth/high-water observations, drops, maximum observed
DRDY-to-read latency and PPS/capture diagnostics. It works even if PSRAM or
estimator startup fails. Counters/depths are sampled separately across cores;
high-water observations can miss a concurrent drain. The latency maximum
includes rejected late bursts; the IMU acceptance deadline is 1 ms for native
collection and 600 microseconds for navigation. The active bound is recorded
in each file's IMU configuration metadata; raw timestamps remain available.
Existing snapshot layouts and enum indices remain unchanged; the new responses
are appended at device response indices 3 and 4.

## Indicators

The optional screen and four RGB indicators show acquisition and estimator health
without button input. Display updates yield between scanlines so that screen
traffic cannot monopolize the acquisition executor. `SetIndicators` controls
display enable and LED brightness. Navigation freshness follows the trajectory's measurement epoch, independently
of recent input arrivals or snapshot publications. The current profile allows
100 ms fusion delay, 20 ms smoothing lag, one 100 ms publication interval and
100 ms processing/cadence margin (320 ms total). Missing, future or older
epochs clear speed/position/velocity and degrade navigation. Publication also
expires after 500 ms if the worker stops. A new advancing epoch restores output.

Flash-safety, timing, IMU alignment, GNSS validity/cold-start, power-policy and
wire/freshness regressions run on the host. On this Mac:

```sh
cargo test -p firmware --lib --target aarch64-apple-darwin \
  --config 'unstable.build-std=[]'
cargo build --release -p firmware
```

Use the host triple `x86_64-pc-windows-msvc` on Windows. Host tests use the
standard-library critical-section implementation; device builds retain the HAL
implementation. See [evaluation procedure](docs/evaluation.md) and the
[rebuild plan](docs/firmware-rebuild.md) for acceptance criteria and scope.


## Connected board configuration and diagnostics

The current connected setup has a GNSS antenna and a user-reported single-cell
2000 mAh, 3.7 V battery (102560), with no panel or buttons.
`device/configuration.rs` sets `DISPLAY_FITTED=false`, `BUTTONS_FITTED=false` and
`BATTERY_FITTED=true`: battery voltage and the gauge's uncalibrated SOC estimate
are read once per second. No panel task or button polling runs. Missing buttons
report unavailable rather than an unpressed control.
The current battery profile is an initial low-current charging configuration,
not a manufacturer-qualified cell model. Enable the corresponding fitted-accessory flag before
using that accessory. Limited sky view can delay a receiver fix. Missing
navigation/PPS lock must not cause receiver reset loops or manufactured output.

Additional device commands append to the existing schema:

- `GnssStatus` (index 6): verified CONFIG readback, query/timeouts, trusted-time
  readiness and raw PPS quality, independent of estimator startup.
- `ImuDiagnostics` (index 7): retained HAL initialization/transfer failure,
  identity and raw startup status. Persistent faults never become healthy data.
- `ResetStatus` (index 8): raw S31 reset reason and checksum-validated retained
  panic source hash/line/core/stage. Also available in recovery.
- `GnssTransportStatus` (index 9): independent COM1/COM2 received-byte counts,
  UART error categories and the last error's port/time. The response is appended
  at index 8. Overflow, glitch, framing and parity faults remain distinguishable
  from parser failures; successful traffic never erases the retained evidence.
- `PowerDiagnostics` (index 10, response index 9): retained charger failure
  stages, register/operation and concrete I2C cause, with failed control/status
  readbacks and subsequent successful-attempt counts. Also available in recovery.
  Timeout, NACK and a policy mismatch remain observable after verification
  recovers. Existing `PowerStatus` is unchanged.
- `GnssFixStatus` (index 11, response index 10): independent raw receiver fix,
  per-field validity/freshness and retained PPS-label diagnostics, available
  before estimator initialization or PPS association succeeds. This is separate
  from the fused Snapshot and remains unqualified.

Recovery mode intentionally leaves application LEDs off. Send Run to start the
application. Watchdog eligibility follows worker, capture-maintenance, GNSS-owner
and IMU-owner progress with
bounded startup grace; it does not require satellites or a valid navigation fix.
OTA candidates start NEW and require sustained worker/capture health, advancing
raw IMU samples and a live GNSS owner before confirmation. A fix is not required.
The collection build additionally requires successful SD persistence without
recording overflow or storage errors before confirming an OTA trial.
An unconfirmed candidate can select the previous healthy image when Rust startup
runs. Faults before Rust startup still require a separately provisioned,
verified rollback-enabled second-stage bootloader.

The GNSS adapter preserves independent position/velocity validity and dates
health at the earliest applicable measurement epoch. Receiver solution/correction
ages are checked per applicable field against the shared two-second policy;
position age never invalidates independent Doppler velocity. CONFIG verification
renews every 30 seconds and is revoked on timeout or contradictory readback.
Repeated transport/navigation failure escalates to hardware receiver reset;
normal cold-start traffic and missing CONFIG replies alone do not.

To capture typed hardware diagnostics, install pyserial and build the host decoder:

```sh
CARGO_TARGET_DIR=/tmp/aevia-board-tools cargo build \
  --manifest-path tools/board-probe/Cargo.toml --target aarch64-apple-darwin \
  --config 'unstable.build-std=[]'
python3 tools/board_check.py \
  --decoder /tmp/aevia-board-tools/aarch64-apple-darwin/debug/aevia-board-decode \
  --run --settle 30 --seconds 60 --expect-imu --output /tmp/aevia-board.json
```

Pass `--image /path/to/application.bin` to program the inactive slot first.
The probe keeps complete wire frames and decodes them against the actual firmware
schema. `--expect-imu` requires advancing IMU counts at every poll, continuous
readiness, no reinitialization and no new IMU errors or queue drops during the
observation window. Use `--settle` to exclude startup. It does not establish
timing or accuracy qualification.

Short IMU SPI delays and controller deadlines use a checked CPU0 cycle counter
at the configured fixed frequency, with an observable vendor-timer fallback.
This removes shared SYSTIMER polling from the hot transfer path; measurement
epochs still come exclusively from MCPWM capture. SPI2 recovery is bounded by
a 600us FIFO-operation timeout, independently of the complete-burst admission
limit (1ms native collection, 600us navigation). Delayed or overwritten
measurements remain rejected.
Capture wrap extension is fast; a separate 100ms wall-clock validation detects
stopped/wrong-rate clocks without frequent reads resetting its reference.
The supervised watchdog bounds outages far below the counter's half-wrap.

In the optional estimator build (`DATA_COLLECTION_ONLY=false`), before the
first accepted GNSS position/velocity seed, the worker drains raw
IMUs and updates diagnostics but avoids futile estimator initialization work.
After that seed, ordinary estimator ingestion uses original hardware timestamps
and sequences. This leaves raw acquisition useful during antenna-free bring-up.
That workload does not exercise normal estimator stepping; navigating CPU
throughput still needs an antenna-equipped run.
