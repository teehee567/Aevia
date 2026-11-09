# V2 Mini firmware

Button-free sensor acquisition and onboard `aevia-trajectory` processing for the
ESP32-S31 / UM980 / SCH16T-K01 board. The application configures the UM980 for
50 Hz binary BESTNAVXYZ and reads every SCH16T DEC4 epoch (about 1.475 kHz).
The receiver's actual advancing epoch rate is reported; requesting 50 Hz does
not guarantee that rate in every receiver firmware/positioning mode.

Core 0 handles acquisition, USB and indicators. Core 1 owns the estimator, with
internal-SRAM hot state and an exclusively reserved PSRAM history. Navigation
runs at 200 Hz with a 100 ms fusion delay and 20 ms smoothing lag. Separate
bounded IMU/GNSS queues never wait for USB or the display. Overload increments
counters and leaves a time gap instead of silently integrating repeated data.

The development profile uses factory sensor scaling, an identity mounting
rotation and zero nominal antenna lever arm with uncertainty. GPIO42/PPS and
GPIO14/IMU DRDY are latched against one 10MHz hardware counter. Valid PPSSTATUS
labels fit GPS measurement epochs to this counter, independently of UART
delivery time; GNSS fusion waits for timing lock. The IMU uses hardware edge
times, nominal LPF delay correction and gyro/acceleration support alignment.
`qualified` is always false. Set the real installation and calibration in
`src/device/profile.rs` before relying on measurement accuracy. Filter delays
still need board calibration; their nonzero uncertainty uses `traj`'s explicit
development timing approximation. See [timing implementation](docs/timing.md)
for sources, assumptions, rejection rules and board validation. The firmware
does not record sessions to SD or enable radios.

## Build and flash

```sh
cargo build --release -p firmware
espflash flash --partition-table partitions.csv \
  ../target/riscv32imafc-unknown-none-elf/release/firmware
```

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
2 before second-core startup, 3 before sensor setup, 4 after sensor setup,
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

GNSS acquisition continues when the full PPS configuration cannot be read back
on the fitted board's incomplete UART return paths. This reports a GNSS error
and keeps PPS association/fusion disabled; raw receiver epoch counts and rates
can still be inspected. No navigation timestamp is substituted for a missing
timing lock.

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
  SetIndicators. These commands work in application mode and are independent
  of loader commands. Both Status and Latest return the current snapshot.
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

## Indicators

The screen and four RGB indicators show acquisition and estimator health
without button input. Display updates yield between scanlines so that screen
traffic cannot monopolize the acquisition executor. `SetIndicators` controls
display enable and LED brightness. A stale or unavailable estimate is shown as
unavailable rather than retaining a speed indefinitely.

Flash-safety and protocol regression checks remain in `src/loader/tests.rs`;
focused timing regressions are in `src/device/timing/tests.rs`.
Run these on Windows with `cargo test -p firmware --lib --target
x86_64-pc-windows-msvc --config 'unstable.build-std=[]'`. Host tests use the
standard-library critical-section implementation; device builds retain the HAL
implementation.
No additional sensor test harness is required to build the firmware. The
implementation needs board validation for sustained acquisition rate, queue
drops, processing time, PSRAM initialization and the fitted display.
