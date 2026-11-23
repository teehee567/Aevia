# V2 Mini firmware

The application requests CRC-checked native UM980 SPPNAVB at 50 Hz in SIGNALGROUP 8
and records every validated SCH16T DEC5 native interval: nominally 737.5 Hz,
about 727 Hz on the connected sensor. Navigation separately averages four native
intervals. SD recording and optional raw USB capture retain the native rate.
The app driving display requests compact snapshots at 5 Hz. Sensor work runs independently
of USB. It monitors the charger/fuel gauge and drives four functional RGB LEDs.
The connected board has no display and its extra buttons are unavailable, so
`src/configuration.rs` disables both. Their absence is reported as optional;
it does not cause a system fault. When explicitly enabled for fitted hardware,
the ST7789 shows fused speed, sensor and recording status and button 1 toggles recording.

Sessions persist independently on an existing exFAT microSD volume and can also
be captured by the USB host through the Aevia app. CPU1 owns native four-bit SD
and all filesystem I/O alongside the bounded navigation worker. Separate queues
hold 384 SD sensor samples and 32 raw GNSS chunks and report their own overflows.
USB starts with an internal 128-sample queue and promotes to a 4,096-sample queue
in an exclusively reserved PSRAM tail after its startup test and CPU0 protection
checks. `usb_queue` reports actual backing, capacity and any fallback error.
Recording files reserve up to 128 MiB contiguous extents, rotate when full, and retain CRC-protected committed
pages after interrupted recording. Start opens the SD extent before setting
the recording boundary. A failure within ten seconds returns an
error without starting later. Stop drains the SD queue and flushes a final
closed page. The recorder never formats the card or deletes existing files;
FAT32/GPT volumes are currently unsupported, and dirty exFAT volumes permit only
retrieval. Live telemetry and recording controls work over USB or Bluetooth LE. SD downloads use USB.

The acquisition adapters were recovered from this repository's `76c78f5`
implementation, with capture, timing and memory support adapted from `31a20b4`.
They retain SafeSPI CRC/address/status/counter checks and the UM980 CRC checks.
MCPWM0 latches GPIO14 IMU DRDY and GPIO42 GNSS PPS against a shared 10 MHz
hardware counter. PPS mapping requires checksummed receiver configuration
readback and consistent pulse labels; missing or stale mapping remains absent.
Original IMU vectors, nanosecond capture/interval, separately aligned gyro,
effective support and 1.2 ms delay uncertainty are retained in USB and SD data.
GNSS retains receive time, GPS epoch, velocity latency and optional mapped
epoch/uncertainty/generation. An indoor no-fix result is normal and remains invalid.

CPU1 runs Traj's `engine::driving::DrivingEstimator` for the live dashboard.
It uses a fixed-size internal SRAM history of 96 IMU estimates to correct
velocity at the GNSS measurement epoch. The processor, including race state,
occupies 5,280 bytes in the V2 Mini release link map. The sensor handoff queues
and worker stack use additional internal RAM. CPU0 can use the final MiB of
fitted PSRAM for raw USB preview after the startup memory test.
See the [hardware validation record](../data/highrate-ppk-validation.md)
for measured sample rates, USB and SD comparisons, and the validated firmware
images. The [20 Hz baseline](../data/aevia-implementation-status.md) records
the earlier firmware and app checks.

Traj integrates the gyro and gravity-compensated specific force between GNSS
fixes. Start with a valid fix and hold still for at least half a second to
initialize tilt and gyro bias. A horizontal change in velocity establishes yaw,
including on a reversing launch. Confirm the rigid forward/left/up mounting
to display signed vehicle G. Recording uses the roughly 182 Hz estimator path
for preliminary gate and drag timing. SD also stores the committed run setup,
5 Hz driving snapshots, final totals and native observations.

Short IMU gaps of at most 12 ms use the neighboring measurements to approximate
the missing impulse. The navigation status counts these intervals. Longer gaps
interrupt the attempt and require fresh at-rest alignment. A GNSS outage over
one second suppresses displayed motion. The production build without the
development feature withholds these preliminary estimates. The compact filter
assumes colocated IMU and antenna. Its build-time sensor-to-body quaternion
and timing uncertainty are exposed in status.
Starting a recording resets driving metrics and keeps the live estimator warm.
The receiver time of week advances the local GNSS measurement clock. A PPS map
captured with the observation disciplines its offset. During PPS outages the
estimator uses the last measured transport delay and the receipt clock. Offset
corrections are bounded so the time base stays continuous through PPS generation
changes. A discrepancy greater than 300 ms interrupts the run and realigns.
The SD record preserves the original timing evidence for each fix.
With continuous IMU support, a GNSS outage during launch preserves tilt.
Fresh fixes restore speed output, and subsequent acceleration can establish yaw.

Firmware reserves a 64 KiB CPU1 worker stack shared by storage and navigation.
This RTOS does not save floating-point task context. CPU1 has one floating-point
worker, and the idle and interrupt paths use integer operations.
CPU0 acquires and validates native IMU intervals in a priority-2 interrupt
executor, using 8 MHz SafeSPI and a table implementation of the same CRC8.
GNSS UART/configuration, integer SPP/PPS decoding and bounded raw-byte copies run
at priority 1; GPIO wakes IMU
acquisition at priority 3. These paths are integer-only: the SDK interrupt entry
does not preserve floating-point registers. Bounded channels hold 256 IMU
intervals, 24 parsed SPP observations, and 16,384 packed COM1 bytes including
per-read metadata. Ordinary context decodes PPK, scales samples and publishes
them. Raw reads retain their original MCU receipt timestamps; SPP/PPS timing is
captured in the integer receiver. Losses reset the affected stream's decoder
after earlier queued bytes are consumed. COM2 uses a 32-byte FIFO threshold
and COM1 uses 64 bytes. Both use the hardware RX-idle timeout. When two
reads smaller than 16 bytes arrive less than 250 µs apart, the UART executor
waits 250 µs to let bytes accumulate and give the sample consumers time to run.
A dropped IMU interval starts a new generation so navigation averages cannot
bridge the gap. The linker rejects builds with less than 32 KiB of CPU0 stack.
USB serializes into a static response buffer. The host selects compact base64,
hex, or JSON telemetry by capability and refreshes full diagnostics about
once per second. See the [application protocol](docs/application-protocol.md)
for batch limits and wire formats. Housekeeping I2C uses a 50 ms software
deadline and retains the hardware bus timeout.

The PPK stream uses native Unicore OBSVMB at a requested 5 Hz plus GPS, GLONASS,
BeiDou, Galileo, QZSS and NavIC ephemerides on change. Start explicitly requests
one-shot cached ephemerides and restores change subscriptions one constellation
at a time, spaced 800 ms apart. Simply
re-registering ONCHANGED did not replay the cache in the indoor bench test.
Raw bytes are preserved in
SD kind-25 fragments, including receiver header/epoch and CRC; they are retrieved
after recording and reassembled into `.unc` for a compatible RTKLIB converter.
Native SPPNAVB/PPS use COM2 at 460800 baud; raw data and command replies use COM1 at 460800.
This separates the loads after a 921600-baud stage exhibited UART errors.
Raw rate and PPK readiness are independent of the 50 Hz SPP transport rate.
SPPNAVB's 148-byte binary frames replace ASCII position messages. Header and CRC
checks stay integer-only; native floating-point coordinates/speeds are decoded
normally and retained in USB/SD without the navigation adapter's integer rounding.
The SCH16T already uses binary 48-bit SafeSPI frames. Status and SD metadata
identify both physical formats explicitly.
`ppk` reports observed frame counts, observation count, freshness, CRC/transport
losses and SD chunk drops. An empty indoor observation frame is not usable PPK
evidence. At 150 tracked signals, OBSVM occupies about 30,160 bytes/s at 5 Hz;
460800 8N1 provides 46,080 bytes/s, so unusually large signal sets and ephemeris
bursts still require measured loss checks. No outdoor positioning or PPK accuracy
claim follows from an indoor transport test.

The connected receiver's Build17548 previously accepted SG8/50 Hz SPP without a
receiver firmware change; see [the local evidence](../data/um980-50hz-firmware-research.md).
Configuration is volatile except the receiver's self-persisting SIGNALGROUP
setting; firmware never sends SAVECONFIG. MCU recovery attempts to restore COM1
to 115200 for older application images. If GNSS is unreachable at that point,
recovery still works, but an older image may need a receiver power cycle before
its command-path probe succeeds.

| LED | Meaning |
|---|---|
| D32 | GNSS: green fresh fix, amber no fix/stale, blue configuring, red transport fault and for two seconds after a UART error |
| D33 | IMU: green fresh valid samples, amber initialization or a sample error in the last two seconds, red missing/invalid samples |
| D34 | Storage/recording: blue idle, blinking green recording, blinking amber draining, red SD fault/drop |
| D35 | Power: green charging, blue USB bypass, purple battery, red low battery, off unknown |

The LED driver uses the fitted LP5813 scan groups and verifies configuration,
PWM readback and driver fault registers. GNSS no-fix does not turn D32 red.
The indicators use one quarter of their USB PWM duty on battery. Steady colours
are still verified at each battery poll and PWM is written only when it changes.

## Bluetooth LE

The application advertises as **Aevia V2 Mini** and accepts one central.
The firmware pins the 21 October 2026 Espressif Rust SDK revision with the
ESP32-S31 BLE controller. During two sets of 25 Bluetooth reads on the bench,
the IMU ran at 733–735 Hz and GNSS at about 50 Hz. The receiver reported one
IMU error and zero SD queue drops. The 18 November revision missed IMU capture
deadlines with PSRAM active. The controller uses an additional 80 KiB internal
heap. [Pinned S31 driver](https://github.com/esp-rs/esp-hal/tree/b46965413093d7b91a35ddcb5579b4579af9d624/esp-radio/src/ble/btdm2)

On 21 November 2026 a stationary V2 Mini supplied 300 fused BLE reads in
59.74 seconds while recording. Its IMU ran at 727.0 Hz and GNSS at 50.0 Hz.
The 60.08-second SD file contained 43,679 native IMU intervals with continuous
support, 3,004 GNSS records and 299 periodic snapshots plus a terminal summary.
The first SD snapshot preceded the first in-window motion estimate. The
navigation and SD queues reported zero drops. The separate raw PPK/command
port reported 12 UART errors. Thirteen successful charger readings averaged
1.59 W at the USB input during recording, with seven failed power polls.
[Plugged-in power measurements](../data/v2-mini-driving-estimator-power-2026-11-21.csv)

The [subsequent validation record](../data/v2-mini-driving-readiness-2026-11-21.md)
covers the release image, stationary recordings, raw SD losses, power measurements
and moving estimator replays.

The app reads a display snapshot at 5 Hz and can start or stop an SD recording.
A lost Bluetooth connection leaves the receiver recording and available for
reconnection. [GATT characteristics](docs/application-protocol.md#bluetooth-le)

## Build and upload

From this directory, build and upload the application:

```powershell
cargo run --release --locked
```

Firmware targets only `riscv32imafc-unknown-none-elf`.
Requires that Rust target, `uv`, and `espflash`.
The single uploader, `tools/upload.py`, accepts the Cargo ELF or an application
`.bin`. It enters recovery over USB, uploads, and waits for application USB.
Its only Python dependency is pyserial, installed automatically by uv.

```powershell
uv run --script tools/upload.py --recover
uv run --script tools/upload.py target/upload/firmware.bin
```

The loader verifies SHA-256 from flash before booting. Interrupted uploads,
panics, and the ten-second watchdog return to recovery. USB and watchdog
handling run independently of `src/app.rs`.

## Initial recovery installation

Enter ROM mode on J6: hold SW2, tap SW1, release SW2. Build both binaries,
then run the following from the repository root in the S31 ESP-IDF environment.
Replace COM3 with the ROM serial port.

```powershell
cargo build --manifest-path firmware/Cargo.toml --target riscv32imafc-unknown-none-elf --release --bins --locked
idf.py -C firmware/bootloader -B target/current-bootloader '-DIDF_TARGET=esp32s31' bootloader
espflash save-image --chip esp32s31 --flash-size 16mb --flash-mode dio --flash-freq 40mhz --merge --skip-padding --bootloader target/current-bootloader/bootloader/bootloader.bin --partition-table firmware/partitions.csv --target-app-partition recovery target/riscv32imafc-unknown-none-elf/release/recovery target/loader-install.bin
python -m esptool --chip esp32s31 --port COM3 --before no-reset --after watchdog-reset --no-stub write-flash --flash-mode dio --flash-freq 40m --flash-size 16MB 0 target/loader-install.bin
```

Subsequent application uploads use `cargo run` without buttons.
Preserve `target/bootloader-backup-before-flash.bin` if present.

## Checks

From this directory:

```powershell
cargo clippy --release --bins --no-deps --locked -- -D warnings
cargo build --release --bins --no-default-features --locked
cargo build --release --bins --locked
uv run --script tools/upload.py --help
uv run --with pyserial==3.5 python -m unittest discover -s tools
```

From the root, independent sensor-model regression tests run with:

```powershell
rustc --edition=2024 --test firmware/tools/sensor_tests.rs -o target/firmware-sensor-tests.exe
target/firmware-sensor-tests.exe
cargo test --manifest-path firmware/tools/host-tests/Cargo.toml --target x86_64-pc-windows-msvc --target-dir target/firmware-host-tests
```

After uploading, inspect actual sensor health or capture a timed hardware check:

```powershell
uv run --script tools/telemetry.py
uv run --script tools/telemetry.py --seconds 10 --output capture.jsonl
```

The capture tool never overwrites an existing output. Device flashing remains
separate. See [the application protocol](docs/application-protocol.md).

## Layout and protocol

`partitions.csv` reserves state at `0x9000`, recovery at `0x10000..0x90000`,
and the application at `0x90000..0x1000000`. The ESP SDK revision is pinned in Cargo.toml;
S31 flash-capacity and watchdog fixes live in `src/flash.rs` and
`src/watchdog.rs`.

USB IDs are `C0C0:CAFE` for recovery and `C0C0:FEED` for the application.
`src/loader.rs` defines postcard/COBS commands: `Recover`,
`Upload { size: u32, sha256: [u8; 32] }`, and `Data(&[u8])`.
Replies are `Ready`, `Reboot`, or `Error`. Send one command per reply,
with 256-byte data blocks and a final short block. Frames end in zero and
partial requests time out after 30 seconds.
