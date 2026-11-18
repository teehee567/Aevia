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
the ST7789 shows speed/fix/recording status and button 1 toggles recording.

Sessions persist independently on an existing exFAT microSD volume and can also
be captured by the USB host through the Aevia app. CPU1 owns native four-bit SD
and all filesystem I/O alongside the bounded navigation worker. Separate queues
hold 384 SD sensor samples and 32 raw GNSS chunks and report their own overflows.
USB starts with an internal 128-sample queue and promotes to a 4,096-sample queue
in an exclusively reserved PSRAM tail after its startup test and CPU0 protection
checks. `usb_queue` reports actual backing, capacity and any fallback error.
Recording files reserve up to 128 MiB contiguous extents, rotate when full, and retain CRC-protected committed
pages after interrupted recording. Start opens the SD extent and rearms navigation
before setting the recording boundary; a failure within ten seconds returns an
error without starting later. Stop drains the SD queue and flushes a final
closed page. The recorder never formats the card or deletes existing files;
FAT32/GPT volumes are currently unsupported, and dirty exFAT volumes permit only
retrieval. Bluetooth is not enabled. The upstream radio SDK has S31 support, but
this build does not include or claim a tested Bluetooth transport.

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

CPU1 runs the actual Traj live navigation engine with an explicitly unqualified
development profile. Hot estimator state remains in internal SRAM; cold state
and metric storage use the first 15 MiB of fitted PSRAM; the final MiB belongs to
CPU0's USB queue. Startup tests all 16 MiB before either owner uses it.
See the [hardware validation record](../data/highrate-ppk-validation.md)
for measured sample rates, USB and SD comparisons, and the validated firmware
images. The [20 Hz baseline](../data/aevia-implementation-status.md) records
the earlier firmware and app checks.

The current profile supports fused navigation diagnostics; on-device horizontal drag/lap
metrics remain unavailable because their root qualification evidence is absent.
The application provides its own explicitly degraded measurement analysis.
Production `--no-default-features` builds retain strict preflight and reject this
unqualified profile. No calibration or accuracy qualification is implied.

Installation inputs are in `src/navigation/settings.rs` and exposed in status:
sensor-to-body scalar-first quaternion, IMU-to-antenna lever arm, and nonzero
timing uncertainty. Identity mounting and zero lever arm are bench assumptions,
with nonzero uncertainty in the profile. Replace them with measured installation
values before interpreting vehicle-axis acceleration or precision results.
A new recording rearms navigation. Its first valid fix selects PPS timing if
available, otherwise arrival-only development timing; it never silently switches
latency models mid-session. Lost PPS mapping suspends GNSS fusion, while inertial
propagation continues; start a new session to select a recovered clock generation.

The engine preflight declares 163,824 bytes of hot SRAM, a 32 KiB stack contract,
and 2,107,760 bytes of PSRAM. Firmware reserves a physical 64 KiB worker stack
because historical hardware preflight exceeded 32 KiB. This allocation is not a
measured high-water bound. This RTOS does not save floating-point task context:
CPU1 has one floating-point worker and integer-only idle/storage/interrupt paths.
CPU0 acquires and validates native IMU intervals in a priority-2 interrupt
executor, using 8 MHz SafeSPI and a table implementation of the same CRC8.
GNSS UART/configuration, integer SPP/PPS decoding and bounded raw-byte copies run
at priority 1; GPIO wakes IMU
acquisition at priority 3. These paths are integer-only: the SDK interrupt entry
does not preserve floating-point registers. Bounded channels hold 32 IMU
intervals, four parsed SPP observations, and 8,192 packed COM1 bytes including
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
one-shot cached ephemerides before restoring change subscriptions; simply
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
and the application at `0x90000..0x1000000`. SDK crates come from crates.io;
S31 flash-capacity and watchdog fixes live in `src/flash.rs` and
`src/watchdog.rs`.

USB IDs are `C0C0:CAFE` for recovery and `C0C0:FEED` for the application.
`src/loader.rs` defines postcard/COBS commands: `Recover`,
`Upload { size: u32, sha256: [u8; 32] }`, and `Data(&[u8])`.
Replies are `Ready`, `Reboot`, or `Error`. Send one command per reply,
with 256-byte data blocks and a final short block. Frames end in zero and
partial requests time out after 30 seconds.
