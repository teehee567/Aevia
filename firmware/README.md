# V2 Mini base firmware

The default image brings up the ESP32-S31 V2 Mini's sensors, power bus,
buttons, display interface, PSRAM and USB console. Hardware ownership,
device protocols, background tasks and USB presentation are separate.
There is no trajectory engine or allocator in the base image.

## Build and flash

Use Rust **1.95.0**, target `riscv32imafc-unknown-none-elf`, and
**espflash 4.5.0**. The HAL revision and dependency graphs are pinned.
From `firmware/`:

```powershell
cargo build --release --locked
cargo run --release --locked
```

`cargo run` uses the Windows Cargo runner to flash over **J6**, without
BOOT/RESET button presses. From the repository root the equivalent is:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/flash-firmware.ps1
```

The script builds firmware and the host helper before touching the board.
The helper discovers the application (`303A:4001`), sends `BOOTLOADER`, waits
for ROM USB (`303A:0020`), flashes through the RAM stub, performs the S31
watchdog reset, then requires two advancing base-firmware status records.
It also accepts a board already in ROM download mode. The standalone helper
source lives in [tools/s31-reset](../tools/s31-reset); its original reset-only
command is retained.

Port numbers are discovered, not assumed. Optional `-AppPort COM5 -RomPort
COM4` arguments select known ports. Close other serial readers first and
connect only the board being flashed. J5 Serial/JTAG is a different transport.
Software entry requires responsive application USB; if both application and
ROM USB are absent, physical recovery is still necessary. A failed flash
leaves the board in download mode so the same command can be retried.

The ELF is `target/riscv32imafc-unknown-none-elf/release/aevia-firmware`
relative to the repository root.

## Source layout

| Location | Responsibility |
| --- | --- |
| `src/main.rs` | Entry point and image descriptor |
| `src/board.rs` | Frozen board pins, bus configuration, safe initial output levels |
| `src/app.rs` | Task composition and core ownership |
| `src/drivers/` | SCH16T, UM980, ST7789, LP5813 and TCA9536 I/O |
| `src/tasks/` | Sensor recovery, shared power-bus scheduling, shutdown and display lifecycle |
| `src/usb/` | USB descriptors, command dispatch, bounded writes and telemetry formatting |
| `src/state.rs` | Short snapshot locks and bounded capture queue |
| `src/lib.rs` | Host-testable command framing, GNSS/SafeSPI protocols, battery/button policies and memory checks |
| `experiments/` | Archived reference source; excluded from every base build |

See [system reference](docs/v2-mini-system-reference.md) for wiring, timing,
failure handling and extension constraints.

## USB console

Open the application COM port at 115200 with DTR enabled. USB sends a `STATUS`
line about once per second; sensor acquisition runs even with no host reader.
Status includes a protocol version, monotonic uptime, sensor counts, independent
GNSS fix validity, battery/LED/button state, LCD diagnostics, PSRAM test results
and cumulative capture/drop counters. `last-*-error` retains historical faults;
the current state and advancing error counters distinguish recovery from a
continuing fault. Battery readings expire after three seconds.

Commands are ASCII lines terminated with LF or CRLF. They work across USB
packet boundaries. Oversized or malformed lines are discarded; a bootloader
token embedded in another line cannot trigger a reset.

| Command | Result |
| --- | --- |
| `BOOTLOADER` | Immediately enter ROM download; independent of the telemetry writer |
| `STREAM 0123abcd` | Clear pending capture and reply `STREAM READY 0123abcd <time_us>`, then emit RAW records |
| `STOP` | Clear capture queue and acknowledge `STREAM STOPPED`; sensors/status keep running |
| `LCDREINIT` | Repeat panel initialization and DC check; SCLK probe requires reboot |
| `LCDTEST` | Enable the animated panel test pattern |
| `LCDLIGHT 0`, `1`, `5`, `10` | Backlight ceiling; 1/5 expire after five seconds, 10 stays on while panel initialization permits it |
| `GNSSBRIDGE` | Pause normal GNSS acquisition/configuration and bridge UM980 COM2 on the second USB serial interface |
| `GNSSNORMAL` | Restore COM2 baud and resume normal GNSS configuration/acquisition |

J6 now exposes two serial ports. Interface 0 is the console (currently COM5);
interface 2 is raw GNSS (currently COM6). The flash helper selects interface 0.
Bridge mode starts COM2 at 460800, 8N1 and follows the GNSS port's host baud
selection. It stays enabled across host disconnects until GNSSNORMAL or reboot.
Firmware telemetry and command parsing stay on the console port.

**Connected-board limitation:** receiver-to-host USB streaming is working;
direct receiver commands have not passed. GPIO49, assigned as COM2 RXD in the
schematic, was observed receiving COM2 output when released as an input.
COM1 replies on GPIO46 were absent. A wiring/assembly fault is suspected, not
confirmed. See [bridge findings and checks](docs/gnss-usb-bridge.md) before
treating this as a working two-way UPrecise connection.

Streaming is opt-in. Its nonce is exactly eight hexadecimal digits. A consumer
must discard bytes before its matching READY line. RAW formats are:

```text
RAW IMU <end_us> <interval_us> <ax> <ay> <az> <gx> <gy> <gz>
RAW GNSS <received_us> <original BESTNAVA line>
```

Acceleration is m/s², angular rate rad/s, timestamps MCU monotonic microseconds.
Frame lines across packets. The fixed 32-record queue drops new records on
overflow and increments `queue-drops`; producers never wait for USB. A blocked
writer has a 250 ms deadline and disables capture. Send a new STREAM request
to resume. This is a bring-up console, not a lossless recording transport.

## Verification

From the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-firmware-code.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-firmware.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-battery.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-psram.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-lcd.ps1
```

The code check runs host tests, strict Clippy for host and device, formatting
and the release build. The base hardware check requires a fresh stream
handshake, progressing GNSS/IMU counts, no new capture errors, the full 16 MiB
memory check and continued acquisition after STOP. Allow roughly 12 seconds
after power-on for receiver configuration before running it. Hardware checks
default to COM5 and accept `-Port`. A valid satellite fix and a working panel
are deliberately separate bring-up results.

Live verification on 2026-09-10 passed USB flashing and automatic restart from
both the former image and the rebuilt base image. After making power I2C
asynchronous, a 15-second check received 2,726 IMU intervals and 300 GNSS
records with zero capture errors and zero queue drops. The full 16 MiB PSRAM
test passed; the charger/gauge and green LED readback were valid; buttons
reported the unpressed mask. STOP preserved acquisition and status.

The connected LCD still reports `dc-stuck-low` (`lcd-dc=4`, expected 5).
Firmware leaves its backlight off and does not write frames into that fault.
Visible display output, physical button transitions, battery-only transitions,
SW7 shutdown and GNSS fixes need separate hardware checks. There was no
satellite fix during this run. GNSS timing is not PPS-disciplined and board
calibration is not implemented.

SD recording, wireless, lap timing and onboard fusion are future application
work. The old `host-poc`, `rtk-50hz`, and implicit onboard-estimator build modes
have been removed. Historical results remain in [trajectory notes](docs/trajectory-poc.md)
and [the preserved bring-up reference](docs/history/poc-system-reference.md).
