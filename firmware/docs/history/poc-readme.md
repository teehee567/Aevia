> Historical snapshot before the base-firmware rebuild. Commands and source paths describe the former image.

# Trajectory speed proof of concept

See the [system and component reference](../v2-mini-system-reference.md) for
current firmware behavior, wiring, register settings, and operating policies.

The default `host-poc` image captures UM980 GNSS and SCH16T-K01 IMU data on
the connected Mini board and streams them over USB. Core 1 captures the
IMU; core 0 handles GNSS and USB. This is a development experiment, with
approximate timing and no board calibration. The host capture, live speed
display, and build/flash helper tools have been removed after bring-up.

The selected receiver profile requests **20 Hz standalone GNSS**, restoring
signal group 1. The 50 Hz experiment produced 50 messages/s but only one
valid standalone solution/s on this receiver; both position and velocity
were invalid between those fixes. There is no RTK correction source in this
test. `SINGLE` therefore means standalone positioning, not an RTK fix.

The IMU uses 1 MHz mode-0 SPI and factory-corrected 20-bit output. DEC5
provides a nominal 737.5 Hz native rate; averaging four consecutive samples
produces nominally 184.4 estimator intervals/s. The live test measured about
182 accepted intervals/s with 20 valid GNSS fixes/s. Profile details and source
documents are in [trajectory-poc.md](../trajectory-poc.md).

## Build

SW7 now powers off the board with a single click while the application is
running. The MAX16169 debounces the press and signals GPIO38; a separate
firmware task pulls GPIO8 (`PWR_KILL_N`) low to switch off the main 3.3 V rail,
including when powered over USB. The startup press is ignored for 100 ms so
turning the board on does not immediately turn it off. Holding SW7 for eight
seconds remains the hardware fallback if firmware is stalled or in ROM mode.
The current POC does not write onboard storage; add a storage flush before
asserting `PWR_KILL_N` when recording is introduced.

Run firmware builds from this directory so its target configuration applies:

```sh
cd firmware
cargo build --release
```

The ELF is written to
`../target/riscv32imafc-unknown-none-elf/release/aevia-firmware`.

`cargo build --release --features rtk-50hz` selects the optional group-8
50 Hz profile for tests with an RTK correction feed. The default build
restores the group-1 standalone profile on startup.

## Validation

The LCD driver writes an ST7789 bring-up pattern: six vertical color bars, a
white border, and a moving white marker along the bottom. SPI3 drives the panel
at 1 MHz independently of the IMU's SPI2. Bring-up is incomplete: the first
backlight-enabled image repeatedly restarted after its first frame. Keeping
the backlight off restored stable operation. The current image enables the
nominal 10% backlight setting after initialization and holds it on, as requested.
It stayed at `lcd-light=10` for a 15-second live check without a reset. Commands
`LCDLIGHT 1` and `LCDLIGHT 5` run five-second tests; `LCDLIGHT 10` stays on and
`LCDLIGHT 0` switches it off (terminate commands with newline). These three tests
stayed stable, but neither 4 MHz nor 1 MHz SPI produced valid panel readback or
TE pulses. After correcting the ribbon contact orientation, the LCD responds
with ID `8181b3`, but GPIO19/LCD_DC senses low even when its output latch is
high (`lcd-dc=4`, healthy value 5). The current firmware reports
`lcd=dc-stuck-low`, holds DC low and stops display writes, retaining 10%
backlight. A fresh retest with the LCD disconnected still reports `lcd-dc=4`,
pointing to the board-side circuit. With power removed, inspect and measure
J3 pin 10 and the GPIO19 net for a short to ground (J3 pin 11 is grounded).
The startup SCLK probe passed with the panel disconnected (`lcd-sclk=26`):
GPIO16 follows both weak pulls and both output levels. This argues against
a hard short to ground on SCLK. This probe repeats on reboot, not LCDREINIT.
`LCDREINIT` followed by newline
rechecks the pin and reinitializes the LCD if the pin passes.
USB STATUS includes `lcd`, `lcd-te`, `lcd-frames`, `lcd-light`, `lcd-id`,
`lcd-regs` and the chip reset reason;
`lcd=scanning` means the panel is returning TE frame pulses, not that the
visible image has been optically verified. See the system reference for wiring.

With the default firmware running on Windows, check live LCD feedback with
`powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-lcd.ps1`
from the repository root (default COM5; override with `-Port`). The check
requires fresh USB status, advancing TE edges and completed frame writes;
visually confirm the pattern separately.

D32 now shows live battery status from [battery.rs](../../src/battery.rs):

| Colour | Meaning |
| --- | --- |
| Green | Charging, including when the battery is low |
| Blue | External power without confirmed charging (including a missing gauge/no cell) |
| Purple | Running on battery |
| Red | Running on battery at or below 15%; clears at 20% |
| Off | Starting, confirming a transition, or unable to establish valid status |

The monitor detects BQ25628E at `0x6A` or BQ25622E at `0x6B` and polls
the charger and MAX17048 about every 500 ms,
cross-checks charger power-good on GPIO40, validates gauge readings, and
waits at least one second for the gauge to settle. Two consecutive samples
confirm a colour. Failed readings immediately invalidate the corresponding
telemetry; USB reports samples older than three seconds as stale. Low-battery
hysteresis prevents flicker around the threshold. LED readback failures
disable D32 and retry after three seconds while battery polling continues.
D33-D35 remain off, and monitoring works independently of USB and GNSS.

USB `STATUS` and the onboard variant's `TRAJ` reports include `battery`,
`battery-pct`, `battery-mv` when valid, charger identity/registers, and
charger/gauge errors. `POWER` remains the saved startup diagnostic.
The battery module only reads: autonomous charging settings stay as documented
in the system reference. It does not implement battery protection or automatic
shutdown. Blue is a power-path indication, not a measurement of zero battery
current; a weak USB source can still need battery assistance. Stable gauge
readings reduce false detection but cannot prove a cell is physically present.

Run the battery policy tests from the repository root on Windows:

```powershell
rustc +1.95.0 --edition 2024 --test firmware/src/battery.rs -o target/battery-tests.exe
.\target\battery-tests.exe
```

The tests cover power transitions, all charge phases, fractional SOC thresholds,
missing/reconnecting gauges, voltage cycling, faults, stale data and recovery.
They do not replace checking colours and power transitions on the board.

The 2026-09-10 flash/live check identified BQ25622E (`0x1A`) at `0x6B`,
contrary to the earlier fitted-part note. Explicit two-byte gauge reads fixed
a burst-read issue that repeated VCELL in place of SOC. Charging and green LED
readback were verified with zero battery I2C errors. The connected gauge still
reported roughly 120% raw SOC, so `battery-pct` remains unknown and
`battery-gauge=soc-out-of-range`; valid voltage and charging status remain usable.
A voltage step briefly caused settling/off. Percentage accuracy and battery-only
transitions still need hardware validation. Ten host battery tests pass.
Repeat the live check with `powershell -NoProfile -ExecutionPolicy Bypass -File
tools/check-battery.ps1` from the repository root.

The following results were recorded before the host tools were removed.

The completed 45-second live capture after 10 seconds of warmup passed all
45 windows, with 20 valid GNSS fixes/s and about 182 accepted IMU intervals/s.
Fusion advanced with zero estimator resets, rejected IMU/GNSS observations
or wire parsing errors. All 440 trajectory tests and seven host tests also
passed, including predictor feedback and hard-reset regressions. This
validates the live data path; speed accuracy has not been checked against
an independent reference. Replay should use a fresh, continuous capture;
accuracy after long gaps or concatenated old sessions is unqualified.

## Onboard estimator experiment

`cargo build --release --no-default-features` runs the estimator on core 1
using the fitted 16 MiB PSRAM at 125 MHz. The startup fault was fixed on
2026-09-10: core 1 inherited a ROM memory-attribute region that permitted
reads but rejected PSRAM writes. [psram.rs](../../src/memory.rs) splits that region,
preserves flash read-only access, and grants read/write access only to PSRAM.
It checks the expected ROM layout and reads back the new attributes.

Before capturing measurements, startup tests all 16 MiB with two complementary
address-dependent patterns, writing back and invalidating the cache before
verification. `psram-tested=16MiB start-stage=8` in USB telemetry indicates
that the test and trajectory workspace initialization completed. Run the
hardware check from the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-psram.ps1
```

The check also requires advancing estimator IMU ingestion. Host tests for
the memory-region split and configuration checks run with:

```powershell
rustc +1.95.0 --edition 2024 --test firmware/src/psram.rs -o target/psram-tests.exe
.\target\psram-tests.exe
```

The board passed the full-memory test and started the onboard estimator.
GNSS had no valid fix during this check, so onboard fusion and speed output
remain unverified. The default build remains the USB-streaming `host-poc`.
