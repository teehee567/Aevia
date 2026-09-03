# V2 Mini minimal bring-up

This is deliberately a first-board firmware, not the product firmware. It proves
that the ESP32-S31 executes code, preserves the power latch, samples the board
status pins, and can communicate with the power-I2C, SCH16T SPI, and UM980 UART
devices.

The charger probe is read-only. The fitted BQ25622E starts autonomously in hardware
because `CE` is grounded, so this firmware does not make charging safe for an
unknown cell. Do not fit a battery until the cell chemistry, 4.2 V termination,
allowable charge current, connector polarity, and the board's NTC curve have
been checked.

## What it touches

- GPIO8 `PWR_KILL_N`: input with pull-up, never driven low.
- GPIO39 `LED_EN`: held low, raised for 2 ms only while reading the LP5813 reset
  registers, then returned low. `chip_en` is never written.
- GPIO4 `LCD_BACK`: held low for the entire bring-up.
- GPIO6/GPIO7: 100 kHz I2C, probing only the documented addresses. There is no
  address scan.
- BQ25622/BQ25622E at `0x6B`: identity, selected configuration, status, and
  fault reads.
- GPIO9..GPIO14: a reset pulse followed by a read-only SCH16T component-ID
  request over 1 MHz, mode-0 SafeSPI. A nonblank response must have a valid CRC.
- GPIO44/GPIO46..GPIO49: UM980 reset level and both UARTs at 115200 baud. The
  probe sends `VERSIONA` and accepts an identity reply or unsolicited NMEA as
  proof of connection; an antenna is not required.
- MAX17048 at `0x36`: version, cell voltage, SOC, and status reads. It is powered
  from `BAT`; the charger's no-battery behavior can make it ACK or NACK as BAT
  cycles, and its SOC is meaningless without a cell.
- TCA9536A at `0x40`: input and configuration reads. The fitted
  `TCA9536ADTMR` is the A-address variant.
- LP5813A page 0 at `0x50`: read-only reset-state check. This part encodes
  register bits 9:8 in I2C addresses `0x50..0x53`.

MAX16169 and TPS63802 have no register buses. Their first-pass coverage is the
released/high `PWR_KILL_N` input and sampled `PWR_INT_N` line.

## Build and flash

The firmware pins the first official esp-hal revision that marks ESP32-S31 GPIO,
UART, and I2C master support as available:
`esp-hal-v1.2.0-rc.0` / `160b10794227eb84805b8676fe188c1110801e9d`.

Run Cargo from this directory so the embedded target configuration does not
affect the host crates elsewhere in the workspace:

```powershell
cd firmware
rustup show
cargo build --release
cargo run --release
```

The pinned toolchain installs the `riscv32imafc-unknown-none-elf` target. The
runner expects `espflash 4.5.0` or newer; 4.5.0 is the first espflash release
with ESP32-S31 support. `cargo run --release` builds, requests ROM download mode
over J6, flashes, waits for the application J6 CDC port, and prints the repeating
peripheral report. Stop the monitor with Ctrl+C.

The first installation of this image needs one manual download-mode entry: hold
SW2 (`GPIO61`), tap SW1, press SW7 to restore the latched rail, wait two seconds,
then release SW2. GPIO60 must remain high; the frozen board leaves it at its
internal pull-up. After that installation, the runner sends `BOOTLOADER` to the
application, which sets the S31 one-shot force-download bit and software-resets.
No buttons are needed for subsequent reflashes.

## Where the log appears

The application implements CDC-ACM on the dedicated high-speed USB controller
wired to USB-C J6. It enumerates as `AEVIA V2 Mini Bring-up Console` with USB
VID/PID `303A:4001`; ROM download mode enumerates separately as `303A:0020`.

J5 pins 3/4/5 remain available as USB Serial/JTAG D-/D+/GND on GPIO33/GPIO34,
and UART0 TX remains available at TP2 through 499 ohms, but neither is required
for the normal J6 workflow.

The full report repeats every five seconds so a monitor can be attached after
reset.

## First power, without a battery

1. Inspect polarity and shorts, and use a current-limited 5 V source. Treat J4
   BAT as live even when empty: the charger can raise it to roughly 4.2 V while
   testing for a battery. Press SW7 for at least 50 ms to latch the TPS63802 on.
2. Verify the 3V3 rail and module EN rise before flashing. The frozen board uses
   10 kΩ/100 nF on EN; Espressif's module guidance shows 10 kΩ/1 µF, so confirm
   reset behavior on the scope rather than assuming it.
3. Run `cargo run --release`; after the one-time initial manual download entry,
   subsequent runs reflash and reconnect without board-button input.
4. Expect the BQ25622E, released-button TCA9536A, and LP5813A to report `PASS`.
   MAX17048 may ACK if BAT is energized or NACK while BAT droops; measure BAT
   and accept either result during the no-cell test.

Typical identity/default values on a fresh board are:

```text
Hello, world!
AEVIA V2 Mini peripheral bring-up v0.1.0
[PASS] BQ25622E @ 0x6B part=0x1A pn=3 rev=2 ...
       cfg (read-only): ICHG=0340 VREG=0D20 VSYSMIN=0B00 CTRL0=06 CTRL3=04 ...
[PASS] TCA9536A @ 0x40 inputs=FF config=FF ...
[PASS] LP5813A @ 0x50 reset regs=00/00/00/E4
```

The MAX17048 line may be either `PASS` with a `0x001x` version or `NACK` without
a cell. Status values vary with VBUS, BAT, thermistor, and button state. A
responding charger is not authorization to attach a battery. Validate the
fitted charger's autonomous limits and thermistor network against the actual
cell first.

The design/register research behind these choices is in
[`docs/v2-mini-bringup-research.md`](docs/v2-mini-bringup-research.md).
