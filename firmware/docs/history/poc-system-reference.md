> Historical hardware evidence and former firmware behavior. See [the current reference](../v2-mini-system-reference.md) for current ownership and commands.

# V2 Mini system and component reference

Checked against the firmware source on 2026-09-10. This document describes
current behavior, component wiring, register conventions, and the policies
that future drivers must preserve. Register reset values below are reference
values, not a claim that firmware programs or continuously verifies them.

Wiring and fitted-part notes are retained from the
[PCB](../../../hardware/v2_mini_pcb/v2_mini.kicad_pcb),
[production BOM](../../../hardware/v2_mini_pcb/production/digikey_bom.csv), and
[design note](../../../v2_mini_design.md). The fitted charger differs from the
BOM in the earlier notes. The 2026-09-10 connected board instead returned
BQ25622E identity `0x1A` at `0x6B`. The battery monitor supports both identities
and shares only status fields verified in both datasheets. Manufacturer references
are kept beside each component so register details can be traced directly.

## Current system and ownership

The default `host-poc` image captures GNSS and IMU measurements and streams
ASCII records over J6 USB. It does not compute onboard speed or write onboard
storage. The former host and flashing tools have been removed.

| Owner | Current responsibility | Implementation |
| --- | --- | --- |
| Startup on core 0 | GPIO setup, one power snapshot, UART/SPI/USB setup | [main.rs](../../src/main.rs) |
| Power diagnostic | Bounded read-only startup snapshot, then hand I2C0 to the battery task | [power_diagnostic.rs](../../experiments/power_diagnostic.rs) |
| Core 0 battery task | Poll charger/gauge and PG, publish live battery status, drive D32 on the same bus | [battery.rs](../../src/battery.rs), [led.rs](../../src/drivers/lp5813.rs) |
| Core 0 GNSS task | Configure UM980, parse BESTNAVA, enqueue raw records | [gnss_profile.rs](../../src/drivers/um980.rs), [gps.rs](../../src/gnss.rs) |
| Core 1 IMU task | Initialize SCH16T, validate and average native samples; 16 KiB stack | [imu.rs](../../src/imu.rs), [main.rs](../../src/main.rs) |
| Core 0 USB tasks | Measurement/status output, stream handshake, bootloader command | [main.rs](../../src/main.rs) |
| Independent core 0 shutdown task | SW7 shutdown | [main.rs](../../src/main.rs) |
| Independent core 0 LCD task | ST7789 color bars, animated marker and TE feedback | [lcd.rs](../../src/drivers/st7789.rs) |

The measurement channel holds 128 entries. Producers use `try_send`; a full
queue drops the new measurement and increments `queue-drops`. A blocked USB
reader therefore does not block capture. LED and shutdown tasks do not depend
on a USB reader. `capture-errors` counts sensor acquisition/parsing errors.

Building with `--no-default-features` moves acquisition to core 0 and runs
the estimator on core 1 with a 64 KiB stack and PSRAM workspace. The former
startup fault was a core-1 ROM memory-attribute region rejecting PSRAM stores.
[psram.rs](../../src/memory.rs) grants the fitted 16 MiB read/write access on the
estimator core while preserving read-only flash and unused address ranges.
It reserves PMA8..11, checks their initial state and verifies the changes.
This is specific to the observed S31 rev 0.0 ROM layout and pinned HAL;
different memory sizes/protection layouts are rejected. It does not configure
PSRAM permissions on core 0.

A destructive full-memory test runs at 125 MHz before workspace construction
and sensor capture. Two complementary address-dependent patterns are flushed
from cache and read back. Failure is reported as `psram-error`; success is
`psram-tested=16MiB`, followed by `start-stage=8` once the live session starts.
USB, battery and power-button tasks remain active during initialization.
Onboard IMU ingestion has been demonstrated; GNSS fusion and onboard speed
still need validation with a valid fix. See [trajectory experiment](../trajectory-poc.md).

## Operating policies

- Keep `PWR_KILL_N` released high using an open-drain output until SW7 requests
  shutdown. Ignore the startup press for 100 ms, wait for INT to be released,
  then shut down on its next falling edge. Keep CLR low until power is cut.
- Start with the LCD backlight off, then hold the requested 10% setting after
  panel initialization. The 1% and 5% diagnostic settings expire after five
  seconds; `LCDLIGHT 0` switches it off. Full-duty drive remains unqualified.
  D32 indicates battery status: green while charging, blue on external power
  without confirmed charging, purple on battery, red on battery at or below
  15% (clears at 20%). Unknown/transition states are off.
- Do not write charger settings, enable its ADC, clear its event flags, reset
  the fuel gauge, or issue QuickStart. The diagnostic and battery monitor only read.
  A future charger writer must explicitly own watchdog service or disable it.
- Use known 7-bit I2C addresses and device-specific byte order. No generic scan.
- Treat a missing gauge with no battery as an allowed power-state outcome.
  Its BAT supply can cycle; SOC without a cell is not meaningful.
- Charging is autonomous with CE grounded. Cell limits and the fitted
  thermistor curve remain prerequisites for choosing a charging policy.
- Reject invalid sensor data and expose drops/errors. Preserve GNSS solution
  validity separately from communication status and message rate.
- The current shutdown path has no storage to flush. Introduce a bounded
  storage flush before asserting CLR if recording is added; do not make
  shutdown wait for a USB reader.

## Fitted IC inventory

| Ref | Production part | Role | Firmware-visible interface |
| --- | --- | --- | --- |
| U1 | UM980 | GNSS/RTK receiver | Two UARTs, reset, PPS/status GPIO |
| U2 | ESP32-S31-WROOM-3-N16R16V | MCU module | RISC-V MCU; 16 MB flash and 16 MB PSRAM option |
| U3 | SCH16T-K01-1 | IMU | SPI, reset, data-ready |
| U4 | LP5813ADRRR | Four-channel RGB LED matrix driver | Power I2C plus `LED_EN` |
| U5 | TCA9536ADTMR | Four-input button expander | Power I2C |
| U6 | TPD4EUSB30DQAR | Four-channel USB/CC ESD protection | No registers |
| U7 | MAX16169AALTA+T | Pushbutton on/off controller | Power-latch GPIOs; no registers |
| U8 | BQ25628ERYKR fitted (`BQ628E` marking); schematic/BOM specifies BQ25622ERYKR | Buck battery charger and power path | Power I2C plus status/interrupt pins |
| U9 | MAX17048G+T10 | Single-cell fuel gauge | Power I2C plus alert |
| U10 | TPS63802DLAR | 3.3 V buck-boost converter | Hardware-configured; no registers |

The MCU/module interpretation is from the official preliminary
[ESP32-S31-WROOM-3 datasheet](https://documentation.espressif.com/esp32-s31-wroom-3_datasheet_en.html).
U1 and U3 are active acquisition peripherals in the default firmware.

## Power tree and reset behavior

```text
USB-C J6 VBUS ----> U8 BQ25628E VBUS
                         | BAT ----> J4 battery + ----> U9 MAX17048 CELL/VDD
                         | SYS ----> 3V3_RAW ----> U7 MAX16169 VCC
                                                \-> U10 TPS63802 VIN
U7 OUT / GATE_EN -------------------------------> U10 EN
U10 VOUT ----------------------------------------> +3V3
                                                   |-> MCU/UI/SD/display
                                                   |-> FB1 -> +3V3_GNSS
                                                   \-> FB2 -> +3V3_IMU
```

U10 is wired exactly as the 3.3 V fixed-divider example: `R25 = 511 kOhm`
from VOUT to FB and `R26 = 91 kOhm` from FB to ground, with a 0.47 uH
inductor. MODE is grounded, selecting automatic PFM/PWM; PG is not connected.
EN is driven by U7 and therefore is not left floating. See the official
[TPS63802 datasheet](https://www.ti.com/lit/ds/symlink/tps63802.pdf) and
[product page](https://www.ti.com/product/TPS63802).

U7 controls the main +3V3 rail even with USB connected. It starts with OUT
deasserted when VCC is first applied. SW7 must satisfy the 50 ms debounce
interval to turn the board on. The AALT option has 8 s long-press shutdown,
a 32 ms normal interrupt pulse, and a 128 ms long-press interrupt pulse.
Long-press shutdown remains available when application firmware is stalled
or the MCU is in ROM download mode. See the
[MAX16169 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max16169.pdf)
and [product page](https://www.analog.com/en/products/max16169.html).

The board fits `R8 = 10 kOhm` and `C11 = 100 nF` on MCU EN. Espressif's module
reference circuit recommends 10 kOhm and 1 uF, and specifies at least 1 ms for
both rail stabilization and the reset-low pulse. The electrical reset-ramp margin is not established by the firmware; retain
this component-value difference when investigating startup/reset failures.

## MCU management and programming pins

The frozen PCB maps the management and programming signals as follows. Numbers
are module pad numbers, followed by the ESP32-S31 GPIO/function.

| Module pad | MCU signal | Frozen-board net and connection | Current use |
| --- | --- | --- | --- |
| 3, 4 | 3V3 | +3V3 | Power |
| 5 | EN | 10 kOhm pull-up, 100 nF to GND, SW1 to GND | Reset input |
| 6 | GPIO2 | `BAT_ALRT_N`, MAX17048 ALERT, 10 kOhm pull-up | Input |
| 8 | GPIO0 | `CHG_INT_N`, BQ25628E INT, 10 kOhm pull-up | Input |
| 9 | GPIO1 | `CHG_STAT_N`, BQ25628E STAT, 10 kOhm pull-up | Input |
| 12 | GPIO6 | `POW_SCL`, 5.1 kOhm pull-up | I2C0 SCL |
| 13 | GPIO7 | `POW_SDA`, 5.1 kOhm pull-up | I2C0 SDA |
| 14 | GPIO8 | `PWR_KILL_N`, MAX16169 CLR, 10 kOhm pull-up | Open-drain output, released high; low for shutdown |
| 40 | dedicated USB_DP | J6 through 22 Ohm | USB-HS/OTG connector |
| 41 | dedicated USB_DM | J6 through 22 Ohm | USB-HS/OTG connector |
| 42 | GPIO33 / USB Serial/JTAG D- | J5 through 22 Ohm | Native debug USB |
| 43 | GPIO34 / USB Serial/JTAG D+ | J5 through 22 Ohm | Native debug USB |
| 44 | GPIO35 | `USB_VBUS_S`; 75 kOhm/100 kOhm VBUS divider | Input; 5 V gives about 2.86 V |
| 49 | GPIO38 | `PWR_INT_N`, MAX16169 INT, 10 kOhm pull-up | Input |
| 50 | GPIO39 | `LED_EN`, LP5813 EN | Output low initially |
| 51 | GPIO40 | `CHG_PG_N`, BQ25628E PG, 10 kOhm pull-up | Input |
| 68 | GPIO58 / UART0_TXD | TP2 through 499 Ohm | Auxiliary UART log output |
| 69 | GPIO59 / UART0_RXD | TP1 | Console input |
| 71 | GPIO61 | BOOT/SW2 to GND, 10 kOhm pull-up | Boot strap |

`GPIO4 / LCD_BACK` starts low, then continuously pulses at the nominal 10% setting.
`GPIO9 / IMU_RESET_N` is driven high except
during IMU initialization; `GPIO44 / GNSS_RESETN` is an input with pull-up.
Both reset nets also have board pull-ups. The battery task polls charger PG
on GPIO40; the other charger/gauge status pins and VBUS sensing remain unused.
J5 is the native USB Serial/JTAG port and J6 is the separate dedicated USB-HS/OTG port; they are not alternate
connectors for the same peripheral.

Normal SPI boot uses GPIO61 high. Joint Download mode uses GPIO61 low
and GPIO60 high: hold SW2, pulse SW1, restore the latch with SW7 if needed,
then release SW2. GPIO61 low with GPIO60 low is invalid. Software entry from
the application is described under USB below.

The original references differed on strapping pins: the preliminary
[module](https://documentation.espressif.com/esp32-s31-wroom-3_datasheet_en.html)
and [series datasheets](https://documentation.espressif.com/esp32-s31_datasheet_en.html)
listed GPIO36/37/60/61, while the
[GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/api-reference/peripherals/gpio.html)
listed GPIO36–40 and GPIO60–61. Preserve that discrepancy for reset-time
changes; application LED/shutdown operation does not resolve strap sampling.
The [boot-mode reference](https://docs.espressif.com/projects/esptool/en/latest/esp32s31/advanced-topics/boot-mode-selection.html)
and [serial connection reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/get-started/establish-serial-connection.html)
provide the underlying programming interface details.

## Power I2C bus

The bus is `GPIO6 / POW_SCL` and `GPIO7 / POW_SDA`, with 5.1 kOhm pull-ups to
+3V3. The startup diagnostic configures 100 kHz, a 100-bus-cycle hardware
SCL timeout, and a 10 ms software transaction timeout. The battery task takes
ownership of the same bus after that snapshot and calls the LED driver;
there is no shared bus manager or concurrent I2C access.

| Device | 7-bit address | Current transactions |
| --- | ---: | --- |
| TCA9536A | `0x40` | No current driver or polling; register reference below |
| MAX17048 | `0x36` | Startup VERSION/VCELL; continuous VERSION and separate two-byte VCELL `0x02`, SOC `0x04` reads |
| BQ25628E | `0x6A` | Startup identity, status, external-limit bit, ADC controls; conditional ADC results. Continuous identity and status `0x1D..0x1F` |
| BQ25622E alternative | `0x6B` | Startup identity only. Live monitor validates PN=3 and reads status `0x1D..0x1F`, whose fields match in both Rev. C datasheets |
| LP5813A | `0x50..0x53` | Active LED configuration, PWM, and readback; pages of one device |

All addresses in firmware are 7-bit addresses; do not shift or append a read/
write bit. BQ25628E 16-bit registers place the low byte at the lower register
address. MAX17048 16-bit transfers are most-significant byte first. That byte
order difference should be explicit in separate device helpers.

## U8 BQ25628E charger and power path

### Frozen connections

| Pin | Signal | Frozen-board connection |
| ---: | --- | --- |
| 1 | BTST | 47 nF to SW |
| 2 | REGN | 4.7 uF to GND |
| 3 | active-low PG | `CHG_PG_N` -> GPIO40, 10 kOhm pull-up |
| 4 | ILIM | 5.62 kOhm to GND |
| 5, 6 | TS_BIAS, TS | 5.23 kOhm upper, TH1 and 30.1 kOhm lower network |
| 7 | QON | Not connected |
| 8 | BAT | Battery J4 pin 1 and MAX17048 CELL/VDD |
| 9 | SYS | `3V3_RAW` |
| 10 | active-low STAT | `CHG_STAT_N` -> GPIO1, 10 kOhm pull-up |
| 11 | active-low INT | `CHG_INT_N` -> GPIO0, 10 kOhm pull-up |
| 12, 13 | SDA, SCL | `POW_SDA`, `POW_SCL` |
| 14 | active-low CE | Grounded: autonomous charging enabled |
| 16 | SW | 1 uH to SYS |
| 17 | PMID | 100 nF and 10 uF to GND |
| 18 | VBUS | USB-C J6 VBUS |

The retained primary reference is the [BQ25628E datasheet, Rev. C](https://www.ti.com/lit/ds/symlink/bq25628e.pdf)
and [BQ25628E product page](https://www.ti.com/product/BQ25628E). The fitted
device is pin-compatible with this board connection set, but it is not
firmware-identical to the schematic/BOM's BQ25622E.

### Identity and startup snapshot

The recorded expected part-information byte at `0x38` is `0x22`: part number field
`PN[5:3] = 100` for BQ25628E and revision field `DEV_REV[2:0] = 010`. Require
the BQ25628E part-number field and log the revision so a newer silicon revision
is visible without being misidentified as a different part.

| Register | Meaning |
| ---: | --- |
| `0x1D` | Charger Status 0: ADC done, thermal regulation, VSYSMIN, IINDPM/ILIM, VINDPM, safety-timer and watchdog status |
| `0x1E` | Charger Status 1: charge phase and VBUS source classification |
| `0x1F` | Fault Status 0: VBUS, BAT, SYS, OTG, thermal shutdown, and TS-zone faults |
| `0x20..0x22` | Event flags; some clear when read, so do not use them as the only retained diagnostic |
| `0x38` | Part Information; expected `0x22` |

In `0x1E`, `CHG_STAT[4:3]` means no charge/terminated, constant-current,
constant-voltage, or top-off for values `00..11`. `VBUS_STAT[2:0]` is `000` for
no input and `100` for an unknown/default source on BQ25628E. In
`0x1F`, a TS value of zero is the normal zone.

### Hardware defaults and write policy

| Register bytes | Reset value | Decoded reset behavior |
| --- | ---: | --- |
| `0x02..0x03` | `0x0100` | ICHG = 320 mA (`0x08 * 40 mA`) |
| `0x04..0x05` | `0x0D20` | VREG = 4.200 V (`0x1A4 * 10 mV`) |
| `0x06..0x07` | `0x0A00` | IINDPM = 3.200 A (`0xA0 * 20 mA`) |
| `0x08..0x09` | `0x0E60` | VINDPM = 4.600 V |
| `0x0E..0x0F` | `0x0B00` | VSYSMIN = 3.520 V |
| `0x10..0x11` | `0x0018` | Precharge current = 30 mA |
| `0x12..0x13` | `0x0010` | Termination current = 20 mA |
| `0x14` | `0x06` | Termination enabled; input-voltage battery tracking enabled |
| `0x16` | `0xA1` | Auto battery discharge and charging enabled; 50 s watchdog selected |

The 5.62 kOhm ILIM resistor imposes about 445 mA typical input-current limit
using the datasheet's 2500 A-Ohm typical coefficient (about 400-489 mA over its
2250-2750 range). The effective limit is the lower of this hardware limit and
IINDPM. On BQ25628E, `REG0x19.EN_EXTILIM` bit 2 defaults enabled; read the bit
rather than assuming the whole reset byte because the family-wide register
table also describes variants without the external-limit feature.

Most importantly, the charger starts in autonomous/default mode after POR. A
write to any register enters host mode and starts the selected watchdog. If the
host does not periodically set `WD_RST` before the 50 s reset interval—or first
explicitly disables the watchdog—the device restores applicable defaults,
halves ICHG, and asserts an interrupt. Therefore the current firmware performs
no BQ writes, including no ADC-enable write. A later driver must make watchdog
ownership an explicit state transition.

ADC control is at `0x26`, channel disable bits at `0x27`; results begin at
`0x28` (IBUS), `0x2A` (IBAT), `0x2C` (VBUS), `0x2E` (PMID), `0x30` (VBAT),
`0x32` (VSYS), `0x34` (TS), and `0x36` (die temperature). Results use
16-bit little-endian transfers. Firmware does not enable the ADC.

The startup diagnostic reads `0x1D..0x1F`, stopping before read-to-clear
flags at `0x20..0x22`, then reads `0x19` bit 2 and `0x26..0x27`.
It reports ADC results only when `(REG26 & 0xC0) == 0x80` (continuous ADC
already enabled) and the corresponding channel is enabled:

| Result | Gate and decode used by firmware |
| --- | --- |
| VSYS `0x32` | `REG27 & 0x08 == 0`; `((raw >> 1) & 0x0FFF) * 199 / 100` mV, integer arithmetic |
| IBAT `0x2A` | `REG27 & 0x40 == 0`; signed 16-bit raw, `(raw >> 2) * 4` mA; `0x8000` means aborted |

Disabled/one-shot ADC contents are not presented as current measurements.
`POWER` output is a **startup snapshot**: USB repeats the saved text every
10 seconds in `host-poc`; it does not resample charger or battery state.
Live readings are separate `battery-*` fields in `STATUS` (or `TRAJ` in the
onboard variant), populated by the independent battery task.

### Continuous battery monitor

[battery.rs](../../src/battery.rs) samples about every 500 ms, rechecks the
detected BQ25628E/BQ25622E identity, and compares `VBUS_STAT` against active-low PG
before and after I2C reads. Source disagreement, unsupported identity,
charging faults or bus failures produce unknown status instead of green.
The monitor never reads the event flags or changes the charger configuration.

The gauge must return VERSION `0x001_`, VCELL in 2500-4500 mV and plausible
SOC. SOC above 100% up to 110% is clamped to 100%; larger values leave
percentage unknown with `soc-out-of-range`, while voltage and charging remain
usable. `battery-soc-raw` preserves the raw SOC word for diagnosis.
The gauge is withheld for at least one second after startup, a failed read,
a sample gap of three seconds, or a voltage step exceeding 200 mV. Empty
J4 can pulse the gauge supply, so these checks reduce false indications but
do not prove physical cell presence. The detected charger address is retained
between polls, avoiding repeated probes of an absent device, and invalidated
on read failure. With external power and a NACKing gauge,
blue is allowed and percentage is unknown. On battery, missing gauge data
leaves the LED off because low-charge status cannot be determined.

Two consecutive samples confirm each colour; transitions are off until
confirmed. Red enters at SOC <=15% and stays latched until SOC >=20%, using
the unrounded gauge value. Charging takes precedence over low charge.
Telemetry invalidates failed readings immediately and reports snapshots older
than three seconds as stale. All reads retry on the next poll. No automatic
shutdown, charge-current policy, or fuel-gauge calibration is introduced.
Blue means external power without confirmed charging; without enabling the
charger ADC this cannot distinguish battery supplementation of a weak input.

On 2026-09-10 the connected board reported BQ25622E `0x1A` at `0x6B` and
MAX17048 version `0x0012`. Four-byte gauge reads repeated the voltage word;
separate register-addressed two-byte reads fixed that transport error. The
gauge's actual SOC register remained near 120%, so percentage accuracy is
unqualified. The flashed image reported charging/green and zero battery I2C
errors in the live check, with one brief voltage-step settling/off interval.
Use [check-battery.ps1](../../../tools/check-battery.ps1) for fresh USB status checks.

### Battery/thermistor safety hold

TH1's production-BOM part is Murata `NCU18XH103F60RB`, while the KiCad value
text still says `103KT1608T-1P`. TI's shown 5.23 kOhm/30.1 kOhm network and
default TS thresholds are designed around the 103AT curve. The intended cell,
its maximum charge voltage/current, and the fitted Murata curve must be checked
together before charging a battery. Until these are established, retain the
no-cell policy. A software current setting cannot protect against an unsuitable default already active before
firmware starts.

## U9 MAX17048 fuel gauge

U9 pins 2 (CELL) and 3 (VDD) connect directly to BAT, pins 4 and exposed pad to
ground, and pin 5 ALERT connects to `BAT_ALRT_N / GPIO2` with a 10 kOhm +3V3
pull-up. QSTRT is grounded. SDA/SCL are power I2C. With charging enabled and no
cell, BQ25628E battery detection can cycle BAT between its recharge and
regulation thresholds. The gauge can therefore ACK while BAT is high or NACK
while it droops; neither outcome alone proves a gauge fault. Treat empty J4's
BAT pin as live and measure it. TI describes this behavior in
[Battery Detection Using Single Cell Charger (Rev. A)](https://www.ti.com/lit/ab/sluab31a/sluab31a.pdf).

The datasheet denotes fixed bus bytes `0x6C` for write and `0x6D` for read; the
corresponding standard 7-bit address required by `embedded-hal` is `0x36`.
Registers are 16-bit and transferred most-significant byte first. The current
startup diagnostic reads VERSION and VCELL; the battery monitor also reads
SOC continuously. The other registers below are retained for future gauge
work. VCELL output uses `(raw * 5 + 32) / 64` rounded mV.

| Register | Reference interpretation |
| ---: | --- |
| `0x02` VCELL | Raw value * 78.125 uV |
| `0x04` SOC | Raw value / 256 percent; allow about 1 s after POR for first estimate |
| `0x08` VERSION | Read-only silicon/version presence check |
| `0x0A` HIBRT | Reset `0x8030` |
| `0x0C` CONFIG | Reset `0x971C`; includes RCOMP, sleep, alert latch and SOC threshold |
| `0x14` VALRT | Reset `0x00FF`, thresholds in 20 mV units |
| `0x16` CRATE | Battery percentage change rate, 0.208 percent/hour per LSB—not amperes |
| `0x1A` STATUS | POR and alert status |

ALERT is open drain and remains asserted until `CONFIG.ALRT` is cleared. Do not
issue QuickStart (`MODE 0x06`) or the POR command (`0x5400` to `0xFE`) in the
current firmware. Analog Devices says most applications should not use
QuickStart; it is only meaningful when the cell voltage is relaxed. See the official
[MAX17048/MAX17049 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf)
and [MAX17048 product page](https://www.analog.com/en/products/max17048.html).

## U5 TCA9536A button expander

The exact fitted suffix matters: `TCA9536A` has 7-bit address `0x40`. The plain
TCA9536 uses a different address. The four active-low buttons are:

| Expander pin | Port | Frozen-board switch |
| ---: | --- | --- |
| 1 | P0 | SW3 to GND |
| 8 | P1 | SW6 to GND |
| 2 | P2 | SW4 to GND |
| 4 | P3 | SW5 to GND |

There are no external button pull-ups. At reset all ports are inputs with the
approximately 100 kOhm internal pull-ups enabled, so input register `0x00`
should have low nibble `0xF` when no button is pressed and clear the associated
bit when pressed. Its upper nibble reads as ones.

| Register | Reset | Meaning |
| ---: | ---: | --- |
| `0x00` | input | Input port |
| `0x01` | `0xFF` | Output port |
| `0x02` | `0x00` | Polarity inversion |
| `0x03` | `0xFF` | Configuration: all inputs |
| `0x50` | `0x00` | Special function; P3 interrupt and pull-disable both off |

The current firmware leaves the expander untouched, including special-function
register `0x50`. Future button handling must preserve input mode and pull-ups
unless it deliberately takes ownership of those functions. See the
official [TCA9536 datasheet](https://www.ti.com/lit/ds/symlink/tca9536.pdf) and
[product page](https://www.ti.com/product/TCA9536).

## U4 LP5813A LED matrix driver

VIN is +3V3, EN is `GPIO39 / LED_EN`, SYNC is grounded, SW uses the fitted 1 uH
inductor, and VOUT has 22 uF. The `A` address option uses address bits `00`.
Because the high two register-address bits are encoded into the I2C slave
address, its four 256-byte pages appear as 7-bit addresses `0x50`, `0x51`,
`0x52`, and `0x53`; the transaction's register byte supplies the low eight
bits. This is not four physical devices.

The exact charlieplexed LED mapping is:

| Driver output | Anode group | Cathodes by color |
| --- | --- | --- |
| OUT0 | all anodes of D32 | D33 red, D34 green, D35 blue |
| OUT1 | all anodes of D35 | D32 green, D33 blue, D34 red |
| OUT2 | all anodes of D33 | D32 red, D34 blue, D35 green |
| OUT3 | all anodes of D34 | D32 blue, D33 green, D35 red |

### Active D32 configuration

EN low is shutdown. The task holds EN low for 10 ms, then high for 10 ms
before initialization. It enables the chip, waits 10 ms, writes configuration
and `CMD_UPDATE`, then waits another 10 ms before checking readback.

| Full register | Written value | Purpose in current driver |
| --- | --- | --- |
| `0x000` | `0x01` | Chip enable |
| `0x002` | `0x40` | Proven four-scan setup, 3 V target / 3.3 V pass-through |
| `0x003` | `0xE4` | Output mapping configuration |
| `0x00D` | `0x0B` | TI recommended short-detection threshold to avoid false shorts |
| `0x010` | `0x55` | Apply configuration (`CMD_UPDATE`) |
| `0x034..0x036` | `20` decimal | D32 green/red/blue current, 2 mA peak |
| `0x020`, `0x021` | `0x70`, `0x00` | Enable only D32 green, red and blue channels |
| `0x044..0x046` | Green: `128,0,0`; blue: `0,0,128`; purple: `0,128,128`; red: `0,128,0`; unknown: all zero | D32 A0 green, A1 red, A2 blue PWM |

Quarter scan and PWM 128 produce the low-brightness steady indicator.
D33–D35 remain off; there is no LED test mode. Configuration
readback requires `0x000..0x003 = [1,0,0x40,0xE4]`, `0x00D = 0x0B`, and
`0x300..0x304 = 0`. Color changes wait 10 ms and verify all 12 PWM bytes,
both enable bytes, and the five fault/status bytes. The same checks run on
every battery poll even if the colour is steady. Any I2C, readback, or fault
failure disables EN and reports an error through `STATUS led=...`. The driver
retries the full sequence after three seconds while battery polling continues.

References: [LP5813 datasheet](https://www.ti.com/lit/ds/symlink/lp5813.pdf),
[register-map guide](https://www.ti.com/lit/ug/snvu859/snvu859.pdf),
[sample-code guide](https://www.ti.com/lit/ug/snvu940/snvu940.pdf), and
[product page](https://www.ti.com/product/LP5813).

## U7 MAX16169 pushbutton controller

| U7 pin | Signal | Frozen-board connection |
| ---: | --- | --- |
| 1 | VCC | `3V3_RAW` |
| 2 | GND | Ground |
| 3 | active-low PB_IN | SW7 to ground |
| 4 | active-low CLR | `PWR_KILL_N / GPIO8`, 10 kOhm pull-up |
| 5 | active-low open-drain INT | `PWR_INT_N / GPIO38`, 10 kOhm pull-up |
| 6 | OUT | `GATE_EN` -> TPS63802 EN |

There are no registers. CLR low turns OUT off, but CLR is ignored while OUT is
already deasserted and during the protected interval immediately after turn-on.
The application arms shutdown after 100 ms (longer than the documented
maximum 76.8 ms startup CLR blanking interval), waits for INT high, then its
next low. It asserts GPIO8 low indefinitely. A single subsequent press cuts
the main rail, including under USB power. No storage flush is currently needed;
the long-press hardware path remains independent of the MCU.

## U6 TPD4EUSB30 USB/CC protection

U6 is passive and has no firmware interface. Its paired pins protect USB DN
(pins 1/10), USB DP (2/9), CC1 (4/7), and CC2 (5/6); pins 3 and 8 are ground.
J6 USB DP and DM each have a 22 Ohm MCU series resistor. CC1 and CC2 each have
5.1 kOhm Rd to ground at the connector and a 22 kOhm series path to MCU sensing
GPIO43 and GPIO45. This confirms J6 advertises a USB device/sink role at the CC
pins. See the official [TPD4EUSB30 datasheet](https://www.ti.com/lit/ds/symlink/tpd4eusb30.pdf)
and [product page](https://www.ti.com/product/TPD4EUSB30).

## U1 UM980 GNSS

| MCU peripheral/pin | Receiver connection |
| --- | --- |
| UART1 TX GPIO47 / RX GPIO46 | COM1 command path, 115200 baud for default setup |
| UART2 TX GPIO49 / RX GPIO48 | COM2 data path, initially 115200 then 460800 baud, 8N1 |
| GPIO44 | Reset released with pull-up; no hardware reset during configuration |

The default profile first proves the COM1 transmit → receiver → COM2 receive
path with `VERSIONA COM2`, trying COM2 at 115200 and 460800 to tolerate a
previous volatile setting. A working COM1 receive/acknowledgement path is not
required for this route. Commands are sent as complete CRLF-terminated lines;
yielding between command text and CRLF previously risked receiver timeouts.

The applied profile is:

```text
CONFIG SIGNALGROUP 1
UNLOG COM1
UNLOG COM2
MODE ROVER AUTOMOTIVE
CONFIG PVTALG AUTO
CONFIG ANTIJAM AUTO
CONFIG COM2 460800
BESTNAVA COM2 0.05
```

`SIGNALGROUP` saves itself and restarts the receiver when changed. The driver
waits three seconds and reprobes before sending the rest; other runtime
settings are volatile and no `SAVECONFIG` is sent. Startup requires ten
consecutive 50 ms receiver epoch steps with valid position and velocity within
an eight-second verification window. Without sufficient valid fixes, setup
can keep retrying; this is not by itself evidence of a UART failure.

Setup errors retry after three seconds. During capture, a three-second UART
read timeout restarts configuration. BESTNAVA status parsing checks CRC and
updates communication status even when the receiver has no solution. The raw
host stream can contain BESTNAVA lines rejected by the parser; a consumer must
validate them itself. `SINGLE` is standalone positioning, `NARROW_FLOAT` RTK
float, and `NARROW_INT` fixed RTK.

`--features rtk-50hz` selects signal group 8 and `BESTNAVA COM2 0.02`.
That path also has command-baud discovery and COM2-command fallbacks; see
[gnss_profile.rs](../../src/drivers/um980.rs). Group 8 produced 50 messages/s but
only one valid standalone solution/s on the tested receiver. Message rate
must not be substituted for valid solution rate. The default group-1 profile
produced 20 valid fixes/s. No correction source is configured by this firmware.

References: [N4 R1.14 command manual](https://en.unicorecomm.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf),
[N4 R1.4 manual, page 43, standalone/group-8 limits](https://en.unicorecomm.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf),
and [experiment details](../trajectory-poc.md).

## U3 SCH16T-K01 IMU

SPI2 uses mode 0, 1 MHz, MSB-first 48-bit SafeSPI frames. GPIO12 is SCK,
GPIO11 MOSI, GPIO13 MISO, GPIO10 software CS, GPIO9 active-low reset, and
GPIO14 active-high data-ready. CS is high between frames, with 1 µs delays
before/after assertion and before deassertion. Replies correspond to the
previous request; reads use `FREQ_CNTR` (`0x13`) to clock out pending replies.

### Register image and startup

These are the exact volatile register values in [imu.rs](../../src/imu.rs).
The profile selects LPF3 Bessel, DYN1 calibrated ranges, and DEC5 on all
decimated axes. It retains continuous and startup self-tests.

| Register(s) | Value | Use |
| --- | --- | --- |
| `0x25..0x27` | `0x00DB` each | Axis filter profile |
| `0x28..0x29` | `0x1324` each | Axis range/decimation profile |
| `0x2A..0x2E` | `0x0000` each | Remaining profile controls |
| `0x33` | `0x202C` | 3.3 V, normal slew, active-high data-ready |
| `0x34` | `0x0FFE` | Continuous/startup self-tests enabled |
| `0x35` CTRL_MODE | `1`, then `3` | Mode transition / end of initialization |
| `0x37` SYS_TEST | `0x5A3C`, then `0` | Write/readback interface check |
| `0x3C` COMP_ID | Expected `0x0023` | Required component identity |
| `0x3B` ASIC ID | Require `(value & 0x0F00) == 0` | ASIC identity check |
| `0x14..0x1D` | Healthy `0xFFFF` each | Ten diagnostic status words |
| `0x0A..0x0F` | Read | Decimated gyro XYZ, then accelerometer XYZ |

Startup asserts reset for 2 ms and waits 32 ms after release with SCK low.
It checks identity and SYS_TEST, writes and verifies the profile, writes
CTRL_MODE=1, and waits 215 ms. It reads latched startup status, writes
CTRL_MODE=3, waits 3 ms, and retires the pre-delay pipelined reply. It requires
two wholly healthy diagnostic passes within ten attempts, waiting 3 ms
between attempts so asynchronous fault clearing can finish. Then it enables
strict frame-status checks and re-verifies the profile, mode, and identity.
An initialization call tries up to five times; task-level failures wait two
seconds before starting another call.

### Measurement and rejection policy

Native output is nominally 737.5 Hz; four consecutive six-axis sets are
averaged into nominally 184.4 intervals/s (about 182/s observed). The first
set establishes a time/counter baseline. The six 4-bit counters can have
different offsets: each must advance by one modulo 16, rather than equal
its neighbors. Each native interval must be 500–2800 µs; data-ready waits
are bounded to 100 ms. These checks detect missed samples and gaps that a
complete counter wrap might hide.

Frames use augmented CRC-8 (initial `0xFF`, polynomial `0x2F`, processing the
40 message bits plus eight zero bits). Response address, command/status,
and frame format are checked. Measurement payloads are signed 20-bit values
in bits 27:8. Acceleration scales by 3200 LSB/(m/s²); gyro scales by
1600 LSB/(°/s) and is converted to rad/s. Values outside ±80 m/s² or ±300°/s
are rejected even if within electrical output headroom.

Any bad native set discards the complete four-set batch. The six-response
pipeline is drained even on a bad burst so data-ready can rearm. Ten
consecutive acquisition errors trigger reinitialization. Successful samples
reset that error streak; the last error remains available in status output.

Timestamps are MCU monotonic microseconds taken when software services
data-ready; an output timestamp marks the interval end. Factory correction
is active, but assembled-board calibration, boresight, filter-delay correction,
and high-rate coning/sculling compensation are not implemented. Nominal
LPF3 bandwidths are 280 Hz gyro and 240 Hz acceleration.

Reference: [Murata Doc. 11624 Rev. 6, sections 5–7](../../../data/datasheets/sch16t-k01-datasheet-full.pdf).
Preserve the startup delays, pipelined response handling, and diagnostic
clearing sequence when changing acquisition; identity reads alone do not
establish readiness.

## J3 LCD bring-up

Latest hardware finding: after correcting ribbon orientation, readback gives
ID `8181b3`. Commands wake the panel (`lcd-init=98000680`), but MADCTL/COLMOD
remain at defaults; pixel transfers reset it (`lcd-regs=0c000600`). Both FIFO
parameter writes and hardware-command-phase parameter writes behave the same.
GPIO19 input feedback is low with its output latch high: `lcd-dc=4` versus
healthy value 5 (bit 2: high output latch, bit 1: input while driven low,
bit 0: input while driven high). This is consistent with the LCD treating data
as commands because LCD_DC cannot rise. After the user disconnected the LCD,
a fresh STREAM handshake and LCDREINIT produced seven status samples with
`lcd-dc=4`, `lcd-id=000000`, and `lcd-regs=00ff0000`. The fault persists without
the panel, pointing to the board-side GPIO19 circuit. The physical cause is
not established. With board power removed, inspect and measure J3 pin 10
(GPIO19, module pad 25) to ground, including a possible bridge to adjacent
grounded J3 pin 11. A short, GPIO damage, and pin configuration remain to be
distinguished by measurement.

A subsequent startup probe of LCD_SCLK (GPIO16, J3 pin 9), with the LCD still
disconnected, passed: `lcd-sclk=26`. Before SPI routing, the pin followed weak
pull-down and pull-up, then sampled low and high during brief output tests.
This argues against a hard SCLK-to-ground short at the MCU; it does not measure
resistance or prove continuity to the connector. DC still reported `lcd-dc=4`.
The SCLK diagnostic runs once per boot. Its bit flags are: bit 0 pull-down
input, bit 1 pull-up input, bit 2 driven-low input, bit 3 driven-high input,
bit 4 output test performed. Output testing is skipped if either pull test
fails. A failed probe leaves SCLK undriven and reports `sclk-check-failed`;
reboot after repair to repeat this startup check.

The current startup checks DC before initialization. A failed check reports
`lcd=dc-stuck-low`, leaves DC low, stops frame writes, and retains the requested
10% backlight. Six consecutive USB status samples confirmed this state without
resets or sensor errors. `LCDREINIT\n` manually repeats the GPIO check and LCD
initialization. No automatic loop drives the fault high. A passing startup
captures register state before and after the first frame as `lcd-init` and
`lcd-regs`; these are snapshots, not continuous polling.

J3's SFV24R-2STE1HLF is **top contact**, despite Adafruit's contradictory
bottom-contact description of that part. The exposed ribbon conductors face
away from the PCB toward the upper contacts, with pin numbering preserved.
See [Amphenol drawing 10171070, sheet 4](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6255/10171070.pdf).

The Adafruit 4520 is a 240x240 ST7789VW panel. J3 pin 6 is tied high to select
4-wire SPI; panel analog and I/O supplies are 3.3 V. Panel pin 1 (LEDA) is
3.3 V, and pin 2 (LEDK) returns through R4 (15 ohms) and Q1 to ground.
GPIO4 drives Q1, so high enables the backlight.

| Signal | MCU GPIO | J3 pin |
| --- | ---: | ---: |
| Reset, active low | 15 | 7 |
| CS, active low | 18 | 8 |
| SCLK | 16 | 9 |
| Data/command | 19 | 10 |
| SDA/MOSI | 17 | 12 |
| TE frame output | 5 (input with pull-down) | 21 |

[lcd.rs](../../src/drivers/st7789.rs) owns SPI3 at 1 MHz, mode 0, MSB first. It applies
hardware and software reset, sleep-out, RGB565, MADCTL 0xC0, inversion on,
normal mode and vertical-blank TE output. The visible window is columns
0..239 and rows 80..319 (Adafruit rotation 0). After writing the first frame,
it enables the display and selects the persistent nominal 10% backlight setting.
Subsequent frames write six color bars,
a white border and a moving white marker. Each 480-byte row yields to other
core-0 work. No framebuffer or PSRAM is needed.

USB STATUS reports `lcd=scanning` after at least five rising TE edges in a
half-second observation window, or `lcd=no-te` otherwise. `lcd-te` counts
observed edges (excluding drawing time), and `lcd-frames` counts completed
frame writes. These prove controller activity and completed SPI transfers;
the visible colors, orientation and backlight still require visual inspection.
SPI errors disable the backlight and retry initialization after three seconds.

Bring-up on 2026-09-09 is incomplete. The first image reported
`lcd=initializing lcd-te=0 lcd-frames=1` before USB disconnected, and the user
confirmed a restart loop. Holding GPIO4 low restored stable operation, confirmed
by the user and advancing frame counts over USB. Manual `LCDLIGHT 1`,
`LCDLIGHT 5` and `LCDLIGHT 10` tests all completed without a reset. These use
10/50/100 microsecond GPIO pulses with at least 1 ms low time; scheduling makes
them duty ceilings, not calibrated brightness. At the user's request the 10%
setting now stays on after initialization and has no timeout. A 15-second live
check kept `lcd-light=10` throughout with advancing frames and no reset. The 1%
and 5% settings still expire after five seconds. The S31 LEDC driver is unavailable
in the pinned HAL.

The panel returns no TE pulses at 4 MHz or 1 MHz. Shared-SDA half-duplex reads
report ID `000000` and packed power/MADCTL/COLMOD/signal bytes `00ff0000`, which
do not match the programmed state. RDDID uses one dummy clock; the four
single-byte register reads use none (ST7789VW section 8.4.5). These observations
do not establish a responding panel. Visual and electrical checks remain.
Release build and formatting pass; strict Clippy has pre-existing diagnostics
outside lcd.rs.

The reliable flashing sequence uses the RAM stub: first send `BOOTLOADER\n`
on application COM5, then use `espflash flash --port COM4 --chip esp32s31
--before no-reset --after no-reset-no-stub --flash-size 16mb --non-interactive
--skip-update-check <ELF>` and run `tools/s31-reset/target/release/s31-reset.exe
COM4`. Build that helper from its own directory with `cargo build --release`.
Two successive flashes and software restarts passed with this sequence.
The ROM-only `--no-stub` path failed at FlashEnd and required SW7 recovery;
the espflash CLI's watchdog-reset dispatch also lacks S31 support, although
the library method called by the helper supports it.

References: [panel specification](../../../data/datasheets/4520_C13462__________.pdf),
[Adafruit product](https://www.adafruit.com/product/4520), and
[Adafruit initialization and offsets](https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp).

## USB protocol and programming

J6 uses the dedicated USB-HS controller and CDC-ACM, with 512-byte data packets.
Application identity is VID:PID `303A:4001`, manufacturer `AEVIA`, product
`V2 Mini Trajectory POC`, serial `V2MINI0001`. ROM download enumerates separately
as `303A:0020`. J5 exposes USB Serial/JTAG; UART0 is at TP2/TP1. These are
separate transports, not interchangeable connectors for the J6 console.

The default image emits CRLF-terminated ASCII records:

```text
RAW IMU <end_us> <interval_us> <ax> <ay> <az> <gx> <gy> <gz>
RAW GNSS <received_us> <original BESTNAVA line>
STATUS imu=... gnss=... led=... battery=... battery-pct=... battery-mv=... battery-part=... battery-regs=... battery-charger=... battery-gauge=... battery-errors=... last-imu-error=... capture-errors=... queue-drops=...
POWER ...
```

Acceleration is m/s² and angular rate is rad/s. GNSS receipt time and IMU end
time share the MCU monotonic clock, not UTC. STATUS is emitted approximately
once per second while the writer makes progress. POWER repeats the startup
snapshot on connection and every ten seconds; it is not live telemetry.
Consumers must frame lines across USB packets and tolerate status records
between measurements.

Send `STREAM <nonce>\n`, where nonce is exactly eight hexadecimal digits.
The USB writer clears queued measurements and responds
`STREAM READY <nonce> <time_us>\r\n` before further RAW records. Discard all
preceding input and match the nonce before starting a new consumer session.
Capture continues independently, and USB disconnect alone does not reset the
queue or estimator state in a consumer.

The command receiver also recognizes the byte sequence `BOOTLOADER`, sets
`LP_SYS.sys_ctrl.force_download_boot`, and software-resets into ROM download
mode. It is a sequence matcher, not a line-only command. Software entry needs
a functioning application USB connection. If neither application nor ROM USB
is present, the recorded recovery was a full SW7 power cycle. The repository
no longer provides a Cargo flashing runner or host tool scripts.

## Build configuration

Build from `firmware/` with `cargo build --release` so
[the target configuration](../../.cargo/config.toml) applies. The ELF is
`target/riscv32imafc-unknown-none-elf/release/aevia-firmware` relative to the
repository root. The pinned toolchain is Rust 1.95.0 and the target is
`riscv32imafc-unknown-none-elf`. The MCU uses `CpuClock::max()`.

[Cargo.toml](../../Cargo.toml) pins the Espressif HAL family to commit
`160b10794227eb84805b8676fe188c1110801e9d` with ESP32-S31 support; use that
revision when checking APIs rather than assuming another ESP target's API
matches. Useful pinned upstream references are the
[HAL manifest](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-hal/Cargo.toml),
[S31 metadata](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-metadata/devices/esp32s31/soc.toml),
and [I2C implementation](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-hal/src/i2c/master/mod.rs).

## Unresolved hardware and measurement limits

These are retained uncertainties, not claims that the current firmware checks
them: cell chemistry/capacity/charge limits and pack protection; fitted
thermistor versus schematic naming and TS curve; MCU EN ramp margin; and the
reset-time strap-documentation discrepancy. The charger can operate before
firmware starts and remains upstream of the switched +3V3 rail.

GNSS/IMU capture has been exercised, but timing is software-based without PPS,
antenna lever arm and board calibration are unestablished, and speed accuracy
has not been compared with an independent reference. PSRAM initialization
and full-memory read/write checks now pass; onboard GNSS fusion remains
unverified. Historical capture/test results and the PSRAM diagnosis are
kept in [trajectory-poc.md](../trajectory-poc.md).
