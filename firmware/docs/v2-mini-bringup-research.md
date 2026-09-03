# V2 mini minimal bring-up research

Research date: 2026-08-27

This note is an implementation reference for the first firmware and the first
powered-board test of the frozen V2 mini. It is not a statement that the board
has been electrically validated: at the time of writing no board has been
soldered. Net names, reference designators, values, and module-pad numbers below
come from the frozen [KiCad PCB](../../hardware/v2_mini_pcb/v2_mini.kicad_pcb),
the [production BOM](../../hardware/v2_mini_pcb/production/digikey_bom.csv), and
the [frozen-design note](../../v2_mini_design.md). Electrical behavior and
register meanings come only from the linked manufacturer documentation and
first-party software repository.

## Decisions for the first firmware

1. Boot, print an unmistakable banner and periodic tick, initialize only the
   safety-relevant GPIOs, then exercise the 100 kHz power I2C bus.
2. Leave `GPIO8 / PWR_KILL_N` released as an input with pull-up and never drive
   it low during bring-up. A sustained low level asks the MAX16169 to remove the
   main 3.3 V rail, so a low glitch can make the MCU reset itself.
3. Keep `GPIO39 / LED_EN` normally low. Pulse it high only for the independent
   LP5813 read, then return it low. Keep `GPIO4 / LCD_BACK` low throughout.
4. With no battery installed, allow the MAX17048 at `0x36` either to ACK while
   the charger energizes BAT or to NACK while BAT droops. Measure BAT; SOC has
   no useful meaning without a cell.
5. Treat the BQ25628E as read-only initially. Any write takes it out of autonomous
   default mode, starts its watchdog, and creates a recurring service obligation.
6. Do not enable charging or change charge voltage/current until the intended
   cell and its thermistor curve are confirmed. The charger is already capable
   of autonomous charging before firmware runs.
7. Probe only the known I2C addresses. Do not issue a generic scan across
   reserved/general-call addresses.

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
U1 and U3 are application peripherals, not prerequisites for the minimal
power/MCU bring-up; their buses should remain idle until the power path and MCU
are proven.

## Power tree and first bench-power test

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

For the very first power test, leave J4 empty, use a current-limited USB or
bench source, and press SW7 for at least the 50 ms debounce interval. Confirm in
order: USB VBUS, `3V3_RAW`, `GATE_EN`, then +3V3 near 3.3 V. The MAX16169 output
is deasserted on first application of VCC; the AALT option used here has 50 ms
debounce, 8 s long-press shutdown, a 32 ms normal interrupt pulse, and a 128 ms
long-press interrupt pulse. Holding the button beyond the shutdown interval
deasserts OUT. These timings and the CLR behavior are defined by the official
[MAX16169 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max16169.pdf)
and [product page](https://www.analog.com/en/products/max16169.html).

The board fits `R8 = 10 kOhm` and `C11 = 100 nF` on MCU EN. Espressif's module
reference circuit recommends 10 kOhm and 1 uF, and specifies at least 1 ms for
both rail stabilization and the reset-low pulse. Capture +3V3 and EN on the
scope during the first starts; firmware cannot repair an inadequate reset ramp.

## MCU bring-up pins

The frozen PCB maps the management and programming signals as follows. Numbers
are module pad numbers, followed by the ESP32-S31 GPIO/function.

| Module pad | MCU signal | Frozen-board net and connection | Initial disposition |
| --- | --- | --- | --- |
| 3, 4 | 3V3 | +3V3 | Power |
| 5 | EN | 10 kOhm pull-up, 100 nF to GND, SW1 to GND | Reset input |
| 6 | GPIO2 | `BAT_ALRT_N`, MAX17048 ALERT, 10 kOhm pull-up | Input |
| 8 | GPIO0 | `CHG_INT_N`, BQ25628E INT, 10 kOhm pull-up | Input |
| 9 | GPIO1 | `CHG_STAT_N`, BQ25628E STAT, 10 kOhm pull-up | Input |
| 12 | GPIO6 | `POW_SCL`, 5.1 kOhm pull-up | I2C0 SCL |
| 13 | GPIO7 | `POW_SDA`, 5.1 kOhm pull-up | I2C0 SDA |
| 14 | GPIO8 | `PWR_KILL_N`, MAX16169 CLR, 10 kOhm pull-up | Released input with pull-up |
| 40 | dedicated USB_DP | J6 through 22 Ohm | USB-HS/OTG connector |
| 41 | dedicated USB_DM | J6 through 22 Ohm | USB-HS/OTG connector |
| 42 | GPIO33 / USB Serial/JTAG D- | J5 through 22 Ohm | Native debug USB |
| 43 | GPIO34 / USB Serial/JTAG D+ | J5 through 22 Ohm | Native debug USB |
| 44 | GPIO35 | `USB_VBUS_S`; 75 kOhm/100 kOhm VBUS divider | Input; 5 V gives about 2.86 V |
| 49 | GPIO38 | `PWR_INT_N`, MAX16169 INT, 10 kOhm pull-up | Input |
| 50 | GPIO39 | `LED_EN`, LP5813 EN | Output low initially |
| 51 | GPIO40 | `CHG_PG_N`, BQ25628E PG, 10 kOhm pull-up | Input |
| 68 | GPIO58 / UART0_TXD | TP2 through 499 Ohm | Conservative first log output |
| 69 | GPIO59 / UART0_RXD | TP1 | Console input |
| 71 | GPIO61 | BOOT/SW2 to GND, 10 kOhm pull-up | Boot strap |

`GPIO4 / LCD_BACK` should also be initialized low; `GPIO9 / IMU_RESET_N` and
`GPIO44 / GNSS_RESETN` have board pull-ups. J5 is the native USB Serial/JTAG
port and J6 is the separate dedicated USB-HS/OTG port; they are not alternate
connectors for the same peripheral.

The current module and series datasheets enumerate GPIO36, GPIO37, GPIO60, and
GPIO61 as strapping pins. Normal SPI boot has GPIO61 high. Joint Download mode
is selected with GPIO61 low and GPIO60 high, so the board procedure is: hold
SW2, pulse SW1, then release SW2. GPIO61 low with GPIO60 low is invalid. The
current ESP-IDF GPIO page lists a broader set of strap-capable pads (GPIO36-40
and GPIO60-61), which conflicts with the preliminary silicon/module datasheet.
Until a board proves otherwise, avoid changing GPIO38-40 during reset and save
the complete ROM boot message. See Espressif's [series datasheet](https://documentation.espressif.com/esp32-s31_datasheet_en.html),
[boot-mode instructions](https://docs.espressif.com/projects/esptool/en/latest/esp32s31/advanced-topics/boot-mode-selection.html),
[serial connection instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/get-started/establish-serial-connection.html),
and [GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/api-reference/peripherals/gpio.html).

## Power I2C bus

The bus is `GPIO6 / POW_SCL` and `GPIO7 / POW_SDA`, with 5.1 kOhm pull-ups to
+3V3. Start at 100 kHz.

| Device | 7-bit address | Expected without a battery | Safe first transaction |
| --- | ---: | --- | --- |
| TCA9536A | `0x40` | Present | Read input register `0x00` |
| MAX17048 | `0x36` | ACK or NACK as BAT cycles | If present: read VERSION `0x08` |
| BQ25628E | `0x6A` | Present from USB/power path | Read part info `0x38`, then status `0x1D..0x1F` |
| LP5813A page 0 | `0x50` | Absent while `LED_EN` is low | Raise EN, wait at least 1 ms, read `0x000`, lower EN |
| LP5813A pages 1-3 | `0x51..0x53` | As above | Do not probe until needed |

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

The primary reference is the current [BQ25628E datasheet, Rev. C](https://www.ti.com/lit/ds/symlink/bq25628e.pdf)
and [BQ25628E product page](https://www.ti.com/product/BQ25628E). The fitted
device is pin-compatible with this board connection set, but it is not
firmware-identical to the schematic/BOM's BQ25622E.

### Read-only identity and status

The current part-information byte at `0x38` should be `0x22`: part number field
`PN[5:3] = 100` for BQ25628E and revision field `DEV_REV[2:0] = 010`. Require
the BQ25628E part-number field and log the revision so a newer silicon revision
is visible without being misidentified as a different part.

| Register | Reset/meaning useful at first boot |
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

### Important defaults and later write policy

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
halves ICHG, and asserts an interrupt. Therefore the initial firmware performs
no BQ writes, including no ADC-enable write. A later driver must make watchdog
ownership an explicit state transition.

If ADC operation is enabled later, control is at `0x26`; results begin at
`0x28` (IBUS), `0x2A` (IBAT), `0x2C` (VBUS), `0x2E` (PMID), `0x30` (VBAT),
`0x32` (VSYS), `0x34` (TS), and `0x36` (die temperature). These are 16-bit
little-endian registers. Do not enable the ADC merely to prove I2C presence.

### Battery/thermistor safety hold

TH1's production-BOM part is Murata `NCU18XH103F60RB`, while the KiCad value
text still says `103KT1608T-1P`. TI's shown 5.23 kOhm/30.1 kOhm network and
default TS thresholds are designed around the 103AT curve. The intended cell,
its maximum charge voltage/current, and the fitted Murata curve must be checked
together before charging a battery. The first powered test should leave J4
empty; a software current setting cannot protect against an unsuitable default
that is already active before firmware.

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
Registers are 16-bit and transferred most-significant byte first.

| Register | First-use interpretation |
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
issue QuickStart (`MODE 0x06`) or the POR command (`0x5400` to `0xFE`) during
bring-up. Analog Devices says most applications should not use QuickStart; it
is only meaningful when the cell voltage is relaxed. See the official
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

Leave special-function register `0x50` unchanged during bring-up. See the
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

EN low is shutdown. After EN rises, soft start is typically 450 us; wait at
least 1 ms before the first transaction. The lowest-risk presence check is a
read of `Chip_Enable` at full register `0x000` through address `0x50`, whose
reset value is zero, followed by returning EN low. LED enables and manual PWM
values reset off, so no LED write is needed to prove the bus.

A later lighting sequence is: EN high, wait, set `Chip_Enable` (`0x000`) to 1,
configure the device/current/mapping, write `CMD_UPDATE` (`0x010`) with `0x55`,
and inspect `Config_Error_Status` (`0x300`, page address `0x53`). Keep current
low for the first optical test. See TI's official [LP5813 datasheet](https://www.ti.com/lit/ds/symlink/lp5813.pdf),
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
Firmware shutdown should therefore request CLR only after logs/storage are
quiescent; the long-press hardware path remains the recovery mechanism.

## U6 TPD4EUSB30 USB/CC protection

U6 is passive and has no firmware interface. Its paired pins protect USB DN
(pins 1/10), USB DP (2/9), CC1 (4/7), and CC2 (5/6); pins 3 and 8 are ground.
J6 USB DP and DM each have a 22 Ohm MCU series resistor. CC1 and CC2 each have
5.1 kOhm Rd to ground at the connector and a 22 kOhm series path to MCU sensing
GPIO43 and GPIO45. This confirms J6 advertises a USB device/sink role at the CC
pins. See the official [TPD4EUSB30 datasheet](https://www.ti.com/lit/ds/symlink/tpd4eusb30.pdf)
and [product page](https://www.ti.com/product/TPD4EUSB30).

## esp-hal ESP32-S31 status and pinned API

ESP32-S31 support is not in the older crates.io release used by established
ESP targets. As of this research date, the first-party repository's
`esp-hal-v1.2.0-rc.0` tag contains S31 GPIO, UART, and I2C-master support. Pin
the dependency to the tag commit rather than an advancing branch:

```toml
esp-hal = { git = "https://github.com/esp-rs/esp-hal", rev = "160b10794227eb84805b8676fe188c1110801e9d", features = ["esp32s31"] }
```

The Rust target is `riscv32imafc-unknown-none-elf`. The S31 metadata still calls
support early-stage; sleep and wireless support are not bring-up dependencies.
The authoritative pinned sources are the [HAL manifest](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-hal/Cargo.toml),
[S31 device metadata](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-metadata/devices/esp32s31/soc.toml),
[I2C master implementation and examples](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-hal/src/i2c/master/mod.rs),
and [`esp-println` manifest](https://github.com/esp-rs/esp-hal/blob/160b10794227eb84805b8676fe188c1110801e9d/esp-println/Cargo.toml).
The upstream [S31 MVP tracker](https://github.com/esp-rs/esp-hal/issues/5782)
remains useful when an apparent hardware fault may instead be a HAL gap.

The relevant current API shape is:

```rust,ignore
use esp_hal::{
    i2c::master::{Config, I2c},
    time::Rate,
};

let peripherals = esp_hal::init(esp_hal::Config::default());
let config = Config::default().with_frequency(Rate::from_khz(100));
let mut power_i2c = I2c::new(peripherals.I2C0, config)?
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO6);

power_i2c.write_read(0x6a, &[0x38], &mut part_info)?;
```

`esp-println`'s auto transport is available in the pinned revision. Still make
the physical connector explicit in the bench instructions: J5 is native USB
Serial/JTAG, J6 is USB-HS/OTG plus charger input, and UART0 is exposed only at
TP2/TP1. A log failure on one connector is not evidence that the MCU did not
boot.

## Minimal acceptance sequence

1. With no battery, inspect for shorts and power from a current-limited source.
2. Press SW7 for at least 50 ms. Measure `3V3_RAW`, `GATE_EN`, +3V3, and MCU EN.
3. Hold SW2, pulse SW1, and confirm a Joint Download connection. Save ROM text.
4. Flash the minimal image; normal-reset with SW1 while SW2 is released.
5. Confirm the firmware version banner and periodic pass counter.
6. Confirm `PWR_KILL_N` remains high, the backlight remains low, and `LED_EN`
   returns low after each brief LP5813 probe.
7. Start I2C0 at 100 kHz on GPIO6/7 and perform only the listed reads.
8. Require `0x40`, configuration `0xFF`, and TCA input low nibble `0xF` when
   buttons are released.
9. Require BQ25628E `0x6A` with PN field 4 (current revision byte `0x22`); log
   raw `0x1D..0x1F`.
10. Allow MAX17048 `0x36` to ACK or NACK while J4 is empty; record BAT voltage.
11. Raise LP5813 EN, wait at least 1 ms, read page-0 register `0x00`, then return
    EN low. No LED-current or PWM write is needed.
12. Hold SW7 longer than 8 s only after logs are captured, and verify the rail
    shuts down. Re-power and repeat to prove the latch path.

Do not call the bring-up successful from I2C ACKs alone. The initial evidence
set should include current consumption, rail/EN captures, ROM boot output, the
firmware banner/tick, all raw status bytes, BAT voltage, and the observed gauge
ACK/version or NACK.

## Hardware facts still requiring first-board validation

- Battery chemistry, capacity, allowed charge voltage/current, and pack
  protection are not documented alongside the frozen design.
- The BOM/KiCad thermistor naming mismatch must be resolved against the actual
  fitted part and the BQ25628E TS thresholds.
- MCU EN capacitance is one tenth of Espressif's reference value; scope it.
- Espressif's preliminary strap lists conflict; preserve ROM logs and keep
  GPIO38-40 passive through reset until silicon behavior is observed.
- J5, J6, and UART test points expose different transports. Record which one is
  actually used for flashing and which one carries application logs.
- The charger is autonomous with CE grounded, so the absence of running
  firmware is not a safe mechanism for disabling battery charge.
