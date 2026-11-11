# Aevia device HAL

Portable `no_std` hardware drivers and interfaces, kept entirely in this folder.
The current board is **V2 Mini**. Earlier full-size V2 components are separate;
their presence here does not mean they are populated on V2 Mini.

This independent crate is named `aevia-device-hal`. The firmware consumes it
through a path dependency for sensor drivers, power peripherals and flash.
It retains its own Cargo workspace so portable driver tests can run separately.
Scheduling, timestamp policy and estimator integration remain in device code.

## Design

- I2C, SPI devices, GPIO and delays use `embedded-hal` 1.0. UART and USB byte
  streams use `embedded-io` 0.7; NOR backends use `embedded-storage` 0.3.
  All three crates are re-exported for adapters.
- Drivers own their bus handles and return the underlying transport errors.
  Construction does no I/O; configuration and destructive operations are explicit.
- `bus::SharedI2cDevice` supports cooperative sharing of the power bus via a
  `RefCell`. For concurrent tasks or interrupt handlers, use the scheduler's
  bus lock instead. `bus::ExclusiveSpiDevice` wraps an owned MCU `SpiBus`,
  active-low CS and delay; explicitly initialize it before passing it to a driver.
- Bus calls are blocking. Set hardware deadlines in the MCU backend and schedule
  long initialization, display and storage operations outside acquisition loops.
  Neither a blocking trait nor a software polling limit can interrupt a hung bus.
- Raw register APIs supplement typed APIs for less common documented features.
  They require the caller to respect reserved, read-to-clear and one-shot bits.
- `interfaces` supplies boundaries for calibrated ADC inputs, temperature,
  hardware edge capture, block/NOR/RAM storage, packet radios, Bluetooth GATT,
  USB state and optional CAN. MCU register access and protocol stacks belong to
  the MCU/vendor backend; a module datasheet alone does not define those drivers.

No heap or unsafe code is used by the portable library. Device timestamps and
measurements are returned to the caller; estimator configuration and session
recording do not belong in these drivers.

## Use from firmware

The firmware manifest declares:

```toml
[dependencies]
aevia-device-hal = { path = "src/device/hal", features = ["esp32s31"] }
```

The feature enables target-specific vendor adapters on RISC-V. Generic drivers
also work without that feature on any MCU implementing the standard bus traits.
Supply already configured peripherals; `board` only describes wiring and never
silently consumes pins. Use distinct device handles on the shared power bus.
Keep UM980 command/data port ownership, IMU timestamps and bus scheduling under
the application's control.

## Components

| Component | Interface / role | Board / source |
| --- | --- | --- |
| [`sch16t`](sch16t/mod.rs) | SafeSPI IMU, temperature, counters and diagnostics | Mini + V2; local SCH16T Rev. 6 |
| [`um980`](um980/mod.rs) | UART commands, binary and NMEA frames, navigation and PPS | Mini + V2; local R1.9 hardware manual and existing N4 protocol evidence |
| [`bq25622`](bq25622/mod.rs) | BQ25622 / BQ25622E charge, power path, ADC and flags | Mini fits E; local base datasheet plus TI E supplement |
| [`max17048`](max17048/mod.rs) | MAX17048/49 voltage, SOC, alerts and battery model | Mini fits MAX17048; local datasheet |
| [`lp5813`](lp5813/mod.rs) | Four scanned RGB LEDs, PWM, current and animation engines | Mini; local datasheet + TI register-map guide |
| [`tca9536`](tca9536/mod.rs) | Expander variants, four UI buttons and debounce | Mini fits TCA9536A; local datasheet |
| [`temperature`](temperature/mod.rs) | NTC resistance/temperature conversion and sensor adapters | Mini TH1; explicit verified Murata preset or caller curve |
| [`display`](display/mod.rs) | ST7789VW SPI RGB565 panel | Mini J3; local ZJY133T-IF05 panel specification |
| [`sdcard`](sdcard/mod.rs) | 512-byte SD block operations through a native host backend | Mini uses 4-bit SDMMC; socket has no software registers |
| [`max16169`](max16169/mod.rs) | Power-button interrupt and explicit shutdown via GPIO | Mini; local timing/variant datasheet |
| [`regulators`](regulators/mod.rs) | Enable/mode/power-good GPIO where exposed | Local TPS63802, TPSM863253 and RPL-3.0 datasheets |
| [`leds`](leds/mod.rs) | WS2812B and APA102 serial LED interfaces | Local WS2812B datasheet; APA102 is an earlier V2 BOM option |
| [`bq25798`](bq25798/mod.rs) | Multi-cell charger, ADC, status and power-path controls | Earlier V2; local datasheet |
| [`max17320`](max17320/mod.rs) | Multi-cell fuel gauge and protection | Earlier V2; local datasheet |
| [`ap33772s`](ap33772s/mod.rs) | USB-PD sink controller | Earlier V2 BOM; no local part datasheet |
| [`memory`](memory/mod.rs) | NOR and PSRAM backend boundaries | Mini integrated flash/PSRAM; earlier V2 identities need explicit selection |
| [`esp32s31`](esp32s31/mod.rs) | MCU peripheral/communication backend boundaries | Mini; local module datasheet and installed vendor HAL |

The native backend targets the fitted ESP32-S31. Earlier STM32N657 and ESP32-C6
platforms can provide the same standard bus/transport interfaces; this crate does
not invent a firmware protocol for an unspecified wireless coprocessor image.

ESP32-S31's module datasheet advertises internal TSENS, but installed `esp-hal`
1.2.2 does not expose it for this chip. Native die temperature needs a future
supported backend; use the implemented external NTC, SCH16T or charger sensors
for their respective measurement locations. Wi-Fi/Bluetooth/802.15.4 also require
a vendor protocol stack behind the supplied transport boundaries.

The local `msl0402rgbu1-e.pdf` specifies the passive RGB LEDs driven by the
LP5813; they have no register interface. `tpd4eusb30.pdf` specifies passive USB
ESD protection and likewise has no software interface. Crystals, antenna, FPC,
microSD socket, resistors, capacitors and inductors need no invented drivers.
Other older BOM regulators and protection parts without data interfaces are
recorded here as electrical components, with GPIO control supplied only where
it actually exists on the board. The V1 BOM adds no programmable component.

## Board wiring and sources

`board.rs` records V2 Mini GPIOs and shared-bus settings from the actual PCB,
not assumed module pad numbers. Power I2C uses **100 kHz** on SCL GPIO6 / SDA
GPIO7. The fitted **TCA9536A address is 0x40** (the unsuffixed TCA9536 uses 0x41).
The LP5813 uses page addresses 0x50–0x53. MAX17048 is 0x36; BQ25622(E) is 0x6B.

TH1 is the 10 kΩ NTC at the charger's TS input. The PCB has a 5.23 kΩ
pullup to TS_BIAS and a 30.1 kΩ parallel resistor to ground. It has no direct
MCU ADC connection. Converting TS readings requires this network and a confirmed
thermistor curve; the BOM's NCU18XH103F60RB MPN differs from the schematic value
103KT1608T-1P. `temperature::BetaCurve::murata_ncu18xh103f60rb` explicitly selects the BOM
part's 10 kΩ / B25/50=3380 K approximation. Select it only after confirming
that fitted part; it does not compensate self-heating or replace a calibrated
resistance/temperature table.

The board has four active-low UI buttons on expander P0–P3. SW1 drives MCU
reset, SW2 is GPIO61 boot/user, and SW7 feeds the MAX16169 power controller.
P3 is wired to a button, so repurposing it as the expander's INT output would
remove that button input. The microSD card-detect contact is not connected.
UM980 COM3, SPI and I2C are not connected; the manual marks non-UART serial
interfaces reserved/not currently supported.

The authoritative local inputs are:

- [V2 Mini production BOM](../../../../hardware/v2_mini_pcb/production/digikey_bom.csv)
  and [PCB](../../../../hardware/v2_mini_pcb/v2_mini.kicad_pcb).
- [Full-size V2 BOM](../../../../hardware/v2_pcb/BOM.md) and
  [netlist review](../../../../hardware/v2_pcb/open-race_v2_net_simplified.txt).
  The older BOM and schematic disagree on PSRAM identity; use an explicit
  verified part/backend rather than automatically applying a memory configuration.
- [Local datasheets](../../../../data/datasheets/). Each component documents its
  exact source and any supplemental primary documentation it needs.
- `sources.json` captures input SHA-256 fingerprints. Run
  `python3 src/device/hal/tools/verify_sources.py` from `firmware/` to check
  source drift and GPIO mappings against the PCB.

## Verification

From `firmware/`, with the installed target and cached dependencies:

```sh
cargo test --manifest-path src/device/hal/Cargo.toml \
  --target aarch64-apple-darwin --config 'unstable.build-std=[]' --offline
cargo check --manifest-path src/device/hal/Cargo.toml \
  --target riscv32imafc-unknown-none-elf --config 'unstable.build-std=[]' --offline
```

Replace the host target with your installed host target on another machine.
Add `--features esp32s31 --release` to the target check to validate the optional
native backend as well. Use `cargo clippy` with `--all-targets -- -D warnings`
and the host target/configuration above for strict linting.

These checks build this crate independently; building the existing firmware does
not compile this folder until it is integrated. Protocol/mock tests check wire
encodings, boundaries, units and failure paths. Electrical timing, bus sharing
under the final scheduler and fitted device behavior still require board tests.
