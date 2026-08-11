# Open Race V2 Mini

V2 Mini is a battery-powered GNSS, motion, storage, and display board built around the ESP32-S31 module. The KiCad project and production BOM define the hardware. This page is only a short map of the design, so it leaves out pin allocations, routing details, and most passive components.

## High-level design

```mermaid
flowchart LR
    MCU["ESP32-S31-WROOM-3<br/>main processor and radio"]
    GNSS["UM980 GNSS<br/>U.FL antenna"]
    IMU["SCH16T-K01-1<br/>6-axis IMU"]
    SD["microSD<br/>4-bit SDIO"]
    LCD["24-pin display FPC<br/>serial display interface"]
    UI["Four UI buttons and four RGB LEDs<br/>TCA9536 + LP5813"]
    USB["USB-C<br/>power and native USB data"]
    PM["Power management<br/>charger, fuel gauge and on/off control"]

    MCU <-->|"Two UARTs, PPS and status"| GNSS
    MCU <-->|"SPI, data-ready and reset"| IMU
    MCU <-->|"4-bit SDIO"| SD
    MCU -->|"Display and backlight control"| LCD
    MCU <-->|"Shared I2C control bus"| UI
    MCU <-->|"I2C and status signals"| PM
    MCU <--> USB
```

The ESP32 module is the main processor and wireless interface. The UM980 supplies GNSS data over two UARTs, with PPS, reset, and status lines. The SCH16T uses a separate SPI bus with data-ready and reset signals.

The microSD socket uses 4-bit SDIO. The display connects through a 24-pin FPC and has a MOSFET-switched backlight. The display panel itself is not a fitted PCB BOM item.

Four user buttons are read through the TCA9536 GPIO expander, and the LP5813 drives four RGB LEDs. The other three buttons are reset, boot/user, and power. The power-management ICs, LED driver, and GPIO expander share the board's I2C control bus.

USB-C provides 5 V input and native USB data. The TPD4EUSB30 protects D+, D-, CC1, and CC2. The board also senses VBUS and both CC lines.

## Power tree

```mermaid
flowchart TD
    USB["USB-C J6<br/>5 V VBUS"] --> CHG["BQ25622 U8<br/>1-cell charger and NVDC power path"]
    BAT["1-cell battery J4"] <--> CHG
    BAT --> GAUGE["MAX17048 U9<br/>fuel gauge"]
    CHG --> RAW["3V3_RAW<br/>BQ25622 SYS rail"]

    RAW --> CTRL["MAX16169 U7<br/>power-button controller"]
    BUTTON["Power button SW7"] --> CTRL
    RAW --> REG["TPS63802 U10<br/>3.3 V buck-boost"]
    CTRL -->|"GATE_EN"| REG

    REG --> V33["+3V3"]
    V33 --> CORE["ESP32, microSD, display and user interface"]
    V33 --> FB1["FB1"] --> VGNSS["+3V3_GNSS"] --> GNSS["UM980 and antenna bias"]
    V33 --> FB2["FB2"] --> VIMU["+3V3_IMU"] --> IMU["SCH16T"]
```

The schematic calls the BQ25622 SYS output `3V3_RAW`. The TPS63802 turns that raw rail into the regulated `+3V3` system rail. FB1 and FB2 then provide filtered supplies for the GNSS receiver and IMU.

The MAX16169 stays on the raw rail and controls the TPS63802 enable input. SW7 is the power button. The ESP32 receives the power-button interrupt and can request a controlled shutdown. The MAX17048 measures the battery state of charge, while the BQ25622 handles charging and power-path status. TH1 is a fitted 10 kOhm NTC in the charger temperature-sense network.

## Main fitted parts

The part numbers below come from the production BOM rather than the shortened values shown on some schematic symbols.

| Ref | Ordered part | Purpose |
|---|---|---|
| U2 | `ESP32-S31-WROOM-3-N16R16V` | Main processor and wireless module |
| U1 | `UM980` | GNSS receiver |
| U3 | `SCH16T-K01-1` | 6-axis accelerometer and gyroscope |
| U4 | `LP5813ADRRR` | I2C RGB LED driver |
| U5 | `TCA9536ADTMR` | GPIO expander for the four UI buttons |
| U6 | `TPD4EUSB30DQAR` | USB data and CC ESD protection |
| U7 | `MAX16169AALTA+T` | Power-button and regulator-enable control |
| U8 | `BQ25622ERYKR` | 1-cell charger and NVDC power path |
| U9 | `MAX17048G+T10` | Battery fuel gauge |
| U10 | `TPS63802DLAR` | 3.3 V buck-boost regulator |
| D32-D35 | `MSL0402RGBU1` | Four RGB indicators |
| SW1-SW7 | `B3U-1000P` | Reset, boot/user, four UI buttons, and power |
| TH1 | `NCU18XH103F60RB` | 10 kOhm charger temperature sensor |
| J1 | `U.FL-R-SMT-1(01)` | GNSS antenna connector |
| J2 | `GTFP08441BEU` | microSD socket |
| J3 | `SFV24R-2STE1HLF` | 24-pin display FPC connector |
| J4 | `DF58-2P-1.2V(21)` | 1-cell battery connector |
| J6 | `USB4105-GF-A` | USB-C connector |

## Hardware source files

- [Root schematic](pcb/v2_mini_pcb/v2_mini.kicad_sch)
- [PCB layout](pcb/v2_mini_pcb/v2_mini.kicad_pcb)
- [Production BOM](pcb/v2_mini_pcb/production/digikey_bom.csv)
- [BOM exclusions](pcb/v2_mini_pcb/production/bom_exclusions.csv)
