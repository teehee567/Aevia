# v2 BOM — Major Components

place to order, if from jlcpcb, then get them to assemble aswel

| # | Category | Function | Part Number | placeto order | Qty | Price |
|---|---|---|---|---|---|---|
| 1 | Compute | Main MCU | STM32N657X0H3Q | jlcpcb | 1 | 40 |
| 2 | Compute | Wireless co-proc | ESP32-C6-MINI-1U | jlcpcb | 1 | 5 |
| 3 | Memory | Boot flash | MX25U51279GXDR00 | digikey | 1 | 18 |
| 4 | Memory | Data storage (micro sd) | anything, maybe sandisk/samsung industrial alter on, for now cheapest | anywhere | 1 | 15 |
| 5 | Memory | micro sd socket | DM3CS-SF | digikey | 1 | 2 |
| 6 | Memory | PSRAM | Infinieon S80KS5122GABHV020 | digikey | 1 | 19 |
| 7 | Clocks | HSE crystal | NDK NX2016SA-24MHZ-EXS00A-CS10820 | digikey | 1 | 1 |
| 8 | Clocks | LSE crystal | NDK NX2012SA-32.768KHZ-EXS00A-MU00527 | digikey | 1 | 1 |
| 9 | Power | USB-PD sink controller | AP33772S | digikey | 1 | 3 |
| 10 | Power | Battery charger | BQ25798 | jlcpcb | 1 | 4 |
| 11 | Power | Fuel gauge / protection | MAX17320 | SoC gauge + protection | 1 | 10 |
| 12 | Power | Battery pack | 21700 cell | anywhere | 2 | 10 |
| 13 | Power | USB-C port protection | TI TPD8S300A | jlcpcb | 1 | 2 |
| 14 | Power | 3.3V buck-boost | TPS63802 | jlcpcb | 2 | 2 |
| 17 | Power | 1.8V buck and 3.3v buck (leds) | TPS62A02 | digikey | 2 | 1 |
| 18 | Power | GNSS LDO | LT3045EDD#TRPBF | jlcpcb | 1 | 9 |
| 19 | Power | IMU LDO | TI TPS7A02 | digikey | 1 | 1 |
| 20 | Power | 5V boost | TBD | Optional, for buzzer | 0 | TBD |
| 23 | Sensors | IMU | Murata SCH16T-K01/K20 | mouser | 1 | 80 |
| 24 | GNSS | GNSS module | Unicore UM980 | aliexpress | 1 | 120 |
| 25 | GNSS | Antenna | Beitian BT-T076 | GNSS antenna | 1 | 35 |
| 26 | HMI | Indicator LEDs | Lite-On LTST-C190 | digikey | 5 | 1 |
| 27 | HMI | Bright RGB LED | APA102-2020-256-8 | digikey | 5 | 1 |
| 28 | HMI | Display | TBD | 5", 1000 nit, possible touchscreen | 1 | 70 |
| 29 | HMI | Buttons | TBD | High-quality tactile switches | 5 | 1 |



## cost estimation
jlcpcb assembly so 2x boards,
um980, batteries, antenna excluded

### JLCPCB-assembled parts (×2 boards)

| Part | ×2 |
|---|---|
| STM32N657X0H3Q | 80 |
| ESP32-C6-MINI-1U | 10 |
| BQ25798 | 8 |
| TPD8S300A | 4 |
| TPS63802 ×2 | 4 |
| LT3045 | 18 |
| **Subtotal** | **124** |

### DNP parts (×1 board)

| Part | Cost |
|---|---|
| MX25U51279 boot flash | 18 |
| microSD card | 15 |
| microSD socket (DM3CS-SF) | 2 |
| S80KS5122 PSRAM | 19 |
| HSE + LSE crystals | 2 |
| AP33772S PD | 3 |
| MAX17320 | 10 |
| TPS62A02 ×2 | 1 |
| TPS7A02 | 1 |
| Murata SCH16T IMU | 80 |
| Indicator LEDs ×5 | 1 |
| APA102 RGB ×5 | 1 |
| Display | 70 |
| Buttons ×5 | 1 |
| um980 | 120 |
| antenna | 36 |
| **Subtotal** | **380** |


### PCB + passives + assembly

| Item | Estimate |
|---|---|
| PCB (5×, 6-layer, ENIG 2u, black, Tg155) | 170 |
| Passives R/C + ~7 inductors, 2 boards | ~25 |
| JLCPCB SMT service | ~35 |

### Total

| | Amount |
|---|---|
| JLCPCB parts ×2 | 124 |
| Own parts ×1 | 380 |
| PCB | 170 |
| Passives + assembly | ~60 |
| **Bare total (parts + fab + assembly)** | **≈ $734** |
| Shipping (JLCPCB + DigiKey/Mouser) | ~55 |
| random bs for safety ~10% | ~58 |
| **All-in for one working board** | **≈ $845** |

aud
