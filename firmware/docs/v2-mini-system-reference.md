# V2 Mini base system reference

This describes the base image after the 2026-09-10 rebuild. Earlier register
investigations, hardware measurements and experimental firmware behavior are
preserved in [the historical reference](history/poc-system-reference.md).
The frozen [PCB](../../hardware/v2_mini_pcb/v2_mini.kicad_pcb) remains the wiring
authority; all firmware pin assignments are in [board.rs](../src/board.rs).

## Ownership and scheduling

| Owner | Hardware and behavior |
| --- | --- |
| Startup/core 0 | Safe outputs, fixed bus settings, RTOS and PSRAM mapping |
| Core 1 | Full PSRAM test, then SCH16T acquisition on its 16 KiB stack |
| GNSS task/core 0 | Owns both UM980 UARTs; bounded configure/read/retry loops |
| Power task/core 0 | Sole I2C0 owner: charger/gauge, D35, four buttons |
| Display tasks/core 0 | SPI3 panel and separate bounded backlight control |
| Shutdown task/core 0 | MAX16169 interrupt and kill output; independent of USB |
| USB/core 0 | Device pump, independent receiver, single ordered transmitter |

Snapshot locks only copy/update small values. Formatting and I/O occur after
releasing them. Sensor capture uses a fixed 32-entry inline queue (~33 KiB).
The queue preserves producer arrival order, has no allocator and uses nonblocking
send. Capture enable/clear/enqueue are serialized in a short critical section.
No task waits for the trajectory engine or a satellite fix.

## Frozen pin assignments

| Module | Peripheral and GPIO |
| --- | --- |
| Power bus | I2C0 SCL 6, SDA 7; 100 kHz, 10 ms transaction timeout |
| Power latch | Open-drain kill 8, active-low interrupt 38 |
| Charger/indicator | PG 40, LP5813 enable 39 |
| UM980 | UART1 TX 47/RX 46; UART2 TX 49/RX 48; reset 44 released with pull-up |
| SCH16T | SPI2 SCK 12/MOSI 11/MISO 13, CS 10, reset 9, data-ready 14 |
| ST7789 | SPI3 SCK 16/shared SDA 17, CS 18, DC 19, reset 15, TE 5, backlight 4 |
| J6 | Dedicated USB-HS peripheral, application 303A:4001, ROM 303A:0020 |

SPI buses use mode 0 at 1 MHz. LCD transfers yield after at most 64 bytes
(~512 µs), within the GNSS UART service budget. I2C uses HAL asynchronous
transactions: blocking sequences caused reproducible UART capture errors in
the first base image; the same hardware smoke test passed after this change.

## Sensor bring-up

UM980 uses the demonstrated COM1 command / COM2 data path. Configure group 1,
automotive rover, PVTALG AUTO, ANTIJAM AUTO, COM2 at 460800 and BESTNAVA at
20 Hz. Do not save all configuration on every boot. SIGNALGROUP itself is
persistent and may restart the receiver. Initialization confirms VERSIONA CRC
and ten consecutive 50 ms BESTNAVA epoch steps, including no-fix records.
Configuration does not prove every setting was read back. Navigation parsing
checks CRC, solution validity and numeric ranges independently. Three seconds
without a valid record triggers reconfiguration after a three-second backoff.

The second J6 CDC interface supports a separately selected GNSS bridge owner.
GNSSBRIDGE pauses the normal configuration/recovery loop; GNSSNORMAL restores
it. See [USB bridge findings](gnss-usb-bridge.md) for the protocol and the
connected assembly's unresolved COM1 return / COM2 command connections.
Receiver-to-host streaming has been observed, but two-way passthrough has
not passed the live command test.

SCH16T retains the verified SafeSPI sequence, startup self-tests and profile
readback. LPF3/DYN1/DEC5 produces nominally 737.5 native sets/s; four sets form
one averaged interval (about 182 observed intervals/s). CRC, response address,
status, all six sample counters, elapsed time and calibrated ranges are checked.
The entire interval is discarded on failure. Ten consecutive acquisition errors
trigger initialization again with backoff. These are factory-corrected values;
software DRY timestamps do not establish calibrated timing or boresight.

The fitted 16 MiB PSRAM runs at 125 MHz. Only core 1's mapped data region is
granted write permission after checking the observed ROM PMA/PMP layout. Flash
stays read-only and PSRAM non-executable. Two complementary address-dependent
patterns are written across all memory, flushed from cache and verified before
reporting `psram=ready psram-tested=16MiB`. No allocator uses this memory yet.
An unexpected protection map or mismatch is reported without gating other tasks.

## Power and user inputs

Power I2C never uses a generic address scan. The battery monitor recognizes
BQ25628E at 0x6A and BQ25622E at 0x6B by identity, reads only shared status
fields, and reads MAX17048 words individually at 0x36. It does not change
charger policy, touch read-to-clear flags, reset the gauge or issue QuickStart.
The connected board identifies as BQ25622E (0x1A at 0x6B), despite earlier
fitted-part notes. Charger settings remain autonomous.

Battery monitoring runs about every 500 ms. Read errors invalidate telemetry,
the gauge settles for at least one second, and two consecutive observations
confirm a state. Telemetry expires after three seconds. Fractional SOC is used
for low-battery hysteresis (15% sets, 20% clears). A missing gauge on external
power is permitted. Plausible voltage is independent of SOC validity.

LP5813 at 0x50 drives D35, the LED closest to the PCB edge: green charging, blue external power without confirmed
charging, purple battery, red low battery, off unknown. Readback/fault checks
disable its enable pin on failure and retry after three seconds. D32–D34 stay
off. Monitoring does not implement cell protection or automatic low-battery cutoff.

D35 uses group B (B0 green, B1 red, B2 blue): current registers 0x37–0x39,
PWM registers 0x47–0x49, and enable registers 0x20/0x21 = 0x80/0x03.

TCA9536A at 0x40 explicitly configures all pins as inputs with internal pull-ups,
normal polarity and special functions disabled. Reads occur about every 10 ms;
30 ms stable observations debounce transitions. Missing reads invalidate the
pressed mask and trigger one-second retries. Bits 0/1/2/3 map to SW3/SW6/SW4/SW5.
These register choices follow the [TI datasheet](https://www.ti.com/lit/ds/symlink/tca9536.pdf).

SW7/MAX16169 startup pulses are ignored for 100 ms. Once INT is released, the
next falling edge asserts open-drain PWR_KILL_N low until power is removed.
This does not wait for USB. Add a bounded storage flush here before recording
is introduced. Hardware long-press shutdown remains available independently.

## Display and recovery

Startup probes SCLK and checks DC input feedback. A failed SCLK probe leaves
SCK disconnected until reboot. A DC failure leaves DC low and awaits LCDREINIT.
Both faults keep the backlight off and leave all other bring-up running.
The connected board still has DC feedback 4 instead of 5; the prior disconnected
panel check also found this fault. Its physical cause is unresolved.

A healthy panel initializes RGB565 with the Adafruit 240x240 window at rows
80–319, clears RAM, enables display output and observes TE. The base screen is
black; LCDTEST enables color bars and an animated marker. `scanning` confirms
TE progress, not optically correct pixels. The current HAL lacks S31 LEDC support,
so backlight uses bounded GPIO pulses with nominal duty ceilings 0/1/5/10.

For commands and flashing see [README](../README.md). J6 software download uses
the application force-download bit followed by software reset. The host then
uses the RAM stub and the S31 library watchdog reset. This preserves the
previously verified sequence; native USB download modes are described in the
[Espressif boot-mode reference](https://docs.espressif.com/projects/esptool/en/latest/esp32s31/advanced-topics/boot-mode-selection.html).

## Extending the base

Give each new peripheral one owner. Keep I/O asynchronous or bounded below the
UART servicing budget. Put packet validation and state transitions in the
portable library, and test through those interfaces. Keep USB input independent
of storage, display, memory tests and telemetry backpressure. Add new application
processing through explicit data ownership, not feature flags in main. Do not
place stacks/DMA or arbitrary allocations in PSRAM before its ownership and
cache behavior have been designed. SDIO storage, PPS synchronization, wireless,
fusion, calibration and lap timing are not implemented by this base.
