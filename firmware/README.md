# Trajectory speed proof of concept

See the [system and component reference](docs/v2-mini-system-reference.md) for
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
documents are in [trajectory-poc.md](docs/trajectory-poc.md).

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

D32 is a steady power/communication status LED while the firmware runs.
It starts blue and turns green when CRC-checked GNSS navigation messages
arrive, including messages without a position fix. It returns to blue after
two seconds without a valid message or while GNSS is being reconfigured.
The other three LEDs stay off. The indicator runs independently of USB and
uses the proven low-brightness LP5813 configuration, including TI's
recommended short-detection threshold. There is no LED test mode.

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

`cargo build --release --no-default-features` retains the attempt to run the
estimator on core 1 using PSRAM. The board reports 16 MiB PSRAM, but this
variant stalled while copying its workspace into PSRAM at both tested
125 MHz and 250 MHz settings. It has not demonstrated onboard speed output;
the default `host-poc` image avoids that initialization path.
