# Live trajectory experiment

The default `host-poc` firmware streams timestamped UM980 records and
prepared SCH16T-K01 intervals through USB. Core 1 captures the IMU;
core 0 handles GNSS and USB. During bring-up, a host runner fed these into
the real `aevia_trajectory` `EmbeddedLive` forward ESKF and printed corrected
horizontal speed at the IMU centre once per second, approximately 100 ms
behind the latest input. The host tools have since been removed; this
document records the experiment and its results.

The firmware still accepts `STREAM <nonce>`. The board clears its
measurement queue, then replies `STREAM READY <nonce> <time_us>` using
the same USB writer as RAW records. A host consumer should discard
everything before the matching reply before processing the subsequent
stream, so queued USB records cannot seed a new live estimator.

## GNSS profile and the 50 Hz result

The standalone profile uses `CONFIG SIGNALGROUP 1`,
`MODE ROVER AUTOMOTIVE`, `CONFIG PVTALG AUTO`, `CONFIG ANTIJAM AUTO` and
`BESTNAVA COM2 0.05` at 460800 baud, 8N1. `PVTALG AUTO` selects a
single-frequency solution with ionospheric error estimation. Setup sends
complete command lines over COM1 and verifies `VERSIONA COM2` on the data
port, then requires ten consecutive 50 ms steps with valid position and
velocity. It handles COM2 retaining a previous volatile baud rate.

The earlier group-8 experiment measured about 50 records/s with 20 ms
receiver epochs, but only one valid standalone solution/s. Other records
explicitly marked **both position and velocity** `INSUFFICIENT_OBS/NONE`;
their retained numeric values cannot be treated as fresh measurements.
This matches the [Unicore N4 R1.4 manual, page 43](https://en.unicorecomm.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf),
which limits standalone positioning to 1 Hz in group 8 while allowing
50 Hz RTK and RTCM raw observations. No correction source is connected, so
the selected test restores group 1. Live capture measured 20 valid
standalone fixes/s.

Changing `SIGNALGROUP` automatically saves that setting and restarts the
receiver when needed; the remaining settings are volatile. The standalone
helper restores group 1. For future tests with corrections, build from
`firmware/` using `cargo build --release --features rtk-50hz`; this selects
group 8 and 50 Hz messages. A real correction feed and valid solution
statuses are still required to demonstrate 50 Hz RTK. Rebuilding the default
image restores group 1 on startup. The [current N4 R1.14 manual](https://en.unicorecomm.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf)
documents signal groups, port rates and navigation messages. `SINGLE` is
standalone, `NARROW_FLOAT` is RTK float and `NARROW_INT` is RTK fixed.

## IMU profile

SCH16T-K01 uses 1 MHz mode-0 SPI, 48-bit SafeSPI and factory-corrected
20-bit output. DYN1 preserves calibrated ±300°/s gyro and ±80 m/s²
accelerometer ranges. LPF3 Bessel bandwidths are nominally 280 Hz for the
gyro and 240 Hz for acceleration. DEC5 supplies nominally 737.5 native
samples/s; each estimator interval averages four consecutive six-axis sets,
nominally 184.4 intervals/s. The dedicated core-1 capture task delivered
about 182 accepted intervals/s in the live run.

CRC, response address/status, counters, time gaps and calibrated range are
checked. Faults discard the affected interval; actual accepted rates can be
lower than nominal. Startup reads identity and control/status registers.
Settings follow [Murata Doc. 11624 Rev. 6, sections 5–7](../../data/datasheets/sch16t-k01-datasheet-full.pdf).

## Validation

Validation used 10 seconds of warmup followed by 45 measured seconds,
before the host tools were removed.

The completed live capture passed all 45 windows: at least 18 valid GNSS
fixes/s and 160 accepted IMU intervals/s, fresh corrected speed relative to
received MCU input, and advancing GNSS fusion. Actual rates were 20 Hz GNSS
and approximately 182 Hz IMU, with zero estimator resets, rejected IMU/GNSS
observations or wire parsing errors.

All 440 trajectory tests and seven host tests passed. Predictor regressions
cover using nominal velocity for feedback, applying one hard reset
consistently to retained history, rotation/time conventions and validation
before mutation. This checks the data path and regression behavior; it does
not establish speed accuracy against a reference instrument.

## Limits of this test

Timing uses software arrival and DRDY service times without PPS. Residual
IMU calibration, board boresight, antenna lever arm, measured filter delay
and coning/sculling correction have not been established. GNSS ellipsoid
height is reconstructed from MSL height plus geoid undulation. Missing or
stale estimates were displayed as unavailable by the host runner. Its detailed
output included input rates, rejected observations and wire errors alongside
speed. After the startup handshake, freshness used received MCU sample/fusion
times and recent host receipt;
the host and MCU clocks are not absolutely synchronized. Replay should
start from a fresh, continuous capture. Accuracy after long gaps or across
concatenated older recording sessions remains unqualified.

The onboard estimator variant (`--no-default-features`) detects 16 MiB PSRAM
but stalled during its workspace copy at both 125 MHz and 250 MHz. It is
retained for investigation; speed output was demonstrated only with the
now-removed host runner. Reflashes used the USB software bootloader command when the
application enumerated. One loss of both application and ROM USB required
a full power reset through SW7 before software flashing could resume.
