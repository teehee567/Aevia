> Historical experiment. The base rebuild removed the `host-poc`, `rtk-50hz`
> and implicit onboard-estimator build modes. The source adapter is preserved
> in [experiments](../experiments/README.md); use [the firmware README](../README.md)
> for current commands. Statements below describe the earlier images.

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

## Onboard PSRAM fix (2026-09-10)

The original onboard variant (`--no-default-features`) detected 16 MiB PSRAM
but stayed at startup stage 2 during the workspace copy at both 125 MHz and
250 MHz. The hardware checker reproduced that failure. A single-word probe
then showed that reads worked and the first write stopped the estimator.
Temporary panic telemetry identified `mcause=0x38000007` (store access fault),
`mtval=0x50000000`, with no PSRAM-controller address or permission error.

Core 1's ROM PMA15 was `cfg=0xc0000015`, `addr=0x13ffffff`: the entire
`0x40000000..0x60000000` external-memory region was read/execute-only.
Its PMP entries were disabled. The pinned HAL initializes the PSRAM
controller and MMU but leaves those per-core ROM attributes unchanged.

[psram.rs](../src/memory.rs) checks that ROM configuration and splits the
region into read-only flash, a non-executable read/write window for the fitted
16 MiB PSRAM, and the remaining read-only address range. PMA8..11 are reserved
for the split; other ROM entries remain intact. Configuration readback must
match before any stores. Attribute definitions follow Espressif's
[RISC-V CSR definitions](https://github.com/espressif/esp-idf/blob/master/components/riscv/include/riscv/csr.h)
and [S31 region setup](https://github.com/espressif/esp-idf/blob/master/components/esp_hw_support/port/esp32s31/cpu_region_protect.c).

The board passed two complementary address-dependent patterns across all
16 MiB at 125 MHz, with cache writeback and invalidation before verification.
The real trajectory workspace then initialized to stage 8 and IMU ingestion
advanced. Three host tests cover the region boundaries, permission split and
rejection of unexpected configurations. Sensor capture waits until startup
completes to avoid filling its queue during the memory test. Run
`powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-psram.ps1`
from the repository root to repeat the memory/startup/IMU-progress check.

The final image passed a 30-second check and a second eight-second check
after a software restart, with zero queue drops and estimator errors.
IMU ingestion advanced at approximately 175 intervals/s. Sensor capture
errors remained present (about nine/s), as in the baseline; they are not
PSRAM-test failures and still need separate acquisition investigation.

GNSS had no valid fix during this test; it does not establish onboard fusion,
speed accuracy or loaded-estimator timing. Speed output has previously been
demonstrated only with the now-removed host runner. The default firmware still
selects `host-poc`, which does not run the estimator or initialize PSRAM.

Application USB initially failed enumeration. Holding SW2 while resetting
with SW1 recovered ROM USB on COM4. Subsequent flashes used the USB software
bootloader command and the documented RAM-stub/reset-helper sequence. The
large onboard ELF repeatedly timed out on its first FlashDeflData transfer;
a subsequent flash/check completed successfully. This flashing-tool issue
is separate from the CPU protection fault.

Later the same day, Windows recorded 37 application USB surprise removals
between 13:53:14 and 13:54:28 local time, after the earlier validation had
finished. The board then disappeared from enumeration. Manual ROM recovery
restored COM4, which stayed connected for a 40-second observation. Starting
the existing image without reflashing passed a 40-second serial capture,
a 50-second observation with the reader closed, and a 15-second reattachment
capture. Uptime advanced to 166.768 seconds without restarting; PSRAM remained
tested at 16 MiB, startup reached stage 8, and estimator errors stayed zero.
The reset recovered USB, but the cause of the intermittent disconnect burst
is unresolved. These short tests do not establish long-term USB stability.
