# Race collection candidate: 2026-11-11

Status: r6 charge-current image installed through the normal application updater.
The board is recording Session 8 and reports active charging with verified
800 mA / 4.10 V settings. Recording/retrieval integrity passes; sensor acquisition
still fails release acceptance because raw GNSS reception loses data.

## Current r6 charge-current update

The user requested 800 mA, exactly representable by the charger. Voltage,
precharge, termination, power-path and temperature protections retain the r5
settings. 166 firmware tests, four decoder tests and 31 host bench/board tests
passed; the embedded release build, formatting and package hashes passed.
Session 7 was sealed at 199,715,840 bytes and remains on SD. The normal updater
installed `firmware/build/race-candidate-2026-11-11-r6-charge800/` without ROM
operations or a physical reset. Image SHA256 is
`20e64ab66b3908e283ffaec06872338b27fb853110870f735ae545bfc11a1670`;
source build ID is
`dff74044d01e6806834ea326af1534c22588dd16946363a8665af842a94a376d`.

Live readbacks confirmed charging active with an 800 mA limit and 4100 mV
target. This is not a measured battery current; available input power and
charger regulation can reduce actual current. Gauge-reported voltage was
3.51375 V and the temperature zone was PreWarm (raw 6), with protection enabled.
Session 8 automatically started and advanced from 1,531,392 to 3,151,360 bytes
over 12 seconds with zero recorder drops or SD errors; a fresh 28-satellite fix
was present. Existing raw-GNSS and LED-indication limitations remain.
Evidence is under `firmware/build/hardware-2026-11-11-r6-charge800/`.

## Earlier r5 charge-current update

The user requested 700 mA. BQ25622E has 80 mA charge-current steps, so the
profile rounds down to 640 mA. Voltage, precharge, termination and temperature
protections retain the r4 settings. The package is
`firmware/build/race-candidate-2026-11-11-r5-charge640/`; image SHA256 is
`0d858558092a4942f25fe31fd4211e31119c3c0359da252c38bc432faf299859`,
source build ID
`5b369acab8fd8077d6cd9eff41774dcb83f59877ee3dbc7566e6fbd6f07d8932`.
166 firmware tests, four decoder tests and 31 host bench/board tests passed;
embedded build and formatting passed. Session 6 was sealed at 62,825,984 bytes
before the update and remains on SD. Application-mode installation succeeded
without ROM operations or a physical reset.

The live check confirmed 640 mA / 4100 mV readbacks, charging enabled and active,
and gauge-reported voltage of 3.395 V. Temperature status was PreWarm (raw 6);
temperature protections remain enabled. This is a programmed limit, not a
measured charge current. Session 7 advanced from 2,086,400 to 3,695,616 bytes
over 12 seconds with zero recorder drops or SD errors. A fresh 28-satellite fix
was present. Existing GNSS UART/raw-observation limitations remain. Evidence is
under `firmware/build/hardware-2026-11-11-r5-charge640/`.

## Earlier r4 hardware result

The installed package is `firmware/build/race-candidate-2026-11-11-r4-battery/`.
Image SHA256 is
`6df66b119bdf2c625f59d080defa711fdfe42b2c81a90d5a6e1c18e35195eb9f`;
source build ID is
`0818bf85cf096c0ec1ae44f96effdbb85f3b18c8622f9d376efe308753d36973`.
The normal updater completed and the application returned on COM3 without ROM
commands or physical reset. Automated firmware, HAL, decoder and host-tool
checks passed, as did the embedded release build, strict Clippy and formatting.

Session 5 sealed and downloaded 14,317,056 bytes with every page/record CRC
valid, no missing sequences or recorder UART-byte offsets, and zero queue drops.
Its SHA256 is
`77c45c6170571233777315507d1eed23572646d6997534492bff9a6284ffe930`.
Native IMU cadence was approximately 726.8 Hz. Two early timing stalls and
472 scaled-invalid samples remain explicitly represented in the evidence.
GNSS acquired a position fix, but UART/CRC failures left zero valid raw OBSVM
frames in this recording. PPK-quality collection and race readiness are unproven.

Recording was restarted as Session 6 after retrieval. The final live check
observed valid bytes advancing from 4,869,632 to 5,275,136 over three seconds,
with zero recording drops or SD I/O errors and a fresh 28-satellite fix.
The charger reported active charging, verified 400 mA / 4.10 V limits,
80 mA precharge and 40 mA termination, and no current charger fault flags.
Gauge-reported voltage rose from approximately 2.91 V to 3.29 V during testing.
This is a configured current limit, not an independently measured charge current.
Earlier warm thermistor status cleared; temperature protection remains enabled.
Gauge SOC is implausible and must not be treated as a usable percentage.
Battery-only operation and USB handover have not been physically qualified.

Evidence is under `firmware/build/hardware-2026-11-11-r4-battery/`, particularly
`install-02.json`, `live-01.samples.json`, `initial-download.json` and
`final-running.json`. The firmware was left recording and charging.

LED mapping in this image: D32 blue means the application has not entered Failed
(red means Failed); D35 green means Recording/Navigating, blue means another
startup/ready phase and red means Degraded/Failed. D33 green means recent GNSS
message activity, red means none within one second. D34 green means an IMU sample
within 100 ms, red means stale. These LEDs do not certify fix quality, raw GNSS
integrity or battery charging state.

## Earlier r3a and r2 evidence

The earlier r2 image recorded to the physical SD card but failed acquisition
checks, then disappeared from USB while stopped and idle. The user subsequently
restored ROM mode. The bounded r3a installation preserved the bootloader,
partition table and r2 fallback image. A deliberate ROM launch was acknowledged,
but no application USB interface appeared; COM5 remained listed and a subsequent
read-only probe hung in Windows port opening before SYNC. The probe processes
were stopped. A normal physical reset without holding BOOT restored application
USB on COM3. The earlier ROM handover/USB disappearance cause remains unproven.

## Implemented behavior

- Automatic button-free recording, independent of a USB data host, satellite
  fix, PPS lock and estimator initialization. CPU1 owns storage; expensive live
  trajectory processing is disabled in this build.
  A single-cell 2000 mAh, 3.7 V battery was subsequently fitted. The r4 build
  enables gauge monitoring and charging; battery-only operation and handover
  need testing.
- SCH16T native six-axis values, per-axis counters/status, timing evidence,
  scaled values when valid, temperature, verified setup and faults. Raw UART
  streams include UM980 PVT at 50 Hz, requested raw observations at 1 Hz,
  ephemerides, receiver version/configuration and actual capability evidence.
- Original PPS captures and all available configuration/reset evidence. Files
  contain a deterministic source SHA256 and preserve explicit queue/transport
  gaps. Calibration and mounting remain unqualified.
- Native four-bit SDMMC with existing exFAT volumes, contiguous file
  preallocation, 2 GiB rotation, a 4 MiB PSRAM queue and 250 ms partial-page
  flush interval. SD write stalls can add queue latency; the flush interval is
  not a promise of maximum data loss after sudden power removal.
- Page and record CRC32, sequence numbers, per-file nonce and grouping across
  rotations. Ordinary streaming does not modify filesystem metadata. Interrupted
  allocation may leak reserved space and leaves the volume read-only until
  repaired; independently valid earlier recordings remain retrievable.
- USB status/start/stop/list/read, bounded request IDs and replay protection,
  resumable downloads and offline exports. Reads require recording to be stopped;
  Stop drains pending capture before sealing. Recovery/update is refused while
  recording. Firmware trial confirmation requires actual IMU records written to
  SD and healthy acquisition progress.

## Scope and limits

The card backend supports exFAT with 512-byte sectors, one FAT, and an MBR or
superfloppy layout. FAT32, GPT and TexFAT are rejected without formatting or
writing. Card capacity alone does not prove a supported filesystem. The firmware
does not delete recordings or reclaim unused file preallocation. It stops with
an explicit error when allocation cannot continue; it never overwrites an old
race. A stopped short session can reserve substantially more space than its
recorded prefix. Consumer SD media cannot guarantee power-loss atomicity.

USB transfers use the existing application CDC interface, not a Windows disk
drive. A Python CLI and matching decoder are supplied; the mobile app is not
required. Downloaded files retain native GNSS bytes for an external offline
processor, but a successful recording does not establish that a PPK solution is
possible. Raw-observation and ephemeris coverage is checked separately.

## Automated evidence

The r3a source passed 407 automated tests: 147 firmware library tests,
190 sensor HAL tests, 3 matching decoder tests and 67 Python host-tool tests.
The optimized embedded build, strict embedded Clippy check and firmware/decoder
format checks passed. Filesystem tests include a sparse 128 GB volume, 2 GiB
allocation, interrupted metadata writes and preservation of earlier recordings.
These filesystem tests use simulated storage; physical evidence is recorded
separately below.

USB recording reads now negotiate blocks up to 4096 bytes inside a maximum
4608-byte response frame. A mock transfer through the real host client and
persistent decoder for r2 measured approximately 1.71 MB/s, compared with
0.26 MB/s for 256-byte blocks. This measures host-tool capacity only: it excludes physical
USB, the serial driver, SD reads and firmware scheduling. Physical r2 retrieval
was substantially slower, as recorded below. Downloads verify checksums and
support resuming an interrupted transfer; preflight requires advancing IMU
samples and GNSS receive traffic.

Inspection of the r2 compiled image found a 33,280-byte recording control object,
a 10,800-byte recording-worker stack frame and a 64 KiB reserved CPU1 stack.
This is a static inspection of selected functions, not a complete nested stack
bound or a physical stack high-water measurement. The associated benchmark and
memory reports identify that packaged decoder/image's hashes. They are not a
physical stack measurement or a new memory audit of r3a.

## Observed hardware evidence and corrective image

The installed r2 image mounted the existing exFAT card and reported
127,865,454,592 bytes of capacity. It automatically recorded, stopped and sealed
an 11,446,784-byte file. USB retrieval and offline verification checked every
page and record CRC, with no recording-queue drops or missing record sequences.
The downloaded metadata matched r2's source build ID. This establishes a working
initial recording/retrieval path, not complete sensor acquisition.

The same approximately 90-second recording reported 7,759 IMU errors,
285 GNSS UART errors and 302 parser errors; 7,758 native IMU records carried an
invalid timing flag. The settled preflight failed. Independent inspection of
the captured UART bytes found CRC-valid raw observation and ephemeris frames,
while receiver reconfiguration had cleared the reported coverage counters.
The evidence is retained under `firmware/build/hardware-2026-11-11-r2/`, including
`initial-download.json`, `settled-preflight.json` and `initial-imu-analysis.json`.

A physical 524,288-byte r2 download profile measured approximately 132 kB/s.
Skipping the host serial flush measured approximately 130 kB/s with the same
downloaded hash, so that experiment did not demonstrate a host-flush bottleneck.
See `usb-live-profile.json`. Sustained r3a transfer throughput is pending.

The r3a changes preserve the wire/log schema and the 600 microsecond completed
IMU read deadline:

- Capture the completed CRC/status-validated native SPI read timestamp before
  software floating-point conversion. Collection retains native sensor axes;
  it does not run navigation's gyro-to-accelerometer interpolation.
- Use a table-based N4 CRC and bounded GNSS receive bursts. Steady-state CONFIG
  and raw-log commands enqueue into COM1 without waiting for transmission,
  allowing COM2 reception to continue. Baud probing/setup still flushes pending
  transmission. Receiver baud rates and requested log rates are unchanged.
- Preserve coverage counters across receiver reconfiguration within a file;
  reset them for a new file generation and exclude partial frames crossing that
  boundary from new-file coverage credit. Original UART bytes remain recorded.
- Read aligned 4 KiB download blocks with eight-sector SD reads, reducing
  per-sector command overhead. Physical throughput improvement remains to be
  measured.

The corrective image is `firmware/build/race-hardware-r3a/firmware.bin`,
423,120 bytes, with SHA256
`ebcdaa17c9c7235c1c8db519bd916f7aa2d6e8d8a5ffca1af85d54241c2d572a`.
Its source build ID is
`f7908fe5ded9aba942cc61cb464af18bc8ca069bf6bbceff585c0480f0abfbe7`.
The matching decoder and image identity are retained beside it. Passing the
automated checks does not establish that these changes resolve the physical
failures. The empty r3a `install.json` records the earlier failed connection
attempt; the later ROM recovery installation is recorded separately below.

The ROM recovery evidence is retained under
`firmware/build/rom-recovery-2026-11-11/`. Before writing, verified backups covered
`0x0..0xc000` and the affected slot0 range `0x10000..0x78000`. The pre-install
state was `APP1` with valid selectors 29 and 30, selecting slot1; no unfinished
trial was present in that snapshot. `install.log` records ROM MD5 plus complete
byte readback of the padded r3a app, `RUN1` mode sector and new sequence 31 trial
selector. The bootloader, partition table and confirmed slot1 selector remained
unchanged. The complete 4 MiB slot1 hash matched before and after installation;
its image prefix matched r2. The SD card was not addressed by the ROM writer.

`launch.log` records a fresh verification of the installed app, boot regions,
fallback and mode/selector bytes, followed by an acknowledged zero-size
FLASH_BEGIN and FLASH_END(1). That acknowledgement proves receipt of the launch
command, not a successful application startup. COM5 (`303A:0020`) remained
listed, and `after-launch-probe.log` records the later port-opening failure.
The normal physical reset supplied the application evidence below. The
tooling faults and inconclusive power review are detailed in
[the recovery incident notes](../build/rom-recovery-2026-11-11/incident-notes.md).

The r3a normal-boot results are under
`firmware/build/hardware-2026-11-11-r3-rom/`. Session 2 sealed and downloaded
13,927,936 bytes, passing every page and record CRC with contiguous record and
UART-byte sequences. Recorded source identity matches the installed r3a build.
Thirty-one aligned, unaligned, cross-sector and tail reads of the earlier r2
session matched its saved download. USB remained available through these checks.

The 60-second recording gate still failed: accepted IMU cadence was 652.69 Hz,
timing rejects 38.12/s, and COM2 receive errors approximately 1.88/s. Native
evidence contained 74,481 valid six-axis frames at 726.77 Hz, with 25 genuinely
missing updates concentrated in two startup stalls. Most telemetry rejects
were caused by the conservative 600us admission bound. Raw GNSS evidence showed
69 recognizable OBSVM frames failing CRC and only two intact advancing FINE
epochs; contiguous recorder UART offsets locate this loss before persistence.

With recording stopped, a 20.64-second comparison measured 726.63 accepted IMU
Hz, only three timing rejects and two COM2 FIFO events. This associates much
of the remaining overhead with recording. The subsequent r4 changes move
PSRAM record copies outside the global interrupt-masking lock, avoid unused
navigation construction, and apply a collection-only 1ms absolute IMU bound
with all existing integrity/counter checks. Their physical results are recorded
at the top of this document; GNSS reception still fails acceptance.

## Required physical acceptance

1. Repeat normal boot and provenance checks for the next corrective image.
   The r3a boot, source identity, SD mount and earlier-file comparison passed.
2. Verify auto-start, measured IMU/GNSS rates, actual advancing raw observations,
   ephemeris receipt, capture timestamps and zero unexplained queue/UART losses.
3. Record for at least 30 minutes on the physical card, measuring sustained
   writes and acquisition headroom. Exercise an accelerated rotation fixture
   before relying on long races.
4. Stop, download through USB, verify every page/record CRC and sensor/transport
   continuity, export and inspect the actual measurements. Measure throughput.
5. Exercise interrupted/resumed USB download, disconnected-host recording,
   explicit restart, recover/run and unexpected-reset/power-loss retrieval.
   Ensure previously completed recordings remain intact.

Both r2 and r3a recording/retrieval passed integrity checks, while sensor
acquisition failed acceptance. The remaining gates above must pass before
claiming race readiness.
