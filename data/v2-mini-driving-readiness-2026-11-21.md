# V2 Mini driving validation, 21 November 2026

The stationary V2 Mini ran the default release profile with development navigation enabled, native IMU capture, 50 Hz GNSS, raw PPK logging, and the 5 Hz fused dashboard. The release application image is 1,224,080 bytes. SHA-256 is `1a30809889032ceecc5fd46b9b20d7f726c804e52098504dd8d1749dbaad90b0`. The navigation processor occupies 5,280 bytes in the linked image.

## Stationary board

The same Mac Bluetooth client read the dashboard for 20 seconds while idle, started a new SD session, polled at 5 Hz for 60 seconds, then stopped and disconnected. USB supplied power and carried full status reads roughly once per second and charger ADC requests every three seconds. Sensor rates use the first and last Bluetooth frame. USB input means use successful charger readings during recording. Charger input includes the board and its power circuitry. Battery discharge runtime requires a battery-powered measurement.

| Bluetooth recording | Pre-flash baseline, file 78 | Final release, file 84 |
| --- | ---: | ---: |
| Frames with fused motion | 300 of 300 | 300 of 300 |
| Dashboard / IMU / GNSS rate (Hz) | 5.00 / 726.91 / 50.00 | 5.00 / 726.73 / 50.04 |
| Oldest motion epoch at read | 233 ms | 220 ms |
| Navigation / SD sensor / SD raw-chunk queue drops | 0 / 0 / 0 | 0 / 0 / 0 |
| Short IMU intervals bridged during sampled recording window | 1 | 6 |
| Raw PPK frames truncated during sampled recording window | 5 | 8 |
| Mean USB input power | 1.652 W | 1.654 W |
| Successful power requests | 19 of 20 | 19 of 20 |

Both files closed successfully. One baseline harness timeout occurred after Stop and disconnect. On the final image, launching the Bluetooth client directly completed the full run after two host launches timed out before discovery. These measurements establish comparable plugged-in recording power. They have insufficient precision and duration to establish a battery-life improvement.

All 13,789 pages and individual record checksums passed on readback of file 84. It held 43,667 native IMU intervals, 3,004 GNSS records, 298 fused snapshots and a terminal summary over 60.086 s. The app imported all 298 snapshots. Stationary distance was 0.069 mm, with zero laps and drag runs. The raw intervals had six gaps totaling 16.5 ms. Terminal metadata reported nine physical UART errors and nine rejected raw read chunks (117 known lost bytes) upstream of the SD queues. The file remains usable for dashboard analysis. Its raw observations are incomplete.

Two trial runs used a 32-byte raw UART watermark. One SD file lost 1,994 native sensor records during a roughly five-second drop in write throughput. The storage worker normally completed about 227 pages/s and fell to 77–98 pages/s. Its 384-sample queue filled. Separate navigation and raw PPK storage queues stayed clear. The repeat on the same image closed with zero SD drops. Both runs delivered 300 fused Bluetooth frames. The lower watermark gave inconsistent raw UART results, so the release retains the existing 64-byte COM1 setting.

Readback verified all 13,114 page CRCs and the individual record CRCs in the affected file (ID 79). It held 41,752 native IMU intervals, 2,920 GNSS records and the terminal drop count of 1,994. The app imported the file with 294 valid fused points and displayed the loss count once. The SD counters identify where the loss occurred. They do not distinguish card programming time from delays in servicing the card.

On the final release, a 10.20 s configured run (ID 81) verified a committed gate, 60 km/h drag target, rollout and signed vehicle acceleration. All 2,350 SD pages passed CRC and sequence checks. The file held 7,416 IMU intervals, 510 GNSS records, 380 raw GNSS chunks, 51 periodic dashboard snapshots and a terminal summary. The app recovered the setup and reported 6.6 mm of stationary distance with zero laps and launches. A separate 5.23 s raw USB recording delivered 3,801 IMU intervals and 262 GNSS records in timestamp order, with zero USB and SD queue drops.

A 60.07 s USB dashboard recording (ID 83) delivered 300 frames. Motion was available in 265. During the interruption the receiver reported horizontal uncertainty above 33 m, the filter rejected fixes, and motion disappeared. Fused output resumed when uncertainty improved. SD and navigation queue drops stayed at zero. The raw PPK port recorded three physical UART errors and three truncated frames. The charger input averaged 1.67 W from 20 successful readings. A second client read 26 fused frames over Bluetooth during this recording.

## Changes exercised

- Continuous IMU support preserves tilt when GNSS goes stale before yaw alignment. New fixes restore speed, and later horizontal acceleration can establish yaw.
- The GNSS adapter passes the same ECEF position, velocity, timestamps and scalar uncertainties to Traj while skipping two unused double-precision covariance rotations per fix.
- Stop retains the finished run's lap and sector state and propagates SD flush faults through USB and BLE. The USB handler holds the finished session through reply construction. Navigation-disabled builds skip the estimator queues and completion waits.
- Requested charger measurements sleep through the expected conversion time before polling. The charger keeps its 12-bit resolution. [TI charger datasheet, ADC timing](https://www.ti.com/lit/ds/symlink/bq25622e.pdf)
- The app computes lap statistics from complete lap reports, retains the board's best when snapshots are missing, and surfaces terminal SD queue losses on import and reanalysis.

## Replay and regression checks

| Exercise | Result |
| --- | --- |
| Two stationary SD replays, approximately 300 s and 10 s | Accepted all 15,512 fixes and 56,377 four-sample IMU intervals. Fused output stayed available after alignment. Integrated distance was 0.71 mm and 0 m, respectively. |
| Two synthetic 14.86 km drives | With 60 ms and 220 ms velocity latency, maximum speed error was 0.011 and 0.078 m/s. The filter withheld stale output and resumed after a 2.4 s GNSS outage. |
| Synthetic 300 s curved drive | After warm-up the 95th-percentile errors were 0.033 m/s in speed, 0.0020 g in lateral acceleration, and 0.156° in course. These errors describe the simulation. |
| Regression suites | 458 Traj tests with offline features, 308 with default features disabled, 158 firmware host tests, 24 standalone sensor tests, 30 Python firmware-tool tests, 119 app Rust tests, and 26 Flutter tests passed. The suites overlap. |
| Embedded release | Both default and no-default-feature binaries built. Release firmware Clippy passed with warnings denied. |

Flutter analysis reported 14 existing Material import deprecation notices, with no errors or warnings. The workspace-wide Rust format check still reports existing differences in `firmware/src/response.rs`, `traj/src/engine.rs` and `traj/src/trajectory/tests/storage.rs`.

The board remained stationary throughout. Moving gate and drag timing, installed vehicle axes and real outage recovery still require comparison against an independent reference on a drive.
