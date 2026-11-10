# V2 Mini firmware evaluation

The rebuild integrates the HAL and includes receiver, IMU startup and transfer
recovery fixes established during connected-board evaluation. Host validation and a
linked image establish software readiness for bench evaluation. Hardware timing,
receiver return paths, calibration and battery characterization remain physical
acceptance gates. The firmware deliberately reports unqualified measurements.

## Start and observe

1. Build using the firmware README and flash through the existing loader/slot
   workflow. Use J6 USB-HS with DTR asserted. New installations enter recovery;
   send Run (`01 02 02 00`) and reconnect after reboot.
2. Read BootStatus (`01 02 07 00`); stage 10 means estimator startup completed.
   Stage 11 reports PSRAM verification failure. A failed estimator must leave
   USB and acquisition diagnostics accessible.
3. Poll Status (`02 01 01 00`), AcquisitionStatus (`03 01 05 00`),
   PeripheralStatus (`03 01 04 00`) and PowerStatus (`03 01 03 00`). Responses
   are the typed Postcard/COBS envelopes in `device/interface.rs`. USB packet
   boundaries are not frame boundaries. Existing command indices are preserved.
4. Confirm power verification, LED colors and all four button bits. Check panel
   orientation and gauge availability only when those accessories are fitted.
   Buttons are diagnostic-only. No thermistor
   curve, charging profile or gauge battery model is guessed by the application.

## Sustained acquisition

After stage 10, record counter baselines and run for at least 30 minutes, first
stationary and then through representative motion. Poll USB throughout. Acquisition
starts after worker memory/workspace initialization. Flash confirmation may still
interrupt startup acquisition; compare steady-state counter deltas after settling,
not only totals since reset. A display-equipped variant needs a separate run with
the display enabled and disabled.

| Check | Evaluation gate |
| --- | --- |
| IMU cadence | Raw admitted rate near the configured DEC4 rate (~1.475 kHz; about 1.454 kHz on this board); investigate persistent reductions. Raw counts are separate from estimator acceptance. |
| Read timing | Captured-edge-to-completed-read latency below 600 us in normal operation, verified with a logic analyser. Late bursts increase timing errors and never enter the estimator. |
| Overload | No new steady-state drops under expected workload; queue high-water observations leave capacity margin (IMU 128, GNSS 16). |
| GNSS epochs | Advancing trusted rate reported honestly. Cold start without FINE time must not restart the receiver repeatedly. |
| PPS | Verify physical rising edges, 1 Hz rate and full CONFIG PPS readback. Lock, residual and age must remain healthy before fusion. |
| Estimator | With antenna reception and an accepted position/velocity seed: stage 10, advancing accepted/fused counts and trajectory epoch; record maximum_step_us, processing errors and late observations. Before that seed, raw IMUs drain while imu_accepted remains zero. |
| Resources | Memory check passes, no unexpected recovery resets, USB queryable and independent peripheral errors. |

The capture clock uses one hardware counter for PPS/DRDY. Measure actual filter
delays and alignment before narrowing uncertainty. Set mounting, calibration and
antenna lever arm in `profile/configuration.rs` before assessing measurement accuracy. The
receiver CONFIG readback must succeed independently of navigation reception;
unverified PPS configuration keeps fusion disabled. Readback has succeeded on
the connected board, including after the initial query timed out.

## Fault and recovery runs

- Interrupt GNSS delivery and stop advancing FINE epochs while traffic continues.
  Expect observable errors, bounded retry and receiver reconfiguration. Withhold
  PPS independently: lock must expire without arrival-time substitution.
- Drop the first CONFIG readback while retaining advancing navigation. Confirm
  raw counts/rates keep advancing, CONFIG retries after the response window and
  backoff, and a subsequent valid readback permits normal PPS lock/fusion.
  Wrong/malformed readback must never enable fusion. Fault the command UART
  independently and confirm it cannot starve or reconfigure healthy COM2.
- Interrupt IMU DRDY or communication. Expect errors, transfer to the initializer
  and recovery without repeated samples. Gaps must remain visible to processing.
- Exercise missing/faulted low-rate peripherals in a suitable bench setup.
  Acquisition must continue. LED/panel retries are independent; stale peripheral
  readings and power verification expire after three seconds. Exercise a stuck
  SPI UPDATE/USR completion: each FIFO operation must time out within 2 ms of
  CPU execution plus acquisition/maintenance preemption, reset SPI3, release CS,
  blank the panel and retry initialization after three seconds. Power readbacks
  must continue through repeated display resets and failed transfers. USB
  command latency can increase during main-executor initialization. Confirm
  IMU latency remains below its deadline with simultaneous GNSS traffic and
  missing/faulted power-bus devices.
- Keep input arrivals/publications advancing while rejecting estimator input or
  holding the retained trajectory epoch fixed. Speed/position/velocity must
  expire after the profile's 320 ms epoch-age budget, with degraded phase.
  Normal 120 ms fusion/smoothing latency and publication jitter must stay usable.
  Stop publication separately and confirm output expires. Request Recover with the worker
  live, verify loader access and the bench charging policy, then Run back into
  the application. Flash safety host regressions cover torn writes and boot
  selection, not electrical faults.

SD session logging and radio stacks are outside this acquisition profile. They
need their own application behavior and throughput evaluation.


## Connected setup

The current board has no display, battery or GNSS antenna. Panel rendering and
battery-gauge polling are disabled by explicit fitted-accessory flags in board.rs.
Panel readiness and battery values remain unavailable. LED/button/charger health
are still observed. No antenna means trusted GNSS time, PPS lock and navigation
output are expected to remain unavailable; verified CONFIG readback can still be
tested independently with GnssStatus. Do not count those missing signals as a
failed cold-start recovery test, or claim an antenna-free run validates fusion.

Use tools/board_check.py with the host decoder to retain typed responses and raw
frames. ResetStatus is readable in recovery and distinguishes watchdog reset
from a retained Rust panic. ImuDiagnostics separates driver faults, late starts,
late finishes, overwritten captures and SPI timeout phase/context. Initial flash
confirmation can pause acquisition; compare steady-state deltas after settling.
The firmware still enforces the 600us completed-read deadline and reports
unqualified measurements until installation/calibration evidence is available.
An antenna-free run exercises raw acquisition and queue draining, but does not
exercise normal estimator stepping. Navigating throughput, PPS timing and accuracy
remain separate acceptance gates.

## Release decision

This build is suitable for continued bench evaluation. It is not a qualified
measurement release. Completing the software hardening does not remove these
physical and deployment gates:

| Remaining gate | Quality or availability consequence | Required evidence |
| --- | --- | --- |
| Antenna-backed PPS and GNSS | Wrong pulse association or epoch semantics can bias fusion; loss/reacquisition must remain observable. | Physical pulse/DRDY timing, valid first seed, advancing fused epochs, delayed delivery and signal-loss tests. |
| Navigation workload on S31 | Raw acquisition before a seed avoids normal estimator work, so an antenna-free soak cannot establish processing capacity. | Sustained IMU acceptance, 50 Hz GNSS fusion and 200 Hz frontier progress without drops or growing lag. The host quota regression proves work-credit semantics only. |
| Installation and sensor timing | Nominal mounting, zero lever arm, filter delays, noise and correlation assumptions can bias estimates or understate uncertainty. The development timing approximation does not propagate independent timing uncertainty. | Measured installation/calibration, uncertainty coverage and a supported qualified timing path; retain qualified=false until then. |
| Rollback before Rust startup | A candidate that fails before application rollback runs can prevent automatic return to the healthy image. | Provision and fault-test a verified rollback-enabled bootloader; application fallback alone covers later failures. |
| Motion envelope and endurance | Saturation or excessive dynamics remove real measurement support; stationary acquisition does not establish field behavior. | Representative motion within the sensor envelope, longer stationary/moving runs and electrical interruption tests. |

The current profile excludes session recording, radios, display and battery
operation. These are separate product variants/features, not validated behavior
of this connected setup. Trial confirmation establishes functional acquisition
and task progress; it is not a navigation accuracy attestation.
