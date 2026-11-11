# Device firmware rebuild

## Scope and ownership

Consume `src/device/hal` as the `aevia-device-hal` path dependency without
changing it. Keep hardware protocols, register layouts and board bus settings
in HAL. Device code owns startup, scheduling, timestamp interpretation,
calibration, estimator inputs, latest-value telemetry and recovery policy.
Keep the recovery loader and existing command indices compatible.

## Execution plan

1. Replace the old SCH16T protocol with the HAL driver. Keep DRDY hardware
   capture, read deadlines, counter continuity and support alignment. Initialize
   on the main executor and transfer exclusive ownership to an interrupt
   executor for sampling; return ownership for recovery.
2. Replace UM980 framing, decoding and command construction with HAL types.
   Keep asynchronous UART transport, verified PPS configuration, independent
   position/velocity epochs, bounded queues and advancing-epoch health checks.
3. Replace charger, LED and panel register implementations with HAL drivers.
   Keep charging disabled for this battery-free bench profile. Add fuel-gauge
   and button diagnostics. Give failed peripherals independent retry schedules.
4. Use HAL's native board initialization and bounded power bus, and its flash
   adapter. Keep unsupported MCU capture and per-core PMA/cache handling in the
   application. Prioritize IMU capture and GNSS reception over blocking UI work.
5. Validate application policy on the host, run the unchanged HAL regression
   suite, compile/link release firmware, inspect image size, and document the
   hardware evaluation gates. Do not flash or claim board qualification from
   compilation or host tests.

## Timing and evaluation requirements

- IMU: read each DEC5 epoch before the next output update; discard ambiguous
  reads and reseed continuity after gaps. No repeated samples to fill gaps.
- GNSS: use receiver measurement epochs mapped to the shared capture counter;
  never substitute UART arrival time for a missing PPS lock. Independent valid
  position and velocity fields must survive an invalid companion field.
- Slow or absent peripherals must not prevent sensor acquisition. Producers
  never wait for consumers. Queue overflows and timing failures are observable.
- Present stale navigation as unavailable. Preserve development calibration
  status until actual mounting, lever arm and filter delays are characterized.

The companion README records build commands and board evaluation procedures.

## Implementation and verification

All five implementation steps are complete. The firmware uses the HAL path
dependency and drivers; native capture/PMA/cache work remains local because HAL
does not implement it. New acquisition diagnostics remain readable independently
of estimator startup. `board.rs` constructs device resources from the HAL wiring
constants. `platform` initializes missing capture/PMA/cache support and supplies
bounded display SPI with peripheral reset on timeout. Rendering and shared-bus
maintenance have separate owners and execution contexts. PPS verification retries
during normal reception; trajectory freshness follows its measurement epoch.
Peripheral and power readbacks expire when their owner stops updating them.
Existing wire layouts and enum indices are preserved.

Verified on the development host after the review fixes: 45 firmware/loader
tests, 174 unchanged HAL source tests, the optimized ESP32-S31 build, and strict
Clippy for firmware (`--no-deps -- -D warnings`). Strict dependency linting also
reaches existing trajectory-crate findings; those are outside this change.
The generated application image is 3,325,472 bytes against the 4,194,304-byte
slot (79.29%). HAL test validation used a temporary consuming manifest to avoid
creating a lockfile or build artifacts inside the HAL folder. Build/test outputs
and the generated image were written under `/tmp`; no device was flashed.

The physical acceptance procedure is in [evaluation.md](evaluation.md).


## Hardware hardening follow-up, 2026-11-10

The user authorized loading and testing the attached board after the initial
review. This follow-up intentionally changes the SCH16T startup/CRC and UM980
age decoder; the earlier "unchanged HAL" scope describes the initial rebuild.
It adds independent GNSS observation validity/age handling, PPS re-verification,
receiver reset escalation, checked cycle-counter SPI timing, fast capture with
periodic validation, separate IMU/GNSS executors, deferred acquisition until
workspace readiness, task-progress supervision and trial-image confirmation.
No calibration data or antenna timing qualification is invented. The connected
board has no display, battery or GNSS antenna; explicit fitment flags disable
unused panel/gauge traffic. Current acceptance procedure remains evaluation.md.
