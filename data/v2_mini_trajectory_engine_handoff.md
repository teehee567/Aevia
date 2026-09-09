# V2 Mini trajectory engine handoff

Last updated: 2026-09-05

## Goal and scope

Deliver only the core trajectory and measurement engine: generic prepared
IMU/GNSS inputs, estimation and fusion, time/frame/uncertainty semantics, bounded
live execution, offline host replay/smoothing, trajectory queries and metrics.
The original implementation plan contains broader system work that is no longer
part of this deliverable.

The embedded target is **ESP32-S31-WROOM-3-N16R16V**. The MCU uses the
allocator-free live ESKF and incremental metric path. Offline full-session
`f64` refinement runs off-device. GTSAM is optional workstation-only work and
must never become a firmware dependency or a requirement for recording, live
navigation, display, or live metric finalization.

Only `traj/` and its trajectory-specific workspace wiring are in scope. **Do
not edit, restore, delete, format, or otherwise modify `firmware/`; its dirty
changes belong to the user.**

## Implemented so far

- `traj/` now contains only `aevia-trajectory`, with an allocator-free/no-std
  live core and optional computer-side processing. The acquisition and log
  packages and their workspace/lockfile entries have been removed.
- Added semantic IDs, checked time/support types, frames, reference points,
  covariance validation, quality/provenance, configuration validation, engine
  lifecycle, trajectory queries, live workspace separation, and typed failure
  handling.
- Implemented the embedded live path: initialization, fixed-capacity delayed
  scheduler, support-aware preintegration, 15-state right-multiplicative ESKF,
  GNSS updates, short-gap modeling, dense history, predictor, re-anchoring,
  work quotas, and SRAM/PSRAM compile-time guards.
- Implemented the offline solution-level smoother, replay/source/sink ports,
  multiple processing attempts, dense output, event projection, and optional
  fail-closed GTSAM feature seams.
- Implemented continuous trajectory and metric evaluation for laps/sectors,
  drag/acceleration/braking, and activity outputs, including bounded live
  mutation ledgers and host evaluation.
- Enforced `AttachmentModel::DeviceTrajectoryOnly` centrally. Rigid body-point
  and body-axis claims fail closed; package/IMU/antenna trajectories remain
  available. Metric ambiguity or unsupported semantics now produce a typed
  per-definition `MetricResultValue::Unavailable` without deleting unrelated
  valid results.
- Kept engine-owned semantic replay/result ports, transactional result output
  and canonical evidence-selection digests. Physical recording formats, framing
  and byte decoding are external responsibilities.
- Corrected several GNSS lever-arm/timing details in both live and offline
  paths:
  - measurement-epoch rate/force ownership;
  - right-owned exact IMU/GNSS boundaries;
  - antenna velocity attitude Jacobian including Earth rate;
  - tangential acceleration from support-centred angular acceleration;
  - clock offset/drift and active delay covariance in timing uncertainty;
  - initialization antenna-to-IMU mean/covariance/consider transforms.
- The offline solver now retains an interval-average IMU sample as one
  correlated latent across scheduler/GNSS splits instead of reinjecting its
  covariance independently on every slice.
- Completed the live held-sample latent implementation: preintegration,
  propagation, GNSS updates, initial antenna-to-IMU correlation, exact boundary
  ownership, short-gap continuity, re-anchoring, and workspace reset.

## Current state — core implementation checked

Processing names now distinguish **live** estimation on the S31 from
**offline** replay/refinement on a computer or phone. The offline path uses
the `offline` Cargo feature, `aevia_trajectory::offline`,
`ProcessingLevel::OfflineSmooth`,
`OfflineResourceLimits`, and `OfflineRun` / `OfflineRunSummary`. Deferred live
observations use `InputDisposition::RetainedForOffline`. The default build
remains the allocator-free live engine. The prepared-input API is intentionally
breaking; the removed sensor/log adapter APIs have no compatibility aliases.

The ESKF and both processing paths compile and pass the core library tests. The latest work deliberately follows the user's narrower scope: core
navigation, trajectory mathematics, and focused verification. It does not
integrate the engine into consumers or qualify firmware, acquisition, artifact
adapters, GTSAM, or hardware resources.

Core mathematical work retained:

- Sample-aware GNSS innovation, navigation covariance, and retained sample-cross
  updates, with transactional commit through covariance conditioning.
- Live propagation retains one sample across scheduler cuts. GNSS at a shared
  boundary activates the right-owned sample; a short gap retains its held
  predecessor. Initialization uses the negative velocity/sample cross required
  by true-minus-nominal navigation errors and observed-minus-true sample errors.
- Re-anchoring rotates the navigation rows of retained sample correlation;
  workspace clearing releases its identity and covariance.
- Cancellation-safe small-angle SO(3) Jacobians and rotation integrals, correct
  continuous-noise position moments, gyro/force coupling, and finite-rotation
  preintegration sample/bias sensitivities.
- Offline rotating-force integration with separate Earth/body rotations,
  scaled exponential state transitions, held-input integrals, and full discrete
  process covariance, including bias-driven cross terms.
- Correct offline initialization sample-cross sign and sensor-frame boresight
  Jacobians. Shared parameter kinds without a supported executable sensitivity
  return an unavailable capability instead of using a guessed mapping.
- Real-run attachment binding in live and offline trajectories and live metric
  preflight.
- Offline dense trajectory values and actual endpoint covariance remain usable
  when the existing conditional bridge cannot represent the full covariance.
  Interior numeric uncertainty is explicitly unavailable in that case. The
  private temporary segment record is now v03 and retains this availability.

## Prepared-input boundary

- `ImuObservation::new(id, measurement_frame, profile, angular_rate,
  specific_force, status)` accepts calibrated SI vectors already averaged over
  the same nonzero interval, with explicit clock, support and covariance. It
  rejects mismatched epochs/clocks/supports and overflowing support arithmetic.
- `ImuStatus::{Valid, Degraded, Unavailable, Initializing, Discontinuity}` replaces
  sensor counters and fault/channel metadata. Incomplete or saturated vectors
  cannot be integrated. Degraded inputs carry their prepared covariance through
  both engines and mark live and offline trajectory quality; unavailable inputs
  use gap policy, while initialization/discontinuity restarts navigation.
- `InputProfileSpec` / `InputProfileId` replace device-specific profiles. Only
  rate/capacity and generic identity information remain; solution-only inputs
  can declare zero raw observations. `CalibrationBundle` retains shared
  installation uncertainty without serial, temperature or hardware range rules.
- Removed device protocols, counters, high-range channel substitution, device
  profile digests, temperature/range qualification and recording adapters.
  Hardware synchronization, calibration application, unit conversion, channel
  selection, resampling and recording will be implemented outside this package
  later. Nothing was added to firmware.
- Keep ESKF/Kalman filtering, IMU integration, GNSS fusion, frame rotations,
  lever-arm corrections, bias estimation, calibration/clock uncertainty,
  smoothing and trajectory/metric mathematics inside `traj`.
- For GNSS, supply generic ECEF position and/or velocity, independently timed,
  with uncertainty and solution quality. Offline ingestion uses semantic
  `EvidenceSource` records and transactional `ResultSink` output. No receiver or
  recording format is required by the engine.

## Explicit remaining mathematical limits

- Offline RTS does not yet retain the held IMU sample in its stored augmented
  state. It therefore rejects a nonzero sample covariance reused across multiple
  stored propagation intervals with `ProcessError::CapabilityUnavailable`.
  Ordinary adjacent-state RTS is mathematically incorrect for that case; a
  scalar counterexample and an end-to-end rejection fixture guard it. Aligned
  supports and deterministic sample covariance remain supported. The live
  engine and captured replay retain the sample latent and support these cuts.
- Full coupled conditional dense covariance remains future work. Unsupported
  interior covariance is not replaced by an independent-noise approximation.
- Offline missing IMU support still requires supported reinitialization;
  inferred short-gap bridging is implemented only in the live path.
- Independent IMU timestamp jitter has no supported temporal sensitivity model
  yet. Live retains such observations unfused; offline now explicitly returns
  `CapabilityUnavailable` instead of silently ignoring the jitter. Shared clock
  offset/drift uncertainty remains supported. Upstream must not erase jitter.
- Actual-board numeric, latency, stack, memory, and calibration qualification
  remain release work. Passing host math tests is not hardware qualification.

## Verification from this pass

```text
cargo test --locked --offline -p aevia-trajectory --lib
# 243 passed, 0 failed

RUST_MIN_STACK=16777216 cargo test --locked --offline -p aevia-trajectory --features offline --lib --quiet
# 357 passed, 0 failed

cargo check --locked --offline -p aevia-trajectory
cargo check --locked --offline -p aevia-trajectory --features offline
cargo check --locked --offline -p aevia-trajectory --features offline,gtsam-system,raw-tight
```

The current split also checks generic input alignment, invalid vectors,
degraded quality in live and offline outputs, gap/discontinuity behavior and
explicit rejection of unsupported independent IMU timestamp jitter. A degraded
status alone leaves the supplied measurement means and covariance unchanged.

The offline debug test fixtures require the larger host test-thread stack;
without it, an existing preflight fixture overflowed Rust's default test stack.
This setting is not a measured live-engine stack requirement.

Changed Rust files passed targeted `rustfmt --check`. Independent focused
reviews covered the ESKF equations, offline discretization, and live sample
ownership. Regression oracles include augmented f64 Schmidt updates, analytic
continuous-noise moments, numerical rotation integration, finite-difference
bias/boresight/sample Jacobians, split invariance, `t-1 ns / t / t+1 ns` ownership,
initialization, short gaps, re-anchoring, and rejected/failed update transactions.

No workspace-wide, firmware, acquisition, log-adapter, native-backend, hardware,
or resource qualification tests were run in this pass. No consumer integration
or firmware source changes were made. Existing unrelated work was preserved.
