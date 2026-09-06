# Aevia trajectory engine

`aevia-trajectory` is the shared trajectory and measurement engine for V2 Mini.
It has two processing paths:

- **Live:** bounded estimation on the S31 as measurements arrive. This is the
  default allocator-free, `no_std` build, selected by `ProcessingLevel::EmbeddedLive`.
- **Offline:** recorded-session replay and refinement on a computer or phone.
  Enable the `offline` Cargo feature and use `aevia_trajectory::offline`.
  `ProcessingLevel::OfflineSmooth` selects fixed-interval smoothing;
  `ProcessingLevel::CapturedReplay` reproduces the captured live processing.

Offline processing uses `OfflineResourceLimits` and returns an `OfflineRun`
with an `OfflineRunSummary`. The `offline` feature also enables the host
dependencies required by optional workstation backends.

This directory contains only the core engine. Sensor drivers, protocol decoding,
clock/counter reconstruction, calibration application, channel selection,
resampling, recording codecs and physical I/O belong to external callers. The
former acquisition and log crates have been removed; nothing has been moved into
firmware.

The source is organized by responsibility. The public module paths remain the
entry points; their private submodules hold the implementations:

- `config/`: installation and calibration, numeric and navigation profiles,
  resource limits, qualification, and live/offline processing specifications.
- `engine/`: live session construction and ingestion, clock transitions,
  initialization, frontier processing, result digests, and offline/replay orchestration.
- `metric/`: definitions and plans, numerical algorithms, distance/event queries,
  uncertainty, and incremental activity, lap, and drag tracking.
- `trajectory/`: dense interpolation, conditional covariance bridges, storage and
  record encoding, public queries, and metric root/derivative calculations.
- `offline/solver/`: evidence validation, filter initialization and updates,
  propagation, smoothing, uncertainty projection, and result publication.
- `live/core/` and `live/eskf/`: live scheduling and ingestion, navigation
  propagation, GNSS updates, bounded extended RTS smoothing, and covariance handling.
- `raw_tight/`: optional backend qualification and preflight, ambiguity arcs,
  phase-use accounting, and conditional fix assessment.

Regression tests live beside these responsibilities in separate test modules.
The host feature suite can be run with
`RUST_MIN_STACK=33554432 cargo test -p aevia-trajectory --features offline,raw-tight,gtsam-system`;
its fixed-capacity replay fixtures need a larger test-thread stack than Rust's
default. Test `gtsam-vendored` separately because the two GTSAM features are
mutually exclusive.

The engine accepts:

- `ImuObservation`: a calibrated angular-rate vector in rad/s and specific-force
  vector in m/s², both averaged over the same nonzero interval in the declared
  IMU measurement frame. Supply their effective timestamps, clock-model identity,
  sample support, full covariance (or a configured model), axis validity and
  generic `ImuStatus`. The constructor rejects misaligned vectors. Any channel
  substitution, range/temperature qualification and covariance adjustment must
  already be complete. `Degraded` is usable, `Unavailable` invokes gap handling,
  and `Initializing`/`Discontinuity` require navigation reinitialization.
- `GnssSolutionObservation`: prepared ECEF position and/or velocity with their
  own timestamps, frame/reference-point identities, uncertainty, optional cross
  covariance and explicit per-field `valid` flags. Invalid fields remain
  recordable and are excluded from fusion and initialization. Positioning mode
  (RTK fixed/float, PPP, standalone, etc.) is not an engine input; the caller
  supplies the appropriate uncertainty and receiver health/freshness diagnostics.
  The caller accounts for receiver latency in each field's effective timestamp.
  Velocity-method labels, DOP and satellite/signal counts belong in application
  diagnostics. No receiver message format is required.
- Configuration: input-rate limits (`InputProfileSpec`), installation rotation
  and antenna lever arm, residual calibration uncertainty, clock uncertainty,
  process/measurement noise and metric definitions. A supplied heading is
  optional; observability remains explicit when heading is unknown.

The Kalman filter, IMU integration, IMU–GNSS fusion, frame/lever-arm mathematics,
bias estimation, covariance propagation, live trajectory and metrics, and
computer-side smoothing remain inside this package. Outputs include continuous
position, velocity and orientation queries at a chosen reference point, supported
uncertainty, quality/provenance, derived metrics and typed unavailable results.
Offline callers supply prepared records through `EvidenceSource` and consume
results through `ResultSink`; these are semantic ports with no file-format
requirement. Frame transforms and timing validation are estimator mathematics,
not hardware input processing.

GNSS output quality reports `Healthy`, `Absent` or `Suspect`, independent of
positioning mode. Migrating callers must replace `solution_class` with `valid`
and remove the `rtk_state` constructor argument. Keep any desired receiver mode
labels in application diagnostics. Offline record readers map legacy accepted
mode tags to `Healthy`; new records use a distinct health tag, which older
readers reject rather than misreport as RTK fixed.
Captured-result digests also use the new tag: replay comparisons against old
mode-labelled output digests will differ and require regenerated captures.

Measurement covariance describes the uncertainty of the supplied interval
average, not a continuous noise density. Preserve shared clock/calibration
uncertainty separately instead of adding it independently to every sample.
Nonzero independent IMU timestamp jitter currently remains unfused in live processing and returns
`CapabilityUnavailable` in the offline solver. Neither implementation has the
required temporal sensitivity model. Do not conceal real jitter by setting it
to zero. Shared clock offset/drift uncertainty remains supported.

Offline smoothing retains the error of each interval-average IMU sample across
stored propagation edges, including observations inside its support. Dense
covariance carries navigation, calibration, bias and sample correlations into
interior and offset-point queries and metric uncertainty. Offset velocity
covariance uses finite-support angular-rate uncertainty; the returned
`angular_rate_uncertainty_support` identifies that averaging interval. It does
not assign an instantaneous variance to white gyro noise.

Gate survey uncertainty is explicit: exact, independent normal displacement,
or a shared three-dimensional survey parameter with navigation correlations.
A variance supplied without a correlation model remains unavailable for
correlated metric uncertainty. Offline missing support still needs explicit
reinitialization; unsupported uncertainty models continue to fail closed.

`FiniteGate::new` keeps its scalar-variance argument. Declare its correlation
model with `with_independent_survey()` or `with_shared_survey_parameter(id)`;
direct field access now uses `survey_uncertainty: GateSurveyUncertainty`.
The private offline trajectory record format is version 4 to retain the full
coupled process; resource preflight includes its larger records and bounded cache.

Live navigation uses the ESKF forward estimate plus an extended RTS backward
pass over a fixed 64-node window. Set `NavigationProfileSpec.smoothing_lag` to a
nonzero duration up to 100 ms to enable smoothing; zero retains the forward
comparison path. The lag is additional to `fusion_delay`. Present projections
continue to follow the forward predictor, while trajectory publication and
metric watermarks wait for smoothed endpoints. `finish()` flushes the shorter
remaining window through the last trusted IMU epoch. The backward pass preserves
the filter's fixed Schmidt calibration/clock means and covariances, including
shared IMU-sample and gap correlations; it does not perform nonlinear iteration
or jointly estimate the calibration parameters.

Moving startup needs two fresh, advancing GNSS position/velocity epochs and
valid current IMU support. It can start without a stillness prerequisite,
using a coarse tilt estimate and calibrated bias priors. Repeated or merely
extrapolated receiver epochs do not count as another fix. A supplied heading
remains optional; course over ground is not treated as body heading.
The existing stationary alignment path remains available.

The release firmware target is the `ESP32-S31-WROOM-3-N16R16V`. It runs the
bounded, fixed-capacity live ESKF/RTS and incremental metric path. Full-session
`f64` smoothing runs on a phone or workstation. GTSAM is a planned optional,
independently qualified workstation backend only; the current feature-gated
native boundary is deliberately unregistered and fails closed. GTSAM is never
a dependency of device navigation, recording, display, or live metric
finalization.

The smoother uses the explicit `LiveResourceLimits::V2_MINI_RTS` development
contract with a 3 MiB PSRAM ceiling; the historical `V2_MINI_INITIAL` constant
retains its 1 MiB ceiling. Internal SRAM and stack ceilings remain 192 KiB and
32 KiB. Callers must supply the reported compiled workspace sizes and record
the new navigation profile/digest. New output timing and startup behavior
require captures from the new build for exact replay. See
[`live_smoothing_requirements.md`](../data/live_smoothing_requirements.md) for
the lifecycle, moving-start and qualification requirements.

The allocator-free core is cross-checked with
`riscv32imafc-unknown-none-elf`, the hardware-FPU target class used by the
ESP32-S31. That check does not replace the release firmware proof: the platform
adapter must statically place the two workspaces in internal SRAM and PSRAM,
pin the floating-point estimator task to an explicit high-performance core,
keep capture ISRs integer-only, and qualify the final ESP-IDF/ESP-HAL link map,
stack, timing, cache contention, and soak behavior on the fitted module.
Run `python3 traj/scripts/audit_s31_firmware_purity.py` from the repository root
for the repeatable generic-target dependency and linked-symbol gate.

`WorkQuota` is a nonzero corrected-frontier credit capped at `u16::MAX` per
call. It charges IMU-slice planning, propagation, delayed measurement updates,
RTS backward steps, and frontier/segment commits; it is not a whole-step
wall-clock budget.
Observation ingestion, bounded corrected-history transfer/reanchoring, metric
refresh, and present projection instead have compile-time capacity and loop
bounds. Their timing, together with interrupts, RTOS overhead, cache behavior,
and final-link effects, remains an exact-firmware hardware release gate.

Non-polynomial gate, speed-threshold, and speed-extremum roots use
`EnclosureNativeF64V1`: outward binary64 interval arithmetic through the full
rigid-point, noncommuting SO(3) bridge, and Bowring ellipsoid-normal expression
graphs. Pinned `fpmath` native elementary functions are expanded beyond their
documented error bounds. The production dependency closure is allocator-free;
software scalar adapters are test-only. Isolation retains fixed stack, depth,
root-count, and evaluation ceilings and fails closed on unresolved cells.
The native contract requires round-to-nearest, gradual underflow, and disabled
implicit contraction. Independent Decimal90 regression fixtures cover values
and derivatives; regenerate them with
`python3 traj/scripts/generate_root_enclosure_oracle.py`.

Live preflight still requires measured qualification for this exact backend,
numeric profile, target, toolchain, input envelope, MPFR and independent-oracle
corpora, target bit fixtures, zero oracle escapes, and resource ceilings.
Legacy Taylor-backend attestations cannot qualify the new graph. The host
regressions are not hardware or MPFR qualification evidence.

Distance absolute tolerance belongs to the complete measurement. Segments and
speed-sign subdivisions share that allowance by duration, and cumulative live
reports retain the error already consumed. Relative error is checked against
the complete reported distance, including cancellation in signed quantities;
exceeding the shared numerical-work budget remains a typed failure.

The public interface contains semantic observations, configuration, trajectory
queries, measurements, quality and provenance. Estimator state, matrices,
storage machinery and optional native backends remain private.
