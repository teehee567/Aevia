# Live extended RTS smoothing and moving startup

This change adds a bounded extended Rauch–Tung–Striebel backward pass to the
existing live ESKF. It also permits a simple start while the module is moving.
These are engine development requirements; their implementation does not
establish timing or accuracy qualification on the ESP32-S31.

## Smoothing behavior

- `NavigationProfileSpec.smoothing_lag` declares additional lookahead beyond
  the effective-time GNSS reorder delay. The supported range is 0–100 ms.
  Zero preserves the forward-filter comparison path. A production caller
  selects a nonzero lag and records it in a new navigation-profile revision
  and digest.
- The forward ESKF still processes measurements once, in effective-time order.
  A backward pass uses stored forward states and covariance relationships to
  refine recent trajectory endpoints. It does not relinearize the entire
  window or run an iterated nonlinear optimizer.
- The forward predictor continues to provide the latest supported present
  state. The navigation watermark, trajectory publication and metric
  consumption advance only through endpoints committed by the smoother.
  Normal publication latency therefore includes both reorder delay and
  smoothing lookahead, plus any queued bounded work.
- Shared calibration/clock uncertainty and held interval-average IMU errors
  retain their correlation semantics through the smoothing pass. They must
  not be counted as fresh independent evidence on each edge.
  The constrained Schmidt backward pass keeps nuisance means and covariances
  fixed, including held-sample and gap errors. It retains the extra
  predicted-to-smoothed error cross covariance required by those constraints;
  it reduces to ordinary RTS for a standard Kalman filter. It does not recover
  information discarded by the forward Schmidt approximation or jointly fit
  the calibration parameters. Singular systems are solved only in their
  supported covariance subspace.
- Every published segment preserves its original IMU attitude support and
  input-degradation flags. GNSS quality is captured at each forward endpoint;
  a later fix must not erase or incorrectly relabel earlier quality evidence.
- Committed endpoints remain immutable. Consecutive published segments share
  the same boundary state and covariance, including at window rollover.

## Lifecycle and bounded work

- `WorkQuota` bounds forward and backward frontier operations. Insufficient
  credit pauses work without partially publishing a segment. Empty-observation
  calls resume the retained work deterministically.
- `finish()` drains the forward filter and remaining smoother tail through
  the final trusted IMU epoch before reporting completion. The shorter final
  window is intentional; no state is extrapolated beyond available support.
- Exact clock transitions flush the old window through the boundary before
  changing the clock covariance coordinates or discarding navigation.
  Independent/unavailable transitions must not produce a segment bridging
  the resulting restart.
- Reanchoring transforms every retained navigation-frame smoother quantity.
  Reset/reinitialization clears unfinished smoothing state while preserving
  already committed trajectory and existing metric withdrawal behavior.
- Storage and solver scratch are caller-owned, with a fixed **64-node**
  smoother window. Queues do not grow to absorb lateness or overload. A full
  window is an explicit capacity failure; it does not silently shorten the
  configured lag or publish an endpoint early. The normal live path remains
  allocator-free and has no GTSAM dependency.

## Moving startup

- Two fresh, advancing GNSS position/velocity epochs and valid current IMU
  support can seed navigation without a stillness prerequisite. Reusing or
  extrapolating one receiver epoch does not count as a second fix, and both
  retained epochs must satisfy the configured age limit.
- Moving alignment uses a coarse gravity-based tilt estimate and calibrated
  bias priors, with uncertainty reflecting the weak initial attitude
  information. It does not estimate a static gyro bias from moving samples.
- A supplied heading remains usable. GNSS course is not assumed to be body
  heading for an arbitrary attachment; unknown heading remains explicitly
  unobservable.
- The existing stationary alignment path remains available. Missing/invalid
  observations do not qualify a moving start, and gaps or clock resets still
  apply the existing reinitialization rules.

## Resource and compatibility contract

`LiveResourceLimits::V2_MINI_RTS` introduces a **3 MiB PSRAM development ceiling**
for the fixed smoothing workspace. `V2_MINI_INITIAL` retains its original
1 MiB value, so old recorded resource definitions keep their meaning. Internal
SRAM remains capped at 192 KiB including a 32 KiB stack, and the recorder keeps
its separate original budget. `WorkspaceRequirements` reports actual compiled
object sizes; the PSRAM compile-time check retains a 32 KiB margin.

The fitted N16R16V module's memory capacity alone is not timing evidence.
Before release, measure the final linker placement, stack high-water mark,
PSRAM cache contention, forward/backward work latency, frontier delay and
combined acquisition/recording/UI load on the actual module. Do not reuse an
old hardware qualification report for the new navigation profile.

Captured replay executes the same configured live implementation. Changing
the smoothing lag or startup behavior changes output values, publication
timing and work consumption, so same-build transcript hashes must be captured
again for the new profile. The implementation does not promise bit-exact
replay of captures produced by an older estimator build.

## Regression acceptance

Verify future GNSS improves a retained historical estimate without changing
the forward predictor into a smoothed present estimate; zero-lag equivalence;
bounded-drain equivalence; immutable shared endpoints; correct historical GNSS
quality; complete finish; clock-boundary flush; reanchor/reset behavior; and
capture/replay identity for the new implementation. Include a start with
nonzero motion and insufficient stationary evidence. Run host tests and the
allocator-free target checks; report hardware qualification separately.
