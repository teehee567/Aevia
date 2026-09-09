# V2 Mini Continuous Trajectory and Measurement Engine

> Scope update (2026-09-05): this document records the original broader system
> plan. The current deliverable is only `aevia-trajectory`: generic prepared
> IMU/GNSS inputs, estimation/fusion, uncertainty, trajectories, smoothing and
> metrics. Acquisition and recording adapters have been removed from `traj`;
> no firmware integration is part of this work. Use
> [the current handoff](v2_mini_trajectory_engine_handoff.md) and
> [the engine README](../traj/README.md) for the implemented input boundary.

## Summary

Create a small `trajectory/` package family, centred on the shared `aevia-trajectory` engine, that turns UM980 and SCH16T-K01 evidence into:

- A bounded, low-latency live trajectory and essential live measurements that run entirely on the V2 Mini.
- A streamed session artifact containing enough timing, sensor, configuration, and calibration evidence to reprocess every complete recorded span later.
- A offline host replay and solution-level smoother that work without GTSAM.
- Optional higher-accuracy workstation refinement, including a GTSAM adapter and later raw tightly coupled RTK/INS processing.
- Explicit lap, sector, drag, braking, distance, activity, and ski measurements derived from continuous velocity and event roots—not a polyline of GNSS positions.
- Quality, uncertainty, timing provenance, coordinate-frame, and physical-reference-point metadata on every result.

The shared abstraction is a continuous rigid-body trajectory. Applications differ only in measurement definitions. There will deliberately be no generic unqualified `speed` or `distance`.

Post-processing is additive. The ESP32-S31 must start, navigate, display live results, finish a session, and retain its live summary without a phone, network, workstation, GTSAM, or any host-native library. A failed or unavailable refinement path must never make the device path unusable.

The modules cannot promise mathematical perfection or certified integrity. Multipath, outages, incorrect RTK ambiguity fixes, mounting flex, timing uncertainty, and an undefined physical reference point remain real limits.

## Deployment decision

Use one trajectory engine across three deployment tiers:

| Deployment tier | Engine processing levels | Required environment | Estimation responsibility | Output |
| --- | --- | --- | --- | --- |
| Embedded live | `EmbeddedLive` | V2 Mini ESP32-S31 | Bounded receiver-solution ESKF, rolling continuous trajectory, selected incremental metrics | Provisional/finalized live results plus the session artifact |
| Offline host | `CapturedReplay`, `OfflineSmooth` | Phone or workstation, no GTSAM | Behaviorally equivalent embedded-profile replay within declared tolerances and a full-session solution-level forward/backward smoother | Replayed or refined trajectory and recalculated metrics |
| Advanced workstation | `AdvancedGraph`, `RawTight` | Strong desktop/server with optional native dependencies | GTSAM factor-graph smoothing and raw tightly coupled RTK/INS | An additional refined result set with explicit backend provenance |

GTSAM is intentionally not an ESP32-S31 implementation option, even if a
particular cross-build can be made to link. Its useful accuracy gains come
from future observations, nonlinear relinearization, and joint nuisance-state
estimation, while its dynamic C++ sparse-graph runtime conflicts with the
device's bounded memory and latency contract. The distinction still matters on
the dual-core 320 MHz S31: its high-performance cores provide single-precision
floating point, whereas GTSAM's core dynamic matrix aliases and navigation
implementation are `double`-based. The N16R16V module's PSRAM provides useful
bounded history, but it is cache-attached external memory rather than a reason
to permit graph growth or allocator jitter in the live deadline path. If the
live ESKF misses a measured release gate, first prototype a small statically
allocated fixed-lag, iterated, or square-root formulation at navigation/GNSS
cadence and qualify it under simultaneous acquisition, storage, display, and
radio load. Do not move the full workstation graph onto the MCU merely to
obtain a different solver API. Sources: [official ESP32-S31-WROOM-3
datasheet](https://documentation.espressif.com/esp32-s31-wroom-3_datasheet_en.html),
[GTSAM matrix types](https://gtsam.org/doxygen/a00047.html).

The versioned session artifact is the data seam between acquisition and later processing. Every `ProcessingLevel` uses the same observation, trajectory, measurement, quality, and provenance semantics. Solver state, matrices, factor types, storage drivers, and native-library details remain private implementation details.

The device result is never overwritten. Every host result is a sidecar that names canonical source/normalization/input digests, processing profile, engine/backend version, embedded definitions, and all parent revisions. Consumers can therefore compare live, offline-refined, and advanced-refined results or fall back to the live result.

## Sensor-derived constraints

- Treat “50 Hz UM980” as a capability, not a fixed input rate. The receiver specification says up to 50 Hz positioning in a particular mode; output combinations, latency, and available diagnostics remain configuration-dependent. Use `BESTNAVXYZ`-equivalent normalized position/vector-velocity data initially and preserve `OBSVM`-equivalent raw code, phase, Doppler, C/N₀, lock, and tracking fields for later tight coupling. Position and velocity must retain separate effective epochs because the velocity log exposes latency. Sources: [local UM980 R1.9 manual](datasheets/UM980_User%20Manual_EN_R1.9.pdf), [official N4 command/log reference](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf).

- Configure PVT and raw-observation rates independently. A recording profile declares enabled constellations/signals, worst-observed satellite/signal count, bytes per epoch including framing, each UART baud/routing, scheduling headroom, and overflow counters; it is rejected if the measured/worst-case port budget does not fit. “50 Hz RTK” never implies that a full all-signal `OBSVM` stream also fits at 50 Hz. The session records receiver firmware plus the complete active configuration so the decoder and evidence contract match the generated logs.

- Make the estimator rate-agnostic. The SCH16T has no native 300 Hz mode and no absolute measurement epoch; it does provide `FREQ_CNTR` plus per-axis data counters for RATE2/ACC2 that must be retained and unwrapped, while MCU DRY capture supplies the absolute-time anchor. At approximately 300 Hz it must use the lower-bandwidth 30 Hz or 13 Hz filter profiles, with their measured group-delay model and uncertainty represented in the effective epoch. The baseline bounded V2 Mini candidate is the approximately 1.475 kHz decimated output with DRY capture and LPF3 Bessel filtering; qualify its navigation error against Murata's maximum-performance reference of interpolated gyro at 6.3 kHz and accelerometer at 4.2 kHz before selecting it for release. RATE2/ACC2 are decimated interval averages, whereas ACC3 is interpolated, has no corresponding 4-bit data counters, and requires a distinct read epoch/support for each sequentially read axis. Source: [SCH16T-K01 datasheet](datasheets/sch16t-k01-datasheet-full.pdf).

`V2MiniLive` is an immutable, content-addressed executable sensor profile, not a name for loose defaults. Its baseline SCH16T candidate is pinned to SCH16T-K01 data sheet Doc. No. 11624 Rev. 6 and uses 48-bit out-of-frame SafeSPI mode 0 at 10 MHz, target-address bits `TA[9:8]=00` for the board's grounded TA9/TA8 selector pins (with `TA[7:0]` carrying the register address), and `FT=1` for every normal frame. It selects LPF3 (`011`) on every RATE/ACC path; DYN1 for RATE1/2 and ACC1/2; DYN0 for ACC3; DEC4 (`011`, `F_PRIM/16`) identically on all RATE2/ACC2 axes; and active-high DRY with standard-speed MISO/DRY. Its complete post-start control image is `CTRL_FILT_RATE=0x00DB`, `CTRL_FILT_ACC12=0x00DB`, `CTRL_FILT_ACC3=0x00DB`, `CTRL_RATE=0x12DB`, `CTRL_ACC12=0x12DB`, `CTRL_ACC3=0x0000`, `CTRL_RATE_FLAG_1=0x0000`, `CTRL_RATE_FLAG_2=0x0000`, `CTRL_ACC_FLAG_1=0x0000`, `CTRL_ACC_FLAG_2=0x0000`, `CTRL_USER_IF=0x202C`, `CTRL_ST=0x0FFE`, and final `CTRL_MODE=0x0003`; the zero flag-filter controls and `CTRL_ST` value deliberately preserve the Rev. 6 reset behavior for saturation filtering and continuous/start-up self-tests. The driver also restores `SYS_TEST=0x0000` after its nonzero bus-access challenge, and treats `CTRL_RESET=0x000A` only as a transient reset command rather than configuration.

The executable startup state machine first waits at least 1 ms after all supplies are within specification, releases `EXTRESN` only after that wait, and then holds SCK low for the full additional 32 ms NVM/SPI start-up interval. In low-power mode it performs the `SYS_TEST` out-of-frame write/read challenge, reads `COMP_ID=0x0023`, requires `ASIC_TYPE=0`, and requires the complete 12-bit `ASIC_ID` to be in the profile's literal qualified-revision allow-list. That allow-list has no wildcard or major-version match and remains empty until the actual production-lot values pass the same compatibility fixtures; an unlisted revision cannot navigate. It also reads `SN_ID1/2/3`, validates every Rev. 6 reserved/format constraint, reconstructs the canonical `DDDYYFHHHHH01` serial, records the three source words and reconstructed value, and requires an exact match to the residual calibration bundle selected for that physical sensor. The state machine writes the complete control image except final `CTRL_MODE`, writes `EN_SENSOR` as `CTRL_MODE=0x0001`, waits at least 215 ms, reads every status register once without judging its intentionally uncleared start-up flags, writes EOI as `CTRL_MODE=0x0003`, waits at least 3 ms, reads every status register twice and requires both complete sets plus frame `S[1:0]` to be OK, and only then reads back the entire control image, identity, and serial. A status, identity, serial/calibration binding, bus-access, or readback mismatch invokes the specified soft/hardware-reset path and restarts at the 32 ms stage; after five failed cycles the device enters a non-navigating safe state.

Before acquisition starts, the driver establishes a known pending `FREQ_CNTR` response. Each normal DRY transaction is then a fixed 12-frame pipeline reading RATE2 XYZ, ACC2 XYZ, ACC3 XYZ, temperature, and `FREQ_CNTR`, including every specified CS/inter-frame delay. Frame 12 transmits another harmless `FREQ_CNTR` primer while receiving the response requested by frame 11; its pending response is carried as explicit driver state and is the source-address-checked first MISO response of the next burst. Thus every out-of-frame request and response is owned across burst boundaries. Every frame's MCU start/end tick is captured; normalization retains three axis-specific ACC3 epochs/supports before resampling them onto a common vector support. The 4-bit RATE2/ACC2 data counters and 14-bit frequency counter are checked and unwrapped; ACC3 is never assigned a fabricated data counter. Every MOSI and MISO frame must pass the Rev. 6 CRC-8 (`poly=0x97`, `init=0xFF`, `xorout=0x00`, MSB-first), each response source address must match the tracked preceding request, and CE/IDS/S[1:0], high-impedance, and protocol-error indications are handled before payload decode. A failed frame invalidates the complete affected IMU epoch. A bounded fault path clears the pending-request state, aborts the normal burst, executes the profile's datasheet-derived flush/read-clear and known-primer response-pipeline resynchronization, verifies that required decimated registers were consumed and DRY can rearm, and resets/reinitializes the sensor if alignment cannot be proved; no error payload becomes numeric sensor data. DRY follows the slowest decimated axis and rearms only after the required decimated reads, so mismatched axis decimation is rejected. DMA/SPI timing is proven against the full `F_PRIM` range: DEC4 spans approximately 1.381–1.569 kHz, and buffers/scheduling accept at least 1.75 kHz. ACC3 samples above its specified ±80 m/s² performance range are out-of-qualified-range unless the board calibration explicitly covers them; electrical headroom is not a metrology claim. Source: [SCH16T-K01 datasheet](datasheets/sch16t-k01-datasheet-full.pdf).

The matching UM980 record names one accepted receiver firmware and the exact `SIGNALGROUP`, dynamic model, correction mode, binary messages, rates, UART routing/baud, maximum observation count and encoded bytes, and position/velocity latency limits. The bring-up candidate routes binary `BESTNAVXYZ` at up to 50 Hz on one 921600-baud UART and binary `OBSVM` at a separately qualified rate on the other 921600-baud UART. It also records `STADOPB` for BESTNAV DOP/used-satellite context, `BESTSATB` for used-signal masks, and `RTKSTATUSB`, `RTCMSTATUSB`, and `HWSTATUSB` for solution/correction/hardware health at their exact accepted rates (`HWSTATUSB` is limited to 1 Hz). No candidate becomes `V2MiniLive` until captured worst-case all-signal traffic plus all diagnostics and RTCM input stays below both measured port/DMA/parser budgets with margin. The accepted command transcript, `VERSION`, `CONFIG`, `UNILOGLIST`, and byte-budget report become part of the profile and session manifest. Sources: [official N4 log reference](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf), [STADOP definition](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf).

- Require the acquisition adapter—not this crate—to correlate MCU timer captures of `GNSS_PPS` and `IMU_DRDY`. V2 Mini does not route the UM980 EVENT pin, so the design must not depend on it.

- Keep UART/SPI parsing, UM980 commands, RTCM/NTRIP transport, register reads, PPS clock fitting, physical storage/filesystem drivers, BLE, Flutter Rust Bridge, and UI DTOs outside the trajectory engine. The versioned session/result formats and codec remain in scope in the log package.

- Design to the fitted `ESP32-S31-WROOM-3-N16R16V`, not to a desktop. The current module/SoC documentation gives 512 KB shared instruction/data SRAM plus 32 KB low-power SRAM, 16 MB in-package PSRAM for this variant, two high-performance cores up to 320 MHz, and a single-precision FPU. Consequently, do not assume native-speed `f64` in the per-IMU hot path or keep a full session in memory. Keep hot state and DMA-critical buffers in internal SRAM, use PSRAM only for measured bounded history/scratch, and stream session records to microSD. Source: [official ESP32-S31-WROOM-3 datasheet](https://documentation.espressif.com/esp32-s31-wroom-3_datasheet_en.html).

- Use official ESP-IDF drivers for ESP32-S31 timers, DMA, SDMMC, PSRAM, and peripheral bring-up. Build the Rust `aevia-trajectory` engine as the sole estimator behind a narrow platform/C ABI where required, and prove compilation, linking, floating-point behavior, panic/allocator policy, timer capture, DMA buffers, and memory layout on the actual module. Statically create and pin the floating-point estimator task to a declared high-performance core; capture ISRs remain integer-only and hand timestamped records to that task, rather than relying on ESP-IDF's lazy first-floating-point-use task affinity. If the selected Rust target cannot execute the engine correctly, choose one engine implementation language before estimator development; do not retain parallel live estimators. GTSAM remains workstation-only. Sources: [ESP-IDF ESP32-S31](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s31/index.html), [ESP32-S31 Rust status](https://docs.espressif.com/projects/rust/esp-hal/1.2.0-rc.0/esp32s31/esp_hal/index.html), [ESP-IDF FreeRTOS floating-point restrictions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/api-reference/system/freertos_idf.html#floating-point-usage).

## Crate and build design

The estimator is one module and one Rust crate: `trajectory/engine` / `aevia-trajectory`. It owns the observation semantics, live ESKF, captured replay, offline smoother, continuous trajectory, metric evaluation, quality/provenance rules, and optional advanced backends. Processing capability changes the implementation selected behind the same interface; it does not create a second public estimator.

Two supporting crates remain outside the estimator because they have different responsibilities:

- `trajectory/acquisition` / `aevia-trajectory-acquisition`: versioned UM980/SCH16T decoding, time normalization, and residual system calibration. UART/SPI/DMA, receiver control, and RTCM/NTRIP transport remain hardware adapters outside it. Bias-linearized batching and IMU preintegration belong to the engine so private estimator state never leaks across the crate boundary.
- `trajectory/log` / `aevia-trajectory-log`: the versioned session/result codec, canonical framing and digests, checksums, capability/completeness derivation, and borrowed record views. Filesystem and transport I/O remain outside it.

The crate dependency graph is acyclic:

- `aevia-trajectory` declares semantic observations/results and, under `offline`, the restartable borrowed `EvidenceSource` and `ResultSink` I/O ports. It depends on neither acquisition nor log.
- `aevia-trajectory-acquisition` and `aevia-trajectory-log` depend on the engine's base contracts with default features disabled. They implement decoding/normalization and artifact source/sink adapters respectively.
- `aevia-core` depends on all three and is the composition root. It validates artifacts through log, runs captured or recomputed normalization through acquisition, selects exactly one evidence lineage per source/span, and invokes the engine with a semantic evidence source/result sink.

The artifact-reader and in-memory-fixture source are the two real adapters at the engine's host I/O seam. They expose borrowed semantic records, manifest/capability metadata, and restart; they never expose a filesystem handle or decoder/backend state. The engine independently validates IDs, ordering, frames, capabilities, and lineage declarations before fusion.

`aevia-trajectory` has an allocator-free `no_std` baseline. Additive build features make implementations available:

- `offline`: `std`, streaming replay, seekable state storage, `f64` forward/backward smoothing, and full host metrics.
- `gtsam-system` or `gtsam-vendored`: mutually exclusive workstation-only GTSAM adapter, implying `offline`.
- `raw-tight`: workstation-only raw GNSS/INS processing, implying `offline`; it may use the selected GTSAM adapter but does not alter the public interface.

Cargo features express compiled availability only. Runtime `ProcessingLevel` selects `EmbeddedLive`, `CapturedReplay`, `OfflineSmooth`, `AdvancedGraph`, or `RawTight`; construction reports `CapabilityUnavailable`, `PlatformUnsupported`, `EvidenceUnavailable`, or `InsufficientResources` precisely. Compile-time target guards reject `gtsam-*` and `raw-tight` on bare-metal/ESP-IDF targets. Firmware dependency-tree and link-map checks fail if `std`, host storage, GTSAM, or a C++ runtime becomes reachable through its engine build.

Because Cargo unifies features inside one dependency graph, firmware, offline-host, and GTSAM builds run as separate target-scoped CI invocations. Firmware depends on `aevia-trajectory` with default features disabled and verifies `cargo tree -e features` plus the final link map. Source: [Cargo feature resolution](https://doc.rust-lang.org/cargo/reference/resolver.html#features).

Use pinned `nalgebra` statically sized matrices/vectors with default features disabled and its pinned `libm`-backed floating-point support for the embedded path. `SMatrix`/`SVector` provide stack-stored fixed dimensions and retain `no_std` operations. Pin the exact crate versions, feature set, math backend, compiler flags, FMA policy, and lockfile/source digests. Use a pinned `heapless` only for small fixed-capacity queues/mutation lists after code-size and operation-count comparison; large history remains in caller-owned arenas. All library/container types remain private, every capacity failure is handled explicitly, and firmware link checks prove that neither an allocator nor `std` entered transitively. Host-only sparse graph algebra remains owned by GTSAM. Sources: [nalgebra fixed-size matrices](https://www.nalgebra.rs/docs/user_guide/vectors_and_matrices/), [nalgebra embedded targets](https://www.nalgebra.rs/docs/user_guide/wasm_and_embedded_targets/), [heapless fixed-capacity structures](https://docs.rs/heapless/latest/heapless/), [GTSAM navigation module](https://borglab.github.io/gtsam/navigation/).

`V2MiniNumericV1` must also pin Rust 1.86 or later and the reviewed source digest of `fpmath 0.1.1` with `soft-float` for a private allocator-free `EnclosureV1` used only to certify metric roots, not in the IMU/filter hot path. The embedded form must use deterministic `SoftF32`; offline processing must use `SoftF64`. Each elementary arithmetic result is expanded with IEEE-754 `next_down`/`next_up`; `sqrt`, `sin`, and `cos` are expanded beyond `fpmath`'s documented error bound, and periodic extrema, domains, division-through-zero, overflow, and the SO(3) small-angle branch are handled explicitly. The layer implements only the interval operations required for Rodrigues/SO(3), norms, ellipsoid normals, reference-point kinematics, and their first derivatives. Any non-finite or unbounded enclosure remains `Ambiguous`. Its source, rounding expansion counts, compiler/FMA policy, and reachable input ranges are part of the numeric profile. Qualification proves inclusion against a high-precision MPFR oracle and an independent host interval implementation across all profile boundaries, critical points, randomized cells, and actual-target bit fixtures, then measures its fixed operation/code-size budget. A live non-polynomial metric capability is unavailable—and a profile requiring it cannot ship—until those gates pass. Sources: [`fpmath` no-std/soft-float and error contract](https://docs.rs/fpmath/latest/fpmath/), [Rust IEEE-754 `next_up`/`next_down`](https://doc.rust-lang.org/core/primitive.f32.html#method.next_up), [IEEE 1788 interval-arithmetic model](https://doi.org/10.1109/IEEESTD.2015.7140721).

Implementation checkpoint (2026-09-03): `traj` currently contains a fixed-capacity `NativeF64TaylorV0` branch-and-bound backend whose Taylor bounds include conservative native roundoff guards. A separate private host/test prototype supplies `SoftF32`/`SoftF64` outward intervals, second-order jets, vector/matrix operations, remainder-bounded SO(3) small-angle rotation, and a conditioned Bowring ellipsoid-normal graph. The current `fpmath` soft-float implementation transitively carries allocator metadata through `rustc_apfloat`/`smallvec`, so the prototype is deliberately excluded from the baseline firmware feature set until a genuinely allocator-free software scalar backend is qualified. It is not wired to root isolation and carries no formal interval-certification claim: dense-segment/base-orientation conversion and each complete non-polynomial root equation still use the native backend. Live preflight therefore fails closed for every configured non-polynomial trigger—including a nonzero reference-point lever arm, horizontal/body speed, offset-point gate/speed evaluation, and cumulative non-polynomial distance root—unless the immutable qualification report carries a zero-escape measured attestation for the exact native backend ID/revision, numeric profile, S31 target, toolchain, reachable-input envelope, MPFR corpus, independent interval oracle, actual-target bit fixtures, case count/error, and fixed evaluation/operation/code-size ceilings. Such a native-backend attestation is intentionally distinct from, and does not satisfy, the `V2MiniNumericV1` `SoftF32` release requirement. Polynomial origin-point gate planes and origin-point 3-D squared-speed thresholds remain available without that non-polynomial attestation.

Keep exact integer session time and sequence numbers at every processing level. The embedded implementation may use validated mixed precision: local position, velocity, attitude error, biases, and covariance may use `f32` only where reference fixtures show the resulting error stays below the allocated sensor/metric budget. ECEF anchors, coordinate transforms, exported values, and sensitive timing calculations use `f64` or fixed-point. `CapturedReplay` executes the same versioned scalar types, reduction order, math backend, FMA policy, and deterministic tie rules as firmware; comparison deadbands around NIS, observability, root-topology, and capacity thresholds are wider than the measured cross-target numerical envelope. It must reproduce every discrete disposition and event identity, while continuous values use the declared tolerance table. `OfflineSmooth` and advanced processing use `f64`. Never propagate ECEF position directly in `f32`.

Host processing uses a private `StateStore` seam with memory and seekable temporary-file adapters, bounded caches, reverse iteration/random access, checksums, space preflight, cleanup, and explicit storage/query errors. Solver state, scalar types, matrix types, factor keys, preintegration objects, and state-store handles never cross the engine interface.

Required build matrix:

- Cross-compile and link the allocator-free engine plus acquisition/log crates for the actual ESP32-S31 target.
- Build and test the same engine on the host with no host feature, then with `offline`.
- Build each GTSAM feature independently on supported workstation targets and reject invalid feature/target combinations.
- Run the same observation/result fixtures across embedded, captured-replay, offline, and available advanced levels. Captured replay must match embedded discrete behavior exactly and continuous fields within its tolerance table; refined levels use their separately declared numeric/accuracy gates.

## Public module seam

The engine has two entry workflows because live streaming and batch processing have fundamentally different lifetimes, while sharing all semantic inputs and outputs:

```rust
let live_plan = TrajectoryEngine::live(live_spec).preflight()?;
let requirements = live_plan.requirements();
let mut live = live_plan.start(workspace)?;
let update = live.step(LiveStep {
    observation: Some(observation),
    work: work_quota,
})?;

let prepared = TrajectoryEngine::process(processing_spec)
    .preflight(session_manifest, resource_limits)?;
let result = prepared.run(session_source, result_sink, run_control)?;
```

Mandatory semantic data—installation, calibration, uncertainty models, processing policy, span, and metric definitions—belongs in `LiveSpec` or `ProcessingSpec`. Builders add resource placement, cancellation/progress controls, and execution preferences only; they do not expose matrix dimensions, factors, scalar types, or solver tuning. Preflight validates sensor/profile compatibility, evidence capabilities, coordinate frames, workspace/storage bounds, compiled and qualified processing levels, requested metrics, and restartability before stateful work begins.

`ProcessingPolicy` is either `Require(ProcessingLevel)` or `BestQualified { preference }`. Offline smoothing is the default host result; an advanced graph may precede it in an explicit preference list. Raw-tight remains explicitly required until it passes its own qualification corpus. A level is selectable only from the intersection of compiled engine capabilities, evidence available for the requested span, installation/calibration capabilities, platform/resource limits, and qualified operating conditions. `BestQualified` may restart the next candidate from immutable inputs after a diagnosed failure; `Require` fails if its level is unavailable. Results record every candidate/rejection/attempt plus the requested and actual levels without exposing a solver trait.

The `aevia-core` acquisition orchestrator assigns the canonical record sequence, timestamps, and normalizes each input once, then fans it out to the live engine and recorder. Neither sink calls the other: a storage failure cannot stop live navigation, and a rejected live measurement cannot erase the evidence needed for later reprocessing.

The embedded workflow has one named, versioned `V2MiniLive` profile. Firmware supplies immutable `EngineConfig`, a validated `LiveMetricPlan`, and a caller-owned `LiveWorkspace` split into explicitly aligned internal-SRAM and PSRAM regions. Session construction borrows that workspace for its lifetime and returns a small handle; large state is never returned or moved by value. Required sizes, alignment, placement, and profile identity are queryable before construction. The firmware path does not require a global allocator.

`LiveSession::step` accepts an optional borrowed fixed-size `LiveObservation` and a nonzero corrected-frontier work credit capped at `u16::MAX`, then returns a borrowed update valid only until the next call. Credits charge IMU-slice planning, propagation, delayed measurement updates, and frontier/segment commits; they are not a whole-step wall-clock budget. Observation ingestion, bounded corrected-history transfer/reanchoring, metric refresh, and present projection instead have fixed compile-time capacity/loop bounds and remain subject to exact-firmware target timing qualification. A no-observation step drains eligible frontier work. Each update has a fixed maximum mutation count and must be synchronously consumed; session state and acquisition do not depend on callers retaining it. Finishing declares end of input, drains its terminal frontier work in bounded calls within the supplied per-call credits, and writes the bounded live summary into caller-provided storage. Its trajectory covers only configured rolling history and never implies that a full session is resident on the device.

`MetricPlan` is the shared semantic definition. Before a live session starts it is validated/compiled into `LiveMetricPlan`, which fixes active-gate/target counts, work limits, output capacity, supported capabilities, and a finite maximum future-support/lookahead for every retrospective rule. An unbounded retrospective live rule is rejected. A live trajectory does not accept an arbitrary new plan through a query. Host evaluation may apply any supported `MetricPlan` through the engine-owned full-session trajectory handle.

The returned `Trajectory` is an opaque engine-owned handle, not an implementable public trait. Its small interface provides `span`, bounded `state_at`, declared capabilities/provenance, and engine-owned `measure`. Internally, metrics use a validated sequential segment cursor containing dense segment coefficients, endpoint transition/cross-covariance data, motion-classification posterior, and quality intervals. This lets memory-backed, rolling, and seekable chunk-backed trajectories share correct full-span evaluation without exposing storage or numerical structure to callers. The live handle permits queries only inside its rolling horizon; host queries surface unavailable/corrupt backing storage explicitly.

The host workflow receives a validated semantic evidence source from `aevia-core` and returns the same `Trajectory` and result bundle forms used by live processing. `ProcessingSpec` records captured or recomputed normalization provenance. Callers never construct solvers, factors, state stores, or backend adapters.

Do not expose:

- ESKF state vectors or factor-graph keys.
- Solver/factor traits.
- Jacobians, preintegration objects, or sparse matrices.
- Backend selection through public dynamic dispatch or public solver traits.
- Sensor-specific drivers or application-specific DTOs.
- Filesystem handles, GTSAM availability, C++ types, or native-library lifecycle.
- Variable-size raw-record ownership or host state-store handles through the embedded interface.

## Time, frame, and uncertainty types

Use exact integer time:

```rust
pub struct SessionTime(i64);        // Nanoseconds relative to SessionEpoch
pub struct SignedDurationNs(i64);
pub struct DurationNs(u64);

pub enum SessionEpoch {
    Gps { week: u32, tow_ns: u64 },
    Local {
        boot_nonce: u128,
        timer_origin: u64,
        timer_hz: u32,
    },
}

pub struct ObservationTime {
    pub registered_at: SessionTime,
    pub correction: SignedDurationNs,
    pub independent_one_sigma: DurationNs,
    pub clock_model: ClockModelRef,
    pub support: SampleSupport,
    pub basis: TimingBasis,
}
```

`effective_time = registered_at + correction`. `SessionTime` is always monotonic within one boot/session. Evidence acquired before trustworthy GPS time uses `SessionEpoch::Local`; a later immutable `EpochResolution` maps that same local timeline to continuous GPS time with covariance and a clock-segment ID without rewriting earlier timestamps. GPS-week/TOW, MCU-timer, sensor-counter, and frequency-counter wrap/reset rules are explicit checked arithmetic, never heuristic overflow behavior.

`ClockModelRef` identifies the contiguous clock segment and fitted affine MCU-tick-to-GPS-time model that produced the epoch. A clock-model record contains its reference tick/time, offset/drift covariance, fit residual distribution, validity interval, PPS source, and discontinuity reason. `independent_one_sigma` contains only capture quantization and sample-specific timing jitter; it excludes uncertainty in shared clock, filter-delay, calibration, and installation parameters. This prevents observations derived from the same fitted or calibrated quantity from being treated as though their errors were independent.

This explicitly represents:

- UM980 velocity latency.
- SCH16T filter/group delay.
- Interval-centre correction for decimated observations.
- PPS-disciplined capture uncertainty.
- Arrival-only timing, which is accepted only with degraded timing quality.

For a measurement at an uncertain epoch, include the temporal sensitivity `dh/dt` in its linearization. Only sample-specific timing variance enters that observation's `R`. Shared affine clock uncertainty must not be added repeatedly as independent noise. `EmbeddedLive` carries the current segment's offset/drift as a two-variable Schmidt/consider block: its fitted mean is fixed, but its covariance and cross-covariance with the 15 navigation errors participate in every update. At a segment boundary the engine performs exact Gaussian marginalization and transfers any cross-covariance still required by retained events/metric accumulators. `OfflineSmooth` carries one fixed-mean Schmidt/consider block for every active clock segment through its forward storage and consider smoother; the graph uses the same priors as private variables. Calibration refinement remains separate: estimating a clock mean is permitted only in an explicitly observable advanced profile with a supplied prior, and timing error is never silently absorbed into sensor bias. Sources: [GPS/INS synchronization-error analysis](https://doi.org/10.1109/PLANS.2008.4570010), [NASA consider-state filter treatment](https://ntrs.nasa.gov/api/citations/20180003657/downloads/20180003657.pdf), [uncertain-delay measurement fusion](https://skoge.folk.ntnu.no/prost/proceedings/acc05/PDFs/Papers/0720_FrB02_2.pdf).

The same shared-parameter rule applies beyond clocks. Every uncertain lever arm, boresight, IMU scale/misalignment or g-sensitivity coefficient, filter/group-delay coefficient, gate survey, and frame transformation has a stable parameter ID, mean, joint covariance, validity span, and declared correlations. Irreducible sample noise alone enters per-observation `R`. Every material shared parameter is either propagated through a profile-bounded Schmidt/consider block with navigation cross-covariance in `EmbeddedLive`, represented as a nuisance variable with the required cross-covariances in offline/advanced processing, or covered by a sequence-level conservative bound whose requested-span coverage is demonstrated in qualification. Repeating its marginal variance in each observation is forbidden. If neither joint propagation nor a qualified bound is available, affected numeric uncertainty is unavailable and any measurement whose integrity depends on it is rejected at preflight. Holding a parameter mean fixed does not remove this uncertainty-accounting requirement.

UART/SPI receipt time may be retained in optional acquisition diagnostics but must never become the measurement epoch.

Use private-field semantic vector and covariance types with validated constructors:

- `EcefPosition`
- `EcefVelocity`
- `SensorSpecificForce`
- `SensorAngularRate`
- `BodyVector`
- `LocalEnuVector`
- `Covariance3`
- `KinematicCovariance`

Reject non-finite values, invalid rotations, negative variances, and covariance matrices that are not symmetric positive semidefinite. Missing uncertainty must be represented explicitly as `Modeled(UncertaintyModelId)`; it must never silently become zero variance.

Canonical conventions:

- Navigation position and velocity: ECEF in a named terrestrial-frame realization and coordinate epoch.
- Attitude: `orientation_ecef_from_body`.
- Body frame: right-handed forward-left-up.
- Internal angle: radians.
- Internal length: metres.
- Internal time scale: continuous GPS time; UTC/leap-second conversion belongs at export.
- Receiver positions, base coordinates, gates, and surveyed geometry require a terrestrial frame, realization, coordinate epoch, antenna reference point, and covariance. Generic “WGS-84” ensemble coordinates are insufficient for centimetre comparisons.
- Elevation defaults to WGS-84 ellipsoidal/local-up semantics. Orthometric or MSL labels require an upstream geoid conversion.

Before fusion or metric evaluation, every spatial input is transformed into the session processing frame by a recorded coordinate operation or rejected as `FrameUnresolved`. Host import uses a pinned PROJ database/operation and records its complete pipeline and grid digests; embedded processing accepts only already-normalized coordinates and the compact operation metadata needed for audit. Approximate/ballpark transforms cannot support timing-grade surveyed gates. This is required because the EPSG WGS-84 ensemble deliberately groups multiple realizations with metre-level ensemble accuracy, while dynamic transformations require an epoch. Sources: [EPSG WGS-84 ensemble](https://epsg.io/6326-datum), [PROJ time-dependent transformations](https://proj.org/en/stable/operations/time_dependent_transformations.html).

## Normalized observations

Do not pass one variable-sized observation superset through the embedded hot path. Split the interface by actual capability:

- Engine `LiveObservation` variants are fixed-size or bounded borrowed views containing only calibrated high-rate IMU samples/status, receiver-solution position/vector velocity, and timing/clock transitions required by the live ESKF. The engine privately batches IMU samples.
- Log `SessionRecord` variants contain one canonical exact-live-input record stream, native sensor evidence for recomputation, raw GNSS epochs, ephemerides, base/correction data, configuration/calibration changes, gaps, and derived audit records. The exact-live-input stream is the normalized high-rate stream; it is not recorded twice under another name.
- Host readers expose validated borrowed/chunked views over those records and materialize variable satellite/signal collections only within host resource policy.

Raw GNSS epochs, ephemerides, base data, and native sensor evidence therefore go directly to the recorder and are not copied through `LiveSession`. Firmware encoding uses caller-supplied/fixed buffers and never requires a session-sized `Vec` or a variable-size enum on the stack.

Every estimator observation and its corresponding record contain:

- `ObservationId { source, sequence }`
- Effective timing and timing uncertainty.
- Explicit coordinate/sensor frame.
- Quality/status.
- Measurement uncertainty or an explicit configured model reference.

### IMU observation

`ImuObservation` contains:

- Independently timed calibrated RATE2 inertial angular rate `ω_ib^b`, ACC2 specific force, and optional ACC3 high-range specific force, because their filter delays and sample support differ. ACC3 retains one capture-derived epoch/support for each X/Y/Z register read until the engine resamples the three axes onto a declared common vector support. Accelerometer output is never named kinematic acceleration in estimator semantics.
- SI values: m/s² and rad/s.
- Optional ACC3 high-range specific force for saturation bridging, with its interpolated-output support and epoch kept distinct from ACC2 and an explicit in-range/out-of-qualified-range status.
- Optional temperature.
- Sample support: filtered point sample or interval average with duration.
- SCH16T output profile ID covering filter, decimation/interpolation, range, resolution, nominal delay model, and configured scale.
- Unwrapped RATE2/ACC2 data-counter and `FREQ_CNTR` evidence, with explicit reset/wrap handling, so missed DRY edges and sensor-clock drift are distinguishable; no ACC3 data counter is implied.
- Per-axis validity and saturation.
- Initialization, common-fault, and timing-provenance flags.

SCH16T loads its factory coefficients internally from NVM during startup. Acquisition verifies the documented startup/status/EOI sequence, decodes the already factory-corrected output, and applies only Aevia's versioned residual system calibration. The estimator still models residual bias and stochastic drift; it never reapplies Murata's internal factory correction.

Never integrate:

- Initialization-status samples.
- Common-fault samples.
- Invalid axes.
- Saturated gyro axes.

When an ACC2 axis saturates but the time-aligned ACC3 axis is valid and inside its qualified calibration range, substitute that specific-force component with its own noise model and emit a quality transition. If any gyro component or any unsubstituted accelerometer component is invalid, reject the complete IMU epoch/batch; a 15-state mechanization never integrates a partial vector unless a separately qualified reduced-axis profile exists.

The canonical exact-live-input record preserves each normalized high-rate `ImuObservation`; the recorder may additionally preserve each engine-emitted preintegrated batch as an audit record. Every batch carries its exact accelerometer/gyro bias linearization values, increment frame, SO(3) tangent/error convention, covariance ordering/basis, first-order correction validity bound, constituent observation-ID range, batching algorithm/profile revision, support interval, and accumulated timing uncertainty. Captured replay deterministically rebuilds batches from that single exact-live-input stream and verifies any recorded audit batch; refined/recomputed processing rebuilds inputs from verified native frames and then rebuilds batches. No processing level fuses both representations of the same evidence.

The selected SCH16T filter profile also names a batch-noise model. Its coefficients come from static/thermal captures and Allan-deviation plus output-autocorrelation analysis. Successive filtered outputs must not be treated as independent white samples; batching composes the measured colored-noise contribution or applies a validated effective covariance/inflation at the navigation cadence.

### GNSS solution observation

`GnssSolutionObservation` contains independently optional:

- Timed ECEF position and full/diagonal covariance.
- Timed ECEF velocity vector and covariance.
- Optional position/velocity cross-covariance.
- Velocity method: receiver Doppler, TDCP, differenced PVT, or unknown.
- Position and velocity solution classes.
- RTK fixed/float/single/DGPS/PPP/invalid state.
- Satellites/signals used, DOP, correction age, solution age, receiver health, and latency evidence.

These diagnostics are independently optional, capability-tagged, and retain their own receiver epochs/ages. The accepted `V2MiniLive` UM980 profile supplies them through its pinned `STADOPB`/`BESTSATB`/status logs. A gate that needs a missing or stale diagnostic is unavailable or uses an explicitly qualified conservative policy; absence is never decoded as healthy or zero.

UM980 per-axis standard deviations are converted to diagonal covariance and then passed through configured covariance floors/inflation because they are not a complete dynamic error model. The static `0.03 m/s` receiver specification must not be used as universal per-epoch covariance.

`GnssSolutionNoiseModel` is selected by receiver firmware/configuration, solution class, correction age, `DynamicsProfileId`, and output rate. The embedded 15-navigation-state profile permits only a fixed decimation/blocking schedule or sequence-level covariance inflation whose coverage has been qualified against the measured innovation autocorrelation; it has no hidden colored-error state. `OfflineSmooth` may select a separately named augmented profile with calibrated first-order Gauss–Markov receiver-solution error states and corresponding smoother transitions when correlation-time identification and observability pass. Fifty-hertz PVT epochs are never assumed independent merely because the receiver reports per-axis standard deviations. Validation includes innovation whiteness, NIS/NEES, and empirical coverage by solution class. If position/velocity cross-covariance is unavailable, use the declared conservative separate-update policy and label it; do not invent a zero cross-covariance as measured truth.

### Raw GNSS observation

Define the typed raw seam in phase 1 even before `raw-tight` is enabled:

- Satellite and extensible signal identity.
- Pseudorange in metres.
- Carrier phase in cycles plus wavelength/signal identity.
- Doppler in hertz with an explicit sign convention.
- Receiver-provided code/phase standard deviations.
- C/N₀, lock time, the complete opaque versioned receiver tracking-status word, and each documented validity field.
- Optional receiver-native half-cycle, parity, loss-of-lock, and cycle-slip indicators only when documented for the captured UM980 firmware/log revision.
- Derived slip/continuity indicators from lock-time resets, geometry-free and Melbourne–Wübbena combinations where available, Doppler/phase consistency, and innovations, each labelled with its derivation revision and provenance.
- Receiver clock data where available.
- Rover/base role.
- Ephemeris issue/version and correction provenance.

Raw tight RTK additionally requires base observations or equivalent RTCM correction content and ephemerides. Rover `OBSVM` data alone is insufficient for re-running RTK.

## Session recording and replay seam

The ability to improve a session later is a first-release requirement, not a future storage concern. `aevia-trajectory-log` defines a canonical, append-only binary artifact with:

- A format major/minor version, session ID/generation, monotonically increasing chunk/record sequence, record length, previous-chunk digest, and per-chunk checksum.
- A start manifest embedding values rather than external registry references: hardware/firmware/engine builds; sensor identities/configurations; UM980 `VERSION`/`CONFIG`/`UNILOGLIST`, port baud/log rates and parser/manual revision; SCH16T register profile; complete `EngineConfig`, `MetricPlan`, compiled `LiveMetricPlan`, installation transforms, calibration coefficients, uncertainty models, numeric/device profile, time basis, requested recording capabilities, and initial-heading/user-control inputs.
- The exact fixed-size `LiveObservation` ingest stream, preserving observation IDs, effective epochs, timing uncertainty, quality, update order, engine batching/profile identity, and all control/reinitialization/re-anchor events needed for behaviorally equivalent replay.
- Complete verified native payloads for every stream advertised as recomputable: SCH16T register words/status/counters and per-frame capture times; complete verified UM980 frames/message versions; PPS/clock-fit evidence and discontinuities. These native records are a recomputation lineage, not a second normalized/live-input stream. A selected decoder must not depend on undocumented fields discarded at capture.
- Raw rover GNSS, ephemerides, and received base/correction content when the selected recording profile enables them.
- Explicit gap, dropped-record, queue-overrun, sensor reset, clock discontinuity, SD error, configuration/control change, user event, navigation reinitialization, anchor change, and session-end records. Each change embeds the new effective values. Missing data must never look like a quiet valid interval.
- Periodic live checkpoints/summaries for replay seeding and comparison. These are derived records and never replace sensor evidence.

Encoding uses declared fixed-width integers, canonical little-endian byte order, finite IEEE-754 fields where floats are unavoidable, and explicit enum discriminants; it never serializes native Rust/C/C++ layout. Per-chunk checksums detect local corruption. An algorithm-tagged cryptographic `ContentDigestV1` covers the canonical checksum-valid logical record stream, session identity/generation, terminal sequence, and span while excluding preallocation padding and rebuildable indexes; result sidecars use this identity. A framing-only minor migration preserves the logical digest, while a semantic reinterpretation creates a new normalization/result digest.

The start manifest advertises requested capabilities only. The reader derives actual capabilities/completeness separately for every checksum-valid contiguous span from present records, explicit gaps, timing quality, and whether a valid end record exists. Host processing rejects only the requested unavailable span/capability; it may explicitly run a lower level and report why. A final index may accelerate access but is rebuildable. Schema evolution follows these rules:

- Unknown optional records are length-skipped and preserved by tools that rewrite/copy an artifact.
- Unknown required records or unsupported major versions fail explicitly.
- Minor-version migrations are deterministic and covered by permanent fixtures.
- Source artifacts are immutable. Imported base-station data, corrected calibration, PPK solutions, and all processed results are linked sidecars identified by canonical digests and provenance.

The firmware storage adapter owns SD/MMC DMA, buffering, filesystem calls, preallocation, and media state. It writes through a bounded queue sized by an explicit worst-case-rate calculation. Before starting a high-accuracy session, firmware verifies free space and sustained write throughput for that profile. If storage falls behind or disappears, live estimation continues and diagnostics immediately mark recording unavailable. Silent overwrite or silent dropping is forbidden.

When recording resumes after an unrecorded interval, the next span begins with an explicit gap plus either:

- `ReplaySeedV1`: the complete versioned engine state required to reproduce subsequent live behavior, including nominal state/covariance, profile-declared shared-parameter consider blocks and cross-covariances, anchor, motion classifier, clock segment, fusion-frontier history, metric accumulators/finalization state, event identities/revisions, and configuration; or
- `NavigationReinitialization`, after which captured replay starts from the same declared initialization inputs.

Without either record, the later span may contain useful evidence for new host processing but cannot advertise captured-replay equivalence. Capability derivation distinguishes `CapturedReplay`, `RecomputedNavigation`, `RawTightEligible`, and `EvidenceOnly` per span.

No-SD operation may retain only a compact versioned `LiveSummary` in a firmware-owned wear-levelled internal-flash/NVS ring written at session finalization, with explicit capacity/eviction and wear budgets. It is not sensor evidence and cannot advertise replay/refinement capability.

The host reader streams records rather than requiring the whole artifact in memory. It supports two normalization modes:

- `Captured`: replay the exact normalized observations used on-device.
- `Recomputed`: regenerate normalized observations from native evidence using an explicitly versioned decoder, timing model, and residual calibration; the selected engine batching profile then regenerates preintegrated batches while every changed model revision is recorded.

The `aevia-core` replay coordinator selects exactly one normalized evidence lineage per source/time span, and the engine verifies that declaration during ingestion. Together they prevent accidental double use of captured versus rebuilt samples/batches, receiver PVT versus the raw measurements that produced it, and original versus PPK replacement trajectories.

An embedded-equivalent host replay over `Captured` observations must reproduce every device disposition/event/revision decision exactly and continuous state fields within declared cross-target numeric tolerances. This is the regression oracle for both the device implementation and future schema migrations.

Result sidecars use a separately versioned immutable artifact envelope owned by `aevia-trajectory-log`. Each has a unique result-revision ID, its own canonical digest, source session/span/content and selected-normalization digests, all external input digests, embedded or content-addressed configuration/calibration/installation/uncertainty/metric definitions, requested policy, actual backend/version, convergence/fallback diagnostics, all parent revisions, and trajectory/metric payloads. Readers accept only complete digest-valid result payloads.

## Installation and physical reference points

`EngineConfig` owns immutable physical truth:

```rust
pub struct Installation {
    pub body_from_imu: SensorToBodyRotation,
    pub imu_to_gnss_antenna: BodyLeverArm,
    pub reference_points: ReferencePointSet,
    pub attachment: AttachmentModel,
    pub dynamics_profile: DynamicsProfileId,
    pub calibration_revision: CalibrationRevision,
}
```

Each transform includes a stable shared-parameter identity and joint uncertainty under the shared-parameter policy above.

The navigation state is located at the SCH16T sensing centre. GNSS factors predict the antenna phase-centre state through the configured lever arm. Gyro measurements are body relative inertial, `ω_ib^b`; rigid-body kinematics and the derivative of `orientation_ecef_from_body` use body relative Earth, expressed in body, `ω_eb^b = ω_ib^b - R_b^e ω_ie^e`. Define `α_eb^b` as the derivative of that same angular velocity represented in the body coordinates. Metric queries transform to a named point using:

- Position includes `R r`.
- Velocity includes `R(ω_eb^b × r)`.
- Acceleration includes `R(α_eb^b × r + ω_eb^b × (ω_eb^b × r))`, with angular acceleration and its uncertainty taken from the same continuous attitude segment. If angular-acceleration quality is insufficient, acceleration at an offset reference point is unavailable rather than silently omitting the tangential term.

`AttachmentModel` is either:

- `RigidBody`: named points and body-axis metrics are permitted.
- `DeviceTrajectoryOnly`: only the instrument package/antenna trajectory is claimed; inferred human centre-of-mass and body-forward metrics are disabled.

`AttachmentModel` states physical rigidity only. A separate immutable `DynamicsProfileId` selects the qualified process-noise schedule, GNSS correlation model, stationary classifier, ZUPT policy, motion/heading observability thresholds, and any permissible constraint for one declared validity envelope. Profiles include unconstrained device, rigid land vehicle, running/walking mount, and ski mount; preflight rejects an attachment/profile/metric combination that was not jointly validated. A constraint is still enabled span by span only while its innovations and observability tests pass.

This matters especially for skiing/running. Research has measured a mean separation around 0.62 m between a skier’s GNSS antenna and reference centre of mass; estimating true human CoM required an additional biomechanical model and sensors. [Alpine ski GNSS/IMU study](https://www.mdpi.com/2072-4292/8/8/671).

## Calibration and model identification

Datasheet values initialize a model; release uncertainty profiles come from the assembled V2 Mini and each supported installation:

- Verify Murata's internally applied factory correction/status, then estimate only residual system bias, scale, non-orthogonality, axis alignment, temperature curves, noise density, bias random walk, gyro g-sensitivity, and any justified correlation time from multi-orientation static, thermal, vibration, and rate/acceleration-reference tests. Use Allan deviation and autocorrelation with documented units; do not map bias-instability directly to random-walk noise. Gyrocompassing uncertainty must include measured residual g-sensitivity because the datasheet limit is material relative to Earth rate. [Kalibr IMU noise-model reference](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
- Measure RATE2/ACC2/ACC3 filter support and relative delay on the assembled sampling path. Store nominal corrections, uncertainty, tested temperature/rate range, and the SCH16T profile ID. Calibrate ACC3 beyond ±80 m/s² only against a traceable high-range reference; otherwise its larger electrical headroom is marked outside the qualified performance range and cannot bridge saturation there.
- Survey IMU sensing-centre to GNSS antenna phase-centre lever arm and boresight with covariance for every rigid installation. Store antenna model/PCO/PCV and receiver/base reference-point definitions when available.
- Characterize UM980 position/velocity latency, shared clock-fit error, per-solution-class covariance floors, position/velocity cross-correlation where observable, and inter-epoch innovation autocorrelation for each firmware/configuration/output-rate profile.
- Validate stationary detection, ZUPT false-positive rate, gyrocompassing convergence/coverage, and dynamic-yaw observability separately for every attachment/motion profile.
- Maintain independent reference datasets: analytic simulation; turntable/static/thermal fixtures; surveyed stationary/straight/circular/3D motion; independent timing gates; and reference GNSS/INS trajectories. Calibration datasets, validation datasets, and acceptance datasets are disjoint.

Every calibration/model bundle is immutable and content-addressed. It records device/sensor serials, firmware/configuration, procedure/software revision, environmental range, fitted parameters/covariance, residual diagnostics, and validity limits. An engine run rejects an incompatible or expired required bundle instead of silently substituting a generic one.

## Live navigation implementation

### State and mechanization

`NavigationMathProfileV1` is a multiplicative ESKF with 15 navigation error states and one complete convention shared by embedded replay, offline smoothing, and graph initialization. A fixed-capacity, profile-declared Schmidt/consider vector augments covariance accounting without becoming navigation state or estimated calibration: it always contains the active clock segment's offset/drift and contains every other material shared installation/calibration/delay parameter that the profile does not cover with a qualified sequence-level bound. Its exact dimension, ordering, covariance, validity spans, and cross-covariance storage are preflighted and recorded. A named offline GNSS-colored-error augmentation, when selected, is additional and recorded explicitly.

The embedded nominal state is `x = (pⁿ, vⁿ, Rⁿ_b, b_a, b_g)` in a fixed-anchor, Earth-fixed ENU frame `n`. `Rⁿ_b` rotates body vectors into ENU. The local axes do not follow the vehicle and do not move with latitude/longitude between re-anchors, so there is no transport-rate term inside one anchor segment. The right-multiplicative error is

`δx = (δpⁿ, δvⁿ, δθᵇ, δb_a, δb_g)`, with `R_true = R_nom Exp([δθᵇ]×)`.

For calibrated measured specific force `fᵇ_m`, calibrated measured inertial angular rate `ωᵇ_m`, Earth rate `ωⁿ_ie`, and normal gravity `gⁿ`:

```text
ṗⁿ   = vⁿ
v̇ⁿ   = Rⁿ_b (fᵇ_m - b_a) + gⁿ(pⁿ) - 2 ωⁿ_ie × vⁿ
Ṙⁿ_b = Rⁿ_b [ωᵇ_m - b_g - Rᵇ_n ωⁿ_ie]×
```

The configured WGS-84 normal-gravity model includes the centrifugal contribution; do not add it again. Its gravity gradient is included in `F` when material over the anchor segment. Bias dynamics are random walks unless board-level Allan-deviation/autocorrelation tests justify a first-order Gauss–Markov correlation time; bias-instability alone is not a process-noise parameter. `NavigationMathProfileV1` owns the matching continuous `F/G`, measurement `H`, error injection, exact SO(3) reset Jacobian, re-anchor Jacobian, and discretization rules. Analytic Jacobians are checked against automatic/central finite differences, and covariance propagation is checked by Monte Carlo fixtures. The convention and reset follow Solà’s ESKF formulation; on-manifold increments and bias Jacobians follow established IMU preintegration. Sources: [Solà ESKF derivation](https://www.iri.upc.edu/files/scidoc/1773-Quaternion-kinematics-for-the-error-state-Kalman-filter.pdf), [Forster IMU preintegration](https://www.roboticsproceedings.org/rss11/p06.html).

The estimator owns an `f64` terrestrial-frame ECEF anchor and converts GNSS observations into fixed ENU at update time. Its hot state/covariance may use validated `f32` or mixed precision. Re-anchor at a deterministic distance/precision threshold with hysteresis and start a new dense segment at the same physical state. Rotate only navigation-frame quantities: nominal position/velocity/attitude representation, `δpⁿ`, `δvⁿ`, their covariance blocks/cross-blocks, Earth/gravity vectors, and cached local coefficients. Right-multiplicative `δθᵇ`, IMU biases, sensor axes, body lever arms, globally defined ECEF gates/reference points, and scalar metric accumulators remain unchanged; local gate coefficients are recomputed from their global definitions. The recorded block Jacobian and round-trip fixtures must preserve the physical ECEF state, covariance, and metric continuity. Host processing uses `f64` and may retain ECEF alongside local linearization frames.

The engine's private preintegrator consumes every valid calibrated SCH16T observation, aligns RATE2/ACC2 support intervals, and produces bounded coning/sculling-corrected batches. Each batch carries `ΔR`, `Δv`, `Δp`, preintegrated measurement covariance, bias/noise Jacobians, the exact bias linearization point and tangent/covariance bases, constituent observation IDs, support interval, correction-validity bound, and quality. The estimator composes the full state-dependent 15-navigation-state transition `Φ` and discrete process covariance `Qd` using the current attitude, force, Earth/gravity, and bias model; these cannot be produced correctly by acquisition. Raw-sample noise is propagated through the declared preintegration scheme, and the filter consumes each corrected increment exactly once—no second midpoint/coning/sculling correction is applied. ACC3 is resampled onto the required acceleration support before substitution and uses its own covariance.

The embedded profile applies batches at a measured navigation cadence initially targeted at 200–400 Hz rather than propagating navigation covariance at every sensor output. Nominal propagation applies the batch's `ΔR/Δv/Δp` through the equations above with exact measured support and SO(3) injection. Covariance and innovation calculations use Cholesky/triangular solves, never explicit inversion; measurement updates use Joseph form. After every propagation/update/reset, symmetrize `P`, enforce profile-defined scale-aware diagonal and Cholesky tolerances, and permit only a fixed number and total magnitude of diagnosed diagonal-regularization attempts. A non-finite value, a negative mode beyond tolerance, or repair above the recorded limit invalidates the navigation segment and triggers reinitialization; platforms use the same deterministic policy. The offline path can reproduce the same batches or rebuild them from preserved high-rate evidence.

`V2MiniLive` defines a 10 ms maximum bridgeable interval without a complete IMU vector. Inside it, propagate the last qualified rate/specific force under a board-validated bounded angular-acceleration/jerk gap model and add its full covariance; mark the span degraded. Beyond 10 ms, or after a common fault/reset, invalidate the navigation segment and reinitialize. Missing data is never replaced by zero, remaining axes, or nominal-rate time.

### Initialization

Use an explicit state machine:

- `Uninitialized`
- `CoarseAligning`
- `FineAligning`
- `Navigating`
- `Degraded`
- `Invalid`

Initialization uses:

- GNSS position and velocity.
- A versioned two-state Bayesian stationary/motion classifier. Its emissions combine a SHOE/GLRT-style windowed gyro/specific-force statistic with the Mahalanobis likelihood of zero GNSS velocity; transition probabilities and dwell/hysteresis are profile-specific. It emits a posterior plus evidence flags, not only a thresholded Boolean. The detector is calibrated separately for each `DynamicsProfileId`; a low false-stationary/false-ZUPT rate is the primary gate. Source: [Bayesian adaptive zero-velocity detection](https://arxiv.org/abs/1903.07929).
- Weighted two-vector static alignment using `mean(ωᵇ) = Rᵇ_n ωⁿ_ie + b_g` and `mean(fᵇ) = -Rᵇ_n gⁿ + b_a` with calibrated bias priors. Gravity supplies roll/pitch; Earth rate may supply north through a TRIAD/Wahba solve when qualified. Static alignment does not make every attitude/bias degree of freedom observable, and the implementation must not both treat stationary gyro mean as zero bias and reuse the same mean for gyrocompassing. [NASA TRIAD/Wahba reference](https://ntrs.nasa.gov/api/citations/19990052720/downloads/19990052720.pdf).
- Zero-velocity updates only during high-confidence, physically stationary rigid-mount intervals. They use a declared covariance and innovation gate; a loose or internally moving device cannot receive a ZUPT merely because GNSS speed is small.
- A supplied heading and covariance when available.
- Stationary Earth-rate gyrocompassing only when duration, latitude, temperature stability, vibration, residual g-sensitivity, per-device bias calibration, predicted heading covariance, and validation gates support it. SCH16T bias-instability specifications make the mode plausible but do not establish its achieved heading accuracy.
- Dynamic yaw alignment only when horizontal accelerations/turns provide sufficient excitation. Over a fixed profile window, stack the accepted measurement sensitivities, whiten them by their full covariance, eliminate position/velocity/bias nuisance directions by a Schur complement, and require both minimum yaw information and posterior-yaw-covariance thresholds. The exact window, singular-value/information threshold, covariance threshold, dwell, and hysteresis are part of `DynamicsProfileId` and are tested against truth; a qualitative “moving” flag is never enough.
- A calibrated rigid-land-vehicle non-holonomic/no-sideslip constraint only while its innovations validate that assumption. It is unavailable for skiing, running, airborne motion, unknown mounting, and vehicle sideslip.

Single-antenna GNSS velocity supplies course over ground, not body heading. Straight constant-speed motion or motion with unknown sideslip cannot initialize body yaw. Over every trajectory span, record `HeadingSource` and `HeadingObservability`; when yaw is not observable, retain its covariance and make body heading, body-axis velocity/acceleration, and heading-dependent reference-point metrics explicitly unavailable while leaving position, vector velocity, and frame-independent metrics usable. Course is available only when horizontal-direction SNR and propagated course variance pass profile thresholds; a valid but near-zero horizontal velocity has no valid course. Sources: [INS/GNSS attitude observability](https://www.ion.org/publications/pdf.cfm?articleID=2161), [non-holonomic-constraint observability](https://www.cpgps.org/wwwroot/issue201201/JoGPS_v11n1p80-88.pdf).

Do not publish timing-grade metrics before position, velocity, attitude, bias, and clock-quality requirements are met.

### Delayed/asynchronous updates

Live delayed fusion consists of one bounded effective-time reorder window, one navigation-finalization watermark, and separate metric-finalization watermarks derived from each compiled metric's bounded future support.

The versioned live profile contains a per-source maximum arrival lateness measured from actual UM980/SCH16T captures plus a guard margin. The engine derives one `fusion_delay` and preallocates a chronological queue and IMU history covering `fusion_delay + history_guard`. Construction reports the measured values, required capacities, and hard profile ceiling, and rejects a configuration that cannot fit.

For latest contiguous trusted IMU time `T`, the corrected navigation frontier target is `W_nav = T - fusion_delay`:

1. Buffer observations by effective epoch while `epoch > W_nav`.
2. Advance the corrected ESKF once in epoch order through all eligible IMU batches and measurements up to `W_nav`.
3. Maintain a separate IMU-propagated present predictor for low-latency display. When the corrected frontier advances, compute its attitude/velocity/position discrepancy from the predictor at the same epoch and transfer that discrepancy to the present with a stable, bounded complementary correction whose time constants and reset thresholds belong to the live profile. The predictor exposes tracking error and `Predicted` quality; it supplies no finalized timing event or covariance-grade claim. This is the PX4-style output-predictor role, not a second estimator.
4. Finalize corrected trajectory states through `W_nav` after all eligible navigation work is complete. This estimator watermark depends only on navigation input ordering and processing, never on recording state.

For metric `m`, let `L_m` be its compiled maximum future support and `S_m` the latest corrected trajectory time its evaluator has completely consumed. Its finalization watermark is `W_metric,m = S_m - L_m`. A backdated launch, change point, stop, gate, or lap-validity decision at time `t` remains provisional until `W_metric,m >= t`; an individual result may equivalently expose `final_after = t + L_m`. No event becomes immutable merely because its timestamp is behind `W_nav`. The update reports each active metric watermark, or a plan-wide minimum plus per-result `final_after`, so bounded retrospective semantics remain explicit.

Any observation whose epoch is at or before an already-processed frontier receives the `TooLateForLive` disposition and cannot mutate live navigation. It remains eligible for host processing when present in the session evidence. Duplicates, invalid frames, and clock discontinuities are transactional input errors; statistical GNSS rejection, lateness, and insufficient observability are dispositions/quality states.

`LiveSession::step` accepts at most one fixed-size observation plus a deterministic corrected-frontier work credit, and may be called with no observation to advance queued work. Credits are capped at `u16::MAX` per call and do not purport to meter fixed-capacity ingestion, transfer/reanchor, metric, or projection phases; exact-board timing of the complete call remains a release gate. The update reports the input disposition, predicted-present projection, bounded metric mutations, diagnostics, navigation-finalization watermark, metric-finalization state, and unused frontier credits. `finish()` irrevocably declares end of input and replaces the normal delayed target with the latest complete trusted IMU epoch `T_end`, because no later delayed observation can then arrive. Repeated bounded drain calls process queued navigation and metric work through `T_end` without extrapolating beyond it. Any work not yet drained and any metric whose declared future support extends past `T_end` remains provisional; the final `fusion_delay` of valid navigation evidence is not silently discarded.

After construction, ingestion, frontier advancement, queries, and live metric updates allocate nothing. Normal work is linear in newly eligible increments/measurements. Construction rejects any rate, lateness, queue, history, or metric contract that cannot fit its workspace and schedule. This follows the established delayed fusion-horizon/output-predictor architecture used by PX4 EKF2 while keeping Aevia's measurement and quality semantics private. Source: [PX4 EKF2 delayed fusion horizon](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf).

### Embedded resource and scheduling gates

Before ESKF implementation is considered viable, record budgets from the complete firmware running on the actual V2 Mini, including acquisition, microSD, display, BLE/radio, and power tasks. Use these initial trajectory-engine ceilings unless a later measured firmware budget decision changes them:

- Accept and record at least 1,750 complete IMU sample sets/s—covering the SCH16T DEC4 maximum of approximately 1.569 kHz plus margin—and up to 50 position plus 50 separately timed velocity updates/s; size actual DMA/queues from measured `FREQ_CNTR` and propagate only at the validated 200–400 Hz device navigation cadence.
- Derive `fusion_delay` from measured per-source latency distributions plus guard margin. Keep `fusion_delay + history_guard` within a 500 ms initial workspace ceiling unless actual-board profiling justifies a new named profile; late data never expands it at runtime.
- At most 192 KiB internal SRAM for live state, stacks, scratch, metric state, and the hot part of history.
- At most 1 MiB PSRAM for the remainder of bounded trajectory history; no correctness-critical operation may assume cache-like PSRAM latency without measurement.
- At most 32 KiB live-task stack high-water, included inside the 192 KiB SRAM ceiling, and 1.5 MiB firmware flash attributable to the live trajectory/metric path.
- At most 64 total configured gates/targets, four active candidates per segment, a profile-declared root-isolation/refinement evaluation budget proven by adversarial fixtures, and 16 metric mutations per step. More general simultaneous-gate scans are host-only.
- Use a separately budgeted recorder queue initially capped at 1 MiB. Before accepting the profile, total the encoded worst-case bytes/s for native IMU, normalized IMU, live increments, solution/raw GNSS, timing, correction, and integrity records and prove the queue covers that rate plus a two-second SD stall; otherwise lower the recording profile or declare a smaller tested stall contract.
- Zero allocations after a session starts and no unbounded recursion or collection growth.
- At configured navigation cadence, p99 total engine work stays below 50% of one inter-arrival interval and the worst observed instrumented time stays below one full interval under simultaneous maximum-rate logging, sustained declared GNSS latency, and normal UI/radio load. Separately, a static operation-count/loop-bound audit proves that every call is bounded; a soak result is never labelled WCET proof.
- No missed IMU DRDY/PPS captures, recorder overruns, watchdogs, or navigation/metric-finalization stalls during a 60-minute maximum-rate hardware soak.

The minimum useful release profile still records the high-rate stream, sustains at least 200 Hz corrected navigation and the maximum declared GNSS update profile, and supports live state, horizontal/3D distance, one ordered lap/sector sequence, and a bounded drag/braking plan. If it cannot pass, the device trajectory release is blocked rather than weakened below that floor.

Report binary size, internal SRAM, PSRAM, stack high-water marks, queue high-water marks, mean/p99/worst-observed execution time, frontier delay, and missed-deadline counters in CI/hardware-test artifacts. Any change to scalar policy, batching/cadence, fusion-delay/history capacity, or live metric set creates a new `V2MiniLive` profile revision and reruns numeric-accuracy, deterministic-replay, resource, and soak gates. Never solve an overrun by silently dropping observations or adding GTSAM dependencies.

### GNSS updates and robustness

In receiver-solution mode:

- Fuse ECEF-referenced position and vector velocity, not scalar speed/heading, after transforming them into the estimator's private state frame.
- Apply the antenna lever arm in both position and velocity measurement models.
- Respect separate position/velocity epochs.
- Linearize position and velocity jointly with respect to the navigation state and shared installation/timing/calibration consider or nuisance parameters. Only irreducible receiver and sample-specific `ω_eb^b`/timing uncertainty enters `R`; lever-arm, boresight, calibration, and delay uncertainty follows the shared-parameter policy and retains its cross-covariances.
- Apply one joint 6-D position/velocity update when a valid cross-covariance and compatible epoch are supplied. Otherwise use the declared conservative separate-update policy.
- Apply configurable covariance floors for each GNSS solution class.
- Use innovation/NIS gating, robust weighting, correction-age limits, and receiver-health checks.
- Detect discontinuities on RTK fixed/float transitions rather than treating `RTK fixed` as proof of correctness.
- Prevent duplicated use of correlated receiver fields.

Any covariance produced after residual-dependent rejection, robust reweighting, constraint enablement, or ambiguity selection is explicitly `ConditionalOnSelection` at every processing level, including live and offline. Qualification measures empirical state/event coverage through the same selection logic; a nominal Gaussian covariance is never presented as unconditional merely because the backend is an ESKF. [GTSAM robust-noise reweighting model](https://gtsam.org/doxygen/a04491.html).

Vehicle non-holonomic constraints are opt-in only for a calibrated rigid land-vehicle profile. They are disabled for skiing, running, cycling, airborne motion, or unknown mounting.

## Full-session host refinement

All full-session refinement is host-only. It consumes an immutable session artifact and produces a new result sidecar through the same trajectory/metric interface; it never changes source observations or the device result.

`OfflineResourceLimits` supplies hard peak-memory, temporary-storage, output-size, worker-count, and optional elapsed-work limits plus cancellation/progress callbacks. Preflight computes a hard bound for offline state/output storage and an explicit estimate for runtime/native graph memory. Phone qualification names the actual device/OS, battery/thermal state, session duration, concurrency, and pass thresholds; “offline” never means unbounded memory or an assumed desktop filesystem.

### Default offline solution-level smoother

The mandatory host path requires only Rust and `std`; it must build and run without GTSAM, CMake, or a C++ toolchain. It:

1. Validates the semantic manifest/capabilities supplied by `aevia-core` and the declared captured or recomputed evidence lineage.
2. Runs an `f64` forward ESKF using the shared measurement, timing, installation, quality, and metric semantics. The base navigation state has 15 errors; clock segments and every material shared installation/calibration/delay parameter are mandatory fixed-mean nuisance/consider blocks unless the profile supplies a qualified sequence-level bound, and a separately qualified GNSS-colored-error profile adds its declared Gauss–Markov states. Embedded-equivalent replay uses the device batching/profile; refined replay may use the preserved higher-rate IMU evidence.
3. Stores filtered and one-step-predicted nominal states/covariances, navigation and estimable colored-error transitions `Φ`, process covariances `Qd`, the static consider-parameter blocks and cross-blocks, error-injection/reset-basis transforms, adjacent cross-covariance, accepted residuals, robust weights, and dispositions through the private `StateStore`. The memory adapter is allowed only when its preflight bound fits; the default long-session adapter is a checksummed seekable temporary chunk store supporting reverse iteration/random access, bounded caching, cleanup, and explicit no-space/corruption errors.
4. Runs a manifold error-state Rauch–Tung–Striebel fixed-interval recursion for the navigation and any explicitly estimable colored-error states, coupled to a Schmidt/consider smoothing recursion for fixed calibration, installation, delay, and clock-segment parameters. At each backward step it forms the augmented covariance/transition blocks, computes corrections only for estimable-state rows, forces every consider-parameter gain and mean correction to zero, leaves its supplied mean and `P_cc` unchanged, and propagates `P_xc` plus requested state/parameter and cross-time covariances. Each next-state residual uses the profile's `boxminus`; Cholesky/triangular solves obtain the smoothing gain; and `boxplus` plus the same SO(3) reset convention injects the estimable correction. All blocks remain in consistent tangent bases. Fixtures compare this recursion with a full augmented linear-Gaussian reference whose consider means are explicitly clamped, proving that shared uncertainty remains correlated across observations/events without allowing `CalibrationPolicy::Fixed` to estimate those parameters or fold them into independent `R` terms.
5. An optional bounded iterated extended Kalman smoother outer loop restarts from source observations, relinearizes the complete dynamics/measurement sequence around the preceding smoothed trajectory, applies robust weights, and preserves the same fixed-mean Schmidt/consider rule on every pass. It accepts a pass only when the declared objective decreases, uses damping/line search when required, and stops on objective/step tolerances or a maximum pass count. A non-convergent pass does not replace the last valid solution.
6. Reconstructs a full continuous trajectory, propagates usable covariance, and recalculates all requested metrics.

This solution-level smoother is the default final processing profile. It provides a practical improvement path even on hosts where native GTSAM integration is unavailable. The recursion follows standard Bayesian filtering/smoothing, and the bounded nonlinear iteration is the established IEKS/Gauss–Newton construction. Sources: [Särkkä and Svensson, Bayesian Filtering and Smoothing](https://users.aalto.fi/~ssarkka/pub/bfs_book_2023_online.pdf), [Bell, iterated Kalman smoother as Gauss–Newton](https://doi.org/10.1137/0804035), [NovAtel forward/reverse processing](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Process_IMU_Data.htm).

More compute does not automatically mean a better result. Each host run reports convergence, residual/innovation diagnostics, changed rejection decisions, uncertainty consistency, and comparison with its parent. A failed or suspect refinement remains a candidate revision and must not be promoted over the live result merely because it used a higher processing level.

### Optional GTSAM graph smoother

The private advanced adapter constructs a nonlinear factor graph initialized from the offline solution. It uses:

- Resource-preflighted adaptive keyframes chosen from elapsed time, rotation, velocity change, IMU linearization error, and requested output accuracy. Correlation-aware GNSS decimation or an established interpolation factor places measurements at their actual epochs without automatically creating 50 navigation states per second.
- GTSAM `NavState`, `PreintegratedImuMeasurements`, `ImuFactor2`, and a separate lower-rate bias Markov chain where—and only where—their equations match the selected navigation profile. `PreintegrationParams::omegaCoriolis` is treated as a supported Coriolis input, not as proof of equivalence to Aevia's Earth-fixed mechanization. Before enabling these facilities for a named profile, equation- and Jacobian-level fixtures compare frame/sign conventions, Earth rotation, Coriolis, normal gravity/centrifugal treatment, position-varying gravity and gradient, and bias propagation with `NavigationMathProfileV1`; each physical term must occur exactly once. Do not use `CombinedImuFactor` by default because current GTSAM guidance recommends separate bias factors.
- Standard `GPSFactor2Arm`-family position factors where applicable, and private independently timed vector-velocity factors.
- A narrowly scoped private inertial/gravity factor whenever any required mechanization term cannot be represented equivalently by GTSAM's established navigation facilities. Every private factor has analytic/automatic versus numerical-Jacobian fixtures and end-to-end equivalence tests against the offline reference.
- GNSS position and velocity factors at their actual epochs.
- Bias evolution factors.
- Optional stationary and vehicle-specific factors.
- Robust loss functions and iterative outlier reclassification.
- Full-session Levenberg–Marquardt relinearization.
- A reverse pass/multi-pass initialization before final optimization.
- Sparse marginal/cross-covariance extraction for trajectory and metric uncertainty.

Keep GTSAM behind `aevia-trajectory`'s private feature-gated adapter and return the same trajectory/result forms as every other processing level. Pin an exact reviewed source revision. Permit either the mutually exclusive matching system build or pinned vendored source. GTSAM supplies sparse graph optimization, manifold navigation, preintegration, robust noise models, and covariance machinery; Aevia owns timing, GNSS preprocessing, metric semantics, result provenance, and the engine interface. Its absence may reject an explicitly required advanced level but cannot prevent validation, replay, offline refinement, comparison, or export. Sources: [GTSAM navigation](https://borglab.github.io/gtsam/navigation/), [preintegration parameters](https://borglab.github.io/gtsam/preintegrationparams/), [GPS lever-arm factors](https://borglab.github.io/gtsam/gpsfactor/).

Advanced preflight reports the proposed state/keyframe/factor counts, estimated sparse nonzeros, peak memory, temporary/output storage, pass limits, and cancellation/progress units for the requested span. It rejects a request beyond the caller's limits before graph construction. Robust-loss marginals and ambiguity-fixed results are labelled conditional on the selected weighting/integer hypothesis and must pass empirical coverage tests.

IMU preintegration and nonlinear smoothing follow [Forster et al.](https://www.roboticsproceedings.org/rss11/p06.html) and [iSAM2](https://doi.org/10.1177/0278364911430419). GTSAM’s published RTK example leaves cycle-slip handling and integer fixing outside its factors and reports occasional harmful incorrect fixes, so it is an optional implementation tool rather than a complete navigation product. [GTSAM RTK factor description](https://gtsam.org/2026/06/10/rtk-gnss-double-difference.html).

The offline smoother accepts `CalibrationPolicy::Fixed` only. Mandatory propagation of every material supplied shared-parameter uncertainty through consider/nuisance blocks is uncertainty accounting, not calibration refinement. Adjusting a fitted timing mean, lever-arm correction, boresight, scale, misalignment, g-sensitivity, or delay coefficient requires solve-for states or a separate batch solve and must not be smuggled into the base navigation recursion. `RefineWithPriors` is an advanced graph capability and returns an explicit unsupported-capability error from the offline profile:

- `Fixed`: production default.
- `RefineWithPriors`: solve timing offset/drift, lever-arm correction, or scale/misalignment only while observable and within supplied priors.

Hold unobservable parameters fixed and report insufficient observability rather than letting them absorb unrelated GNSS/IMU errors.

## Raw tightly coupled RTK/INS

This is a later, explicitly requested processing level, not a device requirement. It remains another private capability of the same engine and returns the same trajectory/result forms. Never feed receiver RTK PVT and the raw measurements that generated it as independent factors.

Implement:

- Per-constellation and per-frequency signal models.
- Receiver clock plus inter-system biases, GLONASS inter-frequency bias where applicable, and documented code/phase hardware biases.
- Satellite state propagation and Sagnac correction.
- Rover/base antenna PCO/PCV, satellite antenna corrections, and carrier-phase wind-up when required by the selected products.
- Orbit/clock source, issue-of-data, validity, and correction-age policy.
- Elevation/C/N₀-dependent weighting.
- Rover/base double-differenced pseudorange and carrier phase as the default ambiguity-bearing phase formulation.
- Explicit rover/base epoch alignment and the complete differenced covariance induced by shared satellites/reference observations.
- Rover Doppler factors with receiver clock-drift state.
- TDCP factors only on explicitly selected samples/spans not already used by ambiguity-bearing carrier-phase factors, unless their full joint covariance is retained. Adjacent TDCP factors model the shared middle-epoch phase noise; a carrier-phase sample may never enter twice under an independence assumption.
- Tropospheric and residual ionospheric modeling appropriate to baseline length.
- Cycle-slip detection from receiver flags, lock continuity, geometry-free combinations, Melbourne–Wübbena where available, Doppler/phase consistency, and innovation checks.
- Physical ambiguity arcs end only on an actual slip, loss of lock, clock jump, or validated discontinuity. A reference-satellite change algebraically re-references the ambiguity mean and full covariance; it does not create a new physical arc.
- Float ambiguities inside the graph.
- LAMBDA/MLAMBDA integer resolution outside the graph.
- Ratio, success-rate, residual, and temporal-consistency validation.
- Partial ambiguity fixing when a full fix is unsafe.
- Fix-and-hold only after validation; immediate reset on slip or integrity failure.
- Satellite/signal exclusion and reinstatement state machines.
- Receiver PVT only as initialization and a dependent diagnostic in this mode; it is not independent evidence when derived from the same raw measurements.

The carrier-phase/TDCP covariance and exclusivity rules follow the standard TDCP differencing model, in which neighboring differences share an epoch and its phase noise. [Carrier-phase/TDCP factor covariance derivation](https://www.iri.upc.edu/files/scidoc/3067-Single-Frequency-GNSS-Carrier-Phase-Cycle-Slip-Detection-and-Identification-Using-a-Factor-Graph-Approach.pdf).

Study GICI-LIB, OB_GINS, RTKLIB, and GTSAM as independent references; do not copy GPL code from GICI-LIB or OB_GINS. GICI-LIB is especially useful as a comparison for factor-graph GNSS/INS modes and detailed GNSS error treatment. [GICI-LIB paper](https://arxiv.org/abs/2306.13268), [GICI-LIB source](https://github.com/chichengcn/gici-open), [OB_GINS](https://github.com/i2Nav-WHU/OB_GINS), [RTKLIB](https://rtklib.com).

Prefer undifferenced or single-difference ambiguity states internally so reference changes are a linear reparameterization rather than a reset; preserve the full transformed covariance. This follows the reference-switch-robust formulation described by [GICI-LIB](https://arxiv.org/abs/2306.13268). Base coordinates and their covariance must already satisfy the terrestrial-frame/epoch contract.

Deep/ultra-tight correlator or tracking-loop integration is out of scope because the UM980 normalized interface does not expose correlator-level measurements.

## Continuous trajectory representation

Do not fit an arbitrary smoothing spline through GNSS positions.

Every private dense segment has one state function and analytic derivatives. It must satisfy `dp/dt = v`; returned kinematic acceleration is `dv/dt`; and returned angular rate is `ω_eb^b`, the derivative of the same `orientation_ecef_from_body` SO(3) curve. Its angular acceleration is the derivative of that same Earth-relative rate. The measured gyro's `ω_ib^b` and accelerometer specific force remain separately named sensor/model quantities. Endpoint state, derivative, quality, and reference-frame continuity are checked whenever a segment is created.

The embedded live trajectory is rolling and causal:

1. Retain only configured recent checkpoints, increments, interpolation segments, and event brackets.
2. Build cubic Hermite position/velocity dense output from the corrected endpoint states, and an SO(3) exponential correction of Earth-relative integrated attitude whose endpoint attitudes match. Kinematic acceleration, `ω_eb^b`, and `α_eb^b` are evaluated from these same curves.
3. Build segments only behind the corrected frontier; states at or before the navigation-finalization watermark are immutable. Events remain provisional until their own compiled future support has closed and their metric-finalization watermark passes them.
4. Drain finalized summaries/events to the caller and recorder instead of retaining a full dense session.

After host smoothing:

1. Retain optimized states and biases at estimator keyframes.
2. Reintegrate corrected IMU data to reconstruct state at actual IMU epochs.
3. Condition the dense error trajectory on both smoothed endpoints using the stored linearized transition/process model, then apply its position, velocity, and SO(3) corrections to the reintegrated nominal trajectory. This is the continuous-discrete smoothing/GP-bridge construction, not an independent fitted path.
4. Evaluate dense covariance from the joint endpoint covariance, interpolation Jacobians, and conditional process covariance. Never linearly interpolate covariance entries.
5. Retain the transition/cross-covariance operations required for requested event pairs and integrated functionals without materializing an all-times covariance matrix.
6. Propagate quality, observability, clock segment, and reference-frame transitions with the same segment boundaries.
7. Keep display resampling and polyline simplification completely separate.

The refined trajectory is logically full-session but may be indexed, chunk-backed, or disk-backed. Neither refinement nor queries require every raw observation, dense state, or dense covariance to be materialized simultaneously.

Both paths implement the same kinematic query contract while allowing the host to use its stronger conditional bridge. Sparse continuous-time GP interpolation generated by an SDE is equivalent to classical smoothing at support states in the linear case and provides the established covariance construction used here. Sources: [Barfoot, Tong, and Särkkä](https://www.roboticsproceedings.org/rss10/p01.pdf), [continuous-time trajectory comparison](https://arxiv.org/abs/2402.00399).

## Trajectory outputs

`state_at` returns:

```rust
pub struct KinematicEstimate {
    pub time: SessionTime,
    pub reference_point: ReferencePointId,
    pub frame: OutputFrame,
    pub position: PositionEstimate,
    pub velocity: VelocityEstimate,
    pub orientation_ecef_from_body: AttitudeEstimate,
    pub angular_rate_body_relative_ecef: AngularRateEstimate,
    pub angular_acceleration_body_relative_ecef: AngularAccelerationEstimate,
    pub kinematic_acceleration: AccelerationEstimate,
    pub specific_force_body: SpecificForceEstimate,
    pub covariance: KinematicCovariance,
    pub quality: EstimateQuality,
    pub observability: ObservabilityReport,
    pub revision: TrajectoryRevision,
}
```

For horizontal, vertical, east/north, and course quantities, compute the instantaneous ellipsoidal normal `u(p_ecef)` and geodetic tangent basis at the selected reference-point ECEF position. Use `v_up = uᵀv`, `v_horizontal = v - u v_up`, and the recorded terrestrial ellipsoid/coordinate operation. Propagation may use fixed-anchor ENU, but metric/output semantics never use its stale tangent plane; therefore re-anchor thresholds cannot change distance, vertical speed, or course. Near the geodetic-pole singularity, course/east-north output requires a declared alternate local-frame convention or is unavailable. Source: [PROJ topocentric conversion](https://proj.org/en/stable/operations/conversions/topocentric.html).

Expose derived, explicitly named quantities:

- Horizontal ground speed.
- Full 3D speed.
- Signed vertical speed.
- Body longitudinal/lateral/vertical velocity.
- Course over ground only above the horizontal-direction SNR/variance gate.
- Body heading.
- Longitudinal/lateral/vertical kinematic acceleration.
- Specific force, separately labelled.
- Yaw/roll/pitch rates.
- Angular acceleration when supported by the segment bandwidth/uncertainty model.
- Slope/grade with its definition and quality.
- GNSS/IMU contribution and outage age.
- Provisional versus finalized state.

Each derived quantity declares required observability. Course over ground requires horizontal-direction SNR and course variance to pass their gates; body heading and body-axis projections require observable yaw; offset-reference acceleration additionally requires angular acceleration. A query returns the available quantities plus typed per-field unavailability instead of converting a weak attitude into apparently valid body-axis output.

Bias estimates and solver residuals belong in diagnostic snapshots, not the primary kinematic state.

The embedded `LiveUpdate` carries only the state/uncertainty projections and bounded metric mutations needed by live consumers; it must not copy a full covariance or full metric collection at IMU rate. A full `KinematicEstimate` is produced only for an explicit bounded-rate query or finalized record. Host processing may materialize richer covariance diagnostics.

## Metric engine

Metric definitions and result semantics are shared, but execution is level-specific. Every `MetricPlan` declares required capabilities and fixed maxima for gates, targets, splits, and provisional events. Live-session construction rejects unsupported or over-capacity plans instead of allocating, scanning unbounded history, or silently downgrading.

The first embedded profile incrementally supports current kinematics, horizontal/3D path distance, elapsed/moving time, bounded lap/sector gates, and bounded drag/acceleration/braking targets. It examines each newly finalized corrected segment once and drains finalized events. Full-session ascent/descent reconstruction, exhaustive resampling-independent scans, ski HMM segmentation, global recomputation, and temporally correlated metric uncertainty are initially host-only.

### Common numerical primitives

Implement metrics inside the engine against its private sequential dense-segment cursor with:

- Adaptive Gauss–Kronrod integration with absolute/relative tolerances and an explicit accumulated numerical-error estimate.
- Root isolation over every dense segment before refinement; never use endpoint signs alone as a detector.
- Safeguarded Brent or TOMS 748 refinement only after a candidate interval has been isolated.
- Numerical tolerances below the underlying trajectory uncertainty.
- Event scanning over every dense estimator interval so crossings are not limited to 50 Hz GNSS or exported sample rates.
- Stable event IDs and revision numbers.

Dense intervals own roots half-open: interval `i` owns `[t_i, t_{i+1})`, and the final interval alone includes the session's terminal endpoint. A candidate exactly on a shared boundary is evaluated/emitted only by the right-hand owner. The engine uses metric-definition digest, canonical estimator-interval ID, ordered isolated-root ordinal, and direction as an internal candidate/deduplication key—not as a published identity and never as a rounded floating timestamp. A published live `EventId`/`LiveResultId` is instead an opaque pair of run namespace and monotonic allocation counter, assigned on first emission, immutable across every later upsert/finalize/withdrawal, and never reused. Captured replay deterministically reproduces the run namespace and allocation order. Gate/target, direction, occurrence, lap, and sector indices are revisioned value fields rather than identity, because retrospective validation can backdate, renumber, split, merge, or withdraw them. This keeps traversal chunking and topology revisions from duplicating an event or changing the identifier already observed by a consumer.

For an `EmbeddedHermite` segment at the IMU/navigation point, ECEF position is a constant affine transform of cubic-Hermite position and remains cubic; a fixed ECEF gate-plane function is therefore cubic. Its 3-D squared-speed threshold `v(t)·v(t) - s²` is quartic. Partition these polynomials at every real derivative root, enumerate endpoints, sign-changing intervals, and zero/near-zero stationary candidates, and only then apply finite-gate, direction, and rearm tests. Brent/TOMS 748 refines a proven sign-changing bracket; it is never used to claim that an unbracketed tangent is absent.

All release non-polynomial value and derivative bounds below must be produced by the qualified private `EnclosureV1` backend; Boost/MPFR are independent test oracles and never firmware dependencies. The implementation checkpoint above records the current fail-closed development state and must not be read as an `EnclosureV1` completion claim.

Every `HostConditionalBridge` root uses the validated adaptive isolator even at the IMU/navigation point because its reintegrated nominal plus conditional LTV/GP correction is generally non-polynomial. An embedded offset reference point adds `R(t)r` to position and `R(t)(ω_eb^b(t) × r)` to velocity; instantaneous-tangent horizontal speed also depends on the ellipsoid normal `u(p(t))`, and body-longitudinal speed depends on `R(t)ᵀ`, so these embedded functions are non-polynomial as well. Isolate all such roots with outward-rounded interval branch-and-bound over the dense segment: enclose the function and first derivative from the stored continuous model on each subinterval, discard only when zero is excluded, use derivative-sign bounds to prove monotonic brackets, and recursively isolate derivative/stationary candidates for possible even-multiplicity contact. A cell that still contains zero and an unresolved stationary point at the depth/width/evaluation limit is `Ambiguous`, never “no event.” Live support is enabled only for declared maximum segment duration, lever arm, angular rate/acceleration, and a fixed subdivision budget that passes adversarial interval-enclosure fixtures; host processing continues adaptively within `OfflineResourceLimits`.

Use the equation that matches the requested speed: 3-D norm uses `||v||²-s²`; instantaneous horizontal norm uses `||v-u(uᵀv)||²-s²`; signed body-longitudinal speed uses `e_xᵀRᵀv-s`; and its magnitude uses `(e_xᵀRᵀv)²-s²` plus the requested sign/direction test. Absolute-value distance integrands split at every isolated sign root so quadrature never crosses an undiscovered cusp. Only integrals with nonnegative integrands—horizontal path, spatial 3-D, and absolute longitudinal distance—are monotone and permit one ordered bracket after launch. Signed longitudinal displacement can reverse; its targets require exhaustive root isolation and are host-only unless a bounded live plan explicitly proves monotonicity for the active span.

The embedded implementation uses the same physical functions, isolation rules, and event definitions for its declared capabilities, but updates fixed accumulators per finalized corrected segment, evaluates only the next ordered gate/target set, and retains at most four active candidates. Polynomial origin-point gates/3-D speed use coefficient/derivative bounds; non-polynomial offset/horizontal/body functions use the fixed interval-subdivision bounds above; path integration uses a fixed evaluation count; and each isolated root uses the profile's fixed refinement limit. Failure to isolate/refine inside that budget becomes ambiguous/degraded and is left for host evaluation. A mutation-output overflow marks affected live metrics degraded/withdrawn while navigation continues; a plan expected to overflow under its declared profile is rejected at construction. These rules follow the standard separation between root isolation and bracketed refinement used by established numerical libraries. Sources: [Boost bracketed root tools](https://www.boost.org/doc/libs/release/libs/math/doc/html/math_toolkit/roots_noderiv.html), [SciPy Brent requirements](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.brentq.html), [Boost interval arithmetic](https://www.boost.org/doc/libs/release/libs/numeric/interval/doc/interval.htm), [Boost Gauss–Kronrod quadrature](https://www.boost.org/doc/libs/release/libs/math/doc/html/math_toolkit/gauss_kronrod.html).

Distance definitions:

- `HorizontalPath = ∫ ||v - u(p)(u(p)ᵀv)|| dt` in the instantaneous ellipsoidal tangent plane.
- `Spatial3d = ∫ ||v|| dt`
- `BodyLongitudinalSigned = ∫ v_body,x dt`
- `BodyLongitudinalAbsolute = ∫ |v_body,x| dt`
- `Ascent` and `Descent` from the refined vertical trajectory with stationary/motion classification.
- Optional surveyed-course distance as a separate map/course projection product.

Do not calculate any canonical metric from a displayed or exported position polyline. This matches established automotive practice of integrating Doppler-derived speed for distance. [Racelogic explanation](https://en.racelogic.support/automotive/kb/how-does-gps-work/), [Racelogic channel definitions](https://racelogic.support/automotive/kb/channel-definitions/).

Near zero speed, the live classifier controls ZUPT eligibility and quality only; it is not multiplied with the ESKF velocity marginal as though those reused measurements were independent. Live distance may hold exactly zero only during a high-confidence stationary span with an accepted ZUPT and a validated false-hold bound; otherwise it integrates the trajectory estimate and reports the profile's empirically measured near-zero bias/quality. A numeric mixture uncertainty is unavailable unless a host processing level runs a jointly estimated switching/IMM state-and-motion smoother over the evidence. Source: [interacting multiple-model estimation](https://doi.org/10.1109/7.18693).

### Lap and sector timing

`LapPlan` contains:

- Named finite oriented gates.
- Gate centre, normal, width, height, frame, and survey covariance.
- Allowed crossing direction.
- Selected physical reference point.
- Minimum normal crossing speed.
- Hysteresis/rearm distance.
- Minimum crossing interval.
- Ordered sector rules and lap validity rules.
- Explicit crossing-kinematics outputs: any requested `InstantaneousHorizontal`, `Spatial3d`, `BodyLongitudinalSigned`, or `BodyLongitudinalMagnitude` speed, and either `CourseOverGround`, `BodyHeading`, or neither for direction.

For each candidate interval, solve the signed gate-plane function continuously, confirm the root lies inside the finite gate, enforce direction/rearm rules, and evaluate state at the root.

`LapReport` contains:

- Gate crossing time and uncertainty.
- Lap and sector duration.
- Each requested named crossing-speed quantity and its uncertainty; no generic crossing speed field.
- Selected `CourseOverGround` and/or `BodyHeading` with observability and typed unavailability; the two are never aliases.
- Reference point and gate survey definition.
- Provisional/final revision.
- Invalid/missed/ambiguous gate diagnostics.

Tangential contact or poorly conditioned crossings are marked ambiguous instead of fabricated.

### Drag, acceleration, and braking

`DragPlan` requires explicit semantics:

- `LaunchRule`:
  - first sustained motion,
  - configured low-speed threshold,
  - acceleration change point,
  - or normalized external timestamp if future hardware supplies one.
- Launch dwell/hysteresis.
- `Rollout::None` or an explicit offset in a named `DistanceQuantity`, including whether the offset is nonnegative path length or signed displacement.
- `InstantaneousHorizontal`, `Spatial3d`, `BodyLongitudinalSigned`, or `BodyLongitudinalMagnitude` speed.
- Ascending or descending speed targets.
- Distance targets, each naming `HorizontalPath`, `Spatial3d`, `BodyLongitudinalSigned`, or `BodyLongitudinalAbsolute` and its path-length versus signed-displacement semantics.
- Stop threshold and dwell.
- Selected reference point.

No API may represent an undefined instantaneous “crossing of exactly zero” as objective truth.

Implement:

- 0–100 and arbitrary speed intervals by continuous speed-root solving.
- 100–0 and braking events with descending roots and stop dwell.
- The `QuarterMileHorizontalPath` template at exactly 402.336 m by rooting cumulative `HorizontalPath`; choosing a different named distance quantity produces a differently labelled result, never an ambiguous “quarter mile.”
- Standing 100 m/200 m/500 m/kilometre templates with an explicit named distance quantity.
- The selected named terminal speed and its matching time derivative at each event.
- `HorizontalPath` one-foot rollout at exactly 0.3048 m, or an arbitrary explicitly named distance-quantity offset.
- Separate elapsed time, rollout-adjusted time, distance, and speed uncertainties.

V2 Mini has no routed UM980 EVENT input, so official staging-beam equivalence is not claimed without another timestamped hardware source.

### Activity tracking

`ActivityPlan` selects:

- Horizontal and/or 3D distance.
- Elapsed and moving time.
- Elevation/ascent/descent definition.
- Reference point.
- Moving/stationary probability thresholds.
- Split definitions.
- Optional cycling/running profile.
- Each requested speed summary quantity (`InstantaneousHorizontal`, `Spatial3d`, or a body-longitudinal quantity) and its peak evaluation bandwidth/duration.

`ActivityReport` contains:

- Horizontal and 3D distance separately.
- Elapsed and moving time.
- Average moving and elapsed speed separately for each selected named distance/speed quantity.
- Maximum of each selected named speed quantity with declared evaluation bandwidth/duration.
- Ascent, descent, minimum/maximum elevation.
- Splits/segments.
- GNSS outage and degraded-distance intervals.
- Sensor-point semantics when body/CoM cannot be inferred.

On-device, initially provide elapsed/moving counters, horizontal/3D distance, and bounded split/peak summaries that fit the selected profile. Final ascent/descent, outage reconstruction, and any whole-session reclassification are host results.

Do not map-snap by default. A future map/course projection is a separate derived result and cannot replace the physical trajectory.

### Skiing

Implement the versioned probabilistic/HMM segmentation model first in host processing, using trajectory velocity, vertical motion, attitude dynamics, and IMU activity:

- `Stationary`
- `Downhill`
- `Ascent`
- `Lift`
- `Other`

`SkiReport` contains:

- Individual downhill runs and lift/ascent segments.
- Run elapsed and moving time.
- Horizontal and 3D distance.
- Vertical descent.
- Horizontal-norm and/or 3-D-norm average and peak speeds as separately named configured outputs, each with its evaluation bandwidth/duration; body-longitudinal speed is absent unless explicitly requested and observable.
- Start/end elevation.
- Transition confidence and degraded intervals.

The default reference is the device/antenna package. True skier centre-of-mass output requires a separately supplied biomechanical model and must not be inferred from one rigid sensor package.

The device may display current motion/elevation and basic activity totals, but it must not label live ski/lift/run segmentation as finalized until a separately profiled bounded live model is implemented and passes the embedded resource gates.

## Uncertainty, revision, and integrity

Every state and metric result carries:

- Physical reference point.
- Measurement definition.
- Time and coordinate frame.
- Live or refined provenance.
- Numeric uncertainty when estimable.
- Quality/integrity classification.
- Contributing sensor modes.
- Relevant diagnostic counts.

For an implicit event `h(t, x, θ) = 0`, compute first-order event sensitivity from the implicit-function rule `δt = -(h_x δx + h_θ δθ) / (dh/dt)`. A gate therefore divides the joint trajectory, gate-survey, installation, calibration/delay, and shared-clock variance projected along the gate normal—including every declared state-parameter and parameter-parameter cross-term—by squared normal crossing speed. Speed events use the corresponding speed slope. Low derivatives, multiple hypotheses, and materially nonlinear uncertainty are marked ill-conditioned.

Lap/sector duration uses the joint event covariance:

`Var(t₂ - t₁) = Var(t₂) + Var(t₁) - 2 Cov(t₁, t₂)`.

Offline processing obtains the required state-state, state-shared-parameter, and shared-parameter cross-covariances recursively from the fixed-interval smoother and its nuisance blocks; a declared independence may zero a cross-term only when the evidence contract justifies it. In particular, `Cov(t₁,t₂)` includes `Jθ₁ Pθ Jθ₂ᵀ` and the trajectory/parameter cross-terms whenever both events reuse a gate, installation, calibration, frame, or timing parameter. Integrated distance/moving-time uncertainty uses augmented scalar accumulators and their state/parameter cross-covariance; host posterior sampling is available only from a joint motion/state model and is otherwise reserved for nonlinear event ambiguity. Every level labels covariance after residual gating, robust reweighting, constraint selection, or ambiguity fixing as conditional on that selected history/hypothesis and must pass empirical coverage tests through the same decisions. No level sums per-sample variances as if time samples were independent, combines separately derived stationary/velocity marginals, linearly interpolates covariance entries, or materializes an `O(N²)` covariance matrix. If the required correlation or joint distribution cannot be justified, numeric uncertainty is unavailable with a reason. Source: [JCGM 102 propagation through implicit models](https://www.bipm.org/documents/20126/2071204/JCGM_102_2011_E.pdf).

Distinguish live mutation revisions inside one run from immutable processing revisions across runs. Every processed result bundle records:

- Source session ID, checksum-valid recorded span, and content digest.
- Captured or recomputed selected-normalization digest/revision and all external base/correction/ephemeris digests.
- Engine, actual backend, configuration, calibration, installation, uncertainty-model, and metric-plan revisions/digests.
- Embedded-live, captured-replay, offline-refined, graph-refined, or raw-refined processing level.
- All parent result revisions, convergence/fallback status, and processed time span.

Device live output, offline refinement, and advanced refinement are separate revisions; none overwrites another. Result comparison first requires matching definitions and reference points, then performs order-preserving association by gate/target ID, direction, lap/sector structure, and declared time/uncertainty windows. A separate comparison-sidecar `EventAssociation` table contains explicit parent-ID and child-ID sets plus `New`, `Withdrawn`, `Retimed`, `Split`, `Merged`, or `Ambiguous`: `New` has no parent, `Withdrawn` has no child, one-to-one retiming names both, and split/merge records name every participant. Thus a nonexistent child is never asked to carry a withdrawal, topology changes never inherit an ID silently, and ambiguous association remains explicit. Associated events report value, time, uncertainty, and quality differences. Different gates, reference points, calibration, measurement definitions, or spans are marked non-comparable.

A checksummed host result sidecar may be reimported transactionally after verifying the source session digest and definitions. The device/app may display a compact refined summary or trajectory without understanding the producing solver. The original live result remains available.

Report timing events as ill-conditioned when:

- Gate normal speed is too small.
- Speed slope near a threshold is too small.
- The crossing lies inside an IMU/GNSS gap.
- Time uncertainty dominates spatial uncertainty.
- Multiple roots cannot be disambiguated.

Quality dimensions remain separate:

- `EstimateStage`: predicted, provisional, finalized.
- `Validity`: nominal, degraded, invalid.
- `GnssState`: fixed, float, standalone, absent, suspect.
- `TimingQuality`: PPS-correlated, modeled, arrival-only, discontinuous.
- `HeadingObservability`: supplied, gyrocompassed, dynamically aligned, constrained, or unobservable.
- `Integrity`: monitored or unavailable.

Do not label covariance as a certified protection level. Formal integrity requires a separately defined fault/threat model.

`LiveUpdate` reports revision operations:

```rust
pub enum MetricMutation {
    Upsert { id: LiveResultId, revision: u64, value: MetricResult },
    Withdraw { id: LiveResultId, revision: u64, reason: WithdrawalReason },
    Finalize { id: LiveResultId, revision: u64 },
}
```

`LiveResultId` is allocated exactly once at first `Upsert` and remains addressable as a tombstone after withdrawal; the bounded result plan preflights IDs/tombstones as well as active values. The update also reports the corrected trajectory interval added by frontier advancement, the navigation-finalization time through which states are immutable, and the metric-finalization watermark or `final_after` bound governing each event.

## Failure handling

The engine surface separates contract errors from estimator dispositions:

- `PrepareError`: invalid definitions, incompatible frame/calibration/profile, unavailable evidence/capability, invalid feature/target, or insufficient/aligned resources. No run is created.
- `StepError`: duplicate/non-monotonic source sequence, invalid value/frame/time/clock transition, or violated caller/workspace contract. The step is transactional and engine state is unchanged.
- `ProcessError`: invalid/incomplete requested evidence, resource/storage exhaustion, cancellation, numerical non-convergence, or a required advanced capability failure.
- `QueryError`: outside available span, unavailable frame/reference point/observability, invalid request, or private backing-store failure.
- `InputDisposition`: fused, statistically rejected, downweighted, `TooLateForLive`, initialization-only, or retained-for-host. These are reported outcomes rather than interface failures.

Operational responses are explicit:

- SafeSPI frame/protocol fault: invalidate the complete affected IMU epoch, preserve the raw fault evidence, and run the bounded response-pipeline/DRY resynchronization state machine. Resume only after CRC/address/status checks and required-register/DRY state are proven; otherwise reset and reinitialize the SCH16T.
- IMU gap: reject any incomplete vector. Bridge at most the profile's 10 ms bound with the qualified gap model and increased covariance; beyond it invalidate/reinitialize. ACC3 substitutes an ACC2 component only after support alignment, range, and uncertainty validation.
- GNSS outage: continue inertial propagation, transition to degraded, and expose uncertainty growth. RTK fixed/float transitions preserve continuity only when residuals support it; multipath/outliers are rejected or robustly downweighted with a reason.
- Clock discontinuity/PPS loss: end the clock segment, transition timing quality, and withhold precision timing until a new model qualifies.
- Observation at or before the finalized frontier: report `TooLateForLive` without mutation; recorded evidence remains available to host processing.
- Numerical divergence: invalidate the current navigation segment, withdraw affected provisional events, and re-enter initialization inside the same session while evidence capture continues.
- Loose/moving mount or unobservable yaw: disable rigid-reference/body-axis claims while retaining supported instrument-frame quantities.
- Cycle slip: end the physical ambiguity arc immediately. Reference-satellite change alone applies covariance-preserving re-referencing.
- Live overload: preserve acquisition priority, mark a stale/gap interval, degrade or withdraw affected live metrics, and keep work bounded.
- Recording interruption: keep live navigation running and expose an evidence discontinuity. Later span capabilities follow the replay-seed/reinitialization rules.
- Optional graph failure: discard the partial candidate. `BestQualified` restarts the next permitted level from canonical evidence and records the attempt; `Require(AdvancedGraph)` returns `ProcessError`.

## Implementation sequence

0. **Establish the target and engine skeleton**
   - Add `aevia-trajectory`, acquisition, and log crates; implement the two engine workflows, feature/target guards, typed capabilities/errors, opaque trajectory handle, and separate firmware/host CI invocations.
   - Materialize the accepted `V2MiniLive` profile before estimator qualification: exact UM980 firmware/commands/signals/messages/rates/UART byte budgets, verified SCH16T register/read schedule, numeric/math backend, dynamics/calibration IDs, resource limits, and `QualificationSpecV1`. Compare the DEC4/LPF3 candidate against the documented interpolated-rate reference and select by measured navigation error plus complete-system resource margin.
   - Cross-compile and flash the allocator-free engine/math/timer/DMA integration on the actual ESP32-S31; record link map, flash, SRAM/PSRAM, stack, interrupt latency, and sustained recorder throughput.
   - Implement host `f64` reference oracles for SO(3), mechanization, `Φ/Qd`, measurement Jacobians, covariance reset, re-anchoring, preintegration, and root isolation before enabling mixed precision.

1. **Implement evidence, time, frame, and calibration semantics**
   - Add exact local/GPS time and clock segments, terrestrial-frame realizations/epochs, observations, installation/reference points, typed dynamics profiles, uncertainty models, quality/observability, metric definitions, capacity semantics, and provenance.
   - Add UM980 solution/raw and SCH16T native/normalized records, separate position/velocity epochs, RATE2/ACC2-only data counters, `FREQ_CNTR`, per-axis ACC3 read epochs/supports, SafeSPI fault evidence, sensor profiles, calibration bundles, and PPS/DRY clock evidence.
   - Implement canonical framing/digests/checksums, captured versus recomputed normalization, evidence-lineage exclusivity, streaming readers/writers, replay seeds, and stable event/result identities. Property/fuzz-test arbitrary chunking, malformed lengths/counts, integer overflow, non-finite values, capacity bombs, and every timer/counter rollover with no panic or unbounded allocation.

2. **Build the embedded vertical slice**
   - Implement engine-owned support-aligned IMU preintegration, `NavigationMathProfileV1`, profile-declared shared-parameter consider covariance, complete-vector/gap policy, covariance-repair policy, motion classification/static alignment, vector position/velocity updates, temporal-correlation policy, robust gating, re-anchoring, and current-state output.
   - Add the single delayed-fusion frontier, predicted-present output, bounded `step`, end-of-session drain, navigation and compiled metric finalization watermarks, reinitialization, and diagnostics.
   - Add horizontal speed/distance and one gate/event plan through the opaque trajectory/metric interface, then pass the actual-board numeric, allocation, resource, and soak gates.

3. **Complete live trajectory and priority measurements**
   - Add Earth-relative reference-point transforms, rolling Hermite/SO(3) dense output, instantaneous-ellipsoid horizontal/vertical semantics, bounded polynomial and interval-subdivision root isolation, stationary quality, bounded lap/sector and drag/braking plans, activity counters, event uncertainty, stable revisions, and finalization.
   - Verify unsupported/unobservable/oversized plans fail during preflight and live overload never compromises acquisition or valid navigation state.

4. **Complete offline processing in the same engine**
   - Reproduce captured embedded processing within the numeric tolerance table.
   - Add `f64` forward ESKF with all required fixed-mean shared-parameter Schmidt/consider blocks and any qualified named estimable GNSS-colored-error augmentation, memory/seekable `StateStore` adapters, manifold RTS plus the consider smoother, bounded IEKS passes, conditional dense reconstruction, cross-covariance, posterior sampling only from a justified joint model, and full refined metrics.
   - Add result sidecars, comparison, fallback, cancellation/progress, and transactional reimport. Preflight a 60-minute mobile/workstation session for peak memory, temporary storage, output size, and estimated runtime.

5. **Integrate consumers**
   - Let `aevia-core` compose acquisition, recording, live engine, replay, and processed revisions through the estimator interface; it contains no estimator algorithm.
   - Keep firmware responsible for physical sensor/storage adapters and PPS/DRY capture, and `app/rust::api` responsible for FRB DTO conversion.
   - Do not modify or depend on the full-size V2 design.

6. **Add advanced processing capabilities**
   - Pin and integrate the private GTSAM adapter using the same evidence, definitions, engine request, trajectory, metric, quality, and provenance forms.
   - Qualify it independently against offline processing; `BestQualified` may select it only for operating conditions where it improves the declared accuracy/coverage gates, and fallback restarts from canonical evidence.
   - Add raw tight RTK/INS when complete rover/base/correction/ephemeris evidence is present, including exclusive carrier-phase/TDCP lineage, ambiguity continuity/re-referencing, LAMBDA/MLAMBDA validation, partial fixing, and integrity diagnostics without double-counting receiver PVT or phase samples.

## Qualification contract

`QualificationSpecV1` is an immutable release input completed before tuning data is unblinded. A processing/profile combination cannot be called qualified while any required field is qualitative or blank. It contains numeric pass/fail values for:

- Independent session counts, durations, environment/motion strata, train/calibration/validation/acceptance splits, and the traceable reference system's accuracy, alignment, latency, and valid-data masks.
- State and event absolute/RMSE/percentile errors, hard failure rate, outage behavior, cross-target numeric envelope, root time/value residual, quadrature error, and re-anchor invariance.
- NIS/NEES significance and empirical coverage intervals, innovation autocorrelation limits, robust-selection conditional coverage, and covariance-repair count/magnitude.
- False-stationary/ZUPT and false heading/course/body-output availability rates with stated binomial confidence bounds, plus gyrocompass/dynamic-heading convergence and covariance coverage.
- Decoder/property/fuzz case counts and runtime, rollover coverage, numerical-Jacobian/Monte-Carlo sample counts, adversarial root cases, and all resource/operation-count/worst-observed timing limits.

Thresholds are derived from the product error budget, sensor calibration, truth-system uncertainty, and measured target limits; they are versioned with the profile and cannot be relaxed using the same acceptance corpus. Passing a mean while violating a tail, coverage, or false-availability limit is a failure.

## Behavioral acceptance scenarios

The engine interface, artifacts, and result semantics are the test surface. Each processing level is qualified independently; optional GTSAM and raw-tight gates do not block embedded or offline processing.

### Embedded and artifact MVP

- The actual V2 Mini boots, navigates, shows priority live metrics, and retains its live summary with no phone, network, workstation, SD card, GTSAM, or host-native library present. With valid media, the same path records the session artifact; without it, artifact availability is explicitly false.
- The firmware dependency tree, feature tree, and final ELF/link map contain no `offline`, GTSAM, graph bridge, host storage, allocator, or unintended C++ runtime. Live, replay, offline, and available advanced processing all cross the same `aevia-trajectory` interface.
- Run `python3 traj/scripts/audit_s31_firmware_purity.py` as the repeatable generic-target dependency/link gate; the final ESP-IDF image and fitted-board qualification remain the release proof.
- Workspace construction rejects rates, fusion-delay/history, alignment/placement, or live metric plans beyond the profile before a partial session begins. The handle remains small, the stack is inside budget, and instrumented runtime RAM growth/allocation is zero.
- A 60-minute actual-board soak covers the unit's measured DEC4 rate across the specified 1.381–1.569 kHz range, with capacity tested at 1.75 kHz; every 12-frame DRY burst completes, counters unwrap without loss, navigation propagates at the validated 200–400 Hz cadence, declared separate position/velocity maxima are accepted, and intended display/radio load stays within all mean/p99/worst-observed timing, memory, queue, and frontier-delay gates.
- Sustained declared GNSS latency is absorbed by the chronological frontier. Observations beyond the lateness contract receive `TooLateForLive`; they never cause rollback, block capture, or accumulate background work.
- End-of-input fixtures place accepted navigation evidence throughout the final `fusion_delay`, call `finish()`, and prove that the terminal target advances from the normal delayed frontier through the last complete trusted IMU epoch `T_end` without extrapolation. Retrospective-metric fixtures backdate a sustained-motion launch, acceleration change point, stop dwell, and lap-validity decision to `t < W_nav`; each remains provisional until its evaluator consumes the declared support through `t + L_m` and `W_metric,m` passes `t`, then finalizes exactly once. `finish()` with an insufficient-lookahead tail leaves that result provisional.
- The minimum useful profile passes accuracy and timing together. Any profile revision caused by scalar/math backend, cadence, batching, fusion-delay/history, sensor schedule, dynamics model, or metric changes repeats both suites.
- Lap gates produce sub-GNSS-epoch crossings at a named physical point; offset-point and high-angular-rate cases exercise interval subdivision rather than polynomial assumptions. 0–100, 100–0, rollout, and quarter-mile results come from the correct continuous speed/distance functions; horizontal, vertical, course, and 3-D distance remain invariant under re-anchor thresholds and display/export resampling.
- GNSS outages, IMU saturation, PPS loss, numerical divergence, live-output overflow, and over-capacity work produce gaps, reinitialization, stale/degraded/withdrawn results, or explicit failure rather than silent confidence. The device emits no finalized `SkiReport` until a bounded live ski model passes a separate profile gate.
- Inside each `CapturedReplay` span, every exact live input resolves to complete configuration/metric/installation/calibration/math-profile/control provenance and its selected native/normalized evidence. A post-gap span has a complete replay seed or reinitialization record; otherwise it cannot claim captured equivalence. Unknown optional records are skipped; unknown required records fail with a capability diagnostic.
- Captured replay is invariant to host read chunking and scheduling, uses `V2MiniNumericV1`, and exactly reproduces dispositions, robust/observability decisions, root topology, reinitializations, anchor changes, stable event IDs, mutation sequence, and navigation/metric-finalization watermarks. Continuous state values satisfy the numeric tolerance table; threshold-adjacent ambiguity-band fixtures prove identical tie behavior.
- Device-versus-host forward comparisons use a versioned per-field table with absolute floors, relative limits, and hard maximum errors for position, velocity, attitude, covariance projections, distances, and event times. Missing/modeled covariance uses absolute limits; reported sigma may tighten but never loosen a hard cap.
- Permanent fixtures cover configuration/control changes, schema-minor migrations, reinitialization, anchor changes, recorder gaps/replay seeds, frame transformations, pre-GPS local time, GPS week/TOW rollover, MCU timer rollover, RATE2/ACC2 4-bit data-counter and 14-bit `FREQ_CNTR` wrap/reset, clock discontinuities, and every input disposition.
- Streaming decoder and semantic-validator fuzz/property suites cover arbitrary chunking, malformed/truncated lengths and counts, checked-integer overflow, non-finite values, invalid covariance/rotation, unknown records, and capacity bombs; they cannot panic, read out of bounds, or grow unbounded memory.
- SCH16T fault-injection fixtures corrupt MOSI/MISO CRC, response source address, CE/IDS/S[1:0], high-impedance/protocol state, and each frame position in the out-of-frame pipeline. The complete affected epoch is invalid, no payload reaches numeric observations, and the bounded flush/read-clear/resynchronize-or-reset path either proves DRY rearm and resumes or reinitializes without deadlock.
- SafeSPI pipeline fixtures prove that every request has exactly one matched response, including the terminal `FREQ_CNTR` primer carried into the next DRY burst, and inject faults into that inter-burst pending-response state as well as all 12 frames.
- SCH16T startup golden vectors cover the undefined first response, known primer, nonzero `SYS_TEST` challenge and restoration, 1 ms supply/`EXTRESN` plus 32 ms SCK-low timing, complete control writes, `EN_SENSOR`, the status-clear read, EOI, both post-EOI status reads, complete control/identity/`SN_ID1/2/3` readback, serial reconstruction, calibration binding, and every accepted literal `ASIC_ID`. A fault fixture at every request/response and state-machine stage covers wrong `COMP_ID`, unlisted ASIC revision, malformed/reserved serial bits, wrong-sensor calibration, CRC/address/status fault, register mismatch, early/late transition, soft/hardware-reset retry, clean pipeline restart, and entry into the non-navigating safe state after the fifth failed cycle.
- Accepted-firmware UM980 binary golden vectors cover every configured revision of `BESTNAVXYZ`, `OBSVM`, `STADOP`, `BESTSAT`, `RTKSTATUS`, `RTCMSTATUS`, and `HWSTATUS`. They verify frame/checksum/length and maximum signal counts; signedness, scaling, units, sentinels, and enum/status mappings for every consumed field; independently effective position and velocity epochs/latencies; covariance versus standard-deviation conversion; stale/invalid/correction/health dispositions; and the exact per-port byte budget. Valid-length field-offset mutations, unsupported message revisions, unlisted firmware/configuration, truncated maximum-size epochs, and inconsistent used-signal/solution diagnostics must fail or produce the explicitly declared unavailable/degraded capability rather than plausible numeric data.
- Navigation-math fixtures compare every analytic Jacobian with numerical differentiation, embedded `Φ/Qd` with a host matrix-exponential/Van-Loan oracle, and covariance/clock/noise models with the sample counts and NEES/NIS/coverage bounds in `QualificationSpecV1`. They cover slight PSD roundoff, bounded repair, excessive repair/divergence, invalid single axes, ACC3 substitution/range, the 10 ms gap boundary, and common-fault reinitialization.
- Observability fixtures include stationary gyrocompassing with vibration/g-sensitivity, straight constant-speed motion, turns/acceleration, deliberate vehicle sideslip, skiing/running, loose mounting, invalid non-holonomic assumptions, stationarity, and near-zero horizontal speed. Body heading/body-axis metrics are unavailable in every unobservable case; course is unavailable below its direction-SNR gate even when the velocity vector itself is valid.
- Metric fixtures include origin and offset reference points, high angular rate/acceleration, endpoint roots, two roots in one dense interval, even-multiplicity/tangent contact, unresolved interval cells, near-zero gate-normal speed, signed versus norm speed targets, reversing signed longitudinal displacement, ascending/descending speed roots, stationary distance, and output-rate/re-anchor invariance.

### Offline host MVP

- A normal host build without GTSAM or a C++ compiler uses log/acquisition adapters to validate artifacts, perform captured replay or recomputed normalization, and completes solution-level fixed-interval refinement through the same engine interface.
- A 60-minute session uses bounded memory plus the seekable state store. Insufficient/corrupt temporary storage fails explicitly and cleanup leaves the immutable source/result parents untouched.
- Recomputed normalization creates a new immutable revision and digest, never changes source bytes, and the `aevia-core` replay coordinator plus engine validation prevent mixing competing evidence lineages.
- On the versioned independent truth corpus, the offline smoother passes predeclared state/event RMSE, failure-rate, convergence, innovation-whiteness, NEES/NIS, and uncertainty-coverage thresholds. A controlled middle-session GNSS outage demonstrates the expected benefit from future observations versus forward-only replay.
- A non-convergent or integrity-worsening host run remains a diagnosed candidate revision and never auto-promotes over the live/offline parent.
- Host activity/ski processing may finalize full-session reports and always identifies the physical reference point; it does not depend on a live ski model.
- A complete refined sidecar publishes/reimports only when all source, selected-normalization, external-input, and embedded/content-addressed definition digests match. Altered, interrupted, truncated, or mismatched results fail transactionally without replacing earlier results.

### Optional GTSAM gates

- With GTSAM unavailable or failed, `Require(AdvancedGraph)` fails. `BestQualified` restarts offline processing from canonical evidence and records the attempt/fallback; no partial graph state leaks through.
- Enabling GTSAM changes no public observation, trajectory, metric, artifact, or result interface. The adapter passes its versioned qualification thresholds before the engine may prefer it, and its result carries actual-backend provenance.

### Conditional raw-tight gates

- Only when complete raw fixtures exist, raw-tight processing identifies missing base/correction/ephemeris inputs precisely, handles cycle slips/false fixes without silent confidence, and refuses to double-count receiver PVT with its originating raw observations.
- It must pass its own versioned accuracy/integrity corpus before being described as more accurate than solution-level refinement.

## Defaults and exclusions

- Default live processing: the bounded V2 Mini profile, fusing UM980 receiver position/full vector velocity with engine-batched calibrated SCH16T evidence in the embedded ESKF.
- Default acquisition: preserve high-rate native evidence while the live filter runs at its validated lower navigation cadence.
- Default host final processing: offline Rust solution-level fixed-interval smoothing; GTSAM is not required.
- Optional workstation processing: pinned GTSAM graph refinement inside the same engine when explicitly built, requested, and qualified.
- Experimental raw-observation processing: host-only tight RTK/INS when complete rover/base observations, corrections, and ephemerides exist; accuracy claims wait for its conditional gate.
- Default body convention: forward-left-up.
- Default global representation: ECEF in the session's resolved terrestrial-frame realization and coordinate epoch. If only a generic WGS-84 ensemble label is available, surveyed cross-frame centimetre metrics remain unavailable until normalization supplies an accepted coordinate operation.
- Default activity/ski physical point: instrument package, not inferred human CoM.
- Default metric policy: no implicit speed, distance, launch, rollout, gate, or elevation semantics.
- Display paths never feed metric calculations.
- No hardware UART/SPI drivers or NTRIP/RTCM transport inside the trajectory packages. Versioned decoding/normalization of captured native UM980/SCH16T payloads is required in `aevia-trajectory-acquisition`; receiver control and physical transport stay outside it.
- No receiver configuration or correction transport.
- The versioned session/result formats, codec, validation, and replay contract are in scope; SD/MMC, FAT/filesystem, USB, BLE, and cloud storage drivers are not.
- No GTSAM, graph optimization, raw ambiguity solving, or full-session state retention on the ESP32-S31.
- No Flutter/Dart API.
- No map matching.
- No full-size V2 work.
- No claim of perfect, official drag-beam-equivalent, or safety-certified output.
