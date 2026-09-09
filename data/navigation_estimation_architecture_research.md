# Aevia navigation and motion-estimation architecture

- **Date checked:** 2026-08-31
- **Hardware in scope:** Unicore UM980 GNSS + Murata SCH16T-K01 IMU
- **Product modes:** drag performance timing, skiing distance/statistics, and later circuit lap timing
- **Evidence convention:** “Documented” means a vendor document, official source tree, or original paper says it. “Recommendation” means an engineering conclusion for Aevia; it is not a disclosed Google, Dragy, Unicore, or Murata algorithm.

## Decision

There is no single library or secret algorithm that every product uses. The closest thing to the common professional pattern is:

1. a hardware-synchronised, lossless measurement log;
2. strapdown inertial propagation with an **error-state extended Kalman filter (ESKF)** for live position, velocity, attitude and sensor-bias estimation;
3. GNSS position **and vector velocity** updates, with integrity checks and weights derived from solution quality;
4. raw-observation RTK/PPK as the GNSS front end;
5. forward/backward or factor-graph smoothing after the session; and
6. separate, precisely defined measurement functions for speed crossings, distance crossings, ski distance and lap-line crossings.

For reasonable development time, Aevia should start **loosely coupled**: let the UM980 solve live RTK position and Doppler velocity, fuse those solution-level observations with the SCH16T in an ESKF, and preserve all raw GNSS observations for later PPK. The best practical offline result is then a PPK-corrected GNSS trajectory fused with the IMU in a full-session smoother. A raw-observation **tightly coupled** GNSS/INS estimator is a later upgrade for difficult reception, not the right first implementation.

This gives one shared trajectory product rather than separate “drag”, “ski” and “lap” filters:

```text
PPS-aligned raw GNSS + receiver solution + synchronised IMU
                              |
              +---------------+----------------+
              |                                |
        live UM980 RTK                  raw GNSS observations
              |                                |
     real-time ESKF                     RTK/PPK processor
              |                                |
              +---------- offline smoother ----+
                              |
            time-continuous pose, vector velocity,
              covariance, quality and motion state
                              |
       +----------------------+----------------------+
       |                      |                      |
  drag events            ski distance          lap crossings
```

Professional post-processing products publicly describe the same broad structure. NovAtel Waypoint supports loose and tight GNSS/INS coupling, calls tight coupling the preferred mode, processes data in both time directions, applies IMU-to-antenna lever arms and body-to-IMU rotations, and uses residual tests to reject inconsistent GNSS updates. It does **not** publish its complete estimator implementation. See the official [Inertial Explorer overview](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Overview_of_IE.htm), [processing workflow](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Process_IMU_Data.htm), [loose/tight coupling explanation](https://novatel.com/support/waypoint-support/waypoint-software-getting-started/videos-getting-started/processing/loosely-coupled-and-tightly-coupled), and [GNSS residual-testing documentation](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/GNSS.htm).

## Why this is the best fit for the hardware

The UM980 is not merely a point generator. Unicore specifies up to 50 Hz RTK output in a specific mode, 20 ns RMS time accuracy and 0.03 m/s RMS velocity accuracy. The important qualification is that the velocity figure is specified for an unobstructed **static** test at 99%, so it is an input to Aevia’s design, not a demonstrated dynamic product accuracy. See the current [UM980 product specification](https://en.unicore.com/products/um980/) and the bundled [UM980 R1.9 manual](./datasheets/UM980_User%20Manual_EN_R1.9.pdf).

The official Unicore command reference documents:

- `OBSVM`, which contains raw observations for currently tracked satellites;
- `BESTNAVXYZ`, which reports ECEF position and velocity, their per-axis standard deviations and solution status;
- a velocity-latency field that must be subtracted from the epoch time to recover the velocity’s correct time; and
- configurable PPS output and delay compensation.

Those details are in the [N4 Products Commands and Logs Reference Book](https://en.unicore.com/uploads/file/unicore-reference-commands-manual-for-n4-high-precision-products-v2-en-r1.2.pdf). They are why Aevia should ingest the binary receiver messages and their measurement epochs, not NMEA values timestamped when UART bytes happen to arrive.

The SCH16T is well suited to interpolation and short GNSS gaps. Murata documents data-ready, timestamp-index and SYNC functions for clock-domain synchronisation, selectable 13–370 Hz filtering, 0.3 deg/h typical gyro bias instability, 0.0004/0.0006 deg/s/sqrt(Hz) gyro noise density, and 80 micro-g/sqrt(Hz) accelerometer noise density. See Murata’s official [SCH16T product page](https://www.murata.com/en-us/products/sensor/gyro/overview/lineup/sch16t) and [SCH16T-K01 datasheet](https://www.murata.com/-/media/webrenewal/products/sensor/pdf/datasheet/datasheet-sch16t-k01-short.ashx).

## Acquisition and time model

This is the part to get right before sophisticated algorithms. Errors here cannot be repaired reliably in the app.

### Required firmware log

Record, without lossy conversion:

- UM980 binary raw observations, navigation/ephemeris messages, correction age, RTK fixed/float/single state, satellite/signal use, C/N0, cycle-slip/lock indicators and receiver-reported covariance;
- the UM980 ECEF position and **three-component velocity**, including its velocity latency;
- every SCH16T accelerometer, gyro, temperature, status and timestamp-index sample;
- a hardware capture of UM980 PPS in the MCU time domain, and SCH16T SYNC/data-ready timing;
- monotonically increasing sequence numbers and dropped-record counters;
- receiver, firmware, antenna, filter, sample-rate and correction configuration; and
- the measured antenna-to-IMU lever arm and body-to-IMU rotation for rigid mounts.

Use one monotonic integer time base internally. Maintain an explicitly estimated mapping from MCU ticks to GNSS time from PPS captures. Do not use phone receipt time, BLE packet time, filesystem time, or civil wall-clock time for performance events.

### Calibration

Factory sensor numbers are insufficient for the last milliseconds or centimetres. Store and version:

- IMU scale, bias, non-orthogonality and temperature calibration;
- GNSS antenna reference point/phase-centre information where available;
- IMU-to-antenna lever arm and boresight;
- fixed transport latency and its uncertainty; and
- the vehicle reference point used for drag/lap results.

For a rigid vehicle, correct antenna velocity to the chosen body reference with the standard rigid-body term `v_ref = v_ant - omega x r`. For skiing, a rigid helmet/backpack antenna follows a known body point. A receiver and antenna moving freely in a pocket measure the path of that moving antenna; no single-sensor algorithm can uniquely recover an unobserved centre-of-mass path without a motion model. Skiing research therefore places high-precision antennas on the head/helmet and explicitly models the centre of mass: see the original [alpine skier centre-of-mass study](https://pmc.ncbi.nlm.nih.gov/articles/PMC3812581/) and [GNSS/IMU centre-of-mass trajectory study](https://pmc.ncbi.nlm.nih.gov/articles/PMC6125645/).

## Real-time estimator

### Recommended v1: solution-level ESKF

Use a local ENU navigation frame and estimate at least:

```text
position p, velocity v, attitude q,
gyro bias bg, accelerometer bias ba,
and their covariance
```

Propagate at the IMU rate with strapdown inertial equations. Update with:

- UM980 vector velocity (not position-differenced velocity);
- UM980 RTK/standalone position;
- zero-velocity constraints when stationary with high probability; and
- optional known-height or surveyed-line constraints only when the product mode makes them valid.

Use innovation/residual gating, robust losses, and state-dependent observation covariance. RTK fixed, RTK float, standalone and invalid observations must not share one noise value. Estimate or adapt short-term noise from filter innovations, but retain physically meaningful lower/upper bounds learned in validation.

The ESKF formulation is a standard way to keep attitude and small error states numerically well behaved; Joan Solà’s original technical report provides the quaternion/error-state derivation: [Quaternion kinematics for the error-state Kalman filter](https://www.iri.upc.edu/publications/show/1773). PX4’s production EKF is also a useful BSD-licensed implementation reference for delayed sensor fusion, innovations and covariance, but it is tuned for aircraft/vehicles and should not be copied as Aevia’s ski/drag algorithm; the old standalone ECL repository is explicitly unmaintained. See [PX4 ECL source](https://github.com/PX4/PX4-ECL) and the current [PX4 EKF documentation](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html).

### Ultimate upgrade: raw-observation tight coupling

Tight coupling applies satellite pseudorange, Doppler and carrier-phase-derived constraints directly alongside IMU factors, so useful satellites can still constrain the INS when the receiver cannot form a full position solution. It is the higher-ceiling architecture in tree cover, mountain blockage and partial outages. It also requires satellite/clock/atmosphere models, ambiguity handling, cycle-slip detection, antenna corrections and much more validation.

Therefore, do not make custom tight coupling a launch dependency. Preserve the observations now so it remains possible later.

### Velocity inputs

Use three complementary sources:

1. **Receiver Doppler vector velocity** for robust absolute velocity.
2. **Time-differenced carrier phase (TDCP)** as a later high-precision relative-velocity/increment factor when lock is continuous.
3. **IMU specific force and rotation** to interpolate between GNSS epochs and bridge short gaps.

The original literature describes Doppler as the usual GNSS velocity method and shows that TDCP can reach a lower noise level but is vulnerable to cycle slips. The 2024 multi-device study reports centimetres/second as the benign-environment scale for Doppler and millimetres/second for TDCP, while stressing diagnostic/fault exclusion; these are method-level experimental results, not guaranteed Aevia performance. See [Velocity Estimation Using TDCP and Doppler Shift](https://www.mdpi.com/1999-4893/17/1/2) and the original [smartphone TDCP evaluation](https://pmc.ncbi.nlm.nih.gov/articles/PMC9655395/).

Do not differentiate successive positions for primary speed, and do not treat raw scalar speed as perfect. Estimate the velocity **vector and covariance** within the navigation state.

## Offline final estimator

After a session:

1. run RTK/PPK over the raw rover observations plus base/network observations;
2. retain ambiguity state, cycle-slip/outlier diagnostics and epoch covariances;
3. fuse corrected GNSS position, Doppler/TDCP velocity and IMU in a full-session factor graph or forward/backward smoother;
4. solve lever arm, biases and any explicitly permitted calibration states;
5. produce a time-continuous state trajectory plus covariance; and
6. calculate all product results from that trajectory, never by mutating raw data.

Forward and backward processing allows observations after a short outage to improve the trajectory inside or before it; NovAtel documents this as the default for differential post-processing. [Inertial Explorer processing workflow](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Process_IMU_Data.htm).

## Distance without the shoreline problem

Never sum unfiltered point-to-point GNSS distances. Never integrate the magnitude of unfiltered noisy speed during stops. Both operations turn zero-mean vector noise into positive distance.

First estimate one continuous, physically plausible trajectory. Then calculate a declared functional of its smoothed velocity:

```text
horizontal path distance: D_h  = integral sqrt(v_E^2 + v_N^2) dt
3D antenna path distance: D_3 = integral sqrt(v_E^2 + v_N^2 + v_U^2) dt
```

Stationary intervals receive a probabilistic zero-velocity constraint and contribute zero only when supported by evidence. Numerical integration uses the continuous state/interpolation, so changing only the output sample rate must not change the answer materially.

This removes the sampling-density pathology, but it does not eliminate the need to define the object being measured. “Antenna path”, “vehicle reference-point path”, and “estimated skier centre path” are different legitimate quantities. An adaptive estimator can learn current observation noise, mount stability and motion state; it cannot infer an unobservable definition of “true distance” with no prior or reference point.

For skiing, expose at least:

- horizontal skier-route distance;
- 3D slope-path distance;
- vertical descent; and
- lift, stopped and downhill segments separately.

High-precision ski research demonstrates the feasibility of centimetre-scale antenna trajectories using a head-mounted external antenna, base station and PPK, but it also applies explicit smoothing and distinguishes head motion from course shape. See the original [cross-country skiing kinematic GNSS study](https://pmc.ncbi.nlm.nih.gov/articles/PMC6891545/) and [alpine skiing methodology paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC7739811/). These studies do not establish the accuracy of a loose-pocket Aevia configuration.

## Activity-specific measurements

### Drag racing

- Define speed as horizontal ground-speed magnitude `sqrt(v_E^2 + v_N^2)` unless a regulation explicitly requires another definition.
- Solve speed events such as 100 km/h as roots of the continuous estimated speed curve, not “the first sample above 100”.
- Define launch and rollout explicitly. Keep “no rollout” and “1 ft rollout” as different result types.
- For consumer 0–400 m tests, integrate estimated horizontal path speed from the declared start event.
- For a surveyed drag strip, the most traceable elapsed time is crossing of defined start/finish planes by a corrected vehicle reference point. Keep this distinct from odometer-like path distance.
- Compute grade/elevation from the final trajectory and report it; do not silently modify elapsed time.

Vehicle-test supplier VBOX publicly states that it derives speed from satellite Doppler and distance by integrating Doppler-derived speed. It advertises 100 Hz systems, RTK, IMU integration and formal calibration/verification workflows. Its published specifications and test claims are useful benchmarks, not proof Aevia will match them. See VBOX’s official [GPS accuracy explanation](https://vboxautomotive.co.uk/en/how-does-it-work-gps-accuracy), [speed/distance verification page](https://www.vboxautomotive.co.uk/en/speed-distance-verification), and [speed-sensor specifications](https://www.vboxautomotive.co.uk/en/products/sensors/vbss).

Dragy publicly says its current unit uses a tenth-generation u-blox GNSS receiver, receives four constellations and measures to 1/100 s. It does **not** publicly disclose its Doppler use, filters, launch detector, distance integration, latency correction or uncertainty model. Do not reverse-engineer a design assumption from its marketing page: [official Dragy product page](https://dragymotorsports.com/products/dragy-gps-performance-box).

### Future lap timing

Represent start/finish and sector lines as oriented finite gates. Correct antenna position to the chosen vehicle reference point, detect a signed gate crossing with direction/hysteresis, and root-solve crossing time on the continuous trajectory. This uses the same estimator and timing model as drag racing; only the event functional changes.

### Skiing

Use the smoothed 3D trajectory for distance, not map snapping. Segment skiing/lift/stationary motion with a probabilistic state model informed by vertical velocity, ground speed, route direction and IMU features. A loose mounting mode should down-weight inertial navigation updates when GNSS/IMU innovations indicate changing lever arms. The best route-distance hardware arrangement remains a rigid antenna and, if strong inertial fusion is required, a rigidly related IMU.

## What Google Maps publicly does—and does not say

Android’s `Location.getSpeed()` returns speed at the fix and explicitly says it may be more accurate than sequential `distance/time` because GNSS Doppler may be used. Android also exposes a 68th-percentile speed-accuracy value and a monotonic elapsed-realtime timestamp. [Android `Location` reference](https://developer.android.com/reference/android/location/Location#getSpeed()).

Google documents that the Fused Location Provider combines GPS, Wi-Fi, cell, accelerometer, gyro, magnetometer and other signals while trading accuracy, frequency and latency against battery use. It does not publish its filter equations, thresholds or precise speed derivation. [Fused Location Provider overview](https://developers.google.com/location-context/fused-location-provider) and [Android battery/location trade-offs](https://developer.android.com/develop/sensors-and-location/location/battery).

Google Maps Navigation additionally provides road-snapped locations that may differ from Fused Location Provider output. Its traveled-route API uses road-snapped locations simplified into line segments and may interpolate more under poor GPS. The Roads API similarly map-matches points to likely road geometry. These are navigation/map-matching functions, not documented metrology-grade speed or distance algorithms. See the official [Navigation SDK events guide](https://developers.google.com/maps/documentation/navigation/android-sdk/events), [`getTraveledRoute()` reference](https://developers.google.com/maps/documentation/navigation/android-sdk/reference/com/google/android/libraries/navigation/Navigator#getTraveledRoute()), and [Roads API overview](https://developers.google.com/maps/documentation/roads/overview).

**Unknown:** Google does not publicly disclose how the consumer Google Maps speedometer computes or smooths displayed speed. It is therefore incorrect to claim that Maps simply differences positions, directly displays raw Doppler, or uses a particular Kalman filter. Google Maps optimises navigation, road adherence, power and UX; it is not the model to copy for drag instrumentation.

## Reusable implementations

| Component | Appropriate use | Caveats |
|---|---|---|
| [RTKLIB](https://github.com/tomojitakasu/RTKLIB/blob/master/readme.txt) | GNSS-only RTK/PPK, RINEX/RTCM/NTRIP, ambiguity resolution, conversion and diagnostics | ANSI C; no complete Aevia GNSS/INS estimator; UM980 binary ingestion may require conversion/decoder work; upstream releases are old and the official support page lists known bugs. Pin, audit and regression-test a chosen fork/version. Its official readme permits commercial use under BSD-2-Clause plus two stated extra clauses; retain notices and obtain legal review before shipping. |
| [GTSAM](https://gtsam.org/) | Offline/fixed-lag factor graph, IMU preintegration, GNSS position factors and lever-arm-aware factors | BSD-3-Clause C++; it is a solver/toolbox, not a GNSS ambiguity engine. Aevia must implement Doppler/TDCP factors, robust gating, clock/time handling and Rust/mobile FFI. Official [IMU factor](https://borglab.github.io/gtsam/imufactor/) and [GPS factor](https://borglab.github.io/gtsam/gpsfactor/) documentation. |
| [PX4 EKF2](https://github.com/PX4/PX4-Autopilot) | Reference for a field-proven real-time EKF, delayed measurements, innovations and fault handling | BSD-3-Clause, but embedded in an autopilot and based on aircraft/vehicle motion assumptions; adapt concepts, not tuning or product definitions. |
| [NovAtel Waypoint/Inertial Explorer](https://novatel.com/products/waypoint-post-processing-software/inertial-explorer) | Commercial benchmark, reference-trajectory generation, or optional server/workstation processing | Proprietary licensing and integration cost; not an on-device open library. Its public SDK may automate licensed processing, but availability/terms must be confirmed with NovAtel. |

For Aevia’s Rust core, the most controlled first step is a small in-house ESKF using a mature linear-algebra crate, with RTKLIB isolated behind a GNSS-processing interface. Add GTSAM offline only when recorded fixtures prove that the simpler forward/backward smoother is insufficient. Avoid making Flutter aware of any estimator internals.

## Uncertainty and validation

Filter covariance is conditional on the model and is not by itself a trustworthy product confidence interval. The final system should combine:

- propagated random uncertainty from the smoothed state;
- solution/ambiguity/cycle-slip and outage diagnostics;
- sensitivity to alternate plausible noise/model settings; and
- empirically observed systematic error from reference tests.

Estimate nonlinear result uncertainty (speed-crossing time, distance, gate crossing) by sampling correlated trajectories from the smoothed posterior or an equivalent sigma-point/linearised propagation. Then calibrate reported coverage empirically.

Required validation:

- stationary tests where distance remains statistically consistent with zero;
- analytic simulated straight, circular and 3D paths;
- identical-recording calculations at multiple output sample rates (sampling-rate invariance is the direct shoreline test);
- surveyed drag distances and independent optical timing gates;
- rigid side-by-side comparison with a calibrated VBOX or survey GNSS/INS reference;
- ski tests with a rigid reference antenna while Aevia is used in each supported mount mode;
- open sky, trees, buildings, mountain masking, RTK fixed/float/loss and intentional cycle slips;
- temperature and vibration sweeps; and
- 95% intervals checked for approximately 95% empirical coverage within each declared operating condition.

Until those tests exist, receiver specifications support a design target but not a product accuracy claim. Display resolution (for example, `0.001 s`) must remain separate from validated accuracy.

## Implementation order

1. Freeze the binary recording schema and hardware time model.
2. Build replay, synthetic trajectories and fault injection before hardware firmware is complete.
3. Implement the receiver-message decoder and a solution-level ESKF.
4. Implement drag event definitions and sampling-invariance tests.
5. Add PPK import/replacement trajectories and offline smoothing.
6. Implement ski segmentation and explicitly named horizontal/3D distances.
7. Validate and calibrate uncertainty against independent references.
8. Add lap/sector gate crossings on the same trajectory API.
9. Consider tight coupling/TDCP only after field logs show which outages and errors dominate.

The important design seam is:

```text
estimate_trajectory(recording, processing_profile) -> TrajectoryEstimate
measure_drag(trajectory, definitions)               -> DragResults
measure_ski(trajectory, definitions)                -> SkiResults
measure_laps(trajectory, gates)                     -> LapResults
```

That is the most common professional idea adapted to Aevia: solve navigation once, retain every raw observation, and keep the meaning of each product measurement explicit and testable.
