# Sensor timing

The implementation uses the fitted board's GNSS PPS on GPIO42 and SCH16T DRDY
on GPIO14. Both rising edges latch one free-running MCPWM0 capture timer. The
timer is clocked from the 40MHz crystal divided by four: 100ns ticks, independent
of CPU frequency. This is timestamp resolution, not a claim of 100ns system
accuracy. Firmware task wake-up and SPI/UART transfer times do not set the
measurement epoch.

`esp-hal` 1.2.2 does not expose S31 MCPWM capture. The small adapter in
`src/device/platform/capture.rs` reserves MCPWM0 and GPIO matrix input slots 95/96
directly through the matching PAC. It follows the S31-specific clock routing
and capture registers, rather than assuming the older S3 APB capture clock.
No PWM output is enabled. Channel 2 is reserved for software reads of the same
counter. A startup cross-stamp gives the timeline a fixed local boot origin;
it does not discipline or step that timeline.

The counter is extended to 64 bits on every read, including a 100ms maintenance
poll when sensors are absent. A stopped/wrong-rate clock or a gap exceeding half
the 32-bit wrap period invalidates capture until reboot. PPS polling transfers
the hardware latch into an eight-entry history; it does not timestamp the edge.

## GNSS

The receiver is explicitly configured with:

```text
CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0
PPSSTATUSB COM2 1
BESTNAVXYZB COM2 0.02
```

ENABLE allows pulses after receiver timing convergence; ENABLE3 is intentionally
not used. Every decoded frame passes CRC/length checks. Integer PPSSTATUS payload
week/millisecond fields retain the documented pulse-label interpretation, including
the manual's example where the header is one second later. The fitted UM980
Build17548 also produces fractional payload tags, with observed 20ms and 160ms
lags on different boots. These are not interpreted as electrical pulse epochs or
corrected with a guessed offset. The earlier assumption of one preceding 50Hz
observation was disproved by the second capture.

For a fractional tag, firmware instead correlates the already-integer FINE/GPS
header with a captured rising edge. This uses two documented contracts: N4 header
week/milliseconds specify the GPS epoch, and output delay relates the observation
to transmission; the verified PPS configuration selects an integer-second edge.
The payload must still lie within the immediately preceding second as a sanity
check. At startup the unique matching capture must also be the latest edge and
at most 100ms before receipt. Neither timestamp is rounded. The precise meaning
of the fractional payload remains unresolved, so this independent correlation
policy is an inference supported by those contracts and retained board evidence,
not vendor confirmation of the payload semantics or a timing qualification.
The signed `offsetTime` remains raw diagnostics and is never added to the epoch;
the observed offsets of roughly -0.373ms and -0.115ms do not explain the tag lags.
Readback must show
the expected GPS, rising-edge, one-second configuration and quality 2 or 3.
Unknown/non-FINE/non-GPS times are not fused.
Before fusion is enabled, a checksum-verified CONFIG reply must also confirm
the complete command, including zero RF and user delay. CONFIG is sent through
the proven COM1 transmit path; replies are accepted on either receive path.
CONFIG readback has succeeded on the connected board. If readback is unavailable,
raw acquisition continues but PPS association and GNSS fusion stay disabled.
The normal mixed UART decoders keep running during a two-second CONFIG response
window. Missing or failed readback retries after three seconds, without
reconfiguring a receiver whose navigation epochs are advancing. A late,
malformed or mismatched reply cannot open the gate. No zero delay or timing
lock is assumed.

During acquisition only, header epoch minus pulse epoch plus reported output
delay selects exactly one captured pulse within a 100ms association window.
For independent integer-header correlation the epoch difference is zero.
This window tolerates serial delivery and scheduling delay without using them
as the timestamp. Three consecutive validated labels are required to lock.
Thereafter an affine fit across up to eight pulse pairs performs association
independently of arrival time. Frequency must be within 500ppm of nominal and
fit residuals within 50us. Absolute GPS nanoseconds are never converted to a
float: integer origins are subtracted before fixed-ratio conversion.

Position uses BESTNAVXYZ's GPS header measurement epoch; velocity uses that
epoch minus the separately reported velocity latency. Each is mapped to the
capture timeline. Receiver output delay is not subtracted again. PPS phase-error
sign conventions are not guessed; errors above 10us reject the label and the
remaining reported error is covered by the conservative timing margin.

Missing pulses for more than 1.5 seconds, stale labelled timing beyond three
seconds, an invalid quality/configuration or a discontinuity clears lock.
Reacquisition needs three fresh consecutive labels. Unsynchronized GNSS fixes
are counted but not assigned UART-arrival fallback timestamps. Previously
processed observations are never retimed when a new fit arrives. Receiver
reconfiguration starts a new mapping generation on the unchanged local timeline.

The current GNSS uncertainty model retains 0.5ms for header epoch granularity,
plus observed fit residual and extrapolation margin. This is conservative
development metadata, not calibrated uncertainty coverage. PPS accuracy, RF
antenna/cable delay and the receiver's observation-time semantics still require
board measurements before making metrology claims.

## IMU

The HAL's typed SCH16T configuration selects LPF3/DEC5 with Dyn1 ranges on all
axes, active-high DRDY and the default enabled self-tests. The HAL owns startup
verification, SafeSPI framing, diagnostic metadata and physical scaling.
Application code only handles sampling and estimator support. Every six-axis burst consumes
the hardware DRDY latch and validates CRC, status and all six data counters.
DEC5 averages sixteen internal samples and runs at F_PRIM/32: 737.5Hz nominal,
690.625..784.375Hz across the documented sensor clock range. This halves the
sample workload compared with DEC4 so the host has more time to read each sample.
Navigation requires the complete hardware read within 600us of its edge. Native
collection uses a 1000us bound: the maximum specified F_PRIM of 25.1kHz gives a
minimum DEC5 interval of 32/25100 seconds (1274.900us), leaving more than 274us
before that interval. These are admission policies, not calibrated timestamp
accuracy. The completed-read timestamp precedes software scaling. An overwritten edge, missed counter,
nonconsecutive interval or late read is rejected; a missing sample is never
replaced with a stretched average. Consecutive edge spacing supplies the
averaging interval duration, preserving actual sensor clock variation. The
accepted spacing is 1.2..1.6ms; the data counter must advance by exactly one on
every axis. LPF3 delays and conservative noise/timing priors remain unchanged.
The absolute bound remains mandatory even when the latest DRDY latch is
unchanged: Murata Rev.6 section 5.4.4 explains that a new DRDY pulse requires all
sensor data to have been read. A stalled burst can therefore retain an old edge
while its data counters advance. Both profiles reject such stale reads and
reseed their interval history. CRC and per-axis sensor status validation remain
in the HAL before admission.

`ImuDiagnostics` late-start and late-finish counters use the active profile's
same bound. Native recording kind2 bit1 reports that bound plus latest-edge
equality, while kind9 offset 88 stores its exact value in nanoseconds. All raw
edge/start/finish/latest timestamps and latency maxima remain recorded, allowing
offline evaluation against a stricter bound. This flag alone does not establish
counter continuity, valid motion, calibrated sensor delay or timing qualification.

Navigation represents the longer interval by its full end-stamped support, without
an extra half-period timestamp shift or an assumed independent-noise reduction.
The stationary classifier's 100-sample minimum now spans about 137ms on this
board, rather than about 69ms; navigation output remains configured at 200Hz.

Murata Rev.6 Table 9 gives nominal LPF3 group delays at 10Hz of 1.4ms for gyro
and 1.5ms for acceleration. Gyro interval averages are linearly interpolated
between consecutive samples onto acceleration's later 1.5ms delay. Both vectors
then have the same support. The corrected support ends at DRDY minus 1.5ms.
`traj::SampleSupport::IntervalAverage` describes the whole end-stamped interval;
subtracting another half interval would double-count the averaging delay.

IMU runs alone on the priority-5 acquisition executor. GNSS decoding and PPS
maintenance run on priority 3; GNSS decodes at most 128 bytes before a 100us
suspension. GPIO and UART wakeups run at priority 6. This keeps packet decoding
out of the IMU deadline path. The power bus runs on the main executor, below the SDK priority-1 timer.
Blocking I2C must not delay the timer that resumes timed acquisition waits. Full HAL initialization includes synchronous reset and filter
settling delays; it runs on the main executor. Two capacity-one ownership
queues move the driver between initialization and acquisition, so a retry
never blocks the acquisition interrupt or accesses SPI concurrently. Four
consecutive driver/DRDY-timeout failures return ownership for reset/reinitialization.
Timing gaps and range rejection reseed alignment without resetting healthy hardware.
A completed poll waits directly on GPIO when DRDY is low. An already-high
DRDY forces a 100us timed suspension to bound overloaded ready loops.
The worker sleeps 500us when idle to bound shared scheduler activity.
Failed startup attempts wait one second before retrying. Sensor measurement
queues remain nonblocking and independent of these ownership queues.

These group delays are frequency dependent and vary between units. Linear
fractional-delay alignment is a development approximation. A nonzero 1.2ms
timing allowance remains, and `traj`'s development timing approximation is kept
explicitly enabled. It retains timing uncertainty as metadata but does not
propagate that uncertainty or resample the interval. Factory scaling, mounting and group delay are unqualified;
the firmware never reports `qualified=true`. Hardware capture removes host
scheduling jitter; it does not make the IMU's analogue/digital filter delay exact.

## Validation on the board

Inspect `Snapshot.timing`: pulse edges/pairs, lock, age, residual, generation,
capture errors, rejected IMU timing and unsynchronized GNSS counts. An optional screen
shows PPS WAIT/LOCK; the current board has no screen. Verify the configured pulse polarity/rate and DRDY against
the capture timer using a scope or logic analyser. Check that delayed UART
delivery leaves mapped measurement epochs unchanged, loss/reacquisition clears
lock correctly, and sustained display/USB work does not cause IMU late reads.
Measure filter alignment/delay before reducing the remaining uncertainty.

The focused host regressions cover wrap extension, wrong-rate/long-gap rejection,
GPS week rollover, drift, delayed pulse labels, UART-latency invariance, loss
of lock, IMU fractional-delay alignment, counter wrap/gaps, late/overwritten
captures and invalid motion. Host tests do not establish board timing. Measure
sustained capture-to-read latency and queue drops with the display, USB and
power monitoring active before accepting the firmware for field evaluation.

## Reviewed primary sources

- [Espressif S31 MCPWM low-level driver](https://github.com/espressif/esp-idf/blob/master/components/esp_hal_mcpwm/esp32s31/include/hal/mcpwm_ll.h): clock selection/division, capture latch and channel configuration.
- [S31 GPIO signal map](https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s31/include/soc/gpio_sig_map.h): PWM0_CAP0/1 inputs 95/96.
- [S31 clock definitions](https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s31/include/soc/clk_tree_defs.h): crystal source and capture clock options.
- [Unicore N4 command reference R1.14](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf): CONFIG PPS, N4 header and BESTNAVXYZ velocity latency.
- [Unicore N4 command reference R1.2](https://en.unicore.com/uploads/file/unicore-reference-commands-manual-for-n4-high-precision-products-v2-en-r1.2.pdf), Tables 7-49/50 and section 7.3.56: GPS header/output-delay contract, selected integer-second edge, and raw PPSSTATUS fields. The fractional-tag correlation policy above is an explicitly identified inference.
- [Local Murata SCH16T-K01 Rev.6 datasheet](../../data/datasheets/sch16t-k01-datasheet-full.pdf): Table 9, sections 5.4.2–5.4.7 and Table 66.


## Hot-path clock access

SPI2 uses CPU0 mcycle intervals at its fixed configured CPU frequency. Startup
enables/checks counting and verifies advancement using a vendor-timed interval;
unsupported/stopped counters retain the vendor-timer fallback. Setup/hold delays
round cycles upward; timeout arithmetic uses wrapping differences and rejects
ambiguous half-wrap intervals. Long initialization waits retain the vendor timer.
This clock is a delay/deadline implementation detail, never a measurement epoch.

Capture reads extend MCPWM ticks without invoking SYSTIMER per observation.
PPS maintenance checks accumulated ticks against a separate wall-time/tick
reference every 100ms. Fast reads cannot reset or hide that validation baseline.
Clock failure still invalidates capture until reboot. Maintenance supervision
and the watchdog bound task outages well below the 214.7s half-wrap; checked
validation independently rejects a long wall-clock outage.
