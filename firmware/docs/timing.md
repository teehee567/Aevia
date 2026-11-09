# Sensor timing

The implementation uses the fitted board's GNSS PPS on GPIO42 and SCH16T DRDY
on GPIO14. Both rising edges latch one free-running MCPWM0 capture timer. The
timer is clocked from the 40MHz crystal divided by four: 100ns ticks, independent
of CPU frequency. This is timestamp resolution, not a claim of 100ns system
accuracy. Firmware task wake-up and SPI/UART transfer times do not set the
measurement epoch.

`esp-hal` 1.2.2 does not expose S31 MCPWM capture. The small adapter in
`src/device/timing/capture.rs` reserves MCPWM0 and GPIO matrix input slots 95/96
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
not used. Every decoded frame passes CRC/length checks. The PPSSTATUS payload's
week/millisecond fields label the pulse. Its header can describe a later second,
so the header epoch is never substituted for the pulse epoch. Readback must show
the expected GPS, rising-edge, one-second configuration and quality 2 or 3.
Unknown/non-FINE/non-GPS times are not fused.
Before starting the logs, a checksum-verified CONFIG reply must also confirm
the complete command, including zero RF and user delay. CONFIG is queried on
both wired UARTs to tolerate a missing COM1 receive path; failure to verify
causes a configuration retry rather than an assumed zero delay.

During acquisition only, header epoch minus pulse epoch plus reported output
delay selects exactly one captured pulse within a 100ms association window.
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

The existing LPF3/DEC4 configuration is retained. Every six-axis burst consumes
the hardware DRDY latch and validates CRC, status and all six data counters.
The burst must finish within 600us of its edge, safely before the earliest
documented next DEC4 epoch (~637us). An overwritten edge, missed counter,
nonconsecutive interval or late read is rejected; a missing sample is never
replaced with a stretched average. Consecutive edge spacing supplies the
averaging interval duration, preserving actual sensor clock variation.

Murata Rev.6 Table 9 gives nominal LPF3 group delays at 10Hz of 1.4ms for gyro
and 1.5ms for acceleration. Gyro interval averages are linearly interpolated
between consecutive samples onto acceleration's later 1.5ms delay. Both vectors
then have the same support. The corrected support ends at DRDY minus 1.5ms.
`traj::SampleSupport::IntervalAverage` describes the whole end-stamped interval;
subtracting another half interval would double-count the averaging delay.

These group delays are frequency dependent and vary between units. Linear
fractional-delay alignment is a development approximation. A nonzero 1.2ms
timing allowance remains, and `traj`'s development timing approximation is kept
explicitly enabled. Factory scaling, mounting and group delay are unqualified;
the firmware never reports `qualified=true`. Hardware capture removes host
scheduling jitter; it does not make the IMU's analogue/digital filter delay exact.

## Validation on the board

Inspect `Snapshot.timing`: pulse edges/pairs, lock, age, residual, generation,
capture errors, rejected IMU timing and unsynchronized GNSS counts. The screen
shows PPS WAIT/LOCK. Verify the configured pulse polarity/rate and DRDY against
the capture timer using a scope or logic analyser. Check that delayed UART
delivery leaves mapped measurement epochs unchanged, loss/reacquisition clears
lock correctly, and sustained display/USB work does not cause IMU late reads.
Measure filter alignment/delay before reducing the remaining uncertainty.

The focused clock regressions cover wrap extension, wrong-rate/long-gap rejection,
GPS week rollover, drift, delayed pulse labels, UART-latency invariance and loss
of lock. They were written and compile-checked; no tests or hardware experiments
were run for this change.

## Reviewed primary sources

- [Espressif S31 MCPWM low-level driver](https://github.com/espressif/esp-idf/blob/master/components/esp_hal_mcpwm/esp32s31/include/hal/mcpwm_ll.h): clock selection/division, capture latch and channel configuration.
- [S31 GPIO signal map](https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s31/include/soc/gpio_sig_map.h): PWM0_CAP0/1 inputs 95/96.
- [S31 clock definitions](https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s31/include/soc/clk_tree_defs.h): crystal source and capture clock options.
- [Unicore N4 command reference R1.14](https://en.unicorecomm.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf): CONFIG PPS, PPSSTATUS, N4 header and BESTNAVXYZ velocity latency.
- [Local Murata SCH16T-K01 Rev.6 datasheet](../../data/datasheets/sch16t-k01-datasheet-full.pdf): Table 9, sections 5.4.2–5.4.7 and Table 66.
