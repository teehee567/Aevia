# V2 Mini installed-battery validation

The stationary V2 Mini stayed on the same Mac USB supply with a cell connected to J4. The baseline started at 22:57 AEDT on 21 November 2026. Times in the CSV files are UTC. Charge percentage, cell voltage and percentage change per hour come from the MAX17048. The BQ25622E provides charge stage, thermistor zone and programmed limits. Current in the first two runs came from its one-shot ADC.

## Charge measurements

The overnight run completed on 22 November at **04:48 AEDT**. In the reduced-load run the pack went from 72% at 23:18 to 80% at 00:25 and 90% at 03:11. Taper began at 04:34. The gauge peaked at 4.193 V and read 98% when the charger stopped. Five consecutive passive samples confirmed external power and idle charging near the 4.20 V target. Following restoration of the driving release, a single explicit bench conversion measured **0 mA battery current**, 4.185 V at the charger battery ADC, and 262 mA from the 5.201 V USB source. GNSS and IMU were ready. [Endpoint readings](v2-mini-installed-pack-endpoint.json)

The 666 samples of the overnight run span 5 h 32 min, including a settling reading immediately after boot. No subsequent sample reported a gauge or charger error. The signed Bluetooth test client read the charged pack on the restored release and obtained three battery updates in 56 driving reads. USB and Bluetooth both reported external power, idle charging and 98%. One minute after the explicit endpoint sample, a passive read confirmed the same programmed limits and autonomous mode. The installed pack remained on USB for the experiment. Its unplugged runtime and capacity have not been measured.

| Run | Observation |
| --- | --- |
| Normal driving firmware before the fix | Gauge rose from 68% to 69%. Nine samples showed 72–76 mA entering the battery, with 306–336 mA drawn from USB. The charger reported its warm zone throughout. |
| Reduced load with GNSS held in reset | Measured 80 mA entering the battery at 218–220 mA USB input. The charger die cooled from 39.5 °C to 36.5 °C over 150 seconds. The thermistor moved to pre-warm. The programmed 4.20 V limit and 0.20 V warm-zone reduction account for the roughly 4.00 V warm ceiling. |
| Read-only configuration check | Charge-current limit 80 mA. Termination threshold 60 mA, termination enabled. Host watchdog 50 s. Charger in autonomous mode. The 3,200 mA input-limit register is separate from the board’s external ILIM resistor. |
| Driving release after the fix | Repeated ordinary USB power queries remained in autonomous mode across several watchdog periods. GNSS and IMU both returned ready after boot. At 72% the running receiver returned to the warm zone and temporarily stopped charging near 4.0 V. |

The normal `power` diagnostic now reads the existing settings and reports when it has withheld an ADC conversion. An explicit `power_sample` retains the bench measurement. Previously an ADC register write entered host mode, and successive watchdog expiries could halve the current limit down to 80 mA. The bench readings establish the present limit. They do not establish its history before logging began. The firmware does not raise the limit or alter charge voltage, termination or the thermal profile. The charger uses board-mounted TH1, so a temperature zone is not a direct measurement of cell temperature. [TI BQ25622E, sections 8.3.5 and 8.4.1](https://www.ti.com/lit/ds/symlink/bq25622e.pdf)

## App and firmware

Idle and recording driving frames keep their existing layout for legacy clients. The updated app opts into a battery trailer over the cached settings and display characteristics. The Mac received three battery updates in 56 reads. Bluetooth and USB both reported 71%, 4.004 V, +15.392 percentage points per hour, external power and pre-warm. This change needs no extra periodic Bluetooth transaction. The app shows a time-to-full estimate in the normal charging zone and a runtime estimate when discharging. Estimates vary with load and taper. [MAX17048 CRATE register](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX17048-MAX17049.pdf#page=13)

159 firmware host tests, 119 Rust app tests and 28 Flutter workflows passed. The signed iPhone release and embedded release built successfully. The driving application image is 1,230,352 bytes, SHA-256 `804c7bed2440c277d1bbf4fb0fdf94b2546cfa880be32c60b3b40f40495754f7`.

The raw [baseline](v2-mini-installed-pack-charge.csv), [reduced-load ADC readings](v2-mini-installed-pack-phases.csv), and [passive charge log](v2-mini-installed-pack-passive.csv) distinguish charge current from total USB input. In the passive charge log, `watchdog_expired=True` identifies the charger’s autonomous/default mode. No fresh ADC current is inferred from that log.
