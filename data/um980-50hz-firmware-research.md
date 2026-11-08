# UM980 firmware for 50 Hz SPP

## Fresh-board result, 2026-11-08

The newly assembled board reports **UM980 R4.10Build17548**, dated 2024/12/23,
hardware HRPT00-S10C-P. This is a different installed receiver version from the
older Build14259 dev-board test below. No receiver firmware update was needed
or performed.

Live commands on COM2 confirmed that SG1 rejects `SPPNAVA COM2 0.02` and
`BESTNAVA COM2 0.02` as unsupported periods. After switching to **SIGNALGROUP 8**,
both commands returned OK. A ten-second capture received **500 SPPNAVA and 500
BESTNAVA records**, with **499 consecutive 20 ms epoch steps per log and zero
CRC errors**. Both streams were transmitted simultaneously at 460800 baud.

No antenna was connected: every position status was INSUFFICIENT_OBS/NONE and
there were zero valid standalone solutions. This verifies accepted 50 Hz
standalone-log configuration and transport/epoch cadence, not 50 valid fresh
position/velocity fixes. Open-sky antenna testing remains necessary for that.

The initial probe restored SG1 and 20 Hz acquisition afterward. The subsequent
application update now defaults to SG8/50 Hz, with signal-group readback and
20 ms epoch validation. Its combined 20-second capture passed 1,000 GNSS records,
3,636 IMU intervals, 999 consecutive 20 ms steps and zero capture errors/drops.
Evidence: [50 Hz application capture](bringup-2026-11-08/gnss-50hz-startup-capture.log).
No SAVECONFIG was issued. The reusable probe is
`tools/check-gnss-50hz.ps1`. Evidence:
[installed version](bringup-2026-11-08/gnss-version-50hz-probe.log),
[SG8 command replies](bringup-2026-11-08/gnss-sg8-50hz-replies.log),
[capture summary](bringup-2026-11-08/gnss-sg8-50hz-summary.json), and
[restored application](bringup-2026-11-08/gnss-after-50hz-probe.log).

## Earlier firmware research

Checked 2026-09-27. Installed receiver reported R4.10Build14259 (May 2024) in the preceding bench test.

## Finding

**A newer firmware explicitly adding 50 Hz PVT computation exists: UM980 Build17548.** This is strong evidence for a firmware route to 50 Hz standalone positioning, but the available release note does not explicitly promise `SPPNAVA 0.02`; command acceptance and genuine fresh fixes still need testing on the receiver.

## Evidence

- Unicore's one-page [Build17548 release note](https://community.sparkfun.com/uploads/short-url/cslDsPIR9OgE3WDot9epiqZczjf.pdf), dated December 24, 2024, identifies UM980 firmware Build17548 and says: “50 Hz PVT computation was supported in SIGNALGROUP 8.” This describes computation, not just serial message frequency. The document is manufacturer-authored but hosted on SparkFun's community site.
- The older official [N4 command manual R1.2](https://en.unicore.com/uploads/file/unicore-reference-commands-manual-for-n4-high-precision-products-v2-en-r1.2.pdf), section 4.23, explicitly limits SG8 SPP calculation to 1 Hz while allowing 50 Hz RTK/raw observations; higher SPPNAV requests fail. That matches the observed old-firmware behavior.
- The current official [N4 command manual R1.14](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf) records the addition of UM980 SG8 50 Hz PVT in its revision history. PDF page 73 (printed 72), Table 4-61, now allows SG8 positioning/velocity at 50 Hz; the old SPP restriction is absent. Its PVT reliability section (printed 59) uses PVT for single-point positioning and Doppler velocity. This strengthens the SPP interpretation, but is not an explicit per-build SPPNAV guarantee.

## Package

SparkFun's official RTK Torch repository includes [UM980_R4.10Build17548.pkg](https://github.com/sparkfun/SparkFun_RTK_Torch/blob/main/UM980_Firmware/UM980_R4.10Build17548.pkg), alongside a manufacturer release note in the [UM980_Firmware folder](https://github.com/sparkfun/SparkFun_RTK_Torch/tree/main/UM980_Firmware). It is receiver firmware, independent of this project's application firmware. Compatibility with the exact dev-board/module variant should be checked before flashing. Nothing was flashed during this research.

## Verification after an approved update

Use SG8 and retry `SPPNAVA 0.02`; inspect command response, record timestamps, solution age, checksums and sustained delivery rate. An indoor test can verify accepted configuration and transport rate. Open-sky reception with corrections absent is needed to confirm valid `SOL_COMPUTED,SINGLE` fixes at fresh 20 ms epochs. Merely receiving 50 BESTNAV messages per second is insufficient proof, as the preceding old-firmware test already demonstrated that transport rate.

The current manual also lists `MODE ROVER RACING`, but the sources checked do not tie it to Build17548 or make it a prerequisite for 50 Hz PVT. Do not assume that command exists in Build17548.
