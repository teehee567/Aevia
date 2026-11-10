# Connected-board bench readiness, 2026-11-10

The acquisition firmware and repeatable initial bench procedures are prepared.
The final stationary functional gate, USB/update rejection checks and deliberate
application-trial rollback test passed. Measurements remain `qualified=false`.
An earlier isolated charger verification failure remains an open observation;
this clean run does not identify or close its original cause.

The board stayed on its desk, powered through USB, without a GNSS antenna,
display, buttons or battery. The normal application was restored after the panic
test and left running, with charging and the charger watchdog disabled.

| Check | Observed result |
| --- | --- |
| Firmware/loader host regressions | 97 passed |
| HAL host regressions | 179 passed |
| Host fixture/offline acceptance regressions | 25 passed |
| ESP32-S31 optimized build and strict Clippy | Passed |
| Stationary soak | 1810.005 seconds, 360 polls |
| Admitted IMU samples during soak | 2,631,521 |
| Per-interval IMU cadence | 1453.339..1454.470 Hz |
| New IMU/GNSS/UART/parser errors or queue drops | Zero |
| Charger-policy attempts since boot | 1778/1778 successful; no retained failure |
| USB/update rejection checks | Passed; 36 retained replies |
| Deliberate stage-1 trial panic | Exact core/source/line record observed after automatic fallback |
| Normal image restoration and follow-up | Passed; 59.999-second follow-up gate |

The application image is 3,334,752 bytes of a 4,194,304-byte slot
(79.51%). Its SHA-256 is
`cb0f7a555710c85d9daa9d4ad387d4e5546044f5af01c5426f241be6088bd1e8`. The restored image has the same digest.
Both slots contain valid normal images after restoration. The temporary panic
injection was removed from source and the ordinary release was rebuilt to the
same application digest before the test.

The code changes fix command-acknowledgement checksums, fair polling between
GNSS return paths, and watchdog supervision of each sensor owner. Missing
accessories have explicit fitment flags. Appended UART and charger diagnostics
retain failures after recovery without changing existing response layouts.
The host tools retain raw wire journals, typed captures, image hashes and
machine-readable verdicts, and can stop at the first failed bench comparison.

The trial deliberately panicked on core 0 at startup stage 1, at line
98 in `firmware\src\runtime.rs` (source hash
`0x36184ef7`). The previous healthy image
automatically reached stage 10 with advancing IMU samples and the exact retained
panic. The fixture then reinstalled the normal image and verified a clean reset
record. This exercises recovery after Rust startup. Bootloader fallback for
faults before Rust startup still needs separate provisioning and fault tests.

Earlier observations are retained in the workspace. The initial receiver
reported three false checksum failures in 90 seconds because valid CONFIG ACKs
include the leading `$` in their XOR. A subsequent 120-second run exposed one
UART/CRC failure. After fair polling, the 302.404-second run had no new sensor or
transport errors, but one charger verification failure recovered before the
next poll. That run was stopped and its typed stdout recovered; its in-memory
raw frame array was lost. The new append-only journal covers such host
termination, and the new charger diagnostics will distinguish transaction
failure from register-policy mismatch if the observation recurs.

Antenna-backed PPS association, valid navigation seeding/fusion and navigating
CPU throughput still need physical tests. Measure capture/filter timing,
mounting, lever arm, vibration-isolated full-rate noise and thermal behavior
before accuracy qualification. Motion, power-interruption and accessory tests
remain separate. SD session recording and radio stacks are outside this
acquisition profile.

Follow the [evaluation procedure](evaluation.md) to repeat the tests. The
[retained result](bench-results/2026-11-10.json) includes source/image digests and
complete summary diagnostics. Workspace evidence includes the
[soak](../../build/bench/diagnostic-soak.json),
[raw journal](../../build/bench/diagnostic-soak.frames.jsonl),
[USB faults](../../build/bench/faults-updated.json),
[rollback](../../build/bench/rollback.json), and
[post-restoration check](../../build/bench/post-restore.json).
