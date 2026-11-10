#!/usr/bin/env python3
"""Test a stage-1 panic trial, then restore a supplied known-good application.

Requires a previously confirmed healthy application and a candidate built to
panic on core 0 at stage 1, after USB startup but before application hardware
initialization. This exercises application rollback only. It never provisions
or modifies the second-stage bootloader. Both images use the normal inactive
slot updater. Complete wire evidence is journalled by board_check.Probe.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import serial

from board_check import Probe, connect


def image_digest(path):
    image = path.read_bytes()
    if not 80 <= len(image) <= 0x400000 or len(image) % 4 or image[0] != 0xE9:
        raise ValueError(f"not a slot-sized ESP application image: {path}")
    return hashlib.sha256(image).hexdigest()


def event(probe, events, phase, **detail):
    record = {"seconds": round(time.monotonic() - probe.started, 3),
              "phase": phase, **detail}
    events.append(record)
    probe.journal("rollback", record)
    print(json.dumps(record), flush=True)


def reconnect(probe):
    probe.device.close()
    probe.device = connect(probe.args.serial)


def wait_ready(probe, events, phase, timeout=90):
    """Require two ready polls across progressing device time and IMU counts.

    The panic candidate and fallback share a USB PID/serial. Boot stage 10,
    rather than the first re-enumeration, identifies the application that got
    past the deterministic stage-1 injection.
    """
    deadline = time.monotonic() + timeout
    previous = None
    last_error = None
    while time.monotonic() < deadline:
        try:
            if probe.system(0)["Info"]["recovery"]:
                raise RuntimeError("unexpected recovery mode while awaiting application")
            boot = probe.query("boot", b"\0\7")["System"]["BootStatus"]
            if boot["stage"] != 10:
                previous = None
                time.sleep(0.2)
                continue
            acquisition = probe.query("acquisition", b"\1\5")["Device"]["AcquisitionStatus"]
            imu = probe.query("imu", b"\1\7")["Device"]["ImuDiagnostics"]
            if not imu["ready"]:
                previous = None
                time.sleep(0.2)
                continue
            if previous is not None:
                elapsed = acquisition["updated_us"] - previous["updated_us"]
                samples = acquisition["imu_samples"] - previous["imu_samples"]
                if elapsed > 0 and samples > 0:
                    power = probe.query("power", b"\1\3")["Device"]["PowerStatus"]
                    if not power["verified"] or power["control"] is None or power["control"] & 0x27:
                        raise RuntimeError(f"battery-free charging policy failed: {power}")
                    reset = probe.query("reset", bytes([1, 8]))["Device"]["ResetStatus"]
                    evidence = {"boot": boot, "reset": reset, "power": power,
                                "imu_samples_delta": samples, "device_elapsed_us": elapsed,
                                "acquisition": acquisition, "imu": imu}
                    event(probe, events, phase, evidence=evidence)
                    return evidence
            previous = acquisition
            time.sleep(1)
        except (OSError, serial.SerialException, RuntimeError, KeyError) as error:
            last_error = str(error)
            previous = None
            event(probe, events, phase + "-reconnect", error=last_error)
            try:
                reconnect(probe)
            except (OSError, serial.SerialException, RuntimeError) as error:
                last_error = str(error)
            time.sleep(0.2)
    raise RuntimeError(f"{phase}: no stable stage-10 application within {timeout}s; {last_error}")


def require_panic(reset, source_hash, line, stage=1, core=0):
    panic = reset.get("panic")
    expected = {"file_hash": source_hash, "line": line,
                "startup_stage": stage, "core": core}
    if not isinstance(panic, dict) or any(panic.get(key) != value for key, value in expected.items()):
        raise RuntimeError(f"retained panic mismatch: {panic}; expected {expected}")


def check(probe, args, report):
    events = report["events"]
    # Root supplies an already confirmed image; a preflight also proves that
    # this fixture has a working USB/acquisition path before selecting a trial.
    preflight = wait_ready(probe, events, "preflight", args.timeout)
    if preflight["reset"].get("panic") is not None:
        raise RuntimeError("preflight has retained panic evidence; reboot the healthy image before this test")
    candidate_attempted = False
    try:
        candidate_attempted = True
        event(probe, events, "upload-candidate")
        try:
            probe.load(args.candidate)
        except (OSError, serial.SerialException, RuntimeError) as error:
            # Commit may have succeeded while its first FEED enumeration was
            # lost to the deliberate panic. The exact retained panic and later
            # stage-10 progress, not upload completion alone, determine success.
            event(probe, events, "candidate-upload-or-reconnect", error=str(error))
            reconnect(probe)
        fallback = wait_ready(probe, events, "fallback", args.timeout)
        require_panic(fallback["reset"], args.panic_source_hash, args.panic_line,
                      args.panic_stage, args.panic_core)
        report["rollback_passed"] = True
    except BaseException as error:
        report["errors"].append(f"rollback: {type(error).__name__}: {error}")
    finally:
        if candidate_attempted:
            try:
                event(probe, events, "upload-restore")
                reconnect(probe)
                probe.load(args.restore_image)
                # Exclude PSRAM/startup and the five-second trial-confirmation
                # interval before assessing the final restored application.
                time.sleep(args.settle)
                restored = wait_ready(probe, events, "restored", args.timeout)
                if restored["reset"]["panic"] is not None:
                    raise RuntimeError(f"restored image retained an unexpected panic: {restored['reset']}")
                report["restore_passed"] = True
            except BaseException as error:
                report["errors"].append(f"restore: {type(error).__name__}: {error}")
    report["passed"] = report["rollback_passed"] and report["restore_passed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--restore-image", type=Path, required=True)
    parser.add_argument("--panic-source-hash", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--panic-line", type=int, required=True)
    parser.add_argument("--panic-stage", type=int, default=1)
    parser.add_argument("--panic-core", type=int, default=0)
    parser.add_argument("--serial", default="AEVIADEVBOARD")
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument("--settle", type=float, default=30)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.panic_source_hash <= 0xFFFFFFFF or args.panic_line <= 0:
        parser.error("panic hash must be u32; panic line must be positive")
    if args.panic_stage != 1 or args.panic_core != 0:
        parser.error("this fixture requires a stage-1 core-0 panic")
    if not 0 < args.timeout <= 300 or not 30 <= args.settle <= 300:
        parser.error("timeout must be in (0,300]; settle must be in [30,300] seconds")
    try:
        candidate_hash = image_digest(args.candidate)
        restore_hash = image_digest(args.restore_image)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if candidate_hash == restore_hash:
        parser.error("candidate and restored application must be different images")
    report = {"passed": False, "rollback_passed": False, "restore_passed": False,
              "started_utc": datetime.now(timezone.utc).isoformat(), "serial": args.serial,
              "candidate_sha256": candidate_hash, "restore_sha256": restore_hash,
              "decoder_sha256": hashlib.sha256(args.decoder.read_bytes()).hexdigest(),
              "expected_panic": {"file_hash": args.panic_source_hash, "line": args.panic_line,
                                 "startup_stage": args.panic_stage, "core": args.panic_core},
              "scope": "application rollback after USB startup; no pre-Rust or bootloader fault test",
              "errors": [], "events": [], "records": []}
    probe = None
    try:
        probe = Probe(args)
        check(probe, args, report)
    except BaseException as error:
        report["errors"].append(f"fixture: {type(error).__name__}: {error}")
    finally:
        if probe is not None:
            report["records"] = probe.records
            report["journal"] = str(probe.journal_path)
            report["journal_session"] = probe.session
            probe.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "output": str(args.output),
                      "errors": report["errors"]}), flush=True)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
