#!/usr/bin/env python3
"""Load an inactive firmware slot and capture typed USB diagnostics (pyserial)."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

import serial
from serial.tools import list_ports

from bench_acceptance import ALL_RESPONSES, evaluate, validate_limits


def encode(data):
    output = bytearray([0])
    position, code = 0, 1
    for byte in data:
        if byte == 0:
            output[position] = code
            position, code = len(output), 1
            output.append(0)
        else:
            output.append(byte)
            code += 1
            if code == 255:
                output[position] = code
                position, code = len(output), 1
                output.append(0)
    output[position] = code
    return bytes(output) + b"\0"


def varint(value):
    output = bytearray()
    while value >= 128:
        output.append((value & 127) | 128)
        value >>= 7
    output.append(value)
    return bytes(output)


def connect(serial_number, pid=None):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        for port in list_ports.comports():
            if port.vid == 0xC0C0 and port.serial_number == serial_number and (pid is None or port.pid == pid):
                try:
                    device = serial.Serial(port.device, 115200, timeout=3, write_timeout=3)
                    device.dtr = True
                    time.sleep(0.15)
                    device.reset_input_buffer()
                    return device
                except serial.SerialException:
                    pass
        time.sleep(0.1)
    raise RuntimeError("Aevia USB device did not become available")


class Probe:
    def __init__(self, args):
        self.args = args
        self.started = time.monotonic()
        self.records = []
        self.journal_path = None
        self._journal_file = None
        self.session = datetime.now(timezone.utc).isoformat()
        if getattr(args, "output", None) is not None:
            self.journal_path = Path(args.output).with_suffix(".frames.jsonl")
            self.journal_path.parent.mkdir(parents=True, exist_ok=True)
            self._journal_file = self.journal_path.open("a", encoding="utf-8")
        try:
            self.decoder = subprocess.Popen([str(args.decoder)], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, text=True, bufsize=1)
            self.device = connect(args.serial)
        except BaseException:
            self.close()
            raise

    def close(self):
        """Close all owned handles; safe for fault-tool finally blocks and repeats."""
        try:
            if getattr(self, "device", None) is not None:
                self.device.close()
            if getattr(self, "decoder", None) is not None:
                try:
                    self.decoder.stdin.close()
                except OSError:
                    pass
                try:
                    self.decoder.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.decoder.kill()
                    self.decoder.wait(timeout=3)
        finally:
            if getattr(self, "_journal_file", None) is not None:
                self._journal_file.close()

    def journal(self, event, record):
        journal = getattr(self, "_journal_file", None)
        if journal is not None:
            journal.write(json.dumps({"session": self.session, "event": event,
                                      "record": len(self.records) - 1, **record}) + "\n")
            journal.flush()

    def query(self, name, payload):
        self.device.write(encode(payload))
        self.device.flush()
        wire = self.device.read_until(b"\0", size=1024)
        record = {"seconds": round(time.monotonic() - self.started, 3),
                  "query": name, "wire": wire.hex(), "response": None}
        self.records.append(record)
        self.journal("received", record)
        try:
            if not wire.endswith(b"\0"):
                raise RuntimeError(f"{name}: incomplete response {wire.hex()}")
            self.decoder.stdin.write(wire.hex() + "\n")
            self.decoder.stdin.flush()
            decoded = self.decoder.stdout.readline()
            if not decoded:
                raise RuntimeError(f"{name}: wire decoder exited")
            response = json.loads(decoded)
            record["response"] = response
            if name in ALL_RESPONSES:
                outer, inner = ALL_RESPONSES[name]
                if not isinstance(response.get(outer), dict) or inner not in response[outer]:
                    raise RuntimeError(f"{name}: unexpected response {response}")
            self.journal("decoded", record)
            return response
        except BaseException as failure:
            record["error"] = f"{type(failure).__name__}: {failure}"
            self.journal("error", record)
            raise

    def system(self, command, body=b"", expected=None):
        response = self.query("system", bytes([0, command]) + body)
        if expected is not None and response != {"System": expected}:
            raise RuntimeError(f"command {command}: {response}")
        return response["System"]

    def reboot(self, command, pid):
        self.system(command, expected="Reboot")
        self.device.close()
        time.sleep(0.4)
        self.device = connect(self.args.serial, pid)

    def load(self, path):
        image = path.read_bytes()
        if not (80 <= len(image) <= 0x400000) or len(image) % 4 or image[0] != 0xE9:
            raise RuntimeError("expected a slot-sized ESP application .bin image")
        if not self.system(0)["Info"]["recovery"]:
            self.reboot(6, 0xCAFE)
        self.system(3, varint(len(image)) + hashlib.sha256(image).digest(), "Ok")
        for offset in range(0, len(image), 256):
            part = image[offset:offset + 256]
            self.system(4, varint(offset) + varint(len(part)) + part, "Ok")
            if offset % 524288 == 0:
                print(f"Uploaded {offset}/{len(image)} bytes", flush=True)
        self.reboot(5, 0xFEED)

    def capture(self):
        if self.args.image:
            self.load(self.args.image)
        if self.system(0)["Info"]["recovery"]:
            if not self.args.run:
                raise RuntimeError("device is in recovery; use --run to start acquisition")
            self.reboot(2, 0xFEED)
        self.query("indicators", bytes([1, 2, 0, self.args.brightness]))
        if self.args.settle:
            time.sleep(self.args.settle)
        deadline = time.monotonic() + self.args.seconds
        acquisitions = []
        imu_health = []
        queries = [("boot", b"\0\7"), ("status", b"\1\0"),
                   ("power", b"\1\3"), ("peripherals", b"\1\4"),
                   ("acquisition", b"\1\5"), ("gnss", b"\1\6"),
                   ("imu", b"\1\7"), ("reset", bytes([1, 8]))]
        if self.args.transport:
            queries.append(("transport", bytes([1, 9])))
        if getattr(self.args, "power_diagnostics", False):
            queries.append(("power_diagnostics", bytes([1, 10])))
        if getattr(self.args, "fix", False):
            queries.append(("fix", bytes([1, 11])))
        if getattr(self.args, "power_charge", False):
            queries.append(("power_charge", bytes([1, 12])))
        while True:
            summary = {}
            for name, payload in queries:
                response = self.query(name, payload)
                summary[name] = response
                if name == "acquisition":
                    acquisitions.append(response["Device"]["AcquisitionStatus"])
                if name == "imu":
                    imu_health.append(response["Device"]["ImuDiagnostics"])
            print(json.dumps(summary), flush=True)
            if getattr(self.args, "fail_fast", False) and len(acquisitions) >= 2:
                report = evaluate(self.records, 0, self.args.minimum_imu_hz,
                                  self.args.maximum_imu_hz, self.args.stationary,
                                  getattr(self.args, "navigation", False))
                if not report["passed"]:
                    raise RuntimeError("bench fail-fast: " + "; ".join(report["failures"]))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(self.args.interval, remaining))
        if self.args.expect_imu:
            if len(acquisitions) < 2 or any(
                    current["imu_samples"] <= previous["imu_samples"]
                    for previous, current in zip(acquisitions, acquisitions[1:])):
                raise RuntimeError("IMU acquisition did not advance")
            for field in ("imu_errors", "imu_drops"):
                if any(sample[field] != acquisitions[0][field] for sample in acquisitions[1:]):
                    raise RuntimeError(f"{field} changed during capture")
            if not all(sample["ready"] for sample in imu_health):
                raise RuntimeError("IMU was not ready throughout capture")
            if any(sample["attempts"] != imu_health[0]["attempts"] for sample in imu_health[1:]):
                raise RuntimeError("IMU reinitialized during capture")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--serial", default="AEVIADEVBOARD")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--interval", type=float, default=5)
    parser.add_argument("--expect-imu", action="store_true")
    parser.add_argument("--transport", action="store_true",
                        help="query appended GNSS UART diagnostics; requires matching new firmware/decoder")
    parser.add_argument("--power-diagnostics", action="store_true",
                        help="also retain charger failure-stage and I2C diagnostics")
    parser.add_argument("--fix", action="store_true",
                        help="also retain receiver fix and raw PPS diagnostics, independent of fusion")
    parser.add_argument("--power-charge", action="store_true",
                        help="also verify the fitted-battery charging profile; requires matching firmware/decoder")
    parser.add_argument("--bench", action="store_true",
                        help="apply full functional bench gates and save a .summary.json verdict")
    parser.add_argument("--fail-fast", action="store_true",
                        help="stop after the first failed complete bench comparison (requires --bench)")
    parser.add_argument("--stationary", action="store_true",
                        help="also require gravity magnitude and near-zero angular rate; implies --bench")
    parser.add_argument("--navigation", action="store_true",
                        help="require sustained GNSS/PPS lock, fresh output and fusion progress; implies --bench")
    parser.add_argument("--minimum-imu-hz", type=float, default=675)
    parser.add_argument("--maximum-imu-hz", type=float, default=800)
    parser.add_argument("--brightness", type=int, default=96)
    parser.add_argument("--settle", type=float, default=0,
                        help="seconds before sampling, to exclude startup/OTA confirmation")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.seconds, args.interval, args.settle)):
        parser.error("seconds, interval and settle must be finite")
    if args.seconds <= 0 or args.interval <= 0 or args.settle < 0:
        parser.error("seconds/interval must be positive; settle must be nonnegative")
    if not 0 <= args.brightness <= 255:
        parser.error("brightness must be between 0 and 255")
    try:
        validate_limits(0, args.minimum_imu_hz, args.maximum_imu_hz)
    except ValueError as error:
        parser.error(str(error))
    args.bench = args.bench or args.stationary or args.navigation
    if args.fail_fast and not args.bench:
        parser.error("--fail-fast requires --bench, --stationary or --navigation")
    probe = None
    error = None
    started = datetime.now(timezone.utc).isoformat()
    try:
        probe = Probe(args)
        probe.capture()
    except BaseException as failure:
        error = f"{type(failure).__name__}: {failure}"
        raise
    finally:
        records = probe.records if probe is not None else []
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(records, indent=2) + "\n")
        if probe is not None:
            probe.close()
        if args.bench:
            report = evaluate(records, args.seconds * 0.98, args.minimum_imu_hz,
                              args.maximum_imu_hz, args.stationary, args.navigation)
            if error is not None:
                report["passed"] = False
                report["failures"].append(error)
            report.update({"started_utc": started, "serial": args.serial,
                           "requested_seconds": args.seconds, "settle_seconds": args.settle,
                           "transport_requested": args.transport,
                           "power_diagnostics_requested": args.power_diagnostics,
                           "fix_requested": args.fix,
                           "power_charge_requested": args.power_charge,
                           "journal": str(probe.journal_path) if probe is not None else None,
                           "journal_session": probe.session if probe is not None else None,
                           "capture_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
                           "decoder_sha256": hashlib.sha256(args.decoder.read_bytes()).hexdigest(),
                           "uploaded_image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest()
                           if args.image else None})
            summary_path = args.output.with_suffix(".summary.json")
            summary_path.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps({"bench_passed": report["passed"], "summary": str(summary_path),
                              "failures": report["failures"]}), flush=True)
            if error is None and not report["passed"]:
                raise SystemExit(1)


if __name__ == "__main__":
    main()
