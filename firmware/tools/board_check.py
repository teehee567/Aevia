#!/usr/bin/env python3
"""Load an inactive firmware slot and capture typed USB diagnostics (pyserial)."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import serial
from serial.tools import list_ports


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
        self.device = connect(args.serial)
        self.decoder = subprocess.Popen([str(args.decoder)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.started = time.monotonic()
        self.records = []

    def query(self, name, payload):
        self.device.write(encode(payload))
        self.device.flush()
        wire = self.device.read_until(b"\0", size=1024)
        if not wire.endswith(b"\0"):
            raise RuntimeError(f"{name}: incomplete response {wire.hex()}")
        self.decoder.stdin.write(wire.hex() + "\n")
        self.decoder.stdin.flush()
        decoded = self.decoder.stdout.readline()
        if not decoded:
            raise RuntimeError(f"{name}: wire decoder exited")
        response = json.loads(decoded)
        self.records.append({"seconds": round(time.monotonic() - self.started, 3),
                             "query": name, "wire": wire.hex(), "response": response})
        return response

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
        while time.monotonic() < deadline:
            summary = {}
            for name, payload in [("boot", b"\0\7"), ("status", b"\1\0"),
                                  ("power", b"\1\3"), ("peripherals", b"\1\4"),
                                  ("acquisition", b"\1\5"), ("gnss", b"\1\6"),
                                  ("imu", b"\1\7"), ("reset", bytes([1, 8]))]:
                response = self.query(name, payload)
                summary[name] = response
                if name == "acquisition":
                    acquisitions.append(response["Device"]["AcquisitionStatus"])
                if name == "imu":
                    imu_health.append(response["Device"]["ImuDiagnostics"])
            print(json.dumps(summary), flush=True)
            time.sleep(self.args.interval)
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
    parser.add_argument("--brightness", type=int, default=96)
    parser.add_argument("--settle", type=float, default=0,
                        help="seconds before sampling, to exclude startup/OTA confirmation")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.seconds <= 0 or args.interval <= 0 or args.settle < 0:
        parser.error("seconds/interval must be positive; settle must be nonnegative")
    if not 0 <= args.brightness <= 255:
        parser.error("brightness must be between 0 and 255")
    probe = Probe(args)
    try:
        probe.capture()
    finally:
        probe.device.close()
        probe.decoder.stdin.close()
        probe.decoder.wait(timeout=3)
        args.output.write_text(json.dumps(probe.records, indent=2) + "\n")


if __name__ == "__main__":
    main()
