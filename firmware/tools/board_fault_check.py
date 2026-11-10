#!/usr/bin/env python3
"""Exercise USB framing and inactive-slot rejection without selecting a candidate.

Requires a previously confirmed application and a matching built application
image. Only the inactive slot's first sector is written. The fixture is left
running its original application; this does not test power cuts or bootloader
rollback before Rust startup.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

from board_check import Probe, connect, encode, varint


def check(probe, image):
    def receive(name):
        wire = probe.device.read_until(b"\0", size=1024)
        if not wire.endswith(b"\0"):
            raise RuntimeError(f"{name}: missing response")
        probe.decoder.stdin.write(wire.hex() + "\n")
        probe.decoder.stdin.flush()
        response = json.loads(probe.decoder.stdout.readline())
        probe.records.append({"seconds": round(time.monotonic() - probe.started, 3),
                              "query": name, "wire": wire.hex(), "response": response})
        return response

    def expect(payload, response):
        actual = probe.query("fault", payload)
        if actual != response:
            raise RuntimeError(f"unexpected fault response: {actual}; wanted {response}")

    def rejected(command, body, error):
        expect(bytes([0, command]) + body, {"System": {"Error": error}})

    def begin(size, digest):
        probe.system(3, varint(size) + digest, "Ok")

    def data(offset, part):
        return varint(offset) + varint(len(part)) + part

    info = probe.system(0)
    if not info["Info"]["recovery"]:
        probe.reboot(6, 0xCAFE)
    recovery_info = probe.system(0)
    if not recovery_info["Info"]["recovery"]:
        raise RuntimeError("Recover did not select recovery mode")
    # Allow the recovery power owner to establish its first verified readback.
    time.sleep(2)
    power = probe.query("power", bytes([1, 3]))["Device"]["PowerStatus"]
    if not power["verified"] or power["control"] is None or power["control"] & 0x27:
        raise RuntimeError(f"recovery charging policy failed: {power}")

    expect(b"", {"System": {"Error": "Command"}})
    expect(b"\xff", {"System": {"Error": "Command"}})
    expect(b"\0\0\x42", {"System": {"Error": "Command"}})
    expect(b"\x01" * 300, {"System": {"Error": "Command"}})
    probe.system(0)
    probe.device.write(b"\x03\x42\0")  # Truncated COBS block.
    if receive("malformed-cobs") != {"System": {"Error": "Command"}}:
        raise RuntimeError("malformed COBS accepted")
    # Multiple frames in one write must yield independent replies.
    probe.device.write(encode(b"\0\0") * 2)
    for _ in range(2):
        if receive("coalesced") != {"System": recovery_info}:
            raise RuntimeError("coalesced response mismatch")
    for byte in encode(b"\0\0"):
        probe.device.write(bytes([byte]))
        time.sleep(0.01)
    if receive("fragmented") != {"System": recovery_info}:
        raise RuntimeError("fragmented response mismatch")

    rejected(3, varint(81) + bytes(32), "Size")
    begin(len(image), hashlib.sha256(image).digest())
    rejected(4, data(256, image[:256]), "Sequence")
    rejected(5, b"", "Sequence")
    begin(80, bytes(32))
    rejected(4, data(0, bytes(80)), "Image")
    begin(len(image), hashlib.sha256(image).digest())
    probe.system(4, data(0, image[:256]), "Ok")
    probe.system(1, expected="Ok")
    rejected(5, b"", "Sequence")
    begin(len(image), hashlib.sha256(image).digest())
    probe.system(4, data(0, image[:256]), "Ok")
    expect(b"\xff", {"System": {"Error": "Command"}})
    rejected(5, b"", "Sequence")
    begin(len(image), hashlib.sha256(image).digest())
    probe.system(4, data(0, image[:256]), "Ok")
    probe.device.close()
    time.sleep(0.3)
    probe.device = connect(probe.args.serial, 0xCAFE)
    rejected(5, b"", "Sequence")
    # Deliberately invalid hash must reject before publishing a boot selector.
    short = bytearray(80)
    short[:24] = image[:24]
    short[1] = 1
    begin(len(short), bytes(32))
    probe.system(4, data(0, short), "Ok")
    rejected(5, b"", "Hash")
    probe.system(0)
    probe.reboot(2, 0xFEED)
    if probe.system(0)["Info"]["recovery"]:
        raise RuntimeError("Run did not restore application mode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--serial", default="AEVIADEVBOARD")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    image = args.image.read_bytes()
    if len(image) < 256 or image[0] != 0xE9:
        parser.error("image must be a built ESP application binary")
    probe = Probe(args)
    passed = False
    try:
        check(probe, image)
        passed = True
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"passed": passed, "records": probe.records}, indent=2) + "\n")
        probe.close()
    print(json.dumps({"passed": passed, "output": str(args.output)}))


if __name__ == "__main__":
    main()
