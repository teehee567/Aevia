# /// script
# dependencies = ["pyserial==3.5"]
# ///
"""Upload an application ELF or .bin through the installed USB recovery."""

import argparse
import hashlib
import subprocess
import time
from pathlib import Path

import serial
from serial.tools import list_ports

APP_PID, RECOVERY_PID = 0xFEED, 0xCAFE
READY, REBOOT = b"\x01\x01\0", b"\x02\x01\0"


def uint(value):
    encoded = bytearray()
    while value >= 128:
        encoded.append((value & 127) | 128)
        value >>= 7
    return encoded + bytes([value])


def frame(payload):
    encoded = bytearray()
    for chunk in payload.split(b"\0"):
        while len(chunk) >= 254:
            encoded.extend(b"\xff" + chunk[:254])
            chunk = chunk[254:]
        encoded.extend(bytes([len(chunk) + 1]) + chunk)
    return encoded + b"\0"


def connect(pid=None, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ports = [p for p in list_ports.comports() if p.vid == 0xC0C0
                 and p.pid in (APP_PID, RECOVERY_PID) and pid in (None, p.pid)]
        if len(ports) > 1:
            raise RuntimeError("Connect only one Aevia device")
        if ports:
            link = None
            try:
                link = serial.Serial(ports[0].device, 115200, timeout=0.1, write_timeout=5)
                link.rts, link.dtr = False, True
                link.reset_input_buffer()
                return link, ports[0].pid
            except OSError:
                if link is not None:
                    link.close()
        time.sleep(0.1)
    raise TimeoutError("Aevia USB did not appear")


def disconnect(link):
    try:
        link.dtr = False
        time.sleep(0.05)  # Let firmware observe the closed session before reconnecting.
    except OSError:
        pass
    finally:
        link.close()


def request(link, payload, reboot=False):
    link.write(frame(payload))
    reply = bytearray()
    deadline = time.monotonic() + 35
    try:
        while len(reply) < 3:
            if time.monotonic() >= deadline:
                raise TimeoutError("No loader response")
            data = link.read(3 - len(reply))
            reply.extend(data)
            if not data:
                time.sleep(0.001)
    except OSError:
        if reboot:
            return  # The caller must observe the expected USB mode after reset.
        raise
    if reply != (REBOOT if reboot else READY):
        raise RuntimeError(f"Loader rejected request: {reply.hex()}")


def recovery():
    link, pid = connect()
    if pid == APP_PID:
        try:
            request(link, b"\0", reboot=True)
        finally:
            disconnect(link)
        link, _ = connect(RECOVERY_PID)
    return link


def upload(image):
    if not 288 <= len(image) <= 0xF70000 or image[0] != 0xE9 or image[12:14] != b"\x20\0":
        raise ValueError("Expected an ESP32-S31 application .bin fitting the app partition")
    link = recovery()
    try:
        request(link, b"\x01" + uint(len(image)) + hashlib.sha256(image).digest())
        for offset in range(0, len(image), 256):
            data = image[offset:offset + 256]
            request(link, b"\x02" + uint(len(data)) + data, reboot=offset + len(data) == len(image))
    finally:
        disconnect(link)
    link, _ = connect(APP_PID)
    port = link.port
    disconnect(link)
    return port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("image", nargs="?", type=Path)
    mode.add_argument("--recover", action="store_true")
    args = parser.parse_args()
    if args.recover:
        link = recovery()
        print(f"Recovery ready on {link.port}")
        disconnect(link)
        return
    image = args.image.read_bytes()
    if image[:4] == b"\x7fELF":
        if args.image.name not in ("firmware", "firmware.elf"):
            parser.error("Upload the firmware application; install recovery separately")
        firmware = Path(__file__).resolve().parent.parent
        binary = firmware / "target/upload/firmware.bin"
        binary.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([
            "espflash", "save-image", "--chip", "esp32s31", "--flash-size", "16mb",
            "--flash-mode", "dio", "--flash-freq", "40mhz", "--partition-table",
            str(firmware / "partitions.csv"), "--target-app-partition", "app",
            str(args.image), str(binary),
        ], check=True)
        image = binary.read_bytes()
    print(f"Uploaded and booted application on {upload(image)}")


if __name__ == "__main__":
    main()
