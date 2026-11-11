#!/usr/bin/env python3
"""Control recordings over USB, download verified logs, and export them offline."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
import zlib

from usb_frame import FrameError, FrameReader

PAGE_BYTES = 512
READ_BYTES = 4096
MAX_RESPONSE_FRAME = 4608
MAGIC = b"AEVLOG1\0"
PAGE_HEADER = struct.Struct("<8sIIHHQ")
RECORD_HEADER = struct.Struct("<BBHQQI")
KINDS = {1: "metadata", 2: "imu", 3: "gnss_com2", 4: "gnss_com1", 5: "pps",
         6: "fault", 7: "temperature", 8: "gnss_tx", 9: "imu_configuration",
         10: "health", 11: "gnss_raw_coverage", 12: "reset_evidence", 13: "bootstrap_generation",
         14: "recording_identity", 15: "close_intent", 255: "end"}


class RecordingError(RuntimeError):
    pass


def varint(value):
    if not isinstance(value, int) or value < 0 or value > 0xFFFFFFFFFFFFFFFF:
        raise ValueError("unsigned integer out of range")
    result = bytearray()
    while value >= 128:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)


def cobs_encode(data):
    result = bytearray([0])
    position, code = 0, 1
    for byte in data:
        if byte == 0:
            result[position] = code
            position, code = len(result), 1
            result.append(0)
        else:
            result.append(byte)
            code += 1
            if code == 255:
                result[position] = code
                position, code = len(result), 1
                result.append(0)
    result[position] = code
    return bytes(result) + b"\0"


def request_bytes(request_id, command, **fields):
    if not 0 <= request_id <= 0xFFFFFFFF:
        raise ValueError("request ID out of range")
    index = {"status": 0, "start": 1, "stop": 2, "list": 3, "read": 4, "result": 5}[command]
    body = b""
    if command == "list":
        if not 0 <= fields["after_id"] <= 0xFFFFFFFF:
            raise ValueError("session ID out of range")
        body = varint(fields["after_id"])
    elif command == "read":
        if not 0 <= fields["session"] <= 0xFFFFFFFF or not 1 <= fields["len"] <= READ_BYTES:
            raise ValueError("invalid read range")
        body = varint(fields["session"]) + varint(fields["offset"]) + varint(fields["len"])
    return cobs_encode(b"\x02" + varint(request_id) + bytes([index]) + body)


class Client:
    """One outstanding operation, with explicit polling and no mutation replay on reconnect."""
    def __init__(self, args):
        import serial
        from serial.tools import list_ports
        self.timeout = args.timeout
        self.device = None
        self.decoder = None
        try:
            if args.port:
                port = args.port
            else:
                ports = [p for p in list_ports.comports()
                         if p.vid == 0xC0C0 and p.serial_number == args.serial]
                if len(ports) != 1:
                    raise RecordingError("expected one connected Aevia device; use --port if needed")
                port = ports[0].device
            self.device = serial.Serial(port, 115200, timeout=3, write_timeout=3)
            self.frame_reader = FrameReader(self.device, max_bytes=512, timeout=3)
            self.device.dtr = True
            time.sleep(0.15)
            self.device.reset_input_buffer()
            self.decoder = subprocess.Popen([str(args.decoder)], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, text=True, bufsize=1)
            info = self._exchange(cobs_encode(b"\0\x08"))
            protocol = info.get("System", {}).get("ProtocolInfo", {})
            self.configure_protocol(protocol)
            self.next_request_id = None
            self.status()
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.device is not None:
            self.device.close()
            self.device = None
        if self.decoder is not None:
            try:
                self.decoder.stdin.close()
            except OSError:
                pass
            try:
                self.decoder.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.decoder.kill()
                self.decoder.wait(timeout=3)
            self.decoder = None

    def _exchange(self, frame):
        self.device.write(frame)
        self.device.flush()
        try:
            wire = self.frame_reader.read_frame()
        except FrameError as failure:
            raise RecordingError(str(failure)) from failure
        self.decoder.stdin.write(wire.hex() + "\n")
        self.decoder.stdin.flush()
        line = self.decoder.stdout.readline()
        if not line:
            raise RecordingError("matching firmware decoder rejected the response")
        return json.loads(line)

    def _reply(self, request_id, command, **fields):
        response = self._exchange(request_bytes(request_id, command, **fields)).get("Recording")
        if not isinstance(response, dict) or response.get("request_id") != request_id:
            raise RecordingError("unexpected response or request ID; reconnect before retrying")
        return response["reply"]

    def device_status(self, command, response_name):
        response = self._exchange(cobs_encode(bytes([1, command]))).get("Device", {})
        value = response.get(response_name)
        if not isinstance(value, dict):
            raise RecordingError(f"expected device {response_name} response")
        return value

    def configure_protocol(self, protocol):
        if protocol.get("protocol_version") != 1 or protocol.get("schema_version") != 1:
            raise RecordingError("unsupported firmware protocol/schema")
        if not protocol.get("capabilities", 0) & (1 << 7):
            raise RecordingError("this firmware does not support SD recording")
        bound = protocol.get("max_response_frame")
        if type(bound) is not int or not 512 <= bound <= MAX_RESPONSE_FRAME:
            raise RecordingError("unsupported response frame limit")
        # Bootstrap is bounded to512; only trusted discovery grows the reader.
        self.frame_reader.max_bytes = bound

    def status(self):
        reply = self._reply(0, "status")
        if not isinstance(reply, dict) or "Status" not in reply:
            raise RecordingError(f"status failed: {reply}")
        status = reply["Status"]
        if status["format_version"] != 1 or not 1 <= status["max_read_len"] <= READ_BYTES:
            raise RecordingError("unsupported recording format or read limit")
        self.max_read_len = status["max_read_len"]
        # Reserve conservative worst-case enum, integer and COBS overhead.
        payload_bound = self.max_read_len + 48
        if payload_bound + payload_bound // 254 + 2 > self.frame_reader.max_bytes:
            raise RecordingError("recording read limit exceeds the advertised response frame")
        self.next_request_id = status["next_request_id"]
        return status

    def operation(self, command, **fields):
        request_id = self.next_request_id
        if not request_id:
            raise RecordingError("operation tokens exhausted; reconnect after a device restart")
        reply = self._reply(request_id, command, **fields)
        deadline = time.monotonic() + self.timeout
        while reply == "Pending":
            if time.monotonic() >= deadline:
                raise RecordingError(f"operation {request_id} timed out; inspect status before retrying mutations")
            time.sleep(0.0005 if command == "read" else 0.01)
            reply = self._reply(request_id, "result")
        if not isinstance(reply, dict) or len(reply) != 1:
            raise RecordingError(f"invalid recording response: {reply}")
        if "Error" in reply:
            raise RecordingError(f"{command}: {reply['Error']}")
        self.next_request_id = request_id + 1 if request_id < 0xFFFFFFFF else 0
        return reply

    def sessions(self):
        after_id = 0
        while True:
            reply = self.operation("list", after_id=after_id)
            if "Entry" not in reply:
                raise RecordingError("expected a recording entry")
            entry = reply["Entry"]
            if entry is None:
                return
            if not after_id < entry["session"] <= 0xFFFFFFFF:
                raise RecordingError("session list did not advance")
            after_id = entry["session"]
            yield entry

    def read(self, session, offset, length):
        if not 1 <= length <= self.max_read_len:
            raise RecordingError("read length exceeds the negotiated limit")
        reply = self.operation("read", session=session, offset=offset, len=length)
        data = reply.get("Data")
        if not isinstance(data, dict) or data["session"] != session or data["offset"] != offset:
            raise RecordingError("read response identifies different data")
        value = bytes(data["bytes"])
        if len(value) != length or zlib.crc32(value) != data["crc32"]:
            raise RecordingError("read response length or CRC mismatch")
        return value


def inspect_page(page, index, identity=None):
    if len(page) != PAGE_BYTES:
        raise RecordingError(f"truncated page {index}")
    magic, session, number, length, flags, nonce = PAGE_HEADER.unpack_from(page)
    if magic != MAGIC or number != index or length > 480 or flags not in (0, 1):
        raise RecordingError(f"invalid page header at {index}")
    if any(page[28 + length:508]) or zlib.crc32(page[:508]) != struct.unpack_from("<I", page, 508)[0]:
        raise RecordingError(f"page CRC/padding failure at {index}")
    if identity is not None and (session, nonce) != identity:
        raise RecordingError(f"session/nonce changed at page {index}")
    return (session, nonce), bool(flags), page[28:28 + length]


def health_payload(payload):
    if len(payload) != 41 or payload[0] != 1:
        raise RecordingError("unsupported health/end payload")
    values = struct.unpack_from("<QIIIIIQI", payload, 1)
    return dict(zip(("accepted_records", "dropped_records", "imu_samples", "imu_errors",
                     "gnss_uart_errors", "gnss_parse_errors", "written_pages", "queued_records"), values))


def verify_log(path, visitor=None):
    """Stream-check every byte with bounded record buffering; never loads a race into RAM."""
    result = {"format_version": 1, "pages": 0, "records": 0, "counts": {}, "closed": False,
              "missing_sequences": 0, "trailing_record_bytes": 0, "metadata": [], "end": None,
              "uart_streams": {}, "imu_invalid_timing": 0, "imu_invalid_scaled": 0,
              "gnss_raw_coverage_initial": None, "gnss_raw_coverage_latest": None,
              "expected_bootstrap_generation": None, "ignored_coverage_records": 0,
              "recording_identity": None, "close_intent": None}
    identity, previous_sequence, last_kind = None, None, None
    pending = bytearray()
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        while page := source.read(PAGE_BYTES):
            if result["closed"]:
                raise RecordingError("data follows the final page")
            digest.update(page)
            identity, result["closed"], payload = inspect_page(page, result["pages"], identity)
            result["pages"] += 1
            pending.extend(payload)
            while len(pending) >= RECORD_HEADER.size:
                kind, version, length, sequence, at_ns, reserved = RECORD_HEADER.unpack_from(pending)
                if version != 1 or length > 256 or reserved != 0:
                    raise RecordingError("unsupported or malformed record header")
                size = RECORD_HEADER.size + length + 4
                if len(pending) < size:
                    break
                if zlib.crc32(pending[:size - 4]) != struct.unpack_from("<I", pending, size - 4)[0]:
                    raise RecordingError(f"record CRC failure at sequence {sequence}")
                if previous_sequence is not None and sequence <= previous_sequence:
                    raise RecordingError("record sequences repeat or move backward")
                if last_kind == 255:
                    raise RecordingError("record follows the end marker")
                if last_kind == 15 and kind != 255:
                    raise RecordingError("close intent must immediately precede the end marker")
                result["missing_sequences"] += sequence - (0 if previous_sequence is None else previous_sequence + 1)
                previous_sequence, last_kind = sequence, kind
                record = {"kind": kind, "sequence": sequence, "at_ns": at_ns,
                          "payload": bytes(pending[RECORD_HEADER.size:size - 4])}
                del pending[:size]
                result["records"] += 1
                label = KINDS.get(kind, f"unknown_{kind}")
                result["counts"][label] = result["counts"].get(label, 0) + 1
                if kind in (3, 4, 8):
                    transport, data = uart_payload(record["payload"])
                    stream = result["uart_streams"].setdefault(label, {
                        "first_offset": transport["stream_offset"], "next_offset": transport["stream_offset"],
                        "exported_bytes": 0, "missing_bytes": 0})
                    if transport["stream_offset"] < stream["next_offset"]:
                        raise RecordingError("UART stream byte offsets repeat or move backward")
                    stream["missing_bytes"] += transport["stream_offset"] - stream["next_offset"]
                    stream["next_offset"] = transport["stream_offset"] + len(data)
                    stream["exported_bytes"] += len(data)
                elif kind == 2:
                    measurement = imu_payload(record["payload"])
                    result["imu_invalid_timing"] += not measurement["read_within_deadline"]
                    result["imu_invalid_scaled"] += not measurement["scaled_valid"]
                elif kind == 11:
                    coverage = event_payload(kind, record["payload"])
                    if coverage["bootstrap_generation"] == result["expected_bootstrap_generation"]:
                        if result["gnss_raw_coverage_initial"] is None:
                            result["gnss_raw_coverage_initial"] = coverage
                        result["gnss_raw_coverage_latest"] = coverage
                    else:
                        result["ignored_coverage_records"] += 1
                elif kind == 13:
                    if len(record["payload"]) != 4 or result["expected_bootstrap_generation"] is not None:
                        raise RecordingError("invalid or repeated bootstrap-generation marker")
                    result["expected_bootstrap_generation"] = struct.unpack("<I", record["payload"])[0]
                elif kind == 14:
                    if result["recording_identity"] is not None:
                        raise RecordingError("repeated recording identity")
                    result["recording_identity"] = event_payload(kind, record["payload"])
                    if result["recording_identity"]["file_id"] != identity[0]:
                        raise RecordingError("recording identity disagrees with page session")
                elif kind == 15:
                    if result["close_intent"] is not None:
                        raise RecordingError("repeated close intent")
                    result["close_intent"] = event_payload(kind, record["payload"])
                if kind == 1 and len(result["metadata"]) < 128:
                    try:
                        result["metadata"].append(json.loads(record["payload"]))
                    except (UnicodeError, ValueError):
                        result["metadata"].append({"invalid_json_hex": record["payload"].hex()})
                elif kind == 255:
                    result["end"] = health_payload(record["payload"])
                if visitor is not None:
                    visitor(record)
    if identity is None:
        raise RecordingError("empty recording")
    if result["closed"] and (pending or last_kind != 255):
        raise RecordingError("closed recording has a partial record or no end marker")
    result.update(session=identity[0], nonce=identity[1], file_bytes=result["pages"] * PAGE_BYTES,
                  sha256=digest.hexdigest(), last_sequence=previous_sequence, trailing_record_bytes=len(pending))
    end = result["end"]
    result["sealed_and_contiguous"] = bool(result["closed"] and end and not result["missing_sequences"]
        and not end["dropped_records"]
        and not any(stream["missing_bytes"] for stream in result["uart_streams"].values()))
    result["sensor_errors_reported"] = None if end is None else {
        name: end[name] for name in ("imu_errors", "gnss_uart_errors", "gnss_parse_errors")}
    capture, closing = result["recording_identity"], result["close_intent"]
    result["continuation_required"] = closing["expected_next_file_id"] if closing and closing["reason"] == "rotation" else None
    result["whole_recording_in_this_file"] = bool(result["sealed_and_contiguous"] and capture and closing
        and capture["previous_file_id"] == 0 and closing["reason"] == "user_stop")
    return result


def raw_gnss_gate(summary):
    first, last = summary["gnss_raw_coverage_initial"], summary["gnss_raw_coverage_latest"]
    failures = []
    if summary["expected_bootstrap_generation"] is None:
        failures.append("no per-file raw GNSS bootstrap boundary")
    if first is None or last is None:
        failures.append("no raw GNSS coverage matching this file's bootstrap generation")
    else:
        if last["advancing_epochs"] < 2 or last["obsvm_frames"] < 2:
            failures.append("no advancing CRC-valid raw observation epochs during this file")
        if last["ephemeris_frames"] == 0 or not last["ephemeris_seen_mask"]:
            failures.append("no newly observed ephemeris frames during this file")
        if last["optional_tx_failures"] != 0:
            failures.append("raw GNSS configuration transmission failures were reported")
    if not summary["uart_streams"].get("gnss_com2", {}).get("exported_bytes"):
        failures.append("no raw COM2 bytes")
    if any(stream["missing_bytes"] for stream in summary["uart_streams"].values()):
        failures.append("UART byte-offset gaps")
    return {"passed": not failures, "failures": failures,
            "meaning": "coverage evidence only; does not qualify individual observables or guarantee a PPK solution"}


def sensor_snapshot(client):
    return {"acquisition": client.device_status(5, "AcquisitionStatus"),
            "imu": client.device_status(7, "ImuDiagnostics"),
            "transport": client.device_status(9, "GnssTransportStatus"),
            "fix": client.device_status(11, "GnssFixStatus")}


def preflight(client, seconds=10, interval=1, require_fix=False):
    if seconds <= 0 or interval <= 0:
        raise ValueError("preflight duration and interval must be positive")
    first_sensors = sensor_snapshot(client)
    records = [client.status()]
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        time.sleep(min(interval, max(0, deadline - time.monotonic())))
        records.append(client.status())
    last_sensors = sensor_snapshot(client)
    failures = []
    if any(item["state"] != "Recording" for item in records):
        failures.append("recording was not active throughout the check")
    if records[-1]["accepted_records"] <= records[0]["accepted_records"]:
        failures.append("no accepted recording progress")
    if records[-1]["written_pages"] <= records[0]["written_pages"]:
        failures.append("no durable page write progress")
    if any(item["dropped_records"] for item in records):
        failures.append("recording queue drops reported")
    if any(item["io_errors"] or item["last_error"] is not None for item in records):
        failures.append("storage errors reported")
    first_acq, last_acq = first_sensors["acquisition"], last_sensors["acquisition"]
    sample_delta = last_acq["imu_samples"] - first_acq["imu_samples"]
    elapsed_us = last_acq["updated_us"] - first_acq["updated_us"]
    if sample_delta <= 0:
        failures.append("no IMU sample progress")
    if elapsed_us <= 0:
        failures.append("acquisition timestamp did not advance")
    for key in ("imu_errors", "gnss_uart_errors", "gnss_parse_errors"):
        if last_acq[key] != first_acq[key]:
            failures.append(f"acquisition {key} changed during the check")
    if not all(item["imu"]["ready"] for item in (first_sensors, last_sensors)):
        failures.append("IMU was not ready at both endpoints")
    for key in ("driver_errors", "timing_errors", "gap_errors", "range_errors",
                "drdy_timeouts", "spi_timeouts", "numeric_range_errors"):
        if last_sensors["imu"][key] != first_sensors["imu"][key]:
            failures.append(f"IMU {key} changed during the check")
    first_ports, last_ports = first_sensors["transport"]["ports"], last_sensors["transport"]["ports"]
    if len(first_ports) != 2 or len(last_ports) != 2:
        raise RecordingError("expected two GNSS UART ports")
    if last_ports[1]["rx_bytes"] <= first_ports[1]["rx_bytes"]:
        failures.append("no GNSS COM2 input progress")
    for port, (first, last) in enumerate(zip(first_ports, last_ports), start=1):
        if last["rx_bytes"] < first["rx_bytes"]:
            failures.append(f"GNSS COM{port} byte counter moved backward")
        if last["rx_errors"] != first["rx_errors"]:
            failures.append(f"GNSS COM{port} RX errors changed during the check")
    fix = last_sensors["fix"]
    fix_valid = fix["position_valid"] and fix["velocity_valid"]
    if require_fix and not fix_valid:
        failures.append("fresh GNSS position and velocity fix required")
    return {"passed": not failures, "failures": failures, "first": records[0], "last": records[-1],
            "samples": len(records), "sensors_first": first_sensors, "sensors_last": last_sensors,
            "imu_sample_delta": sample_delta,
            "observed_imu_rate_hz": sample_delta * 1_000_000 / elapsed_us if elapsed_us > 0 else None,
            "require_fix": require_fix, "fix_valid": fix_valid,
            "meaning": "live capture progress; sensor calibration, timing qualification and raw observation quality are separate",
            "raw_gnss_validation": "download and verify --require-raw-gnss after stopping"}


def first_page(client, session):
    """Read one512-byte identity page regardless of the negotiated chunk size."""
    result = bytearray()
    while len(result) < PAGE_BYTES:
        result.extend(client.read(session, len(result), min(client.max_read_len, PAGE_BYTES - len(result))))
    return bytes(result)


def download(client, session, output, resume=False, entry=None):
    output = Path(output)
    summary_path = output.with_name(output.name + ".json")
    if output.exists() or summary_path.exists():
        raise RecordingError("destination exists, including companion metadata; choose another output path")
    entry = entry or client.operation("list", after_id=session - 1).get("Entry")
    if not entry or entry["session"] != session:
        raise RecordingError("recording session not found")
    length = entry["valid_bytes"]
    if not PAGE_BYTES <= length <= entry["file_bytes"] or length % PAGE_BYTES:
        raise RecordingError("recording has no valid complete page prefix")
    first = first_page(client, session)
    identity, _, _ = inspect_page(first, 0)
    if identity[0] != session:
        raise RecordingError("session ID disagrees with file content")
    descriptor = {**entry, "first_page_sha256": hashlib.sha256(first).hexdigest()}
    partial = output.with_name(output.name + ".part")
    sidecar = output.with_name(output.name + ".part.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    if partial.exists() or sidecar.exists():
        if not resume or not partial.exists() or not sidecar.exists():
            raise RecordingError("partial download exists; use --resume with both partial files intact")
        if json.loads(sidecar.read_text(encoding="utf-8")) != descriptor:
            raise RecordingError("card/session contents changed; choose a new destination")
        offset = partial.stat().st_size // PAGE_BYTES * PAGE_BYTES
        if offset > length:
            raise RecordingError("partial download exceeds the recording length")
        with partial.open("rb") as source:
            for index in range(offset // PAGE_BYTES):
                _, closed, _ = inspect_page(source.read(PAGE_BYTES), index, identity)
                if closed and (index + 1) * PAGE_BYTES != length:
                    raise RecordingError("premature final page in partial download")
        with partial.open("r+b") as target:
            target.truncate(offset)
    else:
        sidecar.write_text(json.dumps(descriptor, indent=2) + "\n", encoding="utf-8")
        with partial.open("xb"):
            pass
        offset = 0
    with partial.open("ab") as target:
        while offset < length:
            value = client.read(session, offset, min(client.max_read_len, length - offset))
            target.write(value)
            offset += len(value)
            if offset % (256 * 1024) == 0:
                target.flush()
                print(f"Downloaded {offset}/{length} bytes", file=sys.stderr, flush=True)
        target.flush()
        os.fsync(target.fileno())
    summary = verify_log(partial)
    if summary["session"] != session or summary["closed"] != entry["closed"]:
        raise RecordingError("download content disagrees with session metadata")
    with partial.open("rb") as source:
        if hashlib.sha256(source.read(PAGE_BYTES)).hexdigest() != descriptor["first_page_sha256"]:
            raise RecordingError("recording identity changed during download")
    partial.rename(output)
    sidecar.unlink()
    with summary_path.open("x", encoding="utf-8") as target:
        target.write(json.dumps(summary, indent=2) + "\n")
    return summary


def download_all(client, output, resume=False):
    if client.status()["state"] in ("Recording", "Stopping"):
        raise RecordingError("stop recording before download-all")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    index_path = output / "recordings.json"
    if index_path.exists():
        previous_index = json.loads(index_path.read_text(encoding="utf-8"))
        if not isinstance(previous_index, dict) or previous_index.get("format") != "aevia-download-index-v1":
            raise RecordingError("output contains unrelated recordings.json metadata")
    sessions = list(client.sessions())
    index = {"format": "aevia-download-index-v1", "sessions": []}
    for entry in sessions:
        session = entry["session"]
        destination = output / f"session-{session:08d}.aev"
        if destination.exists():
            summary = verify_log(destination)
            first = first_page(client, session)
            inspect_page(first, 0)
            with destination.open("rb") as source:
                local_first = source.read(PAGE_BYTES)
            if (summary["session"] != session or summary["file_bytes"] != entry["valid_bytes"]
                    or summary["closed"] != entry["closed"] or first != local_first):
                raise RecordingError(f"existing session {session} differs from the device; choose a new directory")
            skipped = True
        else:
            summary = download(client, session, destination, resume=resume, entry=entry)
            skipped = False
        index["sessions"].append({"session": session, "file": destination.name, "bytes": summary["file_bytes"],
                                  "sha256": summary["sha256"], "closed": summary["closed"],
                                  "recording_identity": summary["recording_identity"],
                                  "continuation_required": summary["continuation_required"], "already_verified": skipped})
    ids = {entry["session"] for entry in index["sessions"]}
    index["missing_rotated_files"] = sorted({entry["continuation_required"] for entry in index["sessions"]
                                            if entry["continuation_required"] and entry["continuation_required"] not in ids})
    previous_files = {identity[field] for entry in index["sessions"] if (identity := entry["recording_identity"])
                      for field in ("previous_file_id", "recording_id") if identity[field] and identity[field] not in ids}
    index["missing_rotated_files"] = sorted(set(index["missing_rotated_files"]) | previous_files)
    # Replace only our explicitly identified index after all source files pass.
    temporary = output / "recordings.json.tmp"
    with temporary.open("x", encoding="utf-8") as target:
        target.write(json.dumps(index, indent=2) + "\n")
        target.flush()
        os.fsync(target.fileno())
    temporary.replace(index_path)
    return index


def imu_payload(payload):
    if len(payload) != 120 or payload[0] != 1 or payload[1] & ~15 or payload[2] & ~63 or payload[3]:
        raise RecordingError("unsupported native IMU payload")
    flags = payload[1]
    row = {"flags": flags, "valid_axis_mask": payload[2],
           "scaled_valid": bool(flags & 1), "read_within_deadline": bool(flags & 2)}
    row.update(zip(("edge_ns", "read_start_ns", "read_finish_ns", "latest_edge_ns"), struct.unpack_from("<4Q", payload, 4)))
    if not flags & 4:
        row["latest_edge_ns"] = None
    if not flags & 8:
        row["read_finish_ns"] = None
    axes = ("gx", "gy", "gz", "ax", "ay", "az")
    row.update(zip((f"raw_{axis}" for axis in axes), struct.unpack_from("<6i", payload, 36)))
    row.update(zip((f"counter_{axis}" for axis in axes), payload[60:66]))
    row.update(zip((f"status_{axis}" for axis in axes), payload[66:72]))
    values = struct.unpack_from("<6d", payload, 72) if flags & 1 else [None] * 6
    row.update(zip(("gx_rps", "gy_rps", "gz_rps", "ax_mps2", "ay_mps2", "az_mps2"), values))
    return row


def uart_payload(payload):
    if len(payload) < 24 or payload[0] != 1 or payload[1] & ~1 or any(payload[2:4]) or any(payload[22:24]):
        raise RecordingError("unsupported UART payload")
    receipt_us, stream_offset, length = struct.unpack_from("<QQH", payload, 4)
    if length > 232 or len(payload) != 24 + length:
        raise RecordingError("UART byte count mismatch")
    return {"capture_clock_valid": bool(payload[1] & 1), "receipt_us": receipt_us,
            "stream_offset": stream_offset}, payload[24:]


def event_payload(kind, payload):
    if not payload or payload[0] != 1:
        raise RecordingError("unsupported sensor event payload")
    if kind == 5 and len(payload) == 24:
        pulse, ticks, sequence, errors = struct.unpack_from("<QIII", payload, 4)
        return {"pulse_ns": pulse, "raw_mcpwm_ticks": ticks, "pulse_sequence": sequence,
                "capture_errors": errors}
    if kind == 7 and len(payload) == 8:
        return {"valid": bool(payload[1]), "die_temperature_c": struct.unpack_from("<f", payload, 4)[0]
                if payload[1] else None}
    if kind == 9 and len(payload) == 104 and payload[1] == 12:
        component, asic, *serial = struct.unpack_from("<5H", payload, 2)
        registers = [{"register": payload[12 + i * 3], "value": struct.unpack_from("<H", payload, 13 + i * 3)[0]}
                     for i in range(12)]
        return {"component": component, "asic": asic, "serial": serial, "registers": registers,
                **dict(zip(("gyro_delay_ns", "accel_delay_ns", "delay_uncertainty_ns", "tick_ns"),
                           struct.unpack_from("<4I", payload, 48))),
                **dict(zip(("rad_s_per_lsb", "mps2_per_lsb", "nominal_hz"), struct.unpack_from("<3d", payload, 64))),
                **dict(zip(("read_deadline_ns", "minimum_interval_ns", "maximum_interval_ns"),
                           struct.unpack_from("<3I", payload, 88))),
                "output": payload[100], "profile_schema": payload[101], "qualified": False}
    if kind == 6 and len(payload) == 32 and payload[1] == 1:
        return {"source": "imu", "code": payload[2], "error_kind": payload[3],
                "edge_ns": struct.unpack_from("<Q", payload, 4)[0] if payload[19] else None,
                "register": None if payload[12] == 255 else payload[12], "sensor_status": payload[13],
                "expected": struct.unpack_from("<H", payload, 14)[0], "actual": struct.unpack_from("<H", payload, 16)[0],
                "counter": payload[18], "sensor_payload": struct.unpack_from("<I", payload, 20)[0]}
    if kind == 6 and len(payload) == 8 and payload[1] == 2:
        return {"source": "gnss", "code": payload[2], "detail": payload[3], "port": payload[4]}
    if kind == 11 and len(payload) == 40:
        result = {"ephemeris_seen_mask": payload[1],
                  "bootstrap_generation": struct.unpack_from("<I", payload, 20)[0],
                  **dict(zip(("obsvm_frames", "advancing_epochs", "ephemeris_frames", "optional_tx_failures"),
                             struct.unpack_from("<4I", payload, 4)))}
        result["last_obsvm_receipt_us"] = struct.unpack_from("<Q", payload, 24)[0] if payload[2] & 1 else None
        result["last_gpst_epoch_ms"] = struct.unpack_from("<Q", payload, 32)[0] if payload[2] & 2 else None
        return result
    if kind in (10, 255):
        return health_payload(payload)
    if kind == 14 and len(payload) == 21:
        recording, file, previous, base = struct.unpack_from("<IIIQ", payload, 1)
        return {"recording_id": recording, "file_id": file, "previous_file_id": previous,
                "source_sequence_base": base}
    if kind == 15 and len(payload) == 6 and payload[1] in (0, 1):
        following = struct.unpack_from("<I", payload, 2)[0]
        if (payload[1] == 0 and following != 0) or (payload[1] == 1 and following == 0):
            raise RecordingError("invalid close intent next-file ID")
        return {"reason": "user_stop" if payload[1] == 0 else "rotation", "expected_next_file_id": following}
    raise RecordingError(f"unsupported payload for record kind {kind}")


def export_log(path, output):
    summary = verify_log(path)
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise RecordingError("export directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    handles = []
    try:
        def open_output(name, mode):
            handle = (output / name).open(mode, **({} if "b" in mode else {"encoding": "utf-8", "newline": ""}))
            handles.append(handle)
            return handle
        gnss = {kind: open_output(name, "wb") for kind, name in
                [(3, "gnss-com2.bin"), (4, "gnss-com1.bin"), (8, "gnss-com1-tx-attempted.bin")]}
        chunks = open_output("gnss-chunks.jsonl", "w")
        timing = open_output("timing.jsonl", "w")
        metadata = open_output("metadata.jsonl", "w")
        imu = open_output("imu.csv", "w")
        imu_writer = None

        def visitor(record):
            nonlocal imu_writer
            kind, payload = record["kind"], record["payload"]
            common = {key: record[key] for key in ("kind", "sequence", "at_ns")}
            if kind in gnss:
                transport, data = uart_payload(payload)
                offset = gnss[kind].tell()
                gnss[kind].write(data)
                chunks.write(json.dumps({**common, **transport, "port": 2 if kind == 3 else 1,
                                         "direction": "tx_attempted" if kind == 8 else "rx",
                                         "offset": offset, "bytes": len(data)}) + "\n")
            elif kind == 2:
                row = {**common, **imu_payload(payload)}
                if imu_writer is None:
                    imu_writer = csv.DictWriter(imu, fieldnames=list(row))
                    imu_writer.writeheader()
                imu_writer.writerow(row)
            else:
                item = {**common, "payload_hex": payload.hex()}
                if kind in (5, 6, 7, 9, 10, 11, 14, 15, 255):
                    item["decoded"] = event_payload(kind, payload)
                elif kind == 13:
                    item["expected_bootstrap_generation"] = struct.unpack("<I", payload)[0]
                if kind == 1:
                    try:
                        item["metadata"] = json.loads(payload)
                    except (UnicodeError, ValueError):
                        item["metadata_error"] = "invalid JSON; original payload retained"
                    metadata.write(json.dumps(item) + "\n")
                else:
                    timing.write(json.dumps(item) + "\n")
        repeated = verify_log(path, visitor)
        if repeated["sha256"] != summary["sha256"]:
            raise RecordingError("source recording changed during export")
    finally:
        for handle in handles:
            handle.close()
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="AEVIADEVBOARD")
    parser.add_argument("--port", help="explicit USB serial port, e.g. COM4")
    sibling_decoder = Path(__file__).resolve().with_name("aevia-board-decode.exe")
    default_decoder = sibling_decoder if sibling_decoder.exists() else Path(__file__).resolve().parents[2] / \
        "target/board-tools/x86_64-pc-windows-msvc/debug/aevia-board-decode.exe"
    parser.add_argument("--decoder", type=Path, default=default_decoder)
    parser.add_argument("--timeout", type=float, default=30, help="operation polling deadline in seconds")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("status", "start", "stop", "list"):
        commands.add_parser(name)
    gate = commands.add_parser("preflight")
    gate.add_argument("--seconds", type=float, default=10)
    gate.add_argument("--interval", type=float, default=1)
    gate.add_argument("--require-fix", action="store_true",
                      help="also require fresh receiver position and velocity at the final check")
    get = commands.add_parser("download")
    get.add_argument("session", type=int)
    get.add_argument("--output", type=Path, required=True)
    get.add_argument("--resume", action="store_true")
    get_all = commands.add_parser("download-all")
    get_all.add_argument("--output", type=Path, required=True)
    get_all.add_argument("--resume", action="store_true")
    verify = commands.add_parser("verify")
    verify.add_argument("input", type=Path)
    verify.add_argument("--require-raw-gnss", action="store_true")
    export = commands.add_parser("export")
    export.add_argument("input", type=Path)
    export.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.command == "download" and not 1 <= args.session <= 0xFFFFFFFF:
        parser.error("session must be in 1..4294967295")
    client = None
    try:
        if args.command == "verify":
            result = verify_log(args.input)
            if args.require_raw_gnss:
                result["raw_gnss_gate"] = raw_gnss_gate(result)
        elif args.command == "export":
            result = export_log(args.input, args.output)
        else:
            client = Client(args)
            if args.command == "status":
                result = client.status()
            elif args.command == "list":
                result = list(client.sessions())
            elif args.command == "download":
                result = download(client, args.session, args.output, args.resume)
            elif args.command == "download-all":
                result = download_all(client, args.output, args.resume)
            elif args.command == "preflight":
                result = preflight(client, args.seconds, args.interval, args.require_fix)
            else:
                expected = {"start": "Started", "stop": "Stopped"}[args.command]
                reply = client.operation(args.command)
                if expected not in reply:
                    raise RecordingError(f"unexpected {args.command} response: {reply}")
                result = reply[expected]
        print(json.dumps(result, indent=2))
        if (args.command == "preflight" and not result["passed"]) or (
                args.command == "verify" and args.require_raw_gnss and not result["raw_gnss_gate"]["passed"]) or (
                args.command == "download-all" and result["missing_rotated_files"]):
            return 1
        return 0
    except (RecordingError, OSError, ValueError) as failure:
        print(f"recording: {failure}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("recording: interrupted; partial downloads can be resumed", file=sys.stderr)
        return 130
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
