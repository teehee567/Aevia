# /// script
# dependencies = ["pyserial==3.5"]
# ///
"""Inspect or capture V2 Mini application telemetry without changing firmware."""
import argparse
import base64
import binascii
import json
import struct
import time
from pathlib import Path
import upload


def decode_frame(frame):
    if not frame or frame[-1] != 0:
        raise ValueError("Missing COBS delimiter")
    frame = frame[:-1]
    out = bytearray()
    cursor = 0
    while cursor < len(frame):
        code = frame[cursor]
        cursor += 1
        end = cursor + code - 1
        if code == 0 or end > len(frame):
            raise ValueError("Invalid COBS frame")
        out.extend(frame[cursor:end])
        cursor = end
        if code != 255 and cursor < len(frame):
            out.append(0)
    return bytes(out)


def decode_reply(frame):
    payload = decode_frame(frame)
    if not payload or payload[0] != 3:
        raise ValueError(f"Application request rejected: {payload.hex()}")
    length = 0
    cursor = 1
    for shift in range(0, 35, 7):
        if cursor >= len(payload):
            raise ValueError("Truncated response length")
        byte = payload[cursor]
        cursor += 1
        length |= (byte & 127) << shift
        if not byte & 128:
            break
    else:
        raise ValueError("Oversized response length")
    if len(payload) - cursor != length:
        raise ValueError("Response length mismatch")
    reply = json.loads(payload[cursor:])
    if not reply.get("ok"):
        raise ValueError(reply.get("error", "Application request failed"))
    if reply.get("binary"):
        if ("records_hex" in reply) == ("records_b64" in reply) or "samples" in reply:
            raise ValueError("Ambiguous binary record representation")
        reply["samples"] = (decode_base64_records(reply["records_b64"]) if "records_b64" in reply
                            else decode_records(reply["records_hex"]))
    return reply


def decode_records(value):
    """Decode lossless SD kind23/24 payloads from bounded binary USB batches."""
    if not isinstance(value, str) or len(value) > 8192 or len(value) % 2:
        raise ValueError("Invalid binary record hex")
    data = bytes.fromhex(value)
    if len(data) * 2 != len(value):
        raise ValueError("Noncanonical binary record hex")
    return decode_record_bytes(data)


def decode_base64_records(value):
    if not isinstance(value, str) or len(value) > 8192 or len(value) % 4:
        raise ValueError("Invalid binary record base64")
    try:
        data = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as error:
        raise ValueError("Invalid binary record base64") from error
    if base64.b64encode(data).decode("ascii") != value:
        raise ValueError("Noncanonical binary record base64")
    return decode_record_bytes(data)


def decode_record_bytes(data):
    samples = []
    cursor = 0
    while cursor < len(data):
        if len(data) - cursor < 3:
            raise ValueError("Truncated binary record header")
        kind, length = struct.unpack_from("<BH", data, cursor)
        cursor += 3
        payload = data[cursor:cursor + length]
        cursor += length
        if len(payload) != length:
            raise ValueError("Truncated binary record")
        if kind == 23 and length == 101:
            captured, interval, effective, sigma = struct.unpack_from("<QIQQ", payload)
            values = struct.unpack_from("<9d", payload, 28)
            if payload[100] not in (0, 1):
                raise ValueError("Invalid IMU validity")
            sample = {"kind": "imu", "t_us": captured // 1000, "dt_us": interval // 1000,
                      "captured_ns": captured, "interval_ns": interval,
                      "effective_end_ns": effective, "timing_uncertainty_ns": sigma,
                      "accel": list(values[:3]), "gyro": list(values[3:6]),
                      "aligned_gyro": list(values[6:]), "valid": bool(payload[100])}
        elif kind == 24 and length == 115:
            received, week, tow = struct.unpack_from("<QII", payload)
            values = struct.unpack_from("<9d", payload, 16)
            latency, satellites, valid, mapped = struct.unpack_from("<IBBB", payload, 88)
            epoch, sigma, generation = struct.unpack_from("<QQI", payload, 95)
            if valid not in (0, 1) or mapped not in (0, 1) or (not mapped and any(payload[95:])):
                raise ValueError("Invalid GNSS mapping/validity")
            sample = dict(zip(("lat", "lon", "alt_m", "speed_mps", "heading_deg", "vertical_mps",
                               "hacc_m", "vacc_m", "sacc_mps"), values))
            sample.update(kind="gnss", t_us=received, gps_week=week, gps_tow_ms=tow,
                          latency_ms=latency, satellites=satellites, valid=bool(valid),
                          mapped_ns=epoch if mapped else None,
                          mapping_uncertainty_ns=sigma if mapped else None,
                          mapping_generation=generation if mapped else None)
        else:
            raise ValueError(f"Unsupported binary record {kind}/{length}")
        samples.append(sample)
        if len(samples) > 48:
            raise ValueError("Oversized binary record batch")
    return samples


def request(link, operation):
    data = json.dumps({"op": operation}, separators=(",", ":")).encode()
    link.write(upload.frame(b"\x03" + upload.uint(len(data)) + data))
    frame = bytearray()
    # A stop waits for the firmware's ten-second SD drain/flush barrier.
    deadline = time.monotonic() + (12 if operation in ("start", "stop") else 5)
    while time.monotonic() < deadline:
        # Block only for the first byte, then drain everything already buffered.
        # Serial.read(4096) with a 100 ms timeout would otherwise wait on every
        # short reply; read(1) for every byte severely limits capture throughput.
        chunk = link.read(max(1, min(link.in_waiting, 4096)))
        if chunk:
            frame.extend(chunk)
            if len(frame) > 12500:
                raise ValueError("Oversized application response")
            delimiter = frame.find(0)
            if delimiter >= 0:
                # The protocol permits only one request/reply in flight. Extra
                # data cannot safely be associated with a future operation.
                if delimiter != len(frame) - 1:
                    raise ValueError("Unexpected data after application response")
                return decode_reply(frame)
    raise TimeoutError("No application response")


def capture(link, seconds, stream=None):
    """Capture through the stop boundary, including samples queued before it."""
    recording = False
    counts = {"gnss": 0, "imu": 0}

    def save(reply, samples=None):
        if samples is not None:
            reply = {**reply, "samples": samples}
            for sample in samples:
                counts[sample["kind"]] += 1
        if stream:
            stream.write(json.dumps(reply, separators=(",", ":")) + "\n")

    try:
        start = request(link, "start")
        recording = True
        operation = ("drain_base64" if start.get("base64_poll") else "drain_binary" if start.get("binary_poll")
                     else "drain" if start.get("compact_poll") else "poll")
        save(start)
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            reply = request(link, operation)
            save(reply, reply["samples"])
            if reply["pending"] == 0:
                time.sleep(.02)
        elapsed = time.monotonic() - started
        final = request(link, "stop")
        recording = False
        save(final)
        boundary = final.get("stopped_at_us")
        if boundary is not None:
            start_boundary = start.get("started_at_us")
            elapsed = max(0, boundary - (start["uptime_us"] if start_boundary is None else start_boundary)) / 1_000_000
        else:
            # Older firmware lacks the exact transition timestamp. Its response
            # time is the only available filter, but retain host capture duration
            # above rather than counting a possible ten-second SD flush.
            boundary = final["uptime_us"]
        remaining = final["pending"]
        while remaining > 0:
            reply = request(link, operation)
            # Preview acquisition continues indefinitely. Drain only the bounded
            # queue observed at stop, and keep only samples inside the recording.
            queued = reply["samples"][:remaining]
            if not queued:
                break
            remaining -= len(queued)
            save(reply, [sample for sample in queued if sample["t_us"] <= boundary])
        return {"elapsed_s": elapsed, "counts": counts, "dropped": final["dropped"],
                "stopped_at_us": final.get("stopped_at_us"),
                "gnss": final["gnss"], "imu": final["imu"]}
    finally:
        if recording:
            try:
                request(link, "stop")
            except (OSError, TimeoutError, ValueError):
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=0, help="Capture duration; zero only reads status")
    parser.add_argument("--output", type=Path, help="Save bounded capture responses as JSONL")
    args = parser.parse_args()
    link, _ = upload.connect(upload.APP_PID)
    stream = None
    try:
        print(json.dumps(request(link, "status"), indent=2))
        if args.seconds <= 0:
            return
        if args.output:
            stream = args.output.open("x", encoding="utf-8")
        print(json.dumps(capture(link, args.seconds, stream), indent=2))
    finally:
        if stream:
            stream.close()
        upload.disconnect(link)


if __name__ == "__main__":
    main()
