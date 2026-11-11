#!/usr/bin/env python3
"""Compare negotiated download sizes using the actual client and Rust decoder.

No serial port is opened. Results exclude board/USB/SD latency and are NOT a
device throughput claim. --call-cost-us optionally models driver-call overhead;
the default measures Python/client/pipe/decoder/polling plus mock reply encoding.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
from unittest.mock import patch
import zlib

from recording import Client, MAX_RESPONSE_FRAME, READ_BYTES, cobs_encode, varint


def decode_request(frame):
    """Decode only the small bounded mock request, independent of real replies."""
    if not frame or frame[-1] != 0:
        raise ValueError("unterminated mock request")
    data = bytearray()
    at = 0
    while at < len(frame) - 1:
        count = frame[at]
        if count == 0 or at + count > len(frame) - 1:
            raise ValueError("malformed mock request")
        data.extend(frame[at + 1:at + count])
        at += count
        if count != 255 and at < len(frame) - 1:
            data.append(0)
    values = []
    value = shift = 0
    for byte in data:
        value |= (byte & 127) << shift
        if byte < 128:
            values.append(value)
            value = shift = 0
        else:
            shift += 7
            if shift >= 70:
                raise ValueError("oversized mock integer")
    if shift:
        raise ValueError("truncated mock integer")
    return values


def discovery_frame(protocol):
    fields = [protocol["protocol_version"], protocol["schema_version"], *protocol["firmware_version"],
              int(protocol["recovery"]), protocol["max_request_frame"], protocol["max_response_frame"],
              protocol["system_commands"], protocol["device_commands"], protocol["capabilities"],
              protocol["fitted_accessories"], protocol["snapshot_interval_us"],
              protocol["navigation_max_age_us"], protocol["publication_max_age_us"]]
    return cobs_encode(b"\0\x05" + b"".join(varint(value) for value in fields))


class MockSerial:
    def __init__(self, protocol, chunk_bytes, call_cost_us):
        self.timeout = 3
        self.buffer = bytearray()
        self.read_calls = 0
        self.waiting_calls = 0
        self.write_calls = 0
        self.flush_calls = 0
        self.call_cost = call_cost_us / 1_000_000
        self.protocol = protocol
        self.chunk_bytes = chunk_bytes
        self.payload = bytes(index % 256 for index in range(chunk_bytes))
        self.crc = zlib.crc32(self.payload)
        self.next_request = 1
        self.pending = None
        self.closed = False

    def _cost(self):
        if self.call_cost:
            until = time.perf_counter() + self.call_cost
            while time.perf_counter() < until:
                pass

    @property
    def in_waiting(self):
        self.waiting_calls += 1
        self._cost()
        return len(self.buffer)

    def read(self, count):
        self.read_calls += 1
        self._cost()
        value = bytes(self.buffer[:count])
        del self.buffer[:count]
        return value

    def write(self, frame):
        self.write_calls += 1
        self._cost()
        if self.buffer:
            raise RuntimeError("unconsumed mock reply")
        values = decode_request(frame)
        if values == [0, 8]:
            self.buffer.extend(discovery_frame(self.protocol))
        elif values == [2, 0, 0]:
            # Ready, exFAT,128GiB; no free-space estimate, active session or error.
            status = [self.next_request, 1, 2, 128 * 1024**3, 0, 0, 0, 0, 0, 0, 0, 0,
                      self.chunk_bytes, 1]
            self.buffer.extend(cobs_encode(b"\x02\0\x01" + b"".join(varint(v) for v in status)))
        elif len(values) == 6 and values[:1] == [2] and values[2] == 4:
            _, request, _, session, offset, length = values
            if request != self.next_request or session != 7 or length != self.chunk_bytes or self.pending:
                raise RuntimeError("client did not obey negotiated read/token limits")
            self.next_request += 1
            self.pending = (request, session, offset)
            self.buffer.extend(cobs_encode(b"\x02" + varint(request) + b"\0"))
        elif len(values) == 3 and values[0] == 2 and values[2] == 5:
            if self.pending is None or values[1] != self.pending[0]:
                raise RuntimeError("client polled a different operation token")
            request, session, offset = self.pending
            self.pending = None
            self.buffer.extend(cobs_encode(b"\x02" + varint(request) + b"\x05" + varint(session)
                                           + varint(offset) + varint(self.chunk_bytes) + self.payload
                                           + varint(self.crc)))
        else:
            raise RuntimeError(f"unsupported mock request: {values}")
        return len(frame)

    def flush(self):
        self.flush_calls += 1
        self._cost()

    def reset_input_buffer(self):
        self.buffer.clear()

    def close(self):
        self.closed = True


def benchmark(decoder, chunks, chunk_bytes, call_cost_us, protocol):
    protocol = dict(protocol)
    # Exercise the actual old discovery limit as well as the larger new one.
    protocol["max_response_frame"] = 512 if chunk_bytes <= 256 else MAX_RESPONSE_FRAME
    device = MockSerial(protocol, chunk_bytes, call_cost_us)
    args = SimpleNamespace(port="MOCK_ONLY_NO_PORT", decoder=decoder, timeout=3)
    # Only Serial construction is replaced. Client discovery, bounded reader,
    # polling, token validation, persistent decoder and CRC checks all execute.
    with patch("serial.Serial", return_value=device):
        client = Client(args)

    try:
        if client.max_read_len != chunk_bytes or client.frame_reader.max_bytes != protocol["max_response_frame"]:
            raise RuntimeError("client did not negotiate mocked capabilities")
        device.read_calls = device.waiting_calls = 0
        device.write_calls = device.flush_calls = 0
        start = time.perf_counter()
        for index in range(chunks):
            if client.read(7, index * chunk_bytes, client.max_read_len) != device.payload:
                raise RuntimeError("downloaded data mismatch")
        elapsed = time.perf_counter() - start
    finally:
        client.close()
    return {"chunk_bytes": chunk_bytes, "negotiated_max_response_frame": protocol["max_response_frame"],
            "chunks": chunks, "bytes": chunks * chunk_bytes, "elapsed_seconds": elapsed,
            "bytes_per_second": chunks * chunk_bytes / elapsed, "serial_read_calls": device.read_calls,
            "serial_waiting_calls": device.waiting_calls, "serial_write_calls": device.write_calls,
            "serial_flush_calls": device.flush_calls}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--chunks", type=int, default=256)
    parser.add_argument("--chunk-sizes", type=int, nargs="+", default=[256, 4096])
    parser.add_argument("--call-cost-us", type=float, default=0)
    args = parser.parse_args()
    if not 1 <= args.chunks <= 0xFFFFFFFF or not 0 <= args.call_cost_us <= 1000:
        parser.error("chunks must be positive and call-cost-us must be in 0..1000")
    if any(not 1 <= size <= READ_BYTES for size in args.chunk_sizes):
        parser.error(f"chunk-sizes must be in 1..{READ_BYTES}")
    protocol = json.loads(subprocess.check_output([str(args.decoder), "--describe"], text=True))
    results = [benchmark(args.decoder, args.chunks, size, args.call_cost_us, protocol)
               for size in args.chunk_sizes]
    result = {"kind": "host_mock_only", "excludes": ["USB latency", "real serial driver costs", "SD reads", "firmware scheduling"],
              "includes": ["actual Client discovery/negotiation/read", "mock reply encoding", "persistent Rust decoder", "chunk CRC validation"],
              "decoder_sha256": hashlib.sha256(args.decoder.read_bytes()).hexdigest(),
              "decoder_build_id": subprocess.check_output([str(args.decoder), "--build-id"], text=True).strip(),
              "simulated_driver_call_us": args.call_cost_us, "results": results,
              "last_vs_first_speedup": results[-1]["bytes_per_second"] / results[0]["bytes_per_second"]}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
