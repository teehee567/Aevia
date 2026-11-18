import json
import base64
import io
import struct
import unittest
from unittest.mock import patch
import upload
import telemetry


def reply_frame(value):
    data = json.dumps(value).encode()
    return upload.frame(b"\x03" + upload.uint(len(data)) + data)


class BufferedSerial:
    def __init__(self, chunks, response_at=0):
        self.chunks = list(chunks)
        self.response_at = response_at
        self.now = 0.0
        self.read_sizes = []
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks and self.now >= self.response_at else 0

    def write(self, value):
        self.writes.append(value)
        return len(value)

    def read(self, size):
        self.read_sizes.append(size)
        if not self.in_waiting:
            self.now += 0.1
            return b""
        result = self.chunks[0][:size]
        self.chunks[0] = self.chunks[0][size:]
        if not self.chunks[0]:
            self.chunks.pop(0)
        return result


class ApplicationProtocolTests(unittest.TestCase):
    def test_base64_mixed_48_records_preserve_exact_payload_values_across_packets(self):
        captured = 9_007_199_254_741_193
        values = [-0.0, 1.25, -9.81, 1e-20, -.125, .375, .5, -.25, .75]
        imu = struct.pack("<QIQQ9dB", captured, 1_375_099, captured - 1_500_000, 1_200_000, *values, 1)
        gnss = struct.pack("<QII9dIBBBQQI", 123, 2444, 999, *values, 75, 19, 1, 1, captured, 500_017, 42)
        pair = struct.pack("<BH", 23, len(imu)) + imu + struct.pack("<BH", 24, len(gnss)) + gnss
        raw = pair * 24
        expected = telemetry.decode_record_bytes(raw)
        encoded = base64.b64encode(raw).decode("ascii")
        frame = reply_frame({"ok": True, "compact": True, "binary": True, "records_b64": encoded})
        link = BufferedSerial([frame[i:i + 512] for i in range(0, len(frame), 512)])
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            reply = telemetry.request(link, "drain_base64")
        self.assertEqual(reply["samples"], expected)
        self.assertEqual(len(expected), 48)
        self.assertEqual(expected[0]["captured_ns"], captured)
        self.assertEqual(struct.pack("<9d", *(expected[0]["accel"] + expected[0]["gyro"] + expected[0]["aligned_gyro"])), struct.pack("<9d", *values))
        # Single records cover each padding length; a mixed pair is unpadded.
        for blob in (raw[:104], raw[104:222], raw[:222], b""):
            self.assertEqual(telemetry.decode_base64_records(base64.b64encode(blob).decode("ascii")), telemetry.decode_record_bytes(blob))

    def test_base64_is_canonical_bounded_and_unambiguous(self):
        for bad in (None, 123, "a", "AAAA====", "AA=A", "AA==\n", "AB==", "____", "A" * 8196):
            with self.assertRaises(ValueError):
                telemetry.decode_base64_records(bad)
        for fields in ({}, {"records_hex": "", "records_b64": ""}, {"records_b64": "", "samples": []}):
            with self.assertRaises(ValueError):
                telemetry.decode_reply(reply_frame({"ok": True, "binary": True, **fields}))

    def test_capture_prefers_base64_over_hex_for_bounded_stop_tail(self):
        replies = [
            {"base64_poll": True, "binary_poll": True, "compact_poll": True, "uptime_us": 100, "samples": []},
            {"uptime_us": 200, "stopped_at_us": 200, "pending": 1, "dropped": 0, "gnss": {}, "imu": {}},
            {"samples": [{"kind": "imu", "t_us": 150}], "pending": 20},
        ]
        with patch.object(telemetry, "request", side_effect=replies) as request:
            summary = telemetry.capture(None, 0)
        self.assertEqual([c.args[1] for c in request.call_args_list], ["start", "stop", "drain_base64"])
        self.assertEqual(summary["counts"]["imu"], 1)

    def test_binary_batch_preserves_nanoseconds_float_bits_and_mapping(self):
        captured = 9_007_199_254_741_193
        values = [-0.0, 1.25, -9.81, 1e-20, -.125, .375, .5, -.25, .75]
        imu = struct.pack("<QIQQ9dB", captured, 1_375_099, captured - 1_500_000, 1_200_000, *values, 1)
        gnss = struct.pack("<QII9dIBBBQQI", 123, 2444, 999, *values, 75, 19, 1, 1,
                           captured, 500_017, 42)
        records = (struct.pack("<BH", 23, len(imu)) + imu + struct.pack("<BH", 24, len(gnss)) + gnss).hex()
        reply = telemetry.decode_reply(reply_frame({"ok": True, "binary": True, "records_hex": records}))
        a, b = reply["samples"]
        self.assertEqual(a["captured_ns"], captured)
        self.assertEqual(a["interval_ns"], 1_375_099)
        self.assertEqual(a["t_us"], captured // 1000)
        self.assertEqual(struct.pack("<9d", *(a["accel"] + a["gyro"] + a["aligned_gyro"])), struct.pack("<9d", *values))
        self.assertEqual(b["mapped_ns"], captured)
        self.assertEqual(b["mapping_generation"], 42)
        for invalid in ["1", "zz", "17", "170100", "180000", "17 0000"]:
            with self.assertRaises(ValueError):
                telemetry.decode_records(invalid)

    def test_capture_prefers_binary_capability_for_stop_tail(self):
        replies = [
            {"binary_poll": True, "compact_poll": True, "uptime_us": 100, "samples": []},
            {"uptime_us": 200, "stopped_at_us": 200, "pending": 1, "dropped": 0, "gnss": {}, "imu": {}},
            {"samples": [{"kind": "imu", "t_us": 150}], "pending": 0},
        ]
        with patch.object(telemetry, "request", side_effect=replies) as request:
            summary = telemetry.capture(None, 0)
        self.assertEqual([c.args[1] for c in request.call_args_list], ["start", "stop", "drain_binary"])
        self.assertEqual(summary["counts"]["imu"], 1)

    def test_capture_uses_advertised_compact_drain_for_the_bounded_tail(self):
        replies = [
            {"compact_poll": True, "uptime_us": 100, "started_at_us": 100, "samples": []},
            {"uptime_us": 500, "stopped_at_us": 200, "pending": 1, "dropped": 0, "gnss": {}, "imu": {}},
            {"compact": True, "pending": 9, "samples": [{"kind": "imu", "t_us": 150}]},
        ]
        with patch.object(telemetry, "request", side_effect=replies) as request:
            summary = telemetry.capture(None, 0)
        self.assertEqual([call.args[1] for call in request.call_args_list], ["start", "stop", "drain"])
        self.assertEqual(summary["counts"], {"imu": 1, "gnss": 0})
        self.assertEqual(summary["elapsed_s"], .0001)

    def test_capture_drains_stop_tail_and_excludes_flush_preview(self):
        now = [0.0]
        operations = []
        replies = iter([
            ("start", 0, {"started_at_us": 1_000_000, "uptime_us": 1_100_000, "samples": []}),
            ("poll", 1.1, {"pending": 4, "samples": [
                {"kind": "imu", "t_us": 1_200_000},
                {"kind": "gnss", "t_us": 1_300_000}]}),
            ("stop", 10.5, {"stopped_at_us": 2_100_000, "uptime_us": 12_600_000,
                            "pending": 4, "dropped": 7, "gnss": {}, "imu": {}}),
            ("poll", 0, {"pending": 6, "samples": [
                {"kind": "imu", "t_us": 2_000_000},
                {"kind": "imu", "t_us": 2_200_000}]}),
            ("poll", 0, {"pending": 7, "samples": [
                {"kind": "gnss", "t_us": 2_300_000},
                {"kind": "imu", "t_us": 2_400_000},
                {"kind": "imu", "t_us": 2_500_000}]}),
        ])

        def request(_link, operation):
            expected, duration, reply = next(replies)
            self.assertEqual(operation, expected)
            operations.append(operation)
            now[0] += duration
            return reply

        stream = io.StringIO()
        with patch.object(telemetry, "request", request), \
                patch.object(telemetry.time, "monotonic", lambda: now[0]):
            summary = telemetry.capture(None, 1, stream)
        self.assertEqual(operations, ["start", "poll", "stop", "poll", "poll"])
        self.assertEqual(summary["counts"], {"gnss": 1, "imu": 2})
        self.assertEqual(summary["elapsed_s"], 1.1)
        self.assertEqual(summary["dropped"], 7)
        saved = [json.loads(line) for line in stream.getvalue().splitlines()]
        self.assertEqual(saved[2]["stopped_at_us"], 2_100_000)
        samples = [sample for reply in saved for sample in reply.get("samples", [])]
        self.assertEqual(len(samples), 3)
        self.assertTrue(all(sample["t_us"] <= 2_100_000 for sample in samples))

    def test_capture_requests_stop_after_poll_failure(self):
        operations = []

        def request(_link, operation):
            operations.append(operation)
            if operation == "poll":
                raise TimeoutError("disconnected")
            return {"uptime_us": 1, "samples": []}

        with patch.object(telemetry, "request", request):
            with self.assertRaises(TimeoutError):
                telemetry.capture(None, 1)
        self.assertEqual(operations, ["start", "poll", "stop"])

    def test_cobs_all_byte_values_and_packet_boundaries(self):
        for payload in (bytes(range(256)), b"x" * 512, b"\0" * 256, b"x" * 12288):
            self.assertEqual(telemetry.decode_frame(upload.frame(payload)), payload)

    def test_multi_packet_reply(self):
        data = json.dumps({"ok": True, "samples": [{"kind": "imu", "t_us": i} for i in range(100)]}).encode()
        actual = telemetry.decode_reply(upload.frame(b"\x03" + upload.uint(len(data)) + data))
        self.assertEqual(len(actual["samples"]), 100)

    def test_invalid_replies_fail_closed(self):
        for payload in (b"\x02", b"\x03\x01{}", b"\x03\xff", b"\x03\x02[]"):
            with self.assertRaises((ValueError, AttributeError)):
                telemetry.decode_reply(upload.frame(payload))

    def test_large_response_uses_buffered_reads_across_usb_packets(self):
        value = {"ok": True, "samples": [{"kind": "imu", "t_us": i} for i in range(250)]}
        frame = reply_frame(value)
        chunks = [frame[i:i + 512] for i in range(0, len(frame), 512)]
        link = BufferedSerial(chunks)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            self.assertEqual(telemetry.request(link, "poll"), value)
        self.assertEqual(len(link.read_sizes), len(chunks))
        self.assertGreater(max(link.read_sizes), 1)

    def test_reply_delimiter_may_arrive_in_separate_read(self):
        value = {"ok": True, "samples": []}
        frame = reply_frame(value)
        link = BufferedSerial([frame[:2], frame[2:-1], frame[-1:]])
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            self.assertEqual(telemetry.request(link, "poll"), value)

    def test_stop_accepts_response_after_ten_second_flush(self):
        value = {"ok": True, "stopped_at_us": 123, "uptime_us": 10_500_123}
        link = BufferedSerial([reply_frame(value)], response_at=10.5)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            self.assertEqual(telemetry.request(link, "stop"), value)
        self.assertLess(link.now, 12)
        link = BufferedSerial([reply_frame(value)], response_at=13)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            with self.assertRaises(TimeoutError):
                telemetry.request(link, "stop")

    def test_start_allows_sd_preparation_and_still_has_a_bound(self):
        value = {"ok": True, "started_at_us": 10_000_000, "uptime_us": 10_500_000}
        link = BufferedSerial([reply_frame(value)], response_at=10.5)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            self.assertEqual(telemetry.request(link, "start"), value)
        link = BufferedSerial([], response_at=13)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            with self.assertRaises(TimeoutError):
                telemetry.request(link, "start")
        self.assertLess(link.now, 12.2)

    def test_status_still_times_out_and_bad_frames_remain_bounded(self):
        link = BufferedSerial([], response_at=13)
        with patch.object(telemetry.time, "monotonic", lambda: link.now):
            with self.assertRaises(TimeoutError):
                telemetry.request(link, "status")
        self.assertLess(link.now, 6)
        for chunks in ([b"x" * 13000], [reply_frame({"ok": True}) + b"extra"]):
            link = BufferedSerial(chunks)
            with patch.object(telemetry.time, "monotonic", lambda: link.now):
                with self.assertRaises(ValueError):
                    telemetry.request(link, "poll")
