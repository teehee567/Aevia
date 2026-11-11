import unittest
from unittest.mock import patch

from usb_frame import FrameError, FrameReader


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Serial:
    """Timed fragments; a blocking read returns at the next arrival or timeout."""
    def __init__(self, clock, fragments=(), short_read=None):
        self.clock = clock
        self.fragments = list(fragments)
        self.buffer = bytearray()
        self.timeout = 3
        self.requests = []
        self.short_read = short_read

    @property
    def in_waiting(self):
        while self.fragments and self.fragments[0][0] <= self.clock.now:
            _, value = self.fragments.pop(0)
            self.buffer.extend(value)
        return len(self.buffer)

    def read(self, count):
        self.requests.append(count)
        if not self.in_waiting:
            until = self.clock.now + self.timeout
            self.clock.now = min(until, self.fragments[0][0]) if self.fragments else until
        if self.in_waiting:
            take = min(count, len(self.buffer), self.short_read or count)
            result = bytes(self.buffer[:take])
            del self.buffer[:take]
            return result
        return b""


class FrameReaderTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        patcher = patch("usb_frame.time.monotonic", self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_available_response_is_one_bulk_read(self):
        response = b"\xff" * 300 + b"\0"
        device = Serial(self.clock, [(0, response)])
        self.assertEqual(FrameReader(device).read_frame(), response)
        self.assertEqual(device.requests, [301])

    def test_usb_packet_fragments_do_not_add_a_timeout_per_packet(self):
        response = b"\x02" * 300 + b"\0"
        device = Serial(self.clock, [(i / 1000, response[i * 64:(i + 1) * 64]) for i in range(5)])
        self.assertEqual(FrameReader(device).read_frame(), response)
        self.assertLessEqual(len(device.requests), 9)
        self.assertAlmostEqual(self.clock.now, 0.004)

    def test_short_bulk_reads_preserve_all_bytes(self):
        response = b"\x55" * 100 + b"\0"
        device = Serial(self.clock, [(0, response)], short_read=7)
        self.assertEqual(FrameReader(device).read_frame(), response)

    def test_maximum_length_includes_delimiter(self):
        response = b"\xff" * 511 + b"\0"
        self.assertEqual(FrameReader(Serial(self.clock, [(0, response)])).read_frame(), response)
        for bad in (b"\xff" * 512, b"\xff" * 512 + b"\0"):
            with self.subTest(length=len(bad)), self.assertRaisesRegex(FrameError, "oversized"):
                FrameReader(Serial(self.clock, [(0, bad)])).read_frame()

    def test_trailing_bytes_and_multiple_responses_are_rejected(self):
        for bad in (b"\x01\0\x02", b"\x01\0\x01\0"):
            with self.subTest(wire=bad), self.assertRaisesRegex(FrameError, "trailing"):
                FrameReader(Serial(self.clock, [(0, bad)])).read_frame()

    def test_silence_and_truncated_reply_share_one_deadline(self):
        for fragments in ([], [(0.1, b"\x01\x02")]):
            self.clock.now = 0
            device = Serial(self.clock, fragments)
            with self.assertRaisesRegex(FrameError, "timed out"):
                FrameReader(device, timeout=0.123).read_frame()
            self.assertAlmostEqual(self.clock.now, 0.123)
            self.assertEqual(device.timeout, 0.05)

    def test_slow_trickle_does_not_extend_whole_response_deadline(self):
        device = Serial(self.clock, [(i / 100, b"\x02") for i in range(1, 100)])
        with self.assertRaisesRegex(FrameError, "timed out"):
            FrameReader(device, timeout=0.123).read_frame()
        self.assertAlmostEqual(self.clock.now, 0.123)

    def test_io_failure_is_not_retried_or_converted_into_partial_success(self):
        device = Serial(self.clock)
        device.read = lambda _: (_ for _ in ()).throw(OSError("USB disconnected"))
        with self.assertRaisesRegex(OSError, "USB disconnected"):
            FrameReader(device).read_frame()

    def test_invalid_limits_fail_before_read(self):
        for max_bytes, timeout in ((0, 3), (512, 0), (512, -1), (512, float("inf")), (512, float("nan"))):
            with self.subTest(max_bytes=max_bytes, timeout=timeout), self.assertRaises(ValueError):
                FrameReader(Serial(self.clock), max_bytes, timeout)

    def test_negotiated_large_response_crosses_usb_packets_and_keeps_exact_bound(self):
        response = b"\x02" * 4607 + b"\0"
        device = Serial(self.clock, [(i / 1000, response[i * 512:(i + 1) * 512]) for i in range(9)])
        self.assertEqual(FrameReader(device, max_bytes=4608).read_frame(), response)
        self.assertLessEqual(len(device.requests), 17)
        for bad in (b"\x02" * 4608, b"\x02" * 4608 + b"\0"):
            with self.assertRaisesRegex(FrameError, "oversized"):
                FrameReader(Serial(self.clock, [(self.clock.now, bad)]), max_bytes=4608).read_frame()


if __name__ == "__main__":
    unittest.main()
