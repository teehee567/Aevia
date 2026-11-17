import hashlib
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import serial

from upload import APP_PID, READY, REBOOT, RECOVERY_PID, connect, disconnect, frame, recovery, request, uint, upload

ERROR = b"\x02\x02\0"


def image():
    data = bytearray(288)
    data[0], data[12] = 0xE9, 32
    return bytes(data)


class Link:
    def __init__(self, reads=()):
        self.reads = iter(reads)
        self.buffer = bytearray()
        self.dtr, self.closed = True, False
        self.port = "COM8"
        self.writes = []
        self.reset_input_buffer = Mock()

    def read(self, size):
        if not self.buffer:
            value = next(self.reads)
            if isinstance(value, Exception):
                raise value
            self.buffer.extend(value)
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def close(self):
        self.closed = True

    def write(self, data):
        self.writes.append(data)
        return len(data)


class UploadTests(unittest.TestCase):
    def test_postcard_wire_vectors(self):
        self.assertEqual(frame(b"\0"), READY)
        self.assertEqual(frame(b"\x02\x03\0\x11\0"), b"\x03\x02\x03\x02\x11\x01\0")
        self.assertEqual(frame(b"\x01" + uint(288) + b"\x5a" * 32),
                         b"\x24\x01\xa0\x02" + b"\x5a" * 32 + b"\0")
        for value, expected in [(0, b"\0"), (127, b"\x7f"), (128, b"\x80\x01"),
                                (288, b"\xa0\x02"), (0xFFFFFFFF, b"\xff\xff\xff\xff\x0f")]:
            self.assertEqual(uint(value), expected)

    def test_cobs_zeroes_and_254_byte_boundaries(self):
        for size in range(261):
            for payload in (bytes(size), b"\xff" * size, bytes(i % 256 for i in range(size))):
                encoded, decoded, offset = frame(payload), bytearray(), 0
                while offset < len(encoded) - 1:
                    code = encoded[offset]
                    self.assertNotEqual(code, 0)
                    decoded.extend(encoded[offset + 1:offset + code])
                    offset += code
                    if code < 255 and offset < len(encoded) - 1:
                        decoded.append(0)
                self.assertEqual(decoded, payload)
                self.assertNotIn(0, encoded[:-1])
                self.assertLessEqual(len(encoded), 264)

    def test_empty_reads_and_fragmented_reply(self):
        request(Link([b"", b"\x01", b"", b"\x01\0"]), b"\0")

    def test_error_and_malformed_replies_are_rejected(self):
        for reply in (ERROR, b"\0\0\0", b"\x03\x01\0", REBOOT):
            with self.subTest(reply=reply), self.assertRaises(RuntimeError):
                request(Link([reply]), b"\0")

    def test_partial_reply_obeys_deadline(self):
        with patch("upload.time.monotonic", side_effect=[0, 0, 36]):
            with self.assertRaises(TimeoutError):
                request(Link([b"\x01"]), b"\0")

    def test_disconnect_signals_dtr_and_closes(self):
        link = Link()
        with patch("upload.time.sleep"):
            disconnect(link)
        self.assertFalse(link.dtr)
        self.assertTrue(link.closed)

    def test_upload_switches_modes_and_sends_exact_frames(self):
        data = image()
        app, loader, booted = Link([REBOOT]), Link([READY, READY, REBOOT]), Link()
        with patch("upload.connect", side_effect=[(app, APP_PID), (loader, RECOVERY_PID),
                                                  (booted, APP_PID)]) as opened, patch("upload.time.sleep"):
            self.assertEqual(upload(data), "COM8")
        self.assertEqual(app.writes, [frame(b"\0")])
        self.assertEqual(loader.writes, [
            frame(b"\x01" + uint(len(data)) + hashlib.sha256(data).digest()),
            frame(b"\x02" + uint(256) + data[:256]),
            frame(b"\x02" + uint(32) + data[256:]),
        ])
        self.assertEqual(opened.call_args_list[-1].args, (APP_PID,))
        self.assertTrue(app.closed and loader.closed and booted.closed)

    def test_recovery_rejection_is_not_a_disconnect(self):
        app = Link([ERROR])
        with patch("upload.connect", return_value=(app, APP_PID)), patch("upload.time.sleep"):
            with self.assertRaises(RuntimeError):
                recovery()
        self.assertTrue(app.closed)

    def test_lost_reboot_reply_requires_new_usb_mode(self):
        app, loader = Link([serial.SerialException("detached")]), Link()
        with patch("upload.connect", side_effect=[(app, APP_PID), (loader, RECOVERY_PID)]) as opened, patch("upload.time.sleep"):
            self.assertIs(recovery(), loader)
        self.assertEqual(opened.call_args_list[-1].args, (RECOVERY_PID,))
        self.assertTrue(app.closed)

    def test_lost_final_reply_requires_application_to_return(self):
        for result in ((Link(), APP_PID), TimeoutError("USB missing")):
            loader = Link([READY, READY, serial.SerialException("detached")])
            with patch("upload.connect", side_effect=[(loader, RECOVERY_PID), result]), patch("upload.time.sleep"):
                if isinstance(result, Exception):
                    with self.assertRaises(TimeoutError):
                        upload(image())
                else:
                    self.assertEqual(upload(image()), "COM8")
            self.assertTrue(loader.closed)

    def test_interrupted_or_rejected_upload_never_waits_for_application(self):
        for reply in (ERROR, serial.SerialException("detached")):
            loader = Link([READY, reply])
            with patch("upload.connect", return_value=(loader, RECOVERY_PID)) as opened, patch("upload.time.sleep"):
                with self.assertRaises((RuntimeError, serial.SerialException)):
                    upload(image())
                self.assertEqual(opened.call_count, 1)
            self.assertTrue(loader.closed)

    def test_invalid_image_never_opens_usb(self):
        with patch("upload.connect") as opened:
            for data in (b"", bytes(288), b"\x7fELF" + bytes(284)):
                with self.assertRaises(ValueError):
                    upload(data)
            opened.assert_not_called()

    def test_multiple_devices_are_rejected(self):
        port = SimpleNamespace(device="COM8", vid=0xC0C0, pid=APP_PID)
        with patch("upload.list_ports.comports", return_value=[port, port]):
            with self.assertRaisesRegex(RuntimeError, "only one"):
                connect()

    def test_failed_port_setup_is_closed_before_retry(self):
        port = SimpleNamespace(device="COM8", vid=0xC0C0, pid=APP_PID)
        failed, ready = Link(), Link()
        failed.reset_input_buffer.side_effect = serial.SerialException("detached")
        with patch("upload.list_ports.comports", return_value=[port]), patch("upload.time.sleep"), \
             patch("upload.serial.Serial", side_effect=[failed, ready]):
            self.assertEqual(connect(), (ready, APP_PID))
        self.assertTrue(failed.closed)


if __name__ == "__main__":
    unittest.main()
