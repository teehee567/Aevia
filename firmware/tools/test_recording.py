import csv
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch
import zlib

import recording as log


def record(kind, sequence, payload, at_ns=123):
    data = log.RECORD_HEADER.pack(kind, 1, len(payload), sequence, at_ns, 0) + payload
    return data + struct.pack("<I", zlib.crc32(data))


def health(accepted, dropped=0):
    return b"\x01" + struct.pack("<QIIIIIQI", accepted, dropped, 10, 0, 0, 0, 0, 0)


def pages(stream, closed=True, session=7, nonce=99):
    result = bytearray()
    for number, offset in enumerate(range(0, len(stream), 480)):
        payload = stream[offset:offset + 480]
        final = closed and offset + len(payload) == len(stream)
        page = log.PAGE_HEADER.pack(log.MAGIC, session, number, len(payload), int(final), nonce)
        page += payload + bytes(480 - len(payload))
        result.extend(page + struct.pack("<I", zlib.crc32(page)))
    return bytes(result)


def fix_page_crc(page):
    page[508:512] = struct.pack("<I", zlib.crc32(page[:508]))
    return bytes(page)


def imu():
    data = bytearray(120)
    data[:4] = bytes([1, 15, 63, 0])
    struct.pack_into("<4Q", data, 4, 1_000, 1_100, 1_300, 1_000)
    struct.pack_into("<6i", data, 36, -524288, 0, 524287, 42, -42, 7)
    data[60:66] = bytes([14, 14, 14, 14, 14, 14])
    struct.pack_into("<6d", data, 72, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0)
    return bytes(data)


def uart(data, offset=0):
    return bytes([1, 1, 0, 0]) + struct.pack("<QQH", 456, offset, len(data)) + bytes(2) + data


def sensors(samples=10, timestamp=1_000_000, rx_bytes=100, fix_valid=False):
    return {"acquisition": {"imu_samples": samples, "updated_us": timestamp,
                            "imu_errors": 0, "gnss_uart_errors": 0, "gnss_parse_errors": 0},
            "imu": {"ready": True, "driver_errors": 0, "timing_errors": 0,
                    "gap_errors": 0, "range_errors": 0, "drdy_timeouts": 0,
                    "spi_timeouts": 0, "numeric_range_errors": 0},
            "transport": {"ports": [{"rx_bytes": 10, "rx_errors": 0},
                                     {"rx_bytes": rx_bytes, "rx_errors": 0}]},
            "fix": {"position_valid": fix_valid, "velocity_valid": fix_valid}}


class FakeClient:
    def __init__(self, data):
        self.data = data
        self.calls = []
        self.fail_at = None
        self.max_read_len = 256

    def operation(self, command, **fields):
        assert command == "list"
        return {"Entry": {"session": 7, "file_bytes": 512 * 1024 * 1024,
                          "valid_bytes": len(self.data), "closed": True}}

    def read(self, session, offset, length):
        self.calls.append((session, offset, length))
        if self.fail_at is not None and offset >= self.fail_at:
            raise OSError("USB disappeared")
        return self.data[offset:offset + length]

    def status(self):
        return {"state": "Ready"}

    def sessions(self):
        yield self.operation("list")["Entry"]


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def verify(self, data):
        path = self.root / "test.aev"
        path.write_bytes(data)
        return log.verify_log(path)

    def test_wire_vectors_match_rust_recording_namespace(self):
        self.assertEqual(log.request_bytes(0, "status"), bytes([2, 2, 1, 1, 0]))
        self.assertEqual(log.request_bytes(42, "read", session=7, offset=256, len=256),
                         bytes([9, 2, 42, 4, 7, 128, 2, 128, 2, 0]))
        self.assertEqual(log.request_bytes(42, "read", session=7, offset=256, len=4096),
                         bytes([9, 2, 42, 4, 7, 128, 2, 128, 32, 0]))
        for length in [0, 4097]:
            with self.assertRaises(ValueError):
                log.request_bytes(1, "read", session=7, offset=0, len=length)
        with self.assertRaises(ValueError):
            log.varint(-1)

    def test_cross_page_records_and_sealed_summary_are_verified(self):
        stream = b"".join(record(42, i, bytes([i]) * 256) for i in range(5))
        stream += record(255, 5, health(5))
        summary = self.verify(pages(stream))
        self.assertEqual(summary["counts"], {"unknown_42": 5, "end": 1})
        self.assertEqual(summary["missing_sequences"], 0)
        self.assertTrue(summary["sealed_and_contiguous"])
        self.assertEqual(summary["session"], 7)
        self.assertEqual(summary["nonce"], 99)

    def test_page_corruption_padding_identity_order_and_truncation_are_rejected(self):
        valid = pages(record(42, 0, b"test") + record(255, 1, health(1)))
        corruptions = []
        changed = bytearray(valid)
        changed[30] ^= 1
        corruptions.append(changed)
        changed = bytearray(valid)
        changed[507] = 1
        corruptions.append(fix_page_crc(changed))
        changed = bytearray(valid)
        struct.pack_into("<I", changed, 12, 1)
        corruptions.append(fix_page_crc(changed))
        corruptions.extend([valid[:-1], valid + valid])
        for data in corruptions:
            with self.subTest(length=len(data)), self.assertRaises(log.RecordingError):
                self.verify(data)
        prefix = pages(record(42, 0, b"abc"), closed=False)
        wrong = bytearray(pages(record(42, 1, b"def"), closed=False, nonce=100))
        struct.pack_into("<I", wrong, 12, 1)
        with self.assertRaisesRegex(log.RecordingError, "nonce"):
            self.verify(prefix + fix_page_crc(wrong))

    def test_record_crc_is_independent_of_valid_page_crc(self):
        broken = bytearray(record(42, 0, b"hello"))
        broken[24] ^= 1
        with self.assertRaisesRegex(log.RecordingError, "record CRC"):
            self.verify(pages(broken, closed=False))

    def test_interrupted_tail_is_reported_and_closed_partial_rejected(self):
        stream = record(42, 0, b"one") + record(42, 1, b"two")[:26]
        result = self.verify(pages(stream, closed=False))
        self.assertEqual(result["records"], 1)
        self.assertEqual(result["trailing_record_bytes"], 26)
        self.assertFalse(result["sealed_and_contiguous"])
        with self.assertRaisesRegex(log.RecordingError, "partial"):
            self.verify(pages(stream))

    def test_gaps_and_loss_are_visible_even_in_closed_files(self):
        result = self.verify(pages(record(42, 2, b"x") + record(255, 4, health(1, 3))))
        self.assertEqual(result["missing_sequences"], 3)
        self.assertFalse(result["sealed_and_contiguous"])
        self.assertEqual(result["end"]["dropped_records"], 3)
        for sequence in (0, 1):
            with self.assertRaisesRegex(log.RecordingError, "sequences"):
                self.verify(pages(record(42, 1, b"x") + record(42, sequence, b"y"), closed=False))

    def test_no_records_after_end_and_unknown_record_versions_rejected(self):
        with self.assertRaisesRegex(log.RecordingError, "end marker"):
            self.verify(pages(record(255, 0, health(0)) + record(42, 1, b"x")))
        changed = bytearray(record(42, 0, b"x"))
        changed[1] = 2
        with self.assertRaisesRegex(log.RecordingError, "record header"):
            self.verify(pages(changed, closed=False))

    def test_export_keeps_raw_uart_native_extrema_and_read_timing(self):
        stream = record(1, 0, b'{"section":"test"}')
        stream += record(2, 1, imu()) + record(3, 2, uart(b"\x00\xaa\xff"))
        stream += record(4, 3, uart(b"command-port")) + record(255, 4, health(4))
        path = self.root / "race.aev"
        path.write_bytes(pages(stream))
        output = self.root / "export"
        summary = log.export_log(path, output)
        self.assertTrue(summary["sealed_and_contiguous"])
        self.assertEqual((output / "gnss-com2.bin").read_bytes(), b"\x00\xaa\xff")
        self.assertEqual((output / "gnss-com1.bin").read_bytes(), b"command-port")
        with (output / "imu.csv").open(newline="") as source:
            row = next(csv.DictReader(source))
        self.assertEqual(row["raw_gx"], "-524288")
        self.assertEqual(row["raw_gz"], "524287")
        self.assertEqual(row["read_finish_ns"], "1300")
        self.assertEqual(row["gx_rps"], "0.1")
        self.assertEqual(row["counter_ax"], "14")
        with self.assertRaisesRegex(log.RecordingError, "new or empty"):
            log.export_log(path, output)

    def test_invalid_scaled_sample_never_exports_zero_as_valid_motion(self):
        payload = bytearray(imu())
        payload[1] = 0
        row = log.imu_payload(payload)
        self.assertIsNone(row["gx_rps"])
        self.assertIsNone(row["latest_edge_ns"])
        self.assertIsNone(row["read_finish_ns"])
        self.assertEqual(row["raw_gx"], -524288)

    def test_download_interruption_resume_and_destination_protection(self):
        stream = b"".join(record(42, i, bytes([i]) * 256) for i in range(5))
        client = FakeClient(pages(stream + record(255, 5, health(5))))
        output = self.root / "race.aev"
        client.fail_at = 512
        with self.assertRaisesRegex(OSError, "USB"):
            log.download(client, 7, output)
        self.assertFalse(output.exists())
        self.assertEqual(output.with_name("race.aev.part").stat().st_size, 512)
        with self.assertRaisesRegex(log.RecordingError, "resume"):
            log.download(client, 7, output)
        client.fail_at = None
        client.calls.clear()
        result = log.download(client, 7, output, resume=True)
        self.assertEqual(output.read_bytes(), client.data)
        self.assertTrue(result["sealed_and_contiguous"])
        self.assertEqual([offset for _, offset, _ in client.calls[:3]], [0, 256, 512])
        with self.assertRaisesRegex(log.RecordingError, "destination exists"):
            log.download(client, 7, output)

    def test_resume_rejects_another_card_or_session_before_modifying_partial(self):
        stream = record(42, 0, bytes(256)) + record(42, 1, bytes(256)) + record(255, 2, health(2))
        client = FakeClient(pages(stream))
        client.fail_at = 512
        output = self.root / "race.aev"
        with self.assertRaises(OSError):
            log.download(client, 7, output)
        original = output.with_name("race.aev.part").read_bytes()
        client.data = pages(stream, nonce=100)
        client.fail_at = None
        with self.assertRaisesRegex(log.RecordingError, "contents changed"):
            log.download(client, 7, output, resume=True)
        self.assertEqual(output.with_name("race.aev.part").read_bytes(), original)

    def test_client_polls_original_token_and_does_not_repeat_mutation(self):
        client = object.__new__(log.Client)
        client.next_request_id = 42
        client.timeout = 1
        client._reply = Mock(side_effect=["Pending", "Pending", {"Started": {"session": 7}}])
        with patch("recording.time.sleep"):
            self.assertEqual(client.operation("start"), {"Started": {"session": 7}})
        self.assertEqual(client._reply.call_args_list[0].args, (42, "start"))
        self.assertTrue(all(call.args == (42, "result") for call in client._reply.call_args_list[1:]))
        self.assertEqual(client.next_request_id, 43)

    def test_client_checks_chunk_identity_length_and_crc(self):
        client = object.__new__(log.Client)
        client.max_read_len = 4096
        valid = {"session": 7, "offset": 256, "bytes": [1, 2], "crc32": zlib.crc32(bytes([1, 2]))}
        client.operation = Mock(return_value={"Data": valid})
        self.assertEqual(client.read(7, 256, 2), bytes([1, 2]))
        for changed in ({**valid, "session": 8}, {**valid, "offset": 0},
                        {**valid, "bytes": [1]}, {**valid, "crc32": 0}):
            client.operation.return_value = {"Data": changed}
            with self.assertRaises(log.RecordingError):
                client.read(7, 256, 2)

    def test_uart_stream_gaps_are_reported_and_backward_offsets_rejected(self):
        stream = record(3, 0, uart(b"abc", 100)) + record(3, 1, uart(b"def", 110))
        result = self.verify(pages(stream + record(255, 2, health(2))))
        self.assertEqual(result["uart_streams"]["gnss_com2"]["missing_bytes"], 7)
        self.assertFalse(result["sealed_and_contiguous"])
        with self.assertRaisesRegex(log.RecordingError, "byte offsets"):
            self.verify(pages(record(3, 0, uart(b"abc", 100)) + record(3, 1, uart(b"def", 102)), closed=False))

    def test_preflight_requires_durable_progress_without_drops(self):
        first = {"state": "Recording", "accepted_records": 10, "written_pages": 1,
                 "dropped_records": 0, "io_errors": 0, "last_error": None}
        last = {**first, "accepted_records": 20, "written_pages": 2}
        client = Mock()
        client.status.side_effect = [first, last]
        with patch("recording.time.monotonic", side_effect=[0, 0, 0.5, 1]), patch("recording.time.sleep"), \
                patch("recording.sensor_snapshot", side_effect=[sensors(), sensors(737, 2_000_000, 200)]):
            self.assertTrue(log.preflight(client, seconds=1)["passed"])
        client.status.side_effect = [first, {**last, "dropped_records": 1}]
        with patch("recording.time.monotonic", side_effect=[0, 0, 0.5, 1]), patch("recording.time.sleep"), \
                patch("recording.sensor_snapshot", side_effect=[sensors(), sensors(737, 2_000_000, 200)]):
            self.assertFalse(log.preflight(client, seconds=1)["passed"])

    def preflight(self, last_sensors, require_fix=False):
        first = {"state": "Recording", "accepted_records": 10, "written_pages": 1,
                 "dropped_records": 0, "io_errors": 0, "last_error": None}
        last = {**first, "accepted_records": 20, "written_pages": 2}
        client = Mock()
        client.status.side_effect = [first, last]
        with patch("recording.time.monotonic", side_effect=[0, 0, 0.5, 1]), patch("recording.time.sleep"), \
                patch("recording.sensor_snapshot", side_effect=[sensors(), last_sensors]):
            return log.preflight(client, seconds=1, require_fix=require_fix)

    def test_preflight_rejects_health_only_progress_and_dead_gnss(self):
        result = self.preflight(sensors(timestamp=2_000_000))
        self.assertFalse(result["passed"])
        self.assertIn("no IMU sample progress", result["failures"])
        self.assertIn("no GNSS COM2 input progress", result["failures"])

    def test_preflight_rejects_sensor_errors_and_counter_resets(self):
        last = sensors(737, 2_000_000, 200)
        last["imu"]["ready"] = False
        last["imu"]["driver_errors"] = 1
        last["imu"]["timing_errors"] = 1
        last["transport"]["ports"][1]["rx_errors"] = 1
        result = self.preflight(last)
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["failures"]), 4)
        result = self.preflight(sensors(1, 1, 1))
        self.assertFalse(result["passed"])
        self.assertIsNone(result["observed_imu_rate_hz"])

    def test_preflight_fix_is_explicit_and_rate_is_reported(self):
        result = self.preflight(sensors(737, 2_000_000, 200))
        self.assertTrue(result["passed"])
        self.assertEqual(result["observed_imu_rate_hz"], 727)
        self.assertFalse(result["fix_valid"])
        self.assertFalse(self.preflight(sensors(737, 2_000_000, 200), require_fix=True)["passed"])
        self.assertTrue(self.preflight(sensors(737, 2_000_000, 200, True), require_fix=True)["passed"])

    def test_device_queries_validate_typed_response_without_recording_tokens(self):
        client = object.__new__(log.Client)
        client._exchange = Mock(return_value={"Device": {"AcquisitionStatus": {"imu_samples": 1}}})
        self.assertEqual(client.device_status(5, "AcquisitionStatus"), {"imu_samples": 1})
        self.assertEqual(client._exchange.call_args.args[0], bytes([3, 1, 5, 0]))
        with self.assertRaisesRegex(log.RecordingError, "expected device"):
            client.device_status(7, "ImuDiagnostics")
        client.device_status = Mock(side_effect=[{}, {}, {}, {}])
        self.assertEqual(set(log.sensor_snapshot(client)), {"acquisition", "imu", "transport", "fix"})
        self.assertEqual([call.args for call in client.device_status.call_args_list],
                         [(5, "AcquisitionStatus"), (7, "ImuDiagnostics"),
                          (9, "GnssTransportStatus"), (11, "GnssFixStatus")])

    def test_raw_gnss_gate_requires_in_file_epoch_and_ephemeris_progress(self):
        def coverage(observations, ephemerides):
            data = bytearray(40)
            data[:4] = bytes([1, 1, 3, 0])
            struct.pack_into("<4I", data, 4, observations, observations, ephemerides, 0)
            struct.pack_into("<I", data, 20, 9)
            struct.pack_into("<2Q", data, 24, 123, 456)
            return bytes(data)
        stream = record(13, 0, struct.pack("<I", 9)) + record(11, 1, coverage(1, 1))
        stream += record(3, 2, uart(b"received")) + record(11, 3, coverage(3, 2)) + record(255, 4, health(4))
        result = self.verify(pages(stream))
        self.assertTrue(log.raw_gnss_gate(result)["passed"])
        result["gnss_raw_coverage_latest"]["advancing_epochs"] = 1
        result["gnss_raw_coverage_latest"]["ephemeris_frames"] = 0
        gate = log.raw_gnss_gate(result)
        self.assertFalse(gate["passed"])
        self.assertEqual(len(gate["failures"]), 2)

    def test_rotation_identity_does_not_claim_one_part_is_the_whole_race(self):
        identity = b"\x01" + struct.pack("<IIIQ", 7, 7, 0, 0)
        rotating = b"\x01\x01" + struct.pack("<I", 8)
        stopped = b"\x01\x00" + bytes(4)
        for intent, expected in [(rotating, False), (stopped, True)]:
            stream = record(14, 0, identity) + record(15, 1, intent) + record(255, 2, health(0))
            result = self.verify(pages(stream))
            self.assertEqual(result["whole_recording_in_this_file"], expected)
            self.assertEqual(result["recording_identity"]["recording_id"], 7)
        with self.assertRaisesRegex(log.RecordingError, "immediately precede"):
            self.verify(pages(record(15, 0, stopped) + record(42, 1, b"bad") + record(255, 2, health(0))))

    def test_download_all_skips_only_verified_same_device_file_and_preserves_index(self):
        stream = record(42, 0, b"race") + record(255, 1, health(1))
        client = FakeClient(pages(stream))
        output = self.root / "all"
        first = log.download_all(client, output)
        self.assertFalse(first["sessions"][0]["already_verified"])
        client.calls.clear()
        second = log.download_all(client, output, resume=True)
        self.assertTrue(second["sessions"][0]["already_verified"])
        self.assertEqual(len(client.calls), 2)
        original = (output / "recordings.json").read_bytes()
        client.data = pages(stream, nonce=100)
        with self.assertRaisesRegex(log.RecordingError, "differs from the device"):
            log.download_all(client, output, resume=True)
        self.assertEqual((output / "recordings.json").read_bytes(), original)

    def test_download_preserves_preexisting_companion_and_rechecks_initial_identity(self):
        stream = record(42, 0, b"race") + record(255, 1, health(1))
        output = self.root / "race.aev"
        companion = self.root / "race.aev.json"
        companion.write_text("unrelated", encoding="utf-8")
        client = FakeClient(pages(stream))
        with self.assertRaisesRegex(log.RecordingError, "companion"):
            log.download(client, 7, output)
        self.assertEqual(companion.read_text(), "unrelated")
        output = self.root / "changed.aev"
        original_read = client.read
        def changed_read(session, offset, length):
            if len(client.calls) == 2:
                client.data = pages(stream, nonce=100)
            return original_read(session, offset, length)
        client.read = changed_read
        with self.assertRaisesRegex(log.RecordingError, "identity changed"):
            log.download(client, 7, output)
        self.assertFalse(output.exists())

    def test_discovery_negotiates_large_frames_and_rejects_inconsistent_limits(self):
        client = object.__new__(log.Client)
        client.frame_reader = Mock(max_bytes=512)
        protocol = {"protocol_version": 1, "schema_version": 1, "capabilities": 128,
                    "max_response_frame": 4608}
        client.configure_protocol(protocol)
        self.assertEqual(client.frame_reader.max_bytes, 4608)
        client._reply = Mock(return_value={"Status": {"format_version": 1, "max_read_len": 4096,
                                                     "next_request_id": 123}})
        client.status()
        self.assertEqual(client.max_read_len, 4096)
        self.assertEqual(client.next_request_id, 123)
        client.configure_protocol({**protocol, "max_response_frame": 512})
        with self.assertRaisesRegex(log.RecordingError, "exceeds"):
            client.status()
        client._reply.return_value["Status"]["max_read_len"] = 256
        client.status()
        self.assertEqual(client.max_read_len, 256)
        for bound in [0, 511, 4609, True]:
            with self.assertRaisesRegex(log.RecordingError, "frame limit"):
                client.configure_protocol({**protocol, "max_response_frame": bound})

    def test_four_kib_download_and_resume_keep_identity_probe_and_page_alignment(self):
        stream = b"".join(record(42, i, bytes([i]) * 256) for i in range(30))
        client = FakeClient(pages(stream + record(255, 30, health(30))))
        client.max_read_len = 4096
        client.fail_at = 4096
        output = self.root / "large.aev"
        with self.assertRaises(OSError):
            log.download(client, 7, output)
        self.assertEqual(client.calls[:2], [(7, 0, 512), (7, 0, 4096)])
        partial = output.with_name("large.aev.part")
        self.assertEqual(partial.stat().st_size, 4096)
        # A failed final disk write may leave a partial sector; it is discarded
        # back to the last independently verifiable512-byte page on resume.
        with partial.open("ab") as target:
            target.write(b"unverified")
        client.fail_at = None
        client.calls.clear()
        log.download(client, 7, output, resume=True)
        self.assertEqual(client.calls[:2], [(7, 0, 512), (7, 4096, 4096)])
        self.assertEqual(output.read_bytes(), client.data)


if __name__ == "__main__":
    unittest.main()
