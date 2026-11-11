"""USB query evidence regressions using mocked serial and decoder streams."""
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from board_check import Probe
from bench_acceptance import evaluate


def probe(wire, decoded=""):
    instance = Probe.__new__(Probe)
    instance.device = Mock()
    instance.device.read_until.return_value = wire
    instance.decoder = SimpleNamespace(stdin=io.StringIO(), stdout=io.StringIO(decoded))
    instance.started = time.monotonic()
    instance.records = []
    return instance


class QueryEvidenceTests(unittest.TestCase):
    def test_incomplete_frame_is_retained_before_any_decode(self):
        instance = probe(b"\x03\x02")
        with self.assertRaisesRegex(RuntimeError, "incomplete response"):
            instance.query("boot", b"\0\7")
        self.assertEqual(instance.records[0]["wire"], "0302")
        self.assertIsNone(instance.records[0]["response"])
        self.assertIn("incomplete response", instance.records[0]["error"])
        self.assertEqual(instance.decoder.stdin.getvalue(), "")
        self.assertFalse(evaluate(instance.records)["passed"])

    def test_bad_wire_decoder_exit_or_bad_json_retains_failing_frame(self):
        for decoded, exception in (("", RuntimeError), ("bad JSON\n", json.JSONDecodeError)):
            with self.subTest(decoded=decoded):
                instance = probe(b"\xff\0", decoded)
                with self.assertRaises(exception):
                    instance.query("boot", b"\0\7")
                self.assertEqual(instance.records[0]["wire"], "ff00")
                self.assertIsNone(instance.records[0]["response"])
                self.assertIn("error", instance.records[0])

    def test_success_populates_the_original_record_without_duplicates(self):
        response = {"System": {"BootStatus": {"stage": 10}}}
        instance = probe(b"\x01\0", json.dumps(response) + "\n")
        self.assertEqual(instance.query("boot", b"\0\7"), response)
        self.assertEqual(len(instance.records), 1)
        self.assertEqual(instance.records[0]["response"], response)
        self.assertNotIn("error", instance.records[0])

    def test_unexpected_typed_response_is_retained_with_decode_and_error(self):
        response = {"System": "Ok"}
        instance = probe(b"\x01\0", json.dumps(response) + "\n")
        with self.assertRaisesRegex(RuntimeError, "unexpected response"):
            instance.query("boot", b"\0\7")
        self.assertEqual(instance.records[0]["response"], response)
        self.assertIn("error", instance.records[0])

    def test_received_frame_is_flushed_to_append_journal_before_decode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.frames.jsonl"
            path.write_text('{"previous_session":true}\n')
            instance = probe(b"\x03\x02")
            instance.session = "mock-session"
            instance._journal_file = path.open("a", encoding="utf-8")
            try:
                with self.assertRaisesRegex(RuntimeError, "incomplete response"):
                    instance.query("boot", b"\0\7")
                # Read while the writer is open: this requires immediate flush.
                lines = [json.loads(line) for line in path.read_text().splitlines()]
                self.assertTrue(lines[0]["previous_session"])
                self.assertEqual(lines[1]["event"], "received")
                self.assertEqual(lines[1]["wire"], "0302")
                self.assertIsNone(lines[1]["response"])
                self.assertEqual(lines[1]["record"], 0)
                self.assertEqual(lines[2]["event"], "error")
            finally:
                instance._journal_file.close()

    def test_constructor_journal_is_optional_and_close_releases_owned_handles(self):
        with tempfile.TemporaryDirectory() as directory:
            for output in (None, Path(directory) / "nested" / "capture.json"):
                decoder = Mock(stdin=io.StringIO(), stdout=io.StringIO())
                device = Mock()
                args = SimpleNamespace(decoder="mock.exe", serial="mock", output=output)
                with patch("board_check.subprocess.Popen", return_value=decoder), \
                        patch("board_check.connect", return_value=device):
                    instance = Probe(args)
                if output is None:
                    self.assertIsNone(instance.journal_path)
                else:
                    self.assertEqual(instance.journal_path, output.with_suffix(".frames.jsonl"))
                    self.assertTrue(instance.journal_path.exists())
                instance.close()
                instance.close()
                self.assertTrue(decoder.stdin.closed)
                device.close.assert_called()
                decoder.wait.assert_called_with(timeout=3)
                if output is not None:
                    self.assertTrue(instance._journal_file.closed)

    def test_fail_fast_waits_for_two_polls_and_ignores_unfinished_soak_duration(self):
        instance = Probe.__new__(Probe)
        instance.args = SimpleNamespace(image=None, brightness=96, settle=0, seconds=1810,
                                        interval=5, transport=False, power_diagnostics=False, fix=True,
                                        power_charge=True,
                                        fail_fast=True, minimum_imu_hz=675,
                                        maximum_imu_hz=800, stationary=True)
        instance.records = []
        instance.system = Mock(return_value={"Info": {"recovery": False}})

        def answer(name, payload):
            if name == "acquisition":
                return {"Device": {"AcquisitionStatus": {}}}
            if name == "imu":
                return {"Device": {"ImuDiagnostics": {}}}
            return {}

        instance.query = Mock(side_effect=answer)
        with patch("board_check.evaluate", return_value={"passed": False, "failures": ["power error"]}) as evaluation, \
                patch("board_check.time.sleep"), patch("builtins.print"):
            with self.assertRaisesRegex(RuntimeError, "bench fail-fast: power error"):
                instance.capture()
        self.assertEqual(sum(call.args[0] == "acquisition" for call in instance.query.call_args_list), 2)
        fix_queries = [call.args for call in instance.query.call_args_list if call.args[0] == "fix"]
        self.assertEqual(fix_queries, [("fix", bytes([1, 11]))] * 2)
        charge_queries = [call.args for call in instance.query.call_args_list if call.args[0] == "power_charge"]
        self.assertEqual(charge_queries, [("power_charge", bytes([1, 12]))] * 2)
        evaluation.assert_called_once_with(instance.records, 0, 675, 800, True, False)

    def test_charge_query_requires_the_matching_typed_response(self):
        for response, valid in (({"Device": {"PowerChargeStatus": {"verified": True}}}, True),
                                ({"Device": {"PowerStatus": {"verified": True}}}, False)):
            with self.subTest(valid=valid):
                instance = probe(b"\x01\0", json.dumps(response) + "\n")
                if valid:
                    self.assertEqual(instance.query("power_charge", bytes([1, 12])), response)
                else:
                    with self.assertRaisesRegex(RuntimeError, "unexpected response"):
                        instance.query("power_charge", bytes([1, 12]))
                self.assertEqual(instance.records[0]["response"], response)


if __name__ == "__main__":
    unittest.main()
