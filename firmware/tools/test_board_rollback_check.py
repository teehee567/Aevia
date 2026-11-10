"""Rollback verdict and mandatory restoration regressions; no hardware access."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

import serial

from board_rollback_check import check, require_panic, wait_ready


def panic(line=98):
    return {"panic": {"file_hash": 123, "line": line, "startup_stage": 1,
                      "core": 0, "column": 9}}


class RollbackTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(candidate="panic.bin", restore_image="healthy.bin",
                                    timeout=90, settle=30, panic_source_hash=123,
                                    panic_line=98, panic_stage=1, panic_core=0)
        self.probe = Mock()
        self.report = {"events": [], "errors": [], "passed": False,
                       "rollback_passed": False, "restore_passed": False}

    def run_check(self, fallback, restored=None):
        responses = [{"reset": {"panic": None}}, fallback,
                     {"reset": {"panic": None}} if restored is None else restored]
        with patch("board_rollback_check.wait_ready", side_effect=responses), \
                patch("board_rollback_check.event"), \
                patch("board_rollback_check.reconnect"), \
                patch("board_rollback_check.time.sleep") as sleep:
            check(self.probe, self.args, self.report)
        sleep.assert_called_once_with(30)
        self.assertEqual(self.probe.load.call_args_list,
                         [call("panic.bin"), call("healthy.bin")])

    def test_only_the_exact_panic_can_establish_rollback(self):
        require_panic(panic(), 123, 98)
        for reset in ({"panic": None}, panic(line=99), {"panic": {"file_hash": 123}}):
            with self.subTest(reset=reset), self.assertRaises(RuntimeError):
                require_panic(reset, 123, 98)

    def test_success_requires_both_rollback_and_clean_restoration(self):
        self.run_check({"reset": panic()})
        self.assertTrue(self.report["passed"])
        self.assertTrue(self.report["rollback_passed"])
        self.assertTrue(self.report["restore_passed"])

    def test_bad_panic_fails_but_still_restores_known_image(self):
        self.run_check({"reset": panic(line=99)})
        self.assertFalse(self.report["passed"])
        self.assertFalse(self.report["rollback_passed"])
        self.assertTrue(self.report["restore_passed"])
        self.assertIn("retained panic mismatch", self.report["errors"][0])

    def test_panic_in_restored_image_cannot_pass(self):
        self.run_check({"reset": panic()}, {"reset": panic()})
        self.assertTrue(self.report["rollback_passed"])
        self.assertFalse(self.report["restore_passed"])
        self.assertFalse(self.report["passed"])

    def test_stale_preflight_panic_never_selects_a_candidate(self):
        with patch("board_rollback_check.wait_ready", return_value={"reset": panic()}):
            with self.assertRaisesRegex(RuntimeError, "preflight has retained panic"):
                check(self.probe, self.args, self.report)
        self.probe.load.assert_not_called()

    def test_disconnect_and_candidate_enumeration_do_not_count_as_fallback(self):
        self.probe.system.side_effect = [serial.SerialException("candidate reset"),
                                        {"Info": {"recovery": False}},
                                        {"Info": {"recovery": False}},
                                        {"Info": {"recovery": False}}]
        stages = iter([1, 10, 10])
        samples = iter([{"updated_us": 1_000_000, "imu_samples": 100},
                        {"updated_us": 2_000_000, "imu_samples": 1550}])

        def query(name, payload):
            if name == "boot":
                return {"System": {"BootStatus": {"stage": next(stages)}}}
            if name == "acquisition":
                return {"Device": {"AcquisitionStatus": next(samples)}}
            if name == "imu":
                return {"Device": {"ImuDiagnostics": {"ready": True}}}
            if name == "power":
                return {"Device": {"PowerStatus": {"verified": True, "control": 128}}}
            if name == "reset":
                return {"Device": {"ResetStatus": panic()}}
            raise AssertionError(name)

        self.probe.query.side_effect = query
        with patch("board_rollback_check.event"), \
                patch("board_rollback_check.reconnect") as reconnect, \
                patch("board_rollback_check.time.sleep"):
            evidence = wait_ready(self.probe, [], "fallback")
        reconnect.assert_called_once_with(self.probe)
        self.assertEqual(evidence["imu_samples_delta"], 1450)
        self.assertEqual(evidence["boot"]["stage"], 10)
        self.assertEqual(evidence["reset"], panic())


if __name__ == "__main__":
    unittest.main()
