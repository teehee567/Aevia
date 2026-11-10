"""Offline acceptance regressions: no USB board or decoder needed."""
import copy
import unittest

from bench_acceptance import ALL_RESPONSES, RESPONSES, evaluate


def capture():
    polls = []
    for index in range(3):
        now = 30_000_000 + index * 5_000_000
        data = {
            "boot": {"stage": 10},
            "status": {"phase": "Degraded", "qualified": False, "updated_us": now - 1000,
                       "processing_errors": 0, "maximum_step_us": 3,
                       "error_kind": None,
                       "acceleration_mps2": [0, 0, 9.80665], "angular_rate_rps": [0, 0, 0]},
            "power": {"verified": True, "errors": 0, "control": 0x80},
            "peripherals": {"updated_us": now - 100000, "leds_ready": True,
                            "led_errors": 0, "display_errors": 0, "gauge_errors": 0,
                            "buttons_errors": 0},
            "acquisition": {"updated_us": now, "last_imu_us": now - 500,
                            "imu_samples": 40000 + index * 7270, "imu_errors": 3,
                            "gnss_errors": 1, "imu_drops": 2, "gnss_drops": 0,
                            "gnss_uart_errors": 0, "gnss_parse_errors": 0,
                            "imu_queue_depth": 0, "gnss_queue_depth": 0,
                            "imu_queue_high_water": 128, "gnss_queue_high_water": 0,
                            "imu_max_read_latency_us": 1000,
                            "timing": {"capture_errors": 0, "imu_timing_rejected": 3}},
            "gnss": {"config_verified": True, "navigation_time_ready": False,
                     "config_timeouts": 1},
            "imu": {"ready": True, "cycle_clock_active": True, "attempts": 1,
                    "driver_errors": 0, "timing_errors": 3, "gap_errors": 0,
                    "range_errors": 0, "drdy_timeouts": 0, "spi_timeouts": 0,
                    "numeric_range_errors": 0},
            "reset": {"reset_reason": 3, "panic": None},
        }
        polls.append(data)
    return polls


def records(polls):
    return [{"query": name, "response": {outer: {inner: poll[name]}}}
            for poll in polls for name, (outer, inner) in ALL_RESPONSES.items() if name in poll]


def add_transport(polls):
    for index, poll in enumerate(polls):
        poll["transport"] = {
            "ports": [{"rx_bytes": 100 + index * (200 if port == 1 else 0),
                       "rx_errors": 3, "fifo_overflows": 3, "glitches": 0,
                       "framing_errors": 0, "parity_errors": 0, "other_errors": 0}
                      for port in range(2)], "last_rx_error": None}
    return polls


class BenchAcceptanceTests(unittest.TestCase):
    def test_absent_accessories_and_no_fix_pass_with_settled_error_baselines(self):
        report = evaluate(records(capture()), minimum_seconds=10, stationary=True)
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["duration_seconds"], 10)
        self.assertEqual(report["imu_rate_hz"]["minimum"], 1454)
        self.assertFalse(report["qualified"])

    def test_new_errors_and_reinitialization_fail(self):
        for section, field in (("acquisition", "imu_drops"), ("acquisition", "gnss_parse_errors"),
                               ("acquisition", "gnss_errors"), ("imu", "attempts"),
                               ("imu", "spi_timeouts"), ("peripherals", "buttons_errors"),
                               ("power", "errors"), ("status", "processing_errors")):
            with self.subTest(field=field):
                polls = capture()
                polls[-1][section][field] += 1
                self.assertFalse(evaluate(records(polls))["passed"])

    def test_readiness_staleness_and_bad_power_cannot_pass_raw_sample_progress(self):
        mutations = (("boot", "stage", 11), ("power", "verified", False),
                     ("power", "control", 0xA0), ("gnss", "config_verified", False),
                     ("imu", "ready", False), ("imu", "cycle_clock_active", False),
                     ("status", "qualified", True), ("status", "phase", "Failed"),
                     ("status", "error_kind", "Estimator"),
                     ("status", "updated_us", 1000), ("peripherals", "updated_us", 1000),
                     ("acquisition", "last_imu_us", None))
        for section, field, value in mutations:
            with self.subTest(field=field):
                polls = capture()
                polls[-1][section][field] = value
                self.assertFalse(evaluate(records(polls))["passed"])

    def test_config_timeout_baseline_allowed_but_new_timeouts_fail(self):
        polls = capture()
        self.assertTrue(evaluate(records(polls))["passed"])
        polls[-1]["gnss"]["config_timeouts"] += 1
        self.assertIn("GNSS CONFIG readback timed out during observation",
                      evaluate(records(polls))["failures"])

    def test_changed_reset_provenance_or_retained_panic_fail(self):
        polls = capture()
        polls[-1]["reset"]["reset_reason"] += 1
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = capture()
        for poll in polls:
            poll["reset"]["panic"] = {"file_hash": 1, "line": 1, "core": 0}
        self.assertIn("reset diagnostics report a retained panic",
                      evaluate(records(polls))["failures"])

    def test_nonfinite_or_invalid_acceptance_limits_are_rejected(self):
        for limits in ((float("nan"), 1350, 1600), (0, float("inf"), 1600),
                       (0, 1350, float("inf")), (0, 1600, 1350), (-1, 1350, 1600)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                evaluate([], *limits)

    def test_optional_transport_reports_deltas_and_requires_healthy_com2_stream(self):
        polls = add_transport(capture())
        report = evaluate(records(polls))
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["gnss_transport"]["port_counter_deltas"]["COM1"]["rx_bytes"], 0)
        self.assertEqual(report["gnss_transport"]["port_counter_deltas"]["COM2"]["rx_bytes"], 400)
        polls[-1]["transport"]["ports"][1]["rx_bytes"] = polls[-2]["transport"]["ports"][1]["rx_bytes"]
        self.assertIn("GNSS COM2 received bytes did not advance on every poll",
                      evaluate(records(polls))["failures"])

    def test_transport_errors_partial_diagnostics_and_wrong_responses_fail(self):
        for port in range(2):
            polls = add_transport(capture())
            polls[-1]["transport"]["ports"][port]["rx_errors"] += 1
            self.assertFalse(evaluate(records(polls))["passed"])
        polls = add_transport(capture())
        del polls[-1]["transport"]
        self.assertFalse(evaluate(records(polls))["passed"])
        data = records(add_transport(capture()))
        data[-1]["response"] = {"System": "Ok"}
        self.assertFalse(evaluate(data)["passed"])

    def test_optional_power_diagnostics_are_retained_without_extra_policy(self):
        polls = capture()
        for index, poll in enumerate(polls):
            poll["power_diagnostics"] = {"last_failure_stages": index, "i2c_kind": None}
        report = evaluate(records(polls))
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["power_diagnostics"]["baseline"]["last_failure_stages"], 0)
        self.assertEqual(report["power_diagnostics"]["final"]["last_failure_stages"], 2)

    def test_stationary_envelope_is_opt_in_and_rejects_nan(self):
        polls = capture()
        polls[-1]["status"]["angular_rate_rps"] = [1, 0, 0]
        self.assertTrue(evaluate(records(polls))["passed"])
        self.assertFalse(evaluate(records(polls), stationary=True)["passed"])
        polls[-1]["status"]["angular_rate_rps"] = [float("nan"), 0, 0]
        self.assertFalse(evaluate(records(polls))["passed"])

    def test_short_partial_or_wrong_response_capture_cannot_pass(self):
        data = records(capture())
        self.assertFalse(evaluate(data, minimum_seconds=11)["passed"])
        self.assertFalse(evaluate(data[:-1])["passed"])
        bad = copy.deepcopy(data)
        bad[-1]["response"] = {"System": "Ok"}
        self.assertFalse(evaluate(bad)["passed"])
        self.assertFalse(evaluate([])["passed"])

    def test_counter_wrap_passes_but_reboot_and_freeze_fail(self):
        polls = capture()
        for index, poll in enumerate(polls):
            poll["acquisition"]["imu_samples"] = (0xfffff000 + index * 7270) & 0xffffffff
        self.assertTrue(evaluate(records(polls))["passed"])
        polls[-1]["acquisition"]["updated_us"] = 1000
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = capture()
        polls[-1]["acquisition"]["imu_samples"] = polls[-2]["acquisition"]["imu_samples"]
        self.assertFalse(evaluate(records(polls))["passed"])


if __name__ == "__main__":
    unittest.main()
