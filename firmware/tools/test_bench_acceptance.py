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
                            "imu_samples": 40000 + index * 3635, "imu_errors": 3,
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


def navigating_capture():
    polls = capture()
    for index, poll in enumerate(polls):
        poll["status"].update({
            "phase": "Navigating", "epoch_us": poll["status"]["updated_us"] - 120000,
            "imu_accepted": 1000 + index * 3635, "gnss_fused": 50 + index * 250,
            "input_rejected": 1, "too_late": 2,
            "horizontal_speed_mps": 0.02, "vertical_speed_mps": -0.01,
            "position_ecef_m": [-4645000.0, 2553000.0, -3534000.0],
            "velocity_ecef_mps": [0.01, -0.01, 0.01],
        })
        poll["acquisition"]["timing"].update({
            "pps_locked": True, "pps_edges": 20 + index * 5,
            "pps_pairs": 18 + index * 5, "pps_rejected": 2,
            "pps_age_ms": 1000, "fit_residual_ns": 2000,
            "generation": 2, "gnss_unsynchronized": 70,
        })
        poll["gnss"].update({"navigation_time_ready": True, "last_pps_quality": 2})
    return polls


def add_fix(polls):
    for index, poll in enumerate(polls):
        poll["fix"] = {
            "qualified": False, "position_valid": True, "velocity_valid": True,
            "received_age_us": 1000, "position_age_us": 11000, "velocity_age_us": 31000,
            "position_ecef_m": [-4645000.0, 2553000.0, -3534000.0],
            "velocity_ecef_mps": [0.01, -0.01, 0.01],
            "gps_epoch_ms": 1470000000000 + index * 5000,
        }
    return polls


def add_charge(polls):
    for poll in polls:
        poll["power"]["control"] = 0xA0
        poll["power_charge"] = {
            "updated_us": poll["acquisition"]["updated_us"] - 100000,
            "verified": True, "charging_enabled": True,
            "charge_current_ma": 800, "charge_voltage_mv": 4100,
            "input_current_limit_ma": 500, "charger_status0": 0,
            "charger_status1": 0, "fault_status": 0,
            "precharge_current_ma": 80, "termination_current_ma": 40,
            "charger_control0": 0, "charger_control1": 0xA0, "ntc_control0": 0,
        }
    return polls


class BenchAcceptanceTests(unittest.TestCase):
    def test_verified_charge_profile_accepts_idle_full_battery(self):
        report = evaluate(records(add_charge(capture())))
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["profile"], "charging acquisition bench")
        self.assertEqual(report["power_charge"]["polls"], 3)
        self.assertEqual(report["power_charge"]["final"]["charger_status1"], 0)
        self.assertTrue(evaluate(records(add_charge(navigating_capture())), navigation=True)["passed"])

    def test_charge_profile_rejects_missing_stale_or_inconsistent_readbacks(self):
        changes = (("verified", False), ("charging_enabled", False),
                   ("charge_current_ma", 480), ("charge_current_ma", None),
                   ("charge_voltage_mv", 4200), ("charge_voltage_mv", None),
                   ("precharge_current_ma", 120), ("termination_current_ma", 80),
                   ("charger_control1", 0x80), ("charger_control1", None),
                   ("fault_status", 1), ("fault_status", None),
                   ("updated_us", None), ("updated_us", 1),
                   ("updated_us", 99_000_000))
        for field, value in changes:
            with self.subTest(field=field, value=value):
                polls = add_charge(capture())
                polls[-1]["power_charge"][field] = value
                self.assertFalse(evaluate(records(polls))["passed"])
        for control in (None, 0x80, 0xA1, 0xA4):
            polls = add_charge(capture())
            polls[-1]["power"]["control"] = control
            self.assertFalse(evaluate(records(polls))["passed"])
        polls = add_charge(capture())
        polls[-1]["power"]["verified"] = False
        self.assertFalse(evaluate(records(polls))["passed"])

    def test_partial_charge_diagnostics_fail_and_absent_evidence_keeps_old_gate(self):
        polls = add_charge(capture())
        del polls[-1]["power_charge"]
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = add_charge(capture())
        for poll in polls:
            del poll["power_charge"]
        self.assertFalse(evaluate(records(polls))["passed"])
        self.assertTrue(evaluate(records(capture()))["passed"])
        polls = add_charge(capture())
        polls[-1]["power_charge"] = None
        self.assertFalse(evaluate(records(polls))["passed"])

    def test_charge_evidence_does_not_relax_acquisition_or_error_gates(self):
        polls = add_charge(capture())
        polls[-1]["acquisition"]["imu_errors"] += 1
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = add_charge(capture())
        polls[-1]["power"]["errors"] += 1
        self.assertFalse(evaluate(records(polls))["passed"])

    def test_optional_fix_is_retained_and_adds_navigation_validity_gates(self):
        polls = add_fix(navigating_capture())
        report = evaluate(records(polls), navigation=True)
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["receiver_fix"]["polls"], 3)
        self.assertEqual(report["receiver_fix"]["final"], polls[-1]["fix"])
        for field, value in (("qualified", True), ("position_valid", False),
                             ("velocity_valid", False), ("received_age_us", 500001),
                             ("position_age_us", None), ("velocity_age_us", -1),
                             ("position_ecef_m", None), ("velocity_ecef_mps", [float("nan"), 0, 0])):
            with self.subTest(field=field):
                polls = add_fix(navigating_capture())
                polls[-1]["fix"][field] = value
                self.assertFalse(evaluate(records(polls), navigation=True)["passed"])
                # Raw receiver evidence remains diagnostic-only for acquisition.
                self.assertTrue(evaluate(records(polls))["passed"])

    def test_partial_or_wrong_fix_diagnostics_cannot_pass_navigation(self):
        polls = add_fix(navigating_capture())
        del polls[-1]["fix"]
        self.assertFalse(evaluate(records(polls), navigation=True)["passed"])
        polls = add_fix(navigating_capture())
        data = records(polls)
        data[-1]["response"] = {"Device": {"GnssStatus": {}}}
        self.assertFalse(evaluate(data, navigation=True)["passed"])

    def test_navigation_requires_a_real_workload_and_keeps_qualification_false(self):
        self.assertFalse(evaluate(records(capture()), navigation=True)["passed"])
        report = evaluate(records(navigating_capture()), navigation=True, stationary=True)
        self.assertTrue(report["passed"], report["failures"])
        self.assertFalse(report["qualified"])
        self.assertEqual(report["profile"], "battery-free navigation bench")
        self.assertEqual(report["navigation"]["counter_deltas"]["gnss_fused"], 500)
        self.assertEqual(report["navigation"]["epoch_age_us"]["maximum"], 120000)

    def test_navigation_does_not_accept_lock_loss_bad_output_or_clock_resets(self):
        changes = (("status", "phase", "Degraded"), ("status", "epoch_us", None),
                   ("status", "epoch_us", 0), ("status", "epoch_us", 99_000_000),
                   ("status", "horizontal_speed_mps", None),
                   ("status", "horizontal_speed_mps", -1),
                   ("status", "vertical_speed_mps", float("nan")),
                   ("status", "position_ecef_m", [1, 2]),
                   ("status", "velocity_ecef_mps", [1, float("inf"), 3]),
                   ("status", "input_rejected", 2), ("status", "too_late", 3),
                   ("gnss", "navigation_time_ready", False), ("gnss", "last_pps_quality", 1),
                   ("timing", "pps_locked", False), ("timing", "pps_age_ms", 3001),
                   ("timing", "pps_age_ms", None), ("timing", "fit_residual_ns", 50001),
                   ("timing", "pps_rejected", 3), ("timing", "generation", 3),
                   ("timing", "gnss_unsynchronized", 71))
        for section, field, value in changes:
            with self.subTest(field=field, value=value):
                polls = navigating_capture()
                target = polls[-1]["acquisition"]["timing"] if section == "timing" else polls[-1][section]
                target[field] = value
                self.assertFalse(evaluate(records(polls), navigation=True)["passed"])

    def test_navigation_requires_progress_from_estimator_and_pps(self):
        for field in ("epoch_us", "imu_accepted", "gnss_fused"):
            with self.subTest(field=field):
                polls = navigating_capture()
                polls[-1]["status"][field] = polls[-2]["status"][field]
                self.assertFalse(evaluate(records(polls), navigation=True)["passed"])
                del polls[-1]["status"][field]
                self.assertFalse(evaluate(records(polls), navigation=True)["passed"])
        for field in ("pps_edges", "pps_pairs"):
            polls = navigating_capture()
            for poll in polls:
                poll["acquisition"]["timing"][field] = 20
            self.assertFalse(evaluate(records(polls), navigation=True)["passed"])
            for index, poll in enumerate(polls):
                poll["acquisition"]["timing"][field] = (0xfffffffe + index * 5) & 0xffffffff
            self.assertTrue(evaluate(records(polls), navigation=True)["passed"])

    def test_navigation_optional_fields_missing_cannot_crash_or_pass(self):
        for section, field in (("status", "position_ecef_m"), ("status", "too_late"),
                               ("status", "imu_accepted"), ("gnss", "last_pps_quality"),
                               ("timing", "pps_pairs"), ("timing", "generation")):
            with self.subTest(field=field):
                polls = navigating_capture()
                target = polls[-1]["acquisition"]["timing"] if section == "timing" else polls[-1][section]
                del target[field]
                self.assertFalse(evaluate(records(polls), navigation=True)["passed"])

    def test_absent_accessories_and_no_fix_pass_with_settled_error_baselines(self):
        report = evaluate(records(capture()), minimum_seconds=10, stationary=True)
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual(report["duration_seconds"], 10)
        self.assertEqual(report["imu_rate_hz"]["minimum"], 727)
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
        for limits in ((float("nan"), 675, 800), (0, float("inf"), 800),
                       (0, 675, float("inf")), (0, 800, 675), (-1, 675, 800)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                evaluate([], *limits)

    def test_dec5_rate_contract_rejects_old_profile_and_actual_losses(self):
        for rate in (674, 801, 1454):
            polls = capture()
            for index, poll in enumerate(polls):
                poll["acquisition"]["imu_samples"] = 40000 + index * rate * 5
            self.assertFalse(evaluate(records(polls))["passed"])
        polls = capture()
        polls[-1]["acquisition"]["timing"]["imu_timing_rejected"] += 1
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = capture()
        polls[-1]["acquisition"]["imu_drops"] += 1
        self.assertFalse(evaluate(records(polls))["passed"])

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
            poll["acquisition"]["imu_samples"] = (0xfffff000 + index * 3635) & 0xffffffff
        self.assertTrue(evaluate(records(polls))["passed"])
        polls[-1]["acquisition"]["updated_us"] = 1000
        self.assertFalse(evaluate(records(polls))["passed"])
        polls = capture()
        polls[-1]["acquisition"]["imu_samples"] = polls[-2]["acquisition"]["imu_samples"]
        self.assertFalse(evaluate(records(polls))["passed"])


if __name__ == "__main__":
    unittest.main()
