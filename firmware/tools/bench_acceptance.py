#!/usr/bin/env python3
"""Evaluate retained board_check frames without accessing hardware.

These are functional bench gates, not sensor/timing accuracy qualification.
Missing GNSS lock, a panel, buttons and a battery are permitted by this profile.
"""
import argparse
import json
import math
from pathlib import Path


RESPONSES = {
    "boot": ("System", "BootStatus"),
    "status": ("Device", "Snapshot"),
    "power": ("Device", "PowerStatus"),
    "peripherals": ("Device", "PeripheralStatus"),
    "acquisition": ("Device", "AcquisitionStatus"),
    "gnss": ("Device", "GnssStatus"),
    "imu": ("Device", "ImuDiagnostics"),
    "reset": ("Device", "ResetStatus"),
}
OPTIONAL_RESPONSES = {"transport": ("Device", "GnssTransportStatus"),
                      "power_diagnostics": ("Device", "PowerDiagnostics")}
ALL_RESPONSES = RESPONSES | OPTIONAL_RESPONSES


def validate_limits(minimum_seconds, minimum_imu_hz, maximum_imu_hz):
    if not all(math.isfinite(value) for value in
               (minimum_seconds, minimum_imu_hz, maximum_imu_hz)):
        raise ValueError("duration and rate limits must be finite")
    if minimum_seconds < 0 or not 0 < minimum_imu_hz < maximum_imu_hz:
        raise ValueError("duration must be nonnegative and IMU rate limits positive and ordered")


def evaluate(records, minimum_seconds=0, minimum_imu_hz=1350,
             maximum_imu_hz=1600, stationary=False):
    """Return a machine-readable verdict; counter baselines exclude startup.

    All diagnostics in a complete poll must have their expected response type.
    Counter wrap is accepted only for the raw u32 sample count. Error counters
    must remain unchanged. Reboot/frozen clocks cannot pass sample-rate gates.
    """
    validate_limits(minimum_seconds, minimum_imu_hz, maximum_imu_hz)
    failures = []
    polls = []
    current = None
    required = set(RESPONSES)

    def require(condition, message):
        if not condition and message not in failures:
            failures.append(message)

    for record in records:
        name = record["query"]
        if name == "boot":
            if current is not None:
                require(required <= set(current), "incomplete diagnostic poll")
            current = {}
            polls.append(current)
        if name in ALL_RESPONSES:
            if current is None:
                require(False, "diagnostics before first boot poll")
                continue
            outer, inner = ALL_RESPONSES[name]
            try:
                current[name] = record["response"][outer][inner]
            except (KeyError, TypeError):
                require(False, f"{name}: unexpected response type")
    if current is not None:
        require(required <= set(current), "incomplete diagnostic poll")
    complete = [poll for poll in polls if required <= set(poll)]
    require(len(complete) >= 2, "at least two complete diagnostic polls required")
    report = {"passed": False, "profile": "battery-free acquisition bench",
              "qualified": False, "stationary": stationary,
              "polls": len(complete), "failures": failures,
              "limitations": ["No GNSS/PPS lock or navigation workload requirement",
                              "No display, button, battery, SD or radio qualification",
                              "No electrical timing, calibration or accuracy qualification"]}
    if len(complete) < 2:
        return report

    power_diagnostics = [poll["power_diagnostics"] for poll in complete
                         if "power_diagnostics" in poll]
    if power_diagnostics:
        report["power_diagnostics"] = {"polls": len(power_diagnostics),
                                       "baseline": power_diagnostics[0],
                                       "final": power_diagnostics[-1]}

    transport_polls = [poll["transport"] for poll in complete if "transport" in poll]
    if transport_polls:
        require(len(transport_polls) == len(complete),
                "GNSS transport diagnostics missing from some polls")
        for port in range(2):
            samples = [sample["ports"][port] for sample in transport_polls]
            for field in ("rx_errors", "fifo_overflows", "glitches", "framing_errors",
                          "parity_errors", "other_errors"):
                require(all(sample[field] == samples[0][field] for sample in samples),
                        f"GNSS COM{port + 1} {field} changed during observation")
            require(all(sample["rx_bytes"] >= previous["rx_bytes"]
                        for previous, sample in zip(samples, samples[1:])),
                    f"GNSS COM{port + 1} received-byte counter decreased")
            if port == 1:
                require(all(sample["rx_bytes"] > previous["rx_bytes"]
                            for previous, sample in zip(samples, samples[1:])),
                        "GNSS COM2 received bytes did not advance on every poll")
        report["gnss_transport"] = {
            "polls": len(transport_polls),
            "port_counter_deltas": {
                f"COM{port + 1}": {
                    field: transport_polls[-1]["ports"][port][field] - value
                    for field, value in transport_polls[0]["ports"][port].items()
                } for port in range(2)
            },
            "baseline": transport_polls[0], "final": transport_polls[-1],
        }

    acquisitions = [poll["acquisition"] for poll in complete]
    first, last = acquisitions[0], acquisitions[-1]
    duration = (last["updated_us"] - first["updated_us"]) / 1e6
    require(duration >= minimum_seconds, "observation shorter than required duration")
    rates = []
    for previous, sample in zip(acquisitions, acquisitions[1:]):
        elapsed = sample["updated_us"] - previous["updated_us"]
        require(elapsed > 0, "device uptime stopped or reset during observation")
        # A modular difference permits long soaks across the u32 counter wrap.
        delta = (sample["imu_samples"] - previous["imu_samples"]) & 0xffffffff
        rate = delta * 1e6 / elapsed if elapsed > 0 else 0
        rates.append(rate)
        require(minimum_imu_hz <= rate <= maximum_imu_hz,
                "IMU sample cadence outside configured acceptance range")

    stable_acquisition = ("imu_errors", "imu_drops", "gnss_errors", "gnss_drops",
                          "gnss_uart_errors", "gnss_parse_errors")
    stable_imu = ("attempts", "driver_errors", "timing_errors", "gap_errors",
                  "range_errors", "drdy_timeouts", "spi_timeouts", "numeric_range_errors")
    for field in stable_acquisition:
        require(all(sample[field] == first[field] for sample in acquisitions),
                f"acquisition.{field} changed during observation")
    for field in ("capture_errors", "imu_timing_rejected"):
        require(all(sample["timing"][field] == first["timing"][field]
                    for sample in acquisitions), f"timing.{field} changed during observation")
    for field in stable_imu:
        require(all(poll["imu"][field] == complete[0]["imu"][field] for poll in complete),
                f"imu.{field} changed during observation")
    for field in ("led_errors", "display_errors", "gauge_errors", "buttons_errors"):
        require(all(poll["peripherals"][field] == complete[0]["peripherals"][field]
                    for poll in complete), f"peripherals.{field} changed during observation")
    require(all(poll["power"]["errors"] == complete[0]["power"]["errors"] for poll in complete),
            "power.errors changed during observation")
    require(all(poll["status"]["processing_errors"] == complete[0]["status"]["processing_errors"]
                for poll in complete), "worker.processing_errors changed during observation")
    require(all(poll["gnss"]["config_timeouts"] == complete[0]["gnss"]["config_timeouts"]
                for poll in complete), "GNSS CONFIG readback timed out during observation")
    require(all(poll["reset"] == complete[0]["reset"] for poll in complete),
            "reset reason or retained panic changed during observation")

    acceleration_norms, angular_rate_norms = [], []
    for poll in complete:
        acq, status, imu = poll["acquisition"], poll["status"], poll["imu"]
        require(poll["boot"]["stage"] == 10, "estimator workspace not ready (boot stage != 10)")
        require(status["phase"] not in ("Failed", "Starting"),
                "worker failed or did not finish startup")
        require(not status["qualified"], "development build unexpectedly claims qualification")
        require(status["error_kind"] is None, "worker reports a retained processing failure")
        require(poll["reset"]["panic"] is None, "reset diagnostics report a retained panic")
        require(0 <= acq["updated_us"] - status["updated_us"] <= 500000,
                "worker snapshot stale or its clock inconsistent")
        require(acq["last_imu_us"] is not None and
                -2000 <= acq["updated_us"] - acq["last_imu_us"] <= 100000,
                "last admitted IMU measurement missing or stale")
        require(imu["ready"], "IMU not continuously ready")
        require(imu["cycle_clock_active"], "IMU cycle-clock fallback active")
        require(acq["imu_queue_depth"] < 128 and acq["gnss_queue_depth"] < 16,
                "sensor queue has no capacity margin")
        require(poll["power"]["verified"], "charging-disabled power policy unverified")
        require(poll["power"]["control"] is not None and
                poll["power"]["control"] & 0x27 == 0,
                "charging/watchdog disable control bits not confirmed")
        require(poll["peripherals"]["leds_ready"], "LED controller unavailable")
        require(0 <= acq["updated_us"] - poll["peripherals"]["updated_us"] <= 3000000,
                "peripheral maintenance stale")
        require(poll["gnss"]["config_verified"], "GNSS PPS CONFIG readback unverified")
        for field, norms in (("acceleration_mps2", acceleration_norms),
                             ("angular_rate_rps", angular_rate_norms)):
            values = status[field]
            if len(values) == 3 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
                norms.append(math.sqrt(sum(x * x for x in values)))
            else:
                require(False, f"nonfinite or malformed {field}")
        if stationary and acceleration_norms and angular_rate_norms:
            require(8.8 <= acceleration_norms[-1] <= 10.8,
                    "stationary acceleration magnitude outside 8.8..10.8 m/s2")
            require(angular_rate_norms[-1] <= 0.15,
                    "stationary angular-rate magnitude above 0.15 rad/s")

    def bounds(values):
        return {"minimum": min(values), "maximum": max(values)} if values else None

    report.update({"passed": not failures, "duration_seconds": duration,
                   "device_uptime_seconds": last["updated_us"] / 1e6,
                   "imu_samples_observed": sum((b["imu_samples"] - a["imu_samples"]) & 0xffffffff
                                                for a, b in zip(acquisitions, acquisitions[1:])),
                   "imu_rate_hz": bounds(rates),
                   "acceleration_norm_mps2": bounds(acceleration_norms),
                   "angular_rate_norm_rps": bounds(angular_rate_norms),
                   "counter_deltas": {field: last[field] - first[field] for field in stable_acquisition},
                   "lifetime_max_read_latency_us": last["imu_max_read_latency_us"],
                   "lifetime_queue_high_water": {"imu": last["imu_queue_high_water"],
                                                  "gnss": last["gnss_queue_high_water"]},
                   "baseline_reset": complete[0]["reset"],
                   "final_phase": complete[-1]["status"]["phase"],
                   "final_engine_error": complete[-1]["status"]["error_kind"],
                   "lifetime_maximum_step_us": complete[-1]["status"]["maximum_step_us"],
                   "final_gnss": complete[-1]["gnss"]})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--minimum-seconds", type=float, default=0)
    parser.add_argument("--minimum-imu-hz", type=float, default=1350)
    parser.add_argument("--maximum-imu-hz", type=float, default=1600)
    parser.add_argument("--stationary", action="store_true")
    args = parser.parse_args()
    try:
        validate_limits(args.minimum_seconds, args.minimum_imu_hz, args.maximum_imu_hz)
    except ValueError as error:
        parser.error(str(error))
    report = evaluate(json.loads(args.capture.read_text()), args.minimum_seconds,
                      args.minimum_imu_hz, args.maximum_imu_hz, args.stationary)
    output = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(output)
    print(output, end="")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
