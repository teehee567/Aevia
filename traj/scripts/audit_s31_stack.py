#!/usr/bin/env python3
"""Fail closed on generated-stack regressions for the generic ESP32-S31 target.

This audit measures compiler-emitted static frames for the exact generic RV32F
target used to qualify the ESP32-S31-WROOM-3-N16R16V live path.  It deliberately
uses a fresh target directory, exact demangled symbol names, and the active Rust
toolchain's llvm-readobj.  Required symbols that disappear or occur more than
once are audit failures rather than silently selecting a stale or ambiguous
entry.

The conservative chains below cover the currently known live entry/callee
relationships reviewed from direct release-object call relocations.  They are
not a whole-program call-graph proof: dependency/runtime frames, indirect
calls, interrupt/RTOS overhead, and final-link effects are outside this check.
The final ESP-IDF link map and an on-hardware task-stack high-water measurement
for the exact linked firmware remain mandatory release evidence.

Run from the repository root with:

    python3 traj/scripts/audit_s31_stack.py

The isolated build uses the equivalent of:

    RUSTC_BOOTSTRAP=1 RUSTFLAGS='-Z emit-stack-sizes' cargo rustc \
        --locked -p aevia-trajectory --lib --no-default-features --release \
        --target riscv32imafc-unknown-none-elf
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass


TARGET = "riscv32imafc-unknown-none-elf"
PACKAGE = "aevia-trajectory"
FRAME_LIMIT_BYTES = 16 * 1024
CHAIN_LIMIT_BYTES = 32 * 1024
CLOCK_SEED_CHAIN_LIMIT_BYTES = 28 * 1024


@dataclass(frozen=True)
class RequiredFrame:
    key: str
    label: str
    symbol: str
    ceiling: int = FRAME_LIMIT_BYTES


REQUIRED_FRAMES = (
    RequiredFrame(
        "preflight",
        "LiveBuilder::preflight",
        "<aevia_trajectory::engine::live::LiveBuilder>::preflight",
    ),
    RequiredFrame(
        "start",
        "LivePlan::start",
        "<aevia_trajectory::engine::live::LivePlan>::start",
    ),
    RequiredFrame(
        "make_initializer",
        "engine::make_initializer",
        "aevia_trajectory::engine::live::configure::make_initializer",
    ),
    RequiredFrame(
        "initializer_new",
        "Initializer::new",
        "<aevia_trajectory::live::initializer::Initializer>::new",
    ),
    RequiredFrame(
        "make_consider_covariance",
        "engine::make_consider_covariance",
        "aevia_trajectory::engine::live::configure::make_consider_covariance",
    ),
    RequiredFrame(
        "tracker_configure",
        "LiveMetricTracker::configure",
        "<aevia_trajectory::metric::live_tracker::LiveMetricTracker>::configure",
    ),
    RequiredFrame(
        "step",
        "LiveSession::step",
        "<aevia_trajectory::engine::live::LiveSession>::step",
    ),
    RequiredFrame(
        "finish",
        "LiveSession::finish",
        "<aevia_trajectory::engine::live::LiveSession>::finish",
    ),
    RequiredFrame(
        "ingest_observation",
        "LiveSession::ingest_observation",
        "<aevia_trajectory::engine::live::LiveSession>::ingest_observation",
    ),
    RequiredFrame(
        "engine_ingest_imu",
        "LiveSession::ingest_imu",
        "<aevia_trajectory::engine::live::LiveSession>::ingest_imu",
    ),
    RequiredFrame(
        "engine_ingest_gnss",
        "LiveSession::ingest_gnss",
        "<aevia_trajectory::engine::live::LiveSession>::ingest_gnss",
    ),
    RequiredFrame(
        "engine_clock_transition",
        "LiveSession::ingest_clock_transition",
        "<aevia_trajectory::engine::live::LiveSession>::ingest_clock_transition",
    ),
    RequiredFrame(
        "engine_commit_clock_transition",
        "LiveSession::commit_clock_transition",
        "<aevia_trajectory::engine::live::LiveSession>::commit_clock_transition",
    ),
    RequiredFrame(
        "drain_work",
        "LiveSession::drain_work",
        "<aevia_trajectory::engine::live::LiveSession>::drain_work",
    ),
    RequiredFrame(
        "transfer_corrected",
        "LiveSession::transfer_corrected_segments",
        "<aevia_trajectory::engine::live::LiveSession>::transfer_corrected_segments",
    ),
    RequiredFrame(
        "maybe_reanchor",
        "LiveSession::maybe_reanchor",
        "<aevia_trajectory::engine::live::LiveSession>::maybe_reanchor",
    ),
    RequiredFrame(
        "initialization",
        "LiveSession::ingest_initialization_imu",
        "<aevia_trajectory::engine::live::LiveSession>::ingest_initialization_imu",
    ),
    RequiredFrame(
        "core_initialize",
        "LiveCoreState::initialize",
        "<aevia_trajectory::live::core::LiveCoreState>::initialize",
    ),
    RequiredFrame(
        "core_ingest",
        "LiveCore::ingest",
        "<aevia_trajectory::live::core::LiveCore>::ingest",
    ),
    RequiredFrame(
        "core_ingest_imu",
        "LiveCore::ingest_imu",
        "<aevia_trajectory::live::core::LiveCore>::ingest_imu",
    ),
    RequiredFrame(
        "stage_predictor",
        "core::stage_predictor_interval",
        "aevia_trajectory::live::core::ingestion::stage_predictor_interval",
    ),
    RequiredFrame(
        "preintegrator_push",
        "Preintegrator::push",
        "<aevia_trajectory::live::preintegration::Preintegrator>::push",
    ),
    RequiredFrame(
        "predictor_propagate",
        "OutputPredictor::propagate",
        "<aevia_trajectory::live::predictor::OutputPredictor>::propagate",
    ),
    RequiredFrame(
        "corrected_batch",
        "PreintegratedBatch::corrected",
        "<aevia_trajectory::live::preintegration::PreintegratedBatch>::corrected",
    ),
    RequiredFrame(
        "core_drain",
        "LiveCore::drain",
        "<aevia_trajectory::live::core::LiveCore>::drain",
    ),
    RequiredFrame(
        "core_propagate",
        "LiveCore::propagate_to",
        "<aevia_trajectory::live::core::LiveCore>::propagate_to",
    ),
    RequiredFrame(
        "core_fuse",
        "LiveCore::fuse_next_measurement",
        "<aevia_trajectory::live::core::LiveCore>::fuse_next_measurement",
    ),
    RequiredFrame(
        "core_reanchor",
        "LiveCore::reanchor",
        "<aevia_trajectory::live::core::LiveCore>::reanchor",
    ),
    RequiredFrame(
        "core_clock_transition",
        "LiveCore::transition_clock_consider",
        "<aevia_trajectory::live::core::LiveCore>::transition_clock_consider",
    ),
    RequiredFrame(
        "core_finish",
        "LiveCore::finish",
        "<aevia_trajectory::live::core::LiveCore>::finish",
    ),
    RequiredFrame(
        "eskf_initialize",
        "Eskf::initialize",
        "<aevia_trajectory::live::eskf::Eskf>::initialize",
    ),
    RequiredFrame(
        "condition_covariance",
        "Eskf::condition_covariance",
        "<aevia_trajectory::live::eskf::Eskf>::condition_covariance",
    ),
    RequiredFrame(
        "eskf_propagate",
        "Eskf::propagate_with_imu_sample",
        "<aevia_trajectory::live::eskf::Eskf>::propagate_with_imu_sample",
    ),
    RequiredFrame(
        "state_transition",
        "eskf::state_transition_into",
        "aevia_trajectory::live::eskf::discretization::state_transition_into",
    ),
    RequiredFrame(
        "map_preintegration_covariance",
        "eskf::add_mapped_preintegration_covariance",
        "aevia_trajectory::live::eskf::propagation::add_mapped_preintegration_covariance",
    ),
    RequiredFrame(
        "bias_random_walk_discrete_covariance",
        "eskf::add_bias_random_walk_discrete_covariance",
        "aevia_trajectory::live::eskf::discretization::add_bias_random_walk_discrete_covariance",
    ),
    RequiredFrame(
        "process_consider_sensitivity",
        "eskf::process_consider_sensitivity",
        "aevia_trajectory::live::eskf::propagation::process_consider_sensitivity",
    ),
    RequiredFrame(
        "eskf_update_gnss",
        "Eskf::update_gnss_with_imu_sample",
        "<aevia_trajectory::live::eskf::Eskf>::update_gnss_with_imu_sample",
    ),
    RequiredFrame(
        "gnss_linearization",
        "Eskf::gnss_linearization",
        "<aevia_trajectory::live::eskf::Eskf>::gnss_linearization",
    ),
    RequiredFrame(
        "linear_update",
        "Eskf::linear_update",
        "<aevia_trajectory::live::eskf::Eskf>::linear_update",
    ),
    RequiredFrame(
        "eskf_clock_transition",
        "Eskf::transition_clock_consider_into",
        "<aevia_trajectory::live::eskf::Eskf>::transition_clock_consider_into",
    ),
    RequiredFrame(
        "transition_consider_covariance_into",
        "eskf::transition_consider_covariance_into",
        "aevia_trajectory::live::eskf::clock::transition_consider_covariance_into",
    ),
    RequiredFrame(
        "independent_clock_consider_covariance_into",
        "eskf::independent_clock_consider_covariance_into",
        "aevia_trajectory::live::eskf::clock::independent_clock_consider_covariance_into",
    ),
    RequiredFrame(
        "active_principal_block_is_psd",
        "eskf::active_principal_block_is_psd",
        "aevia_trajectory::live::eskf::covariance::active_principal_block_is_psd::<32>",
    ),
    RequiredFrame(
        "map_filter",
        "ReanchorTransform::map_filter_into",
        "<aevia_trajectory::live::reanchor::ReanchorTransform>::map_filter_into",
    ),
    RequiredFrame(
        "metric_update",
        "LiveSession::refresh_metrics",
        "<aevia_trajectory::engine::live::LiveSession>::refresh_metrics",
    ),
    RequiredFrame(
        "metric_update_inner",
        "LiveMetricTracker::update_into",
        "<aevia_trajectory::metric::live_tracker::LiveMetricTracker>::update_into",
    ),
    RequiredFrame(
        "activity_extrema",
        "metric::activity_extrema",
        "aevia_trajectory::metric::activity::activity_extrema",
    ),
    RequiredFrame(
        "speed_roots",
        "Trajectory::speed_roots_with_budget",
        "<aevia_trajectory::trajectory::Trajectory>::speed_roots_with_budget",
    ),
    RequiredFrame(
        "speed_root_oracle",
        "trajectory::evaluate_root_oracle(speed)",
        "aevia_trajectory::trajectory::roots::evaluate_root_oracle::"
        "<<aevia_trajectory::trajectory::Trajectory>::speed_roots_with_budget::{closure#0}>",
    ),
    RequiredFrame(
        "speed_root_refinement",
        "trajectory::refine_enclosed_bracket(speed)",
        "aevia_trajectory::trajectory::roots::refine_enclosed_bracket::"
        "<<aevia_trajectory::trajectory::Trajectory>::speed_roots_with_budget::{closure#0}>",
    ),
    RequiredFrame(
        "speed_extrema",
        "Trajectory::speed_extrema_parameters_with_budget",
        "<aevia_trajectory::trajectory::Trajectory>::"
        "speed_extrema_parameters_with_budget",
    ),
    RequiredFrame(
        "speed_extrema_oracle",
        "trajectory::evaluate_root_oracle(speed extrema)",
        "aevia_trajectory::trajectory::roots::evaluate_root_oracle::"
        "<<aevia_trajectory::trajectory::Trajectory>::"
        "speed_extrema_parameters_with_budget::{closure#0}>",
    ),
    RequiredFrame(
        "speed_extrema_refinement",
        "trajectory::refine_enclosed_bracket(speed extrema)",
        "aevia_trajectory::trajectory::roots::refine_enclosed_bracket::"
        "<<aevia_trajectory::trajectory::Trajectory>::"
        "speed_extrema_parameters_with_budget::{closure#0}>",
    ),
    RequiredFrame(
        "projection",
        "LiveSession::present_projection",
        "<aevia_trajectory::engine::live::LiveSession>::present_projection",
    ),
    RequiredFrame(
        "project_nav_state",
        "engine::project_nav_state",
        "aevia_trajectory::engine::live::conversion::project_nav_state",
    ),
)

# These sums intentionally over-approximate sequential metric/projection work.
# Each tuple is (report label, required-frame keys).
KNOWN_CHAINS = (
    (
        "preflight -> initializer",
        ("preflight", "make_initializer", "initializer_new"),
    ),
    ("start -> metric configure", ("start", "tracker_configure")),
    ("step -> drain controller", ("step", "drain_work", "core_drain")),
    (
        "step -> drain -> filter propagation/state transition",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "state_transition",
        ),
    ),
    (
        "step -> drain -> filter propagation/covariance map",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "map_preintegration_covariance",
        ),
    ),
    (
        "step -> drain -> filter propagation/bias random walk",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "map_preintegration_covariance",
            "bias_random_walk_discrete_covariance",
        ),
    ),
    (
        "step -> drain -> filter propagation/consider sensitivity",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "process_consider_sensitivity",
        ),
    ),
    (
        "step -> drain -> filter propagation/corrected batch",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "corrected_batch",
        ),
    ),
    (
        "step -> drain -> filter propagation/conditioning",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_propagate",
            "eskf_propagate",
            "condition_covariance",
        ),
    ),
    (
        "step -> drain -> GNSS linearization",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_fuse",
            "eskf_update_gnss",
            "gnss_linearization",
        ),
    ),
    (
        "step -> drain -> GNSS linear update/conditioning",
        (
            "step",
            "drain_work",
            "core_drain",
            "core_fuse",
            "eskf_update_gnss",
            "linear_update",
            "condition_covariance",
        ),
    ),
    (
        "step -> drain -> corrected transfer",
        ("step", "drain_work", "transfer_corrected"),
    ),
    (
        "step -> drain -> reanchor/filter map",
        (
            "step",
            "drain_work",
            "maybe_reanchor",
            "core_reanchor",
            "map_filter",
        ),
    ),
    (
        "step -> initialization -> core -> ESKF covariance",
        (
            "step",
            "ingest_observation",
            "engine_ingest_imu",
            "initialization",
            "core_initialize",
            "eskf_initialize",
            "condition_covariance",
        ),
    ),
    (
        "step -> initialization -> first IMU/preintegration",
        (
            "step",
            "ingest_observation",
            "engine_ingest_imu",
            "initialization",
            "core_ingest_imu",
            "stage_predictor",
            "preintegrator_push",
        ),
    ),
    (
        "step -> initialization -> first IMU/predictor",
        (
            "step",
            "ingest_observation",
            "engine_ingest_imu",
            "initialization",
            "core_ingest_imu",
            "stage_predictor",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "step -> steady IMU/preintegration",
        (
            "step",
            "ingest_observation",
            "engine_ingest_imu",
            "core_ingest",
            "core_ingest_imu",
            "stage_predictor",
            "preintegrator_push",
        ),
    ),
    (
        "step -> steady IMU/predictor",
        (
            "step",
            "ingest_observation",
            "engine_ingest_imu",
            "core_ingest",
            "core_ingest_imu",
            "stage_predictor",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "step -> clock transition/covariance",
        (
            "step",
            "ingest_observation",
            "engine_clock_transition",
            "engine_commit_clock_transition",
            "core_clock_transition",
            "eskf_clock_transition",
            "transition_consider_covariance_into",
            "active_principal_block_is_psd",
        ),
    ),
    (
        "step -> clock transition/seed covariance",
        (
            "step",
            "ingest_observation",
            "engine_clock_transition",
            "engine_commit_clock_transition",
            "transition_consider_covariance_into",
            "active_principal_block_is_psd",
        ),
    ),
    (
        "step -> clock transition/independent seed covariance",
        (
            "step",
            "ingest_observation",
            "engine_clock_transition",
            "engine_commit_clock_transition",
            "independent_clock_consider_covariance_into",
            "active_principal_block_is_psd",
        ),
    ),
    (
        "step -> metric update/projection",
        (
            "step",
            "metric_update",
            "metric_update_inner",
            "activity_extrema",
            "speed_roots",
            "speed_root_refinement",
            "speed_root_oracle",
            "projection",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "step -> metric extrema/projection",
        (
            "step",
            "metric_update",
            "metric_update_inner",
            "activity_extrema",
            "speed_extrema",
            "speed_extrema_refinement",
            "speed_extrema_oracle",
            "projection",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "finish -> metric update/projection",
        (
            "finish",
            "metric_update",
            "metric_update_inner",
            "activity_extrema",
            "speed_roots",
            "speed_root_refinement",
            "speed_root_oracle",
            "projection",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "finish -> metric extrema/projection",
        (
            "finish",
            "metric_update",
            "metric_update_inner",
            "activity_extrema",
            "speed_extrema",
            "speed_extrema_refinement",
            "speed_extrema_oracle",
            "projection",
            "predictor_propagate",
            "corrected_batch",
        ),
    ),
    (
        "finish -> core predictor flush",
        ("finish", "core_finish", "predictor_propagate", "corrected_batch"),
    ),
)

CLOCK_SEED_CHAINS = frozenset(
    {
        "step -> clock transition/seed covariance",
        "step -> clock transition/independent seed covariance",
    }
)


class AuditSetupError(RuntimeError):
    """The audit could not produce trustworthy stack-size input."""


LLVM_CLONE_SUFFIX = re.compile(r" \(\.llvm\.[0-9]+\)$")


def canonical_symbol(symbol: str) -> str:
    """Removes only LLVM's unstable numeric clone suffix after demangling."""

    return LLVM_CLONE_SUFFIX.sub("", symbol)


def run_checked(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    description: str,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        output = "\n".join(part for part in (result.stdout, result.stderr) if part)
        raise AuditSetupError(f"{description} failed:\n{output.rstrip()}")
    return result


def rust_toolchain(repo_root: Path) -> tuple[Path, str]:
    verbose = run_checked(
        ["rustc", "-vV"], cwd=repo_root, description="rustc toolchain query"
    ).stdout
    fields = {}
    for line in verbose.splitlines():
        key, separator, value = line.partition(": ")
        if separator:
            fields[key] = value
    host = fields.get("host")
    if host is None:
        raise AuditSetupError("rustc -vV did not report a host triple")
    sysroot = Path(
        run_checked(
            ["rustc", "--print", "sysroot"],
            cwd=repo_root,
            description="rustc sysroot query",
        ).stdout.strip()
    )
    readobj = sysroot / "lib" / "rustlib" / host / "bin" / "llvm-readobj"
    if not readobj.is_file() or not os.access(readobj, os.X_OK):
        raise AuditSetupError(
            f"toolchain llvm-readobj is unavailable at {readobj}; "
            "install it with `rustup component add llvm-tools`"
        )
    identity = " ".join(
        value
        for value in (
            fields.get("release"),
            fields.get("commit-hash"),
            f"LLVM-{fields['LLVM version']}" if "LLVM version" in fields else None,
        )
        if value is not None
    )
    return readobj, identity


def build_rlib(repo_root: Path, build_root: Path) -> Path:
    environment = os.environ.copy()
    environment.pop("CARGO_ENCODED_RUSTFLAGS", None)
    environment["CARGO_INCREMENTAL"] = "0"
    environment["CARGO_TARGET_DIR"] = str(build_root)
    environment["CARGO_TERM_COLOR"] = "never"
    environment["RUSTC_BOOTSTRAP"] = "1"
    environment["RUSTFLAGS"] = "-Z emit-stack-sizes"
    run_checked(
        [
            "cargo",
            "rustc",
            "--quiet",
            "--locked",
            "-p",
            PACKAGE,
            "--lib",
            "--no-default-features",
            "--release",
            "--target",
            TARGET,
        ],
        cwd=repo_root,
        env=environment,
        description="RV32F release build with compiler-emitted stack sizes",
    )
    dependency_dir = build_root / TARGET / "release" / "deps"
    artifacts = sorted(dependency_dir.glob("libaevia_trajectory-*.rlib"))
    if len(artifacts) != 1:
        listed = ", ".join(str(path) for path in artifacts) or "none"
        raise AuditSetupError(
            "expected exactly one freshly built aevia-trajectory rlib, "
            f"found {len(artifacts)}: {listed}"
        )
    return artifacts[0]


def emitted_frames(
    repo_root: Path, readobj: Path, artifact: Path
) -> dict[str, list[int]]:
    result = run_checked(
        [
            str(readobj),
            "--elf-output-style=JSON",
            "--stack-sizes",
            "--demangle",
            str(artifact),
        ],
        cwd=repo_root,
        description="llvm-readobj stack-size extraction",
    )
    try:
        documents = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise AuditSetupError(f"llvm-readobj returned invalid JSON: {error}") from error

    frames: dict[str, list[int]] = {}
    saw_target_object = False
    for document in documents:
        summary = document.get("FileSummary", {})
        entries = document.get("StackSizes", [])
        if entries:
            if summary.get("Arch") != "riscv32" or not str(
                summary.get("Format", "")
            ).startswith("elf32-littleriscv"):
                raise AuditSetupError(
                    "stack sizes came from an unexpected object: "
                    f"{summary.get('File', '<unknown>')} "
                    f"({summary.get('Format')}, {summary.get('Arch')})"
                )
            saw_target_object = True
        for item in entries:
            entry = item.get("Entry", {})
            size = entry.get("Size")
            functions = entry.get("Functions")
            if not isinstance(size, int) or not isinstance(functions, list):
                raise AuditSetupError("malformed llvm-readobj stack-size entry")
            for function in functions:
                if not isinstance(function, str):
                    raise AuditSetupError("non-string function in stack-size entry")
                frames.setdefault(canonical_symbol(function), []).append(size)
    if not saw_target_object:
        raise AuditSetupError(
            "llvm-readobj found no populated .stack_sizes section for the RV32F object"
        )
    return frames


def print_report(
    toolchain: str, frames: dict[str, list[int]]
) -> tuple[bool, list[str]]:
    resolved: dict[str, int] = {}
    failures: list[str] = []
    rows: list[tuple[str, str, str]] = []
    for requirement in REQUIRED_FRAMES:
        occurrences = frames.get(requirement.symbol, [])
        if not occurrences:
            rows.append((requirement.label, "missing", "FAIL"))
            failures.append(f"missing required symbol: {requirement.symbol}")
            continue
        if len(occurrences) != 1:
            measured = ",".join(str(size) for size in occurrences)
            rows.append((requirement.label, f"duplicate({measured})", "FAIL"))
            failures.append(
                f"duplicate required symbol ({len(occurrences)} entries): "
                f"{requirement.symbol}"
            )
            continue
        frame = occurrences[0]
        resolved[requirement.key] = frame
        status = "PASS" if frame <= requirement.ceiling else "FAIL"
        rows.append((requirement.label, f"{frame}/{requirement.ceiling}", status))
        if frame > requirement.ceiling:
            failures.append(
                f"{requirement.label} frame {frame} exceeds {requirement.ceiling} bytes"
            )

    print(f"ESP32-S31 generated-stack audit ({TARGET})")
    print(f"toolchain: {toolchain}")
    print("frames (measured/ceiling bytes):")
    width = max(len(row[0]) for row in rows)
    for label, measurement, status in rows:
        print(f"  {label:<{width}}  {measurement:>15}  {status}")

    print("chains (conservative sum/ceiling bytes):")
    chain_rows = []
    for label, keys in KNOWN_CHAINS:
        if any(key not in resolved for key in keys):
            chain_rows.append((label, "unresolved", "FAIL"))
            failures.append(f"cannot resolve required chain: {label}")
            continue
        total = sum(resolved[key] for key in keys)
        ceiling = (
            CLOCK_SEED_CHAIN_LIMIT_BYTES
            if label in CLOCK_SEED_CHAINS
            else CHAIN_LIMIT_BYTES
        )
        status = "PASS" if total <= ceiling else "FAIL"
        chain_rows.append((label, f"{total}/{ceiling}", status))
        if total > ceiling:
            failures.append(
                f"{label} conservative frame sum {total} exceeds "
                f"{ceiling} bytes"
            )
    chain_width = max(len(row[0]) for row in chain_rows)
    for label, measurement, status in chain_rows:
        print(f"  {label:<{chain_width}}  {measurement:>15}  {status}")

    print(
        "proof boundary: generated generic-target frames are a CI check; "
        "dependency/indirect calls, interrupt/RTOS overhead, and final-link "
        "effects are not a whole-program stack proof."
    )
    print(
        "final proof: review the ESP-IDF link map and measure the task-stack "
        "high-water mark on the exact linked firmware and hardware."
    )
    return not failures, failures


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    try:
        readobj, toolchain = rust_toolchain(repo_root)
        with tempfile.TemporaryDirectory(prefix="aevia-s31-stack-") as directory:
            artifact = build_rlib(repo_root, Path(directory))
            frames = emitted_frames(repo_root, readobj, artifact)
        passed, failures = print_report(toolchain, frames)
    except (AuditSetupError, OSError) as error:
        print(f"ESP32-S31 generated-stack audit ERROR: {error}", file=sys.stderr)
        return 2

    if passed:
        print("result: PASS")
        return 0
    print("result: FAIL")
    for failure in failures:
        print(f"  - {failure}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
