#!/usr/bin/env python3
"""Fail closed on host/runtime leakage into the ESP32-S31 live engine.

The audit builds the allocator-free feature set for the generic RV32IMAFC
target, links minimal ``LiveSession::step`` and ``LiveSession::finish`` roots,
and inspects the resulting garbage-collected ELFs.  It is deliberately a
library integration gate, not a substitute for inspecting the final ESP-IDF
image and measuring it on the fitted ESP32-S31 module.

Run from the repository root with:

    python3 traj/scripts/audit_s31_firmware_purity.py
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


TARGET = "riscv32imafc-unknown-none-elf"
PACKAGE = "aevia-trajectory"
CRATE_STEM = "aevia_trajectory"

FORBIDDEN_DEPENDENCIES = ("rustc_apfloat", "smallvec")
FORBIDDEN_FEATURE = 'feature "soft-float"'

PANIC_TARGETS = (
    "core::panicking::",
    "core::option::expect_failed",
    "core::result::unwrap_failed",
)
FUNCTION_HEADER = re.compile(r"^[0-9A-Fa-f]+ <(.+)>:$")
CALL_TARGET = re.compile(r"<([^>]+)>")
HASH_SUFFIX = re.compile(r"::h[0-9a-f]{16}(?: \(\.llvm\.[0-9]+\))?$")
DOUBLE_HELPER = re.compile(r"^__[A-Za-z0-9_]*df[A-Za-z0-9_]*$")
FP_ARITHMETIC = re.compile(
    r"\b(?:fadd|fsub|fmul|fdiv|fsqrt|fmadd|fmsub|fnmadd|fnmsub)\.([sd])\b"
)
LIBM_FUNCTION = re.compile(r"libm::math::([A-Za-z0-9_]+)::\1(?=::|\s|$)")

FORBIDDEN_SYMBOL_PATTERNS = (
    (
        "allocator",
        re.compile(
            r"(?:__rustc::)?__rust_(?:alloc|alloc_zeroed|realloc|dealloc)|"
            r"__(?:rg|rdl)_(?:alloc|alloc_zeroed|realloc|dealloc)|"
            r"__rust_(?:alloc_error_handler|no_alloc_shim_is_unstable)|"
            r"(?:^|\s)alloc::|handle_alloc_error|"
            r"(?:^|\W)(?:malloc|calloc|realloc|free|posix_memalign)(?:$|\W)",
            re.IGNORECASE,
        ),
    ),
    ("Rust std", re.compile(r"(?:^|\W)std::")),
    (
        "C++ runtime",
        re.compile(
            r"__cxa_|__gxx_|operator (?:new|delete)|"
            r"(?:^|\W)_Z(?:nw|na|dl|da)",
        ),
    ),
    ("GTSAM", re.compile(r"gtsam|borglab", re.IGNORECASE)),
    (
        "unwind runtime",
        re.compile(
            r"_Unwind_|rust_eh_personality|panic_unwind|"
            r"__gcc_personality|__gxx_personality",
        ),
    ),
)

STEP_SOURCE = r"""
#![no_std]
#![no_main]

use aevia_trajectory::LiveSession;
use aevia_trajectory::observation::LiveStep;
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_: &PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub fn audit_step<'config, 'workspace, 'observation>(
    session: &mut LiveSession<'config, 'workspace>,
    step: LiveStep<'observation>,
) -> u32 {
    match session.step(step) {
        Ok(_) => 0,
        Err(_) => 1,
    }
}
"""

FINISH_SOURCE = r"""
#![no_std]
#![no_main]

use aevia_trajectory::LiveSession;
use aevia_trajectory::engine::LiveSummary;
use aevia_trajectory::observation::WorkQuota;
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_: &PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub fn audit_finish<'config, 'workspace>(
    session: &mut LiveSession<'config, 'workspace>,
    work: WorkQuota,
    summary: &mut LiveSummary,
) -> u32 {
    match session.finish(work, summary) {
        Ok(_) => 0,
        Err(_) => 1,
    }
}
"""


class AuditSetupError(RuntimeError):
    """The audit could not produce trustworthy inspection input."""


@dataclass(frozen=True)
class Toolchain:
    cargo: Path
    rustc: Path
    nm: Path
    objdump: Path
    readobj: Path
    linker: Path
    identity: str


@dataclass(frozen=True)
class HarnessReport:
    name: str
    architecture: str
    undefined: tuple[str, ...]
    forbidden: tuple[tuple[str, str], ...]
    panic_calls: tuple[tuple[str, tuple[str, ...]], ...]
    double_helpers: tuple[str, ...]
    double_math: tuple[str, ...]
    f_instructions: int
    d_instructions: int


def run_command(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        env=env,
        input=input_text,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def checked_output(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    description: str,
) -> str:
    result = run_command(command, cwd=cwd, env=env)
    if result.returncode != 0:
        output = "\n".join(part for part in (result.stdout, result.stderr) if part)
        raise AuditSetupError(f"{description} failed:\n{output.rstrip()}")
    return result.stdout


def executable(name: str) -> Path:
    path = shutil.which(name)
    if path is None:
        raise AuditSetupError(f"required executable `{name}` is not on PATH")
    # Keep rustup proxy paths intact. Resolving the `rustc` or `cargo` symlink
    # can turn the requested tool back into the multi-call `rustup` binary.
    return Path(path)


def discover_toolchain(repo_root: Path) -> Toolchain:
    cargo = executable("cargo")
    rustc = executable("rustc")
    verbose = checked_output(
        [str(rustc), "-vV"], cwd=repo_root, description="rustc identity query"
    )
    fields: dict[str, str] = {}
    for line in verbose.splitlines():
        key, separator, value = line.partition(": ")
        if separator:
            fields[key] = value
    host = fields.get("host")
    if host is None:
        raise AuditSetupError("rustc -vV did not report its host triple")
    sysroot_text = checked_output(
        [str(rustc), "--print", "sysroot"],
        cwd=repo_root,
        description="rustc sysroot query",
    ).strip()
    sysroot = Path(sysroot_text)
    bundled = sysroot / "lib" / "rustlib" / host / "bin"

    def llvm_tool(name: str) -> Path:
        override = os.environ.get(f"AEVIA_{name.upper().replace('-', '_')}")
        candidates = [Path(override)] if override else []
        candidates.append(bundled / name)
        on_path = shutil.which(name)
        if on_path is not None:
            candidates.append(Path(on_path))
        for candidate in candidates:
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return candidate
        raise AuditSetupError(
            f"unable to find {name}; install "
            "`rustup component add llvm-tools-preview` "
            f"or set AEVIA_{name.upper().replace('-', '_')}"
        )

    identity_parts = [fields.get("release"), fields.get("commit-hash")]
    if fields.get("LLVM version"):
        identity_parts.append(f"LLVM-{fields['LLVM version']}")
    return Toolchain(
        cargo=cargo,
        rustc=rustc,
        nm=llvm_tool("llvm-nm"),
        objdump=llvm_tool("llvm-objdump"),
        readobj=llvm_tool("llvm-readobj"),
        linker=llvm_tool("rust-lld"),
        identity=" ".join(part for part in identity_parts if part),
    )


def controlled_environment(build_root: Path) -> dict[str, str]:
    environment = os.environ.copy()
    for key in ("CARGO_ENCODED_RUSTFLAGS", "RUSTFLAGS"):
        environment.pop(key, None)
    environment["CARGO_INCREMENTAL"] = "0"
    environment["CARGO_TARGET_DIR"] = str(build_root)
    environment["CARGO_TERM_COLOR"] = "never"
    return environment


def dependency_tree(
    repo_root: Path, tools: Toolchain, environment: dict[str, str]
) -> tuple[str, list[str]]:
    output = checked_output(
        [
            str(tools.cargo),
            "tree",
            "--locked",
            "-p",
            PACKAGE,
            "--target",
            TARGET,
            "--no-default-features",
            "-e",
            "normal,build,features",
        ],
        cwd=repo_root,
        env=environment,
        description="normal/build dependency feature tree",
    )
    failures: list[str] = []
    lowered = output.lower()
    for dependency in FORBIDDEN_DEPENDENCIES:
        if re.search(rf"\b{re.escape(dependency.lower())}\s+v[0-9]", lowered):
            failures.append(f"forbidden firmware dependency: {dependency}")
    if FORBIDDEN_FEATURE in lowered:
        failures.append("forbidden firmware feature: soft-float")
    return output, failures


def build_library(
    repo_root: Path,
    build_root: Path,
    tools: Toolchain,
    environment: dict[str, str],
) -> Path:
    checked_output(
        [
            str(tools.cargo),
            "build",
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
        description="fresh RV32IMAFC no-default release build",
    )
    dependency_dir = build_root / TARGET / "release" / "deps"
    artifacts = sorted(dependency_dir.glob(f"lib{CRATE_STEM}-*.rlib"))
    if len(artifacts) != 1:
        listed = ", ".join(str(path) for path in artifacts) or "none"
        raise AuditSetupError(
            f"expected one fresh {PACKAGE} rlib, found {len(artifacts)}: {listed}"
        )
    return artifacts[0]


def link_harness(
    name: str,
    source: str,
    entry: str,
    repo_root: Path,
    build_root: Path,
    artifact: Path,
    tools: Toolchain,
) -> tuple[Path | None, str | None]:
    output = build_root / f"audit-{name}.elf"
    target_dependencies = build_root / TARGET / "release" / "deps"
    host_dependencies = build_root / "release" / "deps"
    result = run_command(
        [
            str(tools.rustc),
            "--crate-name",
            f"firmware_{name}_purity_harness",
            "--crate-type",
            "bin",
            "--edition",
            "2024",
            "--target",
            TARGET,
            "-C",
            "opt-level=3",
            "-C",
            "panic=abort",
            "-C",
            f"linker={tools.linker}",
            "-C",
            "link-arg=-e",
            "-C",
            f"link-arg={entry}",
            "-C",
            "link-arg=--gc-sections",
            "-L",
            f"dependency={target_dependencies}",
            "-L",
            f"dependency={host_dependencies}",
            "--extern",
            f"{CRATE_STEM}={artifact}",
            "-o",
            str(output),
            "-",
        ],
        cwd=repo_root,
        input_text=source,
    )
    if result.returncode == 0:
        return output, None
    diagnostics = "\n".join(
        part.rstrip() for part in (result.stdout, result.stderr) if part.strip()
    )
    if "no global memory allocator found" in diagnostics:
        return None, f"{name} harness requires a global allocator"
    raise AuditSetupError(f"{name} harness link failed:\n{diagnostics}")


def tool_output(
    command: list[str], repo_root: Path, description: str
) -> str:
    return checked_output(command, cwd=repo_root, description=description)


def canonical_symbol(symbol: str) -> str:
    symbol = re.sub(r"\+0x[0-9A-Fa-f]+$", "", symbol)
    return HASH_SUFFIX.sub("", symbol)


def panic_calls(disassembly: str) -> tuple[tuple[str, tuple[str, ...]], ...]:
    current: str | None = None
    callers: dict[str, set[str]] = defaultdict(set)
    for line in disassembly.splitlines():
        header = FUNCTION_HEADER.match(line)
        if header:
            current = canonical_symbol(header.group(1))
            continue
        if current is None or not current.startswith("aevia_trajectory::"):
            continue
        if not any(target in line for target in PANIC_TARGETS):
            continue
        match = CALL_TARGET.search(line)
        if match:
            callers[current].add(canonical_symbol(match.group(1)))
    return tuple(
        (caller, tuple(sorted(targets)))
        for caller, targets in sorted(callers.items())
    )


def software_double_math(
    raw_symbols: str, demangled_symbols: str
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    helpers = set()
    for line in raw_symbols.splitlines():
        fields = line.split()
        if fields and DOUBLE_HELPER.fullmatch(fields[-1]):
            helpers.add(fields[-1])

    functions = set()
    for line in demangled_symbols.splitlines():
        if "fpmath::host_f64::" in line or "fpmath::soft_f64::" in line:
            start = line.find("fpmath::")
            functions.add(canonical_symbol(line[start:]))
        for match in LIBM_FUNCTION.finditer(line):
            module = match.group(1)
            # libm's single-precision entrypoints conventionally carry a
            # trailing `f`; `modf` is the double-precision C entrypoint.
            if not module.endswith("f") or module == "modf":
                functions.add(f"libm::{module}")
    return tuple(sorted(helpers)), tuple(sorted(functions))


def inspect_harness(
    name: str, artifact: Path, repo_root: Path, tools: Toolchain
) -> HarnessReport:
    header = tool_output(
        [str(tools.readobj), "--arch-specific", "--needed-libs", str(artifact)],
        repo_root,
        f"{name} ELF header inspection",
    )
    if "Format: elf32-littleriscv" not in header or "Arch: riscv32" not in header:
        raise AuditSetupError(f"{name} harness is not an ELF32 RISC-V artifact")
    needed = re.search(r"NeededLibraries\s*\[(.*?)\]", header, re.DOTALL)
    if needed is None:
        raise AuditSetupError(f"{name} ELF did not report needed libraries")

    undefined_text = tool_output(
        [str(tools.nm), "--undefined-only", "--demangle", str(artifact)],
        repo_root,
        f"{name} undefined-symbol inspection",
    )
    undefined = tuple(line.strip() for line in undefined_text.splitlines() if line.strip())
    demangled = tool_output(
        [str(tools.nm), "--defined-only", "--demangle", str(artifact)],
        repo_root,
        f"{name} demangled-symbol inspection",
    )
    raw = tool_output(
        [str(tools.nm), "--defined-only", str(artifact)],
        repo_root,
        f"{name} raw-symbol inspection",
    )

    forbidden = []
    symbol_input = "\n".join((undefined_text, demangled, raw))
    for category, pattern in FORBIDDEN_SYMBOL_PATTERNS:
        for line in symbol_input.splitlines():
            if pattern.search(line):
                forbidden.append((category, line.strip()))
    if needed.group(1).strip():
        forbidden.append(("dynamic library", needed.group(1).strip()))

    disassembly = tool_output(
        [str(tools.objdump), "-d", "--demangle", str(artifact)],
        repo_root,
        f"{name} disassembly",
    )
    f_instructions = 0
    d_instructions = 0
    for precision in FP_ARITHMETIC.findall(disassembly):
        if precision == "s":
            f_instructions += 1
        else:
            d_instructions += 1
    helpers, math_functions = software_double_math(raw, demangled)
    architecture_match = re.search(r"Value: (rv32[^\n]+)", header)
    architecture = architecture_match.group(1).strip() if architecture_match else "riscv32"
    return HarnessReport(
        name=name,
        architecture=architecture,
        undefined=undefined,
        forbidden=tuple(sorted(set(forbidden))),
        panic_calls=panic_calls(disassembly),
        double_helpers=helpers,
        double_math=math_functions,
        f_instructions=f_instructions,
        d_instructions=d_instructions,
    )


def print_report(
    tools: Toolchain,
    dependency_failures: list[str],
    link_failures: list[str],
    reports: list[HarnessReport],
    *,
    deny_panics: bool,
    deny_software_f64: bool,
) -> list[str]:
    failures = [*dependency_failures, *link_failures]
    print(f"ESP32-S31 firmware-purity audit ({TARGET})")
    print(f"toolchain: {tools.identity}")
    print(
        "normal/build dependency tree: "
        f"{'FAIL' if dependency_failures else 'PASS'}"
    )

    for report in reports:
        print(f"{report.name} harness: {report.architecture}")
        print(f"  undefined symbols: {len(report.undefined)}")
        print(f"  forbidden reachable symbols: {len(report.forbidden)}")
        for category, symbol in report.forbidden:
            print(f"    [{category}] {symbol}")
            failures.append(f"{report.name}: reachable {category} symbol")
        if report.undefined:
            failures.append(f"{report.name}: undefined symbols remain")
            for symbol in report.undefined:
                print(f"    [undefined] {symbol}")

        print(
            f"  panic callsites: {len(report.panic_calls)} "
            f"({'FAIL' if deny_panics and report.panic_calls else 'informational'})"
        )
        for caller, targets in report.panic_calls:
            print(f"    {caller} -> {', '.join(targets)}")
        if deny_panics and report.panic_calls:
            failures.append(f"{report.name}: panic callsites remain")

        has_software_f64 = bool(report.double_helpers or report.double_math)
        print(
            "  software-f64 helpers: "
            f"{len(report.double_helpers) + len(report.double_math)} "
            f"({'FAIL' if deny_software_f64 and has_software_f64 else 'informational'})"
        )
        if report.double_helpers:
            print(f"    compiler: {', '.join(report.double_helpers)}")
        if report.double_math:
            print(f"    math: {', '.join(report.double_math)}")
        print(
            "  scalar FP arithmetic instructions: "
            f"F={report.f_instructions}, D={report.d_instructions}"
        )
        if deny_software_f64 and has_software_f64:
            failures.append(f"{report.name}: software f64 helpers remain")

    print(
        "proof boundary: this generic-target live-API ELF audit does not "
        "replace the final ESP-IDF dependency/link-map review or S31 timing "
        "and stack measurements."
    )
    return failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deny-panics",
        action="store_true",
        help="make direct Aevia-to-core panic callsites audit failures",
    )
    parser.add_argument(
        "--deny-software-f64",
        action="store_true",
        help="make reachable software-double helpers audit failures",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_args()
    repo_root = Path(__file__).resolve().parents[2]
    try:
        tools = discover_toolchain(repo_root)
        with tempfile.TemporaryDirectory(prefix="aevia-s31-purity-") as directory:
            build_root = Path(directory)
            environment = controlled_environment(build_root)
            _, dependency_failures = dependency_tree(
                repo_root, tools, environment
            )
            library = build_library(
                repo_root, build_root, tools, environment
            )
            link_failures: list[str] = []
            reports: list[HarnessReport] = []
            for name, source, entry in (
                ("step", STEP_SOURCE, "audit_step"),
                ("finish", FINISH_SOURCE, "audit_finish"),
            ):
                artifact, failure = link_harness(
                    name,
                    source,
                    entry,
                    repo_root,
                    build_root,
                    library,
                    tools,
                )
                if failure is not None:
                    link_failures.append(failure)
                elif artifact is not None:
                    reports.append(
                        inspect_harness(name, artifact, repo_root, tools)
                    )
            failures = print_report(
                tools,
                dependency_failures,
                link_failures,
                reports,
                deny_panics=arguments.deny_panics,
                deny_software_f64=arguments.deny_software_f64,
            )
    except (AuditSetupError, OSError) as error:
        print(f"ESP32-S31 firmware-purity audit ERROR: {error}", file=sys.stderr)
        return 2

    if failures:
        print("result: FAIL")
        for failure in sorted(set(failures)):
            print(f"  - {failure}")
        return 1
    print("result: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
