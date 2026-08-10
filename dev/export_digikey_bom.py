#!/usr/bin/env python3
"""Export an upload-ready DigiKey BOM from a KiCad PCB.

KiCad is not required.  The exporter treats the placed PCB footprints as the
assembly source of truth, excludes footprints marked DNP/exclude_from_bom, and
groups the remaining parts by manufacturer part number.
"""

from __future__ import annotations

import argparse
import csv
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


PROPERTY_RE = re.compile(
    r'\(property\s+"(?P<name>(?:[^"\\]|\\.)*)"\s+'
    r'"(?P<value>(?:[^"\\]|\\.)*)"'
)
FOOTPRINT_NAME_RE = re.compile(r'^\(footprint\s+"((?:[^"\\]|\\.)*)"')
ATTR_RE = re.compile(r'\(attr\s+([^)]*)\)')


@dataclass(frozen=True)
class Part:
    reference: str
    value: str
    footprint: str
    mpn: str
    manufacturer: str
    digikey_pn: str
    supplier: str
    excluded_reason: str = ""


def _unescape(value: str) -> str:
    return value.replace(r'\"', '"').replace(r'\\', '\\')


def _footprint_blocks(text: str) -> list[str]:
    """Return top-level footprint expressions, respecting quoted strings."""
    blocks: list[str] = []
    for match in re.finditer(r'(?m)^\s*\(footprint\s+', text):
        start = text.find("(", match.start())
        depth = 0
        in_string = False
        escaped = False

        for index in range(start, len(text)):
            char = text[index]
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
                continue

            if char == '"':
                in_string = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    blocks.append(text[start : index + 1])
                    break
        else:
            raise ValueError(f"Unterminated footprint expression at byte {start}")
    return blocks


def _natural_reference(reference: str) -> tuple[str, int, str]:
    match = re.fullmatch(r"([^0-9]*)([0-9]+)(.*)", reference)
    if not match:
        return reference, -1, ""
    return match.group(1), int(match.group(2)), match.group(3)


def parse_parts(pcb_path: Path) -> list[Part]:
    parts: list[Part] = []
    text = pcb_path.read_text(encoding="utf-8")
    for block in _footprint_blocks(text):
        name_match = FOOTPRINT_NAME_RE.match(block)
        if not name_match:
            raise ValueError("Footprint block has no library/footprint name")
        properties = {
            _unescape(match.group("name")): _unescape(match.group("value"))
            for match in PROPERTY_RE.finditer(block)
        }
        reference = properties.get("Reference", "")
        value = properties.get("Value", "")
        mpn = properties.get("MPN", "").strip()
        attrs = set()
        for attr_match in ATTR_RE.finditer(block):
            attrs.update(attr_match.group(1).split())

        excluded_reason = ""
        if "exclude_from_bom" in attrs:
            excluded_reason = "exclude_from_bom"
        elif "dnp" in attrs or value.upper() == "DNP" or mpn.upper() == "DNP":
            excluded_reason = "DNP"
        elif not mpn:
            excluded_reason = "missing MPN"
        elif mpn.startswith("N/A"):
            excluded_reason = "non-purchased PCB feature"

        parts.append(
            Part(
                reference=reference,
                value=value,
                footprint=_unescape(name_match.group(1)),
                mpn=mpn,
                manufacturer=properties.get("Manufacturer", "").strip(),
                digikey_pn=properties.get("DigiKey_PN", "").strip(),
                supplier=properties.get("Supplier", "").strip(),
                excluded_reason=excluded_reason,
            )
        )
    return parts


def write_order_bom(parts: list[Part], output_path: Path, boards: int) -> None:
    included = [part for part in parts if not part.excluded_reason]
    grouped: dict[tuple[str, str, str], list[Part]] = defaultdict(list)
    for part in included:
        key = (part.mpn, part.value, part.footprint)
        grouped[key].append(part)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "Quantity",
                "Digi-Key Part Number",
                "Manufacturer Part Number",
                "Manufacturer",
                "Customer Reference",
                "Value",
                "Footprint",
            ]
        )
        rows = []
        for key, grouped_parts in grouped.items():
            mpn, value, footprint = key
            digikey_pns = {part.digikey_pn for part in grouped_parts if part.digikey_pn}
            manufacturers = {
                part.manufacturer for part in grouped_parts if part.manufacturer
            }
            if len(digikey_pns) > 1:
                raise ValueError(f"Conflicting DigiKey PNs for {mpn}: {digikey_pns}")
            if len(manufacturers) > 1:
                raise ValueError(f"Conflicting manufacturers for {mpn}: {manufacturers}")
            digikey_pn = next(iter(digikey_pns), "")
            manufacturer = next(iter(manufacturers), "")
            references = sorted(
                (part.reference for part in grouped_parts), key=_natural_reference
            )
            rows.append(
                (
                    references[0],
                    [
                        len(grouped_parts) * boards,
                        digikey_pn,
                        mpn,
                        manufacturer,
                        ",".join(references),
                        value,
                        footprint,
                    ],
                )
            )
        for _, row in sorted(rows, key=lambda item: _natural_reference(item[0])):
            writer.writerow(row)


def write_exclusions(parts: list[Part], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "Reference",
                "Value",
                "Manufacturer Part Number",
                "Footprint",
                "Exclusion Reason",
            ]
        )
        for part in sorted(parts, key=lambda part: _natural_reference(part.reference)):
            if part.excluded_reason:
                writer.writerow(
                    [
                        part.reference,
                        part.value,
                        part.mpn,
                        part.footprint,
                        part.excluded_reason,
                    ]
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcb", type=Path, help="Input .kicad_pcb file")
    parser.add_argument("output", type=Path, help="Output DigiKey CSV")
    parser.add_argument(
        "--exclusions",
        type=Path,
        help="Optional CSV recording DNP, virtual, and incomplete lines",
    )
    parser.add_argument(
        "--boards",
        type=int,
        default=1,
        help="Number of boards to buy for (default: 1)",
    )
    args = parser.parse_args()
    if args.boards < 1:
        parser.error("--boards must be at least 1")

    parts = parse_parts(args.pcb)
    write_order_bom(parts, args.output, args.boards)
    if args.exclusions:
        write_exclusions(parts, args.exclusions)

    included = [part for part in parts if not part.excluded_reason]
    missing = [part for part in parts if part.excluded_reason == "missing MPN"]
    print(
        f"Exported {len(included)} placements from {len(parts)} footprints; "
        f"{len(missing)} footprints have no MPN."
    )


if __name__ == "__main__":
    main()
