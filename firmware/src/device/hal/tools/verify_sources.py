#!/usr/bin/env python3
"""Verify HAL source fingerprints and V2 Mini GPIO mappings; no external packages."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys

HAL = Path(__file__).resolve().parents[1]
REPO = HAL.parents[3]


def sexpr(text: str) -> list:
    tokens = re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', text)
    stack: list[list] = []
    for token in tokens:
        if token == '(':
            value: list = []
            if stack:
                stack[-1].append(value)
            stack.append(value)
        elif token == ')':
            value = stack.pop()
            if not stack:
                return value
        else:
            stack[-1].append(json.loads(token) if token.startswith('"') else token)
    raise ValueError('unclosed PCB s-expression')


def entries(node: list, name: str):
    return (child for child in node if isinstance(child, list) and child and child[0] == name)


def verify() -> list[str]:
    errors = []
    for record in json.loads((HAL / 'sources.json').read_text()):
        source = REPO / record['path']
        if not source.is_file():
            errors.append(f'Missing source: {source}')
        elif hashlib.sha256(source.read_bytes()).hexdigest() != record['sha256']:
            errors.append(f'Source changed; recheck its HAL assumptions: {source}')
    board = sexpr((REPO / 'hardware/v2_mini_pcb/v2_mini.kicad_pcb').read_text())
    actual = {}
    for footprint in entries(board, 'footprint'):
        props = {p[1]: p[2] for p in entries(footprint, 'property')}
        if props.get('Reference') != 'U2':
            continue
        for pad in entries(footprint, 'pad'):
            functions = list(entries(pad, 'pinfunction'))
            nets = list(entries(pad, 'net'))
            if functions and nets:
                gpio = re.search(r'GPIO(\d+)', functions[0][1])
                if gpio:
                    actual[int(gpio[1])] = nets[0][-1].rsplit('/', 1)[-1]
    expected = {
        0: 'CHG_INT', 1: 'CHG_STAT', 2: 'BAT_ALRT', 4: 'LCD_BACK',
        5: 'LCD_TE', 6: 'POW_SCL', 7: 'POW_SDA', 8: 'PWR_KILL_N',
        9: 'IMU_RESET_N', 10: 'IMU_CS_N', 11: 'IMU_MOSI', 12: 'IMU_SCLK',
        13: 'IMU_MISO', 14: 'IMU_DRDY', 15: 'LCD_RN', 16: 'LCD_SCLK',
        17: 'LCD_SDA', 18: 'LCD_CSN', 19: 'LCD_DC', 20: 'SD_D0',
        21: 'SD_D1', 22: 'SD_D2', 23: 'SD_D3', 24: 'SD_CLK', 25: 'SD_CMD',
        35: 'USB_VBUS_S', 38: 'PWR_INT_N', 39: 'LED_EN', 40: 'CHG_PG',
        42: 'GNSS_PPS', 43: 'USB_CC1', 44: 'GNSS_RESETN', 45: 'USB_CC2',
        46: 'GNSS_TXD1', 47: 'GNSS_RXD1', 48: 'GNSS_TXD2', 49: 'GNSS_RXD2',
        50: 'GNSS_PVT', 51: 'GNSS_RTK', 52: 'GNSS_ERR',
    }
    for gpio, signal in expected.items():
        if actual.get(gpio) != signal:
            errors.append(f'GPIO{gpio}: expected {signal}, PCB has {actual.get(gpio)!r}')
    return errors


if __name__ == '__main__':
    failures = verify()
    for failure in failures:
        print(failure, file=sys.stderr)
    if failures:
        sys.exit(1)
    print('Datasheet/BOM fingerprints and V2 Mini GPIO mappings verified.')
