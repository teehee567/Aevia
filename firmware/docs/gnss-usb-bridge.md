# UM980 USB bridge and connected-board findings

J6 has a separate raw GNSS CDC interface, currently **COM6**. The console is
**COM5**. The raw interface transports bytes without parsing commands or
adding STATUS/RAW text. `GNSSBRIDGE` on the console hands GNSS ownership to
the bridge; `GNSSNORMAL` returns ownership to normal acquisition. BOOTLOADER
remains available on the console in either mode.

Bridge entry sets UM980 COM2 to **460800, 8N1** through the working COM1
command path and enables BESTNAVA COM2 at 20 Hz. It then stops sending its
own receiver commands. The host's raw-port baud selection changes the MCU
UART baud; use 8N1 and no flow control. Reopening the port does not leave
bridge mode. Normal mode restores COM2 using COM1. Changing COM1 itself in
receiver software can therefore prevent normal recovery.

These are volatile settings; the bridge does not issue SAVECONFIG. Normal
firmware still restores its existing standalone profile on reboot or after
GNSSNORMAL. Status reports `gnss=bridge`, bridge baud, forwarded byte counts
and transport errors. A blocked reader can lose data; bridge error counters
include USB deadlines and UART faults. This is a bring-up transport.

## Live findings, 2026-09-10

**Full two-way pass-through is not working on the connected assembly.**
The receiver-to-host path does carry CRC-valid UM980 navigation records.
Unicore's UPrecise application itself was not installed/tested in this session.

The original single USB console only recognized firmware commands, so a raw
VERSIONA request received STATUS messages and no receiver reply. Adding the
second CDC interface fixed that transport separation, but exposed another
problem: direct VERSIONA and UNLOG commands sent to COM2 did not produce a
reply or stop its stream. Swapping the ESP32 UART1/UART2 peripherals while
keeping their GPIO assignments did not change the failure.

| Schematic connection | Observation |
| --- | --- |
| GPIO47 -> UM980 RXD1, pad 43 | Working: commands configure receiver COM2 and request VERSIONA COM2 |
| UM980 TXD2, pad 27 -> GPIO48 | Working: CRC-valid COM2 output received |
| UM980 TXD1, pad 42 -> GPIO46 | No replies, including explicit VERSIONA COM1 with both receiver ports normalized to 115200 |
| GPIO49 -> UM980 RXD2, pad 26 | Commands had no observed effect; with GPIO49 released and used as an input, it received an explicit VERSIONA COM2 reply |

The GPIO49 observation is **COM2 output**, not a discovered alternate COM1
receive pin. An initial byte-count-only probe was insufficient to identify it;
the explicit port query and matching baud distinguished it. Production GPIO
assignments remain those in the schematic. No hardware files were changed.

The MCU module pin table and UM980 pin table agree with the schematic labels.
There may be a short, assembly issue, or another physical connection mismatch;
software observations alone do not locate it. With board power removed, check:

- COM2 RX/TX isolation: UM980 pads **26/27** and MCU module pads **59/58**
  (GPIO49/GPIO48).
- COM1 return continuity: UM980 pad **42** to MCU module pad **56** (GPIO46),
  and whether that net is shorted to ground or an adjacent pad.

Reference files: `data/datasheets/UM980_User Manual_EN_R1.9.pdf`, physical
pages 13-14; `data/datasheets/esp32-s31-wroom-3_datasheet_en.pdf`, physical
page 14; `hardware/v2_mini_pcb/v2_mini.kicad_pcb`.

The final image passed 31 firmware host tests, four flashing-helper tests,
strict host/device Clippy, formatting and release build. Flashing and automatic
restart passed with the composite USB device. The receive-only check passed
CRC validation, port close/reopen and return to `gnss=ready`. A subsequent
five-second base capture received 909 IMU and 100 GNSS records with no new
capture errors or queue drops; PSRAM and STOP checks also passed. GNSS retains
its own scheduled task: joining it into the USB console's poll had caused
UART service delays during telemetry streaming and was removed.

## Checks

From the repository root, close other serial readers and run:

```powershell
# Full test: intentionally fails until the receiver command path works.
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-gnss-bridge.ps1 -ConsolePort COM5 -GnssPort COM6

# Receive-only test, then return to normal acquisition.
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-gnss-bridge.ps1 -ConsolePort COM5 -GnssPort COM6 -ReceiveOnly

# Leave the receive stream available for testing in a host serial application.
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check-gnss-bridge.ps1 -ConsolePort COM5 -GnssPort COM6 -ReceiveOnly -LeaveEnabled
```

The full test requires a direct CRC-valid VERSIONA reply, valid navigation
frames, no firmware text on the raw port, exclusive GNSS ownership and a
successful reconnect. `-ReceiveOnly` explicitly skips the command assertion;
its PASS is not a pass-through success. Without `-LeaveEnabled`, a successful
check also requires `gnss=ready` after returning to normal mode.

Send `GNSSNORMAL` plus newline to COM5 at 115200 to return to the base profile,
or reboot. COM6 is released when the script exits. Port numbers can change
with a different PC or USB configuration.
