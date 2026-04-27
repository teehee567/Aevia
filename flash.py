import shutil, string, subprocess, sys, time
from pathlib import Path
import serial
from serial.tools import list_ports
import monitor

XIAO_VIDS = (0x2886, 0x239A, 0xC0DE)
elf = sys.argv[1]
here = Path(__file__).parent

def find_drive():
    for l in string.ascii_uppercase:
        try:
            if Path(f"{l}:/INFO_UF2.TXT").exists():
                return Path(f"{l}:/")
        except OSError:
            pass
    return None

drive = find_drive()
if drive is None:
    p = next(p for p in list_ports.comports() if p.vid in XIAO_VIDS)
    print(f"flash.py: sending reboot magic to {p.device}")
    with serial.Serial(p.device, 115200) as s:
        s.write(b"!")
    while drive is None:
        time.sleep(0.25)
        drive = find_drive()

uf2 = elf + ".uf2"
subprocess.check_call(["rust-objcopy", "-O", "ihex", elf, elf + ".hex"])
subprocess.check_call([sys.executable, str(here / "uf2conv.py"), elf + ".hex", "-c", "-f", "0xADA52840", "-o", uf2])
shutil.copyfile(uf2, drive / Path(uf2).name)
print(f"flash.py: copied {uf2} -> {drive}")

while find_drive() is not None:
    time.sleep(0.25)
time.sleep(0.5)

monitor.monitor(monitor.wait_for_app_port())
