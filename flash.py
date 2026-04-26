import shutil, string, subprocess, sys
from pathlib import Path

def die(msg): sys.exit(f"flash.py: {msg}")

if not shutil.which("rust-objcopy"):
    die("rust-objcopy not found -- cargo install cargo-binutils")
if not any(Path(f"{l}:/INFO_UF2.TXT").exists() for l in string.ascii_uppercase):
    die("XIAO drive not found -- double-tap reset on the board")

elf = sys.argv[1]
here = Path(__file__).parent
subprocess.check_call(["rust-objcopy", "-O", "ihex", elf, elf + ".hex"])
subprocess.check_call([sys.executable, str(here / "uf2conv.py"), elf + ".hex", "-c", "-D", "-f", "0xADA52840", "-o", elf + ".uf2"])
