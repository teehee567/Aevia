import sys, time
import serial
from serial.tools import list_ports

APP_VIDS = (0xC0DE, 0x2886)

def wait_for_app_port():
    while True:
        for p in list_ports.comports():
            if p.vid in APP_VIDS:
                return p.device
        time.sleep(0.25)

def monitor(port, baud=115200):
    print(f"monitor.py: opening {port}")
    with serial.Serial(port, baud, timeout=1) as s:
        try:
            while True:
                data = s.read(256)
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    monitor(sys.argv[1] if len(sys.argv) > 1 else wait_for_app_port())
