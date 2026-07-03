#!/usr/bin/env python3
"""Wait for the XIAO nRF54L15 to be unplugged and replugged (a true power cycle,
which also power-cycles the SD card), then capture the boot + heartbeat output.
Matches the board by USB VID:PID so it survives a COM-number change."""
import sys, time, serial
from serial.tools import list_ports

VID, PID = 0x2886, 0x0066
DEADLINE = time.time() + 180.0


def find_port():
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def wait(cond, msg):
    print(msg, flush=True)
    while time.time() < DEADLINE:
        if cond():
            return True
        time.sleep(0.25)
    return False


# make sure it's present, then gone, then back
if find_port() is None:
    wait(lambda: find_port() is not None, "Waiting for the board to be present...")
if not wait(lambda: find_port() is None, ">>> Please UNPLUG the board's USB now..."):
    print("timeout waiting for unplug"); sys.exit(2)
port = None
def _back():
    global port
    port = find_port()
    return port is not None
if not wait(_back, ">>> Unplug seen. Now PLUG IT BACK IN..."):
    print("timeout waiting for replug"); sys.exit(2)

print("Replug detected on %s -- opening console" % port, flush=True)

ser = None
for _ in range(80):                     # port may take a moment to be openable
    try:
        ser = serial.Serial()
        ser.port = port; ser.baudrate = 115200; ser.timeout = 0.2
        ser.dtr = False; ser.rts = False
        ser.open()
        break
    except Exception:
        ser = None
        time.sleep(0.15)
if ser is None:
    print("could not open %s" % port); sys.exit(2)

t0 = time.time(); buf = b""; passed = failed = False
while time.time() - t0 < 30:
    d = ser.read(4096)
    if d:
        sys.stdout.write(d.decode("utf-8", "replace")); sys.stdout.flush()
        buf += d
        if b"SD CARD TEST: PASSED" in buf: passed = True
        if b"SD CARD TEST: FAILED" in buf: failed = True
        # once we have a verdict AND at least one heartbeat, stop
        if (passed or failed) and b"[status]" in buf:
            sys.stdout.write(ser.read(8192).decode("utf-8", "replace"))
            break
ser.close()
verdict = "PASSED" if (passed and not failed) else ("FAILED" if failed else "NO_VERDICT")
print("\n=== POWER-CYCLE VERDICT: %s ===" % verdict, flush=True)
sys.exit(0 if verdict == "PASSED" else 1)
