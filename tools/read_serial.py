#!/usr/bin/env python3
"""Reset the XIAO nRF54L15 via its CMSIS-DAP probe and capture the SD-card
self-test output from the USB-serial console (COM10)."""
import sys, time, subprocess, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 115200
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0


def reset_target():
    try:
        r = subprocess.run([sys.executable, "-m", "pyocd", "reset", "-t", "nrf54l"],
                           capture_output=True, text=True, timeout=40)
        print("[pyocd reset rc=%d] %s %s" % (r.returncode,
              r.stdout.strip(), r.stderr.strip()), flush=True)
    except Exception as e:
        print("[pyocd reset error] %r" % e, flush=True)


def main():
    try:
        ser = serial.Serial()
        ser.port = PORT
        ser.baudrate = BAUD
        ser.timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
    except Exception as e:
        print("ERROR opening %s: %r" % (PORT, e), flush=True)
        sys.exit(2)

    time.sleep(0.2)
    ser.reset_input_buffer()
    print("=== resetting target via CMSIS-DAP ===", flush=True)
    reset_target()
    print("=== capturing %.0fs from %s @ %d ===" % (DURATION, PORT, BAUD), flush=True)

    t0 = time.time()
    buf = b""
    passed = failed = False
    while time.time() - t0 < DURATION:
        data = ser.read(4096)
        if data:
            try:
                sys.stdout.write(data.decode("utf-8", "replace"))
            except Exception:
                sys.stdout.write(repr(data))
            sys.stdout.flush()
            buf += data
            if b"SD CARD TEST: PASSED" in buf:
                passed = True
            if b"SD CARD TEST: FAILED" in buf:
                failed = True
            if passed or failed:
                time.sleep(0.7)
                tail = ser.read(16384)
                sys.stdout.write(tail.decode("utf-8", "replace"))
                sys.stdout.flush()
                break
    ser.close()
    verdict = "PASSED" if (passed and not failed) else ("FAILED" if failed else "NO_VERDICT")
    print("\n=== CAPTURE_RESULT: %s ===" % verdict, flush=True)
    sys.exit(0 if verdict == "PASSED" else 1)


main()
