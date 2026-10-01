#!/usr/bin/env python3
"""hold_cmd.py - keep a motor command alive on the serial test transmitter.

The transmitter (firmware/transmitter/transmitter.ino, serial mode) has a dead-man: a typed
"m a b" command expires after 0.5 s and the transmitter sends 0. Long bench tests (T7, T8, T9,
T11 for 30 min, T12) need the command to persist, so this script re-sends it every 200 ms and
sends "s" (stop) when it exits: normal end, --duration, Ctrl+C, SIGTERM or any error.
If the script is killed hard (kill -9, unplugged USB) the command simply expires: motors stop
within 0.5 s (dead-man) plus the receiver link timeout.

Usage:
    python hold_cmd.py COM5 800 800                 # hold m1=800, m2=800 until Ctrl+C
    python hold_cmd.py COM5 300 -300 --duration 30  # hold 30 s, then stop
    python hold_cmd.py /dev/ttyUSB0 500 500 --baud 115200

Arguments: port of the TRANSMITTER (not the receiver), m1, m2 in permille (-1000..1000).
Serial settings: 115200 baud 8N1 (transmitter.ino Serial.begin(115200)). Needs: pip install pyserial.
"""
import argparse
import signal
import sys
import time

PERIOD_S = 0.2        # resend period: well inside the transmitter's 0.5 s dead-man
DEFAULT_BAUD = 115200


def clamp(v):
    return max(-1000, min(1000, int(v)))


def run(ser, m1, m2, period=PERIOD_S, duration=None, sleep=time.sleep, clock=time.monotonic, out=sys.stdout):
    """Send 'm m1 m2' every `period` seconds until `duration` (None = forever, Ctrl+C).
    Always sends 's' on the way out. Returns the number of command lines sent."""
    cmd = ("m %d %d\n" % (clamp(m1), clamp(m2))).encode("ascii")
    stop = b"s\n"
    sent = 0
    start = clock()
    try:
        while duration is None or clock() - start < duration:
            ser.write(cmd)
            sent += 1
            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(stop)
            ser.write(stop)  # twice: cheap insurance against one garbled line
            ser.flush()
        except Exception as e:  # port gone: the dead-man covers it
            print("WARN: could not send stop (%s); dead-man will stop the motors in 0.5 s" % e, file=out)
    return sent


def main(argv=None):
    ap = argparse.ArgumentParser(description="Hold a motor command on the serial test transmitter.")
    ap.add_argument("port", help="serial port of the transmitter (COM5, /dev/ttyUSB0)")
    ap.add_argument("m1", type=int, help="motor 1, permille -1000..1000")
    ap.add_argument("m2", type=int, help="motor 2, permille -1000..1000")
    ap.add_argument("--duration", type=float, default=None, help="seconds; default: until Ctrl+C")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args(argv)
    try:
        import serial
    except ImportError:
        print("pyserial missing: pip install pyserial", file=sys.stderr)
        return 2
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0)
    except Exception as e:
        print("cannot open %s: %s" % (args.port, e), file=sys.stderr)
        return 1
    # Opening the port resets many ESP32 boards through DTR/RTS: wait for the transmitter to boot.
    time.sleep(2.0)
    # Turn SIGTERM into the same clean exit path as Ctrl+C.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print("holding m %d %d every %.0f ms; Ctrl+C = stop" % (clamp(args.m1), clamp(args.m2), PERIOD_S * 1000))
    n = run(ser, args.m1, args.m2, duration=args.duration)
    print("stopped after %d commands" % n)
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
