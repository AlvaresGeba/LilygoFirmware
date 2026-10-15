#!/usr/bin/env python3
"""Send console commands to the watch and print what it answers.

  tools/watch_console.py S          # hardware/sensor probe
  tools/watch_console.py Q R        # clock report, RTC tick
  tools/watch_console.py --set-time # set the clock from this computer

The watch must be awake (tap it) and plugged in.
"""

import argparse
import sys
import time

from clock_drift_test import Watch

# Last line of each command's output.
DONE = {"S": "end of probe", "H": "this help", "Q": "CLK ", "R": "TICK", "T": "time set"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("commands", nargs="*")
    ap.add_argument("--port", default="/dev/ttyACM*")
    ap.add_argument("--set-time", action="store_true", help="set the watch clock from this computer")
    ap.add_argument("--timeout", type=float, default=15)
    args = ap.parse_args()

    watch = Watch(args.port)
    watch.connect(timeout=60)
    commands = list(args.commands)
    if args.set_time:
        offset = -(time.altzone if time.localtime().tm_isdst else time.timezone) // 60
        commands.insert(0, f"T{time.time():.6f} {offset}")
    for cmd in commands:
        print(f"> {cmd}")
        watch.send(cmd)
        end = DONE.get(cmd[:1].upper())
        deadline = time.monotonic() + args.timeout
        buf = b""
        while time.monotonic() < deadline:
            buf += watch.ser.read(512)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").rstrip()
                print(line)
                if end and end in line:
                    deadline = 0
        print()
    watch.close()


if __name__ == "__main__":
    sys.exit(main())
