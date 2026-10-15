#!/usr/bin/env python3
"""Hardware test: how far the watch clock drifts over deep sleep.

Drives the firmware's USB console (see "H" on the watch). The host clock is
the reference, so keep it NTP-synced (`timedatectl` says "synchronized: yes").

For each sleep clock under test the watch's RTC correction is switched off
("N0"), the clock is set from the host, and the watch is sent into timed deep
sleeps ("Z<sec>"). After every wake the system clock is compared with the
host: the error growth per second of sleep is the sleep oscillator's drift.
A final run with the correction on ("N1") checks what the user actually
sees: the error must stay below one second however long the watch sleeps.
The PCF8563 itself is measured at the start and end with "R" (it reports the
moment its seconds register ticks, which gives its phase to ~1 ms).

The watch must be awake and plugged in when the test starts (tap it). The
default plan takes about 25 minutes.

  pip install pyserial   (already in PlatformIO's venv)
  tools/clock_drift_test.py --cycles 6 --sleep 60 --csv drift.csv
"""

import argparse
import csv
import glob
import re
import statistics
import sys
import termios
import time

import serial

CLK_RE = re.compile(r"CLK sys=(\d+)\.(\d+) rtc=(-?\d+) rtc_ok=(\d) .*?src=(\w+) cal=(\d+) corr_us=(-?\d+)")
TICK_RE = re.compile(r"TICK rtc=(\d+) sys=(\d+)\.(\d+)")


class Watch:
    """The watch's USB console. The port disappears while the watch sleeps."""

    def __init__(self, port_glob, verbose=False):
        self.port_glob = port_glob
        self.verbose = verbose
        self.ser = None

    def connect(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ports = sorted(glob.glob(self.port_glob))
            if ports:
                try:
                    ser = serial.Serial()
                    ser.port = ports[0]
                    ser.baudrate = 115200
                    ser.timeout = 0.05
                    # The S3's USB-Serial-JTAG resets the chip when RTS is
                    # set while DTR is clear. The kernel raises both on open;
                    # pyserial then applies DTR before RTS, so asking for
                    # both low passes through DTR=0/RTS=1 and resets the
                    # watch. Keep both high: no transition at all.
                    ser.dtr = True
                    ser.rts = True
                    ser.open()
                    self.ser = ser
                    time.sleep(0.3)
                    ser.reset_input_buffer()
                    return
                except (serial.SerialException, OSError):
                    pass  # udev not done yet
            time.sleep(0.1)
        raise TimeoutError(f"no watch on {self.port_glob} after {timeout:.0f} s - is it awake?")

    def attach(self, timeout):
        """Connects and makes sure the watch answers.

        The watch may be just falling asleep when the port is opened (the
        clock screen sleeps after a few seconds): the first write then fails
        or goes unanswered. Each console command keeps it awake, so once a
        "Q" is answered the session holds. Retries until `timeout`.
        """
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.connect(timeout=max(1.0, deadline - time.monotonic()))
                self.command("Q", CLK_RE.pattern)
                return
            except (serial.SerialException, OSError, termios.error, TimeoutError):
                self.close()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"the watch on {self.port_glob} does not answer - tap it")
                time.sleep(0.5)

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None

    def send(self, line):
        if self.verbose:
            print(f"  > {line}")
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def wait_for(self, pattern, timeout=3.0):
        """Returns (match, host time when the line arrived)."""
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            # Whatever is there, or block for one byte: a fixed-size read would
            # wait out the timeout and misdate the line by up to 50 ms.
            chunk = self.ser.read(self.ser.in_waiting or 1)
            arrived = time.time()
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if self.verbose and line:
                    print(f"  < {line}")
                m = pattern.search(line)
                if m:
                    return m, arrived
        raise TimeoutError(f"no reply matching {pattern.pattern!r}")

    def command(self, line, expect, timeout=3.0):
        self.send(line)
        return self.wait_for(re.compile(expect), timeout)

    def set_time(self, tz_minutes):
        # The line takes well under a millisecond over USB; the remaining
        # error is the host's scheduling jitter.
        t = time.time()
        self.send(f"T{t:.6f} {tz_minutes}")
        self.wait_for(re.compile(r"time set from console"))

    def clock(self):
        """System clock error vs the host, in ms, plus the report fields."""
        sent = time.time()
        m, arrived = self.command("Q", CLK_RE.pattern)
        host = (sent + arrived) / 2
        sys_t = int(m[1]) + int(m[2]) / 1e6
        return {
            "sys_err_ms": (sys_t - host) * 1000,
            "rtc": int(m[3]),
            "rtc_ok": m[4] == "1",
            "src": m[5],
            "cal": int(m[6]),
            "corr_us": int(m[7]),
            "rtt_ms": (arrived - sent) * 1000,
        }

    def rtc_error_ms(self):
        """PCF8563 error vs the host, from the moment its second ticks."""
        m, arrived = self.command("R", TICK_RE.pattern, timeout=3.0)
        return (int(m[1]) - arrived) * 1000

    def sleep(self, seconds):
        self.send(f"Z{seconds}")
        self.close()

    def wake(self, sleep_seconds):
        # Wait for the port to vanish (asleep), then to come back (awake).
        time.sleep(max(0.0, sleep_seconds - 2))
        self.connect(timeout=sleep_seconds + 30)


def fit_ppm(points):
    """Least-squares slope of error (ms) over elapsed time (s), in ppm."""
    if len(points) < 2:
        return float("nan")
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return float("nan")
    slope_ms_per_s = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope_ms_per_s * 1000  # ms/s -> ppm


def run_mode(watch, mode, cycles, sleep_s, tz, rows, resync):
    label = {"0": "rc136k", "1": "rcfast256"}[mode]
    print(f"\n=== sleep clock {label}, RTC correction {'on' if resync else 'off'}: "
          f"{cycles} x {sleep_s} s ===")
    watch.command(f"C{mode}", r"sleep clock:")
    watch.command(f"N{1 if resync else 0}", r"RTC correction on wake:")
    watch.set_time(tz)
    start = time.time()
    base = watch.clock()
    print(f"  start: error {base['sys_err_ms']:+8.1f} ms (rtt {base['rtt_ms']:.1f} ms)")
    points = [(0.0, base["sys_err_ms"])]
    for i in range(1, cycles + 1):
        watch.sleep(sleep_s)
        watch.wake(sleep_s)
        # Within a second of waking the firmware steps the clock onto the
        # RTC's next tick (with the correction on): measure after that.
        time.sleep(1.2)
        c = watch.clock()
        elapsed = time.time() - start
        points.append((elapsed, c["sys_err_ms"]))
        rows.append({"mode": label, "resync": int(resync), "cycle": i, "elapsed_s": round(elapsed, 1),
                     "sys_err_ms": round(c["sys_err_ms"], 1), "corr_us": c["corr_us"],
                     "cal": c["cal"], "rtt_ms": round(c["rtt_ms"], 1)})
        print(f"  wake {i:2d} after {elapsed:6.0f} s: error {c['sys_err_ms']:+9.1f} ms, "
              f"RTC correction applied at boot {c['corr_us'] / 1000:+9.1f} ms")
    return points


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM*")
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--sleep", type=int, default=60, help="seconds per deep sleep")
    ap.add_argument("--modes", default="0,1", help="sleep clocks to measure raw: 0=rc136k, 1=rcfast256")
    ap.add_argument("--final-mode", default="0", help="sleep clock for the end-to-end run")
    ap.add_argument("--tz", type=int, default=None, help="UTC offset (minutes) to leave on the watch")
    ap.add_argument("--csv", help="write every measurement here")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    tz = args.tz if args.tz is not None else round(-time.timezone / 60 if not time.localtime().tm_isdst
                                                  else -time.altzone / 60)
    watch = Watch(args.port, args.verbose)
    watch.attach(timeout=150)  # may be in a test sleep from an earlier run
    rows = []
    results = {}
    try:
        watch.set_time(tz)
        rtc_start = watch.rtc_error_ms()
        rtc_start_at = time.time()
        print(f"PCF8563 right after setting: {rtc_start:+.1f} ms")

        for mode in [m for m in args.modes.split(",") if m]:
            points = run_mode(watch, mode, args.cycles, args.sleep, tz, rows, resync=False)
            results[mode] = (fit_ppm(points), points[-1][1])

        final = run_mode(watch, args.final_mode, args.cycles, args.sleep, tz, rows, resync=True)
        worst = max(abs(p[1]) for p in final)

        rtc_end = watch.rtc_error_ms()
        rtc_ppm = (rtc_end - rtc_start) / (time.time() - rtc_start_at) * 1000
    finally:
        # Leave the watch in its normal state: correction on, default sleep
        # clock, correct time.
        try:
            if watch.ser is None:
                watch.connect(timeout=args.sleep + 30)
            watch.command(f"C{args.final_mode}", r"sleep clock:")
            watch.command("N1", r"RTC correction on wake:")
            watch.set_time(tz)
        except Exception as exc:  # noqa: BLE001 - best effort cleanup
            print(f"cleanup failed: {exc}", file=sys.stderr)
        watch.close()

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    print("\n=== summary ===")
    names = {"0": "RC 136 kHz (IDF default)", "1": "RC_FAST/256 (17.5 MHz / 256)"}
    for mode, (ppm, last) in results.items():
        print(f"  {names[mode]:30s} drift {ppm:+9.0f} ppm = {ppm * 86.4 / 1000:+8.1f} s/day "
              f"(last error {last / 1000:+.2f} s)")
    print(f"  PCF8563 RTC                    drift {rtc_ppm:+9.1f} ppm = {rtc_ppm * 86.4 / 1000:+8.2f} s/day "
          f"(+-{2 / (time.time() - rtc_start_at) * 1000:.1f} ppm resolution over this run)")
    print(f"  with RTC correction on: worst error {worst:.0f} ms "
          f"({'PASS' if worst < 1000 else 'FAIL'}: must stay < 1000 ms)")
    return 0 if worst < 1000 else 1


if __name__ == "__main__":
    sys.exit(main())
