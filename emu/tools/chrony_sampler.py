#!/usr/bin/env python3
"""Error bound e_i(t) of the clock of a host once per step (calibration of eps, Section VI-E of the paper).

Runs in every VM of a paper-stack run (GS and vessels; emu/fleet.py starts it from the synchronized
repository and collects RUN/chrony.jsonl with the other files of /tmp/fdt-run). At start(t) + --offset on
the clock of the host, for t = --start-step .. --start-step + --steps - 1, it asks chronyd for its tracking
report and writes one JSON line:

  {"t": 812, "at": 1767225701.5625, "sys": -1.2e-05, "last": 3.1e-06, "root_delay": 0.00041,
   "root_disp": 0.00022, "e": 0.000437, "stratum": 3, "leap": "Normal"}

sys is the offset of the system clock from the time of chronyd ("System time"), last the last measured offset,
root_delay and root_disp the root delay and the root dispersion (seconds), and e the bound of the error of the
clock that the chronyc documentation gives for the tracking report:
e = |sys| + root_disp + root_delay / 2. A failed query writes {"t", "at", "error"}.

One chronyc process (`chronyc -c -n`, line-buffered through stdbuf when available) answers every query on
its standard input, so a sample costs one command over the command socket of chronyd and no fork; --oneshot
runs `chronyc -c -n tracking` per sample instead.

  chrony_sampler.py --epoch E --ts 0.125 --start-step S0 --steps N --out /tmp/fdt-run/chrony.jsonl
                    [--offset 0.0625] [--chronyc chronyc] [--oneshot] [--timeout 0.05]
"""
import argparse
import json
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import time

# fields of `chronyc -c tracking` (chronyc tracking, CSV): reference ID, name, stratum, reference time,
# system time, last offset, RMS offset, frequency, residual frequency, skew, root delay, root dispersion,
# update interval, leap status
F_STRATUM, F_SYS, F_LAST, F_ROOT_DELAY, F_ROOT_DISP, F_LEAP = 2, 4, 5, 10, 11, 13


def parse_tracking(line):
    """Dict of one CSV line of `chronyc -c tracking`, with the error bound e = |sys| + root_disp + root_delay / 2;
    None when the line is not a tracking report."""
    line = line.replace("chronyc>", "").strip()
    f = line.split(",")
    if len(f) < 14:
        return None
    try:
        sys_off, last = float(f[F_SYS]), float(f[F_LAST])
        rd, rdisp = float(f[F_ROOT_DELAY]), float(f[F_ROOT_DISP])
        stratum = int(f[F_STRATUM])
    except ValueError:
        return None
    return {"sys": sys_off, "last": last, "root_delay": rd, "root_disp": rdisp,
            "e": abs(sys_off) + rdisp + rd / 2.0, "stratum": stratum, "leap": f[F_LEAP].strip()}


class Chronyc:
    """One chronyc process for every query (interactive mode on a pipe), restarted when it exits."""

    def __init__(self, cmd, timeout=0.05):
        self.cmd, self.timeout = list(cmd), float(timeout)
        self.p, self.buf = None, b""

    def _start(self):
        self.p = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, bufsize=0)
        self.buf = b""

    def close(self):
        if self.p is not None:
            try:
                self.p.stdin.close()
            except OSError:
                pass
            try:
                self.p.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.p.kill()
            self.p = None

    def _kill(self):
        if self.p is not None:
            self.p.kill()
            self.p.wait()
            self.p = None

    def query(self, command="tracking"):
        """One line of output of `command`, or None after the timeout (the process is then restarted)."""
        if self.p is None or self.p.poll() is not None:
            self._start()
        try:
            self.p.stdin.write(command.encode() + b"\n")
        except OSError:
            self._kill()
            return None
        fd = self.p.stdout.fileno()
        end = time.monotonic() + self.timeout
        while b"\n" not in self.buf:
            rest = end - time.monotonic()
            if rest <= 0:
                self._kill()
                return None
            r, _, _ = select.select([fd], [], [], rest)
            if not r:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                self._kill()
                return None
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("utf-8", "replace")


class OneShot:
    """`chronyc -c -n tracking` per query."""

    def __init__(self, cmd, timeout=0.5):
        self.cmd, self.timeout = list(cmd), float(timeout)

    def query(self, command="tracking"):
        try:
            r = subprocess.run(self.cmd + [command], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, timeout=self.timeout)
        except (OSError, subprocess.TimeoutExpired):
            return None
        out = r.stdout.decode("utf-8", "replace").strip().splitlines()
        return out[0] if out else None

    def close(self):
        pass


def chronyc_cmd(chronyc="chronyc", oneshot=False):
    base = [chronyc, "-c", "-n"]
    if not oneshot and shutil.which("stdbuf"):
        return ["stdbuf", "-oL"] + base
    return base


def sample_once(session, t, now):
    """Record of step t sampled at now (Unix time on the clock of the host)."""
    line = session.query("tracking")
    rec = parse_tracking(line) if line is not None else None
    if rec is None:
        return {"t": int(t), "at": round(now, 6), "error": "no tracking report" if line is None else line[:200]}
    return dict({"t": int(t), "at": round(now, 6)}, **rec)


def run(session, out, epoch, Ts, start_step, steps, offset, clock=time.time, sleep=time.sleep, stop=None):
    """Samples steps start_step .. start_step + steps - 1 at start(t) + offset; a step whose instant has passed
    by more than one step (late start, an overrun) is skipped. Returns the number of records written."""
    n = 0
    t = int(start_step)
    end = int(start_step) + int(steps)
    while t < end and not (stop and stop()):
        at = epoch + t * Ts + offset
        now = clock()
        if now > at + Ts:   # behind: go on with the current step
            t = max(t + 1, int(math.floor((now - epoch - offset) / Ts)) + 1)
            continue
        if now < at:
            sleep(at - now)
        rec = sample_once(session, t, clock())
        out.write(json.dumps(rec, separators=(",", ":")) + "\n")
        out.flush()
        n += 1
        t += 1
    return n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--epoch", type=float, required=True, help="Unix time of start(0)")
    ap.add_argument("--ts", type=float, default=0.125)
    ap.add_argument("--start-step", type=int, required=True)
    ap.add_argument("--steps", type=int, required=True)
    ap.add_argument("--offset", type=float, default=None, help="instant of the sample after start(t) (default Ts / 2)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--chronyc", default="chronyc")
    ap.add_argument("--oneshot", action="store_true", help="one chronyc process per sample")
    ap.add_argument("--timeout", type=float, default=0.05, help="seconds to wait for a report")
    a = ap.parse_args(argv)
    stopped = []
    signal.signal(signal.SIGTERM, lambda *x: stopped.append(1))
    cmd = chronyc_cmd(a.chronyc, a.oneshot)
    session = OneShot(cmd, max(a.timeout, 0.5)) if a.oneshot else Chronyc(cmd, a.timeout)
    offset = a.ts / 2.0 if a.offset is None else a.offset
    with open(a.out, "a", encoding="utf-8") as out:
        try:
            run(session, out, a.epoch, a.ts, a.start_step, a.steps, offset, stop=lambda: bool(stopped))
        finally:
            session.close()


if __name__ == "__main__":
    sys.exit(main())
