"""Allocation system (AS) of the GS: phase 2 of each step (Section V-C of the Fleet-DT paper).

At start(t) + T1 the AS closes M(t), computes C_hat(t) = (1 - alpha) C_hat(t-1) + alpha c_y y(t-1) (eq. 2;
C_hat(t-1) when y(t-1) did not arrive in phase 1), K_hat(t) = C_hat(t) (T3 - T_m) / Ts and
b(t) = max(0, U K_hat(t) - Q(t)) / m(t) (eq. 3, Q(t) = sum of the backlogs Q_i(t) of the demand tables of M(t);
c_y and T_m from the DTP), runs Algorithm 1 over sigma_t and N(t), publishes (t, p(t)) on boat/b<i>/levels
(QoS 1) for every i in M(t) and the JSON summary on fleet/as. The decision (log line and fleet/as) holds D (the demand tables of M(t)),
the table, and the admitted bytes a_i(t) of each vessel ("a") with their sum A(t) ("A"), which the backlog
predictor of the outer loop replays (C5). The optional outer loop (fleetdt.outer, contribution (iv)) updates
U and alpha from the DTA every `outer_every` steps; it runs in a child process (fleetdt.outer.run_job,
SCHED_OTHER and nice 10), so its Python loops do not hold the GIL of the process that publishes p(t)
and the real-time priority of the AS preempts it. Service.connect() returns after the SUBACK of the
subscriptions, so step 0 never runs with a subscription that the broker has not yet acknowledged.

Policies (--policy):
  alg1  Algorithm 1, one table for the fleet (the paper; default)
  b7    baseline B7: per-vessel grants with redistribution of the unused grant (IPACT limited service
        with excess distribution, as_model.admission_b7); each vessel of M(t) gets its own table on
        boat/b<i>/levels, and fleet/as carries "p": null, "p_by" and "grants"
Baseline B4 is this service with --alpha 0 --C0 C_ref (constant prediction) and no outer loop.

  python -m fleetdt.as_service --dtp dtp.json --broker host:port [--epoch T0] [--log decisions.jsonl]
                               [--U u] [--alpha a] [--C0 c] [--policy alg1|b7]
                               [--outer-dta file.jsonl --outer-every steps]
                               [--outer-window W] [--outer-target 0.99]
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .as_model import DTP, admission, admission_b7, grant_cap, smooth
from .model import decode_demand, encode_levels, tcp_nodelay

log = logging.getLogger("fleetdt.as")

DEMAND_RE = re.compile(r"^boat/b(\d+)/demand$")
POLICIES = ("alg1", "b7")
SUBSCRIPTIONS = [("boat/+/demand", 1), ("fleet/throughput", 0), ("fleet/order", 1)]


class AS:
    """Pure logic of the AS (no broker). Times are Unix times on the GS clock; `now` supplies the time of
    an input whose arrival time is not given. Thread-safe: the MQTT thread feeds the inputs and the
    timing thread calls close_phase1."""

    def __init__(self, dtp: DTP, epoch: float, U: Optional[float] = None, alpha: Optional[float] = None,
                 C0: Optional[float] = None, now: Callable[[], float] = time.time, policy: str = "alg1"):
        if policy not in POLICIES:
            raise ValueError("policy must be one of %s" % ", ".join(POLICIES))
        self.policy = policy
        self.dtp = dtp
        self.epoch = float(epoch)
        self.U = dtp.U if U is None else float(U)
        self.alpha = dtp.alpha if alpha is None else float(alpha)
        self.C0 = dtp.C0 if C0 is None else float(C0)
        self.now = now
        self.lock = threading.RLock()
        self.index = {name: k + 1 for k, name in enumerate(dtp.topics)}
        self._demand: Dict[int, Dict[int, List[int]]] = {}   # t -> i -> D_i(t), tables that met the deadline
        self._backlog: Dict[int, Dict[int, int]] = {}        # t -> i -> Q_i(t) of those tables
        self._y: Dict[int, Tuple[float, float]] = {}          # t -> (y(t), arrival time)
        self._orders: List[Tuple[float, List[int], List[int]]] = []  # pending fleet/order: (arrival, sigma, N)
        sigma = self._indices(dtp.order) if dtp.order else list(range(1, dtp.L + 1))
        needed = self._indices(dtp.needed)
        if not self._valid(sigma, needed):
            raise ValueError("the order of the DTP is not a permutation of its topics")
        self.sigma, self.N = sigma, needed
        self.C_hat: Optional[float] = None   # C_hat(last_t)
        self.last_t: Optional[int] = None    # last step closed
        self.last: Optional[Dict] = None     # state of the last step closed (also for m = 0)

    # ------------------------------------------------------------------ step clock
    def start(self, t: int) -> float:
        return self.epoch + t * self.dtp.Ts

    def deadline(self, t: int) -> float:
        """End of phase 1 of step t: start(t) + T1."""
        return self.start(t) + self.dtp.T1

    def step_of(self, at: float) -> int:
        return math.floor((at - self.epoch) / self.dtp.Ts)

    # ------------------------------------------------------------------ inputs
    def _indices(self, names: Sequence) -> List[int]:
        out = []
        for v in names:
            if isinstance(v, int) and not isinstance(v, bool):
                out.append(v if 1 <= v <= self.dtp.L else -1)
            else:
                out.append(self.index.get(str(v), -1))
        return out

    def _valid(self, sigma: List[int], needed: List[int]) -> bool:
        return sorted(sigma) == list(range(1, self.dtp.L + 1)) and all(l in sigma for l in needed) \
            and len(set(needed)) == len(needed)

    def on_demand(self, i: int, t: int, d: Sequence[int], at: Optional[float] = None, Q: int = 0) -> bool:
        """Demand table D_i(t) with the backlog Q_i(t) of vessel i. Returns False when the table cannot enter
        M(t): L differs from the DTP, step t is already closed, or the table arrived at or after
        start(t) + T1."""
        with self.lock:
            at = self.now() if at is None else at
            if len(d) != self.dtp.L:
                return False
            if self.last_t is not None and t <= self.last_t:
                return False
            if at >= self.deadline(t) or t > self.step_of(at) + 1:
                return False
            self._demand.setdefault(t, {})[int(i)] = [int(v) for v in d]
            self._backlog.setdefault(t, {})[int(i)] = max(0, int(Q))
            return True

    def on_throughput(self, t: int, y: float, at: Optional[float] = None) -> None:
        """y(t) of the channel monitor; eq. (2) of step t+1 uses it when it arrives before start(t+1) + T1."""
        with self.lock:
            at = self.now() if at is None else at
            if self.last_t is not None and t + 1 <= self.last_t:
                return
            self._y[int(t)] = (float(y), at)

    def on_order(self, order: Sequence, needed: Sequence, at: Optional[float] = None) -> bool:
        """fleet/order: sigma and N with topic names (or 1-based indices). The operation control software
        ranks the needed topics first; the AS keeps the order as received. Returns False when sigma is
        not a permutation of the DTP topics or N is not a subset of sigma."""
        with self.lock:
            at = self.now() if at is None else at
            sigma, N = self._indices(order), self._indices(needed)
            if not self._valid(sigma, N):
                return False
            self._orders.append((at, sigma, N))
            return True

    # ------------------------------------------------------------------ phase 2
    def close_phase1(self, t: int) -> Optional[Dict]:
        """Closes M(t) and computes the decision of step t (eq. 2, eq. 3, Algorithm 1).
        Returns None when m(t) = 0 (nothing is published for step t)."""
        with self.lock:
            if self.last_t is not None and t <= self.last_t:
                raise ValueError("step %d already closed" % t)
            # eq. (2), once per step since the last step closed; a missing or late y(k-1) keeps C_hat(k-1)
            y_used = None
            if self.last_t is None:
                C = self.C0
            else:
                C = self.C_hat
                for k in range(self.last_t + 1, t + 1):
                    yk = self._y.get(k - 1)
                    y_used = yk[0] if yk is not None and yk[1] < self.deadline(k) else None
                    C = smooth(C, y_used, self.alpha, self.dtp.c_y)
            self.C_hat, self.last_t = C, t
            for k in [k for k in self._y if k < t]:
                del self._y[k]
            # sigma_t and N(t): last fleet/order received before start(t) + T1
            dl = self.deadline(t)
            before = [o for o in self._orders if o[0] < dl]
            if before:
                _, self.sigma, self.N = before[-1]
                self._orders = [o for o in self._orders if o[0] >= dl]
            D = self._demand.pop(t, {})
            Qi = self._backlog.pop(t, {})
            for k in [k for k in self._demand if k < t]:
                del self._demand[k]
            for k in [k for k in self._backlog if k < t]:
                del self._backlog[k]
            M = sorted(D)
            Q = int(sum(Qi.get(i, 0) for i in M))          # Q(t): backlog of the vessels of M(t)
            m = len(M)
            K_hat = self.dtp.K_hat(C)                          # C_hat(t) (T3 - T_m) / Ts
            cap = grant_cap(C, self.U, self.dtp.T3, self.dtp.Ts, Q, self.dtp.T_m)   # max(0, U K_hat - Q)
            # D (the demand tables of M(t)) and y_used (the y(t - 1) of eq. (2), None when missing or late)
            # make the decision reproducible offline (emu/experiments/conformance.py)
            dec = {"t": t, "C_hat": C, "K_hat": K_hat, "cap": cap, "b": None, "U": self.U, "alpha": self.alpha,
                   "c_y": self.dtp.c_y, "T_m": self.dtp.T_m,
                   "m": m, "p": None, "M": M, "order": list(self.sigma), "needed": list(self.N),
                   "D": {str(i): list(D[i]) for i in M}, "y_used": y_used,
                   "Qi": {str(i): int(Qi.get(i, 0)) for i in M}, "Q": Q}
            self.last = dec
            if m == 0:
                return None
            b = cap / m                                               # eq. (3)
            dec["b"] = b
            if self.policy == "b7":                                   # baseline B7
                p_by, g = admission_b7(D, self.sigma, self.N, b)
                dec["p_by"] = {i: p_by[i] for i in M}
                dec["grants"] = {i: g[i] for i in M}
                self._admitted(dec, D, {i: p_by[i] for i in M})
                return dec
            dec["p"] = admission(D, self.sigma, self.N, b)            # Algorithm 1
            self._admitted(dec, D, {i: dec["p"] for i in M})
            return dec

    @staticmethod
    def _admitted(dec: Dict, D: Dict[int, Sequence[int]], p_by: Dict[int, Sequence[int]]) -> None:
        """a_i(t) = sum of d_il(t) over the topics of level 1 or 2 of vessel i (the final a_i of Algorithm 1)
        and A(t) = sum_i a_i(t), the bytes that phase 3 of step t receives at most; the backlog predictor of
        the outer loop (C5, fleetdt.outer.backlog) replays them."""
        a = {str(i): int(sum(v for v, lev in zip(D[i], p_by[i]) if lev > 0)) for i in p_by}
        dec["a"], dec["A"] = a, int(sum(a.values()))

    def set_params(self, U: Optional[float] = None, alpha: Optional[float] = None) -> None:
        with self.lock:
            if U is not None:
                self.U = float(U)
            if alpha is not None:
                self.alpha = float(alpha)


def decode_throughput(payload: bytes) -> Tuple[int, float]:
    """fleet/throughput: JSON {"t", "y", ...} or binary !Id."""
    if payload[:1] == b"{":
        o = json.loads(payload)
        return int(o["t"]), float(o["y"])
    t, y = struct.unpack("!Id", payload)
    return t, y


class Service:
    """MQTT wrapper of the AS. The phases follow absolute deadlines: the wall clock is read once and
    anchored to time.monotonic(), and the deadline of step t is mono0 + (start(t) + T1 - wall0), so the
    schedule accumulates no drift."""

    def __init__(self, dtp: DTP, host: str = "localhost", port: int = 1883, epoch: Optional[float] = None,
                 U: Optional[float] = None, alpha: Optional[float] = None, log_path: Optional[str] = None,
                 outer_dta: Optional[str] = None, outer_every: int = 0, outer_window: Optional[int] = None,
                 outer_target: float = 0.99, client_id: Optional[str] = None, spin: float = 0.001,
                 C0: Optional[float] = None, policy: str = "alg1"):
        import paho.mqtt.client as mqtt
        self.mono0, self.wall0 = time.monotonic(), time.time()
        epoch = dtp.t0 if epoch is None else epoch
        self.dtp = dtp
        self.core = AS(dtp, epoch, U=U, alpha=alpha, C0=C0, now=self.now, policy=policy)
        self.host, self.port = host, port
        self.spin = spin
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self.outer_dta, self.outer_every = outer_dta, int(outer_every or 0)
        self.outer_window, self.outer_target = outer_window, outer_target
        self._outer_busy = False
        self._outer_result: Optional[Dict] = None
        self._outer_pid: Optional[int] = None
        self._outer_out: Optional[str] = None
        self.delays: List[float] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = threading.Event()
        self.subscribed = threading.Event()   # SUBACK of SUBSCRIPTIONS on the current connection
        self._sub_mid: Optional[int] = None
        cid = client_id or "fleetdt-as-%d-%s" % (os.getpid(), uuid.uuid4().hex[:8])
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid, clean_session=True)
        tcp_nodelay(self.client)
        self.client.max_inflight_messages_set(1000)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_subscribe = self._on_subscribe
        self.client.on_message = self._on_message

    # ------------------------------------------------------------------ clock
    def now(self) -> float:
        """GS wall clock derived from the monotonic clock and the anchor."""
        return self.wall0 + (time.monotonic() - self.mono0)

    def _mono(self, wall: float) -> float:
        return self.mono0 + (wall - self.wall0)

    def _sleep_until(self, mono_deadline: float) -> None:
        rest = mono_deadline - time.monotonic()
        if rest > self.spin:
            self._stop.wait(rest - self.spin)
        while time.monotonic() < mono_deadline and not self._stop.is_set():
            pass  # short spin for the last `spin` seconds

    # ------------------------------------------------------------------ MQTT
    def _on_connect(self, client, userdata, flags, rc, props=None) -> None:
        self.subscribed.clear()
        # the SUBACK is handled by the network thread after this callback returns, so the mid is set first
        _, self._sub_mid = client.subscribe(SUBSCRIPTIONS)
        self._connected.set()

    def _on_disconnect(self, client, userdata, flags, rc, props=None) -> None:
        self._connected.clear()
        self.subscribed.clear()

    def _on_subscribe(self, client, userdata, mid, reason_codes, props=None) -> None:
        if mid != self._sub_mid:
            return
        bad = [str(rc) for rc in (reason_codes or []) if getattr(rc, "is_failure", False) or rc == 0x80]
        if bad:
            log.warning("the broker refused subscriptions: %s", bad)
        self.subscribed.set()

    def _on_message(self, client, userdata, msg) -> None:
        try:
            m = DEMAND_RE.match(msg.topic)
            if m:
                t, Q, d = decode_demand(msg.payload)
                self.core.on_demand(int(m.group(1)), t, d, Q=Q)
            elif msg.topic == "fleet/throughput":
                t, y = decode_throughput(msg.payload)
                self.core.on_throughput(t, y)
            elif msg.topic == "fleet/order":
                o = json.loads(msg.payload)
                if not self.core.on_order(o.get("order", []), o.get("needed", [])):
                    log.warning("fleet/order rejected: %s", msg.payload[:200])
        except (ValueError, KeyError, TypeError, struct.error) as e:
            log.warning("malformed message on %s: %s", msg.topic, e)

    def connect(self, timeout: float = 5.0) -> None:
        """Connects and returns after the CONNACK and the SUBACK of the subscriptions of the AS."""
        end = time.monotonic() + timeout
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        if not self._connected.wait(timeout):
            raise TimeoutError("no CONNACK from %s:%d" % (self.host, self.port))
        if not self.subscribed.wait(max(0.0, end - time.monotonic())):
            raise TimeoutError("no SUBACK from %s:%d" % (self.host, self.port))

    # ------------------------------------------------------------------ outer loop
    def _outer_start(self) -> None:
        """Starts fleetdt.outer.run_job in a child process that writes its result to a temporary file.
        posix_spawn sets SCHED_OTHER before the exec, so a child of an AS under SCHED_FIFO never runs at
        the real-time priority; the child lowers itself to nice 10 before its imports."""
        fd, out = tempfile.mkstemp(prefix="fleetdt-outer-", suffix=".json")
        os.close(fd)
        os.unlink(out)
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = dict(os.environ)
        env["PYTHONPATH"] = pkg + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        args = [sys.executable, "-c", "import os, sys; os.nice(10); from fleetdt.outer import run_job; "
                "run_job(sys.argv[1], float(sys.argv[2]), int(sys.argv[3]) or None, sys.argv[4])",
                str(self.outer_dta), repr(float(self.outer_target)), str(int(self.outer_window or 0)), out]
        try:
            try:
                pid = os.posix_spawn(sys.executable, args, env,
                                     setscheduler=(os.SCHED_OTHER, os.sched_param(0)))
            except (AttributeError, NotImplementedError, TypeError):
                pid = subprocess.Popen(args, env=env, stdin=subprocess.DEVNULL).pid
        except OSError as e:
            log.warning("outer loop failed: %s", e)
            return
        self._outer_pid, self._outer_out, self._outer_busy = pid, out, True

    def _outer_poll(self) -> None:
        """Takes the result of a finished child (non-blocking)."""
        if not self._outer_busy or self._outer_pid is None:
            return
        try:
            pid, status = os.waitpid(self._outer_pid, os.WNOHANG)
        except ChildProcessError:
            pid, status = self._outer_pid, -1
        if pid == 0:
            return
        out, self._outer_pid, self._outer_busy = self._outer_out, None, False
        try:
            with open(out, encoding="utf-8") as f:
                msg = json.load(f)
        except (OSError, ValueError) as e:
            msg = {"error": "no result (exit status %s): %s" % (status, e)}
        finally:
            try:
                os.unlink(out)
            except OSError:
                pass
        if msg.get("error"):
            log.warning("outer loop failed: %s", msg["error"])
        elif msg.get("result") is not None:
            self._outer_result = msg["result"]

    def _outer_kill(self) -> None:
        if self._outer_pid is None:
            return
        try:
            os.kill(self._outer_pid, signal.SIGTERM)
            os.waitpid(self._outer_pid, 0)
        except (OSError, ChildProcessError):
            pass
        for p in (self._outer_out, (self._outer_out or "") + ".tmp"):
            try:
                os.unlink(p)
            except OSError:
                pass
        self._outer_pid, self._outer_busy = None, False

    def _outer_tick(self, t: int) -> None:
        self._outer_poll()
        res, self._outer_result = self._outer_result, None
        if res is not None:
            self.core.set_params(U=res.get("U_star"), alpha=res.get("alpha_star"))
            self._write({"event": "outer", "t": t, "alpha_star": res.get("alpha_star"),
                         "U_star": res.get("U_star"), "mse": res.get("mse"), "n": res.get("n_y")})
            log.info("outer loop at step %d: alpha=%.4f U=%s", t, self.core.alpha, res.get("U_star"))
        if self.outer_dta and self.outer_every > 0 and t % self.outer_every == 0 and not self._outer_busy:
            self._outer_start()

    # ------------------------------------------------------------------ main loop
    def _write(self, rec: Dict) -> None:
        if self.log_file:
            self.log_file.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self.log_file.flush()

    def _publish(self, dec: Dict) -> None:
        if dec.get("p_by") is not None:      # B7: one table per vessel
            for i in dec["M"]:
                self.client.publish("boat/b%d/levels" % i, encode_levels(dec["t"], dec["p_by"][i]), qos=1)
        else:
            payload = encode_levels(dec["t"], dec["p"])
            for i in dec["M"]:
                self.client.publish("boat/b%d/levels" % i, payload, qos=1)
        keys = ("t", "C_hat", "K_hat", "cap", "b", "U", "alpha", "m", "p", "M", "order", "needed", "p_by", "grants", "a",
                "A", "Q", "Qi")
        self.client.publish("fleet/as", json.dumps({k: dec[k] for k in keys if k in dec}, separators=(",", ":")),
                            qos=0)

    def run(self, steps: Optional[int] = None) -> None:
        """Runs phase 2 of each step until stop() (or for `steps` steps)."""
        Ts = self.dtp.Ts
        t = max(0, math.floor((self.now() - self.core.epoch - self.dtp.T1) / Ts) + 1)
        done = 0
        while not self._stop.is_set() and (steps is None or done < steps):
            dl = self._mono(self.core.deadline(t))
            self._sleep_until(dl)
            if self._stop.is_set():
                break
            wake = time.monotonic() - dl
            if wake >= Ts:  # overrun: the steps already over advance eq. (2) but publish nothing
                skip = int(wake // Ts)
                for k in range(t, t + skip):
                    self.core.close_phase1(k)
                    self._write({**self.core.last, "published": False, "skipped": True})
                log.warning("overrun of %.3f s: skipped steps %d..%d", wake, t, t + skip - 1)
                t += skip
                dl = self._mono(self.core.deadline(t))
                wake = time.monotonic() - dl
            dec = self.core.close_phase1(t)
            if dec is not None:
                self._publish(dec)
            pub = time.monotonic() - dl
            if dec is not None:
                self.delays.append(pub)
                if pub > self.dtp.T2:
                    log.warning("step %d: p(t) published %.1f ms after start(t)+T1", t, pub * 1e3)
            self._write({**self.core.last, "published": dec is not None, "wake_delay": wake, "pub_delay": pub})
            self._outer_tick(t)
            t += 1
            done += 1

    def start(self) -> None:
        """Connects and runs the timing loop in a background thread."""
        self.connect()
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._outer_kill()
        self.client.loop_stop()
        self.client.disconnect()
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def delay_summary(self) -> Dict:
        """Delay between start(t) + T1 and the end of the publication of p(t), in seconds."""
        if not self.delays:
            return {"n": 0}
        s = sorted(self.delays)
        q = lambda f: s[min(len(s) - 1, int(f * len(s)))]
        return {"n": len(s), "mean": sum(s) / len(s), "p50": q(0.5), "p99": q(0.99), "max": s[-1],
                "over_T2": sum(1 for v in s if v > self.dtp.T2)}


def _broker(s: str) -> Tuple[str, int]:
    host, _, port = s.partition(":")
    return host or "localhost", int(port or 1883)


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.as_service", description="Allocation system (AS) of Fleet-DT")
    ap.add_argument("--dtp", required=True, help="DTP file (dtp.json)")
    ap.add_argument("--broker", default="localhost:1883", help="GS broker host:port")
    ap.add_argument("--epoch", type=float, default=None,
                    help="Unix time of the start of step 0 (default: the DTP epoch, or 2026-01-01T00:00:00Z when it is null)")
    ap.add_argument("--log", default=None, help="JSON-lines file of the decisions")
    ap.add_argument("--U", type=float, default=None, help="target utilization (overrides the DTP)")
    ap.add_argument("--alpha", type=float, default=None, help="gain of eq. (2) (overrides the DTP)")
    ap.add_argument("--C0", type=float, default=None,
                    help="initial prediction C_hat(0), bytes per step (overrides the DTP; B4: C_ref with --alpha 0)")
    ap.add_argument("--policy", choices=POLICIES, default="alg1",
                    help="alg1: Algorithm 1 (one table for the fleet); b7: per-vessel grants with redistribution")
    ap.add_argument("--outer-dta", default=None, help="DTA file written by the VE, input of the outer loop")
    ap.add_argument("--outer-every", type=int, default=0, help="steps between runs of the outer loop")
    ap.add_argument("--outer-window", type=int, default=None, help="last W steps of the DTA used by the outer loop")
    ap.add_argument("--outer-target", type=float, default=0.99, help="coverage target gamma of the outer loop")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    dtp = DTP.load(a.dtp)
    epoch = a.epoch if a.epoch is not None else dtp.t0
    print("epoch = %.3f" % epoch, flush=True)
    host, port = _broker(a.broker)
    svc = Service(dtp, host, port, epoch=epoch, U=a.U, alpha=a.alpha, log_path=a.log,
                  outer_dta=a.outer_dta, outer_every=a.outer_every, outer_window=a.outer_window,
                  outer_target=a.outer_target, C0=a.C0, policy=a.policy)
    svc.connect()
    try:
        svc.run()
    except KeyboardInterrupt:
        pass
    finally:
        svc.stop()
        print(json.dumps({"publication_delay_s": svc.delay_summary()}), file=sys.stderr)


if __name__ == "__main__":
    main()
