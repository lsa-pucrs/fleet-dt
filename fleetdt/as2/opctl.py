"""Operation-control stub: publishes sigma_t and N_i(t) on fleet/order once per step (phase 1) and the
goals of each vessel on boat/b<i>/goals at the DTP goals rate (Section III-C, SPEC Section 4).

Schedule: the operation alternates between survey mode (the output topics, e.g. video, are needed from
the designated vessels and rank right after the needed telemetry) and transit mode (no output topic is
needed; the output topics rank last), switching every `period` steps.

  python -m fleetdt.as2.opctl --dtp dtp_v2.json --broker host:port [--period 80] [--survey-vessels all|1,3]
         [--start survey|transit] [--lead 0.005] [--t0-step S --seed N]

Experiments (paper Section VI-C): --t0-step S counts the periods from step S, and --seed N designates, in
each survey period k, one vessel drawn from the survey vessels by (N, k) alone, so the runs of one seed
follow the same schedule.
"""
from __future__ import annotations

import argparse
import json
import logging
import random
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .dtp import DTP

log = logging.getLogger("fleetdt.as2.opctl")


class Schedule:
    def __init__(self, dtp: DTP, period: int = 80, survey_vessels: Optional[Sequence[int]] = None,
                 start: str = "survey", outputs: Optional[Sequence[str]] = None, t0: int = 1,
                 seed: Optional[int] = None):
        if start not in ("survey", "transit"):
            raise ValueError("start must be survey or transit")
        self.dtp, self.period, self.start = dtp, max(1, int(period)), start
        self.t0, self.seed = int(t0), seed
        self.survey_vessels = list(dtp.vessels if survey_vessels is None else survey_vessels)
        names = [tp.name for tp in dtp.topics]
        self.outputs = list(outputs) if outputs is not None else [tp.name for tp in dtp.topics if tp.output]
        for o in self.outputs:
            if o not in names:
                raise ValueError("unknown topic %r" % o)

    def mode(self, t: int) -> str:
        k = ((t - self.t0) // self.period) % 2
        other = "transit" if self.start == "survey" else "survey"
        return self.start if k == 0 else other

    def designated(self, t: int) -> List[int]:
        """the vessels whose output topics are needed in the survey period of step t: all survey vessels,
        or, with a seed, one of them drawn by (seed, period index)"""
        if self.seed is None:
            return self.survey_vessels
        k = (t - self.t0) // self.period
        return [random.Random("%d:survey:%d" % (self.seed, k)).choice(self.survey_vessels)]

    def plan(self, t: int) -> Tuple[List[str], Dict[str, List[str]], str]:
        """(sigma_t as names, {"<i>": N_i(t) as names}, mode)."""
        dtp = self.dtp
        base = [dtp.topics[l].name for l in dtp.order]
        req = [dtp.topics[l].name for l in dtp.order if l in dtp.R]
        nd = [dtp.topics[l].name for l in dtp.order if l in set(dtp.needed) and l not in dtp.R]
        mode = self.mode(t)
        if mode == "survey":
            head = req + [n for n in nd if n not in self.outputs] + [n for n in base if n in self.outputs and n not in req]
            needed = {}
            des = self.designated(t)
            for i in dtp.vessels:
                extra = self.outputs if i in des else []
                needed[str(i)] = req + [n for n in nd if n not in self.outputs] + [o for o in extra if o not in req]
        else:
            head = req + [n for n in nd if n not in self.outputs]
            needed = {str(i): list(head) for i in dtp.vessels}
        rest = [n for n in base if n not in head and n not in self.outputs]
        tail = [n for n in base if n in self.outputs and n not in head]
        order = head + rest + tail
        return order, needed, mode


def goals_payload(dtp: DTP, i: int, t: int, mode: str, rng: random.Random) -> bytes:
    """A goal of at most goals.s bytes: next waypoint, speed and mode."""
    o = {"t": t, "i": i, "mode": mode, "wp": [round(-30.03 + rng.uniform(-0.01, 0.01), 6),
                                              round(-51.22 + rng.uniform(-0.01, 0.01), 6)],
         "v": round(rng.uniform(0.5, 2.0), 2)}
    b = json.dumps(o, separators=(",", ":")).encode()
    if len(b) > dtp.goals_s:
        b = json.dumps({"t": t, "i": i}, separators=(",", ":")).encode()[:dtp.goals_s]
    return b


class OpCtl:
    def __init__(self, dtp: DTP, schedule: Schedule, host: str = "localhost", port: int = 1883, lead: float = 0.005,
                 seed: int = 1, log_path: Optional[str] = None, first: Optional[int] = None):
        from .service import _client
        self.dtp, self.schedule, self.lead = dtp, schedule, lead
        self.client = _client("opctl")
        self.client.connect(host, port, keepalive=30)
        self.client.loop_start()
        self._stop = threading.Event()
        self.rng = random.Random(seed)
        self.sent = 0
        self.first = first
        self.log = open(log_path, "a") if log_path else None

    def order_message(self, t: int) -> bytes:
        order, needed, mode = self.schedule.plan(t)
        return json.dumps({"t": t, "order": order, "needed": needed, "mode": mode}, separators=(",", ":")).encode()

    def log_line(self, t: int) -> str:
        order, needed, mode = self.schedule.plan(t)
        return json.dumps({"t": t, "order": order, "needed": needed, "mode": mode, "at": time.time()},
                          separators=(",", ":"))

    def run(self, steps: Optional[int] = None) -> None:
        """Publishes the order of step t at start(t) - lead and the goals every 1/goals_rate seconds."""
        dtp = self.dtp
        t = dtp.step_of(time.time() + self.lead) + 1
        if self.first is not None:
            t = max(t, self.first)
        goal_every = max(1, int(round(1.0 / (dtp.goals_rate * dtp.Ts)))) if dtp.goals_rate > 0 else 0
        done = 0
        while not self._stop.is_set() and (steps is None or done < steps):
            when = dtp.start(t) - self.lead
            rest = when - time.time()
            if rest > 0 and self._stop.wait(rest):
                break
            self.client.publish("fleet/order", self.order_message(t), qos=1)
            if self.log is not None:
                self.log.write(self.log_line(t) + "\n")
                self.log.flush()
            if goal_every and t % goal_every == 0:
                mode = self.schedule.mode(t)
                for i in dtp.vessels:
                    self.client.publish("boat/b%d/goals" % i, goals_payload(dtp, i, t, mode, self.rng), qos=1)
            self.sent += 1
            t += 1
            done += 1

    def stop(self) -> None:
        self._stop.set()
        self.client.loop_stop()
        self.client.disconnect()


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.as2.opctl", description="Operation-control stub of Fleet-DT v2")
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--broker", default="localhost:1883")
    ap.add_argument("--period", type=int, default=80, help="steps between switches of survey and transit")
    ap.add_argument("--survey-vessels", default="all", help="'all' or comma-separated vessel indices")
    ap.add_argument("--start", default="survey", choices=("survey", "transit"))
    ap.add_argument("--lead", type=float, default=0.005, help="seconds before start(t) to publish the order")
    ap.add_argument("--steps", type=int, default=None)
    ap.add_argument("--t0-step", type=int, default=1, help="step from which the periods count")
    ap.add_argument("--first", type=int, default=None, help="first step to publish (default: the next one)")
    ap.add_argument("--seed", type=int, default=None, help="draws one designated vessel per survey period")
    ap.add_argument("--log", default=None, help="JSON lines of the published orders")
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    sv = None if a.survey_vessels == "all" else [int(x) for x in a.survey_vessels.split(",") if x]
    sch = Schedule(dtp, a.period, sv, a.start, t0=a.t0_step, seed=a.seed)
    host, _, port = a.broker.partition(":")
    oc = OpCtl(dtp, sch, host or "localhost", int(port or 1883), a.lead, seed=a.seed or 1, log_path=a.log,
               first=a.first)
    try:
        oc.run(a.steps)
    except KeyboardInterrupt:
        pass
    finally:
        oc.stop()


if __name__ == "__main__":
    main()
