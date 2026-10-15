"""One step of the AS without MQTT: phase 2 of step t (Algorithm 1 with eq. pred, eq. aci, eq. grant),
preceded by the closing of step t-1 (f(t-1), e(t-1), kappa(t-1), z(t-1)).

Inputs arrive through on_demand, on_order, on_bridge, the monitor store and the acceptor; phase2(t)
returns the tables to publish and the AS record (SPEC Section 4.4, with extra fields for the analysis).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

from .aci import ACI
from .admission import (AdditiveBudget, Demand, HeldBytes, SlotBudget, Unit, algorithm1, build_units, ipact,
                        slots, tables, tdma_slots)
from .airtime import Cost, Phy
from .bianchi import EtaTable
from .failure import Acceptor, EdgeAcc, indicator
from .monitor import MonitorStore
from .predictor import Predictor, Scores

ADMISSION_MODES = ("fleetdt", "b4", "oracle", "slotted", "b5")
PASS_MODES = ("b1", "b2", "b3", "tdma")
# B6 (addendum to the frozen protocol): IPACT with limited service, per-vessel grants without prediction
GRANT_MODES = ("ipact",)
MODES = ADMISSION_MODES + PASS_MODES + GRANT_MODES
# point predictors of kappa(t) (experiment E2): eq. (pred), persistence kappa_hat(t) = kappa(t-1),
# exponential smoothing of kappa with the gain pred_gain, and the mean of the calibration runs
PREDICTORS = ("eq", "persist", "es", "calmean")


def us(x: float) -> int:
    return int(round(x * 1e6))


@dataclass
class StepInfo:
    t: int
    Mt: List[int]
    A: List[Unit]
    kappa_hat: float
    Z: Optional[float]
    alpha: float
    K: Optional[float]
    sent: Set[int]
    cost_A: float
    slots: Optional[Dict[int, Tuple[int, int]]] = None     # slotted: slot of each vessel (us)
    shared_end: Optional[int] = None                       # slotted: end of the shared slot (us)
    kappa_hat2: Optional[float] = None                     # slotted: kappa_hat_2(t)


class ASCore:
    def __init__(self, dtp, mode: str = "fleetdt", z0: float = 1.0, oracle: Optional[Dict[int, float]] = None,
                 initial_scores: Iterable[float] = (), b_hat0: float = 0.0, score_window: Optional[int] = None,
                 z_empty: float = 1.0, assume_connected: bool = True, count_tab_unsent: bool = True,
                 min_cover: float = 0.5, eta: Optional[Callable[[int], float]] = None,
                 acc_log: Optional[Callable[[Tuple], None]] = None, out_log: Optional[Callable[[Dict], None]] = None,
                 predictor: str = "eq", pred_gain: float = 0.25, cal_kappa: Optional[float] = None):
        if mode not in MODES:
            raise ValueError("mode %r not in %s" % (mode, MODES))
        if predictor not in PREDICTORS:
            raise ValueError("predictor %r not in %s" % (predictor, PREDICTORS))
        if predictor == "calmean" and cal_kappa is None:
            raise ValueError("predictor calmean needs cal_kappa")
        if not 0 < pred_gain <= 1:
            raise ValueError("pred_gain must be in (0, 1]")
        self.dtp, self.mode, self.z0 = dtp, mode, float(z0)
        self.predictor, self.pred_gain, self.cal_kappa = predictor, float(pred_gain), cal_kappa
        self.kh_alt: Optional[float] = cal_kappa
        self.oracle = oracle or {}
        self.phy = Phy(dtp.phy)
        self.cost = Cost.from_dtp(dtp)
        self.eta = eta or EtaTable.from_dtp(dtp)
        self.pred = Predictor(dtp.lam, self.eta, dtp.M, b_hat0)
        self.scores = Scores(initial_scores, score_window, z_empty)
        self.aci = ACI(dtp.beta, dtp.gamma, dtp.alpha1)
        self.monitor = MonitorStore(self.phy)
        self.acceptor = Acceptor(dtp, log=acc_log, out_log=out_log)
        self.held = HeldBytes(dtp)
        self.s_held = max(tp.s + tp.h_P for tp in dtp.topics)    # one held message with its header
        self.assume_connected = assume_connected
        self.count_tab_unsent = count_tab_unsent
        self.min_cover = min_cover
        self.lock = threading.RLock()
        self.demands: Dict[int, Dict[int, Demand]] = {}
        self.orders: Dict[int, Tuple[List[int], Dict[int, Set[int]]]] = {}
        self.bridge: Dict[int, bool] = {}
        self.prev: Optional[StepInfo] = None
        self.last_t: Optional[int] = None
        self.late_demands = 0
        self.bad_inputs = 0
        self.f_hist: List[int] = []
        self.alpha_hist: List[float] = []
        default_rank = [0] * dtp.L
        for r, l in enumerate(dtp.order):
            default_rank[l] = r
        self.default_rank = default_rank

    # ------------------------------------------------------------------ inputs
    def on_demand(self, i: int, obj: Dict, at: float) -> bool:
        """Demand of SPEC 4.1 from boat/b<i>/demand, received at `at` (GS clock). Enters M(t) when it
        arrives in [start(t) - eps, start(t) + T1) and step t is not closed."""
        dem = Demand.parse(obj, self.dtp, at)
        if dem.v != i or i not in self.dtp.vessels:
            raise ValueError("demand of vessel %d on the topic of vessel %d" % (dem.v, i))
        with self.lock:
            if (self.last_t is not None and dem.t <= self.last_t) or not self.dtp.demand_open(dem.t, at):
                self.late_demands += 1
                return False
            self.demands.setdefault(dem.t, {})[i] = dem
            return True

    def on_order(self, obj: Dict, at: Optional[float] = None) -> None:
        """fleet/order {"t", "order":[names], "needed":{"<i>":[names]}}; sigma must start with R, and
        N_i(t) is extended by R."""
        dtp = self.dtp
        t = int(obj["t"])
        sigma = dtp.topic_indices(obj.get("order") or [dtp.topics[l].name for l in dtp.order])
        if sorted(sigma) != list(range(dtp.L)) or set(sigma[:len(dtp.required)]) != dtp.R:
            raise ValueError("fleet/order: order is not a permutation that starts with R")
        rank = [0] * dtp.L
        for r, l in enumerate(sigma):
            rank[l] = r
        needed = {}
        for k, names in (obj.get("needed") or {}).items():
            needed[int(k)] = set(dtp.topic_indices(names)) | dtp.R
        with self.lock:
            self.orders[t] = (rank, needed)
            for old in [s for s in self.orders if s < t - 64]:
                del self.orders[old]

    def order_for(self, t: int):
        with self.lock:
            ks = [s for s in self.orders if s <= t]
            if not ks:
                return self.default_rank, {}
            return self.orders[max(ks)]

    def on_bridge(self, i: int, connected: bool) -> None:
        with self.lock:
            self.bridge[i] = bool(connected)

    def connected(self) -> Set[int]:
        with self.lock:
            return set(i for i in self.dtp.vessels if self.bridge.get(i, self.assume_connected))

    # ------------------------------------------------------------------ closing of step s = t - 1
    def _close(self, s: int, D: Dict[int, Demand]) -> Dict:
        dtp, prev = self.dtp, self.prev
        reports = {i: d.rep for i, d in D.items()}
        acc = self.acceptor.step(s)
        fs = indicator(s, prev.Mt, reports, acc, dtp.L, sent_tables=prev.sent,
                       count_tab_unsent=self.count_tab_unsent)
        w, wend = dtp.window(s)
        held = [i for i, d in D.items() if d.B["x0"] + d.B["x2"] > 0 or d.B["n0"] + d.B["n2"] > 0]
        cap = self.monitor.capacity(us(w), us(wend), us(dtp.end(s)), held)
        kappa, e = cap["kappa"], cap["e"]
        z = None
        zs = None
        if prev.slots is not None:
            # slotted variant (Def. tdma): Lemma 2 applies to each slot with kappa_j(t), so the score of the
            # step is the smallest kappa_j / kappa_hat_j over the slots with a busy period of at least
            # T_min (shared slot: kappa_hat(t); slot of a vessel: kappa_hat_2(t))
            zs = {}
            parts = [(None, us(w), prev.shared_end, prev.kappa_hat)]
            parts += [(i, a, b, prev.kappa_hat2) for i, (a, b) in sorted(prev.slots.items())]
            for i, a, b, ref in parts:
                if b is None or b <= a or not ref:
                    continue
                c = self.monitor.slot_capacity(a, b, us(dtp.end(s)), i)
                if c["kappa"] is not None and (c["e"] - a) * 1e-6 >= dtp.T_min - 1e-9:
                    zs["0" if i is None else str(i)] = c["kappa"] / ref
            if zs:
                z = min(zs.values())
                self.scores.add(z)
        elif kappa is not None and e is not None and (e - us(w)) * 1e-6 >= dtp.T_min - 1e-9 and prev.kappa_hat > 0:
            z = kappa / prev.kappa_hat
            self.scores.add(z)
        accd = {str(i): [acc.get(i, {}).get(l, EdgeAcc()).n_in for l in range(dtp.L)] for i in dtp.vessels}
        dd = {str(i): [acc.get(i, {}).get(l, EdgeAcc()).x_in for l in range(dtp.L)] for i in dtp.vessels}
        late = {str(i): [acc.get(i, {}).get(l, EdgeAcc()).n_late for l in range(dtp.L)] for i in dtp.vessels}
        return {"f": fs.f, "reasons": [list(r) for r in fs.reasons], "kappa": kappa, "e": e, "w": us(w),
                "z": z, "z_slots": zs, "n": cap["n"], "hold": cap["hold"], "T_fleet": cap["T_fleet"],
                "cover": cap["cover"],
                "acc": accd, "d": dd, "late": late,
                "rep": {str(i): r for i, r in reports.items()}}

    def _pending_pubrel(self, i: int, D: Dict[int, Demand], bbar: Dict, t: int) -> int:
        prev = self.prev
        if prev is None or prev.t != t - 1:
            return 0
        R = self.dtp.required
        dem = D.get(i)
        if dem is not None and dem.rep is not None and dem.rep["t"] == t - 1:
            rel = sum(sum(dem.rep["rel"][l]) for l in R if l < len(dem.rep["rel"]))
        else:
            rel = sum(u.n for u in prev.A if u.i == i and u.q == 2)
        acc = 0
        for l in R:
            e = self.acceptor.get(i, l, t - 1)
            acc += e.n_in + e.n_late
        return int(max(0, rel - acc - bbar.get("n2", 0)))

    def _kappa_hat(self, kh_eq: float, close: Optional[Dict]) -> float:
        """kappa_hat(t) of the configured predictor; kappa(t-1) enters only with e - w >= T_min (the
        condition of a score)."""
        if self.predictor == "eq":
            return kh_eq
        if self.predictor == "calmean":
            return float(self.cal_kappa)
        k = None
        if close is not None and close.get("kappa") is not None and close.get("e") is not None and \
                (close["e"] - close["w"]) * 1e-6 >= self.dtp.T_min - 1e-9:
            k = float(close["kappa"])
        if self.kh_alt is None:
            self.kh_alt = kh_eq            # no calibration value: start from eq. (pred)
        if k is not None:
            if self.predictor == "persist":
                self.kh_alt = k
            else:
                self.kh_alt = (1.0 - self.pred_gain) * self.kh_alt + self.pred_gain * k
        return self.kh_alt

    # ------------------------------------------------------------------ phase 2 of step t
    def phase2(self, t: int, now: Optional[float] = None, publish: bool = True) -> Tuple[Dict[int, Dict], Dict]:
        """Phase 2 of step t. publish=False: the caller will not publish the tables (overrun)."""
        c0 = time.perf_counter()
        dtp, mode = self.dtp, self.mode
        with self.lock:
            if self.last_t is not None and t <= self.last_t:
                raise ValueError("step %d already closed" % t)
            D = self.demands.pop(t, {})
            for s in [s for s in self.demands if s < t]:
                del self.demands[s]
            self.last_t = t
        Mt = sorted(D)
        conn = self.connected()

        # step t-1: f, e, kappa, z; level alpha_t (eq. aci)
        close = None
        if self.prev is not None:
            for s in range(self.prev.t + 1, t - 1):    # steps without phase 2: no table was sent
                self.alpha_hist.append(self.aci.alpha)
                self.f_hist.append(1)
                self.aci.update(1)
            if self.prev.t == t - 1:
                close = self._close(t - 1, D)
                self.alpha_hist.append(self.aci.alpha)
                self.f_hist.append(close["f"])
                self.aci.update(close["f"])
        alpha = self.aci.alpha
        self.acceptor.prune(t - 4)

        # eq. (pred); the other point predictors of E2 replace kappa_hat(t) and keep b_hat(t) in the record
        q0, q1 = dtp.quiet(t)
        mb = self.monitor.busy_fraction(us(q0), us(q1), self.min_cover)
        kh_eq = self.pred.step(mb["b"])
        eta = self.eta(dtp.M + 1)
        kh = self._kappa_hat(kh_eq, close)

        # Z and the bound Z kappa_hat
        if mode == "b4":
            Z = self.z0
        elif mode == "oracle":
            kt = self.oracle.get(t)
            Z = (kt / kh if kh > 0 else 0.0) if kt is not None else self.scores.Z(alpha)
        else:
            Z = self.scores.Z(alpha)
        Zk = Z * kh

        # held bytes and eq. (grant)
        bbar = self.held.update(D, conn)
        rates = {i: self.monitor.rates(i, self.phy.rate_init) for i in dtp.vessels}
        cB = {}
        npr = {}
        for i in dtp.vessels:
            if i in conn:
                npr[i] = self._pending_pubrel(i, D, bbar[i], t)
                cB[i] = self.cost.held(bbar[i], rates[i][0], rates[i][1], npr[i], self.s_held)
            else:
                cB[i] = 0.0
        sum_cB = sum(cB.values())
        K = Zk * dtp.T_W - sum_cB

        # units and Algorithm 1
        rank, needed = self.order_for(t)
        cost = self.cost
        hP = [tp.h_P for tp in dtp.topics]
        sl = [tp.s for tp in dtp.topics]
        costfn = lambda u: cost.a(u.x, u.n, u.q, rates[u.i][0], rates[u.i][1], hP[u.l], sl[u.l])
        forced, cand = build_units(dtp, t, D, conn, needed, rank, costfn)
        slot = {}
        kh2 = None
        shared_end = None
        if mode in ADMISSION_MODES:
            if mode == "slotted":
                kh2 = self.pred.kappa_hat(2)
                guard = {i: 2 * dtp.eps + self.phy.t_succ(dtp.mss + dtp.h_T, rates[i][0]) for i in dtp.vessels}
                budget = SlotBudget(dtp.T_W, Zk, Z * kh2, sum_cB, guard)
            else:
                budget = AdditiveBudget(K)
            A = algorithm1(forced, cand, budget)
            if mode == "slotted":
                slot = slots(dtp, t, A, budget, Mt)
                w0, w1 = dtp.window(t)
                sh = budget.shared()
                shared_end = us(min(w1, w0 + sh)) if math.isfinite(sh) else us(w1)
        elif mode == "ipact":
            guard = {i: 2 * dtp.eps + self.phy.t_succ(dtp.mss + dtp.h_T, rates[i][0]) for i in dtp.vessels}
            A, slot, shared_end = ipact(dtp, t, forced, cand, sum_cB, guard, eta, self.eta(2), Mt)
        else:
            A = forced + cand
        tabs: Dict[int, Dict] = {}
        if mode == "b3":
            # strict priority in the order sigma_t: the table carries only the order (every unit is
            # admitted and the regulator does not police); a vessel without it uses the DTP order
            tabs = tables(dtp, t, A, Mt, needed, rank)
            o = sorted(range(dtp.L), key=lambda l: rank[l])
            for tb in tabs.values():
                tb["o"] = list(o)
        elif mode not in ("b1", "b2"):
            tabs = tables(dtp, t, A, Mt, needed, rank)
            if mode == "tdma":
                slot = {i: s for i, s in tdma_slots(dtp, t).items() if i in Mt}
            for i, s in slot.items():
                if i in tabs:
                    tabs[i]["slot"] = list(s)
        self.held.admitted(A)
        cost_A = sum(u.c for u in A)
        cost_R = sum(u.c for u in forced)
        self.prev = StepInfo(t=t, Mt=Mt, A=A, kappa_hat=kh, Z=Z, alpha=alpha, K=K, sent=set(tabs) if publish else set(),
                             cost_A=cost_A, slots=dict(slot) if mode == "slotted" else None, shared_end=shared_end,
                             kappa_hat2=kh2)

        rec = {
            "type": "as", "t": t, "mode": mode, "M": Mt,
            "b": mb["b"], "b_cover": mb["cover"], "b_hat": self.pred.b_hat, "eta": eta, "kappa_hat": kh,
            "alpha": alpha, "Z": Z, "K": K, "T_W": dtp.T_W, "predictor": self.predictor, "kappa_hat_eq": kh_eq,
            "cB": [cB[i] for i in dtp.vessels], "Bbar": {str(i): bbar[i] for i in dtp.vessels},
            "n_pubrel": {str(i): v for i, v in npr.items() if v},
            "R": {str(i): list(rates[i]) for i in dtp.vessels},
            "A": [[u.i, u.l, u.k] for u in A], "cost_A": cost_A, "cost_R": cost_R, "n_cand": len(cand),
            "p": {str(i): tb["p"] for i, tb in tabs.items()}, "k": {str(i): tb["k"] for i, tb in tabs.items()},
            "slot": {str(i): list(s) for i, s in slot.items()},
            "needed": {str(i): sorted(needed.get(i, set(dtp.needed)) | dtp.R) for i in dtp.vessels},
            "rank": list(rank), "connected": sorted(conn),
            "f_prev": close["f"] if close else None,
            "f_reasons": close["reasons"] if close else None,
            "kappa_prev": close["kappa"] if close else None,
            "e_prev": close["e"] if close else None,
            "w_prev": close["w"] if close else None,
            "z_prev": close["z"] if close else None,
            "z_slots_prev": close.get("z_slots") if close else None,
            "shared_end": shared_end, "kappa_hat2": kh2,
            "n_prev": close["n"] if close else None,
            "hold_prev": close["hold"] if close else None,
            "cover_prev": close["cover"] if close else None,
            "acc_prev": close["acc"] if close else None,
            "d_prev": close["d"] if close else None,
            "late_prev": close["late"] if close else None,
            "rep_prev": close["rep"] if close else None,
            "FR": self.aci.FR, "N_scores": len(self.scores), "late_demands": self.late_demands,
        }
        rec["t_compute_ms"] = (time.perf_counter() - c0) * 1e3
        return tabs, rec


def warmup(dtp, mode: str = "fleetdt", steps: int = 3) -> None:
    """Runs phase 2 of a throwaway ASCore on synthetic inputs, so that the first real phase 2 does not
    pay the first-call costs of the libraries (numpy paths, imports, caches)."""
    import copy
    d = copy.copy(dtp)
    core = ASCore(d, mode=mode if mode in MODES else "fleetdt")
    for t in range(1, steps + 1):
        for i in d.vessels:
            dd = [[[tp.n * tp.s, tp.n]] * (2 if tp.output else 1) for tp in d.topics]
            obj = {"t": t, "v": i, "d": dd, "held": [0] * d.L, "B": {"x0": 10, "n0": 1, "x2": 0, "n2": 0},
                   "rep": {"t": t - 1, "rel": [[1]] + [[] for _ in range(d.L - 1)], "disc": 0, "tab": True}}
            core.on_demand(i, obj, d.start(t))
        w, we = d.window(t)
        core.monitor.add({"from_us": us(d.start(t)), "to_us": us(d.end(t)) + us(d.T1),
                          "busy": [[us(w), us(w) + 300]],
                          "fr": [[us(w), us(w) + 300, d.vessels[0], 0, 1000, 24000, 1, 0],
                                 [us(we) + 10, us(we) + 300, d.vessels[0], 0, 1000, 24000, 1, 0]]})
        core.acceptor.on_message(d.topic_string(d.vessels[0], 0), b"\0" * 12, d.start(t))
        core.phase2(t)
