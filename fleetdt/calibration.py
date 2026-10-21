"""Calibration of the constants of Section VI-E of the Fleet-DT paper from the logs of the calibration runs.

For each number of vessels M:
  T1 = max_t w1(t) + eps,  T2 = max_t w2(t) + eps,
  eps = 2 max_{i,t} e_i(t) + lag (1 ms, the largest lag of a valid run) + the largest delay of the vessel
        regulator at a phase boundary,
  T_m = eps + delta_GS (largest delivery time of the GS MQTT broker),
  B_ctl_bar = largest number of bytes outside the data bridges in phase 3 of a calibration step;
over every calibration run: c_y = min{1, min_t C(t)/y(t)}; per (M, scenario): C_ref = mean of C(t); per (M, s)
with s a scenario of the calibration (or "all", no scenario left out): alpha* in {0.01, ..., 1} minimizes the
one-step MSE of eq. (2) on the calibration DTA without s, and U* is the largest U in {0.30, 0.31, ..., 1}
whose lower 95 % block-bootstrap bound of PICP(U) on that DTA reaches PICP_nom = 0.99 (fleetdt.outer).

Measurements and their sources (Table tab:metrics of the paper):
  w1(t)       start of phase 1 until the last demand table d_i(t) reaches the AS, on the GS clock: the AS log
              (as.jsonl), "d_at" of each decision (arrival in seconds after start(t)) and the records
              {"event": "demand_late", "t", "i", "w"} of the tables that arrived after start(t) + T1
  w2(t)       start of phase 2 (start(t) + T1 on the GS clock) until p(t) reaches the last vessel of M(t): the
              step log of each vessel regulator (regsteps.jsonl, "p_rx_us" or, after v(t), "p_late_us",
              microseconds after start(t) on the vessel clock), mapped to the GS clock with the probe offsets,
              arrival_GS = arrival_i - (theta_i(t) - theta_0(t)) (theta_i = clock of host i minus the clock of the
              emulation host, emu/clockprobe/probe.py, RUN/clock.csv; host 0 is the GS)
  e_i(t)      |System time| + root dispersion + root delay / 2 of `chronyc tracking` once per step on every
              host (emu/tools/chrony_sampler.py, chrony.jsonl)
  regulator   max(late_us, late_p1_us) of the step log: delay of the event of the regulator at v(t) (start of
              phase 3) and at the start of phase 1 (internal demand), microseconds
  delta_GS    largest time between the reception of an uplink message of a data bridge by the GS broker and its
              delivery to a subscriber, per step (plugin fdt_gsstamp.so of the GS broker, gsdelivery.jsonl)
  B_ctl(t)    ground truth of the emulator (gt.bin): every transmission (TX) of an IPv4 data frame by the AP or
              a vessel whose TCP/UDP ports are not those of the data bridges (1884, 1885, 1886) and that starts
              in phase 3 of step t on the GS clock (host clock + theta_0), counted as ip_len + h_W bytes (the
              unit of C(t) in emu/replay_subsets.py); retransmissions count again
  C(t), y(t)  RUN/csub/c_t.jsonl of the subset replays and the y(t) of the channel monitor

Every measurement skips the warm-up (the first `warm` steps after S0; 240 steps = 30 s, the warm-up of the
analysis). The functions take parsed logs; emu/experiments/calibrate_paper.py reads the run directories and
writes calibration.json (format in emu/README.md).
"""
from __future__ import annotations

import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import outer as O
from .as_model import DEFAULT_EPOCH

WARM_STEPS = 240
LAG_S = 1e-3                      # largest lag of the emulator in a valid run (the abort threshold)
DATA_PORTS = (1884, 1885, 1886)   # data bridges (B2 adds 1885 and 1886)
PICP_NOM = 0.99
CONF = 0.95
ALL = "all"                       # key of alpha* and U* without a scenario left out
FORMAT_VERSION = 1


def _f(v) -> Optional[float]:
    return None if v is None else float(v)


def _max(vals: Iterable[Optional[float]]) -> Optional[float]:
    v = [float(x) for x in vals if x is not None and x == x]
    return max(v) if v else None


def in_window(t: int, S0: Optional[int], steps: Optional[int], warm: int) -> bool:
    """Step t lies after the warm-up of the run: S0 + warm <= t < S0 + steps (no bound when S0 is None)."""
    if S0 is None:
        return True
    if t < S0 + warm:
        return False
    return steps is None or t < S0 + steps


# --------------------------------------------------------------------------- clock offsets of the probe


class Offsets:
    """theta_h(t) of the probe (RUN/clock.csv: host_ns, host, offset_ns, rtt_ns), linear interpolation in the host
    time. Host names "gs" (0) and "v<i>" (i). theta(h, at) in seconds at host time `at` (Unix seconds); 0 for a
    host without samples."""

    def __init__(self, samples: Dict[int, Tuple[np.ndarray, np.ndarray]]):
        self.s = {int(h): (np.asarray(x, dtype=float), np.asarray(y, dtype=float)) for h, (x, y) in samples.items()}

    @staticmethod
    def from_csv_lines(lines: Iterable[str]) -> "Offsets":
        acc: Dict[int, Tuple[List[float], List[float]]] = {}
        for line in lines:
            p = line.strip().split(",")
            if len(p) < 3 or p[0] == "host_ns":
                continue
            try:
                h = 0 if p[1] == "gs" else int(p[1].lstrip("v"))
                x, y = int(p[0]) / 1e9, int(p[2]) / 1e9
            except ValueError:
                continue
            a = acc.setdefault(h, ([], []))
            a[0].append(x)
            a[1].append(y)
        out = {}
        for h, (x, y) in acc.items():
            o = np.argsort(x, kind="stable")
            out[h] = (np.asarray(x)[o], np.asarray(y)[o])
        return Offsets(out)

    def has(self, h: int) -> bool:
        return int(h) in self.s and len(self.s[int(h)][0]) > 0

    def theta(self, h: int, at) -> np.ndarray:
        if not self.has(h):
            return np.zeros_like(np.asarray(at, dtype=float))
        x, y = self.s[int(h)]
        return np.interp(np.asarray(at, dtype=float), x, y)


# --------------------------------------------------------------------------- w1(t), w2(t)


def w1_series(as_records: Iterable[Dict]) -> Dict[int, Dict]:
    """w1(t) per step from the AS log: the largest arrival (seconds after start(t), GS clock) over the demand tables
    of M(t) ("d_at" of the decision) and the tables that arrived after start(t) + T1 ("demand_late" records).
    Returns {t: {"w1", "n" (tables of M(t)), "late" (late tables)}}."""
    out: Dict[int, Dict] = {}
    for r in as_records:
        if r.get("event") == "demand_late":
            t = int(r["t"])
            e = out.setdefault(t, {"w1": None, "n": 0, "late": 0})
            e["late"] += 1
            e["w1"] = _max([e["w1"], r.get("w")])
        elif r.get("event") is None and r.get("t") is not None and isinstance(r.get("d_at"), dict):
            t = int(r["t"])
            e = out.setdefault(t, {"w1": None, "n": 0, "late": 0})
            e["n"] += len(r["d_at"])
            e["w1"] = _max([e["w1"]] + list(r["d_at"].values()))
    return out


def members(as_records: Iterable[Dict]) -> Dict[int, List[int]]:
    """M(t) of each published decision of the AS log."""
    out = {}
    for r in as_records:
        if r.get("event") is None and r.get("t") is not None and r.get("published") is not False and r.get("M"):
            out[int(r["t"])] = sorted(int(i) for i in r["M"])
    return out


def w2_series(regsteps: Dict[int, Iterable[Dict]], M_by_t: Dict[int, Sequence[int]], T1: float, epoch: float,
              Ts: float, offsets: Optional[Offsets] = None) -> Dict[int, Dict]:
    """w2(t) per step: for each vessel i of M(t), the arrival of p(t) at vessel i (p_rx_us, else p_late_us of its
    step log, microseconds after start(t) on the clock of vessel i) on the GS clock minus the start of phase 2,
        w2_i(t) = arrival_i - T1 - (theta_i(t) - theta_0(t)),
    and w2(t) = max_i w2_i(t). A vessel of M(t) without an arrival in its log counts in "missing" (its table
    never reached it within the step). Returns {t: {"w2", "n", "missing"}}."""
    by: Dict[int, Dict[int, Dict]] = {}
    for i, recs in regsteps.items():
        by[int(i)] = {int(r["t"]): r for r in recs if r.get("t") is not None}
    out: Dict[int, Dict] = {}
    for t, M in sorted(M_by_t.items()):
        at = epoch + t * Ts
        th0 = float(offsets.theta(0, at)) if offsets is not None else 0.0
        vals, missing = [], 0
        for i in M:
            r = by.get(int(i), {}).get(int(t))
            us = None if r is None else (r.get("p_rx_us") if r.get("p_rx_us") is not None else r.get("p_late_us"))
            if us is None:
                missing += 1
                continue
            thi = float(offsets.theta(i, at)) if offsets is not None else 0.0
            vals.append(float(us) / 1e6 - T1 - (thi - th0))
        out[int(t)] = {"w2": max(vals) if vals else None, "n": len(vals), "missing": missing}
    return out


# --------------------------------------------------------------------------- e_i(t), regulator, delta_GS


def chrony_errors(records: Iterable[Dict]) -> Dict[int, float]:
    """e_i(t) of one host from its sampler log (chrony_sampler.py): {t: e} (records with an error skipped)."""
    return {int(r["t"]): float(r["e"]) for r in records if r.get("e") is not None and r.get("t") is not None}


def regulator_delays(records: Iterable[Dict]) -> Dict[int, float]:
    """Largest delay (seconds) of the vessel regulator at a phase boundary per step: max(late_us, late_p1_us) of
    its step log (negative values: event not run)."""
    out = {}
    for r in records:
        if r.get("t") is None:
            continue
        v = [float(r[k]) for k in ("late_us", "late_p1_us") if r.get(k) is not None and float(r[k]) >= 0]
        if v:
            out[int(r["t"])] = max(v) / 1e6
    return out


def delivery_times(records: Iterable[Dict]) -> Dict[int, float]:
    """Largest delivery time (seconds) of the GS broker per step from the plugin log (gsdelivery.jsonl); a step that
    appears twice (two flushes) keeps its largest value."""
    out: Dict[int, float] = {}
    for r in records:
        if r.get("t") is None or r.get("max_us") is None:
            continue
        t = int(r["t"])
        out[t] = max(out.get(t, 0.0), float(r["max_us"]) / 1e6)
    return out


# --------------------------------------------------------------------------- B_ctl(t) from the ground truth


def ctl_bytes_phase3(gt: np.ndarray, origin_ns: int, epoch: float, Ts: float, T1: float, T2: float, M: int,
                     h_W: float, offsets: Optional[Offsets] = None, data_ports: Sequence[int] = DATA_PORTS,
                     steps: Optional[Iterable[int]] = None) -> Dict[int, float]:
    """B_ctl(t): bytes outside the data bridges in phase 3 of step t. gt holds the records of gt.bin (numpy dtype
    of emu/gt.py: fields t_ns, node, ip_len, sport, dport, ev, kind); a record counts when it is a transmission
    (ev 0) of an IPv4 data frame (kind 0, ip_len > 0) by the AP (node 0) or a vessel (nodes 1..M), with neither
    port in data_ports, whose start falls in phase 3 of step t on the GS clock: host time origin_ns + t_ns plus
    theta_0 (the probe offset of the GS). Each frame adds ip_len + h_W. Returns {t: bytes} for the steps of
    `steps` (0 for a step without such frames) or for the steps with frames."""
    r = gt
    sel = (r["ev"] == 0) & (r["kind"] == 0) & (r["ip_len"] > 0) & (r["node"] <= M)
    for p in data_ports:
        sel &= (r["sport"] != p) & (r["dport"] != p)
    r = r[sel]
    host = (int(origin_ns) + r["t_ns"].astype(np.int64)) / 1e9
    gs = host + (offsets.theta(0, host) if offsets is not None else 0.0)
    rel = gs - epoch
    t = np.floor(rel / Ts).astype(np.int64)
    p3 = (rel - t * Ts) >= (T1 + T2) - 1e-12
    t, b = t[p3], r["ip_len"][p3].astype(float) + float(h_W)
    out: Dict[int, float] = {}
    if len(t):
        u, inv = np.unique(t, return_inverse=True)
        sums = np.bincount(inv, weights=b)
        out = {int(k): float(v) for k, v in zip(u, sums)}
    if steps is not None:
        return {int(k): out.get(int(k), 0.0) for k in steps}
    return out


# --------------------------------------------------------------------------- one run


def measure_run(run: Dict, warm: int = WARM_STEPS, data_ports: Sequence[int] = DATA_PORTS) -> Dict:
    """Measurements of one calibration run. run holds the parsed logs:
      "name", "M", "scenario", "S0", "steps", "dtp" (dict of the DTP of the run: epoch, Ts, T1, T2, h_W),
      "as" (AS log records), "regsteps" ({i: step log records}), "chrony" ({host: sampler records}),
      "offsets" (Offsets or None), "gsdelivery" (plugin records), "gt" ((origin_ns, records) or None),
      "C" ({t: C(t)}), "y" ({t: y(t)}), "dta" (G(t) records).
    Missing logs give None for their quantities (listed in "missing")."""
    d = run["dtp"]
    epoch = float(d["epoch"]) if d.get("epoch") is not None else DEFAULT_EPOCH
    Ts, T1, T2 = float(d["Ts"]), float(d["T1"]), float(d["T2"])
    S0, steps, M = run.get("S0"), run.get("steps"), int(run["M"])
    win = lambda t: in_window(int(t), S0, steps, warm)   # noqa: E731
    out = {"name": run.get("name"), "M": M, "scenario": run.get("scenario"), "S0": S0, "steps": steps,
           "T1_run": T1, "T2_run": T2, "missing": []}
    as_recs = list(run.get("as") or [])
    w1 = {t: v for t, v in w1_series(as_recs).items() if win(t)}
    out["w1_max"] = _max(v["w1"] for v in w1.values())
    out["w1_steps"] = len(w1)
    out["w1_incomplete"] = sum(1 for v in w1.values() if v["n"] + v["late"] < M)
    offs = run.get("offsets")
    if offs is None or not offs.has(0):
        out["missing"].append("probe offsets (clock.csv): w2 and B_ctl without the clock mapping")
    regs = run.get("regsteps") or {}
    Mt = {t: m for t, m in members(as_recs).items() if win(t)}
    w2 = w2_series(regs, Mt, T1, epoch, Ts, offs)
    out["w2_max"] = _max(v["w2"] for v in w2.values())
    out["w2_steps"] = sum(1 for v in w2.values() if v["n"])
    out["w2_missing"] = sum(v["missing"] for v in w2.values())
    errs = {h: chrony_errors(recs) for h, recs in (run.get("chrony") or {}).items()}
    out["e_max"] = _max(e for h, es in errs.items() for t, e in es.items() if win(t))
    out["e_hosts"] = sorted(int(h) for h, es in errs.items() if es)
    reg = [v for recs in regs.values() for t, v in regulator_delays(recs).items() if win(t)]
    out["reg_delay_max"] = _max(reg)
    dl = delivery_times(run.get("gsdelivery") or [])
    out["delta_GS"] = _max(v for t, v in dl.items() if win(t))
    gt = run.get("gt")
    if gt is not None and S0 is not None and steps is not None:
        origin_ns, recs = gt
        b = ctl_bytes_phase3(recs, origin_ns, epoch, Ts, T1, T2, M, float(d.get("h_W") or 0.0), offs, data_ports,
                             steps=range(int(S0) + warm, int(S0) + int(steps)))
        out["B_ctl_max"] = _max(b.values())
        out["B_ctl_steps"] = len(b)
    else:
        out["B_ctl_max"] = None
    C = {int(t): float(v) for t, v in (run.get("C") or {}).items() if win(t)}
    y = {int(t): float(v) for t, v in (run.get("y") or {}).items()}
    ratios = [C[t] / y[t] for t in C if t in y and y[t] > 0 and C[t] > 0]
    out["cy_min"] = min(ratios) if ratios else None
    out["cy_n"] = len(ratios)
    out["C_sum"], out["C_n"] = float(sum(C.values())), len(C)
    for k, label in (("w1_max", "AS arrival times (as.jsonl d_at)"), ("w2_max", "table arrivals (regsteps p_rx_us)"),
                     ("e_max", "chrony error bound (chrony.jsonl)"), ("reg_delay_max", "regulator delay (regsteps)"),
                     ("delta_GS", "GS delivery time (gsdelivery.jsonl)"), ("B_ctl_max", "ground truth (gt.bin)")):
        if out[k] is None:
            out["missing"].append(label)
    if not C:
        out["missing"].append("C(t) of the subset replays (csub/c_t.jsonl)")
    return out



# --------------------------------------------------------------------------- the constants


def margins(ms: Sequence[Dict], lag_s: float = LAG_S, quantum_s: float = 1e-6) -> Dict:
    """T1, T2, eps, delta_GS, T_m and B_ctl_bar of one M from the measurements of its runs (measure_run). A
    quantity without any measurement stays None, and so do the constants that need it. T1, T2 and T_m are
    rounded up to `quantum_s` (the microsecond clock of the regulator)."""
    up = lambda x: None if x is None else round(math.ceil(x / quantum_s - 1e-9) * quantum_s, 12)   # noqa: E731
    w1, w2 = _max(m["w1_max"] for m in ms), _max(m["w2_max"] for m in ms)
    e, reg = _max(m["e_max"] for m in ms), _max(m["reg_delay_max"] for m in ms)
    dgs, bctl = _max(m["delta_GS"] for m in ms), _max(m["B_ctl_max"] for m in ms)
    eps = 2.0 * e + lag_s + reg if e is not None and reg is not None else None
    return {"T1": up(w1 + eps) if w1 is not None and eps is not None else None,
            "T2": up(w2 + eps) if w2 is not None and eps is not None else None,
            "eps": eps, "delta_GS": dgs, "T_m": up(eps + dgs) if eps is not None and dgs is not None else None,
            "B_ctl_bar": bctl, "w1_max": w1, "w2_max": w2, "e_max": e, "reg_delay_max": reg, "lag_s": lag_s,
            "runs": len(ms), "w2_missing": int(sum(m["w2_missing"] for m in ms)),
            "w1_incomplete": int(sum(m["w1_incomplete"] for m in ms))}


def c_y_global(ms: Sequence[Dict]) -> Dict:
    """c_y = min{1, min_t C(t)/y(t)} over the steps of every run with C(t) (every load level, scenario and M)."""
    v = [m["cy_min"] for m in ms if m.get("cy_min") is not None]
    return {"c_y": min(1.0, min(v)) if v else None, "n": int(sum(m["cy_n"] for m in ms)), "runs": len(v)}


def c_ref(ms: Sequence[Dict]) -> Dict[int, Dict[str, float]]:
    """C_ref per (M, scenario): mean of C(t) over the steps of the runs of the cell."""
    acc: Dict[Tuple[int, str], List[float]] = {}
    for m in ms:
        if m["C_n"]:
            a = acc.setdefault((int(m["M"]), str(m["scenario"])), [0.0, 0])
            a[0] += m["C_sum"]
            a[1] += m["C_n"]
    out: Dict[int, Dict[str, float]] = {}
    for (M, s), (tot, n) in sorted(acc.items()):
        out.setdefault(M, {})[s] = tot / n
    return out


def dta_window(records: Sequence[Dict], S0: Optional[int], steps: Optional[int], warm: int) -> List[Dict]:
    return [r for r in records if r.get("t") is not None and in_window(int(r["t"]), S0, steps, warm)]


def outer_fit(dtas: Sequence[Sequence[Dict]], c_y: float, B_ctl_bar: float, T3_Ts: float,
              target: float = PICP_NOM, conf: float = CONF, B: int = 1000, seed: int = 0,
              U_grid: np.ndarray = O.U_GRID) -> Dict:
    """alpha* (grid 0.01..1, no refinement) and U* on the pooled DTA of the runs (fleetdt.outer._calibrate: the
    one-step MSE of eq. (2) on c_y y pooled over the runs, PICP(U) with K_y(t) = c_y y(t) T3_Ts - B_ctl_bar and the
    C_hat(t) replayed with alpha*, its lower `conf` circular block bootstrap bound with blocks of ceil(2 tau_int)
    steps inside each run)."""
    ser, C0s = [], []
    for recs in dtas:
        if not recs:
            continue
        _, y, Crec = O.series(recs)
        if not np.any(~np.isnan(y)):
            continue
        ser.append((y, Crec, O.fill_series(recs, T3_Ts), O.backlog_series(recs)))
        C0s.append(O._C0(y, Crec))
    if not ser:
        return {"alpha": None, "U": None, "runs": 0, "n_y": 0}
    r = O._calibrate(ser, C0s, target, None, U_grid, conf, B, None, seed, T3_Ts, c_y, B_ctl_bar, refine=False)
    return {"alpha": r["alpha_star"], "U": r["U_star"], "U_point": r["U_star_point"], "mse": r["mse"],
            "block": r["block"], "runs": len(ser), "n_y": int(sum(np.sum(~np.isnan(s[0])) for s in ser)),
            "PICP_low": {("%.2f" % w["U"]): w["PICP_low"] for w in r["whatif"]}}


def calibrate(runs: Sequence[Dict], warm: int = WARM_STEPS, lag_s: float = LAG_S, target: float = PICP_NOM,
              conf: float = CONF, B: int = 1000, seed: int = 0, data_ports: Sequence[int] = DATA_PORTS,
              quantum_s: float = 1e-6, Ts: Optional[float] = None) -> Dict:
    """calibration.json of the calibration runs (each run a dict of measure_run's input). alpha* and U* use the
    calibrated c_y, B_ctl_bar and (T3 - T_m)/Ts of M (T3 = Ts - T1 - T2 with the calibrated T1 and T2); a
    constant that cannot be computed falls back to the DTP of the runs (c_y 1, B_ctl_bar 0, T1, T2, T_m 0) for
    alpha* and U* and stays null in the output."""
    ms = [measure_run(r, warm, data_ports) for r in runs]
    cy = c_y_global(ms)
    out = {"version": FORMAT_VERSION,
           "params": {"warm_steps": warm, "lag_s": lag_s, "PICP_nom": target, "conf": conf, "B": B, "seed": seed,
                      "alpha_grid": [0.01, 1.0, 0.01], "U_grid": [float(O.U_GRID[0]), float(O.U_GRID[-1]), 0.01],
                      "data_ports": list(data_ports), "quantum_s": quantum_s},
           "c_y": cy["c_y"], "c_y_n": cy["n"], "M": {}, "C_ref": {}, "outer": {}, "runs": ms}
    byM: Dict[int, List[int]] = {}
    for k, m in enumerate(ms):
        byM.setdefault(int(m["M"]), []).append(k)
    cr = c_ref(ms)
    for M, idx in sorted(byM.items()):
        mg = margins([ms[k] for k in idx], lag_s, quantum_s)
        out["M"][str(M)] = mg
        out["C_ref"][str(M)] = cr.get(M, {})
        d = runs[idx[0]]["dtp"]
        Ts_ = float(Ts if Ts is not None else d["Ts"])
        T1 = mg["T1"] if mg["T1"] is not None else float(d["T1"])
        T2 = mg["T2"] if mg["T2"] is not None else float(d["T2"])
        Tm = mg["T_m"] if mg["T_m"] is not None else float(d.get("T_m") or 0.0)
        T3_Ts = (Ts_ - T1 - T2 - Tm) / Ts_
        c_y = cy["c_y"] if cy["c_y"] is not None else float(d.get("c_y") or 1.0)
        bctl = mg["B_ctl_bar"] if mg["B_ctl_bar"] is not None else float(d.get("B_ctl_bar") or 0.0)
        dtas = [(str(ms[k]["scenario"]), dta_window(runs[k].get("dta") or [], runs[k].get("S0"),
                                                     runs[k].get("steps"), warm)) for k in idx]
        scen = sorted({s for s, _ in dtas})
        res = {"T3_Ts": T3_Ts, "c_y": c_y, "B_ctl_bar": bctl, "left_out": {}}
        for s in scen + [ALL]:
            fit = outer_fit([recs for sc, recs in dtas if sc != s], c_y, bctl, T3_Ts, target, conf, B, seed)
            fit["scenarios"] = sorted({sc for sc, recs in dtas if sc != s and recs})
            res["left_out"][s] = fit
        out["outer"][str(M)] = res
    return out


def lookup_outer(cal: Dict, M: int, scenario: Optional[str]) -> Tuple[Optional[float], Optional[float]]:
    """(alpha*, U*) for an evaluation run with M vessels in `scenario`: the fit without that scenario when the
    calibration holds it, else the fit on every calibration scenario ("all", also for scenario None). KeyError
    without the M."""
    o = cal["outer"][str(int(M))]["left_out"]
    e = o[str(scenario)] if scenario is not None and str(scenario) in o else o[ALL]
    return e.get("alpha"), e.get("U")


def apply_to_dtp(dtp: Dict, cal: Dict, M: int, scenario: Optional[str]) -> Tuple[Dict, List[str]]:
    """DTP of an evaluation run: T1, T2, T_m, B_ctl_bar of M, the global c_y and the (alpha*, U*) of lookup_outer.
    A null constant of the calibration leaves the field of the DTP unchanged and is listed in the second value.
    The DTP gets "calibration": {"M", "scenario", "left_out"} as a record of the source."""
    out = dict(dtp)
    kept = []
    mg = cal["M"][str(int(M))]
    for k in ("T1", "T2", "T_m", "B_ctl_bar"):
        if mg.get(k) is None:
            kept.append(k)
        else:
            out[k] = mg[k]
    if cal.get("c_y") is None:
        kept.append("c_y")
    else:
        out["c_y"] = cal["c_y"]
    a, U = lookup_outer(cal, M, scenario)
    o = cal["outer"][str(int(M))]["left_out"]
    left = str(scenario) if scenario is not None and str(scenario) in o and str(scenario) != ALL else None
    for k, v in (("alpha", a), ("U", U)):
        if v is None:
            kept.append(k)
        else:
            out[k] = v
    out["calibration"] = {"M": int(M), "scenario": scenario, "left_out": left}
    T3 = float(out["Ts"]) - float(out["T1"]) - float(out["T2"])
    if not 0 <= float(out.get("T_m") or 0.0) < T3:
        raise ValueError("calibrated T1 + T2 + T_m = %.6f s leaves no phase 3 in Ts = %s s"
                         % (float(out["T1"]) + float(out["T2"]) + float(out.get("T_m") or 0.0), out["Ts"]))
    return out, kept
