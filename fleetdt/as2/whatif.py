"""What-if evaluation (Def. whatif): predicts f-bar and nu of a configuration (M, PHY rates, topics, Ts,
beta) by replaying the scores and the external busy fractions of a DTA through eq. (pred), eq. (aci)
and Algorithm 1 with the efficiency eta of the configuration.

Replay model (assumption of Def. whatif: the scores and the busy fractions of the DTA hold for the
configuration): replay step j takes b_j and z_j of the j-th step of the DTA with a score; the capacity of
the configuration is kappa_j = z_j kappa_hat'_j; the demand of each vessel is the arrival-curve bound
over Ts; f_j = 1 when the admitted cost plus the held cost exceeds kappa_j T_W (Lemma 2 read as an
equivalence), and the excess airtime is held into step j+1.

  python -m fleetdt.as2.whatif --dtp dtp_v2.json --dta dta.jsonl [--M 4 8 16] [--rate 24e6] [--Ts 0.125]
         [--beta 0.05]
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .aci import ACI
from .admission import AdditiveBudget, Unit, algorithm1, arrival_units, unit_key
from .airtime import Cost
from .analysis import load, steps
from .bianchi import EtaTable
from .dtp import DTP
from .predictor import Predictor, Scores


def series(recs: List[Dict]) -> List[Tuple[Optional[float], float]]:
    """(b(t), z(t)) of the steps of the DTA with a score, in step order."""
    a = steps(recs)
    out = []
    for t in sorted(a):
        nx = a.get(t + 1)
        if nx is None or nx.get("z_prev") is None:
            continue
        out.append((a[t].get("b"), float(nx["z_prev"])))
    return out


def configure(dtp: DTP, M: Optional[int] = None, Ts: Optional[float] = None, beta: Optional[float] = None,
              rate: Optional[float] = None, topics: Optional[List[Dict]] = None) -> DTP:
    d = copy.deepcopy(dtp.raw)
    if M is not None:
        d["vessels"] = list(range(1, M + 1))
    if Ts is not None:
        d["Ts"] = Ts
    if beta is not None:
        d["beta"] = beta
    if rate is not None:
        d["phy"] = dict(d["phy"], rate_init=rate)
    if topics is not None:
        d["topics"] = topics
        names = [t["name"] for t in topics]
        d["order"] = [n for n in d.get("order", names) if n in names] + [n for n in names if n not in d.get("order", [])]
        d["needed"] = [n for n in d.get("needed", []) if n in names]
        d["required"] = [n for n in d.get("required", []) if n in names]
    return DTP.from_dict(d)


def replay(cfg: DTP, seq: Sequence[Tuple[Optional[float], float]], rates: Optional[Dict[int, Tuple[float, float]]] = None,
           initial_scores: Iterable[float] = (), z_empty: float = 1.0) -> Dict:
    cost = Cost.from_dtp(cfg)
    eta = EtaTable.from_dtp(cfg, R=(next(iter(rates.values()))[0] if rates else None))
    pred = Predictor(cfg.lam, eta, cfg.M)
    sc = Scores(initial_scores, empty=z_empty)
    aci = ACI(cfg.beta, cfg.gamma, cfg.alpha1)
    R0 = cfg.phy["rate_init"]
    rates = rates or {i: (R0, R0) for i in cfg.vessels}
    rank = [0] * cfg.L
    for r, l in enumerate(cfg.order):
        rank[l] = r
    N = set(cfg.needed) | cfg.R
    held = 0.0
    fs, num, den = [], 0.0, 0.0
    admitted_out = 0
    for t, (b, z) in enumerate(seq, start=1):
        kh = pred.step(b)
        Z = sc.Z(aci.alpha)
        K = Z * kh * cfg.T_W - held
        units: List[Unit] = []
        for i in cfg.vessels:
            for u in arrival_units(cfg, i, range(cfg.L), cfg.Ts):
                u.inR = u.l in cfg.R
                Ri, R0i = rates[i]
                u.c = cost.a(u.x, u.n, u.q, Ri, R0i, cfg.h_P(u.l))
                u.key = unit_key(u, t, cfg, cfg.R, N, rank)
                units.append(u)
        forced = sorted([u for u in units if u.inR], key=lambda u: u.key)
        cand = sorted([u for u in units if not u.inR], key=lambda u: u.key)
        A = algorithm1(forced, cand, AdditiveBudget(K))
        cA = sum(u.c for u in A)
        admitted_out += sum(1 for u in A if cfg.topics[u.l].output)
        kappa = z * kh
        work = cA + held
        f = 1 if work > kappa * cfg.T_W else 0
        held = max(0.0, work - kappa * cfg.T_W) if f else 0.0
        fs.append(f)
        num += cA
        den += kappa * cfg.T_W
        aci.update(f)
        sc.add(z)
    T = len(fs)
    return {"M": cfg.M, "Ts": cfg.Ts, "beta": cfg.beta, "rate": rates[cfg.vessels[0]][0], "steps": T,
            "f_bar": sum(fs) / T if T else None, "nu": num / den if den > 0 else None, "FR": aci.FR,
            "bound": aci.bound() if T else None, "outputs_per_step": admitted_out / T if T else None,
            "eta": eta(cfg.M + 1)}


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.as2.whatif", description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--dta", required=True, nargs="+")
    ap.add_argument("--M", type=int, nargs="*", default=[None])
    ap.add_argument("--Ts", type=float, nargs="*", default=[None])
    ap.add_argument("--beta", type=float, nargs="*", default=[None])
    ap.add_argument("--rate", type=float, nargs="*", default=[None])
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    seq = []
    for p in a.dta:
        seq += series(load(p))
    out = []
    for M in a.M:
        for Ts in a.Ts:
            for beta in a.beta:
                for rate in a.rate:
                    cfg = configure(dtp, M=M, Ts=Ts, beta=beta, rate=rate)
                    rates = {i: (rate, rate) for i in cfg.vessels} if rate else None
                    out.append(replay(cfg, seq, rates))
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
