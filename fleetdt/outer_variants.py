"""Outer-loop variants O1 to O5 of the experiments (Table S4 of the supplementary material, Sections IV-F, VI-C
and VI-E of the Fleet-DT paper). Pure logic, no broker and no numpy: the AS (fleetdt.as_service.AS) holds one
controller and calls it once per step; the window recomputation of O3 runs fleetdt.outer.run_job in a child
process of the Service.

  O1  fixed (alpha_0, U_0), default the alpha and U of the DTP (1/8 and 0.8 in the DTP of the paper).
  O2  fixed (alpha*, U*) of the calibration file for (M, scenario).
  O3  at the end of each window of N_w steps, alpha and U from the DTA of that window by the rules of alpha* and
      U* (fleetdt.outer.outer_loop over the steps of the window: alpha on the grid 0.01..1 without refinement,
      U the largest U of 0.30..1 whose lower 95 % block-bootstrap bound of PICP(U) reaches PICP_nom). The job
      starts `lag` steps after the end of the window, so the VE has written G(t) of its last step, and its result
      applies from the step after it arrives. Before the first result: (alpha_0, U_0) of the DTP (init "dtp",
      Section IV-F) or (alpha*, U*) of the calibration (init "calibration"). A window without U* (no U of the
      grid reaches PICP_nom) sets U to the smallest U of the grid (no_U "min") or keeps U (no_U "keep"); a
      window without alpha* (no step with y) keeps alpha.
  O4  alpha = alpha*, U starts at U*; at the end of each window of N_w steps, U + step when the PICP of the steps
      of the window reaches PICP_nom, U - step otherwise, clamped to [U_min, U_max]; a window without a coverage
      event keeps U. The coverage event of step t is K_y(t) >= U(t) K_hat(t), K_y(t) = c_y y(t) (T3 - T_m)/Ts -
      B_ctl_bar, from the y(t) that reaches the AS and its own U(t) and K_hat(t) (no DTA).
  O5  oracle: at the step of each change of regime, (alpha*, U*) of the calibration of the new regime. The
      regimes are [(t0, scenario), ...] with t0 in seconds from the origin (scenario time 0 = start(S0)); the
      change step is origin + floor(t0 / Ts), the step that contains t0.

Windows: window k holds the steps origin + k N_w .. origin + (k + 1) N_w - 1; origin is the first step of the run
(start(S0)), else the first step that the AS closes.

Calibration file (JSON, written by the calibration pipeline; emu/README.md, "Outer-loop variants"):

  {"format": "fleetdt-calibration/1",
   "PICP_nom": 0.99,                                   (optional; informative)
   "by_M": {"<M>": {"<scenario>": {"alpha_star": a, "U_star": u, ...}, ...}, ...}}

alpha_star in (0, 1] (alpha* of the scenario: minimum of the one-step MSE of eq. (2) on the calibration DTA
without that scenario), U_star in [0.30, 1] (largest U of the grid whose lower 95 % block-bootstrap bound of
PICP(U) on that DTA reaches PICP_nom); other keys of an entry are kept and ignored."""
from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

VARIANTS = ("O1", "O2", "O3", "O4", "O5")
CALIBRATION_FORMAT = "fleetdt-calibration/1"
N_W = 480                 # 60 s at Ts = 125 ms
PICP_NOM = 0.99
U_GRID_MIN, U_GRID_MAX = 0.30, 1.00
O4_STEP, O4_U_MIN, O4_U_MAX = 0.05, 0.30, 1.2
O3_LAG = 2                # steps between the end of a window and the start of its job (the VE grace is 62.5 ms)


# --------------------------------------------------------------------------- calibration file


def validate_calibration(cal: Dict) -> Dict:
    """Checks the format of a calibration file (module docstring); returns it."""
    if not isinstance(cal, dict) or cal.get("format") != CALIBRATION_FORMAT:
        raise ValueError("calibration: \"format\" must be %r" % CALIBRATION_FORMAT)
    by = cal.get("by_M")
    if not isinstance(by, dict) or not by:
        raise ValueError("calibration: \"by_M\" must map M to the scenarios")
    for M, scen in by.items():
        if not str(M).isdigit() or not isinstance(scen, dict) or not scen:
            raise ValueError("calibration: by_M.%s must map scenarios to {alpha_star, U_star}" % M)
        for s, e in scen.items():
            if not isinstance(e, dict):
                raise ValueError("calibration: by_M.%s.%s is not an object" % (M, s))
            a, u = e.get("alpha_star"), e.get("U_star")
            if not isinstance(a, (int, float)) or isinstance(a, bool) or not 0.0 < float(a) <= 1.0:
                raise ValueError("calibration: by_M.%s.%s.alpha_star must lie in (0, 1]" % (M, s))
            if not isinstance(u, (int, float)) or isinstance(u, bool) or \
                    not U_GRID_MIN - 1e-9 <= float(u) <= U_GRID_MAX + 1e-9:
                raise ValueError("calibration: by_M.%s.%s.U_star must lie in [%.2f, %.2f]" % (M, s, U_GRID_MIN,
                                                                                            U_GRID_MAX))
    return cal


def load_calibration(path) -> Dict:
    """Reads and checks a calibration file."""
    with open(path, encoding="utf-8") as f:
        return validate_calibration(json.load(f))


def calibration_values(cal: Dict, M: int, scenario: str) -> Tuple[float, float]:
    """(alpha*, U*) of the calibration for M vessels and the evaluation scenario."""
    by = cal["by_M"].get(str(int(M)))
    if by is None:
        raise KeyError("calibration has no entry for M = %d (has %s)" % (int(M), sorted(cal["by_M"])))
    e = by.get(str(scenario))
    if e is None:
        raise KeyError("calibration has no entry for M = %d, scenario %s (has %s)" % (int(M), scenario, sorted(by)))
    return float(e["alpha_star"]), float(e["U_star"])


def regime_steps(regimes: Sequence, Ts: float, origin: int) -> List[Tuple[int, str]]:
    """[(change step, scenario)] sorted by step: origin + floor(t0 / Ts) for each {"t0", "scenario"} or
    (t0, scenario); t0 in seconds from the origin."""
    out = []
    for r in regimes:
        t0, s = (r["t0"], r["scenario"]) if isinstance(r, dict) else (r[0], r[1])
        if float(t0) < 0:
            raise ValueError("regime t0 must be >= 0")
        out.append((int(origin) + int(math.floor(float(t0) / float(Ts) + 1e-9)), str(s)))
    out.sort(key=lambda x: x[0])
    return out


# --------------------------------------------------------------------------- controllers


class Variant:
    """Base of the controllers. alpha and U hold the values in effect; step(t, cover) runs at the close of step
    t, before eq. (2) and eq. (3) of t, with the coverage event of step t - 1 (cover = (t - 1, hit) or None), and
    returns an update record (dict with the new "alpha" and "U") or None."""
    name = ""

    def __init__(self, alpha: float, U: float, N_w: int = N_W, origin: Optional[int] = None,
                 target: float = PICP_NOM):
        if int(N_w) < 1:
            raise ValueError("N_w must be >= 1")
        self.alpha, self.U = float(alpha), float(U)
        self.N_w, self.origin, self.target = int(N_w), None if origin is None else int(origin), float(target)

    def _start(self, t: int) -> None:
        if self.origin is None:
            self.origin = int(t)

    def window(self, t: int) -> Optional[int]:
        """Index of the window of step t (None before the origin)."""
        if self.origin is None or t < self.origin:
            return None
        return (int(t) - self.origin) // self.N_w

    def step(self, t: int, cover: Optional[Tuple[int, bool]]) -> Optional[Dict]:
        self._start(t)
        return None

    def describe(self) -> Dict:
        return {"variant": self.name, "N_w": self.N_w, "origin": self.origin, "PICP_nom": self.target}


class Fixed(Variant):
    """O1 and O2: fixed (alpha, U)."""

    def __init__(self, name: str, alpha: float, U: float, **kw):
        super().__init__(alpha, U, **kw)
        self.name = name


class WindowRecompute(Variant):
    """O3: recomputation of alpha and U from the DTA of each window (job in a child process)."""
    name = "O3"

    def __init__(self, alpha: float, U: float, lag: int = O3_LAG, refine: bool = False, no_U: str = "min",
                 B: int = 1000, U_floor: float = U_GRID_MIN, **kw):
        super().__init__(alpha, U, **kw)
        if no_U not in ("min", "keep"):
            raise ValueError("no_U must be min or keep")
        if int(lag) < 0:
            raise ValueError("lag must be >= 0")
        self.lag, self.refine, self.no_U, self.B, self.U_floor = int(lag), bool(refine), no_U, int(B), float(U_floor)

    def job_due(self, t: int) -> Optional[Tuple[int, int]]:
        """(t_first, t_last) of the window whose job starts at step t: the window that ended `lag` steps ago."""
        if self.origin is None:
            return None
        end = int(t) - self.lag                       # last step of a window?
        if end < self.origin + self.N_w - 1 or (end - self.origin + 1) % self.N_w:
            return None
        return end - self.N_w + 1, end

    def apply(self, res: Dict) -> Dict:
        """New (alpha, U) from a result of fleetdt.outer.outer_loop over the window."""
        a, u = res.get("alpha_star"), res.get("U_star")
        if a is not None and a == a:
            self.alpha = float(a)
        if u is not None:
            self.U = float(u)
        elif self.no_U == "min":
            self.U = self.U_floor
        return {"alpha": self.alpha, "U": self.U, "alpha_star": a, "U_star": u}

    def describe(self) -> Dict:
        return dict(super().describe(), lag=self.lag, refine=self.refine, no_U=self.no_U, B=self.B)


class CoverageFeedback(Variant):
    """O4: U +/- step per window from the PICP of the coverage events of the AS; alpha fixed."""
    name = "O4"

    def __init__(self, alpha: float, U: float, step: float = O4_STEP, U_min: float = O4_U_MIN,
                 U_max: float = O4_U_MAX, **kw):
        super().__init__(alpha, U, **kw)
        if not 0 < U_min <= U_max:
            raise ValueError("0 < U_min <= U_max")
        self.dU, self.U_min, self.U_max = float(step), float(U_min), float(U_max)
        self.U = min(self.U_max, max(self.U_min, self.U))
        self.w: Optional[int] = None
        self.hits = self.n = 0

    def step(self, t: int, cover: Optional[Tuple[int, bool]]) -> Optional[Dict]:
        self._start(t)
        if self.w is None:
            self.w = self.window(t)
        if cover is not None and self.window(cover[0]) == self.w and self.w is not None:
            self.n += 1
            self.hits += int(bool(cover[1]))
        wt = self.window(t)
        if wt is None or self.w is None or wt <= self.w:
            return None
        first = self.origin + self.w * self.N_w
        picp = self.hits / self.n if self.n else None
        if picp is not None:
            U = self.U + self.dU if picp >= self.target else self.U - self.dU
            self.U = min(self.U_max, max(self.U_min, round(U, 10)))
        rec = {"window": [first, first + self.N_w - 1], "PICP": picp, "n": self.n, "alpha": self.alpha, "U": self.U}
        self.w, self.hits, self.n = wt, 0, 0
        return rec

    def describe(self) -> Dict:
        return dict(super().describe(), step=self.dU, U_min=self.U_min, U_max=self.U_max)


class RegimeOracle(Variant):
    """O5: (alpha*, U*) of the regime in effect, switched at the step of each change."""
    name = "O5"

    def __init__(self, values: Dict[str, Tuple[float, float]], regimes: Sequence, Ts: float, **kw):
        if not regimes:
            raise ValueError("O5 needs the regimes of the run")
        self.values, self.regimes_s, self.Ts = dict(values), list(regimes), float(Ts)
        first = sorted(((r["t0"], r["scenario"]) if isinstance(r, dict) else tuple(r) for r in regimes),
                       key=lambda x: float(x[0]))[0][1]
        for _, s in (((r["t0"], r["scenario"]) if isinstance(r, dict) else tuple(r)) for r in regimes):
            if str(s) not in self.values:
                raise KeyError("O5: no calibration values for regime %s" % s)
        a, u = self.values[str(first)]
        super().__init__(a, u, **kw)
        self.regime = str(first)
        self.changes: Optional[List[Tuple[int, str]]] = None

    def _start(self, t: int) -> None:
        super()._start(t)
        if self.changes is None:
            self.changes = regime_steps(self.regimes_s, self.Ts, self.origin)

    def regime_of(self, t: int) -> str:
        cur = self.changes[0][1]
        for k, s in self.changes:
            if k <= t:
                cur = s
        return cur

    def step(self, t: int, cover: Optional[Tuple[int, bool]]) -> Optional[Dict]:
        self._start(t)
        s = self.regime_of(t)
        if s == self.regime:
            return None
        self.regime = s
        self.alpha, self.U = self.values[s]
        return {"regime": s, "alpha": self.alpha, "U": self.U}

    def describe(self) -> Dict:
        return dict(super().describe(), regimes=self.changes or self.regimes_s)


def make_variant(name: str, *, alpha0: float, U0: float, cal: Optional[Dict] = None, M: Optional[int] = None,
                 scenario: Optional[str] = None, regimes: Optional[Sequence] = None, Ts: float = 0.125,
                 N_w: int = N_W, origin: Optional[int] = None, target: float = PICP_NOM, init: str = "dtp",
                 lag: int = O3_LAG, refine: bool = False, no_U: str = "min", B: int = 1000,
                 step: float = O4_STEP, U_min: float = O4_U_MIN, U_max: float = O4_U_MAX) -> Variant:
    """Controller of variant `name` (O1..O5). alpha0 and U0: the values of O1 and the initial values of O3 (init
    "dtp"); cal, M and scenario: the calibration of O2, O4 and O3 with init "calibration"; regimes: O5 (the
    calibration of each regime for M)."""
    name = str(name).upper()
    if name not in VARIANTS:
        raise ValueError("variant must be one of %s" % ", ".join(VARIANTS))
    kw = dict(N_w=N_w, origin=origin, target=target)
    star = None
    if name in ("O2", "O4") or (name == "O3" and init == "calibration"):
        if cal is None or M is None or scenario is None:
            raise ValueError("%s needs the calibration file, M and the scenario" % name)
        star = calibration_values(cal, M, scenario)
    if name == "O1":
        return Fixed("O1", alpha0, U0, **kw)
    if name == "O2":
        return Fixed("O2", star[0], star[1], **kw)
    if name == "O3":
        if init not in ("dtp", "calibration"):
            raise ValueError("init must be dtp or calibration")
        a, u = star if init == "calibration" else (alpha0, U0)
        return WindowRecompute(a, u, lag=lag, refine=refine, no_U=no_U, B=B, **kw)
    if name == "O4":
        return CoverageFeedback(star[0], star[1], step=step, U_min=U_min, U_max=U_max, **kw)
    if cal is None or M is None:
        raise ValueError("O5 needs the calibration file and M")
    rs = list(regimes or [])
    names = {str(r["scenario"] if isinstance(r, dict) else r[1]) for r in rs}
    return RegimeOracle({s: calibration_values(cal, M, s) for s in names}, rs, Ts, **kw)
