"""Level of adaptive conformal inference (Def. aci, eq. aci) and the counter F_R of Theorem 1."""
from __future__ import annotations

from typing import List, Optional


def project(alpha: float, gamma: float, beta: float) -> float:
    return min(1.0, max(-gamma * (1.0 - beta), alpha))


class ACI:
    """alpha_{t+1} = min{1, max{-gamma (1 - beta), alpha_t + gamma (beta - f(t))}}.
    `alpha` is alpha_t of the current step; update(f) consumes f(t) and returns alpha_{t+1}."""

    def __init__(self, beta: float, gamma: float, alpha1: float):
        if not 0 < beta < 1 or not 0 < gamma < 1 or not 0 <= alpha1 <= 1:
            raise ValueError("beta, gamma in (0, 1) and alpha1 in [0, 1]")
        self.beta, self.gamma, self.alpha1 = beta, gamma, alpha1
        self.alpha = alpha1
        self.FR = 0          # steps with alpha_t <= 0 and f(t) = 1
        self.T = 0           # steps consumed
        self.sum_f = 0
        self.sum_D = 0.0     # sum of the corrections of the projection

    @property
    def floor(self) -> float:
        return -self.gamma * (1.0 - self.beta)

    def update(self, f: int) -> float:
        f = 1 if f else 0
        if self.alpha <= 0 and f == 1:
            self.FR += 1
        raw = self.alpha + self.gamma * (self.beta - f)
        nxt = project(raw, self.gamma, self.beta)
        self.sum_D += nxt - raw
        self.alpha = nxt
        self.T += 1
        self.sum_f += f
        return nxt

    def bound(self, T: Optional[int] = None, FR: Optional[int] = None) -> float:
        """Right side of eq. (thm)."""
        T = self.T if T is None else T
        FR = self.FR if FR is None else FR
        if T <= 0:
            return float("inf")
        return self.beta + self.alpha1 / (self.gamma * T) + (1 - self.beta) * (1 + FR) / T


def theorem_check(f: List[int], alphas: List[float], beta: float, gamma: float, alpha1: float):
    """For every prefix T of a run: (left, right) of eq. (thm), with F_R(T) from the levels alphas[t]
    used in step t (alphas[0] = alpha_1)."""
    out = []
    s = FR = 0
    for T, (ft, a) in enumerate(zip(f, alphas), start=1):
        s += ft
        if a <= 0 and ft:
            FR += 1
        out.append((s / T, beta + alpha1 / (gamma * T) + (1 - beta) * (1 + FR) / T, FR))
    return out
