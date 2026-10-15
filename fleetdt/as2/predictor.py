"""Prediction of the capacity (eq. pred) and the scores z(t) = kappa(t)/kappa_hat(t) with Z_t(alpha)."""
from __future__ import annotations

import bisect
import math
from collections import deque
from typing import Callable, Iterable, List, Optional


class Predictor:
    """b_hat(t) = (1 - lambda) b_hat(t-1) + lambda b(t) (b_hat(t) = b_hat(t-1) without b(t));
    kappa_hat(t) = (1 - b_hat(t)) eta(M + 1)."""

    def __init__(self, lam: float, eta: Callable[[int], float], M: int, b_hat0: float = 0.0):
        if not 0 < lam <= 1:
            raise ValueError("lambda must be in (0, 1]")
        self.lam, self.eta, self.M = lam, eta, M
        self.b_hat = float(b_hat0)

    def step(self, b: Optional[float]) -> float:
        if b is not None:
            b = min(1.0, max(0.0, float(b)))
            self.b_hat = (1 - self.lam) * self.b_hat + self.lam * b
        return self.kappa_hat()

    def kappa_hat(self, n: Optional[int] = None) -> float:
        return (1.0 - self.b_hat) * self.eta(self.M + 1 if n is None else n)


class Scores:
    """Sorted scores of the DTA and of the steps before t. Z_t(alpha) = z_(ceil(alpha N)) for alpha in
    (0, 1], 0 for alpha <= 0, and `empty` while N = 0. `window` keeps the last `window` scores of the
    operation (None: all); the initial scores of the DTA always stay."""

    def __init__(self, initial: Iterable[float] = (), window: Optional[int] = None, empty: float = 1.0):
        self.sorted: List[float] = sorted(float(z) for z in initial if z is not None and math.isfinite(z))
        self.window = window
        self.recent: deque = deque()
        self.empty = float(empty)

    def __len__(self) -> int:
        return len(self.sorted)

    def add(self, z: float) -> None:
        if z is None or not math.isfinite(z):
            return
        bisect.insort(self.sorted, float(z))
        if self.window is not None:
            self.recent.append(float(z))
            if len(self.recent) > self.window:
                old = self.recent.popleft()
                k = bisect.bisect_left(self.sorted, old)
                del self.sorted[k]

    def Z(self, alpha: float) -> float:
        if alpha <= 0:
            return 0.0
        N = len(self.sorted)
        if N == 0:
            return self.empty
        k = min(N, max(1, int(math.ceil(min(alpha, 1.0) * N - 1e-12))))
        return self.sorted[k - 1]
