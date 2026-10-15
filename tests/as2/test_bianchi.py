"""eq. (bianchi) and eq. (eff) against ns-3's reference file (src/wifi/examples/reference/bianchi_11g_difs.txt,
ns-3.48, produced by Bianchi11g.m)."""
from pathlib import Path

import numpy as np
import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.bianchi import EtaTable, eta, mean_frame, ns3_11g, parse_reference, probs, solve, tau_of_p

REF = Path(__file__).parent / "data" / "bianchi_11g_difs.txt"
ACK = {6: 6, 9: 6, 12: 12, 18: 12, 24: 24, 36: 24, 48: 24, 54: 24}   # Bianchi11g.m / generate_bianchi.m


def test_fixed_point_solves_the_paper_form():
    n = np.arange(1, 51)
    tau, p = solve(n, 15, 6)
    W = 16.0
    # paper form (with 1 - 2p), away from p = 1/2
    m = np.abs(1 - 2 * p) > 1e-3
    paper = 2 * (1 - 2 * p) / ((1 - 2 * p) * (15 + 2) + p * (15 + 1) * (1 - (2 * p) ** 6))
    assert np.allclose(tau[m], paper[m], rtol=1e-10)
    assert np.allclose(p, 1 - (1 - tau) ** (n - 1), rtol=1e-12)
    assert np.isclose(tau[0], 2 / (W + 1))      # one station: p = 0, tau = 2/(CWmin + 2)
    assert np.all(np.diff(tau) < 0) and np.all(np.diff(p) > 0)


def test_against_ns3_reference():
    ref = parse_reference(REF.read_text())
    assert sorted(ref) == [6, 9, 12, 18, 24, 36, 48, 54]
    rows = []
    for r, tab in sorted(ref.items()):
        n = sorted(tab)
        v = np.array([tab[k] for k in n])
        ex = ns3_11g(n, r * 1e6, ACK[int(r)] * 1e6)
        gr = ns3_11g(n, r * 1e6, ACK[int(r)] * 1e6, grid=True)
        rel = np.max(np.abs(ex - v) / v)
        absg = np.max(np.abs(gr - v))
        rows.append((r, rel, absg))
        assert rel < 2.5e-3        # exact fixed point vs the script's grid search over tau
        assert absg < 6e-5         # the script's grid search reproduces the file (4 decimals)
    print("\nrate  max rel err (exact solver)  max abs err (grid, Mbit/s)")
    for r, a, b in rows:
        print("%4g  %.2e  %.1e" % (r, a, b))


def test_eta_table():
    d = D.example()
    e = EtaTable.from_dtp(d)
    assert e.nmax >= 32 and len(e.table) == e.nmax
    assert 0 < e(1) < 1 and e(0) == 0
    # eta decreases with n beyond the peak of n = 2 (one station leaves idle backoff slots)
    assert np.all(np.diff(e.table[1:]) < 0)
    assert np.isclose(e(40), eta(np.array([40]), e.T_s, e.T_c, 9e-6, 15, 6)[0])
    mf = mean_frame(d)
    assert mf["T_c"] < mf["T_s"]
