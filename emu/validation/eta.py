#!/usr/bin/env python3
"""Efficiency eta(n) of the paper (sec2_patch.tex, eqs. bianchi and eff) and the reference model of ns-3
(src/wifi/examples/reference/Bianchi11g.m), for IEEE 802.11g ERP-OFDM.

  eta.py table [--rate 24] [--n 1-16] [--ns3 bianchi.csv]        DCF (DIFS) case of wifi-bianchi
  eta.py edca  [--rate 24] [--n 1-16] [--ip 1500] [--aifsn 3 --cwmin 15] [--meas fdt.csv]
  eta.py tcp   [--rate 24] [--M 2]                               TCP upload expectation of the testbed

`table` compares, for n saturated stations and 1500-byte packets, the throughput of ns-3 (CSV n,Mbps from
emu/validation/bianchi.sh), of Bianchi11g.m (DIFS variant) and of the paper's model, S(n) = eta(n) L/T_succ,
which is (eff) with the payload time in the numerator.
"""
import argparse
import math
import sys

T_SYM, T_PRE_SIG, T_SE = 4e-6, 20e-6, 6e-6
SLOT, SIFS = 9e-6, 10e-6


def t_ppdu(nbytes, rate_mbps):
    ndbps = rate_mbps * 4            # data bits per OFDM symbol (T_SYM 4 us)
    return T_PRE_SIG + T_SYM * math.ceil((16 + 6 + 8 * nbytes) / ndbps) + T_SE


def ack_rate(rate_mbps):
    return max(r for r in (6, 12, 24) if r <= rate_mbps)


def tau_pc(n, cwmin, m):
    """fixed point of (bianchi): tau and p_c for n saturated stations"""
    if n == 1:
        return 2.0 / (cwmin + 2), 0.0
    lo, hi = 0.0, 1.0
    for _ in range(200):           # p in [0,1): g(p) = p - (1 - (1 - tau(p))^(n-1)) is increasing
        p = (lo + hi) / 2
        # (bianchi) with (1 - (2p)^m)/(1 - 2p) written as a finite sum (no 0/0 at p = 1/2)
        tau = 2 / ((cwmin + 2) + p * (cwmin + 1) * sum((2 * p) ** i for i in range(m)))
        if p - (1 - (1 - tau) ** (n - 1)) > 0:
            hi = p
        else:
            lo = p
    return tau, p


def eta(n, t_succ, t_coll, cwmin=15, m=6, slot=SLOT):
    tau, p = tau_pc(n, cwmin, m)
    ptr = 1 - (1 - tau) ** n
    ps = n * tau * (1 - tau) ** (n - 1) / ptr
    den = (1 - ptr) * slot + ptr * ps * t_succ + ptr * (1 - ps) * t_coll
    return ptr * ps * t_succ / den, tau, p


def bianchi11g_m(n, data_rate, ack_rt, l_data=1500 * 8, cwmin=15, cwmax=1023):
    """port of ns-3 Bianchi11g.m (DIFS variant), Mb/s"""
    b = 1 / (cwmin + 1)
    ep = l_data / (1 - b)
    t_symbol = 4e-6
    nd = data_rate * t_symbol * 1e6
    t_data = 20e-6 + t_symbol * math.ceil((16 + (24 + 4) * 8 + l_data + 48 + 6) / nd) + 6e-6
    na = ack_rt * t_symbol * 1e6
    t_ack = 20e-6 + t_symbol * math.ceil((16 + 14 * 8 + 6) / na) + 6e-6
    difs = 28e-6
    t_s = t_data + SIFS + t_ack + difs
    t_c = t_data + difs
    t_S = t_s / (1 - b) + SLOT
    w = cwmin + 1
    m = int(round(math.log2((cwmax + 1) / (cwmin + 1))))
    # same grid search as the .m file
    best, tau = 1e9, 0
    for k in range(10000):
        t1 = k / 9999
        p = 1 - (1 - t1) ** (n - 1)
        ps = sum((2 * p) ** i for i in range(m))
        tp = 2 / (1 + w + p * w * ps)
        if abs(t1 - tp) < best:
            best, tau = abs(t1 - tp), tp
    ptr = 1 - (1 - tau) ** n
    pss = n * tau * (1 - tau) ** (n - 1) / ptr
    return pss * ptr * ep / ((1 - ptr) * SLOT + ptr * pss * t_S + ptr * (1 - pss) * t_c) / 1e6


def nrange(s):
    a, _, b = s.partition("-")
    return range(int(a), int(b or a) + 1)


def cmd_table(a):
    meas = {}
    if a.ns3:
        for line in open(a.ns3):
            if line[0].isdigit():
                n, v = line.split(",")[:2]
                meas[int(n)] = float(v)
    ar = ack_rate(a.rate)
    # wifi-bianchi frame: 1500 B packet + 6 B upper-layer header (as Bianchi11g.m) + 28 B MAC (no QoS)
    t_data = t_ppdu(1500 + 6 + 28, a.rate)
    t_ack = t_ppdu(14, ar)
    difs = SIFS + 2 * SLOT
    t_succ = t_data + SIFS + t_ack + difs
    t_coll = t_data + difs
    payload_t = 1500 * 8 / (a.rate * 1e6)
    print("n,ns3_Mbps,bianchi_m_Mbps,paper_eta,paper_S_Mbps,err_ns3_vs_m,err_ns3_vs_paper")
    for n in nrange(a.n):
        e, tau, p = eta(n, t_succ, t_coll)
        s_paper = e * (1500 * 8) / t_succ / 1e6
        s_m = bianchi11g_m(n, a.rate, ar)
        m = meas.get(n)
        f = lambda x: ("%.4f" % x) if x is not None else ""
        print("%d,%s,%.4f,%.4f,%.4f,%s,%s" % (n, f(m), s_m, e, s_paper,
                                               f((m - s_m) / s_m if m else None), f((m - s_paper) / s_paper if m else None)))
    print("# T_succ %.1f us, T_coll %.1f us, payload %.1f us" % (t_succ * 1e6, t_coll * 1e6, payload_t * 1e6),
          file=sys.stderr)


def cmd_edca(a):
    """saturated uplink of n vessels with the EDCA AC_BE parameters of the emulator (QoS data frames)"""
    meas = {}
    if a.meas:
        for line in open(a.meas):
            if line[0].isdigit():
                n, v = line.split(",")[:2]
                meas[int(n)] = float(v)
    ar = ack_rate(a.rate)
    t_data = t_ppdu(a.ip + 8 + 26 + 4, a.rate)      # IP + LLC/SNAP + QoS MAC header + FCS
    t_ack = t_ppdu(14, ar)
    aifs = SIFS + a.aifsn * SLOT
    t_succ = t_data + SIFS + t_ack + aifs
    t_coll = t_data + aifs + SIFS + t_ack           # EIFS-like: the ACK timeout before the next access
    print("n,meas_Mbps,paper_eta,paper_S_ip_Mbps,err")
    for n in nrange(a.n):
        e, tau, p = eta(n, t_succ, t_coll, cwmin=a.cwmin)
        s = e * (a.ip * 8) / t_succ / 1e6
        m = meas.get(n)
        print("%d,%s,%.4f,%.4f,%s" % (n, "%.4f" % m if m else "", e, s, "%.4f" % ((m - s) / s) if m else ""))
    print("# T_succ %.1f us, T_coll %.1f us" % (t_succ * 1e6, t_coll * 1e6), file=sys.stderr)


def cmd_tcp(a):
    """TCP upload of the testbed: per segment one data exchange of the vessel and one TCP ACK of the AP"""
    ar = ack_rate(a.rate)
    t_ack = t_ppdu(14, ar)
    seg = t_ppdu(a.mss + a.h_t + 8 + 26 + 4, a.rate) + SIFS + t_ack + SIFS + 3 * SLOT
    tack = t_ppdu(a.h_t + 8 + 26 + 4, a.rate) + SIFS + t_ack + SIFS + 1 * SLOT
    # mean backoff without collisions: CWmin/2 slots of the station and of the AP
    nocoll = seg + tack + (15 / 2 + 7 / 2) * SLOT
    good0 = a.mss * 8 / nocoll / 1e6
    t_mean = (seg + tack) / 2
    e, tau, p = eta(a.M + 1, t_mean, t_mean)
    good_eta = e * a.mss * 8 / (seg + tack) / 1e6
    print("TCP upload, MSS %d, rate %g: no-collision bound %.2f Mb/s; eta(M+1=%d) = %.4f -> %.2f Mb/s total"
          % (a.mss, a.rate, good0, a.M + 1, e, good_eta))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["table", "edca", "tcp"])
    ap.add_argument("--rate", type=float, default=24)
    ap.add_argument("--n", default="1-16")
    ap.add_argument("--ns3", default="")
    ap.add_argument("--meas", default="")
    ap.add_argument("--ip", type=int, default=1500)
    ap.add_argument("--aifsn", type=int, default=3)
    ap.add_argument("--cwmin", type=int, default=15)
    ap.add_argument("--mss", type=int, default=1448)
    ap.add_argument("--h-t", dest="h_t", type=int, default=52)
    ap.add_argument("--M", type=int, default=2)
    a = ap.parse_args()
    {"table": cmd_table, "edca": cmd_edca, "tcp": cmd_tcp}[a.cmd](a)


if __name__ == "__main__":
    main()
