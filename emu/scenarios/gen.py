#!/usr/bin/env python3
"""Channel scenarios S1 to S5 of the paper (Section VI-C) for the emulator, seeded.

  python3 emu/scenarios/gen.py --out emu/scenarios [--M 2 4 8] [--seed 1] [--duration 180]

writes S<k>-m<M>.json (seed 1: the reference files of the repository; the experiment matrix calls make()
with the seed of each run and puts the scenario into the run file). The times of the schedules and of
the waypoints are relative to the start of the first step of the run (emulator option --runStartUs).

| Scenario | Channel |
|---|---|
| S1 clean | every node hears every node (loss 70 dB between any two nodes), ConstantRate 24 Mb/s (ERP-OFDM) |
| S2 external network | S1 plus a second BSS (ap2, sta2) on the same channel; sta2 sends UDP bursts to ap2 (CSMA) with a seeded on/off schedule: on U[1, 6] s at U[2, 8] Mb/s (1400-byte packets), off U[2, 8] s |
| S3 jammer | S1 plus a transmitter without carrier sense (WaveformGenerator, 20 dBm, 70 dB to every node) with a seeded schedule: on U[1, 6] s with period U[0.5, 5] ms and duty cycle U[0.05, 0.3], off U[2, 8] s |
| S4 hidden vessels | S1 with two pairs of vessels that do not hear each other (loss 200 dB: v1-v2 and v3-v4; M = 2: v1-v2 only); every vessel hears the AP |
| S5 mobility | survey mission over a lagoon: AP on the shore at (0, 0, 6 m); each vessel runs a lawnmower pattern (lanes along x, from 100 m to 800 m from the shore, 30 m apart) in its own strip of y in [-400, 400] m at a seeded speed U[1, 2] m/s from a seeded point of its pattern; TwoRayGround (2.412 GHz, antenna heights 6 m and 1.5 m) with Nakagami fading m = 5 (a strong line-of-sight component over water), Minstrel rate adaptation, standard 11a (see emu/README.md: ns-3 Minstrel needs the OFDM rates without DSSS); the PHY rates of the vessels move between 36 and 54 Mb/s with the distance |

The scenario seed also seeds ns-3 (RngSeedManager), so it fixes the backoff draws and the fading.

Paper scenarios S1 to S9 of the drafted Section VI (Table "Channel scenarios"), written with --paper as
P-S<k>-m<M>.json (the files above stay as they are: they belong to the earlier campaign), plus the queue
variants of S1 and S3 (P-S1-q100-m<M>.json, P-S1-q1000-m<M>.json, P-S1-codel-m<M>.json, the same for S3):

  python3 emu/scenarios/gen.py --paper --out emu/scenarios [--M 2 4] [--seed 1] [--duration 180]

Every paper scenario: ConstantRate 24 Mb/s ERP-OFDM for data and control responses, basic rates ERP
6/12/24 (beacons and management at 6 Mb/s), 20 dBm, RTS/CTS off, no A-MSDU/A-MPDU, MAC queue 100
packets / 500 ms (ns-3 defaults), phy "yans" except S3 and S8 ("spectrum", needed by the injector);
the parameters are in PAPER_PARAMS below and in the "params" field of each file.

| Scenario | Channel (as implemented) |
|---|---|
| S1 | matrix, 70 dB between any two nodes (-50 dBm) |
| S2 | S1 + second BSS (ap2, sta2) at 70 dB: sta2 -> ap2 UDP 1400 B with CSMA, on U[1, 6] s at U[2, 8] Mb/s, off U[2, 8] s; ap2 beacons every 100 TU at DSSS 1 Mb/s (basic mode 0 of its BSS) |
| S3 | S1 + injector without carrier sense (WaveformGenerator, 20 dBm, 70 dB to every node: -50 dBm, above the energy-detection threshold -62 dBm), on U[1, 6] s with period U[0.5, 5] ms and duty cycle U[0.05, 0.3], off U[2, 8] s |
| S4 | S1 with v1-v2 and v3-v4 at 200 dB (M = 2: v1-v2) |
| S5 | lawnmower survey (AP at 6 m, vessels at 1.5 m, lanes 100-800 m, U[1, 2] m/s), TwoRayGround + Nakagami m = 5, fixed 24 Mb/s |
| S6 | S1 with the AP-vessel loss on a seeded walk in [95, 106] dB (+-1 dB per s, reflected, linear between the seconds); preamble detection down to -90 dBm (note at s6_range_db in PAPER_PARAMS) |
| S7 | trajectories of S5 (same seed) over the sea: two-ray with the phase of the direct and reflected rays, Fresnel coefficient of sea water (eps_r 70, sigma 5 S/m, vertical polarization), Miller-Brown roughness (sigma_h = heave / sqrt 2), heave 0.3 m with period U[2, 8] s per vessel, roll 10 deg with the same period, half-wave dipole patterns |
| S8 | S3 (same schedule as S3 of the seed) with the injector at -72 dBm (92 dB to every node, below -62 dBm) and the AP-vessel loss 84 dB, -64 dBm (note at s8_fleet_loss_db in PAPER_PARAMS) |
| S9 | S2 (same schedule as S2 of the seed) with the second BSS at 102 dB to the fleet nodes (mean -82 dBm, the MinimumRssi of ns-3 preamble detection) and Nakagami m = 5 power fading per frame on these links |

Queue variants (S1, S3): q100 = the base file; q1000: MAC queue 1000 packets on the AP and the vessels
with a MaxDelay of 60 s (no lifetime drop, like a Linux txqueuelen of 1000); codel: mq + one CoDel per
access category (target 5 ms, interval 100 ms, limit 1000 packets) on the vessels above a MAC queue of 4
packets.

Every paper scenario sets "channel_number" 6 (the BSS of the radios, radio/openwrt/*/wireless; the
emulator sets the ChannelSettings of every PHY to it), so the two-ray models of S5 and S7 use the
frequency of channel 6, 2.437 GHz (channel_hz).

Modifiers of a paper scenario for the experiments (each returns a copy and records itself in the file):

| Function | Experiment | Effect |
|---|---|---|
| with_outage(sc, dur_s, ...) | E0 | loss of 200 dB between the AP and vessel 1 during [start_s, start_s + dur_s) of scenario time (a timed entry of "channel"."loss"); start_s seeded in start_range_s (after the warm-up of 30 s) when not given; "outage" records the parameters |
| with_rate_adaptation(sc) | E3, ablation R7 | Minstrel rate adaptation over the OFDM rates 6 to 54 Mb/s instead of the constant 24 Mb/s; standard 11a (see the function) |
| make_paper_sequence(M, seed, ...) | E5 | one run over the regimes S1, S3, S1 (130 s each by default) on the spectrum PHY, with the injector of S3 only in the S3 regime; "regimes" lists [{"t0", "scenario"}] |
"""
import argparse
import json
import math
import os
import random

EDCA = {"aifsn_ap": 1, "cwmin_ap": 7, "cwmax_ap": 1023, "aifsn_sta": 3, "cwmin_sta": 15, "cwmax_sta": 1023}
PRIO = {"dscp": {"46": 6, "34": 5}, "tcp_ports": {"1883": 6}}
SCENARIOS = ("S1", "S2", "S3", "S4", "S5")


def base(name, M, seed):
    return {"name": name, "M": M, "seed": int(seed), "run": 1,
            "rate": {"manager": "constant", "data": "ErpOfdmRate24Mbps", "control": "ErpOfdmRate24Mbps",
                     "non_unicast": "ErpOfdmRate6Mbps"},
            "edca": dict(EDCA), "tx_power_dbm": 20, "mac_queue": {"packets": 100, "max_delay_ms": 500},
            "monitor_period_ms": 5, "beacon_tu": 100, "prio": json.loads(json.dumps(PRIO)),
            "channel": {"model": "matrix", "default_loss_db": 70, "loss": []}}


def onoff(rng, duration, on=(1.0, 6.0), off=(2.0, 8.0)):
    """[(t0, t1)] alternating off/on periods over [0, duration), starting with an off period"""
    out, t = [], rng.uniform(*off)
    while t < duration:
        d = rng.uniform(*on)
        out.append((round(t, 3), round(min(duration, t + d), 3)))
        t += d + rng.uniform(*off)
    return out


def lawnmower(rng, i, M, duration, x0=100.0, x1=800.0, ylim=400.0, lane=30.0, z=1.5):
    """waypoints [[t, x, y, z], ...] of vessel i: lanes along x inside strip i of [-ylim, ylim]"""
    w = 2 * ylim / M
    ys = -ylim + (i - 1) * w + lane / 2
    nl = max(1, int(w // lane))
    pts = []
    for k in range(nl):           # boustrophedon over the lanes of the strip, then back
        y = ys + k * lane
        a, b = (x0, x1) if k % 2 == 0 else (x1, x0)
        pts += [(a, y), (b, y)]
    pts += list(reversed(pts))
    seg = [math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]) for k in range(len(pts) - 1)]
    total = sum(seg)
    v = rng.uniform(1.0, 2.0)
    s = rng.uniform(0, total)       # start point along the closed path
    out, t = [], 0.0
    # walk from arc length s for `duration` seconds (wrapping around)
    k, acc = 0, 0.0
    while acc + seg[k] <= s:
        acc += seg[k]
        k = (k + 1) % len(seg)
    f = (s - acc) / seg[k] if seg[k] > 0 else 0.0
    x = pts[k][0] + f * (pts[k + 1][0] - pts[k][0])
    y = pts[k][1] + f * (pts[k + 1][1] - pts[k][1])
    out.append([0.0, round(x, 2), round(y, 2), z])
    while t < duration:
        nx, ny = pts[k + 1]
        d = math.hypot(nx - x, ny - y)
        t += d / v
        x, y = nx, ny
        out.append([round(t, 3), round(x, 2), round(y, 2), z])
        k = (k + 1) % len(seg)
    return out, v


def make(sid, M, seed, duration=180.0):
    """scenario sid (S1..S5) for M vessels, seeded; times relative to the start of the run"""
    if sid not in SCENARIOS:
        raise ValueError("scenario must be one of %s" % (SCENARIOS,))
    rng = random.Random("%s:%d:%d" % (sid, M, seed))
    sc = base("%s-m%d-s%d" % (sid, M, seed), M, seed)
    sc["scenario_id"] = sid
    if sid == "S2":
        sch = []
        for t0, t1 in onoff(rng, duration):
            sch.append([t0, t1, round(rng.uniform(2e6, 8e6), -3), 1400])
        sc["external_bss"] = {"dir": "up", "data": "ErpOfdmRate24Mbps", "schedule": sch}
    elif sid == "S3":
        sch = []
        for t0, t1 in onoff(rng, duration):
            sch.append([t0, t1, round(rng.uniform(500, 5000)), round(rng.uniform(0.05, 0.3), 3), 20])
        sc["jammer"] = {"schedule": sch}
    elif sid == "S4":
        pairs = [("v1", "v2")] if M < 4 else [("v1", "v2"), ("v3", "v4")]
        sc["channel"]["loss"] = [[a, b, 200] for a, b in pairs]
    elif sid == "S5":
        sc["standard"] = "11a"
        sc["rate"] = {"manager": "minstrel", "non_unicast": "OfdmRate6Mbps"}
        sc["basic_modes"] = ["OfdmRate6Mbps", "OfdmRate12Mbps", "OfdmRate24Mbps"]
        pos = {"gs": [0, 0, 6]}
        speeds = {}
        for i in range(1, M + 1):
            wp, v = lawnmower(rng, i, M, duration)
            pos["v%d" % i] = wp
            speeds["v%d" % i] = round(v, 3)
        sc["channel"] = {"model": "tworay", "frequency_hz": 2.412e9,
                         "nakagami": {"m0": 5, "m1": 5, "m2": 5, "d1": 80, "d2": 200},
                         "positions": pos}
        sc["speeds_mps"] = speeds
    return sc


# ------------------------------------------------------------------------------------------------
# paper scenarios S1 to S9 (drafted Section VI) and the queue variants of S1 and S3

PAPER_SCENARIOS = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9")
QUEUE_VARIANTS = ("q100", "q1000", "codel")
QUEUE_SCENARIOS = ("S1", "S3")
PAPER_PARAMS = {
    "tx_dbm": 20.0,
    "fleet_loss_db": 70.0,           # -50 dBm between any two nodes
    "cca_ed_dbm": -62.0,             # ns-3 default CcaEdThreshold
    "s3_injector_rx_dbm": -50.0,     # above the energy-detection threshold
    "s8_injector_rx_dbm": -72.0,     # below the energy-detection threshold
    # S8: with the vessels at -50 dBm the AP receives them 22 dB above a -72 dBm injector, where ns-3
    # decodes 24 Mb/s frames without error (measured PER 0 down to an SINR of 12 dB, 0.014 at 10 dB);
    # 84 dB puts the vessels 8 dB above the injector, where the frames that overlap a burst fail
    "s8_fleet_loss_db": 84.0,
    "s9_ext_loss_db": 102.0,         # mean -82 dBm = MinimumRssi of ThresholdPreambleDetectionModel
    "s9_fading_m": 5.0,
    # S6: ns-3 (noise figure 7 dB) decodes 24 Mb/s frames without error down to the -82 dBm preamble
    # limit and drops every frame below it, so a walk across that limit only switches the link on and
    # off (and the vessel deassociates after 10 lost beacons); with the preamble detection down to
    # -90 dBm (the 6 Mb/s range of a real receiver) the walk crosses the SNR region of 24 Mb/s failures
    "s6_range_db": [95.0, 106.0],
    "s6_step_db_per_s": 1.0,
    "s6_preamble_min_rssi_dbm": -90.0,
    "s5_nakagami_m": 5.0,
    "s7_heave_m": 0.3,
    "s7_period_s": [2.0, 8.0],
    "s7_roll_deg": 10.0,
    "s7_eps_r": 70.0,
    "s7_sigma_s_m": 5.0,
    "codel": {"target_ms": 5, "interval_ms": 100, "limit_packets": 1000, "device_packets": 4},
    "q1000": {"packets": 1000, "max_delay_ms": 60000},
    "channel_number": 6,             # 2.4 GHz channel of the BSS (radio/openwrt/*/etc/config/wireless)
    # E0 outages: loss between the AP and the target vessel, seeded start inside the run after the warm-up
    "outage_loss_db": 200.0,
    "outage_start_range_s": [30.0, 100.0],
    # E5: regimes S1, S3, S1 of 130 s each (390 s run)
    "sequence_regime_s": 130.0,
}


def channel_hz(ch):
    """center frequency (Hz) of 2.4 GHz channel ch = 1..13 (IEEE 802.11-2020, 17.3.8.4.2)"""
    if not 1 <= int(ch) <= 13:
        raise ValueError("2.4 GHz channel must be 1..13")
    return 2.407e9 + 5e6 * int(ch)


def paper_file(sid, M, queue=None):
    q = "" if queue in (None, "", "base") else "-" + queue
    return "P-%s%s-m%d.json" % (sid, q, M)


def _loss_to_all(sc, M, node, loss, extra=()):
    nodes = ["gs"] + ["v%d" % i for i in range(1, M + 1)] + list(extra)
    sc["channel"].setdefault("loss", [])
    sc["channel"]["loss"] += [[node, n, loss] for n in nodes]


def walk_series(rng, lo, hi, step, n):
    """n values (one per second) of a walk of +-step in [lo, hi], reflected at both ends"""
    x = rng.uniform(lo, hi)
    out = [round(x, 3)]
    for _ in range(n - 1):
        x += step if rng.random() < 0.5 else -step
        if x > hi:
            x = 2 * hi - x
        if x < lo:
            x = 2 * lo - x
        out.append(round(x, 3))
    return out


def make_paper(sid, M, seed, duration=180.0, queue=None):
    """paper scenario sid (S1..S9) for M vessels, seeded, with queue variant None/"q100"/"q1000"/"codel";
    times relative to the start of the run (emulator --runStartUs or --shiftS)"""
    if sid not in PAPER_SCENARIOS:
        raise ValueError("scenario must be one of %s" % (PAPER_SCENARIOS,))
    if queue not in (None, "base") + QUEUE_VARIANTS:
        raise ValueError("queue must be one of %s" % (QUEUE_VARIANTS,))
    P = PAPER_PARAMS
    # the schedules of S8 and S9 are those of S3 and S2 with the same seed; the trajectories of S7 are
    # those of S5
    key = {"S8": "S3", "S9": "S2", "S7": "S5"}.get(sid, sid)
    rng = random.Random("paper:%s:%d:%d" % (key, M, seed))
    rng2 = random.Random("paper-extra:%s:%d:%d" % (sid, M, seed))
    name = paper_file(sid, M, queue)[:-5] + "-s%d" % seed
    sc = base(name, M, seed)
    sc["scenario_id"] = sid
    sc["scenario_set"] = "paper-v2"
    sc["queue_variant"] = queue or "q100"
    sc["phy"] = "spectrum" if sid in ("S3", "S8") else "yans"
    sc["channel_number"] = P["channel_number"]
    sc["cca_ed_dbm"] = P["cca_ed_dbm"]
    sc["tx_power_dbm"] = P["tx_dbm"]
    sc["basic_modes"] = ["ErpOfdmRate6Mbps", "ErpOfdmRate12Mbps", "ErpOfdmRate24Mbps"]
    sc["channel"]["default_loss_db"] = P["fleet_loss_db"]
    params = {"R_PHY_mbps": 24, "rts": "off", "aggregation": "off", "tx_dbm": P["tx_dbm"]}
    if sid in ("S2", "S9"):
        sch = [[t0, t1, round(rng.uniform(2e6, 8e6), -3), 1400] for t0, t1 in onoff(rng, duration)]
        sc["external_bss"] = {"dir": "up", "data": "ErpOfdmRate24Mbps", "schedule": sch, "beacon_tu": 100,
                              "basic_modes": ["DsssRate1Mbps", "ErpOfdmRate6Mbps", "ErpOfdmRate12Mbps",
                                              "ErpOfdmRate24Mbps"]}
        params["ext_beacon"] = "DSSS 1 Mb/s every 102.4 ms"
        if sid == "S9":
            L = P["s9_ext_loss_db"]
            fleet = ["gs"] + ["v%d" % i for i in range(1, M + 1)]
            sc["channel"]["loss"] = [[x, f, L] for x in ("ap2", "sta2") for f in fleet]
            sc["channel"]["fading"] = [[x, f, P["s9_fading_m"]] for x in ("ap2", "sta2") for f in fleet]
            params["ext_rx_dbm_mean"] = P["tx_dbm"] - L
            params["ext_fading_m"] = P["s9_fading_m"]
        else:
            params["ext_rx_dbm"] = P["tx_dbm"] - P["fleet_loss_db"]
    elif sid in ("S3", "S8"):
        rx = P["s3_injector_rx_dbm"] if sid == "S3" else P["s8_injector_rx_dbm"]
        sch = [[t0, t1, round(rng.uniform(500, 5000)), round(rng.uniform(0.05, 0.3), 3), P["tx_dbm"]]
               for t0, t1 in onoff(rng, duration)]
        sc["jammer"] = {"schedule": sch}
        _loss_to_all(sc, M, "jam", P["tx_dbm"] - rx)
        params["injector_rx_dbm"] = rx
        if sid == "S8":
            for i in range(1, M + 1):
                sc["channel"]["loss"].append(["gs", "v%d" % i, P["s8_fleet_loss_db"]])
            params["ap_vessel_rx_dbm"] = P["tx_dbm"] - P["s8_fleet_loss_db"]
            params["sinr_at_ap_during_bursts_db"] = round(P["tx_dbm"] - P["s8_fleet_loss_db"] - rx, 1)
    elif sid == "S4":
        pairs = [("v1", "v2")] if M < 4 else [("v1", "v2"), ("v3", "v4")]
        sc["channel"]["loss"] = [[a, b, 200] for a, b in pairs]
        params["hidden_pairs"] = pairs
    elif sid in ("S5", "S7"):
        pos = {"gs": [0, 0, 6]}
        speeds = {}
        for i in range(1, M + 1):
            wp, v = lawnmower(rng, i, M, duration)
            pos["v%d" % i] = wp
            speeds["v%d" % i] = round(v, 3)
        sc["speeds_mps"] = speeds
        if sid == "S5":
            m = P["s5_nakagami_m"]
            sc["channel"] = {"model": "tworay", "frequency_hz": channel_hz(P["channel_number"]),
                             "nakagami": {"m0": m, "m1": m, "m2": m, "d1": 80, "d2": 200}, "positions": pos}
            params["fading_m"] = m
        else:
            motion = {}
            for i in range(1, M + 1):
                motion["v%d" % i] = {"heave_m": P["s7_heave_m"],
                                     "period_s": round(rng2.uniform(*P["s7_period_s"]), 3),
                                     "phase_rad": round(rng2.uniform(0, 2 * math.pi), 4),
                                     "roll_deg": P["s7_roll_deg"],
                                     "roll_phase_rad": round(rng2.uniform(0, 2 * math.pi), 4)}
            # sea water at 2.4 GHz: eps_r ~ 70, sigma ~ 5 S/m (ITU-R P.527); vertical dipoles; rms
            # height of a sinusoidal swell of amplitude A: A / sqrt(2) (Miller-Brown roughness)
            sc["channel"] = {"model": "sea", "frequency_hz": channel_hz(P["channel_number"]), "eps_r": P["s7_eps_r"],
                             "sigma_s_m": P["s7_sigma_s_m"], "polarization": "V",
                             "rough_sigma_m": round(P["s7_heave_m"] / math.sqrt(2), 4), "antenna": "dipole",
                             "min_distance_m": 1, "positions": pos, "motion": motion}
    elif sid == "S6":
        lo, hi = P["s6_range_db"]
        n = int(math.ceil(duration)) + 2
        sc["channel"]["walk"] = {"dt_s": 1.0, "pairs": [["gs", "v%d" % i,
                                                         walk_series(rng, lo, hi, P["s6_step_db_per_s"], n)]
                                                        for i in range(1, M + 1)]}
        sc["preamble_min_rssi_dbm"] = P["s6_preamble_min_rssi_dbm"]
        params["walk_db"] = [lo, hi]
        params["ap_vessel_rx_dbm"] = [P["tx_dbm"] - hi, P["tx_dbm"] - lo]
    if queue == "q1000":
        sc["fleet_queue"] = dict(P["q1000"])
    elif queue == "codel":
        c = P["codel"]
        sc["aqm"] = {"type": "codel", "nodes": "stations", "limit_packets": c["limit_packets"],
                     "target_ms": c["target_ms"], "interval_ms": c["interval_ms"]}
        sc["station_queue"] = {"packets": c["device_packets"]}
    sc["params"] = params
    return sc


# ------------------------------------------------------------------------------------------------
# modifiers of the experiments (E0 outages, E3 ablation R7) and the regime sequence of E5

MATRIX_ONLY = "the %s modifier needs a scenario with the matrix channel model (S1 to S4, S6, S8, S9)"


def outage_start(seed, M, target, dur_s, start_range_s=None):
    """seeded start (s of scenario time, ms resolution) of an outage of dur_s seconds, uniform over the
    starts that keep the outage inside start_range_s (default PAPER_PARAMS outage_start_range_s)"""
    lo, hi = start_range_s or PAPER_PARAMS["outage_start_range_s"]
    if dur_s <= 0 or hi < lo:
        raise ValueError("outage: dur_s > 0 and start_range_s = [lo, hi] with lo <= hi")
    rng = random.Random("outage:%s:%d:%d:%g" % (target, int(M), int(seed), float(dur_s)))
    return round(rng.uniform(float(lo), float(hi)), 3)


def with_outage(sc, dur_s, start_s=None, target="v1", peer="gs", loss_db=None, seed=None, start_range_s=None):
    """E0: copy of matrix scenario sc with the link peer-target at loss_db (default 200 dB) during
    [start_s, start_s + dur_s) of scenario time; start_s None: seeded by (seed or sc["seed"], M, target,
    dur_s) in start_range_s. The emulator reads the timed entry [peer, target, loss, t0, t1] of
    "channel"."loss" (fdt-emu.cc FdtMatrixLoss), which takes precedence over the walk and the fixed loss of
    the pair; "outage" records {"target", "peer", "start_s", "dur_s", "loss_db"}."""
    if (sc.get("channel") or {}).get("model", "matrix") != "matrix":
        raise ValueError(MATRIX_ONLY % "outage")
    M = int(sc["M"])
    names = ["gs"] + ["v%d" % i for i in range(1, M + 1)]
    if target not in names or peer not in names or target == peer:
        raise ValueError("outage: target and peer must be two of %s" % names)
    dur_s = float(dur_s)
    if start_s is None:
        start_s = outage_start(sc["seed"] if seed is None else seed, M, target, dur_s, start_range_s)
    start_s = float(start_s)
    if start_s < 0 or dur_s <= 0:
        raise ValueError("outage: start_s >= 0 and dur_s > 0")
    L = float(PAPER_PARAMS["outage_loss_db"] if loss_db is None else loss_db)
    out = json.loads(json.dumps(sc))
    out["channel"].setdefault("loss", []).append([peer, target, L, start_s, round(start_s + dur_s, 6)])
    out["outage"] = {"target": target, "peer": peer, "start_s": start_s, "dur_s": dur_s, "loss_db": L}
    out["name"] = "%s-out%s%g" % (sc["name"], target, dur_s)
    return out


def with_rate_adaptation(sc):
    """E3 ablation R7: copy of sc with the Minstrel rate adaptation instead of the constant 24 Mb/s.

    ns-3 Minstrel adapts over every rate of the standard, and in 802.11g these include the DSSS rates,
    which SpectrumWifiPhy cannot send in a 20 MHz channel (S3 needs the spectrum PHY) and which the
    radios do not use (OpenWrt legacy_rates 0). The ablation therefore runs 802.11a (OFDM 6 to 54 Mb/s,
    the OFDM rates of the radios with supported_rates opened; the same exchange airtime as ERP-OFDM:
    SIFS 16 us = 10 us + 6 us signal extension, slot 9 us) on the default 5 GHz channel of ns-3 (36),
    because ns-3 allows 802.11a only in the 5 GHz band; the matrix loss model does not depend on the
    frequency. Basic rates OFDM 6/12/24 Mb/s, beacons and management at 6 Mb/s. Only for scenarios with
    the matrix model and without DSSS beacons of a second BSS (S1, S3, S4, S6, S8)."""
    if (sc.get("channel") or {}).get("model", "matrix") != "matrix":
        raise ValueError(MATRIX_ONLY % "R7")
    ext = sc.get("external_bss") or {}
    if any(str(m).startswith("Dsss") for m in ext.get("basic_modes", [])):
        raise ValueError("R7: the DSSS beacons of the second BSS do not exist in 802.11a")
    out = json.loads(json.dumps(sc))
    out["standard"] = "11a"
    out["rate"] = {"manager": "minstrel", "non_unicast": "OfdmRate6Mbps"}
    out["basic_modes"] = ["OfdmRate6Mbps", "OfdmRate12Mbps", "OfdmRate24Mbps"]
    out.pop("channel_number", None)
    if ext:
        out["external_bss"]["data"] = str(ext.get("data", "ErpOfdmRate24Mbps")).replace("ErpOfdm", "Ofdm")
        if "basic_modes" in ext:
            out["external_bss"]["basic_modes"] = [str(m).replace("ErpOfdm", "Ofdm") for m in ext["basic_modes"]]
    out["ablation"] = "R7"
    out.setdefault("params", {})["rate_adaptation"] = ("Minstrel over OFDM 6-54 Mb/s, standard 11a, ns-3 channel 36 "
                                                       "(deviation: 11g Minstrel would try DSSS rates)")
    out["name"] = sc["name"] + "-R7"
    return out


SEQUENCE_SCENARIOS = ("S1", "S3")


def make_paper_sequence(M, seed, regimes=("S1", "S3", "S1"), regime_s=None, duration=None):
    """E5: one scenario over consecutive regimes (each "S1" or "S3", regime_s seconds each, default
    PAPER_PARAMS sequence_regime_s = 130 s): S1 on the spectrum PHY with the injector of S3 present in
    every regime and transmitting only inside the S3 regimes (the on/off schedule of make_paper("S3")
    drawn for the length of the regime, shifted to its start; the same draw in every S3 regime). The
    spectrum PHY holds over the whole run, because the injector of S3 needs it. "regimes" lists
    [{"t0": start in s of scenario time, "scenario": id}]; duration (default: the regimes plus 30 s)
    bounds the last regime."""
    regimes = list(regimes)
    if not regimes or any(r not in SEQUENCE_SCENARIOS for r in regimes):
        raise ValueError("regimes must be a non-empty list of %s" % (SEQUENCE_SCENARIOS,))
    L = float(PAPER_PARAMS["sequence_regime_s"] if regime_s is None else regime_s)
    if L <= 0:
        raise ValueError("regime_s must be > 0")
    total = L * len(regimes)
    duration = total + 30.0 if duration is None else float(duration)
    sc = make_paper("S1", M, seed, duration)
    s3 = make_paper("S3", M, seed, L)
    sc["name"] = "P-SEQ-%s-m%d-s%d" % ("-".join(regimes), M, seed)
    sc["scenario_id"] = "SEQ"
    sc["phy"] = "spectrum"
    sc["channel"]["loss"] = list(s3["channel"]["loss"])      # the injector at -50 dBm from every node
    sch = []
    starts = []
    for k, r in enumerate(regimes):
        t0 = round(k * L, 6)
        t1 = duration if k == len(regimes) - 1 else round((k + 1) * L, 6)
        starts.append({"t0": t0, "scenario": r})
        if r == "S3":
            for a, b, per, duty, pw in s3["jammer"]["schedule"]:
                a, b = round(t0 + a, 3), round(min(t0 + b, t1), 3)
                if b > a:
                    sch.append([a, b, per, duty, pw])
    if sch:
        sc["jammer"] = {"schedule": sch}
    else:                    # no S3 regime: the emulator creates no injector node, so no loss entries for it
        sc["channel"]["loss"] = [e for e in sc["channel"]["loss"] if "jam" not in e[:2]]
    sc["regimes"] = starts
    sc["params"] = dict(sc["params"], regime_s=L, injector_rx_dbm=s3["params"]["injector_rx_dbm"],
                        regimes=list(regimes))
    return sc


def write_paper(out, Ms, seed, duration):
    paths = []
    for M in Ms:
        jobs = [(sid, None) for sid in PAPER_SCENARIOS] + [(sid, q) for sid in QUEUE_SCENARIOS for q in QUEUE_VARIANTS]
        for sid, q in jobs:
            sc = make_paper(sid, M, seed, duration, q)
            p = os.path.join(out, paper_file(sid, M, q))
            with open(p, "w") as f:
                json.dump(sc, f, indent=1)
            paths.append(p)
    return paths


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--M", type=int, nargs="+", default=None, help="default 2 4 8 (old set), 2 4 (--paper)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--paper", action="store_true", help="write the paper scenarios P-S1..P-S9 and queue variants")
    a = ap.parse_args()
    if a.paper:
        for p in write_paper(a.out, a.M or [2, 4], a.seed, a.duration):
            print(p)
        return
    for M in a.M or [2, 4, 8]:
        for sid in SCENARIOS:
            sc = make(sid, M, a.seed, a.duration)
            p = os.path.join(a.out, "%s-m%d.json" % (sid, M))
            with open(p, "w") as f:
                json.dump(sc, f, indent=1)
            print(p)


if __name__ == "__main__":
    main()
