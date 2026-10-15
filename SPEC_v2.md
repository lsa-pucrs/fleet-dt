# Fleet-DT v2: interfaces

This file fixes the interfaces between the programs of Fleet-DT v2. The model is
`/home/claude/fleetdt/revision/sec3_problem.tex`, `sec4_model.tex`, `sec5_platform.tex`
(paper Sections III to V, v2). Symbols follow the paper. Where this file and the paper disagree, the
paper wins and this file must be corrected.

## 1. Programs

| Program | Host | Language | Directory | Paper |
|---|---|---|---|---|
| Vessel regulator and DTI program (one Mosquitto plugin `fdt_regulator.so`) | vessel | C | `vessel/regulator/` | Defs. demand, table, regulator; PR1, PR3 |
| Synthetic sources `fdt_source.py` (stream and output topics, from the DTP or a size trace) | vessel | Python ≥ 3.7 | `vessel/publishers/` | Def. topic |
| AS v2 (prediction, ACI, budget, Algorithm 1, f(t), DTA log) | GS | Python | `fleetdt/as2/` | Section IV-C, IV-D |
| Operation control stub (σ_t, N_i(t) schedule) | GS | Python | `fleetdt/as2/opctl.py` | Section III-C |
| Monitor bridge (AP monitor records → MQTT `fleet/monitor`) | GS side of the emulator | Python | `emu/monitor_bridge.py` | PR6 |
| Emulator `fdt-emu` (ns-3 real time, one Wi-Fi node per host, L2 relay to the VMs) | emulation host | C++ (ns-3.48) | `emu/ns3/` | Section VI |
| Orchestrator (VMs, emulator, runs, clock probe) | emulation host | Python | `emu/run.py`, `emu/vm/` | Section VI |
| Baselines B1 to B5, overlay TDMA, slotted variant | vessel and GS | C / Python | `vessel/regulator/` (modes), `fleetdt/as2/` (modes) | Section VI |
| Analysis of the DTA and of the ground truth | any | Python | `fleetdt/as2/analysis.py` | Section VI |

## 2. DTP v2 (`dtp_v2.json`, one file for every host)

```json
{
  "epoch": null,
  "Ts": 0.125, "T1": 0.020, "T2": 0.010,
  "eps": 0.002, "d_rx": 0.002, "d_m": 0.001,
  "T_tick": 0.001, "L_w": 16384, "B_cap": 1048576, "T_min": 0.005,
  "beta": 0.05, "gamma": 0.01, "alpha1": 0.5, "lambda": 0.25,
  "phy": {"slot": 9e-6, "sifs": 10e-6, "t_pre": 16e-6, "t_sig": 4e-6, "t_sym": 4e-6, "t_se": 6e-6,
          "h_mac": 38, "aifsn_sta": 3, "cwmin_sta": 15, "cwmax_sta": 1023, "m_b": 6,
          "aifsn_ap": 1, "cwmin_ap": 7, "cwmax_ap": 1023,
          "basic_rates": [6e6, 12e6, 24e6], "rate_init": 24e6},
  "tcp": {"mss": 1448, "h_T": 80},
  "mqtt": {"h_A": 4},
  "vessels": [1, 2, 3, 4],
  "topics": [
    {"name": "state", "rate": 8,  "jitter": 0.002, "n": 1,  "s": 48,   "kind": "stream", "qos": 2},
    {"name": "imu",   "rate": 32, "jitter": 0.002, "n": 1,  "s": 48,   "kind": "stream", "qos": 0},
    {"name": "gps",   "rate": 8,  "jitter": 0.002, "n": 1,  "s": 80,   "kind": "stream", "qos": 0},
    {"name": "baro",  "rate": 8,  "jitter": 0.002, "n": 1,  "s": 32,   "kind": "stream", "qos": 0},
    {"name": "power", "rate": 8,  "jitter": 0.002, "n": 1,  "s": 40,   "kind": "stream", "qos": 0},
    {"name": "mag",   "rate": 16, "jitter": 0.002, "n": 1,  "s": 40,   "kind": "stream", "qos": 0},
    {"name": "video", "rate": 15, "jitter": 0.010, "n": 16, "s": 1400, "kind": "output", "qos": 0}
  ],
  "required": ["state"],
  "goals": {"rate": 1, "s": 200},
  "order":  ["state", "imu", "gps", "video", "baro", "power", "mag"],
  "needed": ["state", "imu", "gps"]
}
```

Rules: every topic of `required` has `qos` 2 and every other topic `qos` 0; `order` starts with the
topics of `required`; `T_W = T3 − 2 eps − d_rx`; with `epoch: null` every program uses 1767225600.
`h_P` of topic ℓ = PUBLISH fixed header (1 + remaining-length bytes) + 2 + len(`boat/b<i>/tx/<name>`)
+ (2 if qos > 0) + 6 (step header); the AS and the plugin compute it with the largest vessel index.

## 3. Step clock

Step t ≥ 1 is `[epoch + (t−1) Ts, epoch + t Ts)` (GS clock). Phase 1 starts at the step start, phase 2 at
`+T1`, phase 3 (v_i(t) on the vessel clock) at `+T1+T2`. Data window W(t) on the GS clock:
`[start + T1 + T2 + eps, start + Ts − eps − d_rx)`. Quiet interval Q(t): from the end of W(t−1) to
`start(t) + T1`. Collection window of vessel i: `[v_i(t−1), v_i(t))`; an output belongs to the step
whose collection window holds the arrival of its last message.

## 4. Topics and wire formats

Prefix `boat/b<i>/`. Binary fields in network byte order. JSON keys are fixed.

| Topic | Broker / bridge | Publisher | QoS | Payload |
|---|---|---|---|---|
| `boat/b<i>/raw/<topic>` | vessel, no bridge | sources | 0 | content (≤ s bytes); output topics: content starts with `out:u32, k:u16, n:u16` |
| `boat/b<i>/demand` | control bridge, uplink | plugin (phase 1) | 1 | JSON, Section 4.1 |
| `boat/b<i>/levels` | control bridge, downlink | AS (phase 2) | 1 | JSON `{"t":t, "p":[L ints], "k":[L ints], "o":[L topic indices, π order of topics]}` |
| `boat/b<i>/goals` | control bridge, downlink | operation control stub | 1 | JSON, ≤ goals.s bytes |
| `boat/b<i>/tx/<topic>` | data bridge, uplink | plugin (phase 3) | DTP qos (0 or 2) | `t:u32, seq:u16` + content |
| `boat/b<i>/regulator` | control bridge, uplink | plugin (phase 1) | 0 | JSON vessel log of step t−1 (diagnostics, Section 4.2) |
| `fleet/order` | GS | operation control stub | 1 | JSON `{"t":t, "order":[names], "needed":{"<i>":[names], ...}}` |
| `fleet/monitor` | GS | monitor bridge | 0 | JSON batch, Section 4.3 |
| `fleet/as` | GS | AS | 0 | JSON per step: vertex 0 of G(t) and the admitted units (Section 4.4) |

### 4.1 Demand (Definition demand)

```json
{"t": 17, "v": 2,
 "d": [[[x, n]], [[x, n]], ..., [[x1, n1], [x2, n2], ...]],
 "held": [3, 0, ...],
 "B": {"x0": 0, "n0": 0, "x2": 0, "n2": 0},
 "rep": {"t": 16, "rel": [[n_released per unit of topic 1], ...], "disc": 0, "tab": true},
 "late_us": 120}
```

`d[ℓ]` lists the units of topic ℓ (index from 0 in the DTP order): one pair for a stream topic; one
pair per output for an output topic, first the held complete outputs in release order, then the outputs
of the arrival-curve bound `ceil(rate (T1 + T2 + jitter))`, each `(n·s, n)`. `held[ℓ]` = number of
leading units of topic ℓ that are held (exact). x counts content bytes only; the cost adds h_P (which
holds the 6-byte step header) per message. `B` = unacknowledged bytes and messages of
the data-bridge socket split by QoS (SIOCOUTQ plus the release offsets). `rep` reports step t−1:
released messages per admitted unit, discarded messages of admitted units, whether the table of t−1
arrived before v_i(t−1).

### 4.2 Vessel log (`regulator`)

`{"t", "have_tab", "late_us", "nsd_max", "outq_p1", "rx":[L], "rel":[L], "disc":[L], "xrel":[L], "tick_us_max"}`.

### 4.3 Monitor batch (PR6)

Published every 5 ms of emulated time (and at the end of each phase by the AS request is not needed):

```json
{"from_us": 1767225600123000, "to_us": 1767225600128000,
 "busy": [[start_us, end_us], ...],
 "fr": [[start_us, end_us, sta, dir, ip_len, rate_kbps, ok, kind], ...]}
```

Times in µs on the GS clock (the emulator uses the host clock, which the GS VM follows; the offset of
the GS VM is part of the ground truth). `busy`: CCA-busy or TX intervals of the AP PHY. `fr`: one record
per frame exchange that the AP sends or receives: `sta` = vessel index (0 = AP broadcast, −1 = outside
the fleet), `dir` 0 uplink / 1 downlink, `ip_len` bytes of the IP packet (0 for non-IP), `end_us`
includes the MAC acknowledgment, `ok` 1 when acknowledged, `kind` 0 data, 1 beacon, 2 management or
control. The AS computes b(t), e(t), κ(t), R_i(t), R_{0,i}(t) (Defs. busy, capacity; Section V-B).

### 4.4 AS record (`fleet/as`, DTA line)

`{"t", "M":[participating], "b", "b_hat", "eta", "kappa_hat", "alpha", "Z", "K", "cB":[per vessel],
"A":[[i, ℓ, k], ...], "cost_A", "p":{"<i>":[L]}, "k":{"<i>":[L]}, "f_prev", "kappa_prev",
"e_prev", "z_prev", "FR", "mode"}` — `_prev` fields refer to step t−1, which the AS closes in phase 2 of t.

## 5. Bridges (GS broker)

| Bridge | Vessel address | Topics | Notes |
|---|---|---|---|
| `b<i>-data` | vessel:1884 (dedicated listener) | `boat/b<i>/tx/# in 2` | cleansession true, set_tcp_nodelay true (patch 0002), max_inflight_messages 0 (no limit) |
| `b<i>-ctl` | vessel:1883 | `demand in 1`, `regulator in 0`, `levels out 1`, `goals out 1` | priority: both hosts mark the control connection with socket priority 262 (UP 6) via nftables `meta priority` on port 1883 and DSCP EF |

Vessel and GS brokers: `set_tcp_nodelay true` globally. Routes to the peer: `quickack 1 rto_min 20ms`.
`net.ipv4.tcp_slow_start_after_idle = 0`.

## 6. Plugin behaviour (Def. regulator)

| Event | Action |
|---|---|
| raw message of topic ℓ | append to the collection of the current step (stream) or to the open output (by `out`); an output completes when n messages arrived; assign by Section 3 |
| phase 1 start (vessel clock) | publish `demand` for step t (Section 4.1) and `regulator` for t−1 |
| `levels` for step t arrives before v_i(t) | store the table |
| v_i(t) | apply the table or the default table (R only, arrival bound over Ts) |
| tick (period T_tick) in phase 3 | while admitted messages remain and `SIOCOUTQNSD(data socket) + released_this_tick < L_w`: release the next message in order (level 2 units, then level 1, each by `o` then output index), QoS from the DTP, payload `t, seq` + content |
| end of step (vessel clock) | discard unreleased messages of step t; count discarded admitted messages |

Modes (plugin option `mode`): `fleetdt` (default), `b1` (forward everything at once, no table), `b2`
(forward everything, DSCP per class: state → EF, telemetry → AF41, video → BE), `b3` (strict priority
by `order`, no table, discard at step end), `b5` (Fleet-DT without policing and without discard),
`slotted` (release outside R only inside the slot in `levels` field `"slot":[start_us, end_us]`),
`tdma` (overlay TDMA: fixed equal slots, no admission).

## 7. Emulator (`emu/`)

| Item | Specification |
|---|---|
| Hosts | one QEMU/KVM VM per vessel and one for the GS; Debian 13; 1 vCPU, 768 MB; overlay qcow2 on one base image with the stack installed |
| Data NIC | `-netdev dgram` UDP pair per VM to the emulator; IP 10.10.0.1 (GS), 10.10.0.(10+i) (vessel i); static neighbor entries; IPv6 off |
| Management | `-netdev user` with hostfwd for ssh; vsock CID 100+i for the clock probe |
| Emulator | ns-3.48 RealtimeSimulatorImpl (HardLimit 50 ms, lag logged every 10 ms); SpectrumWifiPhy, 802.11g ERP-OFDM, QoS MAC (EDCA), one AP (GS) and M stations; FdNetDevice per VM fd bridged by an L2 relay (MAC rewrite) to the WifiNetDevice of the node; DSCP/priority of the IP header → UP via SocketPriorityTag |
| Channel | per-link loss matrix or positions with TwoRayGround + Nakagami; scenario JSON |
| Interference | external CSMA BSS (AP2/STA2, OnOff schedule) and non-CSMA WaveformGenerator (duty schedule) |
| Rates | ConstantRate or Minstrel per scenario |
| AP EDCA | AC_BE of the AP: aifsn_ap, cwmin_ap; stations: aifsn_sta, cwmin_sta |
| Outputs | monitor UDP stream to `monitor_bridge.py` (→ `fleet/monitor` on the GS broker); ground-truth trace (every PHY TX/RX/CCA event of every node, binary); lag log |
| Clock | each vessel VM starts with an offset in [−50, 50] ms and a frequency error in [−50, 50] ppm (adjtimex) before chrony; the probe reads each VM clock over vsock every 100 ms and logs the offset to the host clock |
