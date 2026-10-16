# Fleet-DT reference implementation: interfaces

This file fixes the interfaces between the programs of Fleet-DT. The numbering of equations,
definitions and requirements (R1 to R9) follows the IEEE Access paper "Fleet-DT: A Network
Digital Twin with Dynamic Bandwidth Allocation for Fleets of Unmanned Surface Vehicles".

## 1. Programs and hosts

| Program | Host | Language | Directory | Paper |
|---|---|---|---|---|
| Vessel regulator (Mosquitto plugin `fdt_regulator.so`) | vessel RP4, vessel MQTT broker | C | `vessel/regulator/` | Def. 13, R1 |
| DTI program `fdt_dti` | vessel RP4 | C | `vessel/dti/` | Def. 9, R3 |
| Telemetry publisher `mav_publisher.py` | vessel RP4 | Python ≥ 3.7 | `vessel/publishers/` | R2 |
| Video publisher `zed_publisher.py` | vessel RP4 | Python ≥ 3.7 | `vessel/publishers/` | R2 |
| Synthetic load `raw_load.py` | vessel RP4 (bench, tests) | Python ≥ 3.7 | `vessel/publishers/` | Section VI |
| AS (prediction, admission control, outer loop) | GS PC | Python | `fleetdt/as_service.py`, `fleetdt/outer.py` | eq. 2, eq. 3, Alg. 1, Def. 15 |
| VE | GS PC | Python + HTML | `fleetdt/ve/` | Def. 11 |
| Analysis of the DTA (Section VI metrics) | any | Python | `fleetdt/analysis.py` | Section VI |
| Broker configuration generator | GS PC | Python | `fleetdt/gsconf.py` | R5, R8, R9 |
| Channel monitor `fdt_chmon` | GS radio (Bullet M2, OpenWrt, ath9k) | C | `radio/channel_monitor/` | eq. y(t), R6 |
| TX99 injector | extra Bullet M2 (OpenWrt with ATH9K_TX99) | sh | `radio/injector/` | Section VI-B |
| Radio configuration | the Bullet M2 radios | UCI | `radio/openwrt/` | R6 |
| Mosquitto build, patches and configuration | vessels and GS | sh, patch, conf | `mosquitto/` | R1, R5, R8, R9 |
| Host configuration (chrony, sysctl, routes, systemd) | vessels and GS | conf | `system/` | R4, R8 |
| Code shared by the C programs (DTP reader, step clock, wire formats) | all | C | `common/` | Section 2, 3 |

## 2. DTP file (`dtp.json`, one file for every host)

```json
{
  "epoch": 1767225600.0,
  "Ts": 0.125, "T1": 0.020, "T2": 0.010,
  "alpha": 0.125, "U": 0.8, "C0": 300000,
  "tcp": {"mss": 1448, "h_T": 80}, "mqtt": {"h_A": 4},
  "R_PHY": 24000000, "h_W": 570.25, "T_DIFS": 0.000037, "T_bo": 0.0000675,
  "vessels": [1, 2],
  "topics": [
    {"name": "state", "rate": 8,  "bound": 48},
    {"name": "imu",   "rate": 32, "bound": 48},
    {"name": "gps",   "rate": 8,  "bound": 80},
    {"name": "baro",  "rate": 8,  "bound": 32},
    {"name": "power", "rate": 8,  "bound": 40},
    {"name": "mag",   "rate": 16, "bound": 40},
    {"name": "video", "rate": 30, "bound": 14000}
  ],
  "order":  ["state", "imu", "gps", "video", "baro", "power", "mag"],
  "needed": ["state", "imu", "gps"]
}
```

| Field | Meaning |
|---|---|
| `epoch` | Unix time (GS clock) of the start of step 0 of the operation. Step t starts at `epoch + t*Ts`. With `null`, every program uses 1767225600 (2026-01-01T00:00:00Z), so t fits the u32 field until 2043. |
| `Ts`, `T1`, `T2` | Step and phase durations in seconds; `T3 = Ts - T1 - T2` |
| `alpha`, `U`, `C0` | Gain of eq. (2), target utilization, initial prediction (bytes per step) |
| `c_b`, `c_m` | Optional coefficients of the bound c(x, n) = c_b x + c_m n of eq. (5) (channel bytes of n messages with x content bytes). Without them, the readers of the paper stack (`common/fdt_paper.c`, `fleetdt/as_model.py`, `vessel/publishers/video_layers.py`) compute the cost bound of the paper per topic: c_b = 1 + 2 (h_T + h_W) / MSS, c_m,l = c_b h_P,l + 2 (h_T + h_W) + 3 (h_A + 2 h_T + 2 h_W), h_P,l the PUBLISH header of boat/b\<V\>/tx/\<name\> (QoS 2). `dtp.json` gives none of them |
| `tcp`, `mqtt` | MSS and h_T (largest TCP/IP header of a segment, options included; 1448, 80), h_A (bytes of one MQTT control packet; 4) |
| `R_PHY`, `h_W` | Bit rate of the radios (bit/s) and fixed bytes per frame at R_PHY (DIFS or AIFS, mean backoff, PHY preamble and header, signal extension, MAC header, LLC, FCS, SERVICE and tail bits, mean OFDM padding, SIFS, MAC ACK); derivation in emu/README.md, "Fixed bytes per frame h_W" |
| `T_DIFS`, `T_bo` | DIFS (AIFS[AC_BE] of the stations under EDCA) and mean backoff (CWmin/2 slots) inside h_W, s; the channel monitor charges h_W - R_PHY (T_DIFS + T_bo) / 8 per frame (Section 6) |
| `topics` | Uplink topics in the DTP order (topic index l = position + 1); `rate` in messages per second, `bound` = largest content of one message in bytes (before the 6-byte step header) |
| `order`, `needed` | Initial sigma and N of the AS (used until the first `fleet/order`) |

Phases of step t (wall clock of each host, synchronized by chrony):

| Instant | Event |
|---|---|
| `start(t) = epoch + t*Ts` | phase 1: the DTI program publishes D_i(t) |
| `start(t) + T1` | phase 2: the AS closes M(t), computes Ĉ(t), b(t), p(t), publishes p(t) |
| `start(t) + T1 + T2` | phase 3: the vessel regulator forwards the messages of step t |
| `start(t) + Ts` | end of step t: the channel monitor publishes y(t); the GS discards messages of step t that arrive later |

A raw message belongs to step t when it reaches the vessel regulator in the interval
`[start(t-1) + T1 + T2, start(t) + T1 + T2)`, so the vessel regulator forwards no message between
the end of phase 3 of step t-1 and the start of phase 3 of step t.

## 3. Topics

Prefix `boat/b<i>/` with the vessel index i (1..M). All binary fields in network byte order.

| Topic | Broker where published | Publisher | QoS | Payload |
|---|---|---|---|---|
| `boat/b<i>/raw/<topic>` | vessel | publishers | 0 | content of the message (≤ `bound` bytes); no bridge forwards `raw/#` |
| `boat/b<i>/demand` | vessel (control bridge → GS) | DTI program | 1 | `t:u32, L:u8, d_1..d_L:u32` |
| `boat/b<i>/levels` | GS (control bridge → vessel) | AS | 1 | `t:u32, L:u8, p_1..p_L:u8` (p ∈ {0,1,2}) |
| `boat/b<i>/goals` | GS (control bridge → vessel) | operation control software | 1 | JSON `{"t":…, "lat":…, "lon":…, "speed":…}` |
| `boat/b<i>/tx/<topic>` | vessel (data bridge → GS) | vessel regulator | p_l (2 → QoS 2, 1 → QoS 1) | `t:u32, s:u16` + content; s = sequence number of the message in its topic and step, from 0 |
| `fleet/order` | GS | operation control software | 1 | JSON `{"t":…, "order":[names], "needed":[names]}` |
| `fleet/throughput` | GS (from the GS radio) | channel monitor | 0 | JSON `{"t":…, "y":bytes, "Tb":s, "To":s}` |
| `fleet/as` | GS | AS | 0 | JSON `{"t":…, "C_hat":…, "K_hat":…, "b":…, "U":…, "alpha":…, "m":…, "p":[…], "M":[…], "order":[…], "needed":[…]}` (topic indices from 1) |
| `boat/b<i>/regulator` | vessel (control bridge → GS) | vessel regulator | 0 | JSON log of step t, published at start(t+1) + 1 ms (phase 1): `{"t", "have_d", "have_p", "late_us", "outq", "p":[…], "d":[…], "rx":[…], "fwd":[…], "held":[…], "drop":[…], "x":[…]}`; per topic: raw messages of the step, forwarded, withheld, dropped over the queue capacity, forwarded bytes with the step header; `late_us` = delay of phase 3 after start(t) + T1 + T2; `outq` = unsent bytes of the data bridge socket at that instant (-1 unknown) |

The video publisher splits each JPEG frame into messages of at most `bound` bytes; the content of
each message starts with an 8-byte header `frame:u32, k:u16, n:u16` (index of the frame, index of the
message in the frame, number of messages of the frame).

## 4. Bridges (configured on the GS broker, R5)

| Bridge | Address on the vessel | Topics | QoS |
|---|---|---|---|
| `b<i>-data` | vessel:1884 (dedicated listener) | `boat/b<i>/tx/# in 2` | 2 |
| `b<i>-ctl` | vessel:1883 | `boat/b<i>/demand in 1`, `boat/b<i>/regulator in 0`, `boat/b<i>/levels out 1`, `boat/b<i>/goals out 1` | 1 |

Both bridges use `cleansession true`, `bridge_max_packet_size` unset, `max_inflight_messages 65534`
(R9). `python -m fleetdt.gsconf` writes both configurations. Patch
`mosquitto/patches/0002-bridge-apply-set_tcp_nodelay.patch` applies `set_tcp_nodelay true` to the
bridge sockets, and patch `0001-plugin_tick-keep-pending-deadline.patch` keeps the deadline of a
plugin tick when another event wakes the broker, so phase 3 starts within 1 ms (R8).

Every MQTT client of Fleet-DT sets TCP_NODELAY on its socket (paho-mqtt leaves Nagle's algorithm
on): without it, a small PUBLISH that follows a PUBACK of the same client waits for the delayed ACK of
the broker, about 40 ms on Linux, and the priority table misses the end of phase 2.

## 5. Demand and policing (eq. 5, Def. 13)

For topic l with rate r_l and bound s_l: n_l = ceil(r_l * Ts), x_l = n_l * s_l, and
d_{i,l}(t) = ceil(c_b * (x_l + 6 n_l) + c_m * n_l) (the 6 bytes of the step header count as content
on the channel). The vessel regulator adds, for each forwarded message of topic l of step t with
content x (after the step header is added, so x includes the 6 bytes), c_b * x + c_m and withholds
the message that would raise the sum above d_{i,l}(t), together with the later messages of topic l in
step t, so the messages of a topic leave in arrival order with no gap. Within a level, the topics leave
in the order of the DTP field `order`. Without the demand table or the priority table of step t, the
vessel regulator withholds every message of step t. A topic with no raw message in the last
`--idle` steps (default 8) gets d_{i,l}(t) = 0 from the DTI program.

## 6. Channel monitor (Section V-B)

In each step: `Tb` = busy time of the channel in the step (survey), `To` = airtime at R_PHY of the
frames that the GS radio receives and of the frames that it sends and the stations acknowledge,
`To = 8 (B_rx + B_tx_acked) / R_PHY + (N_rx + N_tx_acked) * 8 h_W_busy / R_PHY`, with
`h_W_busy = h_W - R_PHY (T_DIFS + T_bo) / 8` (only the busy part of each frame: the survey counts the
DIFS and the backoff as idle time, so `Ts - Tb` holds them already), and
`y(t) = R_PHY (Ts - Tb + To) / 8` bytes, clamped to [0, R_PHY Ts / 8]. With no survey of step t, the
monitor publishes nothing.
