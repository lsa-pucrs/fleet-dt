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
| `c_y` | Optional (default 1): smallest ratio C(t)/y(t) of a saturation test, in (0, 1]; eq. (2) smooths c_y y(t-1) (`fleetdt.analysis.calibrate_c_y`) |
| `T_m` | Optional (default 0 s): margin of phase 3, T_m = eps + delta_GS (eps = 2 max chrony max error + 1 ms, delta_GS = largest delivery time of the GS broker measured in Q); K_hat(t) = C_hat(t) (T3 - T_m) / Ts. TODO: no measurement of the two terms exists yet, so every DTP of the repository leaves it at 0 (`fleetdt.analysis.calibrate_margins`) |
| `B_ctl_bar` | Optional (default 0 bytes): largest bytes of control traffic outside the data bridges in phase 3 of a calibration step; used by the fidelity metrics only (K_y(t)). TODO as for T_m |
| `c_b`, `c_m` | Optional coefficients of the bound c(x, n) = c_b x + c_m n of eq. (5) (channel bytes of n messages with x content bytes). Without them, the readers of the paper stack (`common/fdt_paper.c`, `fleetdt/as_model.py`, `vessel/publishers/video_layers.py`) compute the cost bound of the paper (eq. cons) per topic: c_b = 1 + 2 (h_T + h_W) / MSS, c_k = 2 (h_T + h_W) (the last segment of the message and its TCP acknowledgment; QoS 0 has no MQTT acknowledgment), c_m,l = c_b h_P,l + c_k, h_P,l the PUBLISH header of boat/b\<V\>/tx/\<name\> (QoS 0, no packet identifier). An optional `c_k` overrides c_k. `dtp.json` gives none of them |
| `tcp`, `mqtt` | MSS and h_T (largest TCP/IP header of a segment, options included; 1448, 80), h_A (bytes of one MQTT control packet; 4; outside the cost bound since the data bridges use QoS 0) |
| `R_PHY`, `h_W` | Bit rate of the radios (bit/s) and fixed bytes per frame at R_PHY (DIFS or AIFS, mean backoff, PHY preamble and header, signal extension, MAC header, LLC, FCS, SERVICE and tail bits, mean OFDM padding, SIFS, MAC ACK); derivation in emu/README.md, "Fixed bytes per frame h_W" |
| `T_DIFS`, `T_bo` | DIFS (AIFS[AC_BE] of the stations under EDCA) and mean backoff (CWmin/2 slots) inside h_W, s; the channel monitor charges h_W - R_PHY (T_DIFS + T_bo) / 8 per frame (Section 6) |
| `topics` | Uplink topics in the DTP order (topic index l = position + 1); `rate` in messages per second, `bound` = largest content of one message in bytes (before the 6-byte step header) |
| `order`, `needed` | Initial sigma and N of the AS (used until the first `fleet/order`) |

Phases of step t (wall clock of each host, synchronized by chrony):

| Instant | Event |
|---|---|
| `start(t) = epoch + t*Ts` | phase 1: the DTI program publishes D_i(t) |
| `start(t) + T1` | phase 2: the AS closes M(t), computes Ĉ(t), b(t), p(t), publishes p(t) (below) |
| `start(t) + T1 + T2` | phase 3: the vessel regulator forwards the messages of step t |
| `start(t) + Ts` | end of step t: the channel monitor publishes y(t); the GS discards messages of step t that arrive later |

Phase 2 (AS): Ĉ(t) = (1 - alpha) Ĉ(t-1) + alpha c_y y(t-1), or Ĉ(t-1) when y(t-1) did not arrive in phase 1;
K̂(t) = Ĉ(t) (T3 - T_m) / Ts; Q(t) = Q_M(t) + Q_out(t), with Q_M(t) = sum of the backlogs Q_i(t) of the demand
tables of M(t) (Section 5) and Q_out(t) = sum of Qbar_i(t) = Q_i(s_i) + a_i(s_i) over the vessels outside M(t),
s_i the step of the last demand table of vessel i that reached the AS in phase 1 and a_i(s_i) the final grant of
Algorithm 1 to vessel i in s_i (Qbar_i = 0 for a vessel that never reported): a vessel outside M(t) gets no p(t)
and forwards nothing new, so its bytes still queued stay within Qbar_i(t); b(t) = max(0, U K̂(t) - Q(t)) / m(t);
Algorithm 1 over sigma_t: level 2 for a needed topic that fits, 1 for another topic that fits, 0 otherwise. The
decision log and `fleet/as` hold `K_hat` (without U), `cap` = max(0, U K̂(t) - Q(t)) = m(t) b(t), `Q`, `Q_M`,
`Q_out`, `Qi`, `Qbar` and, after a state message of a data bridge (Section 4), `bridge_reset`.

A raw message belongs to step t when it reaches the vessel regulator in the interval
`[start(t-1) + T1 + T2, start(t) + T1 + T2)`, so the vessel regulator forwards no message between
the end of phase 3 of step t-1 and the start of phase 3 of step t.

## 3. Topics

Prefix `boat/b<i>/` with the vessel index i (1..M). All binary fields in network byte order.

| Topic | Broker where published | Publisher | QoS | Payload |
|---|---|---|---|---|
| `boat/b<i>/raw/<topic>` | vessel | publishers | 0 | content of the message (≤ `bound` bytes); no bridge forwards `raw/#` |
| `boat/b<i>/demand` | vessel (control bridge → GS) | DTI program (Q_i written by the vessel regulator) | 0 | `t:u32, L:u8, Q_i:u32, d_1..d_L:u32`; Q_i = backlog of vessel i at the start of phase 1 (Section 5) |
| `boat/b<i>/levels` | GS (control bridge → vessel) | AS | 0 | `t:u32, L:u8, p_1..p_L:u8` (p ∈ {0,1,2}) |
| `boat/b<i>/goals` | GS (control bridge → vessel) | operation control software | 0 | JSON `{"t":…, "lat":…, "lon":…, "speed":…}` |
| `boat/b<i>/tx/<topic>` | vessel (data bridge → GS) | vessel regulator | 0 (levels 2 and 1; level 2 leaves first in phase 3) | `t:u32, s:u16` + content; s = sequence number of the message in its topic and step, from 0; no MQTT acknowledgment crosses the channel; the GS removes repeated copies by (t, s) |
| `fleet/order` | GS | operation control software | 0 | JSON `{"t":…, "order":[names], "needed":[names]}` |
| `fleet/throughput` | GS (from the GS radio) | channel monitor | 0 | JSON `{"t":…, "y":bytes, "Tb":s, "To":s}` |
| `fleet/as` | GS | AS | 0 | JSON `{"t":…, "C_hat":…, "K_hat":…, "cap":…, "b":…, "U":…, "alpha":…, "m":…, "p":[…], "M":[…], "order":[…], "needed":[…], "a":{…}, "A":…, "Q":…, "Q_M":…, "Q_out":…, "Qi":{…}, "Qbar":{…}}` (topic indices from 1) |
| `boat/b<i>/regulator` | vessel (control bridge → GS) | vessel regulator | 0 | JSON log of step t, published at start(t+1) + 1 ms (phase 1): `{"t", "have_d", "have_p", "late_us", "outq", "Q", "o", "bq", "n", "p":[…], "d":[…], "rx":[…], "fwd":[…], "held":[…], "drop":[…], "x":[…]}`; per topic: raw messages of the step, forwarded, withheld, dropped over the queue capacity, forwarded bytes with the step header; `late_us` = delay of phase 3 after start(t) + T1 + T2; `outq` = unsent bytes of the data bridge socket at that instant (-1 unknown); `Q`, `o`, `bq`, `n` = Q_i(t) of the demand table and the o_i(t), its part in the output queue of the broker (-1 unknown) and n_i(t) behind it (-1: no data-bridge socket; `Q` null without a table) |

The video publisher splits each JPEG frame into messages of at most `bound` bytes; the content of
each message starts with an 8-byte header `frame:u32, k:u16, n:u16` (index of the frame, index of the
message in the frame, number of messages of the frame).

## 4. Bridges (configured on the GS broker, R5)

| Bridge | Address on the vessel | Topics | QoS |
|---|---|---|---|
| `b<i>-data` | vessel:1884 (dedicated listener) | `boat/b<i>/tx/# in 0` | 0 |
| `b<i>-ctl` | vessel:1883 | `boat/b<i>/demand in 0`, `boat/b<i>/regulator in 0`, `boat/b<i>/levels out 0`, `boat/b<i>/goals out 0` | 0 |

Both bridges use `cleansession true`, `bridge_max_packet_size` unset, `max_inflight_messages 0`
(R9). `python -m fleetdt.gsconf` writes both configurations (the data bridges and the
control bridges subscribe with QoS 0; `--data-qos 2` keeps the QoS 2 data bridge of the as2 stack of SPEC_v2.md). Patch
`mosquitto/patches/0002-bridge-apply-set_tcp_nodelay.patch` applies `set_tcp_nodelay true` to the
bridge sockets, and patch `0001-plugin_tick-keep-pending-deadline.patch` keeps the deadline of a
plugin tick when another event wakes the broker, so phase 3 starts within 1 ms (R8). Patch
`0004-plugin-client-out-queued-bytes.patch` adds the plugin accessor `mosquitto_client_out_queued_bytes`
(bytes of the output queue of a client not yet written to its socket) for o_i of the backlog (Section 5).

The AS subscribes to the state of the data bridges (`fleet/bridge/b<i>-data/state` of gsconf, the default
`$SYS/broker/connection/b<i>-data/state` as well; payload 0 or 1). A live state message (not the retained one
of the subscription) means a disconnection or a reconnection: the new TCP connection (clean session) starts
with an empty send queue, so the AS clears the book entry of vessel i (Qbar_i = 0 until the next demand table of
i) and logs `bridge_reset` in the next decision. Only `b<i>-data` counts: the methods with several data bridges
per vessel (B2 per access category, B9 per uplink topic) run without the AS.

Every MQTT client of Fleet-DT sets TCP_NODELAY on its socket (paho-mqtt leaves Nagle's algorithm
on): without it, a small PUBLISH that follows another unacknowledged segment of the same client waits for the
delayed ACK of the broker, about 40 ms on Linux, and the priority table misses the end of phase 2.

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

Backlog. The demand table carries Q_i(t) = ceil(c_b o_i(t) + c_k n_i(t)), computed at the start of phase 1.
o_i(t) bounds from above the bytes of the data-bridge connection that the channel has not carried yet:
o_i(t) = SIOCOUTQ of the TCP socket of the data bridge (bytes not yet sent or not yet acknowledged) + the bytes
that the vessel broker still holds in its output queue for the data-bridge client and has not written to the
socket (the remaining `to_process` of every queued packet, the unwritten tail of a partly written packet
included; accessor `mosquitto_client_out_queued_bytes` of patch 0004). SIOCOUTQ alone misses these bytes, and
they exist exactly when the socket is full, which is the backlog case. n_i(t) = forwarded MQTT PUBLISH packets
with at least one byte among those o_i(t) bytes. Q_i(t) = 0 when o_i = 0 and when the socket cannot be read
(no data bridge, SIOCOUTQ unavailable). The socket and the queue belong to the vessel broker, so the vessel
regulator (the broker plugin, which finds the socket by its local port, `vessel/regulator/fdt_sock.c`)
computes Q_i(t). It keeps a ring (FIFO, 16384 packets per socket) of the byte lengths of the PUBLISH packets
that the broker hands to the data-bridge client, in order (MOSQ_EVT_MESSAGE_OUT, which comes before the broker
queues the packet; the length of the MQTT 3.1.1 packet); at the start of phase 1 it walks the ring from the
newest packet back, summing the lengths until the sum reaches o_i(t), and n_i(t) is the number of packets
visited. With `plugin_opt_demand external` the DTI program writes Q_i = 0 and the plugin overwrites the 4 bytes in
place when the table enters the vessel broker (MOSQ_EVT_MESSAGE_IN, before the broker routes it to the control
bridge), at start(t) + offset; with `internal` the regulator writes Q_i(t) when it builds the table. The step
log records `Q`, `o`, `bq` (the broker part of o) and `n`.

Limits: (1) a broker without patch 0004 has no accessor; the plugin (weak symbol) then logs a warning at start,
writes `bq` = -1 and uses SIOCOUTQ alone, which is not an upper bound when the socket is full. (2) The bytes of
non-PUBLISH packets of the connection (PINGRESP, 2 bytes; CONNACK and SUBACK at the connection) are in o_i but
not in the ring, so the walk charges them to PUBLISH packets and n_i can exceed the exact count by one packet
(the bound stays an upper bound). (3) When the ring holds fewer bytes than o_i (more than 16384 packets
queued), n_i counts every packet of the ring. The alternative without a patch, o_i = bytes handed to the client
since an anchor minus tcpi_bytes_acked, needs the non-PUBLISH bytes that the broker writes (PINGRESP after each
PINGREQ of the bridge client, which sends nothing else under QoS 0) and an anchor taken while the broker queue is
empty; it is not an upper bound in every case, so the regulator does not use it.

The AS takes Q(t) of the vessels of M(t) and of the vessels outside M(t) (phase 2, Section 2), and vertex i of
G(t) keeps Q_i(t), its membership in M(t) and, outside M(t), the bound Qbar_i(t) (`vertices` of the DTA).

## 6. Channel monitor (Section V-B)

In each step: `Tb` = busy time of the channel in the step (survey), `To` = airtime at R_PHY of the
frames that the GS radio receives and of the frames that it sends and the stations acknowledge,
`To = 8 (B_rx + B_tx_acked) / R_PHY + (N_rx + N_tx_acked) * 8 h_W_busy / R_PHY`, with
`h_W_busy = h_W - R_PHY (T_DIFS + T_bo) / 8` (only the busy part of each frame: the survey counts the
DIFS and the backoff as idle time, so `Ts - Tb` holds them already), and
`y(t) = R_PHY (Ts - Tb + To) / 8` bytes, clamped to [0, R_PHY Ts / 8]. With no survey of step t, the
monitor publishes nothing.
