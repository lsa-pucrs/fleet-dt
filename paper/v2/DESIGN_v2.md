# Fleet-DT v2: design after the simulated IEEE Access review

Scope: communication and networks. The autopilot, the detector at the GS and the sensors appear only as
sources and consumers with a rate, a size bound and a requirement.

## 1. Changes against v1 and the objection each change removes

| # | v1 | v2 | Objection (reviewer) |
|---|---|---|---|
| 1 | Capacity C(t): minimum over sets of saturated radios (counterfactual); measurement y(t) = free airtime at R_PHY | Capacity κ(t): fraction of the data window W(t) in which the channel carries successful exchanges of the fleet while the vessels with admitted data stay backlogged. The emulator measures κ(t) directly; the AS predicts it with a DCF model | R1 W2, W3; R2 M3; R3 W2 |
| 2 | Exponential smoothing of y(t), α by MSE, U tuned on the DTA | Point predictor κ̂(t) = (1 − b̂x(t)) e(m(t)+1) (Bianchi efficiency e, external busy fraction bx smoothed); lower bound by adaptive conformal inference (ACI, Gibbs and Candès 2021) on the delivery event | R1 W8; R2 M4; R3 W12 |
| 3 | Bytes at fixed R_PHY, NOHT, fixed rate | Airtime (s). Each vessel has its own rate R_i(t) (AP station table). ACK at the control response rate. Minstrel allowed | R1 W3a, W10a, W11d; R2 M8 |
| 4 | Eq. 5 with mean backoff, h_A undefined | Airtime of the successful exchange T_s(s, R) (no backoff); backoff idle and collisions enter e(m). Cost of n messages with x bytes in segments and ACK frames; h_A defined | R1 W4; R2 M6; R3 W7 |
| 5 | Equal grant per vessel, one table for the fleet | Admission units (i, ℓ, k); needed sets N_i(t) per vessel; one aggregate airtime constraint; order σ_t over units (topic rank first, vessels interleaved and rotated per step) | R1 W6; R2 M7; R3 W8 |
| 6 | Levels map to QoS 2/1 | DTP marks the mandatory set R (QoS 2, application requirement, always admitted first). Other topics QoS 0 with (t, seq). Level = order and admission only | R1 W5; R2 M5; R3 m9 |
| 7 | Demand from rates and bounds (constant) | Demand = bytes and messages already held for step t at the start of phase 1 + arrival curve of each topic over the rest of the collection window | R1 W9; R2 M7; R3 W7 |
| 8 | Message-level policing | Unit-level policing: a frame for an output topic (video), the step stream for a stream topic. A consumer that needs whole frames (detector) receives whole frames or nothing | R2 M8; R3 W7 |
| 9 | Empty send buffers as precondition; Corollary 1 over all steps since 0 | Each vessel reports the unacknowledged bytes and messages B_i(t) of its data bridge (SIOCOUTQ and message offsets); the AS charges their airtime to the grant. TCP_NOTSENT_LOWAT keeps unsent bytes discardable. Recovery in one step | R1 W7; R2 M6; R3 W4, W5 |
| 10 | Phase 3 ends at the GS step end; no guard | Data window W(t) on the GS clock: [start + T1 + T2 + ε, start + Ts − g); ε bounds the clock offset, g bounds the receive path and the QoS 2 completion | R3 W3 |
| 11 | Headroom 1 − U for non-bridge traffic | Explicit reservation O (measured bound of chrony, keepalive, ARP, goals in W(t)) | R3 W6 |
| 12 | Control bridge shares the radio FIFO | Control bridge marked DSCP EF → WMM AC_VO; data bridge AC_BE | R1 W7d; R2 M6; R3 W4 |
| 13 | RTO min 200 ms | Route option rto_min ≤ T3/4; RACK-TLP; residual loss counted by the guarantee | R1 W7a; R2 M6 |
| 14 | No p(t) → forward nothing | No p(t) → default table: R only | R2 M8 |
| 15 | Fidelity: MAPE, γ (with y), δ | Fidelity: calibration of the failure rate, pinball loss of κ̂ on backlogged steps, per-edge admitted vs. received, utilization ν, age of information of the state topic | R2 M4 |
| 16 | No TDMA comparison | TDMA variant: phase-3 slots sized by the admitted airtime (egress gate); overlay TDMA as baseline | R1 W12; R2 M9 |
| 17 | Ts = 125 ms unjustified | Ts equals the period of the state source (8 Hz); overhead (T1 + T2 + ε + g)/Ts and the AoI bound as functions of Ts; sweep Ts ∈ {62.5, 125, 250} ms | R1 W9; R3 m17 |
| 18 | Field + HIL bench | Software only: ns-3 real-time emulation with one QEMU VM per host (unchanged software), plus mac80211_hwsim + wmediumd; ground truth from the emulator | user constraint |

## 2. Model

### 2.1 Network
- Vessels i ∈ {1..M}, GS = 0. One BSS; AP at the GS radio; each vessel radio is a station.
- Topics ℓ ∈ {1..L}, the same for every vessel. DTP fields per topic: rate r_ℓ (outputs/s), size bound s_ℓ (bytes per message),
  jitter J_ℓ, kind ∈ {stream, output} (output: a consumer needs the whole output, e.g. a frame), messages per output bound n_ℓ.
- Mandatory set R ⊆ {1..L} (DTP): the GS needs R from every vessel in every step. QoS q_ℓ = 2 for ℓ ∈ R, 0 otherwise.

### 2.2 Step, phases, window
- Step t = [t0 + t Ts, t0 + (t+1) Ts) of the GS clock. Phases T1 (request), T2 (grant), T3 (data).
- Assumption 1: |θ_i(t)| ≤ ε for every vessel i and step t (offset of the vessel clock to the GS clock).
- Data window W(t) = [t0 + tTs + T1 + T2 + ε, t0 + (t+1)Ts − g) on the GS clock, length T_W = T3 − ε − g.
- Collection window of step t on vessel i: messages that reach the regulator in [start3(t−1), start3(t)) of the vessel clock.

### 2.3 Airtime
- Successful exchange of an IP packet of s bytes at rate R (Bianchi T_s):
  T_s(s, R) = T_PHY + 8(h_MAC + s)/R + SIFS + T_ACK(R) + DIFS, with T_ACK at the control response rate of R.
- Cost of n messages of one topic with x content bytes on vessel i with rate R_i:
  segments ≤ n + ⌈(x + n h_P)/MSS⌉; each segment is a frame of ≤ MSS + h_T bytes; each segment draws ≤ 1 TCP ACK frame of h_T bytes at the AP rate.
  a_i(x, n) = (n + (x + n h_P)/MSS) [T_s(MSS + h_T, R_i) + T_s(h_T, R_0)] (upper bound with full-size segments) — tighter form in code: per-message segment sizes.
  QoS 2 adds per message 3 control segments of h_A = 4 + h_T bytes and their ACKs.
- External busy fraction bx(t): busy time of the channel not caused by the fleet, measured by the AP monitor in phases 1–2 (fleet sends only the small control tables, whose airtime the AS knows).

### 2.4 Capacity
- κ(t) ∈ [0, 1]: fraction of W(t) in which the channel carries successful exchanges of the fleet (data frames and their TCP ACKs, both directions)
  when every vessel with admitted data keeps a packet in its radio queue during W(t). Measured: emulator ground truth, and on saturated steps at the AP.
- e(m): Bianchi DCF efficiency (fraction of time in successful transmissions) with m saturated contenders, PHY timing and mean frame of the DTP.
- Point prediction: κ̂(t) = (1 − b̂x(t)) e(m(t) + 1), b̂x by exponential smoothing with gain λ of bx(t−1) (phases 1–2 samples) — any predictor works for the theorem.

### 2.5 Demand and backlog (phase 1)
- Demand of unit u = (i, ℓ, k): (x_u, n_u) = held bytes/messages + arrival-curve bound for the remainder of the collection window
  (r_ℓ(T_rem + J_ℓ) outputs of ≤ n_ℓ messages of ≤ s_ℓ bytes).
- Backlog B_i(t): unacknowledged bytes of the data-bridge socket (SIOCOUTQ) and the number of unacknowledged messages, at the start of phase 1.
  Upper bound of the backlog at the start of W(t), because the regulator writes nothing between the end of W(t−1) and the start of W(t).
- Report of step t−1: forwarded units of step t−1 (for err_{t−1}).

### 2.6 Admission (phase 2)
- Grant G(t) = T_W · κ̂(t) · s_t(α_t) − Σ_i a_i(B_i(t)) − O, where s_t(α) is the lower α-quantile of the ratio scores κ_obs/κ̂ on the saturated steps of the DTA and of the operation,
  s_t(α) = 0 for α ≤ 0, s_t(α) = max score for α ≥ 1. s_t is nondecreasing in α.
- Units of R are admitted first and always. Then greedy in σ_t over the remaining units: admit u if the cumulative cost ≤ G(t).
- Levels per vessel: p_{i,ℓ} = 2 for admitted ℓ ∈ N_i(t) (R ⊆ N_i(t)), 1 for other admitted, 0 otherwise; video: number of admitted frames k_i(t).
- ACI update at phase 2 of step t+1: α_{t+1} = α_t + γ(β − err_t), err_t = 1 if some admitted unit of step t missed the GS within step t.

### 2.7 Results
- Lemma 1 (greedy = lexicographic maximum over the downward-closed family of feasible sets with R forced).
- Lemma 2 (mechanism): under Assumption 1 and work-conserving service, if the airtime of the admitted units + backlog + O ≤ κ(t) T_W, every admitted unit reaches the GS within step t and B_i(t+1) = 0.
- Theorem 1 (any channel, any predictor): (1/T) Σ err_t ≤ β + α_1/(γT) + (1 − β)(1 + F_R(T))/T, where F_R(T) counts the steps with α_t ≤ 0 and err_t = 1 (the channel did not carry R alone).
- Corollary 1: one state per vessel per step on the steps of Lemma 2; AoI of the state ≤ Ts + T_W + g.

### 2.8 Fidelity
- Calibration: realized failure fraction vs β; pinball loss of κ̂ on saturated steps; per-edge ratio received/admitted; utilization ν = admitted airtime / (κ(t) T_W); AoI of the state.

## 3. Platform requirements v2 (Table III)

| R | Requirement | Serves |
|---|---|---|
| R1 | Vessel broker plugin: receives each message, republishes, wakes at instants, queries the bridge socket (SIOCOUTQ) and sets TCP_NOTSENT_LOWAT | Def. regulator, backlog |
| R2 | Each topic declares rate, size bound, jitter, kind and QoS in the DTP | demand, policing |
| R3 | DTI program publishes demand, backlog and the report of step t−1 in phase 1 | phase 1 |
| R4 | Clocks follow the GS clock with |θ| ≤ ε (chrony, slew only) | Assumption 1 |
| R5 | One data bridge (DSCP AF/BE) and one control bridge (DSCP EF) per vessel | WMM |
| R6 | The AP radio reports per window: busy time, frames and bytes per station, rate per station | bx, κ, R_i |
| R7 | WMM on; no A-MPDU (legacy or HT without aggregation); rate adaptation allowed | airtime model |
| R8 | TCP_NODELAY on both ends, quickack, no slow start after idle, rto_min ≤ T3/4, NOTSENT_LOWAT | cost bound, recovery |
| R9 | No limit on in-flight messages in the brokers | regulator |

## 4. Emulation (Section VI)
- Backend A: ns-3 (Wi-Fi 802.11g/n, YansWifiPhy or SpectrumWifiPhy, ConstantRate or Minstrel), real-time scheduler, TapBridge to one QEMU VM per host; per-link loss matrix or positions with TwoRay + Nakagami; interferers: CSMA BSS (OnOff) and a non-CSMA waveform generator; ground truth: κ(t), external busy, offsets (VM clocks vs host clock).
- Backend B: mac80211_hwsim + wmediumd (patched), real nl80211 survey and fdt_chmon.
- Validity checks: ns-3 saturation throughput vs Bianchi for N = 1..16; backend A vs B on shared scenarios; scheduler lag logged, runs above the lag limit discarded.
