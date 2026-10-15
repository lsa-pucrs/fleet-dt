# Addendum to the frozen protocol (emu/experiments/frozen/FROZEN.md)

The frozen protocol (tag `v2-freeze-20261016`) fixes H1 to H7 and the experiments E1 to E4. This addendum
adds hypotheses and experiments after that tag. Each section below is fixed in a tagged commit before the
first run of its experiment; the paper reports these hypotheses apart from H1 to H7, with the date and
the tag of each section. Nothing in this file changes H1 to H7, their thresholds, their runs, or their
analysis.

## A1. H8 and E5: B6, IPACT with limited service

Fixed on 2026-10-15, tag `v2-addendum-a1`, before the first E5 run (the E1 to E4 campaign was running;
no E3 result entered this section).

Reason: the baselines of E3 come from the authors (B1 to B5) or from the overlay TDMA of Rao and Stoica.
B6 adds the dynamic bandwidth allocation of the literature that Fleet-DT extends: interleaved polling
with limited service (IPACT) [kramer2002ipact], adapted to the data window W(t).

B6 (AS mode `ipact`, `fleetdt/as2/admission.py:ipact`; vessel regulator mode `slotted`):

| Element | B6 |
|---|---|
| Request | the demand of phase 1 (the same message as Fleet-DT) |
| R and held bytes | shared slot of (c_R + c_B) / η(M+1) from w(t), as the slotted variant |
| Maximum window | W_max = (T_W − shared slot) / M for every vessel |
| Grant (limited service) | vessel i receives the units of its demand in the order π_t with the prefix rule while their cost fits in η(2) (W_max − guard_i) |
| Windows | back to back from the end of the shared slot in the rotated order (i − t) mod M; each window lasts its grant / η(2) plus guard_i = 2ε + T_succ(MSS + h_T, R_i) |
| Capacity | η(M+1) and η(2) of the DCF, without the busy time of the channel, the prediction, or ACI |

E5: B6 on the cells of E3: S1, S2, S4, S5; ρ ∈ {1, 1.5, 2}; M = 4; Ts = 125 ms; the 12 evaluation seeds;
144 runs. `matrix.py b6` writes the run files; every E5 run file equals the E3 run file of the same
cell and seed in the DTP, the scenario, the sources, the ρ setting, and the schedule of the operation
control software (checked by `emu/experiments/addendum/check_e5.py`), so the E5 runs pair with the E3
runs of the same seed. The E5 runs start after the last E1 to E4 run, on the same host, with the same K = 3,
the same lag rule, and the code of tag `v2-addendum-a1`.

| | Hypothesis and threshold | Exp. | Evaluation (`emu/analysis/report.py`, `h8`) |
|---|---|---|---|
| H8 | For ρ ≥ 1.5, Fleet-DT delivers within their steps a larger fraction of the bytes of the needed units, and misses fewer states, than B6. | E3, E5 | ρ ∈ {1.5, 2}, S1, S2, S4, S5: Fleet-DT (E3) against B6 (E5), paired by seed; D_N larger and state miss ratio smaller; Wilcoxon two-sided with Holm over the 8 comparisons of each metric, α = 0.05; Hodges–Lehmann estimate with 95 % interval |

Additional analysis (not a hypothesis): the slotted variant (E3) against B6 with the same test, because
both use the same slot structure and differ in the division of the data window among the vessels.

## A2. H9 and E6: larger fleets in simulation

Fixed on 2026-10-15, tag `v2-addendum-a2`, before the first E6a, E6b and E6d run.

Reason: the emulation host runs the emulator of M ≤ 4 vessels in real time (Section VI); the scale study
runs the same software components in a discrete-event simulation that does not need real time.

### Simulator (`emu/sim`)

| Component | Simulation | Origin |
|---|---|---|
| Channel, Wi-Fi nodes, AP monitor, ground truth, external BSS, jammer | `fdt-sim.cc` | the code of `emu/ns3/fdt-emu.cc`, unchanged |
| Vessel regulator | `fdt_reg.c` linked into `fdt-sim` | the C core of the Mosquitto plugin, unchanged |
| AS | `fleetdt/as2/core.py` (ASCore) driven by `emu/sim/simrun.py` | unchanged |
| Sources, goals, orders | `fdt_source.py` and `opctl.py` functions in `simrun.py` | the draws of the emulation runs of the same seed |
| MQTT bridges | codec in `fdt-sim.cc`: PUBLISH QoS 0, 1, 2, PUBACK, PUBREC, PUBREL, PUBCOMP; QoS 2 delivered at PUBREL | model of Mosquitto 2.1.2 (`src/handle_publish.c`, `src/database.c`) |
| TCP and IP | ns-3: CUBIC, SACK, timestamps, MSS 1448, no Nagle, minimum RTO 200 ms, delayed ACK every segment (`--delAckCount=1`) or after 40 ms | model of Linux |
| Loopback hops | fixed: source → vessel broker 50 µs; GS broker → AS 100 µs (QoS 0, 1) and 300 µs (QoS 2); AS start 4.4 ms after start(t) + T1; AS → bridge 100 µs plus the measured compute time of ASCore | E1 timing (publication − compute = 4.6 ms) |
| Clocks | no offset | |
| Clock synchronization traffic | absent | |

`simrun.py` writes a run directory in the layout of the emulation, so `emu/analysis/gtstep.py`,
`metrics.py` and `report.py` analyze it unchanged.

Inputs inspected before this section (all other E1 to E4 results unseen): the frame statistics and the
metrics of two E3 runs (Fleet-DT, S2, ρ = 1.5, seeds 1004 and 1009), which set the delayed ACK of the
simulator (ACK ratio 0.82 against 0.77 to 0.85 in the emulation); the metrics of three E3 runs of B1 (S2,
ρ = 1.5, seeds 1001, 1007, 1009); the AS timing of two E1 runs (S3 seed 1003, S4 seed 1010); the emulator
lag of the campaign runs. No comparison of Fleet-DT with a baseline (H5, H8) was computed.

### E6c: saturation capacity (calibration, done before this section)

B1 at saturation, S1 and S2, M ∈ {4, 8, 16, 32}, seeds 9101 and 9102, 600 steps (`scale.py sat`,
`scale.py calib` → `calibration-sim.json`, key `sim_sat`). κ_sat: S1 0.807, 0.713, 0.590, 0.433; S2 0.726,
0.667, 0.567, 0.420 for M = 4, 8, 16, 32.

### Pilots (seed 7001, outside the evaluation seeds; design only)

| Pilot | Finding | Consequence |
|---|---|---|
| Fleet-DT and B1, S2, M ∈ {8, 16, 32}, phases of Table param | M = 8 works (f̄ 0.019, state misses 0); at M = 16 and 32 most tables reach their vessel after v(t) (T2 = 10 ms) | T2 grows with M |
| Fleet-DT, S2, M ∈ {8, 16, 32}, T1 = 40 ms, T2 = 30 ms | tables arrive at most 11.9 ms (M = 8) and 18.8 ms (M = 16) after start(t) + T1 at the 0.1 % quantile; at M = 32, 17 % of the demands and 0.7 % of the tables arrive late (AC_VO contention of 33 stations with CWmax 7, TCP retransmission timeouts) | T2(M) = 10 ms + 0.9 ms (M − 4)⁺, T1(M) = 20 ms + 1.25 ms (M − 16)⁺; M = 32 descriptive only |
| sources | with the sources of M = 4 at ρ = 1.5, the telemetry alone needs 4.2 ms of airtime per vessel and step; at M = 32 it exceeds 1.5 κ_sat T_W | the scale study keeps the sources per vessel and reports ρ_eff |

### Experiments

| Exp | Factors | Runs |
|---|---|---|
| E6a (validation) | the E3 run files of Fleet-DT, B1, B3, TDMA and the E5 run files of B6; S1, S2; ρ = 1.5; M = 4; 12 seeds | 120 |
| E6b (scale) | Fleet-DT, B1, B3, B6, TDMA; S1, S2; M ∈ {8, 16}; phases T1(M), T2(M); every vessel with the sources of the M = 4 runs at ρ = 1.5 (ρ_eff 3.2 at M = 8 and 7.5 at M = 16 in S2); 12 seeds | 240 |
| E6d (descriptive) | Fleet-DT and B1; S2; M = 32; T1 = 40 ms, T2 = 35.2 ms; the same sources; 12 seeds | 24 |

All E6 runs use `simcampaign.py` with `--delAckCount=1`, pinned to CPUs 14 and 15 (the CPUs that the
emulation campaign leaves free).

### Validation (gate of H9; `validate_sim.py`)

Per cell (method, scenario) of E6a against the emulated E3 and E5 runs of the same seeds, on the medians
over the 12 seeds: |Δ D_N| ≤ 0.05; |Δ state miss ratio| ≤ 0.01; |Δ mean κ(t)| ≤ 0.05; for Fleet-DT and B6
also |Δ f̄| ≤ 0.02 and |Δ ν| ≤ 0.05; the sign of the median paired difference (Fleet-DT − baseline) in D_N
and in the state miss ratio agrees between simulation and emulation wherever the emulated difference
exceeds 0.01 (D_N) or 0.002 (state miss ratio) in absolute value; and |Δ κ_sat| ≤ 0.03 for S1-m4, S2-m4
and S2-m8 (S2-m8 against the emulated saturation runs of the frozen calibration). H9 is tested only when
every check holds; otherwise the paper reports E6b descriptively with the failed checks. The thresholds
were written into `validate_sim.py` while E6c ran, before its κ_sat values existed; the κ_sat check
already holds (S1-m4 0.807 against 0.779, S2-m4 0.726 against 0.710, S2-m8 0.667 against 0.689).

| | Hypothesis and threshold | Exp. | Evaluation (`report.py`, `h9`) |
|---|---|---|---|
| H9 | With M ∈ {8, 16} vessels and the sources of the M = 4 runs on every vessel, Fleet-DT delivers within their steps a larger fraction of the bytes of the needed units, and misses fewer states, than B1, B3, B6, and TDMA; and f̄ ≤ β + 0.02 in every Fleet-DT run with F_R(T) = 0. | E6b | S1, S2, M ∈ {8, 16}: Fleet-DT against each baseline, paired by seed; Wilcoxon two-sided with Holm over the 16 comparisons of each metric, α = 0.05; f̄ in every Fleet-DT run with F_R(T) = 0 |

E6d enters the paper as a description of the limit of the configuration (Ts = 125 ms, 24 Mb/s, control
bridge in AC_VO) without a hypothesis.
