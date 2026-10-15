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
