# Frozen inputs of the evaluation campaign (E1 to E4)

Frozen on 2026-10-15 before the first evaluation run. The evaluation campaign runs the manifest of this
directory with the code of the tagged commit that holds this directory; `SHA256SUMS` lists the SHA-256 of
every file of the directory except this one, and `traces.sha256` the SHA-256 of the frame-size traces of the
sources (`emu/traces/`).

## Files

| File | Content |
|---|---|
| `calibration.json` | saturation runs ("sat": kappa_sat and mean PHY rates per scenario and M) and the E0 calibration ("e0": lambda, ES gain, c_S, gamma and its replay table, cal_kappa, b_hat0, z_b4, the initial scores) |
| `init/<S>-<predictor>.json` | the initial scores of the AS per scenario and predictor (eq, persist, es, calmean) with b_hat(0); the run files refer to them as `init/<S>-<predictor>.json` of the campaign directory |
| `manifest.json` | the eval manifest: 1524 runs (E1 60, E2 240, E3 1152, E4 72) in the randomized order of order seed 20261015, status "pending" |
| `specs.tar.gz` | the run files of the manifest (`specs/<run>.json`: DTP, scenario of the seed, sources, rho setting, AS options, PHY model); deterministic archive |
| `whatif/S2-m8-ts125.json`, `whatif/S2-m4-ts250.json` | the DTPs of the what-if configurations of H6 without a measured counterpart |
| `seeds.json` | evaluation seeds 1001-1012, calibration seeds 9001-9004, saturation seeds 9101-9102, order seed |
| `dtp_base.json` | the DTP template of `emu/experiments/matrix.py` (Table param) |
| `scenarios/<S>-m<M>.json` | the reference scenario files (seed 1) of the cells; every run file holds the scenario of its own seed (`emu/scenarios/gen.py`) |
| `traces.sha256` | SHA-256 of `emu/traces/*.txt`; the sources use `water-sbs-q{50,70,90}-s0p5.txt` |
| `SHA256SUMS` | SHA-256 of the files above |

## Hypotheses and thresholds (Table hyp of Section VI, sec6_method.tex)

| | Hypothesis and threshold (text of the paper) | Exp. | Evaluation (`emu/analysis/report.py`) |
|---|---|---|---|
| H1 | After the convergence of the clocks, \|θ_i(t)\| ≤ ε in every probe sample of every vessel, and the demands miss phase 1 in at most 10^-3 of the pairs (t, i) of the steps without external interference, pooled over the runs of each scenario. | all | every run: no probe sample of a vessel with \|offset\| > ε (samples with a probe round trip ≤ 500 µs, after the warm-up); per scenario, the pooled fraction over the runs of the scenario of the pairs (t, i) without the demand in phase 1, in the steps without interference (a frame of the external BSS or a jammer burst), ≤ 10^-3 |
| H2 | The values of κ(t) and b(t) that the AS computes from the monitor differ from the ground truth by at most 0.01 at the 99th percentile; every step with X(t) ≤ κ(t) T_W in the ground truth has f(t) = 0 unless a table or a report is lost. | E1 | per E1 run: p99 of \|κ_AS − κ\| and of \|b_AS − b\| ≤ 0.01; no step with X(t) ≤ κ(t) T_W, "empty" data bridges (gtstep.py) and f(t) = 1 without a lost table or report |
| H3 (RQ.II) | On the evaluation runs, the prediction (pred) has a lower mean absolute error of κ(t) than persistence (κ̂(t) = κ(t−1)), exponential smoothing of κ, and the mean of the calibration runs, and, with ACI at the same β, a larger utilization ν. | E2 | per scenario, runs paired by seed: eq. (pred) against each other predictor; MAE smaller and ν larger; Wilcoxon signed-rank, two-sided, Holm per metric over 15 comparisons, α = 0.05; Hodges–Lehmann estimate with 95 % interval |
| H4 (RQ.I) | Inequality (thm) holds at every prefix of every run, and f̄ ≤ β + 0.02 in every run with F_R(T) = 0. | E2, E3 | E2 runs and the ACI methods of E3 (Fleet-DT, slotted, B5): (thm) at every prefix; f̄ ≤ β + 0.02 (β = 0.05) in every run with F_R(T) = 0 |
| H5 (RQ.III) | For ρ ≥ 1.5, Fleet-DT delivers within their steps a larger fraction of the bytes of the needed units, and misses fewer states, than B1, B2, B3, B5, and the overlay TDMA. | E3 | E3, ρ ∈ {1.5, 2}, S1, S2, S4, S5: Fleet-DT against B1, B2, B3, B5, TDMA, paired by seed; D_N larger and state miss ratio smaller; Wilcoxon two-sided with Holm over the 40 comparisons of each metric, α = 0.05 |
| H6 | The what-if evaluation, from the DTA of E1 (M = 4, T_s = 125 ms), predicts f̄ within 0.02 and ν within 0.05 for the configurations of E4. | E4 | the what-if evaluation (fleetdt/as2/whatif.py) of the DTA of the E1 run of the same seed (S2, M = 4, Ts = 125 ms) against each measured Fleet-DT run of E4 (M = 2, Ts = 125 ms; M = 4, Ts = 62.5 ms): \|Δf̄\| ≤ 0.02 and \|Δν\| ≤ 0.05; M = 8 (Ts 125 ms) and M = 4 (Ts 250 ms) reported as predictions without a measured counterpart (`--whatif`) |
| H7 | The delay and the age of information of the states stay within the bounds of Corollary (state) in every step that meets the hypothesis of Lemma (delivery). | E4 | E4 Fleet-DT runs: no state delay above Ts + T3 + ε and no age above 2 Ts + T3 + ε in the steps that meet the hypothesis of Lemma 2 (two consecutive Lemma steps for the age) |

H2, H4, and H7 hold for every step or every run, so one counterexample rejects them. Each run discards
its first 240 steps; the analysis discards the steps with a real-time lag of the emulator above 1 ms and
reports their number. The interval of f̄ within a run uses batch means over 40 batches of 24 steps.

## Experiments (Table exp) as frozen

| Exp | Factors | Runs |
|---|---|---|
| E1 | Fleet-DT; S1 to S5; ρ = 1.5; M = 4 | 60 |
| E2 | Fleet-DT with eq. (pred), persistence, ES of κ, calibration mean; S1 to S5; ρ = 1.5; M = 4 | 240 |
| E3 | Fleet-DT, slotted, B1, B2, B3, B4, B5, TDMA; S1, S2, S4, S5; ρ ∈ {1, 1.5, 2}; M = 4 | 1152 |
| E4 | Fleet-DT, B1, B3; S2; ρ = 1.5; M = 2 with Ts = 125 ms (SpectrumWifiPhy) and M = 4 with Ts = 62.5 ms (YansWifiPhy); K ≤ 3 | 72 |

E4 keeps the cells that meet the lag rule (at most 1 % of the steps with a lag above 1 ms, emu/README.md):
M = 8 (100 % of the steps above 1 ms with yans at K = 1) and M = 4 with Ts = 250 ms (0.6 % and 1.7 % with
yans at K = 1) have no runs and enter H6 as predictions. The run files of E4 set the PHY model in
`scenario.phy`, and the manifest sets `max_K` per run (runner.py).

Sources: the video topic replays `emu/traces/water-sbs-q<Q>-s0p5.txt` (Q = 50, 70, 90 for ρ = 1, 1.5,
2; Pohang Canal Dataset, CC BY-NC 4.0, emu/traces/README.md) from a start that the run seed fixes. Phase 1:
vessel i publishes its demand at i (T1 − g1)/(M + 1) after start(t), g1 = ε + 2 T_tick (default of the
regulator).

## Calibration values used

From `calibration.json`: the saturation runs of 2026-10-15 (B1, 2 seeds per scenario) and the E0
campaign `e0v4` (60 runs, Fleet-DT, SpectrumWifiPhy, guarded stagger, water traces, K = 3).

| Value | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|
| κ_sat (M = 4) | 0.7793 | 0.7102 | 0.6767 | 0.6008 | 0.5762 |
| c_S (failure model) | 0.7823 | 0.5925 | 0.6500 | 0.6252 | 0.6572 |
| Z of B4 = q_β(z)/c_S | 1.1497 | 1.4096 | 1.0471 | 1.0425 | 0.6556 |
| cal_kappa (calibration mean) | 0.7842 | 0.6951 | 0.6870 | 0.6508 | 0.5103 |
| b̂(0) | 0.0006 | 0.1099 | 0.0749 | 0.0050 | 0.0139 |
| initial scores per predictor | 14112 | 14112 | 14027 | 14111 | 14112 |

κ_sat of S2 with M = 2 and M = 8: 0.7178 and 0.6885; mean PHY rates 24 Mb/s (S5: 49.3 Mb/s up).
λ = 0.75 (replay MAE 0.1246, grid 0.02-1.0); gain of the exponential smoothing of κ (E2) = 0.4 (MAE
0.0477); pooled c = 0.6182. γ = 0.05: the smallest value of {0.001, 0.005, 0.01, 0.02, 0.05} that meets
the rule of Section VI-E (largest replay f̄ 0.0667 ≤ β + 0.02; largest |f̄ − β| 0.0188 ≤ 0.02 over the 48
replays of S2-S5, in which α = 1 yields f̄ ≥ β; S1: β not attainable in its 12 replays).

## Analysis entry points

| Program | Role |
|---|---|
| `emu/analysis/gtstep.py RUN` | ground truth per step (κ, e, b, X, empty, interference, lag) → RUN/gt_steps.jsonl |
| `emu/analysis/metrics.py RUN` | metrics of a run (Definition fidelity, D_N, state miss, μ, AoI, H1, H2, Theorem 1, H7) → RUN/metrics.json |
| `emu/analysis/report.py RESULTS... --out DIR --whatif DIR` | hypotheses H1–H7 (hypotheses.json), runs.csv, LaTeX tables, PDF figures |
| `emu/analysis/stats.py` | Wilcoxon signed-rank, Holm, Hodges–Lehmann, batch means, Wilson |
| `emu/experiments/calibrate.py sat / e0` | saturation and E0 calibration (the values above) |
| `emu/experiments/runner.py` | executes a manifest (K sessions, lag rule, retry, max_K, post-processing with gtstep.py and metrics.py) |

## Commands

On the emulation host, in `~/fleetdt-deploy/emu/repo` (the tagged commit), with
`C=~/fleetdt-deploy/emu/campaigns` and `R=~/fleetdt-deploy/emu/results`:

```sh
sha256sum -c emu/experiments/frozen/SHA256SUMS --quiet          # (from emu/experiments/frozen/)
setsid nohup python3 emu/experiments/runner.py $C/eval --K 3 --retry 1 > $C/eval/runner.log 2>&1 < /dev/null &
python3 emu/analysis/report.py $R/eval --out $R/eval/report --whatif $C/eval/whatif
```

The campaign directory `$C/eval` holds the same manifest, run files, initial scores and what-if files as
this directory (written by `python3 emu/experiments/matrix.py eval --calib $C/calibration.json --out $C/eval` and
copied here by `python3 emu/experiments/freeze.py --campaign $C/eval --calib $C/calibration.json`). `touch $C/eval/STOP` stops the slots after their current runs; the runner
resumes from the manifest.
