# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- outer-loop variants O1 to O5 of Table S4 (`fleetdt/outer_variants.py`, `fleetdt.as_service --outer-variant`):
  fixed (alpha_0, U_0), fixed (alpha*, U*) of a calibration file, recomputation from the DTA of each window of
  N_w = 480 steps, coverage feedback on U (+/- 0.05 per window, clamped to [0.30, 1.2]), and the regime oracle;
  the calibration file format `fleetdt-calibration/1` (alpha*, U* per M and scenario); per-step `variant`,
  `cover` and `regime` in the AS decision records; experiment field `outer.variant` and its parameters in
  `emu/fleet.py`, with the regimes of the scenario JSON
- `fleetdt.outer.run_job` and `python -m fleetdt.outer --steps T1:T2`: the outer loop over a range of steps
- emulation factors of the experiments of Section VI (emu/README.md, experiment fields): E0 outages
  (`"outage"`, `gen.with_outage`: 200 dB between the AP and vessel 1 for 1, 3 or 5 s from a seeded start
  after the warm-up; timed entries `[a, b, loss, t0, t1]` of `channel.loss` in fdt-emu), the E3 ablations
  (`"ablation"`: R7 Minstrel over the OFDM rates of 802.11a, `gen.with_rate_adaptation`; R8
  `set_tcp_nodelay false` and routes without quickack, `fleetdt.gsconf --nagle`, `fdt-netsetup.py
  --tcp-only`; R9 `max_inflight_messages 20`, `fleetdt.gsconf --max-inflight`), the E4 overload of the
  video source of vessel 1 (`"overload"`, `fdt_source.py --overload-*`, `video_layers.overload_steps`), the
  E4 fairness cells (`"vessels"`: per-vessel size_scale, fps_scale, video on or off and topics; a vessel
  without video has rate 0 on its layer topics, so its demand table declares none), and the regime
  sequence S1, S3, S1 of E5 (`gen.make_paper_sequence`, `"regimes"` in the scenario)
- `"session"` of a paper-stack experiment (CPU quota, clock injector, probe period), checked against the
  session that is up; `run.py up --probe-period`; `qualify.py manifest --bare-session`
- calibration Cal of Section VI-E: `emu/experiments/calibrate_paper.py compute` writes calibration.json with T1,
  T2, eps, delta_GS, T_m and B_ctl_bar per M, the global c_y, C_ref per (M, scenario) and alpha*, U* per (M,
  left-out scenario) from the logs of the calibration runs (`fleetdt/calibration.py`); `calibrate_paper.py dtp`
  writes these constants into the DTP of an evaluation run
- AS log: `d_at` (arrival of each demand table, seconds after start(t)) in each decision and `demand_late` lines
  for the tables after start(t) + T1 (w1(t))
- regulator step log: `p_rx_us` and `p_late_us`, the arrival of p(t) on the vessel clock (w2(t))
- `emu/tools/chrony_sampler.py`: the error bound of chrony (|System time| + root dispersion + root delay / 2)
  once per step on every host of a run (`chrony.jsonl`)
- GS broker plugin `fdt_gsstamp.so` (`mosquitto/plugins/`, `fleetdt.gsconf gs --stamp`): the delivery time of
  each uplink message of the data bridges, per step (`gsdelivery.jsonl`), and its C unit test
- `fleetdt.outer._calibrate(refine=False)`: alpha* on the grid 0.01..1 only
- hypothesis-level driver `emu/analysis/hypotheses.py`: every component H1a to H6b of the Table of hypotheses
  from the per-run metrics of a campaign (paired exact signed-rank tests with Pratt zeros on block differences
  minus the thresholds, Clopper-Pearson limits with n_eff, block-bootstrap bounds, TOST, leave-scenario-out
  alpha* and U*, IUT over cells, fixed sequence H1 to H6), JSON and Markdown table; manifest fields in
  emu/README.md
- `fleetdt/predictors.py`: the predictors of H5 (last value, moving average, Holt, local level by Kalman filter
  with maximum-likelihood variances, TCP 1/8, ARIMA(p, 1, q) by statsmodels or conditional sum of squares, C_ref)
- analysis: `bb_diff_bounds` (block bootstrap of the difference of two independent samples, H3b),
  `logistic_fit` (Newton iterations, H3c), `fixed_sequence_verdicts`, `h3a_pool` and `condition_from_series`,
  per-vessel PDR_state, the per-step series of run_metrics (`series=True`); outer: `dmr_hat_replay`
- campaign generator `emu/experiments/campaign.py` (emu/README.md, "Campaign"): `cal` writes the specs and
  manifests of E0 (26 runs) and Cal (210), `eval --calibration calibration.json` those of E1 (80), E2 (290), E3
  (190), E4 (676) and E5 (60 runs of 390 s, 156 run-equivalents) as Table S6 defines them, with the calibrated
  DTP constants, rho from C_ref through the frame rate and the stripe-size scale, the pilot U of B4, B6 and B7 and
  the outer-loop variants of E5; blocks of one seed and one session slot with the methods or within-block levels
  in a seeded random order, disjoint seed ranges for the calibration and the evaluation, the manifest fields of
  the hypothesis driver; `count` (1773 with Q and the reruns) and `merge` (one manifest for hypotheses.py); every
  spec passes the checks of emu/fleet.py when it is written
- `fleet.paper_check`: the static checks of a paper-stack experiment that `run_fleet_paper` applies before a run,
  without a session (the run calls it)
- calibration: the U of B4, B6 and B7 from the runs of the E4 pilot by the rule of U* with the prediction of each
  baseline (`pilot` of calibration.json, `fleetdt.calibration.pilot`, `pilot_fit`, `lookup_pilot`)
- runner: a run whose `block.slot` is s runs only on slot s mod K (`--no-pin` ignores it); a relative
  `outer.calibration_file` resolves in the manifest directory

### Changed

- calibration and hypothesis driver: the constants, alpha*, U*, C_ref and the predictors of H5 come from the
  Fleet-DT runs of Cal (`calibrate_paper.py compute --fit-methods`); the runs of the other methods of the E4 pilot
  stay out (before: every run under the arguments)

- outer loop: alpha* stays on the grid 0.01..1 of Section VI-E; the golden-section refinement, which can leave
  the grid, runs only with `refine=True` (`--refine`)
- paper campaign defaults: `runner.py` brings the sessions of paper-stack runs up with the CPU quota of 0.25
  of a core, the clock injector (+-20 ppm ramps, NTP loss episodes) and one probe read per step (Ts), and
  the Q specs carry this session; the emulator flags catch-up at 0.1 ms (`--catchupUs=100`,
  `"catchup_us"`); the sources release the telemetry topics with the jitter J = 2 ms of the regulator
- paper scenarios on channel 6 (`"channel_number"`; the emulator ran on the ns-3 default channel 1), and the
  two-ray models of S5 and S7 at 2.437 GHz; the files `emu/scenarios/P-*.json` regenerated
- eps of `fleetdt.analysis.calibrate_margins` adds the largest delay of the vessel regulator at a phase boundary
- tau_int: the automatic window of Madras and Sokal (c = 5) is the default (method "ips" stays an option)
- fixed sequence H1, H2, H3, H4, H5, H6 (HYPOTHESIS_ORDER)
- DMR-hat: backlog recursion with K_y(t) and the final a_i of Algorithm 1 replayed with alpha, U and
  Q(t) = q_hat(t-1) (was: the logged admitted bytes and max(q_hat(t-1), Q(t)))
- H3a (was H4a): condition K_y(t) >= U K_hat(t) >= Q(t) with the stratum Q(t) = 0, no empty-buffer requirement
  (option `require_buf_empty`), catch-up and lag steps of the emulator as failures; `h4a_bound` is an alias of
  `h3a_bound`
- Jain's index over BDR_i of vessels 1 to 3 (`J_all`: every vessel); BDR_i divides by c_l(x, n) of the bytes and
  messages that the sources produced (regulator step log and vessel log), d_il without them
- PDR_state: exactly once after the deduplication, against the messages that the source produced; a duplicate
  that the GS removed no longer fails a pair
- RelMSE of run_metrics against c_y y(t) (was y(t)); alpha* on the grid {0.01, ..., 1} without the golden-section
  refinement (option `refine`)
- PICP_C: the difference PICP - PICP_C with block-bootstrap bounds (H2)
- paper_post: criterion count by default; lag and catch-up steps (0.1 ms rule) stay in the metrics with flags
  instead of leaving them out, `rt_valid`, `steps_over_1ms` (read by runner.py), source counts, PICP_C from
  csub, clock offsets (H1c), per-step series
- fidelity: the mean absolute scaled error (MASE, Hyndman and Koehler 2006) replaces the MAPE; the scale is
  the error of the last-value predictor c_y y(t-1) over the same steps, so smoothing with alpha = 1 gives 1
- control bridges: demand, levels, goals and fleet/feedback with QoS 0 (no MQTT retransmission; clean session),
  the DTI program, the regulator, the AS, the stubs and the subscribers publish and subscribe with QoS 0
- gs.conf and vessel-b1.conf: max_inflight_messages 0 (R9), as the generator already writes
- radios: the wireless path is the PCIe AR9280 of the XM Bullet M2 (pci0000:00/0000:00:00.0), and the
  channel monitor builds with the SDK of the firmware of the radios (ath79-tiny-22.03.7)
- baselines: the method labels follow Table S3 of the paper. B6 is the former B7 (one table per vessel,
  IPACT limited service with excess distribution; AS policy `b6`, `as_model.admission_b6`), B7 the former
  B8 (DCC-like rate control, regulator mode b8), B8 the former B9 (equal slots of phase 3, mode
  paper_tdma), B9 the former B10 (B1 with one data bridge per uplink topic). The method `B10` and the AS
  policy `b7` fail with the name of the new label; experiments written with the former B7 to B9 must change
- B2: the class follows the topic: `state` (without one, the first topic of "order") in EF and AC_VO, the
  other telemetry topics in AF41 and AC_VI, the video stripes in BE and AC_BE (before: the topics of
  "required" in EF); nftables-fdt.nft also sets skb->priority 0:105 and 0:106 on the B2 ports 1885 and 1886,
  so the UP does not depend on the DSCP mapping of the kernel
- fleet feedback of B3, B7 and B8: published once per step t at start(t) + T1 with y(t - 1) (when it reached
  the GS by then), U, m(t) = |M(t)| of the demand tables at the GS (before: a fixed m, the vessels of the
  DTP) and sigma_t; its t field is the step that it serves
- B8 (paper_tdma): the unsent bytes of the data bridge add the bytes that the broker holds for the bridge
  client (bq) to SIOCOUTQNSD, and a released message counts with its PUBLISH packet

## [2.0.0] - 2026-10-14

The repository now implements the network digital twin with dynamic bandwidth allocation of the
IEEE Access paper and replaces the C library of the ICECS 2026 model.

### Added

- vessel regulator (Definition 13) as a Mosquitto 2.1 plugin: step ownership of raw messages, strict
  priority with QoS 2 and QoS 1, policing by c(x, n) = c_b x + c_m n, step log on `boat/b<i>/regulator`
- DTI program `fdt_dti`: demand table at start(t) under SCHED_FIFO, zero demand for idle topics
- AS: eq. (2), eq. (3) and Algorithm 1 on absolute deadlines, `fleet/as` with σ_t and N(t)
- outer loop: α* by one-step MSE, U* by coverage, what-if table, sliding windows (condition C3)
- Section VI metrics: MAPE, γ, δ, φ, η per vessel with Jain's index, μ, ν, τ_int, n_eff, block
  bootstrap, Clopper-Pearson, Diebold-Mariano, Wilcoxon with Holm and Hodges-Lehmann
- VE: G(t) per step from the GS broker, live page, replay, DTA in JSON lines
- channel monitor `fdt_chmon` for the GS radio (nl80211 survey and station counters through generic
  netlink, replay backend, OpenWrt package)
- TX99 interference injector, OpenWrt configuration of the radios
- publishers of telemetry and video and the synthetic load `raw_load.py`
- broker configuration generator, Mosquitto 2.1.2 build with the plugin tick and bridge TCP_NODELAY
  patches, chrony, sysctl, quick ACK route, systemd units
- C unit test of the vessel regulator, Python tests, end-to-end test of the whole stack on one host

- `tests/armhf/run.sh`: end-to-end test with the vessel programs on an emulated Raspbian 10 (armhf)
- `radio/channel_monitor/openwrt/build-sdk.sh`: `.ipk` of the channel monitor from the OpenWrt SDK
- `tests/hwsim/`: full-software run of the stack over a simulated Wi-Fi channel (mac80211_hwsim with
  airtime in the survey, wmediumd with the real rate per frame, one network namespace per node,
  injector of broadcast frames in place of TX99), in a Debian 13 VM
- VE: forwarded and withheld messages of the vessel regulator in the hover of each edge
- video publisher: warning when the offered messages per second exceed the DTP rate of video

- `tests/hil/`: hardware-in-the-loop run with the RP4 of a vessel (autopilot telemetry, vessel
  regulator and DTI program on the vessel; GS broker, AS and VE collector on the GS PC)
- VE: new page (stat tiles, capacity and prediction chart with admitted and received bytes,
  priority level of each topic over time, vessel x topic matrix, grant per vessel against b(t),
  fidelity and delivery per topic, table view, light and dark themes)

### Fixed

- telemetry publisher: sends its own HEARTBEAT at 1 Hz; an output udpin of MAVProxy learns the
  client address from its first packet, and the publisher waited for the autopilot heartbeat forever

- channel monitor: `sys/time.h` for musl (OpenWrt)
- vessel: paho-mqtt 2.x on Python 3.7 needs `typing_extensions`, which it does not declare

- every MQTT client sets TCP_NODELAY: paho-mqtt leaves Nagle's algorithm on, and a PUBLISH that
  follows a PUBACK waited for the delayed ACK of the broker (about 22 ms after start(t) + T1)

### Removed

- the C library, adapters, benchmarks and results of the ICECS 2026 model (tag the commit before the
  merge to recover them)
