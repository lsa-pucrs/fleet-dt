# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

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

### Changed

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
