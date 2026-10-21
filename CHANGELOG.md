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

### Changed

- outer loop: alpha* stays on the grid 0.01..1 of Section VI-E; the golden-section refinement, which can leave
  the grid, runs only with `refine=True` (`--refine`)
- paper campaign defaults: `runner.py` brings the sessions of paper-stack runs up with the CPU quota of 0.25
  of a core, the clock injector (+-20 ppm ramps, NTP loss episodes) and one probe read per step (Ts), and
  the Q specs carry this session; the emulator flags catch-up at 0.1 ms (`--catchupUs=100`,
  `"catchup_us"`); the sources release the telemetry topics with the jitter J = 2 ms of the regulator
- paper scenarios on channel 6 (`"channel_number"`; the emulator ran on the ns-3 default channel 1), and the
  two-ray models of S5 and S7 at 2.437 GHz; the files `emu/scenarios/P-*.json` regenerated
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
