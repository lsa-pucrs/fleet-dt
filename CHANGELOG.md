# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

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

### Changed

- eps of `fleetdt.analysis.calibrate_margins` adds the largest delay of the vessel regulator at a phase boundary

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
