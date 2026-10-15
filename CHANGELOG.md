# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

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
