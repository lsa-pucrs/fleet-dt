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

### Fixed

- every MQTT client sets TCP_NODELAY: paho-mqtt leaves Nagle's algorithm on, and a PUBLISH that
  follows a PUBACK waited for the delayed ACK of the broker (about 22 ms after start(t) + T1)

### Removed

- the C library, adapters, benchmarks and results of the ICECS 2026 model (tag the commit before the
  merge to recover them)
