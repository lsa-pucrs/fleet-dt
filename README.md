# fleet-dt

Reference implementation of **Fleet-DT**, the network digital twin with dynamic bandwidth
allocation for fleets of unmanned surface vehicles (USVs) of the IEEE Access paper "Fleet-DT: A
Network Digital Twin with Dynamic Bandwidth Allocation for Fleets of Unmanned Surface Vehicles".
Built for the fleet of the Jundiá Project, whose vessels share one Wi-Fi channel (Ubiquiti Bullet
M2 radios) with a ground station (GS).

`SPEC.md` fixes the interfaces between the programs (DTP file, topics, wire formats, bridges, phase
timing); `RUNBOOK.md` gives the installation and the operation on the vessels, the GS and the radios.

## The step cycle

Each step t of Ts = 125 ms has three phases:

| Phase | Interval | Program | Action |
|---|---|---|---|
| 1 | [start(t), start(t) + T1) | DTI program of each vessel | publishes the demand table D_i(t) |
| 2 | [start(t) + T1, start(t) + T1 + T2) | AS on the GS | closes M(t), predicts Ĉ(t) = (1 − α) Ĉ(t − 1) + α c_y y(t − 1) (eq. 2), computes K̂(t) = Ĉ(t) (T3 − T_m)/Ts and the grant limit b(t) = max(0, U K̂(t) − Q(t))/m(t) with the backlog Q(t) of the demand tables (eq. 3), runs Algorithm 1, publishes p(t) |
| 3 | [start(t) + T1 + T2, start(t + 1)) | vessel regulator of each vessel | forwards the messages of step t: level 2, then level 1, both with QoS 1 (the GS deduplicates by (t, s)), each topic policed by c(x, n) = c_b x + c_m n ≤ d_{i,l}(t) (eq. 5) |

At the end of each step the channel monitor of the GS radio publishes y(t), and the VE closes the
graph G(t) of the network (Definition 7), draws it, and appends it to the DTA. The outer loop of the
AS tunes α and U from the DTA, and `fleetdt.analysis` computes the metrics of Section VI.

## Layout

| Directory | Content | Host |
|---|---|---|
| `fleetdt/` | AS (`as_service.py`), outer loop (`outer.py`), Section VI metrics (`analysis.py`), broker configuration generator (`gsconf.py`), shared model and wire formats (`model.py`, `as_model.py`), synthetic fleet (`mock.py`), VE (`ve/`) | GS |
| `vessel/regulator/` | vessel regulator, a Mosquitto 2.1 plugin (`fdt_reg.c` holds the logic) | vessel |
| `vessel/dti/` | DTI program `fdt_dti` | vessel |
| `vessel/publishers/` | telemetry (`mav_publisher.py`), video (`zed_publisher.py`), synthetic load (`raw_load.py`) | vessel |
| `radio/channel_monitor/` | channel monitor `fdt_chmon` (nl80211, no libnl) and its OpenWrt package | GS radio |
| `radio/injector/` | TX99 interference injector | extra radio |
| `radio/openwrt/` | UCI configuration of the radios | radios |
| `mosquitto/` | build of Mosquitto 2.1.2 and cJSON 1.7.19, patches, example configurations | vessels, GS |
| `system/` | chrony, sysctl, quick ACK route, systemd units | vessels, GS |
| `common/` | DTP reader, step clock and wire formats of the C programs | all |
| `tests/` | C unit test of the vessel regulator, Python tests, end-to-end test of the whole stack | any |

## Build and test

```bash
mosquitto/build.sh <dir with mosquitto-2.1.2.tar.gz and cJSON-1.7.19.tar.gz>   # once per host, into /opt/mosquitto-2.1
make                       # build/fdt_regulator.so, build/fdt_dti, build/fdt_chmon, build/fdt_chmon_host, tests
make test                  # C unit test, Python tests, end-to-end test (about 40 s)
sudo make install          # /opt/fleet-dt/{lib,bin}
pip install -r requirements.txt
tests/armhf/run.sh <dir with the tarballs>             # vessel programs on an emulated Raspbian 10 (armhf)
radio/channel_monitor/openwrt/build-sdk.sh             # fdt-chmon .ipk for the Bullet M2 (OpenWrt SDK)
tests/hwsim/vm.sh up && ...                            # full-software run over a simulated Wi-Fi channel (tests/hwsim/README.md)
```

The end-to-end test (`tests/test_integration.py`) runs on one host: two vessel brokers with the
vessel regulator, the GS broker with the bridges of `fleetdt.gsconf`, the DTI program and the
synthetic load of each vessel, the channel monitor replaying a drop of the channel, the AS and the VE.
It checks Algorithm 1 against every G(t), the bound r ≤ d of every edge, the delivery within the step
of every forwarded message, the rejection of the video topic while Ĉ(t) is low, and δ, φ and μ.
`tests/armhf/run.sh` runs the same test with the vessel brokers, DTI programs and load inside a
Debian 10 armhf container under qemu-arm (gcc 8.3, glibc 2.28, Python 3.7, armv6 code), the platform
of the vessel RP4.

## Main commands

```bash
python -m fleetdt.as_service --dtp dtp.json --broker 127.0.0.1:1883 --log as.jsonl
python -m fleetdt.ve live --broker 127.0.0.1:1883 --dtp dtp.json --dta dta.jsonl   # http://localhost:8080
python -m fleetdt.ve replay dta.jsonl
python -m fleetdt.outer dta.jsonl --target 0.99 --window 4800 --every 4800 [--c-y 1.0]
python -m fleetdt.analysis run1.jsonl run2.jsonl --dtp dtp.json --criterion count [--c-y 1.0]   # or bytes
```

The outputs use the metric names of the paper: PICP (steps with K_y(t) >= U K_hat(t), K_y(t) =
c_y y(t) (T3 - T_m)/Ts - B_ctl_bar, K_hat(t) = C_hat(t) (T3 - T_m)/Ts), DMR (deadline miss ratio, 1 -
full-delivery fraction), DMR_hat (backlog predictor q_hat(t) = max(0, q_hat(t-1) + A(t) - K_y(t)) with the
admitted bytes A(t) of the AS log), PDR_state, PDR_i (min/max ratio of H5d over vessels 1 to 3), PCR
(denominator |N(t)|), u (sum r_il / (K_y + B_ctl_bar)), MAPE (of C_hat against c_y y), c_y and RelMSE;
the subcommand `calibrate RUN` of fleetdt.analysis computes c_y (and T_m, B_ctl_bar when measured) for the
DTP; the names of earlier versions (gamma, phi, delta, eta_vessel, Pi, nu, kappa; --kappa) stay as aliases
for one release.

```bash
python -m fleetdt.gsconf gs --vessel 1=192.168.1.99 --vessel 2=192.168.1.98 > gs.conf
fdt_chmon --dtp dtp.json --iface wlan0 --broker 192.168.1.10:1883      # on the GS radio
```

## License

MIT, see `LICENSE`.
