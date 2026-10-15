# Fleet-DT v2 emulation testbed (`emu/`)

Software-only testbed of SPEC_v2.md Section 7: one QEMU/KVM VM per host (GS and M vessels) whose data
NIC is attached to a real-time ns-3.48 emulator of one shared IEEE 802.11g channel (AP = GS, one STA per
vessel). The VMs run the real Fleet-DT stack (Mosquitto 2.1.2 with the patches of `mosquitto/`, Python,
chrony); the emulator also produces the AP monitor stream of PR6 (`fleet/monitor`) and a ground truth of
every PHY event.

| Path | Content |
|---|---|
| `ns3/fdt-emu.cc`, `ns3/minijson.h` | emulator (copied to `scratch/fdt-emu/` of the ns-3 tree by `run.py build`) |
| `run.py` | orchestrator: `build`, `image`, `up`, `sync`, `ssh`, `run`, `status`, `down` |
| `monitor_bridge.py` | UDP monitor batches of the emulator → `fleet/monitor` on the GS broker |
| `clockprobe/probe.py` | host-side clock probe over vsock (and `summary`) |
| `gt.py` | reader and offline analysis of `gt.bin` (`summary`, `windows`, `collisions`, `throughput`) |
| `vm/build_image.sh`, `vm/provision-base.sh`, `vm/guest/*`, `vm/dist.sha256` | base image and guest services |
| `scenarios/*.json` | channel scenarios |
| `experiments/*.json` | experiment files for `run.py run` |
| `tools/mqtt_load.py`, `tools/mon_capture.py` | load/measurement clients (in the VMs) and host capture of `fleet/monitor` |
| `validation/` | `bianchi.sh` (ns-3 wifi-bianchi), `edca_sat.sh` (fdt-emu saturation), `eta.py` (paper model), `monitor_vs_gt.py` |

## Running

On the emulation host (`ssh cassio-server`), with the repository at `~/fleetdt-deploy/emu/repo`
(state, images and results under `~/fleetdt-deploy/emu`, env `FDT_EMU`):

```sh
cd ~/fleetdt-deploy/emu/repo
python3 emu/run.py build                      # copy emu/ns3 into ns-3.48/scratch/fdt-emu and build
python3 emu/run.py image                      # once: ~/fleetdt-deploy/emu/img/base.qcow2 (about 5 min)
python3 emu/run.py up --scenario emu/scenarios/base-m2.json   # VMs + emulator + bridge + probe (~10 s)
python3 emu/run.py ssh v1 'chronyc tracking'  # hosts: gs, v1..vM
python3 emu/run.py run emu/experiments/smoke.json             # -> ~/fleetdt-deploy/emu/runs/<time>-smoke/
python3 emu/run.py down                       # stops only the PIDs recorded by up (cmdline checked)
```

`up` options: `--clock-seed N` (offsets and frequency errors of the vessels), `--no-clock-error`,
`--chrony-minpoll/--chrony-maxpoll` (default 0/2), `--mem` (768), `--rt HardLimit|BestEffort`, `--sync`.
CPU affinity (rootless `taskset`): emulator on CPUs 12-15 (two P-cores), VMs on 16-27 (E-cores), bridge,
probe and capture on 0-11; override with `FDT_EMU_CPUS`, `FDT_VM_CPUS`, `FDT_AUX_CPUS` (empty: no pinning).

`run EXP.json` copies the repository into every VM (`~/fleet-dt`, rsync over the management path), runs
`setup` commands, restarts the emulator with the ground truth on (output in `RUN/emu/`), starts the
`start` commands in the background (`/tmp/fdt-run/<log>.log` in the VM), runs the timed `steps`, waits
`wait_s`, stops everything it started, collects `/tmp/fdt-run/` of every VM into `RUN/vms/<host>/`, and
writes `result.json` (timings, exit codes, CPU use in cores of the emulator and of each VM), `clock.csv`
and `bridge-stats.jsonl` of the run interval, and `monitor.jsonl` (host capture of `fleet/monitor`).
Commands are `str.format` templates with `{i} {name} {ip} {gs_ip} {M} {run} {repo} {vessels} {vessel_ips}`
and the experiment's `vars`. Between runs an idle emulator (no ground truth) keeps the channel up.

Offline (no VMs, not in real time), e.g. for validation:
`ns3.48-fdt-emu-optimized --scenario=S --outdir=D --vms=0 --rt=none --monitor=none --selftest=RATE_BPS,SIZE --duration=T`.

## Hosts and networks

| Item | Value |
|---|---|
| VM | Debian 13, 1 vCPU, 768 MB, qcow2 overlay on `img/base.qcow2`, cloud-init NoCloud seed per VM |
| Data NIC | `-netdev dgram` UDP 127.0.0.1: QEMU `41000+2h`, emulator `41001+2h`; MAC `52:54:00:fd:00:hh`; IP 10.10.0.1 (GS), 10.10.0.(10+i); permanent neighbor entries, no ARP, IPv6 off (`ipv6.disable=1`); route `10.10.0.0/24 quickack 1 rto_min 20ms`; `tcp_slow_start_after_idle=0` (`fdt-netsetup`) |
| Management | `-netdev user`: ssh on 127.0.0.1:`2300+h`, GS broker 1883 on 127.0.0.1:2883; vsock CID `100+h` |
| Clock | GS: chrony with `refclock PHC /dev/ptp0` (ptp_kvm, the host clock) and `allow 10.10.0.0/24`. Vessel: before chronyd, `fdt-clockinit` sets CLOCK_REALTIME to the host clock read through ptp_kvm plus an offset U[−50, 50] ms and the frequency error U[−50, 50] ppm (ADJ_FREQUENCY); chronyd then follows `10.10.0.1` over the emulated channel only, `minpoll 0 maxpoll 2`, no `makestep` (slew only) |
| Stack | `/opt/mosquitto-2.1` (cJSON 1.7.19, Mosquitto 2.1.2 + patches 0001, 0002, built by `mosquitto/build.sh`), python3-paho-mqtt, numpy, chrony, nftables, iproute2, iperf3, build-essential; the repository is copied at run time |

Tarballs (`vm/dist.sha256`, `~/fleetdt-deploy/emu/dist/`): `mosquitto-2.1.2.tar.gz` from mosquitto.org
(GPG signature of Roger Light, key A0D6 EEA1 DCAE 49A6 35A3 B2F0 779B 22DF B3E7 17B7, verified; no sha256 is
published) and `cJSON-1.7.19.tar.gz` = GitHub tag archive v1.7.19.

## Emulator

| Item | Implementation |
|---|---|
| Nodes | 0 = GS (AP), 1..M vessels (STA), then `ap2`, `sta2` (external BSS) and `jam` when the scenario has them |
| PHY | `phy`: `spectrum` (default; SpectrumWifiPhy + MultiModelSpectrumChannel) or `yans` (no jammer); `standard` `11g` (ERP-OFDM, default) or `11a` (OFDM without DSSS, same exchange airtime, see deviations) |
| MAC | QoS (EDCA), no aggregation (non-HT); AP AC_BE `aifsn_ap/cwmin_ap/cwmax_ap`, stations `aifsn_sta/cwmin_sta/cwmax_sta` advertised by the AP (`AifsnsForSta` …); BSS basic rates ERP 6/12/24 first, so beacons and management frames go at 6 Mb/s and ACKs at the control-response rate (24 for 24..54) |
| Rates | `rate.manager` `constant` (`data`, `control`, `non_unicast`) or `minstrel` |
| Relay | `UdpFdNetDevice` (FdNetDevice on the unconnected UDP socket of the VM, `sendto` to QEMU; a read error does not stop the reader) → strip Ethernet, trim padding, destination host from the destination VM MAC (or IP) → `WifiNetDevice::Send(IPv4, 0x0800)` with `SocketPriorityTag` UP = max(`prio.dscp`[DSCP], `prio.tcp_ports`[port]); defaults EF→6, AF41→5, TCP 1883→6. Reception: source host from the Wi-Fi SA, Ethernet frame rebuilt with the VM MACs. Non-IPv4 frames are dropped |
| Channel | `channel.model` `matrix` (`default_loss_db`, `loss: [[a, b, dB], ...]`, symmetric) or `tworay` (TwoRayGround at `frequency_hz` with `z` as antenna height, optional `nakagami` m0/m1/m2/d1/d2); `positions.<node>`: `[x,y,z]` or waypoints `[[t,x,y,z], ...]` |
| Interference | `external_bss`: `dir` up/down, `schedule: [[t0, t1, rate_bps, size], ...]` (OnOffApplication over UDP between ap2 and sta2, ssid `external`); `jammer`: `schedule: [[t0, t1, period_us, duty, power_dbm], ...]` (WaveformGenerator, non-CSMA) |
| Real time | RealtimeSimulatorImpl, HardLimit 50 ms (`--rt`, `--hardLimitMs`); every 10 ms the lag wall clock − (origin + simulator time) is sampled (`lag.txt`, statistics after a 3-s warm-up and for the whole run in `summary.json`) |
| Monitor | every 5 ms (`monitor_period_ms`), UDP JSON batch of SPEC 4.3 to `--monitor` (127.0.0.1:9550) |

### Monitor batch (PR6)

`{"from_us", "to_us", "busy": [[s, e], ...], "fr": [[start_us, end_us, sta, dir, ip_len, rate_kbps, ok, kind], ...]}`,
times = origin (host CLOCK_REALTIME at simulator time 0) + simulator time. `busy`: TX, RX and CCA-busy
(primary channel) of the AP PHY from a WifiPhyListener, clipped to [from, to). `fr`: one record per frame
exchange that the AP sends or decodes (also of other BSSs), in the batch in which `end_us` falls; a
unicast data or management frame waits for the ACK addressed to its transmitter that starts SIFS after it
(then `end_us` = ACK end, `ok` 1) or for a 150 µs timeout (`ok` 0, `end_us` = frame end); a frame that
the AP receives with an FCS error is a record with `ok` 0; frames without ACK (beacons, group addressed)
have `ok` 1; an ACK whose frame the AP did not decode is a record of kind 2. `sta`: vessel index, 0 for the
group address of the AP, −1 outside the fleet; `ip_len` = MSDU − 8 (LLC/SNAP) for IPv4.

### Ground truth `gt.bin`

Header: 8 bytes `FDTGT1\0\0`, u32 length L, L bytes of JSON
(`origin_ns`, `record_size` 40, `M`, `scenario`, `nodes: [{name, mac}]`). Then records of 40 bytes,
little endian (numpy dtype in `gt.py`):

| Field | Type | Meaning |
|---|---|---|
| t_ns | i64 | simulator time (ns) of the PPDU start (TX, RX: start at the receiver), state start, burst start |
| dur_ns | u32 | duration |
| uid | u32 | ns-3 UID of the MSDU (same in the TX and the RX records of one frame) |
| rate_kbps | u32 | PHY rate |
| node | u16 | node that logs the event |
| size | u16 | MPDU bytes (MAC header, body, FCS) |
| ip_len | u16 | IPv4 total length, 0 otherwise |
| src, dst | i16 | node of addr2 (−1 unknown / control frames), node of addr1 (−2 group, −1 unknown) |
| sport, dport | u16 | TCP/UDP ports of the IPv4 payload |
| ev | u8 | 0 TX, 1 RX_OK, 2 RX_ERR (payload failed), 3 RX_DROP (PPDU not received: preamble, busy, TX…), 4 STATE, 5 JAM |
| kind | u8 | 0 data, 1 beacon, 2 management, 3 ACK, 4 other control, 5 jammer, 6 QoS null/null |
| ip_proto | u8 | IPv4 protocol |
| flags | u8 | bit 0 retry, bit 1 QoS data, bits 4..7 TID |
| reason | u8 | RX_DROP: WifiPhyRxfailureReason; STATE: WifiPhyState (0 IDLE, 1 CCA_BUSY, 2 TX, 3 RX); TX: power dBm + 128 |
| pad | u8 | 0 |

TX, RX_OK, RX_ERR and RX_DROP are logged for every Wi-Fi node; STATE for the AP (`--gtStates=all` for
every node, `none`); JAM for every burst. With these, `gt.py windows` recomputes for any window κ (T_succ
of the successful exchanges of the data bridges, port 1884 by default, T_succ = frame + SIFS + ACK + AIFS of
the stations) and b (Def. busy: AP busy time minus its successful exchanges with the vessels, their ACKs
and its beacons). Volume: about 0.5 MB/s (M = 2) to 1.2 MB/s (M = 8) under load. A writer thread does the
file I/O, so the simulation thread never waits for the file system.

## Validation (2026-10-15, i7-14700, RHEL 10.2, ns-3.48 optimized)

### Bianchi (802.11g, ErpOfdmRate24Mbps, 1500-byte packets, DCF, infrastructure, 30 s per point)

`validation/bianchi.sh` runs ns-3's `wifi-bianchi.cc` (copy in which n without a reference value is a
warning); `eta.py table` gives Bianchi11g.m (the DIFS reference of ns-3) and the paper's model,
S(n) = η(n)·L/T_succ with (bianchi) and (eff) of sec2_patch.tex (T_succ 614 µs, T_coll 570 µs).

| n | ns-3 (Mb/s) | Bianchi11g.m | η(n) | paper S(n) | err vs .m | err vs paper |
|---|---|---|---|---|---|---|
| 1 | 17.604 | 17.500 | 0.9010 | 17.608 | +0.59% | −0.03% |
| 2 | 17.236 | 17.397 | 0.8953 | 17.498 | −0.93% | −1.49% |
| 3 | 16.890 | 16.981 | 0.8723 | 17.049 | −0.54% | −0.93% |
| 4 | 16.509 | 16.586 | 0.8506 | 16.625 | −0.47% | −0.70% |
| 5 | 16.215 | 16.247 | 0.8321 | 16.263 | −0.20% | −0.30% |
| 6 | 15.950 | 15.962 | 0.8165 | 15.957 | −0.07% | −0.04% |
| 7 | 15.715 | 15.716 | 0.8030 | 15.695 | −0.01% | +0.13% |
| 8 | 15.517 | 15.500 | 0.7914 | 15.467 | +0.11% | +0.32% |
| 9 | 15.336 | 15.309 | 0.7811 | 15.265 | +0.18% | +0.46% |
| 10 | 15.169 | 15.143 | 0.7719 | 15.085 | +0.17% | +0.55% |
| 11 | 15.028 | 14.991 | 0.7635 | 14.922 | +0.25% | +0.71% |
| 12 | 14.977 | 14.847 | 0.7559 | 14.774 | +0.88% | +1.38% |
| 13 | 14.788 | 14.715 | 0.7489 | 14.637 | +0.49% | +1.03% |
| 14 | 14.730 | 14.599 | 0.7424 | 14.510 | +0.90% | +1.52% |
| 15 | 14.587 | 14.490 | 0.7364 | 14.391 | +0.67% | +1.36% |
| 16 | 14.468 | 14.378 | 0.7307 | 14.280 | +0.63% | +1.31% |

The same saturation through fdt-emu itself (`validation/edca_sat.sh`: QoS data, AC_BE AIFSN 3, CWmin 15,
1500-byte IP packets, AP of the scenario; IP throughput from `gt.bin`) against `eta.py edca`
(T_succ 623 µs): n = 1: 17.20 vs 17.38 (−1.0%), 2: 16.96 vs 17.19 (−1.3%), 4: 16.20 vs 16.25 (−0.3%),
8: 15.17 vs 15.03 (+0.9%), 12: 14.47 vs 14.32 (+1.1%), 16: 13.95 vs 13.81 (+1.0%); |error| ≤ 1.6% for n = 1..16.

### Smoke tests with VMs (`experiments/smoke.json`, `smoke-m8.json`; 24 Mb/s constant, matrix 70 dB)

| Quantity | M = 2 | M = 8 |
|---|---|---|
| MQTT QoS 0, 1000-byte messages, per vessel (vessel broker → GS data bridge) | 2 Mb/s: 5000/5000 delivered per vessel, delay p50 0.55–1.2 ms, p99 1.9 ms | 1 Mb/s: 2500/2500 per vessel, p50 0.64–1.15 ms, p99 0.9–2.7 ms |
| ping GS → vessel (100 × 20 ms), avg RTT | 0.46–0.59 ms | 0.50–0.57 ms |
| iperf3 TCP upload, one vessel alone | 13.21 Mb/s | 13.21 Mb/s |
| iperf3, all vessels at once (sum; per vessel) | 12.59 (6.40, 6.19) | 10.97 (1.12–1.62) |
| expectation (`eta.py tcp`: no-collision bound 13.74; η(M+1)) | 13.14 (−4.2%); alone 13.40 (−1.4%) | 11.82 (−7.2%) |
| `fleet/monitor` at a subscriber in the GS VM: batches, gaps, age (now − to_us) | 6001, 0, p50 255 µs, p99 479 µs | 6001, 0, p50 273 µs, p99 464 µs |
| monitor bridge: emulator → bridge age; bridge → GS broker → host loop | p50 126 / p99 193 µs; p50 119 / p99 294 µs | p50 136 / p99 341 µs; p50 104 / p99 227 µs |
| b from `fleet/monitor` vs b from `gt.bin` (5-s windows) | max |Δ| 0.0001 | max |Δ| 0.0004 |
| real-time lag after 3 s: p50 / p99 / p99.9 / max | 110 / 167 / 190 / 200 µs | 114 / 277 / 1659 / 2951 µs |
| lag over the whole run, max | 213 µs | 10.6 ms (at 1.6 s, recovery of all TCP connections after the emulator restart) |
| CPU (cores): emulator / GS VM / each vessel VM / host total | 0.10 / 0.08 / 0.03–0.04 / 1.46 | 0.25 / 0.09 / 0.02–0.03 / 1.05 |

Clock (`probe.py summary` of the session from boot, M = 2, |offset| to the host clock after
convergence, probe RTT p50 120 µs): GS p50 19 µs, p99 90 µs; v1 initial −36.487 ms (configured −36.564),
below 1 ms 2.0 s after boot, then p50 32 µs, p99 150 µs; v2 initial +26.397 ms (configured +26.377),
2.0 s, p50 69 µs, p99 136 µs. M = 8: vessels p50 100–150 µs, p99 170–210 µs, at most 2 samples > 1 ms
(during iperf3 saturation). The offset of a vessel includes the asymmetry of the NTP path over the
emulated channel (the AP has the shorter AIFS) and the probe error (±RTT/2).

### Interference and b(t) (`scenarios/interf-m2.json`, `experiments/interf-m2.json`, VMs, 2 × 1 Mb/s MQTT)

External BSS 4 Mb/s (20–40 s) and 8 Mb/s (60–80 s), jammer 20 dBm duty 0.2 per 1 ms (100–120 s); 5-s windows:

| Window | b from fleet/monitor | b from gt.bin | AP busy |
|---|---|---|---|
| 5–10 s (quiet) | 0.0000 | 0.0000 | 0.132 |
| 25–30 s (external 4 Mb/s) | 0.1962 | 0.1963 | 0.329 |
| 65–70 s (external 8 Mb/s) | 0.4060 | 0.4060 | 0.539 |
| 105–110 s (jammer, includes the failed frames of the fleet) | 0.2942 | 0.2919 | 0.439 |

26 windows: mean |Δb| 0.0004, max 0.0025; busy identical within 0.0001.

### Hidden terminals (`scenarios/hidden-m2.json`: loss v1–v2 200 dB; offline, 20 s, 1500-byte UDP uplink)

| Scenario, offered per vessel | data frames acked (v1, v2) | retry fraction | AP RX errors | AP PPDU drops | IP throughput |
|---|---|---|---|---|---|
| base-m2, 6 Mb/s | 0.9999, 0.9999 | 0.0001 | 0 | 2 | 12.00 Mb/s |
| hidden-m2, 6 Mb/s | 0.517, 0.528 | 0.45 | 6045 | 9655 | 10.30 Mb/s |
| base-m2, 30 Mb/s (saturated) | 0.889, 0.888 | 0.112 | 0 | 3557 | 16.96 Mb/s |
| hidden-m2, 30 Mb/s | 0.521, 0.525 | 0.445 | 6087 | 9635 | 10.34 Mb/s |

## Deviations from SPEC_v2 Section 7 and limitations

| Item | Deviation or limitation | Reason |
|---|---|---|
| Ground truth | TX, RX ok/error and PPDU drops of every node; PHY state (CCA) records of the AP only by default (`--gtStates=all` for every node) | volume; κ and b need the AP states only |
| Monitor | batch every 5 ms of emulated time only (no extra batch at the phase ends); record semantics as above (`ok` 1 for frames without ACK, FCS-error records with `ok` 0, ACK-only records for undecoded frames) | SPEC 4.3 leaves these cases open |
| `standard: 11a` | Minstrel scenarios use 802.11a OFDM rates (OfdmRate*) instead of ERP-OFDM | ns-3 Minstrel also tries the DSSS rates of 11g, which SpectrumWifiPhy cannot send (abort "Invalid channel width for DSSS") and the real radios do not use (PR7); the exchange airtime is the same (16 µs SIFS = 10 µs SIFS + 6 µs signal extension) |
| Basic rates | ERP 6/12/24 are put first in the BSS basic rate set; ns-3 keeps DSSS 1/2 in the set (unused) | ns-3 sends management frames at basic mode 0 |
| External BSS | ns-3 OnOffApplication over UDP with an Internet stack on ap2/sta2 only | the fleet nodes have no IP stack (L2 relay) |
| Clock | the initial offset is applied relative to the host clock read through ptp_kvm; the GS follows the host clock through the same PHC | the VM RTC has 1 s resolution (offsets of up to 1.25 s were seen without it) |
| Real time | lag p99 < 0.3 ms, but bursts of small frames (e.g. the aligned keepalives of 8 control bridges every 10 s) give 1.5–3 ms; the restart of the emulator at the start of a run gives up to ~11 ms; HardLimit 50 ms was never reached | ns-3 runs only ~1.13× faster than real time when the channel is saturated with 100-byte frames at M = 8 (offline measurement); rootless: no SCHED_FIFO, only `taskset` |
| Scale | M = 16 with saturated small frames would run slower than real time | same |
| Tarballs | the copies in `~/fleetdt-deploy` differ from upstream: `cJSON-1.7.19.tar.gz` there is a master snapshot after 1.7.19 (commit 6d9f244, with `.git`, nesting-limit fixes), `mosquitto-2.1.2.tar.gz` there lacks `man/` (sources identical) | the image uses the upstream files of `vm/dist.sha256` |
| MQTT `lost` in `mqtt_load.py sub` | counts messages published before the subscriber connected when the experiment starts the publisher first | — |
