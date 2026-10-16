# Fleet-DT v2 emulation testbed (`emu/`)

Software-only testbed of SPEC_v2.md Section 7: one QEMU/KVM VM per host (GS and M vessels) whose data
NIC is attached to a real-time ns-3.48 emulator of one shared IEEE 802.11g channel (AP = GS, one STA per
vessel). The VMs run the real Fleet-DT stack (Mosquitto 2.1.2 with the patches of `mosquitto/`, Python,
chrony); the emulator also produces the AP monitor stream of PR6 (`fleet/monitor`) and a ground truth of
every PHY event.

| Path | Content |
|---|---|
| `ns3/fdt-emu.cc`, `ns3/minijson.h` | emulator (copied to `scratch/fdt-emu/` of the ns-3 tree by `run.py build`) |
| `run.py` | orchestrator: `build`, `image`, `up`, `sync`, `ssh`, `run`, `status`, `down`; `--slot K` for parallel sessions |
| `fleet.py` | one Fleet-DT run of the whole stack (`run.py run` of an experiment with `"kind": "fleet"`); `--stack paper` (default) or `--stack as2` |
| `jmcs_stub.py` | operation control stub of the paper stack: `fleet/order` (sigma_t, N(t)) every step, survey/transit every 10 s |
| `experiments/matrix.py`, `runner.py`, `calibrate.py`, `dtp_base.json` | experiment matrix of Section VI (E0 to E4, saturation and pilot), parallel runner with a resumable manifest, calibration (rho, lambda, gamma, scores, Z of B4) |
| `analysis/gtstep.py`, `metrics.py`, `stats.py`, `report.py` | ground truth per step, metrics of a run, statistics, hypothesis tests, LaTeX tables and PDF figures |
| `scenarios/gen.py`, `scenarios/S<k>-m<M>.json` | channel scenarios S1 to S5 (seeded) and their seed-1 files for M = 2, 4, 8 |
| `traces/` | frame-size traces of the video topic (see `traces/README.md`) |
| `monitor_bridge.py` | UDP monitor batches of the emulator → `fleet/monitor` on the GS broker |
| `clockprobe/probe.py` | host-side clock probe over vsock (and `summary`) |
| `gt.py` | reader and offline analysis of `gt.bin` (`summary`, `windows`, `collisions`, `throughput`) |
| `vm/build_image.sh`, `vm/provision-base.sh`, `vm/guest/*`, `vm/dist.sha256` | base image and guest services |
| `scenarios/*.json` | channel scenarios |
| `experiments/*.json` | experiment files for `run.py run` |
| `tools/mqtt_load.py`, `tools/mon_capture.py` | load/measurement clients (in the VMs) and host capture of `fleet/monitor` |
| `validation/` | `bianchi.sh` (ns-3 wifi-bianchi), `edca_sat.sh` (fdt-emu saturation), `eta.py` (paper model), `monitor_vs_gt.py`, `counters_vs_gt.py` (y(t) of the nl80211 counters against gt.bin) |

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
Sessions: `run.py --slot K` (K = 0..9) offsets every resource of a session (data ports 41000 + 100 K,
ssh 2300 + 20 K + h, GS broker forward 2883 + K, monitor 9550 + K, vsock CIDs 100 + 20 K + h, VM names
`fdt-emu-sK-*`, state in `session-K/`, VMs in `vms-K/`), so several sessions run at once.
CPU affinity (rootless `taskset`): the user keeps CPUs 0-7 (4 P-cores) and 24-27 (4 E-cores); the emulator
of session K runs on P-core 4 + K (CPUs 8 + 2K and 9 + 2K, K < 4), the VMs and the host-side helpers of
every session on the E-cores 16-23; override with `FDT_EMU_CPUS`, `FDT_VM_CPUS`, `FDT_AUX_CPUS` (empty:
no pinning). Earlier measurements of this file (validation, smoke tests) used CPUs 12-15, 16-27 and 0-11.

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

## Paper stack (`fleet.py --stack paper`, the default)

`emu/run.py run EXP.json [--stack paper|as2]` runs a fleet experiment on the stack of `--stack`, else on
the `"stack"` of the experiment, else on the paper stack. The experiments of `experiments/matrix.py`
(older model) carry `"stack": "as2"`, and `experiments/runner.py` runs a spec without `"stack"` (the
frozen campaign) on the as2 stack; the paper stack refuses their DTP v2. Both campaigns use the video in
16 spatial stripe layer topics (`experiments/dtp_base.json` holds the content of `dtp_v2.json`; see
"Video in stripe layers" below).
The paper stack runs the model of the paper (Sections III to V):

| Host | Programs (per run) |
|---|---|
| GS VM | Mosquitto (`fleetdt.gsconf gs`: data bridge and control bridge per vessel; B2 three data bridges, B10 one per topic), the AS `fleetdt.as_service` (methods with tables; Algorithm 1 or `--policy b7`, SCHED_FIFO 40, decisions in `as.jsonl`; with `"outer"`, the outer loop `fleetdt.outer` every `every` steps on the DTA, in a child process), the fleet feedback `fleetdt.feedback` (B3, B8, B9; `feedback.jsonl`), the VE collector `fleetdt.ve live` (G(t) of each step in `dta.jsonl`, the DTA of the outer loop), the JMCS stub `emu/jmcs_stub.py` (`order.jsonl`) |
| vessel VM i | `RUN/dtp_vessel.json` = `video_layers.py resolve` of the DTP for vessel i (rates and bounds of the layer topics from its `size_scale`); Mosquitto with `fdt_regulator.so` in the mode of the method, `plugin_opt_log RUN/regulator.jsonl` (vessel log), `plugin_opt_step_log RUN/regsteps.jsonl` (step summaries: p, d, rx, fwd, held, cost per topic), `plugin_opt_demand external`, `plugin_opt_jitter_ms J`; the v1 DTI program `build/fdt_dti --jitter-ms J` (D_i(t) at start(t) + `dti.offset_ms`, SCHED_FIFO 45); the stripe video source `vessel/publishers/fdt_source.py` (frame sizes of a trace times the stripe shares, `src.jsonl`), or `raw_load.py`, or `"source": {"cmd": ...}`; the sampler `build/fdt_outq` (SIOCOUTQ of the bridge sockets, `RUN/outq.jsonl`, built by the preparation from `emu/tools/outq_sampler.c`, started after the bridges connect, steps S0 .. S0 + steps) |
| host | emulator with the ground truth and the nl80211 counter export (`--cnt`, `--cntDst` = 127.0.0.1:9650+K), `fleetdt.monitor_adapter` (y(t) on `fleet/throughput` through 127.0.0.1:2883+K), monitor bridge and capture (ground truth), clock probe |

Steps are 0-based on every program of this stack: start(t) = epoch + t Ts (the DTP epoch; null =
2026-01-01T00:00:00Z). The run picks S0 at least `lead_s` ahead, passes `--runStartUs` = start(S0) to the
emulator (the counter reads fall at start(t) + `monitor.offset_us` + U[0, `monitor.jitter_us`]) and stops
after `steps` steps. The paper stack starts the adapter, the emulator, the brokers, waits for the bridges
(2M; B2 4M; B10 M (L + 1)), then the VE, the JMCS stub, the fleet feedback, the AS, the DTI programs and
the sources.

Methods (`"method"`, design Section 5; the regulator modes are those of `vessel/regulator/fdt_reg_paper.h`):

| Method | AS | Regulator mode | Data bridges | Fleet feedback |
|---|---|---|---|---|
| `fleetdt` | Algorithm 1 | `paper` | one | no |
| `B1` | none | `paper_b1` (forward on arrival, QoS 1) | one | no |
| `B2` | none | `paper_b2` (as B1, one bridge per access category) | three (`--b2`) | no |
| `B3` | none | `paper_b3` (strict priority in sigma_t at v(t), QoS 1) | one | sigma_t |
| `B4` | Algorithm 1, `--alpha 0 --C0 as.C_ref`, no outer loop | `paper` | one | no |
| `B5a`, `B5b` | Algorithm 1 | `paper_nopolice`, `paper_noeos` | one | no |
| `B7` | `--policy b7` (per-vessel grants, redistribution of the unused grant) | `paper` | one | no |
| `B8` | none | `b8` (budget U y(t-1) T3 / (Ts m) in sigma_t, QoS 1) | one | y, U, m, sigma_t |
| `B9` | none | `paper_tdma` (equal slots of phase 3 in the order of `vessels`, QoS 1) | one | sigma_t |
| `B10` | none | `paper_b1` | one per topic (`--per-topic`) | no |

The older core keeps its modes `b1`, `b2`, `b3`, `tdma` for the as2 stack (1-based clock, JSON tables
and logs, QoS of the DTP v2); the GS programs of the paper stack read the 0-based step header and the
binary tables, so the paper stack runs the baselines in the core of mode paper.

Experiment fields (example: `experiments/paper-smoke-m2.json`; `"dtp"` and `"scenario"` may be objects or
repository paths, a DTP path gets `"vessels"` = 1..M):

| Field | Meaning (default) |
|---|---|
| `dtp` | DTP of the paper (SPEC.md: topics with `rate` and `bound`); a DTP v2 is refused (use `--stack as2`) |
| `method` | `fleetdt`; one of the methods above (case-insensitive) |
| `plugin_mode`, `plugin_opts` | the mode of the method (a different mode is refused); extra `plugin_opt_<k> <v>` lines of the vessel brokers; `plugin_opts.jitter_ms` (2) is J of the regulator and of `fdt_dti` |
| `as` | `U`, `alpha`, `C0` (override the DTP), `C_ref` (B4), `extra` (arguments of `fleetdt.as_service`) |
| `feedback` | `U` (B8; default `as.U`, else the DTP), `m` (default the vessels of the DTP) |
| `outer` | `every`, `window`, `target` (0.99); `dta_file`: a DTA of earlier runs (host path) instead of the DTA of the run |
| `dti` | `enabled` (true; false: the plugin publishes D_i(t), `plugin_opt_demand internal`), `offset_ms` (1), `stagger_ms` (0, added per vessel index), `idle` (8), `jitter_ms` (the J of `plugin_opts`; both must agree) |
| `jmcs` | `period_s` (10), `first` (survey), `offset_ms` (2), `video_prefix` (video), `modes` (explicit orders) |
| `monitor` | `offset_us` (500), `jitter_us` (0), `quant` (harvest or cumulative), `harvest_ms` (0), `sigma` (0; 0.02, 0.05), `noise_seed` (seed), `backend` (auto, c, py) |
| `source` | `kind` `fdt_source` (default: `trace` (`emu/traces/water-sbs-q90-s1.txt`; null with `median`, `sigma`: lognormal), `scale` (scale of the frame sizes, `load_mode`), `jitter_ms`, `phase`, `stream_size`, `topics`, `allow_trunc`) or `raw_load` (`scale`, `size`, `min_frac`, `topics`), or `cmd` (template with `{dtp}`, `{dtp_vessel}`, `{vessel}`, `{seed}`, `{t0}`, `{duration}`, `{run}`). The run refuses a trace replay whose largest frame times `scale` exceeds `video_layers.frame_cap` (the source would truncate stripes) or whose share profile (`q<Q>-s<S>` of the file name) differs from `video_layers.profile`, unless `allow_trunc` |
| `abort_lag_us` | lag at which the emulator stops the run (`--abortLagUs`, default 1000; 0 records only, as in Q); the emulator always runs with `--phaseLag` (Ts:T1:T2 of the DTP) and `--stepBytes`; an emulator that exits early (status 3: lag abort) or reports `phase_lag.aborted` fails the run; `emu_args` may not repeat these options |
| `pcap` | true: `--pcap=1`, the emulator writes `RUN/emu/ap.pcap` (IPv4 packets between the AP node and the GS; E0 conformance) |
| `outq` | `enabled` (true), `fifo` (SCHED_FIFO priority of the sampler) |
| `clock` | E1: `{"inject": true, "ramp_ppm", "ramp_s", "hold_s", "loss_n", "loss_s", "loss_windows", "warm_s", "seed"}`: per vessel `fdt_clock_schedule.make(...)` with the origin at start(S0) and ppm0 = the frequency error of the vessel in the session, installed at `/etc/fleetdt/clock-schedule.json` before the emulator starts, `/var/log/fdt-clockerr.jsonl` copied to `RUN/clockerr.jsonl` after the run, the schedule removed; needs a session up with `--clock-inject` |
| `replay_subsets`, `replay_jobs` | E2: after the run, `emu/replay_subsets.py RUN --jobs J` (default 4) on the auxiliary CPUs (`replay_subsets.log`, `result.json` "replay_subsets") |
| `stack_commit` | refuse the run unless the checkout is this commit and the files of the stack are unchanged |

RUN/result.json records `stack`, `revision` (git `commit`, `branch`, `describe`, `dirty`, `dirty_stack`:
the files of the stack that differ from the commit, the tree digest, and `stack_files`: sha256 of every
file of the stack), `dtp_sha256`, `dtp_vessels_sha256`, `method`, `method_config`, `bridges`, `jitter_ms`,
`monitor` (with `backend_used`), `chmon_host_sha256`, `plugin_mode`, `demand`, `commands` (the command
lines of the run), S0, t0 and the emulator summary (with `nl80211`). RUN/dtp_b<i>.json holds the DTP of
vessel i.
`result.json` also records `video` (frame rate, frame cap, profile, trace, scale, mean frame over the cap),
`abort_lag_us`, `pcap`, `cpu_profile` (`ratio` of the session and `emu/vm/cpu_profile.py status` of every
QEMU process before and after the run), `clock` (E1), `emu_exit` and `replay_subsets`; `emu_summary` adds
`phase_lag`, `qdisc`, `conf`, `pcap` and `step_bytes` of `emu/summary.json`. The AS log `as.jsonl` holds
`D` (the demand tables of M(t)) and `y_used` (the y(t - 1) of eq. (2)), and each G(t) of `dta.jsonl` its
emission time `emitted`, for the conformance check.
RUN/ also holds `throughput.jsonl` (each published y(t) with its publication time), `counters_rx.jsonl`
(the counter reads that reached the adapter), `monitor_adapter.log`, `emu/counters.jsonl` (every read of
the emulator) and `vms/<host>/` (`as.jsonl`, `dta.jsonl`, `order.jsonl`, `regulator.jsonl`, broker logs).

### y(t) of R6 from nl80211 counters

The emulator exports only what nl80211 gives on a deployed AP (`fdt-emu --cnt=TS_US:OFF_US[:JITTER_US]`):
the survey of the channel in use (active and busy time in whole ms) and, per associated vessel, RX/TX
bytes, RX/TX packets, tx_failed and tx_retries, read at the step boundaries with a uniform read jitter;
one JSON line per read in the sample format of `fdt_chmon --record` (the sums over the stations, as
fdt_chmon sums a station dump, plus `"sta"`). The counter semantics follow mac80211 + ath9k:

| Counter | Emulated as |
|---|---|
| survey busy | union of the AP PHY TX, RX and primary-channel CCA-busy periods (rx_clear) |
| survey time | radio-on time |
| quantization | `--cntQuant=harvest` (ath9k): each harvest adds floor(time since the last harvest / 1 ms) and drops the remainder; harvests at each read and every `--cntHarvestMs` (ANI poll; 0: off). `cumulative`: floor(total / 1 ms) |
| rx_bytes, rx_packets | unicast data, QoS-null and management frames received without error from the vessel, duplicates (Retry bit and the sequence control of the last frame of the TID) left out; bytes = MPDU without FCS |
| tx_bytes, tx_packets | the same frames sent to the vessel, once per MPDU at its first transmission (Retry bit clear) |
| tx_failed | MPDUs dropped at the retry limit, or at the lifetime after a failed transmission |
| station entry | leaves the dump when the vessel deassociates, returns with zeroed counters |

`fleetdt.monitor_adapter` (host) receives the reads (UDP), pairs consecutive reads as the live loop of
`fdt_chmon` does (a read at start(t) closes step t - 1) and computes y(t) = R_PHY (Ts - Tb + To) / 8 with
the unchanged `compute()` of `radio/channel_monitor/fdt_chmon.c`: backend `c` runs `fdt_chmon
--replay-stream -` built for the host by `radio/channel_monitor/build_host.sh` (`build/fdt_chmon_host`,
built by `fleet.py` before each run); backend `py` is the line-by-line port in the adapter
(`tests/test_monitor_adapter.py` checks that both print the same lines). With `monitor.sigma` > 0 the
adapter multiplies y by (1 + sigma e), e ~ N(0, 1) (seeded), and keeps the noise-free value in `"y0"`.
It publishes `{"t", "y", "Tb", "To"}` (QoS 0) on `fleet/throughput`, the format that
`fleetdt.as_service` parses.

Validation against the ground truth of the same run:
`python3 emu/validation/counters_vs_gt.py RUN/emu/counters.jsonl RUN/emu/gt.bin --dtp RUN/dtp.json --y RUN/throughput.jsonl`
compares y_cnt with y_model (exact busy time, 8 (MPDU bytes + h_W) / R_PHY of every acknowledged unique
MPDU: the error of the counters), y_phy (exact busy time and measured airtime: Section V-B with the true
durations) and y_step (y_phy over the nominal step). Offline, without VMs:

```sh
B=$NS3/build/scratch/fdt-emu/ns3.48-fdt-emu-optimized
$B --scenario=emu/scenarios/S3-m2.json --outdir=D --vms=0 --rt=none --monitor=none \
   --selftest=4000000,1400 --selftestDown=2000000,1200 --duration=40 --runStartUs=U --cnt=125000:500:2000
python3 emu/validation/counters_vs_gt.py D/counters.jsonl D/gt.bin --dtp dtp.json
```

(`--selftestDown=RATE_BPS,SIZE` adds downlink sources of the GS to every vessel, so the TX counters see
traffic; `U` = a step start, e.g. start(floor((now - epoch) / Ts) + 1) in microseconds.)

## Fleet-DT runs and the experiments of Section VI (stack as2)

A fleet experiment of the older model (`"kind": "fleet"`, written by `experiments/matrix.py`) runs the whole stack of SPEC_v2.md in one session (`fleet.py --stack as2`):

| Host | Programs (per run) |
|---|---|
| GS VM | Mosquitto (`fleetdt.gsconf gs`: data and control bridge per vessel; mode b2: three data bridges per vessel), the AS (`fleetdt.as2.service`, SCHED_FIFO 40, DTA in `dta.jsonl`), the operation control stub (`fleetdt.as2.opctl`, `order.jsonl`), nftables (`system/nftables-fdt.nft`), chrony server |
| vessel VM i | Mosquitto with the plugin (`fdt_regulator.so` built from the synchronized repository, mode of the method, SCHED_FIFO 50), the sources (`vessel/publishers/fdt_source.py` with the DTP of the run and the frame-size trace, `src.jsonl`), nftables, chrony client |
| host | emulator with the ground truth (`--runStartUs` = start of the first step, `--flush` = extra monitor batches at the end of Q(t) and of W(t)), monitor bridge to the GS broker, monitor capture, clock probe |

Run timeline: preparation of every VM (DTP, broker configurations, nftables, plugin build, check that the
guest Mosquitto carries exactly the patches of `mosquitto/patches/`), first step S0 at least 15 s ahead,
emulator restart, vessel brokers, GS broker, wait for all bridges, AS (steps S0 .. S0 + steps), stub and
sources, wait, stop, collection. RUN/ holds `experiment.json`, `dtp.json`, `scenario.json`,
`result.json` (status, S0, timings, CPU, emulator lag summary, guest patches, revision: commit of the
repository copy, dirty flag, sha256 of the diff and of the tree), `vms/<host>/` (DTA, broker logs with
the plugin statistics, source logs, orders, chrony before and after, nft ruleset, plugin sha256),
`emu/` (gt.bin, lag.txt, summary.json, emu.out), `monitor.jsonl`, `clock.csv`, `bridge-stats.jsonl`.

Workflow (emulation host, repository at `~/fleetdt-deploy/emu/repo`; C = `~/fleetdt-deploy/emu/campaigns`,
R = `~/fleetdt-deploy/emu/results`):

```sh
python3 emu/experiments/matrix.py sat --out $C/sat                       # 14 saturation runs (B1)
python3 emu/experiments/runner.py $C/sat --K 3                           # -> $R/sat
python3 emu/experiments/calibrate.py sat $R/sat --calib $C/calibration.json
python3 emu/experiments/matrix.py e0 --calib $C/calibration.json --out $C/e0     # 60 runs
python3 emu/experiments/runner.py $C/e0 --K 3                            # -> $R/e0
python3 emu/experiments/calibrate.py e0 $R/e0 --calib $C/calibration.json
python3 emu/experiments/matrix.py eval --calib $C/calibration.json --out $C/eval # E1-E4, 1524 runs
python3 emu/experiments/runner.py $C/eval --K 3                          # -> $R/eval (resumable)
python3 emu/analysis/report.py $R/eval --out $R/report --whatif $C/eval/whatif   # hypotheses, tables, figures
```

`matrix.py eval` writes the initial scores of the AS once per scenario and predictor (`$C/eval/init/`), and
the runner passes the file of each run to `fleet.py`, which copies it to the GS. E0 runs with gamma = 0.05
(the evaluation runs use the calibrated lambda and gamma; `calibrate.py e0` selects gamma by a replay of
eq. (aci) on the E0 scores with a failure model fitted per scenario, see its docstring; the E0 campaign of
2026-10-15 gave lambda 0.75, ES gain 0.4, gamma 0.05). `runner.py` keeps the status of every run in the
manifest (resumable; `touch $C/eval/STOP` stops after the current runs), post-processes each run (`analysis/gtstep.py` -> `gt_steps.jsonl`, `analysis/metrics.py`
-> `metrics.json`, zstd of gt.bin and monitor.jsonl) and lowers K after two lagging runs in a row (p99
lag > 300 us or more than 1 % of the steps with a lag above 1 ms; a lagging run is queued again once; the
scenarios of `--lag-exempt`, default S2, count only by the steps above 1 ms, see the lag table below).

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
| Run start | `--runStartUs=U`: the times of the interference schedules and of the waypoints count from the host time U (the start of the first step of a run); before it, a node stays at its first waypoint |
| Phase batches | `--flush=TS_US:OFF_US,...`: extra monitor batches at the host times U + k TS + OFF (fleet runs: 2.5 ms after the end of Q(t) and of W(t)), so that the AS has every record of Q(t) at phase 2 |
| nl80211 counters | `--cnt=TS_US:OFF_US[:JITTER_US]`: reads of the AP counters at U + k TS + OFF + U[0, JITTER] (own generator, `--cntSeed`), one JSON line each to `OUTDIR/counters.jsonl` (`--cntFile`) and as a UDP datagram to `--cntDst`; `--cntQuant`, `--cntHarvestMs` (see "y(t) of R6 from nl80211 counters"); `summary.json` `nl80211` |
| Self test | `--selftest=RATE_BPS,SIZE` uplink and `--selftestDown=RATE_BPS,SIZE` downlink UDP sources (no VMs) |

### Monitor batch (PR6)

`{"from_us", "to_us", "busy": [[s, e], ...], "fr": [[start_us, end_us, sta, dir, ip_len, rate_kbps, ok, kind, ac], ...]}`,
times = origin (host CLOCK_REALTIME at simulator time 0) + simulator time. `busy`: TX, RX and CCA-busy
(primary channel) of the AP PHY from a WifiPhyListener, clipped to [from, to). `fr`: one record per frame
exchange that the AP sends or decodes (also of other BSSs), in the batch in which `end_us` falls; a
unicast data or management frame waits for the ACK addressed to its transmitter that starts SIFS after it
(then `end_us` = ACK end, `ok` 1) or for a 150 µs timeout (`ok` 0, `end_us` = frame end); a frame that
the AP receives with an FCS error is a record with `ok` 0; frames without ACK (beacons, group addressed)
have `ok` 1; an ACK whose frame the AP did not decode is a record of kind 2. `sta`: vessel index, 0 for the
group address of the AP, −1 outside the fleet; `ip_len` = MSDU − 8 (LLC/SNAP) for IPv4; `ac` = access
category of a QoS data frame from its TID (ns-3 AcIndex: 0 AC_BE, 1 AC_BK, 2 AC_VI, 3 AC_VO), −1 for the
other frames. Check (run `t1`, S1, M = 4, Fleet-DT, 30 s): every frame of TCP port 1883 (control bridges)
and of UDP port 123 (NTP, marked like the control bridge by `system/nftables-fdt.nft`) carries TID 6 and
`ac` 3, every frame of port 1884 (data bridges) TID 0 and `ac` 0; the AS takes the data frames of the
vessels with `ac` 0 as the frames of the data bridges. Mode b2 marks its bridges 1885 (AF41, UP 5, AC_VI)
and 1886 (EF, UP 6, AC_VO); 1884 stays AC_BE.

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
With `"phy": "yans"` (YansWifiChannel, same scenario): n = 1: 17.34 (−0.2%), 2: 17.15 (−0.2%), 4: 16.41 (+1.0%),
8: 15.24 (+1.4%), 12: 14.64 (+2.2%), 16: 14.13 (+2.3%) against `eta.py edca`.

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

### Guest broker and plugin ticks (image of 2026-10-15, patches 0001-0003 of `mosquitto/patches/`)

`validation/tick_lateness.sh` (in a vessel VM; probe plugin `validation/tickprobe.c` asking for a tick every
1 ms; broker under SCHED_FIFO 50; 20 s each). The guest runs Mosquitto 2.1.2 built with exactly the
repository patches (the image records their sha256 in `/opt/mosquitto-2.1/patches.sha256`, and every fleet
run checks them against the repository):

| Load | Ticks | Lateness p50 / p90 / p99 / p99.9 / max (us) | Ticks later than 1 ms |
|---|---|---|---|
| idle | 20381 | 5 / 11 / 24 / 65 / 152 | 0 |
| 2000 messages/s of unrelated traffic | 16795 | 481 / 505 / 522 / 556 / 1314 | 1 |

With other events, patch 0001 keeps the deadline of the tick, but the loop sleeps in whole milliseconds
(`epoll_wait`), so a tick that another event precedes comes up to 1 ms late (rounded up to the next
millisecond); L_w covers T_tick + d_m at 24 Mb/s, and 1 ms more of lateness needs L_w >= R (T_tick + d_m +
1 ms)/8 = 20.3 kB at 54 Mb/s (S5) against 16384 (open issue).

### Real-time lag with K concurrent sessions (fleet runs, M = 4, 1200 steps, 2026-10-15)

| Runs | K | Lag p99 (us) | Steps with a lag above 1 ms |
|---|---|---|---|
| S1 (Fleet-DT, B1, B3, TDMA, slotted), S4, S5 | 3 | 160-225 | 0-0.5 % |
| S2 (Fleet-DT, spectrum) | 3 | 290-408 | 0.2-0.4 % |
| S2 (Fleet-DT, spectrum) | 1 | 415 | 2.9 % |
| S2 (Fleet-DT, `"phy": "yans"`) | 3 | 218 | 0 % |
| saturation runs (B1, video far above capacity) S1, S3, S4, S5 | 3 | 165-240 | 0-2.5 % |
| saturation runs S2, M = 4 | 3 | 418-474 | 0-0.6 % |
| S2, M = 8: B1 and Fleet-DT, rho = 1.5 (spectrum / yans) | 1 | 6227 / 3441 (B1), 8767 / 5602 (Fleet-DT) | 99-100 % |
| E4 cells, Fleet-DT, S2, 600 steps (2026-10-15, after the phase 1 stagger i T1/(M+1)): M = 2, Ts 125, spectrum | 3 | 207 | 0 % |
| M = 4, Ts 62.5: spectrum / yans | 3 / 2 | 467 / 264 | 4.0 % / 0.4 % |
| M = 4, Ts 250: spectrum / yans | 3 / 2 | 755 / 382 | 12.5 % / 5.3 % |
| M = 6, Ts 125, yans, emulator on CPUs 8-15 | 1 | 955 | 15.8 % |
| M = 8, Ts 125, yans, emulator on CPUs 8-15 (two runs) | 1 | 3966 / 4489 (max 8.4 ms) | 100 % |
| S1, M = 8, Ts 125, yans, emulator on CPUs 8-15 | 1 | 1319 | 67 % |
| M = 4, Ts 250, yans, guarded stagger, 600 steps, seeds 1001 / 1002 | 1 | 375 / 337 | 0.6 % / 1.7 % |
| M = 4, Ts 62.5, yans, guarded stagger, 1200 steps, seeds 1001-1003 at once | 3 | 229-255 | 0.2-0.4 % |

E4 (freeze of 2026-10-15) keeps the cells that meet the rule: M = 2 with Ts 125 ms (spectrum) and M = 4
with Ts 62.5 ms (yans), both with K = 3 (`matrix.py` E4_CELLS, field `phy` of the scenario and `max_K` of
the manifest). M = 8 and M = 4 with Ts 250 ms have no runs: `matrix.py eval` writes their DTPs to
`whatif/` of the manifest, and `report.py --whatif DIR` reports the what-if prediction of H6 for them
without a measured counterpart. `runner.py` honours a field `max_K` per run (the run starts only on the
slots 0 .. max_K - 1 and only while at most max_K sessions run).

The lag of S2 does not fall with K (K = 1 is not better than K = 3), so K = 3 stays (3 sessions of M = 4
use 15 VMs and keep more than 20 GB and the CPUs 0-7, 24-27 free). With M = 8, the emulator falls up to
10 ms behind in every step from the tables of phase 2 to the middle of W(t) (its simulation thread uses
about half a core on average): the runs of E4 with M = 8 cannot run in real time on this host (open issue).
The lag builds up where the channel carries the most frames: the ground truth of the M = 8 run (S2, yans)
holds 7 frames/ms in phase 1 (demands, no lag), 10 to 13 frames/ms from the tables of phase 2 (26-30 ms
after start(t)) to the first 12 ms of W(t) (lag up to 5 ms), and fewer than 5 frames/ms later in W(t)
(the lag returns to 0.2 ms). With 11 nodes, the simulation thread (ns-3.48, optimized build, one thread)
processes about 9 to 10 frames per millisecond of wall clock; a perf profile shows no dominant function
(malloc/free 18 %, the scheduler, the PHY and the channel access of ns-3), and 8 CPUs instead of 2 for
the emulator do not change the lag. Neither the PHY model (yans halves the lag) nor the pinning brings
M >= 6 or Ts = 250 ms under the lag rule.
Without VMs, the same relay load (bursts of 11 to 30 frames per vessel at the start of W(t), S2, M = 8)
stays at a lag p99 of 259 to 390 us (`validation/burst_relay.py`), so the extra cost comes with the TCP and
MQTT traffic of the VMs (more frames per step, acknowledgments of the AP).

### Phase 1 stagger and the demand deadline (H1), 2026-10-15

The regulator publishes the demand of vessel i at i (T1 - g1) / (M + 1) after start(t) with the guard
g1 = eps + 2 T_tick of the DTP (4 ms; the rule of the paper since the freeze of 2026-10-15; plugin option
`p1_guard_us` and `matrix.py --p1-guard-us` override it), and a step of the regulator ends at start(t + 1) for every vessel
(the offset does not extend the release of step t). The AS accepts the demands that arrive in
[start(t) - eps, start(t) + T1). With g = 0 (offsets 4, 8, 12, 16 ms at M = 4), the demand of vessel 4
leaves T1/(M + 1) = 4 ms to the deadline, and the measured path takes up to that: the phase 1 lateness
of the plugin (1-ms epoll granularity of the broker; late_p1 p99 1.9 ms, up to 3.7 ms), the way from the
broker of the vessel to the air (about 1 to 2 ms with 15 VMs on the CPUs 16-23) and the GS (the AP
receives the demand 1 to 1.7 ms before start(t) + T1 in missed steps; the GS acknowledges it only after
the deadline). E0 with g = 0 (60 runs, K = 3): demand miss above 1e-3 in 21 runs (S5 10, S4 8, S1, S2,
S3 one each; max 0.008), 276 of 289 missed demands of vessel 4. The same 6 worst runs with g = 4 ms
(offsets 3.2, 6.4, 9.6, 12.8 ms): max 2.6e-4, against 1.2e-3 to 8.0e-3 (E0) and 0 to 4.2e-3 (a repeat
with g = 0).

### Yans and Spectrum (fleet runs, Fleet-DT, S2, M = 4, rho 1.5, 1200 steps, seeds 1001 and 1002, K = 2)

| Seed | PHY | kappa(t) mean / p10 / p50 / p90 | b(t) mean / p90 | f_bar | D_N | MAE | nu | Lag p99 (us), steps above 1 ms |
|---|---|---|---|---|---|---|---|---|
| 1001 | spectrum | 0.689 / 0.513 / 0.745 / 0.804 | 0.109 / 0.305 | 0.0505 | 0.982 | 0.051 | 1.208 | 438, 1.0 % |
| 1001 | yans | 0.692 / 0.516 / 0.742 / 0.808 | 0.109 / 0.307 | 0.0504 | 0.977 | 0.050 | 1.193 | 237, 0.8 % |
| 1002 | spectrum | 0.693 / 0.510 / 0.752 / 0.806 | 0.106 / 0.330 | 0.0526 | 0.983 | 0.051 | 1.194 | 324, 1.0 % |
| 1002 | yans | 0.693 / 0.516 / 0.755 / 0.802 | 0.105 / 0.329 | 0.0504 | 0.974 | 0.052 | 1.198 | 234, 0.8 % |

Two-sample Kolmogorov-Smirnov distance between the PHY models (steps after the warm-up): kappa 0.031 and
0.033 (p 0.74, 0.66), b 0.015 and 0.014 (p 1.0); pooled mean of kappa +0.20 % with yans, of b -0.0009.
The same seed gives other backoff and fading draws with the two models (other random streams), so the
comparison is between distributions. `matrix.py eval --yans E4` writes the cells of E4 with yans.

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
| Monitor batches | besides the batches every 5 ms, one batch 2.5 ms after the end of Q(t) and of W(t) (`--flush`); without it the AS computed b(t) from up to 5 ms old records and missed the exchanges that cross the end of Q(t) (b too high by up to 0.03 in a saturated channel) | records join the batch of the end of their exchange |
| Clock synchronization | NTP (UDP 123) is marked like the control bridges (UP 6, AC_VO) | the AS takes every AC_BE data frame of a vessel for a data-bridge frame |
| MQTT `lost` in `mqtt_load.py sub` | counts messages published before the subscriber connected when the experiment starts the publisher first | — |

### Session options of `run.py up`

| Option | Effect |
|---|---|
| (always) | QEMU with `-cpu host,+invtsc` (invariant TSC for the TSC clock source of the guests) |
| `--cpu-ratio R` | each vessel QEMU runs in a systemd user scope with CPUQuota = R of one CPU (`emu/vm/cpu_profile.py wrap`, inside the CPU pinning); off by default; `state.json` "cpu_ratio" |
| `--clock-inject` | the `emu-host.json` of each vessel names `/etc/fleetdt/clock-schedule.json` in "clock", so `fdt-clockinit` sets only the offset and the frequency error comes from the schedule of each run (`"clock": {"inject": true}`) |

### Post-processing and E0 conformance

`experiments/runner.py` post-processes a paper-stack run with `analysis/paper_post.py RUN` (the metrics of
the paper from `vms/gs/dta.jsonl` with `fleetdt/analysis.py`, after a warm-up of 240 steps, steps with a
lag above 1 ms in `steplag.csv` counted in `lag_discarded` and left out; `RUN/metrics.json`), and an as2 run
with `gtstep.py` and `metrics.py` of the older model.

`python3 emu/experiments/conformance.py RUN` checks a paper-stack run against the model (E0): offline
replay of eq. (2), the grant limit and Algorithm 1 from the logged D_i(t), y(t - 1), sigma_t and N(t)
against the published p(t) (and the tables each regulator received and policed, and the JMCS mode of each
step); forwarded channel bytes per edge <= d_il(t) (regulator step log and r_il of the DTA); the QoS of
each forwarded PUBLISH in `emu/ap.pcap` equal to its level (no level-0 message; baselines QoS 1; skipped
without a pcap); no uplink message of step t from a vessel without p(t); every message not forwarded in
the vessel log with a reason (rx - fwd = log lines = held + drop) and every source message received;
G(t) against the ground truth (content bytes at the GS at most the IP bytes of `stepbytes.csv`, the
relative error of r_il against them, M(t) of the DTA equal to the M(t) of the AS, DTI lag <= 1 step).
`RUN/conformance.json`; exit status 0 when the run conforms. Tests: `tests/test_conformance.py`
(synthetic run directories with injected defects).

## Video in stripe layers (`vessel/publishers/video_layers.py`)

Every frame splits into 16 horizontal stripes (`stripe_rows`), each encoded as its own JPEG; layer topic
`video_j` carries stripe rev4(j - 1) of every frame (layers 1, 2, 3, 4, 5 = stripes 0, 8, 4, 12, 2), so
admitting layers 1..j gives j of 16 stripes spread over the image. A layer topic has one stripe per frame:
with F = ceil(f Ts) frames per step and a stripe cap cap_j = ceil(s_i frame_cap w_rev(j-1)) split into
m_j = ceil(cap_j / 1392) messages of bound_j = ceil(cap_j / m_j) + 8 <= 1400 bytes, the DTP declares
rate_j = m_j F / Ts and bound_j ("jitter": 0), so d_il covers exactly the F stripes of a step.
`frame_cap` bounds the trace size of a frame (dtp.json: 98774 bytes, the largest frame of
`water-sbs-q90-s1.txt`) and w_s is the share profile of `emu/traces/stripe-shares.json`
(`emu/traces/make_stripe_profile.py`: 24 pairs of the water sequence spread over the 4500 frames of the
traces, each side-by-side frame encoded whole and as 16 stripe JPEGs with OpenCV 4.13.0; w_s = mean
stripe bytes / whole-frame bytes; profile q90-s1: shares 0.039 (top) to 0.112, sum 1.178 because every
stripe carries its own JPEG headers; q50-s0p5 sums to 2.10). `fdt_source.py` gives stripe s of frame k
the size round(F_k w_s) (trace size F_k times the size factor s_i of the vessel and the source scale);
`zed_publisher.py` encodes each stripe within its cap (lower quality, then half resolution).
`plan_for_rho(dtp, M, rho, C, frame_max)` picks per cell the smallest frame rate of {8, 16, 24, 32}
frames per second (an integer number of frames per step) whose frame cap for sum_i sum_l d_il = rho C
stays within the largest trace frame, and the source scale = frame_cap / frame_max.

Cells of `tests/test_video_layers.py` (C = 320 kB per step, U = 0.8, J = 2 ms, trace q90-s1, vessels
1..M of dtp.json with size factors 1.0, 0.9, 0.95, 0.85; forwarded / declared from the stripes of
`fdt_source.Source` with release delays U[0, 3 ms], collection windows [v(t-1), v(t)) and the policing of
the regulator, 600 steps):

| M | rho | frames/s | frame cap (scale) | layers admitted | fill f (static; per-step variation mean) | forwarded / declared per layer (mean; min) |
|---|---|---|---|---|---|---|
| 2 | 1.5 | 16 | 86811 (0.879) | 7 | 0.978; 0.965 | 0.937; 0.928 |
| 2 | 2.0 | 24 | 77347 (0.783) | 4 | 0.912; 0.952 | 0.932; 0.924 |
| 4 | 1.5 | 8 | 87640 (0.887) | 6 | 0.932; 0.962 | 0.936; 0.921 |
| 4 | 2.0 | 16 | 58583 (0.593) | 4 | 0.964; 0.951 | 0.920; 0.894 |

The temporal layering it replaces (frame k on layer rev4(k mod 16) + 1, 8 messages of 1400 B declared per
step at 15 frames per second) used about 1/8.5 of the declared demand of each layer.

## Paper scenarios S1 to S9 and queue variants (design v2, item I8)

`python3 emu/scenarios/gen.py --paper --out emu/scenarios [--M 2 4] [--seed 1] [--duration 180]` writes
`P-S<k>-m<M>.json` and the queue variants `P-S1-{q100,q1000,codel}-m<M>.json`, `P-S3-…`; the files of the
earlier campaign (`S<k>-m<M>.json`, `base-*`, `interf-*`, …) are unchanged (`gen.py` without `--paper`
writes them bit for bit). `gen.make_paper(sid, M, seed, duration, queue)` gives the scenario of a run.
Common to all: ConstantRate ERP-OFDM 24 Mb/s for data and ACKs, beacons and management at 6 Mb/s,
20 dBm, RTS/CTS off and A-MSDU/A-MPDU off (set explicitly on every MAC), MAC queue 100 packets / 500 ms,
`phy` yans except S3 and S8 (spectrum, for the injector). Parameters in `gen.PAPER_PARAMS` and in the
`params` field of each file.

| ID | As implemented |
|---|---|
| S1 | matrix 70 dB between any two nodes (-50 dBm) |
| S2 | S1 + ap2/sta2 at 70 dB: UDP 1400 B sta2 -> ap2 with CSMA, on U[1, 6] s at U[2, 8] Mb/s, off U[2, 8] s; ap2 beacons every 100 TU at DSSS 1 Mb/s (basic rate set of its BSS starts with DsssRate1Mbps; needs yans) |
| S3 | S1 + WaveformGenerator 20 dBm at 70 dB from every node (-50 dBm, above the ED threshold -62 dBm), on U[1, 6] s with period U[0.5, 5] ms and duty U[0.05, 0.3], off U[2, 8] s |
| S4 | S1 with v1-v2 and v3-v4 at 200 dB (M = 2: v1-v2) |
| S5 | lawnmower survey, AP at 6 m, vessels at 1.5 m, lanes 100-800 m, U[1, 2] m/s; TwoRayGround + Nakagami m = 5; fixed 24 Mb/s ERP (no Minstrel, no 11a) |
| S6 | S1 with the AP-vessel loss on a seeded walk in [95, 106] dB (+-1 dB per second, reflected, linear between seconds; `channel.walk`); `preamble_min_rssi_dbm` -90 |
| S7 | trajectories of S5 (same seed) over the sea (`channel.model` `sea`): direct and reflected ray with their phase difference, Fresnel coefficient of sea water (eps_r 70, sigma 5 S/m, vertical polarization), Miller-Brown roughness (sigma_h = 0.3 / sqrt 2 m), heave 0.3 m with period U[2, 8] s per vessel, roll 10 deg with the same period (own seeded phase), half-wave dipole field patterns at both ends |
| S8 | S3 schedule of the same seed with the injector at -72 dBm (92 dB to every node) and the AP-vessel loss 84 dB (-64 dBm) |
| S9 | S2 schedule of the same seed with ap2/sta2 at 102 dB from the fleet nodes (mean -82 dBm) and Nakagami m = 5 power fading per frame on these links (`channel.fading`) |
| q1000 | MAC queue 1000 packets with MaxDelay 60 s on the AP and the vessels (`fleet_queue`) |
| codel | mq root with one CoDel per AC (target 5 ms, interval 100 ms, limit 1000 packets) on the vessels (`aqm`), above a MAC queue of 4 packets (`station_queue`); q100 = the base file |

Parameters of the drafted Section VI that ns-3 does not reproduce as intended, and the values used:

| Item | Drafted | Problem in ns-3 | Implemented |
|---|---|---|---|
| S6 | walk in [95, 106] dB | with the default preamble detection (MinimumRssi -82 dBm, noise figure 7 dB) 24 Mb/s frames decode without error down to -82 dBm (101 dB: PER 0) and nothing is received below it (102 dB: every PPDU dropped, the vessel deassociates after 10 beacons): the walk only switches the link on and off | the same walk with the preamble detection down to -90 dBm (6 Mb/s range of a real receiver): PER of 1500-byte frames 0.002 / 0.04 / 0.48 / 0.99 / 1.0 at 103 / 104 / 105 / 106 / 107 dB, beacons at 6 Mb/s keep the association |
| S8 | injector at -72 dBm, vessels at -50 dBm | SINR 22 dB at the AP: ns-3 decodes every 24 Mb/s frame (PER 0 measured), S8 equals S1 | vessels at -64 dBm at the AP (84 dB): SINR 8 dB during the bursts, 37 % of the frames lost at duty 0.3, no CCA busy from the injector; at -62 dBm (82 dB) the loss is 1.4 % |
| S2, S9 beacons | 1 Mb/s | SpectrumWifiPhy cannot send DSSS in a 20 MHz channel | phy yans for S2 and S9 (the setup text names a spectrum channel model) |
| S9 | second BSS "received at -82 dBm" | a fixed loss puts every frame exactly at MinimumRssi (detection decided by rounding) | mean -82 dBm with Nakagami m = 5 (the fading of S5) per frame and receiver, so the AP and each vessel detect a random part of the frames |
| S7 roll | "reduces the antenna gain by the elevation pattern of a half-wave dipole" | physically small: 10 deg of roll costs at most about 0.2 dB for a horizontal ray | implemented as stated; the heave (two-ray phase) dominates: about 3 dB swings within 12 s on the AP links, two-ray minima (about 4 dB below free space with vertical polarization) 115-175 m from the AP |
| S7 sources | "yamamoto2019rssi" | not verified | the code cites Wang et al. IEEE Access 2018, Yang et al. VTC-Fall 2011 (2 GHz over sea), ITU-R P.527 (sea water), Miller-Brown-Vegh 1984 (roughness); heave 0.3 m, 2-8 s and roll 10 deg are the drafted values |

Smoke test (offline, `--rt=none`, saturating UDP 12 Mb/s per vessel of 1500 B + 200 B downlink, 15 s,
M = 4; IP throughput of phase 3 scaled by Ts/T3; ground truth):

| Scenario | Mb/s mean (p5-p95) | AP RX errors | Observation |
|---|---|---|---|
| S1 (q100, q1000, codel) | 14.21 (13.39-15.03) | 0 | identical PHY with saturated UDP; codel drops 10.4k of 15.2k packets per vessel at the qdisc |
| S2 | 12.61 (9.73-14.53) | 0 | 166 ap2 beacons at 1000 kb/s; the AP decodes 1736 external frames |
| S3 | 13.15 (8.77-14.85) | 1647 | CCA busy share 0.150 |
| S4 | 9.68 (8.58-10.74) | 4481 | hidden-node collisions |
| S5 | 9.78 (8.21-11.49) | 5243 | fading, and hidden vessel pairs beyond about 530 m (TwoRayGround at 1.5 m) |
| S6 | 13.00 (10.74-14.65) | 1812 | errors when the walk passes 103 dB |
| S7 | 12.67 (10.85-14.15) | 2805 | hidden pairs (vessel-vessel two-ray loss up to 117 dB), heave fades |
| S8 | 12.64 (6.68-15.03) | 1736 | CCA busy share 0.105 (S3: 0.150): the injector does not raise the busy time |
| S9 | 13.84 (12.88-14.53) | 239 | 25 external frames decoded, 204 with errors, the rest dropped near the detection limit |

Full-length (182 s, 2 Mb/s per vessel, M = 4): no deassociation in S5-S9.

## Emulator options added (fdt-emu.cc)

| Option or field | Meaning |
|---|---|
| `--shiftS=X` | scenario time 0 at simulator time X (offline replays; overrides `--runStartUs`) |
| `--selftestSet=1,3` | self-test sources on these vessels only |
| `--stepBytes=TS_US:P3OFF_US` | `stepbytes.csv`: per step k and vessel, unique IP packets (and bytes) received without error by the AP, in phase 3 and in the step (by the end of the reception, scenario time) |
| `--phaseLag=TS_US:T1_US:T2_US` | `steplag.csv`: per step the largest lag of phases 1, 2 and 3 (samples every `--phaseLagEveryUs`, 250 us, at every PHY TX start and RX end, and at the phase boundaries), the lag at the start of phase 3 and at the end of the step, catch-up flag (`|lag_end - lag_p3| > --catchupUs`, 1 ms); `summary.json` `phase_lag` |
| `--abortLagUs=1000` | stop the emulator (exit status 3, `ABORT lag` line, `phase_lag.aborted`) when the lag exceeds the value at scenario time >= 0; ns-3's HardLimit (`--hardLimitMs`, 50) stays as a backstop because it applies from the emulator start, when the VMs reconnect (lags of several ms) |
| `--probeLoss=A-B,...:DT_MS` | `lossprobe.csv` of the received power of node pairs (diagnostics; draws from the fading streams) |
| `channel.walk`, `channel.fading` | per-pair loss series and Nakagami-m fading of the matrix model (FdtMatrixLoss; without them the matrix model gives the same results as before: gt.bin of S1-S5 identical) |
| `channel.model: sea` | SeaTwoRayLoss (see the table above; `motion.<node>`: heave_m, period_s, phase_rad, roll_deg, roll_phase_rad) |
| `preamble_min_rssi_dbm`, `preamble_snr_db`, `rx_noise_figure_db` | ThresholdPreambleDetectionModel and noise figure of every PHY |
| `external_bss.beacon_tu`, `external_bss.basic_modes` | beacon interval and basic rate set of the second BSS |
| `fleet_queue`, `station_queue`, `aqm` | per-node MAC queues and the mq + CoDel queue disc (traffic-control layer on the node; the relay passes the UP in the DS field of the queue item, so the AC does not change) |
| `summary.json` | `conf.<node>`: `basic0`, `rts_threshold`, `be_queue_p`, `be_queue_delay_ms`, `qdisc`; `qdisc.<node>`: sent, dropped |

## C(t) by offline subset replays (`emu/replay_subsets.py`, H2 reference)

`python3 emu/replay_subsets.py RUN [--jobs 4]` replays `RUN/scenario.json` offline for each nonempty set
S of vessels (15 at M = 4) with saturating UDP (1500 B, 30 Mb/s per vessel of S), scenario time 0 at
2 s, `--stepBytes`, and computes C_S(k) = sum_{i in S} (bytes3 + h_W frames3) Ts/T3 and
C(k) = min_S C_S(k) (`RUN/csub/c_t.jsonl`: k, t = S0 + k, C, argmin, C_S of every set, saturated,
sat_vessels; `replay.json`). The saturation decision comes from the vessel sampler logs
`RUN/vms/v<i>/outq.jsonl` (`emu/tools/outq.py`: every vessel with forwarded bytes in phase 3 keeps unsent
bytes in its data bridge at every 1-ms sample of phase 3, with samples covering at least 95 % of
phase 3). Cost: a short run of 80 steps at M = 4 (S3, spectrum) took 4-5 s per set offline on this
2-core machine (37 s for the 15 sets with 2 jobs); 1200 steps take about 70 s per set.

## Vessel sampler of SIOCOUTQ (`emu/tools/outq_sampler.c`, `emu/tools/outq.py`)

`fdt_outq --epoch E --ts-us 125000 --t1-us 20000 --t2-us 10000 --port 1884 --port 1883 --out
/tmp/fdt-run/outq.jsonl [--start-step S0 --steps N] [--fifo 20]` samples every 1 ms of the step clock of
the vessel the send queue of the bridge sockets (local ports 1884, 1883) through sock_diag
(`idiag_wqueue` = write_seq - snd_una, the value of SIOCOUTQ, which another process cannot ioctl), plus
notsent_bytes and bytes_acked of tcp_info, keeps the step in memory and writes one JSON line per step
(n, nz, q0, nsd0, qmin, qmax, acked of phase 3, n12/nz12 of phases 1-2, steal_ms of the guest, late_us,
miss). `outq.py sample` is the same sampler in Python; `outq.py saturation RUN` applies the rule.

## Guest clock and CPU (item I9; image rebuild needed)

| Part | Files |
|---|---|
| clock source | kernel command line `clocksource=tsc tsc=reliable` (provision-base.sh); `fdt-clocksource.service` checks it at boot, selects tsc if needed and writes `/run/fdt-clock/clocksource.json` |
| oscillator error | `fdt_clock_schedule.py` (schedule: holds U[30, 120] s and ramps U[60, 300] s between ppm0 - 20 and ppm0 + 20 ppm, NTP loss episodes U[10, 60] s inside given windows or after the warm-up); `fdt-freqshim.so` (preloaded into chronyd by `chrony-freqshim.conf`) adds e(t) to every frequency that chronyd writes and hides it on reads; `fdt-clockerr.service` writes e(t) to the shared page and to the kernel, and drops UDP 123 (nftables table `inet fdt_ntploss`) during the loss episodes; log `/var/log/fdt-clockerr.jsonl`. With `"clock": {"schedule": PATH}` in emu-host.json, `fdt-clockinit` sets only the offset (ppm0 goes into the schedule) |
| chrony | `emu/vm/chrony_conf.py render gs|vessel` (vessel: no makestep, offsets slewed; optional single initial step), `convergence` and `warmup` from the probe's clock.csv (the analysis warm-up of 240 steps = 30 s must cover the largest convergence) |
| CPU | `emu/vm/cpu_profile.py`: cpu.max of the QEMU process of a vessel; assumed ratio 0.25 of an E-core (published single-thread results bracket 0.19-0.32) -> `cpu.max` "1000 4000" (cgroup v2 needs quota and period >= 1 ms; stalls up to 3 ms); `wrap` (systemd-run --user --scope with CPUQuota=25% and CPUQuotaPeriodSec=4ms), `apply` (root), `status` (nr_throttled, throttled_usec), `calibrate` and `check` with `emu/vm/cpu_bench.py` run on the Raspberry Pi 4 and in the VM |

Tested here (no VM can boot in this environment): script syntax, schedule generation and evaluation,
the injector against an in-memory kernel and a simulated chronyd, the shim against a fake libc adjtimex
(the caller writes -2 ppm, the kernel gets +3 ppm with e = +5 ppm, the caller reads -2 ppm back), the
chrony rendering and the convergence/warm-up check on a synthetic clock.csv, the profile arithmetic.

## Platform qualification Q (`emu/experiments/qualify.py`)

`qualify.py manifest --out DIR` writes 60 paper-stack runs (M {2, 4} x {S1, S2, S3, S6, S7} x K {1, 3} x 3;
the same seed for K = 1 and K = 3 of a repetition; K = 1 runs last with `"max_K": 1`; every spec sets
`"abort_lag_us": 0`, and the paper stack passes `--phaseLag` itself). Run with
`runner.py DIR --K 3 --retry 0 --lag-exempt S1 S2 S3 S6 S7` (so that the runner neither lowers K nor
repeats lagging runs during the qualification). `qualify.py analyze RESULTS --manifest DIR/manifest.json
[--json q.json]` applies the run rule (completed, not aborted, lag of every step <= 1 ms), the step rule
(after the warm-up of 240 steps, lag in phases 2-3 < delta, default 1 ms, and no catch-up), reports the
lag per phase (p50/p99/p99.9/max of the per-step maxima, share >= delta) per cell (M, profile, K) and per
method, the invalid-run rates with Fisher exact tests (methods, K = 1 against K = 3), the steal time of
the vessels, and the paired TOST of K = 3 against K = 1 per cell and pooled on gamma (margin 0.005), phi
(0.005) and Pi (0.05), from `RUN/outcomes.json` or from `RUN/vms/gs/dta.jsonl` with
`fleetdt.analysis.run_metrics` (after the warm-up). With 3 pairs per cell (t with 2 degrees of freedom)
the per-cell TOST is equivalent only when the three differences agree within a small fraction of the
margin; the pooled TOST has up to 30 pairs.

Tests: `tests/test_emu_paper_v2.py` (24 tests; the emulator tests run with `FDT_EMU_BIN` or `FDT_NS3`).
