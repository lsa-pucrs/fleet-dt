# fdt-sim: simulation of larger fleets (addendum A2)

| File | Content |
|---|---|
| `fdt-sim.cc` | ns-3 program: the channel code of `emu/ns3/fdt-emu.cc` with the hosts inside ns-3 (vessel brokers with the regulator core, MQTT bridges over TCP, sources) and the link to the AS driver |
| `CMakeLists.txt`, `build.sh` | build in a separate ns-3 tree (default `~/fleetdt-deploy/emu/sim-ns3/ns-3.48`) with `libfdtreg.a` (fdt_reg.c, fdt_dtp2.c, fdt_common.c, cJSON) |
| `third_party/cjson` | cJSON 1.7.19 (MIT license) |
| `simrun.py` | one run: sources and stub schedules, fdt-sim, ASCore at each phase 2, run directory in the layout of the emulation, gtstep and metrics |
| `simcampaign.py` | the runs of a campaign directory, K at a time, pinned to CPUs 14 and 15 |

Design, calibration, validation and hypothesis: `emu/experiments/addendum/ADDENDUM.md`, section A2.

```
NS3=~/fleetdt-deploy/emu/sim-ns3/ns-3.48 sh emu/sim/build.sh
python3 emu/experiments/addendum/scale.py e6b --calib CAL.json --out DIR
python3 emu/sim/simcampaign.py DIR --K 2 --cpus 14,15
```
