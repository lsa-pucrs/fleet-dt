# Claim map

Every statement the paper makes about the system, and the file in this
repository that carries it.

Manuscript tracked: revision **`-10`**, pinned in
[`include/fleet_dt/version.h`](../include/fleet_dt/version.h) as
`FDT_PAPER_REVISION`. The model is **Section IV** and the equations run
**(1)** through **(6)**.

> A. R. P. Domingues, C. J. D. Silva, F. da R. Lui, J. Maia, R. S. Cenço,
> C. A. M. Marcon, F. G. Moraes. *A Digital Twin Model and Architecture for
> Monitoring and Controlling Fleets of Autonomous Unmanned Surface Vehicles.*
> ICECS 2026.

## The manuscript at a glance

| Section | Title |
|---|---|
| I | Introduction and Related Work |
| II | The Physical Unit: Jundiá Boat |
| III | The Digital Twin Architecture |
| **IV** | **The Digital Twin Model**, equations (1)–(6) |
| V | Validation, Results, and Discussion (A. Vessel DTI, B. Fleet DT) |
| VI | Conclusion |

| Eq | Content |
|---|---|
| (1) | the matrices `Bᵢᵗ`, `Iᵢᵗ`, `Aᵢᵗ` |
| (2) | `Bᵢᵗ = δ(Bᵢᵗ⁻¹, Iᵢᵗ⁻¹, gᵢᵗ⁻¹)` and `Aᵢᵗ = π(Bᵢᵗ, gᵢᵗ)` |
| (3) | `Bᵢᵗ = δᵉ([Bᵢᵗ⁻¹; Bᵢᵗ⁻ⁿ], I_kᵗ⁻¹, g_kᵗ⁻¹)` |
| (4) | `Fᵗ = [Bᵗ  δᵉ  cᵗ]` |
| (5) | `Fᵗ = Δ(Fᵗ⁻¹, Iᵗ⁻¹)` |
| (6) | `Fᵗ = Δᵉ([Fᵗ⁻¹; Fᵗ⁻ⁿ], Iᵗ⁻¹)` |

`make report` walks this map, checks that every artefact named below is
present, and writes the coverage table of
[`docs/RESULTS.md`](RESULTS.md).

## A. Abstract

| id | Claim | § | Where it lives |
|---|---|---|---|
| C1 | "a fleet-level Digital Twin model and architecture for AUSVs" | Abstract | roll-up of C2 through C19 |
| C2 | "The architecture relies on the MQTT protocol" | Abstract, III | `transport.h` and its loopback, `adapters/mqtt/`, round trip in `make mqtt-test` |
| C3 | "integrates the Ardupilot firmware, the WeBots simulator, and other simulation applications into a single system" | Abstract | `adapters/mavlink/`, `adapters/webots/` |
| C4 | "Link-budget modeling is validated in the context of the Jundiá Project's fleet" | Abstract, V-A | `include/fleet_dt/linkbudget.h`, `tools/bench/bench_bandwidth.c` |
| C5 | "introduces bandwidth regulators that guarantee QoS while maximizing the use of shared wireless links" | Abstract, III | `include/fleet_dt/regulator.h`, `tests/test_regulator.c` |

## B. The three features of Section I

| id | Claim | Where it lives |
|---|---|---|
| C6 | "(i) it models fleets as a DTA composed of DTIs per-vessel" | `fdt_fleet_t` over `fdt_twin_t` |
| C7 | "(ii) it supports running multiple simulations in parallel in the same DTE" | `src/dte.c`, simulations bound to one tick |
| C8 | "(iii) it provides a near-real-time 3D visual reference for the mission operator" | the WeBots world rendered in `--mode=realtime`, the controller writing pose every 125 ms |

## C. The architecture of Section III

| id | Claim | Where it lives |
|---|---|---|
| C9 | brokers "connected in bridge mode to avoid service interruption during temporary connection instability" | `config/mosquitto/`, partition test |
| C10 | LSDT: small packets, "e.g., 8 bytes per IMU axis", against 100 Mbps available | the LSDT profile through `linkbudget` |
| C11 | HSDT: "Separating the camera feed from the MQTT infrastructure reduced latency while improving the DTI's response time" | `adapters/rtsp/`; the wire codec never carries an image |
| C12 | regulators "overcome this problem by dropping the number of samples in the MQTT client", while "real sensors continue sampling at their own pace" | `regulator.h`, two-path test |

## D. The model of Section IV

| id | Claim | Where it lives |
|---|---|---|
| C13 | "The simulation frequency is 8 Hz, i.e. δ is a hard real-time task with deadline of 125 ms" | `FDT_TICK_NS`, `tick.h` |
| C14 | "a state (Bᵢᵗ) occupies 12 floating point values in memory, translating to 48 bytes"; "the queue would grow by 23 KB per minute elapsed, per vessel" | static assertion `sizeof(fdt_state_t) == 48`, `fdt_queue_bytes(d) == 48d`, 23040 B/min at 8 Hz |
| C15 | "The DTI is *feasible* only if δ can be computed in less than \|t_k − t_{k−1}\| for any arbitrary k" | `feasibility.h`, timed per vessel per frame |
| C16 | "In a non-autonomous vehicle, actuation is absorbed by the state, i.e. `Aᵢᵗ ⊆ Bᵢᵗ`" | `fdt_twin_init_passive` |
| C17 | "A fleet DT is *homogeneous* when δᵉ is the same for all vessels. Oppositely, heterogeneous fleet DTs require indexing δᵉ" | one δᵉ pointer per twin |
| C18 | "The coordinator (S) computes cᵗ from the vessel states it receives, in the same step in which it distributes gᵢᵗ" | `coordinator.h` and the `Bᵗ` store of Figure 4 |
| C19 | Table I fixes 21 entries of `Iᵗ`; rates in rad/s, angles in "rad or deg" | `model.h`, every field named with its unit |

## E. The validation of Section V

| id | Claim | Where it lives |
|---|---|---|
| C20 | "Due to the size of packets (48 KB payload plus camera feed frame), the bandwidth usage increased < 1% for an update window of 125 ms" | `tools/bench/bench_bandwidth.c` |
| C21 | "MQTT introduced no notable latency, nor did WeBots' visual feedback (3D model) suffer from stuttering" | `tools/bench/bench_jitter.c` |
| C22 | "running WeBots adds 10% CPU usage for the first boat and less than 1% for subsequent boats" | `tools/bench/bench_webots_cpu.c` over three worlds that differ only in vessel count |
| C23 | "Running δ in less than 125 ms is feasible. However, actuation is delivered late to the boat, as it has to travel back through the network" | two measurements: δ time in `feasibility`, round trip in `bench_latency.c` |
| C24 | "we added a range of states to δᵉ, thereby enabling proactive operation, as in model predictive control (MPC)" | `examples/daemon.c`, window depth 4 |
| C25 | "the resources required to add more DTIs to the fleet are negligible (< 1% CPU usage per DTI)" | `tools/bench/bench_scale.c` |
| C26 | "hard-programming injectors to inject packets periodically could not keep the simulation pace for larger fleets (> 25 boats, same computer model)" | `tools/injector/`; the DTI ceiling and the injector ceiling are reported as two numbers |
| C27 | "the state of some boats was *partially* updated... the state of some boats was updated twice within the same simulation frame" | `src/framesync.c`, fed by the envelope sequence number |
| C28 | "Telemetry was collected from a real boat and compared to the pose and attitude estimation, using a Kalman filter to generate Bᵢᵗ. Data from sensors (Iᵢᵗ) were used unfiltered to achieve the lowest possible latency from sensing to the actuation path" | `examples/two_paths.c` |

## F. What Section VI names as future work

These belong to the paper's own roadmap. The repository implements the system
the paper describes, so they are outside its scope.

| id | Named as future work | § |
|---|---|---|
| D1 | "there is no *formal* connection between the input model and the state transition function δᵉ" | VI |
| D2 | describing fluids and ML in the model, "at least to synchronize them with the DTI pace (simulation tick)" | VI |
| D3 | "Using MQTT to transmit telemetry data from within the firmware" | VI |
| D4 | regulators "directly in the firmware or the broker, e.g., as a QoS/real-time rule" | VI |
| D5 | "adopting the real-time transport protocol *under* MQTT" (RTP, ref. [18]) | VI |
| D6 | "Dropping late packets at the receiver" | VI |
| D7 | "Field validation campaign for the DTI fleet is on schedule at the time of writing"; the fleet result of Section V is HILS | V |

## G. Quantities that are easy to conflate

Four pairs share a name or a digit and mean different things. Each is reported
on its own row, in its own units.

1. **48 bytes and 48 KB.** The state of Section IV occupies 48 **bytes**. The
   packet payload of Section V-A is 48 **KB**. Independent quantities; the
   bandwidth report prints both.
2. **The ~25-boat ceiling belongs to the injectors.** Section V-B attributes it
   to the injectors, not to the DTI. `bench_scale` reports the two ceilings
   separately.
3. **δ compute time is not actuation latency.** Section V-A has δ feasible
   inside 125 ms *and* actuation arriving late, because actuation travels back
   through the network. Two benchmarks, never summed.
4. **Timestamps live in the envelope, not in the model.** Table I lists no time
   field, so the model types carry none. The wire envelope carries a sequence
   number, which is what makes the partial and double updates of C27
   observable.

## H. Decisions the implementation makes

Where the manuscript leaves a design choice open, this is the reading the code
takes, so that a reader can follow the code back to a line of the paper.

1. **The δᵉ window takes `n`.** Section IV: "The number of states to observe
   (n) is purely a design decision", and the `48d` bound is over depth, so the
   primary API takes `n`, meaning the `n` most recent states.
2. **Angles in degrees, rates in rad/s.** Table I gives φ, θ, ψ as "rad or
   deg"; the fields are named `roll_deg`, `pitch_deg`, `yaw_deg` beside
   `roll_rate_rps`, `pitch_rate_rps`, `yaw_rate_rps`, following the reference
   `boat.h`. Any δᵉ integrating a rate into an angle therefore crosses units
   explicitly.
3. **`Bᵢ¹` is seeded, not computed.** Section IV calls it "a known starting
   state", so the code treats the initial state as a boundary condition rather
   than as an output of δᵉ.
4. **The link budget prints both readings of the Section V-A figure.**
   `fdt_link_utilization` gives the payload as a share of the whole link;
   `fdt_link_increase` gives its growth over the traffic already flowing. The
   benchmark prints them side by side and labels each.
5. **`cᵗ` is an input and an output of the same frame.** Equation (4) places it
   inside `Fᵗ`, and Section IV has `S` computing it "in the same step in which
   it distributes gᵢᵗ".

## I. Position resolution

The 48-byte state of equation (1) is 12 floats, so φ and λ are `float32`. At
the Jundiá coordinates, 30.05° S and 51.17° W, one ULP is:

| axis | ULP | in metres |
|---|---|---|
| longitude | 3.81 × 10⁻⁶ ° | 0.368 m |
| latitude | 1.91 × 10⁻⁶ ° | 0.212 m |

`tests/test_geo.c` records these figures, and the WeBots controller prints the
separation it achieves between hulls. The hull of Section II is 1.2 m, so a
formation is represented to roughly a third of a hull length.

## Origin

Parts of this repository come from `lsa-pucrs/boat-digital-twin` (MIT), by
Anderson Domingues and the Jundiá project team: `fdt_state_t` derives from
`dt-daemon/include/boat.h`, `examples/daemon.c` from `dt-daemon/daemon.c`, and
the WeBots world, hull and collision meshes, material, texture, immersion and
drag coefficients from `projeto_barco/`. That repository is private, so this is
a credit rather than a link a reader can follow.
