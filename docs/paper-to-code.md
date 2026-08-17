# Paper to code

Manuscript tracked: revision **`-10`**, pinned in
[`include/fleet_dt/version.h`](../include/fleet_dt/version.h) as
`FDT_PAPER_REVISION` and checked by `make test`. The model is **Section IV**
and the equations run **(1)** through **(6)**.

That pin is not decoration. An earlier revision had δ and π as separate
equations, which shifted every reference from (3) onward; a map that did not
say which revision it tracked went stale without anyone noticing. When the
manuscript moves, the constant moves, the test fails, and this table gets
re-checked.

## Section IV — the model

| Paper | Meaning | Type or function | File |
|---|---|---|---|
| `Iᵢᵗ` | environment input, Table I | `fdt_input_t` | [`model.h`](../include/fleet_dt/model.h) |
| `Bᵢᵗ` | twin state, eq. (1) | `fdt_state_t` | [`model.h`](../include/fleet_dt/model.h) |
| `Aᵢᵗ` | actuation, eq. (2) | `fdt_actuation_t` | [`model.h`](../include/fleet_dt/model.h) |
| `gᵢᵗ` | mission goal | `fdt_goal_t` | [`model.h`](../include/fleet_dt/model.h) |
| δ | transition, eq. (2) | `fdt_twin_step` with `n == 1` | [`transition.c`](../src/transition.c) |
| π | decision, eq. (2) | `fdt_pi_fn` | [`transition.h`](../include/fleet_dt/transition.h) |
| δᵉ | extended transition, eq. (3) | `fdt_delta_e_fn` over the `n` most recent states | [`transition.h`](../include/fleet_dt/transition.h) |
| `[Bᵗ⁻¹; Bᵗ⁻ⁿ]` | the window of the bracket | `fdt_window_at(q, n, k)`, `k == 0` is `Bᵗ⁻¹` | [`transition.c`](../src/transition.c) |
| `Bᵢ¹` | known starting state | `fdt_twin_seed` | [`transition.c`](../src/transition.c) |
| queue of state frames | holds `fdt_state_t` directly | `fdt_queue_t` | [`queue.h`](../include/fleet_dt/queue.h) |
| `48d` bytes | queue bound at depth `d` | `fdt_queue_bytes` | [`queue.c`](../src/queue.c) |
| `Aᵢᵗ ⊆ Bᵢᵗ` | non-autonomous vessel | `fdt_twin_init_passive` | [`transition.c`](../src/transition.c) |
| feasibility | δ inside \|t_k − t_{k−1}\| | `fdt_feas_t` | [`feasibility.h`](../include/fleet_dt/feasibility.h) |
| `Fᵗ` | fleet, eq. (4) | `fdt_fleet_t` | [`fleet.h`](../include/fleet_dt/fleet.h) |
| Δ | fleet transition, eq. (5) | `fdt_fleet_step` | [`fleet.c`](../src/fleet.c) |
| Δᵉ | extended fleet transition, eq. (6) | `fdt_fleet_step` with `n > 1` | [`fleet.c`](../src/fleet.c) |
| `cᵗ` | fleet context, eq. (4) | `fdt_fleet_t.ctx`, delivered to every δᵉ and every π | [`fleet.h`](../include/fleet_dt/fleet.h) |
| `S` | coordinator, Fig. 4 | `fdt_coord_t` | [`coordinator.h`](../include/fleet_dt/coordinator.h) |
| `Bᵗ` database | the cylinder of Fig. 4 | `fdt_store_t` | [`coordinator.h`](../include/fleet_dt/coordinator.h) |
| Δt | frame period, 125 ms | `FDT_TICK_NS` | [`tick.h`](../include/fleet_dt/tick.h) |
| φ, λ onto metres | local tangent plane | `fdt_geo_offset` | [`geo.h`](../include/fleet_dt/geo.h) |

## Section III — the architecture

| Paper | Meaning | Type or function | File |
|---|---|---|---|
| MQTT clients and brokers | the network seam | `fdt_transport_t` | [`transport.h`](../include/fleet_dt/transport.h) |
| brokers in bridge mode | store-and-forward across an outage | `boat.conf`, `ground.conf` | [`config/mosquitto/`](../config/mosquitto/) |
| bandwidth regulators | publication decimated to the DT rate | `fdt_reg_t` | [`regulator.h`](../include/fleet_dt/regulator.h) |
| LSDT | small packets on MQTT | `fdt_stream_t` profiles | [`bench_bandwidth.c`](../tools/bench/bench_bandwidth.c) |
| HSDT | camera feed, off MQTT | `fdt_rtsp_t` | [`fdt_rtsp.h`](../adapters/rtsp/fdt_rtsp.h) |
| Ardupilot / NAVIO2 | sensor source | `fdt_mav_ingest_t` | [`fdt_mavlink.h`](../adapters/mavlink/fdt_mavlink.h) |
| WeBots module running δ | the DTI inside the VE | controller `main` | [`fdt_controller.c`](../adapters/webots/controllers/fdt_controller/fdt_controller.c) |
| 3D model in a WeBots world | the VE of Section III | `DEF PINTADO`, `DEF TILAPIA` | [`jundia_fleet.wbt`](../adapters/webots/worlds/jundia_fleet.wbt) |
| fluid simulator | sharing the DTE | `Fluid` node | [`jundia_fleet.wbt`](../adapters/webots/worlds/jundia_fleet.wbt) |
| DTE with parallel simulations | one tick, many simulations | `fdt_dte_t` | [`dte.h`](../include/fleet_dt/dte.h) |
| MCS | out of scope, per Section IV | goal source in `fdt_plan_fn` | [`coordinator.h`](../include/fleet_dt/coordinator.h) |

## Section V — the validation

| Paper | Meaning | Artefact |
|---|---|---|
| injectors publishing synthetic telemetry | the fleet campaign | [`tools/injector/`](../tools/injector/) |
| partial and double frame updates | the open pathology | [`framesync.h`](../include/fleet_dt/framesync.h) |
| under 1 % CPU per DTI | scaling | [`bench_scale.c`](../tools/bench/bench_scale.c) |
| bandwidth increase | link budget | [`bench_bandwidth.c`](../tools/bench/bench_bandwidth.c) |
| δ feasible, actuation late | two measurements | [`bench_latency.c`](../tools/bench/bench_latency.c) |
| Kalman for `Bᵢᵗ`, raw `Iᵢᵗ` for actuation | two paths | [`examples/two_paths.c`](../examples/two_paths.c) |

## Three things a reader will look for and not find

**There is no `fdt_delta`.** `fdt_twin_step` runs δᵉ over the window on every
call, and equation (2) is what that call does when `n == 1`. A separate δ would
be the same code with a constant baked in.

**There is no frame type.** Equation (1) declares twelve variables and no time
field, so a queued entry *is* a state. That is what makes `fdt_queue_bytes(d)`
exactly `48d`; a wrapper carrying a timestamp would make the paper's bound
false. Sequencing lives in the wire envelope
([`envelope.h`](../include/fleet_dt/envelope.h)), where Section V-B's open
question actually is.

**No signature takes a clock.** `fdt_twin_seed`, `fdt_twin_step` and
`fdt_fleet_step` take no time argument, and `fdt_input_t` has no timestamp
field, because Table I lists none among its 21 entries. The 125 ms pacer in
[`tick.h`](../include/fleet_dt/tick.h) is the runtime that drives frames; it is
not part of the model.

## Where the manuscript is ambiguous

Five places where the code had to choose, each recorded with its reasoning in
[`docs/spec/paper-claims.md`](spec/paper-claims.md), section H:

1. the δᵉ window — the bracket says the `n` most recent states, the trailing
   `i ≤ j ≤ t−1` uses indices that appear nowhere in the equation;
2. equation (3) opens with subscript `i` and switches to `k` mid-line;
3. Table I gives attitude angles as "rad or deg" without choosing, while the
   rates beside them are rad/s;
4. `Bᵢ¹` is called the initial state, but the recurrence needs a `t−1` before
   the first step;
5. "the bandwidth usage increased < 1%" does not reconcile with absolute
   occupancy for the payload and rate the same sentence gives.

Items 1, 2, 3 and 5 are worth fixing in the manuscript.

## Origin

Parts of this repository come from `lsa-pucrs/boat-digital-twin` (MIT), by
Anderson Domingues and the Jundiá project team:

- `fdt_state_t` derives from `dt-daemon/include/boat.h`;
- `examples/daemon.c` from `dt-daemon/daemon.c`;
- the WeBots world, the hull and collision meshes, the material and texture,
  and the immersion and drag coefficients from `projeto_barco/`.

That repository is private. The assets above are redistributed here under its
licence, with the changes to the world noted in
[`adapters/webots/README.md`](../adapters/webots/README.md); the repository itself
remains a credit rather than a link a reader can follow.
