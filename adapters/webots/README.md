# WeBots simulation

Section IV: *"A custom module within WeBots implements δ in C language and
carries out the dynamics of the model."* Section III: *"A 3D model derived from
the DTP resides in a WeBots world as a DTI inside the VE."*

Both halves are here.

    export WEBOTS_HOME=/usr/local/webots
    make webots
    webots adapters/webots/worlds/jundia_fleet.wbt

Without `WEBOTS_HOME` the target skips with a notice; the core library never
links the WeBots controller library.

## Layout

This directory is a WeBots project, in the layout the simulator expects.

| Path | What |
|---|---|
| `worlds/jundia_fleet.wbt` | the world: water, two hulls, obstacles |
| `controllers/fdt_controller/fdt_controller.c` | the module running δ |
| `3d_models/drone_boat/full_model/` | hull mesh, material and texture |
| `3d_models/drone_boat/bounding_box/` | collision mesh |

## What the world contains

- a **`Fluid`** node with `ImmersionProperties` and drag coefficients per hull.
  This is the fluid simulator Section III lists as sharing the DTE with WeBots;
  `include/fleet_dt/dte.h` is what ties simulations like it to the same tick
  the twin runs on.
- two `Robot` nodes, `DEF PINTADO` and `DEF TILAPIA`, after the boats of
  Figure 1, each rendering `boat_model.obj` through a `CadShape`.
- a `Camera` on each hull. The image never travels over MQTT: Section III
  routes the feed over RTSP, so the wire codec carries a presence flag and
  `adapters/rtsp/` carries the frame.
- four `OilBarrel` obstacles, which is what the obstacle-detection application
  of the Application Space has to see.

## Three departures from the source world

The world derives from `projeto_barco/worlds/Barco_2_0.wbt` in
`lsa-pucrs/boat-digital-twin`. Three things changed, each forced by the paper.

**`basicTimeStep` is 25 ms.** The source used the 32 ms default, and 125/32 is
not an integer. At 25 ms the 125 ms frame of Section IV is exactly five physics
steps, so `wb_robot_step(125)` lands on the deadline the model is defined by.
`tests/test_world.c` asserts the division, so it stays that way.

**The hulls carry DEF names.** The source identified them by display name;
the controller resolves them by DEF, because a display name changes the moment
someone edits the scene tree.

**One controller drives both vessels.** The source ran one controller per hull,
each supervising itself with its index passed through `controllerArgs`. Here
the first hull's controller is the coordinator `S` of Figure 4: it resolves
every DEF, steps every twin, computes `cᵗ` across the fleet and writes every
pose. A controller per hull *cannot* compute `cᵗ`, because no instance would
see more than its own state. The second hull therefore has
`controller "<none>"`. It is a DTI the coordinator drives, not an agent.

## The rendered fleet

![the Jundiá fleet in WeBots](../../docs/simulation.jpg)

Both hulls of the fleet, captured from a live run at frame 240. Setting
`FDT_SHOT` to a path makes the controller export the 3D view there, and
`FDT_SHOT_FRAME` chooses when:

    FDT_SHOT=/tmp/view.jpg FDT_SHOT_FRAME=240 \
      webots --batch --mode=realtime adapters/webots/worlds/jundia_fleet.wbt

Section I(iii) states that the operator gets a 3D visual reference. The image
above shows it.

`FDT_NO_POSE=1` leaves the hulls wherever the world placed them, which
separates what the twin writes from what the world file already said, in a
single run.

The controller prints the distance between hulls on every frame. Fleet
geometry belongs to the telemetry rather than to the world file, so the local
tangent plane is anchored once for the whole fleet, in `fdt_geo_offset`
([`geo.h`](../../include/fleet_dt/geo.h)), and the spacing the injectors
produce stays independent of the station-keeping sweep. A digital twin of a fleet
has to show where the vessels are relative to each other.

## What each check covers

`tests/test_world.c` runs in the ordinary suite on any machine and checks the
world against what the controller assumes of it: the DEF names resolve, exactly
one node is a supervisor, the mesh URLs point at files that exist, the frame
divides into whole physics steps, and the world holds what Section III
describes: fluid, drag, and one camera per vessel. It runs with or without the SDK, so the
controller and the world stay in agreement on any checkout.

`make syntax` type-checks the controller against the stub headers in
`tools/stubs/` where WeBots is absent, which covers the API surface the
controller uses. Running the simulation is what covers the rest, and
`make webots` is the target for that.

**C3** and **C8** run against the real toolchain: on 2026-08-17, with WeBots
R2025a installed, `make webots` compiled the controller against the SDK and the
world ran under the coordinator for 674,360 frames.

**C22**, *"running WeBots adds 10% CPU usage for the first boat and less than
1% for subsequent boats"*, is measured by `tools/bench/bench_webots_cpu.c`
across three worlds that differ only in vessel count. The first hull costs two
to three times the second, because the renderer, the physics world and the
fluid run once regardless of vessel count. The absolute figures belong to the
host that runs the benchmark, and the report labels them that way.

## Credit

The world, the hull mesh, its material and texture, the collision mesh and the
obstacle layout come from `lsa-pucrs/boat-digital-twin` (MIT), by Anderson
Domingues and the Jundiá project team. The immersion and drag coefficients are
theirs and were not retuned: they are calibrated against a hull this repository
did not build.
