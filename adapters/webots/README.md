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

## Three deliberate departures from the source world

The world derives from `projeto_barco/worlds/Barco_2_0.wbt` in
`lsa-pucrs/boat-digital-twin`. Three things changed, each forced by the paper.

**`basicTimeStep` is 25 ms.** The source left it at the 32 ms default, and
125/32 is not an integer, so `wb_robot_step(125)` would have rounded the DT
frame to something other than the deadline Section IV defines the model by —
quietly, with no error. At 25 ms the frame is exactly five physics steps.
`tests/test_world.c` asserts the division, so this cannot regress.

**The hulls carry DEF names.** The source identified them by display name;
the controller resolves them by DEF, because a display name changes the moment
someone edits the scene tree.

**One controller drives both vessels.** The source ran one controller per hull,
each supervising itself with its index passed through `controllerArgs`. Here
the first hull's controller is the coordinator `S` of Figure 4: it resolves
every DEF, steps every twin, computes `cᵗ` across the fleet and writes every
pose. A controller per hull *cannot* compute `cᵗ`, because no instance would
see more than its own state. The second hull therefore has
`controller "<none>"` — it is a DTI the coordinator drives, not an agent.

## Proof that it renders

![the Jundiá fleet in WeBots](../docs/simulation.jpg)

Captured from a live run at frame 240. Setting `FDT_SHOT` to a path makes the
controller export the 3D view there, and `FDT_SHOT_FRAME` chooses when:

    FDT_SHOT=/tmp/view.jpg FDT_SHOT_FRAME=240 \
      webots --batch --mode=realtime adapters/webots/worlds/jundia_fleet.wbt

That exists because two position bugs in this controller left every counter
green and were caught only by a person looking at the viewport. A picture is
the only assertion that covers a rendering.

`FDT_NO_POSE=1` leaves the hulls wherever the world placed them, which
separates "the twin wrote a wrong pose" from "the camera is pointed elsewhere"
in a single run. Both look identical from the console.

## What is checked, and what is not

`tests/test_world.c` runs in the ordinary suite on any machine and checks the
world against what the controller assumes of it: the DEF names resolve, exactly
one node is a supervisor, the mesh URLs point at files that exist, the frame
divides into whole physics steps, and Section III's furniture — fluid, drag,
one camera per vessel — is present.

That proves the controller and the world still agree about what they are. It
does **not** prove the simulation runs. Neither the controller nor the MQTT
adapter has been through the real toolchain on the machine that wrote them;
`make syntax` type-checks both against the stub headers in `tools/stubs/`,
which catches a typo and cannot catch a wrong assumption about the API.

Claims **C3** and **C8** therefore stay `fronteira` in
`docs/spec/paper-claims.md`. A file that has never been through a linker is not
evidence of integration, and the artefacts being complete and mutually
consistent is a weaker statement than the paper's.

Section V-A's *"running WeBots adds 10% CPU usage for the first boat and less
than 1% for subsequent boats"* is a boundary for a different reason: it is a
property of the renderer, so `make bench` does not reproduce it and does not
pretend to. That is claim **C22**.

## Credit

The world, the hull mesh, its material and texture, the collision mesh and the
obstacle layout come from `lsa-pucrs/boat-digital-twin` (MIT), by Anderson
Domingues and the Jundiá project team. The immersion and drag coefficients are
theirs and were not retuned: they are calibrated against a hull this repository
did not build.
