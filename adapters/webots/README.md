# WeBots controller

Section IV: *"A custom module within WeBots implements δ in C language and
carries out the dynamics of the model."* `fdt_webots_controller.c` is that
module.

    export WEBOTS_HOME=/usr/local/webots
    make webots

Without `WEBOTS_HOME` the target skips with a notice; the core library never
links the WeBots controller library.

## The world it expects

A minimal world with one `Robot` node per vessel, each with `supervisor TRUE`
and a `DEF` name matching `VESSEL_DEF` in the controller:

```
DEF PINTADO Robot {
  supervisor TRUE
  controller "fdt_controller"
  translation 0 0 0
  rotation 0 1 0 0
  children [ ... hull geometry ... ]
}
DEF TILAPIA Robot { ... }
```

The controller writes each twin's state into `translation` and `rotation`
through the supervisor API. That is Section I feature (iii) — the near-real-time
3D reference for the mission operator — and it is written from the state the
coordinator produced in the same frame, not from a state fetched afterwards.

## One tick, not two

`wb_robot_step()` is called with `FDT_TICK_NS / 1e6`, so the simulator's step
and the model's frame are the same 125 ms read from the same constant. If they
were configured separately they would drift, and the 3D view would render an
instant other than the one just computed — which is the difference between a
near-real-time reference and an approximate one.

## What this cannot measure

Section V-A reports that *"running WeBots adds 10% CPU usage for the first boat
and less than 1% for subsequent boats"*. That is a property of the renderer, so
`make bench` does not reproduce it and does not pretend to. It is recorded as
claim **C22, boundary** in `docs/spec/paper-claims.md`.

The same applies to the second half of C21, stuttering in the 3D feedback.
What `tools/bench/bench_jitter.c` does establish is the precondition: the frame
clock holds while a full fleet publishes, decodes and steps underneath it, so
nothing upstream of the renderer is losing the deadline.

## Camera feed

The controller receives telemetry over MQTT but **not** the camera. Section III:
*"receiving input from the MQTT infrastructure (except for the camera feed,
directly driven by the RTSP client in MCS)"*. The boundary is
`adapters/rtsp/fdt_rtsp.h`, and the wire codec carries a presence flag where an
image would be.
