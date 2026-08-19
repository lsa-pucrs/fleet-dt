# Examples

Two runnable programs. Both build with `make examples` and need nothing
installed.

## `daemon.c`, a fleet frame end to end

    ./examples/daemon

Two boats stepping at 125 ms for 40 frames under a coordinator that computes
`cᵗ` and distributes `gᵗ`, with a second simulation registered on the same DTE
tick. The dynamics are a placeholder; the file exists to show the loop and the
wiring.

Expect `overruns=0` and `feasible=1`. An overrun in a loop this small reports
scheduling on the host.

The window is **4 deep, not 1**. Section V-A ties the window to proactive
operation, "as in model predictive control (MPC)", and depth 1 produces no
such term. The `trend` column is the
term the depth adds, the mean yaw change per frame across the window, and it
reads zero at depth 1 by construction. Each twin is therefore seeded four times
before the first step, because equation (3) needs the queue to already hold `n`
states.

`queue bytes per vessel=768` is `48 × 16`: the `48d` bound of Section IV at
this example's capacity.

`worst_delta` is the feasibility predicate of Section IV, the time to compute
δ. It is **not** the actuation latency. Section V-A observes actuation arriving
late even when δ is feasible, because it travels back through the network;
`tools/bench/bench_latency.c` measures the two apart.

## `two_paths.c`, the filtered state and the raw actuation path

    ./examples/two_paths

Section V-A: "using a Kalman filter to generate `Bᵢᵗ`. Data from sensors
(`Iᵢᵗ`) were used unfiltered to achieve the lowest possible latency from
sensing to the actuation path."

Both paths in one frame. `δᵉ` runs a scalar Kalman filter over yaw and produces
the state the operator sees; `π` decides from the **raw** input stashed in the
twin's context, never from the filtered state. The printed `lag` column is the
gap between them, and it is lag rather than error: the same motion, arriving
later.

The obvious implementation of `π` reads its `b` argument and silently adds the
filter's delay to the control loop. This one does not, which is the whole
point of the file.

---

Both files derive from `dt-daemon/daemon.c` by Anderson Domingues in
`lsa-pucrs/boat-digital-twin`. That repository is private, so this is a credit,
not a link a reader can follow.
