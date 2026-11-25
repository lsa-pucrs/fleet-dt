# Examples

Two runnable programs. Both build with `make examples` and need nothing
installed.

## `daemon.c`, a fleet frame end to end

    ./examples/daemon

Two boats stepping at 125 ms for 40 frames under a coordinator that computes
`cᵗ` and distributes `gᵗ`. `cᵗ` is the inter-vessel distance Section IV gives
as its example, computed over the fleet window of equation (6): the spacing
from this frame's `Bᵗ` and the closing speed against the oldest `cᵗ` the
window holds, which set a throttle ceiling. A second simulation shares the
same DTE tick. The dynamics are a placeholder; the file exists to show the loop and the
wiring.

Expect `overruns=0` and `feasible=1`. An overrun in a loop this small reports
scheduling on the host.

The window is **4 deep, not 1**. Section IV ties the window to "estimating
techniques based on temporal series", and Section V-A to proactive operation,
"as in model predictive control (MPC)". Each twin estimates
x̂ₖ₊d = f(xₖ, …, xₖ₋ₙ₊₁) from its window (`fdt_predict`, a least-squares line
over the 4 states) with d = 2 ticks, the delay a command takes to reach the
boat (talk, backup slide 6), and `π` steers on x̂ₖ₊d rather than on `Bᵗ`. The
`x^k+d` column is that estimate; at depth 1 no line can be fitted. Each twin
is therefore seeded four times before the first step, because equation (3)
needs the queue to already hold `n` states.

`FDT_DB=fleet.db ./examples/daemon` also stores every `Bᵗ` and `Aᵗ` in the
database of Figure 4 ([`db.h`](../include/fleet_dt/db.h)).

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

Both paths in one frame. `δᵉ` runs the Kalman filter of
[`kalman.h`](../include/fleet_dt/kalman.h) (position and velocity from GPS,
roll and pitch from gravity, yaw from the gyro and the tilt-compensated
magnetometer) and produces the `Bᵗ` the operator sees; `π` decides from the **raw** input stashed in the
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
