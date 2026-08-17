# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
this project adheres to [SemVer](https://semver.org/).

## [Unreleased]

### Added

The model of Section IV:

- model types for equation (1) and the 21 entries of Table I, with the state
  pinned at 48 bytes by static assertion
- the bounded state frame queue, and the `48d` bound the paper computes from it
- δ, δᵉ and π over a window of the `n` most recent states; equation (2) is the
  `n == 1` case, so there is no separate δ
- the non-autonomous vessel, where actuation is absorbed by the state
- the fleet aggregate of equations (4), (5) and (6), with `cᵗ` delivered to
  every δᵉ and every π rather than only stored
- the `Bᵗ` store and the coordinator `S` of Figure 4, computing `cᵗ` in the
  same step in which it distributes `gᵗ`
- the 125 ms pacer with absolute deadlines
- the feasibility predicate, measured per vessel over δ compute time only

The architecture of Section III:

- the transport seam, with an in-process loopback that separates publication
  from delivery
- a fixed-width little-endian wire codec for `Iᵗ`, `Bᵗ` and `Aᵗ`
- a wire envelope carrying the per-vessel sequence number Table I does not
- detection and counting of the partial and double frame updates Section V-B
  reports and leaves open
- the bandwidth regulators, decimating publication to the DT rate while the
  sensor path keeps its own
- the link-budget model, with both readings of the Section V-A figure
- the DTE registry, ticking parallel simulations off one clock
- MQTT transport over libmosquitto, and the bridge-mode broker configuration
- the Ardupilot ingest mapping for all 21 Table I entries, and `Aᵗ` back out as
  RC channel overrides
- the HSDT camera boundary, kept off the MQTT path
- the WeBots controller running δ at the simulation tick

The validation of Section V:

- deterministic synthetic telemetry injectors
- five benchmarks — regulator, link budget, scaling, latency, jitter — each
  writing a text report, a CSV series and an SVG chart
- a dependency-free SVG plotter, so a measurement can be regenerated on a bare
  toolchain
- claim coverage and results assembly, generating `docs/RESULTS.md`
- two runnable examples: a coordinated fleet frame, and the filtered/raw
  two-path arrangement of Section V-A

The simulation, verified against the real toolchain:

- the WeBots project — world, hull meshes, material and texture — from
  `lsa-pucrs/boat-digital-twin`, rewired to the model of Section IV
- three worlds differing only in vessel count, so the CPU claim of Section V-A
  can be measured as a difference rather than asserted
- `fdt_geo_offset`, the geodetic-to-local mapping, extracted from the WeBots
  controller after it placed every vessel 5700 km outside the world
- `FDT_SHOT` and `FDT_SHOT_FRAME`, which export the 3D view from a running
  controller, because a rendering cannot be asserted any other way
- `make syntax`, type-checking the SDK adapters against stub headers on a
  machine that has neither
- `make mqtt-test`, a round trip through a real mosquitto broker
- `make webots`, building the controller against a real WeBots install
- header dependency tracking, so editing a header rebuilds what included it

Documentation:

- the claim inventory, one row per falsifiable claim with the artefact that
  discharges it, including the seven items Section VI declares future work
- the paper-to-code map for revision `-10`
- six recorded ambiguities and findings in the manuscript, five worth fixing,
  including that the 48-byte state of Section IV quantises position to between
  0.2 and 0.4 m at the Jundiá coordinates — a third of a hull length
- a captured frame of the running simulation, since two position bugs left
  every counter green and were caught only by looking
