# Changelog

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
this project adheres to [SemVer](https://semver.org/).

## [Unreleased]

### Added

- The Ardupilot link of Figure 3, without a MAVLink library:
  `adapters/mavlink/fdt_mavlink_wire.c` frames MAVLink 1 and 2 (CRC-16
  with CRC_EXTRA, signed frames, trailing-zero truncation), folds
  SCALED_IMU2, GPS_RAW_INT, LOCAL_POSITION_NED, SCALED_PRESSURE and
  BATTERY_STATUS into `Iᵗ`, and encodes `Aᵗ` as RC_CHANNELS_OVERRIDE
  (Rover: steering on channel 1, throttle on 3) and a ground-station
  HEARTBEAT. `fdt_mavlink_udp.c` runs it over UDP, answering the address
  the autopilot spoke from. `tests/test_mavlink_wire.c` checks it against
  frames pymavlink produced, and over loopback UDP.
- Equation (6) over the fleet window: `fdt_fleet_keep_history` keeps past
  `cᵗ` beside the `Bᵗ` each twin queues; `fdt_fleet_state_at` and
  `fdt_fleet_ctx_at` read [Fᵗ⁻¹; Fᵗ⁻ⁿ]; `fdt_coord_set_ctx_window` computes
  `cᵗ` over that window.
- `fdt_store_spacing`: the inter-vessel distance Section IV gives as the
  example `cᵗ`. `examples/daemon.c` uses it, with the closing speed over
  the window, in place of the yaw spread that stood in for it.
- `fdt_dte_start_parallel` and `fdt_dte_stop_parallel`: the DTE runs its
  simulations in parallel, one POSIX thread each, released together at the
  tick edge and joined before the tick returns, as contribution (ii) of
  Section I states. `examples/daemon.c` runs its fluid stub that way. The
  build links `-pthread`.
- `fdt_db_t` (`include/fleet_dt/db.h`): the database of Figure 4, where
  states and actuation are stored. An append-only file of `Bᵗ`, `Aᵗ` and
  `gᵗ` records with a CRC-32 each; the reader keeps everything before a
  torn tail. It grows by 23,040 B of states per minute per vessel, the
  figure of Section IV, while the in-memory queue stays bounded.
  `examples/daemon.c` writes it when `FDT_DB` names a file.
- `fdt_predict` (`include/fleet_dt/predict.h`): the time-series estimator
  x̂ₖ₊d = f(xₖ, …, xₖ₋ₙ₊₁) of Section IV and backup slide 6, a least-squares
  polynomial of degree 0 to 2 per field of `Bᵗ`, angles unwrapped.
  `examples/daemon.c` steers on x̂ₖ₊d with d = 2 instead of storing a yaw
  trend in `pitch_deg`.
- `fdt_kf_t` (`include/fleet_dt/kalman.h`): the operator's Kalman filter of
  Section V-A, generating `Bᵗ` from `Iᵗ` (GPS position and velocity, gravity,
  gyro, tilt-compensated magnetometer with declination).
  `examples/two_paths.c` uses it; `π` stays on the raw path.

### Changed

- The MQTT client is the repository's own: MQTT 5.0 over POSIX sockets
  (`adapters/mqtt/fdt_mqtt_wire.c` encodes and frames the packets,
  `fdt_mqtt.c` runs the connection: CONNECT, PUBLISH at QoS 0 or 1 with
  PUBACK, SUBSCRIBE with wildcards, PINGREQ, DISCONNECT). libmosquitto is
  no longer needed; `make mqtt-test` needs only a `mosquitto` broker, and
  the MQTT client builds with the other adapters. `tests/test_mqtt_wire.c`
  checks the packets byte for byte.
- The WeBots controller drives up to 64 vessels instead of 8, discovered by
  DEF name under the rule of `adapters/webots/fdt_webots_fleet.h`. It times
  the synthetic boats (the in-process injector) apart from the twins and,
  when `FDT_CPU_REPORT` names a file, writes both shares there.
- `bench_webots_cpu` sweeps 0 to 60 boats as slide 14 does: one core
  (`FDT_CPU_CORE`), rendering off, the injector's CPU subtracted, and a
  least-squares slope for the cost per added twin. It prints the worst
  update of all twins against the 36 µs of slide 14 and writes
  `results/webots_update.*`.

- The bandwidth regulators sit in the data path, in the boat broker where
  Section III and the talk (slides 7, 12 and 13) put them. The autopilot
  publishes every sample on `fleet/<v>/raw/<sensor>`; `fdt_relay_t`, an MQTT
  client of the boat broker, runs one regulated stage (`fdt_regpub_t`) per
  vessel, holds the newest sample of each sensor, and at the edge of each
  125 ms tick publishes one message per sensor on `fleet/<v>/lsdt/<sensor>`.
  The injector runs the same stages in process (slide 12(b): Regulator x N),
  and the WeBots controller and the scale, latency and jitter benchmarks
  consume their output.
- The boat bridge carries `fleet/+/lsdt/#` up and `fleet/+/goal`,
  `fleet/+/act` down. Raw samples stay on the boat; actuation now reaches it.
- `make mqtt-test` starts a ground and a boat broker joined by those rules and
  checks that 133 raw samples become 4 regulated messages on the ground, with
  no raw sample across.
- `fdt_reg_t` releases the newest sample at the tick edge
  (`fdt_reg_offer`, `fdt_reg_capture`) instead of decimating by credit;
  `fdt_reg_admit` is gone, and `fdt_reg_init` takes the sensor rate only.
- The injector samples the IMU at 1 kHz, the barometer at 50 Hz, and GPS and
  power at 10 Hz (slide 13), and splits a tick into `fdt_inj_offer_frame` and
  `fdt_inj_release`.
- Partial and double updates are counted per boat over its sensor messages,
  by `fdt_rx_t`; the lossy row of `bench_scale` uses rarer loss to match.
  A stale message is counted and still applied, as Section V-B describes;
  `fdt_rx_set_drop_stale` turns on the drop Section VI lists as future work.
- `bench_scale` reports CPU per DTI as delta time per vessel per tick over
  the 125 ms period. The old ratio divided by the wall time of an unpaced
  loop, which now includes the injector synthesising every sensor sample.
- The loopback queue holds 1024 messages, and both transports match MQTT
  topic filters (`+`, `#`).

### Added

- The TILAPIA hull model (`adapters/webots/3d_models/tilapia/`), exported
  from the LSA's model of the vessel, replaces the `drone_boat` render mesh
  in every world; the collision mesh is unchanged. `docs/simulation.jpg`
  shows it.
- `tools/worldgen/` and `make webots-worlds`: WeBots worlds with 5 to 60
  hulls, generated from `jundia_fleet.wbt` for the slide 14 sweep.
  `tests/test_world.c` checks the naming rule and the generated worlds from
  1 to 64 vessels.
- `fdt_relay_t`, the regulator of the boat broker, and `tests/test_relay.c`.
- `fdt_rx_t`, the twin side of the regulated link: rebuilds `Iᵗ` from the
  per-sensor messages and counts partial, double and stale updates.
- Per-sensor wire codecs (`fdt_enc_sensor`, `fdt_dec_sensor`) and envelope
  kinds for the IMU, GPS, barometer and power.
- `bench_bandwidth` measures one boat through the regulated client, raw and
  regulated, and prints both beside the slide 13 figures.
- `make syntax` type-checks `tests/it_mqtt.c`, which now also runs a
  regulated tick through the broker.

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
- detection and counting of the partial and double frame updates reported in
  Section V-B
- the bandwidth regulators, releasing the newest sample of each sensor at the
  tick edge while the sensor path keeps its own rate
- the link-budget model, with both readings of the Section V-A figure
- the DTE registry, ticking parallel simulations off one clock
- MQTT transport over libmosquitto, and the bridge-mode broker configuration
- the Ardupilot ingest mapping for all 21 Table I entries, and `Aᵗ` back out as
  RC channel overrides
- the HSDT camera boundary, kept off the MQTT path
- the WeBots controller running δ at the simulation tick

The validation of Section V:

- deterministic synthetic telemetry injectors
- five benchmarks (regulator, link budget, scaling, latency, jitter), each
  writing a text report, a CSV series and an SVG chart
- a dependency-free SVG plotter, so a measurement can be regenerated on a bare
  toolchain
- claim coverage and results assembly, generating `docs/RESULTS.md`
- two runnable examples: a coordinated fleet frame, and the filtered/raw
  two-path arrangement of Section V-A

The simulation, verified against the real toolchain:

- the WeBots project (world, hull meshes, material and texture) from
  `lsa-pucrs/boat-digital-twin`, rewired to the model of Section IV
- three worlds differing only in vessel count, so the CPU claim of Section V-A
  can be measured as a difference
- `fdt_geo_offset`, the geodetic-to-local mapping, anchored once for the fleet
  so that the twins hold the geometry the telemetry reports
- `FDT_SHOT` and `FDT_SHOT_FRAME`, which export the 3D view from a running
  controller, so the visual reference of Section I(iii) can be shown
- `make syntax`, type-checking the SDK adapters against stub headers on a
  machine that has neither
- `make mqtt-test`, a round trip through a real mosquitto broker
- `make webots`, building the controller against a real WeBots install
- header dependency tracking, so editing a header rebuilds what included it

Documentation:

- the claim map, one row per statement the paper makes with the file that
  carries it, including the seven items Section VI names as future work
- the paper-to-code map for revision `-10`
- the readings the implementation takes where the text leaves a design choice
  open, and the position resolution that follows from the 48-byte state of
  Section IV: 0.37 m in longitude and 0.21 m in latitude at the Jundiá
  coordinates
- a captured frame of the running simulation, as the visual reference of
  Section I(iii)
