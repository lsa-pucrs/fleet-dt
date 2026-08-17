# Broker configuration

Two files, one per side of the link Section III describes:

    mosquitto -c config/mosquitto/ground.conf   # ground station
    mosquitto -c config/mosquitto/boat.conf     # aboard each boat

## Why bridge mode

Section III: *"local MQTT brokers are connected in bridge mode to avoid
service interruption during temporary connection instability."*

A boat that published directly to the ground station would fail whenever the
Wi-Fi dropped, and every frame produced during the outage would be lost. With
a local broker the publish always succeeds, and the bridge carries the backlog
across once the link returns.

Three settings carry that behaviour, and removing any of them silently defeats
it:

| Setting | Why |
|---|---|
| `persistence true` | Without it the queue lives in memory only, and the outage takes it. |
| `cleansession false` | A clean session drops the queue on reconnect — the exact moment it matters. |
| `topic fleet/# out 1` | QoS 1 outbound: at-least-once, so a message survives a reconnect mid-flight. |

At-least-once permits duplicates, which is why the wire envelope carries a
sequence number. `include/fleet_dt/framesync.h` counts the duplicates that
reach a frame; it does not suppress them, because Section VI lists dropping
late packets as future work.

## Why the two topic rules point opposite ways

    topic fleet/#       out 1 "" ""
    topic fleet/+/goal  in  1 "" ""

Telemetry rises, goals descend. A single bidirectional rule would echo a goal
back up to the ground station as though the boat had issued it, and the
coordinator would then be reading its own output as fleet state.

## The wildcard lives here, not in the code

`include/fleet_dt/transport.h` matches topics exactly. The only wildcards in
the system are the two above, in the broker, which is where Section III puts
the bridging.
