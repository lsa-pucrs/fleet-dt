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
| `cleansession false` | A clean session drops the queue on reconnect, which is when the queue matters. |
| `topic fleet/+/lsdt/# out 1` | QoS 1 outbound: at-least-once, so a message survives a reconnect mid-flight. |

At-least-once permits duplicates, which is why the wire envelope carries a
sequence number. `include/fleet_dt/framesync.h` counts the duplicates that
reach a frame; it does not suppress them, because Section VI lists dropping
late packets as future work.

## Where the regulator sits

Section III drops samples "in the MQTT client in the broker", and the talk
draws the regulator inside the boat broker (slides 7, 12 and 13). Two levels
of topic carry the telemetry:

| Topic | Published by | Rate | Crosses the bridge |
|---|---|---|---|
| `fleet/<v>/raw/<sensor>` | the autopilot, to the boat broker | each sensor's own (IMU 1 kHz) | no |
| `fleet/<v>/lsdt/<sensor>` | the regulator beside the boat broker (`fdt_relay_t`) | one per sensor per 125 ms tick | yes |

The regulator is an MQTT client of the boat broker: it subscribes to
`fleet/+/raw/+`, holds the newest sample of each sensor, and publishes it at
the tick edge. `make mqtt-test` runs this path through two brokers joined by
this bridge.

## Why the topic rules point opposite ways

    topic fleet/+/lsdt/#  out 1 "" ""
    topic fleet/+/goal    in  1 "" ""
    topic fleet/+/act     in  1 "" ""

Regulated telemetry rises; goals and actuation descend. A single
bidirectional rule would echo a goal back up to the ground station as though
the boat had issued it, and the coordinator would then be reading its own
output as fleet state.

## Wildcards

The bridge rules above are broker wildcards. The transports also match MQTT
topic filters (`fdt_topic_match`), so the twin subscribes to every sensor of
every vessel with one filter, `fleet/+/lsdt/+`.
