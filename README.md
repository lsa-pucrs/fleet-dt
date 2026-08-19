# mqtt link test

A software test of one MQTT link between a boat and a station: one broker, one
topic namespace, sensor telemetry and video through the same broker, and a
round-trip probe. It runs on a single machine, and `tc netem` supplies the
impairment.

The boat is a Raspberry Pi 4 with a Navio2 and a ZED camera. The station is the
JMCS. Both sides import one topic module and never spell a topic anywhere else.

```
mqtt/
├── topics.py         the boat/<id>/<group> tree, imported by BOTH sides
├── requirements.txt  paho-mqtt, plus opencv-python for the camera
├── pi/               boat side
│   ├── mosquitto.conf    the broker the station connects to
│   ├── publisher.py      sensor groups at their rates, ping echo, cmd log
│   └── zed_publisher.py  camera -> JPEG -> zed/frame, one frame per message
└── jmcs/             station side
    ├── mosquitto.conf    station-local broker
    ├── subscriber.py     boat/# live table: msg/s and kbps per topic, CSV
    └── probe.py          RTT through ping/pong, percentiles, CSV
```

## Topics

```
boat/<id>/imu        {a_x, a_y, a_z, w_x, w_y, w_z}            8 Hz
boat/<id>/mag        {m_x, m_y, m_z}                           8 Hz
boat/<id>/gps        {lat, lon, h, v_n, v_e, v_d}              5 Hz
boat/<id>/baro       {p_pa, t_c}                               1 Hz
boat/<id>/power      {v_b, i_b}                                1 Hz
boat/<id>/state      {lat, lon, h, roll, pitch, yaw,           8 Hz
                      u, v, w, p, q, r}
boat/<id>/zed/info   {resolution, fps, quality}                retained JSON
boat/<id>/zed/frame  raw JPEG bytes, one whole frame/message   ~15 fps
boat/<id>/cmd        {throttle_pct, cage_angle_rad}            station -> boat
boat/<id>/ping|pong  opaque probe payloads, the boat echoes
```

Every sensor payload carries `seq` and `t`, so the receiver can tell a dropped
message from a late one. The JSON payloads cost 38 kbps in total at these
rates, plus about 4 kbps of MQTT headers across 31 messages per second.

Publishes and subscribes run at QoS 0 throughout. That is deliberate: QoS 1
would retransmit the losses `netem` injects, and the test would measure the
protocol's recovery instead of the link.

## Run it, no impairment

Two terminals on one machine.

    mosquitto -c mqtt/pi/mosquitto.conf
    python3 mqtt/pi/publisher.py --broker localhost --boat-id b1
    python3 mqtt/pi/zed_publisher.py --broker localhost --boat-id b1   # optional

    python3 mqtt/jmcs/subscriber.py --broker localhost --boat-id b1 --csv load.csv
    python3 mqtt/jmcs/probe.py --broker localhost --boat-id b1 --csv rtt.csv

The subscriber's table answers which packets and how many kbps per topic. The
probe's percentiles answer round-trip time. Run the probe with the camera on
and off under the same conditions: the difference is what video through the
broker costs the small packets.

## Run it under impairment

`tc netem` shapes an interface. Shaping `lo` reaches every loopback flow on the
machine, and `lo` carries a 65536-byte MTU, so a 100 KB JPEG travels as two
segments instead of the seventy a 1500-byte link would use. A veth pair in a
network namespace fixes both, still on one machine.

    sudo ip netns add boat
    sudo ip link add veth-station type veth peer name veth-boat
    sudo ip link set veth-boat netns boat

    sudo ip addr add 10.90.0.1/24 dev veth-station
    sudo ip link set veth-station mtu 1500 up

    sudo ip netns exec boat ip addr add 10.90.0.2/24 dev veth-boat
    sudo ip netns exec boat ip link set veth-boat mtu 1500 up
    sudo ip netns exec boat ip link set lo up

The boat runs inside the namespace, the station on the host. `ip netns exec`
runs its command as root, and a `pip install --user` puts paho where root does
not look, so drop back to your own account for the Python:

    sudo ip netns exec boat mosquitto -c mqtt/pi/mosquitto.conf -d
    sudo ip netns exec boat sudo -u "$USER" python3 mqtt/pi/publisher.py \
         --broker 10.90.0.2 --boat-id b1

    python3 mqtt/jmcs/subscriber.py --broker 10.90.0.2 --boat-id b1 --csv load.csv

A system-wide `pip install paho-mqtt` removes the need for the inner `sudo -u`.

Impairment per direction, which is what a radio asks for: the uplink fills with
video while the downlink carries only `cmd`.

    sudo ip netns exec boat tc qdisc add dev veth-boat root netem \
         delay 40ms 10ms distribution normal loss 2% rate 5mbit
    sudo tc qdisc add dev veth-station root netem delay 40ms rate 1mbit

    sudo ip netns exec boat tc qdisc del dev veth-boat root
    sudo tc qdisc del dev veth-station root
    sudo ip netns del boat

`netem` delays each packet once per direction, so `delay 40ms` on both sides
produces a round trip near 80 ms. Measured on this setup: `probe.py` reported
p50 83.7 ms, p90 129.2 ms against a 74.0 ms minimum.

`probe.py` measures the echo that `publisher.py` performs. With the publisher
stopped, every ping is reported lost, which reads the same as a dead link.
Check that the publisher is running before believing a 100 % loss line.

## What the numbers mean

Payload sizes, publish rates and per-topic bandwidth are exact. They follow
from `topics.py`, not from the network.

Comparisons hold: video on against video off, 0 against 2 against 5 percent
loss, 10 against 5 against 1 Mbps. The ratios survive a move to real hardware.

Absolute latency and throughput do not transfer to a radio. `netem` models a
pipe with delay, loss and a rate. An 802.11 link retransmits at the MAC layer
where IP cannot see it, adapts its rate to the signal, loses packets in bursts
rather than independently, and shares airtime between the two directions. Plot
against what `netem` sets, and the axis stays truthful.

## Requirements

    pip install -r mqtt/requirements.txt

`paho-mqtt>=2.0` for every script, `opencv-python` for the camera publisher,
`mosquitto` on PATH for the broker, `iproute2` for the namespace and `netem`.

## License

MIT. See [LICENSE](LICENSE).
