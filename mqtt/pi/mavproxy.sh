#!/bin/bash
# MAVProxy on the boat. It BINDS udp 14550; ArduRover is the client that sends
# there (-A udp:127.0.0.1:14550). This direction survives a MAVProxy restart:
# ArduRover keeps sending to a fixed address instead of latching onto an
# ephemeral peer port. The link is re-exported to the network on 14551.
# 14552 is a fixed udpout for the boat MQTT publisher (fleet-mqtt-sensors):
# udpin never forgets a client, so a local service restarting from a new
# ephemeral port on 14551 would leave a stale client behind each time.
exec mavproxy.py \
    --master udpin:127.0.0.1:14550 \
    --out udpin:0.0.0.0:14551 \
    --out udpout:127.0.0.1:14552 \
    --aircraft barco \
    "$@"
