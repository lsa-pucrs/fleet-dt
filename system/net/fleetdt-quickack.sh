#!/bin/sh
# Fleet-DT (R8): immediate TCP ACKs on the route of the fleet network, so a QoS 1/2 flow of
# the bridges does not wait for the delayed ACK of the peer.
#   fleetdt-quickack.sh [DEV] [SUBNET]     (default: eth0 192.168.1.0/24)
set -eu
DEV=${1:-eth0}
NET=${2:-192.168.1.0/24}
route=$(ip -o route show "$NET" dev "$DEV" | head -n1)
[ -n "$route" ] || { echo "no route $NET on $DEV" >&2; exit 1; }
# shellcheck disable=SC2086
ip route change $(echo "$route" | sed 's/ quickack [0-9]//') quickack 1
ip route show "$NET" dev "$DEV"
