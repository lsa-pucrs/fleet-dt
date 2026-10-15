# Security

## Reporting

Email `cassiojonesdhein@gmail.com`. Please do not open a public issue for a
vulnerability.

## Surface

Every program parses bytes from the network, and each rejects a malformed input
whole rather than accepting part of it:

- the vessel regulator (`vessel/regulator/`) runs inside the vessel broker and
  reads the demand and priority tables (`fdt_demand_decode`,
  `fdt_levels_decode` reject a wrong length or a level above 2), and copies raw
  messages into queues bounded by `plugin_opt_max_msgs`;
- the AS and the VE decode the binary tables and the JSON topics of the GS
  broker and drop a malformed message;
- the channel monitor parses netlink attributes of the kernel with length
  checks on every attribute.

The brokers use `allow_anonymous true`, which suits an isolated field network
and **not** a network reachable from a wider one. A deployment on a shared
network wants TLS and per-client credentials on both brokers and on the bridges.
The repository holds no key of the field network: `radio/openwrt/` carries the
placeholder `CHANGE_ME`.

## Not a threat model for the vessels

This repository is the digital twin and the network of the fleet, not the
vessel firmware. Anything concerning the physical control of a boat belongs to
the ArduPilot side.
