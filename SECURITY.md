# Security

## Reporting

Email `cassiojonesdhein@gmail.com`. Please do not open a public issue for a
vulnerability.

## Surface

The library performs no I/O, allocates nothing, and holds no file-scope state.
All storage belongs to the caller, so its memory safety is bounded by the
caller's.

Two places do touch the outside world, and are where to look first:

- `src/codec.c` and `src/envelope.c` parse bytes off the network. Both reject
  a truncated or malformed frame outright rather than accepting part of it: a
  half-read state entering a twin's queue would be indistinguishable from a
  measurement.
- `adapters/mqtt/` connects to a broker. The shipped configuration in
  `config/mosquitto/` uses `allow_anonymous true`, which is appropriate for an
  isolated field network and **not** for anything reachable from a wider one.
  A deployment on a shared network wants TLS and per-client credentials.

## Not a threat model for the vessels

This repository is the digital twin, not the vessel firmware. Anything
concerning the physical control of a boat belongs to the Ardupilot side.
