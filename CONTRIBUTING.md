# Contributing

## Before a commit

    make && make test

`make test` runs the C unit test of the vessel regulator, the Python tests and the end-to-end test of
the stack (it needs the patched Mosquitto of `mosquitto/build.sh` in `/opt/mosquitto-2.1`, or
`MOSQ=<prefix>`).

## Rules

- `SPEC.md` fixes the interfaces. A change of a topic, a wire format, the DTP file or the phase
  timing changes `SPEC.md` in the same commit, and every program that reads it.
- Numbers of the paper (equations, definitions, requirements R1 to R9) are cited by their number in
  the code and in the commit message.
- The vessel programs run on Raspbian 10: C with gcc 8.3 and glibc 2.28, Python 3.7
  (`tests/test_tools.py` checks the syntax).
- The channel monitor builds with the OpenWrt SDK: no dependency beyond libmosquitto and cJSON.
- Every MQTT client sets TCP_NODELAY (`fleetdt.model.tcp_nodelay`).
- No key, password or address of the field network in the repository.
