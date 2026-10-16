# Hardware-in-the-loop run with a vessel

The vessel programs run on the RP4 of a vessel beside its own services (a second broker on ports
11883/11884), and the GS programs run on the GS PC. The run uses the real autopilot telemetry
(MAVProxy output 14551), the layered video source (`fdt_source.py`, a frame-size trace) for the topics
without a source, and the network between the two hosts. The vessel regulator runs in mode paper and
polices to the demand of `fdt_dti`; both use the jitter allowance `JITTER_MS` (default 2) and the DTP
of the vessel (`video_layers.py resolve`).

```bash
# vessel (RP4), from the repository built with make
tests/hil/vessel_hil.sh start 1 ~/hil/dtp.json ~/hil/run1 11883 udpout:127.0.0.1:14551   # load: the video layers
# GS
python3 tests/hil/gs_hil.py --out runs/hil-1 --dtp dtp.json --vessel 1=192.168.80.102,11883,11884 --seconds 180 --y 300000
# vessel, after the GS run
tests/hil/vessel_hil.sh stop ~/hil/run1
```

`--y` publishes a constant y(t) when the GS radio offers no survey (a Wi-Fi card such as iwlwifi);
`--iface` runs the channel monitor on a radio that offers it. `MAV_ARGS` passes options to
`mav_publisher.py` (for example `--state-offset 0.0925`). `gs_hil.py` writes the DTA, the decisions
of the AS and `summary.json` (Algorithm 1 and r ≤ d per step, publication delay of p(t), counts per
topic, Section VI metrics).
