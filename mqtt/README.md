# mqtt: peer-to-peer MQTT link test (boat <-> station)

Standalone test of the Fleet-DT communication layer over one MQTT namespace:
one boat (Raspberry Pi 4 + Navio2 + ZED) and one station (JMCS/MCS),
everything through the broker -- video included, as MJPEG frames.

```
mqtt/
├── topics.py        the boat/<id>/<group> tree, imported by BOTH sides
├── requirements.txt paho-mqtt (opencv-python for the ZED publisher)
├── pi/              boat side
│   ├── mosquitto.conf   the broker the station connects to
│   ├── publisher.py     sensor groups at their rates + ping echo + cmd log
│   └── zed_publisher.py camera -> JPEG -> zed/frame (1 frame/message)
└── jmcs/            station side
    ├── mosquitto.conf   station-local broker (bridge block optional)
    ├── subscriber.py    boat/# live table: msg/s and kbps per topic + CSV
    └── probe.py         RTT through ping/pong, percentiles + CSV
```

## Topic tree (`topics.py`)

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
boat/<id>/cmd        {throttle_pct, cage_angle_rad}            station -> Pi
boat/<id>/ping|pong  opaque probe payloads (Pi echoes)
```

8 Hz is the paper's 125 ms DT pace. Every sensor payload carries `seq` and
`t`, so a late packet is detectable at the receiver (the fleet-validation
finding). MJPEG through the broker is deliberate: each frame decodes alone,
a lost message costs one frame, and the test measures what the broker adds
to the path (RTP-under-MQTT is ref [18] of the paper).

## Run (software peer-to-peer, two hosts or two terminals)

Boat side:

    mosquitto -c mqtt/pi/mosquitto.conf
    python3 mqtt/pi/publisher.py --broker localhost --boat-id b1
    python3 mqtt/pi/zed_publisher.py --broker localhost --boat-id b1   # optional

Station side (aim at the boat host):

    python3 mqtt/jmcs/subscriber.py --broker <boat-host> --boat-id b1 --csv load.csv
    python3 mqtt/jmcs/probe.py --broker <boat-host> --boat-id b1 --csv rtt.csv

The subscriber's table answers "which packets, how many kbps per sensor";
the probe's percentiles answer RTT with and without the video load. Repeat
the probe with `zed_publisher` on and off: the difference is what video
through the broker costs the small packets.
