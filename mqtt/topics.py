"""The peer-to-peer test's MQTT topic tree: ``boat/<boat_id>/<group>``.

One boat (Raspberry Pi + Navio2 + ZED) and one station (JMCS/MCS), everything
over MQTT -- the ZED included: ``zed/info`` announces the stream (retained
JSON), ``zed/frame`` carries whole MJPEG frames as raw bytes, one frame per
message, each independently decodable, so a lost frame costs exactly that
frame. The sensor groups are the Navio2 channels: raw sensors on one side,
ArduPilot's fused ``state`` on the other, and ``cmd`` carrying the actuation
the station sends down (throttle and cage angle).

``ping``/``pong`` are test infrastructure: the Pi publisher echoes every ping
payload byte-for-byte so the station's probe measures RTT through the broker
and back.

Both sides import this file and never spell a topic string anywhere else.
"""

ROOT = "boat"

SENSOR_FIELDS = {
    "imu": ("a_x", "a_y", "a_z", "w_x", "w_y", "w_z"),
    "mag": ("m_x", "m_y", "m_z"),
    "gps": ("lat", "lon", "h", "v_n", "v_e", "v_d"),
    "baro": ("p_pa", "t_c"),
    "power": ("v_b", "i_b"),
    "state": ("lat", "lon", "h", "roll", "pitch", "yaw", "u", "v", "w", "p", "q", "r"),
}
"""JSON payload fields per sensor group, Pi to station."""

SENSOR_RATES_HZ = {
    "imu": 8.0,
    "mag": 8.0,
    "gps": 5.0,
    "baro": 1.0,
    "power": 1.0,
    "state": 8.0,
}
"""Publish rate per group. 8 Hz is one message every 125 ms."""

ZED_INFO_FIELDS = ("resolution", "fps", "quality")
"""JSON payload of ``zed/info``, published retained."""

CMD_FIELDS = ("throttle_pct", "cage_angle_rad")
"""JSON payload of ``cmd``, station to Pi."""


def sensor(boat_id: str, group: str) -> str:
    """Topic of one sensor group: ``boat/<boat_id>/<group>``."""
    if group not in SENSOR_FIELDS:
        raise ValueError(f"unknown sensor group {group!r}; known: {sorted(SENSOR_FIELDS)}")
    return f"{ROOT}/{boat_id}/{group}"


def zed_info(boat_id: str) -> str:
    """Topic of the ZED announcement (retained JSON)."""
    return f"{ROOT}/{boat_id}/zed/info"


def zed_frame(boat_id: str) -> str:
    """Topic of the MJPEG frames (raw JPEG bytes, one frame per message)."""
    return f"{ROOT}/{boat_id}/zed/frame"


def cmd(boat_id: str) -> str:
    """Topic of the actuation, station to Pi."""
    return f"{ROOT}/{boat_id}/cmd"


def ping(boat_id: str) -> str:
    """Topic the probe publishes opaque ping payloads on."""
    return f"{ROOT}/{boat_id}/ping"


def pong(boat_id: str) -> str:
    """Topic the Pi echoes each ping payload on, untouched."""
    return f"{ROOT}/{boat_id}/pong"


def boat_subscription(boat_id: str) -> str:
    """Filter that subscribes one boat entirely: ``boat/<boat_id>/#``."""
    return f"{ROOT}/{boat_id}/#"
