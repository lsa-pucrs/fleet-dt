"""Mosquitto configuration of the GS broker and of the vessel brokers (SPEC.md Section 4, R5, R8, R9).

  python -m fleetdt.gsconf gs --vessel 1=192.168.1.99 --vessel 2=192.168.1.98 [--listen 0.0.0.0:1883] > gs.conf
  python -m fleetdt.gsconf vessel --vessel 1 --dtp /etc/fleetdt/dtp.json \
         [--plugin /opt/fleet-dt/lib/fdt_regulator.so] [--port 1883 --data-port 1884] > vessel.conf

The GS broker opens two bridges per vessel i: b<i>-ctl to vessel:1883 (demand and the step log of
the vessel regulator in, priority table and goals out, QoS 1) and b<i>-data to vessel:1884, the
dedicated listener of the data bridge (boat/b<i>/tx/# in, QoS 0: the paper forwards both levels with
QoS 0 and the GS removes repeated copies by the pair (t, s); --data-qos 2 keeps the QoS 2 subscription
of the as2 stack, whose regulator forwards the required topics with QoS 2). Both brokers disable Nagle's
algorithm on every socket (set_tcp_nodelay and patch 0002 for the bridge sockets) and set no limit on the
QoS 1/2 messages in flight (max_inflight_messages 0, PR9, SPEC_v2 Section 5).

Baseline B2 (--b2): the data bridge splits into one bridge per DSCP class, each over its own TCP
connection to its own listener of the vessel broker, so the vessel marks each class by its port
(system/nftables-fdt.nft). The plugin (mode b2) republishes topic <name> on boat/b<i>/tx2/<class>/<name>,
and each bridge maps it back to boat/b<i>/tx/<name> at the GS (mosquitto/conf/gs-v2.conf):
  b<i>-data-be    vessel:1884  BE    output topics (video)
  b<i>-data-af41  vessel:1885  AF41  telemetry streams
  b<i>-data-ef    vessel:1886  EF    the topics of R (state)
The paper stack runs B2 with the plugin in mode paper_b2 (fdt_reg_paper.h), which uses the same topics.

Baseline B10 (--per-topic DTP): one data bridge per vessel and uplink topic of the DTP, b<i>-data-<topic>,
each over its own TCP connection to the data listener of the vessel (vessel:1884) and carrying only
boat/b<i>/tx/<topic>, so no topic waits behind another in a TCP send buffer. No listener is added: the
L bridges of a vessel are L client connections of the same listener (Mosquitto sets no limit on the
connections of a listener unless max_connections is given; each bridge needs one descriptor at the GS
and one at the vessel, M (L + 1) + local clients at the GS, within the default limit of 1024 descriptors
for M = 4, L = 22).

Fleet feedback (--feedback): the control bridges also carry fleet/feedback out (QoS 1), the feedback of
fleetdt/feedback.py for the regulator modes paper_b3, b8 and paper_tdma.

Paper stack (vessel --mode paper and its variants): --demand external makes the regulator police to the
D_i(t) of the DTI program fdt_dti, and --jitter-ms J writes plugin_opt_jitter_ms J; the DTI program must
run with the same --jitter-ms J (paper_jitter_ms of emu/fleet.py passes the same value to both).
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, List, Optional, Sequence, Tuple

COMMON = """\
allow_anonymous true
set_tcp_nodelay true
max_packet_size 1048576
max_inflight_messages 0
max_queued_messages 100000
persistence false
"""


def _hostport(s: str, default_port: int) -> Tuple[str, int]:
    host, _, port = s.rpartition(":") if s.count(":") == 1 else (s, "", "")
    if not host:
        return s, default_port
    return host, int(port)


B2_PORTS = {"be": 1884, "af41": 1885, "ef": 1886}


def _bridge(name: str, addr: str, port: int, keepalive: int, topics: List[str]) -> List[str]:
    return ["",
            "connection %s" % name,
            "address %s:%d" % (addr, port),
            "bridge_protocol_version mqttv311",
            "cleansession true",
            "keepalive_interval %d" % keepalive,
            "restart_timeout 1 2",
            "notifications true",
            "notifications_local_only true",
            "notification_topic fleet/bridge/%s/state" % name] + topics


def dtp_topics(path: str) -> List[str]:
    """Uplink topic names of a DTP file (either format)."""
    with open(path, encoding="utf-8") as f:
        return [t["name"] if isinstance(t, dict) else str(t) for t in json.load(f)["topics"]]


def bridge_count(M: int, b2: bool = False, per_topic: Optional[Sequence[str]] = None) -> int:
    """Bridges of the GS broker: a control bridge and one (default), three (B2) or L (B10) data bridges per
    vessel."""
    return M * (1 + (len(per_topic) if per_topic else 3 if b2 else 1))


def gs_conf(vessels: Dict[int, str], listen: str = "0.0.0.0:1883", ctl_port: int = 1883, data_port: int = 1884,
            log: Optional[List[str]] = None, keepalive: int = 10,
            b2: bool = False, per_topic: Optional[Sequence[str]] = None, feedback: bool = False,
            data_qos: int = 0) -> str:
    """Configuration of the GS broker. data_qos is the QoS of the subscriptions of the data bridges: 0 for the
    paper stack (both levels with QoS 0, SPEC.md Section 4), 2 for the as2 stack; the control bridges keep QoS 1."""
    if b2 and per_topic:
        raise ValueError("--b2 and --per-topic exclude each other")
    if data_qos not in (0, 1, 2):
        raise ValueError("data_qos must be 0, 1 or 2")
    dq = int(data_qos)
    host, port = _hostport(listen, 1883)
    out = ["# GS MQTT broker of Fleet-DT, generated by python -m fleetdt.gsconf gs",
           "listener %d %s" % (port, host), COMMON.rstrip()]
    for lt in log or ["error", "warning", "notice"]:
        out.append("log_type %s" % lt)
    for i in sorted(vessels):
        addr, cp, dp = (vessels[i].split(",") + ["", ""])[:3]   # "ADDRESS" or "ADDRESS,CTL_PORT,DATA_PORT"
        cp, dp = int(cp or ctl_port), int(dp or data_port)
        out += ["",
                "connection b%d-ctl" % i,
                "address %s:%d" % (addr, cp),
                "bridge_protocol_version mqttv311",
                "cleansession true",
                "keepalive_interval %d" % keepalive,
                "restart_timeout 1 2",
                "notifications true",
                "notifications_local_only true",
                "notification_topic fleet/bridge/b%d-ctl/state" % i,
                "topic boat/b%d/demand in 1" % i,
                "topic boat/b%d/regulator in 0" % i,
                "topic boat/b%d/levels out 1" % i,
                "topic boat/b%d/goals out 1" % i]
        if feedback:
            out.append("topic fleet/feedback out 1")
        if per_topic:
            for name in per_topic:
                out += _bridge("b%d-data-%s" % (i, name), addr, dp, keepalive, ["topic boat/b%d/tx/%s in %d" % (i, name, dq)])
        elif not b2:
            out += _bridge("b%d-data" % i, addr, dp, keepalive, ["topic boat/b%d/tx/# in %d" % (i, dq)])
        else:
            for cls in ("be", "af41", "ef"):
                port = dp if cls == "be" else B2_PORTS[cls]
                out += _bridge("b%d-data-%s" % (i, cls), addr, port, keepalive,
                               ["topic # in %d boat/b%d/tx/ boat/b%d/tx2/%s/" % (dq, i, i, cls)])
    return "\n".join(out) + "\n"


def vessel_conf(vessel: int, dtp: str, plugin: str = "/opt/fleet-dt/lib/fdt_regulator.so", port: int = 1883,
                data_port: int = 1884, bind: str = "", stats_s: float = 10, log: Optional[List[str]] = None,
                log_dest: Optional[str] = None, b2: bool = False, mode: Optional[str] = None,
                p1_spacing_us: Optional[int] = None, p1_guard_us: Optional[int] = None,
                demand: Optional[str] = None, jitter_ms: Optional[float] = None,
                reg_log: Optional[str] = None) -> str:
    out = ["# MQTT broker of vessel %d of Fleet-DT, generated by python -m fleetdt.gsconf vessel" % vessel,
           "# %d: local programs and the control bridge of the GS; %d: data bridge of the GS only" % (port, data_port),
           ("listener %d %s" % (port, bind)).rstrip(),
           ("listener %d %s" % (data_port, bind)).rstrip()]
    if b2:
        out += [("listener %d %s" % (B2_PORTS[c], bind)).rstrip() for c in ("af41", "ef")]
    out.append(COMMON.rstrip())
    if log_dest:
        out.append("log_dest %s" % log_dest)
    for lt in log or ["error", "warning", "notice"]:
        out.append("log_type %s" % lt)
    out += ["",
            "plugin %s" % plugin,
            "plugin_opt_dtp %s" % dtp,
            "plugin_opt_vessel %d" % vessel]
    if mode:
        out.append("plugin_opt_mode %s" % mode)
    if b2:
        out.append("plugin_opt_data_ports %d,%d,%d" % (data_port, B2_PORTS["af41"], B2_PORTS["ef"]))
    else:
        out.append("plugin_opt_data_port %d" % data_port)
    if p1_spacing_us is not None:
        out.append("plugin_opt_p1_spacing_us %d" % p1_spacing_us)
    if p1_guard_us is not None:
        out.append("plugin_opt_p1_guard_us %d" % p1_guard_us)
    if demand is not None:
        if demand not in ("internal", "external"):
            raise ValueError("demand must be internal or external")
        out.append("plugin_opt_demand %s" % demand)
    if jitter_ms is not None:
        out.append("plugin_opt_jitter_ms %g" % float(jitter_ms))
    if reg_log:
        out.append("plugin_opt_log %s" % reg_log)
    out.append("plugin_opt_stats_s %g" % stats_s)
    return "\n".join(out) + "\n"


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.gsconf", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gs", help="configuration of the GS broker")
    g.add_argument("--vessel", action="append", required=True, metavar="I=ADDRESS[,CTL,DATA]",
                   help="vessel index, address and, optionally, the ports of its two listeners")
    g.add_argument("--listen", default="0.0.0.0:1883")
    g.add_argument("--ctl-port", type=int, default=1883)
    g.add_argument("--data-port", type=int, default=1884)
    g.add_argument("--b2", action="store_true", help="baseline B2: one data bridge per DSCP class")
    g.add_argument("--per-topic", default=None, metavar="DTP",
                   help="baseline B10: one data bridge per vessel and uplink topic of this DTP file")
    g.add_argument("--feedback", action="store_true",
                   help="the control bridges carry fleet/feedback (regulator modes paper_b3, b8, paper_tdma)")
    g.add_argument("--data-qos", type=int, choices=(0, 1, 2), default=0,
                   help="QoS of the data bridge subscriptions (0: paper stack, default; 2: as2 stack)")
    v = sub.add_parser("vessel", help="configuration of a vessel broker")
    v.add_argument("--vessel", type=int, required=True)
    v.add_argument("--dtp", default="/etc/fleetdt/dtp.json")
    v.add_argument("--plugin", default="/opt/fleet-dt/lib/fdt_regulator.so")
    v.add_argument("--port", type=int, default=1883)
    v.add_argument("--data-port", type=int, default=1884)
    v.add_argument("--bind", default="", help="address of both listeners (default: every address)")
    v.add_argument("--b2", action="store_true", help="baseline B2: listeners 1885 (AF41) and 1886 (EF)")
    v.add_argument("--mode", default=None,
                   help="plugin_opt_mode (fleetdt, b1, b2, b3, b5, slotted, tdma, paper, paper_nopolice, paper_noeos, "
                        "paper_b1, paper_b2, paper_b3, b8, paper_tdma)")
    v.add_argument("--demand", choices=("internal", "external"), default=None,
                   help="plugin_opt_demand of the paper modes (external: D_i(t) of fdt_dti)")
    v.add_argument("--jitter-ms", type=float, default=None,
                   help="plugin_opt_jitter_ms of the paper modes (fdt_dti needs the same --jitter-ms)")
    v.add_argument("--reg-log", default=None, help="plugin_opt_log: vessel log of the paper modes")
    v.add_argument("--log-dest", default=None, help="log_dest of the broker (e.g. 'file /tmp/m.log')")
    v.add_argument("--p1-spacing-us", type=int, default=None,
                   help="plugin_opt_p1_spacing_us: demand of vessel i at i * spacing after start(t) (default of "
                        "the plugin: i (T1 - g1) / (M + 1))")
    v.add_argument("--p1-guard-us", type=int, default=None,
                   help="plugin_opt_p1_guard_us: override of g1 = eps + 2 T_tick in i (T1 - g1) / (M + 1)")
    a = ap.parse_args(argv)
    if a.cmd == "gs":
        vessels = {}
        for s in a.vessel:
            i, _, addr = s.partition("=")
            vessels[int(i.lstrip("b"))] = addr
        sys.stdout.write(gs_conf(vessels, a.listen, a.ctl_port, a.data_port, b2=a.b2,
                                 per_topic=dtp_topics(a.per_topic) if a.per_topic else None, feedback=a.feedback,
                                 data_qos=a.data_qos))
    else:
        sys.stdout.write(vessel_conf(a.vessel, a.dtp, a.plugin, a.port, a.data_port, a.bind,
                                     b2=a.b2 or a.mode in ("b2", "paper_b2"),
                                     mode=a.mode, log_dest=a.log_dest, p1_spacing_us=a.p1_spacing_us,
                                     p1_guard_us=a.p1_guard_us, demand=a.demand, jitter_ms=a.jitter_ms,
                                     reg_log=a.reg_log))


if __name__ == "__main__":
    main()
