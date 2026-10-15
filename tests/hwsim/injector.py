#!/usr/bin/env python3
"""Software stand-in of the TX99 injector (Section VI-B) for the simulated network: injects
broadcast data frames of 1500 bytes at 24 Mbit/s on a monitor interface of mac80211_hwsim, so
wmediumd keeps the medium busy for a chosen fraction of the time.

  injector.py IFACE --plan plan.txt --start UNIX_TIME
plan.txt: one line "OFFSET_S DURATION_S DUTY" per burst (DUTY in [0, 1]: airtime fraction of the
injected frames), OFFSET_S counted from UNIX_TIME.
"""
import argparse
import socket
import struct
import time

# radiotap: version 0, length 12, present = rate (bit 2) | tx flags (bit 15); rate in 500 kbit/s
RADIOTAP = struct.pack("<BBHI", 0, 0, 12, (1 << 2) | (1 << 15)) + struct.pack("<BxH", 48, 0x0008)
# 802.11 data frame, broadcast, from-DS 0, to-DS 0
MAC = bytes.fromhex("0800" "0000" "ffffffffffff" "020000000300" "020000000300" "0000")
FRAME_LEN = 1500
AIRTIME_S = (20 + 4 * -(-(22 + 8 * (FRAME_LEN + 4)) // 96) + 6) / 1e6 + 28e-6   # frame + DIFS at 24 Mbit/s


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("iface")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--start", type=float, required=True)
    a = ap.parse_args()
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    s.bind((a.iface, 0))
    frame = RADIOTAP + MAC + bytes(FRAME_LEN - len(MAC))
    plan = []
    for line in open(a.plan):
        line = line.split("#")[0].split()
        if len(line) >= 3:
            plan.append((float(line[0]), float(line[1]), float(line[2])))
    sent = 0
    for off, dur, duty in plan:
        t0 = a.start + off
        while time.time() < t0:
            time.sleep(min(0.01, t0 - time.time()))
        period = AIRTIME_S / max(duty, 1e-3)
        nxt = time.time()
        end = t0 + dur
        while True:
            now = time.time()
            if now >= end:
                break
            if now >= nxt:
                try:
                    s.send(frame)
                    sent += 1
                except OSError:
                    pass
                nxt += period
            else:
                time.sleep(min(nxt - now, 0.0005))
        print("injector: burst at +%.1f s for %.1f s, duty %.2f, %d frames so far" % (off, dur, duty, sent), flush=True)


if __name__ == "__main__":
    main()
