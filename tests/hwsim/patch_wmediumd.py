#!/usr/bin/env python3
"""Patches wmediumd (github.com/bcopeland/wmediumd, commit 717e5d7) so that each frame that it
delivers to a radio carries the rate index of the attempt that succeeded. Upstream passes rate
index 1 (2 Mbit/s) for every frame, and the airtime that the patched mac80211_hwsim accumulates for
the survey (patch_hwsim.py) needs the real rate.

  patch_wmediumd.py wmediumd/wmediumd.c
"""
import sys

HELPER = r'''
/* Fleet-DT: rate index of the last attempt of a frame (tests/hwsim/patch_wmediumd.py) */
static int fdt_last_rate(struct frame *frame)
{
	int i, idx = 0;

	for (i = 0; i < frame->tx_rates_count; i++) {
		if (frame->tx_rates[i].idx < 0 || frame->tx_rates[i].count <= 0)
			break;
		idx = frame->tx_rates[i].idx;
	}
	return idx;
}

static void send_cloned_frame_msg('''


def sub(s, old, new, n, what):
    if s.count(old) != n:
        sys.exit("patch_wmediumd: anchor for %s found %d times" % (what, s.count(old)))
    return s.replace(old, new)


def main():
    path = sys.argv[1]
    s = open(path).read()
    if "fdt_last_rate" in s:
        print("already patched")
        return
    s = sub(s, "nla_put_u32(msg, HWSIM_ATTR_RX_RATE, 1)", "nla_put_u32(msg, HWSIM_ATTR_RX_RATE, rate_idx)", 1, "rate")
    s = sub(s, "\nstatic void send_cloned_frame_msg(", HELPER, 1, "helper")
    s = sub(s, "\t\t\t\t\t\t      1, signal,\n", "\t\t\t\t\t\t      fdt_last_rate(frame), signal,\n", 1, "mcast")
    s = sub(s, "\t\t\t\t\t\t      1, frame->signal,\n", "\t\t\t\t\t\t      fdt_last_rate(frame), frame->signal,\n", 1,
            "unicast")
    open(path, "w").write(s)
    print("patched")


if __name__ == "__main__":
    main()
