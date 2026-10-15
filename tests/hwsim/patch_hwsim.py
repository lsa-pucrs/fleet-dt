#!/usr/bin/env python3
"""Patches mac80211_hwsim.c (Linux 6.12) so that the survey of a simulated radio reports the
busy time of the channel, the input of the channel monitor (Section V-B).

The upstream driver reports a fixed busy time (12.5 % of the dwell time). With the patch, each
radio accumulates the airtime of every frame it hears through wmediumd and of every frame it
sends, and the survey of the channel in use reports
    time      = milliseconds of boot time (cumulative, as the counters of ath9k),
    time_busy = milliseconds of airtime accumulated (cumulative),
with the flag IN_USE. Airtime of a frame of len bytes (FCS included) at a legacy rate of r Mbit/s:
    OFDM (ERP, 2.4 GHz): 20 us + 4 us * ceil((22 + 8 len) / (4 r)) + 6 us of signal extension
    DSSS/CCK:            192 us + 8 len / r
and a unicast frame adds SIFS (10 us) and the airtime of the 14-byte ACK at the same rate.
Each transmission attempt counts; DIFS and backoff are idle time of the channel, as on ath9k.

  patch_hwsim.py mac80211_hwsim.c        (edits the file in place; fails when an anchor is missing)
"""
import sys

HELPERS = r'''
/* Fleet-DT: airtime accounting of the survey (tests/hwsim/patch_hwsim.py) */
static atomic64_t fdt_busy_ns_all[64];

static u64 fdt_airtime_ns(struct ieee80211_supported_band *sband, int idx, int len)
{
	int r100;

	if (!sband || idx < 0 || idx >= sband->n_bitrates)
		return 0;
	r100 = sband->bitrates[idx].bitrate;   /* units of 100 kbit/s */
	if (r100 == 10 || r100 == 20 || r100 == 55 || r100 == 110)
		return 192000ULL + DIV_ROUND_UP(8000ULL * len * 10, r100);
	return (20 + 4 * DIV_ROUND_UP(22 + 8 * len, 4 * r100 / 10) + 6) * 1000ULL;
}

static u64 fdt_exchange_ns(struct ieee80211_supported_band *sband, int idx, int len, bool unicast)
{
	u64 t = fdt_airtime_ns(sband, idx, len);

	if (unicast)
		t += 10000ULL + fdt_airtime_ns(sband, idx, 14);
	return t;
}
'''


def sub(s: str, old: str, new: str, what: str) -> str:
    if s.count(old) != 1:
        sys.exit("patch_hwsim: anchor for %s found %d times" % (what, s.count(old)))
    return s.replace(old, new)


def main() -> None:
    path = sys.argv[1]
    s = open(path).read()
    if "fdt_airtime_ns" in s:
        print("already patched")
        return
    s = sub(s, '#include "mac80211_hwsim.h"\n', '#include "mac80211_hwsim.h"\n' + HELPERS, "includes")
    # per-radio counter: index of the radio in the list (radio_idx), at most 64 radios
    s = sub(s, "\tu64 rx_bytes;\n", "\tu64 rx_bytes;\n\tatomic64_t *fdt_busy_ns;\n", "struct field")
    # every frame that wmediumd delivers to this radio
    s = sub(s, "\tmac80211_hwsim_rx(data2, &rx_status, skb);\n\n\treturn 0;\nerr:",
            "\tif (data2->fdt_busy_ns)\n"
            "\t\tatomic64_add(fdt_exchange_ns(data2->hw->wiphy->bands[rx_status.band], rx_status.rate_idx,\n"
            "\t\t\t\t\t     frame_data_len + 4, !is_multicast_ether_addr(hdr->addr1)),\n"
            "\t\t\t     data2->fdt_busy_ns);\n"
            "\tmac80211_hwsim_rx(data2, &rx_status, skb);\n\n\treturn 0;\nerr:", "rx")
    # every attempt of every frame that this radio sends
    s = sub(s, "\ttxi->status.ack_signal = nla_get_u32(info->attrs[HWSIM_ATTR_SIGNAL]);\n",
            "\ttxi->status.ack_signal = nla_get_u32(info->attrs[HWSIM_ATTR_SIGNAL]);\n"
            "\tif (data2->fdt_busy_ns && data2->channel) {\n"
            "\t\tstruct ieee80211_supported_band *sb = data2->hw->wiphy->bands[data2->channel->band];\n"
            "\t\tbool acked = !(hwsim_flags & HWSIM_TX_CTL_NO_ACK) && (hwsim_flags & HWSIM_TX_STAT_ACK);\n"
            "\t\tint k, last = -1;\n\n"
            "\t\tfor (k = 0; k < IEEE80211_TX_MAX_RATES && tx_attempts[k].idx >= 0; k++)\n"
            "\t\t\tlast = k;\n"
            "\t\tfor (k = 0; k <= last; k++) {\n"
            "\t\t\tint n = tx_attempts[k].count;\n\n"
            "\t\t\tif (n <= 0)\n\t\t\t\tcontinue;\n"
            "\t\t\tatomic64_add(n * fdt_airtime_ns(sb, tx_attempts[k].idx, skb->len + 4), data2->fdt_busy_ns);\n"
            "\t\t\tif (acked && k == last)\n"
            "\t\t\t\tatomic64_add(10000ULL + fdt_airtime_ns(sb, tx_attempts[k].idx, 14), data2->fdt_busy_ns);\n"
            "\t\t}\n\t}\n", "tx status")
    # survey: one entry, the channel in use, cumulative time and busy time
    start = s.index("static int mac80211_hwsim_get_survey(struct ieee80211_hw *hw, int idx,")
    end = s.index("\n}\n", start) + 3
    s = s[:start] + r'''static int mac80211_hwsim_get_survey(struct ieee80211_hw *hw, int idx,
				     struct survey_info *survey)
{
	struct mac80211_hwsim_data *hwsim = hw->priv;

	if (idx != 0 || !hwsim->channel)
		return -ENOENT;
	mutex_lock(&hwsim->mutex);
	survey->channel = hwsim->channel;
	survey->filled = SURVEY_INFO_NOISE_DBM | SURVEY_INFO_TIME | SURVEY_INFO_TIME_BUSY |
			 SURVEY_INFO_IN_USE;
	survey->noise = -92;
	survey->time = div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC);
	survey->time_busy = hwsim->fdt_busy_ns ? div_u64(atomic64_read(hwsim->fdt_busy_ns), NSEC_PER_MSEC) : 0;
	mutex_unlock(&hwsim->mutex);
	return 0;
}
''' + s[end:]
    # attach the counter when the radio is created
    s = sub(s, "\tskb_queue_head_init(&data->pending);\n", "\tskb_queue_head_init(&data->pending);\n"
            "\tdata->fdt_busy_ns = &fdt_busy_ns_all[idx % ARRAY_SIZE(fdt_busy_ns_all)];\n", "new radio")
    open(path, "w").write(s)
    print("patched")


if __name__ == "__main__":
    main()
