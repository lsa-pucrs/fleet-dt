/*
 * Fleet-DT channel monitor of the GS radio (Bullet M2, OpenWrt, ath9k), Section V-B, R6.
 *
 * At each step boundary start(t+1) the monitor reads the counters of the radio and
 * publishes, for step t,
 *     y(t) = R_PHY (Ts - Tb + To) / 8      bytes,
 * on fleet/throughput of the GS broker: JSON {"t", "y", "Tb", "To"}.
 *   Tb  busy time of the channel in the step: survey of the channel in use
 *       (NL80211_SURVEY_INFO_TIME_BUSY over NL80211_SURVEY_INFO_TIME, scaled to Ts)
 *   To  airtime at R_PHY of the frames that the radio receives and of the frames that
 *       it sends and the stations acknowledge (station counters of every station):
 *       To = 8 (B_rx + B_tx_acked) / R_PHY + (N_rx + N_tx_acked) 8 h_W / R_PHY,
 *       N_tx_acked = N_tx - N_tx_failed, B_tx_acked = B_tx N_tx_acked / N_tx,
 *       scaled to Ts by the interval between the two readings.
 * So y(t) counts the idle time of the step plus the time that Fleet-DT traffic took,
 * the throughput that the vessels could have used in the step. With no survey of the
 * step the monitor publishes nothing.
 *
 * The nl80211 backend talks generic netlink directly (no libnl), so the program
 * needs only libmosquitto and cJSON on the radio. The replay backend reads recorded
 * counter samples (JSON lines, the format of --record), for tests and for
 * recomputing y(t) offline with other R_PHY or h_W.
 *
 *   fdt_chmon --dtp dtp.json --iface wlan0 --broker GS:1883 [--record samples.jsonl]
 *   fdt_chmon --dtp dtp.json --replay samples.jsonl [--fast] [--loop] (--broker H:P | --stdout)
 *   fdt_chmon --dtp dtp.json --replay-stream FILE|-      samples as they arrive, y(t) on stdout
 *   fdt_chmon --iface wlan0 --dump        one reading of the counters, then exit
 *
 * The stream backend (--replay-stream) reads samples in the format of --record from a file, a FIFO or
 * stdin ("-") as a producer appends them (the counter export of the emulator, emu/ns3/fdt-emu.cc
 * --cnt, through fleetdt/monitor_adapter.py) and runs the pairing of the live loop: a sample read at
 * the boundary start(t) (t = the step of its instant "at") closes step t - 1 when the previous sample
 * belongs to step t - 1; compute() and emit() are the ones of the live loop.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <net/if.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <linux/genetlink.h>
#include <linux/netlink.h>
#include <linux/nl80211.h>

#include <cjson/cJSON.h>
#ifndef FDT_NO_MQTT
#include <mosquitto.h>
#endif

#include "fdt_common.h"

struct sample {
	int64_t at_us;
	int have_survey;
	uint64_t time_ms, busy_ms;
	uint64_t rx_bytes, tx_bytes;
	uint32_t rx_packets, tx_packets, tx_failed;
	int stations;
};

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

/* -------------------------------------------------------------- generic netlink */

static int nl_fd = -1;
static uint32_t nl_seq = 1;
static uint16_t nl80211_id;

#define NLA_DATA(a) ((void *)((char *)(a) + NLA_HDRLEN))
#define NLA_LEN(a) ((int)(a)->nla_len - NLA_HDRLEN)
#define NLA_OK(a, rem) ((rem) >= (int)sizeof(struct nlattr) && (a)->nla_len >= sizeof(struct nlattr) && (a)->nla_len <= (rem))
#define NLA_NEXT(a, rem) ((rem) -= NLA_ALIGN((a)->nla_len), (struct nlattr *)((char *)(a) + NLA_ALIGN((a)->nla_len)))
#define NLA_TYPE(a) ((a)->nla_type & NLA_TYPE_MASK)

struct nlreq {
	struct nlmsghdr n;
	struct genlmsghdr g;
	char buf[256];
};

static void put_attr(struct nlreq *r, uint16_t type, const void *data, int len)
{
	struct nlattr *a = (struct nlattr *)((char *)r + NLMSG_ALIGN(r->n.nlmsg_len));
	a->nla_type = type;
	a->nla_len = (uint16_t)(NLA_HDRLEN + len);
	memcpy(NLA_DATA(a), data, (size_t)len);
	r->n.nlmsg_len = NLMSG_ALIGN(r->n.nlmsg_len) + NLA_ALIGN(a->nla_len);
}

static int nl_open(void)
{
	nl_fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_GENERIC);
	if (nl_fd < 0)
		return -1;
	struct sockaddr_nl sa = {.nl_family = AF_NETLINK};
	if (bind(nl_fd, (struct sockaddr *)&sa, sizeof sa))
		return -1;
	struct timeval tv = {1, 0};
	setsockopt(nl_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
	return 0;
}

typedef int (*nl_cb)(struct nlmsghdr *h, void *arg);

/* sends a request and runs cb on every reply message until DONE (dump) or the ack */
static int nl_talk(struct nlreq *r, nl_cb cb, void *arg)
{
	r->n.nlmsg_seq = ++nl_seq;
	struct sockaddr_nl sa = {.nl_family = AF_NETLINK};
	if (sendto(nl_fd, r, r->n.nlmsg_len, 0, (struct sockaddr *)&sa, sizeof sa) < 0)
		return -1;
	static char buf[65536];
	for (;;) {
		ssize_t n = recv(nl_fd, buf, sizeof buf, 0);
		if (n < 0)
			return -1;
		for (struct nlmsghdr *h = (struct nlmsghdr *)buf; NLMSG_OK(h, (unsigned)n); h = NLMSG_NEXT(h, n)) {
			if (h->nlmsg_seq != r->n.nlmsg_seq)
				continue;
			if (h->nlmsg_type == NLMSG_DONE)
				return 0;
			if (h->nlmsg_type == NLMSG_ERROR) {
				struct nlmsgerr *e = NLMSG_DATA(h);
				if (e->error) {
					errno = -e->error;
					return -1;
				}
				return 0;   /* ack */
			}
			if (cb && cb(h, arg))
				return -1;
			if (!(h->nlmsg_flags & NLM_F_MULTI))
				return 0;
		}
	}
}

static int family_cb(struct nlmsghdr *h, void *arg)
{
	struct genlmsghdr *g = NLMSG_DATA(h);
	struct nlattr *a = (struct nlattr *)((char *)g + GENL_HDRLEN);
	int rem = (int)h->nlmsg_len - NLMSG_HDRLEN - GENL_HDRLEN;
	for (; NLA_OK(a, rem); a = NLA_NEXT(a, rem))
		if (NLA_TYPE(a) == CTRL_ATTR_FAMILY_ID)
			*(uint16_t *)arg = *(uint16_t *)NLA_DATA(a);
	return 0;
}

static int nl80211_resolve(void)
{
	struct nlreq r;
	memset(&r, 0, sizeof r);
	r.n.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
	r.n.nlmsg_type = GENL_ID_CTRL;
	r.n.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
	r.g.cmd = CTRL_CMD_GETFAMILY;
	r.g.version = 1;
	put_attr(&r, CTRL_ATTR_FAMILY_NAME, "nl80211", 8);
	nl80211_id = 0;
	if (nl_talk(&r, family_cb, &nl80211_id) || !nl80211_id)
		return -1;
	return 0;
}

static uint64_t nla_u64ish(struct nlattr *a)
{
	if (NLA_LEN(a) >= 8) {
		uint64_t v;
		memcpy(&v, NLA_DATA(a), 8);
		return v;
	}
	uint32_t v;
	memcpy(&v, NLA_DATA(a), 4);
	return v;
}

static int survey_cb(struct nlmsghdr *h, void *arg)
{
	struct sample *s = arg;
	struct genlmsghdr *g = NLMSG_DATA(h);
	struct nlattr *a = (struct nlattr *)((char *)g + GENL_HDRLEN);
	int rem = (int)h->nlmsg_len - NLMSG_HDRLEN - GENL_HDRLEN;
	for (; NLA_OK(a, rem); a = NLA_NEXT(a, rem)) {
		if (NLA_TYPE(a) != NL80211_ATTR_SURVEY_INFO)
			continue;
		int in_use = 0, have_t = 0, have_b = 0;
		uint64_t t = 0, b = 0;
		struct nlattr *n = NLA_DATA(a);
		int r2 = NLA_LEN(a);
		for (; NLA_OK(n, r2); n = NLA_NEXT(n, r2)) {
			switch (NLA_TYPE(n)) {
			case NL80211_SURVEY_INFO_IN_USE: in_use = 1; break;
			case NL80211_SURVEY_INFO_TIME: t = nla_u64ish(n); have_t = 1; break;
			case NL80211_SURVEY_INFO_TIME_BUSY: b = nla_u64ish(n); have_b = 1; break;
			default: break;
			}
		}
		if (in_use && have_t && have_b) {
			s->have_survey = 1;
			s->time_ms = t;
			s->busy_ms = b;
		}
	}
	return 0;
}

static int station_cb(struct nlmsghdr *h, void *arg)
{
	struct sample *s = arg;
	struct genlmsghdr *g = NLMSG_DATA(h);
	struct nlattr *a = (struct nlattr *)((char *)g + GENL_HDRLEN);
	int rem = (int)h->nlmsg_len - NLMSG_HDRLEN - GENL_HDRLEN;
	for (; NLA_OK(a, rem); a = NLA_NEXT(a, rem)) {
		if (NLA_TYPE(a) != NL80211_ATTR_STA_INFO)
			continue;
		uint64_t rxb32 = 0, txb32 = 0, rxb64 = 0, txb64 = 0;
		int h64r = 0, h64t = 0;
		struct nlattr *n = NLA_DATA(a);
		int r2 = NLA_LEN(a);
		for (; NLA_OK(n, r2); n = NLA_NEXT(n, r2)) {
			uint32_t v32 = 0;
			if (NLA_LEN(n) >= 4)
				memcpy(&v32, NLA_DATA(n), 4);
			switch (NLA_TYPE(n)) {
			case NL80211_STA_INFO_RX_BYTES: rxb32 = v32; break;
			case NL80211_STA_INFO_TX_BYTES: txb32 = v32; break;
			case NL80211_STA_INFO_RX_BYTES64: rxb64 = nla_u64ish(n); h64r = 1; break;
			case NL80211_STA_INFO_TX_BYTES64: txb64 = nla_u64ish(n); h64t = 1; break;
			case NL80211_STA_INFO_RX_PACKETS: s->rx_packets += v32; break;
			case NL80211_STA_INFO_TX_PACKETS: s->tx_packets += v32; break;
			case NL80211_STA_INFO_TX_FAILED: s->tx_failed += v32; break;
			default: break;
			}
		}
		s->rx_bytes += h64r ? rxb64 : rxb32;
		s->tx_bytes += h64t ? txb64 : txb32;
		s->stations++;
	}
	return 0;
}

static int nl_dump(uint8_t cmd, uint32_t ifindex, nl_cb cb, void *arg)
{
	struct nlreq r;
	memset(&r, 0, sizeof r);
	r.n.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
	r.n.nlmsg_type = nl80211_id;
	r.n.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
	r.g.cmd = cmd;
	r.g.version = 0;
	put_attr(&r, NL80211_ATTR_IFINDEX, &ifindex, 4);
	return nl_talk(&r, cb, arg);
}

static int read_nl80211(uint32_t ifindex, struct sample *s)
{
	memset(s, 0, sizeof *s);
	s->at_us = fdt_now_us();
	if (nl_dump(NL80211_CMD_GET_SURVEY, ifindex, survey_cb, s))
		return -1;
	if (nl_dump(NL80211_CMD_GET_STATION, ifindex, station_cb, s))
		return -1;
	return 0;
}

/* ---------------------------------------------------------------- computation */

struct result { double y, Tb, To; };

static int compute(const struct fdt_dtp *d, const struct sample *a, const struct sample *b, struct result *r)
{
	if (!a->have_survey || !b->have_survey || b->time_ms < a->time_ms || b->busy_ms < a->busy_ms)
		return -1;
	double Ts = d->Ts;
	uint64_t dt_ms = b->time_ms - a->time_ms, db_ms = b->busy_ms - a->busy_ms;
	double Tb = dt_ms > 0 ? Ts * (double)db_ms / (double)dt_ms : (double)db_ms / 1000.0;
	if (Tb > Ts)
		Tb = Ts;
	double wall = (double)(b->at_us - a->at_us) / 1e6;
	double scale = wall > 0 ? Ts / wall : 1.0;
	double rxb = (double)(b->rx_bytes - a->rx_bytes), txb = (double)(b->tx_bytes - a->tx_bytes);
	double rxp = (uint32_t)(b->rx_packets - a->rx_packets), txp = (uint32_t)(b->tx_packets - a->tx_packets);
	double txf = (uint32_t)(b->tx_failed - a->tx_failed);
	if (b->rx_bytes < a->rx_bytes || b->tx_bytes < a->tx_bytes)   /* a station left: counters restart */
		return -1;
	double txa = txp > txf ? txp - txf : 0;
	double txba = txp > 0 ? txb * txa / txp : 0;
	double To = (8.0 * (rxb + txba) + (rxp + txa) * 8.0 * d->h_W) / d->R_PHY * scale;
	double y = d->R_PHY * (Ts - Tb + To) / 8.0;
	double ymax = d->R_PHY * Ts / 8.0;
	r->y = y < 0 ? 0 : (y > ymax ? ymax : y);
	r->Tb = Tb;
	r->To = To;
	return 0;
}

/* ---------------------------------------------------------------- input/output */

static void sample_json(FILE *f, const struct sample *s)
{
	fprintf(f, "{\"at\":%.6f,\"survey\":%d,\"time_ms\":%" PRIu64 ",\"busy_ms\":%" PRIu64 ",\"rx_bytes\":%" PRIu64
	        ",\"tx_bytes\":%" PRIu64 ",\"rx_packets\":%u,\"tx_packets\":%u,\"tx_failed\":%u,\"stations\":%d}\n",
	        (double)s->at_us / 1e6, s->have_survey, s->time_ms, s->busy_ms, s->rx_bytes, s->tx_bytes,
	        s->rx_packets, s->tx_packets, s->tx_failed, s->stations);
}

static double jnum(const cJSON *o, const char *k)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
	return cJSON_IsNumber(v) ? v->valuedouble : 0.0;
}

static int load_replay(const char *path, struct sample **out, size_t *n)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return -1;
	size_t cap = 1024, k = 0;
	struct sample *s = malloc(cap * sizeof *s);
	char *line = NULL;
	size_t ll = 0;
	while (s && getline(&line, &ll, f) > 0) {
		cJSON *o = cJSON_Parse(line);
		if (!o)
			continue;
		if (k == cap) {
			cap *= 2;
			struct sample *ns = realloc(s, cap * sizeof *s);
			if (!ns)
				break;
			s = ns;
		}
		struct sample *x = &s[k++];
		memset(x, 0, sizeof *x);
		x->at_us = (int64_t)(jnum(o, "at") * 1e6);
		const cJSON *sv = cJSON_GetObjectItemCaseSensitive(o, "survey");
		x->have_survey = sv ? (cJSON_IsTrue(sv) || (cJSON_IsNumber(sv) && sv->valuedouble != 0)) : 1;
		x->time_ms = (uint64_t)jnum(o, "time_ms");
		x->busy_ms = (uint64_t)jnum(o, "busy_ms");
		x->rx_bytes = (uint64_t)jnum(o, "rx_bytes");
		x->tx_bytes = (uint64_t)jnum(o, "tx_bytes");
		x->rx_packets = (uint32_t)(uint64_t)jnum(o, "rx_packets");
		x->tx_packets = (uint32_t)(uint64_t)jnum(o, "tx_packets");
		x->tx_failed = (uint32_t)(uint64_t)jnum(o, "tx_failed");
		x->stations = (int)jnum(o, "stations");
		cJSON_Delete(o);
	}
	free(line);
	fclose(f);
	*out = s;
	*n = k;
	return s ? 0 : -1;
}

struct out {
	int to_stdout;
	char topic[128];
#ifndef FDT_NO_MQTT
	struct mosquitto *m;
#endif
};

static void emit(struct out *o, int64_t t, const struct result *r)
{
	char buf[256];
	int n = snprintf(buf, sizeof buf, "{\"t\":%" PRId64 ",\"y\":%.1f,\"Tb\":%.6f,\"To\":%.6f}", t, r->y, r->Tb, r->To);
	if (o->to_stdout) {
		printf("%s\n", buf);
		fflush(stdout);
	}
#ifndef FDT_NO_MQTT
	if (o->m)
		mosquitto_publish(o->m, NULL, o->topic, n, buf, 0, false);
#else
	(void)n;
#endif
}

/* ---------------------------------------------------------------- stream backend */

/* one sample line, with the field rules of load_replay(); returns 0 when the line holds JSON */
static int parse_sample_line(const char *line, struct sample *x)
{
	cJSON *o = cJSON_Parse(line);
	if (!o)
		return -1;
	memset(x, 0, sizeof *x);
	x->at_us = (int64_t)(jnum(o, "at") * 1e6);
	const cJSON *sv = cJSON_GetObjectItemCaseSensitive(o, "survey");
	x->have_survey = sv ? (cJSON_IsTrue(sv) || (cJSON_IsNumber(sv) && sv->valuedouble != 0)) : 1;
	x->time_ms = (uint64_t)jnum(o, "time_ms");
	x->busy_ms = (uint64_t)jnum(o, "busy_ms");
	x->rx_bytes = (uint64_t)jnum(o, "rx_bytes");
	x->tx_bytes = (uint64_t)jnum(o, "tx_bytes");
	x->rx_packets = (uint32_t)(uint64_t)jnum(o, "rx_packets");
	x->tx_packets = (uint32_t)(uint64_t)jnum(o, "tx_packets");
	x->tx_failed = (uint32_t)(uint64_t)jnum(o, "tx_failed");
	x->stations = (int)jnum(o, "stations");
	cJSON_Delete(o);
	return 0;
}

static int run_stream(const struct fdt_dtp *d, const char *path, const char *record)
{
	FILE *f = strcmp(path, "-") ? fopen(path, "r") : stdin;
	if (!f) {
		fprintf(stderr, "fdt_chmon: %s: %s\n", path, strerror(errno));
		return 1;
	}
	FILE *rec = record ? fopen(record, "a") : NULL;
	if (record && !rec) {
		fprintf(stderr, "fdt_chmon: %s: %s\n", record, strerror(errno));
		return 1;
	}
	struct sigaction sa;
	memset(&sa, 0, sizeof sa);
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	struct out o;
	memset(&o, 0, sizeof o);
	o.to_stdout = 1;
	struct sample prev, cur;
	int have_prev = 0;
	int64_t prev_t = 0;
	uint64_t published = 0, gaps = 0, bad = 0;
	char *line = NULL;
	size_t ll = 0;
	while (!stop && getline(&line, &ll, f) > 0) {
		if (parse_sample_line(line, &cur)) {
			bad++;
			continue;
		}
		if (rec)
			sample_json(rec, &cur);
		int64_t t = fdt_step_at(d, cur.at_us);   /* the boundary start(t) at which the sample was read */
		struct result r;
		if (have_prev && prev_t == t - 1 && !compute(d, &prev, &cur, &r)) {
			emit(&o, t - 1, &r);
			published++;
		} else if (have_prev) {
			gaps++;
		}
		prev = cur;
		prev_t = t;
		have_prev = 1;
	}
	free(line);
	if (rec)
		fclose(rec);
	if (f != stdin)
		fclose(f);
	fprintf(stderr, "fdt_chmon: stream published=%" PRIu64 " gaps=%" PRIu64 " unparsed=%" PRIu64 "\n",
	        published, gaps, bad);
	return 0;
}

static void usage(void)
{
	fprintf(stderr,
	        "usage: fdt_chmon --dtp FILE (--iface IF | --replay FILE [--fast] [--loop]) [--broker HOST:PORT]\n"
	        "                 [--topic fleet/throughput] [--stdout] [--record FILE] [--offset-ms MS] [--steps N]\n"
	        "       fdt_chmon --dtp FILE --replay-stream FILE|- [--record FILE]\n"
	        "       fdt_chmon --iface IF --dump\n");
	exit(2);
}

int main(int argc, char **argv)
{
	const char *dtp_path = NULL, *iface = NULL, *replay = NULL, *record = NULL, *broker = NULL;
	const char *stream = NULL;
	int fast = 0, loop = 0, dump = 0, prio = 50;
	double offset_ms = 0.5;
	long long steps = -1;
	struct out o;
	memset(&o, 0, sizeof o);
	snprintf(o.topic, sizeof o.topic, "fleet/throughput");
	static const struct option lo[] = {
		{"dtp", 1, 0, 'd'}, {"iface", 1, 0, 'i'}, {"replay", 1, 0, 'r'}, {"record", 1, 0, 'R'},
		{"broker", 1, 0, 'b'}, {"topic", 1, 0, 't'}, {"stdout", 0, 0, 's'}, {"fast", 0, 0, 'f'},
		{"loop", 0, 0, 'l'}, {"dump", 0, 0, 'D'}, {"offset-ms", 1, 0, 'o'}, {"steps", 1, 0, 'n'},
		{"prio", 1, 0, 'p'}, {"replay-stream", 1, 0, 'S'}, {"help", 0, 0, 'h'}, {0, 0, 0, 0}};
	int c;
	while ((c = getopt_long(argc, argv, "d:i:r:R:b:t:sflDo:n:p:S:h", lo, NULL)) != -1) {
		switch (c) {
		case 'd': dtp_path = optarg; break;
		case 'i': iface = optarg; break;
		case 'r': replay = optarg; break;
		case 'R': record = optarg; break;
		case 'b': broker = optarg; break;
		case 't': snprintf(o.topic, sizeof o.topic, "%s", optarg); break;
		case 's': o.to_stdout = 1; break;
		case 'f': fast = 1; break;
		case 'l': loop = 1; break;
		case 'D': dump = 1; break;
		case 'o': offset_ms = atof(optarg); break;
		case 'n': steps = atoll(optarg); break;
		case 'p': prio = atoi(optarg); break;
		case 'S': stream = optarg; break;
		default: usage();
		}
	}
	uint32_t ifindex = 0;
	if (iface) {
		ifindex = if_nametoindex(iface);
		if (!ifindex) {
			fprintf(stderr, "fdt_chmon: interface %s: %s\n", iface, strerror(errno));
			return 1;
		}
		if (nl_open() || nl80211_resolve()) {
			fprintf(stderr, "fdt_chmon: nl80211: %s\n", strerror(errno));
			return 1;
		}
	}
	if (dump) {
		struct sample s;
		if (!iface || read_nl80211(ifindex, &s)) {
			fprintf(stderr, "fdt_chmon: reading %s failed: %s\n", iface ? iface : "(no --iface)", strerror(errno));
			return 1;
		}
		sample_json(stdout, &s);
		return s.have_survey ? 0 : 3;
	}
	if (!dtp_path || (stream ? (iface || replay) : (!iface == !replay)))
		usage();
	struct fdt_dtp d;
	char err[256];
	if (fdt_dtp_load(dtp_path, &d, err, sizeof err)) {
		fprintf(stderr, "fdt_chmon: %s\n", err);
		return 1;
	}
	if (!(d.R_PHY > 0)) {
		fprintf(stderr, "fdt_chmon: R_PHY must be positive\n");
		return 1;
	}
	if (stream)
		return run_stream(&d, stream, record);
	struct sample *rs = NULL;
	size_t nrs = 0;
	if (replay && (load_replay(replay, &rs, &nrs) || nrs < 2)) {
		fprintf(stderr, "fdt_chmon: %s: no samples\n", replay);
		return 1;
	}
	if (replay && fast) {   /* offline: y of each pair of consecutive samples, t = step of the first */
		for (size_t k = 1; k < nrs; k++) {
			struct result r;
			if (!compute(&d, &rs[k - 1], &rs[k], &r)) {
				o.to_stdout = 1;
				emit(&o, fdt_step_at(&d, rs[k - 1].at_us + d.Ts_us / 2), &r);
			}
		}
		return 0;
	}
#ifndef FDT_NO_MQTT
	if (broker) {
		char host[128] = "localhost";
		int port = 1883;
		const char *colon = strrchr(broker, ':');
		if (colon) {
			snprintf(host, sizeof host, "%.*s", (int)(colon - broker), broker);
			port = atoi(colon + 1);
		} else {
			snprintf(host, sizeof host, "%s", broker);
		}
		mosquitto_lib_init();
		char cid[64];
		snprintf(cid, sizeof cid, "fdt-chmon-%d", (int)getpid());
		o.m = mosquitto_new(cid, true, NULL);
		mosquitto_reconnect_delay_set(o.m, 1, 4, false);
		int rc = mosquitto_connect(o.m, host, port, 10);
		mosquitto_loop_start(o.m);
		if (rc != MOSQ_ERR_SUCCESS) {
			fprintf(stderr, "fdt_chmon: connect %s:%d: %s (retrying)\n", host, port, mosquitto_strerror(rc));
			mosquitto_reconnect_async(o.m);
		}
	}
#else
	if (broker) {
		fprintf(stderr, "fdt_chmon: built without MQTT (FDT_NO_MQTT)\n");
		return 2;
	}
#endif
	if (!broker)
		o.to_stdout = 1;
	FILE *rec = record ? fopen(record, "a") : NULL;
	if (record && !rec) {
		fprintf(stderr, "fdt_chmon: %s: %s\n", record, strerror(errno));
		return 1;
	}
	struct sigaction sa;
	memset(&sa, 0, sizeof sa);
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	struct sched_param sp = {.sched_priority = prio};
	if (prio > 0 && sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "fdt_chmon: SCHED_FIFO unavailable (%s)\n", strerror(errno));

	int64_t off = (int64_t)(offset_ms * 1000);
	int64_t t = fdt_step_at(&d, fdt_now_us()) + 1;
	struct sample prev, cur;
	int have_prev = 0;
	int64_t prev_t = 0;
	size_t k = 0;
	uint64_t published = 0, gaps = 0;
	while (!stop && steps != 0) {
		fdt_sleep_until(fdt_start_us(&d, t) + off);   /* start(t): the end of step t-1 */
		if (stop)
			break;
		if (replay) {
			if (k >= nrs) {
				if (!loop)
					break;
				k = 0;
				have_prev = 0;
			}
			cur = rs[k++];   /* compute() uses only the spacing of the recorded instants */
		} else if (read_nl80211(ifindex, &cur)) {
			fprintf(stderr, "fdt_chmon: nl80211 read failed: %s\n", strerror(errno));
			have_prev = 0;
			t = fdt_step_at(&d, fdt_now_us()) + 1;
			continue;
		}
		if (rec)
			sample_json(rec, &cur);
		struct result r;
		if (have_prev && prev_t == t - 1 && !compute(&d, &prev, &cur, &r)) {
			emit(&o, t - 1, &r);
			published++;
		} else if (have_prev) {
			gaps++;
		}
		prev = cur;
		prev_t = t;
		have_prev = 1;
		t++;
		int64_t now_t = fdt_step_at(&d, fdt_now_us());
		if (now_t >= t) {   /* overran a step: the next reading starts a new pair */
			t = now_t + 1;
			have_prev = 0;
		}
		if (steps > 0)
			steps--;
	}
	if (rec)
		fclose(rec);
	fprintf(stderr, "fdt_chmon: published=%" PRIu64 " gaps=%" PRIu64 "\n", published, gaps);
#ifndef FDT_NO_MQTT
	if (o.m) {
		usleep(100000);
		mosquitto_disconnect(o.m);
		mosquitto_loop_stop(o.m, false);
		mosquitto_destroy(o.m);
		mosquitto_lib_cleanup();
	}
#endif
	free(rs);
	return 0;
}
