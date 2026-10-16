/*
 * fdt_outq: vessel-side sampler of the unsent bytes of the bridge sockets (paper Section VI,
 * "Saturation throughput" and "Data collection").
 *
 * Every PERIOD (default 1 ms) of the step clock start(t) = EPOCH + t TS of this host, the sampler reads
 * the send queue of every established TCP socket whose local port is one of --port (the data bridge
 * 1884 and the control bridge 1883 of the vessel broker) through the sock_diag netlink interface. The
 * queue that sock_diag reports for TCP (idiag_wqueue = write_seq - snd_una) is the value of the ioctl
 * SIOCOUTQ on the socket, which another process cannot call on the broker's socket; tcp_info also gives
 * notsent_bytes (SIOCOUTQNSD) and bytes_acked. The sampler keeps the samples of a step in memory and
 * writes one JSON line per step (buffered; flushed every 64 steps and at exit):
 *
 *   {"t": step, "ports": {"1884": {"n": samples of phase 3, "nz": samples of phase 3 with bytes,
 *    "q0": bytes at the first sample of phase 3, "nsd0": notsent bytes then, "qmin", "qmax": over phase 3,
 *    "acked": bytes_acked during phase 3, "sock": sockets seen, "n12": samples of phases 1-2,
 *    "nz12": of them with bytes}, ...}, "steal_ms": guest steal time during the step,
 *    "late_us": largest lateness of a sample, "miss": samples skipped (more than one period late)}
 *
 * The first line is {"hdr": {...}} with the configuration. Phase 3 = [start(t) + T1 + T2, start(t + 1)).
 *
 *   fdt_outq --epoch 1767225600 --ts-us 125000 --t1-us 20000 --t2-us 10000 --port 1884 --port 1883
 *            --out /tmp/fdt-run/outq.jsonl [--period-us 1000] [--start-step S] [--steps N] [--fifo PRIO]
 *
 * Build: cc -O2 -Wall -o build/fdt_outq emu/tools/outq_sampler.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <linux/inet_diag.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/sock_diag.h>
#include <math.h>
#include <netinet/in.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define MAXP 8
#define TCPI_BYTES_ACKED 120
#define TCPI_NOTSENT 144

static volatile sig_atomic_t g_stop = 0;

static void on_sig(int s)
{
	(void)s;
	g_stop = 1;
}

struct port_sample {
	uint64_t q, nsd, acked;
	int sock;
};

struct port_step {
	int n, nz, n12, nz12, sock;
	int have0;
	uint64_t q0, nsd0, qmin, qmax, acked_first, acked_last;
	int have_acked;
};

static int g_nl = -1;

static int nl_open(void)
{
	int s = socket(AF_NETLINK, SOCK_DGRAM | SOCK_CLOEXEC, NETLINK_SOCK_DIAG);
	if (s < 0)
		return -1;
	int sz = 1 << 20;
	setsockopt(s, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
	return s;
}

/* one sock_diag dump of the established TCP sockets (IPv4); sums per local port of ports[] */
static int sample_ports(const int *ports, int np, struct port_sample *out)
{
	struct {
		struct nlmsghdr nlh;
		struct inet_diag_req_v2 r;
	} req;
	static uint32_t seq = 1;
	memset(out, 0, sizeof(*out) * np);
	memset(&req, 0, sizeof req);
	req.nlh.nlmsg_len = sizeof req;
	req.nlh.nlmsg_type = SOCK_DIAG_BY_FAMILY;
	req.nlh.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
	req.nlh.nlmsg_seq = ++seq;
	req.r.sdiag_family = AF_INET;
	req.r.sdiag_protocol = IPPROTO_TCP;
	req.r.idiag_states = 1u << 1; /* TCP_ESTABLISHED */
	req.r.idiag_ext = 1u << (INET_DIAG_INFO - 1);
	struct sockaddr_nl sa = {.nl_family = AF_NETLINK};
	if (sendto(g_nl, &req, sizeof req, 0, (struct sockaddr *)&sa, sizeof sa) < 0)
		return -1;
	static char buf[65536];
	for (;;) {
		ssize_t len = recv(g_nl, buf, sizeof buf, 0);
		if (len < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		for (struct nlmsghdr *h = (struct nlmsghdr *)buf; NLMSG_OK(h, (size_t)len); h = NLMSG_NEXT(h, len)) {
			if (h->nlmsg_seq != seq)
				continue;
			if (h->nlmsg_type == NLMSG_DONE)
				return 0;
			if (h->nlmsg_type == NLMSG_ERROR)
				return -1;
			struct inet_diag_msg *m = NLMSG_DATA(h);
			int lport = ntohs(m->id.idiag_sport);
			int k;
			for (k = 0; k < np && ports[k] != lport; k++)
				;
			if (k == np)
				continue;
			out[k].q += m->idiag_wqueue;
			out[k].sock++;
			int alen = (int)(h->nlmsg_len - NLMSG_LENGTH(sizeof(*m)));
			for (struct rtattr *a = (struct rtattr *)(m + 1); RTA_OK(a, alen); a = RTA_NEXT(a, alen)) {
				if (a->rta_type != INET_DIAG_INFO)
					continue;
				const unsigned char *ti = RTA_DATA(a);
				int tl = (int)RTA_PAYLOAD(a);
				if (tl >= TCPI_BYTES_ACKED + 8) {
					uint64_t v;
					memcpy(&v, ti + TCPI_BYTES_ACKED, 8);
					out[k].acked += v;
				}
				if (tl >= TCPI_NOTSENT + 4) {
					uint32_t v;
					memcpy(&v, ti + TCPI_NOTSENT, 4);
					out[k].nsd += v;
				}
			}
		}
	}
}

static int64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void sleep_until(int64_t t)
{
	struct timespec ts = {.tv_sec = t / 1000000000LL, .tv_nsec = t % 1000000000LL};
	while (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL) == EINTR && !g_stop)
		;
}

/* steal time of the guest (USER_HZ ticks of /proc/stat, in ms), -1 when unreadable */
static double steal_ms(void)
{
	FILE *f = fopen("/proc/stat", "r");
	if (!f)
		return -1;
	unsigned long long v[8] = {0};
	int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
		       &v[6], &v[7]);
	fclose(f);
	if (n < 8)
		return -1;
	return (double)v[7] * 1000.0 / (double)sysconf(_SC_CLK_TCK);
}

static void usage(void)
{
	fprintf(stderr, "fdt_outq --epoch S --ts-us US --t1-us US --t2-us US --port P [--port P] --out FILE "
			"[--period-us 1000] [--start-step T] [--steps N] [--fifo PRIO]\n");
	exit(2);
}

int main(int argc, char **argv)
{
	double epoch = -1;
	int64_t ts = 125000, t1 = 20000, t2 = 10000, period = 1000;
	int ports[MAXP], np = 0, fifo = 0;
	long long start_step = -1, steps = -1;
	const char *out = NULL;
	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		const char *v = i + 1 < argc ? argv[i + 1] : NULL;
		if (!v)
			usage();
		if (!strcmp(a, "--epoch"))
			epoch = atof(v);
		else if (!strcmp(a, "--ts-us"))
			ts = atoll(v);
		else if (!strcmp(a, "--t1-us"))
			t1 = atoll(v);
		else if (!strcmp(a, "--t2-us"))
			t2 = atoll(v);
		else if (!strcmp(a, "--period-us"))
			period = atoll(v);
		else if (!strcmp(a, "--port") && np < MAXP)
			ports[np++] = atoi(v);
		else if (!strcmp(a, "--out"))
			out = v;
		else if (!strcmp(a, "--start-step"))
			start_step = atoll(v);
		else if (!strcmp(a, "--steps"))
			steps = atoll(v);
		else if (!strcmp(a, "--fifo"))
			fifo = atoi(v);
		else
			usage();
		i++;
	}
	if (epoch < 0 || !out || np == 0 || ts <= 0 || period <= 0 || t1 + t2 >= ts || ts % period)
		usage();
	if (fifo > 0) {
		struct sched_param sp = {.sched_priority = fifo};
		if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
			fprintf(stderr, "fdt_outq: SCHED_FIFO %d: %s\n", fifo, strerror(errno));
	}
	g_nl = nl_open();
	if (g_nl < 0) {
		perror("fdt_outq: netlink sock_diag");
		return 1;
	}
	FILE *f = fopen(out, "w");
	if (!f) {
		perror(out);
		return 1;
	}
	static char fbuf[1 << 20];
	setvbuf(f, fbuf, _IOFBF, sizeof fbuf);
	signal(SIGTERM, on_sig);
	signal(SIGINT, on_sig);
	int64_t E = (int64_t)llround(epoch * 1e9), TS = ts * 1000, P3 = (t1 + t2) * 1000, PER = period * 1000;
	int64_t t = start_step >= 0 ? start_step : (now_ns() - E) / TS + 1;
	fprintf(f, "{\"hdr\":{\"epoch\":%.6f,\"ts_us\":%lld,\"t1_us\":%lld,\"t2_us\":%lld,\"period_us\":%lld,"
		   "\"first_step\":%lld,\"ports\":[",
		epoch, (long long)ts, (long long)t1, (long long)t2, (long long)period, (long long)t);
	for (int k = 0; k < np; k++)
		fprintf(f, "%s%d", k ? "," : "", ports[k]);
	fprintf(f, "],\"source\":\"sock_diag idiag_wqueue (= SIOCOUTQ)\"}}\n");
	fflush(f);
	int nper = (int)(TS / PER);
	long long done = 0;
	double steal0 = steal_ms();
	while (!g_stop && (steps < 0 || done < steps)) {
		int64_t st = E + t * TS;
		struct port_step ps[MAXP];
		memset(ps, 0, sizeof ps);
		int64_t late_max = 0;
		int miss = 0;
		for (int j = 0; j < nper && !g_stop; j++) {
			int64_t at = st + (int64_t)j * PER;
			int64_t now = now_ns();
			if (now > at + PER) {
				miss++;
				continue;
			}
			if (now < at)
				sleep_until(at);
			int64_t late = now_ns() - at;
			if (late > late_max)
				late_max = late;
			struct port_sample s[MAXP];
			if (sample_ports(ports, np, s) < 0)
				continue;
			int ph3 = (int64_t)j * PER >= P3;
			for (int k = 0; k < np; k++) {
				struct port_step *p = &ps[k];
				if (s[k].sock > p->sock)
					p->sock = s[k].sock;
				if (!ph3) {
					p->n12++;
					p->nz12 += s[k].q > 0;
					continue;
				}
				if (!p->have0) {
					p->have0 = 1;
					p->q0 = s[k].q;
					p->nsd0 = s[k].nsd;
					p->qmin = p->qmax = s[k].q;
				}
				p->n++;
				p->nz += s[k].q > 0;
				if (s[k].q < p->qmin)
					p->qmin = s[k].q;
				if (s[k].q > p->qmax)
					p->qmax = s[k].q;
				if (!p->have_acked) {
					p->have_acked = 1;
					p->acked_first = s[k].acked;
				}
				p->acked_last = s[k].acked;
			}
		}
		if (g_stop)
			break;
		double steal1 = steal_ms();
		fprintf(f, "{\"t\":%lld,\"ports\":{", (long long)t);
		for (int k = 0; k < np; k++) {
			struct port_step *p = &ps[k];
			fprintf(f,
				"%s\"%d\":{\"n\":%d,\"nz\":%d,\"q0\":%llu,\"nsd0\":%llu,\"qmin\":%llu,\"qmax\":%llu,"
				"\"acked\":%llu,\"sock\":%d,\"n12\":%d,\"nz12\":%d}",
				k ? "," : "", ports[k], p->n, p->nz, (unsigned long long)p->q0, (unsigned long long)p->nsd0,
				(unsigned long long)p->qmin, (unsigned long long)p->qmax,
				(unsigned long long)(p->acked_last >= p->acked_first ? p->acked_last - p->acked_first : 0),
				p->sock, p->n12, p->nz12);
		}
		fprintf(f, "},\"steal_ms\":%.1f,\"late_us\":%lld,\"miss\":%d}\n",
			steal0 >= 0 && steal1 >= 0 ? steal1 - steal0 : -1.0, (long long)(late_max / 1000), miss);
		steal0 = steal1;
		if (++done % 64 == 0)
			fflush(f);
		t++;
	}
	fflush(f);
	fclose(f);
	return 0;
}
