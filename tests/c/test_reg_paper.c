/*
 * Unit tests of the vessel regulator in mode paper (fdt_reg_paper.c) and of its DTP reader and 0-based
 * step clock (fdt_paper.c): wire formats of fleetdt/model.py, QoS 0 for both levels, no uplink without
 * p(t), policing to the d_il(t) of D_i(t), start(t) = epoch + t Ts, no pacing and no phase-1 offset,
 * the vessel log at the end of the step, the ablations paper_nopolice and paper_noeos, the step header.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cjson/cJSON.h>

#include "fdt_paper.h"
#include "fdt_reg_paper.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); } } while (0)

/* DTP v1 (dtp.json of SPEC.md), with a jitter for imu only */
static const char *DTP1 =
	"{\"epoch\": null, \"Ts\": 0.125, \"T1\": 0.020, \"T2\": 0.010,"
	" \"alpha\": 0.125, \"U\": 0.8, \"C0\": 300000, \"c_b\": 1.06, \"c_m\": 180,"
	" \"R_PHY\": 24000000, \"h_W\": 150, \"vessels\": [1, 2],"
	" \"topics\": ["
	"  {\"name\": \"state\", \"rate\": 8,  \"bound\": 48},"
	"  {\"name\": \"imu\",   \"rate\": 32, \"bound\": 48, \"jitter\": 0.002},"
	"  {\"name\": \"gps\",   \"rate\": 8,  \"bound\": 80},"
	"  {\"name\": \"video\", \"rate\": 30, \"bound\": 14000}],"
	" \"order\": [\"state\", \"imu\", \"gps\", \"video\"], \"needed\": [\"state\", \"imu\"]}";

/* DTP v2 (SPEC_v2.md) without c_b and c_m */
static const char *DTP2 =
	"{\"epoch\": 1767225600.0, \"Ts\": 0.125, \"T1\": 0.020, \"T2\": 0.010, \"tcp\": {\"mss\": 1448, \"h_T\": 80},"
	" \"mqtt\": {\"h_A\": 4}, \"vessels\": [1, 2, 3, 4],"
	" \"topics\": ["
	"  {\"name\": \"state\", \"rate\": 8,  \"jitter\": 0.002, \"n\": 1,  \"s\": 48,   \"kind\": \"stream\", \"qos\": 2},"
	"  {\"name\": \"video\", \"rate\": 15, \"jitter\": 0.010, \"n\": 16, \"s\": 1400, \"kind\": \"output\", \"qos\": 0}],"
	" \"required\": [\"state\"]}";

enum { STATE = 0, IMU = 1, GPS = 2, VIDEO = 3 };

static struct fdt_pdtp D;

/* ------------------------------------------------------------ fake broker */

struct sent {
	char topic[128];
	int qos;
	uint8_t p[4096];
	size_t len;
};
static struct sent sent[8192];
static int nsent;

static int pub(void *ctx, const char *topic, int qos, const void *p, size_t len)
{
	(void)ctx;
	if (nsent >= (int)(sizeof sent / sizeof *sent))
		return -1;
	struct sent *s = &sent[nsent++];
	snprintf(s->topic, sizeof s->topic, "%s", topic);
	s->qos = qos;
	s->len = len < sizeof s->p - 1 ? len : sizeof s->p - 1;
	memcpy(s->p, p, s->len);
	s->p[s->len] = 0;
	return 0;
}

static struct fdt_sock fake;
static void sockfn(void *ctx, struct fdt_sock *s)
{
	(void)ctx;
	*s = fake;
}

static struct fdt_io IO = {NULL, pub, sockfn};

static int count_topic(const char *prefix, int from)
{
	int n = 0;
	for (int i = from; i < nsent; i++)
		if (!strncmp(sent[i].topic, prefix, strlen(prefix)))
			n++;
	return n;
}

static const struct sent *find_topic(const char *topic, int from)
{
	for (int i = from; i < nsent; i++)
		if (!strcmp(sent[i].topic, topic))
			return &sent[i];
	return NULL;
}

/* ------------------------------------------------------------ helpers */

static int64_t S(int64_t t) { return fdt_p_start_us(&D, t); }
static int64_t V(int64_t t) { return fdt_p_v_us(&D, t); }

static uint8_t content[16384];

static void raw(struct fdt_preg *r, int l, size_t len, int64_t at, uint8_t tag)
{
	memset(content, tag, len);
	CHECK(fdt_preg_on_raw(r, l, content, len, at) == 0);
}

static int levels(struct fdt_preg *r, int64_t t, const uint8_t *p, int L, int64_t at)
{
	uint8_t b[5 + FDT_L_MAX];
	fdt_put_u32(b, (uint32_t)t);
	b[4] = (uint8_t)L;
	memcpy(b + 5, p, (size_t)L);
	return fdt_preg_on_levels(r, b, 5 + (size_t)L, at);
}

/* the d_l of the paper formula, computed here independently of fdt_paper.c */
static uint32_t expect_d(double rate, long s, double J, double c_b, double c_m)
{
	double n = ceil(rate * (0.125 + J) - 1e-9);
	return (uint32_t)ceil(c_b * (n * s + 6 * n) + c_m * n - 1e-9);
}

static FILE *memlog(char **buf, size_t *len)
{
	return open_memstream(buf, len);
}

/* number of log lines with the given step, topic and reason */
static int log_count(const char *text, int64_t t, const char *topic, const char *reason)
{
	int n = 0;
	const char *p = text;
	while (p && *p) {
		const char *e = strchr(p, '\n');
		size_t len = e ? (size_t)(e - p) : strlen(p);
		cJSON *o = cJSON_ParseWithLength(p, len);
		if (o) {
			const cJSON *jt = cJSON_GetObjectItem(o, "t"), *jtp = cJSON_GetObjectItem(o, "topic"),
			            *jr = cJSON_GetObjectItem(o, "reason"), *jb = cJSON_GetObjectItem(o, "bytes"),
			            *jl = cJSON_GetObjectItem(o, "l"), *js = cJSON_GetObjectItem(o, "seq");
			if (cJSON_IsNumber(jt) && cJSON_IsString(jtp) && cJSON_IsString(jr) && cJSON_IsNumber(jb) &&
			    cJSON_IsNumber(jl) && cJSON_IsNumber(js) && (int64_t)jt->valuedouble == t &&
			    !strcmp(jtp->valuestring, topic) && !strcmp(jr->valuestring, reason))
				n++;
			cJSON_Delete(o);
		}
		p = e ? e + 1 : NULL;
	}
	return n;
}

/* a regulator started in phase 1 of step t - 1: its first demand is that of step t, and a raw message
 * before v(t - 1) belongs to step t - 1 (activity for the idle rule) */
static void start_at(struct fdt_preg *r, struct fdt_preg_opts *o, int64_t t)
{
	nsent = 0;
	memset(&fake, 0, sizeof fake);
	CHECK(fdt_preg_init(r, &D, 1, o, S(t - 1) + 10000, &IO) == 0);
}

/* ------------------------------------------------------------ tests */

static void test_dtp(void)
{
	char err[256];
	struct fdt_pdtp d;
	CHECK(fdt_pdtp_parse(DTP1, strlen(DTP1), &D, err, sizeof err) == 0);
	CHECK(D.L == 4 && !strcmp(D.topic[VIDEO].name, "video") && D.topic[VIDEO].s == 14000);
	CHECK(fabs(D.c_b - 1.06) < 1e-12 && fabs(D.topic[GPS].c_m - 180) < 1e-12);
	CHECK(D.topic[IMU].has_jitter && fabs(D.topic[IMU].jitter - 0.002) < 1e-12 && !D.topic[STATE].has_jitter);
	CHECK(D.epoch_us == 1767225600LL * 1000000);
	/* v2: rate counts outputs of n messages; c_b and c_m from the cost bound of the paper (h_W = 0) */
	CHECK(fdt_pdtp_parse(DTP2, strlen(DTP2), &d, err, sizeof err) == 0);
	CHECK(d.L == 2 && fabs(d.topic[1].rate - 240.0) < 1e-9 && d.topic[1].s == 1400);
	CHECK(fabs(d.c_b - (1.0 + 160.0 / 1448.0)) < 1e-12);
	/* h_P of boat/b4/tx/state (16 bytes) with QoS 0: 1 + 1 + 2 + 16 = 20 (no packet identifier);
	 * c_k = 2 (h_T + h_W) = 160, c_m = c_b h_P + c_k (eq. cons) */
	CHECK(fdt_p_publish_len(16, 0, 54) == 74 && fdt_p_publish_len(16, 1, 54) == 76);
	CHECK(fdt_p_publish_len(16, 0, 200) == 1 + 2 + 2 + 16 + 200);   /* remaining length 218: two varint bytes */
	CHECK(fabs(d.c_k - 160) < 1e-12);
	CHECK(fabs(d.topic[0].c_m - (d.c_b * 20 + 160)) < 1e-9);
	/* the same file with h_W = 570.25 (802.11g ERP-OFDM at 24 Mb/s, emu/README.md):
	 * c_b = 1 + 2 (80 + 570.25) / 1448, c_k = 2 (650.25) = 1300.5, c_m = c_b 20 + c_k */
	{
		char hw[1024];
		snprintf(hw, sizeof hw, "{\"h_W\": 570.25, %s", DTP2 + 1);
		CHECK(fdt_pdtp_parse(hw, strlen(hw), &d, err, sizeof err) == 0);
		CHECK(fabs(d.c_b - (1.0 + 1300.5 / 1448.0)) < 1e-12);
		CHECK(fabs(d.c_k - 1300.5) < 1e-12);
		CHECK(fabs(d.topic[0].c_m - (d.c_b * 20 + 1300.5)) < 1e-9);
		/* backlog Q_i = ceil(c_b o + c_k n): 0 without bytes and packets, the cost bound otherwise */
		CHECK(fdt_p_backlog(&d, 0, 0) == 0 && fdt_p_backlog(&d, -1, -1) == 0);
		CHECK(fdt_p_backlog(&d, 1000, 0) == (uint32_t)ceil(d.c_b * 1000 - 1e-9));
		CHECK(fdt_p_backlog(&d, 0, 3) == (uint32_t)ceil(3 * 1300.5 - 1e-9));
		CHECK(fdt_p_backlog(&d, 1448, 2) == (uint32_t)ceil(d.c_b * 1448 + 2 * 1300.5 - 1e-9));
		/* a file with c_k keeps it */
		snprintf(hw, sizeof hw, "{\"c_k\": 100, %s", DTP2 + 1);
		CHECK(fdt_pdtp_parse(hw, strlen(hw), &d, err, sizeof err) == 0);
		CHECK(fabs(d.c_k - 100) < 1e-12 && fabs(d.topic[0].c_m - (d.c_b * 20 + 100)) < 1e-9);
	}
	/* L from the file: 6 telemetry topics and 16 video layers */
	char big[8192];
	int n = snprintf(big, sizeof big, "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"c_b\":1.06,\"c_m\":180,\"topics\":["
	                 "{\"name\":\"state\",\"rate\":8,\"bound\":48},{\"name\":\"imu\",\"rate\":32,\"bound\":48},"
	                 "{\"name\":\"gps\",\"rate\":8,\"bound\":80},{\"name\":\"baro\",\"rate\":8,\"bound\":32},"
	                 "{\"name\":\"power\",\"rate\":8,\"bound\":40},{\"name\":\"mag\",\"rate\":16,\"bound\":40}");
	for (int k = 0; k < 16; k++)
		n += snprintf(big + n, sizeof big - n, ",{\"name\":\"video%02d\",\"rate\":30,\"bound\":1400}", k);
	snprintf(big + n, sizeof big - n, "]}");
	CHECK(fdt_pdtp_parse(big, strlen(big), &d, err, sizeof err) == 0);
	CHECK(d.L == 22 && !strcmp(d.topic[21].name, "video15"));
	/* invalid files */
	CHECK(fdt_pdtp_parse("{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1}]}", 66, &d, err,
	                     sizeof err) < 0);
	const char *bad = "{\"Ts\":0.125,\"T1\":0.1,\"T2\":0.05,\"topics\":[{\"name\":\"a\",\"rate\":1,\"bound\":4}]}";
	CHECK(fdt_pdtp_parse(bad, strlen(bad), &d, err, sizeof err) < 0);
}

/* item 5: start(t) = epoch + t Ts, as in fleetdt/as_service.py */
static void test_clock(void)
{
	int64_t e = D.epoch_us;
	CHECK(fdt_p_start_us(&D, 0) == e);
	CHECK(fdt_p_start_us(&D, 7) == e + 7 * 125000);
	CHECK(fdt_p_step_at(&D, e) == 0 && fdt_p_step_at(&D, e - 1) == -1 && fdt_p_step_at(&D, e + 124999) == 0);
	CHECK(fdt_p_step_at(&D, e + 125000) == 1);
	/* the python value: floor((at - epoch) / Ts) at at = epoch + 1000.0625 s */
	CHECK(fdt_p_step_at(&D, e + 1000062500) == (int64_t)floor(1000.0625 / 0.125));
	CHECK(fdt_p_v_us(&D, 3) == e + 3 * 125000 + 30000);
	/* collection window of step t: [v(t-1), v(t)) */
	CHECK(fdt_p_collect_step(&D, V(4)) == 5 && fdt_p_collect_step(&D, V(4) - 1) == 4);
	CHECK(fdt_p_collect_step(&D, S(4)) == 4);
}

/* items 1 and 6: binary D_i(t) at start(t) (no offset), d_l with the jitter allowance, the idle rule */
static void test_demand(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	fdt_preg_opts_default(&o);
	start_at(&r, &o, 10);
	CHECK(r.next_p1 == 10);
	/* imu and gps active, state and video idle */
	raw(&r, IMU, 48, S(9) + 15000, 1);
	raw(&r, GPS, 80, S(9) + 16000, 1);
	CHECK(fdt_preg_tick(&r, S(10) - 1) == S(10));
	CHECK(count_topic("boat/b1/demand", 0) == 0);
	int64_t next = fdt_preg_tick(&r, S(10));
	const struct sent *s = find_topic("boat/b1/demand", 0);
	CHECK(s && s->qos == 0 && s->len == 9 + 4 * (size_t)D.L);
	if (s) {
		CHECK(fdt_get_u32(s->p) == 10 && s->p[4] == D.L);
		uint32_t t, Q = 7, d[FDT_L_MAX];
		CHECK(fdt_demand_decode(s->p, s->len, D.L, &t, &Q, d) == 0 && t == 10);
		CHECK(Q == 0 && fdt_get_u32(s->p + 5) == 0);   /* no data-bridge socket: Q_i = 0 */
		CHECK(d[STATE] == 0 && d[VIDEO] == 0);
		CHECK(d[IMU] == expect_d(32, 48, 0.002, 1.06, 180));       /* DTP jitter 2 ms: n = 5 */
		CHECK(d[GPS] == expect_d(8, 80, 0.002, 1.06, 180));        /* plugin default 2 ms: n = 2 */
		CHECK(d[IMU] == r.dmax[IMU] && d[GPS] == r.dmax[GPS]);
		struct fdt_pstep *st = fdt_preg_step(&r, 10);
		CHECK(st && st->have_d && !memcmp(st->d, d, sizeof(uint32_t) * D.L));
	}
	CHECK(next == V(10));
	/* without jitter the bound is that of fdt_dti (SPEC.md Section 5): ceil(r Ts) messages */
	CHECK(fdt_p_demand(&D, GPS, 0) == fdt_p_demand(&D, GPS, -1));
	CHECK(fdt_p_demand(&D, GPS, 0) == expect_d(8, 80, 0, 1.06, 180));
	/* idle rule: gps falls to 0 after 8 steps without a message */
	for (int64_t t = 11; t <= 19; t++)
		fdt_preg_tick(&r, S(t));
	const struct sent *last = NULL;
	for (int i = 0; i < nsent; i++)
		if (!strcmp(sent[i].topic, "boat/b1/demand"))
			last = &sent[i];
	CHECK(last && fdt_get_u32(last->p) == 19);
	if (last) {
		uint32_t t, d[FDT_L_MAX];
		fdt_demand_decode(last->p, last->len, D.L, &t, NULL, d);
		CHECK(d[GPS] == 0 && d[IMU] == 0);
	}
	/* the own table seen on the broker is accepted, another one is reported */
	uint8_t b[9 + 4 * FDT_L_MAX];
	if (last) {
		CHECK(fdt_preg_on_demand(&r, last->p, last->len, S(19) + 100) == 0);
		memcpy(b, last->p, last->len);
		fdt_put_u32(b + 9, 999);   /* d_1 differs */
		CHECK(fdt_preg_on_demand(&r, b, last->len, S(19) + 100) == 2 && r.s.dem_foreign == 1);
	}
	CHECK(fdt_preg_on_demand(&r, b, 7, S(19) + 100) == -1);
	fdt_preg_free(&r);
}

/* the ring of the PUBLISH lengths written to a data-bridge socket and n_i: the packets, newest first, whose
 * lengths first reach o_i */
static void test_ring(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	fdt_preg_opts_default(&o);
	start_at(&r, &o, 80);
	CHECK(fdt_preg_ring_count(&r, 0, 1000) == 0);          /* empty ring */
	fdt_preg_on_out(&r, 0, 500);                            /* oldest */
	fdt_preg_on_out(&r, 0, 1000);
	fdt_preg_on_out(&r, 0, 1500);                           /* newest */
	CHECK(fdt_preg_ring_count(&r, 0, 0) == 0);             /* o_i = 0 */
	CHECK(fdt_preg_ring_count(&r, 0, 1) == 1);             /* o_i inside the newest packet */
	CHECK(fdt_preg_ring_count(&r, 0, 1500) == 1);
	CHECK(fdt_preg_ring_count(&r, 0, 1501) == 2);          /* spans two packets */
	CHECK(fdt_preg_ring_count(&r, 0, 2500) == 2 && fdt_preg_ring_count(&r, 0, 2501) == 3);
	CHECK(fdt_preg_ring_count(&r, 0, 3000) == 3);
	CHECK(fdt_preg_ring_count(&r, 0, 99999) == 3);         /* the ring holds fewer bytes: every packet */
	CHECK(fdt_preg_ring_count(&r, 1, 100) == 0 && fdt_preg_ring_count(&r, -1, 100) == 0);
	/* wrap: the newest FDT_P_RING packets stay */
	for (int k = 0; k < FDT_P_RING + 5; k++)
		fdt_preg_on_out(&r, 1, (uint32_t)(k + 1));
	CHECK(r.ring_n[1] == FDT_P_RING);
	CHECK(fdt_preg_ring_count(&r, 1, FDT_P_RING + 5) == 1);                    /* newest: length R + 5 */
	CHECK(fdt_preg_ring_count(&r, 1, 2 * FDT_P_RING + 9) == 2);                /* R + 5 + R + 4 */
	CHECK(fdt_preg_ring_count(&r, 1, INT64_MAX / 2) == FDT_P_RING);
	fdt_preg_ring_reset(&r, 1);
	CHECK(r.ring_n[1] == 0 && fdt_preg_ring_count(&r, 1, 10) == 0 && fdt_preg_ring_count(&r, 0, 3000) == 3);
	fdt_preg_free(&r);
}

/* the backlog Q_i(t) = ceil(c_b o_i + c_k n_i) in the demand table, read at the start of phase 1 from the
 * data-bridge socket (o_i: SIOCOUTQ, n_i: PUBLISH packets with bytes among o_i); 0 without a socket */
static void test_backlog(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	uint32_t t, Q, d[FDT_L_MAX];
	fdt_preg_opts_default(&o);
	start_at(&r, &o, 60);
	raw(&r, IMU, 48, S(59) + 15000, 1);
	fake.n = 1;
	fake.s[0].outq = 3000;
	fdt_preg_on_out(&r, 0, 2000);
	fdt_preg_on_out(&r, 0, 1200);
	fdt_preg_on_out(&r, 0, 900);   /* 900 + 1200 < 3000 <= 900 + 1200 + 2000: n_i = 3 */
	fdt_preg_tick(&r, S(60));
	uint32_t want = (uint32_t)ceil(D.c_b * 3000 + D.c_k * 3 - 1e-9);
	CHECK(want == fdt_p_backlog(&D, 3000, 3) && want > 3000);
	const struct sent *s = find_topic("boat/b1/demand", 0);
	CHECK(s && s->len == 9 + 4 * (size_t)D.L);
	if (s) {
		CHECK(fdt_demand_decode(s->p, s->len, D.L, &t, &Q, d) == 0 && t == 60 && Q == want);
		CHECK(d[IMU] == r.dmax[IMU]);
	}
	struct fdt_pstep *st = fdt_preg_step(&r, 60);
	CHECK(st && st->have_Q && st->Q == want && st->bo == 3000 && st->bn == 3);
	/* empty send queue: Q_i = 0; the summary of step 60 carries Q, o and n */
	fake.s[0].outq = 0;
	int from = nsent;
	fdt_preg_tick(&r, S(61));
	s = find_topic("boat/b1/demand", from);
	CHECK(s && fdt_demand_decode(s->p, s->len, D.L, &t, &Q, d) == 0 && t == 61 && Q == 0);
	const struct sent *sm = find_topic("boat/b1/regulator", from);
	CHECK(sm != NULL);
	if (sm) {
		cJSON *j = cJSON_ParseWithLength((const char *)sm->p, sm->len);
		CHECK(j && cJSON_GetObjectItem(j, "Q")->valuedouble == want && cJSON_GetObjectItem(j, "o")->valuedouble == 3000 &&
		      cJSON_GetObjectItem(j, "n")->valuedouble == 3);
		cJSON_Delete(j);
	}
	fdt_preg_free(&r);

	/* external demand: the plugin writes Q_i(t) into the table of fdt_dti in place */
	fdt_preg_opts_default(&o);
	o.demand_ext = 1;
	start_at(&r, &o, 70);
	fake.n = 1;
	fake.s[0].outq = 1448;
	fdt_preg_on_out(&r, 0, 1200);
	fdt_preg_on_out(&r, 0, 2000);   /* o_i inside the newest packet: n_i = 1 */
	uint32_t dz[FDT_L_MAX] = {0};
	dz[GPS] = 777;
	uint8_t b[9 + 4 * FDT_L_MAX];
	size_t n = fdt_demand_encode(b, 70, 0, D.L, dz);
	CHECK(n == 9 + 4 * (size_t)D.L);
	CHECK(fdt_preg_stamp_demand(&r, b, n, S(70) + 1000) == 0);
	CHECK(fdt_demand_decode(b, n, D.L, &t, &Q, d) == 0 && t == 70 && Q == fdt_p_backlog(&D, 1448, 1));
	CHECK(d[GPS] == 777 && d[IMU] == 0);
	CHECK(fdt_preg_on_demand(&r, b, n, S(70) + 1000) == 0);
	st = fdt_preg_step(&r, 70);
	CHECK(st && st->have_d && st->have_Q && st->Q == Q && st->bo == 1448 && st->bn == 1);
	/* no data-bridge socket (SIOCOUTQ unavailable): Q_i = 0 */
	fake.n = 0;
	n = fdt_demand_encode(b, 71, 5, D.L, dz);
	CHECK(fdt_preg_stamp_demand(&r, b, n, S(71) + 1000) == 0);
	CHECK(fdt_demand_decode(b, n, D.L, &t, &Q, d) == 0 && t == 71 && Q == 0);
	CHECK(fdt_preg_stamp_demand(&r, b, 7, S(71) + 1000) == -1);
	CHECK(fdt_preg_stamp_demand(&r, b, n - 4, S(71) + 1000) == -1);
	fdt_preg_free(&r);
}

/* the socket is full: the broker holds the tail of the queue (bq); o_i = SIOCOUTQ + bq and n_i counts the packets,
 * newest first, that hold any of these bytes */
static void test_backlog_full_socket(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	uint32_t t, Q, d[FDT_L_MAX];
	fdt_preg_opts_default(&o);
	start_at(&r, &o, 90);
	raw(&r, IMU, 48, S(89) + 15000, 1);
	/* 20 packets of 1400 bytes written in order: the socket holds 16384 bytes (11.7 packets, the head of the
	 * oldest of them partly acknowledged), the broker the last 4 packets and 600 bytes of the fifth newest */
	for (int k = 0; k < 20; k++)
		fdt_preg_on_out(&r, 0, 1400);
	fake.n = 1;
	fake.s[0].outq = 16384;
	fake.s[0].bq = 4 * 1400 + 600;
	fdt_preg_tick(&r, S(90));
	int64_t ob = 16384 + 4 * 1400 + 600;                    /* 22584 bytes not yet carried */
	int64_t nb = (ob + 1399) / 1400;                          /* 17 packets hold some of them */
	CHECK(fdt_preg_ring_count(&r, 0, ob) == nb && nb == 17);
	struct fdt_pstep *st = fdt_preg_step(&r, 90);
	CHECK(st && st->have_Q && st->bo == ob && st->bq == 4 * 1400 + 600 && st->bn == nb);
	CHECK(st && st->Q == fdt_p_backlog(&D, ob, nb));
	const struct sent *s = find_topic("boat/b1/demand", 0);
	CHECK(s && fdt_demand_decode(s->p, s->len, D.L, &t, &Q, d) == 0 && t == 90 && Q == fdt_p_backlog(&D, ob, nb));
	/* SIOCOUTQ alone misses the tail: the bound with bq exceeds it by the 6200 bytes and their 5 packets */
	CHECK(Q > fdt_p_backlog(&D, 16384, (16384 + 1399) / 1400));
	CHECK(Q - fdt_p_backlog(&D, 16384, 12) >= (uint32_t)floor(D.c_b * 6200 + D.c_k * 5) - 1);
	/* a broker without patch 0004 (bq unknown): SIOCOUTQ alone, and the step log says bq = -1 */
	fake.s[0].bq = -1;
	int from = nsent;
	fdt_preg_tick(&r, S(91));
	st = fdt_preg_step(&r, 91);
	CHECK(st && st->bo == 16384 && st->bq == -1 && st->bn == 12);
	const struct sent *sm = find_topic("boat/b1/regulator", from);
	if (sm) {
		cJSON *j = cJSON_ParseWithLength((const char *)sm->p, sm->len);
		CHECK(j && cJSON_GetObjectItem(j, "o")->valuedouble == ob && cJSON_GetObjectItem(j, "bq")->valuedouble == 6200 &&
		      cJSON_GetObjectItem(j, "n")->valuedouble == nb);
		cJSON_Delete(j);
	} else {
		CHECK(sm != NULL);
	}
	/* an empty socket and an empty broker queue: Q_i = 0 */
	fake.s[0].outq = 0;
	fake.s[0].bq = 0;
	fdt_preg_tick(&r, S(92));
	st = fdt_preg_step(&r, 92);
	CHECK(st && st->Q == 0 && st->bn == 0);
	fdt_preg_free(&r);
}

/* wire format of the demand table: t, L, Q, d_1..d_L in network byte order */
static void test_demand_wire(void)
{
	uint8_t b[9 + 4 * FDT_L_MAX];
	uint32_t d[3] = {1, 2, 300000}, t, Q, e[3];
	size_t n = fdt_demand_encode(b, 7, 0x01020304u, 3, d);
	CHECK(n == 21 && b[4] == 3);
	CHECK(b[0] == 0 && b[3] == 7 && b[5] == 1 && b[6] == 2 && b[7] == 3 && b[8] == 4);
	CHECK(fdt_get_u32(b + 9) == 1 && fdt_get_u32(b + 17) == 300000);
	CHECK(fdt_demand_decode(b, n, 3, &t, &Q, e) == 0 && t == 7 && Q == 0x01020304u && e[2] == 300000);
	CHECK(fdt_demand_decode(b, n - 1, 3, &t, &Q, e) == -1 && fdt_demand_decode(b, n, 2, &t, &Q, e) == -1);
}

/* items 1, 2 and 9: levels 2 then 1 (strict priority), QoS 0 for both levels, level 0 held, order of reception,
 * header t, s */
static void test_forward(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL, *sb = NULL;
	size_t ll = 0, sl = 0;
	fdt_preg_opts_default(&o);
	o.log = memlog(&lb, &ll);
	o.step_log = memlog(&sb, &sl);
	start_at(&r, &o, 20);
	/* activity before the demand of step 20 */
	raw(&r, STATE, 48, S(19) + 15000, 0xA0);
	raw(&r, IMU, 48, S(19) + 16000, 0xB0);
	raw(&r, GPS, 80, S(19) + 17000, 0xC0);
	fdt_preg_tick(&r, S(20));
	/* messages of step 20 (window [v(19), v(20))): interleaved, state and gps at level 2 */
	int64_t a = V(19);
	raw(&r, IMU, 48, a + 1000, 0x11);
	raw(&r, STATE, 48, a + 2000, 0x21);
	raw(&r, GPS, 80, a + 3000, 0x31);
	raw(&r, IMU, 48, a + 4000, 0x12);
	raw(&r, STATE, 48, a + 5000, 0x22);
	raw(&r, VIDEO, 1000, a + 6000, 0x41);
	uint8_t p[FDT_L_MAX] = {0};
	p[STATE] = 2; p[IMU] = 1; p[GPS] = 2; p[VIDEO] = 0;
	CHECK(levels(&r, 20, p, D.L, S(20) + 25000) == 0);
	int from = nsent;
	CHECK(fdt_preg_tick(&r, V(20) - 1) == V(20));
	CHECK(nsent == from);   /* held until phase 3 */
	/* a message at v(20) belongs to step 21 */
	raw(&r, STATE, 48, V(20), 0x23);
	int64_t next = fdt_preg_tick(&r, V(20));
	CHECK(next == S(21));   /* no pacing: everything leaves in this tick, the next event is the end of the step */
	const char *exp_topic[5] = {"boat/b1/tx/state", "boat/b1/tx/gps", "boat/b1/tx/state", "boat/b1/tx/imu",
	                            "boat/b1/tx/imu"};
	/* revised model: QoS 0 for level 2 and level 1; level 2 still leaves first */
	CHECK(fdt_preg_qos(2) == 0 && fdt_preg_qos(1) == 0 && fdt_preg_qos(0) == 0);
	const int exp_qos[5] = {0, 0, 0, 0, 0};
	const uint8_t exp_tag[5] = {0x21, 0x31, 0x22, 0x11, 0x12};
	const int exp_seq[5] = {0, 0, 1, 0, 1};
	CHECK(nsent - from == 5);
	for (int k = 0; k < 5 && from + k < nsent; k++) {
		const struct sent *s = &sent[from + k];
		CHECK(!strcmp(s->topic, exp_topic[k]));
		CHECK(s->qos == exp_qos[k]);
		CHECK(fdt_get_u32(s->p) == 20 && fdt_get_u16(s->p + 4) == exp_seq[k]);
		CHECK(s->p[FDT_STEP_HEADER] == exp_tag[k]);
		CHECK(s->len == FDT_STEP_HEADER + (k == 1 ? 80u : 48u));
	}
	CHECK(count_topic("boat/b1/tx/video", 0) == 0);
	/* end of step 20: video logged as level0, summary on boat/b1/regulator */
	from = nsent;
	fdt_preg_tick(&r, S(21));
	fflush(o.log);
	CHECK(log_count(lb, 20, "video", "level0") == 1);
	const struct sent *sm = find_topic("boat/b1/regulator", from);
	CHECK(sm && sm->qos == 0);
	if (sm) {
		cJSON *j = cJSON_ParseWithLength((const char *)sm->p, sm->len);
		CHECK(j && cJSON_GetObjectItem(j, "t")->valuedouble == 20);
		const cJSON *fwd = cJSON_GetObjectItem(j, "fwd"), *held = cJSON_GetObjectItem(j, "held");
		CHECK(cJSON_GetArraySize(fwd) == D.L && cJSON_GetArrayItem(fwd, STATE)->valueint == 2 &&
		      cJSON_GetArrayItem(fwd, IMU)->valueint == 2 && cJSON_GetArrayItem(held, VIDEO)->valueint == 1);
		CHECK(cJSON_IsTrue(cJSON_GetObjectItem(j, "have_p")));
		cJSON_Delete(j);
		/* the step log holds the same summary as one line */
		fflush(o.step_log);
		const char *line = sb ? strstr(sb, "{\"t\":20,") : NULL;
		CHECK(line && !strncmp(line, (const char *)sm->p, sm->len) && line[sm->len] == '\n');
	}
	/* the message of step 21 has seq 0 of step 21 */
	p[STATE] = 2;
	CHECK(levels(&r, 21, p, D.L, S(21) + 25000) == 0);
	from = nsent;
	fdt_preg_tick(&r, V(21));
	const struct sent *s21 = find_topic("boat/b1/tx/state", from);
	CHECK(s21 && fdt_get_u32(s21->p) == 21 && fdt_get_u16(s21->p + 4) == 0 && s21->p[6] == 0x23);
	fdt_preg_free(&r);
	fclose(o.log);
	fclose(o.step_log);
	free(lb);
	free(sb);
}

/* items 3 and 7: no p(t), no uplink of step t; logged; never forwarded later; late and bad tables */
static void test_no_table(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	fdt_preg_opts_default(&o);
	o.log = memlog(&lb, &ll);
	start_at(&r, &o, 30);
	raw(&r, STATE, 48, S(29) + 15000, 1);
	fdt_preg_tick(&r, S(30));
	raw(&r, STATE, 48, V(29) + 1000, 2);
	raw(&r, IMU, 48, V(29) + 2000, 3);
	uint8_t p[FDT_L_MAX] = {2, 1, 1, 1};
	/* bad tables: wrong L, level 3 */
	uint8_t b[16];
	fdt_put_u32(b, 30);
	b[4] = 3;
	CHECK(fdt_preg_on_levels(&r, b, 8, S(30) + 1000) == -1);
	CHECK(levels(&r, 30, (const uint8_t[]){2, 1, 3, 0}, D.L, S(30) + 1000) == -1);
	int from = nsent;
	fdt_preg_tick(&r, V(30));
	CHECK(count_topic("boat/b1/tx/", from) == 0);
	/* p(30) after v(30): ignored */
	CHECK(levels(&r, 30, p, D.L, V(30) + 1) == 1 && r.s.tab_late == 1);
	fdt_preg_tick(&r, V(30) + 5000);
	CHECK(count_topic("boat/b1/tx/", from) == 0);
	/* step 31 with a table: only the messages of step 31 leave */
	raw(&r, IMU, 48, V(30) + 10000, 4);
	fdt_preg_tick(&r, S(31));
	fflush(o.log);
	CHECK(log_count(lb, 30, "state", "no_table") == 1 && log_count(lb, 30, "imu", "no_table") == 1);
	CHECK(levels(&r, 31, p, D.L, S(31) + 20000) == 0);
	from = nsent;
	fdt_preg_tick(&r, V(31));
	CHECK(count_topic("boat/b1/tx/", from) == 1);
	const struct sent *s = find_topic("boat/b1/tx/imu", from);
	CHECK(s && fdt_get_u32(s->p) == 31 && s->p[6] == 4);
	CHECK(r.s.tab_none == 2 && r.s.tab_ok == 1);   /* steps 29 and 30 without p(t) */
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);
}

static double fwd_cost(int from, const char *topic, int l)
{
	double c = 0;
	for (int i = from; i < nsent; i++)
		if (!strcmp(sent[i].topic, topic))
			c += fdt_p_cost(&D, l, sent[i].len - FDT_STEP_HEADER);
	return c;
}

/* item 4: policing to the d_il(t) of D_i(t), internal and external demand */
static void test_police(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	fdt_preg_opts_default(&o);
	o.log = memlog(&lb, &ll);
	start_at(&r, &o, 40);
	raw(&r, IMU, 48, S(39) + 15000, 1);
	fdt_preg_tick(&r, S(40));
	uint32_t dimu = fdt_preg_step(&r, 40)->d[IMU];
	CHECK(dimu == expect_d(32, 48, 0.002, 1.06, 180));   /* 5 messages of 48 bytes */
	for (int k = 0; k < 7; k++)
		raw(&r, IMU, 48, V(39) + 1000 + 1000 * k, (uint8_t)k);
	uint8_t p[FDT_L_MAX] = {0, 1, 0, 0};
	levels(&r, 40, p, D.L, S(40) + 21000);
	int from = nsent;
	fdt_preg_tick(&r, V(40));
	CHECK(count_topic("boat/b1/tx/imu", from) == 5);
	CHECK(fwd_cost(from, "boat/b1/tx/imu", IMU) <= dimu);
	fdt_preg_tick(&r, S(41));
	fflush(o.log);
	CHECK(log_count(lb, 40, "imu", "police") == 2);
	/* the policing counts the bytes: 6 messages of 10 bytes fit in the same d_l */
	for (int k = 0; k < 8; k++)
		raw(&r, IMU, 10, V(40) + 1000 + 1000 * k, (uint8_t)k);
	levels(&r, 41, p, D.L, S(41) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(41));
	CHECK(fdt_preg_step(&r, 41)->d[IMU] == dimu);
	CHECK(count_topic("boat/b1/tx/imu", from) == 6);
	CHECK(fwd_cost(from, "boat/b1/tx/imu", IMU) <= dimu);
	/* no gap: a short message after the first one over the limit stays */
	fdt_preg_tick(&r, S(42));
	raw(&r, IMU, 48, V(41) + 1000, 1);
	raw(&r, IMU, 48, V(41) + 2000, 2);
	raw(&r, IMU, 48, V(41) + 3000, 3);
	raw(&r, IMU, 48, V(41) + 4000, 4);
	raw(&r, IMU, 900, V(41) + 5000, 5);   /* over d_l alone */
	raw(&r, IMU, 1, V(41) + 6000, 6);
	levels(&r, 42, p, D.L, S(42) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(42));
	CHECK(count_topic("boat/b1/tx/imu", from) == 4);
	/* C17: the first message that would raise the sum above d_l and every later message of the topic in the
	 * step are withheld, the 1-byte message that would still fit included */
	fdt_preg_tick(&r, S(43));
	fflush(o.log);
	CHECK(log_count(lb, 42, "imu", "police") == 2);
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);

	/* external demand (fdt_dti): the regulator polices to the table it sees on boat/b1/demand */
	lb = NULL;
	fdt_preg_opts_default(&o);
	o.demand_ext = 1;
	o.log = memlog(&lb, &ll);
	start_at(&r, &o, 50);
	fdt_preg_tick(&r, S(50));
	CHECK(count_topic("boat/b1/demand", 0) == 0);
	uint32_t d[FDT_L_MAX] = {0};
	d[IMU] = (uint32_t)ceil(2 * fdt_p_cost(&D, IMU, 48));   /* two messages */
	uint8_t b[9 + 4 * FDT_L_MAX];
	size_t n = fdt_demand_encode(b, 50, 0, D.L, d);
	CHECK(fdt_preg_on_demand(&r, b, n, S(50) + 1000) == 0);
	for (int k = 0; k < 4; k++)
		raw(&r, IMU, 48, V(49) + 1000 + 1000 * k, (uint8_t)k);
	raw(&r, STATE, 48, V(49) + 9000, 9);
	levels(&r, 50, (const uint8_t[]){2, 1, 0, 0}, D.L, S(50) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(50));
	CHECK(count_topic("boat/b1/tx/imu", from) == 2 && count_topic("boat/b1/tx/state", from) == 0);
	/* step 51: the table of fdt_dti never came: nothing leaves (no_demand) */
	fdt_preg_tick(&r, S(51));
	raw(&r, IMU, 48, V(50) + 1000, 1);
	levels(&r, 51, (const uint8_t[]){2, 1, 0, 0}, D.L, S(51) + 21000);
	from = nsent;
	fdt_preg_tick(&r, S(52));
	fflush(o.log);
	CHECK(count_topic("boat/b1/tx/", from) == 0);
	CHECK(log_count(lb, 50, "imu", "police") == 2 && log_count(lb, 50, "state", "police") == 1);
	CHECK(log_count(lb, 51, "imu", "no_demand") == 1);
	/* a table of an applied step is late */
	n = fdt_demand_encode(b, 51, 0, D.L, d);
	CHECK(fdt_preg_on_demand(&r, b, n, S(52)) == 1);
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);
}

/* item 8: B5a (no policing) and B5b (no end-of-step rule) */
static void test_ablations(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	uint8_t p[FDT_L_MAX] = {2, 1, 0, 0};

	fdt_preg_opts_default(&o);
	o.mode = fdt_pmode_parse("paper_nopolice");
	CHECK(o.mode == FDT_PMODE_NOPOLICE);
	start_at(&r, &o, 60);
	raw(&r, IMU, 48, S(59) + 15000, 1);
	fdt_preg_tick(&r, S(60));
	for (int k = 0; k < 9; k++)
		raw(&r, IMU, 48, V(59) + 1000 + 1000 * k, (uint8_t)k);
	raw(&r, GPS, 80, V(59) + 20000, 1);
	levels(&r, 60, p, D.L, S(60) + 21000);
	int from = nsent;
	fdt_preg_tick(&r, V(60));
	CHECK(count_topic("boat/b1/tx/imu", from) == 9);   /* over d_l */
	CHECK(count_topic("boat/b1/tx/gps", from) == 0);   /* level 0 still held */
	fdt_preg_free(&r);

	fdt_preg_opts_default(&o);
	o.mode = fdt_pmode_parse("paper_noeos");
	o.log = memlog(&lb, &ll);
	o.idle = 0;   /* every topic claims its demand: no activity message, which would stay queued here */
	CHECK(o.mode == FDT_PMODE_NOEOS);
	start_at(&r, &o, 70);
	fdt_preg_tick(&r, S(70));
	for (int k = 0; k < 7; k++)
		raw(&r, IMU, 48, V(69) + 1000 + 1000 * k, (uint8_t)(0x10 + k));
	levels(&r, 70, p, D.L, S(70) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(70));
	CHECK(count_topic("boat/b1/tx/imu", from) == 5);
	fdt_preg_tick(&r, S(71));
	CHECK(fdt_preg_step(&r, 70) == NULL);   /* summary published */
	/* step 71: the two messages of step 70 leave first, with their own step index, under the d_l of 71 */
	for (int k = 0; k < 3; k++)
		raw(&r, IMU, 48, V(70) + 1000 + 1000 * k, (uint8_t)(0x20 + k));
	levels(&r, 71, p, D.L, S(71) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(71));
	CHECK(count_topic("boat/b1/tx/imu", from) == 5);
	if (nsent - from == 5) {
		CHECK(fdt_get_u32(sent[from].p) == 70 && fdt_get_u16(sent[from].p + 4) == 5 && sent[from].p[6] == 0x15);
		CHECK(fdt_get_u32(sent[from + 1].p) == 70 && fdt_get_u16(sent[from + 1].p + 4) == 6);
		CHECK(fdt_get_u32(sent[from + 2].p) == 71 && fdt_get_u16(sent[from + 2].p + 4) == 0);
	}
	CHECK(fdt_preg_step(&r, 71)->late_fwd[IMU] == 2 && fdt_preg_step(&r, 71)->fwd[IMU] == 3);
	/* without p(72) the messages stay queued and nothing is logged at the end of the step */
	fdt_preg_tick(&r, S(72));
	raw(&r, IMU, 48, V(71) + 1000, 0x30);
	fdt_preg_tick(&r, S(73));
	fflush(o.log);
	CHECK(ll == 0);
	levels(&r, 73, p, D.L, S(73) + 21000);
	from = nsent;
	fdt_preg_tick(&r, V(73));
	CHECK(count_topic("boat/b1/tx/imu", from) == 1);
	CHECK(nsent > from && fdt_get_u32(sent[from].p) == 72);
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);
}

/* item 6: no pacing: a large admitted volume leaves in one tick; the regulator asks for no tick in
 * phase 3; item 9: the sequence number runs per topic and step */
static void test_no_pacing(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	fdt_preg_opts_default(&o);
	o.jitter_s = 0;
	start_at(&r, &o, 80);
	raw(&r, VIDEO, 14000, S(79) + 15000, 1);
	fdt_preg_tick(&r, S(80));
	CHECK(fdt_preg_step(&r, 80)->d[VIDEO] == expect_d(30, 14000, 0, 1.06, 180));   /* 4 messages */
	for (int k = 0; k < 4; k++)
		raw(&r, VIDEO, 14000, V(79) + 1000 + 1000 * k, (uint8_t)k);
	levels(&r, 80, (const uint8_t[]){0, 0, 0, 1}, D.L, S(80) + 21000);
	fake.n = 1;
	fake.s[0].outq = 1234;
	fake.s[0].nsd = 99;
	int from = nsent;
	int64_t next = fdt_preg_tick(&r, V(80) + 300);
	CHECK(count_topic("boat/b1/tx/video", from) == 4 && next == S(81));
	for (int k = 0; k < 4 && from + k < nsent; k++)
		CHECK(fdt_get_u16(sent[from + k].p + 4) == k && sent[from + k].qos == 0);   /* level 1: QoS 0 */
	struct fdt_pstep *s = fdt_preg_step(&r, 80);
	CHECK(s->outq == 1234 && s->nsd == 99 && s->late_v == 300);
	/* the p1 offset option */
	fdt_preg_free(&r);
	fdt_preg_opts_default(&o);
	o.p1_off_us = 5000;
	start_at(&r, &o, 90);
	fdt_preg_tick(&r, S(90) + 4999);
	CHECK(count_topic("boat/b1/demand", 0) == 0);
	fdt_preg_tick(&r, S(90) + 5000);
	CHECK(count_topic("boat/b1/demand", 0) == 1);
	fdt_preg_free(&r);
}

/* the capacity of a topic and step; a jump of the clock; shutdown */
static void test_limits(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	fdt_preg_opts_default(&o);
	o.max_msgs = 3;
	o.log = memlog(&lb, &ll);
	start_at(&r, &o, 100);
	fdt_preg_tick(&r, S(100));
	for (int k = 0; k < 3; k++)
		raw(&r, IMU, 48, V(99) + 1000 * (k + 1), 1);
	memset(content, 0, 48);
	CHECK(fdt_preg_on_raw(&r, IMU, content, 48, V(99) + 5000) == 1);
	fflush(o.log);
	CHECK(log_count(lb, 100, "imu", "overflow") == 1);
	CHECK(fdt_preg_on_raw(&r, 9, content, 48, V(99) + 5000) == -1);
	/* the clock jumps by 100 steps: the queued messages are logged, the regulator starts again */
	fdt_preg_tick(&r, S(200) + 1000);
	fflush(o.log);
	CHECK(r.s.resync == 1 && log_count(lb, 100, "imu", "resync") == 3);
	CHECK(r.next_v == 200 || r.next_v == 201);
	raw(&r, GPS, 80, S(200) + 2000, 1);
	fdt_preg_free(&r);
	fflush(o.log);
	CHECK(log_count(lb, 200, "gps", "shutdown") == 1);
	fclose(o.log);
	free(lb);
}


/* ------------------------------------------------------------ baselines of the paper stack */

static size_t feedback(uint8_t *b, uint32_t t, double y, float U, int m, const int *sigma1, int L)
{
	uint64_t yv;
	uint32_t uv;
	memcpy(&yv, &y, sizeof yv);
	memcpy(&uv, &U, sizeof uv);
	fdt_put_u32(b, t);
	fdt_put_u32(b + 4, (uint32_t)(yv >> 32));
	fdt_put_u32(b + 8, (uint32_t)yv);
	fdt_put_u32(b + 12, uv);
	b[16] = (uint8_t)m;
	b[17] = (uint8_t)L;
	for (int j = 0; j < L; j++)
		b[FDT_FEEDBACK_MIN + j] = (uint8_t)sigma1[j];
	return FDT_FEEDBACK_MIN + (size_t)L;
}

static cJSON *summary_of(int64_t t, int from)
{
	for (int i = from; i < nsent; i++)
		if (!strcmp(sent[i].topic, "boat/b1/regulator")) {
			cJSON *j = cJSON_ParseWithLength((const char *)sent[i].p, sent[i].len);
			if (j && cJSON_GetObjectItem(j, "t")->valuedouble == (double)t)
				return j;
			cJSON_Delete(j);
		}
	return NULL;
}

static void test_dtp_baselines(void)
{
	char err[256];
	struct fdt_pdtp d;
	CHECK(fdt_pmode_parse("b8") == FDT_PMODE_B8 && fdt_pmode_parse("paper_b8") == FDT_PMODE_B8);
	CHECK(fdt_pmode_parse("paper_b1") == FDT_PMODE_B1 && fdt_pmode_parse("paper_tdma") == FDT_PMODE_TDMA);
	CHECK(fdt_pmode_parse("b1") < 0 && fdt_pmode_parse("tdma") < 0 && fdt_pmode_parse("b3") < 0);
	CHECK(fdt_pmode_table(FDT_PMODE_NOEOS) && !fdt_pmode_table(FDT_PMODE_B8));
	CHECK(fabs(D.U - 0.8) < 1e-12 && fabs(D.C0 - 300000) < 1e-9);
	CHECK(D.order[0] == STATE && D.order[3] == VIDEO && D.n_required == 0);
	/* classes of paper_b2 (B2): state ef (AC_VO); imu and gps af41 (AC_VI); "video" is not a stripe (no layer
	 * topic of "video_layers"): af41 */
	CHECK(fdt_pclass(&D, STATE) == 2 && fdt_pclass(&D, IMU) == 1 && fdt_pclass(&D, GPS) == 1 &&
	      fdt_pclass(&D, VIDEO) == 1);
	const char *lay = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"c_b\":1.06,\"c_m\":180,\"U\":0.7,\"vessels\":[3,1],"
	                  "\"topics\":[{\"name\":\"state\",\"rate\":8,\"bound\":48},{\"name\":\"imu\",\"rate\":32,\"bound\":48},"
	                  "{\"name\":\"video_1\",\"rate\":64,\"bound\":1400,\"jitter\":0},{\"name\":\"video_2\",\"rate\":64,"
	                  "\"bound\":1400,\"jitter\":0},{\"name\":\"video_x\",\"rate\":1,\"bound\":10}],"
	                  "\"order\":[\"imu\",\"video_1\"],\"required\":[\"state\"],"
	                  "\"video_layers\":{\"base\":\"video\",\"layers\":2}}";
	CHECK(fdt_pdtp_parse(lay, strlen(lay), &d, err, sizeof err) == 0);
	CHECK(d.order[0] == 1 && d.order[1] == 2 && d.order[2] == 0 && d.order[3] == 3 && d.order[4] == 4);
	CHECK(d.topic[2].video && d.topic[3].video && !d.topic[4].video && !d.topic[1].video);
	CHECK(fdt_pclass(&d, 0) == 2 && fdt_pclass(&d, 1) == 1 && fdt_pclass(&d, 2) == 0 && fdt_pclass(&d, 4) == 1);
	CHECK(fabs(d.U - 0.7) < 1e-12 && d.n_required == 1);
	/* the class follows the topic state, not "required": a required imu stays af41, and state is ef also when
	 * it is neither required nor first in "order" */
	const char *req = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"vessels\":[1],"
	                  "\"topics\":[{\"name\":\"imu\",\"rate\":32,\"bound\":48},{\"name\":\"video_1\",\"rate\":64,"
	                  "\"bound\":1400},{\"name\":\"state\",\"rate\":8,\"bound\":48},{\"name\":\"gps\",\"rate\":8,"
	                  "\"bound\":80}],\"order\":[\"imu\",\"video_1\",\"gps\",\"state\"],\"required\":[\"imu\"],"
	                  "\"video_layers\":{\"base\":\"video\",\"layers\":1}}";
	CHECK(fdt_pdtp_parse(req, strlen(req), &d, err, sizeof err) == 0);
	CHECK(fdt_pclass(&d, 0) == 1 && fdt_pclass(&d, 1) == 0 && fdt_pclass(&d, 2) == 2 && fdt_pclass(&d, 3) == 1);
	/* without a topic named state: the first topic of "order" is ef */
	const char *nost = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"vessels\":[1],"
	                   "\"topics\":[{\"name\":\"imu\",\"rate\":32,\"bound\":48},{\"name\":\"pose\",\"rate\":8,"
	                   "\"bound\":48},{\"name\":\"video_1\",\"rate\":64,\"bound\":1400}],\"order\":[\"pose\",\"imu\"],"
	                   "\"video_layers\":{\"base\":\"video\",\"layers\":1}}";
	CHECK(fdt_pdtp_parse(nost, strlen(nost), &d, err, sizeof err) == 0);
	CHECK(fdt_pclass(&d, 1) == 2 && fdt_pclass(&d, 0) == 1 && fdt_pclass(&d, 2) == 0);
	/* a layer topic with "jitter": 0 claims exactly one frame of 8 messages per step */
	CHECK(fdt_p_msgs(&d, 2, d.topic[2].jitter) == 8 && fdt_p_msgs(&d, 2, 0.002) == 9);
	const char *bad = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"bound\":4}],"
	                  "\"order\":[\"b\"]}";
	CHECK(fdt_pdtp_parse(bad, strlen(bad), &d, err, sizeof err) < 0);
}

/* paper_b1 and paper_b2: forwarded on arrival with QoS 1, header t of the arrival instant, no table */
static void test_b1_b2(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	fdt_preg_opts_default(&o);
	o.mode = FDT_PMODE_B1;
	o.demand_ext = 1;
	start_at(&r, &o, 40);
	fdt_preg_tick(&r, S(40));
	int from = nsent;
	raw(&r, IMU, 48, S(40) + 1000, 0x51);       /* phase 1 of step 40: leaves at once in step 40 */
	raw(&r, VIDEO, 1000, S(40) + 2000, 0x52);
	raw(&r, IMU, 48, V(40) + 1000, 0x53);
	raw(&r, STATE, 48, S(41) + 10, 0x54);       /* step 41 */
	CHECK(nsent - from == 4);
	const char *top[4] = {"boat/b1/tx/imu", "boat/b1/tx/video", "boat/b1/tx/imu", "boat/b1/tx/state"};
	const uint32_t ts[4] = {40, 40, 40, 41};
	const int seq[4] = {0, 0, 1, 0};
	for (int k = 0; k < 4 && from + k < nsent; k++) {
		CHECK(!strcmp(sent[from + k].topic, top[k]) && sent[from + k].qos == 1);
		CHECK(fdt_get_u32(sent[from + k].p) == ts[k] && fdt_get_u16(sent[from + k].p + 4) == seq[k]);
		CHECK(sent[from + k].p[FDT_STEP_HEADER] == 0x51 + k);
	}
	from = nsent;
	fdt_preg_tick(&r, S(41) + 20);
	CHECK(count_topic("boat/b1/tx/", from) == 0);   /* nothing is queued for v(t) */
	cJSON *j = summary_of(40, from);
	CHECK(j != NULL);
	if (j) {
		CHECK(cJSON_GetArrayItem(cJSON_GetObjectItem(j, "fwd"), IMU)->valueint == 2);
		CHECK(!strcmp(cJSON_GetObjectItem(j, "mode")->valuestring, "paper_b1"));
		cJSON_Delete(j);
	}
	CHECK(r.s.logged == 0);
	fdt_preg_free(&r);
	/* paper_b2: the class in the topic (state ef, the other telemetry af41) */
	o.mode = FDT_PMODE_B2;
	start_at(&r, &o, 40);
	from = nsent;
	raw(&r, STATE, 48, S(40) + 1000, 1);
	raw(&r, GPS, 80, S(40) + 2000, 2);
	raw(&r, IMU, 48, S(40) + 3000, 3);
	CHECK(find_topic("boat/b1/tx2/ef/state", from) && find_topic("boat/b1/tx2/af41/gps", from) &&
	      find_topic("boat/b1/tx2/af41/imu", from));
	CHECK(count_topic("boat/b1/tx/", from) == 0);
	fdt_preg_free(&r);
}

/* paper_b3: every message of step t at v(t), topic by topic in sigma_t, QoS 1, without a table */
static void test_b3(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	fdt_preg_opts_default(&o);
	o.mode = FDT_PMODE_B3;
	o.demand_ext = 1;
	start_at(&r, &o, 50);
	fdt_preg_tick(&r, S(50));
	int64_t a = V(49);
	raw(&r, VIDEO, 1000, a + 1000, 0x61);
	raw(&r, IMU, 48, a + 2000, 0x62);
	raw(&r, STATE, 48, a + 3000, 0x63);
	raw(&r, IMU, 48, a + 4000, 0x64);
	int from = nsent;
	fdt_preg_tick(&r, V(50));
	/* sigma of the DTP: state, imu, gps, video */
	const uint8_t tag[4] = {0x63, 0x62, 0x64, 0x61};
	CHECK(nsent - from == 4);
	for (int k = 0; k < 4 && from + k < nsent; k++)
		CHECK(sent[from + k].p[FDT_STEP_HEADER] == tag[k] && sent[from + k].qos == 1 &&
		      fdt_get_u32(sent[from + k].p) == 50);
	/* feedback with sigma = video, gps, imu, state; then the messages of step 51 follow it */
	uint8_t fb[64];
	const int sig[4] = {4, 3, 2, 1};
	CHECK(fdt_preg_on_feedback(&r, fb, feedback(fb, 50, NAN, 0, 0, sig, 4), V(50) + 10000) == 0);
	CHECK(fdt_preg_on_feedback(&r, fb, feedback(fb, 49, NAN, 0, 0, sig, 4), V(50) + 10000) == 1);
	const int dup[4] = {4, 4, 2, 1};
	CHECK(fdt_preg_on_feedback(&r, fb, feedback(fb, 51, NAN, 0, 0, dup, 4), V(50) + 10000) == -1);
	CHECK(fdt_preg_on_feedback(&r, fb, 17, V(50) + 10000) == -1);
	raw(&r, STATE, 48, V(50) + 1000, 0x71);
	raw(&r, VIDEO, 1000, V(50) + 2000, 0x72);
	fdt_preg_tick(&r, S(51));   /* end of step 50 (its summary) */
	from = nsent;
	fdt_preg_tick(&r, V(51));
	CHECK(nsent - from == 2 && sent[from].p[FDT_STEP_HEADER] == 0x72 && sent[from + 1].p[FDT_STEP_HEADER] == 0x71);
	from = nsent;
	fdt_preg_tick(&r, S(52));
	cJSON *j = summary_of(51, from);
	CHECK(j && cJSON_GetObjectItem(j, "fb_t")->valuedouble == 50 &&
	      cJSON_GetArrayItem(cJSON_GetObjectItem(j, "sigma"), 0)->valueint == 4);
	cJSON_Delete(j);
	fdt_preg_free(&r);
}

/* b8: the budget B(t) = U y T3 / (Ts m) in sigma_t; the first message that does not fit stops the rest */
static void test_b8(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	fdt_preg_opts_default(&o);
	o.mode = FDT_PMODE_B8;
	o.demand_ext = 1;
	o.log = memlog(&lb, &ll);
	start_at(&r, &o, 60);
	fdt_preg_tick(&r, S(60));
	/* before any feedback: y = C0 = 300000, U = 0.8 of the DTP, m = 2: B = 0.8 * 300000 * 0.76 / 2 = 91200 */
	int64_t a = V(59);
	for (int k = 0; k < 8; k++)
		raw(&r, VIDEO, 14000, a + 1000 + 100 * k, (uint8_t)k);
	raw(&r, STATE, 48, a + 5000, 0x80);
	int from = nsent;
	fdt_preg_tick(&r, V(60));
	double cv = fdt_p_cost(&D, VIDEO, 14000), cs = fdt_p_cost(&D, STATE, 48);
	int nv = (int)floor((91200 - cs) / cv);
	CHECK(count_topic("boat/b1/tx/state", from) == 1 && count_topic("boat/b1/tx/video", from) == nv);
	CHECK(sent[from].p[FDT_STEP_HEADER] == 0x80);   /* state first in sigma */
	from = nsent;
	fdt_preg_tick(&r, S(61));
	fflush(o.log);
	CHECK(log_count(lb, 60, "video", "budget") == 8 - nv);
	cJSON *j = summary_of(60, from);
	CHECK(j && fabs(cJSON_GetObjectItem(j, "budget")->valuedouble - 91200) < 1);
	cJSON_Delete(j);
	/* feedback: y = 100000, U = 0.5, m = 4, sigma = video first: B = 9500; state does not pass a held
	 * video message (strict priority) */
	uint8_t fb[64];
	const int sig[4] = {4, 1, 2, 3};
	CHECK(fdt_preg_on_feedback(&r, fb, feedback(fb, 60, 100000.0, 0.5f, 4, sig, 4), S(61) + 2000) == 0);
	a = V(60);
	raw(&r, VIDEO, 14000, a + 30000, 0x90);   /* cost about 15000 > B */
	raw(&r, STATE, 48, a + 31000, 0x91);
	from = nsent;
	fdt_preg_tick(&r, V(61));
	CHECK(count_topic("boat/b1/tx/", from) == 0);
	from = nsent;
	fdt_preg_tick(&r, S(62));
	fflush(o.log);
	CHECK(log_count(lb, 61, "video", "budget") == 1 && log_count(lb, 61, "state", "budget") == 1);
	j = summary_of(61, from);
	CHECK(j && fabs(cJSON_GetObjectItem(j, "budget")->valuedouble - 9500) < 1 &&
	      fabs(cJSON_GetObjectItem(j, "y")->valuedouble - 100000) < 1);
	cJSON_Delete(j);
	/* a feedback without a measurement (NaN) keeps y; a small video fits */
	CHECK(fdt_preg_on_feedback(&r, fb, feedback(fb, 61, NAN, 0.5f, 4, sig, 4), S(62) + 2000) == 0);
	raw(&r, VIDEO, 1000, V(61) + 30000, 0x92);
	raw(&r, STATE, 48, V(61) + 31000, 0x93);
	fdt_preg_tick(&r, S(62) + 3000);
	from = nsent;
	fdt_preg_tick(&r, V(62));
	CHECK(count_topic("boat/b1/tx/", from) == 2 && sent[from].p[FDT_STEP_HEADER] == 0x92);
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);
}

/* paper_tdma: vessel 2 of vessels [1, 2] sends in the second half of phase 3, paced by lw_bytes */
static void test_tdma(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *lb = NULL;
	size_t ll = 0;
	fdt_preg_opts_default(&o);
	o.mode = FDT_PMODE_TDMA;
	o.demand_ext = 1;
	o.lw_bytes = 5000;
	o.log = memlog(&lb, &ll);
	nsent = 0;
	memset(&fake, 0, sizeof fake);
	CHECK(fdt_preg_init(&r, &D, 2, &o, S(69) + 10000, &IO) == 0);
	fdt_preg_tick(&r, S(70));
	int64_t a = V(69);
	for (int k = 0; k < 6; k++)
		raw(&r, VIDEO, 2000, a + 1000 + 100 * k, (uint8_t)(0xA0 + k));
	raw(&r, STATE, 48, a + 2000, 0xB0);
	int64_t T3 = D.Ts_us - D.T1_us - D.T2_us, g0 = V(70) + T3 / 2, g1 = V(70) + T3;
	int from = nsent;
	CHECK(fdt_preg_tick(&r, V(70)) == g0);   /* the slot opens at v(70) + T3 / 2 */
	CHECK(count_topic("boat/b2/tx/", from) == 0);
	/* in the slot: one tick releases while the released bytes stay below lw_bytes (state, then 3 videos) */
	fake.n = 1;
	fake.s[0].nsd = 0;
	int64_t nx = fdt_preg_tick(&r, g0);
	CHECK(nx == g0 + 1000);
	CHECK(count_topic("boat/b2/tx/", from) == 4 && sent[from].p[FDT_STEP_HEADER] == 0xB0 && sent[from].qos == 1);
	/* the data bridge still holds unsent bytes: nothing more */
	fake.s[0].nsd = 6000;
	fdt_preg_tick(&r, g0 + 1000);
	CHECK(count_topic("boat/b2/tx/", from) == 4);
	/* the broker still holds bytes for the bridge client (bq), the socket is empty: nothing more either */
	fake.s[0].nsd = 0;
	fake.s[0].bq = 6000;
	fdt_preg_tick(&r, g0 + 1500);
	CHECK(count_topic("boat/b2/tx/", from) == 4);
	fake.s[0].bq = 0;
	fdt_preg_tick(&r, g0 + 2000);
	CHECK(count_topic("boat/b2/tx/", from) == 7);
	/* the rest of a slot stays: two more videos released after the slot are logged as slot */
	raw(&r, VIDEO, 2000, V(70) + 1000, 0xC0);   /* step 71 */
	fdt_preg_tick(&r, V(71));
	fake.s[0].nsd = 9000;
	int64_t g71 = V(71) + T3 / 2;
	fdt_preg_tick(&r, g71);
	from = nsent;
	fdt_preg_tick(&r, g71 + T3 / 2);
	CHECK(count_topic("boat/b2/tx/", from) == 0);
	CHECK(fdt_preg_tick(&r, S(72)) > S(72));
	fflush(o.log);
	CHECK(log_count(lb, 71, "video", "slot") == 1);
	(void)g1;
	fdt_preg_free(&r);
	fclose(o.log);
	free(lb);
}

/* calibration (Section VI-E of the paper): the step log gives the arrival of p(t) on the vessel clock,
 * microseconds after start(t) (p_rx_us), or the arrival of a table after v(t) (p_late_us) */
static double step_field(const char *text, int64_t t, const char *key, int *is_null)
{
	char head[48];
	snprintf(head, sizeof head, "{\"t\":%lld,", (long long)t);
	const char *line = text ? strstr(text, head) : NULL;
	*is_null = -1;
	if (!line)
		return NAN;
	const char *e = strchr(line, '\n');
	cJSON *o = cJSON_ParseWithLength(line, e ? (size_t)(e - line) : strlen(line));
	double v = NAN;
	const cJSON *x = o ? cJSON_GetObjectItem(o, key) : NULL;
	if (x) {
		*is_null = cJSON_IsNull(x);
		if (cJSON_IsNumber(x))
			v = x->valuedouble;
	}
	cJSON_Delete(o);
	return v;
}

static void test_table_arrival(void)
{
	struct fdt_preg r;
	struct fdt_preg_opts o;
	char *sb = NULL;
	size_t sl = 0;
	fdt_preg_opts_default(&o);
	o.step_log = memlog(&sb, &sl);
	start_at(&r, &o, 40);
	uint8_t p[FDT_L_MAX] = {2, 1, 1, 0};
	fdt_preg_tick(&r, S(40));
	/* step 40: the table arrives 23.5 ms after start(40), before v(40) */
	CHECK(levels(&r, 40, p, D.L, S(40) + 23500) == 0);
	fdt_preg_tick(&r, V(40));
	fdt_preg_tick(&r, S(41));
	/* step 41: the table arrives 2 ms after v(41) and is withheld; a second late copy keeps the first time */
	fdt_preg_tick(&r, V(41));
	CHECK(levels(&r, 41, p, D.L, V(41) + 2000) == 1);
	CHECK(levels(&r, 41, p, D.L, V(41) + 5000) == 1);
	fdt_preg_tick(&r, S(42));
	/* step 42: no table */
	fdt_preg_tick(&r, S(43));
	fflush(o.step_log);
	int nul;
	CHECK(step_field(sb, 40, "p_rx_us", &nul) == 23500 && nul == 0);
	CHECK(isnan(step_field(sb, 40, "p_late_us", &nul)) && nul == 1);
	CHECK(isnan(step_field(sb, 41, "p_rx_us", &nul)) && nul == 1);
	CHECK(step_field(sb, 41, "p_late_us", &nul) == (double)(V(41) + 2000 - S(41)) && nul == 0);
	CHECK(isnan(step_field(sb, 42, "p_rx_us", &nul)) && nul == 1);
	CHECK(isnan(step_field(sb, 42, "p_late_us", &nul)) && nul == 1);
	fdt_preg_free(&r);
	fclose(o.step_log);
	free(sb);
}

int main(void)
{
	test_dtp();
	test_clock();
	test_demand();
	test_ring();
	test_backlog();
	test_backlog_full_socket();
	test_demand_wire();
	test_forward();
	test_no_table();
	test_police();
	test_ablations();
	test_no_pacing();
	test_limits();
	test_dtp_baselines();
	test_b1_b2();
	test_b3();
	test_b8();
	test_tdma();
	test_table_arrival();
	if (failures) {
		fprintf(stderr, "test_reg_paper: %d of %d checks failed\n", failures, checks);
		return 1;
	}
	printf("test_reg_paper: %d checks passed\n", checks);
	return 0;
}
