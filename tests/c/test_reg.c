/*
 * Unit tests of the DTP v2 reader, the step clock and the core of the v2 vessel regulator
 * (fdt_reg.c): output assembly, demand, table timing, pacing against injected SIOCOUTQNSD values,
 * release order, discard counting, held bytes from the release offsets, and the modes.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cjson/cJSON.h>

#include "fdt_dtp2.h"
#include "fdt_reg.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); } } while (0)

static const char *DTP =
	"{\"epoch\": null, \"Ts\": 0.125, \"T1\": 0.020, \"T2\": 0.010,"
	" \"eps\": 0.002, \"d_rx\": 0.002, \"d_m\": 0.001,"
	" \"T_tick\": 0.001, \"L_w\": 16384, \"B_cap\": 1048576, \"T_min\": 0.005,"
	" \"beta\": 0.05, \"gamma\": 0.01, \"alpha1\": 0.5, \"lambda\": 0.25,"
	" \"phy\": {\"slot\": 9e-6, \"sifs\": 10e-6, \"t_pre\": 16e-6, \"t_sig\": 4e-6, \"t_sym\": 4e-6, \"t_se\": 6e-6,"
	"   \"h_mac\": 38, \"aifsn_sta\": 3, \"cwmin_sta\": 15, \"cwmax_sta\": 1023, \"m_b\": 6,"
	"   \"aifsn_ap\": 1, \"cwmin_ap\": 7, \"cwmax_ap\": 1023, \"basic_rates\": [6e6, 12e6, 24e6], \"rate_init\": 24e6},"
	" \"tcp\": {\"mss\": 1448, \"h_T\": 80}, \"mqtt\": {\"h_A\": 4},"
	" \"vessels\": [1, 2],"
	" \"topics\": ["
	"  {\"name\": \"state\", \"rate\": 8,  \"jitter\": 0.002, \"n\": 1,  \"s\": 48,   \"kind\": \"stream\", \"qos\": 2},"
	"  {\"name\": \"imu\",   \"rate\": 32, \"jitter\": 0.002, \"n\": 1,  \"s\": 48,   \"kind\": \"stream\", \"qos\": 0},"
	"  {\"name\": \"gps\",   \"rate\": 8,  \"jitter\": 0.002, \"n\": 1,  \"s\": 80,   \"kind\": \"stream\", \"qos\": 0},"
	"  {\"name\": \"video\", \"rate\": 15, \"jitter\": 0.010, \"n\": 16, \"s\": 1400, \"kind\": \"output\", \"qos\": 0}],"
	" \"required\": [\"state\"], \"goals\": {\"rate\": 1, \"s\": 200},"
	" \"order\": [\"state\", \"imu\", \"video\", \"gps\"], \"needed\": [\"state\", \"imu\"]}";

enum { STATE = 0, IMU = 1, GPS = 2, VIDEO = 3 };

static struct fdt_dtp2 D;

/* ------------------------------------------------------------ fake broker */

struct sent {
	char topic[128];
	int qos;
	uint8_t p[2048];
	size_t len;
};
static struct sent sent[8192];
static int nsent;

static int pub(void *ctx, const char *topic, int qos, const void *p, size_t len)
{
	(void)ctx;
	struct sent *s = &sent[nsent++];
	snprintf(s->topic, sizeof s->topic, "%s", topic);
	s->qos = qos;
	s->len = len < sizeof s->p ? len : sizeof s->p;
	memcpy(s->p, p, s->len);
	s->p[s->len < sizeof s->p ? s->len : sizeof s->p - 1] = 0;
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

static const struct sent *last_on(const char *topic)
{
	for (int i = nsent - 1; i >= 0; i--)
		if (!strcmp(sent[i].topic, topic))
			return &sent[i];
	return NULL;
}

static cJSON *last_json(const char *topic)
{
	const struct sent *s = last_on(topic);
	return s ? cJSON_ParseWithLength((const char *)s->p, s->len) : NULL;
}

static int64_t jint(const cJSON *o, const char *k)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
	return cJSON_IsNumber(v) ? (int64_t)v->valuedouble : -999999;
}

/* ------------------------------------------------------------ helpers */

static void stream_msg(struct fdt_reg *r, int l, size_t len, int64_t at)
{
	uint8_t b[2048];
	memset(b, 0x5a, sizeof b);
	CHECK(fdt_reg_on_raw(r, l, b, len, at) == 0);
}

static int out_msg(struct fdt_reg *r, uint32_t id, int k, int n, size_t len, int64_t at)
{
	uint8_t b[2048];
	memset(b, 0xa5, sizeof b);
	fdt_put_u32(b, id);
	fdt_put_u16(b + 4, (uint16_t)k);
	fdt_put_u16(b + 6, (uint16_t)n);
	return fdt_reg_on_raw(r, VIDEO, b, len, at);
}

static void whole_output(struct fdt_reg *r, uint32_t id, int n, size_t len, int64_t at)
{
	for (int k = 0; k < n; k++)
		CHECK(out_msg(r, id, k, n, len, at) == 0);
}

static void table(struct fdt_reg *r, int64_t t, const int *p, const int *k, const int *o, int64_t at, int expect)
{
	char b[512];
	int n = snprintf(b, sizeof b, "{\"t\":%" PRId64 ",\"p\":[%d,%d,%d,%d],\"k\":[%d,%d,%d,%d],\"o\":[%d,%d,%d,%d]}", t,
	                 p[0], p[1], p[2], p[3], k[0], k[1], k[2], k[3], o[0], o[1], o[2], o[3]);
	CHECK(fdt_reg_on_levels(r, b, (size_t)n, at) == expect);
}

static int64_t S(int64_t t) { return fdt2_start_us(&D, t); }
static int64_t V(int64_t t) { return fdt2_v_us(&D, t); }

/* runs ticks every T_tick from a to b (inclusive) */
static void run(struct fdt_reg *r, int64_t a, int64_t b)
{
	for (int64_t now = a; now <= b; now += D.T_tick_us)
		fdt_reg_tick(r, now);
}

/* step header of the released messages from index i0 */
static int tx_of(int i0, const char *suffix, int *idx, int max)
{
	int n = 0;
	for (int i = i0; i < nsent && n < max; i++)
		if (strstr(sent[i].topic, "/tx/") && (!suffix || !strcmp(strrchr(sent[i].topic, '/') + 1, suffix)))
			idx[n++] = i;
	return n;
}

/* ------------------------------------------------------------ tests */

static void test_dtp(void)
{
	char err[256];
	CHECK(fdt2_dtp_parse(DTP, strlen(DTP), &D, err, sizeof err) == 0);
	if (failures) {
		fprintf(stderr, "%s\n", err);
		exit(1);
	}
	CHECK(D.L == 4 && D.M == 2 && D.n_required == 1 && D.required[0] == STATE);
	CHECK(D.epoch_us == 1767225600000000LL && D.Ts_us == 125000 && D.T1_us == 20000 && D.T2_us == 10000);
	CHECK(D.eps_us == 2000 && D.d_rx_us == 2000 && D.T_tick_us == 1000 && D.L_w_bytes == 16384);
	CHECK(D.topic[VIDEO].kind == FDT2_OUTPUT && D.topic[VIDEO].n == 16 && D.topic[VIDEO].s == 1400);
	CHECK(D.topic[STATE].qos == 2 && D.topic[IMU].qos == 0);
	CHECK(D.order[0] == STATE && D.order[1] == IMU && D.order[2] == VIDEO && D.order[3] == GPS);
	CHECK(D.mss == 1448 && D.h_T == 80 && D.h_A == 4 && D.phy.n_basic == 3);
	/* h_P with the largest vessel index (2): fixed header + 2 + len(boat/b2/tx/<name>) + mid + 6 */
	CHECK(fdt2_h_P(&D, STATE) == 1 + 1 + 2 + 16 + 2 + 6);
	CHECK(fdt2_h_P(&D, VIDEO) == 1 + 2 + 2 + 16 + 0 + 6);
	CHECK(fdt2_publish_bytes(16, 2, 54) == 76);
	CHECK(fdt2_varint_bytes(127) == 1 && fdt2_varint_bytes(128) == 2 && fdt2_varint_bytes(16384) == 3);
	/* arrival bounds ceil(r (dur + J)) */
	CHECK(fdt2_arrival_outputs(&D, STATE, 30000) == 1);   /* 8 * 0.032 */
	CHECK(fdt2_arrival_outputs(&D, IMU, 30000) == 2);     /* 32 * 0.032 = 1.024 */
	CHECK(fdt2_arrival_outputs(&D, VIDEO, 30000) == 1);   /* 15 * 0.040 */
	CHECK(fdt2_arrival_outputs(&D, STATE, 125000) == 2);  /* 8 * 0.127 */
	CHECK(fdt2_arrival_outputs(&D, VIDEO, 125000) == 3);  /* 15 * 0.135 = 2.025 */
	/* step clock: step t = [epoch + (t-1) Ts, epoch + t Ts) */
	CHECK(S(1) == D.epoch_us && fdt2_step_at(&D, S(1)) == 1 && fdt2_step_at(&D, S(1) - 1) == 0);
	CHECK(fdt2_collect_step(&D, V(7) - 1) == 7 && fdt2_collect_step(&D, V(7)) == 8);
	CHECK(fdt2_collect_step(&D, S(7)) == 7 && fdt2_collect_step(&D, V(6)) == 7);
	CHECK(fdt2_w_start_us(&D, 7) == S(7) + 32000 && fdt2_w_end_us(&D, 7) == S(7) + 121000);
	/* validation */
	struct fdt_dtp2 bad;
	const char *q = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"s\":10,"
	                "\"kind\":\"stream\",\"qos\":0}],\"required\":[\"a\"]}";
	CHECK(fdt2_dtp_parse(q, strlen(q), &bad, err, sizeof err) == -1 && strstr(err, "qos"));
	const char *q2 = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"s\":10,"
	                 "\"kind\":\"stream\",\"qos\":2}]}";
	CHECK(fdt2_dtp_parse(q2, strlen(q2), &bad, err, sizeof err) == -1);
	const char *o = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"s\":10,"
	                "\"kind\":\"stream\",\"qos\":2},{\"name\":\"b\",\"rate\":1,\"s\":10,\"kind\":\"stream\",\"qos\":0}],"
	                "\"required\":[\"a\"],\"order\":[\"b\",\"a\"]}";
	CHECK(fdt2_dtp_parse(o, strlen(o), &bad, err, sizeof err) == -1 && strstr(err, "order"));
	const char *ok = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"s\":10,"
	                 "\"kind\":\"stream\",\"qos\":2},{\"name\":\"b\",\"rate\":1,\"s\":10,\"kind\":\"output\",\"qos\":0}],"
	                 "\"required\":[\"a\"]}";
	CHECK(fdt2_dtp_parse(ok, strlen(ok), &bad, err, sizeof err) == 0 && bad.n_order == 2 && bad.order[0] == 0);
	const char *kind = "{\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"s\":10,"
	                   "\"kind\":\"blob\",\"qos\":0}]}";
	CHECK(fdt2_dtp_parse(kind, strlen(kind), &bad, err, sizeof err) == -1);
	const char *tw = "{\"Ts\":0.125,\"T1\":0.06,\"T2\":0.06,\"eps\":0.002,\"d_rx\":0.002,\"topics\":[{\"name\":\"a\","
	                 "\"rate\":1,\"s\":10,\"kind\":\"stream\",\"qos\":0}]}";
	CHECK(fdt2_dtp_parse(tw, strlen(tw), &bad, err, sizeof err) == -1);
}

/* the regulator starts in phase 3 of step t0 - 1, so step t0 collects from now on */
static void start(struct fdt_reg *r, int mode, int64_t t0)
{
	nsent = 0;
	memset(&fake, 0, sizeof fake);
	CHECK(fdt_reg_init(r, &D, 2, mode, 0, V(t0 - 1) + 5000, &IO) == 0);
	CHECK(r->next_v == t0 && r->next_p1 == t0);
}

static void test_assembly(void)
{
	struct fdt_reg *r = calloc(1, sizeof *r);
	int64_t t = 1000;
	start(r, FDT_MODE_FLEETDT, t);
	int64_t a = V(t - 1) + 10000;
	/* output 7 out of order completes in C(t) */
	CHECK(out_msg(r, 7, 2, 3, 500, a) == 0);
	CHECK(out_msg(r, 7, 0, 3, 500, a + 10) == 0);
	CHECK(r->nopen[VIDEO] == 1);
	CHECK(out_msg(r, 7, 0, 3, 500, a + 20) == 1);   /* repeated message */
	CHECK(out_msg(r, 7, 1, 3, 300, a + 30) == 0);
	CHECK(r->nopen[VIDEO] == 0);
	struct fdt_step *s = fdt_reg_step(r, t);
	CHECK(s && s->tp[VIDEO].no == 1 && s->tp[VIDEO].o[0].x == 1300 && s->tp[VIDEO].rx == 3);
	/* malformed: n over the DTP, k >= n, short header */
	CHECK(out_msg(r, 8, 0, 17, 100, a) == -1);
	CHECK(out_msg(r, 8, 3, 3, 100, a) == -1);
	uint8_t shortb[4] = {0};
	CHECK(fdt_reg_on_raw(r, VIDEO, shortb, 4, a) == -1);
	/* output 9 starts in C(t) and completes in C(t+1): belongs to t+1 */
	CHECK(out_msg(r, 9, 0, 2, 100, V(t) - 100) == 0);
	/* output 10 starts in C(t) and never completes */
	CHECK(out_msg(r, 10, 0, 2, 100, V(t) - 50) == 0);
	run(r, S(t), V(t) + 1000);
	CHECK(r->nopen[VIDEO] == 2);   /* 9 and 10 survive v(t): first message in C(t) */
	CHECK(out_msg(r, 9, 1, 2, 100, V(t) + 2000) == 0);
	struct fdt_step *s1 = fdt_reg_step(r, t + 1);
	CHECK(s1 && s1->tp[VIDEO].no == 1 && s1->tp[VIDEO].o[0].id == 9);
	CHECK(fdt_reg_step(r, t)->tp[VIDEO].no == 1);
	/* at v(t+1), output 10 (first message in C(t)) is older than one step: dropped and counted */
	run(r, V(t) + 1000, V(t + 1) + 1000);
	CHECK(r->nopen[VIDEO] == 0);
	CHECK(fdt_reg_step(r, t + 1)->tp[VIDEO].inc == 1 && r->st_.inc == 1);
	/* stream messages follow the collection window */
	stream_msg(r, IMU, 40, V(t + 1) + 2000);
	CHECK(fdt_reg_step(r, t + 2) && fdt_reg_step(r, t + 2)->tp[IMU].nm == 1);
	fdt_reg_free(r);
	free(r);
}

static void test_demand_and_table(void)
{
	struct fdt_reg *r = calloc(1, sizeof *r);
	int64_t t = 2000;
	start(r, FDT_MODE_FLEETDT, t);
	int64_t a = V(t - 1) + 1000;
	stream_msg(r, STATE, 48, a);
	stream_msg(r, IMU, 40, a + 100);
	stream_msg(r, IMU, 44, a + 200);
	stream_msg(r, IMU, 48, a + 300);
	whole_output(r, 1, 4, 1000, a + 400);
	whole_output(r, 2, 2, 700, a + 500);
	/* B: three packets written on the data socket after 100 bytes of CONNACK/SUBACK */
	fake.n = 1;
	fake.s[0].has_acked = 1;
	fdt_reg_on_out(r, 0, 2, 76, 54, 100, 1);    /* [100, 176) qos 2, content 48 */
	fdt_reg_on_out(r, 0, 0, 1427, 1406, 176, 1);  /* [176, 1603) qos 0, content 1400 */
	fdt_reg_on_out(r, 0, 0, 1427, 1406, 1603, 1); /* [1603, 3030) */
	fdt_reg_on_out(r, 0, 2, 76, 54, 3034, 1);   /* a PUBREL of 4 bytes before it: [3034, 3110) */
	CHECK(r->trk[0].model_end == 3110 && r->trk[0].n == 4);
	fake.s[0].acked = 1603 + 1000;               /* 427 bytes of the third packet unacknowledged */
	fake.s[0].outq = 3110 - 2603;
	fake.s[0].nsd = 0;
	int i0 = nsent;
	run(r, S(t), S(t) + 1);                      /* phase 1 of step t */
	cJSON *dj = last_json("boat/b2/demand");
	CHECK(dj != NULL && count_topic("boat/b2/demand", i0) == 1 && last_on("boat/b2/demand")->qos == 1);
	CHECK(jint(dj, "t") == t && jint(dj, "v") == 2 && jint(dj, "late_us") == 0);
	const cJSON *d = cJSON_GetObjectItemCaseSensitive(dj, "d");
	CHECK(cJSON_GetArraySize(d) == 4);
	/* state: held (48, 1) + arrival ceil(8 * 0.032) = 1 message of 48 */
	const cJSON *ds = cJSON_GetArrayItem(cJSON_GetArrayItem(d, STATE), 0);
	CHECK(cJSON_GetArrayItem(ds, 0)->valuedouble == 96 && cJSON_GetArrayItem(ds, 1)->valuedouble == 2);
	/* imu: held (132, 3) + 2 x 48 */
	const cJSON *di = cJSON_GetArrayItem(cJSON_GetArrayItem(d, IMU), 0);
	CHECK(cJSON_GetArrayItem(di, 0)->valuedouble == 132 + 96 && cJSON_GetArrayItem(di, 1)->valuedouble == 5);
	/* gps: nothing held: 1 x 80 */
	const cJSON *dg = cJSON_GetArrayItem(cJSON_GetArrayItem(d, GPS), 0);
	CHECK(cJSON_GetArrayItem(dg, 0)->valuedouble == 80 && cJSON_GetArrayItem(dg, 1)->valuedouble == 1);
	/* video: two held outputs (4000, 4), (1400, 2), then ceil(15 * 0.04) = 1 output of (16 * 1400, 16) */
	const cJSON *dv = cJSON_GetArrayItem(d, VIDEO);
	CHECK(cJSON_GetArraySize(dv) == 3);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(dv, 0), 0)->valuedouble == 4000);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(dv, 0), 1)->valuedouble == 4);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(dv, 1), 0)->valuedouble == 1400);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(dv, 2), 0)->valuedouble == 22400);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(dv, 2), 1)->valuedouble == 16);
	const cJSON *held = cJSON_GetObjectItemCaseSensitive(dj, "held");
	CHECK(cJSON_GetArrayItem(held, VIDEO)->valuedouble == 2 && cJSON_GetArrayItem(held, STATE)->valuedouble == 0);
	/* B: packet 3 has 427 bytes unacknowledged (all content: 427 < 1400), packet 4 entirely */
	const cJSON *B = cJSON_GetObjectItemCaseSensitive(dj, "B");
	CHECK(jint(B, "x0") == 427 && jint(B, "n0") == 1 && jint(B, "x2") == 48 && jint(B, "n2") == 1);
	CHECK(r->trk[0].n == 2);   /* the acknowledged packets left the tracker */
	const cJSON *rep = cJSON_GetObjectItemCaseSensitive(dj, "rep");
	CHECK(jint(rep, "t") == t - 1 && cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(rep, "tab")));
	cJSON_Delete(dj);

	/* table of t before v(t): state 2, imu 2, video 1 with k = 1, gps 0; order state, video, imu, gps */
	int p[4] = {2, 2, 0, 1}, k[4] = {1, 1, 0, 1}, o[4] = {STATE, VIDEO, IMU, GPS};
	table(r, t, p, k, o, S(t) + 25000, 0);
	stream_msg(r, IMU, 48, S(t) + 26000);   /* still C(t): beyond the demand (5 messages) is not reached */
	fake.s[0].nsd = 0;
	fake.s[0].outq = 0;
	fake.s[0].acked = 3110;
	i0 = nsent;
	run(r, S(t) + 1000, V(t) + 5000);
	struct fdt_step *s = fdt_reg_step(r, t);
	CHECK(s->have_tab && s->applied);
	int idx[64];
	int n = tx_of(i0, NULL, idx, 64);
	/* level 2 in o order: state (1), imu (4); then level 1: video output 1 (4 messages) */
	CHECK(n == 9);
	CHECK(!strcmp(sent[idx[0]].topic, "boat/b2/tx/state") && sent[idx[0]].qos == 2);
	for (int j = 1; j <= 4; j++)
		CHECK(!strcmp(sent[idx[j]].topic, "boat/b2/tx/imu") && sent[idx[j]].qos == 0);
	for (int j = 5; j <= 8; j++)
		CHECK(!strcmp(sent[idx[j]].topic, "boat/b2/tx/video"));
	/* payload header t, seq; video messages in output index order */
	CHECK(fdt_get_u32(sent[idx[0]].p) == (uint32_t)t && fdt_get_u16(sent[idx[0]].p + 4) == 0);
	CHECK(fdt_get_u16(sent[idx[4]].p + 4) == 3);
	for (int j = 5; j <= 8; j++) {
		CHECK(fdt_get_u16(sent[idx[j]].p + 4) == j - 5);
		CHECK(fdt_get_u32(sent[idx[j]].p + 6) == 1 && fdt_get_u16(sent[idx[j]].p + 10) == j - 5);
		CHECK(sent[idx[j]].len == 6 + 1000);
	}
	CHECK(s->rel[VIDEO] == 4 && s->wh[VIDEO] == 2 && s->wh[GPS] == 0 && s->relu[VIDEO][0] == 4);
	/* a table that arrives late is ignored */
	table(r, t, p, k, o, V(t) + 6000, 1);
	CHECK(r->st_.tab_late == 1);

	/* step t+1: no table before v(t+1) -> default table (state only, arrival bound over Ts) */
	stream_msg(r, STATE, 48, V(t) + 7000);
	stream_msg(r, STATE, 48, V(t) + 8000);
	stream_msg(r, STATE, 48, V(t) + 9000);   /* third state: over the bound ceil(8 * 0.127) = 2 */
	stream_msg(r, IMU, 48, V(t) + 9000);
	whole_output(r, 3, 2, 500, V(t) + 9500);
	i0 = nsent;
	run(r, V(t) + 5000, S(t + 1) + 1);
	dj = last_json("boat/b2/demand");
	rep = cJSON_GetObjectItemCaseSensitive(dj, "rep");
	CHECK(jint(dj, "t") == t + 1 && jint(rep, "t") == t && cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(rep, "tab")));
	const cJSON *rel = cJSON_GetObjectItemCaseSensitive(rep, "rel");
	CHECK(cJSON_GetArraySize(cJSON_GetArrayItem(rel, STATE)) == 1 &&
	      cJSON_GetArrayItem(cJSON_GetArrayItem(rel, STATE), 0)->valuedouble == 1);
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(rel, IMU), 0)->valuedouble == 4);
	CHECK(cJSON_GetArraySize(cJSON_GetArrayItem(rel, VIDEO)) == 1 &&
	      cJSON_GetArrayItem(cJSON_GetArrayItem(rel, VIDEO), 0)->valuedouble == 4);
	CHECK(cJSON_GetArraySize(cJSON_GetArrayItem(rel, GPS)) == 0);
	CHECK(jint(rep, "disc") == 0);
	cJSON_Delete(dj);
	cJSON *lg = last_json("boat/b2/regulator");
	CHECK(lg && jint(lg, "t") == t && cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(lg, "have_tab")));
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "rx"), IMU)->valuedouble == 4);
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "xrel"), VIDEO)->valuedouble == 4000);
	CHECK(jint(lg, "outq_p1") == 507);
	cJSON_Delete(lg);
	i0 = nsent;
	run(r, S(t + 1) + 1, S(t + 2) + 1);
	n = tx_of(i0, NULL, idx, 64);
	CHECK(n == 2);
	CHECK(!strcmp(sent[idx[0]].topic, "boat/b2/tx/state") && !strcmp(sent[idx[1]].topic, "boat/b2/tx/state"));
	s = fdt_reg_step(r, t + 2);
	dj = last_json("boat/b2/demand");
	rep = cJSON_GetObjectItemCaseSensitive(dj, "rep");
	CHECK(jint(rep, "t") == t + 1 && cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(rep, "tab")));
	CHECK(cJSON_GetArrayItem(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(rep, "rel"), STATE), 0)->valuedouble == 2);
	cJSON_Delete(dj);
	lg = last_json("boat/b2/regulator");
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "wh"), STATE)->valuedouble == 1);
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "wh"), VIDEO)->valuedouble == 2);
	cJSON_Delete(lg);
	CHECK(r->st_.tab_default >= 1);
	fdt_reg_free(r);
	free(r);
}

/* with a phase 1 offset (vessel 2 of 2: 2 (T1 - 4 ms) / 3 after start(t)), step t still ends at start(t + 1): the
 * messages that L_w holds back at the end of the step are discarded, not released in phase 1 of t + 1 */
static void test_close_at_step_end(void)
{
	struct fdt_reg *r = calloc(1, sizeof *r);
	int64_t t = 3500;
	start(r, FDT_MODE_FLEETDT, t);
	fdt_reg_set_p1_offset(r, -1);
	int64_t off = r->p1_off_us;
	CHECK(off == 2 * (D.T1_us - 4000) / 3);
	int64_t a = V(t - 1) + 1000;
	stream_msg(r, STATE, 48, a);
	whole_output(r, 200, 16, 1400, a + 100);
	fake.n = 1;
	run(r, S(t), S(t) + off);
	int p[4] = {2, 0, 0, 1}, k[4] = {1, 0, 0, 1}, o[4] = {STATE, IMU, VIDEO, GPS};
	table(r, t, p, k, o, S(t) + off + 2000, 0);
	fake.s[0].nsd = 16384;                       /* L_w holds every message back until the end of W(t) */
	run(r, V(t), S(t + 1) - 1000);
	struct fdt_step *s = fdt_reg_step(r, t);
	CHECK(s && s->irl == 0 && s->nrl == 17 && !s->closed);
	fake.s[0].nsd = 0;                           /* room again, after start(t + 1) and before phase 1 */
	r->pending = 0;
	int i0 = nsent;
	fdt_reg_tick(r, S(t + 1) + 1);
	CHECK(s->closed && s->irl == s->nrl && tx_of(i0, NULL, (int[1]){0}, 1) == 0);
	run(r, S(t + 1) + 1000, S(t + 1) + off + 1000);
	cJSON *dj = last_json("boat/b2/demand");
	CHECK(dj && jint(dj, "t") == t + 1 && jint(cJSON_GetObjectItemCaseSensitive(dj, "rep"), "disc") == 17);
	cJSON_Delete(dj);
	fdt_reg_free(r);
	free(r);
}

static void test_pacing_and_discard(void)
{
	struct fdt_reg *r = calloc(1, sizeof *r);
	int64_t t = 3000;
	start(r, FDT_MODE_FLEETDT, t);
	int64_t a = V(t - 1) + 1000;
	stream_msg(r, STATE, 48, a);
	for (int j = 0; j < 3; j++)
		whole_output(r, 100 + j, 16, 1400, a + 100 + j);
	fake.n = 1;
	run(r, S(t), S(t) + 1);
	int p[4] = {2, 0, 0, 1}, k[4] = {1, 0, 0, 3}, o[4] = {STATE, IMU, VIDEO, GPS};
	table(r, t, p, k, o, S(t) + 25000, 0);
	/* each video message: PUBLISH of 1 + 2 + 2 + 16 + 6 + 1400 = 1427 bytes; state 1+1+2+16+2+54 = 76 */
	int i0 = nsent;
	fake.s[0].nsd = 16384;   /* the socket holds L_w unsent bytes: nothing leaves */
	CHECK(fdt_reg_tick(r, V(t)) == V(t) + 1000);
	CHECK(tx_of(i0, NULL, (int[1]){0}, 1) == 0);
	struct fdt_step *s = fdt_reg_step(r, t);
	CHECK(s->nrl == 1 + 48 && s->irl == 0);
	fake.s[0].nsd = 16000;   /* below L_w: state (16000 < L_w), video (16076 < L_w), then 17503 >= L_w */
	r->pending = 0;
	fdt_reg_tick(r, V(t) + 1000);
	CHECK(s->irl == 2);
	/* no MESSAGE_OUT confirmed the 1503 released bytes: they count as unsent, 17503 >= L_w */
	fdt_reg_tick(r, V(t) + 2000);
	CHECK(s->irl == 2);
	r->pending = 0;
	fake.s[0].nsd = 0;       /* empty socket: messages while released bytes stay below L_w */
	fdt_reg_tick(r, V(t) + 3000);
	CHECK(s->irl == 2 + 12); /* 11 x 1427 = 15697 < 16384, the twelfth reaches 17124 */
	r->pending = 0;
	fake.s[0].nsd = 10000;
	fdt_reg_tick(r, V(t) + 4000);
	CHECK(s->irl == 14 + 5); /* 10000 + 4 x 1427 = 15708 < 16384 -> fifth; 17135 stops */
	/* the broker holds packets it has not written (EAGAIN): MESSAGE_OUT offsets beyond the socket */
	r->pending = 0;
	fake.s[0].nsd = 0;
	fake.s[0].has_acked = 1;
	fake.s[0].acked = 0;
	fake.s[0].outq = 0;
	fdt_reg_on_out(r, 0, 0, 16000, 16000, 0, 1);
	fdt_reg_tick(r, V(t) + 5000);
	CHECK(s->irl == 19 + 1); /* 16000 backlog < 16384: one message */
	CHECK(s->tick_us_max == 1000);
	/* end of step: the 29 unreleased video messages of admitted outputs are discarded */
	fake.s[0].has_acked = 0;
	r->pending = 0;
	fake.s[0].nsd = 16384;
	run(r, V(t) + 6000, S(t + 1) + 1);
	CHECK(s->t == -1 || fdt_reg_step(r, t) == NULL);
	cJSON *dj = last_json("boat/b2/demand");
	const cJSON *rep = cJSON_GetObjectItemCaseSensitive(dj, "rep");
	CHECK(jint(rep, "t") == t && jint(rep, "disc") == 29);
	const cJSON *rv = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(rep, "rel"), VIDEO);
	CHECK(cJSON_GetArraySize(rv) == 3 && cJSON_GetArrayItem(rv, 0)->valuedouble == 16 &&
	      cJSON_GetArrayItem(rv, 1)->valuedouble == 3 && cJSON_GetArrayItem(rv, 2)->valuedouble == 0);
	cJSON_Delete(dj);
	cJSON *lg = last_json("boat/b2/regulator");
	CHECK(cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "disc"), VIDEO)->valuedouble == 29);
	CHECK(jint(lg, "nsd_max") == 16384);
	cJSON_Delete(lg);
	CHECK(r->st_.discarded == 29);

	/* policing: an admitted output larger than its unit (arrival bound n s) is not released, and an
	 * output whose demand was held exactly is released whole */
	whole_output(r, 200, 2, 1400, V(t) + 1000 + 125000);   /* C(t+2) */
	run(r, S(t + 1) + 1, S(t + 2) + 1);                    /* demand of t+2 holds (2800, 2) */
	/* arrives after phase 1, so its unit is the arrival bound (16 x 1400, 16): 16 x 1500 exceeds it */
	whole_output(r, 201, 16, 1500, S(t + 2) + 5000);
	int p2[4] = {2, 0, 0, 1}, k2[4] = {1, 0, 0, 2};
	table(r, t + 2, p2, k2, o, S(t + 2) + 20000, 0);
	fake.s[0].nsd = 0;
	i0 = nsent;
	run(r, S(t + 2) + 20000, S(t + 3) + 1);
	int idx[64];
	CHECK(tx_of(i0, "video", idx, 64) == 2);
	lg = last_json("boat/b2/regulator");
	CHECK(jint(lg, "t") == t + 2 && cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(lg, "disc"), VIDEO)->valuedouble == 16);
	cJSON_Delete(lg);
	fdt_reg_free(r);
	free(r);
}

static void test_modes(void)
{
	struct fdt_reg *r = calloc(1, sizeof *r);
	int idx[256];
	/* b1: forward at once, no table */
	int64_t t = 4000;
	start(r, FDT_MODE_B1, t);
	stream_msg(r, GPS, 80, V(t - 1) + 100);
	CHECK(nsent == 1 && !strcmp(sent[0].topic, "boat/b2/tx/gps") && fdt_get_u32(sent[0].p) == (uint32_t)t);
	CHECK(out_msg(r, 5, 0, 2, 100, V(t - 1) + 200) == 0 && nsent == 2);
	fdt_reg_free(r);
	/* b2: one data bridge per class */
	start(r, FDT_MODE_B2, t);
	stream_msg(r, STATE, 48, V(t - 1) + 100);
	stream_msg(r, IMU, 48, V(t - 1) + 100);
	CHECK(out_msg(r, 5, 0, 2, 100, V(t - 1) + 200) == 0);
	CHECK(!strcmp(sent[0].topic, "boat/b2/tx2/ef/state") && sent[0].qos == 2);
	CHECK(!strcmp(sent[1].topic, "boat/b2/tx2/af41/imu") && !strcmp(sent[2].topic, "boat/b2/tx2/be/video"));
	fdt_reg_free(r);
	/* b3: strict priority by order (state, imu, video, gps), every unit, discard at step end */
	start(r, FDT_MODE_B3, t);
	stream_msg(r, GPS, 80, V(t - 1) + 100);
	whole_output(r, 1, 2, 100, V(t - 1) + 200);
	stream_msg(r, IMU, 48, V(t - 1) + 300);
	stream_msg(r, STATE, 48, V(t - 1) + 400);
	run(r, S(t), V(t) + 2000);
	int n = tx_of(0, NULL, idx, 256);
	CHECK(n == 5 && strstr(sent[idx[0]].topic, "state") && strstr(sent[idx[1]].topic, "imu") &&
	      strstr(sent[idx[2]].topic, "video") && strstr(sent[idx[4]].topic, "gps"));
	fdt_reg_free(r);
	/* b3 with the table of the AS of mode b3: the order sigma_t of "o" (state, gps, video, imu), every
	 * unit (p and k do not police) */
	start(r, FDT_MODE_B3, t);
	stream_msg(r, GPS, 80, V(t - 1) + 100);
	whole_output(r, 1, 2, 100, V(t - 1) + 200);
	stream_msg(r, IMU, 48, V(t - 1) + 300);
	stream_msg(r, STATE, 48, V(t - 1) + 400);
	fdt_reg_tick(r, S(t));
	{
		char tb[160];
		int tl = snprintf(tb, sizeof tb, "{\"t\":%" PRId64 ",\"p\":[2,1,0,1],\"k\":[1,1,0,0],\"o\":[0,2,3,1]}", t);
		CHECK(fdt_reg_on_levels(r, tb, (size_t)tl, S(t) + 25000) == 0);
	}
	run(r, V(t), V(t) + 2000);
	n = tx_of(0, NULL, idx, 256);
	CHECK(n == 5 && strstr(sent[idx[0]].topic, "state") && strstr(sent[idx[1]].topic, "gps") &&
	      strstr(sent[idx[2]].topic, "video") && strstr(sent[idx[4]].topic, "imu"));
	CHECK(fdt_reg_step(r, t)->have_tab && fdt_reg_step(r, t)->tp[VIDEO].xrx == 200 &&
	      fdt_reg_step(r, t)->tp[GPS].xrx == 80);
	fdt_reg_free(r);
	/* phase 1 offset: vessel 2 (position 1, i = 2) publishes its demand 2 (T1 - eps - 2 T_tick) / (M + 1) */
	start(r, FDT_MODE_FLEETDT, t);
	fdt_reg_set_p1_offset(r, -1);
	{
		int64_t off = 2 * (D.T1_us - D.eps_us - 2 * D.T_tick_us) / (D.M + 1);
		CHECK(r->p1_off_us == off);
		int64_t nx = fdt_reg_tick(r, S(t) + off - 1);
		CHECK(nx == S(t) + off && fdt_reg_step(r, t) == NULL);
		fdt_reg_tick(r, S(t) + off);
		CHECK(fdt_reg_step(r, t) && fdt_reg_step(r, t)->have_dem && fdt_reg_step(r, t)->late_p1 == 0);
	}
	/* overrides: guard 0 (i T1 / (M + 1)), spacing 2500 (i 2500) */
	fdt_reg_set_p1_offset_guard(r, -1, 0);
	CHECK(r->p1_off_us == 2 * D.T1_us / (D.M + 1));
	fdt_reg_set_p1_offset_guard(r, 2500, -1);
	CHECK(r->p1_off_us == 5000);
	fdt_reg_free(r);
	char b[256];
	int len;
	/* tdma: release only in the own slot of W(t): vessel 2 is second of 2 */
	start(r, FDT_MODE_TDMA, t);
	stream_msg(r, STATE, 48, V(t - 1) + 400);
	int64_t w0 = fdt2_w_start_us(&D, t), tw = fdt2_w_end_us(&D, t) - w0;
	fdt_reg_tick(r, S(t));
	int64_t nx = fdt_reg_tick(r, V(t));
	CHECK(tx_of(0, NULL, idx, 256) == 0 && nx == w0 + tw / 2);
	fdt_reg_tick(r, w0 + tw / 2);
	CHECK(tx_of(0, NULL, idx, 256) == 1);
	fdt_reg_free(r);
	/* tdma with the slot of a table (AS of mode tdma); every unit leaves, p and k do not apply */
	start(r, FDT_MODE_TDMA, t);
	stream_msg(r, STATE, 48, V(t - 1) + 400);
	stream_msg(r, GPS, 80, V(t - 1) + 500);
	fdt_reg_tick(r, S(t));
	len = snprintf(b, sizeof b, "{\"t\":%" PRId64 ",\"p\":[2,0,0,0],\"k\":[1,0,0,0],"
	               "\"slot\":[%" PRId64 ",%" PRId64 "]}", t, V(t) + 10000, V(t) + 20000);
	CHECK(fdt_reg_on_levels(r, b, (size_t)len, S(t) + 25000) == 0);
	nx = fdt_reg_tick(r, V(t));
	CHECK(tx_of(0, NULL, idx, 256) == 0 && nx == V(t) + 10000);
	fdt_reg_tick(r, V(t) + 10000);
	CHECK(tx_of(0, NULL, idx, 256) == 2 && fdt_reg_step(r, t)->have_tab);
	fdt_reg_free(r);
	/* slotted: R at v(t), the others inside the slot of the table */
	start(r, FDT_MODE_SLOTTED, t);
	stream_msg(r, STATE, 48, V(t - 1) + 400);
	stream_msg(r, IMU, 48, V(t - 1) + 500);
	fdt_reg_tick(r, S(t));
	len = snprintf(b, sizeof b, "{\"t\":%" PRId64 ",\"p\":[2,1,0,0],\"k\":[1,1,0,0],\"o\":[0,1,3,2],"
	                   "\"slot\":[%" PRId64 ",%" PRId64 "]}", t, V(t) + 40000, V(t) + 50000);
	CHECK(fdt_reg_on_levels(r, b, (size_t)len, S(t) + 25000) == 0);
	nx = fdt_reg_tick(r, V(t));
	CHECK(tx_of(0, "state", idx, 256) == 1 && tx_of(0, "imu", idx, 256) == 0 && nx == V(t) + 40000);
	fdt_reg_tick(r, V(t) + 40000);
	CHECK(tx_of(0, "imu", idx, 256) == 1);
	fdt_reg_free(r);
	/* b5: no policing and no discard: unreleased messages of t continue in t+1 */
	start(r, FDT_MODE_B5, t);
	for (int j = 0; j < 4; j++)
		stream_msg(r, STATE, 48, V(t - 1) + 400 + j);   /* 4 > the demand of 2 */
	fdt_reg_tick(r, S(t));
	fake.n = 1;
	fake.s[0].nsd = 16384;
	run(r, S(t) + 1000, S(t + 1) + 1);
	CHECK(tx_of(0, NULL, idx, 256) == 0 && fdt_reg_step(r, t) != NULL);
	fake.s[0].nsd = 0;
	fdt_reg_tick(r, S(t + 1) + 1000);
	CHECK(tx_of(0, "state", idx, 256) == 4);
	fdt_reg_free(r);
	free(r);
}

static void test_held_fallback(void)
{
	/* without tcpi_bytes_acked: SIOCOUTQ counts the newest bytes of the stream */
	struct fdt_reg *r = calloc(1, sizeof *r);
	start(r, FDT_MODE_FLEETDT, 5000);
	fdt_reg_on_out(r, 0, 0, 1000, 990, 0, 0);
	fdt_reg_on_out(r, 0, 2, 100, 60, 0, 0);
	fdt_reg_on_out(r, 0, 0, 500, 480, 0, 0);
	struct fdt_sock sk;
	memset(&sk, 0, sizeof sk);
	sk.n = 1;
	sk.s[0].outq = 650;   /* 500 + 100 + the last 50 bytes of the first packet */
	struct fdt_b b;
	fdt_reg_held(r, &sk, &b);
	CHECK(b.n0 == 2 && b.x0 == 474 + 50 && b.n2 == 1 && b.x2 == 54 && b.other == 0);
	CHECK(r->trk[0].n == 3);
	sk.s[0].outq = 500;
	fdt_reg_held(r, &sk, &b);
	CHECK(b.n0 == 1 && b.x0 == 474 && b.n2 == 0 && r->trk[0].n == 1);
	fdt_reg_free(r);
	free(r);
}

static void test_resync(void)
{
	/* a clock step of one hour forward: no replay of 28800 steps, the regulator restarts at the current step */
	struct fdt_reg *r = calloc(1, sizeof *r);
	int64_t t = 6000;
	start(r, FDT_MODE_FLEETDT, t);
	stream_msg(r, STATE, 48, V(t - 1) + 100);
	int i0 = nsent;
	int64_t jump = S(t + 28800) + 50000;
	fdt_reg_tick(r, jump);
	CHECK(r->st_.resync == 1 && nsent == i0 && r->next_p1 == t + 28801 && r->next_v == t + 28801);
	fdt_reg_tick(r, S(t + 28801));
	CHECK(nsent == i0 + 1 && !strcmp(sent[i0].topic, "boat/b2/demand"));
	/* and backward */
	fdt_reg_tick(r, S(t));
	CHECK(r->st_.resync == 2 && r->next_p1 == t + 1);
	fdt_reg_free(r);
	free(r);
}

int main(void)
{
	test_dtp();
	test_assembly();
	test_demand_and_table();
	test_pacing_and_discard();
	test_close_at_step_end();
	test_modes();
	test_held_fallback();
	test_resync();
	if (failures) {
		fprintf(stderr, "test_reg: %d of %d checks failed\n", failures, checks);
		return 1;
	}
	printf("test_reg: %d checks passed\n", checks);
	return 0;
}
