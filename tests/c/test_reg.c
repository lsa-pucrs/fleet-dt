/* Unit tests of the DTP reader, the wire formats and the core of the vessel regulator. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fdt_common.h"
#include "fdt_reg.h"

static int failures;
#define CHECK(c) do { if (!(c)) { failures++; fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); } } while (0)

static const char *DTP =
	"{\"epoch\": 1000.0, \"Ts\": 0.125, \"T1\": 0.020, \"T2\": 0.010, \"alpha\": 0.125, \"U\": 0.8,"
	" \"C0\": 300000, \"c_b\": 1.06, \"c_m\": 180, \"R_PHY\": 24000000, \"h_W\": 150,"
	" \"topics\": [{\"name\": \"state\", \"rate\": 8, \"bound\": 48},"
	"              {\"name\": \"imu\", \"rate\": 32, \"bound\": 48},"
	"              {\"name\": \"video\", \"rate\": 30, \"bound\": 1000}],"
	" \"order\": [\"state\", \"video\", \"imu\"], \"needed\": [\"state\"]}";

struct sent { int l, qos; uint32_t t; uint16_t s; size_t len; };
static struct sent sent[4096];
static int nsent;

static int pub(void *ctx, int l, int qos, const uint8_t *p, size_t len)
{
	(void)ctx;
	sent[nsent].l = l;
	sent[nsent].qos = qos;
	sent[nsent].t = fdt_get_u32(p);
	sent[nsent].s = fdt_get_u16(p + 4);
	sent[nsent].len = len;
	nsent++;
	return 0;
}

static void test_dtp(const struct fdt_dtp *d)
{
	CHECK(d->L == 3);
	CHECK(d->Ts_us == 125000 && d->T1_us == 20000 && d->T2_us == 10000);
	CHECK(d->epoch_us == 1000000000LL);
	CHECK(d->n_order == 3 && d->order[0] == 0 && d->order[1] == 2 && d->order[2] == 1);
	CHECK(fdt_msgs_per_step(d, 0) == 1 && fdt_msgs_per_step(d, 1) == 4 && fdt_msgs_per_step(d, 2) == 4);
	/* d = ceil(1.06 (n s + 6 n) + 180 n) */
	CHECK(fdt_demand(d, 0) == (uint32_t)ceil(1.06 * 54 + 180));
	CHECK(fdt_demand(d, 1) == (uint32_t)ceil(1.06 * 4 * 54 + 720));
	CHECK(fdt_demand(d, 2) == (uint32_t)ceil(1.06 * 4 * 1006 + 720));
	/* steps */
	CHECK(fdt_step_at(d, fdt_start_us(d, 7)) == 7);
	CHECK(fdt_step_at(d, fdt_start_us(d, 7) - 1) == 6);
	CHECK(fdt_raw_step(d, fdt_start_us(d, 7) + 29999) == 7);
	CHECK(fdt_raw_step(d, fdt_start_us(d, 7) + 30000) == 8);
	CHECK(fdt_raw_step(d, fdt_start_us(d, 6) + 30000) == 7);
	struct fdt_dtp bad;
	char err[200];
	const char *over = "{\"Ts\":0.1,\"T1\":0.05,\"T2\":0.05,\"topics\":[{\"name\":\"a\",\"rate\":1,\"bound\":1}]}";
	CHECK(fdt_dtp_parse(over, strlen(over), &bad, err, sizeof err) == -1);
	const char *nul = "{\"epoch\":null,\"Ts\":0.125,\"T1\":0.02,\"T2\":0.01,\"topics\":[{\"name\":\"a\",\"rate\":1,\"bound\":1}]}";
	CHECK(fdt_dtp_parse(nul, strlen(nul), &bad, err, sizeof err) == 0 && bad.epoch == FDT_DEFAULT_EPOCH);
}

static void tables(struct fdt_reg *r, const struct fdt_dtp *d, uint32_t t, const uint32_t *dem, const uint8_t *p)
{
	uint8_t b[64];
	size_t n = fdt_demand_encode(b, t, d->L, dem);
	CHECK(fdt_reg_on_demand(r, b, n) == 0);
	if (p) {
		fdt_put_u32(b, t);
		b[4] = (uint8_t)d->L;
		memcpy(b + 5, p, (size_t)d->L);
		CHECK(fdt_reg_on_levels(r, b, 5 + (size_t)d->L) == 0);
	}
}

static int64_t run_until(struct fdt_reg *r, int64_t from, int64_t to, struct fdt_reg_log *last, int *nlogs)
{
	int64_t now = from, next;
	struct fdt_reg_log lg;
	while (now <= to) {
		if (fdt_reg_tick(r, now, NULL, NULL, &lg, &next)) {
			*last = lg;
			(*nlogs)++;
		}
		now = next > now ? next : now + 1;
	}
	return now;
}

static void test_core(const struct fdt_dtp *d)
{
	struct fdt_reg r;
	int64_t s5 = fdt_start_us(d, 5), p3 = 30000;
	CHECK(fdt_reg_init(&r, d, 64, s5 - 50000, pub, NULL) == 0);
	CHECK(r.next == 5);
	uint8_t content[1000];
	memset(content, 0xab, sizeof content);
	/* step 5 owns [start(4) + 30 ms, start(5) + 30 ms) */
	fdt_reg_on_raw(&r, 0, content, 48, s5 - 40000);            /* state */
	for (int k = 0; k < 6; k++)
		fdt_reg_on_raw(&r, 1, content, 48, s5 - 30000 + k);     /* imu: 6 > n = 4 */
	for (int k = 0; k < 4; k++)
		fdt_reg_on_raw(&r, 2, content, 1000, s5 + 1000 + k);     /* video */
	fdt_reg_on_raw(&r, 0, content, 48, s5 + p3);                /* belongs to step 6 */
	uint32_t dem[3] = {fdt_demand(d, 0), fdt_demand(d, 1), fdt_demand(d, 2)};
	uint8_t p[3] = {2, 0, 1};
	tables(&r, d, 5, dem, p);
	struct fdt_reg_log lg;
	int nlogs = 0;
	nsent = 0;
	run_until(&r, s5 + p3 - 1, s5 + p3, &lg, &nlogs);
	/* level 2 first (state, QoS 2), then level 1 (video, QoS 1); imu withheld */
	CHECK(nsent == 5);
	CHECK(sent[0].l == 0 && sent[0].qos == 2 && sent[0].t == 5 && sent[0].s == 0 && sent[0].len == 54);
	for (int k = 1; k < 5; k++)
		CHECK(sent[k].l == 2 && sent[k].qos == 1 && sent[k].s == k - 1 && sent[k].len == 1006);
	/* log of step 5 at start(6) + 1 ms */
	run_until(&r, s5 + p3 + 1, fdt_start_us(d, 6) + 1000, &lg, &nlogs);
	CHECK(nlogs == 1 && lg.t == 5 && lg.have_d && lg.have_p);
	CHECK(lg.rx[0] == 1 && lg.rx[1] == 6 && lg.rx[2] == 4);
	CHECK(lg.fwd[0] == 1 && lg.fwd[1] == 0 && lg.fwd[2] == 4);
	CHECK(lg.held[0] == 0 && lg.held[1] == 6 && lg.held[2] == 0);
	CHECK(lg.xfwd[2] == 4 * 1006);
	char js[2048];
	fdt_reg_log_json(&r, &lg, js, sizeof js);
	CHECK(strstr(js, "\"fwd\":[1,0,4]") != NULL);
	CHECK(strstr(js, "\"held\":[0,6,0]") != NULL);

	/* step 6: policing. Video demand cut to 2.5 messages: 2 forwarded, the rest withheld;
	 * imu at level 2 with 6 messages and demand of 4: 4 forwarded */
	for (int k = 0; k < 6; k++)
		fdt_reg_on_raw(&r, 1, content, 48, s5 + p3 + 10 + k);
	for (int k = 0; k < 5; k++)
		fdt_reg_on_raw(&r, 2, content, 1000, s5 + p3 + 100 + k);
	uint32_t dem6[3] = {dem[0], dem[1], (uint32_t)(2.5 * (1.06 * 1006 + 180))};
	uint8_t p6[3] = {2, 2, 1};
	tables(&r, d, 6, dem6, p6);
	nsent = 0;
	nlogs = 0;
	int64_t s6 = fdt_start_us(d, 6);
	run_until(&r, s6 + 1001, s6 + p3 + 1, &lg, &nlogs);
	CHECK(nsent == 1 + 4 + 2);
	CHECK(sent[0].l == 0 && sent[1].l == 1 && sent[4].l == 1 && sent[5].l == 2 && sent[6].l == 2);
	run_until(&r, s6 + p3 + 2, fdt_start_us(d, 7) + 1000, &lg, &nlogs);
	CHECK(lg.t == 6 && lg.fwd[1] == 4 && lg.held[1] == 2 && lg.fwd[2] == 2 && lg.held[2] == 3);
	CHECK(lg.cost[1] <= dem6[1] && lg.cost[2] <= dem6[2]);

	/* step 7: no priority table, so nothing leaves */
	fdt_reg_on_raw(&r, 0, content, 48, s6 + p3 + 5000);
	tables(&r, d, 7, dem, NULL);
	nsent = 0;
	int64_t s7 = fdt_start_us(d, 7);
	run_until(&r, s7 + 1001, fdt_start_us(d, 8) + 1000, &lg, &nlogs);
	CHECK(nsent == 0 && lg.t == 7 && !lg.have_p && lg.held[0] == 1);

	/* a tick 3 steps late: steps 8..10 are stale, step 11 runs */
	int64_t s11 = fdt_start_us(d, 11);
	fdt_reg_on_raw(&r, 0, content, 48, fdt_start_us(d, 8) - 1000);   /* step 8 */
	tables(&r, d, 11, dem, p);
	nsent = 0;
	struct fdt_reg_log lg2;
	int64_t next;
	fdt_reg_tick(&r, s11 + p3 + 500, NULL, NULL, &lg2, &next);
	CHECK(r.next == 12 && r.st.stale >= 1 && nsent == 0);
	fdt_reg_free(&r);
}

int main(void)
{
	struct fdt_dtp d;
	char err[256];
	if (fdt_dtp_parse(DTP, strlen(DTP), &d, err, sizeof err)) {
		fprintf(stderr, "%s\n", err);
		return 1;
	}
	test_dtp(&d);
	test_core(&d);
	if (failures) {
		fprintf(stderr, "%d failures\n", failures);
		return 1;
	}
	printf("test_reg: ok\n");
	return 0;
}
